/*
 * Driver test rig: a stand-in for LinuxCNC around linuxcnc/tiggy.c
 *
 * Built together with the driver and the HAL stand-in (shim/) into a shared
 * library that tools/linuxcnc_driver_test.py loads.  It does what LinuxCNC
 * does to a driver:
 *
 *   - a servo thread calls tiggy.read and tiggy.write every period, on a
 *     schedule that catches up after a delay, as LinuxCNC's does;
 *   - each joint has a small trajectory planner (the one LinuxCNC uses for
 *     jogging and homing): "go to this position at this speed and
 *     acceleration";
 *   - while the machine is off the commanded position follows the feedback.
 *
 * The test can also stall the servo thread, to play a PC that is busy, and
 * tell the driver a period that differs from the real one, to play clocks
 * that disagree.
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hal.h"
#include "rtapi.h"

#ifdef _WIN32
#include <winsock2.h>
#include <windows.h>
#include <process.h>
#define EXPORT __declspec(dllexport)
#else
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#define EXPORT __attribute__((visibility("default")))
#endif

int  rtapi_app_main(void);
void rtapi_app_exit(void);
char **shim_mp_ip(void);
char **shim_mp_io_ip(void);
int   *shim_mp_joints(void);

#define AXES 6

typedef struct {
    double pos, vel;                /* commanded now */
    double target, vmax, amax;
} axis_t;

static axis_t   s_axis[AXES];
static int      s_joints;
static volatile int s_machine_on;
static volatile int s_run;
static volatile int s_stall_ms;
static volatile int s_start_delay_ms;
static volatile long s_told_ns = 1000000, s_real_ns = 1000000;
static volatile uint64_t s_cycles;
static volatile double s_worst_error[AXES];     /* largest |command - feedback| seen */

static void (*s_read)(void *, long), (*s_write)(void *, long);
static void *s_read_arg, *s_write_arg;
static char  s_names[AXES][2][40];

/* ---- clock ---------------------------------------------------------- */

static uint64_t now_ns(void)
{
#ifdef _WIN32
    static LARGE_INTEGER f;
    LARGE_INTEGER c;
    if (!f.QuadPart) QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    return (uint64_t)(c.QuadPart / f.QuadPart) * 1000000000u +
           (uint64_t)(((c.QuadPart % f.QuadPart) * 1000000000LL) / f.QuadPart);
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000u + (uint64_t)ts.tv_nsec;
#endif
}

static void sleep_until(uint64_t t_ns)
{
#ifdef _WIN32
    for (;;) {
        uint64_t now = now_ns();
        if (now >= t_ns) return;
        uint64_t left = t_ns - now;
        if (left > 2000000u) Sleep((DWORD)(left / 1000000u - 1));
        else SwitchToThread();
    }
#else
    struct timespec ts;
    ts.tv_sec = (time_t)(t_ns / 1000000000u);
    ts.tv_nsec = (long)(t_ns % 1000000000u);
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) != 0) { }
#endif
}

/* ---- one servo period ------------------------------------------------ */

/* LinuxCNC's simple_tp: head for the target, never exceeding the speed and
 * acceleration, arriving with no overshoot. */
static void plan(axis_t *a, double dt)
{
    double max_dv = a->amax * dt;
    double tiny = max_dv * dt * 0.001;
    double err = a->target - a->pos, want;

    if (err > tiny) want = -max_dv + sqrt(2.0 * a->amax * err + max_dv * max_dv);
    else if (err < -tiny) want = max_dv - sqrt(-2.0 * a->amax * err + max_dv * max_dv);
    else { want = 0.0; a->pos = a->target; }

    if (want > a->vmax) want = a->vmax;
    if (want < -a->vmax) want = -a->vmax;
    if (want > a->vel + max_dv) a->vel += max_dv;
    else if (want < a->vel - max_dv) a->vel -= max_dv;
    else a->vel = want;
    a->pos += a->vel * dt;
}

static void servo_period(void)
{
    long told = s_told_ns;
    double dt = (double)told * 1e-9;

    s_read(s_read_arg, told);

    for (int j = 0; j < s_joints; j++) {
        axis_t *a = &s_axis[j];
        double fb = 0.0;
        shim_get(s_names[j][1], &fb);
        if (!s_machine_on) {
            /* Machine off: the command follows where the machine is. */
            a->pos = a->target = fb;
            a->vel = 0.0;
        } else {
            if (a->amax > 0.0) plan(a, dt);
            double e = fabs(a->pos - fb);
            if (e > s_worst_error[j]) s_worst_error[j] = e;
        }
        shim_set(s_names[j][0], a->pos);
    }
    shim_set("tiggy.enable", s_machine_on ? 1.0 : 0.0);

    s_write(s_write_arg, told);
    s_cycles++;
}

static void servo_loop(void)
{
    if (s_start_delay_ms > 0) sleep_until(now_ns() + (uint64_t)s_start_delay_ms * 1000000u);
    uint64_t next = now_ns();
    while (s_run) {
        int stall = s_stall_ms;
        if (stall > 0) {
            /* The PC is busy elsewhere: periods fall due and are made up
             * afterwards, one straight after another. */
            s_stall_ms = 0;
            sleep_until(now_ns() + (uint64_t)stall * 1000000u);
        }
        next += (uint64_t)s_real_ns;
        sleep_until(next);
        servo_period();
    }
}

#ifdef _WIN32
static HANDLE s_thread;
static unsigned __stdcall servo_entry(void *arg) { (void)arg; servo_loop(); return 0; }
#else
static pthread_t s_thread;
static void *servo_entry(void *arg) { (void)arg; servo_loop(); return NULL; }
#endif

/* ---- for the test --------------------------------------------------- */

/* servo_delay_ms: how long after the driver is loaded the servo thread
 * first runs.  In LinuxCNC that is when the rest of the HAL file has been
 * read, which can be a second or more. */
EXPORT int rig_start(const char *ip, int joints, const char *io_ip, int servo_delay_ms)
{
    s_start_delay_ms = servo_delay_ms;
#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
    timeBeginPeriod(1);
#endif
    static char ip_copy[64], io_copy[64];
    snprintf(ip_copy, sizeof(ip_copy), "%s", ip ? ip : "auto");
    snprintf(io_copy, sizeof(io_copy), "%s", io_ip ? io_ip : "");
    *shim_mp_ip() = ip_copy;
    *shim_mp_io_ip() = io_copy;
    *shim_mp_joints() = joints;
    s_joints = joints;

    int r = rtapi_app_main();
    if (r < 0) return r;
    if (!shim_funct("tiggy.read", &s_read, &s_read_arg) || !shim_funct("tiggy.write", &s_write, &s_write_arg))
        return -100;
    memset(s_axis, 0, sizeof(s_axis));
    for (int j = 0; j < AXES; j++) {
        snprintf(s_names[j][0], sizeof(s_names[j][0]), "tiggy.%d.position-cmd", j);
        snprintf(s_names[j][1], sizeof(s_names[j][1]), "tiggy.%d.position-fb", j);
        s_worst_error[j] = 0.0;
    }
    s_machine_on = 0;
    s_stall_ms = 0;
    s_run = 1;
#ifdef _WIN32
    s_thread = (HANDLE)_beginthreadex(NULL, 0, servo_entry, NULL, 0, NULL);
    return s_thread ? 0 : -101;
#else
    return pthread_create(&s_thread, NULL, servo_entry, NULL) == 0 ? 0 : -101;
#endif
}

EXPORT void rig_stop(void)
{
    if (s_run) {
        s_run = 0;
#ifdef _WIN32
        WaitForSingleObject(s_thread, INFINITE);
        CloseHandle(s_thread);
#else
        pthread_join(s_thread, NULL);
#endif
    }
    rtapi_app_exit();
}

/* A pin or parameter by name.  rig_get returns NaN for a name that does not exist. */
EXPORT double rig_get(const char *name)
{
    double v = 0.0;
    return shim_get(name, &v) ? v : NAN;
}

EXPORT int rig_set(const char *name, double value)
{
    return shim_set(name, value);
}

EXPORT void rig_machine(int on)
{
    s_machine_on = on ? 1 : 0;
}

/* Send a joint to a position (machine units), as a jog or a rapid would. */
EXPORT void rig_move(int joint, double target, double vmax, double amax)
{
    if (joint < 0 || joint >= AXES) return;
    s_axis[joint].vmax = vmax;
    s_axis[joint].amax = amax;
    s_axis[joint].target = target;
}

/* Shift a joint's commanded position at once: something LinuxCNC never
 * does, and the driver must refuse to follow. */
EXPORT void rig_jump(int joint, double by)
{
    if (joint < 0 || joint >= AXES) return;
    s_axis[joint].pos += by;
    s_axis[joint].target += by;
}

EXPORT double rig_command(int joint)
{
    return (joint >= 0 && joint < AXES) ? s_axis[joint].pos : 0.0;
}

/* True while any joint has not reached its target. */
EXPORT int rig_moving(void)
{
    for (int j = 0; j < s_joints; j++)
        if (s_axis[j].pos != s_axis[j].target || s_axis[j].vel != 0.0) return 1;
    return 0;
}

EXPORT void rig_stall(int ms)
{
    s_stall_ms = ms;
}

/* told_ns: the period the driver is given; real_ns: how often it is really called. */
EXPORT void rig_period(long told_ns, long real_ns)
{
    s_told_ns = told_ns;
    s_real_ns = real_ns;
}

EXPORT double rig_worst_error(int joint, int reset)
{
    if (joint < 0 || joint >= AXES) return 0.0;
    double v = s_worst_error[joint];
    if (reset) s_worst_error[joint] = 0.0;
    return v;
}

EXPORT double rig_cycles(void)
{
    return (double)s_cycles;
}

EXPORT int rig_messages(char *out, int out_size)
{
    return shim_messages(out, out_size);
}
