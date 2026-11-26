/*
 * Tiggy motion controller - LinuxCNC driver
 *
 * A HAL realtime component for LinuxCNC "uspace" (the standard package; a
 * realtime kernel is not needed).
 *
 *   loadrt tiggy ip=auto joints=3
 *   addf tiggy.read  servo-thread        (before the motion controller)
 *   addf tiggy.write servo-thread        (after it)
 *
 * How it works
 *
 *   LinuxCNC plans the motion.  Every servo period this component takes the
 *   position LinuxCNC commands for each joint, and a background thread sends
 *   the movement to the controller as a numbered stream of short timed
 *   segments.  The controller keeps a fraction of a second of them queued
 *   and plays them on its own clock, so neither the timing of this PC nor a
 *   late or lost network packet shows in the step pulses: a missing packet
 *   is simply sent again before its turn comes.
 *
 *   The price is that the machine runs that fraction of a second behind
 *   LinuxCNC (tiggy.buffer-target-ms, 200 ms unless changed).  The position
 *   fed back to LinuxCNC is the controller's real step count, taken together
 *   with the switch and probe inputs of the same report, so homing and
 *   probing latch the true position.  Because of the delay FERROR and
 *   MIN_FERROR in the INI file must allow for about half a second of travel
 *   at the joint's top speed.
 *
 *   tiggy.fault belongs in the E-stop chain (the fault-in of an estop_latch,
 *   whose ok-out goes to iocontrol.0.emc-enable-in): it is true while the
 *   controller cannot be reached, while its E-stop input is operated, and
 *   after the controller has stopped for a reason of its own (a limit
 *   switch, lost contact) until LinuxCNC has switched off.  tiggy.ready is
 *   the same thing the other way up.  tiggy.enable is "machine on"
 *   (motion.motion-enabled): while it is false the controller is held
 *   stopped with its drivers and outputs off.
 *
 * Build:  sudo halcompile --install tiggy.c
 *         (wifi_cnc_protocol.h in the same folder, or in ../protocol)
 */

#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"

#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <ifaddrs.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#if defined(__has_include)
#  if __has_include("wifi_cnc_protocol.h")
#    include "wifi_cnc_protocol.h"
#  else
#    include "../protocol/wifi_cnc_protocol.h"
#  endif
#else
#  include "wifi_cnc_protocol.h"
#endif

#ifdef __KERNEL__
#error "tiggy needs LinuxCNC uspace (the linuxcnc-uspace package), not a kernel-module build"
#endif

MODULE_AUTHOR("Michael Gaylor");
MODULE_DESCRIPTION("Driver for the Tiggy WiFi / Ethernet motion controller");
MODULE_LICENSE("Dual MIT/GPL");

static char *ip = "auto";
RTAPI_MP_STRING(ip, "address of the controller, or auto to find it on the network");
static int joints = 3;
RTAPI_MP_INT(joints, "number of joints (1 to 6)");
static char *io_ip = "";
RTAPI_MP_STRING(io_ip, "address of an I/O expansion module (auto to find it), empty for none");

/* ===================================================================
 * Constants
 * =================================================================== */

#define COMP                "tiggy"
#define MAX_JOINTS          WCNC_MAX_AXES
#define SWITCHES            6
#define AUX_INPUTS          4
#define AUX_OUTPUTS         5
#define IO_CHANNELS         16

#define MIN_FIRMWARE        WCNC_MAKE_VERSION(1, 2, 0)

#define SEG_TARGET_NS       4000000u    /* servo periods are grouped into segments about this long */
#define SAMPLE_RING         4096u       /* servo periods waiting for the link thread (power of two) */
#define SEG_RING            1024u       /* segments the controller has not yet accepted (power of two) */
#define SEND_WINDOW         48          /* most segments in flight at once */
#define REPEAT_SEGS         3u          /* segments already sent that go along with each new one */
#define RESEND_MS           40u         /* everything unacknowledged is sent again this often */

#define BUFFER_DEFAULT_MS   200u        /* motion kept queued between LinuxCNC and the motors */
#define BUFFER_MIN_MS       120u
#define BUFFER_MAX_MS       400u        /* the controller's queue holds 127 segments, about 500 ms */
#define TRIM_GAIN           0.2         /* clock matching: fraction of the buffer error removed per second */
#define TRIM_LIMIT          0.01        /* ...by playing at most this much faster or slower */
#define SETTLE_STEPS        4           /* standing still, LinuxCNC and the controller may differ by this
                                           much (it is then put right); more is a fault */

#define STATUS_INTERVAL_MS  10u         /* how often the controller is asked to report */
#define IO_PERIOD_MS        100u        /* output state heartbeat; the controller's watchdog is 1000 ms */
#define COMMAND_REPEAT_MS   100u
#define STOP_REPEAT_MS      20u         /* "stop" is repeated this often until the controller has stopped */
#define LINK_SILENT_NS      1000000000LL /* a link thread that says nothing for this long is not trusted */
#define FLUSH_REPEAT_MS     50u
#define RESET_TIMEOUT_MS    3000u
#define PING_PERIOD_MS      2000u
#define PONG_TIMEOUT_MS     8000u
#define STATUS_LOST_MS      1500u
#define FIRST_STATUS_MS     3000u
#define CONNECT_TIMEOUT_MS  1500
#define HANDSHAKE_MS        1500u
#define DISCOVER_MS         1200u
#define RETRY_PAUSE_MS      1500u
#define CONFIG_ACK_MS       800u
#define CONFIG_TRIES        4
#define MAX_SETTINGS        48

/* Memory ordering between the servo thread and the link threads.  (The test
 * build for other compilers supplies its own.) */
#ifndef tg_barrier
#define tg_barrier()        __sync_synchronize()
#endif

/* ===================================================================
 * Data shared between the servo thread and the link threads
 *
 * The servo thread must never wait, so nothing here is locked.  Each block
 * has one writer; a reader copies it and checks the sequence counter to see
 * that it was not being changed meanwhile.
 * =================================================================== */

typedef struct {                        /* what LinuxCNC wants: written by the servo thread */
    uint32_t epoch;                     /* counts the times "enable" has come on */
    uint8_t  valid;                     /* the servo thread has run: the settings below are the real ones */
    uint8_t  enable;
    uint8_t  spindle;                   /* 0 off, 1 forward, 2 reverse */
    uint16_t rpm;
    uint8_t  coolant;                   /* bit 0 flood, bit 1 mist */
    uint8_t  outputs;                   /* auxiliary outputs, bit per output */
    uint8_t  homing;                    /* a joint is being homed */
    uint16_t io_outputs;                /* I/O module outputs */
    /* settings */
    uint16_t pulse_us, dir_setup_us, idle_ms, debounce_ms, pwm_freq, pump_freq, buffer_ms;
    uint8_t  inv_step, inv_dir, inv_enable, switch_low, limit_mask, probe_high, estop_high;
    uint8_t  pwm_enable, min_duty;
    uint32_t max_rpm;
    uint32_t max_rate[MAX_JOINTS];      /* steps/s, 0 = not given */
    uint32_t accel[MAX_JOINTS];         /* steps/s^2, 0 = not given */
} tg_wanted_t;

typedef struct {                        /* what the controller reports: written by its link thread */
    uint8_t  online, ready, estop, probe;
    uint8_t  switches, inputs;
    uint16_t rpm;
    int64_t  position[MAX_JOINTS];      /* steps, counted the way LinuxCNC counts them */
    uint32_t underruns, resent, corrections, buffer_ms;
    uint32_t beat;                      /* changes with every report: the link thread is alive */
} tg_actual_t;

typedef struct {                        /* written by the I/O module's link thread */
    uint8_t  online;
    uint16_t inputs;
} tg_io_actual_t;

typedef struct {                        /* one servo period of commanded positions */
    int64_t  target[MAX_JOINTS];        /* 1/256 step */
    uint32_t ns;
    uint32_t epoch;
} tg_sample_t;

static struct { volatile uint32_t seq; tg_wanted_t v; }    sh_wanted;
static struct { volatile uint32_t seq; tg_actual_t v; }    sh_actual;
static struct { volatile uint32_t seq; tg_io_actual_t v; } sh_io;

static tg_sample_t       samples[SAMPLE_RING];
static volatile uint32_t smp_head;      /* written by the servo thread */
static volatile uint32_t smp_tail;      /* written by the link thread */
static volatile uint32_t smp_lost;      /* periods that found the ring full */

/* Messages for the operator.  LinuxCNC shows what a realtime component
 * prints as an error, through a channel that belongs to the servo thread;
 * so a link thread leaves its message here and the servo thread prints it. */
#define MSG_SLOTS           8u          /* power of two */
#define MSG_LEN             240
typedef struct {
    volatile uint32_t head;             /* written by the link thread */
    volatile uint32_t tail;             /* written by the servo thread */
    char     text[MSG_SLOTS][MSG_LEN];
} tg_messages_t;
static tg_messages_t sh_messages[2];    /* one for each link thread */

static void seq_write(volatile uint32_t *seq, void *dst, const void *src, size_t n)
{
    *seq = *seq + 1;                    /* odd: being written */
    tg_barrier();
    memcpy(dst, src, n);
    tg_barrier();
    *seq = *seq + 1;
    tg_barrier();
}

/* Returns 0 if no consistent copy could be had in a few tries (the writer
 * was interrupted half-way); the caller then keeps what it had. */
static int seq_read(volatile uint32_t *seq, const void *src, void *dst, size_t n, int tries)
{
    while (tries-- > 0) {
        uint32_t s = *seq;
        tg_barrier();
        if (s & 1u) continue;
        memcpy(dst, src, n);
        tg_barrier();
        if (*seq == s) return 1;
    }
    return 0;
}

/* ===================================================================
 * HAL pins and parameters
 * =================================================================== */

typedef struct {
    struct {
        hal_float_t *pos_cmd;           /* in:  commanded position */
        hal_float_t *pos_fb;            /* out: where the controller really is */
        hal_s32_t   *counts;            /* out: the same in steps */
        hal_bit_t   *homing;            /* in:  joint is being homed */
        hal_float_t scale;              /* steps per unit */
        hal_float_t maxvel;             /* units/s */
        hal_float_t maxaccel;           /* units/s^2 */
        hal_bit_t   step_invert;
        hal_bit_t   dir_invert;
    } joint[MAX_JOINTS];

    struct {
        hal_bit_t   *in, *in_not;
        hal_bit_t   invert;             /* TRUE: on when the pin is high */
        hal_bit_t   is_limit;           /* TRUE: the controller itself stops when it operates */
    } sw[SWITCHES];

    struct { hal_bit_t *in, *in_not; } aux_in[AUX_INPUTS];
    hal_bit_t   *aux_out[AUX_OUTPUTS];

    hal_bit_t   *enable, *ready, *fault, *connected, *probe, *estop;
    hal_bit_t   *spindle_fwd, *spindle_rev, *flood, *mist;
    hal_float_t *spindle_speed, *spindle_rps, *spindle_rpm;
    hal_float_t *buffer_ms;
    hal_u32_t   *underruns, *resent, *corrections;

    hal_u32_t   pulse_us, dir_setup_us, idle_release_ms, debounce_ms;
    hal_u32_t   pwm_freq, pump_freq, min_duty, buffer_target_ms;
    hal_float_t spindle_max_rpm;
    hal_bit_t   probe_invert, estop_invert, enable_invert, spindle_pwm;

    hal_bit_t   *io_connected;
    hal_bit_t   *io_in[IO_CHANNELS], *io_in_not[IO_CHANNELS], *io_out[IO_CHANNELS];
} tg_hal_t;

static int          comp_id = -1;
static tg_hal_t    *hal;
static int          have_io_module;

/* servo thread only */
static tg_actual_t    rt_actual;
static tg_io_actual_t rt_io;
static uint32_t       rt_epoch;
static int            rt_enabled;
static uint32_t       rt_beat;
static int64_t        rt_silent_ns;

/* ===================================================================
 * Small helpers
 * =================================================================== */

static volatile int stop_threads;

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000);
}

static uint32_t now_us32(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000000u + (uint64_t)(ts.tv_nsec / 1000));
}

static void nap_ms(unsigned ms)
{
    while (ms > 0 && !stop_threads) {
        unsigned part = ms > 50 ? 50 : ms;
        usleep(part * 1000);
        ms -= part;
    }
}

#ifdef __GNUC__
#define TG_PRINTF(f, a)     __attribute__((format(printf, f, a)))
#else
#define TG_PRINTF(f, a)
#endif

static void note(const char *fmt, ...) TG_PRINTF(1, 2);
static void fail(int io_module, const char *fmt, ...) TG_PRINTF(2, 3);

/* For the terminal and the log. */
static void note(const char *fmt, ...)
{
    char line[240];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    rtapi_print(COMP ": %s\n", line);
}

/* For the operator: LinuxCNC shows these as an error on screen.  Called by
 * a link thread (io_module tells which); printed by the servo thread. */
static void fail(int io_module, const char *fmt, ...)
{
    tg_messages_t *q = &sh_messages[io_module ? 1 : 0];
    uint32_t head = q->head;
    if ((uint32_t)(head - q->tail) >= MSG_SLOTS) return;       /* plenty on screen already */
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(q->text[head & (MSG_SLOTS - 1)], MSG_LEN, fmt, ap);
    va_end(ap);
    tg_barrier();
    q->head = head + 1;
}

static uint32_t to_u32(double v)
{
    if (!(v > 0.0)) return 0;
    if (v > 4.0e9) return 4000000000u;
    return (uint32_t)(v + 0.5);
}

static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ===================================================================
 * Servo thread
 * =================================================================== */

/* Controller -> LinuxCNC.  Runs before the motion controller. */
static void tg_read(void *arg, long period)
{
    tg_hal_t *h = (tg_hal_t *)arg;
    tg_actual_t a;

    if (seq_read(&sh_actual.seq, &sh_actual.v, &a, sizeof(a), 4)) rt_actual = a;
    else a = rt_actual;

    /* The link thread reports about every millisecond while it has a
     * controller.  If it has gone quiet, what it last said no longer holds. */
    if (a.beat != rt_beat) {
        rt_beat = a.beat;
        rt_silent_ns = 0;
    } else if (rt_silent_ns < LINK_SILENT_NS) {
        rt_silent_ns += period > 0 ? period : 1000000;
    }
    if (rt_silent_ns >= LINK_SILENT_NS) {
        a.ready = 0;
        a.online = 0;
    }

    for (int i = 0; i < 2; i++) {
        tg_messages_t *q = &sh_messages[i];
        uint32_t head = q->head;
        tg_barrier();
        while (q->tail != head) {
            rtapi_print_msg(RTAPI_MSG_ERR, COMP ": %s\n", q->text[q->tail & (MSG_SLOTS - 1)]);
            tg_barrier();
            q->tail = q->tail + 1;
        }
    }

    for (int j = 0; j < joints; j++) {
        double scale = h->joint[j].scale;
        if (scale == 0.0) scale = 1.0;
        *h->joint[j].pos_fb = (double)a.position[j] / scale;
        *h->joint[j].counts = (int32_t)a.position[j];
    }
    for (int i = 0; i < SWITCHES; i++) {
        int on = (a.switches >> i) & 1;
        *h->sw[i].in = on;
        *h->sw[i].in_not = !on;
    }
    for (int i = 0; i < AUX_INPUTS; i++) {
        int on = (a.inputs >> i) & 1;
        *h->aux_in[i].in = on;
        *h->aux_in[i].in_not = !on;
    }
    *h->connected = a.online;
    *h->ready = a.ready;
    *h->fault = !a.ready;
    *h->probe = a.probe;
    *h->estop = a.estop;
    *h->spindle_rpm = (double)a.rpm;
    *h->spindle_rps = (double)a.rpm / 60.0;
    *h->buffer_ms = (double)a.buffer_ms;
    *h->underruns = a.underruns;
    *h->resent = a.resent;
    *h->corrections = a.corrections;

    if (have_io_module) {
        tg_io_actual_t io;
        if (seq_read(&sh_io.seq, &sh_io.v, &io, sizeof(io), 4)) rt_io = io;
        else io = rt_io;
        *h->io_connected = io.online;
        for (int i = 0; i < IO_CHANNELS; i++) {
            int on = (io.inputs >> i) & 1;
            *h->io_in[i] = on;
            *h->io_in_not[i] = !on;
        }
    }
}

/* LinuxCNC -> controller.  Runs after the motion controller. */
static void tg_write(void *arg, long period)
{
    tg_hal_t *h = (tg_hal_t *)arg;
    tg_wanted_t w;
    tg_sample_t s;

    memset(&w, 0, sizeof(w));
    memset(&s, 0, sizeof(s));

    w.valid = 1;
    w.enable = *h->enable ? 1 : 0;
    if (w.enable && !rt_enabled) rt_epoch++;
    rt_enabled = w.enable;
    w.epoch = rt_epoch;

    if (*h->spindle_fwd) w.spindle = 1;
    else if (*h->spindle_rev) w.spindle = 2;
    {
        double rpm = fabs(*h->spindle_speed);
        if (!(rpm >= 0.0)) rpm = 0.0;               /* not a number */
        w.rpm = (uint16_t)(rpm > 65535.0 ? 65535.0 : rpm + 0.5);
    }
    if (*h->flood) w.coolant |= 1;
    if (*h->mist) w.coolant |= 2;
    for (int i = 0; i < AUX_OUTPUTS; i++)
        if (*h->aux_out[i]) w.outputs |= (uint8_t)(1u << i);
    if (have_io_module) {
        for (int i = 0; i < IO_CHANNELS; i++)
            if (*h->io_out[i]) w.io_outputs |= (uint16_t)(1u << i);
    }

    w.pulse_us = (uint16_t)clamp_u32(h->pulse_us, 1, 50);
    w.dir_setup_us = (uint16_t)clamp_u32(h->dir_setup_us, 0, 50);
    w.idle_ms = (uint16_t)clamp_u32(h->idle_release_ms, 0, 65535);
    w.debounce_ms = (uint16_t)clamp_u32(h->debounce_ms, 1, 100);
    w.pwm_freq = (uint16_t)clamp_u32(h->pwm_freq, 1, 65535);
    w.pump_freq = (uint16_t)clamp_u32(h->pump_freq, 0, 65535);
    w.buffer_ms = (uint16_t)clamp_u32(h->buffer_target_ms, BUFFER_MIN_MS, BUFFER_MAX_MS);
    w.min_duty = (uint8_t)clamp_u32(h->min_duty, 0, 100);
    w.max_rpm = to_u32(h->spindle_max_rpm);
    if (w.max_rpm < 1) w.max_rpm = 1;
    w.pwm_enable = h->spindle_pwm ? 1 : 0;
    w.probe_high = h->probe_invert ? 1 : 0;
    w.estop_high = h->estop_invert ? 1 : 0;
    w.inv_enable = h->enable_invert ? 1 : 0;
    for (int i = 0; i < SWITCHES; i++) {
        if (!h->sw[i].invert) w.switch_low |= (uint8_t)(1u << i);
        if (h->sw[i].is_limit) w.limit_mask |= (uint8_t)(1u << i);
    }

    for (int j = 0; j < joints; j++) {
        double scale = h->joint[j].scale;
        if (scale == 0.0) scale = 1.0;
        if (h->joint[j].step_invert) w.inv_step |= (uint8_t)(1u << j);
        if (h->joint[j].dir_invert) w.inv_dir |= (uint8_t)(1u << j);
        if (*h->joint[j].homing) w.homing = 1;
        w.max_rate[j] = to_u32(fabs(h->joint[j].maxvel * scale));
        w.accel[j] = to_u32(fabs(h->joint[j].maxaccel * scale));

        double sub = *h->joint[j].pos_cmd * scale * (double)WCNC_SUBSTEP_SCALE;
        if (!(fabs(sub) < 9.0e18)) sub = 0.0;       /* also catches a NaN */
        s.target[j] = (int64_t)llround(sub);
    }

    seq_write(&sh_wanted.seq, &sh_wanted.v, &w, sizeof(w));

    if (w.enable) {
        uint32_t head = smp_head;
        if ((uint32_t)(head - smp_tail) >= SAMPLE_RING) {
            smp_lost = smp_lost + 1;
        } else {
            s.ns = period > 0 ? (uint32_t)period : 1000000u;
            s.epoch = rt_epoch;
            samples[head & (SAMPLE_RING - 1)] = s;
            tg_barrier();
            smp_head = head + 1;
        }
    }
}

/* ===================================================================
 * Link: one connection to a controller or to an I/O module
 * =================================================================== */

typedef struct {
    uint16_t key;
    uint8_t  type;
    uint8_t  valid;
    uint32_t value;
} tg_setting_t;

typedef struct {
    int         io_module;              /* this link is to an I/O expansion module */
    const char *what;                   /* for messages */
    char        fixed_ip[64];           /* address given by the user, empty = search */
    char        ip[64];                 /* address in use or last used */
    pthread_t   thread;
    int         started;

    int         udp, tcp;
    struct sockaddr_in peer;
    uint32_t    tx_seq;
    uint8_t     rx[2048];
    int         rx_len;

    int         online;
    wcnc_handshake_resp_t hs;
    int         hs_fresh, hs_wanted;
    uint64_t    hs_ms;

    wcnc_status_report_t st;
    int         have_status;
    uint64_t    status_ms;
    int32_t     raw_position[MAX_JOINTS];
    int         position_valid;
    int64_t     position[MAX_JOINTS];   /* steps as LinuxCNC counts them; survives a reconnection */

    uint64_t    online_ms, ping_ms, pong_ms, io_ms, command_ms;
    uint8_t     io_last[8];
    int         io_sent;

    tg_setting_t sent[MAX_SETTINGS];    /* settings the controller has acknowledged */
    int         n_sent;
    int         cfg_inflight, cfg_tries, cfg_is_save, cfg_dirty, cfg_done;
    int         cfg_once;               /* the settings have all arrived at least once this session */
    uint32_t    beat;
    uint16_t    cfg_key;
    uint32_t    cfg_value, cfg_seq;
    uint8_t     cfg_type;
    uint64_t    cfg_ms;
    int         rates_checked;

    int         rounds_missing;         /* searches that found nothing */
    int         said_missing, said_problem;
    char        loss[120];
} link_t;

static link_t link_main, link_io;

static void finalize(link_t *L, void *pkt, uint8_t type, size_t payload)
{
    wcnc_finalize_packet(pkt, type, (uint16_t)payload, L->tx_seq++, now_us32());
}

static int udp_send(link_t *L, const void *pkt, size_t len)
{
    if (L->udp < 0) return 0;
    return sendto(L->udp, (const char *)pkt, (int)len, 0,
                  (const struct sockaddr *)&L->peer, sizeof(L->peer)) == (int)len;
}

static void udp_send_simple(link_t *L, uint8_t type, int repeat)
{
    wcnc_control_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    finalize(L, &pkt, type, 0);
    while (repeat-- > 0) udp_send(L, &pkt, sizeof(pkt));
}

static int set_nonblocking(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) return 0;
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

static int open_udp(link_t *L)
{
    if (L->udp >= 0) return 1;
    int fd = (int)socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return 0;
    struct sockaddr_in any;
    memset(&any, 0, sizeof(any));
    any.sin_family = AF_INET;           /* any address, any port: replies come back to it */
    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_BROADCAST, (const char *)&yes, sizeof(yes));
    if (bind(fd, (struct sockaddr *)&any, sizeof(any)) != 0 || !set_nonblocking(fd)) {
        close(fd);
        return 0;
    }
    L->udp = fd;
    return 1;
}

static void close_tcp(link_t *L)
{
    if (L->tcp >= 0) close(L->tcp);
    L->tcp = -1;
}

/* A frame on the control connection is a packet preceded by its length
 * (2 bytes, low byte first). */
static int tcp_send_frame(link_t *L, const void *pkt, size_t len)
{
    uint8_t buf[2 + 512];
    if (L->tcp < 0 || len > 512) return 0;
    buf[0] = (uint8_t)(len & 0xFF);
    buf[1] = (uint8_t)(len >> 8);
    memcpy(buf + 2, pkt, len);
    size_t total = len + 2, done = 0;
    uint64_t start = now_ms();
    while (done < total) {
        int n = (int)send(L->tcp, (const char *)buf + done, (int)(total - done), MSG_NOSIGNAL);
        if (n > 0) { done += (size_t)n; continue; }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) && now_ms() - start < 500) {
            struct pollfd p;
            p.fd = L->tcp; p.events = POLLOUT; p.revents = 0;
            poll(&p, 1, 10);
            continue;
        }
        return 0;
    }
    return 1;
}

static void go_offline(link_t *L, const char *why)
{
    close_tcp(L);
    snprintf(L->loss, sizeof(L->loss), "%s", why);
}

/* ---- settings ------------------------------------------------------ */

static tg_setting_t *setting_find(link_t *L, uint16_t key)
{
    for (int i = 0; i < L->n_sent; i++)
        if (L->sent[i].key == key) return &L->sent[i];
    return NULL;
}

static int settings_list(const tg_wanted_t *w, tg_setting_t *it)
{
    int n = 0;
#define ADD(k, t, v) do { it[n].key = (uint16_t)(k); it[n].type = (uint8_t)(t); it[n].value = (uint32_t)(v); n++; } while (0)
    ADD(WCNC_CFG_STATUS_INTERVAL_MS, WCNC_VAL_UINT16, STATUS_INTERVAL_MS);
    ADD(WCNC_CFG_STEP_PULSE_US, WCNC_VAL_UINT16, w->pulse_us);
    ADD(WCNC_CFG_DIR_SETUP_US, WCNC_VAL_UINT16, w->dir_setup_us);
    ADD(WCNC_CFG_STEP_IDLE_DELAY_MS, WCNC_VAL_UINT16, w->idle_ms);
    ADD(WCNC_CFG_INPUT_DEBOUNCE_MS, WCNC_VAL_UINT16, w->debounce_ms);
    ADD(WCNC_CFG_INVERT_STEP, WCNC_VAL_UINT8, w->inv_step);
    ADD(WCNC_CFG_INVERT_DIR, WCNC_VAL_UINT8, w->inv_dir);
    ADD(WCNC_CFG_INVERT_ENABLE, WCNC_VAL_UINT8, w->inv_enable);
    ADD(WCNC_CFG_INVERT_LIMIT, WCNC_VAL_UINT8, w->switch_low);
    ADD(WCNC_CFG_INVERT_HOME, WCNC_VAL_UINT8, w->switch_low);
    ADD(WCNC_CFG_INVERT_ESTOP, WCNC_VAL_UINT8, w->estop_high);
    ADD(WCNC_CFG_INVERT_PROBE, WCNC_VAL_UINT8, w->probe_high);
    ADD(WCNC_CFG_LIMIT_ENABLE_MASK, WCNC_VAL_UINT8, w->limit_mask);
    ADD(WCNC_CFG_SPINDLE_PWM_ENABLE, WCNC_VAL_UINT8, w->pwm_enable);
    ADD(WCNC_CFG_SPINDLE_PWM_FREQ, WCNC_VAL_UINT16, w->pwm_freq);
    ADD(WCNC_CFG_SPINDLE_MAX_RPM, WCNC_VAL_UINT32, w->max_rpm);
    ADD(WCNC_CFG_SPINDLE_MIN_DUTY, WCNC_VAL_UINT8, w->min_duty);
    ADD(WCNC_CFG_CHARGE_PUMP_FREQ, WCNC_VAL_UINT16, w->pump_freq);
    for (int j = 0; j < joints; j++) {
        /* The controller uses these for its own stops (a starved queue, a
         * flush); the moves themselves are LinuxCNC's. */
        if (w->max_rate[j]) ADD(WCNC_CFG_MAX_RATE_X + j, WCNC_VAL_UINT32, w->max_rate[j]);
        if (w->accel[j]) ADD(WCNC_CFG_ACCEL_X + j, WCNC_VAL_UINT32, w->accel[j]);
    }
#undef ADD
    return n;
}

static void send_handshake_request(link_t *L)
{
    wcnc_handshake_req_t req;
    memset(&req, 0, sizeof(req));
    req.host_version = WCNC_MAKE_VERSION(1, 2, 0);
    snprintf(req.host_name, sizeof(req.host_name), "LinuxCNC-Tiggy");
    finalize(L, &req, WCNC_PKT_HANDSHAKE_REQ, sizeof(req) - sizeof(wcnc_header_t));
    if (!tcp_send_frame(L, &req, sizeof(req))) go_offline(L, "cannot send to it");
}

/* Send whatever setting differs from what the controller has, one at a
 * time, each acknowledged before the next. */
static void pump_settings(link_t *L, const tg_wanted_t *w, uint64_t now)
{
    if (L->cfg_inflight) {
        if (now - L->cfg_ms < CONFIG_ACK_MS) return;
        L->cfg_inflight = 0;
        if (++L->cfg_tries >= CONFIG_TRIES) {
            L->cfg_tries = 0;
            go_offline(L, "it did not acknowledge a setting");
            return;
        }
    }

    tg_setting_t want[MAX_SETTINGS];
    int n = settings_list(w, want);
    for (int i = 0; i < n; i++) {
        const tg_setting_t *have = setting_find(L, want[i].key);
        if (have && have->valid && have->value == want[i].value) continue;

        wcnc_config_packet_t pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.key = want[i].key;
        pkt.value_type = want[i].type;
        memcpy(pkt.value, &want[i].value, 4);       /* little-endian, as the protocol is */
        L->cfg_key = want[i].key;
        L->cfg_type = want[i].type;
        L->cfg_value = want[i].value;
        L->cfg_is_save = 0;
        L->cfg_seq = L->tx_seq;
        L->cfg_inflight = 1;
        L->cfg_ms = now;
        L->cfg_done = 0;
        finalize(L, &pkt, WCNC_PKT_CONFIG_SET, sizeof(pkt) - sizeof(wcnc_header_t));
        if (!tcp_send_frame(L, &pkt, sizeof(pkt))) go_offline(L, "cannot send to it");
        return;
    }

    if (L->cfg_dirty) {
        /* Everything has arrived: have the controller keep it. */
        wcnc_control_packet_t pkt;
        memset(&pkt, 0, sizeof(pkt));
        L->cfg_is_save = 1;
        L->cfg_seq = L->tx_seq;
        L->cfg_inflight = 1;
        L->cfg_ms = now;
        finalize(L, &pkt, WCNC_PKT_CONFIG_SAVE, 0);
        if (!tcp_send_frame(L, &pkt, sizeof(pkt))) go_offline(L, "cannot send to it");
        return;
    }

    if (L->hs_wanted) {
        /* Its fastest step rate depends on the pulse timing it now has. */
        if (now - L->hs_ms >= 400) {
            L->hs_ms = now;
            send_handshake_request(L);
        }
        return;
    }

    L->cfg_done = 1;
    L->cfg_once = 1;
    if (!L->rates_checked) {
        L->rates_checked = 1;
        for (int j = 0; j < joints; j++) {
            if (w->max_rate[j] > L->hs.max_step_rate && L->hs.max_step_rate) {
                fail(0, "joint %d needs %u steps a second at its top speed, but with a %u us step pulse the "
                     "controller makes at most %u: lower MAX_VELOCITY or SCALE, or shorten the step pulse",
                     j, (unsigned)w->max_rate[j], (unsigned)w->pulse_us, (unsigned)L->hs.max_step_rate);
            }
        }
    }
}

static void on_config_ack(link_t *L, const wcnc_ack_packet_t *ack)
{
    if (!L->cfg_inflight) return;
    if (L->cfg_is_save) {
        if (ack->acked_type != WCNC_PKT_CONFIG_SAVE) return;
        L->cfg_dirty = 0;
    } else {
        if (ack->acked_type != WCNC_PKT_CONFIG_SET || ack->acked_sequence != L->cfg_seq) return;
        tg_setting_t *have = setting_find(L, L->cfg_key);
        if (!have && L->n_sent < MAX_SETTINGS) have = &L->sent[L->n_sent++];
        if (have) {
            have->key = L->cfg_key;
            have->type = L->cfg_type;
            have->value = L->cfg_value;
            have->valid = 1;
        }
        L->cfg_dirty = 1;
        L->hs_wanted = 1;
        L->rates_checked = 0;
    }
    L->cfg_inflight = 0;
    L->cfg_tries = 0;
}

/* ---- receiving ----------------------------------------------------- */

static void on_tcp_frame(link_t *L, uint8_t *frame, int len)
{
    if (!wcnc_validate_packet(frame, (size_t)len)) return;
    const wcnc_header_t *h = (const wcnc_header_t *)frame;
    L->pong_ms = now_ms();

    if (h->packet_type == WCNC_PKT_ACK && len >= (int)sizeof(wcnc_ack_packet_t)) {
        on_config_ack(L, (const wcnc_ack_packet_t *)frame);
    } else if (h->packet_type == WCNC_PKT_HANDSHAKE_RESP &&
               len >= (int)offsetof(wcnc_handshake_resp_t, device_mode)) {
        memset(&L->hs, 0, sizeof(L->hs));
        memcpy(&L->hs, frame, len < (int)sizeof(L->hs) ? (size_t)len : sizeof(L->hs));
        L->hs_fresh = 1;
        L->hs_wanted = 0;
    }
}

/* Returns 0 when the connection has ended. */
static int tcp_pump(link_t *L)
{
    for (;;) {
        int n = (int)recv(L->tcp, (char *)L->rx + L->rx_len, (int)sizeof(L->rx) - L->rx_len, 0);
        if (n == 0) return 0;
        if (n < 0) return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
        L->rx_len += n;
        while (L->rx_len >= 2) {
            int flen = L->rx[0] | (L->rx[1] << 8);
            if (flen < (int)sizeof(wcnc_header_t) || flen > 512) return 0;
            if (L->rx_len < 2 + flen) break;
            on_tcp_frame(L, L->rx + 2, flen);
            memmove(L->rx, L->rx + 2 + flen, (size_t)(L->rx_len - 2 - flen));
            L->rx_len -= 2 + flen;
        }
        if (L->rx_len >= (int)sizeof(L->rx)) return 0;
    }
}

static void motion_on_status(link_t *L, uint64_t now);

static void on_udp(link_t *L, uint8_t *data, int len, const struct sockaddr_in *from)
{
    if (from->sin_addr.s_addr != L->peer.sin_addr.s_addr) return;       /* not ours */
    if (len < (int)sizeof(wcnc_header_t) || !wcnc_validate_packet(data, (size_t)len)) return;
    const wcnc_header_t *h = (const wcnc_header_t *)data;
    if (h->packet_type != WCNC_PKT_STATUS_REPORT) return;
    int payload = len - (int)sizeof(wcnc_header_t);
    if (payload < WCNC_STATUS_REPORT_V11_SIZE) return;

    wcnc_status_report_t st;
    memset(&st, 0, sizeof(st));
    memcpy(&st, data + sizeof(wcnc_header_t), payload < (int)sizeof(st) ? (size_t)payload : sizeof(st));
    if (L->have_status && L->st.uptime_ms < 0xF0000000u && st.uptime_ms + 2000u < L->st.uptime_ms) {
        /* Its clock has gone back: it has restarted.  Its step count starts
         * again from zero and must not be taken for movement. */
        go_offline(L, "it restarted");
        return;
    }
    L->st = st;
    L->have_status = 1;
    L->status_ms = now_ms();

    if (!L->io_module) {
        /* The controller's count becomes LinuxCNC's by following its
         * changes, so that a controller that was restarted (and counts from
         * zero again) carries on from where LinuxCNC knew the machine to be. */
        for (int j = 0; j < MAX_JOINTS; j++) {
            if (L->position_valid)
                L->position[j] += (int32_t)((uint32_t)L->st.position_steps[j] - (uint32_t)L->raw_position[j]);
            L->raw_position[j] = L->st.position_steps[j];
        }
        L->position_valid = 1;
        motion_on_status(L, L->status_ms);
    }
}

/* ---- finding and connecting ---------------------------------------- */

static int discover(link_t *L, char *out, size_t out_len)
{
    wcnc_control_packet_t req;
    memset(&req, 0, sizeof(req));
    finalize(L, &req, WCNC_PKT_DISCOVER, 0);

    /* Everybody on every local network, the controller's own access point,
     * this computer (a simulator) and the address used last time. */
    uint32_t targets[32];
    int n_targets = 0;
    struct in_addr a;
    targets[n_targets++] = htonl(INADDR_BROADCAST);
    targets[n_targets++] = htonl(INADDR_LOOPBACK);
    if (inet_pton(AF_INET, "192.168.4.1", &a) == 1) targets[n_targets++] = a.s_addr;
    if (L->ip[0] && inet_pton(AF_INET, L->ip, &a) == 1) targets[n_targets++] = a.s_addr;
    {
        struct ifaddrs *list = NULL;
        if (getifaddrs(&list) == 0) {
            for (struct ifaddrs *i = list; i; i = i->ifa_next) {
                if (!i->ifa_addr || i->ifa_addr->sa_family != AF_INET) continue;
                if (!(i->ifa_flags & IFF_UP) || !(i->ifa_flags & IFF_BROADCAST) || !i->ifa_broadaddr) continue;
                if (n_targets < 32)
                    targets[n_targets++] = ((struct sockaddr_in *)i->ifa_broadaddr)->sin_addr.s_addr;
            }
            freeifaddrs(list);
        }
    }

    uint64_t start = now_ms(), last_send = 0;
    char first[64] = "";
    while (!stop_threads && now_ms() - start < DISCOVER_MS) {
        uint64_t now = now_ms();
        if (last_send == 0 || now - last_send >= 400) {
            last_send = now;
            for (int i = 0; i < n_targets; i++) {
                struct sockaddr_in to;
                memset(&to, 0, sizeof(to));
                to.sin_family = AF_INET;
                to.sin_port = htons(WCNC_UDP_MOTION_PORT);
                to.sin_addr.s_addr = targets[i];
                sendto(L->udp, (const char *)&req, (int)sizeof(req), 0, (struct sockaddr *)&to, sizeof(to));
            }
        }
        struct pollfd p;
        p.fd = L->udp; p.events = POLLIN; p.revents = 0;
        if (poll(&p, 1, 50) <= 0) continue;
        for (;;) {
            uint8_t buf[512];
            struct sockaddr_in from;
            socklen_t fl = sizeof(from);
            int n = (int)recvfrom(L->udp, (char *)buf, (int)sizeof(buf), 0, (struct sockaddr *)&from, &fl);
            if (n <= 0) break;
            if (n < (int)sizeof(wcnc_discover_resp_t) || !wcnc_validate_packet(buf, (size_t)n)) continue;
            const wcnc_discover_resp_t *r = (const wcnc_discover_resp_t *)buf;
            if (r->header.packet_type != WCNC_PKT_DISCOVER_RESP) continue;
            if ((r->device_mode == 1) != (L->io_module != 0)) continue;
            char found[64];
            if (!inet_ntop(AF_INET, &from.sin_addr, found, sizeof(found))) continue;
            if (L->ip[0] && strcmp(found, L->ip) == 0) {        /* the one already known */
                snprintf(out, out_len, "%s", found);
                return 1;
            }
            if (!first[0]) snprintf(first, sizeof(first), "%s", found);
        }
        /* Give the known address a moment to answer before taking another. */
        if (first[0] && (!L->ip[0] || now_ms() - start > 500)) break;
    }
    if (!first[0]) return 0;
    snprintf(out, out_len, "%s", first);
    return 1;
}

static int connect_to(link_t *L, const char *address)
{
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(WCNC_TCP_CONTROL_PORT);
    if (inet_pton(AF_INET, address, &addr.sin_addr) != 1) {
        if (!L->said_problem)
            fail(L->io_module, "\"%s\" is not an address (use numbers, e.g. 192.168.1.50, or auto)", address);
        L->said_problem = 1;
        return 0;
    }

    int fd = (int)socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return 0;
    int yes = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (const char *)&yes, sizeof(yes));
    if (!set_nonblocking(fd)) { close(fd); return 0; }

    int ok = 0;
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) == 0) {
        ok = 1;
    } else if (errno == EINPROGRESS || errno == EWOULDBLOCK || errno == EINTR) {
        for (int waited = 0; waited < CONNECT_TIMEOUT_MS && !stop_threads; waited += 50) {
            struct pollfd p;
            p.fd = fd; p.events = POLLOUT; p.revents = 0;
            if (poll(&p, 1, 50) > 0) {
                int err = 0;
                socklen_t el = sizeof(err);
                ok = (p.revents & POLLOUT) && getsockopt(fd, SOL_SOCKET, SO_ERROR, (char *)&err, &el) == 0 && err == 0;
                break;
            }
        }
    }
    if (!ok) { close(fd); return 0; }

    L->tcp = fd;
    L->rx_len = 0;
    L->hs_fresh = 0;
    send_handshake_request(L);
    for (uint64_t start = now_ms(); L->tcp >= 0 && !L->hs_fresh && !stop_threads && now_ms() - start < HANDSHAKE_MS; ) {
        struct pollfd p;
        p.fd = L->tcp; p.events = POLLIN; p.revents = 0;
        if (poll(&p, 1, 50) > 0 && !tcp_pump(L)) close_tcp(L);
    }
    if (L->tcp < 0 || !L->hs_fresh) {
        close_tcp(L);
        if (!L->said_problem)
            fail(L->io_module, "%s accepts a connection but does not answer as a Tiggy %s: is another program "
                 "(Mach3, a second LinuxCNC) using it?", address, L->what);
        L->said_problem = 1;
        return 0;
    }

    uint32_t v = L->hs.firmware_version;
    if (v < MIN_FIRMWARE || !(L->hs.capabilities & WCNC_CAP_STREAM_V2)) {
        close_tcp(L);
        if (!L->said_problem)
            fail(L->io_module, "the %s at %s has firmware %u.%u.%u: update it to 1.2.0 or newer", L->what, address,
                 (unsigned)(v >> 24) & 0xFF, (unsigned)(v >> 16) & 0xFF, (unsigned)v & 0xFFFF);
        L->said_problem = 1;
        return 0;
    }
    if ((L->hs.device_mode == 1) != (L->io_module != 0)) {
        close_tcp(L);
        if (!L->said_problem)
            fail(L->io_module, L->io_module ? "the device at %s is a motion controller, not an I/O module"
                                            : "the device at %s is an I/O module, not a motion controller", address);
        L->said_problem = 1;
        return 0;
    }

    L->peer = addr;
    L->peer.sin_port = htons(WCNC_UDP_MOTION_PORT);
    snprintf(L->ip, sizeof(L->ip), "%s", address);
    note("%s found at %s, firmware %u.%u.%u", L->what, address,
         (unsigned)(v >> 24) & 0xFF, (unsigned)(v >> 16) & 0xFF, (unsigned)v & 0xFFFF);
    return 1;
}

/* ---- outputs: also the heartbeat for the controller's watchdog ------ */

static void pump_outputs(link_t *L, const tg_wanted_t *w, int running, uint64_t now)
{
    wcnc_io_control_packet_t io;
    memset(&io, 0, sizeof(io));
    io.flags = WCNC_IO_FLAG_WATCHDOG | WCNC_IO_FLAG_REPLY_TO_SOURCE;
    if (L->io_module) {
        io.io_outputs = w->io_outputs;
    } else {
        io.misc_outputs = w->outputs;
        io.spindle_state = w->spindle;
        io.spindle_rpm = w->rpm;
        io.coolant_state = w->coolant;
        if (w->enable && running) io.flags |= WCNC_IO_FLAG_CHARGE_PUMP;
        /* The controller's own limit stop must stand aside while LinuxCNC
         * homes onto a switch, and while an axis is being moved off a
         * switch it is standing on.  (The controller can only do that for
         * all its limit inputs at once; LinuxCNC's own limits still apply.) */
        if (w->homing || (L->have_status && (L->st.limit_switches & w->limit_mask)))
            io.flags |= WCNC_IO_FLAG_LIMIT_OVERRIDE;
    }

    const uint8_t *body = (const uint8_t *)&io + sizeof(wcnc_header_t);
    int changed = !L->io_sent || memcmp(body, L->io_last, sizeof(L->io_last)) != 0;
    if (!changed && now - L->io_ms < IO_PERIOD_MS) return;
    memcpy(L->io_last, body, sizeof(L->io_last));
    L->io_sent = 1;
    L->io_ms = now;
    finalize(L, &io, WCNC_PKT_IO_CONTROL, sizeof(io) - sizeof(wcnc_header_t));
    udp_send(L, &io, sizeof(io));
}

/* ===================================================================
 * Motion stream (the controller's link thread only)
 * =================================================================== */

enum { M_OFF = 0, M_RESET, M_FLUSH, M_RUN, M_FAULT };

typedef struct {
    int      state;
    uint32_t epoch;
    int      fault;                     /* latched until LinuxCNC has switched off */
    uint64_t t_state, t_flush, t_open;
    uint64_t t_enable;                  /* when "enable" came on */
    uint64_t t_stop;                    /* when "stop" was last sent */
    uint64_t t_progress;                /* when the controller last acknowledged something new */
    uint64_t t_foreign;                 /* since when its stream has not been ours (0 = it is) */
    uint64_t lag_us;                    /* how late the servo period now being handled is */
    uint8_t  stream;
    uint32_t lost_seen;

    /* servo periods -> segments */
    int64_t  sent[MAX_JOINTS];          /* position handed to the controller so far, 1/256 step */
    int64_t  last[MAX_JOINTS];          /* the newest servo period */
    int      have_last;
    int      seg_moving;                /* a period of the segment being collected moved */
    int      open;                      /* motion has been sent and not yet marked as ended */
    int      idle_checked, force_move;
    uint32_t seg_ns, ns_carry;
    uint64_t gap_us;                    /* standstill not sent since the last segment */
    double   dur_carry, trim, err_filt;

    /* segments the controller has not accepted yet */
    wcnc_motion_segment_t q[SEG_RING];
    uint32_t q_base, q_next, q_end;
    uint64_t unacked_us;

    uint32_t underruns, resent, corrections, buffer_ms;
    uint32_t target_ms;                 /* tiggy.buffer-target-ms */
    int      in_underrun, said_overspeed;
} motion_t;

static motion_t mot;

static const char *alarm_text(uint8_t code, char *buf, size_t n)
{
    if (code >= WCNC_ALARM_LIMIT_X && code <= WCNC_ALARM_LIMIT_C) {
        snprintf(buf, n, "limit switch input %d operated", code - WCNC_ALARM_LIMIT_X);
        return buf;
    }
    switch (code) {
    case WCNC_ALARM_ESTOP_ACTIVE:    return "its E-stop input operated";
    case WCNC_ALARM_HOST_LOST:       return "it heard nothing from LinuxCNC for a second";
    case WCNC_ALARM_BUFFER_UNDERRUN: return "it was sent a move it cannot make";
    case WCNC_ALARM_WATCHDOG:        return "its watchdog expired";
    default:
        snprintf(buf, n, "alarm %u", (unsigned)code);
        return buf;
    }
}

static void motion_fault(const char *fmt, ...) TG_PRINTF(1, 2);

static void motion_fault(const char *fmt, ...)
{
    char line[200];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (!mot.fault) fail(0, "%s", line);
    mot.fault = 1;
    mot.state = M_FAULT;
    mot.t_stop = 0;
}

static void motion_clear_stream(motion_t *m)
{
    m->have_last = 0;
    m->seg_moving = 0;
    m->open = 0;
    m->idle_checked = 1;
    m->force_move = 0;
    m->seg_ns = 0;
    m->ns_carry = 0;
    m->gap_us = (uint64_t)BUFFER_MAX_MS * 1000u;
    m->dur_carry = 0.0;
    m->trim = 0.0;
    m->err_filt = 0.0;
    m->q_base = m->q_next = m->q_end = 0;
    m->unacked_us = 0;
    m->buffer_ms = 0;
    m->in_underrun = 0;
    m->t_foreign = 0;
    m->lag_us = 0;
}

static int push_segment(motion_t *m, const int64_t steps[MAX_JOINTS], uint32_t duration_us, uint8_t flags)
{
    if ((uint32_t)(m->q_end - m->q_base) >= SEG_RING) {
        motion_fault("the controller is not taking the motion it is sent: stopped");
        return 0;
    }
    wcnc_motion_segment_t *s = &m->q[m->q_end & (SEG_RING - 1)];
    memset(s, 0, sizeof(*s));
    if (steps)
        for (int j = 0; j < MAX_JOINTS; j++) s->steps[j] = (int32_t)steps[j];
    s->duration_us = duration_us ? duration_us : 1;
    s->segment_id = (uint16_t)m->q_end;
    s->flags = (uint8_t)(WCNC_SEG_FLAG_TIMED | WCNC_SEG_FLAG_SUBSTEP | flags);
    m->unacked_us += s->duration_us;
    m->q_end++;
    return 1;
}

/* The periods collected so far become one segment. */
static void close_segment(link_t *L, motion_t *m, const tg_wanted_t *w, uint64_t now)
{
    uint32_t ns = m->seg_ns + m->ns_carry;
    uint32_t us = ns / 1000u;
    m->ns_carry = ns % 1000u;
    m->seg_ns = 0;
    int moving = m->seg_moving || m->force_move;
    m->seg_moving = 0;
    m->force_move = 0;
    if (us == 0) us = 1;

    int64_t steps[MAX_JOINTS];

    if (moving) {
        for (int j = 0; j < MAX_JOINTS; j++) {
            steps[j] = m->last[j] - m->sent[j];
            double rate = (double)(w->max_rate[j] ? w->max_rate[j] : L->hs.max_step_rate);
            double most = rate * (double)WCNC_SUBSTEP_SCALE * ((double)us / 1.0e6) * 2.0 +
                          (SETTLE_STEPS + 4.0) * WCNC_SUBSTEP_SCALE;
            if (fabs((double)steps[j]) > most) {
                motion_fault("joint %d: the position LinuxCNC commands jumped by %.0f steps: stopped",
                             j, (double)steps[j] / WCNC_SUBSTEP_SCALE);
                return;
            }
            double asked = fabs((double)steps[j]) / WCNC_SUBSTEP_SCALE / ((double)us / 1.0e6);
            if (L->hs.max_step_rate && asked > 1.05 * (double)L->hs.max_step_rate && !m->said_overspeed) {
                m->said_overspeed = 1;
                fail(0, "joint %d is commanded at %.0f steps a second; the controller makes at most %u and is "
                     "falling behind", j, asked, (unsigned)L->hs.max_step_rate);
            }
        }
        if (!m->open) {
            /* Motion begins.  A pause in front of it puts back the queued
             * time that ran out while nothing was sent, so the controller
             * again has its reserve when the movement starts. */
            uint64_t reserve = (uint64_t)w->buffer_ms * 1000u;
            uint64_t pause = m->gap_us < reserve ? m->gap_us : reserve;
            /* ...less the time this movement has already waited to be
             * handled (while the stream was being started, or the PC was
             * busy), so that it is not later still. */
            pause = pause > m->lag_us ? pause - m->lag_us : 0;
            if (pause && !push_segment(m, NULL, (uint32_t)pause, 0)) return;
            m->gap_us = 0;
            m->open = 1;
            m->t_open = now;
            m->trim = 0.0;
            m->err_filt = 0.0;
        }
        /* Clock matching: tell the controller the segment is a touch longer
         * or shorter than LinuxCNC's time, so its queue stays at its level
         * however the two clocks differ. */
        m->dur_carry += (double)us * (1.0 + m->trim);
        uint32_t dur = (uint32_t)m->dur_carry;
        m->dur_carry -= (double)dur;
        if (!push_segment(m, steps, dur, 0)) return;
        for (int j = 0; j < MAX_JOINTS; j++) m->sent[j] = m->last[j];
    } else if (m->open) {
        /* The first period of standstill ends the movement.  The fraction
         * of a step left over is squared up (it cannot make a step), so
         * that the controller and this driver agree exactly whenever the
         * machine stands still. */
        for (int j = 0; j < MAX_JOINTS; j++) {
            int64_t whole = (m->last[j] + WCNC_SUBSTEP_SCALE / 2) & ~(int64_t)(WCNC_SUBSTEP_SCALE - 1);
            steps[j] = whole - m->sent[j];
        }
        if (!push_segment(m, steps, us, WCNC_SEG_FLAG_LAST)) return;
        for (int j = 0; j < MAX_JOINTS; j++) m->sent[j] += steps[j];
        m->open = 0;
        m->gap_us = 0;
        m->idle_checked = 0;
    } else {
        m->gap_us += us;
        if (m->gap_us > (uint64_t)BUFFER_MAX_MS * 1000u) m->gap_us = (uint64_t)BUFFER_MAX_MS * 1000u;
    }
}

/* Standing still, the position LinuxCNC commands and the one the controller
 * is at agree to within the rounding of a step.  A small difference is put
 * right with a short move; a large one must not be: the machine would jump. */
static void settle(motion_t *m, const int64_t commanded[MAX_JOINTS])
{
    for (int j = 0; j < MAX_JOINTS; j++) {
        int64_t off = commanded[j] - m->sent[j];
        if (off < 0) off = -off;
        if (off > (int64_t)SETTLE_STEPS * WCNC_SUBSTEP_SCALE) {
            motion_fault("joint %d: LinuxCNC's position and the controller's differ by %.1f steps: stopped",
                         j, (double)(commanded[j] - m->sent[j]) / WCNC_SUBSTEP_SCALE);
            return;
        }
        if (off > WCNC_SUBSTEP_SCALE) m->force_move = 1;
    }
}

static void take_samples(link_t *L, motion_t *m, const tg_wanted_t *w, uint64_t now)
{
    uint32_t tail = smp_tail;
    while (m->state == M_RUN) {
        uint32_t head = smp_head;
        tg_barrier();
        if (tail == head) break;
        tg_sample_t s = samples[tail & (SAMPLE_RING - 1)];
        tail++;
        if (s.epoch != m->epoch) continue;
        m->lag_us = (uint64_t)(head - tail) * (s.ns / 1000u);

        if (!m->have_last) {
            settle(m, s.target);                /* the first period after starting */
            if (m->state != M_RUN) break;
        } else if (memcmp(s.target, m->last, sizeof(m->last)) != 0) {
            m->seg_moving = 1;
        }
        memcpy(m->last, s.target, sizeof(m->last));
        m->have_last = 1;
        m->seg_ns += s.ns;
        if (m->seg_ns >= SEG_TARGET_NS) close_segment(L, m, w, now);
    }
    smp_tail = tail;
}

static void send_segments(link_t *L, motion_t *m, uint64_t now)
{
    const wcnc_status_report_t *st = &L->st;
    if (!L->have_status || st->stream_id != m->stream) return;

    uint32_t inflight = m->q_next - m->q_base;
    /* The controller takes segments strictly in order, so one lost packet
     * holds up everything behind it.  Each packet therefore repeats the
     * few segments before its new ones, and anything still unacknowledged
     * after RESEND_MS goes out again in full. */
    if (inflight == 0) {
        m->t_progress = now;
    } else if (now - m->t_progress >= RESEND_MS) {
        m->resent += inflight;
        m->q_next = m->q_base;
        inflight = 0;
        m->t_progress = now;
    }
    int room = (int)st->buffer_available - (int)inflight;
    if (room > SEND_WINDOW - (int)inflight) room = SEND_WINDOW - (int)inflight;

    for (int packets = 0; packets < 6 && m->q_next != m->q_end && room > 0; packets++) {
        uint32_t back = m->q_next - m->q_base;
        if (back > REPEAT_SEGS) back = REPEAT_SEGS;
        uint32_t fresh = m->q_end - m->q_next;
        if (fresh > WCNC_MAX_SEGMENTS_PER_PACKET - back) fresh = WCNC_MAX_SEGMENTS_PER_PACKET - back;
        if (fresh > (uint32_t)room) fresh = (uint32_t)room;
        uint32_t first = m->q_next - back, count = back + fresh;

        wcnc_motion_packet_t p;
        memset(&p, 0, sizeof(wcnc_header_t) + 4);
        p.segment_count = (uint8_t)count;
        p.flags = WCNC_MOTION_FLAG_SEQUENCED;
        p.stream_id = m->stream;
        for (uint32_t i = 0; i < count; i++) p.segments[i] = m->q[(first + i) & (SEG_RING - 1)];
        size_t payload = 4 + count * sizeof(wcnc_motion_segment_t);
        finalize(L, &p, WCNC_PKT_MOTION_SEGMENT, payload);
        udp_send(L, &p, sizeof(wcnc_header_t) + payload);

        m->q_next += fresh;
        room -= (int)fresh;
    }
}

/* A status report has arrived. */
static void motion_on_status(link_t *L, uint64_t now)
{
    motion_t *m = &mot;
    const wcnc_status_report_t *st = &L->st;
    if (m->state != M_RUN || st->stream_id != m->stream) return;

    /* next_segment_id acknowledges everything before it */
    uint32_t acked = m->q_base + (uint16_t)(st->next_segment_id - (uint16_t)m->q_base);
    if (acked != m->q_base && (uint32_t)(acked - m->q_base) <= (uint32_t)(m->q_end - m->q_base)) {
        while (m->q_base != acked) {
            m->unacked_us -= m->q[m->q_base & (SEG_RING - 1)].duration_us;
            m->q_base++;
        }
        if ((int32_t)(m->q_next - m->q_base) < 0) m->q_next = m->q_base;
        m->t_progress = now;
    }

    int underrun = (st->stream_flags & WCNC_STREAM_UNDERRUN) != 0;
    if (underrun && !m->in_underrun && m->open) m->underruns++;
    m->in_underrun = underrun;

    if (m->open) {
        uint64_t ahead_us = (uint64_t)st->queued_ms * 1000u + m->unacked_us + m->seg_ns / 1000u;
        m->buffer_ms = (uint32_t)(ahead_us / 1000u);
        if (now - m->t_open > 500 && !underrun) {
            double target_s = (double)m->target_ms / 1000.0;
            double err = (double)ahead_us / 1.0e6 - target_s;
            m->err_filt += (err - m->err_filt) * 0.02;
            double trim = -TRIM_GAIN * m->err_filt;
            if (trim > TRIM_LIMIT) trim = TRIM_LIMIT;
            if (trim < -TRIM_LIMIT) trim = -TRIM_LIMIT;
            m->trim = trim;
        }
    } else {
        m->buffer_ms = st->queued_ms;
    }
}

static void motion_service(link_t *L, const tg_wanted_t *w, uint64_t now)
{
    motion_t *m = &mot;
    const wcnc_status_report_t *st = &L->st;
    int stopped = st->state == WCNC_STATE_ESTOP || st->state == WCNC_STATE_ALARM;

    m->target_ms = w->buffer_ms;
    if (!w->enable) {
        /* Machine off: nothing may move, and the controller is held stopped
         * with its drivers and outputs off. */
        if (m->state != M_OFF) m->t_stop = 0;
        m->state = M_OFF;
        m->fault = 0;
        smp_tail = smp_head;
        m->lost_seen = smp_lost;
        if (L->have_status && !stopped && (m->t_stop == 0 || now - m->t_stop >= STOP_REPEAT_MS)) {
            m->t_stop = now;
            udp_send_simple(L, WCNC_PKT_ESTOP, 2);
        }
        return;
    }

    if (m->state == M_OFF || (w->epoch != m->epoch && m->state != M_FAULT)) {
        m->epoch = w->epoch;
        m->state = M_RESET;
        m->t_state = m->t_enable = now;
        m->said_overspeed = 0;
    } else if (w->epoch != m->epoch) {      /* off and on again while latched */
        m->epoch = w->epoch;
        m->fault = 0;
        m->state = M_RESET;
        m->t_state = m->t_enable = now;
    }

    switch (m->state) {
    case M_RESET:
        if (!L->have_status || !L->cfg_done) { m->t_state = now; break; }
        if (st->estop_input) { m->t_state = now; break; }       /* tiggy.ready is false meanwhile */
        if (stopped) {
            if (now - L->command_ms >= COMMAND_REPEAT_MS) {
                L->command_ms = now;
                udp_send_simple(L, WCNC_PKT_RESET, 1);
            }
            if (now - m->t_state > RESET_TIMEOUT_MS)
                motion_fault("the controller does not leave its stopped state (%s)", "is its E-stop input operated?");
            break;
        }
        m->stream = (uint8_t)(st->stream_id + 1);
        m->state = M_FLUSH;
        m->t_state = now;
        m->t_flush = 0;
        break;

    case M_FLUSH:
        if (stopped) { m->state = M_RESET; m->t_state = now; break; }
        if (st->stream_id == m->stream && st->next_segment_id == 0 && !(st->stream_flags & WCNC_STREAM_FLUSHING)) {
            /* The controller stands still on a new, empty stream: start
             * from where it really is. */
            motion_clear_stream(m);
            for (int j = 0; j < MAX_JOINTS; j++) m->sent[j] = L->position[j] * WCNC_SUBSTEP_SCALE;
            /* What LinuxCNC commanded meanwhile is played, a little late.
             * If getting ready took too long for that, it is dropped, and
             * the position check that follows decides whether to go on. */
            if (now - m->t_enable > m->target_ms) smp_tail = smp_head;
            m->lost_seen = smp_lost;
            m->state = M_RUN;
            break;
        }
        if (m->t_flush == 0 || now - m->t_flush >= FLUSH_REPEAT_MS) {
            wcnc_stream_flush_packet_t f;
            memset(&f, 0, sizeof(f));
            f.stream_id = m->stream;
            f.mode = WCNC_FLUSH_IMMEDIATE;
            finalize(L, &f, WCNC_PKT_STREAM_FLUSH, sizeof(f) - sizeof(wcnc_header_t));
            udp_send(L, &f, sizeof(f));
            m->t_flush = now;
        }
        if (now - m->t_state > RESET_TIMEOUT_MS) motion_fault("the controller does not start a new motion stream");
        break;

    case M_RUN:
        if (stopped) {
            char buf[48];
            motion_fault("the controller stopped: %s", alarm_text(st->alarm_code, buf, sizeof(buf)));
            break;
        }
        if (smp_lost != m->lost_seen) {
            motion_fault("motion could not be passed on fast enough: stopped");
            break;
        }
        if (st->stream_id == m->stream) {
            m->t_foreign = 0;
        } else if (m->t_foreign == 0) {
            m->t_foreign = now;
        } else if (now - m->t_foreign > 1000) {
            motion_fault("another program has taken over the controller: stopped");
            break;
        }
        take_samples(L, m, w, now);
        if (m->state != M_RUN) break;
        send_segments(L, m, now);

        /* Standing still with everything played: the controller's count is
         * the truth.  It always agrees unless something was lost on the
         * way, and then the difference is made good here. */
        if (!m->open && !m->idle_checked && m->q_base == m->q_end && st->stream_id == m->stream &&
            st->next_segment_id == (uint16_t)m->q_end && st->state == WCNC_STATE_IDLE &&
            !(st->flags & WCNC_STATUS_MOTION_ACTIVE) && st->queued_ms == 0) {
            int differs = 0;
            m->idle_checked = 1;
            for (int j = 0; j < MAX_JOINTS; j++) {
                int64_t real = L->position[j] * WCNC_SUBSTEP_SCALE;
                if (real == m->sent[j]) continue;
                m->corrections++;
                m->sent[j] = real;
                differs = 1;
            }
            if (differs && m->have_last) settle(m, m->last);
        }
        break;

    case M_FAULT:
    default:
        smp_tail = smp_head;
        if (L->have_status && !stopped && (m->t_stop == 0 || now - m->t_stop >= STOP_REPEAT_MS)) {
            m->t_stop = now;
            udp_send_simple(L, WCNC_PKT_ESTOP, 2);
        }
        break;
    }
}

/* ===================================================================
 * Link thread
 * =================================================================== */

static void publish(link_t *L)
{
    if (L->io_module) {
        tg_io_actual_t a;
        memset(&a, 0, sizeof(a));
        a.online = (uint8_t)(L->online && L->have_status);
        a.inputs = a.online ? L->st.io_inputs : 0;
        seq_write(&sh_io.seq, &sh_io.v, &a, sizeof(a));
        return;
    }
    tg_actual_t a;
    memset(&a, 0, sizeof(a));
    int live = L->online && L->have_status;
    a.online = (uint8_t)live;
    a.estop = (uint8_t)(live && L->st.estop_input);
    a.ready = (uint8_t)(live && L->cfg_once && !L->st.estop_input && !mot.fault);
    a.beat = ++L->beat;
    if (live) {
        a.probe = L->st.probe_state ? 1 : 0;
        a.switches = L->st.limit_switches;
        a.inputs = L->st.misc_inputs;
        a.rpm = L->st.spindle_rpm;
    }
    memcpy(a.position, L->position, sizeof(a.position));
    a.underruns = mot.underruns;
    a.resent = mot.resent;
    a.corrections = mot.corrections;
    a.buffer_ms = live ? mot.buffer_ms : 0;
    seq_write(&sh_actual.seq, &sh_actual.v, &a, sizeof(a));
}

static void session_begin(link_t *L)
{
    uint64_t now = now_ms();
    L->online = 1;
    L->have_status = 0;
    L->position_valid = 0;
    L->online_ms = L->ping_ms = L->pong_ms = now;
    L->io_sent = 0;
    L->io_ms = 0;
    L->command_ms = 0;
    L->n_sent = 0;
    L->cfg_inflight = 0;
    L->cfg_tries = 0;
    L->cfg_dirty = 0;
    L->cfg_done = L->io_module ? 1 : 0;
    L->cfg_once = 0;
    L->hs_wanted = 0;
    L->hs_ms = 0;
    L->rates_checked = 0;
    L->rounds_missing = 0;
    L->said_missing = 0;
    L->said_problem = 0;
    L->loss[0] = 0;
    if (!L->io_module) {
        motion_clear_stream(&mot);
        if (mot.fault) {
            /* The controller was lost with the machine on.  It does not
             * move again until LinuxCNC has switched off and on. */
            mot.state = M_FAULT;
            mot.t_stop = 0;
        } else {
            mot.state = M_OFF;
            mot.epoch = sh_wanted.v.epoch - 1;  /* whatever is enabled now starts afresh */
        }
    }
}

static void service(link_t *L)
{
    struct pollfd p[2];
    p[0].fd = L->udp; p[0].events = POLLIN; p[0].revents = 0;
    p[1].fd = L->tcp; p[1].events = POLLIN; p[1].revents = 0;
    int r = poll(p, 2, 1);

    if (r > 0 && (p[0].revents & POLLIN)) {
        for (int guard = 0; guard < 64; guard++) {
            uint8_t buf[600];
            struct sockaddr_in from;
            socklen_t fl = sizeof(from);
            int n = (int)recvfrom(L->udp, (char *)buf, (int)sizeof(buf), 0, (struct sockaddr *)&from, &fl);
            if (n <= 0) break;
            on_udp(L, buf, n, &from);
        }
    }
    if (r > 0 && (p[1].revents & (POLLIN | POLLHUP | POLLERR))) {
        if (!tcp_pump(L)) { go_offline(L, "it closed the connection"); return; }
    }

    uint64_t now = now_ms();
    if (now - L->ping_ms >= PING_PERIOD_MS) {
        wcnc_ping_packet_t ping;
        memset(&ping, 0, sizeof(ping));
        ping.ping_id = (uint32_t)now;
        L->ping_ms = now;
        finalize(L, &ping, WCNC_PKT_PING, sizeof(ping) - sizeof(wcnc_header_t));
        if (!tcp_send_frame(L, &ping, sizeof(ping))) { go_offline(L, "cannot send to it"); return; }
    }
    if (now - L->pong_ms > PONG_TIMEOUT_MS) { go_offline(L, "it stopped answering"); return; }
    if (L->have_status && now - L->status_ms > STATUS_LOST_MS) { go_offline(L, "no status reports arrive"); return; }
    if (!L->have_status && now - L->online_ms > FIRST_STATUS_MS) { go_offline(L, "it sends no status reports"); return; }

    tg_wanted_t w;
    if (!seq_read(&sh_wanted.seq, &sh_wanted.v, &w, sizeof(w), 1000)) return;

    pump_outputs(L, &w, L->io_module || mot.state == M_RUN, now);
    if (!L->io_module) {
        /* Before the servo thread has run there are no settings to pass on
         * yet (and nothing is enabled). */
        if (w.valid) pump_settings(L, &w, now);
        if (L->tcp < 0) return;
        motion_service(L, &w, now);
    }
    publish(L);
}

static void *link_thread(void *arg)
{
    link_t *L = (link_t *)arg;

    while (!stop_threads) {
        if (!L->io_module) smp_tail = smp_head;         /* nothing can be passed on while there is no controller */
        publish(L);
        if (!open_udp(L)) { nap_ms(500); continue; }

        char address[64] = "";
        int found = 0;
        if (L->fixed_ip[0]) {
            snprintf(address, sizeof(address), "%s", L->fixed_ip);
            found = 1;
        } else {
            found = discover(L, address, sizeof(address));
        }
        if (stop_threads) break;
        if (!found || !connect_to(L, address)) {
            if (++L->rounds_missing == 3 && !L->said_missing && !L->said_problem) {
                L->said_missing = 1;
                if (L->fixed_ip[0])
                    fail(L->io_module, "no %s answers at %s (still trying): is it switched on?", L->what, L->fixed_ip);
                else
                    fail(L->io_module, "no %s found on the network (still looking): is it switched on?", L->what);
            }
            nap_ms(RETRY_PAUSE_MS);
            continue;
        }

        session_begin(L);
        while (!stop_threads && L->tcp >= 0) service(L);
        L->online = 0;
        if (!L->io_module) {
            /* Lost with the machine on: that stays a fault until LinuxCNC
             * has switched off, however soon the controller is back. */
            if (L->tcp < 0 && sh_wanted.v.enable) mot.fault = 1;
            mot.state = mot.fault ? M_FAULT : M_OFF;
            mot.buffer_ms = 0;
        }
        publish(L);

        if (L->tcp >= 0) {
            /* Leaving on purpose: stop, switch the outputs off and stand the
             * watchdog down so the controller does not take this for a host
             * that died. */
            wcnc_io_control_packet_t io;
            if (!L->io_module) udp_send_simple(L, WCNC_PKT_ESTOP, 3);
            memset(&io, 0, sizeof(io));
            for (int i = 0; i < 3; i++) {
                finalize(L, &io, WCNC_PKT_IO_CONTROL, sizeof(io) - sizeof(wcnc_header_t));
                udp_send(L, &io, sizeof(io));
                usleep(15000);
            }
            close_tcp(L);
        } else {
            int enabled = !L->io_module && sh_wanted.v.enable;
            if (enabled) fail(L->io_module, "connection to the %s lost: %s", L->what, L->loss);
            else note("connection to the %s lost: %s", L->what, L->loss);
        }
    }

    close_tcp(L);
    if (L->udp >= 0) close(L->udp);
    L->udp = -1;
    return NULL;
}

static int link_start(link_t *L, int io_module, const char *address)
{
    memset(L, 0, sizeof(*L));
    L->io_module = io_module;
    L->what = io_module ? "I/O module" : "controller";
    L->udp = L->tcp = -1;
    if (address && address[0] && strcmp(address, "auto") != 0)
        snprintf(L->fixed_ip, sizeof(L->fixed_ip), "%s", address);
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 512 * 1024);
    int r = pthread_create(&L->thread, &attr, link_thread, L);
    pthread_attr_destroy(&attr);
    L->started = (r == 0);
    return L->started;
}

/* ===================================================================
 * Component set-up
 * =================================================================== */

#define CHECK(call) do { int r_ = (call); if (r_ < 0) return r_; } while (0)

static int export_pins(tg_hal_t *h)
{
    for (int j = 0; j < joints; j++) {
        CHECK(hal_pin_float_newf(HAL_IN, &h->joint[j].pos_cmd, comp_id, COMP ".%d.position-cmd", j));
        CHECK(hal_pin_float_newf(HAL_OUT, &h->joint[j].pos_fb, comp_id, COMP ".%d.position-fb", j));
        CHECK(hal_pin_s32_newf(HAL_OUT, &h->joint[j].counts, comp_id, COMP ".%d.counts", j));
        CHECK(hal_pin_bit_newf(HAL_IN, &h->joint[j].homing, comp_id, COMP ".%d.homing", j));
        CHECK(hal_param_float_newf(HAL_RW, &h->joint[j].scale, comp_id, COMP ".%d.position-scale", j));
        CHECK(hal_param_float_newf(HAL_RW, &h->joint[j].maxvel, comp_id, COMP ".%d.maxvel", j));
        CHECK(hal_param_float_newf(HAL_RW, &h->joint[j].maxaccel, comp_id, COMP ".%d.maxaccel", j));
        CHECK(hal_param_bit_newf(HAL_RW, &h->joint[j].step_invert, comp_id, COMP ".%d.step-invert", j));
        CHECK(hal_param_bit_newf(HAL_RW, &h->joint[j].dir_invert, comp_id, COMP ".%d.dir-invert", j));
        h->joint[j].scale = 1.0;
    }
    for (int i = 0; i < SWITCHES; i++) {
        CHECK(hal_pin_bit_newf(HAL_OUT, &h->sw[i].in, comp_id, COMP ".switch.%d", i));
        CHECK(hal_pin_bit_newf(HAL_OUT, &h->sw[i].in_not, comp_id, COMP ".switch.%d-not", i));
        CHECK(hal_param_bit_newf(HAL_RW, &h->sw[i].invert, comp_id, COMP ".switch.%d-invert", i));
        CHECK(hal_param_bit_newf(HAL_RW, &h->sw[i].is_limit, comp_id, COMP ".switch.%d-is-limit", i));
    }
    for (int i = 0; i < AUX_INPUTS; i++) {
        CHECK(hal_pin_bit_newf(HAL_OUT, &h->aux_in[i].in, comp_id, COMP ".input.%d", i));
        CHECK(hal_pin_bit_newf(HAL_OUT, &h->aux_in[i].in_not, comp_id, COMP ".input.%d-not", i));
    }
    for (int i = 0; i < AUX_OUTPUTS; i++)
        CHECK(hal_pin_bit_newf(HAL_IN, &h->aux_out[i], comp_id, COMP ".output.%d", i));

    CHECK(hal_pin_bit_newf(HAL_IN, &h->enable, comp_id, COMP ".enable"));
    CHECK(hal_pin_bit_newf(HAL_OUT, &h->ready, comp_id, COMP ".ready"));
    CHECK(hal_pin_bit_newf(HAL_OUT, &h->fault, comp_id, COMP ".fault"));
    *h->fault = 1;                      /* until the controller is known to be ready */
    CHECK(hal_pin_bit_newf(HAL_OUT, &h->connected, comp_id, COMP ".connected"));
    CHECK(hal_pin_bit_newf(HAL_OUT, &h->probe, comp_id, COMP ".probe"));
    CHECK(hal_pin_bit_newf(HAL_OUT, &h->estop, comp_id, COMP ".estop-input"));
    CHECK(hal_pin_bit_newf(HAL_IN, &h->spindle_fwd, comp_id, COMP ".spindle-forward"));
    CHECK(hal_pin_bit_newf(HAL_IN, &h->spindle_rev, comp_id, COMP ".spindle-reverse"));
    CHECK(hal_pin_float_newf(HAL_IN, &h->spindle_speed, comp_id, COMP ".spindle-speed"));
    CHECK(hal_pin_float_newf(HAL_OUT, &h->spindle_rps, comp_id, COMP ".spindle-speed-fb"));
    CHECK(hal_pin_float_newf(HAL_OUT, &h->spindle_rpm, comp_id, COMP ".spindle-speed-fb-rpm"));
    CHECK(hal_pin_bit_newf(HAL_IN, &h->flood, comp_id, COMP ".coolant-flood"));
    CHECK(hal_pin_bit_newf(HAL_IN, &h->mist, comp_id, COMP ".coolant-mist"));
    CHECK(hal_pin_float_newf(HAL_OUT, &h->buffer_ms, comp_id, COMP ".buffer-ms"));
    CHECK(hal_pin_u32_newf(HAL_OUT, &h->underruns, comp_id, COMP ".underruns"));
    CHECK(hal_pin_u32_newf(HAL_OUT, &h->resent, comp_id, COMP ".segments-resent"));
    CHECK(hal_pin_u32_newf(HAL_OUT, &h->corrections, comp_id, COMP ".position-corrections"));

    CHECK(hal_param_u32_newf(HAL_RW, &h->pulse_us, comp_id, COMP ".step-pulse-us"));
    CHECK(hal_param_u32_newf(HAL_RW, &h->dir_setup_us, comp_id, COMP ".dir-setup-us"));
    CHECK(hal_param_u32_newf(HAL_RW, &h->idle_release_ms, comp_id, COMP ".idle-release-ms"));
    CHECK(hal_param_u32_newf(HAL_RW, &h->debounce_ms, comp_id, COMP ".debounce-ms"));
    CHECK(hal_param_u32_newf(HAL_RW, &h->buffer_target_ms, comp_id, COMP ".buffer-target-ms"));
    CHECK(hal_param_bit_newf(HAL_RW, &h->enable_invert, comp_id, COMP ".enable-invert"));
    CHECK(hal_param_bit_newf(HAL_RW, &h->probe_invert, comp_id, COMP ".probe-invert"));
    CHECK(hal_param_bit_newf(HAL_RW, &h->estop_invert, comp_id, COMP ".estop-invert"));
    CHECK(hal_param_bit_newf(HAL_RW, &h->spindle_pwm, comp_id, COMP ".spindle-pwm"));
    CHECK(hal_param_u32_newf(HAL_RW, &h->pwm_freq, comp_id, COMP ".spindle-pwm-freq"));
    CHECK(hal_param_float_newf(HAL_RW, &h->spindle_max_rpm, comp_id, COMP ".spindle-max-rpm"));
    CHECK(hal_param_u32_newf(HAL_RW, &h->min_duty, comp_id, COMP ".spindle-min-duty"));
    CHECK(hal_param_u32_newf(HAL_RW, &h->pump_freq, comp_id, COMP ".charge-pump-freq"));
    h->pulse_us = 5;
    h->dir_setup_us = 5;
    h->debounce_ms = 5;
    h->buffer_target_ms = BUFFER_DEFAULT_MS;
    h->pwm_freq = 1000;
    h->spindle_max_rpm = 24000.0;

    if (have_io_module) {
        CHECK(hal_pin_bit_newf(HAL_OUT, &h->io_connected, comp_id, COMP ".io.connected"));
        for (int i = 0; i < IO_CHANNELS; i++) {
            CHECK(hal_pin_bit_newf(HAL_OUT, &h->io_in[i], comp_id, COMP ".io.in-%02d", i));
            CHECK(hal_pin_bit_newf(HAL_OUT, &h->io_in_not[i], comp_id, COMP ".io.in-%02d-not", i));
            CHECK(hal_pin_bit_newf(HAL_IN, &h->io_out[i], comp_id, COMP ".io.out-%02d", i));
        }
    }

    CHECK(hal_export_funct(COMP ".read", tg_read, h, 1, 0, comp_id));
    CHECK(hal_export_funct(COMP ".write", tg_write, h, 1, 0, comp_id));
    return 0;
}

int rtapi_app_main(void)
{
    if (joints < 1 || joints > MAX_JOINTS) {
        rtapi_print_msg(RTAPI_MSG_ERR, COMP ": joints=%d is not possible: 1 to %d\n", joints, MAX_JOINTS);
        return -1;
    }
    have_io_module = io_ip && io_ip[0];

    /* Nothing is left over from an earlier load */
    memset(&mot, 0, sizeof(mot));
    memset(&sh_wanted, 0, sizeof(sh_wanted));
    memset(&sh_actual, 0, sizeof(sh_actual));
    memset(&sh_io, 0, sizeof(sh_io));
    memset(&rt_actual, 0, sizeof(rt_actual));
    memset(&rt_io, 0, sizeof(rt_io));
    memset(&link_main, 0, sizeof(link_main));
    memset(&link_io, 0, sizeof(link_io));
    memset(sh_messages, 0, sizeof(sh_messages));
    smp_head = smp_tail = smp_lost = 0;
    rt_epoch = 0;
    rt_enabled = 0;
    rt_beat = 0;
    rt_silent_ns = 0;

    comp_id = hal_init(COMP);
    if (comp_id < 0) return comp_id;

    hal = (tg_hal_t *)hal_malloc(sizeof(tg_hal_t));
    if (!hal) {
        hal_exit(comp_id);
        return -1;
    }
    memset(hal, 0, sizeof(*hal));

    int r = export_pins(hal);
    if (r < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, COMP ": could not create its HAL pins (%d)\n", r);
        hal_exit(comp_id);
        return r;
    }

    stop_threads = 0;
    if (!link_start(&link_main, 0, ip) || (have_io_module && !link_start(&link_io, 1, io_ip))) {
        rtapi_print_msg(RTAPI_MSG_ERR, COMP ": could not start its network thread\n");
        stop_threads = 1;
        if (link_main.started) pthread_join(link_main.thread, NULL);
        hal_exit(comp_id);
        return -1;
    }

    hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
    stop_threads = 1;
    if (link_main.started) pthread_join(link_main.thread, NULL);
    if (link_io.started) pthread_join(link_io.thread, NULL);
    hal_exit(comp_id);
}
