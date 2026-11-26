/*
 * Firmware-in-a-DLL: the controller's motion core and step engine, built
 * for the PC, with a simulated machine in place of the hardware.
 *
 * tools/firmware_sim.py loads this and puts it behind real UDP and TCP
 * sockets, so the protocol conformance tests and the host software (the
 * Mach3 plugin, LinuxCNC) can be run against the firmware's own logic
 * without a board.
 *
 * Everything here is called from one thread.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../../main/motion/motion_core.h"
#include "../../main/motion/step_engine.h"
#include "../../main/motion/spindle_track.h"
#include "se_port.h"

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif

#define AXES WCNC_MAX_AXES

/* ------------------------------------------------------------------ clock */
uint64_t pc_now_us = 1000000;
uint64_t pc_alarm_us = 0;
bool     pc_alarm_set = false;
static uint64_t s_last_tick_us = 1000000;

/* ---------------------------------------------------------- machine model */
static struct {
    int64_t  phys[AXES];            /* steps made since start; never zeroed by homing */
    bool     dir_neg[AXES];
    double   lim_lo[AXES], lim_hi[AXES];
    bool     probe_z_valid;
    double   probe_z;
    int      force_estop, force_probe, force_limits, force_misc_in, force_io_in;
    /* pulse statistics */
    uint64_t last_step_us[AXES];
    uint32_t min_gap_us[AXES];
    uint64_t steps[AXES];
    uint64_t isr_calls;
    /* outputs */
    mc_outputs_t out;
    double   duty;
    bool     drivers;
    bool     io_module;
    int      encoder_ppr;
    uint32_t fw_version;
    char     log[4096];
    size_t   log_len;
} m;

/* ---------------------------------------------------------------- spindle
 * A spindle that turns at a speed of its own (following the spindle output,
 * or whatever a test asks for, with a limited acceleration and an optional
 * ripple), and the sensor on it: a quadrature encoder with or without an
 * index pulse, or a single pulse sensor.  The sensor's signals go to the
 * firmware's spindle tracking exactly as the hardware would deliver them:
 * the count sampled every SPT_SAMPLE_US, pulses with the time they were
 * noticed (a few microseconds late, as an interrupt would be). */
#define SPIN_STEP_US    50u

static struct {
    int      mode;              /* SPT_* */
    int      ppr;               /* encoder lines, or pulses, per revolution */
    bool     index;             /* quadrature: with an index pulse */
    double   index_at;          /* where in the revolution the index (or first pulse) is */
    double   turns;             /* true angle at 'at', revolutions */
    double   rate;              /* revolutions per microsecond from 'at' on */
    double   rpm;               /* speed now, before ripple; negative = reverse */
    bool     follow;            /* speed follows the spindle output */
    double   forced;            /* ...or is this */
    double   accel;             /* rpm per second */
    double   ripple, ripple_hz; /* speed ripple as a fraction of the speed, and its frequency */
    uint64_t at, next_sample;
    int64_t  counts;
    uint32_t rnd;
} sp;

/* Steps of one axis with the true spindle angle at each: what a thread is */
#define TRACE_MAX   200000
static struct { double t, turns; int32_t pos; } s_trace[TRACE_MAX];
static int s_trace_n = 0, s_trace_axis = -1;

static double spin_turns_now(void)
{
    return sp.turns + sp.rate * ((double)pc_now_us - (double)sp.at);
}

/* When a pulse that came us_into_step into the step being made is noticed:
 * a few microseconds later, as by an interrupt (but not later than the
 * simulation has got to). */
static uint64_t spin_noticed(double us_into_step)
{
    sp.rnd = sp.rnd * 1664525u + 1013904223u;
    uint64_t late = (uint64_t)us_into_step + 2u + (sp.rnd >> 16) % 20u;
    return sp.at + (late > SPIN_STEP_US ? SPIN_STEP_US : late);
}

static void spin_step(void)
{
    double before = sp.turns, after = before + sp.rate * SPIN_STEP_US;

    if (sp.mode == SPT_PULSES) {
        double a = (before - sp.index_at) * sp.ppr, b = (after - sp.index_at) * sp.ppr;
        for (double k = floor(a) + 1.0; k <= b; k += 1.0)
            spt_pulse(spin_noticed((k - a) / (b - a) * SPIN_STEP_US));
    } else if (sp.mode == SPT_QUADRATURE && sp.index) {
        double a = before - sp.index_at, b = after - sp.index_at;
        if (floor(a) != floor(b)) {
            double edge = b > a ? floor(b) : floor(a);
            spt_index(spin_noticed((edge - a) / (b - a) * SPIN_STEP_US));
        }
    }
    sp.turns = after;
    sp.at += SPIN_STEP_US;

    if (sp.at >= sp.next_sample) {
        sp.next_sample += SPT_SAMPLE_US;
        if (sp.mode == SPT_QUADRATURE) {
            int64_t c = (int64_t)floor(sp.turns * 4.0 * sp.ppr);
            spt_counts(sp.at, (int32_t)(c - sp.counts));
            sp.counts = c;
        } else {
            spt_poll(sp.at);
        }
    }

    /* the speed from here on */
    double want = sp.forced;
    if (sp.follow) want = m.out.spindle == 1 ? (double)m.out.rpm : m.out.spindle == 2 ? -(double)m.out.rpm : 0.0;
    double change = sp.accel * SPIN_STEP_US / 1.0e6;
    if (sp.rpm < want) sp.rpm = sp.rpm + change > want ? want : sp.rpm + change;
    else if (sp.rpm > want) sp.rpm = sp.rpm - change < want ? want : sp.rpm - change;
    double wobble = 1.0 + sp.ripple * sin(6.283185307179586 * sp.ripple_hz * (double)sp.at / 1.0e6);
    sp.rate = sp.rpm * wobble / 60.0e6;
}

static void spin_sensor(int mode, int ppr, int index)
{
    sp.mode = ppr > 0 ? mode : SPT_NONE;
    sp.ppr = ppr;
    sp.index = index != 0;
    sp.counts = (int64_t)floor(sp.turns * 4.0 * ppr);
    m.encoder_ppr = sp.mode == SPT_NONE ? 0 : ppr;
    spt_setup((spt_mode_t)sp.mode, sp.mode == SPT_QUADRATURE ? 4u * (uint32_t)ppr : (uint32_t)ppr, sp.index);
}

void pc_set_dir(int axis, bool negative) { m.dir_neg[axis] = negative; }

void pc_step(uint8_t mask)
{
    for (int i = 0; i < AXES; i++) {
        if (!(mask & (1u << i))) continue;
        m.phys[i] += m.dir_neg[i] ? -1 : 1;
        if (i == s_trace_axis && s_trace_n < TRACE_MAX) {
            s_trace[s_trace_n].t = (double)pc_now_us;
            s_trace[s_trace_n].turns = spin_turns_now();
            s_trace[s_trace_n].pos = (int32_t)m.phys[i];
            s_trace_n++;
        }
        if (m.steps[i]) {
            uint64_t gap = pc_now_us - m.last_step_us[i];
            if (gap < m.min_gap_us[i]) m.min_gap_us[i] = (uint32_t)gap;
        }
        m.last_step_us[i] = pc_now_us;
        m.steps[i]++;
    }
}

bool pc_limit_active(int input)
{
    if (input < 0 || input >= AXES) return false;
    if (m.force_limits & (1 << input)) return true;
    return (double)m.phys[input] <= m.lim_lo[input] || (double)m.phys[input] >= m.lim_hi[input];
}

bool pc_probe_active(void)
{
    return m.force_probe || (m.probe_z_valid && (double)m.phys[2] <= m.probe_z);
}

/* ------------------------------------------------------------ config store */
typedef struct { uint16_t key; uint16_t type; uint32_t u; float f; char s[WCNC_CONFIG_VALUE_LEN]; } cfg_item_t;
static cfg_item_t s_cfg[160];
static int s_cfg_count = 0;

static cfg_item_t *cfg_find(uint16_t key)
{
    for (int i = 0; i < s_cfg_count; i++)
        if (s_cfg[i].key == key) return &s_cfg[i];
    return NULL;
}

static void cfg_def(uint16_t key, uint16_t type, uint32_t u, float f)
{
    cfg_item_t *c = cfg_find(key);
    if (!c) c = &s_cfg[s_cfg_count++];
    memset(c, 0, sizeof(*c));
    c->key = key; c->type = type; c->u = u; c->f = f;
}

static uint32_t cfg_u(uint16_t key)
{
    cfg_item_t *c = cfg_find(key);
    return c ? c->u : 0;
}

static void cfg_defaults(void)
{
    static const uint8_t pins[] = { 1, 2, 42, 41, 40, 39,  3, 4, 38, 33, 34, 35,  8,
                                    5, 6, 7, 15, 16, 17,  14, 13, 9, 48, 10, 11, 12, 0xFF, 0xFF, 0xFF };
    s_cfg_count = 0;
    for (int i = 0; i < AXES; i++) {
        cfg_def((uint16_t)(WCNC_CFG_STEPS_PER_MM_X + i), WCNC_VAL_FLOAT, 0, 800.0f);
        cfg_def((uint16_t)(WCNC_CFG_MAX_RATE_X + i), WCNC_VAL_UINT32, 20000, 0);
        cfg_def((uint16_t)(WCNC_CFG_ACCEL_X + i), WCNC_VAL_UINT32, 5000, 0);
    }
    cfg_def(WCNC_CFG_STEP_PULSE_US, WCNC_VAL_UINT16, 3, 0);
    cfg_def(WCNC_CFG_DIR_SETUP_US, WCNC_VAL_UINT16, 2, 0);
    cfg_def(WCNC_CFG_STEP_IDLE_DELAY_MS, WCNC_VAL_UINT16, 255, 0);
    cfg_def(WCNC_CFG_STATUS_INTERVAL_MS, WCNC_VAL_UINT16, 50, 0);
    cfg_def(WCNC_CFG_INVERT_STEP, WCNC_VAL_UINT8, 0, 0);
    cfg_def(WCNC_CFG_INVERT_DIR, WCNC_VAL_UINT8, 0, 0);
    cfg_def(WCNC_CFG_INVERT_ENABLE, WCNC_VAL_UINT8, 0, 0);
    cfg_def(WCNC_CFG_INVERT_LIMIT, WCNC_VAL_UINT8, 0x3F, 0);
    cfg_def(WCNC_CFG_INVERT_HOME, WCNC_VAL_UINT8, 0x3F, 0);
    cfg_def(WCNC_CFG_INVERT_ESTOP, WCNC_VAL_UINT8, 0, 0);
    cfg_def(WCNC_CFG_INVERT_PROBE, WCNC_VAL_UINT8, 0, 0);
    cfg_def(WCNC_CFG_LIMIT_ENABLE_MASK, WCNC_VAL_UINT8, 0x3F, 0);
    cfg_def(WCNC_CFG_INPUT_DEBOUNCE_MS, WCNC_VAL_UINT16, 5, 0);
    cfg_def(WCNC_CFG_HOMING_DIR_MASK, WCNC_VAL_UINT8, 0, 0);
    cfg_def(WCNC_CFG_HOMING_SEEK_RATE, WCNC_VAL_UINT32, 5000, 0);
    cfg_def(WCNC_CFG_HOMING_FEED_RATE, WCNC_VAL_UINT32, 500, 0);
    cfg_def(WCNC_CFG_HOMING_PULLOFF, WCNC_VAL_UINT32, 100, 0);
    cfg_def(WCNC_CFG_CHARGE_PUMP_FREQ, WCNC_VAL_UINT16, 10000, 0);
    cfg_def(WCNC_CFG_SPINDLE_PWM_FREQ, WCNC_VAL_UINT16, 1000, 0);
    cfg_def(WCNC_CFG_SPINDLE_MAX_RPM, WCNC_VAL_UINT32, 24000, 0);
    cfg_def(WCNC_CFG_SPINDLE_PWM_ENABLE, WCNC_VAL_UINT8, 0, 0);
    cfg_def(WCNC_CFG_SPINDLE_MIN_DUTY, WCNC_VAL_UINT8, 0, 0);
    for (int i = 0; i < (int)sizeof(pins); i++)
        cfg_def((uint16_t)(WCNC_CFG_PIN_STEP_X + i), WCNC_VAL_UINT8, pins[i], 0);
    cfg_def(WCNC_CFG_PIN_ENCODER_A, WCNC_VAL_UINT8, 0xFF, 0);
    cfg_def(WCNC_CFG_PIN_ENCODER_B, WCNC_VAL_UINT8, 0xFF, 0);
    cfg_def(WCNC_CFG_PIN_ENCODER_INDEX, WCNC_VAL_UINT8, 0xFF, 0);
    cfg_def(WCNC_CFG_ENCODER_PPR, WCNC_VAL_UINT16, 0, 0);
    cfg_def(WCNC_CFG_ENCODER_MODE, WCNC_VAL_UINT8, 0, 0);
    cfg_def(WCNC_CFG_MACHINE_LAYOUT, WCNC_VAL_UINT8, 0, 0);
    cfg_def(WCNC_CFG_PIN_MISC_IN0, WCNC_VAL_UINT8, 18, 0);
    cfg_def(WCNC_CFG_PIN_MISC_IN1, WCNC_VAL_UINT8, 21, 0);
    cfg_def(WCNC_CFG_PIN_MISC_IN2, WCNC_VAL_UINT8, 36, 0);
    cfg_def(WCNC_CFG_PIN_MISC_IN3, WCNC_VAL_UINT8, 0xFF, 0);
}

/* The terminals of the simulated board (the quad-memory DevKitC) with six
 * axes or four, and the spindle sensor that goes with the settings: what a
 * controller works out when it starts (firmware/main/pin_layout.c). */
static bool s_restart_wanted = false, s_restart_pending = false;
static uint64_t s_boot_us = 0;          /* when the simulated controller last started */
static bool s_sensor_wired = false;     /* a sensor has been fitted by the test, whatever the layout */

static void cfg_pin(uint16_t key, uint8_t pin)
{
    cfg_item_t *c = cfg_find(key);
    if (c) c->u = pin;
    else cfg_def(key, WCNC_VAL_UINT8, pin, 0);
}

static void apply_layout(void)
{
    static const uint8_t six[] = { 1, 2, 42, 41, 40, 39,  3, 4, 38, 33, 34, 35,  8,
                                   5, 6, 7, 15, 16, 17,  14, 13, 9, 48, 10, 11, 12, 0xFF, 0xFF, 0xFF };
    bool four = cfg_u(WCNC_CFG_MACHINE_LAYOUT) == 1;
    int ppr = (int)cfg_u(WCNC_CFG_ENCODER_PPR);
    uint32_t mode = cfg_u(WCNC_CFG_ENCODER_MODE);
    bool pulses = ppr > 0 && mode == 1;                 /* a single sensor */
    bool encoder = ppr > 0 && mode != 1;                /* A and B */
    /* the index terminal: a single sensor's in either layout, an encoder's
     * when the encoder itself has terminals */
    bool index = pulses || (encoder && mode == 0 && (four || s_sensor_wired));

    for (int i = 0; i < (int)sizeof(six); i++) cfg_pin((uint16_t)(WCNC_CFG_PIN_STEP_X + i), six[i]);
    cfg_pin(WCNC_CFG_PIN_MISC_IN0, 18);
    cfg_pin(WCNC_CFG_PIN_MISC_IN1, 21);
    cfg_pin(WCNC_CFG_PIN_MISC_IN2, 36);
    cfg_pin(WCNC_CFG_PIN_MISC_IN3, 0xFF);
    cfg_pin(WCNC_CFG_PIN_ENCODER_A, 0xFF);
    cfg_pin(WCNC_CFG_PIN_ENCODER_B, 0xFF);
    cfg_pin(WCNC_CFG_PIN_ENCODER_INDEX, index ? 37 : 0xFF);     /* GPIO 37 is spare in either layout */
    if (four) {
        static const uint16_t gone[] = { WCNC_CFG_PIN_STEP_B, WCNC_CFG_PIN_DIR_B, WCNC_CFG_PIN_LIMIT_B,
                                         WCNC_CFG_PIN_STEP_C, WCNC_CFG_PIN_DIR_C, WCNC_CFG_PIN_LIMIT_C };
        for (int i = 0; i < (int)(sizeof(gone) / sizeof(gone[0])); i++) cfg_pin(gone[i], 0xFF);
        cfg_pin(WCNC_CFG_PIN_MISC_OUT2, 40);
        cfg_pin(WCNC_CFG_PIN_MISC_OUT3, 39);
        cfg_pin(WCNC_CFG_PIN_MISC_OUT4, 34);
        if (!encoder) cfg_pin(WCNC_CFG_PIN_MISC_IN3, 16);       /* a terminal the sensor does not need */
    }
    if (encoder && (four || s_sensor_wired)) {
        cfg_pin(WCNC_CFG_PIN_ENCODER_A, 16);
        cfg_pin(WCNC_CFG_PIN_ENCODER_B, 17);
    }
    /* the sensor works if it has its terminals */
    bool works = pulses || (encoder && (four || s_sensor_wired));
    spin_sensor(pulses ? SPT_PULSES : SPT_QUADRATURE, works ? ppr : 0, mode == 0);
}

static void apply_timing(void)
{
    se_set_timing(cfg_u(WCNC_CFG_STEP_PULSE_US), cfg_u(WCNC_CFG_DIR_SETUP_US));
}

/* ------------------------------------------------------------------ outbox */
typedef struct { uint32_t ip; uint16_t port; uint16_t len; uint8_t data[256]; } dgram_t;
static dgram_t s_out[64];
static int s_out_head = 0, s_out_tail = 0;

/* --------------------------------------------------------- core's platform */
uint32_t mc_hal_ms(void) { return (uint32_t)(pc_now_us / 1000u); }

void mc_hal_load_config(mc_config_t *c)
{
    memset(c, 0, sizeof(*c));
    for (int i = 0; i < AXES; i++) {
        c->max_rate[i] = cfg_u((uint16_t)(WCNC_CFG_MAX_RATE_X + i));
        c->accel[i] = cfg_u((uint16_t)(WCNC_CFG_ACCEL_X + i));
    }
    c->limit_enable_mask = (uint8_t)cfg_u(WCNC_CFG_LIMIT_ENABLE_MASK);
    c->homing_dir_mask = (uint8_t)cfg_u(WCNC_CFG_HOMING_DIR_MASK);
    c->homing_seek_rate = cfg_u(WCNC_CFG_HOMING_SEEK_RATE);
    c->homing_feed_rate = cfg_u(WCNC_CFG_HOMING_FEED_RATE);
    c->homing_pulloff = cfg_u(WCNC_CFG_HOMING_PULLOFF);
    c->idle_release_ms = (uint16_t)cfg_u(WCNC_CFG_STEP_IDLE_DELAY_MS);
    c->status_interval_ms = (uint16_t)cfg_u(WCNC_CFG_STATUS_INTERVAL_MS);
    c->io_module = m.io_module;
}

uint8_t mc_hal_limit_inputs(void)
{
    uint8_t mask = 0;
    for (int i = 0; i < AXES; i++)
        if (pc_limit_active(i)) mask |= (uint8_t)(1u << i);
    return mask;
}

bool mc_hal_probe_input(void) { return pc_probe_active(); }
bool mc_hal_estop_input(void) { return m.force_estop != 0; }

void mc_hal_apply_outputs(const mc_outputs_t *out)
{
    m.out = *out;
    uint32_t max_rpm = cfg_u(WCNC_CFG_SPINDLE_MAX_RPM);
    if (!max_rpm) max_rpm = 1;
    if (!out->spindle) {
        m.duty = 0.0;
    } else if (cfg_u(WCNC_CFG_SPINDLE_PWM_ENABLE)) {
        double duty = (double)out->rpm / (double)max_rpm;
        double floor_ = cfg_u(WCNC_CFG_SPINDLE_MIN_DUTY) / 100.0;
        if (duty > 1.0) duty = 1.0;
        m.duty = duty < floor_ ? floor_ : duty;
    } else {
        m.duty = 1.0;
    }
}

void mc_hal_drivers(bool enabled) { m.drivers = enabled; }

void mc_hal_udp_send(uint32_t ip, uint16_t port, const void *data, size_t len)
{
    int next = (s_out_head + 1) % 64;
    if (next == s_out_tail || len > sizeof(s_out[0].data)) return;
    s_out[s_out_head].ip = ip;
    s_out[s_out_head].port = port;
    s_out[s_out_head].len = (uint16_t)len;
    memcpy(s_out[s_out_head].data, data, len);
    s_out_head = next;
}

void mc_hal_fill_status(wcnc_status_report_t *r)
{
    uint8_t lim = mc_hal_limit_inputs();
    r->limit_switches = lim;
    r->home_switches = lim;
    r->probe_state = pc_probe_active() ? 1 : 0;
    r->estop_input = m.force_estop ? 1 : 0;
    r->flags |= WCNC_STATUS_WIFI_CONNECTED;
    r->uptime_ms = (uint32_t)((pc_now_us - s_boot_us) / 1000u);
    r->misc_inputs = (uint8_t)(m.force_misc_in & 0x0F);
    r->io_inputs = (uint16_t)m.force_io_in;
    if (spt_ready()) {
        uint32_t rpm = (spt_rpm_milli() + 500u) / 1000u;
        r->spindle_rpm = (uint16_t)(rpm > 65535u ? 65535u : rpm);
        r->spindle_position = spt_position16(pc_now_us);
        r->spindle_index_count = spt_index_count();
    }
}

static uint8_t capabilities(void)
{
    if (m.io_module) return WCNC_CAP_IO_MODULE | WCNC_CAP_STREAM_V2;
    uint8_t caps = WCNC_CAP_CHARGE_PUMP | WCNC_CAP_MISC_OUTPUTS | WCNC_CAP_MISC_INPUTS | WCNC_CAP_SPINDLE_PWM;
    if (m.fw_version >= WCNC_MAKE_VERSION(1, 2, 0)) caps |= WCNC_CAP_STREAM_V2;
    if (m.encoder_ppr) caps |= WCNC_CAP_SPINDLE_ENCODER;
    if (spt_can_sync() && m.fw_version >= WCNC_MAKE_VERSION(1, 2, 0)) caps |= WCNC_CAP_SPINDLE_SYNC;
    return caps;
}

static const char *device_name(void)
{
    return m.io_module ? "Tiggy I/O Module (firmware sim)" : "Tiggy (firmware sim)";
}

void mc_hal_identity(uint8_t *caps, uint8_t mac[6], char name[WCNC_DEVICE_NAME_LEN])
{
    static const uint8_t fake[6] = { 0x02, 0x00, 'F', 'W', 'S', 1 };
    *caps = capabilities();
    memcpy(mac, fake, 6);
    snprintf(name, WCNC_DEVICE_NAME_LEN, "%s", device_name());
}

/* Old-form segments: a small queue standing in for the firmware's planner buffer */
static wcnc_motion_segment_t s_legacy[64];
static int s_leg_head = 0, s_leg_tail = 0;

bool mc_hal_legacy_pop(wcnc_motion_segment_t *seg)
{
    if (s_leg_tail == s_leg_head) return false;
    *seg = s_legacy[s_leg_tail];
    s_leg_tail = (s_leg_tail + 1) % 64;
    return true;
}

bool mc_hal_legacy_push(const wcnc_motion_segment_t *seg)
{
    int next = (s_leg_head + 1) % 64;
    if (next == s_leg_tail) return false;
    s_legacy[s_leg_head] = *seg;
    s_leg_head = next;
    return true;
}

void mc_hal_legacy_clear(void) { s_leg_head = s_leg_tail = 0; }

void mc_hal_log(const char *text)
{
    size_t n = strlen(text);
    if (m.log_len + n + 2 >= sizeof(m.log)) m.log_len = 0;
    memcpy(m.log + m.log_len, text, n);
    m.log_len += n;
    m.log[m.log_len++] = '\n';
    m.log[m.log_len] = '\0';
}

/* ===================================================================
 * Exported to the Python front end
 * =================================================================== */

EXPORT void fws_init(int io_module, int encoder_ppr, uint32_t fw_version)
{
    memset(&m, 0, sizeof(m));
    m.io_module = io_module != 0;
    m.encoder_ppr = encoder_ppr;
    m.fw_version = fw_version ? fw_version : WCNC_FIRMWARE_VERSION;
    for (int i = 0; i < AXES; i++) {
        m.lim_lo[i] = -4000.0;
        m.lim_hi[i] = 200000.0;
        m.min_gap_us[i] = 0xFFFFFFFFu;
    }
    pc_alarm_set = false;
    s_out_head = s_out_tail = 0;
    s_leg_head = s_leg_tail = 0;
    memset(&sp, 0, sizeof(sp));
    sp.at = pc_now_us;
    sp.next_sample = sp.at + SPT_SAMPLE_US;
    sp.follow = true;
    sp.accel = 3000.0;
    sp.index_at = 0.37;
    sp.rnd = 12345u;
    s_trace_n = 0;
    s_trace_axis = -1;
    s_restart_wanted = s_restart_pending = false;
    s_boot_us = pc_now_us;
    s_sensor_wired = encoder_ppr > 0;
    cfg_defaults();
    cfg_find(WCNC_CFG_ENCODER_PPR)->u = (uint32_t)encoder_ppr;
    apply_layout();
    se_init();
    apply_timing();
    mc_init();
}

/* Advance the clock to now_us, running the step interrupt, the 1 ms tick
 * and the spindle at the moments they fall due. */
EXPORT void fws_run(uint64_t now_us)
{
    for (;;) {
        uint64_t next = s_last_tick_us + 1000u;
        int what = 0;                       /* 0 tick, 1 step interrupt, 2 spindle */
        if (pc_alarm_set && pc_alarm_us <= next) {
            next = pc_alarm_us;
            what = 1;
        }
        if (sp.at + SPIN_STEP_US <= next) {
            next = sp.at + SPIN_STEP_US;
            what = 2;
        }
        if (next > now_us) break;
        if (pc_now_us < next) pc_now_us = next;
        if (what == 1) {
            pc_alarm_set = false;
            pc_now_us += 2;                 /* interrupt entry */
            m.isr_calls++;
            se_isr();
        } else if (what == 2) {
            spin_step();
        } else {
            s_last_tick_us += 1000u;
            mc_tick();
        }
    }
    if (pc_now_us < now_us) pc_now_us = now_us;
}

/* The spindle sensor: mode 1 = quadrature encoder of 'ppr' lines (index:
 * with an index pulse), 2 = a sensor giving 'ppr' pulses per revolution.
 * ppr 0 = none. */
EXPORT void fws_encoder(int mode, int ppr, int index)
{
    s_sensor_wired = ppr > 0;
    cfg_find(WCNC_CFG_ENCODER_PPR)->u = (uint32_t)ppr;
    cfg_find(WCNC_CFG_ENCODER_MODE)->u = mode == 2 ? 1u : (index ? 0u : 2u);
    apply_layout();
}

/* True (once) when a setting that a controller reads at start-up has been
 * changed and saved: the real one restarts then, and so should whoever runs
 * this one - by dropping the host's connection. */
EXPORT int fws_restart_wanted(void)
{
    bool wanted = s_restart_wanted;
    s_restart_wanted = false;
    return wanted ? 1 : 0;
}

/* The spindle's speed: follow the spindle output (follow != 0) or turn at
 * 'rpm' (negative = reverse), changing by at most 'accel' rpm per second,
 * with a ripple of 'ripple' (fraction of the speed) at 'ripple_hz'. */
EXPORT void fws_spindle(int follow, double rpm, double accel, double ripple, double ripple_hz)
{
    sp.follow = follow != 0;
    sp.forced = rpm;
    if (accel > 0.0) sp.accel = accel;
    sp.ripple = ripple;
    sp.ripple_hz = ripple_hz;
}

/* Record the steps of one axis (-1: none) with the spindle angle at each. */
EXPORT void fws_trace(int axis)
{
    s_trace_axis = axis;
    s_trace_n = 0;
}

EXPORT int fws_trace_read(double *t, double *turns, int32_t *pos, int max)
{
    int n = s_trace_n < max ? s_trace_n : max;
    for (int i = 0; i < n; i++) {
        t[i] = s_trace[i].t;
        turns[i] = s_trace[i].turns;
        pos[i] = s_trace[i].pos;
    }
    return n;
}

EXPORT uint64_t fws_now(void) { return pc_now_us; }

EXPORT void fws_udp(const uint8_t *data, int len, uint32_t ip, uint16_t port)
{
    mc_on_udp(data, (size_t)len, ip, port);
}

EXPORT void fws_tcp_session(uint32_t ip) { mc_tcp_session(ip); }

/* One TCP frame in, at most one frame out.  Returns the reply length. */
EXPORT int fws_tcp(const uint8_t *data, int len, uint8_t *resp, int resp_size)
{
    static uint32_t seq = 0;
    if (len < (int)sizeof(wcnc_header_t) || resp_size < 128) return 0;
    uint8_t copy[512];
    if (len > (int)sizeof(copy)) return 0;
    memcpy(copy, data, (size_t)len);
    wcnc_header_t hdr;
    memcpy(&hdr, copy, sizeof(hdr));
    if (len < (int)(sizeof(hdr) + hdr.payload_length) ||
        !wcnc_validate_packet(copy, sizeof(hdr) + hdr.payload_length))
        return 0;
    uint32_t ts = (uint32_t)(pc_now_us / 1000u);

    switch (hdr.packet_type) {
    case WCNC_PKT_HANDSHAKE_REQ: {
        wcnc_handshake_resp_t r;
        memset(&r, 0, sizeof(r));
        r.firmware_version = m.fw_version;
        r.num_axes = AXES;
        r.capabilities = capabilities();
        r.buffer_capacity = SE_QUEUE_SIZE - 1;
        r.max_step_rate = mc_max_step_rate();
        snprintf(r.device_name, sizeof(r.device_name), "%s", device_name());
        r.device_mode = m.io_module ? 1 : 0;
        r.encoder_ppr_hi = (uint8_t)(m.encoder_ppr >> 8);
        r.encoder_ppr_lo = (uint8_t)m.encoder_ppr;
        r.io_channel_count = m.io_module ? 16 : 0;
        wcnc_finalize_packet(&r, WCNC_PKT_HANDSHAKE_RESP, sizeof(r) - sizeof(wcnc_header_t), seq++, ts);
        memcpy(resp, &r, sizeof(r));
        return (int)sizeof(r);
    }
    case WCNC_PKT_PING: {
        wcnc_ping_packet_t p;
        memset(&p, 0, sizeof(p));
        if (len >= (int)sizeof(p)) memcpy(&p.ping_id, copy + sizeof(hdr), 4);
        wcnc_finalize_packet(&p, WCNC_PKT_PONG, sizeof(p) - sizeof(wcnc_header_t), seq++, ts);
        memcpy(resp, &p, sizeof(p));
        return (int)sizeof(p);
    }
    case WCNC_PKT_CONFIG_GET: {
        if (len < (int)sizeof(wcnc_config_packet_t)) return 0;
        wcnc_config_packet_t q, r;
        memcpy(&q, copy, sizeof(q));
        memset(&r, 0, sizeof(r));
        r.key = q.key;
        cfg_item_t *c = cfg_find(q.key);
        if (c) {
            r.value_type = c->type;
            if (c->type == WCNC_VAL_FLOAT) memcpy(r.value, &c->f, 4);
            else if (c->type == WCNC_VAL_STRING) memcpy(r.value, c->s, sizeof(r.value));
            else memcpy(r.value, &c->u, 4);
        } else {
            r.value_type = WCNC_VAL_UINT8;
            r.value[0] = (q.key >= 0x0500 && q.key < 0x0600) ? 0xFF : 0;
        }
        wcnc_finalize_packet(&r, WCNC_PKT_CONFIG_RESP, sizeof(r) - sizeof(wcnc_header_t), seq++, ts);
        memcpy(resp, &r, sizeof(r));
        return (int)sizeof(r);
    }
    case WCNC_PKT_CONFIG_SET:
        if (len >= (int)sizeof(wcnc_config_packet_t)) {
            wcnc_config_packet_t q;
            memcpy(&q, copy, sizeof(q));
            cfg_item_t *c = cfg_find(q.key);
            if (!c && s_cfg_count < (int)(sizeof(s_cfg) / sizeof(s_cfg[0]))) {
                c = &s_cfg[s_cfg_count++];
                memset(c, 0, sizeof(*c));
                c->key = q.key;
                c->type = q.value_type;
            }
            if (c && c->type == q.value_type) {
                uint32_t before = c->u;
                c->u = 0;
                if (c->type == WCNC_VAL_FLOAT) memcpy(&c->f, q.value, 4);
                else if (c->type == WCNC_VAL_STRING) memcpy(c->s, q.value, sizeof(c->s));
                else if (c->type == WCNC_VAL_UINT8) c->u = q.value[0];
                else if (c->type == WCNC_VAL_UINT16) memcpy(&c->u, q.value, 2);
                else memcpy(&c->u, q.value, 4);
                apply_timing();
                mc_config_changed();
                if (c->u != before && (q.key == WCNC_CFG_MACHINE_LAYOUT || q.key == WCNC_CFG_ENCODER_PPR ||
                                       q.key == WCNC_CFG_ENCODER_MODE))
                    s_restart_pending = true;
            }
        }
        {
            wcnc_ack_packet_t a;
            memset(&a, 0, sizeof(a));
            a.acked_sequence = hdr.sequence;
            a.acked_type = hdr.packet_type;
            wcnc_finalize_packet(&a, WCNC_PKT_ACK, sizeof(a) - sizeof(wcnc_header_t), seq++, ts);
            memcpy(resp, &a, sizeof(a));
            return (int)sizeof(a);
        }
    case WCNC_PKT_CONFIG_SAVE: {
        if (s_restart_pending && !mc_busy()) {
            /* What a restart does: the settings read at start-up take effect,
             * and everything else begins again - the step counters and the
             * clock at zero, nothing homed, outputs off.  (The machine itself,
             * and its spindle, stay where they are.) */
            s_restart_pending = false;
            s_restart_wanted = true;
            apply_layout();
            pc_alarm_set = false;
            se_init();
            apply_timing();
            mc_init();
            mc_config_changed();
            s_boot_us = pc_now_us;
        }
        wcnc_ack_packet_t a;
        memset(&a, 0, sizeof(a));
        a.acked_sequence = hdr.sequence;
        a.acked_type = hdr.packet_type;
        wcnc_finalize_packet(&a, WCNC_PKT_ACK, sizeof(a) - sizeof(wcnc_header_t), seq++, ts);
        memcpy(resp, &a, sizeof(a));
        return (int)sizeof(a);
    }
    default:
        return 0;
    }
}

/* Next datagram the firmware wants sent.  Returns its length, 0 if none. */
EXPORT int fws_outbox(uint8_t *buf, int size, uint32_t *ip, uint16_t *port)
{
    if (s_out_tail == s_out_head) return 0;
    dgram_t *d = &s_out[s_out_tail];
    s_out_tail = (s_out_tail + 1) % 64;
    if (d->len > size) return 0;
    memcpy(buf, d->data, d->len);
    *ip = d->ip;
    *port = d->port;
    return d->len;
}

EXPORT void fws_set_inputs(int estop, int probe, int limits, int misc_in, int io_in)
{
    m.force_estop = estop;
    m.force_probe = probe;
    m.force_limits = limits;
    m.force_misc_in = misc_in;
    m.force_io_in = io_in;
}

EXPORT void fws_set_limits(const double lo[AXES], const double hi[AXES])
{
    for (int i = 0; i < AXES; i++) { m.lim_lo[i] = lo[i]; m.lim_hi[i] = hi[i]; }
}

EXPORT void fws_set_probe_z(int valid, double z)
{
    m.probe_z_valid = valid != 0;
    m.probe_z = z;
}

EXPORT void fws_set_phys(const double phys[AXES])
{
    for (int i = 0; i < AXES; i++) m.phys[i] = (int64_t)phys[i];
}

typedef struct {
    double   phys[AXES];
    int32_t  pos[AXES];
    int32_t  state, alarm;
    int32_t  misc, spindle, rpm, coolant, charge_pump, drivers, io;
    double   duty;
    uint32_t queued_us, queue_count;
    uint32_t min_gap_us[AXES];
    uint64_t steps[AXES];
    uint64_t isr_calls;
    uint32_t feed;
    int32_t  busy;
    int32_t  sync;                  /* 0 by the clock, 1 waiting for the index, 2 by the spindle */
    double   spindle_turns, spindle_rpm;    /* the true spindle */
    uint32_t tracked_rpm_milli;     /* ...and what the firmware makes of it */
    uint32_t index_count;
} fws_info_t;

EXPORT void fws_get(fws_info_t *o)
{
    memset(o, 0, sizeof(*o));
    se_get_position(o->pos);
    for (int i = 0; i < AXES; i++) {
        o->phys[i] = (double)m.phys[i];
        o->min_gap_us[i] = m.min_gap_us[i];
        o->steps[i] = m.steps[i];
    }
    o->state = (int32_t)mc_state();
    o->alarm = mc_alarm();
    o->misc = m.out.misc; o->spindle = m.out.spindle; o->rpm = m.out.rpm; o->coolant = m.out.coolant;
    o->charge_pump = m.out.charge_pump; o->drivers = m.drivers; o->io = m.out.io;
    o->duty = m.duty;
    o->queued_us = se_queued_us();
    o->queue_count = se_count();
    o->isr_calls = m.isr_calls;
    o->feed = se_feed_rate();
    o->busy = mc_busy();
    o->sync = se_sync_state();
    o->spindle_turns = spin_turns_now();
    o->spindle_rpm = sp.rate * 60.0e6;
    o->tracked_rpm_milli = spt_rpm_milli();
    o->index_count = spt_index_count();
}

EXPORT void fws_clear_stats(void)
{
    for (int i = 0; i < AXES; i++) { m.min_gap_us[i] = 0xFFFFFFFFu; m.steps[i] = 0; }
    m.isr_calls = 0;
}

EXPORT uint32_t fws_cfg(uint16_t key) { return cfg_u(key); }

EXPORT int fws_log(char *buf, int size)
{
    int n = (int)m.log_len < size - 1 ? (int)m.log_len : size - 1;
    memcpy(buf, m.log, (size_t)n);
    buf[n] = '\0';
    m.log_len = 0;
    return n;
}

/* Local control, as the G-code interface uses it */
EXPORT int  fws_legacy_push(const wcnc_motion_segment_t *seg) { return mc_hal_legacy_push(seg) ? 1 : 0; }
EXPORT void fws_feed_hold(void)   { mc_feed_hold(); }
EXPORT void fws_feed_resume(void) { mc_feed_resume(); }
EXPORT int  fws_home(int mask)    { return mc_home((uint8_t)mask) ? 1 : 0; }
EXPORT void fws_estop(void)       { mc_estop(); }
EXPORT int  fws_reset(void)       { return mc_reset() ? 1 : 0; }
