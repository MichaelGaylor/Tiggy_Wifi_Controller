/*
 * WiFi CNC Controller - GRBL Settings Compatibility Layer
 *
 * Maps GRBL $N settings to NVS keys with unit conversion.
 * $100-$105 = steps/mm, $110-$115 = max rate mm/min, $120-$125 = accel mm/sec^2
 * Internally NVS stores rates as steps/sec and accel as steps/sec^2.
 */

#include "grbl_settings.h"
#include "persist/nvs_config.h"
#include "config.h"
#include "motion/stepper.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

/* ===================================================================
 * Setting Map
 * =================================================================== */

typedef enum {
    STYPE_UINT8,
    STYPE_UINT16,
    STYPE_UINT32,
    STYPE_FLOAT,
} setting_type_t;

typedef enum {
    CONV_NONE,              /* Direct value */
    CONV_RATE_MM_TO_SPS,    /* mm/min -> steps/sec (needs steps/mm) */
    CONV_ACCEL_MM_TO_SPS2,  /* mm/sec^2 -> steps/sec^2 (needs steps/mm) */
    CONV_PULL_MM_TO_STEPS,  /* mm -> steps (needs steps/mm) */
    CONV_FEED_MM_TO_SPS,    /* mm/min -> steps/sec (uses axis 0 steps/mm) */
} conversion_t;

typedef struct {
    uint8_t      number;        /* $N */
    const char  *nvs_key;       /* NVS key name */
    setting_type_t type;
    conversion_t convert;
    int          axis;          /* -1 = not per-axis, 0-5 = axis index */
    float        default_val;
} setting_map_t;

/* Steps/mm keys for conversion lookups */
static const char *spm_keys[6] = {"spm_x","spm_y","spm_z","spm_a","spm_b","spm_c"};

static float get_steps_per_mm(int axis)
{
    if (axis < 0 || axis >= 6) return CFG_DEFAULT_STEPS_PER_MM;
    return nvs_config_get_float(spm_keys[axis], CFG_DEFAULT_STEPS_PER_MM);
}

/* Setting table */
static const setting_map_t settings[] = {
    /* Basic settings */
    {  0,  "pulse_us",  STYPE_UINT16, CONV_NONE, -1, CFG_DEFAULT_STEP_PULSE_US },
    {  1,  "idle_ms",   STYPE_UINT16, CONV_NONE, -1, CFG_DEFAULT_STEP_IDLE_DELAY_MS },
    {  2,  "inv_step",  STYPE_UINT8,  CONV_NONE, -1, CFG_DEFAULT_INVERT_STEP },
    {  3,  "inv_dir",   STYPE_UINT8,  CONV_NONE, -1, CFG_DEFAULT_INVERT_DIR },
    {  4,  "inv_en",    STYPE_UINT8,  CONV_NONE, -1, CFG_DEFAULT_INVERT_ENABLE },
    {  5,  "inv_lim",   STYPE_UINT8,  CONV_NONE, -1, CFG_DEFAULT_INVERT_LIMIT },
    {  6,  "inv_probe", STYPE_UINT8,  CONV_NONE, -1, CFG_DEFAULT_INVERT_PROBE },

    /* Junction / arc / jerk */
    { 11,  "junc_dev",  STYPE_FLOAT,  CONV_NONE, -1, 0.01f },
    { 12,  "arc_tol",   STYPE_FLOAT,  CONV_NONE, -1, 0.002f },
    { 13,  "rpt_inch",  STYPE_UINT8,  CONV_NONE, -1, 0 },
    { 40,  "jerk_max",  STYPE_FLOAT,  CONV_NONE, -1, 1000.0f },

    /* Limits / homing */
    { 20,  "soft_lim",  STYPE_UINT8,  CONV_NONE, -1, 0 },
    { 21,  "hard_lim",  STYPE_UINT8,  CONV_NONE, -1, 0 },
    { 22,  "hm_en",     STYPE_UINT8,  CONV_NONE, -1, 1 },
    { 23,  "hm_dir",    STYPE_UINT8,  CONV_NONE, -1, CFG_DEFAULT_HOMING_DIR_MASK },
    { 24,  "hm_feed",   STYPE_UINT32, CONV_FEED_MM_TO_SPS, -1, 25.0f },
    { 25,  "hm_seek",   STYPE_UINT32, CONV_FEED_MM_TO_SPS, -1, 500.0f },
    { 26,  "hm_dbnc",   STYPE_UINT16, CONV_NONE, -1, CFG_INPUT_DEBOUNCE_MS },
    { 27,  "hm_pull",   STYPE_UINT32, CONV_PULL_MM_TO_STEPS, -1, 1.0f },

    /* Status report interval */
    { 10,  "stat_ms",   STYPE_UINT16, CONV_NONE, -1, CFG_DEFAULT_STATUS_INTERVAL_MS },

    /* Per-axis steps/mm ($100-$105) */
    { 100, "spm_x",  STYPE_FLOAT, CONV_NONE, 0, CFG_DEFAULT_STEPS_PER_MM },
    { 101, "spm_y",  STYPE_FLOAT, CONV_NONE, 1, CFG_DEFAULT_STEPS_PER_MM },
    { 102, "spm_z",  STYPE_FLOAT, CONV_NONE, 2, CFG_DEFAULT_STEPS_PER_MM },
    { 103, "spm_a",  STYPE_FLOAT, CONV_NONE, 3, CFG_DEFAULT_STEPS_PER_MM },
    { 104, "spm_b",  STYPE_FLOAT, CONV_NONE, 4, CFG_DEFAULT_STEPS_PER_MM },
    { 105, "spm_c",  STYPE_FLOAT, CONV_NONE, 5, CFG_DEFAULT_STEPS_PER_MM },

    /* Per-axis max rate in mm/min ($110-$115) — stored as steps/sec */
    { 110, "rate_x", STYPE_UINT32, CONV_RATE_MM_TO_SPS, 0, 1500.0f },
    { 111, "rate_y", STYPE_UINT32, CONV_RATE_MM_TO_SPS, 1, 1500.0f },
    { 112, "rate_z", STYPE_UINT32, CONV_RATE_MM_TO_SPS, 2, 1500.0f },
    { 113, "rate_a", STYPE_UINT32, CONV_RATE_MM_TO_SPS, 3, 1500.0f },
    { 114, "rate_b", STYPE_UINT32, CONV_RATE_MM_TO_SPS, 4, 1500.0f },
    { 115, "rate_c", STYPE_UINT32, CONV_RATE_MM_TO_SPS, 5, 1500.0f },

    /* Per-axis acceleration in mm/sec^2 ($120-$125) — stored as steps/sec^2 */
    { 120, "acc_x",  STYPE_UINT32, CONV_ACCEL_MM_TO_SPS2, 0, 10.0f },
    { 121, "acc_y",  STYPE_UINT32, CONV_ACCEL_MM_TO_SPS2, 1, 10.0f },
    { 122, "acc_z",  STYPE_UINT32, CONV_ACCEL_MM_TO_SPS2, 2, 10.0f },
    { 123, "acc_a",  STYPE_UINT32, CONV_ACCEL_MM_TO_SPS2, 3, 10.0f },
    { 124, "acc_b",  STYPE_UINT32, CONV_ACCEL_MM_TO_SPS2, 4, 10.0f },
    { 125, "acc_c",  STYPE_UINT32, CONV_ACCEL_MM_TO_SPS2, 5, 10.0f },

    /* Max travel ($130-$132) — for soft limits */
    { 130, "trvl_x", STYPE_FLOAT, CONV_NONE, 0, 200.0f },
    { 131, "trvl_y", STYPE_FLOAT, CONV_NONE, 1, 200.0f },
    { 132, "trvl_z", STYPE_FLOAT, CONV_NONE, 2, 200.0f },
};

#define NUM_SETTINGS (sizeof(settings) / sizeof(settings[0]))

/* ===================================================================
 * Find Setting by Number
 * =================================================================== */

static const setting_map_t *find_setting(int number)
{
    for (size_t i = 0; i < NUM_SETTINGS; i++) {
        if (settings[i].number == number) return &settings[i];
    }
    return NULL;
}

/* ===================================================================
 * Read Setting (NVS -> user units with conversion)
 * =================================================================== */

static float read_setting_value(const setting_map_t *s)
{
    float raw;

    switch (s->type) {
    case STYPE_UINT8:
        raw = (float)nvs_config_get_u8(s->nvs_key, (uint8_t)s->default_val);
        break;
    case STYPE_UINT16:
        raw = (float)nvs_config_get_u16(s->nvs_key, (uint16_t)s->default_val);
        break;
    case STYPE_UINT32:
        raw = (float)nvs_config_get_u32(s->nvs_key, (uint32_t)s->default_val);
        break;
    case STYPE_FLOAT:
        raw = nvs_config_get_float(s->nvs_key, s->default_val);
        break;
    default:
        raw = s->default_val;
    }

    /* Convert from NVS (internal) units to GRBL (user) units */
    float spm;
    switch (s->convert) {
    case CONV_RATE_MM_TO_SPS:
        /* NVS has steps/sec, user wants mm/min */
        spm = get_steps_per_mm(s->axis);
        return (raw / spm) * 60.0f;  /* steps/sec -> mm/min */

    case CONV_ACCEL_MM_TO_SPS2:
        /* NVS has steps/sec^2, user wants mm/sec^2 */
        spm = get_steps_per_mm(s->axis);
        return raw / spm;  /* steps/sec^2 -> mm/sec^2 */

    case CONV_PULL_MM_TO_STEPS:
        /* NVS has steps, user wants mm */
        spm = get_steps_per_mm(0);  /* Use X axis */
        return raw / spm;

    case CONV_FEED_MM_TO_SPS:
        /* NVS has steps/sec, user wants mm/min */
        spm = get_steps_per_mm(0);
        return (raw / spm) * 60.0f;

    default:
        return raw;
    }
}

/* ===================================================================
 * Write Setting (user units -> NVS with conversion)
 * =================================================================== */

gcode_error_t grbl_setting_set(int number, float value)
{
    const setting_map_t *s = find_setting(number);
    if (!s) return GCODE_ERR_INVALID_STATEMENT;

    /* Validate basic ranges */
    if (number == 0 && value < 1.0f) return GCODE_ERR_STEP_PULSE_MIN;

    /* Convert from GRBL (user) units to NVS (internal) units */
    float nvs_val = value;
    float spm;

    switch (s->convert) {
    case CONV_RATE_MM_TO_SPS:
        /* User gives mm/min, NVS wants steps/sec */
        spm = get_steps_per_mm(s->axis);
        nvs_val = (value / 60.0f) * spm;
        break;

    case CONV_ACCEL_MM_TO_SPS2:
        /* User gives mm/sec^2, NVS wants steps/sec^2 */
        spm = get_steps_per_mm(s->axis);
        nvs_val = value * spm;
        break;

    case CONV_PULL_MM_TO_STEPS:
        spm = get_steps_per_mm(0);
        nvs_val = value * spm;
        break;

    case CONV_FEED_MM_TO_SPS:
        spm = get_steps_per_mm(0);
        nvs_val = (value / 60.0f) * spm;
        break;

    default:
        break;
    }

    /* Write to NVS */
    switch (s->type) {
    case STYPE_UINT8:
        nvs_config_set_u8(s->nvs_key, (uint8_t)lroundf(nvs_val));
        break;
    case STYPE_UINT16:
        nvs_config_set_u16(s->nvs_key, (uint16_t)lroundf(nvs_val));
        break;
    case STYPE_UINT32:
        nvs_config_set_u32(s->nvs_key, (uint32_t)lroundf(nvs_val));
        break;
    case STYPE_FLOAT:
        nvs_config_set_float(s->nvs_key, nvs_val);
        break;
    }

    nvs_config_commit();

    /* Apply immediately for timing/axis settings */
    if (number == 0 || number == 1) {
        stepper_apply_timing_config();
    }
    if (number >= 100 && number <= 125) {
        stepper_apply_axis_config();
    }

    return GCODE_OK;
}

/* ===================================================================
 * Dump All Settings ($$)
 * =================================================================== */

void grbl_settings_dump(grbl_output_fn out, void *ctx)
{
    char buf[64];

    for (size_t i = 0; i < NUM_SETTINGS; i++) {
        float val = read_setting_value(&settings[i]);

        int n;
        if (settings[i].type == STYPE_FLOAT || settings[i].convert != CONV_NONE) {
            n = snprintf(buf, sizeof(buf), "$%d=%.3f\r\n", settings[i].number, val);
        } else {
            n = snprintf(buf, sizeof(buf), "$%d=%d\r\n", settings[i].number, (int)val);
        }
        out(buf, n, ctx);
    }
}

/* ===================================================================
 * Query Single Setting ($N)
 * =================================================================== */

void grbl_setting_query(int number, grbl_output_fn out, void *ctx)
{
    const setting_map_t *s = find_setting(number);
    if (!s) {
        const char *err = "error:3\r\n";
        out(err, strlen(err), ctx);
        return;
    }

    char buf[64];
    float val = read_setting_value(s);
    int n;
    if (s->type == STYPE_FLOAT || s->convert != CONV_NONE) {
        n = snprintf(buf, sizeof(buf), "$%d=%.3f\r\n", number, val);
    } else {
        n = snprintf(buf, sizeof(buf), "$%d=%d\r\n", number, (int)val);
    }
    out(buf, n, ctx);
}

/* ===================================================================
 * Dump Coordinate Offsets ($#)
 * =================================================================== */

void grbl_offsets_dump(const gcode_state_t *state, grbl_output_fn out, void *ctx)
{
    char buf[128];
    int n;

    /* G54-G59 */
    for (int w = 0; w < GCODE_MAX_WCS; w++) {
        n = snprintf(buf, sizeof(buf),
            "[G%d:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]\r\n",
            54 + w,
            state->wcs[w][0], state->wcs[w][1], state->wcs[w][2],
            state->wcs[w][3], state->wcs[w][4], state->wcs[w][5]);
        out(buf, n, ctx);
    }

    /* G28 */
    n = snprintf(buf, sizeof(buf),
        "[G28:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]\r\n",
        state->g28_position[0], state->g28_position[1], state->g28_position[2],
        state->g28_position[3], state->g28_position[4], state->g28_position[5]);
    out(buf, n, ctx);

    /* G30 */
    n = snprintf(buf, sizeof(buf),
        "[G30:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]\r\n",
        state->g30_position[0], state->g30_position[1], state->g30_position[2],
        state->g30_position[3], state->g30_position[4], state->g30_position[5]);
    out(buf, n, ctx);

    /* G92 */
    n = snprintf(buf, sizeof(buf),
        "[G92:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]\r\n",
        state->g92_offset[0], state->g92_offset[1], state->g92_offset[2],
        state->g92_offset[3], state->g92_offset[4], state->g92_offset[5]);
    out(buf, n, ctx);

    /* PRB (probe position) */
    n = snprintf(buf, sizeof(buf),
        "[PRB:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f:%d]\r\n",
        state->probe_position[0], state->probe_position[1], state->probe_position[2],
        state->probe_position[3], state->probe_position[4], state->probe_position[5],
        state->probe_succeeded ? 1 : 0);
    out(buf, n, ctx);
}

/* ===================================================================
 * Build Info ($I)
 * =================================================================== */

void grbl_build_info(grbl_output_fn out, void *ctx)
{
    char buf[128];
    int n = snprintf(buf, sizeof(buf),
        "[VER:1.1h TiggyCNC ESP32-S3]\r\n"
        "[OPT:V,15,128]\r\n");
    out(buf, n, ctx);
}

/* ===================================================================
 * Parser State ($G)
 * =================================================================== */

void grbl_parser_state_dump(const gcode_state_t *state, grbl_output_fn out, void *ctx)
{
    char buf[128];
    int pos = 0;

    pos += snprintf(buf + pos, sizeof(buf) - pos, "[GC:G%d ",
                    state->motion_mode == MOTION_RAPID ? 0 :
                    state->motion_mode == MOTION_LINEAR ? 1 :
                    state->motion_mode == MOTION_CW_ARC ? 2 :
                    state->motion_mode == MOTION_CCW_ARC ? 3 : 0);

    pos += snprintf(buf + pos, sizeof(buf) - pos, "G%d ", 54 + state->wcs_index);
    pos += snprintf(buf + pos, sizeof(buf) - pos, "G%d ", state->plane);
    pos += snprintf(buf + pos, sizeof(buf) - pos, "G%d ", state->inches ? 20 : 21);
    pos += snprintf(buf + pos, sizeof(buf) - pos, "G%d ", state->incremental ? 91 : 90);
    pos += snprintf(buf + pos, sizeof(buf) - pos, "G%d ", state->inverse_time ? 93 : 94);
    pos += snprintf(buf + pos, sizeof(buf) - pos, "G%d ", state->exact_stop ? 61 : 64);

    pos += snprintf(buf + pos, sizeof(buf) - pos, "M%d ",
                    state->spindle == 3 ? 3 : state->spindle == 4 ? 4 : 5);
    pos += snprintf(buf + pos, sizeof(buf) - pos, "M%d ",
                    state->coolant == 7 ? 7 : state->coolant == 8 ? 8 : 9);

    pos += snprintf(buf + pos, sizeof(buf) - pos, "T%d F%.1f S%.0f]\r\n",
                    state->tool, state->feed_rate, state->spindle_speed);

    out(buf, pos, ctx);
}
