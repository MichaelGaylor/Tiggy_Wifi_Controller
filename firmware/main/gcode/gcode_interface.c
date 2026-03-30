/*
 * WiFi CNC Controller - G-Code Interface
 *
 * Central execution task: receives G-code lines, parses, executes,
 * feeds the look-ahead planner, and sends responses.
 */

#include "gcode_interface.h"
#include "gcode_parser.h"
#include "gcode_planner.h"
#include "gcode_arcs.h"
#include "gcode_jog.h"
#include "grbl_settings.h"
#include "motion/stepper.h"
#include "motion/motion_control.h"
#include "motion/planner.h"
#include "io/gpio_control.h"
#include "io/spindle_encoder.h"
#include "persist/nvs_config.h"
#include "config.h"
#include "../../protocol/wifi_cnc_protocol.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "gcode";

/* ===================================================================
 * Module State
 * =================================================================== */

static QueueHandle_t s_line_queue;
static gcode_state_t s_state;
static gcode_planner_t s_planner;

static gcode_output_fn s_output_fn;
static void *s_output_ctx;

static volatile bool s_active;          /* G-code mode active */
static volatile bool s_reset_pending;   /* Soft reset requested */

/* WCO report interval (send every N status queries) */
#define WCO_REPORT_INTERVAL  10
static int s_wco_counter;

/* ===================================================================
 * Output Helpers
 * =================================================================== */

static void send_string(const char *str)
{
    if (s_output_fn) {
        s_output_fn(str, strlen(str), s_output_ctx);
    }
}

static void send_ok(void)
{
    send_string("ok\r\n");
}

static void send_error(gcode_error_t err)
{
    char buf[24];
    snprintf(buf, sizeof(buf), "error:%d\r\n", (int)err);
    send_string(buf);
}

static void send_alarm(int code)
{
    char buf[24];
    snprintf(buf, sizeof(buf), "ALARM:%d\r\n", code);
    send_string(buf);
}

/* ===================================================================
 * Coordinate Math
 * =================================================================== */

/* Compute absolute machine-coordinate target from block + modal state */
static void compute_target(const gcode_block_t *block,
                            const gcode_state_t *state,
                            float target[GCODE_MAX_AXES],
                            bool use_machine_coords)
{
    /* Current WCS offset */
    const float *wcs = state->wcs[state->wcs_index];

    for (int i = 0; i < GCODE_MAX_AXES; i++) {
        /* Axis word bit: WORD_X=bit10, WORD_Y=bit11, etc. */
        uint32_t axis_bit = WORD_X << i;

        if (block->word_bits & axis_bit) {
            float val = block->values[i];

            /* Unit conversion (inches -> mm) */
            if (state->inches) val *= 25.4f;

            if (use_machine_coords) {
                /* G53: value is in machine coordinates directly */
                if (state->incremental) {
                    target[i] = state->position[i] + val;
                } else {
                    target[i] = val;
                }
            } else if (state->incremental) {
                target[i] = state->position[i] + val;
            } else {
                /* Absolute: value is in work coordinates */
                target[i] = val + wcs[i] + state->g92_offset[i];
                /* Add TLO to Z axis (axis 2) */
                if (i == 2 && state->tlo_active) {
                    target[i] += state->tlo;
                }
            }
        } else {
            /* No axis word: target = current position */
            target[i] = state->position[i];
        }
    }
}

/* ===================================================================
 * M-Code Execution
 * =================================================================== */

static void execute_spindle(int mode, float speed)
{
    if (mode == 5 || mode == 0) {
        gpio_control_set_spindle(false);
    } else {
        gpio_control_set_spindle(true);
        /* TODO: set spindle speed PWM when implemented */
    }
}

/* ===================================================================
 * Non-Modal Command Execution
 * =================================================================== */

static gcode_error_t execute_non_modal(const gcode_block_t *block,
                                        gcode_state_t *state)
{
    switch (block->non_modal_command) {
    case 4: {
        /* G4 dwell */
        float seconds = block->p_value;
        if (state->inches) {
            /* GRBL treats P as seconds regardless of units mode,
             * but some implementations use ms. We use seconds. */
        }
        gcode_planner_dwell(&s_planner, seconds);
        break;
    }

    case 10: {
        /* G10 L2/L20 — set WCS offset. P0 = current WCS, P1-P6 = G54-G59 */
        int p = (int)lroundf(block->p_value);
        int wcs_idx = (p == 0) ? state->wcs_index : (p - 1);
        if (wcs_idx < 0 || wcs_idx >= GCODE_MAX_WCS) {
            return GCODE_ERR_INVALID_GCODE;
        }

        /* Flush motion first */
        gcode_planner_sync(&s_planner);
        while (stepper_is_running()) vTaskDelay(pdMS_TO_TICKS(5));

        if (block->l_value == 20) {
            /* L20: set so current position = specified value */
            for (int i = 0; i < GCODE_MAX_AXES; i++) {
                uint32_t bit = WORD_X << i;
                if (block->word_bits & bit) {
                    float val = block->values[i];
                    if (state->inches) val *= 25.4f;
                    state->wcs[wcs_idx][i] = state->position[i] - val
                                             - state->g92_offset[i];
                }
            }
        } else {
            /* L2: set offset directly */
            for (int i = 0; i < GCODE_MAX_AXES; i++) {
                uint32_t bit = WORD_X << i;
                if (block->word_bits & bit) {
                    float val = block->values[i];
                    if (state->inches) val *= 25.4f;
                    state->wcs[wcs_idx][i] = val;
                }
            }
        }
        break;
    }

    case 28: {
        /* G28 — return to G28.1 stored position via intermediate point */
        gcode_planner_sync(&s_planner);
        while (stepper_is_running()) vTaskDelay(pdMS_TO_TICKS(5));

        /* Move to intermediate point if any axis specified */
        if (block->word_bits & WORD_ANY_AXIS) {
            float intermediate[GCODE_MAX_AXES];
            compute_target(block, state, intermediate, false);
            gcode_planner_line(&s_planner, intermediate,
                               state->feed_rate > 0 ? state->feed_rate : 1000.0f,
                               true, WCNC_SEG_FLAG_RAPID);
            gcode_planner_sync(&s_planner);
            while (stepper_is_running()) vTaskDelay(pdMS_TO_TICKS(5));
        }

        /* Move to stored G28 position */
        gcode_planner_line(&s_planner, state->g28_position,
                           1000.0f, true, WCNC_SEG_FLAG_RAPID);
        break;
    }

    case 281: {
        /* G28.1 — store current position */
        memcpy(state->g28_position, state->position, sizeof(state->g28_position));
        break;
    }

    case 30: {
        /* G30 — return to G30.1 stored position */
        gcode_planner_sync(&s_planner);
        while (stepper_is_running()) vTaskDelay(pdMS_TO_TICKS(5));

        if (block->word_bits & WORD_ANY_AXIS) {
            float intermediate[GCODE_MAX_AXES];
            compute_target(block, state, intermediate, false);
            gcode_planner_line(&s_planner, intermediate,
                               state->feed_rate > 0 ? state->feed_rate : 1000.0f,
                               true, WCNC_SEG_FLAG_RAPID);
            gcode_planner_sync(&s_planner);
            while (stepper_is_running()) vTaskDelay(pdMS_TO_TICKS(5));
        }

        gcode_planner_line(&s_planner, state->g30_position,
                           1000.0f, true, WCNC_SEG_FLAG_RAPID);
        break;
    }

    case 301: {
        /* G30.1 — store current position */
        memcpy(state->g30_position, state->position, sizeof(state->g30_position));
        break;
    }

    case 92: {
        /* G92 — set coordinate offset */
        gcode_planner_sync(&s_planner);
        while (stepper_is_running()) vTaskDelay(pdMS_TO_TICKS(5));

        const float *wcs = state->wcs[state->wcs_index];
        for (int i = 0; i < GCODE_MAX_AXES; i++) {
            uint32_t bit = WORD_X << i;
            if (block->word_bits & bit) {
                float val = block->values[i];
                if (state->inches) val *= 25.4f;
                state->g92_offset[i] = state->position[i] - val - wcs[i];
            }
        }
        break;
    }

    case 921: {
        /* G92.1 — clear G92 offset */
        memset(state->g92_offset, 0, sizeof(state->g92_offset));
        break;
    }

    default:
        break;
    }

    return GCODE_OK;
}

/* ===================================================================
 * Block Execution
 * =================================================================== */

static gcode_error_t execute_block(gcode_block_t *block, gcode_state_t *state)
{
    gcode_error_t err;

    /* --- Handle M-codes first --- */
    if (block->spindle_mode >= 0) {
        float speed = (block->word_bits & WORD_S) ? block->s_value : state->spindle_speed;
        execute_spindle(block->spindle_mode, speed);
    }

    /* Coolant */
    if (block->coolant_mode >= 0) {
        /* TODO: implement coolant control when hardware ready */
    }

    /* Program flow */
    if (block->program_flow >= 0) {
        /* Sync before stop/end */
        gcode_planner_sync(&s_planner);
        while (stepper_is_running()) vTaskDelay(pdMS_TO_TICKS(5));

        if (block->program_flow == 0) {
            /* M0: pause — wait for cycle start (~) */
            send_string("[MSG:Program pause]\r\n");
            motion_feed_hold();
            /* The resume will come via real-time command */
        } else if (block->program_flow == 6) {
            /* M6: tool change — tool number already set from T word */
            ESP_LOGI(TAG, "Tool change: T%d", state->tool);
        }
        /* M2/M30: program end handled by gcode_update_state */
    }

    /* --- Handle G64/G61 path control --- */
    if (block->path_control_mode >= 0) {
        bool exact = (block->path_control_mode == 61);
        float tol = 0.0f;
        if (block->path_control_mode == 64 && (block->word_bits & WORD_P)) {
            tol = block->p_value;
            if (state->inches) tol *= 25.4f;
        }
        gcode_planner_set_path_mode(&s_planner, exact, tol);
    }

    /* --- Handle non-modal commands --- */
    if (block->non_modal_command >= 0) {
        err = execute_non_modal(block, state);
        if (err != GCODE_OK) return err;

        /* G53: use machine coords for subsequent motion on this line */
        if (block->non_modal_command == 53) {
            /* Motion will be handled below with machine coords flag */
        } else {
            /* Non-modal commands other than G53 don't have motion on same line
             * (except G28/G30 which handle their own motion above) */
            if (block->non_modal_command != 28 && block->non_modal_command != 30) {
                gcode_update_state(state, block);
                return GCODE_OK;
            }
        }
    }

    /* --- Handle motion --- */
    gcode_motion_mode_t motion = block->motion_mode_set ? block->motion_mode : state->motion_mode;

    if (motion != MOTION_NONE && motion != MOTION_CANNED_CANCEL) {
        /* Check if we have any axis words */
        if (!(block->word_bits & WORD_ANY_AXIS) && motion != MOTION_CANNED_CANCEL) {
            /* No axis words with implicit motion mode — just update state, no move */
            if (!block->motion_mode_set) {
                gcode_update_state(state, block);
                return GCODE_OK;
            }
        }

        bool use_machine = (block->non_modal_command == 53);
        float target[GCODE_MAX_AXES];
        compute_target(block, state, target, use_machine);

        /* Get feed rate */
        float feed = (block->word_bits & WORD_F) ? block->f_value : state->feed_rate;
        if (state->inches) feed *= 25.4f;

        switch (motion) {
        case MOTION_RAPID:
            gcode_planner_flush_smooth(&s_planner);
            gcode_planner_line(&s_planner, target, feed, true,
                               WCNC_SEG_FLAG_RAPID);
            break;

        case MOTION_LINEAR:
            gcode_planner_line_smooth(&s_planner, target, feed, 0);
            break;

        case MOTION_CW_ARC:
        case MOTION_CCW_ARC: {
            gcode_planner_flush_smooth(&s_planner);
            bool cw = (motion == MOTION_CW_ARC);
            bool use_r = (block->word_bits & WORD_R) != 0;
            float arc_tol = nvs_config_get_float("arc_tol",
                                                  GCODE_DEFAULT_ARC_TOLERANCE);
            gcode_arc_execute(&s_planner, state->position, target,
                              block->ijk, block->r_value, use_r, cw,
                              state->plane, feed, arc_tol);
            break;
        }

        case MOTION_PROBE_TOWARD:
        case MOTION_PROBE_TOWARD_NE:
        case MOTION_PROBE_AWAY:
        case MOTION_PROBE_AWAY_NE: {
            /* Flush pending motion before probing */
            gcode_planner_sync(&s_planner);
            while (stepper_is_running()) vTaskDelay(pdMS_TO_TICKS(5));

            /* Push single probe segment */
            gcode_planner_line(&s_planner, target, feed, false,
                               WCNC_SEG_FLAG_PROBE | WCNC_SEG_FLAG_LAST);
            gcode_planner_sync(&s_planner);
            while (stepper_is_running()) vTaskDelay(pdMS_TO_TICKS(5));

            /* Record probe position from actual stepper position */
            gcode_planner_sync_position(&s_planner);
            for (int i = 0; i < GCODE_MAX_AXES; i++) {
                state->probe_position[i] = s_planner.position_mm[i];
            }

            /* Check probe result */
            bool triggered = stepper_probe_triggered();
            state->probe_succeeded = triggered;

            /* Update position to where we actually stopped */
            memcpy(state->position, s_planner.position_mm,
                   sizeof(float) * GCODE_MAX_AXES);

            /* G38.2 / G38.4: error if probe didn't trigger */
            bool expect_contact = (motion == MOTION_PROBE_TOWARD ||
                                   motion == MOTION_PROBE_AWAY);
            if (expect_contact && !triggered) {
                send_alarm(4);  /* Probe fail alarm */
                return GCODE_ERR_PROBE_FAIL;
            }
            break;
        }

        default:
            break;
        }

        /* Update state position to target */
        memcpy(state->position, target, sizeof(target));
    }

    /* Update modal state */
    gcode_update_state(state, block);

    return GCODE_OK;
}

/* ===================================================================
 * Dollar Command Handler
 * =================================================================== */

static void handle_dollar_command(const char *line)
{
    if (line[1] == '$' && (line[2] == '\0' || line[2] == '\r' || line[2] == '\n')) {
        /* $$ — dump all settings */
        grbl_settings_dump(s_output_fn, s_output_ctx);
        send_ok();
        return;
    }

    if (line[1] == '#') {
        /* $# — dump offsets */
        grbl_offsets_dump(&s_state, s_output_fn, s_output_ctx);
        send_ok();
        return;
    }

    if (line[1] == 'I' || line[1] == 'i') {
        /* $I — build info */
        grbl_build_info(s_output_fn, s_output_ctx);
        send_ok();
        return;
    }

    if (line[1] == 'G' || line[1] == 'g') {
        /* $G — parser state */
        grbl_parser_state_dump(&s_state, s_output_fn, s_output_ctx);
        send_ok();
        return;
    }

    if (line[1] == 'H' || line[1] == 'h') {
        /* $H — homing */
        uint8_t enabled = nvs_config_get_u8("hm_en", 1);
        if (!enabled) {
            send_error(GCODE_ERR_SETTING_DISABLED);
            return;
        }
        gcode_planner_sync(&s_planner);
        while (stepper_is_running()) vTaskDelay(pdMS_TO_TICKS(5));
        motion_home(0x3F);  /* Home all axes */
        /* Wait for homing to complete */
        while (motion_is_homing()) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        gcode_planner_sync_position(&s_planner);
        memset(s_state.position, 0, sizeof(s_state.position));
        send_ok();
        return;
    }

    if (line[1] == 'X' || line[1] == 'x') {
        /* $X — kill alarm */
        if (motion_get_state() == WCNC_STATE_ALARM ||
            motion_get_state() == WCNC_STATE_ESTOP) {
            motion_reset();
            gcode_planner_sync_position(&s_planner);
        }
        send_ok();
        return;
    }

    if (line[1] == 'J' || line[1] == 'j') {
        /* $J= jog command */
        if (line[2] != '=') {
            send_error(GCODE_ERR_INVALID_STATEMENT);
            return;
        }
        gcode_error_t err = gcode_jog_execute(line + 3, &s_state, &s_planner);
        if (err != GCODE_OK) {
            send_error(err);
        } else {
            /* Flush jog block immediately — jog moves are standalone
             * (start/stop at zero speed), no look-ahead benefit */
            gcode_planner_sync(&s_planner);
            send_ok();
        }
        return;
    }

    /* $N=value — set setting */
    if (line[1] >= '0' && line[1] <= '9') {
        char *eq = strchr(line, '=');
        if (!eq) {
            /* $N — query single setting */
            int num = atoi(line + 1);
            grbl_setting_query(num, s_output_fn, s_output_ctx);
            send_ok();
            return;
        }

        int num = atoi(line + 1);
        float val = strtof(eq + 1, NULL);
        gcode_error_t err = grbl_setting_set(num, val);
        if (err != GCODE_OK) {
            send_error(err);
        } else {
            /* Reload planner config if axis settings changed */
            if (num >= 100 && num <= 125) {
                gcode_planner_reload_config(&s_planner);
            }
            send_ok();
        }
        return;
    }

    send_error(GCODE_ERR_INVALID_STATEMENT);
}

/* ===================================================================
 * Status Report
 * =================================================================== */

int gcode_format_status_report(char *buf, size_t buf_size)
{
    /* State name */
    const char *state_name;
    switch (motion_get_state()) {
    case WCNC_STATE_IDLE:    state_name = "Idle"; break;
    case WCNC_STATE_RUN:     state_name = "Run"; break;
    case WCNC_STATE_HOLD:    state_name = "Hold:0"; break;
    case WCNC_STATE_JOG:     state_name = "Jog"; break;
    case WCNC_STATE_HOMING:  state_name = "Home"; break;
    case WCNC_STATE_PROBING: state_name = "Run"; break;
    case WCNC_STATE_ALARM:   state_name = "Alarm"; break;
    case WCNC_STATE_ESTOP:   state_name = "Alarm"; break;
    default:                 state_name = "Idle"; break;
    }

    /* Machine position in mm */
    int32_t steps[GCODE_MAX_AXES];
    stepper_get_position(steps);
    float mpos[GCODE_MAX_AXES];
    for (int i = 0; i < GCODE_MAX_AXES; i++) {
        mpos[i] = (float)steps[i] / s_planner.steps_per_mm[i];
    }

    /* Feed rate in mm/min */
    int32_t feed_sps = stepper_get_feed_rate();
    float feed_mm_min = 0.0f;
    if (s_planner.steps_per_mm[0] > 0.0f) {
        feed_mm_min = ((float)feed_sps / s_planner.steps_per_mm[0]) * 60.0f;
    }

    /* Buffer status */
    uint16_t ring_avail = planner_available();
    uint8_t plan_avail = gcode_planner_available(&s_planner);

    /* Actual spindle RPM suffix (only when encoder available) */
    char rpm_str[16] = "";
    if (spindle_encoder_is_available()) {
        snprintf(rpm_str, sizeof(rpm_str), "|RPM:%u",
                 spindle_encoder_get_rpm());
    }

    int n;
    if (s_wco_counter <= 0) {
        /* Include WCO this time */
        const float *wcs = s_state.wcs[s_state.wcs_index];
        n = snprintf(buf, buf_size,
            "<%s|MPos:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f"
            "|FS:%.0f,%.0f"
            "|Bf:%u,%u"
            "%s"
            "|WCO:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f>\r\n",
            state_name,
            mpos[0], mpos[1], mpos[2], mpos[3], mpos[4], mpos[5],
            feed_mm_min, s_state.spindle_speed,
            ring_avail, plan_avail,
            rpm_str,
            wcs[0] + s_state.g92_offset[0],
            wcs[1] + s_state.g92_offset[1],
            wcs[2] + s_state.g92_offset[2],
            wcs[3] + s_state.g92_offset[3],
            wcs[4] + s_state.g92_offset[4],
            wcs[5] + s_state.g92_offset[5]);
        s_wco_counter = WCO_REPORT_INTERVAL;
    } else {
        n = snprintf(buf, buf_size,
            "<%s|MPos:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f"
            "|FS:%.0f,%.0f"
            "|Bf:%u,%u"
            "%s>\r\n",
            state_name,
            mpos[0], mpos[1], mpos[2], mpos[3], mpos[4], mpos[5],
            feed_mm_min, s_state.spindle_speed,
            ring_avail, plan_avail,
            rpm_str);
        s_wco_counter--;
    }

    return n;
}

/* ===================================================================
 * Real-Time Command Handler
 * =================================================================== */

void gcode_handle_realtime(uint8_t cmd)
{
    switch (cmd) {
    case GRBL_RT_STATUS_QUERY: {
        char buf[256];
        int n = gcode_format_status_report(buf, sizeof(buf));
        if (s_output_fn && n > 0) {
            s_output_fn(buf, n, s_output_ctx);
        }
        break;
    }

    case GRBL_RT_FEED_HOLD:
        motion_feed_hold();
        break;

    case GRBL_RT_CYCLE_START:
        motion_feed_resume();
        break;

    case GRBL_RT_SOFT_RESET:
        s_reset_pending = true;
        break;
    }
}

/* ===================================================================
 * Soft Reset Handler
 * =================================================================== */

static void perform_soft_reset(void)
{
    ESP_LOGI(TAG, "Soft reset");

    /* 1. Stop motion */
    motion_estop();

    /* 2. Clear planner buffers */
    gcode_planner_clear(&s_planner);
    planner_clear();

    /* 3. Reset state and resync position */
    motion_reset();
    gcode_planner_sync_position(&s_planner);

    /* 4. Reinit parser to defaults, preserve position */
    float pos[GCODE_MAX_AXES];
    memcpy(pos, s_planner.position_mm, sizeof(pos));
    gcode_parser_init(&s_state);
    memcpy(s_state.position, pos, sizeof(pos));

    /* 5. Flush line queue */
    char discard[GCODE_MAX_LINE_LENGTH];
    while (xQueueReceive(s_line_queue, discard, 0) == pdTRUE) {}

    s_reset_pending = false;

    /* 6. Send welcome message */
    send_string("\r\nGrbl 1.1h ['$' for help]\r\n");
}

/* ===================================================================
 * Initialization
 * =================================================================== */

void gcode_interface_init(void)
{
    s_line_queue = xQueueCreate(8, GCODE_MAX_LINE_LENGTH);
    gcode_parser_init(&s_state);
    gcode_planner_init(&s_planner);
    s_active = false;
    s_reset_pending = false;
    s_wco_counter = 0;
}

void gcode_interface_set_output(gcode_output_fn fn, void *ctx)
{
    s_output_fn = fn;
    s_output_ctx = ctx;
    s_active = (fn != NULL);
}

void gcode_interface_submit_line(const char *line)
{
    if (s_line_queue) {
        /* Copy to fixed-size buffer for queue */
        char buf[GCODE_MAX_LINE_LENGTH];
        strncpy(buf, line, GCODE_MAX_LINE_LENGTH - 1);
        buf[GCODE_MAX_LINE_LENGTH - 1] = '\0';
        xQueueSend(s_line_queue, buf, pdMS_TO_TICKS(100));
    }
}

bool gcode_is_active(void)
{
    return s_active;
}

/* ===================================================================
 * Execution Task
 * =================================================================== */

void gcode_exec_task(void *pvParameters)
{
    char line[GCODE_MAX_LINE_LENGTH];
    gcode_block_t block;

    ESP_LOGI(TAG, "G-code exec task started");

    while (1) {
        /* Check for soft reset */
        if (s_reset_pending) {
            perform_soft_reset();
        }

        /* Wait for a line from the queue */
        if (xQueueReceive(s_line_queue, line, pdMS_TO_TICKS(100)) != pdTRUE) {
            /* No line — keep position in sync */
            if (gcode_planner_is_idle(&s_planner)) {
                /* Periodically sync position from stepper when idle */
                gcode_planner_sync_position(&s_planner);
                for (int i = 0; i < GCODE_MAX_AXES; i++) {
                    s_state.position[i] = s_planner.position_mm[i];
                }
            }
            continue;
        }

        /* Trim trailing whitespace */
        size_t len = strlen(line);
        while (len > 0 && (line[len-1] == '\r' || line[len-1] == '\n'
               || line[len-1] == ' ')) {
            line[--len] = '\0';
        }

        /* Skip empty lines */
        if (len == 0) {
            send_ok();
            continue;
        }

        /* Accept % program start/end delimiter (Mach3, Fanuc, RS-274) */
        if (line[0] == '%') {
            send_ok();
            continue;
        }

        /* Check for alarm state */
        if (motion_get_state() == WCNC_STATE_ESTOP ||
            motion_get_state() == WCNC_STATE_ALARM) {
            if (line[0] != '$') {
                send_error(GCODE_ERR_ALARM_LOCK);
                continue;
            }
        }

        /* Dollar commands */
        if (line[0] == '$') {
            handle_dollar_command(line);
            continue;
        }

        /* Parse G-code line */
        gcode_error_t err = gcode_parse_line(line, &block, &s_state);
        if (err != GCODE_OK) {
            send_error(err);
            continue;
        }

        /* Validate */
        err = gcode_validate_block(&block, &s_state);
        if (err != GCODE_OK) {
            send_error(err);
            continue;
        }

        /* Execute */
        err = execute_block(&block, &s_state);
        if (err != GCODE_OK) {
            send_error(err);
            continue;
        }

        send_ok();
    }
}

/* ===================================================================
 * Binary Protocol Coexistence
 * =================================================================== */

void gcode_planner_preempt(void)
{
    s_planner.preempted = true;
}

void gcode_planner_release(void)
{
    s_planner.preempted = false;
}
