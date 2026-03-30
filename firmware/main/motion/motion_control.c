/*
 * WiFi CNC Controller - Motion Control Task
 *
 * This task runs on Core 1 and is the sole consumer of the planner
 * ring buffer. When the stepper engine finishes a segment, this task
 * loads the next one. It also manages machine state transitions.
 */

#include "motion_control.h"
#include "stepper.h"
#include "planner.h"
#include "../config.h"
#include "../io/gpio_control.h"
#include "../persist/nvs_config.h"
#include "../network/udp_server.h"
#include "../../../protocol/wifi_cnc_protocol.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "motion";

static volatile wcnc_machine_state_t g_state = WCNC_STATE_IDLE;
static volatile uint8_t g_homing_axes = 0;
static TaskHandle_t g_motion_task_handle = NULL;

/* ===================================================================
 * State Management
 * =================================================================== */

wcnc_machine_state_t motion_get_state(void)
{
    return g_state;
}

void motion_estop(void)
{
    stepper_estop();
    planner_clear();
    g_state = WCNC_STATE_ESTOP;
    ESP_LOGW(TAG, "Motion E-STOP");
}

void motion_reset(void)
{
    if (g_state == WCNC_STATE_ESTOP || g_state == WCNC_STATE_ALARM) {
        stepper_reset();
        planner_clear();
        g_homing_axes = 0;
        g_state = WCNC_STATE_IDLE;
        ESP_LOGI(TAG, "Motion reset to IDLE");
    }
}

void motion_feed_hold(void)
{
    if (g_state == WCNC_STATE_RUN) {
        stepper_feed_hold();
        g_state = WCNC_STATE_HOLD;
        ESP_LOGI(TAG, "Feed hold active");
    }
}

void motion_feed_resume(void)
{
    if (g_state == WCNC_STATE_HOLD) {
        stepper_feed_resume();
        g_state = WCNC_STATE_RUN;
        ESP_LOGI(TAG, "Feed resumed");
    }
}

/* ===================================================================
 * Homing
 * =================================================================== */

void motion_home(uint8_t axis_mask)
{
    if (g_state != WCNC_STATE_IDLE) {
        ESP_LOGW(TAG, "Cannot home: not idle (state=%d)", g_state);
        return;
    }

    g_homing_axes = axis_mask;
    g_state = WCNC_STATE_HOMING;
    ESP_LOGI(TAG, "Homing started: axis mask=0x%02X", axis_mask);

    /* Homing is executed as a sequence of jog-like moves toward
     * the limit switches, followed by a slow back-off and re-seek.
     * This is handled in the main loop below. */
}

bool motion_is_homing(void)
{
    return g_state == WCNC_STATE_HOMING;
}

uint8_t motion_homing_state(void)
{
    return g_homing_axes;
}

/* ===================================================================
 * Send HOME_COMPLETE notification to host
 * =================================================================== */

static uint32_t s_home_sequence = 0;

static void send_home_complete(uint8_t axis_mask, bool success)
{
    wcnc_home_complete_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.axis_mask = axis_mask;
    pkt.success = success ? 1 : 0;
    wcnc_finalize_packet(&pkt, WCNC_PKT_HOME_COMPLETE,
                          sizeof(pkt) - sizeof(wcnc_header_t),
                          s_home_sequence++,
                          (uint32_t)(esp_timer_get_time()));
    udp_send_status((const uint8_t *)&pkt, sizeof(pkt));
}

/* ===================================================================
 * Homing Sequence (runs within motion control task)
 * =================================================================== */

static void execute_homing_cycle(void)
{
    /* Save original mask before the loop clears g_homing_axes on failure */
    uint8_t original_mask = g_homing_axes;

    /* Read homing parameters from NVS (fall back to compile-time defaults) */
    uint8_t  dir_mask   = nvs_config_get_u8("hm_dir",   CFG_DEFAULT_HOMING_DIR_MASK);
    uint32_t seek_rate  = nvs_config_get_u32("hm_seek",  CFG_DEFAULT_HOMING_SEEK_RATE);
    uint32_t feed_rate  = nvs_config_get_u32("hm_feed",  CFG_DEFAULT_HOMING_FEED_RATE);
    uint32_t pulloff    = nvs_config_get_u32("hm_pull",  CFG_DEFAULT_HOMING_PULLOFF);

    /* Sanity clamps */
    if (seek_rate == 0) seek_rate = CFG_DEFAULT_HOMING_SEEK_RATE;
    if (feed_rate == 0) feed_rate = CFG_DEFAULT_HOMING_FEED_RATE;
    if (pulloff  == 0) pulloff   = CFG_DEFAULT_HOMING_PULLOFF;

    /* For each axis in the homing mask, execute:
     *   1. Fast seek toward limit switch
     *   2. Back off from switch
     *   3. Slow seek toward switch
     *   4. Set position to zero
     */
    for (int axis = 0; axis < WCNC_MAX_AXES; axis++) {
        if (!(g_homing_axes & (1 << axis))) continue;

        ESP_LOGI(TAG, "Homing axis %d: fast seek", axis);

        /* Determine homing direction: 0=negative (default), 1=positive */
        int8_t dir = (dir_mask & (1 << axis)) ? 1 : -1;

        /* Phase 1: Fast seek toward limit */
        stepper_start_jog(axis, dir, seek_rate);

        while (!gpio_control_get_limit(axis) && !stepper_is_estopped()) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        stepper_stop_jog();

        if (stepper_is_estopped()) {
            g_state = WCNC_STATE_ESTOP;
            g_homing_axes = 0;
            send_home_complete(original_mask, false);
            return;
        }

        /* Phase 2: Back off from switch */
        ESP_LOGI(TAG, "Homing axis %d: back off", axis);
        stepper_start_jog(axis, -dir, feed_rate);

        uint32_t pulloff_steps = pulloff;
        int32_t start_pos;
        {
            int32_t pos[WCNC_MAX_AXES];
            stepper_get_position(pos);
            start_pos = pos[axis];
        }

        while (!stepper_is_estopped()) {
            int32_t cur_pos[WCNC_MAX_AXES];
            stepper_get_position(cur_pos);
            uint32_t distance = (uint32_t)abs(start_pos - cur_pos[axis]);
            if (distance >= pulloff_steps) break;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        stepper_stop_jog();

        if (stepper_is_estopped()) {
            g_state = WCNC_STATE_ESTOP;
            g_homing_axes = 0;
            send_home_complete(original_mask, false);
            return;
        }

        /* Phase 3: Slow seek toward limit */
        ESP_LOGI(TAG, "Homing axis %d: slow seek", axis);
        stepper_start_jog(axis, dir, feed_rate);

        while (!gpio_control_get_limit(axis) && !stepper_is_estopped()) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        stepper_stop_jog();

        if (stepper_is_estopped()) {
            g_state = WCNC_STATE_ESTOP;
            g_homing_axes = 0;
            send_home_complete(original_mask, false);
            return;
        }

        ESP_LOGI(TAG, "Homing axis %d: complete", axis);
    }

    /* Zero all homed axes */
    stepper_zero_position();

    g_homing_axes = 0;
    g_state = WCNC_STATE_IDLE;
    ESP_LOGI(TAG, "Homing complete, positions zeroed");
    send_home_complete(original_mask, true);
}

/* ===================================================================
 * Main Motion Control Task
 * =================================================================== */

void motion_control_task(void *pvParameters)
{
    g_motion_task_handle = xTaskGetCurrentTaskHandle();
    wcnc_motion_segment_t seg;

    ESP_LOGI(TAG, "Motion control task started on Core %d", xPortGetCoreID());

    while (1) {
        /* Handle E-Stop state: wait for reset */
        if (g_state == WCNC_STATE_ESTOP || g_state == WCNC_STATE_ALARM) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* Handle homing */
        if (g_state == WCNC_STATE_HOMING) {
            execute_homing_cycle();
            continue;
        }

        /* Handle feed hold: wait for resume */
        if (g_state == WCNC_STATE_HOLD) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Normal operation: feed segments to stepper */
        if (stepper_segment_complete() || !stepper_is_running()) {
            /* Try to load next segment */
            if (planner_pop_segment(&seg)) {
                stepper_load_segment(&seg);
                g_state = WCNC_STATE_RUN;
            } else {
                /* Buffer empty */
                if (g_state == WCNC_STATE_RUN) {
                    g_state = WCNC_STATE_IDLE;
                }
                /* Wait briefly for segments to arrive */
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        } else {
            /* Stepper is still running current segment.
             * Use task notification to wake when segment completes,
             * with a timeout to check for state changes. */
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
        }
    }
}
