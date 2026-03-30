/*
 * WiFi CNC Controller - Status Reporter
 *
 * Sends wcnc_status_packet_t to the host at regular intervals.
 * Collects data from stepper, planner, GPIO, and WiFi modules.
 */

#include "status_reporter.h"
#include "../config.h"
#include "../motion/stepper.h"
#include "../motion/planner.h"
#include "../motion/motion_control.h"
#include "../network/udp_server.h"
#include "../network/wifi_manager.h"
#include "../network/tcp_server.h"
#include "../io/gpio_control.h"
#include "../io/status_led.h"
#include "../io/spindle_encoder.h"
#include "../io/io_module.h"
#include "../network/protocol_handler.h"
#include "../gcode/gcode_interface.h"
#include "../gcode/gcode_telnet.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "status";
static uint32_t s_sequence = 0;

void status_report_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "Status reporter task started on Core %d", xPortGetCoreID());

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CFG_DEFAULT_STATUS_INTERVAL_MS));

        /* Only send if a host has connected via UDP */
        if (!udp_has_host()) {
            status_led_set_state(LED_STATE_DISCONNECTED);
            status_led_tick();
            continue;
        }

        /* Build status report */
        wcnc_status_packet_t pkt;
        memset(&pkt, 0, sizeof(pkt));

        /* Position */
        stepper_get_position(pkt.report.position_steps);

        /* Buffer state */
        pkt.report.buffer_available = planner_available();
        pkt.report.buffer_total = planner_capacity();

        /* Machine state */
        pkt.report.state = (uint8_t)motion_get_state();

        /* Alarm code */
        if (stepper_is_estopped()) {
            pkt.report.alarm_code = WCNC_ALARM_ESTOP_ACTIVE;
        }

        /* I/O state */
        pkt.report.limit_switches = gpio_control_get_limit_mask();
        pkt.report.probe_state = gpio_control_get_probe() ? 1 : 0;
        pkt.report.homing_state = motion_homing_state();
        pkt.report.home_switches = gpio_control_get_home_mask();
        pkt.report.estop_input = gpio_control_get_estop() ? 1 : 0;

        /* Flags */
        pkt.report.flags = 0;
        if (wifi_is_connected()) pkt.report.flags |= WCNC_STATUS_WIFI_CONNECTED;
        if (tcp_client_connected()) pkt.report.flags |= WCNC_STATUS_HOST_CONNECTED;
        if (stepper_is_running()) pkt.report.flags |= WCNC_STATUS_MOTION_ACTIVE;
        if (motion_is_homing()) pkt.report.flags |= WCNC_STATUS_HOMING_ACTIVE;

        /* Timing */
        pkt.report.uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
        pkt.report.feed_rate = stepper_get_feed_rate();
        pkt.report.current_segment_id = 0; /* TODO: track from planner */

        /* Extended fields (v1.1) */
        pkt.report.misc_outputs = gpio_control_get_misc_output_mask();
        pkt.report.misc_inputs = gpio_control_get_misc_input_mask();
        pkt.report.spindle_state = protocol_get_spindle_state();
        pkt.report.coolant_state = protocol_get_coolant_state();

        /* Spindle encoder */
        if (spindle_encoder_is_available()) {
            pkt.report.spindle_rpm = spindle_encoder_get_rpm();
            pkt.report.spindle_position = spindle_encoder_get_position();
            pkt.report.spindle_index_count = spindle_encoder_get_index_count();
        }

        /* I/O module extended fields */
        if (io_module_is_active()) {
            pkt.report.io_inputs = io_module_get_inputs();
            pkt.report.io_outputs = io_module_get_outputs();
        }

        /* Finalize packet header with checksum */
        wcnc_finalize_packet(&pkt, WCNC_PKT_STATUS_REPORT,
                              sizeof(wcnc_status_report_t),
                              s_sequence++,
                              (uint32_t)(esp_timer_get_time()));

        /* Send via UDP */
        udp_send_status((const uint8_t *)&pkt, sizeof(pkt));

        /* Status LED color based on machine state */
        wcnc_machine_state_t state = motion_get_state();
        if (state == WCNC_STATE_RUN) {
            status_led_set_state(LED_STATE_RUNNING);
        } else if (state == WCNC_STATE_ESTOP || state == WCNC_STATE_ALARM) {
            status_led_set_state(LED_STATE_ALARM);
        } else {
            status_led_set_state(LED_STATE_CONNECTED_IDLE);
        }
        status_led_tick();
    }
}
