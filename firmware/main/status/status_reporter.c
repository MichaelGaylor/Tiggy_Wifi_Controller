/*
 * WiFi CNC Controller - Status LED task
 *
 * The status report itself is built and sent by the motion core, in step
 * with the motion it describes.  What is left here is the indicator LED.
 */

#include "status_reporter.h"
#include "../motion/motion_control.h"
#include "../motion/motion_core.h"
#include "../io/status_led.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void status_report_task(void *pvParameters)
{
    (void)pvParameters;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(50));

        wcnc_machine_state_t state = motion_get_state();
        if (state == WCNC_STATE_ESTOP || state == WCNC_STATE_ALARM) {
            status_led_set_state(LED_STATE_ALARM);
        } else if (!mc_host_present()) {
            status_led_set_state(LED_STATE_DISCONNECTED);
        } else if (state == WCNC_STATE_IDLE) {
            status_led_set_state(LED_STATE_CONNECTED_IDLE);
        } else {
            status_led_set_state(LED_STATE_RUNNING);
        }
        status_led_tick();
    }
}
