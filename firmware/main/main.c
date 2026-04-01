/*
 * WiFi CNC Controller - Main Entry Point
 *
 * Initializes all subsystems and creates FreeRTOS tasks:
 *   Core 0: WiFi, UDP receive, TCP server, status reporter
 *   Core 1: Motion control (segment processing + stepper ISR)
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"

#include "config.h"
#include "pin_config.h"
#include "motion/planner.h"
#include "motion/stepper.h"
#include "motion/motion_control.h"
#include "network/wifi_manager.h"
#include "network/udp_server.h"
#include "network/tcp_server.h"
#include "io/gpio_control.h"
#include "io/status_led.h"
#include "io/spindle_encoder.h"
#include "io/io_module.h"
#include "status/status_reporter.h"
#include "persist/nvs_config.h"
#include "gcode/gcode_interface.h"
#include "gcode/gcode_telnet.h"
#include "gcode/gcode_uart.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "Tiggy Motion Controller v%u.%u.%u starting...",
             (unsigned)((WCNC_FIRMWARE_VERSION >> 24) & 0xFF),
             (unsigned)((WCNC_FIRMWARE_VERSION >> 16) & 0xFF),
             (unsigned)(WCNC_FIRMWARE_VERSION & 0xFFFF));

    /* 1. Initialize NVS (persistent configuration) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 2. Load configuration from NVS */
    nvs_config_init();
    ESP_LOGI(TAG, "Configuration loaded from NVS");

    /* 3. Initialize GPIO pins (step, dir, enable, limits, probe) */
    gpio_control_init();
    status_led_init();
    ESP_LOGI(TAG, "GPIO initialized");

    /* 4. Check device mode (motion controller vs I/O module) */
    uint8_t device_mode = nvs_config_get_u8("dev_mode", 0);
    ESP_LOGI(TAG, "Device mode: %s", device_mode == 0 ? "motion controller" : "I/O module");

    if (device_mode == 0) {
        /* Motion controller mode (default) */
        planner_init();
        ESP_LOGI(TAG, "Planner buffer initialized (%d slots)", CFG_PLANNER_BUFFER_SIZE);
        stepper_init();
        ESP_LOGI(TAG, "Stepper engine initialized");
    } else {
        /* I/O module mode: skip stepper/planner, init expanded I/O */
        io_module_init();
        ESP_LOGI(TAG, "I/O module initialized");
    }

    /* 5. Initialize spindle encoder (works in both modes) */
    if (spindle_encoder_init()) {
        ESP_LOGI(TAG, "Spindle encoder initialized");
    }

    /* 6. Initialize WiFi (handles STA connect or AP fallback) */
    wifi_manager_init();
    ESP_LOGI(TAG, "WiFi manager started");

    /* 7. Create network tasks on Core 0 */
    xTaskCreatePinnedToCore(
        udp_receive_task, "udp_rx",
        CFG_TASK_UDP_RX_STACK, NULL,
        CFG_TASK_UDP_RX_PRIORITY, NULL,
        CFG_TASK_UDP_RX_CORE);

    xTaskCreatePinnedToCore(
        tcp_server_task, "tcp_srv",
        CFG_TASK_TCP_STACK, NULL,
        CFG_TASK_TCP_PRIORITY, NULL,
        CFG_TASK_TCP_CORE);

    xTaskCreatePinnedToCore(
        status_report_task, "status",
        CFG_TASK_STATUS_STACK, NULL,
        CFG_TASK_STATUS_PRIORITY, NULL,
        CFG_TASK_STATUS_CORE);

    /* 8. Create motion control task on Core 1 (motion controller mode only) */
    if (device_mode == 0) {
        xTaskCreatePinnedToCore(
            motion_control_task, "motion",
            CFG_TASK_MOTION_STACK, NULL,
            CFG_TASK_MOTION_PRIORITY, NULL,
            CFG_TASK_MOTION_CORE);
    }

    /* 9. Initialize G-code interface (queue, parser, planner) */
    gcode_interface_init();
    ESP_LOGI(TAG, "G-code interface initialized");

    /* 10. Create G-code tasks on Core 0 */
    xTaskCreatePinnedToCore(
        gcode_exec_task, "gcode_exec",
        CFG_TASK_GCODE_EXEC_STACK, NULL,
        CFG_TASK_GCODE_EXEC_PRIORITY, NULL,
        CFG_TASK_GCODE_EXEC_CORE);

    xTaskCreatePinnedToCore(
        gcode_uart_task, "gcode_uart",
        CFG_TASK_GCODE_UART_STACK, NULL,
        CFG_TASK_GCODE_UART_PRIORITY, NULL,
        CFG_TASK_GCODE_UART_CORE);

    xTaskCreatePinnedToCore(
        gcode_telnet_task, "gcode_tel",
        CFG_TASK_GCODE_TELNET_STACK, NULL,
        CFG_TASK_GCODE_TELNET_PRIORITY, NULL,
        CFG_TASK_GCODE_TELNET_CORE);

    ESP_LOGI(TAG, "All tasks started. System ready.");

    /* 10. Main loop: watchdog feed, periodic diagnostics */
    esp_task_wdt_add(NULL);  /* Register main task with watchdog before feeding it */
    while (1) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
