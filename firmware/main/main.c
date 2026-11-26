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
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_rom_sys.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "config.h"
#include "pin_config.h"
#include "motion/planner.h"
#include "motion/stepper.h"
#include "motion/step_engine.h"
#include "motion/motion_control.h"
#include "network/wifi_manager.h"
#include "network/eth_manager.h"
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

#ifndef WCNC_BOOT_TRACE
#define WCNC_BOOT_TRACE 0
#endif

static void boot_stage(const char *stage)
{
#if WCNC_BOOT_TRACE
    esp_rom_printf("\r\n[BOOT] %s\r\n", stage);
#else
    (void)stage;
#endif
}

/* See app_main(): installs the GPIO interrupt service on the core this runs on. */
static void gpio_isr_service_task(void *arg)
{
    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK) ESP_LOGE(TAG, "GPIO interrupt service not installed: %s", esp_err_to_name(err));
    xTaskNotifyGive((TaskHandle_t)arg);
    vTaskDelete(NULL);
}

void app_main(void)
{
    boot_stage("app_main entry");
    ESP_LOGI(TAG, "Tiggy Motion Controller v%u.%u.%u starting...",
             (unsigned)((WCNC_FIRMWARE_VERSION >> 24) & 0xFF),
             (unsigned)((WCNC_FIRMWARE_VERSION >> 16) & 0xFF),
             (unsigned)(WCNC_FIRMWARE_VERSION & 0xFFFF));

    /* 1. Initialize NVS (persistent configuration) */
    boot_stage("nvs_flash_init");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        boot_stage("nvs_flash_erase");
        ESP_LOGW(TAG, "NVS partition truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 2. Load configuration from NVS */
    boot_stage("nvs_config_init");
    nvs_config_init();
    ESP_LOGI(TAG, "Configuration loaded from NVS");

    /* The GPIO interrupt service: the Ethernet chip's interrupt and the
     * spindle sensor's pulse input use it.  It belongs to the core it is
     * first asked for from, so it is installed from a task on the motion
     * core - the quiet one, where the time a pulse is noticed is not at the
     * mercy of the network - and before anything else can settle it. */
    boot_stage("gpio_isr_service");
    xTaskCreatePinnedToCore(gpio_isr_service_task, "gpio_isr", 3072, xTaskGetCurrentTaskHandle(),
                            CFG_TASK_MOTION_PRIORITY, NULL, CFG_TASK_MOTION_CORE);
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000)) == 0)
        ESP_LOGE(TAG, "GPIO interrupt service: no answer from the motion core");

    /* 3. Initialize GPIO pins (step, dir, enable, limits, probe) */
    boot_stage("gpio_control_init");
    gpio_control_init();
    boot_stage("status_led_init");
    status_led_init();
    ESP_LOGI(TAG, "GPIO initialized");

    /* 4. Check device mode (motion controller vs I/O module) */
    boot_stage("device_mode");
    uint8_t device_mode = nvs_config_get_u8("dev_mode", 0);
    ESP_LOGI(TAG, "Device mode: %s", device_mode == 0 ? "motion controller" : "I/O module");

    boot_stage("planner_init");
    planner_init();
    if (device_mode == 0) {
        /* Motion controller mode (default) */
        boot_stage("stepper_init");
        stepper_init();
        ESP_LOGI(TAG, "Stepper engine initialized");
    } else {
        /* I/O module mode: no motors, the pins are general purpose I/O */
        boot_stage("io_module_init");
        se_init();
        io_module_init();
        ESP_LOGI(TAG, "I/O module initialized");
    }
    motion_control_init();

    /* 5. Initialize the spindle sensor.  (A motion controller's: it is read
     *    by the step timer's clock, which an I/O module does not run.) */
    boot_stage("spindle_encoder_init");
    if (device_mode == 0 && spindle_encoder_init()) {
        ESP_LOGI(TAG, "Spindle encoder initialized");
    }

    /* 6. Initialize TCP/IP stack (shared by Ethernet and WiFi) */
    boot_stage("esp_netif_init");
    ESP_ERROR_CHECK(esp_netif_init());
    boot_stage("esp_event_loop_create_default");
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* 7. Try Ethernet first (W5500 SPI), fall back to WiFi if not available */
    boot_stage("eth_manager_init");
    bool eth_ok = eth_manager_init();
    if (eth_ok) {
        ESP_LOGI(TAG, "Ethernet connected - WiFi disabled");
    } else {
        boot_stage("wifi_manager_init");
        wifi_manager_init();
        ESP_LOGI(TAG, "WiFi manager started");
    }

    /* 7. Create network tasks on Core 0 */
    boot_stage("network_tasks");
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

    /* 8. Create the motion task on Core 1.  It also runs in I/O module mode:
     *    it is the one place that deals with the host (outputs, watchdog,
     *    status reports). */
    boot_stage("motion_task");
    xTaskCreatePinnedToCore(
        motion_control_task, "motion",
        CFG_TASK_MOTION_STACK, NULL,
        CFG_TASK_MOTION_PRIORITY, NULL,
        CFG_TASK_MOTION_CORE);


    /* 9. Initialize G-code interface (queue, parser, planner) */
    boot_stage("gcode_interface_init");
    gcode_interface_init();
    ESP_LOGI(TAG, "G-code interface initialized");

    /* 10. Create G-code tasks on Core 0 */
    boot_stage("gcode_tasks");
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
    boot_stage("main_loop");
    esp_task_wdt_add(NULL);  /* Register main task with watchdog before feeding it */
    while (1) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}