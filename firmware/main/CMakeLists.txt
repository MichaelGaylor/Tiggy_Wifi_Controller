# ============================================================
# Component source registration
#
# Board selection is handled by:
#   PlatformIO:  build_flags in platformio.ini (-DBOARD_*)
#   idf.py:      auto-detected in pin_config.h from IDF target
#                (esp32s3 -> DevKitC, esp32 -> WROOM32)
#                Override with: idf.py build -DBOARD_ESP32S3_ZERO
# ============================================================
idf_component_register(
    SRCS
        "main.c"
        "motion/stepper.c"
        "motion/motion_control.c"
        "motion/planner.c"
        "network/wifi_manager.c"
        "network/eth_manager.c"
        "network/udp_server.c"
        "network/tcp_server.c"
        "network/protocol_handler.c"
        "io/gpio_control.c"
        "io/status_led.c"
        "io/spindle_encoder.c"
        "io/io_module.c"
        "status/status_reporter.c"
        "persist/nvs_config.c"
        "gcode/gcode_parser.c"
        "gcode/gcode_planner.c"
        "gcode/gcode_interface.c"
        "gcode/grbl_settings.c"
        "gcode/gcode_telnet.c"
        "gcode/gcode_uart.c"
        "gcode/gcode_arcs.c"
        "gcode/gcode_jog.c"
    INCLUDE_DIRS
        "."
        "motion"
        "network"
        "io"
        "status"
        "persist"
        "gcode"
        "../../protocol"
    REQUIRES
        driver esp_driver_pcnt esp_wifi esp_eth esp_event nvs_flash esp_timer lwip esp_netif
)
