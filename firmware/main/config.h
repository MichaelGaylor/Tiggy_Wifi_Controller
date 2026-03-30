/*
 * WiFi CNC Controller - Firmware Configuration
 *
 * Compile-time defaults for the ESP32 motion controller firmware.
 * Runtime-configurable values are stored in NVS and override these defaults.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "../../protocol/wifi_cnc_protocol.h"

/* ===================================================================
 * Motion Defaults
 * =================================================================== */

#define CFG_DEFAULT_STEP_PULSE_US       3       /* Step pulse width in microseconds */
#define CFG_DEFAULT_DIR_SETUP_US        2       /* Direction pin setup time before step */
#define CFG_DEFAULT_STEP_IDLE_DELAY_MS  255     /* Disable steppers after this idle time (0=never) */

#define CFG_DEFAULT_STEPS_PER_MM        800.0f  /* Default steps/mm for all axes */
#define CFG_DEFAULT_MAX_RATE            20000   /* Default max rate: steps/sec per axis */
#define CFG_DEFAULT_ACCELERATION        5000    /* Default acceleration: steps/sec^2 */

#define CFG_MAX_AGGREGATE_STEP_RATE     250000  /* Max total steps/sec across all axes */
#define CFG_STEPPER_TIMER_RESOLUTION_HZ 1000000 /* 1 MHz timer = 1us resolution */

/* ===================================================================
 * Planner Ring Buffer
 * =================================================================== */

#define CFG_PLANNER_BUFFER_SIZE         128     /* Must be power of 2 */

/* ===================================================================
 * Network Defaults
 * =================================================================== */

#define CFG_DEFAULT_SSID                ""      /* Empty = start in AP mode */
#define CFG_DEFAULT_PASSWORD            ""
#define CFG_DEFAULT_IP_MODE             0       /* 0=DHCP, 1=static */
#define CFG_DEFAULT_STATUS_INTERVAL_MS  50      /* Status report every 50ms */

#define CFG_WIFI_RECONNECT_BASE_MS      100     /* Initial reconnect delay */
#define CFG_WIFI_RECONNECT_MAX_MS       5000    /* Max reconnect delay */
#define CFG_WIFI_STA_TIMEOUT_MS         10000   /* STA mode timeout before AP fallback */

#define CFG_AP_SSID_PREFIX              "WiFiCNC-"
#define CFG_AP_DEFAULT_PASSWORD         "12345678"
#define CFG_AP_CHANNEL                  1
#define CFG_AP_MAX_CONNECTIONS          2

#define CFG_TCP_KEEPALIVE_INTERVAL_MS   2000    /* Ping interval */
#define CFG_TCP_KEEPALIVE_TIMEOUT_MS    30000   /* No data for 30s = disconnect */
#define CFG_TCP_RECV_TIMEOUT_MS         2000    /* recv() blocks max 2s (must be < watchdog) */

#define CFG_UDP_RX_BUFFER_SIZE          512
#define CFG_TCP_RX_BUFFER_SIZE          256

/* ===================================================================
 * Homing Defaults
 * =================================================================== */

#define CFG_DEFAULT_HOMING_SEEK_RATE    5000    /* steps/sec: fast seek */
#define CFG_DEFAULT_HOMING_FEED_RATE    500     /* steps/sec: slow locate */
#define CFG_DEFAULT_HOMING_PULLOFF      100     /* steps: back off after home */
#define CFG_DEFAULT_HOMING_DIR_MASK     0x00    /* 0=negative direction per axis */

/* ===================================================================
 * I/O Defaults
 * =================================================================== */

#define CFG_DEFAULT_INVERT_STEP         0x00    /* No step pin inversion */
#define CFG_DEFAULT_INVERT_DIR          0x00    /* No direction pin inversion */
#define CFG_DEFAULT_INVERT_ENABLE       0x00    /* 0=active high, 1=active low */
#define CFG_DEFAULT_INVERT_LIMIT        0x3F    /* 1=invert (NO switches w/ pull-up default) */
#define CFG_DEFAULT_INVERT_HOME         0x3F    /* 1=invert (NO switches w/ pull-up default) */
#define CFG_DEFAULT_INVERT_ESTOP        0x00    /* 0=active low (BOOT button pull-up) */
#define CFG_DEFAULT_INVERT_PROBE        0x00    /* 0=active low (probe with pull-up) */

#define CFG_INPUT_DEBOUNCE_MS           5       /* Limit switch debounce time */

/* ===================================================================
 * Watchdog
 * =================================================================== */

#define CFG_WATCHDOG_TIMEOUT_S          5

/* ===================================================================
 * Task Priorities and Stack Sizes
 * =================================================================== */

#define CFG_TASK_MOTION_PRIORITY        20
#define CFG_TASK_MOTION_STACK           4096
#define CFG_TASK_MOTION_CORE            1

#define CFG_TASK_UDP_RX_PRIORITY        10
#define CFG_TASK_UDP_RX_STACK           4096
#define CFG_TASK_UDP_RX_CORE            0

#define CFG_TASK_TCP_PRIORITY           5
#define CFG_TASK_TCP_STACK              4096
#define CFG_TASK_TCP_CORE               0

#define CFG_TASK_STATUS_PRIORITY        3
#define CFG_TASK_STATUS_STACK           2048
#define CFG_TASK_STATUS_CORE            0

#define CFG_TASK_WIFI_PRIORITY          5
#define CFG_TASK_WIFI_STACK             4096
#define CFG_TASK_WIFI_CORE              0

/* G-Code execution task (parser + planner + segment emission) */
#define CFG_TASK_GCODE_EXEC_PRIORITY    8
#define CFG_TASK_GCODE_EXEC_STACK       6144
#define CFG_TASK_GCODE_EXEC_CORE        0

/* G-Code UART serial input (USB port) */
#define CFG_TASK_GCODE_UART_PRIORITY    7
#define CFG_TASK_GCODE_UART_STACK       3072
#define CFG_TASK_GCODE_UART_CORE        0

/* G-Code telnet server (WiFi, port 23) */
#define CFG_TASK_GCODE_TELNET_PRIORITY  6
#define CFG_TASK_GCODE_TELNET_STACK     4096
#define CFG_TASK_GCODE_TELNET_CORE      0

#endif /* CONFIG_H */
