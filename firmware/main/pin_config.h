/*
 * WiFi CNC Controller - GPIO Pin Assignments
 *
 * Pin configurations for supported controller boards.
 * Select board by defining BOARD_ESP32S3_ZERO or BOARD_ESP32S3_DEVKITC
 * in sdkconfig or CMakeLists.txt.
 *
 * Default: auto-detect from IDF target.
 */

#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include "driver/gpio.h"

/* ===================================================================
 * Auto-detect board from ESP-IDF target
 *
 * Define ONE of these in CMakeLists.txt or sdkconfig:
 *   BOARD_ESP32S3_ZERO     - Waveshare ESP32-S3-Zero (FH4R2, Quad SPI)
 *   BOARD_ESP32S3_DEVKITC  - ESP32-S3-DevKitC-1 N16R8 (WROOM-1, Octal SPI)
 *   BOARD_ESP32_WROOM32    - Classic ESP32-WROOM-32 DevKit
 *
 * If none defined, auto-detect from IDF target (S3 -> DevKitC, ESP32 -> WROOM32)
 * =================================================================== */

#if !defined(BOARD_ESP32S3_ZERO) && !defined(BOARD_ESP32S3_DEVKITC) && !defined(BOARD_ESP32_WROOM32)
  #if CONFIG_IDF_TARGET_ESP32S3
    #define BOARD_ESP32S3_DEVKITC   /* Default S3 board: DevKitC (most common) */
  #elif CONFIG_IDF_TARGET_ESP32
    #define BOARD_ESP32_WROOM32
  #else
    #error "Unsupported ESP32 target. Define BOARD_ESP32S3_ZERO, BOARD_ESP32S3_DEVKITC, or BOARD_ESP32_WROOM32."
  #endif
#endif

/* ===================================================================
 * Tiggy Standard Motion Controller (Waveshare ESP32-S3-Zero)
 *
 * Compact 3-axis controller. Pin map matches tiggy_standard.pinmap.
 *
 * Chip: ESP32-S3FH4R2
 * Available GPIO: 0-21, 26-32, 38-48 (minus 22-25, 33-37 for PSRAM)
 * GPIO 0 = BOOT button (usable after boot)
 * GPIO 48 = WS2812 RGB LED
 * GPIO 43 = TX0, GPIO 44 = RX0 (UART, avoid if using serial)
 * =================================================================== */

#ifdef BOARD_ESP32S3_ZERO

/* Step outputs (3-axis only) */
#define PIN_STEP_X      GPIO_NUM_1
#define PIN_STEP_Y      GPIO_NUM_2
#define PIN_STEP_Z      GPIO_NUM_42
#define PIN_STEP_A      GPIO_NUM_NC
#define PIN_STEP_B      GPIO_NUM_NC
#define PIN_STEP_C      GPIO_NUM_NC

/* Direction outputs */
#define PIN_DIR_X       GPIO_NUM_3
#define PIN_DIR_Y       GPIO_NUM_4
#define PIN_DIR_Z       GPIO_NUM_41
#define PIN_DIR_A       GPIO_NUM_NC
#define PIN_DIR_B       GPIO_NUM_NC
#define PIN_DIR_C       GPIO_NUM_NC

/* Global enable output */
#define PIN_ENABLE      GPIO_NUM_8

/* Limit/Home switch inputs (shared pins, active low with pull-up) */
#define PIN_LIMIT_X     GPIO_NUM_5
#define PIN_LIMIT_Y     GPIO_NUM_6
#define PIN_LIMIT_Z     GPIO_NUM_7
#define PIN_LIMIT_A     GPIO_NUM_NC
#define PIN_LIMIT_B     GPIO_NUM_NC
#define PIN_LIMIT_C     GPIO_NUM_NC

/* Auxiliary I/O */
#define PIN_PROBE       GPIO_NUM_14
#define PIN_ESTOP       GPIO_NUM_13
#define PIN_SPINDLE_EN  GPIO_NUM_9
#define PIN_STATUS_LED  GPIO_NUM_48     /* Onboard WS2812 RGB LED */

/* ESP32-S3 uses GPIO matrix for all pins - no input-only restriction below GPIO 46 */
#define LIMIT_PINS_NEED_EXTERNAL_PULLUP  0

/* Charge pump and misc outputs */
#define HAS_CHARGE_PUMP     1
#define PIN_CHARGE_PUMP     GPIO_NUM_10
#define MISC_OUTPUT_COUNT   2
#define PIN_MISC_OUT_0      GPIO_NUM_11
#define PIN_MISC_OUT_1      GPIO_NUM_12
#define MISC_INPUT_COUNT    0

/* Spindle encoder (optional, configurable via NVS) */
#define PIN_ENCODER_A       GPIO_NUM_15
#define PIN_ENCODER_B       GPIO_NUM_16
#define PIN_ENCODER_INDEX   GPIO_NUM_17

/* No Ethernet on Standard board (limited GPIO) */
#define HAS_ETHERNET        0

#endif /* BOARD_ESP32S3_ZERO */

/* ===================================================================
 * Tiggy Pro Motion Controller (ESP32-S3-DevKitC-1 N16R8)
 *
 * Full 6-axis controller. Pin map matches tiggy_pro.pinmap.
 *
 * Module: ESP32-S3-WROOM-1 N16R8 (16MB Flash, 8MB PSRAM, Octal SPI)
 *
 * RESERVED (do NOT use):
 *   GPIO 19, 20         - USB D-/D+ (native USB-OTG)
 *   GPIO 33, 34, 35, 36, 37 - Octal SPI for PSRAM (N16R8 variant)
 *   GPIO 43, 44         - UART0 TX/RX (USB-to-UART bridge, serial console)
 *
 * Strapping pins (usable but noted):
 *   GPIO 0  - Boot mode + BOOT button on DevKitC (avoid for outputs)
 *   GPIO 3  - JTAG source select (harmless, used for DIR_X)
 *   GPIO 45 - VDD_SPI voltage (WROOM-1 handles internally, safe)
 *   GPIO 46 - Boot mode (WROOM-1 handles internally, safe)
 *
 * Free GPIOs: 0, 45, 46 (available for NVS overrides or W5500)
 * =================================================================== */

#ifdef BOARD_ESP32S3_DEVKITC

/* Step outputs */
#define PIN_STEP_X      GPIO_NUM_1
#define PIN_STEP_Y      GPIO_NUM_2
#define PIN_STEP_Z      GPIO_NUM_42
#define PIN_STEP_A      GPIO_NUM_41
#define PIN_STEP_B      GPIO_NUM_40
#define PIN_STEP_C      GPIO_NUM_39

/* Direction outputs */
#define PIN_DIR_X       GPIO_NUM_3
#define PIN_DIR_Y       GPIO_NUM_4
#define PIN_DIR_Z       GPIO_NUM_38
#define PIN_DIR_A       GPIO_NUM_29    /* Was 37, moved for N16R8 PSRAM */
#define PIN_DIR_B       GPIO_NUM_30    /* Was 36, moved for N16R8 PSRAM */
#define PIN_DIR_C       GPIO_NUM_31    /* Was 35, moved for N16R8 PSRAM */

/* Global enable output */
#define PIN_ENABLE      GPIO_NUM_8

/* Limit/Home switch inputs (shared pins, active low with pull-up) */
#define PIN_LIMIT_X     GPIO_NUM_5
#define PIN_LIMIT_Y     GPIO_NUM_6
#define PIN_LIMIT_Z     GPIO_NUM_7
#define PIN_LIMIT_A     GPIO_NUM_15
#define PIN_LIMIT_B     GPIO_NUM_16
#define PIN_LIMIT_C     GPIO_NUM_17

/* Auxiliary I/O */
#define PIN_PROBE       GPIO_NUM_14
#define PIN_ESTOP       GPIO_NUM_13
#define PIN_SPINDLE_EN  GPIO_NUM_9
#define PIN_STATUS_LED  GPIO_NUM_48     /* Onboard WS2812 RGB LED */

/* Charge pump and misc outputs */
#define HAS_CHARGE_PUMP     1
#define PIN_CHARGE_PUMP     GPIO_NUM_10
#define MISC_OUTPUT_COUNT   2
#define PIN_MISC_OUT_0      GPIO_NUM_11
#define PIN_MISC_OUT_1      GPIO_NUM_12
#define MISC_INPUT_COUNT    3           /* Was 4, GPIO 47 reassigned to Ethernet INT */
#define PIN_MISC_IN_0       GPIO_NUM_18
#define PIN_MISC_IN_1       GPIO_NUM_21
#define PIN_MISC_IN_2       GPIO_NUM_32    /* Was GPIO 47/33, moved for Ethernet + PSRAM */

/* Spindle encoder (optional, configurable via NVS) */
#define PIN_ENCODER_A       GPIO_NUM_26
#define PIN_ENCODER_B       GPIO_NUM_27
#define PIN_ENCODER_INDEX   GPIO_NUM_28

/* ESP32-S3 has internal pull-ups on all GPIOs */
#define LIMIT_PINS_NEED_EXTERNAL_PULLUP  0

/* W5500 Ethernet (optional, Pro board only)
 * Uses 4 GPIO pins (CS tied to GND, RESET tied to 3.3V — no GPIO needed).
 * Default: MOSI=45, MISO=46, SCLK=0, INT=47 (reassigned from MISC_IN_2).
 * All pins configurable via saved settings (Protocol Tester Pins tab). */
#define HAS_ETHERNET        1
#define PIN_ETH_MOSI        GPIO_NUM_45
#define PIN_ETH_MISO        GPIO_NUM_46
#define PIN_ETH_SCLK        GPIO_NUM_0
#define PIN_ETH_INT         GPIO_NUM_47     /* Was MISC_IN_2 — reassigned for Ethernet */
#define PIN_ETH_SPI_HOST    3           /* SPI3_HOST */

#endif /* BOARD_ESP32S3_DEVKITC */

/* ===================================================================
 * ESP32-WROOM-32 (Classic DevKit)
 *
 * Available GPIO: 0,2,4,5,12-19,21-23,25-27,32-39
 * GPIO 6-11: SPI flash (DO NOT USE)
 * GPIO 34-39: Input only (no output, no internal pull-up)
 * GPIO 0,2,12,15: Strapping pins (affect boot mode)
 * ADC2 pins (0,2,4,12-15,25-27) cannot do analog reads with WiFi
 *   but digital GPIO works fine.
 * =================================================================== */

#ifdef BOARD_ESP32_WROOM32

#define PIN_STEP_X      GPIO_NUM_25
#define PIN_STEP_Y      GPIO_NUM_26
#define PIN_STEP_Z      GPIO_NUM_27
#define PIN_STEP_A      GPIO_NUM_14
#define PIN_STEP_B      GPIO_NUM_12    /* Strapping: must be LOW at boot */
#define PIN_STEP_C      GPIO_NUM_13

#define PIN_DIR_X       GPIO_NUM_33
#define PIN_DIR_Y       GPIO_NUM_32
#define PIN_DIR_Z       GPIO_NUM_22
#define PIN_DIR_A       GPIO_NUM_21
#define PIN_DIR_B       GPIO_NUM_19
#define PIN_DIR_C       GPIO_NUM_18

#define PIN_ENABLE      GPIO_NUM_23

#define PIN_LIMIT_X     GPIO_NUM_36    /* Input only - needs external pullup */
#define PIN_LIMIT_Y     GPIO_NUM_39    /* Input only - needs external pullup */
#define PIN_LIMIT_Z     GPIO_NUM_34    /* Input only - needs external pullup */
#define PIN_LIMIT_A     GPIO_NUM_35    /* Input only - needs external pullup */
#define PIN_LIMIT_B     GPIO_NUM_4
#define PIN_LIMIT_C     GPIO_NUM_2     /* Strapping; also onboard LED */

#define PIN_PROBE       GPIO_NUM_15    /* Strapping pin (OK after boot) */
#define PIN_ESTOP       GPIO_NUM_0     /* BOOT button - usable as E-Stop after boot */
#define PIN_SPINDLE_EN  GPIO_NUM_5     /* Strapping pin (OK after boot) */
#define PIN_STATUS_LED  GPIO_NUM_2     /* Shared with LIMIT_C on devkits */

/* GPIO 34-39 have no internal pull-up on classic ESP32 */
#define LIMIT_PINS_NEED_EXTERNAL_PULLUP  1

/* No spare GPIO for misc I/O on WROOM-32 */
#define HAS_CHARGE_PUMP     0
#define MISC_OUTPUT_COUNT   0
#define MISC_INPUT_COUNT    0

/* Spindle encoder — no default pins (limited GPIO), configure via NVS */
#define PIN_ENCODER_A       GPIO_NUM_NC
#define PIN_ENCODER_B       GPIO_NUM_NC
#define PIN_ENCODER_INDEX   GPIO_NUM_NC

/* No Ethernet on Classic board (limited GPIO) */
#define HAS_ETHERNET        0

#endif /* BOARD_ESP32_WROOM32 */

/* ===================================================================
 * Pin Arrays (for iteration in init code)
 * =================================================================== */

#define STEP_PINS   { PIN_STEP_X, PIN_STEP_Y, PIN_STEP_Z, PIN_STEP_A, PIN_STEP_B, PIN_STEP_C }
#define DIR_PINS    { PIN_DIR_X, PIN_DIR_Y, PIN_DIR_Z, PIN_DIR_A, PIN_DIR_B, PIN_DIR_C }
#define LIMIT_PINS  { PIN_LIMIT_X, PIN_LIMIT_Y, PIN_LIMIT_Z, PIN_LIMIT_A, PIN_LIMIT_B, PIN_LIMIT_C }

#endif /* PIN_CONFIG_H */
