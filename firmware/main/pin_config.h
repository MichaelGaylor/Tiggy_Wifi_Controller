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
 *   BOARD_ESP32S3_ZERO            - Waveshare ESP32-S3-Zero (FH4R2)
 *   BOARD_ESP32S3_DEVKITC         - Generic ESP32-S3-DevKitC family
 *   BOARD_ESP32S3_DEVKITC_QUAD    - DevKitC quad-memory variants
 *                                   (ESP32-S3-WROOM-1-N8 / N8R2)
 *   BOARD_ESP32S3_DEVKITC_OCTAL   - DevKitC octal-memory variants
 *                                   (ESP32-S3-WROOM-1-N8R8,
 *                                    ESP32-S3-WROOM-2-N16R8V / N32R8V)
 *   BOARD_ESP32_WROOM32           - Classic ESP32-WROOM-32 DevKit
 *
 * If none defined, auto-detect from IDF target
 * (S3 -> DevKitC octal by default, ESP32 -> WROOM32).
 * =================================================================== */

#if defined(BOARD_ESP32S3_DEVKITC_QUAD) && defined(BOARD_ESP32S3_DEVKITC_OCTAL)
  #error "Define only one DevKitC memory variant: QUAD or OCTAL."
#endif

#if (defined(BOARD_ESP32S3_DEVKITC_QUAD) || defined(BOARD_ESP32S3_DEVKITC_OCTAL)) && !defined(BOARD_ESP32S3_DEVKITC)
  #define BOARD_ESP32S3_DEVKITC
#endif

#if !defined(BOARD_ESP32S3_ZERO) && !defined(BOARD_ESP32S3_DEVKITC) && \
    !defined(BOARD_ESP32S3_DEVKITC_QUAD) && !defined(BOARD_ESP32S3_DEVKITC_OCTAL) && \
    !defined(BOARD_ESP32_WROOM32)
  #if CONFIG_IDF_TARGET_ESP32S3
    #define BOARD_ESP32S3_DEVKITC
    #define BOARD_ESP32S3_DEVKITC_OCTAL
  #elif CONFIG_IDF_TARGET_ESP32
    #define BOARD_ESP32_WROOM32
  #else
    #error "Unsupported ESP32 target. Define BOARD_ESP32S3_ZERO, BOARD_ESP32S3_DEVKITC_[QUAD|OCTAL], or BOARD_ESP32_WROOM32."
  #endif
#endif

#if defined(BOARD_ESP32S3_DEVKITC) && !defined(BOARD_ESP32S3_DEVKITC_QUAD) && !defined(BOARD_ESP32S3_DEVKITC_OCTAL)
  #define BOARD_ESP32S3_DEVKITC_OCTAL
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
#define PIN_STATUS_LED  GPIO_NUM_48

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
 * Tiggy Pro Motion Controller (ESP32-S3-DevKitC-1 Quad memory variants)
 *
 * Full 6-axis controller for quad-memory DevKitC modules.
 * Tested target family: ESP32-S3-WROOM-1-N8 / N8R2.
 *
 * Reserved (do NOT use):
 *   GPIO 19, 20         - USB D-/D+ (native USB-OTG)
 *   GPIO 26..32         - SPI0/1 for in-package flash / PSRAM
 *   GPIO 43, 44         - UART0 TX/RX (USB-to-UART bridge)
 *
 * Notes:
 *   GPIO 0, 3, 45, 46 are strapping pins.
 *   GPIO 33..37 are available on quad-memory variants.
 * =================================================================== */

#if defined(BOARD_ESP32S3_DEVKITC_QUAD)

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
#define PIN_DIR_A       GPIO_NUM_33
#define PIN_DIR_B       GPIO_NUM_34
#define PIN_DIR_C       GPIO_NUM_35

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
#define PIN_STATUS_LED  GPIO_NUM_48

/* Charge pump and misc outputs */
#define HAS_CHARGE_PUMP     1
#define PIN_CHARGE_PUMP     GPIO_NUM_10
#define MISC_OUTPUT_COUNT   2
#define PIN_MISC_OUT_0      GPIO_NUM_11
#define PIN_MISC_OUT_1      GPIO_NUM_12
#define MISC_INPUT_COUNT    3
#define PIN_MISC_IN_0       GPIO_NUM_18
#define PIN_MISC_IN_1       GPIO_NUM_21
#define PIN_MISC_IN_2       GPIO_NUM_36

/* Spindle encoder disabled by default to preserve free GPIOs */
#define PIN_ENCODER_A       GPIO_NUM_NC
#define PIN_ENCODER_B       GPIO_NUM_NC
#define PIN_ENCODER_INDEX   GPIO_NUM_NC

/* ESP32-S3 has internal pull-ups on all GPIOs */
#define LIMIT_PINS_NEED_EXTERNAL_PULLUP  0

/* W5500 Ethernet (optional) */
#define HAS_ETHERNET        1
#define PIN_ETH_MOSI        GPIO_NUM_45
#define PIN_ETH_MISO        GPIO_NUM_46
#define PIN_ETH_SCLK        GPIO_NUM_0
#define PIN_ETH_INT         GPIO_NUM_47
#define PIN_ETH_SPI_HOST    2

#endif /* BOARD_ESP32S3_DEVKITC_QUAD */

/* ===================================================================
 * Tiggy Pro Motion Controller (ESP32-S3-DevKitC-1 Octal memory variants)
 *
 * Full 6-axis controller for octal-memory DevKitC modules.
 * Tested target family: ESP32-S3-WROOM-1-N8R8,
 *                      ESP32-S3-WROOM-2-N16R8V / N32R8V.
 *
 * Reserved (do NOT use):
 *   GPIO 19, 20         - USB D-/D+ (native USB-OTG)
 *   GPIO 26..37         - SPI0/1 for in-package flash / PSRAM
 *   GPIO 43, 44         - UART0 TX/RX (USB-to-UART bridge)
 *
 * Notes:
 *   GPIO 0, 3, 45, 46 are strapping pins.
 *   Default map trades charge pump, misc outputs, and encoder pins
 *   to keep 6-axis motion plus Ethernet stable on N16R8-class modules.
 * =================================================================== */

#if defined(BOARD_ESP32S3_DEVKITC_OCTAL)

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
#define PIN_DIR_A       GPIO_NUM_10
#define PIN_DIR_B       GPIO_NUM_11
#define PIN_DIR_C       GPIO_NUM_12

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
#define PIN_STATUS_LED  GPIO_NUM_48

/* Charge pump and misc outputs disabled on N16R8-class modules */
#define HAS_CHARGE_PUMP     0
#define PIN_CHARGE_PUMP     GPIO_NUM_NC
#define MISC_OUTPUT_COUNT   0
#define PIN_MISC_OUT_0      GPIO_NUM_NC
#define PIN_MISC_OUT_1      GPIO_NUM_NC
#define MISC_INPUT_COUNT    2
#define PIN_MISC_IN_0       GPIO_NUM_18
#define PIN_MISC_IN_1       GPIO_NUM_21
#define PIN_MISC_IN_2       GPIO_NUM_NC

/* Spindle encoder disabled by default (no safe spare GPIOs left) */
#define PIN_ENCODER_A       GPIO_NUM_NC
#define PIN_ENCODER_B       GPIO_NUM_NC
#define PIN_ENCODER_INDEX   GPIO_NUM_NC

/* ESP32-S3 has internal pull-ups on all GPIOs */
#define LIMIT_PINS_NEED_EXTERNAL_PULLUP  0

/* W5500 Ethernet (optional) */
#define HAS_ETHERNET        1
#define PIN_ETH_MOSI        GPIO_NUM_45
#define PIN_ETH_MISO        GPIO_NUM_46
#define PIN_ETH_SCLK        GPIO_NUM_0
#define PIN_ETH_INT         GPIO_NUM_47
#define PIN_ETH_SPI_HOST    2

#endif /* BOARD_ESP32S3_DEVKITC_OCTAL */

/* ===================================================================
 * ESP32-WROOM-32 (Classic DevKit)
 * =================================================================== */

#ifdef BOARD_ESP32_WROOM32

#define PIN_STEP_X      GPIO_NUM_25
#define PIN_STEP_Y      GPIO_NUM_26
#define PIN_STEP_Z      GPIO_NUM_27
#define PIN_STEP_A      GPIO_NUM_14
#define PIN_STEP_B      GPIO_NUM_12
#define PIN_STEP_C      GPIO_NUM_13

#define PIN_DIR_X       GPIO_NUM_33
#define PIN_DIR_Y       GPIO_NUM_32
#define PIN_DIR_Z       GPIO_NUM_22
#define PIN_DIR_A       GPIO_NUM_21
#define PIN_DIR_B       GPIO_NUM_19
#define PIN_DIR_C       GPIO_NUM_18

#define PIN_ENABLE      GPIO_NUM_23

#define PIN_LIMIT_X     GPIO_NUM_36
#define PIN_LIMIT_Y     GPIO_NUM_39
#define PIN_LIMIT_Z     GPIO_NUM_34
#define PIN_LIMIT_A     GPIO_NUM_35
#define PIN_LIMIT_B     GPIO_NUM_4
#define PIN_LIMIT_C     GPIO_NUM_2

#define PIN_PROBE       GPIO_NUM_15
#define PIN_ESTOP       GPIO_NUM_0
#define PIN_SPINDLE_EN  GPIO_NUM_5
#define PIN_STATUS_LED  GPIO_NUM_2

#define LIMIT_PINS_NEED_EXTERNAL_PULLUP  1

#define HAS_CHARGE_PUMP     0
#define PIN_CHARGE_PUMP     GPIO_NUM_NC
#define MISC_OUTPUT_COUNT   0
#define MISC_INPUT_COUNT    0

#define PIN_ENCODER_A       GPIO_NUM_NC
#define PIN_ENCODER_B       GPIO_NUM_NC
#define PIN_ENCODER_INDEX   GPIO_NUM_NC

#define HAS_ETHERNET        0

#endif /* BOARD_ESP32_WROOM32 */

/* ===================================================================
 * Board-specific reserved GPIO helper
 * =================================================================== */
#if defined(BOARD_ESP32S3_DEVKITC_OCTAL)
#define PIN_RESERVED_FOR_MEMORY_BUS(pin) ((int)(pin) >= 26 && (int)(pin) <= 37)
#elif defined(BOARD_ESP32S3_DEVKITC_QUAD)
#define PIN_RESERVED_FOR_MEMORY_BUS(pin) ((int)(pin) >= 26 && (int)(pin) <= 32)
#else
#define PIN_RESERVED_FOR_MEMORY_BUS(pin) (0)
#endif

/* ===================================================================
 * Pin Arrays (for iteration in init code)
 * =================================================================== */

#define STEP_PINS   { PIN_STEP_X, PIN_STEP_Y, PIN_STEP_Z, PIN_STEP_A, PIN_STEP_B, PIN_STEP_C }
#define DIR_PINS    { PIN_DIR_X, PIN_DIR_Y, PIN_DIR_Z, PIN_DIR_A, PIN_DIR_B, PIN_DIR_C }
#define LIMIT_PINS  { PIN_LIMIT_X, PIN_LIMIT_Y, PIN_LIMIT_Z, PIN_LIMIT_A, PIN_LIMIT_B, PIN_LIMIT_C }

#endif /* PIN_CONFIG_H */
