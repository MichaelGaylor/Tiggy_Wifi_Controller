/*
 * WiFi CNC Controller - What the terminals are used for
 *
 * See pin_layout.h.
 */

#include "pin_layout.h"
#include "pin_config.h"
#include "persist/nvs_config.h"
#include "../../protocol/wifi_cnc_protocol.h"

#include <stdbool.h>

/* Functions a board may not have at all */
#ifndef PIN_CHARGE_PUMP
#define PIN_CHARGE_PUMP GPIO_NUM_NC
#endif
#ifndef PIN_MISC_OUT_0
#define PIN_MISC_OUT_0  GPIO_NUM_NC
#endif
#ifndef PIN_MISC_OUT_1
#define PIN_MISC_OUT_1  GPIO_NUM_NC
#endif
#ifndef PIN_MISC_IN_0
#define PIN_MISC_IN_0   GPIO_NUM_NC
#endif
#ifndef PIN_MISC_IN_1
#define PIN_MISC_IN_1   GPIO_NUM_NC
#endif
#ifndef PIN_MISC_IN_2
#define PIN_MISC_IN_2   GPIO_NUM_NC
#endif
#ifndef PIN_MISC_IN_3
#define PIN_MISC_IN_3   GPIO_NUM_NC
#endif
#if !HAS_ETHERNET
#define PIN_ETH_MOSI    GPIO_NUM_NC
#define PIN_ETH_MISO    GPIO_NUM_NC
#define PIN_ETH_SCLK    GPIO_NUM_NC
#define PIN_ETH_INT     GPIO_NUM_NC
#endif

static int s_layout = -1, s_sensor = -1;

void pin_layout_reload(void)
{
    s_layout = nvs_config_get_u8("layout", PIN_LAYOUT_SIX_AXES) == PIN_LAYOUT_FOUR_AXES ? PIN_LAYOUT_FOUR_AXES
                                                                                        : PIN_LAYOUT_SIX_AXES;
    uint8_t mode = nvs_config_get_u8("enc_mode", 0);
    if (nvs_config_get_u16("enc_ppr", 0) == 0) s_sensor = PIN_SENSOR_NONE;
    else if (mode == 1) s_sensor = PIN_SENSOR_PULSE;
    else if (mode == 2) s_sensor = PIN_SENSOR_ENCODER;
    else s_sensor = PIN_SENSOR_ENCODER_INDEX;
}

int pin_layout(void)
{
    if (s_layout < 0) pin_layout_reload();
    return s_layout;
}

int pin_layout_sensor(void)
{
    if (s_sensor < 0) pin_layout_reload();
    return s_sensor;
}

/* As pin_config.h has it: six axes */
static gpio_num_t board_default(uint16_t key)
{
    switch (key) {
    case WCNC_CFG_PIN_STEP_X:        return PIN_STEP_X;
    case WCNC_CFG_PIN_STEP_Y:        return PIN_STEP_Y;
    case WCNC_CFG_PIN_STEP_Z:        return PIN_STEP_Z;
    case WCNC_CFG_PIN_STEP_A:        return PIN_STEP_A;
    case WCNC_CFG_PIN_STEP_B:        return PIN_STEP_B;
    case WCNC_CFG_PIN_STEP_C:        return PIN_STEP_C;
    case WCNC_CFG_PIN_DIR_X:         return PIN_DIR_X;
    case WCNC_CFG_PIN_DIR_Y:         return PIN_DIR_Y;
    case WCNC_CFG_PIN_DIR_Z:         return PIN_DIR_Z;
    case WCNC_CFG_PIN_DIR_A:         return PIN_DIR_A;
    case WCNC_CFG_PIN_DIR_B:         return PIN_DIR_B;
    case WCNC_CFG_PIN_DIR_C:         return PIN_DIR_C;
    case WCNC_CFG_PIN_ENABLE:        return PIN_ENABLE;
    case WCNC_CFG_PIN_LIMIT_X:       return PIN_LIMIT_X;
    case WCNC_CFG_PIN_LIMIT_Y:       return PIN_LIMIT_Y;
    case WCNC_CFG_PIN_LIMIT_Z:       return PIN_LIMIT_Z;
    case WCNC_CFG_PIN_LIMIT_A:       return PIN_LIMIT_A;
    case WCNC_CFG_PIN_LIMIT_B:       return PIN_LIMIT_B;
    case WCNC_CFG_PIN_LIMIT_C:       return PIN_LIMIT_C;
    case WCNC_CFG_PIN_PROBE:         return PIN_PROBE;
    case WCNC_CFG_PIN_ESTOP:         return PIN_ESTOP;
    case WCNC_CFG_PIN_SPINDLE:       return PIN_SPINDLE_EN;
    case WCNC_CFG_PIN_LED:           return PIN_STATUS_LED;
    case WCNC_CFG_PIN_CHARGE_PUMP:   return PIN_CHARGE_PUMP;
    case WCNC_CFG_PIN_MISC_OUT0:     return PIN_MISC_OUT_0;
    case WCNC_CFG_PIN_MISC_OUT1:     return PIN_MISC_OUT_1;
    case WCNC_CFG_PIN_ENCODER_A:     return PIN_ENCODER_A;
    case WCNC_CFG_PIN_ENCODER_B:     return PIN_ENCODER_B;
    case WCNC_CFG_PIN_ENCODER_INDEX: return PIN_ENCODER_INDEX;
    case WCNC_CFG_PIN_MISC_IN0:      return PIN_MISC_IN_0;
    case WCNC_CFG_PIN_MISC_IN1:      return PIN_MISC_IN_1;
    case WCNC_CFG_PIN_MISC_IN2:      return PIN_MISC_IN_2;
    case WCNC_CFG_PIN_MISC_IN3:      return PIN_MISC_IN_3;
    case WCNC_CFG_PIN_ETH_MOSI:      return PIN_ETH_MOSI;
    case WCNC_CFG_PIN_ETH_MISO:      return PIN_ETH_MISO;
    case WCNC_CFG_PIN_ETH_SCLK:      return PIN_ETH_SCLK;
    case WCNC_CFG_PIN_ETH_INT:       return PIN_ETH_INT;
    default:                         return GPIO_NUM_NC;
    }
}

#if defined(BOARD_ESP32S3_DEVKITC) || defined(BOARD_ESP32S3_ZERO)
/* A terminal the spindle sensor shares with another job: the sensor's while
 * a sensor of a kind that needs it is set up, the other job's otherwise.
 * 'sensor_key' is the sensor input the terminal can be (encoder A, B or the
 * index), 'key' the function being asked about. */
static bool shared(uint16_t key, uint16_t sensor_key, uint16_t other_key, gpio_num_t terminal, gpio_num_t *pin)
{
    if (key != sensor_key && key != other_key) return false;
    int sensor = pin_layout_sensor();
    bool taken = sensor_key == WCNC_CFG_PIN_ENCODER_INDEX
                     ? (sensor == PIN_SENSOR_ENCODER_INDEX || sensor == PIN_SENSOR_PULSE)
                     : (sensor == PIN_SENSOR_ENCODER_INDEX || sensor == PIN_SENSOR_ENCODER);
    *pin = (key == sensor_key) == taken ? terminal : GPIO_NUM_NC;
    return true;
}
#endif

/* What differs from pin_config.h.  False: nothing does for this function. */
static bool layout_default(uint16_t key, gpio_num_t *pin)
{
    bool four = pin_layout() == PIN_LAYOUT_FOUR_AXES;
    (void)four;

#if defined(BOARD_ESP32S3_DEVKITC)
    if (four) {
        switch (key) {
        case WCNC_CFG_PIN_STEP_B:  case WCNC_CFG_PIN_DIR_B:  case WCNC_CFG_PIN_LIMIT_B:
        case WCNC_CFG_PIN_STEP_C:  case WCNC_CFG_PIN_DIR_C:  case WCNC_CFG_PIN_LIMIT_C:
            *pin = GPIO_NUM_NC;
            return true;
        default:
            break;
        }
    }
#endif

#if defined(BOARD_ESP32S3_DEVKITC_OCTAL)
    /* The index pulse, or a single sensor, goes on what is otherwise
     * auxiliary input 2: an input either way.  A single sensor can go there
     * even with six axes; an encoder's index only when the encoder itself
     * has terminals, which takes four. */
    if (key == WCNC_CFG_PIN_ENCODER_INDEX || key == WCNC_CFG_PIN_MISC_IN1) {
        int sensor = pin_layout_sensor();
        bool index = sensor == PIN_SENSOR_PULSE || (four && sensor == PIN_SENSOR_ENCODER_INDEX);
        *pin = (key == WCNC_CFG_PIN_ENCODER_INDEX) == index ? GPIO_NUM_21 : GPIO_NUM_NC;
        return true;
    }
    if (four) {
        if (shared(key, WCNC_CFG_PIN_ENCODER_A, WCNC_CFG_PIN_MISC_IN2, GPIO_NUM_16, pin)) return true;
        if (shared(key, WCNC_CFG_PIN_ENCODER_B, WCNC_CFG_PIN_MISC_IN3, GPIO_NUM_17, pin)) return true;
        switch (key) {
        case WCNC_CFG_PIN_MISC_OUT0:    *pin = GPIO_NUM_11; return true;
        case WCNC_CFG_PIN_MISC_OUT1:    *pin = GPIO_NUM_12; return true;
        case WCNC_CFG_PIN_MISC_OUT2:    *pin = GPIO_NUM_40; return true;
        case WCNC_CFG_PIN_CHARGE_PUMP:  *pin = GPIO_NUM_39; return true;
        default:                        return false;
        }
    }
    return false;

#elif defined(BOARD_ESP32S3_DEVKITC_QUAD)
    /* GPIO 37 is spare whatever the layout: a single sensor can go there
     * even with six axes (an encoder's index only when the encoder itself
     * has terminals, which takes four) */
    if (key == WCNC_CFG_PIN_ENCODER_INDEX) {
        int sensor = pin_layout_sensor();
        *pin = (sensor == PIN_SENSOR_PULSE || (four && sensor == PIN_SENSOR_ENCODER_INDEX)) ? GPIO_NUM_37 : GPIO_NUM_NC;
        return true;
    }
    if (four) {
        if (shared(key, WCNC_CFG_PIN_ENCODER_A, WCNC_CFG_PIN_MISC_IN3, GPIO_NUM_16, pin)) return true;
        if (key == WCNC_CFG_PIN_ENCODER_B) {
            int sensor = pin_layout_sensor();
            *pin = (sensor == PIN_SENSOR_ENCODER_INDEX || sensor == PIN_SENSOR_ENCODER) ? GPIO_NUM_17 : GPIO_NUM_NC;
            return true;
        }
        switch (key) {
        case WCNC_CFG_PIN_MISC_OUT2:    *pin = GPIO_NUM_40; return true;
        case WCNC_CFG_PIN_MISC_OUT3:    *pin = GPIO_NUM_39; return true;
        case WCNC_CFG_PIN_MISC_OUT4:    *pin = GPIO_NUM_34; return true;
        default:                        return false;
        }
    }
    return false;

#elif defined(BOARD_ESP32S3_ZERO)
    /* The sensor terminals are there in either layout; free ones are inputs */
    if (shared(key, WCNC_CFG_PIN_ENCODER_A, WCNC_CFG_PIN_MISC_IN0, GPIO_NUM_15, pin)) return true;
    if (shared(key, WCNC_CFG_PIN_ENCODER_B, WCNC_CFG_PIN_MISC_IN1, GPIO_NUM_16, pin)) return true;
    if (shared(key, WCNC_CFG_PIN_ENCODER_INDEX, WCNC_CFG_PIN_MISC_IN2, GPIO_NUM_17, pin)) return true;
    return false;

#else
    (void)key;
    (void)pin;
    return false;
#endif
}

gpio_num_t pin_layout_default(uint16_t key)
{
    gpio_num_t pin;
    if (layout_default(key, &pin)) return pin;
    return board_default(key);
}
