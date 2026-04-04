/*
 * WiFi CNC Controller - GPIO Control
 *
 * Configures and reads auxiliary I/O pins.
 * Limit switches use interrupt-driven debouncing with a software timer.
 */

#include "gpio_control.h"
#include "../config.h"
#include "../pin_config.h"
#include "../motion/motion_control.h"
#include "../persist/nvs_config.h"
#include "../../../protocol/wifi_cnc_protocol.h"

#include <string.h>
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "gpio_ctrl";

/* Check if a GPIO pin is usable on this board (not NC / not memory bus) */
static inline bool pin_valid_input(gpio_num_t pin)
{
    return (int)pin >= 0 && (int)pin < GPIO_NUM_MAX &&
           GPIO_IS_VALID_GPIO(pin) &&
           !PIN_RESERVED_FOR_MEMORY_BUS(pin);
}

static inline bool pin_valid_output(gpio_num_t pin)
{
    return (int)pin >= 0 && (int)pin < GPIO_NUM_MAX &&
           GPIO_IS_VALID_OUTPUT_GPIO(pin) &&
           !PIN_RESERVED_FOR_MEMORY_BUS(pin);
}

static gpio_num_t s_limit_pins[WCNC_MAX_AXES];
static gpio_num_t s_estop_pin;
static gpio_num_t s_probe_pin;
static gpio_num_t s_spindle_pin;
static gpio_num_t s_led_pin;
#if HAS_CHARGE_PUMP
static gpio_num_t s_charge_pump_pin;
#endif
#if MISC_OUTPUT_COUNT > 0
static gpio_num_t s_misc_out_pins[MISC_OUTPUT_COUNT];
#endif
#if MISC_INPUT_COUNT > 0
static gpio_num_t s_misc_in_pins[MISC_INPUT_COUNT];
#endif
static volatile uint8_t s_limit_state = 0;
static volatile bool s_probe_state = false;
static uint8_t s_invert_limit = CFG_DEFAULT_INVERT_LIMIT;
static uint8_t s_invert_home  = CFG_DEFAULT_INVERT_HOME;
static uint8_t s_invert_estop = CFG_DEFAULT_INVERT_ESTOP;
static uint8_t s_invert_probe = CFG_DEFAULT_INVERT_PROBE;
static volatile bool s_estop_hw_active = false;

/* Debounce timers */
static esp_timer_handle_t s_debounce_timer = NULL;
static volatile bool s_debounce_pending = false;
static esp_timer_handle_t s_estop_debounce_timer = NULL;
static volatile bool s_estop_debounce_pending = false;

/* ===================================================================
 * Debounce Handler
 * =================================================================== */

static void debounce_timer_callback(void *arg)
{
    (void)arg;
    uint8_t state = 0;

    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        if (!pin_valid_input(s_limit_pins[i])) continue;
        int level = gpio_get_level(s_limit_pins[i]);
        bool triggered = (bool)level ^ ((s_invert_limit >> i) & 1);
        if (triggered) {
            state |= (1 << i);
        }
    }

    s_limit_state = state;
    s_debounce_pending = false;

    /* If any limit is triggered during motion, trigger alarm */
    if (state != 0 && motion_get_state() == WCNC_STATE_RUN) {
        ESP_LOGW(TAG, "Limit switch triggered during motion: 0x%02X", state);
        motion_estop();
    }
}

/* ===================================================================
 * Limit Switch ISR
 * =================================================================== */

static void IRAM_ATTR limit_switch_isr(void *arg)
{
    /* Start debounce timer if not already pending */
    if (!s_debounce_pending) {
        s_debounce_pending = true;
        esp_timer_start_once(s_debounce_timer,
                              CFG_INPUT_DEBOUNCE_MS * 1000); /* microseconds */
    }
}

/* ===================================================================
 * Hardware E-Stop (GPIO 0 / BOOT button)
 *
 * Active LOW with internal pull-up: pressing the button pulls to GND.
 * Triggers immediate motion stop regardless of network state.
 * =================================================================== */

static void estop_debounce_callback(void *arg)
{
    (void)arg;
    int level = gpio_get_level(s_estop_pin);
    /* Configurable active level: invert_estop=0 → active low (level==0 = pressed)
     *                            invert_estop=1 → active high (level==1 = pressed) */
    bool pressed = ((bool)level) ^ (s_invert_estop == 0);

    s_estop_hw_active = pressed;
    s_estop_debounce_pending = false;

    if (pressed) {
        ESP_LOGW(TAG, "Hardware E-Stop activated!");
        motion_estop();
    }
}

static void IRAM_ATTR estop_isr(void *arg)
{
    if (!s_estop_debounce_pending) {
        s_estop_debounce_pending = true;
        esp_timer_start_once(s_estop_debounce_timer,
                              CFG_INPUT_DEBOUNCE_MS * 1000);
    }
}

/* ===================================================================
 * Initialization
 * =================================================================== */

void gpio_control_init(void)
{
    /* Load inversion settings from NVS (overrides compile-time defaults) */
    s_invert_limit = nvs_config_get_u8("inv_lim", CFG_DEFAULT_INVERT_LIMIT);
    s_invert_home  = nvs_config_get_u8("inv_home", CFG_DEFAULT_INVERT_HOME);
    s_invert_estop = nvs_config_get_u8("inv_estop", CFG_DEFAULT_INVERT_ESTOP);
    s_invert_probe = nvs_config_get_u8("inv_probe", CFG_DEFAULT_INVERT_PROBE);
    ESP_LOGI(TAG, "I/O inversion: limit=0x%02X home=0x%02X estop=%d probe=%d",
             s_invert_limit, s_invert_home, s_invert_estop, s_invert_probe);

    /* Load pin assignments from NVS (compile-time defaults as fallback) */
    {
        static const char *lim_nvs[] = {"p_lx","p_ly","p_lz","p_la","p_lb","p_lc"};
        gpio_num_t limit_defaults[] = LIMIT_PINS;
        for (int i = 0; i < WCNC_MAX_AXES; i++) {
            s_limit_pins[i] = (gpio_num_t)nvs_config_get_u8(lim_nvs[i], (uint8_t)limit_defaults[i]);
        }
    }
    s_estop_pin   = (gpio_num_t)nvs_config_get_u8("p_est", (uint8_t)PIN_ESTOP);
    s_probe_pin   = (gpio_num_t)nvs_config_get_u8("p_prb", (uint8_t)PIN_PROBE);
    s_spindle_pin = (gpio_num_t)nvs_config_get_u8("p_spn", (uint8_t)PIN_SPINDLE_EN);
    s_led_pin     = (gpio_num_t)nvs_config_get_u8("p_led", (uint8_t)PIN_STATUS_LED);
    ESP_LOGI(TAG, "Pin config: limits=[%d,%d,%d,%d,%d,%d] estop=%d probe=%d spindle=%d led=%d",
             s_limit_pins[0], s_limit_pins[1], s_limit_pins[2],
             s_limit_pins[3], s_limit_pins[4], s_limit_pins[5],
             s_estop_pin, s_probe_pin, s_spindle_pin, s_led_pin);

    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        if (!pin_valid_input(s_limit_pins[i])) {
            if ((int)s_limit_pins[i] != GPIO_NUM_NC) {
                ESP_LOGW(TAG, "Limit pin %d is not usable on this board, disabling", (int)s_limit_pins[i]);
            }
            s_limit_pins[i] = GPIO_NUM_NC;
        }
    }
    if (!pin_valid_input(s_estop_pin)) {
        ESP_LOGW(TAG, "E-stop pin %d is not usable on this board, disabling", (int)s_estop_pin);
        s_estop_pin = GPIO_NUM_NC;
    }
    if (!pin_valid_input(s_probe_pin)) {
        ESP_LOGW(TAG, "Probe pin %d is not usable on this board, disabling", (int)s_probe_pin);
        s_probe_pin = GPIO_NUM_NC;
    }
    if (!pin_valid_output(s_spindle_pin)) {
        ESP_LOGW(TAG, "Spindle pin %d is not usable on this board, disabling", (int)s_spindle_pin);
        s_spindle_pin = GPIO_NUM_NC;
    }
    /* Configure limit switch inputs (skip NC pins for unused axes) */
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        if (!pin_valid_input(s_limit_pins[i])) continue;
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << s_limit_pins[i]),
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_ANYEDGE,
        };

        /* Use internal pull-up if available on this board */
        #if !LIMIT_PINS_NEED_EXTERNAL_PULLUP
            io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        #else
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        #endif
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;

        gpio_config(&io_conf);
    }

    /* Configure hardware E-Stop input (active low, internal pull-up) */
    if (pin_valid_input(s_estop_pin)) {
        gpio_config_t estop_conf = {
            .pin_bit_mask = (1ULL << s_estop_pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE,
        };
        gpio_config(&estop_conf);
    }

    /* Configure probe input */
    if (pin_valid_input(s_probe_pin)) {
        gpio_config_t probe_conf = {
            .pin_bit_mask = (1ULL << s_probe_pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&probe_conf);
    }

    /* Configure spindle enable output */
    if (pin_valid_output(s_spindle_pin)) {
        gpio_config_t spindle_conf = {
            .pin_bit_mask = (1ULL << s_spindle_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&spindle_conf);
        gpio_set_level(s_spindle_pin, 0);
    }

    /* Status LED is handled by status_led module (supports WS2812) */

    /* Create debounce timers */
    esp_timer_create_args_t timer_args = {
        .callback = debounce_timer_callback,
        .name = "limit_debounce",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_debounce_timer));

    esp_timer_create_args_t estop_timer_args = {
        .callback = estop_debounce_callback,
        .name = "estop_debounce",
    };
    ESP_ERROR_CHECK(esp_timer_create(&estop_timer_args, &s_estop_debounce_timer));

    /* Install GPIO ISR service and attach to limit pins + E-Stop */
    gpio_install_isr_service(0);
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        if (!pin_valid_input(s_limit_pins[i])) continue;
        gpio_isr_handler_add(s_limit_pins[i], limit_switch_isr, NULL);
    }
    if (pin_valid_input(s_estop_pin)) {
        gpio_isr_handler_add(s_estop_pin, estop_isr, NULL);
    }

    /* Configure misc output pins (board-dependent) */
#if MISC_OUTPUT_COUNT > 0
    {
        gpio_num_t misc_defaults[] = {
            PIN_MISC_OUT_0,
        #if MISC_OUTPUT_COUNT > 1
            PIN_MISC_OUT_1,
        #endif
        };
        static const char *misc_nvs[] = {"p_mo0", "p_mo1"};
        for (int i = 0; i < MISC_OUTPUT_COUNT; i++) {
            s_misc_out_pins[i] = (gpio_num_t)nvs_config_get_u8(misc_nvs[i], (uint8_t)misc_defaults[i]);
            if (!pin_valid_output(s_misc_out_pins[i])) {
                ESP_LOGW(TAG, "Misc output pin %d is not usable on this board, disabling", (int)s_misc_out_pins[i]);
                s_misc_out_pins[i] = GPIO_NUM_NC;
                continue;
            }
            gpio_config_t misc_conf = {
                .pin_bit_mask = (1ULL << s_misc_out_pins[i]),
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE,
            };
            gpio_config(&misc_conf);
            gpio_set_level(s_misc_out_pins[i], 0);
        }
        ESP_LOGI(TAG, "Misc outputs configured: %d pins", MISC_OUTPUT_COUNT);
    }
#endif

    /* Configure misc input pins (board-dependent) */
#if MISC_INPUT_COUNT > 0
    {
        gpio_num_t misc_in_defaults[] = {
            PIN_MISC_IN_0,
        #if MISC_INPUT_COUNT > 1
            PIN_MISC_IN_1,
        #endif
        #if MISC_INPUT_COUNT > 2
            PIN_MISC_IN_2,
        #endif
        #if MISC_INPUT_COUNT > 3
            PIN_MISC_IN_3,
        #endif
        };
        static const char *misc_in_nvs[] = {"p_mi0", "p_mi1", "p_mi2", "p_mi3"};
        for (int i = 0; i < MISC_INPUT_COUNT; i++) {
            s_misc_in_pins[i] = (gpio_num_t)nvs_config_get_u8(misc_in_nvs[i], (uint8_t)misc_in_defaults[i]);
            if (pin_valid_input(s_misc_in_pins[i])) {
                gpio_config_t mi_conf = {
                    .pin_bit_mask = (1ULL << s_misc_in_pins[i]),
                    .mode = GPIO_MODE_INPUT,
                    .pull_up_en = GPIO_PULLUP_ENABLE,
                    .pull_down_en = GPIO_PULLDOWN_DISABLE,
                    .intr_type = GPIO_INTR_DISABLE,
                };
                gpio_config(&mi_conf);
            }
        }
        ESP_LOGI(TAG, "Misc inputs configured: %d pins", MISC_INPUT_COUNT);
    }
#endif

#if HAS_CHARGE_PUMP
    /* Charge pump pin configured on-demand when enabled */
    s_charge_pump_pin = (gpio_num_t)nvs_config_get_u8("p_cp", (uint8_t)PIN_CHARGE_PUMP);
    if (!pin_valid_output(s_charge_pump_pin)) {
        ESP_LOGW(TAG, "Charge pump pin %d is not usable on this board, disabling", (int)s_charge_pump_pin);
        s_charge_pump_pin = GPIO_NUM_NC;
    } else {
        gpio_config_t cp_conf = {
            .pin_bit_mask = (1ULL << s_charge_pump_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&cp_conf);
        gpio_set_level(s_charge_pump_pin, 0);
    }
#endif

    ESP_LOGI(TAG, "GPIO control initialized (limits + E-Stop: interrupt+debounce)");
}

/* ===================================================================
 * State Queries
 * =================================================================== */

bool gpio_control_get_limit(int axis)
{
    if (axis < 0 || axis >= WCNC_MAX_AXES) return false;
    if (!pin_valid_input(s_limit_pins[axis])) return false;
    int level = gpio_get_level(s_limit_pins[axis]);
    return (bool)level ^ ((s_invert_limit >> axis) & 1);
}

uint8_t gpio_control_get_limit_mask(void)
{
    return s_limit_state;
}

bool gpio_control_get_probe(void)
{
    if (!pin_valid_input(s_probe_pin)) return false;
    int level = gpio_get_level(s_probe_pin);
    return ((bool)level) ^ (s_invert_probe == 0);
}

bool gpio_control_get_estop(void)
{
    return s_estop_hw_active;
}

uint8_t gpio_control_get_home_mask(void)
{
    uint8_t mask = 0;
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        if (!pin_valid_input(s_limit_pins[i])) continue;
        int level = gpio_get_level(s_limit_pins[i]);  /* Same physical pin as limit */
        bool triggered = (bool)level ^ ((s_invert_home >> i) & 1);  /* Home inversion */
        if (triggered) mask |= (1 << i);
    }
    return mask;
}

void gpio_control_reload_inversion(void)
{
    s_invert_limit = nvs_config_get_u8("inv_lim", CFG_DEFAULT_INVERT_LIMIT);
    s_invert_home  = nvs_config_get_u8("inv_home", CFG_DEFAULT_INVERT_HOME);
    s_invert_estop = nvs_config_get_u8("inv_estop", CFG_DEFAULT_INVERT_ESTOP);
    s_invert_probe = nvs_config_get_u8("inv_probe", CFG_DEFAULT_INVERT_PROBE);
    ESP_LOGI(TAG, "I/O inversion reloaded: limit=0x%02X home=0x%02X estop=%d probe=%d",
             s_invert_limit, s_invert_home, s_invert_estop, s_invert_probe);
}

void gpio_control_set_spindle(bool enabled)
{
    if (pin_valid_output(s_spindle_pin)) {
        gpio_set_level(s_spindle_pin, enabled ? 1 : 0);
    }
}

void gpio_control_set_led(bool on)
{
    /* Legacy stub — LED is now managed by status_led module */
    (void)on;
}

/* ===================================================================
 * Misc I/O (board-dependent)
 * =================================================================== */

#if HAS_CHARGE_PUMP
#include "driver/ledc.h"
static bool s_charge_pump_active = false;
#endif

#if MISC_OUTPUT_COUNT > 0
static uint8_t s_misc_output_state = 0;
#endif

void gpio_control_set_charge_pump(bool enabled)
{
#if HAS_CHARGE_PUMP
    if (!pin_valid_output(s_charge_pump_pin)) return;
    if (enabled && !s_charge_pump_active) {
        /* Start PWM at configured frequency (default 10kHz) */
        uint16_t freq = nvs_config_get_u16("cp_freq", 10000);
        if (freq == 0) freq = 10000;

        ledc_timer_config_t timer_cfg = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_num  = LEDC_TIMER_1,
            .duty_resolution = LEDC_TIMER_8_BIT,
            .freq_hz = freq,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        ledc_timer_config(&timer_cfg);

        ledc_channel_config_t ch_cfg = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = LEDC_CHANNEL_1,
            .timer_sel  = LEDC_TIMER_1,
            .gpio_num   = s_charge_pump_pin,
            .duty       = 128,  /* 50% duty */
            .hpoint     = 0,
        };
        ledc_channel_config(&ch_cfg);
        s_charge_pump_active = true;
        ESP_LOGI(TAG, "Charge pump started at %u Hz", freq);
    } else if (!enabled && s_charge_pump_active) {
        ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        s_charge_pump_active = false;
        ESP_LOGI(TAG, "Charge pump stopped");
    }
#else
    (void)enabled;
#endif
}

void gpio_control_set_misc_output(int idx, bool state)
{
#if MISC_OUTPUT_COUNT > 0
    if (idx < 0 || idx >= MISC_OUTPUT_COUNT) return;
    if (!pin_valid_output(s_misc_out_pins[idx])) return;
    gpio_set_level(s_misc_out_pins[idx], state ? 1 : 0);
    if (state)
        s_misc_output_state |= (1 << idx);
    else
        s_misc_output_state &= ~(1 << idx);
#else
    (void)idx;
    (void)state;
#endif
}

uint8_t gpio_control_get_misc_output_mask(void)
{
#if MISC_OUTPUT_COUNT > 0
    return s_misc_output_state;
#else
    return 0;
#endif
}

uint8_t gpio_control_get_misc_input_mask(void)
{
#if MISC_INPUT_COUNT > 0
    uint8_t mask = 0;
    for (int i = 0; i < MISC_INPUT_COUNT; i++) {
        if (pin_valid_input(s_misc_in_pins[i])) {
            if (gpio_get_level(s_misc_in_pins[i]))
                mask |= (1 << i);
        }
    }
    return mask;
#else
    return 0;
#endif
}

uint8_t gpio_control_get_capabilities(void)
{
    uint8_t caps = 0;
#if HAS_CHARGE_PUMP
    caps |= WCNC_CAP_CHARGE_PUMP;
#endif
#if MISC_OUTPUT_COUNT > 0
    caps |= WCNC_CAP_MISC_OUTPUTS;
#endif
#if MISC_INPUT_COUNT > 0
    caps |= WCNC_CAP_MISC_INPUTS;
#endif
    return caps;
}


