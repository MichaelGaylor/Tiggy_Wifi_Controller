/*
 * WiFi CNC Controller - GPIO Control
 *
 * Inputs are sampled once a millisecond by the motion task and debounced
 * by requiring a new level to hold for the configured time ("hm_dbnc",
 * CFG_INPUT_DEBOUNCE_MS unless a host or $26 changed it).  Polling, not
 * edge interrupts: what matters is whether a switch IS operated (also one
 * that already was when the machine started to move), and a noisy input can
 * then cost no more than one sample per millisecond.
 *
 * Polarity (matching the protocol's configuration keys):
 *   limit / home inputs   bit set in the inversion mask = operated when LOW
 *   probe, E-stop         0 = active low, 1 = active high
 * All inputs have their pull-up enabled where the chip provides one, so an
 * input with nothing connected reads "not operated" in the default set-up.
 */

#include "gpio_control.h"
#include "../config.h"
#include "../pin_config.h"
#include "../persist/nvs_config.h"
#include "../../../protocol/wifi_cnc_protocol.h"

#include <string.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "gpio_ctrl";

#define SPINDLE_LEDC_TIMER      LEDC_TIMER_0
#define SPINDLE_LEDC_CHANNEL    LEDC_CHANNEL_0
#define SPINDLE_LEDC_BITS       LEDC_TIMER_10_BIT
#define SPINDLE_DUTY_FULL       1024u
#define PUMP_LEDC_TIMER         LEDC_TIMER_1
#define PUMP_LEDC_CHANNEL       LEDC_CHANNEL_1

/* Board defaults for the optional pins */
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
#ifndef PIN_CHARGE_PUMP
#define PIN_CHARGE_PUMP GPIO_NUM_NC
#endif

static int s_limit_pins[WCNC_MAX_AXES];
static int s_estop_pin = -1, s_probe_pin = -1, s_spindle_pin = -1, s_pump_pin = -1;
static int s_misc_out_pins[GPIO_MISC_OUTPUTS];
static int s_misc_in_pins[GPIO_MISC_INPUTS];

static uint8_t s_invert_limit, s_invert_home, s_invert_estop, s_invert_probe;
static uint8_t s_debounce_ms = CFG_INPUT_DEBOUNCE_MS;   /* samples a new level must hold */

/* Debounced input levels (raw electrical level, 1 = high) */
typedef struct { uint8_t level; uint8_t count; } debounce_t;
static debounce_t s_db_limit[WCNC_MAX_AXES], s_db_estop, s_db_probe, s_db_misc[GPIO_MISC_INPUTS];
static volatile uint8_t s_limit_levels = 0x3F;      /* bit per input, debounced level */
static volatile uint8_t s_misc_levels = 0x0F;
static volatile bool s_estop_level = true, s_probe_level = true;

/* Outputs */
static uint8_t s_misc_state = 0;
static bool s_pump_on = false;
static uint16_t s_pump_freq = 0;
static bool s_spindle_ledc = false;                 /* spindle pin handed to the PWM unit */
static uint16_t s_spindle_freq = 0;

static bool pin_ok_input(int pin)
{
    return pin >= 0 && pin < GPIO_NUM_MAX && GPIO_IS_VALID_GPIO(pin) && !PIN_RESERVED_FOR_MEMORY_BUS(pin);
}

static bool pin_ok_output(int pin)
{
    return pin >= 0 && pin < GPIO_NUM_MAX && GPIO_IS_VALID_OUTPUT_GPIO(pin) && !PIN_RESERVED_FOR_MEMORY_BUS(pin);
}

static int stored_pin(const char *key, gpio_num_t fallback, bool output)
{
    uint8_t raw = nvs_config_get_u8(key, (uint8_t)fallback);
    int pin = raw == 0xFF ? -1 : (int)raw;
    if (pin >= 0 && !(output ? pin_ok_output(pin) : pin_ok_input(pin))) {
        ESP_LOGW(TAG, "Pin %d (%s) cannot be used on this board - disabled", pin, key);
        pin = -1;
    }
    return pin;
}

static void make_input(int pin)
{
    if (pin < 0) return;
    gpio_config_t conf = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,       /* has no effect on input-only pins: those need an external pull-up */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&conf);
}

static void make_output(int pin)
{
    if (pin < 0) return;
    gpio_set_level((gpio_num_t)pin, 0);
    gpio_config_t conf = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&conf);
    gpio_set_level((gpio_num_t)pin, 0);
}

/* ===================================================================
 * Initialization
 * =================================================================== */

void gpio_control_reload_inversion(void)
{
    s_invert_limit = nvs_config_get_u8("inv_lim", CFG_DEFAULT_INVERT_LIMIT);
    s_invert_home  = nvs_config_get_u8("inv_home", CFG_DEFAULT_INVERT_HOME);
    s_invert_estop = nvs_config_get_u8("inv_estop", CFG_DEFAULT_INVERT_ESTOP);
    s_invert_probe = nvs_config_get_u8("inv_probe", CFG_DEFAULT_INVERT_PROBE);

    uint16_t ms = nvs_config_get_u16("hm_dbnc", CFG_INPUT_DEBOUNCE_MS);
    s_debounce_ms = (uint8_t)(ms < 1 ? 1 : (ms > 100 ? 100 : ms));
}

void gpio_control_init(void)
{
    static const char *lim_keys[] = { "p_lx", "p_ly", "p_lz", "p_la", "p_lb", "p_lc" };
    static const char *out_keys[GPIO_MISC_OUTPUTS] = { "p_mo0", "p_mo1", "p_mo2", "p_mo3", "p_mo4" };
    static const char *in_keys[GPIO_MISC_INPUTS] = { "p_mi0", "p_mi1", "p_mi2", "p_mi3" };
    const gpio_num_t lim_defaults[] = LIMIT_PINS;
    const gpio_num_t out_defaults[GPIO_MISC_OUTPUTS] = { PIN_MISC_OUT_0, PIN_MISC_OUT_1, GPIO_NUM_NC, GPIO_NUM_NC, GPIO_NUM_NC };
    const gpio_num_t in_defaults[GPIO_MISC_INPUTS] = { PIN_MISC_IN_0, PIN_MISC_IN_1, PIN_MISC_IN_2, PIN_MISC_IN_3 };

    gpio_control_reload_inversion();

    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        s_limit_pins[i] = stored_pin(lim_keys[i], lim_defaults[i], false);
        make_input(s_limit_pins[i]);
        s_db_limit[i].level = 1;
    }
    s_estop_pin = stored_pin("p_est", PIN_ESTOP, false);
    s_probe_pin = stored_pin("p_prb", PIN_PROBE, false);
    make_input(s_estop_pin);
    make_input(s_probe_pin);
    s_db_estop.level = 1;
    s_db_probe.level = 1;
    for (int i = 0; i < GPIO_MISC_INPUTS; i++) {
        s_misc_in_pins[i] = stored_pin(in_keys[i], in_defaults[i], false);
        make_input(s_misc_in_pins[i]);
        s_db_misc[i].level = 1;
    }

    s_spindle_pin = stored_pin("p_spn", PIN_SPINDLE_EN, true);
    s_pump_pin = stored_pin("p_cp", PIN_CHARGE_PUMP, true);
    make_output(s_spindle_pin);
    make_output(s_pump_pin);
    for (int i = 0; i < GPIO_MISC_OUTPUTS; i++) {
        s_misc_out_pins[i] = stored_pin(out_keys[i], out_defaults[i], true);
        make_output(s_misc_out_pins[i]);
    }

    /* Start from the levels the pins have now, so that a switch that is
     * already operated at power-up is seen at once. */
    for (int n = 0; n <= s_debounce_ms; n++) gpio_control_poll();

    ESP_LOGI(TAG, "Inputs: limits=[%d,%d,%d,%d,%d,%d] estop=%d probe=%d aux=[%d,%d,%d,%d]",
             s_limit_pins[0], s_limit_pins[1], s_limit_pins[2], s_limit_pins[3], s_limit_pins[4], s_limit_pins[5],
             s_estop_pin, s_probe_pin, s_misc_in_pins[0], s_misc_in_pins[1], s_misc_in_pins[2], s_misc_in_pins[3]);
    ESP_LOGI(TAG, "Outputs: spindle=%d charge pump=%d aux=[%d,%d,%d,%d,%d]", s_spindle_pin, s_pump_pin,
             s_misc_out_pins[0], s_misc_out_pins[1], s_misc_out_pins[2], s_misc_out_pins[3], s_misc_out_pins[4]);
}

/* ===================================================================
 * Inputs
 * =================================================================== */

/* One sample.  An unfitted input reads as its resting level (high). */
static uint8_t debounce(debounce_t *d, int pin)
{
    uint8_t now = pin >= 0 ? (uint8_t)gpio_get_level((gpio_num_t)pin) : 1;
    if (now == d->level) {
        d->count = 0;
    } else if (++d->count >= s_debounce_ms) {
        d->level = now;
        d->count = 0;
    }
    return d->level;
}

void gpio_control_poll(void)
{
    uint8_t limits = 0, misc = 0;
    for (int i = 0; i < WCNC_MAX_AXES; i++)
        if (debounce(&s_db_limit[i], s_limit_pins[i])) limits |= (uint8_t)(1u << i);
    for (int i = 0; i < GPIO_MISC_INPUTS; i++)
        if (debounce(&s_db_misc[i], s_misc_in_pins[i])) misc |= (uint8_t)(1u << i);
    s_limit_levels = limits;
    s_misc_levels = misc;
    s_estop_level = debounce(&s_db_estop, s_estop_pin) != 0;
    s_probe_level = debounce(&s_db_probe, s_probe_pin) != 0;
}

static uint8_t fitted_limits(void)
{
    uint8_t mask = 0;
    for (int i = 0; i < WCNC_MAX_AXES; i++)
        if (s_limit_pins[i] >= 0) mask |= (uint8_t)(1u << i);
    return mask;
}

/* Inversion bit set: operated when the input is low. */
uint8_t gpio_control_get_limit_mask(void)
{
    return (uint8_t)((s_limit_levels ^ s_invert_limit) & fitted_limits());
}

uint8_t gpio_control_get_home_mask(void)
{
    return (uint8_t)((s_limit_levels ^ s_invert_home) & fitted_limits());
}

bool gpio_control_get_limit(int axis)
{
    if (axis < 0 || axis >= WCNC_MAX_AXES) return false;
    return (gpio_control_get_limit_mask() >> axis) & 1;
}

bool gpio_control_get_probe(void)
{
    if (s_probe_pin < 0) return false;
    return s_probe_level == (s_invert_probe != 0);      /* 0 = active low, 1 = active high */
}

bool gpio_control_get_estop(void)
{
    if (s_estop_pin < 0) return false;
    return s_estop_level == (s_invert_estop != 0);
}

uint8_t gpio_control_get_misc_input_mask(void)
{
    uint8_t mask = 0;
    for (int i = 0; i < GPIO_MISC_INPUTS; i++)
        if (s_misc_in_pins[i] >= 0 && !((s_misc_levels >> i) & 1)) mask |= (uint8_t)(1u << i);
    return mask;
}

/* ===================================================================
 * Spindle: an on/off output, or a PWM speed signal when
 * WCNC_CFG_SPINDLE_PWM_ENABLE is set.  Once the pin has been handed to the
 * PWM unit it stays there, and on/off is 100 % / 0 % duty.
 * =================================================================== */

static void spindle_write(bool on, uint16_t rpm)
{
    if (s_spindle_pin < 0) return;
    bool pwm = nvs_config_get_u8("sp_pwm", 0) != 0;
    if (!pwm && !s_spindle_ledc) {
        gpio_set_level((gpio_num_t)s_spindle_pin, on ? 1 : 0);
        return;
    }

    uint16_t freq = nvs_config_get_u16("sp_freq", 1000);
    if (freq < 50) freq = 50;
    if (freq > 50000) freq = 50000;
    if (!s_spindle_ledc) {
        ledc_timer_config_t timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_num = SPINDLE_LEDC_TIMER,
            .duty_resolution = SPINDLE_LEDC_BITS,
            .freq_hz = freq,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        ledc_channel_config_t channel = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = SPINDLE_LEDC_CHANNEL,
            .timer_sel = SPINDLE_LEDC_TIMER,
            .gpio_num = s_spindle_pin,
            .duty = 0,
            .hpoint = 0,
        };
        if (ledc_timer_config(&timer) != ESP_OK || ledc_channel_config(&channel) != ESP_OK) {
            ESP_LOGE(TAG, "Spindle PWM could not be set up at %u Hz", freq);
            gpio_set_level((gpio_num_t)s_spindle_pin, on ? 1 : 0);
            return;
        }
        s_spindle_ledc = true;
        s_spindle_freq = freq;
    } else if (freq != s_spindle_freq) {
        ledc_set_freq(LEDC_LOW_SPEED_MODE, SPINDLE_LEDC_TIMER, freq);
        s_spindle_freq = freq;
    }

    uint32_t duty = 0;
    if (on) {
        duty = SPINDLE_DUTY_FULL;
        if (pwm) {
            uint32_t max_rpm = nvs_config_get_u32("sp_rpm", 24000);
            if (max_rpm < 1) max_rpm = 1;
            duty = (uint32_t)(((uint64_t)rpm * SPINDLE_DUTY_FULL) / max_rpm);
            uint32_t floor_duty = ((uint32_t)nvs_config_get_u8("sp_mind", 0) * SPINDLE_DUTY_FULL) / 100u;
            if (duty < floor_duty) duty = floor_duty;
            if (duty > SPINDLE_DUTY_FULL) duty = SPINDLE_DUTY_FULL;
        }
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, SPINDLE_LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, SPINDLE_LEDC_CHANNEL);
}

void gpio_control_set_spindle(bool enabled)
{
    spindle_write(enabled, 0xFFFF);
}

/* ===================================================================
 * Charge pump: a square wave that tells a safety board the controller is
 * alive and under the control of a host.
 * =================================================================== */

void gpio_control_set_charge_pump(bool enabled)
{
    if (s_pump_pin < 0) return;
    uint16_t freq = nvs_config_get_u16("cp_freq", 10000);
    if (freq < 100) freq = 10000;
    if (enabled && (!s_pump_on || freq != s_pump_freq)) {
        ledc_timer_config_t timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_num = PUMP_LEDC_TIMER,
            .duty_resolution = LEDC_TIMER_8_BIT,
            .freq_hz = freq,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        ledc_channel_config_t channel = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = PUMP_LEDC_CHANNEL,
            .timer_sel = PUMP_LEDC_TIMER,
            .gpio_num = s_pump_pin,
            .duty = 128,                        /* 50 % */
            .hpoint = 0,
        };
        if (ledc_timer_config(&timer) == ESP_OK && ledc_channel_config(&channel) == ESP_OK) {
            s_pump_on = true;
            s_pump_freq = freq;
        }
    } else if (!enabled && s_pump_on) {
        ledc_stop(LEDC_LOW_SPEED_MODE, PUMP_LEDC_CHANNEL, 0);
        s_pump_on = false;
    }
}

/* ===================================================================
 * Auxiliary outputs
 * =================================================================== */

void gpio_control_set_misc_output(int idx, bool state)
{
    if (idx < 0 || idx >= GPIO_MISC_OUTPUTS || s_misc_out_pins[idx] < 0) return;
    gpio_set_level((gpio_num_t)s_misc_out_pins[idx], state ? 1 : 0);
    if (state) s_misc_state |= (uint8_t)(1u << idx);
    else s_misc_state &= (uint8_t)~(1u << idx);
}

uint8_t gpio_control_get_misc_output_mask(void)
{
    return s_misc_state;
}

void gpio_control_apply_outputs(uint8_t misc, uint8_t spindle_state, uint16_t rpm, bool charge_pump)
{
    for (int i = 0; i < GPIO_MISC_OUTPUTS; i++) gpio_control_set_misc_output(i, (misc >> i) & 1);
    spindle_write(spindle_state != 0, rpm);
    gpio_control_set_charge_pump(charge_pump);
}

uint8_t gpio_control_get_capabilities(void)
{
    uint8_t caps = 0;
    if (s_pump_pin >= 0) caps |= WCNC_CAP_CHARGE_PUMP;
    if (s_spindle_pin >= 0) caps |= WCNC_CAP_SPINDLE_PWM;
    for (int i = 0; i < GPIO_MISC_OUTPUTS; i++)
        if (s_misc_out_pins[i] >= 0) caps |= WCNC_CAP_MISC_OUTPUTS;
    for (int i = 0; i < GPIO_MISC_INPUTS; i++)
        if (s_misc_in_pins[i] >= 0) caps |= WCNC_CAP_MISC_INPUTS;
    return caps;
}
