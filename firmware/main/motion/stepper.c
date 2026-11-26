/*
 * WiFi CNC Controller - Stepper hardware
 *
 * Pins and timer for the step engine.  The timer counts microseconds and
 * runs for ever; the engine moves its alarm from one step to the next.  The
 * interrupt is allocated by the task that calls stepper_start_timer(), so
 * it runs on that task's core (the motion core, away from the network
 * stack), and it is kept in RAM so that it keeps its timing while flash is
 * being written.
 */

#include "stepper.h"
#include "step_engine.h"
#include "motion_core.h"
#include "motion_control.h"
#include "planner.h"
#include "se_port.h"
#include "../config.h"
#include "../pin_config.h"
#include "../pin_layout.h"
#include "../persist/nvs_config.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_log.h"

static const char *TAG = "stepper";

DRAM_ATTR se_port_t se_port;
portMUX_TYPE se_port_mux = portMUX_INITIALIZER_UNLOCKED;

static gpio_num_t s_enable_pin = GPIO_NUM_NC;
static uint8_t s_invert_enable = 0;
static bool s_enabled = false;

static bool pin_ok_output(int pin)
{
    return pin >= 0 && pin < GPIO_NUM_MAX && GPIO_IS_VALID_OUTPUT_GPIO(pin) && !PIN_RESERVED_FOR_MEMORY_BUS(pin);
}

static bool pin_ok_input(int pin)
{
    return pin >= 0 && pin < GPIO_NUM_MAX && GPIO_IS_VALID_GPIO(pin) && !PIN_RESERVED_FOR_MEMORY_BUS(pin);
}

/* A stored pin number: 0xFF (and anything else unusable) means "not fitted". */
static int stored_pin(const char *key, gpio_num_t fallback)
{
    uint8_t raw = nvs_config_get_u8(key, (uint8_t)fallback);
    return raw == 0xFF ? -1 : (int)raw;
}

static bool IRAM_ATTR on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *ctx)
{
    (void)timer; (void)edata; (void)ctx;
    se_isr();
    return false;
}

void stepper_init(void)
{
    static const char *step_keys[] = { "p_sx", "p_sy", "p_sz", "p_sa", "p_sb", "p_sc" };
    static const char *dir_keys[]  = { "p_dx", "p_dy", "p_dz", "p_da", "p_db", "p_dc" };
    static const char *lim_keys[]  = { "p_lx", "p_ly", "p_lz", "p_la", "p_lb", "p_lc" };

    memset(&se_port, 0, sizeof(se_port));
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        int step = stored_pin(step_keys[i], pin_layout_default((uint16_t)(WCNC_CFG_PIN_STEP_X + i)));
        int dir = stored_pin(dir_keys[i], pin_layout_default((uint16_t)(WCNC_CFG_PIN_DIR_X + i)));
        if (!pin_ok_output(step) || !pin_ok_output(dir)) {
            if (step >= 0 || dir >= 0)
                ESP_LOGW(TAG, "Axis %d: step/dir pins %d/%d cannot be used on this board - axis disabled", i, step, dir);
            step = dir = -1;
        }
        se_port.step_pin[i] = (int8_t)step;
        se_port.dir_pin[i] = (int8_t)dir;
        int lim = stored_pin(lim_keys[i], pin_layout_default((uint16_t)(WCNC_CFG_PIN_LIMIT_X + i)));
        se_port.limit_pin[i] = (int8_t)(pin_ok_input(lim) ? lim : -1);
    }
    int probe = stored_pin("p_prb", pin_layout_default(WCNC_CFG_PIN_PROBE));
    se_port.probe_pin = (int8_t)(pin_ok_input(probe) ? probe : -1);
    int enable = stored_pin("p_en", pin_layout_default(WCNC_CFG_PIN_ENABLE));
    s_enable_pin = pin_ok_output(enable) ? (gpio_num_t)enable : GPIO_NUM_NC;

    se_init();
    stepper_apply_timing_config();          /* polarities, before the pins become outputs */

    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        if (se_port.step_pin[i] < 0) continue;
        gpio_set_level((gpio_num_t)se_port.step_pin[i], !se_port.step_active[i]);
        gpio_set_level((gpio_num_t)se_port.dir_pin[i], !se_port.dir_negative[i]);
        gpio_config_t conf = {
            .pin_bit_mask = (1ULL << se_port.step_pin[i]) | (1ULL << se_port.dir_pin[i]),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&conf);
        gpio_set_level((gpio_num_t)se_port.step_pin[i], !se_port.step_active[i]);
        gpio_set_level((gpio_num_t)se_port.dir_pin[i], !se_port.dir_negative[i]);
    }
    if (s_enable_pin != GPIO_NUM_NC) {
        gpio_config_t conf = {
            .pin_bit_mask = (1ULL << s_enable_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_set_level(s_enable_pin, s_invert_enable ? 1 : 0);      /* drivers released */
        gpio_config(&conf);
    }
    s_enabled = true;                       /* force the write below */
    stepper_set_enabled(false);

    ESP_LOGI(TAG, "Terminals laid out for %s axes", pin_layout() == PIN_LAYOUT_FOUR_AXES ? "four" : "six");
    ESP_LOGI(TAG, "Pins: step=[%d,%d,%d,%d,%d,%d] dir=[%d,%d,%d,%d,%d,%d] enable=%d",
             se_port.step_pin[0], se_port.step_pin[1], se_port.step_pin[2],
             se_port.step_pin[3], se_port.step_pin[4], se_port.step_pin[5],
             se_port.dir_pin[0], se_port.dir_pin[1], se_port.dir_pin[2],
             se_port.dir_pin[3], se_port.dir_pin[4], se_port.dir_pin[5], (int)s_enable_pin);
}

void stepper_start_timer(void)
{
    gptimer_config_t config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = CFG_STEPPER_TIMER_RESOLUTION_HZ,
        .intr_priority = 3,
    };
    gptimer_handle_t timer = NULL;
    ESP_ERROR_CHECK(gptimer_new_timer(&config, &timer));
    gptimer_event_callbacks_t callbacks = { .on_alarm = on_alarm };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &callbacks, NULL));
    ESP_ERROR_CHECK(gptimer_enable(timer));
    ESP_ERROR_CHECK(gptimer_start(timer));
    se_port.timer = timer;
    ESP_LOGI(TAG, "Step timer running on core %d", xPortGetCoreID());
}

void stepper_apply_timing_config(void)
{
    uint16_t pulse = nvs_config_get_u16("pulse_us", CFG_DEFAULT_STEP_PULSE_US);
    uint16_t dir = nvs_config_get_u16("dir_us", CFG_DEFAULT_DIR_SETUP_US);
    uint8_t inv_step = nvs_config_get_u8("inv_step", CFG_DEFAULT_INVERT_STEP);
    uint8_t inv_dir = nvs_config_get_u8("inv_dir", CFG_DEFAULT_INVERT_DIR);
    uint8_t inv_limit = nvs_config_get_u8("inv_lim", CFG_DEFAULT_INVERT_LIMIT);
    uint8_t inv_probe = nvs_config_get_u8("inv_probe", CFG_DEFAULT_INVERT_PROBE);
    uint8_t inv_enable = nvs_config_get_u8("inv_en", CFG_DEFAULT_INVERT_ENABLE);

    se_set_timing(pulse, dir);

    portENTER_CRITICAL(&se_port_mux);
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        se_port.step_active[i] = ((inv_step >> i) & 1) ? 0 : 1;
        se_port.dir_negative[i] = ((inv_dir >> i) & 1) ? 1 : 0;
        /* inversion set: the switch pulls its input low when operated */
        se_port.limit_active[i] = ((inv_limit >> i) & 1) ? 0 : 1;
    }
    se_port.probe_active = inv_probe ? 1 : 0;       /* 0 = active low, 1 = active high */
    portEXIT_CRITICAL(&se_port_mux);

    /* Bring idle pins to their new resting levels.  (Direction pins are
     * rewritten by the engine before the next step in a new direction.) */
    if (se_port.timer && !se_active()) {
        for (int i = 0; i < WCNC_MAX_AXES; i++)
            if (se_port.step_pin[i] >= 0)
                gpio_set_level((gpio_num_t)se_port.step_pin[i], !se_port.step_active[i]);
    }
    if (inv_enable != s_invert_enable) {
        s_invert_enable = inv_enable;
        bool on = s_enabled;
        s_enabled = !on;
        stepper_set_enabled(on);
    }
}

void stepper_apply_axis_config(void)
{
    motion_config_changed();
}

void stepper_set_enabled(bool enabled)
{
    if (enabled == s_enabled) return;
    s_enabled = enabled;
    if (s_enable_pin != GPIO_NUM_NC) gpio_set_level(s_enable_pin, (enabled ^ (s_invert_enable & 1)) ? 1 : 0);
}

bool stepper_is_running(void)
{
    return mc_busy() || !planner_is_empty();
}

bool stepper_probe_triggered(void)
{
    return mc_probe_latched();
}

void stepper_get_position(int32_t position[WCNC_MAX_AXES])
{
    se_get_position(position);
}

int32_t stepper_get_feed_rate(void)
{
    return (int32_t)se_feed_rate();
}
