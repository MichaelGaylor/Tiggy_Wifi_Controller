/*
 * Tiggy Motion Controller - Spindle sensor (hardware side)
 *
 * Reads the spindle sensor and hands what it sees to the spindle tracking
 * (motion/spindle_track.c), which makes of it the angle and speed everything
 * else goes by: the status report, and the step engine when it plays a move
 * in step with the spindle.
 *
 *   Quadrature encoder (enc_mode 0, or 2 for one without index pulse)
 *     A and B are counted, four counts per line, by a pulse counter unit.
 *     A timer interrupt on the motion core reads the count every
 *     SPT_SAMPLE_US.  The index pulse, if there is one and a pin for it,
 *     raises a GPIO interrupt; it is used only to learn where in the
 *     revolution the index is, so the moment it is noticed need not be exact.
 *
 *   Single sensor (enc_mode 1)
 *     One input (the index pin) giving enc_ppr pulses per revolution,
 *     normally one.  Each pulse raises a GPIO interrupt.
 *
 * Time is the step timer's count - the clock the step engine uses - and the
 * tracking is always called with the step engine's lock held, so the step
 * interrupt sees it whole.  The interrupts here are kept in RAM like the
 * step interrupt, and keep running while flash is written.
 *
 * The pulse input uses the GPIO interrupt service, which main.c installs on
 * the motion core before anything else can claim it for another.
 *
 * Signals.  The inputs have the chip's own pull-ups, about 45 kilohms.  That
 * is enough for a single sensor that switches to ground; an open-collector
 * encoder at speed needs pull-ups of its own (a few kilohms to 3.3 V), or
 * push-pull outputs.  The glitch filter (enc_filt, 1 us unless set) removes
 * anything shorter, which also puts the limit at about 250 000 lines a
 * second.
 */

#include "spindle_encoder.h"
#include "../motion/spindle_track.h"
#include "se_port.h"
#include "../persist/nvs_config.h"
#include "../pin_config.h"
#include "../pin_layout.h"

#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "sdkconfig.h"

/* The sampling interrupt reads the pulse counter while flash may be being
 * written: without these the firmware would crash the first time a setting
 * is saved.  (sdkconfig.defaults sets them; an sdkconfig.<environment> file
 * left over from an older build may not have them - delete it.) */
#if !defined(CONFIG_PCNT_CTRL_FUNC_IN_IRAM) || !defined(CONFIG_PCNT_ISR_IRAM_SAFE) || \
    !defined(CONFIG_GPTIMER_ISR_IRAM_SAFE) || !defined(CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM)
#error "sdkconfig is stale: the PCNT and GPTIMER options of sdkconfig.defaults are missing (delete firmware/sdkconfig.*, not the .defaults)"
#endif

static const char *TAG = "encoder";

#define FILTER_MAX_NS   12000   /* the most the pulse counter's glitch filter can do */

/* The hardware counter is 16 bits wide: it is made to start again from zero
 * at this count (in either direction), and the driver keeps the total. */
#define COUNT_LIMIT     30000

static bool s_configured = false;
static bool s_running = false;
static DRAM_ATTR uint8_t s_mode = SPT_NONE;
static DRAM_ATTR pcnt_unit_handle_t s_pcnt = NULL;
static DRAM_ATTR int s_last_count = 0;
static pcnt_channel_handle_t s_chan_a = NULL, s_chan_b = NULL;
static gptimer_handle_t s_sampler = NULL;
static gpio_num_t s_pulse_pin = GPIO_NUM_NC;

/* Check if a GPIO pin is usable on this board (not NC / not memory bus) */
static inline bool pin_valid(gpio_num_t pin)
{
    return (int)pin >= 0 && (int)pin < GPIO_NUM_MAX &&
           GPIO_IS_VALID_GPIO(pin) &&
           !PIN_RESERVED_FOR_MEMORY_BUS(pin);
}

/* ===================================================================
 * Interrupts
 * =================================================================== */

/* Every SPT_SAMPLE_US */
static bool IRAM_ATTR on_sample(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *ctx)
{
    (void)timer; (void)edata; (void)ctx;
    portENTER_CRITICAL_ISR(&se_port_mux);
    uint64_t now = port_now();
    if (s_mode == SPT_QUADRATURE) {
        int count = 0;
        pcnt_unit_get_count(s_pcnt, &count);
        /* The total is the hardware count plus what the driver has added
         * for each time the hardware count started again; between the
         * hardware doing that and the driver noticing, the total reads one
         * COUNT_LIMIT out.  Either way a jump of about that size is not the
         * spindle. */
        int delta = (int)((unsigned)count - (unsigned)s_last_count);    /* (the driver's total wraps in the end) */
        if (delta > COUNT_LIMIT / 2) delta -= COUNT_LIMIT;
        else if (delta < -COUNT_LIMIT / 2) delta += COUNT_LIMIT;
        s_last_count = count;
        spt_counts(now, delta);
    } else {
        spt_poll(now);
    }
    portEXIT_CRITICAL_ISR(&se_port_mux);
    return false;
}

/* Index pulse of an encoder, or a pulse of a single sensor.  The time is
 * taken first: for a single sensor it is the measurement, and waiting for
 * the lock would be added to it. */
static void IRAM_ATTR on_pulse(void *arg)
{
    (void)arg;
    uint64_t now = port_now();
    portENTER_CRITICAL_ISR(&se_port_mux);
    if (s_mode == SPT_QUADRATURE) spt_index(now);
    else spt_pulse(now);
    portEXIT_CRITICAL_ISR(&se_port_mux);
}

/* ===================================================================
 * Set-up
 * =================================================================== */

static bool setup_counter(gpio_num_t pin_a, gpio_num_t pin_b, uint16_t filter_ns)
{
    pcnt_unit_config_t unit_config = {
        .high_limit = COUNT_LIMIT,
        .low_limit = -COUNT_LIMIT,
        .flags.accum_count = 1,
    };
    esp_err_t err = pcnt_new_unit(&unit_config, &s_pcnt);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT unit: %s", esp_err_to_name(err));
        return false;
    }
    if (filter_ns > 0) {
        pcnt_glitch_filter_config_t filter_config = { .max_glitch_ns = filter_ns > FILTER_MAX_NS ? FILTER_MAX_NS : filter_ns };
        if (pcnt_unit_set_glitch_filter(s_pcnt, &filter_config) != ESP_OK)
            ESP_LOGW(TAG, "Glitch filter of %u ns not accepted: the encoder inputs are not filtered", (unsigned)filter_ns);
    }

    /* Channel A counts the edges of A, with B giving the direction; channel
     * B the edges of B, with A giving it: four counts per line. */
    pcnt_chan_config_t chan_a_config = { .edge_gpio_num = pin_a, .level_gpio_num = pin_b };
    pcnt_chan_config_t chan_b_config = { .edge_gpio_num = pin_b, .level_gpio_num = pin_a };
    if ((err = pcnt_new_channel(s_pcnt, &chan_a_config, &s_chan_a)) != ESP_OK ||
        (err = pcnt_new_channel(s_pcnt, &chan_b_config, &s_chan_b)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT channel: %s", esp_err_to_name(err));
        return false;
    }
    pcnt_channel_set_edge_action(s_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(s_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(s_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(s_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    /* The limits as watch points: that is what makes the driver keep the total */
    pcnt_unit_add_watch_point(s_pcnt, COUNT_LIMIT);
    pcnt_unit_add_watch_point(s_pcnt, -COUNT_LIMIT);

    if ((err = pcnt_unit_enable(s_pcnt)) != ESP_OK ||
        (err = pcnt_unit_clear_count(s_pcnt)) != ESP_OK ||
        (err = pcnt_unit_start(s_pcnt)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start PCNT unit: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

bool spindle_encoder_init(void)
{
    gpio_num_t pin_a = (gpio_num_t)nvs_config_get_u8("p_enc_a", (uint8_t)pin_layout_default(WCNC_CFG_PIN_ENCODER_A));
    gpio_num_t pin_b = (gpio_num_t)nvs_config_get_u8("p_enc_b", (uint8_t)pin_layout_default(WCNC_CFG_PIN_ENCODER_B));
    gpio_num_t pin_index = (gpio_num_t)nvs_config_get_u8("p_enc_i", (uint8_t)pin_layout_default(WCNC_CFG_PIN_ENCODER_INDEX));
    uint16_t ppr = nvs_config_get_u16("enc_ppr", 0);
    uint16_t filter_ns = nvs_config_get_u16("enc_filt", 1000);
    int sensor = pin_layout_sensor();       /* what enc_ppr and enc_mode say, as the terminals were laid out for */

    if (sensor == PIN_SENSOR_NONE) {
        ESP_LOGI(TAG, "Spindle sensor not configured (PPR=0)");
        return false;
    }

    if (sensor != PIN_SENSOR_PULSE) {
        if (!pin_valid(pin_a) || !pin_valid(pin_b)) {
            ESP_LOGW(TAG, "A quadrature encoder needs pins for A and B that this board has");
            return false;
        }
        if (!setup_counter(pin_a, pin_b, filter_ns)) return false;
        s_pulse_pin = (sensor == PIN_SENSOR_ENCODER_INDEX && pin_valid(pin_index)) ? pin_index : GPIO_NUM_NC;
        spt_setup(SPT_QUADRATURE, 4u * ppr, s_pulse_pin != GPIO_NUM_NC);
        s_mode = SPT_QUADRATURE;
        ESP_LOGI(TAG, "Spindle encoder: %u lines on GPIO %d/%d, index %s", ppr, (int)pin_a, (int)pin_b,
                 s_pulse_pin != GPIO_NUM_NC ? "pulse used" : "pulse not connected");
    } else {
        if (!pin_valid(pin_index)) {
            ESP_LOGW(TAG, "A single spindle sensor needs an index pin that this board has");
            return false;
        }
        s_pulse_pin = pin_index;
        spt_setup(SPT_PULSES, ppr, false);
        s_mode = SPT_PULSES;
        ESP_LOGI(TAG, "Spindle sensor: %u pulse(s) per revolution on GPIO %d%s", ppr, (int)pin_index,
                 ppr == 1 ? "" : " (speed only: screw cutting needs one pulse per revolution)");
    }

    if (s_pulse_pin != GPIO_NUM_NC) {
        gpio_config_t conf = {
            .pin_bit_mask = (1ULL << s_pulse_pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,     /* switched on in spindle_encoder_start() */
        };
        gpio_config(&conf);
    }
    s_configured = true;
    return true;
}

/* A sensor that cannot be read is no sensor: the controller then says it has
 * none, and refuses a move that is to be played by the spindle, rather than
 * wait for a spindle it will never see turn. */
static void give_up(const char *what)
{
    ESP_LOGE(TAG, "No %s for the spindle sensor: it is not used", what);
    portENTER_CRITICAL(&se_port_mux);
    spt_setup(SPT_NONE, 0, false);
    s_mode = SPT_NONE;
    portEXIT_CRITICAL(&se_port_mux);
    s_configured = false;
}

void spindle_encoder_start(void)
{
    if (!s_configured || s_running) return;
    if (!se_port.timer) {
        give_up("clock");
        return;
    }

    if (s_pulse_pin != GPIO_NUM_NC) {
        if (gpio_set_intr_type(s_pulse_pin, GPIO_INTR_POSEDGE) != ESP_OK ||
            gpio_isr_handler_add(s_pulse_pin, on_pulse, NULL) != ESP_OK) {
            give_up("interrupt");
            return;
        }
    }

    /* (Priority 3, like the step timer: between them they take both of this
     * core's interrupt lines of that priority, so nothing else here can ask
     * for it.) */
    gptimer_config_t config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
        .intr_priority = 3,
    };
    gptimer_event_callbacks_t callbacks = { .on_alarm = on_sample };
    gptimer_alarm_config_t alarm = {
        .alarm_count = SPT_SAMPLE_US,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    if (gptimer_new_timer(&config, &s_sampler) != ESP_OK ||
        gptimer_register_event_callbacks(s_sampler, &callbacks, NULL) != ESP_OK ||
        gptimer_set_alarm_action(s_sampler, &alarm) != ESP_OK ||
        gptimer_enable(s_sampler) != ESP_OK ||
        gptimer_start(s_sampler) != ESP_OK) {
        if (s_pulse_pin != GPIO_NUM_NC) gpio_isr_handler_remove(s_pulse_pin);
        give_up("timer");
        return;
    }
    s_running = true;
    ESP_LOGI(TAG, "Spindle sensor being read");
}

/* ===================================================================
 * Data Access
 * =================================================================== */

/* True from the moment the sensor is set up; false again if reading it could
 * not be started. */
bool spindle_encoder_is_available(void)
{
    return s_configured;
}

bool spindle_encoder_can_sync(void)
{
    return s_configured && spt_can_sync();
}

uint16_t spindle_encoder_get_rpm(void)
{
    if (!s_running) return 0;
    portENTER_CRITICAL(&se_port_mux);
    uint32_t milli = spt_rpm_milli();
    portEXIT_CRITICAL(&se_port_mux);
    uint32_t rpm = (milli + 500u) / 1000u;
    return (uint16_t)(rpm > 65535u ? 65535u : rpm);
}

uint16_t spindle_encoder_get_position(void)
{
    if (!s_running) return 0;
    portENTER_CRITICAL(&se_port_mux);
    uint16_t position = spt_position16(port_now());
    portEXIT_CRITICAL(&se_port_mux);
    return position;
}

uint32_t spindle_encoder_get_index_count(void)
{
    if (!s_running) return 0;
    portENTER_CRITICAL(&se_port_mux);
    uint32_t count = spt_index_count();
    portEXIT_CRITICAL(&se_port_mux);
    return count;
}
