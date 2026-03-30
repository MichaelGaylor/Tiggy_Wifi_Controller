/*
 * Tiggy Motion Controller - Spindle Encoder
 *
 * Uses ESP32-S3 PCNT (Pulse Counter) peripheral for hardware-based
 * quadrature decoding with zero CPU overhead. Supports:
 *   - Quadrature mode (A+B channels, x4 decoding)
 *   - Index-only mode (single pulse per revolution)
 *   - Configurable PPR and glitch filter
 *
 * RPM is calculated from index pulse intervals (if index pin configured)
 * or from PCNT count rate via periodic timer callback.
 */

#include "spindle_encoder.h"
#include "../persist/nvs_config.h"
#include "../pin_config.h"

#include <string.h>
#include <math.h>
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "encoder";

/* Check if a GPIO pin is valid (not NC / not 0xFF) */
static inline bool pin_valid(gpio_num_t pin)
{
    return (int)pin >= 0 && (int)pin < GPIO_NUM_MAX;
}

/* State */
static bool s_initialized = false;
static pcnt_unit_handle_t s_pcnt_unit = NULL;
static pcnt_channel_handle_t s_pcnt_chan_a = NULL;
static pcnt_channel_handle_t s_pcnt_chan_b = NULL;
static uint16_t s_ppr = 0;           /* Pulses per revolution (x4 for quadrature) */
static uint16_t s_configured_ppr = 0; /* Original PPR as configured by user */

/* Revolution tracking */
static volatile int32_t s_revolution_count = 0;
static volatile uint32_t s_index_count = 0;

/* RPM calculation via index pulse timing */
#define INDEX_HISTORY_SIZE 4
static volatile int64_t s_index_timestamps[INDEX_HISTORY_SIZE];
static volatile int s_index_head = 0;
static volatile bool s_has_index = false;

/* RPM calculation via PCNT rate (fallback when no index pin) */
static esp_timer_handle_t s_rpm_timer = NULL;
static volatile int s_last_count = 0;
static volatile int64_t s_last_time_us = 0;
static volatile uint16_t s_current_rpm = 0;

/* ===================================================================
 * PCNT Watchpoint Callback (revolution counting)
 * =================================================================== */

static bool pcnt_on_reach(pcnt_unit_handle_t unit,
                           const pcnt_watch_event_data_t *edata,
                           void *user_ctx)
{
    /* Called when PCNT count reaches +ppr or -ppr (one full revolution) */
    if (edata->watch_point_value > 0) {
        s_revolution_count++;
    } else {
        s_revolution_count--;
    }
    return false;  /* No need to yield from ISR */
}

/* ===================================================================
 * Index Pulse ISR
 * =================================================================== */

static void IRAM_ATTR index_pulse_isr(void *arg)
{
    int64_t now = esp_timer_get_time();
    s_index_timestamps[s_index_head] = now;
    s_index_head = (s_index_head + 1) % INDEX_HISTORY_SIZE;
    s_index_count++;
    s_has_index = true;
}

/* ===================================================================
 * RPM Timer Callback (for count-rate-based RPM when no index pin)
 * =================================================================== */

static void rpm_timer_callback(void *arg)
{
    int count = 0;
    pcnt_unit_get_count(s_pcnt_unit, &count);
    int64_t now = esp_timer_get_time();

    if (s_has_index) {
        /* Calculate RPM from index pulse intervals */
        int head = s_index_head;
        int oldest = (head + 1) % INDEX_HISTORY_SIZE;
        int64_t newest_ts = s_index_timestamps[(head - 1 + INDEX_HISTORY_SIZE) % INDEX_HISTORY_SIZE];
        int64_t oldest_ts = s_index_timestamps[oldest];

        if (newest_ts > oldest_ts && s_index_count >= INDEX_HISTORY_SIZE) {
            int64_t interval_us = (newest_ts - oldest_ts) / (INDEX_HISTORY_SIZE - 1);
            if (interval_us > 0) {
                s_current_rpm = (uint16_t)(60000000LL / interval_us);
            }
        }

        /* Timeout: if no index pulse in 2 seconds, RPM = 0 */
        if ((now - newest_ts) > 2000000) {
            s_current_rpm = 0;
        }
    } else {
        /* Estimate RPM from PCNT count rate */
        int64_t dt_us = now - s_last_time_us;
        if (dt_us > 0 && s_ppr > 0) {
            int delta = count - s_last_count;
            /* Handle wrap-around from watchpoints */
            int32_t total_counts = delta + (s_revolution_count * (int32_t)s_ppr);
            /* counts per second = delta / (dt_us / 1000000) */
            /* RPM = (counts_per_sec / ppr) * 60 */
            double cps = (double)abs(delta) * 1000000.0 / (double)dt_us;
            double rpm = (cps / (double)s_ppr) * 60.0;
            s_current_rpm = (uint16_t)fmin(rpm, 65535.0);
            (void)total_counts;
        }
    }

    s_last_count = count;
    s_last_time_us = now;
}

/* ===================================================================
 * Initialization
 * =================================================================== */

bool spindle_encoder_init(void)
{
    /* Read configuration from NVS */
    gpio_num_t pin_a = (gpio_num_t)nvs_config_get_u8("p_enc_a", (uint8_t)PIN_ENCODER_A);
    gpio_num_t pin_b = (gpio_num_t)nvs_config_get_u8("p_enc_b", (uint8_t)PIN_ENCODER_B);
    gpio_num_t pin_index = (gpio_num_t)nvs_config_get_u8("p_enc_i", (uint8_t)PIN_ENCODER_INDEX);
    s_configured_ppr = nvs_config_get_u16("enc_ppr", 0);
    uint8_t mode = nvs_config_get_u8("enc_mode", 0);
    uint16_t filter_ns = nvs_config_get_u16("enc_filt", 1000);

    /* If PPR is 0 or pins are not valid, encoder is not configured */
    if (s_configured_ppr == 0) {
        ESP_LOGI(TAG, "Spindle encoder not configured (PPR=0)");
        return false;
    }

    if (mode == 0) {
        /* Quadrature mode: need both A and B pins */
        if (!pin_valid(pin_a) || !pin_valid(pin_b)) {
            ESP_LOGW(TAG, "Quadrature mode requires valid A and B pins");
            return false;
        }
        /* x4 decoding: 4 counts per pulse */
        s_ppr = s_configured_ppr * 4;
    } else {
        /* Index-only mode: count rising edges on index pin */
        if (!pin_valid(pin_index)) {
            ESP_LOGW(TAG, "Index-only mode requires valid index pin");
            return false;
        }
        s_ppr = s_configured_ppr;
    }

    ESP_LOGI(TAG, "Encoder config: mode=%s ppr=%u pins A=%d B=%d I=%d filter=%uns",
             mode == 0 ? "quadrature" : "index-only",
             s_configured_ppr, pin_a, pin_b, pin_index, filter_ns);

    /* Configure PCNT unit */
    pcnt_unit_config_t unit_config = {
        .high_limit = (int)s_ppr,
        .low_limit = -(int)s_ppr,
    };
    esp_err_t err = pcnt_new_unit(&unit_config, &s_pcnt_unit);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT unit: %s", esp_err_to_name(err));
        return false;
    }

    /* Set glitch filter */
    if (filter_ns > 0) {
        pcnt_glitch_filter_config_t filter_config = {
            .max_glitch_ns = filter_ns,
        };
        pcnt_unit_set_glitch_filter(s_pcnt_unit, &filter_config);
    }

    if (mode == 0) {
        /* Quadrature mode: configure two channels for x4 decoding */

        /* Channel A: count on A edges, B determines direction */
        pcnt_chan_config_t chan_a_config = {
            .edge_gpio_num = pin_a,
            .level_gpio_num = pin_b,
        };
        err = pcnt_new_channel(s_pcnt_unit, &chan_a_config, &s_pcnt_chan_a);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create PCNT channel A: %s", esp_err_to_name(err));
            return false;
        }

        /* Rising A + B low = increment, Rising A + B high = decrement */
        pcnt_channel_set_edge_action(s_pcnt_chan_a,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE);
        pcnt_channel_set_level_action(s_pcnt_chan_a,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

        /* Channel B: count on B edges, A determines direction */
        pcnt_chan_config_t chan_b_config = {
            .edge_gpio_num = pin_b,
            .level_gpio_num = pin_a,
        };
        err = pcnt_new_channel(s_pcnt_unit, &chan_b_config, &s_pcnt_chan_b);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create PCNT channel B: %s", esp_err_to_name(err));
            return false;
        }

        pcnt_channel_set_edge_action(s_pcnt_chan_b,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        pcnt_channel_set_level_action(s_pcnt_chan_b,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    } else {
        /* Index-only mode: count rising edges on index pin */
        pcnt_chan_config_t chan_config = {
            .edge_gpio_num = pin_index,
            .level_gpio_num = -1,  /* No level pin */
        };
        err = pcnt_new_channel(s_pcnt_unit, &chan_config, &s_pcnt_chan_a);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create PCNT channel: %s", esp_err_to_name(err));
            return false;
        }

        /* Count on rising edge only */
        pcnt_channel_set_edge_action(s_pcnt_chan_a,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_HOLD);
        pcnt_channel_set_level_action(s_pcnt_chan_a,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP);
    }

    /* Add watchpoints for revolution counting */
    pcnt_unit_add_watch_point(s_pcnt_unit, (int)s_ppr);
    pcnt_unit_add_watch_point(s_pcnt_unit, -(int)s_ppr);

    /* Register watchpoint callback */
    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_reach,
    };
    pcnt_unit_register_event_callbacks(s_pcnt_unit, &cbs, NULL);

    /* Enable and start PCNT */
    pcnt_unit_enable(s_pcnt_unit);
    pcnt_unit_clear_count(s_pcnt_unit);
    pcnt_unit_start(s_pcnt_unit);

    /* Configure index pulse GPIO interrupt (if available in quadrature mode) */
    if (mode == 0 && pin_valid(pin_index)) {
        gpio_config_t idx_conf = {
            .pin_bit_mask = (1ULL << pin_index),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_POSEDGE,
        };
        gpio_config(&idx_conf);
        gpio_isr_handler_add(pin_index, index_pulse_isr, NULL);
        ESP_LOGI(TAG, "Index pulse on GPIO %d", pin_index);
    }

    /* Start RPM calculation timer (10 Hz) */
    s_last_time_us = esp_timer_get_time();
    esp_timer_create_args_t timer_args = {
        .callback = rpm_timer_callback,
        .name = "encoder_rpm",
    };
    esp_timer_create(&timer_args, &s_rpm_timer);
    esp_timer_start_periodic(s_rpm_timer, 100000);  /* 100ms = 10Hz */

    s_initialized = true;
    ESP_LOGI(TAG, "Spindle encoder initialized: %u PPR, %s mode",
             s_configured_ppr, mode == 0 ? "quadrature" : "index-only");
    return true;
}

/* ===================================================================
 * Data Access
 * =================================================================== */

bool spindle_encoder_is_available(void)
{
    return s_initialized;
}

uint16_t spindle_encoder_get_rpm(void)
{
    return s_current_rpm;
}

uint16_t spindle_encoder_get_position(void)
{
    if (!s_initialized || s_ppr == 0) return 0;

    int count = 0;
    pcnt_unit_get_count(s_pcnt_unit, &count);

    /* Normalize to 0..ppr-1 */
    int pos = count % (int)s_ppr;
    if (pos < 0) pos += (int)s_ppr;

    /* Scale to 0-65535 (0-360 degrees) */
    return (uint16_t)((uint32_t)pos * 65535 / s_ppr);
}

uint32_t spindle_encoder_get_index_count(void)
{
    return s_index_count;
}
