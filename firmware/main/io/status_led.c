/*
 * WiFi CNC Controller - Status LED
 *
 * WS2812 RGB support on ESP32-S3 boards, plain GPIO on WROOM-32.
 */

#include "status_led.h"
#include "../pin_config.h"
#include "esp_log.h"

static const char *TAG = "status_led";
static status_led_state_t s_state = LED_STATE_DISCONNECTED;
static uint32_t s_tick_count = 0;
static bool s_blink_on = false;

/* ===================================================================
 * WS2812 RGB LED (ESP32-S3 boards)
 * =================================================================== */

#if defined(BOARD_ESP32S3_ZERO) || defined(BOARD_ESP32S3_DEVKITC)

#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

static rmt_channel_handle_t s_rmt_channel = NULL;
static rmt_encoder_handle_t s_encoder = NULL;

/* GRB pixel buffer for single WS2812 LED */
static uint8_t s_pixel[3] = {0, 0, 0};

static void ws2812_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_rmt_channel || !s_encoder) return;

    /* WS2812 expects GRB order */
    s_pixel[0] = g;
    s_pixel[1] = r;
    s_pixel[2] = b;

    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
    };
    rmt_transmit(s_rmt_channel, s_encoder, s_pixel, sizeof(s_pixel), &tx_cfg);
    rmt_tx_wait_all_done(s_rmt_channel, 10);
}

void status_led_init(void)
{
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = PIN_STATUS_LED,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000,      /* 10 MHz → 0.1 µs per tick */
        .mem_block_symbols = 64,
        .trans_queue_depth = 1,
    };

    esp_err_t err = rmt_new_tx_channel(&tx_cfg, &s_rmt_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RMT channel init failed: %s", esp_err_to_name(err));
        return;
    }
    rmt_enable(s_rmt_channel);

    /* WS2812 bit timing at 10 MHz:
     * Bit 0: high 0.3µs (3 ticks), low 0.9µs (9 ticks)
     * Bit 1: high 0.9µs (9 ticks), low 0.3µs (3 ticks) */
    rmt_bytes_encoder_config_t enc_cfg = {
        .bit0 = { .duration0 = 3, .level0 = 1, .duration1 = 9, .level1 = 0 },
        .bit1 = { .duration0 = 9, .level0 = 1, .duration1 = 3, .level1 = 0 },
        .flags.msb_first = 1,
    };

    err = rmt_new_bytes_encoder(&enc_cfg, &s_encoder);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RMT encoder init failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "WS2812 LED initialized on GPIO %d", PIN_STATUS_LED);
    ws2812_set_color(0, 0, 0);     /* Start off */
}

void status_led_set_state(status_led_state_t state)
{
    if (state == s_state) return;
    s_state = state;
    s_tick_count = 0;
    s_blink_on = true;

    /* Set initial color immediately */
    switch (state) {
    case LED_STATE_DISCONNECTED:    ws2812_set_color(20, 0, 0);   break;  /* Red (dim) */
    case LED_STATE_CONNECTED_IDLE:  ws2812_set_color(0, 0, 20);   break;  /* Blue (dim) */
    case LED_STATE_RUNNING:         ws2812_set_color(0, 20, 0);   break;  /* Green (dim) */
    case LED_STATE_ALARM:           ws2812_set_color(20, 10, 0);  break;  /* Yellow (dim) */
    }
}

void status_led_tick(void)
{
    s_tick_count++;

    switch (s_state) {
    case LED_STATE_DISCONNECTED:
        /* Slow blink red (toggle every 10 ticks = ~500ms) */
        if (s_tick_count % 10 == 0) {
            s_blink_on = !s_blink_on;
            ws2812_set_color(s_blink_on ? 20 : 0, 0, 0);
        }
        break;

    case LED_STATE_CONNECTED_IDLE:
        /* Solid blue — no blinking */
        break;

    case LED_STATE_RUNNING:
        /* Solid green — no blinking */
        break;

    case LED_STATE_ALARM:
        /* Fast blink yellow (toggle every 3 ticks = ~150ms) */
        if (s_tick_count % 3 == 0) {
            s_blink_on = !s_blink_on;
            ws2812_set_color(s_blink_on ? 20 : 0, s_blink_on ? 10 : 0, 0);
        }
        break;
    }
}

/* ===================================================================
 * Plain GPIO LED (ESP32-WROOM-32)
 * =================================================================== */

#else

#include "driver/gpio.h"

void status_led_init(void)
{
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << PIN_STATUS_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_conf);
    gpio_set_level(PIN_STATUS_LED, 0);
    ESP_LOGI(TAG, "Plain LED initialized on GPIO %d", PIN_STATUS_LED);
}

void status_led_set_state(status_led_state_t state)
{
    if (state == s_state) return;
    s_state = state;
    s_tick_count = 0;
    s_blink_on = true;

    switch (state) {
    case LED_STATE_DISCONNECTED:    gpio_set_level(PIN_STATUS_LED, 0); break;
    case LED_STATE_CONNECTED_IDLE:  gpio_set_level(PIN_STATUS_LED, 1); break;
    case LED_STATE_RUNNING:         gpio_set_level(PIN_STATUS_LED, 1); break;
    case LED_STATE_ALARM:           gpio_set_level(PIN_STATUS_LED, 1); break;
    }
}

void status_led_tick(void)
{
    s_tick_count++;

    switch (s_state) {
    case LED_STATE_DISCONNECTED:
        /* Off — no blinking */
        break;

    case LED_STATE_CONNECTED_IDLE:
        /* Solid on */
        break;

    case LED_STATE_RUNNING:
        /* Slow blink (every 10 ticks) */
        if (s_tick_count % 10 == 0) {
            s_blink_on = !s_blink_on;
            gpio_set_level(PIN_STATUS_LED, s_blink_on ? 1 : 0);
        }
        break;

    case LED_STATE_ALARM:
        /* Fast blink (every 3 ticks) */
        if (s_tick_count % 3 == 0) {
            s_blink_on = !s_blink_on;
            gpio_set_level(PIN_STATUS_LED, s_blink_on ? 1 : 0);
        }
        break;
    }
}

#endif
