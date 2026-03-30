/*
 * Tiggy Motion Controller - I/O Expansion Module
 *
 * Configures GPIO as general-purpose digital I/O when running in
 * I/O module mode (device_mode == 1). Pins, directions, pullups,
 * and inversions are all configurable via NVS.
 */

#include "io_module.h"
#include "../persist/nvs_config.h"
#include "../../../protocol/wifi_cnc_protocol.h"

#include <string.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "io_module";

#define IO_MAX_CHANNELS 16

static bool s_active = false;
static uint8_t s_channel_count = 0;
static gpio_num_t s_pins[IO_MAX_CHANNELS];
static uint16_t s_dir_mask = 0;       /* 1=output, 0=input */
static uint16_t s_invert_mask = 0;    /* 1=invert */
static uint16_t s_output_state = 0;

/* Check if a GPIO pin is valid */
static inline bool pin_valid(gpio_num_t pin)
{
    return (int)pin >= 0 && (int)pin < GPIO_NUM_MAX;
}

void io_module_init(void)
{
    s_channel_count = nvs_config_get_u8("io_cnt", 0);
    s_dir_mask = nvs_config_get_u16("io_dir", 0);
    uint16_t pullup_mask = nvs_config_get_u16("io_pull", 0xFFFF);
    s_invert_mask = nvs_config_get_u16("io_inv", 0);

    if (s_channel_count == 0 || s_channel_count > IO_MAX_CHANNELS) {
        ESP_LOGI(TAG, "No I/O channels configured (count=%d)", s_channel_count);
        s_active = false;
        return;
    }

    /* Read individual pin GPIO assignments from NVS */
    for (int i = 0; i < s_channel_count; i++) {
        char key[8];
        snprintf(key, sizeof(key), "io_p%d", i);
        s_pins[i] = (gpio_num_t)nvs_config_get_u8(key, 0xFF);

        if (!pin_valid(s_pins[i])) {
            ESP_LOGW(TAG, "I/O channel %d: invalid pin 0x%02X, skipping", i, s_pins[i]);
            continue;
        }

        bool is_output = (s_dir_mask >> i) & 1;
        bool use_pullup = (pullup_mask >> i) & 1;

        gpio_config_t conf = {
            .pin_bit_mask = (1ULL << s_pins[i]),
            .mode = is_output ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT,
            .pull_up_en = (!is_output && use_pullup) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&conf);

        if (is_output) {
            gpio_set_level(s_pins[i], 0);
        }

        ESP_LOGI(TAG, "I/O ch%d: GPIO %d %s%s",
                 i, s_pins[i],
                 is_output ? "OUT" : "IN",
                 use_pullup && !is_output ? " (pullup)" : "");
    }

    s_active = true;
    ESP_LOGI(TAG, "I/O module initialized: %d channels, dir_mask=0x%04X",
             s_channel_count, s_dir_mask);
}

bool io_module_is_active(void)
{
    return s_active;
}

uint16_t io_module_get_inputs(void)
{
    uint16_t mask = 0;
    for (int i = 0; i < s_channel_count; i++) {
        if (!pin_valid(s_pins[i])) continue;
        if ((s_dir_mask >> i) & 1) continue;  /* Skip outputs */

        int level = gpio_get_level(s_pins[i]);
        bool state = (bool)level ^ ((s_invert_mask >> i) & 1);
        if (state) mask |= (1 << i);
    }
    return mask;
}

void io_module_set_outputs(uint16_t mask)
{
    for (int i = 0; i < s_channel_count; i++) {
        if (!pin_valid(s_pins[i])) continue;
        if (!((s_dir_mask >> i) & 1)) continue;  /* Skip inputs */

        bool state = (mask >> i) & 1;
        bool inverted = state ^ ((s_invert_mask >> i) & 1);
        gpio_set_level(s_pins[i], inverted ? 1 : 0);
    }
    s_output_state = mask;
}

uint16_t io_module_get_outputs(void)
{
    return s_output_state;
}

uint8_t io_module_get_channel_count(void)
{
    return s_channel_count;
}
