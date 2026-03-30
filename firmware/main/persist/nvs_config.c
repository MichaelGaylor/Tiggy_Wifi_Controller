/*
 * WiFi CNC Controller - NVS Configuration Storage
 *
 * Maps protocol configuration keys to NVS key-value pairs.
 * All configuration is stored under the "wcnc" NVS namespace.
 */

#include "nvs_config.h"
#include "../config.h"
#include "../pin_config.h"
#include "../../../protocol/wifi_cnc_protocol.h"

#include <string.h>
#include <stdio.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "nvs_cfg";
static nvs_handle_t s_nvs_handle;

/* ===================================================================
 * Initialization
 * =================================================================== */

void nvs_config_init(void)
{
    esp_err_t err = nvs_open("wcnc", NVS_READWRITE, &s_nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace 'wcnc': %s",
                 esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "NVS namespace 'wcnc' opened");
}

/* ===================================================================
 * Generic Access Functions
 * =================================================================== */

void nvs_config_get_string(const char *key, char *value, size_t max_len,
                            const char *default_value)
{
    size_t required_size = max_len;
    esp_err_t err = nvs_get_str(s_nvs_handle, key, value, &required_size);
    if (err != ESP_OK) {
        strlcpy(value, default_value, max_len);
    }
}

void nvs_config_set_string(const char *key, const char *value)
{
    nvs_set_str(s_nvs_handle, key, value);
}

float nvs_config_get_float(const char *key, float default_value)
{
    /* NVS doesn't have native float support; store as uint32 (bitcast) */
    uint32_t raw;
    esp_err_t err = nvs_get_u32(s_nvs_handle, key, &raw);
    if (err != ESP_OK) return default_value;
    float result;
    memcpy(&result, &raw, sizeof(float));
    return result;
}

void nvs_config_set_float(const char *key, float value)
{
    uint32_t raw;
    memcpy(&raw, &value, sizeof(uint32_t));
    nvs_set_u32(s_nvs_handle, key, raw);
}

uint32_t nvs_config_get_u32(const char *key, uint32_t default_value)
{
    uint32_t value;
    esp_err_t err = nvs_get_u32(s_nvs_handle, key, &value);
    return (err == ESP_OK) ? value : default_value;
}

void nvs_config_set_u32(const char *key, uint32_t value)
{
    nvs_set_u32(s_nvs_handle, key, value);
}

uint16_t nvs_config_get_u16(const char *key, uint16_t default_value)
{
    uint16_t value;
    esp_err_t err = nvs_get_u16(s_nvs_handle, key, &value);
    return (err == ESP_OK) ? value : default_value;
}

void nvs_config_set_u16(const char *key, uint16_t value)
{
    nvs_set_u16(s_nvs_handle, key, value);
}

uint8_t nvs_config_get_u8(const char *key, uint8_t default_value)
{
    uint8_t value;
    esp_err_t err = nvs_get_u8(s_nvs_handle, key, &value);
    return (err == ESP_OK) ? value : default_value;
}

void nvs_config_set_u8(const char *key, uint8_t value)
{
    nvs_set_u8(s_nvs_handle, key, value);
}

void nvs_config_commit(void)
{
    esp_err_t err = nvs_commit(s_nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Configuration saved to NVS");
    }
}

/* ===================================================================
 * Protocol Key Mapping
 * =================================================================== */

/* Map protocol config keys to NVS key names */
static const char *protocol_key_to_nvs(uint16_t key)
{
    switch (key) {
    case WCNC_CFG_STEPS_PER_MM_X: return "spm_x";
    case WCNC_CFG_STEPS_PER_MM_Y: return "spm_y";
    case WCNC_CFG_STEPS_PER_MM_Z: return "spm_z";
    case WCNC_CFG_STEPS_PER_MM_A: return "spm_a";
    case WCNC_CFG_STEPS_PER_MM_B: return "spm_b";
    case WCNC_CFG_STEPS_PER_MM_C: return "spm_c";
    case WCNC_CFG_MAX_RATE_X:     return "rate_x";
    case WCNC_CFG_MAX_RATE_Y:     return "rate_y";
    case WCNC_CFG_MAX_RATE_Z:     return "rate_z";
    case WCNC_CFG_MAX_RATE_A:     return "rate_a";
    case WCNC_CFG_MAX_RATE_B:     return "rate_b";
    case WCNC_CFG_MAX_RATE_C:     return "rate_c";
    case WCNC_CFG_ACCEL_X:        return "acc_x";
    case WCNC_CFG_ACCEL_Y:        return "acc_y";
    case WCNC_CFG_ACCEL_Z:        return "acc_z";
    case WCNC_CFG_ACCEL_A:        return "acc_a";
    case WCNC_CFG_ACCEL_B:        return "acc_b";
    case WCNC_CFG_ACCEL_C:        return "acc_c";
    case WCNC_CFG_STEP_PULSE_US:  return "pulse_us";
    case WCNC_CFG_DIR_SETUP_US:   return "dir_us";
    case WCNC_CFG_STEP_IDLE_DELAY_MS: return "idle_ms";
    case WCNC_CFG_STATUS_INTERVAL_MS: return "stat_ms";
    case WCNC_CFG_WIFI_SSID:      return "wifi_ssid";
    case WCNC_CFG_WIFI_PASSWORD:   return "wifi_pass";
    case WCNC_CFG_IP_MODE:        return "ip_mode";
    case WCNC_CFG_STATIC_IP:      return "ip_addr";
    case WCNC_CFG_STATIC_GATEWAY: return "ip_gw";
    case WCNC_CFG_STATIC_NETMASK: return "ip_mask";
    case WCNC_CFG_INVERT_STEP:    return "inv_step";
    case WCNC_CFG_INVERT_DIR:     return "inv_dir";
    case WCNC_CFG_INVERT_ENABLE:  return "inv_en";
    case WCNC_CFG_INVERT_LIMIT:   return "inv_lim";
    case WCNC_CFG_INVERT_HOME:    return "inv_home";
    case WCNC_CFG_INVERT_ESTOP:   return "inv_estop";
    case WCNC_CFG_INVERT_PROBE:   return "inv_probe";
    case WCNC_CFG_HOMING_DIR_MASK: return "hm_dir";
    case WCNC_CFG_HOMING_SEEK_RATE: return "hm_seek";
    case WCNC_CFG_HOMING_FEED_RATE: return "hm_feed";
    case WCNC_CFG_HOMING_PULLOFF:  return "hm_pull";
    case WCNC_CFG_CHARGE_PUMP_FREQ: return "cp_freq";
    case WCNC_CFG_SPINDLE_PWM_FREQ: return "sp_freq";
    case WCNC_CFG_SPINDLE_MAX_RPM:  return "sp_rpm";
    /* Pin assignments */
    case WCNC_CFG_PIN_STEP_X:      return "p_sx";
    case WCNC_CFG_PIN_STEP_Y:      return "p_sy";
    case WCNC_CFG_PIN_STEP_Z:      return "p_sz";
    case WCNC_CFG_PIN_STEP_A:      return "p_sa";
    case WCNC_CFG_PIN_STEP_B:      return "p_sb";
    case WCNC_CFG_PIN_STEP_C:      return "p_sc";
    case WCNC_CFG_PIN_DIR_X:       return "p_dx";
    case WCNC_CFG_PIN_DIR_Y:       return "p_dy";
    case WCNC_CFG_PIN_DIR_Z:       return "p_dz";
    case WCNC_CFG_PIN_DIR_A:       return "p_da";
    case WCNC_CFG_PIN_DIR_B:       return "p_db";
    case WCNC_CFG_PIN_DIR_C:       return "p_dc";
    case WCNC_CFG_PIN_ENABLE:      return "p_en";
    case WCNC_CFG_PIN_LIMIT_X:     return "p_lx";
    case WCNC_CFG_PIN_LIMIT_Y:     return "p_ly";
    case WCNC_CFG_PIN_LIMIT_Z:     return "p_lz";
    case WCNC_CFG_PIN_LIMIT_A:     return "p_la";
    case WCNC_CFG_PIN_LIMIT_B:     return "p_lb";
    case WCNC_CFG_PIN_LIMIT_C:     return "p_lc";
    case WCNC_CFG_PIN_PROBE:       return "p_prb";
    case WCNC_CFG_PIN_ESTOP:       return "p_est";
    case WCNC_CFG_PIN_SPINDLE:     return "p_spn";
    case WCNC_CFG_PIN_LED:         return "p_led";
    case WCNC_CFG_PIN_CHARGE_PUMP: return "p_cp";
    case WCNC_CFG_PIN_MISC_OUT0:   return "p_mo0";
    case WCNC_CFG_PIN_MISC_OUT1:   return "p_mo1";
    /* Spindle encoder */
    case WCNC_CFG_PIN_ENCODER_A:     return "p_enc_a";
    case WCNC_CFG_PIN_ENCODER_B:     return "p_enc_b";
    case WCNC_CFG_PIN_ENCODER_INDEX: return "p_enc_i";
    case WCNC_CFG_ENCODER_PPR:       return "enc_ppr";
    case WCNC_CFG_ENCODER_MODE:      return "enc_mode";
    case WCNC_CFG_ENCODER_FILTER_NS: return "enc_filt";
    /* Misc input pins */
    case WCNC_CFG_PIN_MISC_IN0:    return "p_mi0";
    case WCNC_CFG_PIN_MISC_IN1:    return "p_mi1";
    case WCNC_CFG_PIN_MISC_IN2:    return "p_mi2";
    case WCNC_CFG_PIN_MISC_IN3:    return "p_mi3";
    /* Device mode + I/O module */
    case WCNC_CFG_DEVICE_MODE:     return "dev_mode";
    case WCNC_CFG_IO_PIN_COUNT:    return "io_cnt";
    case WCNC_CFG_IO_DIR_MASK:     return "io_dir";
    case WCNC_CFG_IO_PULLUP_MASK:  return "io_pull";
    case WCNC_CFG_IO_INVERT_MASK:  return "io_inv";
    default:
        /* I/O module pin assignments: 0x0620..0x062F */
        if (key >= WCNC_CFG_IO_PIN_BASE && key < WCNC_CFG_IO_PIN_BASE + 16) {
            static char io_pin_key[8];
            snprintf(io_pin_key, sizeof(io_pin_key), "io_p%d", key - WCNC_CFG_IO_PIN_BASE);
            return io_pin_key;
        }
        return NULL;
    }
}

/* Determine the default value type for a protocol key */
static uint16_t protocol_key_value_type(uint16_t key)
{
    if (key >= WCNC_CFG_STEPS_PER_MM_X && key <= WCNC_CFG_STEPS_PER_MM_C)
        return WCNC_VAL_FLOAT;
    if (key >= WCNC_CFG_MAX_RATE_X && key <= WCNC_CFG_MAX_RATE_C)
        return WCNC_VAL_UINT32;
    if (key >= WCNC_CFG_ACCEL_X && key <= WCNC_CFG_ACCEL_C)
        return WCNC_VAL_UINT32;
    if (key == WCNC_CFG_STEP_PULSE_US || key == WCNC_CFG_DIR_SETUP_US ||
        key == WCNC_CFG_STEP_IDLE_DELAY_MS || key == WCNC_CFG_STATUS_INTERVAL_MS)
        return WCNC_VAL_UINT16;
    if (key == WCNC_CFG_WIFI_SSID || key == WCNC_CFG_WIFI_PASSWORD)
        return WCNC_VAL_STRING;
    if (key == WCNC_CFG_IP_MODE || key == WCNC_CFG_INVERT_STEP ||
        key == WCNC_CFG_INVERT_DIR || key == WCNC_CFG_INVERT_ENABLE ||
        key == WCNC_CFG_INVERT_LIMIT || key == WCNC_CFG_INVERT_HOME ||
        key == WCNC_CFG_INVERT_ESTOP || key == WCNC_CFG_INVERT_PROBE ||
        key == WCNC_CFG_HOMING_DIR_MASK)
        return WCNC_VAL_UINT8;
    if (key == WCNC_CFG_STATIC_IP || key == WCNC_CFG_STATIC_GATEWAY ||
        key == WCNC_CFG_STATIC_NETMASK || key == WCNC_CFG_HOMING_SEEK_RATE ||
        key == WCNC_CFG_HOMING_FEED_RATE || key == WCNC_CFG_HOMING_PULLOFF ||
        key == WCNC_CFG_SPINDLE_MAX_RPM)
        return WCNC_VAL_UINT32;
    if (key == WCNC_CFG_CHARGE_PUMP_FREQ || key == WCNC_CFG_SPINDLE_PWM_FREQ)
        return WCNC_VAL_UINT16;
    if (key >= WCNC_CFG_PIN_STEP_X && key <= WCNC_CFG_PIN_MISC_OUT1)
        return WCNC_VAL_UINT8;
    /* Encoder: pins and mode are uint8, PPR and filter are uint16 */
    if (key == WCNC_CFG_PIN_ENCODER_A || key == WCNC_CFG_PIN_ENCODER_B ||
        key == WCNC_CFG_PIN_ENCODER_INDEX || key == WCNC_CFG_ENCODER_MODE)
        return WCNC_VAL_UINT8;
    if (key == WCNC_CFG_ENCODER_PPR || key == WCNC_CFG_ENCODER_FILTER_NS)
        return WCNC_VAL_UINT16;
    /* Misc input pins */
    if (key >= WCNC_CFG_PIN_MISC_IN0 && key <= WCNC_CFG_PIN_MISC_IN3)
        return WCNC_VAL_UINT8;
    /* Device mode, I/O module pin count */
    if (key == WCNC_CFG_DEVICE_MODE || key == WCNC_CFG_IO_PIN_COUNT)
        return WCNC_VAL_UINT8;
    /* I/O module masks are uint16 */
    if (key == WCNC_CFG_IO_DIR_MASK || key == WCNC_CFG_IO_PULLUP_MASK ||
        key == WCNC_CFG_IO_INVERT_MASK)
        return WCNC_VAL_UINT16;
    /* I/O module pin assignments */
    if (key >= WCNC_CFG_IO_PIN_BASE && key < WCNC_CFG_IO_PIN_BASE + 16)
        return WCNC_VAL_UINT8;

    return WCNC_VAL_UINT32;
}

/* Return the proper compile-time default for a protocol key */
static uint32_t protocol_key_default_u32(uint16_t key)
{
    if (key >= WCNC_CFG_MAX_RATE_X && key <= WCNC_CFG_MAX_RATE_C)
        return CFG_DEFAULT_MAX_RATE;
    if (key >= WCNC_CFG_ACCEL_X && key <= WCNC_CFG_ACCEL_C)
        return CFG_DEFAULT_ACCELERATION;
    if (key == WCNC_CFG_HOMING_SEEK_RATE) return CFG_DEFAULT_HOMING_SEEK_RATE;
    if (key == WCNC_CFG_HOMING_FEED_RATE) return CFG_DEFAULT_HOMING_FEED_RATE;
    if (key == WCNC_CFG_HOMING_PULLOFF)   return CFG_DEFAULT_HOMING_PULLOFF;
    return 0;
}

static uint16_t protocol_key_default_u16(uint16_t key)
{
    if (key == WCNC_CFG_STEP_PULSE_US)      return CFG_DEFAULT_STEP_PULSE_US;
    if (key == WCNC_CFG_DIR_SETUP_US)        return CFG_DEFAULT_DIR_SETUP_US;
    if (key == WCNC_CFG_STEP_IDLE_DELAY_MS)  return CFG_DEFAULT_STEP_IDLE_DELAY_MS;
    if (key == WCNC_CFG_STATUS_INTERVAL_MS)  return CFG_DEFAULT_STATUS_INTERVAL_MS;
    if (key == WCNC_CFG_CHARGE_PUMP_FREQ)    return 10000;  /* 10 kHz default */
    if (key == WCNC_CFG_SPINDLE_PWM_FREQ)    return 1000;   /* 1 kHz default */
    if (key == WCNC_CFG_ENCODER_PPR)         return 0;      /* 0 = not configured */
    if (key == WCNC_CFG_ENCODER_FILTER_NS)   return 1000;   /* 1us glitch filter */
    return 0;
}

static uint8_t protocol_key_default_u8(uint16_t key)
{
    if (key == WCNC_CFG_INVERT_LIMIT)   return CFG_DEFAULT_INVERT_LIMIT;
    if (key == WCNC_CFG_INVERT_HOME)    return CFG_DEFAULT_INVERT_HOME;
    if (key == WCNC_CFG_INVERT_ESTOP)   return CFG_DEFAULT_INVERT_ESTOP;
    if (key == WCNC_CFG_INVERT_PROBE)   return CFG_DEFAULT_INVERT_PROBE;
    if (key == WCNC_CFG_INVERT_STEP)    return CFG_DEFAULT_INVERT_STEP;
    if (key == WCNC_CFG_INVERT_DIR)     return CFG_DEFAULT_INVERT_DIR;
    if (key == WCNC_CFG_INVERT_ENABLE)  return CFG_DEFAULT_INVERT_ENABLE;
    if (key == WCNC_CFG_HOMING_DIR_MASK) return CFG_DEFAULT_HOMING_DIR_MASK;
    /* Pin assignment defaults from pin_config.h */
    if (key == WCNC_CFG_PIN_STEP_X)      return (uint8_t)PIN_STEP_X;
    if (key == WCNC_CFG_PIN_STEP_Y)      return (uint8_t)PIN_STEP_Y;
    if (key == WCNC_CFG_PIN_STEP_Z)      return (uint8_t)PIN_STEP_Z;
    if (key == WCNC_CFG_PIN_STEP_A)      return (uint8_t)PIN_STEP_A;
    if (key == WCNC_CFG_PIN_STEP_B)      return (uint8_t)PIN_STEP_B;
    if (key == WCNC_CFG_PIN_STEP_C)      return (uint8_t)PIN_STEP_C;
    if (key == WCNC_CFG_PIN_DIR_X)       return (uint8_t)PIN_DIR_X;
    if (key == WCNC_CFG_PIN_DIR_Y)       return (uint8_t)PIN_DIR_Y;
    if (key == WCNC_CFG_PIN_DIR_Z)       return (uint8_t)PIN_DIR_Z;
    if (key == WCNC_CFG_PIN_DIR_A)       return (uint8_t)PIN_DIR_A;
    if (key == WCNC_CFG_PIN_DIR_B)       return (uint8_t)PIN_DIR_B;
    if (key == WCNC_CFG_PIN_DIR_C)       return (uint8_t)PIN_DIR_C;
    if (key == WCNC_CFG_PIN_ENABLE)      return (uint8_t)PIN_ENABLE;
    if (key == WCNC_CFG_PIN_LIMIT_X)     return (uint8_t)PIN_LIMIT_X;
    if (key == WCNC_CFG_PIN_LIMIT_Y)     return (uint8_t)PIN_LIMIT_Y;
    if (key == WCNC_CFG_PIN_LIMIT_Z)     return (uint8_t)PIN_LIMIT_Z;
    if (key == WCNC_CFG_PIN_LIMIT_A)     return (uint8_t)PIN_LIMIT_A;
    if (key == WCNC_CFG_PIN_LIMIT_B)     return (uint8_t)PIN_LIMIT_B;
    if (key == WCNC_CFG_PIN_LIMIT_C)     return (uint8_t)PIN_LIMIT_C;
    if (key == WCNC_CFG_PIN_PROBE)       return (uint8_t)PIN_PROBE;
    if (key == WCNC_CFG_PIN_ESTOP)       return (uint8_t)PIN_ESTOP;
    if (key == WCNC_CFG_PIN_SPINDLE)     return (uint8_t)PIN_SPINDLE_EN;
    if (key == WCNC_CFG_PIN_LED)         return (uint8_t)PIN_STATUS_LED;
#if HAS_CHARGE_PUMP
    if (key == WCNC_CFG_PIN_CHARGE_PUMP) return (uint8_t)PIN_CHARGE_PUMP;
#endif
#if MISC_OUTPUT_COUNT > 0
    if (key == WCNC_CFG_PIN_MISC_OUT0)   return (uint8_t)PIN_MISC_OUT_0;
#endif
#if MISC_OUTPUT_COUNT > 1
    if (key == WCNC_CFG_PIN_MISC_OUT1)   return (uint8_t)PIN_MISC_OUT_1;
#endif
    /* Encoder pin defaults */
    if (key == WCNC_CFG_PIN_ENCODER_A)     return (uint8_t)PIN_ENCODER_A;
    if (key == WCNC_CFG_PIN_ENCODER_B)     return (uint8_t)PIN_ENCODER_B;
    if (key == WCNC_CFG_PIN_ENCODER_INDEX) return (uint8_t)PIN_ENCODER_INDEX;
    if (key == WCNC_CFG_ENCODER_MODE)      return 0;  /* quadrature */
    /* Misc input pin defaults */
#if MISC_INPUT_COUNT > 0
    if (key == WCNC_CFG_PIN_MISC_IN0)     return (uint8_t)PIN_MISC_IN_0;
#endif
#if MISC_INPUT_COUNT > 1
    if (key == WCNC_CFG_PIN_MISC_IN1)     return (uint8_t)PIN_MISC_IN_1;
#endif
#if MISC_INPUT_COUNT > 2
    if (key == WCNC_CFG_PIN_MISC_IN2)     return (uint8_t)PIN_MISC_IN_2;
#endif
#if MISC_INPUT_COUNT > 3
    if (key == WCNC_CFG_PIN_MISC_IN3)     return (uint8_t)PIN_MISC_IN_3;
#endif
    /* Device mode + I/O module defaults */
    if (key == WCNC_CFG_DEVICE_MODE)       return 0;  /* motion controller */
    if (key == WCNC_CFG_IO_PIN_COUNT)      return 0;
    /* I/O module pin defaults: 0xFF = not configured */
    if (key >= WCNC_CFG_IO_PIN_BASE && key < WCNC_CFG_IO_PIN_BASE + 16)
        return 0xFF;
    return 0;
}

void nvs_config_get_by_protocol_key(uint16_t key, uint8_t *value,
                                      size_t value_size, uint16_t *value_type)
{
    const char *nvs_key = protocol_key_to_nvs(key);
    if (!nvs_key) {
        ESP_LOGW(TAG, "Unknown config key: 0x%04X", key);
        memset(value, 0, value_size);
        *value_type = WCNC_VAL_UINT32;
        return;
    }

    *value_type = protocol_key_value_type(key);
    memset(value, 0, value_size);

    switch (*value_type) {
    case WCNC_VAL_FLOAT: {
        float v = nvs_config_get_float(nvs_key, CFG_DEFAULT_STEPS_PER_MM);
        memcpy(value, &v, sizeof(float));
        break;
    }
    case WCNC_VAL_UINT32: {
        uint32_t v = nvs_config_get_u32(nvs_key, protocol_key_default_u32(key));
        memcpy(value, &v, sizeof(uint32_t));
        break;
    }
    case WCNC_VAL_UINT16: {
        uint16_t v = nvs_config_get_u16(nvs_key, protocol_key_default_u16(key));
        memcpy(value, &v, sizeof(uint16_t));
        break;
    }
    case WCNC_VAL_UINT8: {
        uint8_t v = nvs_config_get_u8(nvs_key, protocol_key_default_u8(key));
        value[0] = v;
        break;
    }
    case WCNC_VAL_STRING: {
        nvs_config_get_string(nvs_key, (char *)value, value_size, "");
        break;
    }
    default:
        break;
    }
}

void nvs_config_set_by_protocol_key(uint16_t key, const uint8_t *value,
                                      uint16_t value_type)
{
    const char *nvs_key = protocol_key_to_nvs(key);
    if (!nvs_key) {
        ESP_LOGW(TAG, "Unknown config key for set: 0x%04X", key);
        return;
    }

    switch (value_type) {
    case WCNC_VAL_FLOAT: {
        float v;
        memcpy(&v, value, sizeof(float));
        nvs_config_set_float(nvs_key, v);
        break;
    }
    case WCNC_VAL_UINT32: {
        uint32_t v;
        memcpy(&v, value, sizeof(uint32_t));
        nvs_config_set_u32(nvs_key, v);
        break;
    }
    case WCNC_VAL_UINT16: {
        uint16_t v;
        memcpy(&v, value, sizeof(uint16_t));
        nvs_config_set_u16(nvs_key, v);
        break;
    }
    case WCNC_VAL_UINT8: {
        nvs_config_set_u8(nvs_key, value[0]);
        break;
    }
    case WCNC_VAL_STRING: {
        nvs_config_set_string(nvs_key, (const char *)value);
        break;
    }
    default:
        ESP_LOGW(TAG, "Unknown value type: %d", value_type);
        break;
    }

    ESP_LOGI(TAG, "Config set: key=0x%04X nvs='%s' type=%d", key, nvs_key, value_type);
}
