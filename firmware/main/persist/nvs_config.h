/*
 * WiFi CNC Controller - NVS Configuration Storage
 *
 * Persistent configuration stored in ESP32's NVS (Non-Volatile Storage).
 * Provides get/set functions that map protocol config keys to NVS entries.
 */

#ifndef NVS_CONFIG_H
#define NVS_CONFIG_H

#include <stdint.h>
#include <stddef.h>

/* Initialize NVS config (load defaults, open NVS namespace) */
void nvs_config_init(void);

/* Generic get/set for string values */
void nvs_config_get_string(const char *key, char *value, size_t max_len,
                            const char *default_value);
void nvs_config_set_string(const char *key, const char *value);

/* Generic get/set for numeric values */
float nvs_config_get_float(const char *key, float default_value);
void nvs_config_set_float(const char *key, float value);

uint32_t nvs_config_get_u32(const char *key, uint32_t default_value);
void nvs_config_set_u32(const char *key, uint32_t value);

uint16_t nvs_config_get_u16(const char *key, uint16_t default_value);
void nvs_config_set_u16(const char *key, uint16_t value);

uint8_t nvs_config_get_u8(const char *key, uint8_t default_value);
void nvs_config_set_u8(const char *key, uint8_t value);

/* Protocol-keyed access (maps wcnc_config_key_t to NVS keys) */
void nvs_config_get_by_protocol_key(uint16_t key, uint8_t *value,
                                      size_t value_size, uint16_t *value_type);
void nvs_config_set_by_protocol_key(uint16_t key, const uint8_t *value,
                                      uint16_t value_type);

/* Commit all pending changes to flash */
void nvs_config_commit(void);

#endif /* NVS_CONFIG_H */
