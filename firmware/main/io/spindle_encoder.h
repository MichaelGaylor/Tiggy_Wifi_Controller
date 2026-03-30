/*
 * Tiggy Motion Controller - Spindle Encoder
 *
 * PCNT-based spindle encoder driver for ESP32-S3.
 * Supports quadrature (A+B) and index-only modes.
 * Configurable PPR (pulses per revolution) via NVS.
 */

#ifndef SPINDLE_ENCODER_H
#define SPINDLE_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

/* Initialize PCNT-based spindle encoder.
 * Reads pin config from NVS (p_enc_a, p_enc_b, p_enc_i, enc_ppr, enc_mode).
 * Returns true if encoder hardware is configured and running. */
bool spindle_encoder_init(void);

/* Get current RPM (averaged over recent index pulses or PCNT rate).
 * Returns 0 if no encoder configured or no rotation detected. */
uint16_t spindle_encoder_get_rpm(void);

/* Get angular position within one revolution.
 * Returns 0-65535 mapping to 0-360 degrees. */
uint16_t spindle_encoder_get_position(void);

/* Get cumulative index pulse count since boot. */
uint32_t spindle_encoder_get_index_count(void);

/* Check if encoder hardware is present and initialized. */
bool spindle_encoder_is_available(void);

#endif /* SPINDLE_ENCODER_H */
