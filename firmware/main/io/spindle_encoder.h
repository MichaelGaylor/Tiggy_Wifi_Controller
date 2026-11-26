/*
 * Tiggy Motion Controller - Spindle sensor (hardware side)
 *
 * A quadrature encoder (with or without an index pulse) or a single sensor
 * giving one pulse per revolution.  See spindle_encoder.c; what is made of
 * the signals is in motion/spindle_track.h.
 */

#ifndef SPINDLE_ENCODER_H
#define SPINDLE_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

/* Set up the sensor's inputs from the stored configuration (p_enc_a,
 * p_enc_b, p_enc_i, enc_ppr, enc_mode, enc_filt).  Returns true if a sensor
 * is configured on pins this board has. */
bool spindle_encoder_init(void);

/* Begin reading the sensor.  Call once the step timer runs, from the task
 * (and so on the core) the step interrupt belongs to. */
void spindle_encoder_start(void);

/* Spindle speed, rpm, steadied for display.  0 if no sensor or not turning. */
uint16_t spindle_encoder_get_rpm(void);

/* Angle past the index: 0-65535 for one revolution. */
uint16_t spindle_encoder_get_position(void);

/* Index pulses seen since power-up (revolutions, if there is no index pulse). */
uint32_t spindle_encoder_get_index_count(void);

/* A sensor is configured (and nothing has kept it from being read). */
bool spindle_encoder_is_available(void);

/* ...and moves can be played in step with it: it tells where in the
 * revolution the spindle is (an encoder, or one pulse per revolution). */
bool spindle_encoder_can_sync(void);

#endif /* SPINDLE_ENCODER_H */
