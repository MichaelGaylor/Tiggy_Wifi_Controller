/*
 * WiFi CNC Controller - GRBL Settings Compatibility Layer
 *
 * Maps GRBL $N=value settings to the existing NVS configuration system.
 * Provides $$, $#, $I, $H, $X, $J commands.
 */

#ifndef GRBL_SETTINGS_H
#define GRBL_SETTINGS_H

#include <stdint.h>
#include <stddef.h>
#include "gcode_parser.h"

/* Output callback for sending responses */
typedef void (*grbl_output_fn)(const char *str, size_t len, void *ctx);

/* Handle "$N=value" set command. Returns GCODE_OK or error. */
gcode_error_t grbl_setting_set(int number, float value);

/* Handle "$$" (dump all settings) */
void grbl_settings_dump(grbl_output_fn out, void *ctx);

/* Handle "$#" (dump coordinate offsets: G54-G59, G92, PRB, etc.) */
void grbl_offsets_dump(const gcode_state_t *state, grbl_output_fn out, void *ctx);

/* Handle "$I" (build info) */
void grbl_build_info(grbl_output_fn out, void *ctx);

/* Handle "$N" (query single setting). Prints "$N=value" or error. */
void grbl_setting_query(int number, grbl_output_fn out, void *ctx);

/* Handle "$G" (dump parser state: active modal groups) */
void grbl_parser_state_dump(const gcode_state_t *state, grbl_output_fn out, void *ctx);

#endif /* GRBL_SETTINGS_H */
