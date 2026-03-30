/*
 * Tiggy Motion Controller - I/O Expansion Module
 *
 * When device_mode == 1, the firmware runs as an I/O expansion module
 * instead of a motion controller. Step/dir/enable GPIO are reconfigured
 * as general-purpose digital I/O, controlled by NVS configuration.
 */

#ifndef IO_MODULE_H
#define IO_MODULE_H

#include <stdint.h>
#include <stdbool.h>

/* Initialize I/O module mode: configure GPIO per NVS settings.
 * Only call this when device_mode == 1 (instead of planner/stepper init). */
void io_module_init(void);

/* Check if I/O module mode is active */
bool io_module_is_active(void);

/* Read all digital inputs as bitmask (up to 16 channels) */
uint16_t io_module_get_inputs(void);

/* Set all digital outputs from bitmask */
void io_module_set_outputs(uint16_t mask);

/* Get current output state */
uint16_t io_module_get_outputs(void);

/* Get number of configured I/O channels */
uint8_t io_module_get_channel_count(void);

#endif /* IO_MODULE_H */
