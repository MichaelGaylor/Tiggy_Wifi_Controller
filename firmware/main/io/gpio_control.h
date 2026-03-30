/*
 * WiFi CNC Controller - GPIO Control
 *
 * Manages auxiliary I/O: limit switches, probe input, spindle enable.
 * Step/dir/enable pins are managed by the stepper module.
 */

#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/* Initialize limit switch inputs, probe input, spindle output */
void gpio_control_init(void);

/* Get limit switch state for an axis (true = triggered) */
bool gpio_control_get_limit(int axis);

/* Get bitmask of all limit switches (bit 0=X, bit 5=C) */
uint8_t gpio_control_get_limit_mask(void);

/* Get bitmask of home switch states (same pins as limits, home inversion applied) */
uint8_t gpio_control_get_home_mask(void);

/* Get probe input state (true = triggered) */
bool gpio_control_get_probe(void);

/* Get hardware E-Stop state (true = active, button pressed) */
bool gpio_control_get_estop(void);

/* Set spindle enable output */
void gpio_control_set_spindle(bool enabled);

/* Set status LED */
void gpio_control_set_led(bool on);

/* Reload inversion settings from NVS (call after config change) */
void gpio_control_reload_inversion(void);

/* Charge pump (PWM output, board-dependent) */
void gpio_control_set_charge_pump(bool enabled);

/* Misc outputs (board-dependent, 0..4) */
void gpio_control_set_misc_output(int idx, bool state);
uint8_t gpio_control_get_misc_output_mask(void);

/* Misc inputs (board-dependent, 0..4) */
uint8_t gpio_control_get_misc_input_mask(void);

/* Get board capability flags for handshake */
uint8_t gpio_control_get_capabilities(void);

#endif /* GPIO_CONTROL_H */
