/*
 * WiFi CNC Controller - GPIO Control
 *
 * Inputs (limit / home switches, probe, E-stop, auxiliary inputs) and
 * outputs (spindle on/off or PWM, charge pump, auxiliary outputs).
 */

#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#define GPIO_MISC_OUTPUTS   5
#define GPIO_MISC_INPUTS    4

/* Configure all auxiliary pins */
void gpio_control_init(void);

/* Sample and debounce the inputs.  Call once a millisecond. */
void gpio_control_poll(void);

/* Read the polarity and debounce settings again after a configuration change */
void gpio_control_reload_inversion(void);

/* --- Inputs (debounced; true / bit set = switch operated) --- */
uint8_t gpio_control_get_limit_mask(void);  /* bit per limit input, limit polarity */
uint8_t gpio_control_get_home_mask(void);   /* same inputs, home polarity */
bool    gpio_control_get_limit(int axis);
bool    gpio_control_get_probe(void);
bool    gpio_control_get_estop(void);
/* Auxiliary inputs: bit set = input pulled low (a switch to ground is closed) */
uint8_t gpio_control_get_misc_input_mask(void);

/* --- Outputs --- */
/* Everything at once, as commanded by a host.
 * spindle_state: 0 off, 1 clockwise, 2 counter-clockwise. */
void gpio_control_apply_outputs(uint8_t misc, uint8_t spindle_state, uint16_t rpm, bool charge_pump);

void gpio_control_set_spindle(bool enabled);            /* full on / off (G-code mode) */
void gpio_control_set_charge_pump(bool enabled);
void gpio_control_set_misc_output(int idx, bool state);
uint8_t gpio_control_get_misc_output_mask(void);

/* Get board capability flags (WCNC_CAP_*) */
uint8_t gpio_control_get_capabilities(void);

#endif /* GPIO_CONTROL_H */
