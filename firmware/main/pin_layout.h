/*
 * WiFi CNC Controller - What the terminals are used for
 *
 * pin_config.h gives every function of a board its terminal for six axes.
 * Six axes leave little else: on the octal-memory DevKitC no terminal is
 * left for a spindle sensor, a charge pump or an auxiliary output.  A lathe
 * that cuts threads, and many a mill, would rather have four axes and the
 * inputs and outputs.  The setting WCNC_CFG_MACHINE_LAYOUT chooses, with
 * one number instead of a dozen pin settings to get right:
 *
 *   0  six axes     X, Y, Z, A, B, C, as in pin_config.h
 *   1  four axes    X, Y, Z, A.  The terminals of B and C become inputs and
 *                   outputs: the spindle sensor, auxiliary inputs, a charge
 *                   pump, auxiliary outputs.  Ethernet stays as it is.
 *
 * The same four-axis layout serves a lathe and a mill.  The spindle sensor
 * takes only the inputs its kind needs (WCNC_CFG_ENCODER_PPR and _MODE);
 * every sensor terminal it does not use is an auxiliary input, so a mill
 * with no sensor has them all.
 *
 * No terminal changes between input and output with any of this: the step
 * and direction terminals of B and C become outputs, their limit inputs the
 * encoder's or auxiliary inputs, and the sensor only ever shares a terminal
 * with an input.  So a sensor left wired while the setting is changed is
 * never driven against, and a board that buffers its terminals one way
 * round can still use them.
 *
 * The layout only decides the pin of a function that has none stored for
 * it: a pin set explicitly (WCNC_CFG_PIN_*) always wins.  Like the pins it
 * is read once, at start-up.
 *
 * Four axes, by board ("or": what the terminal is when the sensor leaves it
 * free):
 *
 *   ESP32-S3-DevKitC, octal memory (N16R8, N8R8)
 *       encoder A               GPIO 16   or auxiliary input 3
 *       encoder B               GPIO 17   or auxiliary input 4
 *       index / single sensor   GPIO 21   or auxiliary input 2
 *                               (a single sensor with six axes too)
 *       auxiliary input 1       GPIO 18   (as with six axes)
 *       auxiliary output 1-3    GPIO 11, 12, 40
 *       charge pump             GPIO 39
 *
 *   ESP32-S3-DevKitC, quad memory (N8, N8R2)
 *       encoder A               GPIO 16   or auxiliary input 4
 *       encoder B               GPIO 17
 *       index / single sensor   GPIO 37   (a single sensor with six axes too)
 *       auxiliary input 1-3     GPIO 18, 21, 36   (as with six axes)
 *       auxiliary output 1, 2   GPIO 11, 12       (as with six axes)
 *       auxiliary output 3-5    GPIO 40, 39, 34
 *       charge pump             GPIO 10           (as with six axes)
 *
 *   ESP32-S3-Zero: it has three motor outputs and its sensor terminals
 *   already, so the two layouts are the same.  Encoder A, B and index are
 *   GPIO 15, 16, 17; those the sensor leaves free are auxiliary inputs 1-3.
 *
 *   ESP32-WROOM-32: no terminals to spare; the setting changes nothing.
 */

#ifndef PIN_LAYOUT_H
#define PIN_LAYOUT_H

#include <stdint.h>
#include "driver/gpio.h"

enum { PIN_LAYOUT_SIX_AXES = 0, PIN_LAYOUT_FOUR_AXES = 1 };

/* The spindle sensor that is set up (WCNC_CFG_ENCODER_PPR, _MODE) */
enum {
    PIN_SENSOR_NONE = 0,
    PIN_SENSOR_ENCODER_INDEX,   /* encoder with A, B and an index pulse (mode 0) */
    PIN_SENSOR_PULSE,           /* one sensor giving pulses, on the index terminal (mode 1) */
    PIN_SENSOR_ENCODER,         /* encoder with A and B only (mode 2) */
};

/* The stored layout and sensor (read from the configuration on first use). */
int pin_layout(void);
int pin_layout_sensor(void);

/* Read both again.  (They are read once, like the pins; this is for tests.) */
void pin_layout_reload(void);

/* The terminal a function has when no pin has been stored for it.  'key' is
 * its configuration key (WCNC_CFG_PIN_*).  GPIO_NUM_NC: the board, laid out
 * this way and with this sensor, has none. */
gpio_num_t pin_layout_default(uint16_t key);

#endif /* PIN_LAYOUT_H */
