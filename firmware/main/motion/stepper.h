/*
 * WiFi CNC Controller - Stepper hardware
 *
 * The hardware side of the step engine (step_engine.h): the step, direction
 * and enable pins and the hardware timer whose interrupt generates the
 * pulses.  Motion itself is queued through the motion core.
 */

#ifndef STEPPER_H
#define STEPPER_H

#include <stdbool.h>
#include <stdint.h>
#include "../../../protocol/wifi_cnc_protocol.h"

/* Configure the step, direction and enable pins.  Call once at start-up. */
void stepper_init(void);

/* Create and start the step timer.  Call from the task that is pinned to
 * the core the step interrupt should run on. */
void stepper_start_timer(void);

/* Read pulse timing and pin polarity from the stored configuration and
 * apply them.  Call at start-up and whenever one of them is changed. */
void stepper_apply_timing_config(void);

/* Per-axis rate and acceleration changed: tell the motion core. */
void stepper_apply_axis_config(void);

/* Enable or release the motor drivers (enable pin). */
void stepper_set_enabled(bool enabled);

/* True while anything is moving or still waiting to be played. */
bool stepper_is_running(void);

/* True if the last probing move ended by contact. */
bool stepper_probe_triggered(void);

/* Current position in steps. */
void stepper_get_position(int32_t position[WCNC_MAX_AXES]);

/* Current speed of the fastest axis in steps/sec. */
int32_t stepper_get_feed_rate(void);

#endif /* STEPPER_H */
