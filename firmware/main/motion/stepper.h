/*
 * WiFi CNC Controller - Stepper Engine
 *
 * Hardware timer-driven step pulse generation using Bresenham multi-axis
 * algorithm. Runs entirely on Core 1 via hardware timer ISR.
 *
 * Public interface for loading segments and controlling the stepper state.
 */

#ifndef STEPPER_H
#define STEPPER_H

#include <stdbool.h>
#include <stdint.h>
#include "../../../protocol/wifi_cnc_protocol.h"

/* Initialize the stepper engine: configure GPIO, set up hardware timer */
void stepper_init(void);

/* Load a motion segment into the stepper. The stepper will begin
 * executing it immediately (or after the current segment completes
 * if called while running). Called from motion_control_task on Core 1. */
void stepper_load_segment(const wcnc_motion_segment_t *seg);

/* Start continuous jogging on a single axis */
void stepper_start_jog(uint8_t axis, int8_t direction, uint32_t speed_steps_per_sec);

/* Stop jogging (controlled deceleration) */
void stepper_stop_jog(void);

/* Emergency stop: immediately disable all outputs */
void stepper_estop(void);

/* Reset from E-Stop state (re-enable outputs) */
void stepper_reset(void);

/* Feed hold: controlled deceleration to zero */
void stepper_feed_hold(void);

/* Resume from feed hold */
void stepper_feed_resume(void);

/* Enable/disable stepper drivers via enable pin */
void stepper_set_enabled(bool enabled);

/* Query state */
bool stepper_is_running(void);
bool stepper_segment_complete(void);
bool stepper_is_estopped(void);
bool stepper_is_holding(void);
bool stepper_probe_triggered(void);

/* Get current absolute position in steps (thread-safe atomic reads) */
void stepper_get_position(int32_t position[WCNC_MAX_AXES]);

/* Reset position counters to zero */
void stepper_zero_position(void);

/* Get current actual feed rate in steps/sec */
int32_t stepper_get_feed_rate(void);

/* Apply timing config (step pulse width, dir setup time) from NVS.
 * Called during init and when config changes at runtime. */
void stepper_apply_timing_config(void);

/* Apply per-axis max rate and acceleration from NVS.
 * Called during init and when axis config changes at runtime. */
void stepper_apply_axis_config(void);

#endif /* STEPPER_H */
