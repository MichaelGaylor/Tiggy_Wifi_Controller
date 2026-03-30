/*
 * WiFi CNC Controller - Motion Control Task
 *
 * Bridges the planner ring buffer and the stepper engine. Runs on Core 1.
 * Pops segments from the buffer and feeds them to the stepper ISR.
 */

#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "../../../protocol/wifi_cnc_protocol.h"

/* FreeRTOS task function - runs on Core 1 */
void motion_control_task(void *pvParameters);

/* Get current machine state */
wcnc_machine_state_t motion_get_state(void);

/* Trigger E-Stop from any context */
void motion_estop(void);

/* Reset from E-Stop / alarm state */
void motion_reset(void);

/* Feed hold / resume */
void motion_feed_hold(void);
void motion_feed_resume(void);

/* Start homing cycle for specified axes (bitmask) */
void motion_home(uint8_t axis_mask);

/* Check if homing is in progress */
bool motion_is_homing(void);
uint8_t motion_homing_state(void);

#endif /* MOTION_CONTROL_H */
