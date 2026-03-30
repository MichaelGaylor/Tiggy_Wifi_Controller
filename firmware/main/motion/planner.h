/*
 * WiFi CNC Controller - Motion Segment Planner Ring Buffer
 *
 * Thread-safe ring buffer that bridges the network receive task (Core 0)
 * and the motion control task (Core 1). Uses FreeRTOS spinlock for
 * ISR-safe access.
 */

#ifndef PLANNER_H
#define PLANNER_H

#include <stdbool.h>
#include <stdint.h>
#include "../config.h"
#include "../../../protocol/wifi_cnc_protocol.h"

/* Initialize the planner ring buffer */
void planner_init(void);

/* Push a motion segment into the buffer (called from network task, Core 0).
 * Returns true on success, false if buffer is full. */
bool planner_push_segment(const wcnc_motion_segment_t *seg);

/* Pop a motion segment from the buffer (called from motion task, Core 1).
 * Returns true on success, false if buffer is empty. */
bool planner_pop_segment(wcnc_motion_segment_t *seg);

/* Get the number of free slots available for writing */
uint16_t planner_available(void);

/* Get the number of segments waiting to be consumed */
uint16_t planner_count(void);

/* Flush all segments (used during E-Stop) */
void planner_clear(void);

/* Check if buffer is empty */
bool planner_is_empty(void);

/* Check if buffer is full */
bool planner_is_full(void);

/* Get total buffer capacity */
uint16_t planner_capacity(void);

#endif /* PLANNER_H */
