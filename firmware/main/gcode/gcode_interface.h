/*
 * WiFi CNC Controller - G-Code Interface
 *
 * Central G-code execution task and transport-agnostic interface.
 * Receives lines from telnet/UART, parses, plans, and emits segments.
 * Handles real-time commands (?, !, ~, Ctrl-X) and GRBL status reports.
 */

#ifndef GCODE_INTERFACE_H
#define GCODE_INTERFACE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Output callback type (shared with grbl_settings.h) */
typedef void (*gcode_output_fn)(const char *str, size_t len, void *ctx);

/* Real-time command byte values */
#define GRBL_RT_STATUS_QUERY    '?'
#define GRBL_RT_FEED_HOLD       '!'
#define GRBL_RT_CYCLE_START     '~'
#define GRBL_RT_SOFT_RESET      0x18    /* Ctrl-X */
#define GRBL_RT_JOG_CANCEL      0x85

/* Is this byte a real-time command?  As in Grbl 1.1, every byte from 0x80 up
 * is one and never part of a line - those this firmware does not act on
 * (the overrides) are dropped rather than left to spoil the line around them. */
#define GRBL_IS_REALTIME(b) ((b) == GRBL_RT_STATUS_QUERY || (b) == GRBL_RT_FEED_HOLD || \
                             (b) == GRBL_RT_CYCLE_START || (b) == GRBL_RT_SOFT_RESET || (b) >= 0x80)

/* Initialize the G-code interface (creates queue, inits parser+planner) */
void gcode_interface_init(void);

/* Submit a line to the execution queue (from telnet or UART task).
 * Line must be null-terminated, max 256 chars. */
void gcode_interface_submit_line(const char *line);

/* Process a real-time command byte (from any task context).
 * These are handled immediately, not queued. */
void gcode_handle_realtime(uint8_t cmd);

/* Set the output function for G-code responses (ok, error, status).
 * Each transport (telnet/UART) sets this when it becomes active. */
void gcode_interface_set_output(gcode_output_fn fn, void *ctx);

/* Generate GRBL-format status report string.
 * Returns length written. Format: <State|MPos:x,y,z|FS:f,s|WCO:...> */
int gcode_format_status_report(char *buf, size_t buf_size);

/* FreeRTOS task: G-code execution loop */
void gcode_exec_task(void *pvParameters);

/* Check if G-code mode is active (telnet or UART connected) */
bool gcode_is_active(void);

/* Binary protocol coexistence: set preemption flag on G-code planner.
 * Call when binary protocol starts sending motion segments.
 * G-code planner will pause emission until released. */
void gcode_planner_preempt(void);

/* Release preemption: allow G-code planner to resume emission. */
void gcode_planner_release(void);

#endif /* GCODE_INTERFACE_H */
