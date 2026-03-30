/*
 * WiFi CNC Controller - Jog Command Handler
 *
 * Parses and executes GRBL $J= jog commands.
 * Format: $J=G91 X10 F1000
 * Supports G90/G91, G20/G21, any axis, F feed rate.
 */

#ifndef GCODE_JOG_H
#define GCODE_JOG_H

#include "gcode_parser.h"
#include "gcode_planner.h"

/*
 * Execute a jog command.
 *
 * Parameters:
 *   args     - Command string after "$J=" (e.g., "G91 X10 F1000")
 *   state    - Current parser state (for position, units)
 *   planner  - Motion planner for segment generation
 *
 * Returns GCODE_OK or error code.
 */
gcode_error_t gcode_jog_execute(const char *args,
                                 gcode_state_t *state,
                                 gcode_planner_t *planner);

#endif /* GCODE_JOG_H */
