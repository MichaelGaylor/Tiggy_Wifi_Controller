/*
 * WiFi CNC Controller - Arc Interpolation (G2/G3)
 *
 * Linearizes circular/helical arcs into short line segments fed to
 * the look-ahead planner. Supports IJK center offset and R radius
 * formats in all three planes (G17/G18/G19).
 */

#ifndef GCODE_ARCS_H
#define GCODE_ARCS_H

#include "gcode_parser.h"
#include "gcode_planner.h"

/* Default arc tolerance (mm) — overridden by NVS $12 */
#define GCODE_DEFAULT_ARC_TOLERANCE  0.002f

/* Maximum segments per arc (safety limit) */
#define GCODE_ARC_MAX_SEGMENTS       2000

/*
 * Linearize an arc (G2/G3) into line segments pushed to the planner.
 *
 * Parameters:
 *   planner    - Look-ahead planner to push segments to
 *   position   - Current machine position (mm)
 *   target     - Final arc endpoint (mm, absolute machine coords)
 *   ijk        - Center offset from current position (I, J, K)
 *   r_value    - R radius (used if no IJK specified)
 *   use_radius - true = R format, false = IJK format
 *   clockwise  - true = G2 (CW), false = G3 (CCW)
 *   plane      - Active plane (PLANE_XY, PLANE_ZX, PLANE_YZ)
 *   feed_rate  - Feed rate in mm/min
 *   tolerance  - Arc segment tolerance in mm ($12)
 */
void gcode_arc_execute(gcode_planner_t *planner,
                       const float position[GCODE_MAX_AXES],
                       const float target[GCODE_MAX_AXES],
                       const float ijk[3],
                       float r_value,
                       bool use_radius,
                       bool clockwise,
                       gcode_plane_t plane,
                       float feed_rate,
                       float tolerance);

#endif /* GCODE_ARCS_H */
