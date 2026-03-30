/*
 * WiFi CNC Controller - Jog Command Handler
 *
 * Parses $J= jog commands. The jog line is a minimal G-code subset:
 * only G90/G91 (distance mode), G20/G21 (units), axis words, and F.
 *
 * Jog moves use the RAPID flag and cancel on feed hold.
 */

#include "gcode_jog.h"

#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

#include "../../protocol/wifi_cnc_protocol.h"

gcode_error_t gcode_jog_execute(const char *args,
                                 gcode_state_t *state,
                                 gcode_planner_t *planner)
{
    bool incremental = state->incremental;
    bool inches = state->inches;
    float feed_rate = 0.0f;
    float axis_values[GCODE_MAX_AXES];
    uint32_t axis_bits = 0;

    memset(axis_values, 0, sizeof(axis_values));

    /* Parse the jog command arguments */
    const char *p = args;
    while (*p) {
        /* Skip whitespace */
        while (*p == ' ') p++;
        if (*p == '\0') break;

        char letter = toupper((unsigned char)*p);
        p++;

        /* Parse value */
        char *end;
        float val = strtof(p, &end);
        if (end == p) {
            return GCODE_ERR_INVALID_GCODE;
        }
        p = end;

        switch (letter) {
        case 'G':
            switch ((int)lroundf(val)) {
            case 90: incremental = false; break;
            case 91: incremental = true; break;
            case 20: inches = true; break;
            case 21: inches = false; break;
            default:
                return GCODE_ERR_UNSUPPORTED_CMD;
            }
            break;

        case 'X': axis_values[0] = val; axis_bits |= (1 << 0); break;
        case 'Y': axis_values[1] = val; axis_bits |= (1 << 1); break;
        case 'Z': axis_values[2] = val; axis_bits |= (1 << 2); break;
        case 'A': axis_values[3] = val; axis_bits |= (1 << 3); break;
        case 'B': axis_values[4] = val; axis_bits |= (1 << 4); break;
        case 'C': axis_values[5] = val; axis_bits |= (1 << 5); break;
        case 'F': feed_rate = val; break;
        default:
            return GCODE_ERR_INVALID_GCODE;
        }
    }

    /* Feed rate is required for jog */
    if (feed_rate <= 0.0f) {
        return GCODE_ERR_UNDEFINED_FEED;
    }

    /* Must have at least one axis */
    if (axis_bits == 0) {
        return GCODE_ERR_INVALID_GCODE;
    }

    /* Unit conversion */
    if (inches) {
        feed_rate *= 25.4f;
        for (int i = 0; i < GCODE_MAX_AXES; i++) {
            axis_values[i] *= 25.4f;
        }
    }

    /* Compute target position */
    float target[GCODE_MAX_AXES];
    for (int i = 0; i < GCODE_MAX_AXES; i++) {
        if (axis_bits & (1 << i)) {
            if (incremental) {
                target[i] = state->position[i] + axis_values[i];
            } else {
                /* Absolute jog uses machine coordinates */
                target[i] = axis_values[i];
            }
        } else {
            target[i] = state->position[i];
        }
    }

    /* Push jog move — use RAPID flag for fast motion */
    gcode_planner_line(planner, target, feed_rate, false,
                       WCNC_SEG_FLAG_RAPID);

    /* Update state position */
    memcpy(state->position, target, sizeof(target));

    return GCODE_OK;
}
