/*
 * WiFi CNC Controller - Arc Interpolation (G2/G3)
 *
 * Converts arcs into short line segments. Uses the chord-error method
 * to determine segment count: more segments for tighter radii and
 * smaller tolerance settings.
 *
 * Helical motion: the axis perpendicular to the arc plane is linearly
 * interpolated along with the arc, producing a helix.
 */

#include "gcode_arcs.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"

static const char *TAG = "gcode_arc";

/* Map plane enum to axis indices: [arc_axis_0, arc_axis_1, linear_axis] */
static void plane_axes(gcode_plane_t plane, int *a0, int *a1, int *linear)
{
    switch (plane) {
    case PLANE_XY: *a0 = 0; *a1 = 1; *linear = 2; break;
    case PLANE_ZX: *a0 = 2; *a1 = 0; *linear = 1; break;
    case PLANE_YZ: *a0 = 1; *a1 = 2; *linear = 0; break;
    default:       *a0 = 0; *a1 = 1; *linear = 2; break;
    }
}

void gcode_arc_execute(gcode_planner_t *planner,
                       const float position[GCODE_MAX_AXES],
                       const float target[GCODE_MAX_AXES],
                       const float ijk[3],
                       float r_value,
                       bool use_radius,
                       bool clockwise,
                       gcode_plane_t plane,
                       float feed_rate,
                       float tolerance)
{
    int a0, a1, linear;
    plane_axes(plane, &a0, &a1, &linear);

    float center[2];    /* Arc center in plane coords */
    float radius;

    if (use_radius) {
        /* R format: compute center from start, end, and radius */
        float dx = target[a0] - position[a0];
        float dy = target[a1] - position[a1];
        float d_sq = dx * dx + dy * dy;
        float d = sqrtf(d_sq);

        if (d < 1e-6f) {
            ESP_LOGW(TAG, "Arc: start == end with R format");
            return;
        }

        radius = fabsf(r_value);
        if (radius < d / 2.0f) {
            radius = d / 2.0f;  /* Clamp to minimum valid radius */
        }

        float h_sq = radius * radius - d_sq / 4.0f;
        if (h_sq < 0.0f) h_sq = 0.0f;
        float h = sqrtf(h_sq);

        /* Negative R means >180 degree arc */
        if (r_value < 0.0f) h = -h;

        /* CW/CCW determines which side of the chord the center is */
        float mx = (position[a0] + target[a0]) / 2.0f;
        float my = (position[a1] + target[a1]) / 2.0f;

        if (clockwise) {
            center[0] = mx + h * dy / d;
            center[1] = my - h * dx / d;
        } else {
            center[0] = mx - h * dy / d;
            center[1] = my + h * dx / d;
        }
    } else {
        /* IJK format: center = start + IJK offset (mapped to plane axes) */
        float ijk_mapped[2];
        switch (plane) {
        case PLANE_XY: ijk_mapped[0] = ijk[0]; ijk_mapped[1] = ijk[1]; break;
        case PLANE_ZX: ijk_mapped[0] = ijk[2]; ijk_mapped[1] = ijk[0]; break;
        case PLANE_YZ: ijk_mapped[0] = ijk[1]; ijk_mapped[1] = ijk[2]; break;
        default:       ijk_mapped[0] = ijk[0]; ijk_mapped[1] = ijk[1]; break;
        }

        center[0] = position[a0] + ijk_mapped[0];
        center[1] = position[a1] + ijk_mapped[1];
        radius = sqrtf(ijk_mapped[0] * ijk_mapped[0] +
                        ijk_mapped[1] * ijk_mapped[1]);
    }

    if (radius < 1e-6f) {
        ESP_LOGW(TAG, "Arc: zero radius");
        return;
    }

    /* Compute start and end angles */
    float start_angle = atan2f(position[a1] - center[1],
                               position[a0] - center[0]);
    float end_angle   = atan2f(target[a1] - center[1],
                               target[a0] - center[0]);

    /* Compute angular travel */
    float angular_travel = end_angle - start_angle;

    if (clockwise) {
        if (angular_travel >= 0.0f) angular_travel -= 2.0f * (float)M_PI;
    } else {
        if (angular_travel <= 0.0f) angular_travel += 2.0f * (float)M_PI;
    }

    /* Full circle: if start == end in the arc plane */
    float dp0 = target[a0] - position[a0];
    float dp1 = target[a1] - position[a1];
    if (fabsf(dp0) < 1e-5f && fabsf(dp1) < 1e-5f) {
        if (clockwise)  angular_travel = -2.0f * (float)M_PI;
        else            angular_travel =  2.0f * (float)M_PI;
    }

    /* Compute number of segments based on chord error tolerance */
    /* chord_error = radius * (1 - cos(theta/2)) ≈ tolerance */
    /* cos(theta/2) = 1 - tolerance/radius */
    /* theta = 2 * acos(1 - tolerance/radius) */
    float abs_travel = fabsf(angular_travel);
    int segments;

    if (tolerance >= radius) {
        segments = 4;  /* Minimum for very coarse tolerance */
    } else {
        float theta_per_seg = 2.0f * acosf(1.0f - tolerance / radius);
        if (theta_per_seg < 1e-6f) {
            segments = GCODE_ARC_MAX_SEGMENTS;
        } else {
            segments = (int)ceilf(abs_travel / theta_per_seg);
        }
    }

    /* Clamp segment count */
    if (segments < 1) segments = 1;
    if (segments > GCODE_ARC_MAX_SEGMENTS) segments = GCODE_ARC_MAX_SEGMENTS;

    /* Linear axis travel (helix component) */
    float linear_travel = target[linear] - position[linear];

    /* Generate line segments */
    float theta_per_seg = angular_travel / (float)segments;
    float linear_per_seg = linear_travel / (float)segments;

    float seg_target[GCODE_MAX_AXES];

    /* Copy non-arc axes from target (they don't change during arc) */
    memcpy(seg_target, target, sizeof(float) * GCODE_MAX_AXES);

    for (int s = 1; s <= segments; s++) {
        if (s == segments) {
            /* Last segment: use exact target to avoid float drift */
            gcode_planner_line(planner, target, feed_rate, false, 0);
        } else {
            float angle = start_angle + theta_per_seg * (float)s;
            seg_target[a0] = center[0] + radius * cosf(angle);
            seg_target[a1] = center[1] + radius * sinf(angle);
            seg_target[linear] = position[linear] + linear_per_seg * (float)s;

            gcode_planner_line(planner, seg_target, feed_rate, false, 0);
        }
    }
}
