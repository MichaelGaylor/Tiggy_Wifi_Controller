/*
 * WiFi CNC Controller - G-Code Look-Ahead Motion Planner
 *
 * Implements GRBL-style junction deviation velocity planning with
 * forward/backward passes for smooth constant-velocity motion.
 *
 * This planner sits above the existing ring buffer (planner.h/planner.c).
 * It converts G-code moves in mm-space to pre-planned motion segments
 * with correct entry/exit speeds, then pushes them downstream.
 */

#include "gcode_planner.h"
#include "planner.h"
#include "stepper.h"
#include "config.h"
#include "persist/nvs_config.h"
#include "../../protocol/wifi_cnc_protocol.h"

#include <string.h>
#include <math.h>
#include <float.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ===================================================================
 * Internal Helpers
 * =================================================================== */

static inline uint8_t next_idx(uint8_t idx)
{
    return (idx + 1) & GCODE_PLAN_BUFFER_MASK;
}

static inline uint8_t prev_idx(uint8_t idx)
{
    return (idx - 1 + GCODE_PLAN_BUFFER_SIZE) & GCODE_PLAN_BUFFER_MASK;
}

/* ===================================================================
 * Initialization
 * =================================================================== */

void gcode_planner_init(gcode_planner_t *planner)
{
    memset(planner, 0, sizeof(*planner));

    planner->junction_deviation = GCODE_DEFAULT_JUNCTION_DEV;
    planner->exact_stop_mode = false;
    planner->path_tolerance = 0.0f;
    planner->prev_unit_valid = false;

    gcode_planner_reload_config(planner);
    gcode_planner_sync_position(planner);
}

/* ===================================================================
 * Config Reload
 * =================================================================== */

void gcode_planner_reload_config(gcode_planner_t *planner)
{
    /* Load steps/mm */
    const char *spm_keys[] = {"spm_x","spm_y","spm_z","spm_a","spm_b","spm_c"};
    for (int i = 0; i < GCODE_MAX_AXES; i++) {
        planner->steps_per_mm[i] = nvs_config_get_float(spm_keys[i],
                                                          CFG_DEFAULT_STEPS_PER_MM);
        if (planner->steps_per_mm[i] <= 0.0f) {
            planner->steps_per_mm[i] = CFG_DEFAULT_STEPS_PER_MM;
        }
    }

    /* Load max rates (stored in NVS as steps/sec, convert to mm/sec) */
    const char *rate_keys[] = {"rate_x","rate_y","rate_z","rate_a","rate_b","rate_c"};
    for (int i = 0; i < GCODE_MAX_AXES; i++) {
        uint32_t rate_sps = nvs_config_get_u32(rate_keys[i], CFG_DEFAULT_MAX_RATE);
        planner->max_rate_mm_sec[i] = (float)rate_sps / planner->steps_per_mm[i];
    }

    /* Load accelerations (stored in NVS as steps/sec^2, convert to mm/sec^2) */
    const char *acc_keys[] = {"acc_x","acc_y","acc_z","acc_a","acc_b","acc_c"};
    for (int i = 0; i < GCODE_MAX_AXES; i++) {
        uint32_t acc_sps2 = nvs_config_get_u32(acc_keys[i], CFG_DEFAULT_ACCELERATION);
        planner->max_accel_mm_sec2[i] = (float)acc_sps2 / planner->steps_per_mm[i];
    }

    /* Junction deviation from NVS ($11) */
    planner->junction_deviation = nvs_config_get_float("junc_dev",
                                                        GCODE_DEFAULT_JUNCTION_DEV);
    if (planner->junction_deviation <= 0.0f) {
        planner->junction_deviation = GCODE_DEFAULT_JUNCTION_DEV;
    }
}

/* ===================================================================
 * Position Sync
 * =================================================================== */

void gcode_planner_sync_position(gcode_planner_t *planner)
{
    stepper_get_position(planner->position_steps);
    for (int i = 0; i < GCODE_MAX_AXES; i++) {
        planner->position_mm[i] = (float)planner->position_steps[i]
                                  / planner->steps_per_mm[i];
    }
}

/* ===================================================================
 * Junction Velocity Computation (GRBL method)
 *
 * Computes the maximum safe speed at the junction between two moves
 * based on the angle between their direction vectors and a deviation
 * tolerance parameter.
 * =================================================================== */

static float compute_junction_speed(const gcode_planner_t *planner,
                                     const gcode_plan_block_t *block)
{
    if (!planner->prev_unit_valid) return 0.0f;

    /* Dot product of previous and current unit vectors.
     * We negate because we want the angle between the exit direction
     * of the previous move and the entry direction of the current move.
     * Straight line: cos_theta = -1 (angle 180), 90-deg turn: cos_theta = 0 */
    float cos_theta = 0.0f;
    for (int i = 0; i < GCODE_MAX_AXES; i++) {
        cos_theta -= planner->prev_unit_vec[i] * block->unit_vec[i];
    }

    /* Nearly straight line — no speed limit from junction */
    if (cos_theta < -0.9999f) {
        return block->nominal_speed;
    }

    /* Direction reversal — must stop */
    if (cos_theta > 0.9999f) {
        return 0.0f;
    }

    /* Junction deviation formula:
     * sin(theta/2) = sqrt((1 - cos_theta) / 2)
     * v = sqrt(accel * deviation * sin(theta/2) / (1 - sin(theta/2)))
     */
    float sin_half = sqrtf(0.5f * (1.0f - cos_theta));
    if (sin_half < 0.001f) sin_half = 0.001f;

    /* Use effective deviation (max of junction_deviation and path_tolerance) */
    float deviation = planner->junction_deviation;
    if (planner->path_tolerance > deviation) {
        deviation = planner->path_tolerance;
    }

    float v_junc_sqr = block->acceleration * deviation
                       * sin_half / (1.0f - sin_half);
    float v_junction = sqrtf(v_junc_sqr);

    /* Clamp to the smaller nominal speed of the two blocks */
    uint8_t prev = prev_idx(planner->head);
    float prev_nominal = planner->buffer[prev].nominal_speed;
    if (v_junction > prev_nominal) v_junction = prev_nominal;
    if (v_junction > block->nominal_speed) v_junction = block->nominal_speed;

    return v_junction;
}

/* ===================================================================
 * Backward and Forward Passes (Velocity Re-planning)
 *
 * After adding a new block, we run backward from newest to oldest
 * and then forward from oldest to newest to compute achievable
 * entry/exit speeds given kinematic constraints.
 * =================================================================== */

static void planner_recalculate(gcode_planner_t *planner)
{
    if (planner->count < 2) return;

    /* --- Backward pass --- */
    /* Walk from newest block toward tail. Ensure each block can
     * decelerate from its entry speed to the next block's entry speed. */
    uint8_t idx = prev_idx(planner->head);  /* Newest block */
    float next_entry_speed = planner->buffer[idx].entry_speed;

    for (int i = planner->count - 2; i >= 0; i--) {
        idx = prev_idx(idx);
        gcode_plan_block_t *current = &planner->buffer[idx];

        if (!current->recalculate) break;

        /* Maximum exit speed achievable:
         * v_exit^2 = v_entry^2 + 2 * a * d */
        float max_exit_sqr = current->entry_speed * current->entry_speed
                             + 2.0f * current->acceleration * current->distance_mm;
        float max_exit = sqrtf(max_exit_sqr);

        if (max_exit < next_entry_speed) {
            /* Can't reach required speed — limit next block's entry */
            uint8_t next = next_idx(idx);
            gcode_plan_block_t *next_block = &planner->buffer[next];
            next_block->entry_speed = max_exit;
            if (next_block->entry_speed > next_block->max_entry_speed) {
                next_block->entry_speed = next_block->max_entry_speed;
            }
        }

        next_entry_speed = current->entry_speed;
    }

    /* --- Forward pass --- */
    /* Walk from oldest to newest. Ensure each block can accelerate
     * from its entry speed to the next block's entry speed. */
    idx = planner->tail;
    for (int i = 0; i < planner->count - 1; i++) {
        gcode_plan_block_t *current = &planner->buffer[idx];
        uint8_t nxt = next_idx(idx);
        gcode_plan_block_t *next_block = &planner->buffer[nxt];

        /* Maximum speed achievable at end of this block */
        float max_exit_sqr = current->entry_speed * current->entry_speed
                             + 2.0f * current->acceleration * current->distance_mm;
        float max_exit = sqrtf(max_exit_sqr);

        if (max_exit < next_block->entry_speed) {
            next_block->entry_speed = max_exit;
        }

        current->recalculate = false;
        idx = nxt;
    }
}

/* ===================================================================
 * Emit Block to Ring Buffer
 *
 * Converts a planned block (mm-space) to a motion segment (step-space)
 * and pushes it to the existing planner ring buffer.
 * =================================================================== */

static void emit_block(gcode_planner_t *planner, float exit_speed)
{
    gcode_plan_block_t *block = &planner->buffer[planner->tail];

    wcnc_motion_segment_t seg;
    memset(&seg, 0, sizeof(seg));

    /* Copy step counts */
    for (int i = 0; i < GCODE_MAX_AXES; i++) {
        seg.steps[i] = block->steps[i];
    }

    /* Convert mm/sec speeds to steps/sec for the dominant axis */
    float mm_to_steps = 1.0f;
    if (block->distance_mm > 0.0001f) {
        mm_to_steps = (float)block->step_event_count / block->distance_mm;
    }

    float entry_sps = block->entry_speed * mm_to_steps;
    float exit_sps = exit_speed * mm_to_steps;
    float accel_sps2 = block->acceleration * mm_to_steps;

    /* Minimum speeds to avoid zero-divide in stepper */
    if (entry_sps < 1.0f) entry_sps = 1.0f;
    if (exit_sps < 1.0f) exit_sps = 1.0f;

    /* Encode in protocol format: speed^2 * 1000, accel * 100 */
    seg.entry_speed_sqr = (uint32_t)(entry_sps * entry_sps * 1000.0f);
    if (seg.entry_speed_sqr == 0) seg.entry_speed_sqr = 1;
    seg.exit_speed_sqr = (uint32_t)(exit_sps * exit_sps * 1000.0f);
    if (seg.exit_speed_sqr == 0) seg.exit_speed_sqr = 1;
    seg.acceleration = (uint32_t)(accel_sps2 * 100.0f);
    if (seg.acceleration == 0) seg.acceleration = 1;

    seg.flags = block->flags;
    seg.segment_id = block->segment_id;

    /* Push to ring buffer — block if full, yield to other tasks */
    while (!planner->preempted) {
        if (planner_push_segment(&seg)) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    /* Update step position tracking */
    for (int i = 0; i < GCODE_MAX_AXES; i++) {
        planner->position_steps[i] += block->steps[i];
    }

    /* Advance tail */
    planner->tail = next_idx(planner->tail);
    planner->count--;
    planner->blocks_emitted++;
}

/* ===================================================================
 * Adaptive Emission
 *
 * Monitors ring buffer fill level and adjusts look-ahead depth.
 * Lower look-ahead = faster emission = keeps ring buffer fed.
 * Higher look-ahead = better velocity planning = smoother motion.
 * =================================================================== */

static void planner_try_emit(gcode_planner_t *planner)
{
    if (planner->count == 0) return;
    if (planner->preempted) return;

    /* Determine minimum look-ahead based on ring buffer fill level */
    uint16_t ring_avail = planner_available();
    uint16_t ring_cap = planner_capacity();
    int min_look_ahead;

    if (ring_avail > (uint16_t)(ring_cap * GCODE_EMIT_EMERGENCY_THRESH)) {
        /* Ring buffer critically empty — emit immediately */
        min_look_ahead = GCODE_EMIT_MIN_LOOKAHEAD;
        /* Track underruns: ring buffer empty while we have blocks */
        if (planner_is_empty() && planner->count > 0) {
            planner->underrun_count++;
        }
    } else if (ring_avail > (uint16_t)(ring_cap * GCODE_EMIT_LOW_THRESH)) {
        /* Ring buffer getting low */
        min_look_ahead = GCODE_EMIT_LOW_LOOKAHEAD;
    } else {
        /* Ring buffer healthy — use full look-ahead */
        min_look_ahead = GCODE_EMIT_NORMAL_LOOKAHEAD;
    }

    /* Emit oldest blocks that have settled */
    while (planner->count > min_look_ahead && !planner->preempted) {
        planner_recalculate(planner);

        /* Exit speed of emitted block = entry speed of next block */
        float exit_speed;
        if (planner->count > 1) {
            uint8_t nxt = next_idx(planner->tail);
            exit_speed = planner->buffer[nxt].entry_speed;
        } else {
            /* Last block — decelerate to zero */
            exit_speed = 0.0f;
        }

        emit_block(planner, exit_speed);
    }
}

/* ===================================================================
 * Add Linear Move
 * =================================================================== */

void gcode_planner_line(gcode_planner_t *planner,
                         const float target[GCODE_MAX_AXES],
                         float feed_rate_mm_min,
                         bool is_rapid,
                         uint8_t flags)
{
    /* Wait if look-ahead buffer is full */
    while (planner->count >= GCODE_PLAN_BUFFER_SIZE - 1) {
        planner_try_emit(planner);
        if (planner->count >= GCODE_PLAN_BUFFER_SIZE - 1) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    gcode_plan_block_t *block = &planner->buffer[planner->head];
    memset(block, 0, sizeof(*block));

    /* --- A. Compute geometry --- */
    float dist_sqr = 0.0f;
    block->step_event_count = 0;

    for (int i = 0; i < GCODE_MAX_AXES; i++) {
        block->target[i] = target[i];
        float delta_mm = target[i] - planner->position_mm[i];
        block->steps[i] = (int32_t)lroundf(delta_mm * planner->steps_per_mm[i]);
        dist_sqr += delta_mm * delta_mm;

        uint32_t abs_steps = (uint32_t)abs(block->steps[i]);
        if (abs_steps > block->step_event_count) {
            block->step_event_count = abs_steps;
        }
    }

    block->distance_mm = sqrtf(dist_sqr);

    /* Skip zero-length moves */
    if (block->distance_mm < 0.0001f || block->step_event_count == 0) {
        return;
    }

    /* Compute unit vector */
    float inv_dist = 1.0f / block->distance_mm;
    for (int i = 0; i < GCODE_MAX_AXES; i++) {
        block->unit_vec[i] = (target[i] - planner->position_mm[i]) * inv_dist;
    }

    /* --- B. Axis-limited max speed --- */
    float feed_mm_sec = feed_rate_mm_min / 60.0f;
    if (feed_mm_sec < GCODE_MIN_FEED_RATE / 60.0f) {
        feed_mm_sec = GCODE_MIN_FEED_RATE / 60.0f;
    }

    block->programmed_rate = feed_mm_sec;
    block->nominal_speed = feed_mm_sec;
    block->max_speed = 1e6f;  /* Start large, clamp down */

    for (int i = 0; i < GCODE_MAX_AXES; i++) {
        float uv = fabsf(block->unit_vec[i]);
        if (uv > 0.0001f) {
            float axis_limit = planner->max_rate_mm_sec[i] / uv;
            if (axis_limit < block->max_speed) {
                block->max_speed = axis_limit;
            }
        }
    }

    if (block->nominal_speed > block->max_speed) {
        block->nominal_speed = block->max_speed;
    }

    /* Rapids use maximum axis speeds */
    if (is_rapid) {
        block->nominal_speed = block->max_speed;
    }

    /* --- C. Axis-limited acceleration --- */
    block->acceleration = 1e6f;  /* Start large, clamp down */

    for (int i = 0; i < GCODE_MAX_AXES; i++) {
        float uv = fabsf(block->unit_vec[i]);
        if (uv > 0.0001f) {
            float axis_accel = planner->max_accel_mm_sec2[i] / uv;
            if (axis_accel < block->acceleration) {
                block->acceleration = axis_accel;
            }
        }
    }

    /* --- D. Junction entry speed --- */
    if (planner->exact_stop_mode) {
        /* G61: must stop at every junction */
        block->max_entry_speed = 0.0f;
    } else {
        block->max_entry_speed = compute_junction_speed(planner, block);
    }
    block->entry_speed = block->max_entry_speed;

    /* --- Set flags and metadata --- */
    block->flags = flags;
    if (is_rapid) block->flags |= WCNC_SEG_FLAG_RAPID;
    if (planner->exact_stop_mode) block->flags |= WCNC_SEG_FLAG_EXACT_STOP;
    block->is_rapid = is_rapid;
    block->recalculate = true;
    block->segment_id = planner->next_segment_id++;

    /* Save unit vector for next junction calculation */
    for (int i = 0; i < GCODE_MAX_AXES; i++) {
        planner->prev_unit_vec[i] = block->unit_vec[i];
    }
    planner->prev_unit_valid = true;

    /* Update position tracking */
    for (int i = 0; i < GCODE_MAX_AXES; i++) {
        planner->position_mm[i] = target[i];
    }

    /* Advance head */
    planner->head = next_idx(planner->head);
    planner->count++;

    /* Try to emit settled blocks */
    planner_try_emit(planner);
}

/* ===================================================================
 * Dwell
 * =================================================================== */

void gcode_planner_dwell(gcode_planner_t *planner, float seconds)
{
    /* Flush all pending moves first */
    gcode_planner_sync(planner);

    /* Wait for all motion to complete */
    while (stepper_is_running() || !planner_is_empty()) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    /* Delay */
    if (seconds > 0.0f) {
        vTaskDelay(pdMS_TO_TICKS((uint32_t)(seconds * 1000.0f)));
    }
}

/* ===================================================================
 * Sync (flush all blocks to ring buffer)
 * =================================================================== */

void gcode_planner_sync(gcode_planner_t *planner)
{
    /* Flush corner-smoothing buffer first */
    gcode_planner_flush_smooth(planner);

    /* Emit all remaining blocks with decreasing look-ahead */
    while (planner->count > 0 && !planner->preempted) {
        planner_recalculate(planner);

        float exit_speed;
        if (planner->count > 1) {
            uint8_t nxt = next_idx(planner->tail);
            exit_speed = planner->buffer[nxt].entry_speed;
        } else {
            /* Last block decelerates to zero */
            exit_speed = 0.0f;
        }

        emit_block(planner, exit_speed);
    }

    /* Invalidate previous unit vector (path broken by sync) */
    planner->prev_unit_valid = false;
}

/* ===================================================================
 * Clear (E-stop / Reset)
 * =================================================================== */

void gcode_planner_clear(gcode_planner_t *planner)
{
    planner->head = 0;
    planner->tail = 0;
    planner->count = 0;
    planner->prev_unit_valid = false;
    planner->preempted = false;

    /* CRITICAL: Do NOT reset position — preserve actual machine position.
     * Call gcode_planner_sync_position() after reset to re-align. */
}

/* ===================================================================
 * Query Functions
 * =================================================================== */

bool gcode_planner_is_idle(gcode_planner_t *planner)
{
    return (planner->count == 0) && planner_is_empty() && !stepper_is_running();
}

void gcode_planner_set_path_mode(gcode_planner_t *planner,
                                  bool exact_stop,
                                  float tolerance)
{
    planner->exact_stop_mode = exact_stop;
    planner->path_tolerance = tolerance;
}

uint8_t gcode_planner_available(gcode_planner_t *planner)
{
    return GCODE_PLAN_BUFFER_SIZE - 1 - planner->count;
}

/* ===================================================================
 * Corner Smoothing (G64 P<tolerance>)
 *
 * Buffers one G1 move. When the next move arrives, checks the corner
 * angle. If G64 P is active and the angle is sharp, inserts small
 * rounding arc segments so the machine curves through the corner
 * instead of decelerating to the junction speed.
 * =================================================================== */

void gcode_planner_flush_smooth(gcode_planner_t *planner)
{
    if (!planner->smooth_pending) return;
    gcode_planner_line(planner, planner->smooth_target,
                       planner->smooth_feed, false, planner->smooth_flags);
    planner->smooth_pending = false;
}

void gcode_planner_line_smooth(gcode_planner_t *planner,
                                const float target[GCODE_MAX_AXES],
                                float feed_rate_mm_min,
                                uint8_t flags)
{
    /* If smoothing disabled, pass through directly */
    if (planner->exact_stop_mode || planner->path_tolerance <= 0.0f) {
        gcode_planner_flush_smooth(planner);
        gcode_planner_line(planner, target, feed_rate_mm_min, false, flags);
        return;
    }

    /* If no buffered move, just buffer this one */
    if (!planner->smooth_pending) {
        memcpy(planner->smooth_target, target, sizeof(float) * GCODE_MAX_AXES);
        planner->smooth_feed = feed_rate_mm_min;
        planner->smooth_flags = flags;
        planner->smooth_pending = true;
        return;
    }

    /* --- We have a buffered move and a new move — check the corner ---
     * Buffered: planner->position_mm → smooth_target (the corner point)
     * New:      smooth_target → target */

    float prev_dir[GCODE_MAX_AXES], new_dir[GCODE_MAX_AXES];
    float prev_dist_sq = 0, new_dist_sq = 0;
    for (int i = 0; i < GCODE_MAX_AXES; i++) {
        prev_dir[i] = planner->smooth_target[i] - planner->position_mm[i];
        new_dir[i]  = target[i] - planner->smooth_target[i];
        prev_dist_sq += prev_dir[i] * prev_dir[i];
        new_dist_sq  += new_dir[i]  * new_dir[i];
    }
    float prev_dist = sqrtf(prev_dist_sq);
    float new_dist  = sqrtf(new_dist_sq);

    /* Skip rounding if either move is too short */
    if (prev_dist < 0.01f || new_dist < 0.01f) {
        goto no_rounding;
    }

    /* Normalize direction vectors */
    float inv_prev = 1.0f / prev_dist;
    float inv_new  = 1.0f / new_dist;
    for (int i = 0; i < GCODE_MAX_AXES; i++) {
        prev_dir[i] *= inv_prev;
        new_dir[i]  *= inv_new;
    }

    /* cos(theta): negate dot product (same convention as compute_junction_speed) */
    float cos_theta = 0;
    for (int i = 0; i < GCODE_MAX_AXES; i++) {
        cos_theta -= prev_dir[i] * new_dir[i];
    }

    /* Only round corners with significant angle (> ~18° turn) */
    if (cos_theta <= -0.95f || cos_theta >= 0.99f) {
        goto no_rounding;
    }

    /* --- Compute rounding arc parameters --- */
    {
        float alpha = acosf(-cos_theta);           /* Turn angle */
        float half_alpha = alpha * 0.5f;
        float cos_half = cosf(half_alpha);
        float sin_half = sinf(half_alpha);

        /* Radius from tolerance: deviation = R*(1 - cos(α/2)) = tolerance */
        if (cos_half >= 0.9999f) goto no_rounding; /* Nearly straight */
        float R = planner->path_tolerance / (1.0f - cos_half);

        /* Tangent length: distance from corner along each move */
        float tangent_len = R * sin_half / cos_half; /* R * tan(α/2) */

        /* Skip if tangent doesn't fit in 40% of either move */
        if (tangent_len > prev_dist * 0.4f || tangent_len > new_dist * 0.4f) {
            goto no_rounding;
        }
        if (tangent_len < 0.005f) {
            goto no_rounding;
        }

        /* Approach point: corner - tangent_len * prev_dir */
        float approach[GCODE_MAX_AXES];
        for (int i = 0; i < GCODE_MAX_AXES; i++) {
            approach[i] = planner->smooth_target[i] - tangent_len * prev_dir[i];
        }

        /* Departure point: corner + tangent_len * new_dir */
        float departure[GCODE_MAX_AXES];
        for (int i = 0; i < GCODE_MAX_AXES; i++) {
            departure[i] = planner->smooth_target[i] + tangent_len * new_dir[i];
        }

        /* 1. Emit shortened buffered move (position_mm → approach) */
        gcode_planner_line(planner, approach, planner->smooth_feed,
                           false, planner->smooth_flags);

        /* 2. Emit arc segments (approach → departure) */
        /* Find e2: perpendicular component of new_dir relative to prev_dir */
        float dot_pn = 0;
        for (int i = 0; i < GCODE_MAX_AXES; i++) {
            dot_pn += prev_dir[i] * new_dir[i];
        }

        float e2[GCODE_MAX_AXES];
        float e2_len_sq = 0;
        for (int i = 0; i < GCODE_MAX_AXES; i++) {
            e2[i] = new_dir[i] - dot_pn * prev_dir[i];
            e2_len_sq += e2[i] * e2[i];
        }
        float e2_len = sqrtf(e2_len_sq);

        if (e2_len > 1e-6f) {
            float inv_e2 = 1.0f / e2_len;
            for (int i = 0; i < GCODE_MAX_AXES; i++) e2[i] *= inv_e2;

            /* Number of arc segments (chord-error formula from gcode_arcs.c) */
            float arc_tol = nvs_config_get_float("arc_tol",
                                                  GCODE_DEFAULT_ARC_TOLERANCE);
            int segments;
            if (arc_tol >= R) {
                segments = 3;
            } else {
                float theta_per_seg = 2.0f * acosf(1.0f - arc_tol / R);
                if (theta_per_seg < 1e-6f) {
                    segments = 10;
                } else {
                    segments = (int)ceilf(alpha / theta_per_seg);
                }
            }
            if (segments < 2) segments = 2;
            if (segments > 10) segments = 10;

            /* Generate arc points.
             * Arc formula: P = approach + R*sin(θ)*prev_dir + R*(1-cos(θ))*e2
             * where θ sweeps from 0 to α (the turn angle). */
            float arc_feed = planner->smooth_feed;
            if (feed_rate_mm_min < arc_feed) arc_feed = feed_rate_mm_min;

            for (int s = 1; s <= segments; s++) {
                float seg_pt[GCODE_MAX_AXES];
                if (s == segments) {
                    /* Last segment: use exact departure to avoid float drift */
                    memcpy(seg_pt, departure, sizeof(float) * GCODE_MAX_AXES);
                } else {
                    float frac = (float)s / (float)segments;
                    float ang = frac * alpha;
                    float sin_ang = sinf(ang);
                    float cos_ang = cosf(ang);

                    for (int i = 0; i < GCODE_MAX_AXES; i++) {
                        seg_pt[i] = approach[i]
                                  + R * sin_ang * prev_dir[i]
                                  + R * (1.0f - cos_ang) * e2[i];
                    }
                }
                gcode_planner_line(planner, seg_pt, arc_feed, false, 0);
            }
        } else {
            /* Degenerate: directions nearly parallel, just connect directly */
            gcode_planner_line(planner, departure, planner->smooth_feed,
                               false, 0);
        }

        /* 3. Buffer the new move (departure → target) */
        memcpy(planner->smooth_target, target, sizeof(float) * GCODE_MAX_AXES);
        planner->smooth_feed = feed_rate_mm_min;
        planner->smooth_flags = flags;
        /* smooth_pending remains true */
        return;
    }

no_rounding:
    /* No rounding — emit buffered move as-is, buffer new move */
    gcode_planner_line(planner, planner->smooth_target,
                       planner->smooth_feed, false, planner->smooth_flags);
    memcpy(planner->smooth_target, target, sizeof(float) * GCODE_MAX_AXES);
    planner->smooth_feed = feed_rate_mm_min;
    planner->smooth_flags = flags;
}
