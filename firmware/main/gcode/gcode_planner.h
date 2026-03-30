/*
 * WiFi CNC Controller - G-Code Look-Ahead Motion Planner
 *
 * Converts G-code moves (in mm) into pre-planned motion segments with
 * proper junction velocities for smooth constant-velocity motion.
 *
 * Sits ABOVE the existing planner ring buffer:
 *   G-code move (mm) -> look-ahead buffer -> planner_push_segment()
 *
 * Core 1 (motion_control_task, stepper ISR) is completely untouched.
 */

#ifndef GCODE_PLANNER_H
#define GCODE_PLANNER_H

#include <stdint.h>
#include <stdbool.h>
#include "gcode_parser.h"

/* ===================================================================
 * Configuration
 * =================================================================== */

#define GCODE_PLAN_BUFFER_SIZE      64      /* Look-ahead buffer slots (power-of-2) */
#define GCODE_PLAN_BUFFER_MASK      (GCODE_PLAN_BUFFER_SIZE - 1)

/* Default planning parameters (overridden by NVS $ settings) */
#define GCODE_DEFAULT_JUNCTION_DEV  0.01f   /* mm ($11) */
#define GCODE_DEFAULT_ARC_TOLERANCE 0.002f  /* mm ($12) */
#define GCODE_MIN_FEED_RATE         1.0f    /* mm/min minimum */
#define GCODE_MIN_SPEED             0.001f  /* mm/sec minimum for calculations */

/* Adaptive emission thresholds (fraction of ring buffer capacity) */
#define GCODE_EMIT_EMERGENCY_THRESH 0.75f   /* >75% empty: emit immediately */
#define GCODE_EMIT_LOW_THRESH       0.50f   /* >50% empty: reduce look-ahead */
#define GCODE_EMIT_MIN_LOOKAHEAD    1       /* Emergency: emit with 1 block */
#define GCODE_EMIT_LOW_LOOKAHEAD    8       /* Low buffer: 8 block look-ahead */
#define GCODE_EMIT_NORMAL_LOOKAHEAD 20      /* Normal: 20 block look-ahead */

/* ===================================================================
 * Planned Block (one move in the look-ahead buffer)
 * =================================================================== */

typedef struct {
    /* Target position in machine coordinates (mm) */
    float   target[GCODE_MAX_AXES];

    /* Move geometry */
    float   distance_mm;                    /* Euclidean distance of move */
    float   unit_vec[GCODE_MAX_AXES];       /* Normalized direction vector */

    /* Velocity planning (all in mm/sec) */
    float   programmed_rate;                /* Commanded feed rate (mm/sec) */
    float   max_entry_speed;                /* Max junction speed (from geometry) */
    float   entry_speed;                    /* Planned entry speed */
    float   nominal_speed;                  /* Desired cruise speed */
    float   max_speed;                      /* Axis-limited max speed */
    float   acceleration;                   /* Effective accel (mm/sec^2) */

    /* Step counts (for segment conversion) */
    int32_t steps[GCODE_MAX_AXES];
    uint32_t step_event_count;              /* Max |steps| on any axis */

    /* Flags */
    uint8_t flags;                          /* WCNC_SEG_FLAG_* */
    bool    is_rapid;
    bool    recalculate;                    /* Needs velocity re-plan */

    /* Segment tracking */
    uint16_t segment_id;
} gcode_plan_block_t;

/* ===================================================================
 * Planner State
 * =================================================================== */

typedef struct {
    /* Circular buffer */
    gcode_plan_block_t  buffer[GCODE_PLAN_BUFFER_SIZE];
    uint8_t             head;               /* Next write position */
    uint8_t             tail;               /* Next emit position */
    uint8_t             count;              /* Active blocks */

    /* Position tracking (steps, matches stepper) */
    int32_t  position_steps[GCODE_MAX_AXES];

    /* Position tracking (mm, for target computation) */
    float    position_mm[GCODE_MAX_AXES];

    /* Previous unit vector (for junction angle) */
    float    prev_unit_vec[GCODE_MAX_AXES];
    bool     prev_unit_valid;

    /* Axis configuration (cached from NVS, in mm-space) */
    float    steps_per_mm[GCODE_MAX_AXES];
    float    max_rate_mm_sec[GCODE_MAX_AXES];
    float    max_accel_mm_sec2[GCODE_MAX_AXES];

    /* Planner parameters */
    float    junction_deviation;            /* $11 in mm */
    float    path_tolerance;                /* G64 P value */
    bool     exact_stop_mode;               /* G61 active */

    /* Segment ID counter */
    uint16_t next_segment_id;

    /* Diagnostics */
    uint32_t underrun_count;                /* Times ring buffer went empty */
    uint32_t blocks_emitted;                /* Total blocks emitted */

    /* Flow control */
    volatile bool preempted;                /* Binary protocol has priority */

    /* Corner smoothing: one-move lookahead buffer.
     * Buffers one G1 move so we can detect corners and insert
     * rounding arcs before the second move enters the planner. */
    float    smooth_target[GCODE_MAX_AXES]; /* Buffered move target */
    float    smooth_feed;                   /* Buffered feed rate (mm/min) */
    uint8_t  smooth_flags;                  /* Buffered segment flags */
    bool     smooth_pending;                /* true if a move is buffered */
} gcode_planner_t;

/* ===================================================================
 * API Functions
 * =================================================================== */

/* Initialize the look-ahead planner, load axis config from NVS */
void gcode_planner_init(gcode_planner_t *planner);

/* Reload axis configuration from NVS (after $ setting change) */
void gcode_planner_reload_config(gcode_planner_t *planner);

/* Synchronize planner position with actual stepper position.
 * Call after E-stop/reset to re-align. */
void gcode_planner_sync_position(gcode_planner_t *planner);

/* Add a linear move to the planner.
 * target[] is in machine coordinates (mm).
 * feed_rate is in mm/min.
 * Computes junction velocity and may emit segments to ring buffer. */
void gcode_planner_line(gcode_planner_t *planner,
                         const float target[GCODE_MAX_AXES],
                         float feed_rate_mm_min,
                         bool is_rapid,
                         uint8_t flags);

/* Add a dwell (pause with no motion).
 * Flushes all pending moves first, then delays. */
void gcode_planner_dwell(gcode_planner_t *planner, float seconds);

/* Flush all buffered blocks to the ring buffer.
 * Called for M0/M2/M30 and before probing. Blocks until all emitted. */
void gcode_planner_sync(gcode_planner_t *planner);

/* Discard all planned blocks.
 * Called on E-stop/reset. Does NOT reset position. */
void gcode_planner_clear(gcode_planner_t *planner);

/* Check if planner is empty and all segments emitted */
bool gcode_planner_is_idle(gcode_planner_t *planner);

/* Set path blending mode (G61/G64) */
void gcode_planner_set_path_mode(gcode_planner_t *planner,
                                  bool exact_stop,
                                  float tolerance);

/* Get number of free slots in look-ahead buffer */
uint8_t gcode_planner_available(gcode_planner_t *planner);

/* Corner-smoothing line wrapper (buffers one G1 move for corner detection).
 * When G64 P<tolerance> is active, inserts rounding arcs at sharp corners.
 * Only for G1 feed moves — G0/probes/arcs call gcode_planner_line() directly. */
void gcode_planner_line_smooth(gcode_planner_t *planner,
                                const float target[GCODE_MAX_AXES],
                                float feed_rate_mm_min,
                                uint8_t flags);

/* Flush the corner-smoothing buffer (emit pending move without rounding).
 * Called before G0, arcs, probes, and any other non-G1 operation. */
void gcode_planner_flush_smooth(gcode_planner_t *planner);

#endif /* GCODE_PLANNER_H */
