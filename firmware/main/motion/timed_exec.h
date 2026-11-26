/*
 * WiFi CNC Controller - Timed segment playback (hardware independent core)
 *
 * A timed segment (WCNC_SEG_FLAG_TIMED) asks for a movement to be spread
 * evenly over exactly duration_us.  This module does the arithmetic: when
 * each axis must step, and when the segment ends.
 *
 * It contains no hardware access and no floating point, and what runs per
 * step is a handful of 32-bit additions and comparisons, so it is safe to
 * call from the step interrupt.  The same file is compiled into the
 * PC-side unit test (firmware/test/) so the timing can be verified without
 * a board.
 *
 * Model
 *   Every axis has a position accumulator in units of 1/256 step
 *   (WCNC_SUBSTEP_SCALE).  A segment adds a signed amount to each
 *   accumulator, linearly over its duration, and an axis steps whenever its
 *   accumulator passes a whole step.  The fraction left at the end of a
 *   segment is the starting point of the next one, so:
 *     - each axis produces an evenly spaced pulse train of its own, however
 *       the movement is cut into segments and whatever the other axes do;
 *     - consecutive segments join with no gap and no accumulated error;
 *     - a host can command any velocity exactly (WCNC_SEG_FLAG_SUBSTEP).
 *   Segments without WCNC_SEG_FLAG_SUBSTEP carry whole steps and simply add
 *   256 times as much.
 */

#ifndef TIMED_EXEC_H
#define TIMED_EXEC_H

#include <stdbool.h>
#include <stdint.h>
#include "../../../protocol/wifi_cnc_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Longest duration of a timed segment that moves (dwells may be longer). */
#define TEXEC_MAX_MOVE_US       0x00FFFFFFu     /* about 16.7 s */
/* Longest duration of any timed segment. */
#define TEXEC_MAX_DURATION_US   60000000u
/* Most steps one axis may take in one segment. */
#define TEXEC_MAX_STEPS         0x000FFFFFu

typedef struct {
    /* Carried from segment to segment */
    uint8_t  phase[WCNC_MAX_AXES];   /* fraction of a step accumulated, 0..255 */

    /* Current segment */
    uint32_t next[WCNC_MAX_AXES];    /* time of the axis' next step, 1/256 us from segment start */
    uint32_t period[WCNC_MAX_AXES];  /* time between its steps, 1/256 us */
    uint32_t left[WCNC_MAX_AXES];    /* steps still to make in this segment */
    int8_t   dir[WCNC_MAX_AXES];     /* +1 / -1 */
    uint32_t duration;               /* microseconds (after any stretching) */
    uint8_t  active;                 /* bit per axis with steps left */
    uint8_t  flags;                  /* WCNC_SEG_FLAG_* of the source segment */
} texec_t;

/* Forget the carried fractions: call when a new stream starts. */
void texec_reset(texec_t *t);

/* Prepare a segment.  Returns false if it cannot be played (too many steps
 * for one segment).  A segment too short for its step count is stretched so
 * that no axis steps faster than once per min_interval_us; a moving
 * segment longer than TEXEC_MAX_MOVE_US is refused. */
bool texec_load(texec_t *t, const wcnc_motion_segment_t *seg, uint32_t min_interval_us);

/* Time, in microseconds from the start of the segment, of the next event.
 * When *mask is non-zero the event is a step on those axes (apply dir[]);
 * when *mask is zero the event is the end of the segment.  Steps closer
 * together than 'merge_us' are reported as one event.  After the
 * end-of-segment event has been returned, load the next segment. */
uint32_t texec_next(texec_t *t, uint8_t *mask, uint32_t merge_us);

static inline uint32_t texec_duration(const texec_t *t) { return t->duration; }

#ifdef __cplusplus
}
#endif

#endif /* TIMED_EXEC_H */
