/*
 * WiFi CNC Controller - Spindle tracking (hardware independent)
 *
 * Turns what a spindle sensor delivers into an estimate of the spindle's
 * angle at any moment, smooth enough to time step pulses by.  The step
 * engine uses it to play segments in step with the spindle (screw cutting).
 *
 * Two kinds of sensor:
 *
 *   SPT_QUADRATURE   An encoder with A and B channels, counted by hardware.
 *                    The count is sampled every SPT_SAMPLE_US and followed by
 *                    a tracking filter, which removes the staircase of the
 *                    count and gives the angle between samples.  An index
 *                    pulse, if there is one, is only used to learn where in
 *                    the revolution the index is: the angle itself always
 *                    comes from the count, so the moment an index pulse is
 *                    noticed need not be exact.  Without an index pulse the
 *                    position the spindle had at power-up serves as index.
 *
 *   SPT_PULSES       One sensor giving one pulse (or a few) per revolution.
 *                    Between pulses the angle is carried forward at the
 *                    speed measured from the last two, and each pulse
 *                    corrects it.  This follows a steady spindle exactly; a
 *                    change of speed it cannot see until the turn is over,
 *                    so while the speed changes the estimate is out by about
 *                    what the speed changed by in one turn.  With more than
 *                    one pulse per revolution nothing says which of them is
 *                    which, so only the speed is to be relied on
 *                    (spt_can_sync()).
 *
 * How slow: the quadrature filter wants about a hundred counts a second or
 * more (a 100-line encoder at 15 rpm).  Below that the angle is still right
 * to a count, but the speed estimate wavers between counts.
 *
 * Angles are 64-bit with one revolution = SPT_REV (2^32), so the low 32 bits
 * are the position within the revolution.  Speeds are revolutions per
 * microsecond times 2^48.  Time is the step timer's count in microseconds.
 *
 * No hardware access, no floating point, nothing that blocks: every function
 * may be called from an interrupt.  None of them locks: on the controller
 * all calls are made with the step engine's lock held.  The same file is
 * compiled into the PC test build.
 */

#ifndef SPINDLE_TRACK_H
#define SPINDLE_TRACK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPT_REV         ((int64_t)1 << 32)
#define SPT_SAMPLE_US   500u                /* how often spt_counts() / spt_poll() are to be called */

typedef enum { SPT_NONE = 0, SPT_QUADRATURE = 1, SPT_PULSES = 2 } spt_mode_t;

/* per_rev: counts per revolution (four times the encoder's line count) or
 * pulses per revolution.  index_input: a quadrature encoder's index pulse is
 * connected.  Tracking starts afresh. */
void spt_setup(spt_mode_t mode, uint32_t per_rev, bool index_input);
bool spt_ready(void);                       /* a sensor has been set up */
/* ...and it tells where in the revolution the spindle is, not only how fast
 * it turns: moves can be played in step with it. */
bool spt_can_sync(void);

/* --- From the sensor ---------------------------------------------------- */

/* Quadrature: the count has changed by delta since the previous call. */
void spt_counts(uint64_t now, int32_t delta);
/* Quadrature: the index pulse arrived. */
void spt_index(uint64_t now);
/* Pulses: a pulse arrived. */
void spt_pulse(uint64_t now);
/* Pulses: call every SPT_SAMPLE_US or so, to notice a spindle that stopped. */
void spt_poll(uint64_t now);

/* --- To whoever wants to know ------------------------------------------- */

int64_t  spt_angle(uint64_t now);           /* spindle angle at 'now' */
int64_t  spt_speed(uint64_t now);           /* signed; 0 when not known */
int      spt_direction(void);               /* +1 / -1: which way it last turned */
/* The index position the spindle reaches next when turning in 'direction'
 * from 'angle'. */
int64_t  spt_next_index(int64_t angle, int direction);
/* Changes whenever what spt_next_index() goes by has been found anew (the
 * index pulse located, a pulse sensor's spindle turning again): an index
 * position worked out before is then no longer to be relied on. */
uint32_t spt_index_epoch(void);
/* False while there is no telling where the index is: an encoder whose index
 * pulse has not been seen (twice) yet, a pulse sensor whose spindle is not
 * turning. */
bool     spt_index_known(void);
/* The sensor is being followed: spt_angle() moves with the spindle.  (False
 * for a pulse sensor whose spindle stands still: its angle is only known
 * again a turn after it starts.) */
bool     spt_following(void);

uint32_t spt_rpm_milli(void);               /* speed, 1/1000 rpm, steadied for display */
uint16_t spt_position16(uint64_t now);      /* angle past the index, 0..65535 = one revolution */
uint32_t spt_index_count(void);             /* index pulses seen; without an index input, times
                                               the spindle has come round to where it began */

#ifdef __cplusplus
}
#endif

#endif /* SPINDLE_TRACK_H */
