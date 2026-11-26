/*
 * WiFi CNC Controller - Step engine
 *
 * Plays a queue of timed segments as step pulses.  Everything that moves
 * the motors goes through here: motion streamed by a host, jogging, homing
 * and G-code mode all queue timed segments (see timed_exec.h for how one
 * segment becomes pulses).
 *
 *   - Segments follow each other with no gap: the end of one is the start
 *     of the next, on a timeline that does not drift with interrupt latency.
 *   - Playback can be slowed smoothly to a stop and back ("time dilation"),
 *     which gives feed hold, a decelerating abort and a graceful reaction to
 *     a starved queue without the host having to plan any of them.
 *   - A probe move stops within one step of contact; while homing, each axis
 *     stops within one step of its switch.
 *
 * The code is hardware independent.  The few things that touch hardware are
 * inline functions in se_port.h, of which there is one for the ESP32 (in
 * this folder) and one for the PC test build (firmware/test/pc).
 *
 * Threading: se_isr() runs in the step timer interrupt.  All other
 * functions that change anything must be called from one task (the motion
 * task); the read-only queries may be called from anywhere.
 */

#ifndef STEP_ENGINE_H
#define STEP_ENGINE_H

#include <stdbool.h>
#include <stdint.h>
#include "../../../protocol/wifi_cnc_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SE_QUEUE_SIZE   128                 /* segments; must be a power of two */
#define SE_RATE_ONE     65536u              /* playback rate 1.0 */

void se_init(void);

/* Step pulse width and direction set-up time, microseconds. */
void se_set_timing(uint32_t pulse_us, uint32_t dir_setup_us);
/* Shortest time between two steps of one axis that the engine will produce
 * with the current timing.  A segment asking for more is stretched. */
uint32_t se_min_interval_us(void);

/* --- Queue ------------------------------------------------------------ */

/* Forget the fractions of a step carried between segments.  Call before
 * the first segment of a new stream, with the engine stopped. */
void se_reset_stream(void);

/* Queue a timed segment (WCNC_SEG_FLAG_TIMED, optionally _SUBSTEP).
 * Returns 1 when queued, 0 when the queue is full (try again later) and
 * -1 when the segment cannot be played at all. */
int  se_push(const wcnc_motion_segment_t *seg);

uint16_t se_free(void);                     /* segments that can still be queued */
uint16_t se_count(void);                    /* segments queued behind the one playing */
uint32_t se_queued_us(void);                /* motion time not yet played */

/* --- Playback --------------------------------------------------------- */

/* Begin playing the queue.  Does nothing if already playing or if nothing
 * is queued. */
void se_start(void);
/* The same, but bringing the playback rate up from a standstill over
 * ramp_ms: for picking a stream up again in the middle of a move. */
void se_start_ramped(uint32_t ramp_ms);


/* True from se_start() until the queue runs out or playback is halted;
 * stays true while paused by se_ramp_stop(). */
bool se_active(void);
bool se_paused(void);                       /* slowed to a standstill, queue kept */
bool se_ramping(void);                      /* slowing down or speeding up */

/* Stop at once and discard everything queued. */
void se_halt(void);

/* Slow playback smoothly to a standstill over ramp_ms.  With discard the
 * queue is then thrown away and the engine stops (se_active() becomes
 * false); without, it pauses and se_ramp_resume() continues. */
void se_ramp_stop(uint32_t ramp_ms, bool discard);
void se_ramp_resume(uint32_t ramp_ms);

/* True when the last segment played before the queue ran out had
 * WCNC_SEG_FLAG_LAST: the stream ended on purpose, not by starvation. */
bool se_ended_on_last(void);

/* --- Position and feedback ------------------------------------------- */

void     se_get_position(int32_t position[WCNC_MAX_AXES]);
void     se_set_position(int axis, int32_t value);
uint16_t se_current_id(void);               /* segment_id being played */
uint32_t se_feed_rate(void);                /* steps/s of the fastest axis right now */
void     se_get_rates(uint32_t rate[WCNC_MAX_AXES]);   /* steps/s of each axis in the segment being played */

/* --- Stopping on inputs ----------------------------------------------- */

/* Probe: a segment with WCNC_SEG_FLAG_PROBE stops the engine and discards
 * the queue the moment the probe input is active.  Latched until cleared. */
bool se_probe_tripped(void);
void se_clear_probe(void);

/* Homing: while an axis is in axis_mask it does not step whenever limit
 * input input[axis] is active.  se_gate_tripped() tells which axes have
 * been held back since the gates were set. */
void    se_gate(uint8_t axis_mask, const uint8_t input[WCNC_MAX_AXES]);
uint8_t se_gate_tripped(void);

/* --- Interrupt -------------------------------------------------------- */

/* Called by the port when the alarm set with port_set_alarm() fires. */
void se_isr(void);

/* Call now and then (every millisecond or so) from the motion task.  If
 * the timer ever failed to deliver an alarm, this sets it again. */
void se_watchdog(void);


#ifdef __cplusplus
}
#endif

#endif /* STEP_ENGINE_H */
