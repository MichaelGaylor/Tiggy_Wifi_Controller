/*
 * WiFi CNC Controller - Step engine
 *
 * See step_engine.h.  Hardware access is through the inline functions of
 * se_port.h:
 *
 *   SE_ISR_ATTR, SE_DATA_ATTR        placement of interrupt code and data
 *   SE_LOCK() / SE_UNLOCK()          task side: keep the interrupt out
 *   SE_ISR_LOCK() / SE_ISR_UNLOCK()  interrupt side of the same lock
 *   uint64_t port_now()              step timer count, microseconds
 *   void port_set_alarm(uint64_t t)  call se_isr() when the count reaches t
 *   void port_stop_alarm()
 *   void port_set_dir(int axis, bool negative)
 *   void port_step_high(uint8_t axis_mask), port_step_low(uint8_t axis_mask)
 *   void port_delay_us(uint32_t us)
 *   bool port_limit_active(int input), bool port_probe_active()
 *
 * Timeline.  'alarm' is the time the interrupt was asked for, not the time
 * it ran: each new alarm is the previous one plus an interval, so interrupt
 * latency never accumulates.  Within a segment 'seg_pos' is the position on
 * the segment's own (nominal) clock.  At playback rate 1 the two clocks tick
 * together and the interrupt sleeps until the next step.  While the rate is
 * being ramped the nominal clock runs slower: the interrupt then advances in
 * short hops so the rate changes smoothly even when steps are far apart.
 *
 * Playing by the spindle (WCNC_SEG_FLAG_SPINDLE_SYNC).  For these segments
 * the nominal clock is not driven by time at all but by the spindle: its
 * angle since the run began, at the speed the host planned for, is the
 * nominal time.  The interrupt looks at the spindle at least every
 * SE_SYNC_POLL_US and plays whatever has come due, so the axes follow the
 * angle itself, not an estimate of the speed: nothing accumulates, and a
 * spindle that stops simply stops the axes.  A run that is to begin at the
 * index (WCNC_SEG_FLAG_INDEX_START) starts from the next index position
 * ahead; until the spindle gets there the nominal time is negative and
 * nothing moves.
 *
 * A run is played in step with the spindle or not at all.  When something
 * breaks into one - a stop asked for in the middle of it, a queue that runs
 * dry, a spindle the axes cannot keep up with - the axes slow to a standstill
 * like in any other move, by the clock, and the run has lost its place on
 * the spindle for good.  What is left of it is then played, by the clock,
 * only if it is no more than a tail (SE_SYNC_TAIL_US): the last of a pass
 * coming to rest, whose final segments happened to arrive late.  More than
 * that is not played at all: the queue is dropped and se_sync_broken() says
 * so.  Played on by the clock, the rest of a threading pass would be cut
 * across the thread already there, or against a spindle that has stopped.
 */

#include "step_engine.h"
#include "timed_exec.h"
#include "spindle_track.h"
#include "se_port.h"

#include <string.h>

#define SE_MASK             (SE_QUEUE_SIZE - 1)
#define SE_RATE_MIN         (SE_RATE_ONE / 64)  /* slower than this counts as standing still */
#define SE_HOP_US           500u                /* nominal time per interrupt while the rate is ramping */
#define SE_START_LEAD_US    50u                 /* from se_start() to the start of the first segment */
#define SE_MAX_EVENTS       4                   /* events handled in one interrupt before yielding */
#define SE_ALARM_GUARD_US   4u                  /* an alarm is never set closer than this to the present */
#define SE_OVERDUE_US       2000u               /* se_watchdog(): an alarm this late was missed */
#define SE_SYNC_POLL_US     250u                /* longest the spindle goes unlooked at while playing by it */
#define SE_SYNC_RATE_MAX    (4 * SE_RATE_ONE)   /* a spindle speed beyond this multiple of the reference is not believed */
#define SE_SYNC_EXIT_SLOPE  ((int32_t)((SE_RATE_ONE * 1024u) / 100000u))   /* back to the clock: as a 100 ms ramp */
#define SE_SYNC_SLIP_SHIFT  11                  /* sync_k >> this: an eighth of a revolution, in nominal
                                                   microseconds.  Axes this far behind the spindle have
                                                   lost the thread. */
#define SE_SYNC_TAIL_US     200000u             /* most of a run (nominal time) that is played by the
                                                   clock once it has lost step with the spindle.  More
                                                   than a host keeps back at the end of a move, less
                                                   than it keeps queued in the middle of one. */
#define SE_SYNC_GIVE_UP_SLOPE ((int32_t)((SE_RATE_ONE * 1024u) / 50000u))    /* a 50 ms stop */

enum { SE_STOPPED = 0, SE_RUNNING, SE_PAUSED };
enum { SE_SYNC_OFF = 0, SE_SYNC_WAIT, SE_SYNC_RUN };    /* by the clock / waiting for the index / by the spindle */

typedef struct {
    texec_t  tx;                    /* prepared by texec_load() when queued */
    uint32_t feed;                  /* steps/s of its fastest axis */
    uint32_t sync_k;                /* played by the spindle: microseconds per revolution at the
                                       reference speed, times 256.  0: played by the clock */
    uint16_t id;                    /* segment_id */
} se_entry_t;

typedef struct {
    /* queue: written at head by the motion task, read at tail by the interrupt */
    se_entry_t ring[SE_QUEUE_SIZE];
    volatile uint16_t head, tail;
    volatile uint32_t pushed_us, popped_us;
    texec_t  prod;                  /* carries the step fractions from one queued segment to the next */

    /* playback (interrupt) */
    volatile uint8_t state;
    texec_t  run;                   /* segment being played */
    uint32_t run_count[WCNC_MAX_AXES];  /* steps it had when it started */
    uint16_t run_id;
    uint32_t feed;
    uint32_t seg_pos;               /* nominal microseconds into the segment at the last alarm */
    uint32_t hop_to;                /* ...and at the alarm now pending */
    uint32_t ev_time;               /* next event of the segment, nominal microseconds */
    uint8_t  ev_mask;               /* axes that step at ev_time; 0 = the segment ends */
    uint64_t alarm;                 /* timer count the pending alarm was asked for */
    uint64_t last_alarm;
    uint32_t rate;                  /* playback rate, SE_RATE_ONE = real time */
    int32_t  ramp;                  /* change of rate per 1024 us of real time */
    bool     ramp_discard;
    volatile bool ended_last;
    volatile bool probe_tripped;

    /* playing by the spindle */
    volatile uint8_t sync;          /* SE_SYNC_* */
    bool     sync_index;            /* the run begins at the index (false: wherever the spindle is) */
    volatile bool sync_lost;        /* the run has lost step with the spindle (a stop broke into
                                       it, the queue ran dry): what is still played of it goes
                                       by the clock */
    uint32_t lost_us;               /* ...and this much of it has been taken up since, nominal us */
    int32_t  stop_slope;            /* the ramp of the last stop asked for */
    volatile bool sync_broken;      /* a run was given up: see se_sync_broken() */
    volatile bool sync_slip;        /* the axes have fallen too far behind the spindle */
    int8_t   sync_dir;              /* which way the spindle turns in this run */
    uint32_t sync_epoch;            /* spt_index_epoch() when the index position was worked out */
    uint32_t run_k;                 /* sync_k of the segment being played */
    uint32_t sync_k;                /* ...and of the run */
    int64_t  sync_origin;           /* spindle angle at which the run begins */
    uint64_t sync_base;             /* nominal microseconds of the run before the segment being played */

    /* axes */
    volatile int32_t position[WCNC_MAX_AXES];
    uint8_t  dir_neg;               /* direction pins currently set for negative travel */
    uint8_t  gate_mask;
    uint8_t  gate_input[WCNC_MAX_AXES];
    volatile uint8_t gate_tripped;

    /* timing */
    uint32_t pulse_us, dir_setup_us, min_interval_us, merge_us, event_us;
} se_state_t;

static SE_DATA_ATTR se_state_t se;

/* ===================================================================
 * Interrupt side
 * =================================================================== */

static inline void SE_ISR_ATTR stop_and_discard(void)
{
    se.state = SE_STOPPED;
    se.tail = se.head;
    se.popped_us = se.pushed_us;
    se.feed = 0;
    se.rate = SE_RATE_ONE;
    se.ramp = 0;
    se.sync = SE_SYNC_OFF;
    se.sync_lost = false;
    se.lost_us = 0;
    se.sync_slip = false;
    port_stop_alarm();
}

/* A run that has lost step with the spindle: is there more of it in hand
 * than may be played by the clock?  (What is queued behind it counts as part
 * of it.) */
static inline bool SE_ISR_ATTR lost_too_much(void)
{
    return (uint64_t)se.lost_us + (uint32_t)(se.pushed_us - se.popped_us) > SE_SYNC_TAIL_US;
}

/* Nominal microseconds for 'turned' (>= 0) of spindle travel, k being the
 * microseconds per revolution times 256. */
static inline uint64_t SE_ISR_ATTR sync_time(int64_t turned, uint32_t k)
{
    uint64_t whole = (uint64_t)turned >> 32, part = (uint64_t)turned & 0xFFFFFFFFu;
    if (whole > 0x00FFFFFFu) whole = 0x00FFFFFFu;
    return (whole * k + ((part * k) >> 32)) >> 8;
}

/* The spindle's speed as a fraction of the run's reference speed
 * (SE_RATE_ONE = the same); 0 if it stands still or turns the other way. */
static inline uint32_t SE_ISR_ATTR sync_ratio(void)
{
    int64_t v = spt_speed(port_now());
    if (se.sync_dir < 0) v = -v;
    if (v <= 0) return 0;
    uint64_t r = ((uint64_t)(v >> 8) * se.sync_k) >> 32;
    return r > SE_SYNC_RATE_MAX ? SE_SYNC_RATE_MAX : (uint32_t)r;
}

/* Begin a run played by the spindle: at its next index position, or here.
 * (Here, that is, once there is a spindle angle to go by: a sensor that has
 * yet to see the spindle turn is waited for like an index.) */
static inline void SE_ISR_ATTR sync_begin(uint32_t k, bool at_index)
{
    int64_t here = spt_angle(port_now());
    se.sync_dir = (int8_t)spt_direction();
    se.sync_epoch = spt_index_epoch();
    se.sync_index = at_index;
    se.sync_origin = at_index ? spt_next_index(here, se.sync_dir) : here;
    se.sync_base = 0;
    se.sync_k = k;
    se.sync = (at_index || !spt_following()) ? SE_SYNC_WAIT : SE_SYNC_RUN;
    se.sync_slip = false;
    se.rate = SE_RATE_ONE;
    se.ramp = 0;
}

/* Back to the clock, picking up at the speed the spindle was giving. */
static inline void SE_ISR_ATTR sync_end(void)
{
    uint32_t ratio = sync_ratio();
    se.sync = SE_SYNC_OFF;
    if (ratio < SE_RATE_ONE) {
        se.rate = ratio < SE_RATE_MIN ? SE_RATE_MIN : ratio;
        se.ramp = SE_SYNC_EXIT_SLOPE;
    } else {
        se.rate = SE_RATE_ONE;
        se.ramp = 0;
    }
}

/* Make the next queued segment the one being played.  False: none left. */
static inline bool SE_ISR_ATTR load_next(void)
{
    if (se.tail == se.head) return false;
    const se_entry_t *e = &se.ring[se.tail & SE_MASK];
    se.run = e->tx;
    se.run_id = e->id;
    se.run_k = e->sync_k;
    se.feed = e->feed;
    for (int i = 0; i < WCNC_MAX_AXES; i++) se.run_count[i] = e->tx.left[i];
    se.popped_us += e->tx.duration;
    se.tail = (uint16_t)(se.tail + 1);
    se.seg_pos = 0;
    se.hop_to = 0;
    return true;
}

/* A segment has just been loaded: is it played by the spindle or by the
 * clock?  'ended' is the duration of the segment before it and 'over' how
 * far the nominal clock had run beyond that segment's end. */
static inline void SE_ISR_ATTR choose_clock(uint32_t ended, uint32_t over)
{
    bool at_index = (se.run.flags & WCNC_SEG_FLAG_INDEX_START) != 0;
    if (!se.run_k || at_index) {
        se.sync_lost = false;                           /* the run that lost step is over */
        se.lost_us = 0;
    }
    if (se.run_k && !at_index && se.ramp < 0 && !se.sync_lost) {
        /* A run that would begin, wherever the spindle is, while a stop is
         * slowing the axes down.  It cannot be begun and stopped at once:
         * it has lost its step before it had it.  (A run that begins at the
         * index starts from rest: there the stop is simply complete - see
         * se_isr().) */
        se.sync_lost = true;
        se.lost_us = 0;
    }
    if (se.run_k && se.sync_lost) {
        /* More of a run that has lost step with the spindle.  A tail is
         * played out by the clock; beyond that it is given up - the axes,
         * if they are moving, stop, and the queue is dropped. */
        se.lost_us += se.run.duration;
        if (se.lost_us > SE_SYNC_TAIL_US && !(se.ramp < 0 && se.ramp_discard)) {
            se.sync_broken = true;
            se.ramp_discard = true;
            if (se.ramp >= 0) se.ramp = -(se.stop_slope > 0 ? se.stop_slope : SE_SYNC_GIVE_UP_SLOPE);
        }
    }
    if (se.run_k && !se.sync_lost) {
        if (se.sync == SE_SYNC_RUN && !at_index && se.run_k == se.sync_k) {
            se.sync_base += ended;                      /* the run continues */
            se.seg_pos = over;
        } else {
            sync_begin(se.run_k, at_index);
        }
    } else if (se.sync != SE_SYNC_OFF) {
        sync_end();
    }
}

/* Pulse the axes in mask.  Returns true if any pin was pulsed. */
static inline bool SE_ISR_ATTR pulse(uint8_t mask)
{
    /* Homing: an axis whose switch is active stays where it is. */
    uint8_t gated = se.gate_mask & mask;
    if (gated) {
        for (int i = 0; i < WCNC_MAX_AXES; i++) {
            uint8_t bit = (uint8_t)(1u << i);
            if ((gated & bit) && port_limit_active(se.gate_input[i])) {
                mask &= (uint8_t)~bit;
                se.gate_tripped |= bit;
            }
        }
        if (!mask) return false;
    }

    /* Direction pins change only when the direction does, and then get
     * their set-up time before the step edge. */
    uint8_t want = 0;
    for (int i = 0; i < WCNC_MAX_AXES; i++)
        if ((mask & (1u << i)) && se.run.dir[i] < 0) want |= (uint8_t)(1u << i);
    uint8_t change = (uint8_t)((se.dir_neg ^ want) & mask);
    if (change) {
        for (int i = 0; i < WCNC_MAX_AXES; i++)
            if (change & (1u << i)) port_set_dir(i, (want >> i) & 1);
        se.dir_neg = (uint8_t)((se.dir_neg & ~mask) | want);
        if (se.dir_setup_us) port_delay_us(se.dir_setup_us);
    }

    port_step_high(mask);
    for (int i = 0; i < WCNC_MAX_AXES; i++)
        if (mask & (1u << i)) se.position[i] += se.run.dir[i];
    port_delay_us(se.pulse_us);
    port_step_low(mask);
    return true;
}

void SE_ISR_ATTR se_isr(void)
{
    SE_ISR_LOCK();
    if (se.state != SE_RUNNING) {
        SE_ISR_UNLOCK();
        return;
    }

    uint64_t at = se.alarm;                 /* the moment this interrupt stands for */
    uint64_t to_origin = 0;                 /* waiting for the index: nominal time until it comes */

    if (se.sync != SE_SYNC_OFF) {
        /* By the spindle: the nominal clock is its angle */
        at = port_now();
        bool ready = true;
        if (se.sync == SE_SYNC_WAIT) {
            /* Is there something to begin at?  Not while the index has yet
             * to be found (the spindle has not turned since power-up, or its
             * index pulse never comes), nor, for a run that begins anywhere,
             * while the sensor has yet to see the spindle turn. */
            ready = se.sync_index ? spt_index_known() : spt_following();
            /* Where the run begins is worked out afresh if the spindle turns
             * the other way after all or its index has only now been found;
             * for a run that begins anywhere, it is here and now. */
            if (se.sync_index ? (spt_direction() != se.sync_dir || spt_index_epoch() != se.sync_epoch) : ready)
                sync_begin(se.sync_k, se.sync_index);
        }
        int64_t turned = spt_angle(at) - se.sync_origin;
        if (se.sync_dir < 0) turned = -turned;
        if (!ready) {
            to_origin = 4 * SE_SYNC_POLL_US;        /* look again */
        } else if (turned < 0) {
            /* (in a run this can only be the estimate wavering: carry on) */
            if (se.sync == SE_SYNC_WAIT) to_origin = sync_time(-turned, se.sync_k);
        } else {
            se.sync = SE_SYNC_RUN;
            uint64_t total = sync_time(turned, se.sync_k);
            uint64_t pos = total > se.sync_base ? total - se.sync_base : 0;
            /* Never backwards, and never more than half as fast again as
             * the move was planned: if the spindle jumps ahead (or seems
             * to), the axes catch up at a speed they can manage. */
            uint64_t since = at > se.last_alarm ? at - se.last_alarm : 0;
            if (since > 1000000u) since = 1000000u;
            uint64_t most = (uint64_t)se.seg_pos + since + (since >> 1);
            if (pos > most) {
                /* ...unless they are so far behind that the thread is lost
                 * anyway (the spindle runs far faster than the move was
                 * planned for, or the move asks more of the axes than the
                 * step pins can give): se_sync_slipped() */
                if (pos - most > (se.sync_k >> SE_SYNC_SLIP_SHIFT)) se.sync_slip = true;
                pos = most;
            }
            if (pos > se.seg_pos) se.seg_pos = (uint32_t)pos;
        }
    } else {
        se.seg_pos = se.hop_to;
    }

    /* Rate ramp, by the real time since the previous alarm */
    if (se.ramp) {
        int64_t r = (int64_t)se.rate + (((int64_t)se.ramp * (int64_t)(at - se.last_alarm)) >> 10);
        if (se.ramp < 0 && r <= (int64_t)SE_RATE_MIN) {
            if (!se.ramp_discard && se.sync_lost && lost_too_much()) {
                /* Stopped in a run played by the spindle, with more of it
                 * in hand than a tail: it is not going to be played. */
                se.sync_broken = true;
                se.ramp_discard = true;
            }
            if (se.ramp_discard) {
                se.ended_last = true;
                stop_and_discard();
            } else {
                se.rate = 0;
                se.ramp = 0;
                se.state = SE_PAUSED;
                port_stop_alarm();
            }
            SE_ISR_UNLOCK();
            return;
        }
        if (se.ramp > 0 && r >= (int64_t)SE_RATE_ONE) {
            r = SE_RATE_ONE;                /* up to speed */
            se.ramp = 0;
        }
        se.rate = (uint32_t)r;
    }
    se.last_alarm = at;

    /* Whatever is due */
    bool pulsed = false;
    uint8_t stepped = 0;
    for (int n = 0; n < SE_MAX_EVENTS && se.seg_pos >= se.ev_time && se.sync != SE_SYNC_WAIT; n++) {
        if (se.ev_mask) {
            /* An axis steps once in an interrupt: its pin must have been low
             * for its time before it goes high again.  (By the clock two
             * steps of an axis never come due together; by the spindle they
             * do when it has run ahead.)  The next interrupt takes it up. */
            if (se.ev_mask & stepped) break;
            if ((se.run.flags & WCNC_SEG_FLAG_PROBE) && port_probe_active()) {
                se.probe_tripped = true;
                se.ended_last = true;
                stop_and_discard();
                SE_ISR_UNLOCK();
                return;
            }
            if (pulse(se.ev_mask)) pulsed = true;
            stepped |= se.ev_mask;
        } else {
            bool last = (se.run.flags & WCNC_SEG_FLAG_LAST) != 0;
            bool stopping = se.ramp < 0;
            uint32_t ended = se.run.duration, over = se.seg_pos - se.ev_time;
            if (!load_next()) {
                /* (the queue running out in the middle of a run, the spindle
                 * goes on without the axes: the run has lost its step) */
                se.sync_lost = (se.sync == SE_SYNC_RUN || se.sync_lost) && !last;
                if (!se.sync_lost) se.lost_us = 0;
                se.ended_last = last;
                se.state = SE_STOPPED;
                se.feed = 0;
                se.sync = SE_SYNC_OFF;
                se.sync_slip = false;
                port_stop_alarm();
                SE_ISR_UNLOCK();
                return;
            }
            choose_clock(ended, over);
            if (se.sync == SE_SYNC_WAIT) {
                if (stopping) {
                    /* A stop was slowing the axes down, and here they are to
                     * stand and wait for the spindle anyway: the stop is
                     * complete.  (Resumed, the wait begins afresh.) */
                    if (se.ramp_discard) {
                        se.ended_last = true;
                        stop_and_discard();
                    } else {
                        se.ev_time = texec_next(&se.run, &se.ev_mask, se.merge_us);
                        se.state = SE_PAUSED;
                        port_stop_alarm();
                    }
                    SE_ISR_UNLOCK();
                    return;
                }
                to_origin = SE_SYNC_POLL_US;
            }
        }
        se.ev_time = texec_next(&se.run, &se.ev_mask, se.merge_us);
    }

    /* Sleep until the next event, or for one hop while the rate is not 1 */
    uint32_t delta = se.ev_time > se.seg_pos ? se.ev_time - se.seg_pos : 0;
    uint32_t real;
    if (se.sync != SE_SYNC_OFF) {
        /* ...or until the spindle should have brought the next event,
         * looking again in good time in case its speed has changed */
        uint64_t nominal = to_origin + delta;
        uint32_t ratio = sync_ratio();
        real = SE_SYNC_POLL_US;
        if (ratio >= SE_RATE_MIN && nominal < 4 * SE_SYNC_POLL_US) {
            real = ((uint32_t)nominal << 16) / ratio;
            if (real > SE_SYNC_POLL_US) real = SE_SYNC_POLL_US;
        }
        se.hop_to = se.seg_pos;
    } else if (se.rate == SE_RATE_ONE && !se.ramp) {
        real = delta;
        se.hop_to = se.seg_pos + delta;
    } else {
        uint32_t hop = delta > SE_HOP_US ? SE_HOP_US : delta;
        real = (hop << 16) / se.rate;       /* hop <= 500, so this fits in 32 bits */
        if (real == 0 && hop) real = 1;
        se.hop_to = se.seg_pos + hop;
    }
    se.alarm = at + real;

    /* Never ask for a moment already gone, and give the step pins their low
     * time after a pulse.  The timeline itself (se.alarm) is not shifted. */
    uint64_t earliest = port_now() + ((pulsed && se.pulse_us > SE_ALARM_GUARD_US) ? se.pulse_us : SE_ALARM_GUARD_US);
    port_set_alarm(se.alarm > earliest ? se.alarm : earliest);
    SE_ISR_UNLOCK();
}

void se_watchdog(void)
{
    SE_LOCK();
    if (se.state == SE_RUNNING) {
        uint64_t now = port_now();
        if (now > se.alarm + SE_OVERDUE_US) port_set_alarm(now + SE_ALARM_GUARD_US);
    }
    SE_UNLOCK();
}

/* ===================================================================
 * Task side
 * =================================================================== */

void se_init(void)
{
    memset(&se, 0, sizeof(se));
    se.rate = SE_RATE_ONE;
    se.ended_last = true;
    texec_reset(&se.prod);
    se_set_timing(5, 5);
}

void se_set_timing(uint32_t pulse_us, uint32_t dir_setup_us)
{
    if (pulse_us < 1) pulse_us = 1;
    if (pulse_us > 50) pulse_us = 50;
    if (dir_setup_us > 50) dir_setup_us = 50;
    SE_LOCK();
    se.pulse_us = pulse_us;
    se.dir_setup_us = dir_setup_us;
    /* One interrupt pulses every axis that is due within merge_us (about
     * the time the interrupt is busy anyway), so a step can be up to
     * merge_us early.  The step pin must still be high for pulse_us and low
     * for at least as long, which sets how close two steps of one axis may
     * be asked for:  high + low + the most a step can be early. */
    se.merge_us = pulse_us + 2;
    se.min_interval_us = 2 * pulse_us + se.merge_us + 2;
    if (se.min_interval_us < 10) se.min_interval_us = 10;
    /* What one interrupt costs when nothing can be merged */
    se.event_us = pulse_us + 4;
    SE_UNLOCK();
}

uint32_t se_min_interval_us(void) { return se.min_interval_us; }

void se_reset_stream(void)
{
    texec_reset(&se.prod);
}

int se_push(const wcnc_motion_segment_t *seg)
{
    if (!(seg->flags & WCNC_SEG_FLAG_TIMED)) return -1;
    uint32_t sync_k = 0;
    if (seg->flags & WCNC_SEG_FLAG_SPINDLE_SYNC) {
        /* seg->acceleration: the spindle speed the move was planned for, rpm * 1000 */
        if (!spt_can_sync() || seg->acceleration < WCNC_SYNC_MIN_MILLI_RPM) return -1;
        sync_k = (uint32_t)((60000000000ull * 256u + seg->acceleration / 2) / seg->acceleration);
    }
    uint16_t head = se.head;
    if ((uint16_t)(head - se.tail) >= SE_QUEUE_SIZE - 1) return 0;

    /* A segment played by the spindle is timed by the spindle: how close
     * together its steps come is not known until then, and stretching it
     * here would alter the thread.  It is taken as it is; the interrupt
     * keeps the step pins' timing, and if the axes cannot keep up
     * se_sync_slipped() says so. */
    texec_t before = se.prod;
    if (!texec_load(&se.prod, seg, sync_k ? 1 : se.min_interval_us)) {
        se.prod = before;
        return -1;
    }
    /* All axes together must not ask for more interrupts than there is
     * time for.  Like a single axis that is too fast, such a segment is
     * played more slowly rather than wrongly. */
    uint64_t steps = 0;
    for (int i = 0; i < WCNC_MAX_AXES; i++) steps += se.prod.left[i];
    if (!sync_k && steps * se.event_us > se.prod.duration) {
        wcnc_motion_segment_t slower = *seg;
        uint64_t need = steps * se.event_us;
        slower.duration_us = need > 0xFFFFFFFFu ? 0xFFFFFFFFu : (uint32_t)need;
        se.prod = before;
        if (!texec_load(&se.prod, &slower, se.min_interval_us)) {
            se.prod = before;
            return -1;
        }
    }

    se_entry_t *e = &se.ring[head & SE_MASK];
    e->tx = se.prod;
    if (!sync_k) e->tx.flags &= (uint8_t)~WCNC_SEG_FLAG_INDEX_START;
    e->sync_k = sync_k;
    e->id = seg->segment_id;
    uint32_t most = 0;
    for (int i = 0; i < WCNC_MAX_AXES; i++)
        if (se.prod.left[i] > most) most = se.prod.left[i];
    e->feed = (uint32_t)(((uint64_t)most * 1000000u) / se.prod.duration);

    SE_LOCK();
    se.pushed_us += se.prod.duration;
    se.head = (uint16_t)(head + 1);
    SE_UNLOCK();
    return 1;
}

uint16_t se_free(void)
{
    uint16_t used = (uint16_t)(se.head - se.tail);
    return (uint16_t)(SE_QUEUE_SIZE - 1 - (used > SE_QUEUE_SIZE - 1 ? SE_QUEUE_SIZE - 1 : used));
}

uint16_t se_count(void)
{
    return (uint16_t)(se.head - se.tail);
}

/* Rate change per 1024 us that takes the rate from 1 to 0 in ramp_ms. */
static int32_t ramp_slope(uint32_t ramp_ms)
{
    if (ramp_ms < 5) ramp_ms = 5;
    int32_t s = (int32_t)((SE_RATE_ONE * 1024u) / (ramp_ms * 1000u));
    return s < 1 ? 1 : s;
}

uint32_t se_queued_us(void)
{
    SE_LOCK();
    uint32_t q = se.pushed_us - se.popped_us;
    if (se.state != SE_STOPPED && se.run.duration > se.seg_pos) q += se.run.duration - se.seg_pos;
    SE_UNLOCK();
    return q;
}

void se_start(void)
{
    se_start_ramped(0);
}

void se_start_ramped(uint32_t ramp_ms)
{
    SE_LOCK();
    if (se.state == SE_STOPPED && load_next()) {
        se.ended_last = false;
        se.rate = ramp_ms ? SE_RATE_MIN : SE_RATE_ONE;
        se.ramp = ramp_ms ? ramp_slope(ramp_ms) : 0;
        se.sync = SE_SYNC_OFF;
        choose_clock(0, 0);                 /* (a run played by the spindle sets its own pace) */
        if (se.sync_lost && (se.sync_broken || lost_too_much())) {
            /* More of a run that ran dry, and more than a tail: the spindle
             * has gone on without the axes meanwhile.  It is given up. */
            se.sync_broken = true;
            se.ended_last = true;
            stop_and_discard();
            SE_UNLOCK();
            return;
        }

        se.ev_time = texec_next(&se.run, &se.ev_mask, se.merge_us);
        se.alarm = port_now() + SE_START_LEAD_US;
        se.last_alarm = se.alarm;
        se.state = SE_RUNNING;
        port_set_alarm(se.alarm);
    }
    SE_UNLOCK();
}

bool se_active(void)  { return se.state != SE_STOPPED; }
bool se_paused(void)  { return se.state == SE_PAUSED; }
bool se_ramping(void) { return se.state == SE_RUNNING && se.ramp != 0; }
bool se_ended_on_last(void) { return se.ended_last; }

int se_sync_state(void)
{
    uint8_t sync = se.sync;
    if (se.state == SE_STOPPED || sync == SE_SYNC_OFF) return 0;
    return sync == SE_SYNC_WAIT ? 1 : 2;
}

bool se_sync_broken(void)  { return se.sync_broken; }
bool se_sync_slipped(void) { return se.sync_slip; }
/* (stopped, the question is whether what is queued next is more of that run) */
static bool lost_run_in_hand(void)
{
    if (!se.sync_lost) return false;
    if (se.state != SE_STOPPED) return true;
    if (se.tail == se.head) return false;
    const se_entry_t *e = &se.ring[se.tail & SE_MASK];
    return e->sync_k && !(e->tx.flags & WCNC_SEG_FLAG_INDEX_START);
}

bool se_sync_lost(void)
{
    SE_LOCK();
    bool lost = lost_run_in_hand();
    SE_UNLOCK();
    return lost;
}

void se_give_up(uint32_t ramp_ms)
{
    se_ramp_stop(ramp_ms, true);            /* moving: slow down first */
    SE_LOCK();
    if (se.state == SE_STOPPED) {
        stop_and_discard();
        se.ended_last = true;
    }
    se.sync_broken = true;
    SE_UNLOCK();
}

void se_halt(void)
{
    SE_LOCK();
    stop_and_discard();
    se.ended_last = true;
    se.sync_broken = false;
    SE_UNLOCK();
    texec_reset(&se.prod);
}

void se_ramp_stop(uint32_t ramp_ms, bool discard)
{
    SE_LOCK();
    if (se.state == SE_PAUSED) {
        if (discard) {
            stop_and_discard();
            se.ended_last = true;
        }
    } else if (se.state == SE_RUNNING && se.sync == SE_SYNC_WAIT) {
        /* Waiting for the index: nothing is moving, so there is nothing to
         * slow down.  Paused, the wait begins again on resume. */
        if (discard) {
            stop_and_discard();
            se.ended_last = true;
        } else {
            se.state = SE_PAUSED;
            port_stop_alarm();
        }
    } else if (se.state == SE_RUNNING) {
        if (se.sync == SE_SYNC_RUN) {
            /* A stop in the middle of a run played by the spindle.  The
             * pass is spoilt whatever is done now, so the axes stop as in
             * any other move, from the speed the spindle is giving.  The
             * run has lost its step: see lost_too_much() for what becomes
             * of the rest of it. */
            uint32_t ratio = sync_ratio();
            se.sync = SE_SYNC_OFF;
            se.sync_lost = true;
            se.lost_us = se.run.duration > se.seg_pos ? se.run.duration - se.seg_pos : 0;
            se.sync_slip = false;
            se.rate = ratio > SE_RATE_ONE ? SE_RATE_ONE : (ratio < SE_RATE_MIN ? SE_RATE_MIN : ratio);
            se.ramp = 0;
            se.hop_to = se.seg_pos;
            se.alarm = port_now() + SE_ALARM_GUARD_US;
            se.last_alarm = se.alarm;
            port_set_alarm(se.alarm);
        }
        /* A stop that is to drop the queue is not turned into a pause by a
         * later one that is not */
        if (se.ramp < 0 && se.ramp_discard) discard = true;
        bool cruising = (se.rate == SE_RATE_ONE && !se.ramp);
        se.ramp = -ramp_slope(ramp_ms);
        se.stop_slope = -se.ramp;
        se.ramp_discard = discard;
        if (cruising) {
            /* The interrupt may be asleep until a step that is far away.
             * Wake it now, at the point of the segment reached by now. */
            uint64_t now = port_now();
            if (se.alarm > now + SE_ALARM_GUARD_US) {
                uint64_t early = se.alarm - now - SE_ALARM_GUARD_US;
                uint32_t span = se.hop_to - se.seg_pos;
                se.hop_to = early >= span ? se.seg_pos : se.hop_to - (uint32_t)early;
                se.alarm = now + SE_ALARM_GUARD_US;

                port_set_alarm(se.alarm);
            }
        }
    }
    SE_UNLOCK();
}

void se_ramp_resume(uint32_t ramp_ms)
{
    SE_LOCK();
    if (se.state == SE_PAUSED && se.sync == SE_SYNC_WAIT) {
        sync_begin(se.sync_k, se.sync_index);   /* wait for the index afresh */
        se.alarm = port_now() + SE_ALARM_GUARD_US;
        se.last_alarm = se.alarm;
        se.state = SE_RUNNING;
        port_set_alarm(se.alarm);
    } else if (se.state == SE_PAUSED && se.sync_lost && lost_too_much()) {
        /* What is left of a run that lost step with the spindle is more
         * than a tail: it is not played. */
        se.sync_broken = true;
        se.ended_last = true;
        stop_and_discard();
    } else if (se.state == SE_PAUSED) {
        se.rate = SE_RATE_MIN;
        se.ramp = ramp_slope(ramp_ms);
        se.hop_to = se.seg_pos;
        se.alarm = port_now() + SE_ALARM_GUARD_US;
        se.last_alarm = se.alarm;
        se.state = SE_RUNNING;
        port_set_alarm(se.alarm);
    } else if (se.state == SE_RUNNING && se.ramp < 0 && !se.ramp_discard) {
        se.ramp = ramp_slope(ramp_ms);      /* asked to continue before it had stopped */
    }
    SE_UNLOCK();
}

void se_get_position(int32_t position[WCNC_MAX_AXES])
{
    SE_LOCK();
    for (int i = 0; i < WCNC_MAX_AXES; i++) position[i] = se.position[i];
    SE_UNLOCK();
}

void se_set_position(int axis, int32_t value)
{
    if (axis < 0 || axis >= WCNC_MAX_AXES) return;
    SE_LOCK();
    se.position[axis] = value;
    SE_UNLOCK();
}

uint16_t se_current_id(void) { return se.run_id; }

uint32_t se_feed_rate(void)
{
    SE_LOCK();
    uint32_t feed = se.state == SE_RUNNING ? se.feed : 0;
    uint32_t rate = se.rate;
    if (se.sync == SE_SYNC_WAIT) rate = 0;
    else if (se.sync == SE_SYNC_RUN) rate = sync_ratio();
    SE_UNLOCK();
    return (uint32_t)(((uint64_t)feed * rate) >> 16);
}

void se_get_rates(uint32_t rate[WCNC_MAX_AXES])
{
    uint32_t count[WCNC_MAX_AXES], duration;
    bool running;
    SE_LOCK();
    running = se.state == SE_RUNNING;
    duration = se.run.duration;
    for (int i = 0; i < WCNC_MAX_AXES; i++) count[i] = se.run_count[i];
    SE_UNLOCK();
    for (int i = 0; i < WCNC_MAX_AXES; i++)
        rate[i] = (running && duration) ? (uint32_t)(((uint64_t)count[i] * 1000000u) / duration) : 0;
}

bool se_probe_tripped(void) { return se.probe_tripped; }
void se_clear_probe(void)   { se.probe_tripped = false; }

void se_gate(uint8_t axis_mask, const uint8_t input[WCNC_MAX_AXES])
{
    SE_LOCK();
    se.gate_mask = 0;
    if (input)
        for (int i = 0; i < WCNC_MAX_AXES; i++) se.gate_input[i] = input[i];
    se.gate_tripped = 0;
    se.gate_mask = axis_mask;
    SE_UNLOCK();
}

uint8_t se_gate_tripped(void) { return se.gate_tripped; }
