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
 */

#include "step_engine.h"
#include "timed_exec.h"
#include "se_port.h"

#include <string.h>

#define SE_MASK             (SE_QUEUE_SIZE - 1)
#define SE_RATE_MIN         (SE_RATE_ONE / 64)  /* slower than this counts as standing still */
#define SE_HOP_US           500u                /* nominal time per interrupt while the rate is ramping */
#define SE_START_LEAD_US    50u                 /* from se_start() to the start of the first segment */
#define SE_MAX_EVENTS       4                   /* events handled in one interrupt before yielding */
#define SE_ALARM_GUARD_US   4u                  /* an alarm is never set closer than this to the present */
#define SE_OVERDUE_US       2000u               /* se_watchdog(): an alarm this late was missed */

enum { SE_STOPPED = 0, SE_RUNNING, SE_PAUSED };

typedef struct {
    texec_t  tx;                    /* prepared by texec_load() when queued */
    uint32_t feed;                  /* steps/s of its fastest axis */
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
    port_stop_alarm();
}

/* Make the next queued segment the one being played.  False: none left. */
static inline bool SE_ISR_ATTR load_next(void)
{
    if (se.tail == se.head) return false;
    const se_entry_t *e = &se.ring[se.tail & SE_MASK];
    se.run = e->tx;
    se.run_id = e->id;
    se.feed = e->feed;
    for (int i = 0; i < WCNC_MAX_AXES; i++) se.run_count[i] = e->tx.left[i];
    se.popped_us += e->tx.duration;
    se.tail = (uint16_t)(se.tail + 1);
    se.seg_pos = 0;
    se.hop_to = 0;
    return true;
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
    se.seg_pos = se.hop_to;

    /* Rate ramp, by the real time since the previous alarm */
    if (se.ramp) {
        int64_t r = (int64_t)se.rate + (((int64_t)se.ramp * (int64_t)(at - se.last_alarm)) >> 10);
        if (se.ramp < 0 && r <= (int64_t)SE_RATE_MIN) {
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
        if (r >= (int64_t)SE_RATE_ONE) {
            r = SE_RATE_ONE;
            se.ramp = 0;
        }
        se.rate = (uint32_t)r;
    }
    se.last_alarm = at;

    /* Whatever is due */
    bool pulsed = false;
    for (int n = 0; n < SE_MAX_EVENTS && se.seg_pos >= se.ev_time; n++) {
        if (se.ev_mask) {
            if ((se.run.flags & WCNC_SEG_FLAG_PROBE) && port_probe_active()) {
                se.probe_tripped = true;
                se.ended_last = true;
                stop_and_discard();
                SE_ISR_UNLOCK();
                return;
            }
            if (pulse(se.ev_mask)) pulsed = true;
        } else {
            bool last = (se.run.flags & WCNC_SEG_FLAG_LAST) != 0;
            if (!load_next()) {
                se.ended_last = last;
                se.state = SE_STOPPED;
                se.feed = 0;
                port_stop_alarm();
                SE_ISR_UNLOCK();
                return;
            }
        }
        se.ev_time = texec_next(&se.run, &se.ev_mask, se.merge_us);
    }

    /* Sleep until the next event, or for one hop while the rate is not 1 */
    uint32_t delta = se.ev_time > se.seg_pos ? se.ev_time - se.seg_pos : 0;
    uint32_t real;
    if (se.rate == SE_RATE_ONE && !se.ramp) {
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
    uint16_t head = se.head;
    if ((uint16_t)(head - se.tail) >= SE_QUEUE_SIZE - 1) return 0;

    texec_t before = se.prod;
    if (!texec_load(&se.prod, seg, se.min_interval_us)) {
        se.prod = before;
        return -1;
    }
    /* All axes together must not ask for more interrupts than there is
     * time for.  Like a single axis that is too fast, such a segment is
     * played more slowly rather than wrongly. */
    uint64_t steps = 0;
    for (int i = 0; i < WCNC_MAX_AXES; i++) steps += se.prod.left[i];
    if (steps * se.event_us > se.prod.duration) {
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

void se_halt(void)
{
    SE_LOCK();
    stop_and_discard();
    se.ended_last = true;
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
    } else if (se.state == SE_RUNNING) {
        bool cruising = (se.rate == SE_RATE_ONE && !se.ramp);
        se.ramp = -ramp_slope(ramp_ms);
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
    if (se.state == SE_PAUSED) {
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
