/*
 * WiFi CNC Controller - Motion core
 *
 * See motion_core.h.
 */

#include "motion_core.h"
#include "step_engine.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* A stream starts playing once this much motion is queued, or this long
 * after its first segment arrived, or when its last segment is queued. */
#define START_QUEUE_US          100000u
#define START_MAX_WAIT_MS       150u
/* A stream that has run low is held for this long for more to arrive, then
 * what is left of it is played out. */
#define STARVE_WAIT_MS          300u
/* Without a TCP session the first sender is the host until it is quiet for: */
#define HOST_QUIET_MS           3000u
/* Motion the controller generates itself (jog, homing, G-code mode) is
 * queued in short slices, this far ahead. */
#define GEN_SLICE_US            2000u
#define GEN_LOOKAHEAD_US        12000u
#define LEGACY_SLICE_US         4000u
#define LEGACY_LOOKAHEAD_US     40000u
/* Homing gives up when the fast search for a switch takes longer than the
 * first of these (a long axis at a low speed can take minutes), or any
 * other phase longer than the second. */
#define HOME_SEEK_TIMEOUT_MS    600000u
#define HOME_PHASE_TIMEOUT_MS   60000u
#define RAMP_MIN_MS             20u
#define RAMP_MAX_MS             2000u

enum { OWN_NONE = 0, OWN_STREAM, OWN_JOG, OWN_HOME, OWN_LEGACY };
enum { H_START = 0, H_CLEAR, H_SEEK, H_PULLOFF, H_LATCH, H_FINAL, H_DONE };

typedef struct {
    uint8_t  mask;
    int8_t   dir[WCNC_MAX_AXES];
    double   seek[WCNC_MAX_AXES], latch[WCNC_MAX_AXES];
    int32_t  pull[WCNC_MAX_AXES];
    uint8_t  sw[WCNC_MAX_AXES];
    int      phase, next_phase;
    bool     draining;                      /* waiting for queued motion to finish before next_phase */
    uint32_t t_phase;
    bool     marked[WCNC_MAX_AXES];
    int32_t  mark[WCNC_MAX_AXES];
} homing_t;

typedef struct {
    bool     active;
    double   steps[WCNC_MAX_AXES];          /* whole move, steps */
    double   dist;                          /* steps of the longest axis */
    double   ve, vn, vx, acc;               /* entry, cruise, exit speed and acceleration along it */
    double   t1, t2, t3, d1, d2;            /* durations of the three phases, distance of the first two */
    uint32_t total_us, t_us;
    int64_t  emitted[WCNC_MAX_AXES];        /* 1/256 steps handed to the engine so far */
    uint8_t  flags;
    uint16_t id;
    bool     queue_empty;                   /* the last look at the legacy queue found nothing */
} legacy_t;

static struct {
    mc_config_t cfg;
    wcnc_machine_state_t state;
    uint8_t  alarm;
    uint8_t  owner;                         /* who is producing motion */

    /* host */
    uint32_t tcp_ip, host_ip;
    uint16_t status_port;
    uint32_t last_rx_ms, last_status_ms, tx_seq;
    bool     watchdog_armed, uses_watchdog;
    uint8_t  io_flags;

    /* stream */
    uint8_t  stream_id, stream_flags;
    uint16_t next_seg;
    bool     stream_closed;
    int      pending_stream;                /* stream to switch to when the decelerating flush is done, or -1 */
    bool     have_first, last_queued, starved;
    bool     play_out;                      /* gave up waiting for more: finish what is queued */
    bool     resume_ramped;                 /* the stream ran dry mid-move: start again gently */
    uint32_t first_queued_ms, starve_ms;

    /* jog and homing share a velocity generator */
    double   v[WCNC_MAX_AXES], target[WCNC_MAX_AXES], frac[WCNC_MAX_AXES];
    bool     has_deadline[WCNC_MAX_AXES];
    uint32_t deadline_ms[WCNC_MAX_AXES];
    homing_t home;
    uint8_t  homed_mask;

    legacy_t leg;
    bool     probe_latched;

    mc_outputs_t out;
    bool     drivers_on;
    uint32_t last_motion_ms;
    uint32_t hold_ramp_ms;
} mc;

/* ===================================================================
 * Helpers
 * =================================================================== */

static void logf_(const char *fmt, int a, int b)
{
    char line[96];
    snprintf(line, sizeof(line), fmt, a, b);
    mc_hal_log(line);
}

static bool in_fault(void)
{
    return mc.state == WCNC_STATE_ESTOP || mc.state == WCNC_STATE_ALARM;
}

static void drivers(bool on)
{
    if (on) mc.last_motion_ms = mc_hal_ms();
    if (mc.drivers_on != on) {
        mc.drivers_on = on;
        mc_hal_drivers(on);
    }
}

static void outputs_off(void)
{
    memset(&mc.out, 0, sizeof(mc.out));
    mc_hal_apply_outputs(&mc.out);
}

uint32_t mc_max_step_rate(void)
{
    uint32_t interval = se_min_interval_us();
    return interval ? 1000000u / interval : 50000u;
}

/* Time to stop from the speed of the segment now playing, at the axes'
 * configured accelerations. */
static uint32_t stop_time_ms(void)
{
    uint32_t rate[WCNC_MAX_AXES], t = RAMP_MIN_MS;
    se_get_rates(rate);
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        uint32_t a = mc.cfg.accel[i] ? mc.cfg.accel[i] : 1;
        uint64_t ms = ((uint64_t)rate[i] * 1000u) / a;
        if (ms > t) t = ms > RAMP_MAX_MS ? RAMP_MAX_MS : (uint32_t)ms;
    }
    return t;
}

/* The same, shortened if need be so that the stop is over before the
 * queued motion runs out (slowing down uses up half the ramp time of it). */
static uint32_t stop_time_within_queue_ms(void)
{
    uint32_t t = stop_time_ms();
    uint32_t queued_ms = se_queued_us() / 1000u;
    uint32_t fits = queued_ms > 10 ? 2 * (queued_ms - 10) : 0;
    if (t > fits) t = fits;
    return t < RAMP_MIN_MS ? RAMP_MIN_MS : t;
}

static void generator_reset(void)
{
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        mc.v[i] = 0.0;
        mc.target[i] = 0.0;
        mc.frac[i] = 0.0;
        mc.has_deadline[i] = false;
    }
}

static void stream_idle(void)
{
    mc.have_first = false;
    mc.last_queued = false;
    mc.starved = false;
    mc.play_out = false;
    mc.resume_ramped = false;
}

static void enter_fault(wcnc_machine_state_t state, uint8_t alarm)
{
    logf_(state == WCNC_STATE_ESTOP ? "E-stop (code %d)" : "Alarm %d", alarm, 0);
    se_halt();
    se_gate(0, NULL);
    generator_reset();
    mc.leg.active = false;
    mc_hal_legacy_clear();
    mc.owner = OWN_NONE;
    mc.stream_closed = true;
    mc.pending_stream = -1;
    mc.stream_flags &= (uint8_t)~WCNC_STREAM_FLUSHING;
    stream_idle();
    outputs_off();
    drivers(false);
    mc.state = state;
    mc.alarm = alarm;
}

static void send_packet(void *pkt, uint8_t type, size_t total, uint32_t ip, uint16_t port)
{
    wcnc_finalize_packet(pkt, type, (uint16_t)(total - sizeof(wcnc_header_t)), mc.tx_seq++, mc_hal_ms() * 1000u);
    mc_hal_udp_send(ip, port, pkt, total);
}

/* ===================================================================
 * Motion stream from a host
 * =================================================================== */

static void apply_flush(uint8_t stream)
{
    if (mc.owner == OWN_STREAM || mc.owner == OWN_LEGACY || mc.owner == OWN_NONE) {
        se_halt();
        mc.leg.active = false;
        mc_hal_legacy_clear();
        mc.owner = OWN_NONE;
        if (mc.state == WCNC_STATE_RUN || mc.state == WCNC_STATE_HOLD) mc.state = WCNC_STATE_IDLE;
    }
    mc.stream_id = stream;
    mc.next_seg = 0;
    mc.stream_flags = 0;
    mc.stream_closed = false;
    mc.pending_stream = -1;
    stream_idle();
}

/* Slow to a stop, then drop what is queued and go on as `stream`.  A stream
 * has to stop inside what it has queued.  Speed-described motion has only a
 * moment of itself in the step engine and is fed on while it slows down
 * (tick_legacy), so it takes the time the machine's acceleration asks for. */
static void flush_decelerating(uint8_t stream)
{
    mc.pending_stream = stream;
    mc.stream_flags |= WCNC_STREAM_FLUSHING;
    se_ramp_stop(mc.owner == OWN_LEGACY ? stop_time_ms() : stop_time_within_queue_ms(), true);
}

static void on_flush(const uint8_t *payload, size_t len)
{
    if (len < 2) return;
    uint8_t stream = payload[0], mode = payload[1];
    if (mc.pending_stream >= 0) return;                 /* already on its way */
    if (stream == mc.stream_id) return;                 /* a repeat of the flush already done */

    bool playing = (mc.owner == OWN_STREAM || mc.owner == OWN_LEGACY) && se_active();
    if (mode == WCNC_FLUSH_DECELERATE && playing && !in_fault()) flush_decelerating(stream);
    else apply_flush(stream);
}

static void on_motion(const uint8_t *payload, size_t len)
{
    if (len < 4) return;
    uint8_t count = payload[0], flags = payload[1], stream = payload[2];
    if (count == 0 || count > WCNC_MAX_SEGMENTS_PER_PACKET ||
        len < 4 + (size_t)count * sizeof(wcnc_motion_segment_t))
        return;
    if (in_fault() || mc.state == WCNC_STATE_JOG || mc.state == WCNC_STATE_HOMING) return;

    bool sequenced = (flags & WCNC_MOTION_FLAG_SEQUENCED) != 0;
    if (sequenced && (stream != mc.stream_id || mc.stream_closed || mc.pending_stream >= 0)) return;

    for (int n = 0; n < count; n++) {
        wcnc_motion_segment_t seg;
        memcpy(&seg, payload + 4 + (size_t)n * sizeof(seg), sizeof(seg));
        if (sequenced && seg.segment_id != mc.next_seg) continue;   /* duplicate or out of order */

        if (!(seg.flags & WCNC_SEG_FLAG_TIMED)) {
            /* Protocol 1.0/1.1 segment: goes the way of G-code mode. */
            if (mc.owner == OWN_STREAM && (se_active() || se_count())) break;
            if (!mc_hal_legacy_push(&seg)) break;
        } else {
            if (mc.owner == OWN_LEGACY) {
                /* A host takes over from G-code mode. */
                se_halt();
                mc.leg.active = false;
                mc_hal_legacy_clear();
                mc.owner = OWN_NONE;
                mc.state = WCNC_STATE_IDLE;
            }
            if (mc.owner == OWN_NONE) {
                se_reset_stream();
                mc.owner = OWN_STREAM;
            }
            int r = se_push(&seg);
            if (r == 0) break;                          /* full: the host sends it again */
            if (r < 0) {
                /* Cannot be played.  Refusing it silently would stall the
                 * stream, playing on without it would lose position. */
                enter_fault(WCNC_STATE_ALARM, WCNC_ALARM_BUFFER_UNDERRUN);
                return;
            }
            if (!mc.have_first) {
                mc.have_first = true;
                mc.first_queued_ms = mc_hal_ms();
            }
            mc.last_queued = (seg.flags & WCNC_SEG_FLAG_LAST) != 0;
            mc.play_out = false;
        }
        if (sequenced) mc.next_seg = (uint16_t)(mc.next_seg + 1);
    }
}

static void tick_stream(void)
{
    if (mc.owner != OWN_STREAM) {
        if (mc.pending_stream >= 0 && !se_active()) apply_flush((uint8_t)mc.pending_stream);
        return;
    }
    if (mc.pending_stream >= 0) {
        if (!se_active()) apply_flush((uint8_t)mc.pending_stream);
        return;
    }
    if (se_sync_broken()) {
        /* A run played in step with the spindle was broken into (a stop in
         * the middle of it, a queue that ran dry, axes that could not keep
         * up).  The engine slows to a standstill and drops the rest; so is
         * whatever else the host had sent for this stream, and the host is
         * told why. */
        if (se_active()) return;
        se_halt();
        mc.stream_flags |= WCNC_STREAM_SYNC_LOST;
        mc.stream_closed = true;
        mc.owner = OWN_NONE;
        if (mc.state == WCNC_STATE_RUN || mc.state == WCNC_STATE_HOLD) mc.state = WCNC_STATE_IDLE;
        stream_idle();
        mc_hal_log("A move in step with the spindle was interrupted: abandoned");
        return;
    }
    if (se_probe_tripped()) {
        /* Contact: the engine has stopped where it touched and dropped the
         * rest.  Whatever else the host had sent for this stream is void. */
        se_clear_probe();
        se_reset_stream();
        mc.probe_latched = true;
        mc.stream_flags |= WCNC_STREAM_PROBE_TRIPPED;
        mc.stream_closed = true;
        mc.owner = OWN_NONE;
        if (mc.state == WCNC_STATE_RUN || mc.state == WCNC_STATE_HOLD) mc.state = WCNC_STATE_IDLE;
        stream_idle();
        return;
    }

    if (!se_active()) {
        if (mc.state == WCNC_STATE_RUN) {
            /* The queue ran out */
            if (!se_ended_on_last() && mc.next_seg != 0) {
                mc.stream_flags |= WCNC_STREAM_UNDERRUN;
                mc.resume_ramped = true;                /* whatever comes next continues a move */
            } else {
                mc.resume_ramped = false;
            }
            mc.state = WCNC_STATE_IDLE;
            mc.have_first = se_count() > 0;
            mc.first_queued_ms = mc_hal_ms();
            mc.starved = false;
            mc.play_out = false;
        }
        if (se_count() == 0) {
            if (mc.state == WCNC_STATE_IDLE) {
                bool mid_move = mc.resume_ramped;       /* ran dry in mid-move: what comes next starts gently */
                mc.owner = OWN_NONE;
                stream_idle();
                mc.resume_ramped = mid_move;
            }
            return;
        }
        if (mc.state != WCNC_STATE_IDLE) return;
        if (se_sync_lost() && !mc.last_queued) {
            /* More of a run that lost step with the spindle when the queue
             * ran dry in it.  Only its tail would still be played, and a
             * tail ends the stream: until that end is here nothing is
             * started, and if the run goes on arriving instead, or nothing
             * more comes, it is given up without another step. */
            if (se_queued_us() >= START_QUEUE_US ||
                (uint32_t)(mc_hal_ms() - mc.first_queued_ms) >= STARVE_WAIT_MS)
                se_give_up(0);
            return;
        }
        if (se_queued_us() >= START_QUEUE_US || mc.last_queued ||
            (uint32_t)(mc_hal_ms() - mc.first_queued_ms) >= START_MAX_WAIT_MS) {
            drivers(true);
            if (mc.resume_ramped) se_start_ramped(mc.hold_ramp_ms ? mc.hold_ramp_ms : 300);
            else se_start();
            mc.resume_ramped = false;
            mc.state = WCNC_STATE_RUN;
            mc.stream_flags &= (uint8_t)~WCNC_STREAM_UNDERRUN;
            mc.starved = false;
        }
        return;
    }

    if (mc.state == WCNC_STATE_HOLD && se_paused() && se_sync_lost() && !mc.last_queued) {
        /* Held in the middle of a run played by the spindle.  It has lost
         * its step, and there is more of it to come than is queued: it will
         * not be played on, and the host may as well know now. */
        se_give_up(0);
        return;
    }
    if (mc.state != WCNC_STATE_RUN) return;

    if (se_sync_slipped()) {
        /* The axes are no longer where the spindle would have them: nothing
         * good comes of going on. */
        se_give_up(stop_time_within_queue_ms());
        return;
    }

    /* Playing.  If the host falls behind, slow to a stop inside what is
     * still queued rather than running off the end of it at speed, and pick
     * up again when more has arrived. */
    if (!mc.starved) {
        if (!mc.last_queued && !mc.play_out && !se_ramping() && !se_paused()) {
            uint32_t trigger = stop_time_ms() * 500u + 15000u;
            if (trigger > START_QUEUE_US) trigger = START_QUEUE_US;
            if (se_queued_us() < trigger) {
                mc.hold_ramp_ms = stop_time_ms();
                se_ramp_stop(stop_time_within_queue_ms(), false);
                mc.starved = true;
                mc.starve_ms = mc_hal_ms();
                mc.stream_flags |= WCNC_STREAM_UNDERRUN;
            }
        }
    } else if (se_sync_lost() && !mc.last_queued) {
        /* The stop broke into a run played by the spindle, which has lost
         * its step by it.  Only a tail is still played: the end of the
         * stream, arriving a moment late.  If the run goes on arriving
         * instead, or nothing more comes, it is given up. */
        if (se_queued_us() >= START_QUEUE_US || (uint32_t)(mc_hal_ms() - mc.starve_ms) > STARVE_WAIT_MS) {
            se_give_up(stop_time_within_queue_ms());
            mc.starved = false;
        }
    } else if (mc.last_queued || se_queued_us() >= START_QUEUE_US) {
        se_ramp_resume(mc.hold_ramp_ms);
        mc.starved = false;
    } else if ((uint32_t)(mc_hal_ms() - mc.starve_ms) > STARVE_WAIT_MS) {
        /* Nothing more is coming, or not soon: the host may simply not have
         * marked its last segment.  Finish what there is. */
        se_ramp_resume(mc.hold_ramp_ms);
        mc.starved = false;
        mc.play_out = true;
    }

}

/* ===================================================================
 * Velocity generator (jog and homing)
 * =================================================================== */

/* Move v towards target at the axis' acceleration over dt seconds. */
static void ramp_axis(int i, double target, double dt)
{
    double a = mc.cfg.accel[i] ? (double)mc.cfg.accel[i] : 1.0;
    double dv = target - mc.v[i], step = a * dt;
    if (dv > step) dv = step;
    if (dv < -step) dv = -step;
    mc.v[i] += dv;
}

/* Queue one slice that takes each axis from v_before[] to mc.v[]. */
static bool emit_slice(const double v_before[WCNC_MAX_AXES])
{
    wcnc_motion_segment_t seg;
    memset(&seg, 0, sizeof(seg));
    seg.duration_us = GEN_SLICE_US;
    seg.flags = WCNC_SEG_FLAG_TIMED | WCNC_SEG_FLAG_SUBSTEP;
    double frac[WCNC_MAX_AXES];
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        double want = (v_before[i] + mc.v[i]) * 0.5 * (GEN_SLICE_US / 1.0e6) * WCNC_SUBSTEP_SCALE + mc.frac[i];
        double whole = floor(want + 0.5);
        seg.steps[i] = (int32_t)whole;
        frac[i] = want - whole;
    }
    if (se_push(&seg) != 1) return false;
    memcpy(mc.frac, frac, sizeof(frac));
    return true;
}

static bool generator_still(void)
{
    for (int i = 0; i < WCNC_MAX_AXES; i++)
        if (mc.v[i] != 0.0 || mc.target[i] != 0.0) return false;
    return true;
}

/* ===================================================================
 * Jog
 * =================================================================== */

static void on_jog(const uint8_t *payload, size_t len)
{
    if (len < 8) return;
    uint8_t axis = payload[0], flags = payload[2];
    int8_t dir = (int8_t)payload[1];
    uint32_t speed;
    memcpy(&speed, payload + 4, 4);
    if (axis >= WCNC_MAX_AXES || (dir != 1 && dir != -1)) return;

    if (mc.state == WCNC_STATE_IDLE) {
        if (mc.owner != OWN_NONE || se_active() || se_count() || mc.leg.active) return;
        generator_reset();
        se_reset_stream();
        mc.resume_ramped = false;
        mc.owner = OWN_JOG;
        mc.state = WCNC_STATE_JOG;
        drivers(true);
    } else if (mc.state != WCNC_STATE_JOG) {
        return;
    }
    if (speed > mc.cfg.max_rate[axis]) speed = mc.cfg.max_rate[axis];
    if (speed > mc_max_step_rate()) speed = mc_max_step_rate();
    mc.target[axis] = (double)dir * (double)speed;
    mc.has_deadline[axis] = (flags & WCNC_JOG_FLAG_DEADMAN) != 0;
    mc.deadline_ms[axis] = mc_hal_ms() + WCNC_JOG_DEADMAN_MS;
}

static void on_jog_stop(const uint8_t *payload, size_t len)
{
    uint8_t axis = len >= 1 ? payload[0] : 0xFF;
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        if (axis == 0xFF || axis == i) {
            mc.target[i] = 0.0;
            mc.has_deadline[i] = false;
        }
    }
}

static void tick_jog(void)
{
    if (mc.state != WCNC_STATE_JOG) return;
    uint32_t now = mc_hal_ms();
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        if (mc.has_deadline[i] && (int32_t)(now - mc.deadline_ms[i]) > 0) {
            mc.target[i] = 0.0;                         /* the host stopped asking */
            mc.has_deadline[i] = false;
        }
    }
    while (!generator_still() && se_queued_us() < GEN_LOOKAHEAD_US && se_free() > 0) {
        double before[WCNC_MAX_AXES];
        memcpy(before, mc.v, sizeof(before));
        for (int i = 0; i < WCNC_MAX_AXES; i++) ramp_axis(i, mc.target[i], GEN_SLICE_US / 1.0e6);
        if (!emit_slice(before)) {
            memcpy(mc.v, before, sizeof(before));
            break;
        }
    }
    if (se_count() > 0 && !se_active()) se_start();
    if (generator_still() && !se_active() && se_count() == 0) {
        mc.state = WCNC_STATE_IDLE;
        mc.owner = OWN_NONE;
    }
}

/* ===================================================================
 * Homing
 *
 *   clear    (only if a switch is already operated) back off it
 *   seek     fast towards the switch; each axis stops on its own switch
 *   pull-off back off until the switch releases, plus the pull-off distance
 *   latch    slow towards the switch again, stopping within a step of it
 *   final    back off as before; that point becomes position 0
 *
 * Towards a switch the step engine itself holds an axis the moment its
 * switch operates, so where it stops does not depend on this task's timing.
 * =================================================================== */

static void home_set_phase(int phase)
{
    homing_t *h = &mc.home;
    h->phase = phase;
    h->draining = false;
    h->t_phase = mc_hal_ms();
    memset(h->marked, 0, sizeof(h->marked));
    for (int i = 0; i < WCNC_MAX_AXES; i++) mc.v[i] = 0.0;
    se_gate((phase == H_SEEK || phase == H_LATCH) ? h->mask : 0, h->sw);
}

/* Change phase once the motion already queued has been played. */
static void home_then(int phase)
{
    mc.home.next_phase = phase;
    mc.home.draining = true;
}

static bool start_homing(uint8_t mask, uint8_t dir_mask, const double seek[], const double latch[],
                         const uint32_t pull[], const uint8_t sw_in[])
{
    if (mc.cfg.io_module || mc.state != WCNC_STATE_IDLE || mc.owner != OWN_NONE ||
        se_active() || se_count() || mc.leg.active)
        return false;
    mask &= (uint8_t)((1u << WCNC_MAX_AXES) - 1);
    if (!mask) return false;

    homing_t *h = &mc.home;
    memset(h, 0, sizeof(*h));
    int owner = -1;
    for (int i = 0; i < WCNC_MAX_AXES; i++)
        if ((mask & (1u << i)) && sw_in[i] < WCNC_MAX_AXES && owner < 0) owner = i;
    if (owner < 0) return false;                        /* no switch to home on */

    double fastest = (double)mc_max_step_rate();
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        if (!(mask & (1u << i))) continue;
        h->sw[i] = sw_in[i] < WCNC_MAX_AXES ? sw_in[i] : sw_in[owner];
        h->dir[i] = (dir_mask & (1u << i)) ? 1 : -1;
        double limit = mc.cfg.max_rate[i] ? (double)mc.cfg.max_rate[i] : fastest;
        if (limit > fastest) limit = fastest;
        h->seek[i] = seek[i] < 1.0 ? 1.0 : (seek[i] > limit ? limit : seek[i]);
        h->latch[i] = latch[i] < 1.0 ? 1.0 : (latch[i] > h->seek[i] ? h->seek[i] : latch[i]);
        h->pull[i] = pull[i] > 0x3FFFFFFFu ? 0x3FFFFFFF : (int32_t)pull[i];
    }
    h->mask = mask;
    mc.homed_mask &= (uint8_t)~mask;
    generator_reset();
    se_reset_stream();
    mc.resume_ramped = false;
    mc.owner = OWN_HOME;
    mc.state = WCNC_STATE_HOMING;
    drivers(true);
    home_set_phase(H_START);
    logf_("Homing axes 0x%02X", mask, 0);
    return true;
}

static void on_home(const uint8_t *payload, size_t len)
{
    if (len < 4) return;
    uint8_t mask = payload[0], dir_mask = payload[1], flags = payload[2];
    double seek[WCNC_MAX_AXES], latch[WCNC_MAX_AXES];
    uint32_t pull[WCNC_MAX_AXES];
    uint8_t sw[WCNC_MAX_AXES];
    const size_t full = sizeof(wcnc_home_ex_packet_t) - sizeof(wcnc_header_t);
    if ((flags & WCNC_HOME_FLAG_PARAMS) && len >= full) {
        uint32_t raw[3 * WCNC_MAX_AXES];
        memcpy(raw, payload + 4, sizeof(raw));
        for (int i = 0; i < WCNC_MAX_AXES; i++) {
            seek[i] = (double)raw[i];
            latch[i] = (double)raw[WCNC_MAX_AXES + i];
            pull[i] = raw[2 * WCNC_MAX_AXES + i];
            sw[i] = payload[4 + sizeof(raw) + i];
        }
    } else {
        dir_mask = mc.cfg.homing_dir_mask;
        for (int i = 0; i < WCNC_MAX_AXES; i++) {
            seek[i] = (double)mc.cfg.homing_seek_rate;
            latch[i] = (double)mc.cfg.homing_feed_rate;
            pull[i] = mc.cfg.homing_pulloff;
            sw[i] = (uint8_t)i;
        }
    }
    start_homing(mask, dir_mask, seek, latch, pull, sw);
}

static void home_finish(void)
{
    homing_t *h = &mc.home;
    for (int i = 0; i < WCNC_MAX_AXES; i++)
        if (h->mask & (1u << i)) se_set_position(i, 0);
    se_gate(0, NULL);
    mc.homed_mask |= h->mask;
    mc.state = WCNC_STATE_IDLE;
    mc.owner = OWN_NONE;
    generator_reset();
    logf_("Homing done 0x%02X", h->mask, 0);
    if (mc.host_ip) {
        wcnc_home_complete_packet_t pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.axis_mask = h->mask;
        pkt.success = 1;
        send_packet(&pkt, WCNC_PKT_HOME_COMPLETE, sizeof(pkt), mc.host_ip, mc.status_port);
    }
}

/* Work out the velocity each axis should have next.  Returns false when the
 * phase has nothing more to generate. */
static bool home_step(double dt)
{
    homing_t *h = &mc.home;
    uint8_t raw = mc_hal_limit_inputs();
    int32_t pos[WCNC_MAX_AXES];
    se_get_position(pos);
    bool all_on = true, any_on = false, done = true;

    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        if (!(h->mask & (1u << i))) continue;
        bool on = (raw >> h->sw[i]) & 1;
        if (on) any_on = true; else all_on = false;

        switch (h->phase) {
        case H_CLEAR: case H_PULLOFF: case H_FINAL: {
            if (on) {
                h->marked[i] = false;
            } else if (!h->marked[i]) {
                h->marked[i] = true;
                h->mark[i] = pos[i];
            }
            int32_t gone = pos[i] - h->mark[i];
            if (gone < 0) gone = -gone;
            bool far = h->marked[i] && gone >= h->pull[i];
            ramp_axis(i, far ? 0.0 : -(double)h->dir[i] * h->latch[i], dt);
            if (!far || mc.v[i] != 0.0) done = false;
            break;
        }
        case H_SEEK:
            if (on) mc.v[i] = 0.0;                      /* the engine has already held it */
            else ramp_axis(i, (double)h->dir[i] * h->seek[i], dt);
            break;
        case H_LATCH:
            mc.v[i] = on ? 0.0 : (double)h->dir[i] * h->latch[i];
            break;
        default:
            break;
        }
    }

    switch (h->phase) {
    case H_START:
        home_then(any_on ? H_CLEAR : H_SEEK);
        return false;
    case H_CLEAR:
        if (done) { home_then(H_SEEK); return false; }
        break;
    case H_PULLOFF:
        if (done) { home_then(H_LATCH); return false; }
        break;
    case H_FINAL:
        if (done) { home_then(H_DONE); return false; }
        break;
    case H_SEEK:
        if (all_on) { home_then(H_PULLOFF); return false; }
        break;
    case H_LATCH:
        if (all_on) { home_then(H_FINAL); return false; }
        break;
    default:
        return false;
    }
    return true;
}

static void tick_home(void)
{
    if (mc.state != WCNC_STATE_HOMING) return;
    homing_t *h = &mc.home;

    if (h->draining) {
        if (se_active() || se_count()) return;
        if (h->next_phase == H_DONE) {
            home_finish();
            return;
        }
        se_reset_stream();
        memset(mc.frac, 0, sizeof(mc.frac));
        home_set_phase(h->next_phase);
    }
    if ((uint32_t)(mc_hal_ms() - h->t_phase) > (h->phase == H_SEEK ? HOME_SEEK_TIMEOUT_MS : HOME_PHASE_TIMEOUT_MS)) {

        enter_fault(WCNC_STATE_ALARM, WCNC_ALARM_HOME_FAIL);
        return;
    }
    while (!h->draining && se_queued_us() < GEN_LOOKAHEAD_US && se_free() > 0) {
        double before[WCNC_MAX_AXES];
        memcpy(before, mc.v, sizeof(before));
        if (!home_step(GEN_SLICE_US / 1.0e6)) break;
        /* An axis stopped by its switch must not coast on in the slice */
        for (int i = 0; i < WCNC_MAX_AXES; i++)
            if (mc.v[i] == 0.0 && (h->phase == H_SEEK || h->phase == H_LATCH)) before[i] = 0.0;
        if (!emit_slice(before)) {
            memcpy(mc.v, before, sizeof(before));
            break;
        }
    }
    if (se_count() > 0 && !se_active()) se_start();
}

/* ===================================================================
 * Segments in the old form (entry speed, exit speed, acceleration)
 *
 * G-code mode and hosts older than protocol 1.2 describe a move by its
 * speeds.  Each is worked out as accelerate / cruise / decelerate along its
 * longest axis and handed to the step engine as timed slices.
 * =================================================================== */

static void legacy_begin(const wcnc_motion_segment_t *seg)
{
    legacy_t *l = &mc.leg;
    bool queue_empty = l->queue_empty;
    memset(l, 0, sizeof(*l));
    l->queue_empty = queue_empty;

    double dist = 0.0;
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        l->steps[i] = (double)seg->steps[i];
        if (fabs(l->steps[i]) > dist) dist = fabs(l->steps[i]);
    }
    if (dist < 1.0) return;                             /* nothing to move */

    double vmax = (double)mc_max_step_rate();
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        if (l->steps[i] == 0.0 || !mc.cfg.max_rate[i]) continue;
        double lim = (double)mc.cfg.max_rate[i] * dist / fabs(l->steps[i]);
        if (lim < vmax) vmax = lim;
    }
    /* A duration, if given, is the time the move would take at its cruise
     * speed: it is never played faster than that. */
    if (seg->duration_us) {
        double cruise = dist * 1.0e6 / (double)seg->duration_us;
        if (cruise < 1.0) cruise = 1.0;
        if (cruise < vmax) vmax = cruise;
    }
    double a = (double)seg->acceleration / 100.0;
    if (a < 1.0) a = 1.0;

    double ve = sqrt((double)seg->entry_speed_sqr / 1000.0);
    double vx = sqrt((double)seg->exit_speed_sqr / 1000.0);
    if (ve > vmax) ve = vmax;
    if (vx > vmax) vx = vmax;
    /* The speeds at the two ends must be reachable from each other */
    double reach = sqrt(vx * vx + 2.0 * a * dist);
    if (ve > reach) ve = reach;
    reach = sqrt(ve * ve + 2.0 * a * dist);
    if (vx > reach) vx = reach;

    double peak = sqrt((2.0 * a * dist + ve * ve + vx * vx) / 2.0);
    double vn = peak < vmax ? peak : vmax;
    if (vn < ve) vn = ve;
    if (vn < vx) vn = vx;
    if (vn < 1.0) vn = 1.0;

    l->acc = a;
    l->ve = ve; l->vn = vn; l->vx = vx;
    l->d1 = (vn * vn - ve * ve) / (2.0 * a);
    double d3 = (vn * vn - vx * vx) / (2.0 * a);
    l->d2 = dist - l->d1 - d3;
    if (l->d2 < 0.0) l->d2 = 0.0;
    l->t1 = (vn - ve) / a;
    l->t3 = (vn - vx) / a;
    l->t2 = l->d2 / vn;
    double total = l->t1 + l->t2 + l->t3;
    l->total_us = total * 1.0e6 >= 4.0e9 ? 4000000000u : (uint32_t)ceil(total * 1.0e6);
    if (l->total_us < 1) l->total_us = 1;
    l->dist = dist;
    l->flags = seg->flags;
    l->id = seg->segment_id;
    l->active = true;
    if (seg->flags & WCNC_SEG_FLAG_PROBE) mc.probe_latched = false;
}

/* Distance along the move at time t seconds. */
static double legacy_distance(const legacy_t *l, double t)
{
    if (t <= 0.0) return 0.0;
    if (t < l->t1) return l->ve * t + 0.5 * l->acc * t * t;
    t -= l->t1;
    if (t < l->t2) return l->d1 + l->vn * t;
    t -= l->t2;
    if (t < l->t3) return l->d1 + l->d2 + l->vn * t - 0.5 * l->acc * t * t;
    return l->dist;
}

static bool legacy_emit(void)
{
    legacy_t *l = &mc.leg;
    uint32_t left = l->total_us - l->t_us;
    uint32_t dur = left > LEGACY_SLICE_US ? LEGACY_SLICE_US : left;
    bool final = (dur == left);
    double s = final ? l->dist : legacy_distance(l, (double)(l->t_us + dur) / 1.0e6);
    if (s > l->dist) s = l->dist;

    wcnc_motion_segment_t seg;
    memset(&seg, 0, sizeof(seg));
    int64_t now[WCNC_MAX_AXES];
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        now[i] = (int64_t)floor(l->steps[i] * WCNC_SUBSTEP_SCALE * (s / l->dist) + 0.5);
        seg.steps[i] = (int32_t)(now[i] - l->emitted[i]);
    }
    seg.duration_us = dur;
    seg.segment_id = l->id;
    seg.flags = WCNC_SEG_FLAG_TIMED | WCNC_SEG_FLAG_SUBSTEP | (l->flags & WCNC_SEG_FLAG_PROBE);
    if (se_push(&seg) != 1) return false;
    memcpy(l->emitted, now, sizeof(now));
    l->t_us += dur;
    if (final) l->active = false;
    return true;
}

static void tick_legacy(void)
{
    legacy_t *l = &mc.leg;
    if (mc.cfg.io_module || in_fault() || mc.state == WCNC_STATE_JOG || mc.state == WCNC_STATE_HOMING) return;
    if (mc.owner != OWN_NONE && mc.owner != OWN_LEGACY) return;

    /* While a stop is slowing the axes down - a hold, or a flush that will
     * drop the rest - the path is fed on, so that they slow down along it
     * instead of running off the end of the little that is queued. */
    bool slowing = mc.owner == OWN_LEGACY && se_ramping();
    bool flushing = mc.pending_stream >= 0;
    if (flushing && !slowing) return;

    if (mc.owner == OWN_LEGACY && se_probe_tripped()) {
        /* Contact: drop the rest of the probing move, carry on with what follows. */
        se_clear_probe();
        se_reset_stream();
        mc.probe_latched = true;
        l->active = false;
    }
    if (mc.state != WCNC_STATE_HOLD || slowing) {
        while (se_queued_us() < LEGACY_LOOKAHEAD_US && se_free() > 0) {
            if (!l->active) {
                wcnc_motion_segment_t seg;
                if (!mc_hal_legacy_pop(&seg)) {
                    l->queue_empty = true;
                    break;
                }
                l->queue_empty = false;
                if (mc.owner == OWN_NONE) {
                    se_reset_stream();
                    mc.owner = OWN_LEGACY;
                }
                legacy_begin(&seg);
                continue;
            }
            if (!legacy_emit()) break;
        }
    }
    if (mc.owner != OWN_LEGACY || flushing) return;

    if (se_count() > 0 && !se_active()) {
        drivers(true);
        se_start();
    }
    if (se_active() || l->active || se_count() > 0) {
        if (mc.state == WCNC_STATE_IDLE) mc.state = WCNC_STATE_RUN;
    } else if (l->queue_empty && mc.state != WCNC_STATE_HOLD) {
        if (mc.state == WCNC_STATE_RUN) mc.state = WCNC_STATE_IDLE;
        mc.owner = OWN_NONE;
    }
}

/* ===================================================================
 * Outputs, reset, hold
 * =================================================================== */

static void on_io(const uint8_t *payload, size_t len, uint16_t port)
{
    if (len < 6) return;
    uint8_t misc = payload[0], spindle = payload[1], coolant = payload[4], flags = payload[5];
    uint16_t rpm, io = 0;
    memcpy(&rpm, payload + 2, 2);
    if (len >= 8) memcpy(&io, payload + 6, 2);

    mc.io_flags = flags;
    /* The watchdog follows the flag: a host that stops setting it, as the
     * last thing it does before closing, stands the watchdog down. */
    mc.watchdog_armed = (flags & WCNC_IO_FLAG_WATCHDOG) != 0;
    if (mc.watchdog_armed) mc.uses_watchdog = true;
    mc.status_port = (flags & WCNC_IO_FLAG_REPLY_TO_SOURCE) ? port : WCNC_UDP_STATUS_PORT;
    if (in_fault()) return;                             /* outputs stay off until the fault is cleared */

    mc.out.misc = misc & 0x1F;
    mc.out.spindle = spindle <= 2 ? spindle : 0;
    mc.out.rpm = rpm;
    mc.out.coolant = coolant & 3;
    mc.out.io = io ? io : misc;                         /* older hosts drive channels 0-7 through misc */
    if (mc.uses_watchdog) mc.out.charge_pump = mc.watchdog_armed && (flags & WCNC_IO_FLAG_CHARGE_PUMP);
    else mc.out.charge_pump = mc.tcp_ip != 0;           /* protocol 1.1: on while a host is connected */
    mc_hal_apply_outputs(&mc.out);
}

void mc_estop(void)
{
    if (mc.state != WCNC_STATE_ESTOP) enter_fault(WCNC_STATE_ESTOP, WCNC_ALARM_ESTOP_ACTIVE);
}

bool mc_reset(void)
{
    if (!in_fault()) {
        /* A host older than protocol 1.2 has no other way of saying "stop,
         * and forget the rest" (the Stop button of TiggyUGS sends this): its
         * motion is brought to rest and what it had queued is dropped. */
        if (mc.owner == OWN_LEGACY && mc.pending_stream < 0) {
            if (se_active() && !se_paused()) flush_decelerating(mc.stream_id);
            else apply_flush(mc.stream_id);
        } else if (mc.owner == OWN_NONE) {
            mc_hal_legacy_clear();
        }
        return true;
    }
    if (mc_hal_estop_input()) return false;             /* not while the button is still in */
    mc.state = WCNC_STATE_IDLE;
    mc.alarm = WCNC_ALARM_NONE;
    se_clear_probe();
    se_reset_stream();
    mc_hal_log("Reset");
    return true;
}

void mc_feed_hold(void)
{
    if (mc.state != WCNC_STATE_RUN) return;
    mc.state = WCNC_STATE_HOLD;
    if (se_active() && !se_paused()) {
        mc.hold_ramp_ms = stop_time_ms();
        se_ramp_stop(mc.owner == OWN_LEGACY ? mc.hold_ramp_ms : stop_time_within_queue_ms(), false);
    }
    mc.starved = false;
}

void mc_feed_resume(void)
{
    if (mc.state != WCNC_STATE_HOLD) return;
    mc.state = WCNC_STATE_RUN;
    if (se_sync_lost() && !mc.last_queued) {
        /* (held in a run played by the spindle, and resumed before it had
         * even stopped: the run is not played on) */
        se_give_up(mc.hold_ramp_ms ? mc.hold_ramp_ms : 200);
        return;
    }
    se_ramp_resume(mc.hold_ramp_ms ? mc.hold_ramp_ms : 200);
}

bool mc_home(uint8_t axis_mask)
{
    double seek[WCNC_MAX_AXES], latch[WCNC_MAX_AXES];
    uint32_t pull[WCNC_MAX_AXES];
    uint8_t sw[WCNC_MAX_AXES];
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        seek[i] = (double)mc.cfg.homing_seek_rate;
        latch[i] = (double)mc.cfg.homing_feed_rate;
        pull[i] = mc.cfg.homing_pulloff;
        sw[i] = (uint8_t)i;
    }
    return start_homing(axis_mask, mc.cfg.homing_dir_mask, seek, latch, pull, sw);
}

void mc_abort_motion(void)
{
    if (in_fault()) return;
    se_halt();
    se_gate(0, NULL);
    generator_reset();
    mc.leg.active = false;
    mc_hal_legacy_clear();
    mc.owner = OWN_NONE;
    stream_idle();
    mc.state = WCNC_STATE_IDLE;
}

/* ===================================================================
 * Datagrams
 * =================================================================== */

static void on_discover(uint32_t ip, uint16_t port)
{
    wcnc_discover_resp_t r;
    memset(&r, 0, sizeof(r));
    r.firmware_version = WCNC_FIRMWARE_VERSION;
    mc_hal_identity(&r.capabilities, r.mac, r.device_name);
    r.device_mode = mc.cfg.io_module ? 1 : 0;
    r.host_connected = mc_host_present() ? 1 : 0;
    r.num_axes = WCNC_MAX_AXES;
    send_packet(&r, WCNC_PKT_DISCOVER_RESP, sizeof(r), ip, port);
}

void mc_on_udp(const uint8_t *data, size_t len, uint32_t ip, uint16_t port)
{
    if (len < sizeof(wcnc_header_t)) return;
    wcnc_header_t hdr;
    memcpy(&hdr, data, sizeof(hdr));
    if (hdr.magic != WCNC_MAGIC || hdr.version != WCNC_PROTOCOL_VERSION) return;
    size_t total = sizeof(hdr) + hdr.payload_length;
    if (len < total) return;
    {
        /* Checksum is over the packet with its checksum field zero */
        uint8_t head[sizeof(wcnc_header_t)];
        memcpy(head, data, sizeof(head));
        head[offsetof(wcnc_header_t, checksum)] = 0;
        head[offsetof(wcnc_header_t, checksum) + 1] = 0;
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < total; i++) {
            crc ^= (uint16_t)((i < sizeof(head) ? head[i] : data[i]) << 8);
            for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
        if (crc != hdr.checksum) return;
    }
    const uint8_t *payload = data + sizeof(hdr);
    size_t plen = hdr.payload_length;

    if (hdr.packet_type == WCNC_PKT_DISCOVER) {
        on_discover(ip, port);                          /* anyone may ask who is there */
        return;
    }

    /* One host at a time: the owner of the TCP session if there is one,
     * otherwise the first sender until it has been quiet for a while. */
    uint32_t now = mc_hal_ms();
    if (mc.tcp_ip) {
        if (ip != mc.tcp_ip) return;
        mc.host_ip = ip;
    } else if (!mc.host_ip || (uint32_t)(now - mc.last_rx_ms) > HOST_QUIET_MS) {
        mc.host_ip = ip;
        mc.status_port = WCNC_UDP_STATUS_PORT;
    } else if (ip != mc.host_ip) {
        return;
    }
    mc.last_rx_ms = now;

    if (hdr.packet_type == WCNC_PKT_IO_CONTROL) {
        on_io(payload, plen, port);
        return;
    }
    if (mc.cfg.io_module) return;                       /* an I/O module has no motion */

    switch (hdr.packet_type) {
    case WCNC_PKT_MOTION_SEGMENT: on_motion(payload, plen); break;
    case WCNC_PKT_JOG_COMMAND:    on_jog(payload, plen); break;
    case WCNC_PKT_JOG_STOP:       on_jog_stop(payload, plen); break;
    case WCNC_PKT_ESTOP:          mc_estop(); break;
    case WCNC_PKT_FEED_HOLD:      mc_feed_hold(); break;
    case WCNC_PKT_FEED_RESUME:    mc_feed_resume(); break;
    case WCNC_PKT_RESET:          mc_reset(); break;
    case WCNC_PKT_HOME_COMMAND:   on_home(payload, plen); break;
    case WCNC_PKT_STREAM_FLUSH:   on_flush(payload, plen); break;
    default: break;
    }
}

void mc_tcp_session(uint32_t ip)
{
    if (ip) {
        mc.tcp_ip = ip;
        mc.host_ip = ip;
        mc.status_port = WCNC_UDP_STATUS_PORT;
        mc.last_rx_ms = mc_hal_ms();
        mc.uses_watchdog = false;
        mc.watchdog_armed = false;
    } else {
        mc.tcp_ip = 0;
        if (!mc.uses_watchdog && !in_fault()) {
            /* A protocol 1.1 host has no watchdog: losing its connection is
             * the only sign that it has gone. */
            outputs_off();
        }
    }
}

/* ===================================================================
 * Status report
 * =================================================================== */

bool mc_host_present(void)
{
    if (!mc.host_ip) return false;
    return mc.tcp_ip != 0 || (uint32_t)(mc_hal_ms() - mc.last_rx_ms) <= HOST_QUIET_MS;
}

uint32_t mc_host_silence_ms(void)
{
    return mc.host_ip ? (uint32_t)(mc_hal_ms() - mc.last_rx_ms) : 0xFFFFFFFFu;
}

static void send_status(void)
{

    wcnc_status_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    wcnc_status_report_t *r = &pkt.report;

    int32_t pos[WCNC_MAX_AXES];
    se_get_position(pos);
    memcpy(r->position_steps, pos, sizeof(pos));
    r->buffer_available = se_free();
    r->buffer_total = SE_QUEUE_SIZE - 1;
    {
        /* A host older than protocol 1.2 paces itself by this pair, and what
         * it sends waits in the queue of speed-described segments - only a
         * moment of it is ever in the step engine's.  While that queue is
         * the one in use, it is the one shown. */
        uint16_t leg_free, leg_total;
        mc_hal_legacy_space(&leg_free, &leg_total);
        if (mc.owner == OWN_LEGACY || leg_free != leg_total) {
            r->buffer_available = leg_free;
            r->buffer_total = leg_total;
        }
    }
    r->state = (uint8_t)mc.state;
    r->alarm_code = mc.alarm;
    r->homing_state = mc.state == WCNC_STATE_HOMING ? mc.home.mask : 0;
    if (se_active() && !se_paused()) r->flags |= WCNC_STATUS_MOTION_ACTIVE;
    if (mc.state == WCNC_STATE_HOMING) r->flags |= WCNC_STATUS_HOMING_ACTIVE;
    if (mc.tcp_ip) r->flags |= WCNC_STATUS_HOST_CONNECTED;
    r->current_segment_id = se_current_id();
    r->feed_rate = (int32_t)se_feed_rate();
    r->misc_outputs = mc.out.misc;
    r->spindle_state = mc.out.spindle;
    r->coolant_state = mc.out.coolant;
    r->io_outputs = mc.out.io;
    r->next_segment_id = mc.next_seg;
    r->stream_id = mc.stream_id;
    r->stream_flags = mc.stream_flags;
    if (mc.owner == OWN_STREAM) {
        int sync = se_sync_state();
        if (sync == 1) r->stream_flags |= WCNC_STREAM_WAIT_INDEX;
        else if (sync == 2) r->stream_flags |= WCNC_STREAM_SPINDLE_SYNC;
    }
    r->homed_mask = mc.homed_mask;
    if (mc.state == WCNC_STATE_JOG) {
        for (int i = 0; i < WCNC_MAX_AXES; i++)
            if (mc.v[i] != 0.0 || mc.target[i] != 0.0) r->jog_mask |= (uint8_t)(1u << i);
    }
    uint32_t queued_ms = se_queued_us() / 1000u;
    r->queued_ms = queued_ms > 65535u ? 65535u : (uint16_t)queued_ms;

    mc_hal_fill_status(r);
    send_packet(&pkt, WCNC_PKT_STATUS_REPORT, sizeof(pkt), mc.host_ip, mc.status_port);
}

/* ===================================================================
 * Tick
 * =================================================================== */

void mc_init(void)
{
    memset(&mc, 0, sizeof(mc));
    mc.state = WCNC_STATE_IDLE;
    mc.pending_stream = -1;
    mc.status_port = WCNC_UDP_STATUS_PORT;
    mc.leg.queue_empty = true;
    mc_hal_load_config(&mc.cfg);
    mc.last_motion_ms = mc_hal_ms();
    mc_hal_apply_outputs(&mc.out);
    mc_hal_drivers(false);
}

void mc_config_changed(void)
{
    mc_hal_load_config(&mc.cfg);
}

void mc_tick(void)
{
    uint32_t now = mc_hal_ms();

    if (!mc.cfg.io_module) {
        /* E-stop input */
        if (mc_hal_estop_input() && mc.state != WCNC_STATE_ESTOP)
            enter_fault(WCNC_STATE_ESTOP, WCNC_ALARM_ESTOP_ACTIVE);

        tick_stream();
        tick_jog();
        tick_home();
        tick_legacy();

        /* Limit switches: an operated switch stops anything but homing */
        if ((mc.state == WCNC_STATE_RUN || mc.state == WCNC_STATE_JOG) && se_active() && !se_paused() &&
            !(mc.io_flags & WCNC_IO_FLAG_LIMIT_OVERRIDE)) {
            uint8_t lim = mc_hal_limit_inputs() & mc.cfg.limit_enable_mask;
            if (lim) {
                int axis = 0;
                while (!(lim & (1u << axis))) axis++;
                enter_fault(WCNC_STATE_ALARM, (uint8_t)(WCNC_ALARM_LIMIT_X + axis));
            }
        }

        /* Release the motor drivers after standing idle */
        if (se_active()) {
            mc.last_motion_ms = now;
        } else if (mc.drivers_on && mc.cfg.idle_release_ms && mc.state == WCNC_STATE_IDLE &&
                   (uint32_t)(now - mc.last_motion_ms) > mc.cfg.idle_release_ms) {
            drivers(false);
        }
    }

    /* Host watchdog */
    if (mc.watchdog_armed && (uint32_t)(now - mc.last_rx_ms) > WCNC_HOST_WATCHDOG_MS) {
        mc.watchdog_armed = false;
        bool active = se_active() || se_count() || mc.out.spindle || mc.out.misc || mc.out.charge_pump ||
                      mc.out.io || mc.out.coolant ||
                      mc.state == WCNC_STATE_HOMING || mc.state == WCNC_STATE_HOLD || mc.state == WCNC_STATE_JOG;
        if (mc.cfg.io_module) {
            outputs_off();
        } else if (active && !in_fault()) {
            enter_fault(WCNC_STATE_ALARM, WCNC_ALARM_HOST_LOST);
        }
    }

    /* Status report */
    uint32_t interval = mc.cfg.status_interval_ms ? mc.cfg.status_interval_ms : 50;
    if ((uint32_t)(now - mc.last_status_ms) >= interval) {
        mc.last_status_ms = now;
        if (mc_host_present()) send_status();
    }
}

/* ===================================================================
 * State
 * =================================================================== */

wcnc_machine_state_t mc_state(void) { return mc.state; }
uint8_t mc_alarm(void)              { return mc.alarm; }
uint8_t mc_homing_mask(void)        { return mc.state == WCNC_STATE_HOMING ? mc.home.mask : 0; }
bool    mc_probe_latched(void)      { return mc.probe_latched; }
void    mc_get_outputs(mc_outputs_t *out) { *out = mc.out; }

bool mc_busy(void)
{
    return se_active() || se_count() > 0 || mc.leg.active || mc.owner != OWN_NONE ||
           mc.state == WCNC_STATE_JOG || mc.state == WCNC_STATE_HOMING || mc.state == WCNC_STATE_HOLD;
}
