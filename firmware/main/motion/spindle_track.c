/*
 * WiFi CNC Controller - Spindle tracking (hardware independent)
 *
 * See spindle_track.h.
 */

#include "spindle_track.h"

#include <string.h>

#ifdef ESP_PLATFORM
#include "esp_attr.h"
#else
#define IRAM_ATTR
#define DRAM_ATTR
#endif

/* --- Quadrature ---------------------------------------------------------
 * The count is followed by an alpha-beta filter: each sample moves the
 * angle estimate 1/8 of the way to what the count says, and adjusts the
 * speed estimate in proportion.  The gains are on the lively side of
 * critical damping: the filter settles in about 16 ms and follows a spindle
 * slowing by 10 % in a tenth of a second to within a count, at the price of
 * overshooting a single count by about a fifth of it - which costs nothing,
 * as the axes are never taken back. */
#define ALPHA_SHIFT         3
#define BETA_Q16            ((int64_t)(((uint64_t)1 << 32) / (128u * SPT_SAMPLE_US)))  /* (1/128) / sample time */
#define EXTRAPOLATE_US      2000            /* furthest the angle is carried from the last sample */
#define RESYNC_US           20000u          /* samples this far apart: start again */
#define ERROR_LIMIT         (SPT_REV / 4)
#define SLOW_SHIFT          7               /* display speed: about 64 ms of smoothing */
#define TURNING             9382499         /* 2 rpm: slower than this says nothing about direction */

/* Index pulse of a quadrature encoder */
#define PHASE_NEAR          ((int32_t)1 << 26)  /* 1/64 revolution: "the same place" (or a count and a
                                                   half, if the encoder is that coarse) */
#define INDEX_APART         (SPT_REV / 8)       /* pulses closer together than this are one pulse */

/* --- Pulses ------------------------------------------------------------- */
#define PULSE_MIN_US        50u             /* closer together than this is bounce */
#define PERIOD_MAX_US       10000000u       /* slower than this counts as standing still */

static DRAM_ATTR struct {
    uint8_t  mode;
    bool     index_input;
    uint32_t per_rev;
    int64_t  unit;                  /* angle of one count or pulse */

    /* where the sensor says the spindle is: rev whole turns and rem counts */
    int64_t  rev;
    uint32_t rem;

    /* estimate: angle(t) = angle + speed * (t - at) / 2^16 */
    int64_t  angle, speed;
    uint64_t at;
    bool     tracking;              /* quadrature: the filter is running */
    int64_t  slow;                  /* quadrature: speed, smoothed */
    int8_t   dir;

    /* quadrature: index */
    uint32_t phase;                 /* where in the revolution the index is */
    uint32_t other;                 /* an index pulse seen elsewhere: believed when seen there twice */
    int32_t  same;                  /* how close counts as the same place */
    bool     have_phase, have_other, index_seen;
    int64_t  index_angle;           /* at the last pulse accepted (without an index input: the
                                       turn of the count that was last counted as one) */

    /* pulses */
    uint64_t last_pulse;
    uint32_t period;                /* between the last two pulses */
    int64_t  reach;                 /* how long after a pulse the estimate is carried on */
    bool     have_pulse, locked;    /* locked: the speed is known */
    uint64_t rpm_scale;
    uint32_t rpm;

    uint32_t index_count;
    uint32_t epoch;
} s;

void spt_setup(spt_mode_t mode, uint32_t per_rev, bool index_input)
{
    memset(&s, 0, sizeof(s));
    s.dir = 1;
    if (mode == SPT_NONE || per_rev == 0) return;
    s.per_rev = per_rev;
    s.unit = SPT_REV / (int64_t)per_rev;
    s.index_input = index_input && mode == SPT_QUADRATURE;
    int64_t same = s.unit + s.unit / 2;
    if (same < PHASE_NEAR) same = PHASE_NEAR;
    if (same > INDEX_APART) same = INDEX_APART;
    s.same = (int32_t)same;
    s.rpm_scale = 60000000000ull / per_rev;
    s.mode = (uint8_t)mode;
}

bool spt_ready(void) { return s.mode != SPT_NONE; }

bool spt_can_sync(void)
{
    return s.mode == SPT_QUADRATURE || (s.mode == SPT_PULSES && s.per_rev == 1);
}

/* ===================================================================
 * Quadrature
 * =================================================================== */

void IRAM_ATTR spt_counts(uint64_t now, int32_t delta)
{
    if (s.mode != SPT_QUADRATURE) return;

    int64_t rem = (int64_t)s.rem + delta, per = (int64_t)s.per_rev;
    if (rem >= per || rem < 0) {
        int64_t turns = rem / per;
        rem -= turns * per;
        if (rem < 0) {
            rem += per;
            turns--;
        }
        s.rev += turns;
        if (!s.index_input) {
            /* The count coming round stands for the index pulse.  A spindle
             * rocking to and fro across that place is not making turns: the
             * same crossing is counted once. */
            int64_t crossed = turns > 0 ? s.rev : s.rev + 1;
            if (!s.index_seen || crossed != s.index_angle) {
                s.index_count += (uint32_t)(turns < 0 ? -turns : turns);
                s.index_angle = crossed;
                s.index_seen = true;
            }
        }
    }
    s.rem = (uint32_t)rem;
    int64_t measured = s.rev * SPT_REV + rem * s.unit;

    int64_t dt = (int64_t)(now - s.at);
    if (!s.tracking || dt > (int64_t)RESYNC_US) {
        s.angle = measured;
        s.speed = 0;
        s.slow = 0;
        s.at = now;
        s.tracking = true;
        return;
    }
    if (dt <= 0) return;

    int64_t predicted = s.angle + ((s.speed * dt) >> 16);
    int64_t error = measured - predicted;
    if (error > ERROR_LIMIT) error = ERROR_LIMIT;
    if (error < -ERROR_LIMIT) error = -ERROR_LIMIT;
    s.angle = predicted + (error >> ALPHA_SHIFT);
    s.speed += (error * BETA_Q16) >> 16;
    s.at = now;

    s.slow += (s.speed - s.slow) >> SLOW_SHIFT;
    if (s.slow > TURNING) s.dir = 1;
    else if (s.slow < -TURNING) s.dir = -1;
}

void IRAM_ATTR spt_index(uint64_t now)
{
    if (s.mode != SPT_QUADRATURE || !s.index_input || !s.tracking) return;

    int64_t here = spt_angle(now);
    int64_t moved = here - s.index_angle;
    if (s.index_seen && moved < INDEX_APART && moved > -INDEX_APART) return;    /* bounce or noise */
    s.index_angle = here;
    s.index_seen = true;
    s.index_count++;

    /* Where in the revolution was that?  Pulses that agree with what is
     * known refine it a little each; one that does not is taken for noise
     * unless the next one says the same (the count has slipped, or the
     * spindle now turns the other way and meets the other edge). */
    uint32_t seen = (uint32_t)(uint64_t)here;
    int32_t off = (int32_t)(seen - s.phase);
    if (s.have_phase && off > -s.same && off < s.same) {
        s.phase += (uint32_t)(off >> 3);
        s.have_other = false;
    } else {
        int32_t apart = (int32_t)(seen - s.other);
        if (s.have_other && apart > -s.same && apart < s.same) {
            s.phase = seen;
            s.have_phase = true;
            s.have_other = false;
            s.epoch++;
        } else {
            s.other = seen;
            s.have_other = true;
        }
    }
}

/* ===================================================================
 * Pulses
 * =================================================================== */

void IRAM_ATTR spt_pulse(uint64_t now)
{
    if (s.mode != SPT_PULSES) return;
    int64_t dt = (int64_t)(now - s.last_pulse);
    if (s.have_pulse && (dt < (int64_t)PULSE_MIN_US || (s.locked && dt < (int64_t)(s.period / 4)))) return;  /* bounce */

    /* This is where the spindle really is */
    if (++s.rem >= s.per_rev) {
        s.rem = 0;
        s.rev++;
        s.index_count++;
    }
    int64_t real = s.rev * SPT_REV + (int64_t)s.rem * s.unit;
    int64_t estimate = spt_angle(now);

    if (!s.have_pulse || dt > (int64_t)PERIOD_MAX_US) {
        /* The first pulse after standing still says where, not how fast */
        s.speed = 0;
        s.locked = false;
        s.rpm = 0;
    } else {
        int64_t error = 0;
        if (!s.locked) {
            estimate = real;                /* turning again: start from the truth */
            s.epoch++;
        } else {
            error = real - estimate;
            if (error > s.unit / 2) error = s.unit / 2;
            if (error < -s.unit / 2) error = -s.unit / 2;
        }
        /* Carry on at the speed of the period just measured, adjusted so
         * that whatever the estimate is out by has gone by the next pulse.
         * (Guessing from two periods how the speed is changing was tried:
         * it follows a long steady change better, and the short sag of a
         * tool entering the cut worse - it overshoots when the sag ends.) */
        s.speed = ((s.unit + error) << 16) / dt;
        s.period = (uint32_t)dt;
        /* ...but no further than half a pulse beyond where the next one is
         * due: if it is that late the spindle is slowing, and running on
         * would only have to be taken back. */
        s.reach = dt + dt / 2;
        uint32_t rpm = (uint32_t)(s.rpm_scale / (uint64_t)dt);
        if (!s.locked || s.per_rev == 1) s.rpm = rpm;
        else s.rpm = (uint32_t)((int64_t)s.rpm + ((int64_t)rpm - (int64_t)s.rpm) / 4);
        s.locked = true;
    }
    s.angle = estimate;
    s.at = now;
    s.last_pulse = now;
    s.have_pulse = true;
}

void IRAM_ATTR spt_poll(uint64_t now)
{
    if (s.mode != SPT_PULSES || !s.locked) return;
    if ((int64_t)(now - s.last_pulse) > 2 * (int64_t)s.period + 2000) {
        /* Overdue by a whole period: stopped, or stopping */
        s.angle = spt_angle(now);
        s.at = now;
        s.speed = 0;
        s.rpm = 0;
        s.locked = false;
        s.have_pulse = false;
    }
}

/* ===================================================================
 * Reading
 * =================================================================== */

int64_t IRAM_ATTR spt_angle(uint64_t now)
{
    int64_t dt = (int64_t)(now - s.at);
    if (s.mode == SPT_QUADRATURE) {
        if (!s.tracking) return 0;
        if (dt > EXTRAPOLATE_US) dt = EXTRAPOLATE_US;
        if (dt < -EXTRAPOLATE_US) dt = -EXTRAPOLATE_US;
        return s.angle + ((s.speed * dt) >> 16);
    }
    if (s.mode == SPT_PULSES) {
        if (!s.locked || dt <= 0) return s.angle;
        if (dt > s.reach) dt = s.reach;
        return s.angle + ((s.speed * dt) >> 16);
    }
    return 0;
}

int64_t IRAM_ATTR spt_speed(uint64_t now)
{
    if (s.mode == SPT_QUADRATURE) return s.tracking ? s.speed : 0;
    if (s.mode == SPT_PULSES && s.locked) {
        /* (carried as far as it goes, the estimate stands still) */
        return (int64_t)(now - s.at) > s.reach ? 0 : s.speed;
    }
    return 0;
}

int IRAM_ATTR spt_direction(void) { return s.dir; }

int64_t IRAM_ATTR spt_next_index(int64_t angle, int direction)
{
    uint32_t past = (uint32_t)(uint64_t)angle - s.phase;    /* how far beyond the last index */
    if (direction >= 0) return angle - (int64_t)past + SPT_REV;
    return past ? angle - (int64_t)past : angle - SPT_REV;
}

uint32_t spt_rpm_milli(void)
{
    if (s.mode == SPT_PULSES) return s.rpm;
    if (s.mode != SPT_QUADRATURE || !s.tracking) return 0;
    int64_t v = s.slow < 0 ? -s.slow : s.slow;
    return (uint32_t)(((uint64_t)(v >> 12) * 60000000000ull) >> 36);
}

uint16_t spt_position16(uint64_t now)
{
    return (uint16_t)(((uint32_t)(uint64_t)spt_angle(now) - s.phase) >> 16);
}

uint32_t spt_index_count(void) { return s.index_count; }

uint32_t IRAM_ATTR spt_index_epoch(void) { return s.epoch; }

bool IRAM_ATTR spt_index_known(void)
{
    if (s.mode == SPT_QUADRATURE) return s.tracking && (s.have_phase || !s.index_input);
    return s.mode == SPT_PULSES && s.locked;
}

bool IRAM_ATTR spt_following(void)
{
    if (s.mode == SPT_QUADRATURE) return s.tracking;
    return s.mode == SPT_PULSES && s.locked;
}
