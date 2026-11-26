/*
 * PC-side unit test for the timed segment playback core.
 *
 * Build (any C compiler), from this folder:
 *   cc -o test_timed_exec test_timed_exec.c ../main/motion/timed_exec.c
 * A non-zero exit code means a failure.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../main/motion/timed_exec.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { if (g_fail++ < 20) { printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } } while (0)

static unsigned g_seed = 12345;
static unsigned rnd(void) { g_seed = g_seed * 1103515245u + 12345u; return (g_seed >> 8) & 0xFFFFFF; }

/* Reference model: where each axis' accumulator (1/256 step) stands. */
static long long g_acc[WCNC_MAX_AXES];
static long long g_pos[WCNC_MAX_AXES];      /* steps actually emitted */
static long long g_last_step_us[WCNC_MAX_AXES];
static long long g_min_gap[WCNC_MAX_AXES], g_max_gap[WCNC_MAX_AXES];
static long long g_clock;                   /* absolute time, us */

static long long floor_div(long long a, long long b) { long long q = a / b; if ((a % b) && ((a < 0) != (b < 0))) q--; return q; }

static void model_reset(texec_t *t)
{
    texec_reset(t);
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        g_acc[i] = WCNC_SUBSTEP_SCALE / 2;
        g_pos[i] = 0;
        g_last_step_us[i] = -1;
        g_min_gap[i] = 1LL << 60;
        g_max_gap[i] = 0;
    }
    g_clock = 0;
}

/* Play one segment through the executor; returns the time it occupied. */
static uint32_t play(texec_t *t, const wcnc_motion_segment_t *seg, uint32_t min_iv, int *ok)
{
    *ok = texec_load(t, seg, min_iv);
    if (!*ok) return 0;
    int scale = (seg->flags & WCNC_SEG_FLAG_SUBSTEP) ? 1 : WCNC_SUBSTEP_SCALE;
    for (int i = 0; i < WCNC_MAX_AXES; i++) g_acc[i] += (long long)seg->steps[i] * scale;

    uint32_t prev = 0;
    for (;;) {
        uint8_t mask;
        uint32_t when = texec_next(t, &mask, 2);
        CHECK(when >= prev, "event time went backwards: %u after %u", when, prev);
        CHECK(when <= texec_duration(t), "event at %u beyond the segment end %u", when, texec_duration(t));
        prev = when;
        if (!mask) break;
        for (int i = 0; i < WCNC_MAX_AXES; i++) {
            if (!(mask & (1u << i))) continue;
            g_pos[i] += t->dir[i];
            long long now = g_clock + when;
            if (g_last_step_us[i] >= 0) {
                long long gap = now - g_last_step_us[i];
                if (gap < g_min_gap[i]) g_min_gap[i] = gap;
                if (gap > g_max_gap[i]) g_max_gap[i] = gap;
            }
            g_last_step_us[i] = now;
        }
    }
    g_clock += texec_duration(t);
    return texec_duration(t);
}

static void check_positions(const char *what, int iter)
{
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        long long want = floor_div(g_acc[i], WCNC_SUBSTEP_SCALE);
        CHECK(g_pos[i] == want, "%s %d axis %d: %lld steps emitted, model says %lld", what, iter, i, g_pos[i], want);
    }
}

int main(void)
{
    texec_t t;
    wcnc_motion_segment_t s;
    int ok;

    /* 1. Random sub-step segments in both directions: the steps emitted always
     *    equal the whole part of the accumulated position, and every segment
     *    occupies exactly its duration. */
    model_reset(&t);
    for (int iter = 0; iter < 300000; iter++) {
        memset(&s, 0, sizeof(s));
        for (int i = 0; i < WCNC_MAX_AXES; i++) {
            int r = (int)(rnd() % 8);
            int32_t v = (r < 2) ? 0 : (int32_t)(rnd() % (r < 5 ? 600 : 60000));
            s.steps[i] = (rnd() & 1) ? v : -v;
        }
        s.duration_us = 100 + rnd() % 50000;
        s.flags = WCNC_SEG_FLAG_TIMED | WCNC_SEG_FLAG_SUBSTEP;
        uint32_t most = 0;
        for (int i = 0; i < WCNC_MAX_AXES; i++) { uint32_t n = (uint32_t)(abs(s.steps[i]) / 256 + 1); if (n > most) most = n; }
        uint32_t took = play(&t, &s, 10, &ok);
        CHECK(ok, "iter %d: segment refused", iter);
        CHECK(took >= s.duration_us && took <= (s.duration_us > most * 10 ? s.duration_us : most * 10),
              "iter %d: duration %u for requested %u", iter, took, s.duration_us);
        check_positions("random", iter);
    }

    /* 2. Whole-step segments (no SUBSTEP flag) behave the same way. */
    model_reset(&t);
    for (int iter = 0; iter < 100000; iter++) {
        memset(&s, 0, sizeof(s));
        for (int i = 0; i < WCNC_MAX_AXES; i++) { int32_t v = (int32_t)(rnd() % 40); s.steps[i] = (rnd() & 1) ? v : -v; }
        s.duration_us = 1000 + rnd() % 9000;
        s.flags = WCNC_SEG_FLAG_TIMED;
        play(&t, &s, 10, &ok);
        check_positions("whole-step", iter);
    }

    /* 3. Constant velocity cut into slices: pulses stay evenly spaced across
     *    slice boundaries, at speeds that are not a whole number of steps per
     *    slice.  X: 333.3 steps/s, Y: 2100 steps/s (negative), Z: 7 steps/s. */
    model_reset(&t);
    {
        double rate[3] = { 333.3333333, -2100.0, 7.0 };     /* steps per second */
        double carry[3] = { 0, 0, 0 };
        for (int slice = 0; slice < 4000; slice++) {         /* 20 s in 5 ms slices */
            memset(&s, 0, sizeof(s));
            for (int i = 0; i < 3; i++) {
                double exact = rate[i] * 0.005 * 256.0 + carry[i];
                int32_t q = (int32_t)(exact >= 0 ? exact + 0.5 : exact - 0.5);
                carry[i] = exact - q;
                s.steps[i] = q;
            }
            s.duration_us = 5000;
            s.flags = WCNC_SEG_FLAG_TIMED | WCNC_SEG_FLAG_SUBSTEP;
            play(&t, &s, 10, &ok);
        }
        check_positions("constant velocity", 0);
        CHECK(g_clock == 20000000LL, "20 s of slices took %lld us", g_clock);
        double ideal[3] = { 1e6 / 333.3333333, 1e6 / 2100.0, 1e6 / 7.0 };
        for (int i = 0; i < 3; i++) {
            /* 1/256-step velocity resolution allows a little wander; a slice
             * boundary must not show up as a gap or a bunch. */
            double tol = ideal[i] * 0.02 + 3.0;
            CHECK(g_min_gap[i] >= ideal[i] - tol && g_max_gap[i] <= ideal[i] + tol,
                  "axis %d pulse spacing %lld..%lld us, ideal %.1f", i, g_min_gap[i], g_max_gap[i], ideal[i]);
        }
        CHECK(llabs(g_pos[0] - 6667) <= 1 && llabs(g_pos[1] + 42000) <= 1 && llabs(g_pos[2] - 140) <= 1,
              "distance X %lld Y %lld Z %lld", g_pos[0], g_pos[1], g_pos[2]);
    }

    /* 4. A reversal exactly at a step boundary neither loses nor adds a step. */
    model_reset(&t);
    for (int iter = 0; iter < 20000; iter++) {
        memset(&s, 0, sizeof(s));
        int32_t v = (int32_t)(1 + rnd() % 700);
        s.steps[0] = (iter & 1) ? v : -v;
        s.steps[1] = (iter & 1) ? -128 : 128;                /* back and forth over a boundary */
        s.duration_us = 2000;
        s.flags = WCNC_SEG_FLAG_TIMED | WCNC_SEG_FLAG_SUBSTEP;
        play(&t, &s, 10, &ok);
        check_positions("reversal", iter);
    }

    /* 5. Dwell: no steps, exact duration, fractions untouched. */
    model_reset(&t);
    memset(&s, 0, sizeof(s));
    s.duration_us = 2500000; s.flags = WCNC_SEG_FLAG_TIMED;
    CHECK(play(&t, &s, 10, &ok) == 2500000 && ok, "dwell duration");
    check_positions("dwell", 0);

    /* 6. A segment too fast for the hardware is stretched, never truncated. */
    model_reset(&t);
    memset(&s, 0, sizeof(s));
    s.steps[2] = 1000; s.duration_us = 100; s.flags = WCNC_SEG_FLAG_TIMED;
    {
        uint32_t took = play(&t, &s, 12, &ok);
        CHECK(ok && took == 12000 && g_pos[2] == 1000, "stretch: %u us, %lld steps", took, g_pos[2]);
        CHECK(g_min_gap[2] >= 11, "stretch: pulses %lld us apart", g_min_gap[2]);
    }

    /* 7. Out-of-range requests are refused. */
    memset(&s, 0, sizeof(s));
    s.steps[0] = 0x00100000; s.duration_us = 1000; s.flags = WCNC_SEG_FLAG_TIMED;
    CHECK(!texec_load(&t, &s, 10), "oversize whole-step segment accepted");
    s.steps[0] = (int32_t)0x80000000u; s.flags = WCNC_SEG_FLAG_TIMED | WCNC_SEG_FLAG_SUBSTEP;
    CHECK(!texec_load(&t, &s, 10), "INT32_MIN segment accepted");
    s.steps[0] = 5; s.duration_us = 20000000; s.flags = WCNC_SEG_FLAG_TIMED;
    CHECK(!texec_load(&t, &s, 10), "over-long moving segment accepted");

    /* 8. Largest allowed moving segment does not overflow. */
    model_reset(&t);
    memset(&s, 0, sizeof(s));
    s.steps[0] = 1000000; s.steps[5] = -999999; s.duration_us = TEXEC_MAX_MOVE_US; s.flags = WCNC_SEG_FLAG_TIMED;
    play(&t, &s, 10, &ok);
    CHECK(ok, "largest segment refused");
    check_positions("largest", 0);

    printf(g_fail ? "%d FAILURE(S)\n" : "timed_exec: all tests passed\n", g_fail);
    return g_fail ? 1 : 0;
}
