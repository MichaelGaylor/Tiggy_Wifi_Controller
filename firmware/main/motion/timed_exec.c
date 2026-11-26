/*
 * WiFi CNC Controller - Timed segment playback (hardware independent core)
 *
 * See timed_exec.h for the model.
 */

#include "timed_exec.h"

#ifdef ESP_PLATFORM
#include "esp_attr.h"
#else
#define IRAM_ATTR
#endif

void texec_reset(texec_t *t)
{
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        t->phase[i] = WCNC_SUBSTEP_SCALE / 2;
        t->left[i] = 0;
    }
    t->active = 0;
    t->duration = 0;
    t->flags = 0;
}

bool IRAM_ATTR texec_load(texec_t *t, const wcnc_motion_segment_t *seg, uint32_t min_interval_us)
{
    uint32_t amount[WCNC_MAX_AXES];     /* magnitude in 1/256 step */
    uint32_t count[WCNC_MAX_AXES];      /* whole steps this segment produces */
    uint8_t  mirror[WCNC_MAX_AXES];     /* accumulator seen in the direction of travel */
    uint32_t most = 0;
    bool substep = (seg->flags & WCNC_SEG_FLAG_SUBSTEP) != 0;

    if (min_interval_us == 0) min_interval_us = 1;

    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        int32_t s = seg->steps[i];
        uint32_t a = (s < 0) ? (uint32_t)(-(int64_t)s) : (uint32_t)s;
        if (!substep) {
            if (a > TEXEC_MAX_STEPS) return false;
            a *= WCNC_SUBSTEP_SCALE;
        } else if (a > TEXEC_MAX_STEPS * (uint32_t)WCNC_SUBSTEP_SCALE) {
            return false;
        }
        amount[i] = a;
        t->dir[i] = (s < 0) ? -1 : 1;
        /* Travelling down, the distance to the next whole step is measured
         * from the other end of the accumulator. */
        mirror[i] = (s < 0) ? (uint8_t)(255 - t->phase[i]) : t->phase[i];
        count[i] = (mirror[i] + a) >> 8;
        if (count[i] > most) most = count[i];
    }

    uint32_t d = seg->duration_us;
    if (d > TEXEC_MAX_DURATION_US) d = TEXEC_MAX_DURATION_US;
    if (d == 0) d = 1;
    if (most > 0) {
        /* Never ask for steps closer together than the hardware can make
         * them: stretch the segment instead of losing steps. */
        uint64_t need = (uint64_t)most * min_interval_us;
        if (need > TEXEC_MAX_MOVE_US) return false;
        if (d < (uint32_t)need) d = (uint32_t)need;
        if (d > TEXEC_MAX_MOVE_US) return false;
    }

    t->duration = d;
    t->flags = seg->flags;
    t->active = 0;

    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        uint32_t a = amount[i];
        t->left[i] = count[i];
        if (a == 0) continue;
        if (count[i] > 0) {
            /* The accumulator advances a/256 steps in d microseconds, so one
             * step takes 256*d/a microseconds; times are kept in 1/256 us.
             * The first step falls when the accumulator reaches 256. */
            uint64_t per = ((uint64_t)d << 16) / a;             /* 256 * (d in 1/256 us) / a */
            t->period[i] = per > 0xFFFFFFFFu ? 0xFFFFFFFFu : (uint32_t)per;
            t->next[i] = (uint32_t)(((uint64_t)(256u - mirror[i]) * ((uint64_t)d << 8)) / a);
            t->active |= (uint8_t)(1u << i);
        }
        uint8_t m = (uint8_t)((mirror[i] + a) & 0xFF);
        t->phase[i] = (seg->steps[i] < 0) ? (uint8_t)(255 - m) : m;
    }
    return true;
}

uint32_t IRAM_ATTR texec_next(texec_t *t, uint8_t *mask, uint32_t merge_us)
{
    uint8_t active = t->active;
    if (!active) {
        *mask = 0;
        return t->duration;             /* end of the segment */
    }

    /* Earliest pending step */
    uint32_t first = 0xFFFFFFFFu;
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        if ((active & (1u << i)) && t->next[i] < first) first = t->next[i];
    }

    /* Every axis due within the merge window steps with it */
    uint32_t limit = first + (merge_us << 8);
    uint8_t m = 0;
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        if (!(active & (1u << i)) || t->next[i] > limit) continue;
        m |= (uint8_t)(1u << i);
        if (--t->left[i] == 0) active &= (uint8_t)~(1u << i);
        else t->next[i] += t->period[i];
    }
    t->active = active;
    *mask = m;

    uint32_t us = first >> 8;
    return us < t->duration ? us : t->duration;
}
