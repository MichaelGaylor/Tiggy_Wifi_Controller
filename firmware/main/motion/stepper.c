/*
 * WiFi CNC Controller - Stepper Engine Implementation
 *
 * Uses ESP-IDF GPTimer for variable-rate interrupts. The ISR runs a
 * Bresenham multi-axis step distribution algorithm and writes GPIO
 * registers directly for sub-microsecond pin toggling.
 *
 * Architecture:
 *   - Single hardware timer fires at the current step interval
 *   - ISR sets direction pins, runs Bresenham, pulses step pins
 *   - Step pulse width is achieved via a short busy-wait in ISR
 *   - Trapezoidal velocity profile: accel -> cruise -> decel per segment
 */

#include "stepper.h"
#include "../config.h"
#include "../pin_config.h"
#include "../persist/nvs_config.h"

#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "hal/gpio_ll.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "rom/ets_sys.h"

static const char *TAG = "stepper";

/* ===================================================================
 * Stepper State (in DRAM for ISR access)
 * =================================================================== */

typedef struct {
    /* Bresenham state per axis */
    int32_t  counter[WCNC_MAX_AXES];
    uint32_t steps_remaining;
    uint32_t total_steps;            /* Longest axis (drives Bresenham) */

    /* Current segment per-axis data */
    uint32_t segment_steps[WCNC_MAX_AXES]; /* Absolute step count */
    int8_t   direction[WCNC_MAX_AXES];

    /* Step timing (in timer ticks = microseconds at 1MHz) */
    uint32_t current_interval;
    uint32_t nominal_interval;
    uint32_t initial_interval;
    uint32_t final_interval;

    /* Acceleration ramp boundaries */
    uint32_t accel_until_step;
    uint32_t decel_after_step;

    /* Kinematic acceleration tracking (integer-only, ISR-safe).
     * Tracks v² directly: v²(n+1) = v²(n) ± 2*a per step.
     * Uses bit-by-bit isqrt (no flash function calls). */
    uint32_t speed_sq;              /* Current v² in (steps/sec)² */
    uint32_t two_accel;             /* 2*a in steps/sec² — delta per step (trapezoidal) */
    uint32_t nominal_speed_sq;      /* Cruise v² */
    uint32_t exit_speed_sq;         /* Target exit v² */

    /* S-curve (jerk-limited) acceleration tracking.
     * In jerk phases, current_two_accel ramps linearly instead of
     * being constant. This smooths acceleration transitions. */
    int32_t  current_two_accel;     /* Current 2*a value (ramps during jerk phases) */
    int32_t  jerk_step_delta;       /* Change in current_two_accel per step */
    uint32_t two_accel_max;         /* Maximum 2*a (peak acceleration) */
    uint32_t phase_end[6];          /* 7-phase boundaries (step numbers) */
    /* [0]=accel-jerk-up end, [1]=const-accel end, [2]=accel-jerk-down end
       [3]=cruise end (=decel_after), [4]=decel-jerk-down end, [5]=const-decel end
       Phase 6 runs from [5] to total_steps */
    bool     scurve_active;         /* false = trapezoidal fallback */

    /* Step pulse width in microseconds */
    uint32_t step_pulse_us;

    /* Direction setup time in microseconds */
    uint32_t dir_setup_us;

    /* Per-axis limits (from NVS config) */
    uint32_t max_rate[WCNC_MAX_AXES];   /* steps/sec per axis */
    uint32_t max_accel[WCNC_MAX_AXES];  /* steps/sec^2 per axis */
    float    steps_per_mm[WCNC_MAX_AXES]; /* steps/mm per axis (for jerk conversion) */
    float    jerk_max_mm;               /* Max jerk in mm/sec³ ($40) */

    /* GPIO pin numbers for each axis */
    gpio_num_t step_pins[WCNC_MAX_AXES];
    gpio_num_t dir_pins[WCNC_MAX_AXES];
    gpio_num_t enable_pin;

    /* Inversion masks (from config) */
    uint8_t invert_step;
    uint8_t invert_dir;
    uint8_t invert_enable;

    /* Position tracking (absolute, in steps) */
    volatile int32_t position[WCNC_MAX_AXES];

    /* State flags */
    volatile bool segment_complete;
    volatile bool running;
    volatile bool estopped;
    volatile bool holding;
    volatile bool jog_mode;

    /* Feed rate tracking */
    volatile int32_t current_feed_rate;

    /* Probe detection (G38.x) */
    uint8_t seg_flags;                  /* WCNC_SEG_FLAG_* from current segment */
    volatile bool probe_triggered;      /* Set by ISR when probe pin activates */
    gpio_num_t probe_pin;               /* Cached probe pin for ISR */
    uint8_t invert_probe;               /* Probe inversion flag */

} stepper_state_t;

static DRAM_ATTR stepper_state_t st;
static portMUX_TYPE s_stepper_mux = portMUX_INITIALIZER_UNLOCKED;
static gptimer_handle_t step_timer = NULL;

/* ===================================================================
 * GPIO Helpers - Direct register writes for speed
 * =================================================================== */

static inline void IRAM_ATTR gpio_set_fast(gpio_num_t pin)
{
    gpio_ll_set_level(&GPIO, pin, 1);
}

static inline void IRAM_ATTR gpio_clear_fast(gpio_num_t pin)
{
    gpio_ll_set_level(&GPIO, pin, 0);
}

/* Check if a GPIO pin is usable on this board (not NC / not memory bus) */
static inline bool pin_valid_output(gpio_num_t pin)
{
    return (int)pin >= 0 && (int)pin < GPIO_NUM_MAX &&
           GPIO_IS_VALID_OUTPUT_GPIO(pin) &&
           !PIN_RESERVED_FOR_MEMORY_BUS(pin);
}

static inline bool pin_valid_input(gpio_num_t pin)
{
    return (int)pin >= 0 && (int)pin < GPIO_NUM_MAX &&
           GPIO_IS_VALID_GPIO(pin) &&
           !PIN_RESERVED_FOR_MEMORY_BUS(pin);
}

static inline void IRAM_ATTR set_step_pin(int axis, bool state)
{
    if (!pin_valid_output(st.step_pins[axis])) return;
    bool actual = state ^ ((st.invert_step >> axis) & 1);
    gpio_ll_set_level(&GPIO, st.step_pins[axis], actual ? 1 : 0);
}

static inline void IRAM_ATTR set_dir_pin(int axis, bool positive)
{
    if (!pin_valid_output(st.dir_pins[axis])) return;
    bool actual = positive ^ ((st.invert_dir >> axis) & 1);
    gpio_ll_set_level(&GPIO, st.dir_pins[axis], actual ? 1 : 0);
}

/* ===================================================================
 * Integer Square Root (bit-by-bit, IRAM-safe, ~16 iterations)
 * =================================================================== */

static uint32_t IRAM_ATTR isqrt32(uint32_t n)
{
    uint32_t result = 0;
    uint32_t bit = 1u << 30;
    while (bit > n) bit >>= 2;
    while (bit != 0) {
        if (n >= result + bit) {
            n -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    return result;
}

/* ===================================================================
 * Timer ISR - The Core Step Generation Engine
 * =================================================================== */

static bool IRAM_ATTR stepper_timer_isr(gptimer_handle_t timer,
                                         const gptimer_alarm_event_data_t *edata,
                                         void *user_ctx)
{
    if (st.estopped || st.holding) {
        return false;
    }

    /* Check if segment is complete */
    if (st.steps_remaining == 0) {
        st.segment_complete = true;
        st.running = false;
        st.current_feed_rate = 0;
        return true; /* Request context switch to wake motion_control_task */
    }

    /* Probe detection: check probe pin during probe segments */
    if ((st.seg_flags & WCNC_SEG_FLAG_PROBE) && pin_valid_input(st.probe_pin)) {
        bool pin_state = gpio_ll_get_level(&GPIO, st.probe_pin);
        if (pin_state ^ (st.invert_probe & 1)) {
            /* Probe triggered — stop immediately, preserve position */
            st.probe_triggered = true;
            st.segment_complete = true;
            st.running = false;
            st.steps_remaining = 0;
            st.current_feed_rate = 0;
            return true;
        }
    }

    /* Set direction pins (must be stable before step edge) */
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        if (st.segment_steps[i] > 0) {
            set_dir_pin(i, st.direction[i] > 0);
        }
    }

    /* Brief delay for direction setup time */
    if (st.dir_setup_us > 0) {
        ets_delay_us(st.dir_setup_us);
    }

    /* Bresenham multi-axis step distribution */
    bool any_stepped = false;
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        if (st.segment_steps[i] == 0) continue;

        st.counter[i] += (int32_t)st.segment_steps[i];
        if (st.counter[i] > 0) {
            st.counter[i] -= (int32_t)st.total_steps;
            set_step_pin(i, true);
            st.position[i] += st.direction[i];
            any_stepped = true;
        }
    }

    /* Hold step pulse for configured width */
    if (any_stepped) {
        ets_delay_us(st.step_pulse_us);

        /* Clear all step pins */
        for (int i = 0; i < WCNC_MAX_AXES; i++) {
            if (st.segment_steps[i] > 0) {
                set_step_pin(i, false);
            }
        }
    }

    st.steps_remaining--;

    /* Compute next step interval using exact kinematic v² tracking.
     * Integer-only math — safe for IRAM_ATTR ISR (no flash function calls).
     *
     * Physics: v²(n+1) = v²(n) + 2*a  (per step)
     * Trapezoidal: 2*a is constant.
     * S-curve: 2*a ramps up/down through 7 phases for smooth transitions. */
    uint32_t step_number = st.total_steps - st.steps_remaining;

    if (!st.scurve_active) {
        /* --- Trapezoidal fallback (constant acceleration) --- */
        if (step_number < st.accel_until_step) {
            st.speed_sq += st.two_accel;
            if (st.speed_sq > st.nominal_speed_sq) st.speed_sq = st.nominal_speed_sq;
        } else if (step_number >= st.decel_after_step) {
            if (st.speed_sq > st.exit_speed_sq + st.two_accel) {
                st.speed_sq -= st.two_accel;
            } else {
                st.speed_sq = st.exit_speed_sq;
            }
        } else {
            st.speed_sq = st.nominal_speed_sq;
        }
    } else {
        /* --- S-curve: 7-phase jerk-limited profile ---
         * Phases 0-2: acceleration (jerk-up, constant, jerk-down)
         * Phase 3:    cruise
         * Phases 4-6: deceleration (jerk-down, constant, jerk-up) */
        if (step_number < st.phase_end[0]) {
            /* Phase 0: Accel jerk-up (a ramps 0 → max) */
            st.current_two_accel += st.jerk_step_delta;
            if (st.current_two_accel > (int32_t)st.two_accel_max)
                st.current_two_accel = (int32_t)st.two_accel_max;
            st.speed_sq += (uint32_t)st.current_two_accel;
        } else if (step_number < st.phase_end[1]) {
            /* Phase 1: Constant acceleration */
            st.current_two_accel = (int32_t)st.two_accel_max;
            st.speed_sq += st.two_accel_max;
        } else if (step_number < st.phase_end[2]) {
            /* Phase 2: Accel jerk-down (a ramps max → 0) */
            st.current_two_accel -= st.jerk_step_delta;
            if (st.current_two_accel < 0) st.current_two_accel = 0;
            st.speed_sq += (uint32_t)st.current_two_accel;
        } else if (step_number < st.phase_end[3]) {
            /* Phase 3: Cruise */
            st.speed_sq = st.nominal_speed_sq;
        } else if (step_number < st.phase_end[4]) {
            /* Phase 4: Decel jerk-down (a ramps 0 → -max) */
            st.current_two_accel -= st.jerk_step_delta;
            if (st.current_two_accel < -(int32_t)st.two_accel_max)
                st.current_two_accel = -(int32_t)st.two_accel_max;
            uint32_t dec = (uint32_t)(-st.current_two_accel);
            st.speed_sq = (st.speed_sq > dec + st.exit_speed_sq)
                          ? st.speed_sq - dec : st.exit_speed_sq;
        } else if (step_number < st.phase_end[5]) {
            /* Phase 5: Constant deceleration */
            st.current_two_accel = -(int32_t)st.two_accel_max;
            st.speed_sq = (st.speed_sq > st.two_accel_max + st.exit_speed_sq)
                          ? st.speed_sq - st.two_accel_max : st.exit_speed_sq;
        } else {
            /* Phase 6: Decel jerk-up (a ramps -max → 0) */
            st.current_two_accel += st.jerk_step_delta;
            if (st.current_two_accel > 0) st.current_two_accel = 0;
            uint32_t dec = (uint32_t)(-st.current_two_accel);
            st.speed_sq = (st.speed_sq > dec + st.exit_speed_sq)
                          ? st.speed_sq - dec : st.exit_speed_sq;
        }
        /* Global safety clamps */
        if (st.speed_sq > st.nominal_speed_sq) st.speed_sq = st.nominal_speed_sq;
        if (st.speed_sq < st.exit_speed_sq)    st.speed_sq = st.exit_speed_sq;
    }

    /* Convert v² to timer interval: interval = f_timer / sqrt(v²) */
    uint32_t speed = isqrt32(st.speed_sq);
    uint32_t next_interval;
    if (speed > 1) {
        next_interval = CFG_STEPPER_TIMER_RESOLUTION_HZ / speed;
    } else {
        next_interval = 1000000;
    }

    /* Clamp interval to sane range */
    if (next_interval < 4) next_interval = 4;              /* 250kHz max */
    if (next_interval > 1000000) next_interval = 1000000;  /* 1Hz min */

    st.current_interval = next_interval;

    /* Update feed rate (inverse of interval, in steps/sec) */
    if (next_interval > 0) {
        st.current_feed_rate = (int32_t)(CFG_STEPPER_TIMER_RESOLUTION_HZ / next_interval);
    }

    /* Schedule next alarm */
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = edata->alarm_value + next_interval,
        .flags.auto_reload_on_alarm = false,
    };
    gptimer_set_alarm_action(timer, &alarm_config);

    return false;
}

/* ===================================================================
 * Initialization
 * =================================================================== */

void stepper_init(void)
{
    memset(&st, 0, sizeof(st));

    /* Load pin assignments from NVS (compile-time defaults as fallback) */
    {
        static const char *step_nvs[] = {"p_sx","p_sy","p_sz","p_sa","p_sb","p_sc"};
        static const char *dir_nvs[]  = {"p_dx","p_dy","p_dz","p_da","p_db","p_dc"};
        gpio_num_t step_defaults[] = STEP_PINS;
        gpio_num_t dir_defaults[]  = DIR_PINS;
        for (int i = 0; i < WCNC_MAX_AXES; i++) {
            st.step_pins[i] = (gpio_num_t)nvs_config_get_u8(step_nvs[i], (uint8_t)step_defaults[i]);
            st.dir_pins[i]  = (gpio_num_t)nvs_config_get_u8(dir_nvs[i],  (uint8_t)dir_defaults[i]);
        }
        st.enable_pin = (gpio_num_t)nvs_config_get_u8("p_en",  (uint8_t)PIN_ENABLE);
        st.probe_pin  = (gpio_num_t)nvs_config_get_u8("p_prb", (uint8_t)PIN_PROBE);
        ESP_LOGI(TAG, "Pin config: step=[%d,%d,%d,%d,%d,%d] dir=[%d,%d,%d,%d,%d,%d] en=%d probe=%d",
                 st.step_pins[0], st.step_pins[1], st.step_pins[2],
                 st.step_pins[3], st.step_pins[4], st.step_pins[5],
                 st.dir_pins[0], st.dir_pins[1], st.dir_pins[2],
                 st.dir_pins[3], st.dir_pins[4], st.dir_pins[5],
                 st.enable_pin, st.probe_pin);
    }

    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        bool has_step = pin_valid_output(st.step_pins[i]);
        bool has_dir = pin_valid_output(st.dir_pins[i]);
        if (has_step != has_dir) {
            ESP_LOGW(TAG, "Axis %d has unusable step/dir pin pair (%d,%d), disabling axis", i,
                     (int)st.step_pins[i], (int)st.dir_pins[i]);
            st.step_pins[i] = GPIO_NUM_NC;
            st.dir_pins[i] = GPIO_NUM_NC;
            continue;
        }
        if (!has_step && ((int)st.step_pins[i] != GPIO_NUM_NC || (int)st.dir_pins[i] != GPIO_NUM_NC)) {
            ESP_LOGW(TAG, "Axis %d pins (%d,%d) are not usable on this board, disabling axis", i,
                     (int)st.step_pins[i], (int)st.dir_pins[i]);
            st.step_pins[i] = GPIO_NUM_NC;
            st.dir_pins[i] = GPIO_NUM_NC;
        }
    }
    if (!pin_valid_output(st.enable_pin)) {
        ESP_LOGW(TAG, "Enable pin %d is not usable on this board, disabling", (int)st.enable_pin);
        st.enable_pin = GPIO_NUM_NC;
    }
    if (!pin_valid_input(st.probe_pin)) {
        ESP_LOGW(TAG, "Probe pin %d is not usable on this board, disabling", (int)st.probe_pin);
        st.probe_pin = GPIO_NUM_NC;
    }

    /* Default timing (overridden by NVS config below) */
    st.step_pulse_us = CFG_DEFAULT_STEP_PULSE_US;
    st.segment_complete = true;
    st.running = false;
    st.estopped = false;

    /* Configure step and direction GPIO pins as outputs (skip NC axes) */
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        bool has_step = pin_valid_output(st.step_pins[i]);
        bool has_dir  = pin_valid_output(st.dir_pins[i]);
        if (!has_step && !has_dir) continue;

        uint64_t mask = 0;
        if (has_step) mask |= (1ULL << st.step_pins[i]);
        if (has_dir)  mask |= (1ULL << st.dir_pins[i]);

        gpio_config_t io_conf = {
            .pin_bit_mask = mask,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        if (has_step) gpio_set_level(st.step_pins[i], 0);
        if (has_dir)  gpio_set_level(st.dir_pins[i], 0);
    }

    /* Configure enable pin */
    if (pin_valid_output(st.enable_pin)) {
        gpio_config_t en_conf = {
            .pin_bit_mask = (1ULL << st.enable_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&en_conf);
    }
    stepper_set_enabled(false);

    /* Create hardware timer */
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = CFG_STEPPER_TIMER_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &step_timer));

    /* Register ISR callback */
    gptimer_event_callbacks_t cbs = {
        .on_alarm = stepper_timer_isr,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(step_timer, &cbs, NULL));

    /* Enable timer */
    ESP_ERROR_CHECK(gptimer_enable(step_timer));

    /* Apply timing config from NVS (overrides defaults with Mach3 values) */
    stepper_apply_timing_config();

    /* Load per-axis rate and acceleration from NVS */
    stepper_apply_axis_config();

    ESP_LOGI(TAG, "Stepper engine initialized (timer @ %d Hz, pulse=%luus)",
             CFG_STEPPER_TIMER_RESOLUTION_HZ, (unsigned long)st.step_pulse_us);
}

void stepper_apply_timing_config(void)
{
    uint16_t pulse = nvs_config_get_u16("pulse_us", CFG_DEFAULT_STEP_PULSE_US);
    uint16_t dir   = nvs_config_get_u16("dir_us",   CFG_DEFAULT_DIR_SETUP_US);

    /* Sanity clamp: 1-50 microseconds */
    if (pulse < 1) pulse = 1;
    if (pulse > 50) pulse = 50;
    if (dir > 50) dir = 50;

    st.step_pulse_us = pulse;
    st.dir_setup_us = dir;

    ESP_LOGI(TAG, "Timing config applied: step_pulse=%uus, dir_setup=%uus", pulse, dir);
}

void stepper_apply_axis_config(void)
{
    static const char *rate_keys[] = {"rate_x","rate_y","rate_z","rate_a","rate_b","rate_c"};
    static const char *acc_keys[]  = {"acc_x","acc_y","acc_z","acc_a","acc_b","acc_c"};
    static const char *spm_keys[]  = {"spm_x","spm_y","spm_z","spm_a","spm_b","spm_c"};

    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        st.max_rate[i]  = nvs_config_get_u32(rate_keys[i], CFG_DEFAULT_MAX_RATE);
        st.max_accel[i] = nvs_config_get_u32(acc_keys[i],  CFG_DEFAULT_ACCELERATION);
        st.steps_per_mm[i] = nvs_config_get_float(spm_keys[i], CFG_DEFAULT_STEPS_PER_MM);

        /* Clamp to aggregate limit */
        if (st.max_rate[i] > CFG_MAX_AGGREGATE_STEP_RATE)
            st.max_rate[i] = CFG_MAX_AGGREGATE_STEP_RATE;
    }

    /* Cache jerk setting ($40) */
    st.jerk_max_mm = nvs_config_get_float("jerk_max", 1000.0f);

    ESP_LOGI(TAG, "Axis config applied: X rate=%lu acc=%lu jerk=%.0f",
             (unsigned long)st.max_rate[0], (unsigned long)st.max_accel[0],
             st.jerk_max_mm);
}

/* ===================================================================
 * Segment Loading
 * =================================================================== */

void stepper_load_segment(const wcnc_motion_segment_t *seg)
{
    /* Stop timer while configuring new segment */
    gptimer_stop(step_timer);

    /* Determine dominant axis (most steps) */
    uint32_t max_steps = 0;
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        uint32_t abs_steps = (uint32_t)abs(seg->steps[i]);
        if (abs_steps > max_steps) max_steps = abs_steps;
    }

    if (max_steps == 0) {
        /* Zero-length segment — mark complete immediately */
        st.segment_complete = true;
        st.running = false;
        return;
    }

    /* Set up Bresenham counters */
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        st.segment_steps[i] = (uint32_t)abs(seg->steps[i]);
        st.direction[i] = (seg->steps[i] >= 0) ? 1 : -1;
        st.counter[i] = -(int32_t)(max_steps / 2);
    }
    st.total_steps = max_steps;
    st.steps_remaining = max_steps;

    /* Compute timing from segment speed parameters */
    float entry_speed = sqrtf((float)seg->entry_speed_sqr / 1000.0f);
    float exit_speed  = sqrtf((float)seg->exit_speed_sqr / 1000.0f);
    float accel = (float)seg->acceleration / 100.0f;

    /* Minimum speed clamp — must be high enough that the starting
     * timer interval is reasonable (100 sps = 10000µs = 10ms/step) */
    if (entry_speed < 100.0f) entry_speed = 100.0f;
    if (exit_speed < 100.0f)  exit_speed = 100.0f;
    if (accel < 1.0f) accel = 1.0f;

    st.initial_interval = (uint32_t)((float)CFG_STEPPER_TIMER_RESOLUTION_HZ / entry_speed);
    st.final_interval   = (uint32_t)((float)CFG_STEPPER_TIMER_RESOLUTION_HZ / exit_speed);

    /* Compute nominal (cruise) speed using kinematic equations */
    float nominal_speed_sqr = entry_speed * entry_speed + 2.0f * accel * (float)max_steps;
    float nominal_speed = sqrtf(nominal_speed_sqr);

    /* Clamp to per-axis and aggregate max rate */
    float max_rate_f = (float)CFG_MAX_AGGREGATE_STEP_RATE;
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        if (st.segment_steps[i] > 0 && st.max_rate[i] > 0) {
            /* Scale aggregate limit by this axis's share of total steps */
            float axis_limit = (float)st.max_rate[i] * (float)max_steps
                               / (float)st.segment_steps[i];
            if (axis_limit < max_rate_f) max_rate_f = axis_limit;
        }
    }
    if (nominal_speed > max_rate_f) nominal_speed = max_rate_f;

    st.nominal_interval = (uint32_t)((float)CFG_STEPPER_TIMER_RESOLUTION_HZ / nominal_speed);
    if (st.nominal_interval < 4) st.nominal_interval = 4;

    /* Compute acceleration/deceleration boundaries */
    float steps_to_accel = (nominal_speed * nominal_speed - entry_speed * entry_speed)
                           / (2.0f * accel);
    float steps_to_decel = (nominal_speed * nominal_speed - exit_speed * exit_speed)
                           / (2.0f * accel);

    if (steps_to_accel < 0) steps_to_accel = 0;
    if (steps_to_decel < 0) steps_to_decel = 0;

    /* Check for triangle profile (never reaches nominal speed) */
    if ((uint32_t)(steps_to_accel + steps_to_decel) > max_steps) {
        /* Recompute: find intersection speed */
        float intersection_sqr = (2.0f * accel * (float)max_steps
                                  + entry_speed * entry_speed
                                  + exit_speed * exit_speed) / 2.0f;
        float intersection_speed = sqrtf(intersection_sqr);

        steps_to_accel = (intersection_speed * intersection_speed
                          - entry_speed * entry_speed) / (2.0f * accel);
        steps_to_decel = (float)max_steps - steps_to_accel;

        if (steps_to_accel < 0) steps_to_accel = 0;
        if (steps_to_decel < 0) steps_to_decel = 0;

        nominal_speed = intersection_speed;
        st.nominal_interval = (uint32_t)((float)CFG_STEPPER_TIMER_RESOLUTION_HZ
                                         / intersection_speed);
    }

    st.accel_until_step = (uint32_t)steps_to_accel;
    st.decel_after_step = max_steps - (uint32_t)steps_to_decel;

    /* Initialize kinematic v² tracking for the ISR */
    st.speed_sq         = (uint32_t)(entry_speed * entry_speed);
    st.two_accel        = (uint32_t)(2.0f * accel);
    st.nominal_speed_sq = (uint32_t)(nominal_speed * nominal_speed);
    st.exit_speed_sq    = (uint32_t)(exit_speed * exit_speed);

    /* --- S-Curve (jerk-limited) phase computation ---
     * Smooths acceleration transitions by ramping 2*a from 0 to max
     * instead of switching instantly. Backward-compatible: jerk=0 = trapezoidal. */
    uint32_t accel_steps = st.accel_until_step;
    uint32_t decel_steps = max_steps - st.decel_after_step;

    if (st.jerk_max_mm > 0.0f && accel > 1.0f && (accel_steps + decel_steps) > 6) {
        /* Find dominant axis for steps_per_mm conversion */
        int dominant = 0;
        for (int i = 1; i < WCNC_MAX_AXES; i++) {
            if (st.segment_steps[i] > st.segment_steps[dominant]) dominant = i;
        }
        float spm = st.steps_per_mm[dominant];
        if (spm < 1.0f) spm = 1.0f;

        /* Convert jerk from mm/sec³ to steps/sec³ */
        float jerk_sps3 = st.jerk_max_mm * spm;

        /* Time to ramp accel from 0 to max: t_j = a_max / j_max */
        float t_j = accel / jerk_sps3;

        /* Estimate steps during accel jerk ramp (avg speed * time) */
        float avg_accel_speed = entry_speed + 0.25f * accel * t_j;
        uint32_t accel_jerk = (uint32_t)(avg_accel_speed * t_j);
        if (accel_jerk > accel_steps / 3) accel_jerk = accel_steps / 3;
        if (accel_jerk < 2) accel_jerk = 0;

        /* Estimate steps during decel jerk ramp */
        float avg_decel_speed = exit_speed + 0.25f * accel * t_j;
        uint32_t decel_jerk = (uint32_t)(avg_decel_speed * t_j);
        if (decel_jerk > decel_steps / 3) decel_jerk = decel_steps / 3;
        if (decel_jerk < 2) decel_jerk = 0;

        if (accel_jerk > 0 || decel_jerk > 0) {
            st.scurve_active = true;
            st.two_accel_max = (uint32_t)(2.0f * accel);

            /* Phase boundaries (step numbers) */
            st.phase_end[0] = accel_jerk;                           /* Accel jerk-up end */
            st.phase_end[1] = (accel_steps > accel_jerk)
                              ? accel_steps - accel_jerk : accel_jerk;  /* Const-accel end */
            st.phase_end[2] = accel_steps;                          /* Accel jerk-down end */
            st.phase_end[3] = st.decel_after_step;                  /* Cruise end */
            st.phase_end[4] = st.decel_after_step + decel_jerk;     /* Decel jerk-down end */
            st.phase_end[5] = (max_steps > decel_jerk)
                              ? max_steps - decel_jerk : max_steps;    /* Const-decel end */

            /* Precompute jerk delta per step (use larger ramp for precision) */
            uint32_t jerk_ramp = (accel_jerk > decel_jerk) ? accel_jerk : decel_jerk;
            if (jerk_ramp < 1) jerk_ramp = 1;
            st.jerk_step_delta = (int32_t)(st.two_accel_max / jerk_ramp);
            if (st.jerk_step_delta < 1) st.jerk_step_delta = 1;

            st.current_two_accel = 0;
        } else {
            st.scurve_active = false;
        }
    } else {
        st.scurve_active = false;
    }

    /* Store segment flags for probe detection */
    st.seg_flags = seg->flags;
    if (seg->flags & WCNC_SEG_FLAG_PROBE) {
        st.probe_triggered = false;
        if (!pin_valid_input(st.probe_pin)) {
            ESP_LOGW(TAG, "Probe move requested but probe pin is not usable on this board, ignoring probe flag");
            st.seg_flags &= (uint8_t)~WCNC_SEG_FLAG_PROBE;
        } else {
            st.invert_probe = nvs_config_get_u8("inv_prb", CFG_DEFAULT_INVERT_PROBE);
        }
    }

    /* Start the timer */
    st.current_interval = st.initial_interval;
    st.segment_complete = false;
    st.running = true;
    st.jog_mode = false;

    /* Reset and start timer with initial alarm */
    gptimer_set_raw_count(step_timer, 0);

    gptimer_alarm_config_t alarm = {
        .alarm_count = st.initial_interval,
        .flags.auto_reload_on_alarm = false,
    };
    gptimer_set_alarm_action(step_timer, &alarm);
    gptimer_start(step_timer);

    /* Enable stepper drivers */
    stepper_set_enabled(true);
}

/* ===================================================================
 * Jog Control
 * =================================================================== */

void stepper_start_jog(uint8_t axis, int8_t direction, uint32_t speed_steps_per_sec)
{
    if (axis >= WCNC_MAX_AXES || st.estopped) return;

    gptimer_stop(step_timer);

    /* Configure for single-axis continuous motion */
    memset(st.segment_steps, 0, sizeof(st.segment_steps));
    memset(st.direction, 0, sizeof(st.direction));
    memset(st.counter, 0, sizeof(st.counter));

    st.segment_steps[axis] = 1;
    st.direction[axis] = direction;
    st.total_steps = 1;
    st.steps_remaining = UINT32_MAX; /* Run indefinitely */
    st.counter[axis] = 0;

    /* Constant speed - no acceleration profile for jog */
    uint32_t interval = (speed_steps_per_sec > 0)
        ? CFG_STEPPER_TIMER_RESOLUTION_HZ / speed_steps_per_sec
        : 1000000;
    if (interval < 4) interval = 4;

    st.current_interval = interval;
    st.nominal_interval = interval;
    st.initial_interval = interval;
    st.final_interval = interval;
    st.accel_until_step = 0;
    st.decel_after_step = UINT32_MAX;

    /* Constant speed — initialize v² tracking at cruise speed */
    st.speed_sq         = speed_steps_per_sec * speed_steps_per_sec;
    st.two_accel        = 0;
    st.nominal_speed_sq = st.speed_sq;
    st.exit_speed_sq    = st.speed_sq;
    st.scurve_active    = false;  /* No S-curve for constant-speed jog */

    st.segment_complete = false;
    st.running = true;
    st.jog_mode = true;

    gptimer_set_raw_count(step_timer, 0);
    gptimer_alarm_config_t alarm = {
        .alarm_count = interval,
        .flags.auto_reload_on_alarm = false,
    };
    gptimer_set_alarm_action(step_timer, &alarm);
    gptimer_start(step_timer);

    stepper_set_enabled(true);
}

void stepper_stop_jog(void)
{
    if (st.jog_mode) {
        gptimer_stop(step_timer);
        st.running = false;
        st.segment_complete = true;
        st.jog_mode = false;
        st.current_feed_rate = 0;
    }
}

/* ===================================================================
 * Control Functions
 * =================================================================== */

void stepper_estop(void)
{
    st.estopped = true;
    gptimer_stop(step_timer);
    st.running = false;
    st.segment_complete = true;
    st.current_feed_rate = 0;

    /* Immediately disable all outputs (skip NC pins) */
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        if (pin_valid_output(st.step_pins[i]))
            gpio_set_level(st.step_pins[i], 0);
    }
    stepper_set_enabled(false);

    ESP_LOGW(TAG, "E-STOP activated");
}

void stepper_reset(void)
{
    if (!st.estopped) return;

    st.estopped = false;
    st.holding = false;
    st.running = false;
    st.segment_complete = true;
    st.steps_remaining = 0;

    ESP_LOGI(TAG, "Stepper reset from E-Stop");
}

void stepper_feed_hold(void)
{
    st.holding = true;
    /* The ISR will check this flag and stop generating steps */
    ESP_LOGI(TAG, "Feed hold activated");
}

void stepper_feed_resume(void)
{
    if (st.holding) {
        st.holding = false;
        /* If timer was stopped, restart it */
        if (st.running && st.steps_remaining > 0) {
            gptimer_set_raw_count(step_timer, 0);
            gptimer_alarm_config_t alarm = {
                .alarm_count = st.current_interval,
                .flags.auto_reload_on_alarm = false,
            };
            gptimer_set_alarm_action(step_timer, &alarm);
            gptimer_start(step_timer);
        }
        ESP_LOGI(TAG, "Feed hold released");
    }
}

void stepper_set_enabled(bool enabled)
{
    if (!pin_valid_output(st.enable_pin)) return;
    bool actual = enabled ^ (st.invert_enable & 1);
    gpio_set_level(st.enable_pin, actual ? 1 : 0);
}

/* ===================================================================
 * State Queries
 * =================================================================== */

bool stepper_is_running(void)      { return st.running; }
bool stepper_segment_complete(void){ return st.segment_complete; }
bool stepper_is_estopped(void)     { return st.estopped; }
bool stepper_is_holding(void)      { return st.holding; }
bool stepper_probe_triggered(void) { return st.probe_triggered; }

void stepper_get_position(int32_t position[WCNC_MAX_AXES])
{
    /* Brief interrupt disable to snapshot position atomically.
     * Prevents ISR from updating mid-read (position consistency). */
    portENTER_CRITICAL(&s_stepper_mux);
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        position[i] = st.position[i];
    }
    portEXIT_CRITICAL(&s_stepper_mux);
}

void stepper_zero_position(void)
{
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        st.position[i] = 0;
    }
}

int32_t stepper_get_feed_rate(void)
{
    return st.current_feed_rate;
}


