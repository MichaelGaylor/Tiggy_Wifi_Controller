/*
 * PC-side test of the terminal layouts (firmware/main/pin_layout.c).
 *
 * A wrong pin number cannot be found by any simulator: it shows when someone
 * wires a machine.  This goes through every layout and every kind of
 * spindle sensor of one board and checks what can be checked without it:
 *   - no terminal is given two jobs;
 *   - no job is put on a pin the board cannot spare (flash and PSRAM, USB,
 *     the serial console) or cannot drive (input-only pins);
 *   - four axes means X, Y, Z and A keep their terminals and Ethernet stays;
 *   - the sensor gets the inputs its kind needs, and only those;
 *   - no terminal is an input with one setting and an output with another
 *     (a sensor left wired would be driven against);
 *   - the numbers are the ones the documentation gives.
 *
 * Built once per board:
 *   cc -DBOARD_ESP32S3_DEVKITC_OCTAL -Istub -I../main -o t test_pin_layout.c ../main/pin_layout.c
 * (likewise BOARD_ESP32S3_DEVKITC_QUAD, BOARD_ESP32S3_ZERO, BOARD_ESP32_WROOM32).
 * Prints the layouts; a non-zero exit code means a failure.
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "../main/pin_layout.h"
#include "../main/pin_config.h"
#include "../../protocol/wifi_cnc_protocol.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { g_fail++; printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* --- the stored settings pin_layout.c reads --- */
static uint8_t  t_layout, t_mode;
static uint16_t t_ppr;

uint8_t nvs_config_get_u8(const char *key, uint8_t fallback)
{
    if (strcmp(key, "layout") == 0) return t_layout;
    if (strcmp(key, "enc_mode") == 0) return t_mode;
    return fallback;
}

uint16_t nvs_config_get_u16(const char *key, uint16_t fallback)
{
    return strcmp(key, "enc_ppr") == 0 ? t_ppr : fallback;
}

/* --- every job a terminal can have --- */
typedef struct { uint16_t key; const char *name; bool output; } job_t;
static const job_t JOBS[] = {
    { WCNC_CFG_PIN_STEP_X, "X step", true },   { WCNC_CFG_PIN_DIR_X, "X dir", true },   { WCNC_CFG_PIN_LIMIT_X, "X limit", false },
    { WCNC_CFG_PIN_STEP_Y, "Y step", true },   { WCNC_CFG_PIN_DIR_Y, "Y dir", true },   { WCNC_CFG_PIN_LIMIT_Y, "Y limit", false },
    { WCNC_CFG_PIN_STEP_Z, "Z step", true },   { WCNC_CFG_PIN_DIR_Z, "Z dir", true },   { WCNC_CFG_PIN_LIMIT_Z, "Z limit", false },
    { WCNC_CFG_PIN_STEP_A, "A step", true },   { WCNC_CFG_PIN_DIR_A, "A dir", true },   { WCNC_CFG_PIN_LIMIT_A, "A limit", false },
    { WCNC_CFG_PIN_STEP_B, "B step", true },   { WCNC_CFG_PIN_DIR_B, "B dir", true },   { WCNC_CFG_PIN_LIMIT_B, "B limit", false },
    { WCNC_CFG_PIN_STEP_C, "C step", true },   { WCNC_CFG_PIN_DIR_C, "C dir", true },   { WCNC_CFG_PIN_LIMIT_C, "C limit", false },
    { WCNC_CFG_PIN_ENABLE, "enable", true },   { WCNC_CFG_PIN_PROBE, "probe", false },  { WCNC_CFG_PIN_ESTOP, "E-stop", false },
    { WCNC_CFG_PIN_SPINDLE, "spindle", true }, { WCNC_CFG_PIN_LED, "LED", true },       { WCNC_CFG_PIN_CHARGE_PUMP, "charge pump", true },
    { WCNC_CFG_PIN_MISC_OUT0, "aux out 1", true }, { WCNC_CFG_PIN_MISC_OUT1, "aux out 2", true },
    { WCNC_CFG_PIN_MISC_OUT2, "aux out 3", true }, { WCNC_CFG_PIN_MISC_OUT3, "aux out 4", true },
    { WCNC_CFG_PIN_MISC_OUT4, "aux out 5", true },
    { WCNC_CFG_PIN_ENCODER_A, "encoder A", false }, { WCNC_CFG_PIN_ENCODER_B, "encoder B", false },
    { WCNC_CFG_PIN_ENCODER_INDEX, "index", false },
    { WCNC_CFG_PIN_MISC_IN0, "aux in 1", false },  { WCNC_CFG_PIN_MISC_IN1, "aux in 2", false },
    { WCNC_CFG_PIN_MISC_IN2, "aux in 3", false },  { WCNC_CFG_PIN_MISC_IN3, "aux in 4", false },
    { WCNC_CFG_PIN_ETH_MOSI, "Ethernet MOSI", true }, { WCNC_CFG_PIN_ETH_MISO, "Ethernet MISO", false },
    { WCNC_CFG_PIN_ETH_SCLK, "Ethernet clock", true }, { WCNC_CFG_PIN_ETH_INT, "Ethernet interrupt", false },
};
#define NUM_JOBS ((int)(sizeof(JOBS) / sizeof(JOBS[0])))

#if defined(BOARD_ESP32S3_DEVKITC_OCTAL)
#define BOARD "ESP32-S3-DevKitC, octal memory"
#elif defined(BOARD_ESP32S3_DEVKITC_QUAD)
#define BOARD "ESP32-S3-DevKitC, quad memory"
#elif defined(BOARD_ESP32S3_ZERO)
#define BOARD "ESP32-S3-Zero"
#else
#define BOARD "ESP32-WROOM-32"
#endif

/* Why a job cannot be on a pin; NULL if it can. */
static const char *unusable(int pin, bool output)
{
#if defined(BOARD_ESP32_WROOM32)
    if (pin > 39) return "no such pin";
    if (pin >= 6 && pin <= 11) return "flash";
    if (output && pin >= 34) return "input-only pin";
#else
    (void)output;
    if (pin > 48) return "no such pin";
    if (pin == 19 || pin == 20) return "USB";
    if (pin == 43 || pin == 44) return "serial console";
    if (pin >= 22 && pin <= 25) return "no such pin";
    if (pin >= 26 && pin <= 32) return "flash";
#if !defined(BOARD_ESP32S3_DEVKITC_QUAD)
    if (pin >= 33 && pin <= 37) return "PSRAM";
#endif
#endif
    return NULL;
}

static int pin_of(uint16_t key) { return (int)pin_layout_default(key); }

static const char *SENSORS[] = { "no sensor", "encoder with index", "single sensor", "encoder without index" };

static void set(int layout, int sensor)
{
    t_layout = (uint8_t)layout;
    t_ppr = sensor == PIN_SENSOR_NONE ? 0 : 600;
    t_mode = sensor == PIN_SENSOR_PULSE ? 1 : sensor == PIN_SENSOR_ENCODER ? 2 : 0;
    pin_layout_reload();
}

static void expect(uint16_t key, int pin, const char *what)
{
    CHECK(pin_of(key) == pin, "%s: GPIO %d, the documentation says %d", what, pin_of(key), pin);
}

int main(void)
{
    printf("%s\n", BOARD);
    int base[NUM_JOBS];
    int way[64] = { 0 };                    /* each terminal: 0 not seen yet, 1 input, 2 output */
    const char *way_job[64] = { 0 };
    set(PIN_LAYOUT_SIX_AXES, PIN_SENSOR_NONE);
    for (int j = 0; j < NUM_JOBS; j++) base[j] = pin_of(JOBS[j].key);

    for (int layout = 0; layout <= 1; layout++) {
        for (int sensor = PIN_SENSOR_NONE; sensor <= PIN_SENSOR_ENCODER; sensor++) {
            set(layout, sensor);
            CHECK(pin_layout() == layout && pin_layout_sensor() == sensor, "settings not taken up");
            char where[96];
            snprintf(where, sizeof(where), "%s axes, %s", layout ? "four" : "six", SENSORS[sensor]);

            int user[64];
            for (int p = 0; p < 64; p++) user[p] = -1;
            for (int j = 0; j < NUM_JOBS; j++) {
                int pin = pin_of(JOBS[j].key);
                if (pin < 0) continue;
                const char *why = unusable(pin, JOBS[j].output);
                CHECK(!why, "%s: %s is on GPIO %d (%s)", where, JOBS[j].name, pin, why);
                if (pin > 63) continue;
                if (user[pin] >= 0) {
#if defined(BOARD_ESP32_WROOM32)
                    /* As this board has always been: its lamp is on the C limit input */
                    if (pin == 2) continue;
#endif
                    CHECK(false, "%s: GPIO %d is both %s and %s", where, pin, JOBS[user[pin]].name, JOBS[j].name);
                }
                user[pin] = j;
                int w = JOBS[j].output ? 2 : 1;
                CHECK(way[pin] == 0 || way[pin] == w, "%s: GPIO %d is %s here (%s) and %s elsewhere (%s)", where, pin,
                      w == 2 ? "an output" : "an input", JOBS[j].name, w == 2 ? "an input" : "an output", way_job[pin]);
                if (way[pin] == 0) {
                    way[pin] = w;
                    way_job[pin] = JOBS[j].name;
                }
            }

            /* The axes that stay, and everything the layout has no business with, are where they were */
            for (int j = 0; j < NUM_JOBS; j++) {
                uint16_t k = JOBS[j].key;
                bool axis_xyza = (k >= WCNC_CFG_PIN_STEP_X && k <= WCNC_CFG_PIN_STEP_A) ||
                                 (k >= WCNC_CFG_PIN_DIR_X && k <= WCNC_CFG_PIN_DIR_A) ||
                                 (k >= WCNC_CFG_PIN_LIMIT_X && k <= WCNC_CFG_PIN_LIMIT_A);
                bool fixed = axis_xyza || k == WCNC_CFG_PIN_ENABLE || k == WCNC_CFG_PIN_PROBE || k == WCNC_CFG_PIN_ESTOP ||
                             k == WCNC_CFG_PIN_SPINDLE || k == WCNC_CFG_PIN_LED ||
                             (k >= WCNC_CFG_PIN_ETH_MOSI && k <= WCNC_CFG_PIN_ETH_INT);
                if (fixed) CHECK(pin_of(k) == base[j], "%s: %s moved from GPIO %d to %d", where, JOBS[j].name, base[j], pin_of(k));
            }

            /* The sensor has the inputs its kind needs, and no others */
            bool wants_ab = sensor == PIN_SENSOR_ENCODER || sensor == PIN_SENSOR_ENCODER_INDEX;
            bool wants_index = sensor == PIN_SENSOR_PULSE || sensor == PIN_SENSOR_ENCODER_INDEX;
            if (!wants_ab) CHECK(pin_of(WCNC_CFG_PIN_ENCODER_A) < 0 && pin_of(WCNC_CFG_PIN_ENCODER_B) < 0, "%s: encoder terminals in use", where);
            if (!wants_index) CHECK(pin_of(WCNC_CFG_PIN_ENCODER_INDEX) < 0, "%s: index terminal in use", where);
            /* an index terminal is no use to an encoder that has no terminals itself */
            if (sensor == PIN_SENSOR_ENCODER_INDEX && pin_of(WCNC_CFG_PIN_ENCODER_A) < 0)
                CHECK(pin_of(WCNC_CFG_PIN_ENCODER_INDEX) < 0, "%s: index terminal kept for an encoder with no terminals", where);
#if defined(BOARD_ESP32S3_DEVKITC)
            if (layout == PIN_LAYOUT_FOUR_AXES) {
                CHECK(pin_of(WCNC_CFG_PIN_STEP_B) < 0 && pin_of(WCNC_CFG_PIN_DIR_B) < 0 && pin_of(WCNC_CFG_PIN_LIMIT_B) < 0 &&
                      pin_of(WCNC_CFG_PIN_STEP_C) < 0 && pin_of(WCNC_CFG_PIN_DIR_C) < 0 && pin_of(WCNC_CFG_PIN_LIMIT_C) < 0,
                      "%s: B or C still has a terminal", where);
                if (wants_ab) CHECK(pin_of(WCNC_CFG_PIN_ENCODER_A) >= 0 && pin_of(WCNC_CFG_PIN_ENCODER_B) >= 0, "%s: no terminals for the encoder", where);
                if (wants_index) CHECK(pin_of(WCNC_CFG_PIN_ENCODER_INDEX) >= 0, "%s: no terminal for the index", where);
                CHECK(pin_of(WCNC_CFG_PIN_CHARGE_PUMP) >= 0 && pin_of(WCNC_CFG_PIN_MISC_OUT0) >= 0 && pin_of(WCNC_CFG_PIN_MISC_OUT1) >= 0 &&
                      pin_of(WCNC_CFG_PIN_MISC_OUT2) >= 0 && pin_of(WCNC_CFG_PIN_MISC_IN0) >= 0,
                      "%s: charge pump, three outputs and an input are expected", where);
            }
            /* a single sensor has its terminal with six axes too */
            if (sensor == PIN_SENSOR_PULSE) CHECK(pin_of(WCNC_CFG_PIN_ENCODER_INDEX) >= 0, "%s: no terminal for the sensor", where);
#elif defined(BOARD_ESP32S3_ZERO)
            if (wants_ab) CHECK(pin_of(WCNC_CFG_PIN_ENCODER_A) >= 0 && pin_of(WCNC_CFG_PIN_ENCODER_B) >= 0, "%s: no terminals for the encoder", where);
            if (wants_index) CHECK(pin_of(WCNC_CFG_PIN_ENCODER_INDEX) >= 0, "%s: no terminal for the index", where);
#endif

            printf("  %-34s", where);
            static const uint16_t SHOW[] = { WCNC_CFG_PIN_ENCODER_A, WCNC_CFG_PIN_ENCODER_B, WCNC_CFG_PIN_ENCODER_INDEX,
                                             WCNC_CFG_PIN_MISC_IN0, WCNC_CFG_PIN_MISC_IN1, WCNC_CFG_PIN_MISC_IN2, WCNC_CFG_PIN_MISC_IN3,
                                             WCNC_CFG_PIN_CHARGE_PUMP, WCNC_CFG_PIN_MISC_OUT0, WCNC_CFG_PIN_MISC_OUT1,
                                             WCNC_CFG_PIN_MISC_OUT2, WCNC_CFG_PIN_MISC_OUT3, WCNC_CFG_PIN_MISC_OUT4 };
            static const char *LABEL[] = { "A", "B", "index", "in1", "in2", "in3", "in4", "pump", "out1", "out2", "out3", "out4", "out5" };
            for (int i = 0; i < (int)(sizeof(SHOW) / sizeof(SHOW[0])); i++) {
                int pin = pin_of(SHOW[i]);
                if (pin >= 0) printf(" %s=%d", LABEL[i], pin);
            }
            printf("\n");
        }
    }

    /* The numbers the documentation gives */
#if defined(BOARD_ESP32S3_DEVKITC_OCTAL)
    set(PIN_LAYOUT_FOUR_AXES, PIN_SENSOR_ENCODER_INDEX);
    expect(WCNC_CFG_PIN_ENCODER_A, 16, "encoder A");
    expect(WCNC_CFG_PIN_ENCODER_B, 17, "encoder B");
    expect(WCNC_CFG_PIN_ENCODER_INDEX, 21, "index");
    expect(WCNC_CFG_PIN_MISC_IN0, 18, "aux in 1");
    expect(WCNC_CFG_PIN_MISC_IN1, -1, "aux in 2 (the index has its terminal)");
    expect(WCNC_CFG_PIN_MISC_OUT0, 11, "aux out 1");
    expect(WCNC_CFG_PIN_MISC_OUT1, 12, "aux out 2");
    expect(WCNC_CFG_PIN_MISC_OUT2, 40, "aux out 3");
    expect(WCNC_CFG_PIN_CHARGE_PUMP, 39, "charge pump");
    set(PIN_LAYOUT_FOUR_AXES, PIN_SENSOR_NONE);
    expect(WCNC_CFG_PIN_MISC_IN1, 21, "aux in 2");
    expect(WCNC_CFG_PIN_MISC_IN2, 16, "aux in 3");
    expect(WCNC_CFG_PIN_MISC_IN3, 17, "aux in 4");
    set(PIN_LAYOUT_SIX_AXES, PIN_SENSOR_ENCODER_INDEX);
    expect(WCNC_CFG_PIN_ENCODER_A, -1, "encoder A with six axes");
    expect(WCNC_CFG_PIN_STEP_C, 39, "C step with six axes");
    expect(WCNC_CFG_PIN_MISC_IN1, 21, "aux in 2 with six axes");
    set(PIN_LAYOUT_SIX_AXES, PIN_SENSOR_PULSE);
    expect(WCNC_CFG_PIN_ENCODER_INDEX, 21, "single sensor with six axes");
    expect(WCNC_CFG_PIN_DIR_C, 12, "C dir with six axes");
#elif defined(BOARD_ESP32S3_DEVKITC_QUAD)
    set(PIN_LAYOUT_FOUR_AXES, PIN_SENSOR_ENCODER_INDEX);
    expect(WCNC_CFG_PIN_ENCODER_A, 16, "encoder A");
    expect(WCNC_CFG_PIN_ENCODER_B, 17, "encoder B");
    expect(WCNC_CFG_PIN_ENCODER_INDEX, 37, "index");
    expect(WCNC_CFG_PIN_MISC_IN2, 36, "aux in 3");
    expect(WCNC_CFG_PIN_MISC_IN3, -1, "aux in 4 (encoder A has its terminal)");
    expect(WCNC_CFG_PIN_CHARGE_PUMP, 10, "charge pump");
    expect(WCNC_CFG_PIN_MISC_OUT0, 11, "aux out 1");
    expect(WCNC_CFG_PIN_MISC_OUT1, 12, "aux out 2");
    expect(WCNC_CFG_PIN_MISC_OUT2, 40, "aux out 3");
    expect(WCNC_CFG_PIN_MISC_OUT3, 39, "aux out 4");
    expect(WCNC_CFG_PIN_MISC_OUT4, 34, "aux out 5");
    set(PIN_LAYOUT_FOUR_AXES, PIN_SENSOR_NONE);
    expect(WCNC_CFG_PIN_MISC_IN3, 16, "aux in 4");
    set(PIN_LAYOUT_SIX_AXES, PIN_SENSOR_PULSE);
    expect(WCNC_CFG_PIN_ENCODER_INDEX, 37, "single sensor with six axes");
    expect(WCNC_CFG_PIN_DIR_C, 35, "C dir with six axes");
#elif defined(BOARD_ESP32S3_ZERO)
    set(PIN_LAYOUT_SIX_AXES, PIN_SENSOR_ENCODER_INDEX);
    expect(WCNC_CFG_PIN_ENCODER_A, 15, "encoder A");
    expect(WCNC_CFG_PIN_ENCODER_B, 16, "encoder B");
    expect(WCNC_CFG_PIN_ENCODER_INDEX, 17, "index");
    set(PIN_LAYOUT_SIX_AXES, PIN_SENSOR_PULSE);
    expect(WCNC_CFG_PIN_ENCODER_INDEX, 17, "single sensor");
    expect(WCNC_CFG_PIN_MISC_IN0, 15, "aux in 1");
    expect(WCNC_CFG_PIN_MISC_IN1, 16, "aux in 2");
    set(PIN_LAYOUT_SIX_AXES, PIN_SENSOR_NONE);
    expect(WCNC_CFG_PIN_MISC_IN2, 17, "aux in 3");
#endif

    if (g_fail) printf("%d failure(s)\n", g_fail);
    else printf("  all layouts consistent\n");
    return g_fail ? 1 : 0;
}
