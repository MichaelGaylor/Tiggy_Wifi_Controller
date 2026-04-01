/*
 * WiFi CNC Controller - Plugin Configuration
 *
 * Stores all motion/IO settings in the plugin itself (registry),
 * instead of depending on Mach3's Motor Tuning panel (which is
 * designed for the parallel port and unreliable with external controllers).
 */

#ifndef PLUGINCONFIG_H
#define PLUGINCONFIG_H

#include <cstdint>

#define WCNC_NUM_AXES 6

struct PluginAxisConfig {
    bool   enabled;
    double stepsPerUnit;       /* steps per user unit (mm or inch) */
    double maxVelUnitsPerMin;  /* max velocity in user units/min */
    double accelUnitsPerSec2;  /* acceleration in user units/sec^2 */

    /* Axis cloning (slaving) */
    int    cloneMaster;        /* axis index this is cloned from, -1 = none */
    bool   cloneReversed;      /* move in opposite direction to master */
};

struct PluginConfig {
    /* Connection */
    char espAddress[64];

    /* Per-axis (all 6: X Y Z A B C) */
    PluginAxisConfig axis[WCNC_NUM_AXES];

    /* Timing */
    uint16_t stepPulseUs;
    uint16_t dirSetupUs;

    /* Inversion bitmasks (bit per axis, bit0=X .. bit5=C) */
    uint8_t invertStep;
    uint8_t invertDir;
    uint8_t invertLimit;
    uint8_t invertHome;
    uint8_t invertEstop;    /* 0=active low, 1=active high */
    uint8_t invertProbe;    /* 0=active low, 1=active high */

    /* Homing */
    uint8_t  homingDirMask;     /* bit per axis: 1=positive direction */
    uint32_t homingSeekRate;    /* steps/sec (fast approach) */
    uint32_t homingFeedRate;    /* steps/sec (slow/fine) */
    uint32_t homingPulloff;     /* pulloff distance in steps */

    /* Spindle */
    uint16_t spindlePwmFreq;   /* Hz */
    uint32_t spindleMaxRpm;

    /* Charge pump */
    uint16_t chargePumpFreq;   /* Hz, 0 = disabled */

    /* Step idle */
    uint16_t stepIdleDelayMs;  /* ms before disabling steppers, 0 = never */

    /* Laser mode */
    bool     laserMode;        /* true = spindle PWM tracks feed rate */

    /* Input function mapping: what DoButton() code each misc input triggers.
     * 0 = none, 1000 = Cycle Start, 1001 = Feed Hold, etc.
     * Rising-edge only (triggers once when input goes active). */
    uint16_t miscInputFunction[4];

    /* Board profile (matches Name= in .pinmap file) */
    char boardName[64];

    /* I/O expansion module */
    char ioModuleAddress[64];   /* IP address of I/O module ESP32 */
    bool ioModuleEnabled;       /* Connect to I/O module on startup */

    /* I/O module input function mapping: what action each input triggers.
     * 0 = none, 101-108 = jog X+/X-/Y+/Y-/Z+/Z-/A+/A-,
     * 1000+ = DoButton() code (Cycle Start, Feed Hold, etc.) */
    uint16_t ioInputFunction[16];
    uint32_t ioJogSpeed;        /* Jog speed for pendant buttons (steps/sec) */

    /* I/O module output function mapping: what Mach3 signal each output mirrors.
     * 0 = none, 1 = Spindle CW, 2 = Spindle CCW, 3 = Flood,
     * 4 = Mist, 5 = Output 1, 6 = Output 2, 7 = Output 3, 8 = Output 4 */
    uint16_t ioOutputFunction[16];

    /* License */
    char licenseEmail[128];
    char licenseKey[128];
};

/* Fill config with sensible hobby CNC defaults */
void PluginConfig_LoadDefaults(PluginConfig *cfg);

/* Load from registry. Returns false if key doesn't exist (first run). */
bool PluginConfig_LoadFromRegistry(PluginConfig *cfg);

/* Save all settings to registry */
void PluginConfig_SaveToRegistry(const PluginConfig *cfg);

#endif /* PLUGINCONFIG_H */
