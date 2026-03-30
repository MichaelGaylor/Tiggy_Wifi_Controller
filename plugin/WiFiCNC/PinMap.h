/*
 * WiFi CNC Controller - Board Pin Map
 *
 * Loads board-specific GPIO pin assignments from .pinmap files.
 * Files are placed in the same folder as the DLL (e.g., C:\Mach3\PlugIns\).
 *
 * Format is simple INI-style:
 *   [Board]
 *   Name=Tiggy Standard Motion Controller
 *   ...
 *   [Step]
 *   X=17
 *   Y=18
 *   ...
 *
 * Pin value of -1 means "not assigned / not available on this board".
 */

#ifndef PINMAP_H
#define PINMAP_H

#include <cstdint>

#define PINMAP_MAX_NAME   64
#define PINMAP_MAX_DESC   128
#define PINMAP_MAX_AXES   6
#define PINMAP_MAX_BOARDS 8

struct PinMapEntry {
    char name[PINMAP_MAX_NAME];
    char description[PINMAP_MAX_DESC];

    int8_t stepPin[PINMAP_MAX_AXES];    /* GPIO for step pulse */
    int8_t dirPin[PINMAP_MAX_AXES];     /* GPIO for direction */
    int8_t enablePin;                    /* Shared stepper enable */

    int8_t limitPin[PINMAP_MAX_AXES];   /* GPIO for limit switch */
    int8_t homePin[PINMAP_MAX_AXES];    /* GPIO for home switch */
    int8_t estopPin;                     /* GPIO for E-stop */
    int8_t probePin;                     /* GPIO for probe */

    int8_t spindlePin;                   /* GPIO for spindle PWM */
    int8_t chargePumpPin;                /* GPIO for charge pump */
    int8_t miscOutPin[2];               /* GPIO for misc outputs */
    int8_t miscInPin[4];                /* GPIO for misc inputs */
    int8_t ledPin;                       /* Status LED */
};

/* Initialize a PinMapEntry with all pins = -1 */
void PinMap_Init(PinMapEntry *entry);

/* Load a .pinmap file. Returns true on success. */
bool PinMap_LoadFile(const char *filepath, PinMapEntry *entry);

/* Scan a folder for .pinmap files.
 * Fills names[] with board names (from [Board] Name=).
 * Fills paths[] with full file paths.
 * Returns number of boards found (up to maxEntries). */
int PinMap_ScanFolder(const char *folder,
                      char names[][PINMAP_MAX_NAME],
                      char paths[][260],
                      int maxEntries);

/* Load by board name: scans folder, finds matching file, loads it. */
bool PinMap_LoadByName(const char *folder, const char *boardName,
                       PinMapEntry *entry);

#endif /* PINMAP_H */
