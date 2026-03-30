/*
 * WiFi CNC Controller - Board Pin Map Loader
 *
 * Minimal INI-style parser for .pinmap files.
 */

#include "PinMap.h"
#include <windows.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>

/* ===================================================================
 * Init
 * =================================================================== */

void PinMap_Init(PinMapEntry *entry)
{
    memset(entry, 0, sizeof(PinMapEntry));
    entry->name[0] = '\0';
    entry->description[0] = '\0';

    /* All pins default to -1 (not assigned) */
    for (int i = 0; i < PINMAP_MAX_AXES; i++) {
        entry->stepPin[i]  = -1;
        entry->dirPin[i]   = -1;
        entry->limitPin[i] = -1;
        entry->homePin[i]  = -1;
    }
    entry->enablePin     = -1;
    entry->estopPin      = -1;
    entry->probePin      = -1;
    entry->spindlePin    = -1;
    entry->chargePumpPin = -1;
    entry->ledPin        = -1;

    for (int i = 0; i < 2; i++) entry->miscOutPin[i] = -1;
    for (int i = 0; i < 4; i++) entry->miscInPin[i]  = -1;
}

/* ===================================================================
 * INI Parser Helpers
 * =================================================================== */

static void TrimWhitespace(char *s)
{
    /* Trim trailing */
    int len = (int)strlen(s);
    while (len > 0 && (s[len - 1] == ' ' || s[len - 1] == '\t' ||
                        s[len - 1] == '\r' || s[len - 1] == '\n'))
        s[--len] = '\0';

    /* Trim leading */
    char *p = s;
    while (*p == ' ' || *p == '\t') p++;
    if (p != s) memmove(s, p, strlen(p) + 1);
}

/* Map axis letter to index: X=0, Y=1, Z=2, A=3, B=4, C=5, -1 if unknown */
static int AxisIndex(const char *key)
{
    if (strlen(key) == 1) {
        switch (key[0]) {
        case 'X': case 'x': return 0;
        case 'Y': case 'y': return 1;
        case 'Z': case 'z': return 2;
        case 'A': case 'a': return 3;
        case 'B': case 'b': return 4;
        case 'C': case 'c': return 5;
        }
    }
    return -1;
}

/* ===================================================================
 * Load a single .pinmap file
 * =================================================================== */

bool PinMap_LoadFile(const char *filepath, PinMapEntry *entry)
{
    PinMap_Init(entry);

    FILE *f = nullptr;
    fopen_s(&f, filepath, "r");
    if (!f) return false;

    char section[32] = {0};
    char line[256];

    while (fgets(line, sizeof(line), f)) {
        TrimWhitespace(line);

        /* Skip empty lines and comments */
        if (line[0] == '\0' || line[0] == '#' || line[0] == ';')
            continue;

        /* Section header */
        if (line[0] == '[') {
            char *end = strchr(line, ']');
            if (end) {
                *end = '\0';
                strncpy_s(section, line + 1, sizeof(section) - 1);
            }
            continue;
        }

        /* Key = Value */
        char *eq = strchr(line, '=');
        if (!eq) continue;

        *eq = '\0';
        char *key = line;
        char *val = eq + 1;
        TrimWhitespace(key);
        TrimWhitespace(val);

        int pin = atoi(val);
        int ax = AxisIndex(key);

        if (_stricmp(section, "Board") == 0) {
            if (_stricmp(key, "Name") == 0)
                strncpy_s(entry->name, val, sizeof(entry->name) - 1);
            else if (_stricmp(key, "Description") == 0)
                strncpy_s(entry->description, val, sizeof(entry->description) - 1);
        }
        else if (_stricmp(section, "Step") == 0) {
            if (ax >= 0) entry->stepPin[ax] = (int8_t)pin;
        }
        else if (_stricmp(section, "Direction") == 0) {
            if (ax >= 0) entry->dirPin[ax] = (int8_t)pin;
        }
        else if (_stricmp(section, "Limits") == 0) {
            if (ax >= 0) entry->limitPin[ax] = (int8_t)pin;
        }
        else if (_stricmp(section, "Home") == 0) {
            if (ax >= 0) entry->homePin[ax] = (int8_t)pin;
        }
        else if (_stricmp(section, "Outputs") == 0) {
            if (_stricmp(key, "Enable") == 0)      entry->enablePin = (int8_t)pin;
            else if (_stricmp(key, "Spindle") == 0) entry->spindlePin = (int8_t)pin;
            else if (_stricmp(key, "ChargePump") == 0) entry->chargePumpPin = (int8_t)pin;
            else if (_stricmp(key, "MiscOut1") == 0)   entry->miscOutPin[0] = (int8_t)pin;
            else if (_stricmp(key, "MiscOut2") == 0)   entry->miscOutPin[1] = (int8_t)pin;
            else if (_stricmp(key, "LED") == 0)        entry->ledPin = (int8_t)pin;
        }
        else if (_stricmp(section, "Inputs") == 0) {
            if (_stricmp(key, "EStop") == 0)       entry->estopPin = (int8_t)pin;
            else if (_stricmp(key, "Probe") == 0)  entry->probePin = (int8_t)pin;
            else if (_stricmp(key, "MiscIn1") == 0)  entry->miscInPin[0] = (int8_t)pin;
            else if (_stricmp(key, "MiscIn2") == 0)  entry->miscInPin[1] = (int8_t)pin;
            else if (_stricmp(key, "MiscIn3") == 0)  entry->miscInPin[2] = (int8_t)pin;
            else if (_stricmp(key, "MiscIn4") == 0)  entry->miscInPin[3] = (int8_t)pin;
        }
    }

    fclose(f);
    return entry->name[0] != '\0';
}

/* ===================================================================
 * Scan folder for .pinmap files
 * =================================================================== */

int PinMap_ScanFolder(const char *folder,
                      char names[][PINMAP_MAX_NAME],
                      char paths[][260],
                      int maxEntries)
{
    int count = 0;

    char searchPath[260];
    snprintf(searchPath, sizeof(searchPath), "%s\\*.pinmap", folder);

    WIN32_FIND_DATAA fd;
    HANDLE hFind = FindFirstFileA(searchPath, &fd);
    if (hFind == INVALID_HANDLE_VALUE) return 0;

    do {
        if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) continue;
        if (count >= maxEntries) break;

        char fullPath[260];
        snprintf(fullPath, sizeof(fullPath), "%s\\%s", folder, fd.cFileName);

        PinMapEntry entry;
        if (PinMap_LoadFile(fullPath, &entry)) {
            strncpy_s(names[count], PINMAP_MAX_NAME, entry.name, _TRUNCATE);
            strncpy_s(paths[count], 260, fullPath, _TRUNCATE);
            count++;
        }
    } while (FindNextFileA(hFind, &fd));

    FindClose(hFind);
    return count;
}

/* ===================================================================
 * Load by board name
 * =================================================================== */

bool PinMap_LoadByName(const char *folder, const char *boardName,
                       PinMapEntry *entry)
{
    char names[PINMAP_MAX_BOARDS][PINMAP_MAX_NAME];
    char filePaths[PINMAP_MAX_BOARDS][260];

    int count = PinMap_ScanFolder(folder, names, filePaths, PINMAP_MAX_BOARDS);
    for (int i = 0; i < count; i++) {
        if (_stricmp(names[i], boardName) == 0)
            return PinMap_LoadFile(filePaths[i], entry);
    }

    PinMap_Init(entry);
    return false;
}
