/*
 * WiFi CNC Controller - Plugin Configuration (Registry Persistence)
 */

#include "PluginConfig.h"
#include <windows.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>

static const char *REG_KEY = "Software\\WiFiCNC";

/* Axis name prefixes for registry keys */
static const char *AXIS_NAMES[] = { "X", "Y", "Z", "A", "B", "C" };

/* ===================================================================
 * Defaults
 * =================================================================== */

void PluginConfig_LoadDefaults(PluginConfig *cfg)
{
    memset(cfg, 0, sizeof(PluginConfig));

    strncpy_s(cfg->espAddress, "192.168.4.1", sizeof(cfg->espAddress) - 1);

    for (int i = 0; i < WCNC_NUM_AXES; i++) {
        cfg->axis[i].enabled = (i < 3);    /* X, Y, Z enabled by default */
        cfg->axis[i].stepsPerUnit = 200.0;
        cfg->axis[i].maxVelUnitsPerMin = 1000.0;
        cfg->axis[i].accelUnitsPerSec2 = 100.0;
        cfg->axis[i].cloneMaster = -1;     /* not cloned */
        cfg->axis[i].cloneReversed = false;
    }

    cfg->stepPulseUs = 5;
    cfg->dirSetupUs  = 5;

    cfg->invertStep  = 0;
    cfg->invertDir   = 0;
    cfg->invertLimit = 0;
    cfg->invertHome  = 0;
    cfg->invertEstop = 0;
    cfg->invertProbe = 0;

    cfg->homingDirMask  = 0;
    cfg->homingSeekRate = 500;
    cfg->homingFeedRate = 50;
    cfg->homingPulloff  = 200;

    cfg->spindlePwmFreq = 1000;
    cfg->spindleMaxRpm  = 24000;

    cfg->chargePumpFreq  = 0;    /* disabled */
    cfg->stepIdleDelayMs = 0;    /* never */
    cfg->laserMode       = false;

    /* Input function mapping: all disabled by default */
    for (int i = 0; i < 4; i++)
        cfg->miscInputFunction[i] = 0;

    strncpy_s(cfg->boardName, "Tiggy Standard Motion Controller",
              sizeof(cfg->boardName) - 1);

    /* I/O module */
    cfg->ioModuleAddress[0] = '\0';
    cfg->ioModuleEnabled = false;
    for (int i = 0; i < 16; i++)
        cfg->ioInputFunction[i] = 0;
    cfg->ioJogSpeed = 5000;    /* 5000 steps/sec default pendant jog */
    for (int i = 0; i < 16; i++)
        cfg->ioOutputFunction[i] = 0;

    /* License */
    cfg->licenseEmail[0] = '\0';
    cfg->licenseKey[0] = '\0';
}

/* ===================================================================
 * Registry Helpers
 * =================================================================== */

static bool RegReadString(HKEY hKey, const char *name, char *buf, DWORD bufSize)
{
    DWORD type = 0, size = bufSize;
    return RegQueryValueExA(hKey, name, nullptr, &type, (LPBYTE)buf, &size) == ERROR_SUCCESS
           && type == REG_SZ;
}

static bool RegReadDWORD(HKEY hKey, const char *name, DWORD *val)
{
    DWORD type = 0, size = sizeof(DWORD);
    return RegQueryValueExA(hKey, name, nullptr, &type, (LPBYTE)val, &size) == ERROR_SUCCESS
           && type == REG_DWORD;
}

static bool RegReadDouble(HKEY hKey, const char *name, double *val)
{
    char buf[64];
    if (RegReadString(hKey, name, buf, sizeof(buf))) {
        *val = atof(buf);
        return true;
    }
    return false;
}

static void RegWriteString(HKEY hKey, const char *name, const char *val)
{
    RegSetValueExA(hKey, name, 0, REG_SZ, (const BYTE*)val, (DWORD)strlen(val) + 1);
}

static void RegWriteDWORD(HKEY hKey, const char *name, DWORD val)
{
    RegSetValueExA(hKey, name, 0, REG_DWORD, (const BYTE*)&val, sizeof(DWORD));
}

static void RegWriteDouble(HKEY hKey, const char *name, double val)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "%.6f", val);
    RegWriteString(hKey, name, buf);
}

/* ===================================================================
 * Load from Registry
 * =================================================================== */

bool PluginConfig_LoadFromRegistry(PluginConfig *cfg)
{
    /* Start with defaults, then overwrite with stored values */
    PluginConfig_LoadDefaults(cfg);

    HKEY hKey;
    if (RegOpenKeyExA(HKEY_CURRENT_USER, REG_KEY, 0, KEY_READ, &hKey) != ERROR_SUCCESS)
        return false;

    /* Connection */
    RegReadString(hKey, "ESPAddress", cfg->espAddress, sizeof(cfg->espAddress));

    /* Per-axis */
    for (int i = 0; i < WCNC_NUM_AXES; i++) {
        char key[64];
        DWORD dw;

        snprintf(key, sizeof(key), "%s_Enabled", AXIS_NAMES[i]);
        if (RegReadDWORD(hKey, key, &dw)) cfg->axis[i].enabled = (dw != 0);

        snprintf(key, sizeof(key), "%s_StepsPerUnit", AXIS_NAMES[i]);
        RegReadDouble(hKey, key, &cfg->axis[i].stepsPerUnit);

        snprintf(key, sizeof(key), "%s_VelPerMin", AXIS_NAMES[i]);
        RegReadDouble(hKey, key, &cfg->axis[i].maxVelUnitsPerMin);

        snprintf(key, sizeof(key), "%s_Accel", AXIS_NAMES[i]);
        RegReadDouble(hKey, key, &cfg->axis[i].accelUnitsPerSec2);

        snprintf(key, sizeof(key), "%s_CloneMaster", AXIS_NAMES[i]);
        if (RegReadDWORD(hKey, key, &dw)) cfg->axis[i].cloneMaster = (int)(int32_t)dw;

        snprintf(key, sizeof(key), "%s_CloneReversed", AXIS_NAMES[i]);
        if (RegReadDWORD(hKey, key, &dw)) cfg->axis[i].cloneReversed = (dw != 0);
    }

    /* Timing */
    DWORD dw;
    if (RegReadDWORD(hKey, "StepPulseUs", &dw)) cfg->stepPulseUs = (uint16_t)dw;
    if (RegReadDWORD(hKey, "DirSetupUs", &dw))  cfg->dirSetupUs  = (uint16_t)dw;

    /* Inversion */
    if (RegReadDWORD(hKey, "InvertStep", &dw))  cfg->invertStep  = (uint8_t)dw;
    if (RegReadDWORD(hKey, "InvertDir", &dw))   cfg->invertDir   = (uint8_t)dw;
    if (RegReadDWORD(hKey, "InvertLimit", &dw)) cfg->invertLimit = (uint8_t)dw;
    if (RegReadDWORD(hKey, "InvertHome", &dw))  cfg->invertHome  = (uint8_t)dw;
    if (RegReadDWORD(hKey, "InvertEstop", &dw)) cfg->invertEstop = (uint8_t)dw;
    if (RegReadDWORD(hKey, "InvertProbe", &dw)) cfg->invertProbe = (uint8_t)dw;

    /* Homing */
    if (RegReadDWORD(hKey, "HomingDirMask", &dw))  cfg->homingDirMask  = (uint8_t)dw;
    if (RegReadDWORD(hKey, "HomingSeekRate", &dw)) cfg->homingSeekRate = dw;
    if (RegReadDWORD(hKey, "HomingFeedRate", &dw)) cfg->homingFeedRate = dw;
    if (RegReadDWORD(hKey, "HomingPulloff", &dw))  cfg->homingPulloff  = dw;

    /* Spindle */
    if (RegReadDWORD(hKey, "SpindlePwmFreq", &dw)) cfg->spindlePwmFreq = (uint16_t)dw;
    if (RegReadDWORD(hKey, "SpindleMaxRpm", &dw))  cfg->spindleMaxRpm  = dw;

    /* Charge pump / step idle / laser */
    if (RegReadDWORD(hKey, "ChargePumpFreq", &dw))  cfg->chargePumpFreq  = (uint16_t)dw;
    if (RegReadDWORD(hKey, "StepIdleDelayMs", &dw)) cfg->stepIdleDelayMs = (uint16_t)dw;
    if (RegReadDWORD(hKey, "LaserMode", &dw))        cfg->laserMode       = (dw != 0);

    /* Input function mapping */
    for (int i = 0; i < 4; i++) {
        char key[64];
        snprintf(key, sizeof(key), "MiscInputFunc%d", i + 1);
        if (RegReadDWORD(hKey, key, &dw)) cfg->miscInputFunction[i] = (uint16_t)dw;
    }

    /* Board profile */
    RegReadString(hKey, "BoardName", cfg->boardName, sizeof(cfg->boardName));

    /* I/O module */
    RegReadString(hKey, "IOModuleAddress", cfg->ioModuleAddress, sizeof(cfg->ioModuleAddress));
    if (RegReadDWORD(hKey, "IOModuleEnabled", &dw)) cfg->ioModuleEnabled = (dw != 0);
    for (int i = 0; i < 16; i++) {
        char key[64];
        snprintf(key, sizeof(key), "IOInputFunc%d", i);
        if (RegReadDWORD(hKey, key, &dw)) cfg->ioInputFunction[i] = (uint16_t)dw;
    }
    if (RegReadDWORD(hKey, "IOJogSpeed", &dw)) cfg->ioJogSpeed = dw;
    for (int i = 0; i < 16; i++) {
        char key[64];
        snprintf(key, sizeof(key), "IOOutputFunc%d", i);
        if (RegReadDWORD(hKey, key, &dw)) cfg->ioOutputFunction[i] = (uint16_t)dw;
    }

    /* License */
    RegReadString(hKey, "LicenseEmail", cfg->licenseEmail, sizeof(cfg->licenseEmail));
    RegReadString(hKey, "LicenseKey", cfg->licenseKey, sizeof(cfg->licenseKey));

    RegCloseKey(hKey);
    return true;
}

/* ===================================================================
 * Save to Registry
 * =================================================================== */

void PluginConfig_SaveToRegistry(const PluginConfig *cfg)
{
    HKEY hKey;
    if (RegCreateKeyExA(HKEY_CURRENT_USER, REG_KEY, 0, nullptr,
                        REG_OPTION_NON_VOLATILE, KEY_WRITE,
                        nullptr, &hKey, nullptr) != ERROR_SUCCESS)
        return;

    /* Connection */
    RegWriteString(hKey, "ESPAddress", cfg->espAddress);

    /* Per-axis */
    for (int i = 0; i < WCNC_NUM_AXES; i++) {
        char key[64];

        snprintf(key, sizeof(key), "%s_Enabled", AXIS_NAMES[i]);
        RegWriteDWORD(hKey, key, cfg->axis[i].enabled ? 1 : 0);

        snprintf(key, sizeof(key), "%s_StepsPerUnit", AXIS_NAMES[i]);
        RegWriteDouble(hKey, key, cfg->axis[i].stepsPerUnit);

        snprintf(key, sizeof(key), "%s_VelPerMin", AXIS_NAMES[i]);
        RegWriteDouble(hKey, key, cfg->axis[i].maxVelUnitsPerMin);

        snprintf(key, sizeof(key), "%s_Accel", AXIS_NAMES[i]);
        RegWriteDouble(hKey, key, cfg->axis[i].accelUnitsPerSec2);

        snprintf(key, sizeof(key), "%s_CloneMaster", AXIS_NAMES[i]);
        RegWriteDWORD(hKey, key, (DWORD)(int32_t)cfg->axis[i].cloneMaster);

        snprintf(key, sizeof(key), "%s_CloneReversed", AXIS_NAMES[i]);
        RegWriteDWORD(hKey, key, cfg->axis[i].cloneReversed ? 1 : 0);
    }

    /* Timing */
    RegWriteDWORD(hKey, "StepPulseUs", cfg->stepPulseUs);
    RegWriteDWORD(hKey, "DirSetupUs",  cfg->dirSetupUs);

    /* Inversion */
    RegWriteDWORD(hKey, "InvertStep",  cfg->invertStep);
    RegWriteDWORD(hKey, "InvertDir",   cfg->invertDir);
    RegWriteDWORD(hKey, "InvertLimit", cfg->invertLimit);
    RegWriteDWORD(hKey, "InvertHome",  cfg->invertHome);
    RegWriteDWORD(hKey, "InvertEstop", cfg->invertEstop);
    RegWriteDWORD(hKey, "InvertProbe", cfg->invertProbe);

    /* Homing */
    RegWriteDWORD(hKey, "HomingDirMask",  cfg->homingDirMask);
    RegWriteDWORD(hKey, "HomingSeekRate", cfg->homingSeekRate);
    RegWriteDWORD(hKey, "HomingFeedRate", cfg->homingFeedRate);
    RegWriteDWORD(hKey, "HomingPulloff",  cfg->homingPulloff);

    /* Spindle */
    RegWriteDWORD(hKey, "SpindlePwmFreq", cfg->spindlePwmFreq);
    RegWriteDWORD(hKey, "SpindleMaxRpm",  cfg->spindleMaxRpm);

    /* Charge pump / step idle / laser */
    RegWriteDWORD(hKey, "ChargePumpFreq",  cfg->chargePumpFreq);
    RegWriteDWORD(hKey, "StepIdleDelayMs", cfg->stepIdleDelayMs);
    RegWriteDWORD(hKey, "LaserMode",       cfg->laserMode ? 1 : 0);

    /* Input function mapping */
    for (int i = 0; i < 4; i++) {
        char key[64];
        snprintf(key, sizeof(key), "MiscInputFunc%d", i + 1);
        RegWriteDWORD(hKey, key, cfg->miscInputFunction[i]);
    }

    /* Board profile */
    RegWriteString(hKey, "BoardName", cfg->boardName);

    /* I/O module */
    RegWriteString(hKey, "IOModuleAddress", cfg->ioModuleAddress);
    RegWriteDWORD(hKey, "IOModuleEnabled", cfg->ioModuleEnabled ? 1 : 0);
    for (int i = 0; i < 16; i++) {
        char key[64];
        snprintf(key, sizeof(key), "IOInputFunc%d", i);
        RegWriteDWORD(hKey, key, cfg->ioInputFunction[i]);
    }
    RegWriteDWORD(hKey, "IOJogSpeed", cfg->ioJogSpeed);
    for (int i = 0; i < 16; i++) {
        char key[64];
        snprintf(key, sizeof(key), "IOOutputFunc%d", i);
        RegWriteDWORD(hKey, key, cfg->ioOutputFunction[i]);
    }

    /* License */
    RegWriteString(hKey, "LicenseEmail", cfg->licenseEmail);
    RegWriteString(hKey, "LicenseKey", cfg->licenseKey);

    RegCloseKey(hKey);
}
