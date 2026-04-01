/*
 * WiFi CNC Controller - Mach3 Plugin Core Logic
 *
 * Based on reverse engineering of SmoothStepper_v17fe.dll.
 *
 * KEY RULES:
 *   1. NEVER write to Engine->Axis[].MaxVelocity/MasterVelocity/Acceleration
 *   2. All motion config (steps/unit, velocity, accel) stored in plugin registry
 *   3. Mach3's Motor Tuning panel is irrelevant — plugin owns its own config
 *   4. Position feedback via Engine->Axis[].Index and MasterIndex
 *   5. Monitor Engine spindle/coolant fields, forward to ESP32
 *
 * Data path:
 *   piUpdate() -> Engine->TrajBuffer -> SegmentBuilder -> BufferManager -> NetworkClient -> ESP32
 */

#include "Plugin.h"
#include "NetworkClient.h"
#include "SegmentBuilder.h"
#include "BufferManager.h"
#include "PluginConfig.h"
#include "ConfigDialog.h"
#include "License.h"
#include "../../protocol/wifi_cnc_protocol.h"

#include <cstdio>
#include <share.h>
#include <cstring>
#include <cstdlib>
#include <cstdarg>
#include <ctime>
#include <cstdint>
#include <cstddef>
#include <cmath>
#include <ws2tcpip.h>

static_assert(MAX_AXES == WCNC_MAX_AXES, "MAX_AXES and WCNC_MAX_AXES must match");

/* Lathe trajectory command codes (from SDK TrajectoryControl.h).
 * Defined here to avoid pulling in the full header which would
 * conflict with the EX_* notification enum already in Mach3Plugin.h. */
#ifndef STRAIGHTHREAD
#define STRAIGHTHREAD      125
#define SETCSSON           126
#define SETRPMON           127
#endif

/* ===================================================================
 * Debug Log
 * =================================================================== */

static FILE *g_logFile = nullptr;

static void LogOpen()
{
    if (g_logFile) return;

    char path[MAX_PATH];
    HMODULE hMod = nullptr;
    GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                       GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       (LPCSTR)&LogOpen, &hMod);
    GetModuleFileNameA(hMod, path, sizeof(path));

    char *dot = strrchr(path, '.');
    if (dot) *dot = '\0';
    strcat_s(path, sizeof(path), "_debug.log");

    g_logFile = _fsopen(path, "a", _SH_DENYWR);  /* shared read access */
    if (g_logFile) {
        fprintf(g_logFile, "\n========== Tiggy Motion Controller Plugin Started ==========\n");
        fflush(g_logFile);
    }
}

void Log(const char *fmt, ...)
{
    if (!g_logFile) return;

    time_t now = time(nullptr);
    struct tm t;
    localtime_s(&t, &now);
    fprintf(g_logFile, "[%02d:%02d:%02d] ", t.tm_hour, t.tm_min, t.tm_sec);

    va_list args;
    va_start(args, fmt);
    vfprintf(g_logFile, fmt, args);
    va_end(args);

    fprintf(g_logFile, "\n");
    fflush(g_logFile);
}

static void LogClose()
{
    if (g_logFile) {
        Log("Plugin shutting down");
        fclose(g_logFile);
        g_logFile = nullptr;
    }
}

/* MainPlanner: Mach3 internal object, provided by the host application */
extern void *MainPlanner;

/* MachView: CMach4View* — main Mach3 view object (for diagnostics) */
extern void *MachView;

/* ===================================================================
 * Per-Axis Configuration (derived from PluginConfig)
 * =================================================================== */

struct AxisConfig {
    double stepsPerUnit;        /* steps per user unit */
    double velUnitsPerSec;      /* max velocity in user units/sec */
    double accelUnitsPerSec2;   /* acceleration in user units/sec^2 */
    uint32_t maxStepsPerSec;    /* computed: stepsPerUnit * vel */
    uint32_t accelStepsPerSec2; /* computed: stepsPerUnit * accel */
    bool valid;
};

static AxisConfig g_axisConfig[MAX_AXES];

/* Master plugin configuration (source of truth for all settings) */
static PluginConfig g_pluginConfig;

/* MainPlanner offsets — from SDK TrajectoryControl.h lines 359-361.
 * StepsPerAxis, Velocities, Acceleration are contiguous double[6] arrays.
 * The base offset (0x8F498) is from SmoothStepper reverse engineering.
 *
 * Mach3 resets Motor Tuning velocity to 0 for external motion controllers
 * (m_PrinterOn=false), so the plugin MUST write these values itself.
 * Without this, Mach3's trajectory planner generates zero-length moves. */
#define MP_OFF_STEPS_PER_AXIS   0x8F498
#define MP_OFF_VELOCITIES       (MP_OFF_STEPS_PER_AXIS + 48)   /* 0x8F4C8 */
#define MP_OFF_ACCELERATION     (MP_OFF_STEPS_PER_AXIS + 96)   /* 0x8F4F8 */

/* ExternalType, ExTime, ExBufferHi — offsets found by reverse-engineering
 * the SmoothStepper_v17fe.dll plugin which is compiled against the same
 * Mach3 R3.043.066 TrajectoryControl class layout.
 *
 * ExternalType must be set to EX_VMS (1) for Mach3 to fill
 * Engine->Trajectories[] with per-period step counts.  Without this,
 * only 2 marker entries appear and the program stalls.
 *
 * ExTime is the cycle time (seconds per trajectory period).
 * ExBufferHi is the high-water mark for the movement buffer. */
#define MP_OFF_EXTERNAL_TYPE    0x246356   /* short */
#define MP_OFF_EXTIME           0x29E360   /* double */
#define MP_OFF_EXBUFFER_HI      0x29E368   /* int */

/* ExternalStill and ExternalPulseRates — verified against SmoothStepper DLL.
 * ExternalStill (bool): plugin sets true when idle, false when moving.
 *   Without this, Mach3 stops generating trajectory data after first move.
 * ExternalPulseRates (double[7]): max pulse rate per axis in steps/sec.
 *   Without this, Mach3 limits velocity to near zero (0-1 steps/period). */
#define MP_OFF_EXTERNAL_STILL       0x2461D0   /* bool */
#define MP_OFF_EXTERNAL_PULSE_RATES 0x246198   /* double[7] */

/*
 * Apply plugin config to the internal axis config used by motion code.
 * Also writes StepsPerAxis/Velocities/Acceleration into Mach3's MainPlanner
 * so the trajectory planner generates correct step counts (Mach3 resets
 * Motor Tuning velocity to 0 for external motion controllers).
 * Called after loading from registry or after dialog changes.
 */
static void ApplyPluginConfig()
{
    Log("Applying plugin config...");

    for (int i = 0; i < MAX_AXES; i++) {
        g_axisConfig[i].valid = g_pluginConfig.axis[i].enabled;
        g_axisConfig[i].stepsPerUnit = g_pluginConfig.axis[i].stepsPerUnit;
        g_axisConfig[i].velUnitsPerSec = g_pluginConfig.axis[i].maxVelUnitsPerMin / 60.0;
        g_axisConfig[i].accelUnitsPerSec2 = g_pluginConfig.axis[i].accelUnitsPerSec2;
        g_axisConfig[i].maxStepsPerSec = 0;
        g_axisConfig[i].accelStepsPerSec2 = 0;

        if (g_axisConfig[i].valid && g_axisConfig[i].stepsPerUnit > 0.0) {
            g_axisConfig[i].maxStepsPerSec = (uint32_t)(
                g_axisConfig[i].stepsPerUnit * g_axisConfig[i].velUnitsPerSec);
            g_axisConfig[i].accelStepsPerSec2 = (uint32_t)(
                g_axisConfig[i].stepsPerUnit * g_axisConfig[i].accelUnitsPerSec2);

            Log("  Axis %d: %.4f spu, %.1f u/min, %.1f u/s^2 -> %u steps/sec, %u accel",
                i, g_axisConfig[i].stepsPerUnit,
                g_pluginConfig.axis[i].maxVelUnitsPerMin,
                g_axisConfig[i].accelUnitsPerSec2,
                g_axisConfig[i].maxStepsPerSec,
                g_axisConfig[i].accelStepsPerSec2);
        }

        if (g_pluginConfig.axis[i].cloneMaster >= 0 &&
            g_pluginConfig.axis[i].cloneMaster < WCNC_NUM_AXES) {
            Log("  Axis %d: cloned from axis %d%s", i,
                g_pluginConfig.axis[i].cloneMaster,
                g_pluginConfig.axis[i].cloneReversed ? " (reversed)" : "");
        }
    }

    /* Pre-populate Engine signals with Port=1 and sequential Pin numbers
     * so they appear in Mach3's Config > Ports and Pins dialog.
     * Users enable/disable signals and set inversion (Negated) there.
     *
     * Port 1 = ESP32 WiFi motion controller (virtual mapping).
     * We do NOT force Active or Negated — those are the user's to configure
     * in Mach3 Ports & Pins.  Safety-critical signals (E-stop, probe) get
     * Active=true as a fallback if not already configured. */
    if (Engine) {
        /* Axis enables from our config */
        for (int i = 0; i < MAX_AXES; i++) {
            Engine->Axis[i].Enable = g_pluginConfig.axis[i].enabled;
        }

        /* Input signals: pre-populate port/pin so they show in Ports & Pins.
         * Pin mapping: 1-18 = limit/home (3 per axis), 19-22 = ACTIVATION1-4,
         * 23 = probe (DIGITIZE), 24 = E-stop (EMERGENCY), 25 = INDEX */
        for (int i = 0; i <= CHOME; i++) {
            Engine->InSigs[i].InPort = 1;
            Engine->InSigs[i].InPin  = (char)(i + 1);
        }
        for (int i = 0; i < 4; i++) {
            Engine->InSigs[ACTIVATION1 + i].InPort = 1;
            Engine->InSigs[ACTIVATION1 + i].InPin  = (char)(19 + i);
        }
        Engine->InSigs[DIGITIZE].InPort  = 1;  Engine->InSigs[DIGITIZE].InPin  = 23;
        Engine->InSigs[EMERGENCY].InPort = 1;  Engine->InSigs[EMERGENCY].InPin = 24;
        Engine->InSigs[INDEX].InPort     = 1;  Engine->InSigs[INDEX].InPin     = 25;

        /* Safety-critical signals: ensure Active even if user hasn't
         * configured Ports & Pins yet (defensive fallback) */
        Engine->InSigs[EMERGENCY].Active = true;
        Engine->InSigs[DIGITIZE].Active  = true;

        /* If user hasn't configured limits in Ports & Pins yet,
         * activate them for enabled axes as a safe default */
        for (int i = 0; i < MAX_AXES; i++) {
            if (!g_pluginConfig.axis[i].enabled) continue;
            int base = i * 3;
            if (!Engine->InSigs[XLIMITPLUS + base].Active) {
                Engine->InSigs[XLIMITPLUS  + base].Active = true;
                Engine->InSigs[XLIMITMINUS + base].Active = true;
                Engine->InSigs[XHOME       + base].Active = true;
            }
        }

        /* Output signals: pre-populate port/pin for Ports & Pins visibility.
         * Mach3 requires active=true AND non-zero OutPort/OutPin to
         * process M62-M65 commands. */
        Engine->OutSigs[ENABLE1].OutPort = 1;  Engine->OutSigs[ENABLE1].OutPin = 1;
        Engine->OutSigs[ENABLE1].active  = true;
        Engine->OutSigs[CHARGE].OutPort  = 1;  Engine->OutSigs[CHARGE].OutPin  = 2;
        Engine->OutSigs[CHARGE].active   = true;

        Log("  Input signals pre-populated for Ports & Pins (Port 1, Pin 1-25)");
        Log("  Output signals: ENABLE1=Port1:Pin1 CHARGE=Port1:Pin2");

        /* Laser mode pass-through to Engine */
        Engine->LaserMode = g_pluginConfig.laserMode;

        /* Tell Mach3 spindle is always stable (ESP32 handles spindle timing). */
        Engine->SpindleStable = true;

        Log("  LaserMode=%d, SpindleStable=1", (int)g_pluginConfig.laserMode);
    }

    /* Write motor tuning values to MainPlanner.
     * Mach3 resets these to 0 for external motion controllers, so we
     * must set them ourselves for the trajectory planner to work. */
    if (MainPlanner) {
        char *mp = (char *)MainPlanner;
        bool writeOk = true;

        __try {
            for (int i = 0; i < MAX_AXES; i++) {
                if (!g_axisConfig[i].valid) continue;

                double *pSteps = (double *)(mp + MP_OFF_STEPS_PER_AXIS + i * sizeof(double));
                double *pVel   = (double *)(mp + MP_OFF_VELOCITIES + i * sizeof(double));
                double *pAccel = (double *)(mp + MP_OFF_ACCELERATION + i * sizeof(double));

                *pSteps = g_axisConfig[i].stepsPerUnit;
                *pVel   = g_axisConfig[i].velUnitsPerSec;  /* units/sec (SDK convention) */
                *pAccel = g_axisConfig[i].accelUnitsPerSec2;
            }
        } __except(EXCEPTION_EXECUTE_HANDLER) {
            Log("WARNING: MainPlanner write caused access violation!");
            writeOk = false;
        }

        /* Verify by read-back */
        if (writeOk) {
            __try {
                Log("  MainPlanner StepsPerAxis: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
                    *(double *)(mp + MP_OFF_STEPS_PER_AXIS + 0),
                    *(double *)(mp + MP_OFF_STEPS_PER_AXIS + 8),
                    *(double *)(mp + MP_OFF_STEPS_PER_AXIS + 16),
                    *(double *)(mp + MP_OFF_STEPS_PER_AXIS + 24),
                    *(double *)(mp + MP_OFF_STEPS_PER_AXIS + 32),
                    *(double *)(mp + MP_OFF_STEPS_PER_AXIS + 40));
                Log("  MainPlanner Velocities:   [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
                    *(double *)(mp + MP_OFF_VELOCITIES + 0),
                    *(double *)(mp + MP_OFF_VELOCITIES + 8),
                    *(double *)(mp + MP_OFF_VELOCITIES + 16),
                    *(double *)(mp + MP_OFF_VELOCITIES + 24),
                    *(double *)(mp + MP_OFF_VELOCITIES + 32),
                    *(double *)(mp + MP_OFF_VELOCITIES + 40));
                Log("  MainPlanner Acceleration: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
                    *(double *)(mp + MP_OFF_ACCELERATION + 0),
                    *(double *)(mp + MP_OFF_ACCELERATION + 8),
                    *(double *)(mp + MP_OFF_ACCELERATION + 16),
                    *(double *)(mp + MP_OFF_ACCELERATION + 24),
                    *(double *)(mp + MP_OFF_ACCELERATION + 32),
                    *(double *)(mp + MP_OFF_ACCELERATION + 40));
            } __except(EXCEPTION_EXECUTE_HANDLER) {
                Log("WARNING: MainPlanner verify read caused access violation!");
            }
        }
        /* Set ExternalType = EX_VMS so Mach3 fills Engine->Trajectories[]
         * with per-period step counts.  Also set ExTime (cycle period) and
         * ExBufferHi (buffer high-water mark), matching ncPod/Galil SDK. */
        __try {
            short *pExtType   = (short *)(mp + MP_OFF_EXTERNAL_TYPE);
            double *pExTime   = (double *)(mp + MP_OFF_EXTIME);
            int    *pBufHi    = (int *)   (mp + MP_OFF_EXBUFFER_HI);

            *pExtType = EX_VMS;   /* 1 = Vector Move System mode */
            *pExTime  = 0.001;    /* 1 ms cycle time */
            *pBufHi   = 350;      /* high-water mark (ncPod/Galil default) */

            /* Verify */
            Log("  MainPlanner ExternalType = %d (EX_VMS=%d)", (int)*pExtType, (int)EX_VMS);
            Log("  MainPlanner ExTime = %.6f s", *pExTime);
            Log("  MainPlanner ExBufferHi = %d", *pBufHi);
        } __except(EXCEPTION_EXECUTE_HANDLER) {
            Log("WARNING: MainPlanner ExternalType write caused access violation!");
        }

        /* Set ExternalPulseRates — max steps/sec per axis.
         * Without this, Mach3 limits trajectory velocity to near zero
         * (produces only 0-1 steps per period instead of expected ~17).
         * ncPod sets all to 75000.  We use the plugin's configured value. */
        __try {
            double *pRates = (double *)(mp + MP_OFF_EXTERNAL_PULSE_RATES);
            for (int i = 0; i < 7; i++) {
                double rate = 0.0;
                if (i < MAX_AXES && g_axisConfig[i].valid)
                    rate = (double)g_axisConfig[i].maxStepsPerSec;
                if (rate < 1.0)
                    rate = 75000.0;  /* fallback (ncPod default) */
                pRates[i] = rate;
            }
            Log("  MainPlanner ExternalPulseRates: [%.0f, %.0f, %.0f, %.0f, %.0f, %.0f, %.0f]",
                pRates[0], pRates[1], pRates[2], pRates[3],
                pRates[4], pRates[5], pRates[6]);
        } __except(EXCEPTION_EXECUTE_HANDLER) {
            Log("WARNING: MainPlanner ExternalPulseRates write caused access violation!");
        }

        /* Set ExternalStill = true initially (controller is idle at startup).
         * Managed dynamically in piUpdateImpl() during trajectory processing. */
        __try {
            bool *pStill = (bool *)(mp + MP_OFF_EXTERNAL_STILL);
            *pStill = true;
            Log("  MainPlanner ExternalStill = true (initial)");
        } __except(EXCEPTION_EXECUTE_HANDLER) {
            Log("WARNING: MainPlanner ExternalStill write caused access violation!");
        }
    } else {
        Log("  MainPlanner is NULL - cannot write motor tuning values");
    }
}

/* ===================================================================
 * Plugin State
 * =================================================================== */

static NetworkClient  *g_network    = nullptr;
static SegmentBuilder *g_segBuilder = nullptr;
static BufferManager  *g_bufferMgr  = nullptr;
static bool            g_connected  = false;
static bool            g_proLicense = false;
static bool            g_enabled    = false;
static int             g_lastTrajIndex = 0;

/* Spindle encoder feedback */
static uint16_t        g_lastSpindleRPM = 0;
static uint32_t        g_lastIndexCount = 0;   /* Edge detection for index pulse */

/* I/O expansion module (second ESP32 in I/O module mode) */
static NetworkClient  *g_ioModule   = nullptr;
static bool            g_ioConnected = false;
static uint16_t        g_prevIOInputs = 0;
static bool            g_ioJogActive[MAX_AXES] = {false};

/* I/O module input function codes for pendant jog */
#define IO_FUNC_NONE      0
#define IO_FUNC_JOG_XP  101
#define IO_FUNC_JOG_XN  102
#define IO_FUNC_JOG_YP  103
#define IO_FUNC_JOG_YN  104
#define IO_FUNC_JOG_ZP  105
#define IO_FUNC_JOG_ZN  106
#define IO_FUNC_JOG_AP  107
#define IO_FUNC_JOG_AN  108
#define IO_FUNC_JOG_BP  109
#define IO_FUNC_JOG_BN  110
#define IO_FUNC_JOG_CP  111
#define IO_FUNC_JOG_CN  112

/* VMS trajectory accumulator (used by Phase 9) */
static int      g_accumSteps[6] = {0,0,0,0,0,0};
static int      g_accumCount    = 0;
static int      g_accumLineId   = -1;
static uint16_t g_segId         = 0;
static int      g_trajLogCount  = 0;
static int      g_segLogCount   = 0;

void ResetTrajAccum() {
    memset(g_accumSteps, 0, sizeof(g_accumSteps));
    g_accumCount  = 0;
    g_accumLineId = -1;
    g_trajLogCount = 0;
    g_segLogCount  = 0;
}

/* Spindle/coolant: Engine->SpindleCW/CCW/Flood/Mist are garbage (non-boolean).
 * We don't poll them.  All spindle/coolant IO is driven by EX_SPINON/OFF
 * notifications and TrajCmd handlers.  These prev vars are kept only
 * for TrajCmd coolant handlers that need to sync. */

/* E-stop state tracking (like SmoothStepper Phase 2) */
static bool g_prevEStop = false;
static int32_t g_savedPos[MAX_AXES] = {0};

/* Position offset tracking.
 *
 * The ESP32 reports absolute accumulated step positions from power-on.
 * Mach3 expects Engine->Axis[i].Index to reflect its own coordinate
 * system.  We compute a fixed offset on first status (or after Reset)
 * that maps ESP32 position into Mach3 coordinates.
 *
 *   g_posOffset[i] = Mach3_Index - ESP32_steps   (computed once)
 *   Engine->Axis[i].Index = ESP32_steps + g_posOffset[i]
 *
 * Following the Galil/ncPod SDK pattern, we write Index directly from
 * hardware position with NO per-cycle offset tracking.  DRO zeroing in
 * Mach3 changes G54 work offsets, not Engine->Axis[i].Index, so we
 * don't need to detect external Index changes.  The offset is only
 * recalculated on Reset (g_posInitialized = false).
 */
static int32_t g_posOffset[MAX_AXES] = {0};
static bool    g_posInitialized = false;

/* Cumulative step tracking: compare trajectory steps vs ESP32 position
 * to diagnose toolpath drift.  g_trajCumSteps tracks the total steps
 * we read from Engine->Trajectories[] per axis.  Any discrepancy with
 * ESP32's reported position indicates step loss. */
static int32_t g_trajCumSteps[MAX_AXES] = {0};

/* Misc output / charge pump state tracking (like SS Phase 3/11) */
static uint8_t g_prevMiscOutputs = 0;
static bool    g_prevChargePump  = false;

/* Misc input function mapping: previous state for edge detection.
 * We trigger DoButton() on rising edge only (0->1 transition). */
static bool g_prevMiscInput[4] = { false, false, false, false };

/* Flag: plugin initiated E-Stop via piResetImpl().
 * Suppresses DoButton(OEM_ESTOP) in status handler to prevent feedback loop.
 * Cleared when ESP32 returns to IDLE. */
static bool g_pluginResetActive = false;

/* Homing: sequential queue with slaved axis support.
 *
 * Mach3 calls piHome() separately for each axis within the same millisecond.
 * The ESP32 firmware can only home one axis (or axis group) at a time.
 * We collect all piHome calls during a 50ms window, then send them
 * ONE AT A TIME, waiting for HOMING→IDLE before sending the next.
 *
 * State machine:
 *   HOME_IDLE       → piHome() → HOME_COLLECTING
 *   HOME_COLLECTING → 50ms elapsed → send first axis → HOME_SENT
 *   HOME_SENT       → ESP32 state==HOMING → HOME_ACTIVE
 *   HOME_ACTIVE     → ESP32 state!=HOMING → mark Referenced,
 *                      if more axes → send next → HOME_SENT
 *                      else → HOME_IDLE
 */
enum HomeState { HOME_IDLE, HOME_COLLECTING, HOME_SENT, HOME_ACTIVE };
static HomeState g_homeState = HOME_IDLE;

static int     g_homeQueue[MAX_AXES];    /* axis indices, in piHome call order */
static int     g_homeQueueLen = 0;       /* total axes in queue */
static int     g_homeQueueIdx = 0;       /* next axis to send */
static uint8_t g_activeHomeMask = 0;     /* mask currently being homed (incl. slaves) */
static DWORD   g_homeCollectTick = 0;    /* when we started collecting */
static DWORD   g_homeSentTick = 0;       /* when we sent the last home command */
static DWORD   g_homeActiveTick = 0;     /* when we entered HOME_ACTIVE */
static uint8_t g_prevMachineState = 0xFF;

/* Send the next axis from the homing queue to the ESP32.
 * Includes any slaved axes in the same mask. */
static void HomeSendNext()
{
    if (g_homeQueueIdx >= g_homeQueueLen) {
        /* Queue exhausted */
        g_homeState = HOME_IDLE;
        g_homeQueueLen = 0;
        g_homeQueueIdx = 0;
        Log("Home: queue complete, all axes done");
        return;
    }

    int axis = g_homeQueue[g_homeQueueIdx];
    uint8_t mask = (1 << axis);

    /* Include any axis that is slaved to this one (check both Mach3 and plugin config) */
    for (int i = 0; i < MAX_AXES; i++) {
        if (i == axis) continue;
        bool isSlave = (Engine->Axis[i].Slave &&
                        Engine->Axis[i].SlaveAxis == axis && Engine->Axis[i].Enable);
        if (!isSlave && g_pluginConfig.axis[i].cloneMaster == axis &&
            g_pluginConfig.axis[i].enabled)
            isSlave = true;
        if (isSlave) {
            mask |= (1 << i);
            Engine->Axis[i].Homing = true;
            Log("Home: axis %d is cloned to %d, adding to mask", i, axis);
        }
    }

    g_activeHomeMask = mask;
    g_homeQueueIdx++;
    g_homeSentTick = GetTickCount();
    g_homeState = HOME_SENT;

    Log("Home: sending axis %d (mask=0x%02X) [%d/%d]",
        axis, mask, g_homeQueueIdx, g_homeQueueLen);
    g_network->SendHomeCommand(mask);
}

/* ===================================================================
 * IO Control - Send spindle/coolant state to ESP32
 * =================================================================== */

/* Tracked output state: what we last sent to ESP32 (ground truth).
 * Used by ConfigDialog for output LEDs instead of Engine fields which
 * can have garbage values at startup. */
static uint8_t g_sentSpindle = 0;   /* 0=off, 1=CW, 2=CCW */
static uint8_t g_sentCoolant = 0;   /* bit0=flood, bit1=mist */
static uint8_t g_sentMiscOut = 0;   /* bit mask of misc outputs */

/* Exported getters for ConfigDialog */
uint8_t GetSentSpindleState() { return g_sentSpindle; }
uint8_t GetSentCoolantState() { return g_sentCoolant; }
uint8_t GetSentMiscOutputs()  { return g_sentMiscOut; }
uint16_t GetSpindleRPM()      { return g_lastSpindleRPM; }
bool IsIOModuleConnected()    { return g_ioConnected; }

static void SendIOControl(uint8_t miscOutputs, uint8_t spindleState,
                           uint16_t spindleRpm, uint8_t coolantState)
{
    /* Always track state, even if not connected */
    g_sentSpindle = spindleState;
    g_sentCoolant = coolantState;
    g_sentMiscOut = miscOutputs;

    if (!g_network || !g_connected) return;

    wcnc_io_control_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.misc_outputs = miscOutputs;
    pkt.spindle_state = spindleState;
    pkt.spindle_rpm = spindleRpm;
    pkt.coolant_state = coolantState;

    wcnc_finalize_packet(&pkt, WCNC_PKT_IO_CONTROL,
                          sizeof(pkt) - sizeof(wcnc_header_t), 0, 0);

    /* Send via UDP for real-time */
    g_network->SendIOControlPacket(&pkt);
}

/* ===================================================================
 * Read inversion settings from Mach3 Engine structs
 *
 * Reads Negated fields from Ports & Pins and StepNegate/DirNegate from
 * Motor Outputs, then builds the bitmask config to send to ESP32.
 * This way users configure inversion in Mach3's native dialogs.
 * =================================================================== */

#ifndef LEGACY_INVERSION
static void ReadInversionFromMach3()
{
    if (!Engine) return;

    /* Step/Dir inversion from Axis[i].StepNegate/DirNegate
     * (set in Mach3 Config > Ports and Pins > Motor Outputs) */
    uint8_t invertStep = 0, invertDir = 0;
    for (int i = 0; i < MAX_AXES; i++) {
        if (Engine->Axis[i].StepNegate) invertStep |= (1 << i);
        if (Engine->Axis[i].DirNegate)  invertDir  |= (1 << i);
    }
    g_pluginConfig.invertStep = invertStep;
    g_pluginConfig.invertDir  = invertDir;

    /* Limit/Home inversion from InSigs[].Negated
     * (set in Mach3 Config > Ports and Pins > Input Signals) */
    uint8_t invertLimit = 0, invertHome = 0;
    for (int i = 0; i < MAX_AXES; i++) {
        if (Engine->InSigs[XLIMITPLUS + i * 3].Negated)
            invertLimit |= (1 << i);
        if (Engine->InSigs[XHOME + i * 3].Negated)
            invertHome |= (1 << i);
    }
    g_pluginConfig.invertLimit = invertLimit;
    g_pluginConfig.invertHome  = invertHome;

    /* E-stop and Probe inversion from Negated */
    g_pluginConfig.invertEstop = Engine->InSigs[EMERGENCY].Negated ? 1 : 0;
    g_pluginConfig.invertProbe = Engine->InSigs[DIGITIZE].Negated ? 1 : 0;

    Log("Inversion from Mach3 Ports & Pins: step=0x%02X dir=0x%02X limit=0x%02X home=0x%02X estop=%d probe=%d",
        invertStep, invertDir, invertLimit, invertHome,
        (int)g_pluginConfig.invertEstop, (int)g_pluginConfig.invertProbe);
}
#endif

/* ===================================================================
 * Configuration Sync (Mach3 -> ESP32)
 * =================================================================== */

static void SyncConfigToESP32()
{
    if (!g_network || !g_connected) return;

#ifndef LEGACY_INVERSION
    ReadInversionFromMach3();
#endif

    Log("Syncing config to controller...");

    /* Timing (from plugin config, not Engine) */
    uint16_t pulse_us = g_pluginConfig.stepPulseUs;
    g_network->SendConfig(WCNC_CFG_STEP_PULSE_US, &pulse_us, WCNC_VAL_UINT16);

    uint16_t dir_us = g_pluginConfig.dirSetupUs;
    g_network->SendConfig(WCNC_CFG_DIR_SETUP_US, &dir_us, WCNC_VAL_UINT16);

    /* Inversion flags (from plugin config).
     *
     * Home & limit share the same physical GPIO pins.  The firmware's homing
     * cycle uses gpio_control_get_limit() which reads inv_lim, NOT inv_home.
     * Combine both masks so ticking either "Limit Invert" or "Home Invert"
     * for an axis correctly inverts the pin for homing AND runtime limits.
     *
     * Small delays between packets prevent TCP flooding that causes
     * connection drops during config sync (observed in debug logs). */
    g_network->SendConfig(WCNC_CFG_INVERT_STEP, &g_pluginConfig.invertStep, WCNC_VAL_UINT8);
    Sleep(5);
    g_network->SendConfig(WCNC_CFG_INVERT_DIR, &g_pluginConfig.invertDir, WCNC_VAL_UINT8);
    Sleep(5);

    uint8_t combinedInvert = g_pluginConfig.invertLimit | g_pluginConfig.invertHome;
    g_network->SendConfig(WCNC_CFG_INVERT_LIMIT, &combinedInvert, WCNC_VAL_UINT8);
    Sleep(5);
    g_network->SendConfig(WCNC_CFG_INVERT_HOME, &combinedInvert, WCNC_VAL_UINT8);
    Sleep(5);

    g_network->SendConfig(WCNC_CFG_INVERT_ESTOP, &g_pluginConfig.invertEstop, WCNC_VAL_UINT8);
    Sleep(5);
    g_network->SendConfig(WCNC_CFG_INVERT_PROBE, &g_pluginConfig.invertProbe, WCNC_VAL_UINT8);
    Sleep(5);

    Log("  Sent inversion: step=0x%02X dir=0x%02X limit/home=0x%02X estop=%d probe=%d",
        g_pluginConfig.invertStep, g_pluginConfig.invertDir, combinedInvert,
        (int)g_pluginConfig.invertEstop, (int)g_pluginConfig.invertProbe);

    /* Per-axis config (with small delays to prevent TCP flooding) */
    for (int i = 0; i < MAX_AXES; i++) {
        if (!g_axisConfig[i].valid) continue;

        uint32_t maxRate = g_axisConfig[i].maxStepsPerSec;
        uint32_t accel = g_axisConfig[i].accelStepsPerSec2;

        g_network->SendConfig((uint16_t)(WCNC_CFG_MAX_RATE_X + i),
                               &maxRate, WCNC_VAL_UINT32);
        g_network->SendConfig((uint16_t)(WCNC_CFG_ACCEL_X + i),
                               &accel, WCNC_VAL_UINT32);

        float stepsPerUnit = (float)g_axisConfig[i].stepsPerUnit;
        g_network->SendConfig((uint16_t)(WCNC_CFG_STEPS_PER_MM_X + i),
                               &stepsPerUnit, WCNC_VAL_FLOAT);
        Sleep(5);

        Log("  Sync axis %d: maxRate=%u, accel=%u, steps/unit=%.4f",
            i, maxRate, accel, g_axisConfig[i].stepsPerUnit);
    }

    /* Homing config */
    g_network->SendConfig(WCNC_CFG_HOMING_DIR_MASK, &g_pluginConfig.homingDirMask, WCNC_VAL_UINT8);
    g_network->SendConfig(WCNC_CFG_HOMING_SEEK_RATE, &g_pluginConfig.homingSeekRate, WCNC_VAL_UINT32);
    g_network->SendConfig(WCNC_CFG_HOMING_FEED_RATE, &g_pluginConfig.homingFeedRate, WCNC_VAL_UINT32);
    g_network->SendConfig(WCNC_CFG_HOMING_PULLOFF, &g_pluginConfig.homingPulloff, WCNC_VAL_UINT32);
    Sleep(5);

    Log("  Sent homing: dirMask=0x%02X seek=%u feed=%u pulloff=%u",
        g_pluginConfig.homingDirMask, g_pluginConfig.homingSeekRate,
        g_pluginConfig.homingFeedRate, g_pluginConfig.homingPulloff);

    /* Spindle */
    g_network->SendConfig(WCNC_CFG_SPINDLE_PWM_FREQ, &g_pluginConfig.spindlePwmFreq, WCNC_VAL_UINT16);
    g_network->SendConfig(WCNC_CFG_SPINDLE_MAX_RPM, &g_pluginConfig.spindleMaxRpm, WCNC_VAL_UINT32);
    Sleep(5);

    /* Charge pump */
    g_network->SendConfig(WCNC_CFG_CHARGE_PUMP_FREQ, &g_pluginConfig.chargePumpFreq, WCNC_VAL_UINT16);

    /* Step idle delay */
    g_network->SendConfig(WCNC_CFG_STEP_IDLE_DELAY_MS, &g_pluginConfig.stepIdleDelayMs, WCNC_VAL_UINT16);
    Sleep(5);

    g_network->SendConfigSave();
    Log("Config sync complete.");
}

/* ===================================================================
 * Plugin Lifecycle
 * =================================================================== */

void piInitControlImpl()
{
    LogOpen();
    Log("piInitControl() called");

    g_network    = new NetworkClient();
    g_network->SetLogCallback(Log);
    g_segBuilder = new SegmentBuilder();
    g_bufferMgr  = new BufferManager(g_network);
    g_ioModule   = new NetworkClient();
    g_ioModule->SetLogCallback(Log);

    memset(g_axisConfig, 0, sizeof(g_axisConfig));

    /* Load all plugin settings from registry (or defaults on first run) */
    if (!PluginConfig_LoadFromRegistry(&g_pluginConfig)) {
        Log("First run: using default config");
    }
    ApplyPluginConfig();
    Log("Settings loaded, controller address: %s", g_pluginConfig.espAddress);
}

void piSetProNameImpl(char *name)
{
    strcpy_s(name, 64, "Tiggy Motion Controller");
}

static bool TryConnect(const char *ip)
{
    Log("TryConnect(%s) on port %d...", ip, WCNC_TCP_CONTROL_PORT);

    if (g_network->Connect(ip, WCNC_TCP_CONTROL_PORT)) {
        Log("TCP connected, attempting handshake...");
        g_connected = true;
        if (g_network->Handshake()) {
            const wcnc_handshake_resp_t *resp = g_network->GetHandshakeResp();
            Log("Handshake OK! FW v%u.%u.%u, %d axes, buffer=%u, maxRate=%u",
                (resp->firmware_version >> 24) & 0xFF,
                (resp->firmware_version >> 16) & 0xFF,
                resp->firmware_version & 0xFFFF,
                resp->num_axes, resp->buffer_capacity,
                resp->max_step_rate);

            if (resp->buffer_capacity > 0) {
                g_bufferMgr->SetRemoteCapacity(resp->buffer_capacity);
            } else {
                g_bufferMgr->SetRemoteCapacity(64);
            }

            /* Tell Mach3 the encoder resolution for threading sync */
            if (resp->capabilities & WCNC_CAP_SPINDLE_ENCODER) {
                int ppr = (resp->encoder_ppr_hi << 8) | resp->encoder_ppr_lo;
                if (ppr > 0) {
                    Engine->IntsPerRotation = ppr;
                    Log("Spindle encoder: %d PPR", ppr);
                }
            }

            SyncConfigToESP32();
            g_network->StartUDP(ip);

            /* Send RESET to clear stale state (like SS::Reset) */
            g_network->SendReset();
            Sleep(200);

            /* Force all outputs OFF so firmware and plugin start in sync.
             * Prevents spindle LED showing ON at startup from stale state. */
            SendIOControl(0, 0, 0, 0);

            g_enabled = true;
            strncpy_s(g_pluginConfig.espAddress, ip, sizeof(g_pluginConfig.espAddress) - 1);
            PluginConfig_SaveToRegistry(&g_pluginConfig);
            Log("Connected and enabled. IP: %s", g_pluginConfig.espAddress);
            return true;
        }
        Log("Handshake FAILED");
        g_network->Disconnect();
        g_connected = false;
    } else {
        Log("TCP connect FAILED for %s", ip);
    }
    return false;
}

/* ===================================================================
 * Main Menu Integration
 *
 * Adds a "WiFi CNC" entry to Mach3's menu bar that opens our config
 * dialog.  Subclasses Mach3's main window to intercept the menu click.
 * =================================================================== */

/* Forward declaration — piConfigImpl defined later */
void piConfigImpl();

#define IDM_WIFICNC_SETTINGS  40199

static HWND    g_mach3MainWnd      = nullptr;
static WNDPROC g_origMach3WndProc  = nullptr;

static LRESULT CALLBACK Mach3MenuSubclassProc(HWND hWnd, UINT msg,
                                               WPARAM wParam, LPARAM lParam)
{
    if (msg == WM_COMMAND && LOWORD(wParam) == IDM_WIFICNC_SETTINGS) {
        piConfigImpl();
        return 0;
    }
    return CallWindowProcA(g_origMach3WndProc, hWnd, msg, wParam, lParam);
}

void piPostInitControlImpl()
{
    /* Access diagnostics from MachDevice.cpp */
    extern int g_printerOnOffset;
    extern ptrdiff_t g_plannerOffset;

    Log("piPostInitControl() called, Engine=%p, MainPlanner=%p, MachView=%p",
        (void*)Engine, MainPlanner, MachView);

    if (!Engine) {
        Log("ERROR: Engine pointer is NULL!");
        return;
    }

    /* License check */
    g_proLicense = License_CheckKey(g_pluginConfig.licenseEmail,
                                     g_pluginConfig.licenseKey);
    Log("License: %s (%s)", License_TierName(g_proLicense),
        g_proLicense ? g_pluginConfig.licenseEmail : "no key");

    if (!g_proLicense) {
        /* Free tier: disable axes 4-6 (A/B/C) */
        for (int i = 3; i < WCNC_NUM_AXES; i++) {
            g_pluginConfig.axis[i].enabled = false;
        }
        /* Disable I/O expansion module */
        g_pluginConfig.ioModuleEnabled = false;
    }

    /* Log m_PrinterOn scan results */
    Log("MachView->Planner offset = 0x%X (%d bytes)",
        (unsigned)g_plannerOffset, (int)g_plannerOffset);
    if (g_printerOnOffset >= 0) {
        Log("m_PrinterOn found and cleared at MachView+0x%X (Planner-0x%X)",
            g_printerOnOffset,
            (unsigned)(g_plannerOffset - g_printerOnOffset));
    } else {
        Log("WARNING: m_PrinterOn NOT found by scan! TrajHead may stay 0.");
    }

    /* Log Engine state — comprehensive diagnostics */
    Log("Engine: PPS=%d, TrajIndex=%d, TrajHead=%d, State=%d",
        Engine->PPS, Engine->TrajIndex, Engine->TrajHead, Engine->State);
    Log("Engine: Configured=%d, Busy=%d, Idle=%d, Emergency=%d, Jogging=%d",
        (int)Engine->Configured, (int)Engine->Busy, (int)Engine->Idle,
        (int)Engine->Emergency, (int)Engine->Jogging);
    Log("Engine: Port1On=%d, Port2On=%d, Sync=%d, EStop=%d",
        (int)Engine->Port1On, (int)Engine->Port2On,
        (int)Engine->Sync, (int)Engine->EStop);

    /* Log what Mach3 loaded from XML for EXTACT1 BEFORE we overwrite */
    Log("EXTACT1 from XML: act=%d port=%d pin=%d neg=%d val=%d",
        (int)Engine->OutSigs[EXTACT1].active,
        (int)Engine->OutSigs[EXTACT1].OutPort,
        (int)Engine->OutSigs[EXTACT1].OutPin,
        (int)Engine->OutSigs[EXTACT1].Negated,
        (int)Engine->OutSigs[EXTACT1].Activated);

    /* Re-apply plugin config now that Engine is available.
     * The call in piInitControl ran too early, so Engine axes weren't enabled.
     * We do NOT write to MainPlanner — the user must set Mach3 Motor Tuning
     * to match the plugin config. */
    ApplyPluginConfig();

    /* Check Motor Tuning: if StepsPerAxis is 0 for any enabled axis,
     * warn that trajectories won't work correctly. */
    Log("NOTE: Ensure Mach3 Motor Tuning matches plugin config for each axis.");
    Log("  Plugin config: X=%.1f Y=%.1f Z=%.1f spu",
        g_axisConfig[0].stepsPerUnit, g_axisConfig[1].stepsPerUnit,
        g_axisConfig[2].stepsPerUnit);

    /* Feed per-axis acceleration into SegmentBuilder. */
    for (int i = 0; i < MAX_AXES; i++) {
        if (g_axisConfig[i].valid) {
            g_segBuilder->SetAxisAccel(i, g_axisConfig[i].accelStepsPerSec2);
        }
    }

    /* Add "Tiggy Motion Controller..." to Mach3's Config submenu.
     * Falls back to top-level menu bar if Config submenu not found.
     * Must be done before ESP32 connection which may return early. */
    {
        /* Find Mach3 main window: enumerate top-level windows with a menu */
        HWND hMain = nullptr;
        struct FindCtx { HWND result; DWORD pid; } ctx = { nullptr, GetCurrentProcessId() };
        EnumWindows([](HWND hWnd, LPARAM lParam) -> BOOL {
            auto *c = (FindCtx *)lParam;
            DWORD wndPid = 0;
            GetWindowThreadProcessId(hWnd, &wndPid);
            if (wndPid == c->pid && IsWindowVisible(hWnd) && GetMenu(hWnd)) {
                c->result = hWnd;
                return FALSE;  /* found */
            }
            return TRUE;
        }, (LPARAM)&ctx);
        hMain = ctx.result;

        if (hMain) {
            HMENU hMenu = GetMenu(hMain);
            if (hMenu) {
                /* Try to find the "Config" submenu */
                HMENU hConfigMenu = nullptr;
                int menuCount = GetMenuItemCount(hMenu);
                for (int i = 0; i < menuCount; i++) {
                    char buf[64] = {0};
                    GetMenuStringA(hMenu, i, buf, sizeof(buf), MF_BYPOSITION);
                    if (strstr(buf, "Config") || strstr(buf, "config")) {
                        hConfigMenu = GetSubMenu(hMenu, i);
                        break;
                    }
                }

                if (hConfigMenu) {
                    /* Grey out Mach3 config menus that conflict with our plugin.
                     * These settings are all handled by our dialog — letting users
                     * change them in Mach3 would cause confusion or be ignored. */
                    int configItemCount = GetMenuItemCount(hConfigMenu);
                    for (int ci = 0; ci < configItemCount; ci++) {
                        char itemText[128] = {0};
                        GetMenuStringA(hConfigMenu, ci, itemText, sizeof(itemText), MF_BYPOSITION);
                        if (strstr(itemText, "Motor Tuning") ||
                            strstr(itemText, "Ports and Pins") ||
                            strstr(itemText, "Ports & Pins")) {
                            UINT itemId = GetMenuItemID(hConfigMenu, ci);
                            if (itemId != (UINT)-1) {
                                EnableMenuItem(hConfigMenu, itemId, MF_BYCOMMAND | MF_GRAYED);
                                Log("  Greyed out Config menu: '%s' (id=%u)", itemText, itemId);
                            }
                        }
                    }

                    AppendMenuA(hConfigMenu, MF_SEPARATOR, 0, nullptr);
                    AppendMenuA(hConfigMenu, MF_STRING, IDM_WIFICNC_SETTINGS,
                                "Tiggy Motion Controller...");
                    Log("Added menu item to Config submenu");
                } else {
                    /* Fallback: top-level menu bar */
                    AppendMenuA(hMenu, MF_STRING, IDM_WIFICNC_SETTINGS,
                                "Tiggy Motion Controller");
                    Log("Config submenu not found, added to menu bar");
                }
                DrawMenuBar(hMain);
                g_mach3MainWnd = hMain;
                g_origMach3WndProc = (WNDPROC)SetWindowLongPtrA(
                    hMain, GWLP_WNDPROC, (LONG_PTR)Mach3MenuSubclassProc);
                Log("Added 'Tiggy Motion Controller' menu to Mach3 (HWND=%p)", (void*)hMain);
            }
        } else {
            Log("WARNING: Could not find Mach3 main window for menu");
        }
    }

    /* Connect to controller */
    Log("Step 1: Trying saved IP %s", g_pluginConfig.espAddress);
    bool mainOk = TryConnect(g_pluginConfig.espAddress);

    if (!mainOk) {
        Log("Step 2: UDP auto-discovery (5s timeout)...");
        char discoveredIp[64] = {0};
        if (g_network->Discover(discoveredIp, sizeof(discoveredIp), 5)) {
            Log("Found controller at %s", discoveredIp);
            mainOk = TryConnect(discoveredIp);
        }
    }

    if (!mainOk && strcmp(g_pluginConfig.espAddress, "192.168.4.1") != 0) {
        Log("Step 3: Default AP IP 192.168.4.1");
        mainOk = TryConnect("192.168.4.1");
    }

    if (!g_enabled) {
        Log("WARNING: Could not connect to controller. Plugin inactive.");
    }
    g_lastTrajIndex = Engine->TrajIndex;

    /* Connect to I/O expansion module (separate ESP32) */
    if (g_pluginConfig.ioModuleEnabled && g_pluginConfig.ioModuleAddress[0] != '\0') {
        Log("I/O Module: connecting to %s...", g_pluginConfig.ioModuleAddress);
        if (g_ioModule->Connect(g_pluginConfig.ioModuleAddress, WCNC_TCP_CONTROL_PORT)) {
            if (g_ioModule->Handshake()) {
                const wcnc_handshake_resp_t *ioResp = g_ioModule->GetHandshakeResp();
                if (ioResp->capabilities & WCNC_CAP_IO_MODULE) {
                    g_ioModule->StartUDP(g_pluginConfig.ioModuleAddress);
                    g_ioConnected = true;
                    Log("I/O Module: connected (%d channels)",
                        ioResp->io_channel_count);
                } else {
                    Log("I/O Module: device at %s is not in I/O module mode",
                        g_pluginConfig.ioModuleAddress);
                    g_ioModule->Disconnect();
                }
            } else {
                Log("I/O Module: handshake failed");
                g_ioModule->Disconnect();
            }
        } else {
            Log("I/O Module: TCP connect failed to %s", g_pluginConfig.ioModuleAddress);
        }
    }
}

/* ===================================================================
 * TrajPoint Command Handler
 *
 * Now actually sends commands to ESP32 (not just logging).
 * Matches SmoothStepper's approach of forwarding spindle/coolant/dwell.
 * =================================================================== */

static void HandleTrajCommand(int cmd, int id)
{
    /* Use TRACKED state as the baseline — Engine->Flood/Mist/SpindleCW/CCW
     * can contain garbage values, especially at startup.  The tracked state
     * (g_sentSpindle/g_sentCoolant) reflects what we actually last sent to
     * the ESP32, so it is always reliable. */
    uint8_t spindleState = g_sentSpindle;
    uint8_t coolantState = g_sentCoolant;

    switch (cmd) {
    case SPINAXISCW:
        Log("TrajCmd: Spindle CW (id=%d)", id);
        SendIOControl(g_prevMiscOutputs, 1, (uint16_t)id, coolantState);
        Engine->SpindleStable = true;
        break;
    case SPINAXISCCW:
        Log("TrajCmd: Spindle CCW (id=%d)", id);
        SendIOControl(g_prevMiscOutputs, 2, (uint16_t)id, coolantState);
        Engine->SpindleStable = true;
        break;
    case SPINAXISSTOP:
        Log("TrajCmd: Spindle STOP");
        SendIOControl(g_prevMiscOutputs, 0, 0, coolantState);
        Engine->SpindleStable = true;
        break;
    case FLOODSIGON:
        Log("TrajCmd: Flood ON");
        SendIOControl(g_prevMiscOutputs, spindleState, 0, coolantState | 0x01);
        break;
    case FLOODSIGOFF:
        Log("TrajCmd: Flood OFF");
        SendIOControl(g_prevMiscOutputs, spindleState, 0, coolantState & ~0x01);
        break;
    case MISTSIGON:
        Log("TrajCmd: Mist ON");
        SendIOControl(g_prevMiscOutputs, spindleState, 0, coolantState | 0x02);
        break;
    case MISTSIGOFF:
        Log("TrajCmd: Mist OFF");
        SendIOControl(g_prevMiscOutputs, spindleState, 0, coolantState & ~0x02);
        break;
    case COMDWELL:
        Log("TrajCmd: Dwell (id=%d ms)", id);
        if (g_bufferMgr) {
            wcnc_motion_segment_t dwellSeg;
            memset(&dwellSeg, 0, sizeof(dwellSeg));
            dwellSeg.duration_us = (uint32_t)((double)id * 1000.0);
            dwellSeg.flags = WCNC_SEG_FLAG_EXACT_STOP;
            g_bufferMgr->QueueSegment(&dwellSeg);
        }
        break;
    case PROGRAMEND:
        Log("TrajCmd: Program end");
        break;
    case PROGRAMSTOP:
    case PROGRAMSTOPOPT:
        Log("TrajCmd: Program stop (cmd=%d)", cmd);
        break;

    /* External activation signals -> ESP32 misc outputs */
    case COMEXT1ON:  g_prevMiscOutputs |= 0x01; goto send_misc;
    case COMEXT1OFF: g_prevMiscOutputs &= ~0x01; goto send_misc;
    case COMEXT2ON:  g_prevMiscOutputs |= 0x02; goto send_misc;
    case COMEXT2OFF: g_prevMiscOutputs &= ~0x02; goto send_misc;
    case COMEXT3ON:  g_prevMiscOutputs |= 0x04; goto send_misc;
    case COMEXT3OFF: g_prevMiscOutputs &= ~0x04; goto send_misc;
    case COMEXT4ON:  g_prevMiscOutputs |= 0x08; goto send_misc;
    case COMEXT4OFF: g_prevMiscOutputs &= ~0x08; goto send_misc;
    case COMEXT5ON:  g_prevMiscOutputs |= 0x10; goto send_misc;
    case COMEXT5OFF: g_prevMiscOutputs &= ~0x10; goto send_misc;
    send_misc:
        {
            /* Use TRACKED state, not Engine fields */
            SendIOControl(g_prevMiscOutputs, spindleState, 0, coolantState);
            Log("TrajCmd: ExtAct -> misc=0x%02X", g_prevMiscOutputs);
        }
        break;
    case COMEXT6ON: case COMEXT6OFF:
        Log("TrajCmd: ExtAct6 cmd=%d (no GPIO mapped)", cmd);
        break;

    /* Lathe / spindle sync commands */
    case SPINAXISSPEED:
        Log("TrajCmd: Spindle Speed (id=%d RPM)", id);
        SendIOControl(g_prevMiscOutputs, spindleState, (uint16_t)id, coolantState);
        Engine->SpindleStable = true;
        break;
    case STRAIGHTHREAD:
        Log("TrajCmd: Threading move (id=%d)", id);
        break;
    case SETCSSON:
        Log("TrajCmd: CSS ON (G96)");
        break;
    case SETRPMON:
        Log("TrajCmd: RPM mode (G97)");
        break;

    default:
        {
            static int s_unhandledCount = 0;
            if (s_unhandledCount < 50) {
                Log("TrajCmd: cmd=%d id=%d", cmd, id);
                s_unhandledCount++;
            }
        }
        break;
    }
}

/* ===================================================================
 * Input Signal Update (like SmoothStepper ReadInputs)
 *
 * Maps ESP32 status report fields to Engine->InSigs[]:
 *   - limit_switches bitmask -> InSigs[XLIMITPLUS..CLIMITMINUS]
 *   - home_switches bitmask  -> InSigs[XHOME..CHOME]
 *   - estop_input             -> InSigs[EMERGENCY]
 *   - probe_state             -> InSigs[DIGITIZE]
 *
 * Only updates signals that are Active (configured in Mach3 Ports & Pins).
 * Mach3's Negated flag is applied by the kernel after we set Activated.
 *
 * Signal index layout (from Engine.h):
 *   Axis i: LIMITPLUS = i*3, LIMITMINUS = i*3+1, HOME = i*3+2
 *   X: 0,1,2  Y: 3,4,5  Z: 6,7,8  A: 9,10,11  B: 12,13,14  C: 15,16,17
 *   DIGITIZE=22, EMERGENCY=25
 * =================================================================== */

static void UpdateInputSignals(const wcnc_status_report_t *status)
{
    if (!Engine) return;

    /* Limit switches: firmware already applies inversion via s_invert_limit
     * (in the debounce callback), same as home switches.  Do NOT invert
     * again here — double-inversion would cancel out and Mach3 would always
     * see the raw pin level regardless of the user's inversion setting.
     * One bit per axis → map to both LIMITPLUS and LIMITMINUS. */
    for (int i = 0; i < MAX_AXES; i++) {
        bool triggered = ((status->limit_switches >> i) & 1) != 0;

        int plusSig  = XLIMITPLUS  + i * 3;
        int minusSig = XLIMITMINUS + i * 3;

        if (Engine->InSigs[plusSig].Active)
            Engine->InSigs[plusSig].Activated = triggered;
        if (Engine->InSigs[minusSig].Active)
            Engine->InSigs[minusSig].Activated = triggered;
    }

    /* Home switches: firmware already applies inversion (protocol spec),
     * so we do NOT invert again here. */
    for (int i = 0; i < MAX_AXES; i++) {
        bool triggered = ((status->home_switches >> i) & 1) != 0;
        int homeSig = XHOME + i * 3;

        if (Engine->InSigs[homeSig].Active)
            Engine->InSigs[homeSig].Activated = triggered;
    }

    /* E-Stop: firmware sends RAW physical pin state.
     * Inversion comes from Mach3 Ports & Pins (InSigs[].Negated).
     * Legacy fallback: g_pluginConfig.invertEstop if Negated not set. */
    if (Engine->InSigs[EMERGENCY].Active) {
        bool triggered = (status->estop_input != 0);
#ifdef LEGACY_INVERSION
        if (g_pluginConfig.invertEstop) triggered = !triggered;
#else
        if (Engine->InSigs[EMERGENCY].Negated) triggered = !triggered;
#endif
        Engine->InSigs[EMERGENCY].Activated = triggered;
    }

    /* Probe: firmware sends RAW state.
     * Inversion from Mach3 Ports & Pins (Negated field). */
    if (Engine->InSigs[DIGITIZE].Active) {
        bool triggered = (status->probe_state != 0);
#ifdef LEGACY_INVERSION
        if (g_pluginConfig.invertProbe) triggered = !triggered;
#else
        if (Engine->InSigs[DIGITIZE].Negated) triggered = !triggered;
#endif
        Engine->InSigs[DIGITIZE].Activated = triggered;
    }

    /* Misc inputs: ESP32 misc_inputs bitmask → ACTIVATION1-4 */
    for (int i = 0; i < 4; i++) {
        bool triggered = ((status->misc_inputs >> i) & 1) != 0;
        int sig = ACTIVATION1 + i;
        if (Engine->InSigs[sig].Active)
            Engine->InSigs[sig].Activated = triggered;
    }
}

/* ===================================================================
 * piUpdate - Main callback (~10-50Hz)
 *
 * Modeled after SmoothStepper's 13-phase Update():
 *   Phase 1: First-run init (we do this in PostInitControl)
 *   Phase 2: E-stop state tracking
 *   Phase 3: Control outputs (charge pump, etc.)
 *   Phase 4: Read hardware status (via UDP)
 *   Phase 5: Spindle control
 *   Phase 6-9: Trajectory feeding
 *   Phase 10: Feed hold
 *   Phase 11: Output bits / coolant
 *   Phase 12: Position feedback
 *   Phase 13: Housekeeping
 * =================================================================== */

static DWORD g_updateCount = 0;
static DWORD g_lastLogUpdate = 0;
static DWORD g_nextReconnectTick = 0;
static int   g_reconnectAttempts = 0;

void piUpdateImpl()
{
    g_updateCount++;

    if (!g_enabled || !g_connected || !Engine) return;

    /* --- Reconnection with backoff --- */
    if (!g_network->IsConnected()) {
        DWORD now = GetTickCount();
        if (now < g_nextReconnectTick) return;

        g_connected = false;
        g_enabled = false;
        g_reconnectAttempts++;
        Log("Connection lost, reconnect #%d...", g_reconnectAttempts);

        if (TryConnect(g_pluginConfig.espAddress)) {
            g_lastTrajIndex = Engine->TrajIndex;
            g_reconnectAttempts = 0;
        } else {
            DWORD delay = 1000u << (g_reconnectAttempts < 5 ? g_reconnectAttempts : 4);
            if (delay > 30000) delay = 30000;
            g_nextReconnectTick = now + delay;
        }
        return;
    }

    /* --- Phase 2: E-stop state tracking (like SS) --- */
    bool currentEStop = Engine->Emergency;
    if (currentEStop != g_prevEStop) {
        g_prevEStop = currentEStop;
        if (currentEStop) {
            /* Entering E-stop: save positions */
            for (int i = 0; i < MAX_AXES; i++)
                g_savedPos[i] = Engine->Axis[i].Index;
            Log("E-stop entered, positions saved");
            g_network->SendEStop();
        } else {
            /* Exiting E-stop: positions will be updated from status reports */
            Log("E-stop cleared");
            g_network->SendReset();
        }
    }

    /* --- Phase 3: Charge pump monitoring (like SS) --- */
    {
        bool chargePump = Engine->ChargeAlwaysOn ||
                          (Engine->OutSigs[CHARGE].active && Engine->OutSigs[CHARGE].Activated);
        if (chargePump != g_prevChargePump) {
            g_prevChargePump = chargePump;
            /* Toggle charge pump via config: freq > 0 = on, 0 = off */
            uint16_t freq = chargePump ? g_pluginConfig.chargePumpFreq : 0;
            if (g_network && g_connected) {
                g_network->SendConfig(WCNC_CFG_CHARGE_PUMP_FREQ, &freq, WCNC_VAL_UINT16);
            }
            Log("ChargePump: %s (freq=%u Hz)", chargePump ? "ON" : "OFF", freq);
        }
    }

    /* Phase 5 & 11 (spindle/coolant polling) REMOVED.
     * Engine->SpindleCW/CCW/Flood/Mist contain garbage values (7, not 0/1)
     * that never become reliable.  Polling them causes constant false
     * triggers that override the correct EX_SPINON/EX_SPINOFF and
     * SPINAXISCW/SPINAXISSTOP notifications.
     * All spindle/coolant IO is now driven exclusively by:
     *   - TrajCmd handler (SPINAXISCW, SPINAXISSTOP, FLOODSIGON, etc.)
     *   - EX_SPINON / EX_SPINOFF notifications
     * Config dialog reads state from ESP32 status report (ground truth). */

    /* --- Phase 11b: Misc output monitoring (EXTACT1-5 -> ESP32 GPIO) --- */
    {
        /* High-frequency change detector: fires every cycle (~10ms).
         * Logs the INSTANT any OutSig, State, or TrajHead changes. */
        {
            static int     s_prevState = -1;
            static int     s_prevHead  = -1;
            static bool    s_prevActExt[5] = {false};
            static bool    s_prevActAll[nSigsOut] = {false};
            static DWORD   s_lastPeriodic = 0;

            /* Check State change */
            if (Engine->State != s_prevState) {
                Log("!! State changed: %d -> %d", s_prevState, Engine->State);
                s_prevState = Engine->State;
            }

            /* Check TrajHead change */
            if (Engine->TrajHead != s_prevHead) {
                Log("!! TrajHead changed: %d -> %d (Idx=%d)",
                    s_prevHead, Engine->TrajHead, Engine->TrajIndex);
                /* Log commands in new entries */
                int scan = (s_prevHead >= 0) ? s_prevHead : 0;
                for (int n = 0; n < 20 && scan != Engine->TrajHead; n++) {
                    int cmd = Engine->Trajectories[scan & 0xFFF].Command;
                    if (cmd != 0)
                        Log("  Traj[%d] cmd=%d", scan, cmd);
                    scan = (scan + 1) & 0xFFF;
                }
                s_prevHead = Engine->TrajHead;
            }

            /* Check ALL 30 OutSigs for Activated change */
            for (int i = 0; i < nSigsOut; i++) {
                bool act = Engine->OutSigs[i].Activated;
                if (act != s_prevActAll[i]) {
                    Log("!! OutSigs[%d].Activated changed: %d -> %d (active=%d port=%d pin=%d)",
                        i, (int)s_prevActAll[i], (int)act,
                        (int)Engine->OutSigs[i].active,
                        (int)Engine->OutSigs[i].OutPort,
                        (int)Engine->OutSigs[i].OutPin);
                    s_prevActAll[i] = act;
                }
            }

            /* Periodic summary every 5 seconds */
            DWORD now = GetTickCount();
            if (now - s_lastPeriodic > 5000) {
                s_lastPeriodic = now;
                Log("EXTACT1: act=%d port=%d pin=%d val=%d | Head=%d Idx=%d State=%d",
                    (int)Engine->OutSigs[EXTACT1].active,
                    (int)Engine->OutSigs[EXTACT1].OutPort,
                    (int)Engine->OutSigs[EXTACT1].OutPin,
                    (int)Engine->OutSigs[EXTACT1].Activated,
                    Engine->TrajHead, Engine->TrajIndex, Engine->State);
            }
        }

        /* Misc outputs are handled ONLY via trajectory commands (COMEXT1ON/OFF)
         * in the trajectory processing loop.  We do NOT poll
         * Engine->OutSigs[EXTACT*].Activated here because Mach3 internally
         * toggles EXTACT signals alongside spindle events, causing false
         * activations.  M62/M63 G-codes generate COMEXT trajectory commands
         * which are processed correctly in the trajectory handler. */
    }

    /* --- Homing: sequential queue state machine --- */
    if (g_homeState == HOME_COLLECTING &&
        (GetTickCount() - g_homeCollectTick) > 50)
    {
        /* Collection window elapsed — start sending one axis at a time */
        HomeSendNext();
    }
    else if (g_homeState == HOME_SENT &&
             (GetTickCount() - g_homeSentTick) > 10000)
    {
        /* 10s timeout waiting for ESP32 to enter HOMING state */
        Log("Home: TIMEOUT waiting for HOMING state (axis mask=0x%02X)",
            g_activeHomeMask);
        for (int i = 0; i < MAX_AXES; i++) {
            if (g_activeHomeMask & (1 << i))
                Engine->Axis[i].Homing = false;
        }
        g_activeHomeMask = 0;
        HomeSendNext();  /* Try next axis anyway */
    }
    else if (g_homeState == HOME_ACTIVE &&
             (GetTickCount() - g_homeActiveTick) > 30000)
    {
        /* 30s timeout: firmware stuck in HOMING (switch never triggered).
         * Send E-stop to abort the firmware's homing loop, then reset. */
        Log("Home: TIMEOUT — axis never hit switch (mask=0x%02X), aborting",
            g_activeHomeMask);
        g_network->SendEStop();
        Sleep(100);
        g_network->SendReset();

        for (int i = 0; i < MAX_AXES; i++) {
            if (g_activeHomeMask & (1 << i))
                Engine->Axis[i].Homing = false;
        }
        g_activeHomeMask = 0;

        /* Abort remaining queue — don't try more axes after a timeout */
        g_homeState = HOME_IDLE;
        g_homeQueueLen = 0;
        g_homeQueueIdx = 0;
        Log("Home: aborted all remaining axes");
    }

    /* --- Phase 9: Read VMS trajectory data ---
     *
     * With ExternalType = EX_VMS, Mach3 fills Engine->Trajectories[] with
     * per-period step counts.  Each TrajPoint.Points[axis] is the number
     * of steps to take in one ExTime period (1 ms).  A single G-code move
     * generates many entries (e.g. a 1-second move = 1000 entries).
     *
     * We accumulate consecutive entries for the same G-code line into one
     * motion segment, then compute trapezoidal timing and queue it.  This
     * converts the stream of per-period data into complete move segments
     * that the ESP32 firmware can execute.
     *
     * Flush triggers: line ID change, command entry, or safety batch limit.
     */
    int currentHead = Engine->TrajHead;
    int currentIndex = g_lastTrajIndex;

    /* Periodic status log with MainPlanner diagnostics */
    if (g_updateCount - g_lastLogUpdate > 250) {
        g_lastLogUpdate = g_updateCount;
        Log("Update #%lu: Head=%d, Idx=%d, delta=%d, Cfg=%d",
            g_updateCount, currentHead, currentIndex,
            currentHead - currentIndex,
            (int)Engine->Configured);

        /* Read back key MainPlanner fields to diagnose stalls */
        if (MainPlanner) {
            char *mp = (char *)MainPlanner;
            __try {
                short extType = *(short *)(mp + MP_OFF_EXTERNAL_TYPE);
                bool  still   = *(bool  *)(mp + MP_OFF_EXTERNAL_STILL);
                double rate0  = *(double *)(mp + MP_OFF_EXTERNAL_PULSE_RATES);
                double exTime = *(double *)(mp + MP_OFF_EXTIME);
                int   bufHi   = *(int   *)(mp + MP_OFF_EXBUFFER_HI);
                Log("  MP: ExtType=%d Still=%d Rate0=%.0f ExTime=%.6f BufHi=%d",
                    (int)extType, (int)still, rate0, exTime, bufHi);
            } __except(EXCEPTION_EXECUTE_HANDLER) {}
        }
        Log("  Eng: Sync=%d Busy=%d Idle=%d Dwell=%d EStop=%d Jog=%d FRO=%d%%",
            (int)Engine->Sync, (int)Engine->Busy, (int)Engine->Idle,
            (int)Engine->DwellTime, (int)Engine->EStop,
            (int)Engine->Jogging, (int)Engine->OverRide);

        /* Drift check: compare cumulative trajectory steps with reported position.
         * g_trajCumSteps = initial_index + sum of all trajectory steps consumed.
         * Engine->Axis[i].Index = ESP32 position + offset.
         * Any difference = step loss or accumulation error. */
        Log("  TrajCum: [%d,%d,%d]  Index: [%d,%d,%d]  drift: [%d,%d,%d]",
            g_trajCumSteps[0], g_trajCumSteps[1], g_trajCumSteps[2],
            Engine->Axis[0].Index, Engine->Axis[1].Index, Engine->Axis[2].Index,
            g_trajCumSteps[0] - Engine->Axis[0].Index,
            g_trajCumSteps[1] - Engine->Axis[1].Index,
            g_trajCumSteps[2] - Engine->Axis[2].Index);
    }

    /* Accumulator state is in file-scope globals g_accumSteps/Count/LineId */

    /* ExternalStill is now managed in the ESP32 status feedback section
     * (Phase 4/12 below), based on actual controller motion state.
     * This matches the Galil/ncPod pattern: Still=false while ESP32 is
     * moving, Still=true when idle. */

    /* Lambda: flush accumulated steps into a motion segment */
    auto FlushAccum = [&]() {
        if (g_accumCount == 0) return;

        /* Clone steps to slave axes */
        for (int i = 0; i < MAX_AXES; i++) {
            int master = g_pluginConfig.axis[i].cloneMaster;
            if (master >= 0 && master < MAX_AXES && master != i) {
                int steps = g_accumSteps[master];
                if (g_pluginConfig.axis[i].cloneReversed) steps = -steps;
                g_accumSteps[i] = steps;
            }
        }

        /* Find dominant axis for timing */
        uint32_t maxSteps = 0;
        int dominantAxis = 0;
        bool hasMotion = false;
        for (int i = 0; i < MAX_AXES; i++) {
            uint32_t absSteps = (uint32_t)(g_accumSteps[i] < 0 ? -(int64_t)g_accumSteps[i] : g_accumSteps[i]);
            if (absSteps > maxSteps) {
                maxSteps = absSteps;
                dominantAxis = i;
            }
            if (g_accumSteps[i] != 0) hasMotion = true;
        }

        if (hasMotion && maxSteps > 0) {
            wcnc_motion_segment_t seg;
            memset(&seg, 0, sizeof(seg));

            for (int i = 0; i < MAX_AXES && i < WCNC_MAX_AXES; i++)
                seg.steps[i] = g_accumSteps[i];

            /* Each trajectory entry = 1 ExTime period (1ms).
             * g_accumCount entries = g_accumCount ms of motion at 100% FRO.
             *
             * In VMS mode, Mach3 does NOT adjust entry count for FRO during
             * program execution.  We must explicitly scale the duration using
             * Engine->OverRide (int, percentage: 100 = 100%).
             *   FRO 50%  → duration * 2.0 (slower)
             *   FRO 200% → duration * 0.5 (faster)
             */
            double duration = g_accumCount * 0.001;  /* seconds at 100% */
            if (duration < 0.0001) duration = 0.0001;

            /* Apply feed rate override */
            double froPercent = (double)Engine->OverRide;
            if (froPercent < 1.0) froPercent = 100.0;  /* safety: avoid div-by-zero */
            double froScale = froPercent / 100.0;
            duration = duration / froScale;

            seg.duration_us = (uint32_t)(duration * 1000000.0);
            if (seg.duration_us < 100) seg.duration_us = 100;

            /* Compute entry/exit speeds from accumulated data.
             * Same formula as SegmentBuilder::EmitSegment():
             *   speed = maxSteps / duration   (steps/sec)
             *   speed_sqr = speed^2 * 1000    (protocol scale)
             * Setting equal entry/exit = constant velocity (Mach3 pre-planned).
             * Use double to avoid float-to-uint32 undefined behavior on overflow. */
            double speed = (double)maxSteps / duration;
            double speed_sqr_d = speed * speed * 1000.0;
            uint32_t speed_sqr = (speed_sqr_d > 4294967295.0)
                               ? 0xFFFFFFFFu
                               : (uint32_t)speed_sqr_d;

            seg.entry_speed_sqr = speed_sqr;
            seg.exit_speed_sqr  = speed_sqr;

            double accel = (double)g_axisConfig[dominantAxis].accelStepsPerSec2;
            if (accel < 1.0) accel = 5000.0;
            seg.acceleration    = (uint32_t)(accel * 100.0);
            seg.segment_id      = g_segId++;

            if (g_segBuilder && g_segBuilder->IsProbeMode()) {
                seg.flags |= WCNC_SEG_FLAG_PROBE;
                g_segBuilder->SetProbeMode(false);
            }

            if (g_segLogCount < 50) {
                g_segLogCount++;
                Log("SEG[%d]: steps=[%d,%d,%d,%d,%d,%d] dur=%uus accum=%d line=%d",
                    seg.segment_id,
                    seg.steps[0], seg.steps[1], seg.steps[2],
                    seg.steps[3], seg.steps[4], seg.steps[5],
                    seg.duration_us, g_accumCount, g_accumLineId);
            }

            g_bufferMgr->QueueSegment(&seg);
        }

        /* Reset accumulator */
        memset(g_accumSteps, 0, sizeof(g_accumSteps));
        g_accumCount = 0;
    };

    while (currentIndex != currentHead) {
        int idx = currentIndex & TRAJ_BUFFER_MASK;
        TrajPoint *tp = &Engine->Trajectories[idx];

        Engine->DisplayLine = tp->ID;

        /* Log first 100 trajectory entries for diagnostics */
        if (g_trajLogCount < 100) {
            g_trajLogCount++;
            Log("TRAJ[%d]: Points=[%d,%d,%d,%d,%d,%d], Cmd=%d, ID=%d",
                idx,
                tp->Points[0], tp->Points[1], tp->Points[2],
                tp->Points[3], tp->Points[4], tp->Points[5],
                tp->Command, tp->ID);
        }

        /* Flush accumulated segment when line ID changes */
        if (g_accumCount > 0 && tp->ID != g_accumLineId) {
            FlushAccum();
        }

        /* Handle inline commands (spindle, coolant, dwell, etc.) */
        if (tp->Command != 0) {
            FlushAccum();  /* flush before command */
            HandleTrajCommand(tp->Command, tp->ID);
        }

        /* Accumulate per-period steps */
        g_accumLineId = tp->ID;
        for (int i = 0; i < 6; i++) {
            g_accumSteps[i] += tp->Points[i];
            g_trajCumSteps[i] += tp->Points[i];  /* track total for drift check */
        }
        g_accumCount++;

        /* Safety: flush if accumulated too many entries (prevent unbounded growth) */
        if (g_accumCount >= 5000) {
            FlushAccum();
        }

        currentIndex++;
        currentIndex &= TRAJ_BUFFER_MASK;

        /* Update Engine->TrajIndex INSIDE the loop (like Galil does) */
        Engine->TrajIndex = currentIndex;
    }

    /* Flush remaining accumulated data when buffer is fully consumed.
     * If more data arrives next cycle for the same line, it becomes a
     * new segment — this is acceptable and keeps latency low. */
    if (g_accumCount > 0 && currentIndex == currentHead) {
        FlushAccum();
    }

    g_lastTrajIndex = currentIndex;

    /* --- Phase 4/12: Process ESP32 status (position feedback) --- */
    static DWORD s_statusCount = 0;
    wcnc_status_report_t status;
    if (g_network->GetLatestStatus(&status)) {
        s_statusCount++;

        static bool s_firstStatus = true;
        if (s_firstStatus) {
            s_firstStatus = false;
            Log("FIRST STATUS: state=%d, buf=%u/%u, pos=[%d,%d,%d,%d,%d,%d]",
                status.state, status.buffer_available, status.buffer_total,
                status.position_steps[0], status.position_steps[1],
                status.position_steps[2], status.position_steps[3],
                status.position_steps[4], status.position_steps[5]);
        }

        static uint8_t s_lastState = 0xFF;
        if (status.state != s_lastState) {
            static const char *stateNames[] = {
                "IDLE", "RUN", "HOLD", "JOG", "HOMING",
                "PROBING", "ALARM", "ESTOP", "DISCONNECTED"
            };
            const char *name = (status.state < 9) ? stateNames[status.state] : "?";
            Log("Device: %s(%d) flags=0x%02X buf=%u/%u",
                name, status.state, status.flags,
                status.buffer_available, status.buffer_total);

            /* Detect RUN/JOG/HOMING → IDLE transition.
             * Set Engine->Sync = true so Mach3 re-syncs and continues
             * generating trajectory data (like Galil/ncPod plugins).
             * Without this, Mach3 stalls after the first move batch. */
            bool wasMoving = (s_lastState == WCNC_STATE_RUN ||
                              s_lastState == 3 /*JOG*/ ||
                              s_lastState == 4 /*HOMING*/);
            bool nowIdle   = (status.state == WCNC_STATE_IDLE);
            if (wasMoving && nowIdle) {
                Engine->Sync = true;
                Log("  Sync=true (motion complete)");
            }

            s_lastState = status.state;
        }

        /* Manage ExternalStill based on ESP32 actual motion state.
         * ExternalStill=false while controller is moving/busy,
         * ExternalStill=true when idle (ready for more trajectory data).
         * This matches the Galil/ncPod pattern. */
        if (MainPlanner) {
            char *mp = (char *)MainPlanner;
            __try {
                bool *pStill = (bool *)(mp + MP_OFF_EXTERNAL_STILL);
                bool isMoving = (status.state == WCNC_STATE_RUN ||
                                 status.state == 3 /*JOG*/ ||
                                 status.state == 4 /*HOMING*/);
                *pStill = !isMoving;
            } __except(EXCEPTION_EXECUTE_HANDLER) {}
        }

        /* Flow control */
        g_bufferMgr->UpdateRemoteBufferLevel(
            status.buffer_available, status.buffer_total);

        /* Position feedback (Galil/ncPod pattern: direct write, no per-cycle
         * offset tracking).
         *
         * The ESP32 reports absolute step positions from power-on.
         * We map them to Mach3 coordinates via a fixed offset computed
         * once on init (or after Reset).  No per-cycle machDelta tracking —
         * that caused cumulative drift when Mach3's VMS trajectory system
         * updated Index between our writes. */
        for (int i = 0; i < MAX_AXES; i++) {
            if (!g_posInitialized) {
                /* First status after connect/reset — match whatever Mach3 has */
                g_posOffset[i] = Engine->Axis[i].Index - status.position_steps[i];
                /* Also reset cumulative step tracker */
                g_trajCumSteps[i] = Engine->Axis[i].Index;
            }

            int32_t newIndex = status.position_steps[i] + g_posOffset[i];
            Engine->Axis[i].Index       = newIndex;
            Engine->Axis[i].MasterIndex = newIndex;
        }
        if (!g_posInitialized) {
            g_posInitialized = true;
            Log("Position offsets initialized: [%d,%d,%d,%d,%d,%d]",
                g_posOffset[0], g_posOffset[1], g_posOffset[2],
                g_posOffset[3], g_posOffset[4], g_posOffset[5]);
        }

        /* Velocity display */
        for (int i = 0; i < MAX_AXES; i++) {
            Engine->Axis[i].CurVelocity = status.feed_rate;
        }

        /* Spindle encoder RPM feedback (v1.1)
         * TrajBuffer has no direct RPM field accessible from plugins.
         * Store RPM for our config dialog display; Mach3 DRO feedback
         * can be added later via SetDRO with the appropriate OEM DRO#. */
        if (g_network->GetHandshakeResp()->capabilities & WCNC_CAP_SPINDLE_ENCODER) {
            g_lastSpindleRPM = status.spindle_rpm;

            /* Feed spindle position to Mach3 Engine for threading/CSS.
             * Engine->CurrentSpindleCount: Mach3 reads this to track spindle
             * rotation during threading (G33/G76) and CSS (G96).
             * Our status reports angular position as 0-65535 = 0-360 degrees.
             * Engine->MaxSpinCounts: tells Mach3 the counter range. */
            Engine->CurrentSpindleCount = (int)status.spindle_position;
            Engine->MaxSpinCounts = 65535;

            /* Index pulse: rising-edge detection on spindle_index_count.
             * Mach3 uses InSigs[INDEX] edges for thread start sync. */
            if (status.spindle_index_count != g_lastIndexCount) {
                if (Engine->InSigs[INDEX].Active)
                    Engine->InSigs[INDEX].Activated = true;
                g_lastIndexCount = status.spindle_index_count;
            } else {
                if (Engine->InSigs[INDEX].Active)
                    Engine->InSigs[INDEX].Activated = false;
            }
        }

        /* DRO update via SetDRO API.
         * Also push machine positions directly so DROs work even if
         * the MainPlanner StepsPerAxis write offset is slightly off. */
        if (SetDRO) {
            for (int i = 0; i < MAX_AXES; i++) {
                double spu = g_axisConfig[i].stepsPerUnit;
                if (spu > 0.0) {
                    double pos = (double)(status.position_steps[i] + g_posOffset[i]) / spu;
                    SetDRO((short)(DRO_X_MACH_POS + i), pos);
                }
            }
        }

        /* Update input signals: limits, home, E-stop, probe
         * (like SmoothStepper ReadInputs -> Engine->InSigs[]) */
        UpdateInputSignals(&status);

        /* Trigger Mach3 E-stop via DoButton if ESP32 reports ESTOP/ALARM
         * from an EXTERNAL source (hardware E-stop, limit hit, etc.).
         * Skip if WE initiated the E-Stop via piResetImpl() to avoid
         * a feedback loop (Reset → E-Stop → DoButton → Reset → ...). */
        if (status.state == WCNC_STATE_ESTOP ||
            status.state == WCNC_STATE_ALARM) {
            if (DoButton && !Engine->Emergency && !g_pluginResetActive) {
                DoButton((short)OEM_ESTOP);
            }
        }

        /* Clear the reset flag once ESP32 reaches IDLE */
        if (status.state == WCNC_STATE_IDLE) {
            g_pluginResetActive = false;
        }

        /* Input function mapping: rising-edge detection on ACTIVATION1-4.
         * When a misc input transitions 0->1 and has a function assigned,
         * fire DoButton() to trigger that Mach3 action (Cycle Start, etc.) */
        if (DoButton) {
            for (int i = 0; i < 4; i++) {
                bool active = Engine->InSigs[ACTIVATION1 + i].Activated;
                if (active && !g_prevMiscInput[i]) {
                    uint16_t func = g_pluginConfig.miscInputFunction[i];
                    if (func != 0) {
                        DoButton((short)func);
                        Log("MiscInput%d: rising edge -> DoButton(%u)", i + 1, func);
                    }
                }
                g_prevMiscInput[i] = active;
            }
        }

        /* Homing state machine: track ESP32 state transitions */
        if (g_homeState == HOME_SENT &&
            status.state == WCNC_STATE_HOMING)
        {
            /* ESP32 acknowledged and entered HOMING */
            g_homeState = HOME_ACTIVE;
            g_homeActiveTick = GetTickCount();
            Log("Home: Controller entered HOMING state (mask=0x%02X)", g_activeHomeMask);
        }
        else if (g_homeState == HOME_ACTIVE &&
                 g_prevMachineState == WCNC_STATE_HOMING &&
                 status.state != WCNC_STATE_HOMING)
        {
            /* HOMING → something else = this axis group is done */
            Log("Home: axis group complete (mask=0x%02X)", g_activeHomeMask);
            for (int i = 0; i < MAX_AXES; i++) {
                if (g_activeHomeMask & (1 << i)) {
                    Engine->Referenced[i] = true;
                    Engine->Axis[i].Homing = false;
                    Log("  Axis %d: Referenced, pos=%d",
                        i, status.position_steps[i]);
                }
            }
            g_activeHomeMask = 0;
            /* Send next axis from queue (or finish) */
            HomeSendNext();
        }
        g_prevMachineState = status.state;
    }

    /* Probe/Home completion */
    if (g_network && Engine) {
        int32_t probePos[WCNC_MAX_AXES];
        bool probeSuccess = false;
        if (g_network->GetProbeResult(probePos, &probeSuccess)) {
            Log("Probe: success=%d pos=[%d,%d,%d,%d,%d,%d]",
                (int)probeSuccess,
                probePos[0], probePos[1], probePos[2],
                probePos[3], probePos[4], probePos[5]);
            if (probeSuccess) {
                for (int i = 0; i < MAX_AXES; i++)
                    Engine->Axis[i].Index = probePos[i];
            }
        }

        uint8_t homeMask = 0;
        bool homeSuccess = false;
        if (g_network->GetHomeComplete(&homeMask, &homeSuccess)) {
            Log("Home complete: mask=0x%02X success=%d", homeMask, (int)homeSuccess);
            if (homeSuccess) {
                for (int i = 0; i < MAX_AXES; i++) {
                    if (homeMask & (1 << i)) {
                        Engine->Referenced[i] = true;
                        Engine->Axis[i].Homing = false;
                        Engine->Axis[i].Index = 0;
                        Engine->Axis[i].MasterIndex = 0;
                    }
                }
            }
        }
    }

    /* --- I/O Module polling --- */
    if (g_ioConnected && g_ioModule) {
        /* Check I/O module connection */
        if (!g_ioModule->IsConnected()) {
            g_ioConnected = false;
            Log("I/O Module: connection lost");
            /* Stop any active pendant jogs */
            for (int i = 0; i < MAX_AXES; i++) {
                if (g_ioJogActive[i]) {
                    g_network->SendJogStop((uint8_t)i);
                    g_ioJogActive[i] = false;
                }
            }
        } else {
            wcnc_status_report_t ioStatus;
            g_ioModule->GetLatestStatus(&ioStatus);

            uint16_t ioInputs = ioStatus.io_inputs;
            uint16_t changed = ioInputs ^ g_prevIOInputs;

            if (changed != 0) {
                /* Process each changed input */
                for (int i = 0; i < 16; i++) {
                    if (!(changed & (1 << i))) continue;
                    bool active = (ioInputs >> i) & 1;
                    uint16_t func = g_pluginConfig.ioInputFunction[i];
                    if (func == IO_FUNC_NONE) continue;

                    if (func >= IO_FUNC_JOG_XP && func <= IO_FUNC_JOG_CN) {
                        /* Pendant jog: start on rising edge, stop on falling */
                        int axis = (func - IO_FUNC_JOG_XP) / 2;
                        int8_t dir = ((func - IO_FUNC_JOG_XP) % 2 == 0) ? 1 : -1;

                        if (active) {
                            uint32_t jogSpeed = g_pluginConfig.ioJogSpeed;
                            if (jogSpeed < 1) jogSpeed = 5000;
                            g_network->SendJogCommand((uint8_t)axis, dir, jogSpeed);
                            g_ioJogActive[axis] = true;
                            Log("IOPendant: jog axis %d dir %d speed %u",
                                axis, dir, jogSpeed);
                        } else {
                            g_network->SendJogStop((uint8_t)axis);
                            g_ioJogActive[axis] = false;
                        }
                    } else if (func >= 1000 && active && DoButton) {
                        /* DoButton trigger on rising edge only */
                        DoButton((short)func);
                        Log("IOModule: input %d -> DoButton(%u)", i, func);
                    }
                }
                g_prevIOInputs = ioInputs;
            }

            /* Forward Mach3 output signals to I/O module outputs.
             * Map EXTACT1-5 to I/O module outputs 0-4. */
            uint16_t ioOutputs = 0;
            for (int i = 0; i < 5 && i < 16; i++) {
                if (Engine->OutSigs[EXTACT1 + i].active &&
                    Engine->OutSigs[EXTACT1 + i].Activated)
                    ioOutputs |= (1 << i);
            }
            /* Send I/O control to I/O module if outputs changed */
            static uint16_t prevIOOutputs = 0;
            if (ioOutputs != prevIOOutputs) {
                prevIOOutputs = ioOutputs;
                wcnc_io_control_packet_t ioPkt;
                memset(&ioPkt, 0, sizeof(ioPkt));
                ioPkt.misc_outputs = (uint8_t)(ioOutputs & 0xFF);
                wcnc_finalize_packet(&ioPkt, WCNC_PKT_IO_CONTROL,
                                      sizeof(ioPkt) - sizeof(wcnc_header_t), 0, 0);
                g_ioModule->SendIOControlPacket(&ioPkt);
            }
        }
    }

    /* Send queued segments */
    g_bufferMgr->ProcessSendQueue();
}

/* ===================================================================
 * Jog Control (modeled after SmoothStepper JogOn/JogOff)
 *
 * SS reads MainPlanner->Velocities[axis] when speed==0.
 * SS sets Engine->Jogging, Engine->Axis[].Jogging, etc.
 * SS does NOT write to Engine->Axis[].MaxVelocity.
 * =================================================================== */

void piJogOnImpl(short axis, short dir, double speed)
{
    if (!g_enabled || axis < 0 || axis >= MAX_AXES) return;

    /* SS check: don't jog if trajectory is active */
    if (Engine->TrajHead != Engine->TrajIndex) {
        Log("JogOn: rejected, trajectory active (Head=%d != Idx=%d)",
            Engine->TrajHead, Engine->TrajIndex);
        return;
    }

    /* SS check: don't jog slave axes independently */
    if (Engine->Axis[axis].Slave || g_pluginConfig.axis[axis].cloneMaster >= 0) {
        Log("JogOn: rejected, axis %d is a slave/clone", axis);
        return;
    }

    /* Compute jog speed:
     * If speed != 0: Mach3 passes computed velocity (shift+arrow = full speed)
     * If speed == 0: keyboard jog without shift — use plugin config velocity */
    double jogVelUnitsPerSec;
    if (speed > 0.001) {
        jogVelUnitsPerSec = speed;
    } else {
        /* Keyboard jog: use plugin's own stored velocity (not MainPlanner) */
        jogVelUnitsPerSec = g_axisConfig[axis].velUnitsPerSec;
    }

    double stepsPerUnit = g_axisConfig[axis].stepsPerUnit;

    uint32_t steps_per_sec = (uint32_t)(jogVelUnitsPerSec * stepsPerUnit);
    if (steps_per_sec < 1) steps_per_sec = 1;

    /* Clamp to max velocity */
    uint32_t maxRate = g_axisConfig[axis].maxStepsPerSec;
    if (maxRate > 0 && steps_per_sec > maxRate) {
        steps_per_sec = maxRate;
    }

    /* Set Engine state (like SmoothStepper) */
    Engine->Jogging = true;
    Engine->Axis[axis].Jogging = true;
    Engine->Axis[axis].JoggDir = dir;
    Engine->Axis[axis].Dec = false;

    /* Mach3 dir: 0=negative, 1=positive.  Protocol: -1/+1 */
    int8_t protocol_dir = (dir == 0) ? -1 : 1;

    Log("JogOn: axis=%d dir=%d speed=%.3f -> vel=%.4f u/s * %.1f spu = %u steps/sec",
        axis, dir, speed, jogVelUnitsPerSec, stepsPerUnit, steps_per_sec);

    g_network->SendJogCommand((uint8_t)axis, protocol_dir, steps_per_sec);

    /* Forward jog to any axes cloned from this one */
    for (int i = 0; i < MAX_AXES; i++) {
        if (g_pluginConfig.axis[i].cloneMaster == axis && i != axis) {
            int8_t slave_dir = protocol_dir;
            if (g_pluginConfig.axis[i].cloneReversed) slave_dir = -slave_dir;

            uint32_t slave_sps = steps_per_sec;
            if (g_axisConfig[i].stepsPerUnit > 0.0 && stepsPerUnit > 0.0) {
                slave_sps = (uint32_t)(jogVelUnitsPerSec * g_axisConfig[i].stepsPerUnit);
                if (slave_sps < 1) slave_sps = 1;
            }

            Engine->Axis[i].Jogging = true;
            Engine->Axis[i].JoggDir = (slave_dir > 0) ? 1 : 0;
            g_network->SendJogCommand((uint8_t)i, slave_dir, slave_sps);
            Log("JogOn: clone axis %d (dir=%d, sps=%u)", i, slave_dir, slave_sps);
        }
    }
}

void piJogOffImpl(short axis)
{
    if (!g_enabled) return;

    /* Clear Engine jog state (like SmoothStepper JogOff) */
    if (axis >= 0 && axis < MAX_AXES) {
        Engine->Axis[axis].Jogging = false;
    }

    g_network->SendJogStop((uint8_t)axis);
    Log("JogOff: axis=%d", axis);

    /* Stop any axes cloned from this one */
    if (axis >= 0 && axis < MAX_AXES) {
        for (int i = 0; i < MAX_AXES; i++) {
            if (g_pluginConfig.axis[i].cloneMaster == axis && i != axis) {
                Engine->Axis[i].Jogging = false;
                g_network->SendJogStop((uint8_t)i);
                Log("JogOff: clone axis %d", i);
            }
        }
    }

    /* Check if any axis is still jogging */
    bool anyJogging = false;
    for (int i = 0; i < MAX_AXES; i++) {
        if (Engine->Axis[i].Jogging) anyJogging = true;
    }
    if (!anyJogging) {
        Engine->Jogging = false;
    }
}

/* ===================================================================
 * Homing (modeled after SmoothStepper Home)
 *
 * SS sets Engine->Axis[axis].Homing = true, delegates to FPGA.
 * On completion (via Update), zeroes position and sets Referenced.
 * =================================================================== */

void piHomeImpl(short axis)
{
    if (!g_enabled) return;

    /* Mach3's "Ref All Home" calls piHome separately for each axis (Z, Y, X)
     * within the same millisecond. The ESP32 can only home one axis at a time.
     *
     * We collect axes into a sequential queue (preserving Mach3's call order,
     * which respects the user's configured homing order). After a 50ms quiet
     * period, piUpdateImpl sends them one at a time.
     *
     * Slaved axes (e.g. A slaved to Y) are automatically included when
     * their master is sent — no need to queue them separately. */
    /* Helper lambda: is this axis a slave (either Mach3 or plugin clone)? */
    auto isSlaveAxis = [](int i) -> bool {
        if (Engine->Axis[i].Slave) return true;
        if (g_pluginConfig.axis[i].cloneMaster >= 0) return true;
        return false;
    };

    if (axis < 0) {
        /* Home all enabled axes — add in reverse order (Z first = standard) */
        for (int i = MAX_AXES - 1; i >= 0; i--) {
            if (Engine->Axis[i].Enable && !isSlaveAxis(i)) {
                /* Skip slaves — they'll be auto-included with their master */
                bool alreadyQueued = false;
                for (int q = 0; q < g_homeQueueLen; q++)
                    if (g_homeQueue[q] == i) { alreadyQueued = true; break; }
                if (!alreadyQueued && g_homeQueueLen < MAX_AXES) {
                    g_homeQueue[g_homeQueueLen++] = i;
                    Engine->Axis[i].Homing = true;
                }
            }
        }
    } else if (axis < MAX_AXES) {
        /* Single axis — skip if it's a slave (master will include it) */
        if (!isSlaveAxis(axis)) {
            bool alreadyQueued = false;
            for (int q = 0; q < g_homeQueueLen; q++)
                if (g_homeQueue[q] == axis) { alreadyQueued = true; break; }
            if (!alreadyQueued && g_homeQueueLen < MAX_AXES) {
                g_homeQueue[g_homeQueueLen++] = axis;
                Engine->Axis[axis].Homing = true;
            }
        }
    }

    g_homeCollectTick = GetTickCount();
    if (g_homeState == HOME_IDLE)
        g_homeState = HOME_COLLECTING;

    Log("Home: queued axis=%d, queueLen=%d, state=%d", axis, g_homeQueueLen, g_homeState);
}

/* ===================================================================
 * Reset (modeled after SmoothStepper Reset - 283 lines)
 *
 * SS: stops motion, resets state machines, syncs positions from HW,
 * clears homing flags, resets peripherals.
 * =================================================================== */

void piResetImpl()
{
    if (!g_network) return;

    Log("Reset: EStop=%d", (int)Engine->EStop);

    /* Send E-Stop (immediate halt) then Reset (clear state).
     * The flag prevents the status handler from calling DoButton(OEM_ESTOP)
     * when it sees the ESTOP state we just caused — that would re-trigger
     * piResetImpl() in a feedback loop. */
    g_pluginResetActive = true;
    g_network->SendEStop();
    g_network->SendReset();

    /* Drain trajectory buffer — discard all pending moves */
    if (Engine) {
        Engine->TrajIndex = Engine->TrajHead;
        g_lastTrajIndex   = Engine->TrajHead;
        Engine->DwellTime = 0;
        Engine->Sync      = true;
    }

    /* Clear jog state */
    Engine->Jogging = false;
    for (int i = 0; i < MAX_AXES; i++) {
        Engine->Axis[i].Jogging = false;
        Engine->Axis[i].Homing = false;
    }

    /* Flush local buffers */
    if (g_bufferMgr) g_bufferMgr->Flush();

    /* Reset VMS trajectory accumulator */
    ResetTrajAccum();

    /* Reset tracked output state.
     * E-Stop/Reset stops everything on the ESP32.
     * Config dialog reads from ESP32 status so it will update
     * automatically when ESP32 confirms outputs are off. */
    g_sentSpindle = 0;
    g_sentCoolant = 0;
    g_sentMiscOut = 0;

    /* Reset homing state */
    g_homeState = HOME_IDLE;
    g_homeQueueLen = 0;
    g_homeQueueIdx = 0;
    g_activeHomeMask  = 0;
    g_prevMachineState = 0xFF;

    /* Re-sync position offsets on next status (Mach3 may have
     * changed positions during reset) */
    g_posInitialized = false;

    /* Reset misc input edge detection */
    for (int i = 0; i < 4; i++)
        g_prevMiscInput[i] = false;

    /* Reset reconnect state */
    g_reconnectAttempts = 0;
    g_nextReconnectTick = 0;

    /* Stop any active I/O module pendant jogs */
    for (int i = 0; i < MAX_AXES; i++)
        g_ioJogActive[i] = false;
    g_prevIOInputs = 0;
}

/* ===================================================================
 * Probe
 * =================================================================== */

void piProbeImpl()
{
    if (!g_enabled) return;
    Log("Probe started");
    /* Probe mode flag — the next motion segment emitted in piUpdateImpl
     * will have the probe flag set. */
    g_segBuilder->SetProbeMode(true);
}

/* ===================================================================
 * Dwell (modeled after SmoothStepper DoDwell - 18 lines)
 *
 * SS delegates dwell timing entirely to FPGA (CMD_DWELL_LO/HI).
 * We send it as a zero-motion segment to ESP32.
 * =================================================================== */

void piDoDwellImpl(double seconds)
{
    if (!g_enabled) return;
    Log("Dwell: %.3f sec", seconds);
    if (g_bufferMgr) {
        wcnc_motion_segment_t dwellSeg;
        memset(&dwellSeg, 0, sizeof(dwellSeg));
        dwellSeg.duration_us = (uint32_t)(seconds * 1000000.0);
        dwellSeg.flags = WCNC_SEG_FLAG_EXACT_STOP;
        g_bufferMgr->QueueSegment(&dwellSeg);
    }

    /* Clear DwellTime and set Sync so Mach3 continues generating
     * trajectory data.  Without clearing DwellTime, Mach3 stalls
     * thinking a dwell is still in progress.  Without Sync, Mach3
     * may not resync after the dwell to generate the next batch. */
    if (Engine) {
        Engine->DwellTime = 0;
        Engine->Sync = true;
    }
}

/* ===================================================================
 * Purge (modeled after SmoothStepper Purge - 62 lines)
 *
 * SS: sends STOP_ALL, drains traj queue, resets motion state.
 * =================================================================== */

void piPurgeImpl()
{
    Log("Purge");

    /* Stop motion on ESP32 */
    if (g_network && g_connected) {
        g_network->SendFeedHold();
    }

    /* Flush local segment queue */
    if (g_bufferMgr) {
        g_bufferMgr->Flush();
    }

    /* Drain trajectory queue (like SS: TrajHead = TrajIndex) */
    if (Engine) {
        Engine->TrajIndex = Engine->TrajHead;
        g_lastTrajIndex = Engine->TrajHead;
    }

    /* Reset VMS trajectory accumulator */
    ResetTrajAccum();
}

/* ===================================================================
 * Notify (modeled after SmoothStepper Notify - 501 lines)
 *
 * SS handles: config changed, units changed, E-stop, axis params,
 * DRO update, G-code mode changes.
 * =================================================================== */

void piNotifyImpl(int msg)
{
    switch (msg) {
    case EX_DDA:
        Log("Notify: EX_DDA (mode select)");
        break;

    case EX_VMS:
        Log("Notify: EX_VMS (mode select)");
        break;

    case EX_COMMAND:
        Log("Notify: EX_COMMAND (mode select)");
        break;

    case EX_SPINON:
        Engine->SpindleStable = true;
        {
            /* Mach3 sets Engine->SpindleCW/CCW before this notification.
             * Read direction and forward to ESP32. */
            uint8_t spState = 1;  /* default CW */
            if (Engine->SpindleCCW && !Engine->SpindleCW) spState = 2;

            SendIOControl(g_prevMiscOutputs, spState, 0, g_sentCoolant);

            Log("Notify: EX_SPINON -> spState=%d", spState);
        }
        break;

    case EX_SPINOFF:
        Engine->SpindleStable = true;
        {
            SendIOControl(g_prevMiscOutputs, 0, 0, g_sentCoolant);

            Log("Notify: EX_SPINOFF -> spindle OFF");
        }
        break;

    case EX_SPINSPEED:
        Engine->SpindleStable = true;
        Log("Notify: EX_SPINSPEED");
        /* RPM is forwarded to ESP32 via the SPINAXISSPEED (22) trajectory
         * command in HandleTrajCommand, which carries the actual RPM value.
         * EX_SPINSPEED is just a notification that speed changed. */
        break;

    case EX_MOTORTUNED:
        Log("Notify: EX_MOTORTUNED");
        break;

    case EX_SETUP:
        /* Re-apply plugin config on setup (like SS reconfigures FPGA) */
        Log("Notify: EX_SETUP, re-applying plugin config");
        ApplyPluginConfig();
        for (int i = 0; i < MAX_AXES; i++) {
            if (g_axisConfig[i].valid) {
                g_segBuilder->SetAxisAccel(i, g_axisConfig[i].accelStepsPerSec2);
            }
        }
        if (g_connected) {
            SyncConfigToESP32();
        }
        break;

    case EX_FEEDHOLD:
        Log("Notify: EX_FEEDHOLD");
        if (g_enabled && g_network) g_network->SendFeedHold();
        break;

    case EX_RUN:
        Log("Notify: EX_RUN");
        if (g_enabled && g_network) g_network->SendFeedResume();
        break;

    case EX_ESTOP:
        Log("Notify: EX_ESTOP");
        if (g_network) g_network->SendEStop();
        break;

    case EX_CONFIG:
        Log("Notify: EX_CONFIG");
        break;

    default:
        Log("Notify: msg=%d (unknown)", msg);
        break;
    }
}

/* ===================================================================
 * Configuration Dialog
 * =================================================================== */

void OnConfigChanged()
{
    Log("Config changed (from dialog), applying...");
    PluginConfig_SaveToRegistry(&g_pluginConfig);
    ApplyPluginConfig();

    /* Update segment builder with new accel values */
    for (int i = 0; i < MAX_AXES; i++) {
        if (g_axisConfig[i].valid)
            g_segBuilder->SetAxisAccel(i, g_axisConfig[i].accelStepsPerSec2);
    }

    /* Sync to ESP32 if connected */
    if (g_connected) {
        SyncConfigToESP32();
    }
}

void piConfigImpl()
{
    extern HINSTANCE g_hInstance;

    Log("Opening config dialog...");
    ShowConfigDialog(g_hInstance, &g_pluginConfig, g_network, g_mach3MainWnd);
}

/* ===================================================================
 * Shutdown
 * =================================================================== */

void piStopPlugImpl()
{
    Log("piStopPlug()");
    g_enabled = false;

    /* Remove Mach3 main window subclass */
    if (g_mach3MainWnd && g_origMach3WndProc) {
        SetWindowLongPtrA(g_mach3MainWnd, GWLP_WNDPROC, (LONG_PTR)g_origMach3WndProc);
        /* Remove our menu item */
        HMENU hMenu = GetMenu(g_mach3MainWnd);
        if (hMenu) {
            RemoveMenu(hMenu, IDM_WIFICNC_SETTINGS, MF_BYCOMMAND);
            DrawMenuBar(g_mach3MainWnd);
        }
        g_origMach3WndProc = nullptr;
        g_mach3MainWnd = nullptr;
    }

    /* Close floating config dialog if open */
    CloseConfigDialog();

    if (g_ioModule) {
        g_ioModule->Disconnect();
        delete g_ioModule;
        g_ioModule = nullptr;
        g_ioConnected = false;
    }

    if (g_network) {
        g_network->Disconnect();
        delete g_network;
        g_network = nullptr;
    }

    delete g_segBuilder;
    g_segBuilder = nullptr;

    delete g_bufferMgr;
    g_bufferMgr = nullptr;

    PluginConfig_SaveToRegistry(&g_pluginConfig);
    LogClose();
}
