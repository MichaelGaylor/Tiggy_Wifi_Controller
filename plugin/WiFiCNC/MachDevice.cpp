/*
 * WiFi CNC Controller - Mach3 Device Interface
 *
 * DLL exports that Mach3 expects. Non-prefixed names (InitControl, Update, etc.)
 * These delegate to the *Impl() functions in Plugin.cpp.
 */

#include "MachDevice.h"
#include <cstdint>
#include <cstring>
#include <cfloat>

/* ===================================================================
 * Global State (injected by Mach3 via InitControl and Set* exports)
 * =================================================================== */

TrajBuffer      *Engine       = nullptr;
MachDoButton     DoButton     = nullptr;
MachGetDRO       GetDRO       = nullptr;
MachSetDRO       SetDRO       = nullptr;
MachGetLED       GetLED       = nullptr;
MachSetLED       SetLED       = nullptr;
MachCode         Code         = nullptr;
MachGetMenuRange GetMenuRange = nullptr;

/* DLL instance handle (for creating dialogs) */
HINSTANCE g_hInstance = nullptr;

/* Raw Engine pointer - preserved before any casting for diagnostics */
void *RawEnginePtr = nullptr;

/* Additional Mach3 objects from InitControl (stored for completeness) */
void *MachSetup      = nullptr;   /* setup* in SDK */
void *MainPlanner    = nullptr;   /* _TrajectoryControl* in SDK */
void *MachView       = nullptr;   /* _CMach4View* in SDK */

/* Plugin name returned by SetProName */
static char g_pluginName[] = "Tiggy Motion Controller";

/* Offset of m_PrinterOn found at runtime (for diagnostics) */
int g_printerOnOffset = -1;
ptrdiff_t g_plannerOffset = 0;

/* ===================================================================
 * DLL Exports - Mach3 API Injection
 * =================================================================== */

extern "C" __declspec(dllexport) void SetGetMenuRange(MachGetMenuRange pFunc)
{
    GetMenuRange = pFunc;
}

extern "C" __declspec(dllexport) void SetDoButton(MachDoButton pFunc)
{
    DoButton = pFunc;
}

extern "C" __declspec(dllexport) void SetGetDRO(MachGetDRO pFunc)
{
    GetDRO = pFunc;
}

extern "C" __declspec(dllexport) void SetSetDRO(MachSetDRO pFunc)
{
    SetDRO = pFunc;
}

extern "C" __declspec(dllexport) void SetGetLED(MachGetLED pFunc)
{
    GetLED = pFunc;
}

extern "C" __declspec(dllexport) void SetSetLED(MachSetLED pFunc)
{
    SetLED = pFunc;
}

extern "C" __declspec(dllexport) void SetCode(MachCode pFunc)
{
    Code = pFunc;
}

/* ===================================================================
 * DLL Exports - Plugin Lifecycle
 * =================================================================== */

extern "C" __declspec(dllexport) bool InitControl(void *oEngine,
                                                    void *oSetup,
                                                    void *oMainPlanner,
                                                    void *oView)
{
    RawEnginePtr  = oEngine;
    Engine        = static_cast<TrajBuffer*>(oEngine);
    MachSetup     = oSetup;
    MainPlanner   = oMainPlanner;
    MachView      = oView;

    /*
     * Tell Mach3 this is an external motion controller by clearing
     * CMach4View::m_PrinterOn.
     *
     * We don't link MFC, so we can't use MachView->m_PrinterOn directly.
     * Instead, we find the offset at runtime using the relationship:
     *   MainPlanner = &MachView->Planner  (TrajectoryControl member)
     *
     * From the SDK's Mach4View.h, m_PrinterOn is approximately 160-250
     * bytes before the Planner member.  We scan that region for the
     * pattern: [bool=1] [pad] [3 valid pointers]  (matching m_PrinterOn
     * followed by m_ShiftState, m_CtrlState, MachFrame).
     *
     * The 8 bytes before m_PrinterOn are double m_cyMin (a screen
     * coordinate), providing an extra sanity check.
     */
    if (oView && oMainPlanner) {
        g_plannerOffset = (uint8_t*)oMainPlanner - (uint8_t*)oView;

        ptrdiff_t searchStart = (g_plannerOffset > 500) ? g_plannerOffset - 500 : 64;
        ptrdiff_t searchEnd   = (g_plannerOffset > 80)  ? g_plannerOffset - 80  : 0;

        __try {
            for (ptrdiff_t off = searchStart; off < searchEnd; off++) {
                uint8_t *p = (uint8_t *)oView + off;
                if (*p != 1) continue;

                /* Check 3 consecutive DWORD-aligned pointers at +4, +8, +12 */
                uint32_t p1 = *(uint32_t *)(p + 4);
                uint32_t p2 = *(uint32_t *)(p + 8);
                uint32_t p3 = *(uint32_t *)(p + 12);

                if (p1 < 0x10000u || p1 > 0x7FFFFFFFu) continue;
                if (p2 < 0x10000u || p2 > 0x7FFFFFFFu) continue;
                if (p3 < 0x10000u || p3 > 0x7FFFFFFFu) continue;

                /* Extra check: 8 bytes before should be a finite double
                 * (m_cyMin screen coordinate) */
                double dval;
                memcpy(&dval, p - 8, sizeof(double));
                if (dval != dval) continue;             /* NaN check */
                if (dval > 1e15 || dval < -1e15) continue; /* plausibility */

                /* Found a strong candidate */
                *p = 0;  /* m_PrinterOn = false */
                g_printerOnOffset = (int)off;
                break;
            }
        } __except(EXCEPTION_EXECUTE_HANDLER) {
            /* memory probe faulted — ignore */
        }
    }

    piInitControlImpl();
    return true;  /* SDK convention: always return true */
}

extern "C" __declspec(dllexport) char* SetProName(const char *name)
{
    (void)name;
    piSetProNameImpl(g_pluginName);
    return g_pluginName;
}

extern "C" __declspec(dllexport) void PostInitControl()
{
    piPostInitControlImpl();
}

extern "C" __declspec(dllexport) void Update()
{
    piUpdateImpl();
}

extern "C" __declspec(dllexport) void Config()
{
    piConfigImpl();
}

extern "C" __declspec(dllexport) void StopPlug()
{
    piStopPlugImpl();
}

extern "C" __declspec(dllexport) void JogOn(short axis, short dir, double speed)
{
    piJogOnImpl(axis, dir, speed);
}

extern "C" __declspec(dllexport) void JogOff(short axis)
{
    piJogOffImpl(axis);
}

extern "C" __declspec(dllexport) void Home(short axis)
{
    piHomeImpl(axis);
}

extern "C" __declspec(dllexport) void Reset()
{
    piResetImpl();
}

extern "C" __declspec(dllexport) void Probe()
{
    piProbeImpl();
}

extern "C" __declspec(dllexport) void DoDwell(double seconds)
{
    piDoDwellImpl(seconds);
}

extern "C" __declspec(dllexport) void Purge(short flags)
{
    (void)flags;
    piPurgeImpl();
}

extern "C" __declspec(dllexport) void Notify(int msg)
{
    piNotifyImpl(msg);
}

/* ===================================================================
 * COM Stub Exports
 *
 * Every Mach3 plugin (including SmoothStepper, Flash, Video, etc.)
 * exports these 4 COM functions. The official Mach3 SDK is COM-based.
 * Mach3 may use these to enumerate/identify plugins, especially
 * motion controller plugins. We provide stubs.
 * =================================================================== */

extern "C" __declspec(dllexport) HRESULT DllCanUnloadNow(void)
{
    return S_FALSE;  /* Don't unload - we're in use */
}

extern "C" __declspec(dllexport) HRESULT DllGetClassObject(
    const void *rclsid, const void *riid, void **ppv)
{
    (void)rclsid;
    (void)riid;
    if (ppv) *ppv = nullptr;
    return 0x80040111L;  /* CLASS_E_CLASSNOTAVAILABLE */
}

extern "C" __declspec(dllexport) HRESULT DllRegisterServer(void)
{
    return S_OK;
}

extern "C" __declspec(dllexport) HRESULT DllUnregisterServer(void)
{
    return S_OK;
}

/* ===================================================================
 * DLL Entry Point
 * =================================================================== */

BOOL APIENTRY DllMain(HMODULE hModule, DWORD reason, LPVOID lpReserved)
{
    (void)lpReserved;

    switch (reason) {
    case DLL_PROCESS_ATTACH:
        g_hInstance = (HINSTANCE)hModule;
        break;
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
    case DLL_PROCESS_DETACH:
        break;
    }
    return TRUE;
}
