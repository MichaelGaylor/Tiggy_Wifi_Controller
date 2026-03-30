/*
 * Mach3 Plugin Interface (Reconstructed)
 *
 * Defines the function pointer types and DLL export conventions
 * for Mach3 external motion control plugins.
 *
 * Based on the official Mach3 Plugin SDK (Edward Bryson / Joshua 1 Systems).
 */

#ifndef MACH3_PLUGIN_H
#define MACH3_PLUGIN_H

/* Engine.h defines TrajBuffer, AxisInfo, InputInfo, etc. (all plain C) */
#include "Engine.h"

/* ===================================================================
 * Function Pointer Types (Mach3 -> Plugin injection)
 *
 * Mach3 calls Set*() exports to give the plugin access to its API.
 * The plugin stores these pointers and uses them to interact with Mach3.
 * All use __cdecl calling convention (Mach3 standard).
 * =================================================================== */

typedef void (_cdecl *MachDoButton)(short buttonCode);
typedef double (_cdecl *MachGetDRO)(short droNumber);
typedef void (_cdecl *MachSetDRO)(short droNumber, double value);
typedef bool (_cdecl *MachGetLED)(short ledNumber);
typedef void (_cdecl *MachSetLED)(short ledNumber, bool state);
typedef void (_cdecl *MachCode)(const char *gcode);
typedef int (_cdecl *MachGetMenuRange)(short menuId);

/* ===================================================================
 * Globals (defined in MachDevice.cpp, used by Plugin.cpp)
 * =================================================================== */

extern TrajBuffer       *Engine;
extern MachDoButton      DoButton;
extern MachGetDRO        GetDRO;
extern MachSetDRO        SetDRO;
extern MachGetLED        GetLED;
extern MachSetLED        SetLED;
extern MachCode          Code;
extern MachGetMenuRange  GetMenuRange;

/* Convenience aliases */
#ifndef MAX_AXES
#define MAX_AXES 6
#endif

/* Ring buffer sizing (not in SDK Engine.h) */
#ifndef TRAJ_BUFFER_SIZE
#define TRAJ_BUFFER_SIZE  0x1000
#define TRAJ_BUFFER_MASK  0x0FFF
#endif

/* ===================================================================
 * Plugin Implementation Functions (in Plugin.cpp)
 *
 * Named *Impl() to avoid clashing with the non-prefixed DLL exports
 * in MachDevice.cpp. The DLL exports call these.
 * =================================================================== */

void piInitControlImpl(void);
void piSetProNameImpl(char *name);
void piPostInitControlImpl(void);
void piUpdateImpl(void);
void piConfigImpl(void);
void piStopPlugImpl(void);
void piJogOnImpl(short axis, short dir, double speed);
void piJogOffImpl(short axis);
void piHomeImpl(short axis);
void piResetImpl(void);
void piProbeImpl(void);
void piDoDwellImpl(double seconds);
void piPurgeImpl(void);
void piNotifyImpl(int msg);

/* ===================================================================
 * Notification Messages (piNotify codes)
 *
 * From SDK TrajectoryControl.h enum:
 *   EX_DDA=0, EX_VMS=1, EX_COMMAND=2, EX_SPINON=3, EX_SPINOFF=4,
 *   EX_SPINSPEED=5, EX_MOTORTUNED=6, EX_SETUP=7, EX_FEEDHOLD=8,
 *   EX_RUN=9, EX_ESTOP=10, EX_CONFIG=11
 * =================================================================== */

#define EX_DDA          0
#define EX_VMS          1
#define EX_COMMAND      2
#define EX_SPINON       3
#define EX_SPINOFF      4
#define EX_SPINSPEED    5
#define EX_MOTORTUNED   6
#define EX_SETUP        7
#define EX_FEEDHOLD     8
#define EX_RUN          9
#define EX_ESTOP        10
#define EX_CONFIG       11

/* ===================================================================
 * DRO Numbers (commonly used)
 * =================================================================== */

#define DRO_X_POS       800
#define DRO_Y_POS       801
#define DRO_Z_POS       802
#define DRO_A_POS       803
#define DRO_B_POS       804
#define DRO_C_POS       805

#define DRO_X_MACH_POS  810
#define DRO_Y_MACH_POS  811
#define DRO_Z_MACH_POS  812
#define DRO_A_MACH_POS  813
#define DRO_B_MACH_POS  814
#define DRO_C_MACH_POS  815

/* ===================================================================
 * OEM Button Codes (commonly used)
 * =================================================================== */

#define OEM_ESTOP       1021
#define OEM_RESET       1022
#define OEM_CYCLE_START 1000

#endif /* MACH3_PLUGIN_H */
