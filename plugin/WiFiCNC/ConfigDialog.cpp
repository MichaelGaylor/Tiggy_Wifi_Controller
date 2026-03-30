/*
 * WiFi CNC Controller - Configuration Dialog
 *
 * Win32 PropertySheet with 6 tabs, built at runtime (no .rc file).
 *
 * Tab 1: Connection   - IP address, status, connect/disconnect, spindle RPM
 * Tab 2: Axis Config  - Per-axis steps/unit, velocity, acceleration
 * Tab 3: Inputs/Outs  - Live input monitoring + inversion checkboxes
 * Tab 4: Advanced     - Timing, step/dir inversion, homing, spindle
 * Tab 5: I/O Module   - I/O expansion module config (inputs + outputs)
 * Tab 6: Pin Map      - Board selection + GPIO pin display
 */

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <commctrl.h>

#include "ConfigDialog.h"
#include "Plugin.h"
#include "NetworkClient.h"
#include "PinMap.h"
#include "MachIncludes/Mach3Plugin.h"
#include "../../protocol/wifi_cnc_protocol.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

#pragma comment(lib, "comctl32.lib")

/* Log function defined in Plugin.cpp */
extern void Log(const char *fmt, ...);

/* ===================================================================
 * Shared State (passed to all pages via lParam)
 * =================================================================== */

struct DialogState {
    PluginConfig *cfg;
    PluginConfig  editCfg;     /* Working copy for editing */
    NetworkClient *network;
    bool changed;

    /* Pin map (loaded from .pinmap files) */
    char pinmapFolder[260];     /* Folder containing .pinmap files */
    int  boardCount;
    char boardNames[PINMAP_MAX_BOARDS][PINMAP_MAX_NAME];
    char boardPaths[PINMAP_MAX_BOARDS][260];
    PinMapEntry currentPinMap;
};

static DialogState *s_state = nullptr;

/* ===================================================================
 * Runtime Dialog Template Builder
 *
 * Creates DLGTEMPLATE + DLGITEMTEMPLATE structs in memory.
 * This avoids needing an .rc resource file.
 * =================================================================== */

/* Align pointer to DWORD boundary */
static LPWORD AlignDWord(LPWORD p)
{
    ULONG_PTR ul = (ULONG_PTR)p;
    ul = (ul + 3) & ~3;
    return (LPWORD)ul;
}

/* Add a dialog item (control) to the template buffer */
static LPWORD AddDialogItem(LPWORD p, DWORD style, short x, short y,
                             short cx, short cy, WORD id, WORD classAtom,
                             const wchar_t *text)
{
    p = AlignDWord(p);

    DLGITEMTEMPLATE *item = (DLGITEMTEMPLATE *)p;
    item->style = style | WS_CHILD | WS_VISIBLE;
    item->dwExtendedStyle = 0;
    item->x = x;
    item->y = y;
    item->cx = cx;
    item->cy = cy;
    item->id = id;

    p = (LPWORD)(item + 1);

    /* Class (atom) */
    *p++ = 0xFFFF;
    *p++ = classAtom;

    /* Title text */
    if (text) {
        int len = (int)wcslen(text) + 1;
        memcpy(p, text, len * sizeof(wchar_t));
        p += len;
    } else {
        *p++ = 0;
    }

    /* Creation data */
    *p++ = 0;

    return p;
}

/* Create a basic dialog template in a buffer */
static LPDLGTEMPLATE CreatePageTemplate(int numItems, short w, short h)
{
    /* Allocate generous buffer */
    size_t bufSize = sizeof(DLGTEMPLATE) + 256 + numItems * 128;
    HGLOBAL hGlobal = GlobalAlloc(GMEM_ZEROINIT, bufSize);
    LPDLGTEMPLATE tpl = (LPDLGTEMPLATE)GlobalLock(hGlobal);

    tpl->style = WS_CHILD | DS_SETFONT | DS_CONTROL;
    tpl->dwExtendedStyle = 0;
    tpl->cdit = 0;
    tpl->x = 0;
    tpl->y = 0;
    tpl->cx = w;
    tpl->cy = h;

    LPWORD p = (LPWORD)(tpl + 1);
    *p++ = 0;  /* menu */
    *p++ = 0;  /* class */
    *p++ = 0;  /* title */

    /* Font (DS_SETFONT) */
    *p++ = 8;  /* point size */
    const wchar_t *fontName = L"MS Shell Dlg";
    int fontLen = (int)wcslen(fontName) + 1;
    memcpy(p, fontName, fontLen * sizeof(wchar_t));

    GlobalUnlock(hGlobal);
    return (LPDLGTEMPLATE)hGlobal;
}

/* Win32 class atoms for common controls */
#define CLS_STATIC    0x0082
#define CLS_EDIT      0x0081
#define CLS_BUTTON    0x0080
#define CLS_LISTBOX   0x0083

/* ===================================================================
 * Control IDs
 * =================================================================== */

/* Connection tab */
#define IDC_IP_EDIT         1001
#define IDC_STATUS_LABEL    1002
#define IDC_CONNECT_BTN     1003
#define IDC_DISCONNECT_BTN  1004
#define IDC_DISCOVER_BTN    1005
#define IDC_CONN_RPM_LABEL  1006

/* Axis config tab - base IDs (+ axis * 10) */
#define IDC_AX_ENABLE_BASE  2000
#define IDC_AX_SPU_BASE     2001
#define IDC_AX_VEL_BASE     2002
#define IDC_AX_ACCEL_BASE   2003
#define IDC_AX_CLONE_BASE   2004  /* Combo: clone master axis */
#define IDC_AX_REVERSE_BASE 2005  /* Checkbox: reverse clone direction */
/* Axis 0: 2000-2005, Axis 1: 2010-2015, etc. */

/* Inputs tab */
#define IDC_INPUT_STATUS_BASE  3000  /* +0..13 for input labels (text only) */
#define IDC_INPUT_LED_BASE     3050  /* +0..13 for LED indicators (owner-drawn) */
#define IDC_INV_LIMIT_BASE     3100  /* +0..5 for limit inversion checkboxes */
#define IDC_INV_HOME_BASE      3110  /* +0..5 for home inversion */
#define IDC_INV_ESTOP          3120
#define IDC_INV_PROBE          3121
#define IDC_MISC_FUNC_BASE     3200  /* +0..3 for misc input function combos */
#define IDC_OUTPUT_LED_BASE    3250  /* +0..9 for output LED indicators */

/* Available functions for misc input mapping (DoButton codes) */
struct InputFuncDef { const char *name; uint16_t code; };
static const InputFuncDef g_inputFuncs[] = {
    { "None",         0    },
    { "Cycle Start",  1000 },
    { "Feed Hold",    1001 },
    { "Stop",         1003 },
    { "E-Stop",       1021 },
    { "Reset",        1022 },
    { "Rewind",       1010 },
    { "Single Block", 1002 },
};
static const int g_numInputFuncs = sizeof(g_inputFuncs) / sizeof(g_inputFuncs[0]);

/* Advanced tab */
#define IDC_STEP_PULSE     4001
#define IDC_DIR_SETUP      4002
#define IDC_INV_STEP_BASE  4010  /* +0..5 */
#define IDC_INV_DIR_BASE   4020  /* +0..5 */
#define IDC_HOME_DIR_BASE  4030  /* +0..5 */
#define IDC_HOME_SEEK      4040
#define IDC_HOME_FEED      4041
#define IDC_HOME_PULLOFF   4042
#define IDC_SPINDLE_FREQ   4050
#define IDC_SPINDLE_RPM    4051
#define IDC_CHARGE_FREQ    4060
#define IDC_IDLE_DELAY     4061
#define IDC_LASER_MODE     4062

/* Pin Map tab */
#define IDC_BOARD_COMBO    5000
#define IDC_BOARD_DESC     5001
#define IDC_PINMAP_REFRESH 5002
#define IDC_PINMAP_NOTE    5003
#define IDC_PINMAP_DISPLAY 5010  /* base for pin display labels */

/* I/O Module tab */
#define IDC_IO_ENABLE       6000
#define IDC_IO_IP_EDIT      6001
#define IDC_IO_STATUS_LABEL 6002
#define IDC_IO_FUNC_BASE    6100  /* +0..15 for I/O input function combos */
#define IDC_IO_OUT_BASE     6200  /* +0..15 for I/O output function combos */

/* Available functions for I/O module input mapping */
struct IOInputFuncDef { const char *name; uint16_t code; };
static const IOInputFuncDef g_ioInputFuncs[] = {
    { "None",         0    },
    { "Jog X+",       101  },
    { "Jog X-",       102  },
    { "Jog Y+",       103  },
    { "Jog Y-",       104  },
    { "Jog Z+",       105  },
    { "Jog Z-",       106  },
    { "Jog A+",       107  },
    { "Jog A-",       108  },
    { "Jog B+",       109  },
    { "Jog B-",       110  },
    { "Jog C+",       111  },
    { "Jog C-",       112  },
    { "Cycle Start",  1000 },
    { "Feed Hold",    1001 },
    { "Stop",         1003 },
    { "E-Stop",       1021 },
    { "Reset",        1022 },
};
static const int g_numIOInputFuncs = sizeof(g_ioInputFuncs) / sizeof(g_ioInputFuncs[0]);

/* Available functions for I/O module output mapping */
struct IOOutputFuncDef { const char *name; uint16_t code; };
static const IOOutputFuncDef g_ioOutputFuncs[] = {
    { "None",         0 },
    { "Spindle CW",   1 },
    { "Spindle CCW",  2 },
    { "Flood",        3 },
    { "Mist",         4 },
    { "Output 1",     5 },
    { "Output 2",     6 },
    { "Output 3",     7 },
    { "Output 4",     8 },
};
static const int g_numIOOutputFuncs = sizeof(g_ioOutputFuncs) / sizeof(g_ioOutputFuncs[0]);

/* Timer for live input updates */
#define IDT_INPUT_POLL  100

/* ===================================================================
 * Helper: Set/Get edit control text as number
 * =================================================================== */

static void SetEditDouble(HWND hDlg, int id, double val)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "%.4g", val);
    SetDlgItemTextA(hDlg, id, buf);
}

static double GetEditDouble(HWND hDlg, int id)
{
    char buf[32] = {0};
    GetDlgItemTextA(hDlg, id, buf, sizeof(buf));
    return atof(buf);
}

static void SetEditInt(HWND hDlg, int id, int val)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", val);
    SetDlgItemTextA(hDlg, id, buf);
}

static int GetEditInt(HWND hDlg, int id)
{
    char buf[16] = {0};
    GetDlgItemTextA(hDlg, id, buf, sizeof(buf));
    return atoi(buf);
}

/* ===================================================================
 * Tab 1: Connection
 * =================================================================== */

static INT_PTR CALLBACK ConnectionProc(HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam)
{
    switch (msg) {
    case WM_INITDIALOG:
    {
        if (lParam) {
            PROPSHEETPAGE *psp = (PROPSHEETPAGE *)lParam;
            SetWindowLongPtrA(hDlg, GWLP_USERDATA, (LONG_PTR)psp->lParam);
        }
        DialogState *ds = (DialogState *)GetWindowLongPtrA(hDlg, GWLP_USERDATA);
        if (!ds) return TRUE;

        SetDlgItemTextA(hDlg, IDC_IP_EDIT, ds->editCfg.espAddress);

        /* Update status label */
        if (ds->network && ds->network->IsConnected()) {
            SetDlgItemTextA(hDlg, IDC_STATUS_LABEL, "Status: CONNECTED");
        } else {
            SetDlgItemTextA(hDlg, IDC_STATUS_LABEL, "Status: DISCONNECTED");
        }

        /* Spindle encoder RPM */
        char rpmBuf[32];
        snprintf(rpmBuf, sizeof(rpmBuf), "Spindle RPM: %d", (int)GetSpindleRPM());
        SetDlgItemTextA(hDlg, IDC_CONN_RPM_LABEL, rpmBuf);

        /* Start live RPM update timer */
        SetTimer(hDlg, IDT_INPUT_POLL + 2, 500, nullptr);
        return TRUE;
    }

    case WM_TIMER:
        if (wParam == IDT_INPUT_POLL + 2) {
            char rpmBuf[32];
            snprintf(rpmBuf, sizeof(rpmBuf), "Spindle RPM: %d", (int)GetSpindleRPM());
            SetDlgItemTextA(hDlg, IDC_CONN_RPM_LABEL, rpmBuf);
        }
        return TRUE;

    case WM_COMMAND:
        switch (LOWORD(wParam)) {
        case IDC_CONNECT_BTN:
        {
            DialogState *ds = (DialogState *)GetWindowLongPtrA(hDlg, GWLP_USERDATA);
            if (!ds) break;
            GetDlgItemTextA(hDlg, IDC_IP_EDIT, ds->editCfg.espAddress,
                           sizeof(ds->editCfg.espAddress));
            /* Connection handled by caller after dialog closes */
            ds->changed = true;
            PropSheet_Changed(GetParent(hDlg), hDlg);
            break;
        }
        }
        return TRUE;

    case WM_NOTIFY:
    {
        NMHDR *hdr = (NMHDR *)lParam;
        DialogState *ds = (DialogState *)GetWindowLongPtrA(hDlg, GWLP_USERDATA);
        if (!ds) break;

        if (hdr->code == PSN_APPLY) {
            GetDlgItemTextA(hDlg, IDC_IP_EDIT, ds->editCfg.espAddress,
                           sizeof(ds->editCfg.espAddress));
            ds->changed = true;
            SetWindowLongPtrA(hDlg, DWLP_MSGRESULT, PSNRET_NOERROR);
            return TRUE;
        }
        break;
    }

    case WM_DESTROY:
        KillTimer(hDlg, IDT_INPUT_POLL + 2);
        return TRUE;
    }
    return FALSE;
}

/* ===================================================================
 * Tab 2: Axis Configuration
 * =================================================================== */

static INT_PTR CALLBACK AxisConfigProc(HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam)
{
    switch (msg) {
    case WM_INITDIALOG:
    {
        if (lParam) {
            PROPSHEETPAGE *psp = (PROPSHEETPAGE *)lParam;
            SetWindowLongPtrA(hDlg, GWLP_USERDATA, (LONG_PTR)psp->lParam);
        }
        DialogState *ds = (DialogState *)GetWindowLongPtrA(hDlg, GWLP_USERDATA);
        if (!ds) return TRUE;

        for (int i = 0; i < WCNC_NUM_AXES; i++) {
            int base = IDC_AX_ENABLE_BASE + i * 10;
            CheckDlgButton(hDlg, base, ds->editCfg.axis[i].enabled ? BST_CHECKED : BST_UNCHECKED);
            SetEditDouble(hDlg, base + 1, ds->editCfg.axis[i].stepsPerUnit);
            SetEditDouble(hDlg, base + 2, ds->editCfg.axis[i].maxVelUnitsPerMin);
            SetEditDouble(hDlg, base + 3, ds->editCfg.axis[i].accelUnitsPerSec2);

            /* Clone combo: "None" is index 0, then other axes skipping self.
             * Map cloneMaster (-1=none, 0..5=axis) to combo index. */
            int cloneMaster = ds->editCfg.axis[i].cloneMaster;
            int comboIdx = 0;  /* default "None" */
            if (cloneMaster >= 0 && cloneMaster < WCNC_NUM_AXES && cloneMaster != i) {
                /* Combo items after "None" are axes 0..5 skipping i */
                comboIdx = (cloneMaster < i) ? (cloneMaster + 1) : cloneMaster;
            }
            SendDlgItemMessageA(hDlg, base + 4, CB_SETCURSEL, comboIdx, 0);

            CheckDlgButton(hDlg, base + 5,
                           ds->editCfg.axis[i].cloneReversed ? BST_CHECKED : BST_UNCHECKED);
        }
        return TRUE;
    }

    case WM_COMMAND:
        if (HIWORD(wParam) == EN_CHANGE || HIWORD(wParam) == BN_CLICKED ||
            HIWORD(wParam) == CBN_SELCHANGE) {
            PropSheet_Changed(GetParent(hDlg), hDlg);
        }
        return TRUE;

    case WM_NOTIFY:
    {
        NMHDR *hdr = (NMHDR *)lParam;
        DialogState *ds = (DialogState *)GetWindowLongPtrA(hDlg, GWLP_USERDATA);
        if (!ds) break;

        if (hdr->code == PSN_APPLY) {
            for (int i = 0; i < WCNC_NUM_AXES; i++) {
                int base = IDC_AX_ENABLE_BASE + i * 10;
                ds->editCfg.axis[i].enabled = (IsDlgButtonChecked(hDlg, base) == BST_CHECKED);
                ds->editCfg.axis[i].stepsPerUnit = GetEditDouble(hDlg, base + 1);
                ds->editCfg.axis[i].maxVelUnitsPerMin = GetEditDouble(hDlg, base + 2);
                ds->editCfg.axis[i].accelUnitsPerSec2 = GetEditDouble(hDlg, base + 3);

                /* Clone combo: index 0 = "None", then other axes skipping self */
                int sel = (int)SendDlgItemMessageA(hDlg, base + 4, CB_GETCURSEL, 0, 0);
                if (sel <= 0) {
                    ds->editCfg.axis[i].cloneMaster = -1;
                } else {
                    /* Map combo index back to axis: items are 0..5 skipping i */
                    int axisIdx = sel - 1;
                    if (axisIdx >= i) axisIdx++;
                    ds->editCfg.axis[i].cloneMaster = axisIdx;
                }

                ds->editCfg.axis[i].cloneReversed =
                    (IsDlgButtonChecked(hDlg, base + 5) == BST_CHECKED);
            }
            ds->changed = true;
            SetWindowLongPtrA(hDlg, DWLP_MSGRESULT, PSNRET_NOERROR);
            return TRUE;
        }
        break;
    }
    }
    return FALSE;
}

/* ===================================================================
 * Tab 3: Input Monitoring
 *
 * Each input has a small LED circle (green=LOW, red=HIGH) next to its name.
 * LEDs are owner-drawn static controls painted via WM_DRAWITEM.
 * Labels show the signal name only — no text state suffix.
 * =================================================================== */

/* Track HIGH/LOW state per input for LED painting.
 * 0..5 = limits, 6..11 = homes, 12 = E-stop, 13 = probe */
static bool s_inputHigh[14] = {false};
static bool s_inputConnected = false;   /* have we received at least one status? */

/* Track output states for LED painting.
 * 0=Spindle CW, 1=Spindle CCW, 2=Flood, 3=Mist,
 * 4=Enable, 5=Charge Pump, 6=Misc Out 1, 7=Misc Out 2 */
#define NUM_OUTPUT_LEDS 8
static bool s_outputHigh[NUM_OUTPUT_LEDS] = {false};
static const char *s_outputNames[NUM_OUTPUT_LEDS] = {
    "Spindle CW", "Spindle CCW", "Flood", "Mist",
    "Connected", "Charge Pump", "Misc Out 1", "Misc Out 2"
};

static void UpdateIOLEDs(HWND hDlg, DialogState *ds)
{
    if (!ds) return;

    /* ALL I/O states read from controller status report — ground truth of
     * what the hardware is actually doing.  No tracking, no guessing. */
    if (ds->network) {
        wcnc_status_report_t status;
        ds->network->GetLatestStatus(&status);
        s_inputConnected = true;

        /* Diagnostic: log raw status values every 5 seconds */
        {
            static DWORD s_lastDiag = 0;
            DWORD now = GetTickCount();
            if (now - s_lastDiag > 5000) {
                s_lastDiag = now;
                Log("Dialog status: sp=%d cool=%d misc=0x%02X "
                    "lim=0x%02X home=0x%02X estop=%d probe=%d",
                    status.spindle_state, status.coolant_state,
                    status.misc_outputs, status.limit_switches,
                    status.home_switches, status.estop_input,
                    status.probe_state);
            }
        }

        /* Output states from controller */
        s_outputHigh[0] = (status.spindle_state == 1);           /* Spindle CW */
        s_outputHigh[1] = (status.spindle_state == 2);           /* Spindle CCW */
        s_outputHigh[2] = (status.coolant_state & 0x01) != 0;   /* Flood */
        s_outputHigh[3] = (status.coolant_state & 0x02) != 0;   /* Mist */
        s_outputHigh[6] = (status.misc_outputs & 0x01) != 0;    /* Misc Out 1 */
        s_outputHigh[7] = (status.misc_outputs & 0x02) != 0;    /* Misc Out 2 */

        /* Input states from controller */
        for (int i = 0; i < 6; i++) {
            s_inputHigh[i]     = ((status.limit_switches >> i) & 1) != 0;
            s_inputHigh[6 + i] = ((status.home_switches >> i) & 1) != 0;
        }
        /* E-stop: hardware pin OR software E-stop */
        s_inputHigh[12] = (status.estop_input != 0) ||
                          (Engine && Engine->Emergency);
        s_inputHigh[13] = (status.probe_state != 0);
    }
    /* E-stop still works without controller connection (software-only) */
    else if (Engine) {
        s_inputHigh[12] = Engine->Emergency ? true : false;
    }

    /* Enable: connected to controller.  ChargePump: Mach3-side signal. */
    s_outputHigh[4] = (ds->network && ds->network->IsConnected());
    if (Engine) {
        s_outputHigh[5] = (Engine->OutSigs[CHARGE].active &&
                           Engine->OutSigs[CHARGE].Activated);
    }

    /* Invalidate LED controls so they repaint */
    for (int i = 0; i < 14; i++) {
        HWND hLed = GetDlgItem(hDlg, IDC_INPUT_LED_BASE + i);
        if (hLed) InvalidateRect(hLed, nullptr, TRUE);
    }
    for (int i = 0; i < NUM_OUTPUT_LEDS; i++) {
        HWND hLed = GetDlgItem(hDlg, IDC_OUTPUT_LED_BASE + i);
        if (hLed) InvalidateRect(hLed, nullptr, TRUE);
    }
}

static INT_PTR CALLBACK InputsProc(HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam)
{
    switch (msg) {
    case WM_INITDIALOG:
    {
        if (lParam) {
            PROPSHEETPAGE *psp = (PROPSHEETPAGE *)lParam;
            SetWindowLongPtrA(hDlg, GWLP_USERDATA, (LONG_PTR)psp->lParam);
        }
        DialogState *ds = (DialogState *)GetWindowLongPtrA(hDlg, GWLP_USERDATA);
        if (!ds) return TRUE;

        /* Init inversion checkboxes — now controlled via Mach3 Ports & Pins.
         * The checkboxes are hidden; a label directs users to the Mach3 dialog.
         * Define LEGACY_INVERSION to restore the old checkboxes. */
#ifdef LEGACY_INVERSION
        for (int i = 0; i < 6; i++) {
            CheckDlgButton(hDlg, IDC_INV_LIMIT_BASE + i,
                (ds->editCfg.invertLimit >> i) & 1 ? BST_CHECKED : BST_UNCHECKED);
            CheckDlgButton(hDlg, IDC_INV_HOME_BASE + i,
                (ds->editCfg.invertHome >> i) & 1 ? BST_CHECKED : BST_UNCHECKED);
        }
        CheckDlgButton(hDlg, IDC_INV_ESTOP,
            ds->editCfg.invertEstop ? BST_CHECKED : BST_UNCHECKED);
        CheckDlgButton(hDlg, IDC_INV_PROBE,
            ds->editCfg.invertProbe ? BST_CHECKED : BST_UNCHECKED);
#else
        /* Hide inversion checkboxes — controlled via Mach3 Ports & Pins */
        for (int i = 0; i < 6; i++) {
            HWND h = GetDlgItem(hDlg, IDC_INV_LIMIT_BASE + i);
            if (h) ShowWindow(h, SW_HIDE);
            h = GetDlgItem(hDlg, IDC_INV_HOME_BASE + i);
            if (h) ShowWindow(h, SW_HIDE);
        }
        { HWND h = GetDlgItem(hDlg, IDC_INV_ESTOP); if (h) ShowWindow(h, SW_HIDE); }
        { HWND h = GetDlgItem(hDlg, IDC_INV_PROBE); if (h) ShowWindow(h, SW_HIDE); }
#endif

        /* Init misc input function combos */
        for (int i = 0; i < 4; i++) {
            HWND hCombo = GetDlgItem(hDlg, IDC_MISC_FUNC_BASE + i);
            if (!hCombo) continue;
            int selIdx = 0;
            for (int f = 0; f < g_numInputFuncs; f++) {
                SendMessageA(hCombo, CB_ADDSTRING, 0, (LPARAM)g_inputFuncs[f].name);
                if (g_inputFuncs[f].code == ds->editCfg.miscInputFunction[i])
                    selIdx = f;
            }
            SendMessageA(hCombo, CB_SETCURSEL, selIdx, 0);
        }

        /* Start live update timer (200ms) */
        SetTimer(hDlg, IDT_INPUT_POLL, 200, nullptr);

        /* Initial update */
        s_inputConnected = false;
        UpdateIOLEDs(hDlg, ds);
        return TRUE;
    }

    case WM_TIMER:
        if (wParam == IDT_INPUT_POLL) {
            DialogState *ds = (DialogState *)GetWindowLongPtrA(hDlg, GWLP_USERDATA);
            UpdateIOLEDs(hDlg, ds);
        }
        return TRUE;

    case WM_DESTROY:
        KillTimer(hDlg, IDT_INPUT_POLL);
        return TRUE;

    /* Paint LED indicators as filled circles: green=OFF, red=ON, grey=no data */
    case WM_DRAWITEM:
    {
        DRAWITEMSTRUCT *dis = (DRAWITEMSTRUCT *)lParam;
        int id = (int)dis->CtlID;
        COLORREF color = RGB(160, 160, 160);  /* default grey */
        bool handled = false;

        if (id >= IDC_INPUT_LED_BASE && id < IDC_INPUT_LED_BASE + 14) {
            int idx = id - IDC_INPUT_LED_BASE;
            if (!s_inputConnected)
                color = RGB(160, 160, 160);
            else if (s_inputHigh[idx])
                color = RGB(220, 30, 30);
            else
                color = RGB(20, 180, 20);
            handled = true;
        }
        else if (id >= IDC_OUTPUT_LED_BASE && id < IDC_OUTPUT_LED_BASE + NUM_OUTPUT_LEDS) {
            int idx = id - IDC_OUTPUT_LED_BASE;
            /* Output LEDs always show color — they read from Engine */
            if (s_outputHigh[idx])
                color = RGB(220, 30, 30);
            else
                color = RGB(20, 180, 20);
            handled = true;
        }

        if (handled) {
            HBRUSH hBr = CreateSolidBrush(color);
            HPEN hPen = CreatePen(PS_SOLID, 1, RGB(80, 80, 80));
            HBRUSH oldBr = (HBRUSH)SelectObject(dis->hDC, hBr);
            HPEN oldPen = (HPEN)SelectObject(dis->hDC, hPen);
            Ellipse(dis->hDC, dis->rcItem.left, dis->rcItem.top,
                    dis->rcItem.right, dis->rcItem.bottom);
            SelectObject(dis->hDC, oldBr);
            SelectObject(dis->hDC, oldPen);
            DeleteObject(hBr);
            DeleteObject(hPen);
            return TRUE;
        }
        break;
    }

    case WM_COMMAND:
        if (HIWORD(wParam) == BN_CLICKED || HIWORD(wParam) == CBN_SELCHANGE) {
            PropSheet_Changed(GetParent(hDlg), hDlg);
        }
        return TRUE;

    case WM_NOTIFY:
    {
        NMHDR *hdr = (NMHDR *)lParam;
        DialogState *ds = (DialogState *)GetWindowLongPtrA(hDlg, GWLP_USERDATA);
        if (!ds) break;

        if (hdr->code == PSN_APPLY) {
#ifdef LEGACY_INVERSION
            ds->editCfg.invertLimit = 0;
            ds->editCfg.invertHome = 0;
            for (int i = 0; i < 6; i++) {
                if (IsDlgButtonChecked(hDlg, IDC_INV_LIMIT_BASE + i) == BST_CHECKED)
                    ds->editCfg.invertLimit |= (1 << i);
                if (IsDlgButtonChecked(hDlg, IDC_INV_HOME_BASE + i) == BST_CHECKED)
                    ds->editCfg.invertHome |= (1 << i);
            }
            ds->editCfg.invertEstop =
                (IsDlgButtonChecked(hDlg, IDC_INV_ESTOP) == BST_CHECKED) ? 1 : 0;
            ds->editCfg.invertProbe =
                (IsDlgButtonChecked(hDlg, IDC_INV_PROBE) == BST_CHECKED) ? 1 : 0;
#endif
            /* (When !LEGACY_INVERSION, inversion is read from Mach3 Engine
             * in ReadInversionFromMach3() during config sync) */

            /* Misc input function mapping */
            for (int i = 0; i < 4; i++) {
                HWND hCombo = GetDlgItem(hDlg, IDC_MISC_FUNC_BASE + i);
                if (!hCombo) continue;
                int sel = (int)SendMessageA(hCombo, CB_GETCURSEL, 0, 0);
                if (sel >= 0 && sel < g_numInputFuncs)
                    ds->editCfg.miscInputFunction[i] = g_inputFuncs[sel].code;
                else
                    ds->editCfg.miscInputFunction[i] = 0;
            }

            ds->changed = true;
            SetWindowLongPtrA(hDlg, DWLP_MSGRESULT, PSNRET_NOERROR);
            return TRUE;
        }
        break;
    }
    }
    return FALSE;
}

/* ===================================================================
 * Tab 4: Advanced Settings
 * =================================================================== */

static INT_PTR CALLBACK AdvancedProc(HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam)
{
    switch (msg) {
    case WM_INITDIALOG:
    {
        if (lParam) {
            PROPSHEETPAGE *psp = (PROPSHEETPAGE *)lParam;
            SetWindowLongPtrA(hDlg, GWLP_USERDATA, (LONG_PTR)psp->lParam);
        }
        DialogState *ds = (DialogState *)GetWindowLongPtrA(hDlg, GWLP_USERDATA);
        if (!ds) return TRUE;

        /* Timing */
        SetEditInt(hDlg, IDC_STEP_PULSE, ds->editCfg.stepPulseUs);
        SetEditInt(hDlg, IDC_DIR_SETUP, ds->editCfg.dirSetupUs);

        /* Step/Dir inversion — now read from Mach3 Ports & Pins Motor Outputs */
#ifdef LEGACY_INVERSION
        for (int i = 0; i < 6; i++) {
            CheckDlgButton(hDlg, IDC_INV_STEP_BASE + i,
                (ds->editCfg.invertStep >> i) & 1 ? BST_CHECKED : BST_UNCHECKED);
            CheckDlgButton(hDlg, IDC_INV_DIR_BASE + i,
                (ds->editCfg.invertDir >> i) & 1 ? BST_CHECKED : BST_UNCHECKED);
#else
        for (int i = 0; i < 6; i++) {
            HWND h = GetDlgItem(hDlg, IDC_INV_STEP_BASE + i);
            if (h) ShowWindow(h, SW_HIDE);
            h = GetDlgItem(hDlg, IDC_INV_DIR_BASE + i);
            if (h) ShowWindow(h, SW_HIDE);
#endif
        }

        /* Homing */
        for (int i = 0; i < 6; i++) {
            CheckDlgButton(hDlg, IDC_HOME_DIR_BASE + i,
                (ds->editCfg.homingDirMask >> i) & 1 ? BST_CHECKED : BST_UNCHECKED);
        }
        SetEditInt(hDlg, IDC_HOME_SEEK, ds->editCfg.homingSeekRate);
        SetEditInt(hDlg, IDC_HOME_FEED, ds->editCfg.homingFeedRate);
        SetEditInt(hDlg, IDC_HOME_PULLOFF, ds->editCfg.homingPulloff);

        /* Spindle */
        SetEditInt(hDlg, IDC_SPINDLE_FREQ, ds->editCfg.spindlePwmFreq);
        SetEditInt(hDlg, IDC_SPINDLE_RPM, ds->editCfg.spindleMaxRpm);

        /* Charge pump / step idle / laser */
        SetEditInt(hDlg, IDC_CHARGE_FREQ, ds->editCfg.chargePumpFreq);
        SetEditInt(hDlg, IDC_IDLE_DELAY, ds->editCfg.stepIdleDelayMs);
        CheckDlgButton(hDlg, IDC_LASER_MODE,
            ds->editCfg.laserMode ? BST_CHECKED : BST_UNCHECKED);
        return TRUE;
    }

    case WM_COMMAND:
        if (HIWORD(wParam) == EN_CHANGE || HIWORD(wParam) == BN_CLICKED) {
            PropSheet_Changed(GetParent(hDlg), hDlg);
        }
        return TRUE;

    case WM_NOTIFY:
    {
        NMHDR *hdr = (NMHDR *)lParam;
        DialogState *ds = (DialogState *)GetWindowLongPtrA(hDlg, GWLP_USERDATA);
        if (!ds) break;

        if (hdr->code == PSN_APPLY) {
            /* Timing */
            ds->editCfg.stepPulseUs = (uint16_t)GetEditInt(hDlg, IDC_STEP_PULSE);
            ds->editCfg.dirSetupUs  = (uint16_t)GetEditInt(hDlg, IDC_DIR_SETUP);

            /* Step/Dir inversion */
#ifdef LEGACY_INVERSION
            ds->editCfg.invertStep = 0;
            ds->editCfg.invertDir  = 0;
            for (int i = 0; i < 6; i++) {
                if (IsDlgButtonChecked(hDlg, IDC_INV_STEP_BASE + i) == BST_CHECKED)
                    ds->editCfg.invertStep |= (1 << i);
                if (IsDlgButtonChecked(hDlg, IDC_INV_DIR_BASE + i) == BST_CHECKED)
                    ds->editCfg.invertDir |= (1 << i);
#else
            /* Inversion read from Mach3 Ports & Pins in ReadInversionFromMach3() */
            for (int i = 0; i < 6; i++) { (void)i; }
#endif

            /* Homing */
            ds->editCfg.homingDirMask = 0;
            for (int i = 0; i < 6; i++) {
                if (IsDlgButtonChecked(hDlg, IDC_HOME_DIR_BASE + i) == BST_CHECKED)
                    ds->editCfg.homingDirMask |= (1 << i);
            }
            ds->editCfg.homingSeekRate = GetEditInt(hDlg, IDC_HOME_SEEK);
            ds->editCfg.homingFeedRate = GetEditInt(hDlg, IDC_HOME_FEED);
            ds->editCfg.homingPulloff  = GetEditInt(hDlg, IDC_HOME_PULLOFF);

            /* Spindle */
            ds->editCfg.spindlePwmFreq = (uint16_t)GetEditInt(hDlg, IDC_SPINDLE_FREQ);
            ds->editCfg.spindleMaxRpm  = GetEditInt(hDlg, IDC_SPINDLE_RPM);

            /* Charge pump / step idle / laser */
            ds->editCfg.chargePumpFreq  = (uint16_t)GetEditInt(hDlg, IDC_CHARGE_FREQ);
            ds->editCfg.stepIdleDelayMs = (uint16_t)GetEditInt(hDlg, IDC_IDLE_DELAY);
            ds->editCfg.laserMode = (IsDlgButtonChecked(hDlg, IDC_LASER_MODE) == BST_CHECKED);

            ds->changed = true;
            SetWindowLongPtrA(hDlg, DWLP_MSGRESULT, PSNRET_NOERROR);
            return TRUE;
        }
        break;
    }
    }
    return FALSE;
}

/* ===================================================================
 * Page Template Builders
 *
 * Each function creates the controls for one PropertySheet page.
 * We build DLGTEMPLATE + items in a memory buffer at runtime.
 * =================================================================== */

static const short PAGE_W = 340;
static const short PAGE_H = 300;

static LPDLGTEMPLATE BuildConnectionPage()
{
    /* We'll create controls manually in WM_INITDIALOG instead of in the template.
     * This is simpler than building a complete DLGTEMPLATE with items. */
    return CreatePageTemplate(0, PAGE_W, PAGE_H);
}

static LPDLGTEMPLATE BuildAxisPage()
{
    return CreatePageTemplate(0, PAGE_W, PAGE_H);
}

static LPDLGTEMPLATE BuildInputsPage()
{
    return CreatePageTemplate(0, PAGE_W, PAGE_H);
}

static LPDLGTEMPLATE BuildAdvancedPage()
{
    return CreatePageTemplate(0, PAGE_W, PAGE_H);
}

static LPDLGTEMPLATE BuildIOModulePage()
{
    return CreatePageTemplate(0, PAGE_W, PAGE_H);
}

/* ===================================================================
 * Control Creation Helpers (called from WM_INITDIALOG)
 *
 * Since building DLGITEMTEMPLATE at runtime is error-prone,
 * we create controls with CreateWindowExA after the dialog exists.
 * =================================================================== */

static HWND MakeLabel(HWND parent, const char *text, int x, int y, int w, int h, int id)
{
    return CreateWindowExA(0, "STATIC", text, WS_CHILD | WS_VISIBLE | SS_LEFT,
                           x, y, w, h, parent, (HMENU)(INT_PTR)id, nullptr, nullptr);
}

static HWND MakeEdit(HWND parent, const char *text, int x, int y, int w, int h, int id)
{
    return CreateWindowExA(WS_EX_CLIENTEDGE, "EDIT", text,
                           WS_CHILD | WS_VISIBLE | WS_TABSTOP | ES_AUTOHSCROLL,
                           x, y, w, h, parent, (HMENU)(INT_PTR)id, nullptr, nullptr);
}

static HWND MakeCheck(HWND parent, const char *text, int x, int y, int w, int h, int id)
{
    return CreateWindowExA(0, "BUTTON", text,
                           WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_AUTOCHECKBOX,
                           x, y, w, h, parent, (HMENU)(INT_PTR)id, nullptr, nullptr);
}

static HWND MakeButton(HWND parent, const char *text, int x, int y, int w, int h, int id)
{
    return CreateWindowExA(0, "BUTTON", text,
                           WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_PUSHBUTTON,
                           x, y, w, h, parent, (HMENU)(INT_PTR)id, nullptr, nullptr);
}

static HWND MakeGroupBox(HWND parent, const char *text, int x, int y, int w, int h, int id)
{
    return CreateWindowExA(0, "BUTTON", text,
                           WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                           x, y, w, h, parent, (HMENU)(INT_PTR)id, nullptr, nullptr);
}

static HWND MakeCombo(HWND parent, int x, int y, int w, int h, int id)
{
    return CreateWindowExA(0, "COMBOBOX", "",
                           WS_CHILD | WS_VISIBLE | WS_TABSTOP |
                           CBS_DROPDOWNLIST | CBS_HASSTRINGS,
                           x, y, w, h, parent, (HMENU)(INT_PTR)id, nullptr, nullptr);
}

/* Set a consistent font on all child controls */
static void SetDialogFont(HWND hDlg)
{
    HFONT hFont = (HFONT)SendMessageA(hDlg, WM_GETFONT, 0, 0);
    if (!hFont) hFont = (HFONT)GetStockObject(DEFAULT_GUI_FONT);

    HWND child = GetWindow(hDlg, GW_CHILD);
    while (child) {
        SendMessageA(child, WM_SETFONT, (WPARAM)hFont, TRUE);
        child = GetWindow(child, GW_HWNDNEXT);
    }
}

/* ===================================================================
 * Wrapped Page Procs that create controls then delegate
 * =================================================================== */

static const char *AXIS_LABELS[] = { "X", "Y", "Z", "A", "B", "C" };

static INT_PTR CALLBACK ConnectionPageProc(HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam)
{
    if (msg == WM_INITDIALOG) {
        /* Create controls */
        MakeGroupBox(hDlg, "Connection", 5, 2, 460, 95, -1);
        MakeLabel(hDlg, "Controller IP Address:", 15, 20, 120, 14, -1);
        MakeEdit(hDlg, "", 140, 18, 120, 18, IDC_IP_EDIT);
        MakeButton(hDlg, "Connect", 270, 17, 60, 20, IDC_CONNECT_BTN);
        MakeButton(hDlg, "Disconnect", 335, 17, 65, 20, IDC_DISCONNECT_BTN);
        MakeButton(hDlg, "Auto-Discover", 270, 42, 130, 20, IDC_DISCOVER_BTN);
        MakeLabel(hDlg, "Status: ---", 15, 48, 240, 14, IDC_STATUS_LABEL);
        MakeLabel(hDlg, "Click 'Connect' or 'Auto-Discover' to establish connection.\n"
                        "Settings are applied and synced on OK/Apply.",
                  15, 72, 440, 22, -1);

        /* Spindle Encoder (live RPM from motion controller) */
        MakeGroupBox(hDlg, "Spindle Encoder", 5, 100, 460, 36, -1);
        MakeLabel(hDlg, "Spindle RPM: ---", 15, 116, 200, 14, IDC_CONN_RPM_LABEL);

        SetDialogFont(hDlg);
    }
    return ConnectionProc(hDlg, msg, wParam, lParam);
}

static INT_PTR CALLBACK AxisConfigPageProc(HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam)
{
    if (msg == WM_INITDIALOG) {
        MakeGroupBox(hDlg, "Axis Configuration (these override Mach3 Motor Tuning)", 5, 2, 460, 195, -1);

        /* Column headers */
        MakeLabel(hDlg, "Axis", 15, 18, 30, 14, -1);
        MakeLabel(hDlg, "On", 48, 18, 18, 14, -1);
        MakeLabel(hDlg, "Steps/Unit", 80, 18, 65, 14, -1);
        MakeLabel(hDlg, "Vel (u/min)", 165, 18, 65, 14, -1);
        MakeLabel(hDlg, "Accel (u/s" "\xB2" ")", 245, 18, 65, 14, -1);
        MakeLabel(hDlg, "Clone of", 330, 18, 50, 14, -1);
        MakeLabel(hDlg, "Rev", 410, 18, 22, 14, -1);

        /* Per-axis rows */
        for (int i = 0; i < WCNC_NUM_AXES; i++) {
            int y = 35 + i * 25;
            int base = IDC_AX_ENABLE_BASE + i * 10;

            MakeLabel(hDlg, AXIS_LABELS[i], 20, y + 2, 20, 14, -1);
            MakeCheck(hDlg, "", 50, y, 18, 18, base);        /* Enable */
            MakeEdit(hDlg, "", 80, y, 70, 18, base + 1);     /* Steps/Unit */
            MakeEdit(hDlg, "", 165, y, 70, 18, base + 2);    /* Velocity */
            MakeEdit(hDlg, "", 245, y, 70, 18, base + 3);    /* Accel */

            /* Clone-of combo */
            HWND hCombo = MakeCombo(hDlg, 330, y - 1, 70, 180, base + 4);
            SendMessageA(hCombo, CB_ADDSTRING, 0, (LPARAM)"None");
            for (int j = 0; j < WCNC_NUM_AXES; j++) {
                if (j != i)
                    SendMessageA(hCombo, CB_ADDSTRING, 0, (LPARAM)AXIS_LABELS[j]);
            }

            /* Reverse checkbox */
            MakeCheck(hDlg, "", 415, y, 18, 18, base + 5);
        }

        MakeLabel(hDlg, "Rev = reverse clone direction (slave moves opposite to master)",
                  15, 192, 440, 14, -1);

        SetDialogFont(hDlg);
    }
    return AxisConfigProc(hDlg, msg, wParam, lParam);
}

/* Create a small owner-drawn static for use as an LED indicator */
static HWND MakeLED(HWND parent, int x, int y, int size, int id)
{
    return CreateWindowExA(0, "STATIC", "",
                           WS_CHILD | WS_VISIBLE | SS_OWNERDRAW,
                           x, y, size, size, parent,
                           (HMENU)(INT_PTR)id, nullptr, nullptr);
}

static INT_PTR CALLBACK InputsPageProc(HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam)
{
    if (msg == WM_INITDIALOG) {
        MakeGroupBox(hDlg, "Input Status (live)", 5, 2, 290, 195, -1);

        /* Limit switches column:  (LED) X Limit  [Inv] */
        for (int i = 0; i < 6; i++) {
            int y = 20 + i * 18;
            char label[16];
            snprintf(label, sizeof(label), "%s Limit", AXIS_LABELS[i]);
            MakeLED(hDlg, 15, y + 2, 10, IDC_INPUT_LED_BASE + i);
            MakeLabel(hDlg, label, 28, y, 70, 14, IDC_INPUT_STATUS_BASE + i);
#ifdef LEGACY_INVERSION
            MakeCheck(hDlg, "Inv", 105, y - 2, 38, 16, IDC_INV_LIMIT_BASE + i);
#endif
        }

        /* Home switches column:  (LED) X Home  [Inv] */
        for (int i = 0; i < 6; i++) {
            int y = 20 + i * 18;
            char label[16];
            snprintf(label, sizeof(label), "%s Home", AXIS_LABELS[i]);
            MakeLED(hDlg, 155, y + 2, 10, IDC_INPUT_LED_BASE + 6 + i);
            MakeLabel(hDlg, label, 168, y, 70, 14, IDC_INPUT_STATUS_BASE + 6 + i);
#ifdef LEGACY_INVERSION
            MakeCheck(hDlg, "Inv", 238, y - 2, 38, 16, IDC_INV_HOME_BASE + i);
#endif
        }

        /* E-Stop and Probe */
        MakeLED(hDlg, 15, 134, 10, IDC_INPUT_LED_BASE + 12);
        MakeLabel(hDlg, "E-Stop", 28, 132, 55, 14, IDC_INPUT_STATUS_BASE + 12);
#ifdef LEGACY_INVERSION
        MakeCheck(hDlg, "Inv", 105, 130, 38, 16, IDC_INV_ESTOP);
#endif

        MakeLED(hDlg, 15, 152, 10, IDC_INPUT_LED_BASE + 13);
        MakeLabel(hDlg, "Probe", 28, 150, 55, 14, IDC_INPUT_STATUS_BASE + 13);
#ifdef LEGACY_INVERSION
        MakeCheck(hDlg, "Inv", 105, 148, 38, 16, IDC_INV_PROBE);
#endif

        /* Output status panel */
        MakeGroupBox(hDlg, "Output Status", 300, 2, 165, 195, -1);
        for (int i = 0; i < NUM_OUTPUT_LEDS; i++) {
            int y = 20 + i * 20;
            MakeLED(hDlg, 310, y + 2, 10, IDC_OUTPUT_LED_BASE + i);
            MakeLabel(hDlg, s_outputNames[i], 323, y, 130, 14, -1);
        }

        MakeLabel(hDlg, "LED: green=OFF  red=ON  grey=no data",
                  15, 175, 250, 14, -1);

        /* Misc input function mapping */
        MakeGroupBox(hDlg, "Misc Input Functions (button press -> action)", 5, 200, 460, 90, -1);
        for (int i = 0; i < 4; i++) {
            char label[32];
            snprintf(label, sizeof(label), "Input %d:", i + 1);
            MakeLabel(hDlg, label, 15 + (i % 2) * 230, 218 + (i / 2) * 28, 48, 14, -1);
            MakeCombo(hDlg, 65 + (i % 2) * 230, 215 + (i / 2) * 28, 155, 200,
                      IDC_MISC_FUNC_BASE + i);
        }

        SetDialogFont(hDlg);
    }
    return InputsProc(hDlg, msg, wParam, lParam);
}

static INT_PTR CALLBACK AdvancedPageProc(HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam)
{
    if (msg == WM_INITDIALOG) {
        int y;

        /* Timing */
        MakeGroupBox(hDlg, "Timing", 5, 2, 460, 42, -1);
        MakeLabel(hDlg, "Step Pulse:", 15, 20, 65, 14, -1);
        MakeEdit(hDlg, "", 80, 18, 40, 18, IDC_STEP_PULSE);
        MakeLabel(hDlg, "us", 122, 20, 16, 14, -1);
        MakeLabel(hDlg, "Dir Setup:", 160, 20, 60, 14, -1);
        MakeEdit(hDlg, "", 220, 18, 40, 18, IDC_DIR_SETUP);
        MakeLabel(hDlg, "us", 262, 20, 16, 14, -1);

        /* Step/Dir inversion */
#ifdef LEGACY_INVERSION
        MakeGroupBox(hDlg, "Step/Dir Inversion", 5, 48, 460, 55, -1);
        MakeLabel(hDlg, "Invert Step:", 15, 64, 65, 14, -1);
        for (int i = 0; i < 6; i++)
            MakeCheck(hDlg, AXIS_LABELS[i], 85 + i * 40, 62, 35, 16, IDC_INV_STEP_BASE + i);
        MakeLabel(hDlg, "Invert Dir:", 15, 82, 65, 14, -1);
        for (int i = 0; i < 6; i++)
            MakeCheck(hDlg, AXIS_LABELS[i], 85 + i * 40, 80, 35, 16, IDC_INV_DIR_BASE + i);
#else
        MakeGroupBox(hDlg, "Step/Dir Inversion (use Mach3 Config > Ports and Pins)", 5, 48, 460, 55, -1);
        MakeLabel(hDlg, "Set in Mach3: Config > Ports and Pins > Motor Outputs", 15, 68, 400, 14, -1);
#endif

        /* Homing */
        MakeGroupBox(hDlg, "Homing", 5, 108, 460, 70, -1);
        MakeLabel(hDlg, "Direction (+):", 15, 124, 70, 14, -1);
        for (int i = 0; i < 6; i++)
            MakeCheck(hDlg, AXIS_LABELS[i], 85 + i * 40, 122, 35, 16, IDC_HOME_DIR_BASE + i);
        y = 144;
        MakeLabel(hDlg, "Seek:", 15, y, 30, 14, -1);
        MakeEdit(hDlg, "", 48, y - 2, 55, 18, IDC_HOME_SEEK);
        MakeLabel(hDlg, "s/s", 105, y, 22, 14, -1);
        MakeLabel(hDlg, "Feed:", 140, y, 30, 14, -1);
        MakeEdit(hDlg, "", 172, y - 2, 55, 18, IDC_HOME_FEED);
        MakeLabel(hDlg, "s/s", 229, y, 22, 14, -1);
        MakeLabel(hDlg, "Pulloff:", 265, y, 40, 14, -1);
        MakeEdit(hDlg, "", 308, y - 2, 55, 18, IDC_HOME_PULLOFF);
        MakeLabel(hDlg, "steps", 365, y, 30, 14, -1);

        /* Spindle */
        MakeGroupBox(hDlg, "Spindle", 5, 182, 460, 42, -1);
        MakeLabel(hDlg, "PWM Freq:", 15, 200, 55, 14, -1);
        MakeEdit(hDlg, "", 72, 198, 55, 18, IDC_SPINDLE_FREQ);
        MakeLabel(hDlg, "Hz", 129, 200, 16, 14, -1);
        MakeLabel(hDlg, "Max RPM:", 170, 200, 50, 14, -1);
        MakeEdit(hDlg, "", 222, 198, 65, 18, IDC_SPINDLE_RPM);

        /* Charge Pump / Step Idle / Laser */
        MakeGroupBox(hDlg, "Charge Pump / Step Idle / Laser", 5, 228, 460, 62, -1);
        MakeLabel(hDlg, "Charge Pump:", 15, 246, 72, 14, -1);
        MakeEdit(hDlg, "", 90, 244, 55, 18, IDC_CHARGE_FREQ);
        MakeLabel(hDlg, "Hz  (0=off)", 147, 246, 60, 14, -1);
        MakeLabel(hDlg, "Step Idle:", 230, 246, 55, 14, -1);
        MakeEdit(hDlg, "", 288, 244, 55, 18, IDC_IDLE_DELAY);
        MakeLabel(hDlg, "ms  (0=never)", 345, 246, 75, 14, -1);
        MakeCheck(hDlg, "Laser Mode (spindle PWM tracks feed rate)", 15, 268, 250, 16, IDC_LASER_MODE);

        SetDialogFont(hDlg);
    }
    return AdvancedProc(hDlg, msg, wParam, lParam);
}

/* Forward declaration — IOModuleProc is defined after the wrapper */
static INT_PTR CALLBACK IOModuleProc(HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam);

static INT_PTR CALLBACK IOModulePageProc(HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam)
{
    if (msg == WM_INITDIALOG) {
        int y;

        /* I/O Module Connection */
        MakeGroupBox(hDlg, "I/O Expansion Module", 5, 2, 460, 38, -1);
        MakeCheck(hDlg, "Enable I/O Module", 15, 18, 120, 16, IDC_IO_ENABLE);
        MakeLabel(hDlg, "IP:", 150, 20, 16, 14, -1);
        MakeEdit(hDlg, "", 168, 18, 110, 18, IDC_IO_IP_EDIT);
        MakeLabel(hDlg, "Status: ---", 290, 20, 170, 14, IDC_IO_STATUS_LABEL);

        /* I/O Input Function Mapping (left column, 8 rows) */
        MakeGroupBox(hDlg, "Input Functions", 5, 44, 230, 200, -1);
        for (int i = 0; i < 8; i++) {
            y = 60 + i * 22;
            char label[16];
            snprintf(label, sizeof(label), "In %d:", i);
            MakeLabel(hDlg, label, 15, y + 2, 32, 14, -1);
            MakeCombo(hDlg, 49, y, 175, 200, IDC_IO_FUNC_BASE + i);
        }

        /* I/O Output Function Mapping (right column, 8 rows) */
        MakeGroupBox(hDlg, "Output Functions", 240, 44, 225, 200, -1);
        for (int i = 0; i < 8; i++) {
            y = 60 + i * 22;
            char label[16];
            snprintf(label, sizeof(label), "Out %d:", i);
            MakeLabel(hDlg, label, 250, y + 2, 38, 14, -1);
            MakeCombo(hDlg, 290, y, 165, 200, IDC_IO_OUT_BASE + i);
        }

        SetDialogFont(hDlg);
    }
    return IOModuleProc(hDlg, msg, wParam, lParam);
}

/* ===================================================================
 * Tab 5: I/O Expansion Module
 * =================================================================== */

static INT_PTR CALLBACK IOModuleProc(HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam)
{
    switch (msg) {
    case WM_INITDIALOG:
    {
        if (lParam) {
            PROPSHEETPAGE *psp = (PROPSHEETPAGE *)lParam;
            SetWindowLongPtrA(hDlg, GWLP_USERDATA, (LONG_PTR)psp->lParam);
        }
        DialogState *ds = (DialogState *)GetWindowLongPtrA(hDlg, GWLP_USERDATA);
        if (!ds) return TRUE;

        /* I/O Module enable */
        CheckDlgButton(hDlg, IDC_IO_ENABLE,
            ds->editCfg.ioModuleEnabled ? BST_CHECKED : BST_UNCHECKED);

        /* I/O Module IP */
        SetDlgItemTextA(hDlg, IDC_IO_IP_EDIT, ds->editCfg.ioModuleAddress);

        /* Connection status */
        SetDlgItemTextA(hDlg, IDC_IO_STATUS_LABEL,
            IsIOModuleConnected() ? "Status: Connected" : "Status: Not connected");

        /* I/O input function combos */
        for (int i = 0; i < 8; i++) {
            HWND hCombo = GetDlgItem(hDlg, IDC_IO_FUNC_BASE + i);
            if (!hCombo) continue;
            int sel = 0;
            for (int j = 0; j < g_numIOInputFuncs; j++) {
                SendMessageA(hCombo, CB_ADDSTRING, 0, (LPARAM)g_ioInputFuncs[j].name);
                if (ds->editCfg.ioInputFunction[i] == g_ioInputFuncs[j].code)
                    sel = j;
            }
            SendMessageA(hCombo, CB_SETCURSEL, sel, 0);
        }

        /* I/O output function combos */
        for (int i = 0; i < 8; i++) {
            HWND hCombo = GetDlgItem(hDlg, IDC_IO_OUT_BASE + i);
            if (!hCombo) continue;
            int sel = 0;
            for (int j = 0; j < g_numIOOutputFuncs; j++) {
                SendMessageA(hCombo, CB_ADDSTRING, 0, (LPARAM)g_ioOutputFuncs[j].name);
                if (ds->editCfg.ioOutputFunction[i] == g_ioOutputFuncs[j].code)
                    sel = j;
            }
            SendMessageA(hCombo, CB_SETCURSEL, sel, 0);
        }

        /* Start live update timer (I/O module connection status) */
        SetTimer(hDlg, IDT_INPUT_POLL + 1, 500, nullptr);
        return TRUE;
    }

    case WM_TIMER:
        if (wParam == IDT_INPUT_POLL + 1) {
            SetDlgItemTextA(hDlg, IDC_IO_STATUS_LABEL,
                IsIOModuleConnected() ? "Status: Connected" : "Status: Not connected");
        }
        return TRUE;

    case WM_COMMAND:
        if (HIWORD(wParam) == EN_CHANGE || HIWORD(wParam) == BN_CLICKED ||
            HIWORD(wParam) == CBN_SELCHANGE) {
            PropSheet_Changed(GetParent(hDlg), hDlg);
        }
        return TRUE;

    case WM_NOTIFY:
    {
        NMHDR *hdr = (NMHDR *)lParam;
        DialogState *ds = (DialogState *)GetWindowLongPtrA(hDlg, GWLP_USERDATA);
        if (!ds) break;

        if (hdr->code == PSN_APPLY) {
            /* I/O Module enable */
            ds->editCfg.ioModuleEnabled =
                (IsDlgButtonChecked(hDlg, IDC_IO_ENABLE) == BST_CHECKED);

            /* I/O Module IP */
            GetDlgItemTextA(hDlg, IDC_IO_IP_EDIT,
                ds->editCfg.ioModuleAddress, sizeof(ds->editCfg.ioModuleAddress));

            /* I/O input function mapping */
            for (int i = 0; i < 8; i++) {
                HWND hCombo = GetDlgItem(hDlg, IDC_IO_FUNC_BASE + i);
                if (!hCombo) continue;
                int sel = (int)SendMessageA(hCombo, CB_GETCURSEL, 0, 0);
                if (sel >= 0 && sel < g_numIOInputFuncs)
                    ds->editCfg.ioInputFunction[i] = g_ioInputFuncs[sel].code;
            }

            /* I/O output function mapping */
            for (int i = 0; i < 8; i++) {
                HWND hCombo = GetDlgItem(hDlg, IDC_IO_OUT_BASE + i);
                if (!hCombo) continue;
                int sel = (int)SendMessageA(hCombo, CB_GETCURSEL, 0, 0);
                if (sel >= 0 && sel < g_numIOOutputFuncs)
                    ds->editCfg.ioOutputFunction[i] = g_ioOutputFuncs[sel].code;
            }

            ds->changed = true;
            SetWindowLongPtrA(hDlg, DWLP_MSGRESULT, PSNRET_NOERROR);
            return TRUE;
        }
        break;
    }

    case WM_DESTROY:
        KillTimer(hDlg, IDT_INPUT_POLL + 1);
        return TRUE;
    }
    return FALSE;
}

/* ===================================================================
 * Tab 6: Pin Map
 * =================================================================== */

/* Format a pin value for display: -1 = "n/a", else GPIO number */
static const char *PinStr(int8_t pin, char *buf, int bufLen)
{
    if (pin < 0)
        strncpy_s(buf, bufLen, "n/a", _TRUNCATE);
    else
        snprintf(buf, bufLen, "GPIO %d", (int)pin);
    return buf;
}

/* Refresh the pin map display labels from ds->currentPinMap */
static void UpdatePinMapDisplay(HWND hDlg, DialogState *ds)
{
    const PinMapEntry *pm = &ds->currentPinMap;
    const char *axNames[] = { "X", "Y", "Z", "A", "B", "C" };
    char buf[32];
    char text[64];
    int id = IDC_PINMAP_DISPLAY;

    /* Description */
    SetDlgItemTextA(hDlg, IDC_BOARD_DESC, pm->description);

    /* Step pins row */
    for (int i = 0; i < 6; i++) {
        snprintf(text, sizeof(text), "%s Step: %s", axNames[i], PinStr(pm->stepPin[i], buf, sizeof(buf)));
        SetDlgItemTextA(hDlg, id + i, text);
    }
    id += 6;

    /* Dir pins row */
    for (int i = 0; i < 6; i++) {
        snprintf(text, sizeof(text), "%s Dir:   %s", axNames[i], PinStr(pm->dirPin[i], buf, sizeof(buf)));
        SetDlgItemTextA(hDlg, id + i, text);
    }
    id += 6;

    /* Limit pins row */
    for (int i = 0; i < 6; i++) {
        snprintf(text, sizeof(text), "%s Limit: %s", axNames[i], PinStr(pm->limitPin[i], buf, sizeof(buf)));
        SetDlgItemTextA(hDlg, id + i, text);
    }
    id += 6;

    /* Home pins row */
    for (int i = 0; i < 6; i++) {
        snprintf(text, sizeof(text), "%s Home: %s", axNames[i], PinStr(pm->homePin[i], buf, sizeof(buf)));
        SetDlgItemTextA(hDlg, id + i, text);
    }
    id += 6;

    /* Single-pin outputs/inputs */
    snprintf(text, sizeof(text), "Enable:      %s", PinStr(pm->enablePin, buf, sizeof(buf)));
    SetDlgItemTextA(hDlg, id++, text);
    snprintf(text, sizeof(text), "Spindle:     %s", PinStr(pm->spindlePin, buf, sizeof(buf)));
    SetDlgItemTextA(hDlg, id++, text);
    snprintf(text, sizeof(text), "Charge Pump: %s", PinStr(pm->chargePumpPin, buf, sizeof(buf)));
    SetDlgItemTextA(hDlg, id++, text);
    snprintf(text, sizeof(text), "Misc Out 1:  %s", PinStr(pm->miscOutPin[0], buf, sizeof(buf)));
    SetDlgItemTextA(hDlg, id++, text);
    snprintf(text, sizeof(text), "Misc Out 2:  %s", PinStr(pm->miscOutPin[1], buf, sizeof(buf)));
    SetDlgItemTextA(hDlg, id++, text);
    snprintf(text, sizeof(text), "E-Stop:      %s", PinStr(pm->estopPin, buf, sizeof(buf)));
    SetDlgItemTextA(hDlg, id++, text);
    snprintf(text, sizeof(text), "Probe:       %s", PinStr(pm->probePin, buf, sizeof(buf)));
    SetDlgItemTextA(hDlg, id++, text);
    snprintf(text, sizeof(text), "Misc In 1:   %s", PinStr(pm->miscInPin[0], buf, sizeof(buf)));
    SetDlgItemTextA(hDlg, id++, text);
    snprintf(text, sizeof(text), "Misc In 2:   %s", PinStr(pm->miscInPin[1], buf, sizeof(buf)));
    SetDlgItemTextA(hDlg, id++, text);
    snprintf(text, sizeof(text), "Misc In 3:   %s", PinStr(pm->miscInPin[2], buf, sizeof(buf)));
    SetDlgItemTextA(hDlg, id++, text);
    snprintf(text, sizeof(text), "Misc In 4:   %s", PinStr(pm->miscInPin[3], buf, sizeof(buf)));
    SetDlgItemTextA(hDlg, id++, text);
    snprintf(text, sizeof(text), "Status LED:  %s", PinStr(pm->ledPin, buf, sizeof(buf)));
    SetDlgItemTextA(hDlg, id++, text);
}

/* ── Background pin map loading ── */
#define WM_PINMAP_LOADED  (WM_APP + 100)

struct PinMapLoadCtx {
    HWND         hDlg;
    DialogState *ds;
    PinMapEntry  result;
    bool         success;
};

static DWORD WINAPI PinMapLoadThread(LPVOID lpParam)
{
    PinMapLoadCtx *ctx = (PinMapLoadCtx *)lpParam;
    ctx->success = ctx->ds->network->ReadPinMap(&ctx->result);
    /* Post result to UI thread — if dialog was closed, message is silently dropped */
    PostMessageA(ctx->hDlg, WM_PINMAP_LOADED, 0, (LPARAM)ctx);
    return 0;
}

/* Start async live pin read.  Returns true if load was started (caller
 * should NOT fall back to offline).  Returns false if not connected. */
static bool StartLivePinMapLoad(HWND hDlg, DialogState *ds)
{
    if (!ds->network || !ds->network->IsConnected()) return false;

    SetDlgItemTextA(hDlg, IDC_PINMAP_NOTE, "Reading pins from controller...");
    EnableWindow(GetDlgItem(hDlg, IDC_PINMAP_REFRESH), FALSE);

    PinMapLoadCtx *ctx = new PinMapLoadCtx;
    ctx->hDlg = hDlg;
    ctx->ds   = ds;
    ctx->success = false;
    PinMap_Init(&ctx->result);

    HANDLE hThread = CreateThread(nullptr, 0, PinMapLoadThread, ctx, 0, nullptr);
    if (hThread) CloseHandle(hThread);
    return true;
}

/* Show live pin data after background load succeeded */
static void ShowLivePinMap(HWND hDlg, DialogState *ds)
{
    HWND hCombo = GetDlgItem(hDlg, IDC_BOARD_COMBO);
    SendMessageA(hCombo, CB_RESETCONTENT, 0, 0);
    char liveName[128];
    snprintf(liveName, sizeof(liveName), "%s (Live)", ds->currentPinMap.name);
    SendMessageA(hCombo, CB_ADDSTRING, 0, (LPARAM)liveName);
    SendMessageA(hCombo, CB_SETCURSEL, 0, 0);
    EnableWindow(hCombo, FALSE);

    UpdatePinMapDisplay(hDlg, ds);
    SetDlgItemTextA(hDlg, IDC_PINMAP_NOTE,
                    "Pin assignments read live from controller.");
}

/* Fall back to loading pin map from .pinmap files on disk */
static void LoadOfflinePinMap(HWND hDlg, DialogState *ds)
{
    HWND hCombo = GetDlgItem(hDlg, IDC_BOARD_COMBO);
    SendMessageA(hCombo, CB_RESETCONTENT, 0, 0);
    EnableWindow(hCombo, TRUE);

    int selIdx = 0;
    for (int i = 0; i < ds->boardCount; i++) {
        SendMessageA(hCombo, CB_ADDSTRING, 0, (LPARAM)ds->boardNames[i]);
        if (_stricmp(ds->boardNames[i], ds->editCfg.boardName) == 0)
            selIdx = i;
    }
    if (ds->boardCount > 0) {
        SendMessageA(hCombo, CB_SETCURSEL, selIdx, 0);
        PinMap_LoadFile(ds->boardPaths[selIdx], &ds->currentPinMap);
        UpdatePinMapDisplay(hDlg, ds);
    }
    SetDlgItemTextA(hDlg, IDC_PINMAP_NOTE,
                    "Offline - showing .pinmap file defaults. "
                    "Connect to controller for live pin data.");
}

static INT_PTR CALLBACK PinMapProc(HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam)
{
    switch (msg) {
    case WM_INITDIALOG:
    {
        if (lParam) {
            PROPSHEETPAGE *psp = (PROPSHEETPAGE *)lParam;
            SetWindowLongPtrA(hDlg, GWLP_USERDATA, (LONG_PTR)psp->lParam);
        }
        DialogState *ds = (DialogState *)GetWindowLongPtrA(hDlg, GWLP_USERDATA);
        if (!ds) return TRUE;

        /* Start async live read; fall back to .pinmap files if not connected */
        if (!StartLivePinMapLoad(hDlg, ds))
            LoadOfflinePinMap(hDlg, ds);

        return TRUE;
    }

    case WM_PINMAP_LOADED:
    {
        PinMapLoadCtx *ctx = (PinMapLoadCtx *)lParam;
        DialogState *ds = (DialogState *)GetWindowLongPtrA(hDlg, GWLP_USERDATA);
        EnableWindow(GetDlgItem(hDlg, IDC_PINMAP_REFRESH), TRUE);

        if (ctx && ds && ctx->success) {
            memcpy(&ds->currentPinMap, &ctx->result, sizeof(PinMapEntry));
            ShowLivePinMap(hDlg, ds);
        } else if (ds) {
            LoadOfflinePinMap(hDlg, ds);
        }
        delete ctx;
        return TRUE;
    }

    case WM_COMMAND:
    {
        DialogState *ds = (DialogState *)GetWindowLongPtrA(hDlg, GWLP_USERDATA);
        if (!ds) break;

        if (LOWORD(wParam) == IDC_BOARD_COMBO && HIWORD(wParam) == CBN_SELCHANGE) {
            HWND hCombo = GetDlgItem(hDlg, IDC_BOARD_COMBO);
            int sel = (int)SendMessageA(hCombo, CB_GETCURSEL, 0, 0);
            if (sel >= 0 && sel < ds->boardCount) {
                PinMap_LoadFile(ds->boardPaths[sel], &ds->currentPinMap);
                UpdatePinMapDisplay(hDlg, ds);
                PropSheet_Changed(GetParent(hDlg), hDlg);
            }
        }
        else if (LOWORD(wParam) == IDC_PINMAP_REFRESH) {
            /* Re-read live pins from ESP32 (async) */
            if (!StartLivePinMapLoad(hDlg, ds)) {
                LoadOfflinePinMap(hDlg, ds);
            }
        }
        return TRUE;
    }

    case WM_NOTIFY:
    {
        NMHDR *hdr = (NMHDR *)lParam;
        DialogState *ds = (DialogState *)GetWindowLongPtrA(hDlg, GWLP_USERDATA);
        if (!ds) break;

        if (hdr->code == PSN_APPLY) {
            /* Save selected board name */
            HWND hCombo = GetDlgItem(hDlg, IDC_BOARD_COMBO);
            int sel = (int)SendMessageA(hCombo, CB_GETCURSEL, 0, 0);
            if (sel >= 0 && sel < ds->boardCount) {
                strncpy_s(ds->editCfg.boardName, ds->boardNames[sel],
                          sizeof(ds->editCfg.boardName) - 1);
            }
            ds->changed = true;

            /* This is the last page — all pages have saved to editCfg.
             * Copy back to live config and notify the plugin. */
            memcpy(ds->cfg, &ds->editCfg, sizeof(PluginConfig));
            OnConfigChanged();

            SetWindowLongPtrA(hDlg, DWLP_MSGRESULT, PSNRET_NOERROR);
            return TRUE;
        }
        break;
    }
    }
    return FALSE;
}

static LPDLGTEMPLATE BuildPinMapPage()
{
    return CreatePageTemplate(0, PAGE_W, PAGE_H);
}

static INT_PTR CALLBACK PinMapPageProc(HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam)
{
    if (msg == WM_INITDIALOG) {
        /* Board selector + Refresh button */
        MakeGroupBox(hDlg, "Controller Board", 5, 2, 460, 50, -1);
        MakeLabel(hDlg, "Board:", 15, 20, 35, 14, -1);
        MakeCombo(hDlg, 55, 17, 250, 200, IDC_BOARD_COMBO);
        MakeButton(hDlg, "Refresh", 315, 16, 60, 20, IDC_PINMAP_REFRESH);
        MakeLabel(hDlg, "", 15, 36, 440, 14, IDC_BOARD_DESC);

        /* Pin map display: 4 groups of 6 axes + misc pins */
        int y = 58;
        int colW = 150;
        int id = IDC_PINMAP_DISPLAY;

        MakeGroupBox(hDlg, "Step / Direction Pins", 5, y, 460, 75, -1);
        y += 15;
        for (int i = 0; i < 6; i++) {
            MakeLabel(hDlg, "---", 15 + (i % 3) * colW, y + (i / 3) * 16, colW - 5, 14, id++);
        }
        y += 16;
        for (int i = 0; i < 6; i++) {
            MakeLabel(hDlg, "---", 15 + (i % 3) * colW, y + (i / 3) * 16, colW - 5, 14, id++);
        }
        y += 16 + 20;

        MakeGroupBox(hDlg, "Limit / Home Switch Pins", 5, y, 460, 75, -1);
        y += 15;
        for (int i = 0; i < 6; i++) {
            MakeLabel(hDlg, "---", 15 + (i % 3) * colW, y + (i / 3) * 16, colW - 5, 14, id++);
        }
        y += 16;
        for (int i = 0; i < 6; i++) {
            MakeLabel(hDlg, "---", 15 + (i % 3) * colW, y + (i / 3) * 16, colW - 5, 14, id++);
        }
        y += 16 + 20;

        MakeGroupBox(hDlg, "Output Pins", 5, y, 225, 105, -1);
        int oy = y + 16;
        for (int i = 0; i < 5; i++) {
            MakeLabel(hDlg, "---", 15, oy + i * 16, 200, 14, id++);
        }

        MakeGroupBox(hDlg, "Input Pins", 240, y, 225, 130, -1);
        int iy = y + 16;
        for (int i = 0; i < 7; i++) {
            MakeLabel(hDlg, "---", 250, iy + i * 16, 200, 14, id++);
        }

        /* Status note at bottom */
        MakeLabel(hDlg, "", 15, y + 134, 440, 14, IDC_PINMAP_NOTE);

        SetDialogFont(hDlg);
    }
    return PinMapProc(hDlg, msg, wParam, lParam);
}

/* ===================================================================
 * ShowConfigDialog - Main entry point (modeless / floating)
 * =================================================================== */

/* Persistent state for modeless dialog (survives function return) */
static HWND g_configHwnd = nullptr;
static DialogState *g_dialogState = nullptr;
static LPDLGTEMPLATE g_tpl[6] = {nullptr};
static WNDPROC g_origSheetProc = nullptr;

/* Cleanup helper */
static void CleanupDialogState()
{
    g_configHwnd = nullptr;
    g_origSheetProc = nullptr;
    if (g_dialogState) { delete g_dialogState; g_dialogState = nullptr; }
    s_state = nullptr;
    for (int i = 0; i < 6; i++) {
        if (g_tpl[i]) { GlobalFree((HGLOBAL)g_tpl[i]); g_tpl[i] = nullptr; }
    }
}

/* Subclass proc for the PropertySheet window.
 * Modeless PropertySheets don't auto-close on OK/Cancel/X — we need
 * to call DestroyWindow explicitly. */
static LRESULT CALLBACK SheetSubclassProc(HWND hWnd, UINT msg,
                                           WPARAM wParam, LPARAM lParam)
{
    switch (msg) {
    case WM_COMMAND:
        if (LOWORD(wParam) == IDOK) {
            /* Let original proc send PSN_APPLY to all pages first */
            CallWindowProcA(g_origSheetProc, hWnd, msg, wParam, lParam);
            ShowWindow(hWnd, SW_HIDE);
            return 0;
        }
        if (LOWORD(wParam) == IDCANCEL) {
            ShowWindow(hWnd, SW_HIDE);
            return 0;
        }
        break;

    case WM_CLOSE:
        ShowWindow(hWnd, SW_HIDE);
        return 0;

    case WM_DESTROY:
        SetWindowLongPtrA(hWnd, GWLP_WNDPROC, (LONG_PTR)g_origSheetProc);
        CleanupDialogState();
        break;
    }
    return CallWindowProcA(g_origSheetProc, hWnd, msg, wParam, lParam);
}

void ShowConfigDialog(HINSTANCE hInst, PluginConfig *cfg, NetworkClient *network, HWND parentWnd)
{
    /* Re-entry guard: if already exists, show and bring to front */
    if (g_configHwnd && IsWindow(g_configHwnd)) {
        ShowWindow(g_configHwnd, SW_SHOW);
        SetForegroundWindow(g_configHwnd);
        return;
    }

    /* Initialize common controls for PropertySheet */
    INITCOMMONCONTROLSEX icc = { sizeof(icc), ICC_TAB_CLASSES };
    InitCommonControlsEx(&icc);

    /* Allocate state on heap (persists while dialog is open) */
    if (g_dialogState) { delete g_dialogState; g_dialogState = nullptr; }
    g_dialogState = new DialogState;
    memset(g_dialogState, 0, sizeof(DialogState));
    g_dialogState->cfg = cfg;
    memcpy(&g_dialogState->editCfg, cfg, sizeof(PluginConfig));
    g_dialogState->network = network;
    g_dialogState->changed = false;
    s_state = g_dialogState;

    /* Determine DLL folder for .pinmap file scanning */
    {
        char dllPath[260] = {0};
        GetModuleFileNameA((HMODULE)hInst, dllPath, sizeof(dllPath));
        char *lastSlash = strrchr(dllPath, '\\');
        if (lastSlash) *lastSlash = '\0';
        strncpy_s(g_dialogState->pinmapFolder, dllPath,
                  sizeof(g_dialogState->pinmapFolder) - 1);

        g_dialogState->boardCount = PinMap_ScanFolder(
            g_dialogState->pinmapFolder,
            g_dialogState->boardNames,
            g_dialogState->boardPaths,
            PINMAP_MAX_BOARDS);
    }

    /* Free old templates if any */
    for (int i = 0; i < 6; i++) {
        if (g_tpl[i]) { GlobalFree((HGLOBAL)g_tpl[i]); g_tpl[i] = nullptr; }
    }

    /* Build page templates */
    g_tpl[0] = BuildConnectionPage();
    g_tpl[1] = BuildAxisPage();
    g_tpl[2] = BuildInputsPage();
    g_tpl[3] = BuildAdvancedPage();
    g_tpl[4] = BuildIOModulePage();
    g_tpl[5] = BuildPinMapPage();

    /* Property sheet pages */
    PROPSHEETPAGEA pages[6];
    memset(pages, 0, sizeof(pages));

    static const char *titles[6] = {
        "Connection", "Axis Config", "Inputs / Outputs", "Advanced", "I/O Module", "Pin Map"
    };
    static DLGPROC procs[6] = {
        ConnectionPageProc, AxisConfigPageProc, InputsPageProc,
        AdvancedPageProc, IOModulePageProc, PinMapPageProc
    };

    for (int i = 0; i < 6; i++) {
        pages[i].dwSize = sizeof(PROPSHEETPAGEA);
        pages[i].dwFlags = PSP_DLGINDIRECT | PSP_USETITLE;
        pages[i].pResource = g_tpl[i];
        pages[i].pfnDlgProc = procs[i];
        pages[i].pszTitle = titles[i];
        pages[i].lParam = (LPARAM)g_dialogState;
    }

    /* Property sheet header — MODELESS for floating window */
    PROPSHEETHEADERA psh;
    memset(&psh, 0, sizeof(psh));
    psh.dwSize = sizeof(PROPSHEETHEADERA);
    psh.dwFlags = PSH_PROPSHEETPAGE | PSH_MODELESS;
    psh.hwndParent = parentWnd;
    psh.hInstance = hInst;
    psh.pszCaption = "Tiggy Motion Controller Settings";
    psh.nPages = 6;
    psh.ppsp = pages;

    /* Create modeless dialog — returns HWND immediately */
    g_configHwnd = (HWND)PropertySheetA(&psh);

    /* Subclass the PropertySheet to handle OK/Cancel/X close */
    if (g_configHwnd) {
        g_origSheetProc = (WNDPROC)SetWindowLongPtrA(
            g_configHwnd, GWLP_WNDPROC, (LONG_PTR)SheetSubclassProc);

        /* Keep dialog on top of Mach3 so it's always visible */
        SetWindowPos(g_configHwnd, HWND_TOPMOST, 0, 0, 0, 0,
                     SWP_NOMOVE | SWP_NOSIZE);
    }
}

void CloseConfigDialog()
{
    if (g_configHwnd && IsWindow(g_configHwnd)) {
        /* Restore original wndproc before destroying to avoid re-entry */
        if (g_origSheetProc) {
            SetWindowLongPtrA(g_configHwnd, GWLP_WNDPROC, (LONG_PTR)g_origSheetProc);
        }
        DestroyWindow(g_configHwnd);
    }
    CleanupDialogState();
}
