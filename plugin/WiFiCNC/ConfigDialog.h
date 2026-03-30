/*
 * WiFi CNC Controller - Configuration Dialog
 *
 * Win32 PropertySheet tabbed dialog for plugin settings.
 * Modeless (floating) — user can interact with Mach3 while open.
 * Replaces Mach3's Motor Tuning panel (unreliable with external controllers).
 */

#ifndef CONFIGDIALOG_H
#define CONFIGDIALOG_H

#include "PluginConfig.h"

/* Forward declarations to avoid pulling in <windows.h> here
 * (prevents winsock1 vs winsock2 include-order conflicts). */
typedef struct HINSTANCE__ *HINSTANCE;
typedef struct HWND__ *HWND;

class NetworkClient;

/*
 * Show the tabbed configuration dialog (modeless / floating).
 * If already open, brings the existing window to the foreground.
 *
 * cfg      - Plugin config to edit (modified in place on OK/Apply)
 * network  - For live status display and connect/disconnect (may be NULL)
 * parentWnd - Mach3 main window (dialog groups with it in taskbar)
 */
void ShowConfigDialog(HINSTANCE hInst, PluginConfig *cfg, NetworkClient *network, HWND parentWnd = nullptr);

/* Close the config dialog if open (called on plugin shutdown). */
void CloseConfigDialog();

#endif /* CONFIGDIALOG_H */
