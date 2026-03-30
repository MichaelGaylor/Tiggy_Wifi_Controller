/*
 * WiFi CNC Controller - Mach3 Device Interface
 *
 * Master include for the plugin DLL.
 * Provides Windows API, SDK engine structs, and plugin interface.
 *
 * NO MFC - plain Win32 DLL.
 */

#ifndef MACHDEVICE_H
#define MACHDEVICE_H

/* WIN32_LEAN_AND_MEAN must be defined BEFORE any Windows headers
 * to prevent COM headers (combaseapi.h) from being pulled in,
 * which would conflict with our COM stub exports. */
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

/* Winsock2 MUST come before windows.h to prevent winsock.h conflicts */
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

/* SDK engine structs + plugin interface (function pointers, externs, etc.) */
#include "MachIncludes/Mach3Plugin.h"

#endif /* MACHDEVICE_H */
