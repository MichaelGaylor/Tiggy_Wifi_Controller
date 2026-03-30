/*
 * WiFi CNC Controller - Plugin Core Header
 *
 * Thin include wrapper - brings in MachDevice.h which provides
 * all SDK types, engine structs, function pointers, and externs.
 */

#ifndef PLUGIN_H
#define PLUGIN_H

#include "MachDevice.h"
#include <cstdint>

/* Tracked output state: what the plugin last sent to the ESP32.
 * ConfigDialog uses these instead of Engine fields (which can have garbage). */
uint8_t GetSentSpindleState();  /* 0=off, 1=CW, 2=CCW */
uint8_t GetSentCoolantState();  /* bit0=flood, bit1=mist */
uint8_t GetSentMiscOutputs();   /* bit mask */

/* Spindle encoder feedback (updated from ESP32 status reports) */
uint16_t GetSpindleRPM();

/* I/O expansion module state */
bool IsIOModuleConnected();

/* Called by ConfigDialog when user clicks OK/Apply in modeless dialog.
 * Saves config to registry, re-applies, and syncs to ESP32. */
void OnConfigChanged();

#endif /* PLUGIN_H */
