/*
 * WiFi CNC Controller - G-Code Telnet Server
 *
 * Listens on port 23 for G-code sender connections (UGS, CNCjs, bCNC).
 * Single-client, line-oriented protocol. Real-time characters (?, !, ~, Ctrl-X)
 * are detected and handled immediately.
 */

#ifndef GCODE_TELNET_H
#define GCODE_TELNET_H

#include <stdbool.h>

/* Default telnet port */
#define GCODE_TELNET_PORT  23

/* FreeRTOS task: Telnet server for G-code */
void gcode_telnet_task(void *pvParameters);

/* Check if a telnet G-code client is connected */
bool gcode_telnet_connected(void);

#endif /* GCODE_TELNET_H */
