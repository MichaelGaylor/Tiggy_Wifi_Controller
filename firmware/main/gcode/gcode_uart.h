/*
 * WiFi CNC Controller - G-Code UART Handler
 *
 * Receives G-code lines over USB serial (UART0, 115200 baud).
 * Feeds lines to gcode_interface for execution.
 */

#ifndef GCODE_UART_H
#define GCODE_UART_H

#include <stdbool.h>

/* FreeRTOS task: UART receive loop */
void gcode_uart_task(void *pvParameters);

/* Check if UART input is active */
bool gcode_uart_active(void);

#endif /* GCODE_UART_H */
