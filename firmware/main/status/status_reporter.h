/*
 * WiFi CNC Controller - Status Reporter
 *
 * Periodically sends status reports (position, buffer level, state)
 * to the host via UDP.
 */

#ifndef STATUS_REPORTER_H
#define STATUS_REPORTER_H

/* FreeRTOS task: sends periodic status reports via UDP */
void status_report_task(void *pvParameters);

#endif /* STATUS_REPORTER_H */
