/*
 * WiFi CNC Controller - WiFi Connection Manager
 *
 * Manages WiFi lifecycle:
 *   1. Attempt STA mode with stored credentials
 *   2. Fall back to AP mode if STA fails
 *   3. Auto-reconnect with exponential backoff
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Initialize WiFi subsystem and begin connection attempt */
void wifi_manager_init(void);

/* Check if WiFi is connected in STA mode */
bool wifi_is_connected(void);

/* Check if running in AP mode */
bool wifi_is_ap_mode(void);

/* Get current IP address as string. Returns "0.0.0.0" if not connected. */
void wifi_get_ip_string(char *buf, size_t buf_len);

/* Get WiFi RSSI (signal strength). Returns 0 if not connected. */
int8_t wifi_get_rssi(void);

#endif /* WIFI_MANAGER_H */
