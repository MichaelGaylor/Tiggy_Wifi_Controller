/*
 * WiFi CNC Controller - Ethernet Manager (W5500 SPI)
 *
 * Optional Ethernet connectivity via W5500 SPI module.
 * If a W5500 is detected at boot, Ethernet is used instead of WiFi.
 * If no W5500 is found, the system falls back to WiFi silently.
 *
 * Pin assignments are configurable via saved settings (Protocol Tester).
 * Default pins for the Tiggy Pro board:
 *   MOSI = GPIO 45, MISO = GPIO 46, SCLK = GPIO 0, INT = GPIO 47
 *   CS tied to GND (always selected), RESET tied to 3.3V (always active)
 */

#ifndef ETH_MANAGER_H
#define ETH_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Initialize W5500 Ethernet.
 * Returns true if Ethernet link is up and IP assigned.
 * Returns false if W5500 not detected or link failed (caller should use WiFi).
 */
bool eth_manager_init(void);

/* Check if Ethernet is connected and has an IP address */
bool eth_is_connected(void);

/* Get current Ethernet IP address as string. Returns "0.0.0.0" if not connected. */
void eth_get_ip_string(char *buf, size_t buf_len);

#endif /* ETH_MANAGER_H */
