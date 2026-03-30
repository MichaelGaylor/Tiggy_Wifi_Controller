/*
 * WiFi CNC Controller - UDP Server
 *
 * Receives motion packets on the UDP motion port and dispatches them.
 * Also tracks the host address for sending status replies.
 */

#ifndef UDP_SERVER_H
#define UDP_SERVER_H

#include <stdint.h>
#include <stdbool.h>
#include "lwip/sockets.h"

/* FreeRTOS task: receives UDP motion packets */
void udp_receive_task(void *pvParameters);

/* Check if a host is registered (has sent at least one packet) */
bool udp_has_host(void);

/* Get the host address for sending status packets back */
bool udp_get_host_addr(struct sockaddr_in *addr);

/* Send a UDP packet to the registered host on the status port */
bool udp_send_status(const uint8_t *data, size_t len);

#endif /* UDP_SERVER_H */
