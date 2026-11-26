/*
 * WiFi CNC Controller - UDP Server
 *
 * Receives datagrams on WCNC_UDP_MOTION_PORT and passes them to the motion
 * task.  Replies (status reports, discovery answers) leave from the same
 * socket, so a host that asks for them on its source port needs no fixed
 * listening port and no firewall rule.
 */

#ifndef UDP_SERVER_H
#define UDP_SERVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* FreeRTOS task: receive loop */
void udp_receive_task(void *pvParameters);

/* Send a datagram.  ip is in network byte order.  May be called from any task. */
bool udp_send_to(uint32_t ip, uint16_t port, const void *data, size_t len);

/* True once the network interface (WiFi station, access point or Ethernet) is up. */
bool network_is_up(void);

#endif /* UDP_SERVER_H */
