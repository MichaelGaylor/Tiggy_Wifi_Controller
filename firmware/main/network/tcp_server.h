/*
 * WiFi CNC Controller - TCP Server
 *
 * Handles configuration and control commands over TCP.
 * Single-client server (only one host connected at a time).
 */

#ifndef TCP_SERVER_H
#define TCP_SERVER_H

/* FreeRTOS task: TCP server for config/control */
void tcp_server_task(void *pvParameters);

/* Check if a TCP client is connected */
#include <stdbool.h>
bool tcp_client_connected(void);

#endif /* TCP_SERVER_H */
