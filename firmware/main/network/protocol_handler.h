/*
 * WiFi CNC Controller - Protocol Handler
 *
 * Validates and dispatches incoming packets from both UDP and TCP channels.
 */

#ifndef PROTOCOL_HANDLER_H
#define PROTOCOL_HANDLER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "../../../protocol/wifi_cnc_protocol.h"

/* Validate packet header and checksum. Returns true if packet is valid. */
bool protocol_validate_packet(const uint8_t *data, size_t len);

/* Handle a UDP motion-channel packet. Called from udp_receive_task. */
void protocol_handle_udp_packet(const uint8_t *data, size_t len);

/* Handle a TCP control-channel packet. Called from tcp_server_task.
 * response_buf is filled with the response to send back.
 * Returns the response length, or 0 if no response needed. */
size_t protocol_handle_tcp_packet(const uint8_t *data, size_t len,
                                   uint8_t *response_buf, size_t response_buf_size);

/* Get current output states (ground truth from last IO control packet) */
uint8_t protocol_get_spindle_state(void);   /* 0=off, 1=CW, 2=CCW */
uint8_t protocol_get_coolant_state(void);   /* bit 0=flood, bit 1=mist */

/* Reset output states to off (call on client disconnect for safety) */
void protocol_reset_output_states(void);

#endif /* PROTOCOL_HANDLER_H */
