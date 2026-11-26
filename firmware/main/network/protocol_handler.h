/*
 * WiFi CNC Controller - Protocol Handler
 *
 * Packet validation, and the TCP control packets (handshake, keep-alive,
 * configuration).  Datagrams are handled by the motion core.
 */

#ifndef PROTOCOL_HANDLER_H
#define PROTOCOL_HANDLER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "../../../protocol/wifi_cnc_protocol.h"

/* Check magic, version, length and checksum.  Returns true if valid. */
bool protocol_validate_packet(const uint8_t *data, size_t len);

/* Process one packet from the TCP control channel.
 * Returns the number of bytes written to response_buf, 0 for no reply. */
size_t protocol_handle_tcp_packet(const uint8_t *data, size_t len,
                                   uint8_t *response_buf, size_t response_buf_size);

#endif /* PROTOCOL_HANDLER_H */
