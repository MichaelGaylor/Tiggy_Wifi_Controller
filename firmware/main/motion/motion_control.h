/*
 * WiFi CNC Controller - Motion task
 *
 * One task owns the motion core (motion_core.h) and the step engine.
 * Everything else talks to it through the functions below, which hand the
 * work over to that task: datagrams from the network, the state of the TCP
 * session, and the local commands used by G-code mode.
 */

#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "../../../protocol/wifi_cnc_protocol.h"

/* Create the queue the functions below use.  Call before starting tasks. */
void motion_control_init(void);

/* The motion task (pin it to the core that should run the step interrupt). */
void motion_control_task(void *pvParameters);

/* --- From the network tasks --- */
/* A datagram for the motion port.  ip is in network byte order. */
void motion_post_udp(const uint8_t *data, size_t len, uint32_t ip, uint16_t port);
/* The TCP control connection was made by ip (0: it has ended). */
void motion_tcp_session(uint32_t ip);
/* True if the host that owns the controller has been heard from lately. */
bool motion_host_active(void);
/* A setting changed: apply it. */
void motion_config_changed(void);

/* --- Local commands (G-code mode); these return when carried out --- */
void motion_estop(void);
void motion_reset(void);
void motion_feed_hold(void);
void motion_feed_resume(void);
void motion_home(uint8_t axis_mask);

/* --- State --- */
uint8_t motion_capabilities(void);          /* WCNC_CAP_* of this device */
wcnc_machine_state_t motion_get_state(void);

bool    motion_is_homing(void);
uint8_t motion_homing_state(void);

#endif /* MOTION_CONTROL_H */
