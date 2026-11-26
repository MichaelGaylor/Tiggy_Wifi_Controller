/*
 * WiFi CNC Controller - Motion core
 *
 * The behaviour of the controller: which host it listens to, the motion
 * stream, jogging, homing, probing, feed hold, E-stop and alarms, limit
 * switches, the host watchdog, outputs and the status report.
 *
 * It is plain C with no operating system or hardware calls.  Motion goes to
 * the step engine (step_engine.h); everything else it needs from the
 * platform is the short list of mc_hal_* functions below, implemented once
 * for the ESP32 (motion_control.c) and once for the PC test build
 * (firmware/test/pc), where the same code runs against the protocol
 * conformance tests.
 *
 * All functions must be called from one task.
 */

#ifndef MOTION_CORE_H
#define MOTION_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "../../../protocol/wifi_cnc_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------
 * Settings the core works with (read through mc_hal_load_config)
 * --------------------------------------------------------------------- */
typedef struct {
    uint32_t max_rate[WCNC_MAX_AXES];       /* steps/s */
    uint32_t accel[WCNC_MAX_AXES];          /* steps/s^2 */
    uint8_t  limit_enable_mask;             /* limit inputs that raise an alarm */
    uint8_t  homing_dir_mask;               /* stored homing set-up, used by */
    uint32_t homing_seek_rate;              /*   a home command without its  */
    uint32_t homing_feed_rate;              /*   own parameters              */
    uint32_t homing_pulloff;
    uint16_t idle_release_ms;               /* release the drivers after this idle time, 0 = never */
    uint16_t status_interval_ms;
    bool     io_module;                     /* device runs as an I/O module: no motion */
} mc_config_t;

typedef struct {
    uint8_t  misc;                          /* auxiliary outputs, bit per output */
    uint8_t  spindle;                       /* 0 off, 1 clockwise, 2 counter-clockwise */
    uint16_t rpm;
    uint8_t  coolant;                       /* bit 0 flood, bit 1 mist */
    bool     charge_pump;
    uint16_t io;                            /* I/O module outputs */
} mc_outputs_t;

/* ---------------------------------------------------------------------
 * Provided by the platform
 * --------------------------------------------------------------------- */
uint32_t mc_hal_ms(void);                                   /* monotonic milliseconds */
void     mc_hal_load_config(mc_config_t *cfg);
uint8_t  mc_hal_limit_inputs(void);                         /* bit per limit input, set = switch operated */
bool     mc_hal_probe_input(void);
bool     mc_hal_estop_input(void);
void     mc_hal_apply_outputs(const mc_outputs_t *out);
void     mc_hal_drivers(bool enabled);                      /* motor driver enable */
void     mc_hal_udp_send(uint32_t ip, uint16_t port, const void *data, size_t len);  /* ip in network byte order */
/* Fill in what only the platform knows: inputs, encoder, uptime, link flags. */
void     mc_hal_fill_status(wcnc_status_report_t *report);
/* For the discovery reply. */
void     mc_hal_identity(uint8_t *capabilities, uint8_t mac[6], char name[WCNC_DEVICE_NAME_LEN]);
/* Segments in the old, untimed form (G-code mode, hosts older than 1.2). */
bool     mc_hal_legacy_pop(wcnc_motion_segment_t *seg);
bool     mc_hal_legacy_push(const wcnc_motion_segment_t *seg);
void     mc_hal_legacy_clear(void);
/* Room in that queue: what a host older than 1.2 paces itself by. */
void     mc_hal_legacy_space(uint16_t *free_slots, uint16_t *total);
void     mc_hal_log(const char *text);

/* ---------------------------------------------------------------------
 * Driving the core
 * --------------------------------------------------------------------- */
void mc_init(void);

/* Call about once a millisecond. */
void mc_tick(void);

/* A datagram arrived on the motion port.  ip in network byte order. */
void mc_on_udp(const uint8_t *data, size_t len, uint32_t ip, uint16_t port);

/* The TCP control connection was made by ip, or (ip == 0) has ended. */
void mc_tcp_session(uint32_t ip);

/* A setting was changed: read the configuration again. */
void mc_config_changed(void);

/* --- Local control (G-code mode, hardware inputs) --- */
void mc_estop(void);
bool mc_reset(void);                        /* false: refused (E-stop input still active) */
void mc_feed_hold(void);
void mc_feed_resume(void);
bool mc_home(uint8_t axis_mask);            /* with the stored homing set-up */
void mc_abort_motion(void);                 /* stop and discard motion, stay ready */

/* --- State --- */
wcnc_machine_state_t mc_state(void);
uint8_t  mc_alarm(void);
uint8_t  mc_homing_mask(void);              /* axes being homed now */
bool     mc_busy(void);                     /* anything moving or still queued */
bool     mc_probe_latched(void);            /* the last probe move ended by contact */
bool     mc_host_present(void);
uint32_t mc_host_silence_ms(void);          /* time since the host was last heard on UDP */

uint32_t mc_max_step_rate(void);            /* fastest step rate an axis can be given */
void     mc_get_outputs(mc_outputs_t *out);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_CORE_H */
