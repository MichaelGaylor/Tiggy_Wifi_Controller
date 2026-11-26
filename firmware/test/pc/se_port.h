/*
 * Step engine port for the PC test build.
 *
 * There is no timer and no interrupt: time is a number (pc_now_us) that the
 * harness advances, calling se_isr() exactly when the alarm is due.  Steps
 * move a simple machine model with limit switches and a probe.  The busy
 * waits of the real interrupt advance the clock by the same amount, so the
 * engine's handling of its own execution time is exercised too.
 */

#ifndef SE_PORT_H
#define SE_PORT_H

#include <stdbool.h>
#include <stdint.h>

#define SE_ISR_ATTR
#define SE_DATA_ATTR
#define SE_LOCK()        ((void)0)
#define SE_UNLOCK()      ((void)0)
#define SE_ISR_LOCK()    ((void)0)
#define SE_ISR_UNLOCK()  ((void)0)

extern uint64_t pc_now_us;
extern uint64_t pc_alarm_us;
extern bool     pc_alarm_set;

void pc_set_dir(int axis, bool negative);
void pc_step(uint8_t axis_mask);
bool pc_limit_active(int input);
bool pc_probe_active(void);

static inline uint64_t port_now(void)                 { return pc_now_us; }
static inline void port_set_alarm(uint64_t t)         { pc_alarm_us = t; pc_alarm_set = true; }
static inline void port_stop_alarm(void)              { pc_alarm_set = false; }
static inline void port_set_dir(int axis, bool neg)   { pc_set_dir(axis, neg); }
static inline void port_step_high(uint8_t mask)       { pc_step(mask); }
static inline void port_step_low(uint8_t mask)        { (void)mask; }
static inline void port_delay_us(uint32_t us)         { pc_now_us += us; }
static inline bool port_limit_active(int input)       { return pc_limit_active(input); }
static inline bool port_probe_active(void)            { return pc_probe_active(); }

#endif /* SE_PORT_H */
