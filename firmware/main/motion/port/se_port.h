/*
 * Step engine port for the ESP32 family.
 *
 * The step timer is a general purpose timer counting microseconds; it is
 * never stopped, only its alarm is moved.  Pins are written with the
 * register-level GPIO functions, which are inline and safe in an interrupt
 * that must keep running while the flash cache is disabled.
 *
 * (The PC test build has its own se_port.h in firmware/test/pc.)
 */

#ifndef SE_PORT_H
#define SE_PORT_H

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "driver/gptimer.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"
#include "esp_attr.h"
#include "esp_rom_sys.h"
#include "../../../../protocol/wifi_cnc_protocol.h"

#define SE_ISR_ATTR     IRAM_ATTR
#define SE_DATA_ATTR    DRAM_ATTR

typedef struct {
    gptimer_handle_t timer;
    int8_t  step_pin[WCNC_MAX_AXES];        /* GPIO number, -1 = axis not fitted */
    int8_t  dir_pin[WCNC_MAX_AXES];
    uint8_t step_active[WCNC_MAX_AXES];     /* pin level during a step pulse */
    uint8_t dir_negative[WCNC_MAX_AXES];    /* pin level for travel in the negative direction */
    int8_t  limit_pin[WCNC_MAX_AXES];
    uint8_t limit_active[WCNC_MAX_AXES];    /* pin level when the switch is operated */
    int8_t  probe_pin;
    uint8_t probe_active;
} se_port_t;

extern se_port_t se_port;
extern portMUX_TYPE se_port_mux;

#define SE_LOCK()        portENTER_CRITICAL(&se_port_mux)
#define SE_UNLOCK()      portEXIT_CRITICAL(&se_port_mux)
#define SE_ISR_LOCK()    portENTER_CRITICAL_ISR(&se_port_mux)
#define SE_ISR_UNLOCK()  portEXIT_CRITICAL_ISR(&se_port_mux)

static inline uint64_t IRAM_ATTR port_now(void)
{
    uint64_t count = 0;
    gptimer_get_raw_count(se_port.timer, &count);
    return count;
}

static inline void IRAM_ATTR port_set_alarm(uint64_t t)
{
    gptimer_alarm_config_t alarm = {
        .alarm_count = t,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false,
    };
    gptimer_set_alarm_action(se_port.timer, &alarm);
}

static inline void IRAM_ATTR port_stop_alarm(void)
{
    gptimer_set_alarm_action(se_port.timer, NULL);
}

static inline void IRAM_ATTR port_set_dir(int axis, bool negative)
{
    int pin = se_port.dir_pin[axis];
    if (pin >= 0) gpio_ll_set_level(&GPIO, (uint32_t)pin, negative ? se_port.dir_negative[axis] : !se_port.dir_negative[axis]);
}

static inline void IRAM_ATTR port_step_high(uint8_t mask)
{
    for (int i = 0; i < WCNC_MAX_AXES; i++)
        if ((mask & (1u << i)) && se_port.step_pin[i] >= 0)
            gpio_ll_set_level(&GPIO, (uint32_t)se_port.step_pin[i], se_port.step_active[i]);
}

static inline void IRAM_ATTR port_step_low(uint8_t mask)
{
    for (int i = 0; i < WCNC_MAX_AXES; i++)
        if ((mask & (1u << i)) && se_port.step_pin[i] >= 0)
            gpio_ll_set_level(&GPIO, (uint32_t)se_port.step_pin[i], !se_port.step_active[i]);
}

static inline void IRAM_ATTR port_delay_us(uint32_t us)
{
    esp_rom_delay_us(us);
}

static inline bool IRAM_ATTR port_limit_active(int input)
{
    if (input < 0 || input >= WCNC_MAX_AXES || se_port.limit_pin[input] < 0) return false;
    return (uint8_t)gpio_ll_get_level(&GPIO, (uint32_t)se_port.limit_pin[input]) == se_port.limit_active[input];
}

static inline bool IRAM_ATTR port_probe_active(void)
{
    if (se_port.probe_pin < 0) return false;
    return (uint8_t)gpio_ll_get_level(&GPIO, (uint32_t)se_port.probe_pin) == se_port.probe_active;
}

#endif /* SE_PORT_H */
