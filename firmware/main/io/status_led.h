/*
 * WiFi CNC Controller - Status LED
 *
 * Drives the onboard status LED with color support for WS2812 (ESP32-S3)
 * or simple on/off for plain LEDs (ESP32-WROOM-32).
 *
 * Colors (WS2812 boards):
 *   Red    = disconnected / no host
 *   Blue   = connected, idle
 *   Green  = connected, running
 *   Yellow = alarm / E-stop
 *
 * Plain LED boards:
 *   Off       = disconnected
 *   Solid on  = connected, idle
 *   Blink     = running or alarm
 */

#ifndef STATUS_LED_H
#define STATUS_LED_H

typedef enum {
    LED_STATE_DISCONNECTED,   /* Red / off */
    LED_STATE_CONNECTED_IDLE, /* Blue / solid on */
    LED_STATE_RUNNING,        /* Green / blink slow */
    LED_STATE_ALARM,          /* Yellow / blink fast */
} status_led_state_t;

/* Initialize LED hardware (call once at startup) */
void status_led_init(void);

/* Set the LED state (call from status reporter task) */
void status_led_set_state(status_led_state_t state);

/* Tick the LED blink state (call each status report cycle, ~50ms) */
void status_led_tick(void);

#endif /* STATUS_LED_H */
