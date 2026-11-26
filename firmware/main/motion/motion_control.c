/*
 * WiFi CNC Controller - Motion task
 *
 * Runs the motion core once a millisecond and feeds it what arrives from
 * the network and from G-code mode.  The core and the step engine are only
 * ever touched from this task, so none of that code needs locking; other
 * tasks post messages (see motion_control.h).
 *
 * This file is also the core's platform layer: the mc_hal_* functions.
 */

#include "motion_control.h"
#include "motion_core.h"
#include "step_engine.h"
#include "stepper.h"
#include "planner.h"
#include "se_port.h"
#include "../config.h"
#include "../io/gpio_control.h"
#include "../io/io_module.h"
#include "../io/spindle_encoder.h"
#include "../persist/nvs_config.h"
#include "../network/udp_server.h"
#include "../network/wifi_manager.h"
#include "../network/eth_manager.h"
#include "../gcode/gcode_interface.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"

static const char *TAG = "motion";

/* Largest datagram the core has a use for: a motion packet with eight segments */
#define MSG_DATA_SIZE   (sizeof(wcnc_motion_packet_t))
#define MSG_QUEUE_LEN   12

enum { MSG_UDP, MSG_TCP_SESSION, MSG_ESTOP, MSG_RESET, MSG_HOLD, MSG_RESUME, MSG_HOME };

typedef struct {
    uint8_t  kind;
    uint8_t  arg;
    uint16_t len;
    uint32_t ip;
    uint16_t port;
    TaskHandle_t waiter;                /* task to wake when a local command has been carried out */
    uint8_t  data[MSG_DATA_SIZE];
} motion_msg_t;

static QueueHandle_t s_queue = NULL;
static TaskHandle_t s_task = NULL;
static volatile bool s_config_dirty = false;
static bool s_io_module = false;
static mc_outputs_t s_applied;
static bool s_outputs_dirty = true;
static volatile uint8_t s_home_next = 0;   /* axes to home once the group now homing is done */

/* ===================================================================
 * Platform layer of the motion core
 * =================================================================== */

uint32_t mc_hal_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

void mc_hal_load_config(mc_config_t *c)
{
    static const char *rate_keys[] = { "rate_x", "rate_y", "rate_z", "rate_a", "rate_b", "rate_c" };
    static const char *acc_keys[]  = { "acc_x", "acc_y", "acc_z", "acc_a", "acc_b", "acc_c" };
    memset(c, 0, sizeof(*c));
    for (int i = 0; i < WCNC_MAX_AXES; i++) {
        c->max_rate[i] = nvs_config_get_u32(rate_keys[i], CFG_DEFAULT_MAX_RATE);
        c->accel[i] = nvs_config_get_u32(acc_keys[i], CFG_DEFAULT_ACCELERATION);
        if (c->accel[i] < 1) c->accel[i] = 1;
    }
    c->limit_enable_mask = nvs_config_get_u8("lim_en", 0x3F);
    c->homing_dir_mask = nvs_config_get_u8("hm_dir", CFG_DEFAULT_HOMING_DIR_MASK);
    c->homing_seek_rate = nvs_config_get_u32("hm_seek", CFG_DEFAULT_HOMING_SEEK_RATE);
    c->homing_feed_rate = nvs_config_get_u32("hm_feed", CFG_DEFAULT_HOMING_FEED_RATE);
    c->homing_pulloff = nvs_config_get_u32("hm_pull", CFG_DEFAULT_HOMING_PULLOFF);
    c->idle_release_ms = nvs_config_get_u16("idle_ms", CFG_DEFAULT_STEP_IDLE_DELAY_MS);
    c->status_interval_ms = nvs_config_get_u16("stat_ms", CFG_DEFAULT_STATUS_INTERVAL_MS);
    if (c->status_interval_ms < 10) c->status_interval_ms = 10;
    if (c->status_interval_ms > 1000) c->status_interval_ms = 1000;
    c->io_module = s_io_module;
}

uint8_t mc_hal_limit_inputs(void) { return gpio_control_get_limit_mask(); }
bool    mc_hal_probe_input(void)  { return gpio_control_get_probe(); }
bool    mc_hal_estop_input(void)  { return gpio_control_get_estop(); }

void mc_hal_apply_outputs(const mc_outputs_t *out)
{
    if (!s_outputs_dirty && memcmp(out, &s_applied, sizeof(*out)) == 0) return;
    s_applied = *out;
    s_outputs_dirty = false;
    if (s_io_module) io_module_set_outputs(out->io);
    else gpio_control_apply_outputs(out->misc, out->spindle, out->rpm, out->charge_pump);
}

void mc_hal_drivers(bool enabled)
{
    if (!s_io_module) stepper_set_enabled(enabled);
}

void mc_hal_udp_send(uint32_t ip, uint16_t port, const void *data, size_t len)
{
    udp_send_to(ip, port, data, len);
}

void mc_hal_fill_status(wcnc_status_report_t *r)
{
    r->limit_switches = gpio_control_get_limit_mask();
    r->home_switches = gpio_control_get_home_mask();
    r->probe_state = gpio_control_get_probe() ? 1 : 0;
    r->estop_input = gpio_control_get_estop() ? 1 : 0;
    r->misc_inputs = gpio_control_get_misc_input_mask();
    if (wifi_is_connected() || eth_is_connected()) r->flags |= WCNC_STATUS_WIFI_CONNECTED;
    r->uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (spindle_encoder_is_available()) {
        r->spindle_rpm = spindle_encoder_get_rpm();
        r->spindle_position = spindle_encoder_get_position();
        r->spindle_index_count = spindle_encoder_get_index_count();
    }
    if (io_module_is_active()) {
        r->io_inputs = io_module_get_inputs();
        r->io_outputs = io_module_get_outputs();
    }
}

uint8_t motion_capabilities(void)
{
    uint8_t caps = WCNC_CAP_STREAM_V2;
    if (s_io_module) return caps | WCNC_CAP_IO_MODULE;
    caps |= gpio_control_get_capabilities();
    if (spindle_encoder_is_available()) caps |= WCNC_CAP_SPINDLE_ENCODER;
    if (spindle_encoder_can_sync()) caps |= WCNC_CAP_SPINDLE_SYNC;
    return caps;
}

void mc_hal_identity(uint8_t *capabilities, uint8_t mac[6], char name[WCNC_DEVICE_NAME_LEN])
{
    *capabilities = motion_capabilities();
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) memset(mac, 0, 6);
    strlcpy(name, s_io_module ? "Tiggy I/O Module" : "Tiggy", WCNC_DEVICE_NAME_LEN);
}

bool mc_hal_legacy_pop(wcnc_motion_segment_t *seg)        { return planner_pop_segment(seg); }
bool mc_hal_legacy_push(const wcnc_motion_segment_t *seg) { return planner_push_segment(seg); }
void mc_hal_legacy_clear(void)                            { planner_drain(); }
void mc_hal_legacy_space(uint16_t *free_slots, uint16_t *total)
{
    *free_slots = planner_available();
    *total = planner_capacity();
}

void mc_hal_log(const char *text)
{
    ESP_LOGW(TAG, "%s", text);
}

/* ===================================================================
 * Messages to the motion task
 * =================================================================== */

void motion_control_init(void)
{
    s_io_module = nvs_config_get_u8("dev_mode", 0) == 1;
    s_queue = xQueueCreate(MSG_QUEUE_LEN, sizeof(motion_msg_t));
    configASSERT(s_queue);
}

void motion_post_udp(const uint8_t *data, size_t len, uint32_t ip, uint16_t port)
{
    static motion_msg_t msg;            /* only the UDP receive task calls this */
    if (!s_queue || len < sizeof(wcnc_header_t)) return;
    if (len > MSG_DATA_SIZE) len = MSG_DATA_SIZE;
    msg.kind = MSG_UDP;
    msg.len = (uint16_t)len;
    msg.ip = ip;
    msg.port = port;
    msg.waiter = NULL;
    memcpy(msg.data, data, len);

    /* Motion segments are re-sent by the host if one is dropped here, so
     * they never wait.  Everything else is small, rare and matters now:
     * it goes to the head of the queue and may wait a moment for room. */
    uint8_t type = ((const wcnc_header_t *)data)->packet_type;
    if (type == WCNC_PKT_MOTION_SEGMENT) xQueueSend(s_queue, &msg, 0);
    else xQueueSendToFront(s_queue, &msg, pdMS_TO_TICKS(3));
}

static void post_simple(uint8_t kind, uint8_t arg, uint32_t ip, bool wait)
{
    if (!s_queue) return;
    motion_msg_t *msg = pvPortMalloc(sizeof(motion_msg_t));
    if (!msg) return;
    msg->kind = kind;
    msg->arg = arg;
    msg->len = 0;
    msg->ip = ip;
    msg->port = 0;
    msg->waiter = (wait && xTaskGetCurrentTaskHandle() != s_task) ? xTaskGetCurrentTaskHandle() : NULL;
    bool sent = xQueueSend(s_queue, msg, pdMS_TO_TICKS(50)) == pdTRUE;
    bool waiting = sent && msg->waiter != NULL;
    vPortFree(msg);
    if (waiting) ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));
}

void motion_tcp_session(uint32_t ip) { post_simple(MSG_TCP_SESSION, 0, ip, true); }
void motion_estop(void)              { post_simple(MSG_ESTOP, 0, 0, true); }
void motion_reset(void)              { post_simple(MSG_RESET, 0, 0, true); }
void motion_feed_hold(void)          { post_simple(MSG_HOLD, 0, 0, true); }
void motion_feed_resume(void)        { post_simple(MSG_RESUME, 0, 0, true); }
void motion_home(uint8_t axis_mask)  { post_simple(MSG_HOME, axis_mask, 0, true); }

void motion_config_changed(void)     { s_config_dirty = true; }

bool motion_host_active(void)
{
    return mc_host_present() && mc_host_silence_ms() < 3000;
}

wcnc_machine_state_t motion_get_state(void) { return mc_state(); }
bool    motion_is_homing(void)              { return mc_state() == WCNC_STATE_HOMING || s_home_next != 0; }
uint8_t motion_homing_state(void)           { return mc_homing_mask(); }

/* ===================================================================
 * The task
 * =================================================================== */

static void handle(const motion_msg_t *msg)
{
    switch (msg->kind) {
    case MSG_UDP: {
        /* A host streaming motion takes priority over G-code mode until it
         * resets the controller (as before protocol 1.2). */
        uint8_t type = ((const wcnc_header_t *)msg->data)->packet_type;
        if (type == WCNC_PKT_MOTION_SEGMENT) gcode_planner_preempt();
        else if (type == WCNC_PKT_RESET) gcode_planner_release();
        mc_on_udp(msg->data, msg->len, msg->ip, msg->port);
        break;
    }
    case MSG_TCP_SESSION: mc_tcp_session(msg->ip); break;
    case MSG_ESTOP:       mc_estop(); break;
    case MSG_RESET:       mc_reset(); break;
    case MSG_HOLD:        mc_feed_hold(); break;
    case MSG_RESUME:      mc_feed_resume(); break;
    case MSG_HOME: {
        /* "Home all" from G-code mode: only axes that have both a motor and
         * a switch, and Z on its own first so the tool is clear of the work
         * before anything moves sideways. */
        uint8_t mask = 0;
        for (int i = 0; i < WCNC_MAX_AXES; i++)
            if (se_port.step_pin[i] >= 0 && se_port.limit_pin[i] >= 0) mask |= (uint8_t)(1u << i);
        mask &= msg->arg;
        uint8_t first = (mask & 0x04) ? 0x04 : mask;
        s_home_next = (uint8_t)(mask & ~first);
        if (!first || !mc_home(first)) s_home_next = 0;
        break;
    }
    default: break;
    }
    if (msg->waiter) xTaskNotifyGive(msg->waiter);
}

void motion_control_task(void *pvParameters)
{
    (void)pvParameters;
    static motion_msg_t msg;
    s_task = xTaskGetCurrentTaskHandle();

    /* The step interrupt belongs to the core this task runs on, and so do
     * the interrupts that read the spindle sensor */
    if (!s_io_module) {
        stepper_start_timer();
        spindle_encoder_start();
    }
    mc_init();
    ESP_LOGI(TAG, "Motion task running on core %d (%s)", xPortGetCoreID(),
             s_io_module ? "I/O module" : "motion controller");

    TickType_t last = xTaskGetTickCount();
    for (;;) {
        if (xQueueReceive(s_queue, &msg, 1) == pdTRUE) {
            handle(&msg);
            for (int n = 0; n < MSG_QUEUE_LEN && xQueueReceive(s_queue, &msg, 0) == pdTRUE; n++) handle(&msg);
        }
        TickType_t now = xTaskGetTickCount();
        if (now == last) continue;
        last = now;

        if (s_config_dirty) {
            s_config_dirty = false;
            if (!s_io_module) stepper_apply_timing_config();
            gpio_control_reload_inversion();
            mc_config_changed();
            /* Spindle PWM and charge pump settings take effect at once */
            mc_outputs_t out;
            mc_get_outputs(&out);
            s_outputs_dirty = true;
            mc_hal_apply_outputs(&out);
        }
        gpio_control_poll();
        mc_tick();
        if (!s_io_module) se_watchdog();

        if (s_home_next) {
            wcnc_machine_state_t state = mc_state();
            if (state == WCNC_STATE_ALARM || state == WCNC_STATE_ESTOP) {
                s_home_next = 0;
            } else if (state == WCNC_STATE_IDLE && !mc_busy()) {
                uint8_t mask = s_home_next;
                s_home_next = 0;
                mc_home(mask);
            }
        }

    }
}
