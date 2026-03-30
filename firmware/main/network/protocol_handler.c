/*
 * WiFi CNC Controller - Protocol Handler
 *
 * Central packet dispatcher for both UDP and TCP channels.
 * Validates headers and checksums, then routes to appropriate handlers.
 */

#include "protocol_handler.h"
#include "../motion/planner.h"
#include "../motion/stepper.h"
#include "../motion/motion_control.h"
#include "../persist/nvs_config.h"
#include "../io/gpio_control.h"
#include "../io/spindle_encoder.h"
#include "../io/io_module.h"
#include "../gcode/gcode_interface.h"
#include "../config.h"

#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

static const char *TAG = "protocol";

static uint32_t s_tx_sequence = 0;
static bool s_wifi_config_changed = false;

/* Tracked output states — ground truth of what ESP32 is actually doing.
 * Set when IO control packet is received, read by status reporter. */
static uint8_t s_spindle_state = 0;   /* 0=off, 1=CW, 2=CCW */
static uint8_t s_coolant_state = 0;   /* bit 0=flood, bit 1=mist */
static esp_timer_handle_t s_restart_timer = NULL;

static void restart_timer_callback(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Restarting to apply new WiFi configuration...");
    esp_restart();
}

/* ===================================================================
 * Packet Validation
 * =================================================================== */

bool protocol_validate_packet(const uint8_t *data, size_t len)
{
    if (len < sizeof(wcnc_header_t)) {
        ESP_LOGW(TAG, "Packet too short: %d < %d", (int)len, (int)sizeof(wcnc_header_t));
        return false;
    }

    const wcnc_header_t *hdr = (const wcnc_header_t *)data;
    if (hdr->magic != WCNC_MAGIC) {
        ESP_LOGW(TAG, "Bad magic: 0x%08X (expected 0x%08X)", (unsigned)hdr->magic, (unsigned)WCNC_MAGIC);
        return false;
    }
    if (hdr->version != WCNC_PROTOCOL_VERSION) {
        ESP_LOGW(TAG, "Protocol version mismatch: got %d, expected %d",
                 hdr->version, WCNC_PROTOCOL_VERSION);
        return false;
    }

    size_t expected_len = sizeof(wcnc_header_t) + hdr->payload_length;
    if (len < expected_len) {
        ESP_LOGW(TAG, "Packet truncated: %d < %d (payload_length=%d)",
                 (int)len, (int)expected_len, (int)hdr->payload_length);
        return false;
    }

    if (!wcnc_validate_packet(data, expected_len)) {
        ESP_LOGW(TAG, "CRC mismatch for pkt type=0x%02X len=%d", hdr->packet_type, (int)expected_len);
        return false;
    }
    return true;
}

/* ===================================================================
 * UDP Packet Handlers
 * =================================================================== */

static void handle_motion_segments(const uint8_t *data, size_t len)
{
    if (len < sizeof(wcnc_motion_packet_t) - sizeof(wcnc_motion_segment_t) * WCNC_MAX_SEGMENTS_PER_PACKET) {
        return;
    }

    const wcnc_motion_packet_t *pkt = (const wcnc_motion_packet_t *)data;

    if (pkt->segment_count == 0 || pkt->segment_count > WCNC_MAX_SEGMENTS_PER_PACKET) {
        ESP_LOGW(TAG, "Invalid segment count: %d", pkt->segment_count);
        return;
    }

    /* Preempt G-code planner when binary protocol sends motion */
    gcode_planner_preempt();

    for (int i = 0; i < pkt->segment_count; i++) {
        if (!planner_push_segment(&pkt->segments[i])) {
            ESP_LOGW(TAG, "Planner buffer full, dropped segment %d/%d",
                     i + 1, pkt->segment_count);
            break;
        }
    }
}

static void handle_jog_command(const uint8_t *data, size_t len)
{
    if (len < sizeof(wcnc_jog_packet_t)) return;
    const wcnc_jog_packet_t *pkt = (const wcnc_jog_packet_t *)data;

    if (pkt->axis >= WCNC_MAX_AXES) return;
    if (pkt->direction != 1 && pkt->direction != -1) return;

    stepper_start_jog(pkt->axis, pkt->direction, pkt->speed);
}

static void handle_jog_stop(const uint8_t *data, size_t len)
{
    (void)data;
    (void)len;
    stepper_stop_jog();
}

static void handle_estop(void)
{
    motion_estop();
}

static void handle_home_command(const uint8_t *data, size_t len)
{
    if (len < sizeof(wcnc_home_packet_t)) return;
    const wcnc_home_packet_t *pkt = (const wcnc_home_packet_t *)data;
    motion_home(pkt->axis_mask);
}

static void handle_io_control(const uint8_t *data, size_t len)
{
    if (len < sizeof(wcnc_io_control_packet_t)) return;
    const wcnc_io_control_packet_t *pkt = (const wcnc_io_control_packet_t *)data;

    /* Apply misc outputs */
    for (int i = 0; i < 5; i++) {
        gpio_control_set_misc_output(i, (pkt->misc_outputs >> i) & 1);
    }

    /* Apply spindle state */
    gpio_control_set_spindle(pkt->spindle_state != 0);
    s_spindle_state = pkt->spindle_state;

    /* Coolant state (tracked for status reporting) */
    s_coolant_state = pkt->coolant_state;
    /* Future: drive coolant GPIO pins when available */
}

uint8_t protocol_get_spindle_state(void) { return s_spindle_state; }
uint8_t protocol_get_coolant_state(void) { return s_coolant_state; }

void protocol_reset_output_states(void)
{
    s_spindle_state = 0;
    s_coolant_state = 0;
    gpio_control_set_spindle(false);
}

void protocol_handle_udp_packet(const uint8_t *data, size_t len)
{
    if (!protocol_validate_packet(data, len)) {
        return;
    }

    const wcnc_header_t *hdr = (const wcnc_header_t *)data;

    switch (hdr->packet_type) {
    case WCNC_PKT_MOTION_SEGMENT:
        handle_motion_segments(data, len);
        break;
    case WCNC_PKT_JOG_COMMAND:
        handle_jog_command(data, len);
        break;
    case WCNC_PKT_JOG_STOP:
        handle_jog_stop(data, len);
        break;
    case WCNC_PKT_ESTOP:
        handle_estop();
        break;
    case WCNC_PKT_FEED_HOLD:
        motion_feed_hold();
        break;
    case WCNC_PKT_FEED_RESUME:
        motion_feed_resume();
        break;
    case WCNC_PKT_RESET:
        motion_reset();
        gcode_planner_release();
        break;
    case WCNC_PKT_HOME_COMMAND:
        handle_home_command(data, len);
        break;
    case WCNC_PKT_IO_CONTROL:
        handle_io_control(data, len);
        break;
    default:
        ESP_LOGW(TAG, "Unknown UDP packet type: 0x%02X", hdr->packet_type);
        break;
    }
}

/* ===================================================================
 * TCP Packet Handlers
 * =================================================================== */

static size_t handle_handshake_req(const uint8_t *data, size_t len,
                                    uint8_t *resp, size_t resp_size)
{
    (void)data;
    (void)len;

    if (resp_size < sizeof(wcnc_handshake_resp_t)) return 0;

    wcnc_handshake_resp_t *r = (wcnc_handshake_resp_t *)resp;
    memset(r, 0, sizeof(*r));

    r->firmware_version = WCNC_FIRMWARE_VERSION;
    r->num_axes = WCNC_MAX_AXES;
    r->capabilities = gpio_control_get_capabilities();
    r->buffer_capacity = planner_capacity();
    r->max_step_rate = CFG_MAX_AGGREGATE_STEP_RATE;
    strlcpy(r->device_name, "Tiggy", sizeof(r->device_name));

    /* Spindle encoder capability */
    if (spindle_encoder_is_available()) {
        r->capabilities |= WCNC_CAP_SPINDLE_ENCODER;
        uint16_t ppr = nvs_config_get_u16("enc_ppr", 0);
        r->encoder_ppr_hi = (uint8_t)(ppr >> 8);
        r->encoder_ppr_lo = (uint8_t)(ppr & 0xFF);
    }

    /* I/O module capability */
    r->device_mode = nvs_config_get_u8("dev_mode", 0);
    if (r->device_mode == 1) {
        r->capabilities |= WCNC_CAP_IO_MODULE;
        r->io_channel_count = io_module_get_channel_count();
    }

    wcnc_finalize_packet(r, WCNC_PKT_HANDSHAKE_RESP,
                          sizeof(*r) - sizeof(wcnc_header_t),
                          s_tx_sequence++,
                          (uint32_t)(esp_timer_get_time() / 1000));

    return sizeof(*r);
}

static size_t handle_ping(const uint8_t *data, size_t len,
                           uint8_t *resp, size_t resp_size)
{
    if (len < sizeof(wcnc_ping_packet_t)) return 0;
    if (resp_size < sizeof(wcnc_ping_packet_t)) return 0;

    const wcnc_ping_packet_t *ping = (const wcnc_ping_packet_t *)data;
    wcnc_ping_packet_t *pong = (wcnc_ping_packet_t *)resp;

    memset(pong, 0, sizeof(*pong));
    pong->ping_id = ping->ping_id;

    wcnc_finalize_packet(pong, WCNC_PKT_PONG,
                          sizeof(*pong) - sizeof(wcnc_header_t),
                          s_tx_sequence++,
                          (uint32_t)(esp_timer_get_time() / 1000));

    return sizeof(*pong);
}

static size_t handle_config_get(const uint8_t *data, size_t len,
                                 uint8_t *resp, size_t resp_size)
{
    if (len < sizeof(wcnc_config_packet_t)) return 0;
    if (resp_size < sizeof(wcnc_config_packet_t)) return 0;

    const wcnc_config_packet_t *req = (const wcnc_config_packet_t *)data;
    wcnc_config_packet_t *r = (wcnc_config_packet_t *)resp;

    memset(r, 0, sizeof(*r));
    r->key = req->key;

    /* Read value from NVS based on key */
    nvs_config_get_by_protocol_key(req->key, r->value, sizeof(r->value), &r->value_type);

    wcnc_finalize_packet(r, WCNC_PKT_CONFIG_RESP,
                          sizeof(*r) - sizeof(wcnc_header_t),
                          s_tx_sequence++,
                          (uint32_t)(esp_timer_get_time() / 1000));

    return sizeof(*r);
}

static size_t handle_config_set(const uint8_t *data, size_t len,
                                 uint8_t *resp, size_t resp_size)
{
    if (len < sizeof(wcnc_config_packet_t)) return 0;

    const wcnc_config_packet_t *req = (const wcnc_config_packet_t *)data;

    /* Write value to NVS */
    nvs_config_set_by_protocol_key(req->key, req->value, req->value_type);

    /* Apply timing changes immediately to the running stepper */
    if (req->key == WCNC_CFG_STEP_PULSE_US || req->key == WCNC_CFG_DIR_SETUP_US) {
        stepper_apply_timing_config();
    }

    /* Apply per-axis rate/accel changes immediately */
    if ((req->key >= WCNC_CFG_MAX_RATE_X && req->key <= WCNC_CFG_MAX_RATE_C) ||
        (req->key >= WCNC_CFG_ACCEL_X && req->key <= WCNC_CFG_ACCEL_C)) {
        stepper_apply_axis_config();
    }

    /* Apply I/O inversion changes immediately */
    if (req->key >= WCNC_CFG_INVERT_LIMIT && req->key <= WCNC_CFG_INVERT_PROBE) {
        gpio_control_reload_inversion();
    }

    /* Apply charge pump on/off immediately (freq > 0 = on, 0 = off) */
    if (req->key == WCNC_CFG_CHARGE_PUMP_FREQ) {
        uint16_t freq = nvs_config_get_u16("cp_freq", 10000);
        gpio_control_set_charge_pump(freq > 0);
    }

    /* Track if WiFi credentials were changed (requires reboot) */
    if (req->key == WCNC_CFG_WIFI_SSID || req->key == WCNC_CFG_WIFI_PASSWORD) {
        s_wifi_config_changed = true;
    }

    /* Pin assignments are boot-time only — log reminder */
    if (req->key >= WCNC_CFG_PIN_STEP_X && req->key <= WCNC_CFG_PIN_MISC_OUT1) {
        ESP_LOGI(TAG, "Pin config 0x%04X updated — reboot required to take effect", req->key);
    }

    /* Send ACK response */
    if (resp_size < sizeof(wcnc_ack_packet_t)) return 0;

    wcnc_ack_packet_t *ack = (wcnc_ack_packet_t *)resp;
    memset(ack, 0, sizeof(*ack));

    const wcnc_header_t *hdr = (const wcnc_header_t *)data;
    ack->acked_sequence = hdr->sequence;
    ack->acked_type = hdr->packet_type;

    wcnc_finalize_packet(ack, WCNC_PKT_ACK,
                          sizeof(*ack) - sizeof(wcnc_header_t),
                          s_tx_sequence++,
                          (uint32_t)(esp_timer_get_time() / 1000));

    return sizeof(*ack);
}

static size_t handle_config_save(uint8_t *resp, size_t resp_size)
{
    nvs_config_commit();

    if (resp_size < sizeof(wcnc_ack_packet_t)) return 0;

    wcnc_ack_packet_t *ack = (wcnc_ack_packet_t *)resp;
    memset(ack, 0, sizeof(*ack));
    ack->acked_type = WCNC_PKT_CONFIG_SAVE;

    wcnc_finalize_packet(ack, WCNC_PKT_ACK,
                          sizeof(*ack) - sizeof(wcnc_header_t),
                          s_tx_sequence++,
                          (uint32_t)(esp_timer_get_time() / 1000));

    /* If WiFi credentials changed, schedule a delayed restart so the
       TCP ACK has time to flush before we reboot */
    if (s_wifi_config_changed) {
        s_wifi_config_changed = false;
        ESP_LOGI(TAG, "WiFi config changed, restarting in 1 second...");

        const esp_timer_create_args_t timer_args = {
            .callback = restart_timer_callback,
            .name = "restart"
        };
        if (s_restart_timer == NULL) {
            if (esp_timer_create(&timer_args, &s_restart_timer) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to create restart timer, rebooting now");
                esp_restart();
            }
        }
        esp_timer_start_once(s_restart_timer, 1000000); /* 1 second in us */
    }

    return sizeof(*ack);
}

size_t protocol_handle_tcp_packet(const uint8_t *data, size_t len,
                                   uint8_t *response_buf, size_t response_buf_size)
{
    if (!protocol_validate_packet(data, len)) {
        ESP_LOGW(TAG, "TCP packet validation failed (len=%d)", (int)len);
        return 0;
    }

    const wcnc_header_t *hdr = (const wcnc_header_t *)data;

    switch (hdr->packet_type) {
    case WCNC_PKT_HANDSHAKE_REQ:
        return handle_handshake_req(data, len, response_buf, response_buf_size);
    case WCNC_PKT_PING:
        return handle_ping(data, len, response_buf, response_buf_size);
    case WCNC_PKT_CONFIG_GET:
        return handle_config_get(data, len, response_buf, response_buf_size);
    case WCNC_PKT_CONFIG_SET:
        return handle_config_set(data, len, response_buf, response_buf_size);
    case WCNC_PKT_CONFIG_SAVE:
        return handle_config_save(response_buf, response_buf_size);
    default:
        ESP_LOGW(TAG, "Unknown TCP packet type: 0x%02X", hdr->packet_type);
        return 0;
    }
}
