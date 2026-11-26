/*
 * WiFi CNC Controller - Protocol Handler
 *
 * Packet validation for both channels, and the TCP control packets:
 * handshake, keep-alive and configuration.  Datagrams (motion, jog, home,
 * outputs, E-stop ...) are handled by the motion core.
 */

#include "protocol_handler.h"
#include "../motion/motion_control.h"
#include "../motion/motion_core.h"
#include "../motion/step_engine.h"
#include "../persist/nvs_config.h"
#include "../io/spindle_encoder.h"
#include "../io/io_module.h"
#include "../config.h"

#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

static const char *TAG = "protocol";

static uint32_t s_tx_sequence = 0;
static bool s_restart_needed = false;
static esp_timer_handle_t s_restart_timer = NULL;

static void restart_timer_callback(void *arg)
{
    (void)arg;
    esp_restart();
}

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* ===================================================================
 * Packet Validation
 * =================================================================== */

bool protocol_validate_packet(const uint8_t *data, size_t len)
{
    if (len < sizeof(wcnc_header_t)) return false;
    const wcnc_header_t *hdr = (const wcnc_header_t *)data;
    if (hdr->magic != WCNC_MAGIC || hdr->version != WCNC_PROTOCOL_VERSION) return false;
    size_t expected = sizeof(wcnc_header_t) + hdr->payload_length;
    if (len < expected) return false;
    return wcnc_validate_packet(data, expected) != 0;
}

/* ===================================================================
 * TCP Packet Handlers
 * =================================================================== */

static size_t handle_handshake_req(uint8_t *resp, size_t resp_size)
{
    if (resp_size < sizeof(wcnc_handshake_resp_t)) return 0;

    wcnc_handshake_resp_t *r = (wcnc_handshake_resp_t *)resp;
    memset(r, 0, sizeof(*r));

    r->firmware_version = WCNC_FIRMWARE_VERSION;
    r->num_axes = WCNC_MAX_AXES;
    r->capabilities = motion_capabilities();
    r->buffer_capacity = SE_QUEUE_SIZE - 1;
    r->max_step_rate = mc_max_step_rate();      /* fastest any one axis can be stepped */

    r->device_mode = nvs_config_get_u8("dev_mode", 0);
    strlcpy(r->device_name, r->device_mode == 1 ? "Tiggy I/O Module" : "Tiggy", sizeof(r->device_name));
    if (r->device_mode == 1) r->io_channel_count = io_module_get_channel_count();

    if (spindle_encoder_is_available()) {
        uint16_t ppr = nvs_config_get_u16("enc_ppr", 0);
        r->encoder_ppr_hi = (uint8_t)(ppr >> 8);
        r->encoder_ppr_lo = (uint8_t)(ppr & 0xFF);
    }

    wcnc_finalize_packet(r, WCNC_PKT_HANDSHAKE_RESP, sizeof(*r) - sizeof(wcnc_header_t),
                         s_tx_sequence++, now_ms());
    return sizeof(*r);
}

static size_t handle_ping(const uint8_t *data, size_t len, uint8_t *resp, size_t resp_size)
{
    if (len < sizeof(wcnc_ping_packet_t) || resp_size < sizeof(wcnc_ping_packet_t)) return 0;

    const wcnc_ping_packet_t *ping = (const wcnc_ping_packet_t *)data;
    wcnc_ping_packet_t *pong = (wcnc_ping_packet_t *)resp;
    memset(pong, 0, sizeof(*pong));
    pong->ping_id = ping->ping_id;
    wcnc_finalize_packet(pong, WCNC_PKT_PONG, sizeof(*pong) - sizeof(wcnc_header_t),
                         s_tx_sequence++, now_ms());
    return sizeof(*pong);
}

static size_t handle_config_get(const uint8_t *data, size_t len, uint8_t *resp, size_t resp_size)
{
    if (len < sizeof(wcnc_config_packet_t) || resp_size < sizeof(wcnc_config_packet_t)) return 0;

    const wcnc_config_packet_t *req = (const wcnc_config_packet_t *)data;
    wcnc_config_packet_t *r = (wcnc_config_packet_t *)resp;
    memset(r, 0, sizeof(*r));
    r->key = req->key;
    if (req->key == WCNC_CFG_WIFI_PASSWORD) {
        /* Never handed out again once stored */
        r->value_type = WCNC_VAL_STRING;
    } else {
        uint16_t type = WCNC_VAL_UINT32;
        nvs_config_get_by_protocol_key(req->key, r->value, sizeof(r->value), &type);
        r->value_type = type;
    }

    wcnc_finalize_packet(r, WCNC_PKT_CONFIG_RESP, sizeof(*r) - sizeof(wcnc_header_t),
                         s_tx_sequence++, now_ms());
    return sizeof(*r);
}

static size_t make_ack(const uint8_t *data, uint8_t *resp, size_t resp_size)
{
    if (resp_size < sizeof(wcnc_ack_packet_t)) return 0;
    const wcnc_header_t *hdr = (const wcnc_header_t *)data;
    wcnc_ack_packet_t *ack = (wcnc_ack_packet_t *)resp;
    memset(ack, 0, sizeof(*ack));
    ack->acked_sequence = hdr->sequence;
    ack->acked_type = hdr->packet_type;
    wcnc_finalize_packet(ack, WCNC_PKT_ACK, sizeof(*ack) - sizeof(wcnc_header_t),
                         s_tx_sequence++, now_ms());
    return sizeof(*ack);
}

static size_t handle_config_set(const uint8_t *data, size_t len, uint8_t *resp, size_t resp_size)
{
    if (len < sizeof(wcnc_config_packet_t)) return 0;
    const wcnc_config_packet_t *req = (const wcnc_config_packet_t *)data;
    uint16_t key = req->key;

    /* Network, pins, device mode, encoder and I/O module layout are read
     * once at start-up.  When one of them really changes, the next "save"
     * restarts the device (unless the machine is doing something). */
    bool at_startup = (key >= WCNC_CFG_WIFI_SSID && key <= WCNC_CFG_STATIC_NETMASK) ||
                      (key >= WCNC_CFG_PIN_STEP_X && key <= WCNC_CFG_PIN_MISC_IN3) ||
                      (key >= WCNC_CFG_DEVICE_MODE && key <= WCNC_CFG_PIN_ETH_SPI_HOST);
    uint8_t before[WCNC_CONFIG_VALUE_LEN], after[WCNC_CONFIG_VALUE_LEN];
    uint16_t type;
    if (at_startup) nvs_config_get_by_protocol_key(key, before, sizeof(before), &type);

    nvs_config_set_by_protocol_key(key, req->value, req->value_type);

    if (at_startup) {
        nvs_config_get_by_protocol_key(key, after, sizeof(after), &type);
        if (memcmp(before, after, sizeof(before)) != 0) {
            s_restart_needed = true;
            ESP_LOGI(TAG, "Config 0x%04X takes effect after a restart", key);
        }
    } else {
        /* Everything the motion side works with (rates, timing, polarities,
         * homing, spindle and charge pump set-up) is applied at once. */
        motion_config_changed();
    }
    return make_ack(data, resp, resp_size);

}

static size_t handle_config_save(const uint8_t *data, uint8_t *resp, size_t resp_size)
{
    nvs_config_commit();
    size_t n = make_ack(data, resp, resp_size);

    if (s_restart_needed && !mc_busy()) {
        s_restart_needed = false;
        ESP_LOGW(TAG, "Restarting in 1 second to apply the new configuration");
        /* Delayed so that the acknowledgement gets out first */
        const esp_timer_create_args_t args = { .callback = restart_timer_callback, .name = "restart" };
        if (s_restart_timer == NULL && esp_timer_create(&args, &s_restart_timer) != ESP_OK) esp_restart();
        esp_timer_start_once(s_restart_timer, 1000000);
    }
    return n;
}

size_t protocol_handle_tcp_packet(const uint8_t *data, size_t len,
                                   uint8_t *response_buf, size_t response_buf_size)
{
    if (!protocol_validate_packet(data, len)) {
        ESP_LOGW(TAG, "TCP packet failed validation (len=%d)", (int)len);
        return 0;
    }
    const wcnc_header_t *hdr = (const wcnc_header_t *)data;

    switch (hdr->packet_type) {
    case WCNC_PKT_HANDSHAKE_REQ:
        return handle_handshake_req(response_buf, response_buf_size);
    case WCNC_PKT_PING:
        return handle_ping(data, len, response_buf, response_buf_size);
    case WCNC_PKT_CONFIG_GET:
        return handle_config_get(data, len, response_buf, response_buf_size);
    case WCNC_PKT_CONFIG_SET:
        return handle_config_set(data, len, response_buf, response_buf_size);
    case WCNC_PKT_CONFIG_SAVE:
        return handle_config_save(data, response_buf, response_buf_size);
    default:
        ESP_LOGW(TAG, "Unknown TCP packet type: 0x%02X", hdr->packet_type);
        return 0;
    }
}
