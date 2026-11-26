/*
 * WiFi CNC Controller - TCP Server
 *
 * The control channel: handshake, configuration, keep-alive.  One host at a
 * time owns the controller.
 *
 * Framing: each message is a 2-byte little-endian length followed by a
 * packet (which starts with the standard wcnc_header_t).
 *
 * A second connection is looked at while the first is being served:
 *   - while the host that owns the controller is still talking to it, the
 *     newcomer is turned away at once (it cannot disturb a running machine,
 *     and it is not left hanging either);
 *   - if that host has gone quiet - crashed, unplugged, out of range - the
 *     newcomer takes over, so the controller can never be locked out by a
 *     connection that died without saying so.
 */

#include "tcp_server.h"
#include "udp_server.h"
#include "protocol_handler.h"
#include "../motion/motion_control.h"
#include "../config.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "esp_log.h"

static const char *TAG = "tcp_srv";

/* The host counts as present while either channel has carried data within: */
#define HOST_QUIET_MS   3000

static volatile bool s_client_connected = false;

bool tcp_client_connected(void)
{
    return s_client_connected;
}

static int make_listener(void)
{
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) return -1;
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port = htons(WCNC_TCP_CONTROL_PORT),
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0 || listen(sock, 2) < 0) {
        close(sock);
        return -1;
    }
    return sock;
}

static bool send_all(int sock, const uint8_t *data, size_t len)
{
    while (len > 0) {
        int n = send(sock, data, len, 0);
        if (n <= 0) return false;
        data += n;
        len -= (size_t)n;
    }
    return true;
}

void tcp_server_task(void *pvParameters)
{
    (void)pvParameters;
    static uint8_t rx[CFG_TCP_RX_BUFFER_SIZE + 2];
    static uint8_t tx[512];

    while (!network_is_up()) vTaskDelay(pdMS_TO_TICKS(100));

    int listener;
    while ((listener = make_listener()) < 0) {
        ESP_LOGE(TAG, "Cannot listen on TCP port %d: errno %d", WCNC_TCP_CONTROL_PORT, errno);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "TCP server listening on port %d (core %d)", WCNC_TCP_CONTROL_PORT, xPortGetCoreID());

    int client = -1;
    size_t have = 0;                        /* bytes of an incomplete frame in rx */
    TickType_t last_data = 0;

    for (;;) {
        fd_set readable;
        FD_ZERO(&readable);
        FD_SET(listener, &readable);
        int top = listener;
        if (client >= 0) {
            FD_SET(client, &readable);
            if (client > top) top = client;
        }
        struct timeval wait = { .tv_sec = 1, .tv_usec = 0 };
        int ready = select(top + 1, &readable, NULL, NULL, &wait);
        if (ready < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        bool drop = false;

        /* --- someone new at the door --- */
        if (ready > 0 && FD_ISSET(listener, &readable)) {
            struct sockaddr_in from;
            socklen_t from_len = sizeof(from);
            int incoming = accept(listener, (struct sockaddr *)&from, &from_len);
            if (incoming >= 0) {
                bool tcp_quiet = (xTaskGetTickCount() - last_data) * portTICK_PERIOD_MS > HOST_QUIET_MS;
                if (client >= 0 && (!tcp_quiet || motion_host_active())) {
                    ESP_LOGW(TAG, "Connection refused: the controller is in use by another host");
                    close(incoming);
                } else {
                    if (client >= 0) {
                        ESP_LOGW(TAG, "The previous host has gone quiet: a new one takes over");
                        close(client);
                        motion_tcp_session(0);
                    }
                    client = incoming;
                    have = 0;
                    last_data = xTaskGetTickCount();
                    int one = 1;
                    setsockopt(client, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
                    struct timeval send_wait = { .tv_sec = 2, .tv_usec = 0 };
                    setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &send_wait, sizeof(send_wait));
                    s_client_connected = true;
                    motion_tcp_session(from.sin_addr.s_addr);
                    ESP_LOGI(TAG, "Host connected");
                }
            }
        }

        /* --- data from the host --- */
        if (client >= 0 && ready > 0 && FD_ISSET(client, &readable)) {
            int n = recv(client, rx + have, sizeof(rx) - have, MSG_DONTWAIT);
            if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
                drop = true;
            } else if (n > 0) {
                have += (size_t)n;
                last_data = xTaskGetTickCount();
                while (have >= 2 && !drop) {
                    size_t frame = (size_t)rx[0] | ((size_t)rx[1] << 8);
                    if (frame == 0 || frame > CFG_TCP_RX_BUFFER_SIZE) {
                        ESP_LOGW(TAG, "Bad frame length %u", (unsigned)frame);
                        drop = true;
                        break;
                    }
                    if (have < 2 + frame) break;
                    size_t reply = protocol_handle_tcp_packet(rx + 2, frame, tx + 2, sizeof(tx) - 2);
                    if (reply > 0) {
                        tx[0] = (uint8_t)(reply & 0xFF);
                        tx[1] = (uint8_t)(reply >> 8);
                        if (!send_all(client, tx, reply + 2)) drop = true;
                    }
                    have -= 2 + frame;
                    memmove(rx, rx + 2 + frame, have);
                }
            }
        }

        /* --- a host that has stopped talking on both channels --- */
        if (client >= 0 && !drop &&
            (xTaskGetTickCount() - last_data) * portTICK_PERIOD_MS > CFG_TCP_KEEPALIVE_TIMEOUT_MS &&
            !motion_host_active()) {
            ESP_LOGW(TAG, "Host silent for %d s", CFG_TCP_KEEPALIVE_TIMEOUT_MS / 1000);
            drop = true;
        }

        if (drop && client >= 0) {
            close(client);
            client = -1;
            have = 0;
            s_client_connected = false;
            motion_tcp_session(0);
            ESP_LOGI(TAG, "Host disconnected");
        }
    }
}
