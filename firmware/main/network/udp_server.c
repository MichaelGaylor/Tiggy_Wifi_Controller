/*
 * WiFi CNC Controller - UDP Server
 *
 * See udp_server.h.  Which host the controller listens to is decided by the
 * motion core; this file only moves datagrams.
 */

#include "udp_server.h"
#include "protocol_handler.h"
#include "wifi_manager.h"
#include "eth_manager.h"
#include "../motion/motion_control.h"
#include "../config.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "esp_log.h"

static const char *TAG = "udp_srv";

static int s_sock = -1;

bool network_is_up(void)
{
    return wifi_is_connected() || wifi_is_ap_mode() || eth_is_connected();
}

bool udp_send_to(uint32_t ip, uint16_t port, const void *data, size_t len)
{
    if (s_sock < 0 || ip == 0) return false;
    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
        .sin_addr.s_addr = ip,
    };
    return sendto(s_sock, data, len, 0, (struct sockaddr *)&dest, sizeof(dest)) == (int)len;
}

void udp_receive_task(void *pvParameters)
{
    (void)pvParameters;
    static uint8_t rx_buffer[CFG_UDP_RX_BUFFER_SIZE];

    while (!network_is_up()) vTaskDelay(pdMS_TO_TICKS(100));

    int sock = -1;
    for (;;) {
        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock >= 0) {
            struct sockaddr_in bind_addr = {
                .sin_family = AF_INET,
                .sin_addr.s_addr = htonl(INADDR_ANY),
                .sin_port = htons(WCNC_UDP_MOTION_PORT),
            };
            if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) == 0) break;
            ESP_LOGE(TAG, "Cannot bind UDP port %d: errno %d", WCNC_UDP_MOTION_PORT, errno);
            close(sock);
        } else {
            ESP_LOGE(TAG, "Cannot create UDP socket: errno %d", errno);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));        /* try again rather than leave the controller deaf */
    }
    s_sock = sock;
    ESP_LOGI(TAG, "UDP listening on port %d (core %d)", WCNC_UDP_MOTION_PORT, xPortGetCoreID());

    for (;;) {
        struct sockaddr_in source;
        socklen_t socklen = sizeof(source);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source, &socklen);
        if (len < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                ESP_LOGE(TAG, "recvfrom error: errno %d", errno);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            continue;
        }
        /* Only well-formed packets go any further */
        if (!protocol_validate_packet(rx_buffer, (size_t)len)) continue;
        motion_post_udp(rx_buffer, (size_t)len, source.sin_addr.s_addr, ntohs(source.sin_port));
    }
}
