/*
 * WiFi CNC Controller - UDP Server
 *
 * Listens on WCNC_UDP_MOTION_PORT for incoming motion packets.
 * Tracks the remote host address so the status reporter can send
 * status packets back.
 */

#include "udp_server.h"
#include "protocol_handler.h"
#include "wifi_manager.h"
#include "../config.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "esp_log.h"

static const char *TAG = "udp_srv";

static struct sockaddr_in s_host_addr;
static bool s_host_known = false;
static int s_status_sock = -1;

/* ===================================================================
 * Host Tracking
 * =================================================================== */

bool udp_has_host(void)
{
    return s_host_known;
}

bool udp_get_host_addr(struct sockaddr_in *addr)
{
    if (!s_host_known) return false;
    memcpy(addr, &s_host_addr, sizeof(*addr));
    return true;
}

/* ===================================================================
 * Status Send Socket
 * =================================================================== */

bool udp_send_status(const uint8_t *data, size_t len)
{
    if (!s_host_known || s_status_sock < 0) return false;

    struct sockaddr_in dest = s_host_addr;
    dest.sin_port = htons(WCNC_UDP_STATUS_PORT);

    int sent = sendto(s_status_sock, data, len, 0,
                       (struct sockaddr *)&dest, sizeof(dest));
    return (sent == (int)len);
}

/* ===================================================================
 * UDP Receive Task
 * =================================================================== */

void udp_receive_task(void *pvParameters)
{
    (void)pvParameters;
    uint8_t rx_buffer[CFG_UDP_RX_BUFFER_SIZE];

    ESP_LOGI(TAG, "UDP receive task started on Core %d", xPortGetCoreID());

    /* Wait for WiFi to be ready */
    while (!wifi_is_connected() && !wifi_is_ap_mode()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    /* Create receive socket */
    int rx_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (rx_sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP receive socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port = htons(WCNC_UDP_MOTION_PORT),
    };

    if (bind(rx_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind UDP socket to port %d: errno %d",
                 WCNC_UDP_MOTION_PORT, errno);
        close(rx_sock);
        vTaskDelete(NULL);
        return;
    }

    /* Create status send socket */
    s_status_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_status_sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP status socket: errno %d", errno);
    }

    ESP_LOGI(TAG, "UDP listening on port %d", WCNC_UDP_MOTION_PORT);

    /* Receive loop */
    while (1) {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);

        int len = recvfrom(rx_sock, rx_buffer, sizeof(rx_buffer), 0,
                            (struct sockaddr *)&source_addr, &socklen);

        if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            ESP_LOGE(TAG, "recvfrom error: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (len < (int)sizeof(wcnc_header_t)) continue;

        /* Update host address */
        memcpy(&s_host_addr, &source_addr, sizeof(s_host_addr));
        s_host_known = true;

        /* Dispatch packet */
        protocol_handle_udp_packet(rx_buffer, (size_t)len);
    }
}
