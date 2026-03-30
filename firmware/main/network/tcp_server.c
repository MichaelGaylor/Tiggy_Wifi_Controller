/*
 * WiFi CNC Controller - TCP Server
 *
 * Single-client TCP server for configuration and control commands.
 * Accepts one connection at a time on WCNC_TCP_CONTROL_PORT.
 *
 * Protocol: length-prefixed packets. Each message starts with a 2-byte
 * little-endian length prefix followed by the packet data (which includes
 * the standard wcnc_header_t).
 */

#include "tcp_server.h"
#include "protocol_handler.h"
#include "wifi_manager.h"
#include "../io/gpio_control.h"
#include "../motion/motion_control.h"
#include "../config.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "esp_log.h"

static const char *TAG = "tcp_srv";

static volatile bool s_client_connected = false;

bool tcp_client_connected(void)
{
    return s_client_connected;
}

/* ===================================================================
 * Client Handler
 * =================================================================== */

static void handle_client(int client_sock)
{
    uint8_t rx_buffer[CFG_TCP_RX_BUFFER_SIZE];
    uint8_t tx_buffer[1024];  /* Responses can be larger than rx (e.g. config dump) */

    s_client_connected = true;
    ESP_LOGI(TAG, "TCP client connected");

    /*
     * Use a SHORT recv timeout (2s) so the task wakes frequently and
     * doesn't starve the task watchdog (5s).  We track elapsed time
     * since the last received data manually and only disconnect when
     * that exceeds CFG_TCP_KEEPALIVE_TIMEOUT_MS.
     */
    struct timeval timeout = {
        .tv_sec = CFG_TCP_RECV_TIMEOUT_MS / 1000,
        .tv_usec = (CFG_TCP_RECV_TIMEOUT_MS % 1000) * 1000,
    };
    setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    TickType_t last_data_tick = xTaskGetTickCount();

    while (1) {
        /* Read 2-byte length prefix */
        uint8_t len_buf[2];
        int received = 0;
        while (received < 2) {
            int n = recv(client_sock, len_buf + received, 2 - received, 0);
            if (n > 0) {
                received += n;
                last_data_tick = xTaskGetTickCount();
                continue;
            }

            if (n == 0) {
                ESP_LOGI(TAG, "TCP client disconnected");
                goto disconnect;
            }

            /* n < 0: check if it's just a timeout */
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                /* recv() timed out after CFG_TCP_RECV_TIMEOUT_MS.
                 * Check if we've exceeded the keepalive threshold. */
                uint32_t elapsed_ms = (xTaskGetTickCount() - last_data_tick)
                                      * portTICK_PERIOD_MS;
                if (elapsed_ms >= CFG_TCP_KEEPALIVE_TIMEOUT_MS) {
                    ESP_LOGW(TAG, "TCP keepalive timeout (%lu ms)", (unsigned long)elapsed_ms);
                    goto disconnect;
                }
                /* Still within window — loop and recv() again */
                continue;
            }

            /* Real error */
            ESP_LOGE(TAG, "TCP recv error: errno %d", errno);
            goto disconnect;
        }

        uint16_t pkt_len = (uint16_t)len_buf[0] | ((uint16_t)len_buf[1] << 8);

        if (pkt_len == 0 || pkt_len > sizeof(rx_buffer)) {
            ESP_LOGW(TAG, "Invalid packet length: %u", pkt_len);
            goto disconnect;
        }

        /* Read the full packet */
        received = 0;
        while (received < (int)pkt_len) {
            int n = recv(client_sock, rx_buffer + received, pkt_len - received, 0);
            if (n <= 0) {
                if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                    /* Timeout mid-packet — check keepalive */
                    uint32_t elapsed_ms = (xTaskGetTickCount() - last_data_tick)
                                          * portTICK_PERIOD_MS;
                    if (elapsed_ms >= CFG_TCP_KEEPALIVE_TIMEOUT_MS) {
                        ESP_LOGW(TAG, "TCP timeout mid-packet");
                        goto disconnect;
                    }
                    continue;  /* Retry recv for remaining bytes */
                }
                ESP_LOGE(TAG, "TCP recv error during packet: errno %d", errno);
                goto disconnect;
            }
            received += n;
            last_data_tick = xTaskGetTickCount();
        }

        /* Handle packet and get response */
        size_t resp_len = protocol_handle_tcp_packet(
            rx_buffer, pkt_len, tx_buffer, sizeof(tx_buffer));

        if (resp_len == 0) {
            ESP_LOGW(TAG, "TCP packet produced no response (pkt_len=%d, type=0x%02X)",
                     pkt_len, pkt_len >= 6 ? rx_buffer[5] : 0xFF);
        }

        /* Send response if any */
        if (resp_len > 0) {
            /* Send length prefix */
            uint8_t resp_len_buf[2] = {
                (uint8_t)(resp_len & 0xFF),
                (uint8_t)((resp_len >> 8) & 0xFF),
            };
            if (send(client_sock, resp_len_buf, 2, 0) < 0 ||
                send(client_sock, tx_buffer, resp_len, 0) < 0) {
                ESP_LOGE(TAG, "TCP send error: errno %d", errno);
                goto disconnect;
            }
        }
    }

disconnect:
    s_client_connected = false;
    close(client_sock);

    /* Safety: disable outputs when software disconnects.
     * No software = no heartbeat = everything should stop. */
    gpio_control_set_charge_pump(false);
    protocol_reset_output_states();
    ESP_LOGW(TAG, "Outputs disabled (client disconnected)");

    ESP_LOGI(TAG, "TCP client session ended");
}

/* ===================================================================
 * TCP Server Task
 * =================================================================== */

void tcp_server_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "TCP server task started on Core %d", xPortGetCoreID());

    /* Wait for WiFi */
    while (!wifi_is_connected() && !wifi_is_ap_mode()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    /* Create server socket */
    int server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_sock < 0) {
        ESP_LOGE(TAG, "Failed to create TCP socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port = htons(WCNC_TCP_CONTROL_PORT),
    };

    if (bind(server_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind TCP socket to port %d: errno %d",
                 WCNC_TCP_CONTROL_PORT, errno);
        close(server_sock);
        vTaskDelete(NULL);
        return;
    }

    if (listen(server_sock, 1) < 0) {
        ESP_LOGE(TAG, "TCP listen failed: errno %d", errno);
        close(server_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "TCP server listening on port %d", WCNC_TCP_CONTROL_PORT);

    /* Accept loop: one client at a time */
    while (1) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);

        int client_sock = accept(server_sock,
                                  (struct sockaddr *)&client_addr, &addr_len);
        if (client_sock < 0) {
            ESP_LOGE(TAG, "TCP accept failed: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        char addr_str[16];
        inet_ntoa_r(client_addr.sin_addr, addr_str, sizeof(addr_str));
        ESP_LOGI(TAG, "TCP client from %s:%d", addr_str, ntohs(client_addr.sin_port));

        handle_client(client_sock);
    }
}
