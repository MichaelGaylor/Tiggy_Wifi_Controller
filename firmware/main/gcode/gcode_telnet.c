/*
 * WiFi CNC Controller - G-Code Telnet Server
 *
 * Single-client TCP server on port 23. Accepts lines of G-code and
 * feeds them to gcode_interface for execution.
 */

#include "gcode_telnet.h"
#include "gcode_interface.h"
#include "gcode_parser.h"
#include "../network/wifi_manager.h"

#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "gcode_tel";

static volatile bool s_connected;
static int s_client_sock = -1;

/* ===================================================================
 * Output Callback (sends to telnet client)
 * =================================================================== */

static void telnet_output(const char *str, size_t len, void *ctx)
{
    int sock = (int)(intptr_t)ctx;
    if (sock >= 0) {
        send(sock, str, len, 0);
    }
}

/* ===================================================================
 * Client Handler
 * =================================================================== */

static void handle_client(int sock)
{
    char line_buf[GCODE_MAX_LINE_LENGTH];
    int line_pos = 0;
    uint8_t rx_buf[128];

    s_client_sock = sock;
    s_connected = true;

    /* Set output to this socket */
    gcode_interface_set_output(telnet_output, (void *)(intptr_t)sock);

    /* Send welcome */
    const char *welcome = "\r\nGrbl 1.1h ['$' for help]\r\n";
    send(sock, welcome, strlen(welcome), 0);

    /* Set socket timeout for periodic checks */
    struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 }; /* 100ms */
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    while (1) {
        int n = recv(sock, rx_buf, sizeof(rx_buf), 0);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;  /* Timeout — check for disconnect */
            }
            break;  /* Error */
        }
        if (n == 0) break;  /* Client disconnected */

        /* Process each byte */
        for (int i = 0; i < n; i++) {
            uint8_t byte = rx_buf[i];

            /* Check for real-time commands (handled immediately) */
            if (byte == GRBL_RT_STATUS_QUERY ||
                byte == GRBL_RT_FEED_HOLD ||
                byte == GRBL_RT_CYCLE_START ||
                byte == GRBL_RT_SOFT_RESET) {
                gcode_handle_realtime(byte);
                continue;
            }

            /* Skip telnet IAC sequences (0xFF ...) */
            if (byte == 0xFF) {
                /* Skip next 2 bytes of telnet command */
                i += 2;
                continue;
            }

            /* Line accumulation */
            if (byte == '\n' || byte == '\r') {
                if (line_pos > 0) {
                    line_buf[line_pos] = '\0';
                    gcode_interface_submit_line(line_buf);
                    line_pos = 0;
                }
            } else if (byte == 0x08 || byte == 0x7F) {
                /* Backspace / delete */
                if (line_pos > 0) line_pos--;
            } else if (line_pos < GCODE_MAX_LINE_LENGTH - 1) {
                line_buf[line_pos++] = (char)byte;
            }
        }
    }

    /* Cleanup */
    gcode_interface_set_output(NULL, NULL);
    s_connected = false;
    s_client_sock = -1;
    close(sock);

    ESP_LOGI(TAG, "Client disconnected");
}

/* ===================================================================
 * Server Task
 * =================================================================== */

void gcode_telnet_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Telnet server starting on port %d", GCODE_TELNET_PORT);

    /* Wait for WiFi */
    while (!wifi_is_connected() && !wifi_is_ap_mode()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    int server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(GCODE_TELNET_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(server_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "Bind failed: %d", errno);
        close(server_sock);
        vTaskDelete(NULL);
        return;
    }

    if (listen(server_sock, 1) < 0) {
        ESP_LOGE(TAG, "Listen failed: %d", errno);
        close(server_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Listening on port %d", GCODE_TELNET_PORT);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_sock = accept(server_sock, (struct sockaddr *)&client_addr,
                                  &client_len);
        if (client_sock < 0) {
            ESP_LOGE(TAG, "Accept failed: %d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "Client connected");
        handle_client(client_sock);
    }
}

bool gcode_telnet_connected(void)
{
    return s_connected;
}
