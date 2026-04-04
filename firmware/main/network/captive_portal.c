/*
 * Captive Portal - WiFi Setup via Web Browser
 *
 * Runs a lightweight HTTP server on port 80 and a DNS server on port 53
 * that redirects all lookups to 192.168.4.1. This triggers the captive
 * portal popup on phones and PCs when they connect to the AP.
 *
 * Serves a single page with SSID and password fields. On submit,
 * saves credentials to NVS and reboots into STA mode.
 */

#include "captive_portal.h"
#include "../persist/nvs_config.h"
#include "../config.h"

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

static const char *TAG = "portal";

static httpd_handle_t s_httpd = NULL;
static TaskHandle_t s_dns_task = NULL;
static bool s_dns_running = false;

/* ===================================================================
 * HTML Page
 * =================================================================== */

static const char PORTAL_HTML[] =
    "<!DOCTYPE html>"
    "<html><head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>TiggyCNC WiFi Setup</title>"
    "<style>"
    "*{box-sizing:border-box;margin:0;padding:0}"
    "body{font-family:-apple-system,sans-serif;background:#1a1a2e;color:#e0e0e0;"
    "display:flex;justify-content:center;align-items:center;min-height:100vh}"
    ".card{background:#16213e;border-radius:12px;padding:32px;max-width:400px;"
    "width:90%%;border:1px solid #0f3460}"
    "h1{color:#e94560;font-size:22px;margin-bottom:8px}"
    "p{color:#a0a0a0;font-size:13px;margin-bottom:20px}"
    "label{display:block;font-size:14px;margin-bottom:4px;color:#c0c0c0}"
    "input[type=text],input[type=password]{width:100%%;padding:10px;border:1px solid #0f3460;"
    "border-radius:6px;background:#1a1a2e;color:#fff;font-size:16px;margin-bottom:16px}"
    "input:focus{outline:none;border-color:#e94560}"
    "button{width:100%%;padding:12px;background:#e94560;color:#fff;border:none;"
    "border-radius:6px;font-size:16px;font-weight:bold;cursor:pointer}"
    "button:hover{background:#d63851}"
    ".ok{background:#16213e;border:1px solid #53a8b6;border-radius:8px;padding:20px;"
    "text-align:center}"
    ".ok h1{color:#53a8b6}"
    "</style></head><body>"
    "<div class='card'>"
    "<h1>TiggyCNC WiFi Setup</h1>"
    "<p>Enter your WiFi network credentials. The controller will reboot and connect to your network.</p>"
    "<form method='POST' action='/save'>"
    "<label>WiFi Network Name (SSID)</label>"
    "<input type='text' name='ssid' maxlength='31' required autofocus>"
    "<label>Password</label>"
    "<input type='password' name='pass' maxlength='63'>"
    "<button type='submit'>Save &amp; Reboot</button>"
    "</form>"
    "</div></body></html>";

static const char PORTAL_OK_HTML[] =
    "<!DOCTYPE html>"
    "<html><head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>TiggyCNC - Saved</title>"
    "<style>"
    "*{box-sizing:border-box;margin:0;padding:0}"
    "body{font-family:-apple-system,sans-serif;background:#1a1a2e;color:#e0e0e0;"
    "display:flex;justify-content:center;align-items:center;min-height:100vh}"
    ".ok{background:#16213e;border:1px solid #53a8b6;border-radius:12px;padding:32px;"
    "max-width:400px;width:90%%;text-align:center}"
    "h1{color:#53a8b6;font-size:22px;margin-bottom:12px}"
    "p{color:#a0a0a0;font-size:14px}"
    "</style></head><body>"
    "<div class='ok'>"
    "<h1>WiFi Saved</h1>"
    "<p>Rebooting now. Connect to your WiFi network and use the Protocol Tester to find the controller.</p>"
    "</div></body></html>";

/* ===================================================================
 * URL Decode Helper
 * =================================================================== */

static void url_decode(char *dst, const char *src, size_t dst_size)
{
    size_t di = 0;
    while (*src && di < dst_size - 1) {
        if (*src == '+') {
            dst[di++] = ' ';
            src++;
        } else if (*src == '%' && src[1] && src[2]) {
            char hex[3] = { src[1], src[2], 0 };
            dst[di++] = (char)strtol(hex, NULL, 16);
            src += 3;
        } else {
            dst[di++] = *src++;
        }
    }
    dst[di] = '\0';
}

/* ===================================================================
 * HTTP Handlers
 * =================================================================== */

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, PORTAL_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t save_post_handler(httpd_req_t *req)
{
    char body[256] = {0};
    int received = httpd_req_recv(req, body, sizeof(body) - 1);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty request");
        return ESP_FAIL;
    }
    body[received] = '\0';

    /* Parse ssid= and pass= from form body */
    char ssid_raw[64] = {0};
    char pass_raw[64] = {0};
    char ssid[64] = {0};
    char pass[64] = {0};

    char *s = strstr(body, "ssid=");
    if (s) {
        s += 5;
        char *end = strchr(s, '&');
        size_t len = end ? (size_t)(end - s) : strlen(s);
        if (len >= sizeof(ssid_raw)) len = sizeof(ssid_raw) - 1;
        memcpy(ssid_raw, s, len);
        ssid_raw[len] = '\0';
    }

    char *p = strstr(body, "pass=");
    if (p) {
        p += 5;
        char *end = strchr(p, '&');
        size_t len = end ? (size_t)(end - p) : strlen(p);
        if (len >= sizeof(pass_raw)) len = sizeof(pass_raw) - 1;
        memcpy(pass_raw, p, len);
        pass_raw[len] = '\0';
    }

    url_decode(ssid, ssid_raw, sizeof(ssid));
    url_decode(pass, pass_raw, sizeof(pass));

    if (strlen(ssid) == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SSID is required");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Saving WiFi: SSID='%s'", ssid);
    nvs_config_set_string("wifi_ssid", ssid);
    nvs_config_set_string("wifi_pass", pass);

    /* Send success page */
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, PORTAL_OK_HTML, HTTPD_RESP_USE_STRLEN);

    /* Reboot after a short delay */
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();

    return ESP_OK;
}

/* Catch-all handler for captive portal redirect */
static esp_err_t redirect_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* ===================================================================
 * DNS Server (redirects all lookups to 192.168.4.1)
 * =================================================================== */

static void dns_server_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "DNS: socket failed");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(53),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "DNS: bind failed");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    /* Set receive timeout so we can check s_dns_running */
    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    ESP_LOGI(TAG, "DNS server started on port 53");

    uint8_t buf[512];
    struct sockaddr_in client_addr;
    socklen_t addr_len;

    while (s_dns_running) {
        addr_len = sizeof(client_addr);
        int len = recvfrom(sock, buf, sizeof(buf), 0,
                           (struct sockaddr *)&client_addr, &addr_len);
        if (len < 12) continue;  /* Too short or timeout */

        /*
         * Build a minimal DNS response:
         * - Copy the query ID and question
         * - Set response flags (QR=1, AA=1, RA=1)
         * - Add a single A record pointing to 192.168.4.1
         */
        uint8_t resp[512];
        memcpy(resp, buf, len);

        /* Header flags: QR=1, AA=1, RA=1, RCODE=0 */
        resp[2] = 0x85;  /* QR=1, Opcode=0, AA=1, TC=0, RD=1 */
        resp[3] = 0x80;  /* RA=1, Z=0, RCODE=0 */

        /* Answer count = 1 */
        resp[6] = 0x00;
        resp[7] = 0x01;

        /* Append answer: name pointer + type A + class IN + TTL + 4 bytes IP */
        int pos = len;
        resp[pos++] = 0xC0;  /* Name pointer to offset 12 (question name) */
        resp[pos++] = 0x0C;
        resp[pos++] = 0x00; resp[pos++] = 0x01;  /* Type A */
        resp[pos++] = 0x00; resp[pos++] = 0x01;  /* Class IN */
        resp[pos++] = 0x00; resp[pos++] = 0x00;
        resp[pos++] = 0x00; resp[pos++] = 0x3C;  /* TTL = 60 */
        resp[pos++] = 0x00; resp[pos++] = 0x04;  /* Data length = 4 */
        resp[pos++] = 192;                        /* 192.168.4.1 */
        resp[pos++] = 168;
        resp[pos++] = 4;
        resp[pos++] = 1;

        sendto(sock, resp, pos, 0,
               (struct sockaddr *)&client_addr, addr_len);
    }

    close(sock);
    ESP_LOGI(TAG, "DNS server stopped");
    vTaskDelete(NULL);
}

/* ===================================================================
 * Start / Stop
 * =================================================================== */

void captive_portal_start(void)
{
    if (s_httpd) return;  /* Already running */

    /* Start HTTP server */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;
    config.uri_match_fn = httpd_uri_match_wildcard;

    if (httpd_start(&s_httpd, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return;
    }

    /* Register handlers */
    const httpd_uri_t root = {
        .uri = "/", .method = HTTP_GET, .handler = root_get_handler
    };
    const httpd_uri_t save = {
        .uri = "/save", .method = HTTP_POST, .handler = save_post_handler
    };
    const httpd_uri_t catchall = {
        .uri = "/*", .method = HTTP_GET, .handler = redirect_handler
    };
    httpd_register_uri_handler(s_httpd, &root);
    httpd_register_uri_handler(s_httpd, &save);
    httpd_register_uri_handler(s_httpd, &catchall);

    ESP_LOGI(TAG, "Captive portal HTTP server started on port 80");

    /* Start DNS redirect server */
    s_dns_running = true;
    xTaskCreate(dns_server_task, "dns_srv", 4096, NULL, 5, &s_dns_task);
}

void captive_portal_stop(void)
{
    if (s_httpd) {
        httpd_stop(s_httpd);
        s_httpd = NULL;
    }
    s_dns_running = false;
    s_dns_task = NULL;
    ESP_LOGI(TAG, "Captive portal stopped");
}
