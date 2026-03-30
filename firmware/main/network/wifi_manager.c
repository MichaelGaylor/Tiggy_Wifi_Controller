/*
 * WiFi CNC Controller - WiFi Connection Manager
 *
 * On boot:
 *   - If stored SSID is non-empty, attempt STA connection
 *   - If STA fails within CFG_WIFI_STA_TIMEOUT_MS, fall back to AP mode
 *   - AP SSID: "WiFiCNC-XXXX" (last 4 hex digits of MAC address)
 *
 * Reconnection uses exponential backoff: 100ms, 200ms, 400ms, ... 5000ms
 */

#include "wifi_manager.h"
#include "../config.h"
#include "../persist/nvs_config.h"

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_mac.h"

static const char *TAG = "wifi_mgr";

/* Event group bits */
#define WIFI_CONNECTED_BIT   BIT0
#define WIFI_FAIL_BIT        BIT1

static EventGroupHandle_t s_wifi_event_group;
static esp_netif_t *s_sta_netif = NULL;
static esp_netif_t *s_ap_netif = NULL;
static bool s_connected = false;
static bool s_ap_mode = false;
static int s_retry_count = 0;
static uint32_t s_reconnect_delay_ms = CFG_WIFI_RECONNECT_BASE_MS;

/* ===================================================================
 * Event Handler
 * =================================================================== */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "STA started, connecting...");
            esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            s_connected = false;
            ESP_LOGW(TAG, "WiFi disconnected (retry %d, delay %lums)",
                     s_retry_count, (unsigned long)s_reconnect_delay_ms);

            if (s_retry_count < 50) {
                vTaskDelay(pdMS_TO_TICKS(s_reconnect_delay_ms));
                esp_wifi_connect();
                s_retry_count++;
                /* Exponential backoff */
                s_reconnect_delay_ms *= 2;
                if (s_reconnect_delay_ms > CFG_WIFI_RECONNECT_MAX_MS) {
                    s_reconnect_delay_ms = CFG_WIFI_RECONNECT_MAX_MS;
                }
            } else {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            }
            break;

        case WIFI_EVENT_AP_STACONNECTED: {
            wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
            ESP_LOGI(TAG, "AP: Station connected (MAC:" MACSTR ")",
                     MAC2STR(event->mac));
            break;
        }

        case WIFI_EVENT_AP_STADISCONNECTED: {
            wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
            ESP_LOGI(TAG, "AP: Station disconnected (MAC:" MACSTR ")",
                     MAC2STR(event->mac));
            break;
        }

        default:
            break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_connected = true;
        s_retry_count = 0;
        s_reconnect_delay_ms = CFG_WIFI_RECONNECT_BASE_MS;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ===================================================================
 * AP Mode Fallback
 * =================================================================== */

static void start_ap_mode(void)
{
    ESP_LOGI(TAG, "Starting AP fallback mode");

    /* Generate AP SSID from MAC address */
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char ap_ssid[32];
    snprintf(ap_ssid, sizeof(ap_ssid), "%s%02X%02X",
             CFG_AP_SSID_PREFIX, mac[4], mac[5]);

    wifi_config_t ap_config = {0};
    strlcpy((char *)ap_config.ap.ssid, ap_ssid, sizeof(ap_config.ap.ssid));
    strlcpy((char *)ap_config.ap.password, CFG_AP_DEFAULT_PASSWORD,
            sizeof(ap_config.ap.password));
    ap_config.ap.ssid_len = strlen(ap_ssid);
    ap_config.ap.channel = CFG_AP_CHANNEL;
    ap_config.ap.max_connection = CFG_AP_MAX_CONNECTIONS;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    s_ap_mode = true;
    ESP_LOGI(TAG, "AP mode active: SSID='%s' Password='%s'",
             ap_ssid, CFG_AP_DEFAULT_PASSWORD);
}

/* ===================================================================
 * STA Mode
 * =================================================================== */

static bool start_sta_mode(const char *ssid, const char *password)
{
    ESP_LOGI(TAG, "Attempting STA connection to '%s'", ssid);

    wifi_config_t sta_config = {0};
    strlcpy((char *)sta_config.sta.ssid, ssid, sizeof(sta_config.sta.ssid));
    strlcpy((char *)sta_config.sta.password, password, sizeof(sta_config.sta.password));
    sta_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Wait for connection or failure */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(CFG_WIFI_STA_TIMEOUT_MS));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "STA connected successfully");
        return true;
    }

    ESP_LOGW(TAG, "STA connection failed/timed out");
    esp_wifi_stop();
    return false;
}

/* ===================================================================
 * Initialization
 * =================================================================== */

void wifi_manager_init(void)
{
    s_wifi_event_group = xEventGroupCreate();

    /* Initialize TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    s_sta_netif = esp_netif_create_default_wifi_sta();
    s_ap_netif = esp_netif_create_default_wifi_ap();

    /* WiFi init with default config */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Register event handlers */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    /* Read stored WiFi credentials */
    char ssid[WCNC_SSID_MAX_LEN] = {0};
    char password[WCNC_PASSWORD_MAX_LEN] = {0};
    nvs_config_get_string("wifi_ssid", ssid, sizeof(ssid), CFG_DEFAULT_SSID);
    nvs_config_get_string("wifi_pass", password, sizeof(password), CFG_DEFAULT_PASSWORD);

    /* If SSID is configured, try STA mode; otherwise go straight to AP */
    if (strlen(ssid) > 0) {
        if (!start_sta_mode(ssid, password)) {
            start_ap_mode();
        }
    } else {
        ESP_LOGI(TAG, "No WiFi SSID configured, starting AP mode");
        start_ap_mode();
    }
}

/* ===================================================================
 * Status Queries
 * =================================================================== */

bool wifi_is_connected(void)
{
    return s_connected;
}

bool wifi_is_ap_mode(void)
{
    return s_ap_mode;
}

void wifi_get_ip_string(char *buf, size_t buf_len)
{
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = s_ap_mode ? s_ap_netif : s_sta_netif;

    if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
        snprintf(buf, buf_len, IPSTR, IP2STR(&ip_info.ip));
    } else {
        strlcpy(buf, "0.0.0.0", buf_len);
    }
}

int8_t wifi_get_rssi(void)
{
    if (!s_connected) return 0;

    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        return ap_info.rssi;
    }
    return 0;
}
