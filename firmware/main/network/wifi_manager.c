/*
 * WiFi CNC Controller - WiFi Connection Manager
 *
 * With a network configured, the controller joins it and, if the connection
 * is ever lost, keeps trying to get it back for as long as it is powered
 * (router restarted, out of range for a while ...).
 *
 * If the network cannot be joined at start-up - or none is configured - the
 * controller opens its own access point "TiggyCNC-XXXX" (last four hex
 * digits of the MAC address) with a set-up page.  When a network is
 * configured it goes on trying to join it in the background, so a
 * controller that was powered up before its router still gets on line by
 * itself; the access point is closed again once that has happened and
 * nobody is using it.
 */

#include "wifi_manager.h"
#include "captive_portal.h"
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
#include "esp_timer.h"

static const char *TAG = "wifi_mgr";

#define WIFI_CONNECTED_BIT      BIT0

/* While the set-up access point is open, looking for the configured network
 * interrupts it briefly each time, so it is done less often. */
#define RETRY_WITH_AP_MS        15000
/* The access point closes this long after the network has been joined,
 * provided no device is connected to it. */
#define AP_CLOSE_AFTER_MS       60000

static EventGroupHandle_t s_wifi_event_group;
static esp_netif_t *s_sta_netif = NULL;
static esp_netif_t *s_ap_netif = NULL;
static volatile bool s_connected = false;
static volatile bool s_ap_mode = false;
static bool s_want_sta = false;
static uint32_t s_reconnect_delay_ms = CFG_WIFI_RECONNECT_BASE_MS;
static esp_timer_handle_t s_retry_timer = NULL;
static esp_timer_handle_t s_ap_close_timer = NULL;

/* ===================================================================
 * Timers (they run in the timer task, so the event task never sleeps)
 * =================================================================== */

static void retry_timer_cb(void *arg)
{
    (void)arg;
    if (s_want_sta && !s_connected) esp_wifi_connect();
}

static void ap_close_timer_cb(void *arg)
{
    (void)arg;
    if (!s_ap_mode || !s_connected) return;
    wifi_sta_list_t clients;
    memset(&clients, 0, sizeof(clients));
    if (esp_wifi_ap_get_sta_list(&clients) == ESP_OK && clients.num > 0) {
        esp_timer_start_once(s_ap_close_timer, (uint64_t)AP_CLOSE_AFTER_MS * 1000);   /* someone is on the set-up page */
        return;
    }
    ESP_LOGI(TAG, "On the network: closing the set-up access point");
    captive_portal_stop();
    esp_wifi_set_mode(WIFI_MODE_STA);
    s_ap_mode = false;
}

/* ===================================================================
 * Event Handler
 * =================================================================== */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    (void)arg;
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            if (s_want_sta) esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_DISCONNECTED: {
            s_connected = false;
            if (!s_want_sta) break;
            uint32_t delay = s_ap_mode ? RETRY_WITH_AP_MS : s_reconnect_delay_ms;
            ESP_LOGW(TAG, "WiFi not connected, trying again in %lu ms", (unsigned long)delay);
            esp_timer_stop(s_retry_timer);
            esp_timer_start_once(s_retry_timer, (uint64_t)delay * 1000);
            s_reconnect_delay_ms *= 2;
            if (s_reconnect_delay_ms > CFG_WIFI_RECONNECT_MAX_MS) s_reconnect_delay_ms = CFG_WIFI_RECONNECT_MAX_MS;
            break;
        }

        case WIFI_EVENT_AP_STACONNECTED: {
            wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
            ESP_LOGI(TAG, "AP: Station connected (MAC:" MACSTR ")", MAC2STR(event->mac));
            break;
        }

        case WIFI_EVENT_AP_STADISCONNECTED: {
            wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
            ESP_LOGI(TAG, "AP: Station disconnected (MAC:" MACSTR ")", MAC2STR(event->mac));
            break;
        }

        default:
            break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_connected = true;
        s_reconnect_delay_ms = CFG_WIFI_RECONNECT_BASE_MS;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        if (s_ap_mode) {
            esp_timer_stop(s_ap_close_timer);
            esp_timer_start_once(s_ap_close_timer, (uint64_t)AP_CLOSE_AFTER_MS * 1000);
        }
    }
}

/* ===================================================================
 * Access point for set-up
 * =================================================================== */

static void start_ap_mode(bool keep_sta)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char ap_ssid[32];
    snprintf(ap_ssid, sizeof(ap_ssid), "%s%02X%02X", CFG_AP_SSID_PREFIX, mac[4], mac[5]);

    wifi_config_t ap_config = {0};
    strlcpy((char *)ap_config.ap.ssid, ap_ssid, sizeof(ap_config.ap.ssid));
    strlcpy((char *)ap_config.ap.password, CFG_AP_DEFAULT_PASSWORD, sizeof(ap_config.ap.password));
    ap_config.ap.ssid_len = strlen(ap_ssid);
    ap_config.ap.channel = CFG_AP_CHANNEL;
    ap_config.ap.max_connection = CFG_AP_MAX_CONNECTIONS;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

    s_ap_mode = true;
    ESP_ERROR_CHECK(esp_wifi_set_mode(keep_sta ? WIFI_MODE_APSTA : WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    /* No radio power saving.  By default the station sleeps between the
     * access point's beacons and only then collects what was sent to it:
     * motion and stop commands arrived 150 ms late on average and up to
     * 0.7 s late (measured on a DevKitC, DTIM 3).  A machine controller
     * has the power to spare. */
    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_LOGW(TAG, "Set-up access point open: SSID '%s', password '%s'", ap_ssid, CFG_AP_DEFAULT_PASSWORD);
    captive_portal_start();
}

/* ===================================================================
 * Station
 * =================================================================== */

/* A fixed address instead of DHCP, if one is configured. */
static void apply_static_ip(void)
{
    if (nvs_config_get_u8("ip_mode", CFG_DEFAULT_IP_MODE) != 1) return;
    esp_netif_ip_info_t info;
    memset(&info, 0, sizeof(info));
    /* Stored in network byte order, as inet_addr() gives them */
    info.ip.addr = nvs_config_get_u32("ip_addr", 0);
    info.gw.addr = nvs_config_get_u32("ip_gw", 0);
    info.netmask.addr = nvs_config_get_u32("ip_mask", 0);
    if (info.ip.addr == 0 || info.netmask.addr == 0) {
        ESP_LOGW(TAG, "Static IP selected but no address stored - using DHCP");
        return;
    }
    esp_netif_dhcpc_stop(s_sta_netif);
    if (esp_netif_set_ip_info(s_sta_netif, &info) == ESP_OK) {
        ESP_LOGI(TAG, "Static IP: " IPSTR, IP2STR(&info.ip));
    } else {
        ESP_LOGW(TAG, "Static IP could not be set - using DHCP");
        esp_netif_dhcpc_start(s_sta_netif);
    }
}

static bool start_sta_mode(const char *ssid, const char *password)
{
    ESP_LOGI(TAG, "Joining '%s'", ssid);

    wifi_config_t sta_config = {0};
    strlcpy((char *)sta_config.sta.ssid, ssid, sizeof(sta_config.sta.ssid));
    strlcpy((char *)sta_config.sta.password, password, sizeof(sta_config.sta.password));
    sta_config.sta.threshold.authmode = password[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    s_want_sta = true;
    apply_static_ip();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    /* No radio power saving.  By default the station sleeps between the
     * access point's beacons and only then collects what was sent to it:
     * motion and stop commands arrived 150 ms late on average and up to
     * 0.7 s late (measured on a DevKitC, DTIM 3).  A machine controller
     * has the power to spare. */
    esp_wifi_set_ps(WIFI_PS_NONE);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                                           pdFALSE, pdFALSE, pdMS_TO_TICKS(CFG_WIFI_STA_TIMEOUT_MS));
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

/* ===================================================================
 * Initialization
 * =================================================================== */

void wifi_manager_init(void)
{
    s_wifi_event_group = xEventGroupCreate();

    /* Note: esp_netif_init() and esp_event_loop_create_default() are called
     * once in main.c before either Ethernet or WiFi manager is initialized. */

    s_sta_netif = esp_netif_create_default_wifi_sta();
    s_ap_netif = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    const esp_timer_create_args_t retry_args = { .callback = retry_timer_cb, .name = "wifi_retry" };
    const esp_timer_create_args_t close_args = { .callback = ap_close_timer_cb, .name = "ap_close" };
    ESP_ERROR_CHECK(esp_timer_create(&retry_args, &s_retry_timer));
    ESP_ERROR_CHECK(esp_timer_create(&close_args, &s_ap_close_timer));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    char ssid[WCNC_SSID_MAX_LEN + 1] = {0};
    char password[WCNC_PASSWORD_MAX_LEN + 1] = {0};
    nvs_config_get_string("wifi_ssid", ssid, sizeof(ssid), CFG_DEFAULT_SSID);
    nvs_config_get_string("wifi_pass", password, sizeof(password), CFG_DEFAULT_PASSWORD);

    if (strlen(ssid) == 0) {
        ESP_LOGI(TAG, "No WiFi network configured");
        start_ap_mode(false);
    } else if (!start_sta_mode(ssid, password)) {
        ESP_LOGW(TAG, "'%s' not joined yet: opening the set-up access point and trying on", ssid);
        start_ap_mode(true);
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
    esp_netif_t *netif = s_connected ? s_sta_netif : (s_ap_mode ? s_ap_netif : s_sta_netif);

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
