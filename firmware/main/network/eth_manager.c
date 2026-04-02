/*
 * WiFi CNC Controller - Ethernet Manager (W5500 SPI)
 *
 * Initializes a W5500 SPI Ethernet module if present.
 * Uses the same lwIP TCP/IP stack as WiFi, so all higher-level
 * networking (UDP motion, TCP config, telnet) works unchanged.
 *
 * If the W5500 is not detected (SPI timeout), init returns false
 * and the caller falls back to WiFi.
 */

#include "eth_manager.h"
#include "../config.h"
#include "../pin_config.h"
#include "../persist/nvs_config.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_eth.h"
#include "esp_eth_mac_spi.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

static const char *TAG = "eth_mgr";

#define ETH_CONNECTED_BIT  BIT0
#define ETH_FAIL_BIT       BIT1

static EventGroupHandle_t s_eth_event_group = NULL;
static esp_netif_t       *s_eth_netif       = NULL;
static esp_eth_handle_t   s_eth_handle      = NULL;
static bool               s_connected       = false;

/* ===================================================================
 * Event Handler
 * =================================================================== */

static void eth_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == ETH_EVENT) {
        switch (event_id) {
        case ETHERNET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Ethernet link up");
            break;
        case ETHERNET_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Ethernet link down");
            s_connected = false;
            break;
        case ETHERNET_EVENT_START:
            ESP_LOGI(TAG, "Ethernet started");
            break;
        case ETHERNET_EVENT_STOP:
            ESP_LOGI(TAG, "Ethernet stopped");
            s_connected = false;
            break;
        default:
            break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_ETH_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Ethernet got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_connected = true;
        if (s_eth_event_group) {
            xEventGroupSetBits(s_eth_event_group, ETH_CONNECTED_BIT);
        }
    }
}

/* ===================================================================
 * Initialization
 * =================================================================== */

bool eth_manager_init(void)
{
#if !HAS_ETHERNET
    return false;
#else
    ESP_LOGI(TAG, "Initializing W5500 Ethernet...");

    s_eth_event_group = xEventGroupCreate();

    /* Read SPI pin assignments from saved settings (fallback to compile-time defaults) */
    int mosi_pin = (int)nvs_config_get_u8("p_emosi", PIN_ETH_MOSI);
    int miso_pin = (int)nvs_config_get_u8("p_emiso", PIN_ETH_MISO);
    int sclk_pin = (int)nvs_config_get_u8("p_esclk", PIN_ETH_SCLK);
    int int_pin  = (int)nvs_config_get_u8("p_eint",  PIN_ETH_INT);
    int spi_host = (int)nvs_config_get_u8("p_espi",  PIN_ETH_SPI_HOST);

    ESP_LOGI(TAG, "W5500 pins: MOSI=%d MISO=%d SCLK=%d INT=%d SPI_HOST=%d",
             mosi_pin, miso_pin, sclk_pin, int_pin, spi_host);

    /* Configure SPI bus */
    spi_bus_config_t buscfg = {
        .mosi_io_num   = mosi_pin,
        .miso_io_num   = miso_pin,
        .sclk_io_num   = sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    esp_err_t ret = spi_bus_initialize(spi_host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SPI bus init failed (%s) - no W5500?", esp_err_to_name(ret));
        vEventGroupDelete(s_eth_event_group);
        s_eth_event_group = NULL;
        return false;
    }

    /* Configure W5500 SPI device (CS tied to GND = always selected, use -1) */
    spi_device_interface_config_t devcfg = {
        .command_bits   = 16,   /* W5500 address phase */
        .address_bits   = 8,    /* W5500 control phase */
        .mode           = 0,
        .clock_speed_hz = 20 * 1000 * 1000,  /* 20 MHz SPI clock */
        .spics_io_num   = -1,   /* CS tied to GND (always selected) */
        .queue_size     = 20,
    };

    /* Create W5500 MAC */
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(spi_host, &devcfg);
    w5500_config.int_gpio_num = int_pin;

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    if (!mac) {
        ESP_LOGW(TAG, "W5500 MAC creation failed - module not connected?");
        spi_bus_free(spi_host);
        vEventGroupDelete(s_eth_event_group);
        s_eth_event_group = NULL;
        return false;
    }

    /* Create W5500 PHY */
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.reset_gpio_num = -1;  /* RESET tied to 3.3V, no GPIO needed */
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);

    /* Install Ethernet driver */
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    ret = esp_eth_driver_install(&eth_config, &s_eth_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Ethernet driver install failed: %s", esp_err_to_name(ret));
        mac->del(mac);
        phy->del(phy);
        spi_bus_free(spi_host);
        vEventGroupDelete(s_eth_event_group);
        s_eth_event_group = NULL;
        return false;
    }

    /* Create network interface and attach to Ethernet */
    esp_netif_inherent_config_t netif_cfg = ESP_NETIF_INHERENT_DEFAULT_ETH();
    esp_netif_config_t cfg = {
        .base   = &netif_cfg,
        .stack  = ESP_NETIF_NETSTACK_DEFAULT_ETH,
    };
    s_eth_netif = esp_netif_new(&cfg);
    esp_netif_attach(s_eth_netif, esp_eth_new_netif_glue(s_eth_handle));

    /* Register event handlers */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_ETH_GOT_IP, &eth_event_handler, NULL, NULL));

    /* Start Ethernet */
    ret = esp_eth_start(s_eth_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Ethernet start failed: %s", esp_err_to_name(ret));
        esp_eth_driver_uninstall(s_eth_handle);
        s_eth_handle = NULL;
        esp_netif_destroy(s_eth_netif);
        s_eth_netif = NULL;
        spi_bus_free(spi_host);
        vEventGroupDelete(s_eth_event_group);
        s_eth_event_group = NULL;
        return false;
    }

    /* Wait for IP assignment (DHCP) with timeout */
    ESP_LOGI(TAG, "Waiting for Ethernet IP (DHCP)...");
    EventBits_t bits = xEventGroupWaitBits(s_eth_event_group,
        ETH_CONNECTED_BIT | ETH_FAIL_BIT,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(CFG_ETH_TIMEOUT_MS));

    if (bits & ETH_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Ethernet connected successfully");
        return true;
    }

    /* Timeout - Ethernet link not up or no DHCP. Clean up and let WiFi take over. */
    ESP_LOGW(TAG, "Ethernet timeout (no link or no DHCP) - falling back to WiFi");
    esp_eth_stop(s_eth_handle);
    esp_eth_driver_uninstall(s_eth_handle);
    s_eth_handle = NULL;
    esp_netif_destroy(s_eth_netif);
    s_eth_netif = NULL;
    spi_bus_free(spi_host);
    vEventGroupDelete(s_eth_event_group);
    s_eth_event_group = NULL;
    return false;
#endif /* HAS_ETHERNET */
}

/* ===================================================================
 * Status Queries
 * =================================================================== */

bool eth_is_connected(void)
{
    return s_connected;
}

void eth_get_ip_string(char *buf, size_t buf_len)
{
    if (s_eth_netif) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(s_eth_netif, &ip_info) == ESP_OK) {
            snprintf(buf, buf_len, IPSTR, IP2STR(&ip_info.ip));
            return;
        }
    }
    strlcpy(buf, "0.0.0.0", buf_len);
}
