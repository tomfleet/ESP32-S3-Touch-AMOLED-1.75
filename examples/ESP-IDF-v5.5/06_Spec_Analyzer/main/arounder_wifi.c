#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "arounder_config.h"
#include "arounder_wifi.h"

static const char *TAG = "arounder_wifi";

#define WIFI_CONNECTED_BIT BIT0

static EventGroupHandle_t s_wifi_event_group;
static bool s_connected;
static bool s_sta_associated;
static bool s_sta_has_ip;
static char s_connected_ssid[33];
static char s_connected_ip[16];
static esp_netif_t *s_sta_netif;

static void refresh_connected_state(void)
{
    bool new_connected = s_sta_associated && s_sta_has_ip;
    if (new_connected != s_connected) {
        ESP_LOGI(TAG,
                 "Link state changed: associated=%d has_ip=%d connected=%d",
                 s_sta_associated,
                 s_sta_has_ip,
                 new_connected);
    }
    s_connected = new_connected;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data;
        s_sta_associated = false;
        s_sta_has_ip = false;
        strlcpy(s_connected_ip, "0.0.0.0", sizeof(s_connected_ip));
        refresh_connected_state();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        if (disc != NULL) {
            if (disc->rssi == -128 || disc->reason == 205) {
                ESP_LOGW(TAG,
                         "STA disconnected, reason=%d rssi=unknown",
                         disc->reason);
            } else {
                ESP_LOGW(TAG,
                         "STA disconnected, reason=%d rssi=%d",
                         disc->reason,
                         disc->rssi);
            }
        }
        esp_wifi_connect();
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        s_sta_associated = true;
        refresh_connected_state();
        ESP_LOGI(TAG, "STA associated to AP");
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi connected, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_sta_has_ip = true;
        snprintf(s_connected_ip, sizeof(s_connected_ip), IPSTR, IP2STR(&event->ip_info.ip));
        refresh_connected_state();
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_LOST_IP) {
        s_sta_has_ip = false;
        strlcpy(s_connected_ip, "0.0.0.0", sizeof(s_connected_ip));
        refresh_connected_state();
        ESP_LOGW(TAG, "STA lost IP address");
    }
}

esp_err_t arounder_wifi_connect(void)
{
    if (strlen(AROUNDER_WIFI_SSID) == 0) {
        ESP_LOGW(TAG, "AROUNDER_WIFI_SSID is empty. Skipping Wi-Fi init.");
        strlcpy(s_connected_ssid, "<not set>", sizeof(s_connected_ssid));
        strlcpy(s_connected_ip, "0.0.0.0", sizeof(s_connected_ip));
        s_sta_associated = false;
        s_sta_has_ip = false;
        refresh_connected_state();
        return ESP_OK;
    }

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    if (s_sta_netif == NULL) {
        s_sta_netif = esp_netif_create_default_wifi_sta();
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    if (s_wifi_event_group == NULL) {
        s_wifi_event_group = xEventGroupCreate();
        if (s_wifi_event_group == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_LOST_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, AROUNDER_WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, AROUNDER_WIFI_PASSWORD, sizeof(wifi_config.sta.password));
    strlcpy(s_connected_ssid, AROUNDER_WIFI_SSID, sizeof(s_connected_ssid));
    strlcpy(s_connected_ip, "0.0.0.0", sizeof(s_connected_ip));
    s_sta_associated = false;
    s_sta_has_ip = false;
    refresh_connected_state();

    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_LOGW(TAG, "Wi-Fi credentials (dev): ssid='%s' password='%s'", AROUNDER_WIFI_SSID, AROUNDER_WIFI_PASSWORD);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "Wi-Fi power save disabled (WIFI_PS_NONE)");

    ESP_LOGI(TAG,
             "Wi-Fi connect wait timeout=%d ms (non-fatal; background reconnect continues)",
             AROUNDER_WIFI_CONNECT_TIMEOUT_MS);

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(AROUNDER_WIFI_CONNECT_TIMEOUT_MS));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to SSID '%s'", AROUNDER_WIFI_SSID);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "not connected yet; background reconnect will continue");
    return ESP_OK;
}

void arounder_wifi_get_status(char *ssid, size_t ssid_len, char *ip, size_t ip_len, bool *connected)
{
    wifi_ap_record_t ap_info = {0};
    esp_err_t ap_err = esp_wifi_sta_get_ap_info(&ap_info);
    s_sta_associated = (ap_err == ESP_OK);

    esp_netif_ip_info_t ip_info = {0};
    bool netif_has_ip = false;
    if (s_sta_netif != NULL && esp_netif_get_ip_info(s_sta_netif, &ip_info) == ESP_OK) {
        netif_has_ip = (ip_info.ip.addr != 0);
        if (netif_has_ip) {
            snprintf(s_connected_ip, sizeof(s_connected_ip), IPSTR, IP2STR(&ip_info.ip));
        }
    }
    if (!netif_has_ip) {
        strlcpy(s_connected_ip, "0.0.0.0", sizeof(s_connected_ip));
    }
    s_sta_has_ip = netif_has_ip;
    refresh_connected_state();

    if (ssid != NULL && ssid_len > 0) {
        strlcpy(ssid, s_connected_ssid, ssid_len);
    }
    if (ip != NULL && ip_len > 0) {
        strlcpy(ip, s_connected_ip, ip_len);
    }
    if (connected != NULL) {
        *connected = s_connected;
    }
}
