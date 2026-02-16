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
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num;
static bool s_connected;
static char s_connected_ssid[33];
static char s_connected_ip[16];

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_connected = false;
        strlcpy(s_connected_ip, "0.0.0.0", sizeof(s_connected_ip));
        if (s_retry_num < AROUNDER_WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "Retrying Wi-Fi connection (%d/%d)", s_retry_num, AROUNDER_WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi connected, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_connected = true;
        snprintf(s_connected_ip, sizeof(s_connected_ip), IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t arounder_wifi_connect(void)
{
    if (strlen(AROUNDER_WIFI_SSID) == 0) {
        ESP_LOGW(TAG, "AROUNDER_WIFI_SSID is empty. Skipping Wi-Fi init.");
        strlcpy(s_connected_ssid, "<not set>", sizeof(s_connected_ssid));
        strlcpy(s_connected_ip, "0.0.0.0", sizeof(s_connected_ip));
        s_connected = false;
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

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, AROUNDER_WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, AROUNDER_WIFI_PASSWORD, sizeof(wifi_config.sta.password));
    strlcpy(s_connected_ssid, AROUNDER_WIFI_SSID, sizeof(s_connected_ssid));
    strlcpy(s_connected_ip, "0.0.0.0", sizeof(s_connected_ip));
    s_connected = false;
    wifi_config.sta.threshold.authmode = strlen(AROUNDER_WIFI_PASSWORD) == 0 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "Wi-Fi power save disabled (WIFI_PS_NONE)");

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(AROUNDER_WIFI_CONNECT_TIMEOUT_MS));

    if (bits & WIFI_CONNECTED_BIT) {
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Wi-Fi connect failed or timed out");
    return ESP_FAIL;
}

void arounder_wifi_get_status(char *ssid, size_t ssid_len, char *ip, size_t ip_len, bool *connected)
{
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
