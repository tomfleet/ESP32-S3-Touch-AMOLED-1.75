#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "circular_renderer.h"
#include "lidar_receiver.h"
#include "arounder_wifi.h"

#define TAG "arounder"

void app_main(void)
{
    ESP_LOGI(TAG, "Starting arounder (LiDAR UDP display base)");

    esp_err_t nvs_result = nvs_flash_init();
    if (nvs_result == ESP_ERR_NVS_NO_FREE_PAGES || nvs_result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_result = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_result);

    ESP_ERROR_CHECK(arounder_renderer_start());

    ESP_ERROR_CHECK(arounder_wifi_connect());

    esp_err_t rx_err = lidar_receiver_start();
    if (rx_err != ESP_OK) {
        ESP_LOGE(TAG, "lidar_receiver_start failed: %s", esp_err_to_name(rx_err));
    }
}
