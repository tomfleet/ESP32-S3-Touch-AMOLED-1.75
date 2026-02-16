#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_memory_utils.h"
#include "lvgl.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "lv_demos.h"

void app_main(void)
{

    bsp_display_start();

    bsp_display_lock(-1);

#if defined(LV_USE_DEMO_BENCHMARK) && LV_USE_DEMO_BENCHMARK
    lv_demo_benchmark();
#elif defined(LV_USE_DEMO_WIDGETS) && LV_USE_DEMO_WIDGETS
    lv_demo_widgets();
#elif defined(LV_USE_DEMO_MUSIC)
    lv_demo_music();
#else
    ESP_LOGW("lvgl_demo_v9", "No LVGL demo enabled in config");
#endif

    bsp_display_unlock();
}