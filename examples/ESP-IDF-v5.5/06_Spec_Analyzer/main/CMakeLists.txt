idf_component_register(
    SRCS
        main.c
        lidar_protocol.c
        lidar_receiver.c
        circular_renderer.c
        arounder_wifi.c
    INCLUDE_DIRS .
    REQUIRES
        json
        nvs_flash
        esp_event
        esp_netif
        esp_wifi
        waveshare__esp32_s3_touch_amoled_1_75)
