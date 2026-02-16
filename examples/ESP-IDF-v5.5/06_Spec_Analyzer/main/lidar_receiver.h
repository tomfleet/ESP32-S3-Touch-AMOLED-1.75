#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "lidar_types.h"

esp_err_t lidar_receiver_start(void);
bool lidar_receiver_get_latest_frame(lidar_scan_frame_t *out_frame);
void lidar_receiver_get_stats(lidar_stats_t *out_stats);