#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "lidar_types.h"

esp_err_t lidar_protocol_parse_json_datagram(const uint8_t *buf, uint16_t len, lidar_scan_frame_t *out_frame);