#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "arounder_config.h"

typedef struct {
    float angle_deg;
    float range_mm;
    uint8_t intensity;
} lidar_point_t;

typedef struct {
    char source[16];
    uint32_t scan_id;
    int speed_raw;
    float speed_rpm;
    int crc_fail;
    uint64_t scan_t0_us;
    uint64_t scan_t1_us;
    uint64_t scan_period_us;
    uint16_t point_count;
    lidar_point_t points[AROUNDER_MAX_POINTS_PER_SCAN];
} lidar_scan_frame_t;

typedef struct {
    uint32_t packets_seen_raw;
    uint32_t packets_bad_json;
    uint32_t packets_missing_required_fields;
    uint32_t packets_dup_or_old_scan;
    uint32_t scan_id_gaps;
    uint32_t frames_accepted;
} lidar_stats_t;