#include <string.h>
#include <stdint.h>
#include "cJSON.h"
#include "lidar_protocol.h"

static bool parse_u64_field(cJSON *obj, const char *name, uint64_t *value)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, name);
    if (!cJSON_IsNumber(item)) {
        return false;
    }

    double number = item->valuedouble;
    if (number < 0.0) {
        *value = 0;
    } else if (number > (double)UINT64_MAX) {
        *value = UINT64_MAX;
    } else {
        *value = (uint64_t)number;
    }
    return true;
}

esp_err_t lidar_protocol_parse_json_datagram(const uint8_t *buf, uint16_t len, lidar_scan_frame_t *out_frame)
{
    if (buf == NULL || out_frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *root = cJSON_ParseWithLength((const char *)buf, len);
    if (root == NULL) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    cJSON *scan = cJSON_GetObjectItemCaseSensitive(root, "scan");
    cJSON *points = cJSON_GetObjectItemCaseSensitive(root, "points");
    if (!cJSON_IsNumber(scan) || !cJSON_IsArray(points)) {
        cJSON_Delete(root);
        return ESP_ERR_INVALID_STATE;
    }

    memset(out_frame, 0, sizeof(*out_frame));
    out_frame->scan_id = (uint32_t)(scan->valuedouble < 0.0 ? 0.0 : scan->valuedouble);

    cJSON *source = cJSON_GetObjectItemCaseSensitive(root, "source");
    if (cJSON_IsString(source) && source->valuestring != NULL) {
        strncpy(out_frame->source, source->valuestring, sizeof(out_frame->source) - 1);
    }

    cJSON *speed_raw = cJSON_GetObjectItemCaseSensitive(root, "speed_raw");
    if (cJSON_IsNumber(speed_raw)) {
        out_frame->speed_raw = speed_raw->valueint;
    }

    cJSON *speed_rpm = cJSON_GetObjectItemCaseSensitive(root, "speed_rpm");
    if (cJSON_IsNumber(speed_rpm)) {
        out_frame->speed_rpm = (float)speed_rpm->valuedouble;
    }

    cJSON *crc_fail = cJSON_GetObjectItemCaseSensitive(root, "crc_fail");
    if (cJSON_IsNumber(crc_fail)) {
        out_frame->crc_fail = crc_fail->valueint;
    }

    parse_u64_field(root, "scan_t0_us", &out_frame->scan_t0_us);
    parse_u64_field(root, "scan_t1_us", &out_frame->scan_t1_us);
    parse_u64_field(root, "scan_period_us", &out_frame->scan_period_us);

    uint16_t count = 0;
    cJSON *point = NULL;
    cJSON_ArrayForEach(point, points) {
        if (count >= AROUNDER_MAX_POINTS_PER_SCAN) {
            break;
        }

        if (!cJSON_IsArray(point) || cJSON_GetArraySize(point) < 3) {
            continue;
        }

        cJSON *angle = cJSON_GetArrayItem(point, 0);
        cJSON *range = cJSON_GetArrayItem(point, 1);
        cJSON *intensity = cJSON_GetArrayItem(point, 2);
        if (!cJSON_IsNumber(angle) || !cJSON_IsNumber(range) || !cJSON_IsNumber(intensity)) {
            continue;
        }

        out_frame->points[count].angle_deg = (float)angle->valuedouble;
        out_frame->points[count].range_mm = (float)range->valuedouble;

        int raw_intensity = intensity->valueint;
        if (raw_intensity < 0) {
            raw_intensity = 0;
        }
        if (raw_intensity > 255) {
            raw_intensity = 255;
        }
        out_frame->points[count].intensity = (uint8_t)raw_intensity;
        count++;
    }

    out_frame->point_count = count;
    cJSON_Delete(root);
    return ESP_OK;
}