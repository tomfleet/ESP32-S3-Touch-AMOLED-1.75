#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "arounder_config.h"
#include "lidar_receiver.h"
#include "circular_renderer.h"

static const char *TAG = "renderer";
static const float PI_F = 3.1416f;

static lv_draw_buf_t s_canvas_draw_buf;
static void *s_canvas_buf;
static lv_obj_t *s_canvas;
static uint32_t s_last_rendered_frames_accepted;
static uint32_t s_render_count;
static TickType_t s_last_render_log_tick;

static void draw_point(lv_layer_t *layer, int x, int y, lv_color_t color)
{
    int radius = AROUNDER_POINT_RADIUS_PX;
    lv_area_t point_area = {
        .x1 = x - radius,
        .y1 = y - radius,
        .x2 = x + radius,
        .y2 = y + radius,
    };

    lv_draw_rect_dsc_t dsc;
    lv_draw_rect_dsc_init(&dsc);
    dsc.bg_color = color;
    dsc.bg_opa = LV_OPA_COVER;

    lv_draw_rect(layer, &dsc, &point_area);
}

static void draw_line(lv_layer_t *layer, int x1, int y1, int x2, int y2, lv_color_t color, lv_opa_t opa)
{
    lv_draw_line_dsc_t dsc;
    lv_draw_line_dsc_init(&dsc);
    dsc.base.layer = layer;
    dsc.color = color;
    dsc.opa = opa;
    dsc.width = 1;
    dsc.p1.x = x1;
    dsc.p1.y = y1;
    dsc.p2.x = x2;
    dsc.p2.y = y2;

    lv_draw_line(layer, &dsc);
}

static void render_frame(lv_layer_t *layer, const lidar_scan_frame_t *frame)
{
    int cx = AROUNDER_CANVAS_WIDTH / 2;
    int cy = AROUNDER_CANVAS_HEIGHT / 2;
    int max_r = (AROUNDER_CANVAS_WIDTH < AROUNDER_CANVAS_HEIGHT ? AROUNDER_CANVAS_WIDTH : AROUNDER_CANVAS_HEIGHT) / 2 - 2;

    float range_span = (float)(AROUNDER_RANGE_MAX_MM - AROUNDER_RANGE_MIN_MM);
    if (range_span <= 0.0f) {
        return;
    }

    uint16_t step = 1;
    if (frame->point_count > AROUNDER_MAX_POINTS_PER_RENDER && AROUNDER_MAX_POINTS_PER_RENDER > 0) {
        step = (uint16_t)((frame->point_count + AROUNDER_MAX_POINTS_PER_RENDER - 1) / AROUNDER_MAX_POINTS_PER_RENDER);
    }

    int prev_x = 0;
    int prev_y = 0;
    uint8_t prev_brightness = 0;
    float first_angle_deg = 0.0f;
    float last_angle_deg = 0.0f;
    int first_x = 0;
    int first_y = 0;
    uint8_t first_brightness = 0;
    bool has_prev = false;

    for (uint16_t i = 0; i < frame->point_count; i += step) {
        const lidar_point_t *point = &frame->points[i];

        if (point->range_mm < AROUNDER_RANGE_MIN_MM || point->range_mm > AROUNDER_RANGE_MAX_MM) {
            continue;
        }

        float angle_deg = point->angle_deg;
        float theta_deg = AROUNDER_ANGLE_CLOCKWISE ? (AROUNDER_ANGLE_ZERO_DEG - angle_deg) : (AROUNDER_ANGLE_ZERO_DEG + angle_deg);
        float theta = theta_deg * (PI_F / 180.0f);

        float rn = ((float)point->range_mm - (float)AROUNDER_RANGE_MIN_MM) / range_span;
        if (rn < 0.0f) {
            rn = 0.0f;
        }
        if (rn > 1.0f) {
            rn = 1.0f;
        }

        float rp = rn * (float)max_r;
        int x = (int)lroundf((float)cx - rp * cosf(theta));
        int y = (int)lroundf((float)cy - rp * sinf(theta));

        uint8_t brightness = AROUNDER_USE_INTENSITY_BRIGHTNESS ? point->intensity : 255;

        if (!has_prev) {
            has_prev = true;
            first_angle_deg = point->angle_deg;
            first_x = x;
            first_y = y;
            first_brightness = brightness;
        } else {
            int dx = x - prev_x;
            int dy = y - prev_y;
            int adx = dx >= 0 ? dx : -dx;
            int ady = dy >= 0 ? dy : -dy;
            int max_delta = adx > ady ? adx : ady;
            int seg_steps = max_delta / 3;
            if (seg_steps < 1) {
                seg_steps = 1;
            }
            if (seg_steps > 24) {
                seg_steps = 24;
            }

            for (int s = 0; s <= seg_steps; s++) {
                int xi = prev_x + (dx * s) / seg_steps;
                int yi = prev_y + (dy * s) / seg_steps;
                uint8_t bi = (uint8_t)(prev_brightness + ((int)(brightness - prev_brightness) * s) / seg_steps);
                uint8_t fill_v = (uint8_t)(bi / 4);
                draw_line(layer, cx, cy, xi, yi, lv_color_make(fill_v, fill_v, fill_v), LV_OPA_40);
            }

            uint8_t edge_v = (uint8_t)((brightness + prev_brightness) / 2);
            draw_line(layer, prev_x, prev_y, x, y, lv_color_make(edge_v, edge_v, edge_v), LV_OPA_COVER);
        }

        draw_point(layer, x, y, lv_color_make(brightness, brightness, brightness));

        last_angle_deg = point->angle_deg;
        prev_x = x;
        prev_y = y;
        prev_brightness = brightness;
    }

    if (has_prev && frame->point_count >= 3) {
        float span = last_angle_deg - first_angle_deg;
        if (span < 0.0f) {
            span = -span;
        }
        if (span > 300.0f) {
            draw_line(layer, prev_x, prev_y, first_x, first_y,
                      lv_color_make((uint8_t)((prev_brightness + first_brightness) / 2),
                                    (uint8_t)((prev_brightness + first_brightness) / 2),
                                    (uint8_t)((prev_brightness + first_brightness) / 2)),
                      LV_OPA_80);
        }
    }
}

static void render_timer_cb(lv_timer_t *timer)
{
    lv_obj_t *canvas = (lv_obj_t *)lv_timer_get_user_data(timer);

    lidar_stats_t stats = {0};
    lidar_receiver_get_stats(&stats);
    if (stats.frames_accepted == s_last_rendered_frames_accepted) {
        return;
    }

    static EXT_RAM_BSS_ATTR lidar_scan_frame_t frame;
    if (!lidar_receiver_get_latest_frame(&frame)) {
        return;
    }
    s_last_rendered_frames_accepted = stats.frames_accepted;

    lv_layer_t layer;
    lv_canvas_init_layer(canvas, &layer);

    if (AROUNDER_PERSIST_ALPHA == 0) {
        lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_COVER);
    } else {
        lv_canvas_fill_bg(canvas, lv_color_black(), (lv_opa_t)AROUNDER_PERSIST_ALPHA);
    }
    render_frame(&layer, &frame);

    lv_canvas_finish_layer(canvas, &layer);

    s_render_count++;
    TickType_t now = xTaskGetTickCount();
    if ((now - s_last_render_log_tick) >= pdMS_TO_TICKS(AROUNDER_STATS_LOG_PERIOD_MS)) {
        s_last_render_log_tick = now;
        ESP_LOGI(TAG,
                 "render frames=%lu accepted=%lu scan_id=%lu points=%u",
                 (unsigned long)s_render_count,
                 (unsigned long)stats.frames_accepted,
                 (unsigned long)frame.scan_id,
                 frame.point_count);
    }
}

esp_err_t arounder_renderer_start(void)
{
    lv_display_t *disp = bsp_display_start();
    if (disp == NULL) {
        return ESP_FAIL;
    }

    uint32_t buf_size = LV_DRAW_BUF_SIZE(AROUNDER_CANVAS_WIDTH, AROUNDER_CANVAS_HEIGHT, LV_COLOR_FORMAT_RGB565);
    s_canvas_buf = heap_caps_aligned_alloc(LV_DRAW_BUF_ALIGN, buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_canvas_buf == NULL) {
        ESP_LOGE(TAG, "Canvas allocation failed (%lu bytes)", (unsigned long)buf_size);
        return ESP_ERR_NO_MEM;
    }

    if (lv_draw_buf_init(&s_canvas_draw_buf, AROUNDER_CANVAS_WIDTH, AROUNDER_CANVAS_HEIGHT,
                         LV_COLOR_FORMAT_RGB565, LV_STRIDE_AUTO, s_canvas_buf, buf_size) != LV_RESULT_OK) {
        ESP_LOGE(TAG, "lv_draw_buf_init failed");
        return ESP_FAIL;
    }
    lv_draw_buf_set_flag(&s_canvas_draw_buf, LV_IMAGE_FLAGS_MODIFIABLE);

    bsp_display_lock(-1);
    s_canvas = lv_canvas_create(lv_screen_active());
    lv_obj_set_size(s_canvas, AROUNDER_CANVAS_WIDTH, AROUNDER_CANVAS_HEIGHT);
    lv_obj_center(s_canvas);
    lv_canvas_set_draw_buf(s_canvas, &s_canvas_draw_buf);

    lv_canvas_fill_bg(s_canvas, lv_color_black(), LV_OPA_COVER);

    lv_timer_create(render_timer_cb, 1000 / AROUNDER_RENDER_FPS, s_canvas);
    bsp_display_unlock();

    ESP_LOGI(TAG, "Renderer ready (%dx%d @ %d fps)", AROUNDER_CANVAS_WIDTH, AROUNDER_CANVAS_HEIGHT, AROUNDER_RENDER_FPS);
    return ESP_OK;
}