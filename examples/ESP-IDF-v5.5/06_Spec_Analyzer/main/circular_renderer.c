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
static lv_obj_t *s_radius_popup;
static lv_obj_t *s_radius_arc;
static lv_obj_t *s_radius_value_label;
static lv_obj_t *s_radius_dismiss_touch;
static lv_timer_t *s_radius_popup_timer;
static uint32_t s_last_rendered_frames_accepted;
static uint32_t s_render_count;
static TickType_t s_last_render_log_tick;
static uint16_t s_alert_radius_mm = 1500;
static float s_recent_max_ranges_mm[AROUNDER_DYNAMIC_MAX_WINDOW_FRAMES];
static uint16_t s_recent_max_count;
static uint16_t s_recent_max_write_index;
static int16_t s_poly_x[AROUNDER_MAX_POINTS_PER_RENDER + 2];
static int16_t s_poly_y[AROUNDER_MAX_POINTS_PER_RENDER + 2];
static uint8_t s_poly_brightness[AROUNDER_MAX_POINTS_PER_RENDER + 2];
static int16_t s_fill_x[AROUNDER_MAX_POINTS_PER_RENDER + 2];
static int16_t s_fill_y[AROUNDER_MAX_POINTS_PER_RENDER + 2];
static int16_t s_scanline_intersections[AROUNDER_MAX_POINTS_PER_RENDER + 2];

static void update_radius_value_label(void);

static void hide_radius_popup(void)
{
    if (s_radius_popup != NULL) {
        lv_obj_add_flag(s_radius_popup, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_radius_popup_timer != NULL) {
        lv_timer_pause(s_radius_popup_timer);
    }
}

static void arm_radius_popup_autodismiss(void)
{
    if (s_radius_popup_timer == NULL) {
        return;
    }

    lv_timer_set_period(s_radius_popup_timer, 5000);
    lv_timer_set_repeat_count(s_radius_popup_timer, 1);
    lv_timer_resume(s_radius_popup_timer);
    lv_timer_reset(s_radius_popup_timer);
}

static void show_radius_popup(void)
{
    if (s_radius_popup == NULL || s_radius_arc == NULL) {
        return;
    }

    lv_arc_set_value(s_radius_arc, (int32_t)s_alert_radius_mm);
    update_radius_value_label();
    lv_obj_clear_flag(s_radius_popup, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_radius_popup);
    arm_radius_popup_autodismiss();
}

static void radius_popup_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    hide_radius_popup();
}

static void radius_popup_interaction_cb(lv_event_t *e)
{
    (void)e;
    if (s_radius_popup != NULL && !lv_obj_has_flag(s_radius_popup, LV_OBJ_FLAG_HIDDEN)) {
        arm_radius_popup_autodismiss();
    }
}

static void radius_popup_long_press_cb(lv_event_t *e)
{
    (void)e;
    hide_radius_popup();
}

static void update_radius_value_label(void)
{
    if (s_radius_value_label == NULL || s_radius_arc == NULL) {
        return;
    }

    s_alert_radius_mm = (uint16_t)lv_arc_get_value(s_radius_arc);
    lv_label_set_text_fmt(s_radius_value_label, "Radius: %u mm", (unsigned int)s_alert_radius_mm);
}

static void radius_arc_event_cb(lv_event_t *e)
{
    (void)e;
    update_radius_value_label();
    radius_popup_interaction_cb(e);
}

static void radius_popup_close_cb(lv_event_t *e)
{
    (void)e;
    hide_radius_popup();
}

static void canvas_long_press_cb(lv_event_t *e)
{
    (void)e;
    if (s_radius_popup == NULL) {
        return;
    }

    if (!lv_obj_has_flag(s_radius_popup, LV_OBJ_FLAG_HIDDEN)) {
        hide_radius_popup();
        return;
    }

    show_radius_popup();
}

static void create_radius_popup(lv_obj_t *parent)
{
    if (parent == NULL) {
        return;
    }

    int popup_size = (AROUNDER_CANVAS_WIDTH * 9) / 10;
    if (popup_size > AROUNDER_CANVAS_HEIGHT) {
        popup_size = AROUNDER_CANVAS_HEIGHT;
    }
    int arc_size = (AROUNDER_CANVAS_WIDTH * 8) / 10;
    if (arc_size > (popup_size - 52)) {
        arc_size = popup_size - 52;
    }
    if (arc_size < 140) {
        arc_size = 140;
    }

    s_radius_popup = lv_obj_create(parent);
    lv_obj_set_size(s_radius_popup, popup_size, popup_size);
    lv_obj_center(s_radius_popup);
    lv_obj_set_style_bg_opa(s_radius_popup, LV_OPA_90, 0);
    lv_obj_set_style_border_width(s_radius_popup, 1, 0);
    lv_obj_set_style_radius(s_radius_popup, 12, 0);
    lv_obj_add_flag(s_radius_popup, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(s_radius_popup, radius_popup_interaction_cb, LV_EVENT_PRESSED, NULL);

    lv_obj_t *title = lv_label_create(s_radius_popup);
    lv_label_set_text(title, "Proximity Radius");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    s_radius_arc = lv_arc_create(s_radius_popup);
    lv_obj_set_size(s_radius_arc, arc_size, arc_size);
    lv_obj_align(s_radius_arc, LV_ALIGN_CENTER, 0, 8);
    lv_arc_set_range(s_radius_arc, AROUNDER_RANGE_MIN_MM, AROUNDER_RANGE_MAX_MM);
    lv_arc_set_value(s_radius_arc, (int32_t)s_alert_radius_mm);
    lv_obj_add_event_cb(s_radius_arc, radius_arc_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(s_radius_arc, radius_popup_interaction_cb, LV_EVENT_PRESSED, NULL);

    s_radius_dismiss_touch = lv_obj_create(s_radius_popup);
    lv_obj_set_size(s_radius_dismiss_touch, 200, 200);
    lv_obj_align(s_radius_dismiss_touch, LV_ALIGN_CENTER, 0, 8);
    lv_obj_set_style_radius(s_radius_dismiss_touch, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(s_radius_dismiss_touch, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_radius_dismiss_touch, 1, 0);
    lv_obj_set_style_border_opa(s_radius_dismiss_touch, LV_OPA_20, 0);
    lv_obj_add_flag(s_radius_dismiss_touch, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_radius_dismiss_touch, radius_popup_long_press_cb, LV_EVENT_LONG_PRESSED, NULL);
    lv_obj_add_event_cb(s_radius_dismiss_touch, radius_popup_interaction_cb, LV_EVENT_PRESSED, NULL);

    s_radius_value_label = lv_label_create(s_radius_popup);
    lv_obj_align(s_radius_value_label, LV_ALIGN_BOTTOM_MID, 0, -50);
    update_radius_value_label();

    lv_obj_t *close_btn = lv_button_create(s_radius_popup);
    lv_obj_set_size(close_btn, 90, 34);
    lv_obj_align(close_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_add_event_cb(close_btn, radius_popup_close_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(close_btn, radius_popup_interaction_cb, LV_EVENT_PRESSED, NULL);

    lv_obj_t *close_label = lv_label_create(close_btn);
    lv_label_set_text(close_label, "Close");
    lv_obj_center(close_label);

    if (s_radius_popup_timer == NULL) {
        s_radius_popup_timer = lv_timer_create(radius_popup_timer_cb, 5000, NULL);
    }
    if (s_radius_popup_timer != NULL) {
        lv_timer_pause(s_radius_popup_timer);
        lv_timer_set_repeat_count(s_radius_popup_timer, 1);
    }
}

static float frame_max_distance_mm(const lidar_scan_frame_t *frame)
{
    if (frame == NULL) {
        return 0.0f;
    }

    float max_distance = 0.0f;
    for (uint16_t i = 0; i < frame->point_count; i++) {
        float range_mm = frame->points[i].range_mm;
        if (range_mm < (float)AROUNDER_RANGE_MIN_MM || range_mm > (float)AROUNDER_RANGE_MAX_MM) {
            continue;
        }
        if (range_mm > max_distance) {
            max_distance = range_mm;
        }
    }

    return max_distance;
}

static void push_recent_max_distance(float frame_max_mm)
{
    if (frame_max_mm <= 0.0f) {
        return;
    }

    if (s_recent_max_count < AROUNDER_DYNAMIC_MAX_WINDOW_FRAMES) {
        s_recent_max_ranges_mm[s_recent_max_count++] = frame_max_mm;
        return;
    }

    s_recent_max_ranges_mm[s_recent_max_write_index] = frame_max_mm;
    s_recent_max_write_index = (uint16_t)((s_recent_max_write_index + 1U) % AROUNDER_DYNAMIC_MAX_WINDOW_FRAMES);
}

static float tracked_max_distance_mm(void)
{
    float max_distance = 0.0f;
    for (uint16_t i = 0; i < s_recent_max_count; i++) {
        if (s_recent_max_ranges_mm[i] > max_distance) {
            max_distance = s_recent_max_ranges_mm[i];
        }
    }
    return max_distance;
}

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

static void draw_alert_radius_ring(lv_layer_t *layer, int cx, int cy, int radius_px)
{

    return; // Disabled for now as it causes too much performance overhead. Will be reworked later.
    
    if (radius_px <= 1) {
        return;
    }

    lv_draw_arc_dsc_t dsc;
    lv_draw_arc_dsc_init(&dsc);
    dsc.base.layer = layer;
    dsc.color = lv_color_make(96, 96, 96);
    dsc.opa = LV_OPA_70;
    dsc.width = 1;
    dsc.center.x = cx;
    dsc.center.y = cy;
    dsc.radius = (uint16_t)radius_px;
    dsc.start_angle = 0;
    dsc.end_angle = 360;

    lv_draw_arc(layer, &dsc);
}

static void fill_polygon_scanline(lv_layer_t *layer,
                                  const int16_t *px,
                                  const int16_t *py,
                                  uint16_t count,
                                  lv_color_t color,
                                  lv_opa_t opa)
{
    if (count < 3) {
        return;
    }

    int y_min = py[0];
    int y_max = py[0];
    for (uint16_t i = 1; i < count; i++) {
        if (py[i] < y_min) {
            y_min = py[i];
        }
        if (py[i] > y_max) {
            y_max = py[i];
        }
    }

    if (y_min < 0) {
        y_min = 0;
    }
    if (y_max >= AROUNDER_CANVAS_HEIGHT) {
        y_max = AROUNDER_CANVAS_HEIGHT - 1;
    }

    for (int y = y_min; y <= y_max; y++) {
        uint16_t n_intersections = 0;

        for (uint16_t i = 0; i < count; i++) {
            uint16_t j = (uint16_t)((i + 1U) % count);
            int y1 = py[i];
            int y2 = py[j];

            if (y1 == y2) {
                continue;
            }

            int low_y = y1 < y2 ? y1 : y2;
            int high_y = y1 > y2 ? y1 : y2;
            if (y < low_y || y >= high_y) {
                continue;
            }

            int x1 = px[i];
            int x2 = px[j];
            int x = x1 + (int)(((int32_t)(y - y1) * (int32_t)(x2 - x1)) / (int32_t)(y2 - y1));
            if (n_intersections < (AROUNDER_MAX_POINTS_PER_RENDER + 2)) {
                s_scanline_intersections[n_intersections++] = (int16_t)x;
            }
        }

        if (n_intersections < 2) {
            continue;
        }

        for (uint16_t i = 1; i < n_intersections; i++) {
            int16_t key = s_scanline_intersections[i];
            int j = (int)i - 1;
            while (j >= 0 && s_scanline_intersections[j] > key) {
                s_scanline_intersections[j + 1] = s_scanline_intersections[j];
                j--;
            }
            s_scanline_intersections[j + 1] = key;
        }

        for (uint16_t i = 0; i + 1 < n_intersections; i += 2) {
            int x_start = s_scanline_intersections[i];
            int x_end = s_scanline_intersections[i + 1];
            if (x_end < x_start) {
                int tmp = x_start;
                x_start = x_end;
                x_end = tmp;
            }
            if (x_end < 0 || x_start >= AROUNDER_CANVAS_WIDTH) {
                continue;
            }
            if (x_start < 0) {
                x_start = 0;
            }
            if (x_end >= AROUNDER_CANVAS_WIDTH) {
                x_end = AROUNDER_CANVAS_WIDTH - 1;
            }
            draw_line(layer, x_start, y, x_end, y, color, opa);
        }
    }
}

static void render_frame(lv_layer_t *layer, const lidar_scan_frame_t *frame, float effective_max_range_mm)
{
    int cx = AROUNDER_CANVAS_WIDTH / 2;
    int cy = AROUNDER_CANVAS_HEIGHT / 2;
    int max_r = (AROUNDER_CANVAS_WIDTH < AROUNDER_CANVAS_HEIGHT ? AROUNDER_CANVAS_WIDTH : AROUNDER_CANVAS_HEIGHT) / 2 - 2;

    float clamped_max_range = effective_max_range_mm;
    if (clamped_max_range > (float)AROUNDER_RANGE_MAX_MM) {
        clamped_max_range = (float)AROUNDER_RANGE_MAX_MM;
    }
    if (clamped_max_range < ((float)AROUNDER_RANGE_MIN_MM + 1.0f)) {
        clamped_max_range = (float)AROUNDER_RANGE_MAX_MM;
    }

    float range_span = clamped_max_range - (float)AROUNDER_RANGE_MIN_MM;
    if (range_span <= 0.0f) {
        return;
    }

    uint16_t step = 1;
    if (frame->point_count > AROUNDER_MAX_POINTS_PER_RENDER && AROUNDER_MAX_POINTS_PER_RENDER > 0) {
        step = (uint16_t)((frame->point_count + AROUNDER_MAX_POINTS_PER_RENDER - 1) / AROUNDER_MAX_POINTS_PER_RENDER);
    }

    uint16_t poly_count = 0;
    uint32_t brightness_sum = 0;

    for (uint16_t i = 0; i < frame->point_count; i += step) {
        const lidar_point_t *point = &frame->points[i];

        if (point->range_mm < AROUNDER_RANGE_MIN_MM || point->range_mm > clamped_max_range) {
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

        if (poly_count < (AROUNDER_MAX_POINTS_PER_RENDER + 2)) {
            s_poly_x[poly_count] = (int16_t)x;
            s_poly_y[poly_count] = (int16_t)y;
            s_poly_brightness[poly_count] = brightness;
            poly_count++;
            brightness_sum += brightness;
        }

    }

    float alert_rn = ((float)s_alert_radius_mm - (float)AROUNDER_RANGE_MIN_MM) / range_span;
    if (alert_rn < 0.0f) {
        alert_rn = 0.0f;
    }
    if (alert_rn > 1.0f) {
        alert_rn = 1.0f;
    }
    int alert_r_px = (int)lroundf(alert_rn * (float)max_r);

    //draw_alert_radius_ring(layer, cx, cy, alert_r_px);

    if (poly_count < 2) {
        return;
    }

    if (AROUNDER_ENABLE_FILLED_AREA && poly_count >= 3) {
        uint16_t fill_stride = (AROUNDER_FILL_STRIDE == 0) ? 1U : (uint16_t)AROUNDER_FILL_STRIDE;
        uint16_t fill_count = 0;
        for (uint16_t i = 0; i < poly_count && fill_count < (AROUNDER_MAX_POINTS_PER_RENDER + 2); i += fill_stride) {
            s_fill_x[fill_count] = s_poly_x[i];
            s_fill_y[fill_count] = s_poly_y[i];
            fill_count++;
        }
        if (fill_count >= 3) {
            uint8_t avg_brightness = (uint8_t)(brightness_sum / poly_count);
            uint8_t fill_v = (uint8_t)(avg_brightness / 2U);
            lv_color_t fill_color = lv_color_make(fill_v, fill_v, fill_v);
            fill_polygon_scanline(layer, s_fill_x, s_fill_y, fill_count, fill_color, (lv_opa_t)AROUNDER_FILL_OPA);
        }
    }

    for (uint16_t i = 0; i < poly_count; i++) {
        uint16_t j = (uint16_t)((i + 1U) % poly_count);
        uint8_t edge_v = (uint8_t)((s_poly_brightness[i] + s_poly_brightness[j]) / 2U);
        lv_color_t edge_color = lv_color_make(edge_v, edge_v, edge_v);
        draw_line(layer, s_poly_x[i], s_poly_y[i], s_poly_x[j], s_poly_y[j], edge_color, (j == 0U) ? LV_OPA_80 : LV_OPA_COVER);

        if (AROUNDER_ENABLE_LIGHT_SHADING && AROUNDER_SHADE_STRIDE > 0 && ((i + 1U) % (uint16_t)AROUNDER_SHADE_STRIDE) == 0U) {
            uint8_t shade_v = (uint8_t)(edge_v / 3U);
            lv_color_t shade_color = lv_color_make(shade_v, shade_v, shade_v);
            draw_line(layer, cx, cy, s_poly_x[i], s_poly_y[i], shade_color, (lv_opa_t)AROUNDER_SHADE_OPA);
        }

        lv_color_t point_color = lv_color_make(s_poly_brightness[i], s_poly_brightness[i], s_poly_brightness[i]);
        draw_point(layer, s_poly_x[i], s_poly_y[i], point_color);
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

    float effective_max_range_mm = (float)AROUNDER_RANGE_MAX_MM;
    if (AROUNDER_DYNAMIC_MAX_TRACKING) {
        float current_frame_max = frame_max_distance_mm(&frame);
        push_recent_max_distance(current_frame_max);

        float recent_max = tracked_max_distance_mm();
        if (recent_max > (float)AROUNDER_RANGE_MIN_MM) {
            effective_max_range_mm = recent_max + (float)AROUNDER_DYNAMIC_MAX_HEADROOM_MM;
            if (effective_max_range_mm > (float)AROUNDER_RANGE_MAX_MM) {
                effective_max_range_mm = (float)AROUNDER_RANGE_MAX_MM;
            }
        }
    }

    s_last_rendered_frames_accepted = stats.frames_accepted;

    lv_layer_t layer;
    lv_canvas_init_layer(canvas, &layer);

    if (AROUNDER_PERSIST_ALPHA == 0) {
        lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_COVER);
    } else {
        lv_canvas_fill_bg(canvas, lv_color_black(), (lv_opa_t)AROUNDER_PERSIST_ALPHA);
    }
    render_frame(&layer, &frame, effective_max_range_mm);

    lv_canvas_finish_layer(canvas, &layer);

    s_render_count++;
    TickType_t now = xTaskGetTickCount();
    if ((now - s_last_render_log_tick) >= pdMS_TO_TICKS(AROUNDER_STATS_LOG_PERIOD_MS)) {
        s_last_render_log_tick = now;
        ESP_LOGI(TAG,
                 "render frames=%lu accepted=%lu scan_id=%lu points=%u scale_max=%.0f",
                 (unsigned long)s_render_count,
                 (unsigned long)stats.frames_accepted,
                 (unsigned long)frame.scan_id,
                 frame.point_count,
                 (double)effective_max_range_mm);
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
    lv_obj_add_flag(s_canvas, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_canvas, canvas_long_press_cb, LV_EVENT_LONG_PRESSED, NULL);

    lv_canvas_fill_bg(s_canvas, lv_color_black(), LV_OPA_COVER);

    create_radius_popup(lv_screen_active());

    lv_timer_create(render_timer_cb, 1000 / AROUNDER_RENDER_FPS, s_canvas);
    bsp_display_unlock();

    ESP_LOGI(TAG, "Renderer ready (%dx%d @ %d fps)", AROUNDER_CANVAS_WIDTH, AROUNDER_CANVAS_HEIGHT, AROUNDER_RENDER_FPS);
    return ESP_OK;
}

uint16_t arounder_renderer_get_alert_radius_mm(void)
{
    return s_alert_radius_mm;
}