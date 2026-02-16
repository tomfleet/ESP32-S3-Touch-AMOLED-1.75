#include <string.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <unistd.h>
#include <errno.h>
#include "arpa/inet.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"
#include "lidar_receiver.h"
#include "lidar_protocol.h"
#include "arounder_config.h"

static const char *TAG = "lidar_rx";

static SemaphoreHandle_t s_lock;
static TaskHandle_t s_task_handle;
static StaticSemaphore_t s_lock_buffer;

static EXT_RAM_BSS_ATTR lidar_scan_frame_t s_latest_frame;
static EXT_RAM_BSS_ATTR lidar_scan_frame_t s_rx_frame;
static bool s_latest_frame_valid;
static lidar_stats_t s_stats;
static uint32_t s_last_seen_scan_id = UINT32_MAX;
static TickType_t s_last_stats_log_tick;
static TickType_t s_last_accept_tick;
static uint32_t s_last_stats_raw;
static uint32_t s_last_stats_frames;
static char s_last_sender_ip[16];
static uint16_t s_last_sender_port;
static EXT_RAM_BSS_ATTR uint8_t s_rx_buf[AROUNDER_RX_BUF_BYTES];

static void maybe_log_receiver_stats(void)
{
    TickType_t now = xTaskGetTickCount();
    if ((now - s_last_stats_log_tick) < pdMS_TO_TICKS(AROUNDER_STATS_LOG_PERIOD_MS)) {
        return;
    }

    TickType_t prev_tick = s_last_stats_log_tick;
    s_last_stats_log_tick = now;

    uint32_t rx_hz_milli = 0;
    uint32_t frame_hz_milli = 0;
    uint32_t elapsed_ms = (prev_tick == 0) ? 0 : (uint32_t)pdTICKS_TO_MS(now - prev_tick);
    if (elapsed_ms > 0) {
        uint32_t delta_raw = s_stats.packets_seen_raw - s_last_stats_raw;
        uint32_t delta_frames = s_stats.frames_accepted - s_last_stats_frames;
        rx_hz_milli = (delta_raw * 1000UL * 1000UL) / elapsed_ms;
        frame_hz_milli = (delta_frames * 1000UL * 1000UL) / elapsed_ms;
    }

    s_last_stats_raw = s_stats.packets_seen_raw;
    s_last_stats_frames = s_stats.frames_accepted;

    ESP_LOGI(TAG,
             "stats raw=%lu frames=%lu bad_json=%lu missing=%lu dup_old=%lu gaps=%lu last_scan=%lu rx_hz=%lu.%03lu frame_hz=%lu.%03lu sender=%s:%u",
             (unsigned long)s_stats.packets_seen_raw,
             (unsigned long)s_stats.frames_accepted,
             (unsigned long)s_stats.packets_bad_json,
             (unsigned long)s_stats.packets_missing_required_fields,
             (unsigned long)s_stats.packets_dup_or_old_scan,
             (unsigned long)s_stats.scan_id_gaps,
             (unsigned long)s_last_seen_scan_id,
             (unsigned long)(rx_hz_milli / 1000UL),
             (unsigned long)(rx_hz_milli % 1000UL),
             (unsigned long)(frame_hz_milli / 1000UL),
             (unsigned long)(frame_hz_milli % 1000UL),
             s_last_sender_ip[0] ? s_last_sender_ip : "-",
             s_last_sender_port);
}

static void publish_frame(const lidar_scan_frame_t *frame)
{
    if (frame == NULL) {
        return;
    }

    uint32_t accepted_count;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    memcpy(&s_latest_frame, frame, sizeof(s_latest_frame));
    s_latest_frame_valid = true;
    s_stats.frames_accepted++;
    accepted_count = s_stats.frames_accepted;
    xSemaphoreGive(s_lock);

    if (accepted_count <= 3 || (accepted_count % 100) == 0) {
        TickType_t now = xTaskGetTickCount();
        uint32_t delta_ms = 0;
        if (s_last_accept_tick != 0) {
            delta_ms = (uint32_t)pdTICKS_TO_MS(now - s_last_accept_tick);
        }
        s_last_accept_tick = now;
        ESP_LOGI(TAG,
                 "frame accepted #%lu scan_id=%lu points=%u source=%s dt=%lums",
                 (unsigned long)accepted_count,
                 (unsigned long)frame->scan_id,
                 frame->point_count,
                 frame->source,
                 (unsigned long)delta_ms);
    }
}

static void note_scan_progress(const lidar_scan_frame_t *frame)
{
    if (frame == NULL) {
        return;
    }

    if (s_last_seen_scan_id != UINT32_MAX && frame->scan_id > (s_last_seen_scan_id + 1)) {
        s_stats.scan_id_gaps += (frame->scan_id - s_last_seen_scan_id - 1);
    }
    s_last_seen_scan_id = frame->scan_id;
}

static void receiver_task(void *arg)
{
    (void)arg;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket");
        vTaskDelete(NULL);
        return;
    }

    struct timeval tv = {
        .tv_sec = 0,
        .tv_usec = AROUNDER_SOCKET_RECV_TIMEOUT_MS * 1000,
    };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    int recv_buf_size = AROUNDER_SOCKET_RCVBUF_BYTES;
    int rc = setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &recv_buf_size, sizeof(recv_buf_size));
    if (rc != 0) {
        ESP_LOGW(TAG, "setsockopt(SO_RCVBUF=%d) failed errno=%d", recv_buf_size, errno);
    }

    int actual_recv_buf_size = 0;
    socklen_t actual_recv_buf_len = sizeof(actual_recv_buf_size);
    rc = getsockopt(sock, SOL_SOCKET, SO_RCVBUF, &actual_recv_buf_size, &actual_recv_buf_len);
    if (rc == 0) {
        ESP_LOGI(TAG, "socket SO_RCVBUF requested=%d actual=%d", recv_buf_size, actual_recv_buf_size);
    } else {
        ESP_LOGW(TAG, "getsockopt(SO_RCVBUF) failed errno=%d", errno);
    }

    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(AROUNDER_BIND_PORT),
        .sin_addr.s_addr = inet_addr(AROUNDER_BIND_IP),
    };

    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) != 0) {
        ESP_LOGE(TAG, "Bind failed on %s:%d", AROUNDER_BIND_IP, AROUNDER_BIND_PORT);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Listening on udp://%s:%d", AROUNDER_BIND_IP, AROUNDER_BIND_PORT);

    while (1) {
        struct sockaddr_in src_addr = {0};
        socklen_t src_len = sizeof(src_addr);
        ssize_t received = recvfrom(sock, s_rx_buf, sizeof(s_rx_buf), 0, (struct sockaddr *)&src_addr, &src_len);
        if (received <= 0) {
            maybe_log_receiver_stats();
            continue;
        }

        s_stats.packets_seen_raw++;

        while (1) {
            struct sockaddr_in drain_src_addr = {0};
            socklen_t drain_src_len = sizeof(drain_src_addr);
            ssize_t drained = recvfrom(sock,
                                       s_rx_buf,
                                       sizeof(s_rx_buf),
                                       MSG_DONTWAIT,
                                       (struct sockaddr *)&drain_src_addr,
                                       &drain_src_len);
            if (drained <= 0) {
                break;
            }

            src_addr = drain_src_addr;
            src_len = drain_src_len;
            received = drained;
            s_stats.packets_seen_raw++;
        }

        const char *sender_ip = inet_ntoa(src_addr.sin_addr);
        if (sender_ip != NULL) {
            strlcpy(s_last_sender_ip, sender_ip, sizeof(s_last_sender_ip));
        } else {
            s_last_sender_ip[0] = '\0';
        }
        s_last_sender_port = ntohs(src_addr.sin_port);

        esp_err_t err = lidar_protocol_parse_json_datagram(s_rx_buf, (uint16_t)received, &s_rx_frame);
        if (err == ESP_ERR_INVALID_RESPONSE) {
            s_stats.packets_bad_json++;
            continue;
        }
        if (err == ESP_ERR_INVALID_STATE) {
            s_stats.packets_missing_required_fields++;
            continue;
        }
        if (err != ESP_OK) {
            continue;
        }

        note_scan_progress(&s_rx_frame);
        publish_frame(&s_rx_frame);
        maybe_log_receiver_stats();
    }
}

esp_err_t lidar_receiver_start(void)
{
    if (s_task_handle != NULL) {
        return ESP_OK;
    }

    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutexStatic(&s_lock_buffer);
        if (s_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    BaseType_t ok = xTaskCreate(receiver_task,
                                "lidar_rx",
                                AROUNDER_RX_TASK_STACK_BYTES,
                                NULL,
                                AROUNDER_RX_TASK_PRIORITY,
                                &s_task_handle);
    if (ok != pdPASS) {
        s_task_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

bool lidar_receiver_get_latest_frame(lidar_scan_frame_t *out_frame)
{
    if (out_frame == NULL) {
        return false;
    }

    if (s_lock == NULL) {
        return false;
    }

    bool valid;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    valid = s_latest_frame_valid;
    if (valid) {
        memcpy(out_frame, &s_latest_frame, sizeof(lidar_scan_frame_t));
    }
    xSemaphoreGive(s_lock);

    return valid;
}

void lidar_receiver_get_stats(lidar_stats_t *out_stats)
{
    if (out_stats == NULL) {
        return;
    }

    if (s_lock == NULL) {
        memset(out_stats, 0, sizeof(*out_stats));
        return;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out_stats = s_stats;
    xSemaphoreGive(s_lock);
}