#pragma once

#include <stdbool.h>

#define AROUNDER_BIND_IP                     "0.0.0.0"
#define AROUNDER_BIND_PORT                   8765
#define AROUNDER_RX_BUF_BYTES                4096
#define AROUNDER_SOCKET_RECV_TIMEOUT_MS      50
#define AROUNDER_SOCKET_RCVBUF_BYTES         (256 * 1024)
#define AROUNDER_RX_TASK_STACK_BYTES         (6 * 1024)
#define AROUNDER_RX_TASK_PRIORITY            10

#define AROUNDER_WIFI_SSID                   "smorter"
#define AROUNDER_WIFI_PASSWORD               "surface-electric-famous"
#define AROUNDER_WIFI_MAX_RETRY              10
#define AROUNDER_WIFI_CONNECT_TIMEOUT_MS     20000

#define AROUNDER_MAX_POINTS_PER_SCAN         4096

#define AROUNDER_CANVAS_WIDTH                466
#define AROUNDER_CANVAS_HEIGHT               466
#define AROUNDER_RENDER_FPS                  10
#define AROUNDER_IDLE_UPDATE_MS              500
#define AROUNDER_MAX_POINTS_PER_RENDER       800
#define AROUNDER_POINT_RADIUS_PX             1
#define AROUNDER_PERSIST_ALPHA               0

#define AROUNDER_RANGE_MIN_MM                100
#define AROUNDER_RANGE_MAX_MM                6000

#define AROUNDER_ANGLE_ZERO_DEG              90.0f
#define AROUNDER_ANGLE_CLOCKWISE             true

#define AROUNDER_USE_INTENSITY_BRIGHTNESS    true

#define AROUNDER_STATS_LOG_PERIOD_MS         5000