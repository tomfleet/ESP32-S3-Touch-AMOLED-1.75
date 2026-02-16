#pragma once

#include "esp_err.h"
#include <stdint.h>

esp_err_t arounder_renderer_start(void);
uint16_t arounder_renderer_get_alert_radius_mm(void);