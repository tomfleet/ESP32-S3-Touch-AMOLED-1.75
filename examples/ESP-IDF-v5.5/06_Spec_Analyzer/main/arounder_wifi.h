#pragma once

#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

esp_err_t arounder_wifi_connect(void);
void arounder_wifi_get_status(char *ssid, size_t ssid_len, char *ip, size_t ip_len, bool *connected);