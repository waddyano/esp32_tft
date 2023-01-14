#pragma once

#include "esp_netif_types.h"

extern char wifi_ssid[33];

extern void wifi_init_sta(const char *hostname, bool with_bluetooth);
esp_netif_t *wifi_get_netif();
extern void wifi_reconnect();
