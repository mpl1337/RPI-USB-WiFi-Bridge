#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    char ssid[33];
    int8_t rssi;
    bool secure;
} wifi_bridge_ap_t;

void wifi_bridge_start(void);
bool wifi_bridge_is_connected(void);
bool wifi_bridge_get_status(char *ip, size_t ip_len, int8_t *rssi);
bool wifi_bridge_is_provisioning(void);
esp_err_t wifi_bridge_start_provisioning(void);
void wifi_bridge_get_provisioning_ssid(char *ssid, size_t ssid_len);
esp_err_t wifi_bridge_scan(wifi_bridge_ap_t *aps, size_t max_aps, size_t *count);
esp_err_t wifi_bridge_save_credentials(const char *ssid, const char *password);
