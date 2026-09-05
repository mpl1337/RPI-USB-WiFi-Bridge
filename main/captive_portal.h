#pragma once

#include "esp_err.h"
#include "esp_netif.h"

esp_err_t captive_portal_configure_dhcp(esp_netif_t *ap_netif);
esp_err_t captive_portal_start_dns(void);
void captive_portal_stop_dns(void);
