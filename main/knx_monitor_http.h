#pragma once

#include "esp_http_server.h"

/* Register /knx, /api/knx/status and /api/knx/events. */
void knx_monitor_http_register(httpd_handle_t server);
