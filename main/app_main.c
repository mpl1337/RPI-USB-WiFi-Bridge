#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_app_desc.h"
#include "nvs_flash.h"
#include "bridge_buffers.h"
#include "bridge_stats.h"
#include "wifi_bridge.h"
#include "tcp_bridge.h"
#include "usb_cdc_bridge.h"
#include "http_bridge.h"
#include "firmware_update.h"

static const char *TAG = "bridge";

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(err);
    }

    const esp_app_desc_t *app_desc = esp_app_get_description();
    ESP_LOGI(TAG, "ESP32-S3 USB CDC <-> WiFi/TCP bridge v%s starting",
             (app_desc != NULL && app_desc->version[0] != '\0') ? app_desc->version : "?");

    bridge_buffers_init();
    bridge_stats_init();
    wifi_bridge_start();
    usb_cdc_bridge_start();
    firmware_update_start();
    tcp_bridge_start();
    http_bridge_start();

    ESP_LOGI(TAG, "Ready. TCP port: %d", CONFIG_BRIDGE_TCP_PORT);
}
