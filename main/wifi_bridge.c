#include "wifi_bridge.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "captive_portal.h"

static const char *TAG = "wifi";
static EventGroupHandle_t s_wifi_events;
static char s_ip[16] = "0.0.0.0";
static char s_ap_ssid[33] = "RPI-USB-WiFi-Bridge";
static bool s_provisioning;
static unsigned s_disconnect_count;
static esp_netif_t *s_ap_netif;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_PROVISIONING_RETRY_THRESHOLD 8
#define WIFI_SSID_TEXT_MAX (sizeof(((wifi_sta_config_t *)0)->ssid) + 1)

static size_t copy_wifi_field(char *dst, size_t dst_size,
                              const uint8_t *src, size_t src_size)
{
    if (dst == NULL || dst_size == 0 || src == NULL) {
        return 0;
    }

    size_t len = 0;
    while (len < src_size && src[len] != '\0') {
        len++;
    }
    if (len >= dst_size) {
        len = dst_size - 1;
    }

    memcpy(dst, src, len);
    dst[len] = '\0';
    return len;
}

static bool wifi_config_has_valid_ssid(const wifi_config_t *config)
{
    if (config == NULL || config->sta.ssid[0] == '\0' || config->sta.ssid[0] == 0xff) {
        return false;
    }

    char ssid[WIFI_SSID_TEXT_MAX] = {0};
    copy_wifi_field(ssid, sizeof(ssid), config->sta.ssid, sizeof(config->sta.ssid));
    return ssid[0] != '\0';
}

static void build_ap_ssid(void)
{
    uint8_t mac[6] = {0};
    if (esp_wifi_get_mac(WIFI_IF_AP, mac) == ESP_OK) {
        snprintf(s_ap_ssid, sizeof(s_ap_ssid), "RPI-USB-WiFi-%02X%02X%02X",
                 mac[3], mac[4], mac[5]);
    }
}

static esp_err_t start_provisioning_ap(void)
{
    if (s_provisioning) {
        return ESP_OK;
    }

    build_ap_ssid();

    wifi_config_t ap_config = {0};
    size_t ssid_len = strlen(s_ap_ssid);
    memcpy(ap_config.ap.ssid, s_ap_ssid, ssid_len);
    ap_config.ap.ssid_len = ssid_len;
    ap_config.ap.channel = 1;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_OPEN;

    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    if (err != ESP_OK) {
        return err;
    }

    s_provisioning = true;
    ESP_LOGW(TAG, "WiFi setup AP active: SSID '%s', open http://192.168.4.1", s_ap_ssid);
    return ESP_OK;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
        if (s_provisioning) {
            esp_err_t dhcp_err = captive_portal_configure_dhcp(s_ap_netif);
            if (dhcp_err != ESP_OK) {
                ESP_LOGW(TAG, "Captive portal DHCP setup failed: %s", esp_err_to_name(dhcp_err));
            }
            esp_err_t dns_err = captive_portal_start_dns();
            if (dns_err != ESP_OK) {
                ESP_LOGW(TAG, "Captive DNS start failed: %s", esp_err_to_name(dns_err));
            }
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STOP) {
        captive_portal_stop_dns();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        if (!s_provisioning) {
            esp_wifi_connect();
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_events, WIFI_CONNECTED_BIT);
        strlcpy(s_ip, "0.0.0.0", sizeof(s_ip));
        if (s_provisioning) {
            return;
        }
        s_disconnect_count++;
        if (s_disconnect_count >= WIFI_PROVISIONING_RETRY_THRESHOLD) {
            esp_err_t err = start_provisioning_ap();
            if (err == ESP_OK) {
                return;
            }
            ESP_LOGE(TAG, "Could not start WiFi setup AP: %s", esp_err_to_name(err));
        }
        ESP_LOGW(TAG, "WiFi disconnected, reconnecting");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *evt = (const ip_event_got_ip_t *)event_data;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&evt->ip_info.ip));
        s_disconnect_count = 0;
        ESP_LOGI(TAG, "WiFi connected, IP=%s", s_ip);
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
        if (s_provisioning) {
            s_provisioning = false;
            esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Could not stop WiFi setup AP: %s", esp_err_to_name(err));
            }
        }
    }
}

bool wifi_bridge_is_connected(void)
{
    if (s_wifi_events == NULL) {
        return false;
    }
    return (xEventGroupGetBits(s_wifi_events) & WIFI_CONNECTED_BIT) != 0;
}

bool wifi_bridge_get_status(char *ip, size_t ip_len, int8_t *rssi)
{
    bool connected = wifi_bridge_is_connected();
    if (ip != NULL && ip_len > 0) {
        strlcpy(ip, s_ip, ip_len);
    }
    if (rssi != NULL) {
        *rssi = 0;
        if (connected) {
            wifi_ap_record_t ap = {0};
            if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
                *rssi = ap.rssi;
            }
        }
    }
    return connected;
}

bool wifi_bridge_is_provisioning(void)
{
    return s_provisioning;
}

esp_err_t wifi_bridge_start_provisioning(void)
{
    return start_provisioning_ap();
}

void wifi_bridge_get_provisioning_ssid(char *ssid, size_t ssid_len)
{
    if (ssid == NULL || ssid_len == 0) {
        return;
    }
    strlcpy(ssid, s_ap_ssid, ssid_len);
}

static int compare_ap_rssi(const void *a, const void *b)
{
    const wifi_bridge_ap_t *aa = (const wifi_bridge_ap_t *)a;
    const wifi_bridge_ap_t *bb = (const wifi_bridge_ap_t *)b;
    return (int)bb->rssi - (int)aa->rssi;
}

esp_err_t wifi_bridge_scan(wifi_bridge_ap_t *aps, size_t max_aps, size_t *count)
{
    if (aps == NULL || max_aps == 0 || count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_provisioning) {
        return ESP_ERR_INVALID_STATE;
    }

    *count = 0;
    esp_err_t err = esp_wifi_scan_start(NULL, true);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t found = 0;
    err = esp_wifi_scan_get_ap_num(&found);
    if (err != ESP_OK) {
        return err;
    }
    if (found == 0) {
        return ESP_OK;
    }

    if (found > 32) {
        found = 32;
    }
    wifi_ap_record_t records[32] = {0};
    uint16_t record_count = found;
    err = esp_wifi_scan_get_ap_records(&record_count, records);
    if (err != ESP_OK) {
        return err;
    }

    for (uint16_t i = 0; i < record_count && *count < max_aps; ++i) {
        if (records[i].ssid[0] == '\0') {
            continue;
        }

        char ssid[33] = {0};
        copy_wifi_field(ssid, sizeof(ssid), records[i].ssid, sizeof(records[i].ssid));
        bool duplicate = false;
        for (size_t j = 0; j < *count; ++j) {
            if (strcmp(aps[j].ssid, ssid) == 0) {
                if (records[i].rssi > aps[j].rssi) {
                    aps[j].rssi = records[i].rssi;
                    aps[j].secure = records[i].authmode != WIFI_AUTH_OPEN;
                }
                duplicate = true;
                break;
            }
        }
        if (duplicate) {
            continue;
        }

        strlcpy(aps[*count].ssid, ssid, sizeof(aps[*count].ssid));
        aps[*count].rssi = records[i].rssi;
        aps[*count].secure = records[i].authmode != WIFI_AUTH_OPEN;
        (*count)++;
    }

    qsort(aps, *count, sizeof(aps[0]), compare_ap_rssi);
    return ESP_OK;
}

esp_err_t wifi_bridge_save_credentials(const char *ssid, const char *password)
{
    if (ssid == NULL || password == NULL || ssid[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    size_t ssid_len = strlen(ssid);
    size_t password_len = strlen(password);
    if (ssid_len > sizeof(((wifi_sta_config_t *)0)->ssid) ||
        password_len > sizeof(((wifi_sta_config_t *)0)->password) - 1) {
        return ESP_ERR_INVALID_SIZE;
    }

    wifi_config_t config = {0};
    memcpy(config.sta.ssid, ssid, ssid_len);
    memcpy(config.sta.password, password, password_len);
    config.sta.password[password_len] = '\0';
    config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &config);
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "Stored WiFi credentials in NVS for SSID '%s'", ssid);
    return ESP_OK;
}

void wifi_bridge_start(void)
{
    s_wifi_events = xEventGroupCreate();
    configASSERT(s_wifi_events != NULL);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    s_ap_netif = esp_netif_create_default_wifi_ap();
    configASSERT(s_ap_netif != NULL);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {0};
    esp_err_t stored_err = esp_wifi_get_config(WIFI_IF_STA, &wifi_config);
    bool use_stored = stored_err == ESP_OK && wifi_config_has_valid_ssid(&wifi_config);

    if (use_stored) {
        char ssid[WIFI_SSID_TEXT_MAX] = {0};
        copy_wifi_field(ssid, sizeof(ssid), wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid));
        ESP_LOGI(TAG, "Using WiFi credentials stored in NVS (SSID '%s')", ssid);
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
        return;
    }

    if (stored_err != ESP_OK && stored_err != ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "Could not read stored WiFi configuration: %s",
                 esp_err_to_name(stored_err));
    }

    ESP_ERROR_CHECK(start_provisioning_ap());
    ESP_ERROR_CHECK(esp_wifi_start());
}
