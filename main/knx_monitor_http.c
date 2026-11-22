#include "knx_monitor_http.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "knx_usb_monitor.h"

static const char *TAG = "knx_http";

/* main/web/knx.html, embedded by EMBED_TXTFILES in CMakeLists.txt */
extern const char knx_html_start[] asm("_binary_web_knx_html_start");

static esp_err_t knx_page_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, knx_html_start);
}

static esp_err_t knx_status_handler(httpd_req_t *req)
{
    knx_usb_status_t s = {0};
    knx_usb_monitor_get_status(&s);
    char json[480];
    int n = snprintf(json, sizeof(json),
        "{\"connected\":%s,\"bus_connected\":%s,\"bus_status_known\":%s,"
        "\"vid\":%u,\"pid\":%u,\"active_emi\":%u,\"last_seq\":%" PRIu32 ","
        "\"captured\":%" PRIu32 ",\"group_telegrams\":%" PRIu32 ","
        "\"busmonitor_telegrams\":%" PRIu32 ",\"malformed_reports\":%" PRIu32 ","
        "\"disconnects\":%" PRIu32 ",\"uptime_us\":%" PRIu64 "}",
        s.connected ? "true" : "false",
        s.bus_connected ? "true" : "false",
        s.bus_status_known ? "true" : "false",
        s.vid, s.pid, s.active_emi, s.last_seq,
        s.captured, s.group_telegrams, s.busmonitor_telegrams,
        s.malformed_reports, s.disconnects, s.uptime_us);
    if (n < 0 || n >= (int)sizeof(json)) return ESP_FAIL;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, n);
}

static esp_err_t knx_events_handler(httpd_req_t *req)
{
    uint32_t after = 0;
    uint32_t limit = 64;
    char query[100];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char value[20];
        if (httpd_query_key_value(query, "after", value, sizeof(value)) == ESP_OK) {
            char *end;
            unsigned long parsed = strtoul(value, &end, 10);
            if (end != value && *end == '\0') after = (uint32_t)parsed;
        }
        if (httpd_query_key_value(query, "limit", value, sizeof(value)) == ESP_OK) {
            char *end;
            unsigned long parsed = strtoul(value, &end, 10);
            if (end != value && *end == '\0' && parsed >= 1 && parsed <= 64) {
                limit = (uint32_t)parsed;
            }
        }
    }

    knx_usb_status_t status = {0};
    knx_usb_monitor_get_status(&status);
    uint32_t oldest = status.last_seq >= KNX_USB_HISTORY_SIZE
        ? status.last_seq - KNX_USB_HISTORY_SIZE + 1U : 1U;
    uint32_t missed = 0;
    if (after < oldest - 1U) {
        missed = (oldest - 1U) - after;
        after = oldest - 1U;
    }
    if (after > status.last_seq) after = status.last_seq;

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    char line[1024];
    int n = snprintf(line, sizeof(line),
        "{\"last_seq\":%" PRIu32 ",\"missed\":%" PRIu32 ",\"events\":[",
        status.last_seq, missed);
    if (n < 0 || n >= (int)sizeof(line)) return ESP_FAIL;
    esp_err_t err = httpd_resp_send_chunk(req, line, n);
    if (err != ESP_OK) return err;

    bool first = true;
    uint32_t latest_returned = after;
    for (uint32_t seq = after + 1U;
         seq <= status.last_seq && seq > 0 && limit > 0;
         ++seq, --limit) {
        knx_usb_event_t e;
        if (!knx_usb_monitor_get_event(seq, &e)) {
            /* Overrun while streaming: client will receive explicit gap. */
            continue;
        }
        char hex[KNX_USB_RAW_MAX * 2U + 1U];
        static const char digits[] = "0123456789ABCDEF";
        for (size_t i = 0; i < e.raw_len; ++i) {
            hex[i * 2] = digits[e.raw[i] >> 4];
            hex[i * 2 + 1] = digits[e.raw[i] & 0x0F];
        }
        hex[e.raw_len * 2U] = '\0';
        n = snprintf(line, sizeof(line),
            "%s{\"seq\":%" PRIu32 ",\"received_us\":%" PRIu64 ","
            "\"code\":%u,\"source\":%u,\"destination\":%u,"
            "\"addresses_valid\":%s,\"group\":%s,"
            "\"ctrl1\":%u,\"ctrl2\":%u,\"cemi\":\"%s\"}",
            first ? "" : ",", e.seq, e.received_us,
            e.message_code, e.source, e.destination,
            e.addresses_valid ? "true" : "false",
            e.group_address ? "true" : "false", e.ctrl1, e.ctrl2, hex);
        if (n < 0 || n >= (int)sizeof(line)) return ESP_FAIL;
        err = httpd_resp_send_chunk(req, line, n);
        if (err != ESP_OK) return err;
        first = false;
        latest_returned = seq;
    }
    n = snprintf(line, sizeof(line), "],\"next_after\":%" PRIu32 "}",
                 latest_returned);
    if (n < 0 || n >= (int)sizeof(line)) return ESP_FAIL;
    err = httpd_resp_send_chunk(req, line, n);
    if (err != ESP_OK) return err;
    return httpd_resp_send_chunk(req, NULL, 0);
}

void knx_monitor_http_register(httpd_handle_t server)
{
    const httpd_uri_t page = {
        .uri = "/knx", .method = HTTP_GET, .handler = knx_page_handler
    };
    const httpd_uri_t status = {
        .uri = "/api/knx/status", .method = HTTP_GET, .handler = knx_status_handler
    };
    const httpd_uri_t events = {
        .uri = "/api/knx/events", .method = HTTP_GET, .handler = knx_events_handler
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &page));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &status));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &events));
    ESP_LOGI(TAG, "KNX monitor: /knx");
}
