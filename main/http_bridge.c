#include "http_bridge.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#include "usb_cdc_bridge.h"
#include "tcp_bridge.h"
#include "wifi_bridge.h"
#include "bridge_buffers.h"
#include "bridge_stats.h"
#include "firmware_update.h"
#include "web_ui.h"
#include "sdkconfig.h"

#ifndef CONFIG_HTTPD_WS_POST_HANDSHAKE_CB_SUPPORT
#error "CONFIG_HTTPD_WS_POST_HANDSHAKE_CB_SUPPORT is required on ESP-IDF 6.1+"
#endif

static const char *TAG = "http";
static httpd_handle_t s_server;
#define WS_TX_CHUNK 2048
#define WS_RX_MAX   2048
#define WS_TX_IDLE_DELAY_MS 2
#define WS_LRU_TOUCH_MS 2000


typedef struct {
    int fd;
    bool active;
    bool in_flight;
    uint32_t generation;
    int scheduled_fd;
    uint32_t scheduled_generation;
    size_t tx_len;
    uint8_t tx_data[WS_TX_CHUNK];
} ws_client_t;

static ws_client_t s_ws_clients[BRIDGE_WS_MAX_CLIENTS];
static portMUX_TYPE s_ws_clients_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_ws_lru_touch_pending;

static bool ws_fd_is_live(httpd_handle_t server, int fd)
{
    return server != NULL && fd >= 0 &&
           httpd_ws_get_fd_info(server, fd) == HTTPD_WS_CLIENT_WEBSOCKET;
}

static size_t ws_client_count(void)
{
    size_t count = 0;
    portENTER_CRITICAL(&s_ws_clients_mux);
    for (size_t i = 0; i < BRIDGE_WS_MAX_CLIENTS; ++i) {
        if (s_ws_clients[i].active) {
            count++;
        }
    }
    portEXIT_CRITICAL(&s_ws_clients_mux);
    return count;
}

static int ws_find_slot_by_fd(int fd)
{
    int slot = -1;
    portENTER_CRITICAL(&s_ws_clients_mux);
    for (size_t i = 0; i < BRIDGE_WS_MAX_CLIENTS; ++i) {
        if (s_ws_clients[i].active && s_ws_clients[i].fd == fd) {
            slot = (int)i;
            break;
        }
    }
    portEXIT_CRITICAL(&s_ws_clients_mux);
    return slot;
}

static void ws_sync_usb_capture_state(void)
{
    usb_cdc_bridge_set_ws_client_state(ws_client_count() > 0);
}

static int ws_client_activate(int fd)
{
    int slot = -1;

    portENTER_CRITICAL(&s_ws_clients_mux);
    for (size_t i = 0; i < BRIDGE_WS_MAX_CLIENTS; ++i) {
        if (s_ws_clients[i].active && s_ws_clients[i].fd == fd) {
            slot = (int)i;
            break;
        }
    }
    if (slot < 0) {
        for (size_t i = 0; i < BRIDGE_WS_MAX_CLIENTS; ++i) {

            if (!s_ws_clients[i].active && !s_ws_clients[i].in_flight) {
                ws_client_t *client = &s_ws_clients[i];
                client->fd = fd;
                client->active = true;
                client->generation++;
                client->scheduled_fd = -1;
                client->scheduled_generation = 0;
                client->tx_len = 0;
                slot = (int)i;
                break;
            }
        }
    }
    portEXIT_CRITICAL(&s_ws_clients_mux);

    if (slot >= 0) {
        bridge_usb_to_ws_client_activate((size_t)slot);
        ws_sync_usb_capture_state();
    }
    return slot;
}

static bool ws_client_deactivate_slot(size_t slot, int expected_fd)
{
    if (slot >= BRIDGE_WS_MAX_CLIENTS) {
        return false;
    }

    bool changed = false;
    portENTER_CRITICAL(&s_ws_clients_mux);
    ws_client_t *client = &s_ws_clients[slot];
    if (client->active && (expected_fd < 0 || client->fd == expected_fd)) {
        client->active = false;
        client->fd = -1;
        changed = true;
    }
    portEXIT_CRITICAL(&s_ws_clients_mux);

    if (changed) {
        bridge_usb_to_ws_client_deactivate(slot);
        ws_sync_usb_capture_state();
    }
    return changed;
}

static bool ws_client_deactivate_fd(int fd)
{
    bool changed = false;
    for (size_t i = 0; i < BRIDGE_WS_MAX_CLIENTS; ++i) {
        if (ws_client_deactivate_slot(i, fd)) {
            changed = true;
            break;
        }
    }
    return changed;
}

static void ws_prune_stale_clients(httpd_handle_t server)
{
    int stale_fds[BRIDGE_WS_MAX_CLIENTS];
    size_t stale_count = 0;

    portENTER_CRITICAL(&s_ws_clients_mux);
    for (size_t i = 0; i < BRIDGE_WS_MAX_CLIENTS; ++i) {
        if (s_ws_clients[i].active) {
            stale_fds[stale_count++] = s_ws_clients[i].fd;
        }
    }
    portEXIT_CRITICAL(&s_ws_clients_mux);

    for (size_t i = 0; i < stale_count; ++i) {
        if (!ws_fd_is_live(server, stale_fds[i])) {
            if (ws_client_deactivate_fd(stale_fds[i])) {
                ESP_LOGI(TAG, "Pruned stale Browser WebSocket fd=%d", stale_fds[i]);
            }
        }
    }
}

static void http_session_close(httpd_handle_t hd, int sockfd)
{
    (void)hd;
    if (ws_client_deactivate_fd(sockfd)) {
        ESP_LOGI(TAG, "Browser WebSocket session closed (fd=%d, clients=%u)",
                 sockfd, (unsigned)ws_client_count());
    }


    close(sockfd);
}

bool http_bridge_ws_is_connected(void)
{
    return ws_client_count() > 0;
}

static void ws_send_work(void *arg)
{
    ws_client_t *client = (ws_client_t *)arg;
    if (client == NULL) {
        return;
    }

    int fd = -1;
    uint32_t generation = 0;
    size_t len = 0;
    bool should_send = false;

    portENTER_CRITICAL(&s_ws_clients_mux);
    if (client->in_flight) {
        fd = client->scheduled_fd;
        generation = client->scheduled_generation;
        len = client->tx_len;
        should_send = client->active && client->fd == fd && client->generation == generation;
    }
    portEXIT_CRITICAL(&s_ws_clients_mux);

    esp_err_t err = ESP_ERR_INVALID_STATE;
    if (should_send && ws_fd_is_live(s_server, fd)) {
        httpd_ws_frame_t frame = {0};
        frame.type = HTTPD_WS_TYPE_BINARY;
        frame.payload = client->tx_data;
        frame.len = len;


        err = httpd_ws_send_frame_async(s_server, fd, &frame);
        if (err == ESP_OK) {
            bridge_stats_add_ws_tx(len);
            (void)httpd_sess_update_lru_counter(s_server, fd);
        } else {
            bridge_stats_add_dropped_usb_to_ws(len);
            ESP_LOGW(TAG, "WebSocket TX failed fd=%d: %s", fd, esp_err_to_name(err));
            (void)ws_client_deactivate_fd(fd);
            (void)httpd_sess_trigger_close(s_server, fd);
        }
    }

    portENTER_CRITICAL(&s_ws_clients_mux);

    client->in_flight = false;
    client->tx_len = 0;
    client->scheduled_fd = -1;
    client->scheduled_generation = 0;
    portEXIT_CRITICAL(&s_ws_clients_mux);
}

static void ws_lru_touch_work(void *arg)
{
    (void)arg;
    int fds[BRIDGE_WS_MAX_CLIENTS];
    size_t count = 0;

    portENTER_CRITICAL(&s_ws_clients_mux);
    for (size_t i = 0; i < BRIDGE_WS_MAX_CLIENTS; ++i) {
        if (s_ws_clients[i].active) {
            fds[count++] = s_ws_clients[i].fd;
        }
    }
    portEXIT_CRITICAL(&s_ws_clients_mux);

    for (size_t i = 0; i < count; ++i) {
        if (ws_fd_is_live(s_server, fds[i])) {
            (void)httpd_sess_update_lru_counter(s_server, fds[i]);
        }
    }

    portENTER_CRITICAL(&s_ws_clients_mux);
    s_ws_lru_touch_pending = false;
    portEXIT_CRITICAL(&s_ws_clients_mux);
}

static void ws_tx_task(void *arg)
{
    (void)arg;
    TickType_t last_lru_touch = xTaskGetTickCount();

    while (true) {
        bool did_work = false;

        if (s_server != NULL) {
            for (size_t i = 0; i < BRIDGE_WS_MAX_CLIENTS; ++i) {
                ws_client_t *client = &s_ws_clients[i];
                bool ready = false;
                int fd = -1;
                uint32_t generation = 0;

                portENTER_CRITICAL(&s_ws_clients_mux);
                if (client->active && !client->in_flight) {
                    ready = true;
                    fd = client->fd;
                    generation = client->generation;
                }
                portEXIT_CRITICAL(&s_ws_clients_mux);

                if (!ready) {
                    continue;
                }

                size_t len = bridge_usb_to_ws_pop_client(i, client->tx_data,
                                                         sizeof(client->tx_data), 0);
                if (len == 0) {
                    continue;
                }

                bool queued_owner = false;
                portENTER_CRITICAL(&s_ws_clients_mux);
                if (client->active && !client->in_flight &&
                    client->fd == fd && client->generation == generation) {
                    client->in_flight = true;
                    client->scheduled_fd = fd;
                    client->scheduled_generation = generation;
                    client->tx_len = len;
                    queued_owner = true;
                }
                portEXIT_CRITICAL(&s_ws_clients_mux);

                if (!queued_owner) {
                    continue;
                }

                esp_err_t err = httpd_queue_work(s_server, ws_send_work, client);
                if (err != ESP_OK) {
                    bridge_stats_add_dropped_usb_to_ws(len);
                    portENTER_CRITICAL(&s_ws_clients_mux);
                    client->in_flight = false;
                    client->tx_len = 0;
                    client->scheduled_fd = -1;
                    client->scheduled_generation = 0;
                    portEXIT_CRITICAL(&s_ws_clients_mux);
                    ESP_LOGW(TAG, "Could not queue WebSocket TX fd=%d: %s", fd, esp_err_to_name(err));
                } else {
                    did_work = true;
                }
            }

            TickType_t now = xTaskGetTickCount();
            if ((now - last_lru_touch) >= pdMS_TO_TICKS(WS_LRU_TOUCH_MS)) {
                bool queue_touch = false;
                portENTER_CRITICAL(&s_ws_clients_mux);
                if (!s_ws_lru_touch_pending) {
                    for (size_t i = 0; i < BRIDGE_WS_MAX_CLIENTS; ++i) {
                        if (s_ws_clients[i].active) {
                            s_ws_lru_touch_pending = true;
                            queue_touch = true;
                            break;
                        }
                    }
                }
                portEXIT_CRITICAL(&s_ws_clients_mux);

                if (queue_touch) {
                    esp_err_t lru_err = httpd_queue_work(s_server, ws_lru_touch_work, NULL);
                    if (lru_err != ESP_OK) {
                        portENTER_CRITICAL(&s_ws_clients_mux);
                        s_ws_lru_touch_pending = false;
                        portEXIT_CRITICAL(&s_ws_clients_mux);
                    }
                }
                last_lru_touch = now;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(did_work ? 1 : WS_TX_IDLE_DELAY_MS));
    }
}

static const char WIFI_SETUP_HTML[] =
"<!doctype html><html lang=\"de\"><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\"><title>RPI USB WiFi Bridge</title>"
"<style>*{box-sizing:border-box}body{margin:0;background:#f3f7f4;color:#183126;font:15px/1.45 system-ui,-apple-system,Segoe UI,sans-serif}main{max-width:560px;margin:7vh auto;padding:20px}.card{background:#fff;border:1px solid #d8e2dc;border-radius:14px;padding:22px;box-shadow:0 10px 28px rgba(25,65,43,.08)}h1{font-size:24px;margin:0 0 6px;color:#123023}.sub{color:#687a70;margin-bottom:22px}.field{margin:14px 0}label{display:block;font-weight:650;margin-bottom:6px}input,select,button{width:100%;font:inherit;border:1px solid #b8c9bf;border-radius:8px;padding:10px 11px;background:#fff;color:#183126}button{cursor:pointer;font-weight:700}.primary{background:#079447;border-color:#079447;color:#fff}.primary:hover{background:#056d36}.scan{margin-top:8px;width:auto;padding:8px 12px}.hint{font-size:13px;color:#687a70}.status{margin-top:14px;padding:10px 12px;border-radius:8px;background:#f3f7f4;min-height:42px}.ok{background:#e9f7ef;color:#056d36}.bad{background:#fceeed;color:#a52d28}</style></head><body><main><div class=\"card\"><h1>RPI USB WiFi Bridge</h1><div class=\"sub\">WLAN einrichten</div>"
"<div class=\"field\"><label for=\"networks\">Gefundene WLANs</label><select id=\"networks\"><option>WLANs werden gesucht…</option></select><button class=\"scan\" type=\"button\" id=\"scan\">Erneut suchen</button></div>"
"<form id=\"form\"><div class=\"field\"><label for=\"ssid\">SSID</label><input id=\"ssid\" name=\"ssid\" maxlength=\"32\" required autocomplete=\"off\"></div><div class=\"field\"><label for=\"password\">WLAN-Passwort</label><input id=\"password\" name=\"password\" type=\"password\" maxlength=\"63\" autocomplete=\"new-password\"><div class=\"hint\">Bei einem offenen WLAN leer lassen.</div></div><button class=\"primary\" type=\"submit\">Speichern und verbinden</button></form><div id=\"status\" class=\"status\">Mit dem gewünschten WLAN verbinden. Die Bridge startet danach automatisch neu.</div></div></main>"
"<script>const q=x=>document.getElementById(x),sel=q('networks'),ssid=q('ssid'),status=q('status');async function scan(){status.className='status';status.textContent='WLANs werden gesucht…';sel.innerHTML='<option>Suche…</option>';try{const r=await fetch('/api/wifi/scan',{cache:'no-store'}),d=await r.json();sel.innerHTML='';for(const n of d.networks){const o=document.createElement('option');o.value=n.ssid;o.textContent=n.ssid+' · '+n.rssi+' dBm'+(n.secure?' · geschützt':' · offen');sel.appendChild(o)}if(!d.networks.length)sel.innerHTML='<option value=\"\">Keine WLANs gefunden</option>';if(sel.value)ssid.value=sel.value;status.textContent='WLAN auswählen oder SSID manuell eingeben.'}catch(e){status.className='status bad';status.textContent='WLAN-Suche fehlgeschlagen. SSID kann manuell eingegeben werden.'}}sel.addEventListener('change',()=>{if(sel.value)ssid.value=sel.value});q('scan').addEventListener('click',scan);q('form').addEventListener('submit',async e=>{e.preventDefault();status.className='status';status.textContent='Zugangsdaten werden gespeichert…';const body=new URLSearchParams({ssid:ssid.value,password:q('password').value});try{const r=await fetch('/api/wifi',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});const t=await r.text();if(!r.ok)throw new Error(t);status.className='status ok';status.textContent='Gespeichert. Die Bridge startet neu und verbindet sich mit dem WLAN.'}catch(e){status.className='status bad';status.textContent=e.message||'Speichern fehlgeschlagen.'}});scan();</script></body></html>";

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    if (wifi_bridge_is_provisioning()) {
        return httpd_resp_sendstr(req, WIFI_SETUP_HTML);
    }
    return httpd_resp_send(req, (const char *)BRIDGE_INDEX_HTML, BRIDGE_INDEX_HTML_LEN);
}

static esp_err_t favicon_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t http_not_found_handler(httpd_req_t *req, httpd_err_code_t err)
{
    (void)err;
    if (wifi_bridge_is_provisioning()) {
        httpd_resp_set_status(req, "303 See Other");
        httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        return httpd_resp_sendstr(req, "WLAN-Einrichtung: http://192.168.4.1/");
    }
    return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
}

static void json_escape(const char *src, char *dst, size_t dst_len)
{
    if (dst_len == 0) {
        return;
    }
    size_t o = 0;
    for (size_t i = 0; src != NULL && src[i] != '\0' && o + 1 < dst_len; ++i) {
        unsigned char c = (unsigned char)src[i];
        if ((c == '"' || c == '\\') && o + 2 < dst_len) {
            dst[o++] = '\\';
            dst[o++] = (char)c;
        } else if (c >= 0x20) {
            dst[o++] = (char)c;
        }
    }
    dst[o] = '\0';
}

static esp_err_t usb_refresh_handler(httpd_req_t *req)
{
    firmware_update_status_t fw = {0};
    firmware_update_get_status(&fw);
    if (fw.busy) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        return httpd_resp_sendstr(req, "USB-Informationen können während eines Firmware-Vorgangs nicht aktualisiert werden.");
    }

    esp_err_t err = usb_cdc_bridge_refresh_device_info(1500);
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    if (err == ESP_OK) {
        if (usb_cdc_bridge_is_connected()) {
            usb_cdc_openknx_info_t info = {0};
            esp_err_t info_err = usb_cdc_bridge_query_openknx_info(&info, 2200);
            if (info_err == ESP_OK) {
                char msg[128];
                snprintf(msg, sizeof(msg), "OK: USB-Gerät aktualisiert · Firmware %s.",
                         info.version[0] ? info.version : "erkannt");
                return httpd_resp_sendstr(req, msg);
            }
            usb_cdc_bridge_clear_openknx_info();
            ESP_LOGW(TAG, "OpenKNX firmware version refresh failed: %s", esp_err_to_name(info_err));
            return httpd_resp_sendstr(req, "OK: USB-Gerät aktualisiert · Firmware-Version nicht verfügbar.");
        }

        usb_cdc_bridge_clear_openknx_info();
        return httpd_resp_sendstr(req, "OK: USB-Gerät aktualisiert · Firmware-Version nur im CDC-Betrieb verfügbar.");
    }
    if (err == ESP_ERR_NOT_FOUND) {
        usb_cdc_bridge_clear_openknx_info();
        httpd_resp_set_status(req, "404 Not Found");
        return httpd_resp_sendstr(req, "Kein passendes RP-MCU-USB-Gerät gefunden.");
    }
    if (err == ESP_ERR_TIMEOUT) {
        httpd_resp_set_status(req, "504 Gateway Timeout");
        return httpd_resp_sendstr(req, "Zeitüberschreitung beim Aktualisieren der USB-Geräteinformationen.");
    }

    httpd_resp_set_status(req, "500 Internal Server Error");
    char msg[128];
    snprintf(msg, sizeof(msg), "USB-Geräteinformationen konnten nicht aktualisiert werden: %s", esp_err_to_name(err));
    return httpd_resp_sendstr(req, msg);
}

static void esp32_manual_restart_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(800));
    esp_restart();
}

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void url_decode(char *text)
{
    char *src = text;
    char *dst = text;
    while (*src != '\0') {
        if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else if (*src == '%' && src[1] != '\0' && src[2] != '\0') {
            int hi = hex_value(src[1]);
            int lo = hex_value(src[2]);
            if (hi >= 0 && lo >= 0) {
                *dst++ = (char)((hi << 4) | lo);
                src += 3;
            } else {
                *dst++ = *src++;
            }
        } else {
            *dst++ = *src++;
        }
    }
    *dst = '\0';
}

static bool parse_wifi_form(char *body, char *ssid, size_t ssid_len,
                            char *password, size_t password_len)
{
    ssid[0] = '\0';
    password[0] = '\0';
    char *saveptr = NULL;
    for (char *item = strtok_r(body, "&", &saveptr); item != NULL;
         item = strtok_r(NULL, "&", &saveptr)) {
        char *eq = strchr(item, '=');
        if (eq == NULL) {
            continue;
        }
        *eq = '\0';
        char *key = item;
        char *value = eq + 1;
        url_decode(key);
        url_decode(value);
        if (strcmp(key, "ssid") == 0) {
            strlcpy(ssid, value, ssid_len);
        } else if (strcmp(key, "password") == 0) {
            strlcpy(password, value, password_len);
        }
    }
    return ssid[0] != '\0';
}

static esp_err_t wifi_setup_start_handler(httpd_req_t *req)
{
    esp_err_t err = wifi_bridge_start_provisioning();
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, esp_err_to_name(err));
    }

    char ssid_raw[33] = {0};
    char ssid[80] = {0};
    wifi_bridge_get_provisioning_ssid(ssid_raw, sizeof(ssid_raw));
    json_escape(ssid_raw, ssid, sizeof(ssid));
    char json[160];
    int n = snprintf(json, sizeof(json),
                     "{\"ssid\":\"%s\",\"ip\":\"192.168.4.1\"}", ssid);
    if (n < 0 || (size_t)n >= sizeof(json)) {
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, n);
}

static esp_err_t wifi_scan_handler(httpd_req_t *req)
{
    if (!wifi_bridge_is_provisioning()) {
        httpd_resp_set_status(req, "403 Forbidden");
        return httpd_resp_sendstr(req, "WiFi setup is not active");
    }

    wifi_bridge_ap_t aps[20] = {0};
    size_t count = 0;
    esp_err_t err = wifi_bridge_scan(aps, 20, &count);
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, esp_err_to_name(err));
    }

    char json[3072];
    size_t used = 0;
    int n = snprintf(json, sizeof(json), "{\"networks\":[");
    if (n < 0) return ESP_FAIL;
    used = (size_t)n;
    for (size_t i = 0; i < count; ++i) {
        char escaped[96];
        json_escape(aps[i].ssid, escaped, sizeof(escaped));
        n = snprintf(json + used, sizeof(json) - used,
                     "%s{\"ssid\":\"%s\",\"rssi\":%d,\"secure\":%s}",
                     i == 0 ? "" : ",", escaped, (int)aps[i].rssi,
                     aps[i].secure ? "true" : "false");
        if (n < 0 || (size_t)n >= sizeof(json) - used) {
            httpd_resp_set_status(req, "500 Internal Server Error");
            return httpd_resp_sendstr(req, "scan JSON overflow");
        }
        used += (size_t)n;
    }
    if (used + 3 >= sizeof(json)) return ESP_FAIL;
    memcpy(json + used, "]}", 3);
    used += 2;

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, used);
}

static esp_err_t wifi_save_handler(httpd_req_t *req)
{
    if (!wifi_bridge_is_provisioning()) {
        httpd_resp_set_status(req, "403 Forbidden");
        return httpd_resp_sendstr(req, "WiFi setup is not active");
    }
    if (req->content_len <= 0 || req->content_len >= 384) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "Ungültige Formulardaten.");
    }

    char body[384];
    size_t received = 0;
    while (received < (size_t)req->content_len) {
        int n = httpd_req_recv(req, body + received,
                               (size_t)req->content_len - received);
        if (n <= 0) {
            httpd_resp_set_status(req, "400 Bad Request");
            return httpd_resp_sendstr(req, "Formulardaten konnten nicht gelesen werden.");
        }
        received += (size_t)n;
    }
    body[received] = '\0';

    char ssid[33];
    char password[64];
    if (!parse_wifi_form(body, ssid, sizeof(ssid), password, sizeof(password))) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "SSID fehlt.");
    }

    esp_err_t err = wifi_bridge_save_credentials(ssid, password);
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "400 Bad Request");
        char msg[128];
        snprintf(msg, sizeof(msg), "WLAN-Daten konnten nicht gespeichert werden: %s", esp_err_to_name(err));
        return httpd_resp_sendstr(req, msg);
    }

    BaseType_t ok = xTaskCreate(esp32_manual_restart_task, "wifi_restart", 2048, NULL, 8, NULL);
    if (ok != pdPASS) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "WLAN-Daten gespeichert, Neustart konnte aber nicht gestartet werden.");
    }

    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, "OK");
}

static esp_err_t esp32_restart_handler(httpd_req_t *req)
{
    firmware_update_status_t fw = {0};
    firmware_update_get_status(&fw);
    if (fw.busy) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        return httpd_resp_sendstr(req, "ESP32 kann während eines Firmware-Vorgangs nicht neu gestartet werden.");
    }

    BaseType_t ok = xTaskCreate(esp32_manual_restart_task, "manual_restart", 2048, NULL, 8, NULL);
    if (ok != pdPASS) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        return httpd_resp_sendstr(req, "Neustart-Task konnte nicht erstellt werden.");
    }

    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, "OK: ESP32 wird neu gestartet.");
}

static esp_err_t status_handler(httpd_req_t *req)
{
    char ip[16];
    int8_t rssi = 0;
    bool wifi = wifi_bridge_get_status(ip, sizeof(ip), &rssi);
    char setup_ssid_raw[33] = {0};
    char setup_ssid[80] = {0};
    wifi_bridge_get_provisioning_ssid(setup_ssid_raw, sizeof(setup_ssid_raw));
    json_escape(setup_ssid_raw, setup_ssid, sizeof(setup_ssid));

    ws_prune_stale_clients(req->handle);
    size_t ws_clients = ws_client_count();
    bool ws = ws_clients > 0;

    usb_cdc_device_info_t dev = {0};
    usb_cdc_bridge_get_device_info(&dev);
    usb_cdc_serial_state_t serial_state = {0};
    usb_cdc_bridge_get_serial_state(&serial_state);
    usb_cdc_diag_t usb_diag = {0};
    usb_cdc_bridge_get_diag(&usb_diag);
    usb_cdc_openknx_info_t openknx = {0};
    usb_cdc_bridge_get_openknx_info(&openknx);
    char manufacturer[136], product[136], serial[136];
    char openknx_version[64], openknx_name[192];
    json_escape(dev.manufacturer, manufacturer, sizeof(manufacturer));
    json_escape(dev.product, product, sizeof(product));
    json_escape(dev.serial, serial, sizeof(serial));
    json_escape(openknx.version, openknx_version, sizeof(openknx_version));
    json_escape(openknx.name, openknx_name, sizeof(openknx_name));
    char usb_diag_event_text[180];
    json_escape(usb_diag.last_event, usb_diag_event_text, sizeof(usb_diag_event_text));

    bridge_stats_snapshot_t st = {0};
    bridge_stats_get(&st);
    uint64_t uptime_s = (uint64_t)(esp_timer_get_time() / 1000000LL);
    size_t heap_free = esp_get_free_heap_size();
    size_t heap_min = esp_get_minimum_free_heap_size();
    size_t heap_largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);

    firmware_update_status_t fw = {0};
    firmware_update_get_status(&fw);
    char fw_message[720];
    json_escape(fw.message, fw_message, sizeof(fw_message));

    const esp_app_desc_t *app_desc = esp_app_get_description();
    const esp_partition_t *running_partition = esp_ota_get_running_partition();
    const esp_partition_t *next_ota_partition = esp_ota_get_next_update_partition(NULL);
    char bridge_version[72], bridge_project[72], bridge_idf[72], bridge_partition[40], bridge_ota_target[40];
    json_escape(app_desc != NULL ? app_desc->version : "", bridge_version, sizeof(bridge_version));
    json_escape(app_desc != NULL ? app_desc->project_name : "", bridge_project, sizeof(bridge_project));
    json_escape(app_desc != NULL ? app_desc->idf_ver : "", bridge_idf, sizeof(bridge_idf));
    json_escape(running_partition != NULL ? running_partition->label : "", bridge_partition, sizeof(bridge_partition));
    json_escape(next_ota_partition != NULL ? next_ota_partition->label : "", bridge_ota_target, sizeof(bridge_ota_target));

    char json[4512];
    int n = snprintf(json, sizeof(json),
        "{\"wifi\":%s,\"ip\":\"%s\",\"rssi\":%d,"
        "\"wifi_setup\":{\"active\":%s,\"ap_ssid\":\"%s\",\"ap_ip\":\"192.168.4.1\"},"
        "\"usb\":%s,\"msc\":%s,\"tcp\":%s,\"tcp_mode\":\"%s\",\"ws\":%s,\"ws_clients\":%u,\"ws_max\":%u,\"tcp_port\":%d,\"uptime_s\":%" PRIu64 ","
        "\"serial\":{\"valid\":%s,\"baud\":%u,\"data_bits\":%u,\"parity\":%u,\"stop\":%u,\"dtr\":%s,\"rts\":%s},"
        "\"usb_debug\":{\"reconnect_paused\":%s,\"reconnect_hold\":%s,\"target_present\":%s,\"open_fail_count\":%u,\"last_open_error\":%d,\"last_open_error_name\":\"%s\",\"connect_count\":%u,\"disconnect_count\":%u,\"event_seq\":%u,\"last_event_ms\":%" PRIu64 ",\"last_event\":\"%s\"},"
        "\"bridge\":{\"version\":\"%s\",\"project\":\"%s\",\"idf\":\"%s\",\"partition\":\"%s\",\"ota\":%s,\"ota_target\":\"%s\",\"ota_size\":%u},"
        "\"device\":{\"valid\":%s,\"vid\":%u,\"pid\":%u,\"bcd_usb\":%u,\"bcd_device\":%u,\"family\":\"%s\","
        "\"class\":%u,\"subclass\":%u,\"protocol\":%u,\"address\":%u,\"speed\":%u,"
        "\"manufacturer\":\"%s\",\"product\":\"%s\",\"serial\":\"%s\"},"
        "\"openknx\":{\"valid\":%s,\"version\":\"%s\",\"app_id\":%u,\"name\":\"%s\"},"
        "\"firmware\":{\"state\":\"%s\",\"busy\":%s,\"msc_ready\":%s,\"staged_ready\":%s,\"bootsel_family\":\"%s\",\"staged_family\":\"%s\",\"staged_family_id\":%u,\"received\":%u,\"total\":%u,"
        "\"blocks\":%u,\"total_blocks\":%u,\"message\":\"%s\"},"
        "\"stats\":{\"usb_rx\":%" PRIu64 ",\"usb_tx\":%" PRIu64 ",\"tcp_rx\":%" PRIu64 ",\"tcp_tx\":%" PRIu64 ","
        "\"ws_rx\":%" PRIu64 ",\"ws_tx\":%" PRIu64 ",\"drop_tcp\":%" PRIu64 ",\"drop_ws\":%" PRIu64 ",\"drop_usb\":%" PRIu64 "},"
        "\"memory\":{\"heap_free\":%u,\"heap_min\":%u,\"heap_largest\":%u}}",
        wifi ? "true" : "false", ip, (int)rssi,
        wifi_bridge_is_provisioning() ? "true" : "false", setup_ssid,
        usb_cdc_bridge_is_connected() ? "true" : "false",
        firmware_update_msc_ready() ? "true" : "false",
        tcp_bridge_is_client_connected() ? "true" : "false",
        tcp_bridge_client_mode_name(),
        ws ? "true" : "false",
        (unsigned)ws_clients, (unsigned)BRIDGE_WS_MAX_CLIENTS,
        CONFIG_BRIDGE_TCP_PORT, uptime_s,
        serial_state.valid ? "true" : "false", (unsigned)serial_state.baud,
        (unsigned)serial_state.data_bits, (unsigned)serial_state.parity_type,
        (unsigned)serial_state.char_format, serial_state.dtr ? "true" : "false",
        serial_state.rts ? "true" : "false",
        usb_diag.reconnect_paused ? "true" : "false",
        usb_diag.reconnect_hold ? "true" : "false",
        usb_diag.normal_target_present ? "true" : "false",
        (unsigned)usb_diag.open_fail_count, (int)usb_diag.last_open_error,
        esp_err_to_name((esp_err_t)usb_diag.last_open_error),
        (unsigned)usb_diag.connect_count, (unsigned)usb_diag.disconnect_count,
        (unsigned)usb_diag.event_seq, usb_diag.last_event_ms, usb_diag_event_text,
        bridge_version, bridge_project, bridge_idf, bridge_partition,
        next_ota_partition != NULL ? "true" : "false", bridge_ota_target,
        next_ota_partition != NULL ? (unsigned)next_ota_partition->size : 0U,
        dev.valid ? "true" : "false", (unsigned)dev.vid, (unsigned)dev.pid,
        (unsigned)dev.bcd_usb, (unsigned)dev.bcd_device, rp_mcu_family_name(dev.family),
        (unsigned)dev.dev_class, (unsigned)dev.dev_subclass, (unsigned)dev.dev_protocol,
        (unsigned)dev.dev_addr, (unsigned)dev.speed,
        manufacturer, product, serial,
        openknx.valid ? "true" : "false", openknx_version, (unsigned)openknx.app_id, openknx_name,
        firmware_update_state_name(fw.state), fw.busy ? "true" : "false", fw.msc_ready ? "true" : "false", fw.staged_ready ? "true" : "false",
        rp_mcu_family_name(fw.bootsel_family), rp_mcu_family_name(fw.staged_family), (unsigned)fw.staged_family_id,
        (unsigned)fw.received_bytes, (unsigned)fw.total_bytes, (unsigned)fw.blocks_written, (unsigned)fw.total_blocks, fw_message,
        st.usb_rx_bytes, st.usb_tx_bytes, st.tcp_rx_bytes, st.tcp_tx_bytes,
        st.ws_rx_bytes, st.ws_tx_bytes, st.dropped_usb_to_tcp,
        st.dropped_usb_to_ws, st.dropped_to_usb,
        (unsigned)heap_free, (unsigned)heap_min, (unsigned)heap_largest);

    if (n < 0 || (size_t)n >= sizeof(json)) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "status JSON overflow");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, n);
}

static esp_err_t stats_reset_handler(httpd_req_t *req)
{
    bridge_stats_reset();
    return httpd_resp_sendstr(req, "OK");
}

static esp_err_t ws_on_connect(httpd_req_t *req)
{
    int fd = httpd_req_to_sockfd(req);
    int slot = ws_client_activate(fd);
    if (slot < 0) {
        ESP_LOGW(TAG, "Browser WebSocket limit reached (%u clients); rejecting fd=%d",
                 (unsigned)BRIDGE_WS_MAX_CLIENTS, fd);
        (void)httpd_sess_trigger_close(req->handle, fd);
        return ESP_OK;
    }

    (void)httpd_sess_update_lru_counter(req->handle, fd);
    ESP_LOGI(TAG, "Browser WebSocket connected (fd=%d, slot=%d, clients=%u/%u)",
             fd, slot, (unsigned)ws_client_count(), (unsigned)BRIDGE_WS_MAX_CLIENTS);
    return ESP_OK;
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    int fd = httpd_req_to_sockfd(req);

    if (ws_find_slot_by_fd(fd) < 0) {
        ESP_LOGW(TAG, "WebSocket RX from untracked fd=%d", fd);
        return ESP_ERR_INVALID_STATE;
    }

    httpd_ws_frame_t frame = {0};
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WebSocket frame length failed fd=%d: %s", fd, esp_err_to_name(err));
        (void)ws_client_deactivate_fd(fd);
        return err;
    }

    if (frame.len == 0) {
        return ESP_OK;
    }
    if (frame.len > WS_RX_MAX) {
        ESP_LOGW(TAG, "WebSocket RX frame too large fd=%d: %u", fd, (unsigned)frame.len);
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t *buf = malloc(frame.len);
    if (buf == NULL) {
        return ESP_ERR_NO_MEM;
    }
    frame.payload = buf;
    err = httpd_ws_recv_frame(req, &frame, frame.len);
    if (err != ESP_OK) {
        free(buf);
        (void)ws_client_deactivate_fd(fd);
        return err;
    }

    (void)httpd_sess_update_lru_counter(req->handle, fd);

    if (frame.type == HTTPD_WS_TYPE_TEXT || frame.type == HTTPD_WS_TYPE_BINARY) {
        bridge_stats_add_ws_rx(frame.len);
        if (usb_cdc_bridge_is_connected()) {
            size_t pushed = bridge_net_to_usb_push(buf, frame.len);
            if (pushed != frame.len) {
                bridge_stats_add_dropped_to_usb(frame.len - pushed);
            }
        } else {
            bridge_stats_add_dropped_to_usb(frame.len);
        }
    }

    free(buf);
    return ESP_OK;
}

void http_bridge_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.stack_size = 12288;
    config.max_uri_handlers = 16;


    config.max_open_sockets = 5;
    config.backlog_conn = 3;
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 5;
    config.send_wait_timeout = 1;
    config.close_fn = http_session_close;

    esp_err_t err = httpd_start(&s_server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server failed: %s", esp_err_to_name(err));
        s_server = NULL;
        return;
    }

    const httpd_uri_t index_uri = {
        .uri = "/", .method = HTTP_GET, .handler = index_handler,
    };
    const httpd_uri_t favicon_uri = {
        .uri = "/favicon.ico", .method = HTTP_GET, .handler = favicon_handler,
    };
    const httpd_uri_t status_uri = {
        .uri = "/api/status", .method = HTTP_GET, .handler = status_handler,
    };
    const httpd_uri_t wifi_setup_uri = {
        .uri = "/api/wifi/setup", .method = HTTP_POST, .handler = wifi_setup_start_handler,
    };
    const httpd_uri_t wifi_scan_uri = {
        .uri = "/api/wifi/scan", .method = HTTP_GET, .handler = wifi_scan_handler,
    };
    const httpd_uri_t wifi_save_uri = {
        .uri = "/api/wifi", .method = HTTP_POST, .handler = wifi_save_handler,
    };
    const httpd_uri_t usb_refresh_uri = {
        .uri = "/api/usb-refresh", .method = HTTP_POST, .handler = usb_refresh_handler,
    };
    const httpd_uri_t restart_uri = {
        .uri = "/api/restart", .method = HTTP_POST, .handler = esp32_restart_handler,
    };
    const httpd_uri_t bootloader_uri = {
        .uri = "/api/bootloader", .method = HTTP_POST, .handler = firmware_update_bootloader_http_handler,
    };
    const httpd_uri_t start_firmware_uri = {
        .uri = "/api/start-firmware", .method = HTTP_POST, .handler = firmware_update_start_firmware_http_handler,
    };
    const httpd_uri_t nuke_uri = {
        .uri = "/api/flash-nuke", .method = HTTP_POST, .handler = firmware_update_nuke_http_handler,
    };
    const httpd_uri_t stats_reset_uri = {
        .uri = "/api/stats/reset", .method = HTTP_POST, .handler = stats_reset_handler,
    };
    const httpd_uri_t firmware_uri = {
        .uri = "/api/firmware", .method = HTTP_POST, .handler = firmware_update_http_handler,
    };
    const httpd_uri_t firmware_retry_uri = {
        .uri = "/api/firmware-retry", .method = HTTP_POST, .handler = firmware_update_retry_http_handler,
    };
    const httpd_uri_t esp32_firmware_uri = {
        .uri = "/api/esp32-firmware", .method = HTTP_POST, .handler = firmware_update_esp32_http_handler,
    };
    const httpd_uri_t ws_uri = {
        .uri = "/ws", .method = HTTP_GET, .handler = ws_handler,
        .is_websocket = true,
        .ws_post_handshake_cb = ws_on_connect,
    };

    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &index_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &favicon_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &status_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &wifi_setup_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &wifi_scan_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &wifi_save_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &usb_refresh_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &restart_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &bootloader_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &start_firmware_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &nuke_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &stats_reset_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &firmware_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &firmware_retry_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &esp32_firmware_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &ws_uri));
    ESP_ERROR_CHECK(httpd_register_err_handler(s_server, HTTPD_404_NOT_FOUND, http_not_found_handler));

    BaseType_t ok = xTaskCreate(ws_tx_task, "ws_tx", 4096, NULL, 5, NULL);
    configASSERT(ok == pdPASS);

    ESP_LOGI(TAG, "Web terminal/status page listening on port 80 (HTTP sessions=%u, lwIP sockets=%d)",
             (unsigned)config.max_open_sockets, CONFIG_LWIP_MAX_SOCKETS);
}
