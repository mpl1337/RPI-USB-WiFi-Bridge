#include "captive_portal.h"

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

static const char *TAG = "captive";
static TaskHandle_t s_dns_task;
static volatile bool s_dns_running;
static int s_dns_socket = -1;
static char s_captive_uri[32] = "http://192.168.4.1/";

#define DNS_PORT 53
#define DNS_MAX_PACKET 512
#define DNS_ANSWER_LEN 16
#define DHCPS_OFFER_DNS 0x02

static int dns_question_end(const uint8_t *packet, size_t len, size_t *question_end)
{
    if (packet == NULL || question_end == NULL || len < 17) {
        return -1;
    }

    size_t p = 12;
    while (p < len) {
        uint8_t label_len = packet[p];
        if (label_len == 0) {
            p++;
            break;
        }
        if ((label_len & 0xc0) != 0 || label_len > 63 || p + 1 + label_len > len) {
            return -1;
        }
        p += 1 + label_len;
    }

    if (p + 4 > len) {
        return -1;
    }

    *question_end = p + 4;
    return 0;
}

static int build_dns_reply(const uint8_t *request, size_t request_len,
                           uint8_t *reply, size_t reply_size, uint32_t ip_addr)
{
    size_t question_end = 0;
    if (dns_question_end(request, request_len, &question_end) != 0 ||
        question_end + DNS_ANSWER_LEN > reply_size) {
        return -1;
    }

    uint16_t qd_count = ((uint16_t)request[4] << 8) | request[5];
    if (qd_count == 0) {
        return -1;
    }

    size_t qtype_pos = question_end - 4;
    uint16_t qtype = ((uint16_t)request[qtype_pos] << 8) | request[qtype_pos + 1];
    uint16_t qclass = ((uint16_t)request[qtype_pos + 2] << 8) | request[qtype_pos + 3];

    if (request_len > reply_size) {
        return -1;
    }

    memcpy(reply, request, request_len);
    reply[2] |= 0x80;
    reply[2] &= (uint8_t)~0x04;
    reply[3] &= 0xf0;
    reply[6] = 0;
    reply[7] = 0;
    reply[8] = 0;
    reply[9] = 0;
    reply[10] = 0;
    reply[11] = 0;

    if (qtype != 1 || qclass != 1) {
        return (int)request_len;
    }

    reply[6] = 0;
    reply[7] = 1;

    uint8_t *answer = reply + request_len;
    answer[0] = 0xc0;
    answer[1] = 0x0c;
    answer[2] = 0x00;
    answer[3] = 0x01;
    answer[4] = 0x00;
    answer[5] = 0x01;
    answer[6] = 0x00;
    answer[7] = 0x00;
    answer[8] = 0x00;
    answer[9] = 0x3c;
    answer[10] = 0x00;
    answer[11] = 0x04;
    memcpy(answer + 12, &ip_addr, 4);

    return (int)(request_len + DNS_ANSWER_LEN);
}

static void dns_task(void *arg)
{
    (void)arg;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "DNS socket create failed: errno %d", errno);
        s_dns_running = false;
        s_dns_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    s_dns_socket = sock;

    struct timeval timeout = {
        .tv_sec = 0,
        .tv_usec = 500000,
    };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(DNS_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "DNS socket bind failed: errno %d", errno);
        close(sock);
        s_dns_socket = -1;
        s_dns_running = false;
        s_dns_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Captive DNS active on UDP 53");

    while (s_dns_running) {
        uint8_t request[DNS_MAX_PACKET];
        uint8_t reply[DNS_MAX_PACKET + DNS_ANSWER_LEN];
        struct sockaddr_storage source = {0};
        socklen_t source_len = sizeof(source);

        int received = recvfrom(sock, request, sizeof(request), 0,
                                (struct sockaddr *)&source, &source_len);
        if (received < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            if (s_dns_running) {
                ESP_LOGW(TAG, "DNS receive failed: errno %d", errno);
            }
            continue;
        }

        esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
        esp_netif_ip_info_t info = {0};
        if (ap_netif == NULL || esp_netif_get_ip_info(ap_netif, &info) != ESP_OK) {
            continue;
        }

        int reply_len = build_dns_reply(request, (size_t)received, reply,
                                        sizeof(reply), info.ip.addr);
        if (reply_len <= 0) {
            continue;
        }

        sendto(sock, reply, (size_t)reply_len, 0,
               (struct sockaddr *)&source, source_len);
    }

    shutdown(sock, SHUT_RDWR);
    close(sock);
    s_dns_socket = -1;
    s_dns_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t captive_portal_configure_dhcp(esp_netif_t *ap_netif)
{
    if (ap_netif == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_netif_ip_info_t info = {0};
    esp_err_t err = esp_netif_get_ip_info(ap_netif, &info);
    if (err != ESP_OK) {
        return err;
    }

    snprintf(s_captive_uri, sizeof(s_captive_uri), "http://" IPSTR "/", IP2STR(&info.ip));

    esp_err_t stop_err = esp_netif_dhcps_stop(ap_netif);
    if (stop_err != ESP_OK) {
        ESP_LOGD(TAG, "DHCP stop returned: %s", esp_err_to_name(stop_err));
    }

    uint8_t dns_offer = DHCPS_OFFER_DNS;
    err = esp_netif_dhcps_option(ap_netif, ESP_NETIF_OP_SET,
                                 ESP_NETIF_DOMAIN_NAME_SERVER,
                                 &dns_offer, sizeof(dns_offer));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DHCP DNS option failed: %s", esp_err_to_name(err));
    }

    esp_netif_dns_info_t dns = {0};
    dns.ip.type = IPADDR_TYPE_V4;
    dns.ip.u_addr.ip4.addr = info.ip.addr;
    err = esp_netif_set_dns_info(ap_netif, ESP_NETIF_DNS_MAIN, &dns);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DHCP DNS address failed: %s", esp_err_to_name(err));
    }

    err = esp_netif_dhcps_option(ap_netif, ESP_NETIF_OP_SET,
                                 ESP_NETIF_CAPTIVEPORTAL_URI,
                                 s_captive_uri, strlen(s_captive_uri));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DHCP captive portal option failed: %s", esp_err_to_name(err));
    }

    esp_err_t start_err = esp_netif_dhcps_start(ap_netif);
    if (start_err != ESP_OK) {
        return start_err;
    }

    ESP_LOGI(TAG, "Captive portal DHCP active: %s", s_captive_uri);
    return ESP_OK;
}

esp_err_t captive_portal_start_dns(void)
{
    if (s_dns_running) {
        return ESP_OK;
    }

    s_dns_running = true;
    BaseType_t ok = xTaskCreate(dns_task, "captive_dns", 4096, NULL, 5, &s_dns_task);
    if (ok != pdPASS) {
        s_dns_running = false;
        s_dns_task = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void captive_portal_stop_dns(void)
{
    if (!s_dns_running) {
        return;
    }

    s_dns_running = false;
    int sock = s_dns_socket;
    if (sock >= 0) {
        shutdown(sock, SHUT_RDWR);
    }
}
