#include "usb_cdc_bridge.h"

#include <stdbool.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "bridge_buffers.h"
#include "bridge_stats.h"
#include "rp_mcu.h"

static const char *TAG = "usb_cdc";
static cdc_acm_dev_hdl_t s_cdc_dev;
static SemaphoreHandle_t s_device_gone;
static SemaphoreHandle_t s_cdc_mutex;
static EventGroupHandle_t s_cdc_state_events;
static volatile bool s_connected;
static volatile bool s_tcp_client_connected;
static volatile bool s_ws_client_connected;
static int64_t s_last_ws_drop_log_us;
static size_t s_ws_drop_since_log;
static int64_t s_last_tcp_drop_log_us;
static size_t s_tcp_drop_since_log;
static volatile bool s_reconnect_paused;
static volatile bool s_reconnect_hold;
static volatile bool s_normal_cdc_present;
static int64_t s_last_cdc_open_fail_log_us;
static esp_err_t s_last_cdc_open_fail_err = ESP_OK;
static usb_cdc_device_info_t s_device_info;
static portMUX_TYPE s_info_mux = portMUX_INITIALIZER_UNLOCKED;
static usb_host_client_handle_t s_info_client;
static TaskHandle_t s_info_task_handle;
static SemaphoreHandle_t s_info_request_mutex;
static SemaphoreHandle_t s_info_request_done;
static volatile bool s_info_refresh_requested;
static volatile uint8_t s_info_pending_addr;
static volatile esp_err_t s_info_refresh_result = ESP_ERR_INVALID_STATE;

static usb_cdc_diag_t s_diag;
static portMUX_TYPE s_diag_mux = portMUX_INITIALIZER_UNLOCKED;

static void usb_diag_event(const char *fmt, ...)
{
    char message[sizeof(s_diag.last_event)];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(message, sizeof(message), fmt, ap);
    va_end(ap);

    uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000LL);
    portENTER_CRITICAL(&s_diag_mux);
    s_diag.event_seq++;
    s_diag.last_event_ms = now_ms;
    strlcpy(s_diag.last_event, message, sizeof(s_diag.last_event));
    portEXIT_CRITICAL(&s_diag_mux);
}

#define OPENKNX_CAPTURE_SIZE 6144U
static uint8_t s_openknx_capture[OPENKNX_CAPTURE_SIZE];
static volatile size_t s_openknx_capture_len;
static volatile bool s_openknx_capture_active;
static volatile int64_t s_openknx_capture_last_rx_us;
static portMUX_TYPE s_openknx_capture_mux = portMUX_INITIALIZER_UNLOCKED;
static usb_cdc_openknx_info_t s_openknx_info;
static portMUX_TYPE s_openknx_info_mux = portMUX_INITIALIZER_UNLOCKED;
static usb_cdc_serial_state_t s_serial_state;
static portMUX_TYPE s_serial_state_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile rp_mcu_family_t s_known_family = RP_MCU_FAMILY_UNKNOWN;

#define CDC_STATE_RELEASED BIT0
#define RP_SDK_CDC_PID_GENERIC 0x0009U
#define RP_SDK_CDC_PID_RP2040  0x000AU

static void usb_string_to_ascii(const usb_str_desc_t *src, char *dst, size_t dst_len)
{
    if (dst_len == 0) {
        return;
    }
    dst[0] = '\0';
    if (src == NULL || src->bLength < 2) {
        return;
    }

    size_t chars = (src->bLength - 2U) / 2U;
    if (chars >= dst_len) {
        chars = dst_len - 1U;
    }
    for (size_t i = 0; i < chars; ++i) {
        uint16_t ch = src->wData[i];
        dst[i] = (ch >= 0x20 && ch <= 0x7e) ? (char)ch : '?';
    }
    dst[chars] = '\0';
}

static bool usb_device_is_normal_cdc_target(const usb_device_desc_t *desc)
{
    if (desc == NULL || rp_mcu_is_bootsel_usb(desc->idVendor, desc->idProduct)) {
        return false;
    }
    if (desc->idVendor == (uint16_t)CONFIG_BRIDGE_CDC_VID &&
        desc->idProduct == (uint16_t)CONFIG_BRIDGE_CDC_PID) {
        return true;
    }
#if CONFIG_BRIDGE_CDC_TRY_RP_SDK_PIDS
    if (desc->idVendor == RP_USB_VID &&
        (desc->idProduct == RP_SDK_CDC_PID_GENERIC || desc->idProduct == RP_SDK_CDC_PID_RP2040)) {
        return true;
    }
#endif
    return false;
}

static bool usb_device_is_rp_target(const usb_device_desc_t *desc)
{
    if (desc == NULL) {
        return false;
    }
    return rp_mcu_is_bootsel_usb(desc->idVendor, desc->idProduct) ||
           usb_device_is_normal_cdc_target(desc);
}

static esp_err_t store_usb_device_info(usb_device_handle_t usb_dev, bool log_device)
{
    usb_cdc_device_info_t info = {0};
    const usb_device_desc_t *desc = NULL;
    usb_device_info_t dev_info = {0};

    esp_err_t err = usb_host_get_device_descriptor(usb_dev, &desc);
    if (err != ESP_OK || desc == NULL) {
        return err != ESP_OK ? err : ESP_FAIL;
    }

    info.valid = true;
    info.vid = desc->idVendor;
    info.pid = desc->idProduct;
    info.bcd_usb = desc->bcdUSB;
    info.bcd_device = desc->bcdDevice;
    info.dev_class = desc->bDeviceClass;
    info.dev_subclass = desc->bDeviceSubClass;
    info.dev_protocol = desc->bDeviceProtocol;
    info.family = rp_mcu_family_from_bootsel_usb(desc->idVendor, desc->idProduct);
    if (info.family == RP_MCU_FAMILY_UNKNOWN &&
        desc->idVendor == RP_USB_VID && desc->idProduct == RP_SDK_CDC_PID_RP2040) {

        info.family = RP_MCU_FAMILY_RP2040;
    }
    if (info.family == RP_MCU_FAMILY_UNKNOWN) {
        info.family = s_known_family;
    } else {
        s_known_family = info.family;
    }

    err = usb_host_device_info(usb_dev, &dev_info);
    if (err == ESP_OK) {
        info.dev_addr = dev_info.dev_addr;
        info.speed = (uint8_t)dev_info.speed;
        usb_string_to_ascii(dev_info.str_desc_manufacturer, info.manufacturer, sizeof(info.manufacturer));
        usb_string_to_ascii(dev_info.str_desc_product, info.product, sizeof(info.product));
        usb_string_to_ascii(dev_info.str_desc_serial_num, info.serial, sizeof(info.serial));
    }

    portENTER_CRITICAL(&s_info_mux);
    s_device_info = info;
    portEXIT_CRITICAL(&s_info_mux);

    if (log_device) {
        ESP_LOGI(TAG, "USB device info refreshed VID=%04x PID=%04x product='%s'",
                 info.vid, info.pid, info.product[0] ? info.product : "?");
    }
    return ESP_OK;
}

static void on_new_usb_device(usb_device_handle_t usb_dev)
{
    const usb_device_desc_t *desc = NULL;
    if (usb_host_get_device_descriptor(usb_dev, &desc) != ESP_OK || desc == NULL) {
        return;
    }

    if (usb_device_is_rp_target(desc)) {
        (void)store_usb_device_info(usb_dev, false);
    }

    ESP_LOGI(TAG, "USB device discovered VID=%04x PID=%04x",
             desc->idVendor, desc->idProduct);
    usb_diag_event("USB device discovered %04x:%04x", desc->idVendor, desc->idProduct);


    if (usb_device_is_normal_cdc_target(desc)) {
        s_normal_cdc_present = true;
        if (s_reconnect_paused && !s_reconnect_hold) {
            ESP_LOGW(TAG, "Normal RP-series CDC device appeared while reconnect was paused - clearing stale pause");
            usb_diag_event("Normal CDC target appeared; clearing stale reconnect pause");
            usb_cdc_bridge_set_reconnect_paused(false);
        } else if (s_reconnect_hold) {
            ESP_LOGI(TAG, "Normal RP-series CDC device appeared while reconnect hold is active");
            usb_diag_event("Normal CDC target present while hard reconnect hold is active");
        }
    }


    rp_mcu_family_t bootsel_family = rp_mcu_family_from_bootsel_usb(desc->idVendor, desc->idProduct);
    if (bootsel_family != RP_MCU_FAMILY_UNKNOWN) {
        s_normal_cdc_present = false;
        s_known_family = bootsel_family;
        usb_cdc_bridge_set_reconnect_paused(true);
        ESP_LOGI(TAG, "%s BOOTSEL detected - CDC probe suppressed", rp_mcu_family_name(bootsel_family));
        usb_diag_event("%s BOOTSEL detected; CDC reconnect paused", rp_mcu_family_name(bootsel_family));
    }
}

static esp_err_t info_capture_address(uint8_t address)
{
    if (s_info_client == NULL || address == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    usb_device_handle_t dev = NULL;
    esp_err_t err = usb_host_device_open(s_info_client, address, &dev);
    if (err != ESP_OK || dev == NULL) {
        return err != ESP_OK ? err : ESP_FAIL;
    }

    const usb_device_desc_t *desc = NULL;
    err = usb_host_get_device_descriptor(dev, &desc);
    if (err == ESP_OK && desc != NULL && usb_device_is_rp_target(desc)) {
        err = store_usb_device_info(dev, true);
    } else if (err == ESP_OK) {
        err = ESP_ERR_NOT_FOUND;
    }

    esp_err_t close_err = usb_host_device_close(s_info_client, dev);
    if (err == ESP_OK && close_err != ESP_OK) {
        err = close_err;
    }
    return err;
}

static esp_err_t info_scan_connected_devices(void)
{
    if (s_info_client == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t addresses[8] = {0};
    int count = 0;
    esp_err_t err = usb_host_device_addr_list_fill((int)sizeof(addresses), addresses, &count);
    if (err != ESP_OK) {
        return err;
    }

    for (int i = 0; i < count; ++i) {
        err = info_capture_address(addresses[i]);
        if (err == ESP_OK) {
            return ESP_OK;
        }
        if (err != ESP_ERR_NOT_FOUND) {
            ESP_LOGD(TAG, "USB info probe addr=%u failed: %s",
                     (unsigned)addresses[i], esp_err_to_name(err));
        }
    }
    return ESP_ERR_NOT_FOUND;
}

static void info_client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    (void)arg;
    if (event_msg == NULL) {
        return;
    }
    if (event_msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        s_info_pending_addr = event_msg->new_dev.address;
    }
}

static void usb_info_task(void *arg)
{
    (void)arg;
    s_info_task_handle = xTaskGetCurrentTaskHandle();

    while (true) {
        esp_err_t err = usb_host_client_handle_events(s_info_client, pdMS_TO_TICKS(200));
        if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "USB info client events: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        uint8_t pending = s_info_pending_addr;
        if (pending != 0) {
            s_info_pending_addr = 0;
            (void)info_capture_address(pending);
        }

        if (s_info_refresh_requested) {
            s_info_refresh_requested = false;
            s_info_refresh_result = info_scan_connected_devices();
            xSemaphoreGive(s_info_request_done);
        }
    }
}

esp_err_t usb_cdc_bridge_refresh_device_info(uint32_t timeout_ms)
{
    if (s_info_client == NULL || s_info_task_handle == NULL ||
        s_info_request_mutex == NULL || s_info_request_done == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    TickType_t wait_ticks = pdMS_TO_TICKS(timeout_ms == 0 ? 1U : timeout_ms);
    if (xSemaphoreTake(s_info_request_mutex, wait_ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    while (xSemaphoreTake(s_info_request_done, 0) == pdTRUE) {
    }
    s_info_refresh_result = ESP_ERR_INVALID_STATE;
    s_info_refresh_requested = true;

    esp_err_t err = ESP_ERR_TIMEOUT;
    if (xSemaphoreTake(s_info_request_done, wait_ticks) == pdTRUE) {
        err = s_info_refresh_result;
    } else {
        s_info_refresh_requested = false;
    }

    xSemaphoreGive(s_info_request_mutex);
    return err;
}

bool usb_cdc_bridge_is_connected(void)
{
    return s_connected;
}

void usb_cdc_bridge_set_reconnect_paused(bool paused)
{
    bool changed = (s_reconnect_paused != paused);
    s_reconnect_paused = paused;
    if (changed) {
        ESP_LOGI(TAG, "CDC reconnect %s%s", paused ? "paused" : "enabled",
                 s_reconnect_hold ? " (hard hold active)" : "");
        usb_diag_event("CDC reconnect %s%s", paused ? "paused" : "enabled",
                       s_reconnect_hold ? " (hard hold active)" : "");
    }
}

void usb_cdc_bridge_set_reconnect_hold(bool hold)
{
    bool changed = (s_reconnect_hold != hold);
    s_reconnect_hold = hold;
    if (changed) {
        ESP_LOGI(TAG, "CDC reconnect hard hold %s", hold ? "enabled" : "released");
        usb_diag_event("CDC reconnect hard hold %s", hold ? "enabled" : "released");
    }
}

bool usb_cdc_bridge_reconnect_is_paused(void)
{
    return s_reconnect_paused || s_reconnect_hold;
}

void usb_cdc_bridge_get_diag(usb_cdc_diag_t *out)
{
    if (out == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_diag_mux);
    *out = s_diag;
    portEXIT_CRITICAL(&s_diag_mux);
    out->reconnect_paused = s_reconnect_paused;
    out->reconnect_hold = s_reconnect_hold;
    out->normal_target_present = s_normal_cdc_present;
}

bool usb_cdc_bridge_wait_released(uint32_t timeout_ms)
{
    if (s_cdc_state_events == NULL) {
        return false;
    }
    EventBits_t bits = xEventGroupWaitBits(
        s_cdc_state_events, CDC_STATE_RELEASED, pdFALSE, pdTRUE,
        pdMS_TO_TICKS(timeout_ms));
    return (bits & CDC_STATE_RELEASED) != 0;
}

bool usb_cdc_bridge_get_device_info(usb_cdc_device_info_t *out)
{
    if (out == NULL) {
        return false;
    }
    portENTER_CRITICAL(&s_info_mux);
    *out = s_device_info;
    portEXIT_CRITICAL(&s_info_mux);
    return out->valid;
}

bool usb_cdc_bridge_get_openknx_info(usb_cdc_openknx_info_t *out)
{
    if (out == NULL) {
        return false;
    }
    portENTER_CRITICAL(&s_openknx_info_mux);
    *out = s_openknx_info;
    portEXIT_CRITICAL(&s_openknx_info_mux);
    return out->valid;
}

void usb_cdc_bridge_clear_openknx_info(void)
{
    portENTER_CRITICAL(&s_openknx_info_mux);
    memset(&s_openknx_info, 0, sizeof(s_openknx_info));
    portEXIT_CRITICAL(&s_openknx_info_mux);
}

void usb_cdc_bridge_set_tcp_client_state(bool connected)
{
    s_tcp_client_connected = connected;
    if (!connected) {
        bridge_usb_to_net_reset();
    }
}

esp_err_t usb_cdc_bridge_set_line_coding(uint32_t baud, uint8_t data_bits, uint8_t parity_type, uint8_t char_format, uint32_t timeout_ms)
{
    if (baud == 0 || data_bits < 5 || data_bits > 8 || parity_type > 4 || char_format > 2) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_connected || s_cdc_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    TickType_t wait_ticks = pdMS_TO_TICKS(timeout_ms == 0 ? 1 : timeout_ms);
    if (xSemaphoreTake(s_cdc_mutex, wait_ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = ESP_ERR_INVALID_STATE;
    if (s_connected && s_cdc_dev != NULL) {
        cdc_acm_line_coding_t coding = {
            .dwDTERate = baud,
            .bCharFormat = char_format,
            .bParityType = parity_type,
            .bDataBits = data_bits,
        };
        err = cdc_acm_host_line_coding_set(s_cdc_dev, &coding);
        if (err == ESP_OK) {
            portENTER_CRITICAL(&s_serial_state_mux);
            s_serial_state.valid = true;
            s_serial_state.baud = baud;
            s_serial_state.data_bits = data_bits;
            s_serial_state.parity_type = parity_type;
            s_serial_state.char_format = char_format;
            portEXIT_CRITICAL(&s_serial_state_mux);
        }
    }
    xSemaphoreGive(s_cdc_mutex);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "CDC line coding set: %" PRIu32 " baud, data=%u parity=%u stop=%u",
                 baud, (unsigned)data_bits, (unsigned)parity_type, (unsigned)char_format);
    } else {
        ESP_LOGW(TAG, "CDC line coding failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t usb_cdc_bridge_set_control_lines(bool dtr, bool rts, uint32_t timeout_ms)
{
    if (!s_connected || s_cdc_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    TickType_t wait_ticks = pdMS_TO_TICKS(timeout_ms == 0 ? 1 : timeout_ms);
    if (xSemaphoreTake(s_cdc_mutex, wait_ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = ESP_ERR_INVALID_STATE;
    if (s_connected && s_cdc_dev != NULL) {
        err = cdc_acm_host_set_control_line_state(s_cdc_dev, dtr, rts);
        if (err == ESP_OK) {
            portENTER_CRITICAL(&s_serial_state_mux);
            s_serial_state.valid = true;
            s_serial_state.dtr = dtr;
            s_serial_state.rts = rts;
            portEXIT_CRITICAL(&s_serial_state_mux);
        }
    }
    xSemaphoreGive(s_cdc_mutex);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "CDC control lines set: DTR=%u RTS=%u", dtr ? 1U : 0U, rts ? 1U : 0U);
    } else {
        ESP_LOGW(TAG, "CDC control line update failed: %s", esp_err_to_name(err));
    }
    return err;
}

bool usb_cdc_bridge_get_serial_state(usb_cdc_serial_state_t *out)
{
    if (out == NULL) {
        return false;
    }
    portENTER_CRITICAL(&s_serial_state_mux);
    *out = s_serial_state;
    portEXIT_CRITICAL(&s_serial_state_mux);
    return out->valid;
}

esp_err_t usb_cdc_bridge_set_baud(uint32_t baud, uint32_t timeout_ms)
{
    usb_cdc_serial_state_t state = {0};
    if (!usb_cdc_bridge_get_serial_state(&state)) {
        state.baud = CONFIG_BRIDGE_CDC_BAUD;
        state.data_bits = 8;
        state.parity_type = 0;
        state.char_format = 0;
    }
    return usb_cdc_bridge_set_line_coding(baud, state.data_bits, state.parity_type, state.char_format, timeout_ms);
}

esp_err_t usb_cdc_bridge_openknx_1200_touch(uint32_t normal_baud, uint32_t timeout_ms)
{
    if (normal_baud == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_connected || s_cdc_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    TickType_t wait_ticks = pdMS_TO_TICKS(timeout_ms == 0 ? 1 : timeout_ms);
    if (xSemaphoreTake(s_cdc_mutex, wait_ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t result = ESP_OK;
    cdc_acm_dev_hdl_t dev = s_cdc_dev;
    if (!s_connected || dev == NULL) {
        result = ESP_ERR_INVALID_STATE;
        goto out;
    }


    cdc_acm_line_coding_t normal = {
        .dwDTERate = normal_baud,
        .bCharFormat = 0,
        .bParityType = 0,
        .bDataBits = 8,
    };
    esp_err_t err = cdc_acm_host_line_coding_set(dev, &normal);
    if (err == ESP_OK) {
        portENTER_CRITICAL(&s_serial_state_mux);
        s_serial_state.valid = true;
        s_serial_state.baud = normal_baud;
        s_serial_state.data_bits = 8;
        s_serial_state.parity_type = 0;
        s_serial_state.char_format = 0;
        portEXIT_CRITICAL(&s_serial_state_mux);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "OpenKNX touch: normal line coding failed: %s", esp_err_to_name(err));
        result = err;
        goto out;
    }

    err = cdc_acm_host_set_control_line_state(dev, false, false);
    if (err == ESP_OK) {
        portENTER_CRITICAL(&s_serial_state_mux);
        s_serial_state.dtr = false;
        s_serial_state.rts = false;
        portEXIT_CRITICAL(&s_serial_state_mux);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "OpenKNX touch: DTR/RTS low failed: %s", esp_err_to_name(err));
        result = err;
        goto out;
    }

    static const uint8_t save_state = 0x07;
    ESP_LOGI(TAG, "OpenKNX upload handshake: sending 0x07 at %" PRIu32 " baud (DTR/RTS low)", normal_baud);
    err = cdc_acm_host_data_tx_blocking(dev, &save_state, 1, timeout_ms == 0 ? 1 : timeout_ms);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "OpenKNX touch: 0x07 state-save byte failed: %s", esp_err_to_name(err));
        result = err;
        goto out;
    }
    bridge_stats_add_usb_tx(1);


    vTaskDelay(pdMS_TO_TICKS(1000));

    if (!s_connected || s_cdc_dev != dev) {


        ESP_LOGI(TAG, "OpenKNX touch: CDC disappeared before 1200-baud request");
        result = ESP_OK;
        goto out;
    }

    cdc_acm_line_coding_t touch = {
        .dwDTERate = 1200,
        .bCharFormat = 0,
        .bParityType = 0,
        .bDataBits = 8,
    };
    ESP_LOGI(TAG, "OpenKNX upload handshake: switching to 1200 baud with DTR/RTS low");
    err = cdc_acm_host_line_coding_set(dev, &touch);
    if (err == ESP_OK) {
        portENTER_CRITICAL(&s_serial_state_mux);
        s_serial_state.baud = 1200;
        s_serial_state.data_bits = 8;
        s_serial_state.parity_type = 0;
        s_serial_state.char_format = 0;
        portEXIT_CRITICAL(&s_serial_state_mux);
    }
    if (err != ESP_OK) {


        ESP_LOGW(TAG, "OpenKNX touch: 1200-baud request returned %s", esp_err_to_name(err));
        result = err;
    }

out:
    xSemaphoreGive(s_cdc_mutex);
    return result;
}

esp_err_t usb_cdc_bridge_restore_normal_state(uint32_t normal_baud, uint32_t timeout_ms)
{
    if (normal_baud == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_connected || s_cdc_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    TickType_t wait_ticks = pdMS_TO_TICKS(timeout_ms == 0 ? 1 : timeout_ms);
    if (xSemaphoreTake(s_cdc_mutex, wait_ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t result = ESP_ERR_INVALID_STATE;
    cdc_acm_dev_hdl_t dev = s_cdc_dev;
    if (s_connected && dev != NULL) {
        cdc_acm_line_coding_t coding = {
            .dwDTERate = normal_baud,
            .bCharFormat = 0,
            .bParityType = 0,
            .bDataBits = 8,
        };
        result = cdc_acm_host_line_coding_set(dev, &coding);
        if (result == ESP_OK) {
            esp_err_t line_err = cdc_acm_host_set_control_line_state(dev, true, true);
            if (line_err != ESP_OK) {
                result = line_err;
            }
        }
    }
    xSemaphoreGive(s_cdc_mutex);

    if (result == ESP_OK) {
        portENTER_CRITICAL(&s_serial_state_mux);
        s_serial_state.valid = true;
        s_serial_state.baud = normal_baud;
        s_serial_state.data_bits = 8;
        s_serial_state.parity_type = 0;
        s_serial_state.char_format = 0;
        s_serial_state.dtr = true;
        s_serial_state.rts = true;
        portEXIT_CRITICAL(&s_serial_state_mux);
        ESP_LOGI(TAG, "CDC normal state restored: %" PRIu32 " baud, DTR/RTS high", normal_baud);
    } else {
        ESP_LOGW(TAG, "CDC normal state restore failed: %s", esp_err_to_name(result));
    }
    return result;
}

static void strip_ansi_and_controls(const uint8_t *src, size_t src_len, char *dst, size_t dst_len)
{
    if (dst == NULL || dst_len == 0) {
        return;
    }
    size_t o = 0;
    uint8_t esc_state = 0;
    for (size_t i = 0; i < src_len && o + 1 < dst_len; ++i) {
        uint8_t c = src[i];
        if (esc_state == 1) {
            esc_state = (c == '[') ? 2 : 0;
            continue;
        }
        if (esc_state == 2) {


            if (c >= 0x40 && c <= 0x7e) {
                esc_state = 0;
            }
            continue;
        }
        if (c == 0x1b) {
            esc_state = 1;
            continue;
        }
        if (c == '\r') {
            continue;
        }
        if (c == '\n' || c == '\t' || (c >= 0x20 && c <= 0x7e)) {
            dst[o++] = (char)c;
        }
    }
    dst[o] = '\0';
}

static bool extract_console_field(const char *section, const char *label, char *out, size_t out_len)
{
    if (section == NULL || label == NULL || out == NULL || out_len == 0) {
        return false;
    }
    out[0] = '\0';
    const char *p = section;
    const size_t label_len = strlen(label);
    while ((p = strstr(p, label)) != NULL) {


        const char *colon = strchr(p + label_len, ':');
        const char *line_end = strchr(p, '\n');
        if (line_end == NULL) {
            line_end = p + strlen(p);
        }
        if (colon != NULL && colon < line_end && (size_t)(colon - (p + label_len)) <= 3U) {
            const char *v = colon + 1;
            while (v < line_end && (*v == ' ' || *v == '\t')) {
                ++v;
            }
            const char *e = line_end;
            while (e > v && (e[-1] == ' ' || e[-1] == '\t')) {
                --e;
            }
            size_t n = (size_t)(e - v);
            if (n >= out_len) {
                n = out_len - 1U;
            }
            memcpy(out, v, n);
            out[n] = '\0';
            return n > 0;
        }
        p += label_len;
    }
    return false;
}

static bool parse_openknx_info_capture(const uint8_t *raw, size_t raw_len, usb_cdc_openknx_info_t *out)
{
    if (raw == NULL || raw_len == 0 || out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));

    char *clean = calloc(1, raw_len + 1U);
    if (clean == NULL) {
        return false;
    }
    strip_ansi_and_controls(raw, raw_len, clean, raw_len + 1U);


    char *section = strstr(clean, "Firmware");
    if (section == NULL) {
        free(clean);
        return false;
    }

    (void)extract_console_field(section, "Name", out->name, sizeof(out->name));
    bool have_version = extract_console_field(section, "Version", out->version, sizeof(out->version));
    bool have_number = extract_console_field(section, "Number", out->number, sizeof(out->number));

    if (have_number) {
        const char *n = out->number;
        while (*n == ' ' || *n == '\t') {
            ++n;
        }
        if (*n == '$') {
            ++n;
        } else if (n[0] == '0' && (n[1] == 'x' || n[1] == 'X')) {
            n += 2;
        }
        char *end = NULL;
        unsigned long id = strtoul(n, &end, 16);
        if (end != n && id <= 0xffffUL) {
            out->app_id = (uint16_t)id;
        }
    }

    out->valid = have_version || (have_number && out->app_id != 0);
    free(clean);
    return out->valid;
}

esp_err_t usb_cdc_bridge_query_openknx_info(usb_cdc_openknx_info_t *out, uint32_t timeout_ms)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));
    if (!s_connected || s_cdc_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (timeout_ms < 500U) {
        timeout_ms = 500U;
    }

    portENTER_CRITICAL(&s_openknx_capture_mux);
    s_openknx_capture_len = 0;
    s_openknx_capture_last_rx_us = esp_timer_get_time();
    s_openknx_capture_active = true;
    portEXIT_CRITICAL(&s_openknx_capture_mux);

    static const uint8_t command[] = "info\r\n";
    esp_err_t err = usb_cdc_bridge_send_direct(command, sizeof(command) - 1U, 1000);
    if (err != ESP_OK) {
        portENTER_CRITICAL(&s_openknx_capture_mux);
        s_openknx_capture_active = false;
        portEXIT_CRITICAL(&s_openknx_capture_mux);
        return err;
    }

    const int64_t start_us = esp_timer_get_time();
    while (true) {
        int64_t now = esp_timer_get_time();
        size_t captured;
        int64_t last_rx;
        portENTER_CRITICAL(&s_openknx_capture_mux);
        captured = s_openknx_capture_len;
        last_rx = s_openknx_capture_last_rx_us;
        portEXIT_CRITICAL(&s_openknx_capture_mux);

        const uint32_t elapsed_ms = (uint32_t)((now - start_us) / 1000LL);
        const uint32_t idle_ms = (uint32_t)((now - last_rx) / 1000LL);
        if ((captured > 0 && elapsed_ms >= 300U && idle_ms >= 250U) || elapsed_ms >= timeout_ms) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    uint8_t *snapshot = malloc(OPENKNX_CAPTURE_SIZE);
    if (snapshot == NULL) {
        portENTER_CRITICAL(&s_openknx_capture_mux);
        s_openknx_capture_active = false;
        portEXIT_CRITICAL(&s_openknx_capture_mux);
        return ESP_ERR_NO_MEM;
    }

    size_t snapshot_len;
    portENTER_CRITICAL(&s_openknx_capture_mux);
    s_openknx_capture_active = false;
    snapshot_len = s_openknx_capture_len;
    if (snapshot_len > OPENKNX_CAPTURE_SIZE) {
        snapshot_len = OPENKNX_CAPTURE_SIZE;
    }
    memcpy(snapshot, s_openknx_capture, snapshot_len);
    portEXIT_CRITICAL(&s_openknx_capture_mux);

    bool parsed = parse_openknx_info_capture(snapshot, snapshot_len, out);
    free(snapshot);
    if (!parsed) {
        ESP_LOGW(TAG, "OpenKNX info query: response received but firmware fields were not recognized");
        return ESP_ERR_NOT_FOUND;
    }
    if (!s_connected || s_cdc_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&s_openknx_info_mux);
    s_openknx_info = *out;
    portEXIT_CRITICAL(&s_openknx_info_mux);

    ESP_LOGI(TAG, "OpenKNX firmware: id=0x%04x version=%s name='%s'",
             out->app_id, out->version[0] ? out->version : "?", out->name[0] ? out->name : "?");
    return ESP_OK;
}

esp_err_t usb_cdc_bridge_send_direct(const uint8_t *data, size_t len, uint32_t timeout_ms)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_connected || s_cdc_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    TickType_t wait_ticks = pdMS_TO_TICKS(timeout_ms == 0 ? 1 : timeout_ms);
    if (xSemaphoreTake(s_cdc_mutex, wait_ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = ESP_ERR_INVALID_STATE;
    if (s_connected && s_cdc_dev != NULL) {
        err = cdc_acm_host_data_tx_blocking(s_cdc_dev, data, len, timeout_ms == 0 ? 1 : timeout_ms);
        if (err == ESP_OK) {
            bridge_stats_add_usb_tx(len);
        }
    }
    xSemaphoreGive(s_cdc_mutex);
    return err;
}

void usb_cdc_bridge_set_ws_client_state(bool connected)
{


    s_ws_client_connected = connected;
}

static bool on_cdc_rx(const uint8_t *data, size_t data_len, void *arg)
{
    (void)arg;
    if (data_len == 0) {
        return true;
    }

    bridge_stats_add_usb_rx(data_len);

    if (s_openknx_capture_active) {
        portENTER_CRITICAL(&s_openknx_capture_mux);
        if (s_openknx_capture_active) {
            size_t free_space = OPENKNX_CAPTURE_SIZE - s_openknx_capture_len;
            size_t copy_len = data_len < free_space ? data_len : free_space;
            if (copy_len > 0) {
                memcpy(s_openknx_capture + s_openknx_capture_len, data, copy_len);
                s_openknx_capture_len += copy_len;
            }
            s_openknx_capture_last_rx_us = esp_timer_get_time();
        }
        portEXIT_CRITICAL(&s_openknx_capture_mux);
    }

    if (s_tcp_client_connected) {
        const size_t written = bridge_usb_to_net_push(data, data_len);
        if (written != data_len) {
            const size_t dropped = data_len - written;
            bridge_stats_add_dropped_usb_to_tcp(dropped);
            s_tcp_drop_since_log += dropped;
            const int64_t now = esp_timer_get_time();
            if (now - s_last_tcp_drop_log_us >= 1000000LL) {
                ESP_LOGW(TAG, "USB->TCP backpressure: dropped %u bytes in last interval", (unsigned)s_tcp_drop_since_log);
                s_tcp_drop_since_log = 0;
                s_last_tcp_drop_log_us = now;
            }
        }
    }

    if (s_ws_client_connected) {


        const size_t dropped = bridge_usb_to_ws_broadcast(data, data_len);
        if (dropped > 0) {
            bridge_stats_add_dropped_usb_to_ws(dropped);
            s_ws_drop_since_log += dropped;
            const int64_t now = esp_timer_get_time();
            if (now - s_last_ws_drop_log_us >= 1000000LL) {
                ESP_LOGW(TAG, "USB->WebSocket backpressure: dropped %u client-bytes in last interval", (unsigned)s_ws_drop_since_log);
                s_ws_drop_since_log = 0;
                s_last_ws_drop_log_us = now;
            }
        }
    }
    return true;
}

static void on_cdc_event(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;

    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "CDC error: %d", event->data.error);
        usb_diag_event("CDC host error: %d", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "RP MCU/CDC device disconnected");
        portENTER_CRITICAL(&s_diag_mux);
        s_diag.disconnect_count++;
        portEXIT_CRITICAL(&s_diag_mux);
        usb_diag_event("RP MCU/CDC device disconnected");
        s_connected = false;
        s_normal_cdc_present = false;
        portENTER_CRITICAL(&s_serial_state_mux);
        s_serial_state.valid = false;
        portEXIT_CRITICAL(&s_serial_state_mux);
        portENTER_CRITICAL(&s_info_mux);
        s_device_info.valid = false;
        portEXIT_CRITICAL(&s_info_mux);


        s_known_family = RP_MCU_FAMILY_UNKNOWN;
        usb_cdc_bridge_clear_openknx_info();
        xSemaphoreGive(s_device_gone);
        break;
    case CDC_ACM_HOST_SERIAL_STATE:
        ESP_LOGD(TAG, "CDC serial state: 0x%04x", event->data.serial_state.val);
        break;
    case CDC_ACM_HOST_NETWORK_CONNECTION:
    default:
        break;
    }
}

static void usb_lib_task(void *arg)
{
    (void)arg;
    while (true) {
        uint32_t event_flags = 0;
        esp_err_t err = usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "usb_host_lib_handle_events: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

static void usb_tx_task(void *arg)
{
    (void)arg;
    uint8_t buf[CONFIG_BRIDGE_USB_TRANSFER_SIZE];

    while (true) {
        size_t len = bridge_net_to_usb_pop(buf, sizeof(buf), pdMS_TO_TICKS(100));
        if (len == 0) {
            continue;
        }

        if (!s_connected) {
            bridge_stats_add_dropped_to_usb(len);
            continue;
        }

        if (xSemaphoreTake(s_cdc_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
            bridge_stats_add_dropped_to_usb(len);
            ESP_LOGW(TAG, "CDC mutex timeout");
            continue;
        }

        cdc_acm_dev_hdl_t dev = s_cdc_dev;
        esp_err_t err = ESP_ERR_INVALID_STATE;
        if (s_connected && dev != NULL) {
            err = cdc_acm_host_data_tx_blocking(dev, buf, len, 1000);
        }
        xSemaphoreGive(s_cdc_mutex);

        if (err == ESP_OK) {
            bridge_stats_add_usb_tx(len);
        } else {
            bridge_stats_add_dropped_to_usb(len);
            if (err != ESP_ERR_INVALID_STATE) {
                ESP_LOGW(TAG, "Network->USB transfer failed: %s", esp_err_to_name(err));
            }
        }
    }
}

static esp_err_t open_target_cdc(const cdc_acm_host_device_config_t *dev_cfg, cdc_acm_dev_hdl_t *out_dev)
{
    if (dev_cfg == NULL || out_dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint16_t configured_vid = (uint16_t)CONFIG_BRIDGE_CDC_VID;
    const uint16_t configured_pid = (uint16_t)CONFIG_BRIDGE_CDC_PID;
    esp_err_t err = cdc_acm_host_open(configured_vid, configured_pid, 0, dev_cfg, out_dev);
    if (err == ESP_OK) {
        return ESP_OK;
    }

#if CONFIG_BRIDGE_CDC_TRY_RP_SDK_PIDS
    if (configured_vid == RP_USB_VID) {
        const uint16_t fallback_pids[] = {RP_SDK_CDC_PID_GENERIC, RP_SDK_CDC_PID_RP2040};
        for (size_t i = 0; i < sizeof(fallback_pids) / sizeof(fallback_pids[0]); ++i) {
            if (fallback_pids[i] == configured_pid) {
                continue;
            }
            err = cdc_acm_host_open(RP_USB_VID, fallback_pids[i], 0, dev_cfg, out_dev);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "CDC connected via standard RP-series PID 0x%04x", fallback_pids[i]);
                return ESP_OK;
            }
        }
    }
#endif
    return err;
}

static void usb_connect_task(void *arg)
{
    (void)arg;

    const cdc_acm_host_device_config_t dev_cfg = {
        .connection_timeout_ms = 1000,
        .out_buffer_size = CONFIG_BRIDGE_USB_TRANSFER_SIZE,
        .in_buffer_size = CONFIG_BRIDGE_USB_TRANSFER_SIZE,
        .event_cb = on_cdc_event,
        .data_cb = on_cdc_rx,
        .user_arg = NULL,
    };

    while (true) {


        if (s_reconnect_paused || s_reconnect_hold) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        cdc_acm_dev_hdl_t dev = NULL;


        esp_err_t err = open_target_cdc(&dev_cfg, &dev);
        if (err != ESP_OK) {
            portENTER_CRITICAL(&s_diag_mux);
            s_diag.open_fail_count++;
            s_diag.last_open_error = (int32_t)err;
            portEXIT_CRITICAL(&s_diag_mux);
            const int64_t now = esp_timer_get_time();
            if (err != s_last_cdc_open_fail_err || now - s_last_cdc_open_fail_log_us >= 5000000LL) {
                ESP_LOGW(TAG, "CDC open failed for %04x:%04x: %s%s",
                         (unsigned)CONFIG_BRIDGE_CDC_VID, (unsigned)CONFIG_BRIDGE_CDC_PID,
                         esp_err_to_name(err), s_normal_cdc_present ? " (target USB device is present)" : "");
                usb_diag_event("CDC open failed: %s%s", esp_err_to_name(err),
                               s_normal_cdc_present ? "; USB target present" : "");
                s_last_cdc_open_fail_err = err;
                s_last_cdc_open_fail_log_us = now;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        s_last_cdc_open_fail_err = ESP_OK;
        s_normal_cdc_present = true;

        xSemaphoreTake(s_cdc_mutex, portMAX_DELAY);
        xEventGroupClearBits(s_cdc_state_events, CDC_STATE_RELEASED);
        s_cdc_dev = dev;
        bridge_net_to_usb_reset();
        bridge_usb_to_net_reset();
        bridge_usb_to_ws_reset();

        cdc_acm_line_coding_t coding = {
            .dwDTERate = CONFIG_BRIDGE_CDC_BAUD,
            .bCharFormat = 0,
            .bParityType = 0,
            .bDataBits = 8,
        };
        err = cdc_acm_host_line_coding_set(dev, &coding);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "CDC line coding not accepted: %s", esp_err_to_name(err));
        }

        err = cdc_acm_host_set_control_line_state(dev, true, true);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "DTR/RTS not accepted: %s", esp_err_to_name(err));
        }

        portENTER_CRITICAL(&s_serial_state_mux);
        s_serial_state.valid = true;
        s_serial_state.baud = CONFIG_BRIDGE_CDC_BAUD;
        s_serial_state.data_bits = 8;
        s_serial_state.parity_type = 0;
        s_serial_state.char_format = 0;
        s_serial_state.dtr = true;
        s_serial_state.rts = true;
        portEXIT_CRITICAL(&s_serial_state_mux);

        s_connected = true;
        portENTER_CRITICAL(&s_diag_mux);
        s_diag.connect_count++;
        s_diag.last_open_error = ESP_OK;
        portEXIT_CRITICAL(&s_diag_mux);
        xSemaphoreGive(s_cdc_mutex);
        usb_cdc_bridge_clear_openknx_info();

        ESP_LOGI(TAG, "CDC device connected - bridge active");
        usb_diag_event("CDC device connected; bridge active");
        esp_err_t info_err = usb_cdc_bridge_refresh_device_info(1200);
        if (info_err != ESP_OK) {
            ESP_LOGW(TAG, "USB device metadata refresh after connect failed: %s", esp_err_to_name(info_err));
        }
        cdc_acm_host_desc_print(dev);

        xSemaphoreTake(s_device_gone, portMAX_DELAY);

        xSemaphoreTake(s_cdc_mutex, portMAX_DELAY);
        if (s_cdc_dev != NULL) {
            esp_err_t close_err = cdc_acm_host_close(s_cdc_dev);
            if (close_err != ESP_OK) {
                ESP_LOGW(TAG, "CDC close: %s", esp_err_to_name(close_err));
            }
            s_cdc_dev = NULL;
        }
        s_connected = false;
        portENTER_CRITICAL(&s_serial_state_mux);
        s_serial_state.valid = false;
        portEXIT_CRITICAL(&s_serial_state_mux);
        bridge_net_to_usb_reset();
        bridge_usb_to_net_reset();
        bridge_usb_to_ws_reset();
        xSemaphoreGive(s_cdc_mutex);
        xEventGroupSetBits(s_cdc_state_events, CDC_STATE_RELEASED);
        ESP_LOGI(TAG, "CDC device fully released");
        usb_diag_event("CDC device fully released; reconnect loop resumes");
    }
}

void usb_cdc_bridge_start(void)
{
    s_device_gone = xSemaphoreCreateBinary();
    s_cdc_mutex = xSemaphoreCreateMutex();
    s_cdc_state_events = xEventGroupCreate();
    s_info_request_mutex = xSemaphoreCreateMutex();
    s_info_request_done = xSemaphoreCreateBinary();
    configASSERT(s_device_gone != NULL);
    configASSERT(s_cdc_mutex != NULL);
    configASSERT(s_cdc_state_events != NULL);
    configASSERT(s_info_request_mutex != NULL);
    configASSERT(s_info_request_done != NULL);
    s_reconnect_paused = false;
    s_reconnect_hold = false;
    s_normal_cdc_present = false;
    portENTER_CRITICAL(&s_diag_mux);
    memset(&s_diag, 0, sizeof(s_diag));
    portEXIT_CRITICAL(&s_diag_mux);
    usb_diag_event("USB CDC diagnostics initialized");
    s_last_cdc_open_fail_log_us = 0;
    s_last_cdc_open_fail_err = ESP_OK;
    s_info_refresh_requested = false;
    s_info_pending_addr = 0;
    xEventGroupSetBits(s_cdc_state_events, CDC_STATE_RELEASED);

    const usb_host_config_t host_cfg = {
        .skip_phy_setup = false,
        .intr_flags = 0,
    };

    ESP_LOGI(TAG, "Installing USB host stack");
    ESP_ERROR_CHECK(usb_host_install(&host_cfg));


    const usb_host_client_config_t info_client_cfg = {
        .is_synchronous = false,
        .max_num_event_msg = 4,
        .async = {
            .client_event_callback = info_client_event_cb,
            .callback_arg = NULL,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&info_client_cfg, &s_info_client));

    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));
    ESP_ERROR_CHECK(cdc_acm_host_register_new_dev_callback(on_new_usb_device));

    BaseType_t ok = xTaskCreate(usb_info_task, "usb_info", 4096, NULL, 9, NULL);
    configASSERT(ok == pdPASS);
    ok = xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 12, NULL);
    configASSERT(ok == pdPASS);

    ok = xTaskCreate(usb_tx_task, "usb_tx", 4096, NULL, 8, NULL);
    configASSERT(ok == pdPASS);
    ok = xTaskCreate(usb_connect_task, "usb_connect", 5120, NULL, 7, NULL);
    configASSERT(ok == pdPASS);
}
