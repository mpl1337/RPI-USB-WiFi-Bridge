#include "knx_usb_monitor.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "usb/hid_host.h"

#define KNX_HID_REPORT_ID          0x01U
#define KNX_USB_HEADER_SIZE        8U
#define KNX_USB_REPORT_SIZE       64U
#define KNX_USB_PACKET_START       0x01U
#define KNX_USB_PACKET_END         0x02U
#define KNX_USB_PACKET_PARTIAL     0x04U
#define KNX_USB_TUNNEL            0x01U
#define KNX_USB_BUS_ACCESS        0x0FU
#define KNX_USB_CEMI              0x03U
#define KNX_FEATURE_GET           0x01U
#define KNX_FEATURE_RESPONSE      0x02U
#define KNX_FEATURE_SET           0x03U
#define KNX_FEATURE_BUS_STATUS    0x03U
#define KNX_FEATURE_ACTIVE_EMI    0x05U
#define KNX_CEMI_DATA_IND         0x29U
#define KNX_CEMI_BUSMON_IND       0x2BU
#define KNX_USB_REASSEMBLY_MAX   (KNX_USB_RAW_MAX + KNX_USB_HEADER_SIZE)

static const char *TAG = "knx_usb";
static knx_usb_event_t s_events[KNX_USB_HISTORY_SIZE];
static knx_usb_status_t s_status;
static portMUX_TYPE s_monitor_lock = portMUX_INITIALIZER_UNLOCKED;
static hid_host_device_handle_t s_active_device;

typedef enum {
    KNX_HID_CONNECTED,
    KNX_HID_DISCONNECTED
} knx_hid_event_type_t;

typedef struct {
    knx_hid_event_type_t type;
    hid_host_device_handle_t handle;
} knx_hid_event_t;

static QueueHandle_t s_hid_queue;

/*
 * Input callbacks are serialized by the HID driver's event task. Report
 * reassembly therefore stays outside the history lock and never blocks HTTP.
 */
static struct {
    uint8_t data[KNX_USB_REASSEMBLY_MAX];
    size_t size;
    uint8_t next_packet;
    bool running;
} s_assembly;

static void report_malformed(void)
{
    portENTER_CRITICAL(&s_monitor_lock);
    s_status.malformed_reports++;
    portEXIT_CRITICAL(&s_monitor_lock);
}

static bool knx_hid_report_descriptor(const uint8_t *desc, size_t len)
{
    if (desc == NULL) return false;
    /* KNX HID Usage Page 0xFFA0 (little-endian short HID item 0x06). */
    for (size_t i = 0; i + 2 < len; ++i) {
        if (desc[i] == 0x06 && desc[i + 1] == 0xA0 && desc[i + 2] == 0xFF) {
            return true;
        }
    }
    return false;
}

static void store_cemi(const uint8_t *cemi, size_t len)
{
    if (len < 2 || len > KNX_USB_RAW_MAX || len < 2U + cemi[1]) {
        report_malformed();
        return;
    }

    knx_usb_event_t event = {0};
    event.received_us = (uint64_t)esp_timer_get_time();
    event.message_code = cemi[0];
    event.raw_len = (uint16_t)len;
    memcpy(event.raw, cemi, len);

    size_t base = 2U + cemi[1]; /* variable-length cEMI additional information */
    if ((event.message_code == KNX_CEMI_DATA_IND ||
         event.message_code == 0x2EU || event.message_code == 0x11U) &&
        len >= base + 9U) {
        event.ctrl1 = cemi[base];
        event.ctrl2 = cemi[base + 1];
        event.source = (uint16_t)((cemi[base + 2] << 8) | cemi[base + 3]);
        event.destination = (uint16_t)((cemi[base + 4] << 8) | cemi[base + 5]);
        event.group_address = (event.ctrl2 & 0x80U) != 0;
        event.addresses_valid = true;
    }

    portENTER_CRITICAL(&s_monitor_lock);
    event.seq = ++s_status.last_seq;
    s_events[(event.seq - 1U) % KNX_USB_HISTORY_SIZE] = event;
    s_status.captured++;
    if (event.message_code == KNX_CEMI_DATA_IND && event.group_address) {
        s_status.group_telegrams++;
    }
    if (event.message_code == KNX_CEMI_BUSMON_IND) {
        s_status.busmonitor_telegrams++;
    }
    portEXIT_CRITICAL(&s_monitor_lock);
}

static void handle_knx_message(const uint8_t *payload, size_t len)
{
    if (len < KNX_USB_HEADER_SIZE ||
        payload[0] != 0x00 || payload[1] != KNX_USB_HEADER_SIZE) {
        report_malformed();
        return;
    }
    const size_t body_len = ((size_t)payload[2] << 8) | payload[3];
    if (KNX_USB_HEADER_SIZE + body_len != len) {
        report_malformed();
        return;
    }

    if (payload[4] == KNX_USB_TUNNEL && payload[5] == KNX_USB_CEMI) {
        store_cemi(payload + KNX_USB_HEADER_SIZE, body_len);
    } else if (payload[4] == KNX_USB_BUS_ACCESS &&
               payload[5] == KNX_FEATURE_RESPONSE && body_len >= 2) {
        const uint8_t *body = payload + KNX_USB_HEADER_SIZE;
        portENTER_CRITICAL(&s_monitor_lock);
        if (body[0] == KNX_FEATURE_ACTIVE_EMI) {
            s_status.active_emi = body[1];
        } else if (body[0] == KNX_FEATURE_BUS_STATUS) {
            s_status.bus_status_known = true;
            s_status.bus_connected = body[1] != 0;
        }
        portEXIT_CRITICAL(&s_monitor_lock);
    }
}

static void knx_process_hid_report(const uint8_t *data, size_t len)
{
    if (len < 3 || data[0] != KNX_HID_REPORT_ID ||
        (size_t)data[2] + 3U > len) {
        report_malformed();
        s_assembly.running = false;
        return;
    }
    const uint8_t info = data[1];
    const uint8_t seq = info >> 4;
    const uint8_t flags = info & 0x07U;
    const bool start = (flags & KNX_USB_PACKET_START) != 0;
    const bool end = (flags & KNX_USB_PACKET_END) != 0;
    const bool partial = (flags & KNX_USB_PACKET_PARTIAL) != 0;

    if (start) {
        s_assembly.size = 0;
        s_assembly.next_packet = 1;
        s_assembly.running = true;
    }

    if (!s_assembly.running ||
        seq != s_assembly.next_packet ||
        (seq == 1 && !start) ||
        (seq > 1 && (start || !partial)) ||
        s_assembly.size + data[2] > sizeof(s_assembly.data)) {
        report_malformed();
        s_assembly.running = false;
        return;
    }

    memcpy(s_assembly.data + s_assembly.size, data + 3, data[2]);
    s_assembly.size += data[2];
    ++s_assembly.next_packet;

    if (end) {
        handle_knx_message(s_assembly.data, s_assembly.size);
        s_assembly.running = false;
    } else if (!partial || s_assembly.next_packet > 5) {
        report_malformed();
        s_assembly.running = false;
    }
}

/* USB HID output reports configure the USB interface, never send KNX bus data. */
static esp_err_t knx_send_feature(hid_host_device_handle_t dev,
                                  uint8_t service, const uint8_t *body, size_t len)
{
    if (len > 53) return ESP_ERR_INVALID_SIZE;
    uint8_t report[KNX_USB_REPORT_SIZE] = {0};
    report[0] = KNX_HID_REPORT_ID;
    report[1] = 0x13; /* sequence 1, START | END */
    report[2] = (uint8_t)(KNX_USB_HEADER_SIZE + len);
    report[3] = 0x00; /* protocol version */
    report[4] = KNX_USB_HEADER_SIZE;
    report[5] = (uint8_t)(len >> 8);
    report[6] = (uint8_t)len;
    report[7] = KNX_USB_BUS_ACCESS;
    report[8] = service;
    if (len > 0) memcpy(report + 11, body, len);

    return hid_class_request_set_report(dev, HID_REPORT_TYPE_OUTPUT,
                                         KNX_HID_REPORT_ID, report, sizeof(report));
}

static void knx_hid_interface_event(hid_host_device_handle_t dev,
                                     hid_host_interface_event_t event, void *arg)
{
    (void)arg;
    if (event == HID_HOST_INTERFACE_EVENT_INPUT_REPORT) {
        bool ours;
        portENTER_CRITICAL(&s_monitor_lock);
        ours = (dev == s_active_device);
        portEXIT_CRITICAL(&s_monitor_lock);
        if (!ours) return;

        uint8_t data[KNX_USB_REPORT_SIZE];
        size_t len = 0;
        if (hid_host_device_get_raw_input_report_data(dev, data, sizeof(data), &len) == ESP_OK) {
            knx_process_hid_report(data, len);
        } else {
            report_malformed();
        }
    } else if (event == HID_HOST_INTERFACE_EVENT_DISCONNECTED) {
        knx_hid_event_t item = {.type = KNX_HID_DISCONNECTED, .handle = dev};
        (void)xQueueSend(s_hid_queue, &item, 0);
    } else if (event == HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR) {
        report_malformed();
    }
}

static void knx_hid_driver_event(hid_host_device_handle_t dev,
                                  hid_host_driver_event_t event, void *arg)
{
    (void)arg;
    if (event == HID_HOST_DRIVER_EVENT_CONNECTED) {
        knx_hid_event_t item = {.type = KNX_HID_CONNECTED, .handle = dev};
        (void)xQueueSend(s_hid_queue, &item, 0);
    }
}

static void knx_hid_task(void *arg)
{
    (void)arg;
    knx_hid_event_t item;
    while (xQueueReceive(s_hid_queue, &item, portMAX_DELAY) == pdTRUE) {
        if (item.type == KNX_HID_DISCONNECTED) {
            portENTER_CRITICAL(&s_monitor_lock);
            if (s_active_device == item.handle) {
                s_active_device = NULL;
                s_status.connected = false;
                s_status.bus_status_known = false;
                s_status.bus_connected = false;
                s_status.active_emi = 0;
                s_status.disconnects++;
                s_assembly.running = false;
            }
            portEXIT_CRITICAL(&s_monitor_lock);
            (void)hid_host_device_close(item.handle);
            ESP_LOGI(TAG, "HID interface disconnected");
            continue;
        }

        hid_host_device_config_t cfg = {
            .callback = knx_hid_interface_event,
            .callback_arg = NULL,
        };
        esp_err_t err = hid_host_device_open(item.handle, &cfg);
        if (err != ESP_OK) continue;

        size_t report_desc_len = 0;
        uint8_t *report_desc = hid_host_get_report_descriptor(item.handle, &report_desc_len);
        if (!knx_hid_report_descriptor(report_desc, report_desc_len)) {
            (void)hid_host_device_close(item.handle);
            continue; /* unrelated USB HID device */
        }

        bool busy;
        portENTER_CRITICAL(&s_monitor_lock);
        busy = s_active_device != NULL;
        portEXIT_CRITICAL(&s_monitor_lock);
        if (busy) {
            (void)hid_host_device_close(item.handle);
            continue;
        }

        hid_host_dev_info_t info = {0};
        (void)hid_host_get_device_info(item.handle, &info);
        portENTER_CRITICAL(&s_monitor_lock);
        s_active_device = item.handle;
        s_status.connected = true;
        s_status.vid = info.VID;
        s_status.pid = info.PID;
        s_status.bus_status_known = false;
        s_status.bus_connected = false;
        s_status.active_emi = 0;
        s_assembly.running = false;
        portEXIT_CRITICAL(&s_monitor_lock);

        err = hid_host_device_start(item.handle);
        if (err != ESP_OK) {
            portENTER_CRITICAL(&s_monitor_lock);
            s_active_device = NULL;
            s_status.connected = false;
            portEXIT_CRITICAL(&s_monitor_lock);
            (void)hid_host_device_close(item.handle);
            ESP_LOGW(TAG, "KNX HID input failed: %s", esp_err_to_name(err));
            continue;
        }

        ESP_LOGI(TAG, "KNX HID connected: %04x:%04x", info.VID, info.PID);
        const uint8_t enable_cemi[] = {KNX_FEATURE_ACTIVE_EMI, KNX_USB_CEMI};
        const uint8_t query_cemi[] = {KNX_FEATURE_ACTIVE_EMI};
        const uint8_t query_bus[] = {KNX_FEATURE_BUS_STATUS};

        /* Selection is an interface feature, not a KNX bus write. */
        err = knx_send_feature(item.handle, KNX_FEATURE_SET, enable_cemi, sizeof(enable_cemi));
        if (err != ESP_OK) ESP_LOGW(TAG, "Select cEMI: %s", esp_err_to_name(err));
        err = knx_send_feature(item.handle, KNX_FEATURE_GET, query_cemi, sizeof(query_cemi));
        if (err != ESP_OK) ESP_LOGW(TAG, "Query active EMI: %s", esp_err_to_name(err));
        err = knx_send_feature(item.handle, KNX_FEATURE_GET, query_bus, sizeof(query_bus));
        if (err != ESP_OK) ESP_LOGW(TAG, "Query KNX bus: %s", esp_err_to_name(err));
    }
}

void knx_usb_monitor_start(void)
{
    s_hid_queue = xQueueCreate(12, sizeof(knx_hid_event_t));
    configASSERT(s_hid_queue != NULL);
    BaseType_t ok = xTaskCreate(knx_hid_task, "knx_hid", 6144, NULL, 7, NULL);
    configASSERT(ok == pdPASS);

    hid_host_driver_config_t cfg = {
        .create_background_task = true,
        .task_priority = 8,
        .stack_size = 5120,
        .core_id = tskNO_AFFINITY,
        .callback = knx_hid_driver_event,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK(hid_host_install(&cfg));
    ESP_LOGI(TAG, "KNX USB HID monitor ready (passive KNX capture)");
}

void knx_usb_monitor_get_status(knx_usb_status_t *out)
{
    if (out == NULL) return;
    portENTER_CRITICAL(&s_monitor_lock);
    *out = s_status;
    portEXIT_CRITICAL(&s_monitor_lock);
    out->uptime_us = (uint64_t)esp_timer_get_time();
}

bool knx_usb_monitor_get_event(uint32_t seq, knx_usb_event_t *out)
{
    if (out == NULL || seq == 0) return false;
    bool found = false;
    portENTER_CRITICAL(&s_monitor_lock);
    if (seq <= s_status.last_seq &&
        s_status.last_seq - seq < KNX_USB_HISTORY_SIZE) {
        const knx_usb_event_t *event = &s_events[(seq - 1U) % KNX_USB_HISTORY_SIZE];
        if (event->seq == seq) {
            *out = *event;
            found = true;
        }
    }
    portEXIT_CRITICAL(&s_monitor_lock);
    return found;
}
