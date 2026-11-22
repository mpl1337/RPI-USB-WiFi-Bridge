#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Passive KNX USB HID/cEMI capture. No KNX group/individual telegrams are sent.
 * The only USB output reports select/query the interface's cEMI mode.
 */
#define KNX_USB_RAW_MAX       272U
#define KNX_USB_HISTORY_SIZE  128U

typedef struct {
    uint32_t seq;
    uint64_t received_us; /* ESP32 monotonic microseconds, not KNX bus time */
    uint16_t raw_len;
    uint16_t source;
    uint16_t destination;
    uint8_t message_code;
    uint8_t ctrl1;
    uint8_t ctrl2;
    bool addresses_valid;
    bool group_address;
    uint8_t raw[KNX_USB_RAW_MAX]; /* full unmodified cEMI telegram */
} knx_usb_event_t;

typedef struct {
    bool connected;
    bool bus_connected;
    bool bus_status_known;
    uint16_t vid;
    uint16_t pid;
    uint8_t active_emi; /* 0 = not reported, 3 = cEMI */
    uint32_t last_seq;
    uint32_t captured;
    uint32_t group_telegrams;
    uint32_t busmonitor_telegrams;
    uint32_t malformed_reports;
    uint32_t disconnects;
    uint64_t uptime_us;
} knx_usb_status_t;

/* Call after usb_cdc_bridge_start(): it owns the one USB Host library. */
void knx_usb_monitor_start(void);
void knx_usb_monitor_get_status(knx_usb_status_t *out);
bool knx_usb_monitor_get_event(uint32_t seq, knx_usb_event_t *out);
