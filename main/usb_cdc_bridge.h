#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "rp_mcu.h"

typedef struct {
    bool valid;
    uint16_t vid;
    uint16_t pid;
    uint16_t bcd_usb;
    uint16_t bcd_device;
    uint8_t dev_class;
    uint8_t dev_subclass;
    uint8_t dev_protocol;
    uint8_t dev_addr;
    uint8_t speed;
    rp_mcu_family_t family;
    char manufacturer[64];
    char product[64];
    char serial[64];
} usb_cdc_device_info_t;

typedef struct {
    bool valid;
    uint16_t app_id;
    char name[96];
    char version[24];
    char number[16];
} usb_cdc_openknx_info_t;

typedef struct {
    bool valid;
    uint32_t baud;
    uint8_t data_bits;
    uint8_t parity_type;
    uint8_t char_format;
    bool dtr;
    bool rts;
} usb_cdc_serial_state_t;

typedef struct {
    bool reconnect_paused;
    bool reconnect_hold;
    bool normal_target_present;
    uint32_t open_fail_count;
    int32_t last_open_error;
    uint32_t connect_count;
    uint32_t disconnect_count;
    uint32_t event_seq;
    uint64_t last_event_ms;
    char last_event[128];
} usb_cdc_diag_t;

void usb_cdc_bridge_start(void);
bool usb_cdc_bridge_is_connected(void);
void usb_cdc_bridge_set_tcp_client_state(bool connected);
void usb_cdc_bridge_set_ws_client_state(bool connected);
bool usb_cdc_bridge_get_device_info(usb_cdc_device_info_t *out);
esp_err_t usb_cdc_bridge_refresh_device_info(uint32_t timeout_ms);

bool usb_cdc_bridge_get_openknx_info(usb_cdc_openknx_info_t *out);
void usb_cdc_bridge_clear_openknx_info(void);


esp_err_t usb_cdc_bridge_query_openknx_info(usb_cdc_openknx_info_t *out, uint32_t timeout_ms);

esp_err_t usb_cdc_bridge_send_direct(const uint8_t *data, size_t len, uint32_t timeout_ms);
esp_err_t usb_cdc_bridge_set_baud(uint32_t baud, uint32_t timeout_ms);
esp_err_t usb_cdc_bridge_set_line_coding(uint32_t baud, uint8_t data_bits, uint8_t parity_type, uint8_t char_format, uint32_t timeout_ms);
esp_err_t usb_cdc_bridge_set_control_lines(bool dtr, bool rts, uint32_t timeout_ms);
bool usb_cdc_bridge_get_serial_state(usb_cdc_serial_state_t *out);


esp_err_t usb_cdc_bridge_openknx_1200_touch(uint32_t normal_baud, uint32_t timeout_ms);


esp_err_t usb_cdc_bridge_restore_normal_state(uint32_t normal_baud, uint32_t timeout_ms);


void usb_cdc_bridge_set_reconnect_paused(bool paused);


void usb_cdc_bridge_set_reconnect_hold(bool hold);
bool usb_cdc_bridge_reconnect_is_paused(void);


void usb_cdc_bridge_get_diag(usb_cdc_diag_t *out);


bool usb_cdc_bridge_wait_released(uint32_t timeout_ms);
