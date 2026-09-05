#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_http_server.h"
#include "rp_mcu.h"

typedef enum {
    FW_UPDATE_IDLE = 0,
    FW_UPDATE_VALIDATING,
    FW_UPDATE_STAGING,
    FW_UPDATE_ENTERING_BOOTLOADER,
    FW_UPDATE_WAITING_MSC,
    FW_UPDATE_WRITING,
    FW_UPDATE_WAITING_REBOOT,
    FW_UPDATE_STARTING_FIRMWARE,
    FW_UPDATE_NUKING,
    FW_UPDATE_ESP32_OTA,
    FW_UPDATE_SUCCESS,
    FW_UPDATE_ERROR,
} firmware_update_state_t;

typedef struct {
    firmware_update_state_t state;
    bool busy;
    bool msc_ready;
    bool staged_ready;
    rp_mcu_family_t bootsel_family;
    rp_mcu_family_t staged_family;
    uint32_t staged_family_id;
    size_t received_bytes;
    size_t total_bytes;
    uint32_t blocks_written;
    uint32_t total_blocks;
    char message[224];
} firmware_update_status_t;

void firmware_update_start(void);
bool firmware_update_msc_ready(void);
void firmware_update_get_status(firmware_update_status_t *out);
const char *firmware_update_state_name(firmware_update_state_t state);
esp_err_t firmware_update_http_handler(httpd_req_t *req);
esp_err_t firmware_update_retry_http_handler(httpd_req_t *req);
esp_err_t firmware_update_bootloader_http_handler(httpd_req_t *req);
esp_err_t firmware_update_start_firmware_http_handler(httpd_req_t *req);
esp_err_t firmware_update_nuke_http_handler(httpd_req_t *req);
esp_err_t firmware_update_esp32_http_handler(httpd_req_t *req);
