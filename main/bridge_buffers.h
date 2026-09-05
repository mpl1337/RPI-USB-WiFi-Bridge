#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"


#define BRIDGE_WS_MAX_CLIENTS 2U

void bridge_buffers_init(void);


size_t bridge_usb_to_net_push(const uint8_t *data, size_t len);
size_t bridge_usb_to_net_pop(uint8_t *data, size_t max_len, TickType_t wait);
void bridge_usb_to_net_reset(void);


void bridge_usb_to_ws_client_activate(size_t client_index);
void bridge_usb_to_ws_client_deactivate(size_t client_index);
bool bridge_usb_to_ws_client_is_active(size_t client_index);
size_t bridge_usb_to_ws_broadcast(const uint8_t *data, size_t len);
size_t bridge_usb_to_ws_pop_client(size_t client_index, uint8_t *data, size_t max_len, TickType_t wait);
void bridge_usb_to_ws_reset(void);


size_t bridge_net_to_usb_push(const uint8_t *data, size_t len);
size_t bridge_net_to_usb_pop(uint8_t *data, size_t max_len, TickType_t wait);
void bridge_net_to_usb_reset(void);
