#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint64_t usb_rx_bytes;
    uint64_t usb_tx_bytes;
    uint64_t tcp_rx_bytes;
    uint64_t tcp_tx_bytes;
    uint64_t ws_rx_bytes;
    uint64_t ws_tx_bytes;
    uint64_t dropped_usb_to_tcp;
    uint64_t dropped_usb_to_ws;
    uint64_t dropped_to_usb;
} bridge_stats_snapshot_t;

void bridge_stats_init(void);
void bridge_stats_reset(void);
void bridge_stats_add_usb_rx(size_t n);
void bridge_stats_add_usb_tx(size_t n);
void bridge_stats_add_tcp_rx(size_t n);
void bridge_stats_add_tcp_tx(size_t n);
void bridge_stats_add_ws_rx(size_t n);
void bridge_stats_add_ws_tx(size_t n);
void bridge_stats_add_dropped_usb_to_tcp(size_t n);
void bridge_stats_add_dropped_usb_to_ws(size_t n);
void bridge_stats_add_dropped_to_usb(size_t n);
void bridge_stats_get(bridge_stats_snapshot_t *out);
