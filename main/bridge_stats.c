#include "bridge_stats.h"

#include <string.h>
#include "freertos/FreeRTOS.h"

static bridge_stats_snapshot_t s_stats;
static portMUX_TYPE s_stats_mux = portMUX_INITIALIZER_UNLOCKED;

void bridge_stats_init(void)
{
    bridge_stats_reset();
}

void bridge_stats_reset(void)
{
    portENTER_CRITICAL(&s_stats_mux);
    memset(&s_stats, 0, sizeof(s_stats));
    portEXIT_CRITICAL(&s_stats_mux);
}

#define ADD_STAT(field, value) \
    do { \
        portENTER_CRITICAL(&s_stats_mux); \
        s_stats.field += (uint64_t)(value); \
        portEXIT_CRITICAL(&s_stats_mux); \
    } while (0)

void bridge_stats_add_usb_rx(size_t n) { ADD_STAT(usb_rx_bytes, n); }
void bridge_stats_add_usb_tx(size_t n) { ADD_STAT(usb_tx_bytes, n); }
void bridge_stats_add_tcp_rx(size_t n) { ADD_STAT(tcp_rx_bytes, n); }
void bridge_stats_add_tcp_tx(size_t n) { ADD_STAT(tcp_tx_bytes, n); }
void bridge_stats_add_ws_rx(size_t n) { ADD_STAT(ws_rx_bytes, n); }
void bridge_stats_add_ws_tx(size_t n) { ADD_STAT(ws_tx_bytes, n); }
void bridge_stats_add_dropped_usb_to_tcp(size_t n) { ADD_STAT(dropped_usb_to_tcp, n); }
void bridge_stats_add_dropped_usb_to_ws(size_t n) { ADD_STAT(dropped_usb_to_ws, n); }
void bridge_stats_add_dropped_to_usb(size_t n) { ADD_STAT(dropped_to_usb, n); }

void bridge_stats_get(bridge_stats_snapshot_t *out)
{
    if (out == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_stats_mux);
    *out = s_stats;
    portEXIT_CRITICAL(&s_stats_mux);
}
