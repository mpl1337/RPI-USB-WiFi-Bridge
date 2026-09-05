#pragma once

#include <stdbool.h>

typedef enum {
    TCP_BRIDGE_CLIENT_NONE = 0,
    TCP_BRIDGE_CLIENT_AUTO,
    TCP_BRIDGE_CLIENT_RAW,
    TCP_BRIDGE_CLIENT_RFC2217,
} tcp_bridge_client_mode_t;

void tcp_bridge_start(void);
bool tcp_bridge_is_client_connected(void);
tcp_bridge_client_mode_t tcp_bridge_get_client_mode(void);
const char *tcp_bridge_client_mode_name(void);
