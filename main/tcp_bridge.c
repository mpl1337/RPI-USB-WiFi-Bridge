#include "tcp_bridge.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_app_desc.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"
#include "bridge_buffers.h"
#include "usb_cdc_bridge.h"
#include "bridge_stats.h"

static const char *TAG = "tcp";
static volatile bool s_client_connected;
static volatile tcp_bridge_client_mode_t s_client_mode = TCP_BRIDGE_CLIENT_NONE;

#define TELNET_SE   240U
#define TELNET_SB   250U
#define TELNET_WILL 251U
#define TELNET_WONT 252U
#define TELNET_DO   253U
#define TELNET_DONT 254U
#define TELNET_IAC  255U

#define TELNET_BINARY 0U
#define TELNET_ECHO   1U
#define TELNET_SGA    3U
#define RFC2217_COM_PORT_OPTION 44U

#define RFC2217_SIGNATURE          0U
#define RFC2217_SET_BAUDRATE       1U
#define RFC2217_SET_DATASIZE       2U
#define RFC2217_SET_PARITY         3U
#define RFC2217_SET_STOPSIZE       4U
#define RFC2217_SET_CONTROL        5U
#define RFC2217_NOTIFY_LINESTATE   6U
#define RFC2217_NOTIFY_MODEMSTATE  7U
#define RFC2217_FLOW_SUSPEND       8U
#define RFC2217_FLOW_RESUME        9U
#define RFC2217_SET_LINE_MASK     10U
#define RFC2217_SET_MODEM_MASK    11U
#define RFC2217_PURGE_DATA        12U
#define RFC2217_SERVER_OFFSET    100U

#define RFC2217_CTRL_REQ_FLOW       0U
#define RFC2217_CTRL_NO_FLOW        1U
#define RFC2217_CTRL_SW_FLOW        2U
#define RFC2217_CTRL_HW_FLOW        3U
#define RFC2217_CTRL_REQ_BREAK      4U
#define RFC2217_CTRL_BREAK_ON       5U
#define RFC2217_CTRL_BREAK_OFF      6U
#define RFC2217_CTRL_REQ_DTR        7U
#define RFC2217_CTRL_DTR_ON         8U
#define RFC2217_CTRL_DTR_OFF        9U
#define RFC2217_CTRL_REQ_RTS       10U
#define RFC2217_CTRL_RTS_ON        11U
#define RFC2217_CTRL_RTS_OFF       12U
#define RFC2217_CTRL_REQ_FLOW_IN   13U
#define RFC2217_CTRL_NO_FLOW_IN    14U
#define RFC2217_CTRL_SW_FLOW_IN    15U
#define RFC2217_CTRL_HW_FLOW_IN    16U

#define RFC2217_PURGE_RX 1U
#define RFC2217_PURGE_TX 2U
#define RFC2217_PURGE_BOTH 3U

#define RFC2217_SUBNEG_MAX 96U

typedef enum {
    RFC_PARSE_DATA = 0,
    RFC_PARSE_IAC,
    RFC_PARSE_NEGOTIATE,
    RFC_PARSE_SB,
    RFC_PARSE_SB_IAC,
} rfc_parse_state_t;

typedef struct {
    rfc_parse_state_t state;
    uint8_t negotiate_cmd;
    uint8_t subneg[RFC2217_SUBNEG_MAX];
    size_t subneg_len;
} rfc2217_parser_t;

bool tcp_bridge_is_client_connected(void)
{
    return s_client_connected;
}

tcp_bridge_client_mode_t tcp_bridge_get_client_mode(void)
{
    return s_client_mode;
}

const char *tcp_bridge_client_mode_name(void)
{
    switch (s_client_mode) {
        case TCP_BRIDGE_CLIENT_RAW: return "raw";
        case TCP_BRIDGE_CLIENT_RFC2217: return "rfc2217";
        case TCP_BRIDGE_CLIENT_AUTO: return "auto";
        default: return "none";
    }
}

static void close_client(int *fd)
{
    if (*fd >= 0) {
        shutdown(*fd, SHUT_RDWR);
        close(*fd);
        *fd = -1;
    }
    s_client_connected = false;
    s_client_mode = TCP_BRIDGE_CLIENT_NONE;
    usb_cdc_bridge_set_tcp_client_state(false);
    ESP_LOGI(TAG, "TCP client disconnected");
}

static bool send_all(int fd, const uint8_t *data, size_t len)
{
    size_t sent = 0;
    while (sent < len) {
        int ret = send(fd, data + sent, len - sent, 0);
        if (ret <= 0) {
            return false;
        }
        sent += (size_t)ret;
    }
    return true;
}

static bool telnet_option_supported(uint8_t option)
{
    return option == TELNET_BINARY || option == TELNET_ECHO || option == TELNET_SGA ||
           option == RFC2217_COM_PORT_OPTION;
}

static bool send_telnet_option(int fd, uint8_t action, uint8_t option)
{
    const uint8_t msg[3] = {TELNET_IAC, action, option};
    return send_all(fd, msg, sizeof(msg));
}

static bool handle_telnet_negotiation(int fd, uint8_t command, uint8_t option)
{
    const bool supported = telnet_option_supported(option);
    uint8_t answer;
    switch (command) {
        case TELNET_DO:
            answer = supported ? TELNET_WILL : TELNET_WONT;
            break;
        case TELNET_WILL:
            answer = supported ? TELNET_DO : TELNET_DONT;
            break;
        case TELNET_DONT:
            answer = TELNET_WONT;
            break;
        case TELNET_WONT:
            answer = TELNET_DONT;
            break;
        default:
            return true;
    }
    return send_telnet_option(fd, answer, option);
}

static bool send_rfc2217_subneg(int fd, uint8_t command, const uint8_t *value, size_t value_len)
{
    uint8_t frame[2U * RFC2217_SUBNEG_MAX + 8U];
    size_t out = 0;
    frame[out++] = TELNET_IAC;
    frame[out++] = TELNET_SB;
    frame[out++] = RFC2217_COM_PORT_OPTION;
    frame[out++] = command;
    for (size_t i = 0; i < value_len; ++i) {
        if (out + 2 >= sizeof(frame)) {
            return false;
        }
        frame[out++] = value[i];
        if (value[i] == TELNET_IAC) {
            frame[out++] = TELNET_IAC;
        }
    }
    frame[out++] = TELNET_IAC;
    frame[out++] = TELNET_SE;
    return send_all(fd, frame, out);
}

static usb_cdc_serial_state_t current_serial_state(void)
{
    usb_cdc_serial_state_t state = {
        .valid = false,
        .baud = CONFIG_BRIDGE_CDC_BAUD,
        .data_bits = 8,
        .parity_type = 0,
        .char_format = 0,
        .dtr = true,
        .rts = true,
    };
    usb_cdc_serial_state_t actual = {0};
    if (usb_cdc_bridge_get_serial_state(&actual)) {
        state = actual;
    }
    return state;
}

static bool apply_line_coding(usb_cdc_serial_state_t *state)
{
    esp_err_t err = usb_cdc_bridge_set_line_coding(state->baud, state->data_bits,
                                                    state->parity_type, state->char_format, 1000);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "RFC2217 line coding rejected: %s", esp_err_to_name(err));
        *state = current_serial_state();
        return false;
    }
    *state = current_serial_state();
    return true;
}

static bool apply_control_lines(usb_cdc_serial_state_t *state)
{
    esp_err_t err = usb_cdc_bridge_set_control_lines(state->dtr, state->rts, 1000);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "RFC2217 control lines rejected: %s", esp_err_to_name(err));
        *state = current_serial_state();
        return false;
    }
    *state = current_serial_state();
    return true;
}

static uint8_t rfc_stop_from_cdc(uint8_t char_format)
{
    if (char_format == 2U) return 2U;
    if (char_format == 1U) return 3U;
    return 1U;
}

static bool handle_rfc2217_subneg(int fd, const uint8_t *data, size_t len)
{
    if (len < 2 || data[0] != RFC2217_COM_PORT_OPTION) {
        return true;
    }

    const uint8_t command = data[1];
    const uint8_t *value = data + 2;
    const size_t value_len = len - 2;
    usb_cdc_serial_state_t state = current_serial_state();

    if (command == RFC2217_SIGNATURE) {
        const esp_app_desc_t *app_desc = esp_app_get_description();
        const char *version = (app_desc != NULL && app_desc->version[0] != '\0') ? app_desc->version : "?";
        char signature[64];
        int signature_len = snprintf(signature, sizeof(signature),
                                     "ESP32-S3 USB WiFi Bridge v%s", version);
        if (signature_len < 0) {
            return false;
        }
        size_t send_len = (size_t)signature_len;
        if (send_len >= sizeof(signature)) {
            send_len = sizeof(signature) - 1U;
        }
        return send_rfc2217_subneg(fd, RFC2217_SIGNATURE,
                                   (const uint8_t *)signature, send_len);
    }

    if (command == RFC2217_SET_BAUDRATE && value_len >= 4) {
        uint32_t requested = ((uint32_t)value[0] << 24) | ((uint32_t)value[1] << 16) |
                             ((uint32_t)value[2] << 8) | (uint32_t)value[3];
        if (requested != 0U) {
            state.baud = requested;
            (void)apply_line_coding(&state);
        }
        uint8_t answer[4] = {
            (uint8_t)(state.baud >> 24), (uint8_t)(state.baud >> 16),
            (uint8_t)(state.baud >> 8), (uint8_t)state.baud
        };
        return send_rfc2217_subneg(fd, RFC2217_SET_BAUDRATE + RFC2217_SERVER_OFFSET, answer, sizeof(answer));
    }

    if (command == RFC2217_SET_DATASIZE && value_len >= 1) {
        const uint8_t requested = value[0];
        if (requested >= 5U && requested <= 8U) {
            state.data_bits = requested;
            (void)apply_line_coding(&state);
        }
        const uint8_t answer = state.data_bits;
        return send_rfc2217_subneg(fd, RFC2217_SET_DATASIZE + RFC2217_SERVER_OFFSET, &answer, 1);
    }

    if (command == RFC2217_SET_PARITY && value_len >= 1) {
        const uint8_t requested = value[0];
        if (requested >= 1U && requested <= 5U) {
            state.parity_type = requested - 1U;
            (void)apply_line_coding(&state);
        }
        const uint8_t answer = state.parity_type + 1U;
        return send_rfc2217_subneg(fd, RFC2217_SET_PARITY + RFC2217_SERVER_OFFSET, &answer, 1);
    }

    if (command == RFC2217_SET_STOPSIZE && value_len >= 1) {
        const uint8_t requested = value[0];
        if (requested == 1U || requested == 2U || requested == 3U) {
            state.char_format = requested == 1U ? 0U : (requested == 2U ? 2U : 1U);
            (void)apply_line_coding(&state);
        }
        const uint8_t answer = rfc_stop_from_cdc(state.char_format);
        return send_rfc2217_subneg(fd, RFC2217_SET_STOPSIZE + RFC2217_SERVER_OFFSET, &answer, 1);
    }

    if (command == RFC2217_SET_CONTROL && value_len >= 1) {
        const uint8_t requested = value[0];
        uint8_t answer = requested;
        switch (requested) {
            case RFC2217_CTRL_REQ_FLOW:
            case RFC2217_CTRL_NO_FLOW:
            case RFC2217_CTRL_SW_FLOW:
            case RFC2217_CTRL_HW_FLOW:
                answer = RFC2217_CTRL_NO_FLOW;
                break;
            case RFC2217_CTRL_REQ_BREAK:
            case RFC2217_CTRL_BREAK_ON:
            case RFC2217_CTRL_BREAK_OFF:

                answer = RFC2217_CTRL_BREAK_OFF;
                break;
            case RFC2217_CTRL_REQ_DTR:
                answer = state.dtr ? RFC2217_CTRL_DTR_ON : RFC2217_CTRL_DTR_OFF;
                break;
            case RFC2217_CTRL_DTR_ON:
            case RFC2217_CTRL_DTR_OFF:
                state.dtr = requested == RFC2217_CTRL_DTR_ON;
                (void)apply_control_lines(&state);
                answer = state.dtr ? RFC2217_CTRL_DTR_ON : RFC2217_CTRL_DTR_OFF;
                break;
            case RFC2217_CTRL_REQ_RTS:
                answer = state.rts ? RFC2217_CTRL_RTS_ON : RFC2217_CTRL_RTS_OFF;
                break;
            case RFC2217_CTRL_RTS_ON:
            case RFC2217_CTRL_RTS_OFF:
                state.rts = requested == RFC2217_CTRL_RTS_ON;
                (void)apply_control_lines(&state);
                answer = state.rts ? RFC2217_CTRL_RTS_ON : RFC2217_CTRL_RTS_OFF;
                break;
            case RFC2217_CTRL_REQ_FLOW_IN:
            case RFC2217_CTRL_NO_FLOW_IN:
            case RFC2217_CTRL_SW_FLOW_IN:
            case RFC2217_CTRL_HW_FLOW_IN:
                answer = RFC2217_CTRL_NO_FLOW_IN;
                break;
            default:
                break;
        }
        return send_rfc2217_subneg(fd, RFC2217_SET_CONTROL + RFC2217_SERVER_OFFSET, &answer, 1);
    }

    if (command == RFC2217_NOTIFY_LINESTATE) {
        const uint8_t line_state = 0U;
        return send_rfc2217_subneg(fd, RFC2217_NOTIFY_LINESTATE + RFC2217_SERVER_OFFSET, &line_state, 1);
    }

    if (command == RFC2217_NOTIFY_MODEMSTATE) {

        const uint8_t modem_state = 0U;
        return send_rfc2217_subneg(fd, RFC2217_NOTIFY_MODEMSTATE + RFC2217_SERVER_OFFSET, &modem_state, 1);
    }

    if ((command == RFC2217_SET_LINE_MASK || command == RFC2217_SET_MODEM_MASK) && value_len >= 1) {
        const uint8_t answer = value[0];
        return send_rfc2217_subneg(fd, command + RFC2217_SERVER_OFFSET, &answer, 1);
    }

    if (command == RFC2217_PURGE_DATA && value_len >= 1) {
        const uint8_t requested = value[0];
        if (requested == RFC2217_PURGE_RX || requested == RFC2217_PURGE_BOTH) {
            bridge_usb_to_net_reset();
        }
        if (requested == RFC2217_PURGE_TX || requested == RFC2217_PURGE_BOTH) {
            bridge_net_to_usb_reset();
        }
        return send_rfc2217_subneg(fd, RFC2217_PURGE_DATA + RFC2217_SERVER_OFFSET, &requested, 1);
    }

    if (command == RFC2217_FLOW_SUSPEND || command == RFC2217_FLOW_RESUME) {
        return send_rfc2217_subneg(fd, command + RFC2217_SERVER_OFFSET, NULL, 0);
    }

    return true;
}

static bool rfc2217_process_input(rfc2217_parser_t *parser, int fd,
                                  const uint8_t *input, size_t input_len,
                                  uint8_t *serial_data, size_t serial_capacity,
                                  size_t *serial_len)
{
    *serial_len = 0;
    for (size_t i = 0; i < input_len; ++i) {
        const uint8_t byte = input[i];
        switch (parser->state) {
            case RFC_PARSE_DATA:
                if (byte == TELNET_IAC) {
                    parser->state = RFC_PARSE_IAC;
                } else if (*serial_len < serial_capacity) {
                    serial_data[(*serial_len)++] = byte;
                }
                break;

            case RFC_PARSE_IAC:
                if (byte == TELNET_IAC) {
                    if (*serial_len < serial_capacity) {
                        serial_data[(*serial_len)++] = TELNET_IAC;
                    }
                    parser->state = RFC_PARSE_DATA;
                } else if (byte == TELNET_WILL || byte == TELNET_WONT ||
                           byte == TELNET_DO || byte == TELNET_DONT) {
                    parser->negotiate_cmd = byte;
                    parser->state = RFC_PARSE_NEGOTIATE;
                } else if (byte == TELNET_SB) {
                    parser->subneg_len = 0;
                    parser->state = RFC_PARSE_SB;
                } else {
                    parser->state = RFC_PARSE_DATA;
                }
                break;

            case RFC_PARSE_NEGOTIATE:
                if (!handle_telnet_negotiation(fd, parser->negotiate_cmd, byte)) {
                    return false;
                }
                parser->state = RFC_PARSE_DATA;
                break;

            case RFC_PARSE_SB:
                if (byte == TELNET_IAC) {
                    parser->state = RFC_PARSE_SB_IAC;
                } else if (parser->subneg_len < sizeof(parser->subneg)) {
                    parser->subneg[parser->subneg_len++] = byte;
                }
                break;

            case RFC_PARSE_SB_IAC:
                if (byte == TELNET_IAC) {
                    if (parser->subneg_len < sizeof(parser->subneg)) {
                        parser->subneg[parser->subneg_len++] = TELNET_IAC;
                    }
                    parser->state = RFC_PARSE_SB;
                } else if (byte == TELNET_SE) {
                    if (!handle_rfc2217_subneg(fd, parser->subneg, parser->subneg_len)) {
                        return false;
                    }
                    parser->subneg_len = 0;
                    parser->state = RFC_PARSE_DATA;
                } else {
                    parser->subneg_len = 0;
                    parser->state = RFC_PARSE_DATA;
                }
                break;
        }
    }
    return true;
}

static bool send_rfc2217_data(int fd, const uint8_t *data, size_t len)
{
    uint8_t escaped[2048];
    size_t out = 0;
    for (size_t i = 0; i < len; ++i) {
        if (out + 2 > sizeof(escaped)) {
            if (!send_all(fd, escaped, out)) return false;
            out = 0;
        }
        escaped[out++] = data[i];
        if (data[i] == TELNET_IAC) {
            escaped[out++] = TELNET_IAC;
        }
    }
    return out == 0 || send_all(fd, escaped, out);
}

static tcp_bridge_client_mode_t detect_client_mode(const uint8_t *probe, size_t len)
{
    if (len == 0) return TCP_BRIDGE_CLIENT_AUTO;
    if (probe[0] != TELNET_IAC) return TCP_BRIDGE_CLIENT_RAW;
    if (len < 2) return TCP_BRIDGE_CLIENT_AUTO;
    if (probe[1] == TELNET_WILL || probe[1] == TELNET_WONT ||
        probe[1] == TELNET_DO || probe[1] == TELNET_DONT) {
        return len >= 3 ? TCP_BRIDGE_CLIENT_RFC2217 : TCP_BRIDGE_CLIENT_AUTO;
    }
    if (probe[1] == TELNET_SB) {
        return len >= 3 ? TCP_BRIDGE_CLIENT_RFC2217 : TCP_BRIDGE_CLIENT_AUTO;
    }
    return TCP_BRIDGE_CLIENT_RAW;
}

static void push_to_usb(const uint8_t *data, size_t len)
{
    if (len == 0) return;
    if (usb_cdc_bridge_is_connected()) {
        size_t pushed = bridge_net_to_usb_push(data, len);
        if (pushed != len) {
            size_t dropped = len - pushed;
            bridge_stats_add_dropped_to_usb(dropped);
            ESP_LOGW(TAG, "TCP->USB buffer full, dropped %u bytes", (unsigned)dropped);
        }
    } else {
        bridge_stats_add_dropped_to_usb(len);
    }
}

static void tcp_server_task(void *arg)
{
    (void)arg;
    int listen_fd = -1;

    while (true) {
        listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (listen_fd < 0) {
            ESP_LOGE(TAG, "socket failed: errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        int reuse = 1;
        setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        struct sockaddr_in addr = {
            .sin_family = AF_INET,
            .sin_port = htons(CONFIG_BRIDGE_TCP_PORT),
            .sin_addr.s_addr = htonl(INADDR_ANY),
        };

        if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
            ESP_LOGE(TAG, "bind failed: errno=%d", errno);
            close(listen_fd);
            listen_fd = -1;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (listen(listen_fd, 1) != 0) {
            ESP_LOGE(TAG, "listen failed: errno=%d", errno);
            close(listen_fd);
            listen_fd = -1;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "Listening on TCP port %d (Raw/RFC2217 auto-detect)", CONFIG_BRIDGE_TCP_PORT);

        while (true) {
            struct sockaddr_in peer;
            socklen_t peer_len = sizeof(peer);
            int client_fd = accept(listen_fd, (struct sockaddr *)&peer, &peer_len);
            if (client_fd < 0) {
                ESP_LOGW(TAG, "accept failed: errno=%d", errno);
                break;
            }

            int one = 1;
            setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
            setsockopt(client_fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));

            struct timeval snd_timeout = {.tv_sec = 1, .tv_usec = 0};
            setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &snd_timeout, sizeof(snd_timeout));

            bridge_usb_to_net_reset();
            s_client_connected = true;
            s_client_mode = TCP_BRIDGE_CLIENT_AUTO;
            usb_cdc_bridge_set_tcp_client_state(true);
            ESP_LOGI(TAG, "TCP client connected%s",
                     usb_cdc_bridge_is_connected() ? " - USB ready" : " - waiting for USB");

            uint8_t net_rx[1024];
            uint8_t serial_rx[1024];
            uint8_t usb_rx[1024];
            uint8_t probe[16];
            size_t probe_len = 0;
            rfc2217_parser_t parser = {.state = RFC_PARSE_DATA};
            bool connected = true;

            while (connected) {
                fd_set rfds;
                FD_ZERO(&rfds);
                FD_SET(client_fd, &rfds);
                struct timeval timeout = {.tv_sec = 0, .tv_usec = 20000};

                int sel = select(client_fd + 1, &rfds, NULL, NULL, &timeout);
                if (sel < 0) {
                    ESP_LOGW(TAG, "select failed: errno=%d", errno);
                    break;
                }

                if (sel > 0 && FD_ISSET(client_fd, &rfds)) {
                    int n = recv(client_fd, net_rx, sizeof(net_rx), 0);
                    if (n <= 0) {
                        connected = false;
                    } else {
                        bridge_stats_add_tcp_rx((size_t)n);

                        if (s_client_mode == TCP_BRIDGE_CLIENT_AUTO) {
                            size_t copy = (size_t)n;
                            if (copy > sizeof(probe) - probe_len) copy = sizeof(probe) - probe_len;
                            memcpy(probe + probe_len, net_rx, copy);
                            probe_len += copy;
                            tcp_bridge_client_mode_t detected = detect_client_mode(probe, probe_len);
                            if (detected != TCP_BRIDGE_CLIENT_AUTO || probe_len == sizeof(probe)) {
                                if (detected == TCP_BRIDGE_CLIENT_AUTO) detected = TCP_BRIDGE_CLIENT_RAW;
                                s_client_mode = detected;
                                ESP_LOGI(TAG, "TCP protocol selected: %s", tcp_bridge_client_mode_name());
                                if (detected == TCP_BRIDGE_CLIENT_RFC2217) {
                                    size_t serial_len = 0;
                                    if (!rfc2217_process_input(&parser, client_fd, probe, probe_len,
                                                               serial_rx, sizeof(serial_rx), &serial_len)) {
                                        connected = false;
                                    } else {
                                        push_to_usb(serial_rx, serial_len);
                                    }
                                } else {
                                    push_to_usb(probe, probe_len);
                                }
                                probe_len = 0;
                                if (copy < (size_t)n && connected) {
                                    const uint8_t *rest = net_rx + copy;
                                    size_t rest_len = (size_t)n - copy;
                                    if (detected == TCP_BRIDGE_CLIENT_RFC2217) {
                                        size_t serial_len = 0;
                                        if (!rfc2217_process_input(&parser, client_fd, rest, rest_len,
                                                                   serial_rx, sizeof(serial_rx), &serial_len)) {
                                            connected = false;
                                        } else {
                                            push_to_usb(serial_rx, serial_len);
                                        }
                                    } else {
                                        push_to_usb(rest, rest_len);
                                    }
                                }
                            }
                        } else if (s_client_mode == TCP_BRIDGE_CLIENT_RFC2217) {
                            size_t serial_len = 0;
                            if (!rfc2217_process_input(&parser, client_fd, net_rx, (size_t)n,
                                                       serial_rx, sizeof(serial_rx), &serial_len)) {
                                connected = false;
                            } else {
                                push_to_usb(serial_rx, serial_len);
                            }
                        } else {
                            push_to_usb(net_rx, (size_t)n);
                        }
                    }
                }

                size_t n = bridge_usb_to_net_pop(usb_rx, sizeof(usb_rx), 0);
                if (n > 0 && connected) {
                    bool ok = s_client_mode == TCP_BRIDGE_CLIENT_RFC2217
                                  ? send_rfc2217_data(client_fd, usb_rx, n)
                                  : send_all(client_fd, usb_rx, n);
                    if (!ok) {
                        connected = false;
                    } else {
                        bridge_stats_add_tcp_tx(n);
                    }
                }
            }


            if (probe_len > 0 && s_client_mode == TCP_BRIDGE_CLIENT_AUTO) {
                push_to_usb(probe, probe_len);
            }
            close_client(&client_fd);
        }

        if (listen_fd >= 0) {
            close(listen_fd);
            listen_fd = -1;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void tcp_bridge_start(void)
{
    BaseType_t ok = xTaskCreate(tcp_server_task, "tcp_server", 8192, NULL, 6, NULL);
    configASSERT(ok == pdPASS);
}
