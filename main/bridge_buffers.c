#include "bridge_buffers.h"

#include "freertos/task.h"
#include "sdkconfig.h"

typedef struct {
    uint8_t *data;
    size_t capacity;
    size_t head;
    size_t tail;
    size_t count;
    portMUX_TYPE mux;
} byte_ring_t;

typedef struct {
    uint8_t *data;
    size_t capacity;
    uint64_t write_seq;
    uint64_t read_seq[BRIDGE_WS_MAX_CLIENTS];
    bool active[BRIDGE_WS_MAX_CLIENTS];
    portMUX_TYPE mux;
} fanout_ring_t;

static uint8_t s_usb_to_net_storage[CONFIG_BRIDGE_USB_BUFFER_SIZE];
static uint8_t s_usb_to_ws_storage[CONFIG_BRIDGE_WS_BUFFER_SIZE];
static uint8_t s_net_to_usb_storage[CONFIG_BRIDGE_USB_BUFFER_SIZE];

static byte_ring_t s_usb_to_net = {
    .data = s_usb_to_net_storage, .capacity = sizeof(s_usb_to_net_storage), .mux = portMUX_INITIALIZER_UNLOCKED
};
static fanout_ring_t s_usb_to_ws = {
    .data = s_usb_to_ws_storage, .capacity = sizeof(s_usb_to_ws_storage), .mux = portMUX_INITIALIZER_UNLOCKED
};
static byte_ring_t s_net_to_usb = {
    .data = s_net_to_usb_storage, .capacity = sizeof(s_net_to_usb_storage), .mux = portMUX_INITIALIZER_UNLOCKED
};

static size_t ring_push(byte_ring_t *ring, const uint8_t *data, size_t len)
{
    size_t written = 0;
    portENTER_CRITICAL(&ring->mux);
    while (written < len && ring->count < ring->capacity) {
        ring->data[ring->head] = data[written++];
        ring->head = (ring->head + 1U) % ring->capacity;
        ring->count++;
    }
    portEXIT_CRITICAL(&ring->mux);
    return written;
}

static size_t ring_pop_now(byte_ring_t *ring, uint8_t *data, size_t max_len)
{
    size_t read = 0;
    portENTER_CRITICAL(&ring->mux);
    while (read < max_len && ring->count > 0) {
        data[read++] = ring->data[ring->tail];
        ring->tail = (ring->tail + 1U) % ring->capacity;
        ring->count--;
    }
    portEXIT_CRITICAL(&ring->mux);
    return read;
}

static size_t ring_pop(byte_ring_t *ring, uint8_t *data, size_t max_len, TickType_t wait)
{
    const TickType_t start = xTaskGetTickCount();
    while (true) {
        size_t read = ring_pop_now(ring, data, max_len);
        if (read > 0 || wait == 0) {
            return read;
        }
        if (wait != portMAX_DELAY && (xTaskGetTickCount() - start) >= wait) {
            return 0;
        }
        vTaskDelay(1);
    }
}

static void ring_reset(byte_ring_t *ring)
{
    portENTER_CRITICAL(&ring->mux);
    ring->head = 0;
    ring->tail = 0;
    ring->count = 0;
    portEXIT_CRITICAL(&ring->mux);
}

static size_t fanout_pop_now(fanout_ring_t *ring, size_t client_index, uint8_t *data, size_t max_len)
{
    if (client_index >= BRIDGE_WS_MAX_CLIENTS || data == NULL || max_len == 0) {
        return 0;
    }

    size_t read = 0;
    portENTER_CRITICAL(&ring->mux);
    if (ring->active[client_index]) {
        uint64_t available64 = ring->write_seq - ring->read_seq[client_index];
        size_t available = available64 > SIZE_MAX ? SIZE_MAX : (size_t)available64;
        read = available < max_len ? available : max_len;
        uint64_t seq = ring->read_seq[client_index];
        size_t pos = (size_t)(seq % ring->capacity);
        for (size_t i = 0; i < read; ++i) {
            data[i] = ring->data[pos++];
            if (pos == ring->capacity) {
                pos = 0;
            }
        }
        ring->read_seq[client_index] += read;
    }
    portEXIT_CRITICAL(&ring->mux);
    return read;
}

void bridge_buffers_init(void)
{
    bridge_usb_to_net_reset();
    bridge_usb_to_ws_reset();
    bridge_net_to_usb_reset();
}

size_t bridge_usb_to_net_push(const uint8_t *data, size_t len) { return ring_push(&s_usb_to_net, data, len); }
size_t bridge_usb_to_net_pop(uint8_t *data, size_t max_len, TickType_t wait) { return ring_pop(&s_usb_to_net, data, max_len, wait); }
void bridge_usb_to_net_reset(void) { ring_reset(&s_usb_to_net); }

void bridge_usb_to_ws_client_activate(size_t client_index)
{
    if (client_index >= BRIDGE_WS_MAX_CLIENTS) {
        return;
    }
    portENTER_CRITICAL(&s_usb_to_ws.mux);
    s_usb_to_ws.read_seq[client_index] = s_usb_to_ws.write_seq;
    s_usb_to_ws.active[client_index] = true;
    portEXIT_CRITICAL(&s_usb_to_ws.mux);
}

void bridge_usb_to_ws_client_deactivate(size_t client_index)
{
    if (client_index >= BRIDGE_WS_MAX_CLIENTS) {
        return;
    }
    portENTER_CRITICAL(&s_usb_to_ws.mux);
    s_usb_to_ws.active[client_index] = false;
    s_usb_to_ws.read_seq[client_index] = s_usb_to_ws.write_seq;
    portEXIT_CRITICAL(&s_usb_to_ws.mux);
}

bool bridge_usb_to_ws_client_is_active(size_t client_index)
{
    if (client_index >= BRIDGE_WS_MAX_CLIENTS) {
        return false;
    }
    bool active;
    portENTER_CRITICAL(&s_usb_to_ws.mux);
    active = s_usb_to_ws.active[client_index];
    portEXIT_CRITICAL(&s_usb_to_ws.mux);
    return active;
}

size_t bridge_usb_to_ws_broadcast(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0 || s_usb_to_ws.capacity == 0) {
        return 0;
    }

    size_t aggregate_dropped = 0;
    portENTER_CRITICAL(&s_usb_to_ws.mux);


    const uint8_t *src = data;
    size_t write_len = len;
    uint64_t logical_start = s_usb_to_ws.write_seq;
    if (write_len > s_usb_to_ws.capacity) {
        size_t skipped = write_len - s_usb_to_ws.capacity;
        src += skipped;
        write_len = s_usb_to_ws.capacity;
        logical_start += skipped;
    }

    const uint64_t new_write_seq = s_usb_to_ws.write_seq + len;
    const uint64_t oldest_kept_seq = new_write_seq > s_usb_to_ws.capacity
        ? new_write_seq - s_usb_to_ws.capacity : 0;

    for (size_t i = 0; i < BRIDGE_WS_MAX_CLIENTS; ++i) {
        if (!s_usb_to_ws.active[i]) {
            continue;
        }
        if (s_usb_to_ws.read_seq[i] < oldest_kept_seq) {
            uint64_t dropped64 = oldest_kept_seq - s_usb_to_ws.read_seq[i];
            aggregate_dropped += dropped64 > SIZE_MAX ? SIZE_MAX : (size_t)dropped64;
            s_usb_to_ws.read_seq[i] = oldest_kept_seq;
        }
    }

    size_t pos = (size_t)(logical_start % s_usb_to_ws.capacity);
    for (size_t i = 0; i < write_len; ++i) {
        s_usb_to_ws.data[pos++] = src[i];
        if (pos == s_usb_to_ws.capacity) {
            pos = 0;
        }
    }
    s_usb_to_ws.write_seq = new_write_seq;

    portEXIT_CRITICAL(&s_usb_to_ws.mux);
    return aggregate_dropped;
}

size_t bridge_usb_to_ws_pop_client(size_t client_index, uint8_t *data, size_t max_len, TickType_t wait)
{
    const TickType_t start = xTaskGetTickCount();
    while (true) {
        size_t read = fanout_pop_now(&s_usb_to_ws, client_index, data, max_len);
        if (read > 0 || wait == 0 || !bridge_usb_to_ws_client_is_active(client_index)) {
            return read;
        }
        if (wait != portMAX_DELAY && (xTaskGetTickCount() - start) >= wait) {
            return 0;
        }
        vTaskDelay(1);
    }
}

void bridge_usb_to_ws_reset(void)
{
    portENTER_CRITICAL(&s_usb_to_ws.mux);
    s_usb_to_ws.write_seq = 0;
    for (size_t i = 0; i < BRIDGE_WS_MAX_CLIENTS; ++i) {
        s_usb_to_ws.read_seq[i] = 0;
    }
    portEXIT_CRITICAL(&s_usb_to_ws.mux);
}

size_t bridge_net_to_usb_push(const uint8_t *data, size_t len) { return ring_push(&s_net_to_usb, data, len); }
size_t bridge_net_to_usb_pop(uint8_t *data, size_t max_len, TickType_t wait) { return ring_pop(&s_net_to_usb, data, max_len, wait); }
void bridge_net_to_usb_reset(void) { ring_reset(&s_net_to_usb); }
