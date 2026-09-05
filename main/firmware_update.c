#include "firmware_update.h"

#include <errno.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "psa/crypto.h"
#include "esp_vfs_fat.h"
#include "usb/usb_host.h"
#include "usb/msc_host.h"
#include "esp_private/msc_scsi_bot.h"
#include "usb/msc_host_vfs.h"
#include "usb_cdc_bridge.h"
#include "rp_mcu.h"
#include "sdkconfig.h"

static const char *TAG = "fw_update";

typedef struct {
    psa_hash_operation_t operation;
    bool active;
} bridge_sha256_context_t;

static void bridge_sha256_init(bridge_sha256_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    *ctx = (bridge_sha256_context_t){
        .operation = PSA_HASH_OPERATION_INIT,
        .active = false,
    };
}

static int bridge_sha256_starts(bridge_sha256_context_t *ctx)
{
    if (ctx == NULL || ctx->active) {
        return -1;
    }
    const psa_status_t status = psa_hash_setup(&ctx->operation, PSA_ALG_SHA_256);
    if (status != PSA_SUCCESS) {
        (void)psa_hash_abort(&ctx->operation);
        return -1;
    }
    ctx->active = true;
    return 0;
}

static int bridge_sha256_update(bridge_sha256_context_t *ctx, const uint8_t *data, size_t len)
{
    if (ctx == NULL || !ctx->active || (data == NULL && len != 0U)) {
        return -1;
    }
    return psa_hash_update(&ctx->operation, data, len) == PSA_SUCCESS ? 0 : -1;
}

static int bridge_sha256_finish(bridge_sha256_context_t *ctx, uint8_t hash[32])
{
    if (ctx == NULL || !ctx->active || hash == NULL) {
        return -1;
    }
    size_t hash_len = 0;
    const psa_status_t status = psa_hash_finish(&ctx->operation, hash, 32U, &hash_len);
    ctx->active = false;
    if (status != PSA_SUCCESS || hash_len != 32U) {
        (void)psa_hash_abort(&ctx->operation);
        return -1;
    }
    return 0;
}

static void bridge_sha256_free(bridge_sha256_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    if (ctx->active) {
        (void)psa_hash_abort(&ctx->operation);
    }
    bridge_sha256_init(ctx);
}


#define FW_MOUNT_PATH "/rp2"
#define FW_FILE_PATH  FW_MOUNT_PATH "/FIRMWARE.UF2"
#define FW_NUKE_FILE_PATH FW_MOUNT_PATH "/FLASH_NUKE.UF2"
#define FW_NUKE_EXPECTED_SIZE 114688U
#define FW_EVENT_MSC_READY BIT0
#define FW_EVENT_MSC_GONE  BIT1
#define FW_UF2_BLOCK_SIZE 512U
#define FW_UF2_MAGIC_START0 0x0A324655U
#define FW_UF2_MAGIC_START1 0x9E5D5157U
#define FW_UF2_MAGIC_END    0x0AB16F30U
#define FW_UF2_FLAG_FAMILY_ID 0x00002000U
#define FW_UF2_FLAG_EXTENSION_TAGS 0x00008000U
#define FW_OPENKNX_EXTENSION_TYPE 0x584E4BU
#define FW_STAGE_PARTITION_LABEL "fwstage"
#define FW_STAGE_ERASE_ALIGN 4096U
#define PICOBOOT_MAGIC 0x431FD10BU
#define PICOBOOT_CMD_REBOOT 0x02U
#define PICOBOOT_CMD_REBOOT2 0x0AU
#define PICOBOOT_REBOOT2_NORMAL 0x00000000U
#define PICOBOOT_IF_RESET 0x41U
#define PICOBOOT_REBOOT_DELAY_MS 500U
#define USB_DESC_TYPE_INTERFACE 0x04U
#define USB_DESC_TYPE_ENDPOINT 0x05U

extern const uint8_t nuke_uf2_start[] asm("_binary_nuke_universal_uf2_start");
extern const uint8_t nuke_uf2_end[] asm("_binary_nuke_universal_uf2_end");

typedef struct __attribute__((packed)) {
    uint32_t magic_start0;
    uint32_t magic_start1;
    uint32_t flags;
    uint32_t target_addr;
    uint32_t payload_size;
    uint32_t block_no;
    uint32_t num_blocks;
    uint32_t family_id;
    uint8_t data[476];
    uint32_t magic_end;
} uf2_block_t;

_Static_assert(sizeof(uf2_block_t) == FW_UF2_BLOCK_SIZE, "UF2 block must be 512 bytes");


typedef struct __attribute__((packed, aligned(4))) {
    uint32_t magic;
    uint32_t token;
    uint8_t cmd_id;
    uint8_t cmd_size;
    uint16_t unused;
    uint32_t transfer_length;
    uint32_t pc;
    uint32_t sp;
    uint32_t delay_ms;
    uint8_t pad[4];
} picoboot_reboot_packet_t;

_Static_assert(sizeof(picoboot_reboot_packet_t) == 32, "PICOBOOT command packet must be 32 bytes");

typedef struct __attribute__((packed, aligned(4))) {
    uint32_t magic;
    uint32_t token;
    uint8_t cmd_id;
    uint8_t cmd_size;
    uint16_t unused;
    uint32_t transfer_length;
    uint32_t flags;
    uint32_t delay_ms;
    uint32_t param0;
    uint32_t param1;
} picoboot_reboot2_packet_t;

_Static_assert(sizeof(picoboot_reboot2_packet_t) == 32, "PICOBOOT REBOOT2 packet must be 32 bytes");

typedef struct {
    uint8_t interface_num;
    uint8_t alternate_setting;
    uint8_t ep_out;
    uint8_t ep_in;
    uint16_t ep_out_mps;
    uint16_t ep_in_mps;
} picoboot_interface_t;

typedef struct {
    volatile bool complete;
    usb_transfer_status_t status;
    int actual_num_bytes;
} picoboot_transfer_wait_t;

typedef struct {
    volatile bool device_gone;
    usb_device_handle_t opened_device;
} picoboot_client_ctx_t;

typedef struct {
    enum {
        MSC_MSG_CONNECTED,
        MSC_MSG_DISCONNECTED,
    } type;
    union {
        uint8_t address;
        msc_host_device_handle_t handle;
    } device;
} msc_message_t;

typedef struct {
    bool valid;
    size_t size;
    uint32_t total_blocks;
    uint32_t family_id;
    rp_mcu_family_t family;
    uint8_t sha256[32];
    bool openknx_tag_valid;
    uint16_t app_id;
    uint8_t version_major;
    uint8_t version_minor;
    uint8_t version_patch;
} staged_firmware_t;

static staged_firmware_t s_staged_firmware;
static const esp_partition_t *s_stage_partition;

static QueueHandle_t s_msc_queue;
static EventGroupHandle_t s_events;
static SemaphoreHandle_t s_io_mutex;
static SemaphoreHandle_t s_operation_gate;
static msc_host_device_handle_t s_msc_device;
static msc_host_vfs_handle_t s_vfs_handle;
static volatile bool s_msc_ready;
static volatile uint8_t s_msc_address;
static volatile rp_mcu_family_t s_bootsel_family = RP_MCU_FAMILY_UNKNOWN;
static volatile bool s_hold_cdc_pause;
static firmware_update_status_t s_status;
static portMUX_TYPE s_status_mux = portMUX_INITIALIZER_UNLOCKED;

static void set_cdc_pause_hold(bool hold)
{
    s_hold_cdc_pause = hold;
    usb_cdc_bridge_set_reconnect_hold(hold);
}

static void release_cdc_pause_after_msc_failure(const char *reason)
{
    if (s_hold_cdc_pause) {
        ESP_LOGI(TAG, "CDC reconnect remains paused after %s (Flash Nuke hold active)", reason);
        return;
    }
    ESP_LOGW(TAG, "Re-enabling CDC reconnect after %s", reason);
    usb_cdc_bridge_set_reconnect_paused(false);
}

static void status_set(firmware_update_state_t state, bool busy, const char *message)
{
    portENTER_CRITICAL(&s_status_mux);
    s_status.state = state;
    s_status.busy = busy;
    if (message != NULL) {
        strlcpy(s_status.message, message, sizeof(s_status.message));
    }
    portEXIT_CRITICAL(&s_status_mux);
}

static void status_setf(firmware_update_state_t state, bool busy, const char *fmt, ...)
{
    char message[sizeof(s_status.message)];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(message, sizeof(message), fmt, ap);
    va_end(ap);
    status_set(state, busy, message);
}

static void status_progress(size_t received, size_t total, uint32_t blocks, uint32_t total_blocks)
{
    portENTER_CRITICAL(&s_status_mux);
    s_status.received_bytes = received;
    s_status.total_bytes = total;
    s_status.blocks_written = blocks;
    s_status.total_blocks = total_blocks;
    portEXIT_CRITICAL(&s_status_mux);
}

void firmware_update_get_status(firmware_update_status_t *out)
{
    if (out == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_status_mux);
    *out = s_status;
    out->msc_ready = s_msc_ready;
    out->staged_ready = s_staged_firmware.valid;
    out->bootsel_family = s_bootsel_family;
    out->staged_family = s_staged_firmware.family;
    out->staged_family_id = s_staged_firmware.family_id;
    portEXIT_CRITICAL(&s_status_mux);
}

bool firmware_update_msc_ready(void)
{
    return s_msc_ready;
}

const char *firmware_update_state_name(firmware_update_state_t state)
{
    switch (state) {
    case FW_UPDATE_IDLE: return "idle";
    case FW_UPDATE_VALIDATING: return "validating";
    case FW_UPDATE_STAGING: return "staging";
    case FW_UPDATE_ENTERING_BOOTLOADER: return "entering_bootloader";
    case FW_UPDATE_WAITING_MSC: return "waiting_msc";
    case FW_UPDATE_WRITING: return "writing";
    case FW_UPDATE_WAITING_REBOOT: return "waiting_reboot";
    case FW_UPDATE_STARTING_FIRMWARE: return "starting_firmware";
    case FW_UPDATE_NUKING: return "nuking";
    case FW_UPDATE_ESP32_OTA: return "esp32_ota";
    case FW_UPDATE_SUCCESS: return "success";
    case FW_UPDATE_ERROR: return "error";
    default: return "unknown";
    }
}

static void cleanup_msc_locked(void)
{
    s_msc_ready = false;
    s_msc_address = 0;
    s_bootsel_family = RP_MCU_FAMILY_UNKNOWN;
    xEventGroupClearBits(s_events, FW_EVENT_MSC_READY);

    if (s_vfs_handle != NULL) {
        esp_err_t err = msc_host_vfs_unregister(s_vfs_handle);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "MSC VFS unregister: %s", esp_err_to_name(err));
        }
        s_vfs_handle = NULL;
    }
    if (s_msc_device != NULL) {
        esp_err_t err = msc_host_uninstall_device(s_msc_device);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "MSC uninstall device: %s", esp_err_to_name(err));
        }
        s_msc_device = NULL;
    }
}

static void msc_worker_task(void *arg)
{
    (void)arg;
    while (true) {
        msc_message_t msg;
        if (xQueueReceive(s_msc_queue, &msg, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (msg.type == MSC_MSG_CONNECTED) {


            if (!usb_cdc_bridge_wait_released(3000)) {
                ESP_LOGW(TAG, "MSC connect: CDC handle not fully released yet");
            }

            if (xSemaphoreTake(s_io_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
                ESP_LOGW(TAG, "MSC connect: I/O mutex timeout");
                release_cdc_pause_after_msc_failure("MSC I/O mutex timeout");
                continue;
            }

            cleanup_msc_locked();

            esp_err_t err = msc_host_install_device(msg.device.address, &s_msc_device);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "msc_host_install_device: %s", esp_err_to_name(err));
                s_msc_device = NULL;
                xSemaphoreGive(s_io_mutex);
                release_cdc_pause_after_msc_failure("MSC device install failure");
                continue;
            }

            msc_host_device_info_t info = {0};
            err = msc_host_get_device_info(s_msc_device, &info);
            rp_mcu_family_t bootsel_family = RP_MCU_FAMILY_UNKNOWN;
            if (err == ESP_OK) {
                bootsel_family = rp_mcu_family_from_bootsel_usb(info.idVendor, info.idProduct);
                ESP_LOGI(TAG, "MSC device VID=%04x PID=%04x sector=%" PRIu32 " count=%" PRIu32,
                         info.idVendor, info.idProduct, info.sector_size, info.sector_count);
            }
            if (err != ESP_OK || bootsel_family == RP_MCU_FAMILY_UNKNOWN) {
                ESP_LOGW(TAG, "Ignoring non-RP BOOTSEL MSC device");
                msc_host_uninstall_device(s_msc_device);
                s_msc_device = NULL;
                s_vfs_handle = NULL;
                xSemaphoreGive(s_io_mutex);
                release_cdc_pause_after_msc_failure("non-RP MSC device");
                continue;
            }

            const esp_vfs_fat_mount_config_t mount_cfg = {
                .format_if_mount_failed = false,
                .max_files = 2,
                .allocation_unit_size = 4096,
            };
            err = msc_host_vfs_register(s_msc_device, FW_MOUNT_PATH, &mount_cfg, &s_vfs_handle);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "msc_host_vfs_register: %s", esp_err_to_name(err));
                msc_host_uninstall_device(s_msc_device);
                s_msc_device = NULL;
                s_vfs_handle = NULL;
                xSemaphoreGive(s_io_mutex);
                release_cdc_pause_after_msc_failure("MSC VFS mount failure");
                continue;
            }

            s_msc_address = msg.device.address;
            s_bootsel_family = bootsel_family;
            s_msc_ready = true;
            xEventGroupClearBits(s_events, FW_EVENT_MSC_GONE);
            xEventGroupSetBits(s_events, FW_EVENT_MSC_READY);
            ESP_LOGI(TAG, "%s BOOTSEL mass storage mounted at %s",
                     rp_mcu_family_name(bootsel_family), FW_MOUNT_PATH);
            xSemaphoreGive(s_io_mutex);
        } else if (msg.type == MSC_MSG_DISCONNECTED) {
            if (s_msc_device == NULL || msg.device.handle != s_msc_device) {
                continue;
            }

            s_msc_ready = false;
            s_msc_address = 0;
            s_bootsel_family = RP_MCU_FAMILY_UNKNOWN;
            xEventGroupClearBits(s_events, FW_EVENT_MSC_READY);
            xEventGroupSetBits(s_events, FW_EVENT_MSC_GONE);

            if (xSemaphoreTake(s_io_mutex, portMAX_DELAY) == pdTRUE) {
                cleanup_msc_locked();
                xSemaphoreGive(s_io_mutex);
            }
            ESP_LOGI(TAG, "RP-series BOOTSEL mass storage disconnected");
            if (!s_hold_cdc_pause) {
                usb_cdc_bridge_set_reconnect_paused(false);
            } else {
                ESP_LOGI(TAG, "CDC reconnect remains paused for Flash Nuke operation");
            }
        }
    }
}

static void msc_event_cb(const msc_host_event_t *event, void *arg)
{
    (void)arg;
    if (event == NULL || s_msc_queue == NULL) {
        return;
    }

    msc_message_t msg = {0};
    if (event->event == MSC_DEVICE_CONNECTED) {


        usb_cdc_bridge_set_reconnect_paused(true);
        msg.type = MSC_MSG_CONNECTED;
        msg.device.address = event->device.address;
    } else if (event->event == MSC_DEVICE_DISCONNECTED) {
        msg.type = MSC_MSG_DISCONNECTED;
        msg.device.handle = event->device.handle;
    } else {
        return;
    }

    if (xQueueSend(s_msc_queue, &msg, 0) != pdTRUE) {
        ESP_LOGW(TAG, "MSC event queue full (%s event lost)",
                 msg.type == MSC_MSG_CONNECTED ? "connect" : "disconnect");


        release_cdc_pause_after_msc_failure("MSC event queue overflow");
    }
}

void firmware_update_start(void)
{
    memset(&s_status, 0, sizeof(s_status));
    s_status.state = FW_UPDATE_IDLE;
    strlcpy(s_status.message, "Bereit", sizeof(s_status.message));
    memset(&s_staged_firmware, 0, sizeof(s_staged_firmware));

    s_stage_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, FW_STAGE_PARTITION_LABEL);
    if (s_stage_partition == NULL) {
        ESP_LOGE(TAG, "Firmware staging partition '%s' not found", FW_STAGE_PARTITION_LABEL);
        strlcpy(s_status.message, "Firmware-Staging-Partition fehlt", sizeof(s_status.message));
        s_status.state = FW_UPDATE_ERROR;
    } else {
        ESP_LOGI(TAG, "Firmware staging partition: offset=0x%" PRIx32 " size=%" PRIu32,
                 s_stage_partition->address, s_stage_partition->size);
    }

    const size_t embedded_nuke_size = (size_t)(nuke_uf2_end - nuke_uf2_start);
    if (embedded_nuke_size != FW_NUKE_EXPECTED_SIZE) {
        ESP_LOGE(TAG, "Embedded Flash Nuke size mismatch: %u != %u",
                 (unsigned)embedded_nuke_size, (unsigned)FW_NUKE_EXPECTED_SIZE);
    } else {
        ESP_LOGI(TAG, "Embedded Raspberry Pi Flash Nuke ready (%u bytes)", (unsigned)embedded_nuke_size);
    }

    s_msc_queue = xQueueCreate(6, sizeof(msc_message_t));
    s_events = xEventGroupCreate();
    s_io_mutex = xSemaphoreCreateMutex();
    s_operation_gate = xSemaphoreCreateBinary();
    configASSERT(s_msc_queue != NULL);
    configASSERT(s_events != NULL);
    configASSERT(s_io_mutex != NULL);
    configASSERT(s_operation_gate != NULL);
    xSemaphoreGive(s_operation_gate);

    BaseType_t ok = xTaskCreate(msc_worker_task, "msc_worker", 6144, NULL, 6, NULL);
    configASSERT(ok == pdPASS);

    const msc_host_driver_config_t cfg = {
        .create_backround_task = true,
        .task_priority = 6,
        .stack_size = 4096,
        .core_id = tskNO_AFFINITY,
        .callback = msc_event_cb,
        .callback_arg = NULL,
    };
    esp_err_t err = msc_host_install(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MSC host install failed: %s", esp_err_to_name(err));
        status_set(FW_UPDATE_ERROR, false, "MSC-Host konnte nicht gestartet werden");
    } else {
        ESP_LOGI(TAG, "MSC host ready for RP-series UF2 updates");
    }
}

static bool uf2_validate_block(const uf2_block_t *b, uint32_t expected_total, char *reason, size_t reason_len)
{
    if (b->magic_start0 != FW_UF2_MAGIC_START0 || b->magic_start1 != FW_UF2_MAGIC_START1 || b->magic_end != FW_UF2_MAGIC_END) {
        snprintf(reason, reason_len, "Keine gueltige UF2-Datei (Magic)");
        return false;
    }
    if ((b->flags & FW_UF2_FLAG_FAMILY_ID) == 0 || !rp_mcu_uf2_family_is_supported(b->family_id)) {
        snprintf(reason, reason_len, "UF2 Family-ID wird von der RP-series Bridge nicht unterstuetzt");
        return false;
    }
    if (b->payload_size != 256U) {
        snprintf(reason, reason_len, "Unerwartete UF2 Payload-Groesse: %" PRIu32, b->payload_size);
        return false;
    }


    if (b->num_blocks == 0U || b->num_blocks > expected_total || b->block_no >= b->num_blocks) {
        snprintf(reason, reason_len, "UF2 Blockzaehler unplausibel");
        return false;
    }
    return true;
}

static bool uf2_extract_openknx_tag(const uint8_t *raw_block, staged_firmware_t *meta)
{
    if (raw_block == NULL || meta == NULL) {
        return false;
    }
    const uf2_block_t *b = (const uf2_block_t *)raw_block;
    if ((b->flags & FW_UF2_FLAG_EXTENSION_TAGS) == 0 || b->payload_size > 476U) {
        return false;
    }

    size_t p = 32U + b->payload_size;
    const size_t end = 508U;
    while (p + 4U <= end) {
        uint8_t size = raw_block[p];
        uint8_t t0 = raw_block[p + 1U];
        uint8_t t1 = raw_block[p + 2U];
        uint8_t t2 = raw_block[p + 3U];
        if (size == 0U || t0 == 0U) {
            break;
        }
        if (size < 4U || p + size > end) {
            break;
        }
        uint32_t type = (uint32_t)t0 | ((uint32_t)t1 << 8) | ((uint32_t)t2 << 16);
        if (type == FW_OPENKNX_EXTENSION_TYPE && size >= 8U) {
            const uint8_t *d = raw_block + p + 4U;
            meta->openknx_tag_valid = true;
            meta->app_id = ((uint16_t)d[0] << 8) | d[1];
            meta->version_major = d[2] >> 4;
            meta->version_minor = d[2] & 0x0fU;
            meta->version_patch = d[3];
            return true;
        }
        p += size + ((4U - (size % 4U)) % 4U);
    }
    return false;
}

static void sha256_to_hex(const uint8_t hash[32], char out[65])
{
    static const char hex[] = "0123456789abcdef";
    for (size_t i = 0; i < 32U; ++i) {
        out[i * 2U] = hex[hash[i] >> 4];
        out[i * 2U + 1U] = hex[hash[i] & 0x0fU];
    }
    out[64] = '\0';
}

static bool get_expected_sha256_header(httpd_req_t *req, char out[65])
{
    out[0] = '\0';
    size_t len = httpd_req_get_hdr_value_len(req, "X-UF2-SHA256");
    if (len != 64U) {
        return false;
    }
    if (httpd_req_get_hdr_value_str(req, "X-UF2-SHA256", out, 65U) != ESP_OK) {
        out[0] = '\0';
        return false;
    }
    for (size_t i = 0; i < 64U; ++i) {
        if (out[i] >= 'A' && out[i] <= 'F') {
            out[i] = (char)(out[i] - 'A' + 'a');
        }
        if (!((out[i] >= '0' && out[i] <= '9') || (out[i] >= 'a' && out[i] <= 'f'))) {
            out[0] = '\0';
            return false;
        }
    }
    return true;
}

static size_t align_up_size(size_t value, size_t alignment)
{
    return (value + alignment - 1U) & ~(alignment - 1U);
}

static void staged_version_string(const staged_firmware_t *meta, char *out, size_t out_len)
{
    if (out == NULL || out_len == 0) {
        return;
    }
    if (meta != NULL && meta->openknx_tag_valid) {
        snprintf(out, out_len, "%u.%u.%u", meta->version_major, meta->version_minor, meta->version_patch);
    } else {
        strlcpy(out, "?", out_len);
    }
}

static esp_err_t sha256_staged_partition(size_t size, uint8_t hash[32], uint8_t *scratch, size_t scratch_len)
{
    if (s_stage_partition == NULL || hash == NULL || scratch == NULL || scratch_len == 0U ||
        size == 0U || size > s_stage_partition->size) {
        return ESP_ERR_INVALID_ARG;
    }

    bridge_sha256_context_t ctx;
    bridge_sha256_init(&ctx);
    if (bridge_sha256_starts(&ctx) != 0) {
        bridge_sha256_free(&ctx);
        return ESP_FAIL;
    }

    esp_err_t result = ESP_OK;
    for (size_t off = 0; off < size;) {
        size_t chunk = size - off;
        if (chunk > scratch_len) {
            chunk = scratch_len;
        }
        esp_err_t err = esp_partition_read(s_stage_partition, off, scratch, chunk);
        if (err != ESP_OK) {
            result = err;
            break;
        }
        if (bridge_sha256_update(&ctx, scratch, chunk) != 0) {
            result = ESP_FAIL;
            break;
        }
        off += chunk;
    }

    if (result == ESP_OK && bridge_sha256_finish(&ctx, hash) != 0) {
        result = ESP_FAIL;
    }
    bridge_sha256_free(&ctx);
    return result;
}

static esp_err_t recv_exact(httpd_req_t *req, uint8_t *buf, size_t len)
{
    size_t off = 0;
    int timeout_count = 0;
    while (off < len) {
        int r = httpd_req_recv(req, (char *)buf + off, len - off);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            if (++timeout_count < 5) {
                continue;
            }
            return ESP_ERR_TIMEOUT;
        }
        if (r <= 0) {
            return ESP_FAIL;
        }
        timeout_count = 0;
        off += (size_t)r;
    }
    return ESP_OK;
}

static bool wait_for_msc(uint32_t timeout_ms)
{
    if (s_msc_ready) {
        return true;
    }
    EventBits_t bits = xEventGroupWaitBits(
        s_events,
        FW_EVENT_MSC_READY,
        pdFALSE,
        pdTRUE,
        pdMS_TO_TICKS(timeout_ms)
    );
    return (bits & FW_EVENT_MSC_READY) != 0 || s_msc_ready;
}

static esp_err_t enter_bootloader_and_wait(void)
{
    if (s_msc_ready) {
        return ESP_OK;
    }
    if (!usb_cdc_bridge_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }


    usb_cdc_bridge_set_reconnect_paused(true);
    xEventGroupClearBits(s_events, FW_EVENT_MSC_READY | FW_EVENT_MSC_GONE);


    status_set(FW_UPDATE_ENTERING_BOOTLOADER, true, "OpenKNX Upload-Handshake: 0x07, dann 1200 Baud/DTR aus");
    ESP_LOGI(TAG, "Trying OpenKNX RP-MCU 1200-baud touch (0x07 + DTR low)");
    esp_err_t touch_err = usb_cdc_bridge_openknx_1200_touch(CONFIG_BRIDGE_CDC_BAUD, 2000);
    if (touch_err != ESP_OK) {
        ESP_LOGW(TAG, "OpenKNX 1200-baud touch returned %s; waiting for RP BOOTSEL anyway",
                 esp_err_to_name(touch_err));
    }

    status_set(FW_UPDATE_WAITING_MSC, true, "Warte auf RP BOOTSEL (OpenKNX 1200-Baud-Touch)");
    if (wait_for_msc(CONFIG_BRIDGE_FW_1200_TOUCH_TIMEOUT_MS)) {
        ESP_LOGI(TAG, "RP MCU entered BOOTSEL via 1200-baud touch");
        return ESP_OK;
    }


    if (!usb_cdc_bridge_is_connected()) {
        status_set(FW_UPDATE_WAITING_MSC, true, "USB CDC getrennt; warte weiter auf RP BOOTSEL");
        ESP_LOGI(TAG, "CDC disappeared after 1200-baud touch; extending RP BOOTSEL wait");
        if (wait_for_msc(CONFIG_BRIDGE_FW_BOOT_TIMEOUT_MS)) {
            return ESP_OK;
        }
        usb_cdc_bridge_set_reconnect_paused(false);
        return ESP_ERR_TIMEOUT;
    }


    esp_err_t restore_err = usb_cdc_bridge_restore_normal_state(CONFIG_BRIDGE_CDC_BAUD, 2000);
    if (restore_err != ESP_OK) {
        ESP_LOGW(TAG, "Could not restore normal CDC state before fallback: %s", esp_err_to_name(restore_err));
    }

    status_set(FW_UPDATE_ENTERING_BOOTLOADER, true, "1200-Baud/DTR-Touch ohne Reaktion; Fallback: OpenKNX bootloader");
    ESP_LOGW(TAG, "OpenKNX 1200-baud/DTR touch did not trigger BOOTSEL; using console fallback");

    static const uint8_t command[] = "bootloader\r\n";
    esp_err_t err = usb_cdc_bridge_send_direct(command, sizeof(command) - 1, 2000);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Bootloader command fallback failed: %s", esp_err_to_name(err));
        usb_cdc_bridge_set_reconnect_paused(false);
        return err;
    }

    status_set(FW_UPDATE_WAITING_MSC, true, "Warte auf RP BOOTSEL (OpenKNX-Fallback)");
    if (wait_for_msc(CONFIG_BRIDGE_FW_BOOT_TIMEOUT_MS)) {
        ESP_LOGI(TAG, "RP MCU entered BOOTSEL via OpenKNX console fallback");
        return ESP_OK;
    }
    usb_cdc_bridge_set_reconnect_paused(false);
    return ESP_ERR_TIMEOUT;
}

static void send_http_error(httpd_req_t *req, const char *status, const char *message)
{
    status_set(FW_UPDATE_ERROR, false, message);
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_sendstr(req, message);
}

static void bootloader_request_task(void *arg)
{
    (void)arg;
    esp_err_t err = enter_bootloader_and_wait();

    if (err == ESP_OK) {
        status_set(FW_UPDATE_IDLE, false, "RP-MCU-Bootloader aktiv (RP BOOTSEL)");
        ESP_LOGI(TAG, "Manual RP-MCU BOOTSEL request completed");
    } else if (err == ESP_ERR_INVALID_STATE) {
        status_set(FW_UPDATE_ERROR, false, "Kein RP-MCU-CDC-Geraet verbunden");
        ESP_LOGW(TAG, "Manual BOOTSEL request failed: no RP-MCU CDC device");
    } else {
        status_set(FW_UPDATE_ERROR, false, "RP-MCU-Bootloader wurde nicht als RP BOOTSEL erkannt");
        ESP_LOGW(TAG, "Manual BOOTSEL request failed: %s", esp_err_to_name(err));
    }

    xSemaphoreGive(s_operation_gate);
    vTaskDelete(NULL);
}

esp_err_t firmware_update_bootloader_http_handler(httpd_req_t *req)
{
    if (xSemaphoreTake(s_operation_gate, 0) != pdTRUE) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "Ein Firmware-Update oder Bootloader-Wechsel laeuft bereits");
    }

    if (s_msc_ready) {
        status_set(FW_UPDATE_IDLE, false, "RP-MCU-Bootloader ist bereits aktiv (RP BOOTSEL)");
        xSemaphoreGive(s_operation_gate);
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        return httpd_resp_sendstr(req, "OK: RP-MCU-Bootloader ist bereits aktiv");
    }

    if (!usb_cdc_bridge_is_connected()) {
        status_set(FW_UPDATE_ERROR, false, "Kein RP-MCU-CDC-Geraet verbunden");
        xSemaphoreGive(s_operation_gate);
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        return httpd_resp_sendstr(req, "Kein RP-MCU-CDC-Geraet verbunden");
    }

    status_set(FW_UPDATE_ENTERING_BOOTLOADER, true, "Bootloader-Wechsel wird gestartet");
    BaseType_t ok = xTaskCreate(bootloader_request_task, "rp_boot", 4096, NULL, 6, NULL);
    if (ok != pdPASS) {
        status_set(FW_UPDATE_ERROR, false, "Bootloader-Task konnte nicht gestartet werden");
        xSemaphoreGive(s_operation_gate);
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "Bootloader-Task konnte nicht gestartet werden");
    }

    httpd_resp_set_status(req, "202 Accepted");
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_sendstr(req, "OK: Bootloader-Wechsel gestartet");
}


static void picoboot_client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    picoboot_client_ctx_t *ctx = (picoboot_client_ctx_t *)arg;
    if (ctx == NULL || event_msg == NULL) {
        return;
    }
    if (event_msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE &&
        event_msg->dev_gone.dev_hdl == ctx->opened_device) {
        ctx->device_gone = true;
    }
}

static void picoboot_transfer_done_cb(usb_transfer_t *transfer)
{
    if (transfer == NULL || transfer->context == NULL) {
        return;
    }
    picoboot_transfer_wait_t *wait = (picoboot_transfer_wait_t *)transfer->context;
    wait->status = transfer->status;
    wait->actual_num_bytes = transfer->actual_num_bytes;
    wait->complete = true;
}

static esp_err_t picoboot_submit_and_wait(usb_host_client_handle_t client,
                                          usb_transfer_t *transfer,
                                          bool control,
                                          uint32_t timeout_ms)
{
    if (client == NULL || transfer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    picoboot_transfer_wait_t wait = {
        .complete = false,
        .status = USB_TRANSFER_STATUS_ERROR,
        .actual_num_bytes = 0,
    };
    transfer->callback = picoboot_transfer_done_cb;
    transfer->context = &wait;

    esp_err_t err = control ? usb_host_transfer_submit_control(client, transfer)
                            : usb_host_transfer_submit(transfer);
    if (err != ESP_OK) {
        return err;
    }

    uint32_t elapsed = 0;
    bool timeout_reported = false;
    while (!wait.complete) {


        err = usb_host_client_handle_events(client, pdMS_TO_TICKS(50));
        if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {


            ESP_LOGW(TAG, "PICOBOOT client event handling returned %s while transfer is active",
                     esp_err_to_name(err));
        }

        if (elapsed < UINT32_MAX - 50U) {
            elapsed += 50U;
        }
        if (!timeout_reported && timeout_ms != 0U && elapsed >= timeout_ms) {
            timeout_reported = true;
            ESP_LOGW(TAG, "PICOBOOT transfer exceeded %" PRIu32
                     " ms; waiting for safe USB completion before cleanup", timeout_ms);
        }
    }
    switch (wait.status) {
    case USB_TRANSFER_STATUS_COMPLETED:
        return ESP_OK;
    case USB_TRANSFER_STATUS_NO_DEVICE:
        return ESP_ERR_NOT_FOUND;
    case USB_TRANSFER_STATUS_TIMED_OUT:
        return ESP_ERR_TIMEOUT;
    default:
        return ESP_FAIL;
    }
}

static bool picoboot_find_interface(const usb_config_desc_t *config, picoboot_interface_t *out)
{
    if (config == NULL || out == NULL || config->wTotalLength < config->bLength) {
        return false;
    }

    const uint8_t *raw = (const uint8_t *)config;
    const size_t total = config->wTotalLength;

    for (size_t off = config->bLength; off + 2U <= total;) {
        const usb_standard_desc_t *std = (const usb_standard_desc_t *)(raw + off);
        if (std->bLength < 2U || off + std->bLength > total) {
            break;
        }

        if (std->bDescriptorType == USB_DESC_TYPE_INTERFACE && std->bLength >= sizeof(usb_intf_desc_t)) {
            const usb_intf_desc_t *intf = (const usb_intf_desc_t *)std;
            if (intf->bInterfaceClass == USB_CLASS_VENDOR_SPEC && intf->bNumEndpoints == 2U) {
                picoboot_interface_t candidate = {
                    .interface_num = intf->bInterfaceNumber,
                    .alternate_setting = intf->bAlternateSetting,
                };

                for (size_t ep_off = off + std->bLength; ep_off + 2U <= total;) {
                    const usb_standard_desc_t *ep_std = (const usb_standard_desc_t *)(raw + ep_off);
                    if (ep_std->bLength < 2U || ep_off + ep_std->bLength > total) {
                        break;
                    }
                    if (ep_std->bDescriptorType == USB_DESC_TYPE_INTERFACE) {
                        break;
                    }
                    if (ep_std->bDescriptorType == USB_DESC_TYPE_ENDPOINT && ep_std->bLength >= sizeof(usb_ep_desc_t)) {
                        const usb_ep_desc_t *ep = (const usb_ep_desc_t *)ep_std;
                        if ((ep->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) == USB_BM_ATTRIBUTES_XFER_BULK) {
                            if (ep->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK) {
                                candidate.ep_in = ep->bEndpointAddress;
                                candidate.ep_in_mps = USB_EP_DESC_GET_MPS(ep);
                            } else {
                                candidate.ep_out = ep->bEndpointAddress;
                                candidate.ep_out_mps = USB_EP_DESC_GET_MPS(ep);
                            }
                        }
                    }
                    ep_off += ep_std->bLength;
                }

                if (candidate.ep_out != 0 && candidate.ep_in != 0) {
                    *out = candidate;
                    return true;
                }
            }
        }
        off += std->bLength;
    }
    return false;
}

static esp_err_t picoboot_interface_reset(usb_host_client_handle_t client,
                                          usb_device_handle_t device,
                                          uint8_t interface_num)
{
    usb_transfer_t *transfer = NULL;
    esp_err_t err = usb_host_transfer_alloc(sizeof(usb_setup_packet_t), 0, &transfer);
    if (err != ESP_OK) {
        return err;
    }

    usb_setup_packet_t *setup = (usb_setup_packet_t *)transfer->data_buffer;
    setup->bmRequestType = 0x41;
    setup->bRequest = PICOBOOT_IF_RESET;
    setup->wValue = 0;
    setup->wIndex = interface_num;
    setup->wLength = 0;

    transfer->num_bytes = sizeof(usb_setup_packet_t);
    transfer->device_handle = device;
    transfer->bEndpointAddress = 0;
    transfer->timeout_ms = 1000;
    err = picoboot_submit_and_wait(client, transfer, true, 1500);
    usb_host_transfer_free(transfer);
    return err;
}

static esp_err_t picoboot_bulk_out(usb_host_client_handle_t client,
                                   usb_device_handle_t device,
                                   uint8_t endpoint,
                                   const void *data,
                                   size_t len)
{
    usb_transfer_t *transfer = NULL;
    esp_err_t err = usb_host_transfer_alloc(len, 0, &transfer);
    if (err != ESP_OK) {
        return err;
    }
    memcpy(transfer->data_buffer, data, len);
    transfer->num_bytes = (int)len;
    transfer->device_handle = device;
    transfer->bEndpointAddress = endpoint;
    transfer->timeout_ms = 2000;
    err = picoboot_submit_and_wait(client, transfer, false, 2500);
    if (err == ESP_OK && transfer->actual_num_bytes != (int)len) {
        err = ESP_ERR_INVALID_SIZE;
    }
    usb_host_transfer_free(transfer);
    return err;
}

static esp_err_t picoboot_wait_ack(usb_host_client_handle_t client,
                                   usb_device_handle_t device,
                                   uint8_t endpoint,
                                   uint16_t max_packet_size)
{
    const size_t alloc_size = max_packet_size ? max_packet_size : 64U;
    usb_transfer_t *transfer = NULL;
    esp_err_t err = usb_host_transfer_alloc(alloc_size, 0, &transfer);
    if (err != ESP_OK) {
        return err;
    }


    transfer->num_bytes = (int)alloc_size;
    transfer->device_handle = device;
    transfer->bEndpointAddress = endpoint;
    transfer->timeout_ms = 2000;
    err = picoboot_submit_and_wait(client, transfer, false, 2500);
    if (err == ESP_OK && transfer->actual_num_bytes != 0) {
        ESP_LOGW(TAG, "PICOBOOT reboot ACK contained %d bytes instead of ZLP", transfer->actual_num_bytes);
    }
    usb_host_transfer_free(transfer);
    return err;
}

static esp_err_t picoboot_reboot_application(void)
{
    const uint8_t device_address = s_msc_address;
    if (!s_msc_ready || device_address == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t result = ESP_FAIL;
    usb_host_client_handle_t client = NULL;
    usb_device_handle_t device = NULL;
    bool interface_claimed = false;
    picoboot_interface_t pb = {0};
    picoboot_client_ctx_t client_ctx = {0};

    const usb_host_client_config_t client_cfg = {
        .is_synchronous = false,
        .max_num_event_msg = 4,
        .async = {
            .client_event_callback = picoboot_client_event_cb,
            .callback_arg = &client_ctx,
        },
    };

    result = usb_host_client_register(&client_cfg, &client);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "PICOBOOT client register failed: %s", esp_err_to_name(result));
        goto out;
    }

    result = usb_host_device_open(client, device_address, &device);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "PICOBOOT device open failed: %s", esp_err_to_name(result));
        goto out;
    }
    client_ctx.opened_device = device;

    const usb_device_desc_t *device_desc = NULL;
    result = usb_host_get_device_descriptor(device, &device_desc);
    rp_mcu_family_t family = RP_MCU_FAMILY_UNKNOWN;
    if (result == ESP_OK && device_desc != NULL) {
        family = rp_mcu_family_from_bootsel_usb(device_desc->idVendor, device_desc->idProduct);
    }
    if (result != ESP_OK || device_desc == NULL || family == RP_MCU_FAMILY_UNKNOWN) {
        ESP_LOGE(TAG, "PICOBOOT device is not a supported RP-series BOOTSEL device");
        result = ESP_ERR_NOT_SUPPORTED;
        goto out;
    }

    const usb_config_desc_t *config_desc = NULL;
    result = usb_host_get_active_config_descriptor(device, &config_desc);
    if (result != ESP_OK || !picoboot_find_interface(config_desc, &pb)) {
        ESP_LOGE(TAG, "PICOBOOT vendor interface not found");
        result = ESP_ERR_NOT_FOUND;
        goto out;
    }

    ESP_LOGI(TAG, "PICOBOOT interface=%u OUT=0x%02x IN=0x%02x",
             pb.interface_num, pb.ep_out, pb.ep_in);

    result = usb_host_interface_claim(client, device, pb.interface_num, pb.alternate_setting);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "PICOBOOT interface claim failed: %s", esp_err_to_name(result));
        goto out;
    }
    interface_claimed = true;

    esp_err_t reset_err = picoboot_interface_reset(client, device, pb.interface_num);
    if (reset_err != ESP_OK) {


        ESP_LOGW(TAG, "PICOBOOT interface reset returned %s; continuing", esp_err_to_name(reset_err));
    }

    static uint32_t token = 1;
    if (family == RP_MCU_FAMILY_RP2350) {
        picoboot_reboot2_packet_t packet = {
            .magic = PICOBOOT_MAGIC,
            .token = token++,
            .cmd_id = PICOBOOT_CMD_REBOOT2,
            .cmd_size = 16,
            .unused = 0,
            .transfer_length = 0,
            .flags = PICOBOOT_REBOOT2_NORMAL,
            .delay_ms = PICOBOOT_REBOOT_DELAY_MS,
            .param0 = 0,
            .param1 = 0,
        };
        ESP_LOGI(TAG, "Sending RP2350 PICOBOOT REBOOT2 to normal boot path (delay=%u ms)",
                 (unsigned)PICOBOOT_REBOOT_DELAY_MS);
        result = picoboot_bulk_out(client, device, pb.ep_out, &packet, sizeof(packet));
    } else {
        picoboot_reboot_packet_t packet = {
            .magic = PICOBOOT_MAGIC,
            .token = token++,
            .cmd_id = PICOBOOT_CMD_REBOOT,
            .cmd_size = 12,
            .unused = 0,
            .transfer_length = 0,
            .pc = 0,
            .sp = 0,
            .delay_ms = PICOBOOT_REBOOT_DELAY_MS,
            .pad = {0},
        };
        ESP_LOGI(TAG, "Sending RP2040 PICOBOOT REBOOT to normal flash boot path (delay=%u ms)",
                 (unsigned)PICOBOOT_REBOOT_DELAY_MS);
        result = picoboot_bulk_out(client, device, pb.ep_out, &packet, sizeof(packet));
    }
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "PICOBOOT reboot command failed: %s", esp_err_to_name(result));
        goto out;
    }


    result = picoboot_wait_ack(client, device, pb.ep_in, pb.ep_in_mps);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "PICOBOOT REBOOT ACK failed: %s", esp_err_to_name(result));
        goto out;
    }
    ESP_LOGI(TAG, "PICOBOOT REBOOT accepted");

out:
    if (interface_claimed && client != NULL && device != NULL) {
        esp_err_t release_err = usb_host_interface_release(client, device, pb.interface_num);
        if (release_err != ESP_OK && result == ESP_OK) {
            ESP_LOGW(TAG, "PICOBOOT interface release: %s", esp_err_to_name(release_err));
        }
    }
    if (device != NULL && client != NULL) {
        esp_err_t close_err = usb_host_device_close(client, device);
        if (close_err != ESP_OK && result == ESP_OK) {
            ESP_LOGW(TAG, "PICOBOOT device close: %s", esp_err_to_name(close_err));
        }
    }
    if (client != NULL) {
        esp_err_t dereg_err = usb_host_client_deregister(client);
        if (dereg_err != ESP_OK && result == ESP_OK) {
            ESP_LOGW(TAG, "PICOBOOT client deregister: %s", esp_err_to_name(dereg_err));
        }
    }
    return result;
}

static void start_firmware_task(void *arg)
{
    (void)arg;
    xEventGroupClearBits(s_events, FW_EVENT_MSC_GONE);
    status_set(FW_UPDATE_STARTING_FIRMWARE, true, "Sende PICOBOOT-Reboot in die Firmware");

    if (xSemaphoreTake(s_io_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        status_set(FW_UPDATE_ERROR, false, "RP BOOTSEL ist beschaeftigt");
        xSemaphoreGive(s_operation_gate);
        vTaskDelete(NULL);
        return;
    }

    esp_err_t err = picoboot_reboot_application();
    xSemaphoreGive(s_io_mutex);

    if (err != ESP_OK) {
        status_set(FW_UPDATE_ERROR, false, "PICOBOOT-Reboot konnte nicht gesendet werden");
        ESP_LOGW(TAG, "Manual firmware start failed: %s", esp_err_to_name(err));
        xSemaphoreGive(s_operation_gate);
        vTaskDelete(NULL);
        return;
    }

    status_set(FW_UPDATE_WAITING_REBOOT, true, "Reboot akzeptiert; warte auf RP-MCU-Neustart");
    EventBits_t gone = xEventGroupWaitBits(
        s_events, FW_EVENT_MSC_GONE, pdFALSE, pdTRUE, pdMS_TO_TICKS(5000));

    if ((gone & FW_EVENT_MSC_GONE) == 0 && s_msc_ready) {
        status_set(FW_UPDATE_ERROR, false, "PICOBOOT-Reboot akzeptiert, aber RP BOOTSEL blieb aktiv");
        ESP_LOGW(TAG, "PICOBOOT reboot ACK received, but MSC did not disconnect");
        xSemaphoreGive(s_operation_gate);
        vTaskDelete(NULL);
        return;
    }


    for (unsigned i = 0; i < 100; ++i) {
        if (usb_cdc_bridge_is_connected()) {
            status_set(FW_UPDATE_SUCCESS, false, " ");
            ESP_LOGI(TAG, "RP MCU application started and CDC reconnected");
            xSemaphoreGive(s_operation_gate);
            vTaskDelete(NULL);
            return;
        }
        if (s_msc_ready) {
            status_set(FW_UPDATE_ERROR, false, "RP MCU kehrte in RP BOOTSEL zurueck - keine startbare Firmware erkannt");
            ESP_LOGW(TAG, "RP MCU returned to BOOTSEL instead of application CDC");
            xSemaphoreGive(s_operation_gate);
            vTaskDelete(NULL);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }


    status_set(FW_UPDATE_SUCCESS, false, "RP MCU neu gestartet - noch kein CDC-Geraet erkannt");
    ESP_LOGW(TAG, "RP MCU rebooted from BOOTSEL, but no CDC application appeared within 10 s");
    xSemaphoreGive(s_operation_gate);
    vTaskDelete(NULL);
}

esp_err_t firmware_update_start_firmware_http_handler(httpd_req_t *req)
{
    if (xSemaphoreTake(s_operation_gate, 0) != pdTRUE) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "Ein Firmware-/Bootloader-Vorgang laeuft bereits");
    }

    if (!s_msc_ready || s_msc_address == 0) {
        status_set(FW_UPDATE_ERROR, false, "RP MCU ist nicht im RP BOOTSEL Bootloader");
        xSemaphoreGive(s_operation_gate);
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        return httpd_resp_sendstr(req, "RP MCU ist nicht im RP BOOTSEL Bootloader");
    }

    status_set(FW_UPDATE_STARTING_FIRMWARE, true, "Firmware-Start wird vorbereitet");
    BaseType_t ok = xTaskCreate(start_firmware_task, "rp2_start", 6144, NULL, 6, NULL);
    if (ok != pdPASS) {
        status_set(FW_UPDATE_ERROR, false, "Firmware-Start-Task konnte nicht gestartet werden");
        xSemaphoreGive(s_operation_gate);
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "Firmware-Start-Task konnte nicht gestartet werden");
    }

    httpd_resp_set_status(req, "202 Accepted");
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_sendstr(req, "OK: Firmware-Start ueber PICOBOOT gestartet");
}


static esp_err_t prepare_raw_msc_io(void)
{
    if (!s_msc_ready || s_msc_device == NULL) {
        return ESP_ERR_INVALID_STATE;
    }


    if (s_vfs_handle != NULL) {
        esp_err_t err = msc_host_vfs_unregister(s_vfs_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Could not unmount RP BOOTSEL before raw MSC write: %s", esp_err_to_name(err));
            return err;
        }
        s_vfs_handle = NULL;
        ESP_LOGI(TAG, "RP BOOTSEL FAT VFS unmounted; switching to raw MSC sector writes");
    }
    return ESP_OK;
}

static esp_err_t raw_msc_write_uf2_block(uint32_t block_index,
                                         uint32_t total_blocks,
                                         const uint8_t block[FW_UF2_BLOCK_SIZE])
{
    if (!s_msc_ready || s_msc_device == NULL) {
        return ESP_ERR_INVALID_STATE;
    }


    const size_t lba = (size_t)block_index + 1U;
    esp_err_t last = ESP_FAIL;

    for (unsigned attempt = 0; attempt < 3U; ++attempt) {
        last = scsi_cmd_write10(s_msc_device, block, (uint32_t)lba, 1U, FW_UF2_BLOCK_SIZE);
        if (last == ESP_OK) {
            return ESP_OK;
        }

        if (!s_msc_ready) {
            break;
        }

        ESP_LOGW(TAG,
                 "Raw MSC UF2 write failed at block %" PRIu32 "/%" PRIu32
                 " LBA=%u (attempt %u/3): %s",
                 block_index + 1U, total_blocks, (unsigned)lba,
                 attempt + 1U, esp_err_to_name(last));


        vTaskDelay(pdMS_TO_TICKS(120));
    }


    if (block_index + 1U == total_blocks) {
        EventBits_t gone = xEventGroupWaitBits(
            s_events, FW_EVENT_MSC_GONE, pdFALSE, pdTRUE, pdMS_TO_TICKS(1500));
        if ((gone & FW_EVENT_MSC_GONE) || !s_msc_ready) {
            ESP_LOGI(TAG, "RP BOOTSEL disconnected after final raw UF2 block");
            return ESP_OK;
        }
    }
    return last;
}

static esp_err_t write_embedded_flash_nuke(void)
{
    esp_err_t err = prepare_raw_msc_io();
    if (err != ESP_OK) {
        return err;
    }

    const size_t total = (size_t)(nuke_uf2_end - nuke_uf2_start);
    if (total != FW_NUKE_EXPECTED_SIZE || (total % FW_UF2_BLOCK_SIZE) != 0U) {
        ESP_LOGE(TAG, "Embedded Flash Nuke is invalid: %u bytes", (unsigned)total);
        return ESP_ERR_INVALID_SIZE;
    }

    const uint32_t total_blocks = (uint32_t)(total / FW_UF2_BLOCK_SIZE);
    ESP_LOGW(TAG, "Writing embedded Raspberry Pi Flash Nuke v2.3.0 via raw MSC (%u blocks)",
             (unsigned)total_blocks);

    for (uint32_t i = 0; i < total_blocks; ++i) {
        err = raw_msc_write_uf2_block(i, total_blocks,
                                      nuke_uf2_start + ((size_t)i * FW_UF2_BLOCK_SIZE));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Flash Nuke raw MSC write failed at block %" PRIu32 ": %s",
                     i + 1U, esp_err_to_name(err));
            return err;
        }
        status_progress((size_t)(i + 1U) * FW_UF2_BLOCK_SIZE, total, i + 1U, total_blocks);
    }

    ESP_LOGI(TAG, "Embedded Flash Nuke raw UF2 transfer complete; waiting for erase/re-enumeration");
    return ESP_OK;
}

static void flash_nuke_task(void *arg)
{
    (void)arg;
    esp_err_t err = enter_bootloader_and_wait();
    if (err != ESP_OK) {
        status_set(FW_UPDATE_ERROR, false,
                   err == ESP_ERR_INVALID_STATE ? "Kein RP MCU verbunden" : "RP-MCU-Bootloader nicht erreichbar");
        xSemaphoreGive(s_operation_gate);
        vTaskDelete(NULL);
        return;
    }


    set_cdc_pause_hold(true);
    xEventGroupClearBits(s_events, FW_EVENT_MSC_GONE);
    status_progress(0, FW_NUKE_EXPECTED_SIZE, 0, 0);
    status_set(FW_UPDATE_NUKING, true, "Eingebetteter Flash Nuke wird ausgefuehrt");

    if (xSemaphoreTake(s_io_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        status_set(FW_UPDATE_ERROR, false, "RP BOOTSEL ist beschaeftigt");
        set_cdc_pause_hold(false);
        xSemaphoreGive(s_operation_gate);
        vTaskDelete(NULL);
        return;
    }

    err = write_embedded_flash_nuke();
    xSemaphoreGive(s_io_mutex);

    if (err != ESP_OK) {
        status_set(FW_UPDATE_ERROR, false, "Flash Nuke konnte nicht uebertragen werden");
        set_cdc_pause_hold(false);
        if (!s_msc_ready) {
            usb_cdc_bridge_set_reconnect_paused(false);
        }
        xSemaphoreGive(s_operation_gate);
        vTaskDelete(NULL);
        return;
    }

    status_set(FW_UPDATE_NUKING, true, "Flash wird geloescht; warte auf RP BOOTSEL");


    if (s_msc_ready) {
        xEventGroupWaitBits(s_events, FW_EVENT_MSC_GONE, pdFALSE, pdTRUE, pdMS_TO_TICKS(10000));
    }
    xEventGroupClearBits(s_events, FW_EVENT_MSC_READY);
    bool back = wait_for_msc(20000);

    set_cdc_pause_hold(false);
    if (back) {
        status_progress(FW_NUKE_EXPECTED_SIZE, FW_NUKE_EXPECTED_SIZE, 0, 0);
        status_set(FW_UPDATE_SUCCESS, false, "Flash Nuke abgeschlossen - RP BOOTSEL ist bereit");
        ESP_LOGW(TAG, "Flash Nuke completed; RP MCU returned to BOOTSEL");

    } else {
        status_set(FW_UPDATE_ERROR, false, "Flash Nuke uebertragen, aber RP BOOTSEL kam nicht zurueck");
        usb_cdc_bridge_set_reconnect_paused(false);
    }

    xSemaphoreGive(s_operation_gate);
    vTaskDelete(NULL);
}

esp_err_t firmware_update_nuke_http_handler(httpd_req_t *req)
{


    char confirm_token[8] = {0};
    if (httpd_req_get_hdr_value_str(req, "X-Confirm-Nuke", confirm_token, sizeof(confirm_token)) != ESP_OK ||
        strcmp(confirm_token, "NUKE") != 0) {
        httpd_resp_set_status(req, "428 Precondition Required");
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        return httpd_resp_sendstr(req, "Flash Nuke abgelehnt: Sicherheitsbestaetigung fehlt");
    }

    if (xSemaphoreTake(s_operation_gate, 0) != pdTRUE) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "Ein Firmware-/Bootloader-Vorgang laeuft bereits");
    }

    if (!s_msc_ready && !usb_cdc_bridge_is_connected()) {
        status_set(FW_UPDATE_ERROR, false, "Kein RP MCU verbunden");
        xSemaphoreGive(s_operation_gate);
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "Kein RP MCU verbunden");
    }

    status_set(FW_UPDATE_NUKING, true, "Flash Nuke wird gestartet");
    BaseType_t ok = xTaskCreate(flash_nuke_task, "rp_nuke", 7168, NULL, 6, NULL);
    if (ok != pdPASS) {
        status_set(FW_UPDATE_ERROR, false, "Flash-Nuke-Task konnte nicht gestartet werden");
        xSemaphoreGive(s_operation_gate);
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "Flash-Nuke-Task konnte nicht gestartet werden");
    }

    httpd_resp_set_status(req, "202 Accepted");
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_sendstr(req, "OK: Flash Nuke gestartet");
}

static esp_err_t write_staged_firmware_to_msc(void)
{
    if (!s_staged_firmware.valid || s_stage_partition == NULL) {
        status_set(FW_UPDATE_ERROR, true, "Lokaler UF2-Puffer ist nicht gueltig");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = prepare_raw_msc_io();
    if (err != ESP_OK) {
        status_setf(FW_UPDATE_ERROR, true,
                    "RP BOOTSEL konnte nicht fuer direkten MSC-Transfer vorbereitet werden (%s)",
                    esp_err_to_name(err));
        return err;
    }

    uint8_t *block = malloc(FW_UF2_BLOCK_SIZE);
    if (block == NULL) {
        status_set(FW_UPDATE_ERROR, true, "Nicht genug RAM fuer den RP BOOTSEL Schreibpuffer");
        return ESP_ERR_NO_MEM;
    }

    status_set(FW_UPDATE_WRITING, true, "Gepufferte Firmware wird direkt auf den RP MCU geschrieben");
    ESP_LOGI(TAG, "Starting raw MSC UF2 transfer: %" PRIu32 " blocks, FAT filesystem bypassed",
             s_staged_firmware.total_blocks);

    for (uint32_t i = 0; i < s_staged_firmware.total_blocks; ++i) {
        err = esp_partition_read(s_stage_partition, (size_t)i * FW_UF2_BLOCK_SIZE,
                                 block, FW_UF2_BLOCK_SIZE);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Staging read failed at block %" PRIu32 ": %s", i, esp_err_to_name(err));
            status_setf(FW_UPDATE_ERROR, true,
                        "ESP32-Staging: Lesefehler bei UF2-Block %" PRIu32 "/%" PRIu32 " (%s)",
                        i + 1U, s_staged_firmware.total_blocks, esp_err_to_name(err));
            free(block);
            return err;
        }

        err = raw_msc_write_uf2_block(i, s_staged_firmware.total_blocks, block);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Raw MSC UF2 write failed at block %" PRIu32 "/%" PRIu32 ": %s",
                     i + 1U, s_staged_firmware.total_blocks, esp_err_to_name(err));
            status_setf(FW_UPDATE_ERROR, true,
                        "RP BOOTSEL Direkttransfer fehlgeschlagen: Block %" PRIu32 "/%" PRIu32 " (%s)",
                        i + 1U, s_staged_firmware.total_blocks, esp_err_to_name(err));
            free(block);
            return err;
        }

        status_progress(s_staged_firmware.size, s_staged_firmware.size,
                        i + 1U, s_staged_firmware.total_blocks);
    }

    free(block);
    ESP_LOGI(TAG, "Raw MSC UF2 transfer completed");
    return ESP_OK;
}

static void staged_flash_task(void *arg)
{
    (void)arg;
    esp_err_t err = enter_bootloader_and_wait();
    if (err != ESP_OK) {
        status_set(FW_UPDATE_ERROR, false,
                   err == ESP_ERR_INVALID_STATE ? "Kein RP MCU verbunden" : "RP-MCU-Bootloader nicht erreichbar");
        xSemaphoreGive(s_operation_gate);
        vTaskDelete(NULL);
        return;
    }

    const rp_mcu_family_t connected_family = s_bootsel_family;
    if (connected_family == RP_MCU_FAMILY_UNKNOWN || connected_family != s_staged_firmware.family) {
        status_setf(FW_UPDATE_ERROR, false, "UF2 fuer %s passt nicht zum verbundenen %s",
                    rp_mcu_family_name(s_staged_firmware.family),
                    rp_mcu_family_name(connected_family));
        ESP_LOGE(TAG, "UF2/device family mismatch: UF2=%s (0x%08" PRIx32 ") device=%s",
                 rp_mcu_family_name(s_staged_firmware.family), s_staged_firmware.family_id,
                 rp_mcu_family_name(connected_family));
        xSemaphoreGive(s_operation_gate);
        vTaskDelete(NULL);
        return;
    }

    xEventGroupClearBits(s_events, FW_EVENT_MSC_GONE);
    if (xSemaphoreTake(s_io_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        status_set(FW_UPDATE_ERROR, false, "RP BOOTSEL ist beschaeftigt");
        xSemaphoreGive(s_operation_gate);
        vTaskDelete(NULL);
        return;
    }

    err = write_staged_firmware_to_msc();
    xSemaphoreGive(s_io_mutex);
    if (err != ESP_OK) {


        portENTER_CRITICAL(&s_status_mux);
        s_status.busy = false;
        portEXIT_CRITICAL(&s_status_mux);
        xSemaphoreGive(s_operation_gate);
        vTaskDelete(NULL);
        return;
    }

    status_set(FW_UPDATE_WAITING_REBOOT, true, "Firmware geschrieben; warte auf OpenKNX-Neustart");
    if (s_msc_ready) {
        (void)xEventGroupWaitBits(s_events, FW_EVENT_MSC_GONE, pdFALSE, pdTRUE, pdMS_TO_TICKS(5000));
    }


    bool cdc_seen = false;
    for (unsigned i = 0; i < 150U; ++i) {
        if (usb_cdc_bridge_is_connected()) {
            cdc_seen = true;
            break;
        }
        if (s_msc_ready && i > 20U) {
            status_set(FW_UPDATE_ERROR, false, "RP MCU kehrte in RP BOOTSEL zurueck - Firmware nicht gestartet");
            xSemaphoreGive(s_operation_gate);
            vTaskDelete(NULL);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (!cdc_seen) {
        status_set(FW_UPDATE_ERROR, false, "Firmware geschrieben, aber OpenKNX-CDC kam nicht zurueck");
        xSemaphoreGive(s_operation_gate);
        vTaskDelete(NULL);
        return;
    }

    status_progress(s_staged_firmware.size, s_staged_firmware.size,
                    s_staged_firmware.total_blocks, s_staged_firmware.total_blocks);
    status_set(FW_UPDATE_SUCCESS, false, "Firmware erfolgreich gestartet");
    ESP_LOGI(TAG, "RP MCU application CDC returned after firmware update");

    xSemaphoreGive(s_operation_gate);
    vTaskDelete(NULL);
}

esp_err_t firmware_update_http_handler(httpd_req_t *req)
{
    if (xSemaphoreTake(s_operation_gate, 0) != pdTRUE) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "Ein Firmware-Update laeuft bereits");
    }

    esp_err_t result = ESP_FAIL;
    uint8_t *block_buf = NULL;
    bool sha_started = false;
    bridge_sha256_context_t sha;
    bridge_sha256_init(&sha);
    memset(&s_staged_firmware, 0, sizeof(s_staged_firmware));

    if (s_stage_partition == NULL) {
        send_http_error(req, "500 Internal Server Error", "Firmware-Staging-Partition fehlt");
        goto out;
    }

    const size_t total = (size_t)req->content_len;
    if (total < FW_UF2_BLOCK_SIZE || total > CONFIG_BRIDGE_FW_MAX_SIZE ||
        total > s_stage_partition->size || (total % FW_UF2_BLOCK_SIZE) != 0U) {
        send_http_error(req, "400 Bad Request", "UF2-Dateigroesse ist fuer den lokalen Puffer ungueltig");
        goto out;
    }

    block_buf = malloc(FW_UF2_BLOCK_SIZE);
    if (block_buf == NULL) {
        send_http_error(req, "500 Internal Server Error", "Nicht genug RAM fuer UF2-Puffer");
        goto out;
    }

    const uint32_t total_blocks = (uint32_t)(total / FW_UF2_BLOCK_SIZE);
    status_progress(0, total, 0, total_blocks);
    status_set(FW_UPDATE_STAGING, true, "UF2 wird vollstaendig im ESP32 gepuffert");

    size_t erase_len = align_up_size(total, FW_STAGE_ERASE_ALIGN);
    esp_err_t err = esp_partition_erase_range(s_stage_partition, 0, erase_len);
    if (err != ESP_OK) {
        send_http_error(req, "500 Internal Server Error", "Firmware-Puffer konnte nicht geloescht werden");
        goto out;
    }

    if (bridge_sha256_starts(&sha) != 0) {
        send_http_error(req, "500 Internal Server Error", "SHA-256 konnte nicht initialisiert werden");
        goto out;
    }
    sha_started = true;

    char reason[96];
    for (uint32_t i = 0; i < total_blocks; ++i) {
        err = recv_exact(req, block_buf, FW_UF2_BLOCK_SIZE);
        if (err != ESP_OK) {
            send_http_error(req, "408 Request Timeout", "UF2-Upload wurde unterbrochen");
            goto out;
        }

        const uf2_block_t *b = (const uf2_block_t *)block_buf;
        if (!uf2_validate_block(b, total_blocks, reason, sizeof(reason))) {
            send_http_error(req, "400 Bad Request", reason);
            goto out;
        }
        if (rp_mcu_uf2_family_is_bootable(b->family_id)) {
            const rp_mcu_family_t block_family = rp_mcu_family_from_uf2(b->family_id);
            if (s_staged_firmware.family == RP_MCU_FAMILY_UNKNOWN) {
                s_staged_firmware.family_id = b->family_id;
                s_staged_firmware.family = block_family;
            } else if (b->family_id != s_staged_firmware.family_id) {
                send_http_error(req, "400 Bad Request",
                                "UF2 enthaelt mehrere ausfuehrbare Zielfamilien");
                goto out;
            }
        }

        staged_firmware_t found = s_staged_firmware;
        if (uf2_extract_openknx_tag(block_buf, &found)) {
            if (s_staged_firmware.openknx_tag_valid &&
                (found.app_id != s_staged_firmware.app_id ||
                 found.version_major != s_staged_firmware.version_major ||
                 found.version_minor != s_staged_firmware.version_minor ||
                 found.version_patch != s_staged_firmware.version_patch)) {
                send_http_error(req, "400 Bad Request", "UF2 enthaelt widerspruechliche OpenKNX-Versions-Tags");
                goto out;
            }
            s_staged_firmware.openknx_tag_valid = true;
            s_staged_firmware.app_id = found.app_id;
            s_staged_firmware.version_major = found.version_major;
            s_staged_firmware.version_minor = found.version_minor;
            s_staged_firmware.version_patch = found.version_patch;
        }

        err = esp_partition_write(s_stage_partition, (size_t)i * FW_UF2_BLOCK_SIZE,
                                  block_buf, FW_UF2_BLOCK_SIZE);
        if (err != ESP_OK) {
            send_http_error(req, "500 Internal Server Error", "UF2 konnte nicht im ESP32-Flash gepuffert werden");
            goto out;
        }
        if (bridge_sha256_update(&sha, block_buf, FW_UF2_BLOCK_SIZE) != 0) {
            send_http_error(req, "500 Internal Server Error", "SHA-256 Berechnung fehlgeschlagen");
            goto out;
        }
        status_progress((size_t)(i + 1U) * FW_UF2_BLOCK_SIZE, total, 0, total_blocks);
    }

    if (s_staged_firmware.family == RP_MCU_FAMILY_UNKNOWN) {
        send_http_error(req, "400 Bad Request",
                        "UF2 enthaelt keine ausfuehrbare RP-MCU-Zielfamilie");
        goto out;
    }

    if (bridge_sha256_finish(&sha, s_staged_firmware.sha256) != 0) {
        send_http_error(req, "500 Internal Server Error", "SHA-256 Abschluss fehlgeschlagen");
        goto out;
    }
    sha_started = false;

    char actual_sha[65], expected_sha[65];
    sha256_to_hex(s_staged_firmware.sha256, actual_sha);
    if (get_expected_sha256_header(req, expected_sha) && strcmp(actual_sha, expected_sha) != 0) {
        ESP_LOGE(TAG, "Browser/ESP32 SHA-256 mismatch: browser=%s esp32=%s", expected_sha, actual_sha);
        send_http_error(req, "400 Bad Request", "SHA-256 stimmt nach lokalem Puffern nicht ueberein");
        goto out;
    }


    status_set(FW_UPDATE_VALIDATING, true, "Lokaler Firmware-Puffer wird per SHA-256 rueckgelesen");
    uint8_t staged_hash[32];
    err = sha256_staged_partition(total, staged_hash, block_buf, FW_UF2_BLOCK_SIZE);
    if (err != ESP_OK || memcmp(staged_hash, s_staged_firmware.sha256, sizeof(staged_hash)) != 0) {
        char readback_sha[65] = {0};
        if (err == ESP_OK) {
            sha256_to_hex(staged_hash, readback_sha);
            ESP_LOGE(TAG, "Staging readback SHA-256 mismatch: upload=%s flash=%s", actual_sha, readback_sha);
        } else {
            ESP_LOGE(TAG, "Staging readback failed: %s", esp_err_to_name(err));
        }
        send_http_error(req, "500 Internal Server Error", "Lokaler Firmware-Puffer konnte nicht verifiziert werden");
        goto out;
    }

    s_staged_firmware.size = total;
    s_staged_firmware.total_blocks = total_blocks;
    s_staged_firmware.valid = true;
    char staged_ver[24];
    staged_version_string(&s_staged_firmware, staged_ver, sizeof(staged_ver));
    if (s_staged_firmware.openknx_tag_valid) {
        ESP_LOGI(TAG, "UF2 staged and verified: target=%s family=0x%08" PRIx32 " SHA256=%s OpenKNX=0x%04x %s",
                 rp_mcu_family_name(s_staged_firmware.family), s_staged_firmware.family_id,
                 actual_sha, s_staged_firmware.app_id, staged_ver);
    } else {
        ESP_LOGI(TAG, "UF2 staged and verified: target=%s family=0x%08" PRIx32 " SHA256=%s (no OpenKNX version tag)",
                 rp_mcu_family_name(s_staged_firmware.family), s_staged_firmware.family_id, actual_sha);
    }

    status_set(FW_UPDATE_VALIDATING, true, "UF2 lokal gespeichert und per SHA-256 geprueft");
    BaseType_t ok = xTaskCreate(staged_flash_task, "rp_flash", 8192, NULL, 6, NULL);
    if (ok != pdPASS) {
        s_staged_firmware.valid = false;
        send_http_error(req, "500 Internal Server Error", "Firmware-Flash-Task konnte nicht gestartet werden");
        goto out;
    }

    httpd_resp_set_status(req, "202 Accepted");
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    result = httpd_resp_sendstr(req, "OK: Firmware lokal gespeichert und geprueft; Flash-Vorgang gestartet");


    bridge_sha256_free(&sha);
    free(block_buf);
    return result;

out:
    if (sha_started) {

    }
    bridge_sha256_free(&sha);
    free(block_buf);
    s_staged_firmware.valid = false;
    xSemaphoreGive(s_operation_gate);
    return result;
}


static void staged_retry_task(void *arg)
{
    (void)arg;
    staged_flash_task(NULL);
}

esp_err_t firmware_update_retry_http_handler(httpd_req_t *req)
{
    if (xSemaphoreTake(s_operation_gate, 0) != pdTRUE) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "Ein Firmware-/Bootloader-Vorgang laeuft bereits");
    }

    if (!s_staged_firmware.valid || s_stage_partition == NULL) {
        status_set(FW_UPDATE_ERROR, false, "Keine gepufferte UF2 fuer einen erneuten Schreibversuch vorhanden");
        xSemaphoreGive(s_operation_gate);
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "Keine gepufferte UF2 vorhanden");
    }

    if (!s_msc_ready && !usb_cdc_bridge_is_connected()) {
        status_set(FW_UPDATE_ERROR, false, "Kein RP MCU verbunden");
        xSemaphoreGive(s_operation_gate);
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "Kein RP MCU verbunden");
    }

    status_set(FW_UPDATE_VALIDATING, true, "Erneuter Schreibversuch aus dem ESP32-Firmwarepuffer");
    BaseType_t ok = xTaskCreate(staged_retry_task, "rp2_retry", 8192, NULL, 6, NULL);
    if (ok != pdPASS) {
        status_set(FW_UPDATE_ERROR, false, "Retry-Task konnte nicht gestartet werden");
        xSemaphoreGive(s_operation_gate);
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "Retry-Task konnte nicht gestartet werden");
    }

    httpd_resp_set_status(req, "202 Accepted");
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_sendstr(req, "OK: Gepufferte UF2 wird erneut auf RP BOOTSEL geschrieben");
}

static void esp32_restart_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(1200));
    esp_restart();
}

static bool sha256_header_matches(httpd_req_t *req, const uint8_t hash[32], char actual_hex[65])
{
    static const char HEX[] = "0123456789abcdef";
    for (size_t i = 0; i < 32; ++i) {
        actual_hex[i * 2] = HEX[hash[i] >> 4];
        actual_hex[i * 2 + 1] = HEX[hash[i] & 0x0fU];
    }
    actual_hex[64] = '\0';

    size_t header_len = httpd_req_get_hdr_value_len(req, "X-Firmware-SHA256");
    if (header_len == 0) {
        return true;
    }
    if (header_len != 64U) {
        return false;
    }
    char expected[65] = {0};
    if (httpd_req_get_hdr_value_str(req, "X-Firmware-SHA256", expected, sizeof(expected)) != ESP_OK) {
        return false;
    }
    return strcmp(expected, actual_hex) == 0;
}

esp_err_t firmware_update_esp32_http_handler(httpd_req_t *req)
{
    if (xSemaphoreTake(s_operation_gate, 0) != pdTRUE) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "Ein Firmware-/Bootloader-Vorgang laeuft bereits");
    }

    esp_err_t result = ESP_FAIL;
    esp_ota_handle_t ota_handle = 0;
    bool ota_started = false;
    uint8_t *buffer = NULL;
    bridge_sha256_context_t sha;
    bool sha_started = false;
    bridge_sha256_init(&sha);

    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (target == NULL) {
        send_http_error(req, "500 Internal Server Error", "Keine freie ESP32-OTA-Partition vorhanden");
        goto out;
    }

    const size_t total = (size_t)req->content_len;
    if (total < 288U || total > target->size) {
        send_http_error(req, "400 Bad Request", "ESP32-Firmwaregroesse ist ungueltig oder zu gross fuer die OTA-Partition");
        goto out;
    }

    buffer = malloc(4096U);
    if (buffer == NULL) {
        send_http_error(req, "500 Internal Server Error", "Nicht genug RAM fuer ESP32-OTA-Puffer");
        goto out;
    }

    if (bridge_sha256_starts(&sha) != 0) {
        send_http_error(req, "500 Internal Server Error", "SHA-256 konnte nicht gestartet werden");
        goto out;
    }
    sha_started = true;

    ESP_LOGI(TAG, "ESP32 OTA start: target=%s offset=0x%" PRIx32 " size=%u image=%u",
             target->label, target->address, (unsigned)target->size, (unsigned)total);
    status_set(FW_UPDATE_ESP32_OTA, true, "ESP32 Bridge-Firmware wird uebertragen");

    esp_err_t err = esp_ota_begin(target, total, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        send_http_error(req, "500 Internal Server Error", "ESP32 OTA konnte nicht gestartet werden");
        goto out;
    }
    ota_started = true;

    size_t received = 0;
    unsigned timeout_count = 0;
    while (received < total) {
        size_t wanted = total - received;
        if (wanted > 4096U) {
            wanted = 4096U;
        }
        int n = httpd_req_recv(req, (char *)buffer, wanted);
        if (n == HTTPD_SOCK_ERR_TIMEOUT) {
            if (++timeout_count <= 8U) {
                continue;
            }
            send_http_error(req, "408 Request Timeout", "Timeout beim ESP32-Firmware-Upload");
            goto out;
        }
        if (n <= 0) {
            send_http_error(req, "400 Bad Request", "ESP32-Firmware-Upload wurde unterbrochen");
            goto out;
        }
        timeout_count = 0;

        if (bridge_sha256_update(&sha, buffer, (size_t)n) != 0) {
            send_http_error(req, "500 Internal Server Error", "SHA-256 Berechnung fehlgeschlagen");
            goto out;
        }
        err = esp_ota_write(ota_handle, buffer, (size_t)n);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed after %u bytes: %s", (unsigned)received, esp_err_to_name(err));
            send_http_error(req, "400 Bad Request", "Ungueltige oder nicht schreibbare ESP32-Firmware");
            goto out;
        }
        received += (size_t)n;
    }

    uint8_t hash[32];
    if (bridge_sha256_finish(&sha, hash) != 0) {
        send_http_error(req, "500 Internal Server Error", "SHA-256 Abschluss fehlgeschlagen");
        goto out;
    }
    sha_started = false;
    char actual_sha[65];
    if (!sha256_header_matches(req, hash, actual_sha)) {
        ESP_LOGE(TAG, "ESP32 OTA SHA-256 mismatch");
        send_http_error(req, "400 Bad Request", "SHA-256 der empfangenen ESP32-Firmware stimmt nicht");
        goto out;
    }

    err = esp_ota_end(ota_handle);
    ota_started = false;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end validation failed: %s", esp_err_to_name(err));
        send_http_error(req, "400 Bad Request", "ESP32-Firmwareabbild ist ungueltig");
        goto out;
    }

    err = esp_ota_set_boot_partition(target);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        send_http_error(req, "500 Internal Server Error", "Neue ESP32-Firmware konnte nicht als Boot-Partition gesetzt werden");
        goto out;
    }

    ESP_LOGI(TAG, "ESP32 OTA verified: %u bytes SHA256=%s; rebooting to %s",
             (unsigned)total, actual_sha, target->label);
    status_set(FW_UPDATE_ESP32_OTA, true, "ESP32 Update erfolgreich; Neustart");
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    result = httpd_resp_sendstr(req, "OK: ESP32-Firmware geschrieben und geprueft. Neustart...");


    xSemaphoreGive(s_operation_gate);
    BaseType_t ok = xTaskCreate(esp32_restart_task, "esp32_restart", 2048, NULL, 8, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Could not create ESP32 restart task; restarting immediately");
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
    }
    free(buffer);
    bridge_sha256_free(&sha);
    return result;

out:
    if (ota_started) {
        (void)esp_ota_abort(ota_handle);
    }
    if (sha_started) {

    }
    bridge_sha256_free(&sha);
    free(buffer);
    status_set(FW_UPDATE_ERROR, false, "ESP32 Firmware-Update fehlgeschlagen");
    xSemaphoreGive(s_operation_gate);
    return result == ESP_FAIL ? ESP_OK : result;
}
