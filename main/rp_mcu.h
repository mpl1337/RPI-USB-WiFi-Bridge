#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    RP_MCU_FAMILY_UNKNOWN = 0,
    RP_MCU_FAMILY_RP2040,
    RP_MCU_FAMILY_RP2350,
} rp_mcu_family_t;

#define RP_USB_VID 0x2E8AU
#define RP2040_BOOTSEL_PID 0x0003U
#define RP2350_BOOTSEL_PID 0x000FU

#define RP_UF2_FAMILY_RP2040        0xE48BFF56U
#define RP_UF2_FAMILY_ABSOLUTE      0xE48BFF57U
#define RP_UF2_FAMILY_DATA          0xE48BFF58U
#define RP_UF2_FAMILY_RP2350_ARM_S  0xE48BFF59U
#define RP_UF2_FAMILY_RP2350_RISCV  0xE48BFF5AU
#define RP_UF2_FAMILY_RP2350_ARM_NS 0xE48BFF5BU

rp_mcu_family_t rp_mcu_family_from_bootsel_usb(uint16_t vid, uint16_t pid);
rp_mcu_family_t rp_mcu_family_from_uf2(uint32_t family_id);
bool rp_mcu_is_bootsel_usb(uint16_t vid, uint16_t pid);
bool rp_mcu_uf2_family_is_bootable(uint32_t family_id);
bool rp_mcu_uf2_family_is_auxiliary(uint32_t family_id);
bool rp_mcu_uf2_family_is_supported(uint32_t family_id);
const char *rp_mcu_family_name(rp_mcu_family_t family);
const char *rp_mcu_uf2_family_name(uint32_t family_id);
