#include "rp_mcu.h"

rp_mcu_family_t rp_mcu_family_from_bootsel_usb(uint16_t vid, uint16_t pid)
{
    if (vid != RP_USB_VID) {
        return RP_MCU_FAMILY_UNKNOWN;
    }
    if (pid == RP2040_BOOTSEL_PID) {
        return RP_MCU_FAMILY_RP2040;
    }
    if (pid == RP2350_BOOTSEL_PID) {
        return RP_MCU_FAMILY_RP2350;
    }
    return RP_MCU_FAMILY_UNKNOWN;
}

rp_mcu_family_t rp_mcu_family_from_uf2(uint32_t family_id)
{
    switch (family_id) {
    case RP_UF2_FAMILY_RP2040:
        return RP_MCU_FAMILY_RP2040;
    case RP_UF2_FAMILY_RP2350_ARM_S:
    case RP_UF2_FAMILY_RP2350_RISCV:
        return RP_MCU_FAMILY_RP2350;
    default:
        return RP_MCU_FAMILY_UNKNOWN;
    }
}

bool rp_mcu_is_bootsel_usb(uint16_t vid, uint16_t pid)
{
    return rp_mcu_family_from_bootsel_usb(vid, pid) != RP_MCU_FAMILY_UNKNOWN;
}

bool rp_mcu_uf2_family_is_bootable(uint32_t family_id)
{
    return rp_mcu_family_from_uf2(family_id) != RP_MCU_FAMILY_UNKNOWN;
}

bool rp_mcu_uf2_family_is_auxiliary(uint32_t family_id)
{
    switch (family_id) {
    case RP_UF2_FAMILY_ABSOLUTE:
    case RP_UF2_FAMILY_DATA:
    case RP_UF2_FAMILY_RP2350_ARM_NS:
        return true;
    default:
        return false;
    }
}

bool rp_mcu_uf2_family_is_supported(uint32_t family_id)
{
    return rp_mcu_uf2_family_is_bootable(family_id) || rp_mcu_uf2_family_is_auxiliary(family_id);
}

const char *rp_mcu_family_name(rp_mcu_family_t family)
{
    switch (family) {
    case RP_MCU_FAMILY_RP2040:
        return "RP2040";
    case RP_MCU_FAMILY_RP2350:
        return "RP2350/RP2354";
    default:
        return "RP MCU";
    }
}

const char *rp_mcu_uf2_family_name(uint32_t family_id)
{
    switch (family_id) {
    case RP_UF2_FAMILY_RP2040:
        return "RP2040";
    case RP_UF2_FAMILY_ABSOLUTE:
        return "Absolute";
    case RP_UF2_FAMILY_DATA:
        return "Data";
    case RP_UF2_FAMILY_RP2350_ARM_S:
        return "RP2350/RP2354 Arm Secure";
    case RP_UF2_FAMILY_RP2350_RISCV:
        return "RP2350/RP2354 RISC-V";
    case RP_UF2_FAMILY_RP2350_ARM_NS:
        return "RP2350/RP2354 Arm Non-secure";
    default:
        return "Unbekannt";
    }
}
