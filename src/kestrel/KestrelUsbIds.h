#ifndef KESTREL_USB_IDS_H
#define KESTREL_USB_IDS_H

#include <cstdint>
#include <optional>

#include "ChipVariant.h"

namespace kestrel {

/* USB VID:PID → Kestrel variant. Extracted from the vendor drivers' usb_intf.c
 * id tables (reference/rtl8852bu and reference/rtl8852cu os_dep/linux/
 * usb_intf.c) — the same tables the kernel modules match on. The Kestrel PIDs
 * are disjoint from every 11ac PID devourer dispatches, which is what makes
 * the PID-first factory gate safe (see ChipVariant.h for why the 11ac
 * SYS_CFG2-byte dispatch cannot be reused on AX silicon).
 *
 * 8852A-family ids (0bda:8832/885a/885c, 2357:0141 TP-Link TX20UH) are
 * deliberately absent: the only vendor tree for that die is the 2021 v1.15
 * drop, out of scope for the Kestrel HAL (see docs / issue #236 plan). */
struct KestrelUsbId {
  uint16_t vid;
  uint16_t pid;
  ChipVariant variant;
};

inline constexpr KestrelUsbId kKestrelUsbIds[] = {
    /* RTL8852B / RTL8832B — Realtek default ids */
    {0x0bda, 0xb832, ChipVariant::C8852B},
    {0x0bda, 0xb83a, ChipVariant::C8852B},
    {0x0bda, 0xb852, ChipVariant::C8852B},
    {0x0bda, 0xb85a, ChipVariant::C8852B},
    {0x0bda, 0xa85b, ChipVariant::C8852B},
    /* RTL8852B — seller ids */
    {0x0586, 0x3428, ChipVariant::C8852B}, /* ZyXEL NWD7605 */
    {0x0b05, 0x1a62, ChipVariant::C8852B}, /* ASUS USB-AX55 Nano */
    {0x0db0, 0x6931, ChipVariant::C8852B}, /* MSI AX1800 */
    {0x0db0, 0xf0c8, ChipVariant::C8852B}, /* MSI AX1800 Nano */
    {0x2001, 0x3327, ChipVariant::C8852B}, /* D-Link AX18U */
    {0x3574, 0x6121, ChipVariant::C8852B}, /* COMFAST CF-943AX */
    {0x3574, 0x6221, ChipVariant::C8852B}, /* COMFAST 8832BU */
    {0x3574, 0x6241, ChipVariant::C8852B}, /* COMFAST 8852BU */
    {0x35bc, 0x0100, ChipVariant::C8852B}, /* TP-Link Archer TX20U */
    {0x35bc, 0x0108, ChipVariant::C8852B}, /* TP-Link Archer TX20U Nano */
    {0x7392, 0x6822, ChipVariant::C8852B}, /* EDIMAX EW-7822UMX */
    /* RTL8852C / RTL8832C — Realtek default ids */
    {0x0bda, 0xc85a, ChipVariant::C8852C},
    {0x0bda, 0xc832, ChipVariant::C8852C},
    {0x0bda, 0xc85d, ChipVariant::C8852C},
    /* RTL8852C — seller ids */
    {0x0db0, 0x991d, ChipVariant::C8852C}, /* MSI AXE5400 */
    {0x2c4e, 0x0127, ChipVariant::C8852C}, /* Mercusys MA86XH */
    {0x3574, 0x6251, ChipVariant::C8852C}, /* Sihai Lianzong */
    {0x35b2, 0x0502, ChipVariant::C8852C}, /* TP-Link Archer TXE70UH */
    {0x35bc, 0x0101, ChipVariant::C8852C}, /* TP-Link Archer TX50UH V1 */
    {0x35bc, 0x0102, ChipVariant::C8852C}, /* TP-Link Archer TXE70UH(EU) V1 */
};

inline std::optional<ChipVariant> variant_for_usb_id(uint16_t vid,
                                                     uint16_t pid) {
  for (const auto &id : kKestrelUsbIds)
    if (id.vid == vid && id.pid == pid)
      return id.variant;
  return std::nullopt;
}

} /* namespace kestrel */

#endif /* KESTREL_USB_IDS_H */
