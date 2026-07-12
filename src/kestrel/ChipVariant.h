#ifndef KESTREL_CHIP_VARIANT_H
#define KESTREL_CHIP_VARIANT_H

namespace kestrel {

/* Which Kestrel (Wi-Fi 6 / 802.11ax, Realtek "G6 phl" generation) die a device
 * is. Unlike the 11ac generations there is no shared PHY fork: each variant has
 * its own halbb/halrf table set, firmware blob and mac_ax per-chip code in its
 * vendor tree (reference/rtl8852bu vs reference/rtl8852cu).
 *
 *   C8852B — RTL8852BU / RTL8832BU  (die-id 0x51 at R_AX_SYS_CHIPINFO)
 *   C8852C — RTL8852CU / RTL8832CU  (die-id 0x52)
 *
 * Resolved at construction in WiFiDriver::CreateRtlDevice — from the USB PID
 * (kestrel::variant_for_usb_id), NOT from the 0x00FC register byte the 11ac
 * dispatch reads: on AX silicon 0x00FC is R_AX_SYS_CHIPINFO, whose 8852A value
 * (0x50) collides with the 8822B cold-boot transient the Jaguar2 dispatch
 * accepts. The die-id is then read back in-HAL as confirmation. */
enum class ChipVariant { C8852B, C8852C };

} /* namespace kestrel */

#endif /* KESTREL_CHIP_VARIANT_H */
