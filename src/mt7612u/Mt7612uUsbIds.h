#ifndef MT7612U_USB_IDS_H
#define MT7612U_USB_IDS_H

#include <cstdint>

namespace mt7612u {

/* USB identities of the MediaTek MT7662-MAC adapters (MT7612U / MT7662U).
 *
 * The complete mt76x2u_device_table, transcribed from the pinned reference tree
 * this port was ported from — reference/mt76 @ be5ce79,
 * mt76x2/usb.c:12-28 — so it is re-verifiable from a fresh checkout rather than
 * from whatever kernel happens to be installed. Only 0e8d:7612 is
 * hardware-verified here (an Alfa AWUS036ACM: ASIC version 0x76120044, EEPROM
 * chip id 0x7612 — docs/mt7612u.md); the rest are the same MAC behind an OEM
 * identity and are listed so they are refused rather than misdetected. The
 * library's own identify step (MT_ASIC_VERSION >> 16 == 0x7612, usb.cpp) stays
 * authoritative after the handle is open: an entry here only decides which
 * backend gets to look.
 *
 * WHY VID:PID AND NOT VID. Six of these vendor ids also ship Realtek silicon
 * devourer serves, and the two families are not merely adjacent — they
 * INTERLEAVE inside one vendor's product-id space:
 *
 *   0x0846 Netgear   MediaTek 9014, 9053   Realtek 9051, 9052, 9054
 *   0x056e ELECOM    MediaTek 400a         Realtek 4007, 400b, 400d, 400e, ...
 *   0x0b05 ASUS      MediaTek 17eb, 180b, 1833
 *                                          Realtek 17d2, 1817, 1852, 1853, 1a62
 *   0x2357 TP-Link   MediaTek 0137         Realtek 0101, 0103, 0106, 010d, ...
 *   0x7392 Edimax    MediaTek b711         Realtek 6822, a811, a812, a813, ...
 *   0x2c4e Mercury   MediaTek 0103         Realtek 0127
 *
 * A vendor-id gate would therefore refuse working Realtek adapters, and a
 * single mistyped product id here would refuse a specific one. The full PAIR
 * sets, by contrast, are disjoint: none of the 16 ids below matches any of the
 * 91 Realtek ids devourer can serve (64 Jaguar1/2/3 from
 * reference/rtl8812au, plus KestrelUsbIds.h and Rtl8733bUsbIds.h). That is what
 * makes the pair gate authoritative ahead of the Realtek SYS_CFG2 read — the
 * same argument KestrelUsbIds.h relies on.
 *
 * Mt7612uUsbIdsSelftest.cpp checks that disjointness against all 91, not just
 * the tabled 27: Jaguar dispatches by chip-id and has no table in devourer, so
 * a check limited to the tables here would be blind to the very ids that
 * interleave above. */

struct UsbId {
  uint16_t vid;
  uint16_t pid;
};

inline constexpr UsbId kUsbIds[] = {
    {0x0b05, 0x1833}, /* Asus USB-AC54 */
    {0x0b05, 0x17eb}, /* Asus USB-AC55 */
    {0x0b05, 0x180b}, /* Asus USB-N53 B1 */
    {0x0e8d, 0x7612}, /* Aukey USBAC1200 / Alfa AWUS036ACM — the verified part */
    {0x057c, 0x8503}, /* AVM FRITZ!WLAN AC860 */
    {0x7392, 0xb711}, /* Edimax EW-7722UAC */
    {0x056e, 0x400a}, /* ELECOM WDC-867SU3S */
    {0x0e8d, 0x7632}, /* HC-M7662BU1 */
    {0x0471, 0x2126}, /* LiteOn WN4516R module */
    {0x0471, 0x7600}, /* LiteOn WN4519R module */
    {0x2c4e, 0x0103}, /* Mercury UD13 */
    {0x0846, 0x9014}, /* Netgear WNDA3100v3 */
    {0x0846, 0x9053}, /* Netgear A6210 */
    {0x045e, 0x02e6}, /* XBox One Wireless Adapter */
    {0x045e, 0x02fe}, /* XBox One Wireless Adapter */
    {0x2357, 0x0137}, /* TP-Link TL-WDN6200 */
};

inline bool is_usb_id(uint16_t vid, uint16_t pid) {
  for (const UsbId &id : kUsbIds)
    if (id.vid == vid && id.pid == pid)
      return true;
  return false;
}

} // namespace mt7612u

#endif /* MT7612U_USB_IDS_H */
