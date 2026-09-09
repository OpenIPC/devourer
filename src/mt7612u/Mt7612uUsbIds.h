#ifndef MT7612U_USB_IDS_H
#define MT7612U_USB_IDS_H

#include <cstdint>

namespace mt7612u {

/* USB identities of the MediaTek MT7662-MAC adapters (MT7612U / MT7662U).
 *
 * Transcribed from the mainline mt76 driver's mt76x2u_device_table
 * (drivers/net/wireless/mediatek/mt76/mt76x2/usb.c) — the set of devices the
 * kernel binds to this MAC. Only 0e8d:7612 is hardware-verified here (an Alfa
 * AWUS036ACM: ASIC version 0x76120044, EEPROM chip id 0x7612 — docs/mt7612u.md);
 * the rest are the same MAC behind an OEM identity and are listed so they are
 * refused rather than misdetected. The library's own identify step
 * (MT_ASIC_VERSION >> 16 == 0x7612, usb.c) stays authoritative after the handle
 * is open: an entry here only decides which backend gets to look.
 *
 * WHY VID:PID AND NOT VID. Three of these vendor ids also appear in devourer's
 * Realtek tables — 0x0b05 (ASUS, RTL8832BU 0b05:1a62), 0x7392 (Edimax,
 * RTL8812AU 7392:6822) and 0x2c4e (Mercury, 2c4e:0127) — because OEMs ship both
 * silicon families under one vendor id. A vendor-id gate would therefore refuse
 * working Realtek adapters. The full pair sets, by contrast, are disjoint: no
 * vid:pid below matches any id in KestrelUsbIds.h or Rtl8733bUsbIds.h, which is
 * what makes the pair gate authoritative ahead of the Realtek SYS_CFG2 read
 * (the same argument KestrelUsbIds.h relies on). See Mt7612uUsbIdsSelftest.cpp,
 * which fails if a future id addition breaks that disjointness. */

struct UsbId {
  uint16_t vid;
  uint16_t pid;
};

inline constexpr UsbId kUsbIds[] = {
    {0x0e8d, 0x7612}, /* Alfa AWUS036ACM / Aukey USBAC1200 — the verified part */
    {0x0e8d, 0x7632}, /* HC-M7662BU1 (MT7662U) */
    {0x0b05, 0x1833}, /* Asus USB-AC54 */
    {0x0b05, 0x17eb}, /* Asus USB-AC55 */
    {0x0b05, 0x180b}, /* Asus USB-N53 B1 */
    {0x057c, 0x8503}, /* AVM FRITZ!WLAN AC860 */
    {0x7392, 0xb711}, /* Edimax EW-7722UAC */
    {0x2c4e, 0x0103}, /* Mercury UD13 */
    {0x0846, 0x9053}, /* Netgear A6210 */
    {0x045e, 0x02e6}, /* XBox One Wireless Adapter */
    {0x045e, 0x02fe}, /* XBox One Wireless Adapter */
};

inline bool is_usb_id(uint16_t vid, uint16_t pid) {
  for (const UsbId &id : kUsbIds)
    if (id.vid == vid && id.pid == pid)
      return true;
  return false;
}

} // namespace mt7612u

#endif /* MT7612U_USB_IDS_H */
