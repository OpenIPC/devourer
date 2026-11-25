#ifndef MT76X0U_USB_IDS_H
#define MT76X0U_USB_IDS_H

#include <cstdint>

namespace mt76x0u {

/* USB identities of the MediaTek MT76x0U adapters (MT7610U / MT7630U /
 * MT7650U: the mt76x0 MAC, 1T1R 802.11ac).
 *
 * The complete mt76x0_device_table, transcribed from the pinned reference tree
 * reference/mt76 @ be5ce79, mt76x0/usb.c:14-44, and held to it by
 * tests/mt76x0u_usb_ids_vs_mt76.py. No entry is hardware-verified yet; the
 * development target is 0e8d:7610 (sold as the Panda PAU08 and the Sabrent
 * NTWLAC). The backend's own identify step (MT_ASIC_VERSION >> 16 in
 * {0x7610, 0x7630, 0x7650}, mt76x02.h is_mt76x0) stays authoritative once the
 * handle is open: an entry here only decides which backend gets to look.
 *
 * WHY VID:PID AND NOT VID: the same argument as mt7612u/Mt7612uUsbIds.h, and
 * sharper here, because this table interleaves with Realtek ids devourer
 * serves, mostly at product-id distance ONE:
 *
 *   0x0586 ZyXEL     MediaTek 3425          Realtek 3426
 *   0x04bb I-O DATA  MediaTek 0951          Realtek 0952, 0953
 *   0x2019 Planex    MediaTek ab31          Realtek ab30, ab32
 *   0x0df6 Sitecom   MediaTek 0075, 0079    Realtek 0074
 *   0x13b1 Linksys   MediaTek 003e          Realtek 003f
 *   0x2357 TP-Link   MediaTek 0105, 010b,   Realtek 0101, 0103, 0106, 010d,
 *                             0123                  ..., 0120, 0122
 *
 * tests/mt76x0u_usb_ids_selftest.cpp checks the pair set against all 91
 * Realtek ids, not only the tabled ones.
 *
 * ONE ID IS SHARED WITH THE MT7612U TABLE. 7392:b711 sits in both kernel
 * tables (mt76x0/usb.c "Edimax / Elecom" and mt76x2/usb.c "Edimax
 * EW-7722UAC"); the kernel binds whichever driver's probe accepts the chip.
 * devourer does the same thing explicitly: for a shared id WiFiDriver reads
 * MT_ASIC_VERSION (MediatekProbe.h) and routes on the chip family. Every
 * other pair is in exactly one MediaTek table, which the selftest pins.
 *
 * Two kernel entries match on interface class too
 * (USB_DEVICE_AND_INTERFACE_INFO(0x0e8d, 0x7630|0x7650, 0xff, 0x2, 0xff)): the
 * combo dies expose a Bluetooth interface beside the vendor one. devourer
 * claims interface 0 by number, which on those dies is the vendor interface;
 * that is unverified until one is on a bench. */

/* driver_info = 1 in the kernel table: mt76x0u_probe() sets dev->no_2ghz. */
constexpr uint8_t kFlagNo2Ghz = 1u << 0;
/* Also in mt7612u::kUsbIds; route on MT_ASIC_VERSION, not on the id. */
constexpr uint8_t kFlagSharedWithMt7612u = 1u << 1;

struct UsbId {
  uint16_t vid;
  uint16_t pid;
  uint8_t flags;
};

inline constexpr UsbId kUsbIds[] = {
    {0x148f, 0x7610, 0},                      /* MT7610U */
    {0x13b1, 0x003e, 0},                      /* Linksys AE6000 */
    {0x0e8d, 0x7610, 0},                      /* Sabrent NTWLAC / Panda PAU08 */
    {0x7392, 0xa711, 0},                      /* Edimax 7711mac */
    {0x7392, 0xb711, kFlagSharedWithMt7612u}, /* Edimax / Elecom */
    {0x148f, 0x761a, 0},                      /* TP-Link TL-WDN5200 */
    {0x148f, 0x760a, 0},                      /* TP-Link unknown */
    {0x0b05, 0x17d1, 0},                      /* Asus USB-AC51 */
    {0x0b05, 0x17db, 0},                      /* Asus USB-AC50 */
    {0x0df6, 0x0075, 0},                      /* Sitecom WLA-3100 */
    {0x2019, 0xab31, 0},                      /* Planex GW-450D */
    {0x2001, 0x3d02, 0},                      /* D-LINK DWA-171 rev B1 */
    {0x0586, 0x3425, 0},                      /* Zyxel NWD6505 */
    {0x07b8, 0x7610, 0},                      /* AboCom AU7212 */
    {0x04bb, 0x0951, 0},                      /* I-O DATA WN-AC433UK */
    {0x057c, 0x8502, 0},                      /* AVM FRITZ!WLAN USB Stick AC 430 */
    {0x293c, 0x5702, 0},                      /* Comcast Xfinity KXW02AAA */
    {0x20f4, 0x806b, 0},                      /* TRENDnet TEW-806UBH */
    {0x7392, 0xc711, 0},                      /* Devolo Wifi ac Stick */
    {0x0df6, 0x0079, 0},                      /* Sitecom Europe B.V. ac Stick */
    {0x2357, 0x0123, 0},                      /* TP-LINK T2UHP_US_v1 */
    {0x2357, 0x010b, 0},                      /* TP-LINK T2UHP_UN_v1 */
    {0x2357, 0x0105, kFlagNo2Ghz},            /* TP-LINK Archer T1U */
    {0x0e8d, 0x7630, 0},                      /* MT7630U (interface-class match) */
    {0x0e8d, 0x7650, 0},                      /* MT7650U (interface-class match) */
};

inline const UsbId *find_usb_id(uint16_t vid, uint16_t pid) {
  for (const UsbId &id : kUsbIds)
    if (id.vid == vid && id.pid == pid)
      return &id;
  return nullptr;
}

inline bool is_usb_id(uint16_t vid, uint16_t pid) {
  return find_usb_id(vid, pid) != nullptr;
}

/* The pair is ours alone: no chip-id read needed to route it. */
inline bool is_exclusive_usb_id(uint16_t vid, uint16_t pid) {
  const UsbId *id = find_usb_id(vid, pid);
  return id && !(id->flags & kFlagSharedWithMt7612u);
}

inline bool usb_id_has_no_2ghz(uint16_t vid, uint16_t pid) {
  const UsbId *id = find_usb_id(vid, pid);
  return id && (id->flags & kFlagNo2Ghz);
}

} // namespace mt76x0u

#endif /* MT76X0U_USB_IDS_H */
