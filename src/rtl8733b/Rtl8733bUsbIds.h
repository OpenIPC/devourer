#ifndef RTL8733B_USB_IDS_H
#define RTL8733B_USB_IDS_H

#include <cstdint>

namespace rtl8733b {

/* SYS_CFG2 is authoritative after the handle is open. The PID table is still
 * useful for discovery and for refusing a dangerous fallback when a register
 * read fails on a device the vendor driver identifies as RTL8733B-family. */
inline constexpr uint8_t kChipId = 0x16;

struct UsbId {
  uint16_t vid;
  uint16_t pid;
};

inline constexpr UsbId kUsbIds[] = {
    {0x0bda, 0xf72b}, /* Wi-Fi-only RTL8733BU reference identity */
    {0x0bda, 0xb733}, /* RTL8733BU combo-module Wi-Fi function */
};

inline bool is_usb_id(uint16_t vid, uint16_t pid) {
  for (const UsbId &id : kUsbIds)
    if (id.vid == vid && id.pid == pid)
      return true;
  return false;
}

inline bool is_chip_id(uint8_t chip_id) { return chip_id == kChipId; }

} // namespace rtl8733b

#endif /* RTL8733B_USB_IDS_H */
