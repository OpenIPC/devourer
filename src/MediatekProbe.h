#ifndef DEVOURER_MEDIATEK_PROBE_H
#define DEVOURER_MEDIATEK_PROBE_H

/* Chip-family routing for MediaTek USB ids that more than one backend lists.
 *
 * mt76 keeps one USB id table per family and lets probe() decide: an id in two
 * tables binds whichever driver's probe accepts MT_ASIC_VERSION. devourer
 * dispatches before any backend exists, so for those ids it reads the same
 * register itself, here, and routes on the answer. Today that is one id,
 * 7392:b711, listed by both mt76x0/usb.c and mt76x2/usb.c
 * (mt76x0u/Mt76x0uUsbIds.h).
 *
 * The family sets are mt76x02.h's is_mt76x0() and is_mt76x2() verbatim
 * (reference/mt76 @ be5ce79). The read is the transfer both MediaTek
 * backends use for every register (mt76's __mt76u_rr): vendor request
 * MULTI_READ (0x07), wValue = address bits 31:16, wIndex = bits 15:0, four
 * bytes little-endian. MT_ASIC_VERSION is offset 0 in the always-on CSR
 * block, so it answers before power-on and before firmware, which is when
 * mt76x2u_probe() and mt76x0u_probe() read it too.
 *
 * Synchronous libusb_control_transfer is safe here and only here: it runs in
 * WiFiDriver::CreateRadio, before a backend has started an event thread on the
 * context (see the sync-transfer note in mt7612u/usb.cpp for why the backends
 * themselves do not use it). */

#include <cstdint>
#include <optional>

#include <libusb.h>

namespace devourer {
namespace mediatek {

enum class Family { Unknown, Mt76x0, Mt76x2 };

/* mt76_chip(): the upper half of MT_ASIC_VERSION. */
constexpr uint16_t chip_of(uint32_t asic_version) {
  return static_cast<uint16_t>(asic_version >> 16);
}

constexpr Family family_of(uint32_t asic_version) {
  const uint16_t c = chip_of(asic_version);
  if (c == 0x7610 || c == 0x7630 || c == 0x7650)
    return Family::Mt76x0;
  if (c == 0x7612 || c == 0x7632 || c == 0x7662 || c == 0x7602)
    return Family::Mt76x2;
  return Family::Unknown;
}

constexpr const char *family_name(Family f) {
  return f == Family::Mt76x0   ? "mt76x0"
         : f == Family::Mt76x2 ? "mt76x2"
                               : "unknown";
}

constexpr uint8_t kVendMultiRead = 0x07;
constexpr uint32_t kRegAsicVersion = 0x0000;

/* nullopt when the transfer fails after `attempts` tries. */
inline std::optional<uint32_t>
read_asic_version(libusb_device_handle *h, int attempts = 3,
                  unsigned timeout_ms = 300) {
  if (!h)
    return std::nullopt;
  const uint8_t type = static_cast<uint8_t>(LIBUSB_ENDPOINT_IN) |
                       static_cast<uint8_t>(LIBUSB_REQUEST_TYPE_VENDOR) |
                       static_cast<uint8_t>(LIBUSB_RECIPIENT_DEVICE);
  for (int i = 0; i < attempts; ++i) {
    unsigned char b[4] = {0, 0, 0, 0};
    const int rc = libusb_control_transfer(
        h, type, kVendMultiRead,
        static_cast<uint16_t>(kRegAsicVersion >> 16),
        static_cast<uint16_t>(kRegAsicVersion & 0xffff), b, sizeof b,
        timeout_ms);
    if (rc == static_cast<int>(sizeof b))
      return static_cast<uint32_t>(b[0]) | (static_cast<uint32_t>(b[1]) << 8) |
             (static_cast<uint32_t>(b[2]) << 16) |
             (static_cast<uint32_t>(b[3]) << 24);
    if (rc == LIBUSB_ERROR_NO_DEVICE)
      break;
  }
  return std::nullopt;
}

} // namespace mediatek
} // namespace devourer

#endif /* DEVOURER_MEDIATEK_PROBE_H */
