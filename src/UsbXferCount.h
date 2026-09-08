/* Process-wide count of USB vendor control transfers (register reads and
 * writes). Bumped by UsbTransport; read by InitTimer so every init.timing
 * stage reports how many transfers it spent, which is the unit the
 * bring-up is actually paid in (each one is a synchronous EP0 round trip). */
#pragma once
#include <atomic>
#include <cstdint>

namespace devourer {
inline std::atomic<uint64_t> &usb_ctrl_xfers() {
  static std::atomic<uint64_t> n{0};
  return n;
}
} // namespace devourer
