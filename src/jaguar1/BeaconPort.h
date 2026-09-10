/* Jaguar1 port-0 beacon control helpers.
 *
 * The beacon engine is autonomous once armed.  Its logical port ownership may
 * therefore be released only after all three hardware stop controls read back
 * inactive, irrespective of the transport write return values. */
#ifndef DEVOURER_JAGUAR1_BEACON_PORT_H
#define DEVOURER_JAGUAR1_BEACON_PORT_H

#include <cstdint>

#include "RtlAdapter.h"

namespace devourer::jaguar1 {

struct BeaconStopResult {
  bool transfers_ok = false;
  bool function_off = false;
  bool tx_stopped = false;
  bool no_link = false;

  bool verified() const noexcept {
    return function_off && tx_stopped && no_link;
  }
};

inline BeaconStopResult stop_port0_beacon_verified(RtlAdapter &dev) noexcept {
  auto write8 = [&dev](uint16_t reg, uint8_t value) noexcept {
    try {
      return dev.rtw_write8(reg, value);
    } catch (...) {
      return false;
    }
  };
  auto clear8 = [&dev](uint16_t reg, uint8_t mask) noexcept {
    try {
      const uint8_t value = dev.rtw_read8(reg);
      return dev.rtw_write8(reg, static_cast<uint8_t>(value & ~mask));
    } catch (...) {
      return false;
    }
  };
  auto bits_clear = [&dev](uint16_t reg, uint8_t mask) noexcept {
    try {
      return (dev.rtw_read8(reg) & mask) == 0;
    } catch (...) {
      return false;
    }
  };

  /* Keep the operations independent: a failure stopping one control must not
   * suppress the attempts to stop the other two. */
  const bool function_transfer = write8(0x0550, 0x10);
  const bool tx_transfer = clear8(0x0422, 0x40);
  const bool link_transfer = clear8(0x0102, 0x03);

  BeaconStopResult result;
  result.transfers_ok = function_transfer && tx_transfer && link_transfer;
  result.function_off = bits_clear(0x0550, 0x08);
  result.tx_stopped = bits_clear(0x0422, 0x40);
  result.no_link = bits_clear(0x0102, 0x03);
  return result;
}

} // namespace devourer::jaguar1

#endif // DEVOURER_JAGUAR1_BEACON_PORT_H
