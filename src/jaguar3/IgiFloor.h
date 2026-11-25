#pragma once
#include <cstdint>

namespace jaguar3 {

/* The IGI to write when the wide-TX floor moves from `prev_floor` to `floor`,
 * or -1 for no write. `window_hi` is the top of the normal DIG window on the
 * current channel. A non-zero floor lifts IGI up to it; a floor that was
 * lowered or dropped pulls IGI down to the new floor (never below it) rather
 * than leaving the raised gain until the next DIG tick. */
constexpr int igi_on_floor_change(uint8_t prev_floor, uint8_t floor,
                                  uint8_t cur_igi, uint8_t window_hi) {
  if (floor != 0 && cur_igi < floor)
    return floor;
  if (floor < prev_floor) {
    const uint8_t hi = window_hi > floor ? window_hi : floor;
    if (cur_igi > hi)
      return hi;
  }
  return -1;
}

} // namespace jaguar3
