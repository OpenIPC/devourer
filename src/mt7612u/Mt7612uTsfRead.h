#ifndef MT7612U_TSF_READ_H
#define MT7612U_TSF_READ_H

#include <cstdint>

#include "regs.h"

namespace mt7612u {

/*
 * A coherent read of the 64-bit TSF from its two 32-bit halves,
 * MT_TSF_TIMER_DW0 (low) and MT_TSF_TIMER_DW1 (high).
 *
 * WHY NOT TWO READS. Nothing latches the pair: a DW0 read does not freeze DW1,
 * so a read whose halves straddle the 2^32 µs low-word wrap tears by 2^32 µs.
 * Bring-up restarts the counter, so that is 71.6 min in. Measured on hardware
 * by the bringup `tsfwrap` gate; docs/mt7612u.md has the numbers.
 *
 * THE DISCIPLINE. High, low, high again; if the high word moved, the low word
 * wrapped somewhere in between, so read the low word once more and pair it with
 * the second high word. The retry cannot itself tear: that would take a second
 * wrap, 71.6 min later, inside two control transfers. Same idea as the Realtek
 * read_tsftr (src/RtlTsf.h), which re-reads both words instead.
 *
 * FAILURE. `rd(addr, &val)` returns 0 on success and nonzero on a failed
 * transfer; any failure fails the whole read, and *out is left untouched.
 * 0xffffffff is a legitimate word here (the low word passes through it once a
 * wrap), so no value can double as a failure sentinel - hence the separate
 * return.
 *
 * Pure: no device, no libusb, no lock (the caller serializes), so a headless
 * cell can drive it with a scripted register sequence
 * (tests/mt7612u_tsf_read_selftest.cpp).
 *
 * Returns 0 and fills *out, or -1. `*retried`, when given, reports whether the
 * wrap retry ran, so a test can show the path was exercised rather than merely
 * that nothing went wrong.
 */
template <typename Rd>
int tsf_read(Rd &&rd, uint64_t *out, bool *retried = nullptr) {
  uint32_t hi, lo, hi2;

  if (retried)
    *retried = false;
  if (rd(MT_TSF_TIMER_DW1, &hi) || rd(MT_TSF_TIMER_DW0, &lo) ||
      rd(MT_TSF_TIMER_DW1, &hi2))
    return -1;
  if (hi2 != hi) {
    if (rd(MT_TSF_TIMER_DW0, &lo))
      return -1;
    hi = hi2;
    if (retried)
      *retried = true;
  }
  *out = (static_cast<uint64_t>(hi) << 32) | lo;
  return 0;
}

} // namespace mt7612u

#endif /* MT7612U_TSF_READ_H */
