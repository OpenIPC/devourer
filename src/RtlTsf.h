#ifndef DEVOURER_RTL_TSF_H
#define DEVOURER_RTL_TSF_H

#include <cstdint>
#include <exception>

#include "RtlAdapter.h"

/* REG_TSFTR — the port-0 MAC TSF on the Realtek generations that expose it
 * as a plain register pair: 0x0560 (low 32) / 0x0564 (high 32). Shared by the
 * Jaguar1/2/3 ReadTsf/WriteTsf overrides so the read discipline and the
 * write's success rule live in one place. These helpers take no lock; each
 * backend serializes as it needs (Jaguar2/3 hold _reg_mu against their coex
 * tick, Jaguar1 holds _port0_mu for the write so it cannot interleave with a
 * beacon steer). A lock keeps the two words atomic against other register
 * users, not a caller's read-compute-write: a write that waits behind a long
 * sequence lands its precomputed value that much late. */

namespace devourer {

/* hi, lo, hi again, and retry the pair once if the low word wrapped between
 * the reads. */
inline uint64_t read_tsftr(RtlAdapter &a) {
  uint32_t hi = a.rtw_read<uint32_t>(0x0564);
  uint32_t lo = a.rtw_read<uint32_t>(0x0560);
  if (a.rtw_read<uint32_t>(0x0564) != hi) {
    hi = a.rtw_read<uint32_t>(0x0564);
    lo = a.rtw_read<uint32_t>(0x0560);
  }
  return (static_cast<uint64_t>(hi) << 32) | lo;
}

/* How far past the target a PCIe readback may land and still count as the
 * write having taken. The MMIO store-then-read round trip is µs-scale; the
 * slack only absorbs a host preemption between the two. */
inline constexpr uint64_t kTsfWriteReadbackWindowUs = 100000;

/* Load the counter. Both words are always attempted, low then high: the pair
 * is the unit, and a half-written TSF is worse than a reported failure.
 *
 * True means the transport accepted both writes, and on USB that alone is two
 * completed control transfers - with one exception. The counter runs between
 * the two transfers, so a target whose low word is within the window of
 * wrapping can carry into the high word before the high write lands, leaving
 * target - 2^32 behind two successful transfers. That case is read back.
 *
 * A PCIe register write is a posted MMIO store with no completion
 * (PcieTransport::guarded_write reports true for any register below the USB
 * page, whether or not the device is still there), so on PCIe every write is
 * read back: true only when the counter reads within kTsfWriteReadbackWindowUs
 * after the target. A device that has left the bus typically reads all-ones
 * and fails that (platform behaviour, not measured here). A readback cannot
 * tell "ignored" from "took" when the target sits less than the window behind
 * the live clock (an ignored small backward nudge still reads as target + a
 * little), so it catches a dead transport or a torn pair, not an ignored
 * nudge. A USB register read throws on a failed transfer where a write
 * returns false; the readback folds that into false so the bool stays the
 * whole answer. */
inline bool write_tsftr(RtlAdapter &a, uint64_t tsf) {
  const bool lo_ok = a.rtw_write<uint32_t>(0x0560, static_cast<uint32_t>(tsf));
  const bool hi_ok =
      a.rtw_write<uint32_t>(0x0564, static_cast<uint32_t>(tsf >> 32));
  if (!(lo_ok && hi_ok))
    return false;
  const bool near_wrap = static_cast<uint32_t>(tsf) >
                         UINT32_MAX - kTsfWriteReadbackWindowUs;
  if (a.is_usb() && !near_wrap)
    return true;
  try {
    return read_tsftr(a) - tsf < kTsfWriteReadbackWindowUs;
  } catch (const std::exception &) {
    return false;
  }
}

} // namespace devourer

#endif /* DEVOURER_RTL_TSF_H */
