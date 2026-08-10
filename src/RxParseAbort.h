#ifndef DEVOURER_RX_PARSE_ABORT_H
#define DEVOURER_RX_PARSE_ABORT_H

/* rx.parse_abort — the RX descriptor walk hit a malformed/truncated
 * descriptor mid-aggregate and abandoned the rest of the bulk-IN buffer.
 * Every abandoned frame was already admitted by the chip (and, with an ACK
 * responder armed, already ACKed to the peer), so this is post-admission
 * loss a hardware-ARQ peer counts as delivered — it must never be silent.
 * Normal end-of-aggregate zero padding (all-zero remainder) is excluded.
 * Shared by every generation's RX walk; schema: docs/logging.md. */

#include <cstddef>
#include <cstdint>

#include "Event.h"

namespace devourer {

/* Returns true when the remainder was a real abort (event emitted),
 * false for benign all-zero padding. `total` is the caller's cumulative
 * abort counter, incremented on emit. */
inline bool emit_rx_parse_abort(EventSink &sink, const uint8_t *rem,
                                size_t rem_len, long long off,
                                long long buf_len, long long frame_len,
                                long long drvinfo, long long shift,
                                long long &total) {
  if (!sink.enabled())
    return false;
  bool all_zero = true;
  for (size_t i = 0; i < rem_len; ++i)
    if (rem[i] != 0) {
      all_zero = false;
      break;
    }
  if (all_zero)
    return false;
  ++total;
  Ev(sink, "rx.parse_abort")
      .t()
      .f("off", off)
      .f("buf_len", buf_len)
      .f("remaining", static_cast<long long>(rem_len))
      .f("frame_len", frame_len)
      .f("drvinfo", drvinfo)
      .f("shift", shift)
      .f("total", total);
  return true;
}

} // namespace devourer

#endif /* DEVOURER_RX_PARSE_ABORT_H */
