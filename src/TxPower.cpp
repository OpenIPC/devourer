#include "TxPower.h"

namespace devourer {

int quantize_offset_qdb(int qdb, const TxPowerCaps &caps, int *steps_out) {
  if (steps_out)
    *steps_out = 0;
  if (!caps.supported || caps.step_qdb == 0)
    return 0;
  /* Clamp the request to the representable range first so the rounding below
   * cannot step past a rail. */
  if (qdb > caps.offset_max_qdb)
    qdb = caps.offset_max_qdb;
  if (qdb < caps.offset_min_qdb)
    qdb = caps.offset_min_qdb;
  /* Round to the nearest whole step, ties away from zero: a controller asking
   * for +1 qdB on a 0.5 dB-step family gets +1 step (0.5 dB), not silence. */
  const int step = caps.step_qdb;
  int steps;
  if (qdb >= 0)
    steps = (qdb + step / 2) / step;
  else
    steps = -((-qdb + step / 2) / step);
  if (steps_out)
    *steps_out = steps;
  return steps * step;
}

uint32_t pack_rate_diff_word(int8_t d0, int8_t d1, int8_t d2, int8_t d3) {
  const uint32_t b0 = static_cast<uint32_t>(d0) & 0x7fu;
  const uint32_t b1 = static_cast<uint32_t>(d1) & 0x7fu;
  const uint32_t b2 = static_cast<uint32_t>(d2) & 0x7fu;
  const uint32_t b3 = static_cast<uint32_t>(d3) & 0x7fu;
  return b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
}

} // namespace devourer
