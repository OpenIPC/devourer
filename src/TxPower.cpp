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

int rate_diff_steps(int qdb, uint8_t step_qdb) {
  if (step_qdb == 0)
    return 0;
  const int step = step_qdb;
  if (qdb >= 0)
    return (qdb + step / 2) / step;
  return -((-qdb + step / 2) / step);
}

int rate_diff_qdb_for_rate(uint8_t mgn_rate, const TxRateDiffsQdb &d) {
  /* MGN_* values from RateDefinitions.h, spelled numerically so this stays a
   * neutral translation unit: CCK 1M/2M/5.5M/11M = 0x02/0x04/0x0b/0x16, the
   * eight OFDM rates 6M..54M, HT MCS0..7 = 0x80..0x87. */
  switch (mgn_rate) {
  case 0x02:
  case 0x04:
  case 0x0b:
  case 0x16:
    return d.cck;
  case 0x0c:
  case 0x12:
  case 0x18:
  case 0x24:
  case 0x30:
  case 0x48:
  case 0x60:
  case 0x6c:
    return d.legacy;
  default:
    break;
  }
  if (mgn_rate >= 0x80 && mgn_rate <= 0x87)
    return d.mcs[mgn_rate - 0x80];
  /* HT MCS8+, all VHT/HE, every 2SS+ stream: no diff, i.e. the anchor. */
  return 0;
}

std::optional<TxRateDiffsQdb>
clamp_rate_diffs(const std::optional<TxRateDiffsQdb> &d) {
  if (!d)
    return std::nullopt;
  auto cl = [](int8_t v) -> int8_t { return v < -64 ? -64 : (v > 63 ? 63 : v); };
  TxRateDiffsQdb out = *d;
  out.cck = cl(out.cck);
  out.legacy = cl(out.legacy);
  for (int i = 0; i < 8; ++i)
    out.mcs[i] = cl(out.mcs[i]);
  return out;
}

uint32_t pack_rate_diff_word(int8_t d0, int8_t d1, int8_t d2, int8_t d3) {
  const uint32_t b0 = static_cast<uint32_t>(d0) & 0x7fu;
  const uint32_t b1 = static_cast<uint32_t>(d1) & 0x7fu;
  const uint32_t b2 = static_cast<uint32_t>(d2) & 0x7fu;
  const uint32_t b3 = static_cast<uint32_t>(d3) & 0x7fu;
  return b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
}

} // namespace devourer
