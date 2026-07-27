#include <cstdio>
#include "TxPower.h"

static int fails = 0;
#define CHECK(c) do { if (!(c)) { std::fprintf(stderr, "FAIL: %s\n", #c); ++fails; } } while (0)

int main() {
  using devourer::pack_rate_diff_word;
  CHECK(pack_rate_diff_word(0, 0, 0, 0) == 0u);
  CHECK(pack_rate_diff_word(1, 2, 3, 4) == 0x04030201u);
  /* negative diffs are 7-bit two's complement per byte: -1 -> 0x7f */
  CHECK(pack_rate_diff_word(-1, 0, 0, 0) == 0x0000007fu);
  CHECK(pack_rate_diff_word(-24, 24, -1, 63) == // -24=0x68, 24=0x18, -1=0x7f, 63=0x3f
        0x3f7f1868u);
  /* bit 7 of every byte must be clear (the field is 7 bits wide) */
  CHECK((pack_rate_diff_word(-64, -64, -64, -64) & 0x80808080u) == 0u);
  devourer::TxRateDiffsQdb d;
  CHECK(d.cck == 0 && d.legacy == 0 && d.mcs[7] == 0); /* zero-init default */

  /* Quantization to the family step. Jaguar3/Kestrel (step 1) pass values
   * through; Jaguar1/Jaguar2 (step 2, i.e. 0.5 dB) round to nearest with ties
   * away from zero — the same rule quantize_offset_qdb applies to the offset
   * knob, so a request never vanishes on one knob and not the other. */
  using devourer::rate_diff_steps;
  CHECK(rate_diff_steps(0, 2) == 0);
  CHECK(rate_diff_steps(1, 2) == 1);   /* tie, away from zero */
  CHECK(rate_diff_steps(-1, 2) == -1);
  CHECK(rate_diff_steps(2, 2) == 1);
  CHECK(rate_diff_steps(3, 2) == 2);   /* tie */
  CHECK(rate_diff_steps(-3, 2) == -2);
  CHECK(rate_diff_steps(4, 2) == 2);
  CHECK(rate_diff_steps(-16, 2) == -8);
  CHECK(rate_diff_steps(-16, 1) == -16);
  CHECK(rate_diff_steps(63, 1) == 63);
  CHECK(rate_diff_steps(7, 0) == 0);   /* unsupported family */

  /* Rate -> field map. Everything the struct does not describe sits at the
   * anchor, which is a diff of 0. */
  using devourer::rate_diff_qdb_for_rate;
  devourer::TxRateDiffsQdb m;
  m.cck = 10;
  m.legacy = -4;
  for (int i = 0; i < 8; ++i)
    m.mcs[i] = static_cast<int8_t>(i + 1);
  const uint8_t cck_rates[] = {0x02, 0x04, 0x0b, 0x16};
  for (uint8_t r : cck_rates)
    CHECK(rate_diff_qdb_for_rate(r, m) == 10);
  const uint8_t ofdm_rates[] = {0x0c, 0x12, 0x18, 0x24, 0x30, 0x48, 0x60, 0x6c};
  for (uint8_t r : ofdm_rates)
    CHECK(rate_diff_qdb_for_rate(r, m) == -4);
  for (int i = 0; i < 8; ++i)
    CHECK(rate_diff_qdb_for_rate(static_cast<uint8_t>(0x80 + i), m) == i + 1);
  CHECK(rate_diff_qdb_for_rate(0x88, m) == 0); /* MCS8  (2SS) */
  CHECK(rate_diff_qdb_for_rate(0x8f, m) == 0); /* MCS15 */
  CHECK(rate_diff_qdb_for_rate(0x90, m) == 0); /* MCS16 (3SS) */
  CHECK(rate_diff_qdb_for_rate(0xa0, m) == 0); /* VHT1SS_MCS0 */
  CHECK(rate_diff_qdb_for_rate(0xb3, m) == 0); /* VHT2SS_MCS9 */
  CHECK(rate_diff_qdb_for_rate(0x7f, m) == 0); /* MCS32 */

  /* Storage clamp to the width of the hardware diff field. */
  using devourer::clamp_rate_diffs;
  CHECK(!clamp_rate_diffs(std::nullopt).has_value());
  devourer::TxRateDiffsQdb wide;
  wide.cck = 100;
  wide.legacy = -100;
  wide.mcs[0] = 63;
  wide.mcs[1] = -64;
  wide.mcs[2] = 7;
  const auto clamped = clamp_rate_diffs(wide);
  CHECK(clamped.has_value());
  CHECK(clamped->cck == 63);
  CHECK(clamped->legacy == -64);
  CHECK(clamped->mcs[0] == 63);
  CHECK(clamped->mcs[1] == -64);
  CHECK(clamped->mcs[2] == 7);

  /* The contract, composed: a zero table is a no-op at the reference rate and
   * flattens every other rate onto it. */
  const devourer::TxRateDiffsQdb zero;
  const int anchor = 40;
  CHECK(anchor + rate_diff_steps(rate_diff_qdb_for_rate(0x87, zero), 2) == anchor);
  CHECK(anchor + rate_diff_steps(rate_diff_qdb_for_rate(0x02, zero), 2) == anchor);
  CHECK(anchor + rate_diff_steps(rate_diff_qdb_for_rate(0x80, zero), 2) == anchor);
  /* ...and a -16 qdB trim at MCS7 is -8 index steps on a 0.5 dB family. */
  devourer::TxRateDiffsQdb trim;
  trim.mcs[7] = -16;
  CHECK(anchor + rate_diff_steps(rate_diff_qdb_for_rate(0x87, trim), 2) == anchor - 8);
  CHECK(anchor + rate_diff_steps(rate_diff_qdb_for_rate(0x87, trim), 1) == anchor - 16);

  if (fails) { std::fprintf(stderr, "%d failure(s)\n", fails); return 1; }
  std::puts("rate_diffs_selftest OK");
  return 0;
}
