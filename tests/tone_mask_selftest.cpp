/* Headless regression guard for the ToneMask tone-index math (src/ToneMask.h):
 * fc derivation (phydm_find_fc port), subcarrier enumeration for a frequency
 * range, and the env-spec parser. No libusb device is opened — only the pure
 * helpers are exercised — but the header pulls in RtlUsbAdapter.h, so the
 * target links like the demos. Register encodings are validated on hardware
 * (readback logging in apply_from_env), not here. */
#include <cstdio>
#include <cstdlib>

#include "ToneMask.h"

using namespace devourer::tonemask;

static int failures = 0;

#define CHECK_EQ(expr, want)                                                   \
  do {                                                                         \
    auto got_ = (expr);                                                        \
    if (got_ != (want)) {                                                      \
      std::fprintf(stderr, "FAIL %s:%d: %s = %ld, want %ld\n", __FILE__,       \
                   __LINE__, #expr, long(got_), long(want));                   \
      failures++;                                                              \
    }                                                                          \
  } while (0)

int main() {
  /* fc derivation — 5 GHz wide blocks center on the block, not the primary. */
  CHECK_EQ(find_fc_mhz(36, 80, kSecDontCare), 5210);  /* ch36-48 block  */
  CHECK_EQ(find_fc_mhz(48, 80, kSecDontCare), 5210);  /* any primary    */
  CHECK_EQ(find_fc_mhz(36, 40, kSecDontCare), 5190);  /* ch36+40        */
  CHECK_EQ(find_fc_mhz(100, 80, kSecDontCare), 5530); /* ch100-112      */
  CHECK_EQ(find_fc_mhz(149, 80, kSecDontCare), 5775); /* ch149-161      */
  CHECK_EQ(find_fc_mhz(36, 20, kSecDontCare), 5180);
  /* 2.4 GHz: 40 MHz fc via the secondary position; 80 MHz is invalid. */
  CHECK_EQ(find_fc_mhz(6, 20, kSecDontCare), 2437);
  CHECK_EQ(find_fc_mhz(6, 40, kSecAbove), 2447);
  CHECK_EQ(find_fc_mhz(6, 40, kSecBelow), 2427);
  CHECK_EQ(find_fc_mhz(11, 40, kSecAbove), 0); /* no room above ch>=10 */
  CHECK_EQ(find_fc_mhz(1, 40, kSecBelow), 0);  /* no room below ch<=2  */
  CHECK_EQ(find_fc_mhz(6, 80, kSecDontCare), 0);
  CHECK_EQ(find_fc_mhz(15, 20, kSecDontCare), 0); /* dead channel */

  /* Subcarrier enumeration — mask the top 20 MHz slice (5230-5250) of the
   * ch36-48 80 MHz block (fc 5210): tones +64..+128 inclusive = 65. */
  auto top = enumerate_tones(5210, 80, 5230000, 5250000);
  CHECK_EQ(top.size(), 65);
  CHECK_EQ(top.front(), 64);
  CHECK_EQ(top.back(), 128);
  /* Bottom slice (5170-5190): -128..-64. */
  auto bot = enumerate_tones(5210, 80, 5170000, 5190000);
  CHECK_EQ(bot.size(), 65);
  CHECK_EQ(bot.front(), -128);
  CHECK_EQ(bot.back(), -64);
  /* A DC-spanning middle range. */
  auto mid = enumerate_tones(5210, 80, 5209000, 5211000);
  CHECK_EQ(mid.size(), 7); /* -3.2..+3.2 -> -3..+3 */
  CHECK_EQ(mid.front(), -3);
  CHECK_EQ(mid.back(), 3);
  /* Clipping at the RF edge: a spec wider than the channel. */
  auto wide = enumerate_tones(5210, 80, 5100000, 5300000);
  CHECK_EQ(wide.size(), 257); /* -128..+128 */
  /* Single-frequency spec on an exact tone / between tones. */
  auto exact = enumerate_tones(5210, 80, 5220000, 5220000);
  CHECK_EQ(exact.size(), 1);
  CHECK_EQ(exact.front(), 32);
  auto between = enumerate_tones(5210, 80, 5211000, 5211000);
  CHECK_EQ(between.size(), 1);
  CHECK_EQ(between.front(), 3); /* 3.2 rounds to 3 */
  /* Out-of-band single frequency -> nothing. */
  auto oob = enumerate_tones(5210, 80, 5300000, 5300000);
  CHECK_EQ(oob.size(), 0);

  /* 20 MHz channel budget: +-32 tones. */
  auto ch20 = enumerate_tones(5180, 20, 5170000, 5190000);
  CHECK_EQ(ch20.size(), 65);
  CHECK_EQ(ch20.front(), -32);
  CHECK_EQ(ch20.back(), 32);

  /* Spec parsing. */
  auto s1 = parse_csi_spec("5230-5250");
  CHECK_EQ(s1.valid, true);
  CHECK_EQ(s1.f_lo_khz, 5230000);
  CHECK_EQ(s1.f_hi_khz, 5250000);
  CHECK_EQ(s1.wgt, 7);
  auto s2 = parse_csi_spec("5230-5250/5");
  CHECK_EQ(s2.valid, true);
  CHECK_EQ(s2.wgt, 5);
  auto s3 = parse_csi_spec("2447");
  CHECK_EQ(s3.valid, true);
  CHECK_EQ(s3.f_lo_khz, 2447000);
  CHECK_EQ(s3.f_hi_khz, 2447000);
  CHECK_EQ(parse_csi_spec("junk").valid, false);
  CHECK_EQ(parse_csi_spec("5250-5230").valid, false);
  CHECK_EQ(parse_csi_spec("").valid, false);
  CHECK_EQ(parse_csi_spec("5230-").valid, false);

  /* Prime-offset -> vendor secondary mapping. */
  CHECK_EQ(sec_from_prime_offset(0), kSecDontCare);
  CHECK_EQ(sec_from_prime_offset(1), kSecAbove); /* primary LOWER */
  CHECK_EQ(sec_from_prime_offset(2), kSecBelow); /* primary UPPER */

  if (failures) {
    std::fprintf(stderr, "tone_mask_selftest: %d failure(s)\n", failures);
    return 1;
  }
  std::printf("tone_mask_selftest: all checks passed\n");
  return 0;
}
