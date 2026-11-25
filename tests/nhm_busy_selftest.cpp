/* Headless guard for the busy-NHM register composers (src/jaguar3/
 * NhmBusyMath.h). The failure this exists for is silent: a threshold in
 * the wrong byte or a clobbered neighbour bit reads on air as a plausible
 * but wrong histogram. Every composed dword is checked against a value
 * built by hand from the register map, and every bit OUTSIDE the fields is
 * checked to round-trip from the current register value. */
#include "jaguar3/NhmBusyMath.h"
#include "NoiseFloorMath.h"

#include <cstdio>

static int fails = 0;
#define CHECK(c)                                                               \
  do {                                                                         \
    if (!(c)) {                                                                \
      std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #c);         \
      ++fails;                                                                 \
    }                                                                          \
  } while (0)

int main() {
  using namespace devourer::jgr3;
  CHECK(kNhmBusyCfg == 0x3); /* ccx_en|inc_cca, inc_tx off */
  const uint8_t th[11] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
  // Neighbour bits set outside the fields (the q vector below covers the
  // fields starting set) so a composer that zeroes them fails.
  const uint32_t c40 = 0x0000abcd, c5c = 0xff00ffff, c60 = 0x0000f0fd;
  const NhmProgram p = compose_nhm_program(c40, c5c, c60, th, kNhmBusyCfg, 0x1234);
  CHECK(p.w1e40 == 0x1234abcdu);                 // period [31:16], low half kept
  CHECK(p.w1e44 == 0x04030201u);                 // th0..3
  CHECK(p.w1e48 == 0x08070605u);                 // th4..7
  CHECK(p.w1e5c == 0xff09ffffu);                 // th8 in [23:16], rest kept
  // ctrl: th10<<24 | th9<<16 | cfg<<8 | low bits kept except trigger [1] and [11:8]
  CHECK(p.w1e60 == ((11u << 24) | (10u << 16) | (0x3u << 8) | (c60 & 0xf0fdu)));
  CHECK((p.w1e60 & 0x2u) == 0);                  // trigger left low by the program
  CHECK(nhm_with_period(0x1234abcd, 0xffff) == 0xffffabcdu);
  CHECK(nhm_trigger_low(0xffffffff) == 0xfffffffdu);
  CHECK(nhm_trigger_high(0) == 0x2u);
  // Opposite-polarity vector: cfg bits and trigger start set.
  const NhmProgram q = compose_nhm_program(0xffffffff, 0, 0xffffffff, th, 0x1, 1);
  CHECK(q.w1e40 == 0x0001ffffu);
  CHECK(q.w1e5c == 0x00090000u);
  CHECK((q.w1e60 & 0xf00u) == 0x100u);
  CHECK((q.w1e60 & 0x2u) == 0);
  CHECK((q.w1e60 & 0xffff0000u) == ((11u << 24) | (10u << 16)));
  // Period per BB tick: 4/8/16 us at 20/10/5 MHz, clamped to [1, 65535].
  CHECK(nhm_period_for_us(240000, 0) == 60000);
  CHECK(nhm_period_for_us(240000, 1) == 30000);   // 10 MHz: 8 us tick
  CHECK(nhm_period_for_us(240000, 2) == 15000);   // 5 MHz: 16 us tick
  CHECK(nhm_period_for_us(1, 0) == 1);
  CHECK(nhm_period_for_us(300000, 0) == 0xffff);
  CHECK(nhm_period_for_us(300000, 1) == 37500);   // fits at 8 us
  // Result parse.
  const NhmResult r = parse_nhm_result(0x0001002a, 0x04030201, 0x08070605, 0x0c0b0a09);
  CHECK(r.ready && r.duration == 0x2a);
  for (int i = 0; i < 12; ++i) CHECK(r.buckets[i] == i + 1);
  CHECK(!parse_nhm_result(0x0000002a, 0, 0, 0).ready);
  // Absolute table: PWdB U(8,1) = 2*(dBm+110), strictly rising, and the
  // bucket lookup the busy share is read from.
  using namespace devourer::nf;
  uint8_t ath[11];
  nhm_abs_thresholds(ath);
  CHECK(ath[0] == 12 && ath[7] == 54 && ath[10] == 80);  // -104, -83, -70 dBm
  for (int i = 1; i < 11; ++i) CHECK(ath[i] > ath[i - 1]);
  CHECK(nhm_abs_bucket_from(-104) == 1);
  CHECK(nhm_abs_bucket_from(-83) == 8);
  CHECK(nhm_abs_bucket_from(-70) == 11);
  CHECK(nhm_abs_bucket_from(-84) == -1);  // not an edge
  if (fails) return 1;
  std::printf("nhm_busy_selftest OK\n");
  return 0;
}
