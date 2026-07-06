/* Headless guard for the per-packet TX-power quantizer (jaguar2
 * txpkt_pwr_step_for_db): maps a requested per-packet dB delta to the nearest
 * hardware TXPWR_OFSET LUT step {0, -3, -7, -11, +3, +6} dB. A rounding
 * regression fails `ctest` instead of only surfacing as wrong on-air power. */
#include <cstdio>

#include "jaguar2/FrameParserJaguar2.h"

static int g_fail = 0;

static void expect(int db, uint8_t want_step) {
  uint8_t got = jaguar2::txpkt_pwr_step_for_db(db);
  if (got == want_step)
    return;
  ++g_fail;
  std::printf("FAIL: %+d dB -> step %u, want %u (=%+d dB)\n", db, got,
              want_step, jaguar2::txpkt_pwr_db_for_step(want_step));
}

int main() {
  /* Exact LUT points. */
  expect(0, 0);
  expect(-3, 1);
  expect(-7, 2);
  expect(-11, 3);
  expect(3, 4);
  expect(6, 5);

  /* Nearest-rung rounding. */
  expect(-1, 0);   /* 1 from 0, 2 from -3 -> step 0 */
  expect(-2, 1);   /* 2 from 0, 1 from -3 -> -3 */
  expect(-5, 1);   /* 2 from -3, 2 from -7 -> ties to first (-3) */
  expect(-9, 2);   /* 2 from -7, 2 from -11 -> ties to first (-7) */
  expect(-20, 3);  /* clamps to the most-negative rung */
  expect(1, 0);    /* 1 from 0, 2 from +3 -> 0 */
  expect(2, 4);    /* 2 from 0, 1 from +3 -> +3 */
  expect(4, 4);    /* +3 */
  expect(5, 5);    /* 2 from +3, 1 from +6 -> +6 */
  expect(20, 5);   /* clamps to +6 */

  /* Inverse. */
  if (jaguar2::txpkt_pwr_db_for_step(2) != -7) {
    ++g_fail;
    std::printf("FAIL: db_for_step(2) != -7\n");
  }

  if (g_fail) {
    std::printf("%d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("txpkt-pwr quantizer selftest: all OK\n");
  return 0;
}
