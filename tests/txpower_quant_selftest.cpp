/* Headless guard for the TX-power offset quantization (src/TxPower.h) — the
 * qdB -> index-step rounding (round-nearest, ties away from zero) and range
 * clamping behind SetTxPowerOffsetQdb, so a rounding regression fails `ctest`
 * instead of only surfacing as a wrong on-air level. Prints the failing case
 * and exits nonzero. */
#include <cstdio>

#include "TxPower.h"

static int g_fail = 0;

static void expect(const devourer::TxPowerCaps &caps, int qdb, int want_qdb,
                   int want_steps) {
  int steps = -9999;
  const int got = devourer::quantize_offset_qdb(qdb, caps, &steps);
  if (got == want_qdb && steps == want_steps)
    return;
  ++g_fail;
  std::printf("FAIL: step=%u range=[%d,%d] qdb=%d -> (%d qdb, %d steps) "
              "want (%d qdb, %d steps)\n",
              caps.step_qdb, caps.offset_min_qdb, caps.offset_max_qdb, qdb,
              got, steps, want_qdb, want_steps);
}

int main() {
  /* Jaguar1/Jaguar2-shaped caps: 6-bit index, 0.5 dB (2 qdB) per step. */
  devourer::TxPowerCaps half{};
  half.supported = true;
  half.index_max = 63;
  half.step_qdb = 2;
  half.offset_min_qdb = -126;
  half.offset_max_qdb = 126;

  expect(half, 0, 0, 0);
  expect(half, 2, 2, 1);    /* exact step */
  expect(half, -2, -2, -1);
  expect(half, 1, 2, 1);    /* tie (0.25 dB from both) -> away from zero */
  expect(half, -1, -2, -1);
  expect(half, 3, 4, 2);    /* 0.75 dB tie -> 1.0 dB */
  expect(half, -3, -4, -2);
  expect(half, 4, 4, 2);
  expect(half, -24, -24, -12); /* the -6 dB validation cell */
  expect(half, 126, 126, 63);  /* rails */
  expect(half, 127, 126, 63);
  expect(half, 200, 126, 63);
  expect(half, -200, -126, -63);

  /* Jaguar3-shaped caps: 7-bit reference, 0.25 dB (1 qdB) per step. */
  devourer::TxPowerCaps quarter{};
  quarter.supported = true;
  quarter.index_max = 127;
  quarter.step_qdb = 1;
  quarter.offset_min_qdb = -127;
  quarter.offset_max_qdb = 127;

  expect(quarter, 0, 0, 0);
  expect(quarter, 1, 1, 1);   /* native quarter-dB, no rounding */
  expect(quarter, -1, -1, -1);
  expect(quarter, 24, 24, 24);
  expect(quarter, -24, -24, -24);
  expect(quarter, 127, 127, 127);
  expect(quarter, 200, 127, 127);
  expect(quarter, -200, -127, -127);

  /* Unsupported caps: everything is a 0-applied no-op. */
  devourer::TxPowerCaps none{};
  expect(none, 24, 0, 0);
  expect(none, -24, 0, 0);

  if (g_fail) {
    std::printf("%d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("txpower quantization selftest: all OK\n");
  return 0;
}
