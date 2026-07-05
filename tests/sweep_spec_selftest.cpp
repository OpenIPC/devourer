/* Headless guard for the sweep/hop bin-list grammar (src/SweepSpec.h) — the
 * channel-list / channel-range / MHz-range parsing behind DEVOURER_RX_SWEEP
 * and DEVOURER_HOP_CHANNELS, so a grammar regression fails `ctest` instead of
 * only surfacing on a radio. Prints the failing spec and exits nonzero. */
#include <cstdio>
#include <vector>

#include "SweepSpec.h"

static int g_fail = 0;

static void expect(const char *spec, const std::vector<int> &want) {
  const std::vector<int> got = devourer::parse_sweep_spec(spec);
  if (got == want)
    return;
  ++g_fail;
  std::printf("FAIL: spec \"%s\" -> [", spec ? spec : "(null)");
  for (size_t i = 0; i < got.size(); i++)
    std::printf("%s%d", i ? "," : "", got[i]);
  std::printf("] want [");
  for (size_t i = 0; i < want.size(); i++)
    std::printf("%s%d", i ? "," : "", want[i]);
  std::printf("]\n");
}

int main() {
  /* Plain channel lists (the pre-existing grammar). */
  expect("1,6,11", {1, 6, 11});
  expect("36", {36});
  expect("", {});
  expect(nullptr, {});
  expect(",,6,", {6});

  /* Channel ranges. */
  expect("1-13", {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13});
  expect("36-48/4", {36, 40, 44, 48});
  expect("1-11/5", {1, 6, 11});
  expect("100-100", {100});

  /* MHz ranges (>= 1000 = frequency; issue #149's "5170-5250/5" example:
   * ch34..ch50, one bin per 5 MHz grid step). */
  expect("5170-5190/5", {34, 35, 36, 37, 38});
  expect("2412-2462/25", {1, 6, 11});
  expect("2484-2484", {14});
  expect("5180-5200/20", {36, 40});

  /* Mixed lists + junk tolerance. */
  expect("1,36-40/4,5180-5180", {1, 36, 40, 36});
  expect("0,-3,abc", {});
  expect("2400-2410/5", {});  /* below the 2.4 GHz channel grid -> dropped */
  expect("5170-5250/0", {34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46,
                         47, 48, 49, 50}); /* MHz step clamps to the 5 MHz grid */
  expect("5170-5180/1", {34, 35, 36});     /* sub-grid MHz step -> 5 */

  if (g_fail) {
    std::printf("sweep_spec_selftest: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("sweep_spec_selftest: all checks passed\n");
  return 0;
}
