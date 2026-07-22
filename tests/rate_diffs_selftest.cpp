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
  if (fails) { std::fprintf(stderr, "%d failure(s)\n", fails); return 1; }
  std::puts("rate_diffs_selftest OK");
  return 0;
}
