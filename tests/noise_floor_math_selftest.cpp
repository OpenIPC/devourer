/* Headless guard for the active idle-noise-floor math helpers
 * (src/NoiseFloorMath.h): the 10-bit sign conversion of the 0x0FA0 debug-port
 * I/Q and the 3*log2 pwdb approximation. Pure math; no hardware. */
#include "NoiseFloorMath.h"

#include <cstdio>

using devourer::nf::pwdb_conversion;
using devourer::nf::sign_conversion;

static int g_fail = 0;

static void check(const char *what, long got, long want) {
  if (got != want) {
    std::printf("FAIL %s: got %ld want %ld\n", what, got, want);
    ++g_fail;
  }
}

int main() {
  /* 10-bit two's-complement (rxi/rxq packing). */
  check("sign 0x000", sign_conversion(0x000, 10), 0);
  check("sign 0x1ff", sign_conversion(0x1ff, 10), 511);   /* largest positive */
  check("sign 0x200", sign_conversion(0x200, 10), -512);  /* sign bit only */
  check("sign 0x3ff", sign_conversion(0x3ff, 10), -1);    /* all ones = -1 */

  /* pwdb = 3*log2(X) - 3*decimal_bit + coarse-decimal, called as (X,20,18). */
  check("pwdb 0",       pwdb_conversion(0, 20, 18), -54);       /* X==0 -> 1 */
  check("pwdb 1",       pwdb_conversion(1, 20, 18), -54);       /* only bit0 */
  check("pwdb 4",       pwdb_conversion(4, 20, 18), -48);       /* bit2, no dec */
  check("pwdb 6",       pwdb_conversion(6, 20, 18), -46);       /* bit2 + bit1 dec */
  check("pwdb 1<<18",   pwdb_conversion(1 << 18, 20, 18), 0);   /* full scale */
  check("pwdb 1<<19",   pwdb_conversion(1 << 19, 20, 18), 3);
  /* A representative accepted idle sample: I=Q=20 -> val=800, leading bit 9. */
  check("pwdb 800",     pwdb_conversion(800, 20, 18), -25);     /* in [-27,0) */

  if (g_fail) {
    std::printf("noise_floor_math: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("noise_floor_math: all checks passed\n");
  return 0;
}
