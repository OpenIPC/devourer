/* Guard for the IGI write a changed floor implies (src/jaguar3/IgiFloor.h). */
#include "jaguar3/IgiFloor.h"

#include <cstdio>

using jaguar3::igi_on_floor_change;

static int g_fail = 0;

// (prev_floor, floor, cur_igi, window_hi) -> IGI to write, -1 = none
static void check(const char *what, int got, int want) {
  if (got != want) {
    std::printf("FAIL %s: got %d want %d\n", what, got, want);
    ++g_fail;
  }
}

int main() {
  check("set: raise to it", igi_on_floor_change(0x00, 0x22, 0x1e, 0x22), 0x22);
  check("raised: follow", igi_on_floor_change(0x22, 0x24, 0x22, 0x22), 0x24);
  check("already above", igi_on_floor_change(0x22, 0x24, 0x30, 0x22), -1);
  check("lowered: park at once", igi_on_floor_change(0x35, 0x22, 0x35, 0x22), 0x22);
  check("lowered: floor above window top",
        igi_on_floor_change(0x35, 0x24, 0x35, 0x22), 0x24);
  check("lowered, DFS window", igi_on_floor_change(0x35, 0x24, 0x35, 0x20), 0x24);
  check("dropped: back into window", igi_on_floor_change(0x22, 0x00, 0x22, 0x20), 0x20);
  check("dropped, already inside", igi_on_floor_change(0x22, 0x00, 0x1e, 0x22), -1);
  check("no floor before or after", igi_on_floor_change(0x00, 0x00, 0x30, 0x22), -1);
  check("unchanged", igi_on_floor_change(0x22, 0x22, 0x22, 0x22), -1);
  if (g_fail == 0) std::printf("igi_floor: all ok\n");
  return g_fail ? 1 : 0;
}
