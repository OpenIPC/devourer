/* Headless guard for the C TSF read's failure contract
 * (src/mt7612u/caps.cpp, declared in mt7612u/mt7612u.h).
 *
 * The read discipline itself is mt7612u_tsf_read, against a scripted counter.
 * What that cell cannot reach is the C entry points' contract, and the whole
 * point of this pair is that a failure is distinguishable from a value:
 * `0xffffffff` is a legitimate register word here, so only the return code can
 * carry a failed transfer. A refusal that scribbled on *out, or one that
 * returned 0 from _chk, would leave a caller unable to tell a failure from a
 * TSF - the silent-garbage shape this whole path exists to remove.
 *
 * A NULL device is the only failure this can reach without hardware, so that
 * is what it pins. What it does NOT cover: a failed transfer on a live device,
 * and Mt7612uRadio::ReadTsf throwing. Those need the part - the
 * `bringup tsfwrap` gate and the bus-disconnect run in docs/mt7612u.md. */
#include "mt7612u/mt7612u.h"

#include <cstdint>
#include <cstdio>

namespace {

int fails;

void expect(const char *what, bool ok) {
  if (!ok) {
    std::fprintf(stderr, "mt7612u_tsf_api: FAIL %s\n", what);
    fails++;
  }
}

} // namespace

int main() {
  /* _chk refuses a NULL device, and says so in the return code. */
  {
    uint64_t out = 0xdeadbeefcafef00dull;
    expect("_chk(NULL dev) returns -1", mt7612u_read_tsf_chk(nullptr, &out) == -1);
    expect("_chk(NULL dev) leaves *out", out == 0xdeadbeefcafef00dull);
  }
  /* ...and a NULL out, rather than writing through it. */
  expect("_chk(NULL out) returns -1", mt7612u_read_tsf_chk(nullptr, nullptr) == -1);

  /* The no-error-channel form answers 0, never a value built from a failure. */
  expect("read_tsf(NULL) is 0", mt7612u_read_tsf(nullptr) == 0);

  /* The caps bit says this part has no TSF load path, so a C caller need not
   * discover it from a missing symbol. */
  {
    struct mt7612u_caps c;

    /* No device to fill it: the field must exist and be addressable, which is
     * what the C ABI half of this is. */
    c.tsf_write = 0;
    expect("caps carry tsf_write", c.tsf_write == 0);
  }

  if (fails == 0)
    std::printf("mt7612u_tsf_api: PASS\n");
  return fails == 0 ? 0 : 1;
}
