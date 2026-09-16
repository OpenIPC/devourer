/* Headless guard for the MT7612U coherent TSF read
 * (src/mt7612u/Mt7612uTsfRead.h).
 *
 * The two TSF halves are not latched, so the read order is the whole
 * correctness argument - and on a bench it only matters once every 71.6 min,
 * for the few hundred microseconds around a low-word wrap. A broken order looks
 * perfect for an hour. This cell scripts the register sequence instead: a
 * simulated counter that advances a fixed step per register access, started
 * just below the wrap at every phase, so each read-to-read gap gets the wrap
 * in turn.
 *
 * Negative control: the order the library used before this cell existed (DW0
 * then DW1, no retry) runs against the same scripts and MUST tear, or the rig
 * cannot see the bug it exists for.
 *
 * What this does NOT cover: that the hardware behaves like the model (no latch
 * between the halves). That is a forced-straddle soak on two units, recorded
 * in docs/mt7612u.md. */
#include "mt7612u/Mt7612uTsfRead.h"

#include <cstdint>
#include <cstdio>

namespace {

int fails;

void expect(const char *what, bool ok) {
  if (!ok) {
    std::fprintf(stderr, "mt7612u_tsf_read: FAIL %s\n", what);
    fails++;
  }
}

/* A counter that advances `step` µs on every register access, reporting the
 * half the address names. `fail_at` makes that access (0-based) fail. */
struct Sim {
  uint64_t now;
  uint64_t step;
  int accesses = 0;
  int fail_at = -1;

  int operator()(uint32_t addr, uint32_t *v) {
    const int n = accesses++;
    now += step;
    if (n == fail_at)
      return -1;
    *v = addr == MT_TSF_TIMER_DW1 ? static_cast<uint32_t>(now >> 32)
                                  : static_cast<uint32_t>(now);
    return 0;
  }
};

/* The pre-fix reader, kept only as the negative control. */
template <typename Rd> uint64_t old_read(Rd &&rd) {
  uint32_t lo = 0, hi = 0;
  rd(MT_TSF_TIMER_DW0, &lo);
  rd(MT_TSF_TIMER_DW1, &hi);
  return (static_cast<uint64_t>(hi) << 32) | lo;
}

/* A coherent value lies between the counter before the first access and after
 * the last one. */
bool coherent(uint64_t v, uint64_t before, uint64_t after) {
  return v >= before && v <= after;
}

} // namespace

int main() {
  const uint64_t step = 300; /* ~one control transfer */
  const uint64_t wrap = 1ull << 32;
  int retries = 0, old_tears = 0;

  /* Sweep the start so the wrap falls before, inside and after every gap of
   * the read, one microsecond at a time across five steps. */
  for (uint64_t off = 0; off <= 5 * step; off++) {
    const uint64_t start = wrap - 2 * step + off - 150;

    Sim s{start, step};
    uint64_t v = 0;
    bool retried = false;
    expect("read succeeds", mt7612u::tsf_read(s, &v, &retried) == 0);
    if (!coherent(v, start, s.now)) {
      std::fprintf(stderr, "  off=%llu value=%llu window=[%llu,%llu]\n",
                   (unsigned long long)off, (unsigned long long)v,
                   (unsigned long long)start, (unsigned long long)s.now);
      expect("value is coherent across the wrap", false);
    }
    expect("3 accesses, or 4 with the retry",
           s.accesses == (retried ? 4 : 3));
    retries += retried;

    Sim o{start, step};
    uint64_t ov = old_read(o);
    old_tears += !coherent(ov, start, o.now);
  }
  expect("the sweep exercised the retry path", retries > 0);
  expect("negative control: the old DW0,DW1 order tears in the same sweep",
         old_tears > 0);

  /* Far from a wrap: no retry, exact join. */
  {
    Sim s{0x12345678'9abcdef0ull, 0};
    uint64_t v = 0;
    bool retried = true;
    expect("plain read", mt7612u::tsf_read(s, &v, &retried) == 0);
    expect("plain read joins high:low", v == 0x12345678'9abcdef0ull);
    expect("plain read does not retry", !retried && s.accesses == 3);
  }

  /* 0xffffffff is a word, not a failure: a low word of all-ones reads back. */
  {
    Sim s{0x00000002'ffffffffull, 0};
    uint64_t v = 0;
    expect("all-ones low word is a value",
           mt7612u::tsf_read(s, &v) == 0 && v == 0x00000002'ffffffffull);
  }

  /* A failure at any access fails the read and leaves *out untouched. Four
   * positions, with the start placed so the retry (the fourth access) runs. */
  for (int at = 0; at < 4; at++) {
    Sim s{wrap - 400, 300};
    s.fail_at = at;
    uint64_t v = 0xdeadbeefull;
    bool retried = false;
    char what[64];
    std::snprintf(what, sizeof what, "failure at access %d fails the read", at);
    expect(what, mt7612u::tsf_read(s, &v, &retried) == -1);
    std::snprintf(what, sizeof what, "failure at access %d leaves *out", at);
    expect(what, v == 0xdeadbeefull);
  }
  /* ...and the retry position really is reached in that script. */
  {
    Sim s{wrap - 400, 300};
    uint64_t v = 0;
    bool retried = false;
    expect("the failure script reaches the retry",
           mt7612u::tsf_read(s, &v, &retried) == 0 && retried && s.accesses == 4);
  }

  if (fails == 0)
    std::printf("mt7612u_tsf_read: PASS (%d retries, old order tore %d times)\n",
                retries, old_tears);
  return fails == 0 ? 0 : 1;
}
