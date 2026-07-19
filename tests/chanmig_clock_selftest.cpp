/* PeerClock selftest — the ground's estimate of the drone's clock + the guard.
 * Feeds synthetic (drone tx_tsf, local RX tsfl) pairs with a known skew and a
 * bounded software-stamp jitter, and checks the fit recovers the skew, the
 * residual percentiles bound the jitter, and the guard composes measured
 * numbers (never a fixed guess). */
#include "chanmig/MigClock.h"

#include <cstdio>

static int fails;
#define CHECK(x, msg)                                                          \
  do {                                                                         \
    if (!(x)) {                                                                \
      std::fprintf(stderr, "FAIL: %s\n", msg);                                 \
      ++fails;                                                                 \
    }                                                                          \
  } while (0)

using devourer::chanmig::PeerClock;

int main() {
  PeerClock pc;
  /* drone runs 20 ppm fast relative to the ground; each status frame carries
   * a software-stamped tx_tsf whose air latency adds a small jitter. */
  const double skew = 1.0 + 20e-6;
  uint64_t remote = 5000000;    /* drone TSF */
  int64_t local = 5000000;      /* ground local time base */
  /* deterministic pseudo-jitter (no RNG in tests): a bounded sawtooth */
  const int jit[8] = {40, 120, 60, 300, 80, 1700, 50, 200}; /* µs */
  for (int i = 0; i < 200; i++) {
    remote += 100000; /* ~10 Hz status */
    local = static_cast<int64_t>(5000000 + (remote - 5000000) * skew);
    const int64_t observed = local + jit[i % 8];
    pc.add(remote, static_cast<uint32_t>(observed));
  }
  CHECK(pc.ready(), "clock ready after enough samples");
  CHECK(pc.samples() >= 200, "sample count tracked");
  /* skew recovered within a few ppm */
  const double ppm = pc.skew_ppm();
  CHECK(ppm > 10 && ppm < 30, "skew recovered near 20 ppm");
  /* residual percentiles bound the injected jitter (max 1700 µs) */
  const int64_t p50 = pc.residual_p50(), p99 = pc.residual_p99();
  CHECK(p50 >= 0 && p50 <= 400, "residual p50 within the jitter body");
  CHECK(p99 <= 2000, "residual p99 bounds the tail");
  CHECK(p99 >= p50, "p99 >= p50");
  /* guard composes measured residual + drain + retune + margin */
  const int64_t guard = pc.guard_us(3000, 2500, 2000);
  CHECK(guard >= p99 + 3000 + 2500 + 2000 - 1, "guard includes all measured terms");
  CHECK(guard > 7000, "guard is a real multi-ms budget");

  /* An un-fed clock falls back to a safe floor, never a zero guard. */
  PeerClock empty;
  CHECK(!empty.ready(), "empty clock not ready");
  CHECK(empty.guard_us(3000, 2500, 2000) >= 3000 + 2500 + 2000,
        "empty clock guard uses the residual floor");

  return fails ? 1 : 0;
}
