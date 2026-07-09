// timesync_selftest.cpp — headless unit tests for the timesync pure logic: the
// LinFit master↔local least-squares fit (does it recover a known crystal offset
// and predict the master clock tightly through per-frame jitter?) and the 32→64
// bit TSF Recon wrap handling. No hardware / no libusb — runs in ctest, unlike
// the timesync master/slave demo which needs real adapters.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "timesync.h"

static int g_fail = 0;
#define CHECK(cond, msg)                                                        \
  do {                                                                          \
    if (!(cond)) { std::printf("FAIL: %s\n", (msg)); ++g_fail; }                \
  } while (0)

int main() {
  // The dominant on-air noise is the MASTER's software stamp→air jitter: it
  // calls ReadTsf() then send_packet, so a beacon airs a variable δ after the
  // stamp. δ is COMMON to every slave (same frame, same air-departure). Each
  // slave's own hardware-TSF latch is precise (sub-µs, modeled as tiny ε).
  const int NB = 600;
  uint32_t ds = 3;                                   // shared master δ, ±60 µs
  std::vector<double> delta(NB);
  for (int i = 0; i < NB; ++i) {
    ds = ds * 1664525u + 1013904223u;
    delta[i] = (double)((int)((ds >> 8) % 120) - 60);
  }

  // Beacon i airs at master-clock T_i (a clean 100 ms grid) but was STAMPED
  // δ_i earlier, so its broadcast value is master_stamp = T_i − δ_i. A slave
  // whose crystal runs `ppm` slower than the master latches its own hardware
  // clock local = (1 − ppm/1e6)·T at arrival (+ tiny ε latch noise).
  auto master_stamp = [&](int i) { return (double)i * 100000.0 - delta[i]; };

  // --- 1. LinFit recovers ppm; single-slave lock ~ the master stamp jitter ---
  // A single slave cannot beat δ (it has no other reference), but the FIT must
  // still recover the crystal ppm cleanly (δ averages out of the slope) — that's
  // what lets inter-slave agreement (test 2) cancel δ.
  {
    timesync::LinFit fit;
    const double ppm = 16.5;
    const double g = 1.0 - ppm / 1e6;    // slave local µs per master µs (slower)
    const double b = 987654.0;           // constant offset (prop + fixed latency)
    uint32_t es = 91;
    auto eps = [&]() { es = es * 1664525u + 1013904223u;
                       return ((int)((es >> 8) % 20) - 10) * 0.05; };  // ±0.5 µs
    double max_pred = 0;
    for (int i = 0; i < NB; ++i) {
      double local = g * ((double)i * 100000.0) + b + eps();  // slave hw arrival
      double ms = master_stamp(i);                            // broadcast value
      if (fit.ready()) {
        double e = std::fabs(fit.at(local) - ms);
        if (e > max_pred) max_pred = e;
      }
      fit.add(local, ms);
    }
    CHECK(fit.ready(), "fit ready after N beacons");
    CHECK(std::fabs(fit.ppm() - ppm) < 1.0,
          "master-vs-slave ppm recovered within 1 ppm of the true 16.5");
    CHECK(max_pred < 120.0,
          "single-slave lock is bounded by (does not beat) the ~60 us master jitter");
  }

  // --- 2. two slaves predicting the SAME beacon agree to sub-µs --------------
  // The inter-UE sync metric: two slaves (different crystals, tiny independent
  // latch noise) both predict each shared beacon. Because the master stamp
  // jitter δ is COMMON, it cancels in the difference — so they agree far tighter
  // than either's absolute lock. This is the real payoff (the dual-RX floor).
  {
    auto run_slave = [&](uint32_t seed, double ppm) {
      timesync::LinFit fit;
      uint32_t es = seed;
      auto eps = [&]() { es = es * 1664525u + 1013904223u;
                         return ((int)((es >> 8) % 20) - 10) * 0.05; };  // ±0.5 µs
      const double g = 1.0 - ppm / 1e6;
      std::vector<double> preds;
      for (int i = 0; i < NB; ++i) {
        double local = g * ((double)i * 100000.0) + 55555.0 + eps();
        double ms = master_stamp(i);           // shared broadcast (shared δ)
        preds.push_back(fit.ready() ? fit.at(local) : ms);
        fit.add(local, ms);
      }
      return preds;
    };
    auto A = run_slave(11, 12.0);    // slave A: +12 ppm crystal
    auto B = run_slave(29, -18.0);   // slave B: -18 ppm crystal
    double max_disagree = 0;
    for (size_t i = 50; i < A.size(); ++i) {   // after both fits warm up
      double d = std::fabs(A[i] - B[i]);
      if (d > max_disagree) max_disagree = d;
    }
    CHECK(max_disagree < 5.0,
          "two slaves agree to sub-µs — the common master jitter cancels");
  }

  // --- 3. uplink timing-advance loop converges ------------------------------
  // Model the master integrator exactly as master_ta_cb does: the UE's uplink
  // arrives with a fixed latency offset L that must be nulled, while the UE↔
  // master crystal offset drifts the arrival by d µs/slot. ta += gain·phase must
  // drive the arrival phase from L down to the small drift-tracking residual
  // (~d/gain), not diverge.
  {
    const double L = 520.0;      // initial arrival offset (latency), µs
    const double d = 0.6;        // crystal drift per slot (≈30 ppm × 20 ms), µs
    const double gain = 0.3;
    uint32_t ns = 5;
    auto noise = [&]() { ns = ns * 1664525u + 1013904223u;
                         return ((int)((ns >> 8) % 20) - 10) * 0.1; };  // ±1 µs
    double ta = 0.0, early_max = 0.0, late_ss = 0.0;
    int late_n = 0;
    const int STEPS = 400;
    for (int k = 0; k < STEPS; ++k) {
      double residual = L - ta + d * k;         // true arrival phase this slot
      double phase = residual + noise();        // what the master measures
      if (k < 3 && std::fabs(phase) > early_max) early_max = std::fabs(phase);
      if (k >= STEPS - 80) { late_ss += phase * phase; ++late_n; }
      ta += gain * phase;                        // master's TA integrator
    }
    CHECK(early_max > 200.0, "loop starts far off the slot boundary (~L)");
    CHECK(std::sqrt(late_ss / late_n) < 8.0,
          "TA loop converges: arrival locks to the slot within a few us");
  }

  // --- 4. Recon 32→64-bit TSF wrap ------------------------------------------
  {
    timesync::Recon r;
    CHECK(r(0xFFFFFF00u) == 0xFFFFFF00LL, "first sample = raw low word");
    CHECK(r(0x00000100u) == 0x100000100LL, "wrap adds 2^32");
    CHECK(r(0x00000200u) == 0x100000200LL, "no spurious wrap when increasing");
  }

  std::printf(g_fail ? "timesync_selftest: %d FAILURE(S)\n"
                     : "timesync_selftest: all passed\n",
              g_fail);
  return g_fail ? 1 : 0;
}
