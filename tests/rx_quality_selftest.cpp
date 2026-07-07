/* Headless guard for the RxQuality accumulator + build_rx_quality fuse
 * (src/RxQuality.h) — the windowed RX-link-quality feed behind
 * rx.quality event / IRtlDevice::GetRxQuality(). Verifies the aggregate
 * math (mean/max/min, EVM present-gate), the PASSIVE noise-floor formula
 * (nf_dbm = (rssi_raw-110) - snr_raw/2), the unit conversions, and that the
 * fused verdict matches classify_link_health. A regression fails ctest instead
 * of silently feeding a controller wrong numbers. */
#include <cmath>
#include <cstdio>

#include "RxQuality.h"

static int g_fail = 0;

static void check(const char *name, bool ok) {
  if (!ok) {
    ++g_fail;
    std::printf("FAIL: %s\n", name);
  }
}

static bool approx(double a, double b) { return std::fabs(a - b) < 1e-6; }

int main() {
  using namespace devourer;

  /* Empty window -> invalid. */
  {
    RxQualityAccumulator acc;
    RxQuality q = build_rx_quality(acc.snapshot(), RxEnergy{});
    check("empty.invalid", !q.valid && q.frames == 0);
    check("empty.nf_invalid", !q.nf_valid);
    check("empty.verdict", q.verdict == LinkVerdict::NoSignal);
  }

  /* Aggregate math + noise floor. Three OFDM frames:
   *   rssi_raw = 60, 70, 80  (mean 70, max 80)
   *   snr_raw  = 40, 30, 20  (mean 30 -> 15 dB, min 20 -> 10 dB)
   *   evm_raw  = -50, -40, 0 (present: -50,-40 -> mean -45 -> -22.5 dB)
   * NF per frame (snr!=0): (60-110)-20 = -70 ; (70-110)-15 = -55 ;
   *                        (80-110)-10 = -40  -> mean -55 dBm. */
  {
    RxQualityAccumulator acc;
    acc.add(60, 40, -50);
    acc.add(70, 30, -40);
    acc.add(80, 20, 0);
    RxQualitySnapshot s = acc.snapshot();
    check("agg.frames", s.frames == 3);
    check("agg.rssi_mean_raw", s.rssi_mean_raw == 70);
    check("agg.rssi_max_raw", s.rssi_max_raw == 80);
    check("agg.snr_mean_raw", s.snr_mean_raw == 30);
    check("agg.snr_min_raw", s.snr_min_raw == 20);
    check("agg.evm_valid", s.evm_valid);
    check("agg.evm_mean_raw", s.evm_mean_raw == -45);
    check("agg.nf_valid", s.nf_valid);
    check("agg.nf_mean", approx(s.nf_mean_dbm, -55.0));

    RxQuality q = build_rx_quality(s, RxEnergy{});
    check("q.valid", q.valid);
    check("q.rssi_mean_dbm", q.rssi_mean_dbm == 70 - 110);
    check("q.rssi_max_dbm", q.rssi_max_dbm == 80 - 110);
    check("q.snr_mean_db", approx(q.snr_mean_db, 15.0));
    check("q.snr_min_db", approx(q.snr_min_db, 10.0));
    check("q.evm_mean_db", approx(q.evm_mean_db, -22.5));
    check("q.noise_floor", approx(q.noise_floor_dbm, -55.0));
  }

  /* snapshot() resets (delta semantics). */
  {
    RxQualityAccumulator acc;
    acc.add(70, 30, -50);
    (void)acc.snapshot();
    RxQualitySnapshot s2 = acc.snapshot();
    check("reset.empty", s2.frames == 0 && !s2.nf_valid);
  }

  /* Frames with no phy-status (rssi_raw <= 0) are not quality samples. */
  {
    RxQualityAccumulator acc;
    acc.add(0, 0, 0);
    acc.add(0, 0, 0);
    check("nophy.skipped", acc.snapshot().frames == 0);
  }

  /* CCK-only frames (snr/evm 0) count for rssi but leave NF/EVM invalid. */
  {
    RxQualityAccumulator acc;
    acc.add(50, 0, 0);
    acc.add(55, 0, 0);
    RxQualitySnapshot s = acc.snapshot();
    check("cck.frames", s.frames == 2);
    check("cck.rssi", s.rssi_mean_raw == 52); /* 105/2 truncated */
    check("cck.nf_invalid", !s.nf_valid);
    check("cck.evm_invalid", !s.evm_valid);
  }

  /* Self-jamming signature: strong RSSI + dirty EVM -> Saturated, and the NF
   * estimate is high (near the signal) because SNR collapsed. Mirrors the
   * saturation-knee measurement. */
  {
    RxQualityAccumulator acc;
    /* rssi 72 (raw), snr 36 (18 dB), evm collapsed to -27 (raw). */
    for (int i = 0; i < 4; ++i)
      acc.add(72, 36, -27);
    RxQuality q = build_rx_quality(acc.snapshot(), RxEnergy{});
    check("sat.verdict", q.verdict == LinkVerdict::Saturated);
    check("sat.nf_valid", q.nf_valid);
  }

  if (g_fail == 0)
    std::printf("rx_quality_selftest: all passed\n");
  return g_fail ? 1 : 0;
}
