/* MigClock — the ground side's estimate of the drone's activation instant,
 * and the guard time around it.
 *
 * The drone stamps its hardware TSF (tx_tsf) into every commit/status; the
 * ground feeds (tx_tsf, local RX tsfl) pairs into the existing TsfSync fit.
 * Because tx_tsf is software-stamped (not a beacon's hardware egress), the fit
 * residual carries the host->air latency — floor tens of µs but a CSMA tail in
 * the low ms. PeerClock wraps TsfSync with a ring of recent residuals so the
 * guard is derived from the MEASURED p99, never a fixed guess.
 *
 * Scheduling itself does not need the fit: the machines schedule activation
 * from the RELATIVE offset (activate_tsf - commit.tx_tsf), whose error is one
 * frame's one-way latency, absorbed by the lead + verify window. The fit only
 * tightens the guard and is the health readout. Pure, header-only. */
#ifndef DEVOURER_CHANMIG_MIG_CLOCK_H
#define DEVOURER_CHANMIG_MIG_CLOCK_H

#include <algorithm>
#include <cstdint>
#include <vector>

#include "TsfSync.h"

namespace devourer {
namespace chanmig {

class PeerClock {
public:
  /* Feed one (drone tx_tsf, local RX tsfl) pair. */
  void add(uint64_t remote_tsf, uint32_t local_tsfl) {
    if (sync_.Ready()) {
      const int64_t predicted = sync_.LocalForRemote(remote_tsf);
      /* the reconstructed local time of this sample */
      const int64_t observed = recon_local(local_tsfl);
      int64_t resid = observed - predicted;
      if (resid < 0)
        resid = -resid;
      if (ring_.size() >= kRing)
        ring_.erase(ring_.begin());
      ring_.push_back(resid);
    } else {
      (void)recon_local(local_tsfl); /* keep the local wrap tracker warm */
    }
    sync_.Add(remote_tsf, local_tsfl);
  }

  bool ready() const { return sync_.Ready() && ring_.size() >= 8; }
  double skew_ppm() const { return sync_.SkewPpm(); }
  int64_t offset_us() const { return sync_.OffsetUs(); }
  long long samples() const { return sync_.Count(); }

  int64_t residual_p50() const { return percentile(0.50); }
  int64_t residual_p99() const { return percentile(0.99); }

  /* The activation guard (µs): the measured fit residual p99, plus the TX
   * drain p99.9 and the worst retune the demo passes in, plus a margin. All
   * measured, none guessed. Falls back to a floor when the fit isn't ready. */
  int64_t guard_us(int64_t drain_p999_us, int64_t retune_worst_us,
                   int64_t margin_us = 2000) const {
    const int64_t resid = ready() ? residual_p99() : 3000;
    return resid + drain_p999_us + retune_worst_us + margin_us;
  }

private:
  static constexpr size_t kRing = 64;
  int64_t recon_local(uint32_t lo) {
    if (linit_ && lo < plo_)
      hi_ += (1LL << 32);
    plo_ = lo;
    linit_ = true;
    return hi_ + lo;
  }
  int64_t percentile(double q) const {
    if (ring_.empty())
      return 0;
    std::vector<int64_t> v = ring_;
    std::sort(v.begin(), v.end());
    size_t idx = static_cast<size_t>(q * (v.size() - 1));
    return v[idx];
  }

  TsfSync sync_;
  std::vector<int64_t> ring_;
  int64_t hi_ = 0;
  uint32_t plo_ = 0;
  bool linit_ = false;
};

} /* namespace chanmig */
} /* namespace devourer */

#endif /* DEVOURER_CHANMIG_MIG_CLOCK_H */
