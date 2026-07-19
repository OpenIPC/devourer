/* ScanScheduler — the pure dwell scheduler behind the chanscout demo.
 *
 * The scout surveys CANDIDATES but tunes 20 MHz BINS (a 40/80 MHz candidate
 * is its constituent bins; two candidates sharing a bin share its evidence).
 * Each bin carries a revisit deadline: bins belonging to a preferred-backup
 * candidate get the fast cadence, everything else the background cadence, and
 * next() always returns the most-overdue bin — so the scout scans at 100%
 * duty while backup bins are simply revisited more often. Deterministic:
 * deadline order, ties broken by plan position; no clock reads (the caller
 * passes now), no RNG.
 *
 * The round counter advances only when EVERY bin has been successfully
 * observed since the last advance. That is the structural anti-positive-
 * feedback guard: backup favoritism can never starve the full rescan a
 * scoring policy's min-rounds gate keys on.
 *
 * Wide candidates optionally get a periodic full-width verification dwell
 * (the demo pays a full SetMonitorChannel for it); those ride their own
 * cadence and never count toward bin coverage. */
#ifndef DEVOURER_CHANMIG_SCAN_PLAN_H
#define DEVOURER_CHANMIG_SCAN_PLAN_H

#include <cstdint>
#include <vector>

#include "chanmig/ChannelDef.h"

namespace devourer {
namespace chanmig {

struct ScanPlanConfig {
  std::vector<ChannelDef> candidates; /* immutable for the process lifetime */
  int dwell_ms = 100;
  int settle_ms = 30;
  int backup_revisit_ms = 1000; /* bins of backup candidates */
  int bg_revisit_ms = 5000;     /* everything else */
  int fail_retry_ms = 250;      /* deadline pushback after a failed dwell */
  int fullwidth_ms = 0;         /* 0 = no wide verification dwells */
  int64_t max_age_ms = 60000;   /* evidence-freshness bound (consumers) */

  /* Stable identity of the whole plan (candidates + cadences). Emitted in
   * scout.plan and stamped into every record so a consumer can refuse to fold
   * records produced under a different plan. FNV-1a over the canonical
   * candidate formats and the cadence fields. */
  uint32_t plan_hash() const {
    uint32_t h = 2166136261u;
    auto fold = [&h](const void *p, size_t n) {
      const uint8_t *b = static_cast<const uint8_t *>(p);
      for (size_t i = 0; i < n; i++) {
        h ^= b[i];
        h *= 16777619u;
      }
    };
    for (const ChannelDef &c : candidates) {
      const uint32_t k = c.key();
      const uint8_t fl = static_cast<uint8_t>((c.backup ? 1 : 0) |
                                              (c.no_ir ? 2 : 0) |
                                              (c.dfs ? 4 : 0));
      fold(&k, sizeof(k));
      fold(&fl, 1);
    }
    const int32_t cad[4] = {dwell_ms, settle_ms, backup_revisit_ms,
                            bg_revisit_ms};
    fold(cad, sizeof(cad));
    return h;
  }
};

class ScanScheduler {
public:
  struct DwellPlan {
    bool valid = false;
    uint8_t bin_ch = 0;
    ChannelDef def;    /* what to tune: 20 MHz bin def, or the wide candidate
                          when full_width */
    uint64_t round = 0;
    bool full_width = false;
    int plan_index = -1; /* internal: bin slot / wide-candidate slot */
  };

  explicit ScanScheduler(const ScanPlanConfig &cfg) : cfg_(cfg) {
    for (size_t ci = 0; ci < cfg_.candidates.size(); ++ci) {
      const ChannelDef &c = cfg_.candidates[ci];
      uint8_t bins[4];
      const int n = constituent_bins(c, bins);
      const int cadence =
          c.backup ? cfg_.backup_revisit_ms : cfg_.bg_revisit_ms;
      for (int i = 0; i < n; i++) {
        Bin *b = find_bin(bins[i]);
        if (b == nullptr) {
          bins_.push_back(Bin{bins[i], c.band, cadence, INT64_MIN, false, 0});
        } else if (cadence < b->cadence_ms) {
          b->cadence_ms = cadence; /* fastest containing candidate wins */
        }
      }
      if (c.width == CHANNEL_WIDTH_40 || c.width == CHANNEL_WIDTH_80)
        wide_.push_back(static_cast<int>(ci));
    }
  }

  size_t bin_count() const { return bins_.size(); }
  uint64_t rounds_complete() const { return rounds_; }
  int consecutive_failures() const { return consec_fail_; }

  /* Pick the next dwell: the periodic wide verification dwell when due,
   * otherwise the most-overdue bin (ties by plan position — which also makes
   * the very first sweep visit every bin exactly once in plan order, since
   * all deadlines start equal). */
  DwellPlan next(int64_t now_ms) {
    DwellPlan p;
    if (bins_.empty())
      return p;
    /* Arm the wide-dwell cadence off the first call, so the initial full
     * sweep is always plain bins. */
    if (next_wide_ms_ == INT64_MIN)
      next_wide_ms_ = now_ms + cfg_.fullwidth_ms;
    if (cfg_.fullwidth_ms > 0 && !wide_.empty() && now_ms >= next_wide_ms_) {
      const int ci = wide_[wide_rr_ % wide_.size()];
      p.valid = true;
      p.full_width = true;
      p.def = cfg_.candidates[static_cast<size_t>(ci)];
      p.bin_ch = p.def.primary;
      p.round = rounds_;
      p.plan_index = ci;
      return p;
    }
    int best = 0;
    for (size_t i = 1; i < bins_.size(); ++i)
      if (bins_[i].deadline_ms < bins_[best].deadline_ms)
        best = static_cast<int>(i);
    const Bin &b = bins_[static_cast<size_t>(best)];
    p.valid = true;
    p.bin_ch = b.ch;
    p.def.band = b.band;
    p.def.primary = b.ch;
    p.def.width = CHANNEL_WIDTH_20;
    p.round = rounds_;
    p.plan_index = best;
    return p;
  }

  void complete(const DwellPlan &p, int64_t now_ms, bool ok) {
    if (!p.valid)
      return;
    if (p.full_width) {
      wide_rr_++;
      next_wide_ms_ = now_ms + cfg_.fullwidth_ms;
      /* wide dwells never touch bin coverage or the failure streak */
      return;
    }
    if (p.plan_index < 0 || p.plan_index >= static_cast<int>(bins_.size()))
      return;
    Bin &b = bins_[static_cast<size_t>(p.plan_index)];
    if (ok) {
      b.deadline_ms = now_ms + b.cadence_ms;
      b.fails = 0;
      consec_fail_ = 0;
      if (!b.covered) {
        b.covered = true;
        if (++covered_n_ == bins_.size()) {
          ++rounds_;
          covered_n_ = 0;
          for (Bin &x : bins_)
            x.covered = false;
        }
      }
    } else {
      b.deadline_ms = now_ms + cfg_.fail_retry_ms;
      ++b.fails;
      ++consec_fail_;
    }
  }

private:
  struct Bin {
    uint8_t ch;
    uint8_t band;
    int cadence_ms;
    int64_t deadline_ms;
    bool covered;
    int fails;
  };
  Bin *find_bin(uint8_t ch) {
    for (Bin &b : bins_)
      if (b.ch == ch)
        return &b;
    return nullptr;
  }

  ScanPlanConfig cfg_;
  std::vector<Bin> bins_;
  std::vector<int> wide_;
  size_t covered_n_ = 0;
  uint64_t rounds_ = 0;
  size_t wide_rr_ = 0;
  int64_t next_wide_ms_ = INT64_MIN;
  int consec_fail_ = 0;
};

} /* namespace chanmig */
} /* namespace devourer */

#endif /* DEVOURER_CHANMIG_SCAN_PLAN_H */
