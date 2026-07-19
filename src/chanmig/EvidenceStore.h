/* EvidenceStore — the rolling per-candidate evidence the scoring engine
 * decides from, fed by SurveyDwell records (live from the dwell executor, or
 * replayed from JSONL).
 *
 * Structure follows the physics: evidence is stored per 20 MHz BIN (a dwell
 * observes one bin; two candidates sharing a bin share its evidence), and a
 * candidate's occupancy derives from its constituent bins at decision time
 * with worst-bin gating — a wide channel is only clean if every bin is clean.
 *
 * The fold path is the trust boundary. A record is folded only when it
 * carries the same plan hash (same immutable candidate set), the same scout
 * identity (raw energy units are only comparable within one adapter — a
 * record from a different scout resets the store rather than silently mixing
 * calibration domains), no failure flags, a plausible counter delta for its
 * observation window (a hardware counter wrap or mid-read reset shows up as
 * an absurd rate — better to drop one dwell than to poison a ranking), and
 * an age inside the freshness bound. Every rejection is counted by reason so
 * a health layer can tell "quiet" from "being fed garbage".
 *
 * Pure: no clocks (caller passes now), no I/O, no env. Selftested. */
#ifndef DEVOURER_CHANMIG_EVIDENCE_STORE_H
#define DEVOURER_CHANMIG_EVIDENCE_STORE_H

#include <cstdint>
#include <vector>

#include "chanmig/ChannelDef.h"
#include "chanmig/SurveyRecord.h"

namespace devourer {
namespace chanmig {

/* One accepted dwell, reduced to the rate/fraction form scoring consumes.
 * Rates are per-second of valid observation. nhm_busy_pct 255 = absent. */
struct BinCell {
  int64_t t_ms = 0; /* record time (dwell end, producer's monotonic clock) */
  int64_t observe_ms = 0;
  double cca_rate = 0.0;
  double fa_rate = 0.0;
  uint8_t nhm_busy_pct = 255;
  double dvr_air_frac = 0.0; /* recognized own-video airtime / observe */
  double oth_air_frac = 0.0; /* other decoded airtime / observe */
  uint32_t frames = 0;
  uint64_t round = 0;
};

class EvidenceStore {
public:
  enum class Fold {
    Accepted,
    RejectedFlags,        /* dwell aborted / truncated / read failed */
    RejectedSuspect,      /* counter delta implausible for the window */
    RejectedStale,        /* record older than max_age at ingest */
    RejectedUnknownBin,   /* bin serves no configured candidate */
    RejectedMixedScout,   /* different scout_id — store reset */
    RejectedPlanMismatch, /* produced under a different plan hash */
  };
  static constexpr int kFoldReasons = 7;

  EvidenceStore(std::vector<ChannelDef> candidates, uint32_t plan_hash,
                int64_t max_age_ms, size_t ring_cap = 32)
      : plan_hash_(plan_hash), max_age_ms_(max_age_ms), ring_cap_(ring_cap) {
    for (const ChannelDef &c : candidates) {
      Cand cd;
      cd.def = c;
      cd.n_bins = constituent_bins(c, cd.bins);
      cands_.push_back(cd);
      for (int i = 0; i < cd.n_bins; i++)
        if (find_bin(cd.bins[i]) == nullptr)
          bins_.push_back(BinRing{cd.bins[i], {}});
    }
  }

  /* Counter-plausibility ceiling: no chip emits more than ~1000 CCA or FA
   * events per millisecond of observation; a wrapped/reset hardware counter
   * reads orders of magnitude above it. */
  static constexpr uint64_t kMaxCountsPerMs = 1000;

  Fold ingest(const SurveyDwell &d, int64_t now_ms) {
    if (d.plan_hash != plan_hash_)
      return count(Fold::RejectedPlanMismatch);
    if (d.flags & (kFlagRetuneFailed | kFlagReadFailed | kFlagTruncated))
      return count(Fold::RejectedFlags);
    if (scout_id_ != 0 && d.scout_id != scout_id_) {
      /* Calibration domain changed: raw units are no longer comparable with
       * what the rings hold. Reset rather than mix. */
      for (BinRing &b : bins_)
        b.cells.clear();
      scout_id_ = d.scout_id;
      return count(Fold::RejectedMixedScout);
    }
    if (scout_id_ == 0)
      scout_id_ = d.scout_id;
    if (now_ms - d.t_end_ms > max_age_ms_)
      return count(Fold::RejectedStale);
    if (d.flags & kFlagCounterSuspect)
      return count(Fold::RejectedSuspect);
    if (d.valid_fa) {
      const uint64_t ceiling =
          kMaxCountsPerMs * static_cast<uint64_t>(
                                d.observe_ms > 0 ? d.observe_ms : 1);
      if (d.cca_ofdm > ceiling || d.fa_ofdm > ceiling ||
          d.cca_cck > ceiling || d.fa_cck > ceiling)
        return count(Fold::RejectedSuspect);
    }
    if (d.observe_ms <= 0)
      return count(Fold::RejectedFlags);

    if (d.flags & kFlagFullWidth) {
      Cand *c = find_cand(d.def);
      if (c == nullptr)
        return count(Fold::RejectedUnknownBin);
      push_cell(c->wide, d);
      bump();
      return count(Fold::Accepted);
    }
    BinRing *b = find_bin(d.def.primary);
    if (b == nullptr)
      return count(Fold::RejectedUnknownBin);
    push_cell(b->cells, d);
    bump();
    return count(Fold::Accepted);
  }

  /* Evidence generation: increments on every accepted fold. Decisions cite
   * the generation they were computed at, making any output reproducible
   * from the record stream. */
  uint64_t generation() const { return gen_; }
  uint32_t scout_id() const { return scout_id_; }
  uint32_t fold_count(Fold f) const {
    return fold_counts_[static_cast<int>(f)];
  }

  size_t candidate_count() const { return cands_.size(); }
  const ChannelDef &candidate(size_t i) const { return cands_[i].def; }

  /* Coverage: how many of the candidate's constituent bins hold at least one
   * fresh cell (age <= max_age at `now`). A wide candidate with partial
   * coverage must never rank — the missing bin could hide the interferer. */
  int bins_covered(size_t cand, int64_t now_ms) const {
    const Cand &c = cands_[cand];
    int n = 0;
    for (int i = 0; i < c.n_bins; i++) {
      const BinRing *b = find_bin_const(c.bins[i]);
      if (b && newest_fresh(*b, now_ms) != nullptr)
        ++n;
    }
    return n;
  }
  int bin_count(size_t cand) const { return cands_[cand].n_bins; }

  /* Age of the candidate's OLDEST freshest-per-bin cell — the evidence age a
   * decision must cite (the least-recently-observed constituent bin bounds
   * how much can have changed unseen). -1 when any bin has no fresh cell. */
  int64_t evidence_age_ms(size_t cand, int64_t now_ms) const {
    const Cand &c = cands_[cand];
    int64_t worst = -1;
    for (int i = 0; i < c.n_bins; i++) {
      const BinRing *b = find_bin_const(c.bins[i]);
      const BinCell *cell = b ? newest_fresh(*b, now_ms) : nullptr;
      if (cell == nullptr)
        return -1;
      const int64_t age = now_ms - cell->t_ms;
      if (age > worst)
        worst = age;
    }
    return worst;
  }

  /* All fresh cells of one constituent bin (newest last), for the scoring
   * layer's quantile/burstiness derivations. */
  void fresh_cells(uint8_t bin_ch, int64_t now_ms,
                   std::vector<const BinCell *> &out) const {
    out.clear();
    const BinRing *b = find_bin_const(bin_ch);
    if (b == nullptr)
      return;
    for (const BinCell &cell : b->cells)
      if (now_ms - cell.t_ms <= max_age_ms_)
        out.push_back(&cell);
  }
  void wide_cells(size_t cand, int64_t now_ms,
                  std::vector<const BinCell *> &out) const {
    out.clear();
    for (const BinCell &cell : cands_[cand].wide)
      if (now_ms - cell.t_ms <= max_age_ms_)
        out.push_back(&cell);
  }

  /* Distinct sweep rounds represented in the candidate's fresh evidence —
   * the min-rounds gate input (a single lucky round is not persistence). */
  uint64_t rounds_covered(size_t cand, int64_t now_ms) const {
    const Cand &c = cands_[cand];
    uint64_t lo = UINT64_MAX, hi = 0;
    bool any = false;
    for (int i = 0; i < c.n_bins; i++) {
      const BinRing *b = find_bin_const(c.bins[i]);
      if (b == nullptr)
        continue;
      for (const BinCell &cell : b->cells) {
        if (now_ms - cell.t_ms > max_age_ms_)
          continue;
        any = true;
        if (cell.round < lo)
          lo = cell.round;
        if (cell.round > hi)
          hi = cell.round;
      }
    }
    return any ? hi - lo + 1 : 0;
  }

  int64_t max_age_ms() const { return max_age_ms_; }

private:
  struct BinRing {
    uint8_t ch;
    std::vector<BinCell> cells; /* bounded ring, newest last */
  };
  struct Cand {
    ChannelDef def;
    uint8_t bins[4] = {};
    int n_bins = 0;
    std::vector<BinCell> wide; /* full-width verification dwells */
  };

  BinRing *find_bin(uint8_t ch) {
    for (BinRing &b : bins_)
      if (b.ch == ch)
        return &b;
    return nullptr;
  }
  const BinRing *find_bin_const(uint8_t ch) const {
    for (const BinRing &b : bins_)
      if (b.ch == ch)
        return &b;
    return nullptr;
  }
  Cand *find_cand(const ChannelDef &d) {
    for (Cand &c : cands_)
      if (c.def.same_rf(d))
        return &c;
    return nullptr;
  }
  const BinCell *newest_fresh(const BinRing &b, int64_t now_ms) const {
    for (auto it = b.cells.rbegin(); it != b.cells.rend(); ++it)
      if (now_ms - it->t_ms <= max_age_ms_)
        return &*it;
    return nullptr;
  }
  void push_cell(std::vector<BinCell> &ring, const SurveyDwell &d) {
    BinCell cell;
    cell.t_ms = d.t_end_ms;
    cell.observe_ms = d.observe_ms;
    const double sec = static_cast<double>(d.observe_ms) / 1000.0;
    if (d.valid_fa && sec > 0) {
      cell.cca_rate = (d.cca_ofdm + d.cca_cck) / sec;
      cell.fa_rate = (d.fa_ofdm + d.fa_cck) / sec;
    }
    cell.nhm_busy_pct = d.valid_nhm ? d.nhm_busy_pct : 255;
    if (d.observe_ms > 0) {
      const double us = static_cast<double>(d.observe_ms) * 1000.0;
      cell.dvr_air_frac = static_cast<double>(d.dvr_air_us) / us;
      cell.oth_air_frac = static_cast<double>(d.oth_air_us) / us;
      if (cell.dvr_air_frac > 1.0)
        cell.dvr_air_frac = 1.0;
      if (cell.oth_air_frac > 1.0)
        cell.oth_air_frac = 1.0;
    }
    cell.frames = d.frames;
    cell.round = d.round;
    if (ring.size() >= ring_cap_)
      ring.erase(ring.begin());
    ring.push_back(cell);
  }
  Fold count(Fold f) {
    ++fold_counts_[static_cast<int>(f)];
    return f;
  }
  void bump() { ++gen_; }

  uint32_t plan_hash_;
  int64_t max_age_ms_;
  size_t ring_cap_;
  uint32_t scout_id_ = 0;
  uint64_t gen_ = 0;
  std::vector<Cand> cands_;
  std::vector<BinRing> bins_;
  uint32_t fold_counts_[kFoldReasons] = {};
};

} /* namespace chanmig */
} /* namespace devourer */

#endif /* DEVOURER_CHANMIG_EVIDENCE_STORE_H */
