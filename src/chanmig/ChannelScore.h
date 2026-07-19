/* RecommendEngine — the pure, explainable channel-recommendation policy.
 *
 * Inputs are timestamped records (scout survey folds + primary active-link
 * windows); output is a Decision that is either Hold or Recommend, carrying a
 * full per-candidate ranking with exact reason/rejection codes and the
 * evidence generation it was computed at. The engine reads no clock (the
 * caller passes now), no env, no device — it only ever emits advice, so it
 * structurally cannot retune anything.
 *
 * The recommendation law has two independent legs, deliberately kept in
 * separate unit worlds (the "never compare raw across adapters / across the
 * primary-scout boundary" rule):
 *   leg 1  the ACTIVE channel is PERSISTENTLY impaired by DELIVERY evidence
 *          (the primary receiver's loss/FEC/CRC), and the impairment's fault
 *          domain is channel-attributable — not weak-signal, not near-field
 *          saturation, not a broad multi-candidate degradation;
 *   leg 2  a CANDIDATE is sufficiently observed (rounds, per-bin clean time,
 *          full-width coverage, freshness), legal, and materially clean in
 *          the scout's own within-scout-normalized occupancy terms.
 * A single threshold crossing on either leg never recommends a move.
 *
 * Thresholds live in PolicyConfig, loadable from a bare `key value` file so
 * the exact policy artifact is versionable and hashable; policy_hash is
 * stamped into every decision, making any output reproducible from the logs.
 *
 * Pure; selftested against the issue's offline scenario matrix. */
#ifndef DEVOURER_CHANMIG_CHANNEL_SCORE_H
#define DEVOURER_CHANMIG_CHANNEL_SCORE_H

#include <cstdint>
#include <string>
#include <vector>

#include "chanmig/ActiveLink.h"
#include "chanmig/ChannelDef.h"
#include "chanmig/EvidenceStore.h"

namespace devourer {
namespace chanmig {

enum class Reason {
  /* recommend */
  RecommendBetterCandidate,
  /* holds (whole-decision) */
  HoldActiveHealthy,
  HoldImpairmentNotPersistent,
  HoldImpairmentNotChannel,
  HoldBroadDegradation,
  HoldCooldown,
  HoldPrimaryTelemetryStale,
  HoldNoQualifiedCandidate,
  HoldImprovementMargin,
  HoldScoutUnhealthy,
  /* per-candidate rejections */
  RejEvidenceStale,
  RejInsufficientObservation,
  RejInsufficientRounds,
  RejIncompleteWidth,
  RejIllegalDef,
  RejNoIr,
  RejOccupied,
  RejBursty,
  RejOverlapPenalty,
  RejMixedAdapter,
  RejScoutUnhealthy,
  RejActiveChannel,
};

const char *reason_name(Reason r);

struct PolicyConfig {
  /* leg 1 — active-channel impairment */
  int impaired_windows_min = 5;         /* consecutive channel-impaired windows */
  ActivePolicy active;

  /* leg 2 — candidate qualification */
  uint64_t min_rounds = 3;              /* distinct complete scan rounds */
  int64_t min_clean_obs_ms = 3000;      /* fresh observation per constituent bin */
  int64_t max_evidence_age_ms = 60000;
  double qualify_max_occupancy = 0.20;  /* candidate occupancy above = Occupied */
  double bursty_max = 0.30;             /* q90-q50 above = Bursty */
  /* A candidate can be acceptable (qualifies) yet not worth a whole-link move.
   * To RECOMMEND, its score must clear the qualify floor (1 - occupancy bar)
   * by this margin — hysteresis between "fine to sit on" and "worth escaping
   * to". Kept small enough that floor + margin stays <= 1. */
  double improvement_margin = 0.08;

  /* anti-churn / broad-degradation */
  int64_t min_recommend_interval_ms = 30000;
  double broad_degrade_frac = 0.75;     /* fraction of candidates busy = broad */

  /* occupancy model + confidence */
  double fa_half_rate = 200.0;          /* fa/s giving a 0.5 fa occupancy term */
  double overlap_penalty = 0.15;        /* added occupancy for an active-adjacent cand */
  double age_decay_tau_ms = 30000.0;    /* confidence decay constant */

  uint32_t policy_hash() const;
};

/* Parse a bare `key value` policy file into cfg (unknown keys ignored, values
 * clamped to sane ranges). Returns false only on an unreadable file. */
bool parse_policy_file(const char *path, PolicyConfig &cfg,
                       std::string *err = nullptr);

struct CandidateScore {
  ChannelDef def;
  bool qualified = false;
  double score = 0.0;      /* [0,1], 1 = clean */
  double confidence = 0.0; /* [0,1] engineering scalar (obs, rounds, age) */
  double occ_q50 = 0.0, occ_q90 = 0.0;
  double burstiness = 0.0;
  double overlap_pen = 0.0;
  int64_t evidence_age_ms = -1;
  uint64_t observe_ms = 0;
  uint64_t rounds = 0;
  int bins_covered = 0, bins_total = 0;
  Reason rejections[6] = {};
  int n_rejections = 0;
};

struct Decision {
  enum class Kind { Hold, Recommend } kind = Kind::Hold;
  Reason primary_reason = Reason::HoldPrimaryTelemetryStale;
  ChannelDef target;                    /* Recommend only */
  std::vector<CandidateScore> ranking;  /* every candidate, best first */
  int active_impaired_windows = 0;
  LinkVerdict active_verdict = LinkVerdict::NoSignal;
  Impair active_domain = Impair::TelemetryDown;
  uint64_t evidence_gen = 0;
  uint32_t plan_hash = 0, policy_hash = 0;
  std::string human_reason;
};

class RecommendEngine {
public:
  RecommendEngine(PolicyConfig policy, std::vector<ChannelDef> candidates,
                  uint32_t plan_hash, ChannelDef active);

  EvidenceStore::Fold ingest_dwell(const SurveyDwell &d, int64_t now_ms) {
    return store_.ingest(d, now_ms);
  }
  void ingest_active(const ActiveLinkWindow &w) { track_.add(w); }
  void note_scout_health(bool healthy) { scout_healthy_ = healthy; }

  uint64_t generation() const { return store_.generation(); }

  Decision decide(int64_t now_ms);

private:
  CandidateScore score_candidate(size_t i, int64_t now_ms) const;

  PolicyConfig policy_;
  uint32_t plan_hash_;
  ChannelDef active_;
  EvidenceStore store_;
  ActiveLinkTrack track_;
  bool scout_healthy_ = true;
  int64_t last_recommend_ms_ = INT64_MIN;
};

} /* namespace chanmig */
} /* namespace devourer */

#endif /* DEVOURER_CHANMIG_CHANNEL_SCORE_H */
