#include "chanmig/ChannelScore.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace devourer {
namespace chanmig {

const char *reason_name(Reason r) {
  switch (r) {
  case Reason::RecommendBetterCandidate: return "RecommendBetterCandidate";
  case Reason::HoldActiveHealthy: return "HoldActiveHealthy";
  case Reason::HoldImpairmentNotPersistent: return "HoldImpairmentNotPersistent";
  case Reason::HoldImpairmentNotChannel: return "HoldImpairmentNotChannel";
  case Reason::HoldBroadDegradation: return "HoldBroadDegradation";
  case Reason::HoldCooldown: return "HoldCooldown";
  case Reason::HoldPrimaryTelemetryStale: return "HoldPrimaryTelemetryStale";
  case Reason::HoldNoQualifiedCandidate: return "HoldNoQualifiedCandidate";
  case Reason::HoldImprovementMargin: return "HoldImprovementMargin";
  case Reason::HoldScoutUnhealthy: return "HoldScoutUnhealthy";
  case Reason::RejEvidenceStale: return "RejEvidenceStale";
  case Reason::RejInsufficientObservation: return "RejInsufficientObservation";
  case Reason::RejInsufficientRounds: return "RejInsufficientRounds";
  case Reason::RejIncompleteWidth: return "RejIncompleteWidth";
  case Reason::RejIllegalDef: return "RejIllegalDef";
  case Reason::RejNoIr: return "RejNoIr";
  case Reason::RejOccupied: return "RejOccupied";
  case Reason::RejBursty: return "RejBursty";
  case Reason::RejOverlapPenalty: return "RejOverlapPenalty";
  case Reason::RejMixedAdapter: return "RejMixedAdapter";
  case Reason::RejScoutUnhealthy: return "RejScoutUnhealthy";
  case Reason::RejActiveChannel: return "RejActiveChannel";
  }
  return "?";
}

uint32_t PolicyConfig::policy_hash() const {
  uint32_t h = 2166136261u;
  auto fold = [&h](const void *p, size_t n) {
    const uint8_t *b = static_cast<const uint8_t *>(p);
    for (size_t i = 0; i < n; i++) {
      h ^= b[i];
      h *= 16777619u;
    }
  };
  fold(&impaired_windows_min, sizeof(impaired_windows_min));
  fold(&active.loss_impair, sizeof(active.loss_impair));
  fold(&active.crc_impair, sizeof(active.crc_impair));
  fold(&active.fec_margin_impair, sizeof(active.fec_margin_impair));
  fold(&active.min_frames, sizeof(active.min_frames));
  fold(&min_rounds, sizeof(min_rounds));
  fold(&min_clean_obs_ms, sizeof(min_clean_obs_ms));
  fold(&max_evidence_age_ms, sizeof(max_evidence_age_ms));
  fold(&qualify_max_occupancy, sizeof(qualify_max_occupancy));
  fold(&bursty_max, sizeof(bursty_max));
  fold(&improvement_margin, sizeof(improvement_margin));
  fold(&min_recommend_interval_ms, sizeof(min_recommend_interval_ms));
  fold(&broad_degrade_frac, sizeof(broad_degrade_frac));
  fold(&fa_half_rate, sizeof(fa_half_rate));
  fold(&overlap_penalty, sizeof(overlap_penalty));
  fold(&age_decay_tau_ms, sizeof(age_decay_tau_ms));
  return h;
}

bool parse_policy_file(const char *path, PolicyConfig &cfg, std::string *err) {
  std::FILE *f = std::fopen(path, "r");
  if (f == nullptr) {
    if (err)
      *err = "cannot open policy file";
    return false;
  }
  char line[256];
  while (std::fgets(line, sizeof(line), f)) {
    char key[64];
    double val;
    /* `key value`, '#' comments, blank lines ignored. */
    if (line[0] == '#' || line[0] == '\n')
      continue;
    if (std::sscanf(line, "%63s %lf", key, &val) != 2)
      continue;
    if (!std::strcmp(key, "impaired_windows_min"))
      cfg.impaired_windows_min = static_cast<int>(val);
    else if (!std::strcmp(key, "loss_impair"))
      cfg.active.loss_impair = val;
    else if (!std::strcmp(key, "crc_impair"))
      cfg.active.crc_impair = val;
    else if (!std::strcmp(key, "fec_margin_impair"))
      cfg.active.fec_margin_impair = val;
    else if (!std::strcmp(key, "active_min_frames"))
      cfg.active.min_frames = static_cast<uint32_t>(val);
    else if (!std::strcmp(key, "min_rounds"))
      cfg.min_rounds = static_cast<uint64_t>(val < 0 ? 0 : val);
    else if (!std::strcmp(key, "min_clean_obs_ms"))
      cfg.min_clean_obs_ms = static_cast<int64_t>(val);
    else if (!std::strcmp(key, "max_evidence_age_ms"))
      cfg.max_evidence_age_ms = static_cast<int64_t>(val);
    else if (!std::strcmp(key, "qualify_max_occupancy"))
      cfg.qualify_max_occupancy = val;
    else if (!std::strcmp(key, "bursty_max"))
      cfg.bursty_max = val;
    else if (!std::strcmp(key, "improvement_margin"))
      cfg.improvement_margin = val;
    else if (!std::strcmp(key, "min_recommend_interval_ms"))
      cfg.min_recommend_interval_ms = static_cast<int64_t>(val);
    else if (!std::strcmp(key, "broad_degrade_frac"))
      cfg.broad_degrade_frac = val;
    else if (!std::strcmp(key, "fa_half_rate"))
      cfg.fa_half_rate = val;
    else if (!std::strcmp(key, "overlap_penalty"))
      cfg.overlap_penalty = val;
    else if (!std::strcmp(key, "age_decay_tau_ms"))
      cfg.age_decay_tau_ms = val;
  }
  std::fclose(f);
  return true;
}

RecommendEngine::RecommendEngine(PolicyConfig policy,
                                 std::vector<ChannelDef> candidates,
                                 uint32_t plan_hash, ChannelDef active)
    : policy_(policy), plan_hash_(plan_hash), active_(active),
      store_(std::move(candidates), plan_hash, policy.max_evidence_age_ms),
      track_(policy.active) {}

/* Per-cell occupancy proxy in [0,1] from the two physical-ish signals that
 * on-air-track real occupancy across generations: decoded foreign airtime,
 * and a bounded false-alarm term (NHM is deliberately excluded — its absolute
 * floor is generation-dependent, e.g. the 8822B reads busy on a quiet
 * channel). */
static double cell_occupancy(const BinCell &c, double fa_half) {
  const double fa_term = c.fa_rate / (c.fa_rate + fa_half);
  double occ = c.oth_air_frac + fa_term;
  if (occ > 1.0)
    occ = 1.0;
  return occ;
}

static double quantile(std::vector<double> v, double q) {
  if (v.empty())
    return 0.0;
  std::sort(v.begin(), v.end());
  const double idx = q * (v.size() - 1);
  const size_t lo = static_cast<size_t>(idx);
  const size_t hi = lo + 1 < v.size() ? lo + 1 : lo;
  const double frac = idx - lo;
  return v[lo] * (1.0 - frac) + v[hi] * frac;
}

CandidateScore RecommendEngine::score_candidate(size_t i,
                                                int64_t now_ms) const {
  CandidateScore s;
  s.def = store_.candidate(i);
  uint8_t bins[4];
  const int nb = constituent_bins(s.def, bins);
  s.bins_total = nb;
  s.bins_covered = store_.bins_covered(i, now_ms);
  s.evidence_age_ms = store_.evidence_age_ms(i, now_ms);
  s.rounds = store_.rounds_covered(i, now_ms);

  auto reject = [&s](Reason r) {
    if (s.n_rejections < 6)
      s.rejections[s.n_rejections++] = r;
  };

  /* Occupancy = the WORST constituent bin (a wide channel is only as clean as
   * its dirtiest 20 MHz slice). Per bin, q50/q90 of cell occupancy. */
  double worst_q50 = 0.0, worst_q90 = 0.0;
  uint64_t total_obs = 0;
  bool any_cells = false;
  int64_t min_bin_clean_ms = INT64_MAX;
  for (int b = 0; b < nb; b++) {
    std::vector<const BinCell *> cells;
    store_.fresh_cells(bins[b], now_ms, cells);
    int64_t bin_obs = 0;
    std::vector<double> occ;
    for (const BinCell *c : cells) {
      occ.push_back(cell_occupancy(*c, policy_.fa_half_rate));
      bin_obs += c->observe_ms;
      total_obs += static_cast<uint64_t>(c->observe_ms);
      any_cells = true;
    }
    if (!occ.empty()) {
      const double q50 = quantile(occ, 0.5), q90 = quantile(occ, 0.9);
      if (q50 > worst_q50)
        worst_q50 = q50;
      if (q90 > worst_q90)
        worst_q90 = q90;
    }
    if (bin_obs < min_bin_clean_ms)
      min_bin_clean_ms = bin_obs;
  }
  s.observe_ms = total_obs;
  s.occ_q50 = worst_q50;
  s.occ_q90 = worst_q90;
  s.burstiness = worst_q90 - worst_q50;

  /* Overlap penalty: a candidate overlapping or adjacent to the (impaired)
   * active channel may inherit its interferer. */
  if (overlap_mhz(s.def, active_) > 0 || adjacent(s.def, active_))
    s.overlap_pen = policy_.overlap_penalty;

  const double occupancy =
      std::min(1.0, s.occ_q50 + s.overlap_pen);
  s.score = 1.0 - occupancy;

  /* Confidence: an engineering scalar (not a formal CI) folding observation
   * volume, round coverage, and evidence age. Documented in the doc. */
  const double obs_factor =
      std::min(1.0, static_cast<double>(total_obs) /
                        std::max<int64_t>(1, policy_.min_clean_obs_ms * nb * 2));
  const double rounds_factor =
      std::min(1.0, static_cast<double>(s.rounds) /
                        std::max<uint64_t>(1, policy_.min_rounds * 2));
  const double age_factor =
      s.evidence_age_ms < 0
          ? 0.0
          : std::exp(-static_cast<double>(s.evidence_age_ms) /
                     policy_.age_decay_tau_ms);
  s.confidence = obs_factor * rounds_factor * age_factor;

  /* Gates (order = most-fundamental first; all recorded). */
  if (s.def.no_ir)
    reject(Reason::RejNoIr);
  if (validate(s.def) != DefError::Ok)
    reject(Reason::RejIllegalDef);
  if (s.def.same_rf(active_))
    reject(Reason::RejActiveChannel);
  if (!scout_healthy_)
    reject(Reason::RejScoutUnhealthy);
  if (!any_cells || s.evidence_age_ms < 0)
    reject(Reason::RejEvidenceStale);
  else if (s.evidence_age_ms > policy_.max_evidence_age_ms)
    reject(Reason::RejEvidenceStale);
  if (s.bins_covered < s.bins_total)
    reject(Reason::RejIncompleteWidth);
  if (any_cells && min_bin_clean_ms < policy_.min_clean_obs_ms)
    reject(Reason::RejInsufficientObservation);
  if (s.rounds < policy_.min_rounds)
    reject(Reason::RejInsufficientRounds);
  if (any_cells && occupancy > policy_.qualify_max_occupancy)
    reject(Reason::RejOccupied);
  if (any_cells && s.burstiness > policy_.bursty_max)
    reject(Reason::RejBursty);

  s.qualified = (s.n_rejections == 0);
  return s;
}

Decision RecommendEngine::decide(int64_t now_ms) {
  Decision d;
  d.plan_hash = plan_hash_;
  d.policy_hash = policy_.policy_hash();
  d.evidence_gen = store_.generation();
  d.active_impaired_windows = track_.consecutive_channel_impaired();
  d.active_verdict = track_.last_verdict();
  d.active_domain = track_.last_domain();

  /* Score + rank every candidate (best score first, deterministic tie-break
   * by ascending RF key). */
  for (size_t i = 0; i < store_.candidate_count(); ++i)
    d.ranking.push_back(score_candidate(i, now_ms));
  std::sort(d.ranking.begin(), d.ranking.end(),
            [](const CandidateScore &a, const CandidateScore &b) {
              if (a.qualified != b.qualified)
                return a.qualified; /* qualified first */
              if (a.score != b.score)
                return a.score > b.score;
              return a.def.key() < b.def.key();
            });

  auto finish = [&](Decision::Kind k, Reason r, const std::string &text) {
    d.kind = k;
    d.primary_reason = r;
    d.human_reason = text;
    return d;
  };

  /* leg 1 — active-channel fault domain and persistence. */
  if (!track_.have() || d.active_domain == Impair::TelemetryDown)
    return finish(Decision::Kind::Hold, Reason::HoldPrimaryTelemetryStale,
                  "no trustworthy primary delivery telemetry");
  if (!scout_healthy_)
    return finish(Decision::Kind::Hold, Reason::HoldScoutUnhealthy,
                  "scout unhealthy — candidate evidence untrusted");
  if (d.active_domain == Impair::WeakSignal)
    return finish(Decision::Kind::Hold, Reason::HoldImpairmentNotChannel,
                  "active link weak (range/sensitivity) — a move will not help");
  if (d.active_domain == Impair::Saturation)
    return finish(Decision::Kind::Hold, Reason::HoldImpairmentNotChannel,
                  "active link saturated (near-field) — reduce power, do not move");
  if (d.active_domain == Impair::Healthy &&
      d.active_impaired_windows < policy_.impaired_windows_min)
    return finish(Decision::Kind::Hold, Reason::HoldActiveHealthy,
                  "active video delivering — hold");
  if (d.active_impaired_windows < policy_.impaired_windows_min)
    return finish(Decision::Kind::Hold, Reason::HoldImpairmentNotPersistent,
                  "impairment not yet persistent enough to act");

  /* Broad degradation: if most candidates are ALSO busy, the interference is
   * not this channel's — moving won't escape it. */
  int busy = 0, scored = 0;
  for (const CandidateScore &c : d.ranking) {
    if (c.evidence_age_ms < 0)
      continue;
    ++scored;
    if (std::min(1.0, c.occ_q50 + c.overlap_pen) > policy_.qualify_max_occupancy)
      ++busy;
  }
  if (scored > 0 &&
      static_cast<double>(busy) / scored >= policy_.broad_degrade_frac)
    return finish(Decision::Kind::Hold, Reason::HoldBroadDegradation,
                  "interference spans most candidates — broad, not channel-local");

  /* leg 2 — a qualified, materially-clean candidate. */
  const CandidateScore &best = d.ranking.front();
  if (!best.qualified)
    return finish(Decision::Kind::Hold, Reason::HoldNoQualifiedCandidate,
                  "no candidate passed all qualification gates");
  /* Materially better than merely-acceptable: score must clear the qualify
   * floor by the configured margin (a barely-clean candidate is not worth a
   * whole-link move). */
  const double floor = 1.0 - policy_.qualify_max_occupancy;
  if (best.score < floor + policy_.improvement_margin)
    return finish(Decision::Kind::Hold, Reason::HoldImprovementMargin,
                  "best candidate clean but not by the required margin");

  /* Cooldown: rate-limit recommendations so a rank alternation between two
   * near-equal candidates cannot churn the operator. */
  if (last_recommend_ms_ != INT64_MIN &&
      now_ms - last_recommend_ms_ < policy_.min_recommend_interval_ms)
    return finish(Decision::Kind::Hold, Reason::HoldCooldown,
                  "within the minimum interval since the last recommendation");

  last_recommend_ms_ = now_ms;
  d.target = best.def;
  char from[20], to[20];
  active_.format(from, sizeof(from));
  best.def.format(to, sizeof(to));
  char text[160];
  std::snprintf(text, sizeof(text),
                "%s impaired %d windows; %s clean (occ %.2f, %llu rounds, "
                "%llu ms) — recommend move",
                from, d.active_impaired_windows, to, best.occ_q50,
                (unsigned long long)best.rounds,
                (unsigned long long)best.observe_ms);
  return finish(Decision::Kind::Recommend, Reason::RecommendBetterCandidate,
                text);
}

} /* namespace chanmig */
} /* namespace devourer */
