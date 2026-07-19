/* RecommendEngine offline-scenario selftest — the #277 deliverable gate.
 *
 * Each of the issue's ten offline scenarios is a deterministic synthetic
 * trace fed through the shipping engine, asserting the exact decision kind +
 * reason code (and generation). Plus the cross-cutting edges: age boundary,
 * cooldown latch, generation monotonicity, deterministic tie-break, and the
 * policy-file parse + hash stability. */
#include "chanmig/ChannelScore.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

static int fails;
#define CHECK(x, msg)                                                          \
  do {                                                                         \
    if (!(x)) {                                                                \
      std::fprintf(stderr, "FAIL: %s\n", msg);                                 \
      ++fails;                                                                 \
    }                                                                          \
  } while (0)

using namespace devourer;
using namespace devourer::chanmig;

static constexpr uint32_t kPlan = 0x5c0a7d01;
static constexpr int kObs = 200; /* ms per synthetic dwell */

static std::vector<ChannelDef> parse(const char *spec) {
  std::vector<ChannelDef> out;
  std::vector<PlanParseError> errs;
  parse_scan_plan(spec, out, errs);
  return out;
}

/* Feed n dwells for one 20 MHz bin at occupancy `occ` (via foreign airtime),
 * spaced 200 ms from base_t, cycling rounds 1..4. */
static void feed_bin(RecommendEngine &e, uint8_t bin, int64_t base_t,
                     double occ, int n = 20, uint32_t scout = 1) {
  for (int i = 0; i < n; i++) {
    SurveyDwell d;
    d.def.band = bin <= 14 ? 2 : 5;
    d.def.primary = bin;
    d.def.width = CHANNEL_WIDTH_20;
    d.plan_hash = kPlan;
    d.observe_ms = kObs;
    d.t_start_ms = base_t + i * 200;
    d.t_end_ms = d.t_start_ms + kObs;
    d.valid_fa = true;
    d.round = 1 + (i % 4);
    d.scout_id = scout;
    d.oth_air_us = static_cast<uint64_t>(occ * kObs * 1000.0);
    e.ingest_dwell(d, d.t_end_ms);
  }
}

/* Feed a bursty bin: mostly clean with occasional strong bursts (low median,
 * high q90) so burstiness triggers without median occupancy. */
static void feed_bin_bursty(RecommendEngine &e, uint8_t bin, int64_t base_t,
                            int n = 40) {
  for (int i = 0; i < n; i++) {
    const double occ = (i % 5 == 0) ? 0.9 : 0.0; /* 1-in-5 burst */
    SurveyDwell d;
    d.def.band = 5;
    d.def.primary = bin;
    d.def.width = CHANNEL_WIDTH_20;
    d.plan_hash = kPlan;
    d.observe_ms = kObs;
    d.t_start_ms = base_t + i * 100;
    d.t_end_ms = d.t_start_ms + kObs;
    d.valid_fa = true;
    d.round = 1 + (i % 4);
    d.scout_id = 1;
    d.oth_air_us = static_cast<uint64_t>(occ * kObs * 1000.0);
    e.ingest_dwell(d, d.t_end_ms);
  }
}

static ActiveLinkWindow active_win(int64_t t, LinkVerdict v, bool impaired) {
  ActiveLinkWindow w;
  w.t_ms = t;
  w.telemetry_ok = true;
  w.have_quality = true;
  w.verdict = v;
  w.rssi_max_dbm = -55;
  w.snr_mean_db = 20;
  w.evm_mean_db = -50;
  w.have_delivery = true;
  w.expected = 100;
  w.delivered = impaired ? 70 : 100; /* 30% loss when impaired */
  return w;
}

static void feed_active(RecommendEngine &e, LinkVerdict v, bool impaired,
                        int n = 8, int64_t t0 = 100) {
  for (int i = 0; i < n; i++)
    e.ingest_active(active_win(t0 + i * 100, v, impaired));
}

static const char *kind(const Decision &d) {
  return d.kind == Decision::Kind::Recommend ? "Recommend" : "Hold";
}

int main() {
  PolicyConfig pol; /* defaults */

  /* Scenario 1 — persistent narrowband interferer, one clean candidate. */
  {
    RecommendEngine e(pol, parse("36,149"), kPlan,
                      ChannelDef{5, 60, CHANNEL_WIDTH_20, 0});
    feed_bin(e, 36, 1000, 0.0);   /* clean */
    feed_bin(e, 149, 1000, 0.6);  /* interfered */
    feed_active(e, LinkVerdict::Marginal, /*impaired=*/true);
    Decision d = e.decide(6000);
    CHECK(d.kind == Decision::Kind::Recommend, "S1 recommends");
    CHECK(d.primary_reason == Reason::RecommendBetterCandidate, "S1 reason");
    CHECK(d.target.primary == 36, "S1 targets the clean candidate");
    CHECK(d.ranking.front().def.primary == 36, "S1 ranks clean first");
    CHECK(d.evidence_gen > 0, "S1 cites an evidence generation");
  }

  /* Scenario 2 — bursts misaligned with dwells never flap to a move. */
  {
    RecommendEngine e(pol, parse("36"), kPlan,
                      ChannelDef{5, 60, CHANNEL_WIDTH_20, 0});
    feed_bin_bursty(e, 36, 1000);
    feed_active(e, LinkVerdict::Marginal, true);
    Decision d = e.decide(6000);
    CHECK(d.kind == Decision::Kind::Hold, "S2 holds on a bursty candidate");
    CHECK(d.ranking.front().n_rejections > 0, "S2 candidate rejected");
    bool bursty = false;
    for (int i = 0; i < d.ranking.front().n_rejections; i++)
      bursty = bursty || d.ranking.front().rejections[i] == Reason::RejBursty;
    CHECK(bursty, "S2 rejection is burstiness");
    /* Re-deciding with the same evidence is stable (no flap). */
    Decision d2 = e.decide(6100);
    CHECK(d2.kind == Decision::Kind::Hold, "S2 stays held");
  }

  /* Scenario 3 — wanted video dominating CCA but delivering: hold, healthy. */
  {
    RecommendEngine e(pol, parse("36,149"), kPlan,
                      ChannelDef{5, 60, CHANNEL_WIDTH_20, 0});
    feed_bin(e, 36, 1000, 0.0);
    feed_bin(e, 149, 1000, 0.0);
    feed_active(e, LinkVerdict::Healthy, /*impaired=*/false);
    Decision d = e.decide(6000);
    CHECK(d.kind == Decision::Kind::Hold, "S3 holds");
    CHECK(d.primary_reason == Reason::HoldActiveHealthy, "S3 healthy hold");
  }

  /* Scenario 4 — weak wanted signal must not read as congestion. */
  {
    RecommendEngine e(pol, parse("36,149"), kPlan,
                      ChannelDef{5, 60, CHANNEL_WIDTH_20, 0});
    feed_bin(e, 36, 1000, 0.0);
    feed_bin(e, 149, 1000, 0.0);
    feed_active(e, LinkVerdict::Weak, /*impaired=*/true);
    Decision d = e.decide(6000);
    CHECK(d.kind == Decision::Kind::Hold, "S4 holds");
    CHECK(d.primary_reason == Reason::HoldImpairmentNotChannel,
          "S4 weak != congestion");
  }

  /* Scenario 5 — near-field saturation: back power off, do not move. */
  {
    RecommendEngine e(pol, parse("36,149"), kPlan,
                      ChannelDef{5, 60, CHANNEL_WIDTH_20, 0});
    feed_bin(e, 36, 1000, 0.0);
    feed_bin(e, 149, 1000, 0.0);
    feed_active(e, LinkVerdict::Saturated, true);
    Decision d = e.decide(6000);
    CHECK(d.kind == Decision::Kind::Hold, "S5 holds");
    CHECK(d.primary_reason == Reason::HoldImpairmentNotChannel,
          "S5 saturation != channel");
  }

  /* Scenario 6 — adjacent/overlap penalty orders a clean-but-adjacent
   * candidate below a clean-and-clear one. active=ch40, so ch36 is adjacent. */
  {
    RecommendEngine e(pol, parse("36,149"), kPlan,
                      ChannelDef{5, 40, CHANNEL_WIDTH_20, 0});
    feed_bin(e, 36, 1000, 0.0);  /* adjacent to active ch40 */
    feed_bin(e, 149, 1000, 0.0); /* far and clear */
    feed_active(e, LinkVerdict::Marginal, true);
    Decision d = e.decide(6000);
    CHECK(d.ranking.front().def.primary == 149,
          "S6 far candidate outranks the adjacent one");
    CHECK(d.ranking.front().overlap_pen == 0.0 &&
              d.ranking.back().overlap_pen > 0.0,
          "S6 overlap penalty applied to the adjacent candidate");
    CHECK(d.kind == Decision::Kind::Recommend && d.target.primary == 149,
          "S6 recommends the clear candidate");
  }

  /* Scenario 7 — every candidate degrades together: broad, not channel-local. */
  {
    RecommendEngine e(pol, parse("36,149"), kPlan,
                      ChannelDef{5, 60, CHANNEL_WIDTH_20, 0});
    feed_bin(e, 36, 1000, 0.6);
    feed_bin(e, 149, 1000, 0.7);
    feed_active(e, LinkVerdict::Interference, true);
    Decision d = e.decide(6000);
    CHECK(d.kind == Decision::Kind::Hold, "S7 holds");
    CHECK(d.primary_reason == Reason::HoldBroadDegradation, "S7 broad");
  }

  /* Scenario 8 — rank alternation + cooldown: a recommendation rate-limits
   * the next, so two near-equal candidates cannot churn. */
  {
    RecommendEngine e(pol, parse("36,149"), kPlan,
                      ChannelDef{5, 60, CHANNEL_WIDTH_20, 0});
    feed_bin(e, 36, 1000, 0.0);
    feed_bin(e, 149, 1000, 0.02);
    feed_active(e, LinkVerdict::Marginal, true);
    Decision d1 = e.decide(6000);
    CHECK(d1.kind == Decision::Kind::Recommend, "S8 first recommends");
    /* Make 149 marginally better and re-decide immediately. */
    feed_bin(e, 149, 6200, 0.0, 4);
    Decision d2 = e.decide(6300);
    CHECK(d2.kind == Decision::Kind::Hold &&
              d2.primary_reason == Reason::HoldCooldown,
          "S8 cooldown suppresses the churn");
  }

  /* Scenario 9 — a clean candidate goes stale after the interferer moved on. */
  {
    RecommendEngine e(pol, parse("36,149"), kPlan,
                      ChannelDef{5, 60, CHANNEL_WIDTH_20, 0});
    feed_bin(e, 36, 1000, 0.0);   /* observed long ago */
    feed_bin(e, 149, 1000, 0.6);
    feed_active(e, LinkVerdict::Marginal, true, 8, 90000);
    /* Decide far in the future: ch36 evidence is now beyond max age. */
    Decision d = e.decide(1000 + pol.max_evidence_age_ms + 5000);
    CHECK(d.kind == Decision::Kind::Hold, "S9 holds on stale evidence");
    CHECK(d.primary_reason == Reason::HoldNoQualifiedCandidate,
          "S9 no fresh qualified candidate");
    bool stale = false;
    for (const CandidateScore &c : d.ranking)
      for (int i = 0; i < c.n_rejections; i++)
        stale = stale || c.rejections[i] == Reason::RejEvidenceStale;
    CHECK(stale, "S9 candidate rejected as stale");
  }

  /* Scenario 10 — scout disconnect: candidate evidence untrusted. */
  {
    RecommendEngine e(pol, parse("36,149"), kPlan,
                      ChannelDef{5, 60, CHANNEL_WIDTH_20, 0});
    feed_bin(e, 36, 1000, 0.0);
    feed_bin(e, 149, 1000, 0.0);
    feed_active(e, LinkVerdict::Marginal, true);
    e.note_scout_health(false);
    Decision d = e.decide(6000);
    CHECK(d.kind == Decision::Kind::Hold, "S10 holds");
    CHECK(d.primary_reason == Reason::HoldScoutUnhealthy, "S10 scout unhealthy");
  }

  /* Edge — telemetry down forces a hold regardless of scout evidence. */
  {
    RecommendEngine e(pol, parse("36"), kPlan,
                      ChannelDef{5, 60, CHANNEL_WIDTH_20, 0});
    feed_bin(e, 36, 1000, 0.0);
    ActiveLinkWindow w;
    w.t_ms = 500;
    w.telemetry_ok = false;
    e.ingest_active(w);
    Decision d = e.decide(6000);
    CHECK(d.primary_reason == Reason::HoldPrimaryTelemetryStale,
          "telemetry-down hold");
  }

  /* Edge — impairment not yet persistent: impaired but under the window
   * count. */
  {
    RecommendEngine e(pol, parse("36"), kPlan,
                      ChannelDef{5, 60, CHANNEL_WIDTH_20, 0});
    feed_bin(e, 36, 1000, 0.0);
    feed_active(e, LinkVerdict::Marginal, true, /*n=*/3); /* < 5 */
    Decision d = e.decide(6000);
    CHECK(d.primary_reason == Reason::HoldImpairmentNotPersistent,
          "sub-threshold impairment holds");
  }

  /* Edge — generation is monotone across folds; decisions cite the latest. */
  {
    RecommendEngine e(pol, parse("36"), kPlan,
                      ChannelDef{5, 60, CHANNEL_WIDTH_20, 0});
    const uint64_t g0 = e.generation();
    feed_bin(e, 36, 1000, 0.0, 5);
    const uint64_t g1 = e.generation();
    CHECK(g1 == g0 + 5, "generation increments per accepted fold");
    Decision d = e.decide(6000);
    CHECK(d.evidence_gen == g1, "decision cites the current generation");
  }

  /* Edge — deterministic tie-break: two identically-clean candidates rank by
   * ascending RF key, stably across runs. */
  {
    auto run = []() {
      PolicyConfig p;
      RecommendEngine e(p, parse("36,149"), kPlan,
                        ChannelDef{5, 60, CHANNEL_WIDTH_20, 0});
      feed_bin(e, 36, 1000, 0.0);
      feed_bin(e, 149, 1000, 0.0);
      feed_active(e, LinkVerdict::Marginal, true);
      return e.decide(6000).target.primary;
    };
    CHECK(run() == run(), "tie-break deterministic");
    CHECK(run() == 36, "tie-break by ascending key");
  }

  /* Edge — improvement margin: a barely-clean candidate (just under the
   * occupancy floor) holds rather than triggering a whole-link move. */
  {
    PolicyConfig p;
    p.qualify_max_occupancy = 0.20; /* qualify floor = score 0.80 */
    p.improvement_margin = 0.05;    /* recommend needs score >= 0.85 */
    RecommendEngine e(p, parse("36"), kPlan,
                      ChannelDef{5, 60, CHANNEL_WIDTH_20, 0});
    feed_bin(e, 36, 1000, 0.18); /* qualifies (occ 0.18<=0.20) but score ~0.82 */
    feed_active(e, LinkVerdict::Marginal, true);
    Decision d = e.decide(6000);
    CHECK(d.primary_reason == Reason::HoldImprovementMargin,
          "barely-clean candidate held by the margin");
  }

  /* Edge — policy-file parse + hash stability. */
  {
    /* A relative path in the CWD (the build dir under ctest) — portable, unlike
     * a hardcoded /tmp which doesn't exist on Windows. */
    const char *path = "chanmig_policy_selftest.tmp";
    std::FILE *f = std::fopen(path, "w");
    CHECK(f != nullptr, "policy scratch file opens");
    if (f == nullptr)
      return fails ? 1 : 0;
    std::fprintf(f, "# test policy\nimpaired_windows_min 8\n"
                    "min_rounds 5\nimprovement_margin 0.4\n"
                    "qualify_max_occupancy 0.1\n");
    std::fclose(f);
    PolicyConfig a;
    std::string err;
    CHECK(parse_policy_file(path, a, &err), "policy file parses");
    CHECK(a.impaired_windows_min == 8 && a.min_rounds == 5, "values loaded");
    CHECK(a.improvement_margin == 0.4 && a.qualify_max_occupancy == 0.1,
          "float values loaded");
    PolicyConfig b;
    parse_policy_file(path, b, nullptr);
    CHECK(a.policy_hash() == b.policy_hash(), "policy hash stable");
    PolicyConfig def;
    CHECK(a.policy_hash() != def.policy_hash(), "hash sensitive to values");
    PolicyConfig missing;
    CHECK(!parse_policy_file("/nonexistent/policy", missing, &err),
          "missing file reported");
    std::remove(path);
  }

  return fails ? 1 : 0;
}
