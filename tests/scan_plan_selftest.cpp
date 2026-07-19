/* ScanScheduler policy selftest — synthetic clock, no hardware.
 *
 * Pins: the initial sweep covers every bin exactly once in plan order; the
 * revisit-priority invariant (backup bins visited more often, yet NO bin's
 * inter-visit gap ever exceeds its cadence bound — the starvation guard);
 * round advancement only on full coverage; failed-bin retry pushback; and
 * byte-for-byte determinism of the dwell sequence. */
#include "chanmig/ScanPlan.h"

#include <cstdio>
#include <map>
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

using devourer::chanmig::ChannelDef;
using devourer::chanmig::parse_scan_plan;
using devourer::chanmig::PlanParseError;
using devourer::chanmig::ScanPlanConfig;
using devourer::chanmig::ScanScheduler;

static ScanPlanConfig make_cfg(const char *plan) {
  ScanPlanConfig cfg;
  std::vector<PlanParseError> errs;
  parse_scan_plan(plan, cfg.candidates, errs);
  return cfg;
}

int main() {
  /* Plan: one 20 MHz bg candidate, one 40 MHz backup (bins 44+48), one more
   * bg 20 MHz. Unique bins: 36, 44, 48, 149. */
  ScanPlanConfig cfg = make_cfg("36,44/40:b,149");
  cfg.dwell_ms = 100;
  cfg.backup_revisit_ms = 1000;
  cfg.bg_revisit_ms = 5000;
  cfg.fail_retry_ms = 250;

  {
    ScanScheduler s(cfg);
    CHECK(s.bin_count() == 4, "4 unique bins");

    /* Initial sweep: plan order, each bin exactly once, then round 1. */
    const uint8_t want[4] = {36, 44, 48, 149};
    int64_t now = 0;
    for (int i = 0; i < 4; i++) {
      auto p = s.next(now);
      CHECK(p.valid && !p.full_width, "initial sweep dwell valid");
      CHECK(p.bin_ch == want[i], "initial sweep in plan order");
      CHECK(p.def.width == CHANNEL_WIDTH_20, "bin dwells are 20 MHz");
      CHECK(p.round == 0, "round 0 during first sweep");
      s.complete(p, now, true);
      now += 100;
    }
    CHECK(s.rounds_complete() == 1, "round advances on full coverage");

    /* Steady state over 30 simulated seconds: backup bins dominate, but no
     * bin's inter-visit gap exceeds its cadence bound (+ one dwell slack). */
    std::map<int, int> visits;
    std::map<int, int64_t> last_seen;
    std::map<int, int64_t> worst_gap;
    for (uint8_t b : want) {
      last_seen[b] = now;
      worst_gap[b] = 0;
    }
    for (int i = 0; i < 300; i++) {
      auto p = s.next(now);
      CHECK(p.valid, "steady-state dwell valid");
      ++visits[p.bin_ch];
      const int64_t gap = now - last_seen[p.bin_ch];
      if (gap > worst_gap[p.bin_ch])
        worst_gap[p.bin_ch] = gap;
      last_seen[p.bin_ch] = now;
      s.complete(p, now, true);
      now += 100;
    }
    CHECK(visits[44] > visits[36] * 2 && visits[48] > visits[149] * 2,
          "backup bins visited more often");
    CHECK(worst_gap[36] <= cfg.bg_revisit_ms + 200 &&
              worst_gap[149] <= cfg.bg_revisit_ms + 200,
          "bg bins never starve past their cadence");
    CHECK(worst_gap[44] <= cfg.backup_revisit_ms + 200 &&
              worst_gap[48] <= cfg.backup_revisit_ms + 200,
          "backup bins hold their cadence");
    CHECK(s.rounds_complete() >= 5, "rounds keep advancing in steady state");
  }

  /* Failed dwells: the scheduler continues with the remaining bins first
   * (never head-bangs a wedged bin while others wait), retries the failed
   * one at its pushback deadline, counts the streak, and blocks the round
   * until the failing bin finally succeeds. */
  {
    ScanScheduler s(cfg);
    int64_t now = 0;
    auto p = s.next(now);
    CHECK(p.bin_ch == 36, "first pick in plan order");
    s.complete(p, now, false);
    CHECK(s.consecutive_failures() == 1, "failure streak counts");
    now += 100;
    /* The other never-visited bins run before the failed bin's retry. */
    const uint8_t rest[3] = {44, 48, 149};
    for (uint8_t want_bin : rest) {
      auto q = s.next(now);
      CHECK(q.bin_ch == want_bin, "remaining bins continue first");
      s.complete(q, now, true);
      now += 100;
    }
    CHECK(s.rounds_complete() == 0, "round blocked by the failed bin");
    auto r = s.next(now);
    CHECK(r.bin_ch == 36, "failed bin retried at its pushback deadline");
    s.complete(r, now, false);
    CHECK(s.consecutive_failures() == 1, "streak counts across the sweep");
    now += cfg.fail_retry_ms;
    auto r2 = s.next(now);
    CHECK(r2.bin_ch == 36, "retry keeps priority once others are fresh");
    s.complete(r2, now, true);
    CHECK(s.consecutive_failures() == 0, "success clears the streak");
    CHECK(s.rounds_complete() == 1, "round completes only after recovery");
  }

  /* Determinism: two schedulers over the identical now-sequence produce the
   * identical dwell sequence. */
  {
    ScanScheduler a(cfg), b(cfg);
    int64_t now = 0;
    bool same = true;
    for (int i = 0; i < 500; i++) {
      auto pa = a.next(now);
      auto pb = b.next(now);
      if (pa.bin_ch != pb.bin_ch || pa.full_width != pb.full_width ||
          pa.round != pb.round)
        same = false;
      /* fail every 7th dwell in both */
      const bool ok = (i % 7) != 0;
      a.complete(pa, now, ok);
      b.complete(pb, now, ok);
      now += 100;
    }
    CHECK(same, "dwell sequence is deterministic");
  }

  /* Wide verification dwells: off by default; when armed they appear at the
   * cadence, carry the full candidate def, and never touch bin coverage. */
  {
    ScanPlanConfig wcfg = cfg;
    wcfg.fullwidth_ms = 1000;
    ScanScheduler s(wcfg);
    int64_t now = 0;
    int wide_seen = 0;
    uint64_t rounds_at_first_wide = 0;
    for (int i = 0; i < 100; i++) {
      auto p = s.next(now);
      if (p.full_width) {
        ++wide_seen;
        CHECK(p.def.width == CHANNEL_WIDTH_40 && p.def.primary == 44,
              "wide dwell carries the candidate def");
        if (wide_seen == 1)
          rounds_at_first_wide = s.rounds_complete();
      }
      s.complete(p, now, true);
      now += 100;
    }
    CHECK(wide_seen >= 8 && wide_seen <= 11, "wide cadence honored");
    CHECK(rounds_at_first_wide >= 1, "first sweep ran before wide dwells");
    ScanScheduler off(cfg);
    int64_t n2 = 0;
    bool any_wide = false;
    for (int i = 0; i < 100; i++) {
      auto p = off.next(n2);
      any_wide = any_wide || p.full_width;
      off.complete(p, n2, true);
      n2 += 100;
    }
    CHECK(!any_wide, "no wide dwells unless armed");
  }

  /* Shared bins: a 20 MHz candidate inside a backup 40 MHz block inherits the
   * faster cadence (one physical bin, fastest containing candidate wins). */
  {
    ScanPlanConfig sh = make_cfg("44/40:b,48");
    sh.backup_revisit_ms = 1000;
    sh.bg_revisit_ms = 5000;
    ScanScheduler s(sh);
    CHECK(s.bin_count() == 2, "shared bin not duplicated");
    int64_t now = 0;
    std::map<int, int> visits;
    for (int i = 0; i < 100; i++) {
      auto p = s.next(now);
      ++visits[p.bin_ch];
      s.complete(p, now, true);
      now += 100;
    }
    CHECK(visits[48] >= visits[44] - 5 && visits[44] >= visits[48] - 5,
          "shared bin rides the backup cadence");
  }

  /* plan_hash: stable across runs, sensitive to candidates and cadences. */
  {
    ScanPlanConfig a = make_cfg("36,44/40:b");
    ScanPlanConfig b = make_cfg("36,44/40:b");
    CHECK(a.plan_hash() == b.plan_hash(), "plan hash stable");
    ScanPlanConfig c = make_cfg("36,44/40");
    CHECK(a.plan_hash() != c.plan_hash(), "flags change the hash");
    ScanPlanConfig d = make_cfg("36,44/40:b");
    d.bg_revisit_ms += 1;
    CHECK(a.plan_hash() != d.plan_hash(), "cadence changes the hash");
  }

  return fails ? 1 : 0;
}
