/* The transmitter-side evidence classifier over its decision matrix: scoring
 * and admission, the deliberately-stricter thresholds, hysteresis, cooldown,
 * broad/flat band detection, the two structural rules (an absent sensor must
 * never read as a clean channel; pre- and post-burst populations are never
 * pooled), the anti-herding floors, and determinism. */
#include "hopset/HopsetSense.h"
#include <cstdio>
#include <vector>

static int fails;
#define CHECK(x, msg)                                                          \
  do {                                                                         \
    if (!(x)) {                                                                \
      std::fprintf(stderr, "FAIL: %s\n", msg);                                 \
      ++fails;                                                                 \
    }                                                                          \
  } while (0)

using namespace devourer::hopset;

namespace {

constexpr uint32_t kWin = 4000; /* 4 ms windows, as the demo uses */

/* Build a sample whose score lands on `occ`, expressed through the CCA
 * counter so the rate normalization is exercised rather than bypassed. With
 * only CCA+FA valid, w_cca and w_fa both contribute; drive both to `occ` so
 * the weighted mean is exactly `occ`. */
TxSenseSample mk(uint32_t idx, uint64_t round, double occ, SensePhase phase,
                 bool probe = false) {
  TxSenseConfig cfg;
  TxSenseSample s;
  s.base_index = idx;
  s.round = round;
  s.slot = round * 16 + idx;
  s.phase = phase;
  s.window_us = kWin;
  s.probe = probe;
  s.valid_fa = true;
  const double win_ms = kWin / 1000.0;
  s.cca_ofdm = static_cast<uint32_t>(occ * cfg.cca_rate_sat * win_ms + 0.5);
  s.fa_ofdm = static_cast<uint32_t>(occ * cfg.fa_rate_sat * win_ms + 0.5);
  return s;
}

/* Feed `rounds` rounds over the active set, giving `dirty_index` occupancy
 * `dirty` and everyone else `clean`. */
uint64_t feed(HopsetSensor &s, uint64_t mask, size_t n, uint64_t from,
              uint32_t rounds, int dirty_index, double dirty, double clean,
              SensePhase phase = SensePhase::PostBurst) {
  uint64_t r = from;
  for (uint32_t i = 0; i < rounds; ++i, ++r)
    for (size_t c = 0; c < n; ++c) {
      if (!(mask & (uint64_t(1) << c)))
        continue;
      const double o =
          (dirty_index >= 0 && c == size_t(dirty_index)) ? dirty : clean;
      s.ingest(mk(uint32_t(c), r, o, phase));
    }
  return r;
}

/* Both phases, same occupancy — the ordinary case. */
uint64_t feed_both(HopsetSensor &s, uint64_t mask, size_t n, uint64_t from,
                   uint32_t rounds, int dirty_index, double dirty,
                   double clean) {
  uint64_t r = from;
  for (uint32_t i = 0; i < rounds; ++i, ++r)
    for (size_t c = 0; c < n; ++c) {
      if (!(mask & (uint64_t(1) << c)))
        continue;
      const double o =
          (dirty_index >= 0 && c == size_t(dirty_index)) ? dirty : clean;
      s.ingest(mk(uint32_t(c), r, o, SensePhase::PreBurst));
      s.ingest(mk(uint32_t(c), r, o, SensePhase::PostBurst));
    }
  return r;
}

/* The synthetic-evidence lever: the same sample with the occupancy handed in
 * directly instead of derived from counters. */
TxSenseSample mk_inj(uint32_t idx, uint64_t round, double occ, SensePhase phase,
                     bool probe = false) {
  TxSenseSample s;
  s.base_index = idx;
  s.round = round;
  s.slot = round * 16 + idx;
  s.phase = phase;
  s.window_us = kWin;
  s.probe = probe;
  s.injected = true;
  s.injected_occupancy = occ;
  s.flags |= kTsInjected;
  return s;
}

uint64_t feed_both_inj(HopsetSensor &s, uint64_t mask, size_t n, uint64_t from,
                       uint32_t rounds, int dirty_index, double dirty,
                       double clean) {
  uint64_t r = from;
  for (uint32_t i = 0; i < rounds; ++i, ++r)
    for (size_t c = 0; c < n; ++c) {
      if (!(mask & (uint64_t(1) << c)))
        continue;
      const double o =
          (dirty_index >= 0 && c == size_t(dirty_index)) ? dirty : clean;
      s.ingest(mk_inj(uint32_t(c), r, o, SensePhase::PreBurst));
      s.ingest(mk_inj(uint32_t(c), r, o, SensePhase::PostBurst));
    }
  return r;
}

} // namespace

int main() {
  const TxSenseConfig def;

  /* --- the config hash --- */
  {
    TxSenseConfig a, b, c;
    CHECK(a.policy_hash() == b.policy_hash(), "sense hash stable");
    b.dirty_run = 11;
    CHECK(a.policy_hash() != b.policy_hash(), "sense hash tracks fields");
    c.occupancy_dirty = 0.61;
    CHECK(a.policy_hash() != c.policy_hash(), "sense hash tracks doubles");
  }

  /* --- the TX layer must be strictly more reluctant than the RX layer --- */
  {
    const HopsetPolicyConfig rx;
    CHECK(def.cooldown_rounds >= 4 * rx.update_cooldown_rounds,
          "TX cooldown is at least 4x the RX cooldown");
    CHECK(def.dirty_run > rx.impaired_rounds, "TX hysteresis is longer");
    CHECK(def.min_obs_rounds > rx.min_obs_rounds, "TX needs more rounds");
    CHECK(def.min_cleaner_chans >= 2,
          "TX needs more than one cleaner alternative");
  }

  /* --- scoring --- */
  {
    SenseScore sc;
    CHECK(score_sample(mk(0, 0, 0.75, SensePhase::PostBurst), def, sc) &&
              sc.occupancy > 0.70 && sc.occupancy < 0.80,
          "score round-trips through the counters");
    CHECK((sc.sources & kSrcCca) && (sc.sources & kSrcFa), "sources reported");
    /* no valid source at all: must REFUSE to score, not report 0.0 */
    TxSenseSample none;
    none.window_us = kWin;
    CHECK(!score_sample(none, def, sc), "a sourceless sample cannot be scored");
    /* a railed IGI contributes nothing at either rail */
    TxSenseSample lo = mk(0, 0, 0.5, SensePhase::PostBurst);
    lo.valid_igi = true;
    lo.igi = def.igi_floor;
    SenseScore a, b;
    score_sample(lo, def, a);
    TxSenseSample hi = lo;
    hi.igi = def.igi_ceiling;
    score_sample(hi, def, b);
    CHECK(a.occupancy == b.occupancy && !(a.sources & kSrcIgi),
          "railed IGI contributes nothing at either rail");
  }

  /* --- holds below the observation bar --- */
  {
    HopsetSensor s(def, 6);
    uint64_t r = feed(s, full_mask(6), 6, 0, 10, 2, 0.85, 0.10);
    CHECK(s.evaluate(r).hold == TxSenseHold::InsufficientRounds,
          "holds below min_obs_rounds");
  }

  /* --- the canonical TX exclusion --- */
  {
    HopsetSensor s(def, 6);
    uint64_t r = feed(s, full_mask(6), 6, 0, 30, 2, 0.85, 0.10);
    auto d = s.evaluate(r);
    CHECK(d.kind == TxSenseCandidate::Kind::ProposeExclude, "proposes");
    CHECK(d.target_index == 2, "targets the occupied channel");
    CHECK(d.proposed_mask == (full_mask(6) & ~(uint64_t(1) << 2)),
          "drops exactly one bit");
    CHECK((d.reason_bitmap & RB_TX_CCA_HIGH) &&
              (d.reason_bitmap & RB_TX_POST_BURST) &&
              (d.reason_bitmap & RB_TX_CLEANER_ALT),
          "evidence bits set");
    CHECK(d.chans.size() == 6 && d.chans[2].post.occupancy > 0.8 &&
              d.chans[0].post.occupancy < 0.2,
          "per-channel evidence summary");
    CHECK(s.evaluate(r).hold == TxSenseHold::ProposalOutstanding,
          "one proposal at a time");
  }

  /* --- occupancy threshold edge --- */
  {
    HopsetSensor s(def, 6);
    uint64_t r = feed(s, full_mask(6), 6, 0, 30, 2, 0.55, 0.10);
    CHECK(s.evaluate(r).kind == TxSenseCandidate::Kind::None,
          "below the dirty threshold holds");
    HopsetSensor t(def, 6);
    r = feed(t, full_mask(6), 6, 0, 30, 2, 0.65, 0.10);
    CHECK(t.evaluate(r).kind == TxSenseCandidate::Kind::ProposeExclude,
          "above the dirty threshold proposes");
  }

  /* --- hysteresis: a run shorter than dirty_run is not persistent --- */
  {
    HopsetSensor s(def, 6);
    uint64_t r = feed(s, full_mask(6), 6, 0, 26, -1, 0.0, 0.10);
    r = feed(s, full_mask(6), 6, r, def.dirty_run - 1, 2, 0.85, 0.10);
    CHECK(s.evaluate(r).hold == TxSenseHold::NotPersistent,
          "a short dirty run holds");
    r = feed(s, full_mask(6), 6, r, 1, 2, 0.85, 0.10);
    CHECK(s.evaluate(r).kind == TxSenseCandidate::Kind::ProposeExclude,
          "the run completing trips it");
  }

  /* --- a flapping channel never accumulates a run --- */
  {
    HopsetSensor s(def, 6);
    uint64_t r = 0;
    for (int i = 0; i < 60; ++i)
      r = feed(s, full_mask(6), 6, r, 1, 2, (i & 1) ? 0.85 : 0.05, 0.10);
    CHECK(s.evaluate(r).kind == TxSenseCandidate::Kind::None,
          "a flapping channel is never excluded");
  }

  /* --- broad occupancy: the whole band is busy, so it is not a channel
   * fault. Spread is deliberately wide so FlatBand cannot mask the row. --- */
  {
    HopsetSensor s(def, 6);
    uint64_t r = 0;
    for (uint32_t i = 0; i < 30; ++i, ++r)
      for (size_t c = 0; c < 6; ++c)
        s.ingest(mk(uint32_t(c), r, c == 2 ? 0.98 : 0.66,
                    SensePhase::PostBurst));
    CHECK(s.evaluate(r).hold == TxSenseHold::BroadOccupancy,
          "broad occupancy stops exclusion");
  }

  /* --- flat band: the channels are indistinguishable, so attribution is
   * doubtful even though one has a long run --- */
  {
    TxSenseConfig cfg = def;
    cfg.broad_occupancy_frac = 1.01; /* isolate the spread check */
    HopsetSensor s(cfg, 6);
    uint64_t r = 0;
    for (uint32_t i = 0; i < 30; ++i, ++r)
      for (size_t c = 0; c < 6; ++c)
        s.ingest(mk(uint32_t(c), r, c == 2 ? 0.70 : 0.62,
                    SensePhase::PostBurst));
    CHECK(s.evaluate(r).hold == TxSenseHold::FlatBand,
          "a flat band stops exclusion");
    CHECK(s.evaluate(r + 1).flat_band, "flat_band reported as a band fact");
  }

  /* --- flat AND clean is not a flat-band hold: nothing was dirty, so the
   * ladder never reaches the spread check --- */
  {
    HopsetSensor s(def, 6);
    uint64_t r = feed(s, full_mask(6), 6, 0, 30, -1, 0.0, 0.05);
    auto d = s.evaluate(r);
    CHECK(d.hold == TxSenseHold::NoOccupancy,
          "a uniformly clean band holds as no-occupancy, not flat-band");
    CHECK(d.flat_band && d.band_max_occ < 0.2,
          "geometry still reports flat, with a low ceiling for fusion");
  }

  /* --- cleaner alternatives: one is not enough --- */
  {
    HopsetSensor s(def, 6);
    uint64_t r = 0;
    for (uint32_t i = 0; i < 30; ++i, ++r)
      for (size_t c = 0; c < 6; ++c) {
        double o = 0.58; /* not dirty, and not cleaner by the margin */
        if (c == 2)
          o = 0.85;
        else if (c == 0)
          o = 0.05;
        s.ingest(mk(uint32_t(c), r, o, SensePhase::PostBurst));
      }
    CHECK(s.evaluate(r).hold == TxSenseHold::NoCleanerAlternative,
          "a single cleaner alternative is not enough");
    HopsetSensor t(def, 6);
    r = 0;
    for (uint32_t i = 0; i < 30; ++i, ++r)
      for (size_t c = 0; c < 6; ++c) {
        double o = 0.58;
        if (c == 2)
          o = 0.85;
        else if (c == 0 || c == 4)
          o = 0.05;
        t.ingest(mk(uint32_t(c), r, o, SensePhase::PostBurst));
      }
    CHECK(t.evaluate(r).kind == TxSenseCandidate::Kind::ProposeExclude,
          "two cleaner alternatives suffice");
  }

  /* --- cooldown --- */
  {
    HopsetSensor s(def, 6);
    uint64_t r = feed(s, full_mask(6), 6, 0, 30, 2, 0.85, 0.10);
    auto d = s.evaluate(r);
    CHECK(d.kind == TxSenseCandidate::Kind::ProposeExclude, "cooldown setup");
    s.on_activation(d.proposed_mask, r);
    r = feed(s, d.proposed_mask, 6, r, 20, 4, 0.85, 0.10);
    CHECK(s.evaluate(r).hold == TxSenseHold::Cooldown,
          "a second exclusion inside the cooldown holds");
    r = feed(s, d.proposed_mask, 6, r, 25, 4, 0.85, 0.10);
    CHECK(s.evaluate(r).kind == TxSenseCandidate::Kind::ProposeExclude,
          "past the cooldown it may act again");
  }

  /* --- THE SENSOR-ABSENT RULE: no facility must never read as clean --- */
  {
    HopsetSensor s(def, 6);
    uint64_t r = 0;
    for (uint32_t i = 0; i < 40; ++i, ++r)
      for (size_t c = 0; c < 6; ++c) {
        TxSenseSample x; /* no valid_fa, no valid_igi, no valid_nhm */
        x.base_index = uint32_t(c);
        x.round = r;
        x.window_us = kWin;
        s.ingest(x);
      }
    auto d = s.evaluate(r);
    CHECK(d.hold == TxSenseHold::SensorAbsent, "absent sensor holds");
    CHECK(d.chans_with_evidence == 0, "no channel claims evidence");
    for (size_t c = 0; c < 6; ++c) {
      CHECK(d.chans[c].sensor_absent, "each channel marked sensor-absent");
      CHECK(!d.chans[c].have_evidence(),
            "an absent sensor is not evidence of a clean channel");
      CHECK(d.chans[c].pre.dropped + d.chans[c].post.dropped > 0,
            "the unscorable samples were counted, not silently lost");
    }
  }

  /* --- a partial sensor still works (NHM only, no FA/IGI) --- */
  {
    HopsetSensor s(def, 6);
    uint64_t r = 0;
    for (uint32_t i = 0; i < 30; ++i, ++r)
      for (size_t c = 0; c < 6; ++c) {
        TxSenseSample x;
        x.base_index = uint32_t(c);
        x.round = r;
        x.window_us = kWin;
        x.valid_nhm = true;
        x.nhm_duration = 500;
        x.nhm_busy_pct = c == 2 ? 90 : 5;
        x.phase = SensePhase::PostBurst;
        s.ingest(x);
      }
    auto d = s.evaluate(r);
    CHECK(d.kind == TxSenseCandidate::Kind::ProposeExclude &&
              d.target_index == 2,
          "an NHM-only sensor still classifies");
    CHECK((d.reason_bitmap & RB_TX_NHM_BUSY) &&
              !(d.reason_bitmap & RB_TX_CCA_HIGH),
          "reasons name the source that actually spoke");
  }

  /* --- admission: every inadmissible sample is dropped and counted --- */
  {
    struct Row {
      const char *name;
      uint16_t flags;
      uint32_t window_us;
      uint32_t cca_override;
    };
    const Row rows[] = {
        {"counter-suspect flag", kTsCounterSuspect, kWin, 0},
        {"barrier missed", kTsBarrierMissed, kWin, 0},
        {"read failed", kTsReadFailed, kWin, 0},
        {"truncated", kTsTruncated, kWin, 0},
        {"short window", 0, 400, 0},
        {"implausible delta", 0, kWin, 5000000},
    };
    for (const Row &row : rows) {
      HopsetSensor s(def, 6);
      uint64_t r = 0;
      for (uint32_t i = 0; i < 40; ++i, ++r)
        for (size_t c = 0; c < 6; ++c) {
          TxSenseSample x = mk(uint32_t(c), r, c == 2 ? 0.85 : 0.10,
                               SensePhase::PostBurst);
          if (c == 2) {
            x.flags = row.flags;
            x.window_us = row.window_us;
            if (row.cca_override)
              x.cca_ofdm = row.cca_override;
          }
          s.ingest(x);
        }
      auto d = s.evaluate(r);
      CHECK(d.kind == TxSenseCandidate::Kind::None, row.name);
      CHECK(d.chans[2].post.samples == 0 && d.chans[2].post.dropped > 0,
            "the inadmissible samples were dropped and counted");
    }
  }

  /* --- PRE AND POST ARE NEVER POOLED: the reactive-emitter signature.
   * Clean before our burst, dirty after it. Pooling would average to ~0.45
   * and never trip the 0.60 threshold — the adversary would be invisible. --- */
  {
    HopsetSensor s(def, 6);
    uint64_t r = 0;
    for (uint32_t i = 0; i < 30; ++i, ++r)
      for (size_t c = 0; c < 6; ++c) {
        s.ingest(mk(uint32_t(c), r, 0.05, SensePhase::PreBurst));
        s.ingest(mk(uint32_t(c), r, c == 2 ? 0.90 : 0.05,
                    SensePhase::PostBurst));
      }
    auto d = s.evaluate(r);
    CHECK(d.kind == TxSenseCandidate::Kind::ProposeExclude &&
              d.target_index == 2,
          "a post-burst-only emitter is detected");
    CHECK(d.chans[2].pre.occupancy < 0.2 && d.chans[2].post.occupancy > 0.8,
          "the two populations were kept apart");
    CHECK((d.reason_bitmap & RB_TX_POST_BURST) != 0,
          "the decision names post-burst evidence");
  }

  /* --- pre-burst alone cannot exclude --- */
  {
    HopsetSensor s(def, 6);
    uint64_t r = feed(s, full_mask(6), 6, 0, 30, 2, 0.85, 0.10,
                      SensePhase::PreBurst);
    CHECK(s.evaluate(r).hold == TxSenseHold::PostBurstMissing,
          "pre-burst evidence alone holds");
    TxSenseConfig cfg = def;
    cfg.require_post_burst = false;
    HopsetSensor t(cfg, 6);
    r = feed(t, full_mask(6), 6, 0, 30, 2, 0.85, 0.10, SensePhase::PreBurst);
    auto d = t.evaluate(r);
    CHECK(d.kind == TxSenseCandidate::Kind::ProposeExclude &&
              (d.reason_bitmap & RB_TX_PRE_BURST),
          "with the requirement relaxed, pre-burst may act");
  }

  /* --- probe-driven restoration: the TX's own recovery evidence --- */
  {
    HopsetSensor s(def, 6);
    const uint64_t m = full_mask(6) & ~(uint64_t(1) << 2);
    s.on_activation(m, 0);
    uint64_t r = feed(s, m, 6, 1, 45, -1, 0.0, 0.10);
    s.ingest(mk(2, r++, 0.05, SensePhase::PreBurst, /*probe=*/true));
    s.ingest(mk(2, r++, 0.05, SensePhase::PreBurst, true));
    CHECK(s.evaluate(r).kind == TxSenseCandidate::Kind::None,
          "two clean probes are not enough");
    s.ingest(mk(2, r++, 0.05, SensePhase::PreBurst, true));
    auto d = s.evaluate(r);
    CHECK(d.kind == TxSenseCandidate::Kind::ProposeRestore &&
              d.target_index == 2 && d.proposed_mask == full_mask(6),
          "three clean probes restore");
    /* a dirty probe resets the run */
    HopsetSensor t(def, 6);
    t.on_activation(m, 0);
    r = feed(t, m, 6, 1, 45, -1, 0.0, 0.10);
    t.ingest(mk(2, r++, 0.05, SensePhase::PreBurst, true));
    t.ingest(mk(2, r++, 0.90, SensePhase::PreBurst, true));
    t.ingest(mk(2, r++, 0.05, SensePhase::PreBurst, true));
    t.ingest(mk(2, r++, 0.05, SensePhase::PreBurst, true));
    CHECK(t.evaluate(r).kind == TxSenseCandidate::Kind::None,
          "a dirty probe resets the restoration run");
  }

  /* --- the anti-herding floors --- */
  {
    HopsetSensor s(def, 4);
    const uint64_t m = 0b0111;
    s.on_activation(m, 0);
    uint64_t r = feed(s, m, 4, 1, 45, 1, 0.85, 0.10);
    CHECK(s.evaluate(r).hold == TxSenseHold::MinActiveFloor,
          "the diversity floor holds at 3");
  }
  {
    TxSenseConfig cfg = def;
    cfg.min_active = 1;
    cfg.max_excluded_frac = 0.10;
    HopsetSensor s(cfg, 8);
    uint64_t r = feed(s, full_mask(8), 8, 0, 30, 2, 0.85, 0.10);
    CHECK(s.evaluate(r).hold == TxSenseHold::MaxExcludedFrac,
          "the excluded-fraction cap holds");
  }

  /* --- too few scored channels to compare against --- */
  {
    HopsetSensor s(def, 6);
    uint64_t r = 0;
    for (uint32_t i = 0; i < 30; ++i, ++r) {
      s.ingest(mk(2, r, 0.85, SensePhase::PostBurst));
      s.ingest(mk(0, r, 0.05, SensePhase::PostBurst));
    }
    CHECK(s.evaluate(r).hold == TxSenseHold::InsufficientChannels,
          "cannot claim 'others are cleaner' from two channels");
  }

  /* --- herding: the emitter follows every exclusion; the floor must hold --- */
  {
    HopsetSensor s(def, 6);
    uint64_t mask = full_mask(6), r = 0;
    int excluded = 0;
    for (int step = 0; step < 10; ++step) {
      int victim = -1;
      for (size_t i = 0; i < 6 && victim < 0; ++i)
        if (mask & (uint64_t(1) << i))
          victim = int(i);
      r = feed(s, mask, 6, r, 45, victim, 0.85, 0.10);
      auto d = s.evaluate(r);
      if (d.kind == TxSenseCandidate::Kind::ProposeExclude) {
        ++excluded;
        mask = d.proposed_mask;
        s.on_activation(mask, r);
      } else {
        s.clear_outstanding(r);
      }
      CHECK(popcount64(mask) >= 3, "herding never breaches the floor");
    }
    CHECK(popcount64(mask) == 3 && excluded == 3,
          "herding converges to the floor and stops");
  }

  /* --- synthetic evidence ---
   * The lever exists so a bench where both endpoints hear the same interferer
   * can still build a genuine disagreement. It must therefore be exact (an
   * approximate view proves nothing about the veto) and it must not be a way
   * around the floors: a fabricated occupancy earns no more authority than a
   * measured one. */
  {
    SenseScore sc;
    /* Counters present AND injection set: the injection wins outright, so a
     * caller cannot half-inject and wonder which reading it got. */
    TxSenseSample s = mk(0, 0, 0.90, SensePhase::PostBurst);
    s.injected = true;
    s.injected_occupancy = 0.05;
    CHECK(score_sample(s, def, sc), "injected sample scores");
    CHECK(sc.occupancy == 0.05, "injected occupancy is returned verbatim");
    CHECK(sc.sources == kSrcInjected, "injected evidence names itself");
    s.injected_occupancy = 4.0;
    score_sample(s, def, sc);
    CHECK(sc.occupancy == 1.0, "injected occupancy clamps high");
    s.injected_occupancy = -1.0;
    score_sample(s, def, sc);
    CHECK(sc.occupancy == 0.0, "injected occupancy clamps low");
    /* Inert when unset: the counter path is untouched. */
    TxSenseSample plain = mk(0, 0, 0.90, SensePhase::PostBurst);
    SenseScore pc;
    CHECK(score_sample(plain, def, pc) && pc.sources == (kSrcCca | kSrcFa),
          "an uninjected sample still scores from its counters");
  }
  {
    /* An injected stream reaches the same verdict as a measured one of equal
     * occupancy — the hidden-node rig's premise. */
    HopsetSensor inj(def, 6), meas(def, 6);
    feed_both_inj(inj, full_mask(6), 6, 0, 40, 2, 0.85, 0.10);
    feed_both(meas, full_mask(6), 6, 0, 40, 2, 0.85, 0.10);
    const auto di = inj.evaluate(40), dm = meas.evaluate(40);
    CHECK(di.kind == TxSenseCandidate::Kind::ProposeExclude &&
              di.target_index == 2,
          "injected dirt drives the exclusion candidate");
    CHECK(di.kind == dm.kind && di.target_index == dm.target_index &&
              di.proposed_mask == dm.proposed_mask,
          "injected and measured evidence classify alike");
  }
  {
    /* A fabricated view of a uniformly dirty band is still broad degradation,
     * not a licence to spend channels. */
    HopsetSensor s(def, 6);
    feed_both_inj(s, full_mask(6), 6, 0, 40, -1, 0.0, 0.85);
    const auto d = s.evaluate(40);
    CHECK(d.kind == TxSenseCandidate::Kind::None,
          "injected broad dirt proposes nothing");
    CHECK(d.hold == TxSenseHold::BroadOccupancy ||
              d.hold == TxSenseHold::FlatBand,
          "injected broad dirt is held on the band's geometry");
  }
  {
    /* And the diversity floor holds against it too. */
    HopsetSensor s(def, 6);
    uint64_t mask = full_mask(6), r = 0;
    for (int round = 0; round < 8; ++round) {
      int victim = -1;
      for (size_t i = 0; i < 6; ++i)
        if ((mask & (uint64_t(1) << i)) && victim < 0)
          victim = int(i);
      r = feed_both_inj(s, mask, 6, r, 45, victim, 0.85, 0.10);
      const auto d = s.evaluate(r);
      if (d.kind == TxSenseCandidate::Kind::ProposeExclude) {
        mask = d.proposed_mask;
        s.on_activation(mask, r);
      } else {
        s.clear_outstanding(r);
      }
      CHECK(popcount64(mask) >= 3, "injection never breaches the floor");
    }
  }

  /* --- determinism --- */
  {
    std::vector<int> kinds[2];
    std::vector<uint32_t> holds[2];
    for (int pass = 0; pass < 2; ++pass) {
      HopsetSensor s(def, 6);
      uint64_t mask = full_mask(6), r = 0;
      for (int step = 0; step < 12; ++step) {
        r = feed_both(s, mask, 6, r, 8, step % 5, step % 3 ? 0.85 : 0.05, 0.10);
        auto d = s.evaluate(r);
        kinds[pass].push_back(int(d.kind));
        holds[pass].push_back(uint32_t(d.hold));
        if (d.kind != TxSenseCandidate::Kind::None) {
          mask = d.proposed_mask;
          s.on_activation(mask, r);
        }
      }
    }
    CHECK(kinds[0] == kinds[1] && holds[0] == holds[1],
          "identical sample streams classify identically");
  }

  return fails ? 1 : 0;
}
