/* The receiver-side exclusion/restoration policy over its full decision
 * matrix: thresholds, persistence hysteresis, the update cooldown, broad
 * degradation, the diversity floor and excluded-fraction cap, the
 * probe-driven restoration cycle, the adversarial rows (a flapping jammer
 * that must never accumulate persistence, a herding jammer that must hit the
 * floor instead of walking the link down), the classify-never-triggers rule,
 * and determinism + policy-hash stability. */
#include "hopset/HopsetPolicy.h"
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

/* Feed `rounds` rounds over the active set of `mask`, giving channel
 * `bad_index` the delivery `bad` and everyone else `good`. Returns the round
 * index just past the feed. */
uint64_t feed(HopsetPolicy &p, uint64_t mask, size_t n, uint64_t from_round,
              uint32_t rounds, int bad_index, double bad, double good) {
  uint64_t r = from_round;
  for (uint32_t i = 0; i < rounds; ++i, ++r) {
    for (size_t c = 0; c < n; ++c) {
      if (!(mask & (uint64_t(1) << c)))
        continue;
      SlotObservation o;
      o.round = r;
      o.slot = r * 64 + c;
      o.base_index = static_cast<uint32_t>(c);
      const double d =
          (bad_index >= 0 && c == static_cast<size_t>(bad_index)) ? bad : good;
      /* express the ratio through frames/crc so the delivery math is
       * exercised, not just the decoded flag */
      o.decoded = d > 0.0;
      o.frames = 10;
      o.crc_errs = static_cast<uint32_t>(10.0 * (1.0 - d) + 0.5);
      if (!o.decoded) {
        o.frames = 0;
        o.crc_errs = 0;
      }
      p.ingest(o);
    }
  }
  return r;
}

void probe(HopsetPolicy &p, uint64_t round, size_t index, double delivery) {
  SlotObservation o;
  o.round = round;
  o.slot = round * 64 + index;
  o.base_index = static_cast<uint32_t>(index);
  o.is_probe = true;
  o.decoded = delivery > 0.0;
  o.frames = delivery > 0.0 ? 10 : 0;
  o.crc_errs = delivery > 0.0
                   ? static_cast<uint32_t>(10.0 * (1.0 - delivery) + 0.5)
                   : 0;
  p.ingest(o);
}

} // namespace

int main() {
  const HopsetPolicyConfig def;

  /* --- policy hash: deterministic, field-sensitive --- */
  {
    HopsetPolicyConfig a, b;
    CHECK(a.policy_hash() == b.policy_hash(), "policy hash stable");
    b.impaired_rounds = 6;
    CHECK(a.policy_hash() != b.policy_hash(), "policy hash tracks fields");
    HopsetPolicyConfig c;
    c.exclude_delivery = 0.31;
    CHECK(a.policy_hash() != c.policy_hash(), "policy hash tracks doubles");
  }

  /* --- insufficient rounds: no decision before the observation bar --- */
  {
    HopsetPolicy p(def, 6);
    uint64_t r = feed(p, full_mask(6), 6, 0, 4, 2, 0.0, 1.0);
    auto d = p.decide(r);
    CHECK(d.kind == HopsetDecision::Kind::Hold &&
              d.hold == HopsetHold::InsufficientRounds,
          "holds below min_obs_rounds");
  }

  /* --- the canonical exclusion: one channel dead, others healthy --- */
  {
    HopsetPolicy p(def, 6);
    uint64_t r = feed(p, full_mask(6), 6, 0, 12, 2, 0.0, 1.0);
    auto d = p.decide(r);
    CHECK(d.kind == HopsetDecision::Kind::ProposeExclude, "proposes exclusion");
    CHECK(d.target_index == 2, "targets the impaired channel");
    CHECK(d.proposed_mask == (full_mask(6) & ~(uint64_t(1) << 2)),
          "mask drops exactly one bit");
    CHECK((d.reason_bitmap &
           (RB_DELIVERY_FLOOR | RB_PERSISTENT | RB_HEALTHY_ALT)) ==
              (RB_DELIVERY_FLOOR | RB_PERSISTENT | RB_HEALTHY_ALT),
          "evidence bits set");
    CHECK((d.reason_bitmap & RB_SYNC_LOSS) != 0,
          "classification: nothing decoded = sync loss");
    CHECK(d.observation_count >= 12, "observation count reported");
    CHECK(d.chans.size() == 6 && d.chans[2].impaired_run >= 5 &&
              d.chans[0].delivery > 0.9,
          "per-channel evidence summary");
    /* a second decide while the proposal is outstanding must hold */
    auto d2 = p.decide(r);
    CHECK(d2.kind == HopsetDecision::Kind::Hold &&
              d2.hold == HopsetHold::ProposalOutstanding,
          "one proposal at a time");
  }

  /* --- threshold edge: 0.29 delivery excludes, 0.31 does not --- */
  {
    HopsetPolicy p(def, 6);
    uint64_t r = feed(p, full_mask(6), 6, 0, 14, 2, 0.30, 1.0); /* 3/10 good */
    auto d = p.decide(r);
    CHECK(d.kind == HopsetDecision::Kind::Hold, "delivery at the floor holds");
    HopsetPolicy q(def, 6);
    r = feed(q, full_mask(6), 6, 0, 14, 2, 0.20, 1.0);
    CHECK(q.decide(r).kind == HopsetDecision::Kind::ProposeExclude,
          "delivery below the floor excludes");
  }

  /* --- persistence hysteresis: 4 impaired rounds is not enough --- */
  {
    HopsetPolicy p(def, 6);
    uint64_t r = feed(p, full_mask(6), 6, 0, 10, -1, 0.0, 1.0);
    r = feed(p, full_mask(6), 6, r, 4, 2, 0.0, 1.0);
    auto d = p.decide(r);
    CHECK(d.kind == HopsetDecision::Kind::Hold &&
              d.hold == HopsetHold::NotPersistent,
          "4 impaired rounds hold as not-persistent");
    r = feed(p, full_mask(6), 6, r, 1, 2, 0.0, 1.0);
    CHECK(p.decide(r).kind == HopsetDecision::Kind::ProposeExclude,
          "the 5th impaired round trips it");
  }

  /* --- flapping jammer: alternating rounds never accumulate persistence --- */
  {
    HopsetPolicy p(def, 6);
    uint64_t r = 0;
    for (int i = 0; i < 40; ++i)
      r = feed(p, full_mask(6), 6, r, 1, 2, (i & 1) ? 1.0 : 0.0, 1.0);
    auto d = p.decide(r);
    CHECK(d.kind == HopsetDecision::Kind::Hold,
          "a flapping channel is never excluded");
  }

  /* --- broad degradation: everything bad excludes nothing --- */
  {
    HopsetPolicy p(def, 6);
    uint64_t r = feed(p, full_mask(6), 6, 0, 14, -1, 0.0, 0.0);
    auto d = p.decide(r);
    CHECK(d.kind == HopsetDecision::Kind::Hold &&
              d.hold == HopsetHold::BroadDegradation,
          "broad degradation stops exclusion");
  }

  /* --- no healthy alternative: the others are merely mediocre --- */
  {
    HopsetPolicyConfig cfg = def;
    cfg.broad_degrade_frac = 1.01; /* isolate the healthy-alt gate */
    HopsetPolicy p(cfg, 6);
    uint64_t r = feed(p, full_mask(6), 6, 0, 14, 2, 0.0, 0.5);
    auto d = p.decide(r);
    CHECK(d.kind == HopsetDecision::Kind::Hold &&
              d.hold == HopsetHold::NoHealthyAlternative,
          "no healthy alternative holds");
  }

  /* --- diversity floor: never below max(3, configured) --- */
  {
    HopsetPolicy p(def, 4);
    const uint64_t m = 0b0111; /* one already excluded, 3 active */
    p.on_activation(m, 0);
    uint64_t r = feed(p, m, 4, 1, 20, 1, 0.0, 1.0);
    auto d = p.decide(r);
    CHECK(d.kind == HopsetDecision::Kind::Hold &&
              d.hold == HopsetHold::MinActiveFloor,
          "the hard active floor holds at 3");
  }

  /* --- excluded-fraction cap --- */
  {
    HopsetPolicyConfig cfg = def;
    cfg.min_active = 1; /* the floor is still max(3, ...) — cap must bind */
    cfg.max_excluded_frac = 0.10; /* 1-of-8 = 0.125 already exceeds it */
    HopsetPolicy p(cfg, 8);
    uint64_t r = feed(p, full_mask(8), 8, 0, 14, 2, 0.0, 1.0);
    auto d = p.decide(r);
    CHECK(d.kind == HopsetDecision::Kind::Hold &&
              d.hold == HopsetHold::MaxExcludedFrac,
          "excluded-fraction cap holds");
  }

  /* --- cooldown latch between updates --- */
  {
    HopsetPolicy p(def, 6);
    uint64_t r = feed(p, full_mask(6), 6, 0, 12, 2, 0.0, 1.0);
    auto d = p.decide(r);
    CHECK(d.kind == HopsetDecision::Kind::ProposeExclude, "cooldown setup");
    p.on_activation(d.proposed_mask, r);
    r = feed(p, d.proposed_mask, 6, r, 8, 4, 0.0, 1.0);
    auto d2 = p.decide(r);
    CHECK(d2.kind == HopsetDecision::Kind::Hold &&
              d2.hold == HopsetHold::Cooldown,
          "second exclusion inside the cooldown holds");
    r = feed(p, d.proposed_mask, 6, r, 4, 4, 0.0, 1.0);
    auto d3 = p.decide(r);
    CHECK(d3.kind == HopsetDecision::Kind::ProposeExclude &&
              d3.target_index == 4,
          "past the cooldown a second exclusion is allowed");
  }

  /* --- herding jammer: the interferer moves to each newly-active channel;
   * the policy must ratchet down to the floor and stop, never oscillate --- */
  {
    HopsetPolicy p(def, 6);
    uint64_t mask = full_mask(6), r = 0;
    int excluded = 0;
    for (int step = 0; step < 12; ++step) {
      /* the jammer sits on the lowest active channel each step */
      int victim = -1;
      for (size_t i = 0; i < 6 && victim < 0; ++i)
        if (mask & (uint64_t(1) << i))
          victim = static_cast<int>(i);
      r = feed(p, mask, 6, r, 14, victim, 0.0, 1.0);
      auto d = p.decide(r);
      if (d.kind == HopsetDecision::Kind::ProposeExclude) {
        ++excluded;
        mask = d.proposed_mask;
        p.on_activation(mask, r);
      } else {
        p.clear_outstanding(r);
      }
      CHECK(popcount64(mask) >= 3, "herding never breaches the floor");
    }
    CHECK(popcount64(mask) == 3, "herding converges to the floor");
    CHECK(excluded == 3, "exactly one exclusion per update, then it stops");
    auto d = p.decide(r + 100);
    CHECK(d.kind == HopsetDecision::Kind::Hold &&
              (d.hold == HopsetHold::MinActiveFloor ||
               d.hold == HopsetHold::BroadDegradation),
          "at the floor the policy refuses further exclusions");
  }

  /* --- classification never triggers: bad energy, fine delivery --- */
  {
    HopsetPolicy p(def, 6);
    uint64_t r = 0;
    for (int i = 0; i < 14; ++i, ++r)
      for (size_t c = 0; c < 6; ++c) {
        SlotObservation o;
        o.round = r;
        o.base_index = static_cast<uint32_t>(c);
        o.decoded = true;
        o.frames = 10;
        o.crc_errs = 0;
        o.verdict = 2; /* Interference on every channel */
        o.rssi = 20;
        p.ingest(o);
      }
    auto d = p.decide(r);
    CHECK(d.kind == HopsetDecision::Kind::Hold &&
              d.hold == HopsetHold::NoImpairment,
          "energy verdicts alone never exclude");
  }

  /* --- restoration cycle: probes must clear the bar --- */
  {
    HopsetPolicy p(def, 6);
    const uint64_t m = full_mask(6) & ~(uint64_t(1) << 2);
    p.on_activation(m, 0);
    uint64_t r = feed(p, m, 6, 1, 20, -1, 0.0, 1.0);
    probe(p, r++, 2, 1.0);
    probe(p, r++, 2, 1.0);
    auto d = p.decide(r);
    CHECK(d.kind == HopsetDecision::Kind::Hold &&
              d.hold == HopsetHold::ProbeHysteresisPending,
          "two good probes are not enough");
    probe(p, r++, 2, 1.0);
    auto d2 = p.decide(r);
    CHECK(d2.kind == HopsetDecision::Kind::ProposeRestore &&
              d2.target_index == 2 && d2.proposed_mask == full_mask(6),
          "three good probes restore");
    CHECK((d2.reason_bitmap & (RB_PROBE_RECOVERY | RB_PROBE_EVIDENCE)) ==
              (RB_PROBE_RECOVERY | RB_PROBE_EVIDENCE),
          "restoration evidence bits");
  }

  /* --- a failed probe resets the restoration run --- */
  {
    HopsetPolicy p(def, 6);
    const uint64_t m = full_mask(6) & ~(uint64_t(1) << 2);
    p.on_activation(m, 0);
    uint64_t r = feed(p, m, 6, 1, 20, -1, 0.0, 1.0);
    probe(p, r++, 2, 1.0);
    probe(p, r++, 2, 1.0);
    probe(p, r++, 2, 0.0); /* still jammed */
    probe(p, r++, 2, 1.0);
    probe(p, r++, 2, 1.0);
    auto d = p.decide(r);
    CHECK(d.kind == HopsetDecision::Kind::Hold,
          "a failed probe resets the restoration run");
    probe(p, r++, 2, 1.0);
    CHECK(p.decide(r).kind == HopsetDecision::Kind::ProposeRestore,
          "the run completes after three consecutive good probes");
  }

  /* --- restoration is preferred over a fresh exclusion --- */
  {
    HopsetPolicy p(def, 6);
    const uint64_t m = full_mask(6) & ~(uint64_t(1) << 2);
    p.on_activation(m, 0);
    uint64_t r = feed(p, m, 6, 1, 20, 4, 0.0, 1.0); /* ch4 now bad */
    probe(p, r++, 2, 1.0);
    probe(p, r++, 2, 1.0);
    probe(p, r++, 2, 1.0);
    auto d = p.decide(r);
    CHECK(d.kind == HopsetDecision::Kind::ProposeRestore,
          "giving a channel back outranks taking one away");
  }

  /* --- determinism: the same feed twice yields the same decision stream --- */
  {
    std::vector<int> kinds[2];
    std::vector<uint32_t> holds[2];
    for (int pass = 0; pass < 2; ++pass) {
      HopsetPolicy p(def, 6);
      uint64_t mask = full_mask(6), r = 0;
      for (int step = 0; step < 20; ++step) {
        r = feed(p, mask, 6, r, 3, step % 5, step % 3 ? 0.0 : 1.0, 1.0);
        auto d = p.decide(r);
        kinds[pass].push_back(static_cast<int>(d.kind));
        holds[pass].push_back(static_cast<uint32_t>(d.hold));
        if (d.kind != HopsetDecision::Kind::Hold) {
          mask = d.proposed_mask;
          p.on_activation(mask, r);
        }
      }
    }
    CHECK(kinds[0] == kinds[1] && holds[0] == holds[1],
          "identical feeds decide identically");
  }

  return fails ? 1 : 0;
}
