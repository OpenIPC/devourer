/* MigGate policy selftest — the #279 deterministic scenario matrix.
 *
 * Thirteen named scenarios drive the gate through the issue's cases; each is
 * a fixed input trace asserting the exact verdict + reason. Plus the
 * determinism guarantee: every scenario is run twice and must produce the
 * identical decision (no RNG, no clock reads). */
#include "chanmig/MigGate.h"

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

using namespace devourer::chanmig;

static ChannelDef ch(uint8_t p) {
  ChannelDef d;
  d.band = p <= 14 ? 2 : 5;
  d.primary = p;
  d.width = CHANNEL_WIDTH_20;
  normalize(d);
  return d;
}

/* Build a Recommend decision for `target` with a given confidence + score. */
static Decision recommend(const ChannelDef &target, double score, double conf,
                          uint64_t gen = 100) {
  Decision d;
  d.kind = Decision::Kind::Recommend;
  d.primary_reason = Reason::RecommendBetterCandidate;
  d.target = target;
  d.evidence_gen = gen;
  CandidateScore c;
  c.def = target;
  c.qualified = true;
  c.score = score;
  c.confidence = conf;
  d.ranking.push_back(c);
  return d;
}
static Decision hold_dec(Reason r) {
  Decision d;
  d.kind = Decision::Kind::Hold;
  d.primary_reason = r;
  return d;
}

static GateInputs automatic(const Decision *rec) {
  GateInputs in;
  in.mode = MigMode::Automatic;
  in.rec = rec;
  in.telemetry_ok = true;
  in.clock_synced = true;
  in.scout_healthy = true;
  in.rescue_verified = true;
  in.control_link_margin = 1.0;
  return in;
}

int main() {
  GatePolicy pol;
  const ChannelDef tgt = ch(36), other = ch(149);

  /* run each scenario twice; the second run must match (determinism). */
  auto twice = [&](const char *name, GateInputs in, GateState st,
                   int64_t now, GateVerdict v, GateReason r) {
    GateState s1 = st, s2 = st;
    GateOutcome o1 = mig_gate_decide(in, s1, pol, now);
    GateOutcome o2 = mig_gate_decide(in, s2, pol, now);
    CHECK(o1.verdict == v && o1.reason == r, name);
    CHECK(o1.verdict == o2.verdict && o1.reason == o2.reason,
          "gate is deterministic");
    if (o1.reason != r)
      std::fprintf(stderr, "  [%s] got %s\n", name, gate_reason_name(o1.reason));
  };

  /* 1. interferer appears: a fresh recommend with no cooldown -> Propose. */
  {
    Decision d = recommend(tgt, 1.0, 0.95);
    GateState st;
    twice("interferer_appears", automatic(&d), st, 10000,
          GateVerdict::Propose, GateReason::Propose);
  }
  /* 2. interferer persists: same recommend but a recent move -> Cooldown. */
  {
    Decision d = recommend(tgt, 1.0, 0.95);
    GateState st;
    st.last_move_ms = 10000;
    twice("interferer_persists", automatic(&d), st, 10000 + 60000,
          GateVerdict::Hold, GateReason::Cooldown);
  }
  /* 3. interferer leaves: the advisory now holds (active healthy). */
  {
    Decision d = hold_dec(Reason::HoldActiveHealthy);
    GateState st;
    twice("interferer_leaves", automatic(&d), st, 10000, GateVerdict::Hold,
          GateReason::NotImpaired);
  }
  /* 4. burst straddle: a low-confidence recommend -> ConfidenceLow. */
  {
    Decision d = recommend(tgt, 1.0, 0.6);
    GateState st;
    twice("burst_straddle", automatic(&d), st, 10000, GateVerdict::Hold,
          GateReason::ConfidenceLow);
  }
  /* 5. herding: a channel under holddown backoff -> HolddownChannel. */
  {
    Decision d = recommend(tgt, 1.0, 0.95);
    GateState st;
    st.set_holddown(tgt.key(), 500000);
    twice("herding", automatic(&d), st, 100000, GateVerdict::Hold,
          GateReason::HolddownChannel);
  }
  /* 6. rank alternation: cooldown suppresses the churn (as #2). */
  {
    Decision d = recommend(other, 1.0, 0.95);
    GateState st;
    st.last_move_ms = 50000;
    twice("rank_alternation", automatic(&d), st, 60000, GateVerdict::Hold,
          GateReason::Cooldown);
  }
  /* 7. all degrade: the advisory returns broad-degradation hold. */
  {
    Decision d = hold_dec(Reason::HoldBroadDegradation);
    GateState st;
    twice("all_degrade", automatic(&d), st, 10000, GateVerdict::Hold,
          GateReason::FaultBroadband);
  }
  /* 8. out of range, clean spectrum: weak-link fault, do not move. */
  {
    Decision d = hold_dec(Reason::HoldImpairmentNotChannel);
    GateState st;
    twice("out_of_range_clean", automatic(&d), st, 10000, GateVerdict::Hold,
          GateReason::FaultWeakLink);
  }
  /* 9. saturation: same non-channel fault path. */
  {
    Decision d = hold_dec(Reason::HoldImpairmentNotChannel);
    GateState st;
    twice("saturation", automatic(&d), st, 10000, GateVerdict::Hold,
          GateReason::FaultWeakLink);
  }
  /* 10. scout stall: survey too old -> ScoutStale. */
  {
    Decision d = recommend(tgt, 1.0, 0.95);
    GateInputs in = automatic(&d);
    in.scout_survey_age_ms = 90000; /* > 60 s */
    GateState st;
    twice("scout_stall", in, st, 10000, GateVerdict::Hold,
          GateReason::ScoutStale);
  }
  /* 11. current channel recovers pre-commit: in-flight + advisory now holds
   * with the target unchanged but a new recommend absent -> stay in-flight;
   * if the target CHANGES materially -> AbortInFlight. */
  {
    Decision d = recommend(other, 1.0, 0.95); /* target moved off frozen tgt */
    GateInputs in = automatic(&d);
    in.in_flight = true;
    GateState st;
    st.have_frozen = true;
    st.frozen_gen = 100;
    st.frozen_target = tgt;
    st.frozen_score = 1.0;
    twice("recover_pre_commit", in, st, 10000, GateVerdict::AbortInFlight,
          GateReason::MaterialChange);
  }
  /* 12. destination occupied between recommend and activation: probation
   * delivery bad -> ProposeRollback. */
  {
    GateInputs in = automatic(nullptr);
    in.probation_active = true;
    in.probation_delivery_ok = false;
    GateState st;
    twice("dest_occupied_pre_activation", in, st, 10000,
          GateVerdict::ProposeRollback, GateReason::ProbationRollback);
  }
  /* 13. rollback + move-cap exhaustion: cap reached -> MoveCap. */
  {
    Decision d = recommend(tgt, 1.0, 0.95);
    GateInputs in = automatic(&d);
    GateState st;
    st.moves_session = pol.move_cap;
    twice("rollback_cap_exhaustion", in, st, 1000000, GateVerdict::Hold,
          GateReason::MoveCap);
  }

  /* --- mode + operator overrides --- */
  {
    Decision d = recommend(tgt, 1.0, 0.95);
    GateInputs off = automatic(&d);
    off.mode = MigMode::Off;
    GateState st;
    twice("mode_off", off, st, 10000, GateVerdict::Hold, GateReason::ModeOff);

    GateInputs adv = automatic(&d);
    adv.mode = MigMode::Advisory;
    twice("mode_advisory", adv, st, 10000, GateVerdict::Hold,
          GateReason::Advisory);

    GateInputs man = automatic(&d);
    man.mode = MigMode::Manual;
    twice("manual_needs_approval", man, st, 10000, GateVerdict::Hold,
          GateReason::ApproveRequired);
    man.approve_next = true;
    twice("manual_approved", man, st, 10000, GateVerdict::Propose,
          GateReason::Propose);

    GateInputs pin = automatic(&d);
    pin.pinned = &tgt;
    twice("pinned", pin, st, 10000, GateVerdict::Hold, GateReason::Pinned);

    GateInputs inh = automatic(&d);
    inh.inhibit_until_ms = 20000;
    twice("inhibited", inh, st, 10000, GateVerdict::Hold,
          GateReason::Inhibited);
  }

  /* --- residency, control margin, rescue, telemetry --- */
  {
    Decision d = recommend(tgt, 1.0, 0.95);
    GateState st;
    st.residency_start_ms = 5000;
    twice("residency", automatic(&d), st, 5000 + 60000, GateVerdict::Hold,
          GateReason::Residency);

    GateInputs cm = automatic(&d);
    cm.control_link_margin = 0.1;
    GateState st2;
    twice("control_margin_low", cm, st2, 500000, GateVerdict::Hold,
          GateReason::ControlMarginLow);

    GateInputs nr = automatic(&d);
    nr.rescue_verified = false;
    twice("no_rescue_verified", nr, st2, 500000, GateVerdict::Hold,
          GateReason::NoRescueVerified);

    GateInputs tel = automatic(&d);
    tel.telemetry_ok = false;
    twice("telemetry_unhealthy", tel, st2, 500000, GateVerdict::Hold,
          GateReason::TelemetryUnhealthy);
  }

  /* --- state callbacks: confirm resets streak + starts residency; rollback
   * applies exponential holddown backoff --- */
  {
    GateState st;
    mig_gate_on_confirmed(st, tgt, 100000);
    CHECK(st.moves_session == 1 && st.residency_start_ms == 100000 &&
              st.rollback_streak == 0,
          "confirm updates move/residency/streak");
    mig_gate_on_rolledback(st, tgt, pol, 200000);
    const int64_t hd1 = st.holddown_for(tgt.key());
    CHECK(st.rollback_streak == 1 && hd1 == 200000 + pol.holddown_base_ms,
          "first rollback: base holddown");
    mig_gate_on_rolledback(st, tgt, pol, 300000);
    const int64_t hd2 = st.holddown_for(tgt.key());
    CHECK(st.rollback_streak == 2 && hd2 == 300000 + 2 * pol.holddown_base_ms,
          "second rollback: doubled backoff");
  }

  return fails ? 1 : 0;
}
