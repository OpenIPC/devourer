#include "chanmig/MigGate.h"

namespace devourer {
namespace chanmig {

const char *gate_reason_name(GateReason r) {
  switch (r) {
  case GateReason::ModeOff: return "MODE_OFF";
  case GateReason::Advisory: return "ADVISORY";
  case GateReason::NotImpaired: return "NOT_IMPAIRED";
  case GateReason::FaultWeakLink: return "FAULT_WEAK_LINK";
  case GateReason::FaultBroadband: return "FAULT_BROADBAND";
  case GateReason::FaultSaturation: return "FAULT_SATURATION";
  case GateReason::FaultScout: return "FAULT_SCOUT";
  case GateReason::FaultUsb: return "FAULT_USB";
  case GateReason::TelemetryUnhealthy: return "TELEMETRY_UNHEALTHY";
  case GateReason::ClockUnsynced: return "CLOCK_UNSYNCED";
  case GateReason::ScoutStale: return "SCOUT_STALE";
  case GateReason::EvidenceStale: return "EVIDENCE_STALE";
  case GateReason::EvidenceIncomplete: return "EVIDENCE_INCOMPLETE";
  case GateReason::Illegal: return "ILLEGAL";
  case GateReason::MarginLow: return "MARGIN_LOW";
  case GateReason::ConfidenceLow: return "CONFIDENCE_LOW";
  case GateReason::Cooldown: return "COOLDOWN";
  case GateReason::MoveCap: return "MOVE_CAP";
  case GateReason::Residency: return "RESIDENCY";
  case GateReason::HolddownChannel: return "HOLDDOWN_CHANNEL";
  case GateReason::Backoff: return "BACKOFF";
  case GateReason::InFlight: return "IN_FLIGHT";
  case GateReason::Pinned: return "PINNED";
  case GateReason::Inhibited: return "INHIBITED";
  case GateReason::ApproveRequired: return "APPROVE_REQUIRED";
  case GateReason::ControlMarginLow: return "CONTROL_MARGIN_LOW";
  case GateReason::NoRescueVerified: return "NO_RESCUE_VERIFIED";
  case GateReason::ProbationActive: return "PROBATION_ACTIVE";
  case GateReason::ProbationRollback: return "PROBATION_ROLLBACK";
  case GateReason::MaterialChange: return "MATERIAL_CHANGE";
  case GateReason::Propose: return "PROPOSE";
  }
  return "?";
}

static GateOutcome hold(GateReason r) {
  GateOutcome o;
  o.verdict = GateVerdict::Hold;
  o.reason = r;
  return o;
}

GateOutcome mig_gate_decide(const GateInputs &in, GateState &st,
                            const GatePolicy &pol, int64_t now_ms) {
  /* --- operator overrides + mode (highest precedence) --- */
  if (in.mode == MigMode::Off)
    return hold(GateReason::ModeOff);
  if (in.pinned != nullptr)
    return hold(GateReason::Pinned);
  if (in.inhibit_until_ms != INT64_MIN && now_ms < in.inhibit_until_ms)
    return hold(GateReason::Inhibited);

  /* --- probation: destination must prove itself, else roll back --- */
  if (in.probation_active) {
    if (!in.probation_delivery_ok) {
      GateOutcome o;
      o.verdict = GateVerdict::ProposeRollback;
      o.reason = GateReason::ProbationRollback;
      return o;
    }
    return hold(GateReason::ProbationActive);
  }

  /* --- an in-flight migration: only abort on a material evidence change --- */
  if (in.in_flight) {
    if (st.have_frozen && in.rec != nullptr &&
        in.rec->kind == Decision::Kind::Recommend) {
      const bool target_moved = !in.rec->target.same_rf(st.frozen_target);
      double best = 0.0;
      for (const CandidateScore &c : in.rec->ranking)
        if (c.def.same_rf(st.frozen_target)) {
          best = c.score;
          break;
        }
      const bool score_dropped =
          best < st.frozen_score - pol.material_change_frac * 0.1;
      if (target_moved || score_dropped) {
        GateOutcome o;
        o.verdict = GateVerdict::AbortInFlight;
        o.reason = GateReason::MaterialChange;
        return o;
      }
    }
    return hold(GateReason::InFlight);
  }

  /* Advisory mode never proposes (but still reports the counterfactual). */
  if (in.mode == MigMode::Advisory)
    return hold(GateReason::Advisory);

  /* --- health / fault domains --- */
  if (!in.telemetry_ok)
    return hold(GateReason::TelemetryUnhealthy);
  if (!in.clock_synced)
    return hold(GateReason::ClockUnsynced);
  if (!in.usb_ok)
    return hold(GateReason::FaultUsb);
  if (!in.scout_healthy)
    return hold(GateReason::FaultScout);
  if (in.scout_survey_age_ms > in.scout_max_age_ms)
    return hold(GateReason::ScoutStale);

  /* --- the advisory recommendation must itself be a Recommend --- */
  if (in.rec == nullptr)
    return hold(GateReason::NotImpaired);
  if (in.rec->kind != Decision::Kind::Recommend)
    return hold(gate_reason_for_hold(in.rec->primary_reason));

  /* find the recommended target's score/confidence in the ranking */
  double conf = 0.0;
  bool found = false;
  for (const CandidateScore &c : in.rec->ranking)
    if (c.def.same_rf(in.rec->target)) {
      conf = c.confidence;
      found = true;
      break;
    }
  if (!found)
    return hold(GateReason::EvidenceIncomplete);
  if (conf < pol.confidence_min)
    return hold(GateReason::ConfidenceLow);

  /* --- anti-oscillation gates --- */
  if (st.last_move_ms != INT64_MIN && now_ms - st.last_move_ms < pol.cooldown_ms)
    return hold(GateReason::Cooldown);
  if (st.residency_start_ms != INT64_MIN &&
      now_ms - st.residency_start_ms < pol.residency_ms)
    return hold(GateReason::Residency);
  if (st.moves_session >= pol.move_cap)
    return hold(GateReason::MoveCap);
  const int64_t hd = st.holddown_for(in.rec->target.key());
  if (hd != INT64_MIN && now_ms < hd)
    return hold(GateReason::HolddownChannel);

  /* --- can we actually complete the move? --- */
  if (in.control_link_margin < pol.control_margin_min)
    return hold(GateReason::ControlMarginLow);
  if (!in.rescue_verified)
    return hold(GateReason::NoRescueVerified);

  /* --- manual mode requires an explicit one-shot approval --- */
  if (in.mode == MigMode::Manual && !in.approve_next)
    return hold(GateReason::ApproveRequired);

  /* all gates passed — propose, and freeze the evidence for the
   * material-change abort test. */
  st.have_frozen = true;
  st.frozen_gen = in.rec->evidence_gen;
  st.frozen_target = in.rec->target;
  double best = 0.0;
  for (const CandidateScore &c : in.rec->ranking)
    if (c.def.same_rf(in.rec->target)) {
      best = c.score;
      break;
    }
  st.frozen_score = best;

  GateOutcome o;
  o.verdict = GateVerdict::Propose;
  o.reason = GateReason::Propose;
  o.target = in.rec->target;
  o.evidence_gen = in.rec->evidence_gen;
  return o;
}

void mig_gate_on_confirmed(GateState &st, const ChannelDef &now_on,
                           int64_t now_ms) {
  (void)now_on;
  st.last_move_ms = now_ms;
  st.residency_start_ms = now_ms;
  ++st.moves_session;
  st.rollback_streak = 0;
  st.have_frozen = false;
}

void mig_gate_on_rolledback(GateState &st, const ChannelDef &target,
                            const GatePolicy &pol, int64_t now_ms) {
  ++st.rollback_streak;
  int64_t backoff = pol.holddown_base_ms;
  for (int i = 1; i < st.rollback_streak && backoff < pol.holddown_cap_ms; ++i)
    backoff *= 2;
  if (backoff > pol.holddown_cap_ms)
    backoff = pol.holddown_cap_ms;
  st.set_holddown(target.key(), now_ms + backoff);
  st.have_frozen = false;
  /* a rollback also counts against the move budget so a flapping channel
   * cannot exhaust retries silently */
  st.last_move_ms = now_ms;
}

} /* namespace chanmig */
} /* namespace devourer */
