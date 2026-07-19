/* MigGate — the conservative autonomous-migration policy (issue #279).
 *
 * It sits between the advisory engine (a ChannelScore Decision) and the
 * migration protocol (MigProposer): it may submit a proposal automatically,
 * but only when a conjunction of conservative conditions holds, and it hedges
 * every migration with cooldown, residency, per-channel backoff, a session
 * move cap, probation, and an operator kill switch. Stable single-channel
 * video is the normal state; migrations are exceptional.
 *
 * Pure and deterministic: no clock reads (the caller passes now), no RNG, no
 * device — same input trace ⇒ same decisions, so the whole policy is replayable
 * and selftested against the issue's scenario matrix. The gate does not
 * actuate; it returns a verdict the ground controller feeds to MigProposer. */
#ifndef DEVOURER_CHANMIG_MIG_GATE_H
#define DEVOURER_CHANMIG_MIG_GATE_H

#include <cstdint>
#include <vector>

#include "chanmig/ChannelDef.h"
#include "chanmig/ChannelScore.h"

namespace devourer {
namespace chanmig {

enum class MigMode : uint8_t { Off, Advisory, Manual, Automatic };

inline const char *mig_mode_name(MigMode m) {
  switch (m) {
  case MigMode::Off: return "off";
  case MigMode::Advisory: return "advisory";
  case MigMode::Manual: return "manual";
  case MigMode::Automatic: return "automatic";
  }
  return "?";
}

enum class GateVerdict : uint8_t { Hold, Propose, AbortInFlight, ProposeRollback };

enum class GateReason : uint8_t {
  ModeOff,
  Advisory,
  NotImpaired,        /* the recommendation was a hold */
  FaultWeakLink,      /* migration cannot fix a range problem */
  FaultBroadband,
  FaultSaturation,
  FaultScout,
  FaultUsb,
  TelemetryUnhealthy,
  ClockUnsynced,
  ScoutStale,
  EvidenceStale,
  EvidenceIncomplete,
  Illegal,
  MarginLow,
  ConfidenceLow,
  Cooldown,
  MoveCap,
  Residency,
  HolddownChannel,
  Backoff,
  InFlight,
  Pinned,
  Inhibited,
  ApproveRequired,
  ControlMarginLow,
  NoRescueVerified,
  ProbationActive,
  ProbationRollback,
  MaterialChange,     /* in-flight evidence changed → abort + rescore */
  Propose,            /* all gates passed */
};

const char *gate_reason_name(GateReason r);

struct GatePolicy {
  double confidence_min = 0.9;
  int64_t cooldown_ms = 300000;   /* 5 min between moves */
  int64_t residency_ms = 120000;  /* 2 min minimum on a channel */
  int move_cap = 10;              /* per session */
  int64_t holddown_base_ms = 60000; /* per-channel backoff base (×2^streak) */
  int64_t holddown_cap_ms = 3600000;
  int64_t probation_ms = 60000;
  double control_margin_min = 0.3; /* min recent control-frame delivery */
  double material_change_frac = 0.5; /* score drop (× margin) that aborts in-flight */
};

/* Everything the gate reads for one decision. `rec` is the newest advisory
 * Decision (from ChannelScore); the rest are health/fault flags the ground
 * controller assembles from the primary + scout + control link. */
struct GateInputs {
  MigMode mode = MigMode::Advisory;
  const Decision *rec = nullptr; /* null = no recommendation yet */

  bool telemetry_ok = true;
  bool clock_synced = true;
  bool scout_healthy = true;
  int64_t scout_survey_age_ms = 0;
  int64_t scout_max_age_ms = 60000;
  bool usb_ok = true;

  double control_link_margin = 1.0; /* recent status/ack delivery rate */
  bool rescue_verified = true;
  bool in_flight = false;

  bool probation_active = false;
  bool probation_delivery_ok = true;

  /* operator surface */
  bool approve_next = false;   /* manual mode: one-shot approval armed */
  int64_t inhibit_until_ms = INT64_MIN;
  const ChannelDef *pinned = nullptr;
};

/* Mutable anti-oscillation state the gate carries across decisions. */
struct GateState {
  int64_t last_move_ms = INT64_MIN;
  int moves_session = 0;
  int64_t residency_start_ms = INT64_MIN;
  int rollback_streak = 0;
  struct Holddown {
    uint32_t key;
    int64_t until_ms;
  };
  std::vector<Holddown> holddowns;
  /* frozen in-flight proposal, for the material-change abort test */
  bool have_frozen = false;
  uint64_t frozen_gen = 0;
  ChannelDef frozen_target;
  double frozen_score = 0.0;

  int64_t holddown_for(uint32_t key) const {
    for (const Holddown &h : holddowns)
      if (h.key == key)
        return h.until_ms;
    return INT64_MIN;
  }
  void set_holddown(uint32_t key, int64_t until) {
    for (Holddown &h : holddowns)
      if (h.key == key) {
        h.until_ms = until;
        return;
      }
    holddowns.push_back({key, until});
  }
};

struct GateOutcome {
  GateVerdict verdict = GateVerdict::Hold;
  GateReason reason = GateReason::Advisory;
  ChannelDef target;
  uint64_t evidence_gen = 0;
};

/* Map a ChannelScore hold reason to the gate's fault taxonomy, so a hold's
 * cause is preserved in the migrate.gate event. */
inline GateReason gate_reason_for_hold(Reason r) {
  switch (r) {
  case Reason::HoldActiveHealthy:
  case Reason::HoldImpairmentNotPersistent:
    return GateReason::NotImpaired;
  case Reason::HoldImpairmentNotChannel:
    return GateReason::FaultWeakLink; /* weak OR saturated — both non-channel */
  case Reason::HoldBroadDegradation:
    return GateReason::FaultBroadband;
  case Reason::HoldScoutUnhealthy:
    return GateReason::FaultScout;
  case Reason::HoldPrimaryTelemetryStale:
    return GateReason::TelemetryUnhealthy;
  case Reason::HoldImprovementMargin:
    return GateReason::MarginLow;
  case Reason::HoldCooldown:
    return GateReason::Cooldown;
  case Reason::HoldNoQualifiedCandidate:
    return GateReason::EvidenceIncomplete;
  default:
    return GateReason::NotImpaired;
  }
}

/* The decision. Pure; `now_ms` is caller-supplied. Mutates only GateState
 * (cooldown/holddown/frozen latches); it does NOT record a move — the caller
 * calls on_confirmed()/on_rolledback() when the protocol resolves. */
GateOutcome mig_gate_decide(const GateInputs &in, GateState &st,
                            const GatePolicy &pol, int64_t now_ms);

/* Protocol-resolution callbacks the controller invokes so the anti-oscillation
 * state stays honest. */
void mig_gate_on_confirmed(GateState &st, const ChannelDef &now_on,
                           int64_t now_ms);
void mig_gate_on_rolledback(GateState &st, const ChannelDef &target,
                            const GatePolicy &pol, int64_t now_ms);

} /* namespace chanmig */
} /* namespace devourer */

#endif /* DEVOURER_CHANMIG_MIG_GATE_H */
