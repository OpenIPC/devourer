/* Shared types for the coordinated channel-migration protocol (ground proposes,
 * drone commits). The state/reason/action enums both pure machines
 * (MigProposer, MigResponder) speak, kept in one header so the wire codec, the
 * machines, the demo, and the selftests agree on every code.
 *
 * Authority model: ground proposes a target, the drone (the video TX) validates
 * and becomes the final schedule authority by committing an activation instant
 * on its own clock. Neither endpoint retunes on a PROPOSAL; both act only on an
 * authenticated, matching, acknowledged COMMIT. */
#ifndef DEVOURER_CHANMIG_MIG_TYPES_H
#define DEVOURER_CHANMIG_MIG_TYPES_H

#include <cstdint>
#include <vector>

#include "chanmig/ChannelDef.h"

namespace devourer {
namespace chanmig {

enum MigMsgType : uint8_t {
  MT_PROPOSAL = 1,
  MT_COMMIT = 2,
  MT_STATUS = 3,
  MT_CONFIRM = 4,
  MT_ABORT = 5,
  MT_MARKER = 6,
  MT_VALIDATION = 7,
};

/* Canonical migration state names (issue #278). Both roles use this enum;
 * a role only visits the states meaningful to it. */
enum class MigState : uint8_t {
  Stable,
  Proposed,   /* ground: proposal sent, awaiting commit */
  Validating, /* drone: proposal received, running target validation (#280) */
  Committed,  /* commit exchanged; activation scheduled */
  Switching,  /* draining + retuning at the activation instant */
  Verifying,  /* on the new channel, checking generation-tagged evidence */
  Confirmed,  /* the move succeeded */
  Rollback,   /* returning to the fallback channel */
  Recovery,   /* ground: lost the peer, scanning old/new/rescue */
};

inline const char *mig_state_name(MigState s) {
  switch (s) {
  case MigState::Stable: return "STABLE";
  case MigState::Proposed: return "PROPOSED";
  case MigState::Validating: return "VALIDATING";
  case MigState::Committed: return "COMMITTED";
  case MigState::Switching: return "SWITCHING";
  case MigState::Verifying: return "VERIFYING";
  case MigState::Confirmed: return "CONFIRMED";
  case MigState::Rollback: return "ROLLBACK";
  case MigState::Recovery: return "RECOVERY";
  }
  return "?";
}

/* Reject / status reason codes (wire byte + event string). */
enum class MigReason : uint8_t {
  None = 0,
  BadMac,
  BadVersion,
  BadType,
  BadLinkId,
  Truncated,
  ReplayGen,
  StaleEpoch,
  SourceMismatch,
  UnknownChannel,
  UnsupportedWidth,
  IllegalTarget,
  ActivationBounds,
  Busy,
  NoAck,
  RetuneFail,
  Veto,
  NonceMismatch,
};

inline const char *mig_reason_name(MigReason r) {
  switch (r) {
  case MigReason::None: return "none";
  case MigReason::BadMac: return "bad_mac";
  case MigReason::BadVersion: return "bad_version";
  case MigReason::BadType: return "bad_type";
  case MigReason::BadLinkId: return "bad_link_id";
  case MigReason::Truncated: return "truncated";
  case MigReason::ReplayGen: return "replay_gen";
  case MigReason::StaleEpoch: return "stale_epoch";
  case MigReason::SourceMismatch: return "source_mismatch";
  case MigReason::UnknownChannel: return "unknown_channel";
  case MigReason::UnsupportedWidth: return "unsupported_width";
  case MigReason::IllegalTarget: return "illegal_target";
  case MigReason::ActivationBounds: return "activation_bounds";
  case MigReason::Busy: return "busy";
  case MigReason::NoAck: return "no_ack";
  case MigReason::RetuneFail: return "retune_fail";
  case MigReason::Veto: return "veto";
  case MigReason::NonceMismatch: return "nonce_mismatch";
  }
  return "?";
}

/* The decoded control message — a flat superset; each type populates its own
 * fields (kept flat, no variant, so the state machines read fields by name).
 * The pure machines produce and consume these; the demo/LinkSim does the
 * crypto (encode/decode/MAC via MigWire) and the TSF stamping. */
struct MigMsg {
  MigMsgType type = MT_PROPOSAL;
  uint32_t link_id = 0;

  /* identity / anti-replay */
  uint32_t ground_epoch = 0, drone_epoch = 0, sender_epoch = 0;
  uint32_t generation = 0;
  uint32_t ground_nonce = 0, drone_nonce = 0;
  uint32_t ground_nonce_echo = 0, drone_nonce_echo = 0, peer_nonce_echo = 0;

  /* channels */
  ChannelDef source, target, rescue, current, aired_on;

  /* proposal */
  uint32_t evidence_gen = 0;
  uint64_t evidence_digest = 0;
  uint64_t earliest_tsf = 0, latest_tsf = 0;
  uint8_t fallback_mode = 0; /* 0 = source, 1 = rescue */

  /* commit */
  uint64_t activate_tsf = 0, rollback_deadline_tsf = 0;
  uint32_t confirm_window_us = 0;
  uint8_t armed = 0; /* bit0: activation armed (ground ack seen) */

  /* status */
  uint8_t role = 0;  /* 0 = ground, 1 = drone */
  uint8_t state = 0; /* MigState */

  /* confirm */
  uint16_t marker_count = 0, video_frames = 0;
  uint32_t first_marker_tsfl = 0;

  /* abort / reason-carrying */
  uint8_t reason = 0;    /* MigReason */
  uint8_t effective = 0; /* 0 = cancel pre-activation, 1 = rollback */

  /* marker */
  uint32_t seq = 0;
  uint8_t marker_flags = 0; /* bit0 rollback, bit1 probe-return */

  /* validation (#280) */
  uint8_t method = 0; /* 0 checks / 1 probe / 2 probation / 3 scout */
  uint8_t result = 0; /* 0 accept / 1 veto / 2 unknown */
  uint32_t obs_age_ms = 0, obs_dur_ms = 0, cca_delta = 0, fa_delta = 0;
  uint8_t igi = 0, nhm_busy_pct = 0, energy_valid = 0;
  uint16_t cost_est_ms = 0;

  /* clock: the sender's TSF, stamped by the demo at send time */
  uint64_t tx_tsf = 0;
};

/* An action the pure machine asks its host (the demo) to perform. The machine
 * itself does no I/O and no crypto — it returns a list of these from every
 * input; the host stamps tx_tsf, encodes+MACs a `msg`, and transmits. */
struct MigAction {
  enum Kind : uint8_t {
    SendUnicast,   /* msg -> the peer's unicast address (+ tx.report) */
    SendBroadcast, /* msg -> broadcast (status / markers) */
    RetuneTo,      /* channel */
    StartDrain,    /* stop the TX pump, drain/deadline-drop pending frames */
    ResumePump,    /* resume the TX pump (probe return / post-switch) */
    ArmMarkers,    /* channel = the channel; emit gen-tagged markers there */
    StopMarkers,
    EmitEvent,     /* code = a MigReason or a state; for logging */
    GateNotify,    /* code: 0=confirmed 1=rolledback — tell the #279 gate */
    Done,          /* code: terminal result */
  } kind;
  MigMsg msg;
  ChannelDef channel;
  uint8_t code = 0;
};

} /* namespace chanmig */
} /* namespace devourer */

#endif /* DEVOURER_CHANMIG_MIG_TYPES_H */
