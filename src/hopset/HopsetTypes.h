/* Shared types for the adaptive-hopset control protocol (RX proposes, TX —
 * the schedule authority — validates and commits). One header so the wire
 * codec, the two pure machines (HopsetAuthority, HopsetFollower), the demos
 * and the selftests agree on every code.
 *
 * Authority model: the receiver proposes a mask change; the transmitter
 * validates policy (diversity floor, monotonic generation, activation lead)
 * and repeatedly broadcasts an authenticated commit naming a future
 * activation slot; both sides swap state exactly at that slot. Nobody acts on
 * a proposal alone, and acquisition always scans the immutable base hopset. */
#ifndef DEVOURER_HOPSET_HOPSET_TYPES_H
#define DEVOURER_HOPSET_HOPSET_TYPES_H

#include <cstdint>

#include "hopset/HopsetState.h"

namespace devourer {
namespace hopset {

enum HopsetMsgType : uint8_t {
  HT_PROPOSAL = 1,
  HT_COMMIT = 2,
  HT_STATUS = 3,
};

/* Reject / status reason codes (wire byte + event string). */
enum class HopsetReason : uint8_t {
  None = 0,
  BadMac,
  BadVersion,
  BadType,
  BadLinkId,
  Truncated,
  StaleEpoch,
  ReplayGen,
  NonMonotonicGen,
  GenCeiling,
  BadBaseFp,
  BadMask,
  LowDiversity,
  ActivationWindow,
  NoChange,
  Busy,
  NonceMismatch,
  Timeout,
  MaskDelta,
  Cooldown,
  TxVeto,
};

inline const char *hopset_reason_name(HopsetReason r) {
  switch (r) {
  case HopsetReason::None: return "none";
  case HopsetReason::BadMac: return "bad_mac";
  case HopsetReason::BadVersion: return "bad_version";
  case HopsetReason::BadType: return "bad_type";
  case HopsetReason::BadLinkId: return "bad_link_id";
  case HopsetReason::Truncated: return "truncated";
  case HopsetReason::StaleEpoch: return "stale_epoch";
  case HopsetReason::ReplayGen: return "replay_gen";
  case HopsetReason::NonMonotonicGen: return "non_monotonic_gen";
  case HopsetReason::GenCeiling: return "gen_ceiling";
  case HopsetReason::BadBaseFp: return "bad_base_fp";
  case HopsetReason::BadMask: return "bad_mask";
  case HopsetReason::LowDiversity: return "low_diversity";
  case HopsetReason::ActivationWindow: return "activation_window";
  case HopsetReason::NoChange: return "no_change";
  case HopsetReason::Busy: return "busy";
  case HopsetReason::NonceMismatch: return "nonce_mismatch";
  case HopsetReason::Timeout: return "timeout";
  case HopsetReason::MaskDelta: return "mask_delta";
  case HopsetReason::Cooldown: return "cooldown";
  case HopsetReason::TxVeto: return "tx_veto";
  }
  return "?";
}

/* The decoded control message — a flat superset; each type populates its own
 * fields. The pure machines produce/consume these; the host does the crypto
 * (encode/decode/MAC via HopsetWire) and the transmission. */
struct HopsetMsg {
  HopsetMsgType type = HT_PROPOSAL;
  uint32_t link_id = 0;

  /* identity / anti-replay */
  uint32_t rx_epoch = 0;      /* proposal: proposer's per-boot epoch */
  uint32_t tx_epoch = 0;      /* commit: authority's per-boot epoch */
  uint32_t sender_epoch = 0;  /* status */
  uint32_t rx_epoch_echo = 0; /* commit: 0 = authority-originated */
  uint32_t rx_nonce = 0, rx_nonce_echo = 0;

  /* hopset state */
  uint32_t generation = 0; /* commit/status; proposal: proposed (0 = next) */
  uint64_t active_mask = 0;
  uint32_t base_fp = 0;
  uint64_t activate_round = 0; /* commit: previous-generation rounds */
  uint64_t activate_slot = 0;  /* commit/status: absolute-slot anchor */
  uint64_t current_round = 0;  /* sender's round at send time */

  /* proposal evidence summary — opaque here; the observation policy that
   * fills it lives with the exclusion-decision layer */
  uint32_t observation_count = 0;
  uint32_t reason_bitmap = 0;
  uint64_t earliest_activate_round = 0; /* 0 = authority picks */

  /* status */
  uint8_t role = 0;   /* 0 = follower(RX), 1 = authority(TX) */
  uint8_t reason = 0; /* HopsetReason; nonzero = proposal rejection answer */
};

/* Structured-event codes the machines report through actions; the host maps
 * them to the hopset.* JSONL events. */
enum class HopsetEvent : uint8_t {
  Propose = 1, /* proposal sent (follower) or accepted (authority) */
  Commit,      /* commit issued / adopted as pending */
  Activate,    /* committed state swapped in at the boundary */
  Reject,      /* + reason */
  GenMismatch, /* marker advertises a generation/mask we don't hold */
  Recover,     /* re-synchronized from an authenticated commit/status */
};

inline const char *hopset_event_name(HopsetEvent e) {
  switch (e) {
  case HopsetEvent::Propose: return "hopset.propose";
  case HopsetEvent::Commit: return "hopset.commit";
  case HopsetEvent::Activate: return "hopset.activate";
  case HopsetEvent::Reject: return "hopset.reject";
  case HopsetEvent::GenMismatch: return "hopset.gen_mismatch";
  case HopsetEvent::Recover: return "hopset.recover";
  }
  return "?";
}

/* An action a pure machine asks its host to perform: no I/O, no crypto, no
 * clock inside the machines — the host encodes+MACs `msg` for SendControl,
 * applies `state` to its AdaptiveScheduleView on Activate, and logs Events. */
struct HopsetAction {
  enum Kind : uint8_t {
    SendControl, /* msg -> encode + MAC + air */
    Activate,    /* state -> the new committed schedule state */
    Event,       /* event (+ reason, + state context) -> JSONL */
  } kind;
  HopsetMsg msg{};
  HopsetState state{};
  HopsetEvent event = HopsetEvent::Propose;
  HopsetReason reason = HopsetReason::None;
};

/* Static policy knobs, host-provided (the demos map env; the library reads
 * none). Slot-denominated intervals so behavior is deterministic in tests. */
struct HopsetParams {
  size_t n_base = 0;      /* immutable base hopset size (1..64) */
  uint32_t base_fp = 0;   /* hopset_fp over the configured channel list */
  uint32_t link_id = 0;   /* 0 accepts any link on decode */
  unsigned min_active = 2;
  uint64_t lead_rounds = 8;           /* min commit->activation lead */
  uint64_t max_lead_slots = 4096;     /* upper activation window bound */
  uint64_t commit_repeat_slots = 4;   /* re-broadcast cadence until active */
  /* The status beacon is the ONLY way home for a follower that missed the
   * commits: it is scanning the base hopset and cannot ask for anything. So
   * the cadence sets the recovery latency, and it has to be dense enough that
   * a scanning receiver — which is only on the transmitter's channel a
   * fraction of the time — actually coincides with one. Measured on a
   * 3-channel base: at 64 slots a fresh follower missed every beacon in an
   * 18 s window often enough to matter. A 58-byte frame at 6M is ~100 us of
   * air, so density is nearly free and latency is not. */
  uint64_t status_interval_slots = 24;
  /* How often the follower announces itself when it has nothing to propose.
   * Without it a healthy, quiet receiver is indistinguishable from a dead
   * uplink, and a transmitter failsafe keyed on silence would fire on a link
   * that is working perfectly.
   *
   * Off by default, and deliberately so: a lockstep receiver's whole job is
   * to be listening inside tightly-scheduled slots, and every frame it airs
   * is a slot it cannot hear. Only a receiver that is already a talker — one
   * running the exclusion policy, which transmits proposals anyway — should
   * pay that price, and only it has ambiguous silence worth resolving. */
  uint64_t follower_status_slots = 0;
  unsigned proposal_retries = 8;
  uint64_t proposal_backoff_slots = 16;
  /* Structural limits the authority enforces on follower proposals (a
   * locally-originated start_change is exempt). 0 disables a check. */
  unsigned max_mask_delta = 1;         /* channels changed per update */
  uint64_t min_update_gap_rounds = 10; /* rounds between accepted updates */
  unsigned max_excluded_frac_pct = 50; /* cap on excluded/base */
  /* How long after an activation a marker advertising the generation we just
   * left is read as a frame still in flight rather than as a disagreement.
   *
   * Both endpoints swap at the same absolute slot, but a frame composed on one
   * side of that boundary is decoded on the other: the marker is stamped when
   * the frame is assembled and the frame then queues through the bus. Measured
   * on air, treating that one frame as a mismatch cost 46 seconds — the
   * follower dropped lockstep at the exact instant the exclusion took effect,
   * which is the worst moment to start scanning for a way back. The relaxation
   * is self-healing: an authority that really has not moved keeps advertising
   * the old generation, and past the grace the mismatch stands. */
  uint64_t stale_marker_slots = 2;
};

} /* namespace hopset */
} /* namespace devourer */

#endif /* DEVOURER_HOPSET_HOPSET_TYPES_H */
