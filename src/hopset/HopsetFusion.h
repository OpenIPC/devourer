/* HopsetFusion — arbitrating between what the receiver measured and what the
 * transmitter sensed, without letting the blinder endpoint overrule the other.
 *
 * The naive design is to let the transmitter veto a receiver proposal it
 * disagrees with: "the receiver wants to drop channel 3, but my own CCA says 3
 * is quiet, so refuse." That is wrong, and it is wrong in exactly the case
 * this feature exists to serve. A hidden interferer sitting next to the
 * receiver produces precisely that signature — delivery on 3 collapses while
 * the transmitter, far away, hears nothing. The naive veto fires and blocks
 * the correct decision, and it does so most confidently when the
 * transmitter's reading is cleanest, so its confidence is anti-correlated
 * with its correctness.
 *
 * The framing that works: the transmitter has no standing to contradict the
 * receiver ABOUT THE TARGET, because the receiver measured the endpoint that
 * has to decode and the transmitter measured a different point in space. The
 * only thing the transmitter knows that the receiver cannot is its own local
 * conditions — whether it can get on air at all on the channels the move would
 * leave it with. So the veto answers one question and only one:
 *
 *     does this move leave the transmitter worse off, at the transmitter?
 *
 * Two grounds qualify. Neither is a disagreement about the target.
 *
 *   BroadTxDegradation -> Delay. The whole band reads uniformly occupied, so
 *     the transmitter cannot tell channels apart and has no opinion to offer
 *     about this one. Not a refusal: "I cannot see well enough to judge; look
 *     again." It does not spend the receiver's update budget.
 *
 *   RemainingWorse -> Reject. Every channel that would survive the exclusion
 *     is markedly more occupied here than the target is. The transmitter may
 *     fully believe the receiver that the target is dead; it is saying the
 *     alternatives are a cul-de-sac. That is a fact only it possesses.
 *
 * Everything else is reported and ignored. Disagreement is information, not
 * authority, so it is surfaced in the decision and never averaged into it —
 * the returned mask is always one endpoint's proposal verbatim.
 *
 * Pure: no I/O, no clock, no state. The host supplies both endpoints' views. */
#ifndef DEVOURER_HOPSET_HOPSET_FUSION_H
#define DEVOURER_HOPSET_HOPSET_FUSION_H

#include <cstdint>
#include <string>
#include <vector>

#include "hopset/HopsetPolicy.h"
#include "hopset/HopsetSense.h"

namespace devourer {
namespace hopset {

enum class FusionMode : uint8_t {
  RxAuthoritative = 0, /* the transmitter observes but never acts */
  RxPlusTxVeto = 1,    /* default: receiver proposes, transmitter may object */
  EitherEndpoint = 2,  /* either may originate; the receiver still preempts */
  TxFailsafe = 3,      /* transmitter originates only once feedback is gone */
};

inline const char *fusion_mode_name(FusionMode m) {
  switch (m) {
  case FusionMode::RxAuthoritative: return "rx";
  case FusionMode::RxPlusTxVeto: return "veto";
  case FusionMode::EitherEndpoint: return "either";
  case FusionMode::TxFailsafe: return "failsafe";
  }
  return "?";
}

/* Parsed from the demo's DEVOURER_HOP_FUSION. Returns false on an unknown
 * spelling rather than silently picking a mode. */
inline bool parse_fusion_mode(const char *s, FusionMode &out) {
  if (!s)
    return false;
  const std::string v(s);
  if (v == "rx") { out = FusionMode::RxAuthoritative; return true; }
  if (v == "veto") { out = FusionMode::RxPlusTxVeto; return true; }
  if (v == "either") { out = FusionMode::EitherEndpoint; return true; }
  if (v == "failsafe") { out = FusionMode::TxFailsafe; return true; }
  return false;
}

enum class DecisionOrigin : uint8_t { Rx = 0, Tx = 1, Fused = 2, Failsafe = 3 };

inline const char *decision_origin_name(DecisionOrigin o) {
  switch (o) {
  case DecisionOrigin::Rx: return "rx";
  case DecisionOrigin::Tx: return "tx";
  case DecisionOrigin::Fused: return "fused";
  case DecisionOrigin::Failsafe: return "failsafe";
  }
  return "?";
}

/* Origin rides the spare bits of the proposal's existing reason bitmap, so it
 * costs no wire change and no marker version. Rx == 0 means every bitmap
 * written before this layer existed decodes with the right origin. */
inline uint32_t rb_set_origin(uint32_t bm, DecisionOrigin o) {
  return (bm & ~uint32_t(RB_ORIGIN_MASK)) | (uint32_t(o) << RB_ORIGIN_SHIFT);
}
inline DecisionOrigin rb_origin(uint32_t bm) {
  return DecisionOrigin((bm & RB_ORIGIN_MASK) >> RB_ORIGIN_SHIFT);
}

enum class FusedAction : uint8_t { Hold, Exclude, Restore, Delay };

inline const char *fused_action_name(FusedAction a) {
  switch (a) {
  case FusedAction::Hold: return "hold";
  case FusedAction::Exclude: return "exclude";
  case FusedAction::Restore: return "restore";
  case FusedAction::Delay: return "delay";
  }
  return "?";
}

enum class FusionVeto : uint8_t {
  None = 0,
  BroadTxDegradation,     /* -> Delay */
  RemainingWorse,         /* -> Reject */
  SensorAbsent,           /* recorded; can never block */
  InsufficientTxEvidence, /* recorded; can never block */
};

inline const char *fusion_veto_name(FusionVeto v) {
  switch (v) {
  case FusionVeto::None: return "none";
  case FusionVeto::BroadTxDegradation: return "tx_broad";
  case FusionVeto::RemainingWorse: return "tx_alt_dirty";
  case FusionVeto::SensorAbsent: return "tx_sensor_absent";
  case FusionVeto::InsufficientTxEvidence: return "tx_insufficient";
  }
  return "?";
}

enum class FusionHold : uint8_t {
  None = 0,
  RxHeld,
  ModeForbidsTxOrigin,
  FeedbackAlive,
  VetoDelay,
  VetoReject,
};

inline const char *fusion_hold_name(FusionHold h) {
  switch (h) {
  case FusionHold::None: return "none";
  case FusionHold::RxHeld: return "rx_held";
  case FusionHold::ModeForbidsTxOrigin: return "mode_forbids_tx_origin";
  case FusionHold::FeedbackAlive: return "feedback_alive";
  case FusionHold::VetoDelay: return "veto_delay";
  case FusionHold::VetoReject: return "veto_reject";
  }
  return "?";
}

/* "Never heard from the peer" — maximally stale, so a session that starts with
 * a dead return channel still adapts once the observation bar is met. */
inline constexpr uint64_t kNoFeedback = ~uint64_t(0);

struct FusionConfig {
  FusionMode mode = FusionMode::RxPlusTxVeto;

  uint64_t feedback_timeout_rounds = 30;

  bool veto_on_broad_tx = true;
  bool veto_on_remaining_worse = true;
  double remaining_worse_margin = 0.20;
  uint32_t veto_min_chans_scored = 3;
  /* A flat band is only doubt when it is flat AND dirty. Flat and quiet is a
   * clean band — exactly what the transmitter sees when the interferer is at
   * the far end — and must not read as confusion. */
  double veto_broad_floor = 0.35;
  /* A persistently flat transmitter view must not suppress the receiver
   * forever; after this many consecutive delays the veto yields. */
  uint32_t max_consecutive_delays = 3;

  bool reexamine_tx_exclusions = true;
  uint64_t reexamine_grace_rounds = 30;
  /* Below this delivery the receiver is taken to have corroborated an
   * autonomous exclusion, so it is left standing. */
  double corroborate_delivery = 0.50;

  uint32_t policy_hash() const {
    uint32_t h = 2166136261u;
    auto fold = [&h](uint64_t v) {
      for (int i = 0; i < 8; ++i) {
        h ^= static_cast<uint32_t>((v >> (8 * i)) & 0xff);
        h *= 16777619u;
      }
    };
    auto foldd = [&fold](double d) {
      fold(static_cast<uint64_t>(static_cast<int64_t>(d * 10000.0 + 0.5)));
    };
    fold(uint64_t(mode));
    fold(feedback_timeout_rounds);
    fold(veto_on_broad_tx ? 1 : 0);
    fold(veto_on_remaining_worse ? 1 : 0);
    foldd(remaining_worse_margin);
    fold(veto_min_chans_scored);
    foldd(veto_broad_floor);
    fold(max_consecutive_delays);
    fold(reexamine_tx_exclusions ? 1 : 0);
    fold(reexamine_grace_rounds);
    foldd(corroborate_delivery);
    return h;
  }
};

struct FusedDecision {
  DecisionOrigin origin = DecisionOrigin::Rx;
  FusedAction action = FusedAction::Hold;
  uint64_t mask = 0;
  size_t target_index = 0;
  uint32_t reason_bitmap = 0;
  FusionHold hold = FusionHold::None;
  FusionVeto veto = FusionVeto::None;
  uint64_t feedback_age_rounds = 0;
  uint32_t fusion_hash = 0;

  /* Conflict surfacing — populated on every call, acted on by nobody. */
  bool rx_wanted_exclude = false;
  bool tx_wanted_exclude = false;
  bool endpoints_disagree_on_target = false;
  bool tx_evidence_valid = false;
  double tx_target_occupancy = 0.0;
  double tx_band_min_occupancy = 0.0;
  double tx_band_max_occupancy = 0.0;
};

/* What a caller assembles per decision point. `tx_originated_mask`,
 * `consecutive_delays` and `feedback_fresh_since_round` are host-held so this
 * function stays stateless. */
struct FusionInput {
  const HopsetDecision *rx = nullptr;
  const TxSenseCandidate *tx = nullptr;
  uint64_t active_mask = 0;
  uint64_t feedback_age_rounds = kNoFeedback;
  uint64_t now_round = 0;
  uint64_t tx_originated_mask = 0;
  uint64_t feedback_fresh_since_round = 0;
  uint32_t consecutive_delays = 0;
};

namespace detail {

inline bool tx_has(const TxSenseCandidate &tx, size_t i) {
  return i < tx.chans.size() && tx.chans[i].have_evidence();
}
inline double tx_occ(const TxSenseCandidate &tx, size_t i) {
  return i < tx.chans.size() ? tx.chans[i].occupancy() : 0.0;
}

/* The two veto grounds, against an exclusion the receiver proposed. Returns
 * the ground, or None to let the proposal through untouched. */
inline FusionVeto veto_for(const FusionConfig &cfg, const TxSenseCandidate &tx,
                           uint64_t active_mask, size_t target,
                           uint32_t consecutive_delays) {
  /* Without our own evidence we have nothing to say. Recorded so the log
   * shows the veto was unreachable rather than merely unused. */
  if (tx.chans_with_evidence == 0) {
    for (const auto &c : tx.chans)
      if (c.sensor_absent)
        return FusionVeto::SensorAbsent;
    return FusionVeto::InsufficientTxEvidence;
  }
  if (tx.chans_with_evidence < cfg.veto_min_chans_scored || !tx_has(tx, target))
    return FusionVeto::InsufficientTxEvidence;

  /* (a) The band is uniform AND dirty: nothing can be attributed to a
   * channel. A uniform QUIET band is not confusion — it is the ordinary
   * reading when the interferer sits at the far end — hence the floor. */
  if (cfg.veto_on_broad_tx && consecutive_delays < cfg.max_consecutive_delays &&
      (tx.broad_occupancy ||
       (tx.flat_band && tx.band_max_occ >= cfg.veto_broad_floor)))
    return FusionVeto::BroadTxDegradation;

  /* (b) Every survivor is worse here than the target. Unanimity-gated: one
   * acceptable survivor, or one we have not measured, kills the veto — an
   * unmeasured escape route is not a road we may declare closed. */
  if (cfg.veto_on_remaining_worse) {
    const double t = tx_occ(tx, target);
    bool any_survivor = false;
    for (size_t i = 0; i < tx.chans.size(); ++i) {
      if (i == target || !(active_mask & (uint64_t(1) << i)))
        continue;
      if (!tx_has(tx, i))
        return FusionVeto::None; /* unmeasured survivor */
      if (tx_occ(tx, i) < t + cfg.remaining_worse_margin)
        return FusionVeto::None; /* an acceptable survivor exists */
      any_survivor = true;
    }
    if (any_survivor)
      return FusionVeto::RemainingWorse;
  }
  return FusionVeto::None;
}

} /* namespace detail */

inline FusedDecision fuse(const FusionConfig &cfg, const FusionInput &in) {
  FusedDecision d;
  d.fusion_hash = cfg.policy_hash();
  d.feedback_age_rounds = in.feedback_age_rounds;

  const HopsetDecision rx_none;
  const TxSenseCandidate tx_none;
  const HopsetDecision &rx = in.rx ? *in.rx : rx_none;
  const TxSenseCandidate &tx = in.tx ? *in.tx : tx_none;

  d.rx_wanted_exclude = rx.kind == HopsetDecision::Kind::ProposeExclude;
  d.tx_wanted_exclude = tx.kind == TxSenseCandidate::Kind::ProposeExclude;
  d.tx_evidence_valid = tx.chans_with_evidence > 0;
  d.tx_band_min_occupancy = tx.band_min_occ;
  d.tx_band_max_occupancy = tx.band_max_occ;
  /* Whichever endpoint named a target, report OUR occupancy on it — when the
   * receiver named one that is the number the veto turns on, and when only
   * this side did it is the number a reader needs to judge a disagreement the
   * mode forbids acting on. */
  if (d.rx_wanted_exclude && detail::tx_has(tx, rx.target_index))
    d.tx_target_occupancy = detail::tx_occ(tx, rx.target_index);
  else if (d.tx_wanted_exclude && detail::tx_has(tx, tx.target_index))
    d.tx_target_occupancy = detail::tx_occ(tx, tx.target_index);
  d.endpoints_disagree_on_target =
      (d.rx_wanted_exclude != d.tx_wanted_exclude) ||
      (d.rx_wanted_exclude && d.tx_wanted_exclude &&
       rx.target_index != tx.target_index);
  if (d.endpoints_disagree_on_target)
    d.reason_bitmap |= RB_FUSE_TX_DISAGREES;
  if (!d.tx_evidence_valid)
    for (const auto &c : tx.chans)
      if (c.sensor_absent) {
        d.reason_bitmap |= RB_TX_SENSOR_ABSENT;
        break;
      }

  const bool feedback_fresh =
      in.feedback_age_rounds != kNoFeedback &&
      in.feedback_age_rounds <= cfg.feedback_timeout_rounds;

  /* Nothing the transmitter decided while blind survives without the
   * receiver's ratification. Once the uplink is back and the receiver has had
   * a grace window to form an opinion, an un-corroborated autonomous
   * exclusion is handed back — the receiver will take it again under its own
   * authority if the channel really is bad, which is the outcome we want. */
  if (cfg.reexamine_tx_exclusions && in.tx_originated_mask && feedback_fresh &&
      in.now_round >=
          in.feedback_fresh_since_round + cfg.reexamine_grace_rounds) {
    for (size_t i = 0; i < kMaxBaseChannels; ++i) {
      if (!(in.tx_originated_mask & (uint64_t(1) << i)))
        continue;
      const bool corroborated = i < rx.chans.size() && rx.chans[i].scored &&
                                rx.chans[i].delivery < cfg.corroborate_delivery;
      if (corroborated)
        continue;
      d.origin = DecisionOrigin::Fused;
      d.action = FusedAction::Restore;
      d.target_index = i;
      d.mask = in.active_mask | (uint64_t(1) << i);
      d.reason_bitmap |= RB_TX_REEXAMINED;
      d.reason_bitmap = rb_set_origin(d.reason_bitmap, DecisionOrigin::Fused);
      return d;
    }
  }

  /* Handing a channel back outranks taking one away, as on the receiver. */
  if (rx.kind == HopsetDecision::Kind::ProposeRestore) {
    d.origin = DecisionOrigin::Rx;
    d.action = FusedAction::Restore;
    d.mask = rx.proposed_mask;
    d.target_index = rx.target_index;
    d.reason_bitmap |= rx.reason_bitmap;
    d.reason_bitmap = rb_set_origin(d.reason_bitmap, DecisionOrigin::Rx);
    return d; /* restores are never vetoed: they only enlarge the option set */
  }

  if (d.rx_wanted_exclude) {
    FusionVeto v = FusionVeto::None;
    if (cfg.mode != FusionMode::RxAuthoritative)
      v = detail::veto_for(cfg, tx, in.active_mask, rx.target_index,
                           in.consecutive_delays);
    d.veto = v;
    d.target_index = rx.target_index;
    d.reason_bitmap |= rx.reason_bitmap;
    if (v == FusionVeto::BroadTxDegradation) {
      d.origin = DecisionOrigin::Fused;
      d.action = FusedAction::Delay;
      d.hold = FusionHold::VetoDelay;
      d.reason_bitmap |= RB_FUSE_VETO_BROAD;
      d.reason_bitmap = rb_set_origin(d.reason_bitmap, DecisionOrigin::Fused);
      return d;
    }
    if (v == FusionVeto::RemainingWorse) {
      d.origin = DecisionOrigin::Fused;
      d.action = FusedAction::Hold;
      d.hold = FusionHold::VetoReject;
      d.reason_bitmap |= RB_FUSE_VETO_REMAIN;
      d.reason_bitmap = rb_set_origin(d.reason_bitmap, DecisionOrigin::Fused);
      return d;
    }
    d.origin = DecisionOrigin::Rx;
    d.action = FusedAction::Exclude;
    d.mask = rx.proposed_mask;
    d.reason_bitmap = rb_set_origin(d.reason_bitmap, DecisionOrigin::Rx);
    return d;
  }

  /* The receiver had nothing to say. Whether the transmitter may speak is a
   * question of mode and, for the failsafe, of whether the uplink is gone. */
  if (tx.kind == TxSenseCandidate::Kind::None) {
    d.origin = DecisionOrigin::Rx;
    d.action = FusedAction::Hold;
    d.hold = FusionHold::RxHeld;
    return d;
  }

  /* Held or not, the record names what this side wanted: an unacted-upon
   * disagreement still has to be legible after the fact. */
  d.target_index = tx.target_index;

  DecisionOrigin origin = DecisionOrigin::Tx;
  switch (cfg.mode) {
  case FusionMode::RxAuthoritative:
  case FusionMode::RxPlusTxVeto:
    d.origin = DecisionOrigin::Rx;
    d.action = FusedAction::Hold;
    d.hold = FusionHold::ModeForbidsTxOrigin;
    return d;
  case FusionMode::TxFailsafe:
    if (feedback_fresh) {
      d.origin = DecisionOrigin::Rx;
      d.action = FusedAction::Hold;
      d.hold = FusionHold::FeedbackAlive;
      return d;
    }
    origin = DecisionOrigin::Failsafe;
    break;
  case FusionMode::EitherEndpoint:
    origin = DecisionOrigin::Tx;
    break;
  }

  d.origin = origin;
  d.action = tx.kind == TxSenseCandidate::Kind::ProposeRestore
                 ? FusedAction::Restore
                 : FusedAction::Exclude;
  d.mask = tx.proposed_mask;
  d.target_index = tx.target_index;
  d.reason_bitmap |= tx.reason_bitmap;
  d.reason_bitmap = rb_set_origin(d.reason_bitmap, origin);
  return d;
}

} /* namespace hopset */
} /* namespace devourer */

#endif /* DEVOURER_HOPSET_HOPSET_FUSION_H */
