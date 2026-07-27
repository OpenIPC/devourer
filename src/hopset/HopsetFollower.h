/* HopsetFollower — the RX-side pure state machine for adaptive hopset
 * changes. The follower proposes mask changes (the observation policy that
 * decides WHAT to propose is a separate layer — here it is caller input),
 * adopts only
 * authenticated commits/statuses from the authority, activates exactly at the
 * committed absolute slot, and recovers from a missed transition: a sync
 * marker advertising a generation/mask it doesn't hold sends it to Recovering
 * (the host falls back to the base-hopset acquire scan, which is always legal
 * because acquisition scans the immutable base hopset), until a repeated
 * commit or the authority's status beacon re-synchronizes it.
 *
 * Pure: no I/O, no crypto, no clock, no randomness — the host feeds
 * MAC-verified messages, the current absolute slot, and its own per-boot
 * random epoch/nonces. */
#ifndef DEVOURER_HOPSET_HOPSET_FOLLOWER_H
#define DEVOURER_HOPSET_HOPSET_FOLLOWER_H

#include <vector>

#include "hopset/HopsetTypes.h"
#include "hopset/HopsetWire.h"

namespace devourer {
namespace hopset {

class HopsetFollower {
public:
  enum class State : uint8_t { Synced, Proposing, Pending, Recovering };

  HopsetFollower(const HopsetParams &params, uint32_t epoch)
      : p_(params), epoch_(epoch) {
    cur_.generation = 0;
    cur_.active_mask = full_mask(p_.n_base);
  }

  State fsm() const { return st_; }
  const HopsetState &state() const { return cur_; }
  const HopsetState &pending() const { return pend_; }
  bool has_pending() const { return st_ == State::Pending; }

  /* Ask the authority for a mask change. nonce: host-random per proposal. */
  std::vector<HopsetAction> propose(uint64_t mask, uint32_t observation_count,
                                    uint32_t reason_bitmap, uint32_t nonce,
                                    uint64_t now_slot) {
    std::vector<HopsetAction> out;
    if (st_ != State::Synced) {
      HopsetAction ev{};
      ev.kind = HopsetAction::Event;
      ev.event = HopsetEvent::Reject;
      ev.reason = HopsetReason::Busy;
      out.push_back(ev);
      return out;
    }
    prop_ = HopsetMsg{};
    prop_.type = HT_PROPOSAL;
    prop_.link_id = p_.link_id;
    prop_.rx_epoch = epoch_;
    prop_.generation = 0; /* 0 = "next" — the authority numbers it */
    prop_.active_mask = mask;
    prop_.base_fp = p_.base_fp;
    prop_.observation_count = observation_count;
    prop_.reason_bitmap = reason_bitmap;
    prop_.rx_nonce = nonce;
    tries_ = 1;
    last_prop_slot_ = now_slot;
    st_ = State::Proposing;
    HopsetAction a{};
    a.kind = HopsetAction::SendControl;
    a.msg = prop_;
    out.push_back(a);
    HopsetAction ev{};
    ev.kind = HopsetAction::Event;
    ev.event = HopsetEvent::Propose;
    ev.msg = prop_;
    out.push_back(ev);
    return out;
  }

  /* An authenticated commit from the authority. */
  std::vector<HopsetAction> on_commit(const HopsetMsg &m, uint64_t now_slot) {
    std::vector<HopsetAction> out;
    if (m.base_fp != p_.base_fp)
      return drop(out, HopsetReason::BadBaseFp);
    if (m.generation >= kGenCeiling)
      return drop(out, HopsetReason::GenCeiling);
    if (!mask_valid(m.active_mask, p_.n_base, p_.min_active))
      return drop(out, m.active_mask & ~full_mask(p_.n_base)
                           ? HopsetReason::BadMask
                           : HopsetReason::LowDiversity);
    const bool same_epoch = have_auth_ && m.tx_epoch == auth_epoch_;
    if (same_epoch) {
      if (st_ == State::Pending && m.generation == pend_.generation &&
          m.active_mask == pend_.active_mask &&
          m.activate_slot == pend_.activate_slot)
        return out; /* repeated commit — already armed */
      if (m.generation <= cur_.generation)
        return drop(out, HopsetReason::ReplayGen);
    }
    /* A commit echoing OUR epoch must echo our outstanding nonce. */
    if (m.rx_epoch_echo == epoch_ && epoch_ != 0 &&
        (st_ != State::Proposing || m.rx_nonce_echo != prop_.rx_nonce))
      return drop(out, HopsetReason::NonceMismatch);
    /* Activation window: a future activation must sit within the allowed
     * lead; a past one is only ever adopted as recovery (the transition
     * already happened — join it now). */
    if (m.activate_slot > now_slot &&
        m.activate_slot - now_slot > p_.max_lead_slots)
      return drop(out, HopsetReason::ActivationWindow);
    adopt_epoch(m.tx_epoch);
    const bool was_recovering = st_ == State::Recovering;
    pend_.epoch = m.tx_epoch;
    pend_.generation = m.generation;
    pend_.active_mask = m.active_mask;
    pend_.activate_round = m.activate_round;
    pend_.activate_slot = m.activate_slot;
    HopsetAction ev{};
    ev.kind = HopsetAction::Event;
    ev.event = was_recovering ? HopsetEvent::Recover : HopsetEvent::Commit;
    ev.state = pend_;
    out.push_back(ev);
    if (m.activate_slot <= now_slot) {
      activate(out);
    } else {
      st_ = State::Pending;
    }
    return out;
  }

  /* The authority's status beacon (also carries proposal rejections). */
  std::vector<HopsetAction> on_status(const HopsetMsg &m, uint64_t now_slot) {
    std::vector<HopsetAction> out;
    if (m.role != 1)
      return out; /* peer-follower chatter — not authority state */
    if (m.base_fp != p_.base_fp)
      return drop(out, HopsetReason::BadBaseFp);
    /* A nonzero reason echoing our outstanding nonce: proposal denied. */
    if (m.reason != static_cast<uint8_t>(HopsetReason::None) &&
        st_ == State::Proposing && m.rx_nonce_echo == prop_.rx_nonce) {
      st_ = State::Synced;
      HopsetAction ev{};
      ev.kind = HopsetAction::Event;
      ev.event = HopsetEvent::Reject;
      ev.reason = static_cast<HopsetReason>(m.reason);
      out.push_back(ev);
      return out;
    }
    if (!mask_valid(m.active_mask, p_.n_base, p_.min_active) &&
        m.generation != 0)
      return drop(out, HopsetReason::BadMask);
    /* The same activation-window bound a commit gets. The status path adopts
     * outright, and its anchor is what the in-flight-marker grace measures
     * against — an activation slot far in the future would stretch that grace
     * over every marker the follower will ever decode and silently disable
     * the only hot-path mismatch tripwire. */
    if (m.activate_slot > now_slot &&
        m.activate_slot - now_slot > p_.max_lead_slots)
      return drop(out, HopsetReason::ActivationWindow);
    const bool same_epoch = have_auth_ && m.sender_epoch == auth_epoch_;
    if (same_epoch && m.generation < cur_.generation)
      return drop(out, HopsetReason::ReplayGen);
    if (same_epoch && m.generation == cur_.generation)
      return resync_check(out); /* in agreement */
    /* Newer generation (or a restarted authority): adopt its current state
     * outright — the status carries the full mask and the activation anchor,
     * which is exactly the recovery an RX that missed the commits needs. */
    adopt_epoch(m.sender_epoch);
    pend_.epoch = m.sender_epoch;
    pend_.generation = m.generation;
    pend_.active_mask =
        m.generation == 0 ? full_mask(p_.n_base) : m.active_mask;
    pend_.activate_round = 0;
    pend_.activate_slot = m.activate_slot;
    HopsetAction ev{};
    ev.kind = HopsetAction::Event;
    ev.event = HopsetEvent::Recover;
    ev.state = pend_;
    out.push_back(ev);
    activate(out);
    return out;
  }

  /* Hot-path check from a decoded v2 sync marker. fp_matches: host-computed
   * "the marker's (generation, mask_fp) equals mask_fp(keys, gen, mask) of
   * the state we hold for that generation" (current or pending).
   * marker_slot is the slot the MARKER was stamped in — the transmitter's
   * clock at composition, not the local clock at decode, which is what makes
   * a boundary-crossing frame recognizable as one. */
  std::vector<HopsetAction> on_marker(uint32_t marker_gen, bool fp_matches,
                                      uint64_t marker_slot) {
    std::vector<HopsetAction> out;
    const bool known = (marker_gen == cur_.generation ||
                        (st_ == State::Pending &&
                         marker_gen == pend_.generation)) &&
                       fp_matches;
    if (known)
      return out;
    /* A marker carrying the generation we just left, STAMPED at or before the
     * activation slot, is a frame that crossed the boundary in flight — the
     * transmitter was telling the truth when it composed it. Acting on it
     * drops lockstep at the one moment the link can least afford it (see
     * HopsetParams::stale_marker_slots). A transmitter that really has not
     * moved keeps stamping fresh slots, so it clears the grace within
     * milliseconds and the mismatch stands. */
    if (have_prev_ && marker_gen == prev_generation_ &&
        marker_slot < activated_slot_ + p_.stale_marker_slots)
      return out;
    if (st_ == State::Recovering && marker_gen == last_mismatch_gen_)
      return out; /* already recovering from this one — don't spam */
    last_mismatch_gen_ = marker_gen;
    st_ = State::Recovering;
    HopsetAction ev{};
    ev.kind = HopsetAction::Event;
    ev.event = HopsetEvent::GenMismatch;
    ev.msg.generation = marker_gen;
    ev.state = cur_;
    out.push_back(ev);
    return out;
  }

  /* Slot heartbeat: pending activation, proposal retry/backoff. */
  std::vector<HopsetAction> on_tick(uint64_t now_slot) {
    std::vector<HopsetAction> out;
    if (st_ == State::Pending && now_slot >= pend_.activate_slot) {
      activate(out);
      return out;
    }
    if (st_ == State::Proposing &&
        now_slot - last_prop_slot_ >= p_.proposal_backoff_slots) {
      if (tries_ >= p_.proposal_retries) {
        st_ = State::Synced;
        HopsetAction ev{};
        ev.kind = HopsetAction::Event;
        ev.event = HopsetEvent::Reject;
        ev.reason = HopsetReason::Timeout;
        out.push_back(ev);
        return out;
      }
      ++tries_;
      last_prop_slot_ = now_slot;
      HopsetAction a{};
      a.kind = HopsetAction::SendControl;
      a.msg = prop_;
      out.push_back(a);
    }
    /* Announce ourselves even with nothing to propose. A receiver that is
     * simply content sends no proposals, which from the transmitter's side
     * looks exactly like a receiver that has gone deaf — and a transmitter
     * failsafe keyed on that silence would fire on a perfectly good link.
     * The status also carries our committed generation and mask, so the
     * transmitter can see a split-brain rather than infer one. */
    /* The receiver's slot clock is FITTED from received markers, so unlike
     * the authority's it can jump backwards when the fit is re-anchored
     * during acquisition. An unsigned difference would underflow there and
     * fire the heartbeat on every tick — the receiver would spend its air
     * time transmitting instead of listening, and never re-acquire. Re-anchor
     * on a backwards jump instead of announcing. And say nothing at all while
     * recovering: there is no useful state to report, and transmitting is the
     * opposite of what re-acquisition needs. */
    if (!have_status_anchor_ || now_slot < last_status_slot_) {
      have_status_anchor_ = true;
      last_status_slot_ = now_slot;
    } else if (p_.follower_status_slots && st_ != State::Recovering &&
               now_slot - last_status_slot_ >= p_.follower_status_slots) {
      last_status_slot_ = now_slot;
      HopsetAction a{};
      a.kind = HopsetAction::SendControl;
      HopsetMsg &m = a.msg;
      m.type = HT_STATUS;
      m.link_id = p_.link_id;
      m.role = 0; /* follower */
      m.sender_epoch = epoch_;
      m.generation = cur_.generation;
      m.active_mask = cur_.active_mask;
      m.base_fp = p_.base_fp;
      m.activate_slot = cur_.activate_slot;
      m.reason = static_cast<uint8_t>(HopsetReason::None);
      out.push_back(a);
    }
    return out;
  }

private:
  void adopt_epoch(uint32_t e) {
    if (!have_auth_ || auth_epoch_ != e) {
      have_auth_ = true;
      auth_epoch_ = e;
    }
  }

  void activate(std::vector<HopsetAction> &out) {
    /* Remember what we just left, and when: a marker stamped with the old
     * generation can still be in flight across the boundary. */
    prev_generation_ = cur_.generation;
    have_prev_ = true;
    activated_slot_ = pend_.activate_slot;
    cur_ = pend_;
    st_ = State::Synced;
    HopsetAction act{};
    act.kind = HopsetAction::Activate;
    act.state = cur_;
    out.push_back(act);
    HopsetAction ev{};
    ev.kind = HopsetAction::Event;
    ev.event = HopsetEvent::Activate;
    ev.state = cur_;
    out.push_back(ev);
  }

  std::vector<HopsetAction> &resync_check(std::vector<HopsetAction> &out) {
    if (st_ == State::Recovering) {
      /* the authority's state matches what we already hold — the mismatch
       * was transient (e.g. a marker raced its own activation) */
      st_ = State::Synced;
      HopsetAction ev{};
      ev.kind = HopsetAction::Event;
      ev.event = HopsetEvent::Recover;
      ev.state = cur_;
      out.push_back(ev);
    }
    return out;
  }

  std::vector<HopsetAction> &drop(std::vector<HopsetAction> &out,
                                  HopsetReason r) {
    HopsetAction ev{};
    ev.kind = HopsetAction::Event;
    ev.event = HopsetEvent::Reject;
    ev.reason = r;
    out.push_back(ev);
    return out;
  }

  HopsetParams p_;
  uint32_t epoch_;
  State st_ = State::Synced;
  HopsetState cur_{};
  HopsetState pend_{};
  HopsetMsg prop_{};
  unsigned tries_ = 0;
  uint64_t last_prop_slot_ = 0;
  uint64_t last_status_slot_ = 0;
  bool have_status_anchor_ = false;
  bool have_auth_ = false;
  uint32_t auth_epoch_ = 0;
  uint32_t last_mismatch_gen_ = 0;
  /* The generation we left at the last activation, and where that was, for
   * the in-flight-marker grace. */
  uint32_t prev_generation_ = 0;
  uint64_t activated_slot_ = 0;
  bool have_prev_ = false;
};

} /* namespace hopset */
} /* namespace devourer */

#endif /* DEVOURER_HOPSET_HOPSET_FOLLOWER_H */
