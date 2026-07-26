/* HopsetAuthority — the TX-side pure state machine for adaptive hopset
 * changes. The transmitter is the schedule authority: it validates proposals
 * against policy (mask legality, diversity floor, monotonic generation,
 * activation lead), then repeatedly broadcasts an identical authenticated
 * commit naming an absolute future activation slot until that slot arrives,
 * and swaps its own committed state exactly there. It also beacons a status
 * message so a follower that missed every commit can recover.
 *
 * Pure: no I/O, no crypto, no clock — the host feeds MAC-verified decoded
 * messages plus the current absolute slot, and executes the returned actions
 * (encode+MAC+send, apply-to-schedule-view, emit-event). */
#ifndef DEVOURER_HOPSET_HOPSET_AUTHORITY_H
#define DEVOURER_HOPSET_HOPSET_AUTHORITY_H

#include <vector>

#include "hopset/HopsetTypes.h"
#include "hopset/HopsetWire.h"

namespace devourer {
namespace hopset {

class HopsetAuthority {
public:
  /* epoch: random per boot, host-seeded (std::random_device). The authority
   * starts at generation 0 = the legacy full-mask fixed schedule. */
  HopsetAuthority(const HopsetParams &params, uint32_t epoch)
      : p_(params), epoch_(epoch) {
    cur_.epoch = epoch;
    cur_.generation = 0;
    cur_.active_mask = full_mask(p_.n_base);
  }

  const HopsetState &state() const { return cur_; }
  bool committing() const { return committing_; }
  const HopsetState &pending() const { return pend_; }

  /* Adopt a committed state wholesale (test rigs, or a host resuming a
   * session it persisted itself — the protocol never persists state). */
  void restore(const HopsetState &st) {
    cur_ = st;
    cur_.epoch = epoch_;
    committing_ = false;
  }

  /* A follower proposal (already MAC-verified by the host's decode). */
  std::vector<HopsetAction> on_proposal(const HopsetMsg &m,
                                        uint64_t now_slot) {
    std::vector<HopsetAction> out;
    if (m.base_fp != p_.base_fp)
      return reject(out, m, HopsetReason::BadBaseFp, now_slot);
    if (replays_.contains(m.rx_epoch, m.rx_nonce)) {
      /* the exact exchange we already answered: if it is the one still being
       * committed, re-answer idempotently; anything else is a replay. */
      if (committing_ && m.rx_epoch == pend_echo_epoch_ &&
          m.rx_nonce == pend_echo_nonce_) {
        push_commit(out, now_slot);
        return out;
      }
      return reject(out, m, HopsetReason::ReplayGen, now_slot);
    }
    if (committing_)
      return reject(out, m, HopsetReason::Busy, now_slot);
    if (m.generation != 0 && m.generation != cur_.generation + 1)
      return reject(out, m, HopsetReason::NonMonotonicGen, now_slot);
    const HopsetReason mr = mask_reason(m.active_mask);
    if (mr != HopsetReason::None)
      return reject(out, m, mr, now_slot);
    /* Structural limits the authority enforces on its own account: a
     * follower is the decision-maker but not trusted with the shape of the
     * change. One channel per update, a floor on how often updates may
     * happen — a buggy or hostile proposer cannot walk the link down in one
     * step or by flooding. Locally-originated changes (start_change) are
     * exempt: they are the operator's own lever. */
    if (p_.max_mask_delta &&
        popcount64(m.active_mask ^ cur_.active_mask) > p_.max_mask_delta)
      return reject(out, m, HopsetReason::MaskDelta, now_slot);
    if (p_.min_update_gap_rounds && have_activation_ &&
        round_of(now_slot) - last_activation_round_ < p_.min_update_gap_rounds)
      return reject(out, m, HopsetReason::Cooldown, now_slot);
    replays_.remember(m.rx_epoch, m.rx_nonce);
    pend_echo_epoch_ = m.rx_epoch;
    pend_echo_nonce_ = m.rx_nonce;
    start_commit(out, m.active_mask, m.earliest_activate_round, now_slot);
    HopsetAction ev{};
    ev.kind = HopsetAction::Event;
    ev.event = HopsetEvent::Propose;
    ev.msg = m;
    out.push_back(ev);
    return out;
  }

  /* A locally-originated change (scripted commits, or a TX-side exclusion
   * policy once one exists). rx_epoch_echo stays 0 on the wire. */
  std::vector<HopsetAction> start_change(uint64_t mask, uint64_t now_slot) {
    std::vector<HopsetAction> out;
    if (committing_) {
      HopsetAction ev{};
      ev.kind = HopsetAction::Event;
      ev.event = HopsetEvent::Reject;
      ev.reason = HopsetReason::Busy;
      out.push_back(ev);
      return out;
    }
    const HopsetReason mr = mask_reason(mask);
    if (mr != HopsetReason::None) {
      HopsetAction ev{};
      ev.kind = HopsetAction::Event;
      ev.event = HopsetEvent::Reject;
      ev.reason = mr;
      out.push_back(ev);
      return out;
    }
    pend_echo_epoch_ = pend_echo_nonce_ = 0;
    start_commit(out, mask, 0, now_slot);
    return out;
  }

  /* Slot heartbeat: commit repetition, the activation swap, status beacon. */
  std::vector<HopsetAction> on_tick(uint64_t now_slot) {
    std::vector<HopsetAction> out;
    if (committing_) {
      if (now_slot >= pend_.activate_slot) {
        cur_ = pend_;
        committing_ = false;
        last_activation_round_ = round_of(now_slot);
        have_activation_ = true;
        HopsetAction act{};
        act.kind = HopsetAction::Activate;
        act.state = cur_;
        out.push_back(act);
        HopsetAction ev{};
        ev.kind = HopsetAction::Event;
        ev.event = HopsetEvent::Activate;
        ev.state = cur_;
        out.push_back(ev);
      } else if (now_slot - last_commit_slot_ >= p_.commit_repeat_slots) {
        push_commit(out, now_slot);
      }
    }
    if (now_slot - last_status_slot_ >= p_.status_interval_slots) {
      last_status_slot_ = now_slot;
      HopsetAction a{};
      a.kind = HopsetAction::SendControl;
      a.msg = status_msg(now_slot, HopsetReason::None);
      out.push_back(a);
    }
    return out;
  }

  /* Round index of `slot` in the current generation's numbering. */
  uint64_t round_of(uint64_t slot) const {
    const size_t k = cur_.generation == 0 ? p_.n_base
                                          : popcount64(cur_.active_mask);
    return (slot - cur_.activate_slot) / k;
  }

private:
  HopsetReason mask_reason(uint64_t mask) const {
    if (cur_.generation + 1 >= kGenCeiling)
      return HopsetReason::GenCeiling;
    if (mask & ~full_mask(p_.n_base))
      return HopsetReason::BadMask;
    if (popcount64(mask) < p_.min_active)
      return HopsetReason::LowDiversity;
    if (p_.max_excluded_frac_pct < 100 &&
        popcount64(full_mask(p_.n_base) & ~mask) * 100 >
            p_.max_excluded_frac_pct * p_.n_base)
      return HopsetReason::LowDiversity;
    if (mask == cur_.active_mask && cur_.generation != 0)
      return HopsetReason::NoChange;
    return HopsetReason::None;
  }

  void start_commit(std::vector<HopsetAction> &out, uint64_t mask,
                    uint64_t earliest_round, uint64_t now_slot) {
    const size_t k_cur = cur_.generation == 0 ? p_.n_base
                                              : popcount64(cur_.active_mask);
    const uint64_t cur_round = round_of(now_slot);
    uint64_t lead = p_.lead_rounds < 8 ? 8 : p_.lead_rounds;
    uint64_t act_round = cur_round + lead;
    if (earliest_round > act_round)
      act_round = earliest_round;
    pend_ = cur_;
    pend_.generation = cur_.generation + 1;
    pend_.active_mask = mask;
    pend_.activate_round = act_round;
    pend_.activate_slot = cur_.activate_slot + act_round * k_cur;
    pend_.epoch = epoch_;
    committing_ = true;
    push_commit(out, now_slot);
    HopsetAction ev{};
    ev.kind = HopsetAction::Event;
    ev.event = HopsetEvent::Commit;
    ev.state = pend_;
    out.push_back(ev);
  }

  void push_commit(std::vector<HopsetAction> &out, uint64_t now_slot) {
    last_commit_slot_ = now_slot;
    HopsetAction a{};
    a.kind = HopsetAction::SendControl;
    HopsetMsg &m = a.msg;
    m.type = HT_COMMIT;
    m.link_id = p_.link_id;
    m.tx_epoch = epoch_;
    m.rx_epoch_echo = pend_echo_epoch_;
    m.generation = pend_.generation;
    m.active_mask = pend_.active_mask;
    m.base_fp = p_.base_fp;
    m.activate_round = pend_.activate_round;
    m.activate_slot = pend_.activate_slot;
    m.current_round = round_of(now_slot);
    m.rx_nonce_echo = pend_echo_nonce_;
    out.push_back(a);
  }

  HopsetMsg status_msg(uint64_t now_slot, HopsetReason reason) const {
    HopsetMsg m;
    m.type = HT_STATUS;
    m.link_id = p_.link_id;
    m.role = 1;
    m.sender_epoch = epoch_;
    m.generation = cur_.generation;
    m.active_mask = cur_.active_mask;
    m.base_fp = p_.base_fp;
    m.current_round = round_of(now_slot);
    m.activate_slot = cur_.activate_slot;
    m.reason = static_cast<uint8_t>(reason);
    return m;
  }

  std::vector<HopsetAction> &reject(std::vector<HopsetAction> &out,
                                    const HopsetMsg &m, HopsetReason r,
                                    uint64_t now_slot) {
    HopsetAction a{};
    a.kind = HopsetAction::SendControl;
    a.msg = status_msg(now_slot, r);
    a.msg.rx_nonce_echo = m.rx_nonce;
    out.push_back(a);
    HopsetAction ev{};
    ev.kind = HopsetAction::Event;
    ev.event = HopsetEvent::Reject;
    ev.reason = r;
    out.push_back(ev);
    return out;
  }

  HopsetParams p_;
  uint32_t epoch_;
  HopsetState cur_{};
  HopsetState pend_{};
  bool committing_ = false;
  uint32_t pend_echo_epoch_ = 0, pend_echo_nonce_ = 0;
  uint64_t last_commit_slot_ = 0;
  uint64_t last_status_slot_ = 0;
  uint64_t last_activation_round_ = 0;
  bool have_activation_ = false;
  ProposalReplayRing replays_;
};

} /* namespace hopset */
} /* namespace devourer */

#endif /* DEVOURER_HOPSET_HOPSET_AUTHORITY_H */
