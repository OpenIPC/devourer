/* MigProposer — the GROUND state machine (pure).
 *
 * Ground proposes a target and, on an authenticated matching COMMIT, becomes a
 * follower of the drone's activation schedule: it acks the commit (its nonce
 * echo IS the ack the drone waits for), retunes its primary RX at the relative
 * activation instant, verifies generation-tagged markers + video on the new
 * channel, and confirms — or falls into a bounded recovery scan of
 * old/new/rescue. It reads no clock and does no crypto: inputs are decoded
 * MigMsgs + local observations + monotonic `now`; outputs are MigActions the
 * demo executes (stamping tx_tsf and encoding on the way out).
 *
 * Convergence invariants (proven per row in the failure-matrix selftest):
 *   I1  never retune before adopting a nonce+generation-matched commit
 *   I2  every post-activation path has a bounded route back to a channel in
 *       the recovery scan set
 *   I5  a replayed/forged exchange can never be adopted (nonce+gen gate) */
#ifndef DEVOURER_CHANMIG_MIG_PROPOSER_H
#define DEVOURER_CHANMIG_MIG_PROPOSER_H

#include <vector>

#include "chanmig/MigConfig.h"
#include "chanmig/MigTypes.h"

namespace devourer {
namespace chanmig {

/* Deterministic nonce: unpredictable across sessions (the epoch is random,
 * seeded by the demo from std::random_device), reproducible within a trace. */
inline uint32_t mig_mix_nonce(uint32_t epoch, uint32_t gen, uint32_t salt) {
  uint32_t h = epoch * 2654435761u + gen * 40503u + salt * 2246822519u;
  h ^= h >> 15;
  h *= 2246822519u;
  h ^= h >> 13;
  return h ? h : 1u;
}

class MigProposer {
public:
  MigProposer(MigParams params, uint32_t link_id, uint32_t ground_epoch,
              ChannelDef source)
      : p_(params), link_id_(link_id), epoch_(ground_epoch), source_(source),
        current_(source) {}

  MigState state() const { return st_; }
  ChannelDef current_channel() const { return current_; }
  uint32_t generation() const { return gen_; }
  uint32_t wrong_channel_frames() const { return wrong_channel_; }

  /* Operator/gate trigger. No-op unless Stable. */
  std::vector<MigAction> start(const ChannelDef &target, uint32_t evidence_gen,
                               const ChannelDef &rescue, int64_t now) {
    std::vector<MigAction> a;
    if (st_ != MigState::Stable)
      return a;
    ++gen_;
    nonce_ = mig_mix_nonce(epoch_, gen_, 0xA1);
    target_ = target;
    rescue_ = rescue;
    evidence_gen_ = evidence_gen;
    st_ = MigState::Proposed;
    tries_ = 1;
    next_send_ms_ = now + p_.proposal_interval_ms;
    a.push_back(send(MT_PROPOSAL, make_proposal(), true));
    a.push_back(ev());
    return a;
  }

  std::vector<MigAction> on_message(const MigMsg &m, int64_t now) {
    switch (m.type) {
    case MT_COMMIT: return on_commit(m, now);
    case MT_STATUS: return on_status(m, now);
    case MT_MARKER: return on_marker(m, now);
    case MT_ABORT: return on_abort(m, now);
    default: return {};
    }
  }

  std::vector<MigAction> on_tick(int64_t now) {
    switch (st_) {
    case MigState::Proposed: return tick_proposed(now);
    case MigState::Committed: return tick_committed(now);
    case MigState::Verifying: return tick_verifying(now);
    case MigState::Recovery: return tick_recovery(now);
    default: return {};
    }
  }

  /* The demo's RX retune finished. */
  std::vector<MigAction> on_retune_done(int64_t now) {
    if (st_ != MigState::Switching)
      return {};
    if (follow_pending_) {
      /* we were retuning to follow the drone's reported settled channel */
      follow_pending_ = false;
      st_ = MigState::Stable;
      current_ = follow_channel_;
      const uint8_t code = follow_channel_.same_rf(commit_target_) ? 0 : 1;
      return {ev(), gate(code), done(code)};
    }
    st_ = MigState::Verifying;
    verify_deadline_ms_ = activate_local_ms_ + p_.verify_ms;
    markers_seen_ = 0;
    video_seen_ = 0;
    got_gen_marker_ = false;
    verified_ = false;
    current_ = commit_target_;
    return {ev()};
  }

  /* A canonical-SA video frame decoded on the current channel. */
  std::vector<MigAction> on_video(int64_t now) {
    if (st_ == MigState::Verifying) {
      if (now < blank_until_ms_ || !got_gen_marker_)
        return {}; /* blank window / pre-marker: stale, ignore */
      ++video_seen_;
      return maybe_confirm(now);
    }
    if (st_ == MigState::Recovery)
      return recover_to(recov_channel(), now);
    return {};
  }

private:
  MigMsg make_proposal() const {
    MigMsg m;
    m.type = MT_PROPOSAL;
    m.link_id = link_id_;
    m.ground_epoch = epoch_;
    m.generation = gen_;
    m.source = source_;
    m.target = target_;
    m.evidence_gen = evidence_gen_;
    m.fallback_mode = 0;
    m.rescue = rescue_;
    m.ground_nonce = nonce_;
    return m;
  }
  MigMsg make_status_ack() const {
    MigMsg m;
    m.type = MT_STATUS;
    m.link_id = link_id_;
    m.role = 0;
    m.sender_epoch = epoch_;
    m.generation = gen_;
    m.state = static_cast<uint8_t>(st_);
    m.current = current_;
    m.peer_nonce_echo = drone_nonce_; /* THE commit-ack */
    return m;
  }
  MigMsg make_confirm() const {
    MigMsg m;
    m.type = MT_CONFIRM;
    m.link_id = link_id_;
    m.ground_epoch = epoch_;
    m.generation = gen_;
    m.drone_nonce_echo = drone_nonce_;
    m.marker_count = static_cast<uint16_t>(markers_seen_);
    m.video_frames = static_cast<uint16_t>(video_seen_);
    return m;
  }
  MigAction send(MigMsgType t, const MigMsg &m, bool unicast) const {
    MigAction a;
    a.kind = unicast ? MigAction::SendUnicast : MigAction::SendBroadcast;
    a.msg = m;
    a.msg.type = t;
    return a;
  }
  MigAction retune(const ChannelDef &c) const {
    MigAction a;
    a.kind = MigAction::RetuneTo;
    a.channel = c;
    return a;
  }
  MigAction ev() const {
    MigAction a;
    a.kind = MigAction::EmitEvent;
    a.code = static_cast<uint8_t>(st_);
    return a;
  }
  MigAction gate(uint8_t code) const {
    MigAction a;
    a.kind = MigAction::GateNotify;
    a.code = code;
    return a;
  }
  MigAction done(uint8_t code) const {
    MigAction a;
    a.kind = MigAction::Done;
    a.code = code;
    return a;
  }

  std::vector<MigAction> on_commit(const MigMsg &m, int64_t now) {
    /* nonce + generation gate — the replay/forgery wall (I5). */
    if (m.generation != gen_ || m.ground_nonce_echo != nonce_)
      return {};
    if (st_ == MigState::Proposed) {
      drone_epoch_ = m.drone_epoch;
      drone_nonce_ = m.drone_nonce;
      commit_target_ = m.target;
      commit_rescue_ = m.rescue;
      activate_tsf_ = m.activate_tsf;
      commit_tx_tsf_ = m.tx_tsf;
      const int64_t offset_ms =
          static_cast<int64_t>((activate_tsf_ - commit_tx_tsf_) / 1000);
      activate_local_ms_ = now + offset_ms;
      retune_at_ms_ = activate_local_ms_ - p_.lead_ms;
      if (retune_at_ms_ < now)
        retune_at_ms_ = now;
      st_ = MigState::Committed;
      next_status_ms_ = now;
      /* The ground follows the drone's authoritative settled STATUS; this
       * deadline (past the drone's own rollback deadline) is the backstop for
       * a total control-path loss, after which the ground recovers by scan. */
      follow_deadline_ms_ = activate_local_ms_ + p_.rollback_ms + 2000;
      std::vector<MigAction> a;
      a.push_back(ev());
      auto t = tick_committed(now); /* sends the first ack immediately */
      a.insert(a.end(), t.begin(), t.end());
      return a;
    }
    if (st_ == MigState::Committed) /* drone retransmit: re-ack */
      return {send(MT_STATUS, make_status_ack(), false)};
    return {};
  }

  std::vector<MigAction> on_status(const MigMsg &m, int64_t now) {
    if (st_ == MigState::Proposed && m.reason != 0) {
      /* the drone rejected our proposal (source mismatch / illegal / busy) */
      st_ = MigState::Stable;
      current_ = source_;
      MigAction e;
      e.kind = MigAction::EmitEvent;
      e.code = m.reason;
      return {e, done(1)};
    }
    /* Follow the drone's authoritative settled channel. The drone is the
     * schedule authority: once it reports Stable somewhere, the ground goes
     * there (this is what closes every dropped-confirm / rollback split-brain
     * — the ground never unilaterally commits to a channel the drone left). */
    if (m.role == 1 && m.generation == gen_ &&
        (st_ == MigState::Committed || st_ == MigState::Switching ||
         st_ == MigState::Verifying) &&
        m.state == static_cast<uint8_t>(MigState::Stable))
      return follow_drone(m.current, now);
    return {};
  }

  std::vector<MigAction> follow_drone(const ChannelDef &chan, int64_t now) {
    (void)now;
    if (current_.same_rf(chan)) {
      st_ = MigState::Stable;
      current_ = chan;
      const uint8_t code = chan.same_rf(commit_target_) ? 0 : 1;
      std::vector<MigAction> a;
      if (chan.same_rf(commit_target_))
        a.push_back(send(MT_CONFIRM, make_confirm(), true)); /* ack the drone */
      a.push_back(ev());
      a.push_back(gate(code));
      a.push_back(done(code));
      return a;
    }
    /* retune to where the drone settled, then go Stable there */
    follow_channel_ = chan;
    follow_pending_ = true;
    st_ = MigState::Switching;
    return {retune(chan), ev()};
  }

  std::vector<MigAction> on_marker(const MigMsg &m, int64_t now) {
    if (m.generation != gen_ || m.drone_epoch != drone_epoch_)
      return {}; /* foreign / stale generation */
    if (st_ == MigState::Verifying) {
      if (!m.aired_on.same_rf(commit_target_)) {
        ++wrong_channel_; /* delayed-USB / wrong-channel frame */
        return {};
      }
      if (now < blank_until_ms_)
        return {};
      got_gen_marker_ = true;
      ++markers_seen_;
      return maybe_confirm(now);
    }
    if (st_ == MigState::Recovery)
      return recover_to(m.aired_on, now);
    return {};
  }

  std::vector<MigAction> on_abort(const MigMsg &m, int64_t now) {
    if (m.effective == 0) { /* pre-activation cancel: nothing moved */
      st_ = MigState::Stable;
      current_ = source_;
      return {ev(), done(1)};
    }
    /* drone rolled back — scan to reunite */
    enter_recovery(now);
    return {ev(), retune(recov_channel())};
  }

  std::vector<MigAction> tick_proposed(int64_t now) {
    if (now < next_send_ms_)
      return {};
    if (tries_ >= p_.proposal_max_tries) {
      st_ = MigState::Stable;
      current_ = source_;
      return {ev(), done(1)}; /* gave up; stayed on old */
    }
    ++tries_;
    next_send_ms_ = now + p_.proposal_interval_ms;
    return {send(MT_PROPOSAL, make_proposal(), true)};
  }

  std::vector<MigAction> tick_committed(int64_t now) {
    std::vector<MigAction> a;
    if (now >= next_status_ms_) {
      a.push_back(send(MT_STATUS, make_status_ack(), false));
      next_status_ms_ = now + p_.commit_interval_ms;
    }
    if (now >= retune_at_ms_) {
      st_ = MigState::Switching;
      blank_until_ms_ = now + p_.blank_window_ms;
      a.push_back(retune(commit_target_));
      a.push_back(ev());
    }
    return a;
  }

  std::vector<MigAction> tick_verifying(int64_t now) {
    std::vector<MigAction> a;
    /* Once the markers prove the channel, tell the drone we followed (CONFIRM,
     * repeated) — but do NOT declare success: the ground reaches Stable only
     * when the drone's own STATUS says it settled on the target (follow_drone),
     * so a dropped CONFIRM + drone rollback can never strand the ground here. */
    if (verified_ && now >= next_confirm_ms_) {
      a.push_back(send(MT_CONFIRM, make_confirm(), true));
      next_confirm_ms_ = now + p_.commit_interval_ms;
    }
    /* Control-path-loss backstop: no authoritative drone STATUS by the follow
     * deadline -> scan to reunite. */
    if (now >= follow_deadline_ms_) {
      enter_recovery(now);
      a.push_back(ev());
      a.push_back(retune(recov_channel()));
    }
    return a;
  }

  std::vector<MigAction> tick_recovery(int64_t now) {
    if (now < recov_next_ms_)
      return {};
    recov_idx_ = (recov_idx_ + 1) % recov_len_;
    recov_next_ms_ = now + p_.recovery_dwell_ms;
    std::vector<MigAction> a{retune(recov_channel())};
    if (now - recov_start_ms_ > p_.recovery_giveup_ms) {
      MigAction e;
      e.kind = MigAction::EmitEvent;
      e.code = 255; /* recovery exhausted */
      a.push_back(e);
    }
    return a;
  }

  std::vector<MigAction> maybe_confirm(int64_t now) {
    /* Markers proved the channel — start confirming (the drone needs to learn
     * the ground followed), but the ground stays Verifying until the drone's
     * STATUS confirms it settled (follow_drone). */
    if (!verified_ &&
        (markers_seen_ >= p_.verify_markers || video_seen_ >= p_.verify_video)) {
      verified_ = true;
      next_confirm_ms_ = now + p_.commit_interval_ms;
      return {send(MT_CONFIRM, make_confirm(), true)};
    }
    return {};
  }

  void enter_recovery(int64_t now) {
    recov_pattern_[0] = source_;
    recov_pattern_[1] = commit_target_;
    recov_pattern_[2] = source_;
    recov_pattern_[3] = commit_rescue_;
    recov_len_ = 4;
    recov_idx_ = 0;
    recov_start_ms_ = now;
    recov_next_ms_ = now + p_.recovery_dwell_ms;
    st_ = MigState::Recovery;
    current_ = recov_channel();
  }
  ChannelDef recov_channel() const { return recov_pattern_[recov_idx_]; }

  std::vector<MigAction> recover_to(const ChannelDef &c, int64_t now) {
    (void)now;
    st_ = MigState::Stable;
    current_ = c;
    const uint8_t code = c.same_rf(commit_target_) ? 0 : 1;
    return {ev(), gate(code), done(code)};
  }

  MigParams p_;
  uint32_t link_id_, epoch_;
  ChannelDef source_, current_;
  MigState st_ = MigState::Stable;

  /* current exchange */
  uint32_t gen_ = 0, nonce_ = 0, evidence_gen_ = 0;
  ChannelDef target_, rescue_;
  int tries_ = 0;
  int64_t next_send_ms_ = 0;

  /* committed */
  uint32_t drone_epoch_ = 0, drone_nonce_ = 0;
  ChannelDef commit_target_, commit_rescue_;
  uint64_t activate_tsf_ = 0, commit_tx_tsf_ = 0;
  int64_t activate_local_ms_ = 0, retune_at_ms_ = 0, next_status_ms_ = 0;

  /* verify */
  int64_t verify_deadline_ms_ = 0, blank_until_ms_ = 0, next_confirm_ms_ = 0;
  int64_t follow_deadline_ms_ = 0;
  int markers_seen_ = 0, video_seen_ = 0;
  bool got_gen_marker_ = false, verified_ = false;
  uint32_t wrong_channel_ = 0;
  /* following the drone's authoritative settled channel */
  ChannelDef follow_channel_;
  bool follow_pending_ = false;

  /* recovery */
  ChannelDef recov_pattern_[4];
  int recov_len_ = 0, recov_idx_ = 0;
  int64_t recov_next_ms_ = 0, recov_start_ms_ = 0;
};

} /* namespace chanmig */
} /* namespace devourer */

#endif /* DEVOURER_CHANMIG_MIG_PROPOSER_H */
