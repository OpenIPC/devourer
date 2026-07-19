/* MigResponder — the DRONE (video TX) state machine (pure).
 *
 * The drone is the final schedule authority: it validates a ground proposal,
 * fixes an absolute activation instant on its own TSF, and — crucially —
 * arms that activation ONLY after it has seen the ground echo its nonce (the
 * commit-ack). That single rule closes every replay-initiated and
 * control-loss split-brain row: a commit the current ground never heard is
 * never activated, so the drone can never strand itself on a channel the
 * ground isn't following.
 *
 * At activation it drains/deterministically-drops pending TX, retunes through
 * the safe gate, re-arms its ACK responder, and airs generation-tagged low-MCS
 * markers; if the ground never confirms by the rollback deadline it returns to
 * the fallback channel. Pure: inputs are decoded MigMsgs + local observations
 * + monotonic `now` + the drone's own TSF (needed once, to fix the absolute
 * activation instant); outputs are MigActions.
 *
 * The #280 target-validation hook lives in validate_target(); stage 3 ships
 * the variant-A instant checks (legality / caps / allowed-list), stage 5 adds
 * the pre-commit probe. */
#ifndef DEVOURER_CHANMIG_MIG_RESPONDER_H
#define DEVOURER_CHANMIG_MIG_RESPONDER_H

#include <vector>

#include "chanmig/MigConfig.h"
#include "chanmig/MigProposer.h" /* mig_mix_nonce */
#include "chanmig/MigTypes.h"
#include "chanmig/MigWire.h"     /* ReplayWindow */

namespace devourer {
namespace chanmig {

/* Static capabilities + policy the drone validates a target against. */
struct MigCaps {
  uint8_t bands = 0x02 | 0x04; /* bit1 = 2.4, bit2 = 5 (matches ChannelDef.band as a set) */
  uint8_t widths = 0xFF;       /* 1<<ChannelWidth_t supported */
  std::vector<ChannelDef> allowed; /* empty = any legal channel */
  bool band_ok(uint8_t band) const {
    if (band == 2)
      return bands & 0x02;
    if (band == 5)
      return bands & 0x04;
    return false;
  }
  bool width_ok(ChannelWidth_t w) const { return widths & (1u << w); }
  bool allowed_ok(const ChannelDef &d) const {
    if (allowed.empty())
      return true;
    for (const ChannelDef &c : allowed)
      if (c.same_rf(d))
        return true;
    return false;
  }
  /* #280 variant B: opt-in single-radio pre-commit probe (research). Off by
   * default — variant A (these checks) is the product baseline. When on, the
   * drone briefly retunes to the target before committing and vetoes an
   * extreme-occupancy destination. */
  bool probe = false;
  double veto_busy_frac = 0.6;
};

class MigResponder {
public:
  MigResponder(MigParams params, uint32_t link_id, uint32_t drone_epoch,
               ChannelDef current, MigCaps caps)
      : p_(params), link_id_(link_id), epoch_(drone_epoch), current_(current),
        caps_(caps) {}

  MigState state() const { return st_; }
  ChannelDef current_channel() const { return current_; }
  uint32_t generation() const { return gen_; }

  std::vector<MigAction> on_message(const MigMsg &m, int64_t now,
                                    uint64_t now_tsf) {
    switch (m.type) {
    case MT_PROPOSAL: return on_proposal(m, now, now_tsf);
    case MT_STATUS: return on_status(m, now);
    case MT_CONFIRM: return on_confirm(m, now);
    case MT_ABORT: return {}; /* ground doesn't abort us */
    default: return {};
    }
  }

  std::vector<MigAction> on_tick(int64_t now, uint64_t now_tsf) {
    std::vector<MigAction> a;
    /* Periodic STATUS is the ground's authoritative signal for where the
     * drone actually settled — the ground FOLLOWS it, so it must keep flowing
     * for a grace window after the drone reaches a terminal state. */
    if (now >= next_status_ms_ &&
        (st_ != MigState::Stable || now < settle_status_until_ms_) &&
        gen_ != 0) {
      a.push_back(send(make_status(), false));
      next_status_ms_ = now + p_.status_interval_ms;
    }
    switch (st_) {
    case MigState::Committed: {
      auto t = tick_committed(now, now_tsf);
      a.insert(a.end(), t.begin(), t.end());
      break;
    }
    case MigState::Switching: {
      /* Drain-stall backstop: if the drain callback never lands by its
       * deadline, drop the pending TX deterministically and retune anyway —
       * the migration must not hang on a stuck USB completion. */
      if (phase_ == Phase::Draining && now >= drain_deadline_ms_) {
        phase_ = Phase::Retuning;
        a.push_back(retune(commit_target_));
      }
      break;
    }
    case MigState::Verifying: {
      auto t = tick_verifying(now, now_tsf);
      a.insert(a.end(), t.begin(), t.end());
      break;
    }
    default: break;
    }
    return a;
  }

  std::vector<MigAction> on_drain_done(int64_t now) {
    (void)now;
    if (st_ == MigState::Switching && phase_ == Phase::Draining) {
      phase_ = Phase::Retuning;
      return {retune(commit_target_)};
    }
    if (st_ == MigState::Validating && pp_ == ProbePhase::Drain) {
      pp_ = ProbePhase::Retune;
      return {retune(commit_target_)}; /* probe: hop to the target to sample */
    }
    return {};
  }

  /* #280 variant B: the demo samples GetRxEnergy after the probe retune settles
   * and reports the destination's occupancy here. valid=false ⇒ unknown. */
  std::vector<MigAction> on_probe_sample(double busy_frac, bool valid,
                                         int64_t now) {
    (void)now;
    if (st_ != MigState::Validating || pp_ != ProbePhase::Sampling)
      return {};
    probe_busy_ = busy_frac;
    probe_valid_ = valid;
    probe_veto_ = valid && busy_frac > caps_.veto_busy_frac;
    pp_ = ProbePhase::Return;
    return {retune(commit_source_)}; /* always return to source before deciding */
  }

  std::vector<MigAction> on_retune_done(bool ok, int64_t now, uint64_t now_tsf) {
    (void)now;
    (void)now_tsf;
    if (st_ == MigState::Switching && phase_ == Phase::Retuning) {
      if (ok) {
        current_ = commit_target_;
        st_ = MigState::Verifying;
        MigAction arm;
        arm.kind = MigAction::ArmMarkers;
        arm.channel = commit_target_;
        MigAction rp;
        rp.kind = MigAction::ResumePump;
        return {arm, rp, ev()};
      }
      /* retune failed: fall back to the source, then rescue */
      phase_ = Phase::RollbackRetune;
      current_ = fallback_channel(); /* provisional */
      return {retune(fallback_channel())};
    }
    if (st_ == MigState::Switching && phase_ == Phase::RollbackRetune) {
      if (ok) {
        current_ = fallback_channel();
        return finish_rollback(now);
      }
      /* even the fallback retune failed: rescue + abort */
      current_ = commit_rescue_;
      st_ = MigState::Stable;
      hold_down_until_ms_ = now + p_.proposal_hold_down_ms;
      return {retune(commit_rescue_), abort_msg(MigReason::RetuneFail, 1),
              gate(1), done(2)};
    }
    if (st_ == MigState::Rollback) {
      current_ = fallback_channel();
      return finish_rollback(now);
    }
    /* --- probe (variant B) retune callbacks --- */
    if (st_ == MigState::Validating && pp_ == ProbePhase::Retune) {
      if (ok) {
        current_ = commit_target_; /* on the target, awaiting the sample */
        pp_ = ProbePhase::Sampling;
        MigAction e; /* tell the demo to sample GetRxEnergy now */
        e.kind = MigAction::EmitEvent;
        e.code = kProbeSampleNow;
        return {e};
      }
      /* couldn't even reach the target: return to source, veto */
      probe_veto_ = true;
      probe_valid_ = false;
      pp_ = ProbePhase::Return;
      return {retune(commit_source_)};
    }
    if (st_ == MigState::Validating && pp_ == ProbePhase::Return) {
      current_ = ok ? commit_source_ : commit_rescue_;
      pp_ = ProbePhase::None;
      MigAction resume;
      resume.kind = MigAction::ResumePump;
      if (!ok) {
        /* return retune failed — land on rescue, reject the migration */
        st_ = MigState::Stable;
        rw_.clear_in_flight();
        return {resume,
                validation_msg(commit_target_, 1, 1, MigReason::RetuneFail),
                status_reject(ground_epoch_, gen_, MigReason::RetuneFail),
                ev()};
      }
      if (probe_veto_) {
        st_ = MigState::Stable;
        rw_.clear_in_flight();
        return {resume,
                validation_msg(commit_target_, 1, 1, MigReason::Veto),
                status_reject(ground_epoch_, gen_, MigReason::Veto), ev()};
      }
      /* accepted — report it and commit for real */
      auto acts = enter_committed(now, now_tsf);
      acts.insert(acts.begin(), resume);
      acts.insert(acts.begin() + 1,
                  validation_msg(commit_target_, 1, 0, MigReason::None));
      return acts;
    }
    return {};
  }

private:
  enum class Phase { None, Draining, Retuning, RollbackRetune };
  enum class ProbePhase { None, Drain, Retune, Sampling, Return };
  static constexpr uint8_t kProbeSampleNow = 200;

  std::vector<MigAction> enter_probe(int64_t now) {
    (void)now;
    st_ = MigState::Validating;
    pp_ = ProbePhase::Drain;
    probe_veto_ = false;
    probe_valid_ = false;
    MigAction d;
    d.kind = MigAction::StartDrain;
    return {d, ev()};
  }

  MigAction validation_msg(const ChannelDef &target, uint8_t method,
                           uint8_t result, MigReason reason) const {
    MigMsg m = base(MT_VALIDATION);
    m.drone_epoch = epoch_;
    m.generation = gen_;
    m.target = target;
    m.method = method;
    m.result = result;
    m.reason = static_cast<uint8_t>(reason);
    m.nhm_busy_pct = static_cast<uint8_t>(probe_busy_ * 100.0);
    m.energy_valid = probe_valid_ ? 1 : 0;
    return send(m, false);
  }

  MigMsg base(MigMsgType t) const {
    MigMsg m;
    m.type = t;
    m.link_id = link_id_;
    return m;
  }
  MigAction send(const MigMsg &m, bool unicast) const {
    MigAction a;
    a.kind = unicast ? MigAction::SendUnicast : MigAction::SendBroadcast;
    a.msg = m;
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
  MigAction abort_msg(MigReason r, uint8_t effective) const {
    MigMsg m = base(MT_ABORT);
    m.role = 1;
    m.sender_epoch = epoch_;
    m.generation = gen_;
    m.reason = static_cast<uint8_t>(r);
    m.effective = effective;
    return send(m, false);
  }
  MigAction status_reject(uint32_t ground_epoch, uint32_t gen,
                          MigReason r) const {
    MigMsg m = base(MT_STATUS);
    m.role = 1;
    m.sender_epoch = epoch_;
    m.generation = gen;
    m.state = static_cast<uint8_t>(MigState::Stable);
    m.current = current_;
    m.reason = static_cast<uint8_t>(r);
    (void)ground_epoch;
    return send(m, false);
  }
  MigMsg make_status() const {
    MigMsg m = base(MT_STATUS);
    m.role = 1;
    m.sender_epoch = epoch_;
    m.generation = gen_;
    m.state = static_cast<uint8_t>(st_);
    m.current = current_;
    return m;
  }
  ChannelDef fallback_channel() const {
    return fallback_mode_ == 1 ? commit_rescue_ : commit_source_;
  }

  /* #280 variant-A validation: legality + caps + allowed list. */
  MigReason validate_target(const ChannelDef &t) const {
    if (validate(t) != DefError::Ok)
      return MigReason::IllegalTarget;
    if (!caps_.band_ok(t.band))
      return MigReason::UnsupportedWidth;
    if (!caps_.width_ok(t.width))
      return MigReason::UnsupportedWidth;
    if (t.no_ir)
      return MigReason::IllegalTarget;
    if (!caps_.allowed_ok(t))
      return MigReason::IllegalTarget;
    return MigReason::None;
  }

  std::vector<MigAction> on_proposal(const MigMsg &m, int64_t now,
                                     uint64_t now_tsf) {
    /* epoch handling: a proposal may introduce/adopt a ground epoch. A ground
     * restart (fresh epoch) orphans our in-flight migration (I4). Crucially,
     * if we have already left the source channel, we must ROLL BACK to it —
     * the restarted ground believes we are on source, so sitting on the target
     * would strand us (a split-brain). Reject the triggering proposal as Busy
     * while the rollback runs; the ground reunites with us on source. */
    const bool epoch_change =
        rw_.have_peer && m.ground_epoch != rw_.peer_epoch;
    if (epoch_change && st_ != MigState::Stable) {
      const bool moved = gen_ != 0 && !current_.same_rf(commit_source_);
      phase_ = Phase::None;
      rw_.accept_epoch(m.ground_epoch, /*may_introduce=*/true);
      if (moved) {
        st_ = MigState::Rollback;
        rw_.clear_in_flight();
        return {retune(commit_source_),
                status_reject(m.ground_epoch, m.generation, MigReason::Busy),
                ev()};
      }
      st_ = MigState::Stable; /* still on source: just orphan and re-evaluate */
    }
    if (!rw_.accept_epoch(m.ground_epoch, /*may_introduce=*/true))
      return {status_reject(m.ground_epoch, m.generation, MigReason::StaleEpoch)};

    const int cls = rw_.classify_proposal(m.generation);
    if (cls == 0) /* replay */
      return {};
    if (cls == 1) { /* idempotent: re-send the cached commit */
      if (st_ == MigState::Committed)
        return {send(cached_commit_, true)};
      return {};
    }
    /* fresh proposal */
    if (st_ != MigState::Stable || now < hold_down_until_ms_) {
      rw_.clear_in_flight();
      return {status_reject(m.ground_epoch, m.generation, MigReason::Busy)};
    }
    /* source must match where we actually are */
    if (!m.source.same_rf(current_)) {
      rw_.clear_in_flight();
      return {status_reject(m.ground_epoch, m.generation,
                            MigReason::SourceMismatch)};
    }
    const MigReason vr = validate_target(m.target); /* variant A checks */
    if (vr != MigReason::None) {
      rw_.clear_in_flight();
      return {status_reject(m.ground_epoch, m.generation, vr),
              validation_msg(m.target, 0 /*A*/, 1 /*veto*/, vr)};
    }

    /* stash the accepted proposal (used by enter_committed, and by the probe
     * before it commits). */
    gen_ = m.generation;
    ground_epoch_ = m.ground_epoch;
    ground_nonce_ = m.ground_nonce;
    commit_target_ = m.target;
    commit_source_ = m.source;
    commit_rescue_ = m.rescue;
    fallback_mode_ = m.fallback_mode;

    /* Variant B: run the single-radio pre-commit probe before committing.
     * Off by default (variant A is the product baseline). */
    if (caps_.probe)
      return enter_probe(now);

    return enter_committed(now, now_tsf);
  }

  /* Build the commit + arm the Committed timers from the stashed proposal. */
  std::vector<MigAction> enter_committed(int64_t now, uint64_t now_tsf) {
    drone_nonce_ = mig_mix_nonce(epoch_, gen_, 0xD2);
    activate_tsf_ = now_tsf + p_.countdown_ms * 1000;
    rollback_deadline_tsf_ = activate_tsf_ + p_.rollback_ms * 1000;
    armed_ = false;
    ack_deadline_ms_ = now + p_.ack_timeout_ms;
    next_commit_ms_ = now + p_.commit_interval_ms;
    next_status_ms_ = now + p_.status_interval_ms;
    cached_commit_ = make_commit();
    st_ = MigState::Committed;
    return {send(cached_commit_, true), ev()};
  }

  MigMsg make_commit() const {
    MigMsg m = base(MT_COMMIT);
    m.drone_epoch = epoch_;
    m.ground_epoch = ground_epoch_;
    m.generation = gen_;
    m.target = commit_target_;
    m.activate_tsf = activate_tsf_;
    m.confirm_window_us = static_cast<uint32_t>(p_.verify_ms * 1000);
    m.rollback_deadline_tsf = rollback_deadline_tsf_;
    m.rescue = commit_rescue_;
    m.ground_nonce_echo = ground_nonce_;
    m.drone_nonce = drone_nonce_;
    m.armed = 0;
    return m;
  }

  std::vector<MigAction> on_status(const MigMsg &m, int64_t now) {
    (void)now;
    /* the ground ack: it echoes OUR nonce. */
    if (st_ == MigState::Committed && m.peer_nonce_echo == drone_nonce_ &&
        m.generation == gen_)
      armed_ = true;
    return {};
  }

  std::vector<MigAction> on_confirm(const MigMsg &m, int64_t now) {
    (void)now;
    if (st_ == MigState::Verifying && m.generation == gen_ &&
        m.drone_nonce_echo == drone_nonce_) {
      st_ = MigState::Stable;
      current_ = commit_target_;
      settle_status_until_ms_ = now + kSettleStatusMs;
      next_status_ms_ = now; /* let the ground learn we settled on target */
      MigAction stop;
      stop.kind = MigAction::StopMarkers;
      return {stop, ev(), gate(0), done(0)};
    }
    return {};
  }

  std::vector<MigAction> tick_committed(int64_t now, uint64_t now_tsf) {
    std::vector<MigAction> a;
    if (!armed_ && now >= ack_deadline_ms_) {
      /* never heard the ground ack: abort, never activate unheard */
      st_ = MigState::Stable;
      settle_status_until_ms_ = now + kSettleStatusMs;
      next_status_ms_ = now;
      rw_.clear_in_flight();
      a.push_back(abort_msg(MigReason::NoAck, 0));
      a.push_back(ev());
      return a;
    }
    if (now >= next_commit_ms_) {
      a.push_back(send(cached_commit_, true)); /* tx_tsf re-stamped by demo */
      next_commit_ms_ = now + p_.commit_interval_ms;
    }
    const uint64_t drain_budget_tsf =
        static_cast<uint64_t>(p_.drain_deadline_ms) * 1000;
    if (armed_ && now_tsf + drain_budget_tsf >= activate_tsf_) {
      st_ = MigState::Switching;
      phase_ = Phase::Draining;
      drain_deadline_ms_ = now + p_.drain_deadline_ms;
      MigAction d;
      d.kind = MigAction::StartDrain;
      a.push_back(d);
      a.push_back(ev());
    }
    return a;
  }

  std::vector<MigAction> tick_verifying(int64_t now, uint64_t now_tsf) {
    if (now_tsf >= rollback_deadline_tsf_) {
      st_ = MigState::Rollback;
      (void)now;
      MigAction stop;
      stop.kind = MigAction::StopMarkers;
      return {stop, retune(fallback_channel()), ev()};
    }
    return {};
  }

  std::vector<MigAction> finish_rollback(int64_t now) {
    st_ = MigState::Stable;
    hold_down_until_ms_ = now + p_.proposal_hold_down_ms;
    settle_status_until_ms_ = now + kSettleStatusMs;
    next_status_ms_ = now; /* tell the ground we returned to the fallback */
    rw_.clear_in_flight();
    MigAction stop;
    stop.kind = MigAction::StopMarkers;
    return {stop, ev(), gate(1), done(1)};
  }

  static constexpr int64_t kSettleStatusMs = 8000;

  MigParams p_;
  uint32_t link_id_, epoch_;
  ChannelDef current_;
  MigCaps caps_;
  MigState st_ = MigState::Stable;
  Phase phase_ = Phase::None;
  ReplayWindow rw_;

  uint32_t gen_ = 0, ground_epoch_ = 0, ground_nonce_ = 0, drone_nonce_ = 0;
  ChannelDef commit_target_, commit_source_, commit_rescue_;
  uint8_t fallback_mode_ = 0;
  uint64_t activate_tsf_ = 0, rollback_deadline_tsf_ = 0;
  bool armed_ = false;
  int64_t ack_deadline_ms_ = 0, next_commit_ms_ = 0, hold_down_until_ms_ = 0;
  int64_t next_status_ms_ = 0, settle_status_until_ms_ = 0, drain_deadline_ms_ = 0;
  MigMsg cached_commit_;
  /* variant-B probe */
  ProbePhase pp_ = ProbePhase::None;
  double probe_busy_ = 0.0;
  bool probe_valid_ = false, probe_veto_ = false;
};

} /* namespace chanmig */
} /* namespace devourer */

#endif /* DEVOURER_CHANMIG_MIG_RESPONDER_H */
