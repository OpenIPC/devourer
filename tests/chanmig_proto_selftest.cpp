/* MigProposer + MigResponder failure-matrix selftest — the #278 convergence
 * gate. A LinkSim couples the two pure machines through drop/duplicate/reorder/
 * delay queues on a virtual clock, and drives the 14-row failure matrix plus a
 * drop-every-single-message sweep. Every row asserts the terminal invariant:
 * both endpoints converge to a shared channel in {source, target, rescue}
 * within a bounded time — never a lasting split-brain. */
#include "chanmig/MigProposer.h"
#include "chanmig/MigResponder.h"

#include <cstdio>
#include <deque>
#include <optional>
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

static ChannelDef mkc(uint8_t prim, ChannelWidth_t w = CHANNEL_WIDTH_20,
                      uint8_t off = 0) {
  ChannelDef d;
  d.band = prim <= 14 ? 2 : 5;
  d.primary = prim;
  d.width = w;
  d.offset = off;
  normalize(d);
  return d;
}

/* Fault knobs for one run. */
struct Fault {
  std::vector<DropRule> drop_g2d; /* messages ground->drone */
  std::vector<DropRule> drop_d2g;
  int64_t g_restart_ms = -1; /* restart the ground machine (fresh epoch) at */
  int64_t d_restart_ms = -1;
  bool drone_retune_fails = false; /* first drone retune returns !ok */
  bool target_silent = false;      /* target channel: no markers AND no video */
  bool duplicate = false;          /* deliver every message twice (+reorder) */
  bool drop_drain = false;         /* never deliver DrainDone (drain stall) */
  bool illegal_target = false;     /* drone caps reject the target */
  int64_t stale_marker_ms = -1;    /* inject a wrong-channel/old-gen marker at */
  bool drone_probe = false;        /* variant-B pre-commit probe enabled */
  double probe_busy = 0.1;         /* the target occupancy the probe measures */
};

struct Outcome {
  bool converged = false;
  bool split_brain = false;
  ChannelDef channel;
  int64_t time_ms = 0;
  bool ground_confirmed = false, drone_rolledback = false;
};

class LinkSim {
public:
  LinkSim(ChannelDef source, ChannelDef target, ChannelDef rescue, Fault f)
      : source_(source), target_(target), rescue_(rescue), f_(f) {
    rebuild_ground(0x11110000u);
    rebuild_drone(0x22220000u);
  }

  Outcome run(int64_t max_ms) {
    /* kick off the proposal at t=100 */
    bool started = false;
    for (now_ = 0; now_ <= max_ms; now_ += kTick) {
      /* scheduled restarts */
      if (f_.g_restart_ms >= 0 && now_ >= f_.g_restart_ms && !g_restarted_) {
        g_restarted_ = true;
        rebuild_ground(0x33330000u);
        started = false; /* new ground: re-propose */
      }
      if (f_.d_restart_ms >= 0 && now_ >= f_.d_restart_ms && !d_restarted_) {
        d_restarted_ = true;
        rebuild_drone(0x44440000u);
      }
      if (!started && now_ >= 100) {
        started = true;
        process(g_->start(target_, 7, rescue_, now_), true);
      }
      deliver_due();
      /* markers from the drone while armed (a silent/interfered target lands
       * none) */
      if (drone_markers_on_ && !f_.target_silent && now_ >= next_marker_ms_) {
        next_marker_ms_ = now_ + 20;
        emit_marker();
      }
      /* a stale/wrong-channel marker delayed in the USB pipe post-switch */
      if (f_.stale_marker_ms >= 0 && now_ >= f_.stale_marker_ms &&
          !stale_marker_sent_) {
        stale_marker_sent_ = true;
        MigMsg m;
        m.type = MT_MARKER;
        m.link_id = kLink;
        m.drone_epoch = d_epoch();
        m.generation = d_->generation();
        m.aired_on = source_; /* wrong channel — a delayed old-channel frame */
        q_.push_back({now_ + kMsgLatency, true, m});
      }
      /* co-channel video delivery to the ground. A silent/interfered TARGET
       * passes nothing (only the target is bad); other channels still carry
       * video, so the recovery-via-video backstop works on source/rescue. */
      if (drone_pump_ && now_ >= next_video_ms_) {
        next_video_ms_ = now_ + 5;
        const bool co = g_->current_channel().same_rf(d_->current_channel());
        const bool silent_here =
            f_.target_silent && g_->current_channel().same_rf(target_);
        if (co && !silent_here)
          process(g_->on_video(now_), true);
      }
      process(g_->on_tick(now_), true);
      process(d_->on_tick(now_, tsf()), false);
      run_local(now_);

      if (trace_ && (g_->state() != last_g_ || d_->state() != last_d_)) {
        std::fprintf(stderr, "  t=%lld G=%s@%s D=%s@%s\n", (long long)now_,
                     mig_state_name(g_->state()),
                     g_->current_channel().str().c_str(),
                     mig_state_name(d_->state()),
                     d_->current_channel().str().c_str());
        last_g_ = g_->state();
        last_d_ = d_->state();
      }
      if (g_->state() != MigState::Stable || d_->state() != MigState::Stable)
        progressed_ = true;
      /* Only judge convergence once the protocol has actually run (both start
       * Stable and co-channel, which would be a trivial false match). Require
       * a short settle after progress so late in-flight frames land. */
      if (progressed_ && converged()) {
        if (settle_at_ < 0)
          settle_at_ = now_ + 200;
        else if (now_ >= settle_at_ && converged())
          return finish(true);
      } else {
        settle_at_ = -1;
      }
    }
    return finish(false);
  }

private:
  static constexpr int64_t kTick = 1;
  static constexpr int64_t kMsgLatency = 2;
  static constexpr int64_t kRetuneLatency = 3;
  static constexpr int64_t kDrainLatency = 5;

  uint64_t tsf() const { return kTsfBase + static_cast<uint64_t>(now_) * 1000; }

  void rebuild_ground(uint32_t epoch) {
    MigParams p;
    g_.emplace(p, kLink, epoch, source_);
  }
  void rebuild_drone(uint32_t epoch) {
    MigParams p;
    MigCaps caps; /* any legal 2.4/5 channel + width */
    if (f_.illegal_target)
      caps.allowed = {source_, rescue_}; /* target NOT in the allowed set */
    caps.probe = f_.drone_probe;
    d_.emplace(p, kLink, epoch, source_, caps);
    drone_markers_on_ = false;
    drone_pump_ = true;
  }

  struct Pending {
    int64_t at;
    bool to_ground;
    MigMsg msg;
  };
  struct LocalEv {
    int64_t at;
    bool ground;
    int kind; /* 0 retune-done, 1 drain-done */
    bool ok;
  };

  void process(const std::vector<MigAction> &acts, bool from_ground) {
    for (const MigAction &a : acts) {
      switch (a.kind) {
      case MigAction::SendUnicast:
      case MigAction::SendBroadcast: {
        MigMsg m = a.msg;
        if (!from_ground)
          m.tx_tsf = tsf();
        const int dir_from_ground = from_ground ? 1 : 0;
        int &idx = send_idx_[dir_from_ground][m.type];
        ++idx;
        const auto &rules = from_ground ? f_.drop_g2d : f_.drop_d2g;
        if (drop_matches(rules, m.type, idx))
          break;
        q_.push_back({now_ + kMsgLatency, !from_ground, m});
        /* duplication (+ mild reorder via a second, slightly-later copy) */
        if (f_.duplicate)
          q_.push_back({now_ + kMsgLatency + 3, !from_ground, m});
        break;
      }
      case MigAction::RetuneTo:
        local_.push_back({now_ + kRetuneLatency, from_ground, 0,
                          !(from_ground ? false : f_.drone_retune_fails &&
                                                       !drone_retuned_once_)});
        if (!from_ground)
          drone_retuned_once_ = true;
        break;
      case MigAction::StartDrain:
        if (!f_.drop_drain) /* drop_drain: DrainDone never lands (drain stall) */
          local_.push_back({now_ + kDrainLatency, false, 1, true});
        break;
      case MigAction::ArmMarkers:
        drone_markers_on_ = true;
        next_marker_ms_ = now_;
        break;
      case MigAction::StopMarkers:
        drone_markers_on_ = false;
        break;
      case MigAction::ResumePump:
        drone_pump_ = true;
        break;
      case MigAction::GateNotify:
        if (from_ground && a.code == 0)
          ground_confirmed_ = true;
        if (!from_ground && a.code == 1)
          drone_rolledback_ = true;
        break;
      case MigAction::EmitEvent:
        /* the drone's probe asks the host to sample GetRxEnergy now
         * (kProbeSampleNow = 200) */
        if (!from_ground && a.code == 200)
          process(d_->on_probe_sample(f_.probe_busy, true, now_), false);
        break;
      case MigAction::Done:
        break;
      }
    }
  }

  /* Remove-then-process: processing a message/event can enqueue new ones, so
   * we must extract the due items and shrink the queue BEFORE dispatching —
   * otherwise a follow-up scheduled during dispatch is clobbered. */
  void deliver_due() {
    std::vector<Pending> due;
    std::deque<Pending> keep;
    for (auto &pm : q_) {
      if (pm.at <= now_)
        due.push_back(pm);
      else
        keep.push_back(pm);
    }
    q_ = std::move(keep);
    for (auto &pm : due) {
      if (pm.to_ground)
        process(g_->on_message(pm.msg, now_), true);
      else
        process(d_->on_message(pm.msg, now_, tsf()), false);
    }
  }

  void run_local(int64_t now) {
    std::vector<LocalEv> due, keep;
    for (auto &e : local_)
      (e.at <= now ? due : keep).push_back(e);
    local_ = keep;
    for (auto &e : due) {
      if (e.kind == 0) {
        if (e.ground)
          process(g_->on_retune_done(now), true);
        else
          process(d_->on_retune_done(e.ok, now, tsf()), false);
      } else {
        process(d_->on_drain_done(now), false);
      }
    }
  }

  void emit_marker() {
    MigMsg m;
    m.type = MT_MARKER;
    m.link_id = kLink;
    m.drone_epoch = d_epoch();
    m.generation = d_->generation();
    m.aired_on = d_->current_channel();
    m.tx_tsf = tsf();
    if (!drop_matches(f_.drop_d2g, MT_MARKER, ++send_idx_[0][MT_MARKER]))
      q_.push_back({now_ + kMsgLatency, true, m});
  }
  uint32_t d_epoch() const { return d_restarted_ ? 0x44440000u : 0x22220000u; }

  bool converged() const {
    return g_->state() == MigState::Stable && d_->state() == MigState::Stable &&
           g_->current_channel().same_rf(d_->current_channel());
  }
  Outcome finish(bool ok) {
    Outcome o;
    o.converged = ok && converged();
    o.channel = g_->current_channel();
    o.time_ms = now_;
    o.ground_confirmed = ground_confirmed_;
    o.drone_rolledback = drone_rolledback_;
    /* split-brain: both Stable but on different channels */
    o.split_brain = g_->state() == MigState::Stable &&
                    d_->state() == MigState::Stable &&
                    !g_->current_channel().same_rf(d_->current_channel());
    return o;
  }

  static constexpr uint32_t kLink = 0xABCD1234;
  static constexpr uint64_t kTsfBase = 1000000;
  ChannelDef source_, target_, rescue_;
  Fault f_;
  std::optional<MigProposer> g_;
  std::optional<MigResponder> d_;
  std::deque<Pending> q_;
  std::vector<LocalEv> local_;
  int send_idx_[2][8] = {};
  int64_t now_ = 0, next_marker_ms_ = 0, next_video_ms_ = 0;
  bool drone_markers_on_ = false, drone_pump_ = true;
  bool drone_retuned_once_ = false;
  bool g_restarted_ = false, d_restarted_ = false;
  bool ground_confirmed_ = false, drone_rolledback_ = false;
  bool progressed_ = false;
  int64_t settle_at_ = -1;
  bool stale_marker_sent_ = false;

public:
  bool trace_ = false;

private:
  MigState last_g_ = MigState::Stable, last_d_ = MigState::Stable;
};

static bool in_set(const ChannelDef &c, const ChannelDef &a, const ChannelDef &b,
                   const ChannelDef &d) {
  return c.same_rf(a) || c.same_rf(b) || c.same_rf(d);
}

int main() {
  const ChannelDef src = mkc(60), tgt = mkc(36), rsc = mkc(149);

  auto run = [&](Fault f, int64_t max_ms = 90000) {
    LinkSim s(src, tgt, rsc, f);
    s.trace_ = std::getenv("TRACE") != nullptr;
    return s.run(max_ms);
  };

  auto all = DropRule{0, -1, -1, 0, true};   /* drop-all of a type */
  auto drop = [](uint8_t type) { return DropRule{type, -1, -1, 0, true}; };

  /* Happy path: a clean migration converges on the target. */
  {
    Outcome o = run({});
    CHECK(o.converged && !o.split_brain, "happy: converged, no split");
    CHECK(o.channel.same_rf(tgt), "happy: landed on target");
    CHECK(o.ground_confirmed, "happy: ground confirmed");
  }

  /* Every row: converge to a shared channel in {src,tgt,rsc}, never split. */
  auto expect = [&](const char *name, Outcome o, const ChannelDef &want) {
    if (o.split_brain || !o.converged || !o.channel.same_rf(want))
      std::fprintf(stderr,
                   "  [%s] converged=%d split=%d ch=%s want=%s t=%lld\n", name,
                   o.converged, o.split_brain, o.channel.str().c_str(),
                   want.str().c_str(), (long long)o.time_ms);
    CHECK(!o.split_brain, name);
    CHECK(o.converged, name);
    CHECK(in_set(o.channel, src, tgt, rsc), name);
    CHECK(o.channel.same_rf(want), name);
  };

  { Fault f; f.drop_g2d = {drop(MT_PROPOSAL)};
    expect("F1 drop all proposals", run(f), src); }
  { Fault f; f.drop_d2g = {drop(MT_COMMIT)};
    expect("F2 drop all commits", run(f), src); }
  { Fault f; f.drop_d2g = {DropRule{MT_COMMIT, 3, 9999, 0, false}};
    expect("F3 drop late commit repeats", run(f), tgt); }
  { Fault f; f.drop_g2d = {drop(MT_STATUS)};
    expect("F4 drop ground ack", run(f), src); }
  { Fault f; f.drop_g2d = {drop(MT_CONFIRM)};
    expect("F5 drop confirm -> drone rolls back", run(f), src); }
  { Fault f; f.duplicate = true;
    expect("F6 duplicate + reorder every message", run(f), tgt); }
  { Fault f; f.g_restart_ms = 300; /* restart ground pre-activation */
    expect("F7a restart ground pre-activation", run(f), tgt); }
  { Fault f; f.g_restart_ms = 590; /* restart ground mid-verify: drone rolls back */
    Outcome o = run(f);
    if (o.split_brain || !o.converged)
      std::fprintf(stderr, "  [F7b] conv=%d split=%d ch=%s\n", o.converged,
                   o.split_brain, o.channel.str().c_str());
    CHECK(!o.split_brain && o.converged, "F7b restart ground mid-migration");
    CHECK(in_set(o.channel, src, tgt, rsc), "F7b in set"); }
  { Fault f; f.d_restart_ms = 590; /* drone restarts mid-migration */
    Outcome o = run(f);
    if (o.split_brain || !o.converged)
      std::fprintf(stderr, "  [F8] conv=%d split=%d ch=%s\n", o.converged,
                   o.split_brain, o.channel.str().c_str());
    CHECK(!o.split_brain && o.converged, "F8 restart drone");
    CHECK(in_set(o.channel, src, tgt, rsc), "F8 in set"); }
  { Fault f; f.target_silent = true;
    expect("F10 silent target -> rollback", run(f), src); }
  { Fault f; f.illegal_target = true;
    expect("F11 illegal target rejected", run(f), src); }
  { Fault f; f.drop_drain = true;
    expect("F12 drain stall -> drop-and-proceed", run(f), tgt); }
  { Fault f; f.drop_g2d = {all}; /* total uplink loss */
    expect("F13 control-path loss (all uplink)", run(f), src); }
  { Fault f; f.stale_marker_ms = 600; /* delayed wrong-channel frame */
    Outcome o = run(f);
    expect("F14 delayed old-channel marker", o, tgt); }
  { Fault f; f.drone_retune_fails = true; /* retune failure -> fallback */
    Outcome o = run(f);
    CHECK(!o.split_brain && o.converged, "F-retune-fail no split");
    CHECK(in_set(o.channel, src, tgt, rsc), "F-retune-fail in set"); }

  /* --- #280 variant-B pre-commit probe --- */
  { Fault f; f.drone_probe = true; f.probe_busy = 0.1; /* clean target */
    expect("probe accept -> migrates", run(f), tgt); }
  { Fault f; f.drone_probe = true; f.probe_busy = 0.9; /* busy target */
    Outcome o = run(f);
    CHECK(!o.split_brain && o.converged, "probe veto no split");
    CHECK(o.channel.same_rf(src), "probe veto stays on source"); }
  { Fault f; f.drone_probe = true; f.probe_busy = 0.1; f.drone_retune_fails = true;
    Outcome o = run(f); /* probe retune fails -> safe return, no migration */
    CHECK(!o.split_brain && o.converged, "probe retune-fail no split");
    CHECK(in_set(o.channel, src, tgt, rsc), "probe retune-fail in set"); }

  /* Exhaustive single-message drop sweep: for each message type, dropping
   * exactly its Nth instance (N=1..4) must still converge without split. */
  for (uint8_t type = MT_PROPOSAL; type <= MT_MARKER; ++type) {
    for (int n = 1; n <= 4; ++n) {
      Fault fg;
      fg.drop_g2d = {DropRule{type, n, n, 0, false}};
      Outcome og = run(fg);
      CHECK(!og.split_brain && og.converged, "drop-single g2d converges");
      CHECK(in_set(og.channel, src, tgt, rsc), "drop-single g2d in set");
      Fault fd;
      fd.drop_d2g = {DropRule{type, n, n, 0, false}};
      Outcome od = run(fd);
      CHECK(!od.split_brain && od.converged, "drop-single d2g converges");
      CHECK(in_set(od.channel, src, tgt, rsc), "drop-single d2g in set");
    }
  }

  (void)all;
  return fails ? 1 : 0;
}
