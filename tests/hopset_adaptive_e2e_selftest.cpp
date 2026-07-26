/* The complete receiver-driven adaptive loop, end to end: two full stacks
 * (RX = policy + follower + schedule view, TX = authority + schedule view)
 * on a shared slot clock, coupled by the real wire codec and MAC, over a
 * per-channel delivery model. A jammed channel must be observed, proposed,
 * committed, activated on both sides at the same slot, then revisited by
 * keyed probes; when the jammer leaves, the probes must restore it. At no
 * point may the two sides hold different schedules, and the diversity floor
 * must hold under a herding jammer that moves after every exclusion. */
#include "hopset/HopsetAuthority.h"
#include "hopset/HopsetFollower.h"
#include "hopset/HopsetFusion.h"
#include "hopset/HopsetSense.h"
#include "hopset/HopsetPolicy.h"
#include "hopset/HopsetState.h"
#include "hopset/HopsetWire.h"

#include <cstdio>
#include <functional>
#include <set>
#include <vector>

static int fails;
#define CHECK(x, msg)                                                          \
  do {                                                                         \
    if (!(x)) {                                                                \
      std::fprintf(stderr, "FAIL: %s\n", msg);                                 \
      ++fails;                                                                 \
    }                                                                          \
  } while (0)

using namespace devourer::hopset;

namespace {

constexpr size_t kBase = 6;
constexpr unsigned kProbeRounds = 4;

/* One simulated link: both endpoints, real crypto between them. */
struct Sim {
  devourer::HopSchedule base_tx, base_rx;
  HopsetKeys keys;
  HopsetParams params;
  HopsetPolicyConfig pcfg;
  HopsetAuthority auth;
  HopsetFollower fol;
  HopsetPolicy policy;
  HopsetSensor sensor;
  FusionConfig fcfg;
  AdaptiveScheduleView vt, vr;

  uint64_t slot = 0;
  int jammed = -1;         /* base index the interferer sits on, -1 = clean */
  bool drop_proposals = false, drop_commits = false;
  bool tx_senses = false;      /* feed the transmitter's own sensor */
  bool tx_can_see = true;      /* false = the interferer is hidden from TX */
  uint64_t tx_originated = 0;  /* bits excluded on the transmitter's account */
  static constexpr size_t kNoBit = 64;
  size_t tx_pending_bit = kNoBit;
  int failsafe_activations = 0;
  int proposals_sent = 0, commits_sent = 0;
  int activations_tx = 0, activations_rx = 0;
  int probes_seen = 0, probes_on_jammed = 0;
  std::set<size_t> probed_channels;
  std::vector<uint64_t> rx_masks, tx_masks;

  Sim()
      : base_tx(devourer::HopSchedule::parse_seed(
            "0f1e2d3c4b5a69788796a5b4c3d2e1f0")),
        base_rx(devourer::HopSchedule::parse_seed(
            "0f1e2d3c4b5a69788796a5b4c3d2e1f0")),
        keys(HopsetKeys::derive(devourer::HopSchedule::parse_seed(
            "0f1e2d3c4b5a69788796a5b4c3d2e1f0"))),
        params(mk_params()), pcfg(mk_policy()),
        auth(params, 0xA11CE), fol(params, 0xB0B),
        policy(pcfg, kBase), sensor(TxSenseConfig{}, kBase),
        vt(base_tx, keys, kBase), vr(base_rx, keys, kBase) {
    vt.set_state(auth.state());
    vr.set_state(fol.state());
    vt.set_probe_period(kProbeRounds);
    vr.set_probe_period(kProbeRounds);
  }

  static HopsetParams mk_params() {
    HopsetParams p;
    p.n_base = kBase;
    p.base_fp = 0xFACEB00C;
    p.link_id = 0x2264;
    p.min_active = 3;
    p.commit_repeat_slots = 4;
    p.status_interval_slots = 48;
    p.proposal_retries = 6;
    p.proposal_backoff_slots = 12;
    return p;
  }
  static HopsetPolicyConfig mk_policy() {
    HopsetPolicyConfig c;
    c.min_obs_rounds = 8;
    c.update_cooldown_rounds = 10;
    return c;
  }

  /* Deliver an authority action to the follower (or a follower action to the
   * authority) through encode -> MAC -> decode. */
  void route(const std::vector<HopsetAction> &acts, bool from_auth) {
    for (const auto &a : acts) {
      if (a.kind == HopsetAction::Activate) {
        if (from_auth) {
          if (a.state.active_mask != vt.state().active_mask) {
            ++activations_tx;
            tx_masks.push_back(a.state.active_mask);
          }
          vt.set_state(a.state);
          sensor.on_activation(a.state.active_mask, auth.round_of(slot));
          /* remember which bits the transmitter took on its own account, so
           * the fusion layer can hand them back once the receiver is heard
           * from again */
          if (tx_pending_bit != kNoBit) {
            const uint64_t b = uint64_t(1) << tx_pending_bit;
            if (a.state.active_mask & b)
              tx_originated &= ~b; /* it was a restore */
            else
              tx_originated |= b;
            tx_pending_bit = kNoBit;
          }
        } else {
          if (a.state.active_mask != vr.state().active_mask) {
            ++activations_rx;
            rx_masks.push_back(a.state.active_mask);
          }
          vr.set_state(a.state);
          policy.on_activation(a.state.active_mask, round_rx());
        }
        continue;
      }
      /* a rejected or timed-out proposal releases the policy's latch — the
       * host owns that handshake, the pure machine only reports it */
      if (a.kind == HopsetAction::Event) {
        if (a.event == HopsetEvent::Reject && !from_auth)
          policy.clear_outstanding(round_rx());
        continue;
      }
      if (a.kind != HopsetAction::SendControl)
        continue;
      if (from_auth) {
        ++commits_sent;
        if (drop_commits && a.msg.type == HT_COMMIT)
          continue;
      } else {
        ++proposals_sent;
        if (drop_proposals)
          continue;
      }
      const auto w = hopset_encode(a.msg, keys);
      HopsetMsg r;
      if (hopset_decode(w.data(), w.size(), keys, params.link_id, r) !=
          HopsetReason::None)
        continue;
      if (from_auth) {
        if (r.type == HT_COMMIT)
          route(fol.on_commit(r, slot), false);
        else if (r.type == HT_STATUS) {
          route(fol.on_status(r, slot), false);
          if (r.reason != 0)
            policy.clear_outstanding(round_rx());
        }
      } else if (r.type == HT_PROPOSAL) {
        route(auth.on_proposal(r, slot), true);
      } else if (r.type == HT_STATUS && r.role == 0) {
        auth.note_feedback(slot); /* the receiver's heartbeat */
      }
    }
  }

  uint64_t round_rx() const {
    const auto &st = vr.state();
    const size_t k = st.generation == 0 ? kBase : popcount64(st.active_mask);
    return (slot - st.activate_slot) / k;
  }

  /* Advance one slot: both sides retune from their own committed state, a
   * frame is delivered iff they agree on the channel and it is not jammed,
   * the receiver scores the closed dwell, and at each round boundary the
   * policy gets to decide. */
  void tick() {
    const auto ti = vt.slot_info(slot);
    const auto ri = vr.slot_info(slot);
    const bool agree = ti.base_index == ri.base_index;
    const bool clear = static_cast<int>(ri.base_index) != jammed;
    const bool delivered = agree && clear;

    SlotObservation o;
    o.slot = slot;
    o.round = round_rx();
    o.base_index = static_cast<uint32_t>(ri.base_index);
    o.is_probe = ri.is_probe;
    o.decoded = delivered;
    o.frames = delivered ? 8 : 0;
    policy.ingest(o);
    if (ri.is_probe) {
      ++probes_seen;
      probed_channels.insert(ri.base_index);
      if (static_cast<int>(ri.base_index) == jammed)
        ++probes_on_jammed;
    }

    if (tx_senses) {
      /* the transmitter senses the channel it is dwelling on */
      TxSenseSample x;
      x.slot = slot;
      x.round = round_rx();
      x.base_index = static_cast<uint32_t>(ti.base_index);
      x.phase = SensePhase::PostBurst;
      x.window_us = 4000;
      x.probe = ti.is_probe;
      x.valid_fa = true;
      const bool hot =
          tx_can_see && static_cast<int>(ti.base_index) == jammed;
      x.cca_ofdm = hot ? 340 : 8;
      x.fa_ofdm = hot ? 170 : 4;
      sensor.ingest(x);
    }

    const uint64_t r_before = round_rx();
    route(auth.on_tick(slot), true);
    route(fol.on_tick(slot), false);
    ++slot;
    if (round_rx() != r_before) { /* a round just closed */
      auto d = policy.decide(round_rx());
      if (d.kind != HopsetDecision::Kind::Hold)
        route(fol.propose(d.proposed_mask, d.observation_count,
                          d.reason_bitmap,
                          0xC0DE0000u + static_cast<uint32_t>(slot), slot),
              false);
      if (tx_senses) {
        const uint64_t ar = auth.round_of(slot);
        auto cand = sensor.evaluate(ar);
        FusionInput in;
        in.tx = &cand;
        in.active_mask = auth.state().generation ? auth.state().active_mask
                                                 : full_mask(kBase);
        in.feedback_age_rounds = auth.feedback_age_rounds(slot);
        in.now_round = ar;
        in.tx_originated_mask = tx_originated;
        auto f = fuse(fcfg, in);
        if (f.action == FusedAction::Exclude ||
            f.action == FusedAction::Restore) {
          const int before = activations_tx;
          tx_pending_bit = f.target_index;
          route(auth.start_local_change(f.mask, slot, f.reason_bitmap), true);
          if (activations_tx == before && !auth.committing())
            sensor.clear_outstanding(ar); /* the authority refused it */
          if (f.origin == DecisionOrigin::Failsafe)
            ++failsafe_activations;
        } else if (cand.kind != TxSenseCandidate::Kind::None) {
          sensor.clear_outstanding(ar); /* fusion declined to act on it */
        }
      }
    }
  }

  void run(uint64_t slots) {
    for (uint64_t i = 0; i < slots; ++i)
      tick();
  }

  bool converged() const {
    return vt.state().generation == vr.state().generation &&
           vt.state().active_mask == vr.state().active_mask &&
           vt.state().activate_slot == vr.state().activate_slot;
  }
};

} // namespace

int main() {
  /* --- the full arc: jam -> exclude -> probe -> unjam -> restore --- */
  {
    Sim s;
    s.jammed = 3;
    s.run(1200);
    CHECK(s.vr.state().generation >= 1, "an exclusion was committed");
    CHECK((s.vr.state().active_mask & (uint64_t(1) << 3)) == 0,
          "the jammed channel was excluded");
    CHECK(s.converged(), "both sides hold the same schedule");
    CHECK(s.activations_tx == s.activations_rx,
          "activation counts match (no split-brain)");
    CHECK(s.tx_masks == s.rx_masks, "identical committed mask history");
    CHECK(s.probes_seen > 0 && s.probes_on_jammed > 0,
          "keyed probes revisit the excluded channel");
    CHECK(s.probed_channels.size() == 1 && *s.probed_channels.begin() == 3,
          "probes only visit excluded channels");
    const uint64_t excluded_at = s.slot;

    /* the interferer leaves; probe hysteresis must bring the channel back */
    s.jammed = -1;
    s.run(1600);
    CHECK(s.vr.state().active_mask == full_mask(6),
          "the recovered channel was restored");
    CHECK(s.vr.state().generation >= 2, "restoration is its own generation");
    CHECK(s.converged() && s.tx_masks == s.rx_masks,
          "still converged after restoration");
    CHECK(s.slot > excluded_at, "restoration followed exclusion");
  }

  /* --- schedules stay identical slot-by-slot across every transition --- */
  {
    Sim s;
    s.jammed = 1;
    for (int i = 0; i < 1500; ++i) {
      const auto a = s.vt.slot_info(s.slot), b = s.vr.slot_info(s.slot);
      CHECK(a.base_index == b.base_index && a.is_probe == b.is_probe,
            "per-slot schedules identical");
      s.tick();
    }
    CHECK(s.vr.state().generation >= 1, "transition happened during the run");
  }

  /* --- a clean link never excludes anything --- */
  {
    Sim s;
    s.run(1500);
    CHECK(s.vr.state().active_mask == full_mask(6) && s.activations_rx == 0,
          "no exclusion without impairment");
    CHECK(s.probes_seen == 0, "no probes while the mask is full");
  }

  /* --- broad degradation: everything jammed excludes nothing --- */
  {
    Sim s;
    /* jam by making every channel disagree: an impossible channel index */
    s.jammed = -2; /* matches nothing... so instead drop every frame: */
    for (int i = 0; i < 1200; ++i) {
      SlotObservation o;
      o.slot = s.slot;
      o.round = s.round_rx();
      o.base_index = static_cast<uint32_t>(s.vr.slot_info(s.slot).base_index);
      o.decoded = false;
      s.policy.ingest(o);
      auto d = s.policy.decide(s.round_rx());
      CHECK(d.kind == HopsetDecision::Kind::Hold,
            "broad degradation never excludes");
      ++s.slot;
    }
  }

  /* --- feedback loss: dropped proposals and commits still converge --- */
  {
    Sim s;
    s.jammed = 3;
    s.drop_proposals = true;
    s.run(400);
    CHECK(s.proposals_sent > 0 && s.vr.state().generation == 0,
          "proposals were sent and lost");
    s.drop_proposals = false;
    s.drop_commits = true;
    s.run(400);
    s.drop_commits = false;
    s.run(1200);
    CHECK(s.vr.state().generation >= 1 && s.converged(),
          "the loop converges after proposal and commit loss");
    CHECK((s.vr.state().active_mask & (uint64_t(1) << 3)) == 0,
          "the right channel was still excluded");
    CHECK(s.tx_masks == s.rx_masks, "no split-brain through the losses");
  }

  /* --- a genuine one-way outage: the receiver can still HEAR the
   * transmitter, it just cannot answer. Nothing adapts under the receiver's
   * authority because its proposals never arrive, so the transmitter must
   * eventually act on its own evidence — and the receiver must follow the
   * resulting commits without ever holding a different generation. --- */
  {
    Sim s;
    s.jammed = 3;
    s.tx_senses = true;
    s.fcfg.mode = FusionMode::TxFailsafe;
    s.drop_proposals = true; /* the uplink is down; the downlink is fine */
    s.run(4000);
    CHECK(s.failsafe_activations > 0,
          "the transmitter originated once the uplink went quiet");
    CHECK(s.vt.state().generation >= 1, "an autonomous exclusion committed");
    CHECK((s.vt.state().active_mask & (uint64_t(1) << 3)) == 0,
          "it excluded the channel its own sensing condemned");
    CHECK(s.converged(), "the receiver followed, holding the same schedule");
    CHECK(s.tx_masks == s.rx_masks, "no split-brain through the outage");
    /* every committed change moved exactly one channel: the autonomous path
     * is bound by the same structural limits a proposal is */
    for (size_t i = 1; i < s.tx_masks.size(); ++i)
      CHECK(popcount64(s.tx_masks[i] ^ s.tx_masks[i - 1]) == 1,
            "one channel per autonomous update");
    CHECK(popcount64(s.vt.state().active_mask) >= 3,
          "the diversity floor held");
  }

  /* --- with the uplink alive the transmitter defers, even in failsafe mode,
   * and even though its own sensing can see the interferer perfectly well --- */
  {
    Sim s;
    s.jammed = 3;
    s.tx_senses = true;
    s.fcfg.mode = FusionMode::TxFailsafe;
    s.run(4000);
    CHECK(s.failsafe_activations == 0,
          "a live receiver keeps the transmitter from originating");
    CHECK(s.vr.state().generation >= 1 &&
              (s.vr.state().active_mask & (uint64_t(1) << 3)) == 0,
          "the receiver did the deciding");
    CHECK(s.converged() && s.tx_masks == s.rx_masks, "still one schedule");
  }

  /* --- the default mode must not disturb the receiver-driven result, even
   * when the transmitter cannot see the interferer at all (the hidden node,
   * which is the shape of the parked-jammer bench test) --- */
  {
    Sim s;
    s.jammed = 3;
    s.tx_senses = true;
    s.tx_can_see = false; /* the interferer is invisible at the transmitter */
    s.fcfg.mode = FusionMode::RxPlusTxVeto;
    s.run(2000);
    CHECK(s.vr.state().generation >= 1 &&
              (s.vr.state().active_mask & (uint64_t(1) << 3)) == 0,
          "the receiver's exclusion went through unimpeded");
    CHECK(s.failsafe_activations == 0, "the transmitter originated nothing");
    CHECK(s.converged() && s.tx_masks == s.rx_masks, "one schedule");
  }

  /* --- herding: the jammer follows every exclusion; the floor must hold --- */
  {
    Sim s;
    s.jammed = 0;
    for (int step = 0; step < 40; ++step) {
      s.run(200);
      /* move onto the lowest still-active channel */
      const uint64_t m = s.vr.state().generation == 0 ? full_mask(6)
                                                      : s.vr.state().active_mask;
      for (size_t i = 0; i < 6; ++i)
        if (m & (uint64_t(1) << i)) {
          s.jammed = static_cast<int>(i);
          break;
        }
      const size_t active =
          s.vr.state().generation == 0 ? 6 : popcount64(s.vr.state().active_mask);
      CHECK(active >= 3, "herding never breaches the diversity floor");
      CHECK(s.converged(), "herding never splits the schedule");
    }
    CHECK(popcount64(s.vr.state().active_mask) >= 3,
          "final active set at or above the floor");
  }

  return fails ? 1 : 0;
}
