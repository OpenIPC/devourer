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
  AdaptiveScheduleView vt, vr;

  uint64_t slot = 0;
  int jammed = -1;         /* base index the interferer sits on, -1 = clean */
  bool drop_proposals = false, drop_commits = false;
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
        policy(pcfg, kBase), vt(base_tx, keys, kBase), vr(base_rx, keys, kBase) {
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
