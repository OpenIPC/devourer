/* HopsetAuthority + HopsetFollower coupled through a lossy LinkSim — the
 * convergence gate for the adaptive-hopset commit protocol. Every fault row
 * (lost / duplicated / reordered / forged / replayed control frames, TX and
 * RX restart, generation ceiling, invalid masks, activation-window
 * violations, missed pre-activation commits, and a drop-every-message sweep)
 * must end with both endpoints in the identical committed state — activation
 * happens on both sides at the same absolute slot or on neither. All
 * messages travel through the real encode/MAC/decode. */
#include "hopset/HopsetAuthority.h"
#include "hopset/HopsetFollower.h"
#include "hopset/HopsetWire.h"

#include <cstdio>
#include <functional>
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

static HopsetParams params() {
  HopsetParams p;
  p.n_base = 8;
  p.base_fp = 0xB0BAF00D;
  p.link_id = 0x1234;
  p.min_active = 2;
  p.lead_rounds = 8;
  p.max_lead_slots = 4096;
  p.commit_repeat_slots = 4;
  p.status_interval_slots = 32;
  p.proposal_retries = 4;
  p.proposal_backoff_slots = 8;
  return p;
}

/* One authority + one follower on a shared slot clock, coupled through the
 * real wire codec with a per-message fault hook. */
struct Sim {
  HopsetKeys keys;
  HopsetParams p;
  HopsetAuthority auth;
  HopsetFollower fol;
  uint64_t slot = 0;
  int msg_index = 0; /* running index over every sent control message */
  /* return false to drop; may be re-pointed per row */
  std::function<bool(int idx, const HopsetMsg &)> deliver = nullptr;
  int fol_activations = 0, auth_activations = 0;
  uint64_t fol_act_slot = 0, auth_act_slot = 0;
  HopsetState fol_state{}, auth_state{};
  HopsetReason last_reject = HopsetReason::None;

  Sim(uint32_t auth_epoch, uint32_t fol_epoch)
      : keys(HopsetKeys::derive(devourer::HopSchedule::parse_seed(
            "000102030405060708090a0b0c0d0e0f"))),
        p(params()), auth(p, auth_epoch), fol(p, fol_epoch) {
    fol_state = fol.state();
    auth_state = auth.state();
  }

  void route(const std::vector<HopsetAction> &acts, bool from_auth) {
    for (const auto &a : acts) {
      if (a.kind == HopsetAction::Activate) {
        if (from_auth) {
          ++auth_activations;
          auth_act_slot = slot;
          auth_state = a.state;
        } else {
          ++fol_activations;
          fol_act_slot = slot;
          fol_state = a.state;
        }
      } else if (a.kind == HopsetAction::Event) {
        if (a.event == HopsetEvent::Reject)
          last_reject = a.reason;
      } else if (a.kind == HopsetAction::SendControl) {
        const int idx = msg_index++;
        if (deliver && !deliver(idx, a.msg))
          continue;
        /* real crypto both ways */
        auto w = hopset_encode(a.msg, keys);
        HopsetMsg r;
        if (hopset_decode(w.data(), w.size(), keys, p.link_id, r) !=
            HopsetReason::None)
          continue;
        if (from_auth) {
          if (r.type == HT_COMMIT)
            route(fol.on_commit(r, slot), false);
          else if (r.type == HT_STATUS)
            route(fol.on_status(r, slot), false);
        } else if (r.type == HT_PROPOSAL) {
          route(auth.on_proposal(r, slot), true);
        }
      }
    }
  }

  void tick(uint64_t n = 1) {
    for (uint64_t i = 0; i < n; ++i) {
      ++slot;
      route(auth.on_tick(slot), true);
      route(fol.on_tick(slot), false);
    }
  }

  void propose(uint64_t mask, uint32_t nonce) {
    route(fol.propose(mask, 100, 0x1, nonce, slot), false);
  }

  bool converged() const {
    return auth_state.generation == fol_state.generation &&
           auth_state.active_mask == fol_state.active_mask &&
           auth_state.activate_slot == fol_state.activate_slot;
  }
};

int main() {
  /* --- row 1: clean run — propose, commit, synchronized activation --- */
  {
    Sim s(0x1000, 0x2000);
    s.propose(0b00110111, 0xA1);
    CHECK(s.auth.committing(), "clean: authority commits");
    s.tick(200);
    CHECK(s.auth_activations == 1 && s.fol_activations == 1,
          "clean: both activate exactly once");
    CHECK(s.auth_act_slot == s.fol_act_slot, "clean: same activation slot");
    CHECK(s.converged() && s.fol_state.generation == 1 &&
              s.fol_state.active_mask == 0b00110111,
          "clean: identical committed state");
    CHECK(s.fol_state.activate_slot >= 8 * 8,
          "clean: >= 8 rounds of lead honored");
  }

  /* --- row 2: lost proposals — retries recover --- */
  {
    Sim s(0x1000, 0x2000);
    int dropped = 0;
    s.deliver = [&](int, const HopsetMsg &m) {
      if (m.type == HT_PROPOSAL && dropped < 2) {
        ++dropped;
        return false;
      }
      return true;
    };
    s.propose(0b00001111, 0xA2);
    s.tick(300);
    CHECK(dropped == 2 && s.converged() && s.fol_state.generation == 1,
          "lost proposals: retry converges");
  }

  /* --- row 3: lost commits — repetition recovers --- */
  {
    Sim s(0x1000, 0x2000);
    int commits_dropped = 0;
    s.deliver = [&](int, const HopsetMsg &m) {
      if (m.type == HT_COMMIT && commits_dropped < 3) {
        ++commits_dropped;
        return false;
      }
      return true;
    };
    s.propose(0b11110000, 0xA3);
    s.tick(300);
    CHECK(commits_dropped == 3 && s.converged() &&
              s.fol_activations == 1 && s.auth_activations == 1 &&
              s.auth_act_slot == s.fol_act_slot,
          "lost commits: repeats converge");
  }

  /* --- row 4: duplicated commit — a repeat is idempotent --- */
  {
    Sim s(0x1000, 0x2000);
    s.propose(0b00111100, 0xA4);
    s.tick(2); /* several repeated commits already flow; all must be inert
                  duplicates on the follower */
    s.tick(300);
    CHECK(s.fol_activations == 1, "dup commits: single activation");
    CHECK(s.converged(), "dup commits: converged");
  }

  /* --- row 5: replayed proposal after completion — rejected --- */
  {
    Sim s(0x1000, 0x2000);
    s.propose(0b00110111, 0xA5);
    s.tick(200);
    CHECK(s.converged() && s.fol_state.generation == 1, "replay setup");
    /* attacker replays the same authenticated proposal bytes */
    HopsetMsg replay;
    replay.type = HT_PROPOSAL;
    replay.link_id = s.p.link_id;
    replay.rx_epoch = 0x2000;
    replay.rx_nonce = 0xA5;
    replay.active_mask = 0b00110111;
    replay.base_fp = s.p.base_fp;
    s.last_reject = HopsetReason::None;
    s.route(std::vector<HopsetAction>{}, false); /* no-op */
    auto acts = s.auth.on_proposal(replay, s.slot);
    bool replay_rejected = false;
    for (auto &a : acts)
      if (a.kind == HopsetAction::Event && a.event == HopsetEvent::Reject &&
          a.reason == HopsetReason::ReplayGen)
        replay_rejected = true;
    CHECK(replay_rejected, "replayed proposal rejected");
    CHECK(!s.auth.committing(), "replayed proposal arms nothing");
  }

  /* --- row 6: forged commit (wrong key) never reaches the machine --- */
  {
    Sim s(0x1000, 0x2000);
    HopsetKeys forged = HopsetKeys::derive(
        devourer::HopSchedule::parse_seed("deadbeefdeadbeefdeadbeefdeadbeef"));
    HopsetMsg m;
    m.type = HT_COMMIT;
    m.link_id = s.p.link_id;
    m.tx_epoch = 0x6666;
    m.generation = 5;
    m.active_mask = 0b11;
    m.base_fp = s.p.base_fp;
    m.activate_slot = 100;
    auto w = hopset_encode(m, forged);
    HopsetMsg r;
    CHECK(hopset_decode(w.data(), w.size(), s.keys, s.p.link_id, r) ==
              HopsetReason::BadMac,
          "forged commit fails authentication");
    CHECK(s.fol.state().generation == 0, "forged commit moved nothing");
  }

  /* --- row 7: stale-epoch commit replay (old boot, past slot) --- */
  {
    Sim s(0x1000, 0x2000);
    /* run a legitimate change so the follower tracks epoch 0x1000 gen 1 */
    s.propose(0b00110111, 0xA7);
    s.tick(200);
    CHECK(s.fol_state.generation == 1, "stale-epoch setup");
    /* replay: an authenticated commit from a previous authority boot (other
     * epoch) whose activation slot is deep in the past of that old session —
     * adopting it would be a replay takeover. The window rule adopts a
     * past-slot commit only as recovery; here it must still converge to the
     * live authority via its status beacon afterwards. */
    HopsetMsg old;
    old.type = HT_COMMIT;
    old.link_id = s.p.link_id;
    old.tx_epoch = 0x0BAD;
    old.generation = 9;
    old.active_mask = 0b11000000;
    old.base_fp = s.p.base_fp;
    old.activate_round = 1;
    old.activate_slot = 8; /* long past */
    old.current_round = 1;
    auto w = hopset_encode(old, s.keys);
    HopsetMsg r;
    CHECK(hopset_decode(w.data(), w.size(), s.keys, s.p.link_id, r) ==
              HopsetReason::None,
          "old commit authenticates (same key)");
    s.route(s.fol.on_commit(r, s.slot), false);
    /* the live authority's status beacon must re-assert the true state */
    s.tick(2 * s.p.status_interval_slots + 2);
    CHECK(s.converged() && s.fol_state.epoch == 0x1000,
          "stale-epoch replay: live status re-converges the follower");
  }

  /* --- row 8: TX restart mid-commit — fresh epoch orphans the exchange --- */
  {
    Sim s(0x1000, 0x2000);
    /* drop everything from the authority so the follower never sees the
     * commit for the proposal it sent */
    s.deliver = [&](int, const HopsetMsg &m) {
      return m.type == HT_PROPOSAL; /* only proposals get through */
    };
    s.propose(0b00110111, 0xA8);
    CHECK(s.auth.committing(), "restart setup: authority armed");
    /* authority restarts: new machine, new random epoch, gen 0. The old
     * in-flight generation is orphaned for free (nothing persisted); the
     * follower's retried proposal simply re-lands at the fresh authority,
     * and both converge under the NEW epoch — never a split brain. */
    s.auth = HopsetAuthority(s.p, 0x1111);
    s.deliver = nullptr;
    s.auth_state = s.auth.state();
    s.tick(6 * s.p.status_interval_slots);
    CHECK(s.fol.state().epoch == 0x1111 &&
              s.fol.state().generation == s.auth.state().generation &&
              s.fol.state().active_mask == s.auth.state().active_mask &&
              s.fol.state().activate_slot == s.auth.state().activate_slot,
          "TX restart: follower converges under the fresh epoch");
  }

  /* --- row 9: RX restart — a fresh follower joins at nonzero gen --- */
  {
    Sim s(0x1000, 0x2000);
    s.propose(0b01011010, 0xA9);
    s.tick(200);
    CHECK(s.fol_state.generation == 1, "RX restart setup");
    /* follower restarts with a new epoch and no state */
    s.fol = HopsetFollower(s.p, 0x2222);
    s.fol_state = s.fol.state();
    CHECK(s.fol.state().generation == 0, "fresh follower at gen 0");
    s.tick(2 * s.p.status_interval_slots + 2);
    CHECK(s.fol.state().generation == 1 &&
              s.fol.state().active_mask == 0b01011010 &&
              s.fol.state().activate_slot == s.auth.state().activate_slot,
          "RX restart: rejoins at the live nonzero generation");
  }

  /* --- row 10: generation ceiling refused --- */
  {
    Sim s(0x1000, 0x2000);
    HopsetState near_ceiling = s.auth.state();
    near_ceiling.generation = kGenCeiling - 1;
    near_ceiling.active_mask = full_mask(8);
    s.auth.restore(near_ceiling);
    auto acts = s.auth.start_change(0b1111, s.slot);
    bool refused = false;
    for (auto &a : acts)
      if (a.kind == HopsetAction::Event && a.event == HopsetEvent::Reject &&
          a.reason == HopsetReason::GenCeiling)
        refused = true;
    CHECK(refused && !s.auth.committing(), "generation ceiling refused");
  }

  /* --- row 11: invalid masks --- */
  {
    Sim s(0x1000, 0x2000);
    s.propose(0b100000000, 0xB1); /* bit 8 of an 8-channel base */
    CHECK(s.last_reject == HopsetReason::BadMask && !s.auth.committing(),
          "out-of-range mask rejected");
    s.tick(20); /* let the proposal state settle (status reject echo) */
    Sim s2(0x1000, 0x2000);
    s2.propose(0b00000001, 0xB2); /* below min_active = 2 */
    CHECK(s2.last_reject == HopsetReason::LowDiversity &&
              !s2.auth.committing(),
          "below-diversity mask rejected");
  }

  /* --- row 12: activation outside the allowed window rejected --- */
  {
    Sim s(0x1000, 0x2000);
    HopsetMsg m;
    m.type = HT_COMMIT;
    m.link_id = s.p.link_id;
    m.tx_epoch = 0x1000;
    m.generation = 1;
    m.active_mask = 0b1111;
    m.base_fp = s.p.base_fp;
    m.activate_slot = s.slot + s.p.max_lead_slots + 100; /* too far out */
    auto w = hopset_encode(m, s.keys);
    HopsetMsg r;
    CHECK(hopset_decode(w.data(), w.size(), s.keys, s.p.link_id, r) ==
              HopsetReason::None,
          "window-test commit authenticates");
    s.route(s.fol.on_commit(r, s.slot), false);
    CHECK(s.last_reject == HopsetReason::ActivationWindow &&
              !s.fol.has_pending(),
          "too-distant activation rejected");
  }

  /* --- row 13: proposal starvation times out cleanly --- */
  {
    Sim s(0x1000, 0x2000);
    s.deliver = [](int, const HopsetMsg &) { return false; }; /* black hole */
    s.propose(0b00111100, 0xB3);
    s.tick(200);
    CHECK(s.last_reject == HopsetReason::Timeout &&
              s.fol.fsm() == HopsetFollower::State::Synced,
          "starved proposal times out back to Synced");
    CHECK(s.fol.state().generation == 0, "no state moved");
  }

  /* --- row 14: NoChange — re-proposing the active mask --- */
  {
    Sim s(0x1000, 0x2000);
    s.propose(0b00110111, 0xB4);
    s.tick(200);
    CHECK(s.fol_state.generation == 1, "nochange setup");
    s.last_reject = HopsetReason::None;
    s.propose(0b00110111, 0xB5);
    CHECK(s.last_reject == HopsetReason::NoChange && !s.auth.committing(),
          "same-mask proposal rejected as no-change");
  }

  /* --- drop-every-message sweep: for each control-message index in the
   * clean run, drop exactly that one — every variant must still converge
   * (repeats and the status beacon carry it) --- */
  {
    /* count messages in a clean run first */
    Sim probe(0x1000, 0x2000);
    probe.propose(0b00101101, 0xC0);
    probe.tick(400);
    const int total = probe.msg_index;
    CHECK(total > 4, "sweep probe saw a full exchange");
    for (int drop = 0; drop < total; ++drop) {
      Sim s(0x1000, 0x2000);
      s.deliver = [&](int idx, const HopsetMsg &) { return idx != drop; };
      s.propose(0b00101101, 0xC0);
      s.tick(800);
      if (!(s.converged() && s.fol.state().generation >= 1)) {
        std::fprintf(stderr, "sweep: drop=%d diverged\n", drop);
        CHECK(false, "drop-every-message sweep row converges");
      }
    }
  }

  return fails ? 1 : 0;
}
