/* Process-level adaptive-hopset activation gate: two full stacks (schedule
 * view + machine + real wire crypto + v2 sync markers) on a shared slot
 * clock. (a) A committed mask change activates on BOTH sides at the same
 * absolute slot and the per-slot channel sequences are identical across the
 * boundary. (b) A follower that misses every commit detects the transition
 * from the marker's generation/mask fingerprint, recovers via the authority's
 * status beacon, and re-synchronizes — with a bounded desync window. */
#include "hopset/HopsetAuthority.h"
#include "hopset/HopsetFollower.h"
#include "hopset/HopsetState.h"
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

int main() {
  const auto master =
      devourer::HopSchedule::parse_seed("00112233445566778899aabbccddeeff");
  const HopsetKeys keys = HopsetKeys::derive(master);
  devourer::HopSchedule base_tx(master), base_rx(master);

  HopsetParams p;
  p.n_base = 10;
  p.base_fp = 0xFEEDF00D;
  p.link_id = 0x77;
  p.status_interval_slots = 40;
  p.commit_repeat_slots = 4;

  /* ---------------- (a) synchronized future-round activation ------------- */
  {
    HopsetAuthority auth(p, 0xAAAA);
    HopsetFollower fol(p, 0xBBBB);
    AdaptiveScheduleView vt(base_tx, keys, p.n_base);
    AdaptiveScheduleView vr(base_rx, keys, p.n_base);

    uint64_t slot = 0;
    uint64_t tx_act = 0, rx_act = 0;
    std::function<void(const std::vector<HopsetAction> &, bool)> pump =
        [&](const std::vector<HopsetAction> &acts, bool from_auth) {
      for (const auto &a : acts) {
        if (a.kind == HopsetAction::Activate) {
          if (from_auth) {
            vt.set_state(a.state);
            tx_act = slot;
          } else {
            vr.set_state(a.state);
            rx_act = slot;
          }
        } else if (a.kind == HopsetAction::SendControl && from_auth) {
          auto w = hopset_encode(a.msg, keys);
          HopsetMsg r;
          if (hopset_decode(w.data(), w.size(), keys, p.link_id, r) !=
              HopsetReason::None)
            continue;
          if (r.type == HT_COMMIT)
            pump(fol.on_commit(r, slot), false);
          else if (r.type == HT_STATUS)
            pump(fol.on_status(r, slot), false);
        }
      }
    };

    /* a scripted authority-originated change (mask drops 4 of 10 channels) */
    pump(auth.start_change(0b0110110110, slot), true);
    CHECK(auth.committing(), "authority armed");

    std::vector<size_t> tx_seq, rx_seq;
    for (int i = 0; i < 400; ++i) {
      ++slot;
      pump(auth.on_tick(slot), true);
      pump(fol.on_tick(slot), false);
      tx_seq.push_back(vt.channel_index(slot));
      rx_seq.push_back(vr.channel_index(slot));
    }
    CHECK(tx_act != 0 && tx_act == rx_act,
          "both sides activated at the same slot");
    CHECK(tx_seq == rx_seq, "identical channel sequence across the boundary");
    CHECK(vt.state().generation == 1 &&
              vt.state().active_mask == 0b0110110110,
          "generation 1 active");
    /* every post-activation channel is in the active set */
    for (uint64_t s = vt.state().activate_slot; s < slot; ++s)
      CHECK(vt.state().active_mask & (uint64_t(1) << vt.channel_index(s)),
            "post-activation slots stay inside the mask");
  }

  /* -------- (b) missed-commit recovery via marker + status beacon -------- */
  {
    HopsetAuthority auth(p, 0xCCCC);
    HopsetFollower fol(p, 0xDDDD);
    AdaptiveScheduleView vt(base_tx, keys, p.n_base);
    AdaptiveScheduleView vr(base_rx, keys, p.n_base);

    uint64_t slot = 0;
    bool control_blackout = true; /* the follower misses every commit */
    uint64_t recovered_slot = 0;
    int gen_mismatch = 0, recover_ev = 0;

    auto to_follower = [&](const std::vector<HopsetAction> &acts,
                           auto &&self) -> void {
      for (const auto &a : acts) {
        if (a.kind == HopsetAction::Activate) {
          vr.set_state(a.state);
          if (a.state.generation > 0)
            recovered_slot = slot;
        } else if (a.kind == HopsetAction::Event) {
          if (a.event == HopsetEvent::GenMismatch)
            ++gen_mismatch;
          if (a.event == HopsetEvent::Recover)
            ++recover_ev;
        }
      }
      (void)self;
    };
    auto from_auth = [&](const std::vector<HopsetAction> &acts) {
      for (const auto &a : acts) {
        if (a.kind == HopsetAction::Activate)
          vt.set_state(a.state);
        else if (a.kind == HopsetAction::SendControl) {
          if (control_blackout)
            continue;
          auto w = hopset_encode(a.msg, keys);
          HopsetMsg r;
          if (hopset_decode(w.data(), w.size(), keys, p.link_id, r) !=
              HopsetReason::None)
            continue;
          if (r.type == HT_COMMIT)
            to_follower(fol.on_commit(r, slot), 0);
          else if (r.type == HT_STATUS)
            to_follower(fol.on_status(r, slot), 0);
        }
      }
    };

    from_auth(auth.start_change(0b1111000011, slot));
    for (int i = 0; i < 200; ++i) {
      ++slot;
      from_auth(auth.on_tick(slot));
      to_follower(fol.on_tick(slot), 0);
      /* the TX hot path: every slot carries a v2 marker with the LIVE
       * (generation, mask fingerprint) — encode/decode it for real */
      HopSyncMarkerV2 mk;
      mk.fingerprint = base_tx.fingerprint();
      mk.slot = slot;
      mk.generation = vt.state().generation;
      mk.mask_fp = mask_fp(keys, vt.state().generation,
                           vt.state().active_mask);
      auto mw = HopSyncMarkerV2::encode(mk);
      HopSyncMarkerV2 rm;
      CHECK(HopSyncMarkerV2::decode(mw.data(), mw.size(), rm),
            "marker decodes");
      const bool matches =
          (rm.generation == vr.state().generation &&
           rm.mask_fp ==
               mask_fp(keys, vr.state().generation, vr.state().active_mask)) ||
          (fol.has_pending() && rm.generation == fol.pending().generation &&
           rm.mask_fp == mask_fp(keys, fol.pending().generation,
                                 fol.pending().active_mask));
      to_follower(fol.on_marker(rm.generation, matches, slot), 0);
      /* once the follower notices the mismatch, the "reverse path heals":
       * recovery means the control plane is reachable again (the follower
       * fell back to the base-hopset acquire scan and now hears the
       * authority's repeated status beacon) */
      if (fol.fsm() == HopsetFollower::State::Recovering)
        control_blackout = false;
    }
    CHECK(vt.state().generation == 1, "authority activated alone");
    CHECK(gen_mismatch >= 1, "marker exposed the missed transition");
    CHECK(recover_ev >= 1, "follower recovered via authenticated control");
    CHECK(recovered_slot != 0, "follower adopted the live state");
    CHECK(vr.state().generation == vt.state().generation &&
              vr.state().active_mask == vt.state().active_mask &&
              vr.state().activate_slot == vt.state().activate_slot,
          "post-recovery states identical");
    /* bounded desync: recovery must land within one status interval of the
     * mismatch being detectable (activation), plus delivery slack */
    CHECK(recovered_slot - vt.state().activate_slot <=
              p.status_interval_slots + p.commit_repeat_slots + 2,
          "desync window bounded by the status cadence");
    /* and the two schedule views agree for every slot after recovery */
    for (uint64_t s = recovered_slot; s < slot; ++s)
      CHECK(vt.channel_index(s) == vr.channel_index(s),
            "post-recovery channel sequence identical");
  }

  /* ------- (c) the marker that crossed the activation boundary ----------
   * Both endpoints swap at the same absolute slot, but a frame stamped just
   * before it is decoded just after. Measured on air, reading that one frame
   * as a disagreement cost 46 seconds of re-acquisition at the exact instant
   * an exclusion took effect. It must be ignored — and only for as long as it
   * can plausibly still be in flight. */
  {
    HopsetAuthority auth(p, 0xC0DE);
    HopsetFollower fol(p, 0xF00D);
    uint32_t mismatches = 0;
    auto count = [&](const std::vector<HopsetAction> &acts) {
      for (const auto &a : acts)
        if (a.kind == HopsetAction::Event &&
            a.event == HopsetEvent::GenMismatch)
          ++mismatches;
    };
    HopsetMsg commit{};
    for (const auto &a : auth.start_change(0x3FE, 100))
      if (a.kind == HopsetAction::SendControl)
        commit = a.msg;
    count(fol.on_commit(commit, 100));
    const uint64_t act_slot = commit.activate_slot;
    for (uint64_t s = 100; s <= act_slot + 1; ++s) {
      auth.on_tick(s);
      count(fol.on_tick(s));
    }
    CHECK(fol.state().generation == 1, "follower reached generation 1");

    /* the straggler: a generation-0 marker STAMPED just before the swap,
     * decoded after it — the signature measured on air */
    count(fol.on_marker(0, /*fp_matches=*/false, act_slot - 1));
    CHECK(mismatches == 0,
          "a marker stamped before the swap is not a disagreement");
    count(fol.on_marker(0, false, act_slot + 1));
    CHECK(mismatches == 0, "nor is one inside the grace");
    CHECK(fol.fsm() == HopsetFollower::State::Synced,
          "lockstep survives the straggler");

    /* ...but an authority that genuinely never moved keeps saying so, and
     * past the grace that is exactly what a mismatch is for */
    count(fol.on_marker(0, false, act_slot + p.stale_marker_slots + 1));
    CHECK(mismatches == 1, "a persistent old generation still mismatches");
    CHECK(fol.fsm() == HopsetFollower::State::Recovering,
          "and still drives recovery");

    /* The grace measures against the activation anchor, so an authority that
     * announces an activation far in the future must not be able to stretch
     * it over every marker the follower will ever see — that would silently
     * disable the tripwire rather than trip it. The status path bounds the
     * activation window exactly as the commit path does. */
    HopsetMsg far{};
    far.type = HT_STATUS;
    far.role = 1;
    far.base_fp = p.base_fp;
    far.link_id = p.link_id;
    far.sender_epoch = 0xC0DE;
    far.generation = 9;
    far.active_mask = 0x3FE;
    far.activate_slot = act_slot + p.max_lead_slots + 1000;
    const uint64_t before = fol.state().generation;
    fol.on_status(far, act_slot + 2);
    CHECK(fol.state().generation == before,
          "a status announcing an out-of-window activation is refused");
  }

  return fails ? 1 : 0;
}
