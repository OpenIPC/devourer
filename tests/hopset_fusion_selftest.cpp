/* Endpoint-evidence fusion: whose measurement wins, when the transmitter may
 * object, and what it may object to. The load-bearing row is the hidden node —
 * a receiver that is right and a transmitter that cannot see why must end with
 * the receiver's proposal passing through untouched, because that is the case
 * a naive "I disagree" veto gets systematically wrong. */
#include "hopset/HopsetFusion.h"
#include <cstdio>
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

constexpr size_t kN = 6;

HopsetDecision mk_rx(HopsetDecision::Kind k, size_t target, uint64_t mask) {
  HopsetDecision d;
  d.kind = k;
  d.target_index = target;
  d.proposed_mask = mask;
  d.chans.resize(kN);
  if (k == HopsetDecision::Kind::ProposeExclude)
    d.reason_bitmap = RB_DELIVERY_FLOOR | RB_PERSISTENT | RB_HEALTHY_ALT;
  return d;
}

/* A transmitter view: per-channel occupancy, all scored unless `scored` says
 * otherwise. `absent` marks the no-sensor case. */
TxSenseCandidate mk_tx(const std::vector<double> &occ,
                       TxSenseCandidate::Kind kind = TxSenseCandidate::Kind::None,
                       size_t target = 0, uint64_t mask = 0,
                       bool absent = false,
                       const std::vector<bool> &scored = {}) {
  TxSenseCandidate t;
  t.kind = kind;
  t.target_index = target;
  t.proposed_mask = mask;
  t.chans.resize(kN);
  double mn = 2.0, mx = -1.0;
  uint32_t n = 0, dirty = 0;
  for (size_t i = 0; i < kN && i < occ.size(); ++i) {
    const bool has = !absent && (scored.empty() || (i < scored.size() && scored[i]));
    t.chans[i].active = true;
    t.chans[i].sensor_absent = absent;
    if (!has)
      continue;
    t.chans[i].post.scored = true;
    t.chans[i].post.samples = 20;
    t.chans[i].post.occupancy = occ[i];
    ++n;
    if (occ[i] < mn) mn = occ[i];
    if (occ[i] > mx) mx = occ[i];
    if (occ[i] >= 0.60) ++dirty;
  }
  t.chans_with_evidence = n;
  t.band_min_occ = n ? mn : 0.0;
  t.band_max_occ = n ? mx : 0.0;
  t.broad_occupancy = n && double(dirty) / double(n) >= 0.50;
  t.flat_band = n >= 3 && (t.band_max_occ - t.band_min_occ) < 0.25;
  if (kind == TxSenseCandidate::Kind::ProposeExclude)
    t.reason_bitmap = RB_TX_CCA_HIGH | RB_TX_POST_BURST | RB_TX_CLEANER_ALT;
  return t;
}

FusionInput mk_in(const HopsetDecision &rx, const TxSenseCandidate &tx,
                  uint64_t mask, uint64_t fb_age) {
  FusionInput in;
  in.rx = &rx;
  in.tx = &tx;
  in.active_mask = mask;
  in.feedback_age_rounds = fb_age;
  in.now_round = 500;
  return in;
}

} // namespace

int main() {
  const uint64_t M = full_mask(kN);
  const FusionConfig def;

  /* --- the config hash --- */
  {
    FusionConfig a, b, c;
    CHECK(a.policy_hash() == b.policy_hash(), "fusion hash stable");
    b.mode = FusionMode::EitherEndpoint;
    CHECK(a.policy_hash() != b.policy_hash(), "fusion hash tracks fields");
    c.remaining_worse_margin = 0.21;
    CHECK(a.policy_hash() != c.policy_hash(), "fusion hash tracks doubles");
    FusionMode m;
    CHECK(parse_fusion_mode("veto", m) && m == FusionMode::RxPlusTxVeto,
          "mode parses");
    CHECK(!parse_fusion_mode("nonsense", m), "an unknown mode is refused");
  }

  /* === THE HIDDEN NODE: the receiver is right and the transmitter is blind.
   * The interferer sits at the far end, so the transmitter reads every channel
   * as quiet — including the one that is killing delivery. Its proposal must
   * pass through bit-for-bit. This is the #264 parked-jammer gate. === */
  {
    auto rx = mk_rx(HopsetDecision::Kind::ProposeExclude, 3,
                    M & ~(uint64_t(1) << 3));
    auto tx = mk_tx({0.08, 0.09, 0.07, 0.08, 0.10, 0.08});
    auto d = fuse(def, mk_in(rx, tx, M, 5));
    CHECK(d.action == FusedAction::Exclude, "hidden node: exclusion proceeds");
    CHECK(d.origin == DecisionOrigin::Rx, "hidden node: origin stays rx");
    CHECK(d.veto == FusionVeto::None, "hidden node: no veto");
    CHECK(d.mask == rx.proposed_mask, "hidden node: mask is the rx proposal");
    CHECK(d.endpoints_disagree_on_target && (d.reason_bitmap & RB_FUSE_TX_DISAGREES),
          "hidden node: the disagreement is surfaced, not acted on");
  }

  /* A maximally flat, maximally clean band is still not confusion. */
  {
    auto rx = mk_rx(HopsetDecision::Kind::ProposeExclude, 3,
                    M & ~(uint64_t(1) << 3));
    auto tx = mk_tx({0.02, 0.02, 0.02, 0.02, 0.02, 0.02});
    CHECK(tx.flat_band, "the band really is geometrically flat");
    auto d = fuse(def, mk_in(rx, tx, M, 5));
    CHECK(d.action == FusedAction::Exclude && d.veto == FusionVeto::None,
          "flat AND clean is confidence, not doubt");
  }

  /* --- transmitter-local interference with healthy delivery: never costs a
   * channel under the default mode --- */
  {
    auto rx = mk_rx(HopsetDecision::Kind::Hold, 0, 0);
    rx.hold = HopsetHold::NoImpairment;
    auto tx = mk_tx({0.05, 0.05, 0.90, 0.05, 0.05, 0.05},
                    TxSenseCandidate::Kind::ProposeExclude, 2,
                    M & ~(uint64_t(1) << 2));
    auto d = fuse(def, mk_in(rx, tx, M, 5));
    CHECK(d.action == FusedAction::Hold &&
              d.hold == FusionHold::ModeForbidsTxOrigin,
          "local noise alone never excludes while delivery is fine");
    CHECK(d.tx_wanted_exclude && !d.rx_wanted_exclude,
          "the transmitter's opinion is recorded");
    /* The on-air mirror test reads this record and nothing else, so the record
     * has to name the channel and the occupancy that were argued about — a
     * held decision that says only "held" is indistinguishable from a sensor
     * that never fired. */
    CHECK(d.endpoints_disagree_on_target &&
              (d.reason_bitmap & RB_FUSE_TX_DISAGREES),
          "the disagreement is surfaced on the held path too");
    CHECK(d.target_index == 2, "the held record names the tx's own target");
    CHECK(d.tx_evidence_valid && d.tx_target_occupancy > 0.8,
          "the held record carries the occupancy that drove it");
  }

  /* --- ground (a): a uniformly DIRTY band delays, it does not refuse --- */
  {
    auto rx = mk_rx(HopsetDecision::Kind::ProposeExclude, 3,
                    M & ~(uint64_t(1) << 3));
    auto tx = mk_tx({0.80, 0.82, 0.79, 0.81, 0.83, 0.80});
    auto d = fuse(def, mk_in(rx, tx, M, 5));
    CHECK(d.action == FusedAction::Delay, "broad tx degradation delays");
    CHECK(d.veto == FusionVeto::BroadTxDegradation &&
              d.hold == FusionHold::VetoDelay,
          "the delay names its ground");
    CHECK(d.origin == DecisionOrigin::Fused &&
              (d.reason_bitmap & RB_FUSE_VETO_BROAD),
          "the delay is attributed to fusion");
    CHECK(d.mask == 0, "a delay applies nothing");
  }

  /* flat-and-dirty just over the floor delays; the same geometry far below it
   * does not — the only difference is the absolute level */
  {
    auto rx = mk_rx(HopsetDecision::Kind::ProposeExclude, 3,
                    M & ~(uint64_t(1) << 3));
    auto hot = mk_tx({0.40, 0.42, 0.41, 0.40, 0.43, 0.40});
    auto cold = mk_tx({0.10, 0.12, 0.11, 0.10, 0.13, 0.10});
    CHECK(hot.flat_band && cold.flat_band, "both are geometrically flat");
    CHECK(fuse(def, mk_in(rx, hot, M, 5)).action == FusedAction::Delay,
          "flat and dirty delays");
    CHECK(fuse(def, mk_in(rx, cold, M, 5)).action == FusedAction::Exclude,
          "flat and clean passes");
  }

  /* a persistently flat view cannot suppress the receiver forever */
  {
    auto rx = mk_rx(HopsetDecision::Kind::ProposeExclude, 3,
                    M & ~(uint64_t(1) << 3));
    auto tx = mk_tx({0.80, 0.82, 0.79, 0.81, 0.83, 0.80});
    auto in = mk_in(rx, tx, M, 5);
    in.consecutive_delays = def.max_consecutive_delays;
    auto d = fuse(def, in);
    CHECK(d.action == FusedAction::Exclude,
          "after the delay budget the veto yields");
  }

  /* --- ground (b): every survivor is worse here, so the move is a
   * cul-de-sac --- */
  {
    auto rx = mk_rx(HopsetDecision::Kind::ProposeExclude, 3,
                    M & ~(uint64_t(1) << 3));
    auto tx = mk_tx({0.45, 0.48, 0.46, 0.10, 0.47, 0.49});
    auto d = fuse(def, mk_in(rx, tx, M, 5));
    CHECK(d.action == FusedAction::Hold && d.veto == FusionVeto::RemainingWorse,
          "remaining-worse rejects");
    CHECK(d.hold == FusionHold::VetoReject &&
              (d.reason_bitmap & RB_FUSE_VETO_REMAIN),
          "the rejection names its ground");
  }
  {
    /* one acceptable survivor kills it — the veto is unanimity-gated */
    auto rx = mk_rx(HopsetDecision::Kind::ProposeExclude, 3,
                    M & ~(uint64_t(1) << 3));
    auto tx = mk_tx({0.45, 0.48, 0.15, 0.10, 0.47, 0.49});
    CHECK(fuse(def, mk_in(rx, tx, M, 5)).action == FusedAction::Exclude,
          "one acceptable survivor kills the veto");
  }
  {
    /* an UNMEASURED survivor also kills it: an unexplored escape route is not
     * a road we may declare closed */
    auto rx = mk_rx(HopsetDecision::Kind::ProposeExclude, 3,
                    M & ~(uint64_t(1) << 3));
    auto tx = mk_tx({0.45, 0.48, 0.46, 0.10, 0.47, 0.49},
                    TxSenseCandidate::Kind::None, 0, 0, false,
                    {true, true, false, true, true, true});
    auto d = fuse(def, mk_in(rx, tx, M, 5));
    CHECK(d.action == FusedAction::Exclude && d.veto == FusionVeto::None,
          "an unmeasured survivor kills the veto");
  }

  /* --- restores are never vetoed --- */
  {
    auto rx = mk_rx(HopsetDecision::Kind::ProposeRestore, 3, M);
    auto tx = mk_tx({0.80, 0.82, 0.79, 0.81, 0.83, 0.80});
    auto d = fuse(def, mk_in(rx, tx, M & ~(uint64_t(1) << 3), 5));
    CHECK(d.action == FusedAction::Restore && d.veto == FusionVeto::None,
          "a restore is never vetoed");
  }

  /* --- with no sensor the veto is structurally unreachable: veto mode
   * degrades exactly to rx-authoritative --- */
  {
    auto rx = mk_rx(HopsetDecision::Kind::ProposeExclude, 3,
                    M & ~(uint64_t(1) << 3));
    auto tx = mk_tx({0, 0, 0, 0, 0, 0}, TxSenseCandidate::Kind::None, 0, 0,
                    /*absent=*/true);
    auto d = fuse(def, mk_in(rx, tx, M, 5));
    CHECK(d.action == FusedAction::Exclude, "sensor-absent cannot block");
    CHECK(d.veto == FusionVeto::SensorAbsent &&
              (d.reason_bitmap & RB_TX_SENSOR_ABSENT),
          "absence is recorded as the reason the veto was unreachable");
    CHECK(!d.tx_evidence_valid, "absence is not evidence");
  }
  {
    /* too few scored channels: also unreachable */
    auto rx = mk_rx(HopsetDecision::Kind::ProposeExclude, 3,
                    M & ~(uint64_t(1) << 3));
    auto tx = mk_tx({0.80, 0.82, 0.79, 0.81, 0.83, 0.80},
                    TxSenseCandidate::Kind::None, 0, 0, false,
                    {true, true, false, false, false, false});
    auto d = fuse(def, mk_in(rx, tx, M, 5));
    CHECK(d.action == FusedAction::Exclude &&
              d.veto == FusionVeto::InsufficientTxEvidence,
          "too little evidence cannot block");
  }

  /* --- rx_authoritative never vetoes and never originates --- */
  {
    FusionConfig cfg = def;
    cfg.mode = FusionMode::RxAuthoritative;
    auto rx = mk_rx(HopsetDecision::Kind::ProposeExclude, 3,
                    M & ~(uint64_t(1) << 3));
    auto tx = mk_tx({0.80, 0.82, 0.79, 0.81, 0.83, 0.80});
    CHECK(fuse(cfg, mk_in(rx, tx, M, 5)).action == FusedAction::Exclude,
          "rx_authoritative never vetoes");
    auto hold = mk_rx(HopsetDecision::Kind::Hold, 0, 0);
    auto want = mk_tx({0.05, 0.05, 0.90, 0.05, 0.05, 0.05},
                      TxSenseCandidate::Kind::ProposeExclude, 2,
                      M & ~(uint64_t(1) << 2));
    auto d = fuse(cfg, mk_in(hold, want, M, 5));
    CHECK(d.action == FusedAction::Hold &&
              d.hold == FusionHold::ModeForbidsTxOrigin,
          "rx_authoritative never originates");
  }

  /* --- the failsafe gate --- */
  {
    FusionConfig cfg = def;
    cfg.mode = FusionMode::TxFailsafe;
    auto hold = mk_rx(HopsetDecision::Kind::Hold, 0, 0);
    auto want = mk_tx({0.05, 0.05, 0.90, 0.05, 0.05, 0.05},
                      TxSenseCandidate::Kind::ProposeExclude, 2,
                      M & ~(uint64_t(1) << 2));
    auto stale = fuse(cfg, mk_in(hold, want, M, cfg.feedback_timeout_rounds + 1));
    CHECK(stale.action == FusedAction::Exclude &&
              stale.origin == DecisionOrigin::Failsafe,
          "a stale uplink lets the transmitter originate");
    CHECK(stale.mask == want.proposed_mask, "the mask is the tx proposal");
    CHECK(rb_origin(stale.reason_bitmap) == DecisionOrigin::Failsafe,
          "origin travels in the reason bitmap");
    auto fresh = fuse(cfg, mk_in(hold, want, M, cfg.feedback_timeout_rounds));
    CHECK(fresh.action == FusedAction::Hold &&
              fresh.hold == FusionHold::FeedbackAlive,
          "at the timeout boundary the uplink still counts as alive");
    auto never = fuse(cfg, mk_in(hold, want, M, kNoFeedback));
    CHECK(never.action == FusedAction::Exclude,
          "never having heard the peer counts as stale");
    /* with feedback alive the receiver keeps authority even in failsafe */
    auto rx = mk_rx(HopsetDecision::Kind::ProposeExclude, 1,
                    M & ~(uint64_t(1) << 1));
    auto d = fuse(cfg, mk_in(rx, want, M, 5));
    CHECK(d.origin == DecisionOrigin::Rx && d.target_index == 1,
          "a live receiver outranks the transmitter's own candidate");
  }

  /* --- feedback returns: an un-corroborated autonomous exclusion is handed
   * back, and a corroborated one is left standing --- */
  {
    FusionConfig cfg = def;
    auto rx = mk_rx(HopsetDecision::Kind::Hold, 0, 0);
    auto tx = mk_tx({0.05, 0.05, 0.05, 0.05, 0.05, 0.05});
    const uint64_t reduced = M & ~(uint64_t(1) << 2);
    FusionInput in = mk_in(rx, tx, reduced, 3);
    in.tx_originated_mask = uint64_t(1) << 2;
    in.feedback_fresh_since_round = 400;
    in.now_round = 400 + cfg.reexamine_grace_rounds;
    auto d = fuse(cfg, in);
    CHECK(d.action == FusedAction::Restore && d.target_index == 2 &&
              d.origin == DecisionOrigin::Fused,
          "an un-corroborated autonomous exclusion is re-examined");
    CHECK((d.reason_bitmap & RB_TX_REEXAMINED), "the restore names its reason");
    /* inside the grace window, nothing happens yet */
    in.now_round = 400 + cfg.reexamine_grace_rounds - 1;
    CHECK(fuse(cfg, in).action == FusedAction::Hold,
          "the grace window is respected");
    /* the receiver has since seen the channel deliver badly: it stands */
    in.now_round = 400 + cfg.reexamine_grace_rounds;
    auto rx2 = mk_rx(HopsetDecision::Kind::Hold, 0, 0);
    rx2.chans[2].scored = true;
    rx2.chans[2].delivery = 0.05;
    in.rx = &rx2;
    CHECK(fuse(cfg, in).action == FusedAction::Hold,
          "a corroborated autonomous exclusion is left standing");
  }

  /* --- either_endpoint: the receiver preempts, the transmitter fills gaps --- */
  {
    FusionConfig cfg = def;
    cfg.mode = FusionMode::EitherEndpoint;
    auto rx = mk_rx(HopsetDecision::Kind::ProposeExclude, 1,
                    M & ~(uint64_t(1) << 1));
    auto tx = mk_tx({0.05, 0.05, 0.05, 0.05, 0.90, 0.05},
                    TxSenseCandidate::Kind::ProposeExclude, 4,
                    M & ~(uint64_t(1) << 4));
    auto d = fuse(cfg, mk_in(rx, tx, M, 5));
    CHECK(d.origin == DecisionOrigin::Rx && d.target_index == 1,
          "either_endpoint: the receiver preempts");
    CHECK(d.endpoints_disagree_on_target, "the disagreement is surfaced");
    auto hold = mk_rx(HopsetDecision::Kind::Hold, 0, 0);
    auto e = fuse(cfg, mk_in(hold, tx, M, 5));
    CHECK(e.origin == DecisionOrigin::Tx && e.action == FusedAction::Exclude &&
              e.mask == tx.proposed_mask,
          "either_endpoint: the transmitter may originate");
    CHECK(rb_origin(e.reason_bitmap) == DecisionOrigin::Tx, "origin encoded");
  }

  /* --- the mode/decision precedence table, swept --- */
  {
    const FusionMode modes[] = {
        FusionMode::RxAuthoritative, FusionMode::RxPlusTxVeto,
        FusionMode::EitherEndpoint, FusionMode::TxFailsafe};
    auto clean = mk_tx({0.05, 0.05, 0.05, 0.05, 0.05, 0.05});
    auto wants = mk_tx({0.05, 0.05, 0.90, 0.05, 0.05, 0.05},
                       TxSenseCandidate::Kind::ProposeExclude, 2,
                       M & ~(uint64_t(1) << 2));
    for (FusionMode m : modes)
      for (int rxk = 0; rxk < 3; ++rxk)
        for (int txk = 0; txk < 2; ++txk)
          for (int fresh = 0; fresh < 2; ++fresh) {
            FusionConfig cfg = def;
            cfg.mode = m;
            auto rx = rxk == 0 ? mk_rx(HopsetDecision::Kind::Hold, 0, 0)
                      : rxk == 1
                          ? mk_rx(HopsetDecision::Kind::ProposeExclude, 3,
                                  M & ~(uint64_t(1) << 3))
                          : mk_rx(HopsetDecision::Kind::ProposeRestore, 3, M);
            const auto &tx = txk ? wants : clean;
            auto d = fuse(cfg, mk_in(rx, tx, M, fresh ? 5 : kNoFeedback));
            /* Invariant 1: the mask is always one endpoint's proposal
             * verbatim — nothing is ever blended. */
            CHECK(d.mask == 0 || d.mask == rx.proposed_mask ||
                      d.mask == tx.proposed_mask,
                  "the applied mask is never a blend");
            /* Invariant 2: the transmitter only ever originates in the two
             * modes that permit it, and the failsafe only when the uplink
             * is gone. */
            if (d.origin == DecisionOrigin::Tx)
              CHECK(m == FusionMode::EitherEndpoint, "tx origin only in either");
            if (d.origin == DecisionOrigin::Failsafe)
              CHECK(m == FusionMode::TxFailsafe && !fresh,
                    "failsafe origin only when the uplink is gone");
            /* Invariant 3: a receiver restore always survives. */
            if (rxk == 2)
              CHECK(d.action == FusedAction::Restore,
                    "a receiver restore always survives");
            /* Invariant 4: conflicting views are always reported. */
            if (d.rx_wanted_exclude != d.tx_wanted_exclude)
              CHECK(d.endpoints_disagree_on_target &&
                        (d.reason_bitmap & RB_FUSE_TX_DISAGREES),
                    "disagreement is always surfaced");
          }
  }

  /* --- origin round-trips and stays compatible with the pre-existing bits --- */
  {
    const DecisionOrigin all[] = {DecisionOrigin::Rx, DecisionOrigin::Tx,
                                  DecisionOrigin::Fused,
                                  DecisionOrigin::Failsafe};
    const uint32_t legacy = RB_DELIVERY_FLOOR | RB_PERSISTENT | RB_HEALTHY_ALT |
                            RB_PROBE_RECOVERY;
    for (DecisionOrigin o : all) {
      const uint32_t bm = rb_set_origin(legacy, o);
      CHECK(rb_origin(bm) == o, "origin round-trips");
      CHECK((bm & 0x1ffu) == legacy, "the receiver's own bits survive");
    }
    CHECK(rb_origin(legacy) == DecisionOrigin::Rx,
          "a bitmap written before this layer decodes as receiver-origin");
  }

  /* --- determinism --- */
  {
    auto rx = mk_rx(HopsetDecision::Kind::ProposeExclude, 3,
                    M & ~(uint64_t(1) << 3));
    auto tx = mk_tx({0.80, 0.10, 0.79, 0.10, 0.83, 0.80});
    auto a = fuse(def, mk_in(rx, tx, M, 7));
    auto b = fuse(def, mk_in(rx, tx, M, 7));
    CHECK(a.origin == b.origin && a.action == b.action && a.mask == b.mask &&
              a.veto == b.veto && a.hold == b.hold &&
              a.reason_bitmap == b.reason_bitmap,
          "identical inputs fuse identically");
  }

  return fails ? 1 : 0;
}
