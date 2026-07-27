/* hopset.* JSONL binding — one emitter for every action the pure machines
 * report, so the TX and RX demos (and the tests' log assertions) share one
 * schema. Every line carries the generation and mask so any transition is
 * reproducible from the logs alone. */
#ifndef DEVOURER_HOPSET_HOPSET_EVENTS_H
#define DEVOURER_HOPSET_HOPSET_EVENTS_H

#include "Event.h"
#include "hopset/HopsetFusion.h"
#include "hopset/HopsetPolicy.h"
#include "hopset/HopsetSense.h"
#include "hopset/HopsetTypes.h"

namespace devourer {
namespace hopset {

/* Emit one machine-reported Event action. role: "tx" | "rx". */
inline void emit_action(EventSink &sink, const HopsetAction &a,
                        const char *role, uint64_t now_slot) {
  if (a.kind != HopsetAction::Event)
    return;
  Ev ev(sink, hopset_event_name(a.event));
  ev.t().f("v", 1).f("role", role).f("slot", (unsigned long long)now_slot);
  switch (a.event) {
  case HopsetEvent::Propose:
    ev.f("gen", (unsigned long long)a.msg.generation)
        .hexf("mask", a.msg.active_mask, 0)
        .f("obs", (unsigned long long)a.msg.observation_count)
        .hexf("reasons", a.msg.reason_bitmap, 0);
    break;
  case HopsetEvent::Commit:
  case HopsetEvent::Activate:
  case HopsetEvent::Recover:
    ev.f("gen", (unsigned long long)a.state.generation)
        .hexf("mask", a.state.active_mask, 0)
        .f("activate_round", (unsigned long long)a.state.activate_round)
        .f("activate_slot", (unsigned long long)a.state.activate_slot);
    break;
  case HopsetEvent::Reject:
    ev.f("reason", hopset_reason_name(a.reason));
    break;
  case HopsetEvent::GenMismatch:
    ev.f("seen_gen", (unsigned long long)a.msg.generation)
        .f("cur_gen", (unsigned long long)a.state.generation)
        .hexf("cur_mask", a.state.active_mask, 0);
    break;
  }
}

/* One policy verdict. `verbose` adds the per-channel evidence the decision
 * was made from, so a log replay can re-derive it without the receiver. */
inline void emit_decision(EventSink &sink, const HopsetDecision &d,
                          uint64_t slot, uint64_t round, bool verbose) {
  Ev ev(sink, "hopset.decision");
  ev.t().f("v", 1).f("role", "rx").f("slot", (unsigned long long)slot)
      .f("round", (unsigned long long)round);
  ev.f("kind", d.kind == HopsetDecision::Kind::ProposeExclude   ? "exclude"
                : d.kind == HopsetDecision::Kind::ProposeRestore ? "restore"
                                                                 : "hold");
  ev.f("obs", (unsigned long long)d.observation_count)
      .hexf("policy", d.policy_hash, 8);
  if (d.kind == HopsetDecision::Kind::Hold) {
    ev.f("hold", hopset_hold_name(d.hold));
  } else {
    ev.hexf("mask", d.proposed_mask, 0)
        .hexf("reasons", d.reason_bitmap, 0)
        .f("target", (unsigned long long)d.target_index);
  }
  if (!verbose)
    return;
  for (size_t i = 0; i < d.chans.size(); ++i) {
    char key[8], buf[64];
    std::snprintf(key, sizeof(key), "c%zu", i);
    std::snprintf(buf, sizeof(buf), "%s d=%.2f p=%.2f v=%u imp=%u",
                  d.chans[i].active ? "on" : "off", d.chans[i].delivery,
                  d.chans[i].probe_delivery, d.chans[i].visits,
                  d.chans[i].impaired_run);
    ev.f(key, buf);
  }
}

/* One transmitter quiet-window observation. Raw deltas AND the window they
 * span, because the classifier normalizes them itself and a pre-divided rate
 * loses the low end; `rate` is a convenience for log-scraping harnesses. */
inline void emit_sense(EventSink &sink, const TxSenseSample &s,
                       int channel, uint32_t generation, uint32_t settle_us,
                       uint32_t read_us, double occupancy, bool scored) {
  Ev ev(sink, "hopset.sense");
  ev.t().f("v", 1).f("role", "tx")
      .f("slot", (unsigned long long)s.slot)
      .f("round", (unsigned long long)s.round)
      .f("gen", (unsigned long long)generation)
      .f("base_idx", (unsigned long long)s.base_index)
      .f("ch", channel)
      .f("phase", s.phase == SensePhase::PostBurst ? "post" : "pre")
      .f("probe", s.probe)
      .f("window_us", (unsigned long long)s.window_us)
      .f("settle_us", (unsigned long long)settle_us)
      .f("read_us", (unsigned long long)read_us);
  ev.f("valid_fa", s.valid_fa);
  if (s.valid_fa) {
    ev.f("cca_ofdm", (unsigned long long)s.cca_ofdm)
        .f("cca_cck", (unsigned long long)s.cca_cck)
        .f("fa_ofdm", (unsigned long long)s.fa_ofdm)
        .f("fa_cck", (unsigned long long)s.fa_cck);
    const double ms = s.window_us ? s.window_us / 1000.0 : 1.0;
    ev.f("cca_rate", (s.cca_ofdm + s.cca_cck) / ms)
        .f("fa_rate", (s.fa_ofdm + s.fa_cck) / ms);
  }
  ev.f("valid_igi", s.valid_igi);
  if (s.valid_igi)
    ev.f("igi", (unsigned long long)s.igi);
  ev.f("valid_nhm", s.valid_nhm);
  if (s.valid_nhm)
    ev.f("nhm_busy", (unsigned long long)s.nhm_busy_pct)
        .f("nhm_dur", (unsigned long long)s.nhm_duration);
  if (s.injected)
    ev.f("inject", 1);
  ev.hexf("flags", s.flags, 0).f("scored", scored);
  if (scored)
    ev.f("occ", occupancy);
}

/* Why this side's sensor proposed nothing. Held is not the same as blind, and
 * neither is the same as content: without this, a sensor sitting on a floor,
 * a sensor with no evidence and a sensor that is not running all produce the
 * same silence. Rate-limited by the caller to the shape of the answer. */
inline void emit_sense_eval(EventSink &sink, const TxSenseCandidate &c,
                            uint64_t slot, uint64_t round) {
  Ev(sink, "hopset.sense_eval")
      .t()
      .f("v", 1)
      .f("role", "tx")
      .f("slot", (unsigned long long)slot)
      .f("round", (unsigned long long)round)
      .f("hold", tx_sense_hold_name(c.hold))
      .f("chans", (unsigned long long)c.chans_with_evidence)
      .f("samples", (unsigned long long)c.sample_count)
      .f("band_min", c.band_min_occ)
      .f("band_max", c.band_max_occ)
      .f("broad", c.broad_occupancy)
      .f("flat", c.flat_band);
}

/* One fused verdict. Both endpoints' views are on the line so a log reader can
 * see the argument, not just the conclusion — the whole point of surfacing a
 * disagreement rather than averaging it. */
inline void emit_fusion(EventSink &sink, const FusedDecision &d, uint64_t slot,
                        uint64_t round, const char *mode) {
  Ev ev(sink, "hopset.decision");
  ev.t().f("v", 1).f("role", "tx")
      .f("slot", (unsigned long long)slot)
      .f("round", (unsigned long long)round)
      .f("mode", mode)
      .f("origin", decision_origin_name(d.origin))
      .f("kind", fused_action_name(d.action));
  if (d.action == FusedAction::Hold || d.action == FusedAction::Delay)
    ev.f("hold", fusion_hold_name(d.hold));
  if (d.action != FusedAction::Hold && d.action != FusedAction::Delay)
    ev.hexf("mask", d.mask, 0);
  ev.f("target", (unsigned long long)d.target_index)
      .hexf("reasons", d.reason_bitmap, 0)
      .f("veto", fusion_veto_name(d.veto))
      .f("rx_wanted", d.rx_wanted_exclude)
      .f("tx_wanted", d.tx_wanted_exclude)
      .f("disagree", d.endpoints_disagree_on_target)
      .f("tx_valid", d.tx_evidence_valid);
  if (d.tx_evidence_valid)
    ev.f("tx_target_occ", d.tx_target_occupancy)
        .f("tx_band_min", d.tx_band_min_occupancy)
        .f("tx_band_max", d.tx_band_max_occupancy);
  if (d.feedback_age_rounds == kNoFeedback)
    ev.f("fb_age", "never");
  else
    ev.f("fb_age", (unsigned long long)d.feedback_age_rounds);
  ev.hexf("fusion", d.fusion_hash, 8);
}

/* One keyed recovery-probe dwell: the transmitter logs that it aired on the
 * excluded channel, the receiver logs whether anything arrived. */
inline void emit_probe(EventSink &sink, const char *role, uint64_t slot,
                       uint64_t round, uint32_t base_index, int channel,
                       bool delivered, uint32_t frames) {
  Ev(sink, "hopset.probe")
      .t()
      .f("v", 1)
      .f("role", role)
      .f("slot", (unsigned long long)slot)
      .f("round", (unsigned long long)round)
      .f("base_idx", (unsigned long long)base_index)
      .f("ch", channel)
      .f("delivered", delivered)
      .f("frames", (unsigned long long)frames);
}

} /* namespace hopset */
} /* namespace devourer */

#endif /* DEVOURER_HOPSET_HOPSET_EVENTS_H */
