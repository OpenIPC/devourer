/* hopset.* JSONL binding — one emitter for every action the pure machines
 * report, so the TX and RX demos (and the tests' log assertions) share one
 * schema. Every line carries the generation and mask so any transition is
 * reproducible from the logs alone. */
#ifndef DEVOURER_HOPSET_HOPSET_EVENTS_H
#define DEVOURER_HOPSET_HOPSET_EVENTS_H

#include "Event.h"
#include "hopset/HopsetPolicy.h"
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
