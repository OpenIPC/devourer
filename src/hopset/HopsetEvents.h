/* hopset.* JSONL binding — one emitter for every action the pure machines
 * report, so the TX and RX demos (and the tests' log assertions) share one
 * schema. Every line carries the generation and mask so any transition is
 * reproducible from the logs alone. */
#ifndef DEVOURER_HOPSET_HOPSET_EVENTS_H
#define DEVOURER_HOPSET_HOPSET_EVENTS_H

#include "Event.h"
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

} /* namespace hopset */
} /* namespace devourer */

#endif /* DEVOURER_HOPSET_HOPSET_EVENTS_H */
