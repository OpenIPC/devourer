/* channel.recommend / channel.hold / channel.reject JSONL binding — the
 * advisory engine's output, emitted from a Decision. One header so the schema
 * has a single definition (chanscout emits; the migration controller and the
 * dashboard parse). Every event carries the evidence generation, plan hash,
 * and policy hash, so any decision is reproducible from the logs. */
#ifndef DEVOURER_CHANMIG_CHANNEL_EVENTS_H
#define DEVOURER_CHANMIG_CHANNEL_EVENTS_H

#include "Event.h"
#include "chanmig/ChannelScore.h"

namespace devourer {
namespace chanmig {

inline const char *impair_name(Impair im) {
  switch (im) {
  case Impair::Healthy: return "healthy";
  case Impair::Channel: return "channel";
  case Impair::WeakSignal: return "weak_signal";
  case Impair::Saturation: return "saturation";
  case Impair::TelemetryDown: return "telemetry_down";
  }
  return "?";
}

/* Emit the decision. `recommend` and `hold` share one envelope (kind field);
 * a candidate that lost qualification since the previous decision is emitted
 * as a separate channel.reject so a consumer watching only rejects sees it. */
inline void emit_decision(EventSink &sink, const Decision &d,
                          const ChannelDef &active) {
  Ev ev(sink, d.kind == Decision::Kind::Recommend ? "channel.recommend"
                                                   : "channel.hold");
  ev.t().f("v", 1);
  ev.hexf("gen", d.evidence_gen, 0);
  ev.hexf("plan", d.plan_hash, 8);
  ev.hexf("policy", d.policy_hash, 8);
  char from[20];
  active.format(from, sizeof(from));
  ev.f("from", from).f("reason", reason_name(d.primary_reason));
  ev.f("active_verdict", verdict_label(d.active_verdict));
  ev.f("active_domain", impair_name(d.active_domain))
      .f("impaired_windows", d.active_impaired_windows);
  if (d.kind == Decision::Kind::Recommend) {
    char to[20];
    d.target.format(to, sizeof(to));
    ev.f("to", to);
    for (const CandidateScore &c : d.ranking)
      if (c.def.same_rf(d.target)) {
        ev.f("score", c.score).f("conf", c.confidence).f("occ", c.occ_q50)
            .f("rounds", (unsigned long long)c.rounds)
            .f("obs_ms", (unsigned long long)c.observe_ms);
        break;
      }
  }
  ev.f("text", d.human_reason.c_str());
}

/* Full per-candidate ranking as one line, so the dashboard can render the
 * counterfactual (why NOT each candidate) purely from the log. */
inline void emit_ranking(EventSink &sink, const Decision &d) {
  Ev ev(sink, "channel.ranking");
  ev.t().hexf("gen", d.evidence_gen, 0).f("n", (long long)d.ranking.size());
  int rank = 0;
  for (const CandidateScore &c : d.ranking) {
    char chan[20];
    c.def.format(chan, sizeof(chan));
    /* one flat field per candidate, keyed by rank index — no nested objects */
    char key[8];
    std::snprintf(key, sizeof(key), "c%d", rank);
    char buf[112];
    std::snprintf(buf, sizeof(buf), "%s q=%d score=%.2f occ=%.2f rej=%d", chan,
                  c.qualified ? 1 : 0, c.score, c.occ_q50, c.n_rejections);
    ev.f(key, buf);
    ++rank;
  }
}

/* Emit a candidate that just lost qualification (a channel.reject watcher). */
inline void emit_reject(EventSink &sink, const CandidateScore &c,
                        uint64_t gen) {
  Ev ev(sink, "channel.reject");
  char chan[20];
  c.def.format(chan, sizeof(chan));
  ev.t().hexf("gen", gen, 0).f("chan", chan);
  if (c.n_rejections > 0)
    ev.f("reason", reason_name(c.rejections[0]));
  ev.f("occ", c.occ_q50).f("age_ms", (long long)c.evidence_age_ms)
      .f("rounds", (unsigned long long)c.rounds);
}

} /* namespace chanmig */
} /* namespace devourer */

#endif /* DEVOURER_CHANMIG_CHANNEL_EVENTS_H */
