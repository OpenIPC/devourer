/* ChanMigReplay — replay a recorded gate-input trace through the SHIPPING
 * MigGate policy (no python re-implementation, so a replay can never drift
 * from the deployed decision logic). Reads one JSON-lite object per line on
 * stdin and emits one migrate.gate decision per line on stdout.
 *
 * Trace line fields (all optional except mode; a Recommend needs target):
 *   mode        off|advisory|manual|automatic
 *   now_ms      caller clock (default: line number × 1000)
 *   kind        recommend|hold          (advisory decision kind)
 *   reason      a ChannelScore Reason name (for a hold's fault domain)
 *   target      "5:36/20"               (recommend target)
 *   score conf  candidate score/confidence
 *   gen         evidence generation
 *   telemetry_ok clock_synced scout_healthy usb_ok  (0/1)
 *   survey_age_ms control_margin rescue_verified in_flight approve  ...
 *   confirmed <chan> / rolledback <chan>   (resolution callbacks — advance state)
 *
 * The GateState persists across lines, so a whole session's anti-oscillation
 * behaviour replays. Driven by tests/chanmig_replay.py for metrics. */
#include "chanmig/JsonlLite.h"
#include "chanmig/MigGate.h"

#include <cstdio>
#include <string>

using namespace devourer::chanmig;

static MigMode parse_mode(const std::string &s) {
  if (s == "off") return MigMode::Off;
  if (s == "manual") return MigMode::Manual;
  if (s == "automatic") return MigMode::Automatic;
  return MigMode::Advisory;
}
static Reason parse_reason(const std::string &s) {
  if (s == "HoldActiveHealthy") return Reason::HoldActiveHealthy;
  if (s == "HoldImpairmentNotChannel") return Reason::HoldImpairmentNotChannel;
  if (s == "HoldBroadDegradation") return Reason::HoldBroadDegradation;
  if (s == "HoldScoutUnhealthy") return Reason::HoldScoutUnhealthy;
  if (s == "HoldImprovementMargin") return Reason::HoldImprovementMargin;
  if (s == "HoldCooldown") return Reason::HoldCooldown;
  return Reason::HoldNoQualifiedCandidate;
}

int main() {
  GatePolicy pol;
  GateState st;
  char buf[1024];
  int line_no = 0;
  while (std::fgets(buf, sizeof(buf), stdin)) {
    std::string_view line(buf);
    ++line_no;
    std::string s;
    long long iv;
    double dv;

    /* resolution callbacks advance the anti-oscillation state */
    if (jsonl_str(line, "confirmed", &s)) {
      ChannelDef c;
      std::string err;
      parse_chan_token(s, c, err);
      int64_t now = jsonl_int(line, "now_ms", &iv) ? iv : line_no * 1000;
      mig_gate_on_confirmed(st, c, now);
      std::printf("{\"ev\":\"migrate.replay\",\"act\":\"confirmed\"}\n");
      continue;
    }
    if (jsonl_str(line, "rolledback", &s)) {
      ChannelDef c;
      std::string err;
      parse_chan_token(s, c, err);
      int64_t now = jsonl_int(line, "now_ms", &iv) ? iv : line_no * 1000;
      mig_gate_on_rolledback(st, c, pol, now);
      std::printf("{\"ev\":\"migrate.replay\",\"act\":\"rolledback\"}\n");
      continue;
    }

    if (!jsonl_str(line, "mode", &s))
      continue;
    GateInputs in;
    in.mode = parse_mode(s);
    const int64_t now = jsonl_int(line, "now_ms", &iv) ? iv : line_no * 1000;

    Decision dec;
    ChannelDef target;
    bool have_rec = false;
    std::string kind;
    if (jsonl_str(line, "kind", &kind)) {
      if (kind == "recommend" && jsonl_str(line, "target", &s)) {
        std::string err;
        if (parse_chan_token(s, target, err)) {
          dec.kind = Decision::Kind::Recommend;
          dec.target = target;
          dec.primary_reason = Reason::RecommendBetterCandidate;
          dec.evidence_gen = jsonl_int(line, "gen", &iv) ? iv : 0;
          CandidateScore c;
          c.def = target;
          c.qualified = true;
          c.score = jsonl_num(line, "score", &dv) ? dv : 1.0;
          c.confidence = jsonl_num(line, "conf", &dv) ? dv : 1.0;
          dec.ranking.push_back(c);
          have_rec = true;
        }
      } else {
        dec.kind = Decision::Kind::Hold;
        std::string rs;
        dec.primary_reason =
            jsonl_str(line, "reason", &rs) ? parse_reason(rs)
                                           : Reason::HoldActiveHealthy;
        have_rec = true;
      }
    }
    in.rec = have_rec ? &dec : nullptr;

    auto flag = [&](const char *k, bool def) {
      return jsonl_int(line, k, &iv) ? iv != 0 : def;
    };
    in.telemetry_ok = flag("telemetry_ok", true);
    in.clock_synced = flag("clock_synced", true);
    in.scout_healthy = flag("scout_healthy", true);
    in.usb_ok = flag("usb_ok", true);
    in.rescue_verified = flag("rescue_verified", true);
    in.in_flight = flag("in_flight", false);
    in.approve_next = flag("approve", false);
    in.probation_active = flag("probation", false);
    in.probation_delivery_ok = flag("probation_ok", true);
    if (jsonl_int(line, "survey_age_ms", &iv))
      in.scout_survey_age_ms = iv;
    if (jsonl_num(line, "control_margin", &dv))
      in.control_link_margin = dv;

    GateOutcome o = mig_gate_decide(in, st, pol, now);
    const char *verd = o.verdict == GateVerdict::Propose ? "propose"
                       : o.verdict == GateVerdict::AbortInFlight
                           ? "abort"
                           : o.verdict == GateVerdict::ProposeRollback
                                 ? "rollback"
                                 : "hold";
    std::printf("{\"ev\":\"migrate.gate\",\"now\":%lld,\"mode\":\"%s\","
                "\"verdict\":\"%s\",\"reason\":\"%s\"",
                (long long)now, mig_mode_name(in.mode), verd,
                gate_reason_name(o.reason));
    if (o.verdict == GateVerdict::Propose) {
      char c[20];
      o.target.format(c, sizeof(c));
      std::printf(",\"to\":\"%s\",\"gen\":%llu", c,
                  (unsigned long long)o.evidence_gen);
    }
    std::printf("}\n");
  }
  return 0;
}
