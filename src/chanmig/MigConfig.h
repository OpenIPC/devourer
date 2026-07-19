/* MigConfig — timer/bound parameters for the migration state machines, plus
 * the pure string parsers the demo uses to map env → config (the library
 * reads no env; the demo does, matching the rest of devourer).
 *
 * The defaults are conservative starting points; the on-air clock bench
 * (tests/chanmig_clock_bench.sh) measures the real guard and the demo may
 * raise lead_ms from it. */
#ifndef DEVOURER_CHANMIG_MIG_CONFIG_H
#define DEVOURER_CHANMIG_MIG_CONFIG_H

#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

#include "chanmig/ChannelDef.h"

namespace devourer {
namespace chanmig {

struct MigParams {
  /* proposal (ground) */
  int64_t proposal_interval_ms = 200;
  int proposal_max_tries = 25;
  /* commit (drone) + status ack (ground) */
  int64_t commit_interval_ms = 100;
  int64_t status_interval_ms = 500;   /* 2 Hz drone status */
  int64_t ack_timeout_ms = 2000;      /* drone: no ground ack -> abort */
  /* activation */
  int64_t countdown_ms = 500;         /* drone: activation lead it picks */
  int64_t lead_ms = 40;               /* ground: retune this early (from clock) */
  int64_t min_lead_ms = 50;           /* activation bounds (drone TSF) */
  int64_t max_horizon_ms = 5000;
  int64_t drain_deadline_ms = 20;
  int64_t blank_window_ms = 5;        /* ground: drop stale post-retune frames */
  /* verify (ground) */
  int64_t verify_ms = 3000;
  int verify_markers = 3;
  int verify_video = 10;
  /* rollback / recovery */
  int64_t rollback_ms = 5000;         /* drone: rollback if unconfirmed by */
  int64_t recovery_dwell_ms = 500;    /* ground: per-channel scan dwell */
  int64_t recovery_giveup_ms = 60000;
  int64_t proposal_hold_down_ms = 10000; /* drone: after a rollback */
};

/* Parse a channel spec token, "5:104/40l" or "104/40u" or "36/80" — reuses the
 * ChannelDef grammar (parse_chan_token). Returns false + fills err on a bad
 * token. */
inline bool parse_mig_chanspec(const std::string &tok, ChannelDef &out,
                               std::string &err) {
  return parse_chan_token(tok, out, err);
}

/* Parse a comma-separated allowed-channel list. Malformed tokens are reported
 * in errs; the caller decides whether to run. */
inline bool parse_mig_allowed(const char *text, std::vector<ChannelDef> &out,
                              std::vector<PlanParseError> &errs) {
  out.clear();
  errs.clear();
  if (text == nullptr || *text == '\0')
    return false;
  const std::string s = text;
  size_t pos = 0;
  while (pos <= s.size()) {
    const size_t comma = s.find(',', pos);
    const std::string tok = s.substr(
        pos, comma == std::string::npos ? std::string::npos : comma - pos);
    pos = (comma == std::string::npos) ? s.size() + 1 : comma + 1;
    if (tok.empty())
      continue;
    ChannelDef d;
    std::string err;
    if (parse_chan_token(tok, d, err))
      out.push_back(d);
    else
      errs.push_back({tok, err});
  }
  return errs.empty() && !out.empty();
}

/* Fault-injection drop spec (demo-side, deterministic): "type:spec[,...]",
 * spec = N | N-M | * | every:K. Parsed into a per-type matcher the demo's
 * send/receive seam consults. Kept here so the grammar has one definition
 * shared by the on-air scripts' documentation and the code. */
struct DropRule {
  uint8_t type = 0;    /* MigMsgType, 0 = any */
  int lo = -1, hi = -1; /* index range, -1 = unset */
  int every = 0;       /* every:K, 0 = off */
  bool all = false;    /* '*' */
};

inline bool parse_drop_spec(const char *text, std::vector<DropRule> &out) {
  out.clear();
  if (text == nullptr || *text == '\0')
    return true;
  const std::string s = text;
  size_t pos = 0;
  while (pos <= s.size()) {
    const size_t comma = s.find(',', pos);
    std::string tok = s.substr(
        pos, comma == std::string::npos ? std::string::npos : comma - pos);
    pos = (comma == std::string::npos) ? s.size() + 1 : comma + 1;
    if (tok.empty())
      continue;
    DropRule r;
    const size_t colon = tok.find(':');
    std::string tstr = tok.substr(0, colon);
    r.type = static_cast<uint8_t>(std::strtoul(tstr.c_str(), nullptr, 0));
    if (colon == std::string::npos) {
      out.push_back(r);
      continue;
    }
    std::string spec = tok.substr(colon + 1);
    if (spec == "*") {
      r.all = true;
    } else if (spec.rfind("every:", 0) == 0) {
      r.every = std::atoi(spec.c_str() + 6);
    } else {
      const size_t dash = spec.find('-');
      if (dash == std::string::npos) {
        r.lo = r.hi = std::atoi(spec.c_str());
      } else {
        r.lo = std::atoi(spec.substr(0, dash).c_str());
        r.hi = std::atoi(spec.substr(dash + 1).c_str());
      }
    }
    out.push_back(r);
  }
  return true;
}

/* Should the index-th message of `type` be dropped under these rules? */
inline bool drop_matches(const std::vector<DropRule> &rules, uint8_t type,
                         int index) {
  for (const DropRule &r : rules) {
    if (r.type != 0 && r.type != type)
      continue;
    if (r.all)
      return true;
    if (r.every > 0 && index % r.every == 0)
      return true;
    if (r.lo >= 0 && index >= r.lo && index <= r.hi)
      return true;
  }
  return false;
}

} /* namespace chanmig */
} /* namespace devourer */

#endif /* DEVOURER_CHANMIG_MIG_CONFIG_H */
