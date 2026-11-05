// ran-controller — the M5 (#259) CU: collects the DUs' F1 reports, plans
// the facility-wide slot map with src/cell/Coord.h, and pushes each DU its
// schedule (src/cell/F1.h). It never touches a radio; it needs no PTP
// clock either: the DUs report the network superframe they are in.
//
// Per plan round (RAN_PLAN_MS) with every expected DU reporting and locked:
//   - the interference relation from the reports: UE u of cell c is heard
//     by DU o when o reports (c, u) at RSSI >= RAN_EDGE_DBM;
//   - the plan (RAN_PLAN=1), or the identity (RAN_PLAN=0: every UE stays
//     where its DU put it — the uncoordinated baseline, with the CU present
//     so the DUs still run coordinated, deterministic access);
//   - a DU whose assignment changed gets a new schedule version; every
//     changed DU takes effect on ONE network superframe (now + RAN_LEAD_SF);
//     unchanged ones get their current schedule again (the keepalive).
// A DU that answers Stale / Late / Invalid gets a fresh version next round.
// Channel separation the planner proposes is logged (ran.retune), not
// applied: the DUs retune only at start (M5 scope).
//
// Env (defaults in brackets):
//   RAN_PORT [47254]          UDP port the DUs report to
//   RAN_F1_KEY                the F1 key (HMAC-SHA1; = the DUs' CELL_F1_KEY)
//   RAN_DUS                   expected DU (cell) ids, comma-separated
//   RAN_PLAN [1]              1 = coordinate, 0 = identity (baseline)
//   RAN_EDGE_DBM [-90]        RSSI from which another DU "hears" a UE
//   RAN_LEAD_SF [12]          schedule lead (> the DUs' CELL_F1_MIN_LEAD_SF)
//   RAN_PLAN_MS [1000]        plan / keepalive period
//   RAN_SPARE_CHANNELS        channels the planner may propose (advisory)
// Handover evaluation (M6, #260) — OBSERVE ONLY: for every UE the CU runs the
// A3 (CHO) condition (src/cell/Handover.h) on the DUs' filtered reports —
// the serving DU's own measurement against every other DU's heard one — and
// logs when a handover would trigger (ran.ho_candidate) and when the
// condition clears (ran.ho_clear). Nothing is pushed to the DUs.
//   RAN_HO [1]                0 = off
//   RAN_A3_METRIC [rssi]      rssi | snr
//   RAN_A3_OFFSET_DB [3] RAN_A3_HYST_DB [1] RAN_A3_TTT_MS [640]
//   RAN_HO_EXECUTE [0]        1 = act on a candidate (M6): a handover command
//                             (src/cell/HoCommand.h) to both DUs over F1: the
//                             UE's lowest free id and first free position at
//                             the target, the switch RAN_HO_LEAD_SF [24]
//                             superframes ahead, the TSF delta from the two
//                             DUs' (phase, net_off). Two-phase: sent to the
//                             target until it stages the UE, only then to the
//                             source (which airs it to the UE); ran.ho_done
//                             when the target reports the UE. One handover in
//                             flight per DU pair end.
//   RAN_A3_DWELL_MS [2000]    after a handover, the moved UE's A3 dwell
//   RAN_MEAS_STALE_MS [2000]  a reported value counts until its sample age
//                             (carried in F1) plus the report's age passes
//                             this (= the DUs' CELL_MEAS_STALE_MS)
// The A3 runs once per RAN_PLAN_MS on the latest reports (the DUs report
// every CELL_F1_REPORT_MS), so the effective time-to-trigger rounds up to
// that period: 640 ms fires on the round after it, about 1 s at the
// defaults.
//   RAN_SECS [120]            run time
// Output: JSONL on stdout ("ev":"ran.*").
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#if defined(DEVOURER_CELL_OPENSSL)
#include <openssl/evp.h>
#include <openssl/hmac.h>
#endif

#include "cell/Coord.h"
#include "cell/F1.h"
#include "cell/Handover.h"

namespace f1 = devourer::cell::f1;
using devourer::cell::CoordAssign;
using devourer::cell::CoordDu;
using devourer::cell::CoordUe;

namespace {

long envl(const char *k, long d, long lo, long hi) {
  const char *v = std::getenv(k);
  if (!v || !*v)
    return d;
  char *end = nullptr;
  const long x = std::strtol(v, &end, 0);
  if (*end || x < lo || x > hi) {
    fprintf(stderr, "invalid %s=\"%s\" (want an integer in %ld..%ld)\n", k, v, lo, hi);
    std::exit(2);
  }
  return x;
}
std::vector<long> env_list(const char *k, long lo, long hi) {
  std::vector<long> v;
  const char *s = std::getenv(k);
  if (!s || !*s)
    return v;
  std::string t = s;
  for (size_t p = 0; p <= t.size();) {
    const size_t q = t.find(',', p);
    char *end = nullptr;
    const std::string item = t.substr(p, q == std::string::npos ? std::string::npos : q - p);
    const long x = std::strtol(item.c_str(), &end, 10);
    if (item.empty() || *end || x < lo || x > hi) {
      fprintf(stderr, "invalid %s=\"%s\"\n", k, s);
      std::exit(2);
    }
    v.push_back(x);
    if (q == std::string::npos)
      break;
    p = q + 1;
  }
  return v;
}
devourer::cell::Mac hmac_sha1(const std::string &key) {
  if (key.empty())
    return {};
#if defined(DEVOURER_CELL_OPENSSL)
  return [key](const uint8_t *d, size_t n, uint8_t out[20]) {
    unsigned int len = 0;
    return HMAC(EVP_sha1(), key.data(), static_cast<int>(key.size()), d, n, out, &len) != nullptr &&
           len == 20;
  };
#else
  fprintf(stderr, "RAN_F1_KEY needs a build with OpenSSL\n");
  std::exit(2);
#endif
}
void emit(const std::string &s) {
  fputs(s.c_str(), stdout);
  fputc('\n', stdout);
  fflush(stdout);
}
std::string assign_json(const std::vector<CoordAssign> &v) {
  std::string s = "{";
  for (size_t i = 0; i < v.size(); ++i)
    s += (i ? ",\"" : "\"") + std::to_string(v[i].ue) + "\":" + std::to_string(v[i].pos);
  return s + "}";
}

struct DuState {
  f1::Report rep;
  sockaddr_in addr{};
  std::chrono::steady_clock::time_point at;
  std::optional<f1::Schedule> sched; /* the schedule this DU should be running */
};

}  // namespace

int main() {
  const int port = static_cast<int>(envl("RAN_PORT", 47254, 1, 65535));
  const std::string key = std::getenv("RAN_F1_KEY") ? std::getenv("RAN_F1_KEY") : "";
  const auto mac = hmac_sha1(key);
  const auto expected = env_list("RAN_DUS", 1, 65535);
  const bool plan_on = envl("RAN_PLAN", 1, 0, 1) != 0;
  const int edge_dbm = static_cast<int>(envl("RAN_EDGE_DBM", -90, -127, 0));
  const uint64_t lead_sf = static_cast<uint64_t>(envl("RAN_LEAD_SF", 12, 2, 100000));
  const int plan_ms = static_cast<int>(envl("RAN_PLAN_MS", 1000, 50, 60000));
  const int secs = static_cast<int>(envl("RAN_SECS", 120, 1, 86400));
  const bool ho_on = envl("RAN_HO", 1, 0, 1) != 0;
  const char *metric_env = std::getenv("RAN_A3_METRIC");
  const std::string metric = metric_env && *metric_env ? metric_env : "rssi";
  if (metric != "rssi" && metric != "snr") {
    fprintf(stderr, "RAN_A3_METRIC must be rssi or snr\n");
    return 2;
  }
  auto envd = [](const char *k, double d, double lo, double hi) {
    const char *v = std::getenv(k);
    if (!v || !*v)
      return d;
    char *end = nullptr;
    const double x = std::strtod(v, &end);
    if (*end || !(x >= lo && x <= hi)) {
      fprintf(stderr, "invalid %s=\"%s\" (want %g..%g)\n", k, v, lo, hi);
      std::exit(2);
    }
    return x;
  };
  devourer::cell::A3Config a3c;
  a3c.offset_db = envd("RAN_A3_OFFSET_DB", 3.0, -30.0, 30.0);
  a3c.hysteresis_db = envd("RAN_A3_HYST_DB", 1.0, 0.0, 20.0);
  a3c.ttt_ms = static_cast<int64_t>(envl("RAN_A3_TTT_MS", 640, 0, 60000));
  const bool ho_exec = envl("RAN_HO_EXECUTE", 0, 0, 1) != 0;
  const uint64_t ho_lead_sf = static_cast<uint64_t>(envl("RAN_HO_LEAD_SF", 24, 4, 10000));
  const int64_t ho_dwell_ms = envl("RAN_A3_DWELL_MS", 2000, 0, 600000);
  a3c.min_dwell_ms = ho_dwell_ms; /* counted from a moved UE's switch */
  const int64_t meas_stale_ms = envl("RAN_MEAS_STALE_MS", 2000, 100, 60000);
  std::vector<uint8_t> spare;
  for (long ch : env_list("RAN_SPARE_CHANNELS", 1, 196))
    spare.push_back(static_cast<uint8_t>(ch));
  if (expected.empty()) {
    fprintf(stderr, "set RAN_DUS (the DU cell ids to coordinate)\n");
    return 2;
  }

  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in me{};
  me.sin_family = AF_INET;
  me.sin_addr.s_addr = htonl(INADDR_ANY);
  me.sin_port = htons(static_cast<uint16_t>(port));
  if (fd < 0 || bind(fd, reinterpret_cast<sockaddr *>(&me), sizeof me) != 0) {
    fprintf(stderr, "bind UDP %d failed\n", port);
    return 1;
  }
  emit("{\"ev\":\"ran.start\",\"port\":" + std::to_string(port) + ",\"plan\":" +
       std::to_string(plan_on ? 1 : 0) + ",\"edge_dbm\":" + std::to_string(edge_dbm) +
       ",\"lead_sf\":" + std::to_string(lead_sf) + ",\"auth\":" + (mac ? "1" : "0") + "}");

  std::map<uint16_t, DuState> dus;
  /* Per DU, the last version issued. Versions start from this CU's start
   * time (s), so a restarted CU's versions still move forward: a DU never
   * mistakes a new CU's schedule for the one it already runs. */
  const uint32_t version_base = static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
  std::map<uint16_t, uint32_t> version;
  const auto t0 = std::chrono::steady_clock::now();
  const auto deadline = t0 + std::chrono::seconds(secs);
  auto next_plan = t0 + std::chrono::milliseconds(plan_ms);
  auto next_meas = t0 + std::chrono::seconds(10);
  /* Per UE (its serving cell, its id): the A3 state, and whether a
   * candidate is currently logged as firing. */
  struct HoState {
    devourer::cell::A3Trigger a3;
    std::optional<uint16_t> firing;
  };
  std::map<std::pair<uint16_t, uint16_t>, HoState> ho;
  /* Handovers in flight, by the UE's (source cell, id). */
  struct HoFlight {
    devourer::cell::HoCommand cmd;
    bool src_ok = false, tgt_ok = false, switched = false;
  };
  std::map<std::pair<uint16_t, uint16_t>, HoFlight> flights;
  std::map<std::pair<uint16_t, uint16_t>, int64_t> moved_at; /* new key -> switch time (ms) */
  /* Old keys of UEs that handed over: the source keeps reporting their last
   * measurement until it goes stale, which must not trigger again. */
  std::set<std::pair<uint16_t, uint16_t>> retired;
  uint32_t ho_last_id = 0;
  auto ms_now = [&] {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0)
        .count();
  };
  /* Schedule freshness (F1.h): the realtime clock in us, strictly
   * increasing, so a restarted CU's sequence still moves forward. */
  uint64_t last_seq = 0;
  auto next_seq = [&] {
    const uint64_t rt = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
    last_seq = std::max(last_seq + 1, rt);
    return last_seq;
  };
  auto send = [&](const DuState &d, const std::vector<uint8_t> &m) {
    if (!m.empty())
      sendto(fd, m.data(), m.size(), 0, reinterpret_cast<const sockaddr *>(&d.addr), sizeof d.addr);
  };
  while (std::chrono::steady_clock::now() < deadline) {
    pollfd pf{fd, POLLIN, 0};
    if (poll(&pf, 1, 50) > 0 && (pf.revents & POLLIN)) {
      uint8_t buf[2048];
      sockaddr_in from{};
      socklen_t fl = sizeof from;
      const ssize_t n = recvfrom(fd, buf, sizeof buf, 0, reinterpret_cast<sockaddr *>(&from), &fl);
      if (n > 0) {
        const auto ty = f1::peek_type(buf, static_cast<size_t>(n));
        if (ty == f1::Type::Report) {
          auto r = f1::decode_report(buf, static_cast<size_t>(n), mac);
          /* Only the configured DUs: no state for any other id, however
           * many a sender makes up. */
          if (r && std::find(expected.begin(), expected.end(), static_cast<long>(r->du)) ==
                       expected.end())
            r.reset();
          if (r) {
            DuState &d = dus[r->du];
            const bool first = d.at.time_since_epoch().count() == 0;
            /* Freshness: a report at or below the last one taken from this DU
             * is a replay (or reordering) and changes nothing, not even the
             * address this DU's schedules go to. */
            if (!first && r->seq <= d.rep.seq)
              continue;
            d.rep = *r;
            d.addr = from;
            d.at = std::chrono::steady_clock::now();
            if (first) {
              char a[INET_ADDRSTRLEN] = {};
              inet_ntop(AF_INET, &from.sin_addr, a, sizeof a);
              emit("{\"ev\":\"ran.du\",\"du\":" + std::to_string(r->du) + ",\"addr\":\"" + a +
                   ":" + std::to_string(ntohs(from.sin_port)) + "\",\"tier\":" +
                   std::to_string(r->tier) + ",\"phase_us\":" + std::to_string(r->phase_us) +
                   ",\"positions\":" + std::to_string(r->positions.size()) + "}");
            }
          }
        } else if (ty == f1::Type::Ack) {
          if (auto a = f1::decode_ack(buf, static_cast<size_t>(n), mac);
              a && a->kind == f1::AckKind::Handover) {
            for (auto it = flights.begin(); it != flights.end(); ++it) {
              HoFlight &fl = it->second;
              if (fl.cmd.id != a->version)
                continue;
              const bool ok = a->status == f1::AckStatus::Staged;
              const bool was = a->du == fl.cmd.source ? fl.src_ok : fl.tgt_ok;
              (a->du == fl.cmd.source ? fl.src_ok : fl.tgt_ok) = ok;
              if (!ok) {
                emit("{\"ev\":\"ran.ho_failed\",\"id\":" + std::to_string(fl.cmd.id) +
                     ",\"du\":" + std::to_string(a->du) + ",\"status\":\"" +
                     f1::ack_status_name(a->status) + "\"}");
                /* A target that already staged the UE drops it again. */
                if (fl.tgt_ok || a->du == fl.cmd.source)
                  if (auto t = dus.find(fl.cmd.target); t != dus.end()) {
                    devourer::cell::HoCommand cancel = fl.cmd;
                    cancel.target_pos = 0;
                    cancel.seq = next_seq();
                    send(t->second, f1::encode_handover(fl.cmd.target, cancel, mac));
                  }
                flights.erase(it); /* the A3 fires again later: a fresh command */
              } else if (!was) {
                emit("{\"ev\":\"ran.ho_ack\",\"id\":" + std::to_string(fl.cmd.id) +
                     ",\"du\":" + std::to_string(a->du) + ",\"role\":\"" +
                     (a->du == fl.cmd.source ? "source" : "target") + "\"}");
                /* The target has it: the source (and so the UE) now, not on the
                 * next round — the lead is spent waiting otherwise. */
                if (a->du == fl.cmd.target && !fl.src_ok)
                  if (auto s2 = dus.find(fl.cmd.source); s2 != dus.end()) {
                    fl.cmd.seq = next_seq();
                    send(s2->second, f1::encode_handover(fl.cmd.source, fl.cmd, mac));
                  }
              }
              break;
            }
          } else if (a) {
            auto it = dus.find(a->du);
            const bool bad = a->status == f1::AckStatus::Stale || a->status == f1::AckStatus::Late ||
                             a->status == f1::AckStatus::Invalid;
            if (a->kind == f1::AckKind::Schedule && it != dus.end() && it->second.sched &&
                it->second.sched->version == a->version && bad)
              it->second.sched.reset(); /* re-issue a fresh version next round */
            static std::map<uint16_t, std::pair<uint32_t, int>> last; /* log changes only */
            auto &l = last[a->du];
            if (l.first != a->version || l.second != static_cast<int>(a->status)) {
              l = {a->version, static_cast<int>(a->status)};
              emit("{\"ev\":\"ran.ack\",\"du\":" + std::to_string(a->du) + ",\"version\":" +
                   std::to_string(a->version) + ",\"status\":\"" + f1::ack_status_name(a->status) +
                   "\",\"net_sf\":" + std::to_string(a->net_sf) + "}");
            }
          }
        }
      }
    }
    const auto now = std::chrono::steady_clock::now();
    /* The DUs' filtered uplink measurements (M6), every 10 s. */
    if (now >= next_meas) {
      next_meas = now + std::chrono::seconds(10);
      for (const auto &kv : dus) {
        std::string m = "{\"ev\":\"ran.meas\",\"du\":" + std::to_string(kv.first) + ",\"own\":[";
        for (size_t i = 0; i < kv.second.rep.meas.size(); ++i) {
          const auto &q = kv.second.rep.meas[i];
          char b[64];
          snprintf(b, sizeof b, "%s[%u,%.2f,%.2f]", i ? "," : "", q.ue, q.rssi_qdbm / 4.0,
                   q.snr_qdb / 4.0);
          m += b;
        }
        m += "],\"heard\":[";
        for (size_t i = 0; i < kv.second.rep.heard.size(); ++i) {
          const auto &h = kv.second.rep.heard[i];
          char b[64];
          snprintf(b, sizeof b, "%s[%u,%u,%d,%d]", i ? "," : "", h.cell, h.ue, h.rssi_dbm, h.snr_db);
          m += b;
        }
        emit(m + "]}");
      }
    }
    /* A3, every plan round, on the configured DUs' latest reports. A value
     * counts only while its sample is younger than meas_stale_ms (its age
     * at report time plus the report's age since). */
    if (ho_on && now >= next_plan) {
      const int64_t tms = ms_now();
      auto live = [&](const DuState &d, uint16_t age_ms) {
        const int64_t rep_age =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - d.at).count();
        return d.at.time_since_epoch().count() != 0 && age_ms + rep_age <= meas_stale_ms;
      };
      auto log_candidate = [&](uint16_t ue, uint16_t from, uint16_t to, double sv, double tv,
                               int64_t held) {
        char b[256];
        snprintf(b, sizeof b,
                 "{\"ev\":\"ran.ho_candidate\",\"t_ms\":%lld,\"ue\":%u,\"from\":%u,"
                 "\"to\":%u,\"metric\":\"%s\",\"serving\":%.2f,\"target\":%.2f,"
                 "\"held_ms\":%lld}",
                 (long long)tms, ue, from, to, metric.c_str(), sv, tv, (long long)held);
        emit(b);
      };
      auto log_clear = [&](uint16_t ue, uint16_t from, uint16_t to, const char *why) {
        char b[200];
        snprintf(b, sizeof b,
                 "{\"ev\":\"ran.ho_clear\",\"t_ms\":%lld,\"ue\":%u,\"from\":%u,"
                 "\"to\":%u,\"why\":\"%s\"}",
                 (long long)tms, ue, from, to, why);
        emit(b);
      };
      std::set<std::pair<uint16_t, uint16_t>> seen, still_retired;
      for (long id : expected) {
        auto dit = dus.find(static_cast<uint16_t>(id));
        if (dit == dus.end())
          continue;
        const uint16_t cell = dit->first;
        for (const f1::UeMeas &q : dit->second.rep.meas) {
          if (!live(dit->second, q.age_ms))
            continue; /* no valid serving measurement: handled below */
          const double serving = metric == "rssi" ? q.rssi_qdbm / 4.0 : q.snr_qdb / 4.0;
          std::map<uint16_t, std::optional<double>> neigh;
          for (long oid : expected) {
            if (oid == cell)
              continue;
            std::optional<double> v;
            auto oit = dus.find(static_cast<uint16_t>(oid));
            if (oit != dus.end())
              for (const f1::HeardUe &h : oit->second.rep.heard)
                if (h.cell == cell && h.ue == q.ue && live(oit->second, h.age_ms))
                  v = metric == "rssi" ? h.rssi_dbm : h.snr_db;
            neigh[static_cast<uint16_t>(oid)] = v; /* absent or expired: no evidence */
          }
          const auto key = std::make_pair(cell, q.ue);
          if (retired.count(key)) {
            still_retired.insert(key); /* still reported: keep ignoring it */
            continue;
          }
          seen.insert(key);
          auto it = ho.find(key);
          if (it == ho.end())
            it = ho.emplace(key, HoState{devourer::cell::A3Trigger(cell, a3c), std::nullopt}).first;
          HoState &st = it->second;
          if (auto mv = moved_at.find(key); mv != moved_at.end()) { /* a moved UE: dwell */
            st.a3.handed_over(cell, mv->second);
            moved_at.erase(mv);
          }
          const auto tgt = st.a3.evaluate(serving, neigh, tms);
          if (tgt != st.firing) {
            if (st.firing) /* also when switching straight to another target */
              log_clear(q.ue, cell, *st.firing, tgt ? "retarget" : "condition");
            if (tgt) {
              const auto since = st.a3.holding_since(*tgt);
              log_candidate(q.ue, cell, *tgt, serving, neigh.at(*tgt).value_or(0),
                            since ? tms - *since : 0);
            }
          }
          st.firing = tgt;
          /* Execute: one command per candidate, one handover in flight per
           * DU (source or target). */
          if (ho_exec && tgt) {
            bool busy = flights.count(key) > 0;
            for (const auto &fl : flights)
              busy = busy || fl.second.cmd.target == *tgt || fl.second.cmd.source == *tgt ||
                     fl.second.cmd.target == cell || fl.second.cmd.source == cell;
            auto tit = dus.find(*tgt);
            if (!busy && tit != dus.end() && tit->second.rep.net_sf > 0 && dit->second.rep.net_sf > 0) {
              const f1::Report &sr = dit->second.rep, &tr = tit->second.rep;
              uint16_t tue = 0;
              for (uint16_t u = 1; u <= tr.positions.size() && !tue; ++u) {
                bool used = false;
                for (const auto &a : tr.ues)
                  used = used || a.ue == u;
                if (!used)
                  tue = u;
              }
              uint8_t tpos = 0;
              for (uint8_t p = 1; p <= tr.positions.size() && !tpos; ++p) {
                bool used = false;
                for (const auto &a : tr.ues)
                  used = used || a.pos == p;
                if (!used)
                  tpos = p;
              }
              if (tue && tpos) {
                devourer::cell::HoCommand cmd;
                ho_last_id = std::max(ho_last_id + 1, version_base);
                cmd.id = ho_last_id;
                cmd.ue = q.ue;
                cmd.source = cell;
                cmd.target = *tgt;
                cmd.target_ue = tue;
                cmd.target_pos = tpos;
                const double age_us =
                    std::chrono::duration<double, std::micro>(now - dit->second.at).count();
                cmd.effective_net_sf =
                    sr.net_sf + static_cast<uint64_t>(age_us / sr.period_us) + ho_lead_sf;
                cmd.tsf_delta_us = devourer::cell::ho_tsf_delta_us(sr.phase_us, sr.net_off, tr.phase_us,
                                                                   tr.net_off, sr.period_us);
                flights[key] = HoFlight{cmd};
                char b[300];
                snprintf(b, sizeof b,
                         "{\"ev\":\"ran.ho_cmd\",\"id\":%u,\"ue\":%u,\"from\":%u,\"to\":%u,"
                         "\"target_ue\":%u,\"pos\":%u,\"effective_net_sf\":%llu,\"delta_us\":%lld}",
                         cmd.id, cmd.ue, cmd.source, cmd.target, cmd.target_ue, cmd.target_pos,
                         (unsigned long long)cmd.effective_net_sf, (long long)cmd.tsf_delta_us);
                emit(b);
              }
            }
          }
        }
      }
      /* In flight: (re)send to both DUs until staged; conclude when the
       * target reports the UE; give up long after the instant. */
      for (auto it = flights.begin(); it != flights.end();) {
        HoFlight &fl = it->second;
        auto sit = dus.find(fl.cmd.source), tit = dus.find(fl.cmd.target);
        if (sit == dus.end() || tit == dus.end()) {
          it = flights.erase(it);
          continue;
        }
        const double age_us = std::chrono::duration<double, std::micro>(now - tit->second.at).count();
        const uint64_t tnet =
            tit->second.rep.net_sf + static_cast<uint64_t>(age_us / tit->second.rep.period_us);
        bool there = false;
        for (const f1::UeMeas &q : tit->second.rep.meas)
          there = there || (q.ue == fl.cmd.target_ue && live(tit->second, q.age_ms));
        if (there && tnet >= fl.cmd.effective_net_sf) {
          char b[220];
          snprintf(b, sizeof b,
                   "{\"ev\":\"ran.ho_done\",\"id\":%u,\"ue\":%u,\"from\":%u,\"to\":%u,"
                   "\"target_ue\":%u,\"net_sf\":%llu,\"after_sf\":%lld}",
                   fl.cmd.id, fl.cmd.ue, fl.cmd.source, fl.cmd.target, fl.cmd.target_ue,
                   (unsigned long long)tnet, (long long)(tnet - fl.cmd.effective_net_sf));
          emit(b);
          moved_at[{fl.cmd.target, fl.cmd.target_ue}] = tms;
          retired.insert({fl.cmd.source, fl.cmd.ue});
          still_retired.insert({fl.cmd.source, fl.cmd.ue});
          if (auto h = ho.find({fl.cmd.source, fl.cmd.ue}); h != ho.end()) {
            if (h->second.firing)
              log_clear(fl.cmd.ue, fl.cmd.source, *h->second.firing, "handed_over");
            ho.erase(h);
          }
          it = flights.erase(it);
          continue;
        }
        if (tnet > fl.cmd.effective_net_sf + 100) { /* ~10 s past: did not happen */
          emit("{\"ev\":\"ran.ho_timeout\",\"id\":" + std::to_string(fl.cmd.id) + "}");
          it = flights.erase(it);
          continue;
        }
        /* Two-phase: the target first; the source (which tells the UE) only
         * once the target has staged it, so a target that refuses never
         * leaves a command on the air. */
        if (tnet < fl.cmd.effective_net_sf && !(fl.src_ok && fl.tgt_ok)) {
          fl.cmd.seq = next_seq();
          const uint16_t du = fl.tgt_ok ? fl.cmd.source : fl.cmd.target;
          send(dus.at(du), f1::encode_handover(du, fl.cmd, mac));
        }
        ++it;
      }
      /* A retired key the source no longer reports is forgotten (the id may
       * be reused by a new UE there). */
      retired = still_retired;
      /* A UE without a valid serving measurement this round: its evidence
       * is gone, so its timers go, and a logged candidate clears. */
      for (auto it = ho.begin(); it != ho.end();) {
        if (seen.count(it->first)) {
          ++it;
          continue;
        }
        if (it->second.firing)
          log_clear(it->first.second, it->first.first, *it->second.firing, "no_serving");
        it = ho.erase(it);
      }
    }
    if (now < next_plan)
      continue;
    next_plan = now + std::chrono::milliseconds(plan_ms);
    /* The keepalive, every round and whatever the planning state: every DU
     * keeps its current schedule (a fresh sequence each send) while one
     * DU cannot be planned (e.g. it is re-locking). Only the plan waits. */
    for (auto &kv : dus)
      if (kv.second.sched && now - kv.second.at < std::chrono::seconds(10)) {
        kv.second.sched->seq = next_seq();
        send(kv.second, f1::encode_schedule(*kv.second.sched, mac));
      }

    /* Every expected DU fresh, locked and on the network count. */
    bool ready = true;
    for (long id : expected) {
      auto it = dus.find(static_cast<uint16_t>(id));
      ready = ready && it != dus.end() && now - it->second.at < std::chrono::seconds(3) &&
              it->second.rep.net_sf > 0 && it->second.rep.sync_state == 2;
    }
    if (!ready)
      continue;
    std::vector<CoordDu> view;
    uint64_t net_now = 0;
    for (long id : expected) {
      const DuState &d = dus.at(static_cast<uint16_t>(id));
      CoordDu c;
      c.du = d.rep.du;
      c.channel = d.rep.channel;
      c.tier = static_cast<devourer::SyncTier>(d.rep.tier);
      c.phase_us = d.rep.phase_us;
      c.period_us = d.rep.period_us;
      c.fixed = d.rep.fixed;
      c.positions = d.rep.positions;
      for (const CoordAssign &a : d.rep.ues) {
        CoordUe u{a.ue, a.pos, {}};
        for (long oid : expected) {
          const DuState &o = dus.at(static_cast<uint16_t>(oid));
          if (o.rep.du == c.du)
            continue;
          for (const f1::HeardUe &h : o.rep.heard)
            if (h.cell == c.du && h.ue == a.ue && h.rssi_dbm >= edge_dbm)
              u.heard_by.push_back(o.rep.du);
        }
        c.ues.push_back(u);
      }
      view.push_back(c);
      /* The network superframe now, from the report's age. */
      const double age_us =
          std::chrono::duration<double, std::micro>(now - d.at).count();
      const uint64_t est = d.rep.net_sf + static_cast<uint64_t>(age_us / d.rep.period_us);
      net_now = std::max(net_now, est);
    }
    const size_t before = devourer::cell::coord_conflicts(view);
    std::map<uint16_t, std::vector<CoordAssign>> target;
    devourer::cell::CoordPlan p;
    if (plan_on) {
      /* The positions pushed are planned on the channels the DUs are on: a
       * retune the CU only proposes (below) must not make a co-channel
       * conflict look resolved. */
      p = devourer::cell::coord_plan(view, {});
      target = p.assign;
      if (!spare.empty() && (p.unresolved || p.fixed_conflicts || !p.unseparated.empty())) {
        const auto adv = devourer::cell::coord_plan(view, spare);
        static std::string last_adv; /* log a proposal once, not every round */
        std::string txt;
        for (uint16_t du : adv.retuned)
          txt += "{\"ev\":\"ran.retune\",\"du\":" + std::to_string(du) + ",\"channel\":" +
                 std::to_string(adv.channel.at(du)) + ",\"applied\":0}\n";
        if (!txt.empty() && txt != last_adv) {
          last_adv = txt;
          fputs(txt.c_str(), stdout);
          fflush(stdout);
        }
      }
    } else {
      for (const CoordDu &c : view)
        for (const CoordUe &u : c.ues)
          target[c.du].push_back({u.ue, u.pos});
    }
    std::vector<CoordDu> after_view = view;
    for (CoordDu &c : after_view)
      for (size_t k = 0; k < c.ues.size(); ++k)
        c.ues[k].pos = target[c.du][k].pos;
    const size_t after = devourer::cell::coord_conflicts(after_view);
    static std::string last_plan;
    std::string pl = "{\"ev\":\"ran.plan\",\"net_sf\":" + std::to_string(net_now) +
                     ",\"conflicts_now\":" + std::to_string(before) + ",\"conflicts_planned\":" +
                     std::to_string(after) + ",\"moved\":" + std::to_string(p.moved) +
                     ",\"unresolved\":" + std::to_string(p.unresolved) + ",\"fixed_conflicts\":" +
                     std::to_string(p.fixed_conflicts) + ",\"unseparated\":" +
                     std::to_string(p.unseparated.size()) + ",\"edges\":{";
    bool firstk = true;
    for (const CoordDu &c : view)
      for (const CoordUe &u : c.ues)
        for (uint16_t o : u.heard_by) {
          pl += std::string(firstk ? "" : ",") + "\"" + std::to_string(c.du) + "/" +
                std::to_string(u.ue) + "\":" + std::to_string(o);
          firstk = false;
        }
    pl += "}}";
    if (pl.substr(pl.find("\"conflicts_now\"")) != last_plan) { /* log on change */
      last_plan = pl.substr(pl.find("\"conflicts_now\""));
      emit(pl);
    }

    /* Changed DUs: new versions, one network-wide instant. */
    const uint64_t eff = net_now + lead_sf;
    for (const CoordDu &c : view) {
      DuState &d = dus.at(c.du);
      const auto &want = target[c.du];
      if (!d.sched || d.sched->ues != want) {
        /* Above anything this CU issued AND anything the DU reports applied
         * or staged (a restarted CU, or a DU that outlived several CUs). */
        uint32_t &v = version[c.du];
        v = std::max({v ? v + 1 : version_base, d.rep.applied_version + 1,
                      d.rep.staged_version + 1});
        d.sched = f1::Schedule{c.du, v, eff, next_seq(), want};
        emit("{\"ev\":\"ran.schedule\",\"du\":" + std::to_string(c.du) + ",\"version\":" +
             std::to_string(d.sched->version) + ",\"effective_net_sf\":" + std::to_string(eff) +
             ",\"net_sf\":" + std::to_string(net_now) + ",\"ues\":" + assign_json(want) + "}");
        send(d, f1::encode_schedule(*d.sched, mac));
      }
    }
  }
  emit("{\"ev\":\"ran.done\"}");
  close(fd);
  return 0;
}
