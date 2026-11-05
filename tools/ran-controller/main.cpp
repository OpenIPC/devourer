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
#include <string>
#include <vector>

#if defined(DEVOURER_CELL_OPENSSL)
#include <openssl/evp.h>
#include <openssl/hmac.h>
#endif

#include "cell/Coord.h"
#include "cell/F1.h"

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
          if (auto r = f1::decode_report(buf, static_cast<size_t>(n), mac)) {
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
          if (auto a = f1::decode_ack(buf, static_cast<size_t>(n), mac)) {
            auto it = dus.find(a->du);
            const bool bad = a->status == f1::AckStatus::Stale || a->status == f1::AckStatus::Late ||
                             a->status == f1::AckStatus::Invalid;
            if (it != dus.end() && it->second.sched && it->second.sched->version == a->version && bad)
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
