/* Headless guard for the M5 cell core: src/cell/Coord.h (the CU's
 * coordination planner) and src/cell/F1.h (the DU <-> CU messages and the
 * DU's failure model). The planner runs on the rig's two-cell layout and on
 * random scenarios: a plan with nothing unresolved must have no conflicts. */
#include <array>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include "cell/Coord.h"
#include "cell/F1.h"
#include "cell/SlotGrid.h"

using namespace devourer::cell;
using devourer::SyncTier;

static int failures = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      failures++;                                                              \
    }                                                                          \
  } while (0)

/* A keyed stand-in for HMAC-SHA1 (NOT cryptographic), as cell_m3. */
static Mac fake_mac(uint64_t key) {
  return [key](const uint8_t *d, size_t n, uint8_t out[20]) {
    uint64_t h = 1469598103934665603ull ^ key;
    for (size_t i = 0; i < n; ++i)
      h = (h ^ d[i]) * 1099511628211ull;
    for (int i = 0; i < 20; ++i) {
      h = (h ^ static_cast<uint64_t>(i)) * 1099511628211ull;
      out[i] = static_cast<uint8_t>(h >> 32);
    }
    return true;
  };
}

/* The rig layout (examples/cell defaults, capacity n): beacon 1500, DL 4000,
 * turnaround 1000, UL 2000. */
static CoordDu rig_du(uint16_t du, int64_t phase, size_t cap, int64_t rach_us = 0) {
  std::vector<uint16_t> pos;
  for (size_t i = 1; i <= cap; ++i)
    pos.push_back(static_cast<uint16_t>(i));
  auto g = build_fixed_grid(100, 0, 1500, pos, 4000, 1000, 2000, rach_us);
  CoordDu d;
  d.du = du;
  d.channel = 149;
  d.phase_us = phase;
  coord_layout(*g, d);
  return d;
}

static std::vector<CoordDu> with_plan(std::vector<CoordDu> dus, const CoordPlan &p) {
  for (CoordDu &d : dus) {
    d.channel = p.channel.at(d.du);
    const auto &a = p.assign.at(d.du);
    for (size_t k = 0; k < d.ues.size(); ++k)
      d.ues[k].pos = a[k].pos;
  }
  return dus;
}

int main() {
  /* --- Layout from the grid. --- */
  {
    CoordDu d = rig_du(1, 0, 3, 8000);
    CHECK(d.period_us == 102400);
    CHECK(d.positions.size() == 3);
    CHECK(d.fixed.size() == 2); /* beacon + RACH */
    CHECK(d.fixed[0].start_us == 0 && d.fixed[0].len_us == 1500);
    CHECK(d.fixed[1].len_us == 8000 + kRachTailGapUs);
    CHECK(d.positions[0].dl.start_us == 1500 && d.positions[0].dl.len_us == 4000);
    CHECK(d.positions[1].dl.start_us == 5500);
    CHECK(d.positions[0].ul.start_us == 1500 + 3 * 4000 + 1000);
  }

  /* --- The rig scenario: two co-channel cells, staggered one beacon slot,
   * each UE in position 1 and heard by the other DU. Uncoordinated, B's
   * beacon lands on A's DL; the plan clears it. --- */
  {
    CoordDu a = rig_du(1, 0, 3), b = rig_du(2, 1500, 3);
    a.ues.push_back({1, 1, {2}});
    b.ues.push_back({1, 1, {1}});
    const std::vector<CoordDu> dus = {a, b};
    const size_t before = coord_conflicts(dus);
    CHECK(before > 0);
    const CoordPlan p = coord_plan(dus);
    CHECK(p.unresolved == 0 && p.fixed_conflicts == 0 && p.retuned.empty());
    CHECK(p.moved >= 1);
    CHECK(coord_conflicts(with_plan(dus, p)) == 0);
    std::printf("cell_m5: rig layout: %zu conflicts uncoordinated -> 0 planned "
                "(A ue1 pos %u, B ue1 pos %u, %zu moved)\n",
                before, p.assign.at(1)[0].pos, p.assign.at(2)[0].pos, p.moved);
    /* Re-planning a clean schedule moves nothing (no churn). */
    const CoordPlan p2 = coord_plan(with_plan(dus, p));
    CHECK(p2.moved == 0 && p2.unresolved == 0);
  }

  /* --- No relation (UEs not heard across), or another channel: no moves. --- */
  {
    CoordDu a = rig_du(1, 0, 3), b = rig_du(2, 1500, 3);
    a.ues.push_back({1, 1, {}});
    b.ues.push_back({1, 1, {}});
    CHECK(coord_conflicts({a, b}) == 0);
    CHECK(coord_plan({a, b}).moved == 0);
    a.ues[0].heard_by = {2};
    b.ues[0].heard_by = {1};
    b.channel = 36;
    CHECK(coord_conflicts({a, b}) == 0 && coord_plan({a, b}).moved == 0);
  }

  /* --- Tiers: a coarse DU related to a co-channel DU gets a spare channel,
   * never time slots; with no spare it stays (and conflicts remain). --- */
  {
    CoordDu a = rig_du(1, 0, 3), b = rig_du(2, 1500, 3);
    a.ues.push_back({1, 1, {2}});
    b.ues.push_back({1, 1, {1}});
    b.tier = SyncTier::Coarse;
    const CoordPlan p = coord_plan({a, b}, {36, 149});
    CHECK(p.channel.at(2) == 36 && p.retuned.size() == 1 && p.retuned[0] == 2);
    CHECK(p.moved == 0); /* separated in frequency: nothing to move */
    /* No spare left: never a resolved shared-channel plan for it. */
    const CoordPlan q = coord_plan({a, b}, {149});
    CHECK(q.retuned.empty() && q.unseparated.size() == 1 && q.unseparated[0] == 2);
    CHECK(q.unresolved >= 1);
  }

  /* --- Infeasible in time (both full, everything heard): a spare channel
   * resolves it; without one the plan reports it unresolved. --- */
  {
    CoordDu a = rig_du(1, 0, 2), b = rig_du(2, 0, 2); /* same phase: beacons collide */
    a.ues = {{1, 1, {2}}, {2, 2, {2}}};
    b.ues = {{1, 1, {1}}, {2, 2, {1}}};
    const CoordPlan none = coord_plan({a, b});
    CHECK(none.fixed_conflicts > 0 && none.unresolved > 0 && none.retuned.empty());
    const CoordPlan sep = coord_plan({a, b}, {36});
    CHECK(sep.retuned.size() == 1 && sep.unresolved == 0 && sep.fixed_conflicts == 0);
    CHECK(coord_conflicts(with_plan({a, b}, sep)) == 0);
  }

  /* --- Random scenarios: whenever a plan resolves everything, it has no
   * conflicts; positions are unique per DU. --- */
  {
    std::mt19937 rng(254);
    int resolved = 0;
    for (int it = 0; it < 400; ++it) {
      std::vector<CoordDu> dus;
      const int ndu = 2 + static_cast<int>(rng() % 3);
      for (int k = 0; k < ndu; ++k) {
        CoordDu d = rig_du(static_cast<uint16_t>(k + 1), static_cast<int64_t>(rng() % 102400),
                           2 + rng() % 5, (rng() % 4 == 0) ? 6000 : 0);
        d.channel = (rng() % 4 == 0) ? 36 : 149;
        const size_t nue = rng() % (d.positions.size() + 1);
        for (size_t u = 0; u < nue; ++u) {
          CoordUe ue{static_cast<uint16_t>(u + 1), static_cast<uint8_t>(u + 1), {}};
          for (int o = 0; o < ndu; ++o)
            if (o != k && rng() % 2)
              ue.heard_by.push_back(static_cast<uint16_t>(o + 1));
          d.ues.push_back(ue);
        }
        dus.push_back(d);
      }
      const CoordPlan p = coord_plan(dus, {40, 44});
      for (const CoordDu &d : dus) {
        std::vector<int> seen(32, 0);
        for (const CoordAssign &a : p.assign.at(d.du)) {
          CHECK(a.pos >= 1 && a.pos <= d.positions.size());
          CHECK(++seen[a.pos] == 1);
        }
      }
      if (p.unresolved == 0 && p.fixed_conflicts == 0) {
        ++resolved;
        CHECK(coord_conflicts(with_plan(dus, p)) == 0);
      }
    }
    CHECK(resolved > 200);
    std::printf("cell_m5: %d/400 random scenarios resolved, all conflict-free\n", resolved);
  }

  /* --- F1 messages: round trips, tags, malformed input. --- */
  {
    const Mac k1 = fake_mac(1), k2 = fake_mac(2);
    f1::Report r;
    r.du = 7;
    r.tier = 2;
    r.sync_state = 2;
    r.coordinated = true;
    r.channel = 149;
    r.phase_us = 1500;
    r.period_us = 102400;
    r.net_sf = 0x1234567890ull;
    r.net_off = -17518356000ll;
    r.seq = 0x0102030405060708ull;
    r.applied_version = 4;
    r.staged_version = 5;
    CoordDu d = rig_du(7, 1500, 3, 6000);
    r.fixed = d.fixed;
    r.positions = d.positions;
    r.ues = {{1, 2}, {3, 1}};
    r.heard = {{2, 1, -61, 25, 100}, {3, 4, -90, 3, 1999}};
    r.meas = {{1, -55 * 4, 31 * 4, 0}, {3, -83 * 4 - 2, 7 * 4 + 1, 40}};
    const auto w = f1::encode_report(r, k1);
    CHECK(!w.empty() && f1::peek_type(w.data(), w.size()) == f1::Type::Report);
    CHECK(f1::decode_report(w.data(), w.size(), k1) == r);
    CHECK(!f1::decode_report(w.data(), w.size(), k2));       /* wrong key */
    CHECK(!f1::decode_schedule(w.data(), w.size(), k1));     /* wrong type */
    const auto plain = f1::encode_report(r, {});
    CHECK(!f1::decode_report(plain.data(), plain.size(), k1)); /* keyed rejects unkeyed */
    CHECK(f1::decode_report(plain.data(), plain.size(), {}) == r);
    for (size_t cut = 0; cut < w.size(); ++cut) /* every truncation refused */
      CHECK(!f1::decode_report(w.data(), cut, k1));
    auto flip = w;
    flip[10] ^= 1;
    CHECK(!f1::decode_report(flip.data(), flip.size(), k1));

    f1::Schedule s{7, 9, 1000, 0x1122334455667788ull, {{1, 3}, {3, 1}}};
    const auto ws = f1::encode_schedule(s, k1);
    CHECK(f1::decode_schedule(ws.data(), ws.size(), k1) == s);
    f1::Ack a{7, f1::AckKind::Schedule, 9, f1::AckStatus::Late, 990};
    const auto wa = f1::encode_ack(a, k1);
    CHECK(f1::decode_ack(wa.data(), wa.size(), k1) == a);
    auto bad = wa; /* an unknown status byte is refused (unkeyed: tag ignored) */
    bad[4 + 2 + 1 + 4] = 9;
    CHECK(!f1::decode_ack(bad.data(), bad.size(), {}));
    f1::Report big = r;
    big.heard.resize(f1::kMaxHeard + 1);
    CHECK(f1::encode_report(big, k1).empty());
  }

  /* --- The DU's failure model. --- */
  {
    f1::StagerConfig sc;
    sc.min_lead_sf = 6;
    sc.publish_lead_sf = 3;
    sc.cu_timeout_ms = 3000;
    f1::Stager st(sc);
    uint64_t seq = 100;
    auto S = [&](uint32_t v, uint64_t eff) { return f1::Schedule{1, v, eff, ++seq, {}}; };
    auto ok = [](const f1::Schedule &) { return true; };
    auto no = [](const f1::Schedule &) { return false; };
    CHECK(!st.cu_alive(0));
    /* Too close to its instant: Late; the DU carries nothing. */
    CHECK(st.on_schedule(S(1, 104), 100, 0, ok) == f1::AckStatus::Late);
    /* One the DU cannot carry: Invalid. */
    CHECK(st.on_schedule(S(1, 120), 100, 0, no) == f1::AckStatus::Invalid);
    CHECK(st.cu_alive(10));
    CHECK(st.on_schedule(S(2, 120), 100, 0, ok) == f1::AckStatus::Staged);
    CHECK(st.on_schedule(S(2, 120), 101, 100, ok) == f1::AckStatus::Staged); /* keepalive */
    CHECK(st.on_schedule(S(1, 130), 101, 100, ok) == f1::AckStatus::Stale);
    /* A newer one supersedes the staged one before it is due. */
    CHECK(st.on_schedule(S(3, 125), 102, 200, ok) == f1::AckStatus::Staged);
    CHECK(st.staged_version() == 3);
    CHECK(!st.published(102)); /* never released by due(): not applied */
    CHECK(st.staged_version() == 3 && st.applied_version() == 0);
    CHECK(!st.due(110));  /* not yet: 125 - 3 = 122 */
    auto d = st.due(122);
    CHECK(d && d->version == 3);
    /* While it is being published, nothing overtakes it. */
    CHECK(st.on_schedule(S(4, 140), 122, 300, ok) == f1::AckStatus::Late);
    CHECK(st.published(122));
    CHECK(st.applied_version() == 3 && st.staged_version() == 0);
    CHECK(st.on_schedule(S(3, 125), 123, 400, ok) == f1::AckStatus::Applied);
    CHECK(st.on_schedule(S(2, 140), 123, 400, ok) == f1::AckStatus::Stale);
    /* An old applied version while a newer one is staged: Stale, not Applied. */
    CHECK(st.on_schedule(S(5, 140), 124, 500, ok) == f1::AckStatus::Staged);
    CHECK(st.on_schedule(S(3, 125), 124, 500, ok) == f1::AckStatus::Stale);
    /* A staged schedule whose publish never lands is abandoned, not applied
     * off its instant, and its version cannot come back. */
    CHECK(st.due(137));  /* publishing... but the publish keeps failing */
    CHECK(!st.due(139)); /* 139 + 2 > 140: missed */
    auto ab = st.take_abandoned();
    CHECK(ab && ab->version == 5 && st.applied_version() == 3 && st.staged_version() == 0);
    CHECK(st.on_schedule(S(5, 160), 139, 600, ok) == f1::AckStatus::Stale);
    /* A publication completing after its deadline is abandoned too. */
    CHECK(st.on_schedule(S(6, 160), 139, 600, ok) == f1::AckStatus::Staged);
    CHECK(st.due(157));
    CHECK(!st.published(159)); /* 159 + 2 > 160 */
    ab = st.take_abandoned();
    CHECK(ab && ab->version == 6 && st.applied_version() == 3);
    /* CU outage, and a replay that must not hide it. */
    const f1::Schedule last = S(7, 200);
    CHECK(st.on_schedule(last, 160, 1000, ok) == f1::AckStatus::Staged);
    CHECK(st.cu_alive(1000 + 3000) && !st.cu_alive(1000 + 3001));
    CHECK(st.on_schedule(last, 170, 3500, ok) == f1::AckStatus::Stale); /* replayed datagram */
    CHECK(!st.cu_alive(4100));
    f1::Schedule fresh = last;
    fresh.seq = ++seq; /* the CU itself, resending (new sequence) */
    CHECK(st.on_schedule(fresh, 170, 4100, ok) == f1::AckStatus::Staged);
    CHECK(st.cu_alive(4100));
  }

  if (failures == 0)
    std::printf("cell_m5_selftest: all checks passed\n");
  return failures == 0 ? 0 : 1;
}
