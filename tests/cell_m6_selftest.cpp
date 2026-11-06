/* Headless guard for the M6 cell core: src/cell/Handover.h — the L3
 * measurement filter, the A3 (CHO) trigger, DAPS sequence dedup — and the
 * F1 v2 measurement fields. The centre piece is a noisy cell-edge crossing:
 * triggering on raw RSSI ping-pongs; the filtered A3 with hysteresis,
 * time-to-trigger and dwell moves once, after the crossing. */
#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <optional>
#include <random>
#include <vector>

#include "cell/F1.h"
#include "cell/GrantMap.h"
#include "cell/Handover.h"
#include "cell/HoCommand.h"
#include "cell/MeasGap.h"
#include "cell/PhasePlan.h"
#include "cell/SlotGrid.h"
#include "cell/TsfDiscipline.h"

using namespace devourer::cell;

static int failures = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      failures++;                                                              \
    }                                                                          \
  } while (0)

/* A keyed stand-in for HMAC-SHA1 (NOT cryptographic), as cell_m3/m5. */
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

int main() {
  /* --- L3 filter: seeding, the k=4 step response, staleness. --- */
  {
    L3Filter f({4, 2000});
    CHECK(!f.valid(0));
    f.update(-60, 0);
    CHECK(f.valid(0) && f.value() == -60); /* the first sample seeds it */
    for (int i = 1; i <= 20; ++i)
      f.update(-80, i * 100);
    /* a = 1/2: twenty halvings of the 20 dB step. */
    CHECK(std::fabs(f.value() + 80) < 1e-3);
    CHECK(f.valid(2000 + 2000) && !f.valid(2000 + 2001));
    f.reset();
    CHECK(!f.valid(2000));
    f.update(-50, 2100);
    CHECK(f.value() == -50);
    /* Samples resuming after a stale gap seed afresh, without reset(). */
    L3Filter g({4, 2000});
    g.update(-60, 0);
    g.update(-90, 2001 + 0); /* 2001 ms later: stale, so not blended */
    CHECK(g.value() == -90);
    g.update(-80, 2100);
    CHECK(std::fabs(g.value() + 85) < 1e-9); /* fresh again: blended (a = 1/2) */
    L3Filter raw({0, 2000}); /* k = 0: a = 1, no filtering */
    raw.update(-60, 0);
    raw.update(-70, 1);
    CHECK(raw.value() == -70);
  }

  /* --- A3: below the offset nothing; above it only after time-to-trigger;
   * leaving the condition resets the timer; the hysteresis band holds it. --- */
  {
    A3Config c;
    c.offset_db = 3;
    c.hysteresis_db = 1;
    c.ttt_ms = 640;
    c.min_dwell_ms = 2000;
    A3Trigger t(1, c);
    auto ev = [&](double s, double n, int64_t ms) {
      return t.evaluate(s, {{2, n}}, ms);
    };
    CHECK(!ev(-60, -57, 0));  /* +3: on the offset, inside the hysteresis */
    CHECK(!ev(-60, -55, 100)); /* +5 > 3 + 1: enters, timer starts */
    CHECK(t.holding_since(2) == 100);
    CHECK(!ev(-60, -55, 700)); /* 600 ms < ttt */
    CHECK(!ev(-60, -57.5, 739)); /* in the band: the timer keeps running */
    auto hit = ev(-60, -55, 740);
    CHECK(hit && *hit == 2);
    /* Leaving resets: */
    A3Trigger u(1, c);
    CHECK(!u.evaluate(-60, {{2, -55}}, 0));
    CHECK(!u.evaluate(-60, {{2, -59}}, 500)); /* -59 + 1 < -57: leaves */
    CHECK(!u.holding_since(2));
    CHECK(!u.evaluate(-60, {{2, -55}}, 600));
    CHECK(!u.evaluate(-60, {{2, -55}}, 1200)); /* 600 since re-entry */
    CHECK(u.evaluate(-60, {{2, -55}}, 1240));
    /* A stale serving or neighbour measurement: no evidence, timer reset. */
    A3Trigger v(1, c);
    CHECK(!v.evaluate(-60, {{2, -50}}, 0));
    CHECK(!v.evaluate(std::nullopt, {{2, -50}}, 700));
    CHECK(!v.holding_since(2));
    CHECK(!v.evaluate(-60, {{2, std::nullopt}}, 800));
    /* The best of several qualifying neighbours. */
    A3Trigger w(1, c);
    w.evaluate(-70, {{2, -60}, {3, -55}}, 0);
    auto b = w.evaluate(-70, {{2, -60}, {3, -55}}, 700);
    CHECK(b && *b == 3);
    /* ttt 0: the first qualifying evaluation fires. */
    A3Config z = c;
    z.ttt_ms = 0;
    A3Trigger zt(1, z);
    auto zh = zt.evaluate(-60, {{2, -50}}, 0);
    CHECK(zh && *zh == 2);
    /* Dwell: right after a handover nothing fires, even with a strong
     * neighbour held past time-to-trigger. */
    w.handed_over(3, 700);
    CHECK(w.serving() == 3);
    w.evaluate(-70, {{1, -50}}, 800);
    CHECK(!w.evaluate(-70, {{1, -50}}, 1500)); /* ttt met, dwell not */
    CHECK(w.evaluate(-70, {{1, -50}}, 2700));  /* dwell over */
  }

  /* --- The cell-edge crossing. A UE walks from cell 1 to cell 2 over 20 s;
   * each cell's mean RSSI is linear in position (cell 1 falls -50 -> -80,
   * cell 2 rises -80 -> -50), with 6 dB shadowing noise per sample, one
   * uplink beacon per 102.4 ms per cell. Raw triggering (no filter, no
   * hysteresis, no ttt, no dwell) against the filtered A3. --- */
  {
    auto run = [&](bool filtered, unsigned seed) {
      std::mt19937 rng(seed);
      std::normal_distribution<double> noise(0.0, 6.0);
      L3Filter f1(filtered ? L3FilterConfig{8, 2000} : L3FilterConfig{0, 2000});
      L3Filter f2(filtered ? L3FilterConfig{8, 2000} : L3FilterConfig{0, 2000});
      A3Config c;
      if (!filtered) {
        c.offset_db = 0;
        c.hysteresis_db = 0;
        c.ttt_ms = 0;
        c.min_dwell_ms = 0;
      } else {
        c.offset_db = 3;
        c.hysteresis_db = 1;
        c.ttt_ms = 640;
        c.min_dwell_ms = 2000;
      }
      A3Trigger t(1, c);
      int handovers = 0;
      int64_t first_ms = -1;
      for (int64_t ms = 0; ms <= 20000; ms += 102) {
        const double x = static_cast<double>(ms) / 20000.0;
        f1.update(-50 - 30 * x + noise(rng), ms);
        f2.update(-80 + 30 * x + noise(rng), ms);
        const uint16_t s = t.serving(), o = s == 1 ? 2 : 1;
        const L3Filter &fs = s == 1 ? f1 : f2, &fo = s == 1 ? f2 : f1;
        if (auto tgt = t.evaluate(fs.value(), {{o, fo.value()}}, ms)) {
          t.handed_over(*tgt, ms);
          ++handovers;
          if (first_ms < 0)
            first_ms = ms;
        }
      }
      return std::make_pair(handovers, std::make_pair(first_ms, t.serving()));
    };
    /* Over many shadowing draws (one draw can be kind to the raw trigger):
     * the filtered A3 moves exactly once, after the midpoint, every time;
     * the raw trigger ping-pongs on average. */
    const int kSeeds = 100;
    int raw_total = 0, a3_single = 0;
    int64_t a3_first_min = INT64_MAX;
    for (unsigned seed = 1; seed <= kSeeds; ++seed) {
      raw_total += run(false, seed).first;
      const auto a3 = run(true, seed);
      if (a3.first == 1 && a3.second.second == 2)
        ++a3_single;
      a3_first_min = std::min(a3_first_min, a3.second.first);
    }
    std::printf("cell_m6: edge crossing (6 dB shadowing, %d draws): raw-RSSI trigger %.1f "
                "handovers on average; filtered A3 exactly one in %d/%d (earliest at %.1f s)\n",
                kSeeds, static_cast<double>(raw_total) / kSeeds, a3_single, kSeeds,
                a3_first_min / 1000.0);
    CHECK(raw_total >= 3 * kSeeds); /* raw RSSI ping-pongs */
    CHECK(a3_single == kSeeds);     /* the filtered A3 moves once ... */
    CHECK(a3_first_min > 10000);    /* ... and only after the midpoint */
  }

  /* --- DAPS dedup: duplicates over two paths, reordering, the window, the
   * 2^32 wrap. --- */
  {
    SeqDedup d;
    CHECK(d.accept(100));
    CHECK(!d.accept(100));          /* the same frame via the other cell */
    CHECK(d.accept(102));
    CHECK(d.accept(101));           /* late, inside the window */
    CHECK(!d.accept(101));
    CHECK(d.accept(100 + 300));     /* a jump past the window */
    CHECK(!d.accept(102));          /* now older than the window: dropped */
    CHECK(d.accept(400 - 255));     /* the oldest still inside */
    CHECK(!d.accept(400 - 256));    /* just outside */
    SeqDedup w;
    CHECK(w.accept(0xfffffffeu));
    CHECK(w.accept(0xffffffffu));
    CHECK(w.accept(0u));            /* wraps forward */
    CHECK(w.accept(1u));
    CHECK(!w.accept(0xffffffffu));  /* before the wrap, already seen */
    CHECK(w.top() == 1u);
    /* Two paths, each losing a different 30 %: the union arrives once. */
    std::mt19937 rng(7);
    SeqDedup u;
    int delivered = 0, dups = 0, lost_both = 0;
    for (uint32_t s = 0; s < 5000; ++s) {
      const bool a = rng() % 10 >= 3, b = rng() % 10 >= 3;
      if (!a && !b)
        ++lost_both;
      int got = 0;
      if (a)
        got += u.accept(s);
      if (b)
        got += u.accept(s);
      delivered += got;
      dups += (a && b) ? 1 : 0;
      CHECK(got <= 1);
    }
    CHECK(delivered == 5000 - lost_both);
    std::printf("cell_m6: DAPS dedup: 5000 frames, two 30%%-lossy paths: %d delivered once "
                "(%d duplicates dropped, %d lost on both)\n",
                delivered, dups, lost_both);
  }

  /* --- The DU's producer: a MeasTable fed per-frame quality, its snapshot
   * into a report (own UEs -> meas in quarter dB, others -> heard), and
   * staleness dropping a link that went quiet. --- */
  {
    MeasTable t({4, 2000});
    for (int i = 0; i < 20; ++i) {
      t.update(2, 1, -55, 30, i * 100);  /* cell 2 = this DU: its UE 1 */
      t.update(1, 1, -72, 12, i * 100);  /* cell 1's UE 1, heard here */
    }
    t.update(3, 4, -90, 2, 0);           /* heard once, long ago */
    f1::Report r;
    r.du = 2;
    f1::fill_report_measurements(r, 2, t.snapshot(1900));
    CHECK(r.meas.size() == 1 && r.meas[0].ue == 1 && r.meas[0].rssi_qdbm == -220 &&
          r.meas[0].snr_qdb == 120 && r.meas[0].age_ms == 0);
    CHECK(r.heard.size() == 2); /* 3/4 still valid at 1900 ms */
    for (const auto &h : r.heard) /* the sample ages travel with the values */
      CHECK(h.cell == 3 ? h.age_ms == 1900 : h.age_ms == 0);
    f1::fill_report_measurements(r, 2, t.snapshot(2100));
    CHECK(r.heard.size() == 1 && r.heard[0].cell == 1 && r.heard[0].rssi_dbm == -72 &&
          r.heard[0].snr_db == 12);
    const auto w = f1::encode_report(r, {});
    auto d = f1::decode_report(w.data(), w.size(), {});
    CHECK(d && d->meas == r.meas && d->heard == r.heard);
  }

  /* --- F1 v2: the measurement fields round-trip. --- */
  {
    f1::Report r;
    r.du = 2;
    r.heard = {{1, 1, -71, 18, 350}};
    r.meas = {{1, -52 * 4 - 1, 33 * 4 + 3, 0}, {2, -90 * 4, -2 * 4, 65535}};
    const auto w = f1::encode_report(r, {});
    auto d = f1::decode_report(w.data(), w.size(), {});
    CHECK(d && *d == r);
    CHECK(w[2] == f1::kVersion); /* the current version on the wire */
    auto old = w;
    old[2] = static_cast<uint8_t>(f1::kVersion - 1); /* an older sender is refused */
    CHECK(!f1::decode_report(old.data(), old.size(), {}));
  }

  /* --- TSF continuity: the delta from (phase, net_off) is exact at every
   * instant, and both grids sit on network superframe boundaries. --- */
  {
    const int64_t P = 102400;
    const int64_t ps = 0, os = -17518355000ll, pt = 13450, ot = -17518354000ll + 7;
    auto tsf = [&](int64_t t, int64_t ph, int64_t off) { return t - ph - off * P; };
    const int64_t d = ho_tsf_delta_us(ps, os, pt, ot, P);
    for (int64_t t : {int64_t{17518356123ll} * P + 777, int64_t{17518356999ll} * P + 51200,
                      int64_t{17518357000ll} * P})
      CHECK(tsf(t, pt, ot) - tsf(t, ps, os) == d);
    const int64_t N = 17518356500ll; /* network superframe N: each DU's TBTT */
    CHECK(tsf(ps + N * P, ps, os) % P == 0 && tsf(pt + N * P, pt, ot) % P == 0);
    CHECK(tsf(ps + N * P, ps, os) / P == N - os && tsf(pt + N * P, pt, ot) / P == N - ot);
  }

  /* --- F1 Handover + Ack kind. --- */
  {
    const Mac k1 = fake_mac(1), k2 = fake_mac(2);
    HoCommand h{1793880000u, 1, 1, 2, 3, 157, 2, 17518356600ull, -1234567890123ll, 99};
    const auto w = f1::encode_handover(2, h, k1);
    CHECK(f1::peek_type(w.data(), w.size()) == f1::Type::Handover);
    auto d = f1::decode_handover(w.data(), w.size(), k1);
    CHECK(d && d->first == 2 && d->second == h);
    CHECK(!f1::decode_handover(w.data(), w.size(), k2));
    CHECK(!f1::decode_schedule(w.data(), w.size(), k1));
    for (size_t cut = 0; cut < w.size(); ++cut)
      CHECK(!f1::decode_handover(w.data(), cut, k1));
    f1::Ack a{2, f1::AckKind::Handover, h.id, f1::AckStatus::Staged, 17518356590ull};
    const auto wa = f1::encode_ack(a, k1);
    auto da = f1::decode_ack(wa.data(), wa.size(), k1);
    CHECK(da && *da == a);
  }

  /* --- The beacon IE: in an MPDU, CRC, tag, the newest id for this UE. --- */
  {
    const Mac k = fake_mac(7), other = fake_mac(8);
    HoIe a{100, 1, 1, 2, 3, 3, 7, 157, 4200, 13450 - 1000 * 102400ll};
    HoIe newer = a;
    newer.id = 101;
    newer.target_ue = 4;
    HoIe someone_else = a;
    someone_else.ue = 2;
    someone_else.id = 500;
    std::vector<uint8_t> mpdu(36, 0); /* beacon header + fixed fields */
    mpdu[0] = 0x80;
    for (const HoIe &x : {a, newer, someone_else}) {
      auto ie = encode_ho_ie(x, k);
      CHECK(ie.size() == 2 + 3 + 34 + 2 + kHoTagLen);
      mpdu.insert(mpdu.end(), ie.begin(), ie.end());
    }
    auto got = find_ho_ie(mpdu.data(), mpdu.size(), 1, 1, k);
    CHECK(got && *got == newer); /* the newest id addressed to (cell 1, UE 1) */
    CHECK(got->channel == 157);  /* the target's channel travels with it */
    CHECK(!find_ho_ie(mpdu.data(), mpdu.size(), 1, 1, other)); /* wrong key: inert */
    CHECK(!find_ho_ie(mpdu.data(), mpdu.size(), 2, 1, k));     /* another cell's UE */
    auto torn = mpdu;
    torn[36 + 5 + 10] ^= 0x40; /* inside the first IE's body */
    auto t = find_ho_ie(torn.data(), torn.size(), 1, 1, k);
    CHECK(t && *t == newer); /* the torn one is dropped, the intact newer one stays */
    auto plain = encode_ho_ie(a, {});
    std::vector<uint8_t> m2(36, 0);
    m2[0] = 0x80;
    m2.insert(m2.end(), plain.begin(), plain.end());
    CHECK(find_ho_ie(m2.data(), m2.size(), 1, 1, {}));  /* unkeyed cell: tag ignored */
    CHECK(!find_ho_ie(m2.data(), m2.size(), 1, 1, k));  /* keyed UE: zero tag refused */
    auto not_bcn = mpdu;
    not_bcn[0] = 0x08; /* a data frame carrying the same bytes */
    CHECK(!find_ho_ie(not_bcn.data(), not_bcn.size(), 1, 1, k));
  }

  /* --- Replay across beacons: the UE acts once, on a fresh command. --- */
  {
    HoReceiver rx;
    HoIe h{100, 1, 1, 2, 3, 3, 7, 0, 4200, 0};
    CHECK(rx.accept(h, 4190));   /* new id, instant ahead */
    CHECK(!rx.accept(h, 4191));  /* the same IE in the next beacons (or replayed) */
    HoIe older = h;
    older.id = 99;
    older.effective_sf = 9000;
    CHECK(!rx.accept(older, 4192)); /* superseded id, even with a later instant */
    HoIe spent = h;
    spent.id = 101;
    spent.effective_sf = 4195;
    CHECK(!rx.accept(spent, 4195)); /* its instant has passed */
    HoIe next = h;
    next.id = 102;
    next.effective_sf = 4300;
    CHECK(rx.accept(next, 4200) && rx.last_id() == 102);
  }

  /* --- DAPS: the CU's uplink streams. A UE crossing from cell 1 (id 1) to
   * cell 2 (id 2): both DUs forward what they hear; cell 2 aliases its id
   * onto the stream; at 60 % of the run the SOURCE path dies. Every uplink
   * either DU heard is delivered exactly once, and after the kill only the
   * target carries it. --- */
  {
    std::mt19937 rng(11);
    UlStreams s;
    const UlStreams::Id src{1, 1}, tgt{2, 2};
    s.alias(tgt, src);
    int heard_any = 0, delivered = 0, after_kill_via_tgt = 0, after_kill = 0;
    for (uint32_t seq = 1000; seq < 2000; ++seq) {
      const bool switched = seq >= 1500;           /* the UE's identity flips at 1500 */
      const bool src_alive = seq < 1600;           /* the source DU dies at 1600 */
      const UlStreams::Id who = switched ? tgt : src;
      const bool a = src_alive && rng() % 10 >= 2; /* each path loses 20 % */
      const bool b = rng() % 10 >= 2;
      heard_any += (a || b) ? 1 : 0;
      int got = 0;
      if (a)
        got += s.deliver(who, seq, 1).fresh;
      if (b)
        got += s.deliver(who, seq, 2).fresh;
      CHECK(got <= 1);
      delivered += got;
      if (!src_alive && got) {
        ++after_kill;
        after_kill_via_tgt += b ? 1 : 0;
      }
    }
    CHECK(delivered == heard_any);
    CHECK(after_kill > 0 && after_kill == after_kill_via_tgt);
    const auto *st = s.stats(src);
    CHECK(st && st == s.stats(tgt)); /* one stream under both identities */
    CHECK(st->delivered == static_cast<uint64_t>(delivered));
    CHECK(st->first_by_du.at(1) + st->first_by_du.at(2) == st->delivered);
    std::printf("cell_m6: DAPS streams: %d uplinks heard, %d delivered once (%llu duplicates), "
                "%d after the source died, all via the target\n",
                heard_any, delivered, (unsigned long long)st->duplicates, after_kill);
    const auto sid = s.stream_id(tgt);
    CHECK(sid && s.stats_of_stream(*sid) == st);
    s.forget(src);
    CHECK(!s.stats(src) && s.stats(tgt)); /* the stream lives on via the target */
    s.forget(tgt);
    CHECK(!s.stats(tgt) && s.streams() == 0 && s.identities() == 0); /* the last one: gone */
    /* forget_stream drops every identity of it. */
    UlStreams f;
    f.deliver({5, 1}, 1, 5);
    f.alias({6, 2}, {5, 1});
    f.forget_stream(*f.stream_id({5, 1}));
    CHECK(f.identities() == 0 && f.streams() == 0);
    /* A reused identity after forget starts a fresh stream (no inherited window). */
    f.deliver({5, 1}, 900000, 5);
    CHECK(f.stats({5, 1})->delivered == 1);
    /* The common seq: the reception instant in network superframes — two
     * DUs a few us apart on the fabric agree (away from a boundary). */
    const int64_t P = 102400;
    CHECK(ul_seq_from_network_time(1000 * P + 50000 + 3, P) ==
          ul_seq_from_network_time(1000 * P + 50000 - 4, P));
    CHECK(ul_seq_from_network_time(1001 * P + 7, P) == ul_seq_from_network_time(1000 * P + 7, P) + 1);
    /* F1 v5 Uplink round trip. */
    const Mac k = fake_mac(3);
    f1::UplinkFwd u{2, 1, 1, 123456, -61};
    const auto w = f1::encode_uplink(u, k);
    CHECK(f1::peek_type(w.data(), w.size()) == f1::Type::Uplink);
    auto du = f1::decode_uplink(w.data(), w.size(), k);
    CHECK(du && *du == u);
    CHECK(!f1::decode_uplink(w.data(), w.size(), fake_mac(4)));
    for (size_t cut = 0; cut < w.size(); ++cut)
      CHECK(!f1::decode_uplink(w.data(), cut, k));
  }

  /* --- Measurement gaps (MeasGap.h): the neighbour offset, the NL and MR
   * IEs, and where a gap fits. --- */
  {
    const int64_t P = 102400;
    CHECK(neighbour_offset_us(60000, 0, P) == 60000);
    CHECK(neighbour_offset_us(0, 13450, P) == P - 13450); /* the other way round wraps */
    CHECK(neighbour_offset_us(13450 + 3 * P, 13450, P) == 0);
    const Mac k = fake_mac(21), other = fake_mac(22);
    std::vector<Neighbour> nl = {{2, 157, 60000}, {3, 36, 81234}};
    std::vector<uint8_t> bcn(36, 0);
    bcn[0] = 0x80;
    const auto ie = encode_nl_ie(1, 7, nl, k);
    CHECK(!ie.empty());
    bcn.insert(bcn.end(), ie.begin(), ie.end());
    auto got = find_nl_ie(bcn.data(), bcn.size(), 1, k);
    CHECK(got && got->cells == nl && got->gen == 7); /* the generation travels, tagged */
    /* An empty list is a valid NL (a list going away; the beacon's provenance). */
    std::vector<uint8_t> eb(36, 0);
    eb[0] = 0x80;
    const auto eie = encode_nl_ie(1, 8, {}, k);
    eb.insert(eb.end(), eie.begin(), eie.end());
    auto eg = find_nl_ie(eb.data(), eb.size(), 1, k);
    CHECK(eg && eg->cells.empty() && eg->gen == 8);
    CHECK(!find_nl_ie(bcn.data(), bcn.size(), 2, k));     /* another cell's list */
    CHECK(!find_nl_ie(bcn.data(), bcn.size(), 1, other)); /* a forged list is inert */
    auto torn = bcn;
    torn[36 + 5 + 7] ^= 1;
    CHECK(!find_nl_ie(torn.data(), torn.size(), 1, k));
    auto data = bcn;
    data[0] = 0x08;
    CHECK(!find_nl_ie(data.data(), data.size(), 1, k)); /* beacons only */
    std::vector<MeasEntryUe> mr = {{1, -52, 38, 40}, {2, -49, 41, 2900}};
    std::vector<uint8_t> ub(36, 0);
    ub[0] = 0x80;
    const auto mie = encode_mr_ie(1, 1, 12345, mr, k);
    ub.insert(ub.end(), mie.begin(), mie.end());
    auto gm = find_mr_ie(ub.data(), ub.size(), 1, 1, k);
    CHECK(gm && gm->cells == mr && gm->gen == 12345); /* sample ages + generation travel */
    CHECK(!find_mr_ie(ub.data(), ub.size(), 1, 2, k));     /* another UE's */
    CHECK(!find_mr_ie(ub.data(), ub.size(), 1, 1, other)); /* a forged report is inert */
    CHECK(encode_nl_ie(1, 1, std::vector<Neighbour>(kMaxNeighbours + 1), k).empty());
    /* The rig grid (beacon 1500, DL 4000 x3 from 1500, turnaround 1000, UL
     * 2000 x3 from 14500), a UE in position 2: busy = the serving beacon, its
     * DL [5500, 9500), its UL [16500, 18500) with the uplink-beacon margin. */
    const std::vector<GridSpan> busy = {{0, 1500}, {5500, 4000}, {16500, 2000}};
    const int64_t retune = 24000, guard = 2000, beacon = 500;
    auto g = gap_window(60000, beacon, retune, guard, P, busy);
    CHECK(g && g->start == 60000 - 26000 && g->len == 2 * 26000 + 500);
    CHECK(!gap_window(13450, beacon, retune, guard, P, busy)); /* over the UE's own slots */
    CHECK(!gap_window(100000, beacon, retune, guard, P, busy)); /* wraps onto the serving beacon */
    CHECK(!gap_window(80000, beacon, retune, guard, P, busy));  /* [54000, 106500) wraps onto the beacon */
    CHECK(gap_window(70000, beacon, retune, guard, P, busy));   /* [44000, 96500): clear */
    /* F1 v7: Neighbours, and the UEs' own measurements in a Report. */
    f1::Neighbours nb{1, 777, {{2, 157, 60000}, {3, 36, 13450}}};
    const auto wn = f1::encode_neighbours(nb, k);
    CHECK(f1::peek_type(wn.data(), wn.size()) == f1::Type::Neighbours);
    auto dn = f1::decode_neighbours(wn.data(), wn.size(), k);
    CHECK(dn && *dn == nb);
    for (size_t cut = 0; cut < wn.size(); ++cut)
      CHECK(!f1::decode_neighbours(wn.data(), cut, k));
    f1::Report rr;
    rr.du = 1;
    rr.ue_meas = {{1, 1, -52 * 4, 38 * 4, 120}, {1, 2, -49 * 4 - 1, 41 * 4, 900}};
    /* A full cell's worth of UE measurements fits the dedicated cap. */
    f1::Report full;
    full.du = 1;
    full.ue_meas.resize(f1::kMaxUeMeas);
    const auto wf = f1::encode_report(full, k);
    CHECK(!wf.empty());
    auto df = f1::decode_report(wf.data(), wf.size(), k);
    CHECK(df && df->ue_meas.size() == f1::kMaxUeMeas);
    full.ue_meas.resize(f1::kMaxUeMeas + 1);
    CHECK(f1::encode_report(full, k).empty());
    const auto wr = f1::encode_report(rr, k);
    auto drr = f1::decode_report(wr.data(), wr.size(), k);
    CHECK(drr && drr->ue_meas == rr.ue_meas);
  }

  /* --- Phase planning (PhasePlan.h) on the rig grid: beacon 1500, three
   * positions (DL 4000, turnaround 1000, UL 2000), gaps of 26 + 2 ms. --- */
  {
    auto g = build_fixed_grid(100, 0, 1500, {1, 2, 3}, 4000, 1000, 2000);
    CHECK(g);
    CoordDu d;
    coord_layout(*g, d);
    const PhaseCell a = phase_cell(1, 149, 0, 102400, d.fixed, d.positions);
    CHECK(a.beacon_us == 1500 && a.span_us == 20500 && a.positions.size() == 3);
    PhaseParams pp;
    pp.retune_us = 26000;
    pp.guard_us = 2000;
    /* Another channel: every position of each cell can measure the other's
     * beacon only for a phase in [48500, 53900]; the plan takes its middle. */
    PhaseCell b = phase_cell(2, 157, 0, 102400, d.fixed, d.positions);
    auto p = plan_phase(b, {a}, pp);
    CHECK(p.score == 2 && p.max_score == 2 && p.phase_us == 51200);
    CHECK(p.pairs.size() == 1 && p.pairs[0].first == 1 && p.pairs[0].second == 2);
    CHECK(mg_detail::pair_score(b, 48500, a, pp) == 2 && mg_detail::pair_score(b, 53900, a, pp) == 2);
    CHECK(mg_detail::pair_score(b, 48400, a, pp) < 2 && mg_detail::pair_score(b, 54000, a, pp) < 2);
    CHECK(mg_detail::pair_score(b, 60000, a, pp) < 2); /* the old hand-picked phase: one way only */
    /* The 30 + 3 ms bound leaves no phase measurable both ways. */
    PhaseParams wide = pp;
    wide.retune_us = 30000;
    wide.guard_us = 3000;
    CHECK(plan_phase(b, {a}, wide).score < 2);
    /* The same channel: disjoint active spans. */
    b.channel = 149;
    p = plan_phase(b, {a}, pp);
    CHECK(p.score == 2 && !mg_detail::arcs_overlap(p.phase_us, b.span_us, 0, a.span_us, 102400));
    /* A third cell on a third channel cannot be measured both ways by both
     * (a physical limit at this bound): the best is partial, not a failure. */
    PhaseCell c2 = b;
    c2.channel = 157;
    c2.phase_us = 51200;
    PhaseCell j = phase_cell(3, 36, 0, 102400, d.fixed, d.positions);
    p = plan_phase(j, {a, c2}, pp);
    CHECK(p.max_score == 4 && p.score >= 1 && p.score < 4);
    /* Across bands: no gap credit (the UEs never gap there). */
    PhaseCell lo = phase_cell(4, 6, 0, 102400, d.fixed, d.positions);
    CHECK(mg_detail::pair_score(lo, 51200, a, pp) == 0);
    CHECK(gap_cross_band(6, 149) && !gap_cross_band(36, 149));
    /* Another period: left out, not mis-scored on this one's modulus. */
    PhaseCell longer = a;
    longer.du = 9;
    longer.period_us = 204800;
    p = plan_phase(b, {a, longer}, pp);
    CHECK(p.max_score == 2 && p.other_period.size() == 1 && p.other_period[0] == 9);
    /* A long period: the candidate grid is capped. */
    PhaseParams lp = pp;
    lp.period_us = 1000 * 1024;
    lp.step_us = 1;
    CHECK(plan_phase(j, {}, lp).phase_us < lp.period_us);
    /* No other cell: any phase (the middle of the circle). */
    CHECK(plan_phase(j, {}, pp).max_score == 0);
    /* F1 v8: Phase. */
    const Mac k = fake_mac(7);
    f1::PhaseAssign pa{2, 51200, 1234567};
    const auto wp = f1::encode_phase(pa, k);
    CHECK(f1::peek_type(wp.data(), wp.size()) == f1::Type::Phase);
    auto dp = f1::decode_phase(wp.data(), wp.size(), k);
    CHECK(dp && *dp == pa);
    CHECK(!f1::decode_phase(wp.data(), wp.size(), fake_mac(8))); /* forged */
    for (size_t cut = 0; cut < wp.size(); ++cut)
      CHECK(!f1::decode_phase(wp.data(), cut, k));
    /* The discipline takes an assigned phase before it acquires. */
    TsfDisciplineConfig dc;
    dc.phase_us = 0;
    TsfDiscipline disc(dc);
    disc.set_phase(51200);
    CHECK(disc.phase() == 51200);
  }

  /* --- Re-attachment after a DU restart (GrantMap.h offer_after_restart). --- */
  {
    GrantMapReceiver rx(1);
    GrantMap m;
    m.cell_id = 1;
    m.epoch = 13;
    m.effective_sf = 400;
    m.grants = {{}};
    m.grants[0].ue = 1;
    CHECK(rx.offer(m) && rx.newest_epoch() == 13);
    /* The restarted DU: epoch 2, due at 97 while its beacons read sf 94. */
    GrantMap n = m;
    n.epoch = 2;
    n.effective_sf = 97;
    CHECK(!rx.offer(n));                              /* an old epoch: refused */
    CHECK(!rx.offer_after_restart(n, 400));           /* not near the beacon's count */
    GrantMap stale = m;
    stale.epoch = 12;
    stale.effective_sf = 390;
    CHECK(!rx.offer_after_restart(stale, 94));        /* a map from before the restart */
    GrantMap other = n;
    other.cell_id = 2;
    CHECK(!rx.offer_after_restart(other, 94));        /* another cell's */
    CHECK(rx.offer_after_restart(n, 94));             /* starts over */
    CHECK(rx.newest_epoch() == 2 && rx.current(97) && rx.current(97)->epoch == 2);
    GrantMap n2 = n;
    n2.epoch = 3;
    n2.effective_sf = 115;
    CHECK(rx.offer(n2) && rx.current(115)->epoch == 3); /* and runs on normally */
    CHECK(!rx.offer_after_restart(m, 115));            /* never back to a higher epoch this way */
    CHECK(!rx.offer_after_restart(n2, 115));           /* the live map repeated: no restart */
  }

  if (failures == 0)
    std::printf("cell_m6_selftest: all checks passed\n");
  return failures == 0 ? 0 : 1;
}
