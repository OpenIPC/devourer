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
#include "cell/Keys.h"
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
    /* A weaker neighbour whose timer started first does not win over a
     * stronger one still in its time-to-trigger (the three-cell rig: cell 2
     * fired 1 s before cell 3, 19 dB stronger, had held for ttt). */
    A3Trigger x(1, c);
    CHECK(!x.evaluate(-60, {{2, -51}}, 0));                       /* only 2 qualifies */
    CHECK(!x.evaluate(-60, {{2, -51}, {3, -32}}, 500));           /* 3 enters, stronger */
    CHECK(!x.evaluate(-60, {{2, -51}, {3, -32}}, 700));           /* 2 held ttt; 3 has not */
    auto x3 = x.evaluate(-60, {{2, -51}, {3, -32}}, 1140);        /* 3 held ttt */
    CHECK(x3 && *x3 == 3);
    /* ... and once the stronger one leaves, the weaker can fire again. */
    A3Trigger y(1, c);
    y.evaluate(-60, {{2, -51}, {3, -32}}, 0);
    CHECK(!y.evaluate(-60, {{2, -51}, {3, std::nullopt}}, 300)); /* 3 stale: dropped */
    auto y2 = y.evaluate(-60, {{2, -51}, {3, std::nullopt}}, 700);
    CHECK(y2 && *y2 == 2);
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

  /* --- F1 v16: EVM, filtered, from the frames that carry one; "none" when
   * none did, in both directions of the report. --- */
  {
    MeasTable t({4, 2000});
    for (int i = 0; i < 20; ++i) {
      t.update(2, 1, -55, 30, i * 100, -31.0);            /* own UE: EVM -31 dB */
      t.update(1, 1, -72, 12, i * 100, i % 2 ? std::optional<double>(-18.0) : std::nullopt);
      t.update(3, 2, -80, 8, i * 100);                    /* heard, no EVM ever */
    }
    f1::Report r;
    r.du = 2;
    f1::fill_report_measurements(r, 2, t.snapshot(1900));
    CHECK(r.meas.size() == 1 && r.meas[0].evm_qdb == -124);
    const f1::HeardUe *h1 = nullptr, *h3 = nullptr;
    for (const auto &h : r.heard)
      (h.cell == 1 ? h1 : h3) = &h;
    CHECK(h1 && h1->evm_db == -18 && h3 && h3->evm_db == f1::kNoEvm8);
    const auto w = f1::encode_report(r, {});
    auto d = f1::decode_report(w.data(), w.size(), {});
    CHECK(d && d->meas == r.meas && d->heard == r.heard);
    /* a real EVM never encodes as "none": heard (8-bit dB) clamps at -127 */
    MeasTable u({4, 2000});
    u.update(5, 1, -55, 30, 0, -200.0);
    f1::fill_report_measurements(r, 2, u.snapshot(0));
    CHECK(r.heard.size() == 1 && r.heard[0].evm_db == f1::kNoEvm8 + 1);
  }

  /* --- F1 v17: the DU's TA per own UE round-trips in its report. --- */
  {
    f1::Report r;
    r.du = 1;
    r.meas = {{1, -200, 120, 0, -124, -431}, {2, -220, 100, 5, f1::kNoEvmQ, 37}};
    const auto w = f1::encode_report(r, {});
    auto d = f1::decode_report(w.data(), w.size(), {});
    CHECK(d && d->meas == r.meas && d->meas[0].ta_us == -431 && d->meas[1].ta_us == 37);
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
      CHECK(ie.size() == 2 + 3 + 58 + 2 + kHoTagLen);
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
    /* DAPS across channels: a copy heard later in the superframe (a visit to
     * the target's channel) maps to the UE's own uplink slot; one at the slot
     * (incl. a timing advance) maps there too; the next slot is the next seq. */
    {
      const int64_t ph = 51200, ul = 16500; /* slot instant at 67700 in each superframe */
      const int64_t S = 1000 * P;
      const uint32_t at_slot = ul_seq_for_copy(S + 67700, ph, ul, P);
      CHECK(at_slot == ul_seq_from_network_time(S + 67700, P));
      CHECK(ul_seq_for_copy(S + 67700 - 400, ph, ul, P) == at_slot);       /* TA: heard 0.4 ms early */
      CHECK(ul_seq_for_copy(S + 67700 + 40000, ph, ul, P) == at_slot);     /* a visit 40 ms later: next */
      CHECK(ul_seq_from_network_time(S + 67700 + 40000, P) == at_slot + 1); /* ... the raw seq would differ */
      CHECK(ul_seq_for_copy(S + 67700 + P, ph, ul, P) == at_slot + 1);     /* the next slot */
      CHECK(ul_seq_for_copy(S + 67700 + P - 6000, ph, ul, P) == at_slot);  /* still before it */
      /* The slot 200 us after a network-superframe boundary: the serving
       * copy heard 400 us early (timing advance) is in the PREVIOUS raw
       * superframe, a visit 30 ms later in the slot's — mapping BOTH (the CU
       * maps every copy) gives one seq. */
      CHECK(ul_seq_for_copy(S - 200, 0, 200, P) == ul_seq_for_copy(S + 30200, 0, 200, P));
      CHECK(ul_seq_for_copy(S - 200, 0, 200, P) == ul_seq_from_network_time(S, P));
      CHECK(ul_seq_from_network_time(S - 200, P) + 1 == ul_seq_from_network_time(S, P)); /* raw differ */
      /* phase 0: slot at 16500 of the same network superframe */
      CHECK(ul_seq_for_copy(S + 16500 + 50000, 0, ul, P) == ul_seq_from_network_time(S, P));
    }
    /* F1 v5 Uplink round trip. */
    const Mac k = fake_mac(3);
    f1::UplinkFwd u{2, 1, 1, 123456, 17521968566LL * 102400 + 51200, -61};
    const auto w = f1::encode_uplink(u, k);
    CHECK(f1::peek_type(w.data(), w.size()) == f1::Type::Uplink);
    auto du = f1::decode_uplink(w.data(), w.size(), k);
    CHECK(du && *du == u);
    CHECK(!f1::decode_uplink(w.data(), w.size(), fake_mac(4)));
    for (size_t cut = 0; cut < w.size(); ++cut)
      CHECK(!f1::decode_uplink(w.data(), cut, k));
  }

  /* --- DL duplication: F1 v12 Downlink, and the UE's DL stream. --- */
  {
    const Mac k = fake_mac(3);
    f1::DownlinkPdu pdu{2, 7, 0x0102030405060708ull, 4000000000u};
    const auto w = f1::encode_downlink(pdu, k);
    CHECK(f1::peek_type(w.data(), w.size()) == f1::Type::Downlink);
    auto dd = f1::decode_downlink(w.data(), w.size(), k);
    CHECK(dd && *dd == pdu);
    CHECK(!f1::decode_downlink(w.data(), w.size(), fake_mac(4)));
    for (size_t cut = 0; cut < w.size(); ++cut)
      CHECK(!f1::decode_downlink(w.data(), cut, k));

    const uint64_t E = 1000;
    DlStreamRx rx;
    CHECK(rx.accept(E, 10) && rx.accept(E, 11));
    CHECK(!rx.accept(E, 11)); /* the target's copy of what the source delivered */
    CHECK(rx.accept(E, 14));  /* 12, 13 missing ... */
    CHECK(rx.missing_total() == 2 && rx.take_gaps().empty());
    CHECK(rx.accept(E, 12));  /* ... 12 late: recovered */
    CHECK(rx.missing_total() == 1);
    for (uint32_t s = 15; s <= 13 + DlStreamRx::kReorder; ++s)
      CHECK(rx.accept(E, s));
    CHECK(rx.take_gaps().empty()); /* 13 is exactly kReorder behind: not yet */
    CHECK(rx.accept(E, 14 + DlStreamRx::kReorder));
    auto g = rx.take_gaps();
    CHECK(g.size() == 1 && g[0].after_sn == 12 && g[0].missing == 1);
    CHECK(rx.missing_confirmed() == 1 && rx.missing_total() == 1);
    CHECK(rx.accept(E, 13));  /* a very late copy is still delivered once (counted lost) */
    CHECK(!rx.accept(E, 13));
    CHECK(rx.duplicates() == 2);
    CHECK(rx.delivered() == 4 + (DlStreamRx::kReorder - 1) + 1 + 1);
    /* An older epoch: a stale stream, ignored. */
    CHECK(!rx.accept(E - 1, 100) && rx.stale() == 1);
    /* A newer epoch starts over at sn 1 (a re-created stream, a restarted
     * CU): nothing of it is a duplicate of the old one. */
    CHECK(rx.accept(E + 1, 1) && rx.accept(E + 1, 2));
    CHECK(rx.missing_total() == 1); /* only the old stream's 13 */

    /* A jump beyond kMaxJump starts the count over: no gap of hundreds; a
     * hole still pending before it is lost, attributed to where it was. */
    DlStreamRx r2;
    CHECK(r2.accept(E, 100) && r2.accept(E, 102));
    CHECK(r2.accept(E, 102 + DlStreamRx::kMaxJump + 1));
    g = r2.take_gaps();
    CHECK(g.size() == 1 && g[0].after_sn == 100 && g[0].missing == 1);
    CHECK(r2.missing_total() == 1);
    /* A jump of exactly kMaxJump is tracked, and every hole of it is still
     * inside the dedup window: a late copy at the oldest recovers it. */
    DlStreamRx r4;
    CHECK(r4.accept(E, 0) && r4.accept(E, DlStreamRx::kMaxJump));
    CHECK(r4.take_gaps().empty() && r4.missing_total() == DlStreamRx::kMaxJump - 1); /* one run, pending */
    CHECK(r4.accept(E, 1)); /* the oldest hole: delivered (late), not a duplicate */
    CHECK(r4.duplicates() == 0 && r4.missing_total() == DlStreamRx::kMaxJump - 2);

    /* Wrap: one run across 0xffffffff -> 0, one gap event. */
    DlStreamRx r3;
    CHECK(r3.accept(E, 0xfffffffeu) && r3.accept(E, 1));
    CHECK(r3.missing_total() == 2); /* 0xffffffff, 0 */
    CHECK(r3.accept(E, 2 + DlStreamRx::kReorder));
    g = r3.take_gaps();
    CHECK(g.size() == 1 && g[0].after_sn == 0xfffffffeu && g[0].missing == 2);
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
    /* Suspend gaps: the pattern, the DU's DL skip, and which neighbours. */
    {
      int n = 0;
      for (int64_t sf = 0; sf < 200; ++sf)
        n += suspend_sf(sf, 2, 20) ? 1 : 0;
      CHECK(n == 10);                                       /* one in `every` */
      CHECK(suspend_sf(14, 2, 20) && !suspend_sf(14, 1, 20)); /* staggered by UE id */
      CHECK(suspend_dl_skip(14, 2, 20) && suspend_dl_skip(15, 2, 20) && !suspend_dl_skip(16, 2, 20));
      CHECK(!suspend_sf(14, 2, 0) && !suspend_dl_skip(15, 2, 0)); /* off */
      CHECK(suspend_sf(-6, 2, 20));                         /* negative superframes stay in step */
      const Neighbour idle_ok{2, 157, 60000}, slots{3, 157, 13450}, same{4, 149, 13450}, band{5, 6, 13450};
      CHECK(!suspend_only(idle_ok, 149, retune, guard, P, busy)); /* an idle window exists */
      CHECK(suspend_only(slots, 149, retune, guard, P, busy));    /* only over its own slots */
      CHECK(!suspend_only(same, 149, retune, guard, P, busy));    /* same channel: no gap at all */
      CHECK(!suspend_only(band, 149, retune, guard, P, busy));    /* across bands: never */
      CHECK(!needs_suspend({idle_ok, same, band}, 149, retune, guard, P, busy));
      CHECK(needs_suspend({idle_ok, slots}, 149, retune, guard, P, busy));
      /* A retune too long for any window in P: no suspend gap either. */
      CHECK(!suspend_only(slots, 149, 60000, guard, P, busy));
    }
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
    /* With suspend gaps every direction of every pair is covered (score > 0
     * per direction), and the plan still prefers the idle windows: the
     * idle-only optimum is kept where it exists. */
    PhaseParams sp = pp;
    sp.suspend_weight = 0.25;
    CHECK(plan_phase(phase_cell(2, 157, 0, 102400, d.fixed, d.positions), {a}, sp).phase_us == 51200);
    auto p3 = plan_phase(j, {a, c2}, sp);
    for (const auto &pr : p3.pairs) {
      const PhaseCell &e = pr.first == 1 ? a : c2;
      CHECK(mg_detail::measurable(e, neighbour_offset_us(p3.phase_us, e.phase_us, 102400), sp) > 0);
      CHECK(mg_detail::measurable(j, neighbour_offset_us(e.phase_us, p3.phase_us, 102400), sp) > 0);
    }
    /* ... and the pair the third cell cannot reach idle is suspend-only. */
    {
      const std::vector<GridSpan> busy3 = {{0, 1500}, {1500, 4000}, {14500, 2000}};
      bool some_suspend = false;
      for (const PhaseCell *e : std::vector<const PhaseCell *>{&a, &c2}) {
        Neighbour n{e->du, e->channel, neighbour_offset_us(e->phase_us, p3.phase_us, 102400)};
        some_suspend = some_suspend || suspend_only(n, 36, 26000, 2000, 102400, busy3);
      }
      Neighbour back_a{3, 36, neighbour_offset_us(p3.phase_us, 0, 102400)};
      Neighbour back_c{3, 36, neighbour_offset_us(p3.phase_us, 51200, 102400)};
      some_suspend = some_suspend || suspend_only(back_a, 149, 26000, 2000, 102400, busy3) ||
                     suspend_only(back_c, 157, 26000, 2000, 102400, busy3);
      CHECK(some_suspend);
    }
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

  /* --- The per-cell security context (Keys.h). --- */
  {
    const Mac master = fake_mac(0xA5);
    const auto ka = area_key(master), k1 = cell_key(master, 1), k2 = cell_key(master, 2);
    const auto u11 = ue_key(master, 1, 1), u12 = ue_key(master, 1, 2);
    CHECK(ka && k1 && k2 && u11 && u12);
    CHECK(k1->size() == 40 && *k1 != *k2 && *k1 != *ka && *u11 != *u12 && *u11 != *k1);
    CHECK(*cell_key(master, 1) == *k1);                           /* deterministic */
    CHECK(*cell_key(fake_mac(0xA6), 1) != *k1);                   /* another master */
    const auto b2 = key_from_hex(*k2);
    CHECK(b2 && key_to_hex(*b2) == *k2);                          /* hex round trip */
    CHECK(!key_from_hex("zz") && !key_from_hex(std::string(40, 'g')));
    /* The context: K_2 for UE (1,1), only that UE's key unwraps it. */
    const Mac uk = fake_mac(0x11), other_ue = fake_mac(0x12);
    const auto ctx = wrap_context(uk, 77, 2, *b2);
    CHECK(ctx && !std::equal(b2->begin(), b2->end(), ctx->begin()));
    CHECK(unwrap_context(uk, 77, 2, *ctx) == b2);
    /* The key check refuses what is not this UE's for this handover. */
    CHECK(!unwrap_context(other_ue, 77, 2, *ctx));                /* another UE */
    CHECK(!unwrap_context(uk, 78, 2, *ctx));                      /* another handover id */
    CHECK(!unwrap_context(uk, 77, 3, *ctx));                      /* another target */
    auto bent = *ctx;
    bent[3] ^= 1;
    CHECK(!unwrap_context(uk, 77, 2, bent));                      /* a tampered key byte */
    bent = *ctx;
    bent[22] ^= 1;
    CHECK(!unwrap_context(uk, 77, 2, bent));                      /* a tampered check */
    CHECK(!unwrap_context(uk, 77, 2, Context{}));                 /* none */
    CHECK(*wrap_context(uk, 78, 2, *b2) != *ctx);                 /* the pad is per id */
    CHECK(!wrap_context(Mac{}, 77, 2, *b2));                      /* no UE key: no context */
    /* A joiner's key: bound to the MAC it joins with, distinct from any
     * static UE key and from another joiner's. */
    const std::array<uint8_t, 6> jm = {0x02, 0xce, 0xfe, 0x51, 0x99, 0x46};
    auto jm2 = jm;
    jm2[5] ^= 1;
    const auto kj = joiner_key(master, jm);
    CHECK(kj && kj->size() == 40 && *kj == *joiner_key(master, jm));
    CHECK(*kj != *joiner_key(master, jm2) && *kj != *u11 && *kj != *k1);
    /* The DU reports each UE's home (F1 v10): static or a joiner's MAC. */
    f1::Report jr;
    jr.du = 1;
    jr.homes = {{1, HomeId{false, 1, 1, {}}}, {2, HomeId{true, 0, 0, jm}}};
    const auto wj = f1::encode_report(jr, fake_mac(0xF1));
    auto dj = f1::decode_report(wj.data(), wj.size(), fake_mac(0xF1));
    CHECK(dj && dj->homes == jr.homes);
    CHECK(*home_key(master, jr.homes[0].home) == *u11 && *home_key(master, jr.homes[1].home) == *kj);
    jr.homes.resize(f1::kMaxUes + 1);
    CHECK(f1::encode_report(jr, fake_mac(0xF1)).empty());
    /* The handover command carries the home to the target. */
    HoCommand hc;
    hc.id = 9;
    hc.home = HomeId{true, 0, 0, jm};
    hc.ta_us = -428; /* v17: the source's TA travels with the UE */
    const auto wh = f1::encode_handover(2, hc, fake_mac(0xF1));
    auto dh = f1::decode_handover(wh.data(), wh.size(), fake_mac(0xF1));
    CHECK(dh && dh->second.home == hc.home && dh->second.ta_us == -428);
    auto badk = wh;
    badk[badk.size() - 8 - 2 - 7] = 2; /* home kind 2: unknown (before the 2-byte TA) */
    CHECK(!f1::decode_handover(badk.data(), badk.size(), Mac{}));
    /* Per-UE area keys: A_u from the UE's key, wrapped for one DU. */
    const auto au = ue_area_key(fake_mac(0x11));
    CHECK(au && *au != *ue_area_key(fake_mac(0x12)));             /* per UE */
    const auto raw = key_from_hex(*au);
    const Mac duk = fake_mac(0xD2);
    const auto w1 = wrap_area_key(duk, 1, 1, 100, *raw);
    CHECK(w1 && *w1 != *raw && wrap_area_key(duk, 1, 1, 100, *w1) == raw); /* its own inverse */
    CHECK(*wrap_area_key(duk, 1, 1, 101, *raw) != *w1);           /* a fresh nonce, a fresh pad */
    CHECK(wrap_area_key(fake_mac(0xD3), 1, 1, 100, *w1) != raw);  /* another DU's key */
    f1::AreaKeys ak;
    ak.du = 2;
    ak.seq = 5;
    ak.keys = {{1, 1, 100, *w1}};
    const auto wak = f1::encode_area_keys(ak, fake_mac(0xF1));
    CHECK(f1::peek_type(wak.data(), wak.size()) == f1::Type::AreaKeys);
    auto dak = f1::decode_area_keys(wak.data(), wak.size(), fake_mac(0xF1));
    CHECK(dak && *dak == ak);
    CHECK(!f1::decode_area_keys(wak.data(), wak.size(), fake_mac(0xF2)));
    ak.keys.resize(f1::kMaxAreaKeys);
    const auto wfull = f1::encode_area_keys(ak, fake_mac(0xF1));
    CHECK(!wfull.empty() && wfull.size() <= f1::kMaxDatagram);
    ak.keys.resize(f1::kMaxAreaKeys + 1);
    CHECK(f1::encode_area_keys(ak, fake_mac(0xF1)).empty());
    /* The report flags the DU's per-cell keys (F1 v11): a CU without the
     * master refuses such a DU. Unknown flag bits are refused. */
    f1::Report pf;
    pf.du = 1;
    pf.coordinated = true;
    pf.per_cell_keys = true;
    const auto wpf = f1::encode_report(pf, Mac{});
    auto dpf = f1::decode_report(wpf.data(), wpf.size(), Mac{});
    CHECK(dpf && dpf->per_cell_keys && dpf->coordinated);
    auto bpf = wpf;
    bpf[4 + 2 + 1 + 1] = 4; /* du(2) tier(1) sync_state(1), then the flags */
    CHECK(!f1::decode_report(bpf.data(), bpf.size(), Mac{}));
    /* A report at EVERY list bound still fits the F1 datagram both sides
     * receive into. */
    f1::Report big;
    big.fixed.resize(f1::kMaxFixed);
    big.positions.resize(f1::kMaxPositions);
    big.ues.resize(f1::kMaxUes);
    big.heard.resize(f1::kMaxHeard);
    big.meas.resize(f1::kMaxUes);
    big.ue_meas.resize(f1::kMaxUeMeas);
    big.homes.resize(f1::kMaxUes);
    const auto wb = f1::encode_report(big, fake_mac(0xF1));
    CHECK(!wb.empty() && wb.size() <= f1::kMaxDatagram);
    std::printf("cell_m6: a report at every bound: %zu bytes (datagram %zu)\n", wb.size(), f1::kMaxDatagram);

    /* It rides the handover command (F1 v9) and the IE (v3) intact. */
    HoCommand cmd;
    cmd.id = 77;
    cmd.ue = 1;
    cmd.source = 1;
    cmd.target = 2;
    cmd.target_ue = 2;
    cmd.target_pos = 1;
    cmd.ctx = *ctx;
    const Mac f1k = fake_mac(0xF1);
    const auto w = f1::encode_handover(2, cmd, f1k);
    auto dc = f1::decode_handover(w.data(), w.size(), f1k);
    CHECK(dc && dc->second == cmd);
    HoIe ie;
    ie.id = 77;
    ie.cell = 1;
    ie.ue = 1;
    ie.target = 2;
    ie.ctx = cmd.ctx;
    std::vector<uint8_t> bcn(36, 0);
    bcn[0] = 0x80;
    const Mac ck1 = fake_mac(0xC1);
    const auto hie = encode_ho_ie(ie, ck1);
    bcn.insert(bcn.end(), hie.begin(), hie.end());
    auto got = find_ho_ie(bcn.data(), bcn.size(), 1, 1, ck1);
    CHECK(got && got->ctx == *ctx);
    auto tamper = bcn;
    tamper[36 + 5 + 34 + 3] ^= 1; /* a context byte: the CRC and the tag cover it */
    CHECK(!find_ho_ie(tamper.data(), tamper.size(), 1, 1, ck1));
  }

  /* --- Signed neighbour lists (MeasGap.h NlVerifier). A fake signature
   * scheme stands in for Ed25519 (the logic, not the crypto): a "public key"
   * names its private one, sig = fake_mac(key)(msg) padded. --- */
  {
    auto fake_pub = [](uint8_t k) { Ed25519Pub p{}; p[0] = k; return p; };
    auto signer = [](uint8_t k) -> NlSigner {
      return [k](const uint8_t *m, size_t n, uint8_t sig[64]) {
        std::memset(sig, 0, 64);
        return fake_mac(k)(m, n, sig);
      };
    };
    const NlSigVerifier verify = [](const uint8_t pub[32], const uint8_t *m, size_t n, const uint8_t sig[64]) {
      uint8_t want[64] = {0};
      fake_mac(pub[0])(m, n, want);
      return std::memcmp(want, sig, 64) == 0;
    };
    const uint8_t ca = 0x5A, k2 = 0x22, k3 = 0x33;
    /* The CA certifies cell 2's key. */
    Ed25519Sig cert2{};
    {
      const auto m = nl_cert_message(2, fake_pub(k2));
      signer(ca)(m.data(), m.size(), cert2.data());
    }
    auto beacon = [&](uint16_t cell, uint32_t gen, std::vector<Neighbour> list, uint8_t sign_key,
                      bool with_sig, const Ed25519Pub &cert_pub, const Ed25519Sig &cert_sig) {
      std::vector<uint8_t> b(36, 0);
      b[0] = 0x80;
      const auto nl = encode_nl_ie(cell, gen, list, Mac{});
      b.insert(b.end(), nl.begin(), nl.end());
      if (with_sig) {
        const auto sg = encode_nl_sig_ie(cell, gen, 500, nl, signer(sign_key));
        b.insert(b.end(), sg.begin(), sg.end());
      }
      const auto ce = encode_nl_cert_ie(cell, cert_pub, cert_sig);
      b.insert(b.end(), ce.begin(), ce.end());
      return b;
    };
    const std::vector<Neighbour> l1 = {{1, 149, 51200}};
    NlVerifier v(fake_pub(ca), verify);
    auto good = beacon(2, 7, l1, k2, true, fake_pub(k2), cert2);
    auto r = v.check(good.data(), good.size(), 2);
    CHECK(r && r->nl.gen == 7 && r->nl.cells == l1 && r->win == 500); /* the chain holds, with its window */
    auto again = beacon(2, 7, l1, k2, true, fake_pub(k2), cert2);
    CHECK(v.check(again.data(), again.size(), 2));                   /* the same list + window: from the cache */
    const std::vector<Neighbour> l2 = {{1, 149, 12345}};
    auto changed = beacon(2, 7, l2, k2, false, fake_pub(k2), cert2);
    CHECK(!v.check(changed.data(), changed.size(), 2));              /* a new list needs its signature */
    auto forged = beacon(2, 8, l2, k3, true, fake_pub(k2), cert2);
    CHECK(!v.check(forged.data(), forged.size(), 2));                /* signed by another key */
    NlVerifier fresh(fake_pub(ca), verify);
    auto selfcert = beacon(2, 8, l2, k3, true, fake_pub(k3), cert2);
    CHECK(!fresh.check(selfcert.data(), selfcert.size(), 2));        /* a key the CA did not certify */
    Ed25519Sig cert_other{};
    {
      const auto m = nl_cert_message(3, fake_pub(k2)); /* certified for cell 3, not 2 */
      signer(ca)(m.data(), m.size(), cert_other.data());
    }
    auto wrongcell = beacon(2, 8, l2, k2, true, fake_pub(k2), cert_other);
    CHECK(!fresh.check(wrongcell.data(), wrongcell.size(), 2));
    {
      /* A CRC-valid list whose signature (same cell, same gen) covers ANOTHER
       * list: the signature is over the exact NL bytes. */
      std::vector<uint8_t> swapped(36, 0);
      swapped[0] = 0x80;
      const auto nl_a = encode_nl_ie(2, 9, l1, Mac{});
      const auto nl_b = encode_nl_ie(2, 9, l2, Mac{});
      const auto sg = encode_nl_sig_ie(2, 9, 500, nl_b, signer(k2));
      const auto ce = encode_nl_cert_ie(2, fake_pub(k2), cert2);
      for (const auto *ie : {&nl_a, &sg, &ce})
        swapped.insert(swapped.end(), ie->begin(), ie->end());
      CHECK(!fresh.check(swapped.data(), swapped.size(), 2));
    }
    auto okb = beacon(2, 9, l2, k2, true, fake_pub(k2), cert2);
    CHECK(fresh.check(okb.data(), okb.size(), 2));
    CHECK(!fresh.check(okb.data(), okb.size(), 5));                  /* not cell 5's list */
    {
      /* The window is signed: a genuine signature relabelled with another
       * window (a replayer making an old beacon look fresh) is refused. */
      NlVerifier v3(fake_pub(ca), verify);
      auto w = beacon(2, 12, l1, k2, true, fake_pub(k2), cert2); /* signed for window 500 */
      for (size_t i = 36; i + 1 < w.size(); ++i)
        if (w[i] == 221 && w[i + 5] == 'N' && w[i + 6] == 'S') {
          w[i + 2 + 3 + 9] ^= 1; /* the window's low byte: 500 -> 501 */
          break;
        }
      CHECK(!v3.check(w.data(), w.size(), 2));
      auto w2 = beacon(2, 12, l1, k2, true, fake_pub(k2), cert2);
      auto ok2 = v3.check(w2.data(), w2.size(), 2);
      CHECK(ok2 && ok2->win == 500);
      /* A list with no signature IE at all: refused even when cached. */
      auto bare = beacon(2, 12, l1, k2, false, fake_pub(k2), cert2);
      CHECK(!v3.check(bare.data(), bare.size(), 2));
    }
    {
      /* A genuine signed list FOLLOWED by a forged CRC-valid one (same cell,
       * same gen): refused — never the forged entries. Before any cache. */
      NlVerifier v2(fake_pub(ca), verify);
      auto dup = beacon(2, 11, l1, k2, true, fake_pub(k2), cert2);
      const auto forged_nl = encode_nl_ie(2, 11, l2, Mac{});
      dup.insert(dup.end(), forged_nl.begin(), forged_nl.end());
      CHECK(!v2.check(dup.data(), dup.size(), 2));
      /* ... and after the genuine one is cached. */
      auto alone = beacon(2, 11, l1, k2, true, fake_pub(k2), cert2);
      CHECK(v2.check(alone.data(), alone.size(), 2));
      CHECK(!v2.check(dup.data(), dup.size(), 2));
      /* The forged one first, the genuine one second: refused too. */
      std::vector<uint8_t> rev(36, 0);
      rev[0] = 0x80;
      rev.insert(rev.end(), forged_nl.begin(), forged_nl.end());
      rev.insert(rev.end(), alone.begin() + 36, alone.end());
      CHECK(!v2.check(rev.data(), rev.size(), 2));
    }
  }

  if (failures == 0)
    std::printf("cell_m6_selftest: all checks passed\n");
  return failures == 0 ? 0 : 1;
}
