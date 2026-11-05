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
#include "cell/Handover.h"
#include "cell/HoCommand.h"

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
    HoCommand h{1793880000u, 1, 1, 2, 3, 2, 17518356600ull, -1234567890123ll, 99};
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
    HoIe a{100, 1, 1, 2, 3, 3, 7, 4200, 13450 - 1000 * 102400ll};
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
      CHECK(ie.size() == 2 + 3 + 33 + 2 + kHoTagLen);
      mpdu.insert(mpdu.end(), ie.begin(), ie.end());
    }
    auto got = find_ho_ie(mpdu.data(), mpdu.size(), 1, 1, k);
    CHECK(got && *got == newer); /* the newest id addressed to (cell 1, UE 1) */
    CHECK(!find_ho_ie(mpdu.data(), mpdu.size(), 1, 1, other)); /* wrong key: inert */
    CHECK(!find_ho_ie(mpdu.data(), mpdu.size(), 2, 1, k));     /* another cell's UE */
    auto torn = mpdu;
    torn[36 + 5 + 10] ^= 0x40; /* inside the first IE's body */
    auto t = find_ho_ie(torn.data(), torn.size(), 1, 1, k);
    CHECK(t && *t == newer); /* the torn one is dropped, the intact newer one stays */
    auto plain = encode_ho_ie(a, {});
    std::vector<uint8_t> m2(36, 0);
    m2.insert(m2.end(), plain.begin(), plain.end());
    CHECK(find_ho_ie(m2.data(), m2.size(), 1, 1, {}));  /* unkeyed cell: tag ignored */
    CHECK(!find_ho_ie(m2.data(), m2.size(), 1, 1, k));  /* keyed UE: zero tag refused */
  }

  if (failures == 0)
    std::printf("cell_m6_selftest: all checks passed\n");
  return failures == 0 ? 0 : 1;
}
