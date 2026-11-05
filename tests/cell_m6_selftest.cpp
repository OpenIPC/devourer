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

using namespace devourer::cell;

static int failures = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      failures++;                                                              \
    }                                                                          \
  } while (0)

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

  /* --- F1 v2: the measurement fields round-trip. --- */
  {
    f1::Report r;
    r.du = 2;
    r.heard = {{1, 1, -71, 18}};
    r.meas = {{1, -52 * 4 - 1, 33 * 4 + 3}, {2, -90 * 4, -2 * 4}};
    const auto w = f1::encode_report(r, {});
    auto d = f1::decode_report(w.data(), w.size(), {});
    CHECK(d && *d == r);
    CHECK(w[2] == 2); /* version 2 on the wire */
    auto old = w;
    old[2] = 1; /* a v1 sender is refused, never half-parsed */
    CHECK(!f1::decode_report(old.data(), old.size(), {}));
  }

  if (failures == 0)
    std::printf("cell_m6_selftest: all checks passed\n");
  return failures == 0 ? 0 : 1;
}
