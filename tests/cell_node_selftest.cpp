/* Headless guard for the M1 cell node's pure logic (examples/cell/cell.h):
 * identities, the DU beacon / UE beacon / DL frame build-parse round trips and
 * their rejects, 32->64-bit counter extension, phase wrapping, the UE TSF-step
 * rule (incl. its sign convention with the timing advance), and the DU TA
 * controller. No hardware. */
#include <cstdint>
#include <cstdio>
#include <vector>

#include "cell.h"

using namespace cellnode;

static int failures = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      failures++;                                                              \
    }                                                                          \
  } while (0)

int main() {
  /* Identities. */
  uint8_t m[6];
  cell_mac(m, 0x0102, 7);
  CHECK(m[0] == 0x02 && m[3] == 0x01 && m[4] == 0x02 && m[5] == 7);
  CHECK(cell_station(m, 0x0102) == 7);
  CHECK(cell_station(m, 0x0103) == -1);
  cell_mac(m, 0x0102, 0);
  CHECK(cell_station(m, 0x0102) == 0);

  /* DU beacon carries the grant map; find_grant_map recovers it. */
  {
    GrantMap g;
    g.cell_id = 5;
    g.epoch = 9;
    g.period_tu = 100;
    g.grants = {{1, 1, 4, 6, 0, -3}, {2, 2, 5, 6, 0, 7}};
    const auto b = build_du_beacon(g);
    CHECK(!b.empty() && b[0] == 0x80 && cell_station(b.data() + 10, 5) == 0);
    CHECK(b[32] == 100 && b[33] == 0); /* beacon interval */
    auto d = devourer::cell::find_grant_map(b.data(), b.size());
    CHECK(d.has_value() && *d == g);
    GrantMap big = g;
    big.grants.resize(devourer::cell::kGrantMax + 1);
    CHECK(build_du_beacon(big).empty());
  }

  /* UE beacon round trip and rejects. */
  {
    auto b = build_ue_beacon(5, 3, 100, 42);
    auto u = parse_ue_beacon(b.data(), b.size(), 5);
    CHECK(u && u->ue == 3 && u->epoch == 42);
    CHECK(!parse_ue_beacon(b.data(), b.size(), 6));      /* other cell */
    auto du = build_ue_beacon(5, 3, 100, 42);
    du[15] = 0;                                            /* SA = the DU */
    CHECK(!parse_ue_beacon(du.data(), du.size(), 5));
    auto bad = b;
    bad[bad.size() - 6] = 4;                               /* body ue != SA */
    CHECK(!parse_ue_beacon(bad.data(), bad.size(), 5));
    bad = b;
    bad[bad.size() - 5] = 1;                               /* ue high byte: 0x0103 != 3 */
    CHECK(!parse_ue_beacon(bad.data(), bad.size(), 5));
    CHECK(!parse_ue_beacon(b.data(), b.size() - 1, 5));   /* torn IE */
    CHECK(!parse_ue_beacon(nullptr, 0, 5));
  }

  /* DL frame round trip and rejects. */
  {
    auto f = build_dl_frame(5, 2, 0x0102030405ull, 77, 10);
    CHECK(f.size() == 26 + 16 + 10);
    CHECK(f[24] == 0x00); /* QoS: TID 0, normal ACK (the UE ACKs in hardware) */
    auto d = parse_dl_frame(f.data(), f.size(), 5);
    CHECK(d && d->ue == 2 && d->sf == 0x0102030405ull && d->seq == 77);
    CHECK(!parse_dl_frame(f.data(), f.size(), 4));
    auto g = f;
    g[29] = 1; /* body ue != RA */
    CHECK(!parse_dl_frame(g.data(), g.size(), 5));
    g = f;
    g[10 + 5] = 3; /* TA is not the DU */
    CHECK(!parse_dl_frame(g.data(), g.size(), 5));
    CHECK(!parse_dl_frame(f.data(), 41, 5));
  }

  /* Counter extension. */
  {
    const int64_t ref = (int64_t{5} << 32) + 100;
    CHECK(extend32_near(150, ref) == (int64_t{5} << 32) + 150);
    CHECK(extend32_near(0xFFFFFF00u, ref) == (int64_t{4} << 32) + 0xFFFFFF00u);
    const int64_t ref2 = (int64_t{5} << 32) + 0xFFFFFF00u;
    CHECK(extend32_near(20, ref2) == (int64_t{6} << 32) + 20);
    Wrap32 w;
    CHECK(!w.ready());
    CHECK(w.feed(0xFFFFFFF0u) == 0xFFFFFFF0ll);
    CHECK(w.extend(0xFFFFFFF8u) == 0xFFFFFFF8ll);
    CHECK(w.extend(0x10u) == (int64_t{1} << 32) + 0x10); /* wrapped since */
    CHECK(w.feed(0x20u) == (int64_t{1} << 32) + 0x20);
  }

  /* Phase wrap. */
  CHECK(wrap_phase(10, 1000) == 10);
  CHECK(wrap_phase(990, 1000) == -10);
  CHECK(wrap_phase(-10, 1000) == -10);
  CHECK(wrap_phase(2500, 1000) == -500);
  CHECK(wrap_phase(499, 1000) == 499);

  /* UE TSF step: fires at du % P == ul_off - ta. */
  {
    const int64_t P = 102400;
    /* UE tsf already du - ul_off: no step. */
    CHECK(ue_tsf_step(1000000 - 23000, 1000000, 23000, 0, P, 2) == 0);
    /* UE 7 us ahead -> step -7. */
    CHECK(ue_tsf_step(1000000 - 23000 + 7, 1000000, 23000, 0, P, 2) == -7);
    /* Within the deadband: no step. */
    CHECK(ue_tsf_step(1000000 - 23000 + 2, 1000000, 23000, 0, P, 2) == 0);
    /* TA 5: the UE wants to read 5 us MORE (fire 5 us earlier on the DU grid). */
    CHECK(ue_tsf_step(1000000 - 23000, 1000000, 23000, 5, P, 2) == 5);
    /* Whole-period differences are not errors. */
    CHECK(ue_tsf_step(1000000 - 23000 + 3 * P, 1000000, 23000, 0, P, 2) == 0);
    /* After the step the UE's TBTT (tsf % P == 0) lands at du % P == ul_off - ta. */
    const int64_t du = 5 * P + 777, ul_off = 23000, ta = 12;
    int64_t ue = 31337;
    ue += ue_tsf_step(ue, du, ul_off, ta, P, 0);
    const int64_t du_at_tbtt = du + ((P - (ue % P)) % P); /* advance to the UE's TBTT */
    CHECK(wrap_phase(du_at_tbtt - (ul_off - ta), P) == 0);
  }

  /* TA loop: late arrivals raise the TA (fire earlier), with gain, clamp, min. */
  {
    TaLoop t(0.5, 50);
    t.sample(10);
    t.sample(10);
    CHECK(!t.update(3)); /* too few samples: kept */
    t.sample(10);
    CHECK(t.update(3) && t.ta() == 5);
    CHECK(t.pending() == 0);
    for (int i = 0; i < 3; ++i) t.sample(-4);
    CHECK(t.update(3) && t.ta() == 3);
    for (int i = 0; i < 3; ++i) t.sample(1000);
    CHECK(t.update(3) && t.ta() == 50); /* clamped */
    for (int i = 0; i < 3; ++i) t.sample(1);
    CHECK(!t.update(3) && t.ta() == 50); /* still clamped: unchanged */
    t.sample(100);
    t.discard();
    CHECK(t.pending() == 0 && t.ta() == 50);
  }

  if (failures == 0)
    std::printf("cell_node_selftest: all checks passed\n");
  return failures == 0 ? 0 : 1;
}
