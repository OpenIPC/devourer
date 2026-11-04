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
    auto b = build_ue_beacon(5, 3, 100, 42,
                             devourer::cell::HarqFeedback{0x01020304u, 0xA0000005u, 0x1'2345'6789ull}, 61,
                             9);
    auto u = parse_ue_beacon(b.data(), b.size(), 5);
    CHECK(u && u->ue == 3 && u->epoch == 42);
    CHECK(u && u->fb.base == 0x01020304u && u->fb.bitmap == 0xA0000005u && u->dl_snr_half_db == 61);
    CHECK(u && u->fb.gen_sf == 0x2345'6789ull && u->snr_gen == 9); /* gen: low 32 bits on air */
    CHECK(extend32_near(0x2345'6789u, 0x1'2345'6000ll) == 0x1'2345'6789ll);
    auto plain = build_ue_beacon(5, 3, 100, 7); /* defaults: no feedback, no SNR */
    auto up = parse_ue_beacon(plain.data(), plain.size(), 5);
    CHECK(up && up->fb == devourer::cell::HarqFeedback{} && up->dl_snr_half_db == 0);
    CHECK(!parse_ue_beacon(b.data(), b.size(), 6));      /* other cell */
    auto du = build_ue_beacon(5, 3, 100, 42);
    du[15] = 0;                                            /* SA = the DU */
    CHECK(!parse_ue_beacon(du.data(), du.size(), 5));
    auto bad = b;
    bad[bad.size() - 28] = 4;                              /* body ue != SA */
    CHECK(!parse_ue_beacon(bad.data(), bad.size(), 5));
    bad = b;
    bad[bad.size() - 27] = 1;                              /* ue high byte: 0x0103 != 3 */
    CHECK(!parse_ue_beacon(bad.data(), bad.size(), 5));
    CHECK(!parse_ue_beacon(b.data(), b.size() - 1, 5));   /* torn IE */
    CHECK(!parse_ue_beacon(nullptr, 0, 5));
  }

  /* DL frame round trip and rejects. */
  {
    auto f = build_dl_frame(5, 2, 0x0102030405ull, 77, 10, DlHarq{70, 5, 3});
    CHECK(f.size() == 26 + kDlBodyLen + 10);
    CHECK(f[24] == 0x00); /* QoS: TID 0, normal ACK (the UE ACKs in hardware) */
    auto d = parse_dl_frame(f.data(), f.size(), 5);
    CHECK(d && d->ue == 2 && d->sf == 0x0102030405ull && d->seq == 77);
    CHECK(d && d->harq.low == 70 && d->harq.pid == 5 && d->harq.attempt == 3);
    CHECK(!parse_dl_frame(f.data(), f.size(), 4));
    auto g = f;
    g[30] = 1; /* body ue high byte: != RA */
    CHECK(!parse_dl_frame(g.data(), g.size(), 5));
    g = f;
    g[10 + 5] = 3; /* TA is not the DU */
    CHECK(!parse_dl_frame(g.data(), g.size(), 5));
    CHECK(!parse_dl_frame(f.data(), 26 + kDlBodyLen - 1, 5));
  }

  /* DL radiotap: RATE (500 kb/s units) + signed DBM_TX_POWER, no TX_FLAGS. */
  {
    const auto rt = build_dl_radiotap(24, -12);
    CHECK(rt.size() == 10 && rt[2] == 10 && rt[3] == 0);
    CHECK(rt[4] == 0x04 && rt[5] == 0x04 && rt[6] == 0 && rt[7] == 0);
    CHECK(rt[8] == 48 && static_cast<int8_t>(rt[9]) == -12);
    /* No power requested: no DBM_TX_POWER field at all (on Kestrel its mere
     * presence is an absolute level that bypasses the session power). */
    const auto r0 = build_dl_radiotap(24);
    CHECK(r0.size() == 9 && r0[2] == 9 && r0[4] == 0x04 && r0[5] == 0x00 && r0[8] == 48);
  }

  /* Scheduling-frame integrity: version + tag (keyed by a stand-in MAC). */
  {
    auto keyed = [](uint64_t k) -> CellMac {
      return [k](const uint8_t *d, size_t n, uint8_t out[20]) {
        uint64_t h = 1469598103934665603ull ^ k;
        for (size_t i = 0; i < n; ++i) h = (h ^ d[i]) * 1099511628211ull;
        for (int i = 0; i < 20; ++i) { h = (h ^ static_cast<uint64_t>(i)) * 1099511628211ull; out[i] = static_cast<uint8_t>(h >> 32); }
        return true;
      };
    };
    const CellMac k1 = keyed(1), k2 = keyed(2);
    const devourer::cell::HarqFeedback fb{7, 3, 99};
    auto ub = build_ue_beacon(5, 2, 100, 4, fb, 50, 1, k1);
    CHECK(parse_ue_beacon(ub.data(), ub.size(), 5, k1).has_value());
    CHECK(!parse_ue_beacon(ub.data(), ub.size(), 5, k2));    /* wrong key */
    CHECK(parse_ue_beacon(ub.data(), ub.size(), 5).has_value()); /* unauthenticated receiver */
    auto unsigned_ub = build_ue_beacon(5, 2, 100, 4, fb, 50, 1);
    CHECK(!parse_ue_beacon(unsigned_ub.data(), unsigned_ub.size(), 5, k1)); /* keyed: tag required */
    auto tampered = ub;
    tampered[tampered.size() - 8 - 23 + 9] ^= 1; /* fb.base byte */
    CHECK(!parse_ue_beacon(tampered.data(), tampered.size(), 5, k1));
    auto ver = ub;
    ver[ver.size() - 8 - 23 + 2] = 1; /* an older protocol version */
    CHECK(!parse_ue_beacon(ver.data(), ver.size(), 5));
    auto dl = build_dl_frame(5, 2, 10, 20, 0, DlHarq{18, 4, 2}, k1);
    CHECK(parse_dl_frame(dl.data(), dl.size(), 5, k1).has_value());
    CHECK(!parse_dl_frame(dl.data(), dl.size(), 5, k2));
    auto dlt = dl;
    dlt[26 + 13] ^= 1; /* seq */
    CHECK(!parse_dl_frame(dlt.data(), dlt.size(), 5, k1));
    auto dlv = dl;
    dlv[28] = 1; /* version */
    CHECK(!parse_dl_frame(dlv.data(), dlv.size(), 5));
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
