// tdma_tsf_selftest.cpp — headless unit tests for the burst-TDMA pure logic:
// the TsfClock host↔TSF least-squares fit (does it recover a known crystal drift
// and de-jitter the anchor?), the on-air frame-tag build/parse round-trip, and
// the schedule phase math. No hardware / no libusb — runs in ctest, unlike the
// tsf_probe / tsf_tdoa_probe / tdma_tsf_test harnesses which need real adapters.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "tdma.h"

static int g_fail = 0;
#define CHECK(cond, msg)                                                        \
  do {                                                                          \
    if (!(cond)) { std::printf("FAIL: %s\n", (msg)); ++g_fail; }                \
  } while (0)

int main() {
  // --- 1. TsfClock recovers a known drift and de-jitters the anchor ---------
  // Synthesise host = a·tsf + b with heavy jitter (like the ~200 µs+ USB noise),
  // and check the fit both recovers the drift (slope) and predicts the true host
  // far more tightly than the noisy samples do.
  {
    tdma::TsfClock clk;
    const double drift_ppm = 20.0;
    const double a = 1.0 + drift_ppm / 1e6;   // host µs per tsf µs
    const double b = 12345.0;
    const int N = 1000;
    uint32_t s = 1;                            // deterministic ±200 µs jitter
    auto jit = [&]() { s = s * 1664525u + 1013904223u;
                       return (int)((s >> 8) % 400) - 200; };
    std::vector<int64_t> tsf;
    std::vector<double> host_true;
    double max_raw = 0, max_fit = 0;
    for (int i = 0; i < N; ++i) {
      int64_t t = 1000 + (int64_t)i * 5000;    // µs, no 32-bit wrap over the run
      double ht = a * (double)t + b;
      int j = jit();
      clk.add((uint32_t)t, (int64_t)(ht + j));
      tsf.push_back(t);
      host_true.push_back(ht);
      if (std::fabs((double)j) > max_raw) max_raw = std::fabs((double)j);
    }
    CHECK(clk.ready(), "clock ready after N frames");
    for (int i = 0; i < N; ++i) {
      double e = std::fabs(clk.host_at(tsf[i]) - host_true[i]);
      if (e > max_fit) max_fit = e;
    }
    const double slope = (clk.host_at(1000000) - clk.host_at(0)) / 1000000.0;
    CHECK(std::fabs((slope - 1.0) * 1e6 - drift_ppm) < 5.0,
          "drift recovered within 5 ppm of the true 20 ppm");
    CHECK(max_raw > 150.0, "raw samples carry the injected ~200 us jitter");
    CHECK(max_fit < 80.0, "fit de-jitters the anchor to << the raw jitter");
  }

  // --- 2. frame-tag build/parse round-trip ----------------------------------
  {
    std::vector<uint8_t> rt(10, 0xAA);   // dummy radiotap prefix (stripped on RX)
    for (tdma::Class cls :
         {tdma::Class::Marker, tdma::Class::Critical, tdma::Class::Bulk}) {
      const uint32_t seq = 0x11223344u, burst = 0x0055aa77u;
      const uint64_t txtsf = 0x0123456789abcdefULL;
      auto f = tdma::build_frame(rt, cls, seq, burst, txtsf);
      // The RX sees the 802.11 MPDU (radiotap already stripped) → parse past rt.
      auto p = tdma::parse_frame(f.data() + rt.size(), f.size() - rt.size());
      CHECK(p.ok, "parse ok");
      CHECK(p.cls == cls, "class round-trips");
      CHECK(p.seq == seq, "seq round-trips");
      CHECK(p.burst == burst, "burst round-trips");
      CHECK(p.tx_tsf == txtsf, "tx_tsf round-trips");
    }
    std::vector<uint8_t> junk(60, 0x00);      // wrong SA / no TD magic
    CHECK(!tdma::parse_frame(junk.data(), junk.size()).ok, "foreign frame rejected");
    CHECK(!tdma::parse_frame(rt.data(), 8).ok, "short buffer rejected");

    // v2 tag: host_ns round-trips; v1 frames parse with host_ns = 0.
    {
      const uint64_t txtsf = 0x1122334455667788ULL;
      const uint64_t hns = 0xfedcba9876543210ULL;
      auto v2 = tdma::build_frame(rt, tdma::Class::Marker, 7, 3, txtsf, hns);
      CHECK(v2.size() == rt.size() + tdma::kHdrLen + tdma::kTagLenV2,
            "v2 frame is 8 bytes longer");
      auto p2 = tdma::parse_frame(v2.data() + rt.size(), v2.size() - rt.size());
      CHECK(p2.ok && p2.ver == 2, "v2 parses with ver=2");
      CHECK(p2.tx_tsf == txtsf, "v2 tx_tsf round-trips");
      CHECK(p2.host_ns == hns, "v2 host_ns round-trips");
      auto v1 = tdma::build_frame(rt, tdma::Class::Marker, 7, 3, txtsf);
      auto p1 = tdma::parse_frame(v1.data() + rt.size(), v1.size() - rt.size());
      CHECK(p1.ok && p1.ver == 1 && p1.host_ns == 0,
            "v1 parses with ver=1, host_ns=0");
      // A truncated v2 (ver byte says 2 but only 20 tag bytes) must not read
      // past the buffer — host_ns stays 0.
      auto trunc = v2;
      trunc.resize(rt.size() + tdma::kHdrLen + tdma::kTagLen);
      auto pt = tdma::parse_frame(trunc.data() + rt.size(), trunc.size() - rt.size());
      CHECK(pt.ok && pt.host_ns == 0, "truncated v2 parses safely, host_ns=0");
    }
  }

  // --- 3. schedule phase math -----------------------------------------------
  {
    tdma::Schedule s;
    s.nb_ms = 100; s.wide_ms = 100; s.epoch_ms = 0;   // period 200
    CHECK(s.period() == 200, "period");
    CHECK(s.at(0).phase == tdma::Phase::NB, "t=0 NB");
    CHECK(s.at(99).phase == tdma::Phase::NB, "t=99 NB");
    CHECK(s.at(100).phase == tdma::Phase::WIDE, "t=100 WIDE");
    CHECK(s.at(199).phase == tdma::Phase::WIDE, "t=199 WIDE");
    CHECK(s.at(200).phase == tdma::Phase::NB && s.at(200).burst == 1,
          "t=200 NB, burst 1");
    CHECK(s.at(250).phase == tdma::Phase::NB, "t=250 (pos 50) NB");
    CHECK(s.at(350).phase == tdma::Phase::WIDE, "t=350 (pos 150) WIDE");
    s.epoch_ms = 1000;                                 // negative-rel handling
    CHECK(s.at(1050).phase == tdma::Phase::NB, "epoch-anchored NB");
    CHECK(s.at(950).phase == tdma::Phase::WIDE, "pre-epoch wraps correctly");
  }

  std::printf(g_fail ? "tdma_tsf_selftest: %d FAILURE(S)\n"
                     : "tdma_tsf_selftest: all passed\n",
              g_fail);
  return g_fail ? 1 : 0;
}
