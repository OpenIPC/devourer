// tsf_sync_selftest.cpp — headless unit tests for the TsfSync one-way time
// distribution fit (src/TsfSync.h): does it recover a known crystal skew and
// offset from {remote egress TSF, local RX TSF} beacon pairs, translate a
// timestamp between the two clocks, survive the local 32-bit TSF wrap, and pick
// beacons out of a frame stream via the Packet convenience? No hardware / no
// libusb — runs in ctest, unlike the txegress_witness harness (real adapters).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "RxPacket.h"
#include "TsfSync.h"

static int g_fail = 0;
#define CHECK(cond, msg)                                                        \
  do {                                                                          \
    if (!(cond)) { std::printf("FAIL: %s\n", (msg)); ++g_fail; }                \
  } while (0)

int main() {
  // --- 1. recover a known skew + offset, and translate between the clocks ----
  // Beacon pairs: local = (1 + skew)·remote + offset, at a 20 ms cadence, with
  // the inherent 1 µs TSF quantization (llround). This mirrors the bench setup
  // (a devourer beacon master's egress TSF vs a receiver's rx tsfl).
  {
    devourer::TsfSync s;
    const double skew_ppm = 30.0;
    const double a = 1.0 + skew_ppm / 1e6;
    const int64_t offset = 14799307;            // ~ the measured crystal offset
    const int64_t remote0 = 19410073;
    const int N = 1000;
    for (int i = 0; i < N; ++i) {
      const int64_t remote = remote0 + (int64_t)i * 20000;   // 20 ms beacons
      const int64_t local = (int64_t)llround(a * (double)remote) + offset;
      s.Add((uint64_t)remote, (uint32_t)(local & 0xffffffffu));  // local < 2^32
    }
    CHECK(s.Ready(), "ready after N beacons");
    CHECK(std::fabs(s.SkewPpm() - skew_ppm) < 1.0,
          "skew recovered within 1 ppm of the true 30 ppm");

    const int64_t r = remote0 + 500 * 20000;
    const int64_t expect_local = (int64_t)llround(a * (double)r) + offset;
    CHECK(std::llabs(s.LocalForRemote((uint64_t)r) - expect_local) <= 2,
          "LocalForRemote matches the true local within quantization");
    // Round-trip: remote -> local -> remote.
    CHECK(std::llabs(s.RemoteForLocal((uint64_t)expect_local) - r) <= 2,
          "RemoteForLocal inverts LocalForRemote");
  }

  // --- 2. local 32-bit TSF wrap is reconstructed --------------------------
  // Feed pairs whose local low word crosses 2^32 (a=1, skew 0). If the wrap were
  // mishandled the fitted slope would collapse (the low word drops ~4.29e9 at
  // the wrap); a recovered ~0 ppm proves the reconstruction.
  {
    devourer::TsfSync s;
    const int64_t base = (int64_t)0xFFFFF000;   // start just below the wrap
    const int64_t step = 20000;                 // 20 ms
    for (int i = 0; i < 200; ++i) {
      const int64_t remote = 1000 + (int64_t)i * step;
      const int64_t local = base + (int64_t)i * step;     // crosses 2^32
      s.Add((uint64_t)remote, (uint32_t)(local & 0xffffffffu));
    }
    CHECK(s.Ready(), "ready across the wrap");
    CHECK(std::fabs(s.SkewPpm()) < 1.0,
          "skew ~0 ppm across the 32-bit wrap (reconstruction correct)");
  }

  // --- 3. Packet convenience: ingest beacons, skip everything else --------
  {
    devourer::TsfSync s;
    uint8_t beacon[40] = {0};
    beacon[0] = 0x80;                            // beacon frame control
    const uint64_t egress = 0x0000000123456789ULL;
    for (int i = 0; i < 8; ++i) beacon[24 + i] = (uint8_t)(egress >> (8 * i));
    Packet bp;
    bp.Data = std::span<uint8_t>(beacon, sizeof(beacon));
    bp.RxAtrib.tsfl = 0x11223344u;
    CHECK(s.Add(bp), "beacon accepted by Packet convenience");
    CHECK(s.Count() == 1, "beacon counted");

    uint8_t data[40] = {0};
    data[0] = 0x08;                              // data frame — no egress TSF
    Packet dp;
    dp.Data = std::span<uint8_t>(data, sizeof(data));
    dp.RxAtrib.tsfl = 0x55667788u;
    CHECK(!s.Add(dp), "data frame rejected by Packet convenience");
    CHECK(s.Count() == 1, "data frame not counted");
  }

  {
    // ShiftRemote: fitted to transmitter A, then moved to B whose clock is
    // A's + delta (same rate: both held to one PTP timebase). The shifted fit
    // maps local time to B's clock without a sample of B, and B's beacons
    // then extend the same line (no reset, Ready() throughout).
    devourer::TsfSync s;
    const double skew = 1.0 + 15e-6; // local runs 15 ppm fast vs both
    const int64_t a0 = 5'000'000, delta = 13450 - 7 * 102400LL;
    const uint32_t l0 = 1'000'000;
    for (int i = 0; i < 40; ++i) {
      const int64_t r = a0 + i * 102400LL;
      s.Add(static_cast<uint64_t>(r), static_cast<uint32_t>(l0 + llround((r - a0) * skew)));
    }
    CHECK(s.Ready(), "ready on A");
    const int64_t at = a0 + 40 * 102400LL + 5000; // A's TSF at some later instant
    const uint64_t local_at = l0 + llround((at - a0) * skew);
    const int64_t a_est = s.RemoteForLocal(local_at);
    s.ShiftRemote(delta);
    CHECK(s.Ready(), "still ready after the shift");
    CHECK(std::llabs(s.RemoteForLocal(local_at) - (a_est + delta)) <= 1,
          "shifted fit = A's clock + delta");
    CHECK(std::llabs(s.RemoteForLocal(local_at) - (at + delta)) <= 1, "= B's true clock");
    for (int i = 40; i < 60; ++i) { // B's beacons (B's TSF = A's + delta)
      const int64_t r = a0 + i * 102400LL;
      s.Add(static_cast<uint64_t>(r + delta), static_cast<uint32_t>(l0 + llround((r - a0) * skew)));
    }
    CHECK(std::fabs(s.SkewPpm() - 15.0) < 0.5, "B's beacons extend the same line");
    const int64_t later = a0 + 70 * 102400LL;
    CHECK(std::llabs(s.RemoteForLocal(l0 + llround((later - a0) * skew)) - (later + delta)) <= 1,
          "tracks B after the switch");
  }

  std::printf(g_fail ? "tsf_sync_selftest: %d FAILURE(S)\n"
                     : "tsf_sync_selftest: all passed\n",
              g_fail);
  return g_fail ? 1 : 0;
}
