// ue_rx_attribution_selftest.cpp — headless unit tests for the per-UE RX
// attribution seed (src/cell/UeRxAttribution.h): TA extraction over the frame
// types that do / don't carry addr2, multi-TA accumulation with the
// RxQualityAccumulator folding conventions, drain-and-reset semantics, and the
// bounded-table eviction accounting. No hardware — runs in ctest; the on-air
// counterpart is tests/ue_rx_attribution_check.sh.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "cell/UeRxAttribution.h"

static int g_fail = 0;
#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    if (!(cond)) { std::printf("FAIL: %s\n", (msg)); ++g_fail; }               \
  } while (0)

// Minimal 24-byte MPDU header with the given FC byte 0 and addr2.
static std::vector<uint8_t> mk_mpdu(uint8_t fc0, const uint8_t ta[6],
                                    size_t len = 24) {
  std::vector<uint8_t> f(len, 0);
  f[0] = fc0;
  for (int i = 0; i < 6; ++i)
    f[4 + i] = 0xff; // addr1 broadcast
  if (len >= 16)
    std::memcpy(f.data() + 10, ta, 6);
  return f;
}

int main() {
  using devourer::cell::UeRxAttribution;
  using devourer::cell::extract_ta;

  const uint8_t taA[6] = {0x02, 0xaa, 0xbb, 0xcc, 0xdd, 0x01};
  const uint8_t taB[6] = {0x02, 0xaa, 0xbb, 0xcc, 0xdd, 0x02};

  // --- 1. extract_ta over frame types ---------------------------------------
  {
    uint8_t out[6];
    auto data = mk_mpdu(0x08, taA); // data
    CHECK(extract_ta(data.data(), data.size(), out) &&
              std::memcmp(out, taA, 6) == 0,
          "data frame yields addr2");
    auto qos = mk_mpdu(0x88, taB, 26); // QoS data
    CHECK(extract_ta(qos.data(), qos.size(), out) &&
              std::memcmp(out, taB, 6) == 0,
          "QoS data frame yields addr2");
    auto beacon = mk_mpdu(0x80, taA);
    CHECK(extract_ta(beacon.data(), beacon.size(), out), "beacon yields addr2");
    auto rts = mk_mpdu(0xb4, taA, 16); // RTS: control WITH a TA
    CHECK(extract_ta(rts.data(), rts.size(), out) &&
              std::memcmp(out, taA, 6) == 0,
          "RTS yields addr2");
    auto ba = mk_mpdu(0x94, taB, 16); // BlockAck: control WITH a TA
    CHECK(extract_ta(ba.data(), ba.size(), out), "BlockAck yields addr2");
    auto cts = mk_mpdu(0xc4, taA, 16); // CTS ends at addr1
    CHECK(!extract_ta(cts.data(), cts.size(), out), "CTS rejected (no addr2)");
    auto ack = mk_mpdu(0xd4, taA, 16); // ACK ends at addr1
    CHECK(!extract_ta(ack.data(), ack.size(), out), "ACK rejected (no addr2)");
    auto shortf = mk_mpdu(0x08, taA, 12);
    CHECK(!extract_ta(shortf.data(), shortf.size(), out), "short frame rejected");
    const uint8_t zero[6] = {0, 0, 0, 0, 0, 0};
    auto zta = mk_mpdu(0x08, zero);
    CHECK(!extract_ta(zta.data(), zta.size(), out), "zero TA rejected");
    CHECK(!extract_ta(nullptr, 24, out), "null buffer rejected");
  }

  // --- 2. multi-TA accumulation + folding conventions -----------------------
  {
    UeRxAttribution attr;
    // UE A: two frames, rssi 60/70 raw, snr 20/40 raw, evm only on the second.
    attr.add(taA, 60, 20, 0, 1000);
    attr.add(taA, 70, 40, -30, 2000);
    // UE B: one CCK-ish frame (no snr/evm).
    attr.add(taB, 50, 0, 0, 1500);
    // Non-samples: rssi<=0 must not create or touch an entry.
    attr.add(taB, 0, 10, -10, 9999);
    auto s = attr.snapshot();
    CHECK(s.ues.size() == 2, "two UEs attributed");
    CHECK(s.evicted_frames == 0, "no evictions");
    const devourer::cell::UeRxWindow *a = nullptr, *b = nullptr;
    for (const auto &w : s.ues) {
      if (std::memcmp(w.ta.data(), taA, 6) == 0) a = &w;
      if (std::memcmp(w.ta.data(), taB, 6) == 0) b = &w;
    }
    CHECK(a && b, "both TAs present");
    if (a) {
      CHECK(a->frames == 2, "UE A frame count");
      CHECK(a->rssi_mean_dbm == 65 - 110, "UE A rssi mean (raw-110)");
      CHECK(a->rssi_max_dbm == 70 - 110, "UE A rssi max");
      CHECK(std::fabs(a->snr_mean_db - 15.0) < 1e-9, "UE A snr mean (raw/2)");
      CHECK(std::fabs(a->snr_min_db - 10.0) < 1e-9, "UE A snr min");
      CHECK(a->evm_valid && std::fabs(a->evm_mean_db + 15.0) < 1e-9,
            "UE A evm folded only from the frame that carried one");
      // nf: frame1 (60-110)-10 = -60; frame2 (70-110)-20 = -60 → mean -60.
      CHECK(a->nf_valid && std::fabs(a->noise_floor_dbm + 60.0) < 1e-9,
            "UE A passive noise floor");
      CHECK(a->last_tsfl == 2000, "UE A last_tsfl is the newest frame");
    }
    if (b) {
      CHECK(b->frames == 1, "UE B frame count (rssi<=0 frame skipped)");
      CHECK(!b->evm_valid && !b->nf_valid, "UE B no snr/evm → flags false");
      CHECK(b->last_tsfl == 1500, "UE B last_tsfl untouched by the non-sample");
    }
  }

  // --- 3. drain-and-reset (delta semantics) ---------------------------------
  {
    UeRxAttribution attr;
    attr.add(taA, 60, 20, 0, 1);
    auto s1 = attr.snapshot();
    CHECK(s1.ues.size() == 1, "first window has the UE");
    auto s2 = attr.snapshot();
    CHECK(s2.ues.empty() && s2.evicted_frames == 0,
          "second window empty after drain");
    attr.add(taA, 80, 30, 0, 2);
    auto s3 = attr.snapshot();
    CHECK(s3.ues.size() == 1 && s3.ues[0].frames == 1 &&
              s3.ues[0].rssi_mean_dbm == 80 - 110,
          "UE reappears fresh after drain");
  }

  // --- 4. bounded table + eviction accounting -------------------------------
  {
    UeRxAttribution attr(2); // cap 2
    attr.add(taA, 60, 0, 0, 1);
    attr.add(taB, 60, 0, 0, 2);
    const uint8_t taC[6] = {0x02, 0xaa, 0xbb, 0xcc, 0xdd, 0x03};
    attr.add(taC, 60, 0, 0, 3); // table full → evicted
    attr.add(taC, 60, 0, 0, 4);
    attr.add(taA, 70, 0, 0, 5); // known TA still accumulates
    auto s = attr.snapshot();
    CHECK(s.ues.size() == 2, "cap holds");
    CHECK(s.evicted_frames == 2, "evicted frames counted, not silently lost");
    // After the drain the table is empty again — taC now fits.
    attr.add(taC, 60, 0, 0, 6);
    auto s2 = attr.snapshot();
    CHECK(s2.ues.size() == 1 && std::memcmp(s2.ues[0].ta.data(), taC, 6) == 0,
          "evicted TA admitted in the next window");
  }

  std::printf(g_fail ? "ue_rx_attribution_selftest: %d FAILURE(S)\n"
                     : "ue_rx_attribution_selftest: all passed\n",
              g_fail);
  return g_fail ? 1 : 0;
}
