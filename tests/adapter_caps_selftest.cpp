/* Headless guard for the pure-logic parts of the adapter-capabilities API
 * (src/AdapterCaps.h + the ActiveRxPaths classifier and extended freq<->chan
 * mapping in src/RxQuality.h / src/ChannelFreq.h) — so a regression fails
 * `ctest` instead of only surfacing on a radio. Prints the failing check and
 * exits nonzero. */
#include <cstdio>
#include <string_view>

#include "AdapterCaps.h"
#include "ChannelFreq.h"
#include "RxQuality.h"
#include "rtl8733b/Rtl8733bUsbIds.h"

static int g_fail = 0;

static void expect(const char *what, bool ok) {
  if (ok)
    return;
  ++g_fail;
  std::printf("FAIL: %s\n", what);
}

int main() {
  using namespace devourer;

  /* --- bandwidth mask per generation --- */
  const uint8_t ac = kBw20 | kBw40 | kBw80;
  expect("J1 bw = 20/40/80",
         bw_mask_for_generation(ChipGeneration::Jaguar1) == ac);
  expect("J2 bw adds 5/10",
         bw_mask_for_generation(ChipGeneration::Jaguar2) ==
             (ac | kBw5 | kBw10));
  expect("J3 bw adds 5/10",
         bw_mask_for_generation(ChipGeneration::Jaguar3) ==
             (ac | kBw5 | kBw10));
  expect("Unknown bw = 0",
         bw_mask_for_generation(ChipGeneration::Unknown) == 0);
  expect("RTL8733B bw = 10/20/40 (5 MHz refused on this die)",
         bw_mask_for_generation(ChipGeneration::Rtl8733b) ==
             (kBw10 | kBw20 | kBw40));
  /* Kestrel (11ax) does 20/40/80 + 160 MHz (RTL8852C, validated on-air at
   * 6 GHz). */
  expect("Kestrel bw = 20/40/80/160",
         bw_mask_for_generation(ChipGeneration::Kestrel) ==
             (ac | kBw5 | kBw10));

  /* --- generation names --- */
  expect("gen name jaguar3",
         std::string_view(generation_name(ChipGeneration::Jaguar3)) ==
             "jaguar3");
  expect("gen name kestrel",
         std::string_view(generation_name(ChipGeneration::Kestrel)) ==
             "kestrel");
  expect("gen name rtl8733b",
         std::string_view(generation_name(ChipGeneration::Rtl8733b)) ==
             "rtl8733b");
  expect("gen name unknown",
         std::string_view(generation_name(ChipGeneration::Unknown)) ==
             "unknown");

  /* --- RTL8733B identity gates --- */
  expect("RTL8733B chip id accepted", rtl8733b::is_chip_id(0x16));
  expect("RTL8733B neighboring chip id rejected", !rtl8733b::is_chip_id(0x17));
  expect("RTL8733B Wi-Fi PID accepted", rtl8733b::is_usb_id(0x0bda, 0xf72b));
  expect("RTL8733B combo PID accepted", rtl8733b::is_usb_id(0x0bda, 0xb733));
  expect("RTL8733B wrong VID rejected", !rtl8733b::is_usb_id(0x1234, 0xf72b));

  /* --- extended freq<->chan round-trip (Part B lifted the 5895 MHz cap) --- */
  expect("5080 MHz -> ch16", freq_to_chan(5080) == 16);
  expect("ch16 -> 5080 MHz", chan_to_freq(16) == 5080);
  expect("5900 MHz -> ch180", freq_to_chan(5900) == 180);
  expect("ch180 -> 5900 MHz", chan_to_freq(180) == 5900);
  expect("6165 MHz -> ch233", freq_to_chan(6165) == 233);
  expect("6265 MHz -> ch253 (ceiling)", freq_to_chan(6265) == 253);
  expect("6270 MHz dropped (past ceiling)", freq_to_chan(6270) == 0);
  expect("legacy ch36 still 5180", chan_to_freq(36) == 5180);
  expect("2.4G ch6 unaffected", freq_to_chan(2437) == 6);

  /* --- active-RX-path classification --- */
  {
    /* Single strong chain, second at the noise floor -> only chain 0. */
    const int rssi[4] = {-40, -95, 0, 0};
    const bool samp[4] = {true, true, false, false};
    uint8_t mask = 0;
    const uint8_t n = classify_active_paths(rssi, samp, 2, 20, &mask);
    expect("one strong + one deaf -> mask 0x1", mask == 0x1 && n == 1);
  }
  {
    /* Two balanced chains -> both active. */
    const int rssi[4] = {-50, -58, 0, 0};
    const bool samp[4] = {true, true, false, false};
    uint8_t mask = 0;
    const uint8_t n = classify_active_paths(rssi, samp, 2, 20, &mask);
    expect("two balanced -> mask 0x3", mask == 0x3 && n == 2);
  }
  {
    /* No samples -> nothing active. */
    const int rssi[4] = {0, 0, 0, 0};
    const bool samp[4] = {false, false, false, false};
    uint8_t mask = 0xff;
    const uint8_t n = classify_active_paths(rssi, samp, 2, 20, &mask);
    expect("no samples -> mask 0", mask == 0 && n == 0);
  }

  /* --- accumulator drain semantics --- */
  {
    RxPathActivityAccumulator acc;
    const uint8_t f1[4] = {70, 30, 0, 0}; /* raw; chain0 strong, chain1 weak */
    const int8_t snr1[4] = {60, 20, 0, 0}; /* raw dB*2: 30 dB / 10 dB */
    const int8_t evm1[4] = {-52, 0, 0, 0}; /* raw half-dB: -26 dB; chain1 none */
    acc.add(f1, snr1, evm1, 2);
    acc.add(f1, snr1, evm1, 2);
    ActiveRxPaths s = acc.snapshot(20);
    expect("acc valid after frames", s.valid && s.frames == 2);
    expect("acc chain0 active", (s.active_mask & 0x1) != 0);
    expect("acc chain1 inactive (40 raw below)",
           (s.active_mask & 0x2) == 0);
    expect("acc per-chain snr means",
           s.snr_sampled[0] && s.snr_mean_db[0] == 30.0 &&
               s.snr_sampled[1] && s.snr_mean_db[1] == 10.0);
    expect("acc evm chain0 sampled, chain1 not",
           s.evm_sampled[0] && s.evm_mean_db[0] == -26.0 && !s.evm_sampled[1]);
    /* Second snapshot drains to empty (delta semantics). */
    ActiveRxPaths s2 = acc.snapshot(20);
    expect("acc drained on read", !s2.valid && s2.frames == 0);
    expect("acc snr/evm drained", !s2.snr_sampled[0] && !s2.evm_sampled[0]);
  }
  {
    /* EVM -128 is the phy-status no-stream rail — excluded from the mean. */
    RxPathActivityAccumulator acc;
    const uint8_t f1[4] = {70, 60, 0, 0};
    const int8_t snr1[4] = {40, 40, 0, 0};
    const int8_t evm1[4] = {-52, -128, 0, 0};
    acc.add(f1, snr1, evm1, 2);
    ActiveRxPaths s = acc.snapshot(20);
    expect("evm -128 rail not a sample",
           s.evm_sampled[0] && !s.evm_sampled[1]);
  }
  {
    /* Null SNR/EVM pointers (RSSI-only caller) are accepted. */
    RxPathActivityAccumulator acc;
    const uint8_t f1[4] = {70, 60, 0, 0};
    acc.add(f1, nullptr, nullptr, 2);
    ActiveRxPaths s = acc.snapshot(20);
    expect("acc rssi-only add works",
           s.valid && !s.snr_sampled[0] && !s.evm_sampled[0]);
  }

  if (g_fail) {
    std::printf("adapter_caps_selftest: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("adapter_caps_selftest: all checks passed\n");
  return 0;
}
