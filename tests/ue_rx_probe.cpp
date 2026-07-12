// ue_rx_probe.cpp — on-air probe for the per-UE RX attribution seed
// (src/cell/UeRxAttribution.h): feeds every decoded frame's TA + path-A
// RSSI/SNR/EVM into a UeRxAttribution and drains it once a second, emitting one
// `ue.rx` JSONL event per UE per window — the per-transmitter view that the
// device-wide GetRxQuality (a single draining accumulator) cannot give.
//
// Validation harness: tests/ue_rx_attribution_check.sh runs two transmitters
// with distinct unicast SAs against this probe and asserts both TAs show up
// with sane, separate frame counts.
//
// Build: g++ -std=c++20 -O2 -Isrc -Iexamples/common \
//   tests/ue_rx_probe.cpp examples/common/env_config.cpp build/libdevourer.a \
//   $(pkg-config --cflags --libs libusb-1.0) -lpthread -o build/ue_rx_probe
// Run: sudo DEVOURER_PID=0x012d DEVOURER_CHANNEL=6 build/ue_rx_probe [seconds]
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <thread>
#include <unistd.h>

#include <libusb.h>

#include "RxPacket.h"
#include "SelectedChannel.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "cell/UeRxAttribution.h"
#include "env_config.h"
#include "logger.h"

static devourer::cell::UeRxAttribution g_attr;
static std::atomic<uint64_t> g_frames{0};

int main(int argc, char** argv) {
  int secs = argc > 1 ? atoi(argv[1]) : 15;
  auto logger = std::make_shared<Logger>();
  libusb_context* ctx = nullptr;
  libusb_init(&ctx);
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);

  uint16_t vid = 0x0bda, pid = 0x012d;
  if (const char* p = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(p, 0, 0);
  if (const char* v = std::getenv("DEVOURER_VID")) vid = (uint16_t)strtoul(v, 0, 0);
  uint8_t ch = 6;
  if (const char* c = std::getenv("DEVOURER_CHANNEL")) ch = atoi(c);

  auto* h = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!h) { fprintf(stderr, "open fail %04x:%04x\n", vid, pid); return 1; }
  std::shared_ptr<devourer::UsbDeviceLock> lk;
  if (devourer::claim_interface_then_reset(h, 0, logger, true, lk) != 0) return 1;
  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRtlDevice(h, ctx, lk, devourer_config_from_env());
  if (!dev) return 1;

  auto cb = [](const Packet& p) {
    if (p.RxAtrib.crc_err || p.RxAtrib.icv_err) return;
    if (g_attr.add_mpdu(p.Data.data(), p.Data.size(), p.RxAtrib.rssi[0],
                        p.RxAtrib.snr[0], p.RxAtrib.evm[0], p.RxAtrib.tsfl))
      ++g_frames;
  };

  std::thread rx([&] { dev->Init(cb, SelectedChannel{ch, 0, CHANNEL_WIDTH_20}); });

  for (int t = 0; t < secs; ++t) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto s = g_attr.snapshot();
    for (const auto& w : s.ues) {
      printf("{\"ev\":\"ue.rx\",\"ta\":\"%02x:%02x:%02x:%02x:%02x:%02x\","
             "\"frames\":%u,\"rssi_dbm\":%d,\"rssi_max_dbm\":%d,"
             "\"snr_db\":%.1f,\"snr_min_db\":%.1f,\"evm_db\":%.1f,"
             "\"evm_valid\":%s,\"nf_dbm\":%.1f,\"nf_valid\":%s,"
             "\"last_tsfl\":%u}\n",
             w.ta[0], w.ta[1], w.ta[2], w.ta[3], w.ta[4], w.ta[5], w.frames,
             w.rssi_mean_dbm, w.rssi_max_dbm, w.snr_mean_db, w.snr_min_db,
             w.evm_mean_db, w.evm_valid ? "true" : "false",
             w.noise_floor_dbm, w.nf_valid ? "true" : "false", w.last_tsfl);
    }
    if (s.evicted_frames)
      printf("{\"ev\":\"ue.rx.evicted\",\"frames\":%u}\n", s.evicted_frames);
    fflush(stdout);
  }
  fprintf(stderr, "ue_rx_probe: %llu attributed frames in %d s\n",
          (unsigned long long)g_frames.load(), secs);
  _exit(0);
}
