// nb_toggle_rxcheck.cpp — does EVERY switch into narrowband decode, not just
// most of them? A partner transmits the canonical beacon in 5 or 10 MHz; the
// RX device switches 20 MHz -> narrowband -> 20 MHz repeatedly and counts
// canonical-SA hits in each narrowband window. One bring-up, many switches,
// because the 8822B fault this guards against was a per-switch lottery: a
// baseband reset placed after the RF18 re-latch edge left the receiver with no
// in-band power at all in most windows while the registers read back correct.
//
// Modes: fast = FastSetBandwidth, full = SetMonitorChannel, init = bring the
// device up directly in narrowband (one window per run; repeat the run).
// A window passes with more than 50 hits. Exit status: 0 all windows passed,
// 1 at least one failed, 2 could not run.
//
// Build (one line):
//   g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/nb_toggle_rxcheck.cpp examples/common/env_config.cpp build/libdevourer.a $(pkg-config --cflags --libs libusb-1.0) -lpthread -o build/nb_toggle_rxcheck
// Run, with a partner already transmitting (txdemo, DEVOURER_NB_BW=10):
//   sudo DEVOURER_VID=0x0bda DEVOURER_PID=0xb812 DEVOURER_CHANNEL=36 build/nb_toggle_rxcheck <fast|full|init> <5|10> [switches=8]
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <thread>

#include <libusb.h>

#include "RxPacket.h"
#include "SelectedChannel.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"

static const uint8_t kSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
static std::atomic<uint64_t> g_hits{0};
static std::atomic<bool> g_rx_live{false};

int main(int argc, char **argv) {
  /* A mistyped argument must not quietly test something else, or nothing:
   * every one of them is checked before the device is opened. */
  const char *mode = argc > 1 ? argv[1] : "";
  const bool fast = !strcmp(mode, "fast"), init = !strcmp(mode, "init");
  const int mhz = argc > 2 ? atoi(argv[2]) : 0;
  const int n = init ? 1 : (argc > 3 ? atoi(argv[3]) : 8);
  if ((!fast && !init && strcmp(mode, "full")) || (mhz != 5 && mhz != 10) ||
      n < 1) {
    fprintf(stderr, "usage: %s <fast|full|init> <5|10> [switches >= 1, "
                    "default 8]\n", argv[0]);
    return 2;
  }
  const ChannelWidth_t nb = mhz == 5 ? CHANNEL_WIDTH_5 : CHANNEL_WIDTH_10;
  uint8_t ch = 36;
  if (const char *c = std::getenv("DEVOURER_CHANNEL")) ch = (uint8_t)atoi(c);

  auto logger = std::make_shared<Logger>();
  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) < 0) return 2;
  uint16_t vid = 0x0bda, pid = 0;
  if (const char *e = std::getenv("DEVOURER_VID")) vid = (uint16_t)strtoul(e, 0, 0);
  if (const char *e = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(e, 0, 0);
  auto *h = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!h) { fprintf(stderr, "open %04x:%04x failed\n", vid, pid); return 2; }
  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (devourer::claim_interface_then_reset(h, devourer::find_wifi_interface(h), logger, true, lock) != 0) return 2;
  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRadio(h, ctx, lock, devourer_config_from_env());
  if (!dev) return 2;

  auto cb = [](const Packet &p) {
    if (p.RxAtrib.crc_err || p.Data.size() < 16)
      return;
    g_rx_live.store(true, std::memory_order_relaxed);
    if (std::memcmp(p.Data.data() + 10, kSa, 6) == 0)
      g_hits.fetch_add(1, std::memory_order_relaxed);
  };
  const SelectedChannel at20{ch, 0, CHANNEL_WIDTH_20}, at_nb{ch, 0, nb};
  std::atomic<bool> rx_done{false};
  std::thread rx([&] {
    dev->Init(cb, init ? at_nb : at20);
    rx_done = true;
  });
  auto shutdown = [&] {
    while (!rx_done) {
      dev->StopRxLoop();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    rx.join();
    dev->Stop();
  };
  auto hits_over = [](int ms) {
    g_hits.store(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    return (unsigned long long)g_hits.load();
  };

  int failed = 0;
  printf("%s %d MHz:", mode, mhz);
  if (init) {
    /* No ambient traffic decodes in narrowband, so there is no frame to wait
     * on when the window is dead: give bring-up a fixed allowance instead. */
    std::this_thread::sleep_for(std::chrono::seconds(4));
    const auto hits = hits_over(1500);
    printf(" %llu", hits);
    failed += hits <= 50;
  } else {
    /* Bring-up must have finished before the first switch; the first decoded
     * 20 MHz frame proves it. */
    for (int i = 0; i < 150 && !g_rx_live && !rx_done; i++)
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!g_rx_live) {
      printf(" no frame decoded at 20 MHz within 15 s of bring-up\n");
      shutdown();
      return 2;
    }
    for (int i = 0; i < n; i++) {
      if (fast) dev->FastSetBandwidth(nb);
      else dev->SetMonitorChannel(at_nb);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      const auto hits = hits_over(1300);
      printf(" %llu", hits);
      fflush(stdout);
      failed += hits <= 50;
      if (fast) dev->FastSetBandwidth(CHANNEL_WIDTH_20);
      else dev->SetMonitorChannel(at20);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }
  printf("\n");
  shutdown();
  if (failed)
    printf("verdict: FAIL — %d of %d narrowband windows decoded nothing "
           "(check the partner is transmitting that width)\n", failed, n);
  else
    printf("verdict: PASS — all %d narrowband windows decoded\n", n);
  return failed ? 1 : 0;
}
