// retune_bench.cpp — measure the wall-clock cost of a runtime bandwidth switch
// (20 MHz <-> 5 MHz narrowband) on a plugged adapter. Throwaway measurement
// harness: brings the chip up at 20 MHz, then times N alternating
// SetMonitorChannel() calls between 20 MHz and 5 MHz, reporting min/median/max
// per direction in microseconds.
//
// Build (against the already-built static lib):
//   g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/retune_bench.cpp \
//       examples/common/env_config.cpp build/libdevourer.a \
//       $(pkg-config --cflags --libs libusb-1.0) -lpthread -o build/retune_bench
// Run: sudo DEVOURER_PID=0x8812 DEVOURER_CHANNEL=36 build/retune_bench [reps]
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>

#include <libusb.h>

#include "SelectedChannel.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"

using clk = std::chrono::steady_clock;

static double us(clk::time_point a, clk::time_point b) {
  return std::chrono::duration<double, std::micro>(b - a).count();
}

static void report(const char *label, std::vector<double> &v) {
  std::sort(v.begin(), v.end());
  double sum = 0;
  for (double x : v) sum += x;
  printf("  %-14s n=%zu  min=%.0f  median=%.0f  mean=%.0f  max=%.0f us\n",
         label, v.size(), v.front(), v[v.size() / 2], sum / v.size(), v.back());
}

int main(int argc, char **argv) {
  const int reps = argc > 1 ? atoi(argv[1]) : 20;
  uint8_t ch = 36;
  if (const char *c = std::getenv("DEVOURER_CHANNEL")) ch = (uint8_t)atoi(c);

  auto logger = std::make_shared<Logger>();
  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) < 0) { fprintf(stderr, "libusb_init failed\n"); return 1; }
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);

  uint16_t vid = 0x0bda, pid = 0;
  if (const char *e = std::getenv("DEVOURER_VID")) vid = (uint16_t)strtoul(e, nullptr, 0);
  if (const char *e = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(e, nullptr, 0);
  libusb_device_handle *h = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!h) { fprintf(stderr, "open %04x:%04x failed\n", vid, pid); return 1; }

  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (devourer::claim_interface_then_reset(
          h, 0, logger, std::getenv("DEVOURER_SKIP_RESET") == nullptr, lock) != 0) {
    fprintf(stderr, "claim failed (adapter busy?)\n");
    return 1;
  }

  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRtlDevice(h, ctx, lock, devourer_config_from_env());
  if (!dev) { fprintf(stderr, "no driver for this chip\n"); return 1; }

  // Bring up TX at 20 MHz on the target channel.
  dev->InitWrite(SelectedChannel{ch, 0, CHANNEL_WIDTH_20});

  auto to20 = SelectedChannel{ch, 0, CHANNEL_WIDTH_20};
  auto to5  = SelectedChannel{ch, 0, CHANNEL_WIDTH_5};

  // Parity mode: prove the fast path leaves 0x8ac bit-identical to the full
  // path. Run with DEVOURER_DUMP_CANARY=1; the "0x8ac" canary line printed
  // after each PARITY marker is the value to compare.
  if (std::getenv("PARITY")) {
    const ChannelWidth_t widths[] = {CHANNEL_WIDTH_5, CHANNEL_WIDTH_10,
                                     CHANNEL_WIDTH_20};
    for (ChannelWidth_t w : widths) {
      int mhz = w == CHANNEL_WIDTH_5 ? 5 : w == CHANNEL_WIDTH_10 ? 10 : 20;
      fprintf(stderr, "PARITY_MARK full %d\n", mhz);
      dev->SetMonitorChannel(SelectedChannel{ch, 0, w});
      dev->SetMonitorChannel(to20);        // reset to 20 for the fast run
      fprintf(stderr, "PARITY_MARK fast %d\n", mhz);
      dev->FastSetBandwidth(w);
      dev->SetMonitorChannel(to20);        // reset (full) between cases
    }
    return 0;
  }

  std::vector<double> t_205, t_520, f_205, f_520;
  // Warm one full cycle first (first switch pays one-time cache fills).
  dev->SetMonitorChannel(to5);
  dev->SetMonitorChannel(to20);

  // Full-path SetMonitorChannel (the status quo).
  for (int i = 0; i < reps; ++i) {
    auto a = clk::now(); dev->SetMonitorChannel(to5);  auto b = clk::now();
    dev->SetMonitorChannel(to20);                      auto c = clk::now();
    t_205.push_back(us(a, b));
    t_520.push_back(us(b, c));
  }

  // Fast path (FastSetBandwidth). Prime the compose-cache with one warm cycle.
  dev->FastSetBandwidth(CHANNEL_WIDTH_5);
  dev->FastSetBandwidth(CHANNEL_WIDTH_20);
  for (int i = 0; i < reps; ++i) {
    auto a = clk::now(); dev->FastSetBandwidth(CHANNEL_WIDTH_5);  auto b = clk::now();
    dev->FastSetBandwidth(CHANNEL_WIDTH_20);                      auto c = clk::now();
    f_205.push_back(us(a, b));
    f_520.push_back(us(b, c));
  }

  printf("\n== bandwidth-switch timing on ch %u (%d reps) ==\n", ch, reps);
  printf(" full SetMonitorChannel:\n");
  report("20 -> 5 MHz", t_205);
  report("5 -> 20 MHz", t_520);
  printf(" fast FastSetBandwidth:\n");
  report("20 -> 5 MHz", f_205);
  report("5 -> 20 MHz", f_520);
  return 0;
}
