// tsf_probe.cpp — is the hardware TSF alive, and how jittery is a host-clock
// timestamp vs the per-frame hardware RX TSF? Prints, for each decoded frame,
// the RX-descriptor TSF-low (RxAtrib.tsfl, latched in the MAC at receive) next
// to the host steady_clock time when the callback ran. If tsfl advances ~1 us
// per real us, the TSF is running and usable as a precise timing reference.
//
// Build: g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/tsf_probe.cpp \
//   examples/common/env_config.cpp build/libdevourer.a \
//   $(pkg-config --cflags --libs libusb-1.0) -lpthread -o build/tsf_probe
// Run: sudo DEVOURER_PID=0x8812 DEVOURER_CHANNEL=6 build/tsf_probe [nframes]
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <unistd.h>

#include <libusb.h>

#include "RxPacket.h"
#include "SelectedChannel.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"

static std::atomic<int> g_n{0};
static int g_want = 20;

static int64_t steady_us() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

int main(int argc, char** argv) {
  if (argc > 1) g_want = atoi(argv[1]);
  uint8_t ch = 6;
  if (const char* c = std::getenv("DEVOURER_CHANNEL")) ch = (uint8_t)atoi(c);
  auto logger = std::make_shared<Logger>();
  libusb_context* ctx = nullptr;
  if (libusb_init(&ctx) < 0) return 1;
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
  uint16_t vid = 0x0bda, pid = 0;
  if (const char* v = std::getenv("DEVOURER_VID")) vid = (uint16_t)strtoul(v, 0, 0);
  if (const char* p = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(p, 0, 0);
  auto* h = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!h) { fprintf(stderr, "open %04x:%04x failed\n", vid, pid); return 1; }
  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (devourer::claim_interface_then_reset(h, 0, logger, true, lock) != 0) return 1;
  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRtlDevice(h, ctx, lock, devourer_config_from_env());
  if (!dev) return 1;

  // TSF_REG=1: read the host-facing TSF register standalone (TX bring-up, no RX
  // flood) to tell "register not running in this mode" from "flood race".
  if (std::getenv("TSF_REG")) {
    dev->InitWrite(SelectedChannel{ch, 0, CHANNEL_WIDTH_20});
    std::this_thread::sleep_for(std::chrono::seconds(1));
    for (int i = 0; i < 6; ++i) {
      uint64_t t = dev->ReadTsf();
      printf("ReadTsf() #%d = %llu us\n", i, (unsigned long long)t);
      fflush(stdout);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    _exit(0);
  }

  // Collect (hardware TSF, host steady) pairs; a frame's TSF is latched in the
  // MAC at receive, the host time is when our callback ran.
  std::mutex mu;
  std::vector<double> tsf_us, host_us;
  int64_t tsf_hi = 0, prev_lo = 0;   // rebuild a 64-bit TSF from the 32-bit low
  std::atomic<int> readtsf_ok{0}, readtsf_fail{0};
  std::atomic<long> readtsf_delta{0};   // ReadTsf() - frame.tsfl (should be > 0)
  auto cb = [&](const Packet& p) {
    if (p.RxAtrib.crc_err) return;
    int n = ++g_n;
    if (n > g_want) return;
    uint32_t lo = p.RxAtrib.tsfl;
    // ReadTsf() reliability during the RX flood: a control read racing bulk-IN.
    try {
      uint64_t rt = dev->ReadTsf();
      if (rt) { readtsf_ok++; readtsf_delta.store((long)((uint32_t)(rt - lo))); }
      else readtsf_fail++;
    } catch (...) { readtsf_fail++; }
    if (lo < prev_lo) tsf_hi += (1LL << 32);   // low-word wrap (~71.6 min)
    prev_lo = lo;
    std::lock_guard<std::mutex> lk(mu);
    tsf_us.push_back((double)(tsf_hi + lo));
    host_us.push_back((double)steady_us());
  };
  std::thread rx([&] { dev->Init(cb, SelectedChannel{ch, 0, CHANNEL_WIDTH_20}); });
  for (int i = 0; i < 3000 && g_n.load() < g_want; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // A frame's *true* arrival is the hardware TSF; the host timestamp is that
  // plus USB + scheduling latency. Fit host = a*tsf + b (removes constant offset
  // + the crystal rate difference), and the residual stdev is exactly the host
  // clock's timing jitter relative to the hardware TSF — i.e. the anchor jitter
  // that switching the schedule anchor from host time to TSF would remove.
  std::lock_guard<std::mutex> lk(mu);
  size_t N = tsf_us.size();
  if (N < 8) { fprintf(stderr, "too few frames (%zu)\n", N); _exit(1); }
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (size_t i = 0; i < N; ++i) {
    sx += tsf_us[i]; sy += host_us[i];
    sxx += tsf_us[i] * tsf_us[i]; sxy += tsf_us[i] * host_us[i];
  }
  double a = (N * sxy - sx * sy) / (N * sxx - sx * sx);
  double b = (sy - a * sx) / N;
  double ss = 0, mx = 0;
  for (size_t i = 0; i < N; ++i) {
    double r = host_us[i] - (a * tsf_us[i] + b);
    ss += r * r;
    if (std::fabs(r) > mx) mx = std::fabs(r);
  }
  double sd = std::sqrt(ss / N);
  printf("\n== ReadTsf() during RX flood: %d ok / %d failed (last delta vs "
         "frame.tsfl = %ld us) ==\n", readtsf_ok.load(), readtsf_fail.load(),
         readtsf_delta.load());
  printf("\n== TSF vs host-clock timing jitter (%zu frames, ch %u) ==\n", N, ch);
  printf("  host-clock rate vs TSF : %.6f (1.0 = identical crystals; "
         "%.1f ppm offset)\n", a, (a - 1.0) * 1e6);
  printf("  host-timestamp jitter relative to hardware TSF:\n");
  printf("    stdev = %.1f us    max = %.1f us\n", sd, mx);
  printf("  => anchoring the TDMA schedule on tsfl removes ~this much per-frame\n"
         "     anchor noise (the guard can shrink toward the hardware floor).\n");
  fflush(stdout);
  _exit(0);
}
