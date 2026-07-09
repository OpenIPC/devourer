// tsf_tdoa_probe.cpp — multi-radio TSF correlation: two receivers hear the same
// transmitter, and we relate their independent hardware TSF clocks from frames
// they BOTH decode. Uses only the per-frame RX-descriptor tsfl (reliable during
// RX, unlike a register ReadTsf which the bulk-IN flood starves), so the
// measurement is clean on both ends.
//
// For each frame heard by both RXs (matched by the tdma TD tag's class+seq), we
// take Δ = tsfl_A − tsfl_B. Δ = (crystal-rate difference)·t + (constant offset)
// + (propagation-delay difference). Fitting Δ vs host time recovers the
// inter-receiver crystal DRIFT (ppm) and, from the residual, how tightly the two
// hardware timestamps track. That residual is the noise floor a TDOA solver
// would work against — though at 1 µs TSF resolution real geometry needs
// hundreds-of-metres baselines (30 cm ≈ 1 ns, far below a µs), so a bench shows
// the CLOCK RELATIONSHIP, not localization.
//
// Build: g++ -std=c++20 -O2 -Isrc -Iexamples/common -Iexamples/tdma \
//   tests/tsf_tdoa_probe.cpp examples/common/env_config.cpp build/libdevourer.a \
//   $(pkg-config --cflags --libs libusb-1.0) -lpthread -o build/tsf_tdoa_probe
// Run: sudo DEVOURER_CHANNEL=36 build/tsf_tdoa_probe <VID_A:PID_A> <VID_B:PID_B> [nmatch]
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <string>
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
#include "tdma.h"

static int g_want = 400;
static uint8_t g_ch = 36;
static std::atomic<bool> g_done{false};

static double steady_us() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Per-receiver 32→64-bit TSF reconstruction (each owned by one RX thread).
struct Recon { int64_t hi = 0; uint32_t plo = 0; bool init = false;
  int64_t operator()(uint32_t lo) {
    if (init && lo < plo) hi += (1LL << 32);
    plo = lo; init = true; return hi + lo;
  }
};

// Shared: frames seen by only one RX so far, keyed by (class,seq); and the
// matched Δ samples.
static std::mutex g_mu;
static std::map<uint64_t, std::pair<int, int64_t>> g_pending;  // key -> (rx, tsf)
static std::vector<double> g_host, g_delta;                    // matched samples

static libusb_device_handle* open_pid(libusb_context* ctx, uint16_t vid,
                                       uint16_t pid, const std::shared_ptr<Logger>& lg,
                                       std::shared_ptr<devourer::UsbDeviceLock>& lk) {
  auto* h = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!h) { fprintf(stderr, "open %04x:%04x failed\n", vid, pid); return nullptr; }
  if (devourer::claim_interface_then_reset(h, 0, lg, true, lk) != 0) {
    fprintf(stderr, "claim %04x:%04x failed\n", vid, pid); return nullptr;
  }
  return h;
}

static void run_rx(IRtlDevice* dev, int idx) {
  Recon recon;
  auto cb = [&, idx](const Packet& p) {
    auto pr = tdma::parse_frame(p.Data.data(), p.Data.size());
    if (!pr.ok || p.RxAtrib.crc_err) return;
    int64_t tsf = recon(p.RxAtrib.tsfl);
    uint64_t key = ((uint64_t)pr.cls << 32) | pr.seq;
    std::lock_guard<std::mutex> lk(g_mu);
    auto it = g_pending.find(key);
    if (it == g_pending.end()) {
      if (g_pending.size() > 20000) g_pending.clear();  // drop stale singletons
      g_pending[key] = {idx, tsf};
    } else if (it->second.first != idx) {               // the OTHER RX had it
      int64_t a = idx == 0 ? tsf : it->second.second;
      int64_t b = idx == 0 ? it->second.second : tsf;
      g_host.push_back(steady_us());
      g_delta.push_back((double)(a - b));
      g_pending.erase(it);
      if ((int)g_delta.size() >= g_want) g_done.store(true);
    }
  };
  dev->Init(cb, SelectedChannel{g_ch, 0, CHANNEL_WIDTH_20});
}

int main(int argc, char** argv) {
  if (argc < 3) { fprintf(stderr, "usage: %s <vidA:pidA> <vidB:pidB> [nmatch]\n", argv[0]); return 2; }
  auto parse = [](const char* s, uint16_t& v, uint16_t& p) {
    const char* c = strchr(s, ':');
    v = (uint16_t)strtoul(s, nullptr, 0); p = (uint16_t)strtoul(c ? c + 1 : s, nullptr, 0);
  };
  uint16_t vA, pA, vB, pB; parse(argv[1], vA, pA); parse(argv[2], vB, pB);
  if (argc > 3) g_want = atoi(argv[3]);
  if (const char* c = std::getenv("DEVOURER_CHANNEL")) g_ch = (uint8_t)atoi(c);

  auto logger = std::make_shared<Logger>();
  libusb_context* ctx = nullptr;
  if (libusb_init(&ctx) < 0) return 1;
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
  std::shared_ptr<devourer::UsbDeviceLock> lkA, lkB;
  auto* hA = open_pid(ctx, vA, pA, logger, lkA);
  auto* hB = open_pid(ctx, vB, pB, logger, lkB);
  if (!hA || !hB) return 1;
  WiFiDriver wifi(logger);
  auto devA = wifi.CreateRtlDevice(hA, ctx, lkA, devourer_config_from_env());
  auto devB = wifi.CreateRtlDevice(hB, ctx, lkB, devourer_config_from_env());
  if (!devA || !devB) { fprintf(stderr, "device create failed\n"); return 1; }

  std::thread tA([&] { run_rx(devA.get(), 0); });
  std::thread tB([&] { run_rx(devB.get(), 1); });
  for (int i = 0; i < 4000 && !g_done.load(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  std::lock_guard<std::mutex> lk(g_mu);
  size_t N = g_delta.size();
  if (N < 16) { fprintf(stderr, "too few common frames (%zu) — are both RXs in range?\n", N); _exit(1); }
  // Fit Δ = a·host + b (host in µs). a = crystal-rate difference (ppm = a·1e6);
  // residual = how tightly the two hardware timestamps track.
  double x0 = g_host[0], y0 = g_delta[0], sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (size_t i = 0; i < N; ++i) {
    double xi = g_host[i] - x0, yi = g_delta[i] - y0;
    sx += xi; sy += yi; sxx += xi * xi; sxy += xi * yi;
  }
  double den = N * sxx - sx * sx;
  double a = den ? (N * sxy - sx * sy) / den : 0, b = (sy - a * sx) / N;
  double ss = 0, mx = 0;
  for (size_t i = 0; i < N; ++i) {
    double r = (g_delta[i] - y0) - (a * (g_host[i] - x0) + b);
    ss += r * r; if (std::fabs(r) > mx) mx = std::fabs(r);
  }
  printf("\n== dual-RX TSF correlation (%zu common frames, ch %u) ==\n", N, g_ch);
  printf("  inter-receiver crystal drift : %.1f ppm\n", a * 1e6);
  printf("  timestamp-tracking residual  : stdev %.2f us   max %.2f us\n",
         std::sqrt(ss / N), mx);
  printf("  (residual = the TDOA noise floor; real geometry needs baselines\n"
         "   where propagation delay >> this, i.e. hundreds of metres at 1 µs TSF.)\n");
  fflush(stdout);
  _exit(0);
}
