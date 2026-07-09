// timesync — LTE-eNB-style over-the-air time distribution. One binary, one
// adapter, one role (compose scenarios by running instances):
//
//   DEVOURER_TSYNC_ROLE=master   TX-only. Every DEVOURER_TSYNC_INTERVAL_MS,
//                                broadcast a sync beacon stamped with the chip's
//                                hardware TSF (ReadTsf(), reliable TX-side — no
//                                RX flood starving the control read). This is the
//                                eNB distributing its SFN.
//   DEVOURER_TSYNC_ROLE=slave    RX-only. Lock a running fit of the master's
//                                broadcast TSF against this slave's own per-frame
//                                hardware TSF, PREDICT each beacon before it
//                                arrives, and emit the prediction error — how
//                                tightly this UE tracks the eNB. No host clock,
//                                no GPS.
//
// Run a master + two slaves and join the slaves' {"ev":"timesync.lock"} streams
// on `seq`: pred_master_A vs pred_master_B is the inter-UE sync error, measured
// without either slave touching a wall clock (tests/timesync_demo.sh).
//
// See timesync.h for the fit + env knobs. Metrics are JSONL on stdout.
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#if defined(_MSC_VER)
  #include <libusb.h>
#elif defined(__MINGW32__) || defined(__MINGW64__)
  #include <libusb-1.0/libusb.h>
#elif defined(__APPLE__) || defined(__ANDROID__)
  #include <libusb.h>
#else
  #include <libusb-1.0/libusb.h>
#endif

#include "RadiotapBuilder.h"
#include "RxPacket.h"
#include "SignalStop.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"
#include "timesync.h"

#define USB_VENDOR_ID 0x0bda
static constexpr uint16_t kRealtekProductIds[] = {
    0x8812, 0x0811, 0xa811, 0xb811, 0x8813,
};

static inline void sleep_ms(long ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
static void emit(const char* json) { std::fputs(json, stdout); std::fflush(stdout); }

// --- device open (mirrors examples/tdma/main.cpp) ---------------------------
static libusb_device_handle* open_device(
    const std::shared_ptr<Logger>& logger, libusb_context** ctx,
    std::shared_ptr<devourer::UsbDeviceLock>& lock) {
  if (libusb_init(ctx) < 0) return nullptr;
  libusb_set_option(*ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
  uint16_t vid = USB_VENDOR_ID, pid = 0;
  if (const char* v = std::getenv("DEVOURER_VID")) vid = (uint16_t)strtoul(v, 0, 0);
  if (const char* p = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(p, 0, 0);
  libusb_device_handle* h = nullptr;
  for (uint16_t p : kRealtekProductIds) {
    if (pid != 0 && p != pid) continue;
    h = libusb_open_device_with_vid_pid(*ctx, vid, p);
    if (h) break;
  }
  if (!h && pid != 0) h = libusb_open_device_with_vid_pid(*ctx, vid, pid);
  if (!h) { logger->error("no device {:04x}:{:04x}", vid, pid); return nullptr; }
  if (devourer::claim_interface_then_reset(
          h, 0, logger, std::getenv("DEVOURER_SKIP_RESET") == nullptr, lock) != 0) {
    logger->error("claim failed (busy?)");
    return nullptr;
  }
  return h;
}

// --- MASTER (eNB): broadcast the hardware TSF -------------------------------
static void run_master(IRtlDevice* dev, const timesync::Config& c) {
  dev->InitWrite(SelectedChannel{c.channel, 0, CHANNEL_WIDTH_20});
  sleep_ms(2000);
  const auto rt = devourer::build_stream_radiotap(c.rate);
  fprintf(stderr, "timesync master: ch%d, sync beacon every %d ms\n", c.channel,
          c.interval_ms);

  uint32_t seq = 0;
  auto next_stat = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(c.secs);
  while (!g_devourer_should_stop) {
    // Stamp with the master's hardware TSF at send time. TX-side ReadTsf() is
    // reliable (no bulk-IN flood), unlike on a busy receiver.
    uint64_t tsf = dev->ReadTsf();
    auto f = tdma::build_frame(rt, tdma::Class::Marker, seq++, 0, tsf);
    dev->send_packet(f.data(), f.size());

    if (std::chrono::steady_clock::now() >= next_stat) {
      next_stat += std::chrono::seconds(1);
      char buf[160];
      std::snprintf(buf, sizeof(buf),
                    "{\"ev\":\"timesync.master\",\"beacons\":%u,\"tsf\":%llu}\n",
                    seq, (unsigned long long)tsf);
      emit(buf);
    }
    if (c.secs && std::chrono::steady_clock::now() >= deadline) break;
    sleep_ms(c.interval_ms);
  }
  fprintf(stderr, "timesync master: %u beacons sent\n", seq);
}

// --- SLAVE (UE): lock to the master, predict each beacon --------------------
static timesync::Recon g_recon;
static timesync::LinFit g_fit;
static std::mutex g_mu;
static uint64_t g_beacons = 0;       // master frames heard
static uint64_t g_predicted = 0;     // frames predicted (fit was ready)
static double g_resid_ss = 0;        // Σ resid² (µs²), for RMS
static double g_resid_max = 0;

static void slave_cb(const Packet& p) {
  auto pr = tdma::parse_frame(p.Data.data(), p.Data.size());
  if (!pr.ok || p.RxAtrib.crc_err) return;
  if (pr.cls != tdma::Class::Marker || pr.tx_tsf == 0) return;  // sync beacons only

  std::lock_guard<std::mutex> lk(g_mu);
  double local_us = (double)g_recon(p.RxAtrib.tsfl);
  double master_us = (double)pr.tx_tsf;
  ++g_beacons;

  // Predict this beacon's master TSF from the fit built on PRIOR beacons,
  // evaluated at this beacon's clean local hardware TSF. resid = lock error.
  if (g_fit.ready()) {
    double pred = g_fit.at(local_us);
    double resid = master_us - pred;
    ++g_predicted;
    g_resid_ss += resid * resid;
    if (std::fabs(resid) > g_resid_max) g_resid_max = std::fabs(resid);
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "{\"ev\":\"timesync.lock\",\"seq\":%u,\"master_tsf\":%llu,"
                  "\"local_tsf\":%llu,\"pred_master\":%.1f,\"resid_us\":%.2f,"
                  "\"ppm\":%.2f}\n",
                  pr.seq, (unsigned long long)pr.tx_tsf,
                  (unsigned long long)(int64_t)local_us, pred, resid, g_fit.ppm());
    emit(buf);
  }
  g_fit.add(local_us, master_us);
}

static void run_slave(IRtlDevice* dev, const timesync::Config& c) {
  std::thread rx([&] {
    dev->Init(slave_cb, SelectedChannel{c.channel, 0, CHANNEL_WIDTH_20});
  });
  fprintf(stderr, "timesync slave: ch%d, locking to master beacons\n", c.channel);

  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(c.secs);
  while (!g_devourer_should_stop) {
    sleep_ms(100);
    if (c.secs && std::chrono::steady_clock::now() >= deadline) break;
  }
  dev->StopRxLoop();
  rx.join();

  std::lock_guard<std::mutex> lk(g_mu);
  double rms = g_predicted ? std::sqrt(g_resid_ss / (double)g_predicted) : 0;
  fprintf(stderr,
          "\n=== timesync slave summary ===\n"
          "  beacons heard      : %llu\n"
          "  predicted          : %llu\n"
          "  lock error (RMS)   : %.2f us    max %.2f us\n"
          "  master-vs-slave ppm: %.2f\n",
          (unsigned long long)g_beacons, (unsigned long long)g_predicted, rms,
          g_resid_max, g_fit.ppm());
}

int main() {
  auto logger = std::make_shared<Logger>();
  apply_logging_env(*logger);
  install_devourer_signal_handlers();

  timesync::Config c = timesync::config_from_env();

  libusb_context* ctx = nullptr;
  std::shared_ptr<devourer::UsbDeviceLock> lock;
  auto* handle = open_device(logger, &ctx, lock);
  if (!handle) { if (ctx) libusb_exit(ctx); return 1; }

  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRtlDevice(handle, ctx, lock, devourer_config_from_env());
  if (!dev) { logger->error("no driver for this chip"); return 1; }

  if (c.role == timesync::Role::Master) run_master(dev.get(), c);
  else run_slave(dev.get(), c);
  return 0;
}
