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
// UPLINK TIMING ADVANCE (DEVOURER_TSYNC_UPLINK=1, roles master + ue) is the LTE
// closed-loop extension — a full-duplex master phase-measures each UE uplink
// against its TSF slot grid and feeds back a timing advance. It is EXPERIMENTAL:
// the control math converges in the headless selftest and the full-duplex
// plumbing works on-air (arrivals cluster tightly, ~±0.2 ms with a FIXED TA),
// but the closed loop does NOT converge on the bench — a fixed-TA authority test
// shows the TA shifts the UE's send-CALL time yet not the master-measured
// arrival phase. Root cause: under full-duplex, send_packet queues the frame and
// the chip airs it on its own schedule, so userspace call-timing has no
// sub-slot control over air departure (plus this bench has only one clean
// full-duplex Jaguar2/3 adapter; the 8822E desenses its RX in TX+RX). See
// docs and tests/timesync_ta_demo.sh.
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
// A minimal raw 802.11 beacon MPDU (canonical SA/BSSID) — byte-identical to the
// bench-validated tests/beacon_tbtt.cpp beacon (which the MAC fills with a clean
// live TSF). The 8-byte timestamp is left zero; the MAC inserts the hardware TSF
// at each TBTT. (An extended body — longer SSID / more rate IEs — broke the
// hardware TSF insertion in testing, so keep this exact layout.)
static std::vector<uint8_t> build_std_beacon(int interval_tu) {
  return {
      0x80, 0x00, 0x00, 0x00,                          // FC beacon + dur
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff,              // addr1 broadcast
      0x57, 0x42, 0x75, 0x05, 0xd6, 0x00,             // addr2 = SA (canonical)
      0x57, 0x42, 0x75, 0x05, 0xd6, 0x00,             // addr3 = BSSID
      0x00, 0x00,                                      // seq
      0, 0, 0, 0, 0, 0, 0, 0,                          // timestamp (HW fills)
      static_cast<uint8_t>(interval_tu & 0xff),
      static_cast<uint8_t>((interval_tu >> 8) & 0xff), // beacon interval
      0x00, 0x00,                                      // capability
      0x00, 0x03, 'T', 'B', 'T',                       // SSID IE
      0x01, 0x01, 0x82};                               // supported rates (1M)
}

static void run_master(IRtlDevice* dev, const timesync::Config& c) {
  dev->InitWrite(SelectedChannel{c.channel, 0, CHANNEL_WIDTH_20});
  sleep_ms(2000);
  if (c.hwbeacon) {
    // Hardware-timed, hardware-TSF-stamped beacon at TBTT — no software send loop,
    // no ReadTsf jitter. The MAC inserts the live TSF into the beacon at TX.
    auto b = build_std_beacon(c.interval_ms > 0 ? c.interval_ms * 1000 / 1024 : 100);
    bool ok = dev->StartBeacon(b.data(), b.size(),
                               c.interval_ms > 0 ? c.interval_ms * 1000 / 1024 : 100);
    fprintf(stderr, "timesync master(HW beacon): StartBeacon -> %s, ch%d\n",
            ok ? "OK" : "UNSUPPORTED", c.channel);
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(c.secs);
    while (!g_devourer_should_stop) {
      sleep_ms(200);
      if (c.secs && std::chrono::steady_clock::now() >= deadline) break;
    }
    return;
  }
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

static bool g_hwbeacon = false;

static void slave_cb(const Packet& p) {
  uint64_t master_tsf; uint32_t seq;
  if (g_hwbeacon) {
    // Standard 802.11 beacon: canonical SA at addr2, and the master's LIVE
    // hardware TSF is the 8-byte timestamp field (MPDU offset 24). No TD tag.
    static const uint8_t kSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
    if (p.Data.size() < 32 || p.RxAtrib.crc_err) return;
    if ((p.Data[0] & 0xfc) != 0x80) return;                        // beacon subtype
    if (std::memcmp(p.Data.data() + 10, kSa, 6) != 0) return;      // our master
    master_tsf = 0;
    for (int i = 0; i < 8; ++i) master_tsf |= (uint64_t)p.Data[24 + i] << (8 * i);
    seq = (uint32_t)(p.RxAtrib.seq_num);
  } else {
    auto pr = tdma::parse_frame(p.Data.data(), p.Data.size());
    if (!pr.ok || p.RxAtrib.crc_err) return;
    if (pr.cls != tdma::Class::Marker || pr.tx_tsf == 0) return;   // sync beacons only
    master_tsf = pr.tx_tsf; seq = pr.seq;
  }

  std::lock_guard<std::mutex> lk(g_mu);
  double local_us = (double)g_recon(p.RxAtrib.tsfl);
  double master_us = (double)master_tsf;
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
                  seq, (unsigned long long)master_tsf,
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

// --- UPLINK TIMING ADVANCE (LTE TA), full-duplex ---------------------------
// The master broadcasts beacons AND phase-measures each UE uplink against its
// own TSF slot grid (arrival tsfl mod slot_us — reliable per-frame, no ReadTsf
// which the RX loop would starve), integrating a per-UE timing-advance it feeds
// back. The UE schedules its uplinks off the beacon-arrival cadence (a seq↔host
// fit — rate-locked to the master) minus the TA. The loop drives each uplink's
// arrival onto the master's slot boundary; open-loop it drifts across the slot
// at the crystal offset. Both nodes are full-duplex (InitWrite + StartRxLoop).
static int64_t steady_us() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Master TA state.
static timesync::Recon g_m_recon;
static std::atomic<double> g_ta_us{0};
static std::mutex g_m_mu;
static uint64_t g_uplinks = 0;
static double g_phase_ss = 0, g_phase_max = 0;
static uint64_t g_phase_n = 0;
static double g_slot_us = 20000, g_ta_gain = 0.3;
static bool g_ta_fixed = false;   // DEVOURER_TSYNC_TA_FIXED: hold TA constant (authority test)

static void master_ta_cb(const Packet& p) {
  auto pr = tdma::parse_frame(p.Data.data(), p.Data.size());
  if (!pr.ok || p.RxAtrib.crc_err) return;
  if (static_cast<uint8_t>(pr.cls) != timesync::kClassUplink) return;
  std::lock_guard<std::mutex> lk(g_m_mu);
  double arrival = (double)g_m_recon(p.RxAtrib.tsfl);
  double phase = arrival - std::round(arrival / g_slot_us) * g_slot_us;  // (-slot/2, slot/2]
  double ta = g_ta_us.load();
  if (!g_ta_fixed) {
    ta += g_ta_gain * phase;                       // late (phase>0) → more TA → UE earlier
    if (ta > g_slot_us) ta = g_slot_us;            // clamp to one slot (no wrap — a hard
    if (ta < -g_slot_us) ta = -g_slot_us;          // mod jump kicks the loop)
    g_ta_us.store(ta);
  }
  ++g_uplinks; g_phase_ss += phase * phase; ++g_phase_n;
  if (std::fabs(phase) > g_phase_max) g_phase_max = std::fabs(phase);
  char buf[224];
  std::snprintf(buf, sizeof(buf),
                "{\"ev\":\"timesync.ta\",\"seq\":%u,\"n\":%llu,\"phase_us\":%.2f,"
                "\"ta_us\":%.2f}\n",
                pr.seq, (unsigned long long)g_uplinks, phase, ta);
  emit(buf);
}

static void run_master_ta(IRtlDevice* dev, const timesync::Config& c) {
  g_slot_us = c.slot_ms * 1000.0; g_ta_gain = c.ta_gain;
  if (const char* f = std::getenv("DEVOURER_TSYNC_TA_FIXED")) {
    g_ta_fixed = true; g_ta_us.store(std::atof(f));   // authority test: hold TA constant
  }
  dev->InitWrite(SelectedChannel{c.channel, 0, CHANNEL_WIDTH_20});
  std::thread rx([&] { dev->StartRxLoop(master_ta_cb); });
  sleep_ms(2000);
  const auto rt = devourer::build_stream_radiotap(c.rate);
  fprintf(stderr, "timesync master(TA): ch%d beacons=%dms slot=%dms gain=%.2f\n",
          c.channel, c.interval_ms, c.slot_ms, c.ta_gain);

  uint32_t seq = 0;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(c.secs);
  while (!g_devourer_should_stop) {
    // The beacon carries the current TA in its tx_tsf field (unused for cadence),
    // so every beacon the UE decodes delivers the latest TA — one send per slot,
    // no separate frame to drop.
    int64_t ta_i = (int64_t)std::llround(g_ta_us.load());  // signed µs → 8-byte tag
    uint64_t ta_b; std::memcpy(&ta_b, &ta_i, 8);
    auto b = tdma::build_frame(rt, tdma::Class::Marker, seq++, 0, ta_b);
    dev->send_packet(b.data(), b.size());
    if (c.secs && std::chrono::steady_clock::now() >= deadline) break;
    sleep_ms(c.interval_ms);
  }
  dev->StopRxLoop(); rx.join();
  std::lock_guard<std::mutex> lk(g_m_mu);
  double rms = g_phase_n ? std::sqrt(g_phase_ss / (double)g_phase_n) : 0;
  fprintf(stderr,
          "\n=== timesync master(TA) summary ===\n"
          "  uplinks measured   : %llu\n"
          "  arrival phase (all): RMS %.2f us   max %.2f us\n"
          "  final TA           : %.2f us\n",
          (unsigned long long)g_uplinks, rms, g_phase_max, g_ta_us.load());
}

// UE state. Event-driven off actual beacon arrivals (rate-locked to the master),
// so the TA directly and unambiguously shifts the uplink's arrival phase.
static std::atomic<int64_t> g_ue_beacon_us{0};   // steady_us of the last beacon
static std::atomic<uint32_t> g_ue_beacon_seq{0};
static std::atomic<bool> g_ue_have{false};
static std::atomic<double> g_ue_ta_us{0};
static std::atomic<uint64_t> g_ue_beacons{0}, g_ue_tx{0};

static void ue_cb(const Packet& p) {
  auto pr = tdma::parse_frame(p.Data.data(), p.Data.size());
  if (!pr.ok || p.RxAtrib.crc_err) return;
  if (pr.cls == tdma::Class::Marker) {
    g_ue_beacon_us.store(steady_us(), std::memory_order_relaxed);
    g_ue_beacon_seq.store(pr.seq, std::memory_order_relaxed);
    g_ue_have.store(true, std::memory_order_relaxed);
    g_ue_beacons.fetch_add(1, std::memory_order_relaxed);
    int64_t ta_i; uint64_t ta_b = pr.tx_tsf; std::memcpy(&ta_i, &ta_b, 8);  // TA rides the beacon
    g_ue_ta_us.store((double)ta_i, std::memory_order_relaxed);
  }
}

static void run_ue(IRtlDevice* dev, const timesync::Config& c) {
  dev->InitWrite(SelectedChannel{c.channel, 0, CHANNEL_WIDTH_20});
  std::thread rx([&] { dev->StartRxLoop(ue_cb); });
  sleep_ms(2000);
  const auto rt = devourer::build_stream_radiotap(c.rate);
  const int64_t slot_us = (int64_t)c.slot_ms * 1000;
  fprintf(stderr, "timesync ue: ch%d, uplink one frame per beacon (TA-corrected)\n",
          c.channel);

  uint32_t done = 0;   // last beacon seq we've already answered with an uplink
  auto next_stat = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(c.secs);
  while (!g_devourer_should_stop) {
    if (std::chrono::steady_clock::now() >= next_stat) {
      next_stat += std::chrono::seconds(1);
      char buf[160];
      std::snprintf(buf, sizeof(buf),
                    "{\"ev\":\"timesync.ue\",\"beacons\":%llu,\"tx\":%llu,\"ta_us\":%.1f}\n",
                    (unsigned long long)g_ue_beacons.load(),
                    (unsigned long long)g_ue_tx.load(), g_ue_ta_us.load());
      emit(buf);
    }
    if (!g_ue_have.load(std::memory_order_relaxed)) { sleep_ms(5); continue; }
    uint32_t s = g_ue_beacon_seq.load(std::memory_order_relaxed);
    if (s == done) { sleep_ms(1); continue; }   // wait for the next beacon (a slot tick)
    done = s;
    // Aim the uplink to ARRIVE one slot after this beacon (the next boundary),
    // advanced by the master's TA. TA authority is direct: earlier send → earlier
    // arrival → smaller measured phase.
    int64_t send_at = g_ue_beacon_us.load(std::memory_order_relaxed) + slot_us -
                      (int64_t)g_ue_ta_us.load(std::memory_order_relaxed);
    int64_t wait = send_at - steady_us();
    if (wait > 0 && wait < 2 * slot_us)
      std::this_thread::sleep_for(std::chrono::microseconds(wait));
    else if (wait >= 2 * slot_us)
      continue;   // absurd — skip this slot
    auto f = tdma::build_frame(rt, static_cast<tdma::Class>(timesync::kClassUplink),
                               s, 0, 0);
    dev->send_packet(f.data(), f.size());
    g_ue_tx.fetch_add(1, std::memory_order_relaxed);
    if (c.secs && std::chrono::steady_clock::now() >= deadline) break;
  }
  dev->StopRxLoop(); rx.join();
  fprintf(stderr,
          "\n=== timesync ue summary ===\n"
          "  beacons heard : %llu\n"
          "  uplinks sent  : %llu\n"
          "  final TA      : %.2f us\n",
          (unsigned long long)g_ue_beacons.load(),
          (unsigned long long)g_ue_tx.load(), g_ue_ta_us.load());
}

int main() {
  auto logger = std::make_shared<Logger>();
  apply_logging_env(*logger);
  install_devourer_signal_handlers();

  timesync::Config c = timesync::config_from_env();
  g_hwbeacon = c.hwbeacon;   // slave reads the standard 802.11 beacon timestamp

  libusb_context* ctx = nullptr;
  std::shared_ptr<devourer::UsbDeviceLock> lock;
  auto* handle = open_device(logger, &ctx, lock);
  if (!handle) { if (ctx) libusb_exit(ctx); return 1; }

  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRtlDevice(handle, ctx, lock, devourer_config_from_env());
  if (!dev) { logger->error("no driver for this chip"); return 1; }

  if (c.role == timesync::Role::Ue) run_ue(dev.get(), c);
  else if (c.role == timesync::Role::Master && c.uplink) run_master_ta(dev.get(), c);
  else if (c.role == timesync::Role::Master) run_master(dev.get(), c);
  else run_slave(dev.get(), c);
  return 0;
}
