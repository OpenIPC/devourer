// tdma — burst-level bandwidth TDMA demo (see docs/narrowband.md).
//
// One binary, one adapter, one role (compose scenarios by running instances):
//   DEVOURER_TDMA_ROLE=tx        alternate narrowband/wide BURSTS, injecting
//                                critical frames in the narrowband burst and
//                                bulk frames in the wide burst, flipping
//                                bandwidth with FastSetBandwidth.
//   DEVOURER_TDMA_ROLE=rx-sync   switch bandwidth in LOCKSTEP with the TX,
//                                DEVOURER_TDMA_SYNC=wallclock (shared system
//                                clock) or =marker (self-clock off the TX's
//                                per-burst marker frame — no shared clock).
//   DEVOURER_TDMA_ROLE=rx-camp   camp permanently on one width
//                                (DEVOURER_TDMA_CAMP) — mode 2's per-band RX.
//
// See tdma.h for the schedule math, the on-air TD frame tag, and the full env
// knob list. Metrics are JSONL on stdout ({"ev":"tdma.*"}).
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#if defined(__MINGW32__) || defined(__MINGW64__)
  #include <libusb-1.0/libusb.h>
  #include <unistd.h>
#elif defined(__APPLE__) || defined(__ANDROID__)
  #include <libusb.h>
  #include <unistd.h>
#else
  #include <unistd.h>
  #include <libusb-1.0/libusb.h>
#endif

#include "RadiotapBuilder.h"
#include "RxPacket.h"
#include "SignalStop.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"
#include "tdma.h"

#define USB_VENDOR_ID 0x0bda
static constexpr uint16_t kRealtekProductIds[] = {
    0x8812, 0x0811, 0xa811, 0xb811, 0x8813,
};

// --- shared RX state (control thread switches, RX thread counts) ------------
static std::atomic<int> g_rx_mhz{20};                  // RX's current width
static std::atomic<uint64_t> g_cnt[2][3];              // [nb?][class]
static std::atomic<int64_t> g_marker_anchor_ns{0};     // steady ns of last marker
static std::atomic<bool> g_have_anchor{false};

static int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
static void emit(const char* json) { std::fputs(json, stdout); std::fflush(stdout); }

// --- device open (mirrors examples/svctx/main.cpp) --------------------------
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

// --- TX role ----------------------------------------------------------------
static void run_tx(IRtlDevice* dev, const tdma::Config& c) {
  dev->InitWrite(SelectedChannel{c.channel, 0, CHANNEL_WIDTH_20});
  sleep(2);
  const auto rt_crit = devourer::build_stream_radiotap(c.crit_rate);
  const auto rt_bulk = devourer::build_stream_radiotap(c.bulk_rate);
  // Markers ride the robust critical rate (they must be heard to sync).
  const auto& rt_marker = rt_crit;

  ChannelWidth_t cur_w = CHANNEL_WIDTH_20;
  int64_t last_marker_burst = -1;
  uint32_t seq[3] = {0, 0, 0};
  auto next_stat = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  fprintf(stderr, "tdma tx: nb=%dMHz/%dms wide=%dMHz/%dms crit=%s bulk=%s\n",
          tdma::mhz_of(c.sched.nb_w), c.sched.nb_ms, tdma::mhz_of(c.sched.wide_w),
          c.sched.wide_ms, "crit", "bulk");

  while (!g_devourer_should_stop) {
    auto a = c.sched.at(tdma::wall_ms());
    ChannelWidth_t w = c.sched.width(a.phase);
    if (w != cur_w) { dev->FastSetBandwidth(w); cur_w = w; }

    if (a.phase == tdma::Phase::NB && a.burst != last_marker_burst) {
      last_marker_burst = a.burst;
      auto f = tdma::build_frame(rt_marker, tdma::Class::Marker,
                                 seq[0]++, (uint32_t)a.burst);
      dev->send_packet(f.data(), f.size());
    }
    tdma::Class cls =
        a.phase == tdma::Phase::NB ? tdma::Class::Critical : tdma::Class::Bulk;
    const auto& rt = cls == tdma::Class::Critical ? rt_crit : rt_bulk;
    auto f = tdma::build_frame(rt, cls, seq[(int)cls]++, (uint32_t)a.burst);
    dev->send_packet(f.data(), f.size());

    if (std::chrono::steady_clock::now() >= next_stat) {
      next_stat += std::chrono::seconds(1);
      char buf[256];
      std::snprintf(buf, sizeof(buf),
                    "{\"ev\":\"tdma.tx\",\"marker\":%u,\"critical\":%u,"
                    "\"bulk\":%u,\"width_mhz\":%d}\n",
                    seq[0], seq[1], seq[2], tdma::mhz_of(cur_w));
      emit(buf);
    }
    if (c.gap_us > 0) usleep(c.gap_us);
  }
}

// --- RX roles ---------------------------------------------------------------
static void rx_callback(const Packet& p) {
  auto pr = tdma::parse_frame(p.Data.data(), p.Data.size());
  if (!pr.ok || p.RxAtrib.crc_err) return;
  int nb = g_rx_mhz.load(std::memory_order_relaxed) <= 10 ? 1 : 0;
  g_cnt[nb][(int)pr.cls].fetch_add(1, std::memory_order_relaxed);
  if (pr.cls == tdma::Class::Marker) {
    g_marker_anchor_ns.store(steady_ns(), std::memory_order_relaxed);
    g_have_anchor.store(true, std::memory_order_relaxed);
  }
}

// Desired RX width `guard` ms ahead, per the sync mode.
static ChannelWidth_t desired_width(const tdma::Config& c) {
  if (c.role == tdma::Role::RxCamp) return c.camp_w;
  if (c.sync == tdma::Sync::WallClock) {
    auto a = c.sched.at(tdma::wall_ms() + c.guard_ms);
    return c.sched.width(a.phase);
  }
  // marker: camp narrowband until a marker anchors us, then coast on steady_clock.
  if (!g_have_anchor.load(std::memory_order_relaxed)) return c.sched.nb_w;
  int64_t elapsed_ms =
      (steady_ns() - g_marker_anchor_ns.load(std::memory_order_relaxed)) / 1000000;
  if (elapsed_ms > 3LL * c.sched.period()) {  // markers stopped — re-acquire
    g_have_anchor.store(false, std::memory_order_relaxed);
    return c.sched.nb_w;
  }
  int pos = (int)((elapsed_ms + c.guard_ms) % c.sched.period());
  return pos < c.sched.nb_ms ? c.sched.nb_w : c.sched.wide_w;
}

static void run_rx(IRtlDevice* dev, const tdma::Config& c) {
  // Bring RX up: rx-camp at its band; rx-sync wide (the control loop corrects).
  ChannelWidth_t start_w = c.role == tdma::Role::RxCamp ? c.camp_w : CHANNEL_WIDTH_20;
  g_rx_mhz.store(tdma::mhz_of(start_w));
  std::thread rx([&] { dev->Init(rx_callback, SelectedChannel{c.channel, 0, start_w}); });

  const char* role = c.role == tdma::Role::RxCamp ? "rx-camp"
                     : c.sync == tdma::Sync::Marker ? "rx-sync/marker"
                                                    : "rx-sync/wallclock";
  fprintf(stderr, "tdma %s: start %dMHz nb=%dMHz wide=%dMHz guard=%dms\n", role,
          tdma::mhz_of(start_w), tdma::mhz_of(c.sched.nb_w),
          tdma::mhz_of(c.sched.wide_w), c.guard_ms);

  // Let Init finish bring-up before the control thread touches the RF registers
  // — a FastSetBandwidth racing the bring-up corrupts a half-configured chip.
  for (int i = 0; i < 250 && !g_devourer_should_stop; ++i)
    usleep(10000);  // ~2.5 s settle

  ChannelWidth_t cur_w = start_w;
  auto next_stat = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (!g_devourer_should_stop) {
    if (c.role == tdma::Role::RxSync) {
      ChannelWidth_t w = desired_width(c);
      if (w != cur_w) {
        dev->FastSetBandwidth(w);
        cur_w = w;
        g_rx_mhz.store(tdma::mhz_of(w), std::memory_order_relaxed);
      }
    }
    if (std::chrono::steady_clock::now() >= next_stat) {
      next_stat += std::chrono::seconds(1);
      char buf[320];
      std::snprintf(
          buf, sizeof(buf),
          "{\"ev\":\"tdma.rx\",\"nb_marker\":%llu,\"nb_critical\":%llu,"
          "\"nb_bulk\":%llu,\"wide_marker\":%llu,\"wide_critical\":%llu,"
          "\"wide_bulk\":%llu,\"width_mhz\":%d}\n",
          (unsigned long long)g_cnt[1][0].load(), (unsigned long long)g_cnt[1][1].load(),
          (unsigned long long)g_cnt[1][2].load(), (unsigned long long)g_cnt[0][0].load(),
          (unsigned long long)g_cnt[0][1].load(), (unsigned long long)g_cnt[0][2].load(),
          tdma::mhz_of(cur_w));
      emit(buf);
    }
    usleep(2000);
  }
  dev->StopRxLoop();
  rx.join();

  // Final summary: per (RX phase-band, class). In lockstep, critical+marker land
  // under the narrowband band, bulk under wide; leakage is the off-diagonal.
  fprintf(stderr,
          "\n=== tdma %s summary ===\n"
          "                marker   critical      bulk\n"
          "  narrowband  %8llu  %8llu  %8llu\n"
          "  wide        %8llu  %8llu  %8llu\n",
          role, (unsigned long long)g_cnt[1][0].load(),
          (unsigned long long)g_cnt[1][1].load(), (unsigned long long)g_cnt[1][2].load(),
          (unsigned long long)g_cnt[0][0].load(), (unsigned long long)g_cnt[0][1].load(),
          (unsigned long long)g_cnt[0][2].load());
}

int main() {
  auto logger = std::make_shared<Logger>();
  apply_logging_env(*logger);
  install_devourer_signal_handlers();

  tdma::Config c = tdma::config_from_env();

  libusb_context* ctx = nullptr;
  std::shared_ptr<devourer::UsbDeviceLock> lock;
  auto* handle = open_device(logger, &ctx, lock);
  if (!handle) { if (ctx) libusb_exit(ctx); return 1; }

  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRtlDevice(handle, ctx, lock, devourer_config_from_env());
  if (!dev) { logger->error("no driver for this chip"); return 1; }

  if (c.role == tdma::Role::Tx) run_tx(dev.get(), c);
  else run_rx(dev.get(), c);
  return 0;
}
