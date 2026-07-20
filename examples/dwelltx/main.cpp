// dwelltx — dwell-1 A/B injection over the firmware channel switch.
//
// The data-plane experiment (issue #272): a two-context A/B schedule where the
// radio spends one wall-clock slot per channel and each slot carries EXACTLY
// ONE admitted data-frame opportunity — "dwell-1". The channel switch at each
// slot boundary is the on-chip firmware switch when DEVOURER_FASTRETUNE_FW is
// set (Jaguar2 8822B / Jaguar3 8822C/8822E; H2C 0x1D — see
// docs/experiments/kernel-channel-switch-offload.md); otherwise the software FastRetune,
// so the two are a controlled A/B.
//
// This is the caller-side answer to the rejected MCC/FCS scheduler
// (docs/experiments/mcc-fcs-investigation.md): the library already switches channels in
// ~1 ms and hops per-packet from a radiotap CHANNEL field. What a real hopping
// DATA plane needs on top of that is bounded admission — one frame per slot,
// placed inside a window that guarantees it finishes airing before the slot
// flips, so a frame built for context A can NEVER air on B. That admission
// policy lives here, in the caller, not in the library.
//
// Each per-slot frame carries a HopSyncMarker (src/HopSchedule.h) whose `slot`
// field, run through the same public/keyed schedule, tells any receiver which
// channel the slot belongs to — so an oracle pinned to channel A that decodes
// a frame whose slot maps to B is a self-evident wrong-channel event, and the
// unique per-slot marker doubles as the duplicate/dedup key. No extra tag.
//
// Env (mapped through examples/common/env_config.h for the library knobs):
//   DEVOURER_PID / DEVOURER_VID / DEVOURER_USB_BUS / DEVOURER_USB_PORT
//   DEVOURER_FASTRETUNE_FW=1|2   firmware switch (1 intra-band, 2 + cross-band)
//   DEVOURER_DWELL_CHANNELS=36,40   the A,B pair (exactly two)
//   DEVOURER_DWELL_SLOT_MS=N        wall-clock slot length (default 20)
//   DEVOURER_DWELL_SLOTS=N          total slots then exit (0 = run forever)
//   DEVOURER_DWELL_SETTLE_US=N      post-switch settle before admitting (500)
//   DEVOURER_DWELL_GUARD_US=N       guard before the slot end (1000)
//   DEVOURER_DWELL_AIRTIME_US=N     budgeted frame airtime (300 @ 6M/96B)
//   DEVOURER_DWELL_LATE_US=N        inject: sleep this long into each slot
//                                   before admitting (fault: forces drops)
//   DEVOURER_HOP_SEED=<hex>         keyed A/B order (default public sequential)
//   DEVOURER_TX_RATE / DEVOURER_TX_PWR  as the other demos

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#if defined(_MSC_VER)
  #include <libusb.h>
#elif defined(__MINGW32__) || defined(__MINGW64__)
  #include <libusb-1.0/libusb.h>
  #include <unistd.h>
#elif defined(__APPLE__)
  #include <libusb.h>
  #include <unistd.h>
#else
  #include <unistd.h>
  #include <libusb-1.0/libusb.h>
#endif

#include "ChannelFreq.h"
#include "Event.h"
#include "HopSchedule.h"
#include "RadiotapBuilder.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"
#include "usb_select.h"

namespace {

using clock_t_ = std::chrono::steady_clock;

long env_long(const char *name, long def) {
  if (const char *e = std::getenv(name)) {
    long v = std::strtol(e, nullptr, 0);
    return v;
  }
  return def;
}

// Full 24-byte probe-request MAC header with the canonical SA the RX path
// already matches (kept in lockstep with examples/tx and examples/rx — see
// CLAUDE.md). Address 3 (BSSID) = broadcast, so the header is exactly 24
// bytes and the appended HopSyncMarker lands at the body boundary the RX
// oracle reports (packet.Data + 24).
std::vector<uint8_t> build_dot11_probe_req() {
  static const uint8_t sa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
  static const uint8_t bcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  std::vector<uint8_t> h = {0x40, 0x00, 0x00, 0x00};
  h.insert(h.end(), bcast, bcast + 6); // Address 1 (DA)
  h.insert(h.end(), sa, sa + 6);       // Address 2 (SA)
  h.insert(h.end(), bcast, bcast + 6); // Address 3 (BSSID)
  h.push_back(0x80);                   // seq/frag control
  h.push_back(0x00);
  return h;
}

} // namespace

int main() {
  auto logger = std::make_shared<Logger>();
  apply_logging_env(*logger);
  auto &ev = logger->events();

  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) < 0)
    return 1;
  static const uint16_t kDefaultPids[] = {0x8812, 0xb812, 0xc811, 0xc820,
                                          0xc82c, 0x8814, 0xb82c};
  libusb_device_handle *handle = open_selected_usb(
      ctx, logger, kDefaultPids, sizeof(kDefaultPids) / sizeof(kDefaultPids[0]));
  if (!handle) {
    libusb_exit(ctx);
    return 1;
  }
  std::shared_ptr<devourer::UsbDeviceLock> usb_lock;
  if (devourer::claim_interface_reset_reopen(
          ctx, handle, logger, std::getenv("DEVOURER_SKIP_RESET") == nullptr,
          usb_lock) != 0) {
    libusb_exit(ctx);
    return 1;
  }

  WiFiDriver driver{logger};
  auto cfg = devourer_config_from_env(); // honors DEVOURER_FASTRETUNE_FW
  auto dev = driver.CreateRtlDevice(handle, nullptr, usb_lock, cfg);
  if (!dev) {
    libusb_exit(ctx);
    return 1;
  }

  // --- schedule + admission parameters ---------------------------------------
  std::vector<int> chans;
  if (const char *e = std::getenv("DEVOURER_DWELL_CHANNELS")) {
    std::string s(e);
    size_t pos = 0;
    while (pos < s.size()) {
      size_t c = s.find(',', pos);
      std::string tok = s.substr(pos, c == std::string::npos ? c : c - pos);
      if (!tok.empty())
        chans.push_back(std::atoi(tok.c_str()));
      if (c == std::string::npos)
        break;
      pos = c + 1;
    }
  }
  if (chans.size() < 2) {
    logger->error("DEVOURER_DWELL_CHANNELS must name at least two channels");
    libusb_exit(ctx);
    return 2;
  }
  const long slot_ms = env_long("DEVOURER_DWELL_SLOT_MS", 20);
  const long total_slots = env_long("DEVOURER_DWELL_SLOTS", 0);
  const long settle_us = env_long("DEVOURER_DWELL_SETTLE_US", 500);
  const long guard_us = env_long("DEVOURER_DWELL_GUARD_US", 1000);
  const long airtime_us = env_long("DEVOURER_DWELL_AIRTIME_US", 300);
  const long late_us = env_long("DEVOURER_DWELL_LATE_US", 0);
  const int hop_fast = static_cast<int>(env_long("DEVOURER_HOP_FAST", 1));

  std::optional<devourer::HopSchedule> sched;
  if (const char *seed = std::getenv("DEVOURER_HOP_SEED"))
    sched.emplace(devourer::HopSchedule::parse_seed(seed));
  else
    sched.emplace(devourer::HopSchedule::sequential());

  dev->InitWrite(SelectedChannel{.Channel = static_cast<uint8_t>(chans[0]),
                                 .ChannelOffset = 0,
                                 .ChannelWidth = CHANNEL_WIDTH_20});
  if (const char *p = std::getenv("DEVOURER_TX_PWR"))
    dev->SetTxPower(static_cast<uint8_t>(std::atoi(p)));

  const auto radiotap = devourer::build_stream_radiotap(devourer_tx_mode_from_env());
  const auto dot11 = build_dot11_probe_req();
  const uint32_t seed_fp = sched->fingerprint();

  devourer::Ev(ev, "dwell.start")
      .arr("channels", chans.data(), chans.size())
      .f("n", (long long)chans.size())
      .f("slot_ms", slot_ms)
      .f("settle_us", settle_us)
      .f("guard_us", guard_us)
      .f("airtime_us", airtime_us)
      .f("fw", cfg.tuning.fastretune_fw)
      .f("hop_fast", hop_fast)
      .hexf("seed_fp", seed_fp, 8);

  const auto t0 = clock_t_::now();
  auto us_since = [&](clock_t_::time_point t) {
    return std::chrono::duration_cast<std::chrono::microseconds>(t - t0).count();
  };
  const long slot_us = slot_ms * 1000;
  int64_t cur_slot = -1;
  bool admitted = false;
  long admitted_ct = 0, dropped_late_ct = 0, empty_ct = 0;

  std::vector<uint8_t> buf;
  buf.reserve(radiotap.size() + dot11.size() + 64);
  uint32_t epoch = 0;

  while (true) {
    const auto now = clock_t_::now();
    const int64_t slot = us_since(now) / slot_us;
    if (total_slots > 0 && slot >= total_slots)
      break;

    if (slot != cur_slot) {
      // --- slot boundary: retune to this slot's channel via the fw switch ----
      if (cur_slot >= 0 && !admitted)
        ++empty_ct; // previous slot aired nothing (shouldn't happen w/ budget)
      cur_slot = slot;
      admitted = false;
      const size_t idx =
          sched->channel_index(static_cast<uint64_t>(slot), chans.size());
      const int ch = chans[idx];
      const auto sw0 = clock_t_::now();
      dev->FastRetune(static_cast<uint8_t>(ch), /*cache_rf=*/hop_fast != 2);
      const long switch_us =
          std::chrono::duration_cast<std::chrono::microseconds>(
              clock_t_::now() - sw0)
              .count();
      devourer::Ev(ev, "dwell.slot")
          .f("slot", (unsigned long long)slot)
          .f("ctx", (int)idx)
          .f("channel", ch)
          .f("switch_us", switch_us);
      // Fault injection: sleep past the admission window on purpose.
      if (late_us > 0)
        std::this_thread::sleep_for(std::chrono::microseconds(late_us));
    }

    if (!admitted) {
      const int64_t slot_start_us = slot * slot_us;
      const int64_t admit_open = slot_start_us + settle_us;
      const int64_t admit_close = slot_start_us + slot_us - guard_us - airtime_us;
      const int64_t t = us_since(clock_t_::now());
      if (t < admit_open) {
        std::this_thread::sleep_for(std::chrono::microseconds(admit_open - t));
        continue;
      }
      if (t > admit_close) {
        // Missed the window — DROP with a structured reason. Never air a frame
        // that would cross into the next slot's channel.
        ++dropped_late_ct;
        admitted = true;
        devourer::Ev(ev, "dwell.drop")
            .f("slot", (unsigned long long)slot)
            .f("reason_late_us", (long long)(t - admit_close));
        continue;
      }
      // --- admit exactly one frame, tagged by the marker's slot --------------
      const size_t idx =
          sched->channel_index(static_cast<uint64_t>(slot), chans.size());
      buf.assign(radiotap.begin(), radiotap.end());
      buf.insert(buf.end(), dot11.begin(), dot11.end());
      devourer::HopSyncMarker m{seed_fp, epoch, static_cast<uint32_t>(t % slot_us),
                                static_cast<uint64_t>(slot)};
      const auto wire = devourer::HopSyncMarker::encode(m);
      buf.insert(buf.end(), wire.begin(), wire.end());
      const auto air0 = clock_t_::now();
      dev->send_packet(buf.data(), buf.size());
      admitted = true;
      ++admitted_ct;
      devourer::Ev(ev, "dwell.tx")
          .f("slot", (unsigned long long)slot)
          .f("ctx", (int)idx)
          .f("channel", chans[idx])
          .f("in_slot_us", (long long)(t - slot_start_us))
          .f("send_us",
             (long long)std::chrono::duration_cast<std::chrono::microseconds>(
                 clock_t_::now() - air0)
                 .count());
    } else {
      // one opportunity per slot already spent — idle until the boundary
      const int64_t next_boundary = (slot + 1) * slot_us;
      const int64_t t = us_since(clock_t_::now());
      if (next_boundary - t > 1500)
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }
  }

  devourer::Ev(ev, "dwell.done")
      .f("admitted", admitted_ct)
      .f("dropped_late", dropped_late_ct)
      .f("empty", empty_ct);
  libusb_exit(ctx);
  return 0;
}
