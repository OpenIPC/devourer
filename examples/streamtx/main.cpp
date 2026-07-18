// streamtx — stdin-driven TX for the precoder stream link.
//
// Mirrors precoder's chip-setup boilerplate (legacy 6M OFDM probe-request
// carrier, single-stream BPSK/BCC, RTL8812AU/8821AU/8811AU), but instead of
// looping on one shaped PSDU it reads a sequence of length-prefixed PSDU
// bodies from stdin and sends one probe-request per body. The encoder
// (tools/precoder/stream_tx.py) drives this binary; the two are intentionally
// split so the C++ side stays USB-only and the framing math stays in Python.
//
// On-wire frame protocol (stdin):
//   <u32_le length><length bytes of descrambled PSDU body>
// EOF on stdin = orderly shutdown.
//
// Why "descrambled" body bytes: the Realtek chip applies its own scrambler
// before BCC. So the bytes we hand to `send_packet` are the bits the chip
// will scramble — i.e. the bits the encoder produced as `descramble(...)`'s
// pre-image. Symmetric on RX: DEVOURER_DUMP_BODY / DEVOURER_STREAM_OUT print
// what the chip has already descrambled. The byte stream is the same on both
// ends.
//
// Usage:
//   DEVOURER_PID=0x8812 DEVOURER_CHANNEL=6 ./build/streamtx \\
//       [--interval-ms MS] [--max-psdu BYTES] < bodies.bin
//   uv run python tools/precoder/stream_tx.py < data.bin | \\
//       ./build/streamtx
//
// Env: same conventions as the other demos (DEVOURER_VID / DEVOURER_PID /
// DEVOURER_CHANNEL / DEVOURER_SKIP_RESET).

#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#if defined(_MSC_VER)
  /* libusb.h explicitly: the pre-seam RtlUsbAdapter.h used to pull it in
   * for every consumer; the bus-neutral RtlAdapter.h no longer does. */
  #include <libusb.h>
  #include <io.h>
  #include <fcntl.h>
  #include <windows.h>
  #include <process.h>
  typedef int pid_t;
  #define sleep(seconds) Sleep((seconds)*1000)
#elif defined(__MINGW32__) || defined(__MINGW64__)
  // mingw builds: POSIX libusb/unistd PLUS io.h/fcntl.h for binary stdin.
  #include <io.h>
  #include <fcntl.h>
  #include <unistd.h>
  #include <libusb-1.0/libusb.h>
#elif defined(__ANDROID__)
  #include <libusb.h>
  #include <unistd.h>
#elif defined(__APPLE__)
  #include <unistd.h>
  #include <libusb.h>
#else
  #include <unistd.h>
  #include <libusb-1.0/libusb.h>
#endif

#include "HopSchedule.h"
#include "RadiotapBuilder.h"
#include "RtlAdapter.h"
#if defined(DEVOURER_HAVE_JAGUAR1)
#include "jaguar1/RtlJaguarDevice.h"
#endif
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"
#include "stream_stdin.h"

#define USB_VENDOR_ID 0x0bda

  static constexpr uint16_t kRealtekProductIds[] = {
      0x8812, 0x0811, 0xa811, 0xb811, 0x8813,
  };

  // Identical 802.11 probe-request header to precoder; radiotap is now
  // built once at startup from DEVOURER_STREAM_RATE — accepts legacy
  // (6M..54M), HT (MCS0..MCS31), or VHT (VHT1SS_MCS0..VHT4SS_MCS9) carrier
  // modes. Default is 6M legacy OFDM, bit-identical to the historic
  // kRadiotapLegacy6M constant. Same canonical SA, same matcher in
  // examples/rx/main.cpp's RX path — keep these three in lockstep, see
  // CLAUDE.md.
  static const std::vector<uint8_t> kStreamRadiotap =
      devourer::build_stream_radiotap(devourer_tx_mode_from_env());
  static const uint8_t kCanonicalSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};

  static std::vector<uint8_t> build_dot11_probe_req() {
    std::vector<uint8_t> h = {
        0x40, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    };
    h.insert(h.end(), kCanonicalSa, kCanonicalSa + 6);
    h.insert(h.end(), kCanonicalSa, kCanonicalSa + 6);
    h.push_back(0x80);
    h.push_back(0x00);
    return h;
  }

int main(int argc, char **argv) {
  auto logger = std::make_shared<Logger>();
  apply_logging_env(*logger); /* DEVOURER_LOG_LEVEL / DEVOURER_EVENTS / ... */
  /* Events ride stderr here (overriding the stdout default): stdout is left
   * clean for downstream callers that may chain this binary. */
  logger->events().configure(stderr);

  int interval_ms = 2;
  // Lockstep sync-marker cadence: emit one marker-only frame every N data
  // frames in slot-hop mode so a tracking RX keeps its slot lock (a marker only
  // at each slot boundary is too sparse — a single miss drops the lock). The
  // caller's FEC PSDUs are never touched; the marker rides its own frame.
  int sync_every = 4;
  if (const char *e = std::getenv("DEVOURER_HOP_SYNC_EVERY")) {
    sync_every = std::atoi(e);   // avoid std::max — windows.h's max macro
    if (sync_every < 1) sync_every = 1;
  }
  // Sanity cap on a single PSDU body; protects against an upstream framing
  // bug that would otherwise have us allocate gigabytes from a stray length
  // prefix. 4096 covers any realistic legacy-6M probe-request payload.
  size_t max_psdu = 4096;
  long termux_fd = 0;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--interval-ms" && i + 1 < argc) {
      interval_ms = std::atoi(argv[++i]);
    } else if (a == "--max-psdu" && i + 1 < argc) {
      max_psdu = static_cast<size_t>(std::strtoul(argv[++i], nullptr, 0));
    } else {
      char *end = nullptr;
      long v = std::strtol(a.c_str(), &end, 0);
      if (end && *end == '\0' && v > 0) termux_fd = v;
    }
  }

  // Make stdin binary so a 0x1A or CRLF doesn't corrupt PSDU bytes. Gated on
  // _WIN32 (not _MSC_VER) inside the shared helper — see examples/common/stream_stdin.h.
  stream_stdin::set_stdin_binary();

  libusb_context *context = nullptr;
  libusb_device_handle *handle = nullptr;
  int rc;

  if (termux_fd > 0) {
    logger->info("Termux mode: wrapping fd {}", termux_fd);
    libusb_set_option(NULL, LIBUSB_OPTION_NO_DEVICE_DISCOVERY);
    libusb_set_option(NULL, LIBUSB_OPTION_WEAK_AUTHORITY);
    libusb_init(&context);
    rc = libusb_wrap_sys_device(context, (intptr_t)termux_fd, &handle);
    if (rc < 0) {
      logger->error("libusb_wrap_sys_device: {}", rc);
      return 1;
    }
  } else {
    rc = libusb_init(&context);
    if (rc < 0) return rc;

    uint16_t target_pid = 0;
    if (const char *pid_env = std::getenv("DEVOURER_PID")) {
      target_pid = static_cast<uint16_t>(std::strtoul(pid_env, nullptr, 0));
      logger->info("DEVOURER_PID={:04x} (limiting to this PID)", target_pid);
    }
    uint16_t target_vid = USB_VENDOR_ID;
    if (const char *vid_env = std::getenv("DEVOURER_VID")) {
      target_vid = static_cast<uint16_t>(std::strtoul(vid_env, nullptr, 0));
    }
    for (uint16_t pid : kRealtekProductIds) {
      if (target_pid != 0 && pid != target_pid) continue;
      handle = libusb_open_device_with_vid_pid(context, target_vid, pid);
      if (handle != NULL) {
        logger->info("Opened device {:04x}:{:04x}", target_vid, pid);
        break;
      }
    }
    if (handle == NULL && target_pid != 0) {
      handle = libusb_open_device_with_vid_pid(context, target_vid, target_pid);
    }
    if (handle == NULL) {
      logger->error("No supported device found under VID {:04x}", target_vid);
      libusb_exit(context);
      return 1;
    }
  }

  /* Claim-before-reset (see src/UsbOpen.h): the exclusive claim is the primary
   * guard — a second devourer on this adapter gets BUSY here and bails before
   * the reset, so it can't re-enumerate the adapter out from under the owner. */
  std::shared_ptr<devourer::UsbDeviceLock> usb_lock;
  /* Reopen variant: recovers in place when the reset re-enumerates the
   * device (warm Kestrel firmware-drop through ROM / ZeroCD). */
  rc = devourer::claim_interface_reset_reopen(context, handle, logger,
      termux_fd == 0 && std::getenv("DEVOURER_SKIP_RESET") == nullptr, usb_lock);
  if (rc != 0) {
    if (handle != nullptr)
      libusb_close(handle);
    libusb_exit(context);
    return 1;
  }

  WiFiDriver wifi_driver{logger};
  auto stream_cfg = devourer_config_from_env();
  /* FPV downlink default: disable the MAC carrier-sense gate so the video TX
   * punches through co-channel traffic instead of deferring — on-air ~1.5-2.2x
   * inject-rate recovery under a co-channel transmitter (issue #199, SetCcaMode /
   * DEVOURER_DIS_CCA). The link owns the channel, so CSMA back-off only stutters
   * it. Explicit DEVOURER_DIS_CCA=0 still forces standard carrier-sense back on. */
  if (std::getenv("DEVOURER_DIS_CCA") == nullptr)
    stream_cfg.tuning.disable_cca = true;
  auto rtlDevice = wifi_driver.CreateRtlDevice(handle, nullptr, usb_lock,
                                               stream_cfg);
  /* Jaguar1-only research features (TXAGC override, fast-retune hopping) aren't
   * on the IRtlDevice contract — downcast for them; jag is null on Jaguar3, and
   * the downcast plus its call sites compile out when Jaguar1 isn't built. */
#if defined(DEVOURER_HAVE_JAGUAR1)
  RtlJaguarDevice *jag = dynamic_cast<RtlJaguarDevice *>(rtlDevice.get());
#endif

  int channel = 6;
  if (const char *ch_env = std::getenv("DEVOURER_CHANNEL")) {
    channel = std::atoi(ch_env);
    logger->info("DEVOURER_CHANNEL set — tuning TX to channel {}", channel);
  }

  /* DEVOURER_TX_POWER forces a flat TXAGC index (low single-digits for an
   * attenuated/noisy bench). Useful for stress-testing the RX path's
   * corruption handling — lowering this forces marginal SNR, which raises the
   * chip's CRC-failure rate so the corrupted-frame surfacing path actually
   * gets exercised. Unset = each family's calibrated default (SetTxPower is
   * now a real flat override on EVERY generation — the old unconditional
   * SetTxPower(40) here was a no-op on Jaguar1/2 and would now flatten their
   * efuse per-rate table; the 8822C's 40 default lives in its reference base). */
  if (const char *p = std::getenv("DEVOURER_TX_POWER"))
    rtlDevice->SetTxPower(static_cast<uint8_t>(std::atoi(p)));
  rtlDevice->InitWrite(SelectedChannel{.Channel = static_cast<uint8_t>(channel),
                                       .ChannelOffset = 0,
                                       .ChannelWidth = CHANNEL_WIDTH_20});

  /* DEVOURER_TX_PWR_OVERRIDE: force an absolute per-rate TXAGC index,
   * bypassing the EFUSE/SetTxPower table — the finest-grained, lowest TX-power
   * knob for pushing the link into the marginal-SNR regime where the RX's
   * corrupted-frame salvage path gets exercised (pairs with the B210 interferer
   * in tests/fused_fec_onair.sh). Applied once and held, unlike
   * txdemo's DEVOURER_TX_PWR_START ramp. Must follow InitWrite so it
   * applies live. Generation-agnostic (IRtlDevice runtime TX-power API). */
  if (const char *o = std::getenv("DEVOURER_TX_PWR_OVERRIDE")) {
    int idx = std::atoi(o);
    rtlDevice->SetTxPowerIndexOverride(idx);
    logger->info("DEVOURER_TX_PWR_OVERRIDE — forced absolute TXAGC index {}", idx);
  }
  /* Channel hopping for frequency diversity. DEVOURER_HOP_CHANNELS="1,6,11"
   * cycles the TX channel every DEVOURER_HOP_DWELL_FRAMES PSDUs (default 1 =
   * per-packet hop, which spreads an outer-FEC block's shards across channels
   * so a single-channel fade/interferer only erases ~1/N of each block —
   * recoverable by the RS layer; see tools/precoder/stream_fec_rs.py
   * --hop-interleave). Uses FastRetune (lean intra-band retune, ~1-2 ms) unless
   * DEVOURER_HOP_FAST=0 (full SetMonitorChannel) / =2 (FastRetune, no RF cache).
   * Intra-band 20 MHz only; a cross-band entry falls back automatically. */
  std::vector<int> hop_channels;
  long hop_dwell = 1;
  long hop_slot_ms = 0;
  std::optional<devourer::HopSchedule> hop_schedule;
  const int hop_fast = std::getenv("DEVOURER_HOP_FAST")
                           ? std::atoi(std::getenv("DEVOURER_HOP_FAST"))
                           : 1;
  if (const char *e = std::getenv("DEVOURER_HOP_CHANNELS")) {
    std::string s(e);
    size_t pos = 0;
    while (pos < s.size()) {
      size_t c = s.find(',', pos);
      std::string tok =
          s.substr(pos, c == std::string::npos ? std::string::npos : c - pos);
      if (!tok.empty()) {
        int ch = std::atoi(tok.c_str());
        if (ch > 0) hop_channels.push_back(ch);
      }
      if (c == std::string::npos) break;
      pos = c + 1;
    }
    if (const char *d = std::getenv("DEVOURER_HOP_DWELL_FRAMES")) {
      hop_dwell = std::strtol(d, nullptr, 0);
      if (hop_dwell < 1) hop_dwell = 1;
    }
    if (const char *s = std::getenv("DEVOURER_HOP_SLOT_MS")) {
      hop_slot_ms = std::strtol(s, nullptr, 0);
      if (hop_slot_ms < 1)
        throw std::invalid_argument("DEVOURER_HOP_SLOT_MS must be positive");
      if (std::getenv("DEVOURER_HOP_DWELL_FRAMES"))
        throw std::invalid_argument(
            "hop slot and frame dwell are mutually exclusive");
    }
    if (const char *seed = std::getenv("DEVOURER_HOP_SEED"))
      hop_schedule.emplace(devourer::HopSchedule::parse_seed(seed));
    else if (hop_slot_ms > 0)
      // Slot-mode sequential hopping rides the keyless schedule so it still
      // emits the lockstep sync marker (same channels[slot % n] order).
      hop_schedule.emplace(devourer::HopSchedule::sequential());
    if (!hop_channels.empty()) {
      std::string list;
      for (size_t i = 0; i < hop_channels.size(); ++i)
        list += (i ? "," : "") + std::to_string(hop_channels[i]);
      logger->info("DEVOURER_HOP_CHANNELS — stream hopping [{}] dwell={} "
                   "fast={}", list, hop_dwell, hop_fast);
    }
  }

  sleep(2);

  auto dot11 = build_dot11_probe_req();
  std::vector<uint8_t> tx_buf;
  tx_buf.reserve(kStreamRadiotap.size() + dot11.size() + max_psdu);

  logger->info(
      "stream TX ready (legacy 6M OFDM, ch {}); reading length-prefixed PSDUs "
      "from stdin", channel);

  long tx_count = 0;
  const auto hop_start = std::chrono::steady_clock::now();
  uint64_t last_hop_slot = UINT64_MAX;
  // Per-process epoch for the lockstep sync marker (see below): lets a tracking
  // RX detect a TX restart and re-anchor its slot clock.
  const uint32_t hop_epoch = static_cast<uint32_t>(
      std::chrono::high_resolution_clock::now().time_since_epoch().count());
  std::vector<uint8_t> sync_buf;
  while (true) {
    uint8_t len_bytes[4];
    {
      auto r = stream_stdin::read_exact(stdin, len_bytes, sizeof(len_bytes));
      if (r == stream_stdin::ReadResult::Eof) break;  // clean stdin close
      if (r == stream_stdin::ReadResult::Short) {
        logger->error("short read on stdin len-prefix; record truncated");
        std::exit(2);
      }
    }
    uint32_t len = static_cast<uint32_t>(len_bytes[0])
                 | (static_cast<uint32_t>(len_bytes[1]) << 8)
                 | (static_cast<uint32_t>(len_bytes[2]) << 16)
                 | (static_cast<uint32_t>(len_bytes[3]) << 24);
    if (len == 0 || len > max_psdu) {
      logger->error("PSDU length {} out of range (max {}); stopping", len,
                    max_psdu);
      break;
    }
    std::vector<uint8_t> psdu(len);
    {
      auto r = stream_stdin::read_exact(stdin, psdu.data(), len);
      if (r == stream_stdin::ReadResult::Eof) {
        logger->warn("EOF mid-PSDU (expected {} bytes)", len);
        break;
      }
      if (r == stream_stdin::ReadResult::Short) {
        logger->error("short read mid-PSDU (expected {} bytes); record "
                      "truncated", len);
        std::exit(2);
      }
    }

    /* Retune to this PSDU's hop channel before sending. FastRetune/fast_retune
     * is a cheap no-op when the channel is unchanged within a dwell, so calling
     * it per packet is fine. */
    if (!hop_channels.empty()) {
      uint64_t slot =
          hop_slot_ms > 0
              ? static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - hop_start)
                        .count() /
                    hop_slot_ms)
              : static_cast<uint64_t>(tx_count / hop_dwell);
      int ch = hop_schedule ? hop_schedule->channel(slot, hop_channels)
                            : hop_channels[slot % hop_channels.size()];
      const bool slot_changed = (slot != last_hop_slot);
      if (slot_changed) {
        auto ev = devourer::Ev(logger->events(), "hop.dwell");
        ev.f("slot", (unsigned long long)slot)
            .f("round", (unsigned long long)(slot / hop_channels.size()))
            .f("channel", ch);
        if (hop_schedule)
          ev.hexf("seed_fp", hop_schedule->fingerprint(), 8);
        last_hop_slot = slot;
      }
      if (hop_fast)
        rtlDevice->FastRetune(static_cast<uint8_t>(ch),
                              /*cache_rf=*/hop_fast != 2);
      else
        rtlDevice->SetMonitorChannel(SelectedChannel{
            .Channel = static_cast<uint8_t>(ch),
            .ChannelOffset = 0,
            .ChannelWidth = CHANNEL_WIDTH_20});

      /* Lockstep sync: emit a marker-only frame at each slot boundary AND every
       * sync_every data frames, so a tracking RX keeps the TX slot clock locked
       * (one marker per slot is too sparse to survive a miss). It rides its own
       * frame — the caller's FEC PSDUs stay byte-for-byte untouched — with the
       * canonical SA, so the RX's marker matcher sees it. Slot-hop mode only. */
      if (hop_schedule && hop_slot_ms > 0 &&
          (slot_changed || tx_count % sync_every == 0)) {
        const uint64_t slot_us = static_cast<uint64_t>(hop_slot_ms) * 1000;
        const uint64_t us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - hop_start)
                .count());
        devourer::HopSyncMarker marker{hop_schedule->fingerprint(), hop_epoch,
                                       static_cast<uint32_t>(us % slot_us),
                                       us / slot_us};
        auto wire = devourer::HopSyncMarker::encode(marker);
        sync_buf.clear();
        sync_buf.insert(sync_buf.end(), kStreamRadiotap.begin(),
                        kStreamRadiotap.end());
        sync_buf.insert(sync_buf.end(), dot11.begin(), dot11.end());
        sync_buf.insert(sync_buf.end(), wire.begin(), wire.end());
        rtlDevice->send_packet(sync_buf.data(), sync_buf.size());
      }
    }

    tx_buf.clear();
    tx_buf.insert(tx_buf.end(), kStreamRadiotap.begin(),
                  kStreamRadiotap.end());
    tx_buf.insert(tx_buf.end(), dot11.begin(), dot11.end());
    tx_buf.insert(tx_buf.end(), psdu.begin(), psdu.end());
    bool ok = rtlDevice->send_packet(tx_buf.data(), tx_buf.size());
    ++tx_count;
    // TX progress marker (event stream rides stderr in this demo, keeping
    // stdout clean for downstream callers that may chain this binary).
    if (tx_count <= 5 || tx_count % 500 == 0) {
      devourer::Ev(logger->events(), "stream.tx")
          .f("n", tx_count)
          .f("ok", ok)
          .f("psdu", len)
          .f("total", tx_buf.size());
    }
    if (interval_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    }
  }

  devourer::Ev(logger->events(), "stream.done").f("sent", tx_count);
  // Destruct the device before tearing down libusb: on Jaguar1 the TX path is
  // asynchronous, and closing the handle with transfers still in flight reaps
  // completions against a freed handle (segfault, and the crash leaks the USB
  // claim so the next run hits "adapter in use"). reset() runs the dtor, which
  // drains outstanding transfers while the handle is still valid.
  rtlDevice.reset();
  libusb_release_interface(handle, 0);
  libusb_close(handle);
  libusb_exit(context);
  return 0;
}
