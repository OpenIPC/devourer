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
// examples/rx/main.cpp's RX path — keep these three in lockstep, see CLAUDE.md.
static const std::vector<uint8_t> kStreamRadiotap =
    devourer::build_stream_radiotap(devourer_tx_mode_from_env());
static const uint8_t kCanonicalSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};

static std::vector<uint8_t> build_dot11_probe_req() {
  std::vector<uint8_t> h = {
      0x40, 0x00, 0x00, 0x00,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
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
  rc = devourer::claim_interface_then_reset(
      handle, 0, logger,
      termux_fd == 0 && std::getenv("DEVOURER_SKIP_RESET") == nullptr, usb_lock);
  if (rc != 0) {
    libusb_close(handle);
    libusb_exit(context);
    return 1;
  }

  WiFiDriver wifi_driver{logger};
  auto rtlDevice = wifi_driver.CreateRtlDevice(handle, nullptr, usb_lock,
                                               devourer_config_from_env());
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
      int ch = hop_channels[(tx_count / hop_dwell) % hop_channels.size()];
      if (hop_fast)
        rtlDevice->FastRetune(static_cast<uint8_t>(ch),
                              /*cache_rf=*/hop_fast != 2);
      else
        rtlDevice->SetMonitorChannel(SelectedChannel{
            .Channel = static_cast<uint8_t>(ch),
            .ChannelOffset = 0,
            .ChannelWidth = CHANNEL_WIDTH_20});
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
  libusb_release_interface(handle, 0);
  libusb_close(handle);
  libusb_exit(context);
  return 0;
}
