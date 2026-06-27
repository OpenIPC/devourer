// StreamTxDemo — stdin-driven TX for the precoder stream link.
//
// Mirrors PrecoderDemo's chip-setup boilerplate (legacy 6M OFDM probe-request
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
//   DEVOURER_PID=0x8812 DEVOURER_CHANNEL=6 ./build/StreamTxDemo \\
//       [--interval-ms MS] [--max-psdu BYTES] < bodies.bin
//   uv run python tools/precoder/stream_tx.py < data.bin | \\
//       ./build/StreamTxDemo
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

#include "FrameParser.h"
#include "RadiotapBuilder.h"
#include "RtlUsbAdapter.h"
#include "WiFiDriver.h"
#include "logger.h"
#include "stream_stdin.h"

#define USB_VENDOR_ID 0x0bda

static constexpr uint16_t kRealtekProductIds[] = {
    0x8812, 0x0811, 0xa811, 0xb811, 0x8813,
};

// Identical 802.11 probe-request header to PrecoderDemo; radiotap is now
// built once at startup from DEVOURER_STREAM_RATE — accepts legacy
// (6M..54M), HT (MCS0..MCS31), or VHT (VHT1SS_MCS0..VHT4SS_MCS9) carrier
// modes. Default is 6M legacy OFDM, bit-identical to the historic
// kRadiotapLegacy6M constant. Same canonical SA, same matcher in
// demo/main.cpp's RX path — keep these three in lockstep, see CLAUDE.md.
static const std::vector<uint8_t> kStreamRadiotap =
    devourer::build_stream_radiotap(devourer::parse_tx_mode_env());
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
  // _WIN32 (not _MSC_VER) inside the shared helper — see txdemo/stream_stdin.h.
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

  if (libusb_kernel_driver_active(handle, 0)) {
    rc = libusb_detach_kernel_driver(handle, 0);
    if (rc != 0) logger->error("libusb_detach_kernel_driver: {}", rc);
  }
  if (termux_fd == 0 && !std::getenv("DEVOURER_SKIP_RESET")) {
    libusb_reset_device(handle);
  }
  rc = libusb_claim_interface(handle, 0);
  assert(rc == 0);

  WiFiDriver wifi_driver{logger};
  auto rtlDevice = wifi_driver.CreateRtlDevice(handle);

  int channel = 6;
  if (const char *ch_env = std::getenv("DEVOURER_CHANNEL")) {
    channel = std::atoi(ch_env);
    logger->info("DEVOURER_CHANNEL set — tuning TX to channel {}", channel);
  }

  /* DEVOURER_TX_POWER overrides the per-rate "txpower" register value
   * (default 40, low single-digits for an attenuated/noisy bench). Useful
   * for stress-testing the RX path's corruption handling — lowering this
   * forces marginal SNR, which raises the chip's CRC-failure rate so the
   * corrupted-frame surfacing path actually gets exercised. */
  int tx_power = 40;
  if (const char *p = std::getenv("DEVOURER_TX_POWER")) tx_power = std::atoi(p);
  rtlDevice->SetTxPower(static_cast<uint8_t>(tx_power));
  rtlDevice->InitWrite(SelectedChannel{.Channel = static_cast<uint8_t>(channel),
                                       .ChannelOffset = 0,
                                       .ChannelWidth = CHANNEL_WIDTH_20});

  /* DEVOURER_TX_PWR_OVERRIDE: force an absolute per-rate TXAGC index (0..63),
   * bypassing the EFUSE/SetTxPower table — the finest-grained, lowest TX-power
   * knob for pushing the link into the marginal-SNR regime where the RX's
   * corrupted-frame salvage path gets exercised (pairs with the B210 interferer
   * in tests/fused_fec_onair.sh). Applied once and held, unlike
   * WiFiDriverTxDemo's DEVOURER_TX_PWR_START ramp. Must follow InitWrite so the
   * channel-set has run; ApplyTxPower re-pushes the index to the registers. */
  if (const char *o = std::getenv("DEVOURER_TX_PWR_OVERRIDE")) {
    int idx = std::atoi(o);
    rtlDevice->SetTxPowerOverride(idx);
    rtlDevice->ApplyTxPower();
    logger->info("DEVOURER_TX_PWR_OVERRIDE — forced absolute TXAGC index {}", idx);
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
        std::fprintf(stderr,
                     "stream_tx_demo: short read on stdin len-prefix; record "
                     "truncated\n");
        std::exit(2);
      }
    }
    uint32_t len = static_cast<uint32_t>(len_bytes[0])
                 | (static_cast<uint32_t>(len_bytes[1]) << 8)
                 | (static_cast<uint32_t>(len_bytes[2]) << 16)
                 | (static_cast<uint32_t>(len_bytes[3]) << 24);
    if (len == 0 || len > max_psdu) {
      std::fprintf(stderr,
                   "stream_tx_demo: PSDU length %u out of range (max %zu); "
                   "stopping\n", len, max_psdu);
      break;
    }
    std::vector<uint8_t> psdu(len);
    {
      auto r = stream_stdin::read_exact(stdin, psdu.data(), len);
      if (r == stream_stdin::ReadResult::Eof) {
        std::fprintf(stderr,
                     "stream_tx_demo: EOF mid-PSDU (expected %u bytes)\n", len);
        break;
      }
      if (r == stream_stdin::ReadResult::Short) {
        std::fprintf(stderr,
                     "stream_tx_demo: short read mid-PSDU (expected %u bytes); "
                     "record truncated\n", len);
        std::exit(2);
      }
    }

    tx_buf.clear();
    tx_buf.insert(tx_buf.end(), kStreamRadiotap.begin(),
                  kStreamRadiotap.end());
    tx_buf.insert(tx_buf.end(), dot11.begin(), dot11.end());
    tx_buf.insert(tx_buf.end(), psdu.begin(), psdu.end());
    bool ok = rtlDevice->send_packet(tx_buf.data(), tx_buf.size());
    ++tx_count;
    // Tag matches PrecoderDemo's so existing log-watchers keep working; route
    // to stderr to leave stdout clean for downstream callers that may chain
    // this binary.
    if (tx_count <= 5 || tx_count % 500 == 0) {
      std::fprintf(stderr,
                   "<stream-tx>TX #%ld ok=%d psdu=%u total=%zu\n",
                   tx_count, ok ? 1 : 0, len, tx_buf.size());
      std::fflush(stderr);
    }
    if (interval_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    }
  }

  std::fprintf(stderr, "<stream-tx>done; sent %ld PSDUs\n", tx_count);
  libusb_release_interface(handle, 0);
  libusb_close(handle);
  libusb_exit(context);
  return 0;
}
