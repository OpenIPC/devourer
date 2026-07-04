// PrecoderDemo — transmit a pre-shaped PSDU produced by
// tools/precoder/encode_subcarriers.py.
//
// This is the on-air vehicle for the pre-modulator subcarrier PoC: the Python
// encoder inverts the chip's BPSK/BCC/interleaver/scrambler pipeline and emits
// the PSDU bytes that make chosen OFDM data subcarriers carry chosen bits.
// This demo wraps those bytes in the fixed radiotap + 802.11 header the chip
// and the existing TX-validation tooling expect, then streams them out.
//
// Scope (matches let-s-plan-this PoC): RTL8812AU / RTL8821AU / RTL8811AU,
// single-stream BPSK, BCC, 20 MHz. RTL8814AU is out of scope (issue #36 TX
// flakiness would mask the experiment).
//
// RATE CHOICE — legacy 6 Mbps OFDM, not HT MCS 0. The plan said HT MCS 0, but
// RtlJaguarDevice::send_packet only wires `fixed_rate` from the radiotap RATE
// field (legacy) or the VHT field — the HT MCS *index* is never read, so an
// HT-MCS radiotap with no RATE field would transmit at the MGN_1M default =
// 1 Mbps CCK (DSSS, NO OFDM subcarriers at all), defeating the whole PoC.
// Legacy 6 Mbps is BPSK rate-1/2 OFDM (48 data subcarriers, 16x3 interleaver)
// = exactly the encoder's `--phy legacy` and the plan's "48 subcarrier" prose.
// Encode shaped PSDUs with `--phy legacy` (the encoder default) to match.
//
// Usage:
//   DEVOURER_PID=0x8812 DEVOURER_CHANNEL=6 ./build/PrecoderDemo --psdu shaped.bin
//   [--count N] [--interval-ms MS]   (Termux: pass the numeric USB fd as argv[1])
//
// Env: DEVOURER_VID / DEVOURER_PID / DEVOURER_CHANNEL / DEVOURER_SKIP_RESET —
// same conventions as the other demos.

#include <cassert>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#if defined(_MSC_VER)
  #include <windows.h>
  #include <process.h>
  typedef int pid_t;
  #define sleep(seconds) Sleep((seconds)*1000)
#elif defined(__ANDROID__)
  #include <libusb.h>
#elif defined(__APPLE__)
  #include <unistd.h>
  #include <libusb.h>
#else
  #include <unistd.h>
  #include <libusb-1.0/libusb.h>
#endif

#include "RtlUsbAdapter.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "logger.h"

#define USB_VENDOR_ID 0x0bda

// Same PID set as the RX/TX demos. The precoder PoC targets the single-stream
// Jaguar parts; pin one with DEVOURER_PID (e.g. 0x8812).
static constexpr uint16_t kRealtekProductIds[] = {
    0x8812, 0x0811, 0xa811, 0xb811, 0x8813,
};

// Legacy 6 Mbps OFDM radiotap header (13 bytes). Presence = RATE(bit2) |
// TX_FLAGS(bit15); it_present = 0x00008004. Field layout after the 8-byte
// header: RATE @ offset 8 = 0x0c (12 * 500 kbps = 6 Mbps; numerically == the
// MGN_6M enum send_packet feeds to MRateToHwRate -> DESC_RATE6M), one pad byte
// for TX_FLAGS' 2-byte alignment, TX_FLAGS @ 10-11 = 0x0008, one trailing pad.
// Kept at exactly 13 bytes (0x0d) so send_packet's `len != 0x0d -> vht=true`
// heuristic leaves us on the non-VHT path (rate_id 8).
static const uint8_t kRadiotapLegacy6M[13] = {
    0x00, 0x00, 0x0d, 0x00, 0x04, 0x80, 0x00,
    0x00, 0x0c, 0x00, 0x08, 0x00, 0x00};

// Canonical TX-validation source MAC — shared with txdemo/main.cpp,
// demo/main.cpp's `<devourer-tx-hit>` matcher, tests/regress.py (CANONICAL_SA)
// and tests/inject_beacon.py. Change all of them together if it ever moves.
static const uint8_t kCanonicalSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};

// 802.11 probe-request mgmt header (24 bytes), mirroring txdemo/main.cpp's
// frame. DATA frames (ToDS) get silently NAKed by the chip in monitor mode —
// the plan's "Data frame" wording is superseded by txdemo's hard-won
// probe-request, which the SA matcher recognises identically (addr2 at +10).
static std::vector<uint8_t> build_dot11_probe_req() {
  std::vector<uint8_t> h = {
      0x40, 0x00,             // frame control: mgmt / probe request
      0x00, 0x00,             // duration
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff,  // addr1 / DA (broadcast)
  };
  h.insert(h.end(), kCanonicalSa, kCanonicalSa + 6);  // addr2 / SA
  h.insert(h.end(), kCanonicalSa, kCanonicalSa + 6);  // addr3 / BSSID
  h.push_back(0x80);  // seq_ctl
  h.push_back(0x00);
  return h;  // 24 bytes
}

static bool read_file(const std::string &path, std::vector<uint8_t> &out) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  out.assign(std::istreambuf_iterator<char>(f),
             std::istreambuf_iterator<char>());
  return true;
}

int main(int argc, char **argv) {
  auto logger = std::make_shared<Logger>();

  std::string psdu_path;
  long count = -1;        // -1 == loop forever (like txdemo)
  int interval_ms = 2;    // ~500 fps, gentle on the bulk EP
  long termux_fd = 0;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--psdu" && i + 1 < argc) {
      psdu_path = argv[++i];
    } else if (a == "--count" && i + 1 < argc) {
      count = std::strtol(argv[++i], nullptr, 0);
    } else if (a == "--interval-ms" && i + 1 < argc) {
      interval_ms = std::atoi(argv[++i]);
    } else {
      // A bare numeric arg is the Termux USB fd (libusb_wrap_sys_device).
      char *end = nullptr;
      long v = std::strtol(a.c_str(), &end, 0);
      if (end && *end == '\0' && v > 0) termux_fd = v;
    }
  }

  if (psdu_path.empty()) {
    logger->error("usage: PrecoderDemo --psdu <shaped.bin> [--count N] "
                  "[--interval-ms MS]");
    return 2;
  }

  std::vector<uint8_t> psdu;
  if (!read_file(psdu_path, psdu) || psdu.empty()) {
    logger->error("cannot read PSDU file (or it is empty): {}", psdu_path);
    return 2;
  }
  logger->info("loaded {} PSDU bytes from {}", psdu.size(), psdu_path);

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
      logger->info("DEVOURER_VID={:04x} (overriding default VID)", target_vid);
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
  auto rtlDevice = wifi_driver.CreateRtlDevice(handle, nullptr, usb_lock);

  // 2.4 GHz channel 6 is the plan's matrix-validated cell for these chips.
  int channel = 6;
  if (const char *ch_env = std::getenv("DEVOURER_CHANNEL")) {
    channel = std::atoi(ch_env);
    logger->info("DEVOURER_CHANNEL set — tuning TX to channel {}", channel);
  }

  rtlDevice->SetTxPower(40);
  rtlDevice->InitWrite(SelectedChannel{.Channel = static_cast<uint8_t>(channel),
                                       .ChannelOffset = 0,
                                       .ChannelWidth = CHANNEL_WIDTH_20});
  sleep(2);

  // [ radiotap (13) | 802.11 probe-req hdr (24) | shaped PSDU ]. The MAC header
  // is 24 bytes; with the PHY's 16-bit SERVICE prefix that is 16 + 24*8 = 208
  // scrambled-stream bits before the body. For *exact* per-subcarrier control
  // the encoder must be run with --offset 208 and the matching entry_state
  // (legacy N_DBPS=24 -> the body starts mid-symbol, so the first fully
  // controllable symbol is the next 24-bit boundary; see
  // tools/precoder/README.md). For a byte-level round-trip (Phase A) the offset
  // is irrelevant — the shaped bytes come back verbatim.
  auto dot11 = build_dot11_probe_req();
  std::vector<uint8_t> tx_buf;
  tx_buf.reserve(sizeof(kRadiotapLegacy6M) + dot11.size() + psdu.size());
  tx_buf.insert(tx_buf.end(), kRadiotapLegacy6M,
                kRadiotapLegacy6M + sizeof(kRadiotapLegacy6M));
  tx_buf.insert(tx_buf.end(), dot11.begin(), dot11.end());
  tx_buf.insert(tx_buf.end(), psdu.begin(), psdu.end());
  logger->info("TX frame (legacy 6M OFDM): {} radiotap + {} hdr + {} PSDU = {} "
               "bytes total", sizeof(kRadiotapLegacy6M), dot11.size(),
               psdu.size(), tx_buf.size());

  long tx_count = 0;
  while (count < 0 || tx_count < count) {
    bool ok = rtlDevice->send_packet(tx_buf.data(), tx_buf.size());
    ++tx_count;
    if (tx_count <= 10 || tx_count % 500 == 0) {
      printf("<precoder-tx>TX #%ld ok=%d\n", tx_count, ok ? 1 : 0);
      fflush(stdout);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
  }

  libusb_release_interface(handle, 0);
  libusb_close(handle);
  libusb_exit(context);
  return 0;
}
