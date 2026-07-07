// svctx — TID-aware unequal-error-protection (UEP) injector.
//
// Reads a sequence of length-prefixed HEVC NAL units from stdin, classifies
// each by its temporal_id / criticality (svc_tx.h), and injects it at the PHY
// rate its layer deserves (robust MCS for base/IDR, fast MCS for enhancement).
// The per-packet radiotap carries the rate, so each frame is an independent
// PHY mode (radiotap-per-packet wins; see RtlJaguarDevice::send_packet).
//
// Input protocol (stdin):  <u32_le len><len bytes of NAL>  ...  EOF
//   (Length-prefixed = the AVCC/HVCC/MP4 framing many depacketizers already
//    produce. A production link would feed Annex-B or RTP instead — same NAL
//    header parse, trivial framing adapter.)
//
// All records are read up front, then injected in a continuous loop (so a
// receiver can capture a stable rate histogram). A live deployment would
// inject each NAL as it arrives, once.
//
// Usage:
//   DEVOURER_PID=0x8812 DEVOURER_CHANNEL=6 ./build/svctx \
//       [--mtu N] [--gap-us US] < nals.bin

#ifndef NOMINMAX
#define NOMINMAX  // keep <windows.h> from defining min()/max() macros (breaks std::min)
#endif

#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#if defined(_MSC_VER)
  #include <io.h>
  #include <fcntl.h>
  #include <windows.h>
  typedef int pid_t;
  #define sleep(seconds) Sleep((seconds)*1000)
#elif defined(__MINGW32__) || defined(__MINGW64__)
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
#include "RtlUsbAdapter.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"
#include "stream_stdin.h"
#include "svc_tx.h"

#define USB_VENDOR_ID 0x0bda
static constexpr uint16_t kRealtekProductIds[] = {
    0x8812, 0x0811, 0xa811, 0xb811, 0x8813,
};
static const uint8_t kCanonicalSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};

static std::vector<uint8_t> build_dot11_probe_req() {
  std::vector<uint8_t> h = {0x40, 0x00, 0x00, 0x00,
                            0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  h.insert(h.end(), kCanonicalSa, kCanonicalSa + 6);
  h.insert(h.end(), kCanonicalSa, kCanonicalSa + 6);
  h.push_back(0x80);
  h.push_back(0x00);
  return h;
}

static const char* mode_str(const devourer::TxMode& m) {
  static char buf[24];
  if (m.mode == devourer::TxMode::Mode::HT)
    std::snprintf(buf, sizeof(buf), "MCS%u/%u%s%s%s", m.ht_mcs, m.bw_mhz,
                  m.sgi ? "/SGI" : "", m.ldpc ? "/LDPC" : "",
                  m.stbc ? "/STBC" : "");
  else if (m.mode == devourer::TxMode::Mode::VHT)
    std::snprintf(buf, sizeof(buf), "VHT%uSS_MCS%u/%u", m.vht_nss, m.vht_mcs,
                  m.bw_mhz);
  else
    std::snprintf(buf, sizeof(buf), "%uM", m.legacy_rate_500kbps / 2);
  return buf;
}

int main(int argc, char** argv) {
  auto logger = std::make_shared<Logger>();
  apply_logging_env(*logger); /* DEVOURER_LOG_LEVEL / DEVOURER_EVENTS / ... */

  size_t mtu = 1400;
  long gap_us = 2000;
  long termux_fd = 0;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--mtu" && i + 1 < argc)
      mtu = static_cast<size_t>(std::strtoul(argv[++i], nullptr, 0));
    else if (a == "--gap-us" && i + 1 < argc)
      gap_us = std::strtol(argv[++i], nullptr, 0);
    else {
      char* end = nullptr;
      long v = std::strtol(a.c_str(), &end, 0);
      if (end && *end == '\0' && v > 0) termux_fd = v;
    }
  }
  if (mtu < 16) mtu = 16;
  stream_stdin::set_stdin_binary();

  libusb_context* context = nullptr;
  libusb_device_handle* handle = nullptr;
  int rc;
  if (termux_fd > 0) {
    libusb_set_option(NULL, LIBUSB_OPTION_NO_DEVICE_DISCOVERY);
    libusb_set_option(NULL, LIBUSB_OPTION_WEAK_AUTHORITY);
    libusb_init(&context);
    rc = libusb_wrap_sys_device(context, (intptr_t)termux_fd, &handle);
    if (rc < 0) { logger->error("wrap_sys_device: {}", rc); return 1; }
  } else {
    rc = libusb_init(&context);
    if (rc < 0) return rc;
    uint16_t target_pid = 0;
    if (const char* p = std::getenv("DEVOURER_PID"))
      target_pid = static_cast<uint16_t>(std::strtoul(p, nullptr, 0));
    uint16_t target_vid = USB_VENDOR_ID;
    if (const char* v = std::getenv("DEVOURER_VID"))
      target_vid = static_cast<uint16_t>(std::strtoul(v, nullptr, 0));
    for (uint16_t pid : kRealtekProductIds) {
      if (target_pid != 0 && pid != target_pid) continue;
      handle = libusb_open_device_with_vid_pid(context, target_vid, pid);
      if (handle != NULL) { logger->info("Opened {:04x}:{:04x}", target_vid, pid); break; }
    }
    if (handle == NULL && target_pid != 0)
      handle = libusb_open_device_with_vid_pid(context, target_vid, target_pid);
    if (handle == NULL) { logger->error("No supported device"); libusb_exit(context); return 1; }
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

  int channel = 6;
  if (const char* ch = std::getenv("DEVOURER_CHANNEL")) channel = std::atoi(ch);
  /* DEVOURER_TX_POWER: flat TXAGC index. Unset = the family's calibrated
   * default (SetTxPower is now a real flat override on every generation). */
  if (const char* p = std::getenv("DEVOURER_TX_POWER"))
    rtlDevice->SetTxPower(static_cast<uint8_t>(std::atoi(p)));
  rtlDevice->InitWrite(SelectedChannel{.Channel = static_cast<uint8_t>(channel),
                                       .ChannelOffset = 0,
                                       .ChannelWidth = CHANNEL_WIDTH_20});
  sleep(2);

  // Policy (DEVOURER_SVC_LADDER or the built-in default) + precomputed
  // radiotaps (one per rung — built once, reused).
  const svc::LayerPolicy policy = svc::policy_from_env();
  std::vector<uint8_t> rt_crit = devourer::build_stream_radiotap(policy.critical);
  std::vector<std::vector<uint8_t>> rt_tid;
  for (const auto& m : policy.by_tid)
    rt_tid.push_back(devourer::build_stream_radiotap(m));
  logger->info("SVC UEP policy: critical={}", mode_str(policy.critical));
  for (size_t i = 0; i < policy.by_tid.size(); ++i)
    logger->info("  T{} -> {}", i, mode_str(policy.by_tid[i]));

  const auto dot11 = build_dot11_probe_req();

  // Read all length-prefixed NALs up front.
  std::vector<std::vector<uint8_t>> nals;
  while (true) {
    uint8_t lb[4];
    if (stream_stdin::read_exact(stdin, lb, 4) != stream_stdin::ReadResult::Ok)
      break;
    uint32_t len = lb[0] | (lb[1] << 8) | (lb[2] << 16) | ((uint32_t)lb[3] << 24);
    if (len == 0 || len > 200000) break;
    std::vector<uint8_t> nal(len);
    if (stream_stdin::read_exact(stdin, nal.data(), len) !=
        stream_stdin::ReadResult::Ok)
      break;
    nals.push_back(std::move(nal));
  }
  if (nals.empty()) { logger->error("no NALs on stdin"); return 2; }
  logger->info("svctx: {} NALs, mtu={}, ch{} — looping", nals.size(), mtu,
               channel);

  long sent[9] = {0};  // [0..7] per TID, [8] = critical
  long frames = 0;
  while (true) {
    for (const auto& nal : nals) {
      svc::NalInfo info = svc::parse_hevc_nal(nal.data(), nal.size());
      const std::vector<uint8_t>& rt =
          info.critical ? rt_crit
          : rt_tid.empty() ? rt_crit
          : rt_tid[info.tid < rt_tid.size() ? info.tid : rt_tid.size() - 1];
      sent[info.critical ? 8 : (info.tid > 7 ? 7 : info.tid)]++;
      // Fragment the NAL to the radio MTU; every fragment carries the layer rate.
      for (size_t off = 0; off < nal.size(); off += mtu) {
        size_t n = std::min(mtu, nal.size() - off);
        std::vector<uint8_t> frame;
        frame.reserve(rt.size() + dot11.size() + n);
        frame.insert(frame.end(), rt.begin(), rt.end());
        frame.insert(frame.end(), dot11.begin(), dot11.end());
        frame.insert(frame.end(), nal.begin() + off, nal.begin() + off + n);
        rtlDevice->send_packet(frame.data(), frame.size());
        if (gap_us > 0)
          std::this_thread::sleep_for(std::chrono::microseconds(gap_us));
      }
      /* Per-layer injection counters. JSON keys are lowercase (t3plus stands
       * in for "T3+": temporal layers 3..7 aggregated). */
      if (++frames % 200 == 0) {
        devourer::Ev(logger->events(), "svc.stats")
            .f("frames", frames)
            .f("crit", sent[8])
            .f("t0", sent[0])
            .f("t1", sent[1])
            .f("t2", sent[2])
            .f("t3plus", sent[3] + sent[4] + sent[5] + sent[6] + sent[7]);
      }
    }
  }
  return 0;
}
