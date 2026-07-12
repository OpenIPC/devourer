// txegress_witness.cpp — benchmark the "read TSF after send to approximate the
// TX-egress time" micro-option against a real hardware egress reference, using a
// single witness receiver.
//
// Every frame carries its own TX timestamp, so one receiver captures both the TX
// stamp and a true-air proxy from the SAME packet — no cross-node clock join:
//   * software stamp  — the TD tag's tx_tsf (the transmitter's ReadTsf() near
//     the send call), the micro-option under test.
//   * hardware stamp  — a beacon's bytes 24-31 (the MAC-inserted TSF, latched
//     when the frame is actually clocked out — a genuine hw egress timestamp).
//   * rx_tsfl         — this receiver's hardware RX timestamp (RxAtrib.tsfl),
//     latched in the MAC at receive: a low-jitter proxy for true on-air time.
//
// For each decoded frame from the transmitter we emit {seq, tx_sw, tx_hw,
// rx_tsfl, host_ns}. txegress_analyze.py then least-squares fits rx_tsfl against
// each TX stamp; the residual RMS is that stamp's jitter versus true air. The
// software stamp's residual minus the hardware stamp's residual (in quadrature)
// is exactly the error the micro-option injects — the number the literature
// lacks.
//
// Build: g++ -std=c++20 -O2 -Isrc -Iexamples/common -Iexamples/tdma \
//   tests/txegress_witness.cpp examples/common/env_config.cpp build/libdevourer.a \
//   $(pkg-config --cflags --libs libusb-1.0) -lpthread -o build/txegress_witness
// Run: sudo DEVOURER_PID=0x012d DEVOURER_CHANNEL=6 build/txegress_witness [nframes]
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <thread>
#include <unistd.h>

#include <libusb.h>

#include "RxPacket.h"
#include "SelectedChannel.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"
#include "tdma.h"

static std::atomic<int> g_n{0};
static int g_want = 400;

static int64_t host_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

int main(int argc, char** argv) {
  if (argc > 1) g_want = atoi(argv[1]);
  auto logger = std::make_shared<Logger>();
  libusb_context* ctx = nullptr;
  libusb_init(&ctx);
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);

  uint16_t vid = 0x0bda, pid = 0x012d;
  if (const char* p = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(p, 0, 0);
  if (const char* v = std::getenv("DEVOURER_VID")) vid = (uint16_t)strtoul(v, 0, 0);
  uint8_t ch = 6;
  if (const char* c = std::getenv("DEVOURER_CHANNEL")) ch = atoi(c);

  auto* h = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!h) { fprintf(stderr, "open fail %04x:%04x\n", vid, pid); return 1; }
  std::shared_ptr<devourer::UsbDeviceLock> lk;
  if (devourer::claim_interface_then_reset(h, 0, logger, true, lk) != 0) return 1;
  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRtlDevice(h, ctx, lk, devourer_config_from_env());
  if (!dev) return 1;

  auto cb = [](const Packet& p) {
    if (p.RxAtrib.crc_err || p.Data.size() < 32) return;
    // Parse the TD tag (software tx_tsf); accept only frames that carry it.
    tdma::Parsed td = tdma::parse_frame(p.Data.data(), p.Data.size());
    uint64_t tx_sw = 0, tx_hw = 0, tx_host = 0;
    static const uint8_t kSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
    if (td.ok) {
      tx_sw = td.tx_tsf;          // software-mode: the transmitter's ReadTsf()
      tx_host = td.host_ns;       // TD v2: TX host steady_clock at send (0 on v1)
    } else if (p.Data[0] == 0x80 && std::memcmp(p.Data.data() + 10, kSa, 6) == 0) {
      // hwbeacon-mode standard beacon (our SA): bytes 24-31 = MAC-inserted egress TSF
      for (int i = 0; i < 8; ++i) tx_hw |= (uint64_t)p.Data[24 + i] << (8 * i);
    } else {
      return;                     // not one of ours
    }
    ++g_n;
    printf("{\"ev\":\"txeg\",\"seq\":%u,\"tx_sw\":%llu,\"tx_hw\":%llu,"
           "\"tx_host_ns\":%llu,\"rx_tsfl\":%u,\"host_ns\":%lld}\n",
           td.seq, (unsigned long long)tx_sw, (unsigned long long)tx_hw,
           (unsigned long long)tx_host, p.RxAtrib.tsfl, (long long)host_ns());
    fflush(stdout);
  };

  std::thread rx([&] { dev->Init(cb, SelectedChannel{ch, 0, CHANNEL_WIDTH_20}); });
  while (g_n.load() < g_want) std::this_thread::sleep_for(std::chrono::milliseconds(50));
  fprintf(stderr, "txegress_witness: %d frames\n", g_n.load());
  _exit(0);
}
