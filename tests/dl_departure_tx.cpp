// dl_departure_tx.cpp — DL-departure probe transmitter (M0 contract 1): airs TD
// v2 data frames over ANY USB adapter, each carrying BOTH submit-time stamps —
//   * tx_tsf  — ReadTsf() near the send call (the TX hardware clock), and
//   * host_ns — steady_clock immediately before send_packet (the HOST clock a
//     slot scheduler actually controls).
// Paired with tests/txegress_witness on a co-located receiver: the witness's
// hardware rx_tsfl is the true-air proxy, txegress_analyze.py fits it against
// each stamp, and the host_ns residual tail IS the send_packet→air guard-time
// contract for this adapter/transport. PCIe counterpart:
// tests/pcie_txegress_tx.cpp; matrix orchestration: tests/dl_departure_matrix.sh.
//
// Build: g++ -std=c++20 -O2 -Isrc -Iexamples/common -Iexamples/tdma \
//   tests/dl_departure_tx.cpp examples/common/env_config.cpp build/libdevourer.a \
//   $(pkg-config --cflags --libs libusb-1.0) -lpthread -o build/dl_departure_tx
// Run: sudo DEVOURER_PID=0x8812 DEVOURER_CHANNEL=6 build/dl_departure_tx [secs]
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <thread>

#include <libusb.h>

#include "RadiotapBuilder.h"
#include "SelectedChannel.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"
#include "tdma.h"

static uint64_t host_ns() {
  return (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

int main(int argc, char** argv) {
  int secs = argc > 1 ? atoi(argv[1]) : 30;
  int gap_us = 8000;
  if (const char* g = std::getenv("GAP_US")) gap_us = atoi(g);
  uint16_t vid = 0x0bda, pid = 0x8812;
  if (const char* p = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(p, 0, 0);
  if (const char* v = std::getenv("DEVOURER_VID")) vid = (uint16_t)strtoul(v, 0, 0);
  uint8_t ch = 6;
  if (const char* c = std::getenv("DEVOURER_CHANNEL")) ch = atoi(c);

  auto logger = std::make_shared<Logger>();
  libusb_context* ctx = nullptr;
  libusb_init(&ctx);
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
  auto* h = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!h) { fprintf(stderr, "open fail %04x:%04x\n", vid, pid); return 1; }
  std::shared_ptr<devourer::UsbDeviceLock> lk;
  if (devourer::claim_interface_then_reset(h, devourer::find_wifi_interface(h), logger, true, lk) != 0) return 1;
  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRtlDevice(h, ctx, lk, devourer_config_from_env());
  if (!dev) return 1;

  dev->InitWrite(SelectedChannel{ch, 0, CHANNEL_WIDTH_20});
  dev->SetCcaMode(true);  // disable EDCCA — suppress CSMA backoff so the residual
                          // reflects the transport + MAC-pipeline floor
  std::this_thread::sleep_for(std::chrono::seconds(2));

  const auto rt = devourer::build_stream_radiotap(
      devourer::parse_tx_mode_str("6M"));               // legacy 6M, widely heard
  uint32_t seq = 0;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(secs);
  while (std::chrono::steady_clock::now() < deadline) {
    uint64_t tsf = dev->ReadTsf();                       // TX-clock submit stamp
    auto f = tdma::build_frame(rt, tdma::Class::Marker, seq++, 0, tsf, host_ns());
    // Re-stamp host_ns as the LAST thing before send: bytes [20..28) of the tag.
    uint64_t hn = host_ns();
    for (int i = 0; i < 8; ++i)
      f[f.size() - 8 + i] = (uint8_t)(hn >> (8 * i));
    dev->send_packet(f.data(), f.size());
    std::this_thread::sleep_for(std::chrono::microseconds(gap_us));
  }
  fprintf(stderr, "dl_departure_tx: %u frames on ch%d (%04x:%04x)\n", seq, ch,
          vid, pid);
  return 0;
}
