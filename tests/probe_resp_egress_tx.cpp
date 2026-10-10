// probe_resp_egress_tx.cpp — Phase-0 gate for the stream-timing marker: does
// the MAC overwrite the 8-byte timestamp field of a HOST-INJECTED management
// frame with its live TSF at egress? src/RxPacket.h:TxEgressTsf() reads that
// field for beacons (FC 0x80) and probe responses (FC 0x50); the beacon case is
// bench-proven for the hardware TBTT beacon (StartBeacon), never for a frame
// pushed through send_packet. This program alternates injected probe responses
// and injected beacons whose timestamp field is a CONSTANT; the witness
// (rxdemo, rx.frame tx_tsf) shows either that constant (not stamped) or a live
// TSF. Once a second it also logs ReadTsf() so the analyzer can tell a live
// stamp from garbage. Driven by tests/probe_resp_egress_tsf_check.sh.
//
// Build: see tests/dl_departure_matrix.sh build_local (same recipe).
// Run:   sudo DEVOURER_PID=0xc812 DEVOURER_CHANNEL=6 build/probe_resp_egress_tx [secs]
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

#include <libusb.h>

#include "RadiotapBuilder.h"
#include "RtlAdapter.h"
#include "SelectedChannel.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"

static const uint8_t kSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
static constexpr uint64_t kConstTs = 0x1122334455667788ull;

static std::vector<uint8_t> build(const std::vector<uint8_t>& rt, uint8_t fc0,
                                  uint16_t seq) {
  std::vector<uint8_t> f(rt);
  const uint8_t hdr[24] = {fc0, 0x00, 0x00, 0x00,
                           0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                           kSa[0], kSa[1], kSa[2], kSa[3], kSa[4], kSa[5],
                           kSa[0], kSa[1], kSa[2], kSa[3], kSa[4], kSa[5],
                           (uint8_t)(seq << 4), (uint8_t)(seq >> 4)};
  f.insert(f.end(), hdr, hdr + 24);
  for (int i = 0; i < 8; ++i) f.push_back((uint8_t)(kConstTs >> (8 * i)));
  f.push_back(0x64); f.push_back(0x00);           // beacon interval 100 TU
  f.push_back(0x21); f.push_back(0x04);           // cap: ESS + short preamble
  static const char ssid[] = "devourer-ts";
  f.push_back(0x00); f.push_back((uint8_t)(sizeof(ssid) - 1));
  f.insert(f.end(), ssid, ssid + sizeof(ssid) - 1);
  f.back() = fc0 == 0x50 ? 'P' : 'B';              // "devourer-tP"/"-tB": the witness tells the two apart by SSID
  f.push_back(0x01); f.push_back(0x01); f.push_back(0x8c);  // rates: 6M basic
  f.push_back(0x03); f.push_back(0x01); f.push_back(0x00);  // DS param, patched below
  return f;
}

int main(int argc, char** argv) {
  int secs = argc > 1 ? atoi(argv[1]) : 15;
  int gap_us = 10000;
  if (const char* g = std::getenv("GAP_US")) gap_us = atoi(g);
  uint16_t vid = 0x0bda, pid = 0xc812;
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
  auto dev = wifi.CreateRadio(h, ctx, lk, devourer_config_from_env());
  if (!dev) return 1;

  dev->InitWrite(SelectedChannel{ch, 0, CHANNEL_WIDTH_20});
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // PR0_REGS=1: a second RtlAdapter on the same handle reads the two TSF
  // ports + beacon control every 500 ms (test-only; never on a send path).
  std::unique_ptr<RtlAdapter> regs;
  if (std::getenv("PR0_REGS")) regs = std::make_unique<RtlAdapter>(h, logger, ctx, lk);
  const auto rt = devourer::build_stream_radiotap(devourer::parse_tx_mode_str("6M"));
  auto presp = build(rt, 0x50, 0);
  auto bcn = build(rt, 0x80, 0);
  presp.back() = ch; bcn.back() = ch;

  uint32_t n = 0;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(secs);
  auto next_tsf = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() < deadline) {
    if (std::chrono::steady_clock::now() >= next_tsf) {
      uint64_t tsf = 0;
      try { tsf = dev->ReadTsf(); } catch (...) { tsf = 0; }
      // JSONL on stdout so the analyzer can bracket the witness's tx_tsf.
      printf("{\"ev\":\"pr0.tsf\",\"tsf\":%llu,\"sent\":%u}\n",
             (unsigned long long)tsf, n);
      fflush(stdout);
      next_tsf += std::chrono::seconds(1);
    }
    static auto next_regs = std::chrono::steady_clock::now();
    if (regs && std::chrono::steady_clock::now() >= next_regs) {
      try {
        uint64_t t0 = regs->rtw_read32(0x0560) | ((uint64_t)regs->rtw_read32(0x0564) << 32);
        uint64_t t1 = regs->rtw_read32(0x0568) | ((uint64_t)regs->rtw_read32(0x056c) << 32);
        printf("{\"ev\":\"pr0.regs\",\"tsftr\":%llu,\"tsftr1\":%llu,\"bcn_ctrl\":\"0x%04x\",\"dual_rst\":\"0x%02x\",\"sent\":%u}\n",
               (unsigned long long)t0, (unsigned long long)t1,
               regs->rtw_read16(0x0550), regs->rtw_read8(0x0553), n);
        fflush(stdout);
      } catch (...) {}
      next_regs += std::chrono::milliseconds(500);
    }
    auto& f = (n & 1) ? bcn : presp;
    dev->send_packet(f.data(), f.size());
    ++n;
    std::this_thread::sleep_for(std::chrono::microseconds(gap_us));
  }
  dev->Stop();
  fprintf(stderr, "probe_resp_egress_tx: %u frames (half 0x50, half 0x80) on ch%d (%04x:%04x)\n",
          n, ch, vid, pid);
  return 0;
}
