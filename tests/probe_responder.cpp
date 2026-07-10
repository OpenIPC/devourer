// probe_responder.cpp — devourer as an AP that answers ACTIVE-SCAN probe
// requests, like a kernel AP. Full-duplex (InitWrite + StartRxLoop, so needs a
// clean TX+RX adapter — e.g. 8822CU): the RX callback matches a probe-request and
// queues the requester; the MAIN thread send_packet's a probe-response (send_packet
// must NOT be called from the RX event thread — libusb returns BUSY). No beacon is
// aired, so a scan hit proves the RESPONSE path (userspace RX->TX ~few ms fits the
// active-scan dwell; management-frame timing is tens of ms, unlike SIFS ACKs).
//
// PROVEN: a real Linux station (rtw88, wlp13s0u1u4) `iw scan` lists "devourerAP"
// via the probe response with NO beacon — devourer answers active scans like an AP.
// This de-risks the AP-side handshake (probe-resp -> auth -> assoc).
//
// Build: g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/probe_responder.cpp \
//   examples/common/env_config.cpp build/libdevourer.a \
//   $(pkg-config --cflags --libs libusb-1.0) -lpthread -o build/probe_responder
// Run: sudo DEVOURER_PID=0xc812 DEVOURER_CHANNEL=6 DEVOURER_TX_WITH_RX=thread \
//   build/probe_responder [sec]   # then `iw dev <other-kernel-if> scan freq 2437`
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <memory>
#include <thread>
#include <vector>
#include <array>
#include <unistd.h>
#include <libusb.h>
#include "RadiotapBuilder.h"
#include "RxPacket.h"
#include "SelectedChannel.h"
#include "TxMode.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"

#include <mutex>
static const uint8_t kBssid[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
static IRtlDevice* g_dev = nullptr;
static std::vector<uint8_t> g_rt;   // radiotap prefix (6M)
static uint8_t g_chan = 6;
static std::atomic<uint64_t> g_reqs{0}, g_resps{0};
// Requester MACs queued by the RX callback, drained by the main (TX) thread —
// send_packet must NOT be called from the RX event thread (libusb BUSY).
static std::mutex g_q_mu;
static std::vector<std::array<uint8_t, 6>> g_q;

static void on_rx(const Packet& p) {
  if (p.Data.size() < 24 || p.RxAtrib.crc_err) return;
  if (p.Data[0] != 0x40) return;                 // FC: mgmt / probe-request
  const uint8_t* req = p.Data.data() + 10;        // addr2 = requester
  g_reqs.fetch_add(1);
  std::array<uint8_t, 6> a{req[0],req[1],req[2],req[3],req[4],req[5]};
  std::lock_guard<std::mutex> lk(g_q_mu);
  if (g_q.size() < 64) g_q.push_back(a);
}

static void send_resp(const std::array<uint8_t, 6>& req) {
  std::vector<uint8_t> m = {
      0x50, 0x00, 0x00, 0x00,                     // FC probe-resp + dur
      req[0], req[1], req[2], req[3], req[4], req[5],   // addr1 = requester
      kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5],  // addr2 = BSSID
      kBssid[0],kBssid[1],kBssid[2],kBssid[3],kBssid[4],kBssid[5],  // addr3 = BSSID
      0x00, 0x00,                                 // seq
      0,0,0,0,0,0,0,0,                            // timestamp
      0x64, 0x00,                                 // beacon interval 100 TU
      0x01, 0x00,                                 // capability ESS
      0x00, 0x0a, 'd','e','v','o','u','r','e','r','A','P',   // SSID
      0x01, 0x08, 0x82,0x84,0x8b,0x96,0x24,0x30,0x48,0x6c,   // rates
      0x03, 0x01, g_chan};                        // DS param
  std::vector<uint8_t> f;
  f.reserve(g_rt.size() + m.size());
  f.insert(f.end(), g_rt.begin(), g_rt.end());
  f.insert(f.end(), m.begin(), m.end());
  if (g_dev->send_packet(f.data(), f.size())) g_resps.fetch_add(1);
}

int main(int argc, char** argv) {
  int sec = argc > 1 ? atoi(argv[1]) : 40;
  if (const char* c = std::getenv("DEVOURER_CHANNEL")) g_chan = (uint8_t)atoi(c);
  auto logger = std::make_shared<Logger>();
  apply_logging_env(*logger);
  libusb_context* ctx = nullptr; libusb_init(&ctx);
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
  uint16_t vid = 0x0bda, pid = 0xc812;
  if (const char* v = std::getenv("DEVOURER_VID")) vid = (uint16_t)strtoul(v, 0, 0);
  if (const char* p = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(p, 0, 0);
  auto* h = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!h) { fprintf(stderr, "open %04x:%04x fail\n", vid, pid); return 1; }
  std::shared_ptr<devourer::UsbDeviceLock> lk;
  if (devourer::claim_interface_then_reset(h, 0, logger, true, lk) != 0) return 1;
  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRtlDevice(h, ctx, lk, devourer_config_from_env());
  g_dev = dev.get();
  if (!g_dev) return 1;
  g_rt = devourer::build_stream_radiotap(devourer::parse_tx_mode_str("6M"));
  g_dev->InitWrite(SelectedChannel{g_chan, 0, CHANNEL_WIDTH_20});
  std::thread rx([&]{ g_dev->StartRxLoop(on_rx); });
  fprintf(stderr, "probe-responder up on ch%d (BSSID devourerAP, NO beacon). %ds.\n",
          g_chan, sec);
  // Main (TX) thread: drain queued requesters and answer — off the RX event thread.
  auto end = std::chrono::steady_clock::now() + std::chrono::seconds(sec);
  while (std::chrono::steady_clock::now() < end) {
    std::vector<std::array<uint8_t, 6>> batch;
    { std::lock_guard<std::mutex> lk(g_q_mu); batch.swap(g_q); }
    for (auto& r : batch) send_resp(r);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  fprintf(stderr, "probe-reqs seen=%llu  probe-resps sent=%llu\n",
          (unsigned long long)g_reqs.load(), (unsigned long long)g_resps.load());
  _exit(0);
}
