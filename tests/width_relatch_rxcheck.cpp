// width_relatch_rxcheck.cpp — does RX still decode after a width change that
// keeps the RF18 channel byte? Brings the device up at one width on centre
// channel 36, counts CRC-good frames per second, switches width via
// SetMonitorChannel on the same centre, and counts again. Needs nothing but
// ambient traffic on channel 36 (any AP beaconing there). Also prints, per
// second, the mean RSSI (paths A/B) and path-A SNR of the beacons of the
// most-heard beaconing transmitter: a fixed reference signal, so a gain loss
// shows even while strong frames still decode. Compare with a fresh bring-up
// at the target width.
//
// Build: g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/width_relatch_rxcheck.cpp \
//   examples/common/env_config.cpp build/libdevourer.a \
//   $(pkg-config --cflags --libs libusb-1.0) -lpthread -o build/width_relatch_rxcheck
// Run: DEVOURER_VID=0x0bda DEVOURER_PID=0xb812 build/width_relatch_rxcheck <from> <to>
//   from/to in MHz: 20, 40 or 80, all centred on channel 36 (40: primary 34,
//   80: primary 30). from == to is the fresh-bring-up reference.
#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <vector>
#include <thread>
#include <unistd.h>

#include <libusb.h>

#include "RxPacket.h"
#include "SelectedChannel.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"

static std::atomic<uint64_t> g_ok{0};
static std::atomic<int> g_sec{0}; // second index the RX callback files beacons under
struct Beacon {
  int sec;
  std::array<uint8_t, 6> sa;
  int rssi_a, rssi_b, snr_a;
};
static std::mutex g_mu;
static std::vector<Beacon> g_beacons;

static SelectedChannel centre36(int mhz) {
  if (mhz == 40)
    return SelectedChannel{34, 1, CHANNEL_WIDTH_40};
  if (mhz == 80)
    return SelectedChannel{30, 1, CHANNEL_WIDTH_80};
  return SelectedChannel{36, 0, CHANNEL_WIDTH_20};
}

int main(int argc, char **argv) {
  const int from = argc > 1 ? atoi(argv[1]) : 40;
  const int to = argc > 2 ? atoi(argv[2]) : 20;

  auto logger = std::make_shared<Logger>();
  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) < 0) return 1;
  uint16_t vid = 0x0bda, pid = 0;
  if (const char *e = std::getenv("DEVOURER_VID")) vid = (uint16_t)strtoul(e, 0, 0);
  if (const char *e = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(e, 0, 0);
  auto *h = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!h) { fprintf(stderr, "open %04x:%04x failed\n", vid, pid); return 1; }
  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (devourer::claim_interface_then_reset(h, devourer::find_wifi_interface(h), logger, true, lock) != 0) return 1;
  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRadio(h, ctx, lock, devourer_config_from_env());
  if (!dev) return 1;

  auto cb = [](const Packet &p) {
    if (p.RxAtrib.crc_err)
      return;
    g_ok.fetch_add(1, std::memory_order_relaxed);
    if (p.Data.size() >= 16 && p.Data[0] == 0x80) { // beacon
      Beacon b{g_sec.load(), {}, p.RxAtrib.rssi[0], p.RxAtrib.rssi[1], p.RxAtrib.snr[0]};
      std::memcpy(b.sa.data(), p.Data.data() + 10, 6);
      std::lock_guard<std::mutex> lk(g_mu);
      g_beacons.push_back(b);
    }
  };
  std::atomic<bool> rx_done{false};
  std::thread rx([&] {
    dev->Init(cb, centre36(from));
    rx_done = true;
  });
  std::this_thread::sleep_for(std::chrono::seconds(2)); // settle bring-up

  auto second = [] {
    g_sec.fetch_add(1);
    g_ok.store(0);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return (unsigned long long)g_ok.load();
  };
  printf("%d MHz:", from);
  for (int i = 0; i < 3; i++) printf(" %llu", second());
  if (to != from) {
    dev->SetMonitorChannel(centre36(to));
    printf(" | -> %d MHz:", to);
  } else {
    printf(" |");
  }
  for (int i = 0; i < 6; i++) printf(" %llu", second());
  printf("\n");
  /* StartRxLoop clears the stop flag on entry, so repeat the stop until the
   * RX thread has really ended, then tear down in order. */
  while (!rx_done) {
    dev->StopRxLoop();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  rx.join();
  dev->Stop();

  /* Beacon RSSI/SNR of the most-heard beaconing transmitter, per second
   * (seconds 1-3 before the switch, 4-9 after). */
  std::map<std::array<uint8_t, 6>, int> n;
  for (const auto &b : g_beacons) n[b.sa]++;
  if (n.empty()) {
    printf("  no beacons\n");
    return 0;
  }
  auto top = n.begin();
  for (auto it = n.begin(); it != n.end(); ++it)
    if (it->second > top->second) top = it;
  printf("  beacons of %02x:..:%02x, per second rssiA/rssiB/snrA (n):",
         top->first[0], top->first[5]);
  for (int sec = 1; sec <= 9; sec++) {
    int c = 0, ra = 0, rb = 0, sn = 0;
    for (const auto &b : g_beacons)
      if (b.sec == sec && b.sa == top->first) { c++; ra += b.rssi_a; rb += b.rssi_b; sn += b.snr_a; }
    if (sec == 4) printf(" |");
    if (c) printf(" %d/%d/%d(%d)", ra / c, rb / c, sn / c, c);
    else printf(" -");
  }
  printf("\n");
  return 0;
}
