// fast_bw_after_hop_rxcheck.cpp — does a FastSetBandwidth toggle after a
// FastRetune hop leave the receiver on the hop's channel? The fast bandwidth
// path replays a cached RF18; if a hop does not move that cache, the toggle
// writes the previous channel's RF18 back and the synth returns there while
// the driver still believes it is on the new channel.
//
// Brings RX up at 20 MHz on <from>, hops to <to> with FastRetune, toggles
// 20 -> 10 -> 20 MHz with FastSetBandwidth, and for each phase prints how many
// beacons were decoded per advertised primary channel (HT Operation element,
// DS Parameter Set as fallback). Needs only ambient APs on both channels, in
// the same band. PASS = the last phase hears <to>'s beacons and none of
// <from>'s. Exit status: 0 PASS, 1 FAIL, 2 INCONCLUSIVE.
//
// Build (one line):
//   g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/fast_bw_after_hop_rxcheck.cpp examples/common/env_config.cpp build/libdevourer.a $(pkg-config --cflags --libs libusb-1.0) -lpthread -o build/fast_bw_after_hop_rxcheck
// Run (default 36 44):
//   sudo DEVOURER_VID=0x0bda DEVOURER_PID=0xb812 build/fast_bw_after_hop_rxcheck <from> <to>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <mutex>
#include <thread>

#include <libusb.h>

#include "RxPacket.h"
#include "SelectedChannel.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"

static std::mutex g_mu;
static std::map<int, int> g_by_ch; // advertised primary channel -> beacons
static std::atomic<bool> g_rx_live{false}; // a frame arrived: bring-up is done

/* Primary channel a beacon advertises, 0 when it carries neither element. */
static int beacon_channel(const uint8_t *d, size_t n) {
  int ds = 0;
  for (size_t i = 24 + 12; i + 2 <= n;) {
    const uint8_t id = d[i], len = d[i + 1];
    if (i + 2 + len > n)
      break;
    if (id == 61 && len >= 1) // HT Operation: primary channel
      return d[i + 2];
    if (id == 3 && len == 1) // DS Parameter Set
      ds = d[i + 2];
    i += 2 + len;
  }
  return ds;
}

int main(int argc, char **argv) {
  const uint8_t from = argc > 1 ? (uint8_t)atoi(argv[1]) : 36;
  const uint8_t to = argc > 2 ? (uint8_t)atoi(argv[2]) : 44;

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
    g_rx_live.store(true, std::memory_order_relaxed);
    if (p.Data.size() < 38 || p.Data[0] != 0x80)
      return;
    const int ch = beacon_channel(p.Data.data(), p.Data.size());
    std::lock_guard<std::mutex> lk(g_mu);
    g_by_ch[ch]++;
  };
  std::atomic<bool> rx_done{false};
  std::thread rx([&] {
    dev->Init(cb, SelectedChannel{from, 0, CHANNEL_WIDTH_20});
    rx_done = true;
  });
  /* Stop the RX loop and tear down in order. StartRxLoop clears the stop
   * flag on entry, so repeat the stop until the thread has really ended. */
  auto shutdown = [&] {
    while (!rx_done) {
      dev->StopRxLoop();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    rx.join();
    dev->Stop();
  };
  /* Init runs bring-up before its RX loop, and the hop and toggle below must
   * not overlap it. The first decoded frame is the proof it has finished. */
  for (int i = 0; i < 150 && !g_rx_live && !rx_done; i++)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  if (!g_rx_live) {
    printf("verdict: INCONCLUSIVE — no frame decoded on ch%d within 15 s of "
           "bring-up\n", from);
    shutdown();
    return 2;
  }

  /* Beacons per advertised channel over `ms`; returns {on `want`, on `other`}. */
  auto window = [&](const char *label, int ms, int want, int other) {
    { std::lock_guard<std::mutex> lk(g_mu); g_by_ch.clear(); }
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    std::lock_guard<std::mutex> lk(g_mu);
    printf("  %-28s", label);
    if (g_by_ch.empty()) printf(" no beacons");
    for (const auto &[ch, n] : g_by_ch) printf(" ch%d:%d", ch, n);
    printf("\n");
    return std::pair<int, int>{g_by_ch[want], g_by_ch[other]};
  };

  printf("== fast_bw_after_hop_rxcheck: %d -> %d ==\n", from, to);
  /* from == to skips the hop and checks the bare toggle. */
  const int other = from == to ? -1 : from;
  const auto a = window("20 MHz on <from>", 3000, from, -1);
  dev->FastRetune(to);
  const auto b = window("FastRetune -> <to>", 3000, to, other);
  dev->FastSetBandwidth(CHANNEL_WIDTH_10);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  dev->FastSetBandwidth(CHANNEL_WIDTH_20);
  const auto c = window("after 20 -> 10 -> 20 toggle", 3000, to, other);

  shutdown();

  if (a.first == 0 || b.first == 0) {
    printf("verdict: INCONCLUSIVE — need beacons on both channels "
           "(<from>:%d, <to> after the hop:%d)\n", a.first, b.first);
    return 2;
  }
  if (c.first > 0 && c.second == 0) {
    printf("verdict: PASS — still on <to> after the toggle (%d beacons)\n",
           c.first);
    return 0;
  }
  printf("verdict: FAIL — after the toggle <to>:%d, <from>:%d\n", c.first,
         c.second);
  return 1;
}
