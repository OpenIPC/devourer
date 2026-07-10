// Capture the canonical-SA beacon and print its 802.11 timestamp field (MPDU
// bytes 24..31, the beacon body's first 8 bytes = the TSF the AP inserts) across
// beacons. If the hardware inserts a live TSF, consecutive timestamps step by
// ~102400 us (100 TU) — proving a real time-distributing beacon.
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <memory>
#include <thread>
#include <libusb.h>
#include "RxPacket.h"
#include "SelectedChannel.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"

static const uint8_t kSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
static std::atomic<int> g_n{0};
static uint64_t g_prev = 0;

int main() {
  auto logger = std::make_shared<Logger>();
  libusb_context* ctx = nullptr; libusb_init(&ctx);
  uint16_t vid = 0x0bda, pid = 0xa81a;
  if (const char* p = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(p, 0, 0);
  uint8_t ch = 6; if (const char* c = std::getenv("DEVOURER_CHANNEL")) ch = atoi(c);
  auto* h = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!h) { fprintf(stderr, "open fail\n"); return 1; }
  std::shared_ptr<devourer::UsbDeviceLock> lk;
  if (devourer::claim_interface_then_reset(h, 0, logger, true, lk) != 0) return 1;
  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRtlDevice(h, ctx, lk, devourer_config_from_env());
  if (!dev) return 1;
  auto cb = [](const Packet& p) {
    if (p.Data.size() < 32 || p.RxAtrib.crc_err) return;
    if (std::memcmp(p.Data.data() + 10, kSa, 6) != 0) return;   // canonical SA (addr2)
    uint64_t ts = 0; for (int i = 0; i < 8; ++i) ts |= (uint64_t)p.Data[24 + i] << (8 * i);
    int n = ++g_n;
    if (n <= 12) {
      long d = g_prev ? (long)(ts - g_prev) : 0;
      printf("beacon %2d: timestamp=%llu us   step=%ld us   (arrival tsfl=%u)\n",
             n, (unsigned long long)ts, d, p.RxAtrib.tsfl);
      fflush(stdout);
    }
    g_prev = ts;
  };
  std::thread rx([&] { dev->Init(cb, SelectedChannel{ch, 0, CHANNEL_WIDTH_20}); });
  std::this_thread::sleep_for(std::chrono::seconds(8));
  dev->StopRxLoop(); rx.join();
  printf("== total canonical-SA beacons: %d ==\n", g_n.load());
  _exit(0);
}
