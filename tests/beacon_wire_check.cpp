// beacon_wire_check.cpp — verify devourer's hardware beacon looks like a real
// kernel-AP beacon ON THE WIRE. For the canonical-SA beacon it prints, per
// frame: the frame control (FC, must be 0x0080 = beacon), the 802.11
// sequence-control field (MPDU bytes 22-23: seq[15:4] must INCREMENT by 1 per
// beacon — the hardware sequence numbering a kernel AP does via EN_HWSEQ), and
// the beacon-body timestamp (bytes 24-31, the live hardware TSF, steps ~102400 µs
// at 100 TU). Bench-confirmed kernel-equivalent on all three generations:
//   J3 8822C : seq 16,17,18,19,20,21...   ts steps ~102400 us
//   J2 8812BU: seq 730,731,732,733...     ts steps ~102400 us
//   J1 8821AU: seq increments, ts steps; the 8814A pins seq at 0 (kernel rtw88
//   parity on that chip — judge it by presence/cadence/ts, not seq)
// (Elsewhere a beacon whose seq is FROZEN is a degraded engine — e.g. a beacon
// left in the post-re-latch drop state; a healthy fresh beacon increments.)
//
// Build: g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/beacon_wire_check.cpp \
//   examples/common/env_config.cpp build/libdevourer.a \
//   $(pkg-config --cflags --libs libusb-1.0) -lpthread -o build/beacon_wire_check
// Run against any devourer HW beacon airing on the channel:
//   sudo DEVOURER_PID=<observer> DEVOURER_CHANNEL=36 build/beacon_wire_check
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

int main() {
  auto logger = std::make_shared<Logger>();
  libusb_context* ctx = nullptr; libusb_init(&ctx);
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
  uint16_t vid = 0x0bda, pid = 0xa81a;
  if (const char* p = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(p, 0, 0);
  uint8_t ch = 36; if (const char* c = std::getenv("DEVOURER_CHANNEL")) ch = atoi(c);
  auto* h = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!h) { fprintf(stderr, "open fail\n"); return 1; }
  std::shared_ptr<devourer::UsbDeviceLock> lk;
  if (devourer::claim_interface_then_reset(h, devourer::find_wifi_interface(h), logger, true, lk) != 0) return 1;
  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRtlDevice(h, ctx, lk, devourer_config_from_env());
  if (!dev) return 1;
  auto cb = [](const Packet& p) {
    if (p.Data.size() < 32 || p.RxAtrib.crc_err) return;
    if (std::memcmp(p.Data.data() + 10, kSa, 6) != 0) return;   // canonical SA (addr2)
    uint16_t fc = p.Data[0] | (p.Data[1] << 8);
    uint16_t seqctl = p.Data[22] | (p.Data[23] << 8);
    uint16_t seq = seqctl >> 4, frag = seqctl & 0xf;
    uint64_t ts = 0; for (int i = 0; i < 8; ++i) ts |= (uint64_t)p.Data[24 + i] << (8 * i);
    int n = ++g_n;
    if (n <= 16) {
      printf("beacon %2d: FC=0x%04x seq=%u frag=%u  ts=%llu us  (len=%zu)\n",
             n, fc, seq, frag, (unsigned long long)ts, p.Data.size());
      fflush(stdout);
    }
  };
  std::thread rx([&] { dev->Init(cb, SelectedChannel{ch, 0, CHANNEL_WIDTH_20}); });
  std::this_thread::sleep_for(std::chrono::seconds(6));
  printf("== canonical-SA beacons seen: %d (seq should step +1, ts ~102400 us) ==\n",
         g_n.load());
  _exit(0);
}
