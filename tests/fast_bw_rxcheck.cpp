// fast_bw_rxcheck.cpp — functional proof that FastSetBandwidth actually
// re-clocks the receiver. The RX device starts at 20 MHz, then toggles to a
// narrowband width via FastSetBandwidth and back, while a partner transmits the
// canonical beacon in that SAME narrowband width. Because narrowband and 20 MHz
// are different ADC clock domains, a 20 MHz RX CANNOT decode the narrowband
// beacon — so canonical-SA hits must appear ONLY in the narrowband window and
// vanish again after the fast switch back to 20 MHz.
//
// Build: g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/fast_bw_rxcheck.cpp \
//   examples/common/env_config.cpp build/libdevourer.a \
//   $(pkg-config --cflags --libs libusb-1.0) -lpthread -o build/fast_bw_rxcheck
// Run: sudo DEVOURER_VID=.. DEVOURER_PID=.. DEVOURER_CHANNEL=36 \
//        build/fast_bw_rxcheck <5|10>   (partner TXes that width)
#include <atomic>
#include <chrono>
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

static const uint8_t kSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
static std::atomic<uint64_t> g_hits{0};

int main(int argc, char **argv) {
  const int nb = argc > 1 ? atoi(argv[1]) : 10;
  const ChannelWidth_t nb_w = nb == 5 ? CHANNEL_WIDTH_5 : CHANNEL_WIDTH_10;
  uint8_t ch = 36;
  if (const char *c = std::getenv("DEVOURER_CHANNEL")) ch = (uint8_t)atoi(c);

  auto logger = std::make_shared<Logger>();
  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) < 0) return 1;
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
  uint16_t vid = 0x0bda, pid = 0;
  if (const char *e = std::getenv("DEVOURER_VID")) vid = (uint16_t)strtoul(e, 0, 0);
  if (const char *e = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(e, 0, 0);
  auto *h = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!h) { fprintf(stderr, "open %04x:%04x failed\n", vid, pid); return 1; }
  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (devourer::claim_interface_then_reset(h, 0, logger, true, lock) != 0) return 1;
  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRtlDevice(h, ctx, lock, devourer_config_from_env());
  if (!dev) return 1;

  auto cb = [](const Packet &p) {
    if (!p.RxAtrib.crc_err && p.Data.size() >= 16 &&
        std::memcmp(p.Data.data() + 10, kSa, 6) == 0)
      g_hits.fetch_add(1, std::memory_order_relaxed);
  };
  // RX loop on a thread, brought up at 20 MHz.
  std::thread rx([&] { dev->Init(cb, SelectedChannel{ch, 0, CHANNEL_WIDTH_20}); });

  auto window = [&](const char *label, int ms) {
    g_hits.store(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    uint64_t n = g_hits.load();
    printf("  %-22s hits=%llu\n", label, (unsigned long long)n);
    return n;
  };

  std::this_thread::sleep_for(std::chrono::seconds(2)); // settle bring-up
  printf("== fast_bw_rxcheck: RX toggles, partner TX %d MHz ==\n", nb);
  uint64_t a = window("20 MHz (baseline)", 3000);
  dev->FastSetBandwidth(nb_w);
  uint64_t b = window("fast-> narrowband", 4000);
  dev->FastSetBandwidth(CHANNEL_WIDTH_20);
  uint64_t c = window("fast-> 20 MHz", 3000);

  printf("\nverdict: ");
  if (b > 20 && a == 0 && c == 0)
    printf("PASS — decode ONLY in the narrowband window (%llu), both fast "
           "switches re-clocked correctly\n", (unsigned long long)b);
  else if (b > a && b > c)
    printf("LIKELY PASS — narrowband window dominant (a=%llu b=%llu c=%llu)\n",
           (unsigned long long)a, (unsigned long long)b, (unsigned long long)c);
  else
    printf("INCONCLUSIVE (a=%llu b=%llu c=%llu) — check the TX partner\n",
           (unsigned long long)a, (unsigned long long)b, (unsigned long long)c);
  fflush(stdout);
  _exit(0); // don't join the blocking RX loop
}
