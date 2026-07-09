// beacon_tbtt.cpp — experiment (idea 6): can the MAC transmit a beacon at each
// TBTT (hardware-timed off the TSF) in devourer's monitor/injection mode? Brings
// up TX on a Jaguar1 adapter, loads a beacon into the beacon queue + enables the
// beacon function (IRtlDevice::StartBeacon), then IDLES — no send loop. If
// the beacon function works, the chip transmits the beacon on its own at the
// interval. Observe with a second adapter running rxdemo (count rx.txhit of the
// canonical SA; ~1 per 102.4 ms at 100 TU => hardware-timed TX confirmed).
//
// Build: g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/beacon_tbtt.cpp \
//   examples/common/env_config.cpp build/libdevourer.a \
//   $(pkg-config --cflags --libs libusb-1.0) -lpthread -o build/beacon_tbtt
// Run: sudo DEVOURER_PID=0x8812 DEVOURER_CHANNEL=36 build/beacon_tbtt [interval_tu] [sec]
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <thread>
#include <vector>

#include <libusb.h>

#include "SelectedChannel.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"

int main(int argc, char** argv) {
  int interval_tu = argc > 1 ? atoi(argv[1]) : 100;  // 100 TU ≈ 102.4 ms
  int sec = argc > 2 ? atoi(argv[2]) : 20;
  uint8_t ch = 36;
  if (const char* c = std::getenv("DEVOURER_CHANNEL")) ch = (uint8_t)atoi(c);

  auto logger = std::make_shared<Logger>();
  apply_logging_env(*logger);
  libusb_context* ctx = nullptr;
  if (libusb_init(&ctx) < 0) return 1;
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
  uint16_t vid = 0x0bda, pid = 0;
  if (const char* v = std::getenv("DEVOURER_VID")) vid = (uint16_t)strtoul(v, 0, 0);
  if (const char* p = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(p, 0, 0);
  auto* h = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!h) { fprintf(stderr, "open %04x:%04x failed\n", vid, pid); return 1; }
  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (devourer::claim_interface_then_reset(h, 0, logger, true, lock) != 0) return 1;
  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRtlDevice(h, ctx, lock, devourer_config_from_env());
  if (!dev) { fprintf(stderr, "no driver\n"); return 1; }

  dev->InitWrite(SelectedChannel{ch, 0, CHANNEL_WIDTH_20});
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Minimal 802.11 beacon behind a rate-less radiotap (SA = the canonical
  // 57:42:75:05:d6:00 so the RX's rx.txhit matcher recognises it).
  std::vector<uint8_t> f = {
      0x00, 0x00, 0x0a, 0x00, 0x00, 0x80, 0x00, 0x00, 0x08, 0x00,  // radiotap
      0x80, 0x00, 0x00, 0x00,                                      // FC beacon + dur
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff,                          // addr1 broadcast
      0x57, 0x42, 0x75, 0x05, 0xd6, 0x00,                          // addr2 = SA
      0x57, 0x42, 0x75, 0x05, 0xd6, 0x00,                          // addr3 = BSSID
      0x00, 0x00,                                                  // seq
      0, 0, 0, 0, 0, 0, 0, 0,                                      // timestamp (HW fills)
      static_cast<uint8_t>(interval_tu & 0xff),
      static_cast<uint8_t>((interval_tu >> 8) & 0xff),             // beacon interval
      0x00, 0x00,                                                  // capability
      0x00, 0x03, 'T', 'B', 'T',                                   // SSID IE "TBT"
      0x01, 0x01, 0x82};                                          // supported rates (1M)

  bool ok = dev->StartBeacon(f.data(), f.size(), interval_tu);
  fprintf(stderr, "StartBeacon -> %s. Idling %d s (chip should self-TX at "
                  "TBTT every %d TU ≈ %.1f ms). Watch a 2nd RX's rx.txhit.\n",
          ok ? "OK" : "FAILED/unsupported", sec, interval_tu, interval_tu * 1.024);
  if (!ok) return 1;

  // Idle — NO send loop. Any airing frames are the hardware beacon engine.
  std::this_thread::sleep_for(std::chrono::seconds(sec));
  return 0;
}
