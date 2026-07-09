// beacon_fullbody.cpp — is the "minimal beacon body" constraint real?
//
// docs/time-distribution.md claims an extended beacon body breaks the hardware
// TSF insertion (the MAC writes a per-beacon counter instead of the live TSF).
// But kernel APs beacon with FULL bodies (SSID + rates + DS + TIM + HT/VHT) and
// the HW inserts the TSF fine — so the limit is suspect. This brings up a beacon
// with a realistic, WELL-FORMED full body (canonical SA so the observer matches)
// and idles; run tests/beacon_ts_check against it (DEVOURER_PID of the observer)
// and watch the timestamp field (MPDU bytes 24..31): if it steps ~102400 µs per
// beacon, the HW TSF insertion works with a full body and the constraint is a
// misdiagnosis (a malformed test frame, not a hardware limit).
//
// Build: g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/beacon_fullbody.cpp \
//   examples/common/env_config.cpp build/libdevourer.a \
//   $(pkg-config --cflags --libs libusb-1.0) -lpthread -o build/beacon_fullbody
// Run: sudo DEVOURER_PID=0xc812 DEVOURER_CHANNEL=36 build/beacon_fullbody [sec]
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

// A full, well-formed 802.11 beacon MPDU (behind a rate-less radiotap). Same
// canonical SA/BSSID the rx.txhit / beacon_ts_check matchers use, but with a
// realistic body: 10-char SSID + 8 supported rates + DS param + TIM + ERP.
static std::vector<uint8_t> full_beacon(int interval_tu, uint8_t chan) {
  std::vector<uint8_t> f = {
      0x00, 0x00, 0x0a, 0x00, 0x00, 0x80, 0x00, 0x00, 0x08, 0x00,  // radiotap
      0x80, 0x00, 0x00, 0x00,                                      // FC beacon + dur
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff,                          // addr1 broadcast
      0x57, 0x42, 0x75, 0x05, 0xd6, 0x00,                          // addr2 = SA
      0x57, 0x42, 0x75, 0x05, 0xd6, 0x00,                          // addr3 = BSSID
      0x00, 0x00,                                                  // seq (HW fills)
      0, 0, 0, 0, 0, 0, 0, 0,                                      // timestamp (HW fills)
      static_cast<uint8_t>(interval_tu & 0xff),
      static_cast<uint8_t>((interval_tu >> 8) & 0xff),             // beacon interval
      0x01, 0x00,                                                  // capability (ESS)
      // SSID IE — 10 chars (vs the 3-char minimal)
      0x00, 0x0a, 'd', 'e', 'v', 'o', 'u', 'r', 'e', 'r', 'A', 'P',
      // Supported rates IE — 8 rates (vs the 1 minimal)
      0x01, 0x08, 0x82, 0x84, 0x8b, 0x96, 0x24, 0x30, 0x48, 0x6c,
      // DS Parameter Set — current channel
      0x03, 0x01, chan,
      // TIM IE — DTIM count 0, period 1, bitmap control 0, partial bitmap 0
      0x05, 0x04, 0x00, 0x01, 0x00, 0x00,
      // ERP Information
      0x2a, 0x01, 0x00};
  return f;
}

int main(int argc, char** argv) {
  int sec = argc > 1 ? atoi(argv[1]) : 20;
  int interval_tu = 100;
  if (const char* iv = std::getenv("DEVOURER_BCN_TU")) interval_tu = atoi(iv);
  uint8_t ch = 36;
  if (const char* c = std::getenv("DEVOURER_CHANNEL")) ch = (uint8_t)atoi(c);

  auto logger = std::make_shared<Logger>();
  apply_logging_env(*logger);
  libusb_context* ctx = nullptr;
  if (libusb_init(&ctx) < 0) return 1;
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
  uint16_t vid = 0x0bda, pid = 0xc812;
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
  auto f = full_beacon(interval_tu, ch);
  bool ok = dev->StartBeacon(f.data(), f.size(), interval_tu);
  fprintf(stderr, "FULL-BODY beacon (%zu-byte MPDU: SSID 'devourerAP' + 8 rates + "
                  "DS + TIM + ERP) StartBeacon -> %s. Idling %d s. Observe the "
                  "timestamp field with beacon_ts_check.\n",
          f.size() - 10, ok ? "OK" : "FAILED", sec);
  if (!ok) return 1;
  std::this_thread::sleep_for(std::chrono::seconds(sec));
  return 0;
}
