/*
 * Does StopBeacon actually silence the MAC?
 *
 * This exists because the obvious way to check it does not check it. Both AP
 * harnesses (tests/ap_responder.cpp, tests/ap_wpa2.cpp) end in `_exit(0)`,
 * which bypasses every destructor - so the radio's Stop(), and with it
 * StopBeacon(), never run, and "the beacon was gone after the process exited"
 * measures nothing. On this part that is not a cosmetic difference: the MAC
 * beacons AUTONOMOUSLY from the reserved page once armed, so a session that
 * skips the teardown leaves it airing until the adapter is power-cycled.
 *
 * Three phases, each long enough for a station to complete a scan:
 *   1. armed      - a scan MUST see the SSID
 *   2. stopped    - StopBeacon(), then a scan MUST NOT
 *   3. re-armed   - StartBeacon() again, to prove the stop left the engine
 *                   usable rather than wedged
 *
 * The witness is external (`iw scan` from a station); this program only drives
 * the transitions and prints when each phase begins, so the operator or a
 * script can scan in the right window. It is not a ctest cell - it needs an
 * adapter and a second radio to look.
 *
 *   sudo DEVOURER_VID=0x0e8d DEVOURER_PID=0x7612 DEVOURER_CHANNEL=36 \
 *        DEVOURER_MT7612U_FW_DIR=<dir> ./beacon_stop_check [phase_secs]
 */
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

#include <libusb.h>

#include "DeviceConfig.h"
#include "UsbDeviceLock.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"

namespace {

/* BSSID and SSID of our own, so this cannot be confused with a neighbour or
 * with ap_responder's devourerAP. Locally administered on purpose: that is the
 * case that lands in APC slot 1 rather than 0, and the case the first draft of
 * mt7612u_beacon_start() refused outright. */
const uint8_t kBssid[6] = { 0x02, 0x4d, 0x54, 0x53, 0x54, 0x50 };
const char kSsid[] = "mtStopCheck";

std::vector<uint8_t> build_beacon(uint8_t chan) {
  std::vector<uint8_t> f = {
      /* radiotap: 8-byte header + TX flags, the shape ap_responder uses */
      0x00, 0x00, 0x0a, 0x00, 0x00, 0x80, 0x00, 0x00, 0x08, 0x00,
      0x80, 0x00, 0x00, 0x00,                          /* FC + duration */
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff,              /* addr1 broadcast */
  };
  f.insert(f.end(), kBssid, kBssid + 6);               /* addr2 */
  f.insert(f.end(), kBssid, kBssid + 6);               /* addr3 */
  const uint8_t tail[] = {
      0x00, 0x00,                                      /* seq (HW assigns) */
      0, 0, 0, 0, 0, 0, 0, 0,                          /* timestamp (HW) */
      0x64, 0x00,                                      /* 100 TU */
      0x01, 0x00,                                      /* ESS */
      0x00, (uint8_t)(sizeof kSsid - 1),
  };
  f.insert(f.end(), tail, tail + sizeof tail);
  f.insert(f.end(), kSsid, kSsid + sizeof kSsid - 1);
  const uint8_t ies[] = {
      0x01, 0x08, 0x8c, 0x12, 0x98, 0x24, 0xb0, 0x48, 0x60, 0x6c,
      0x03, 0x01, chan,
  };
  f.insert(f.end(), ies, ies + sizeof ies);
  return f;
}

void banner(const char *phase) {
  std::printf("\n=== %s === (scan now)\n", phase);
  std::fflush(stdout);
}

} // namespace

int main(int argc, char **argv) {
  const int secs = argc > 1 ? atoi(argv[1]) : 20;
  uint8_t chan = 36;
  if (const char *c = std::getenv("DEVOURER_CHANNEL")) chan = (uint8_t)atoi(c);

  auto logger = std::make_shared<Logger>();
  libusb_context *ctx = nullptr;
  libusb_init(&ctx);
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);

  uint16_t vid = 0x0e8d, pid = 0x7612;
  if (const char *v = std::getenv("DEVOURER_VID")) vid = (uint16_t)strtoul(v, 0, 0);
  if (const char *p = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(p, 0, 0);
  auto *h = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!h) { std::fprintf(stderr, "open %04x:%04x fail\n", vid, pid); return 1; }

  std::shared_ptr<devourer::UsbDeviceLock> lk;
  if (devourer::claim_interface_then_reset(
          h, devourer::find_wifi_interface(h), logger, true, lk) != 0)
    return 1;

  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRadio(h, ctx, lk, devourer_config_from_env());
  if (!dev) return 1;
  dev->InitWrite(SelectedChannel{chan, 0, CHANNEL_WIDTH_20});

  const auto bcn = build_beacon(chan);
  int fails = 0;

  banner("PHASE 1: armed - the SSID mtStopCheck MUST appear");
  if (!dev->StartBeacon(bcn.data(), bcn.size(), 100)) {
    std::fprintf(stderr, "FAIL: StartBeacon returned false\n");
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::seconds(secs));

  banner("PHASE 2: stopped - the SSID MUST be gone");
  if (!dev->StopBeacon()) {
    std::fprintf(stderr, "FAIL: StopBeacon returned false\n");
    fails++;
  }
  /* Documented to return false the second time: no beacon is active. */
  if (dev->StopBeacon()) {
    std::fprintf(stderr, "FAIL: a second StopBeacon returned true\n");
    fails++;
  }
  /* And an update with nothing armed must refuse rather than report success
   * for a write into a disarmed engine. */
  if (dev->UpdateBeaconPayload(bcn.data(), bcn.size())) {
    std::fprintf(stderr, "FAIL: UpdateBeaconPayload succeeded with no beacon\n");
    fails++;
  }
  std::this_thread::sleep_for(std::chrono::seconds(secs));

  banner("PHASE 3: re-armed - the SSID MUST come back");
  if (!dev->StartBeacon(bcn.data(), bcn.size(), 100)) {
    std::fprintf(stderr, "FAIL: StartBeacon after a stop returned false\n");
    fails++;
  }
  std::this_thread::sleep_for(std::chrono::seconds(secs));

  /* Explicit, not left to the destructor - the point of this program is that
   * the teardown path is the thing under test. */
  dev->StopBeacon();
  std::printf("\nlocal checks: %d failure(s). The SSID phases are the "
              "witness's call.\n", fails);
  return fails ? 1 : 0;
}
