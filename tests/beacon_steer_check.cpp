// beacon_steer_check.cpp — beacon-TBTT steer SURVIVAL + accuracy master (issue
// #241: the Jaguar2 steer-then-re-download actuator). Brings up TX, starts the
// hardware beacon, then applies N repeated AdjustBeaconTimingFine (or coarse
// AdjustBeaconTiming) steers at a fixed cadence. The beacon must keep airing
// across every steer — watch with an observer (tests/beacon_steer_survival.sh
// pairs this with a phase-logging RX): a frozen 802.11 seq / vanished beacon =
// the J2 post-re-latch drop; a phase step per steer = the actuator works.
//
// Opens USB (DEVOURER_VID/DEVOURER_PID) or PCIe (DEVOURER_PCIE_BDF, needs a
// DEVOURER_PCIE=ON build + -DDEVOURER_HAVE_PCIE on this TU).
//
// Build (USB): g++ -std=c++20 -O2 -Isrc -Iexamples/common \
//   tests/beacon_steer_check.cpp examples/common/env_config.cpp \
//   build/libdevourer.a $(pkg-config --cflags --libs libusb-1.0) -lpthread \
//   -o build/beacon_steer_check
// Run: sudo DEVOURER_PID=0x012d DEVOURER_VID=0x2357 DEVOURER_CHANNEL=36 \
//   build/beacon_steer_check [n_steers] [steer_us] [period_s]
//   STEER_MODE=coarse selects AdjustBeaconTiming (TU-quantized) instead;
//   STEER_MODE=pin selects PinBeaconTbtt (TSF-preserving absolute pin —
//   steer_us is then the absolute TBTT offset vs the TSF grid; successive
//   steers alternate offset/offset+|steer_us| so the observer sees a step).
//   TSF_WATCH=1: sample ReadTsf every ~20 ms around each steer and report the
//   worst TSF-timeline discontinuity (the fit-corruption a controller sees).
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

#include <libusb.h>

#include "SelectedChannel.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"
#if defined(DEVOURER_HAVE_PCIE)
#include "PcieTransport.h"
#endif

static long now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

int main(int argc, char **argv) {
  int n_steers = argc > 1 ? atoi(argv[1]) : 5;
  int steer_us = argc > 2 ? atoi(argv[2]) : -5000;
  int period_s = argc > 3 ? atoi(argv[3]) : 5;
  const char *steer_mode = std::getenv("STEER_MODE");
  const bool coarse = steer_mode && std::strcmp(steer_mode, "coarse") == 0;
  const bool pin = steer_mode && std::strcmp(steer_mode, "pin") == 0;
  const bool tsf_watch = [] {
    const char *w = std::getenv("TSF_WATCH");
    return w && *w && std::strcmp(w, "0") != 0;
  }();
  uint8_t ch = 36;
  if (const char *c = std::getenv("DEVOURER_CHANNEL")) ch = (uint8_t)atoi(c);
  int interval_tu = 100;

  auto logger = std::make_shared<Logger>();
  apply_logging_env(*logger);
  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) < 0) return 1;
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);

  WiFiDriver wifi(logger);
  std::unique_ptr<IRtlDevice> dev;
  const char *bdf = std::getenv("DEVOURER_PCIE_BDF");
#if defined(DEVOURER_HAVE_PCIE)
  if (bdf) {
    auto transport = devourer::PcieTransport::Open(bdf, logger);
    if (!transport) { fprintf(stderr, "PCIe open %s failed\n", bdf); return 1; }
    dev = wifi.CreateRtlDevicePcie(std::move(transport),
                                   devourer_config_from_env());
  } else
#endif
  {
    if (bdf) { fprintf(stderr, "DEVOURER_PCIE_BDF set but built without DEVOURER_HAVE_PCIE\n"); return 1; }
    uint16_t vid = 0x0bda, pid = 0;
    if (const char *v = std::getenv("DEVOURER_VID")) vid = (uint16_t)strtoul(v, 0, 0);
    if (const char *p = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(p, 0, 0);
    auto *h = libusb_open_device_with_vid_pid(ctx, vid, pid);
    if (!h) { fprintf(stderr, "open %04x:%04x failed\n", vid, pid); return 1; }
    std::shared_ptr<devourer::UsbDeviceLock> lock;
    if (devourer::claim_interface_then_reset(h, 0, logger, true, lock) != 0)
      return 1;
    dev = wifi.CreateRtlDevice(h, ctx, lock, devourer_config_from_env());
  }
  if (!dev) { fprintf(stderr, "no driver\n"); return 1; }

  dev->InitWrite(SelectedChannel{ch, 0, CHANNEL_WIDTH_20});
  std::this_thread::sleep_for(std::chrono::seconds(2));
  /* KEEP_CSMA=1: leave carrier-sense deferral on (A/B lever for measuring
   * SetCcaMode's effect on TBTT punctuality). Default: EDCCA off — the beacon
   * airs exactly at TBTT, the master owns the channel. */
  if (const char *k = std::getenv("KEEP_CSMA"); !(k && *k && std::strcmp(k, "0") != 0))
    dev->SetCcaMode(true);

  // Minimal beacon, canonical SA 57:42:75:05:d6:00 (rx.txhit matcher).
  std::vector<uint8_t> f = {
      0x00, 0x00, 0x0a, 0x00, 0x00, 0x80, 0x00, 0x00, 0x08, 0x00,  // radiotap
      0x80, 0x00, 0x00, 0x00,                                      // FC + dur
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff,                          // addr1
      0x57, 0x42, 0x75, 0x05, 0xd6, 0x00,                          // addr2 = SA
      0x57, 0x42, 0x75, 0x05, 0xd6, 0x00,                          // addr3
      0x00, 0x00,                                                  // seq
      0, 0, 0, 0, 0, 0, 0, 0,                                      // timestamp
      static_cast<uint8_t>(interval_tu & 0xff),
      static_cast<uint8_t>((interval_tu >> 8) & 0xff),
      0x00, 0x00,                                                  // capability
      0x00, 0x03, 'T', 'B', 'T',                                   // SSID
      0x01, 0x01, 0x82};                                           // rates
  if (!dev->StartBeacon(f.data(), f.size(), interval_tu)) {
    fprintf(stderr, "StartBeacon FAILED\n");
    return 1;
  }
  printf("BEACON_MS %ld ch=%d interval_tu=%d mode=%s n=%d steer_us=%d period_s=%d\n",
         now_ms(), ch, interval_tu,
         coarse ? "coarse" : (pin ? "pin" : "fine"), n_steers, steer_us,
         period_s);
  fflush(stdout);
  std::this_thread::sleep_for(std::chrono::seconds(period_s));

  for (int i = 0; i < n_steers; ++i) {
    /* TSF_WATCH: bracket the steer with a TSF sample train; a controller's
     * ref = a·tsf + b fit sees exactly these discontinuities. */
    uint64_t pre_tsf = 0;
    long long pre_us = 0;
    if (tsf_watch) {
      pre_tsf = dev->ReadTsf();
      pre_us = std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
                   .count();
    }
    long t0 = now_ms();
    int req = steer_us;
    if (pin && (i & 1))
      req = steer_us + std::abs(steer_us); /* alternate so the phase steps */
    int applied = coarse ? dev->AdjustBeaconTiming(req)
                 : pin   ? dev->PinBeaconTbtt(req)
                         : dev->AdjustBeaconTimingFine(req);
    printf("STEER_MS %ld n=%d req_us=%d applied_us=%d dur_ms=%ld\n", t0, i + 1,
           req, applied, now_ms() - t0);
    if (tsf_watch) {
      uint64_t post_tsf = dev->ReadTsf();
      long long post_us = std::chrono::duration_cast<std::chrono::microseconds>(
                              std::chrono::steady_clock::now().time_since_epoch())
                              .count();
      /* Discontinuity = TSF advance minus wall-clock advance across the
       * steer (host jitter ~tens of µs; the fine steer's jump is the steer
       * magnitude, the pin's is ~register-write latency). */
      long long disc = (long long)(post_tsf - pre_tsf) - (post_us - pre_us);
      printf("TSFJUMP_MS %ld n=%d disc_us=%lld\n", now_ms(), i + 1, disc);
    }
    fflush(stdout);
    if (applied == 0 && !(pin && req % (interval_tu * 1024) == 0)) {
      fprintf(stderr, "steer %d refused/failed (applied=0)\n", i + 1);
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::seconds(period_s));
  }
  printf("DONE_MS %ld\n", now_ms());
  fflush(stdout);
  return 0;
}
