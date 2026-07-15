// beacon_update_probe.cpp — M0 contract 2 probe: dynamic beacon-content
// delivery via UpdateBeaconPayload (the DCI-style grant-map carrier).
//
// TX mode (default): starts a hardware beacon whose body carries a versioned
// vendor IE — 'G','M' magic, a monotonically-increasing u32 version, a 32-byte
// version-derived pattern, and a CRC16 over all of it (so a torn/partial swap
// is detectable from ONE frame). Every K beacon intervals it bumps the version
// and calls UpdateBeaconPayload, emitting {"ev":"bcn.update","ver":..,
// "host_ns":..,"ok":..} per call.
//
// Witness mode (MODE=rx): a monitor RX on another adapter records every beacon
// from the probe's SA as {"ev":"bcn.rx","seq":..,"tsfl":..,"ver":..,
// "crc_ok":..,"host_ns":..}. Both processes run on ONE host, so host_ns
// (CLOCK_MONOTONIC) is directly comparable and update→air latency falls out.
// tests/beacon_update_analyze.py turns the two streams into the skip / dup /
// late / stale / torn verdict; tests/beacon_update_check.sh orchestrates.
//
// Build: g++ -std=c++20 -O2 -Isrc -Iexamples/common \
//   tests/beacon_update_probe.cpp examples/common/env_config.cpp \
//   build/libdevourer.a $(pkg-config --cflags --libs libusb-1.0) -lpthread \
//   -o build/beacon_update_probe
// TX:      sudo DEVOURER_PID=0x012d DEVOURER_CHANNEL=36 build/beacon_update_probe [n_updates] [k_intervals]
// witness: sudo MODE=rx DEVOURER_PID=0x8812 DEVOURER_CHANNEL=36 build/beacon_update_probe [secs]
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <thread>
#include <unistd.h>
#include <vector>

#include <libusb.h>

#include "RxPacket.h"
#include "SelectedChannel.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"

static const uint8_t kSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
static constexpr uint8_t kOui[3] = {0x00, 0x57, 0x42};
static constexpr size_t kPatLen = 32;

static int64_t host_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// CRC-16/CCITT-FALSE — one frame proves the swap wasn't torn.
static uint16_t crc16(const uint8_t* d, size_t n) {
  uint16_t c = 0xffff;
  for (size_t i = 0; i < n; ++i) {
    c ^= (uint16_t)d[i] << 8;
    for (int b = 0; b < 8; ++b)
      c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
  }
  return c;
}

// [radiotap][beacon MPDU with the versioned grant-map vendor IE].
static std::vector<uint8_t> build_beacon(uint32_t ver, int interval_tu) {
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
      0x00, 0x03, 'G', 'N', 'T',                                   // SSID
      0x01, 0x01, 0x82};                                           // rates
  // Vendor IE: DD len | OUI | 'G' 'M' | ver u32 LE | pat[32] | crc16 LE.
  std::vector<uint8_t> body;
  body.push_back('G'); body.push_back('M');
  for (int i = 0; i < 4; ++i) body.push_back((uint8_t)(ver >> (8 * i)));
  for (size_t i = 0; i < kPatLen; ++i) body.push_back((uint8_t)(ver + i));
  uint16_t c = crc16(body.data(), body.size());
  body.push_back((uint8_t)(c & 0xff)); body.push_back((uint8_t)(c >> 8));
  f.push_back(0xdd);
  f.push_back((uint8_t)(3 + body.size()));
  f.insert(f.end(), kOui, kOui + 3);
  f.insert(f.end(), body.begin(), body.end());
  return f;
}

// Find our vendor IE in an RX beacon MPDU; return ver + crc verdict.
static bool parse_beacon(const uint8_t* mpdu, size_t len, uint32_t* ver,
                         bool* crc_ok) {
  if (len < 38 || mpdu[0] != 0x80 || std::memcmp(mpdu + 10, kSa, 6) != 0)
    return false;
  size_t off = 36;  // header(24) + timestamp(8) + interval(2) + cap(2)
  while (off + 2 <= len) {
    uint8_t tag = mpdu[off], tl = mpdu[off + 1];
    if (off + 2 + tl > len) break;
    const uint8_t* v = mpdu + off + 2;
    if (tag == 0xdd && tl >= 3 + 2 + 4 + 2 && std::memcmp(v, kOui, 3) == 0 &&
        v[3] == 'G' && v[4] == 'M') {
      const uint8_t* body = v + 3;
      size_t blen = (size_t)tl - 3;
      uint32_t x = 0;
      for (int i = 0; i < 4; ++i) x |= (uint32_t)body[2 + i] << (8 * i);
      uint16_t want = (uint16_t)(body[blen - 2] | (body[blen - 1] << 8));
      *ver = x;
      *crc_ok = crc16(body, blen - 2) == want;
      return true;
    }
    off += 2 + tl;
  }
  return false;
}

static std::atomic<uint64_t> g_rx{0};

int main(int argc, char** argv) {
  const char* mode = std::getenv("MODE");
  const bool rx_mode = mode && std::strcmp(mode, "rx") == 0;
  uint8_t ch = 36;
  if (const char* c = std::getenv("DEVOURER_CHANNEL")) ch = (uint8_t)atoi(c);
  const int interval_tu = 100;

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
  if (devourer::claim_interface_then_reset(h, devourer::find_wifi_interface(h), logger, true, lock) != 0)
    return 1;
  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRtlDevice(h, ctx, lock, devourer_config_from_env());
  if (!dev) { fprintf(stderr, "no driver\n"); return 1; }

  if (rx_mode) {
    int secs = argc > 1 ? atoi(argv[1]) : 60;
    auto cb = [](const Packet& p) {
      if (p.RxAtrib.crc_err) return;
      uint32_t ver = 0; bool crc_ok = false;
      if (!parse_beacon(p.Data.data(), p.Data.size(), &ver, &crc_ok)) return;
      uint16_t seq = (uint16_t)((p.Data[22] | (p.Data[23] << 8)) >> 4);
      ++g_rx;
      printf("{\"ev\":\"bcn.rx\",\"seq\":%u,\"tsfl\":%u,\"ver\":%u,"
             "\"crc_ok\":%s,\"host_ns\":%lld}\n",
             seq, p.RxAtrib.tsfl, ver, crc_ok ? "true" : "false",
             (long long)host_ns());
      fflush(stdout);
    };
    std::thread rx([&] { dev->Init(cb, SelectedChannel{ch, 0, CHANNEL_WIDTH_20}); });
    std::this_thread::sleep_for(std::chrono::seconds(secs));
    fprintf(stderr, "beacon_update_probe(rx): %llu beacons\n",
            (unsigned long long)g_rx.load());
    _exit(0);
  }

  int n_updates = argc > 1 ? atoi(argv[1]) : 30;
  int k_intervals = argc > 2 ? atoi(argv[2]) : 10;
  dev->InitWrite(SelectedChannel{ch, 0, CHANNEL_WIDTH_20});
  std::this_thread::sleep_for(std::chrono::seconds(2));
  dev->SetCcaMode(true);

  uint32_t ver = 1;
  auto f = build_beacon(ver, interval_tu);
  if (!dev->StartBeacon(f.data(), f.size(), interval_tu)) {
    fprintf(stderr, "StartBeacon FAILED\n");
    return 1;
  }
  printf("{\"ev\":\"bcn.start\",\"ver\":%u,\"interval_tu\":%d,\"host_ns\":%lld}\n",
         ver, interval_tu, (long long)host_ns());
  fflush(stdout);
  std::this_thread::sleep_for(std::chrono::seconds(3));  // baseline window

  const auto gap = std::chrono::microseconds((int64_t)k_intervals * interval_tu * 1024);
  for (int i = 0; i < n_updates; ++i) {
    std::this_thread::sleep_for(gap);
    ++ver;
    auto nf = build_beacon(ver, interval_tu);
    int64_t t0 = host_ns();
    bool ok = dev->UpdateBeaconPayload(nf.data(), nf.size());
    int64_t t1 = host_ns();
    printf("{\"ev\":\"bcn.update\",\"ver\":%u,\"ok\":%s,\"host_ns\":%lld,"
           "\"call_us\":%lld}\n",
           ver, ok ? "true" : "false", (long long)t0,
           (long long)((t1 - t0) / 1000));
    fflush(stdout);
  }
  std::this_thread::sleep_for(std::chrono::seconds(3));  // tail window
  /* The chip beacons autonomously — silence it before exiting, or the stale
   * beacon (same SA) contaminates the next test's witness. */
  dev->StopBeacon();
  fprintf(stderr, "beacon_update_probe(tx): %d updates, final ver %u\n",
          n_updates, ver);
  return 0;
}
