// mcast_da_tx.cpp — gate for carrying telemetry in addr1 (devourer#474):
// does a monitor receiver deliver a stream frame whose DA is a non-broadcast
// GROUP address? Alternates the canonical stream probe request with DA
// ff:ff:ff:ff:ff:ff and the same frame with DA 03:57:42:75:05:d6 (group +
// locally administered, not broadcast); the body carries "DAff" / "DA03" so
// a witness tells them apart by content alone. Every TX descriptor path sets
// BMC from addr1's group bit, so the air behaviour is the same either way —
// what is under test is each receiver family's RX filter.
// Driven by tests/mcast_da_rx_check.sh. Build recipe: dl_departure_matrix.sh.
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

#include <libusb.h>

#include "RadiotapBuilder.h"
#include "SelectedChannel.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"

static const uint8_t kSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
static const uint8_t kGroupDa[6] = {0x03, 0x57, 0x42, 0x75, 0x05, 0xd6};

static std::vector<uint8_t> build(const std::vector<uint8_t>& rt, const uint8_t da[6],
                                  const char* tag, uint32_t n) {
  std::vector<uint8_t> f(rt);
  const uint8_t hdr[24] = {0x40, 0x00, 0x00, 0x00,
                           da[0], da[1], da[2], da[3], da[4], da[5],
                           kSa[0], kSa[1], kSa[2], kSa[3], kSa[4], kSa[5],
                           kSa[0], kSa[1], kSa[2], kSa[3], kSa[4], kSa[5],
                           0x00, 0x00};
  f.insert(f.end(), hdr, hdr + 24);
  f.insert(f.end(), tag, tag + 4);
  for (int i = 0; i < 4; ++i) f.push_back((uint8_t)(n >> (8 * i)));
  f.insert(f.end(), 40, 0x5a);
  return f;
}

int main(int argc, char** argv) {
  int secs = argc > 1 ? atoi(argv[1]) : 15;
  int gap_us = 5000;
  if (const char* g = std::getenv("GAP_US")) gap_us = atoi(g);
  uint16_t vid = 0x0bda, pid = 0xc812;
  if (const char* p = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(p, 0, 0);
  if (const char* v = std::getenv("DEVOURER_VID")) vid = (uint16_t)strtoul(v, 0, 0);
  uint8_t ch = 6;
  if (const char* c = std::getenv("DEVOURER_CHANNEL")) ch = atoi(c);

  auto logger = std::make_shared<Logger>();
  libusb_context* ctx = nullptr;
  libusb_init(&ctx);
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
  auto* h = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!h) { fprintf(stderr, "open fail %04x:%04x\n", vid, pid); return 1; }
  std::shared_ptr<devourer::UsbDeviceLock> lk;
  if (devourer::claim_interface_then_reset(h, devourer::find_wifi_interface(h), logger, true, lk) != 0) return 1;
  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRadio(h, ctx, lk, devourer_config_from_env());
  if (!dev) return 1;
  try {
    dev->InitWrite(SelectedChannel{ch, 0, CHANNEL_WIDTH_20});
  } catch (const std::exception& e) {
    fprintf(stderr, "bring-up failed: %s\n", e.what());
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::seconds(2));

  const auto rt = devourer::build_stream_radiotap(devourer::parse_tx_mode_str("6M"));
  static const uint8_t bcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  uint32_t n = 0, ok_ff = 0, ok_03 = 0;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(secs);
  while (std::chrono::steady_clock::now() < deadline) {
    const bool grp = n & 1;
    auto f = build(rt, grp ? kGroupDa : bcast, grp ? "DA03" : "DAff", n);
    if (dev->send_packet(f.data(), f.size())) (grp ? ok_03 : ok_ff)++;
    ++n;
    std::this_thread::sleep_for(std::chrono::microseconds(gap_us));
  }
  dev->Stop();
  printf("{\"ev\":\"mcast.tx\",\"sent\":%u,\"ok_ff\":%u,\"ok_03\":%u}\n", n, ok_ff, ok_03);
  return 0;
}
