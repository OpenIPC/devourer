// pcie_ptp_beacon.cpp — the AP-beacon → PTP discipline loop, on a PCIe adapter.
//
// Runs a hardware beacon on the 8821CE and holds its TBTT to a network PTP
// reference (the Intel I226 PHC, read directly via FD_TO_CLOCKID — nothing
// disciplines the system clock). Uses PinBeaconTbtt (#245): an ABSOLUTE,
// TSF-preserving actuator, so steering doesn't corrupt the ref=a*tsf+b fit the
// loop reads — which lets a full-gain PI controller hold tight, unlike the
// incremental AdjustBeaconTimingFine (which jumped the TSF ~5 ms/steer and drove
// a PI into a limit cycle; the proportional-only sweet spot there was ~60 µs).
//
// Build (DEVOURER_PCIE=ON tree):
//   g++ -std=c++20 -O2 -DDEVOURER_HAVE_PCIE -Isrc tests/pcie_ptp_beacon.cpp \
//     build-pcie/libdevourer.a $(pkg-config --cflags --libs libusb-1.0) \
//     -lpthread -o build-pcie/pcie_ptp_beacon
// Run: sudo DEVOURER_PCIE_BDF=0000:01:00.0 DEVOURER_CHANNEL=6 REF_PTP=/dev/ptp0 \
//        build-pcie/pcie_ptp_beacon [secs]
#include <fcntl.h>
#include <time.h>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <thread>
#include <vector>

#include "PcieTransport.h"
#include "SelectedChannel.h"
#include "WiFiDriver.h"
#include "logger.h"

#define FD_TO_CLOCKID(fd) ((~(clockid_t)(fd) << 3) | 3)

static int64_t phc_us(clockid_t c) {
  struct timespec t; clock_gettime(c, &t);
  return (int64_t)t.tv_sec * 1000000 + t.tv_nsec / 1000;
}

int main(int argc, char **argv) {
  const char *bdf = std::getenv("DEVOURER_PCIE_BDF");
  if (!bdf) { fprintf(stderr, "set DEVOURER_PCIE_BDF\n"); return 2; }
  const char *ref = std::getenv("REF_PTP"); if (!ref) ref = "/dev/ptp0";
  uint8_t ch = 6; if (const char *c = std::getenv("DEVOURER_CHANNEL")) ch = atoi(c);
  int secs = argc > 1 ? atoi(argv[1]) : 180;
  int interval_tu = 100; if (const char *i = std::getenv("BCN_TU")) interval_tu = atoi(i);
  const int64_t interval = (int64_t)interval_tu * 1024;             // µs
  double gain = 0.9; if (const char *g = std::getenv("GAIN")) gain = atof(g);
  double ki = 0.10; if (const char *k = std::getenv("KI")) ki = atof(k);
  double capture = 300; if (const char *ca = std::getenv("CAPTURE_US")) capture = atof(ca);
  int loop_ms = 500; if (const char *l = std::getenv("LOOP_MS")) loop_ms = atoi(l);

  int fd = open(ref, O_RDONLY);
  if (fd < 0) { fprintf(stderr, "open %s failed\n", ref); return 1; }
  clockid_t refclk = FD_TO_CLOCKID(fd);

  auto logger = std::make_shared<Logger>();
  auto t = devourer::PcieTransport::Open(bdf, logger);
  if (!t) { fprintf(stderr, "pcie open failed\n"); return 1; }
  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRtlDevicePcie(std::move(t));
  if (!dev) { fprintf(stderr, "create failed\n"); return 1; }
  dev->InitWrite(SelectedChannel{ch, 0, CHANNEL_WIDTH_20});

  std::vector<uint8_t> bcn = {
      0x80,0x00,0x00,0x00, 0xff,0xff,0xff,0xff,0xff,0xff,
      0x57,0x42,0x75,0x05,0xd6,0x00, 0x57,0x42,0x75,0x05,0xd6,0x00, 0x00,0x00,
      0,0,0,0,0,0,0,0,
      (uint8_t)(interval_tu & 0xff), (uint8_t)(interval_tu >> 8), 0x00,0x00,
      0x00,0x03,'P','T','P', 0x01,0x01,0x82};
  if (!dev->StartBeacon(bcn.data(), bcn.size(), interval_tu)) {
    fprintf(stderr, "StartBeacon UNSUPPORTED\n"); return 1;
  }
  dev->SetCcaMode(true);
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // running least-squares ref = a*tsf + b (offset coords) — stays clean because
  // PinBeaconTbtt preserves the TSF timeline.
  bool init = false; double x0 = 0, y0 = 0, sx = 0, sy = 0, sxx = 0, sxy = 0; long n = 0;
  double cur = 0;                              // current commanded TBTT offset (µs into interval)
  dev->PinBeaconTbtt(0);
  double integ = 0;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(secs);
  while (std::chrono::steady_clock::now() < deadline) {
    int64_t tsf = (int64_t)dev->ReadTsf();
    int64_t r = phc_us(refclk);
    if (!init) { x0 = tsf; y0 = r; init = true; }
    double xi = tsf - x0, yi = r - y0;
    ++n; sx += xi; sy += yi; sxx += xi * xi; sxy += xi * yi;
    if (n >= 8) {
      double den = n * sxx - sx * sx;
      double a = den ? (n * sxy - sx * sy) / den : 1.0;
      double b = (sy - a * sx) / n;
      // next TBTT (TSF) with tsf ≡ cur (mod interval); its reference time via the fit
      int64_t next_tbtt = ((int64_t)((tsf - (int64_t)cur) / interval) + 1) * interval + (int64_t)cur;
      double tbtt_ref = y0 + a * (next_tbtt - x0) + b;
      double e = fmod(tbtt_ref, (double)interval);
      if (e > interval / 2.0) e -= interval; else if (e < -interval / 2.0) e += interval;
      // PI (integral only near lock — anti-windup); full gain is safe now.
      if (std::fabs(e) < capture) { integ += e; if (integ > 1500) integ = 1500; if (integ < -1500) integ = -1500; }
      else integ = 0;
      cur -= (gain * e + ki * integ);
      cur = fmod(cur, (double)interval); if (cur < 0) cur += interval;   // normalize into period
      int32_t applied = dev->PinBeaconTbtt((int32_t)llround(cur));
      printf("{\"ev\":\"ptpbcn\",\"i\":%ld,\"phase_us\":%.1f,\"offset_us\":%.0f,\"applied\":%d,\"ppm\":%.1f}\n",
             n, e, cur, applied, (a - 1.0) * 1e6);
      fflush(stdout);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(loop_ms));
  }
  fprintf(stderr, "pcie_ptp_beacon: %ld iterations\n", n);
  return 0;
}
