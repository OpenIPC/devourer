// pcie_ptp_tsf_discipline.cpp — the AP-beacon → PTP discipline loop for parts
// whose beacon TBTT is hardware-locked to the TSF grid (Kestrel/AX, Jaguar1):
// there PinBeaconTbtt cannot hold a nonzero offset, and the actuator is the TSF
// itself — step the TSF and the TBTT moves with it in hardware.
//
// Sibling of pcie_ptp_beacon.cpp (the PinBeaconTbtt loop for Jaguar2) with the
// same model and output, so the two are directly comparable: a least-squares
// fit ref = a·tsf + b against a PTP hardware clock, the TBTT at the next TSF
// grid point mapped through the fit, and a PI on the phase error. The steps
// are subtracted out of the fit's TSF axis (tsf - S, S = sum of applied
// steps), and the fit is a sliding window so step-accounting error ages out.
// A read-add-write lands STEP_COMP_US late (the TSF runs during the write);
// the default 2 µs is the bench readback on an RTL8852CE behind PCIe.
//
// Build: the pcie_ptp_tsf_discipline CMake target (DEVOURER_PCIE=ON).
// Run: sudo DEVOURER_PCIE_BDF=0000:05:00.0 DEVOURER_CHANNEL=36 REF_PTP=/dev/ptp0 \
//        build/pcie_ptp_tsf_discipline [secs]
// Output: one {"ev":"ptptsf",...} line per iteration, then a summary line
//         with the locked-phase RMS / p95 / max (iterations after SETTLE).
#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <memory>
#include <thread>
#include <vector>

#include "PcieTransport.h"
#include "SelectedChannel.h"
#include "WiFiDriver.h"
#include "logger.h"

#define FD_TO_CLOCKID(fd) ((~(clockid_t)(fd) << 3) | 3)

static int64_t phc_us(clockid_t c) {
  struct timespec t;
  clock_gettime(c, &t);
  return (int64_t)t.tv_sec * 1000000 + t.tv_nsec / 1000;
}

static double envd(const char *k, double d) {
  const char *v = std::getenv(k);
  return v ? atof(v) : d;
}

int main(int argc, char **argv) {
  const char *bdf = std::getenv("DEVOURER_PCIE_BDF");
  if (!bdf) { fprintf(stderr, "set DEVOURER_PCIE_BDF\n"); return 2; }
  const char *ref = std::getenv("REF_PTP"); if (!ref) ref = "/dev/ptp0";
  const uint8_t ch = (uint8_t)envd("DEVOURER_CHANNEL", 36);
  const int secs = argc > 1 ? atoi(argv[1]) : 180;
  const int interval_tu = (int)envd("BCN_TU", 100);
  const int64_t interval = (int64_t)interval_tu * 1024;
  const double gain = envd("GAIN", 0.9), ki = envd("KI", 0.10);
  const double capture = envd("CAPTURE_US", 300);
  const int loop_ms = (int)envd("LOOP_MS", 500);
  const double comp = envd("STEP_COMP_US", 2.0);
  const size_t window = (size_t)envd("FIT_WINDOW", 64);
  const long settle = (long)envd("SETTLE", 40);

  int fd = open(ref, O_RDONLY);
  if (fd < 0) { fprintf(stderr, "open %s failed\n", ref); return 1; }
  const clockid_t refclk = FD_TO_CLOCKID(fd);

  auto logger = std::make_shared<Logger>();
  auto t = devourer::PcieTransport::Open(bdf, logger);
  if (!t) { fprintf(stderr, "pcie open failed\n"); return 1; }
  WiFiDriver wifi(logger);
  auto dev = wifi.CreateRadioPcie(std::move(t));
  if (!dev) { fprintf(stderr, "create failed\n"); return 1; }
  {
    const auto caps = dev->GetAdapterCaps();
    if (!caps.tsf_write_ok || !caps.tbtt_follows_tsf) {
      // A TSF write that doesn't move the beacon would "lock" the loop's
      // estimate while the on-air TBTT stays put (Jaguar2/3: use
      // tests/pcie_ptp_beacon.cpp, the PinBeaconTbtt loop).
      fprintf(stderr, "this part cannot steer its beacon through the TSF "
                      "(tsf_write_ok=%d tbtt_follows_tsf=%d)\n",
              caps.tsf_write_ok ? 1 : 0, caps.tbtt_follows_tsf ? 1 : 0);
      return 1;
    }
  }
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

  struct Pt { double x, y; };
  std::deque<Pt> pts;  // (tsf - S - x0, ref - y0)
  bool init = false;
  double x0 = 0, y0 = 0, S = 0, integ = 0;
  long n = 0;
  int write_fail_run = 0;
  std::vector<double> locked;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(secs);
  while (std::chrono::steady_clock::now() < deadline) {
    const int64_t tsf = (int64_t)dev->ReadTsf();
    const int64_t r = phc_us(refclk);
    if (!init) { x0 = (double)tsf; y0 = (double)r; init = true; }
    pts.push_back({(double)tsf - S - x0, (double)r - y0});
    if (pts.size() > window) pts.pop_front();
    ++n;
    if (pts.size() >= 8) {
      double sx = 0, sy = 0, sxx = 0, sxy = 0;
      const double m = (double)pts.size();
      for (const auto &p : pts) { sx += p.x; sy += p.y; sxx += p.x * p.x; sxy += p.x * p.y; }
      const double den = m * sxx - sx * sx;
      const double a = den != 0 ? (m * sxy - sx * sy) / den : 1.0;
      const double b = (sy - a * sx) / m;
      // The TBTT sits on the (real) TSF grid: next grid point after now.
      const int64_t next_tbtt = (tsf / interval + 1) * interval;
      const double tbtt_ref = y0 + a * ((double)next_tbtt - S - x0) + b;
      double e = std::fmod(tbtt_ref, (double)interval);
      if (e > interval / 2.0) e -= interval; else if (e < -interval / 2.0) e += interval;
      if (std::fabs(e) < capture) {
        integ = std::clamp(integ + e, -1500.0, 1500.0);
      } else {
        integ = 0;
      }
      // TBTT lands e late in ref time: advance the TSF by u so the grid point
      // arrives u earlier.
      const double u = gain * e + ki * integ;
      const int64_t step = (int64_t)std::llround(u);
      bool ok = true;
      if (step != 0) {
        const uint64_t now = dev->ReadTsf();
        ok = dev->WriteTsf(now + (uint64_t)(step + (int64_t)std::llround(comp)));
        if (ok) {
          S += (double)step;
          write_fail_run = 0;
        } else {
          // The counter may be half-updated: its axis is no longer the one the
          // fit window was built on. Drop the fit and the ledger and re-anchor
          // from fresh samples; give up if the device keeps refusing.
          fprintf(stderr, "WriteTsf failed (TSF now 0x%016llx) — refitting\n",
                  (unsigned long long)dev->ReadTsf());
          pts.clear();
          init = false;
          S = 0;
          integ = 0;
          if (++write_fail_run >= 3) {
            fprintf(stderr, "3 consecutive TSF write failures — stopping\n");
            break;
          }
        }
      }
      if (n > settle) locked.push_back(e);
      printf("{\"ev\":\"ptptsf\",\"i\":%ld,\"phase_us\":%.1f,\"step_us\":%lld,"
             "\"ok\":%d,\"ppm\":%.2f}\n",
             n, e, (long long)step, ok ? 1 : 0, (a - 1.0) * 1e6);
      fflush(stdout);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(loop_ms));
  }
  if (!locked.empty()) {
    double ss = 0;
    std::vector<double> ab;
    for (double e : locked) { ss += e * e; ab.push_back(std::fabs(e)); }
    std::sort(ab.begin(), ab.end());
    printf("{\"ev\":\"ptptsf.summary\",\"bdf\":\"%s\",\"n\":%zu,\"rms_us\":%.2f,"
           "\"p95_abs_us\":%.1f,\"max_abs_us\":%.1f}\n",
           bdf, locked.size(), std::sqrt(ss / locked.size()),
           ab[ab.size() * 95 / 100], ab.back());
  }
  dev->Stop();
  return 0;
}
