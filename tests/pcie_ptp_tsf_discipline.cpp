// pcie_ptp_tsf_discipline.cpp — the AP-beacon → PTP discipline loop for parts
// whose beacon TBTT is hardware-locked to the TSF grid (Kestrel/AX, Jaguar1):
// there PinBeaconTbtt cannot hold a nonzero offset, and the actuator is the TSF
// itself — step the TSF and the TBTT moves with it in hardware.
//
// Sibling of pcie_ptp_beacon.cpp (the PinBeaconTbtt loop for Jaguar2) with the
// same model and output, so the two are directly comparable: a least-squares
// fit ref = a·tsf + b against a PTP hardware clock, the TBTT at the next TSF
// grid point mapped through the fit, and a PI on the phase error.
//
// The fit's x axis is a free-running counter the TSF steps never touch
// (IRadio::ReadTsfAndFreeRun; Kestrel R_AX_FREERUN_CNT): tsf - freerun is read
// every iteration, so every step's TRUE effect is known and nothing is booked.
// Each sample is bracketed by two PHC reads (midpoint used). The bracket's
// baseline is the read cost itself — ~15-25 us for the six MMIO + two PHC
// reads, more behind a PCIe switch — so a sample is dropped only when its
// bracket exceeds the running median of the last 32 by BRACKET_SLACK_US
// (default 5; host preemption between the reads), or an explicit
// BRACKET_MAX_US cap when set. LEDGER=1 (or a
// part without a free-running counter) falls back to the older ledger axis,
// tsf - S with S = sum of commanded steps, which assumes every read-add-write
// landed exactly STEP_COMP_US late. On the bench it didn't: three cells drifted
// ~10 us rms apart on air while each loop's own estimate stayed sub-us.
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
#include "cell/TsfDiscipline.h"
#include "logger.h"

#define FD_TO_CLOCKID(fd) ((~(clockid_t)(fd) << 3) | 3)

static double phc_us(clockid_t c) {
  struct timespec t;
  clock_gettime(c, &t);
  return (double)t.tv_sec * 1e6 + (double)t.tv_nsec / 1e3;
}

// Strict env parsing: a malformed or non-finite value is a configuration error,
// not something to fall back from silently (atof would turn "x" into 0 and let
// "nan" reach the steering maths).
static double envd(const char *k, double d) {
  const char *v = std::getenv(k);
  if (!v) return d;
  char *end = nullptr;
  const double x = std::strtod(v, &end);
  if (end == v || *end != '\0' || !std::isfinite(x)) {
    fprintf(stderr, "invalid %s=\"%s\" (want a finite number)\n", k, v);
    std::exit(2);
  }
  return x;
}
// Integer in [lo, hi], else a configuration error.
static long envi(const char *k, long d, long lo, long hi) {
  const char *v = std::getenv(k);
  if (!v) return d;
  char *end = nullptr;
  const long x = std::strtol(v, &end, 0);
  if (end == v || *end != '\0' || x < lo || x > hi) {
    fprintf(stderr, "invalid %s=\"%s\" (want an integer in %ld..%ld)\n", k, v, lo, hi);
    std::exit(2);
  }
  return x;
}

int main(int argc, char **argv) {
  const char *bdf = std::getenv("DEVOURER_PCIE_BDF");
  if (!bdf) { fprintf(stderr, "set DEVOURER_PCIE_BDF\n"); return 2; }
  const char *ref = std::getenv("REF_PTP"); if (!ref) ref = "/dev/ptp0";
  const uint8_t ch = (uint8_t)envi("DEVOURER_CHANNEL", 36, 1, 196);
  const int secs = argc > 1 ? atoi(argv[1]) : 180;
  const int interval_tu = (int)envi("BCN_TU", 100, 1, 65535);  // the period is a divisor
  const int64_t interval = (int64_t)interval_tu * 1024;
  const double gain = envd("GAIN", 0.9), ki = envd("KI", 0.10);
  const double capture = envd("CAPTURE_US", 300);
  const int loop_ms = (int)envd("LOOP_MS", 500);
  const double comp = envd("STEP_COMP_US", 2.0);
  const size_t window = (size_t)envd("FIT_WINDOW", 64);
  const long settle = (long)envd("SETTLE", 40);
  // Multi-node runs: PHASE_US puts this node's TBTT at ref % interval ==
  // PHASE_US (stagger nodes so their beacons don't collide on one channel);
  // NODE_ID is the last octet of the beacon SA/BSSID so an observer can tell
  // the nodes apart (tests/beacon_phase_observer.cpp).
  const double phase = envd("PHASE_US", 0);
  const uint8_t node_id = (uint8_t)envi("NODE_ID", 0, 0, 255);
  const double bracket_max = envd("BRACKET_MAX_US", 0);   // 0 = adaptive
  const double bracket_slack = envd("BRACKET_SLACK_US", 5);
  // BCN_BACKOFF=0: deterministic departure (fixed AIFS after the TBTT) — only
  // with co-channel cells staggered, as the multi-cell tests do via PHASE_US.
  const bool bcn_backoff = envi("BCN_BACKOFF", 1, 0, 1) != 0;

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
  // Before StartBeacon: a refusal must not leave a beacon airing.
  if (!bcn_backoff && !dev->SetBeaconBackoff(false)) {
    fprintf(stderr, "BCN_BACKOFF=0 refused (beacon_backoff_ctl=%d)\n",
            dev->GetAdapterCaps().beacon_backoff_ctl ? 1 : 0);
    return 1;
  }
  std::vector<uint8_t> bcn = {
      0x80,0x00,0x00,0x00, 0xff,0xff,0xff,0xff,0xff,0xff,
      0x57,0x42,0x75,0x05,0xd6,node_id, 0x57,0x42,0x75,0x05,0xd6,node_id, 0x00,0x00,
      0,0,0,0,0,0,0,0,
      (uint8_t)(interval_tu & 0xff), (uint8_t)(interval_tu >> 8), 0x00,0x00,
      0x00,0x03,'P','T','P', 0x01,0x01,0x82};
  if (!dev->StartBeacon(bcn.data(), bcn.size(), interval_tu)) {
    fprintf(stderr, "StartBeacon UNSUPPORTED\n"); return 1;
  }
  dev->SetCcaMode(true);
  std::this_thread::sleep_for(std::chrono::seconds(2));
  uint64_t probe_t = 0, probe_f = 0;
  const bool use_fr = envd("LEDGER", 0) == 0 && dev->ReadTsfAndFreeRun(probe_t, probe_f);
  fprintf(stderr, "pcie_ptp_tsf_discipline: fit axis = %s\n",
          use_fr ? "free-running counter (steps measured)" : "ledger (steps booked)");

  devourer::cell::TsfDisciplineConfig dc;
  dc.interval_us = interval;
  dc.phase_us = phase;
  dc.gain = gain;
  dc.ki = ki;
  dc.capture_us = capture;
  dc.window = window;
  dc.bracket_max_us = bracket_max;
  dc.bracket_slack_us = bracket_slack;
  dc.step_comp_us = comp;
  dc.ledger = !use_fr;
  devourer::cell::TsfDiscipline loop(dc);
  long n = 0;
  double last_bracket = 0;
  std::vector<double> locked;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(secs);
  while (std::chrono::steady_clock::now() < deadline) {
    /* One bracketed sample: PHC, radio (tsf, freerun), PHC. On the ledger
     * axis the radio read is ReadTsf alone (no free-running counter). */
    uint64_t t = 0, f = 0;
    const double r1 = phc_us(refclk);
    bool ok_read = true;
    if (use_fr)
      ok_read = dev->ReadTsfAndFreeRun(t, f);
    else
      t = dev->ReadTsf();
    const double r2 = use_fr ? phc_us(refclk) : r1;
    const auto d = loop.sample(r1, r2, ok_read, (int64_t)t, (int64_t)f);
    if (d.accepted) {
      ++n;
      last_bracket = d.bracket_us;
    }
    if (d.acting) {
      bool ok = true;
      if (d.step_us != 0) {
        const uint64_t now = dev->ReadTsf();
        ok = dev->WriteTsf(now + (uint64_t)d.write_us);
        loop.wrote(ok);
        if (!ok) {
          // The counter may be half-updated: its axis is no longer the one the
          // fit window was built on; the loop dropped the fit and re-anchors
          // from fresh samples. Give up if the device keeps refusing.
          fprintf(stderr, "WriteTsf failed (TSF now 0x%016llx) — refitting\n",
                  (unsigned long long)dev->ReadTsf());
          if (loop.write_fail_run() >= 3) {
            fprintf(stderr, "3 consecutive TSF write failures — stopping\n");
            break;
          }
        }
      }
      if (n > settle) locked.push_back(d.phase_err_us);
      printf("{\"ev\":\"ptptsf\",\"i\":%ld,\"phase_us\":%.1f,\"step_us\":%lld,"
             "\"ok\":%d,\"ppm\":%.2f,\"bracket_us\":%.1f}\n",
             n, d.phase_err_us, (long long)d.step_us, ok ? 1 : 0, d.ppm, last_bracket);
      fflush(stdout);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(loop_ms));
  }
  if (!locked.empty()) {
    double ss = 0;
    std::vector<double> ab;
    for (double e : locked) { ss += e * e; ab.push_back(std::fabs(e)); }
    std::sort(ab.begin(), ab.end());
    printf("{\"ev\":\"ptptsf.summary\",\"bdf\":\"%s\",\"axis\":\"%s\",\"n\":%zu,"
           "\"rejected\":%ld,\"rms_us\":%.2f,\"p95_abs_us\":%.1f,\"max_abs_us\":%.1f}\n",
           bdf, use_fr ? "freerun" : "ledger", locked.size(), (long)loop.rejected(),
           std::sqrt(ss / locked.size()), ab[ab.size() * 95 / 100], ab.back());
    fflush(stdout);
  }
  dev->Stop();
  return 0;
}
