/* sense — a runnable Wi-Fi motion/presence sensor built on devourer.
 *
 * An 802.11ac beamforming report is a measurement taken at the beamformee: its
 * per-tone Givens angles track the channel, so a moving person perturbs them
 * frame-to-frame. This demo captures reports, decodes them (src/BfReportDecode.h),
 * and shows a live motion readout — the per-tone cross-frame variance of the
 * phase angle. See docs/beamforming-victim-sensing.md.
 *
 * One binary drives TWO adapters: a sounder that injects NDPAs (the MAC
 * hardware-generates the NDP) and self-captures the reports, and a beamformee
 * that responds in hardware. Two dongles; the effect is stronger when they are
 * physically separated (a static short channel barely moves).
 */
#ifdef _WIN32
#define NOMINMAX /* keep windows.h (via libusb.h) from defining min/max macros */
#endif

#if defined(__ANDROID__) || defined(_MSC_VER) || defined(__APPLE__)
#include <libusb.h>
#else
#include <libusb-1.0/libusb.h>
#endif

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "BfReportDecode.h"
#include "RadiotapBuilder.h"
#include "RxPacket.h"
#include "SignalStop.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"

using devourer::bf::MotionMeter;
using devourer::bf::parse_report;
using devourer::bf::ReportHdr;

/* Portable environment set (the demo hands arming flags to the library via env):
 * POSIX setenv, or _putenv_s on Windows (MSVC + MinGW have no POSIX setenv). */
static void set_env(const char *name, const char *value) {
#ifdef _WIN32
  _putenv_s(name, value);
#else
  ::setenv(name, value, 1);
#endif
}

/* The sounder's TA and the address a beamformee arms to respond to (matches the
 * canonical SA used across devourer's TX path). */
static const uint8_t kCanonicalSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
static constexpr uint16_t REG_MACID = 0x0610;
static constexpr int kCalReports = 48;   /* reports to calibrate the bit split */
static constexpr size_t kWindow = 512;   /* variance window (~0.3 s at a fast
                                          * Jaguar3 sounding rate — long enough
                                          * to span human motion, short enough
                                          * to feel live) */
static constexpr int kStallSec = 8;      /* no reports for this long => the
                                          * beamformee stopped responding; stop
                                          * rather than sound into the void */

/* -------------------------------------------------------------- Detector ---- */
/* Adaptive presence detector. Fed the MotionMeter's energy once per report, it
 * self-calibrates a noise floor and fires when the energy rises a configurable
 * number of standard deviations above it (a CFAR-style test), then holds the
 * verdict for a short time so a moving-then-pausing subject doesn't flicker.
 *
 * Why not a fixed threshold: the still-state energy floor varies with hardware,
 * channel and geometry, so a magic constant tuned on one rig is wrong on the
 * next. The floor tracks the quiet state with a rate-independent time constant
 * but FREEZES while motion is held — otherwise a subject who keeps moving would
 * slowly raise the floor and blind the detector (the classic motion-sensor
 * failure). During the initial warm-up the floor latches onto the quietest
 * instant seen, so a subject already moving at startup can't set a high floor. */
class AdaptiveDetector {
public:
  explicit AdaptiveDetector(double k) : _k(k) {}

  /* Call only once the MotionMeter window is full, so `energy` is a real
   * still-state estimate and not the window-fill transient (which is ~0 and
   * would peg the floor at zero -> always-MOTION). */
  void update(double energy, double t) {
    _energy = energy;
    if (!_init) {
      _floor = energy;
      _dev = kDevSeed;
      _last = t;
      _warm_until = t + kWarmSec;
      _init = true;
      return;
    }
    double dt = t - _last;
    if (dt < 0)
      dt = 0;
    _last = t;
    bool warming = t < _warm_until;
    _warm = warming;
    /* Track the floor (mean still energy) and its jitter with an EMA — fast
     * during warm-up to converge, slow after. Freeze while motion is held so a
     * moving subject can't drag the floor up and blind the detector. */
    if (warming || !_active) {
      double tau = warming ? kWarmTau : kTauFloor;
      double a = 1.0 - std::exp(-dt / tau);
      double resid = energy - _floor;
      _floor += a * resid;
      _dev += a * (std::fabs(resid) - _dev);
    }
    _thr = _floor + _k * std::max(_dev, kDevFloor);
    if (warming)
      return; /* no verdict until the floor is acquired */
    /* Hysteresis: arm only after the energy stays over threshold for kArmSec
     * (a lone noise spike can't trigger), then hold kHoldSec after it drops (a
     * moving-then-pausing subject doesn't flicker). */
    if (energy > _thr) {
      if (_over_since < 0.0)
        _over_since = t;
      if (_active || (t - _over_since) >= kArmSec) {
        _active = true;
        _hold = kHoldSec;
      }
    } else {
      _over_since = -1.0;
      if (_active) {
        _hold -= dt;
        if (_hold <= 0.0)
          _active = false;
      }
    }
  }

  bool active() const { return _active; }
  bool warming() const { return _warm; }
  double energy() const { return _energy; }
  double floor() const { return _floor; }
  /* signal strength: how many floor-jitter sigmas the energy sits above the
   * floor. The detector fires at sigma == k. */
  double sigma() const { return (_energy - _floor) / std::max(_dev, kDevFloor); }

private:
  static constexpr double kWarmSec = 2.5;    /* floor-acquisition window (s) */
  static constexpr double kWarmTau = 0.4;    /* fast tracking during warm-up (s) */
  static constexpr double kTauFloor = 4.0;   /* floor/jitter tracking constant (s) */
  static constexpr double kArmSec = 0.15;      /* dwell over threshold before MOTION (s) */
  static constexpr double kHoldSec = 1.2;      /* hold MOTION after last trigger (s) */
  /* Scaled to the measured signal: with the correct (6,4) split the still-channel
   * phi circular-variance floor is ~0.0003 and its window-to-window jitter is
   * ~0.00002; a hand wave lifts it to ~0.0006–0.002. So the minimum sensitivity
   * margin must be tens of micro-units, not milli-units. */
  static constexpr double kDevSeed = 0.00005;  /* initial jitter estimate */
  static constexpr double kDevFloor = 0.00004; /* min jitter → min sensitivity margin */
  double _k;
  bool _init = false, _active = false, _warm = true;
  double _energy = 0, _floor = 0, _dev = 0, _thr = 0;
  double _last = 0, _warm_until = 0, _hold = 0, _over_since = -1.0;
};

/* ---------------------------------------------------------------- Sensor ---- */
/* Turns a stream of report frames into a live motion signal. Thread-safe: the
 * RX callback calls feed(); the display thread reads the snapshot. */
class Sensor {
public:
  explicit Sensor(double k) : _det(k) {}

  void feed(const uint8_t *frame, size_t n) {
    ReportHdr hdr;
    if (!parse_report(frame, n, hdr))
      return;
    if (std::getenv("DEVOURER_SENSE_DUMP")) {
      /* python-tool-compatible raw dump (stderr, so the stdout display is
       * untouched): capture with 2>file, analyse with tools/bf_report_decode.py */
      std::fprintf(stderr, "<devourer-bf-report-raw>");
      for (size_t i = 0; i < n; ++i)
        std::fprintf(stderr, "%02x", frame[i]);
      std::fprintf(stderr, "\n");
    }
    if (std::getenv("DEVOURER_SENSE_DEBUG")) {
      static int dbg = 0;
      if (dbg < 8) {
        ++dbg;
        std::fprintf(stderr,
                     "[report] len=%zu SA=%02x:%02x:%02x:%02x:%02x:%02x nc=%d "
                     "nr=%d mu=%d ns=%d\n",
                     n, frame[10], frame[11], frame[12], frame[13], frame[14],
                     frame[15], hdr.nc, hdr.nr, hdr.mu, hdr.ns);
      }
    }
    std::lock_guard<std::mutex> lk(_mu);
    ++_total;
    if (!_meter) {
      /* calibration: copy full frames until we can pick a stable split */
      _cal.emplace_back(frame, frame + n);
      _ns = hdr.ns;
      _per = hdr.per_sc_bits;
      if ((int)_cal.size() >= kCalReports)
        calibrate();
      return;
    }
    if (hdr.ns != _ns)
      return;
    if (!_meter->push(hdr))
      return;
    /* Wait for a full window before feeding the detector: a partial window
     * under-reports circular variance, and that transient would poison the
     * self-calibrating floor. */
    if (_meter->count() < kWindow)
      return;
    _det.update(_meter->motion_energy(), now_sec());
    _localized = _meter->localized();
  }

  struct Snap {
    bool calibrated, warming, motion, localized;
    double energy, floor, sigma;
    long total;
    int ns;
  };
  Snap snapshot() {
    std::lock_guard<std::mutex> lk(_mu);
    bool cal = _meter != nullptr;
    return Snap{cal,        cal && _det.warming(),
                _det.active(), _localized,
                _det.energy(), _det.floor(),
                cal ? _det.sigma() : 0.0,
                _total,     _ns};
  }

private:
  static double now_sec() {
    using namespace std::chrono;
    return duration_cast<duration<double>>(
               steady_clock::now().time_since_epoch())
        .count();
  }

  void calibrate() {
    /* re-parse the copies so ReportHdr.angles point into stable storage */
    std::vector<ReportHdr> batch;
    batch.reserve(_cal.size());
    for (auto &f : _cal) {
      ReportHdr h;
      if (parse_report(f.data(), f.size(), h))
        batch.push_back(h);
    }
    int bphi = 0, bpsi = 0;
    if (!devourer::bf::pick_split(batch, bphi, bpsi)) {
      /* fallback for the 2x1 case: the standard Givens split (b_phi = b_psi + 2)
       * summing to per_sc_bits — NOT a coarse split like (8,2), whose
       * bit-misaligned phi decodes to garbage. */
      bpsi = (_per - 2) / 2;
      bphi = bpsi + 2;
    }
    _bphi = bphi;
    _bpsi = bpsi;
    _meter = std::make_unique<MotionMeter>(_ns, bphi, bpsi, kWindow);
    _cal.clear();
    _cal.shrink_to_fit();
  }

  std::mutex _mu;
  std::vector<std::vector<uint8_t>> _cal;
  std::unique_ptr<MotionMeter> _meter;
  AdaptiveDetector _det;
  int _ns = 0, _per = 0, _bphi = 0, _bpsi = 0;
  bool _localized = false;
  long _total = 0;
};

/* --------------------------------------------------------------- Display ---- */
static void run_display(Sensor &sensor) {
  using namespace std::chrono;
  auto last = steady_clock::now();
  long last_total = 0;
  std::printf("\n  Wi-Fi motion sensor — move near the adapters to trigger; it "
              "stays CLEAR when the room is still.\n"
              "  (self-calibrating; Ctrl-C to stop)\n\n");
  while (!g_devourer_should_stop) {
    std::this_thread::sleep_for(milliseconds(200));
    auto s = sensor.snapshot();
    auto now = steady_clock::now();
    double dt = duration_cast<duration<double>>(now - last).count();
    double rate = dt > 0 ? (s.total - last_total) / dt : 0;
    last = now;
    last_total = s.total;
    if (!s.calibrated) {
      std::printf("\r  calibrating decoder… %ld reports (%.0f/s)              ",
                  s.total, rate);
      std::fflush(stdout);
      continue;
    }
    if (s.warming) {
      std::printf("\r  acquiring noise floor — one moment…  (%.0f rep/s)        ",
                  rate);
      std::fflush(stdout);
      continue;
    }
    /* Signal strength = floor-jitter sigmas above the adaptive floor; the
     * detector fires around a few sigma. Bar saturates at 10 sigma. */
    double sig = s.sigma;
    if (sig < 0)
      sig = 0;
    int bar = (int)(sig / 10.0 * 40);
    if (bar > 40)
      bar = 40;
    char b[41];
    for (int i = 0; i < 40; ++i)
      b[i] = i < bar ? '#' : ' ';
    b[40] = 0;
    const char *tag = s.motion ? (s.localized ? "MOTION·nb" : " MOTION  ")
                               : "  CLEAR  ";
    std::printf("\r  [%s] %5.1fσ  |%s|  now %.4f base %.4f  %.0f/s ", tag, sig,
                b, s.energy, s.floor, rate);
    std::fflush(stdout);
  }
  std::printf("\n");
}

/* ------------------------------------------------------------ USB helpers --- */
struct Adapter {
  libusb_context *ctx = nullptr;
  libusb_device_handle *handle = nullptr;
  std::shared_ptr<devourer::UsbDeviceLock> lock;
  std::unique_ptr<IRtlDevice> dev;
};

/* Open one adapter by VID:PID on its own libusb context, claim + reset, and build
 * the device (not yet brought up). Returns false (logged) on failure. */
static bool open_adapter(Adapter &a, uint16_t vid, uint16_t pid,
                         const Logger_t &logger) {
  if (libusb_init(&a.ctx) < 0)
    return false;
  a.handle = libusb_open_device_with_vid_pid(a.ctx, vid, pid);
  if (!a.handle) {
    logger->error("could not open {:04x}:{:04x} — is it plugged in? (a prior "
                  "hang can drop an adapter off the bus; unplug/replug it)",
                  vid, pid);
    return false;
  }
  int rc = devourer::claim_interface_then_reset(a.handle, 0, logger, true, a.lock);
  if (rc != 0)
    return false;
  WiFiDriver driver(logger);
  a.dev = driver.CreateRtlDevice(a.handle, a.ctx, a.lock,
                                 devourer_config_from_env());
  return a.dev != nullptr;
}

static bool read_mac(libusb_device_handle *h, uint8_t mac[6]) {
  /* REG_MACID is IDR0 (4 bytes @ 0x0610) + IDR4 (2 bytes @ 0x0614). Realtek
   * vendor reads are 1/2/4-byte; a single 6-byte read returns garbage, so split
   * it the way rtw_read32 + rtw_read16 would. */
  int r1 = libusb_control_transfer(h, 0xC0, 5, REG_MACID, 0, mac, 4, 1000);
  int r2 = libusb_control_transfer(h, 0xC0, 5, REG_MACID + 4, 0, mac + 4, 2, 1000);
  return r1 == 4 && r2 == 2;
}

/* ------------------------------------------------------------- active mode -- */
static int run_active(uint16_t snd_vid, uint16_t snd_pid, uint16_t bfe_vid,
                      uint16_t bfe_pid, int channel, const Logger_t &logger,
                      Sensor &sensor) {
  /* Beamformee first: arm it (env, read at Init), bring it up on a thread. */
  set_env("DEVOURER_BF_ARM_BFEE", "57:42:75:05:d6:00");
  set_env("DEVOURER_BF_ARM_BFEE_MU", "1");
  Adapter bfe;
  if (!open_adapter(bfe, bfe_vid, bfe_pid, logger)) {
    logger->error("active: failed to open beamformee {:04x}", bfe_pid);
    return 1;
  }
  std::thread bfe_thread([&bfe, channel]() {
    bfe.dev->Init([](const Packet &) {}, /* responds in hardware; RX ignored */
                  SelectedChannel{.Channel = (uint8_t)channel, .ChannelOffset = 0,
                                  .ChannelWidth = CHANNEL_WIDTH_20});
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(1500)); /* bring-up + MAC */
  uint8_t bfe_mac[6];
  if (!read_mac(bfe.handle, bfe_mac)) {
    logger->error("active: could not read beamformee MAC (REG_MACID)");
    g_devourer_should_stop = true;
    bfe_thread.join();
    return 1;
  }
  logger->info("active: beamformee MAC {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
               bfe_mac[0], bfe_mac[1], bfe_mac[2], bfe_mac[3], bfe_mac[4],
               bfe_mac[5]);

  /* Sounder: arm the sounding engine (env, read at InitWrite), VHT2SS_MCS0.
   * DEVOURER_TX_WITH_RX=thread must be set BEFORE InitWrite so a Jaguar3 sounder
   * keeps its RX filters open for the self-capture (no-op on Jaguar1/2). */
  set_env("DEVOURER_BF_ARM_SOUNDER", "1");
  set_env("DEVOURER_TX_WITH_RX", "thread");
  Adapter snd;
  if (!open_adapter(snd, snd_vid, snd_pid, logger)) {
    logger->error("active: failed to open sounder {:04x}", snd_pid);
    g_devourer_should_stop = true;
    bfe_thread.join();
    return 1;
  }
  snd.dev->InitWrite(SelectedChannel{.Channel = (uint8_t)channel,
                                     .ChannelOffset = 0,
                                     .ChannelWidth = CHANNEL_WIDTH_20});
  snd.dev->SetTxMode(devourer::parse_tx_mode_str("VHT2SS_MCS0"));
  set_env("DEVOURER_TX_NDPA", "1"); /* send_packet marks the NDPA descriptor */

  /* Self-capture the returned reports on the sounder's RX loop. */
  std::thread snd_rx([&snd, &sensor]() {
    snd.dev->StartRxLoop(
        [&sensor](const Packet &p) { sensor.feed(p.Data.data(), p.Data.size()); });
  });
  std::thread disp(run_display, std::ref(sensor));

  /* Build the NDPA once (10-byte rate-less radiotap + 19-byte VHT NDPA body,
   * RA = beamformee MAC, TA = canonical SA). Rate is the SetTxMode default. */
  std::vector<uint8_t> ndpa = {
      0x00, 0x00, 0x0a, 0x00, 0x00, 0x80, 0x00, 0x00, 0x08, 0x00, /* radiotap */
      0x54, 0x00, 0x64, 0x00,                                     /* NDPA FC+dur */
      bfe_mac[0], bfe_mac[1], bfe_mac[2], bfe_mac[3], bfe_mac[4], bfe_mac[5],
      0x57, 0x42, 0x75, 0x05, 0xd6, 0x00, /* TA */
      0x04, 0x00, 0x10 /* dialog token; STA Info: AID0, MU feedback, Nc0 */};

  logger->info("active: sounding ch{} — move near the setup to see it react",
               channel);
  /* Sound in a loop, watching the report stream. The beamformee's firmware can
   * crash on a long run (and then drop off the USB bus); if reports stop
   * advancing, don't spin forever sounding into the void — report it and stop
   * cleanly, so a stalled stream can't wedge the adapter. */
  using clk = std::chrono::steady_clock;
  long last_total = 0;
  auto last_adv = clk::now();
  bool stalled = false;
  while (!g_devourer_should_stop) {
    if (!snd.dev->send_packet(ndpa.data(), ndpa.size()))
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    long t = sensor.snapshot().total;
    auto now = clk::now();
    if (t != last_total) {
      last_total = t;
      last_adv = now;
    } else if (now - last_adv > std::chrono::seconds(kStallSec)) {
      logger->warn("report stream stalled {}s — beamformee stopped responding; "
                   "stopping (unplug/replug it before re-running)",
                   kStallSec);
      stalled = true;
      g_devourer_should_stop = true;
    }
  }

  /* Deadlock-proof shutdown: a wedged USB handle can make an RX-loop join block
   * forever. Arm a force-exit safety net so the process is guaranteed to die;
   * if the clean joins finish first (the normal case) this timer is killed with
   * the process on return and never fires. */
  std::thread([]() {
    std::this_thread::sleep_for(std::chrono::seconds(4));
    std::_Exit(0);
  }).detach();
  snd.dev->StopRxLoop();
  bfe.dev->StopRxLoop();
  disp.join();
  snd_rx.join();
  bfe_thread.join();
  snd.dev->Stop();
  bfe.dev->Stop();
  return stalled ? 2 : 0;
}

/* ---------------------------------------------------------------- main ------ */
static uint16_t hex16(const char *s) {
  return (uint16_t)std::strtoul(s, nullptr, 0);
}

int main(int argc, char **argv) {
  auto logger = std::make_shared<Logger>();
  install_devourer_signal_handlers();

  int channel = 6;
  bool verbose = false;
  double sens_k = 4.0; /* CFAR threshold in sigmas; --sensitivity overrides */
  uint16_t snd_vid = 0x0bda, bfe_vid = 0x0bda;
  uint16_t snd_pid = 0x8812, bfe_pid = 0xc812;
  /* selector: "0xVID:0xPID" or just "0xPID" (VID defaults to 0x0bda). */
  auto parse_sel = [](const std::string &s, uint16_t &v, uint16_t &p) {
    auto c = s.find(':');
    if (c != std::string::npos) {
      v = hex16(s.substr(0, c).c_str());
      p = hex16(s.substr(c + 1).c_str());
    } else {
      p = hex16(s.c_str());
    }
  };
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
    if (a == "--channel") channel = std::atoi(next().c_str());
    else if (a == "--vid") { uint16_t v = hex16(next().c_str()); snd_vid = bfe_vid = v; }
    else if (a == "--sounder") parse_sel(next(), snd_vid, snd_pid);
    else if (a == "--beamformee") parse_sel(next(), bfe_vid, bfe_pid);
    else if (a == "--sensitivity") {
      std::string s = next();
      sens_k = s == "high" ? 2.5 : s == "low" ? 6.0 : 4.0; /* default med */
    }
    else if (a == "--verbose" || a == "-v") verbose = true;
    else if (a == "-h" || a == "--help") {
      std::printf(
          "sense — Wi-Fi motion sensing from beamforming reports\n"
          "  drives two adapters: a sounder + a beamformee, on one host.\n"
          "  --channel N                 (default 6)\n"
          "  --sensitivity low|med|high  detector threshold (default med)\n"
          "  --vid 0xNNNN                default VID for both (default 0x0bda)\n"
          "  --sounder [VID:]PID         sounder adapter selector\n"
          "  --beamformee [VID:]PID      beamformee adapter selector\n"
          "  -v, --verbose               show the library's bring-up logs\n");
      return 0;
    }
  }

  /* Quiet the library's per-operation info logging so the live display owns the
   * console; --verbose restores the full bring-up log. */
  if (!verbose)
    logger->set_level(Logger::Level::Warn);

  /* DEVOURER_SENSE_K overrides the detector threshold for on-rig fine-tuning. */
  if (const char *kenv = std::getenv("DEVOURER_SENSE_K"))
    sens_k = std::atof(kenv);

  Sensor sensor(sens_k);
  return run_active(snd_vid, snd_pid, bfe_vid, bfe_pid, channel, logger, sensor);
}
