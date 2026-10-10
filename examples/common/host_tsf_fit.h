// HostTsfFit — predict the chip's TSF for any host instant without reading a
// register per frame.
//
// The stream-timing field (src/StreamTelemetry.h) wants the transmitter's TSF
// at every send_packet call, and the standing rule is that nothing reads a
// register on the send path (a control transfer is ~200-340 µs on USB and
// serialises against the bulk pipe). So a poller thread samples
// IRadio::ReadTsf() once per period against std::chrono::steady_clock, and a
// least-squares line maps host ns -> TSF µs; predict() is pure arithmetic.
// The residual is the host's read-latency jitter, tens of µs on USB
// (docs/timing-accuracy.md), which is far below the millisecond-scale
// latencies the field reports.
//
// A read that throws (USB failure, or the race with a heavy RX bulk-IN load
// the IRadio contract warns about) or returns 0 is dropped. A part whose
// ReadTsf is not ported returns 0 every time (the RTL8733B): after
// kGiveUpZeros consecutive zeros the fit declares itself unsupported and the
// caller marks its frames has_tsf=0. ready() follows LinFit's 16-sample floor:
// 1.6 s at the default 100 ms period, the timesync-master cadence.
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>

#include "IRadio.h"
#include "tsf_linfit.h"

class HostTsfFit {
 public:
  static constexpr int kGiveUpZeros = 5;

  explicit HostTsfFit(IRadio &dev, int period_ms = 100)
      : _dev(dev), _period_ms(period_ms) {}
  ~HostTsfFit() { stop(); }
  HostTsfFit(const HostTsfFit &) = delete;
  HostTsfFit &operator=(const HostTsfFit &) = delete;

  void start() {
    if (_thread.joinable()) return;
    _stop.store(false);
    _thread = std::thread([this] { run(); });
  }
  void stop() {
    _stop.store(true);
    if (_thread.joinable()) _thread.join();
  }

  static uint64_t host_ns() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
  }

  bool unsupported() const { return _unsupported.load(); }
  bool ready() const {
    std::lock_guard<std::mutex> lk(_mu);
    return _fit.ready();
  }
  // Predicted TSF (µs) at the given host instant; 0 before ready().
  uint64_t predict(uint64_t host_ns_at) const {
    std::lock_guard<std::mutex> lk(_mu);
    if (!_fit.ready()) return 0;
    const double tsf = _fit.at(static_cast<double>(host_ns_at) / 1000.0);
    return tsf < 0 ? 0 : static_cast<uint64_t>(tsf);
  }
  // Host-clock vs TSF rate in ppm (0.0 before ready()).
  double ppm() const {
    std::lock_guard<std::mutex> lk(_mu);
    return _fit.ready() ? _fit.ppm() : 0.0;
  }
  int samples() const {
    std::lock_guard<std::mutex> lk(_mu);
    return static_cast<int>(_fit.n);
  }
  // Residual of the latest sample against the fit built before it (µs); a
  // live quality figure for the stream.timing event.
  double last_resid_us() const { return _last_resid.load(); }
  // Times the fit started over because a sample landed more than
  // kDiscontinuityUs off the line: the chip's TSF was reset under us (the
  // Jaguar1 beacon arm pulses DUAL_TSF_RST; a re-init zeroes it).
  int resets() const { return _resets.load(); }
  static constexpr double kDiscontinuityUs = 50000.0;

 private:
  void run() {
    int zeros = 0;
    while (!_stop.load()) {
      const uint64_t t0 = host_ns();
      uint64_t tsf = 0;
      try {
        tsf = _dev.ReadTsf();
      } catch (...) {
        tsf = 0;
      }
      const uint64_t t1 = host_ns();
      if (tsf == 0) {
        if (++zeros >= kGiveUpZeros && samples() == 0) {
          _unsupported.store(true);
          return;
        }
      } else {
        zeros = 0;
        // The read's midpoint is the best host estimate of when the chip
        // latched the value; half the round trip is the irreducible error.
        const double x = static_cast<double>(t0 / 2 + t1 / 2) / 1000.0;
        std::lock_guard<std::mutex> lk(_mu);
        if (_fit.ready()) {
          const double r = static_cast<double>(tsf) - _fit.at(x);
          _last_resid.store(r);
          if (r > kDiscontinuityUs || r < -kDiscontinuityUs) {
            _fit = tsffit::LinFit{};
            _resets.fetch_add(1);
          }
        }
        _fit.add(x, static_cast<double>(tsf));
      }
      for (int i = 0; i < _period_ms && !_stop.load(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  IRadio &_dev;
  const int _period_ms;
  mutable std::mutex _mu;
  tsffit::LinFit _fit;
  std::atomic<bool> _stop{false};
  std::atomic<bool> _unsupported{false};
  std::atomic<double> _last_resid{0.0};
  std::atomic<int> _resets{0};
  std::thread _thread;
};
