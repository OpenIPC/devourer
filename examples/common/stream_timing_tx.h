// StreamTimingTx — the TX side of the per-frame timing telemetry
// (src/StreamTelemetry.h), shared by streamtx, svctx and duplex so the three
// demos stamp identically.
//
//   DEVOURER_STREAM_TIMING=N   air a timing marker every N data frames and run
//                              the host<->TSF fit (one ReadTsf per 100 ms, off
//                              the send path). Unset or 0: no marker, no fit —
//                              the addr3 field still carries depth and
//                              capture->send on every frame, at zero cost.
//
// Per frame the demo calls stamp() right before send_packet (fills the six
// addr3 bytes) and sent() right after it; maybe_marker() before each data
// frame airs the marker when due. On a part whose MAC does not stamp an
// injected probe response (AdapterCaps::hw_injected_mgmt_txtsf == false,
// Jaguar1) and a fixed channel, start() arms the hardware TBTT beacon with the
// same SA so the receiver still gets hardware egress pairs; the marker's flags
// tell the receiver which frames to trust. A hopping session on such a part
// gets no absolute latency and says so.
#pragma once

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>

#include "AdapterCaps.h"
#include "Event.h"
#include "IRadio.h"
#include "StreamTelemetry.h"
#include "TxStats.h"
#include "host_tsf_fit.h"
#include "logger.h"

class StreamTimingTx {
 public:
  using FrameTiming = devourer::stream_timing::FrameTiming;
  using TimingMarker = devourer::stream_timing::TimingMarker;

  StreamTimingTx(IRadio &dev, devourer::EventSink &ev, Logger &log,
                 const uint8_t sa[6], bool hopping)
      : _dev(dev), _ev(ev), _log(log), _hopping(hopping) {
    std::memcpy(_sa, sa, 6);
  }
  ~StreamTimingTx() { stop(); }

  static uint64_t now_ns() { return HostTsfFit::host_ns(); }

  // Reads DEVOURER_STREAM_TIMING. The fit thread and the beacon fallback are
  // armed on the first record (ensure_started), not here: the duplex demo
  // spawns its TX thread before the chip is brought up, and a ReadTsf or a
  // StartBeacon landing inside bring-up is not something to find out about
  // on air. By the first record the chip is up on every demo.
  void start() {
    if (const char *e = std::getenv("DEVOURER_STREAM_TIMING")) {
      _marker_every = std::atol(e);
      if (_marker_every < 0) _marker_every = 0;
    }
  }
  void ensure_started() {
    if (_started) return;
    _started = true;
    const auto caps = _dev.GetAdapterCaps();
    _tx_async = caps.generation == devourer::ChipGeneration::Jaguar1;
    _presp_stamped = caps.hw_injected_mgmt_txtsf;
    if (_marker_every == 0) return;
    /* The beacon first: arming it pulses the TSF on Jaguar1, and the fit
     * must not sample across that (it would restart anyway). */
    if (!_presp_stamped && caps.hw_beacon_txtsf && !_hopping) {
      std::vector<uint8_t> bcn;
      devourer::stream_timing::append_beacon_mpdu(bcn, _sa, _channel);
      _beacon = _dev.StartBeacon(bcn.data(), bcn.size(), 100);
      _log.info("stream timing: injected frames are not egress-stamped on this "
                "part — hardware beacon {} as the clock carrier",
                _beacon ? "armed" : "REFUSED (no absolute latency)");
    } else if (!_presp_stamped) {
      _log.warn("stream timing: no egress-stamped frame on this part while "
                "hopping — the receiver gets stage durations, not absolute latency");
    }
    _fit = std::make_unique<HostTsfFit>(_dev);
    _fit->start();
    _log.info("stream timing: marker every {} frames, fit {}", _marker_every,
              _fit->unsupported() ? "unsupported" : "running");
  }
  void stop() {
    if (_beacon) {
      _dev.StopBeacon();
      _beacon = false;
    }
    if (_fit) _fit->stop();
  }

  // The marker and the beacon name the channel in their DS parameter set.
  void set_channel(uint8_t ch) { _channel = ch; }

  // Right before send_packet. `read_ns` = when the record came off stdin,
  // `capture_ns` = the producer's stamp (has_capture) or read_ns. Fills the six
  // addr3 bytes; remembers the instants for sent().
  void stamp(uint8_t *addr3, uint64_t read_ns, uint64_t capture_ns, bool has_capture) {
    ensure_started();
    _t0 = now_ns();
    _read_ns = read_ns;
    _capture_ns = capture_ns;
    _has_capture = has_capture;
    const auto st = _dev.GetTxStats();
    _depth = st.inflight;
    FrameTiming t;
    t.has_capture = has_capture;
    t.has_depth = true;
    t.tx_async = _tx_async;
    t.depth = static_cast<uint8_t>(_depth > 255 ? 255 : _depth);
    const uint64_t c2s = _t0 > capture_ns ? (_t0 - capture_ns) / 1000 : 0;
    t.c2s10 = devourer::stream_timing::clip10(c2s);
    if (_fit && _fit->ready()) {
      const uint64_t tsf = _fit->predict(_t0);
      t.has_tsf = tsf != 0;
      t.tsf10_lo = static_cast<uint16_t>((tsf / 10) & 0xffff);
    }
    t.encode(addr3);
  }
  // Right after send_packet returned.
  void sent() {
    const uint64_t t1 = now_ns();
    const uint64_t tq = _t0 > _read_ns ? (_t0 - _read_ns) / 1000 : 0;
    const uint64_t tw = (t1 - _t0) / 1000;
    const uint64_t c2s = _t0 > _capture_ns ? (_t0 - _capture_ns) / 1000 : 0;
    _window.add(tq, tw, c2s, _has_capture, _depth);
    ++_frames;
  }

  // Before each data frame: air the marker when due. `radiotap` is the stream
  // radiotap header. Returns true when a marker was sent.
  bool maybe_marker(const std::vector<uint8_t> &radiotap) {
    if (_marker_every == 0 || (_frames % static_cast<uint64_t>(_marker_every)) != 0 ||
        _frames == _last_marker_frame)
      return false;
    _last_marker_frame = _frames;
    TimingMarker m;
    if (_presp_stamped) m.flags |= devourer::stream_timing::kMkPrespStamped;
    if (_beacon) m.flags |= devourer::stream_timing::kMkBeaconRunning;
    if (_tx_async) m.flags |= devourer::stream_timing::kMkTxAsync;
    if (_capture_seen) m.flags |= devourer::stream_timing::kMkCaptureSource;
    const uint64_t hn = now_ns();
    m.host_ns = hn;
    if (_fit && _fit->ready()) {
      m.flags |= devourer::stream_timing::kMkFitReady;
      m.tsf_pred_us = _fit->predict(hn);
      m.fit_ppm_x100 = static_cast<int32_t>(_fit->ppm() * 100.0);
    }
    m.fit_n = static_cast<uint16_t>(_fit ? (_fit->samples() > 0xffff ? 0xffff : _fit->samples()) : 0);
    _window.drain_into(m);
    _buf.clear();
    _buf.insert(_buf.end(), radiotap.begin(), radiotap.end());
    devourer::stream_timing::append_marker_mpdu(_buf, _sa, _channel, m);
    const bool ok = _dev.send_packet(_buf.data(), _buf.size());
    devourer::Ev(_ev, "stream.timing")
        .f("ok", ok ? 1 : 0)
        .f("frames", m.frames)
        .f("tq_p50_us", m.t_queue_p50_10 * 10)
        .f("tq_max_us", m.t_queue_max_10 * 10)
        .f("tw_p50_us", m.t_write_p50_10 * 10)
        .f("tw_max_us", m.t_write_max_10 * 10)
        .f("c2s_p50_us", m.c2s_p50_10 * 10)
        .f("c2s_max_us", m.c2s_max_10 * 10)
        .f("depth_max", m.depth_max)
        .f("captured", m.captured)
        .f("tsf_pred", (unsigned long long)m.tsf_pred_us)
        .f("fit_ppm", _fit && _fit->ready() ? _fit->ppm() : 0.0)
        .f("fit_n", m.fit_n)
        .f("fit_resid_us", _fit ? _fit->last_resid_us() : 0.0)
        .f("fit_resets", _fit ? _fit->resets() : 0)
        .f("presp_stamped", _presp_stamped ? 1 : 0)
        .f("beacon", _beacon ? 1 : 0)
        .f("tx_async", _tx_async ? 1 : 0);
    return true;
  }

  // A producer capture stamp arrived (kCtlCaptureTs): remembered for the next
  // data record. A second stamp before a record replaces the first (counted).
  void capture_stamp(uint64_t ns) {
    if (_pending_capture) ++_capture_dropped;
    _pending_capture = true;
    _pending_capture_ns = ns;
    _capture_seen = true;
  }
  // Take the pending stamp for the record read at `read_ns` (or fall back to it).
  uint64_t take_capture(uint64_t read_ns, bool &has_capture) {
    ensure_started();
    has_capture = _pending_capture;
    _pending_capture = false;
    return has_capture ? _pending_capture_ns : read_ns;
  }
  uint64_t capture_dropped() const { return _capture_dropped; }

 private:
  IRadio &_dev;
  devourer::EventSink &_ev;
  Logger &_log;
  const bool _hopping;
  uint8_t _sa[6];
  uint8_t _channel = 0;
  long _marker_every = 0;
  bool _started = false;
  bool _tx_async = false, _presp_stamped = false, _beacon = false;
  std::unique_ptr<HostTsfFit> _fit;
  devourer::stream_timing::TimingWindow _window;
  std::vector<uint8_t> _buf;
  uint64_t _frames = 0, _last_marker_frame = UINT64_MAX;
  uint64_t _t0 = 0, _read_ns = 0, _capture_ns = 0;
  bool _has_capture = false;
  uint32_t _depth = 0;
  bool _pending_capture = false, _capture_seen = false;
  uint64_t _pending_capture_ns = 0, _capture_dropped = 0;
};
