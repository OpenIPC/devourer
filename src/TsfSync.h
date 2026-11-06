#ifndef DEVOURER_TSF_SYNC_H
#define DEVOURER_TSF_SYNC_H

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <utility>
#include <vector>

#include "RxPacket.h"

/* One-way hardware time distribution from beacon TSFs.
 *
 * A transmitter — any AP, or a devourer StartBeacon master — inserts its live
 * hardware TSF into every beacon at the instant of transmission: a genuine
 * sub-µs TX-egress timestamp (Packet::TxEgressTsf). A receiver latches its own
 * hardware TSF when the beacon arrives (RxPacket::RxAtrib::tsfl). Feeding both
 * into TsfSync fits the receiver's clock to the transmitter's:
 *
 *     local_tsf  ≈  skew · remote_tsf  +  offset
 *
 * Both endpoints are hardware-latched below the CSMA/queueing layer, so the fit
 * converges to the two crystals' relationship, not the medium's jitter — the
 * basis of the reference-broadcast (RBIS) one-way sync that is the only sub-µs
 * option on Wi-Fi silicon lacking a host-reported TX-egress timestamp. Once
 * Ready(), LocalForRemote()/RemoteForLocal() translate a timestamp between the
 * two clocks (e.g. to schedule a local action at a transmitter-defined instant).
 *
 * Least-squares in offset coordinates for numerical conditioning; the local
 * 32-bit tsfl is reconstructed to 64 bits across its ~71.6 min wrap. Header-only,
 * no chip dependency. Not internally synchronised — feed it from one thread
 * (typically the RX callback).
 *
 * Step detection (opt-in, SetStepDetect): the fit is cumulative, so one
 * jump of the transmitter's clock (a TSF write on its side) would otherwise
 * be absorbed as skew and bias every translation for a long time. With it
 * on, a beacon more than `threshold_us` off the line is held out of the fit;
 * `confirm` consecutive held beacons that agree on one new offset (within a
 * quarter of the threshold) are a STEP: the remote axis is re-anchored by it
 * (ShiftRemote — a TSF step is an offset change, the rate is the crystals')
 * and the held beacons join the fit. Held beacons that never agree are
 * dropped (a stray frame never bends the line). */
namespace devourer {

class TsfSync {
 public:
  /* Feed one beacon: the transmitter's 64-bit hardware egress TSF (µs, from
   * Packet::TxEgressTsf) and this receiver's 32-bit RX TSF low word (µs,
   * RxAtrib.tsfl). */
  void Add(uint64_t remote_tsf, uint32_t local_tsfl) {
    const int64_t local = ReconLocal(local_tsfl);
    const int64_t remote = static_cast<int64_t>(remote_tsf);
    if (_step_thr > 0 && _init) {
      const int64_t res = remote - RemoteForLocal(static_cast<uint64_t>(local));
      if (std::llabs(res) > _step_thr) {
        if (!_held.empty() && std::llabs(res - _held_res) > _step_thr / 4) {
          _rejected += static_cast<long long>(_held.size());
          _held.clear();
        }
        if (_held.empty())
          _held_res = res;
        _held.push_back({remote, local});
        if (static_cast<int>(_held.size()) < _step_confirm)
          return;
        ShiftAxis(_held_res);
        ++_steps;
        _last_step = _held_res;
        for (const auto &h : _held)
          Ingest(h.first, h.second);
        _held.clear();
        return;
      }
      _rejected += static_cast<long long>(_held.size());
      _held.clear();
    }
    Ingest(remote, local);
  }

  /* Step detection (see the header); threshold 0 = off (the default). */
  void SetStepDetect(int64_t threshold_us, int confirm) {
    _step_thr = threshold_us > 0 ? threshold_us : 0;
    _step_confirm = confirm > 1 ? confirm : 1;
  }
  long long Steps() const { return _steps; }      /* confirmed transmitter steps */
  int64_t LastStepUs() const { return _last_step; }
  long long Rejected() const { return _rejected; } /* held beacons never confirmed */

 private:
  void Ingest(int64_t remote, int64_t local) {
    if (!_init) {
      _x0 = static_cast<double>(remote);
      _y0 = static_cast<double>(local);
      _init = true;
    }
    const double xi = static_cast<double>(remote) - _x0;
    const double yi = static_cast<double>(local) - _y0;
    ++_n;
    _sx += xi;
    _sy += yi;
    _sxx += xi * xi;
    _sxy += xi * yi;
    _last_remote = remote;
    _last_local = local;
  }

 public:

  /* Convenience: feed a received Packet. Returns true iff it carried an egress
   * TSF (a beacon / probe response) and was ingested — so a caller can simply
   * `sync.Add(packet)` on every frame and let non-beacons fall through. */
  bool Add(const Packet &p) {
    const auto tx = p.TxEgressTsf();
    if (!tx) return false;
    Add(*tx, p.RxAtrib.tsfl);
    return true;
  }

  /* Enough samples for a stable line. */
  bool Ready() const { return _n >= 16; }
  long long Count() const { return _n; }

  /* Local clock rate relative to the transmitter, in ppm (positive = the local
   * crystal runs fast). ~0 until a few samples in. */
  double SkewPpm() const { return (Slope() - 1.0) * 1e6; }

  /* Fitted slope of local vs remote (≈ 1 + skew). */
  double Slope() const {
    const double den = static_cast<double>(_n) * _sxx - _sx * _sx;
    if (den == 0.0) return 1.0;
    return (static_cast<double>(_n) * _sxy - _sx * _sy) / den;
  }

  /* Translate a transmitter (remote) TSF to the equivalent local TSF (µs). */
  int64_t LocalForRemote(uint64_t remote_tsf) const {
    const double a = Slope();
    const double b = Intercept(a);
    return llround(_y0 + a * (static_cast<double>(static_cast<int64_t>(remote_tsf)) - _x0) + b);
  }

  /* Translate a local TSF to the equivalent transmitter (remote) TSF (µs). */
  int64_t RemoteForLocal(uint64_t local_tsf) const {
    const double a = Slope();
    if (a == 0.0) return static_cast<int64_t>(local_tsf);
    const double b = Intercept(a);
    return llround(_x0 + ((static_cast<double>(static_cast<int64_t>(local_tsf)) - _y0) - b) / a);
  }

  /* Re-express the fit against another transmitter whose clock is this
   * one's plus `delta_us` at every instant (same rate): the remote axis moves
   * by delta, the fit and its history stay, Ready() stays. What a UE does on
   * a handover between two cells held to one PTP timebase (M6,
   * src/cell/HoCommand.h ho_tsf_delta_us): it keeps its clock, no
   * re-acquisition, and the new transmitter's beacons extend the same line. */
  void ShiftRemote(int64_t delta_us) {
    ShiftAxis(delta_us);
    _held.clear(); /* beacons held from the previous transmitter */
  }

  /* Instantaneous (local − remote) offset at the most recent beacon, µs — the
   * raw crystal offset before the fit, useful as a coarse health readout. */
  int64_t OffsetUs() const { return _last_local - _last_remote; }

 private:
  void ShiftAxis(int64_t delta_us) {
    _x0 += static_cast<double>(delta_us);
    _last_remote += delta_us;
  }
  double Intercept(double a) const {
    return (_sy - a * _sx) / (_n ? static_cast<double>(_n) : 1.0);
  }
  int64_t ReconLocal(uint32_t lo) {
    if (_linit && lo < _plo) _hi += (1LL << 32);
    _plo = lo;
    _linit = true;
    return _hi + lo;
  }

  bool _init = false, _linit = false;
  double _x0 = 0, _y0 = 0, _sx = 0, _sy = 0, _sxx = 0, _sxy = 0;
  long long _n = 0;
  int64_t _hi = 0, _last_remote = 0, _last_local = 0;
  uint32_t _plo = 0;
  int64_t _step_thr = 0;
  int _step_confirm = 3;
  std::vector<std::pair<int64_t, int64_t>> _held; /* (remote, local) off the line */
  int64_t _held_res = 0;                           /* the first held beacon's offset */
  long long _steps = 0, _rejected = 0;
  int64_t _last_step = 0;
};

}  // namespace devourer

#endif /* DEVOURER_TSF_SYNC_H */
