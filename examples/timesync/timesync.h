// timesync — LTE-eNB-style over-the-air time distribution: one MASTER broadcasts
// its hardware TSF periodically (a "sync beacon"), and any number of SLAVES lock
// their notion of the master clock to it from the beacons alone — no GPS at the
// slaves, only the master holds a reference. This is the 802.11 analog of an eNB
// distributing frame timing to UEs: the master's TSF is the SFN, each slave a UE
// slaving to it.
//
// A slave relates the master's broadcast TSF to its OWN per-frame hardware TSF
// (rx_pkt_attrib::tsfl) with a running least-squares fit — both are clean
// MAC-latched microsecond clocks, so the fit residual is the true lock quality
// (the ~sub-µs floor of the dual-RX TSF correlation), NOT the ~1 ms host-callback
// jitter. Each slave PREDICTS the next beacon's master TSF from its fit; the
// prediction error is how tightly it tracks the eNB. Two slaves predicting the
// SAME beacon (matched by seq) agree to within their combined residual — the
// inter-UE sync error, measured without either slave reading a host clock.
//
// App-level only (no WiFiDriver core changes); reuses the TD frame tag + SA from
// the tdma example (examples/tdma/tdma.h) so one marker layout serves both.
#pragma once

#include <cstdint>
#include <cstdlib>
#include <string>

#include "RadiotapBuilder.h"  // devourer::TxMode / parse_tx_mode_str
#include "tdma.h"             // tdma::build_frame / parse_frame / kSa / Class

namespace timesync {

// 32→64-bit TSF reconstruction (the MAC latches only the low 32 bits per frame;
// it wraps every ~71 min). One per clock source.
struct Recon {
  int64_t hi = 0;
  uint32_t plo = 0;
  bool init = false;
  int64_t operator()(uint32_t lo) {
    if (init && lo < plo) hi += (1LL << 32);
    plo = lo;
    init = true;
    return hi + lo;
  }
};

// Incremental ordinary-least-squares fit y = a·x + b, offset-normalized to the
// first sample so the sums stay well within double precision over a long run.
// Here x = the slave's local TSF (µs), y = the master's broadcast TSF (µs).
struct LinFit {
  bool init = false;
  double x0 = 0, y0 = 0;
  long long n = 0;
  double sx = 0, sy = 0, sxx = 0, sxy = 0;

  void add(double x, double y) {
    if (!init) { x0 = x; y0 = y; init = true; }
    double xi = x - x0, yi = y - y0;
    ++n; sx += xi; sy += yi; sxx += xi * xi; sxy += xi * yi;
  }
  bool ready() const { return n >= 16; }
  double slope() const {
    double den = (double)n * sxx - sx * sx;
    return den == 0 ? 1.0 : ((double)n * sxy - sx * sy) / den;
  }
  // Predicted master TSF (µs) at local TSF x (µs).
  double at(double x) const {
    double a = slope();
    double b = (sy - a * sx) / (double)n;
    return y0 + a * (x - x0) + b;
  }
  // Master-vs-slave crystal offset in ppm (slope = dmaster/dlocal).
  double ppm() const { return (slope() - 1.0) * 1e6; }
};

// --- Config (env) -----------------------------------------------------------
enum class Role { Master, Slave };

struct Config {
  Role role = Role::Slave;
  int interval_ms = 100;   // master: sync-beacon period (LTE beacon ≈ 100 ms)
  int secs = 0;            // 0 = run until signalled
  uint8_t channel = 36;
  devourer::TxMode rate;   // master beacon rate (default 6M — must be heard)
};

inline int env_int(const char* n, int dflt) {
  const char* e = std::getenv(n);
  return (e && *e) ? std::atoi(e) : dflt;
}

inline Config config_from_env() {
  Config c;
  if (const char* r = std::getenv("DEVOURER_TSYNC_ROLE")) {
    std::string s(r);
    c.role = (s == "master") ? Role::Master : Role::Slave;
  }
  c.interval_ms = env_int("DEVOURER_TSYNC_INTERVAL_MS", 100);
  c.secs = env_int("DEVOURER_TSYNC_SECS", 0);
  c.channel = static_cast<uint8_t>(env_int("DEVOURER_CHANNEL", 36));
  const char* rt = std::getenv("DEVOURER_TSYNC_RATE");
  c.rate = devourer::parse_tx_mode_str(rt && *rt ? rt : "6M");
  return c;
}

}  // namespace timesync
