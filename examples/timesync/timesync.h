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
#include "tsf_linfit.h"       // tsffit::Recon / LinFit

namespace timesync {

// Recon / LinFit live in examples/common/tsf_linfit.h (shared with the
// stream-timing TX fit); re-exported here so the fit sites read unchanged.
// Here x = the slave's local TSF (µs), y = the master's broadcast TSF (µs).
using tsffit::Recon;
using tsffit::LinFit;

// Uplink extends the downlink beacon with two more TD-tag class codes (reusing
// tdma::build_frame; parse_frame round-trips any class value, so tdma.h is
// untouched). Uplink = UE→master frame the master phase-measures; Ta = the
// master→UE timing-advance correction.
static constexpr uint8_t kClassUplink = 3;
static constexpr uint8_t kClassTa = 4;

// --- Config (env) -----------------------------------------------------------
enum class Role { Master, Slave, Ue };

struct Config {
  Role role = Role::Slave;
  int interval_ms = 100;   // master: sync-beacon period (LTE beacon ≈ 100 ms)
  int secs = 0;            // 0 = run until signalled
  uint8_t channel = 36;
  devourer::TxMode rate;   // master beacon rate (default 6M — must be heard)
  // Uplink timing-advance (full-duplex, DEVOURER_TSYNC_UPLINK=1):
  bool uplink = false;     // master: measure UE uplinks + feed back TA. ue: TX uplinks
  bool hwbeacon = false;  // master: HW-TBTT beacon (StartBeacon); slave: read 802.11 TS
  bool no_csma = true;    // master: disable EDCCA by DEFAULT (master owns the channel,
                          // beacon airs exactly at TBTT -> sub-µs); DEVOURER_TSYNC_CSMA=1 keeps CSMA
  int slot_ms = 20;        // uplink slot grid on the master TSF (a TDMA slot)
  double ta_gain = 0.3;    // master TA integrator gain (0..1; error fraction/step)
};

inline int env_int(const char* n, int dflt) {
  const char* e = std::getenv(n);
  return (e && *e) ? std::atoi(e) : dflt;
}

inline Config config_from_env() {
  Config c;
  if (const char* r = std::getenv("DEVOURER_TSYNC_ROLE")) {
    std::string s(r);
    c.role = (s == "master") ? Role::Master : (s == "ue") ? Role::Ue : Role::Slave;
  }
  c.interval_ms = env_int("DEVOURER_TSYNC_INTERVAL_MS", 100);
  c.secs = env_int("DEVOURER_TSYNC_SECS", 0);
  c.channel = static_cast<uint8_t>(env_int("DEVOURER_CHANNEL", 36));
  c.uplink = std::getenv("DEVOURER_TSYNC_UPLINK") != nullptr;
  c.hwbeacon = std::getenv("DEVOURER_TSYNC_HWBEACON") != nullptr;
  c.no_csma = std::getenv("DEVOURER_TSYNC_CSMA") == nullptr;  // on by default; opt out to keep CSMA
  c.slot_ms = env_int("DEVOURER_TSYNC_SLOT_MS", 20);
  if (const char* g = std::getenv("DEVOURER_TSYNC_TA_GAIN")) c.ta_gain = std::atof(g);
  const char* rt = std::getenv("DEVOURER_TSYNC_RATE");
  c.rate = devourer::parse_tx_mode_str(rt && *rt ? rt : "6M");
  return c;
}

}  // namespace timesync
