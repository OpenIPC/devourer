// TID -> TxMode unequal-error-protection (UEP) shim for SVC-T video over
// devourer. Maps each HEVC NAL's temporal_id (and IRAP/parameter-set
// criticality) to a PHY TxMode, so the most important layers fly at the most
// robust rate and the enhancement layers ride the fast rate — a graceful
// degradation staircase instead of a single MCS cliff.
//
// App-level (no WiFiDriver core changes): consumes devourer::TxMode +
// build_stream_radiotap + RtlJaguarDevice::send_packet.
#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include "RadiotapBuilder.h"  // parse_tx_mode_str
#include "TxMode.h"

namespace svc {

// Per-temporal-layer PHY policy. by_tid[i] = mode for temporal layer i; TIDs
// past the end clamp to the last (least-protected) entry. `critical` overrides
// for frames you cannot lose: IRAP/IDR slices and parameter sets.
struct LayerPolicy {
  std::vector<devourer::TxMode> by_tid;
  devourer::TxMode critical;

  const devourer::TxMode& mode_for(uint8_t tid, bool is_critical) const {
    if (is_critical || by_tid.empty()) return critical;
    return by_tid[tid < by_tid.size() ? tid : by_tid.size() - 1];
  }
};

inline devourer::TxMode HT(uint8_t mcs, uint8_t bw = 20, bool ldpc = false,
                           bool stbc = false, bool sgi = false) {
  devourer::TxMode m;
  m.mode = devourer::TxMode::Mode::HT;
  m.ht_mcs = mcs;
  m.bw_mhz = bw;
  m.ldpc = ldpc;
  m.stbc = stbc;
  m.sgi = sgi;
  return m;
}

// Default 3-temporal-layer ladder for an 8812AU long-range link.
//   critical (IDR / VPS/SPS/PPS) : MCS0 20MHz LDPC STBC  — max range
//   T0 base                      : MCS1 20MHz LDPC STBC
//   T1 (->30fps)                 : MCS4 20MHz
//   T2 (->60fps)                 : MCS7 40MHz SGI        — max throughput
inline LayerPolicy default_policy() {
  LayerPolicy p;
  p.critical = HT(/*mcs*/ 0, /*bw*/ 20, /*ldpc*/ true, /*stbc*/ true);
  p.by_tid = {
      HT(1, 20, /*ldpc*/ true, /*stbc*/ true),
      HT(4, 20),
      HT(7, 40, /*ldpc*/ false, /*stbc*/ false, /*sgi*/ true),
  };
  return p;
}

// HEVC NAL header is 2 bytes:
//   nal_unit_type = (b0 >> 1) & 0x3F
//   nuh_temporal_id_plus1 = b1 & 0x07   (TID = that - 1)
// Critical = IRAP slices (16..23, incl. IDR/CRA/BLA) or parameter sets
// (VPS=32 / SPS=33 / PPS=34) — losing any of these stalls the decoder.
struct NalInfo {
  uint8_t tid = 0;
  bool critical = false;
  uint8_t type = 0;
};

inline NalInfo parse_hevc_nal(const uint8_t* nal, size_t len) {
  NalInfo n;
  if (len < 2) return n;  // malformed -> treat as base layer
  n.type = (nal[0] >> 1) & 0x3F;
  int tid = static_cast<int>(nal[1] & 0x07) - 1;
  n.tid = tid < 0 ? 0 : static_cast<uint8_t>(tid);
  n.critical = (n.type >= 16 && n.type <= 23) || (n.type >= 32 && n.type <= 34);
  return n;
}

// Build a LayerPolicy from DEVOURER_SVC_LADDER, a ';'-separated list of
// "<key>=<spec>" where <key> is CRIT or T0/T1/T2/... and <spec> is a
// DEVOURER_TX_RATE token string (e.g. "MCS0/20/LDPC/STBC"). Tn rungs are
// ordered by index into by_tid. Unset -> default_policy().
//   DEVOURER_SVC_LADDER="CRIT=MCS0/20/LDPC/STBC;T0=MCS1/20/LDPC/STBC;T1=MCS4;T2=MCS7/40/SGI"
inline LayerPolicy policy_from_env() {
  const char* raw = std::getenv("DEVOURER_SVC_LADDER");
  if (raw == nullptr || *raw == '\0') return default_policy();

  LayerPolicy p;
  p.critical = HT(0, 20, /*ldpc*/ true, /*stbc*/ true);  // default if CRIT omitted
  std::vector<std::pair<int, devourer::TxMode>> tids;
  const std::string s(raw);
  size_t i = 0;
  while (i < s.size()) {
    size_t sc = s.find(';', i);
    std::string tok = s.substr(i, sc == std::string::npos ? std::string::npos : sc - i);
    i = (sc == std::string::npos) ? s.size() : sc + 1;
    size_t eq = tok.find('=');
    if (eq == std::string::npos) continue;
    std::string key = tok.substr(0, eq), val = tok.substr(eq + 1);
    devourer::TxMode m = devourer::parse_tx_mode_str(val);
    if (key == "CRIT" || key == "crit") {
      p.critical = m;
    } else if (key.size() >= 2 && (key[0] == 'T' || key[0] == 't')) {
      tids.emplace_back(std::atoi(key.c_str() + 1), m);
    }
  }
  std::sort(tids.begin(), tids.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });
  for (const auto& tm : tids) p.by_tid.push_back(tm.second);
  if (p.by_tid.empty()) return default_policy();
  return p;
}

}  // namespace svc
