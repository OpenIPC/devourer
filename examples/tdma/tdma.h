// Burst-level bandwidth-TDMA support: schedule math, the on-air frame tag, and
// the env-var config — self-contained (app-level, no WiFiDriver core changes),
// shared by the tx / rx-sync / rx-camp roles in main.cpp.
//
// The idea (see docs/narrowband.md, "Burst-level bandwidth TDMA"): a transmitter
// alternates BURSTS between a robust narrowband width (5/10 MHz — ~6 dB link
// budget, for critical frames) and a wide width (20/40 MHz — throughput, for
// bulk frames), flipping bandwidth with the cheap IRtlDevice::FastSetBandwidth.
// Narrowband is an ADC-clock-domain state, not a per-packet radiotap field, and
// a receiver decodes exactly one width at a time — so the scheme is inherently
// burst-level and the hard part is schedule synchronization.
#pragma once

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "RadiotapBuilder.h"  // devourer::build_stream_radiotap / parse_tx_mode_str
#include "SelectedChannel.h"  // ChannelWidth_t
#include "TxMode.h"

namespace tdma {

// --- Frame class ------------------------------------------------------------
// Marker frames are emitted at each narrowband-burst start; the marker-sync RX
// aligns its local schedule to them. Critical rides the narrowband burst, Bulk
// the wide burst.
enum class Class : uint8_t { Marker = 0, Critical = 1, Bulk = 2 };

inline const char* class_name(Class c) {
  switch (c) {
    case Class::Marker:   return "marker";
    case Class::Critical: return "critical";
    case Class::Bulk:     return "bulk";
  }
  return "?";
}

// --- On-air frame -----------------------------------------------------------
// The canonical devourer beacon layout (SA 57:42:75:05:d6:00, so existing SA
// matchers/tests recognise it) followed by a "TD" tag. Offsets below are into
// the RX-side Packet.Data (the 802.11 MPDU, radiotap already stripped):
//   [0..1]  FC (0x40 probe-req)   [4..9] addr1 (broadcast)
//   [10..15] addr2 = SA           [24..] the TD tag
static const uint8_t kSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
static constexpr size_t kHdrLen = 24;     // 802.11 header up to the body
// 'T''D' ver cls seq[4] burst[4] tx_tsf[8]. The 8-byte TX-TSF stamp lets the
// TSF-sync RX measure the TX↔RX crystal drift (0 when the TX can't read TSF).
static constexpr size_t kTagLen = 20;
static constexpr size_t kMinData = kHdrLen + kTagLen;

// Build a full TX buffer: [radiotap for this class][802.11 header][TD tag].
inline std::vector<uint8_t> build_frame(const std::vector<uint8_t>& radiotap,
                                        Class cls, uint32_t seq, uint32_t burst,
                                        uint64_t tx_tsf = 0) {
  static const uint8_t hdr[kHdrLen] = {
      0x40, 0x00, 0x00, 0x00,                         // FC + duration
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff,             // addr1 broadcast
      0x57, 0x42, 0x75, 0x05, 0xd6, 0x00,             // addr2 = SA
      0x57, 0x42, 0x75, 0x05, 0xd6, 0x00,             // addr3
      0x80, 0x00};                                    // seq ctl
  std::vector<uint8_t> f;
  f.reserve(radiotap.size() + kHdrLen + kTagLen);
  f.insert(f.end(), radiotap.begin(), radiotap.end());
  f.insert(f.end(), hdr, hdr + kHdrLen);
  uint8_t tag[kTagLen] = {
      'T', 'D', 1, static_cast<uint8_t>(cls),
      static_cast<uint8_t>(seq),        static_cast<uint8_t>(seq >> 8),
      static_cast<uint8_t>(seq >> 16),  static_cast<uint8_t>(seq >> 24),
      static_cast<uint8_t>(burst),      static_cast<uint8_t>(burst >> 8),
      static_cast<uint8_t>(burst >> 16),static_cast<uint8_t>(burst >> 24)};
  for (int i = 0; i < 8; ++i) tag[12 + i] = static_cast<uint8_t>(tx_tsf >> (8 * i));
  f.insert(f.end(), tag, tag + kTagLen);
  return f;
}

struct Parsed {
  bool ok = false;
  Class cls = Class::Bulk;
  uint32_t seq = 0;
  uint32_t burst = 0;
  uint64_t tx_tsf = 0;
};

// Parse an RX Packet.Data span (802.11 MPDU) into a TD tag, if it is one of ours.
inline Parsed parse_frame(const uint8_t* data, size_t len) {
  Parsed p;
  if (len < kMinData) return p;
  if (std::memcmp(data + 10, kSa, 6) != 0) return p;    // not our SA
  const uint8_t* t = data + kHdrLen;
  if (t[0] != 'T' || t[1] != 'D') return p;
  p.cls = static_cast<Class>(t[3]);
  p.seq = static_cast<uint32_t>(t[4]) | (static_cast<uint32_t>(t[5]) << 8) |
          (static_cast<uint32_t>(t[6]) << 16) | (static_cast<uint32_t>(t[7]) << 24);
  p.burst = static_cast<uint32_t>(t[8]) | (static_cast<uint32_t>(t[9]) << 8) |
            (static_cast<uint32_t>(t[10]) << 16) | (static_cast<uint32_t>(t[11]) << 24);
  for (int i = 0; i < 8; ++i)
    p.tx_tsf |= static_cast<uint64_t>(t[12 + i]) << (8 * i);
  p.ok = true;
  return p;
}

// --- TSF clock: a running host↔hardware-TSF least-squares fit ---------------
// Every RX frame carries a hardware TSF (rx_pkt_attrib::tsfl, latched in the MAC
// at receive) and a host time (when our callback ran). The host time is noisy
// (USB batching + scheduling, ~1 ms RMS on Jaguar1); the TSF is not. Fitting
// host = a·tsf + b over many frames averages the noise out, so evaluating the
// line at a marker's tsf gives a de-jittered host time for that marker — the
// precise schedule anchor. All state in offset coords (relative to the first
// sample) to keep the sums numerically well-conditioned.
struct TsfClock {
  bool init = false;
  double x0 = 0, y0 = 0;
  long long n = 0;
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  int64_t hi = 0;            // 32→64-bit tsf reconstruction (low word wraps)
  uint32_t plo = 0;

  int64_t recon(uint32_t lo) {
    if (init && lo < plo) hi += (1LL << 32);
    plo = lo;
    return hi + lo;
  }
  // Feed one frame; returns the reconstructed 64-bit tsf (µs).
  int64_t add(uint32_t tsfl, int64_t host_ns) {
    int64_t t = recon(tsfl);
    if (!init) { x0 = (double)t; y0 = (double)host_ns; init = true; }
    double xi = (double)t - x0, yi = (double)host_ns - y0;
    ++n; sx += xi; sy += yi; sxx += xi * xi; sxy += xi * yi;
    return t;
  }
  bool ready() const { return n >= 16; }
  // The de-jittered host time (ns) for a given reconstructed tsf.
  double host_at(int64_t t) const {
    double den = (double)n * sxx - sx * sx;
    if (den == 0) return y0;
    double a = ((double)n * sxy - sx * sy) / den;
    double b = (sy - a * sx) / (double)n;
    return y0 + a * ((double)t - x0) + b;
  }
};

// --- Schedule ---------------------------------------------------------------
enum class Phase { NB, WIDE };

inline int64_t wall_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

struct Schedule {
  int nb_ms = 100;
  int wide_ms = 100;
  ChannelWidth_t nb_w = CHANNEL_WIDTH_10;
  ChannelWidth_t wide_w = CHANNEL_WIDTH_20;
  int64_t epoch_ms = 0;  // shared wall-clock anchor (both ends must match)

  int period() const { return nb_ms + wide_ms; }

  // Phase active at wall-clock instant `t_ms`, plus the burst (period) index.
  struct At { Phase phase; int64_t burst; int ms_into_phase; };
  At at(int64_t t_ms) const {
    int64_t rel = t_ms - epoch_ms;
    int64_t burst = rel >= 0 ? rel / period() : (rel - period() + 1) / period();
    int pos = static_cast<int>(((rel % period()) + period()) % period());
    if (pos < nb_ms) return {Phase::NB, burst, pos};
    return {Phase::WIDE, burst, pos - nb_ms};
  }
  ChannelWidth_t width(Phase p) const { return p == Phase::NB ? nb_w : wide_w; }
};

// --- Config (env) -----------------------------------------------------------
enum class Role { Tx, RxSync, RxCamp };
enum class Sync { WallClock, Marker, Tsf };

struct Config {
  Role role = Role::Tx;
  Sync sync = Sync::WallClock;
  Schedule sched;
  ChannelWidth_t camp_w = CHANNEL_WIDTH_10;  // rx-camp fixed width
  int guard_ms = 20;                         // rx-sync: switch this early
  int gap_us = 500;                          // tx: intra-burst inter-frame gap
  devourer::TxMode crit_rate;                // default 6M
  devourer::TxMode bulk_rate;                // default MCS7
  uint8_t channel = 36;
};

inline ChannelWidth_t width_of(int mhz) {
  switch (mhz) {
    case 5:  return CHANNEL_WIDTH_5;
    case 10: return CHANNEL_WIDTH_10;
    case 40: return CHANNEL_WIDTH_40;
    default: return CHANNEL_WIDTH_20;
  }
}
inline int mhz_of(ChannelWidth_t w) {
  switch (w) {
    case CHANNEL_WIDTH_5:  return 5;
    case CHANNEL_WIDTH_10: return 10;
    case CHANNEL_WIDTH_40: return 40;
    default:               return 20;
  }
}

inline int env_int(const char* n, int dflt) {
  const char* e = std::getenv(n);
  return (e && *e) ? std::atoi(e) : dflt;
}

inline Config config_from_env() {
  Config c;
  if (const char* r = std::getenv("DEVOURER_TDMA_ROLE")) {
    std::string s(r);
    if (s == "rx-sync") c.role = Role::RxSync;
    else if (s == "rx-camp") c.role = Role::RxCamp;
    else c.role = Role::Tx;
  }
  if (const char* s = std::getenv("DEVOURER_TDMA_SYNC")) {
    std::string v(s);
    c.sync = v == "marker" ? Sync::Marker
             : v == "tsf"  ? Sync::Tsf
                           : Sync::WallClock;
  }

  c.sched.nb_w = width_of(env_int("DEVOURER_TDMA_NB", 10));
  c.sched.wide_w = width_of(env_int("DEVOURER_TDMA_WIDE", 20));
  c.sched.nb_ms = env_int("DEVOURER_TDMA_NB_MS", 100);
  c.sched.wide_ms = env_int("DEVOURER_TDMA_WIDE_MS", 100);
  c.sched.epoch_ms = env_int("DEVOURER_TDMA_EPOCH_MS", 0);
  c.camp_w = width_of(env_int("DEVOURER_TDMA_CAMP", 10));
  c.guard_ms = env_int("DEVOURER_TDMA_GUARD_MS", 20);
  c.gap_us = env_int("DEVOURER_TDMA_GAP_US", 500);
  c.channel = static_cast<uint8_t>(env_int("DEVOURER_CHANNEL", 36));

  const char* cr = std::getenv("DEVOURER_TDMA_CRIT_RATE");
  const char* br = std::getenv("DEVOURER_TDMA_BULK_RATE");
  c.crit_rate = devourer::parse_tx_mode_str(cr && *cr ? cr : "6M");
  c.bulk_rate = devourer::parse_tx_mode_str(br && *br ? br : "MCS7");
  return c;
}

}  // namespace tdma
