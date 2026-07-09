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
static constexpr size_t kTagLen = 12;     // 'T''D' ver cls seq[4] burst[4]
static constexpr size_t kMinData = kHdrLen + kTagLen;

// Build a full TX buffer: [radiotap for this class][802.11 header][TD tag].
inline std::vector<uint8_t> build_frame(const std::vector<uint8_t>& radiotap,
                                        Class cls, uint32_t seq,
                                        uint32_t burst) {
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
  const uint8_t tag[kTagLen] = {
      'T', 'D', 1, static_cast<uint8_t>(cls),
      static_cast<uint8_t>(seq),        static_cast<uint8_t>(seq >> 8),
      static_cast<uint8_t>(seq >> 16),  static_cast<uint8_t>(seq >> 24),
      static_cast<uint8_t>(burst),      static_cast<uint8_t>(burst >> 8),
      static_cast<uint8_t>(burst >> 16),static_cast<uint8_t>(burst >> 24)};
  f.insert(f.end(), tag, tag + kTagLen);
  return f;
}

struct Parsed {
  bool ok = false;
  Class cls = Class::Bulk;
  uint32_t seq = 0;
  uint32_t burst = 0;
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
  p.ok = true;
  return p;
}

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
enum class Sync { WallClock, Marker };

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
  if (const char* s = std::getenv("DEVOURER_TDMA_SYNC"))
    c.sync = std::string(s) == "marker" ? Sync::Marker : Sync::WallClock;

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
