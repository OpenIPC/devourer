/* F1.h — M5 (#259) the DU <-> CU control link, and the DU's side of its
 * failure model.
 *
 * Datagrams (UDP over the LAN), little-endian:
 *   'F' '1' ver(1)=1 type(1) body tag(8)
 * The tag is the first 8 bytes of the injected Mac (HMAC-SHA1 keyed by the
 * F1 key, as src/cell/Rach.h) over 'F' + everything before it. An
 * unauthenticated sender (no Mac) writes zeros; a keyed receiver then rejects
 * the message, an unkeyed one ignores the tag (a development network).
 *
 *   Report   DU -> CU: what the CU plans from — the DU's tier, sync state
 *            and mode, channel, TBTT phase, the network superframe it is in,
 *            a freshness sequence,
 *            the schedule versions it has applied / staged, its grid (fixed
 *            items + capacity positions, relative to its TBTT), its UEs and
 *            their positions and filtered uplink measurements (RSSI/SNR),
 *            and the OTHER cells' UEs its radio hears (filtered RSSI/SNR).
 *   Schedule CU -> DU: version, the network superframe it takes effect in,
 *            a freshness sequence, and the position of every UE. The CU
 *            resends the current schedule (with a new sequence) as its
 *            keepalive.
 *   Ack      DU -> CU: the outcome for one schedule version.
 *
 * The network superframe is floor((PTP time - phase) / period): the same
 * count at every DU of the fabric, so `effective_net_sf` names one instant
 * network-wide.
 *
 * Failure model (F1Stager, the DU side):
 *   - versions only move forward: an older version than one applied or
 *     staged is Stale; the same version again is a keepalive and is answered
 *     with its current status;
 *   - a schedule must arrive `min_lead_sf` superframes before it takes
 *     effect, else Late. The DU publishes it in its grant map
 *     `publish_lead_sf` before that instant (the M1 N+2 rule), so a late
 *     update never alters a superframe already in progress;
 *   - a schedule the DU cannot carry (unknown UE, position out of range, one
 *     position twice) is Invalid and changes nothing;
 *   - a staged schedule whose publish does not succeed in time is abandoned
 *     (Late) rather than applied off its instant;
 *   - no FRESH authenticated CU message (a sequence above every one seen)
 *     for `cu_timeout_ms` = the CU is gone: the DU falls back to autonomous
 *     (the M4 mode) until it hears it again. A replay changes nothing and
 *     is answered Stale. (A CU restart keeps its sequence moving forward:
 *     it is the CU's realtime clock.)
 *
 * Pure header; cell_m5 ctest covers encode/decode, tags and the stager. */
#ifndef DEVOURER_CELL_F1_H
#define DEVOURER_CELL_F1_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <array>
#include <cstdint>
#include <cstring>
#include <functional>
#include <optional>
#include <vector>

#include "cell/Coord.h"
#include "cell/HoCommand.h"
#include "cell/Keys.h"
#include "cell/Handover.h"
#include "cell/MeasGap.h"
#include "cell/Rach.h"

namespace devourer::cell::f1 {

/* 2 (M6): Report carries the DU's filtered measurements of its own UEs, and
 * the SNR of the other cells' UEs it hears. 3: every measurement carries its
 * sample age, so the CU can tell when it expires. */
/* 5: Uplink (DAPS: a DU forwarding a UE uplink it heard). 4: Handover
 * messages; Report carries the DU's network-superframe offset
 * (for the TSF delta, HoCommand.h); Ack names what it acknowledges. */
/* 7: Neighbours (CU -> DU, the neighbour list for UE measurement gaps,
 * src/cell/MeasGap.h); Report carries the UEs' own measurements (ue_meas).
 * 6: Handover carries the target's channel (inter-frequency). */
/* 8: Phase (CU -> DU: a joining DU's TBTT phase, src/cell/PhasePlan.h); a
 * Report with phase_us < 0 asks for one. */
constexpr uint8_t kVersion = 11; /* v11: AreaKeys (per-UE area credentials) */
constexpr size_t kTagLen = 8;
constexpr size_t kMaxFixed = 8;
constexpr size_t kMaxPositions = kMaxUesPerCell;
constexpr size_t kMaxUes = kMaxUesPerCell;
constexpr size_t kMaxHeard = 64;
/* Every UE of a full cell with a full MR (serving + kMaxNeighbours). */
constexpr size_t kMaxUeMeas = kMaxUesPerCell * (kMaxNeighbours + 1); /* 216 < 256 */
/* Every F1 message fits this (a report at every list bound is 3673 bytes;
 * the cell_m6 ctest encodes one): the receive buffer on both sides. */
constexpr size_t kMaxDatagram = 8192;

enum class Type : uint8_t { Report = 1, Schedule = 2, Ack = 3, Handover = 4, Uplink = 5, Neighbours = 6,
                            Phase = 7, AreaKeys = 8 };
enum class AckStatus : uint8_t { Staged = 0, Applied = 1, Stale = 2, Late = 3, Invalid = 4 };

inline const char *ack_status_name(AckStatus s) {
  switch (s) {
  case AckStatus::Staged: return "staged";
  case AckStatus::Applied: return "applied";
  case AckStatus::Stale: return "stale";
  case AckStatus::Late: return "late";
  default: return "invalid";
  }
}

/* A UE this DU serves and its home identity (src/cell/Keys.h HomeId), as
 * recorded when the DU admitted it: what the CU derives its UE key from to
 * wrap a handover's context for it. */
struct UeHome {
  uint16_t ue = 0;
  HomeId home;
  bool operator==(const UeHome &) const = default;
};

struct HeardUe {
  uint16_t cell = 0, ue = 0;
  int8_t rssi_dbm = 0; /* filtered (src/cell/Handover.h L3Filter), dBm */
  int8_t snr_db = 0;   /* filtered, dB */
  uint16_t age_ms = 0; /* since its newest sample, at report time (saturates) */
  bool operator==(const HeardUe &) const = default;
};
/* What a UE measured itself (its MR IE, src/cell/MeasGap.h): `cell`'s
 * beacons — its serving cell's, and in gaps a neighbour's — relayed by the
 * serving DU. Quarter dB, with the report's age at the DU. */
struct UeCellMeas {
  uint16_t ue = 0, cell = 0;
  int16_t rssi_qdbm = 0, snr_qdb = 0;
  uint16_t age_ms = 0;
  bool operator==(const UeCellMeas &) const = default;
};
/* The DU's filtered uplink measurement of one of its own UEs (M6), quarter
 * dB, so the CU compares serving and neighbour cells on one scale. */
struct UeMeas {
  uint16_t ue = 0;
  int16_t rssi_qdbm = 0;
  int16_t snr_qdb = 0;
  uint16_t age_ms = 0; /* since its newest sample, at report time (saturates) */
  bool operator==(const UeMeas &) const = default;
};

struct Report {
  uint16_t du = 0; /* the DU's cell id */
  uint8_t tier = 0;       /* SyncTier */
  uint8_t sync_state = 0; /* SyncState */
  bool coordinated = false;
  /* The DU runs per-cell keys (Keys.h): it needs the CU's AreaKeys, so a CU
   * without the master key is a configuration error it reports. */
  bool per_cell_keys = false;
  uint8_t channel = 0;
  int64_t phase_us = 0;
  int64_t period_us = 0;
  uint64_t net_sf = 0;
  /* Network superframe - local superframe (valid with net_sf != 0): with
   * the phase, what fixes this DU's TSF against every other's. */
  int64_t net_off = 0;
  /* Freshness, as Schedule::seq (the DU's realtime clock, us): the CU
   * accepts a report only above the last one it took from this DU, so a
   * replayed report can neither refresh the DU's state nor move the address
   * its schedules go to. */
  uint64_t seq = 0;
  uint32_t applied_version = 0, staged_version = 0;
  std::vector<CoordInterval> fixed;
  std::vector<CoordPosition> positions;
  std::vector<CoordAssign> ues;
  std::vector<HeardUe> heard;
  std::vector<UeMeas> meas;
  std::vector<UeCellMeas> ue_meas;
  std::vector<UeHome> homes;
  bool operator==(const Report &) const = default;
};

struct Schedule {
  uint16_t du = 0;
  uint32_t version = 0;
  uint64_t effective_net_sf = 0;
  /* Freshness: strictly increasing per send (the CU's realtime clock, us),
   * authenticated with the rest. A replayed datagram never refreshes the
   * DU's CU contact. */
  uint64_t seq = 0;
  std::vector<CoordAssign> ues;
  bool operator==(const Schedule &) const = default;
};

/* What an Ack acknowledges. */
enum class AckKind : uint8_t { Schedule = 0, Handover = 1 };

struct Ack {
  uint16_t du = 0;
  AckKind kind = AckKind::Schedule;
  uint32_t version = 0; /* the schedule version, or the handover id */
  AckStatus status = AckStatus::Staged;
  uint64_t net_sf = 0;
  bool operator==(const Ack &) const = default;
};

namespace detail {
inline void put(std::vector<uint8_t> &o, uint64_t v, int n) {
  for (int i = 0; i < n; ++i)
    o.push_back(static_cast<uint8_t>(v >> (8 * i)));
}
/* A bounds-checked reader: every get fails once the buffer is short. */
struct Rd {
  const uint8_t *p;
  size_t n, i = 0;
  bool ok = true;
  uint64_t get(int k) {
    if (!ok || i + static_cast<size_t>(k) > n) {
      ok = false;
      return 0;
    }
    uint64_t v = 0;
    for (int b = 0; b < k; ++b)
      v |= static_cast<uint64_t>(p[i + b]) << (8 * b);
    i += static_cast<size_t>(k);
    return v;
  }
};
/* A home identity on the wire: kind(1) (0 static, 1 joiner) + 6 bytes (the
 * static cell(2) ue(2) 0(2), or the joiner MAC). */
inline void put_home(std::vector<uint8_t> &o, const HomeId &h) {
  o.push_back(h.joiner ? 1 : 0);
  if (h.joiner) {
    o.insert(o.end(), h.mac.begin(), h.mac.end());
  } else {
    put(o, h.cell, 2);
    put(o, h.ue, 2);
    put(o, 0, 2);
  }
}
inline HomeId get_home(Rd &r) {
  HomeId h;
  const uint64_t kind = r.get(1);
  if (kind > 1)
    r.ok = false;
  h.joiner = kind == 1;
  if (h.joiner) {
    for (auto &b : h.mac)
      b = static_cast<uint8_t>(r.get(1));
  } else {
    h.cell = static_cast<uint16_t>(r.get(2));
    h.ue = static_cast<uint16_t>(r.get(2));
    if (r.get(2) != 0)
      r.ok = false;
  }
  return h;
}
inline bool tag(const Mac &mac, const uint8_t *d, size_t n, uint8_t out[kTagLen]) {
  if (!mac) {
    std::memset(out, 0, kTagLen);
    return true;
  }
  std::vector<uint8_t> m = {'F'};
  m.insert(m.end(), d, d + n);
  uint8_t full[20];
  if (!mac(m.data(), m.size(), full))
    return false;
  std::memcpy(out, full, kTagLen);
  return true;
}
inline std::vector<uint8_t> head(Type t) {
  return {'F', '1', kVersion, static_cast<uint8_t>(t)};
}
inline std::vector<uint8_t> seal(std::vector<uint8_t> m, const Mac &mac) {
  uint8_t t[kTagLen];
  if (!tag(mac, m.data(), m.size(), t))
    return {};
  m.insert(m.end(), t, t + kTagLen);
  return m;
}
/* The body (after the 4-byte head, before the tag) of a verified message of
 * type `t`; nullopt for another type/version or a tag that does not verify. */
inline std::optional<Rd> open(const uint8_t *p, size_t n, Type t, const Mac &mac) {
  if (p == nullptr || n < 4 + kTagLen || p[0] != 'F' || p[1] != '1' || p[2] != kVersion ||
      p[3] != static_cast<uint8_t>(t))
    return std::nullopt;
  if (mac) {
    uint8_t want[kTagLen];
    if (!tag(mac, p, n - kTagLen, want))
      return std::nullopt;
    uint8_t d = 0;
    for (size_t i = 0; i < kTagLen; ++i)
      d |= static_cast<uint8_t>(want[i] ^ p[n - kTagLen + i]);
    if (d != 0)
      return std::nullopt;
  }
  return Rd{p + 4, n - 4 - kTagLen};
}
inline void put_assigns(std::vector<uint8_t> &o, const std::vector<CoordAssign> &v) {
  o.push_back(static_cast<uint8_t>(v.size()));
  for (const CoordAssign &a : v) {
    put(o, a.ue, 2);
    o.push_back(a.pos);
  }
}
inline bool get_assigns(Rd &r, std::vector<CoordAssign> &v) {
  const size_t n = r.get(1);
  if (n > kMaxUes)
    return false;
  for (size_t i = 0; i < n && r.ok; ++i) {
    CoordAssign a;
    a.ue = static_cast<uint16_t>(r.get(2));
    a.pos = static_cast<uint8_t>(r.get(1));
    v.push_back(a);
  }
  return r.ok;
}
inline void put_iv(std::vector<uint8_t> &o, const CoordInterval &iv) {
  put(o, static_cast<uint32_t>(iv.start_us), 4);
  put(o, static_cast<uint32_t>(iv.len_us), 4);
}
inline CoordInterval get_iv(Rd &r) {
  CoordInterval iv;
  iv.start_us = static_cast<int32_t>(static_cast<uint32_t>(r.get(4)));
  iv.len_us = static_cast<int32_t>(static_cast<uint32_t>(r.get(4)));
  return iv;
}
} /* namespace detail */

/* A DU's report measurements from its MeasTable snapshot: entries of its own
 * cell `own` go to `meas` (quarter dB), the others to `heard` (dB), each list
 * capped at its wire bound. */
inline void fill_report_measurements(Report &r, uint16_t own, const std::vector<MeasEntry> &snap) {
  auto clamp8 = [](double v) {
    return static_cast<int8_t>(std::lround(std::max(-128.0, std::min(127.0, v))));
  };
  auto clampq = [](double v) {
    return static_cast<int16_t>(std::lround(std::max(-32768.0, std::min(32767.0, v * 4.0))));
  };
  auto age16 = [](int64_t a) {
    return static_cast<uint16_t>(std::max<int64_t>(0, std::min<int64_t>(a, 65535)));
  };
  r.meas.clear();
  r.heard.clear();
  for (const MeasEntry &e : snap) {
    if (e.cell == own) {
      if (r.meas.size() < kMaxUes)
        r.meas.push_back({e.ue, clampq(e.rssi_dbm), clampq(e.snr_db), age16(e.age_ms)});
    } else if (r.heard.size() < kMaxHeard) {
      r.heard.push_back({e.cell, e.ue, clamp8(e.rssi_dbm), clamp8(e.snr_db), age16(e.age_ms)});
    }
  }
}

/* Message type of a datagram (before any verification), nullopt if not F1. */
inline std::optional<Type> peek_type(const uint8_t *p, size_t n) {
  if (p == nullptr || n < 4 + kTagLen || p[0] != 'F' || p[1] != '1' || p[2] != kVersion)
    return std::nullopt;
  if (p[3] < 1 || p[3] > 8)
    return std::nullopt;
  return static_cast<Type>(p[3]);
}

/* Empty when a list exceeds its bound (or the Mac fails). */
inline std::vector<uint8_t> encode_report(const Report &r, const Mac &mac) {
  using namespace detail;
  if (r.fixed.size() > kMaxFixed || r.positions.size() > kMaxPositions ||
      r.ues.size() > kMaxUes || r.heard.size() > kMaxHeard || r.meas.size() > kMaxUes ||
      r.ue_meas.size() > kMaxUeMeas || r.homes.size() > kMaxUes)
    return {};
  auto m = head(Type::Report);
  put(m, r.du, 2);
  m.push_back(r.tier);
  m.push_back(r.sync_state);
  m.push_back(static_cast<uint8_t>((r.coordinated ? 1 : 0) | (r.per_cell_keys ? 2 : 0)));
  m.push_back(r.channel);
  put(m, static_cast<uint64_t>(r.phase_us), 8);
  put(m, static_cast<uint64_t>(r.period_us), 8);
  put(m, r.net_sf, 8);
  put(m, static_cast<uint64_t>(r.net_off), 8);
  put(m, r.seq, 8);
  put(m, r.applied_version, 4);
  put(m, r.staged_version, 4);
  m.push_back(static_cast<uint8_t>(r.fixed.size()));
  for (const CoordInterval &f : r.fixed)
    put_iv(m, f);
  m.push_back(static_cast<uint8_t>(r.positions.size()));
  for (const CoordPosition &p : r.positions) {
    put_iv(m, p.dl);
    put_iv(m, p.ul);
  }
  put_assigns(m, r.ues);
  m.push_back(static_cast<uint8_t>(r.heard.size()));
  for (const HeardUe &h : r.heard) {
    put(m, h.cell, 2);
    put(m, h.ue, 2);
    m.push_back(static_cast<uint8_t>(h.rssi_dbm));
    m.push_back(static_cast<uint8_t>(h.snr_db));
    put(m, h.age_ms, 2);
  }
  m.push_back(static_cast<uint8_t>(r.meas.size()));
  for (const UeMeas &q : r.meas) {
    put(m, q.ue, 2);
    put(m, static_cast<uint16_t>(q.rssi_qdbm), 2);
    put(m, static_cast<uint16_t>(q.snr_qdb), 2);
    put(m, q.age_ms, 2);
  }
  m.push_back(static_cast<uint8_t>(r.ue_meas.size()));
  for (const UeCellMeas &q : r.ue_meas) {
    put(m, q.ue, 2);
    put(m, q.cell, 2);
    put(m, static_cast<uint16_t>(q.rssi_qdbm), 2);
    put(m, static_cast<uint16_t>(q.snr_qdb), 2);
    put(m, q.age_ms, 2);
  }
  m.push_back(static_cast<uint8_t>(r.homes.size()));
  for (const UeHome &h : r.homes) {
    put(m, h.ue, 2);
    put_home(m, h.home);
  }
  return seal(std::move(m), mac);
}

inline std::optional<Report> decode_report(const uint8_t *p, size_t n, const Mac &mac) {
  using namespace detail;
  auto o = open(p, n, Type::Report, mac);
  if (!o)
    return std::nullopt;
  Rd &r = *o;
  Report x;
  x.du = static_cast<uint16_t>(r.get(2));
  x.tier = static_cast<uint8_t>(r.get(1));
  x.sync_state = static_cast<uint8_t>(r.get(1));
  {
    const uint64_t f = r.get(1);
    if (f > 3)
      return std::nullopt;
    x.coordinated = (f & 1) != 0;
    x.per_cell_keys = (f & 2) != 0;
  }
  x.channel = static_cast<uint8_t>(r.get(1));
  x.phase_us = static_cast<int64_t>(r.get(8));
  x.period_us = static_cast<int64_t>(r.get(8));
  x.net_sf = r.get(8);
  x.net_off = static_cast<int64_t>(r.get(8));
  x.seq = r.get(8);
  x.applied_version = static_cast<uint32_t>(r.get(4));
  x.staged_version = static_cast<uint32_t>(r.get(4));
  const size_t nf = r.get(1);
  if (nf > kMaxFixed)
    return std::nullopt;
  for (size_t i = 0; i < nf && r.ok; ++i)
    x.fixed.push_back(get_iv(r));
  const size_t np = r.get(1);
  if (np > kMaxPositions)
    return std::nullopt;
  for (size_t i = 0; i < np && r.ok; ++i) {
    CoordPosition q;
    q.dl = get_iv(r);
    q.ul = get_iv(r);
    x.positions.push_back(q);
  }
  if (!get_assigns(r, x.ues))
    return std::nullopt;
  const size_t nh = r.get(1);
  if (nh > kMaxHeard)
    return std::nullopt;
  for (size_t i = 0; i < nh && r.ok; ++i) {
    HeardUe h;
    h.cell = static_cast<uint16_t>(r.get(2));
    h.ue = static_cast<uint16_t>(r.get(2));
    h.rssi_dbm = static_cast<int8_t>(r.get(1));
    h.snr_db = static_cast<int8_t>(r.get(1));
    h.age_ms = static_cast<uint16_t>(r.get(2));
    x.heard.push_back(h);
  }
  const size_t nm = r.get(1);
  if (nm > kMaxUes)
    return std::nullopt;
  for (size_t i = 0; i < nm && r.ok; ++i) {
    UeMeas q;
    q.ue = static_cast<uint16_t>(r.get(2));
    q.rssi_qdbm = static_cast<int16_t>(static_cast<uint16_t>(r.get(2)));
    q.snr_qdb = static_cast<int16_t>(static_cast<uint16_t>(r.get(2)));
    q.age_ms = static_cast<uint16_t>(r.get(2));
    x.meas.push_back(q);
  }
  const size_t nu = r.get(1);
  if (nu > kMaxUeMeas)
    return std::nullopt;
  for (size_t i = 0; i < nu && r.ok; ++i) {
    UeCellMeas q;
    q.ue = static_cast<uint16_t>(r.get(2));
    q.cell = static_cast<uint16_t>(r.get(2));
    q.rssi_qdbm = static_cast<int16_t>(static_cast<uint16_t>(r.get(2)));
    q.snr_qdb = static_cast<int16_t>(static_cast<uint16_t>(r.get(2)));
    q.age_ms = static_cast<uint16_t>(r.get(2));
    x.ue_meas.push_back(q);
  }
  const size_t nj = r.get(1);
  if (nj > kMaxUes)
    return std::nullopt;
  for (size_t i = 0; i < nj && r.ok; ++i) {
    UeHome h;
    h.ue = static_cast<uint16_t>(r.get(2));
    h.home = get_home(r);
    x.homes.push_back(h);
  }
  if (!r.ok || r.i != r.n)
    return std::nullopt;
  return x;
}

inline std::vector<uint8_t> encode_schedule(const Schedule &s, const Mac &mac) {
  using namespace detail;
  if (s.ues.size() > kMaxUes)
    return {};
  auto m = head(Type::Schedule);
  put(m, s.du, 2);
  put(m, s.version, 4);
  put(m, s.effective_net_sf, 8);
  put(m, s.seq, 8);
  put_assigns(m, s.ues);
  return seal(std::move(m), mac);
}

inline std::optional<Schedule> decode_schedule(const uint8_t *p, size_t n, const Mac &mac) {
  using namespace detail;
  auto o = open(p, n, Type::Schedule, mac);
  if (!o)
    return std::nullopt;
  Rd &r = *o;
  Schedule s;
  s.du = static_cast<uint16_t>(r.get(2));
  s.version = static_cast<uint32_t>(r.get(4));
  s.effective_net_sf = r.get(8);
  s.seq = r.get(8);
  if (!get_assigns(r, s.ues) || !r.ok || r.i != r.n)
    return std::nullopt;
  return s;
}

inline std::vector<uint8_t> encode_ack(const Ack &a, const Mac &mac) {
  using namespace detail;
  auto m = head(Type::Ack);
  put(m, a.du, 2);
  m.push_back(static_cast<uint8_t>(a.kind));
  put(m, a.version, 4);
  m.push_back(static_cast<uint8_t>(a.status));
  put(m, a.net_sf, 8);
  return seal(std::move(m), mac);
}

inline std::optional<Ack> decode_ack(const uint8_t *p, size_t n, const Mac &mac) {
  using namespace detail;
  auto o = open(p, n, Type::Ack, mac);
  if (!o)
    return std::nullopt;
  Rd &r = *o;
  Ack a;
  a.du = static_cast<uint16_t>(r.get(2));
  const uint8_t kind = static_cast<uint8_t>(r.get(1));
  if (kind > static_cast<uint8_t>(AckKind::Handover))
    return std::nullopt;
  a.kind = static_cast<AckKind>(kind);
  a.version = static_cast<uint32_t>(r.get(4));
  const uint8_t st = static_cast<uint8_t>(r.get(1));
  a.net_sf = r.get(8);
  if (!r.ok || r.i != r.n || st > static_cast<uint8_t>(AckStatus::Invalid))
    return std::nullopt;
  a.status = static_cast<AckStatus>(st);
  return a;
}

/* The handover command (HoCommand.h), CU -> both DUs; `du` addresses the
 * DU this copy is for. */
inline std::vector<uint8_t> encode_handover(uint16_t du, const HoCommand &h, const Mac &mac) {
  using namespace detail;
  auto m = head(Type::Handover);
  put(m, du, 2);
  put(m, h.id, 4);
  put(m, h.ue, 2);
  put(m, h.source, 2);
  put(m, h.target, 2);
  put(m, h.target_ue, 2);
  m.push_back(h.target_channel);
  m.push_back(h.target_pos);
  put(m, h.effective_net_sf, 8);
  put(m, static_cast<uint64_t>(h.tsf_delta_us), 8);
  put(m, h.seq, 8);
  m.insert(m.end(), h.ctx.begin(), h.ctx.end());
  put_home(m, h.home);
  return seal(std::move(m), mac);
}

/* {addressed DU, command}; nullopt for another type or a bad tag. */
inline std::optional<std::pair<uint16_t, HoCommand>> decode_handover(const uint8_t *p, size_t n,
                                                                     const Mac &mac) {
  using namespace detail;
  auto o = open(p, n, Type::Handover, mac);
  if (!o)
    return std::nullopt;
  Rd &r = *o;
  const uint16_t du = static_cast<uint16_t>(r.get(2));
  HoCommand h;
  h.id = static_cast<uint32_t>(r.get(4));
  h.ue = static_cast<uint16_t>(r.get(2));
  h.source = static_cast<uint16_t>(r.get(2));
  h.target = static_cast<uint16_t>(r.get(2));
  h.target_ue = static_cast<uint16_t>(r.get(2));
  h.target_channel = static_cast<uint8_t>(r.get(1));
  h.target_pos = static_cast<uint8_t>(r.get(1));
  h.effective_net_sf = r.get(8);
  h.tsf_delta_us = static_cast<int64_t>(r.get(8));
  h.seq = r.get(8);
  for (auto &b : h.ctx)
    b = static_cast<uint8_t>(r.get(1));
  h.home = get_home(r);
  if (!r.ok || r.i != r.n)
    return std::nullopt;
  return std::make_pair(du, h);
}

/* DAPS (M6): one UE uplink a DU heard, forwarded to the CU. `cell`/`ue`
 * are the identity it was heard under (the UE beacon's SA); `seq` its
 * common sequence (Handover.h ul_seq_from_egress). */
struct UplinkFwd {
  uint16_t du = 0;
  uint16_t cell = 0, ue = 0;
  uint32_t seq = 0;
  int8_t rssi_dbm = 0;
  bool operator==(const UplinkFwd &) const = default;
};

inline std::vector<uint8_t> encode_uplink(const UplinkFwd &u, const Mac &mac) {
  using namespace detail;
  auto m = head(Type::Uplink);
  put(m, u.du, 2);
  put(m, u.cell, 2);
  put(m, u.ue, 2);
  put(m, u.seq, 4);
  m.push_back(static_cast<uint8_t>(u.rssi_dbm));
  return seal(std::move(m), mac);
}

inline std::optional<UplinkFwd> decode_uplink(const uint8_t *p, size_t n, const Mac &mac) {
  using namespace detail;
  auto o = open(p, n, Type::Uplink, mac);
  if (!o)
    return std::nullopt;
  Rd &r = *o;
  UplinkFwd u;
  u.du = static_cast<uint16_t>(r.get(2));
  u.cell = static_cast<uint16_t>(r.get(2));
  u.ue = static_cast<uint16_t>(r.get(2));
  u.seq = static_cast<uint32_t>(r.get(4));
  u.rssi_dbm = static_cast<int8_t>(r.get(1));
  if (!r.ok || r.i != r.n)
    return std::nullopt;
  return u;
}

/* CU -> DU: the cells a DU's UEs should measure (their channel and TBTT
 * phase); the DU airs them as offsets on its own grid (MeasGap.h NL IE).
 * Resent each round; `seq` is freshness (as Schedule::seq). */
struct NeighbourEntry {
  uint16_t cell = 0;
  uint8_t channel = 0;
  int64_t phase_us = 0;
  bool operator==(const NeighbourEntry &) const = default;
};
struct Neighbours {
  uint16_t du = 0;
  uint64_t seq = 0;
  std::vector<NeighbourEntry> cells;
  bool operator==(const Neighbours &) const = default;
};

/* CU -> DU: the area keys of the OTHER cells' UEs (src/cell/Keys.h A_u),
 * each wrapped under this DU's cell key with a fresh nonce
 * (wrap_area_key): what the DU checks a heard UE beacon's area tag with.
 * Replaces the DU's whole table; `seq` is freshness. */
struct AreaKeyEntry {
  uint16_t cell = 0, ue = 0;
  uint64_t nonce = 0;
  std::array<uint8_t, 20> wrapped{};
  bool operator==(const AreaKeyEntry &) const = default;
};
constexpr size_t kMaxAreaKeys = kMaxUes * kMaxNeighbours; /* 192 x 32 B: fits kMaxDatagram */
struct AreaKeys {
  uint16_t du = 0;
  uint64_t seq = 0;
  std::vector<AreaKeyEntry> keys;
  bool operator==(const AreaKeys &) const = default;
};

inline std::vector<uint8_t> encode_area_keys(const AreaKeys &a, const Mac &mac) {
  using namespace detail;
  if (a.keys.size() > kMaxAreaKeys)
    return {};
  auto m = head(Type::AreaKeys);
  put(m, a.du, 2);
  put(m, a.seq, 8);
  put(m, a.keys.size(), 2);
  for (const AreaKeyEntry &e : a.keys) {
    put(m, e.cell, 2);
    put(m, e.ue, 2);
    put(m, e.nonce, 8);
    m.insert(m.end(), e.wrapped.begin(), e.wrapped.end());
  }
  return seal(std::move(m), mac);
}

inline std::optional<AreaKeys> decode_area_keys(const uint8_t *p, size_t n, const Mac &mac) {
  using namespace detail;
  auto o = open(p, n, Type::AreaKeys, mac);
  if (!o)
    return std::nullopt;
  Rd &r = *o;
  AreaKeys a;
  a.du = static_cast<uint16_t>(r.get(2));
  a.seq = r.get(8);
  const size_t k = r.get(2);
  if (k > kMaxAreaKeys)
    return std::nullopt;
  for (size_t i = 0; i < k && r.ok; ++i) {
    AreaKeyEntry e;
    e.cell = static_cast<uint16_t>(r.get(2));
    e.ue = static_cast<uint16_t>(r.get(2));
    e.nonce = r.get(8);
    for (auto &b : e.wrapped)
      b = static_cast<uint8_t>(r.get(1));
    a.keys.push_back(e);
  }
  if (!r.ok || r.i != r.n)
    return std::nullopt;
  return a;
}

inline std::vector<uint8_t> encode_neighbours(const Neighbours &n, const Mac &mac) {
  using namespace detail;
  if (n.cells.size() > kMaxNeighbours)
    return {};
  auto m = head(Type::Neighbours);
  put(m, n.du, 2);
  put(m, n.seq, 8);
  m.push_back(static_cast<uint8_t>(n.cells.size()));
  for (const NeighbourEntry &c : n.cells) {
    put(m, c.cell, 2);
    m.push_back(c.channel);
    put(m, static_cast<uint64_t>(c.phase_us), 8);
  }
  return seal(std::move(m), mac);
}

inline std::optional<Neighbours> decode_neighbours(const uint8_t *p, size_t n, const Mac &mac) {
  using namespace detail;
  auto o = open(p, n, Type::Neighbours, mac);
  if (!o)
    return std::nullopt;
  Rd &r = *o;
  Neighbours x;
  x.du = static_cast<uint16_t>(r.get(2));
  x.seq = r.get(8);
  const size_t cnt = r.get(1);
  if (cnt > kMaxNeighbours)
    return std::nullopt;
  for (size_t i = 0; i < cnt && r.ok; ++i) {
    NeighbourEntry c;
    c.cell = static_cast<uint16_t>(r.get(2));
    c.channel = static_cast<uint8_t>(r.get(1));
    c.phase_us = static_cast<int64_t>(r.get(8));
    x.cells.push_back(c);
  }
  if (!r.ok || r.i != r.n)
    return std::nullopt;
  return x;
}

/* CU -> DU: the phase a joining DU is to hold its grid at (the answer to a
 * Report with phase_us < 0); `seq` echoes that Report's, so the DU takes
 * only an answer to its own run's requests. */
struct PhaseAssign {
  uint16_t du = 0;
  int64_t phase_us = 0;
  uint64_t seq = 0;
  bool operator==(const PhaseAssign &) const = default;
};

inline std::vector<uint8_t> encode_phase(const PhaseAssign &a, const Mac &mac) {
  using namespace detail;
  auto m = head(Type::Phase);
  put(m, a.du, 2);
  put(m, static_cast<uint64_t>(a.phase_us), 8);
  put(m, a.seq, 8);
  return seal(std::move(m), mac);
}

inline std::optional<PhaseAssign> decode_phase(const uint8_t *p, size_t n, const Mac &mac) {
  using namespace detail;
  auto o = open(p, n, Type::Phase, mac);
  if (!o)
    return std::nullopt;
  Rd &r = *o;
  PhaseAssign a;
  a.du = static_cast<uint16_t>(r.get(2));
  a.phase_us = static_cast<int64_t>(r.get(8));
  a.seq = r.get(8);
  if (!r.ok || r.i != r.n)
    return std::nullopt;
  return a;
}

struct StagerConfig {
  uint64_t min_lead_sf = 6;     /* a schedule must arrive this far ahead */
  uint64_t publish_lead_sf = 3; /* the DU's grant map goes out this far ahead */
  int64_t cu_timeout_ms = 3000; /* no CU message this long = the CU is gone */
};

/* The DU's side of the failure model (see the header). Time is the caller's:
 * the network superframe now, and a monotonic ms clock. */
class Stager {
public:
  explicit Stager(StagerConfig c = {}) : cfg_(c) {}

  /* A verified schedule for this DU. `valid` says whether the DU can carry
   * it (its UEs, its positions). Any call counts as CU contact. */
  AckStatus on_schedule(const Schedule &s, uint64_t now_net_sf, int64_t now_ms,
                        const std::function<bool(const Schedule &)> &valid) {
    if (have_seq_ && s.seq <= last_seq_)
      return AckStatus::Stale; /* a replay (or reordering): not CU contact */
    have_seq_ = true;
    last_seq_ = s.seq;
    last_contact_ms_ = now_ms;
    contact_ = true;
    /* Older than anything applied or staged: stale, before any
     * same-version answer (an old applied version is not "applied" while a
     * newer one is staged). */
    const uint32_t floor_v = std::max(applied_ ? applied_->version : 0u,
                                      staged_ ? staged_->version : 0u);
    if ((applied_ || staged_) && s.version < floor_v)
      return AckStatus::Stale;
    if (applied_ && s.version == applied_->version)
      return AckStatus::Applied;
    if (staged_ && s.version == staged_->version)
      return AckStatus::Staged;
    if (s.version <= abandoned_floor_)
      return AckStatus::Stale;
    if (s.effective_net_sf < now_net_sf + cfg_.min_lead_sf)
      return AckStatus::Late;
    if (!valid || !valid(s))
      return AckStatus::Invalid;
    if (publishing_) /* a newer one may not overtake a schedule being published */
      return AckStatus::Late;
    staged_ = s;
    return AckStatus::Staged;
  }

  /* The staged schedule, once it is time to publish it (now >= effective -
   * publish_lead). It stays "publishing" until published() or it misses its
   * instant (then abandoned, returned by take_abandoned()). */
  std::optional<Schedule> due(uint64_t now_net_sf) {
    if (!staged_)
      return std::nullopt;
    const uint64_t eff = staged_->effective_net_sf;
    /* The grant map needs the M1 N+2 lead: past effective - 2 the UEs could
     * not apply it on the instant, so it is not applied at all. */
    if (now_net_sf + 2 > eff) {
      abandoned_ = staged_;
      abandoned_floor_ = std::max(abandoned_floor_, staged_->version);
      staged_.reset();
      publishing_ = false;
      return std::nullopt;
    }
    if (now_net_sf + cfg_.publish_lead_sf >= eff) {
      publishing_ = true;
      return staged_;
    }
    return std::nullopt;
  }
  /* The grant map carrying the due schedule is on air, at network
   * superframe `now_net_sf`. Only a schedule due() released counts, and only
   * within its deadline (the UEs need the N+2 lead); a publication that
   * completes later is abandoned, not applied. Returns whether it applied. */
  bool published(uint64_t now_net_sf) {
    if (!staged_ || !publishing_)
      return false;
    publishing_ = false;
    if (now_net_sf + 2 > staged_->effective_net_sf) {
      abandoned_ = staged_;
      abandoned_floor_ = std::max(abandoned_floor_, staged_->version);
      staged_.reset();
      return false;
    }
    applied_ = staged_;
    staged_.reset();
    return true;
  }
  std::optional<Schedule> take_abandoned() {
    auto a = abandoned_;
    abandoned_.reset();
    return a;
  }

  bool cu_alive(int64_t now_ms) const {
    return contact_ && now_ms - last_contact_ms_ <= cfg_.cu_timeout_ms;
  }
  uint32_t applied_version() const { return applied_ ? applied_->version : 0; }
  uint32_t staged_version() const { return staged_ ? staged_->version : 0; }
  const std::optional<Schedule> &applied() const { return applied_; }

private:
  StagerConfig cfg_;
  std::optional<Schedule> applied_, staged_, abandoned_;
  uint32_t abandoned_floor_ = 0;
  bool publishing_ = false;
  bool contact_ = false;
  bool have_seq_ = false;
  uint64_t last_seq_ = 0;
  int64_t last_contact_ms_ = 0;
};

} /* namespace devourer::cell::f1 */

#endif /* DEVOURER_CELL_F1_H */
