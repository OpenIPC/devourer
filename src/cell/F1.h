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
 *            their positions, and the OTHER cells' UEs its radio hears (RSSI).
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
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <optional>
#include <vector>

#include "cell/Coord.h"
#include "cell/Rach.h"

namespace devourer::cell::f1 {

constexpr uint8_t kVersion = 1;
constexpr size_t kTagLen = 8;
constexpr size_t kMaxFixed = 8;
constexpr size_t kMaxPositions = kMaxUesPerCell;
constexpr size_t kMaxUes = kMaxUesPerCell;
constexpr size_t kMaxHeard = 64;

enum class Type : uint8_t { Report = 1, Schedule = 2, Ack = 3 };
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

struct HeardUe {
  uint16_t cell = 0, ue = 0;
  int8_t rssi_dbm = 0;
  bool operator==(const HeardUe &) const = default;
};

struct Report {
  uint16_t du = 0; /* the DU's cell id */
  uint8_t tier = 0;       /* SyncTier */
  uint8_t sync_state = 0; /* SyncState */
  bool coordinated = false;
  uint8_t channel = 0;
  int64_t phase_us = 0;
  int64_t period_us = 0;
  uint64_t net_sf = 0;
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

struct Ack {
  uint16_t du = 0;
  uint32_t version = 0;
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

/* Message type of a datagram (before any verification), nullopt if not F1. */
inline std::optional<Type> peek_type(const uint8_t *p, size_t n) {
  if (p == nullptr || n < 4 + kTagLen || p[0] != 'F' || p[1] != '1' || p[2] != kVersion)
    return std::nullopt;
  if (p[3] < 1 || p[3] > 3)
    return std::nullopt;
  return static_cast<Type>(p[3]);
}

/* Empty when a list exceeds its bound (or the Mac fails). */
inline std::vector<uint8_t> encode_report(const Report &r, const Mac &mac) {
  using namespace detail;
  if (r.fixed.size() > kMaxFixed || r.positions.size() > kMaxPositions ||
      r.ues.size() > kMaxUes || r.heard.size() > kMaxHeard)
    return {};
  auto m = head(Type::Report);
  put(m, r.du, 2);
  m.push_back(r.tier);
  m.push_back(r.sync_state);
  m.push_back(r.coordinated ? 1 : 0);
  m.push_back(r.channel);
  put(m, static_cast<uint64_t>(r.phase_us), 8);
  put(m, static_cast<uint64_t>(r.period_us), 8);
  put(m, r.net_sf, 8);
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
  x.coordinated = r.get(1) != 0;
  x.channel = static_cast<uint8_t>(r.get(1));
  x.phase_us = static_cast<int64_t>(r.get(8));
  x.period_us = static_cast<int64_t>(r.get(8));
  x.net_sf = r.get(8);
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
    x.heard.push_back(h);
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
  a.version = static_cast<uint32_t>(r.get(4));
  const uint8_t st = static_cast<uint8_t>(r.get(1));
  a.net_sf = r.get(8);
  if (!r.ok || r.i != r.n || st > static_cast<uint8_t>(AckStatus::Invalid))
    return std::nullopt;
  a.status = static_cast<AckStatus>(st);
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
