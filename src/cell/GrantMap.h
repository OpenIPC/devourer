/* GrantMap.h — the M1 (#255) DCI-style grant map: what the DU's beacon tells
 * each UE about its slots, as one 802.11 vendor-specific IE.
 *
 * Delivery is the M0 contract: the beacon body via UpdateBeaconPayload, which
 * airs on the next or next-but-one TBTT and is not atomic versus the TBTT. So
 * the map names the superframe it takes effect in (`effective_sf`); the
 * scheduler publishes for sf N+2 at sf N, and a UE applies a map only once its
 * own superframe count reaches effective_sf. `epoch` is the schedule version,
 * strictly increasing per cell, so a UE rejects a replayed or reordered map
 * (decode does not; that is UE state, see GrantMapReceiver).
 *
 * Wire format (little-endian), inside IE 221 (vendor) with OUI kGrantOui:
 *   'G' 'M' ver(1)=1 flags(1) cell_id(2) epoch(4) effective_sf(8)
 *   period_tu(2) n(1) n x {ue(2) dl_slot(1) ul_slot(1) rate_mbps(1)
 *   tx_pwr_qdb(1, signed) ta_us(2, signed)} crc16(2)
 * The CRC (CCITT-FALSE, over 'G' through the last grant) detects a torn or
 * corrupted body from one frame. It is not authentication: integrity against
 * an active attacker (EPIC #254's envelope item) is out of M1's scope.
 * Pure header; cell_selftest covers encode/decode round trips and rejects. */
#ifndef DEVOURER_CELL_GRANTMAP_H
#define DEVOURER_CELL_GRANTMAP_H

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "cell/SlotGrid.h"

namespace devourer::cell {

constexpr uint8_t kGrantOui[3] = {0x57, 0x42, 0x75}; /* devourer test OUI */
constexpr uint8_t kGrantVersion = 1;
constexpr size_t kGrantMax = kMaxUesPerCell; /* 3+21+24*8+2 = 218 <= 255 IE bytes */
/* GrantMap::flags. kGrantAutonomous (M4, #258): the DU has suspended its
 * COORDINATED schedule (it lost sync with the fabric, see src/cell/SyncHealth.h).
 * Its UEs fall back to carrier sense and a random uplink-beacon backoff, as
 * the DU does, so the desynced cell cannot collide with its neighbours'
 * slots. Clear (the default) = deterministic access on the grid. */
constexpr uint8_t kGrantAutonomous = 0x01;

struct Grant {
  uint16_t ue = 0;
  uint8_t dl_slot = 0;   /* SlotGrid index */
  uint8_t ul_slot = 0;
  uint8_t rate_mbps = 6;
  int8_t tx_pwr_qdb = 0; /* UE TX power offset, quarter dB */
  int16_t ta_us = 0;     /* timing advance the UE applies to its UL slot */
  bool operator==(const Grant &) const = default;
};

struct GrantMap {
  uint8_t flags = 0;
  uint16_t cell_id = 0;
  uint32_t epoch = 0;
  uint64_t effective_sf = 0;
  uint16_t period_tu = 0;
  std::vector<Grant> grants;
  bool operator==(const GrantMap &) const = default;

  const Grant *find(uint16_t ue) const {
    for (const Grant &g : grants)
      if (g.ue == ue)
        return &g;
    return nullptr;
  }
};

namespace detail {
inline uint16_t crc16_ccitt(const uint8_t *d, size_t n) {
  uint16_t c = 0xffff;
  for (size_t i = 0; i < n; ++i) {
    c ^= static_cast<uint16_t>(d[i] << 8);
    for (int b = 0; b < 8; ++b)
      c = (c & 0x8000) ? static_cast<uint16_t>((c << 1) ^ 0x1021) : static_cast<uint16_t>(c << 1);
  }
  return c;
}
inline void put(std::vector<uint8_t> &o, uint64_t v, int n) {
  for (int i = 0; i < n; ++i)
    o.push_back(static_cast<uint8_t>(v >> (8 * i)));
}
inline uint64_t get(const uint8_t *p, int n) {
  uint64_t v = 0;
  for (int i = 0; i < n; ++i)
    v |= static_cast<uint64_t>(p[i]) << (8 * i);
  return v;
}
constexpr size_t kHdrLen = 2 + 1 + 1 + 2 + 4 + 8 + 2 + 1; /* 'G'..n */
constexpr size_t kGrantLen = 2 + 1 + 1 + 1 + 1 + 2;
} /* namespace detail */

/* The full IE (221, len, OUI, body). Empty when there are > kGrantMax grants. */
inline std::vector<uint8_t> encode_grant_ie(const GrantMap &m) {
  if (m.grants.size() > kGrantMax)
    return {};
  std::vector<uint8_t> body;
  body.push_back('G');
  body.push_back('M');
  body.push_back(kGrantVersion);
  body.push_back(m.flags);
  detail::put(body, m.cell_id, 2);
  detail::put(body, m.epoch, 4);
  detail::put(body, m.effective_sf, 8);
  detail::put(body, m.period_tu, 2);
  body.push_back(static_cast<uint8_t>(m.grants.size()));
  for (const Grant &g : m.grants) {
    detail::put(body, g.ue, 2);
    body.push_back(g.dl_slot);
    body.push_back(g.ul_slot);
    body.push_back(g.rate_mbps);
    body.push_back(static_cast<uint8_t>(g.tx_pwr_qdb));
    detail::put(body, static_cast<uint16_t>(g.ta_us), 2);
  }
  detail::put(body, detail::crc16_ccitt(body.data(), body.size()), 2);
  std::vector<uint8_t> ie = {221, static_cast<uint8_t>(3 + body.size()),
                             kGrantOui[0], kGrantOui[1], kGrantOui[2]};
  ie.insert(ie.end(), body.begin(), body.end());
  return ie;
}

/* Decode one IE (starting at its 221 tag). nullopt on anything but an intact
 * version-1 grant map: wrong tag/OUI/magic/version, a length that disagrees
 * with n, or a CRC mismatch. */
inline std::optional<GrantMap> decode_grant_ie(const uint8_t *ie, size_t len) {
  if (ie == nullptr || len < 2 || ie[0] != 221 || static_cast<size_t>(ie[1]) + 2 > len)
    return std::nullopt;
  const size_t ie_len = ie[1];
  if (ie_len < 3 + detail::kHdrLen + 2 || ie[2] != kGrantOui[0] ||
      ie[3] != kGrantOui[1] || ie[4] != kGrantOui[2])
    return std::nullopt;
  const uint8_t *b = ie + 5;
  const size_t blen = ie_len - 3;
  if (b[0] != 'G' || b[1] != 'M' || b[2] != kGrantVersion)
    return std::nullopt;
  const size_t n = b[detail::kHdrLen - 1];
  if (n > kGrantMax || blen != detail::kHdrLen + n * detail::kGrantLen + 2)
    return std::nullopt;
  if (detail::get(b + blen - 2, 2) != detail::crc16_ccitt(b, blen - 2))
    return std::nullopt;
  GrantMap m;
  m.flags = b[3];
  m.cell_id = static_cast<uint16_t>(detail::get(b + 4, 2));
  m.epoch = static_cast<uint32_t>(detail::get(b + 6, 4));
  m.effective_sf = detail::get(b + 10, 8);
  m.period_tu = static_cast<uint16_t>(detail::get(b + 18, 2));
  const uint8_t *g = b + detail::kHdrLen;
  for (size_t i = 0; i < n; ++i, g += detail::kGrantLen) {
    Grant x;
    x.ue = static_cast<uint16_t>(detail::get(g, 2));
    x.dl_slot = g[2];
    x.ul_slot = g[3];
    x.rate_mbps = g[4];
    x.tx_pwr_qdb = static_cast<int8_t>(g[5]);
    x.ta_us = static_cast<int16_t>(detail::get(g + 6, 2));
    m.grants.push_back(x);
  }
  return m;
}

/* Find and decode the grant IE in a beacon MPDU (24-byte header, 12 fixed
 * bytes, then IEs). nullopt when the beacon carries none or it is damaged. */
inline std::optional<GrantMap> find_grant_map(const uint8_t *mpdu, size_t len) {
  if (mpdu == nullptr || len < 36 || mpdu[0] != 0x80)
    return std::nullopt;
  size_t off = 36;
  while (off + 2 <= len) {
    const size_t l = mpdu[off + 1];
    if (off + 2 + l > len)
      break;
    if (mpdu[off] == 221 && l >= 5 && mpdu[off + 2] == kGrantOui[0] &&
        mpdu[off + 3] == kGrantOui[1] && mpdu[off + 4] == kGrantOui[2] &&
        mpdu[off + 5] == 'G' && mpdu[off + 6] == 'M')
      return decode_grant_ie(mpdu + off, len - off);
    off += 2 + l;
  }
  return std::nullopt;
}

/* UE side: holds the map in force for ONE cell (epochs are per cell).
 * offer() takes a decoded map of that cell; it is queued if its epoch is newer
 * than anything seen from the cell, and current(sf) puts in force the newest
 * queued map whose effective_sf has been reached — so two future maps offered
 * back to back each take effect at their own superframe. Rejected: older or
 * equal epochs (replays, reordering) and other cells' maps. A newer map
 * replaces queued maps due at or after its own superframe (the scheduler
 * changed its mind). Moving to another cell is an explicit rebind(), which
 * forgets everything. */
class GrantMapReceiver {
public:
  static constexpr size_t kMaxPending = 8;

  explicit GrantMapReceiver(uint16_t cell_id) : cell_id_(cell_id) {}
  uint16_t cell_id() const { return cell_id_; }
  uint32_t newest_epoch() const { return newest_epoch_; }

  /* True when `m` was accepted (queued). */
  bool offer(const GrantMap &m) {
    if (m.cell_id != cell_id_)
      return false;
    if (have_any_ && m.epoch <= newest_epoch_)
      return false;
    have_any_ = true;
    newest_epoch_ = m.epoch;
    /* A newer epoch supersedes any queued map due at or after its own
     * superframe; maps due earlier still take effect first. */
    while (!pending_.empty() && pending_.back().effective_sf >= m.effective_sf)
      pending_.pop_back();
    if (pending_.size() == kMaxPending)
      pending_.erase(pending_.begin());
    pending_.push_back(m);
    return true;
  }
  /* The map in force at superframe `sf` (promotes the newest due one). */
  const GrantMap *current(uint64_t sf) {
    size_t due = 0;
    while (due < pending_.size() && sf >= pending_[due].effective_sf)
      ++due;
    if (due > 0) {
      current_ = std::move(pending_[due - 1]);
      pending_.erase(pending_.begin(), pending_.begin() + static_cast<std::ptrdiff_t>(due));
    }
    return current_ ? &*current_ : nullptr;
  }
  /* A restarted DU: its maps begin again at a low epoch, on another
   * superframe count. A map offer() would refuse as old, but due within
   * `window` superframes of the beacon carrying it (`beacon_sf`, that beacon's
   * TSF / period: the DU's count NOW), starts the receiver over — the caller
   * asks only once it has seen the DU's clock jump (a TsfSync step). A stale
   * map from before the restart is due far from the new count and still
   * refused. True when it started over. */
  bool offer_after_restart(const GrantMap &m, uint64_t beacon_sf, uint64_t window = 16) {
    /* Strictly lower: the same epoch again is the live map repeated. */
    if (m.cell_id != cell_id_ || !have_any_ || m.epoch >= newest_epoch_)
      return false;
    const uint64_t d = m.effective_sf > beacon_sf ? m.effective_sf - beacon_sf : beacon_sf - m.effective_sf;
    if (d > window)
      return false;
    rebind(cell_id_);
    return offer(m);
  }
  /* Serve another cell from scratch (handover, re-association). */
  void rebind(uint16_t cell_id) {
    cell_id_ = cell_id;
    have_any_ = false;
    newest_epoch_ = 0;
    pending_.clear();
    current_.reset();
  }

private:
  uint16_t cell_id_;
  bool have_any_ = false;
  uint32_t newest_epoch_ = 0;
  std::vector<GrantMap> pending_; /* ascending effective_sf (and epoch) */
  std::optional<GrantMap> current_;
};

} /* namespace devourer::cell */

#endif /* DEVOURER_CELL_GRANTMAP_H */
