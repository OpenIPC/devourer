/* MeasGap.h — M6 (#260) inter-frequency measurements: what lets the A3
 * compare a UE's serving cell with a neighbour on ANOTHER channel when no
 * DU can hear the UE there (one radio per DU).
 *
 * The UE measures the cells' BEACONS (downlink): its serving DU's all the
 * time, a neighbour's in a measurement gap — it retunes to the neighbour's
 * channel just before that beacon airs and back right after. When to listen
 * comes from the shared PTP timebase: a neighbour's TBTT sits at a fixed
 * offset on the serving grid,
 *     offset = (phase_neighbour - phase_serving) mod P,
 * so the UE needs nothing beyond its serving clock.
 *
 *   NL IE (every synced DU beacon, also with zero entries; the CU's list):
 *     'N' 'L' ver(1)=2 cell(2) gen(4) n(1) n x {cell(2) channel(1)
 *     offset_us(4)} crc16(2) tag(8)
 *   MR IE (the UE's uplink beacon; relayed by the serving DU over F1):
 *     'M' 'R' ver(1)=2 cell(2) ue(2) gen(4) n(1) n x {cell(2)
 *     rssi_dbm(1, signed) snr_db(1, signed) age_ms(2)} crc16(2) tag(8)
 * Both carry the CRC (a torn body) and the cell key's tag (labels 'N' / 'M',
 * as the other scheduling frames): a forged list or report is inert.
 * Freshness: `gen` is covered by the tag and only moves forward — the NL's
 * per DU (a new list), the MR's per UE (each report, its serving superframe)
 * — so a replayed older list or report is refused by its receiver; an MR
 * entry's age is its sample's, so the CU never takes an expired sample for
 * a fresh one. Provenance: a UE counts a beacon as a measurement of a cell
 * only if it carries that cell's valid NL IE. The residual: a genuine beacon
 * replayed by someone else (only the real cell's, at the replayer's power).
 *
 * gap_window(): where in its superframe a UE may go deaf for one neighbour
 * beacon — [offset - retune - guard, offset + beacon + retune + guard) — or
 * nullopt when that overlaps anything the UE must be on its channel for
 * (the serving beacon, its own DL and UL slots). A UE that cannot fit a gap
 * simply does not measure that neighbour (no uplink is ever sacrificed).
 *
 * Pure header; the cell_m6 ctest covers it. */
#ifndef DEVOURER_CELL_MEASGAP_H
#define DEVOURER_CELL_MEASGAP_H

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

#include "cell/GrantMap.h"
#include "cell/Rach.h"

namespace devourer::cell {

constexpr uint8_t kNlVersion = 2;
constexpr uint8_t kMrVersion = 2;
constexpr size_t kMgTagLen = 8;
constexpr size_t kMaxNeighbours = 8;

struct Neighbour {
  uint16_t cell = 0;
  uint8_t channel = 0;
  int32_t offset_us = 0; /* its TBTT on the serving grid, [0, P) */
  bool operator==(const Neighbour &) const = default;
};

struct MeasEntryUe {
  uint16_t cell = 0;
  int8_t rssi_dbm = 0;
  int8_t snr_db = 0;
  uint16_t age_ms = 0; /* the sample's, at report time (saturates) */
  bool operator==(const MeasEntryUe &) const = default;
};
struct NlIe {
  uint32_t gen = 0;
  std::vector<Neighbour> cells;
  bool operator==(const NlIe &) const = default;
};
struct MrIe {
  uint32_t gen = 0;
  std::vector<MeasEntryUe> cells;
  bool operator==(const MrIe &) const = default;
};

/* A gap never crosses bands: a 2.4 <-> 5 GHz retune takes the full channel
 * path (~90 ms on the 8852CE), longer than any gap. */
inline bool gap_cross_band(uint8_t ch_a, uint8_t ch_b) { return (ch_a <= 14) != (ch_b <= 14); }

/* A neighbour's TBTT on the serving grid. */
inline int32_t neighbour_offset_us(int64_t phase_neighbour, int64_t phase_serving, int64_t period_us) {
  int64_t o = (phase_neighbour - phase_serving) % period_us;
  if (o < 0)
    o += period_us;
  return static_cast<int32_t>(o);
}

namespace mg_detail {
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
inline bool tag(const Mac &mac, char label, const uint8_t *body, size_t n, uint8_t out[kMgTagLen]) {
  if (!mac) {
    std::memset(out, 0, kMgTagLen);
    return true;
  }
  std::vector<uint8_t> m = {static_cast<uint8_t>(label)};
  m.insert(m.end(), body, body + n);
  uint8_t full[20];
  if (!mac(m.data(), m.size(), full))
    return false;
  std::memcpy(out, full, kMgTagLen);
  return true;
}
/* body + crc + tag -> the full vendor IE (empty if too long or Mac fails). */
inline std::vector<uint8_t> seal_ie(std::vector<uint8_t> b, char label, const Mac &mac) {
  put(b, detail::crc16_ccitt(b.data(), b.size()), 2);
  uint8_t t[kMgTagLen];
  if (!tag(mac, label, b.data(), b.size(), t) || 3 + b.size() + kMgTagLen > 255)
    return {};
  b.insert(b.end(), t, t + kMgTagLen);
  std::vector<uint8_t> ie = {221, static_cast<uint8_t>(3 + b.size()), kGrantOui[0], kGrantOui[1],
                             kGrantOui[2]};
  ie.insert(ie.end(), b.begin(), b.end());
  return ie;
}
/* Verify an IE payload (after the OUI) of `n` bytes ending crc + tag;
 * returns the body length (before the crc) or 0. */
inline size_t open_ie(const uint8_t *b, size_t n, char label, const Mac &mac) {
  if (n < 2 + kMgTagLen + 3)
    return 0;
  const size_t body = n - 2 - kMgTagLen;
  if (get(b + body, 2) != detail::crc16_ccitt(b, body))
    return 0;
  if (mac) {
    uint8_t want[kMgTagLen];
    if (!tag(mac, label, b, body + 2, want))
      return 0;
    uint8_t d = 0;
    for (size_t i = 0; i < kMgTagLen; ++i)
      d |= static_cast<uint8_t>(want[i] ^ b[body + 2 + i]);
    if (d != 0)
      return 0;
  }
  return body;
}
/* Each vendor IE payload (after the OUI) in a beacon, beginning with m0 m1. */
template <typename F>
inline void for_each_ie(const uint8_t *mpdu, size_t len, uint8_t m0, uint8_t m1, F &&f) {
  if (mpdu == nullptr || len < 36 || mpdu[0] != 0x80)
    return;
  for (size_t off = 36; off + 2 <= len;) {
    const size_t l = mpdu[off + 1];
    if (off + 2 + l > len)
      break;
    const uint8_t *v = mpdu + off + 2;
    if (mpdu[off] == 221 && l > 5 && v[0] == kGrantOui[0] && v[1] == kGrantOui[1] &&
        v[2] == kGrantOui[2] && v[3] == m0 && v[4] == m1)
      f(v + 3, l - 3);
    off += 2 + l;
  }
}
} /* namespace mg_detail */

inline std::vector<uint8_t> encode_nl_ie(uint16_t cell, uint32_t gen, const std::vector<Neighbour> &n,
                                         const Mac &mac) {
  using namespace mg_detail;
  if (n.size() > kMaxNeighbours)
    return {};
  std::vector<uint8_t> b = {'N', 'L', kNlVersion};
  put(b, cell, 2);
  put(b, gen, 4);
  b.push_back(static_cast<uint8_t>(n.size()));
  for (const Neighbour &x : n) {
    put(b, x.cell, 2);
    b.push_back(x.channel);
    put(b, static_cast<uint32_t>(x.offset_us), 4);
  }
  return seal_ie(std::move(b), 'N', mac);
}

/* The neighbour list in cell `cell`'s beacon (nullopt: none / not authentic).
 * Its presence also authenticates the beacon as that cell's. */
inline std::optional<NlIe> find_nl_ie(const uint8_t *mpdu, size_t len, uint16_t cell, const Mac &mac) {
  using namespace mg_detail;
  std::optional<NlIe> out;
  for_each_ie(mpdu, len, 'N', 'L', [&](const uint8_t *b, size_t n) {
    const size_t body = open_ie(b, n, 'N', mac);
    if (body < 10 || b[2] != kNlVersion || get(b + 3, 2) != cell)
      return;
    const size_t cnt = b[9];
    if (cnt > kMaxNeighbours || body != 10 + cnt * 7)
      return;
    NlIe x;
    x.gen = static_cast<uint32_t>(get(b + 5, 4));
    for (size_t i = 0; i < cnt; ++i) {
      const uint8_t *e = b + 10 + i * 7;
      x.cells.push_back({static_cast<uint16_t>(get(e, 2)), e[2],
                         static_cast<int32_t>(static_cast<uint32_t>(get(e + 3, 4)))});
    }
    out = std::move(x);
  });
  return out;
}

inline std::vector<uint8_t> encode_mr_ie(uint16_t cell, uint16_t ue, uint32_t gen,
                                         const std::vector<MeasEntryUe> &m, const Mac &mac) {
  using namespace mg_detail;
  if (m.size() > kMaxNeighbours + 1)
    return {};
  std::vector<uint8_t> b = {'M', 'R', kMrVersion};
  put(b, cell, 2);
  put(b, ue, 2);
  put(b, gen, 4);
  b.push_back(static_cast<uint8_t>(m.size()));
  for (const MeasEntryUe &x : m) {
    put(b, x.cell, 2);
    b.push_back(static_cast<uint8_t>(x.rssi_dbm));
    b.push_back(static_cast<uint8_t>(x.snr_db));
    put(b, x.age_ms, 2);
  }
  return seal_ie(std::move(b), 'M', mac);
}

/* The measurement report in a UE beacon of cell `cell`'s UE `ue`. */
inline std::optional<MrIe> find_mr_ie(const uint8_t *mpdu, size_t len, uint16_t cell, uint16_t ue,
                                      const Mac &mac) {
  using namespace mg_detail;
  std::optional<MrIe> out;
  for_each_ie(mpdu, len, 'M', 'R', [&](const uint8_t *b, size_t n) {
    const size_t body = open_ie(b, n, 'M', mac);
    if (body < 12 || b[2] != kMrVersion || get(b + 3, 2) != cell || get(b + 5, 2) != ue)
      return;
    const size_t cnt = b[11];
    if (cnt > kMaxNeighbours + 1 || body != 12 + cnt * 6)
      return;
    MrIe x;
    x.gen = static_cast<uint32_t>(get(b + 7, 4));
    for (size_t i = 0; i < cnt; ++i) {
      const uint8_t *e = b + 12 + i * 6;
      x.cells.push_back({static_cast<uint16_t>(get(e, 2)), static_cast<int8_t>(e[2]),
                         static_cast<int8_t>(e[3]), static_cast<uint16_t>(get(e + 4, 2))});
    }
    out = std::move(x);
  });
  return out;
}

/* An interval on the serving grid, [start, start + len) mod P. */
struct GridSpan {
  int64_t start = 0, len = 0;
};

/* The gap for one neighbour beacon (see the header): its start on the
 * serving grid and length, or nullopt if it would cover any `busy` span. */
inline std::optional<GridSpan> gap_window(int32_t neighbour_offset_us, int64_t beacon_us,
                                          int64_t retune_us, int64_t guard_us, int64_t period_us,
                                          const std::vector<GridSpan> &busy) {
  const int64_t P = period_us;
  const int64_t start = neighbour_offset_us - retune_us - guard_us;
  const int64_t len = 2 * (retune_us + guard_us) + beacon_us;
  if (len <= 0 || len >= P)
    return std::nullopt;
  auto mod = [P](int64_t x) { return ((x % P) + P) % P; };
  for (const GridSpan &b : busy) {
    if (b.len <= 0)
      continue;
    if (mod(b.start - start) < len || mod(start - b.start) < b.len)
      return std::nullopt;
  }
  return GridSpan{mod(start), len};
}

} /* namespace devourer::cell */

#endif /* DEVOURER_CELL_MEASGAP_H */
