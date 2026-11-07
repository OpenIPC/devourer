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
 * Signed neighbour lists (per-cell keys): an HMAC tag a UE can check is one
 * it could forge, so with the area key every UE could fake any cell's NL. A
 * DU therefore also SIGNS its NL (Ed25519, its own key) and airs a
 * certificate for that key signed by the area CA:
 *   NS IE: 'N' 'S' ver(1)=1 cell(2) gen(4) sig(64)   sig over 'S' || the NL IE
 *   NC IE: 'N' 'C' ver(1)=1 cell(2) pub(32) ca_sig(64)  ca_sig over 'C' cell(2) pub
 * A UE holding only the CA's PUBLIC key (CELL_CA_PUB) checks the chain
 * (NlVerifier) — and can forge nothing. It caches the NL bytes it verified
 * per cell, so a signature is checked once per list, not per beacon. The
 * residual is the one any provenance has: a genuine beacon replayed.
 * The crypto is injected (examples fill it from OpenSSL), as the HMAC is.
 *
 * Pure header; the cell_m6 ctest covers it. */
#ifndef DEVOURER_CELL_MEASGAP_H
#define DEVOURER_CELL_MEASGAP_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <map>
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

/* --- Signed neighbour lists (see the header). --- */
constexpr uint8_t kNlSigVersion = 1;
using Ed25519Pub = std::array<uint8_t, 32>;
using Ed25519Sig = std::array<uint8_t, 64>;
/* msg -> its signature under the signer's key. False on failure. */
using NlSigner = std::function<bool(const uint8_t *msg, size_t n, uint8_t sig[64])>;
/* (public key, msg, signature) -> valid. */
using NlSigVerifier = std::function<bool(const uint8_t pub[32], const uint8_t *msg, size_t n, const uint8_t sig[64])>;

/* What the CA signs: 'C' cell(2) pub(32). */
inline std::vector<uint8_t> nl_cert_message(uint16_t cell, const Ed25519Pub &pub) {
  std::vector<uint8_t> m = {'C'};
  mg_detail::put(m, cell, 2);
  m.insert(m.end(), pub.begin(), pub.end());
  return m;
}

namespace mg_detail {
/* A full vendor IE (221, len, OUI, body), no CRC/tag: the signature and the
 * certificate carry their own integrity. */
inline std::vector<uint8_t> plain_ie(const std::vector<uint8_t> &body) {
  if (3 + body.size() > 255)
    return {};
  std::vector<uint8_t> ie = {221, static_cast<uint8_t>(3 + body.size()), kGrantOui[0], kGrantOui[1],
                             kGrantOui[2]};
  ie.insert(ie.end(), body.begin(), body.end());
  return ie;
}
/* The raw bytes (from the 221) of cell `cell`'s NL IE in a beacon — only
 * when the beacon carries exactly ONE for that cell: with two, which one the
 * signature covers and which one would be parsed could differ (a genuine
 * signed list followed by a forged one). */
inline std::optional<std::vector<uint8_t>> raw_nl_ie(const uint8_t *mpdu, size_t len, uint16_t cell) {
  if (mpdu == nullptr || len < 36 || mpdu[0] != 0x80)
    return std::nullopt;
  std::optional<std::vector<uint8_t>> one;
  for (size_t off = 36; off + 2 <= len;) {
    const size_t l = mpdu[off + 1];
    if (off + 2 + l > len)
      break;
    const uint8_t *v = mpdu + off + 2;
    if (mpdu[off] == 221 && l > 7 && v[0] == kGrantOui[0] && v[1] == kGrantOui[1] &&
        v[2] == kGrantOui[2] && v[3] == 'N' && v[4] == 'L' && get(v + 6, 2) == cell) {
      if (one)
        return std::nullopt; /* more than one list for this cell */
      one = std::vector<uint8_t>(mpdu + off, mpdu + off + 2 + l);
    }
    off += 2 + l;
  }
  return one;
}
} /* namespace mg_detail */

/* The NS IE for an NL IE (its full bytes, as aired). Empty if signing fails. */
inline std::vector<uint8_t> encode_nl_sig_ie(uint16_t cell, uint32_t gen, const std::vector<uint8_t> &nl_ie,
                                             const NlSigner &sign) {
  using namespace mg_detail;
  std::vector<uint8_t> msg = {'S'};
  msg.insert(msg.end(), nl_ie.begin(), nl_ie.end());
  uint8_t sig[64];
  if (!sign || !sign(msg.data(), msg.size(), sig))
    return {};
  std::vector<uint8_t> b = {'N', 'S', kNlSigVersion};
  put(b, cell, 2);
  put(b, gen, 4);
  b.insert(b.end(), sig, sig + 64);
  return plain_ie(b);
}
/* The NC IE: this cell's signing key and the CA's signature over it. */
inline std::vector<uint8_t> encode_nl_cert_ie(uint16_t cell, const Ed25519Pub &pub, const Ed25519Sig &ca_sig) {
  using namespace mg_detail;
  std::vector<uint8_t> b = {'N', 'C', kNlSigVersion};
  put(b, cell, 2);
  b.insert(b.end(), pub.begin(), pub.end());
  b.insert(b.end(), ca_sig.begin(), ca_sig.end());
  return plain_ie(b);
}

/* The UE's side: an NL counts only through the chain CA -> cell key -> NL.
 * Holds only public keys. Not thread-safe (the UE calls it under its lock). */
class NlVerifier {
public:
  NlVerifier(Ed25519Pub ca_pub, NlSigVerifier verify) : ca_(ca_pub), verify_(std::move(verify)) {}

  /* Cell `cell`'s neighbour list in this beacon, if it is signed by that
   * cell's certified key (or is exactly the list last verified for it). */
  std::optional<NlIe> check(const uint8_t *mpdu, size_t len, uint16_t cell) {
    using namespace mg_detail;
    auto raw = raw_nl_ie(mpdu, len, cell); /* the one list: what is verified IS what is parsed */
    if (!raw)
      return std::nullopt;
    /* Parse exactly those bytes (structure; the area tag is not the proof). */
    std::vector<uint8_t> only(36, 0);
    only[0] = 0x80;
    only.insert(only.end(), raw->begin(), raw->end());
    auto nl = find_nl_ie(only.data(), only.size(), cell, Mac{});
    if (!nl)
      return std::nullopt;
    if (auto it = ok_.find(cell); it != ok_.end() && it->second == *raw)
      return nl; /* verified before, byte for byte */
    /* The cell's certificate (verified once per key). */
    for_each_ie(mpdu, len, 'N', 'C', [&](const uint8_t *b, size_t n) {
      if (n != 5 + 32 + 64 || b[2] != kNlSigVersion || get(b + 3, 2) != cell)
        return;
      Ed25519Pub pub;
      std::copy(b + 5, b + 37, pub.begin());
      if (auto it = keys_.find(cell); it != keys_.end() && it->second == pub)
        return;
      const auto m = nl_cert_message(cell, pub);
      if (verify_ && verify_(ca_.data(), m.data(), m.size(), b + 37))
        keys_[cell] = pub;
    });
    auto key = keys_.find(cell);
    if (key == keys_.end())
      return std::nullopt;
    bool good = false;
    for_each_ie(mpdu, len, 'N', 'S', [&](const uint8_t *b, size_t n) {
      if (good || n != 9 + 64 || b[2] != kNlSigVersion || get(b + 3, 2) != cell ||
          static_cast<uint32_t>(get(b + 5, 4)) != nl->gen)
        return;
      std::vector<uint8_t> m = {'S'};
      m.insert(m.end(), raw->begin(), raw->end());
      good = verify_ && verify_(key->second.data(), m.data(), m.size(), b + 9);
    });
    if (!good)
      return std::nullopt;
    ok_[cell] = *raw;
    return nl;
  }

private:
  Ed25519Pub ca_;
  NlSigVerifier verify_;
  std::map<uint16_t, Ed25519Pub> keys_;              /* certified cell keys */
  std::map<uint16_t, std::vector<uint8_t>> ok_;      /* last verified NL IE, per cell */
};

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
