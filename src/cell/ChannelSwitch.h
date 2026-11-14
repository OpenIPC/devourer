/* ChannelSwitch.h — M5 (#259) a running cell moves to another channel, with
 * its UEs: the CU's coordination planner gives a DU a spare channel
 * (Coord.h coord_plan, frequency separation); the DU announces it in its
 * beacon for some superframes and, at the instant, it and every UE it serves
 * retune together. An 802.11 channel-switch announcement in spirit, on the
 * cell's synced superframe count.
 *
 * The IE (221, OUI kGrantOui), body little-endian:
 *   'C' 'S' ver(1) id(4) cell(2) channel(1) effective_sf(8, the DU's count)
 *   crc16(2) tag(8)
 * The tag (label 'S') is the cell key (as the grant map's): only the serving
 * DU announces a move its UEs act on.
 *
 * Timing: the move happens in the idle part of superframe effective_sf - 1,
 * after every slot (the active span) and before the next beacon; the DU and
 * the UE each stop their beacon, retune and restart it there — the TSF does
 * not move, so the beacon grid and every slot keep their phase. The first
 * superframe on the new channel is effective_sf.
 *
 * v2, a move of PHASE (M6 #566, re-phasing a live cell; one or the other per
 * move): the cell's TBTT goes to another phase of the PTP timebase, with its
 * UEs. `step_us` != 0: at `at_us` into superframe effective_sf - 1 (the old
 * grid), the DU steps its TSF by `step_us` and every UE shifts its view of
 * the DU's clock by the same (and re-places its own uplink beacon). The
 * instant is idle in BOTH grids — after the old active span, and landing
 * after the new one — so no slot is cut or aired twice, and the superframe
 * count runs on (effective_sf - 1 is shortened or lengthened by the step);
 * rephase_plan() below picks it. The body gains step_us(4, signed)
 * at_us(4).
 *
 * Headless; cell_m5 ctest. */
#ifndef DEVOURER_CELL_CHANNELSWITCH_H
#define DEVOURER_CELL_CHANNELSWITCH_H

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

#include "cell/GrantMap.h"

namespace devourer::cell {

constexpr uint8_t kCsIeVersion = 2;
constexpr size_t kCsTagLen = 8;

struct CsIe {
  uint32_t id = 0;        /* CU-wide, increasing: a later move supersedes */
  uint16_t cell = 0;      /* the cell that moves (whose beacon carries it) */
  uint8_t channel = 0;    /* where to */
  uint64_t effective_sf = 0; /* the first superframe on the new channel, the DU's count */
  int32_t step_us = 0;    /* v2, a phase move: the DU's TSF step (0 = a channel move) */
  int32_t at_us = 0;      /* ... made this far into superframe effective_sf - 1 (old grid) */
  bool operator==(const CsIe &) const = default;
};

namespace cs_detail {
constexpr size_t kBodyLen = 2 + 1 + 4 + 2 + 1 + 8 + 4 + 4; /* 'C' 'S' .. effective_sf step at */
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
inline bool tag(const Mac &mac, const uint8_t *body, size_t n, uint8_t out[kCsTagLen]) {
  if (!mac) {
    std::memset(out, 0, kCsTagLen);
    return true;
  }
  std::vector<uint8_t> m = {'S'};
  m.insert(m.end(), body, body + n);
  uint8_t full[20];
  if (!mac(m.data(), m.size(), full))
    return false;
  std::memcpy(out, full, kCsTagLen);
  return true;
}
} /* namespace cs_detail */

/* The full IE; empty if the Mac fails. */
inline std::vector<uint8_t> encode_cs_ie(const CsIe &s, const Mac &mac) {
  using namespace cs_detail;
  std::vector<uint8_t> b = {'C', 'S', kCsIeVersion};
  put(b, s.id, 4);
  put(b, s.cell, 2);
  b.push_back(s.channel);
  put(b, s.effective_sf, 8);
  put(b, static_cast<uint32_t>(s.step_us), 4);
  put(b, static_cast<uint32_t>(s.at_us), 4);
  put(b, detail::crc16_ccitt(b.data(), b.size()), 2);
  uint8_t t[kCsTagLen];
  if (!tag(mac, b.data(), b.size(), t))
    return {};
  b.insert(b.end(), t, t + kCsTagLen);
  std::vector<uint8_t> ie = {221, static_cast<uint8_t>(3 + b.size()), kGrantOui[0], kGrantOui[1],
                             kGrantOui[2]};
  ie.insert(ie.end(), b.begin(), b.end());
  return ie;
}

/* One IE's payload (after the OUI): nullopt for another IE, a bad CRC or
 * (keyed) a tag that does not verify. */
inline std::optional<CsIe> decode_cs_body(const uint8_t *b, size_t n, const Mac &mac) {
  using namespace cs_detail;
  if (b == nullptr || n != kBodyLen + 2 + kCsTagLen || b[0] != 'C' || b[1] != 'S' || b[2] != kCsIeVersion)
    return std::nullopt;
  if (get(b + kBodyLen, 2) != detail::crc16_ccitt(b, kBodyLen))
    return std::nullopt;
  if (mac) {
    uint8_t want[kCsTagLen];
    if (!tag(mac, b, kBodyLen + 2, want))
      return std::nullopt;
    uint8_t d = 0;
    for (size_t i = 0; i < kCsTagLen; ++i)
      d |= static_cast<uint8_t>(want[i] ^ b[kBodyLen + 2 + i]);
    if (d != 0)
      return std::nullopt;
  }
  CsIe s;
  s.id = static_cast<uint32_t>(get(b + 3, 4));
  s.cell = static_cast<uint16_t>(get(b + 7, 2));
  s.channel = b[9];
  s.effective_sf = get(b + 10, 8);
  s.step_us = static_cast<int32_t>(static_cast<uint32_t>(get(b + 18, 4)));
  s.at_us = static_cast<int32_t>(static_cast<uint32_t>(get(b + 22, 4)));
  return s;
}

/* The channel-switch IE in one BEACON (radiotap stripped) of cell `cell`. */
inline std::optional<CsIe> find_cs_ie(const uint8_t *mpdu, size_t len, uint16_t cell, const Mac &mac) {
  if (mpdu == nullptr || len < 36 || mpdu[0] != 0x80)
    return std::nullopt;
  std::optional<CsIe> best;
  for (size_t off = 36; off + 2 <= len;) {
    const size_t l = mpdu[off + 1];
    if (off + 2 + l > len)
      break;
    const uint8_t *v = mpdu + off + 2;
    if (mpdu[off] == 221 && l > 3 && v[0] == kGrantOui[0] && v[1] == kGrantOui[1] && v[2] == kGrantOui[2])
      if (auto s = decode_cs_body(v + 3, l - 3, mac))
        if (s->cell == cell && (!best || s->id > best->id))
          best = s;
    off += 2 + l;
  }
  return best;
}

/* The UE's side: a captured, validly tagged beacon replayed later must not
 * move it — an IE is taken only when its id is above every one taken so far
 * AND its instant is still ahead of the UE's view of the cell's superframe.
 * The same move again (its id) is not a new one. */
class CsReceiver {
public:
  bool accept(const CsIe &s, uint64_t now_sf) {
    if (have_ && s.id <= last_id_)
      return false;
    if (s.effective_sf <= now_sf)
      return false;
    have_ = true;
    last_id_ = s.id;
    return true;
  }

private:
  bool have_ = false;
  uint32_t last_id_ = 0;
};

/* When in superframe effective_sf - 1 a node retunes (us into it): after
 * the active span (`span_us`, every slot) and `guard_us`; nullopt if the
 * retune (`retune_us`) and the guard would not end before the next beacon
 * (`period_us`): then the grid leaves no room for a move. */
inline std::optional<int64_t> cs_retune_at_us(int64_t span_us, int64_t retune_us, int64_t guard_us,
                                              int64_t period_us) {
  const int64_t at = span_us + guard_us;
  if (at + retune_us + guard_us > period_us)
    return std::nullopt;
  return at;
}

/* A phase move: the cell's TBTT from `old_phase` to `new_phase` (us of the
 * PTP timebase, in [0, period)). The TSF at an instant t goes from
 * t - old - n_old*P to t - new - n_new*P, a step of (old - new) mod P minus
 * 0 or P. It is made at `at_us` into the last old superframe, idle in both
 * grids: after the old active span + guard, and landing at
 * at_us + step_us, after the new active span + guard and a guard before the
 * next TBTT — so the superframe count runs on. Kestrel's beacon timer fires
 * ONCE more at the old phase after a TSF write: at the next old TBTT, which
 * on the new grid is (P + step) into the last superframe for a negative
 * step (idle: after the landing) and `step` into the first new one for a
 * positive step — so a positive step must clear the active span too.
 * Prefers the negative step (the stale beacon stays in the old superframe's
 * idle time). nullopt: no such instant (the phases closer than about the
 * active span: move twice). */
struct Rephase {
  int64_t at_us = 0;   /* into superframe effective_sf - 1, the old grid */
  int64_t step_us = 0; /* the TSF step there */
};
inline std::optional<Rephase> rephase_plan(int64_t old_phase, int64_t new_phase, int64_t span_us,
                                           int64_t guard_us, int64_t period_us) {
  const int64_t P = period_us;
  if (P <= 0 || span_us < 0 || guard_us < 0)
    return std::nullopt;
  const int64_t d = (((old_phase - new_phase) % P) + P) % P; /* the forward step */
  if (d == 0)
    return std::nullopt;
  const int64_t lo = span_us + guard_us, hi = P - guard_us; /* the idle part of a superframe */
  /* Negative step d - P: land at at + d - P >= lo, so at >= lo + P - d; the
   * stale old-phase beacon then fires d into the lengthened last superframe,
   * which must clear the guard before the next TBTT too (d < hi). */
  if (const int64_t at = lo + P - d; at >= lo && at < hi && d < hi)
    return Rephase{at, d - P};
  /* Positive step d: land at lo + d < hi; the stale TBTT at d into the next
   * superframe, clear of its active span. */
  if (lo + d < hi && d >= lo)
    return Rephase{lo, d};
  return std::nullopt;
}

} /* namespace devourer::cell */

#endif /* DEVOURER_CELL_CHANNELSWITCH_H */
