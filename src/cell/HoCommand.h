/* HoCommand.h — M6 (#260) the handover command: what moves a UE's schedule
 * from a source cell to a target cell, make-before-break, on the shared
 * PTP timebase.
 *
 * The CU decides (A3, src/cell/Handover.h) and sends the command to both
 * DUs over F1 (F1.h Type::Handover). The target DU pre-stages the UE in the
 * given position from `effective_net_sf`. The source DU relays the command
 * to the UE in its beacon (the IE below), and releases the UE only after
 * the target has it.
 *
 * TSF continuity. Every DU on the fabric holds its TSF grid to PTP time t
 * (M4): at network superframe N, DU k's TBTT is at t = phase_k + N * P,
 * with its local superframe N - net_off_k, so
 *     tsf_k(t) = t - phase_k - net_off_k * P
 * and the target's TSF is the source's plus a constant,
 *     tsf_target - tsf_source = (phase_s - phase_t) + (net_off_s - net_off_t) * P.
 * A UE locked to the source therefore knows the target's clock exactly:
 * no scan, no beacon re-acquisition, and no association exchange (its
 * identity at the target is assigned in the command).
 *
 * The beacon IE (vendor 221, kGrantOui), little-endian:
 *   'H' 'O' ver(1)=2 id(4) cell(2) ue(2) target(2) target_ue(2)
 *   dl_slot(1) ul_slot(1) channel(1) effective_sf(8) tsf_delta(8, signed)
 *   crc16(2) tag(8)
 * `channel` is the target's: when it differs from the source's the UE
 * retunes at the switch (inter-frequency; the TSF delta holds regardless,
 * so it needs no beacon of the target first).
 * `effective_sf` is in the SOURCE cell's superframe count (what the UE
 * runs). The CRC detects a torn body; the tag (the cell key's HMAC, as the
 * other scheduling frames, label 'H') makes a forged handover inert. An
 * unkeyed cell writes and ignores a zero tag (development).
 *
 * Replay: the tag stops forgery, not replay. HoReceiver (the UE side) acts
 * on an IE only if its id is above every one it accepted and its instant
 * is still ahead, so an old beacon replayed later is inert.
 *
 * Pure header; the cell_m6 ctest covers it. */
#ifndef DEVOURER_CELL_HOCOMMAND_H
#define DEVOURER_CELL_HOCOMMAND_H

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

#include "cell/GrantMap.h"
#include "cell/Keys.h"
#include "cell/Rach.h"

namespace devourer::cell {

/* 2: the target's channel (inter-frequency handover, the UE retunes). */
/* v3: the per-cell security context (src/cell/Keys.h) rides the IE. */
constexpr uint8_t kHoIeVersion = 3;
constexpr size_t kHoTagLen = 8;

/* target TSF - source TSF, at any instant (see the header). */
inline int64_t ho_tsf_delta_us(int64_t phase_source, int64_t net_off_source, int64_t phase_target,
                               int64_t net_off_target, int64_t period_us) {
  return (phase_source - phase_target) + (net_off_source - net_off_target) * period_us;
}

/* What the CU sends both DUs (F1.h). */
struct HoCommand {
  uint32_t id = 0;               /* handover id (monotone, as schedule versions) */
  uint16_t ue = 0;               /* the UE's id at the source */
  uint16_t source = 0, target = 0;
  uint16_t target_ue = 0;        /* its id at the target */
  uint8_t target_channel = 0;    /* the target cell's channel (0 = unknown: same) */
  uint8_t target_pos = 0;        /* its capacity position at the target; 0 =
                                    CANCEL this id (the target drops the UE it
                                    pre-admitted for it) */
  uint64_t effective_net_sf = 0; /* the switch instant, network-wide */
  int64_t tsf_delta_us = 0;      /* target TSF - source TSF */
  uint64_t seq = 0;              /* F1 freshness (as Schedule::seq) */
  /* The target cell's key wrapped for this UE, with its check (Keys.h
   * wrap_context), opaque to both DUs; all zero = none (one shared key). */
  std::array<uint8_t, 24> ctx{};
  /* The UE's home identity (Keys.h): the target records it for the id it
   * gives the UE and reports it on, for the UE's next handover. */
  HomeId home{};
  bool operator==(const HoCommand &) const = default;
};

/* What the source DU tells the UE (its beacon). */
struct HoIe {
  uint32_t id = 0;
  uint16_t cell = 0; /* the source cell (whose beacon carries it) */
  uint16_t ue = 0;
  uint16_t target = 0, target_ue = 0;
  uint8_t dl_slot = 0, ul_slot = 0; /* the target grid's slot indices */
  uint8_t channel = 0;              /* the target's channel (0 = the source's) */
  uint64_t effective_sf = 0;        /* in the source cell's count */
  int64_t tsf_delta_us = 0;
  std::array<uint8_t, 24> ctx{};    /* HoCommand::ctx, relayed */
  bool operator==(const HoIe &) const = default;
};

namespace ho_detail {
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
constexpr size_t kBodyLen = 2 + 1 + 4 + 2 + 2 + 2 + 2 + 1 + 1 + 1 + 8 + 8 + 24; /* 'H' 'O' .. ctx */
inline bool tag(const Mac &mac, const uint8_t *body, size_t n, uint8_t out[kHoTagLen]) {
  if (!mac) {
    std::memset(out, 0, kHoTagLen);
    return true;
  }
  std::vector<uint8_t> m = {'H'};
  m.insert(m.end(), body, body + n);
  uint8_t full[20];
  if (!mac(m.data(), m.size(), full))
    return false;
  std::memcpy(out, full, kHoTagLen);
  return true;
}
} /* namespace ho_detail */

/* The full IE (221, len, OUI, body, crc, tag); empty if the Mac fails. */
inline std::vector<uint8_t> encode_ho_ie(const HoIe &h, const Mac &mac) {
  using namespace ho_detail;
  std::vector<uint8_t> b = {'H', 'O', kHoIeVersion};
  put(b, h.id, 4);
  put(b, h.cell, 2);
  put(b, h.ue, 2);
  put(b, h.target, 2);
  put(b, h.target_ue, 2);
  b.push_back(h.dl_slot);
  b.push_back(h.ul_slot);
  b.push_back(h.channel);
  put(b, h.effective_sf, 8);
  put(b, static_cast<uint64_t>(h.tsf_delta_us), 8);
  b.insert(b.end(), h.ctx.begin(), h.ctx.end());
  put(b, detail::crc16_ccitt(b.data(), b.size()), 2);
  uint8_t t[kHoTagLen];
  if (!tag(mac, b.data(), b.size(), t))
    return {};
  b.insert(b.end(), t, t + kHoTagLen);
  std::vector<uint8_t> ie = {221, static_cast<uint8_t>(3 + b.size()), kGrantOui[0], kGrantOui[1],
                             kGrantOui[2]};
  ie.insert(ie.end(), b.begin(), b.end());
  return ie;
}

/* Decode one IE's payload (after the OUI); nullopt for another IE, a bad
 * CRC, or (keyed) a tag that does not verify. */
inline std::optional<HoIe> decode_ho_body(const uint8_t *b, size_t n, const Mac &mac) {
  using namespace ho_detail;
  if (b == nullptr || n != kBodyLen + 2 + kHoTagLen || b[0] != 'H' || b[1] != 'O' ||
      b[2] != kHoIeVersion)
    return std::nullopt;
  if (get(b + kBodyLen, 2) != detail::crc16_ccitt(b, kBodyLen))
    return std::nullopt;
  if (mac) {
    uint8_t want[kHoTagLen];
    if (!tag(mac, b, kBodyLen + 2, want))
      return std::nullopt;
    uint8_t d = 0;
    for (size_t i = 0; i < kHoTagLen; ++i)
      d |= static_cast<uint8_t>(want[i] ^ b[kBodyLen + 2 + i]);
    if (d != 0)
      return std::nullopt;
  }
  HoIe h;
  h.id = static_cast<uint32_t>(get(b + 3, 4));
  h.cell = static_cast<uint16_t>(get(b + 7, 2));
  h.ue = static_cast<uint16_t>(get(b + 9, 2));
  h.target = static_cast<uint16_t>(get(b + 11, 2));
  h.target_ue = static_cast<uint16_t>(get(b + 13, 2));
  h.dl_slot = b[15];
  h.ul_slot = b[16];
  h.channel = b[17];
  h.effective_sf = get(b + 18, 8);
  h.tsf_delta_us = static_cast<int64_t>(get(b + 26, 8));
  std::copy(b + 34, b + 58, h.ctx.begin());
  return h;
}

/* The handover IE in one BEACON (radiotap stripped) addressed to cell
 * `cell`'s UE `ue`; the newest id within this frame wins. Across beacons,
 * replay is the caller's to reject: HoReceiver below. */
inline std::optional<HoIe> find_ho_ie(const uint8_t *mpdu, size_t len, uint16_t cell, uint16_t ue,
                                      const Mac &mac) {
  if (mpdu == nullptr || len < 36 || mpdu[0] != 0x80) /* beacons only */
    return std::nullopt;
  std::optional<HoIe> best;
  for (size_t off = 36; off + 2 <= len;) {
    const size_t l = mpdu[off + 1];
    if (off + 2 + l > len)
      break;
    const uint8_t *v = mpdu + off + 2;
    if (mpdu[off] == 221 && l > 3 && v[0] == kGrantOui[0] && v[1] == kGrantOui[1] &&
        v[2] == kGrantOui[2])
      if (auto h = decode_ho_body(v + 3, l - 3, mac))
        if (h->cell == cell && h->ue == ue && (!best || h->id > best->id))
          best = h;
    off += 2 + l;
  }
  return best;
}

/* The UE's side: which handover IE it acts on. A captured, validly tagged
 * beacon replayed later must not re-present a superseded or spent handover,
 * so an IE is accepted only when its id is above every one accepted so far
 * AND its switch instant (effective_sf, in the source cell's count, covered
 * by the tag) is still ahead of the UE's current source superframe. */
class HoReceiver {
public:
  bool accept(const HoIe &h, uint64_t now_source_sf) {
    if (have_ && h.id <= last_id_)
      return false; /* superseded, or the same one again */
    if (h.effective_sf <= now_source_sf)
      return false; /* its instant has passed: spent */
    have_ = true;
    last_id_ = h.id;
    return true;
  }
  /* A new serving cell (after the switch): ids stay monotone (CU-wide). */
  uint32_t last_id() const { return last_id_; }

private:
  bool have_ = false;
  uint32_t last_id_ = 0;
};

} /* namespace devourer::cell */

#endif /* DEVOURER_CELL_HOCOMMAND_H */
