/* HopsetWire — the authenticated wire codec for the adaptive-hopset control
 * protocol. Byte-packed little-endian (put/get helpers, no struct casts —
 * portable and KAT-stable, the MigWire style), every message MAC'd with
 * SipHash-2-4 under the control key domain-separated from the hop seed
 * (hopset/HopsetState.h). A schedule fingerprint is not authentication — the
 * base_fp/mask_fp fields are config/routing checks, the MAC is the auth.
 *
 * Anti-replay is structural: epochs are random per process start and never
 * persisted, generations are monotonic within an authority epoch, and a
 * commit's absolute activate_slot must sit inside the follower's allowed
 * window — a replayed commit from an old boot names a slot in the past of a
 * different epoch and cannot arm anything.
 *
 * Every message is <= 96 bytes (asserted in the KAT). Pure, header-only. */
#ifndef DEVOURER_HOPSET_HOPSET_WIRE_H
#define DEVOURER_HOPSET_HOPSET_WIRE_H

#include <array>
#include <cstdint>
#include <vector>

#include "hopset/HopsetTypes.h"

namespace devourer {
namespace hopset {

inline constexpr uint16_t kHopsetMagic = 0x5348; /* bytes 0x48 0x53 = "HS" */
inline constexpr uint8_t kHopsetVersion = 1;
inline constexpr size_t kHopsetHeader = 8; /* magic(2) ver(1) type(1) link(4) */
inline constexpr size_t kHopsetMacLen = 8;
inline constexpr size_t kHopsetMaxLen = 96;

namespace wire {
inline void p8(std::vector<uint8_t> &b, uint8_t v) { b.push_back(v); }
inline void p16(std::vector<uint8_t> &b, uint16_t v) {
  b.push_back(uint8_t(v));
  b.push_back(uint8_t(v >> 8));
}
inline void p32(std::vector<uint8_t> &b, uint32_t v) {
  for (int i = 0; i < 4; i++)
    b.push_back(uint8_t(v >> (8 * i)));
}
inline void p64(std::vector<uint8_t> &b, uint64_t v) {
  for (int i = 0; i < 8; i++)
    b.push_back(uint8_t(v >> (8 * i)));
}
inline uint8_t g8(const uint8_t *p, size_t &o) { return p[o++]; }
inline uint16_t g16(const uint8_t *p, size_t &o) {
  uint16_t v = uint16_t(p[o]) | uint16_t(p[o + 1]) << 8;
  o += 2;
  return v;
}
inline uint32_t g32(const uint8_t *p, size_t &o) {
  uint32_t v = 0;
  for (int i = 0; i < 4; i++)
    v |= uint32_t(p[o + i]) << (8 * i);
  o += 4;
  return v;
}
inline uint64_t g64(const uint8_t *p, size_t &o) {
  uint64_t v = 0;
  for (int i = 0; i < 8; i++)
    v |= uint64_t(p[o + i]) << (8 * i);
  o += 8;
  return v;
}
} /* namespace wire */

/* Finish a partially-built frame: append the SipHash MAC (control key) over
 * everything so far. */
inline std::vector<uint8_t> hopset_seal(std::vector<uint8_t> b,
                                        const HopsetKeys &keys) {
  const uint64_t mac = keys.mac(b.data(), b.size());
  wire::p64(b, mac);
  return b;
}

inline std::vector<uint8_t> hopset_encode(const HopsetMsg &m,
                                          const HopsetKeys &keys) {
  std::vector<uint8_t> b;
  wire::p16(b, kHopsetMagic);
  wire::p8(b, kHopsetVersion);
  wire::p8(b, static_cast<uint8_t>(m.type));
  wire::p32(b, m.link_id);
  switch (m.type) {
  case HT_PROPOSAL: /* 56 bytes */
    wire::p32(b, m.rx_epoch);
    wire::p32(b, m.generation);
    wire::p64(b, m.active_mask);
    wire::p32(b, m.base_fp);
    wire::p32(b, m.observation_count);
    wire::p32(b, m.reason_bitmap);
    wire::p64(b, m.earliest_activate_round);
    wire::p32(b, m.rx_nonce);
    break;
  case HT_COMMIT: /* 68 bytes */
    wire::p32(b, m.tx_epoch);
    wire::p32(b, m.rx_epoch_echo);
    wire::p32(b, m.generation);
    wire::p64(b, m.active_mask);
    wire::p32(b, m.base_fp);
    wire::p64(b, m.activate_round);
    wire::p64(b, m.activate_slot);
    wire::p64(b, m.current_round);
    wire::p32(b, m.rx_nonce_echo);
    break;
  case HT_STATUS: /* 58 bytes */
    wire::p8(b, m.role);
    wire::p32(b, m.sender_epoch);
    wire::p32(b, m.generation);
    wire::p64(b, m.active_mask);
    wire::p32(b, m.base_fp);
    wire::p64(b, m.current_round);
    wire::p64(b, m.activate_slot);
    wire::p8(b, m.reason);
    wire::p32(b, m.rx_nonce_echo); /* binds a rejection to the proposal */
    break;
  }
  return hopset_seal(std::move(b), keys);
}

/* Decode + authenticate. Returns HopsetReason::None on success (out filled).
 * n may exceed the message length (trailing FCS/pad — the message occupies a
 * prefix; the MAC sits right after the fixed per-type body and is verified
 * over exactly [0..body)). link_id 0 skips the link check (KATs). */
inline HopsetReason hopset_decode(const uint8_t *p, size_t n,
                                  const HopsetKeys &keys, uint32_t expect_link,
                                  HopsetMsg &out) {
  if (n < kHopsetHeader + kHopsetMacLen || n > kHopsetMaxLen + 8)
    return HopsetReason::Truncated;
  size_t o = 0;
  if (wire::g16(p, o) != kHopsetMagic)
    return HopsetReason::BadType;
  if (wire::g8(p, o) != kHopsetVersion)
    return HopsetReason::BadVersion;
  const uint8_t type = wire::g8(p, o);
  const uint32_t link = wire::g32(p, o);
  if (type < HT_PROPOSAL || type > HT_STATUS)
    return HopsetReason::BadType;
  if (expect_link != 0 && link != expect_link)
    return HopsetReason::BadLinkId;

  out = HopsetMsg{};
  out.type = static_cast<HopsetMsgType>(type);
  out.link_id = link;
  switch (type) {
  case HT_PROPOSAL:
    if (n < 56)
      return HopsetReason::Truncated;
    out.rx_epoch = wire::g32(p, o);
    out.generation = wire::g32(p, o);
    out.active_mask = wire::g64(p, o);
    out.base_fp = wire::g32(p, o);
    out.observation_count = wire::g32(p, o);
    out.reason_bitmap = wire::g32(p, o);
    out.earliest_activate_round = wire::g64(p, o);
    out.rx_nonce = wire::g32(p, o);
    break;
  case HT_COMMIT:
    if (n < 68)
      return HopsetReason::Truncated;
    out.tx_epoch = wire::g32(p, o);
    out.rx_epoch_echo = wire::g32(p, o);
    out.generation = wire::g32(p, o);
    out.active_mask = wire::g64(p, o);
    out.base_fp = wire::g32(p, o);
    out.activate_round = wire::g64(p, o);
    out.activate_slot = wire::g64(p, o);
    out.current_round = wire::g64(p, o);
    out.rx_nonce_echo = wire::g32(p, o);
    break;
  case HT_STATUS:
    if (n < 58)
      return HopsetReason::Truncated;
    out.role = wire::g8(p, o);
    out.sender_epoch = wire::g32(p, o);
    out.generation = wire::g32(p, o);
    out.active_mask = wire::g64(p, o);
    out.base_fp = wire::g32(p, o);
    out.current_round = wire::g64(p, o);
    out.activate_slot = wire::g64(p, o);
    out.reason = wire::g8(p, o);
    out.rx_nonce_echo = wire::g32(p, o);
    break;
  }
  if (o + kHopsetMacLen > n)
    return HopsetReason::Truncated;
  const uint64_t want = keys.mac(p, o);
  size_t mo = o;
  const uint64_t got = wire::g64(p, mo);
  if (want != got)
    return HopsetReason::BadMac;
  return HopsetReason::None;
}

/* Sync-marker v2 — the v1 HopSyncMarker layout (HopSchedule.h) with the
 * version byte bumped to 2 and (generation, mask_fp) inserted before the
 * tail, so a follower detects a missed transition from the TX hot path
 * alone. The v1 decoder requires version==1 and the tail at its v1 offset,
 * so each version rejects the other (versioned backward rejection); the
 * demos emit v1 byte-identically whenever adaptive mode is off. */
struct HopSyncMarkerV2 {
  uint32_t fingerprint = 0, epoch = 0, phase_us = 0;
  uint64_t slot = 0;
  uint32_t generation = 0, mask_fp = 0;
  static constexpr size_t kSize = 37;
  static std::array<uint8_t, kSize> encode(const HopSyncMarkerV2 &m) {
    std::array<uint8_t, kSize> b{{221, kSize - 2, 0x57, 0x42, 0x75, 0x48, 2}};
    put32(b.data() + 7, m.fingerprint);
    put32(b.data() + 11, m.epoch);
    put64(b.data() + 15, m.slot);
    put32(b.data() + 23, m.phase_us);
    put32(b.data() + 27, m.generation);
    put32(b.data() + 31, m.mask_fp);
    b[35] = 0xd7;
    b[36] = 0x3a;
    return b;
  }
  static bool decode(const uint8_t *p, size_t n, HopSyncMarkerV2 &m) {
    for (size_t i = 0; i + kSize <= n; ++i)
      if (p[i] == 221 && p[i + 1] == kSize - 2 && p[i + 2] == 0x57 &&
          p[i + 3] == 0x42 && p[i + 4] == 0x75 && p[i + 5] == 0x48 &&
          p[i + 6] == 2 && p[i + 35] == 0xd7 && p[i + 36] == 0x3a) {
        m.fingerprint = get32(p + i + 7);
        m.epoch = get32(p + i + 11);
        m.slot = get64(p + i + 15);
        m.phase_us = get32(p + i + 23);
        m.generation = get32(p + i + 27);
        m.mask_fp = get32(p + i + 31);
        return true;
      }
    return false;
  }

private:
  static void put32(uint8_t *p, uint32_t v) {
    for (int i = 0; i < 4; ++i)
      p[i] = uint8_t(v >> (8 * i));
  }
  static void put64(uint8_t *p, uint64_t v) {
    for (int i = 0; i < 8; ++i)
      p[i] = uint8_t(v >> (8 * i));
  }
  static uint32_t get32(const uint8_t *p) {
    uint32_t v = 0;
    for (int i = 0; i < 4; ++i)
      v |= uint32_t(p[i]) << (8 * i);
    return v;
  }
  static uint64_t get64(const uint8_t *p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i)
      v |= uint64_t(p[i]) << (8 * i);
    return v;
  }
};

/* What an authority remembers about proposal replays: recently accepted
 * (epoch, nonce) pairs in a small ring — a replayed proposal from any boot
 * matches a remembered pair and is rejected, while generation monotonicity
 * (checked against the authority's own state) covers everything older. */
struct ProposalReplayRing {
  static constexpr size_t kDepth = 16;
  std::array<uint64_t, kDepth> seen{};
  size_t head = 0, count = 0;

  static uint64_t pack(uint32_t epoch, uint32_t nonce) {
    return (uint64_t(epoch) << 32) | nonce;
  }
  bool contains(uint32_t epoch, uint32_t nonce) const {
    const uint64_t v = pack(epoch, nonce);
    for (size_t i = 0; i < count; ++i)
      if (seen[i] == v)
        return true;
    return false;
  }
  void remember(uint32_t epoch, uint32_t nonce) {
    seen[head] = pack(epoch, nonce);
    head = (head + 1) % kDepth;
    if (count < kDepth)
      ++count;
  }
};

} /* namespace hopset */
} /* namespace devourer */

#endif /* DEVOURER_HOPSET_HOPSET_WIRE_H */
