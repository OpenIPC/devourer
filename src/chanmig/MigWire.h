/* MigWire — the canonical, authenticated wire codec for the channel-migration
 * control protocol. Byte-packed little-endian (put/get helpers, no struct
 * casts — portable and KAT-stable, the HopSyncMarker style), every message
 * MAC'd with SipHash-2-4 (the codebase's existing primitive, src/HopSchedule.h)
 * under a control key domain-separated from any hop/link key.
 *
 * Anti-replay is structural: epochs are random per process start and never
 * persisted, so a restart orphans every in-flight generation for free (there
 * is no persisted in-flight state at all). A receiver adopts a peer epoch only
 * from a PROPOSAL/STATUS, tracks the max generation, and binds COMMIT/CONFIRM
 * to the nonce it issued — so a replayed old exchange can never be acked by
 * the current peer, and since the drone activates only after an ack, a replay
 * can never move it.
 *
 * Every message is <= 96 bytes (asserted in the KAT), so a control frame is
 * ~120 µs of 6M air. Pure, header-only, selftested. */
#ifndef DEVOURER_CHANMIG_MIG_WIRE_H
#define DEVOURER_CHANMIG_MIG_WIRE_H

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

#include "HopSchedule.h"
#include "chanmig/ChannelDef.h"
#include "chanmig/MigTypes.h"

namespace devourer {
namespace chanmig {

inline constexpr uint16_t kMigMagic = 0x4D43; /* bytes 0x43 0x4D = "CM" */
inline constexpr uint8_t kMigVersion = 1;
inline constexpr size_t kMigHeader = 8;   /* magic(2) ver(1) type(1) link(4) */
inline constexpr size_t kMigMacLen = 8;
inline constexpr size_t kMigMaxLen = 96;

/* 16-byte control key, domain-separated from a master seed. */
struct MigKey {
  std::array<uint8_t, 16> k{};
  static MigKey derive(const HopSchedule::Key &master) {
    MigKey out;
    static const uint8_t d0[] = {'d', 'e', 'v', 'o', 'u', 'r', 'e', 'r',
                                 '-', 'm', 'i', 'g', '-', 'c', '0'};
    static const uint8_t d1[] = {'d', 'e', 'v', 'o', 'u', 'r', 'e', 'r',
                                 '-', 'm', 'i', 'g', '-', 'c', '1'};
    const uint64_t lo = HopSchedule::siphash24(master, d0, sizeof(d0));
    const uint64_t hi = HopSchedule::siphash24(master, d1, sizeof(d1));
    for (int i = 0; i < 8; i++) {
      out.k[i] = static_cast<uint8_t>(lo >> (8 * i));
      out.k[8 + i] = static_cast<uint8_t>(hi >> (8 * i));
    }
    return out;
  }
  /* From DEVOURER_MIG_KEY hex text (reuses the hop seed parser). */
  static MigKey from_seed(const char *text) {
    return derive(HopSchedule::parse_seed(text));
  }
  uint64_t mac(const uint8_t *p, size_t n) const {
    return HopSchedule::siphash24(k, p, n);
  }
};

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
inline void pdef(std::vector<uint8_t> &b, const ChannelDef &d) {
  p8(b, d.band);
  p8(b, d.primary);
  p8(b, static_cast<uint8_t>(d.center_mhz() >= 5000
                                 ? (d.center_mhz() - 5000) / 5
                                 : (d.center_mhz() - 2407) / 5));
  p8(b, static_cast<uint8_t>(d.width));
  p8(b, d.offset);
  p8(b, static_cast<uint8_t>((d.no_ir ? 1 : 0) | (d.dfs ? 2 : 0) |
                             (d.backup ? 4 : 0)));
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
inline ChannelDef gdef(const uint8_t *p, size_t &o) {
  ChannelDef d;
  d.band = g8(p, o);
  d.primary = g8(p, o);
  (void)g8(p, o); /* center: derived + re-validated, carried for cross-check */
  d.width = static_cast<ChannelWidth_t>(g8(p, o));
  d.offset = g8(p, o);
  const uint8_t fl = g8(p, o);
  d.no_ir = fl & 1;
  d.dfs = fl & 2;
  d.backup = fl & 4;
  return d;
}
} /* namespace wire */

/* Finish a partially-built frame: append the SipHash MAC over everything so
 * far, returning the complete frame. */
inline std::vector<uint8_t> mig_seal(std::vector<uint8_t> b, const MigKey &key) {
  const uint64_t mac = key.mac(b.data(), b.size());
  wire::p64(b, mac);
  return b;
}

inline void mig_header(std::vector<uint8_t> &b, MigMsgType t, uint32_t link_id) {
  wire::p16(b, kMigMagic);
  wire::p8(b, kMigVersion);
  wire::p8(b, static_cast<uint8_t>(t));
  wire::p32(b, link_id);
}

/* --- per-type encoders (input is a fully-populated MigMsg) --- */
inline std::vector<uint8_t> mig_encode(const MigMsg &m, const MigKey &key) {
  std::vector<uint8_t> b;
  mig_header(b, m.type, m.link_id);
  switch (m.type) {
  case MT_PROPOSAL:
    wire::p32(b, m.ground_epoch);
    wire::p32(b, m.generation);
    wire::pdef(b, m.source);
    wire::pdef(b, m.target);
    wire::p32(b, m.evidence_gen);
    wire::p64(b, m.evidence_digest);
    wire::p64(b, m.earliest_tsf);
    wire::p64(b, m.latest_tsf);
    wire::p8(b, m.fallback_mode);
    wire::pdef(b, m.rescue);
    wire::p32(b, m.ground_nonce);
    break;
  case MT_COMMIT:
    wire::p32(b, m.drone_epoch);
    wire::p32(b, m.ground_epoch);
    wire::p32(b, m.generation);
    wire::pdef(b, m.target);
    wire::p64(b, m.activate_tsf);
    wire::p64(b, m.tx_tsf);
    wire::p32(b, m.confirm_window_us);
    wire::p64(b, m.rollback_deadline_tsf);
    wire::pdef(b, m.rescue);
    wire::p32(b, m.ground_nonce_echo);
    wire::p32(b, m.drone_nonce);
    wire::p8(b, m.armed);
    break;
  case MT_STATUS:
    wire::p8(b, m.role);
    wire::p32(b, m.sender_epoch);
    wire::p32(b, m.generation);
    wire::p8(b, m.state);
    wire::pdef(b, m.current);
    wire::p64(b, m.tx_tsf);
    wire::p8(b, m.reason);
    wire::p32(b, m.peer_nonce_echo);
    break;
  case MT_CONFIRM:
    wire::p32(b, m.ground_epoch);
    wire::p32(b, m.generation);
    wire::p32(b, m.drone_nonce_echo);
    wire::p16(b, m.marker_count);
    wire::p16(b, m.video_frames);
    wire::p32(b, m.first_marker_tsfl);
    break;
  case MT_ABORT:
    wire::p8(b, m.role);
    wire::p32(b, m.sender_epoch);
    wire::p32(b, m.generation);
    wire::p8(b, m.reason);
    wire::p8(b, m.effective);
    wire::p64(b, m.tx_tsf);
    break;
  case MT_MARKER:
    wire::p32(b, m.drone_epoch);
    wire::p32(b, m.generation);
    wire::p32(b, m.seq);
    wire::pdef(b, m.aired_on);
    wire::p8(b, m.marker_flags);
    wire::p64(b, m.tx_tsf);
    break;
  case MT_VALIDATION:
    wire::p32(b, m.drone_epoch);
    wire::p32(b, m.generation);
    wire::pdef(b, m.target);
    wire::p8(b, m.method);
    wire::p8(b, m.result);
    wire::p8(b, m.reason);
    wire::p32(b, m.obs_age_ms);
    wire::p32(b, m.obs_dur_ms);
    wire::p32(b, m.cca_delta);
    wire::p32(b, m.fa_delta);
    wire::p8(b, m.igi);
    wire::p8(b, m.nhm_busy_pct);
    wire::p8(b, m.energy_valid);
    wire::p16(b, m.cost_est_ms);
    wire::p64(b, m.tx_tsf);
    break;
  }
  return mig_seal(std::move(b), key);
}

/* Decode + authenticate. Returns MigReason::None on success (out filled), or
 * the specific rejection. link_id 0 skips the link check (KATs). */
inline MigReason mig_decode(const uint8_t *p, size_t n, const MigKey &key,
                            uint32_t expect_link, MigMsg &out) {
  /* n may exceed the message length: an on-air RX frame carries a trailing
   * 4-byte FCS (and some parsers pad), so the message occupies a PREFIX of the
   * buffer. The body length is fixed per type, so the decoder advances `o`
   * through the fixed fields, then locates the MAC right after — any bytes
   * beyond that (FCS/pad) are ignored. The upper bound allows the message +
   * a small trailer. */
  if (n < kMigHeader + kMigMacLen || n > kMigMaxLen + 8)
    return MigReason::Truncated;
  size_t o = 0;
  if (wire::g16(p, o) != kMigMagic)
    return MigReason::BadType;
  if (wire::g8(p, o) != kMigVersion)
    return MigReason::BadVersion;
  const uint8_t type = wire::g8(p, o);
  const uint32_t link = wire::g32(p, o);
  if (type < MT_PROPOSAL || type > MT_VALIDATION)
    return MigReason::BadType;
  if (expect_link != 0 && link != expect_link)
    return MigReason::BadLinkId;

  out = MigMsg{};
  out.type = static_cast<MigMsgType>(type);
  out.link_id = link;
  /* A per-type body reader; `o` ends at the byte after the fixed body, where
   * the MAC begins. A body that would read past the buffer is a truncation
   * (guarded after the switch). */
  switch (type) {
  case MT_PROPOSAL:
    out.ground_epoch = wire::g32(p, o);
    out.generation = wire::g32(p, o);
    out.source = wire::gdef(p, o);
    out.target = wire::gdef(p, o);
    out.evidence_gen = wire::g32(p, o);
    out.evidence_digest = wire::g64(p, o);
    out.earliest_tsf = wire::g64(p, o);
    out.latest_tsf = wire::g64(p, o);
    out.fallback_mode = wire::g8(p, o);
    out.rescue = wire::gdef(p, o);
    out.ground_nonce = wire::g32(p, o);
    break;
  case MT_COMMIT:
    out.drone_epoch = wire::g32(p, o);
    out.ground_epoch = wire::g32(p, o);
    out.generation = wire::g32(p, o);
    out.target = wire::gdef(p, o);
    out.activate_tsf = wire::g64(p, o);
    out.tx_tsf = wire::g64(p, o);
    out.confirm_window_us = wire::g32(p, o);
    out.rollback_deadline_tsf = wire::g64(p, o);
    out.rescue = wire::gdef(p, o);
    out.ground_nonce_echo = wire::g32(p, o);
    out.drone_nonce = wire::g32(p, o);
    out.armed = wire::g8(p, o);
    break;
  case MT_STATUS:
    out.role = wire::g8(p, o);
    out.sender_epoch = wire::g32(p, o);
    out.generation = wire::g32(p, o);
    out.state = wire::g8(p, o);
    out.current = wire::gdef(p, o);
    out.tx_tsf = wire::g64(p, o);
    out.reason = wire::g8(p, o);
    out.peer_nonce_echo = wire::g32(p, o);
    break;
  case MT_CONFIRM:
    out.ground_epoch = wire::g32(p, o);
    out.generation = wire::g32(p, o);
    out.drone_nonce_echo = wire::g32(p, o);
    out.marker_count = wire::g16(p, o);
    out.video_frames = wire::g16(p, o);
    out.first_marker_tsfl = wire::g32(p, o);
    break;
  case MT_ABORT:
    out.role = wire::g8(p, o);
    out.sender_epoch = wire::g32(p, o);
    out.generation = wire::g32(p, o);
    out.reason = wire::g8(p, o);
    out.effective = wire::g8(p, o);
    out.tx_tsf = wire::g64(p, o);
    break;
  case MT_MARKER:
    out.drone_epoch = wire::g32(p, o);
    out.generation = wire::g32(p, o);
    out.seq = wire::g32(p, o);
    out.aired_on = wire::gdef(p, o);
    out.marker_flags = wire::g8(p, o);
    out.tx_tsf = wire::g64(p, o);
    break;
  case MT_VALIDATION:
    out.drone_epoch = wire::g32(p, o);
    out.generation = wire::g32(p, o);
    out.target = wire::gdef(p, o);
    out.method = wire::g8(p, o);
    out.result = wire::g8(p, o);
    out.reason = wire::g8(p, o);
    out.obs_age_ms = wire::g32(p, o);
    out.obs_dur_ms = wire::g32(p, o);
    out.cca_delta = wire::g32(p, o);
    out.fa_delta = wire::g32(p, o);
    out.igi = wire::g8(p, o);
    out.nhm_busy_pct = wire::g8(p, o);
    out.energy_valid = wire::g8(p, o);
    out.cost_est_ms = wire::g16(p, o);
    out.tx_tsf = wire::g64(p, o);
    break;
  }
  /* the MAC sits immediately after the fixed body; trailing bytes (FCS/pad)
   * are ignored. Verify over exactly [0..o). */
  if (o + kMigMacLen > n)
    return MigReason::Truncated; /* body ran past the buffer */
  const uint64_t want = key.mac(p, o);
  size_t mo = o;
  const uint64_t got = wire::g64(p, mo);
  if (want != got)
    return MigReason::BadMac;
  return MigReason::None;
}

/* Anti-replay window: what a receiver remembers about one peer. Epochs are
 * random per boot (the caller seeds them from std::random_device), never
 * persisted. */
struct ReplayWindow {
  bool have_peer = false;
  uint32_t peer_epoch = 0;
  uint32_t max_gen = 0;
  uint32_t in_flight_gen = 0; /* 0 = none in flight */

  /* Adopt/verify a peer epoch. Only PROPOSAL/STATUS may introduce a new
   * epoch; a new epoch resets the generation ceiling (a fresh process). */
  bool accept_epoch(uint32_t epoch, bool may_introduce) {
    if (!have_peer) {
      if (!may_introduce)
        return false;
      have_peer = true;
      peer_epoch = epoch;
      max_gen = 0;
      in_flight_gen = 0;
      return true;
    }
    if (epoch == peer_epoch)
      return true;
    if (may_introduce) { /* peer restarted */
      peer_epoch = epoch;
      max_gen = 0;
      in_flight_gen = 0;
      return true;
    }
    return false; /* unknown epoch on a non-introducing message */
  }

  /* A proposal's generation: reject a replay (< max), idempotent re-answer
   * (== in-flight), or a fresh exchange (> max). Returns:
   *   0 = replay (reject), 1 = idempotent, 2 = fresh (adopt as in-flight). */
  int classify_proposal(uint32_t gen) {
    if (in_flight_gen != 0 && gen == in_flight_gen)
      return 1;
    if (gen <= max_gen)
      return 0;
    max_gen = gen;
    in_flight_gen = gen;
    return 2;
  }
  void clear_in_flight() { in_flight_gen = 0; }
};

} /* namespace chanmig */
} /* namespace devourer */

#endif /* DEVOURER_CHANMIG_MIG_WIRE_H */
