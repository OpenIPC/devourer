/* Keys.h — M6 (#260) the per-cell security context (802.11r-style).
 *
 * Every key comes from one MASTER key that only the CU and provisioning
 * hold. Keys are 40 hex characters (the HMAC-SHA1 of an info string under
 * the master), used as the HMAC keys of the cell's tags exactly as CELL_PSK
 * strings are:
 *   area key   K_A       = hex(HMAC(master, 'A'))
 *   cell key   K_c       = hex(HMAC(master, 'C' cell(2)))
 *   UE key     K_(c,u)   = hex(HMAC(master, 'U' home_cell(2) home_ue(2)))
 *   joiner key K_j       = hex(HMAC(master, 'J' joiner_mac(6)))  (a UE that
 *                          joins through the RACH: its UE key, provisioned out
 *                          of band against the MAC it joins with, as a SIM's)
 * (little-endian fields; the master is the HMAC key as given).
 *
 * Who holds what:
 *   DU of cell c   K_c (its CELL_PSK) and K_A (CELL_AREA_KEY)
 *   UE (home c,u)  K_c of its serving cell, K_A, and its own K_(c,u)
 *                  (CELL_UE_KEY); a joiner: its K_j there
 * The DU records every UE's home when it admits it — a static UE its (cell,
 * UE), a RACH joiner the MAC it joined with, a handover arrival the home the
 * CU's command names — and reports it (F1 Report.homes). The CU wraps each
 * context for the home in the SOURCE DU's current report: nothing survives
 * only in CU memory (a CU restart, an id reused for another UE). Claiming
 * another UE's MAC at the join gains nothing: only K_j's holder unwraps.
 *   CU             the master (RAN_MASTER_KEY)
 *
 * What each key tags:
 *   K_c   everything that controls cell c's UEs: the grant map, the handover
 *         IE, the DL body, the UE beacon's own tag (HARQ feedback), the MR IE,
 *         the RACH. A compromised DU or UE of one cell forges that cell only.
 *   K_A   what a UE must check of ANOTHER cell: the neighbour list (the
 *         provenance of a neighbour beacon it measures) — unless the DUs SIGN
 *         it: then a UE checks it with the CA's public key alone (below) and
 *         nobody but the cell's DU can make it.
 *   Ed25519 seeds (32 bytes = HMAC(master, info 0) || HMAC(master, info 1)[0..12]):
 *     a DU's NL signing key  info 'S' cell(2)  (CELL_NL_SIGN_KEY)
 *     the area CA            info 'R'          (its public key: CELL_CA_PUB;
 *                                               its signature over each DU's
 *                                               public key: CELL_NL_CERT)
 *     — the signing itself is provisioning's (MeasGap.h signed NL).
 *   A_u   the UE's own area key, hex(HMAC(K_u, 'V')): its beacon's second
 *         (area) tag, which neighbour DUs check before counting it as heard
 *         or forwarding it for DAPS. The CU derives every A_u from the homes
 *         the DUs report and sends each DU the other cells' UEs' (F1
 *         AreaKeys), each wrapped under that DU's cell key with a fresh
 *         nonce: no UE can forge another UE's area tag.
 *
 * The handover context. The UE needs the target cell's key before it switches
 * (to take the target's grant maps; after the switch, for everything). The CU
 * wraps K_t for that UE alone and the source DU relays the 24 opaque bytes in
 * its handover IE (HoCommand.h):
 *     ctx = raw(K_t) XOR HMAC(K_(c,u), 'X' id(4) target(2))
 *           || HMAC(K_(c,u), 'K' id(4) target(2) raw(K_t))[0..4]   (key check)
 * The pad is unique per (UE key, handover id, target): ids are CU-wide
 * monotone. The key check lets the UE refuse a context that is not its own
 * (another UE's, a wrong key, a tampered one) before it accepts the handover,
 * instead of finding out from target maps that never verify. The source DU
 * never learns K_t, and the other UEs of the source cell (which share K_s,
 * not K_(c,u)) cannot unwrap it. The UE keeps K_s until the switch commits
 * (its fallback).
 *
 * Without an area key (CELL_AREA_KEY unset) everything is the single shared
 * key, as before: K_A = K_c = CELL_PSK and no context is needed.
 *
 * Pure header; the cell_m6 ctest covers it. */
#ifndef DEVOURER_CELL_KEYS_H
#define DEVOURER_CELL_KEYS_H

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "cell/GrantMap.h"

namespace devourer::cell {

using KeyBytes = std::array<uint8_t, 20>;
constexpr size_t kCtxLen = 24; /* the wrapped key + its 4-byte check */
using Context = std::array<uint8_t, kCtxLen>;

inline std::string key_to_hex(const KeyBytes &k) {
  static const char *h = "0123456789abcdef";
  std::string s;
  for (uint8_t b : k) {
    s.push_back(h[b >> 4]);
    s.push_back(h[b & 15]);
  }
  return s;
}
inline std::optional<KeyBytes> key_from_hex(const std::string &s) {
  if (s.size() != 40)
    return std::nullopt;
  auto nib = [](char c) -> int {
    if (c >= '0' && c <= '9')
      return c - '0';
    if (c >= 'a' && c <= 'f')
      return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
      return c - 'A' + 10;
    return -1;
  };
  KeyBytes k{};
  for (size_t i = 0; i < 20; ++i) {
    const int a = nib(s[2 * i]), b = nib(s[2 * i + 1]);
    if (a < 0 || b < 0)
      return std::nullopt;
    k[i] = static_cast<uint8_t>(a << 4 | b);
  }
  return k;
}

namespace key_detail {
inline std::optional<KeyBytes> prf(const Mac &mac, const std::vector<uint8_t> &info) {
  KeyBytes out{};
  if (!mac || !mac(info.data(), info.size(), out.data()))
    return std::nullopt;
  return out;
}
inline void le(std::vector<uint8_t> &v, uint64_t x, int n) {
  for (int i = 0; i < n; ++i)
    v.push_back(static_cast<uint8_t>(x >> (8 * i)));
}
} /* namespace key_detail */

/* `master` = HMAC-SHA1 keyed by the master key. nullopt if it fails. */
inline std::optional<std::string> area_key(const Mac &master) {
  auto k = key_detail::prf(master, {'A'});
  return k ? std::optional<std::string>(key_to_hex(*k)) : std::nullopt;
}
inline std::optional<std::string> cell_key(const Mac &master, uint16_t cell) {
  std::vector<uint8_t> i = {'C'};
  key_detail::le(i, cell, 2);
  auto k = key_detail::prf(master, i);
  return k ? std::optional<std::string>(key_to_hex(*k)) : std::nullopt;
}
inline std::optional<std::string> ue_key(const Mac &master, uint16_t home_cell, uint16_t home_ue) {
  std::vector<uint8_t> i = {'U'};
  key_detail::le(i, home_cell, 2);
  key_detail::le(i, home_ue, 2);
  auto k = key_detail::prf(master, i);
  return k ? std::optional<std::string>(key_to_hex(*k)) : std::nullopt;
}

/* A 32-byte Ed25519 seed from the master (see the header). */
inline std::optional<std::array<uint8_t, 32>> ed25519_seed(const Mac &master, std::vector<uint8_t> info) {
  info.push_back(0);
  auto a = key_detail::prf(master, info);
  info.back() = 1;
  auto b = key_detail::prf(master, info);
  if (!a || !b)
    return std::nullopt;
  std::array<uint8_t, 32> seed{};
  std::copy(a->begin(), a->end(), seed.begin());
  std::copy(b->begin(), b->begin() + 12, seed.begin() + 20);
  return seed;
}
inline std::optional<std::array<uint8_t, 32>> nl_sign_seed(const Mac &master, uint16_t cell) {
  std::vector<uint8_t> i = {'S'};
  key_detail::le(i, cell, 2);
  return ed25519_seed(master, i);
}
inline std::optional<std::array<uint8_t, 32>> ca_seed(const Mac &master) {
  return ed25519_seed(master, {'R'});
}

/* A UE's home identity: where its UE key comes from. */
struct HomeId {
  bool joiner = false;
  uint16_t cell = 0, ue = 0;      /* static */
  std::array<uint8_t, 6> mac{};   /* joiner */
  bool operator==(const HomeId &) const = default;
};

inline std::optional<std::string> joiner_key(const Mac &master, const std::array<uint8_t, 6> &mac) {
  std::vector<uint8_t> i = {'J'};
  i.insert(i.end(), mac.begin(), mac.end());
  auto k = key_detail::prf(master, i);
  return k ? std::optional<std::string>(key_to_hex(*k)) : std::nullopt;
}

inline std::optional<std::string> home_key(const Mac &master, const HomeId &h) {
  return h.joiner ? joiner_key(master, h.mac) : ue_key(master, h.cell, h.ue);
}

/* A UE's own area key, from its UE key (`ue_mac` = HMAC keyed by it). */
inline std::optional<std::string> ue_area_key(const Mac &ue_mac) {
  auto k = key_detail::prf(ue_mac, {'V'});
  return k ? std::optional<std::string>(key_to_hex(*k)) : std::nullopt;
}
/* One UE's area key wrapped for one DU (`du_mac` = HMAC keyed by that DU's
 * cell key): key XOR HMAC(K_du, 'W' cell(2) ue(2) nonce(8)). The nonce is
 * fresh per wrap, so the pad never repeats. Its own inverse. */
inline std::optional<KeyBytes> wrap_area_key(const Mac &du_mac, uint16_t cell, uint16_t ue,
                                             uint64_t nonce, const KeyBytes &key) {
  std::vector<uint8_t> i = {'W'};
  key_detail::le(i, cell, 2);
  key_detail::le(i, ue, 2);
  key_detail::le(i, nonce, 8);
  auto pad = key_detail::prf(du_mac, i);
  if (!pad)
    return std::nullopt;
  KeyBytes out{};
  for (size_t k = 0; k < out.size(); ++k)
    out[k] = static_cast<uint8_t>(key[k] ^ (*pad)[k]);
  return out;
}

/* The context for one handover: the target's key wrapped for one UE
 * (`ue_mac` = HMAC keyed by that UE's key). nullopt if the Mac fails. */
inline std::optional<Context> wrap_context(const Mac &ue_mac, uint32_t id, uint16_t target,
                                           const KeyBytes &key) {
  std::vector<uint8_t> i = {'X'};
  key_detail::le(i, id, 4);
  key_detail::le(i, target, 2);
  auto pad = key_detail::prf(ue_mac, i);
  i[0] = 'K';
  i.insert(i.end(), key.begin(), key.end());
  auto chk = key_detail::prf(ue_mac, i);
  if (!pad || !chk)
    return std::nullopt;
  Context out{};
  for (size_t k = 0; k < key.size(); ++k)
    out[k] = static_cast<uint8_t>(key[k] ^ (*pad)[k]);
  std::copy(chk->begin(), chk->begin() + (kCtxLen - key.size()), out.begin() + key.size());
  return out;
}
/* The target's key, or nullopt when the context is not this UE's for this
 * handover (its key check fails) or the Mac fails. */
inline std::optional<KeyBytes> unwrap_context(const Mac &ue_mac, uint32_t id, uint16_t target,
                                              const Context &ctx) {
  std::vector<uint8_t> i = {'X'};
  key_detail::le(i, id, 4);
  key_detail::le(i, target, 2);
  auto pad = key_detail::prf(ue_mac, i);
  if (!pad)
    return std::nullopt;
  KeyBytes key{};
  for (size_t k = 0; k < key.size(); ++k)
    key[k] = static_cast<uint8_t>(ctx[k] ^ (*pad)[k]);
  i[0] = 'K';
  i.insert(i.end(), key.begin(), key.end());
  auto chk = key_detail::prf(ue_mac, i);
  if (!chk)
    return std::nullopt;
  uint8_t d = 0;
  for (size_t k = key.size(); k < kCtxLen; ++k)
    d |= static_cast<uint8_t>(ctx[k] ^ (*chk)[k - key.size()]);
  if (d != 0)
    return std::nullopt;
  return key;
}

} /* namespace devourer::cell */

#endif /* DEVOURER_CELL_KEYS_H */
