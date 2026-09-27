/* Eapol — the EAPOL-Key wire format and the key derivations built on it.
 *
 * This is format and arithmetic only: parse, build, derive, verify. The
 * decisions - what to do with a message that arrives in the wrong state,
 * whether a replay counter is acceptable, when to install a key - belong to
 * Supplicant.h, which is where they can be tested as decisions.
 *
 * Everything cryptographic goes through CryptoOps, so `libdevourer` gains no
 * dependency. That is not a stylistic preference: hand-rolled AES, CCM,
 * PBKDF2 or PRF code without known-answer tests is the defect class this
 * module exists to avoid, and every primitive used here is pinned by vectors
 * in tests/.
 *
 * ONLY KEY DESCRIPTOR VERSION 2 (HMAC-SHA1 MIC, AES key wrap, CCMP). Version 1
 * is TKIP - HMAC-MD5 and RC4 - and version 3 is AES-128-CMAC. Neither is
 * implemented, and both are REFUSED rather than treated as version 2, because
 * a version mismatch means the MIC is computed with a different algorithm and
 * "the MIC did not verify" would be the only symptom.
 */
#ifndef DEVOURER_STA_EAPOL_H
#define DEVOURER_STA_EAPOL_H

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "sta/CryptoOps.h"

namespace devourer {
namespace sta {

/* An EAPOL-Key frame is 99 bytes before its key data. The offsets below are
 * 802.11-2016 12.7.2.
 *
 * DO NOT CONVERT tests/ap_wpa2.cpp ONTO THESE. That authenticator hand-rolls
 * every one of these offsets inline and shares no code with this header -
 * which is exactly what makes a cross-role
 * test between the two a real oracle. Two implementations from
 * one set of constants cannot disagree, and cannot catch a misreading either.
 * The duplication is the test. */
inline constexpr size_t kEapolKeyFixedLen = 99;
inline constexpr size_t kEapolMicOff = 81;
inline constexpr size_t kEapolMicLen = 16;
inline constexpr size_t kEapolNonceOff = 17;
inline constexpr size_t kEapolReplayOff = 9;
inline constexpr size_t kEapolRscOff = 65;
inline constexpr size_t kEapolKeyDataLenOff = 97;

/* Key Information bits (802.11-2016 Figure 12-34). */
enum : uint16_t {
  kKiVersionMask = 0x0007,
  kKiPairwise = 0x0008,
  kKiKeyIdMask = 0x0030,
  kKiInstall = 0x0040,
  kKiAck = 0x0080,
  kKiMic = 0x0100,
  kKiSecure = 0x0200,
  kKiError = 0x0400,
  kKiRequest = 0x0800,
  kKiEncrypted = 0x1000,
};

/* The only key descriptor version this implements: HMAC-SHA1-128 MIC and
 * NIST AES key wrap, which is what WPA2-PSK with CCMP uses. */
inline constexpr uint16_t kKeyDescVersionCcmp = 2;
inline constexpr uint8_t kKeyDescTypeRsn = 2;

/* THE 802.1X PROTOCOL VERSION A SUPPLICANT SENDS.
 *
 * One, not two, and not an echo of what the authenticator sent. This is what
 * wpa_supplicant ships as its default, and the reason is compatibility: the
 * octet is inside the MIC'd region, some access points have historically
 * misbehaved on version 2 from a station, and there is no upside to claiming
 * a higher number - the field is not a negotiation. Echoing the
 * authenticator's version would send 2 against hostapd; the captured
 * wpa_supplicant message 2 carries 1, and test_against_a_real_four_way
 * compares ours with it byte for byte. */
inline constexpr uint8_t kEapolVersionSupplicant = 1;

/* A parsed EAPOL-Key frame. The pointers alias the caller's buffer and are
 * valid only as long as it is. */
struct EapolKey {
  const uint8_t* frame = nullptr;  /* the whole EAPOL frame, from byte 0 */
  size_t frame_len = 0;            /* its true length, key data included */
  uint8_t descriptor = 0;
  uint16_t key_info = 0;
  uint16_t version = 0;
  uint16_t key_len = 0;
  uint64_t replay = 0;
  const uint8_t* nonce = nullptr;  /* 32 bytes */
  const uint8_t* rsc = nullptr;    /* 8 bytes */
  const uint8_t* mic = nullptr;    /* 16 bytes */
  const uint8_t* key_data = nullptr;
  size_t key_data_len = 0;

  bool pairwise() const { return (key_info & kKiPairwise) != 0; }
  bool install() const { return (key_info & kKiInstall) != 0; }
  bool ack() const { return (key_info & kKiAck) != 0; }
  bool has_mic() const { return (key_info & kKiMic) != 0; }
  bool secure() const { return (key_info & kKiSecure) != 0; }
  bool error() const { return (key_info & kKiError) != 0; }
  bool request() const { return (key_info & kKiRequest) != 0; }
  bool encrypted() const { return (key_info & kKiEncrypted) != 0; }
  uint8_t key_id() const { return (uint8_t)((key_info & kKiKeyIdMask) >> 4); }
};

inline uint16_t eapol_be16(const uint8_t* p) {
  return (uint16_t)((p[0] << 8) | p[1]);
}

inline uint64_t eapol_be64(const uint8_t* p) {
  uint64_t v = 0;
  for (int i = 0; i < 8; i++) v = (v << 8) | p[i];
  return v;
}

inline void eapol_put_be64(uint8_t* p, uint64_t v) {
  for (int i = 0; i < 8; i++) p[i] = (uint8_t)((v >> (8 * (7 - i))) & 0xff);
}

/* Is this EAPOL packet an EAPOL-Key (802.1X packet type 3)? Only the type
 * octet is read, so this answers "should the key handshake see this at all"
 * without judging whether the frame is well formed - EAP, EAPOL-Start and
 * EAPOL-Logoff share the ethertype and are not the supplicant's to consume.
 * False when the type octet is not even present. */
inline constexpr uint8_t kEapolTypeKey = 3;
inline bool eapol_is_key(const uint8_t* eapol, size_t len) {
  return eapol && len >= 2 && eapol[1] == kEapolTypeKey;
}

/* Parse an EAPOL frame (starting at the 802.1X version octet, i.e. what
 * follows the LLC/SNAP header of an 0x888E data frame).
 *
 * Everything is bounds-checked against `len` because this arrives from the
 * air before anything has authenticated it. In particular the declared key
 * data length is checked against what is actually present: a frame claiming
 * 4096 bytes of key data in a 99-byte buffer is the first thing an attacker
 * tries, and the GTK is read out of that region.
 *
 * `len` may be longer than the frame (a padded MSDU); the 802.1X body length
 * is authoritative and is what bounds the parse. A body length longer than
 * the buffer is refused rather than clamped - clamping would let a truncated
 * frame's MIC be computed over fewer bytes than the sender signed.
 */
inline bool parse_eapol_key(const uint8_t* eapol, size_t len, EapolKey* out) {
  size_t body_len, total, kdlen;

  if (!eapol || !out || len < kEapolKeyFixedLen) return false;
  if (eapol[1] != 3) return false;             /* packet type: EAPOL-Key */
  body_len = eapol_be16(eapol + 2);
  /* The body starts at offset 4; the fixed part is 95 bytes of body. */
  if (body_len < kEapolKeyFixedLen - 4) return false;
  total = 4 + body_len;
  if (total > len) return false;

  if (eapol[4] != kKeyDescTypeRsn) return false;
  kdlen = eapol_be16(eapol + kEapolKeyDataLenOff);
  if (kEapolKeyFixedLen + kdlen != total) return false;

  *out = EapolKey{};
  out->frame = eapol;
  out->frame_len = total;
  out->descriptor = eapol[4];
  out->key_info = eapol_be16(eapol + 5);
  out->version = (uint16_t)(out->key_info & kKiVersionMask);
  out->key_len = eapol_be16(eapol + 7);
  out->replay = eapol_be64(eapol + kEapolReplayOff);
  out->nonce = eapol + kEapolNonceOff;
  out->rsc = eapol + kEapolRscOff;
  out->mic = eapol + kEapolMicOff;
  out->key_data = kdlen ? eapol + kEapolKeyFixedLen : nullptr;
  out->key_data_len = kdlen;
  return true;
}

/* Build an EAPOL-Key frame. `mic_kck` non-null sets the MIC over the finished
 * frame with the MIC field zeroed, which is the only order that works: the
 * MIC covers the key data and the key data length, so nothing may be appended
 * afterwards.
 *
 * `proto_version` IS THE 802.1X VERSION OCTET, and it is an argument because
 * it sits inside the MIC'd region and the two reference implementations do
 * not agree on it: hostapd sends 2 and wpa_supplicant sends 1, in the same
 * exchange, and each accepts the other. It is "the highest version the sender
 * supports", not a negotiation, so neither is wrong.
 *
 * The default is 2 because the authenticators in this tree send 2. A
 * supplicant should send kEapolVersionSupplicant - see the note there.
 *
 * Returns an EMPTY vector, rather than a truncated frame, when the key data
 * would not fit: the body length and the key data length are 16-bit fields,
 * and writing their low 16 bits would produce a frame whose declared lengths
 * disagree with its bytes - under a MIC that then signs the lie.
 *
 * EMPTY TOO for two inconsistent requests: a nonzero `key_data_len` with no
 * `key_data` (the frame would declare bytes it does not carry), and a
 * `mic_kck` with no `crypto` to compute the MIC (the frame would go out
 * unsigned though the caller asked for a MIC). Both null is the valid
 * unsigned frame, which message 1 is. */
inline std::vector<uint8_t> build_eapol_key(uint16_t key_info, uint16_t key_len,
                                            uint64_t replay,
                                            const uint8_t nonce[32],
                                            const uint8_t rsc[8],
                                            const uint8_t* key_data,
                                            size_t key_data_len,
                                            CryptoOps* crypto,
                                            const uint8_t* mic_kck,
                                            uint8_t proto_version = 2) {
  if (key_data_len > 0xffffu - (kEapolKeyFixedLen - 4)) return {};
  if (key_data_len != 0 && !key_data) return {};
  if (mic_kck && !crypto) return {};
  std::vector<uint8_t> e(kEapolKeyFixedLen, 0);

  e[0] = proto_version;                        /* 802.1X version */
  e[1] = 3;                                    /* EAPOL-Key */
  const size_t body = kEapolKeyFixedLen - 4 + key_data_len;
  e[2] = (uint8_t)((body >> 8) & 0xff);
  e[3] = (uint8_t)(body & 0xff);
  e[4] = kKeyDescTypeRsn;
  e[5] = (uint8_t)(key_info >> 8);
  e[6] = (uint8_t)(key_info & 0xff);
  e[7] = (uint8_t)(key_len >> 8);
  e[8] = (uint8_t)(key_len & 0xff);
  eapol_put_be64(e.data() + kEapolReplayOff, replay);
  if (nonce) std::memcpy(e.data() + kEapolNonceOff, nonce, 32);
  if (rsc) std::memcpy(e.data() + kEapolRscOff, rsc, 8);
  e[kEapolKeyDataLenOff] = (uint8_t)((key_data_len >> 8) & 0xff);
  e[kEapolKeyDataLenOff + 1] = (uint8_t)(key_data_len & 0xff);
  if (key_data && key_data_len)
    e.insert(e.end(), key_data, key_data + key_data_len);
  if (crypto && mic_kck) {
    uint8_t d[20];

    std::memset(e.data() + kEapolMicOff, 0, kEapolMicLen);
    if (crypto->hmac_sha1(mic_kck, 16, e.data(), e.size(), d))
      std::memcpy(e.data() + kEapolMicOff, d, kEapolMicLen);
    else
      e.clear();                               /* refuse, do not ship unsigned */
  }
  return e;
}

/* Verify an EAPOL-Key MIC with the KCK.
 *
 * THE COMPARISON IS THE POINT. It is done over a copy with the MIC field
 * zeroed - the field is part of the signed region, so it has to be removed
 * before the HMAC, and doing that in place would write through a pointer into
 * a received frame. The result is compared with a constant-time reduction:
 * this runs against attacker-supplied input, and an early-exit memcmp over a
 * MAC is the textbook way to hand out a forgery oracle.
 *
 * A wrong descriptor version fails here rather than being tolerated, because
 * version 1 and 3 use different MIC algorithms entirely.
 *
 * THREE OUTCOMES, NOT TWO. `Mismatch` is the frame's fault: a MIC that does
 * not verify, or a frame that cannot carry a version-2 MIC at all.
 * `CryptoError` is ours: the HMAC provider failed, so nothing is known about
 * the frame. A caller that folds the two together reports its own provider
 * failing as a forgery.
 */
enum class MicCheck : uint8_t { Ok, Mismatch, CryptoError };

inline MicCheck eapol_mic_ok(CryptoOps& crypto, const uint8_t kck[16],
                             const EapolKey& k) {
  uint8_t got[kEapolMicLen], want[20];
  uint8_t diff = 0;

  /* Checked BEFORE the copy: an EapolKey that did not come from
   * parse_eapol_key may carry a null frame or a short length. */
  if (!k.frame || k.frame_len < kEapolKeyFixedLen) return MicCheck::Mismatch;
  if (k.version != kKeyDescVersionCcmp) return MicCheck::Mismatch;
  if (!k.has_mic()) return MicCheck::Mismatch;
  std::vector<uint8_t> copy(k.frame, k.frame + k.frame_len);
  std::memcpy(got, k.frame + kEapolMicOff, kEapolMicLen);
  std::memset(copy.data() + kEapolMicOff, 0, kEapolMicLen);
  if (!crypto.hmac_sha1(kck, 16, copy.data(), copy.size(), want))
    return MicCheck::CryptoError;
  for (size_t i = 0; i < kEapolMicLen; i++) diff |= (uint8_t)(got[i] ^ want[i]);
  return diff == 0 ? MicCheck::Ok : MicCheck::Mismatch;
}

/* The 802.11 PRF built on HMAC-SHA1 (802.11-2016 12.7.1.2). `olen` bytes are
 * produced 20 at a time; the label's terminating NUL is part of the input. */
inline bool prf_sha1(CryptoOps& crypto, const uint8_t* key, size_t key_len,
                     const char* label, const uint8_t* data, size_t data_len,
                     uint8_t* out, size_t olen) {
  const size_t ll = std::strlen(label);
  std::vector<uint8_t> buf(ll + 1 + data_len + 1);

  std::memcpy(buf.data(), label, ll);
  buf[ll] = 0;
  if (data_len) std::memcpy(buf.data() + ll + 1, data, data_len);
  for (size_t gen = 0, i = 0; gen < olen; gen += 20, i++) {
    uint8_t d[20];
    const size_t take = (olen - gen < 20) ? olen - gen : 20;

    buf[ll + 1 + data_len] = (uint8_t)i;
    if (!crypto.hmac_sha1(key, key_len, buf.data(), buf.size(), d))
      return false;
    std::memcpy(out + gen, d, take);
  }
  return true;
}

/* PMK = PBKDF2(passphrase, SSID, 4096, 32). The SSID is the salt, which is
 * why two networks with the same passphrase and different names do not share
 * a PMK.
 *
 * THE PSK HAS TWO SPELLINGS (802.11-2016 J.4.1), as hostapd's wpa_psk= and
 * wpa_supplicant's psk= both accept: a passphrase of 8..63 characters, run
 * through PBKDF2, or EXACTLY 64 hex digits, which ARE the PMK and are
 * decoded, not hashed. Anything else is refused here. Hashing a 64-hex PSK
 * as though it were a passphrase produces a PMK nothing else derives, whose
 * only symptom is MIC failures and a handshake timeout; refusing here turns
 * a malformed PSK into a configuration failure at the point it is given. */
inline void secure_wipe(void* p, size_t n);   /* defined below */
inline bool pmk_from_psk(CryptoOps& crypto, const char* passphrase,
                         const std::string& ssid, uint8_t pmk[32]) {
  if (!passphrase || ssid.empty() || ssid.size() > 32) return false;
  const size_t n = std::strlen(passphrase);
  if (n == 64) {
    uint8_t raw[32];
    for (size_t i = 0; i < 64; i++) {
      const char c = passphrase[i];
      int v;
      if (c >= '0' && c <= '9') v = c - '0';
      else if (c >= 'a' && c <= 'f') v = c - 'a' + 10;
      else if (c >= 'A' && c <= 'F') v = c - 'A' + 10;
      else { secure_wipe(raw, sizeof raw); return false; }
      if (i % 2 == 0) raw[i / 2] = (uint8_t)(v << 4);
      else raw[i / 2] = (uint8_t)(raw[i / 2] | v);
    }
    std::memcpy(pmk, raw, 32);
    secure_wipe(raw, sizeof raw);
    return true;
  }
  if (n < 8 || n > 63) return false;
  return crypto.pbkdf2_sha1(passphrase, (const uint8_t*)ssid.data(),
                            ssid.size(), 4096, pmk, 32);
}

/* PTK = PRF-384(PMK, "Pairwise key expansion",
 *               min(AA,SPA) || max(AA,SPA) || min(ANonce,SNonce) || max(...))
 *
 * THE SORTING IS NOT DECORATION. Both ends derive the same key only because
 * each orders the pair the same way, and each end knows the addresses and
 * nonces by different names - the authenticator's "own" is the supplicant's
 * "peer". Sorting removes the asymmetry. Getting it wrong produces a PTK that
 * works against nothing, and the only symptom is a MIC failure.
 *
 * Layout: KCK[0:16] KEK[16:32] TK[32:48].
 */
inline bool derive_ptk(CryptoOps& crypto, const uint8_t pmk[32],
                       const uint8_t aa[6], const uint8_t spa[6],
                       const uint8_t anonce[32], const uint8_t snonce[32],
                       uint8_t ptk[48]) {
  uint8_t b[76];
  const uint8_t* amin = std::memcmp(aa, spa, 6) < 0 ? aa : spa;
  const uint8_t* amax = std::memcmp(aa, spa, 6) < 0 ? spa : aa;
  const uint8_t* nmin = std::memcmp(anonce, snonce, 32) < 0 ? anonce : snonce;
  const uint8_t* nmax = std::memcmp(anonce, snonce, 32) < 0 ? snonce : anonce;

  std::memcpy(b, amin, 6);
  std::memcpy(b + 6, amax, 6);
  std::memcpy(b + 12, nmin, 32);
  std::memcpy(b + 44, nmax, 32);
  return prf_sha1(crypto, pmk, 32, "Pairwise key expansion", b, sizeof b, ptk,
                  48);
}

/* A GTK lifted out of a key-data KDE. */
struct GtkKde {
  uint8_t key_id = 0;
  bool tx = false;
  uint8_t gtk[32] = {0};
  size_t gtk_len = 0;
};

/* THREE OUTCOMES, NOT TWO. "there is no GTK KDE here" and "a KDE in here is
 * truncated or claims an impossible key length" are completely different
 * facts: the first can be legitimate, the second is hostile input. A single
 * `false` for both invites two callers to read it in opposite ways - one
 * carrying on and completing the handshake, the other refusing the frame -
 * and the first leaves a station `Connected` and keyed with no group key, no
 * counter moved and nothing to diagnose from. */
enum class KdeResult : uint8_t {
  Found,
  Absent,     /* well-formed key data with no GTK KDE in it */
  Malformed,  /* a KDE that runs past the buffer or declares a bad length */
};

/* THE KEY-DATA WALK, the one way this module reads unwrapped key data:
 * find_gtk_kde and Supplicant's RSN-element search both use it, so the two
 * cannot disagree about where an element starts.
 *
 * Key data is a sequence of elements - ID, length, body. The 802.11i padding
 * is 0xDD followed by zeros (802.11-2016 12.7.2), and it walks as elements
 * too: the 0xDD reads as an empty element, each following pair of zeros as an
 * empty element with ID 0, and a single trailing byte ends the walk. A lone
 * 0x00 is NOT skipped as padding - ID 0 is a real element ID. Every step is
 * bounds-checked: this region comes out of an AES unwrap of attacker-supplied
 * bytes.
 *
 * `visit(id, body, body_len)` is called for each element and returns false to
 * stop early. Returns false when an element runs past the end of the buffer,
 * true otherwise. */
template <typename Visit>
inline bool walk_key_data(const uint8_t* kd, size_t len, Visit&& visit) {
  size_t i = 0;

  if (!kd) return len == 0;
  while (i + 2 <= len) {
    const uint8_t id = kd[i];
    const size_t l = kd[i + 1];

    if (i + 2 + l > len) return false;         /* truncated: stop, do not guess */
    if (!visit(id, kd + i + 2, l)) return true;
    i += 2 + l;
  }
  return true;
}

/* The GTK KDE (00-0F-AC type 1) in unwrapped key data, read with
 * walk_key_data. A KDE is `0xDD len 00 0F AC type` followed by its body;
 * every other element is stepped over.
 *
 * Returns Absent when the key data is well formed and simply carries no GTK
 * KDE, and Malformed when something in it does not add up. The caller decides
 * what Absent means for the message it arrived in.
 *
 * THE WHOLE FIELD IS WALKED, not just up to the GTK: key data that goes on to
 * a truncated element does not parse, and a frame whose key data does not
 * parse installs nothing, even with a verified MIC. Any truncation anywhere is
 * Malformed, and so is a second GTK KDE: two group keys in one message is not
 * something to pick between. `out` is only written for Found, and wiped
 * otherwise.
 */
inline KdeResult find_gtk_kde(const uint8_t* kd, size_t len, GtkKde* out) {
  bool found = false, bad = false;

  if (!kd || !out) return KdeResult::Malformed;
  *out = GtkKde{};
  const bool whole = walk_key_data(
      kd, len, [&](uint8_t id, const uint8_t* b, size_t l) {
        if (!(id == 0xdd && l >= 4 && b[0] == 0x00 && b[1] == 0x0f &&
              b[2] == 0xac && b[3] == 0x01))
          return true;                         /* not a GTK KDE: step over */
        /* The KDE length counts OUI(3) + data type(1) + data. The GTK KDE's
         * data is a KeyID/Tx octet, a reserved octet, then the key, so a
         * 16-byte GTK gives a length of 22 - and subtracting 6 rather than 4
         * here would make every real KDE look two bytes short and silently
         * truncate the key. */
        const size_t body = l - 4;             /* keyid/tx + reserved + key */
        if (found || body < 2 + 16 || body > 2 + 32) {
          bad = true;                          /* a second KDE, or a bad length */
          return false;
        }
        out->key_id = (uint8_t)(b[4] & 0x03);
        out->tx = (b[4] & 0x04) != 0;
        out->gtk_len = body - 2;
        std::memcpy(out->gtk, b + 6, out->gtk_len);
        found = true;
        return true;
      });
  if (!whole || bad) {
    secure_wipe(out, sizeof *out);
    return KdeResult::Malformed;
  }
  return found ? KdeResult::Found : KdeResult::Absent;
}

/* Overwrite key material so it does not outlive the object holding it.
 *
 * Through a volatile pointer, because a compiler is entitled to delete a
 * memset whose result is never read - which is every memset in a destructor.
 * Keys are zeroized when their owner lets them go; the Supplicant and
 * StationSm use this for every key they hold.
 */
inline void secure_wipe(void* p, size_t n) {
  volatile uint8_t* v = static_cast<volatile uint8_t*>(p);

  while (n--) *v++ = 0;
}

}  // namespace sta
}  // namespace devourer

#endif /* DEVOURER_STA_EAPOL_H */
