/* Ccmp — 802.11 CCMP framing, role-neutral: the same code serves an AP and a
 * station. In this tree its user is the station.
 *
 * This is the framing only: AAD construction, nonce construction, the 8-byte
 * CCMP header, and the PN. The cipher itself is a CryptoOps call, so nothing
 * here includes a crypto library and `libdevourer` gains no dependency.
 *
 * The same rules sit inline and untested in `tests/ap_wpa2.cpp`; this is the
 * tested form. Each rule is recorded at the function that implements it,
 * because each one fails silently - a wrong byte anywhere looks exactly like
 * a wrong key: the AAD masks the FC subtype/retry/pwr-mgmt/more-data bits and
 * sets Protected, and masks the sequence number while KEEPING the fragment
 * number; the nonce is Flags | A2 | PN as six big-endian bytes, where Flags
 * carries the QoS TID and the management bit; and the CCMP header carries
 * the 48-bit PN split across two discontiguous ranges with an Ext IV bit that
 * is not optional.
 *
 * Both roles use the identical framing — an AP encrypting to a station and a
 * station encrypting to its AP differ only in which address is A2 and which
 * PN counter they draw from. That is the whole reason this is one file rather
 * than two.
 */
#ifndef DEVOURER_STA_CCMP_H
#define DEVOURER_STA_CCMP_H

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "sta/CryptoOps.h"
#include "sta/Dot11.h"

namespace devourer {
namespace sta {

/* An 802.11 CCMP header is 8 bytes and the MIC is 8; a protected MPDU is
 * therefore 16 bytes longer than its plaintext plus the 802.11 header. */
constexpr size_t kCcmpHdrLen = 8;
constexpr size_t kCcmpMicLen = 8;
constexpr size_t kCcmpAadMax = 32;
constexpr size_t kCcmpNonceLen = 13;

/* Additional authenticated data over the 802.11 header (802.11-2016 12.5.3.3.3).
 *
 * Everything is derived from the header itself: `hdr_len` is explicit and
 * the TID comes out of the frame. A separate `qos_tid` argument would let a
 * caller pass a TID that disagrees with the QoS Control field it hands in, and
 * a fixed 24-byte read would let the CCMP header overwrite that field.
 *
 * The masking is the part that is easy to get subtly wrong and impossible to
 * debug from the other end, because a wrong AAD looks exactly like a wrong key:
 * the MIC simply fails.
 *
 *  - Frame Control: Retry, Pwr Mgmt and More Data are masked because they may
 *    legitimately differ between transmission and reception. Protected is
 *    forced ON because the receiver sees it set. The SUBTYPE bits are masked
 *    only for non-management frames - a protected management frame (802.11w)
 *    keeps its subtype in the AAD, which is what mac80211 and hostapd's
 *    wlantest both do.
 *  - Sequence Control: the sequence number is masked, the FRAGMENT number is
 *    kept. Masking both breaks fragmented frames; keeping both breaks every
 *    frame, because the sequence number is assigned after the MIC is computed
 *    on a hardware-sequencing MAC.
 *  - A4 is included for a 4-address frame (ToDS and FromDS both set).
 *  - QoS Control is included for a QoS data frame, with only the TID retained;
 *    the ack-policy, EOSP and A-MSDU bits are masked.
 *
 * Returns the AAD length: 22, +6 for A4, +2 for QoS. Zero if `hdr_len` is too
 * short for what the frame control claims.
 */
inline size_t ccmp_aad(const uint8_t* hdr, size_t hdr_len, uint8_t* aad) {
  bool four_addr, qos, mgmt;
  size_t need, n;
  uint16_t fc;

  /* THE LENGTH CHECK COMES FIRST, before the three reads below: a
   * precondition a function checks after dereferencing is not a
   * precondition. (ccmp_decrypt proves the length before calling this, so no
   * in-tree caller reaches it from the air; a direct caller could.) */
  if (hdr_len < 24) return 0;
  four_addr = (hdr[1] & (kFcToDs | kFcFromDs)) == (kFcToDs | kFcFromDs);
  qos = is_qos_data(hdr[0]);
  mgmt = (hdr[0] & 0x0c) == 0x00; /* type 0 = management */
  need = 24 + (four_addr ? 6 : 0) + (qos ? 2 : 0);
  fc = (uint16_t)(hdr[0] | (hdr[1] << 8));

  if (hdr_len < need) return 0;
  if (!mgmt) fc &= (uint16_t)~0x0070u;             /* subtype: data/control */
  fc &= (uint16_t)~(0x0800u | 0x1000u | 0x2000u);  /* retry, pwr mgmt, more data */
  /* THE ORDER BIT, masked for a QoS data frame and ONLY for one.
   *
   * On a QoS data frame bit 15 means "+HTC: an HT Control field follows", it
   * is set per transmission, and 802.11-2016 12.5.3.3.3 masks it - Linux does
   * the same inside its is_data_qos branch (net/mac80211/wpa.c,
   * "Retry, PwrMgt, MoreData, Order (if Qos Data)"). On a NON-QoS frame the
   * same bit is the strictly-ordered service class and is NOT masked, which
   * is why this is not folded into the line above.
   *
   * Without this mask every +HTC QoS frame fails its MIC - refused, and
   * indistinguishable from a wrong key - and it has to agree with
   * data_hdr_len(), which counts the four HT Control bytes. The devourer AP
   * advertises no HT, so only a real AP sends such frames; the qos_order_htc
   * vector pins it. */
  if (qos) fc &= (uint16_t)~0x8000u;
  fc |= 0x4000u;                                   /* protected */
  aad[0] = (uint8_t)(fc & 0xff);
  aad[1] = (uint8_t)(fc >> 8);
  std::memcpy(aad + 2, hdr + 4, 18);               /* addr1, addr2, addr3 */
  {
    uint16_t seq = (uint16_t)((hdr[22] | (hdr[23] << 8)) & 0x000f);

    aad[20] = (uint8_t)(seq & 0xff);
    aad[21] = (uint8_t)(seq >> 8);
  }
  n = 22;
  if (four_addr) {
    std::memcpy(aad + n, hdr + 24, 6);             /* addr4 */
    n += 6;
  }
  if (qos) {
    /* The QoS Control field sits immediately after the addresses. */
    const size_t qoff = four_addr ? 30 : 24;

    aad[n++] = (uint8_t)(hdr[qoff] & 0x0f);
    aad[n++] = 0;
  }
  return n;
}

/* CCM nonce: the Nonce Flags octet, then A2, then the 48-bit PN BIG-endian
 * (802.11-2016 12.5.3.3.4). Big-endian here and little-endian in the CCMP
 * header below — they genuinely differ, and reusing one for the other is the
 * classic way to produce a frame only your own implementation can read.
 *
 * THE FLAGS OCTET IS NOT ZERO. It is Priority (b0..b3) | Management (b4),
 * where Priority is the QoS TID for a QoS data frame and 0 otherwise. Linux
 * builds the same byte as `qos_tid | (ieee80211_is_mgmt(fc) << 4)`
 * (net/mac80211/wpa.c).
 *
 * A zero flags octet is correct only for TID 0, which is ordinary client
 * traffic - so the mistake is invisible until TID 1..7, the video and voice
 * access categories, and a same-author vector generator shares it. The TID
 * must reach BOTH the AAD (ccmp_aad) and this octet; the kernel-captured
 * vectors at all eight TIDs pin both (test_kernel_vectors, test_nonce_flags).
 *
 * Takes the header rather than a priority argument on purpose: ccmp_aad
 * derives the same facts from the same bytes, and a caller that had to pass
 * the TID separately is a caller that can pass a different one to each.
 *
 * Returns false if `hdr_len` is too short for what the frame control claims,
 * matching ccmp_aad's zero return. */
inline bool ccmp_nonce(const uint8_t* hdr, size_t hdr_len,
                       const uint8_t a2[6], uint64_t pn, uint8_t* nonce) {
  bool four_addr, qos, mgmt;
  size_t need;
  uint8_t flags = 0;

  /* Length first, for the reason ccmp_aad gives. */
  if (hdr_len < 24) return false;
  four_addr = (hdr[1] & (kFcToDs | kFcFromDs)) == (kFcToDs | kFcFromDs);
  qos = is_qos_data(hdr[0]);
  mgmt = (hdr[0] & 0x0c) == 0x00; /* type 0 = management */
  need = 24 + (four_addr ? 6 : 0) + (qos ? 2 : 0);

  if (hdr_len < need) return false;
  if (qos) flags = (uint8_t)(hdr[four_addr ? 30 : 24] & 0x0f);
  if (mgmt) flags |= 0x10;

  nonce[0] = flags;
  std::memcpy(nonce + 1, a2, 6);
  for (int i = 0; i < 6; i++)
    nonce[7 + i] = (uint8_t)((pn >> (8 * (5 - i))) & 0xff);
  return true;
}

/* The 8-byte CCMP header: PN0, PN1, a reserved byte, then the key-id octet
 * with Ext IV set, then PN2..PN5 little-endian. The PN is deliberately NOT
 * contiguous - byte 2 is reserved and byte 3 is not part of the PN at all. */
inline void ccmp_header(uint64_t pn, uint8_t key_id, uint8_t* out) {
  out[0] = (uint8_t)(pn & 0xff);
  out[1] = (uint8_t)((pn >> 8) & 0xff);
  out[2] = 0;
  out[3] = (uint8_t)(0x20 | ((key_id & 0x03) << 6)); /* Ext IV | KeyID */
  out[4] = (uint8_t)((pn >> 16) & 0xff);
  out[5] = (uint8_t)((pn >> 24) & 0xff);
  out[6] = (uint8_t)((pn >> 32) & 0xff);
  out[7] = (uint8_t)((pn >> 40) & 0xff);
}

/* Recover the 48-bit PN from a received CCMP header. */
inline uint64_t ccmp_header_pn(const uint8_t* ccmp_hdr) {
  return (uint64_t)ccmp_hdr[0] | ((uint64_t)ccmp_hdr[1] << 8) |
         ((uint64_t)ccmp_hdr[4] << 16) | ((uint64_t)ccmp_hdr[5] << 24) |
         ((uint64_t)ccmp_hdr[6] << 32) | ((uint64_t)ccmp_hdr[7] << 40);
}

/* Which key this frame was protected with: the top two bits of the CCMP
 * header's fourth octet (802.11-2016 12.5.3.2).
 *
 * A RECEIVER MUST READ THIS AND NOT INFER IT. The obvious inference - group
 * address means the group key - is wrong in both directions: an AP may
 * unicast under the GTK during a rekey, and a station that guesses by address
 * fails the MIC and reports it as an attack. The mirror bug on the transmit
 * side costs a whole BSS's broadcast traffic: a group frame sent at key id 0
 * is looked up as the PAIRWISE key at the station and fails its MIC with no
 * diagnostic at either end. */
inline uint8_t ccmp_key_id(const uint8_t* ccmp_hdr) {
  return (uint8_t)((ccmp_hdr[3] >> 6) & 0x03);
}

/* Protect one MPDU.
 *
 * `hdr` is the frame header and `hdr_len` its true length - 24, 26 for QoS, 30
 * for 4-address, 32 for both. `a2` is the transmitter address the nonce is
 * built from, passed separately rather than read from the header because a
 * 4-address frame puts it elsewhere and silently reading the wrong one would
 * produce frames that only decrypt locally.
 *
 * Writes hdr_len + 8 + plain_len + 8 bytes to `out`. `out_cap` is that
 * buffer's size and is CHECKED: without it a caller who sizes `out` wrong gets
 * a heap overflow with no diagnostic, in a function whose whole job is
 * handling frames from the air. `ccmp_encrypted_len()` computes the size to
 * allocate, and a total that does not fit in size_t is refused rather than
 * wrapped into a small number that a small buffer would pass.
 *
 * Returns the length written, or 0 on failure (including a short buffer).
 *
 * THE PN IS 48 BITS, and a larger one is refused before anything is written.
 * The nonce and the CCMP header each carry only the low 48 bits, so PN 2^48
 * would air with the same nonce as PN 0 under the same TK - keystream reuse.
 * A transmitter that reaches 2^48-1 has to rekey: 802.11 requires that a PN
 * never repeat under one temporal key.
 */
inline constexpr uint64_t kCcmpPnMax = 0xffffffffffffull;

/* The protected MPDU's length: hdr_len + 8 + plain_len + 8. ZERO means
 * IMPOSSIBLE - the sum does not fit in size_t. Zero is never a real result
 * (the CCMP overhead alone is 16), and a buffer sized from it is one
 * ccmp_encrypt refuses, where a wrapped sum would pass its capacity check. */
inline size_t ccmp_encrypted_len(size_t hdr_len, size_t plain_len) {
  const size_t overhead = kCcmpHdrLen + kCcmpMicLen;
  const size_t max = SIZE_MAX;
  if (hdr_len > max - overhead || plain_len > max - overhead - hdr_len)
    return 0;
  return hdr_len + overhead + plain_len;
}

/* How many plaintext bytes ccmp_decrypt can write for an MPDU of this size,
 * so a caller can size its buffer instead of guessing. Zero when the frame is
 * too short to hold a header, a CCMP header and a MIC. Computed by
 * subtraction, so no header length can wrap it. */
inline size_t ccmp_decrypted_len(size_t mpdu_len, size_t hdr_len) {
  const size_t overhead = kCcmpHdrLen + kCcmpMicLen;
  if (hdr_len >= mpdu_len || mpdu_len - hdr_len <= overhead) return 0;
  return mpdu_len - hdr_len - overhead;
}

inline size_t ccmp_encrypt(CryptoOps& crypto, const uint8_t tk[16],
                           const uint8_t* hdr, size_t hdr_len,
                           const uint8_t a2[6], uint64_t pn, uint8_t key_id,
                           const uint8_t* plain, size_t plain_len,
                           uint8_t* out, size_t out_cap) {
  uint8_t aad[kCcmpAadMax];
  uint8_t nonce[kCcmpNonceLen];
  size_t aad_len;
  const size_t need = ccmp_encrypted_len(hdr_len, plain_len);

  if (pn > kCcmpPnMax) return 0;
  if (need == 0 || !out || out_cap < need) return 0;
  aad_len = ccmp_aad(hdr, hdr_len, aad);
  if (aad_len == 0) return 0;
  if (!ccmp_nonce(hdr, hdr_len, a2, pn, nonce)) return 0;
  std::memcpy(out, hdr, hdr_len);
  out[1] |= 0x40; /* Protected, in the frame we actually air */
  ccmp_header(pn, key_id, out + hdr_len);
  if (!crypto.aes_ccm(true, tk, nonce, aad, aad_len, plain, plain_len,
                      out + hdr_len + kCcmpHdrLen,
                      out + hdr_len + kCcmpHdrLen + plain_len))
    return 0;
  return need;
}

/* Unprotect one MPDU. `mpdu` is the whole received frame starting at the
 * 802.11 header, WITHOUT the FCS, and `hdr_len` is its header length
 * (data_hdr_len() computes it).
 *
 * Returns false when the frame is too short, when `out_cap` is too small, or
 * when the MIC does not verify. A false return means DROP: `out` holds no
 * trustworthy bytes, and the PN must not be admitted to a replay window.
 * Replay checking itself is the caller's - it needs per-TID state this
 * function does not own.
 *
 * `out_cap` IS NOT OPTIONAL, for the same reason ccmp_encrypt takes one: the
 * plaintext length comes from the MPDU, which comes from the air. Without it
 * a caller with a fixed-size buffer is one full-MTU frame away from a stack
 * smash, and the cipher writes the plaintext out BEFORE the tag is checked,
 * so a forged frame is enough - while short test vectors never trigger it.
 * ccmp_decrypted_len() computes the size to allocate.
 *
 * A NULL `out` IS REFUSED for a non-empty body, and never reaches the cipher
 * for an empty one. OpenSSL's CCM reads a NULL output pointer as "this is
 * AAD", so a decrypt handed one returns success with NO TAG CHECK - and the
 * natural idiom `std::vector<uint8_t> plain(ccmp_decrypted_len(...))` hands
 * over exactly that (`data()` of an empty vector) for a forged zero-body
 * frame, whose attacker-chosen PN would then move the replay window. A
 * zero-length body is decrypted into a local scratch byte instead, so its
 * tag is still checked.
 *
 * THE FRAME MUST SAY IT IS PROTECTED (Frame Control bit 14, fc[1] & 0x40).
 * ccmp_aad forces that bit to 1 in the AAD, so a received frame with it
 * cleared would otherwise still verify - and a frame that claims to be
 * unprotected must not come out of here as authenticated plaintext.
 *
 * THE EXT IV BIT MUST BE SET (bit 5 of the key-id octet). CCMP always uses
 * the extended IV (802.11-2016 12.5.3.2); a header without it is not a CCMP
 * header, and since that octet is outside the AAD the MIC alone would not
 * notice. The reserved bits of that octet (0-4) and the reserved octet 2 are
 * NOT checked: 802.11's convention is that reserved fields are set to 0 on
 * transmission and ignored on reception (9.2.2).
 */
inline bool ccmp_decrypt(CryptoOps& crypto, const uint8_t tk[16],
                         const uint8_t* mpdu, size_t mpdu_len, size_t hdr_len,
                         const uint8_t a2[6], uint8_t* out, size_t out_cap,
                         size_t* out_len, uint64_t* pn_out) {
  const size_t overhead = kCcmpHdrLen + kCcmpMicLen;
  uint8_t aad[kCcmpAadMax];
  uint8_t nonce[kCcmpNonceLen];
  /* The tag is COPIED rather than passed by casting away const on the input.
   * A CryptoOps that writes the computed tag into the buffer before comparing
   * - a perfectly normal implementation shape - would otherwise write through
   * a pointer into the caller's frame, or into .rodata for a static test
   * vector. */
  uint8_t tag[kCcmpMicLen];
  uint8_t scratch[1];
  size_t aad_len, body;
  uint64_t pn;

  /* By subtraction, before any indexing: hdr_len comes from the caller and a
   * sum with it could wrap past a short frame. */
  if (hdr_len > mpdu_len || mpdu_len - hdr_len < overhead) return false;
  body = mpdu_len - hdr_len - overhead;
  if (out_cap < body) return false;
  if (body > 0 && !out) return false;
  if (body == 0) out = scratch;
  if (!(mpdu[1] & kFcProtected)) return false;     /* Protected */
  if (!(mpdu[hdr_len + 3] & 0x20)) return false;   /* Ext IV */
  pn = ccmp_header_pn(mpdu + hdr_len);
  aad_len = ccmp_aad(mpdu, hdr_len, aad);
  if (aad_len == 0) return false;
  if (!ccmp_nonce(mpdu, hdr_len, a2, pn, nonce)) return false;
  std::memcpy(tag, mpdu + hdr_len + kCcmpHdrLen + body, kCcmpMicLen);
  if (!crypto.aes_ccm(false, tk, nonce, aad, aad_len,
                      mpdu + hdr_len + kCcmpHdrLen, body, out, tag))
    return false;
  if (out_len) *out_len = body;
  if (pn_out) *pn_out = pn;
  return true;
}

/* CCMP replay protection: a sliding window per TID, not a bare counter.
 *
 * ONE WINDOW PER TID, not one per key. 802.11 keeps a separate receive replay
 * counter for each TID of QoS traffic, and a single shared one drops
 * legitimate frames as soon as two TIDs interleave - routine the moment voice
 * or video shares a link with best-effort. Non-QoS traffic uses a window of
 * its own (index kNonQosTid).
 *
 * An EQUAL PN is a replay and must be refused: admitting an equal-counter
 * replay is exactly how a key-reinstallation (KRACK-class) attack lands.
 * Here that is bit 0 of the mask, which is always set for the head, so
 * `pn == last_` can never be re-admitted (test_replay).
 *
 * 802.11-2016 12.5.3.4.4 requires a receiver to discard an MPDU whose PN is
 * not greater than the replay counter for its TID. Implemented as a strict
 * `pn > last` test that is correct ONLY while frames cannot arrive out of
 * order - and that is an assumption about the peer, not about the standard.
 * The moment a BlockAck agreement exists, an A-MPDU can deliver
 * PN 5, 7, 6 legitimately, and a strict counter drops frame 6 as a replay:
 * silent, unattributable data loss that looks like a radio problem.
 *
 * This tree's AP harnesses never set up a BlockAck agreement, so a strict
 * counter is safe against them. A STATION does not get to choose - it
 * associates with whatever the AP offers - so the assumption has to go before
 * a station relies on it.
 *
 * The window is the standard anti-replay bitmap (the shape IPsec RFC 4303
 * appendix B and every 802.11 driver use): `last_` is the highest PN
 * accepted, and bit i of `mask_` records whether PN (last_ - i) has already
 * been seen. So a frame is accepted exactly once whether it arrives early,
 * on time, or late but still inside the window.
 *
 * kWindow = 64 because that is IEEE80211_MAX_AMPDU_BUF_HT: the largest
 * BlockAck buffer an HT or VHT peer can negotiate, so anything such a reorder
 * buffer can legitimately hold fits. A PN older than that is not reordering -
 * it is a replay, or a peer that has lost its way - and is refused.
 *
 * THAT BOUND IS NOT UNIVERSAL. The kernel's own constants are HT 0x40, HE
 * 0x100, EHT 0x400
 * (linux/ieee80211.h). So on an 802.11ax die a peer may negotiate a 256-frame
 * reorder buffer and a frame 64..255 behind would be refused here - the exact
 * silent loss this window exists to prevent, one generation up. It is left at
 * 64 because the station role is MT7612U-first and that part is VHT, where 64
 * IS the bound; widening it means a multi-word mask, which is real complexity
 * for a die this code does not yet run on. Anyone porting the station role to
 * Kestrel must revisit this constant first.
 *
 * What this deliberately does NOT do: tolerate a PN that jumps forward and
 * then asks for the skipped values later beyond the window. A forward jump of
 * more than 64 clears the mask, so the skipped PNs can never be accepted
 * afterwards. That is the safe direction - an attacker who can inject one
 * frame with a huge PN can deny the window, but cannot replay anything. */
class CcmpReplay {
public:
  static constexpr int kNonQosTid = 16;
  static constexpr int kSlots = 17;
  static constexpr int kWindow = 64;

  /* True when this PN is acceptable AND records it. A rejected PN leaves the
   * window untouched - accepting a frame's PN before its MIC verifies would
   * let an attacker advance the window with garbage, locking out the real
   * peer. Call this only after a successful decrypt.
   *
   * PN 0 is never valid: a CCMP transmitter starts at 1, so a frame claiming
   * 0 is malformed or forged. */
  bool accept(uint64_t pn, int tid = kNonQosTid) {
    if (pn == 0) return false;
    if (tid < 0 || tid >= kSlots) return false;

    if (!seen_[tid]) {
      seen_[tid] = true;
      last_[tid] = pn;
      mask_[tid] = 1;                       /* bit 0 == last_ itself */
      return true;
    }
    if (pn > last_[tid]) {
      const uint64_t shift = pn - last_[tid];
      /* A jump of kWindow or more leaves nothing in the old window
       * reachable, and shifting a uint64_t by >= 64 is undefined - which is
       * exactly the kind of gap a hostile peer would aim for. */
      mask_[tid] = (shift >= (uint64_t)kWindow) ? 1u
                                                : ((mask_[tid] << shift) | 1u);
      last_[tid] = pn;
      return true;
    }
    const uint64_t behind = last_[tid] - pn;
    if (behind >= (uint64_t)kWindow) return false;   /* too old to judge */
    const uint64_t bit = 1ull << behind;
    if (mask_[tid] & bit) return false;              /* already seen */
    mask_[tid] |= bit;
    return true;
  }

  /* Start every window at a KNOWN head: after seed(rsc) only a PN strictly
   * greater than `rsc` is accepted, on every TID. This is how a group key
   * is installed - 802.11-2016 12.7.6.4 hands the receiver the GTK's current
   * receive sequence counter (the Key RSC field) precisely so that the first
   * group frame it accepts is not simply whichever arrives first, which
   * would let a capture from earlier in the key's life in. The whole mask is
   * marked seen, so nothing at or below the seed gets in through the window
   * either. seed(0) is reset(). */
  void seed(uint64_t pn) {
    for (int i = 0; i < kSlots; i++) {
      last_[i] = pn;
      mask_[i] = pn ? ~0ull : 0;
      seen_[i] = pn != 0;
    }
  }

  /* Every rekey resets every counter: a new key means a new PN space. */
  void reset() {
    for (int i = 0; i < kSlots; i++) {
      last_[i] = 0;
      mask_[i] = 0;
      seen_[i] = false;
    }
  }
  uint64_t last(int tid = kNonQosTid) const {
    return (tid >= 0 && tid < kSlots) ? last_[tid] : 0;
  }

private:
  uint64_t last_[kSlots] = {0};
  uint64_t mask_[kSlots] = {0};
  bool seen_[kSlots] = {false};
};

}  // namespace sta
}  // namespace devourer

#endif /* DEVOURER_STA_CCMP_H */
