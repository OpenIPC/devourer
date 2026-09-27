/* Headless guard for src/sta/Ccmp.h — the role-neutral 802.11 CCMP framing.
 *
 * WHAT THE GENERATED VECTORS PIN: the CIPHER PLUMBING, against a third
 * implementation - python-cryptography's AESCCM against OpenSSL. They do NOT
 * pin the 802.11 framing rules. tests/ccmp_gen_vectors.py transcribes the
 * framing from the same reading of the standard as src/sta/Ccmp.h, so a
 * misreading made once is made twice and the vectors agree with it (a zero
 * CCM nonce Flags octet passes them). Independence of IMPLEMENTATION is not
 * independence of INTERPRETATION. The framing rules are pinned by the direct
 * assertion cells below - test_nonce_flags, test_qos_aad, test_aad_masking -
 * and by test_kernel_vectors().
 *
 * test_kernel_vectors() is the one cell here whose expected values were not
 * written by anyone reading the standard. They are frames the LINUX KERNEL
 * encrypted, captured off a mac80211_hwsim rig running hostapd with WMM on,
 * so all eight TIDs appear — the case the devourer AP cannot produce on the
 * bench, because it advertises neither WMM nor HT. See ccmp_kernel_vectors.h.
 *
 * WHAT IT IS NOT. Not the official IEEE Annex J vector; mac80211 is an
 * interop reference, not the specification, and if the kernel and this header
 * misread the same clause in the same way, no cell in this file would notice.
 * That is a smaller risk than it sounds — mac80211 interoperates with every
 * commercial AP — but it is not zero, and Annex J remains a drop-in upgrade.
 * And it is not a round-trip: hand-rolled crypto with no known-answer test is
 * exactly what a round-trip cannot catch, because an implementation that is
 * wrong in both directions round-trips perfectly.
 *
 * The cipher here is OpenSSL, via tests/ccmp_software.h - the same primitive
 * tests/openssl_crypto_ops.h hands the supplicant and state-machine cells, so
 * those run on a cipher these vectors have pinned.
 */
#include <openssl/evp.h>

#include <cstdio>
#include <cstring>
#include <vector>

#include "ccmp_kernel_vectors.h"
#include "ccmp_software.h"
#include "ccmp_vectors.h"
#include "sta/Ccmp.h"

namespace {

int g_fail = 0;

void check(bool ok, const char* what) {
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    g_fail++;
  }
}

/* check(), but it says whether it passed, so a cell can stop working on a
 * vector whose decrypt already failed instead of asserting on garbage. */
bool checked(bool ok, const char* what) {
  check(ok, what);
  return ok;
}

/* The OpenSSL CryptoOps the harnesses will supply. Only aes_ccm is exercised
 * here; the rest refuse rather than pretend, so a future test that needs them
 * fails loudly instead of silently passing on a stub. */
struct OpenSslCcm : devourer::sta::CryptoOps {
  bool aes_ccm(bool encrypt, const uint8_t key[16], const uint8_t nonce[13],
               const uint8_t* aad, size_t aad_len, const uint8_t* in,
               size_t in_len, uint8_t* out, uint8_t* tag) override {
    return devourer::test::ccmp_software(encrypt, key, nonce, aad, (int)aad_len,
                                         in, (int)in_len, out, tag);
  }
  bool hmac_sha1(const uint8_t*, size_t, const uint8_t*, size_t,
                 uint8_t[20]) override {
    return false;
  }
  bool pbkdf2_sha1(const char*, const uint8_t*, size_t, unsigned, uint8_t*,
                   size_t) override {
    return false;
  }
  bool aes_key_unwrap(const uint8_t*, size_t, const uint8_t*, size_t,
                      uint8_t*) override {
    return false;
  }
};

void test_vectors() {
  OpenSslCcm crypto;

  for (size_t i = 0; i < kCcmpVectorCount; i++) {
    const CcmpVector& v = kCcmpVectors[i];
    std::vector<uint8_t> out(v.mpdu_len + 64, 0);
    char label[128];

    std::snprintf(label, sizeof label, "encrypt vector '%s'", v.name);
    size_t n = devourer::sta::ccmp_encrypt(crypto, v.tk, v.hdr, v.hdr_len,
                                           v.a2, v.pn, v.key_id, v.plain,
                                           v.plain_len, out.data(),
                                           out.size());
    {
      uint8_t aad[devourer::sta::kCcmpAadMax];
      check(devourer::sta::ccmp_aad(v.hdr, v.hdr_len, aad) == v.aad_len,
            label); /* 22 / +6 four-address / +2 QoS */
    }
    check(n == v.mpdu_len, label);
    if (n == v.mpdu_len)
      check(std::memcmp(out.data(), v.mpdu, n) == 0, label);

    /* Decrypt the vector's own bytes, not the ones we just produced — a
     * mutual-agreement test between our two directions would pass even if both
     * disagreed with the standard. */
    std::vector<uint8_t> mpdu(v.mpdu, v.mpdu + v.mpdu_len);
    std::vector<uint8_t> plain(v.mpdu_len, 0);
    size_t plain_len = 0;
    uint64_t pn = 0;

    std::snprintf(label, sizeof label, "decrypt vector '%s'", v.name);
    bool ok = devourer::sta::ccmp_decrypt(crypto, v.tk, mpdu.data(),
                                          mpdu.size(), v.hdr_len, v.a2,
                                          plain.data(), plain.size(),
                                          &plain_len, &pn);
    check(ok, label);
    if (ok) {
      check(plain_len == v.plain_len, label);
      check(std::memcmp(plain.data(), v.plain, v.plain_len) == 0, label);
      check(pn == v.pn, label);
    }
  }
}

/* A corrupted MIC must be rejected. Without this the decrypt path could ignore
 * the tag entirely and every other test above would still pass. */
/* A short output buffer must be refused rather than overflowed. */
void test_short_output_refused() {
  OpenSslCcm crypto;
  const CcmpVector& v = kCcmpVectors[0];
  const size_t need = devourer::sta::ccmp_encrypted_len(v.hdr_len, v.plain_len);
  std::vector<uint8_t> out(need);

  check(need == v.mpdu_len, "ccmp_encrypted_len matches the vector");
  check(devourer::sta::ccmp_encrypt(crypto, v.tk, v.hdr, v.hdr_len, v.a2, v.pn,
                                    v.key_id, v.plain, v.plain_len, out.data(),
                                    need) == need,
        "an exactly-sized buffer is accepted");
  check(devourer::sta::ccmp_encrypt(crypto, v.tk, v.hdr, v.hdr_len, v.a2, v.pn,
                                    v.key_id, v.plain, v.plain_len, out.data(),
                                    need - 1) == 0,
        "a buffer one byte short is REFUSED, not overflowed");
}

/* LENGTHS THAT WOULD WRAP ARE REFUSED. The caller's hdr_len and plain_len are
 * summed with the CCMP overhead; a sum past SIZE_MAX would wrap to a small
 * number that a small buffer passes, so ccmp_encrypted_len reports 0
 * (impossible) and both calls refuse before touching any buffer. */
void test_length_overflow_refused() {
  OpenSslCcm crypto;
  const CcmpVector& v = kCcmpVectors[0];
  const size_t max = SIZE_MAX;
  std::vector<uint8_t> out(64, 0xa5);
  const std::vector<uint8_t> untouched = out;

  check(devourer::sta::ccmp_encrypted_len(24, max - 40) == max,
        "overflow: a total of exactly SIZE_MAX is still a length");
  check(devourer::sta::ccmp_encrypted_len(24, max - 39) == 0,
        "overflow: one more byte is impossible (0)");
  check(devourer::sta::ccmp_encrypted_len(max - 8, 0) == 0,
        "overflow: ...and so is a header length near SIZE_MAX");
  check(devourer::sta::ccmp_encrypt(crypto, v.tk, v.hdr, v.hdr_len, v.a2, v.pn,
                                    v.key_id, v.plain, max - 10, out.data(),
                                    out.size()) == 0 &&
            out == untouched,
        "overflow: encrypt refuses plain_len near SIZE_MAX, buffer untouched");

  check(devourer::sta::ccmp_decrypted_len(v.mpdu_len, max - 8) == 0,
        "overflow: decrypted_len of a header near SIZE_MAX is 0");
  check(!devourer::sta::ccmp_decrypt(crypto, v.tk, v.mpdu, v.mpdu_len,
                                     max - 8, v.a2, out.data(), out.size(),
                                     nullptr, nullptr) &&
            out == untouched,
        "overflow: decrypt refuses hdr_len near SIZE_MAX, buffer untouched");
}

void test_mic_rejected() {
  OpenSslCcm crypto;
  const CcmpVector& v = kCcmpVectors[0];
  std::vector<uint8_t> mpdu(v.mpdu, v.mpdu + v.mpdu_len);
  std::vector<uint8_t> plain(v.mpdu_len, 0);

  mpdu[mpdu.size() - 1] ^= 0x01;
  check(!devourer::sta::ccmp_decrypt(crypto, v.tk, mpdu.data(), mpdu.size(),
                                     v.hdr_len, v.a2, plain.data(),
                                     plain.size(), nullptr, nullptr),
        "a flipped MIC bit must be rejected");

  /* Same for the ciphertext: CCM authenticates it, so a body edit must fail
   * the tag too. */
  std::vector<uint8_t> body(v.mpdu, v.mpdu + v.mpdu_len);
  body[v.hdr_len + devourer::sta::kCcmpHdrLen] ^= 0x80;
  check(!devourer::sta::ccmp_decrypt(crypto, v.tk, body.data(), body.size(),
                                     v.hdr_len, v.a2, plain.data(),
                                     plain.size(), nullptr, nullptr),
        "a flipped ciphertext bit must be rejected");

  /* A buffer too small for the plaintext is refused rather than written past.
   * The capacity check matters on decrypt as much as on encrypt: the cipher
   * writes the plaintext out before it verifies the tag, so a forged
   * oversized frame is enough to overrun an unchecked buffer. */
  check(!devourer::sta::ccmp_decrypt(crypto, v.tk, v.mpdu, v.mpdu_len,
                                     v.hdr_len, v.a2, plain.data(),
                                     v.plain_len - 1, nullptr, nullptr),
        "an output buffer one byte too small must be refused");
  check(devourer::sta::ccmp_decrypted_len(v.mpdu_len, v.hdr_len) == v.plain_len,
        "ccmp_decrypted_len says exactly how much room to allocate");

  /* And a frame shorter than its own overhead must be refused rather than
   * read past its end. */
  check(!devourer::sta::ccmp_decrypt(crypto, v.tk, v.mpdu,
                                     v.hdr_len + 8 + 8 - 1, v.hdr_len, v.a2,
                                     plain.data(), plain.size(), nullptr,
                                     nullptr),
        "a frame shorter than its own overhead must be refused");
}

/* A QoS frame's AAD must include the TID, and the module must not be able to
 * disagree with the frame it was handed. The header length is explicit and
 * the TID comes out of the header, so a caller cannot pass a 26-byte QoS
 * header to a function that copies only 24 bytes and overwrites the QoS
 * Control field with the CCMP header. What is left to check is that the TID
 * actually reaches the MIC. */
void test_qos_aad() {
  OpenSslCcm crypto;
  const CcmpVector* q = nullptr;

  for (size_t i = 0; i < kCcmpVectorCount; i++)
    if (kCcmpVectors[i].hdr_len == 26) q = &kCcmpVectors[i];
  check(q != nullptr, "there is a real 26-byte QoS vector");
  if (!q) return;

  uint8_t aad[devourer::sta::kCcmpAadMax];
  size_t n = devourer::sta::ccmp_aad(q->hdr, q->hdr_len, aad);
  check(n == 24, "a QoS AAD is 24 bytes");
  check(aad[22] == (q->hdr[24] & 0x0f) && aad[23] == 0,
        "the QoS AAD carries the TID with the other bits masked");

  /* Treating the same frame as non-QoS - the mistake a 24-byte-only
   * implementation makes - must not verify. */
  std::vector<uint8_t> plain(q->mpdu_len, 0);
  check(!devourer::sta::ccmp_decrypt(crypto, q->tk, q->mpdu, q->mpdu_len, 24,
                                     q->a2, plain.data(), plain.size(), nullptr,
                                     nullptr),
        "a QoS frame read as non-QoS must NOT verify");

  /* A different TID in the header must not verify either, or the TID is not
   * really authenticated. */
  std::vector<uint8_t> tweak(q->mpdu, q->mpdu + q->mpdu_len);
  tweak[24] = (uint8_t)((tweak[24] & 0xf0) | ((q->hdr[24] + 1) & 0x0f));
  check(!devourer::sta::ccmp_decrypt(crypto, q->tk, tweak.data(), tweak.size(),
                                     q->hdr_len, q->a2, plain.data(),
                                     plain.size(), nullptr, nullptr),
        "a altered TID must not verify");

  /* And a header too short for what its frame control claims is refused
   * rather than read past. */
  check(devourer::sta::ccmp_aad(q->hdr, 24, aad) == 0,
        "a QoS header declared as 24 bytes is refused");

  /* THE SUBFIELDS ABOVE THE TID, which no vector in this repository can see.
   *
   * 802.11-2016 12.5.3.3.3 keeps only the TID out of the QoS Control field:
   * EOSP, the ack policy and A-MSDU-present are masked, and the second octet
   * is replaced by zero. Every real frame available here - the generated
   * vectors AND the kernel-captured ones - carries a plain TID with a zero
   * upper nibble and a zero second octet, so `hdr[qoff] & 0x0f` and
   * `hdr[qoff]` are the same byte and the mask is invisible to all of them.
   *
   * So deleting that mask passes every other cell in this file, kernel frames
   * included. This cell is the only thing that distinguishes it, and it has
   * to build its own header to do so, because the mask only matters for a
   * frame nobody in this rig transmits: a block-acked or A-MSDU frame, which
   * is what a real WMM/HT station sends constantly.
   *
   * The nonce half of the same rule is test_nonce_flags, which feeds it a
   * 0xf5 QoS octet; this is the AAD half. */
  {
    uint8_t loud[26], plainq[26];
    uint8_t a_loud[devourer::sta::kCcmpAadMax];
    uint8_t a_plain[devourer::sta::kCcmpAadMax];

    std::memcpy(loud, q->hdr, 26);
    loud[24] = 0x05 | 0x10 | 0x60 | 0x80; /* TID 5, EOSP, ack policy 3, A-MSDU */
    loud[25] = 0xff;                      /* TXOP limit / queue size */
    std::memcpy(plainq, loud, 26);
    plainq[24] = 0x05;                    /* the same TID, nothing else */
    plainq[25] = 0x00;

    check(devourer::sta::ccmp_aad(loud, 26, a_loud) == 24 &&
              devourer::sta::ccmp_aad(plainq, 26, a_plain) == 24,
          "both QoS AADs are 24 bytes");
    check(a_loud[22] == 0x05 && a_loud[23] == 0x00,
          "the QoS AAD masks EOSP, ack policy and A-MSDU-present");
    check(std::memcmp(a_loud, a_plain, 24) == 0,
          "only the TID of the QoS control field reaches the AAD");
  }
}

/* The nonce's Flags octet, asserted DIRECTLY rather than through a vector.
 *
 * The generated vectors cannot catch a wrong Flags octet: the generator in
 * tests/ccmp_gen_vectors.py builds the nonce from the same reading as
 * ccmp_nonce(), so a `nonce[0] = 0` in both would be self-consistent, and
 * test_qos_aad above checks the AAD, not the nonce. Independence of
 * implementation (python-cryptography vs OpenSSL) is not independence of
 * interpretation.
 *
 * 802.11-2016 12.5.3.3.4: Flags = Priority (b0..b3) | Management (b4).
 * Priority is the QoS TID for a QoS data frame, 0 otherwise. Linux builds the
 * identical byte as `qos_tid | (ieee80211_is_mgmt(fc) << 4)`
 * (net/mac80211/wpa.c) - that is the independent reading this pins against.
 *
 * A round trip against ourselves cannot see this at all, so the interop proof
 * is test_kernel_vectors(): frames mac80211 actually encrypted at every TID,
 * whose MIC fails the moment this octet is wrong. Zeroing the Flags octet
 * breaks 14 of those 16 vectors and leaves the two TID-0 ones green: TID 0
 * is where a zero octet is coincidentally right, which is why ordinary
 * best-effort traffic alone can never show this defect. */
void test_nonce_flags() {
  const uint8_t a2[6] = {0x02, 0xaa, 0xbb, 0xcc, 0xdd, 0x01};
  uint8_t nonce[devourer::sta::kCcmpNonceLen];
  uint8_t hdr[32];

  std::memset(hdr, 0, sizeof hdr);

  /* Non-QoS data, to-DS: priority 0, not management. */
  hdr[0] = 0x08; hdr[1] = 0x01;
  check(devourer::sta::ccmp_nonce(hdr, 24, a2, 1, nonce),
        "a 24-byte non-QoS header is accepted");
  check(nonce[0] == 0x00, "non-QoS data has a zero Flags octet");
  check(std::memcmp(nonce + 1, a2, 6) == 0, "the nonce carries A2");
  check(nonce[7] == 0 && nonce[12] == 1, "the nonce PN is big-endian");

  /* QoS data, TID 5 - the case a zero Flags octet gets wrong, and the reason
   * it matters: TID 5 is the video access category. */
  hdr[0] = 0x88; hdr[1] = 0x01; hdr[24] = 0x05;
  check(devourer::sta::ccmp_nonce(hdr, 26, a2, 1, nonce),
        "a 26-byte QoS header is accepted");
  check(nonce[0] == 0x05, "QoS TID 5 puts 5 in the Flags octet");

  /* The ack-policy / EOSP / A-MSDU bits in the QoS Control must be masked
   * out, exactly as the AAD masks them. */
  hdr[24] = 0xf5;
  devourer::sta::ccmp_nonce(hdr, 26, a2, 1, nonce);
  check(nonce[0] == 0x05, "only the TID's low nibble reaches the Flags octet");

  /* Four-address QoS: the QoS Control moves to offset 30. Reading it at 24
   * would take an address byte as the TID. */
  std::memset(hdr, 0, sizeof hdr);
  hdr[0] = 0x88; hdr[1] = 0x03; hdr[30] = 0x07;
  check(devourer::sta::ccmp_nonce(hdr, 32, a2, 1, nonce),
        "a 32-byte four-address QoS header is accepted");
  check(nonce[0] == 0x07, "four-address QoS reads its TID at offset 30");

  /* Management: bit 4 set, priority 0. */
  std::memset(hdr, 0, sizeof hdr);
  hdr[0] = 0xd0; /* type 0 (management), subtype 13 (action) */
  check(devourer::sta::ccmp_nonce(hdr, 24, a2, 1, nonce),
        "a management header is accepted");
  check(nonce[0] == 0x10, "management sets bit 4 of the Flags octet");

  /* Fails CLOSED on a header too short for what the frame control claims,
   * matching ccmp_aad's zero return rather than reading past the buffer. */
  hdr[0] = 0x88; hdr[1] = 0x01;
  check(!devourer::sta::ccmp_nonce(hdr, 24, a2, 1, nonce),
        "a QoS header declared as 24 bytes is refused");
  hdr[1] = 0x03;
  check(!devourer::sta::ccmp_nonce(hdr, 30, a2, 1, nonce),
        "a four-address QoS header declared as 30 bytes is refused");
}

/* A 4-address frame's AAD includes A4. Nothing in the tree builds one yet,
 * which is why the branch needs a vector - it would otherwise be dead code
 * that nobody would notice was broken. */
void test_four_address_aad() {
  const CcmpVector* f = nullptr;

  for (size_t i = 0; i < kCcmpVectorCount; i++)
    if (kCcmpVectors[i].aad_len == 30) f = &kCcmpVectors[i];
  check(f != nullptr, "there is a 4-address QoS vector");
  if (!f) return;

  uint8_t aad[devourer::sta::kCcmpAadMax];
  check(devourer::sta::ccmp_aad(f->hdr, f->hdr_len, aad) == 30,
        "a 4-address QoS AAD is 30 bytes");
  check(std::memcmp(aad + 22, f->hdr + 24, 6) == 0,
        "the 4-address AAD carries A4");
}

/* The AAD rules, asserted directly, because they are the part that is silent
 * when wrong: a bad AAD is indistinguishable from a bad key at the far end. */
void test_aad_masking() {
  /* TWENTY-SIX, not twenty-four. This cell declares a QoS frame and passes
   * hdr_len 26 below, so ccmp_aad reads the QoS Control field at offset 24 -
   * off the end of a 24-byte array. Only a sanitizer build notices that
   * overread, which is why these cells run under ASan/UBSan in CI. */
  uint8_t hdr[26];
  uint8_t aad[devourer::sta::kCcmpAadMax];

  std::memset(hdr, 0, sizeof hdr);
  hdr[0] = 0x88;                    /* QoS data, subtype bits set */
  /* ToDS SET, so the DS-bit check below can actually fail. With ToDS and
   * FromDS both clear the assertion `(aad[1] & 0x03) == (hdr[1] & 0x03)`
   * reads `0 == 0` and passes an AAD that clears the DS bits. */
  hdr[1] = devourer::sta::kFcToDs | 0x08 | 0x10 | 0x20;      /* retry | pwr mgmt | more data */
  hdr[22] = 0x35;                   /* frag 5, seq low bits */
  hdr[23] = 0x12;                   /* seq high */

  size_t n = devourer::sta::ccmp_aad(hdr, 26, aad);
  check(n == 24, "a QoS 3-address AAD is 24 bytes");
  check((aad[0] & 0x70) == 0, "AAD masks the FC subtype bits");
  check((aad[1] & 0x08) == 0, "AAD masks Retry");
  check((aad[1] & 0x10) == 0, "AAD masks Pwr Mgmt");
  check((aad[1] & 0x20) == 0, "AAD masks More Data");
  check((aad[1] & 0x40) != 0, "AAD forces Protected on");
  check(aad[20] == 0x05, "AAD keeps the fragment number");
  check(aad[21] == 0x00, "AAD masks the sequence number");
  /* What SURVIVES matters as much as what is masked: an AAD that zeroed the
   * addresses or the DS bits would pass every assertion above. */
  check((aad[1] & 0x03) == (hdr[1] & 0x03), "AAD preserves ToDS/FromDS");
  check(std::memcmp(aad + 2, hdr + 4, 18) == 0,
        "AAD carries addr1/addr2/addr3 verbatim");

  /* THE ORDER BIT (+HTC), which no vector in this file can see because none
   * of them sets it. On a QoS data frame bit 15 means "an HT Control field
   * follows" and 802.11-2016 12.5.3.3.3 masks it; on a NON-QoS frame the same
   * bit is the strictly-ordered service class and must survive. Both arms,
   * because masking it unconditionally is as wrong as never masking it. */
  {
    uint8_t htc[30];
    uint8_t a[devourer::sta::kCcmpAadMax];

    std::memset(htc, 0, sizeof htc);
    htc[0] = 0x88;                                      /* QoS data */
    htc[1] = (uint8_t)(devourer::sta::kFcToDs | 0x80);  /* +HTC / Order */
    htc[24] = 0x03;                                     /* TID 3 */
    check(devourer::sta::ccmp_aad(htc, 30, a) == 24,
          "a QoS +HTC AAD is still 24 bytes - HT Control is not in it");
    check((a[1] & 0x80) == 0, "AAD masks the Order bit on a QoS data frame");
    check((a[1] & 0x01) != 0, "...without losing ToDS");

    /* The same bit on a non-QoS data frame is a different field. */
    uint8_t ord[24];
    std::memset(ord, 0, sizeof ord);
    ord[0] = 0x08;                                      /* data, not QoS */
    ord[1] = (uint8_t)(devourer::sta::kFcToDs | 0x80);
    check(devourer::sta::ccmp_aad(ord, 24, a) == 22, "a non-QoS AAD is 22");
    check((a[1] & 0x80) != 0,
          "AAD KEEPS bit 15 on a non-QoS frame - there it is the "
          "strictly-ordered service class, not +HTC");
  }

  /* A header shorter than a frame control is refused rather than READ: the
   * length check has to come before the DS bits and the subtype are derived,
   * or it is an overread for any caller that gets the length wrong.
   *
   * THE BUFFERS ARE ONE BYTE ON THE HEAP, deliberately. Passing a short
   * length over a long buffer proves nothing: the read succeeds, the second
   * length check returns 0 anyway, and the reordering is invisible. With a
   * genuine one-byte allocation the overread is a heap-buffer-overflow, which
   * the `build-sanitizers` CI job turns into a failure. IN A NON-SANITIZED
   * BUILD THIS ARM CANNOT FAIL: reading before checking passes here, and only
   * the sanitizer job catches it. */
  {
    std::vector<uint8_t> one(1, 0x88);
    std::vector<uint8_t> none;
    uint8_t a2[6] = {0};
    uint8_t nonce[devourer::sta::kCcmpNonceLen];

    check(devourer::sta::ccmp_aad(one.data(), 1, aad) == 0,
          "ccmp_aad refuses a one-byte header without reading past it");
    check(devourer::sta::ccmp_aad(none.data(), 0, aad) == 0,
          "ccmp_aad refuses a zero-length header without reading it");
    check(!devourer::sta::ccmp_nonce(one.data(), 1, a2, 1, nonce),
          "ccmp_nonce refuses a one-byte header without reading past it");
  }

  /* A protected MANAGEMENT frame keeps its subtype - mac80211 masks the
   * subtype only for non-management frames, and 802.11w depends on it. */
  uint8_t mgmt[24];
  std::memset(mgmt, 0, sizeof mgmt);
  mgmt[0] = 0xd0;  /* Action frame: type 0, subtype 13 */
  mgmt[1] = 0x08;  /* retry, which must still be masked */
  check(devourer::sta::ccmp_aad(mgmt, 24, aad) == 22, "a mgmt AAD is 22 bytes");
  check((aad[0] & 0x70) == 0x50, "a management frame KEEPS its subtype");
  check((aad[1] & 0x08) == 0, "a management frame still masks Retry");
}

/* The CCMP header's PN is split across two discontiguous ranges and the Ext IV
 * bit is not optional. Round-tripping the maximum PN catches a 32-bit
 * truncation, which would otherwise only appear after 4 billion frames. */
void test_header_pn() {
  uint8_t h[8];
  const uint64_t pn = 0xfedcba987654ULL;

  devourer::sta::ccmp_header(pn, 2, h);
  check((h[3] & 0x20) != 0, "CCMP header sets Ext IV");
  check(((h[3] >> 6) & 3) == 2, "CCMP header carries the key id");
  check(h[2] == 0, "CCMP header byte 2 is reserved and zero");
  check(devourer::sta::ccmp_header_pn(h) == pn, "48-bit PN round-trips");

  devourer::sta::ccmp_header(0xffffffffffffULL, 0, h);
  check(devourer::sta::ccmp_header_pn(h) == 0xffffffffffffULL,
        "the maximum PN round-trips");
}

/* The replay gate.
 *
 * AN EQUAL PN IS A REPLAY. A gate written as `<` admits an equal-counter
 * replay, which is how a group-key reinstallation (KRACK class) lands. That
 * is the first block below.
 *
 * The rest covers the sliding window. A bare counter is only correct while
 * frames cannot arrive out of order, which is an assumption about the PEER:
 * the moment a BlockAck agreement exists an A-MPDU can deliver PN 5, 7, 6
 * legitimately and a counter drops 6 as a replay. This tree's AP harnesses
 * never set up a BlockAck agreement; a station does not get to choose. */
void test_replay() {
  devourer::sta::CcmpReplay r;

  check(!r.accept(0), "PN 0 is never valid");
  check(r.accept(1), "first PN is accepted");
  check(!r.accept(1), "an EQUAL-counter replay must be REJECTED");
  check(!r.accept(0), "a lower PN must be rejected");
  check(r.accept(2), "a higher PN is accepted");
  check(r.last() == 2, "the window advanced to the accepted PN");
  check(!r.accept(2), "the advanced counter still rejects its equal");

  /* A rejected PN must not advance the window; otherwise a forged high PN
   * would lock out the legitimate peer. The rejected value here is one that
   * is genuinely out of range: PN 50 after 100 would not do, because a
   * sliding window correctly ACCEPTS it - 50 frames of reordering is not a
   * replay. */
  devourer::sta::CcmpReplay r2;
  check(r2.accept(100), "setup");
  check(!r2.accept(100 - devourer::sta::CcmpReplay::kWindow),
        "a PN exactly one window behind is too old to judge");
  check(r2.last() == 100, "a rejected PN does not move the window");

  /* --- REORDERING, which a bare counter gets wrong -----------------------
   *
   * A BlockAck agreement lets an A-MPDU deliver PNs out of order. The strict
   * `pn > last` rule drops the late ones as replays: silent data loss that
   * looks like a radio problem. A station does not get to choose whether its
   * AP sets up BlockAck. */
  devourer::sta::CcmpReplay w;
  check(w.accept(10), "window: first frame");
  check(w.accept(12), "window: a gap is fine");
  check(w.accept(11), "window: THE LATE FRAME IN THE GAP IS ACCEPTED");
  check(!w.accept(11), "window: but only once");
  check(!w.accept(12), "window: and the one that arrived early is not replayable");
  check(w.last() == 12, "window: a late frame does not move the head backwards");

  /* Fill a whole window out of order, then prove every one of them is a
   * replay on a second pass. */
  devourer::sta::CcmpReplay f;
  check(f.accept(1000), "window: head");
  for (int i = 1; i < devourer::sta::CcmpReplay::kWindow; i++)
    if (!f.accept(1000 - (uint64_t)i))
      check(false, "window: every PN inside the window is accepted once");
  {
    bool any = false;
    for (int i = 0; i < devourer::sta::CcmpReplay::kWindow; i++)
      if (f.accept(1000 - (uint64_t)i)) any = true;
    check(!any, "window: and every one of them is a replay the second time");
  }

  /* The edges. One inside is accepted, one outside is refused - off by one
   * here is either a dropped frame or an admitted replay. */
  devourer::sta::CcmpReplay e;
  check(e.accept(1000), "edge: head");
  check(e.accept(1000 - (devourer::sta::CcmpReplay::kWindow - 1)),
        "edge: the oldest PN still inside the window is accepted");
  check(!e.accept(1000 - devourer::sta::CcmpReplay::kWindow),
        "edge: one past the window is refused");

  /* A forward jump larger than the window must clear it, and must not shift
   * a uint64_t by >= 64 on the way - undefined behaviour, and exactly the
   * gap a hostile peer would choose. Everything behind becomes unreachable,
   * which is the safe direction: no replay can be admitted afterwards. */
  devourer::sta::CcmpReplay j;
  check(j.accept(10), "jump: head");
  check(j.accept(10 + devourer::sta::CcmpReplay::kWindow + 5), "jump: a big skip");
  check(!j.accept(11), "jump: the skipped PNs are gone, not replayable");
  check(!j.accept(10), "jump: including the one already seen");
  check(j.accept(10 + devourer::sta::CcmpReplay::kWindow + 4),
        "jump: but a frame inside the NEW window is still accepted");

  /* THE SHIFT GUARD, and it needs a carefully chosen jump to be visible.
   *
   * Shifting a uint64_t by >= 64 is undefined, and on x86 the count is taken
   * modulo 64 - so an UNGUARDED `mask << shift` with shift == kWindow + 1
   * becomes `mask << 1`, and the head's own bit survives as bit 1 of the new
   * window. That marks a PN the receiver has NEVER SEEN as already seen, and
   * the next legitimate frame at that PN is dropped as a replay.
   *
   * So the symptom is a LOST FRAME, not an admitted one, and it only appears
   * for a jump that lands a stale bit inside the new window. A jump of
   * kWindow + 5 does not: everything behind it falls outside the window and
   * is refused for that reason instead, so a cell using that jump passes
   * with the guard removed. kWindow + 1 is the jump that shows it. */
  devourer::sta::CcmpReplay sh;
  check(sh.accept(10), "shift: head at 10");
  check(sh.accept(10 + devourer::sta::CcmpReplay::kWindow + 1),
        "shift: jump one past the window");
  check(sh.accept(10 + devourer::sta::CcmpReplay::kWindow),
        "shift: THE PN JUST BEHIND THE NEW HEAD WAS NEVER SEEN - accept it");

  /* An enormous jump - the same shift, at the top of the PN space. */
  devourer::sta::CcmpReplay h2;
  check(h2.accept(1), "huge: head");
  check(h2.accept(0xffffffffffffULL), "huge: the maximum PN is accepted");
  check(!h2.accept(1), "huge: the old PN is not replayable");
  check(h2.last() == 0xffffffffffffULL, "huge: the head moved");

  /* NOT TESTABLE HERE, stated rather than implied: the `behind >= kWindow`
   * bound. Relaxing it to `>` lets `behind == kWindow` through to a
   * `1ull << 64`, which is undefined - but on x86 that evaluates to 1, and
   * bit 0 is the head, which is always set, so the frame is refused anyway
   * and the defect is invisible. The guard is there by construction, not
   * because a test on this ISA can distinguish it. */

  /* ONE COUNTER PER TID. A single shared counter drops legitimate frames as
   * soon as two TIDs interleave, which is routine the moment voice or video
   * shares a link with best-effort. */
  devourer::sta::CcmpReplay t;
  check(t.accept(10, 0), "TID 0 accepts PN 10");
  check(t.accept(5, 6), "TID 6 accepts a LOWER PN than TID 0 has seen");
  check(!t.accept(5, 6), "TID 6 still rejects its own replay");
  check(t.accept(11, 0), "TID 0 continues independently");
  check(t.last(0) == 11 && t.last(6) == 5, "the two windows are separate");
  /* ACCEPTING ONE PN PROVES NOTHING: the first PN in any window is accepted,
   * and PN 1 is inside TID 0's window too, so aliasing kNonQosTid to 0 would
   * pass that alone. The heads have to be read back separately. */
  check(t.accept(1, devourer::sta::CcmpReplay::kNonQosTid),
        "non-QoS traffic has a window of its own");
  check(t.last(devourer::sta::CcmpReplay::kNonQosTid) == 1 && t.last(0) == 11,
        "...a SEPARATE one - its head is 1 while TID 0's is still 11");
  check(!t.accept(99, -1), "an out-of-range TID is refused");
  check(!t.accept(99, 17), "an out-of-range TID is refused");

  /* A rekey resets every counter: a new key is a new PN space.
   *
   * ACCEPTING PN 1 AFTER THE RESET PROVES NOTHING - it is inside the window
   * and unmarked whether or not reset() ran, so an empty reset() would pass
   * it. What cannot pass is re-accepting a PN the window has ALREADY SEEN,
   * and reading the head back. */
  t.reset();
  check(t.last(0) == 0 && t.last(6) == 0, "reset clears both heads");
  check(t.accept(11, 0), "a PN already accepted on TID 0 is accepted again");
  check(t.accept(5, 6), "...and one already accepted on TID 6");
  check(t.accept(1, 0) && t.accept(1, 6), "reset clears every TID window");
}


/* The Linux kernel's own CCMP output, decrypted and then reproduced.
 *
 * Every other vector in this file comes from software that shares this
 * repository's reading of 802.11-2016 12.5.3.3, so a shared misreading - a
 * nonce with a zero Flags octet, wrong for TID 1..7 - passes them all.
 * These frames are mac80211's output, captured off the air of a two-radio
 * mac80211_hwsim rig; nothing in this repository chose their bytes.
 *
 * Two directions of proof per vector:
 *
 *   DECRYPT. The MIC is the oracle. CCM authenticates the AAD and the nonce,
 *   so a single wrong bit in either — a masked frame-control bit, the kept
 *   fragment number, the TID in the Flags octet, the big-endian PN — makes the
 *   tag fail. There is no way to pass this by accident.
 *
 *   ENCRYPT. Re-protecting the recovered plaintext under the same TK and PN
 *   must reproduce the captured MPDU byte for byte, which additionally pins
 *   the CCMP header layout (the reserved byte, the Ext IV bit, the split
 *   little-endian PN) that a decrypt-only test would let drift.
 */
void test_kernel_vectors() {
  OpenSslCcm crypto;
  unsigned tids_seen = 0;

  for (size_t i = 0; i < devourer::test::kKernelCcmpVectorCount; i++) {
    const devourer::test::KernelCcmpVector& v =
        devourer::test::kKernelCcmpVectors[i];
    /* A 3-address frame's A2 is the transmitter in both directions. */
    const uint8_t* a2 = v.mpdu + 10;
    /* SIZED FROM THE VECTOR, not from a guess. A fixed `uint8_t plain[512]`
     * is tied to nothing: regenerate on a rig whose first frame for some TID
     * is full-MTU and the decrypt would write a kilobyte past it. */
    std::vector<uint8_t> plain(
        devourer::sta::ccmp_decrypted_len(v.mpdu_len, v.hdr_len));
    std::vector<uint8_t> again(v.mpdu_len);
    size_t plen = 0, n;
    uint64_t pn = 0;
    char what[128];

    std::snprintf(what, sizeof what, "kernel %s: the kernel's MIC verifies",
                  v.name);
    if (!checked(devourer::sta::ccmp_decrypt(crypto, devourer::test::kKernelTk,
                                             v.mpdu, v.mpdu_len, v.hdr_len, a2,
                                             plain.data(), plain.size(), &plen,
                                             &pn),
                 what))
      continue;
    tids_seen |= 1u << v.tid;

    /* The recovered plaintext is an 802.11 MSDU, so it opens with LLC/SNAP.
     * Redundant given the tag verified, but it turns a corrupted vector file
     * into a legible failure instead of a crypto mystery. */
    std::snprintf(what, sizeof what, "kernel %s: plaintext is LLC/SNAP",
                  v.name);
    check(plen > 8 && plain[0] == 0xaa && plain[1] == 0xaa && plain[2] == 0x03,
          what);

    /* The PN the kernel wrote into the header is the PN we must encrypt
     * under, and the key id comes out of the same octet. */
    std::snprintf(what, sizeof what, "kernel %s: re-encrypts to the same bytes",
                  v.name);
    n = devourer::sta::ccmp_encrypt(
        crypto, devourer::test::kKernelTk, v.mpdu, v.hdr_len, a2, pn,
        (uint8_t)((v.mpdu[v.hdr_len + 3] >> 6) & 0x03), plain.data(), plen,
        again.data(), again.size());
    check(n == v.mpdu_len && std::memcmp(again.data(), v.mpdu, n) == 0, what);
  }

  /* The coverage assertion is the point of the exercise. TID 0 passes with a
   * zero Flags octet too, so a run that silently lost the QoS vectors would
   * still be green without this. */
  check(tids_seen == 0xffu, "kernel vectors cover all eight TIDs");
  check(devourer::test::kKernelCcmpVectorCount == 16,
        "kernel vectors: eight TIDs in each direction");
}

/* A NULL OUTPUT MUST NEVER REACH THE CIPHER. OpenSSL's CCM reads a NULL
 * output pointer as "this is AAD" and returns success with NO TAG CHECK, and
 * `std::vector<uint8_t> plain(ccmp_decrypted_len(...))` hands exactly that
 * over (`data()` of an empty vector) for a ZERO-BODY frame. A forged one
 * then decrypts "successfully" with an attacker-chosen PN, which moves the
 * replay window to wherever the attacker likes. Both layers are pinned: the
 * module never passes NULL, and the software primitive refuses it anyway. */
struct NullSpy : OpenSslCcm {
  bool saw_null = false;
  bool aes_ccm(bool encrypt, const uint8_t key[16], const uint8_t nonce[13],
               const uint8_t* aad, size_t aad_len, const uint8_t* in,
               size_t in_len, uint8_t* out, uint8_t* tag) override {
    if (!out) saw_null = true;
    return OpenSslCcm::aes_ccm(encrypt, key, nonce, aad, aad_len, in, in_len,
                               out, tag);
  }
};

void test_null_output_and_zero_body() {
  NullSpy crypto;
  const CcmpVector& v = kCcmpVectors[0];
  const uint64_t pn = 0x7fffffffffffULL;

  /* Forged: header, CCMP header at a huge PN, and eight bytes of made-up
   * MIC. No body. */
  std::vector<uint8_t> forged(v.hdr, v.hdr + v.hdr_len);
  forged[1] |= 0x40;
  forged.resize(v.hdr_len + devourer::sta::kCcmpHdrLen);
  devourer::sta::ccmp_header(pn, 0, forged.data() + v.hdr_len);
  forged.insert(forged.end(), devourer::sta::kCcmpMicLen, 0x5a);

  std::vector<uint8_t> plain(
      devourer::sta::ccmp_decrypted_len(forged.size(), v.hdr_len));
  size_t plen = 99;
  uint64_t got = 0;
  check(plain.empty(), "a zero-body frame sizes an EMPTY output vector");
  check(!devourer::sta::ccmp_decrypt(crypto, v.tk, forged.data(),
                                     forged.size(), v.hdr_len, v.a2,
                                     plain.data(), plain.size(), &plen, &got),
        "A FORGED ZERO-BODY FRAME IS REFUSED with a NULL output buffer");
  check(got == 0, "...and reports no PN");
  check(!crypto.saw_null, "...and the cipher was never handed NULL");

  /* The primitive on its own, the layer an integrator's CryptoOps sits at. */
  uint8_t nonce[13] = {0}, aad[32] = {0}, tag[8];
  std::memset(tag, 0x5a, sizeof tag);
  check(!devourer::test::ccmp_software(false, v.tk, nonce, aad, 22,
                                       forged.data(), 0, nullptr, tag),
        "ccmp_software refuses a forged tag with a NULL output");

  /* THE POSITIVE ARM: a genuine zero-body frame still decrypts, with the
   * same empty output vector, and its tag IS checked. */
  std::vector<uint8_t> real(
      devourer::sta::ccmp_encrypted_len(v.hdr_len, 0));
  const size_t n = devourer::sta::ccmp_encrypt(
      crypto, v.tk, v.hdr, v.hdr_len, v.a2, 7, 0, nullptr, 0, real.data(),
      real.size());
  check(n == real.size(), "a zero-body frame encrypts");
  got = 0;
  check(devourer::sta::ccmp_decrypt(crypto, v.tk, real.data(), real.size(),
                                    v.hdr_len, v.a2, plain.data(),
                                    plain.size(), &plen, &got),
        "...and a GENUINE zero-body frame decrypts into an empty vector");
  check(plen == 0 && got == 7, "...with no bytes and its own PN");
  real[real.size() - 1] ^= 0x01;
  check(!devourer::sta::ccmp_decrypt(crypto, v.tk, real.data(), real.size(),
                                     v.hdr_len, v.a2, plain.data(),
                                     plain.size(), nullptr, nullptr),
        "...whose tag is still verified");
  check(!crypto.saw_null, "the cipher was never handed NULL");
}

/* seed(): the group window starts at the Key RSC the AP quoted. */
void test_replay_seed() {
  devourer::sta::CcmpReplay s;

  s.seed(1000);
  check(!s.accept(1000), "seed: the RSC itself is refused");
  check(!s.accept(999), "seed: one below it is refused");
  check(!s.accept(1000 - 40), "seed: inside the window below it is refused");
  check(!s.accept(3), "seed: far below it is refused");
  check(s.accept(1001, 3), "seed: above it is accepted, on any TID");
  check(s.accept(1001), "seed: ...and on the non-QoS window too");

  s.seed(0);
  check(s.accept(1), "seed(0) is reset(): PN 1 is accepted");
}

}  // namespace

/* THE PN IS 48 BITS. ccmp_encrypt takes a uint64_t, but the nonce and the
 * CCMP header carry only the low 48 bits, so PN 2^48 would air with PN 0's
 * nonce under the same TK. The last valid PN encrypts and round-trips; one
 * more is refused and writes nothing. */
void test_pn_is_48_bits() {
  OpenSslCcm crypto;
  const CcmpVector& v = kCcmpVectors[0];
  const uint64_t last = devourer::sta::kCcmpPnMax;

  check(last == 0xffffffffffffull, "pn: the limit is 2^48 - 1");
  std::vector<uint8_t> out(v.mpdu_len, 0);
  const size_t n = devourer::sta::ccmp_encrypt(
      crypto, v.tk, v.hdr, v.hdr_len, v.a2, last, 0, v.plain, v.plain_len,
      out.data(), out.size());
  check(n == v.mpdu_len, "pn: 2^48 - 1 encrypts");
  std::vector<uint8_t> plain(v.plain_len);
  size_t plen = 0;
  uint64_t pn = 0;
  check(n == v.mpdu_len &&
            devourer::sta::ccmp_decrypt(crypto, v.tk, out.data(), n, v.hdr_len,
                                        v.a2, plain.data(), plain.size(), &plen,
                                        &pn) &&
            pn == last && plen == v.plain_len &&
            std::memcmp(plain.data(), v.plain, v.plain_len) == 0,
        "pn: ...and round-trips with that PN");

  std::vector<uint8_t> untouched(v.mpdu_len, 0xa5);
  check(devourer::sta::ccmp_encrypt(crypto, v.tk, v.hdr, v.hdr_len, v.a2,
                                    last + 1, 0, v.plain, v.plain_len,
                                    untouched.data(), untouched.size()) == 0,
        "pn: 2^48 is refused");
  bool clean = true;
  for (uint8_t b : untouched) clean = clean && b == 0xa5;
  check(clean, "pn: ...and nothing is written");
  check(devourer::sta::ccmp_encrypt(crypto, v.tk, v.hdr, v.hdr_len, v.a2,
                                    ~0ull, 0, v.plain, v.plain_len,
                                    untouched.data(), untouched.size()) == 0,
        "pn: so is the largest uint64_t");
}

/* EXT IV IS REQUIRED on receive. The key-id octet is not in the AAD, so a
 * valid frame with Ext IV cleared still carries a correct MIC - only an
 * explicit check refuses it. The reserved bits of that octet are ignored,
 * per 802.11's reserved-field convention. */
void test_ext_iv_required() {
  OpenSslCcm crypto;
  const CcmpVector& v = kCcmpVectors[0];
  std::vector<uint8_t> f(v.mpdu, v.mpdu + v.mpdu_len);
  std::vector<uint8_t> plain(v.plain_len);
  size_t plen = 0;

  check(devourer::sta::ccmp_decrypt(crypto, v.tk, f.data(), f.size(), v.hdr_len,
                                    v.a2, plain.data(), plain.size(), &plen,
                                    nullptr),
        "ext iv: the untouched vector decrypts");
  f[v.hdr_len + 3] &= (uint8_t)~0x20;
  check(!devourer::sta::ccmp_decrypt(crypto, v.tk, f.data(), f.size(),
                                     v.hdr_len, v.a2, plain.data(),
                                     plain.size(), &plen, nullptr),
        "ext iv: the same frame with Ext IV cleared is refused");
  f[v.hdr_len + 3] |= 0x20;
  f[v.hdr_len + 3] |= 0x1f;                      /* reserved bits 0-4 */
  f[v.hdr_len + 2] = 0xff;                       /* reserved octet */
  check(devourer::sta::ccmp_decrypt(crypto, v.tk, f.data(), f.size(),
                                    v.hdr_len, v.a2, plain.data(),
                                    plain.size(), &plen, nullptr),
        "ext iv: reserved bits are ignored on receive");
}

/* THE FRAME MUST SAY IT IS PROTECTED. ccmp_aad forces the Protected bit to 1
 * in the AAD, so a valid MPDU with the bit cleared on the wire still carries a
 * correct MIC - only an explicit check refuses it. */
void test_protected_bit_required() {
  OpenSslCcm crypto;
  const CcmpVector& v = kCcmpVectors[0];
  std::vector<uint8_t> f(v.mpdu, v.mpdu + v.mpdu_len);
  std::vector<uint8_t> plain(v.plain_len);
  size_t plen = 0;

  f[1] &= (uint8_t)~devourer::sta::kFcProtected;
  check(!devourer::sta::ccmp_decrypt(crypto, v.tk, f.data(), f.size(),
                                     v.hdr_len, v.a2, plain.data(),
                                     plain.size(), &plen, nullptr),
        "protected bit: a valid frame with Protected cleared is refused");
  f[1] |= devourer::sta::kFcProtected;
  check(devourer::sta::ccmp_decrypt(crypto, v.tk, f.data(), f.size(),
                                    v.hdr_len, v.a2, plain.data(),
                                    plain.size(), &plen, nullptr),
        "protected bit: ...and decrypts with it set again");
}

int main() {
  test_vectors();
  test_protected_bit_required();
  test_pn_is_48_bits();
  test_ext_iv_required();
  test_short_output_refused();
  test_length_overflow_refused();
  test_mic_rejected();
  test_qos_aad();
  test_nonce_flags();
  test_kernel_vectors();
  test_four_address_aad();
  test_aad_masking();
  test_header_pn();
  test_replay();
  test_null_output_and_zero_body();
  test_replay_seed();

  if (g_fail) {
    std::printf("ccmp_selftest: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("ccmp_selftest: OK (%zu vectors, %zu kernel frames)\n",
              kCcmpVectorCount, devourer::test::kKernelCcmpVectorCount);
  return 0;
}
