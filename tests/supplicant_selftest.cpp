/* Headless guard for src/sta/Eapol.h and src/sta/Supplicant.h — the station
 * half of the WPA2-PSK key exchange.
 *
 * THE SUPPLICANT'S ACCEPTANCE IS TWO NEGATIVE CASES, named here so that
 * a future edit that deletes them has to delete a stated requirement:
 *
 *   test_forged_mic_is_rejected()        — a forged EAPOL-Key MIC
 *   test_group_rekey_replay_rejected()   — an equal-counter group rekey
 *
 * Either defect hands an attacker key material: a forged MIC installs keys
 * nobody authenticated, and an equal-counter group rekey reinstalls an old
 * GTK with its PN space reset. A supplicant that cannot fail these two tests
 * has not been tested.
 *
 * WHAT IS AND IS NOT INDEPENDENT HERE. The authenticator below is a fixture,
 * and it builds its frames with the same `build_eapol_key` the supplicant
 * parses — so this file pins the STATE MACHINE and the CHECKS, not the wire
 * format. Three things cover the format instead:
 *
 *   - the IEEE 802.11i Annex H.4.2 vectors at the top, which cover PBKDF2
 *     AND NOTHING ELSE - not the EAPOL-Key format.
 *   - test_against_a_real_four_way(), which replays the four EAPOL-Key frames
 *     HOSTAPD AND WPA_SUPPLICANT actually exchanged and checks our PTK
 *     against the one wpa_supplicant derived. That is the PRF, the address
 *     and nonce sorting, the MIC and the GTK KDE layout, all pinned against
 *     software that has never read this repository.
 *   - NOT a cross-role cell against the AP harness's authenticator, which
 *     hand-rolls every offset: that harness has no headless tests in this
 *     tree, so the hostapd replay above is the independent format check.
 */
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "eapol_kernel_vectors.h"
#include "openssl_crypto_ops.h"
#include "sta/Eapol.h"
#include "sta/Supplicant.h"

namespace {

using devourer::sta::EapolKey;
using devourer::sta::Supplicant;
using devourer::sta::MicCheck;
using devourer::test::OpenSslCryptoOps;

int g_fail = 0;

void check(bool ok, const char* what) {
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    g_fail++;
  }
}

const uint8_t kAa[6] = {0x02, 0x42, 0x75, 0x05, 0xd6, 0x00};   /* the AP */
const uint8_t kSpa[6] = {0x02, 0x11, 0x22, 0x33, 0x44, 0x01};  /* the station */
const char* kPsk = "devourer123";
const char* kSsid = "devourerAP";

/* ---- known answers ------------------------------------------------------
 *
 * Hand-rolled or not, a PBKDF2 or PRF with no known-answer test can be wrong
 * in a way no round-trip notices. These are the IEEE 802.11i Annex H.4.2
 * passphrase-to-PSK vectors —
 * published, widely reproduced, and recomputed with Python's hashlib before
 * being written down here, so they are not this repository's own arithmetic
 * repeated back at itself. */
struct PskVector {
  const char* passphrase;
  const char* ssid;
  const char* pmk_hex;
};
const PskVector kPskVectors[] = {
    {"password", "IEEE",
     "f42c6fc52df0ebef9ebb4b90b38a5f902e83fe1b135a70e23aed762e9710a12e"},
    {"ThisIsAPassword", "ThisIsASSID",
     "0dc0d6eb90555ed6419756b9a15ec3e3209b63df707dd508d14581f8982721af"},
    {"aaaaaaaaaa", "ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ",
     "727c76be68d32158bed0787c002aaf97ac38a34d11da8be1afdcf3e96965346d"},
};

std::vector<uint8_t> unhex(const char* h) {
  std::vector<uint8_t> v;
  for (size_t i = 0; h[i] && h[i + 1]; i += 2) {
    auto nib = [](char c) -> int {
      return c >= 'a' ? c - 'a' + 10 : (c >= 'A' ? c - 'A' + 10 : c - '0');
    };
    v.push_back((uint8_t)((nib(h[i]) << 4) | nib(h[i + 1])));
  }
  return v;
}

void test_psk_known_answers() {
  OpenSslCryptoOps c;

  for (const PskVector& v : kPskVectors) {
    uint8_t pmk[32];
    const std::vector<uint8_t> want = unhex(v.pmk_hex);
    char label[96];

    std::snprintf(label, sizeof label, "PSK vector '%s' / '%s'", v.passphrase,
                  v.ssid);
    check(devourer::sta::pmk_from_psk(c, v.passphrase, v.ssid, pmk), label);
    check(want.size() == 32 && std::memcmp(pmk, want.data(), 32) == 0, label);
  }

  /* The SSID is the SALT, which is why the same passphrase on two networks
   * does not share a PMK. Without this, dropping the salt entirely would pass
   * every symmetry test in the file. */
  uint8_t a[32], b[32];
  devourer::sta::pmk_from_psk(c, "password", "IEEE", a);
  devourer::sta::pmk_from_psk(c, "password", "IEEF", b);
  check(std::memcmp(a, b, 32) != 0, "the SSID is part of the derivation");

  /* Refusals: an SSID must be 1..32 octets. */
  check(!devourer::sta::pmk_from_psk(c, "password", "", a),
        "an empty SSID is refused");
  check(!devourer::sta::pmk_from_psk(c, "password", std::string(33, 'x'), a),
        "a 33-octet SSID is refused");

  /* THE OTHER SPELLING: exactly 64 hex digits ARE the PMK (802.11-2016
   * J.4.1; hostapd wpa_psk=, wpa_supplicant psk=). Known answer: the first
   * Annex H vector's PMK, given raw, must come back byte for byte and must
   * NOT be the PBKDF2 of those 64 characters. */
  const char* hex = kPskVectors[0].pmk_hex;
  const std::vector<uint8_t> want = unhex(hex);
  std::memset(a, 0, sizeof a);
  check(devourer::sta::pmk_from_psk(c, hex, "IEEE", a) &&
            std::memcmp(a, want.data(), 32) == 0,
        "a 64-hex-digit PSK is decoded as the raw PMK");
  std::string upper(hex);
  for (char& ch : upper)
    if (ch >= 'a' && ch <= 'f') ch = (char)(ch - 'a' + 'A');
  std::memset(b, 0, sizeof b);
  check(devourer::sta::pmk_from_psk(c, upper.c_str(), "other", b) &&
            std::memcmp(b, want.data(), 32) == 0,
        "...in either case, and independent of the SSID");
  std::string bad(hex);
  bad[10] = 'g';
  check(!devourer::sta::pmk_from_psk(c, bad.c_str(), "IEEE", a),
        "64 characters that are not all hex are refused");

  /* And a passphrase must be 8..63 characters. */
  check(!devourer::sta::pmk_from_psk(c, "1234567", "IEEE", a),
        "a 7-character passphrase is refused");
  check(devourer::sta::pmk_from_psk(c, std::string(63, 'p').c_str(), "IEEE",
                                    a),
        "a 63-character passphrase is accepted");
  check(!devourer::sta::pmk_from_psk(c, std::string(65, 'a').c_str(), "IEEE",
                                     a),
        "a 65-character passphrase is refused");
}

/* The PTK derivation's address and nonce sorting, asserted directly. Both
 * ends derive the same key ONLY because each sorts the pair the same way, and
 * each end knows them by opposite names. */
void test_ptk_sorting() {
  OpenSslCryptoOps c;
  uint8_t pmk[32], n1[32], n2[32], p1[48], p2[48], p3[48];

  std::memset(pmk, 0x11, 32);
  std::memset(n1, 0xa0, 32);
  std::memset(n2, 0xb0, 32);

  check(devourer::sta::derive_ptk(c, pmk, kAa, kSpa, n1, n2, p1), "derive");
  check(devourer::sta::derive_ptk(c, pmk, kSpa, kAa, n1, n2, p2), "derive");
  check(std::memcmp(p1, p2, 48) == 0,
        "swapping the two addresses derives the SAME PTK");
  check(devourer::sta::derive_ptk(c, pmk, kAa, kSpa, n2, n1, p3), "derive");
  check(std::memcmp(p1, p3, 48) == 0,
        "swapping the two nonces derives the SAME PTK");

  /* And it is not simply constant: a different nonce is a different key. */
  uint8_t n3[32], p4[48];
  std::memset(n3, 0xc0, 32);
  devourer::sta::derive_ptk(c, pmk, kAa, kSpa, n1, n3, p4);
  check(std::memcmp(p1, p4, 48) != 0, "a different nonce is a different PTK");

  /* The three sections must not alias: KCK, KEK and TK are different bytes.
   * A PRF that produced 48 identical bytes would pass every test above. */
  check(std::memcmp(p1, p1 + 16, 16) != 0 &&
            std::memcmp(p1 + 16, p1 + 32, 16) != 0,
        "KCK, KEK and TK differ");
}

/* ---- the fixture authenticator -----------------------------------------
 *
 * Enough of an authenticator to drive a supplicant through everything it
 * implements, and no more: no retransmission schedule, no station table.
 */
struct Authenticator {
  OpenSslCryptoOps crypto;
  uint8_t pmk[32] = {0};
  uint8_t anonce[32] = {0};
  uint8_t ptk[48] = {0};
  uint8_t gtk[16] = {0};
  uint8_t gtk_keyid = 1;
  uint64_t replay = 0;
  bool have_ptk = false;
  /* The RSN element message 3 carries: empty = the advertised one
   * (append_rsn_ccmp_psk); `rsn_omit` leaves it out altogether. For the
   * downgrade-check cells. */
  std::vector<uint8_t> rsn_override;
  bool rsn_omit = false;

  void init() {
    devourer::sta::pmk_from_psk(crypto, kPsk, kSsid, pmk);
    std::memset(anonce, 0x5e, 32);
    std::memset(gtk, 0x31, 16);
  }

  std::vector<uint8_t> msg1() {
    replay++;
    return devourer::sta::build_eapol_key(
        devourer::sta::kKeyDescVersionCcmp | devourer::sta::kKiPairwise |
            devourer::sta::kKiAck,
        16, replay, anonce, nullptr, nullptr, 0, nullptr, nullptr);
  }

  /* Derive from the supplicant's SNonce, then verify its MIC. */
  bool on_msg2(const std::vector<uint8_t>& e) {
    EapolKey k;

    if (!devourer::sta::parse_eapol_key(e.data(), e.size(), &k)) return false;
    if (!devourer::sta::derive_ptk(crypto, pmk, kAa, kSpa, anonce, k.nonce,
                                   ptk))
      return false;
    if (devourer::sta::eapol_mic_ok(crypto, ptk, k) != MicCheck::Ok)
      return false;
    have_ptk = true;
    return true;
  }

  /* Key data: the RSN element then a GTK KDE, 802.11i-padded, AES-wrapped. */
  std::vector<uint8_t> wrapped_gtk(const uint8_t* key, uint8_t keyid) {
    std::vector<uint8_t> kd;
    if (!rsn_override.empty())
      kd = rsn_override;
    else if (!rsn_omit)
      devourer::sta::append_rsn_ccmp_psk(kd);
    const uint8_t hdr[8] = {0xdd, 0x16, 0x00, 0x0f, 0xac, 0x01, keyid, 0x00};
    kd.insert(kd.end(), hdr, hdr + 8);
    kd.insert(kd.end(), key, key + 16);
    if (kd.size() % 8) {
      kd.push_back(0xdd);
      while (kd.size() % 8) kd.push_back(0x00);
    }
    std::vector<uint8_t> w(kd.size() + 8);
    const int n = OpenSslCryptoOps::key_wrap(ptk + 16, 16, kd.data(),
                                             kd.size(), w.data());
    w.resize(n > 0 ? (size_t)n : 0);
    return w;
  }

  std::vector<uint8_t> msg3() {
    const std::vector<uint8_t> w = wrapped_gtk(gtk, gtk_keyid);

    replay++;
    return devourer::sta::build_eapol_key(
        devourer::sta::kKeyDescVersionCcmp | devourer::sta::kKiPairwise |
            devourer::sta::kKiInstall | devourer::sta::kKiAck |
            devourer::sta::kKiMic | devourer::sta::kKiSecure |
            devourer::sta::kKiEncrypted,
        16, replay, anonce, nullptr, w.data(), w.size(), &crypto, ptk);
  }

  bool on_msg4(const std::vector<uint8_t>& e) {
    EapolKey k;

    if (!devourer::sta::parse_eapol_key(e.data(), e.size(), &k)) return false;
    return devourer::sta::eapol_mic_ok(crypto, ptk, k) == MicCheck::Ok;
  }

  /* Group key handshake message 1, at a chosen replay counter so a test can
   * hand the supplicant one it has already seen. */
  std::vector<uint8_t> group1(const uint8_t* key, uint8_t keyid,
                              uint64_t at_replay,
                              const uint8_t* rsc = nullptr) {
    const std::vector<uint8_t> w = wrapped_gtk(key, keyid);

    return devourer::sta::build_eapol_key(
        devourer::sta::kKeyDescVersionCcmp | devourer::sta::kKiAck |
            devourer::sta::kKiMic | devourer::sta::kKiSecure |
            devourer::sta::kKiEncrypted,
        16, at_replay, nullptr, rsc, w.data(), w.size(), &crypto, ptk);
  }

  std::vector<uint8_t> group1_next(const uint8_t* key, uint8_t keyid,
                                   const uint8_t* rsc = nullptr) {
    replay++;
    return group1(key, keyid, replay, rsc);
  }
};

/* Run a complete four-way. Leaves both sides keyed. */
bool handshake(Authenticator& ap, Supplicant& sup, OpenSslCryptoOps& crypto) {
  uint8_t snonce[32];
  std::vector<uint8_t> out;

  std::memset(snonce, 0x7a, 32);
  ap.init();
  sup.start(crypto, ap.pmk, kSpa, kAa, snonce);

  const std::vector<uint8_t> m1 = ap.msg1();
  if (sup.on_eapol(m1.data(), m1.size(), &out) != Supplicant::Verdict::Reply)
    return false;
  if (!ap.on_msg2(out)) return false;

  const std::vector<uint8_t> m3 = ap.msg3();
  if (sup.on_eapol(m3.data(), m3.size(), &out) != Supplicant::Verdict::Reply)
    return false;
  return ap.on_msg4(out);
}

/* ---- the downgrade check (802.11-2016 12.7.6.4) ---------------------------
 *
 * Message 3 carries the AP's RSN element under the MIC and the KEK; the one
 * the station chose the BSS by came off the air unauthenticated. A station
 * given the advertisement must refuse a message 3 whose element differs -
 * and must refuse it BEFORE installing anything. */
/* `element` is a whole RSN element, EID and length included; empty means the
 * canonical one-of-each append_rsn_ccmp_psk. */
devourer::sta::RsnInfo advertised_rsn(const std::vector<uint8_t>& element = {}) {
  std::vector<uint8_t> e = element;
  if (e.empty()) devourer::sta::append_rsn_ccmp_psk(e);
  devourer::sta::RsnInfo r;
  devourer::sta::parse_rsn(e.data() + 2, e.size() - 2, &r);
  return r;
}

/* One four-way against `ap` with the advertisement passed to start();
 * returns the message 3 verdict. */
Supplicant::Verdict four_way_with_advert(Authenticator& ap, Supplicant& sup,
                                          OpenSslCryptoOps& crypto,
                                          const std::vector<uint8_t>& advert = {}) {
  uint8_t snonce[32];
  std::vector<uint8_t> out;
  std::memset(snonce, 0x7a, 32);
  ap.init();
  const devourer::sta::RsnInfo adv = advertised_rsn(advert);
  sup.start(crypto, ap.pmk, kSpa, kAa, snonce, &adv);
  const std::vector<uint8_t> m1 = ap.msg1();
  if (sup.on_eapol(m1.data(), m1.size(), &out) != Supplicant::Verdict::Reply ||
      !ap.on_msg2(out))
    return Supplicant::Verdict::CryptoError;
  const std::vector<uint8_t> m3 = ap.msg3();
  return sup.on_eapol(m3.data(), m3.size(), &out);
}

void test_rsne_downgrade_check() {
  {
    Authenticator ap;
    Supplicant sup;
    OpenSslCryptoOps crypto;
    check(four_way_with_advert(ap, sup, crypto) == Supplicant::Verdict::Reply &&
              sup.rsn_mismatches == 0 && sup.ptk_generation() == 1,
          "a message 3 carrying the ADVERTISED RSN element completes the four-way");
  }
  {
    Authenticator ap;
    Supplicant sup;
    OpenSslCryptoOps crypto;
    /* The real AP offers TKIP as well as CCMP; the advertisement the station
     * saw had been rewritten to CCMP alone. */
    ap.rsn_override = {0x30, 0x18, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04,
                       0x02, 0x00, 0x00, 0x0f, 0xac, 0x02, 0x00, 0x0f,
                       0xac, 0x04, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x02,
                       0x00, 0x00};
    const uint32_t pg = sup.ptk_generation(), gg = sup.gtk_generation();
    check(four_way_with_advert(ap, sup, crypto) == Supplicant::Verdict::Malformed &&
              sup.rsn_mismatches == 1,
          "a message 3 whose RSN element DIFFERS from the advertisement is refused");
    check(sup.ptk_generation() == pg && sup.gtk_generation() == gg && !sup.ptk_valid(),
          "...and nothing is installed");
  }
  {
    Authenticator ap;
    Supplicant sup;
    OpenSslCryptoOps crypto;
    ap.rsn_omit = true;
    check(four_way_with_advert(ap, sup, crypto) == Supplicant::Verdict::Malformed &&
              sup.rsn_mismatches == 1 && !sup.ptk_valid(),
          "a message 3 with NO RSN element is refused when an advertisement is known");
  }
  {
    Authenticator ap;
    Supplicant sup;
    OpenSslCryptoOps crypto;
    /* Same suites, different capabilities (MFP capable): still a different
     * element. */
    ap.rsn_override = {0x30, 0x14, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04,
                       0x01, 0x00, 0x00, 0x0f, 0xac, 0x04, 0x01, 0x00,
                       0x00, 0x0f, 0xac, 0x02, 0x80, 0x00};
    check(four_way_with_advert(ap, sup, crypto) == Supplicant::Verdict::Malformed &&
              sup.rsn_mismatches == 1,
          "a capabilities difference alone is a mismatch");
  }

  /* THE SUITE LISTS, NOT JUST THEIR LENGTHS. Two pairwise suites in each:
   * the advertisement offers CCMP + GCMP-128 (00-0F-AC-08), message 3 offers
   * CCMP + TKIP (00-0F-AC-02). The counts agree and CCMP is in both, so a
   * comparison of presence flags and counts alone would call them equal. */
  const std::vector<uint8_t> ccmp_gcmp = {
      0x30, 0x18, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04, 0x02, 0x00,
      0x00, 0x0f, 0xac, 0x04, 0x00, 0x0f, 0xac, 0x08,             /* CCMP, GCMP */
      0x01, 0x00, 0x00, 0x0f, 0xac, 0x02, 0x00, 0x00};
  const std::vector<uint8_t> gcmp_ccmp = {
      0x30, 0x18, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04, 0x02, 0x00,
      0x00, 0x0f, 0xac, 0x08, 0x00, 0x0f, 0xac, 0x04,             /* GCMP, CCMP */
      0x01, 0x00, 0x00, 0x0f, 0xac, 0x02, 0x00, 0x00};
  const std::vector<uint8_t> ccmp_tkip = {
      0x30, 0x18, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04, 0x02, 0x00,
      0x00, 0x0f, 0xac, 0x04, 0x00, 0x0f, 0xac, 0x02,             /* CCMP, TKIP */
      0x01, 0x00, 0x00, 0x0f, 0xac, 0x02, 0x00, 0x00};
  {
    Authenticator ap;
    Supplicant sup;
    OpenSslCryptoOps crypto;
    ap.rsn_override = ccmp_tkip;
    check(four_way_with_advert(ap, sup, crypto, ccmp_gcmp) ==
                  Supplicant::Verdict::Malformed &&
              sup.rsn_mismatches == 1 && !sup.ptk_valid(),
          "a different pairwise suite LIST with the same count is a mismatch");
  }
  {
    Authenticator ap;
    Supplicant sup;
    OpenSslCryptoOps crypto;
    ap.rsn_override = ccmp_gcmp;
    check(four_way_with_advert(ap, sup, crypto, ccmp_gcmp) ==
                  Supplicant::Verdict::Reply &&
              sup.rsn_mismatches == 0 && sup.ptk_valid(),
          "a byte-identical two-suite element is accepted");
  }
  {
    Authenticator ap;
    Supplicant sup;
    OpenSslCryptoOps crypto;
    /* The comparison is of SETS: the standard orders nothing within a
     * suite list, and wpa_supplicant's parsed comparison accepts this too. */
    ap.rsn_override = gcmp_ccmp;
    check(four_way_with_advert(ap, sup, crypto, ccmp_gcmp) ==
                  Supplicant::Verdict::Reply &&
              sup.rsn_mismatches == 0 && sup.ptk_valid(),
          "the same suites in a different order are accepted");
  }
  {
    Authenticator ap;
    Supplicant sup;
    OpenSslCryptoOps crypto;
    /* A different AKM with the same count: PSK advertised, PSK-SHA256
     * (00-0F-AC-06) in message 3. */
    ap.rsn_override = {0x30, 0x14, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04,
                       0x01, 0x00, 0x00, 0x0f, 0xac, 0x04, 0x01, 0x00,
                       0x00, 0x0f, 0xac, 0x06, 0x00, 0x00};
    check(four_way_with_advert(ap, sup, crypto) == Supplicant::Verdict::Malformed &&
              sup.rsn_mismatches == 1,
          "a different AKM with the same count is a mismatch");
  }
  {
    Authenticator ap;
    Supplicant sup;
    OpenSslCryptoOps crypto;
    /* A different group cipher: TKIP group, CCMP pairwise. */
    ap.rsn_override = {0x30, 0x14, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x02,
                       0x01, 0x00, 0x00, 0x0f, 0xac, 0x04, 0x01, 0x00,
                       0x00, 0x0f, 0xac, 0x02, 0x00, 0x00};
    check(four_way_with_advert(ap, sup, crypto) == Supplicant::Verdict::Malformed &&
              sup.rsn_mismatches == 1,
          "a different group cipher is a mismatch");
  }
}

/* parse_rsn keeps the whole element: the suite sets and the group
 * management cipher that follows the PMKID list. */
void test_parse_rsn_sets() {
  /* CCMP group; pairwise {CCMP, GCMP-128}; AKM {PSK, vendor 00-50-F2-01};
   * caps 0x0080; one PMKID; group mgmt BIP-CMAC-128 (00-0F-AC-06). */
  std::vector<uint8_t> e = {0x01, 0x00, 0x00, 0x0f, 0xac, 0x04,
                            0x02, 0x00, 0x00, 0x0f, 0xac, 0x04,
                            0x00, 0x0f, 0xac, 0x08,
                            0x02, 0x00, 0x00, 0x0f, 0xac, 0x02,
                            0x00, 0x50, 0xf2, 0x01,
                            0x80, 0x00, 0x01, 0x00};
  e.insert(e.end(), 16, 0xab);                    /* the PMKID */
  const uint8_t bip[4] = {0x00, 0x0f, 0xac, 0x06};
  e.insert(e.end(), bip, bip + 4);
  devourer::sta::RsnInfo r;
  check(devourer::sta::parse_rsn(e.data(), e.size(), &r) && r.valid,
        "rsn sets: a full element parses");
  check(r.group_suite == 0x000fac04u, "rsn sets: the group cipher selector");
  check(r.pairwise_mask == ((1u << 4) | (1u << 8)) && !r.pairwise_other,
        "rsn sets: both pairwise suites are in the set");
  check(r.akm_mask == (1u << 2) && r.akm_other,
        "rsn sets: a vendor AKM sets the other flag, not a bit");
  check(r.capabilities == 0x0080, "rsn sets: capabilities");
  check(r.group_mgmt_suite == 0x000fac06u,
        "rsn sets: the group management cipher after the PMKID list");

  /* A PMKID count that runs past the element is malformed, like the suite
   * counts - not silently skipped. */
  std::vector<uint8_t> bad(e.begin(), e.begin() + 30);
  bad[28] = 0x02;                                 /* two PMKIDs, none present */
  check(!devourer::sta::parse_rsn(bad.data(), bad.size(), &r) && !r.valid,
        "rsn sets: an overrunning PMKID count is refused");

  /* And an element that stops after the capabilities is still valid, with
   * no group management cipher: trailing fields may be omitted. */
  std::vector<uint8_t> shorter(e.begin(), e.begin() + 28);
  check(devourer::sta::parse_rsn(shorter.data(), shorter.size(), &r) &&
            r.valid && r.group_mgmt_suite == 0,
        "rsn sets: omitted trailing fields leave the group mgmt cipher absent");

  /* TRAILING PARTIAL FIELDS ARE IGNORED, not refused - wpa_supplicant's
   * shape ("ie has trailing bytes - ignored"), which leaves room for fields
   * a later amendment appends. An optional field is read only when all of it
   * is present. */
  std::vector<uint8_t> stray(e.begin(), e.begin() + 28);
  stray.push_back(0x01);                          /* half a PMKID count */
  check(devourer::sta::parse_rsn(stray.data(), stray.size(), &r) && r.valid &&
            r.capabilities == 0x0080 && r.group_mgmt_suite == 0,
        "rsn sets: a lone byte after the capabilities is ignored");
  std::vector<uint8_t> half_bip(e.begin(), e.end() - 1);  /* 3 of 4 bytes */
  check(devourer::sta::parse_rsn(half_bip.data(), half_bip.size(), &r) &&
            r.valid && r.group_mgmt_suite == 0,
        "rsn sets: a partial group mgmt cipher is ignored, not half-read");
  std::vector<uint8_t> extra = e;
  extra.push_back(0xee);
  extra.push_back(0xee);
  check(devourer::sta::parse_rsn(extra.data(), extra.size(), &r) && r.valid &&
            r.group_mgmt_suite == 0x000fac06u,
        "rsn sets: bytes after the last known field are ignored");
}

/* ---- the four-way ------------------------------------------------------ */

void test_four_way() {
  Authenticator ap;
  Supplicant sup;
  OpenSslCryptoOps crypto;

  check(handshake(ap, sup, crypto), "the four-way completes");
  check(sup.state() == Supplicant::State::Done, "the supplicant is Done");
  check(sup.ptk_valid(), "the PTK is installed");
  check(std::memcmp(sup.ptk(), ap.ptk, 48) == 0,
        "both sides derived the SAME PTK");
  check(sup.gtk_valid() && sup.gtk_len() == 16, "the GTK is installed");
  check(std::memcmp(sup.gtk(), ap.gtk, 16) == 0,
        "the GTK is the one the authenticator sent");
  check(sup.gtk_key_id() == ap.gtk_keyid,
        "the GTK's key id survives the KDE — a group frame at the wrong key id "
        "is looked up as the pairwise key and MIC-fails");
  check(sup.mic_failures == 0 && sup.replays == 0 && sup.malformed == 0,
        "a clean handshake trips no refusal counter");
}

/* ACCEPTANCE NEGATIVE 1. Message 3 carries the GTK and confirms the PTK. A
 * forged MIC must change nothing: not the state, not the installed keys, not
 * the replay counter, and it must produce no message 4 — replying would tell
 * an attacker the forgery was accepted even if the keys were not installed. */
void test_forged_mic_is_rejected() {
  Authenticator ap;
  Supplicant sup;
  OpenSslCryptoOps crypto;
  uint8_t snonce[32];
  std::vector<uint8_t> out;

  std::memset(snonce, 0x7a, 32);
  ap.init();
  sup.start(crypto, ap.pmk, kSpa, kAa, snonce);

  const std::vector<uint8_t> m1 = ap.msg1();
  check(sup.on_eapol(m1.data(), m1.size(), &out) == Supplicant::Verdict::Reply,
        "msg1 is answered");
  check(ap.on_msg2(out), "the authenticator accepts msg2");
  check(!sup.ptk_valid(), "nothing is installed before msg3");

  std::vector<uint8_t> m3 = ap.msg3();
  const uint64_t before_replay = sup.replay_counter();

  /* One bit, in the MIC field. Everything else about this frame is correct —
   * the counter advances, the ANonce matches, the key data unwraps. */
  /* THE LAST BYTE, not the first. A bit in the first eight would be caught by
   * a comparison that only looks at half the MIC, so it could not tell that
   * comparison from a correct one. */
  m3[devourer::sta::kEapolMicOff + devourer::sta::kEapolMicLen - 1] ^= 0x01;
  out.clear();
  check(sup.on_eapol(m3.data(), m3.size(), &out) ==
            Supplicant::Verdict::MicFailed,
        "A FORGED EAPOL-Key MIC IS REJECTED");
  check(sup.mic_failures == 1, "...and counted");
  check(out.empty(), "...with no message 4 sent");
  check(!sup.ptk_valid(), "...the PTK is NOT installed");
  check(!sup.gtk_valid(), "...the GTK is NOT installed");
  check(sup.state() == Supplicant::State::WaitMsg3,
        "...and the state does not advance");
  check(sup.replay_counter() == before_replay,
        "...and the replay counter does not advance, so the real msg3 still "
        "works");

  /* THE POSITIVE ARM. Without it, a supplicant that rejected everything would
   * pass the test above. The genuine message 3 must still be accepted. */
  const std::vector<uint8_t> good = ap.msg3();
  out.clear();
  check(sup.on_eapol(good.data(), good.size(), &out) ==
            Supplicant::Verdict::Reply,
        "the genuine msg3 is still accepted afterwards");
  check(sup.ptk_valid() && sup.gtk_valid(), "...and installs both keys");
  check(ap.on_msg4(out), "...and its msg4 verifies at the authenticator");
}

/* ACCEPTANCE NEGATIVE 2. Replaying a captured group message 1 must not
 * reinstall its GTK. The old key comes back with its PN space reset, so every
 * group frame sent since is encrypted again under a nonce already used — CCM
 * keystream reuse across the whole BSS, from one captured frame. */
void test_group_rekey_replay_rejected() {
  Authenticator ap;
  Supplicant sup;
  OpenSslCryptoOps crypto;
  std::vector<uint8_t> out;
  uint8_t gtk2[16], gtk3[16];

  check(handshake(ap, sup, crypto), "the four-way completes");
  std::memset(gtk2, 0x62, 16);
  std::memset(gtk3, 0x93, 16);

  /* A genuine rekey first, so the test is not measuring a supplicant that
   * refuses all group messages. */
  const std::vector<uint8_t> g1 = ap.group1_next(gtk2, 2);
  check(sup.on_eapol(g1.data(), g1.size(), &out) == Supplicant::Verdict::Reply,
        "a genuine group rekey is accepted");
  check(sup.gtk_valid() && std::memcmp(sup.gtk(), gtk2, 16) == 0,
        "...and installs the new GTK");
  check(sup.gtk_key_id() == 2, "...at its key id");
  const uint64_t after = sup.replay_counter();

  /* THE REPLAY. Byte-identical to the frame just accepted, which is what an
   * attacker captures off the air. Its MIC is valid — that is the point. */
  out.clear();
  const Supplicant::Verdict v = sup.on_eapol(g1.data(), g1.size(), &out);
  check(v == Supplicant::Verdict::Retransmit,
        "AN EQUAL-COUNTER GROUP REKEY IS NOT TREATED AS NEW");
  check(std::memcmp(sup.gtk(), gtk2, 16) == 0,
        "...the GTK is unchanged (it was already this one)");
  check(sup.replay_counter() == after, "...the counter does not move");

  /* The case that actually matters: the SAME counter carrying a DIFFERENT
   * key. This is a captured older rekey, and installing it is the
   * reinstallation - the supplicant must not take the key out of it. */
  const std::vector<uint8_t> forged = ap.group1(gtk3, 3, after);
  out.clear();
  check(sup.on_eapol(forged.data(), forged.size(), &out) ==
            Supplicant::Verdict::Retransmit,
        "AN EQUAL-COUNTER REKEY CARRYING A DIFFERENT GTK IS REFUSED");
  check(std::memcmp(sup.gtk(), gtk2, 16) == 0,
        "...AND THE OLD GTK IS NOT REINSTALLED");
  check(sup.gtk_key_id() == 2, "...nor its key id");

  /* A LOWER counter is a plain replay and is refused outright. */
  const std::vector<uint8_t> older = ap.group1(gtk3, 3, after - 1);
  out.clear();
  check(sup.on_eapol(older.data(), older.size(), &out) ==
            Supplicant::Verdict::Replayed,
        "a LOWER counter is refused outright");
  check(sup.replays == 1, "...and counted");
  check(std::memcmp(sup.gtk(), gtk2, 16) == 0, "...installing nothing");

  /* POSITIVE ARM: a strictly greater counter still works, so the rule above
   * is a window and not a wall. */
  const std::vector<uint8_t> g2 = ap.group1_next(gtk3, 3);
  out.clear();
  check(sup.on_eapol(g2.data(), g2.size(), &out) == Supplicant::Verdict::Reply,
        "a greater counter installs the new GTK");
  check(std::memcmp(sup.gtk(), gtk3, 16) == 0, "...and it is the new key");
}

/* MESSAGE 3 MUST CARRY A GTK, and a key-data field that does not yield one
 * must not complete the handshake.
 *
 * In RSN, message 3 always carries the GTK KDE. A station that accepted one
 * without it would end up Connected, keyed, and unable to decrypt a single
 * broadcast frame - with no counter moved and nothing to diagnose from. Both
 * an absent GTK KDE and a truncated one must refuse the message, which is
 * why the walker reports them as two different results. */
void test_msg3_without_a_usable_gtk_is_refused() {
  const char* names[] = {"no GTK KDE at all", "a truncated GTK KDE"};

  for (int variant = 0; variant < 2; variant++) {
    Authenticator ap;
    Supplicant sup;
    OpenSslCryptoOps crypto;
    uint8_t snonce[32];
    std::vector<uint8_t> out;
    char label[96];

    std::memset(snonce, 0x7a, 32);
    ap.init();
    sup.start(crypto, ap.pmk, kSpa, kAa, snonce);
    const std::vector<uint8_t> m1 = ap.msg1();
    sup.on_eapol(m1.data(), m1.size(), &out);
    if (!ap.on_msg2(out)) { check(false, "setup: msg2"); return; }

    /* Key data that is correctly wrapped and correctly MIC'd - the ONLY thing
     * wrong is what is inside it. */
    std::vector<uint8_t> kd;
    devourer::sta::append_rsn_ccmp_psk(kd);
    if (variant == 1) {
      /* A GTK KDE whose declared length runs past the key data. */
      const uint8_t hdr[8] = {0xdd, 0x30, 0x00, 0x0f, 0xac, 0x01, 1, 0x00};
      kd.insert(kd.end(), hdr, hdr + 8);
      kd.insert(kd.end(), ap.gtk, ap.gtk + 16);
    }
    if (kd.size() % 8) {
      kd.push_back(0xdd);
      while (kd.size() % 8) kd.push_back(0x00);
    }
    std::vector<uint8_t> w(kd.size() + 8);
    const int n = OpenSslCryptoOps::key_wrap(ap.ptk + 16, 16, kd.data(),
                                             kd.size(), w.data());
    w.resize(n > 0 ? (size_t)n : 0);
    const std::vector<uint8_t> m3 = devourer::sta::build_eapol_key(
        devourer::sta::kKeyDescVersionCcmp | devourer::sta::kKiPairwise |
            devourer::sta::kKiInstall | devourer::sta::kKiAck |
            devourer::sta::kKiMic | devourer::sta::kKiSecure |
            devourer::sta::kKiEncrypted,
        16, ap.replay + 1, ap.anonce, nullptr, w.data(), w.size(), &ap.crypto,
        ap.ptk);

    out.clear();
    std::snprintf(label, sizeof label, "msg3 with %s is refused",
                  names[variant]);
    check(sup.on_eapol(m3.data(), m3.size(), &out) ==
              Supplicant::Verdict::Malformed,
          label);
    check(!sup.ptk_valid(), "...and installs no PTK");
    check(!sup.gtk_valid(), "...and no GTK");
    check(sup.state() != Supplicant::State::Done,
          "...and does NOT complete the handshake");
    check(out.empty(), "...and sends no message 4");
  }
}

/* A retransmission is answered with the reply that answered THAT message, not
 * with whatever was cached last. Without the message-type check, a message 1
 * replayed at a message 3's counter collects a message 4. */
void test_retransmit_is_matched_to_its_message() {
  Authenticator ap;
  Supplicant sup;
  OpenSslCryptoOps crypto;
  std::vector<uint8_t> out;

  check(handshake(ap, sup, crypto), "the four-way completes");
  const uint64_t at = sup.replay_counter();

  /* A message 1 quoting the counter that message 3 was authenticated at. */
  std::vector<uint8_t> m1 = devourer::sta::build_eapol_key(
      devourer::sta::kKeyDescVersionCcmp | devourer::sta::kKiPairwise |
          devourer::sta::kKiAck,
      16, at, ap.anonce, nullptr, nullptr, 0, nullptr, nullptr);
  out.clear();
  check(sup.on_eapol(m1.data(), m1.size(), &out) ==
            Supplicant::Verdict::Replayed,
        "a msg1 at msg3's counter is a replay, not a retransmission");
  check(out.empty(), "...and collects no cached message 4");
}

/* Key material does not outlive the object. forget() is what the destructor
 * calls; asserting on a live object is the only way to see it at all. */
void test_forget_wipes_key_material() {
  Authenticator ap;
  Supplicant sup;
  OpenSslCryptoOps crypto;
  uint8_t zero[48] = {0};

  check(handshake(ap, sup, crypto), "the four-way completes");
  check(std::memcmp(sup.ptk(), zero, 48) != 0, "there is a PTK to wipe");
  sup.forget();
  check(std::memcmp(sup.ptk(), zero, 48) == 0, "forget() wipes the PTK");
  check(std::memcmp(sup.gtk(), zero, 16) == 0, "...and the GTK");
  check(!sup.ptk_valid() && !sup.gtk_valid(), "...and says so");
  check(sup.state() == Supplicant::State::Idle, "...and goes back to Idle");
}

/* A group message whose MIC does not verify installs nothing either — the
 * same rule as message 3, on the path that runs for the life of the
 * association rather than once at the start. */
void test_group_forged_mic_rejected() {
  Authenticator ap;
  Supplicant sup;
  OpenSslCryptoOps crypto;
  std::vector<uint8_t> out;
  uint8_t gtk2[16];

  check(handshake(ap, sup, crypto), "the four-way completes");
  std::memset(gtk2, 0x62, 16);

  std::vector<uint8_t> g1 = ap.group1_next(gtk2, 2);
  g1[devourer::sta::kEapolMicOff + 7] ^= 0x80;
  check(sup.on_eapol(g1.data(), g1.size(), &out) ==
            Supplicant::Verdict::MicFailed,
        "a forged group-rekey MIC is rejected");
  check(std::memcmp(sup.gtk(), ap.gtk, 16) == 0,
        "...and the GTK from the four-way is still installed");
}

/* A group rekey arriving BEFORE the four-way has finished has no key to be
 * verified with. No other cell sends one early, so this is the only thing
 * that pins the state guard - and without it the MIC check runs against an
 * all-zero PTK, which is a comparison of attacker-supplied bytes against a
 * key the station does not have. */
void test_group_rekey_before_four_way() {
  Authenticator ap;
  Supplicant sup;
  OpenSslCryptoOps crypto;
  uint8_t snonce[32], gtk2[16];
  std::vector<uint8_t> out;

  std::memset(snonce, 0x7a, 32);
  std::memset(gtk2, 0x62, 16);
  ap.init();
  sup.start(crypto, ap.pmk, kSpa, kAa, snonce);

  /* ap.ptk is still zero here, which is exactly the situation: neither side
   * has a pairwise key yet. */
  const std::vector<uint8_t> g = ap.group1(gtk2, 2, 1);
  check(sup.on_eapol(g.data(), g.size(), &out) ==
            Supplicant::Verdict::OutOfState,
        "a group rekey before the four-way is out of state");
  check(sup.out_of_state == 1, "...and counted as such, not as a MIC failure");
  check(!sup.gtk_valid(), "...installing nothing");

  /* Same again after msg1 but before msg3: a PTK is a CANDIDATE at that
   * point, not an installed key, and a group rekey must not use it. */
  const std::vector<uint8_t> m1 = ap.msg1();
  sup.on_eapol(m1.data(), m1.size(), &out);
  check(sup.state() == Supplicant::State::WaitMsg3, "msg1 was accepted");
  const std::vector<uint8_t> g2 = ap.group1(gtk2, 2, 5);
  out.clear();
  check(sup.on_eapol(g2.data(), g2.size(), &out) ==
            Supplicant::Verdict::OutOfState,
        "a group rekey between msg1 and msg3 is out of state too");
  check(!sup.gtk_valid(), "...still installing nothing");
}

/* eapol_mic_ok() refuses a descriptor version whose MIC is a different
 * algorithm, and that refusal has to be asserted HERE rather than through the
 * supplicant: on_eapol() rejects a wrong version before the MIC is ever
 * reached, so no supplicant-level cell can see the check inside eapol_mic_ok.
 * The function is public and an authenticator calls it directly. */
void test_mic_refuses_other_descriptor_versions() {
  OpenSslCryptoOps crypto;
  uint8_t kck[16];
  EapolKey k;

  std::memset(kck, 0x2b, 16);
  for (uint16_t ver : {1, 2, 3}) {
    /* build_eapol_key computes HMAC-SHA1 whatever the version says, so each
     * of these frames carries a MIC that is correct FOR THAT ALGORITHM. Only
     * version 2 declares that algorithm. */
    const std::vector<uint8_t> e = devourer::sta::build_eapol_key(
        (uint16_t)(ver | devourer::sta::kKiPairwise | devourer::sta::kKiMic),
        16, 1, nullptr, nullptr, nullptr, 0, &crypto, kck);
    char label[96];

    check(devourer::sta::parse_eapol_key(e.data(), e.size(), &k), "parses");
    std::snprintf(label, sizeof label,
                  "eapol_mic_ok %s descriptor version %u",
                  ver == 2 ? "accepts" : "refuses", ver);
    const bool ok = devourer::sta::eapol_mic_ok(crypto, kck, k) == MicCheck::Ok;
    check(ok == (ver == 2), label);
  }

  /* And the positive arm is not an accident: the wrong KCK on a version-2
   * frame still fails. */
  const std::vector<uint8_t> e = devourer::sta::build_eapol_key(
      devourer::sta::kKeyDescVersionCcmp | devourer::sta::kKiPairwise |
          devourer::sta::kKiMic,
      16, 1, nullptr, nullptr, nullptr, 0, &crypto, kck);
  devourer::sta::parse_eapol_key(e.data(), e.size(), &k);
  uint8_t other[16];
  std::memset(other, 0x99, 16);
  check(devourer::sta::eapol_mic_ok(crypto, other, k) == MicCheck::Mismatch,
        "a version-2 MIC under the wrong KCK is refused");
}

/* 12.7.6.4: message 3's ANonce must equal message 1's. A mismatch is a
 * mix-and-match of two exchanges and is refused BEFORE the MIC, so it reads
 * as what it is rather than as a key failure. */
void test_anonce_mismatch_rejected() {
  Authenticator ap;
  Supplicant sup;
  OpenSslCryptoOps crypto;
  uint8_t snonce[32];
  std::vector<uint8_t> out;

  std::memset(snonce, 0x7a, 32);
  ap.init();
  sup.start(crypto, ap.pmk, kSpa, kAa, snonce);
  const std::vector<uint8_t> m1 = ap.msg1();
  sup.on_eapol(m1.data(), m1.size(), &out);
  ap.on_msg2(out);

  std::vector<uint8_t> m3 = ap.msg3();
  m3[devourer::sta::kEapolNonceOff] ^= 0xff;
  out.clear();
  check(sup.on_eapol(m3.data(), m3.size(), &out) ==
            Supplicant::Verdict::Malformed,
        "msg3 with a different ANonce is refused");
  check(!sup.ptk_valid(), "...and installs nothing");
}

/* Message 3 with its key data in the CLEAR is a downgrade: the GTK would be
 * readable off the air. Refused rather than parsed. */
void test_unencrypted_key_data_rejected() {
  Authenticator ap;
  Supplicant sup;
  OpenSslCryptoOps crypto;
  uint8_t snonce[32];
  std::vector<uint8_t> out;

  std::memset(snonce, 0x7a, 32);
  ap.init();
  sup.start(crypto, ap.pmk, kSpa, kAa, snonce);
  const std::vector<uint8_t> m1 = ap.msg1();
  sup.on_eapol(m1.data(), m1.size(), &out);
  ap.on_msg2(out);

  /* Build msg3 with the GTK KDE unwrapped and the Encrypted bit clear, then
   * MIC it correctly — so the ONLY thing wrong is the missing encryption. */
  std::vector<uint8_t> kd;
  const uint8_t hdr[8] = {0xdd, 0x16, 0x00, 0x0f, 0xac, 0x01, 1, 0x00};
  kd.insert(kd.end(), hdr, hdr + 8);
  kd.insert(kd.end(), ap.gtk, ap.gtk + 16);
  const std::vector<uint8_t> m3 = devourer::sta::build_eapol_key(
      devourer::sta::kKeyDescVersionCcmp | devourer::sta::kKiPairwise |
          devourer::sta::kKiInstall | devourer::sta::kKiAck |
          devourer::sta::kKiMic | devourer::sta::kKiSecure,
      16, ap.replay + 1, ap.anonce, nullptr, kd.data(), kd.size(), &ap.crypto,
      ap.ptk);
  out.clear();
  check(sup.on_eapol(m3.data(), m3.size(), &out) ==
            Supplicant::Verdict::Malformed,
        "msg3 with unencrypted key data is refused");
  check(!sup.gtk_valid(), "...and the GTK is not taken from the clear");
}

/* The descriptor version selects the MIC ALGORITHM. Version 1 is HMAC-MD5 and
 * version 3 is AES-CMAC; treating either as version 2 means verifying with
 * the wrong primitive, and the only symptom would be a MIC failure pointing
 * at the key. */
void test_descriptor_version_refused() {
  Authenticator ap;
  Supplicant sup;
  OpenSslCryptoOps crypto;
  uint8_t snonce[32];
  std::vector<uint8_t> out;

  std::memset(snonce, 0x7a, 32);
  ap.init();
  sup.start(crypto, ap.pmk, kSpa, kAa, snonce);

  for (uint8_t ver : {1, 3}) {
    std::vector<uint8_t> m1 = ap.msg1();
    char label[80];

    m1[6] = (uint8_t)((m1[6] & ~0x07) | ver);
    std::snprintf(label, sizeof label, "key descriptor version %u is refused",
                  ver);
    out.clear();
    check(sup.on_eapol(m1.data(), m1.size(), &out) ==
              Supplicant::Verdict::Malformed,
          label);
  }
  check(sup.state() == Supplicant::State::WaitMsg1,
        "...and none of them started a handshake");
}

/* The replay rule on the PAIRWISE path, including the retransmission window
 * that makes it a rule rather than a wall. */
void test_replay_counter_rules() {
  Authenticator ap;
  Supplicant sup;
  OpenSslCryptoOps crypto;
  uint8_t snonce[32];
  std::vector<uint8_t> out, first;

  std::memset(snonce, 0x7a, 32);
  ap.init();
  sup.start(crypto, ap.pmk, kSpa, kAa, snonce);

  const std::vector<uint8_t> m1 = ap.msg1();
  check(sup.on_eapol(m1.data(), m1.size(), &first) ==
            Supplicant::Verdict::Reply,
        "msg1 is answered");

  /* An authenticator that does NOT increment on retransmission (802.11-2016
   * 12.7.6.4 permits either) must still be able to finish. The same reply
   * goes back out, byte for byte, and nothing is re-derived. */
  out.clear();
  check(sup.on_eapol(m1.data(), m1.size(), &out) ==
            Supplicant::Verdict::Retransmit,
        "an equal counter is answered as a retransmission");
  check(out == first, "...with the identical reply");
  check(sup.retransmits == 1, "...and counted as one");

  /* A LOWER COUNTER ON AN UNAUTHENTICATED MESSAGE 1 IS NOT REFUSED, and this
   * is deliberate. The counter in a MIC-less frame is worth nothing: a
   * supplicant that remembers it lets one forged message 1 quoting 2^64-1
   * refuse every genuine EAPOL-Key for the rest of the association. Nothing
   * is installed from a message 1, so answering a
   * low-counter one costs a message 2 and no security; refusing it would cost
   * the whole association. */
  std::vector<uint8_t> older = ap.msg1();
  devourer::sta::eapol_put_be64(older.data() + devourer::sta::kEapolReplayOff,
                                0);
  out.clear();
  check(sup.on_eapol(older.data(), older.size(), &out) ==
            Supplicant::Verdict::Reply,
        "a low-counter msg1 is answered, because its counter is unauthenticated");
  check(sup.replay_counter() == 0,
        "...and the AUTHENTICATED counter has not moved - nothing has been");

  /* ONCE SOMETHING IS AUTHENTICATED, the rule bites. Finish the handshake and
   * the same trick is refused. */
  Authenticator ap2;
  Supplicant s2;
  OpenSslCryptoOps c2;
  check(handshake(ap2, s2, c2), "a second handshake completes");
  const uint64_t authed = s2.replay_counter();
  check(authed != 0, "...and authenticated a counter");

  std::vector<uint8_t> low = ap2.msg1();
  devourer::sta::eapol_put_be64(low.data() + devourer::sta::kEapolReplayOff,
                                authed - 1);
  out.clear();
  check(s2.on_eapol(low.data(), low.size(), &out) ==
            Supplicant::Verdict::Replayed,
        "a counter below the last AUTHENTICATED one is refused");
  check(out.empty(), "...with no reply at all");
  check(s2.replays == 1, "...and counted");
}

/* A FORGED MESSAGE 1 MUST NOT MOVE THE REPLAY COUNTER.
 *
 * Message 1 has no MIC. If accepting one advances the replay counter, a
 * single forged frame quoting the top of the counter space refuses every
 * genuine EAPOL-Key from then on - and the station does not notice, because
 * it is already keyed: it stays Connected while its rekey path is dead, and
 * the failure only shows up as "multicast stopped working" at the next GTK
 * rotation. */
void test_forged_msg1_cannot_poison_the_counter() {
  Authenticator ap;
  Supplicant sup;
  OpenSslCryptoOps crypto;
  std::vector<uint8_t> out;
  uint8_t gtk2[16], ptk_before[48];

  check(handshake(ap, sup, crypto), "the four-way completes");
  std::memcpy(ptk_before, sup.ptk(), 48);
  std::memset(gtk2, 0x62, 16);

  /* Anyone can build this: it needs the BSSID, our address, and no key. */
  std::vector<uint8_t> forged = devourer::sta::build_eapol_key(
      devourer::sta::kKeyDescVersionCcmp | devourer::sta::kKiPairwise |
          devourer::sta::kKiAck,
      16, 0xffffffffffffffffULL, ap.anonce, nullptr, nullptr, 0, nullptr,
      nullptr);
  const uint64_t authed = sup.replay_counter();
  sup.on_eapol(forged.data(), forged.size(), &out);

  check(sup.replay_counter() == authed,
        "A FORGED MESSAGE 1 DOES NOT ADVANCE THE AUTHENTICATED COUNTER");
  check(sup.state() == Supplicant::State::Done,
        "...and does not take a working station out of Done");
  check(std::memcmp(sup.ptk(), ptk_before, 48) == 0,
        "...nor disturb the installed PTK");

  /* THE PROOF THAT IT MATTERS: the AP's next group rekey still works. With a
   * poisoned counter it is refused as a replay and the GTK never rotates
   * again. */
  const std::vector<uint8_t> g = ap.group1_next(gtk2, 2);
  out.clear();
  check(sup.on_eapol(g.data(), g.size(), &out) == Supplicant::Verdict::Reply,
        "...and the AP's next group rekey is still accepted");
  check(std::memcmp(sup.gtk(), gtk2, 16) == 0, "...and installs the new GTK");

  /* And the genuine four-way can still be restarted, which is the other half
   * of what a poisoned counter would break. */
  std::vector<uint8_t> m1 = ap.msg1();
  out.clear();
  check(sup.on_eapol(m1.data(), m1.size(), &out) == Supplicant::Verdict::Reply,
        "a genuine msg1 is still answered after the forgery");
}

/* Message 1 arriving on a working association must not disturb it. It is
 * unauthenticated — anyone can send one — so a station that re-derived over
 * its live PTK would lose the link to a single forged frame. */
void test_msg1_on_a_live_association() {
  Authenticator ap;
  Supplicant sup;
  OpenSslCryptoOps crypto;
  std::vector<uint8_t> out;
  uint8_t live[48], gtk_before[16];

  check(handshake(ap, sup, crypto), "the four-way completes");
  std::memcpy(live, sup.ptk(), 48);
  std::memcpy(gtk_before, sup.gtk(), 16);

  /* A forged msg1 with a fresh ANonce and an advancing counter: everything an
   * attacker can produce without the PMK. */
  std::vector<uint8_t> m1 = ap.msg1();
  m1[devourer::sta::kEapolNonceOff] ^= 0xff;
  out.clear();
  check(sup.on_eapol(m1.data(), m1.size(), &out) == Supplicant::Verdict::Reply,
        "msg1 on a live association is answered");
  check(std::memcmp(sup.ptk(), live, 48) == 0,
        "...and the INSTALLED PTK is untouched");
  check(std::memcmp(sup.gtk(), gtk_before, 16) == 0,
        "...as is the GTK");
  check(sup.ptk_valid(), "...and the station stays keyed");
}

/* KEY REINSTALLATION (KRACK, CVE-2017-13077). The AP retransmits message 3
 * whenever our message 4 is lost, and hostapd increments the replay counter
 * on every retransmission - so a STRICTLY GREATER counter carrying the key
 * already installed is routine, not an attack. It must be answered (the AP
 * is owed a message 4) and must install nothing: the caller restarts its TX
 * PN when ptk_generation() moves, and doing that under the unchanged TK
 * reuses CCMP nonces. The equal- and lower-counter cells elsewhere in this
 * file pass even for a supplicant that reinstalls here; this one does not. */
void test_msg3_retransmit_does_not_reinstall() {
  Authenticator ap;
  Supplicant sup;
  OpenSslCryptoOps crypto;
  std::vector<uint8_t> out;
  uint8_t ptk_before[48];

  check(handshake(ap, sup, crypto), "the four-way completes");
  std::memcpy(ptk_before, sup.ptk(), 48);
  const uint32_t pg = sup.ptk_generation(), gg = sup.gtk_generation();
  const uint64_t authed = sup.replay_counter();

  const std::vector<uint8_t> m3 = ap.msg3();   /* counter + 1 */
  out.clear();
  check(sup.on_eapol(m3.data(), m3.size(), &out) ==
            Supplicant::Verdict::Reply,
        "a msg3 retransmitted at a GREATER counter is answered");
  check(ap.on_msg4(out), "...with a message 4 that verifies at the AP");
  check(sup.ptk_generation() == pg,
        "...AND THE PTK IS NOT REINSTALLED (ptk_generation unchanged)");
  check(sup.gtk_generation() == gg,
        "...NOR THE GTK (gtk_generation unchanged)");
  check(std::memcmp(sup.ptk(), ptk_before, 48) == 0, "...the PTK is the same");
  check(sup.replay_counter() == authed + 1,
        "...and the authenticated counter advances past it");

  /* Retransmitted again at the counter just answered: the cached reply. */
  out.clear();
  check(sup.on_eapol(m3.data(), m3.size(), &out) ==
            Supplicant::Verdict::Retransmit,
        "the same msg3 again is a retransmission");
  check(sup.ptk_generation() == pg && sup.gtk_generation() == gg,
        "...still installing nothing");

  /* A message 1 re-quoting the INSTALLED ANonce re-derives the identical PTK
   * into the candidate. Its message 3 must not count as a new key either. */
  const std::vector<uint8_t> m1 = ap.msg1();
  out.clear();
  check(sup.on_eapol(m1.data(), m1.size(), &out) == Supplicant::Verdict::Reply,
        "a msg1 re-quoting the installed ANonce is answered");
  const std::vector<uint8_t> m3b = ap.msg3();
  out.clear();
  check(sup.on_eapol(m3b.data(), m3b.size(), &out) ==
            Supplicant::Verdict::Reply,
        "...and its msg3 is answered");
  check(sup.ptk_generation() == pg && sup.gtk_generation() == gg,
        "...WITHOUT reinstalling the identical PTK or GTK");
}

/* The group half of the same rule (CVE-2017-13078/13080). A group message 1
 * at a greater counter carrying the GTK already installed, and a PTK rekey
 * whose message 3 re-delivers the current GTK - which EVERY PTK rekey does -
 * must not move gtk_generation(), or the caller reopens its group replay
 * window each time. A genuinely new PTK must still move ptk_generation(). */
void test_group1_same_gtk_does_not_reinstall() {
  Authenticator ap;
  Supplicant sup;
  OpenSslCryptoOps crypto;
  std::vector<uint8_t> out;
  const uint8_t rsc[8] = {0x10, 0x27, 0, 0, 0, 0, 0xaa, 0xbb};
  const uint8_t rsc2[8] = {0xff, 0xff, 0, 0, 0, 0, 0, 0};
  uint8_t gtk2[16];

  check(handshake(ap, sup, crypto), "the four-way completes");
  check(sup.gtk_rsc() == 0, "the four-way's message 3 quoted RSC 0");
  const uint32_t pg = sup.ptk_generation(), gg = sup.gtk_generation();

  const std::vector<uint8_t> same = ap.group1_next(ap.gtk, ap.gtk_keyid, rsc2);
  out.clear();
  check(sup.on_eapol(same.data(), same.size(), &out) ==
            Supplicant::Verdict::Reply,
        "a group msg1 re-delivering the installed GTK is answered");
  check(sup.gtk_generation() == gg,
        "...AND THE GTK IS NOT REINSTALLED (gtk_generation unchanged)");
  check(sup.gtk_rsc() == 0, "...nor its RSC re-read");

  /* A genuinely new GTK installs, with its RSC: 48 bits, little-endian. */
  std::memset(gtk2, 0x62, 16);
  const std::vector<uint8_t> g2 = ap.group1_next(gtk2, 2, rsc);
  out.clear();
  check(sup.on_eapol(g2.data(), g2.size(), &out) == Supplicant::Verdict::Reply,
        "a new GTK is accepted");
  check(sup.gtk_generation() == gg + 1, "...and counted as one install");
  check(sup.gtk_rsc() == 0x2710,
        "...with the Key RSC it arrived with (48-bit LE, octets 6-7 ignored)");

  /* A PTK rekey: fresh ANonce, new PTK, the same (now current) GTK. */
  ap.gtk_keyid = 2;
  std::memcpy(ap.gtk, gtk2, 16);
  std::memset(ap.anonce, 0x6f, 32);
  const std::vector<uint8_t> m1 = ap.msg1();
  out.clear();
  check(sup.on_eapol(m1.data(), m1.size(), &out) == Supplicant::Verdict::Reply,
        "a PTK rekey's msg1 is answered");
  check(ap.on_msg2(out), "...and its msg2 verifies");
  const std::vector<uint8_t> m3 = ap.msg3();
  out.clear();
  check(sup.on_eapol(m3.data(), m3.size(), &out) ==
            Supplicant::Verdict::Reply,
        "the rekey's msg3 is accepted");
  check(ap.on_msg4(out), "...and its msg4 verifies under the NEW PTK");
  check(sup.ptk_generation() == pg + 1, "...a new PTK IS a new install");
  check(sup.gtk_generation() == gg + 1,
        "...but the GTK it re-delivers is NOT reinstalled");
}

/* An UNAUTHENTICATED message 1 on a live association replaces the in-flight
 * candidate, and must not stop the AP's own retransmitted message 3 - at an
 * equal OR a greater counter - from being answered. That is why the installed
 * handshake is kept apart from the candidate: a supplicant that compared the
 * retransmission against the forged candidate would refuse it as Malformed,
 * and the AP would deauthenticate after its retries. */
void test_forged_msg1_does_not_orphan_msg3_retransmit() {
  Authenticator ap;
  Supplicant sup;
  OpenSslCryptoOps crypto;
  std::vector<uint8_t> out;

  check(handshake(ap, sup, crypto), "the four-way completes");
  const uint32_t pg = sup.ptk_generation();
  const uint64_t at = sup.replay_counter();

  std::vector<uint8_t> forged = ap.msg1();
  forged[devourer::sta::kEapolNonceOff] ^= 0xff;
  devourer::sta::eapol_put_be64(forged.data() + devourer::sta::kEapolReplayOff,
                                at + 5);
  out.clear();
  sup.on_eapol(forged.data(), forged.size(), &out);

  /* Equal counter: the handshake's own msg3, rebuilt at its own counter. */
  ap.replay = at - 1;
  const std::vector<uint8_t> same = ap.msg3();   /* counter == at */
  out.clear();
  check(sup.on_eapol(same.data(), same.size(), &out) ==
            Supplicant::Verdict::Retransmit,
        "after a forged msg1, an EQUAL-counter msg3 still collects its msg4");
  check(ap.on_msg4(out), "...and it is a real message 4");

  const std::vector<uint8_t> next = ap.msg3();   /* counter == at + 1 */
  out.clear();
  check(sup.on_eapol(next.data(), next.size(), &out) ==
            Supplicant::Verdict::Reply,
        "after a forged msg1, a GREATER-counter msg3 is still answered");
  check(ap.on_msg4(out), "...with a message 4 the AP accepts");
  check(sup.malformed == 0, "...and nothing was refused as malformed");
  check(sup.ptk_generation() == pg, "...and nothing was reinstalled");
}

/* A REAL FOUR-WAY, FROM HOSTAPD AND WPA_SUPPLICANT.
 *
 * Everything else in this file is this repository talking to itself. These
 * are the four EAPOL-Key frames two independent daemons exchanged over a
 * mac80211_hwsim rig, with the PTK wpa_supplicant derived and the GTK hostapd
 * generated, captured by tests/eapol_capture_vectors.sh.
 *
 * Four separate claims, each of which fails on its own:
 *
 *   1. Our PMK from the passphrase and SSID matches what both ends used —
 *      otherwise nothing below verifies at all.
 *   2. Our PTK, derived from that PMK and the two nonces in the captured
 *      frames, equals the one WPA_SUPPLICANT LOGGED. That is the 802.11 PRF
 *      and the min/max sorting of the addresses and nonces, pinned.
 *   3. Our MIC check accepts hostapd's message 3 and wpa_supplicant's
 *      messages 2 and 4 — three MICs, two implementations, one of them the
 *      other role.
 *   4. Our Supplicant, given the captured SNonce, drives the whole exchange
 *      to Done and arrives at the same PTK and at HOSTAPD'S OWN GTK.
 */
void test_against_a_real_four_way() {
  using namespace devourer::test;
  OpenSslCryptoOps crypto;
  devourer::sta::EapolKey k1, k2, k3, k4;
  uint8_t pmk[32], ptk[48];

  check(devourer::sta::parse_eapol_key(kEapolFourWay[0].eapol,
                                       kEapolFourWay[0].len, &k1) &&
            devourer::sta::parse_eapol_key(kEapolFourWay[1].eapol,
                                           kEapolFourWay[1].len, &k2) &&
            devourer::sta::parse_eapol_key(kEapolFourWay[2].eapol,
                                           kEapolFourWay[2].len, &k3) &&
            devourer::sta::parse_eapol_key(kEapolFourWay[3].eapol,
                                           kEapolFourWay[3].len, &k4),
        "all four captured EAPOL-Key frames parse");

  /* Our reading of the key-info bits, against what the two daemons actually
   * set. If message 3 does not look like message 3 to us, nothing else in
   * this file means what it says. */
  check(k1.pairwise() && k1.ack() && !k1.has_mic(),
        "hostapd's message 1 is pairwise+ack with no MIC");
  check(k2.pairwise() && k2.has_mic() && !k2.ack() && !k2.secure(),
        "wpa_supplicant's message 2 is pairwise+MIC, not ack, not secure");
  check(k3.pairwise() && k3.ack() && k3.has_mic() && k3.install() &&
            k3.secure() && k3.encrypted(),
        "hostapd's message 3 is install+ack+MIC+secure+encrypted");
  check(k4.pairwise() && k4.has_mic() && k4.secure() && !k4.ack(),
        "wpa_supplicant's message 4 is MIC+secure");
  check(k1.version == 2 && k3.version == 2,
        "both are key descriptor version 2");

  check(devourer::sta::pmk_from_psk(crypto, kHostapdPassphrase, kHostapdSsid,
                                    pmk),
        "the PMK derives from the captured passphrase and SSID");
  check(devourer::sta::derive_ptk(crypto, pmk, kEapolAa, kEapolSpa, k1.nonce,
                                  k2.nonce, ptk),
        "the PTK derives from the two captured nonces");
  check(std::memcmp(ptk, kSupplicantPtk, 48) == 0,
        "OUR PTK EQUALS THE ONE WPA_SUPPLICANT DERIVED — the PRF and the "
        "address/nonce sorting, against an implementation that has never read "
        "this repository");

  check(devourer::sta::eapol_mic_ok(crypto, ptk, k2) == MicCheck::Ok,
        "our MIC check accepts wpa_supplicant's message 2");
  check(devourer::sta::eapol_mic_ok(crypto, ptk, k3) == MicCheck::Ok,
        "...and hostapd's message 3");
  check(devourer::sta::eapol_mic_ok(crypto, ptk, k4) == MicCheck::Ok,
        "...and wpa_supplicant's message 4");
  /* The negative arm, so the three above are not a function that returns
   * true: the same frames under a key one bit different must fail. */
  {
    uint8_t wrong[48];
    std::memcpy(wrong, ptk, 48);
    wrong[0] ^= 0x01;
    check(devourer::sta::eapol_mic_ok(crypto, wrong, k3) == MicCheck::Mismatch,
          "...and refuses message 3 under a KCK one bit out");
  }

  /* THE WHOLE EXCHANGE, through our own state machine. The SNonce is
   * wpa_supplicant's, taken from its own message 2, so our message 2 should
   * be the one it sent. */
  Supplicant sup;
  std::vector<uint8_t> out;
  sup.start(crypto, pmk, kEapolSpa, kEapolAa, k2.nonce);
  check(sup.on_eapol(kEapolFourWay[0].eapol, kEapolFourWay[0].len, &out) ==
            Supplicant::Verdict::Reply,
        "our supplicant answers hostapd's message 1");
  check(out.size() == kEapolFourWay[1].len &&
            std::memcmp(out.data(), kEapolFourWay[1].eapol, out.size()) == 0,
        "OUR MESSAGE 2 IS BYTE-FOR-BYTE THE ONE WPA_SUPPLICANT SENT");

  out.clear();
  check(sup.on_eapol(kEapolFourWay[2].eapol, kEapolFourWay[2].len, &out) ==
            Supplicant::Verdict::Reply,
        "our supplicant accepts hostapd's message 3");
  check(sup.state() == Supplicant::State::Done, "...and reaches Done");
  check(std::memcmp(sup.ptk(), kSupplicantPtk, 48) == 0,
        "...on wpa_supplicant's PTK");
  check(sup.gtk_valid() && sup.gtk_len() == 16 &&
            std::memcmp(sup.gtk(), kHostapdGtk, 16) == 0,
        "...having unwrapped HOSTAPD'S OWN GTK out of message 3");
  check(out.size() == kEapolFourWay[3].len &&
            std::memcmp(out.data(), kEapolFourWay[3].eapol, out.size()) == 0,
        "...and our message 4 is byte-for-byte the one wpa_supplicant sent");
}

/* ---- the wire format, from the air ------------------------------------- */

void test_parse_bounds() {
  Authenticator ap;
  EapolKey k;

  ap.init();
  const std::vector<uint8_t> good = ap.msg1();

  check(devourer::sta::parse_eapol_key(good.data(), good.size(), &k),
        "a well-formed EAPOL-Key parses");
  check(k.frame_len == good.size(), "...to its whole length");
  check(k.key_data_len == 0, "...with no key data");

  /* TRUNCATION THAT ONLY A LENGTH FIELD CAN CATCH. Message 1 is exactly the
   * 99-byte fixed part, so every truncation of it is refused by the
   * fixed-size check alone and a loop over it could not fail. Message 3
   * carries key data: every cut from the fixed part up is long enough for
   * that check, and must be refused by the declared lengths. */
  const std::vector<uint8_t> full = ap.msg3();
  check(full.size() >= devourer::sta::kEapolKeyFixedLen + 24 &&
            devourer::sta::parse_eapol_key(full.data(), full.size(), &k) &&
            k.key_data_len == full.size() - devourer::sta::kEapolKeyFixedLen,
        "a message 3 with key data parses, to its whole length");
  bool parsed_short = false;
  for (size_t n = devourer::sta::kEapolKeyFixedLen; n < full.size(); n++)
    if (devourer::sta::parse_eapol_key(full.data(), n, &k)) parsed_short = true;
  check(!parsed_short,
        "a frame cut anywhere in its key data is refused (by its body length)");
  /* The same cuts with the 802.1X body length REWRITTEN to match each one,
   * so the body-length check passes and only the key data length can refuse
   * the frame. */
  parsed_short = false;
  for (size_t n = devourer::sta::kEapolKeyFixedLen; n < full.size(); n++) {
    std::vector<uint8_t> t(full.begin(), full.begin() + (std::ptrdiff_t)n);
    t[2] = (uint8_t)((n - 4) >> 8);
    t[3] = (uint8_t)((n - 4) & 0xff);
    if (devourer::sta::parse_eapol_key(t.data(), t.size(), &k))
      parsed_short = true;
  }
  check(!parsed_short,
        "...and so is one whose body length was rewritten to match the cut: "
        "the key data length still disagrees");

  /* A KEY DATA LENGTH THAT LIES. This is the first thing an attacker tries,
   * because the GTK is read out of that region: claim 4096 bytes in a 99-byte
   * frame and a parser that trusts the field reads four kilobytes of the
   * heap. */
  std::vector<uint8_t> liar = good;
  liar[devourer::sta::kEapolKeyDataLenOff] = 0x10;
  liar[devourer::sta::kEapolKeyDataLenOff + 1] = 0x00;
  check(!devourer::sta::parse_eapol_key(liar.data(), liar.size(), &k),
        "a key data length longer than the frame is refused");

  /* A body length longer than the buffer is refused rather than clamped:
   * clamping would compute the MIC over fewer bytes than the sender signed. */
  std::vector<uint8_t> big = good;
  big[2] = 0x7f;
  check(!devourer::sta::parse_eapol_key(big.data(), big.size(), &k),
        "a body length longer than the buffer is refused");

  std::vector<uint8_t> wrong = good;
  wrong[1] = 0;
  check(!devourer::sta::parse_eapol_key(wrong.data(), wrong.size(), &k),
        "a non-EAPOL-Key packet type is refused");
  wrong = good;
  wrong[4] = 1;
  check(!devourer::sta::parse_eapol_key(wrong.data(), wrong.size(), &k),
        "a non-RSN key descriptor type is refused");
}

/* build_eapol_key REFUSES what its 16-bit length fields cannot say. Writing
 * the low 16 bits would produce a frame whose declared lengths disagree with
 * its bytes - and then MIC it. */
void test_build_length_limit() {
  const size_t fixed_body = devourer::sta::kEapolKeyFixedLen - 4;
  const size_t max_kd = 0xffff - fixed_body;
  std::vector<uint8_t> kd(max_kd + 1, 0x5a);

  const std::vector<uint8_t> at_max = devourer::sta::build_eapol_key(
      devourer::sta::kKeyDescVersionCcmp, 0, 1, nullptr, nullptr, kd.data(),
      max_kd, nullptr, nullptr);
  check(at_max.size() == devourer::sta::kEapolKeyFixedLen + max_kd &&
            at_max[2] == 0xff && at_max[3] == 0xff,
        "build: the largest key data whose body length fits is built");
  const std::vector<uint8_t> over = devourer::sta::build_eapol_key(
      devourer::sta::kKeyDescVersionCcmp, 0, 1, nullptr, nullptr, kd.data(),
      max_kd + 1, nullptr, nullptr);
  check(over.empty(),
        "build: one byte more is refused, not written with a wrapped length");

  /* INCONSISTENT REQUESTS ARE REFUSED. A declared key-data length with no key
   * data would ship a frame claiming bytes it does not carry; a MIC key with
   * no CryptoOps would ship unsigned what the caller asked to sign. Both null
   * is the valid unsigned frame (message 1). */
  OpenSslCryptoOps crypto;
  const uint8_t kck[16] = {1};
  check(devourer::sta::build_eapol_key(devourer::sta::kKeyDescVersionCcmp, 0,
                                       1, nullptr, nullptr, nullptr, 16,
                                       &crypto, kck)
            .empty(),
        "build: a key data length with no key data is refused");
  check(devourer::sta::build_eapol_key(devourer::sta::kKeyDescVersionCcmp, 0,
                                       1, nullptr, nullptr, kd.data(), 16,
                                       nullptr, kck)
            .empty(),
        "build: a MIC key with no CryptoOps is refused, not sent unsigned");
  check(devourer::sta::build_eapol_key(devourer::sta::kKeyDescVersionCcmp, 0,
                                       1, nullptr, nullptr, nullptr, 0,
                                       nullptr, nullptr)
                .size() == devourer::sta::kEapolKeyFixedLen,
        "build: ...while the valid unsigned frame is still built");
}

/* eapol_mic_ok checks the frame BEFORE copying it. An EapolKey that did not
 * come from parse_eapol_key may be anything, and copying first would read
 * frame_len bytes from a null pointer. */
void test_mic_ok_refuses_a_bad_key() {
  OpenSslCryptoOps crypto;
  const uint8_t kck[16] = {0};
  EapolKey k;
  k.key_info = devourer::sta::kKeyDescVersionCcmp | devourer::sta::kKiMic;
  k.version = devourer::sta::kKeyDescVersionCcmp;
  k.frame = nullptr;
  k.frame_len = 200;
  check(devourer::sta::eapol_mic_ok(crypto, kck, k) == MicCheck::Mismatch,
        "mic: a null frame is refused, not copied");
  const uint8_t shortbuf[10] = {0};
  k.frame = shortbuf;
  k.frame_len = sizeof shortbuf;
  check(devourer::sta::eapol_mic_ok(crypto, kck, k) == MicCheck::Mismatch,
        "mic: a frame shorter than the fixed part is refused");
}

/* The KDE walker, against the shapes that come out of a failed unwrap. */
void test_gtk_kde() {
  devourer::sta::GtkKde g;
  uint8_t kd[64];

  /* The exact shape a real authenticator emits: length 22 counts the OUI, the
   * data type, and 18 bytes of data. Subtracting 6 rather than 4 here makes
   * every real KDE look two bytes short, which this cell would catch. */
  const uint8_t good[24] = {0xdd, 0x16, 0x00, 0x0f, 0xac, 0x01, 0x02, 0x00,
                            1, 2, 3, 4, 5, 6, 7, 8,
                            9, 10, 11, 12, 13, 14, 15, 16};
  check(devourer::sta::find_gtk_kde(good, sizeof good, &g) ==
            devourer::sta::KdeResult::Found,
        "a GTK KDE parses");
  check(g.gtk_len == 16, "...with a 16-byte key");
  check(g.key_id == 2, "...and its key id");
  check(g.gtk[0] == 1 && g.gtk[15] == 16, "...and the key bytes");

  /* Truncated: the length runs past the buffer. Stop, do not read on - and
   * say MALFORMED, not "no GTK here", because the two mean opposite things to
   * the caller. */
  check(devourer::sta::find_gtk_kde(good, 12, &g) ==
            devourer::sta::KdeResult::Malformed,
        "a KDE truncated by the buffer is MALFORMED");

  /* A vendor OUI is not ours and must not be mistaken for a GTK. */
  std::memcpy(kd, good, sizeof good);
  kd[2] = 0x00; kd[3] = 0x50; kd[4] = 0xf2;
  check(devourer::sta::find_gtk_kde(kd, sizeof good, &g) ==
            devourer::sta::KdeResult::Absent,
        "a vendor OUI is ABSENT, not malformed - it is a well-formed KDE that "
        "simply is not ours");

  /* A declared length too small to hold a key. */
  std::memcpy(kd, good, sizeof good);
  kd[1] = 0x08;
  check(devourer::sta::find_gtk_kde(kd, sizeof good, &g) ==
            devourer::sta::KdeResult::Malformed,
        "a KDE too short for a key is MALFORMED");

  /* And no KDE at all is a clean false, not a crash: an RSN element followed
   * by 802.11i padding is exactly what message 3 carries when there is no
   * group key. */
  std::vector<uint8_t> rsn;
  devourer::sta::append_rsn_ccmp_psk(rsn);
  rsn.push_back(0xdd);
  while (rsn.size() % 8) rsn.push_back(0x00);
  check(devourer::sta::find_gtk_kde(rsn.data(), rsn.size(), &g) ==
            devourer::sta::KdeResult::Absent,
        "key data with no GTK KDE is ABSENT rather than read on");
}

}  // namespace

/* ---- hand-built key data, for the cells below ---------------------------- */

const uint16_t kMsg3Info =
    devourer::sta::kKeyDescVersionCcmp | devourer::sta::kKiPairwise |
    devourer::sta::kKiInstall | devourer::sta::kKiAck | devourer::sta::kKiMic |
    devourer::sta::kKiSecure | devourer::sta::kKiEncrypted;
const uint16_t kGroup1Info =
    devourer::sta::kKeyDescVersionCcmp | devourer::sta::kKiAck |
    devourer::sta::kKiMic | devourer::sta::kKiSecure |
    devourer::sta::kKiEncrypted;

/* A GTK KDE carrying `key_len` bytes of key. */
std::vector<uint8_t> gtk_kde(const uint8_t* key, size_t key_len, uint8_t keyid) {
  std::vector<uint8_t> k = {0xdd, (uint8_t)(6 + key_len), 0x00, 0x0f, 0xac,
                            0x01, keyid, 0x00};
  k.insert(k.end(), key, key + key_len);
  return k;
}

/* 802.11i-pad (0xdd, then zeros) and AES-wrap under the AP's KEK: key data
 * that unwraps cleanly, so whatever a cell put inside it is the only fault. */
std::vector<uint8_t> wrap_key_data(const Authenticator& ap,
                                   std::vector<uint8_t> kd) {
  if (kd.size() % 8) {
    kd.push_back(0xdd);
    while (kd.size() % 8) kd.push_back(0x00);
  }
  std::vector<uint8_t> w(kd.size() + 8);
  const int n = OpenSslCryptoOps::key_wrap(ap.ptk + 16, 16, kd.data(),
                                           kd.size(), w.data());
  w.resize(n > 0 ? (size_t)n : 0);
  return w;
}

/* An EAPOL-Key at the AP's next counter carrying exactly `kd`, MIC'd with the
 * AP's KCK - authentic in every respect a cell does not change. */
std::vector<uint8_t> authentic_with(Authenticator& ap, uint16_t info,
                                    const std::vector<uint8_t>& kd,
                                    bool pairwise_nonce) {
  const std::vector<uint8_t> w = wrap_key_data(ap, kd);
  ap.replay++;
  return devourer::sta::build_eapol_key(
      info, 16, ap.replay, pairwise_nonce ? ap.anonce : nullptr, nullptr,
      w.data(), w.size(), &ap.crypto, ap.ptk);
}

/* Start a handshake and answer message 1; the AP then holds the PTK. */
bool through_msg2(Authenticator& ap, Supplicant& sup,
                  OpenSslCryptoOps& crypto) {
  uint8_t snonce[32];
  std::vector<uint8_t> out;
  std::memset(snonce, 0x7a, 32);
  ap.init();
  sup.start(crypto, ap.pmk, kSpa, kAa, snonce);
  const std::vector<uint8_t> m1 = ap.msg1();
  if (sup.on_eapol(m1.data(), m1.size(), &out) != Supplicant::Verdict::Reply)
    return false;
  return ap.on_msg2(out);
}

/* MESSAGE 3 SETS SECURE (802.11-2016 12.7.6.4). A pairwise Ack+MIC+Install
 * frame with Secure clear is not message 3 and must install nothing. Here
 * its MIC is genuine, so the only thing wrong is the missing bit - and
 * the same frame with the bit set, next, completes the handshake. */
void test_msg3_without_secure_installs_nothing() {
  Authenticator ap;
  Supplicant sup;
  OpenSslCryptoOps crypto;
  if (!through_msg2(ap, sup, crypto)) { check(false, "setup: msg2"); return; }

  std::vector<uint8_t> kd;
  devourer::sta::append_rsn_ccmp_psk(kd);
  const std::vector<uint8_t> g = gtk_kde(ap.gtk, 16, 1);
  kd.insert(kd.end(), g.begin(), g.end());

  const uint64_t before = sup.replay_counter();
  const std::vector<uint8_t> bad = authentic_with(
      ap, (uint16_t)(kMsg3Info & ~devourer::sta::kKiSecure), kd, true);
  EapolKey k;
  check(devourer::sta::parse_eapol_key(bad.data(), bad.size(), &k) &&
            devourer::sta::eapol_mic_ok(ap.crypto, ap.ptk, k) == MicCheck::Ok,
        "no-secure msg3: setup - its MIC is genuine");
  std::vector<uint8_t> out;
  check(sup.on_eapol(bad.data(), bad.size(), &out) !=
                Supplicant::Verdict::Reply &&
            out.empty(),
        "no-secure msg3: is not answered");
  check(!sup.ptk_valid() && !sup.gtk_valid(), "no-secure msg3: installs nothing");
  check(sup.replay_counter() == before,
        "no-secure msg3: does not advance the replay counter");
  check(sup.state() != Supplicant::State::Done,
        "no-secure msg3: does not reach Done");

  const std::vector<uint8_t> good = authentic_with(ap, kMsg3Info, kd, true);
  out.clear();
  check(sup.on_eapol(good.data(), good.size(), &out) ==
                Supplicant::Verdict::Reply &&
            sup.state() == Supplicant::State::Done && ap.on_msg4(out),
        "no-secure msg3: ...while the same message with Secure set completes");
}

/* A FORGED MESSAGE 1 THAT ARRIVES FIRST must not poison the genuine one. A
 * msg1 cache matched on the counter alone would answer the genuine message 1
 * - same counter, the AP's own ANonce - with the reply derived from the
 * forger's ANonce, whose MIC the AP cannot verify, and the join would fail
 * every time the forger won the race. */
void test_forged_msg1_first_does_not_poison_the_genuine_one() {
  Authenticator ap;
  Supplicant sup;
  OpenSslCryptoOps crypto;
  uint8_t snonce[32], forged_anonce[32];
  std::vector<uint8_t> out;

  std::memset(snonce, 0x7a, 32);
  std::memset(forged_anonce, 0x66, 32);
  ap.init();
  sup.start(crypto, ap.pmk, kSpa, kAa, snonce);

  const std::vector<uint8_t> forged = devourer::sta::build_eapol_key(
      devourer::sta::kKeyDescVersionCcmp | devourer::sta::kKiPairwise |
          devourer::sta::kKiAck,
      16, ap.replay + 1, forged_anonce, nullptr, nullptr, 0, nullptr, nullptr);
  check(sup.on_eapol(forged.data(), forged.size(), &out) ==
            Supplicant::Verdict::Reply,
        "forged-first msg1: setup - the forgery is answered (msg1 has no MIC)");

  const std::vector<uint8_t> m1 = ap.msg1();     /* the same counter */
  out.clear();
  check(sup.on_eapol(m1.data(), m1.size(), &out) == Supplicant::Verdict::Reply,
        "forged-first msg1: the genuine msg1 at the same counter is processed "
        "as new, not answered from the forger's cache");
  const std::vector<uint8_t> genuine_reply = out;
  check(ap.on_msg2(out), "forged-first msg1: ...and its message 2 verifies");

  /* A true retransmission - same counter AND same ANonce - still gets the
   * cached reply. */
  std::vector<uint8_t> again;
  check(sup.on_eapol(m1.data(), m1.size(), &again) ==
                Supplicant::Verdict::Retransmit &&
            again == genuine_reply,
        "forged-first msg1: a real retransmission still gets the cached reply");

  const std::vector<uint8_t> m3 = ap.msg3();
  out.clear();
  check(sup.on_eapol(m3.data(), m3.size(), &out) ==
                Supplicant::Verdict::Reply &&
            sup.state() == Supplicant::State::Done && ap.on_msg4(out),
        "forged-first msg1: ...and the join completes");
}

/* A GTK FOLLOWED BY A TRUNCATED ELEMENT is malformed key data. A walker that
 * returned at the first GTK KDE would install the key whatever came after
 * it. Both routes a GTK arrives by. */
void test_gtk_followed_by_truncated_element_is_refused() {
  const uint8_t truncated[5] = {0xdd, 0x30, 0x00, 0x0f, 0xac};
  {
    Authenticator ap;
    Supplicant sup;
    OpenSslCryptoOps crypto;
    if (!through_msg2(ap, sup, crypto)) { check(false, "setup: msg2"); return; }
    std::vector<uint8_t> kd;
    devourer::sta::append_rsn_ccmp_psk(kd);
    const std::vector<uint8_t> g = gtk_kde(ap.gtk, 16, 1);
    kd.insert(kd.end(), g.begin(), g.end());
    kd.insert(kd.end(), truncated, truncated + sizeof truncated);
    const std::vector<uint8_t> m3 = authentic_with(ap, kMsg3Info, kd, true);
    std::vector<uint8_t> out;
    check(sup.on_eapol(m3.data(), m3.size(), &out) ==
                  Supplicant::Verdict::Malformed &&
              !sup.ptk_valid() && !sup.gtk_valid() &&
              sup.state() != Supplicant::State::Done && out.empty(),
          "trailing truncation: msg3 is refused, nothing installed, not Done");
  }
  {
    Authenticator ap;
    Supplicant sup;
    OpenSslCryptoOps crypto;
    if (!handshake(ap, sup, crypto)) { check(false, "setup: four-way"); return; }
    const uint32_t gg = sup.gtk_generation();
    uint8_t gtk2[16];
    std::memset(gtk2, 0x62, 16);
    std::vector<uint8_t> kd = gtk_kde(gtk2, 16, 2);
    kd.insert(kd.end(), truncated, truncated + sizeof truncated);
    const std::vector<uint8_t> g1 = authentic_with(ap, kGroup1Info, kd, false);
    std::vector<uint8_t> out;
    check(sup.on_eapol(g1.data(), g1.size(), &out) ==
                  Supplicant::Verdict::Malformed &&
              sup.gtk_generation() == gg &&
              std::memcmp(sup.gtk(), ap.gtk, 16) == 0 && out.empty(),
          "trailing truncation: a group rekey is refused and the old GTK stays");
  }

  /* The walker itself: truncation after the GTK, and a second GTK KDE. */
  devourer::sta::GtkKde g;
  uint8_t key16[16];
  std::memset(key16, 0x27, sizeof key16);
  std::vector<uint8_t> kd = gtk_kde(key16, 16, 1);
  kd.insert(kd.end(), truncated, truncated + sizeof truncated);
  check(devourer::sta::find_gtk_kde(kd.data(), kd.size(), &g) ==
                devourer::sta::KdeResult::Malformed &&
            g.gtk_len == 0,
        "trailing truncation: find_gtk_kde says Malformed and returns no key");
  std::vector<uint8_t> two = gtk_kde(key16, 16, 1);
  const std::vector<uint8_t> second = gtk_kde(key16, 16, 2);
  two.insert(two.end(), second.begin(), second.end());
  check(devourer::sta::find_gtk_kde(two.data(), two.size(), &g) ==
            devourer::sta::KdeResult::Malformed,
        "trailing truncation: two GTK KDEs are Malformed, not a choice");
}

/* ONLY A 16-BYTE GTK INSTALLS. The KDE format allows up to 32 (CCMP-256 and
 * GCMP-256 keys), but this station speaks CCMP-128 only, so an authenticated
 * 32-byte GTK must not install. */
void test_32_byte_gtk_is_refused() {
  uint8_t key32[32];
  std::memset(key32, 0x44, sizeof key32);
  {
    Authenticator ap;
    Supplicant sup;
    OpenSslCryptoOps crypto;
    if (!through_msg2(ap, sup, crypto)) { check(false, "setup: msg2"); return; }
    std::vector<uint8_t> kd;
    devourer::sta::append_rsn_ccmp_psk(kd);
    const std::vector<uint8_t> g = gtk_kde(key32, 32, 1);
    kd.insert(kd.end(), g.begin(), g.end());
    const std::vector<uint8_t> m3 = authentic_with(ap, kMsg3Info, kd, true);
    std::vector<uint8_t> out;
    check(sup.on_eapol(m3.data(), m3.size(), &out) ==
                  Supplicant::Verdict::Malformed &&
              !sup.ptk_valid() && !sup.gtk_valid() &&
              sup.state() != Supplicant::State::Done && out.empty(),
          "32-byte GTK: msg3 is refused, nothing installed, not Done");
  }
  {
    Authenticator ap;
    Supplicant sup;
    OpenSslCryptoOps crypto;
    if (!handshake(ap, sup, crypto)) { check(false, "setup: four-way"); return; }
    const uint32_t gg = sup.gtk_generation();
    const std::vector<uint8_t> kd = gtk_kde(key32, 32, 2);
    const std::vector<uint8_t> g1 = authentic_with(ap, kGroup1Info, kd, false);
    std::vector<uint8_t> out;
    check(sup.on_eapol(g1.data(), g1.size(), &out) ==
                  Supplicant::Verdict::Malformed &&
              sup.gtk_generation() == gg && sup.gtk_len() == 16 && out.empty(),
          "32-byte GTK: a group rekey is refused and the old GTK stays");
  }
}

/* MESSAGE 3'S KEY LENGTH IS CCMP'S: 16. Any other value names a pairwise
 * cipher this station did not negotiate; the frame is refused before its MIC
 * is checked and installs nothing. The same message at 16 then completes. */
void test_msg3_key_length_must_be_16() {
  Authenticator ap;
  Supplicant sup;
  OpenSslCryptoOps crypto;
  if (!through_msg2(ap, sup, crypto)) { check(false, "setup: msg2"); return; }

  std::vector<uint8_t> kd;
  devourer::sta::append_rsn_ccmp_psk(kd);
  const std::vector<uint8_t> g = gtk_kde(ap.gtk, 16, 1);
  kd.insert(kd.end(), g.begin(), g.end());
  const std::vector<uint8_t> w = wrap_key_data(ap, kd);

  for (uint16_t bad : {(uint16_t)0, (uint16_t)32}) {
    ap.replay++;
    const std::vector<uint8_t> m3 = devourer::sta::build_eapol_key(
        kMsg3Info, bad, ap.replay, ap.anonce, nullptr, w.data(), w.size(),
        &ap.crypto, ap.ptk);
    std::vector<uint8_t> out;
    check(sup.on_eapol(m3.data(), m3.size(), &out) ==
                  Supplicant::Verdict::Malformed &&
              !sup.ptk_valid() && !sup.gtk_valid() &&
              sup.state() != Supplicant::State::Done && out.empty(),
          bad == 0 ? "key length: a message 3 with Key Length 0 is refused"
                   : "key length: a message 3 with Key Length 32 is refused");
  }
  const std::vector<uint8_t> good = authentic_with(ap, kMsg3Info, kd, true);
  std::vector<uint8_t> out;
  check(sup.on_eapol(good.data(), good.size(), &out) ==
                Supplicant::Verdict::Reply &&
            sup.state() == Supplicant::State::Done,
        "key length: ...and the same message at 16 completes");
}

/* A GTK AT KEY ID 0 IS REFUSED. 0 is where the pairwise key is used, and a
 * data plane selecting the key by the CCMP header's key id could not tell
 * the two apart. Both routes a GTK arrives by. */
void test_gtk_key_id_zero_is_refused() {
  {
    Authenticator ap;
    Supplicant sup;
    OpenSslCryptoOps crypto;
    if (!through_msg2(ap, sup, crypto)) { check(false, "setup: msg2"); return; }
    std::vector<uint8_t> kd;
    devourer::sta::append_rsn_ccmp_psk(kd);
    const std::vector<uint8_t> g = gtk_kde(ap.gtk, 16, 0);
    kd.insert(kd.end(), g.begin(), g.end());
    const std::vector<uint8_t> m3 = authentic_with(ap, kMsg3Info, kd, true);
    std::vector<uint8_t> out;
    check(sup.on_eapol(m3.data(), m3.size(), &out) ==
                  Supplicant::Verdict::Malformed &&
              !sup.ptk_valid() && !sup.gtk_valid() &&
              sup.state() != Supplicant::State::Done && out.empty(),
          "gtk key id 0: message 3 is refused and installs nothing");
  }
  {
    Authenticator ap;
    Supplicant sup;
    OpenSslCryptoOps crypto;
    if (!handshake(ap, sup, crypto)) { check(false, "setup: four-way"); return; }
    const uint32_t gg = sup.gtk_generation();
    const uint8_t id0 = sup.gtk_key_id();
    uint8_t gtk2[16];
    std::memset(gtk2, 0x62, 16);
    const std::vector<uint8_t> g1 =
        authentic_with(ap, kGroup1Info, gtk_kde(gtk2, 16, 0), false);
    std::vector<uint8_t> out;
    check(sup.on_eapol(g1.data(), g1.size(), &out) ==
                  Supplicant::Verdict::Malformed &&
              sup.gtk_generation() == gg && sup.gtk_key_id() == id0 &&
              std::memcmp(sup.gtk(), ap.gtk, 16) == 0 && out.empty(),
          "gtk key id 0: a group rekey to key id 0 is refused, old GTK kept");
  }
}

/* KEY-DATA PADDING IS 0xDD THEN ZEROS, 1 to 7 bytes of it, and it walks as
 * elements: every length is accepted after a GTK KDE. A lone 0x00 is not
 * padding - ID 0 is a real element - so a 0x00 ahead of the KDE is read as an
 * element whose length runs off the end. */
void test_gtk_kde_padding() {
  uint8_t key[16];
  std::memset(key, 0x3c, sizeof key);
  for (size_t n = 1; n <= 7; n++) {
    std::vector<uint8_t> kd = gtk_kde(key, 16, 2);
    kd.push_back(0xdd);
    kd.insert(kd.end(), n - 1, 0x00);
    devourer::sta::GtkKde g;
    char label[80];
    std::snprintf(label, sizeof label,
                  "padding: %zu byte(s) of 0xdd+zeros after the GTK KDE", n);
    check(devourer::sta::find_gtk_kde(kd.data(), kd.size(), &g) ==
                  devourer::sta::KdeResult::Found &&
              g.key_id == 2 && g.gtk_len == 16 &&
              std::memcmp(g.gtk, key, 16) == 0,
          label);
  }
  std::vector<uint8_t> lone = {0x00};
  const std::vector<uint8_t> k = gtk_kde(key, 16, 2);
  lone.insert(lone.end(), k.begin(), k.end());
  devourer::sta::GtkKde g;
  check(devourer::sta::find_gtk_kde(lone.data(), lone.size(), &g) ==
            devourer::sta::KdeResult::Malformed,
        "padding: a lone 0x00 is an element, not padding to step over");
}

/* A CryptoOps whose HMAC fails once when armed - the PRF under derive_ptk. */
struct FlakyHmac : OpenSslCryptoOps {
  int fail_next = 0;
  bool hmac_sha1(const uint8_t* key, size_t key_len, const uint8_t* data,
                 size_t data_len, uint8_t out[20]) override {
    if (fail_next > 0) {
      fail_next--;
      return false;
    }
    return OpenSslCryptoOps::hmac_sha1(key, key_len, data, data_len, out);
  }
};

/* A FAILED DERIVATION LEAVES THE CANDIDATE AS IT WAS. A message 1 whose PTK
 * derivation fails in our own CryptoOps must not leave its ANonce beside the
 * previous handshake's PTK: the message 3 of the handshake already in flight
 * still completes. */
void test_msg1_crypto_failure_keeps_the_candidate() {
  Authenticator ap;
  Supplicant sup;
  FlakyHmac crypto;
  uint8_t snonce[32], other_anonce[32];
  std::vector<uint8_t> out;

  std::memset(snonce, 0x7a, 32);
  std::memset(other_anonce, 0x6b, 32);
  ap.init();
  sup.start(crypto, ap.pmk, kSpa, kAa, snonce);
  const std::vector<uint8_t> m1 = ap.msg1();
  if (sup.on_eapol(m1.data(), m1.size(), &out) != Supplicant::Verdict::Reply ||
      !ap.on_msg2(out)) {
    check(false, "setup: msg1/msg2");
    return;
  }

  const std::vector<uint8_t> other = devourer::sta::build_eapol_key(
      devourer::sta::kKeyDescVersionCcmp | devourer::sta::kKiPairwise |
          devourer::sta::kKiAck,
      16, ap.replay + 1, other_anonce, nullptr, nullptr, 0, nullptr, nullptr);
  crypto.fail_next = 1;
  out.clear();
  check(sup.on_eapol(other.data(), other.size(), &out) ==
                Supplicant::Verdict::CryptoError &&
            out.empty() && sup.crypto_errors == 1,
        "msg1 crypto failure: the derivation failure is a CryptoError");

  const std::vector<uint8_t> m3 = ap.msg3();
  out.clear();
  check(sup.on_eapol(m3.data(), m3.size(), &out) ==
                Supplicant::Verdict::Reply &&
            sup.state() == Supplicant::State::Done && ap.on_msg4(out),
        "msg1 crypto failure: ...and the handshake in flight still completes");
}

/* THE RSN ELEMENT IS FOUND THE WAY THE GTK KDE IS. Both searches use one
 * walk, so an empty 0xdd element (a padding marker) ahead of the RSN element
 * is stepped over, not taken as the end of the key data. */
void test_rsn_element_after_an_empty_dd() {
  Authenticator ap;
  Supplicant sup;
  OpenSslCryptoOps crypto;
  std::vector<uint8_t> rsn;
  devourer::sta::append_rsn_ccmp_psk(rsn);
  ap.rsn_override = {0xdd, 0x00};
  ap.rsn_override.insert(ap.rsn_override.end(), rsn.begin(), rsn.end());
  check(four_way_with_advert(ap, sup, crypto) == Supplicant::Verdict::Reply &&
            sup.rsn_mismatches == 0 && sup.ptk_valid(),
        "rsn after 0xdd 0x00: the RSN element is found and matches");
}

/* A FAILED HMAC IS NOT A FORGERY. When our own CryptoOps fails while checking
 * a MIC, message 3 and group message 1 are refused as CryptoError and counted
 * in crypto_errors, never in mic_failures; nothing installs, the counter does
 * not move, and the same frame is accepted once the provider works. */
void test_mic_crypto_failure_is_not_a_mic_failure() {
  Authenticator ap;
  Supplicant sup;
  FlakyHmac crypto;
  uint8_t snonce[32], gtk2[16];
  const uint8_t rsc[8] = {0x10, 0x27, 0, 0, 0, 0, 0, 0};
  std::vector<uint8_t> out;

  std::memset(snonce, 0x7a, 32);
  ap.init();
  sup.start(crypto, ap.pmk, kSpa, kAa, snonce);
  const std::vector<uint8_t> m1 = ap.msg1();
  if (sup.on_eapol(m1.data(), m1.size(), &out) != Supplicant::Verdict::Reply ||
      !ap.on_msg2(out)) {
    check(false, "setup: msg1/msg2");
    return;
  }

  const std::vector<uint8_t> m3 = ap.msg3();
  crypto.fail_next = 1;
  out.clear();
  check(sup.on_eapol(m3.data(), m3.size(), &out) ==
                Supplicant::Verdict::CryptoError &&
            out.empty() && !sup.ptk_valid(),
        "mic crypto failure: msg3 is a CryptoError and installs nothing");
  check(sup.crypto_errors == 1 && sup.mic_failures == 0,
        "mic crypto failure: ...counted in crypto_errors, not mic_failures");
  out.clear();
  check(sup.on_eapol(m3.data(), m3.size(), &out) ==
                Supplicant::Verdict::Reply &&
            ap.on_msg4(out),
        "mic crypto failure: ...and the same msg3 completes afterwards");

  const uint32_t gg = sup.gtk_generation();
  std::memset(gtk2, 0x62, 16);
  const std::vector<uint8_t> g = ap.group1_next(gtk2, 2, rsc);
  crypto.fail_next = 1;
  out.clear();
  check(sup.on_eapol(g.data(), g.size(), &out) ==
                Supplicant::Verdict::CryptoError &&
            out.empty() && sup.gtk_generation() == gg,
        "mic crypto failure: group msg1 is a CryptoError and installs nothing");
  check(sup.crypto_errors == 2 && sup.mic_failures == 0,
        "mic crypto failure: ...counted in crypto_errors, not mic_failures");
  out.clear();
  check(sup.on_eapol(g.data(), g.size(), &out) == Supplicant::Verdict::Reply &&
            sup.gtk_generation() == gg + 1,
        "mic crypto failure: ...and the same group msg1 installs afterwards");
}

int main() {
  test_psk_known_answers();
  test_msg1_crypto_failure_keeps_the_candidate();
  test_rsn_element_after_an_empty_dd();
  test_mic_crypto_failure_is_not_a_mic_failure();
  test_msg3_key_length_must_be_16();
  test_gtk_key_id_zero_is_refused();
  test_gtk_kde_padding();
  test_ptk_sorting();
  test_against_a_real_four_way();
  test_parse_bounds();
  test_build_length_limit();
  test_mic_ok_refuses_a_bad_key();
  test_gtk_kde();
  test_four_way();
  test_forged_mic_is_rejected();
  test_group_rekey_replay_rejected();
  test_group_forged_mic_rejected();
  test_msg3_without_a_usable_gtk_is_refused();
  test_retransmit_is_matched_to_its_message();
  test_forget_wipes_key_material();
  test_anonce_mismatch_rejected();
  test_unencrypted_key_data_rejected();
  test_descriptor_version_refused();
  test_mic_refuses_other_descriptor_versions();
  test_group_rekey_before_four_way();
  test_replay_counter_rules();
  test_forged_msg1_cannot_poison_the_counter();
  test_msg1_on_a_live_association();
  test_msg3_retransmit_does_not_reinstall();
  test_group1_same_gtk_does_not_reinstall();
  test_forged_msg1_does_not_orphan_msg3_retransmit();
  test_rsne_downgrade_check();
  test_parse_rsn_sets();
  test_msg3_without_secure_installs_nothing();
  test_forged_msg1_first_does_not_poison_the_genuine_one();
  test_gtk_followed_by_truncated_element_is_refused();
  test_32_byte_gtk_is_refused();

  if (g_fail) {
    std::printf("supplicant_selftest: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("supplicant_selftest: OK\n");
  return 0;
}
