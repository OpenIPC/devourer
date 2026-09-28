/* Headless guard for src/sta/StationSm.h — the association state machine.
 *
 * Authenticate, associate, four-way, connected; and every way that stops.
 *
 * THE TIMEOUTS ARE TESTABLE HERE BECAUSE THE CLOCK IS AN ARGUMENT. The AP
 * harness's equivalent retry logic reads a steady_clock, which is why its
 * schedule can only be watched on a bench, never asserted. Every
 * cell below that involves a deadline advances `now_ms` by hand.
 *
 * The AP is a fixture: it answers what a real one would, and nothing more. It
 * does not model a station table, retransmissions, or refusal for any reason
 * a cell does not ask for.
 */
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "openssl_crypto_ops.h"
#include "sta/BssTable.h"
#include "sta/StationSm.h"

namespace {

using devourer::sta::BssEntry;
using devourer::sta::BssTable;
using devourer::sta::StationSm;
using devourer::sta::MicCheck;
using devourer::test::OpenSslCryptoOps;

int g_fail = 0;

void check(bool ok, const char* what) {
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    g_fail++;
  }
}

const uint8_t kBssid[6] = {0x02, 0x42, 0x75, 0x05, 0xd6, 0x00};
const uint8_t kOwn[6] = {0x02, 0x11, 0x22, 0x33, 0x44, 0x01};
const char* kSsid = "devourerAP";
const char* kPsk = "devourer123";

std::vector<uint8_t> beacon(const uint8_t bssid[6], uint8_t chan,
                            bool rsn = true, bool ds = true) {
  static const uint8_t bcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  std::vector<uint8_t> m =
      devourer::sta::mgmt_hdr(devourer::sta::kFcBeacon, bcast, bssid, bssid);

  m.insert(m.end(), 8, 0);
  devourer::sta::put_le16(m, 100);
  /* Privacy tracks the RSN element. An open BSS that still set the bit would
   * be refused by the open path for the right reason by accident, which is
   * the sort of agreement that makes a cell unfalsifiable. */
  devourer::sta::put_le16(m, (uint16_t)(rsn ? 0x0011 : 0x0001));
  devourer::sta::append_ssid(m, kSsid);
  devourer::sta::append_supported_rates(m);
  /* `ds` false: no DS Parameter Set, as many 5 GHz beacons send it. */
  if (ds) devourer::sta::append_ds_params(m, chan);
  if (rsn) devourer::sta::append_rsn_ccmp_psk(m);
  return m;
}

/* Put a BSS in a table and hand back the entry, so join() is always reached
 * the way a real station reaches it. */
const BssEntry* discovered(BssTable& t, uint8_t chan = 6, bool rsn = true) {
  std::vector<uint8_t> b = beacon(kBssid, chan, rsn);
  return t.observe(b.data(), b.size(), -40, chan, 0);
}

/* ---- the fixture AP ---------------------------------------------------- */

struct FixtureAp {
  OpenSslCryptoOps crypto;
  uint8_t pmk[32] = {0};
  uint8_t anonce[32] = {0};
  uint8_t ptk[48] = {0};
  uint8_t gtk[16] = {0};
  uint64_t replay = 0;
  uint16_t assoc_status = 0;
  uint16_t auth_status = 0;
  uint16_t aid = 3;
  /* A real AP answers a refusal with AID 0, which means the AID check would
   * catch a refusal even if the status check were deleted. This knob makes
   * the status field the only thing that can refuse, so a cell can pin it. */
  bool aid_even_when_refused = false;
  /* An AP on an OPEN BSS sends no message 1. The knob exists so an open cell
   * can choose either: quiet, which is what a real open AP does, or noisy,
   * which is the configuration mismatch an open station must survive. */
  bool sends_msg1 = true;
  bool saw_auth = false, saw_assoc = false, saw_msg4 = false;
  /* The association request as it went out, so a cell can read the bytes
   * rather than infer them from the outcome. */
  std::vector<uint8_t> last_assoc;

  FixtureAp() {
    devourer::sta::pmk_from_psk(crypto, kPsk, kSsid, pmk);
    std::memset(anonce, 0x5e, 32);
    std::memset(gtk, 0x31, 16);
  }

  std::vector<uint8_t> mgmt(uint8_t fc) {
    return devourer::sta::mgmt_hdr(fc, kOwn, kBssid, kBssid);
  }

  /* Wrap an EAPOL body in the from-DS data frame a station receives. */
  std::vector<uint8_t> eapol_frame(const std::vector<uint8_t>& body) {
    std::vector<uint8_t> m = devourer::sta::data_hdr_from_ds(
        kOwn, kBssid, kBssid, /*protect=*/false, 1);
    devourer::sta::append_llc_snap(m, 0x888e);
    m.insert(m.end(), body.begin(), body.end());
    return m;
  }

  /* Answer one frame from the station. Returns what the AP would send back,
   * which may be empty. */
  std::vector<uint8_t> respond(const std::vector<uint8_t>& f) {
    if (f.size() < 24) return {};
    const uint8_t fc0 = f[0];

    if (fc0 == devourer::sta::kFcAuth) {
      saw_auth = true;
      std::vector<uint8_t> m = mgmt(devourer::sta::kFcAuth);
      devourer::sta::put_le16(m, 0);             /* open system */
      devourer::sta::put_le16(m, 2);             /* sequence 2 */
      devourer::sta::put_le16(m, auth_status);
      return m;
    }
    if (fc0 == devourer::sta::kFcAssocReq) {
      saw_assoc = true;
      last_assoc = f;
      std::vector<uint8_t> m = mgmt(devourer::sta::kFcAssocResp);
      devourer::sta::put_le16(m, 0x0011);
      devourer::sta::put_le16(m, assoc_status);
      devourer::sta::put_le16(
          m, (uint16_t)(assoc_status && !aid_even_when_refused
                            ? 0
                            : (0xc000 | aid)));
      return m;
    }
    if (fc0 != devourer::sta::kFcData) return {};

    /* A data frame from the station: the only one this fixture speaks is
     * EAPOL, and the only messages are 2 and 4. */
    const size_t hlen = 24;
    if (f.size() < hlen + 8 + devourer::sta::kEapolKeyFixedLen) return {};
    devourer::sta::EapolKey k;
    if (!devourer::sta::parse_eapol_key(f.data() + hlen + 8,
                                        f.size() - hlen - 8, &k))
      return {};
    if (!k.secure()) {                            /* message 2 */
      if (!devourer::sta::derive_ptk(crypto, pmk, kBssid, kOwn, anonce,
                                     k.nonce, ptk))
        return {};
      if (devourer::sta::eapol_mic_ok(crypto, ptk, k) != MicCheck::Ok)
        return {};
      return eapol_frame(msg3());
    }
    saw_msg4 = devourer::sta::eapol_mic_ok(crypto, ptk, k) == MicCheck::Ok;
    return {};
  }

  std::vector<uint8_t> msg1() {
    replay++;
    return devourer::sta::build_eapol_key(
        devourer::sta::kKeyDescVersionCcmp | devourer::sta::kKiPairwise |
            devourer::sta::kKiAck,
        16, replay, anonce, nullptr, nullptr, 0, nullptr, nullptr);
  }

  /* Non-empty: the RSN element message 3 carries instead of the advertised
   * one - the downgrade-check cell. */
  std::vector<uint8_t> rsn_override;

  std::vector<uint8_t> msg3() {
    std::vector<uint8_t> kd;
    const uint8_t hdr[8] = {0xdd, 0x16, 0x00, 0x0f, 0xac, 0x01, 1, 0x00};

    if (!rsn_override.empty())
      kd = rsn_override;
    else
      devourer::sta::append_rsn_ccmp_psk(kd);
    kd.insert(kd.end(), hdr, hdr + 8);
    kd.insert(kd.end(), gtk, gtk + 16);
    if (kd.size() % 8) {
      kd.push_back(0xdd);
      while (kd.size() % 8) kd.push_back(0x00);
    }
    std::vector<uint8_t> w(kd.size() + 8);
    const int n = OpenSslCryptoOps::key_wrap(ptk + 16, 16, kd.data(),
                                             kd.size(), w.data());
    w.resize(n > 0 ? (size_t)n : 0);
    replay++;
    return devourer::sta::build_eapol_key(
        devourer::sta::kKeyDescVersionCcmp | devourer::sta::kKiPairwise |
            devourer::sta::kKiInstall | devourer::sta::kKiAck |
            devourer::sta::kKiMic | devourer::sta::kKiSecure |
            devourer::sta::kKiEncrypted,
        16, replay, anonce, nullptr, w.data(), w.size(), &crypto, ptk);
  }

  /* Group key handshake message 1: a new GTK at `keyid`, wrapped with the
   * KEK and MIC'd with the KCK of the PTK the four-way left - what an AP
   * sends when it rotates the group key. */
  std::vector<uint8_t> group1(const uint8_t* key, uint8_t keyid) {
    std::vector<uint8_t> kd;
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
    replay++;
    return devourer::sta::build_eapol_key(
        devourer::sta::kKeyDescVersionCcmp | devourer::sta::kKiAck |
            devourer::sta::kKiMic | devourer::sta::kKiSecure |
            devourer::sta::kKiEncrypted,
        16, replay, nullptr, nullptr, w.data(), w.size(), &crypto, ptk);
  }
};

/* Pump: drain the station's transmit queue into the AP, feed the AP's answers
 * back, until nothing moves. `now_ms` does not advance, so nothing here can
 * accidentally depend on a timeout. */
void pump(StationSm& sm, FixtureAp& ap, uint32_t now_ms, int rounds = 8) {
  std::vector<uint8_t> f;

  for (int i = 0; i < rounds; i++) {
    bool moved = false;
    while (sm.pop_tx(&f)) {
      moved = true;
      const std::vector<uint8_t> r = ap.respond(f);
      if (!r.empty()) sm.on_rx(r.data(), r.size(), now_ms);
      /* The AP sends message 1 unprompted once it has associated us. */
      if (f[0] == devourer::sta::kFcAssocReq && ap.assoc_status == 0 &&
          ap.sends_msg1) {
        const std::vector<uint8_t> m1 = ap.eapol_frame(ap.msg1());
        sm.on_rx(m1.data(), m1.size(), now_ms);
      }
    }
    if (!moved) break;
  }
}

/* ---- cells ------------------------------------------------------------- */

void test_full_association() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];

  std::memset(snonce, 0x7a, 32);
  check(sm.configure(crypto, kSsid, kPsk, kOwn),
        "configure derives the PMK");
  const BssEntry* bss = table.select(kSsid);
  check(bss == nullptr, "nothing is selectable before a beacon");
  discovered(table);
  bss = table.select(kSsid);
  check(bss != nullptr, "the BSS is selectable after one beacon");
  if (!bss) return;

  check(sm.join(*bss, snonce, 0), "join starts");
  check(sm.state() == StationSm::State::Authenticating,
        "...in Authenticating");
  check(sm.pending_tx() == 1, "...with an authentication request queued");

  pump(sm, ap, 0);

  check(ap.saw_auth && ap.saw_assoc, "the AP saw both requests");
  check(sm.state() == StationSm::State::Connected, "the station connects");
  check(sm.aid() == ap.aid, "...with the AID the AP allocated");
  check(sm.keyed(), "...and is keyed");
  check(std::memcmp(sm.supplicant().ptk(), ap.ptk, 48) == 0,
        "...on the same PTK the AP derived");
  check(sm.supplicant().gtk_valid() &&
            std::memcmp(sm.supplicant().gtk(), ap.gtk, 16) == 0,
        "...and the GTK the AP sent");
  check(ap.saw_msg4, "the AP's message 4 verified");
  check(sm.auth_tx == 1 && sm.assoc_tx == 1,
        "neither request needed a retransmission");
  check(sm.eapol_rx == 2 && sm.eapol_tx == 2,
        "two EAPOL frames in, two out");
}

/* No AP at all. Three transmissions, then a give-up that names the reason —
 * not a machine that sits in Authenticating forever. */
void test_auth_timeout() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  uint8_t snonce[32];
  std::vector<uint8_t> f;

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  const BssEntry* bss = discovered(table);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);

  /* Below the deadline nothing happens: a tick is not a retransmission. */
  sm.tick(StationSm::kMgmtTimeoutMs - 1);
  check(sm.auth_tx == 1, "no retransmission before the deadline");

  uint32_t now = 0;
  for (int i = 0; i < 6; i++) {
    now += StationSm::kMgmtTimeoutMs;
    sm.tick(now);
  }
  check(sm.auth_tx == StationSm::kMaxTries,
        "exactly kMaxTries authentication requests are sent");
  check(sm.state() == StationSm::State::Failed, "then it gives up");
  check(sm.fail_reason() == StationSm::Failure::AuthTimeout,
        "...saying which step timed out");

  while (sm.pop_tx(&f)) {
  }
  /* The rule is "sends nothing further", so give it the things that would
   * make a live machine send: the authentication success it was waiting
   * for (Authenticating answers that with an association request) and a
   * message 1 (the four-way answers that with message 2), then deadlines. */
  FixtureAp ap;
  std::vector<uint8_t> ok = ap.mgmt(devourer::sta::kFcAuth);
  devourer::sta::put_le16(ok, 0);
  devourer::sta::put_le16(ok, 2);
  devourer::sta::put_le16(ok, 0);
  sm.on_rx(ok.data(), ok.size(), now + 1);
  const std::vector<uint8_t> m1 = ap.eapol_frame(ap.msg1());
  sm.on_rx(m1.data(), m1.size(), now + 2);
  for (int i = 1; i <= 4; i++) sm.tick(now + i * StationSm::kMgmtTimeoutMs);
  sm.tick(now + 100000);
  check(sm.pending_tx() == 0 && sm.assoc_tx == 0 && sm.eapol_tx == 0 &&
            sm.auth_tx == StationSm::kMaxTries,
        "a failed machine sends nothing further, whatever it hears");
  check(sm.state() == StationSm::State::Failed &&
            sm.fail_reason() == StationSm::Failure::AuthTimeout,
        "...and stays failed for the reason it failed");
}

void test_auth_refused() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  ap.auth_status = 1;                             /* unspecified failure */
  const BssEntry* bss = discovered(table);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);
  pump(sm, ap, 0);

  check(sm.state() == StationSm::State::Failed, "a refused auth fails");
  check(sm.fail_reason() == StationSm::Failure::AuthRefused, "...as refused");
  check(sm.status() == 1, "...carrying the AP's status code");
  check(!ap.saw_assoc, "...and no association is attempted");
}

void test_assoc_refused() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  ap.assoc_status = 17;                           /* cannot handle more STAs */
  const BssEntry* bss = discovered(table);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);
  pump(sm, ap, 0);

  check(sm.state() == StationSm::State::Failed, "a refused association fails");
  check(sm.fail_reason() == StationSm::Failure::AssocRefused, "...as refused");
  check(sm.status() == 17, "...carrying status 17, not a generic timeout");
}

/* THE STATUS FIELD IS WHAT REFUSES, not the AID. An AP that answers a
 * refusal with a plausible-looking AID must still be refused. Without this
 * cell the AID-zero check would do the status check's job, and deleting the
 * status check would change nothing any other cell can see. */
void test_assoc_refused_with_a_plausible_aid() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  ap.assoc_status = 12;                           /* denied, unspecified */
  ap.aid_even_when_refused = true;
  const BssEntry* bss = discovered(table);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);
  pump(sm, ap, 0);

  check(sm.state() == StationSm::State::Failed,
        "a refusal carrying an AID is still a refusal");
  check(sm.fail_reason() == StationSm::Failure::AssocRefused, "...as refused");
  check(sm.status() == 12, "...with the status the AP gave");
  check(sm.aid() == 0, "...and no AID is recorded");
  check(sm.eapol_rx == 0, "...and no handshake is attempted");
}

/* A success status with AID 0 means the AP answered yes without allocating
 * anything. Taking it would leave the station associated with an AID a TIM
 * bitmap cannot index. */
void test_assoc_success_with_zero_aid() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  ap.aid = 0;
  const BssEntry* bss = discovered(table);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);
  pump(sm, ap, 0);

  check(sm.state() == StationSm::State::Failed,
        "a success response with AID 0 is refused");
  check(sm.aid() == 0, "...and no AID is recorded");
}

void test_deauth_during_handshake() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];
  std::vector<uint8_t> f;

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  const BssEntry* bss = discovered(table);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);

  /* Auth and assoc only: stop before the four-way finishes. */
  while (sm.pop_tx(&f)) {
    const std::vector<uint8_t> r = ap.respond(f);
    if (!r.empty()) sm.on_rx(r.data(), r.size(), 0);
    if (f[0] == devourer::sta::kFcAssocReq) break;
  }
  check(sm.state() == StationSm::State::FourWay, "the four-way is running");

  std::vector<uint8_t> d = devourer::sta::build_deauth(kOwn, kBssid, 7);
  /* build_deauth builds a frame FROM the station; retarget it so it arrives
   * from the AP, which is the direction that matters here. */
  std::memcpy(d.data() + 4, kOwn, 6);
  std::memcpy(d.data() + 10, kBssid, 6);
  std::memcpy(d.data() + 16, kBssid, 6);
  sm.on_rx(d.data(), d.size(), 0);

  check(sm.state() == StationSm::State::Failed, "a deauth ends the attempt");
  check(sm.fail_reason() == StationSm::Failure::Deauthenticated, "...as such");
  check(sm.status() == 7, "...with the reason code the AP gave");
}

/* A deauth or disassoc TOO SHORT TO CARRY ITS REASON CODE is not one: it is
 * malformed, it does not end the association "with reason 0", and a
 * connected station stays connected. The full-length frame, right after,
 * still ends it - so the cell cannot pass by ignoring deauths altogether. */
void test_short_deauth_is_malformed() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  discovered(table);
  const BssEntry* bss = table.select(kSsid);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);
  pump(sm, ap, 0);
  if (!sm.keyed()) { check(false, "short deauth: the station connects first"); return; }

  std::vector<uint8_t> d = devourer::sta::build_deauth(kOwn, kBssid, 7);
  std::memcpy(d.data() + 4, kOwn, 6);
  std::memcpy(d.data() + 10, kBssid, 6);
  std::memcpy(d.data() + 16, kBssid, 6);
  const uint32_t m0 = sm.rx_malformed;
  sm.on_rx(d.data(), 25, 0);                     /* header + one reason octet */
  d[0] = devourer::sta::kFcDisassoc;
  sm.on_rx(d.data(), 24, 0);                     /* header alone */
  check(sm.state() == StationSm::State::Connected && sm.keyed(),
        "a deauth/disassoc with no room for a reason code does not end the link");
  check(sm.rx_malformed == m0 + 2, "...and both are counted malformed");

  d[0] = devourer::sta::kFcDeauth;
  sm.on_rx(d.data(), d.size(), 0);
  check(sm.state() == StationSm::State::Failed &&
            sm.fail_reason() == StationSm::Failure::Deauthenticated &&
            sm.status() == 7,
        "...while the full-length deauth still ends it, with its reason");
}

/* THE GROUP REKEY, END TO END THROUGH THE STATE MACHINE. After the four-way
 * every EAPOL frame the AP sends is protected, so on_rx() only counts it;
 * on_decrypted_msdu() is the one route by which a group key rotation reaches
 * the supplicant. Without it hostapd logged "group key handshake failed
 * (RSN) after 4 tries" and disconnected the station. The supplicant's own
 * cells test the handshake; this one tests that the machine wires it up. */
void test_group_rekey_through_the_decrypted_path() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32], gtk2[16];

  std::memset(snonce, 0x7a, 32);
  std::memset(gtk2, 0x62, 16);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  discovered(table);
  const BssEntry* bss = table.select(kSsid);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);
  pump(sm, ap, 0);
  if (!sm.keyed()) { check(false, "rekey: the four-way completes first"); return; }

  const uint32_t gg = sm.supplicant().gtk_generation();
  const uint32_t rx0 = sm.eapol_rx, tx0 = sm.eapol_tx;
  const size_t q0 = sm.pending_tx();

  std::vector<uint8_t> msdu;
  devourer::sta::append_llc_snap(msdu, 0x888e);
  const std::vector<uint8_t> g1 = ap.group1(gtk2, 2);
  msdu.insert(msdu.end(), g1.begin(), g1.end());

  std::vector<uint8_t> reply;
  check(sm.on_decrypted_msdu(msdu.data(), msdu.size(), 50, &reply),
        "rekey: a decrypted group message 1 is consumed as EAPOL");
  devourer::sta::EapolKey k;
  check(!reply.empty() &&
            devourer::sta::parse_eapol_key(reply.data(), reply.size(), &k) &&
            !k.pairwise() && k.has_mic() && k.secure() &&
            k.replay == ap.replay &&
            devourer::sta::eapol_mic_ok(ap.crypto, ap.ptk, k) == MicCheck::Ok,
        "rekey: the reply is group message 2, at the AP's counter, under the "
        "AP's KCK");
  check(sm.supplicant().gtk_generation() == gg + 1 &&
            sm.supplicant().gtk_key_id() == 2 &&
            sm.supplicant().gtk_len() == 16 &&
            std::memcmp(sm.supplicant().gtk(), gtk2, 16) == 0,
        "rekey: the new GTK is installed, under its key id");
  check(sm.eapol_rx == rx0 + 1 && sm.eapol_tx == tx0 + 1,
        "rekey: one EAPOL frame counted in and one out");
  check(sm.pending_tx() == q0,
        "rekey: the reply is handed back for encryption, never queued in the "
        "clear");
  check(sm.state() == StationSm::State::Connected && sm.keyed(),
        "rekey: the station stays connected");

  /* The AP's retransmission of the same message (its message 2 was lost):
   * answered again with the cached reply, and nothing reinstalled. */
  std::vector<uint8_t> again;
  check(sm.on_decrypted_msdu(msdu.data(), msdu.size(), 60, &again) &&
            again == reply && sm.supplicant().gtk_generation() == gg + 1,
        "rekey: an equal-counter repeat gets the cached reply and installs "
        "nothing");

  /* Not EAPOL: handed back to the caller as data, untouched. */
  std::vector<uint8_t> ip;
  devourer::sta::append_llc_snap(ip, 0x0800);
  ip.insert(ip.end(), 20, 0x45);
  std::vector<uint8_t> none = {1};
  check(!sm.on_decrypted_msdu(ip.data(), ip.size(), 70, &none) &&
            none.size() == 1,
        "rekey: a non-EAPOL MSDU is not consumed");
}

/* Every frame must come from the BSS being talked to. Without the addr2
 * check, any AP on the channel drives this machine — including one sending
 * association responses to somebody else. */
void test_frames_from_elsewhere_are_ignored() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  const BssEntry* bss = discovered(table);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);

  /* A perfectly good authentication response, from the wrong AP. */
  std::vector<uint8_t> m = ap.mgmt(devourer::sta::kFcAuth);
  devourer::sta::put_le16(m, 0);
  devourer::sta::put_le16(m, 2);
  devourer::sta::put_le16(m, 0);
  m[10] ^= 0xff;                                  /* addr2: another BSSID */
  sm.on_rx(m.data(), m.size(), 0);
  check(sm.state() == StationSm::State::Authenticating,
        "an auth response from another BSSID does not advance the machine");

  /* And a deauth from a third party must not tear anything down. */
  std::vector<uint8_t> d = devourer::sta::build_deauth(kOwn, kBssid, 7);
  std::memcpy(d.data() + 4, kOwn, 6);
  std::memcpy(d.data() + 10, kBssid, 6);
  d[10] ^= 0xff;
  sm.on_rx(d.data(), d.size(), 0);
  check(sm.state() == StationSm::State::Authenticating,
        "a deauth from another BSSID is ignored");

  /* Addressed to a different station, from the right AP. */
  std::vector<uint8_t> n = ap.mgmt(devourer::sta::kFcAuth);
  devourer::sta::put_le16(n, 0);
  devourer::sta::put_le16(n, 2);
  devourer::sta::put_le16(n, 0);
  /* THE LAST OCTET of addr1. Flipping byte 0 sets the group bit and makes the
   * frame a BROADCAST, which is addressed to this station as much as to
   * anyone - it then survives the unicast filter and is dropped only by the
   * `if (to_us)` inside the auth branch, so deleting the filter would not
   * change the outcome. Flipping the last octet keeps the frame unicast, so
   * only the address filter can drop it. */
  const uint32_t before = sm.rx_not_for_us;
  n[9] ^= 0xff;
  sm.on_rx(n.data(), n.size(), 0);
  check(sm.state() == StationSm::State::Authenticating,
        "an auth response for another station is ignored");
  check(sm.rx_not_for_us == before + 1,
        "...by the address filter, which counted it");
}

/* The four-way is never protected. A PROTECTED frame claiming to be EAPOL
 * cannot be one, because the keys it carries are what protection would need. */
/* A protected data frame is the caller's to decrypt, and is counted apart
 * from a protocol error. Counted in rx_ignored, it would be EVERY data frame
 * on a working link - a 60-second on-air run carrying 75 frames reads as
 * ignored=75 - and that counter set is the one thing that answers "why did
 * nothing associate". */
void test_protected_data_is_counted_apart() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  discovered(table);
  const BssEntry* bss = table.select(kSsid);
  if (!bss) { check(false, "BSS discovered"); return; }
  sm.join(*bss, snonce, 0);
  pump(sm, ap, 0);
  check(sm.state() == StationSm::State::Connected, "connected");

  const uint32_t ignored = sm.rx_ignored;
  std::vector<uint8_t> f = devourer::sta::data_hdr_from_ds(
      kOwn, kBssid, kBssid, /*protect=*/true, 7);
  f.insert(f.end(), 40, 0x11);
  sm.on_rx(f.data(), f.size(), 0);
  check(sm.rx_protected == 1, "a protected data frame is counted as protected");
  check(sm.rx_ignored == ignored, "...and NOT as ignored");
}

/* A FRAGMENT AND AN A-MSDU ARE NOT MSDUs, and this machine reassembles
 * neither. The bytes at the LLC offset are a piece of a frame, or a subframe
 * header - so feeding them to the EAPOL parser asks it to read the wrong
 * bytes. The caller's data plane refuses both; the two receive layers
 * disagreeing about it is how a later reader closes the gap in one place. */
void test_fragmented_and_amsdu_eapol_are_refused() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  discovered(table);
  const BssEntry* bss = table.select(kSsid);
  if (!bss) { check(false, "BSS discovered"); return; }
  sm.join(*bss, snonce, 0);
  /* Authenticate and associate, and stop in FourWay: the clear carries
   * EAPOL-Key messages 1 and 3 only before the PTK is installed. */
  std::vector<uint8_t> f;
  while (sm.pop_tx(&f)) {
    const std::vector<uint8_t> r = ap.respond(f);
    if (!r.empty()) sm.on_rx(r.data(), r.size(), 0);
    if (f[0] == devourer::sta::kFcAssocReq) break;
  }
  check(sm.state() == StationSm::State::FourWay, "in the four-way");

  /* A well-formed EAPOL frame is the control: the arms below must differ
   * from it by exactly the bit under test. */
  const uint32_t rx_before = sm.eapol_rx;
  std::vector<uint8_t> good = ap.eapol_frame(ap.msg1());
  sm.on_rx(good.data(), good.size(), 0);
  check(sm.eapol_rx == rx_before + 1, "a whole EAPOL frame reaches the supplicant");

  uint32_t bad_before = sm.rx_malformed;
  std::vector<uint8_t> frag = ap.eapol_frame(ap.msg1());
  frag[1] |= devourer::sta::kFcMoreFrag;
  sm.on_rx(frag.data(), frag.size(), 0);
  check(sm.rx_malformed == bad_before + 1, "a More Fragments EAPOL is refused");
  check(sm.eapol_rx == rx_before + 1, "...and never reaches the supplicant");

  /* THE LAST FRAGMENT HAS MoreFrag CLEAR. */
  bad_before = sm.rx_malformed;
  std::vector<uint8_t> last = ap.eapol_frame(ap.msg1());
  last[22] = 0x02;                        /* fragment number 2 */
  sm.on_rx(last.data(), last.size(), 0);
  check(sm.rx_malformed == bad_before + 1, "...and so is a LAST fragment");

  /* An A-MSDU. The bit lives in the QoS Control field, so the frame has to
   * be a QoS one - which is why no non-QoS arm can reach this branch. */
  bad_before = sm.rx_malformed;
  std::vector<uint8_t> amsdu = ap.eapol_frame(ap.msg1());
  amsdu[0] = 0x88;                        /* QoS Data */
  amsdu.insert(amsdu.begin() + 24, {0x80, 0x00});   /* QoS Control: A-MSDU */
  sm.on_rx(amsdu.data(), amsdu.size(), 0);
  check(sm.rx_malformed == bad_before + 1, "an A-MSDU is refused");
  check(sm.eapol_rx == rx_before + 1, "...and never reaches the supplicant");

  /* The control for THAT: the same QoS frame without the bit does get
   * through, so the arm is about the A-MSDU bit and not about QoS. */
  std::vector<uint8_t> qos = ap.eapol_frame(ap.msg1());
  qos[0] = 0x88;
  qos.insert(qos.begin() + 24, {0x00, 0x00});
  sm.on_rx(qos.data(), qos.size(), 0);
  check(sm.eapol_rx == rx_before + 2, "...while a plain QoS EAPOL does");
}

void test_protected_eapol_ignored() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];
  std::vector<uint8_t> f;

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  const BssEntry* bss = discovered(table);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);
  while (sm.pop_tx(&f)) {
    const std::vector<uint8_t> r = ap.respond(f);
    if (!r.empty()) sm.on_rx(r.data(), r.size(), 0);
    if (f[0] == devourer::sta::kFcAssocReq) break;
  }

  std::vector<uint8_t> m1 = ap.eapol_frame(ap.msg1());
  m1[1] |= devourer::sta::kFcProtected;
  sm.on_rx(m1.data(), m1.size(), 0);
  check(sm.eapol_rx == 0, "a protected EAPOL frame is not fed to the supplicant");
  check(sm.pending_tx() == 0, "...and nothing is answered");

  /* The same frame unprotected IS accepted — so the cell above is testing the
   * protected bit and not something else about the frame. */
  m1[1] &= (uint8_t)~devourer::sta::kFcProtected;
  sm.on_rx(m1.data(), m1.size(), 0);
  check(sm.eapol_rx == 1, "the same frame unprotected is accepted");
  check(sm.pending_tx() == 1, "...and answered");
}

/* The four-way give-up. The authenticator owns the retransmission schedule,
 * so this side must not sit in FourWay forever when it stops. */
void test_handshake_timeout() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];
  std::vector<uint8_t> f;

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  const BssEntry* bss = discovered(table);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);
  while (sm.pop_tx(&f)) {
    const std::vector<uint8_t> r = ap.respond(f);
    if (!r.empty()) sm.on_rx(r.data(), r.size(), 0);
    if (f[0] == devourer::sta::kFcAssocReq) break;
  }
  check(sm.state() == StationSm::State::FourWay, "the four-way is running");

  sm.tick(StationSm::kHandshakeTimeoutMs - 1);
  check(sm.state() == StationSm::State::FourWay, "it waits out its deadline");
  sm.tick(StationSm::kHandshakeTimeoutMs);
  check(sm.state() == StationSm::State::Failed, "then gives up");
  check(sm.fail_reason() == StationSm::Failure::HandshakeTimeout,
        "...saying the handshake timed out, not the association");
}

/* AN AUTHENTICATOR THAT ONLY RETRANSMITS IS NOT MAKING PROGRESS. Answering a
 * retransmitted message 1 is correct, but letting it push the give-up
 * deadline out means a stuck AP holds this state open forever — a station
 * that never connects and never tries anything else. */
void test_retransmission_does_not_extend_the_deadline() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];
  std::vector<uint8_t> f;

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  const BssEntry* bss = discovered(table);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);
  while (sm.pop_tx(&f)) {
    const std::vector<uint8_t> r = ap.respond(f);
    if (!r.empty()) sm.on_rx(r.data(), r.size(), 0);
    if (f[0] == devourer::sta::kFcAssocReq) break;
  }
  check(sm.state() == StationSm::State::FourWay, "the four-way is running");

  /* The AP's message 1, over and over, at the same replay counter. */
  const std::vector<uint8_t> m1 = ap.eapol_frame(ap.msg1());
  sm.on_rx(m1.data(), m1.size(), 0);
  while (sm.pop_tx(&f)) {
  }
  for (uint32_t t = 500; t < StationSm::kHandshakeTimeoutMs; t += 500) {
    sm.on_rx(m1.data(), m1.size(), t);
    while (sm.pop_tx(&f)) {
    }
    sm.tick(t);
  }
  check(sm.supplicant().retransmits > 0,
        "the retransmissions were seen as retransmissions");
  check(sm.state() == StationSm::State::FourWay,
        "...and the machine is still waiting");

  sm.tick(StationSm::kHandshakeTimeoutMs);
  check(sm.state() == StationSm::State::Failed,
        "the give-up fires on time DESPITE the retransmissions");
  check(sm.fail_reason() == StationSm::Failure::HandshakeTimeout,
        "...as a handshake timeout");
}

/* JOINING A SECOND BSS MUST NOT AIR THE FIRST ONE'S FRAMES.
 *
 * join() reset everything except the transmit queue, so auth requests still
 * queued for the BSS we gave up on went out at the one we just joined -
 * addressed to the old BSSID, after the radio had retuned to the new channel.
 * Every other cell here drains the queue between steps, which is exactly why
 * none of them saw it. */
void test_join_clears_the_transmit_queue() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  uint8_t snonce[32];
  std::vector<uint8_t> f;
  const uint8_t kOther[6] = {0x02, 0x42, 0x75, 0x05, 0xd6, 0x99};

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  const BssEntry* bss = discovered(table);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);

  /* Three unanswered authentication requests pile up, unread. */
  uint32_t now = 0;
  for (int i = 0; i < 2; i++) { now += StationSm::kMgmtTimeoutMs; sm.tick(now); }
  check(sm.pending_tx() == 3, "three auth requests are queued for the first BSS");

  BssEntry other = *bss;
  std::memcpy(other.info.bssid, kOther, 6);
  sm.join(other, snonce, now);
  check(sm.pending_tx() == 1,
        "join() clears the queue - only the new BSS's request is pending");
  check(sm.pop_tx(&f) && std::memcmp(f.data() + 4, kOther, 6) == 0,
        "...and it is addressed to the BSS we actually joined");
}

/* THE QUEUE IS BOUNDED. Every frame in it is produced in answer to a received
 * one, so an unbounded queue is an unbounded allocation an attacker controls:
 * one captured EAPOL frame replayed at the current counter is answered every
 * time. */
void test_transmit_queue_is_bounded() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];
  std::vector<uint8_t> f;

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  const BssEntry* bss = discovered(table);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);
  while (sm.pop_tx(&f)) {
    const std::vector<uint8_t> r = ap.respond(f);
    if (!r.empty()) sm.on_rx(r.data(), r.size(), 0);
    if (f[0] == devourer::sta::kFcAssocReq) break;
  }
  const std::vector<uint8_t> m1 = ap.eapol_frame(ap.msg1());
  sm.on_rx(m1.data(), m1.size(), 0);

  /* The caller never drains. An attacker replays the same frame. */
  for (int i = 0; i < 2000; i++) sm.on_rx(m1.data(), m1.size(), 0);
  check(sm.pending_tx() <= StationSm::tx_capacity(),
        "the transmit queue never exceeds its capacity");
  check(sm.tx_dropped > 0, "...and the drops are counted, not silent");
}

/* CONNECTED HAS AN EXIT. Without beacon supervision the only way out is a
 * deauth from an AP that may have been switched off, and the caller sees
 * keyed() forever. */
void test_beacon_loss() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  const BssEntry* bss = discovered(table);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);
  pump(sm, ap, 0);
  check(sm.state() == StationSm::State::Connected, "the station connects");

  /* Beacons keep arriving: nothing happens, however long it runs. */
  uint32_t now = 0;
  for (int i = 0; i < 20; i++) {
    now += StationSm::kBeaconLossMs / 2;
    std::vector<uint8_t> b = beacon(kBssid, 6);
    sm.on_rx(b.data(), b.size(), now);
    sm.tick(now);
  }
  check(sm.state() == StationSm::State::Connected,
        "a beaconing AP keeps the station connected indefinitely");
  check(sm.beacons_rx == 20, "...and the beacons are counted");

  /* They stop. */
  sm.tick(now + StationSm::kBeaconLossMs - 1);
  check(sm.state() == StationSm::State::Connected, "...it waits out the window");
  sm.tick(now + StationSm::kBeaconLossMs);
  check(sm.state() == StationSm::State::Failed, "a silent AP ends the link");
  check(sm.fail_reason() == StationSm::Failure::BeaconLost,
        "...saying the beacon was lost, not that the handshake timed out");
  check(!sm.keyed(), "...and keyed() stops claiming a link that is gone");
}

/* leave() tells the AP rather than letting it time the station out - which on
 * this project's own AP holds an AID and one of seven table slots. */
void test_leave() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];
  std::vector<uint8_t> f;

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  const BssEntry* bss = discovered(table);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);
  pump(sm, ap, 0);
  check(sm.state() == StationSm::State::Connected, "the station connects");

  sm.leave(3);
  check(sm.state() == StationSm::State::Idle, "leave() goes back to Idle");
  check(!sm.keyed(), "...and drops the keys");
  check(sm.aid() == 0, "...and the AID");
  check(sm.pop_tx(&f), "...having queued a frame");
  check(f[0] == devourer::sta::kFcDeauth, "...which is a deauthentication");
  check(std::memcmp(f.data() + 4, kBssid, 6) == 0, "...addressed to the AP");
  sm.leave(3);
  check(sm.pending_tx() == 0, "leaving twice sends nothing the second time");
}

/* The RX filter counts what it discards. On hardware this is the only address
 * filter in the system, so "it did not associate" must come with a number
 * saying whether the AP was ever heard. */
void test_rx_counters() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  const BssEntry* bss = discovered(table);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);

  std::vector<uint8_t> m = ap.mgmt(devourer::sta::kFcAuth);
  devourer::sta::put_le16(m, 0);
  devourer::sta::put_le16(m, 2);
  devourer::sta::put_le16(m, 0);

  std::vector<uint8_t> foreign = m;
  foreign[10] ^= 0xff;
  sm.on_rx(foreign.data(), foreign.size(), 0);
  check(sm.rx_not_our_bss == 1, "a frame from another BSS is counted");

  std::vector<uint8_t> elsewhere = m;
  /* The LAST octet of addr1, not the first: flipping byte 0 sets the
   * group bit and turns the frame into a broadcast, which is addressed to
   * this station as much as to anyone, so the cell would measure the wrong
   * counter. */
  elsewhere[9] ^= 0xff;
  sm.on_rx(elsewhere.data(), elsewhere.size(), 0);
  check(sm.rx_not_for_us == 1, "a frame for another station is counted");

  /* Somebody else's data traffic, correctly addressed to us: not an error,
   * but it must not be confused with one. */
  std::vector<uint8_t> d = devourer::sta::data_hdr_from_ds(
      kOwn, kBssid, kBssid, /*protect=*/false, 1);
  devourer::sta::append_llc_snap(d, 0x0800);
  d.insert(d.end(), 20, 0x41);
  sm.on_rx(d.data(), d.size(), 0);
  check(sm.rx_ignored == 1, "a non-EAPOL data frame is counted separately");
  check(sm.rx_not_our_bss == 1 && sm.rx_not_for_us == 1,
        "...and does not move the address counters");
}

/* join() must refuse a BSS this station cannot finish with, rather than
 * authenticating and discovering it three frames later. */
void test_join_refuses_an_unusable_bss() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  uint8_t snonce[32];

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);

  BssEntry open{};
  std::memcpy(open.info.bssid, kBssid, 6);
  open.info.ssid = kSsid;
  open.info.rsn_ccmp_psk = false;
  check(!sm.join(open, snonce, 0), "an open BSS is refused by join()");
  check(sm.state() == StationSm::State::Failed, "...and says so");
  check(sm.pending_tx() == 0, "...without sending anything");
}

/* A CryptoOps whose PBKDF2 refuses, which is the only way to reach the NoPmk
 * branch: OpenSSL's does not fail for any passphrase a caller can supply.
 * Without it that branch is unreachable from this file and deleting it costs
 * nothing - which is what "the test could not fail" means. */
struct NoPbkdf2Crypto : OpenSslCryptoOps {
  bool pbkdf2_sha1(const char*, const uint8_t*, size_t, unsigned, uint8_t*,
                   size_t) override {
    return false;
  }
};

/* configure() reports the failure, and join() must then name it NoPmk rather
 * than authenticating at an AP it can never finish a handshake with. */
void test_no_pmk() {
  NoPbkdf2Crypto crypto;
  BssTable table;
  StationSm sm;
  uint8_t snonce[32];

  std::memset(snonce, 0x7a, 32);
  check(!sm.configure(crypto, kSsid, kPsk, kOwn),
        "configure reports a failed PMK derivation");
  discovered(table);
  const BssEntry* bss = table.select(kSsid);
  if (!bss) { check(false, "BSS discovered"); return; }
  check(!sm.join(*bss, snonce, 0), "join refuses");
  check(sm.fail_reason() == StationSm::Failure::NoPmk,
        "...naming NoPmk, not NotConfigured");
  check(sm.pending_tx() == 0, "...without airing an authentication request");
}

/* ---- the open-network path ---------------------------------------------
 *
 * WHY IT EXISTS: without it a station that never reaches Connected cannot
 * say whether the failure is in authentication/association or in the key
 * exchange, because on a WPA2 BSS the two halves come up together or not at
 * all. An on-air harness can run an `open` cell first for exactly that
 * reason, mirroring the AP side's ap_responder/ap_wpa2 ladder.
 */
void test_open_association() {
  BssTable table;
  StationSm sm;
  FixtureAp ap;

  ap.sends_msg1 = false;                 /* a real open AP keys nothing */
  check(sm.configure_open(kSsid, kOwn), "configure_open succeeds");
  check(sm.security() == StationSm::Security::Open, "...and says it is open");

  discovered(table, 6, /*rsn=*/false);
  const BssEntry* bss = table.select_open(kSsid);
  check(bss != nullptr, "an open BSS is selectable by select_open");
  check(table.select(kSsid) == nullptr,
        "...and NOT by select(), which wants WPA2-PSK");
  if (!bss) return;

  /* A null SNonce, deliberately: the open path must not read it. Passing a
   * real one would leave a mutation that deleted the Open branch of the copy
   * undetectable. */
  check(sm.join(*bss, nullptr, 0), "join starts without an SNonce");
  pump(sm, ap, 0);

  check(ap.saw_auth && ap.saw_assoc, "the AP saw both requests");
  check(sm.state() == StationSm::State::Connected,
        "the station connects with no four-way");
  check(sm.connected(), "connected() is true");
  check(!sm.keyed(), "...and keyed() is FALSE - there is no key");
  check(sm.aid() == ap.aid, "...with the AID the AP allocated");
  check(sm.eapol_tx == 0 && sm.eapol_rx == 0, "no EAPOL in either direction");
  check(sm.supplicant().state() == devourer::sta::Supplicant::State::Idle,
        "the supplicant was never started");

  /* THE WIRE BYTES, not the outcome. An association request that still
   * carried the RSN element, or still claimed Privacy, would associate
   * against this fixture exactly as happily - the fixture does not look - and
   * would be refused by a real open AP. */
  check(!ap.last_assoc.empty(), "the association request was captured");
  if (ap.last_assoc.size() >= 28) {
    const uint16_t cap = devourer::sta::get_le16(ap.last_assoc.data() + 24);
    size_t ie_len = 0;
    check((cap & 0x0010) == 0, "...with the Privacy capability bit CLEAR");
    check((cap & 0x0001) != 0, "...and ESS still set");
    check(devourer::sta::find_ie(ap.last_assoc.data() + 28,
                                 ap.last_assoc.size() - 28,
                                 devourer::sta::kEidRsn, &ie_len) == nullptr,
          "...and no RSN element");
  }
}

/* An open station on a BSS that encrypts. It must refuse before
 * authenticating: associating would succeed and every data frame would then
 * be dropped by one side or the other, with no diagnostic anywhere. */
void test_open_station_refuses_a_protected_bss() {
  BssTable table;
  StationSm sm;

  sm.configure_open(kSsid, kOwn);
  discovered(table, 6, /*rsn=*/true);
  check(table.select_open(kSsid) == nullptr,
        "select_open skips a BSS that advertises Privacy");

  /* select_open refusing is not enough - join() is reachable with a
   * hand-picked entry, which is how a caller with a configured BSSID gets
   * here. Both gates are tested because either alone can be deleted. */
  BssEntry e{};
  std::memcpy(e.info.bssid, kBssid, 6);
  e.info.ssid = kSsid;
  e.info.privacy = true;
  check(!sm.join(e, nullptr, 0), "join() refuses it too");
  check(sm.state() == StationSm::State::Failed, "...and says so");
  check(sm.pending_tx() == 0, "...without airing an authentication request");
}

/* The configuration mismatch: an open station at an AP that tries to key it.
 * The supplicant holds no PMK and cannot verify a MIC, so feeding it an
 * EAPOL-Key frame would either crash or invent a reply. It is counted and
 * dropped, and the link stays up. */
void test_open_station_ignores_eapol() {
  BssTable table;
  StationSm sm;
  FixtureAp ap;

  ap.sends_msg1 = true;                  /* the mismatch, on purpose */
  sm.configure_open(kSsid, kOwn);
  discovered(table, 6, /*rsn=*/false);
  const BssEntry* bss = table.select_open(kSsid);
  if (!bss) { check(false, "open BSS discovered"); return; }
  sm.join(*bss, nullptr, 0);
  pump(sm, ap, 0);

  const uint32_t ignored_before = sm.rx_ignored;
  const std::vector<uint8_t> m1 = ap.eapol_frame(ap.msg1());
  sm.on_rx(m1.data(), m1.size(), 0);

  check(sm.rx_ignored > ignored_before, "the EAPOL-Key frame is counted");
  check(sm.eapol_rx == 0, "...and never reaches the supplicant");
  check(sm.pending_tx() == 0, "...and is not answered");
  check(sm.state() == StationSm::State::Connected, "...and the link stays up");
}

/* A WPA2 join with no SNonce is refused. Copying 32 bytes from whatever the
 * caller passed would let a caller that configures once and joins repeatedly
 * reuse the PREVIOUS association's nonce - the defect configure()'s comment
 * warns about from the other direction. */
void test_wpa2_join_needs_an_snonce() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;

  sm.configure(crypto, kSsid, kPsk, kOwn);
  discovered(table);
  const BssEntry* bss = table.select(kSsid);
  if (!bss) { check(false, "BSS discovered"); return; }
  check(!sm.join(*bss, nullptr, 0), "a WPA2 join without an SNonce is refused");
  check(sm.fail_reason() == StationSm::Failure::NotConfigured,
        "...as NotConfigured");
  check(sm.pending_tx() == 0, "...without airing anything");
}

/* A GROUP-ADDRESSED EAPOL-Key is part of no handshake with this station, and
 * a caller's decrypted path should refuse one too. This layer does not feed
 * it to the supplicant, so a broadcast forged message 1 cannot drive
 * on_msg1. */
void test_broadcast_eapol_is_not_fed_to_the_supplicant() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];
  std::vector<uint8_t> f;
  static const uint8_t bcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  const BssEntry* bss = discovered(table);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);
  while (sm.pop_tx(&f)) {
    const std::vector<uint8_t> r = ap.respond(f);
    if (!r.empty()) sm.on_rx(r.data(), r.size(), 0);
    if (f[0] == devourer::sta::kFcAssocReq) break;
  }

  std::vector<uint8_t> m1 = ap.eapol_frame(ap.msg1());
  std::memcpy(m1.data() + 4, bcast, 6);            /* addr1 = broadcast */
  sm.on_rx(m1.data(), m1.size(), 0);
  check(sm.eapol_rx == 0, "a BROADCAST EAPOL-Key is not fed to the supplicant");
  check(sm.pending_tx() == 0, "...and nothing is answered");

  /* The control: the same frame to us is accepted. */
  std::memcpy(m1.data() + 4, kOwn, 6);
  sm.on_rx(m1.data(), m1.size(), 0);
  check(sm.eapol_rx == 1, "the same frame unicast to us is accepted");
}

/* Beacon supervision follows the BSS's OWN interval: an AP beaconing at
 * 1000 TU (1.024 s) must not be declared lost after one late beacon. */
void test_beacon_loss_follows_the_interval() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  std::vector<uint8_t> b = beacon(kBssid, 6);
  b[32] = (uint8_t)(1000 & 0xff);                  /* beacon interval, TU */
  b[33] = (uint8_t)(1000 >> 8);
  const BssEntry* bss = table.observe(b.data(), b.size(), -40, 6, 0);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);
  pump(sm, ap, 0);
  check(sm.state() == StationSm::State::Connected, "the station connects");
  check(sm.beacon_loss_ms() == 10240,
        "the loss window is ten of the BSS's own intervals");

  sm.tick(StationSm::kBeaconLossMs * 2);
  check(sm.state() == StationSm::State::Connected,
        "two 100-TU windows of silence do not end a 1000-TU link");
  sm.tick(10240);
  check(sm.state() == StationSm::State::Failed &&
            sm.fail_reason() == StationSm::Failure::BeaconLost,
        "...ten of its own intervals do");
}

/* Reconfiguring drops the Supplicant's keys too, not just this object's PMK:
 * configure(WPA2) -> join -> configure_open() must not leave the PTK and GTK
 * resident, nor a station claiming keyed() under the new configuration. */
void test_reconfigure_forgets_the_keys() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32], zero[48] = {0};

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  const BssEntry* bss = discovered(table);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);
  pump(sm, ap, 0);
  check(sm.keyed() && sm.supplicant().ptk_valid(), "keyed");

  sm.configure_open(kSsid, kOwn);
  check(!sm.supplicant().ptk_valid() && !sm.supplicant().gtk_valid(),
        "configure_open() drops the supplicant's keys");
  check(std::memcmp(sm.supplicant().ptk(), zero, 48) == 0,
        "...and wipes the PTK bytes");
  check(!sm.keyed() && sm.state() == StationSm::State::Idle,
        "...and the station no longer claims a keyed link");
  check(sm.aid() == 0, "...nor the old association's AID");
  check(!sm.has_pmk() && std::memcmp(sm.pmk(), zero, 32) == 0,
        "...and configure_open() wipes this object's PMK too");
}

}  // namespace

/* THE ADVERTISEMENT REACHES THE SUPPLICANT. join() keeps the RSN element of
 * the BSS it chose and the four-way holds message 3 to it (802.11-2016
 * 12.7.6.4). Here the beacon advertises CCMP alone while the AP's
 * MIC-protected message 3 offers TKIP as well: someone rewrote the beacon. The
 * station must not come up keyed. Without this cell, a join() that stopped
 * passing the element would leave the supplicant's check silently off. */
void test_rsn_downgrade_is_refused() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];

  std::memset(snonce, 0x7a, 32);
  check(sm.configure(crypto, kSsid, kPsk, kOwn), "configure derives the PMK");
  discovered(table);
  const BssEntry* bss = table.select(kSsid);
  if (!bss) { check(false, "the BSS is selectable"); return; }
  ap.rsn_override = {0x30, 0x18, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04,
                     0x02, 0x00, 0x00, 0x0f, 0xac, 0x02, 0x00, 0x0f,
                     0xac, 0x04, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x02,
                     0x00, 0x00};
  check(sm.join(*bss, snonce, 0), "join starts");
  pump(sm, ap, 0);
  check(!sm.keyed() && !ap.saw_msg4,
        "a message 3 RSN element that differs from the beacon's: not keyed, "
        "no message 4");
  check(sm.supplicant().rsn_mismatches == 1,
        "...and the supplicant counted the mismatch");
}

/* A RECONFIGURE DROPS WHAT WAS QUEUED for the association it lets go: the
 * authentication request join() queued must not air afterwards, built for a
 * configuration the machine no longer has. */
void test_reconfigure_drops_queued_frames() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  uint8_t snonce[32];
  std::memset(snonce, 0x7a, 32);
  check(sm.configure(crypto, kSsid, kPsk, kOwn), "configure");
  discovered(table);
  const BssEntry* bss = table.select(kSsid);
  if (!bss) { check(false, "beacon"); return; }
  check(sm.join(*bss, snonce, 0) && sm.pending_tx() == 1,
        "join queues an authentication request");
  check(sm.configure(crypto, kSsid, kPsk, kOwn), "reconfigure");
  check(sm.pending_tx() == 0,
        "...and the reconfigure dropped it: nothing for the old association airs");
}

/* LEAVE CLEARS WHAT IS QUEUED, then says goodbye. A deauth queued BEHIND
 * whatever is pending would let a caller draining the queue air a stale
 * authentication or association retry first - and after it. */
void test_leave_drops_stale_frames() {
  OpenSslCryptoOps crypto;
  std::vector<uint8_t> f;
  uint8_t snonce[32];
  std::memset(snonce, 0x7a, 32);

  {
    /* During authentication, with two auth requests pending. leave() sends
     * no deauth in this state (nothing has been authenticated to), so the
     * queue ends empty. */
    BssTable table;
    StationSm sm;
    sm.configure(crypto, kSsid, kPsk, kOwn);
    const BssEntry* bss = discovered(table);
    if (!bss) { check(false, "beacon"); return; }
    sm.join(*bss, snonce, 0);
    sm.tick(StationSm::kMgmtTimeoutMs);          /* a retransmission */
    check(sm.pending_tx() == 2, "leave/auth: setup - two auth requests pending");
    sm.leave();
    check(sm.pending_tx() == 0,
          "leave/auth: nothing stale airs after leaving during authentication");
  }
  {
    /* During association, with two association requests pending: the
     * deauth pops first and nothing follows it. */
    BssTable table;
    StationSm sm;
    FixtureAp ap;
    sm.configure(crypto, kSsid, kPsk, kOwn);
    const BssEntry* bss = discovered(table);
    if (!bss) { check(false, "beacon"); return; }
    sm.join(*bss, snonce, 0);
    sm.pop_tx(&f);
    const std::vector<uint8_t> r = ap.respond(f);
    sm.on_rx(r.data(), r.size(), 0);             /* auth success: assoc queued */
    sm.tick(StationSm::kMgmtTimeoutMs);          /* and retransmitted */
    check(sm.state() == StationSm::State::Associating && sm.pending_tx() == 2,
          "leave/assoc: setup - two association requests pending");
    sm.leave();
    check(sm.pop_tx(&f) && !f.empty() && f[0] == devourer::sta::kFcDeauth,
          "leave/assoc: the deauth is the first frame out");
    check(!sm.pop_tx(&f), "leave/assoc: ...and nothing stale follows it");
  }
}

/* A PEER DEAUTH ENDS THE ASSOCIATION, KEYS AND ALL. fail() drops the queue
 * and the supplicant's PTK, GTK and cached replies, which would otherwise
 * stay readable through supplicant(). The PMK and configuration stay: a
 * rejoin needs no reconfigure. */
void test_peer_deauth_drops_the_association_keys() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];
  std::vector<uint8_t> f;

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  discovered(table);
  const BssEntry* bss = table.select(kSsid);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);
  pump(sm, ap, 0);
  if (!sm.keyed()) { check(false, "peer deauth: setup - connected"); return; }
  check(sm.aid() == ap.aid, "peer deauth: setup - the AP's AID is held");

  std::vector<uint8_t> d = devourer::sta::build_deauth(kOwn, kBssid, 15);
  std::memcpy(d.data() + 4, kOwn, 6);
  std::memcpy(d.data() + 10, kBssid, 6);
  std::memcpy(d.data() + 16, kBssid, 6);
  sm.on_rx(d.data(), d.size(), 10);

  check(sm.state() == StationSm::State::Failed &&
            sm.fail_reason() == StationSm::Failure::Deauthenticated &&
            sm.status() == 15,
        "peer deauth: failed, with the peer's reason");
  check(sm.aid() == 0, "peer deauth: aid() no longer reports the old AID");
  check(!sm.supplicant().ptk_valid() && !sm.supplicant().gtk_valid(),
        "peer deauth: the PTK and GTK are no longer valid");
  bool zero = true;
  for (size_t i = 0; i < 48; i++) zero = zero && sm.supplicant().ptk()[i] == 0;
  for (size_t i = 0; i < 32; i++) zero = zero && sm.supplicant().gtk()[i] == 0;
  check(zero, "peer deauth: ...and their bytes are wiped");

  /* During the four-way, with message 2 still queued: it must not air at an
   * AP that has just thrown us off. */
  StationSm sm2;
  FixtureAp ap2;
  sm2.configure(crypto, kSsid, kPsk, kOwn);
  sm2.join(*bss, snonce, 0);
  while (sm2.pop_tx(&f)) {
    const std::vector<uint8_t> r = ap2.respond(f);
    if (!r.empty()) sm2.on_rx(r.data(), r.size(), 0);
    if (f[0] == devourer::sta::kFcAssocReq) break;
  }
  const std::vector<uint8_t> m1 = ap2.eapol_frame(ap2.msg1());
  sm2.on_rx(m1.data(), m1.size(), 0);
  check(sm2.pending_tx() == 1, "peer deauth: setup - message 2 queued");
  sm2.on_rx(d.data(), d.size(), 0);
  check(sm2.pending_tx() == 0,
        "peer deauth: the queued message 2 is dropped with the association");

  /* The PMK survived: the same machine rejoins without reconfiguring. */
  FixtureAp ap3;
  check(sm.join(*bss, snonce, 100), "peer deauth: a rejoin starts");
  pump(sm, ap3, 100);
  check(sm.keyed(), "peer deauth: ...and completes on the kept PMK");
}

/* A REPLY THE QUEUE DROPPED DID NOT GO OUT, so it must not move the
 * four-way's give-up deadline. Refreshing the deadline before queue(), which
 * drops when full, would let an attacker who fills the queue with answered
 * retransmissions slide the deadline forward with each new message 1 while
 * nothing is sent. */
void test_dropped_reply_does_not_move_the_deadline() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];
  std::vector<uint8_t> f;

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  const BssEntry* bss = discovered(table);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);
  while (sm.pop_tx(&f)) {
    const std::vector<uint8_t> r = ap.respond(f);
    if (!r.empty()) sm.on_rx(r.data(), r.size(), 0);
    if (f[0] == devourer::sta::kFcAssocReq) break;
  }
  const std::vector<uint8_t> m1 = ap.eapol_frame(ap.msg1());
  sm.on_rx(m1.data(), m1.size(), 0);
  /* Retransmissions of the same message 1, answered and never drained,
   * until the queue is full. */
  for (int i = 0; i < 64 && sm.pending_tx() < StationSm::tx_capacity(); i++)
    sm.on_rx(m1.data(), m1.size(), 0);
  check(sm.pending_tx() == StationSm::tx_capacity(),
        "dropped reply: setup - the transmit queue is full");
  const uint32_t tx0 = sm.eapol_tx, dropped0 = sm.tx_dropped;

  /* A NEW message 1 (next counter) is real progress for the supplicant -
   * but its reply cannot be queued. */
  const std::vector<uint8_t> m1b = ap.eapol_frame(ap.msg1());
  sm.on_rx(m1b.data(), m1b.size(), 2000);
  check(sm.tx_dropped == dropped0 + 1 && sm.eapol_tx == tx0,
        "dropped reply: the reply was dropped, and not counted as sent");
  sm.tick(StationSm::kHandshakeTimeoutMs);
  check(sm.state() == StationSm::State::Failed &&
            sm.fail_reason() == StationSm::Failure::HandshakeTimeout,
        "dropped reply: the deadline still runs from the last reply that went "
        "out");
}

/* Up to the point where message 3 is in hand and the transmit queue is full
 * of answered message-1 retransmissions, so message 4 will be dropped. */
bool msg4_drop_setup(StationSm& sm, FixtureAp& ap, OpenSslCryptoOps& crypto,
                     BssTable& table, std::vector<uint8_t>* m3) {
  uint8_t snonce[32];
  std::vector<uint8_t> f;

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  const BssEntry* bss = discovered(table);
  if (!bss) return false;
  sm.join(*bss, snonce, 0);
  while (sm.pop_tx(&f)) {
    const std::vector<uint8_t> r = ap.respond(f);
    if (!r.empty()) sm.on_rx(r.data(), r.size(), 0);
    if (f[0] == devourer::sta::kFcAssocReq) break;
  }
  const std::vector<uint8_t> m1 = ap.eapol_frame(ap.msg1());
  sm.on_rx(m1.data(), m1.size(), 0);
  if (!sm.pop_tx(&f)) return false;              /* message 2 */
  *m3 = ap.respond(f);                           /* the AP's message 3 */
  if (m3->empty()) return false;
  for (int i = 0; i < 64 && sm.pending_tx() < StationSm::tx_capacity(); i++)
    sm.on_rx(m1.data(), m1.size(), 0);
  return sm.pending_tx() == StationSm::tx_capacity();
}

/* MESSAGE 4 MUST LEAVE BEFORE THE STATION CALLS ITSELF CONNECTED. The
 * supplicant is Done the moment it accepts message 3, but queue() can still
 * drop message 4 when full. A station Connected and keyed whose message 4
 * never reached the AP carries traffic the AP has not keyed. Three ways on
 * from a dropped message 4. */
void test_dropped_msg4_does_not_connect() {
  for (int variant = 0; variant < 3; variant++) {
    OpenSslCryptoOps crypto;
    BssTable table;
    StationSm sm;
    FixtureAp ap;
    std::vector<uint8_t> m3, f;

    if (!msg4_drop_setup(sm, ap, crypto, table, &m3)) {
      check(false, "msg4 drop: setup - message 3 in hand, queue full");
      return;
    }
    const uint32_t dropped0 = sm.tx_dropped;
    sm.on_rx(m3.data(), m3.size(), 100);
    check(sm.supplicant().state() == devourer::sta::Supplicant::State::Done &&
              sm.tx_dropped == dropped0 + 1,
          "msg4 drop: setup - message 3 accepted, message 4 dropped");
    check(sm.state() == StationSm::State::FourWay && !sm.keyed() &&
              !sm.connected(),
          "msg4 drop: the station stays in FourWay, not Connected");

    if (variant == 0 || variant == 1) {
      while (sm.pop_tx(&f)) {
      }
      /* 0: the AP's retransmission at the SAME counter - answered from the
       *    cached message 4 (Retransmit).
       * 1: hostapd's shape, a retransmission at a GREATER counter - a fresh
       *    Reply under the installed PTK, which must not reinstall. */
      const uint32_t pg = sm.supplicant().ptk_generation();
      const std::vector<uint8_t> again =
          variant == 0 ? m3 : ap.eapol_frame(ap.msg3());
      sm.on_rx(again.data(), again.size(), 200);
      check(sm.state() == StationSm::State::Connected && sm.keyed(),
            variant == 0
                ? "msg4 drop: an equal-counter msg3 retransmission queues "
                  "message 4 and THAT promotes to Connected"
                : "msg4 drop: a greater-counter msg3 retransmission queues "
                  "message 4 and THAT promotes to Connected");
      check(sm.supplicant().ptk_generation() == pg,
            "msg4 drop: ...without reinstalling the PTK");
      check(sm.pop_tx(&f) && ap.respond(f).empty() && ap.saw_msg4,
            "msg4 drop: ...and that message 4 verifies at the AP");
    } else {
      /* 2: the queue never drains. Retransmissions are answered and dropped
       * too, none of it moves the deadline, and the give-up fires. */
      sm.on_rx(m3.data(), m3.size(), 1000);
      sm.on_rx(m3.data(), m3.size(), 2000);
      check(sm.state() == StationSm::State::FourWay,
            "msg4 drop: under a full queue it stays in FourWay");
      sm.tick(StationSm::kHandshakeTimeoutMs);
      check(sm.state() == StationSm::State::Failed &&
                sm.fail_reason() == StationSm::Failure::HandshakeTimeout,
            "msg4 drop: ...until the handshake timeout ends it");
    }
  }
}

/* THE CHANNEL OF A BEACON WITHOUT A DS ELEMENT. Many 5 GHz beacons carry no
 * DS Parameter Set; an entry left at channel 0 would be joined with the
 * 2.4 GHz rate set, which a 5 GHz AP refuses. The channel it was received on
 * is used instead, and with no channel at all the entry is not offered. */
void test_5ghz_beacon_without_ds_uses_the_rx_channel() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];

  std::memset(snonce, 0x7a, 32);
  const std::vector<uint8_t> b = beacon(kBssid, 36, true, /*ds=*/false);
  const BssEntry* seen = table.observe(b.data(), b.size(), -40, 36, 0);
  check(seen && seen->info.channel == 36,
        "5 GHz no-DS: the entry takes the channel it was received on");
  const BssEntry* bss = table.select(kSsid);
  check(bss && bss->info.channel == 36, "5 GHz no-DS: ...and is selectable");
  if (!bss) return;

  sm.configure(crypto, kSsid, kPsk, kOwn);
  sm.join(*bss, snonce, 0);
  pump(sm, ap, 0);
  check(sm.channel() == 36 && sm.keyed(),
        "5 GHz no-DS: the station joins on channel 36");
  const std::vector<uint8_t>& a = ap.last_assoc;
  size_t len = 0;
  const uint8_t* rates =
      a.size() > 28 ? devourer::sta::find_ie(a.data() + 28, a.size() - 28,
                                             devourer::sta::kEidSupportedRates,
                                             &len)
                    : nullptr;
  check(rates && len >= 1 && (rates[0] & 0x7f) == 0x0c,
        "5 GHz no-DS: the association request carries the 5 GHz rate set "
        "(6 Mbps first, no CCK)");
  check(a.size() > 28 &&
            !devourer::sta::find_ie(a.data() + 28, a.size() - 28,
                                    devourer::sta::kEidExtSupportedRates,
                                    nullptr),
        "5 GHz no-DS: ...and no Extended Supported Rates element");

  /* No channel from either source: kept, but never offered - and refused if
   * a caller hands it to join() anyway. */
  BssTable blind;
  blind.observe(b.data(), b.size(), -40, 0, 0);
  check(blind.select(kSsid) == nullptr,
        "5 GHz no-DS: with no RX channel either, the entry is not selectable");
  const BssEntry* hand = blind.find(kBssid);
  StationSm sm2;
  sm2.configure(crypto, kSsid, kPsk, kOwn);
  check(hand && !sm2.join(*hand, snonce, 0) &&
            sm2.fail_reason() == StationSm::Failure::NoChannel &&
            sm2.pending_tx() == 0,
        "5 GHz no-DS: ...and join() refuses it if it is hand-picked");
}

/* A HAND-PICKED IBSS IS REFUSED. BssTable never offers one; join() is also
 * reachable with an entry a caller picked itself, and an infrastructure
 * authenticate/associate against an IBSS has no AP to answer it. */
void test_join_refuses_an_ibss() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  uint8_t snonce[32];

  std::memset(snonce, 0x7a, 32);
  std::vector<uint8_t> b = beacon(kBssid, 6);
  b[34] = 0x12;                                   /* IBSS | Privacy, no ESS */
  b[35] = 0x00;
  table.observe(b.data(), b.size(), -40, 6, 0);
  check(table.select(kSsid) == nullptr, "ibss join: not offered by select()");
  const BssEntry* hand = table.find(kBssid);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  check(hand && !sm.join(*hand, snonce, 0) &&
            sm.fail_reason() == StationSm::Failure::NotInfrastructure &&
            sm.pending_tx() == 0 && sm.auth_tx == 0,
        "ibss join: a hand-picked IBSS is refused before any frame is queued");
}

/* A FAILED RECONFIGURE DOES NOT KEEP THE OLD PMK. pmk_from_psk writes nothing
 * when it refuses a passphrase, so configure() must wipe the previous
 * network's PMK itself or leave it resident. */
void test_failed_reconfigure_wipes_the_old_pmk() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  uint8_t snonce[32];
  const uint8_t zero[32] = {0};

  std::memset(snonce, 0x7a, 32);
  check(sm.configure(crypto, kSsid, kPsk, kOwn) && sm.has_pmk() &&
            std::memcmp(sm.pmk(), zero, 32) != 0,
        "reconfigure: setup - a valid passphrase yields a PMK");
  check(!sm.configure(crypto, kSsid, "short", kOwn),
        "reconfigure: a 5-character passphrase is refused");
  check(!sm.has_pmk() && std::memcmp(sm.pmk(), zero, 32) == 0,
        "reconfigure: ...and the previous PMK is wiped, not kept");
  const BssEntry* bss = discovered(table);
  check(bss && !sm.join(*bss, snonce, 0) &&
            sm.fail_reason() == StationSm::Failure::NoPmk,
        "reconfigure: ...and join() still says NoPmk");
}

/* ONLY EAPOL-KEY IS CLAIMED. EAP, EAPOL-Start and Logoff share ethertype
 * 0x888e, and a key parser would only refuse them. On the decrypted path a
 * non-Key packet goes back to the caller; a Key packet is claimed even when
 * malformed. The cleartext path has
 * no caller to give it back to, so it counts a non-Key packet as ignored
 * and never shows it to the supplicant. */
void test_only_eapol_key_is_claimed() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  discovered(table);
  const BssEntry* bss = table.select(kSsid);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);
  pump(sm, ap, 0);
  if (!sm.keyed()) { check(false, "eapol type: setup - connected"); return; }

  const auto msdu = [](std::initializer_list<uint8_t> eapol) {
    std::vector<uint8_t> m;
    devourer::sta::append_llc_snap(m, 0x888e);
    m.insert(m.end(), eapol.begin(), eapol.end());
    return m;
  };
  const std::vector<uint8_t> start = msdu({0x02, 0x01, 0x00, 0x00});
  const std::vector<uint8_t> eap = msdu({0x02, 0x00, 0x00, 0x05,
                                         0x01, 0x01, 0x00, 0x05, 0x01});
  const std::vector<uint8_t> bad_key = msdu({0x02, 0x03, 0x00, 0x05,
                                             0x02, 0x00, 0x8a, 0x00, 0x10});
  const std::vector<uint8_t> runt = msdu({0x02});
  const uint32_t mal0 = sm.supplicant().malformed;
  std::vector<uint8_t> reply = {0xaa};

  check(!sm.on_decrypted_msdu(start.data(), start.size(), 1, &reply) &&
            reply.size() == 1,
        "eapol type: an EAPOL-Start is handed back to the caller");
  check(!sm.on_decrypted_msdu(eap.data(), eap.size(), 1, &reply),
        "eapol type: so is an EAP packet");
  check(!sm.on_decrypted_msdu(runt.data(), runt.size(), 1, &reply),
        "eapol type: ...and one too short to carry a packet type");
  check(sm.supplicant().malformed == mal0,
        "eapol type: ...none of which reaches the key parser");
  check(sm.on_decrypted_msdu(bad_key.data(), bad_key.size(), 1, &reply) &&
            reply.empty() && sm.supplicant().malformed == mal0 + 1,
        "eapol type: a malformed EAPOL-Key is still claimed, and counted");

  /* The cleartext path, same packets. */
  const auto from_ap = [&](const std::vector<uint8_t>& body) {
    std::vector<uint8_t> f = devourer::sta::data_hdr_from_ds(
        kOwn, kBssid, kBssid, /*protect=*/false, 7);
    f.insert(f.end(), body.begin(), body.end());
    return f;
  };
  const uint32_t ign0 = sm.rx_ignored, rxm0 = sm.rx_malformed;
  const uint32_t mal1 = sm.supplicant().malformed;
  const std::vector<uint8_t> fs = from_ap(start), fe = from_ap(eap),
                             fr = from_ap(runt);
  sm.on_rx(fs.data(), fs.size(), 2);
  sm.on_rx(fe.data(), fe.size(), 2);
  sm.on_rx(fr.data(), fr.size(), 2);
  check(sm.rx_ignored == ign0 + 2 && sm.rx_malformed == rxm0 + 1 &&
            sm.supplicant().malformed == mal1,
        "eapol type: in the clear, Start and EAP are ignored, a runt is "
        "malformed, and the supplicant sees none of them");
  check(sm.keyed(), "eapol type: ...and the link is untouched");
}

/* A BARE HEADER IS NOT A BEACON. A Beacon or Probe Response from the BSSID
 * without its 12-byte fixed body counts as malformed, not as liveness, and a
 * stream of them does not hold off BeaconLost. */
void test_header_only_beacons_do_not_hold_off_loss() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];
  static const uint8_t bcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  discovered(table);
  const BssEntry* bss = table.select(kSsid);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);
  pump(sm, ap, 0);
  if (!sm.keyed()) { check(false, "bare beacon: setup - connected"); return; }

  const std::vector<uint8_t> bare =
      devourer::sta::mgmt_hdr(devourer::sta::kFcBeacon, bcast, kBssid, kBssid);
  const uint32_t seen0 = sm.beacons_rx, mal0 = sm.rx_malformed;
  uint32_t t = 0;
  for (; t <= sm.beacon_loss_ms() + 100; t += 100) {
    sm.on_rx(bare.data(), bare.size(), t);
    sm.tick(t);
  }
  check(sm.beacons_rx == seen0 && sm.rx_malformed > mal0,
        "bare beacon: a 24-byte beacon is counted malformed, not as a beacon");
  check(sm.state() == StationSm::State::Failed &&
            sm.fail_reason() == StationSm::Failure::BeaconLost,
        "bare beacon: ...and a stream of them does not prevent BeaconLost");
}

/* ONLY AN ASSOCIATION RESPONSE ANSWERS AN ASSOCIATION REQUEST. This station
 * never sends a Reassociation Request, so a success-form Reassociation
 * Response must not complete its join. */
void test_reassoc_resp_does_not_complete_a_join() {
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  std::vector<uint8_t> f;

  sm.configure_open(kSsid, kOwn);
  const BssEntry* bss = discovered(table, 6, /*rsn=*/false);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, nullptr, 0);
  sm.pop_tx(&f);                                  /* auth request */
  const std::vector<uint8_t> ok = ap.respond(f);
  sm.on_rx(ok.data(), ok.size(), 0);
  check(sm.state() == StationSm::State::Associating,
        "reassoc: setup - associating");

  std::vector<uint8_t> r = ap.mgmt(devourer::sta::kFcReassocResp);
  devourer::sta::put_le16(r, 0x0001);             /* capability: ESS */
  devourer::sta::put_le16(r, 0);                  /* status: success */
  devourer::sta::put_le16(r, 0xc000 | 5);         /* AID 5 */
  const uint32_t ign0 = sm.rx_ignored;
  sm.on_rx(r.data(), r.size(), 1);
  check(sm.state() == StationSm::State::Associating && sm.aid() == 0 &&
            sm.rx_ignored == ign0 + 1,
        "reassoc: a success-form Reassociation Response is ignored");

  pump(sm, ap, 2);
  check(sm.state() == StationSm::State::Connected && sm.aid() == ap.aid,
        "reassoc: ...and the real Association Response still completes it");
}

/* A HAND-PICKED RSN BSS WITH PRIVACY CLEAR IS REFUSED, as select() refuses
 * it. */
void test_join_refuses_rsn_without_privacy() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  uint8_t snonce[32];

  std::memset(snonce, 0x7a, 32);
  std::vector<uint8_t> b = beacon(kBssid, 6);
  b[34] = 0x01;                                   /* ESS only: no Privacy */
  b[35] = 0x00;
  table.observe(b.data(), b.size(), -40, 6, 0);
  const BssEntry* hand = table.find(kBssid);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  check(hand && hand->info.rsn_ccmp_psk && table.select(kSsid) == nullptr,
        "privacy join: setup - RSN present, not selectable");
  check(hand && !sm.join(*hand, snonce, 0) &&
            sm.fail_reason() == StationSm::Failure::AssocRefused &&
            sm.pending_tx() == 0,
        "privacy join: a hand-picked one is refused before any frame");
}

/* DATA FROM THE AP IS LIVENESS TOO. Under load a promiscuous receiver drops
 * beacons first; a Connected station receiving its downlink must not be
 * declared lost for want of them. Three windows of traffic and no beacons:
 * protected data frames through on_rx, then decrypted MSDUs through
 * on_decrypted_msdu. Then silence, and the loss still fires. */
void test_data_keeps_the_link_alive() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];

  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  discovered(table);
  const BssEntry* bss = table.select(kSsid);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);
  pump(sm, ap, 0);
  if (!sm.keyed()) { check(false, "liveness: setup - connected"); return; }

  const uint32_t w = sm.beacon_loss_ms();
  std::vector<uint8_t> prot = devourer::sta::data_hdr_from_ds(
      kOwn, kBssid, kBssid, /*protect=*/true, 9);
  prot.insert(prot.end(), 40, 0x5a);              /* opaque ciphertext */
  std::vector<uint8_t> msdu;
  devourer::sta::append_llc_snap(msdu, 0x0800);
  msdu.insert(msdu.end(), 20, 0x45);

  uint32_t t = 0;
  for (; t <= 3 * w; t += 100) {
    if (t < (3 * w) / 2) {
      sm.on_rx(prot.data(), prot.size(), t);
    } else {
      std::vector<uint8_t> reply;
      sm.on_decrypted_msdu(msdu.data(), msdu.size(), t, &reply);
    }
    sm.tick(t);
  }
  check(sm.state() == StationSm::State::Connected,
        "liveness: three loss windows of downlink data and no beacons: still "
        "Connected");

  /* The control: nothing at all from the AP, and the loss fires. */
  sm.tick(t + w);
  check(sm.state() == StationSm::State::Failed &&
            sm.fail_reason() == StationSm::Failure::BeaconLost,
        "liveness: ...and with nothing at all from the AP, BeaconLost");
}

/* LEAVE SENDS A DEAUTH ONLY IF THE AP HOLDS STATE FOR US. After an
 * AuthTimeout the AP never accepted an authentication, so there is nothing to
 * deauthenticate; after an AssocTimeout it did, and the deauth clears it.
 * After the AP's own deauth there is nothing left either. */
void test_leave_after_a_timeout() {
  OpenSslCryptoOps crypto;
  uint8_t snonce[32];
  std::vector<uint8_t> f;
  std::memset(snonce, 0x7a, 32);

  {
    BssTable table;
    StationSm sm;
    sm.configure(crypto, kSsid, kPsk, kOwn);
    const BssEntry* bss = discovered(table);
    if (!bss) { check(false, "beacon"); return; }
    sm.join(*bss, snonce, 0);
    for (uint32_t t = 0; t <= 4 * StationSm::kMgmtTimeoutMs;
         t += StationSm::kMgmtTimeoutMs)
      sm.tick(t);
    check(sm.fail_reason() == StationSm::Failure::AuthTimeout,
          "leave after timeout: setup - AuthTimeout");
    sm.leave();
    check(sm.pending_tx() == 0,
          "leave after timeout: no deauth after an AuthTimeout (never "
          "authenticated)");
  }
  {
    BssTable table;
    StationSm sm;
    FixtureAp ap;
    sm.configure(crypto, kSsid, kPsk, kOwn);
    const BssEntry* bss = discovered(table);
    if (!bss) { check(false, "beacon"); return; }
    sm.join(*bss, snonce, 0);
    sm.pop_tx(&f);
    const std::vector<uint8_t> r = ap.respond(f);
    sm.on_rx(r.data(), r.size(), 0);             /* authenticated */
    for (uint32_t t = 0; t <= 4 * StationSm::kMgmtTimeoutMs;
         t += StationSm::kMgmtTimeoutMs)
      sm.tick(t);
    check(sm.fail_reason() == StationSm::Failure::AssocTimeout,
          "leave after timeout: setup - AssocTimeout");
    sm.leave();
    check(sm.pop_tx(&f) && f[0] == devourer::sta::kFcDeauth &&
              !sm.pop_tx(&f),
          "leave after timeout: one deauth after an AssocTimeout (the AP "
          "holds our authentication)");
  }
  {
    BssTable table;
    StationSm sm;
    FixtureAp ap;
    sm.configure(crypto, kSsid, kPsk, kOwn);
    discovered(table);
    const BssEntry* bss = table.select(kSsid);
    if (!bss) { check(false, "beacon"); return; }
    sm.join(*bss, snonce, 0);
    pump(sm, ap, 0);
    std::vector<uint8_t> d = devourer::sta::build_deauth(kOwn, kBssid, 3);
    std::memcpy(d.data() + 4, kOwn, 6);
    std::memcpy(d.data() + 10, kBssid, 6);
    std::memcpy(d.data() + 16, kBssid, 6);
    sm.on_rx(d.data(), d.size(), 1);
    sm.leave();
    check(sm.pending_tx() == 0,
          "leave after timeout: no deauth back after the AP's own deauth");
  }
}

/* Connected to the fixture AP after a real four-way. */
bool connected_to(StationSm& sm, FixtureAp& ap, BssTable& table,
                  OpenSslCryptoOps& crypto) {
  uint8_t snonce[32];
  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  discovered(table);
  const BssEntry* bss = table.select(kSsid);
  if (!bss) return false;
  sm.join(*bss, snonce, 0);
  pump(sm, ap, 0);
  return sm.keyed();
}

/* ONCE KEYED, THE CLEAR CARRIES ONE EAPOL-KEY MESSAGE ONLY: a retransmission
 * of the installed handshake's message 3 (the AP sends it in the clear when
 * our message 4 is lost, because it installs its PTK only on receiving it).
 * A group message 1 in the clear is never taken, and a message 3 with a new
 * ANonce - a rekey, which the AP protects - is ignored. */
void test_cleartext_eapol_after_keying() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  if (!connected_to(sm, ap, table, crypto)) {
    check(false, "cleartext eapol: setup - connected");
    return;
  }
  const uint32_t gg = sm.supplicant().gtk_generation();
  const uint32_t pg = sm.supplicant().ptk_generation();
  const uint32_t ign0 = sm.rx_ignored, rx0 = sm.eapol_rx;

  uint8_t gtk2[16];
  std::memset(gtk2, 0x62, 16);
  const std::vector<uint8_t> g1 = ap.eapol_frame(ap.group1(gtk2, 2));
  sm.on_rx(g1.data(), g1.size(), 1);
  check(sm.pending_tx() == 0 && sm.eapol_rx == rx0 &&
            sm.rx_ignored == ign0 + 1 &&
            sm.supplicant().gtk_generation() == gg &&
            std::memcmp(sm.supplicant().gtk(), ap.gtk, 16) == 0,
        "cleartext eapol: a group message 1 in the clear is not answered, and "
        "the GTK is unchanged");

  uint8_t anonce[32];
  std::memcpy(anonce, ap.anonce, 32);
  std::memset(ap.anonce, 0x4d, 32);               /* a rekey's new ANonce */
  const std::vector<uint8_t> m3new = ap.eapol_frame(ap.msg3());
  sm.on_rx(m3new.data(), m3new.size(), 2);
  check(sm.pending_tx() == 0 && sm.eapol_rx == rx0 &&
            sm.rx_ignored == ign0 + 2 &&
            sm.supplicant().ptk_generation() == pg,
        "cleartext eapol: a message 3 with a new ANonce after Connected is "
        "ignored");

  std::memcpy(ap.anonce, anonce, 32);             /* the installed handshake */
  const std::vector<uint8_t> m3again = ap.eapol_frame(ap.msg3());
  sm.on_rx(m3again.data(), m3again.size(), 3);
  std::vector<uint8_t> f;
  ap.saw_msg4 = false;
  check(sm.pop_tx(&f) && ap.respond(f).empty() && ap.saw_msg4 &&
            sm.supplicant().ptk_generation() == pg && sm.keyed(),
        "cleartext eapol: ...while a retransmission of the installed message "
        "3 is answered, with no reinstall");
}

/* A JOIN ON A LIVE ASSOCIATION DEAUTHENTICATES THE OLD AP FIRST, and a join
 * is held to the configured SSID, as select() is. */
void test_join_on_a_live_association() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];
  std::memset(snonce, 0x7a, 32);
  if (!connected_to(sm, ap, table, crypto)) {
    check(false, "live join: setup - connected");
    return;
  }

  const uint8_t kBssid2[6] = {0x02, 0x42, 0x75, 0x05, 0xd6, 0x01};
  const std::vector<uint8_t> b2 = beacon(kBssid2, 6);
  table.observe(b2.data(), b2.size(), -40, 6, 0);
  const BssEntry* ap2 = table.find(kBssid2);
  if (!ap2) { check(false, "live join: setup - second AP"); return; }
  check(sm.join(*ap2, snonce, 10), "live join: the join starts");
  std::vector<uint8_t> f;
  check(sm.pop_tx(&f) && f[0] == devourer::sta::kFcDeauth &&
            std::memcmp(f.data() + 4, kBssid, 6) == 0 &&
            std::memcmp(f.data() + 16, kBssid, 6) == 0,
        "live join: the first frame out is a deauth to the OLD AP");
  check(sm.pop_tx(&f) && f[0] == devourer::sta::kFcAuth &&
            std::memcmp(f.data() + 4, kBssid2, 6) == 0 && !sm.pop_tx(&f),
        "live join: ...then the authentication request to the new one");

  StationSm fresh;
  fresh.configure(crypto, kSsid, kPsk, kOwn);
  BssEntry other = *ap2;
  other.info.ssid = "not-this-network";
  check(!fresh.join(other, snonce, 0) &&
            fresh.fail_reason() == StationSm::Failure::SsidMismatch &&
            fresh.pending_tx() == 0,
        "live join: an entry with another SSID is refused before any frame");
}

/* THE BEACON INTERVAL THAT WIDENS THE LOSS WINDOW IS CAPPED. It comes off
 * the air; 65535 TU would stretch the window to about 11 minutes. */
void test_beacon_interval_is_capped() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  uint8_t snonce[32];
  std::memset(snonce, 0x7a, 32);
  std::vector<uint8_t> b = beacon(kBssid, 6);
  b[32] = 0xff;                                   /* beacon interval 65535 TU */
  b[33] = 0xff;
  const BssEntry* bss = table.observe(b.data(), b.size(), -40, 6, 0);
  if (!bss) { check(false, "beacon"); return; }
  sm.configure(crypto, kSsid, kPsk, kOwn);
  sm.join(*bss, snonce, 0);
  check(sm.beacon_loss_ms() ==
            StationSm::kMaxBeaconIntervalTu * 1024u * 10u / 1000u,
        "interval cap: 65535 TU gives the capped 10240 ms window, not minutes");
}

/* pop_tx(nullptr) REFUSES rather than dropping a frame and saying it
 * delivered one. */
void test_pop_tx_refuses_null() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  uint8_t snonce[32];
  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  const BssEntry* bss = discovered(table);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);
  check(!sm.pop_tx(nullptr) && sm.pending_tx() == 1,
        "pop_tx(nullptr): refused, and the frame stays queued");
}

/* A TRUNCATED AUTH OR ASSOC RESPONSE IS COUNTED MALFORMED, like a short
 * deauth, and changes nothing. */
void test_truncated_auth_and_assoc_are_malformed() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  uint8_t snonce[32];
  std::vector<uint8_t> f;
  std::memset(snonce, 0x7a, 32);
  sm.configure(crypto, kSsid, kPsk, kOwn);
  const BssEntry* bss = discovered(table);
  if (!bss) { check(false, "beacon"); return; }
  sm.join(*bss, snonce, 0);

  std::vector<uint8_t> short_auth = ap.mgmt(devourer::sta::kFcAuth);
  devourer::sta::put_le16(short_auth, 0);         /* 2 of the 6 body bytes */
  uint32_t m0 = sm.rx_malformed;
  sm.on_rx(short_auth.data(), short_auth.size(), 0);
  check(sm.rx_malformed == m0 + 1 &&
            sm.state() == StationSm::State::Authenticating,
        "truncated: a short authentication response is malformed");

  sm.pop_tx(&f);
  const std::vector<uint8_t> ok = ap.respond(f);
  sm.on_rx(ok.data(), ok.size(), 0);             /* now associating */
  std::vector<uint8_t> short_assoc = ap.mgmt(devourer::sta::kFcAssocResp);
  devourer::sta::put_le16(short_assoc, 0x0011);   /* 2 of the 6 body bytes */
  m0 = sm.rx_malformed;
  sm.on_rx(short_assoc.data(), short_assoc.size(), 0);
  check(sm.rx_malformed == m0 + 1 &&
            sm.state() == StationSm::State::Associating,
        "truncated: ...and so is a short association response");
}

/* A QoS NULL IS WELL FORMED. It has no body by definition, so it is ignored
 * rather than malformed - and it is still the AP talking, so it keeps the
 * link alive. */
void test_qos_null_is_ignored_and_alive() {
  OpenSslCryptoOps crypto;
  BssTable table;
  StationSm sm;
  FixtureAp ap;
  if (!connected_to(sm, ap, table, crypto)) {
    check(false, "qos null: setup - connected");
    return;
  }
  std::vector<uint8_t> qn = devourer::sta::data_hdr_from_ds(
      kOwn, kBssid, kBssid, /*protect=*/false, 5);
  qn[0] = 0xc8;                                   /* QoS Null */
  qn.insert(qn.begin() + 24, {0x00, 0x00});       /* QoS Control */
  const uint32_t ign0 = sm.rx_ignored, mal0 = sm.rx_malformed;
  sm.on_rx(qn.data(), qn.size(), 1);
  check(sm.rx_ignored == ign0 + 1 && sm.rx_malformed == mal0,
        "qos null: a 26-byte QoS Null is ignored, not malformed");

  const uint32_t w = sm.beacon_loss_ms();
  for (uint32_t t = 100; t <= 2 * w; t += 100) {
    sm.on_rx(qn.data(), qn.size(), t);
    sm.tick(t);
  }
  check(sm.state() == StationSm::State::Connected,
        "qos null: ...and a stream of them keeps the link alive");
}

int main() {
  test_cleartext_eapol_after_keying();
  test_join_on_a_live_association();
  test_beacon_interval_is_capped();
  test_pop_tx_refuses_null();
  test_truncated_auth_and_assoc_are_malformed();
  test_qos_null_is_ignored_and_alive();
  test_data_keeps_the_link_alive();
  test_leave_after_a_timeout();
  test_header_only_beacons_do_not_hold_off_loss();
  test_reassoc_resp_does_not_complete_a_join();
  test_join_refuses_rsn_without_privacy();
  test_join_refuses_an_ibss();
  test_failed_reconfigure_wipes_the_old_pmk();
  test_only_eapol_key_is_claimed();
  test_dropped_msg4_does_not_connect();
  test_5ghz_beacon_without_ds_uses_the_rx_channel();
  test_leave_drops_stale_frames();
  test_peer_deauth_drops_the_association_keys();
  test_dropped_reply_does_not_move_the_deadline();
  test_full_association();
  test_auth_timeout();
  test_auth_refused();
  test_assoc_refused();
  test_assoc_refused_with_a_plausible_aid();
  test_assoc_success_with_zero_aid();
  test_deauth_during_handshake();
  test_short_deauth_is_malformed();
  test_group_rekey_through_the_decrypted_path();
  test_frames_from_elsewhere_are_ignored();
  test_protected_eapol_ignored();
  test_protected_data_is_counted_apart();
  test_fragmented_and_amsdu_eapol_are_refused();
  test_handshake_timeout();
  test_retransmission_does_not_extend_the_deadline();
  test_join_clears_the_transmit_queue();
  test_transmit_queue_is_bounded();
  test_beacon_loss();
  test_leave();
  test_rx_counters();
  test_join_refuses_an_unusable_bss();
  test_no_pmk();
  test_open_association();
  test_open_station_refuses_a_protected_bss();
  test_open_station_ignores_eapol();
  test_wpa2_join_needs_an_snonce();
  test_broadcast_eapol_is_not_fed_to_the_supplicant();
  test_beacon_loss_follows_the_interval();
  test_reconfigure_forgets_the_keys();
  test_rsn_downgrade_is_refused();
  test_reconfigure_drops_queued_frames();

  if (g_fail) {
    std::printf("station_sm_selftest: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("station_sm_selftest: OK\n");
  return 0;
}
