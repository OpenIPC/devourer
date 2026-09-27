/* Headless guard for src/sta/Dot11.h — the role-neutral management-frame
 * module.
 *
 * The properties worth testing here are the ones whose failure is silent on
 * air. A malformed IE walk reads off the end of a hostile frame; a builder
 * that emits the right bytes in the wrong order produces a frame the peer
 * ignores without comment; a station that never assigns a sequence number
 * feeds the AP's duplicate detector and watches its own traffic vanish.
 *
 * Every case below is either a round-trip through the module's own parser
 * (build → parse → compare) or an assertion about exact wire bytes. The
 * round-trips would pass on a self-consistently wrong implementation, so the
 * byte-exact assertions carry the weight and the round-trips catch the rest.
 */
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "sta/Dot11.h"

namespace {

using namespace devourer::sta;

int g_fail = 0;

void check(bool ok, const char* what) {
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    g_fail++;
  }
}

const uint8_t kBssid[6] = {0x02, 0x42, 0x75, 0x05, 0xd6, 0x00};
const uint8_t kOwn[6] = {0x40, 0xa5, 0xef, 0x5a, 0x32, 0xf8};

/* The header's address order IS the difference between the two roles, so it
 * gets an exact-bytes test rather than a round-trip. */
void test_mgmt_hdr() {
  std::vector<uint8_t> m = mgmt_hdr(kFcAuth, kBssid, kOwn, kBssid);

  check(m.size() == 24, "management header is 24 bytes");
  check(m[0] == kFcAuth, "fc0 carries type+subtype");
  check(m[1] == 0, "fc1 starts clear");
  check(std::memcmp(m.data() + 4, kBssid, 6) == 0, "addr1 is the DA");
  check(std::memcmp(m.data() + 10, kOwn, 6) == 0, "addr2 is the SA");
  check(std::memcmp(m.data() + 16, kBssid, 6) == 0, "addr3 is the BSSID");
  check(m[22] == 0 && m[23] == 0, "sequence control starts zero");
}

/* Sequence assignment: the low 4 bits are the fragment number and must stay
 * clear, and the counter must wrap at 12 bits rather than overflow into them. */
void test_seq() {
  std::vector<uint8_t> m = mgmt_hdr(kFcAuth, kBssid, kOwn, kBssid);

  assign_seq(m, 1);
  check(m[22] == 0x10 && m[23] == 0x00, "seq 1 lands above the fragment nibble");
  assign_seq(m, 0x0fff);
  check(m[22] == 0xf0 && m[23] == 0xff, "the maximum sequence number packs");
  assign_seq(m, 0x1001);
  check(m[22] == 0x10 && m[23] == 0x00, "a sequence number wraps at 12 bits");

  SeqCounter c;
  check(c.next() == 0 && c.next() == 1, "the counter starts at 0 and advances");
  for (int i = 0; i < 4093; i++) c.next();
  check(c.next() == 4095, "the counter reaches 4095");
  check(c.next() == 0, "the counter wraps to 0, not to 4096");

  /* A short buffer must be refused, not written past. */
  /* THE SHORT-BUFFER GUARD IS NOT ASSERTABLE FROM HERE, and saying so is
   * better than an assertion that cannot fail. assign_seq writes only at
   * offsets 22 and 23, which for a 10-byte buffer is precisely the overflow
   * the guard prevents - so `tiny[0] == 0` is true whether the guard exists
   * or not. Deleting `if (frame.size() < 24) return;` is caught by the
   * sanitizer job (ctest `build-sanitizers`), not by this file.
   *
   * What IS assertable is the positive: the minimum legal buffer gets its
   * sequence number and nothing else is touched. */
  std::vector<uint8_t> tiny(10, 0);
  assign_seq(tiny, 7);
  check(tiny.size() == 10, "assign_seq on a short buffer changes nothing "
                           "observable - the guard is proved by ASan, not here");

  std::vector<uint8_t> exact(24, 0);
  assign_seq(exact, 7);
  check(exact[22] == 0x70 && exact[23] == 0x00,
        "a 24-byte buffer - the minimum legal header - IS numbered");
  for (size_t i = 0; i < 22; i++)
    if (exact[i] != 0) {
      check(false, "assign_seq touches nothing but the sequence control");
      break;
    }
}

/* The IE walker is the one function here that reads attacker-controlled
 * lengths. A truncated element must end the walk, never read past the end. */
void test_ie_walk() {
  std::vector<uint8_t> m;
  size_t len = 0;

  append_ssid(m, "devourerAP");
  append_supported_rates(m);
  append_ds_params(m, 149);

  const uint8_t* p = find_ie(m.data(), m.size(), kEidSsid, &len);
  check(p && len == 10 && std::memcmp(p, "devourerAP", 10) == 0,
        "SSID element round-trips");
  p = find_ie(m.data(), m.size(), kEidDsParams, &len);
  check(p && len == 1 && p[0] == 149, "DS Parameter Set carries the channel");
  check(find_ie(m.data(), m.size(), kEidVhtCaps, &len) == nullptr,
        "an absent element reports absent");

  /* An element claiming more bytes than the buffer holds. */
  const uint8_t truncated[] = {kEidSsid, 200, 'a', 'b'};
  check(find_ie(truncated, sizeof truncated, kEidSsid, &len) == nullptr,
        "an element longer than the buffer is refused, not read");
  /* And one whose header itself is cut off. */
  const uint8_t stub[] = {kEidSsid};
  check(find_ie(stub, sizeof stub, kEidSsid, &len) == nullptr,
        "a one-byte element header is refused");
  /* A zero-length element must not stall the walk. */
  const uint8_t empty_then_ds[] = {kEidSsid, 0, kEidDsParams, 1, 36};
  p = find_ie(empty_then_ds, sizeof empty_then_ds, kEidDsParams, &len);
  check(p && len == 1 && p[0] == 36,
        "a zero-length element does not stall the walk");
}

/* The RSN element's exact bytes matter in both directions: an AP advertises
 * them and a station echoes them back. */
/* Golden bytes. The rate sets and the RSN element are byte for byte what the
 * two on-air validated AP harnesses (tests/ap_responder.cpp,
 * tests/ap_wpa2.cpp) emit inline, so a builder change that would alter them
 * fails here rather than on air. */
void test_golden_bytes() {
  static const uint8_t k24[] = {0x01, 0x08, 0x82, 0x84, 0x8b,
                                0x96, 0x24, 0x30, 0x48, 0x6c};
  static const uint8_t k5[] = {0x01, 0x08, 0x8c, 0x12, 0x98,
                               0x24, 0xb0, 0x48, 0x60, 0x6c};
  static const uint8_t kSsidIe[] = {0x00, 0x0a, 'd', 'e', 'v', 'o',
                                    'u', 'r', 'e', 'r', 'A', 'P'};
  static const uint8_t kDs[] = {0x03, 0x01, 0x24};
  std::vector<uint8_t> m;

  append_supported_rates(m);
  check(m.size() == sizeof k24 && std::memcmp(m.data(), k24, sizeof k24) == 0,
        "2.4 GHz Supported Rates bytes are unchanged");
  m.clear();
  append_supported_rates_5g(m);
  check(m.size() == sizeof k5 && std::memcmp(m.data(), k5, sizeof k5) == 0,
        "5 GHz Supported Rates bytes are unchanged");
  m.clear();
  check(append_ssid(m, "devourerAP"), "the SSID element builds");
  check(m.size() == sizeof kSsidIe &&
            std::memcmp(m.data(), kSsidIe, sizeof kSsidIe) == 0,
        "SSID element bytes are unchanged");
  m.clear();
  append_ds_params(m, 36);
  check(m.size() == sizeof kDs && std::memcmp(m.data(), kDs, sizeof kDs) == 0,
        "DS Parameter Set bytes are unchanged");

  /* Over-length bodies must be refused, not truncated into a corrupt frame. */
  m.clear();
  check(!append_ssid(m, std::string(33, 'x')), "a 33-byte SSID is refused");
  check(m.empty(), "a refused element emits nothing at all");
  m.clear();
  std::vector<uint8_t> big(256, 0);
  check(!append_ie(m, kEidSsid, big.data(), big.size()),
        "an IE body over 255 bytes is refused");
  check(m.empty(), "a refused IE emits nothing at all");
}

void test_rsn() {
  std::vector<uint8_t> m;
  size_t len = 0;

  append_rsn_ccmp_psk(m);
  const uint8_t* p = find_ie(m.data(), m.size(), kEidRsn, &len);
  check(p && len == 20, "RSN element is 20 bytes");
  if (!p) return;
  check(p[0] == 0x01 && p[1] == 0x00, "RSN version 1");
  check(p[2] == 0x00 && p[3] == 0x0f && p[4] == 0xac && p[5] == 0x04,
        "group cipher is CCMP");
  check(p[10] == 0xac && p[11] == 0x04, "pairwise cipher is CCMP");
  check(p[16] == 0xac && p[17] == 0x02, "AKM is PSK");
  /* The COUNT fields. A swapped count makes parse_beacon reject real APs, and
   * a build/parse round-trip would not notice because both sides share it. */
  check(p[6] == 0x01 && p[7] == 0x00, "exactly one pairwise cipher suite");
  check(p[12] == 0x01 && p[13] == 0x00, "exactly one AKM suite");
  check(p[8] == 0x00 && p[9] == 0x0f, "pairwise suite OUI 00-0F-AC");
  check(p[14] == 0x00 && p[15] == 0x0f, "AKM suite OUI 00-0F-AC");
  check(p[18] == 0x00 && p[19] == 0x00, "RSN capabilities are zero (no MFP)");
}

/* Build a beacon the way an AP does, parse it the way a station will. */
void test_beacon_roundtrip() {
  // SA deliberately DIFFERENT from the BSSID. With both set to kBssid the
  // "BSSID comes from addr3" assertion below would pass even if the parser
  // read addr2, and pin nothing.
  static const uint8_t kOtherSa[6] = {0x06, 0x06, 0x06, 0x06, 0x06, 0x06};
  std::vector<uint8_t> m = mgmt_hdr(kFcBeacon, (const uint8_t*)"\xff\xff\xff\xff\xff\xff",
                                    kOtherSa, kBssid);
  for (int i = 0; i < 8; i++) m.push_back(0); /* timestamp */
  put_le16(m, 100);                            /* beacon interval */
  put_le16(m, 0x0011);                         /* ESS | Privacy */
  append_ssid(m, "devourerAP");
  append_supported_rates_5g(m);
  append_ds_params(m, 36);
  append_rsn_ccmp_psk(m);

  BssInfo b;
  check(parse_beacon(m.data(), m.size(), &b), "a beacon parses");
  check(std::memcmp(b.bssid, kBssid, 6) == 0, "BSSID comes from addr3");
  check(std::memcmp(b.bssid, kOtherSa, 6) != 0,
        "BSSID is NOT addr2 (the test can tell them apart)");
  check(b.ssid == "devourerAP", "SSID round-trips");
  check(b.beacon_interval_tu == 100, "beacon interval round-trips");
  check(b.capability == 0x0011, "capability round-trips");
  check(b.privacy, "the Privacy bit is decoded");
  check(b.channel == 36, "the channel comes from the DS Parameter Set");
  check(b.has_rsn && b.rsn_ccmp_psk, "our own RSN element is recognised");

  /* An RSN element we do not implement must be reported as such rather than
   * associated with and failed later. TKIP pairwise (…ac 02 in the cipher
   * position) is the classic case. */
  std::vector<uint8_t> tkip = m;
  size_t rl = 0;
  uint8_t* r = const_cast<uint8_t*>(find_ie(tkip.data() + 36, tkip.size() - 36,
                                            kEidRsn, &rl));
  check(r != nullptr, "setup: found the RSN element to corrupt");
  if (r) {
    r[11] = 0x02; /* pairwise CCMP -> TKIP */
    BssInfo b2;
    check(parse_beacon(tkip.data(), tkip.size(), &b2), "setup: still parses");
    check(b2.has_rsn && !b2.rsn_ccmp_psk,
          "an unsupported RSN suite is rejected, not silently accepted");
  }

  /* Too short to hold the fixed body. */
  BssInfo b3;
  check(!parse_beacon(m.data(), 30, &b3), "a truncated beacon is refused");

  /* A REUSED BssInfo must not carry the previous BSS's fields. A scan loop
   * does exactly this, and a frame missing the SSID/DS/RSN elements would
   * otherwise report the last BSS's values as this one's. */
  std::vector<uint8_t> bare = mgmt_hdr(kFcBeacon,
                                       (const uint8_t*)"\xff\xff\xff\xff\xff\xff",
                                       kOtherSa, kOwn);
  for (int i = 0; i < 8; i++) bare.push_back(0);
  put_le16(bare, 200);
  put_le16(bare, 0x0001); /* ESS, no Privacy, and no IEs at all */
  check(parse_beacon(bare.data(), bare.size(), &b), "a bare beacon parses");
  check(b.ssid.empty(), "a reused BssInfo does not keep the old SSID");
  check(b.channel == 0, "a reused BssInfo does not keep the old channel");
  check(!b.has_rsn, "a reused BssInfo does not keep the old RSN flag");
  check(!b.rsn_ccmp_psk, "a reused BssInfo does not keep the old RSN verdict");
  check(!b.privacy, "a reused BssInfo does not keep the old Privacy bit");
  check(b.beacon_interval_tu == 200, "the new beacon's own fields are set");

  /* An RSN element that stops before the Capabilities field cannot be judged
   * safe to join, and an MFP-required BSS must be surfaced rather than
   * associated with and failed later. */
  std::vector<uint8_t> mfp = m;
  size_t ml = 0;
  uint8_t* mr = const_cast<uint8_t*>(find_ie(mfp.data() + 36, mfp.size() - 36,
                                             kEidRsn, &ml));
  check(mr && ml == 20, "setup: RSN element found");
  if (mr && ml == 20) {
    mr[18] = 0x40; /* RSN Capabilities: MFPR */
    BssInfo b4;
    check(parse_beacon(mfp.data(), mfp.size(), &b4), "setup: parses");
    check(b4.rsn_mfp_required,
          "a BSS that REQUIRES management-frame protection is flagged");
  }
}

void test_station_builders() {
  std::vector<uint8_t> pr = build_probe_req(kOwn, "devourerAP", 36, true);
  size_t len = 0;

  check(pr[0] == kFcProbeReq, "probe request subtype");
  check(pr[4] == 0xff && pr[9] == 0xff, "probe request addr1 is broadcast");
  check(std::memcmp(pr.data() + 10, kOwn, 6) == 0, "probe request SA is ours");
  const uint8_t* p = find_ie(pr.data() + 24, pr.size() - 24, kEidSsid, &len);
  check(p && len == 10, "a directed probe carries the SSID");
  p = find_ie(pr.data() + 24, pr.size() - 24, kEidSupportedRates, &len);
  check(p && len == 8, "a 5 GHz probe advertises eight rates");
  /* NO CCK on 5 GHz. The rates are OFDM 6..54. */
  check(p && p[0] == 0x0c && p[7] == 0x6c,
        "a 5 GHz probe advertises OFDM 6..54, not CCK");
  /* AND NONE OF THEM IS MARKED BASIC. A station's Supported Rates element
   * says what the STATION can do; the basic set is the AP's statement about
   * its BSS, and mac80211 sets no basic bit in a station's requests either.
   * The AP's 5 GHz builder marks four of them basic, which is why a station
   * does not reuse it. */
  for (size_t i = 0; p && i < len; i++)
    if (p[i] & 0x80) {
      check(false, "a station's own rate set marks nothing BASIC");
      break;
    }
  check(find_ie(pr.data() + 24, pr.size() - 24, kEidExtSupportedRates, &len) ==
            nullptr,
        "a 5 GHz probe needs no Extended Supported Rates - eight rates fit");

  /* 2.4 GHz: the mandatory OFDM rates 6, 12 and 24 must be advertised, or an
   * AP whose basic set includes them refuses the association with status 18.
   * The AP's builder omits 6, 9, 12 and 48 for reasons of its own, which is
   * why a station has its own. 24 (0x30) lands in the extended element. */
  {
    std::vector<uint8_t> g = build_probe_req(kOwn, "devourerAP", 6, false);
    size_t sl = 0, el = 0;
    const uint8_t* sr =
        find_ie(g.data() + 24, g.size() - 24, kEidSupportedRates, &sl);
    const uint8_t* er =
        find_ie(g.data() + 24, g.size() - 24, kEidExtSupportedRates, &el);
    bool has6 = false, has12 = false, has24 = false;

    check(sr && sl == 8, "a 2.4 GHz probe carries eight supported rates");
    check(er && el == 4, "...and four more in Extended Supported Rates");
    for (size_t i = 0; sr && i < sl; i++) {
      if ((sr[i] & 0x7f) == 0x0c) has6 = true;
      if ((sr[i] & 0x7f) == 0x18) has12 = true;
      if ((sr[i] & 0x7f) == 0x30) has24 = true;
    }
    for (size_t i = 0; er && i < el; i++) {
      if ((er[i] & 0x7f) == 0x0c) has6 = true;
      if ((er[i] & 0x7f) == 0x18) has12 = true;
      if ((er[i] & 0x7f) == 0x30) has24 = true;
    }
    check(has6 && has12 && has24,
          "the mandatory OFDM rates 6, 12 and 24 are advertised");
    check(sr && (sr[0] & 0x7f) == 0x02,
          "...and 1 Mbps CCK is still there for a 2.4 GHz BSS");
    for (size_t i = 0; sr && i < sl; i++)
      if (sr[i] & 0x80) {
        check(false, "a 2.4 GHz station marks none of its rates BASIC");
        break;
      }
  }

  /* THE ASSOCIATION REQUEST, not just the probe. This is the frame an AP
   * refuses with status 18 when a mandatory rate is missing, and a check on
   * the probe alone would not notice it carrying the AP's set instead. */
  {
    std::vector<uint8_t> a =
        build_assoc_req(kOwn, kBssid, "devourerAP", true, false);
    size_t sl = 0, el = 0;
    const uint8_t* sr =
        find_ie(a.data() + 28, a.size() - 28, kEidSupportedRates, &sl);
    const uint8_t* er =
        find_ie(a.data() + 28, a.size() - 28, kEidExtSupportedRates, &el);
    bool has6 = false, has12 = false;

    check(sr && sl == 8 && er && el == 4,
          "a 2.4 GHz association request carries twelve rates in two elements");
    for (size_t i = 0; sr && i < sl; i++) {
      if ((sr[i] & 0x7f) == 0x0c) has6 = true;
      if ((sr[i] & 0x7f) == 0x18) has12 = true;
      if (sr[i] & 0x80) { check(false, "assoc request marks nothing BASIC"); break; }
    }
    check(has6 && has12,
          "the association request advertises the mandatory 6 and 12 Mbps");
  }

  std::vector<uint8_t> wildcard = build_probe_req(kOwn, "", 0, false);
  p = find_ie(wildcard.data() + 24, wildcard.size() - 24, kEidSsid, &len);
  check(p && len == 0, "a wildcard probe carries an EMPTY SSID element");
  check(find_ie(wildcard.data() + 24, wildcard.size() - 24, kEidDsParams,
                &len) == nullptr,
        "channel 0 omits the DS Parameter Set");

  AuthFields af;
  std::vector<uint8_t> au = build_auth_req(kOwn, kBssid);
  check(parse_auth(au.data(), au.size(), &af), "auth request parses");
  check(af.algorithm == 0, "open-system authentication");
  check(af.seq == 1, "the station's auth is sequence 1");
  check(af.status == 0, "auth request status is 0");
  check(std::memcmp(au.data() + 4, kBssid, 6) == 0, "auth is addressed to the AP");

  /* The capability/RSN agreement an AP checks. */
  std::vector<uint8_t> ar = build_assoc_req(kOwn, kBssid, "devourerAP", true, true);
  check(ar[0] == kFcAssocReq, "assoc request subtype");
  check(get_le16(ar.data() + 24) == 0x0011,
        "an RSN assoc request sets ESS and Privacy together");
  check(find_ie(ar.data() + 28, ar.size() - 28, kEidRsn, &len) != nullptr,
        "an RSN assoc request carries the RSN element");

  std::vector<uint8_t> open = build_assoc_req(kOwn, kBssid, "devourerAP", false, false);
  check(get_le16(open.data() + 24) == 0x0001,
        "an open assoc request claims ESS without Privacy");
  check(find_ie(open.data() + 28, open.size() - 28, kEidRsn, &len) == nullptr,
        "an open assoc request carries no RSN element");

  std::vector<uint8_t> dr = build_deauth(kOwn, kBssid, 3);
  uint16_t reason = 0;
  check(dr[0] == kFcDeauth, "deauth subtype");
  check(parse_reason(dr.data(), dr.size(), &reason) && reason == 3,
        "deauth carries its reason code");
}

void test_assoc_resp_parse() {
  std::vector<uint8_t> m = mgmt_hdr(kFcAssocResp, kOwn, kBssid, kBssid);
  put_le16(m, 0x0011);
  put_le16(m, 0);
  put_le16(m, 0xc001); /* AID 1 with both top bits set, as on the wire */
  append_supported_rates_5g(m);

  AssocRespFields f;
  check(parse_assoc_resp(m.data(), m.size(), &f), "assoc response parses");
  check(f.status == 0, "status 0 is success");
  check(f.aid == 1, "the AID has its two top bits masked off");

  AssocRespFields g;
  check(!parse_assoc_resp(m.data(), 24, &g),
        "an assoc response with no body is refused");
}

/* Data-frame direction and header length. The QoS +2 is the offset error that
 * silently drops every QoS frame. */
void test_data_frames() {
  const uint8_t dest[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
  std::vector<uint8_t> up = data_hdr_to_ds(kBssid, kOwn, dest, true);

  check(up[1] == (kFcToDs | kFcProtected), "uplink sets to-DS and Protected");
  check(std::memcmp(up.data() + 4, kBssid, 6) == 0, "uplink addr1 is the BSSID");
  check(std::memcmp(up.data() + 10, kOwn, 6) == 0, "uplink addr2 is us");
  check(std::memcmp(up.data() + 16, dest, 6) == 0, "uplink addr3 is the dest");

  std::vector<uint8_t> down = data_hdr_from_ds(kOwn, kBssid, dest, false);
  check(down[1] == kFcFromDs, "downlink sets from-DS only");
  check(std::memcmp(down.data() + 4, kOwn, 6) == 0, "downlink addr1 is the STA");
  check(std::memcmp(down.data() + 10, kBssid, 6) == 0,
        "downlink addr2 is the BSSID");

  /* The builders and the length function must not drift apart. */
  check(up.size() == data_hdr_len(up[0], up[1]),
        "data_hdr_to_ds's size matches data_hdr_len");
  check(down.size() == data_hdr_len(down[0], down[1]),
        "data_hdr_from_ds's size matches data_hdr_len");

  /* Every QoS data subtype, not just QoS Data. A real station sends QoS Null
   * (0xc8); an exact fc0 == 0x88 test reads its body two bytes early. */
  check(is_qos_data(0x88), "QoS Data is QoS");
  check(is_qos_data(0xc8), "QoS Null is QoS");
  check(is_qos_data(0x98), "QoS Data+CF-Ack is QoS");
  check(!is_qos_data(0x08), "plain Data is not QoS");
  check(!is_qos_data(0x48), "Null (non-QoS) is not QoS");
  check(!is_qos_data(0x80), "a Beacon is not QoS data");
  check(data_hdr_len(0xc8, kFcToDs) == 26, "QoS Null carries the QoS field");
  /* HT Control rides the Order bit on a QoS frame, and means something else
   * on a non-QoS one. */
  check(data_hdr_len(kFcQosData, kFcToDs | 0x80) == 30,
        "QoS + Order adds the 4-byte HT Control field");
  check(data_hdr_len(kFcData, kFcToDs | 0x80) == 24,
        "Order on a non-QoS frame adds nothing");

  check(data_hdr_len(kFcData, kFcToDs) == 24, "a 3-address data header is 24");
  check(data_hdr_len(kFcQosData, kFcToDs) == 26, "QoS data adds 2 bytes");
  check(data_hdr_len(kFcData, kFcToDs | kFcFromDs) == 30,
        "a 4-address frame adds 6 bytes");
  check(data_hdr_len(kFcQosData, kFcToDs | kFcFromDs) == 32,
        "4-address QoS adds both");

  std::vector<uint8_t> llc;
  append_llc_snap(llc, 0x0800);
  check(llc.size() == 8 && llc[0] == 0xaa && llc[1] == 0xaa && llc[2] == 0x03,
        "LLC/SNAP prefix");
  check(llc[6] == 0x08 && llc[7] == 0x00, "ethertype is big-endian in SNAP");
}


/* RSN elements as they actually appear in the field.
 *
 * A byte comparison against a canonical one-pairwise/one-AKM layout would
 * report every mixed-mode WPA/WPA2 AP and every WPA3-transition AP as
 * unusable, and a station would skip BSSes it can join. These cases are those
 * real-world shapes, plus the malformed ones that must be refused. */
namespace rsn {

std::vector<uint8_t> build(uint16_t ver, const std::vector<uint32_t>& group,
                           const std::vector<uint32_t>& pairwise,
                           const std::vector<uint32_t>& akm,
                           bool caps, uint16_t capval) {
  std::vector<uint8_t> v;
  auto suite = [&](uint32_t t) {
    v.push_back(0x00); v.push_back(0x0f); v.push_back(0xac);
    v.push_back((uint8_t)t);
  };
  put_le16(v, ver);
  for (uint32_t g : group) suite(g);
  if (!pairwise.empty() || !akm.empty() || caps) {
    put_le16(v, (uint16_t)pairwise.size());
    for (uint32_t c : pairwise) suite(c);
  }
  if (!akm.empty() || caps) {
    put_le16(v, (uint16_t)akm.size());
    for (uint32_t a : akm) suite(a);
  }
  if (caps) put_le16(v, capval);
  return v;
}

}  // namespace rsn

void test_rsn_real_world() {
  RsnInfo r;

  /* The canonical one-of-each element. */
  auto plain = rsn::build(1, {4}, {4}, {2}, true, 0x0000);
  check(parse_rsn(plain.data(), plain.size(), &r), "canonical RSN parses");
  check(r.group_ccmp && r.pairwise_ccmp && r.akm_psk, "canonical is CCMP/PSK");
  check(!r.mfp_required && !r.mfp_capable, "canonical has no MFP");

  /* MIXED MODE: TKIP group, TKIP+CCMP pairwise, PSK - a very common real
   * AP. The group cipher is TKIP, so this is NOT joinable by this
   * project - but the PAIRWISE search must still find CCMP. */
  auto mixed = rsn::build(1, {2}, {2, 4}, {2}, true, 0x0000);
  check(parse_rsn(mixed.data(), mixed.size(), &r), "mixed-mode RSN parses");
  check(r.pairwise_count == 2, "two pairwise suites are counted");
  check(r.pairwise_ccmp, "CCMP is found AMONG several pairwise suites");
  check(!r.group_ccmp, "a TKIP group cipher is reported as not CCMP");

  /* WPA2-only AP that still lists two pairwise suites (CCMP first). */
  auto two_cc = rsn::build(1, {4}, {4, 2}, {2}, true, 0x0000);
  check(parse_rsn(two_cc.data(), two_cc.size(), &r), "parses");
  check(r.group_ccmp && r.pairwise_ccmp && r.akm_psk,
        "CCMP group with a TKIP fallback pairwise is joinable");

  /* WPA3 TRANSITION: CCMP, AKM = PSK + PSK-SHA256, MFP capable but not
   * required - an extremely common real AP. */
  auto trans = rsn::build(1, {4}, {4}, {2, 6}, true, 0x0080);
  check(parse_rsn(trans.data(), trans.size(), &r), "WPA3-transition parses");
  check(r.akm_count == 2, "two AKMs are counted");
  check(r.akm_psk, "PSK is found AMONG several AKMs");
  check(r.mfp_capable && !r.mfp_required, "MFP capable, not required");

  /* WPA3-ONLY: SAE AKM, MFP required. Must NOT be reported as joinable. */
  auto sae = rsn::build(1, {4}, {4}, {8}, true, 0x00c0);
  check(parse_rsn(sae.data(), sae.size(), &r), "WPA3-only parses");
  check(!r.akm_psk, "SAE is not PSK");
  check(r.mfp_required, "WPA3-only requires MFP");

  /* A vendor OUI must not be mistaken for an 802.11 suite of the same type. */
  std::vector<uint8_t> vendor = plain;
  vendor[8] = 0x00; vendor[9] = 0x50; vendor[10] = 0xf2;  /* pairwise OUI */
  check(parse_rsn(vendor.data(), vendor.size(), &r), "vendor-OUI RSN parses");
  check(!r.pairwise_ccmp, "a vendor OUI is not 00-0F-AC CCMP");

  /* Truncation and malformed counts: refuse, never read past the end. */
  check(!parse_rsn(plain.data(), 1, &r), "a 1-byte RSN body is refused");
  auto bad_ver = rsn::build(2, {4}, {4}, {2}, true, 0);
  check(!parse_rsn(bad_ver.data(), bad_ver.size(), &r),
        "an unknown RSN version is refused");
  std::vector<uint8_t> overrun = plain;
  overrun[6] = 0xff; overrun[7] = 0xff;  /* pairwise count = 65535 */
  check(!parse_rsn(overrun.data(), overrun.size(), &r),
        "a pairwise count that overruns the element is refused");
  std::vector<uint8_t> akm_overrun = plain;
  akm_overrun[12] = 0xff; akm_overrun[13] = 0xff;
  check(!parse_rsn(akm_overrun.data(), akm_overrun.size(), &r),
        "an AKM count that overruns the element is refused");

  /* An element that stops BEFORE RSN Capabilities is accepted, with MFPR
   * defaulting to 0 (trailing fields may be omitted). Pinned both at
   * parse_rsn and through parse_beacon, so a flip in either direction is a
   * failing cell rather than a silent change. */
  auto no_caps = rsn::build(1, {4}, {4}, {2}, false, 0);
  check(no_caps.size() == 18, "the no-capabilities element is 18 bytes");
  check(parse_rsn(no_caps.data(), no_caps.size(), &r),
        "an RSN element without capabilities parses");
  check(r.group_ccmp && r.pairwise_ccmp && r.akm_psk,
        "its suites are still read");
  check(!r.mfp_required && !r.mfp_capable,
        "absent capabilities mean MFPR=0, not unknown");
  {
    std::vector<uint8_t> m = mgmt_hdr(kFcBeacon,
                                      (const uint8_t*)"\xff\xff\xff\xff\xff\xff",
                                      kOwn, kBssid);
    for (int i = 0; i < 8; i++) m.push_back(0);
    put_le16(m, 100); put_le16(m, 0x0011);
    append_ssid(m, "ap");
    append_ie(m, kEidRsn, no_caps.data(), no_caps.size());
    BssInfo b;
    check(parse_beacon(m.data(), m.size(), &b), "parses");
    check(b.rsn_ccmp_psk,
          "a BSS whose RSN element omits capabilities IS joinable");
  }

  /* A count landing exactly on the element boundary is legal; one byte past
   * is not. `>` vs `>=` in the overrun check is the difference. */
  auto exact = rsn::build(1, {4}, {4}, {2}, false, 0);
  check(parse_rsn(exact.data(), exact.size(), &r),
        "a count reaching exactly the element end is accepted");
  check(!parse_rsn(exact.data(), exact.size() - 1, &r),
        "one byte short of that same count is refused");

  /* A refused element must leave NO partial state behind. */
  std::vector<uint8_t> ov = plain;
  ov[6] = 0xff; ov[7] = 0xff;
  check(!parse_rsn(ov.data(), ov.size(), &r), "setup: refused");
  check(r.pairwise_count == 0 && !r.pairwise_ccmp && !r.group_ccmp,
        "a refused element leaves no partial state");

  check(!parse_rsn(nullptr, 20, &r), "a null body is refused");

  /* A short-but-legal element stops early and leaves later flags false. */
  auto group_only = rsn::build(1, {4}, {}, {}, false, 0);
  check(parse_rsn(group_only.data(), group_only.size(), &r),
        "an element with only a group cipher is legal");
  check(r.group_ccmp && !r.pairwise_ccmp && !r.akm_psk,
        "absent lists leave their flags false");

  /* And the end-to-end verdict through parse_beacon: a WPA3-transition BSS
   * is JOINABLE, and an MFP-required one is not. */
  auto mkbeacon = [&](const std::vector<uint8_t>& ie) {
    std::vector<uint8_t> m = mgmt_hdr(kFcBeacon,
                                      (const uint8_t*)"\xff\xff\xff\xff\xff\xff",
                                      kOwn, kBssid);
    for (int i = 0; i < 8; i++) m.push_back(0);
    put_le16(m, 100);
    put_le16(m, 0x0011);
    append_ssid(m, "ap");
    append_ie(m, kEidRsn, ie.data(), ie.size());
    return m;
  };
  BssInfo b;
  auto bt = mkbeacon(trans);
  check(parse_beacon(bt.data(), bt.size(), &b), "transition beacon parses");
  check(b.rsn_ccmp_psk,
        "a WPA3-TRANSITION BSS is joinable");
  auto bm = mkbeacon(mixed);
  check(parse_beacon(bm.data(), bm.size(), &b), "mixed beacon parses");
  check(!b.rsn_ccmp_psk, "a TKIP-group BSS is not joinable");
  auto bs = mkbeacon(sae);
  check(parse_beacon(bs.data(), bs.size(), &b), "WPA3-only beacon parses");
  check(!b.rsn_ccmp_psk && b.rsn_mfp_required,
        "an MFP-REQUIRED BSS is not joinable and says why");
}

/* Data frames carry sequence numbers too. Management frames are the visible
 * half; the data plane is the one that feeds a duplicate detector in
 * volume. */
void test_data_seq() {
  const uint8_t dest[6] = {1, 2, 3, 4, 5, 6};

  std::vector<uint8_t> a = data_hdr_to_ds(kBssid, kOwn, dest, true, 1);
  check(a[22] == 0x10 && a[23] == 0x00, "uplink carries its sequence number");
  std::vector<uint8_t> b = data_hdr_from_ds(kOwn, kBssid, dest, false, 0x0fff);
  check(b[22] == 0xf0 && b[23] == 0xff, "downlink carries its sequence number");
  std::vector<uint8_t> c = data_hdr_to_ds(kBssid, kOwn, dest, true);
  check(c[22] == 0 && c[23] == 0, "the default is still zero");
  std::vector<uint8_t> d = data_hdr_to_ds(kBssid, kOwn, dest, true, 0x1001);
  check(d[22] == 0x10 && d[23] == 0x00, "a data sequence wraps at 12 bits");
}

}  // namespace

/* The TIM element (802.11-2016 9.4.2.6).
 *
 * Every conforming beacon carries one. The element built here is the MINIMUM
 * conforming one - "nothing is buffered for anyone" - which is the truth for
 * a sender that buffers nothing. It is not power-save support: a dozing
 * station still misses replies that are never buffered, whatever schedule
 * the beacon advertises (see append_tim in Dot11.h). The test pins exactly
 * that content. */
void test_tim() {
  std::vector<uint8_t> m;
  append_tim(m);

  check(m.size() == 6, "tim: 2 bytes of header and 4 of body");
  check(m[0] == kEidTim, "tim: element id 5");
  check(m[1] == 4, "tim: length 4 - the minimum conforming body");
  check(m[2] == 0, "tim: DTIM count 0 - this beacon IS a DTIM beacon");
  check(m[3] == 1, "tim: DTIM period 1 - every beacon is, so nobody waits");
  check(m[4] == 0, "tim: bitmap control 0 - offset 0, no buffered group traffic");
  check(m[5] == 0, "tim: empty partial virtual bitmap - nothing buffered");

  /* The walker must accept it, since a real station parses beacons with it. */
  size_t len = 0;
  const uint8_t* found = find_ie(m.data(), m.size(), kEidTim, &len);
  check(found != nullptr && len == 4, "tim: the IE walker finds it");

  /* An AID inside the one-octet bitmap sets exactly its own bit. */
  std::vector<uint8_t> a3;
  append_tim(a3, 0, 1, 3);
  check(a3[5] == 0x08, "tim: AID 3 sets bit 3 and nothing else");
  std::vector<uint8_t> a7;
  append_tim(a7, 0, 1, 7);
  check(a7[5] == 0x80, "tim: AID 7 sets the top bit of the octet");

  /* AID 0 is not a station - it is the group-addressed indication, and that
   * lives in bit 0 of the BITMAP CONTROL octet, not the bitmap. Setting
   * bitmap bit 0 for "AID 0" would announce buffered multicast that does not
   * exist and make a station wait for it. */
  std::vector<uint8_t> a0;
  append_tim(a0, 0, 1, 0);
  check(a0[5] == 0, "tim: AID 0 sets no bitmap bit");
  check(a0[4] == 0, "tim: AID 0 does not claim buffered group traffic");

  /* An AID this minimum form CANNOT express must set nothing rather than
   * half-encode it - a wrapped shift would set some other station's bit and
   * tell the wrong peer to stay awake. */
  std::vector<uint8_t> big;
  append_tim(big, 0, 1, 8);
  check(big[5] == 0, "tim: AID 8 is out of range for a one-octet bitmap");
  std::vector<uint8_t> huge;
  append_tim(huge, 0, 1, 2007);
  check(huge[5] == 0, "tim: the maximum AID does not wrap into someone else's bit");

  /* DTIM count and period are passed through, for a caller that does buffer. */
  std::vector<uint8_t> d;
  append_tim(d, 2, 3);
  check(d[2] == 2 && d[3] == 3, "tim: DTIM count and period are carried");

  /* BEACON ONLY. 802.11-2016 9.4.2.6 puts the TIM in the Beacon frame body;
   * a TIM in a probe response is a malformed frame some stations reject
   * outright. The harnesses build beacons and probe/assoc responses from the
   * SAME IE helper with a flag, so nothing but this pins the flag - and
   * wiring that is correct by inspection but unpinned is exactly what
   * regresses unnoticed.
   *
   * Modelled the way the harnesses order it: SSID, rates, DS Params, then
   * the TIM only when the frame is a beacon. */
  auto ies = [](bool beacon) {
    std::vector<uint8_t> m;
    append_ssid(m, "devourerAP");
    append_supported_rates(m);
    append_ds_params(m, 6);
    if (beacon) append_tim(m);
    return m;
  };
  size_t n = 0;
  check(find_ie(ies(true).data(), ies(true).size(), kEidTim, &n) != nullptr,
        "tim: present when the frame is a beacon");
  check(find_ie(ies(false).data(), ies(false).size(), kEidTim, &n) == nullptr,
        "tim: ABSENT from a probe/assoc response");
  /* And it must come after the DS Parameter Set, which is the element order
   * the standard gives and which a strict parser will check. */
  {
    std::vector<uint8_t> b = ies(true);
    size_t dl = 0, tl = 0;
    const uint8_t* ds = find_ie(b.data(), b.size(), kEidDsParams, &dl);
    const uint8_t* tm = find_ie(b.data(), b.size(), kEidTim, &tl);
    check(ds != nullptr && tm != nullptr && ds < tm,
          "tim: ordered after the DS Parameter Set");
  }
}

/* Direction-aware DA/SA, against 802.11-2016 Table 9-26, and the
 * relayed header an AP has to build.
 *
 * A receive path that takes addr1 and addr2 and assumes the frame is for the
 * AP holds exactly until there is a second station to forward to; the
 * destination is addr3 on a to-DS frame.
 *
 * The relay cell is the load-bearing one. It builds the to-DS frame station A
 * sends for station B, extracts DA and SA with the accessors, rebuilds the
 * from-DS frame the AP must air, and byte-compares the result against the
 * layout written out by hand - not against another call of the same builder,
 * which would only prove the builder agrees with itself. */
static void test_relay_addressing() {
  const uint8_t A[6]     = {0x02, 0xaa, 0, 0, 0, 0x01};   /* station A */
  const uint8_t B[6]     = {0x02, 0xbb, 0, 0, 0, 0x02};   /* station B */
  const uint8_t BSSID[6] = {0x02, 0x42, 0x75, 0x05, 0xd6, 0x00};
  const uint8_t GRP[6]   = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

  /* --- to-DS: addr1 = BSSID, addr2 = SA, addr3 = DA ---------------------- */
  /* data_hdr_to_ds is (bssid, own, dest) - A sending to B. A caller that
   * guesses the order as (dest, bssid, own) fails here, so the cell pins the
   * argument order as well as the bytes. */
  std::vector<uint8_t> up = devourer::sta::data_hdr_to_ds(BSSID, A, B,
                                                          /*protect=*/false, 0);
  check(up.size() == 24, "a to-DS data header is 24 bytes");
  check(std::memcmp(up.data() + 4, BSSID, 6) == 0, "to-DS addr1 is the BSSID");
  check(std::memcmp(up.data() + 10, A, 6) == 0,    "to-DS addr2 is the source");
  check(std::memcmp(up.data() + 16, B, 6) == 0,    "to-DS addr3 is the destination");
  check(std::memcmp(devourer::sta::data_da(up.data(), up[1]), B, 6) == 0,
        "data_da reads addr3 on a to-DS frame");
  check(std::memcmp(devourer::sta::data_sa(up.data(), up[1]), A, 6) == 0,
        "data_sa reads addr2 on a to-DS frame");

  /* --- the relay: same payload, rebuilt as from-DS ----------------------- */
  const uint8_t* da = devourer::sta::data_da(up.data(), up[1]);
  const uint8_t* sa = devourer::sta::data_sa(up.data(), up[1]);
  std::vector<uint8_t> down = devourer::sta::data_hdr_from_ds(da, BSSID, sa,
                                                              /*protect=*/false, 0);
  /* Written out by hand rather than by calling the builder again. */
  uint8_t want[24] = {0};
  want[0] = 0x08;                       /* type data, subtype data */
  want[1] = 0x02;                       /* From DS */
  std::memcpy(want + 4,  B, 6);         /* addr1 = DA */
  std::memcpy(want + 10, BSSID, 6);     /* addr2 = BSSID */
  std::memcpy(want + 16, A, 6);         /* addr3 = SA */
  check(down.size() == 24 && std::memcmp(down.data(), want, 24) == 0,
        "the relayed from-DS header is byte-for-byte Table 9-26");

  /* And it round-trips: the relayed frame's DA/SA are the originals. */
  check(std::memcmp(devourer::sta::data_da(down.data(), down[1]), B, 6) == 0,
        "data_da reads addr1 on a from-DS frame");
  check(std::memcmp(devourer::sta::data_sa(down.data(), down[1]), A, 6) == 0,
        "data_sa reads addr3 on a from-DS frame");

  /* --- a station's broadcast is individually addressed to the AP --------- */
  std::vector<uint8_t> bc = devourer::sta::data_hdr_to_ds(BSSID, A, GRP,
                                                          /*protect=*/false, 0);
  check((bc[4] & 0x01) == 0,
        "a station's broadcast has an INDIVIDUAL addr1 - it goes to the AP");
  check(devourer::sta::data_da_is_group(bc.data(), bc[1]),
        "...while its DA in addr3 is the group address");

  /* This distinction is why the uplink is pairwise-protected however broadcast
   * its payload: the key follows addr1, not the DA. */
  check(!devourer::sta::data_da_is_group(up.data(), up[1]),
        "a unicast relay's DA is not a group address");

  /* --- IBSS (ToDS=0, FromDS=0): DA is addr1, SA is addr2 ----------------- */
  uint8_t ibss[24] = {0};
  ibss[0] = 0x08; ibss[1] = 0x00;
  std::memcpy(ibss + 4,  B, 6);        /* addr1 = DA */
  std::memcpy(ibss + 10, A, 6);        /* addr2 = SA */
  std::memcpy(ibss + 16, BSSID, 6);    /* addr3 = BSSID */
  check(std::memcmp(devourer::sta::data_da(ibss, ibss[1]), B, 6) == 0,
        "IBSS DA is addr1");
  check(std::memcmp(devourer::sta::data_sa(ibss, ibss[1]), A, 6) == 0,
        "IBSS SA is addr2");
  check(!devourer::sta::data_da_is_group(ibss, ibss[1]),
        "an IBSS unicast DA is not a group address");

  /* --- a from-DS frame whose DA (addr1) is the group address ------------- */
  std::vector<uint8_t> flood = devourer::sta::data_hdr_from_ds(GRP, BSSID, A,
                                                               /*protect=*/false, 0);
  check(devourer::sta::data_da_is_group(flood.data(), flood[1]),
        "a from-DS flood reads its group DA from addr1, not addr3");

  /* --- 4-address: SA moves to addr4 -------------------------------------- */
  uint8_t four[30] = {0};
  four[0] = 0x08;
  four[1] = (uint8_t)(devourer::sta::kFcToDs | devourer::sta::kFcFromDs);
  std::memcpy(four + 16, B, 6);         /* addr3 = DA */
  std::memcpy(four + 24, A, 6);         /* addr4 = SA */
  check(std::memcmp(devourer::sta::data_da(four, four[1]), B, 6) == 0,
        "4-address DA is addr3");
  check(std::memcmp(devourer::sta::data_sa(four, four[1]), A, 6) == 0,
        "4-address SA is addr4, not addr2");
}

/* 802.11 MSDU <-> Ethernet II.
 *
 * The round trip is the weakest possible test on its own: an encoder and a
 * decoder that share a misreading round-trip perfectly. So the cells below
 * assert the WIRE BYTES by hand first, and only then round-trip. */
static void test_eth_translation() {
  const uint8_t DA[6] = {0x02, 0xbb, 0, 0, 0, 0x02};
  const uint8_t SA[6] = {0x02, 0xaa, 0, 0, 0, 0x01};
  const uint8_t payload[4] = {0xde, 0xad, 0xbe, 0xef};

  /* An MSDU as it appears on air: LLC/SNAP then payload. */
  uint8_t msdu[12] = {0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00, 0x08, 0x00,
                      0xde, 0xad, 0xbe, 0xef};
  uint8_t eth[64];
  size_t n = devourer::sta::msdu_to_eth(DA, SA, msdu, sizeof msdu,
                                        eth, sizeof eth);
  check(n == 14 + 4, "an MSDU becomes an Ethernet frame 6 bytes shorter");
  check(std::memcmp(eth, DA, 6) == 0, "Ethernet DA comes first");
  check(std::memcmp(eth + 6, SA, 6) == 0, "then the SA");
  check(eth[12] == 0x08 && eth[13] == 0x00,
        "then the ethertype, lifted out of the SNAP header");
  check(std::memcmp(eth + 14, payload, 4) == 0, "then the payload, unchanged");

  /* And back. The addresses come out through the out-parameters, because the
   * caller needs them for the 802.11 header rather than for the MSDU. */
  uint8_t back[64], bda[6], bsa[6];
  size_t m = devourer::sta::eth_to_msdu(eth, n, back, sizeof back, bda, bsa);
  check(m == sizeof msdu, "and back to the same length");
  check(std::memcmp(back, msdu, sizeof msdu) == 0,
        "byte-for-byte the MSDU we started with");
  check(std::memcmp(bda, DA, 6) == 0 && std::memcmp(bsa, SA, 6) == 0,
        "with the addresses handed back out");

  /* --- refusals, all of which must return 0 rather than truncate -------- */
  uint8_t small[8];
  check(devourer::sta::msdu_to_eth(DA, SA, msdu, sizeof msdu,
                                   small, sizeof small) == 0,
        "msdu_to_eth refuses an output buffer that cannot hold the result");
  check(devourer::sta::eth_to_msdu(eth, n, small, sizeof small, bda, bsa) == 0,
        "eth_to_msdu refuses the same way");

  /* An MSDU that is not an ethertype SNAP carries no ethertype at bytes 6..7,
   * so rewriting it would invent one. Other LLC encodings are real. */
  uint8_t not_snap[12];
  std::memcpy(not_snap, msdu, sizeof msdu);
  not_snap[2] = 0x04;                       /* control field, not 0x03 */
  check(devourer::sta::msdu_to_eth(DA, SA, not_snap, sizeof not_snap,
                                   eth, sizeof eth) == 0,
        "a non-SNAP MSDU is refused, not reinterpreted");
  not_snap[2] = 0x03; not_snap[3] = 0x01;   /* non-zero OUI */
  check(devourer::sta::msdu_to_eth(DA, SA, not_snap, sizeof not_snap,
                                   eth, sizeof eth) == 0,
        "a SNAP header with a non-zero OUI is refused too");

  check(devourer::sta::msdu_to_eth(DA, SA, msdu, 7, eth, sizeof eth) == 0,
        "an MSDU shorter than its own SNAP header is refused");
  check(devourer::sta::eth_to_msdu(eth, 13, back, sizeof back, bda, bsa) == 0,
        "an Ethernet frame shorter than its own header is refused");

  /* A zero-payload frame is legal and must survive both ways rather than
   * being refused as if it were truncated. */
  uint8_t bare[8] = {0xaa, 0xaa, 0x03, 0, 0, 0, 0x86, 0xdd};
  n = devourer::sta::msdu_to_eth(DA, SA, bare, sizeof bare, eth, sizeof eth);
  check(n == 14, "an MSDU with no payload becomes a bare Ethernet header");
  check(eth[12] == 0x86 && eth[13] == 0xdd, "carrying its ethertype");
  m = devourer::sta::eth_to_msdu(eth, n, back, sizeof back, bda, bsa);
  check(m == 8 && std::memcmp(back, bare, 8) == 0, "and round-trips");
}

/* DupDetector - the 802.11 duplicate cache (Retry set AND the same Sequence
 * Control as the last frame accepted on that TID). Each rule its comment
 * states, one cell each. */
void test_dup_detector() {
  using devourer::sta::DupDetector;
  const auto sc = [](uint16_t seq, uint8_t frag) {
    return (uint16_t)((seq << 4) | (frag & 0x0f));
  };

  {
    DupDetector d;
    check(!d.is_duplicate(true, sc(10, 0)),
          "dup: the first frame is never a duplicate, even with Retry set");
    check(d.is_duplicate(true, sc(10, 0)),
          "dup: a retry of the same sequence control is dropped");
    check(d.is_duplicate(true, sc(10, 0)),
          "dup: ...every time it is retried");
    check(!d.is_duplicate(false, sc(10, 0)),
          "dup: the same number WITHOUT Retry is not a duplicate - only a "
          "retransmission can be one");
    check(!d.is_duplicate(true, sc(11, 0)),
          "dup: a new sequence number is accepted, Retry or not");
    check(d.is_duplicate(true, sc(11, 0)),
          "dup: ...and becomes the one a retry is compared with");
    check(!d.is_duplicate(true, sc(11, 1)),
          "dup: the next fragment of the same MSDU is not a duplicate");
  }
  {
    /* WRAP: 4095 then 0 is the ordinary next number, not a repeat. */
    DupDetector d;
    check(!d.is_duplicate(false, sc(4095, 0)), "dup wrap: 4095 accepted");
    check(!d.is_duplicate(true, sc(0, 0)),
          "dup wrap: 0 after 4095 is new, even with Retry set");
    check(d.is_duplicate(true, sc(0, 0)), "dup wrap: ...and its retry drops");
  }
  {
    /* PER TID: the cache keeps one "last" per TID, so interleaved access
     * categories do not evict each other. */
    DupDetector d;
    check(!d.is_duplicate(false, sc(20, 0), 5), "dup tid: TID 5 accepts 20");
    check(!d.is_duplicate(false, sc(900, 0), 6), "dup tid: TID 6 accepts 900");
    check(d.is_duplicate(true, sc(20, 0), 5),
          "dup tid: TID 5's retry still matches after TID 6 moved");
    check(!d.is_duplicate(true, sc(20, 0), 6),
          "dup tid: the same number on another TID is not its duplicate");
    check(!d.is_duplicate(true, sc(20, 0)),
          "dup tid: nor on the non-QoS slot");
    /* An out-of-range TID folds onto the non-QoS slot rather than indexing
     * out of bounds. */
    check(d.is_duplicate(true, sc(20, 0), 99),
          "dup tid: an out-of-range TID uses the non-QoS slot");
    check(d.is_duplicate(true, sc(20, 0), -1),
          "dup tid: ...a negative one too");
  }
  {
    DupDetector d;
    d.is_duplicate(false, sc(7, 0));
    d.reset();
    check(!d.is_duplicate(true, sc(7, 0)),
          "dup: after reset() nothing is remembered");
  }
}

int main() {
  test_tim();
  test_dup_detector();
  test_mgmt_hdr();
  test_seq();
  test_ie_walk();
  test_golden_bytes();
  test_rsn();
  test_beacon_roundtrip();
  test_station_builders();
  test_assoc_resp_parse();
  test_data_frames();
  test_rsn_real_world();
  test_data_seq();
  test_relay_addressing();
  test_eth_translation();

  if (g_fail) {
    std::printf("dot11_selftest: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("dot11_selftest: OK\n");
  return 0;
}
