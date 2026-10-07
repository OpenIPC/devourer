/* sta_client_selftest.inc — the headless half of tests/sta_client.cpp.
 *
 * WHY THIS IS AN .inc AND NOT A .cpp. Everything it drives lives in
 * sta_client.cpp's anonymous namespace: rx_frame(), tap_down_one(),
 * supervise(), the keys, the replay windows and the ledger. A separate
 * translation unit cannot reach any of it, and giving them external linkage
 * would export a harness's internals for anything to link against. So this
 * file is textually included at the end of sta_client.cpp and reached through
 * `sta_client --self-test`, which returns before libusb is touched. It is not
 * a header; do not include it anywhere else.
 *
 * WHAT IT DRIVES. Real frames into the real rx_frame(), the real transmit
 * queue read back out, and the real TAP write path with a pipe standing in
 * for the device. It plays the AUTHENTICATOR: it answers authentication and
 * association, derives the PTK from the same PSK, builds messages 1 and 3
 * with a correct MIC, and encrypts data frames under the PTK it negotiated.
 * Nothing is stubbed; the only missing piece is the radio.
 *
 * WHY THAT MATTERS HERE MORE THAN USUAL. This harness's whole job is the part
 * src/sta/ refused to guess at - the scan, the reconnect policy and the key
 * selection in the data plane - and none of those has any other test. Without
 * this file they would exist only inside an on-air script.
 *
 * WHAT IT DOES NOT COVER, stated rather than implied:
 *   - USB, the radio, SetStationIdentity, the channel retune, and everything
 *     below send_packet(). Those need a device and stay in
 *     tests/sta_client_onair.sh.
 *   - The four-way's INTEROPERABILITY. The fixture below and the code it
 *     tests share an author, so a shared misreading is invisible to it.
 *     tests/eapol_kernel_vectors.h (ctest `supplicant`) is what pins that,
 *     against frames hostapd and wpa_supplicant actually exchanged.
 *   - The TAP's reader THREAD, and the main loop's. tap_down_one() and
 *     supervise() are called directly; the threads around them are a read()
 *     loop and a sleep loop and nothing else.
 */

namespace selftest {

int g_fail = 0;

void check(bool ok, const char* what) {
  if (!ok) {
    std::fprintf(stdout, "FAIL: %s\n", what);
    g_fail++;
  }
}

const uint8_t kBssid[6] = {0x02, 0x42, 0x75, 0x05, 0xd6, 0x00};
const uint8_t kStaMac[6] = {0x02, 0x11, 0x22, 0x33, 0x44, 0x01};
const uint8_t kPeer[6] = {0x02, 0x99, 0x88, 0x77, 0x66, 0x55};
const uint8_t kBcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
const char* kSsid = "devourerSTA";
const char* kPsk = "devourer123";

int g_tap_rd = -1;

/* Every cell starts from a clean process. These are file-scope globals in a
 * harness that normally runs once, so a cell that inherited the previous
 * one's replay window or join counter would pass or fail for reasons its own
 * name does not mention. */
void reset_all() {
  std::lock_guard<std::mutex> l(g_mu);
  g_bss.clear();
  /* DESTROY AND RECONSTRUCT rather than assign: StationSm holds a SeqCounter,
   * whose counter is a std::atomic, so the class is not copy-assignable - and
   * a cell that inherited the previous one's counters would pass or fail for
   * reasons its own name does not mention. The destructor wipes the PMK,
   * which is the behaviour we want here anyway. */
  g_sm.~StationSm();
  ::new (&g_sm) StationSm();
  std::memcpy(g_own, kStaMac, 6);
  g_ssid = kSsid;
  g_psk = kPsk;
  g_tx_pn = 1;
  g_rx_replay.reset();
  g_rx_dup.reset();
  g_dup_drop = 0;
  g_group_replay.reset();
  g_scan_idx = 0;
  g_scan_switch_ms = 0;
  g_next_join_ms = 0;
  g_join_attempts = 0;
  g_gave_up = false;
  g_failed_noted = false;
  g_was_associated = false;
  g_reconnect = true;
  g_rejoin_backoff_ms = 1000;
  g_scan_chans.clear();
  g_scan_chans.push_back(6);
  g_chan = 6;
  g_bss.set_wanted(kSsid);
  { std::lock_guard<std::mutex> q(g_q_mu); g_q.clear(); }
  g_beacons = 0; g_probe_tx = 0; g_joins = 0; g_associations = 0;
  g_reconnects = 0; g_enc_rx = 0; g_mic_fail = 0; g_replays = 0;
  g_group_rx = 0; g_plain_rx = 0; g_plain_refused = 0; g_rx_short = 0;
  g_tap_tx = 0; g_tap_rx = 0; g_tap_drop = 0; g_tap_down_drop = 0;
  g_q_in = 0;
  g_tx_enc = 0; g_tx_enc_fail = 0; g_tx_plain = 0;
  g_crc_err = 0; g_amsdu_drop = 0; g_frag_drop = 0;
  g_eapol_enc_rx = 0; g_eapol_enc_tx = 0;
  g_ptk_installs = 0; g_gtk_installs = 0; g_no_key = 0;
  g_ptk_gen_seen = 0; g_gtk_gen_seen = 0;
  g_scan_dwell_ms = 250;
  g_tuned = 6;
  g_retune_ms = 0u - kRetuneGuardMs;   /* no retune in progress at now = 0 */
  g_fault = 0;
  g_stop = 0;
  g_tap_read_err = 0;
  g_end = RunEnd{};
  g_judge = false;
  g_judge_from_ms = 0;
  g_strikes = 0;
  std::memset(g_strike_bss, 0, 6);
  g_uplink_seen = false;
  g_uplink_first_ms = 0;
  g_uplink_unconfirmed = 0;
  g_unconfirmed_lost = 0;
  g_nudges = 0;
  g_nudge_ms = 0;
  g_nudge_again = false;
  g_nudge_eapol_rx = 0;
}

/* Drain whatever the station queued, stripping the radiotap prefix the real
 * enqueue() adds, so a cell reads MPDUs. */
std::vector<std::vector<uint8_t>> drain_tx() {
  std::vector<std::vector<uint8_t>> out;
  std::lock_guard<std::mutex> l(g_q_mu);
  for (auto& f : g_q) {
    if (f.size() <= g_rt.size()) continue;
    out.emplace_back(f.begin() + (long)g_rt.size(), f.end());
  }
  g_q.clear();
  return out;
}

/* Everything the main loop does between ticks, minus the radio. */
void pump_tx() {
  std::lock_guard<std::mutex> l(g_mu);
  std::vector<uint8_t> f;
  while (g_sm.pop_tx(&f)) {
    devourer::sta::assign_seq(f, g_data_seq.next());
    enqueue(std::move(f));
  }
}

void tick(uint32_t now) {
  std::lock_guard<std::mutex> l(g_mu);
  g_sm.tick(now);
}

/* One Ethernet frame off the TAP pipe, or empty. Non-blocking, so a cell that
 * expects nothing does not hang. */
std::vector<uint8_t> tap_read() {
  uint8_t buf[2048];
  const ssize_t n = ::read(g_tap_rd, buf, sizeof buf);
  if (n <= 0) return {};
  return std::vector<uint8_t>(buf, buf + n);
}

/* ---- the fixture authenticator ----------------------------------------- */

std::vector<uint8_t> beacon(bool rsn, uint8_t chan = 6,
                            const uint8_t* bssid = kBssid,
                            const char* ssid = kSsid, bool with_ds = true) {
  std::vector<uint8_t> m =
      devourer::sta::mgmt_hdr(devourer::sta::kFcBeacon, kBcast, bssid, bssid);
  m.insert(m.end(), 8, 0);
  devourer::sta::put_le16(m, 100);
  devourer::sta::put_le16(m, (uint16_t)(rsn ? 0x0011 : 0x0001));
  devourer::sta::append_ssid(m, ssid);
  devourer::sta::append_supported_rates(m);
  if (with_ds) devourer::sta::append_ds_params(m, chan);
  if (rsn) devourer::sta::append_rsn_ccmp_psk(m);
  return m;
}

struct Ap {
  devourer::test::OpenSslCryptoOps crypto;
  uint8_t pmk[32] = {0};
  uint8_t anonce[32] = {0};
  uint8_t ptk[48] = {0};
  uint8_t gtk[32] = {0};
  uint8_t gtk_id = 1;
  uint64_t replay = 0;
  uint64_t tx_pn = 1;
  uint64_t group_pn = 1;
  uint16_t aid = 3;
  devourer::sta::SeqCounter seq;
  bool saw_auth = false, saw_assoc = false, saw_msg2 = false, saw_msg4 = false;

  Ap() {
    devourer::sta::pmk_from_psk(crypto, kPsk, kSsid, pmk);
    std::memset(anonce, 0x5e, 32);
    std::memset(gtk, 0x31, 32);
  }

  std::vector<uint8_t> mgmt(uint8_t fc) {
    return devourer::sta::mgmt_hdr(fc, kStaMac, kBssid, kBssid);
  }

  std::vector<uint8_t> eapol_frame(const std::vector<uint8_t>& body) {
    std::vector<uint8_t> m = devourer::sta::data_hdr_from_ds(
        kStaMac, kBssid, kBssid, /*protect=*/false, seq.next());
    devourer::sta::append_llc_snap(m, 0x888e);
    m.insert(m.end(), body.begin(), body.end());
    return m;
  }

  std::vector<uint8_t> msg1() {
    replay++;
    return devourer::sta::build_eapol_key(
        devourer::sta::kKeyDescVersionCcmp | devourer::sta::kKiPairwise |
            devourer::sta::kKiAck,
        16, replay, anonce, nullptr, nullptr, 0, nullptr, nullptr);
  }

  /* 16 for CCMP. A KDE may carry 16 to 32 (find_gtk_kde accepts that whole
   * range, because the field is cipher-independent), so an AP can hand a
   * station a group key its data plane cannot use. */
  size_t gtk_len = 16;

  std::vector<uint8_t> msg3() {
    std::vector<uint8_t> kd;
    const uint8_t hdr[8] = {0xdd, (uint8_t)(6 + gtk_len), 0x00, 0x0f, 0xac,
                            0x01, gtk_id, 0x00};
    devourer::sta::append_rsn_ccmp_psk(kd);
    kd.insert(kd.end(), hdr, hdr + 8);
    kd.insert(kd.end(), gtk, gtk + gtk_len);
    if (kd.size() % 8) {
      kd.push_back(0xdd);
      while (kd.size() % 8) kd.push_back(0x00);
    }
    std::vector<uint8_t> w(kd.size() + 8);
    const int n = devourer::test::OpenSslCryptoOps::key_wrap(
        ptk + 16, 16, kd.data(), kd.size(), w.data());
    w.resize(n > 0 ? (size_t)n : 0);
    replay++;
    return devourer::sta::build_eapol_key(
        devourer::sta::kKeyDescVersionCcmp | devourer::sta::kKiPairwise |
            devourer::sta::kKiInstall | devourer::sta::kKiAck |
            devourer::sta::kKiMic | devourer::sta::kKiSecure |
            devourer::sta::kKiEncrypted,
        16, replay, anonce, nullptr, w.data(), w.size(), &crypto, ptk);
  }

  /* Answer one frame the station aired. Returns what the AP would send. */
  std::vector<uint8_t> respond(const std::vector<uint8_t>& f, bool rsn) {
    if (f.size() < 24) return {};
    const uint8_t fc0 = f[0];
    if (fc0 == devourer::sta::kFcAuth) {
      saw_auth = true;
      std::vector<uint8_t> m = mgmt(devourer::sta::kFcAuth);
      devourer::sta::put_le16(m, 0);
      devourer::sta::put_le16(m, 2);
      devourer::sta::put_le16(m, 0);
      return m;
    }
    if (fc0 == devourer::sta::kFcAssocReq) {
      saw_assoc = true;
      std::vector<uint8_t> m = mgmt(devourer::sta::kFcAssocResp);
      devourer::sta::put_le16(m, (uint16_t)(rsn ? 0x0011 : 0x0001));
      devourer::sta::put_le16(m, 0);
      devourer::sta::put_le16(m, (uint16_t)(0xc000 | aid));
      return m;
    }
    if (fc0 != devourer::sta::kFcData) return {};
    const size_t hlen = 24;
    if (f.size() < hlen + 8 + devourer::sta::kEapolKeyFixedLen) return {};
    if (!(f[hlen + 6] == 0x88 && f[hlen + 7] == 0x8e)) return {};
    devourer::sta::EapolKey k;
    if (!devourer::sta::parse_eapol_key(f.data() + hlen + 8,
                                        f.size() - hlen - 8, &k))
      return {};
    if (!k.secure()) {
      saw_msg2 = true;
      if (!devourer::sta::derive_ptk(crypto, pmk, kBssid, kStaMac, anonce,
                                     k.nonce, ptk))
        return {};
      if (devourer::sta::eapol_mic_ok(crypto, ptk, k) !=
          devourer::sta::MicCheck::Ok)
        return {};
      return eapol_frame(msg3());
    }
    saw_msg4 = devourer::sta::eapol_mic_ok(crypto, ptk, k) ==
               devourer::sta::MicCheck::Ok;
    return {};
  }

  /* A downlink data frame to the station, protected under the pairwise key. */
  std::vector<uint8_t> data_to_sta(const uint8_t* payload, size_t len,
                                   uint16_t ethertype = 0x0800) {
    std::vector<uint8_t> pt;
    devourer::sta::append_llc_snap(pt, ethertype);
    pt.insert(pt.end(), payload, payload + len);
    std::vector<uint8_t> hdr = devourer::sta::data_hdr_from_ds(
        kStaMac, kBssid, kPeer, /*protect=*/true, seq.next());
    std::vector<uint8_t> m(
        devourer::sta::ccmp_encrypted_len(hdr.size(), pt.size()));
    const size_t n = devourer::sta::ccmp_encrypt(
        crypto, ptk + 32, hdr.data(), hdr.size(), kBssid, tx_pn++, 0,
        pt.data(), pt.size(), m.data(), m.size());
    m.resize(n);
    return m;
  }

  /* A group frame, under the GTK at the key id message 3 advertised. */
  std::vector<uint8_t> group_to_bss(const uint8_t* payload, size_t len,
                                    uint8_t key_id_override = 0xff) {
    std::vector<uint8_t> pt;
    devourer::sta::append_llc_snap(pt, 0x0806);
    pt.insert(pt.end(), payload, payload + len);
    std::vector<uint8_t> hdr = devourer::sta::data_hdr_from_ds(
        kBcast, kBssid, kPeer, /*protect=*/true, seq.next());
    std::vector<uint8_t> m(
        devourer::sta::ccmp_encrypted_len(hdr.size(), pt.size()));
    const uint8_t kid = key_id_override == 0xff ? gtk_id : key_id_override;
    const size_t n = devourer::sta::ccmp_encrypt(
        crypto, gtk, hdr.data(), hdr.size(), kBssid, group_pn++, kid,
        pt.data(), pt.size(), m.data(), m.size());
    m.resize(n);
    return m;
  }

  /* A QoS downlink, so the A-MSDU and per-TID arms have a frame to use. Built
   * by hand rather than through data_hdr_from_ds, which only makes the
   * non-QoS form - and the two differ by exactly the two octets this cell is
   * about. */
  std::vector<uint8_t> qos_to_sta(const uint8_t* payload, size_t len,
                                  uint8_t tid, bool amsdu) {
    std::vector<uint8_t> pt;
    devourer::sta::append_llc_snap(pt, 0x0800);
    pt.insert(pt.end(), payload, payload + len);
    std::vector<uint8_t> hdr;
    hdr.push_back(0x88);                               /* QoS Data */
    hdr.push_back((uint8_t)(devourer::sta::kFcFromDs |
                            devourer::sta::kFcProtected));
    devourer::sta::put_le16(hdr, 0);
    hdr.insert(hdr.end(), kStaMac, kStaMac + 6);
    hdr.insert(hdr.end(), kBssid, kBssid + 6);
    hdr.insert(hdr.end(), kPeer, kPeer + 6);
    devourer::sta::put_le16(hdr, (uint16_t)((seq.next() & 0x0fff) << 4));
    hdr.push_back((uint8_t)((tid & 0x0f) | (amsdu ? 0x80 : 0)));
    hdr.push_back(0);
    std::vector<uint8_t> m(
        devourer::sta::ccmp_encrypted_len(hdr.size(), pt.size()));
    const size_t n = devourer::sta::ccmp_encrypt(
        crypto, ptk + 32, hdr.data(), hdr.size(), kBssid, tx_pn++, 0,
        pt.data(), pt.size(), m.data(), m.size());
    m.resize(n);
    return m;
  }

  /* A group rekey: message 1 of the group key handshake, carrying a NEW GTK.
   * Not pairwise, not install; ack + MIC + secure + encrypted. */
  std::vector<uint8_t> group_msg1(const uint8_t new_gtk[16], uint8_t key_id,
                                  const uint8_t* rsc = nullptr) {
    std::memcpy(gtk, new_gtk, 16);
    gtk_id = key_id;
    group_pn = 1;                     /* a new key means a new PN space */
    std::vector<uint8_t> kd;
    const uint8_t hdr[8] = {0xdd, 0x16, 0x00, 0x0f, 0xac, 0x01, key_id, 0x00};
    kd.insert(kd.end(), hdr, hdr + 8);
    kd.insert(kd.end(), gtk, gtk + 16);
    std::vector<uint8_t> w(kd.size() + 8);
    const int n = devourer::test::OpenSslCryptoOps::key_wrap(
        ptk + 16, 16, kd.data(), kd.size(), w.data());
    w.resize(n > 0 ? (size_t)n : 0);
    replay++;
    return devourer::sta::build_eapol_key(
        devourer::sta::kKeyDescVersionCcmp | devourer::sta::kKiAck |
            devourer::sta::kKiMic | devourer::sta::kKiSecure |
            devourer::sta::kKiEncrypted,
        16, replay, anonce, rsc, w.data(), w.size(), &crypto, ptk);
  }

  /* The same QoS downlink with an HT CONTROL field, which is what an HT AP
   * sends whenever it wants link adaptation - so, routinely. HT Control sits
   * AFTER the QoS Control, which is why a receiver that looks for the QoS
   * Control at "header length minus two" reads link-adaptation bits instead.
   *
   * AND HT CONTROL IS NOT IN THE CCMP AAD (see ccmp_aad: frame control,
   * addr1-3, the fragment number, addr4, the QoS TID - and nothing else), so
   * an attacker can rewrite it on a captured frame without breaking the MIC.
   * That is what turns a wrong offset from an accounting error into a replay
   * bypass: flip HT Control, and the replayed frame is filed under a
   * different per-TID window from the original. */
  std::vector<uint8_t> qos_htc_to_sta(const uint8_t* payload, size_t len,
                                      uint8_t tid, uint32_t htc,
                                      uint64_t pn_override = 0) {
    std::vector<uint8_t> pt;
    devourer::sta::append_llc_snap(pt, 0x0800);
    pt.insert(pt.end(), payload, payload + len);
    std::vector<uint8_t> hdr;
    hdr.push_back(0x88);                               /* QoS Data */
    hdr.push_back((uint8_t)(devourer::sta::kFcFromDs |
                            devourer::sta::kFcProtected | 0x80)); /* +HTC */
    devourer::sta::put_le16(hdr, 0);
    hdr.insert(hdr.end(), kStaMac, kStaMac + 6);
    hdr.insert(hdr.end(), kBssid, kBssid + 6);
    hdr.insert(hdr.end(), kPeer, kPeer + 6);
    devourer::sta::put_le16(hdr, (uint16_t)((seq.next() & 0x0fff) << 4));
    hdr.push_back((uint8_t)(tid & 0x0f));
    hdr.push_back(0);
    for (int i = 0; i < 4; i++)
      hdr.push_back((uint8_t)((htc >> (8 * i)) & 0xff));
    std::vector<uint8_t> m(
        devourer::sta::ccmp_encrypted_len(hdr.size(), pt.size()));
    const uint64_t pn = pn_override ? pn_override : tx_pn++;
    const size_t n = devourer::sta::ccmp_encrypt(
        crypto, ptk + 32, hdr.data(), hdr.size(), kBssid, pn, 0,
        pt.data(), pt.size(), m.data(), m.size());
    m.resize(n);
    return m;
  }

  /* A PTK rekey: the four-way again, with the association already up. Both
   * messages ride INSIDE the cipher, under the CURRENT pairwise key. */
  std::vector<uint8_t> ptk_rekey_msg1() {
    std::memset(anonce, 0x77, 32);          /* a fresh ANonce, as an AP does */
    return eapol_protected(msg1());
  }

  /* An EAPOL-Key frame protected under the GROUP key and addressed to the
   * BROADCAST. Not part of any handshake, and not the host's either. */
  std::vector<uint8_t> group_eapol(const std::vector<uint8_t>& body) {
    std::vector<uint8_t> pt;
    devourer::sta::append_llc_snap(pt, 0x888e);
    pt.insert(pt.end(), body.begin(), body.end());
    std::vector<uint8_t> hdr = devourer::sta::data_hdr_from_ds(
        kBcast, kBssid, kBssid, /*protect=*/true, seq.next());
    std::vector<uint8_t> m(
        devourer::sta::ccmp_encrypted_len(hdr.size(), pt.size()));
    const size_t n = devourer::sta::ccmp_encrypt(
        crypto, gtk, hdr.data(), hdr.size(), kBssid, group_pn++, gtk_id,
        pt.data(), pt.size(), m.data(), m.size());
    m.resize(n);
    return m;
  }

  /* An EAPOL-Key frame PROTECTED under the pairwise key - which is how a
   * real AP sends a group rekey, because it runs after the PTK is
   * installed. The cleartext eapol_frame() above is the four-way's shape and
   * is not this one. */
  /* `tk` defaults to the currently installed pairwise key. A PTK REKEY has
   * to pass the OLD one explicitly: message 3 carries the material for the
   * new key and is itself protected by the key still in force. Getting this
   * wrong in the FIXTURE produced exactly the failure the code under test
   * must not have - the station could not decrypt message 3 at all. */
  std::vector<uint8_t> eapol_protected(const std::vector<uint8_t>& body,
                                       const uint8_t* tk = nullptr) {
    std::vector<uint8_t> pt;
    devourer::sta::append_llc_snap(pt, 0x888e);
    pt.insert(pt.end(), body.begin(), body.end());
    std::vector<uint8_t> hdr = devourer::sta::data_hdr_from_ds(
        kStaMac, kBssid, kBssid, /*protect=*/true, seq.next());
    std::vector<uint8_t> m(
        devourer::sta::ccmp_encrypted_len(hdr.size(), pt.size()));
    const size_t n = devourer::sta::ccmp_encrypt(
        crypto, tk ? tk : ptk + 32, hdr.data(), hdr.size(), kBssid, tx_pn++,
        0, pt.data(), pt.size(), m.data(), m.size());
    m.resize(n);
    return m;
  }

  /* A UNICAST frame encrypted under the GROUP key. Not a contrivance: an AP
   * may do this during a rekey, and it is the one shape where the address
   * and the key id disagree in the direction that a key-by-address receiver
   * gets WRONG (it reaches for the pairwise key and fails the MIC). */
  std::vector<uint8_t> unicast_under_gtk(const uint8_t* payload, size_t len) {
    std::vector<uint8_t> pt;
    devourer::sta::append_llc_snap(pt, 0x0800);
    pt.insert(pt.end(), payload, payload + len);
    std::vector<uint8_t> hdr = devourer::sta::data_hdr_from_ds(
        kStaMac, kBssid, kPeer, /*protect=*/true, seq.next());
    std::vector<uint8_t> m(
        devourer::sta::ccmp_encrypted_len(hdr.size(), pt.size()));
    const size_t n = devourer::sta::ccmp_encrypt(
        crypto, gtk, hdr.data(), hdr.size(), kBssid, group_pn++, gtk_id,
        pt.data(), pt.size(), m.data(), m.size());
    m.resize(n);
    return m;
  }

  /* An UNPROTECTED data frame, which a WPA2 link must refuse. */
  std::vector<uint8_t> plain_to_sta(const uint8_t* payload, size_t len) {
    return plain_to(kStaMac, payload, len);
  }
  /* ...to any destination: a group address is the AP's broadcast. */
  std::vector<uint8_t> plain_to(const uint8_t da[6], const uint8_t* payload,
                                size_t len) {
    std::vector<uint8_t> m = devourer::sta::data_hdr_from_ds(
        da, kBssid, kPeer, /*protect=*/false, seq.next());
    devourer::sta::append_llc_snap(m, 0x0800);
    m.insert(m.end(), payload, payload + len);
    return m;
  }
};

/* Run the whole ladder: beacon -> supervise joins -> the AP answers until the
 * station is Connected. Returns the time it finished at. */
uint32_t associate(Ap& ap, bool rsn, uint32_t now = 0) {
  const std::vector<uint8_t> b = beacon(rsn);
  rx_frame(b.data(), b.size(), -40, now);
  supervise(now);
  pump_tx();

  for (int round = 0; round < 8; round++) {
    const std::vector<std::vector<uint8_t>> tx = drain_tx();
    if (tx.empty()) break;
    for (const std::vector<uint8_t>& f : tx) {
      const std::vector<uint8_t> r = ap.respond(f, rsn);
      if (!r.empty()) rx_frame(r.data(), r.size(), -40, now);
      if (rsn && f[0] == devourer::sta::kFcAssocReq) {
        const std::vector<uint8_t> m1 = ap.eapol_frame(ap.msg1());
        rx_frame(m1.data(), m1.size(), -40, now);
      }
    }
    pump_tx();
  }
  return now;
}

/* ---- cells -------------------------------------------------------------- */

/* THE OPEN LADDER. It exists so that a station which fails to connect can say
 * whether authentication/association or the key exchange is what broke: on a
 * WPA2 BSS the two come up together or not at all. */
void test_open_ladder() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_psk.clear();
    /* NOT `check(configure_open(...))`: that return is `true` on every path
     * of the function, so the assertion could only fail if someone edited
     * the `return` statement itself. The observable consequence is what is
     * worth asserting. */
    g_sm.configure_open(kSsid, g_own);
    check(g_sm.security() == StationSm::Security::Open,
          "configure_open puts the station on the open path");
  }
  Ap ap;
  associate(ap, /*rsn=*/false);

  check(ap.saw_auth && ap.saw_assoc, "the AP saw both requests");
  check(g_sm.state() == StationSm::State::Connected, "the station connects");
  check(g_sm.connected() && !g_sm.keyed(), "connected, and not keyed");
  check(g_associations.load() == 1, "one association counted");
  check(g_beacons.load() == 1, "the beacon was folded into the scan");

  /* An unprotected downlink is the ONLY kind an open link carries, so it must
   * reach the host - the WPA2 arm below asserts the opposite for the same
   * frame shape, which is what makes either arm mean anything. */
  const uint8_t payload[20] = {0xde, 0xad, 0xbe, 0xef};
  const std::vector<uint8_t> d = ap.plain_to_sta(payload, sizeof payload);
  rx_frame(d.data(), d.size(), -40, 0);
  const std::vector<uint8_t> eth = tap_read();
  check(eth.size() == devourer::sta::kEthHdrLen + sizeof payload,
        "the plaintext frame reaches the host");
  if (eth.size() >= 14) {
    check(std::memcmp(eth.data(), kStaMac, 6) == 0, "...addressed to us");
    check(std::memcmp(eth.data() + 6, kPeer, 6) == 0, "...from the sender");
    check(eth[12] == 0x08 && eth[13] == 0x00, "...with its ethertype");
  }
  check(g_plain_rx.load() == 1 && g_enc_rx.load() == 0,
        "counted as plaintext, not as encrypted");

  /* And the uplink is unprotected too. A station that encrypted here with no
   * key would air a frame under a zero TK. */
  uint8_t e[60] = {0};
  std::memcpy(e, kPeer, 6);
  std::memcpy(e + 6, kStaMac, 6);
  e[12] = 0x08;
  e[13] = 0x00;
  tap_down_one(e, sizeof e);
  const std::vector<std::vector<uint8_t>> tx = drain_tx();
  check(tx.size() == 1, "the host's frame is aired");
  if (tx.size() == 1) {
    check((tx[0][1] & devourer::sta::kFcProtected) == 0,
          "...unprotected, on an open link");
    check((tx[0][1] & devourer::sta::kFcToDs) != 0, "...and to the DS");
  }
  check(g_tx_plain.load() == 1 && g_tx_enc.load() == 0,
        "counted as a plaintext transmission");
}

/* THE WPA2 LADDER, end to end through this harness's own receive path. */
void test_wpa2_ladder() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    check(g_sm.configure(g_crypto, kSsid, kPsk, g_own), "configure");
  }
  Ap ap;
  associate(ap, /*rsn=*/true);

  check(g_sm.state() == StationSm::State::Connected, "the station connects");
  check(g_sm.keyed(), "...and is keyed");
  check(ap.saw_msg2 && ap.saw_msg4, "the AP saw messages 2 and 4");
  check(std::memcmp(g_sm.supplicant().ptk(), ap.ptk, 48) == 0,
        "both sides derived the same PTK");
  check(g_sm.supplicant().gtk_valid() &&
            std::memcmp(g_sm.supplicant().gtk(), ap.gtk, 16) == 0,
        "the GTK the AP sent was installed");
  check(g_sm.supplicant().gtk_key_id() == ap.gtk_id,
        "...at the key id it advertised");
  check(g_gtk_installs.load() == 1, "...and counted as one group key");

  const uint8_t payload[40] = {1, 2, 3, 4, 5};
  const std::vector<uint8_t> d = ap.data_to_sta(payload, sizeof payload);
  rx_frame(d.data(), d.size(), -40, 0);
  const std::vector<uint8_t> eth = tap_read();
  check(eth.size() == devourer::sta::kEthHdrLen + sizeof payload,
        "the encrypted frame decrypts and reaches the host");
  if (eth.size() >= 14)
    check(std::memcmp(eth.data() + 14, payload, sizeof payload) == 0,
          "...with its payload intact");
  check(g_enc_rx.load() == 1 && g_mic_fail.load() == 0 &&
            g_replays.load() == 0,
        "counted as one clean encrypted frame");

  /* The uplink, and the AP decrypts it - which is what proves the station
   * built the AAD and the nonce the way the peer expects, rather than the
   * way this file happens to read them back. */
  uint8_t e[60] = {0};
  std::memcpy(e, kPeer, 6);
  std::memcpy(e + 6, kStaMac, 6);
  e[12] = 0x08;
  e[13] = 0x00;
  for (size_t i = 14; i < sizeof e; i++) e[i] = (uint8_t)i;
  tap_down_one(e, sizeof e);
  const std::vector<std::vector<uint8_t>> tx = drain_tx();
  check(tx.size() == 1, "the host's frame is aired");
  if (tx.size() == 1) {
    const std::vector<uint8_t>& f = tx[0];
    check((f[1] & devourer::sta::kFcProtected) != 0, "...protected");
    check((f[1] & devourer::sta::kFcToDs) != 0 &&
              (f[1] & devourer::sta::kFcFromDs) == 0,
          "...to the DS and not from it");
    check(std::memcmp(f.data() + 4, kBssid, 6) == 0, "...addr1 = the BSSID");
    check(std::memcmp(f.data() + 10, kStaMac, 6) == 0, "...addr2 = us");
    check(std::memcmp(f.data() + 16, kPeer, 6) == 0,
          "...addr3 = the destination");
    check(devourer::sta::ccmp_key_id(f.data() + 24) == 0,
          "...under the pairwise key id");
    uint8_t plain[128];
    size_t plain_len = 0;
    uint64_t pn = 0;
    const bool ok = devourer::sta::ccmp_decrypt(
        ap.crypto, ap.ptk + 32, f.data(), f.size(), 24, f.data() + 10, plain,
        sizeof plain, &plain_len, &pn);
    check(ok, "...and the AP decrypts it");
    if (ok) {
      check(pn == 1, "...at PN 1, the first of this association");
      check(plain_len == devourer::sta::kLlcSnapLen + sizeof e - 14 &&
                std::memcmp(plain + devourer::sta::kLlcSnapLen, e + 14,
                            sizeof e - 14) == 0,
            "...carrying the host's bytes");
    }
  }
  check(g_tx_enc.load() == 1, "counted as one encrypted transmission");
}

/* A replayed frame must be refused AFTER its MIC verified, and must not reach
 * the host. The window is the station's only defence against an attacker who
 * records one frame and airs it a million times. */
void test_replay_is_refused() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  Ap ap;
  associate(ap, /*rsn=*/true);

  const uint8_t payload[16] = {9};
  const std::vector<uint8_t> d = ap.data_to_sta(payload, sizeof payload);
  rx_frame(d.data(), d.size(), -40, 0);
  check(!tap_read().empty(), "the first copy reaches the host");
  rx_frame(d.data(), d.size(), -40, 0);
  check(tap_read().empty(), "the replayed copy does NOT");
  check(g_replays.load() == 1, "...and is counted as a replay");
  check(g_mic_fail.load() == 0, "...not as a MIC failure - the MIC was fine");
  check(g_enc_rx.load() == 2, "both copies were seen");
}

/* A tampered frame. The cipher writes plaintext out BEFORE the tag is
 * checked, so "it decrypted to something" is never the test - the return
 * value is. */
void test_forged_mic_is_refused() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  Ap ap;
  associate(ap, /*rsn=*/true);

  const uint8_t payload[16] = {7};
  std::vector<uint8_t> d = ap.data_to_sta(payload, sizeof payload);
  d[24 + 8] ^= 0x01;                       /* one ciphertext byte */
  rx_frame(d.data(), d.size(), -40, 0);
  check(tap_read().empty(), "a tampered frame does not reach the host");
  check(g_mic_fail.load() == 1, "...and is counted as a MIC failure");
  /* AND THE PN IS NOT ADMITTED. Otherwise an attacker advances the window
   * with garbage and locks the real AP out - the same rule the four-way's
   * replay counter follows. */
  check(g_rx_replay.last(devourer::sta::CcmpReplay::kNonQosTid) == 0,
        "...and its PN never entered the replay window");
  /* So the real AP's frame at the SAME PN the forgery claimed still gets
   * through, and only then does the window move. */
  ap.tx_pn--;
  const std::vector<uint8_t> good = ap.data_to_sta(payload, sizeof payload);
  rx_frame(good.data(), good.size(), -40, 0);
  check(tap_read().size() == devourer::sta::kEthHdrLen + sizeof payload,
        "the real AP's frame at that PN is delivered");
  check(g_rx_replay.last(devourer::sta::CcmpReplay::kNonQosTid) == ap.tx_pn - 1,
        "...and its PN is the one admitted");
  rx_frame(good.data(), good.size(), -40, 0);
  check(tap_read().empty() && g_replays.load() == 1,
        "...and a replay of it is refused");
}

/* The key is chosen by the frame's own key id, not by its address. Guessing
 * from the address is wrong in both directions, and the AP half of this tree
 * had the mirror of that bug - it cost the whole BSS its broadcast traffic. */
void test_group_key_is_chosen_by_key_id() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  Ap ap;
  associate(ap, /*rsn=*/true);

  const uint8_t arp[28] = {0, 1, 8, 0, 6, 4, 0, 1};
  const std::vector<uint8_t> g = ap.group_to_bss(arp, sizeof arp);
  rx_frame(g.data(), g.size(), -40, 0);
  const std::vector<uint8_t> eth = tap_read();
  check(eth.size() == devourer::sta::kEthHdrLen + sizeof arp,
        "a group frame under the GTK reaches the host");
  if (eth.size() >= 14)
    check((eth[0] & 0x01) != 0, "...addressed to the group");
  check(g_group_rx.load() == 1, "...counted as a group frame");

  /* THE ARM THAT ACTUALLY DISCRIMINATES. A UNICAST frame under the group
   * key: the address says pairwise, the key id says group, and they disagree
   * in the direction a key-by-address receiver gets wrong - it reaches for
   * the pairwise key and fails the MIC on a frame that was perfectly good.
   * (A GROUP frame mislabelled key id 0 would not discriminate: it is
   * refused a line later by the installed-GTK key-id check either way.) */
  const uint8_t payload[24] = {0x5a};
  const std::vector<uint8_t> u = ap.unicast_under_gtk(payload, sizeof payload);
  rx_frame(u.data(), u.size(), -40, 0);
  check(tap_read().size() == devourer::sta::kEthHdrLen + sizeof payload,
        "a UNICAST frame under the group key decrypts - the key id decides");
  check(g_mic_fail.load() == 0, "...with no MIC failure");

  /* And the converse: a group frame claiming a key id we hold NO key for.
   *
   * KEY ID 3, NOT 0: the receiver maps 0 to the PAIRWISE branch before the
   * "we hold no key for this id" rule is reached, so the MIC would fail on
   * the cipher and the rule under test would go unexercised. */
  const std::vector<uint8_t> mis = ap.group_to_bss(arp, sizeof arp, 3);
  const uint64_t before = g_mic_fail.load();
  rx_frame(mis.data(), mis.size(), -40, 0);
  check(tap_read().empty(),
        "a group frame at a key id we hold no key for is refused");
  check(g_no_key.load() == 1, "...as a key we do not have");
  check(g_mic_fail.load() == before, "...and NOT as a MIC failure");
}

/* A GROUP KEY THIS DATA PLANE CANNOT USE never reaches it. The GTK KDE's
 * key field is cipher-independent (find_gtk_kde admits 16 to 32 bytes), so
 * an AP can offer a 32-byte group key in message 3; the supplicant refuses
 * it there (Supplicant::kGtkLenCcmp), so the four-way does not complete and
 * the data plane is never handed a key it would misuse. */
void test_a_group_key_we_cannot_use() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  Ap ap;
  ap.gtk_len = 32;                       /* valid KDE, unusable by CCMP */
  associate(ap, /*rsn=*/true);
  check(!g_sm.keyed(), "a 32-byte group key stops the four-way");
  check(!ap.saw_msg4, "...no message 4 answers it");
  check(g_gtk_installs.load() == 0 && g_ptk_installs.load() == 0,
        "...and the data plane installed no key");
}

/* A RETRANSMISSION IS A DUPLICATE, NOT A REPLAY. The AP retries a frame
 * whose ACK it missed: same sequence number, same PN, Retry set. The
 * DupDetector drops it before decrypt; without it, the replay window would
 * count the AP's ordinary retransmissions as attacks. */
void test_a_retransmission_is_a_duplicate() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  Ap ap;
  associate(ap, /*rsn=*/true);
  check(g_sm.keyed(), "the link is keyed");
  const uint8_t pl[16] = {7};
  std::vector<uint8_t> d = ap.data_to_sta(pl, sizeof pl);
  rx_frame(d.data(), d.size(), -40, 0);
  check(tap_read().size() == devourer::sta::kEthHdrLen + sizeof pl,
        "the original arrives");
  /* Retry is masked out of the CCMP AAD, so setting it keeps the MIC valid. */
  d[1] |= devourer::sta::kFcRetry;
  rx_frame(d.data(), d.size(), -40, 0);
  check(tap_read().empty(), "its retransmission does not");
  check(g_dup_drop.load() == 1, "...counted as a duplicate");
  check(g_replays.load() == 0, "...and NOT as a replay");
  /* The control: the same bytes WITHOUT Retry are not a duplicate by 802.11's
   * rule, so they reach the replay window - which refuses the reused PN. */
  d[1] &= (uint8_t)~devourer::sta::kFcRetry;
  rx_frame(d.data(), d.size(), -40, 0);
  check(tap_read().empty() && g_replays.load() == 1,
        "without Retry the same frame is a replay");
}

/* THE DUPLICATE CACHE OUTLIVES A REKEY AND NOT AN ASSOCIATION. Sequence
 * Control is per transmitter and TID (DupDetector, Dot11.h); a PTK rekey does
 * not restart it, so a Retry copy of the rekey's own message 3 - whose ACK
 * the AP missed - is a duplicate. Decrypted instead, it would fail under the
 * new key and read as a MIC failure. A NEW association is a new sequence
 * space, and the cache must not refuse its frames. */
void test_the_dup_cache_spans_a_rekey_not_an_association() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
    g_rejoin_backoff_ms = 100;
  }
  Ap ap;
  associate(ap, /*rsn=*/true);
  uint8_t old_ptk[48];
  std::memcpy(old_ptk, ap.ptk, sizeof old_ptk);
  const std::vector<uint8_t> m1 = ap.ptk_rekey_msg1();
  rx_frame(m1.data(), m1.size(), -40, 0);
  std::vector<std::vector<uint8_t>> tx = drain_tx();
  if (tx.size() == 1) {
    uint8_t plain[256];
    size_t plen = 0;
    uint64_t pn = 0;
    devourer::sta::EapolKey k;
    if (devourer::sta::ccmp_decrypt(ap.crypto, old_ptk + 32, tx[0].data(),
                                    tx[0].size(), 24, tx[0].data() + 10, plain,
                                    sizeof plain, &plen, &pn) &&
        devourer::sta::parse_eapol_key(plain + devourer::sta::kLlcSnapLen,
                                       plen - devourer::sta::kLlcSnapLen, &k))
      devourer::sta::derive_ptk(ap.crypto, ap.pmk, kBssid, kStaMac, ap.anonce,
                                k.nonce, ap.ptk);
  }
  std::vector<uint8_t> m3 = ap.eapol_protected(ap.msg3(), old_ptk + 32);
  rx_frame(m3.data(), m3.size(), -40, 0);
  check(g_ptk_installs.load() == 2, "the rekey installed a new pairwise key");
  drain_tx();

  m3[1] |= devourer::sta::kFcRetry;      /* the AP's retransmission */
  rx_frame(m3.data(), m3.size(), -40, 0);
  check(g_dup_drop.load() == 1, "a Retry copy of message 3 is a duplicate");
  check(g_mic_fail.load() == 0, "...not a MIC failure under the new key");

  /* The link drops and a new association comes up on the same AP address. */
  uint32_t now = StationSm::kBeaconLossMs + 1;
  tick(now);
  supervise(now);
  now += 200;
  const std::vector<uint8_t> b = beacon(true);
  rx_frame(b.data(), b.size(), -40, now);
  supervise(now);
  /* The new association's message 3 completes it, and it is the first frame
   * of that association to reach the duplicate check. Sent here with the
   * Sequence Control the old cache last saw and Retry set: a cache that
   * survived the association would count it as a duplicate. (Message 3's
   * MIC covers the EAPOL body, not the header.) */
  Ap ap2;
  for (int round = 0; round < 8; round++) {
    pump_tx();
    tx = drain_tx();
    if (tx.empty()) break;
    for (const std::vector<uint8_t>& f : tx) {
      std::vector<uint8_t> r = ap2.respond(f, true);
      if (r.size() > 24 && r[0] == devourer::sta::kFcData) {
        r[22] = m3[22];
        r[23] = m3[23];
        r[1] |= devourer::sta::kFcRetry;
      }
      if (!r.empty()) rx_frame(r.data(), r.size(), -40, now);
      if (f[0] == devourer::sta::kFcAssocReq) {
        const std::vector<uint8_t> e = ap2.eapol_frame(ap2.msg1());
        rx_frame(e.data(), e.size(), -40, now);
      }
    }
  }
  check(g_associations.load() == 2, "a second association");
  check(g_dup_drop.load() == 1,
        "its first frame is not a duplicate of the old association's");
}

/* A HOST THAT IS NOT READING COSTS A DROP, NOT A STALL. The TAP is
 * non-blocking (tap_open), and tap_up() runs on the RX thread under g_mu: a
 * full TAP must return EAGAIN and be counted as an up-drop. The fixture's
 * pipe is non-blocking at both ends, like the real fd; filled here, a
 * blocking write would hang this cell. */
void test_a_full_tap_is_a_drop_not_a_stall() {
  reset_all();
  uint8_t fill[4096];
  std::memset(fill, 0, sizeof fill);
  while (::write(g_tap_fd, fill, sizeof fill) > 0) {}
  check(errno == EAGAIN || errno == EWOULDBLOCK, "the TAP is full");

  uint8_t msdu[devourer::sta::kLlcSnapLen + 16];
  std::memset(msdu, 0, sizeof msdu);
  std::vector<uint8_t> snap;
  devourer::sta::append_llc_snap(snap, 0x0800);
  std::memcpy(msdu, snap.data(), snap.size());
  bool delivered = true;
  {
    std::lock_guard<std::mutex> l(g_mu);
    delivered = tap_up(kStaMac, kPeer, msdu, sizeof msdu);
  }
  check(!delivered, "a frame for a full TAP is not delivered");
  check(g_tap_drop.load() == 1 && g_tap_tx.load() == 0,
        "...and is counted as an up-drop");

  /* Empty the pipe for the cells that follow. */
  uint8_t sink[4096];
  while (::read(g_tap_rd, sink, sizeof sink) > 0) {}
}

/* THE DURATION AND THE CHANNEL ARE PARSED STRICTLY: garbage is refused,
 * not read as 0 (a zero-length run, or channel 0). */
void test_duration_and_channel_parse_strictly() {
  int secs = -1;
  check(parse_secs("60", &secs) && secs == 60, "60 seconds");
  check(!parse_secs("0", &secs) && !parse_secs("-5", &secs) &&
            !parse_secs("abc", &secs) && !parse_secs("60s", &secs) &&
            !parse_secs("", &secs),
        "a zero, negative or non-numeric duration is refused");
  uint8_t ch = 0;
  check(parse_channel("6", &ch) && ch == 6, "channel 6");
  check(parse_channel("36", &ch) && ch == 36, "channel 36");
  check(!parse_channel("0", &ch) && !parse_channel("x", &ch) &&
            !parse_channel("6a", &ch) && !parse_channel("300", &ch) &&
            !parse_channel("", &ch),
        "channel 0, garbage or out of range is refused");

  const char* name = "DEVOURER_STA_SELFTEST_MS";
  uint32_t ms = 250;
  ::unsetenv(name);
  check(parse_env_ms(name, kDwellMinMs, kDwellMaxMs, &ms) && ms == 250,
        "an unset millisecond knob keeps its default");
  ::setenv(name, "100", 1);
  check(parse_env_ms(name, kDwellMinMs, kDwellMaxMs, &ms) && ms == 100,
        "100 ms");
  const char* bad[] = {"", "abc", "100ms", "5", "20000", "-1"};
  bool all_refused = true;
  for (const char* b : bad) {
    ::setenv(name, b, 1);
    uint32_t keep = 250;
    if (parse_env_ms(name, kDwellMinMs, kDwellMaxMs, &keep)) all_refused = false;
  }
  check(all_refused, "an empty, non-numeric or out-of-range dwell is refused");
  ::setenv(name, "0", 1);
  check(parse_env_ms(name, kBackoffMinMs, kBackoffMaxMs, &ms) && ms == 0,
        "a zero backoff is a valid one");
  ::setenv(name, "60001", 1);
  check(!parse_env_ms(name, kBackoffMinMs, kBackoffMaxMs, &ms),
        "...and one past its bound is refused");
  ::unsetenv(name);
}

/* A CAUGHT FAULT IS NOT A COMPLETED RUN. Every guard stops the run through
 * the normal teardown, and the exit status is what tells the harness the two
 * apart: 0 would read as "ran out of time". */
void test_a_fault_exits_nonzero() {
  reset_all();
  check(exit_status() == 0, "a run without a fault exits 0");
  fault("selftest", "a deliberate exception");
  check(g_stop.load() == 1, "a fault stops the run");
  check(exit_status() == 3, "...and the run exits 3");
  g_fault = 0;
  g_stop = 0;
}

/* THE STATION DEFAULT RETRY LIMIT applies unless the library actually took
 * DEVOURER_TX_RETRY_LIMIT - an empty or non-numeric value is not one. */
void test_the_retry_limit_env_is_parsed() {
  const char* saved = std::getenv("DEVOURER_TX_RETRY_LIMIT");
  const std::string keep = saved ? saved : "";
  struct Case { const char* v; bool from_env; int limit; } cases[] = {
      {nullptr, false, kStationRetryLimit},
      {"", false, kStationRetryLimit},
      {"5x", false, kStationRetryLimit},
      {"0", true, 0},
      {"12", true, 12},
  };
  for (const Case& c : cases) {
    if (c.v) ::setenv("DEVOURER_TX_RETRY_LIMIT", c.v, 1);
    else ::unsetenv("DEVOURER_TX_RETRY_LIMIT");
    devourer::DeviceConfig cfg = devourer_config_from_env();
    const bool from_env = apply_station_retry_limit(cfg);
    check(from_env == c.from_env && cfg.tx.retry_limit == c.limit,
          c.v ? c.v : "(unset)");
  }
  if (saved) ::setenv("DEVOURER_TX_RETRY_LIMIT", keep.c_str(), 1);
  else ::unsetenv("DEVOURER_TX_RETRY_LIMIT");
}

/* A BEACON IN THE RETUNE WINDOW. No backend reports a frame's RX channel, so
 * one received just before a retune can arrive just after it. While the
 * channel is unknown a beacon that states its own channel (DS Parameter Set)
 * is still folded in; one that does not is not, rather than being tagged
 * with the new channel. */
void test_a_beacon_in_the_retune_window() {
  reset_all();
  const uint8_t other[6] = {0x02, 0x42, 0x75, 0x05, 0xd6, 0x01};
  g_retune_ms = 1000;                       /* a retune just finished */
  const std::vector<uint8_t> no_ds = beacon(false, 6, kBssid, kSsid, false);
  rx_frame(no_ds.data(), no_ds.size(), -40, 1010);
  check(g_beacons.load() == 0, "a DS-less beacon inside the window is dropped");
  const std::vector<uint8_t> ds = beacon(false, 11, other, kSsid, true);
  rx_frame(ds.data(), ds.size(), -40, 1010);
  check(g_beacons.load() == 1, "...one that states its channel is kept");
  rx_frame(no_ds.data(), no_ds.size(), -40, 1000 + kRetuneGuardMs);
  {
    std::lock_guard<std::mutex> l(g_mu);
    const devourer::sta::BssEntry* e = g_bss.find(kBssid);
    check(g_beacons.load() == 2 && e && e->info.channel == 6,
          "...and after the window a DS-less beacon takes the tuned channel");
  }
}

/* AN EAPOL PACKET THAT IS NOT A KEY IS THE HOST'S. on_decrypted_msdu claims
 * only EAPOL-Key frames; an EAP packet under the pairwise key is returned
 * unconsumed and must be delivered, not dropped as if it were a rekey. */
void test_a_non_key_eapol_reaches_the_host() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  Ap ap;
  associate(ap, /*rsn=*/true);
  check(g_sm.keyed(), "the link is keyed");
  /* EAPOL version 2, type 0 (EAP-Packet), body length 4, an EAP request. */
  const uint8_t eap[8] = {2, 0, 0, 4, 1, 1, 0, 4};
  const std::vector<uint8_t> d = ap.data_to_sta(eap, sizeof eap, 0x888e);
  const uint64_t enc_before = g_eapol_enc_rx.load();
  rx_frame(d.data(), d.size(), -40, 0);
  check(tap_read().size() == devourer::sta::kEthHdrLen + sizeof eap,
        "an EAP packet inside the cipher reaches the host");
  check(g_eapol_enc_rx.load() == enc_before,
        "...and is not counted as a rekey");
}

/* THE NUDGE: an accepted Association Response queues one probe request at
 * once, open or WPA2, so an AP that holds the response's TX status until its
 * next transmission is made to transmit (hostapd starts both the association
 * and the four-way from that status). On WPA2 a second one follows if no
 * EAPOL has arrived kNudgeAgainMs later, and none once it has. */
int probes_in(const std::vector<std::vector<uint8_t>>& tx) {
  int n = 0;
  for (const std::vector<uint8_t>& f : tx)
    if (f.size() >= 24 && f[0] == devourer::sta::kFcProbeReq) n++;
  return n;
}

/* Beacon, authentication, association - answered, but no message 1. */
void join_without_msg1(Ap& ap, bool rsn) {
  const std::vector<uint8_t> b = beacon(rsn);
  rx_frame(b.data(), b.size(), -40, 0);
  supervise(0);
  pump_tx();
  for (int round = 0; round < 4; round++) {
    const std::vector<std::vector<uint8_t>> tx = drain_tx();
    if (tx.empty()) break;
    for (const std::vector<uint8_t>& f : tx) {
      if (f.size() >= 24 && f[0] == devourer::sta::kFcProbeReq) {
        enqueue(f);   /* keep it for the cell to see */
        continue;
      }
      const std::vector<uint8_t> r = ap.respond(f, rsn);
      if (!r.empty()) rx_frame(r.data(), r.size(), -40, 0);
    }
    pump_tx();
    if (g_sm.state() != StationSm::State::Authenticating &&
        g_sm.state() != StationSm::State::Associating)
      break;
  }
}

void test_an_accepted_association_nudges_the_ap() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure_open(kSsid, g_own);
  }
  Ap ap;
  join_without_msg1(ap, /*rsn=*/false);
  check(g_sm.state() == StationSm::State::Connected, "the open link is up");
  check(probes_in(drain_tx()) == 1 && g_nudges.load() == 1,
        "...and one probe request is queued the moment it is");
  supervise(kNudgeAgainMs * 3);
  check(probes_in(drain_tx()) == 0, "...and no second one on an open link");

  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  Ap ap2;
  join_without_msg1(ap2, /*rsn=*/true);
  check(g_sm.state() == StationSm::State::FourWay,
        "WPA2: associated, waiting for message 1");
  check(probes_in(drain_tx()) == 1 && g_nudges.load() == 1,
        "...nudged the moment the Association Response was accepted");
  supervise(kNudgeAgainMs - 1);
  check(probes_in(drain_tx()) == 0, "...not again before kNudgeAgainMs");
  supervise(kNudgeAgainMs);
  check(probes_in(drain_tx()) == 1 && g_nudges.load() == 2,
        "...and once more when no EAPOL has come by then");
  supervise(kNudgeAgainMs * 3);
  check(probes_in(drain_tx()) == 0, "...and never a third time");

  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  Ap ap3;
  associate(ap3, /*rsn=*/true);   /* message 1 arrives at once */
  supervise(kNudgeAgainMs * 3);
  check(g_sm.keyed() && g_nudges.load() == 1,
        "WPA2 with message 1 at once: nudged once, no second nudge");
}

/* A NUDGE IS ONE FRAME AND NOTHING ELSE: no retune, no scan state, no
 * retune guard, and the AP's probe response it provokes neither moves the
 * BSS's channel nor changes the beacon-loss window - it only counts as AP
 * liveness. */
void test_a_nudge_touches_no_tuning_or_liveness_state() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  Ap ap;
  associate(ap, /*rsn=*/true);
  check(g_sm.keyed(), "the link is keyed");
  drain_tx();

  const uint8_t tuned = g_tuned.load();
  const uint32_t guard = g_retune_ms.load();
  const size_t scan_idx = g_scan_idx;
  const uint32_t scan_switch = g_scan_switch_ms;
  const uint32_t loss_ms = g_sm.beacon_loss_ms();
  {
    std::lock_guard<std::mutex> l(g_mu);
    nudge(100);
  }
  const std::vector<std::vector<uint8_t>> tx = drain_tx();
  check(tx.size() == 1 && probes_in(tx) == 1, "a nudge is exactly one probe request");
  check(g_tuned.load() == tuned && g_retune_ms.load() == guard,
        "...that touches neither the tuned channel nor the retune guard");
  check(g_scan_idx == scan_idx && g_scan_switch_ms == scan_switch,
        "...nor the scan sweep");

  /* The AP's answer: a probe response from the BSSID to this station. */
  std::vector<uint8_t> pr = beacon(true);
  pr[0] = devourer::sta::kFcProbeResp;
  std::memcpy(pr.data() + 4, kStaMac, 6);
  rx_frame(pr.data(), pr.size(), -40, 200);
  {
    std::lock_guard<std::mutex> l(g_mu);
    const devourer::sta::BssEntry* e = g_bss.find(kBssid);
    check(e && e->info.channel == 6, "the probe response leaves the BSS on its channel");
  }
  check(g_sm.beacon_loss_ms() == loss_ms, "...and the beacon-loss window unchanged");
  tick(200 + loss_ms - 1);
  check(g_sm.connected(), "...and counts as AP liveness, not against it");
}

/* Host frames for the confirmation cells: an ARP request (broadcast; sender
 * and target IPv4 10.0.0.<spa> / 10.0.0.<tpa>, spa 0 = a DAD probe, spa ==
 * tpa = gratuitous), a unicast IPv4 frame to kPeer carrying `proto` (ICMP: an
 * echo request; UDP: to `port`; TCP: flags `port & 0xff`), and an IPv6 frame
 * to `da` carrying ICMPv6 `type` (80 bytes, so the 40-byte header and the
 * ICMPv6 type are all there). */
void arp_request(uint8_t e[60], uint8_t spa, uint8_t tpa) {
  std::memset(e, 0, 60);
  std::memset(e, 0xff, 6);
  std::memcpy(e + 6, kStaMac, 6);
  e[12] = 0x08;
  e[13] = 0x06;
  uint8_t* a = e + 14;
  a[1] = 0x01; a[2] = 0x08; a[4] = 6; a[5] = 4;   /* Ethernet / IPv4 */
  a[7] = 0x01;                                     /* request */
  std::memcpy(a + 8, kStaMac, 6);
  if (spa) { a[14] = 10; a[17] = spa; }
  a[24] = 10; a[27] = tpa;
}

void ipv4_to_peer(uint8_t e[60], uint8_t proto, uint16_t port) {
  std::memset(e, 0, 60);
  std::memcpy(e, kPeer, 6);
  std::memcpy(e + 6, kStaMac, 6);
  e[12] = 0x08;
  e[13] = 0x00;
  uint8_t* ip = e + 14;
  ip[0] = 0x45;
  ip[9] = proto;
  uint8_t* l4 = ip + 20;
  if (proto == 1) l4[0] = 8;                       /* echo request */
  if (proto == 17) { l4[2] = (uint8_t)(port >> 8); l4[3] = (uint8_t)port; }
  if (proto == 6) l4[13] = (uint8_t)port;          /* TCP flags */
}

const uint8_t kAllRouters6[6] = {0x33, 0x33, 0, 0, 0, 2};
void icmpv6_to(uint8_t e[80], const uint8_t da[6], uint8_t type) {
  std::memset(e, 0, 80);
  std::memcpy(e, da, 6);
  std::memcpy(e + 6, kStaMac, 6);
  e[12] = 0x86;
  e[13] = 0xdd;
  uint8_t* ip = e + 14;
  ip[0] = 0x60;
  ip[5] = 26;                                      /* payload length */
  ip[6] = 58;                                      /* ICMPv6 */
  ip[7] = 64;
  ip[40] = type;
}

/* Whether one host frame counted as a question on a judged open link. */
bool is_question(const uint8_t* e, size_t len) {
  const uint32_t before = g_uplink_unconfirmed;
  tap_down_one(e, len);
  drain_tx();
  return g_uplink_unconfirmed == before + 1;
}

/* MULTICAST CHATTER IS NOT A QUESTION: frames that expect no reply (here
 * IPv6 echo requests to a multicast group) never count towards the
 * judgement. */
void test_multicast_chatter_is_not_judged() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure_open(kSsid, g_own);
  }
  Ap ap;
  associate(ap, /*rsn=*/false);
  drain_tx();
  for (int i = 0; i < 10; i++) {
    uint8_t e[80];
    icmpv6_to(e, kAllRouters6, 128);
    tap_down_one(e, sizeof e);
  }
  drain_tx();
  supervise(kConfirmMs * 4);
  supervise(kConfirmMs * 5 + 1);
  check(g_sm.state() == StationSm::State::Connected &&
            g_uplink_unconfirmed == 0,
        "multicast chatter with no reply is never judged");

  /* Gratuitous and probe ARPs are announcements, not questions. */
  for (int i = 0; i < 10; i++) {
    uint8_t e[60];
    arp_request(e, i % 2 ? 0 : 7, 7);
    tap_down_one(e, sizeof e);
  }
  drain_tx();
  supervise(kConfirmMs * 5 + 2);
  supervise(kConfirmMs * 6 + 3);
  check(g_sm.state() == StationSm::State::Connected &&
            g_uplink_unconfirmed == 0,
        "gratuitous and probe ARPs are never judged");

  /* ...but ARP requests are questions: an ARP that is never answered is how
   * an AP that dropped us shows - the first time on this BSS (the one
   * verdict on it fires at once; test_a_struck_bss_backs_off). */
  for (int i = 0; i < (int)kConfirmUplink; i++) {
    uint8_t e[60];
    arp_request(e, 2, 1);
    tap_down_one(e, sizeof e);
  }
  drain_tx();
  supervise(kConfirmMs * 7);
  supervise(kConfirmMs * 8);
  check(g_sm.state() == StationSm::State::Failed &&
            g_sm.fail_reason() == StationSm::Failure::Unconfirmed,
        "unanswered ARP requests are judged, the first time on a BSS");
}

/* AN OPEN ASSOCIATION THE AP DOES NOT HOLD IS FOUND AND RE-JOINED. The host
 * asks (kConfirmUplink questions) and no unicast reply comes back within
 * kConfirmMs: the link is lost as Unconfirmed and the ordinary re-join policy
 * takes over. The two controls: a unicast reply confirms the association for
 * good, and an idle host is never judged. */
void open_link_with_uplink(Ap& ap, int frames) {
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure_open(kSsid, g_own);
    g_rejoin_backoff_ms = 100;
  }
  associate(ap, /*rsn=*/false);
  drain_tx();
  uint8_t e[60];
  ipv4_to_peer(e, 1, 0);                           /* a ping */
  for (int i = 0; i < frames; i++) tap_down_one(e, sizeof e);
  drain_tx();
}

void test_an_unconfirmed_open_association_is_lost() {
  reset_all();
  Ap ap;
  open_link_with_uplink(ap, (int)kConfirmUplink);
  check(g_sm.state() == StationSm::State::Connected, "the open link is up");
  supervise(10);                       /* the window opens */
  supervise(10 + kConfirmMs - 1);
  check(g_sm.state() == StationSm::State::Connected,
        "...and not judged before kConfirmMs");
  supervise(10 + kConfirmMs);
  check(g_sm.state() == StationSm::State::Failed &&
            g_sm.fail_reason() == StationSm::Failure::Unconfirmed,
        "uplink with no unicast reply: lost as Unconfirmed");
  check(g_unconfirmed_lost.load() == 1 && g_reconnects.load() == 1,
        "...counted once, as a lost link");
  const uint64_t joins = g_joins.load();
  const std::vector<uint8_t> b = beacon(false);
  rx_frame(b.data(), b.size(), -40, kConfirmMs + 210);
  supervise(kConfirmMs + 210);
  check(g_joins.load() == joins + 1, "...and re-joined after the backoff");
}

/* FEWER THAN kConfirmUplink QUESTIONS ARE NOT JUDGED, however long they go
 * unanswered: one lost ping is not a lost association. */
void test_fewer_questions_than_the_threshold_are_not_judged() {
  for (int n = 1; n < (int)kConfirmUplink; n++) {
    reset_all();
    Ap ap;
    open_link_with_uplink(ap, n);
    supervise(10);
    supervise(10 + kConfirmMs * 4);
    check(g_sm.state() == StationSm::State::Connected &&
              g_uplink_unconfirmed == (uint32_t)n,
          n == 1 ? "one unanswered question is not judged"
                 : "two unanswered questions are not judged");
  }
}

/* THE WINDOW OPENS AT THE HOST'S FIRST QUESTION, not at the association: a
 * host idle for longer than kConfirmMs that then asks kConfirmUplink things
 * at once still gets kConfirmMs for the reply. */
void test_a_burst_after_idle_gets_its_window() {
  reset_all();
  Ap ap;
  open_link_with_uplink(ap, 0);
  supervise(1000);                     /* the main loop runs while idle */
  supervise(kConfirmMs * 4);           /* idle, long past kConfirmMs */
  uint8_t e[60];
  ipv4_to_peer(e, 6, 0x02);                        /* a TCP SYN */
  for (int i = 0; i < (int)kConfirmUplink; i++) tap_down_one(e, sizeof e);
  drain_tx();
  check(g_uplink_unconfirmed == kConfirmUplink, "the SYNs are questions");
  supervise(kConfirmMs * 4 + 1);
  supervise(kConfirmMs * 4 + 2);
  check(g_sm.state() == StationSm::State::Connected,
        "a burst after an idle spell is not judged at once");
  const uint8_t pl[16] = {1};
  const std::vector<uint8_t> d = ap.plain_to_sta(pl, sizeof pl);
  rx_frame(d.data(), d.size(), -40, kConfirmMs * 4 + 50);
  (void)tap_read();
  supervise(kConfirmMs * 6);
  check(g_sm.state() == StationSm::State::Connected,
        "...and the reply inside its window confirms it");
}

/* A unicast reply confirms the association: questions asked after it, left
 * unanswered across more than a whole window, are never judged. */
void test_a_unicast_reply_confirms_the_association() {
  reset_all();
  Ap ap;
  open_link_with_uplink(ap, 0);
  const uint8_t pl[16] = {1};
  const std::vector<uint8_t> d = ap.plain_to_sta(pl, sizeof pl);
  rx_frame(d.data(), d.size(), -40, 100);
  (void)tap_read();
  uint8_t e[60];
  ipv4_to_peer(e, 1, 0);                           /* real echo requests */
  for (int i = 0; i < 10; i++) tap_down_one(e, sizeof e);
  drain_tx();
  supervise(1000);
  supervise(1000 + kConfirmMs * 2);
  check(g_sm.state() == StationSm::State::Connected,
        "a unicast reply confirms the open association for good");
}

/* ONLY A UNICAST REPLY CONFIRMS: a group-addressed data frame from the AP
 * (a broadcast ARP, mDNS) reaches every station it ever held, and says
 * nothing about this association. */
void test_a_group_frame_from_the_ap_does_not_confirm() {
  reset_all();
  Ap ap;
  open_link_with_uplink(ap, 0);
  const uint8_t pl[16] = {1};
  const std::vector<uint8_t> d = ap.plain_to(kBcast, pl, sizeof pl);
  rx_frame(d.data(), d.size(), -40, 100);
  check(tap_read().size() == devourer::sta::kEthHdrLen + sizeof pl,
        "the AP's broadcast reaches the host");
  uint8_t e[60];
  ipv4_to_peer(e, 1, 0);
  for (int i = 0; i < (int)kConfirmUplink; i++) tap_down_one(e, sizeof e);
  drain_tx();
  supervise(1000);
  supervise(1000 + kConfirmMs);
  check(g_sm.state() == StationSm::State::Failed &&
            g_sm.fail_reason() == StationSm::Failure::Unconfirmed,
        "...but does not confirm the association");
}

/* WHAT A QUESTION IS (solicits_reply), through the real transmit path. */
void test_what_counts_as_a_question() {
  reset_all();
  Ap ap;
  open_link_with_uplink(ap, 0);
  uint8_t e4[60], e6[80];
  ipv4_to_peer(e4, 6, 0x02);
  check(is_question(e4, sizeof e4), "a TCP SYN is a question");
  ipv4_to_peer(e4, 6, 0x12);
  check(!is_question(e4, sizeof e4), "a SYN-ACK is not");
  ipv4_to_peer(e4, 6, 0x10);
  check(!is_question(e4, sizeof e4), "a bare TCP ACK is not");
  ipv4_to_peer(e4, 6, 0x18);
  check(!is_question(e4, sizeof e4), "TCP data (PSH-ACK) is not");
  ipv4_to_peer(e4, 6, 0x04);
  check(!is_question(e4, sizeof e4), "a TCP RST is not");
  ipv4_to_peer(e4, 1, 0);
  check(is_question(e4, sizeof e4), "an ICMP echo request is a question");
  ipv4_to_peer(e4, 1, 0);
  e4[14 + 20] = 0;
  check(!is_question(e4, sizeof e4), "an ICMP echo reply is not");
  e4[14 + 20] = 3;
  check(!is_question(e4, sizeof e4), "an ICMP destination unreachable is not");
  ipv4_to_peer(e4, 1, 0);
  e4[14 + 7] = 1;                                  /* fragment offset 8 */
  check(!is_question(e4, sizeof e4), "a non-first fragment is not");
  ipv4_to_peer(e4, 17, 53);
  check(is_question(e4, sizeof e4), "a DNS query is a question");
  ipv4_to_peer(e4, 17, 5600);
  check(!is_question(e4, sizeof e4), "other UDP is not");
  icmpv6_to(e6, kPeer, 128);
  check(is_question(e6, sizeof e6), "a unicast ICMPv6 echo request is a question");
  icmpv6_to(e6, kPeer, 135);
  check(is_question(e6, sizeof e6), "a unicast neighbour solicitation is a question");
  icmpv6_to(e6, kPeer, 129);
  check(!is_question(e6, sizeof e6), "an ICMPv6 echo reply is not");
  icmpv6_to(e6, kAllRouters6, 128);
  check(!is_question(e6, sizeof e6), "a multicast ICMPv6 echo request is not");
}

/* BACKOFF PER BSS. A host that keeps asking a peer that is switched off gets
 * no answer on a perfectly healthy link. The first verdict fires at once;
 * each re-joined association on the same BSS is then judged only after a
 * backoff that doubles per consecutive verdict, so the link is not torn down
 * every kConfirmMs - but an association the AP really dropped is still found.
 * A unicast reply resets it; a group frame does not. */
uint32_t rejoin_open(uint32_t now) {
  const std::vector<uint8_t> b = beacon(false);
  rx_frame(b.data(), b.size(), -40, now);
  supervise(now);
  Ap ap2;
  for (int round = 0; round < 8; round++) {
    pump_tx();
    const std::vector<std::vector<uint8_t>> tx = drain_tx();
    if (tx.empty()) break;
    for (const std::vector<uint8_t>& f : tx) {
      const std::vector<uint8_t> r = ap2.respond(f, false);
      if (!r.empty()) rx_frame(r.data(), r.size(), -40, now);
    }
  }
  drain_tx();
  return now;
}

/* A ping to the dead peer every second over [from, to], each followed by a
 * supervise pass. The time of the pass at which a verdict fired, or 0. */
uint32_t ping_until(uint32_t from, uint32_t to) {
  const uint64_t lost = g_unconfirmed_lost.load();
  uint8_t e[60];
  ipv4_to_peer(e, 1, 0);
  for (uint32_t t = from; t <= to; t += 1000) {
    tap_down_one(e, sizeof e);
    drain_tx();
    supervise(t);
    if (g_unconfirmed_lost.load() != lost) return t;
  }
  return 0;
}

/* A first verdict on kBssid, then the re-join. Returns its time. */
uint32_t struck_and_rejoined(Ap& ap) {
  open_link_with_uplink(ap, (int)kConfirmUplink);
  supervise(10);
  supervise(10 + kConfirmMs);
  check(g_unconfirmed_lost.load() == 1, "the first verdict on a BSS fires at once");
  const uint32_t t = rejoin_open(10 + kConfirmMs + 200);
  check(g_sm.state() == StationSm::State::Connected &&
            g_associations.load() == 2,
        "...and the station re-joins");
  return t;
}

void test_a_struck_bss_backs_off() {
  check(strike_backoff_ms(0) == 0 && strike_backoff_ms(1) == 2 * kConfirmMs &&
            strike_backoff_ms(2) == 4 * kConfirmMs &&
            strike_backoff_ms(5) == kStrikeCapMs &&
            strike_backoff_ms(40) == kStrikeCapMs,
        "the backoff doubles per consecutive verdict, capped at kStrikeCapMs");
  reset_all();
  Ap ap;
  uint32_t t = struck_and_rejoined(ap);

  const uint32_t b1 = t + strike_backoff_ms(1);
  check(ping_until(t, b1 - 1) == 0 && g_uplink_unconfirmed == 0,
        "inside the backoff the dead peer is not judged, nor its questions "
        "counted");
  uint32_t v = ping_until(b1, b1 + 2 * kConfirmMs);
  check(v >= b1 + kConfirmMs && v <= b1 + kConfirmMs + 1000,
        "after the backoff the association is judged again");

  t = rejoin_open(v + 200);
  const uint32_t b2 = t + strike_backoff_ms(2);
  check(ping_until(t, b2 - 1) == 0, "the second backoff is twice as long");
  v = ping_until(b2, b2 + 2 * kConfirmMs);
  check(v >= b2 + kConfirmMs && v <= b2 + kConfirmMs + 1000,
        "...and the association is judged after it");

  /* A unicast reply resets the backoff: the next association is judged at
   * once. */
  t = rejoin_open(v + 200);
  const uint8_t pl[16] = {1};
  const std::vector<uint8_t> d = ap.plain_to_sta(pl, sizeof pl);
  rx_frame(d.data(), d.size(), -40, t + 10);
  (void)tap_read();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.link_lost();
  }
  supervise(t + 20);
  t = rejoin_open(t + 300);
  v = ping_until(t, t + 2 * kConfirmMs);
  check(v >= t + kConfirmMs && v <= t + kConfirmMs + 1000,
        "a unicast reply resets the backoff: the next association is judged "
        "at once");
}

/* Only a unicast reply resets the backoff. A broadcast from the AP reaches
 * every station it ever held, and says nothing about this association: after
 * one, a link lost some other way and re-joined is still backed off. */
void test_a_group_frame_does_not_reset_the_backoff() {
  reset_all();
  Ap ap;
  uint32_t t = struck_and_rejoined(ap);
  const uint8_t pl[16] = {1};
  const std::vector<uint8_t> d = ap.plain_to(kBcast, pl, sizeof pl);
  rx_frame(d.data(), d.size(), -40, t + 10);
  (void)tap_read();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.link_lost();
  }
  supervise(t + 20);
  t = rejoin_open(t + 300);
  check(g_sm.state() == StationSm::State::Connected &&
            ping_until(t, t + strike_backoff_ms(1) - 1) == 0,
        "a broadcast from the AP does not reset the backoff");
}

/* THE BACKOFF IS PER BSS: one earned on another BSS does not spare this
 * one. The fixture has one BSS, so the strike is moved to another address
 * directly - what an earlier verdict on a different AP leaves behind. */
void test_the_strike_is_per_bss() {
  reset_all();
  Ap ap;
  open_link_with_uplink(ap, (int)kConfirmUplink);
  supervise(10);
  supervise(10 + kConfirmMs);
  {
    std::lock_guard<std::mutex> l(g_mu);
    std::memcpy(g_strike_bss, kPeer, 6);
  }
  const uint32_t now = rejoin_open(10 + kConfirmMs + 200);
  const uint32_t v = ping_until(now, now + 2 * kConfirmMs);
  check(v >= now + kConfirmMs && v <= now + kConfirmMs + 1000,
        "a backoff earned on another BSS does not spare this one");
}

/* A TCP segment too short to hold its flags byte is not a question - and the
 * flags byte is never read past the end. The frame sits in an allocation of
 * exactly its own length, so an ASan build catches a missing bound. */
void test_a_truncated_tcp_header_is_not_read() {
  std::vector<uint8_t> m;
  devourer::sta::append_llc_snap(m, 0x0800);
  uint8_t ip[20] = {0x45};
  ip[9] = 6;
  m.insert(m.end(), ip, ip + sizeof ip);
  m.insert(m.end(), 13, 0x02);          /* TCP, one byte short of the flags */
  std::unique_ptr<uint8_t[]> exact(new uint8_t[m.size()]);
  std::memcpy(exact.get(), m.data(), m.size());
  check(!solicits_reply(exact.get(), m.size(), kPeer),
        "a TCP segment too short to hold its flags is not a question");
}

/* THE at= STAMPS are what the on-air harness orders against hostapd -t: the
 * wall clock (CLOCK_REALTIME, within a minute of time()), seconds and exactly
 * six fractional digits, on the association line AND the verdict line. Read
 * back off stderr. */
bool stamp_ok(const std::string& line, const char* what) {
  const size_t p = line.find(" at=");
  if (p == std::string::npos) {
    std::fprintf(stdout, "  (%s: no at= in '%s')\n", what, line.c_str());
    return false;
  }
  const std::string t = line.substr(p + 4);
  const size_t dot = t.find('.');
  size_t end = dot == std::string::npos ? 0 : dot + 1;
  while (end < t.size() && t[end] >= '0' && t[end] <= '9') end++;
  const bool digits = dot != std::string::npos && dot > 0 &&
                      t.find_first_not_of("0123456789") == dot &&
                      end - dot - 1 == 6 && end == t.size();
  const long long sec = digits ? std::atoll(t.c_str()) : 0;
  const long long now = (long long)time(nullptr);
  /* Generous: the point is the wall clock rather than the monotonic one,
   * which is off by about the epoch (~1.7e9 s), not a tight bound that a
   * loaded or sanitised build could miss. */
  const bool wall = sec >= now - 60 && sec <= now + 60;
  if (!digits || !wall)
    std::fprintf(stdout, "  (%s: at=%s - %s)\n", what, t.c_str(),
                 !digits ? "not sec.usec with 6 digits" : "not the wall clock");
  return digits && wall;
}

void test_the_stamps_are_wall_clock_microseconds() {
  reset_all();
  FILE* cap = std::tmpfile();
  if (!cap) { check(false, "tmpfile for the stderr capture"); return; }
  std::fflush(stderr);
  const int saved = ::dup(2);
  if (saved < 0 || ::dup2(fileno(cap), 2) < 0) {
    if (saved >= 0) ::close(saved);
    std::fclose(cap);
    check(false, "redirect stderr for the capture");
    return;
  }
  Ap ap;
  open_link_with_uplink(ap, (int)kConfirmUplink);
  supervise(10);
  supervise(10 + kConfirmMs);
  std::fflush(stderr);
  const bool restored = ::dup2(saved, 2) >= 0;
  ::close(saved);
  check(restored, "stderr restored after the capture");
  std::rewind(cap);
  std::string assoc, verdict;
  char buf[512];
  while (std::fgets(buf, sizeof buf, cap)) {
    std::string l(buf);
    while (!l.empty() && (l.back() == '\n' || l.back() == '\r')) l.pop_back();
    if (l.find("station connected (association") != std::string::npos) assoc = l;
    if (l.find("station association unconfirmed:") != std::string::npos) verdict = l;
  }
  std::fclose(cap);
  check(!assoc.empty() && stamp_ok(assoc, "association"),
        "the association line carries at=<wall-clock sec.usec>");
  check(!verdict.empty() && stamp_ok(verdict, "verdict"),
        "the verdict line carries at=<wall-clock sec.usec>");
}

/* A QUESTION THE FULL QUEUE DROPPED IS NOT COUNTED - it never reached the
 * AP, so its missing answer says nothing - and neither is a nudge. */
void test_a_dropped_frame_is_not_counted() {
  reset_all();
  Ap ap;
  open_link_with_uplink(ap, 0);
  {
    std::lock_guard<std::mutex> q(g_q_mu);
    while (g_q.size() < 128) g_q.push_back(std::vector<uint8_t>(1, 0));
  }
  const uint64_t drops = g_q_drop.load();
  uint8_t e[60];
  ipv4_to_peer(e, 1, 0);
  tap_down_one(e, sizeof e);
  const uint64_t nudges = g_nudges.load(), probes = g_probe_tx.load();
  {
    std::lock_guard<std::mutex> l(g_mu);
    nudge(0);
  }
  check(g_q_drop.load() == drops + 2, "the full queue dropped both frames");
  check(g_uplink_unconfirmed == 0, "...a dropped question is not counted");
  check(g_nudges.load() == nudges && g_probe_tx.load() == probes,
        "...and neither is a dropped nudge");
  drain_tx();
}

/* ONE-WAY TRAFFIC IS NOT A QUESTION: a UDP uplink to a peer behind the AP
 * (video, telemetry) gets nothing unicast back, and must never be judged -
 * while the same link's first DNS query is. */
void test_a_one_way_udp_uplink_is_not_judged() {
  reset_all();
  Ap ap;
  open_link_with_uplink(ap, 0);
  uint8_t e[60];
  ipv4_to_peer(e, 17, 5600);
  for (uint32_t t = 0; t <= kConfirmMs * 4; t += 100) {
    tap_down_one(e, sizeof e);
    supervise(t);
  }
  drain_tx();
  check(g_sm.state() == StationSm::State::Connected,
        "a one-way UDP uplink is never judged unconfirmed");
  check(g_uplink_unconfirmed == 0, "...and none of it counts as a question");
  ipv4_to_peer(e, 17, 53);
  for (int i = 0; i < (int)kConfirmUplink; i++) tap_down_one(e, sizeof e);
  drain_tx();
  supervise(kConfirmMs * 5);
  supervise(kConfirmMs * 6);
  check(g_sm.state() == StationSm::State::Failed &&
            g_sm.fail_reason() == StationSm::Failure::Unconfirmed,
        "...but unanswered DNS queries are judged");
}

void test_an_idle_open_association_is_not_judged() {
  reset_all();
  Ap ap;
  open_link_with_uplink(ap, 0);
  supervise(kConfirmMs * 4);
  check(g_sm.state() == StationSm::State::Connected,
        "an idle host is never judged unconfirmed");
}

/* THE FCS IS TRIMMED ONLY WHERE IT IS PRESENT, and a runt shorter than its
 * FCS is length 0, not an unsigned wrap that every later bounds check would
 * pass. */
void test_the_fcs_is_trimmed_only_where_present() {
  check(mpdu_len(100, false) == 100, "no FCS: the buffer is the MPDU");
  check(mpdu_len(100, true) == 96, "an FCS: four bytes come off");
  check(mpdu_len(2, true) == 0, "a runt shorter than its FCS is empty");
}

/* A WPA2 link must not carry plaintext. The four-way's whole point is that
 * everything after it is protected; accepting an unprotected data frame lets
 * anyone on the channel inject straight into the host's stack. */
void test_plaintext_is_refused_on_a_protected_link() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  Ap ap;
  associate(ap, /*rsn=*/true);

  const uint8_t payload[16] = {0x42};
  const std::vector<uint8_t> d = ap.plain_to_sta(payload, sizeof payload);
  rx_frame(d.data(), d.size(), -40, 0);
  check(tap_read().empty(), "an unprotected data frame is refused");
  check(g_plain_rx.load() == 0, "...and is not counted as plaintext traffic");
  check(g_plain_refused.load() == 1, "...but is counted as refused plaintext");

  /* A QoS Null (26 bytes, no payload) is not plaintext data: not counted. */
  std::vector<uint8_t> qn = devourer::sta::data_hdr_from_ds(
      kStaMac, kBssid, kPeer, /*protect=*/false, ap.seq.next());
  qn[0] = 0xc8;                               /* QoS Null */
  qn.insert(qn.begin() + 24, {0x00, 0x00});   /* QoS Control, TID 0 */
  check(qn.size() == 26, "the QoS Null fixture is 26 bytes");
  rx_frame(qn.data(), qn.size(), -40, 0);
  check(g_plain_refused.load() == 1, "...and a QoS Null is not counted");
}

/* Fragments and A-MSDUs are refused rather than misread. Handing half an MSDU
 * to the host as a whole one produces a frame that looks well-formed and
 * decodes to nonsense. */
void test_fragments_and_amsdu_are_refused() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  Ap ap;
  associate(ap, /*rsn=*/true);

  const uint8_t payload[16] = {5};
  std::vector<uint8_t> d = ap.data_to_sta(payload, sizeof payload);
  d[1] |= devourer::sta::kFcMoreFrag;
  rx_frame(d.data(), d.size(), -40, 0);
  check(g_frag_drop.load() == 1, "a More Fragments frame is refused");

  /* THE LAST FRAGMENT HAS MoreFrag CLEAR. A refusal that only checked that
   * bit would forward it as a whole frame. */
  d = ap.data_to_sta(payload, sizeof payload);
  d[22] = 0x03;                           /* fragment number 3, MoreFrag clear */
  rx_frame(d.data(), d.size(), -40, 0);
  check(g_frag_drop.load() == 2, "...and so is a LAST fragment");
  check(tap_read().empty(), "neither reaches the host");

  /* AND THE A-MSDU. This cell was named for both and tested only one - the
   * A-MSDU bit lives in the QoS Control field, so a non-QoS frame cannot
   * carry it and no arm here reached that branch at all. An A-MSDU is a
   * whole list of subframes; handing one to msdu_to_eth reads the first
   * subframe's 14-byte header as an LLC/SNAP and produces garbage. */
  const uint8_t big[32] = {6};
  const std::vector<uint8_t> a = ap.qos_to_sta(big, sizeof big, 0, true);
  rx_frame(a.data(), a.size(), -40, 0);
  check(g_amsdu_drop.load() == 1, "an A-MSDU is refused");
  check(tap_read().empty(), "...and does not reach the host");

  /* The control: the SAME frame without the A-MSDU bit goes through, so the
   * arm above cannot be passing because QoS frames are refused wholesale. */
  const std::vector<uint8_t> q = ap.qos_to_sta(big, sizeof big, 0, false);
  rx_frame(q.data(), q.size(), -40, 0);
  check(g_amsdu_drop.load() == 1, "...while a plain QoS frame is not");
  check(tap_read().size() == devourer::sta::kEthHdrLen + sizeof big,
        "...and reaches the host");
}

/* THE REPLAY WINDOW IS INDEXED BY THE REAL TID, AND THAT IS A SECURITY RULE.
 *
 * The QoS Control field is at a fixed offset and HT Control follows it, so
 * "header length minus two" lands inside HT Control on any +HTC frame. HT
 * Control is NOT covered by the CCMP AAD, so an attacker can rewrite it on a
 * CAPTURED frame without breaking its MIC - and a receiver that takes the
 * TID from there files the replay under a different per-TID window from the
 * original, where it is accepted. That is a replay-protection bypass, not an
 * accounting error.
 */
void test_the_tid_comes_from_the_qos_control_field() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  Ap ap;
  associate(ap, /*rsn=*/true);

  const uint8_t payload[16] = {0x2c};
  const std::vector<uint8_t> f = ap.qos_htc_to_sta(
      payload, sizeof payload, /*tid=*/2, /*htc=*/0x00000005, /*pn=*/7);
  rx_frame(f.data(), f.size(), -40, 0);
  check(tap_read().size() == devourer::sta::kEthHdrLen + sizeof payload,
        "a +HTC QoS frame decrypts and reaches the host");
  check(g_rx_replay.last(2) == 7, "...and its PN landed in TID 2's window");
  check(g_rx_replay.last(5) == 0, "...NOT in the window HT Control names");

  /* THE ATTACK, end to end. The same MPDU with HT Control rewritten - which
   * the MIC does not cover - replayed at the same PN. A receiver reading the
   * TID from HT Control files it under TID 6, which has never seen PN 7, and
   * accepts it. */
  std::vector<uint8_t> replayed = ap.qos_htc_to_sta(
      payload, sizeof payload, /*tid=*/2, /*htc=*/0x00000005, /*pn=*/7);
  replayed[26] = 0x06;                    /* HT Control, byte 0 */
  rx_frame(replayed.data(), replayed.size(), -40, 0);
  check(tap_read().empty(), "the replay is refused despite the rewritten HTC");
  check(g_replays.load() == 1, "...as a replay");
}

/* A PTK REKEY, which happens with the association already up and the machine
 * already Connected - so nothing hung off the "we just connected" transition
 * sees it. Both PN spaces restart with the new key; a receive window left at
 * the old key's head rejects everything until the AP's PN climbs back within
 * 64 of it, which is a link that reports itself keyed and carries nothing. */
void test_ptk_rekey() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  Ap ap;
  associate(ap, /*rsn=*/true);
  check(g_ptk_installs.load() == 1, "one pairwise key so far");

  /* Carry enough traffic that a window which is NOT reset would reject the
   * new key's PN 1 as ancient - the whole failure mode. */
  const uint8_t pl[8] = {1};
  for (int i = 0; i < 70; i++) {
    const std::vector<uint8_t> d = ap.data_to_sta(pl, sizeof pl);
    rx_frame(d.data(), d.size(), -40, 0);
    (void)tap_read();
  }
  check(g_rx_replay.last() > 64, "the receive window is past a whole window");

  uint8_t old_ptk[48];
  std::memcpy(old_ptk, ap.ptk, sizeof old_ptk);

  /* Message 1 of the rekey, inside the cipher, under the CURRENT key. */
  const std::vector<uint8_t> m1 = ap.ptk_rekey_msg1();
  rx_frame(m1.data(), m1.size(), -40, 0);
  std::vector<std::vector<uint8_t>> tx = drain_tx();
  check(tx.size() == 1, "message 2 goes out");
  if (tx.size() == 1) {
    /* UNDER THE OLD KEY. The authenticator has not switched its own yet. */
    uint8_t plain[256];
    size_t plen = 0;
    uint64_t pn = 0;
    check(devourer::sta::ccmp_decrypt(ap.crypto, old_ptk + 32, tx[0].data(),
                                      tx[0].size(), 24, tx[0].data() + 10,
                                      plain, sizeof plain, &plen, &pn),
          "...encrypted under the key the rekey arrived under");
    devourer::sta::EapolKey k;
    if (devourer::sta::parse_eapol_key(plain + devourer::sta::kLlcSnapLen,
                                       plen - devourer::sta::kLlcSnapLen, &k))
      check(devourer::sta::derive_ptk(ap.crypto, ap.pmk, kBssid, kStaMac,
                                      ap.anonce, k.nonce, ap.ptk),
            "...and the AP derives the new PTK from it");
  }

  /* Message 3 completes it, still under the OLD key - it carries the NEW
   * key's material and is protected by the one still in force. */
  const std::vector<uint8_t> m3 = ap.eapol_protected(ap.msg3(), old_ptk + 32);
  rx_frame(m3.data(), m3.size(), -40, 0);
  check(g_ptk_installs.load() == 2, "the new pairwise key is installed");
  check(std::memcmp(g_sm.supplicant().ptk(), ap.ptk, 48) == 0,
        "...and both sides have the same one");
  tx = drain_tx();
  check(tx.size() == 1, "message 4 goes out");
  if (tx.size() == 1) {
    uint8_t plain[256];
    size_t plen = 0;
    uint64_t pn = 0;
    /* THE SAME RULE, and the one that matters most: an authenticator that
     * gets a message 4 it cannot decrypt fails the rekey and deauthenticates
     * the station. */
    check(devourer::sta::ccmp_decrypt(ap.crypto, old_ptk + 32, tx[0].data(),
                                      tx[0].size(), 24, tx[0].data() + 10,
                                      plain, sizeof plain, &plen, &pn),
          "...also under the OLD key, which is all the AP can read");
  }

  check(g_tx_pn == 1, "the transmit PN restarted with the new key");
  check(g_rx_replay.last() == 0, "...and so did the receive window");

  /* And the AP's first frame under the new key, at PN 1, gets through. */
  ap.tx_pn = 1;
  const std::vector<uint8_t> d = ap.data_to_sta(pl, sizeof pl);
  rx_frame(d.data(), d.size(), -40, 0);
  check(tap_read().size() == devourer::sta::kEthHdrLen + sizeof pl,
        "the first frame under the new key reaches the host");
  check(g_replays.load() == 0, "...and is not rejected as a replay");
}

/* A group-addressed EAPOL-Key frame. It is not part of any handshake this
 * station is in, and it is emphatically not the host's: writing an 0x888e
 * frame onto the TAP hands the host stack a link-layer control frame the
 * supplicant owns. */
void test_group_addressed_eapol_is_not_the_hosts() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  Ap ap;
  associate(ap, /*rsn=*/true);
  drain_tx();

  const uint64_t rx_before = g_eapol_enc_rx.load();
  const std::vector<uint8_t> g = ap.group_eapol(ap.msg1());
  rx_frame(g.data(), g.size(), -40, 0);
  check(tap_read().empty(), "a group-addressed EAPOL-Key is not given to the host");
  check(g_eapol_enc_rx.load() == rx_before,
        "...and is not fed to the state machine either");
  check(drain_tx().empty(), "...and is not answered");

  /* The control: a group frame that is NOT EAPOL still reaches the host, so
   * the check above is about the ethertype and not about group frames. */
  const uint8_t arp[28] = {0, 1};
  const std::vector<uint8_t> ok_frame = ap.group_to_bss(arp, sizeof arp);
  rx_frame(ok_frame.data(), ok_frame.size(), -40, 0);
  check(tap_read().size() == devourer::sta::kEthHdrLen + sizeof arp,
        "...while an ordinary group frame still does");
}

/* The multi-channel sweep. The docs claimed a cell covered this and none
 * did: reset_all() configures exactly one channel, and scan_step() returns
 * early for a single-entry list, so the rotation was unreached by the whole
 * file. */
void test_the_channel_sweep_rotates() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
    g_scan_chans.clear();
    g_scan_chans.push_back(1);
    g_scan_chans.push_back(6);
    g_scan_chans.push_back(11);
    g_scan_dwell_ms = 100;
    g_scan_switch_ms = 0;
    g_scan_idx = 0;
  }
  /* No BSS in the table, so supervise() stays in the scan and returns the
   * channel it wants the radio on. */
  check(supervise(0) == 1, "the sweep starts on the first channel");
  check(supervise(50) == 1, "...and holds it for the dwell");
  check(supervise(150) == 6, "...then moves on");
  check(supervise(300) == 11, "...and on");
  check(supervise(450) == 1, "...and wraps");

  /* AND IT STOPS SWEEPING ONCE THERE IS SOMETHING TO JOIN. Retuning under a
   * live association is how a station loses one. */
  const std::vector<uint8_t> b = beacon(true, 6);
  rx_frame(b.data(), b.size(), -40, 500);
  check(supervise(600) == 6, "the joined BSS's channel is what it asks for");
  check(g_joins.load() == 1, "...because it joined");
  check(supervise(5000) == 6, "...and it does not sweep away from it");
}

/* A GROUP REKEY installs a new GTK under the same association, and its PN
 * space restarts with the key. A station that kept the old window goes deaf
 * to broadcast for a whole window - every ARP the AP relays - so the link
 * looks up and carries nothing. */
void test_group_rekey() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  Ap ap;
  associate(ap, /*rsn=*/true);

  const uint8_t arp[28] = {0, 1};
  /* Push the group PN well up, so a window that is NOT reset would reject
   * the new key's PN 1 as ancient. */
  for (int i = 0; i < 5; i++) {
    const std::vector<uint8_t> g = ap.group_to_bss(arp, sizeof arp);
    rx_frame(g.data(), g.size(), -40, 0);
    (void)tap_read();
  }
  check(g_group_rx.load() == 5, "five group frames under the first GTK");

  /* THE REKEY ARRIVES INSIDE THE CIPHER, because that is where a real AP
   * puts it - it runs after the PTK is installed. Sending it in cleartext
   * here, which the first version of this cell did, tests the four-way's
   * path and leaves the one that matters unreached: against hostapd all
   * four of its message 1s were counted as rx_ignored and the AP threw the
   * station off the BSS ("group key handshake failed (RSN) after 4 tries").
   * A green cell and a dead link. */
  uint8_t gtk2[16];
  std::memset(gtk2, 0xa7, sizeof gtk2);
  const std::vector<uint8_t> rk =
      ap.eapol_protected(ap.group_msg1(gtk2, 2));
  rx_frame(rk.data(), rk.size(), -40, 0);
  check(g_eapol_enc_rx.load() == 1,
        "the protected rekey is recognised as EAPOL, not handed to the host");
  check(tap_read().empty(), "...so nothing of it reaches the host");
  check(g_sm.supplicant().gtk_valid() &&
            std::memcmp(g_sm.supplicant().gtk(), gtk2, 16) == 0,
        "the new GTK is installed");
  check(g_sm.supplicant().gtk_key_id() == 2, "...at its new key id");
  check(g_gtk_installs.load() == 2, "...and counted as a SECOND group key");

  /* AND THE ANSWER IS ENCRYPTED TOO. An AP that gets a cleartext message 2
   * to a protected message 1 drops it, which is the same failure with an
   * extra round trip. */
  check(g_eapol_enc_tx.load() == 1, "the rekey is answered");
  const std::vector<std::vector<uint8_t>> ans = drain_tx();
  check(ans.size() == 1, "...with exactly one frame");
  if (ans.size() == 1) {
    check((ans[0][1] & devourer::sta::kFcProtected) != 0,
          "...and it is PROTECTED");
    uint8_t plain[256];
    size_t plain_len = 0;
    uint64_t pn = 0;
    const bool ok = devourer::sta::ccmp_decrypt(
        ap.crypto, ap.ptk + 32, ans[0].data(), ans[0].size(), 24,
        ans[0].data() + 10, plain, sizeof plain, &plain_len, &pn);
    check(ok, "...the AP decrypts it");
    if (ok) {
      check(plain_len > devourer::sta::kLlcSnapLen &&
                plain[6] == 0x88 && plain[7] == 0x8e,
            "...and it is an EAPOL-Key frame");
      devourer::sta::EapolKey k;
      check(devourer::sta::parse_eapol_key(
                plain + devourer::sta::kLlcSnapLen,
                plain_len - devourer::sta::kLlcSnapLen, &k) &&
                devourer::sta::eapol_mic_ok(ap.crypto, ap.ptk, k) ==
                    devourer::sta::MicCheck::Ok,
            "...whose MIC the AP accepts");
    }
  }

  const std::vector<uint8_t> g = ap.group_to_bss(arp, sizeof arp);
  rx_frame(g.data(), g.size(), -40, 0);
  check(tap_read().size() == devourer::sta::kEthHdrLen + sizeof arp,
        "a frame under the NEW GTK, at PN 1, reaches the host");
  check(g_replays.load() == 0, "...and is not rejected as a replay");
}

/* KRACK, END TO END THROUGH THIS HARNESS (CVE-2017-13077/13078). Our message
 * 4 is lost, so the AP retransmits message 3 - in the clear, it has not
 * installed its key yet - at a GREATER replay counter, as hostapd does. The
 * station must answer it and must NOT restart its transmit PN, its receive
 * windows or its group window: every one of those restarts under an
 * unchanged key is nonce reuse or a reopened replay window. And a new GTK's
 * window starts at the Key RSC the AP quoted, not at whatever arrives. */
void test_msg3_retransmit_keeps_the_pn() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  Ap ap;
  associate(ap, /*rsn=*/true);

  uint8_t e[60] = {0};
  std::memcpy(e, kPeer, 6);
  std::memcpy(e + 6, kStaMac, 6);
  e[12] = 0x08;
  for (int i = 0; i < 3; i++) tap_down_one(e, sizeof e);
  (void)drain_tx();
  const uint8_t arp[28] = {0, 1};
  const std::vector<uint8_t> g1 = ap.group_to_bss(arp, sizeof arp);
  rx_frame(g1.data(), g1.size(), -40, 0);
  (void)tap_read();
  const uint64_t pn_before = g_tx_pn;
  check(pn_before > 1, "the transmit PN has moved past 1");

  const std::vector<uint8_t> m3 = ap.eapol_frame(ap.msg3());
  rx_frame(m3.data(), m3.size(), -40, 0);
  pump_tx();
  check(drain_tx().size() == 1, "the retransmitted message 3 is answered");
  check(g_ptk_installs.load() == 1 && g_gtk_installs.load() == 1,
        "...AND NOTHING IS REINSTALLED");
  check(g_tx_pn == pn_before, "...THE TRANSMIT PN IS NOT RESET");
  rx_frame(g1.data(), g1.size(), -40, 0);
  check(tap_read().empty() && g_replays.load() == 1,
        "...and the group window was not reopened: a replayed group frame "
        "is still refused");

  /* A real group rekey, quoting RSC 100: its window starts THERE. */
  uint8_t gtk2[16];
  const uint8_t rsc[8] = {100, 0, 0, 0, 0, 0, 0, 0};
  std::memset(gtk2, 0xa7, sizeof gtk2);
  const std::vector<uint8_t> rk =
      ap.eapol_protected(ap.group_msg1(gtk2, 2, rsc));
  rx_frame(rk.data(), rk.size(), -40, 0);
  (void)drain_tx();
  check(g_gtk_installs.load() == 2, "a new GTK is installed");
  ap.group_pn = 50;
  const std::vector<uint8_t> old = ap.group_to_bss(arp, sizeof arp);
  rx_frame(old.data(), old.size(), -40, 0);
  check(tap_read().empty(), "a group frame BELOW the Key RSC is refused");
  ap.group_pn = 101;
  const std::vector<uint8_t> fresh = ap.group_to_bss(arp, sizeof arp);
  rx_frame(fresh.data(), fresh.size(), -40, 0);
  check(tap_read().size() == devourer::sta::kEthHdrLen + sizeof arp,
        "...and one above it gets through");
}

/* A BSS heard once and never again must not be joined minutes later. A
 * station that keeps them forever reports "no response" as though the AP were
 * broken, when what it actually did was try to talk to one that left. */
void test_a_stale_bss_is_not_joined() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  const std::vector<uint8_t> b = beacon(true);
  rx_frame(b.data(), b.size(), -40, 0);
  check(g_bss.count() == 1, "the BSS is in the table");

  supervise(20000);                      /* twenty seconds later */
  check(g_joins.load() == 0, "a BSS unheard for 20 s is not joined");
  check(g_bss.count() == 0, "...it was expired out of the table");
  check(g_probe_tx.load() > 0, "...and a probe went out instead");

  /* The control: the same table, aged only two seconds, IS joined. Without
   * it this cell passes for a supervisor that never joins anything. */
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  rx_frame(b.data(), b.size(), -40, 0);
  supervise(2000);
  check(g_joins.load() == 1, "a BSS heard 2 s ago IS joined");
}

/* The RSSI conversion, which nothing else reaches - it is called only from
 * on_rx(), which needs a Packet and therefore a radio. Only the ORDERING
 * matters to BssTable, but a raw 0 (the PHY reported nothing) must not
 * outrank a real reading, and that is a value and not an order. */
void test_rssi_conversion() {
  check(rssi_dbm(0) == -128, "a raw 0 is the floor, not -110 dBm");
  check(rssi_dbm(70) == -40, "raw 70 is -40 dBm");
  check(rssi_dbm(90) == -20, "raw 90 is -20 dBm");
  check(rssi_dbm(90) > rssi_dbm(70), "and a stronger raw reading ranks higher");
  check(rssi_dbm(1) > rssi_dbm(0),
        "...while any real reading beats 'nothing reported'");
}

/* THE RECONNECT POLICY. StationSm notices beacon loss and fails, and does
 * not re-join; supervise() is the harness's answer. */
void test_reconnect_after_beacon_loss() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
    g_rejoin_backoff_ms = 100;
  }
  Ap ap;
  associate(ap, /*rsn=*/true);
  check(g_sm.state() == StationSm::State::Connected, "connected first");
  const uint32_t joins_before = (uint32_t)g_joins.load();

  /* Carry real traffic first, so both PN spaces are well past 1 when the
   * link drops. Without this the "it restarted" checks below are true
   * whether or not anything resets them. */
  for (int i = 0; i < 4; i++) {
    const uint8_t pl[8] = {(uint8_t)i};
    const std::vector<uint8_t> d = ap.data_to_sta(pl, sizeof pl);
    rx_frame(d.data(), d.size(), -40, 0);
    (void)tap_read();
    uint8_t e[60] = {0};
    std::memcpy(e, kPeer, 6);
    std::memcpy(e + 6, kStaMac, 6);
    e[12] = 0x08;
    e[13] = 0x00;
    tap_down_one(e, sizeof e);
  }
  drain_tx();
  check(g_tx_pn > 1 && g_rx_replay.last() > 1,
        "both PN spaces have advanced");

  /* Nothing is fed for longer than kBeaconLossMs, which is what an AP that
   * was switched off looks like. */
  uint32_t now = StationSm::kBeaconLossMs + 1;
  tick(now);
  check(g_sm.state() == StationSm::State::Failed, "beacon loss fails the link");
  check(g_sm.fail_reason() == StationSm::Failure::BeaconLost, "...and says so");

  /* The backoff has to actually hold: a supervisor that re-joins on the same
   * millisecond spins against an AP that is off. */
  supervise(now);
  check(g_joins.load() == joins_before, "no re-join before the backoff");
  check(g_reconnects.load() == 1, "the loss is counted as a reconnect attempt");

  /* The AP comes back. The table still holds it, so a fresh beacon is not
   * strictly needed - but feeding one is what actually happens, and it also
   * proves expire() has not thrown the entry away. */
  now += 200;
  const std::vector<uint8_t> b = beacon(true);
  rx_frame(b.data(), b.size(), -40, now);
  supervise(now);
  check(g_joins.load() == joins_before + 1, "...and a re-join after it");
  check(g_sm.state() == StationSm::State::Authenticating,
        "the machine is authenticating again");

  Ap ap2;
  for (int round = 0; round < 8; round++) {
    pump_tx();
    const std::vector<std::vector<uint8_t>> tx = drain_tx();
    if (tx.empty()) break;
    for (const std::vector<uint8_t>& f : tx) {
      const std::vector<uint8_t> r = ap2.respond(f, true);
      if (!r.empty()) rx_frame(r.data(), r.size(), -40, now);
      if (f[0] == devourer::sta::kFcAssocReq) {
        const std::vector<uint8_t> m1 = ap2.eapol_frame(ap2.msg1());
        rx_frame(m1.data(), m1.size(), -40, now);
      }
    }
  }
  check(g_sm.state() == StationSm::State::Connected, "the link comes back");
  check(g_associations.load() == 2, "two associations, not one");
  /* A NEW PTK MEANS A NEW PN SPACE. Carrying the transmit PN across an
   * association is keystream reuse, not a replay problem. */
  check(g_tx_pn == 1, "the transmit PN restarted with the new key");

  /* AND THE RECEIVE WINDOW RESTARTED TOO. The new AP's PN space also begins
   * at 1, so a station holding the old window rejects the first frames of
   * the new association as ancient replays - a link that associates, reports
   * itself keyed, and carries nothing. */
  const uint8_t payload[16] = {3};
  const std::vector<uint8_t> d = ap2.data_to_sta(payload, sizeof payload);
  rx_frame(d.data(), d.size(), -40, now);
  check(tap_read().size() == devourer::sta::kEthHdrLen + sizeof payload,
        "the first frame of the new association reaches the host");
  check(g_replays.load() == 0, "...and is not rejected as a replay");
}

/* ONE LOST LINK IS ONE RECONNECT, however many times the retry fails.
 *
 * The first latch has to be cleared by every join attempt, or a SECOND loss
 * could never be noticed - so on its own it counts attempts, not losses: an
 * AP that is away for several backoff periods would read as several lost
 * links. */
void test_a_lost_link_is_counted_once() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
    g_rejoin_backoff_ms = 100;
  }
  Ap ap;
  associate(ap, /*rsn=*/true);
  check(g_reconnects.load() == 0, "no reconnect while the link is up");

  uint32_t now = StationSm::kBeaconLossMs + 1;
  tick(now);
  check(g_sm.state() == StationSm::State::Failed, "the link drops");

  /* The AP stays OFF. Every re-join therefore authenticates into silence and
   * times out, which is exactly the on-air shape. */
  for (int i = 0; i < 6; i++) {
    supervise(now);
    pump_tx();
    drain_tx();                            /* nothing answers */
    now += StationSm::kMgmtTimeoutMs * (StationSm::kMaxTries + 1);
    tick(now);
    now += 200;
  }
  check(g_joins.load() > 1, "it kept trying");
  check(g_reconnects.load() == 1,
        "...and the lost link is still counted exactly once");
}

/* With reconnect off, the harness gives up - and says so by not joining
 * again. The control arm for the cell above: without it, "it re-joined"
 * could just be what this code always does. */
void test_reconnect_can_be_disabled() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
    g_reconnect = false;
    g_rejoin_backoff_ms = 100;
  }
  Ap ap;
  associate(ap, /*rsn=*/true);
  const uint32_t joins_before = (uint32_t)g_joins.load();
  check(joins_before == 1, "one join so far");

  uint32_t now = StationSm::kBeaconLossMs + 1;
  tick(now);
  check(g_sm.state() == StationSm::State::Failed, "the link fails");
  for (int i = 0; i < 5; i++) {
    now += 500;
    const std::vector<uint8_t> b = beacon(true);
    rx_frame(b.data(), b.size(), -40, now);
    supervise(now);
  }
  check(g_joins.load() == joins_before, "no re-join with reconnect disabled");

  /* And the ledger says so: the teardown's leave() returns the machine to
   * Idle, but the run ENDED Failed, and why. */
  {
    std::lock_guard<std::mutex> l(g_mu);
    take_run_end();
    g_sm.leave();
  }
  check(g_sm.state() == StationSm::State::Idle, "leave() returns to Idle");
  check(g_end.state == StationSm::State::Failed &&
            g_end.reason == StationSm::Failure::BeaconLost,
        "...but the run's end state is Failed, reason beacon-lost");
}

/* A frame from the host claiming somebody else's source address. A real AP
 * refuses it and one that does not is an injection tool; either way airing it
 * is wrong, and a TAP whose MAC was never set is the ordinary cause. */
void test_tap_refuses_a_foreign_source() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  Ap ap;
  associate(ap, /*rsn=*/true);
  drain_tx();

  uint8_t e[60] = {0};
  std::memcpy(e, kPeer, 6);
  std::memcpy(e + 6, kBssid, 6);          /* not us */
  e[12] = 0x08;
  e[13] = 0x00;
  const uint64_t dropped = g_tap_down_drop.load();
  tap_down_one(e, sizeof e);
  check(drain_tx().empty(), "a frame with a foreign source is not aired");
  check(g_tap_down_drop.load() == dropped + 1, "...and is counted as a drop");

  /* The control: the same frame with OUR address goes out. Without this the
   * cell passes for a tap_down_one() that airs nothing at all. */
  std::memcpy(e + 6, kStaMac, 6);
  tap_down_one(e, sizeof e);
  check(drain_tx().size() == 1, "the same frame from us IS aired");
}

/* Nothing is transmitted before the link is up. A data plane that airs
 * frames while unassociated leaks the host's traffic onto the channel in
 * plaintext, addressed to a BSSID we have not joined. */
void test_no_data_before_association() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  uint8_t e[60] = {0};
  std::memcpy(e, kPeer, 6);
  std::memcpy(e + 6, kStaMac, 6);
  e[12] = 0x08;
  e[13] = 0x00;
  tap_down_one(e, sizeof e);
  check(drain_tx().empty(), "nothing is aired before the association");
  check(g_tap_down_drop.load() == 1, "...and the frame is counted as dropped");

  /* And a protected frame that arrives BEFORE the four-way finishes must be
   * refused by the "are we connected" gate specifically.
   *
   * IT HAS TO REACH THAT GATE TO TEST IT. Fed while the machine is Idle,
   * bssid_ is all zeros and the address check refuses the frame instead.
   * Driving authentication and association but NOT the key exchange puts the
   * machine in FourWay with the right BSSID, where the gate is the only
   * thing left. */
  Ap ap;
  const std::vector<uint8_t> b = beacon(true);
  rx_frame(b.data(), b.size(), -40, 0);
  supervise(0);
  for (int round = 0; round < 6; round++) {
    pump_tx();
    const std::vector<std::vector<uint8_t>> tx = drain_tx();
    if (tx.empty()) break;
    for (const std::vector<uint8_t>& f : tx) {
      const std::vector<uint8_t> r = ap.respond(f, true);
      if (!r.empty()) rx_frame(r.data(), r.size(), -40, 0);
    }
  }
  check(g_sm.state() == StationSm::State::FourWay,
        "associated, with the key exchange still outstanding");
  check(std::memcmp(g_sm.bssid(), kBssid, 6) == 0,
        "...so the BSSID check can no longer be what refuses the frame");

  /* The AP's PTK is whatever derive_ptk left it - all zeros, since message 2
   * never arrived - which is emphatically not a key this station holds. */
  const uint8_t payload[16] = {1};
  std::memset(ap.ptk, 0xab, sizeof ap.ptk);
  const std::vector<uint8_t> d = ap.data_to_sta(payload, sizeof payload);
  rx_frame(d.data(), d.size(), -40, 0);
  check(g_enc_rx.load() == 0, "a protected frame before the key is not counted");
  check(g_mic_fail.load() == 0, "...not even as a MIC failure - it is refused");
  check(tap_read().empty(), "...and does not reach the host");
}

/* The scan picks the network by NAME, out of a channel carrying others - and
 * a neighbour airing the same SSID with a cipher we cannot speak is not a
 * candidate. */
void test_scan_selects_the_wanted_network() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  const uint8_t other[6] = {0x02, 0xaa, 0xbb, 0xcc, 0xdd, 0xee};
  const std::vector<uint8_t> nb = beacon(true, 6, other, "somebody-else");
  const std::vector<uint8_t> open_same = beacon(false, 6, other, kSsid);
  rx_frame(nb.data(), nb.size(), -20, 0);          /* stronger */
  rx_frame(open_same.data(), open_same.size(), -20, 0);
  check(g_bss.count() >= 1, "the neighbour is in the table");
  supervise(0);
  check(g_joins.load() == 0, "nothing joinable yet, so no join");
  check(g_probe_tx.load() > 0, "...but a directed probe went out");

  const std::vector<uint8_t> ours = beacon(true);
  rx_frame(ours.data(), ours.size(), -70, 1000);   /* weaker, and ours */
  supervise(1000);
  check(g_joins.load() == 1, "our network is joined once it appears");
  check(std::memcmp(g_sm.bssid(), kBssid, 6) == 0,
        "...and it is OUR BSSID, not the stronger neighbour's");
}

/* A short frame, a runt, and a frame that is not ours. The receive path is
 * promiscuous on this hardware, so these are the ordinary case and not the
 * edge case. */
void test_hostile_and_foreign_frames() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  Ap ap;
  associate(ap, /*rsn=*/true);
  const uint32_t not_ours_before = g_sm.rx_not_our_bss;

  uint8_t runt[23] = {0};
  rx_frame(runt, sizeof runt, -40, 0);
  check(g_rx_short.load() >= 1, "a frame shorter than a header is refused");

  /* A PROTECTED FRAME WITH NOTHING AFTER THE HEADER. A key-id read before
   * the CCMP header is proved present reads three bytes past the end of the
   * buffer - on MT7612U past the allocation, because this part strips the
   * FCS. Any station on the channel can air it.
   *
   * THIS ARM IS ONLY MEANINGFUL UNDER THE SANITIZER BUILD. In a normal build
   * an overread succeeds and the counters come out the same; run it under
   * -DDEVOURER_SANITIZE=address+undefined. */
  const uint64_t mic_before = g_mic_fail.load();
  const uint64_t short_before = g_rx_short.load();
  std::vector<uint8_t> stub = devourer::sta::data_hdr_from_ds(
      kStaMac, kBssid, kPeer, /*protect=*/true, 9);
  rx_frame(stub.data(), stub.size(), -40, 0);
  check(g_rx_short.load() == short_before + 1,
        "a protected frame with no CCMP header is short");
  check(g_mic_fail.load() == mic_before,
        "...and NOT a MIC failure - it is malformed, not forged");

  /* A well-formed protected frame from a DIFFERENT BSS. Our address filter is
   * the only one in the system - the MAC runs promiscuous. */
  std::vector<uint8_t> foreign = ap.data_to_sta((const uint8_t*)"x", 1);
  foreign[10] ^= 0x02;                    /* addr2: a locally-administered bit */
  const uint64_t enc_before = g_enc_rx.load();
  rx_frame(foreign.data(), foreign.size(), -40, 0);
  check(g_enc_rx.load() == enc_before, "a frame from another BSS is not decrypted");
  check(g_sm.rx_not_our_bss > not_ours_before, "...and is counted as such");
  check(tap_read().empty(), "...and reaches nothing");
}

/* THE BOOKS CLOSE. The ledger states two identities, and an identity that
 * holds only because every confounding term is zero proves nothing - so
 * they are pinned here with the confounding terms made non-zero on purpose.
 *
 *   from host == encrypted + plaintext + dropped down
 *   queued    == aired + queue dropped + send failed
 */
void test_the_books_close() {
  reset_all();
  {
    std::lock_guard<std::mutex> l(g_mu);
    g_sm.configure(g_crypto, kSsid, kPsk, g_own);
  }
  Ap ap;
  associate(ap, /*rsn=*/true);
  drain_tx();

  uint8_t e[60] = {0};
  std::memcpy(e, kPeer, 6);
  std::memcpy(e + 6, kStaMac, 6);
  e[12] = 0x08;
  e[13] = 0x00;
  for (int i = 0; i < 5; i++) tap_down_one(e, sizeof e);      /* aired */

  /* A frame the host had no business sending: counted DOWN, not up - not in
   * the same counter as a failed write to the host. */
  std::memcpy(e + 6, kBssid, 6);
  tap_down_one(e, sizeof e);
  std::memcpy(e + 6, kStaMac, 6);

  /* And one the Ethernet parser refuses: counted in `from host` too, or it
   * would be a loss no total contained. */
  tap_down_one(e, 6);

  /* AND A REKEY. Its encrypted EAPOL answer goes through the same cipher
   * path as host traffic, and must not be counted as `encrypted`: there is
   * no `from host` to match it, and any rekeying AP (hostapd's
   * wpa_group_rekey) would break the identity. */
  uint8_t gtk2[16];
  std::memset(gtk2, 0xa7, sizeof gtk2);
  const std::vector<uint8_t> rk = ap.eapol_protected(ap.group_msg1(gtk2, 2));
  rx_frame(rk.data(), rk.size(), -40, 0);
  check(g_eapol_enc_tx.load() == 1, "the rekey in this cell is answered");

  const uint64_t from = g_tap_rx.load();
  const uint64_t down = g_tap_down_drop.load();
  check(from == 7, "every frame the host handed us is counted, malformed included");
  check(down == 2, "...and the two it could not air are counted as down-drops");
  check(from == g_tx_enc.load() + g_tx_plain.load() + down,
        "from host == encrypted + plaintext + dropped down");

  /* THE SECOND IDENTITY, with the queue cap (128) made to bite: ordinary
   * traffic never reaches it, which is why it needs a cell. */
  drain_tx();            /* empty the queue: the identity below is a DELTA,
                          * because drain_tx() is the cells' stand-in for the
                          * caller and does not count what it takes. */
  const uint64_t q_in_before = g_q_in.load();
  const uint64_t q_drop_before = g_q_drop.load();
  for (int i = 0; i < 200; i++) enqueue(std::vector<uint8_t>(32, 0xa5));
  check(g_q_in.load() == q_in_before + 200, "every frame offered to the queue is counted");
  check(g_q_drop.load() > q_drop_before,
        "...and the ones the cap refused are counted as queue drops");

  /* What send_batch() would take. Its split into aired / send failed needs
   * a device, so this half of the identity is checked up to the queue. */
  size_t kept = 0;
  {
    std::lock_guard<std::mutex> l(g_q_mu);
    kept = g_q.size();
    g_q.clear();
  }
  check(g_q_in.load() - q_in_before == kept + (g_q_drop.load() - q_drop_before),
        "queued == kept + queue dropped (send_batch's split needs a device)");
  check(kept > 0 && kept < 200, "...and neither term was vacuous");
}

}  // namespace selftest

namespace {

int self_test() {
  int fds[2];
  if (::pipe(fds) != 0) {
    std::fprintf(stdout, "FAIL: pipe()\n");
    return 1;
  }
  /* The read end is non-blocking so a cell that expects NOTHING on the TAP
   * does not hang waiting for it - and "expects nothing" is the assertion in
   * six of the cells below. */
  ::fcntl(fds[0], F_SETFL, O_NONBLOCK);
  /* And the write end, like the real TAP fd (tap_open). */
  ::fcntl(fds[1], F_SETFL, O_NONBLOCK);
  selftest::g_tap_rd = fds[0];
  g_tap_fd = fds[1];
  /* The radiotap prefix enqueue() adds. Empty would work; a realistic one
   * means drain_tx()'s stripping is exercised rather than trivially true. */
  g_rt = devourer::build_stream_radiotap(devourer::parse_tx_mode_str("6M"));

  selftest::test_open_ladder();
  selftest::test_wpa2_ladder();
  selftest::test_replay_is_refused();
  selftest::test_forged_mic_is_refused();
  selftest::test_group_key_is_chosen_by_key_id();
  selftest::test_plaintext_is_refused_on_a_protected_link();
  selftest::test_fragments_and_amsdu_are_refused();
  selftest::test_the_tid_comes_from_the_qos_control_field();
  selftest::test_a_group_key_we_cannot_use();
  selftest::test_the_fcs_is_trimmed_only_where_present();
  selftest::test_an_unconfirmed_open_association_is_lost();
  selftest::test_a_unicast_reply_confirms_the_association();
  selftest::test_an_idle_open_association_is_not_judged();
  selftest::test_a_burst_after_idle_gets_its_window();
  selftest::test_a_one_way_udp_uplink_is_not_judged();
  selftest::test_multicast_chatter_is_not_judged();
  selftest::test_fewer_questions_than_the_threshold_are_not_judged();
  selftest::test_a_group_frame_from_the_ap_does_not_confirm();
  selftest::test_what_counts_as_a_question();
  selftest::test_a_struck_bss_backs_off();
  selftest::test_a_group_frame_does_not_reset_the_backoff();
  selftest::test_a_truncated_tcp_header_is_not_read();
  selftest::test_the_stamps_are_wall_clock_microseconds();
  selftest::test_the_strike_is_per_bss();
  selftest::test_a_dropped_frame_is_not_counted();
  selftest::test_an_accepted_association_nudges_the_ap();
  selftest::test_a_nudge_touches_no_tuning_or_liveness_state();
  selftest::test_a_retransmission_is_a_duplicate();
  selftest::test_the_dup_cache_spans_a_rekey_not_an_association();
  selftest::test_a_full_tap_is_a_drop_not_a_stall();
  selftest::test_a_fault_exits_nonzero();
  selftest::test_duration_and_channel_parse_strictly();
  selftest::test_the_retry_limit_env_is_parsed();
  selftest::test_a_beacon_in_the_retune_window();
  selftest::test_a_non_key_eapol_reaches_the_host();
  selftest::test_group_addressed_eapol_is_not_the_hosts();
  selftest::test_ptk_rekey();
  selftest::test_the_channel_sweep_rotates();
  selftest::test_group_rekey();
  selftest::test_msg3_retransmit_keeps_the_pn();
  selftest::test_a_stale_bss_is_not_joined();
  selftest::test_rssi_conversion();
  selftest::test_reconnect_after_beacon_loss();
  selftest::test_a_lost_link_is_counted_once();
  selftest::test_reconnect_can_be_disabled();
  selftest::test_tap_refuses_a_foreign_source();
  selftest::test_no_data_before_association();
  selftest::test_scan_selects_the_wanted_network();
  selftest::test_hostile_and_foreign_frames();
  selftest::test_the_books_close();

  ::close(fds[0]);
  ::close(fds[1]);
  g_tap_fd = -1;
  if (selftest::g_fail) {
    std::fprintf(stdout, "sta_client_selftest: %d failure(s)\n",
                 selftest::g_fail);
    return 1;
  }
  std::fprintf(stdout, "sta_client_selftest: OK\n");
  return 0;
}

}  // namespace
