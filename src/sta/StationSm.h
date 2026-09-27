/* StationSm — the association state machine: scan result in, connected out.
 *
 * Authenticate, associate, run the four-way, and notice when any of it stops
 * working. It owns a Supplicant and drives it; it does not reimplement any of
 * the key exchange.
 *
 * NO CLOCK AND NO RADIO. Time arrives as a `now_ms` argument and frames arrive
 * through on_rx(); frames to send come out of pop_tx(). That is what makes the
 * retransmission and timeout behaviour testable at all: logic that reads a
 * steady_clock itself can only be observed on a bench, never asserted.
 *
 * WHAT IT IS NOT. Not a scanner: it has no notion of channels or dwell times,
 * because those need a radio. Feed it beacons through a BssTable and hand it
 * the entry to join.
 */
#ifndef DEVOURER_STA_STATION_SM_H
#define DEVOURER_STA_STATION_SM_H

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "sta/BssTable.h"
#include "sta/CryptoOps.h"
#include "sta/Dot11.h"
#include "sta/Eapol.h"
#include "sta/Supplicant.h"

namespace devourer {
namespace sta {

class StationSm {
 public:
  enum class State : uint8_t {
    Idle,            /* nothing in progress */
    Authenticating,  /* auth request sent */
    Associating,     /* association request sent */
    FourWay,         /* associated; the key exchange is running */
    Connected,       /* keyed, and the data plane may run */
    Failed,          /* gave up, or was thrown off; see fail_reason() */
  };

  /* Why the machine is in Failed. `status` carries the 802.11 status code of
   * a refused authentication or association, or the reason code of a deauth,
   * so "it did not connect" always comes with the number the AP gave. */
  enum class Failure : uint8_t {
    None,
    AuthTimeout,
    AuthRefused,
    AssocTimeout,
    AssocRefused,
    Deauthenticated,
    HandshakeTimeout,
    BeaconLost,   /* no frame from the AP - beacon or data - for the window */
    NoPmk,
    NotConfigured,
    NoChannel,   /* the BSS entry carries no channel: the band is unknown */
    NotInfrastructure,  /* the BSS is an IBSS (or claims no ESS): no AP */
    SsidMismatch,       /* the entry is not the configured network */
  };

  /* WHICH KIND OF BSS THIS STATION IS CONFIGURED FOR.
   *
   * Not a hypothetical second mode: without an open path a station that never
   * reaches Connected cannot say whether authentication/association or the
   * key exchange is what failed, because on a WPA2 BSS the two halves come up
   * together or not at all. The AP side of this tree has had the same ladder
   * since the beginning - tests/ap_responder.cpp is the open AP and
   * tests/ap_wpa2.cpp the protected one - and the station half did not.
   *
   * Open costs this file almost nothing: it skips the four-way and takes no
   * CryptoOps, which is the same property the open AP harness has (it links
   * no crypto library at all). */
  enum class Security : uint8_t {
    Open,
    Wpa2Psk,
  };

  /* Three transmissions of each management frame, 300 ms apart. An AP that
   * has not answered three probes in a second is not going to. */
  static constexpr int kMaxTries = 3;
  static constexpr uint32_t kMgmtTimeoutMs = 300;
  /* The authenticator drives the four-way and retransmits it; this side only
   * answers, so its timeout is a give-up, not a retry schedule. */
  static constexpr uint32_t kHandshakeTimeoutMs = 3000;
  /* The AP-liveness window. "Beacon loss" is the conventional name; what it
   * measures is ANY frame from the AP: a beacon with its fixed body, a data
   * frame from the BSSID addressed to this station, or a decrypted MSDU
   * handed to on_decrypted_msdu. Beacons alone would declare a link dead
   * while traffic flows whenever a receiver under load drops beacons first.
   *
   * Ten beacon intervals at the usual 100 TU: long enough that a few lost
   * frames mean nothing, short enough that a station does not sit Connected
   * to an AP that has been switched off. This is the FLOOR: join() widens it
   * to ten of the BSS's own advertised intervals, so an AP beaconing at
   * 500-1000 TU is not declared lost after one or two late beacons -
   * beacon_loss_ms() is the window actually in force. */
  static constexpr uint32_t kBeaconLossMs = 1024;
  /* THE CEILING on the beacon interval that widens that window. The interval
   * comes from a beacon, which anyone can forge: 65535 TU would stretch the
   * window to about 11 minutes, during which a vanished AP goes unnoticed.
   * 1000 TU (1.024 s) is ten times the usual 100 TU and covers every
   * interval an AP uses in practice, so the window is at most 10240 ms. */
  static constexpr uint32_t kMaxBeaconIntervalTu = 1000;
  /* THE TRANSMIT QUEUE IS BOUNDED. Three authentication retries plus one
   * in-flight EAPOL reply is the most this machine legitimately owes, and
   * every frame in here is produced in response to a received one - so an
   * unbounded queue is an unbounded allocation an attacker controls. */
  static constexpr size_t kMaxTxQueue = 8;

  ~StationSm() { secure_wipe(pmk_, sizeof pmk_); }

  bool configure(CryptoOps& crypto, const std::string& ssid, const char* psk,
                 const uint8_t own[6]) {
    crypto_ = &crypto;
    ssid_ = ssid;
    std::memcpy(own_, own, 6);
    /* PBKDF2 once, here, rather than per association attempt: it is 4096
     * HMAC-SHA1 iterations and the answer only depends on the passphrase and
     * the SSID, neither of which changes between retries.
     *
     * THE SNONCE IS NOT HERE. The PMK and the SNonce have opposite
     * lifetimes: the PMK is fixed for the network and the SNonce must be
     * fresh for every association. Taken here, a caller doing the obvious
     * thing - configure once, join repeatedly - would reuse one nonce across
     * every attempt and every roam, making the PTK a function of the ANonce
     * alone. It is an argument to join().
     *
     * THE OLD PMK GOES FIRST. pmk_from_psk writes nothing when it refuses a
     * passphrase, so without this wipe a reconfigure with an invalid one would
     * leave the PREVIOUS network's PMK resident. Wiped before the derivation,
     * and again after a failed one (a CryptoOps may have written part of an
     * answer). */
    secure_wipe(pmk_, sizeof pmk_);
    have_pmk_ = pmk_from_psk(crypto, psk, ssid, pmk_);
    if (!have_pmk_) secure_wipe(pmk_, sizeof pmk_);
    security_ = Security::Wpa2Psk;
    drop_association_keys();
    /* CONFIGURED EVEN WHEN THE PMK DERIVATION FAILED, on purpose. Gating this
     * on have_pmk_ would make the NoPmk branch in join() unreachable - the
     * caller would get NotConfigured, which names the wrong thing - and a
     * caller that ignores this return value is exactly the one that needs the
     * accurate diagnosis. */
    configured_ = true;
    return have_pmk_;
  }

  /* The same station on an OPEN BSS: no PSK, no PMK, no four-way, and no
   * CryptoOps - which is why this overload takes none. See the Security enum
   * for why an open path is worth having at all.
   *
   * It is a separate function rather than a null `psk` because a null
   * passphrase reads like a caller's mistake, and because the WPA2 form needs
   * a CryptoOps this one has no use for. */
  bool configure_open(const std::string& ssid, const uint8_t own[6]) {
    crypto_ = nullptr;
    security_ = Security::Open;
    ssid_ = ssid;
    std::memcpy(own_, own, 6);
    /* A station reconfigured from WPA2 to open must not keep the old PMK
     * sitting in memory for the rest of the process's life - and that means
     * the Supplicant's copy too, with the PTK and GTK it derived, not only
     * this object's. drop_association_keys() does that half. The wipe is
     * observable through pmk() and pinned by the reconfigure cell in
     * tests/station_sm_selftest.cpp. */
    secure_wipe(pmk_, sizeof pmk_);
    have_pmk_ = false;
    drop_association_keys();
    configured_ = true;
    return true;
  }

  /* A reconfigured station is associated to nothing: the Supplicant forgets
   * its PMK, PTK and GTK, and a live association drops back to Idle rather
   * than claiming keyed() under a configuration that no longer matches it.
   * A Failed state keeps its reason for the caller to read. */
  void drop_association_keys() {
    sup_.forget();
    /* And whatever was queued for that association - an auth, assoc or
     * EAPOL frame built under the old configuration must not air after the
     * machine has let the association go (join() clears it for the same
     * reason). */
    tx_.clear();
    aid_ = 0;
    authenticated_ = false;
    if (state_ != State::Failed) state_ = State::Idle;
  }

  /* Begin an association with this BSS.
   *
   * `snonce` must be UNPREDICTABLE AND FRESH FOR THIS ATTEMPT - see
   * Supplicant::start, which explains at length why this library takes it
   * rather than inventing it. */
  bool join(const BssEntry& bss, const uint8_t snonce[32], uint32_t now_ms) {
    if (!configured_) { fail(Failure::NotConfigured, 0); return false; }
    /* The entry must be the network this station is configured for:
     * BssTable::select matches on the SSID, and a hand-picked entry is held to
     * the same rule - the PMK is derived from this SSID, so a different one
     * could never complete the four-way. */
    if (bss.info.ssid != ssid_) { fail(Failure::SsidMismatch, 0); return false; }
    /* The channel picks the band, and the band picks the association
     * request's rate set: channel 0 would send 802.11b rates to a 5 GHz AP,
     * which refuses them. BssTable::select never offers such an entry; this
     * catches a hand-picked one. */
    if (!channel_valid(bss.info.channel)) {
      fail(Failure::NoChannel, 0);
      return false;
    }
    /* Authentication and association are an exchange with an AP; an IBSS
     * has none. BssTable::select never offers one either. */
    if (!bss_is_infrastructure(bss.info)) {
      fail(Failure::NotInfrastructure, 0);
      return false;
    }
    /* Refuse a BSS this station cannot finish with, rather than authenticating
     * and discovering it at the four-way. BssTable::select already filters on
     * this; join() is also reachable with a hand-picked entry. */
    if (security_ == Security::Wpa2Psk) {
      if (!have_pmk_) { fail(Failure::NoPmk, 0); return false; }
      if (!bss_is_wpa2_psk(bss.info)) {
        fail(Failure::AssocRefused, 0);
        return false;
      }
      /* Kept for the four-way: message 3 must carry the same RSN element
       * (Supplicant::start, the downgrade check). */
      ap_rsn_ = bss.info.rsn;
      /* The SNonce is only read on this path, so it is only required on this
       * path - but a WPA2 join without one would start the supplicant with a
       * nonce of whatever was in the buffer, which for a caller that
       * configures once and joins repeatedly is the PREVIOUS association's.
       * See the comment in configure() about why it is an argument at all. */
      if (!snonce) { fail(Failure::NotConfigured, 0); return false; }
    } else if (bss.info.privacy) {
      /* An open station cannot carry traffic on a BSS that encrypts it. The
       * Privacy bit is set by WEP, WPA and RSN alike, so this one test covers
       * every protected BSS without parsing any of them. Refusing here rather
       * than at the data plane is the difference between "no candidate" and
       * an association that succeeds and then passes nothing. */
      fail(Failure::AssocRefused, 0);
      return false;
    }

    /* THE QUEUE IS CLEARED. Without this, frames still queued for the BSS we
     * gave up on are transmitted at the one we just joined - addressed to the
     * old BSSID, on the new channel, after the radio has retuned. A test that
     * drains the queue between steps cannot see it;
     * test_join_clears_the_transmit_queue does not drain. */
    tx_.clear();
    /* A join on a live association says goodbye to the OLD AP first, for the
     * reason leave() does: an AP that accepted our authentication holds state
     * for this station until it is told otherwise or times it out. */
    if (authenticated_) {
      std::vector<uint8_t> m = build_deauth(own_, bssid_, 3);
      assign_seq(m, seq_.next());
      queue(std::move(m));
    }
    std::memcpy(bssid_, bss.info.bssid, 6);
    if (security_ == Security::Wpa2Psk) std::memcpy(snonce_, snonce, 32);
    channel_ = bss.info.channel;
    {
      /* Ten of THIS BSS's beacon intervals (1 TU = 1.024 ms), never less
       * than the kBeaconLossMs floor, and with the interval capped at
       * kMaxBeaconIntervalTu: it comes off the air. */
      const uint32_t tu = bss.info.beacon_interval_tu > kMaxBeaconIntervalTu
                              ? kMaxBeaconIntervalTu
                              : bss.info.beacon_interval_tu;
      const uint32_t ten = tu * 1024u * 10u / 1000u;
      beacon_loss_ms_ = ten > kBeaconLossMs ? ten : kBeaconLossMs;
    }
    aid_ = 0;
    fail_ = Failure::None;
    status_ = 0;
    sup_.forget();
    authenticated_ = false;
    state_ = State::Authenticating;
    tries_ = 0;
    last_heard_ms_ = now_ms;
    send_auth(now_ms);
    return true;
  }

  /* Leave cleanly: tell the AP, drop the keys, and go back to Idle.
   *
   * An association this side simply abandons stays alive at the AP until it
   * times the station out, holding an AID and, on this project's own AP, a
   * slot in a seven-entry table. */
  void leave(uint16_t reason = 3) {
    if (state_ == State::Idle) return;
    /* Whatever was still queued for this association - an authentication or
     * association retry, an EAPOL reply - must not air after the station has
     * said goodbye, and must not air BEFORE the goodbye either. */
    tx_.clear();
    /* A deauthentication goes out only if the AP holds state for this
     * station - an authentication it accepted, whether or not an association
     * followed. After an AuthTimeout there is none, and a deauth would be
     * addressed to an AP that never heard of us. */
    if (authenticated_) {
      std::vector<uint8_t> m = build_deauth(own_, bssid_, reason);
      assign_seq(m, seq_.next());
      queue(std::move(m));
    }
    sup_.forget();
    state_ = State::Idle;
    fail_ = Failure::None;
    aid_ = 0;
    authenticated_ = false;
  }

  /* One received frame. `len` is the true MPDU length with no FCS. */
  void on_rx(const uint8_t* frame, size_t len, uint32_t now_ms) {
    if (!frame || len < 24) return;
    if (state_ == State::Idle || state_ == State::Failed) return;

    const uint8_t fc0 = frame[0], fc1 = frame[1];
    const uint8_t* a1 = frame + 4;
    const uint8_t* a2 = frame + 10;

    /* EVERYTHING must come from the BSS we are talking to and be addressed to
     * this station (or broadcast). Without the addr2 check, any frame from any
     * AP on the channel drives this machine.
     *
     * THE DROPS ARE COUNTED. On real hardware this is the only address filter
     * in the system - the MT7612U RX path runs promiscuous - so most of a busy
     * channel lands here, and a station that connects to nothing has to be
     * able to say whether it heard its AP at all. */
    if (std::memcmp(a2, bssid_, 6) != 0) { rx_not_our_bss++; return; }
    const bool to_us = std::memcmp(a1, own_, 6) == 0;
    const bool bcast = (a1[0] & 0x01) != 0;
    if (!to_us && !bcast) { rx_not_for_us++; return; }

    /* A beacon from our own BSS is a liveness signal (so is data - below).
     * Counted before the switch because it matters in every state. */
    if (fc0 == kFcBeacon || fc0 == kFcProbeResp) {
      /* Only a frame that carries the 12-byte fixed body (timestamp,
       * interval, capability) counts: a bare 24-byte header from the BSSID
       * is not a beacon, and must not hold off beacon-loss supervision. */
      if (len < 24 + 12) { rx_malformed++; return; }
      beacons_rx++;
      last_heard_ms_ = now_ms;
      return;
    }

    switch (fc0) {
      case kFcAuth:
        if (to_us) on_auth(frame, len, now_ms);
        return;
      /* Association Response only. This station sends an Association
       * Request, never a Reassociation Request, so a Reassociation Response
       * answers nothing it asked - ignored, not taken as success. */
      case kFcAssocResp:
        if (to_us) on_assoc_resp(frame, len, now_ms);
        return;
      case kFcReassocResp:
        rx_ignored++;
        return;
      case kFcDeauth:
      case kFcDisassoc: {
        uint16_t reason = 0;

        /* ACCEPTED UNAUTHENTICATED, and that is a known cost rather than an
         * oversight: 802.11w is not implemented here, so there is no way to
         * tell a real deauthentication from a forged one, and a station that
         * ignored them would stay associated to an AP that has forgotten it.
         * This is the accepted price of having no MFP.
         *
         * A frame too short to carry its reason code is not a
         * deauthentication at all: it is counted malformed and changes
         * nothing, rather than ending the association "with reason 0". */
        if (!parse_reason(frame, len, &reason)) { rx_malformed++; return; }
        fail(Failure::Deauthenticated, reason);
        authenticated_ = false;   /* the AP has already let us go */
        return;
      }
      default:
        break;
    }

    /* Data frames: the only one this machine cares about is EAPOL. Anything
     * else is somebody's traffic, not a protocol error, so it is counted
     * separately from a frame that was addressed wrongly. */
    if (fc0 != kFcData && !is_qos_data(fc0)) { rx_ignored++; return; }
    if (!(fc1 & kFcFromDs) || (fc1 & kFcToDs)) { rx_ignored++; return; }
    /* AP LIVENESS: a from-DS data frame from the BSSID, addressed to this
     * station, with its whole header present, is proof the AP is there -
     * protected or not. Under load a receiver drops beacons first, and a link
     * carrying traffic must not be declared lost for want of them. */
    if (to_us && len >= data_hdr_len(fc0, fc1)) last_heard_ms_ = now_ms;
    /* A "no data" subtype (Null, QoS Null: subtype bit 2, fc0 & 0x40) is a
     * well-formed frame with no body by definition - ignored, not malformed,
     * and it has already counted as liveness above. */
    if (fc0 & 0x40) { rx_ignored++; return; }
    /* The FOUR-WAY is never protected - the keys it carries are what
     * protection would need - so a protected data frame is not one of its
     * messages and this machine cannot read it anyway: it holds no cipher.
     *
     * THE GROUP KEY HANDSHAKE IS A DIFFERENT MATTER, and this refusal used
     * to be the end of the story for it. It runs AFTER the PTK is installed
     * and is therefore protected like any other data frame. The caller
     * decrypts and hands the plaintext back through on_decrypted_msdu().
     *
     * COUNTED SEPARATELY FROM rx_ignored, because on a working link this is
     * EVERY DATA FRAME, and lumping it in would bury the one counter set that
     * answers "why did nothing associate": 75 frames of ordinary traffic
     * would read as ignored=75, i.e. as 75 protocol errors. */
    if (fc1 & kFcProtected) { rx_protected++; return; }
    /* FRAGMENTS AND A-MSDUs ARE NOT MSDUs. Nothing here reassembles, so the
     * bytes at the LLC offset are a piece of a frame, not a frame - and an
     * A-MSDU's are a subframe header. Feeding either to the EAPOL parser
     * asks it to interpret the wrong bytes.
     *
     * A caller's data plane should refuse both as well, and the two receive
     * layers disagreeing about it is the kind of gap a
     * later reader closes in only one place. More Fragments is CLEAR on the
     * LAST fragment, so the fragment number has to be tested too. */
    if ((fc1 & kFcMoreFrag) || (frame[22] & 0x0f)) { rx_malformed++; return; }
    const size_t hlen = data_hdr_len(fc0, fc1);
    if (len < hlen + kLlcSnapLen) { rx_malformed++; return; }
    /* The A-MSDU Present bit, in the QoS Control field - which is at a FIXED
     * offset, with HT Control after it, so it is 24 and not hlen - 2. A
     * 4-address frame would put it at 30, and cannot reach here: the
     * FromDS/ToDS test above accepts only from-the-DS frames. */
    if (is_qos_data(fc0) && (frame[24] & 0x80)) { rx_malformed++; return; }
    const uint8_t* llc = frame + hlen;
    if (!(llc[0] == 0xaa && llc[1] == 0xaa && llc[2] == 0x03)) {
      rx_ignored++;
      return;
    }
    if (!(llc[6] == 0x88 && llc[7] == 0x8e)) { rx_ignored++; return; }
    /* A GROUP-ADDRESSED EAPOL-Key is part of no handshake with this station:
     * every message of the four-way and the group key handshake is unicast
     * to the supplicant. The decrypted path (on_decrypted_msdu's caller)
     * already refuses one; this layer refuses it too, so the two receive
     * paths agree and a broadcast forgery cannot reach the supplicant. */
    if (!to_us) { rx_ignored++; return; }
    /* ONLY EAPOL-KEY REACHES THE SUPPLICANT. EAP, EAPOL-Start and Logoff
     * share the ethertype; the key parser would refuse them as malformed,
     * which counts a protocol this station does not speak as a broken
     * handshake. Not a Key packet: ignored. No packet-type octet at all:
     * malformed. (on_decrypted_msdu applies the same test and hands a
     * non-Key packet back to its caller instead.) */
    const uint8_t* eapol = llc + kLlcSnapLen;
    const size_t eapol_len = len - hlen - kLlcSnapLen;
    if (eapol_len < 2) { rx_malformed++; return; }
    if (!eapol_is_key(eapol, eapol_len)) { rx_ignored++; return; }
    if (!cleartext_eapol_allowed(eapol, eapol_len)) { rx_ignored++; return; }
    on_eapol(eapol, eapol_len, now_ms);
  }

  /* One DECRYPTED MSDU - LLC/SNAP followed by its payload - that arrived
   * from our AP addressed to this station. Returns true when it was an
   * EAPOL-Key frame and has been consumed; the caller gives anything else to
   * the host.
   *
   * WHY THIS EXISTS. on_rx() refuses every protected data frame, which is
   * right for the four-way (it runs before there is a key) and WRONG for the
   * GROUP KEY HANDSHAKE, which runs after the PTK is installed and is
   * protected like any other data frame. Without this path hostapd logs
   *
   *     WPA: pairwise key handshake completed (RSN)
   *     WPA: group key handshake failed (RSN) after 4 tries
   *     AP-STA-DISCONNECTED
   *
   * A station that cannot answer a rekey is thrown off by every AP that
   * performs one, which is most of them - and the link looks healthy right up
   * until it ends. (test_group_rekey_through_the_decrypted_path)
   *
   * `out_reply` takes the EAPOL-Key body to send back. It is a BODY and not
   * a frame because the answer has to be encrypted, and the cipher belongs
   * to the caller - see eapol_reply().
   *
   * The caller has already verified the frame's MIC, which is a stronger
   * statement than "the Protected bit was set", so nothing is given up by
   * taking the plaintext. */
  bool on_decrypted_msdu(const uint8_t* msdu, size_t len, uint32_t now_ms,
                         std::vector<uint8_t>* out_reply) {
    if (!msdu || len == 0) return false;
    /* A decrypted MSDU from the AP is the strongest liveness signal there is
     * - its MIC verified - whatever it carries. */
    last_heard_ms_ = now_ms;
    if (!is_ethertype_snap(msdu, len)) return false;
    if (!(msdu[6] == 0x88 && msdu[7] == 0x8e)) return false;
    /* Claimed only if it is an EAPOL-KEY packet. An EAP packet, an
     * EAPOL-Start or a Logoff is returned unconsumed so the caller can deliver
     * it, rather than vanishing into a key parser that refuses it. A Key
     * packet is claimed even when it is malformed - it was ours, and it was
     * bad. */
    if (!eapol_is_key(msdu + kLlcSnapLen, len - kLlcSnapLen)) return false;
    bool progressed = false;
    std::vector<uint8_t> reply =
        eapol_reply(msdu + kLlcSnapLen, len - kLlcSnapLen, now_ms, &progressed);
    /* Handed to the caller, who sends it: counted as sent here, because this
     * machine never sees it again. */
    if (!reply.empty()) {
      eapol_tx++;
      if (progressed) last_tx_ms_ = now_ms;
      promote_if_keyed(now_ms);
    }
    if (out_reply) *out_reply = std::move(reply);
    return true;
  }

  /* Drive timeouts and retransmissions. Call it as often as convenient; it
   * does nothing until a deadline has passed. */
  void tick(uint32_t now_ms) {
    const uint32_t since = (uint32_t)(now_ms - last_tx_ms_);

    switch (state_) {
      case State::Authenticating:
        if (since < kMgmtTimeoutMs) return;
        if (tries_ >= kMaxTries) { fail(Failure::AuthTimeout, 0); return; }
        send_auth(now_ms);
        return;
      case State::Associating:
        if (since < kMgmtTimeoutMs) return;
        if (tries_ >= kMaxTries) { fail(Failure::AssocTimeout, 0); return; }
        send_assoc(now_ms);
        return;
      case State::FourWay:
        /* No retransmission: the authenticator owns that schedule. This is
         * only the give-up, and it is measured from the last thing that
         * actually moved the handshake forward. */
        if (since >= kHandshakeTimeoutMs) fail(Failure::HandshakeTimeout, 0);
        return;
      case State::Connected:
        /* AP-LIVENESS SUPERVISION ("beacon loss": see kBeaconLossMs for what
         * counts as hearing the AP). Without it Connected has no exit but a
         * deauth, and an AP that is switched off leaves the station reporting
         * a link that does not exist - the caller sees keyed() forever and
         * has no hook to notice. */
        if ((uint32_t)(now_ms - last_heard_ms_) >= beacon_loss_ms_)
          fail(Failure::BeaconLost, 0);
        return;
      default:
        return;
    }
  }

  /* Take one frame to transmit, oldest first. Returns false when empty, and
   * for a null `out`: a frame taken off the queue with nowhere to put it
   * would be lost while the caller is told it was delivered. */
  bool pop_tx(std::vector<uint8_t>* out) {
    if (!out || tx_.empty()) return false;
    *out = std::move(tx_.front());
    tx_.erase(tx_.begin());
    return true;
  }

  size_t pending_tx() const { return tx_.size(); }
  /* The Connected-state beacon-loss window in force - see kBeaconLossMs. */
  uint32_t beacon_loss_ms() const { return beacon_loss_ms_; }
  static constexpr size_t tx_capacity() { return kMaxTxQueue; }
  State state() const { return state_; }
  Failure fail_reason() const { return fail_; }
  uint16_t status() const { return status_; }
  uint16_t aid() const { return aid_; }
  uint8_t channel() const { return channel_; }
  const uint8_t* bssid() const { return bssid_; }
  const Supplicant& supplicant() const { return sup_; }
  /* The PMK this station derived, 32 bytes, all zero when it holds none -
   * like supplicant().ptk(), exposed so a caller or a test can confirm key
   * material is gone rather than take it on trust. */
  const uint8_t* pmk() const { return pmk_; }
  bool has_pmk() const { return have_pmk_; }
  bool keyed() const { return state_ == State::Connected && sup_.ptk_valid(); }
  Security security() const { return security_; }
  /* Associated and able to carry data. On a WPA2 BSS that is keyed(); on an
   * open one there is no key, so a data plane gated on keyed() would never
   * transmit at all. This is the predicate a caller wants. */
  bool connected() const {
    return state_ == State::Connected &&
           (security_ == Security::Open || sup_.ptk_valid());
  }

  uint32_t auth_tx = 0;
  uint32_t assoc_tx = 0;
  uint32_t eapol_tx = 0;
  uint32_t eapol_rx = 0;
  uint32_t beacons_rx = 0;
  uint32_t rx_not_our_bss = 0;
  uint32_t rx_not_for_us = 0;
  uint32_t rx_ignored = 0;
  /* Protected data frames, which this machine cannot read and the caller is
   * expected to decrypt. Ordinary traffic on a keyed link, NOT an error. */
  uint32_t rx_protected = 0;
  uint32_t rx_malformed = 0;
  uint32_t tx_dropped = 0;

 private:
  /* WHAT AN UNPROTECTED EAPOL-KEY FRAME MAY BE. The clear carries the
   * four-way, which runs before there is a key; once keys exist, the AP
   * protects its EAPOL frames and they arrive through on_decrypted_msdu.
   *
   *  - A group-key message (not pairwise) is never taken from the clear: the
   *    group handshake runs only after the PTK is installed, protected.
   *  - Before the PTK is installed, pairwise messages 1 and 3 are.
   *  - After it, one pairwise message only: a retransmission of the message 3
   *    of the handshake already installed (same ANonce). The AP installs its
   *    own PTK only when our message 4 arrives, so if that message 4 is lost
   *    it retransmits message 3 in the clear; ignoring it would leave the AP
   *    to give up and deauthenticate a working station. The supplicant
   *    answers it and never reinstalls. A PTK rekey is protected, so a
   *    cleartext message 1, or a message 3 with a new ANonce, is ignored.
   *
   * A frame that does not parse is let through, so the supplicant counts it
   * as malformed. */
  bool cleartext_eapol_allowed(const uint8_t* eapol, size_t len) const {
    EapolKey k;
    if (!parse_eapol_key(eapol, len, &k)) return true;
    if (!k.pairwise()) return false;
    if (!sup_.ptk_valid()) return true;
    return sup_.is_installed_msg3(k);
  }

  /* One place that enforces the bound, so no future sender can forget it.
   * Returns false when the frame was dropped, so a caller whose bookkeeping
   * assumes the frame went out (a deadline, a counter) can tell. */
  bool queue(std::vector<uint8_t> m) {
    if (tx_.size() >= kMaxTxQueue) { tx_dropped++; return false; }
    tx_.push_back(std::move(m));
    return true;
  }

  /* A try, its deadline and its counter advance only for a frame that was
   * actually queued, as on the EAPOL path. join() clears the queue and adds
   * at most one deauthentication, and nothing else is queued while
   * authenticating or associating, so kMaxTries of each request always fit:
   * a management request is never the frame queue() drops, and the
   * static_assert keeps that true. */
  static_assert((size_t)(2 * kMaxTries + 1) <= kMaxTxQueue,
                "a deauth plus every auth and assoc retry must fit the queue");
  void send_auth(uint32_t now_ms) {
    std::vector<uint8_t> m = build_auth_req(own_, bssid_);
    assign_seq(m, seq_.next());
    if (!queue(std::move(m))) return;
    last_tx_ms_ = now_ms;
    tries_++;
    auth_tx++;
  }

  void send_assoc(uint32_t now_ms) {
    /* The band comes from the joined entry's channel - the beacon's DS
     * Parameter Set, or the channel it was received on when the beacon has
     * none (BssTable::observe) - because the rate set differs: a 5 GHz
     * association request carrying 802.11b rates is refused. join() refuses
     * an entry with no channel, so channel_ is never 0 here. */
    std::vector<uint8_t> m =
        build_assoc_req(own_, bssid_, ssid_,
                        /*rsn=*/security_ == Security::Wpa2Psk,
                        /*five_ghz=*/channel_ > 14);
    /* build_assoc_req returns an empty vector for an SSID it cannot encode.
     * Sending a truncated association request would be worse than failing. */
    if (m.empty()) { fail(Failure::AssocRefused, 0); return; }
    assign_seq(m, seq_.next());
    if (!queue(std::move(m))) return;
    last_tx_ms_ = now_ms;
    tries_++;
    assoc_tx++;
  }

  void on_auth(const uint8_t* frame, size_t len, uint32_t now_ms) {
    AuthFields a;

    if (state_ != State::Authenticating) return;
    /* Too short for its fixed fields: malformed, counted like a short deauth. */
    if (!parse_auth(frame, len, &a)) { rx_malformed++; return; }
    /* Open System only. A Shared Key response is a four-frame exchange this
     * does not implement, and treating its sequence 2 as success would send an
     * association request into a state the AP is not in. */
    if (a.algorithm != 0) { fail(Failure::AuthRefused, a.status); return; }
    if (a.seq != 2) return;
    if (a.status != 0) { fail(Failure::AuthRefused, a.status); return; }

    authenticated_ = true;   /* the AP now holds state for this station */
    state_ = State::Associating;
    tries_ = 0;
    send_assoc(now_ms);
  }

  void on_assoc_resp(const uint8_t* frame, size_t len, uint32_t now_ms) {
    AssocRespFields r;

    if (state_ != State::Associating) return;
    if (!parse_assoc_resp(frame, len, &r)) { rx_malformed++; return; }
    if (r.status != 0) { fail(Failure::AssocRefused, r.status); return; }
    /* AID 0 is not a valid association identifier; an AP that answers success
     * with one has not actually allocated anything. */
    if (r.aid == 0) { fail(Failure::AssocRefused, r.status); return; }

    aid_ = r.aid;
    last_tx_ms_ = now_ms;
    last_heard_ms_ = now_ms;
    /* An open association is complete the moment the AP accepts it - there is
     * no key exchange to wait for, so FourWay would be a state nothing could
     * ever leave. */
    if (security_ == Security::Open) {
      state_ = State::Connected;
      return;
    }
    state_ = State::FourWay;
    sup_.start(*crypto_, pmk_, own_, bssid_, snonce_, &ap_rsn_);
  }

  /* One EAPOL-Key frame in, the EAPOL-Key body to send back out (or empty).
   *
   * THE REPLY IS A BODY AND NOT A FRAME because the two callers need
   * different framing: the four-way is unprotected and this machine can
   * build it, while a group rekey's answer must be encrypted and this
   * machine holds no cipher. The caller with the keys does that half.
   *
   * `*progressed` says whether the handshake moved (a Reply, not a
   * Retransmit). The CALLER moves the deadline, and only once the reply has
   * actually left: moving it here, before a queue() that can drop the frame,
   * would let the FourWay give-up keep sliding forward while nothing is
   * sent. */
  std::vector<uint8_t> eapol_reply(const uint8_t* eapol, size_t len,
                                   uint32_t now_ms, bool* progressed) {
    std::vector<uint8_t> reply;

    *progressed = false;

    /* An EAPOL-Key frame on an open link is never ours: the supplicant was
     * never started, so it holds no PMK and has nothing to verify a MIC
     * against. Counted as ignored rather than dropped silently, because "the
     * AP is trying to key us and we are configured open" is a configuration
     * mismatch worth being able to see. */
    if (security_ != Security::Wpa2Psk) { rx_ignored++; return {}; }
    if (state_ != State::FourWay && state_ != State::Connected) return {};
    eapol_rx++;
    const Supplicant::Verdict v = sup_.on_eapol(eapol, len, &reply);

    /* The deadline moves only when the handshake moved. A retransmission we
     * answered again is not progress, and letting it push the give-up out
     * would let a stuck authenticator hold this state open forever. */
    *progressed = v == Supplicant::Verdict::Reply;
    if (v != Supplicant::Verdict::Reply &&
        v != Supplicant::Verdict::Retransmit)
      return {};
    return reply;
  }

  void on_eapol(const uint8_t* eapol, size_t len, uint32_t now_ms) {
    bool progressed = false;
    const std::vector<uint8_t> reply =
        eapol_reply(eapol, len, now_ms, &progressed);

    if (reply.empty()) return;
    std::vector<uint8_t> m = data_hdr_to_ds(bssid_, own_, bssid_,
                                            /*protect=*/false, seq_.next());
    append_llc_snap(m, 0x888e);
    m.insert(m.end(), reply.begin(), reply.end());
    if (!queue(std::move(m))) return;           /* dropped: nothing was sent */
    eapol_tx++;
    if (progressed) last_tx_ms_ = now_ms;
    promote_if_keyed(now_ms);
  }

  /* FourWay -> Connected, ONLY once message 4 has actually left. The
   * supplicant is Done the moment it accepts message 3, but a message 4 that
   * queue() dropped never reached the AP, and a station that called itself
   * Connected then would carry traffic the AP has not keyed. Staying in
   * FourWay is safe: the AP retransmits message 3, the supplicant answers it
   * again (Retransmit at an equal counter, a fresh Reply at a greater one,
   * never a reinstall), and THAT reply being queued is what promotes. The
   * give-up still bounds it, because a dropped reply does not move
   * last_tx_ms_. */
  void promote_if_keyed(uint32_t now_ms) {
    if (state_ == State::FourWay && sup_.state() == Supplicant::State::Done &&
        sup_.ptk_valid()) {
      state_ = State::Connected;
      last_heard_ms_ = now_ms;
    }
  }

  /* A failure ends the association, whatever caused it - a timeout, a
   * refusal, or the peer's deauth/disassoc. So it drops what an ended
   * association leaves behind: frames still queued for it (they must not air
   * at an AP that has let us go) and the supplicant's per-association keys
   * and cached replies, which would otherwise stay readable through
   * supplicant() for as long as this object lives. The PMK and the
   * configuration stay - a caller rejoins without reconfiguring - and so do
   * the failure reason and status, which are what the caller reads next. */
  void fail(Failure why, uint16_t status) {
    tx_.clear();
    sup_.forget();
    aid_ = 0;   /* no association, so no association ID to report */
    state_ = State::Failed;
    fail_ = why;
    status_ = status;
  }

  CryptoOps* crypto_ = nullptr;
  Security security_ = Security::Wpa2Psk;
  bool configured_ = false;
  State state_ = State::Idle;
  Failure fail_ = Failure::None;
  std::string ssid_;
  uint8_t own_[6] = {0};
  uint8_t bssid_[6] = {0};
  RsnInfo ap_rsn_{};  /* the joined BSS's advertised RSN element */
  uint8_t snonce_[32] = {0};
  uint8_t pmk_[32] = {0};
  bool have_pmk_ = false;
  uint8_t channel_ = 0;
  uint16_t aid_ = 0;
  bool authenticated_ = false;   /* the AP accepted our authentication */
  uint16_t status_ = 0;
  int tries_ = 0;
  uint32_t last_tx_ms_ = 0;
  uint32_t last_heard_ms_ = 0;
  uint32_t beacon_loss_ms_ = kBeaconLossMs;
  SeqCounter seq_;
  Supplicant sup_;
  std::vector<std::vector<uint8_t>> tx_;
};

}  // namespace sta
}  // namespace devourer

#endif /* DEVOURER_STA_STATION_SM_H */
