/* Supplicant — the station half of the WPA2-PSK key exchange.
 *
 * The opposite role to the authenticator in tests/ap_wpa2.cpp, and written
 * from the standard rather than by mirroring that code, so the two are
 * independent implementations that a test can run against each other.
 *
 * Reactive and timer-free. The authenticator retransmits message 1 and
 * message 3 (802.11-2016 12.7.6.4); a supplicant answers what arrives and
 * never retransmits on its own, so there is no clock in here and no schedule
 * to get wrong.
 *
 * WHAT IT REFUSES, and why each refusal is a counter rather than a silent
 * drop - a handshake that does not complete has to say which rule stopped it,
 * or the only diagnosis available is "it did not associate":
 *
 *   - a MIC that does not verify                        mic_failures
 *   - a key replay counter that does not advance        replays
 *   - a retransmission (equal counter): answered again, installs nothing
 *                                                       retransmits
 *   - a message arriving in a state that cannot use it  out_of_state
 *   - a message 3 whose RSN element differs from the
 *     one the AP advertised (a downgrade, 12.7.6.4)     rsn_mismatches
 *     (counted under malformed too)
 *   - anything malformed, over-long, or a descriptor
 *     version whose MIC is a different algorithm        malformed
 *   - a well-formed EAPOL-Key this role does not handle ignored
 *   - our own CryptoOps failing, which is not the
 *     frame's fault and must not read as an attack      crypto_errors
 *
 * THE FOUR RULES THIS IS BUILT ON, each pinned by the cells named:
 *
 *   1. NOTHING IS INSTALLED BEFORE A MIC VERIFIES. Message 3 carries the GTK
 *      and confirms the PTK; accepting it without a verified MIC hands an
 *      attacker the ability to install key material. Nothing here touches the
 *      installed keys until a MIC verifies under a key derived from the PMK.
 *      (test_forged_mic_is_rejected)
 *
 *   2. AN EQUAL-COUNTER REPLAYED GROUP REKEY INSTALLS NOTHING. Replaying a
 *      captured group message 1 would reinstall an OLD GTK - which, with its
 *      own PN space reset, is keystream reuse across every group frame since.
 *      Only a STRICTLY GREATER counter installs anything; an equal one is
 *      treated as the AP's retransmission and answered with the cached reply.
 *      (test_group_rekey_replay_rejected)
 *
 *   3. MESSAGE 1 NEVER MOVES THE REPLAY COUNTER. It carries no MIC - anyone
 *      who can hear the BSSID can build one - so if it could advance the
 *      counter, one forged frame quoting 2^64-1 would refuse every genuine
 *      EAPOL-Key for the rest of the association, silently: the station would
 *      stay Connected and keyed with a dead rekey path. The counter advances
 *      ONLY where a MIC has verified. What an unauthenticated message 1 can
 *      still do is replace the candidate PTK of a handshake in flight - see
 *      on_msg1 for exactly what that costs. The INSTALLED handshake's ANonce
 *      and its message 4 are kept apart from the candidate for exactly that
 *      reason: a forged message 1 must not stop the AP's own retransmitted
 *      message 3 from being answered. (test_forged_msg1_cannot_poison_the_
 *      counter, test_forged_msg1_does_not_orphan_msg3_retransmit)
 *
 *   4. NO KEY REINSTALLATION (KRACK, CVE-2017-13077/13078/13080). The AP
 *      retransmits message 3 whenever our message 4 is lost, at a STRICTLY
 *      GREATER counter (hostapd increments on every retransmission), so the
 *      replay gate rightly lets it through. It must be answered, and it must
 *      NOT reinstall: a caller that restarts its PN when the key generation
 *      moves would then reuse CCMP nonces under the unchanged TK. So a key
 *      that is already installed is never installed again - the PTK
 *      generation moves only when the PTK bytes change, and the GTK
 *      generation only when the key id or bytes change, as wpa_supplicant's
 *      "not reinstalling already in-use" rule does.
 *      (test_msg3_retransmit_does_not_reinstall,
 *      test_group1_same_gtk_does_not_reinstall)
 */
#ifndef DEVOURER_STA_SUPPLICANT_H
#define DEVOURER_STA_SUPPLICANT_H

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "sta/CryptoOps.h"
#include "sta/Dot11.h"
#include "sta/Eapol.h"

namespace devourer {
namespace sta {

class Supplicant {
 public:
  enum class State : uint8_t {
    Idle,       /* not started */
    WaitMsg1,   /* started; nothing derived yet */
    WaitMsg3,   /* msg1 seen, candidate PTK derived, msg2 sent */
    Done,       /* PTK and GTK installed, msg4 sent */
  };

  /* What on_eapol() did with the frame. `Reply` means `out` holds an EAPOL
   * frame body for the caller to wrap in a to-DS data frame and send. */
  enum class Verdict : uint8_t {
    Ignored,      /* a well-formed EAPOL-Key this role does not handle */
    Malformed,
    CryptoError,  /* our own CryptoOps failed; not the frame's fault */
    MicFailed,
    Replayed,
    Retransmit,   /* equal counter, same message: `out` holds the same reply */
    OutOfState,
    Reply,        /* `out` holds msg2, msg4, or group msg2 */
  };

  ~Supplicant() { forget(); }

  /* Begin a handshake.
   *
   * `snonce` IS SUPPLIED BY THE CALLER, and this is deliberate rather than
   * lazy. `libdevourer` has no random-number dependency and CryptoOps offers
   * none, so the alternative is a stub that looks like entropy and is not.
   * Making it an argument puts the requirement where somebody has to read it:
   * THE SNONCE MUST BE UNPREDICTABLE, AND FRESH PER ASSOCIATION. A constant, a
   * counter or a timestamp makes the PTK derivable from the air by anyone who
   * knows the PSK - which on a PSK network is every other station.
   */
  /* `ap_rsn` is the RSN element the AP advertised in the Beacon or Probe
   * Response this association was chosen from. When given, message 3's RSN
   * element must match it (802.11-2016 12.7.6.4, the downgrade check): an
   * attacker who forged the advertisement to steer the choice is caught by
   * the MIC-protected copy. Compared field by field, not byte by byte - an AP
   * may encode the same element differently in the two places, which is what
   * wpa_supplicant tolerates too. Null skips the check; StationSm always
   * passes it. */
  void start(CryptoOps& crypto, const uint8_t pmk[32], const uint8_t own[6],
             const uint8_t bssid[6], const uint8_t snonce[32],
             const RsnInfo* ap_rsn = nullptr) {
    forget();
    if (ap_rsn && ap_rsn->valid) {
      ap_rsn_ = *ap_rsn;
      ap_rsn_set_ = true;
    }
    crypto_ = &crypto;
    std::memcpy(pmk_, pmk, 32);
    std::memcpy(own_, own, 6);
    std::memcpy(bssid_, bssid, 6);
    std::memcpy(snonce_, snonce, 32);
    state_ = State::WaitMsg1;
  }

  /* Drop every key this object holds. Called by start() and the destructor;
   * a caller that is finished early may call it directly. */
  void forget() {
    secure_wipe(pmk_, sizeof pmk_);
    secure_wipe(snonce_, sizeof snonce_);
    secure_wipe(anonce_, sizeof anonce_);
    secure_wipe(inst_anonce_, sizeof inst_anonce_);
    secure_wipe(m1_anonce_, sizeof m1_anonce_);
    secure_wipe(ptk_, sizeof ptk_);
    secure_wipe(cand_ptk_, sizeof cand_ptk_);
    secure_wipe(gtk_, sizeof gtk_);
    if (!last_reply_.empty())
      secure_wipe(last_reply_.data(), last_reply_.size());
    last_reply_.clear();
    if (!m1_reply_.empty()) secure_wipe(m1_reply_.data(), m1_reply_.size());
    m1_reply_.clear();
    m1_replay_ = 0;
    m1_answered_ = false;
    gtk_rsc_ = 0;
    ap_rsn_ = RsnInfo{};
    ap_rsn_set_ = false;
    state_ = State::Idle;
    crypto_ = nullptr;
    rx_replay_ = 0;
    rx_replay_set_ = false;
    answered_replay_ = 0;
    answered_ = false;
    answered_kind_ = Kind::None;
    ptk_valid_ = false;
    cand_valid_ = false;
    gtk_valid_ = false;
    gtk_len_ = 0;
    gtk_key_id_ = 0;
  }

  /* Feed one EAPOL frame — the bytes after the LLC/SNAP header of an 0x888E
   * data frame. */
  Verdict on_eapol(const uint8_t* eapol, size_t len,
                   std::vector<uint8_t>* out) {
    EapolKey k;

    if (!crypto_ || state_ == State::Idle) return note(Verdict::OutOfState);
    if (!parse_eapol_key(eapol, len, &k)) return note(Verdict::Malformed);
    /* Version 1 is TKIP's HMAC-MD5 and version 3 is AES-CMAC. Treating either
     * as version 2 means computing the MIC with the wrong algorithm, and the
     * only symptom would be "the MIC failed" — a diagnosis that sends the
     * reader looking at the key rather than at the cipher suite. */
    if (k.version != kKeyDescVersionCcmp) return note(Verdict::Malformed);
    /* A Request bit set is a SUPPLICANT-to-authenticator frame. Receiving one
     * means something is echoing our own traffic back at us. */
    if (k.request()) return note(Verdict::Malformed);
    /* Key data longer than an MSDU can hold is not a frame anyone sent. */
    if (k.key_data_len > kMaxKeyData) return note(Verdict::Malformed);

    const Kind kind = classify(k);
    if (kind == Kind::None) return note(Verdict::Ignored);

    /* THE REPLAY GATE, ahead of every branch below so no message type added
     * later can forget it.
     *
     * `rx_replay_` is the last counter this station AUTHENTICATED - it moves
     * in on_msg3 and on_group1 and nowhere else. Message 1 has no MIC, so
     * letting it move this would let one forged frame refuse every genuine
     * message for the rest of the association (rule 3 at the top of this
     * file).
     *
     * Strictly greater is required to install anything. Equal is answered
     * with the cached reply and installs NOTHING, which is what lets an
     * authenticator that does not increment on retransmission (802.11-2016
     * 12.7.6.4 permits either) finish its handshake; installing on one would
     * be a reinstallation (rule 2). The cached reply is returned only for the
     * SAME message type that produced it, so a msg1 replayed at a msg3's
     * counter does not get a msg4 back. */
    if (rx_replay_set_ && k.replay <= rx_replay_)
      return retransmit_or_replay(k, kind, out);
    /* A repeat of something answered but never authenticated: a retransmitted
     * message 1. It has its own cache, so it can never collect a message 4.
     *
     * A retransmission repeats the ANONCE as well as the counter, so both
     * must match. On the counter alone, a forged message 1 that arrives FIRST
     * would poison the genuine one: the genuine frame, at the same counter,
     * would be answered with the reply derived from the forger's ANonce,
     * whose MIC the AP cannot verify, and every join the forger raced would
     * fail. A same-counter message 1 with a different ANonce is processed as
     * new (with the same SNonce, which is also what wpa_supplicant does
     * within one handshake). */
    if (kind == Kind::Msg1 && m1_answered_ && k.replay == m1_replay_ &&
        std::memcmp(k.nonce, m1_anonce_, 32) == 0 && !m1_reply_.empty()) {
      retransmits++;
      if (out) *out = m1_reply_;
      return Verdict::Retransmit;
    }

    switch (kind) {
      case Kind::Msg1: return on_msg1(k, out);
      case Kind::Msg3: return on_msg3(k, out);
      case Kind::Group1: return on_group1(k, out);
      default: return note(Verdict::Ignored);
    }
  }

  State state() const { return state_; }
  bool ptk_valid() const { return ptk_valid_; }
  /* Is `k` a message 3 of the handshake already installed - same ANonce as
   * the one the live PTK was derived from? The AP retransmits exactly that
   * when our message 4 is lost. A classification only: nothing is verified
   * or installed here, and on_eapol still applies every rule to it. */
  bool is_installed_msg3(const EapolKey& k) const {
    return ptk_valid_ && k.nonce && k.pairwise() && k.ack() && k.has_mic() &&
           k.install() && k.secure() &&
           std::memcmp(k.nonce, inst_anonce_, 32) == 0;
  }
  bool gtk_valid() const { return gtk_valid_; }
  /* KCK[0:16] KEK[16:32] TK[32:48] */
  const uint8_t* ptk() const { return ptk_; }
  const uint8_t* tk() const { return ptk_ + 32; }
  const uint8_t* gtk() const { return gtk_; }
  size_t gtk_len() const { return gtk_len_; }
  uint8_t gtk_key_id() const { return gtk_key_id_; }
  /* The installed GTK's receive sequence counter, from the MIC-verified
   * message that delivered it: the Key RSC field, a 48-bit little-endian PN
   * for CCMP. 802.11-2016 12.7.6.4 has the receiver START its group replay
   * counter here rather than at whatever group frame happens to arrive
   * first - otherwise a capture from earlier in the GTK's life is accepted
   * as the first frame. Updated only when the GTK itself changes. */
  uint64_t gtk_rsc() const { return gtk_rsc_; }
  /* The last counter this station AUTHENTICATED, not the last it saw. */
  uint64_t replay_counter() const { return rx_replay_; }

  /* HOW MANY TIMES EACH KEY HAS BEEN INSTALLED, monotonic for the life of
   * this object and NOT reset by forget(). A message that re-delivers the
   * key already installed does NOT count - see the fourth defect at the top
   * of this file.
   *
   * A caller with a cipher has per-key state - packet numbers, replay
   * windows - that must restart when the key does, and "are we connected
   * now" is not the event: a PTK or GTK rekey happens with the association
   * already up and the state machine already Connected. Without this the
   * caller either misses the rekey or keeps a copy of the key to diff
   * against, and a harness holding its own copy of the pairwise key is worse
   * than a counter.
   *
   * A caller with a data plane must read both. Getting this wrong is not
   * subtle on air: the AP's new key starts at PN 1, so a stale window rejects every
   * frame until the PN climbs back within 64 of the old head - a link that
   * reports itself keyed and carries nothing. */
  uint32_t ptk_generation() const { return ptk_gen_; }
  uint32_t gtk_generation() const { return gtk_gen_; }

  uint32_t mic_failures = 0;
  uint32_t replays = 0;
  uint32_t retransmits = 0;
  uint32_t malformed = 0;
  uint32_t out_of_state = 0;
  uint32_t ignored = 0;
  uint32_t crypto_errors = 0;
  uint32_t rsn_mismatches = 0;

 private:
  /* An MSDU is 2304 bytes; key data larger than that never crossed a link. */
  static constexpr size_t kMaxKeyData = 2048;
  /* The only group cipher this station speaks is CCMP-128, and its key is 16
   * bytes. find_gtk_kde accepts up to 32 (the KDE format allows CCMP-256 and
   * GCMP-256 keys); a GTK of any other length is not one this station could
   * use: a CCMP-128 data plane would take 16 of its bytes and decrypt every
   * group frame with the wrong key.
   *
   * THE GTK'S KEY ID IS 1, 2 OR 3, never 0. A data plane picks the key by the
   * CCMP header's key id, and 0 is where the pairwise key is used: a GTK "at
   * 0" could not be told from the PTK. hostapd does not send one; a GTK KDE
   * that claims it is refused on both routes a GTK arrives by. */
  static constexpr size_t kGtkLenCcmp = 16;
  /* Message 3's Key Length is the pairwise cipher's key length: 16 for
   * CCMP-128, the only pairwise cipher this supplicant speaks (802.11-2016
   * 12.7.2). Any other value names a cipher this station did not negotiate. */
  static constexpr uint16_t kPtkTkLenCcmp = 16;

  enum class Kind : uint8_t { None, Msg1, Msg3, Group1 };

  /* Message 3 of an RSNA sets Secure (802.11-2016 12.7.6.4), exactly as the
   * group message 1 below must: a pairwise Ack+MIC+Install frame WITHOUT it
   * is not a message this supplicant answers, and it installs nothing. */
  static Kind classify(const EapolKey& k) {
    if (k.pairwise()) {
      if (k.ack() && !k.has_mic()) return Kind::Msg1;
      if (k.ack() && k.has_mic() && k.install() && k.secure())
        return Kind::Msg3;
      return Kind::None;
    }
    if (k.ack() && k.has_mic() && k.secure()) return Kind::Group1;
    return Kind::None;
  }

  Verdict note(Verdict v) {
    switch (v) {
      case Verdict::Malformed: malformed++; break;
      case Verdict::CryptoError: crypto_errors++; break;
      case Verdict::MicFailed: mic_failures++; break;
      case Verdict::Replayed: replays++; break;
      case Verdict::OutOfState: out_of_state++; break;
      case Verdict::Ignored: ignored++; break;
      default: break;
    }
    return v;
  }

  /* A failed MIC check is the frame's fault only when the MIC mismatched; a
   * failed HMAC is our provider's, and counts as crypto_errors. */
  static Verdict mic_verdict(MicCheck m) {
    return m == MicCheck::CryptoError ? Verdict::CryptoError
                                      : Verdict::MicFailed;
  }

  Verdict retransmit_or_replay(const EapolKey& k, Kind kind,
                               std::vector<uint8_t>* out) {
    if (k.replay == rx_replay_ && kind == answered_kind_ && answered_ &&
        k.replay == answered_replay_ && !last_reply_.empty()) {
      retransmits++;
      if (out) *out = last_reply_;
      return Verdict::Retransmit;
    }
    return note(Verdict::Replayed);
  }

  /* Message 1: ANonce, no MIC, nothing to verify.
   *
   * NOTHING INSTALLED IS TOUCHED, AND THE STATE DOES NOT GO BACKWARDS. The
   * PTK is derived into a CANDIDATE and the live one is left alone; a station
   * that is already Done stays Done, because on_group1 requires Done and an
   * unauthenticated frame must not be able to switch the group-rekey path
   * off. The candidate is promoted in on_msg3, after a MIC computed with it
   * verifies.
   *
   * WHAT A FORGED MESSAGE 1 STILL COSTS. There is one candidate, and anyone
   * who can spoof the BSSID can replace it with one derived from an ANonce of
   * their choosing. The genuine message 3 then carries an ANonce that matches
   * neither that candidate nor the installed handshake, so on_msg3 refuses it
   * as Malformed at the ANonce comparison, before any MIC is computed, and
   * the handshake cannot complete until the AP starts it again with a fresh
   * message 1. That is the same exposure wpa_supplicant has with
   * its single TPTK. During the initial four-way it costs an attempt. During
   * a PTK rekey on an ESTABLISHED association it can cost the link: an AP
   * whose rekey fails after its retries typically deauthenticates the
   * station. The installed PTK is never touched, so traffic keeps flowing
   * until then.
   */
  Verdict on_msg1(const EapolKey& k, std::vector<uint8_t>* out) {
    /* THE CANDIDATE CHANGES ONLY ON SUCCESS. The new PTK and message 2 are
     * built in locals and committed together; if our own CryptoOps fails
     * part-way, the previous candidate (ANonce and PTK) is left exactly as it
     * was, so a message 3 for the handshake already in flight still finds a
     * consistent candidate - never an ANonce from one handshake beside a
     * PTK from another. */
    uint8_t new_anonce[32];
    uint8_t new_ptk[48];
    std::memcpy(new_anonce, k.nonce, 32);
    if (!derive_ptk(*crypto_, pmk_, bssid_, own_, new_anonce, snonce_,
                    new_ptk)) {
      secure_wipe(new_ptk, sizeof new_ptk);
      return note(Verdict::CryptoError);
    }

    std::vector<uint8_t> rsn;
    append_rsn_ccmp_psk(rsn);
    /* The key data carries the whole RSN element, EID and length included -
     * a conforming authenticator compares it with the one in the association
     * request. */
    /* KEY LENGTH ZERO. 802.11-2016 12.7.6.3: the Key Length field of message
     * 2 is 0 in an RSNA - the pairwise key length is the AUTHENTICATOR's
     * statement, made in messages 1 and 3, and a supplicant does not repeat
     * it. The field is inside the MIC, so hostapd accepts a nonzero value and
     * an AP that checks it would not; test_against_a_real_four_way compares
     * this message 2 with wpa_supplicant's byte for byte. */
    std::vector<uint8_t> e = build_eapol_key(
        (uint16_t)(kKeyDescVersionCcmp | kKiPairwise | kKiMic), 0, k.replay,
        snonce_, nullptr, rsn.data(), rsn.size(), crypto_, new_ptk,
        kEapolVersionSupplicant);
    if (e.empty()) {
      secure_wipe(new_ptk, sizeof new_ptk);
      return note(Verdict::CryptoError);
    }

    std::memcpy(anonce_, new_anonce, 32);
    std::memcpy(cand_ptk_, new_ptk, 48);
    secure_wipe(new_ptk, sizeof new_ptk);
    cand_valid_ = true;
    std::memcpy(m1_anonce_, k.nonce, 32);
    answer(Kind::Msg1, k.replay, e);
    if (state_ == State::WaitMsg1) state_ = State::WaitMsg3;
    if (out) *out = e;
    return Verdict::Reply;
  }

  /* Message 3: the GTK, and the confirmation that the authenticator holds the
   * same PTK. Everything is checked before anything is installed. */
  Verdict on_msg3(const EapolKey& k, std::vector<uint8_t>* out) {
    /* WHICH PTK THIS MESSAGE 3 BELONGS TO. Either the CANDIDATE of a
     * handshake in flight (gated on the candidate, not on the state: a
     * message 1 that arrived on a working association leaves the state at
     * Done deliberately, and its message 3 still has to be processable), or
     * the handshake ALREADY INSTALLED - the AP retransmitting message 3
     * because our message 4 was lost, which must be answered and must not
     * reinstall anything.
     *
     * 12.7.6.4: the ANonce in message 3 must equal the one in message 1, or
     * the authenticator is not the party we derived against. Checked BEFORE
     * the MIC so a mix-and-match is refused as what it is rather than as a
     * key mismatch. */
    const uint8_t* kptk = nullptr;
    if (cand_valid_ && std::memcmp(k.nonce, anonce_, 32) == 0)
      kptk = cand_ptk_;
    else if (ptk_valid_ && std::memcmp(k.nonce, inst_anonce_, 32) == 0)
      kptk = ptk_;
    else if (!cand_valid_ && !ptk_valid_)
      return note(Verdict::OutOfState);
    else
      return note(Verdict::Malformed);

    if (k.key_len != kPtkTkLenCcmp) return note(Verdict::Malformed);

    /* THE FORGERY GATE. Verified with the KCK of the PTK chosen above, which
     * exists only because a message 1 named an ANonce and we hold the PMK. */
    const MicCheck mic = eapol_mic_ok(*crypto_, kptk, k);
    if (mic != MicCheck::Ok) return note(mic_verdict(mic));

    /* Message 3's key data is AES-key-wrapped with the KEK. An unwrap is an
     * integrity check in its own right, so a failure here is not a decode
     * hiccup — it is a frame whose MIC verified but whose key data did not,
     * which should not be possible and is refused rather than parsed. */
    GtkKde g;
    if (!k.encrypted() || k.key_data_len == 0) {
      /* Unencrypted key data in message 3 is a downgrade - the GTK would be
       * in the clear - and message 3 with no key data at all carries no GTK,
       * which in RSN it always does. Both are refused rather than completed
       * into a station that is keyed with no group key.
       *
       * DELETING THIS CHECK CHANGES NO OUTCOME, and that is recorded rather
       * than hidden: a mutation removing it survives the whole test suite,
       * because plaintext key data fails the AES unwrap below on its
       * integrity check and an absent one fails the length check. It stays
       * because "the unwrap happened to refuse it" is a different reason from
       * "we do not accept an unprotected GTK", and only one of those survives
       * a future edit to the unwrap path. */
      return note(Verdict::Malformed);
    }
    if (k.key_data_len < 16 || (k.key_data_len % 8) != 0)
      return note(Verdict::Malformed);
    std::vector<uint8_t> plain(k.key_data_len - 8);
    /* Wiped on EVERY exit, the failed unwrap included: an implementation may
     * have written some plaintext before its integrity check failed. */
    if (!crypto_->aes_key_unwrap(kptk + 16, 16, k.key_data, k.key_data_len,
                                 plain.data())) {
      secure_wipe(plain.data(), plain.size());
      return note(Verdict::Malformed);
    }
    /* Absent and Malformed are both refusals HERE - see the note at
     * KdeResult. A message 3 with no GTK would otherwise complete the
     * handshake into a station that can decrypt no broadcast at all, with
     * nothing counted and nothing to look at. */
    if (find_gtk_kde(plain.data(), plain.size(), &g) != KdeResult::Found ||
        g.gtk_len != kGtkLenCcmp || g.key_id == 0) {
      secure_wipe(plain.data(), plain.size());
      secure_wipe(&g, sizeof g);
      return note(Verdict::Malformed);
    }
    /* THE DOWNGRADE CHECK (12.7.6.4). The RSN element in this MIC-verified,
     * KEK-wrapped key data is the AP's own; the one we chose the BSS by came
     * off the air unauthenticated. They must agree, or someone rewrote the
     * advertisement - refused before anything is installed. */
    if (ap_rsn_set_) {
      RsnInfo got;
      if (!find_rsn_element(plain.data(), plain.size(), &got) ||
          !rsn_equivalent(got, ap_rsn_)) {
        secure_wipe(plain.data(), plain.size());
        secure_wipe(&g, sizeof g);
        rsn_mismatches++;
        return note(Verdict::Malformed);
      }
    }

    std::vector<uint8_t> e = build_eapol_key(
        (uint16_t)(kKeyDescVersionCcmp | kKiPairwise | kKiMic | kKiSecure), 0,
        k.replay, nullptr, nullptr, nullptr, 0, crypto_, kptk,
        kEapolVersionSupplicant);
    if (e.empty()) {
      secure_wipe(plain.data(), plain.size());
      secure_wipe(&g, sizeof g);
      return note(Verdict::CryptoError);
    }

    /* INSTALL LAST. Up to here a failure has cost nothing.
     *
     * AND NEVER REINSTALL (the fourth defect at the top of this file). The
     * comparison is on the key bytes, not on which path chose `kptk`: a
     * message 1 re-quoting the installed ANonce re-derives the identical PTK
     * into the candidate, and its message 3 must not count as a new key
     * either. */
    if (!ptk_valid_ || std::memcmp(kptk, ptk_, 48) != 0) {
      std::memcpy(ptk_, kptk, 48);
      std::memcpy(inst_anonce_, k.nonce, 32);
      ptk_valid_ = true;
      ptk_gen_++;
    }
    /* The candidate is spent: only a fresh message 1 re-arms it. */
    if (kptk == cand_ptk_) {
      secure_wipe(cand_ptk_, sizeof cand_ptk_);
      cand_valid_ = false;
    }
    install_gtk(g, k.rsc);
    authenticated(k.replay);
    answer(Kind::Msg3, k.replay, e);
    state_ = State::Done;
    secure_wipe(plain.data(), plain.size());
    secure_wipe(&g, sizeof g);
    if (out) *out = e;
    return Verdict::Reply;
  }

  /* Group key handshake, message 1: a new GTK under the KEK, MIC'd with the
   * KCK. Needs an installed PTK; one arriving before the 4-way has finished is
   * out of state, not merely unverifiable. */
  Verdict on_group1(const EapolKey& k, std::vector<uint8_t>* out) {
    if (state_ != State::Done || !ptk_valid_)
      return note(Verdict::OutOfState);
    const MicCheck mic = eapol_mic_ok(*crypto_, ptk_, k);
    if (mic != MicCheck::Ok) return note(mic_verdict(mic));
    if (!k.encrypted() || k.key_data_len < 16 || (k.key_data_len % 8) != 0)
      return note(Verdict::Malformed);

    std::vector<uint8_t> plain(k.key_data_len - 8);
    if (!crypto_->aes_key_unwrap(ptk_ + 16, 16, k.key_data, k.key_data_len,
                                 plain.data())) {
      secure_wipe(plain.data(), plain.size());
      return note(Verdict::Malformed);
    }
    GtkKde g;
    if (find_gtk_kde(plain.data(), plain.size(), &g) != KdeResult::Found ||
        g.gtk_len != kGtkLenCcmp || g.key_id == 0) {
      secure_wipe(plain.data(), plain.size());
      secure_wipe(&g, sizeof g);
      return note(Verdict::Malformed);
    }

    std::vector<uint8_t> e = build_eapol_key(
        (uint16_t)(kKeyDescVersionCcmp | kKiMic | kKiSecure), 0, k.replay,
        nullptr, nullptr, nullptr, 0, crypto_, ptk_, kEapolVersionSupplicant);
    if (e.empty()) {
      secure_wipe(plain.data(), plain.size());
      secure_wipe(&g, sizeof g);
      return note(Verdict::CryptoError);
    }

    install_gtk(g, k.rsc);
    authenticated(k.replay);
    answer(Kind::Group1, k.replay, e);
    secure_wipe(plain.data(), plain.size());
    secure_wipe(&g, sizeof g);
    if (out) *out = e;
    return Verdict::Reply;
  }

  /* The first RSN element (EID 48) in unwrapped key data, read with
   * walk_key_data (Eapol.h) - the same walk find_gtk_kde uses, padding
   * included. Elements before it are stepped over; a truncation before it,
   * or no RSN element at all, is "not found". */
  static bool find_rsn_element(const uint8_t* kd, size_t len, RsnInfo* out) {
    bool found = false, ok = false;

    walk_key_data(kd, len, [&](uint8_t id, const uint8_t* b, size_t l) {
      if (id != 48) return true;
      found = true;
      ok = parse_rsn(b, l, out) && out->valid;
      return false;
    });
    return found && ok;
  }

  /* 802.11-2016 12.7.6.4: message 3's RSN element must match the one the BSS
   * was chosen by. Compared PARSED, field by field - version, group cipher,
   * the full pairwise and AKM suite SETS, capabilities (exactly), and the
   * group management cipher - which is the shape of wpa_supplicant's
   * wpa_compare_rsn_ie() once its memcmp has failed. A byte-identical element
   * therefore always matches, and so does one listing the same suites in a
   * different order: the standard orders nothing within a list, and an AP may
   * encode the same element differently in its beacon and its message 3.
   *
   * THE SETS, NOT THE COUNTS. Comparing only "CCMP is among the pairwise
   * suites" plus the suite counts would judge {CCMP, GCMP-128} advertised
   * against {CCMP, TKIP} in message 3 - one suite swapped for a weaker one -
   * equivalent. The counts are compared as well, so a duplicated suite is a
   * difference too.
   *
   * Suites outside 00-0F-AC (vendor) are compared by presence only - which
   * is stricter than wpa_supplicant, whose bitfields drop them entirely. */
  static bool rsn_equivalent(const RsnInfo& a, const RsnInfo& b) {
    return a.version == b.version && a.group_suite == b.group_suite &&
           a.pairwise_mask == b.pairwise_mask &&
           a.pairwise_other == b.pairwise_other &&
           a.pairwise_count == b.pairwise_count && a.akm_mask == b.akm_mask &&
           a.akm_other == b.akm_other && a.akm_count == b.akm_count &&
           a.capabilities == b.capabilities &&
           a.group_mgmt_suite == b.group_mgmt_suite;
  }

  /* The GTK already in use is NOT reinstalled - not copied, not counted, and
   * its RSC is not re-read. Every PTK rekey's message 3 re-delivers the
   * current GTK, and a caller that reopens its group replay window on a
   * generation change would otherwise reopen it on every rekey, with no
   * attacker involved (CVE-2017-13078/13080 class). */
  void install_gtk(const GtkKde& g, const uint8_t* rsc) {
    if (gtk_valid_ && gtk_len_ == g.gtk_len && gtk_key_id_ == g.key_id &&
        std::memcmp(gtk_, g.gtk, g.gtk_len) == 0)
      return;
    std::memcpy(gtk_, g.gtk, g.gtk_len);
    gtk_len_ = g.gtk_len;
    gtk_key_id_ = g.key_id;
    gtk_rsc_ = 0;
    if (rsc)
      for (int i = 0; i < 6; i++) gtk_rsc_ |= (uint64_t)rsc[i] << (8 * i);
    gtk_valid_ = true;
    gtk_gen_++;
  }

  /* THE ONLY PLACE rx_replay_ MOVES, and both call sites have verified a MIC
   * before reaching it. */
  void authenticated(uint64_t replay) {
    rx_replay_ = replay;
    rx_replay_set_ = true;
  }

  /* The retransmission caches, kept with the message type they answered so a
   * different message quoting the same counter cannot collect them. Message
   * 1's answer is cached APART from the authenticated one: an
   * unauthenticated message 1 must not be able to evict the message 4 that
   * the AP's equal-counter retransmission of message 3 is owed. */
  void answer(Kind kind, uint64_t replay, const std::vector<uint8_t>& reply) {
    if (kind == Kind::Msg1) {
      m1_replay_ = replay;
      m1_answered_ = true;
      m1_reply_ = reply;
      return;
    }
    answered_kind_ = kind;
    answered_replay_ = replay;
    answered_ = true;
    last_reply_ = reply;
  }

  CryptoOps* crypto_ = nullptr;
  State state_ = State::Idle;
  uint8_t pmk_[32] = {0};
  uint8_t own_[6] = {0};
  uint8_t bssid_[6] = {0};
  uint8_t snonce_[32] = {0};
  uint8_t anonce_[32] = {0};       /* the in-flight candidate's ANonce */
  uint8_t inst_anonce_[32] = {0};  /* the installed PTK's ANonce */
  uint8_t ptk_[48] = {0};
  uint8_t cand_ptk_[48] = {0};
  uint8_t gtk_[32] = {0};
  size_t gtk_len_ = 0;
  uint8_t gtk_key_id_ = 0;
  uint64_t gtk_rsc_ = 0;
  RsnInfo ap_rsn_{};         /* the advertisement message 3 must match */
  bool ap_rsn_set_ = false;
  uint32_t ptk_gen_ = 0;
  uint32_t gtk_gen_ = 0;
  bool ptk_valid_ = false;
  bool cand_valid_ = false;
  bool gtk_valid_ = false;
  uint64_t rx_replay_ = 0;
  bool rx_replay_set_ = false;
  uint64_t answered_replay_ = 0;
  bool answered_ = false;
  Kind answered_kind_ = Kind::None;
  std::vector<uint8_t> last_reply_;
  uint64_t m1_replay_ = 0;
  uint8_t m1_anonce_[32] = {0};   /* the ANonce the cached reply answered */
  bool m1_answered_ = false;
  std::vector<uint8_t> m1_reply_;
};

}  // namespace sta
}  // namespace devourer

#endif /* DEVOURER_STA_SUPPLICANT_H */
