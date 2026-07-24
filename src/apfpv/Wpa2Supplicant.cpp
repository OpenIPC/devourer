// ============================================================================
//  Wpa2Supplicant.cpp — WPA2-PSK 4-way handshake + CCMP (impl)
//  Declarations in Wpa2Supplicant.h. Pure userspace protocol logic; crypto via
//  injected Crypto backend (Wpa2Crypto.cpp). Not timing-critical.
// ============================================================================
#include "Wpa2Supplicant.h"
#include "Dot11Frames.h"
#include "crypto/AesCcm.h"
#include "crypto/AesCmac.h"
#include <cstdlib>
#include <cstdio>
#include <cstring>
#if defined(__ANDROID__)
#include <android/log.h>
#define WPALOG(...) __android_log_print(ANDROID_LOG_INFO, "apfpv-scan", __VA_ARGS__)
#else
#define WPALOG(...) do{}while(0)
#endif
#include <android/log.h>

#ifdef _WIN32
extern "C" __declspec(dllimport) long __stdcall
    BCryptGenRandom(void* hAlg, unsigned char* pb, unsigned long cb, unsigned long flags);
#endif
// Cryptographically-secure RNG for the SNonce (must be unpredictable): Windows CNG
// (BCryptGenRandom, system-preferred DRBG), else /dev/urandom (Linux/Android), else a
// weak last-resort mix. Replaces the bare std::rand() Windows fallback.
static void secure_random(uint8_t* buf, size_t n) {
#ifdef _WIN32
    if (BCryptGenRandom(nullptr, buf, (unsigned long)n, 0x00000002u /*USE_SYSTEM_PREFERRED_RNG*/) == 0) return;
#else
    FILE* u = fopen("/dev/urandom", "rb");
    if (u) { size_t r = fread(buf, 1, n, u); fclose(u); if (r == n) return; }
#endif
    for (size_t i = 0; i < n; ++i) buf[i] = (uint8_t)((std::rand() >> 3) ^ (i * 131));
}

namespace apfpv {

Wpa2Supplicant::Wpa2Supplicant(Crypto c, SendFn send)
    : _c(std::move(c)), _send(std::move(send)) {}

void Wpa2Supplicant::begin(const std::string& passphrase, const std::string& ssid,
                           const Mac& self, const Mac& bssid) {
    _self = self; _bssid = bssid;
    _pmk = _c.pbkdf2_pmk(passphrase,
                         reinterpret_cast<const uint8_t*>(ssid.data()), ssid.size());
    _state = State::WaitMsg1;
    // fresh session: clear replay/PN windows + GTK slots so a reconnect can't be falsely
    // rejected by stale counters from the previous association.
    _replay = 0; _txPn = 1; _rxPnPair = 0;
    _rxPnGtk[0]=_rxPnGtk[1]=_rxPnGtk[2]=_rxPnGtk[3]=0;
    winClear(_rxPnPairWin); for(int _k=0;_k<4;_k++) winClear(_rxPnGtkWin[_k]);
    _gtkKeyId = 0xff; _gtkPrevId = 0xff;
}

void Wpa2Supplicant::beginCached(const std::array<uint8_t,32>& pmk,
                                 const Mac& self, const Mac& bssid) {
    _self = self; _bssid = bssid;
    _pmk = pmk;                 // pre-computed — no PBKDF2 here (keeps the M1 window)
    _state = State::WaitMsg1;
    // fresh session: clear replay/PN windows + GTK slots so a reconnect can't be falsely
    // rejected by stale counters from the previous association.
    _replay = 0; _txPn = 1; _rxPnPair = 0;
    _rxPnGtk[0]=_rxPnGtk[1]=_rxPnGtk[2]=_rxPnGtk[3]=0;
    winClear(_rxPnPairWin); for(int _k=0;_k<4;_k++) winClear(_rxPnGtkWin[_k]);
    _gtkKeyId = 0xff; _gtkPrevId = 0xff;
}

bool Wpa2Supplicant::ready() const { return _state == State::Done; }

void Wpa2Supplicant::derivePtk(const uint8_t* aNonce) {
    std::memcpy(_aNonce.data(), aNonce, 32);
    // SNonce: cryptographically-secure + unpredictable (CNG on Windows, /dev/urandom on
    // Linux/Android). MUST be fresh per 4-way (incl. PTK rekey) or PTK derivation is predictable.
    secure_random(_sNonce.data(), 32);
    uint8_t data[76];
    const Mac& aa = _bssid; const Mac& sa = _self;
    bool aaFirst = std::memcmp(aa.data(), sa.data(), 6) < 0;
    std::memcpy(data + 0, (aaFirst?aa:sa).data(), 6);
    std::memcpy(data + 6, (aaFirst?sa:aa).data(), 6);
    bool aFirst = std::memcmp(aNonce, _sNonce.data(), 32) < 0;
    std::memcpy(data + 12, aFirst?aNonce:_sNonce.data(), 32);
    std::memcpy(data + 44, aFirst?_sNonce.data():aNonce, 32);
    auto ptk = _c.prf(_pmk.data(), 32, "Pairwise key expansion", data, sizeof(data), 48);
    std::memcpy(_kck.data(), ptk.data() + 0,  16);
    std::memcpy(_kek.data(), ptk.data() + 16, 16);
    std::memcpy(_tk.data(),  ptk.data() + 32, 16);
}

// Build an EAPOL-Key body (key info, nonce, MIC). MIC computed over the body
// with the MIC field zeroed, using KCK, then patched in. Returns full 802.11
// data frame via BuildEapolKey.
std::vector<uint8_t> Wpa2Supplicant::buildMsg2() {
    // Canonical 802.1X EAPOL-Key frame (WPA2/RSN, Key Descriptor Version 2 = AES):
    //   [0]      EAPOL Protocol Version (0x02)
    //   [1]      EAPOL Packet Type      (0x03 = EAPOL-Key)
    //   [2-3]    EAPOL Body Length (BE) = frame length - 4
    //   [4]      Descriptor Type        (0x02 = RSN)
    //   [5-6]    Key Information (BE)
    //   [7-8]    Key Length (BE)
    //   [9-16]   Key Replay Counter (echo the AP's)
    //   [17-48]  Key Nonce (SNonce)
    //   [49-64]  EAPOL Key IV (0) | [65-72] Key RSC (0) | [73-80] Reserved (0)
    //   [81-96]  Key MIC (zeroed for the HMAC, then patched in)
    //   [97-98]  Key Data Length (BE)
    //   [99+]    Key Data = STA RSN IE (MUST match the Assoc-Req RSN IE)
    // The old code omitted the 4-byte EAPOL header + the RSN-IE Key Data and
    // computed the MIC over that non-canonical buffer, so the AP's MIC check
    // always failed and it never sent M3.
    static const uint8_t rsnIe[] = {            // == Dot11Frames BuildAssocRequest RSN IE
        0x30,0x14, 0x01,0x00, 0x00,0x0f,0xac,0x04,
        0x01,0x00, 0x00,0x0f,0xac,0x04, 0x01,0x00, 0x00,0x0f,0xac,0x02, 0x00,0x00 };
    const size_t kdLen = sizeof(rsnIe);         // 22
    std::vector<uint8_t> kb(99 + kdLen, 0);
    kb[0] = 0x02; kb[1] = 0x03;
    uint16_t bodyLen = (uint16_t)(kb.size() - 4);
    kb[2] = bodyLen >> 8; kb[3] = bodyLen & 0xFF;
    kb[4] = 0x02;                               // descriptor type = RSN
    uint16_t info = 0x010A;                     // ver2(AES) + pairwise + MIC
    kb[5] = info >> 8; kb[6] = info & 0xFF;
    kb[7] = 0x00; kb[8] = 0x10;                 // key length = 16 (CCMP pairwise)
    std::memcpy(kb.data() + 9, _replayCtr, 8);  // echo the AP's replay counter
    std::memcpy(kb.data() + 17, _sNonce.data(), 32);
    kb[97] = (uint8_t)(kdLen >> 8); kb[98] = (uint8_t)(kdLen & 0xFF);
    std::memcpy(kb.data() + 99, rsnIe, kdLen);
    { uint16_t caps = (uint16_t)((_pmf>=1?0x0080:0)|(_pmf>=2?0x0040:0));   // must match the assoc-req RSN caps
      kb[99 + kdLen - 2] = (uint8_t)(caps & 0xff); kb[99 + kdLen - 1] = (uint8_t)(caps >> 8); }
    // MIC over the WHOLE frame with the MIC field [81..96] zeroed.
    auto mic = _c.eapol_mic(_kck.data(), kb.data(), kb.size());
    std::memcpy(kb.data() + 81, mic.data(), 16);
    return BuildEapolKey(_self, _bssid, kb);
}

std::vector<uint8_t> Wpa2Supplicant::buildMsg4() {
    // Canonical EAPOL-Key, no Key Data, no SNonce, secure bit set. Same layout as M2.
    std::vector<uint8_t> kb(99, 0);
    kb[0] = 0x02; kb[1] = 0x03;
    uint16_t bodyLen = (uint16_t)(kb.size() - 4);
    kb[2] = bodyLen >> 8; kb[3] = bodyLen & 0xFF;
    kb[4] = 0x02;                               // descriptor type = RSN
    uint16_t info = 0x030A;                     // ver2 + pairwise + MIC + secure
    kb[5] = info >> 8; kb[6] = info & 0xFF;
    // key length 0, nonce 0, key data length 0 for M4.
    std::memcpy(kb.data() + 9, _replayCtr, 8);  // echo M3's replay counter
    auto mic = _c.eapol_mic(_kck.data(), kb.data(), kb.size());
    std::memcpy(kb.data() + 81, mic.data(), 16);
    return BuildEapolKey(_self, _bssid, kb);
}

// Unwrap the EAPOL-Key Data (NIST AES-key-wrap with the KEK) and install the GTK from its
// KDE. Shared by M3 (initial 4-way) AND the periodic group rekey. Tracks the GTK key id.
bool Wpa2Supplicant::installGtkFromKeyData(const uint8_t* body, size_t len) {
    uint16_t kdLen = (len >= 99) ? ((body[97] << 8) | body[98]) : 0;
    if (kdLen < 24 || (kdLen % 8) != 0 || len < (size_t)(99 + kdLen)) return false;
    std::vector<uint8_t> kp(kdLen, 0);
    if (!crypto::aes_key_unwrap(_kek.data(), body + 99, kdLen, kp.data())) return false;
    size_t plen = kdLen - 8;
    bool gotGtk = false;
    for (size_t i = 0; i + 2 <= plen; ) {
        uint8_t t = kp[i], l = kp[i+1];
        if (t == 0 || l == 0) break;
        if (t == 0xdd && (size_t)(i + 2 + l) <= plen &&
            kp[i+2]==0x00 && kp[i+3]==0x0f && kp[i+4]==0xac) {
            if (kp[i+5]==0x01 && l >= 22) {        // GTK KDE: <keyid(low2)|tx><rsv> <GTK16..>
                uint8_t newId = kp[i+6] & 0x03;
                if (_gtkKeyId != 0xff && _gtkKeyId != newId) { _gtkPrev = _gtk; _gtkPrevId = _gtkKeyId; }
                _gtkKeyId = newId;
                std::memcpy(_gtk.data(), &kp[i+8], 16);
                _rxPnGtk[newId & 3] = 0; winClear(_rxPnGtkWin[newId & 3]);  // fresh key: reset RX replay window
                fprintf(stderr, "[wpa] GTK installed keyid=%d (%02x%02x..)\n", _gtkKeyId, _gtk[0], _gtk[1]);
                gotGtk = true;
            } else if (kp[i+5]==0x09 && l >= 24) {  // IGTK KDE (802.11w): <keyid LE(2)><ipn(6)> <IGTK16>
                _igtkKeyId = (uint16_t)(kp[i+6] | (kp[i+7] << 8));
                std::memcpy(_igtk.data(), &kp[i+14], 16);
                _igtkSet = true;
                fprintf(stderr, "[wpa] IGTK installed keyid=%u\n", _igtkKeyId);
            }
        }
        i += 2 + l;
    }
    return gotGtk;
}

// Group-rekey reply (M2 of the 2-way group handshake): Key Type=Group, MIC+Secure set,
// no Key Data, echo the AP's replay counter, MIC over the frame with the KCK.
std::vector<uint8_t> Wpa2Supplicant::buildGroupMsg2() {
    std::vector<uint8_t> kb(99, 0);
    kb[0] = 0x02; kb[1] = 0x03;
    uint16_t bodyLen = (uint16_t)(kb.size() - 4);
    kb[2] = bodyLen >> 8; kb[3] = bodyLen & 0xFF;
    kb[4] = 0x02;                               // descriptor type = RSN
    uint16_t info = 0x0302;                     // ver2(AES) + GROUP(pairwise=0) + MIC + secure
    kb[5] = info >> 8; kb[6] = info & 0xFF;
    kb[7] = 0x00; kb[8] = 0x10;                 // key length = 16 (CCMP group)
    std::memcpy(kb.data() + 9, _replayCtr, 8);
    auto mic = _c.eapol_mic(_kck.data(), kb.data(), kb.size());
    std::memcpy(kb.data() + 81, mic.data(), 16);
    return BuildEapolKey(_self, _bssid, kb);
}

// 802.11w BIP-CMAC-128 verify of a protected broadcast/multicast Deauth/Disassoc. True only if
// the trailing MME's MIC == AES-CMAC(IGTK, maskedFC|A1|A2|A3|body-with-MME-MIC-zeroed). Per
// 802.11-2016 §12.5.4; best-effort (validate against a real PMF AP). On any mismatch we return
// false and the caller ignores the frame — the safe default (a genuine deauth still trips the
// RX-silence watchdog), so an imperfect BIP can never cause a spurious disconnect.
bool Wpa2Supplicant::verifyProtectedMgmt(const uint8_t* frame, size_t len) const {
    if (!_igtkSet || len < 24 + 18) return false;
    size_t mmeOff = len - 18;                          // MME = 0x4c len(16) keyid(2) ipn(6) mic(8)
    if (frame[mmeOff] != 0x4c || frame[mmeOff+1] != 16) return false;
    const uint8_t* mme = frame + mmeOff;
    if ((uint16_t)(mme[2] | (mme[3] << 8)) != _igtkKeyId) return false;
    std::vector<uint8_t> buf; buf.reserve(14 + (len - 24));
    uint16_t mfc = (uint16_t)((frame[0] | (frame[1] << 8)) & ~0x7800); // mask Retry/PwrMgmt/MoreData/Protected
    buf.push_back(mfc & 0xff); buf.push_back(mfc >> 8);
    buf.insert(buf.end(), frame + 4, frame + 16);      // A1 | A2 | A3 (duration excluded)
    buf.insert(buf.end(), frame + 24, frame + len);    // body incl. the MME
    for (int k = 0; k < 8; ++k) buf[buf.size() - 8 + k] = 0;   // zero the MME MIC for the calc
    uint8_t mic[8]; crypto::bip_cmac_128(_igtk.data(), buf.data(), buf.size(), mic);
    return std::memcmp(mic, mme + 10, 8) == 0;
}

bool Wpa2Supplicant::onEapolKey(const uint8_t* body, size_t len) {
    WPALOG("[wpa] onEapolKey len=%zu state=%d", len, (int)_state);
    if (len < 95) { WPALOG("[wpa] EAPOL too short (len=%zu <95) — dropped", len); return false; }
    // EAPOL anti-replay: the AP's 8-byte replay counter (offset 9) must not go backwards once
    // the handshake is up — rejects replayed EAPOL-Key frames (KRACK-class). Only enforced past
    // Done so a reconnect's fresh (possibly lower) M1 counter is never rejected; begin() resets.
    uint64_t rc = 0; for (int i = 0; i < 8; ++i) rc = (rc << 8) | body[9 + i];
    if (_state == State::Done && rc < _replay) {
        fprintf(stderr, "[wpa] EAPOL-Key replay dropped (rc < %llu)\n", (unsigned long long)_replay);
        return false;
    }
    if (rc > _replay) _replay = rc;
    // Capture so our M2/M4 echo the AP's counter (else the AP rejects them as replay-mismatched).
    std::memcpy(_replayCtr, body + 9, 8);
    const uint8_t* keyInfo = body + 5;
    uint16_t info = (keyInfo[0] << 8) | keyInfo[1];
    const bool hasMic     = info & 0x0100;
    const bool isPairwise = info & 0x0008;
    const uint8_t* nonce  = body + 17;
    fprintf(stderr, "[wpa] EAPOL-Key rx len=%zu info=0x%04x mic=%d pairwise=%d state=%d\n",
            len, info, (int)hasMic, (int)isPairwise, (int)_state);
    WPALOG("[wpa] EAPOL-Key info=0x%04x mic=%d pairwise=%d state=%d", info, (int)hasMic, (int)isPairwise, (int)_state);
    switch (_state) {
        case State::WaitMsg1:
            if (!hasMic && isPairwise) {
                derivePtk(nonce);
                bool sent = _send(buildMsg2());
                _state = State::WaitMsg3;
                fprintf(stderr, "[wpa] M1 -> built+sent M2 (sent=%d) -> WaitMsg3\n", (int)sent);
                WPALOG("[wpa] M1 rx -> M2 sent=%d -> WaitMsg3", (int)sent);
            } else {
                WPALOG("[wpa] WaitMsg1 but frame not M1 (mic=%d pairwise=%d) — ignored", (int)hasMic, (int)isPairwise);
            }
            return false;
        case State::WaitMsg3:
            if (hasMic && isPairwise) {
                // M3 carries the new GTK as a KEK-wrapped KDE in the Key Data field.
                installGtkFromKeyData(body, len);
                bool sent4 = _send(buildMsg4());
                _state = State::Done;
                fprintf(stderr, "[wpa] M3 -> built+sent M4 (sent=%d) -> DONE\n", (int)sent4);
                WPALOG("[wpa] M3 rx -> M4 sent=%d -> DONE (keys installed)", (int)sent4);
                return true;
            }
            return false;
        default:
            // POST-HANDSHAKE REKEYS — a real station MUST service these or its keys go
            // stale and decryption silently dies mid-session (the ~10-min "stream stops" bug).
            if (hasMic && !isPairwise) {                 // GROUP rekey (2-way): AP rotates the GTK
                bool ok = installGtkFromKeyData(body, len);
                bool sent = _send(buildGroupMsg2());
                fprintf(stderr, "[wpa] GROUP REKEY: GTK %s, M2 sent=%d\n", ok?"reinstalled":"PARSE-FAIL", (int)sent);
                return ok;
            }
            if (!hasMic && isPairwise) {                 // PTK rekey: AP restarts the 4-way
                derivePtk(nonce);
                _txPn = 1; _rxPnPair = 0; winClear(_rxPnPairWin);  // fresh pairwise key -> fresh PN windows
                _send(buildMsg2());
                _state = State::WaitMsg3;
                fprintf(stderr, "[wpa] PTK REKEY: re-derived PTK, sent M2 -> WaitMsg3\n");
                return false;
            }
            return false;
    }
}

// Build the CCMP AAD per IEEE 802.11: masked FC (clear subtype b4-6 + Retry/PwrMgt/MoreData,
// set Protected) | A1 | A2 | A3 | masked SC (fragment only, sequence zeroed) [ | QoS TID ].
// The DURATION field is EXCLUDED. The old code passed the raw 24-byte header (duration in,
// FC/SC unmasked) so the MIC never matched and NO encrypted frame decrypted.
static size_t ccmpAad(const uint8_t* hdr, bool qos, uint8_t* aad) {
    uint16_t fc = hdr[0] | (hdr[1] << 8);
    uint16_t mfc = (fc & ~0x3870) | 0x4000;
    aad[0] = mfc & 0xff; aad[1] = (mfc >> 8) & 0xff;
    std::memcpy(aad + 2, hdr + 4, 18);              // A1 | A2 | A3  (skip duration hdr[2:4])
    aad[20] = hdr[22] & 0x0f; aad[21] = 0x00;       // SC: fragment number only
    if (qos) { aad[22] = hdr[24] & 0x0f; aad[23] = 0x00; return 24; }
    return 22;
}

bool Wpa2Supplicant::decryptData(const uint8_t* frame, size_t len, std::vector<uint8_t>& plain) {
    if (_state != State::Done) return false;
    if (len < 24 + 8) return false;
    uint16_t fc = frame[0] | (frame[1] << 8);
    size_t hdr = 24;
    if (fc & 0x0080) hdr += 2;                 // QoS-data (subtype bit, NOT the Order flag
                                               // 0x8000 — same bug that hid the handshake)
    const uint8_t* ccmp = frame + hdr;         // 8-byte CCMP header
    bool qos = (fc & 0x0080) != 0;
    // CCMP nonce: priority(1)=QoS TID (0 if non-QoS) | A2(6) | PN(6).
    uint8_t pn[6] = { ccmp[7], ccmp[6], ccmp[5], ccmp[4], ccmp[1], ccmp[0] };
    uint8_t nonce[13]; nonce[0] = qos ? (frame[24] & 0x0f) : 0;
    std::memcpy(nonce + 1, frame + 10, 6);     // A2
    std::memcpy(nonce + 7, pn, 6);
    const uint8_t* enc = ccmp + 8;
    size_t encLen = len - hdr - 8;
    // Proper CCMP AAD (masked, duration excluded). Group-addressed (broadcast/multicast:
    // DHCP OFFER, ARP, multicast video) decrypts with the GTK; unicast with the pairwise TK.
    uint8_t aad[30]; size_t aadLen = ccmpAad(frame, qos, aad);
    // Group frames pick a GTK by the CCMP KeyID (two slots survive a rekey); unicast uses the TK.
    uint8_t kid = (ccmp[3] >> 6) & 0x03;
    const uint8_t* key;
    if (frame[4] & 0x01) key = (kid == _gtkPrevId) ? _gtkPrev.data() : _gtk.data();
    else                 key = _tk.data();
    // The RX frame carries a trailing 4-byte 802.11 FCS (RCR APPFCS). It sits AFTER the CCMP MIC,
    // so the FCS-INCLUSIVE length ALWAYS fails the MIC check. PERF FIX (2026-06-21): the old code
    // tried the full length FIRST and only retried with -4 on failure — doing a WASTED full
    // CTR+CBC-MAC decrypt on EVERY packet (the single RX worker was 93% CPU-bound, ~22Mbps cap,
    // ~580µs/pkt). APPFCS is always on, so try the FCS-stripped length FIRST; the full-length
    // fallback only runs if that fails (no-FCS edge case). Halves the per-packet AES work.
    bool ok = (encLen > 12) && _c.ccmp_decrypt(key, nonce, enc, encLen - 4, aad, aadLen, plain);
    if (!ok) ok = _c.ccmp_decrypt(key, nonce, enc, encLen, aad, aadLen, plain);
    if (!ok) {
        // MIC/CCM failure = wrong key (stale PTK/GTK after reconnect) or corrupt frame.
        static int micFails = 0;
        if ((++micFails % 200) == 1)
            __android_log_print(ANDROID_LOG_WARN, "rxd-decfail",
                "MIC-fail #%d grp=%d kid=%d (wrong key / corrupt)", micFails, (int)(frame[4]&1), (int)kid);
        return false;
    }
    // RX anti-replay (post-MIC) with a 64-PN SLIDING WINDOW. A-MPDU subframes legitimately
    // arrive out-of-order within the Block-Ack window (bufsz=64), so the old strict
    // "pn <= lastPn -> drop" rejected ~53% of valid reordered subframes (the real A-MPDU
    // throughput killer once the AP started aggregating). The window accepts any in-window,
    // not-yet-seen PN — matching how the chip/kernel do CCMP replay under aggregation.
    uint64_t pnVal = ((uint64_t)pn[0]<<40)|((uint64_t)pn[1]<<32)|((uint64_t)pn[2]<<24)
                   | ((uint64_t)pn[3]<<16)|((uint64_t)pn[4]<<8)|(uint64_t)pn[5];
    uint64_t& highest = (frame[4]&0x01) ? _rxPnGtk[kid]    : _rxPnPair;
    uint64_t* win     = (frame[4]&0x01) ? _rxPnGtkWin[kid] : _rxPnPairWin;
    if (!pnWindowCheck(highest, win, pnVal)) {
        // True duplicate (an unacked-A-MPDU retransmit) or a PN below the 64-frame window.
        static int pnReplays = 0;
        if ((++pnReplays % 500) == 1)
            __android_log_print(ANDROID_LOG_WARN, "rxd-decfail",
                "PN-replay #%d grp=%d kid=%d pn=%llu high=%llu (dup or out-of-window)",
                pnReplays, (int)(frame[4]&1), (int)kid,
                (unsigned long long)pnVal, (unsigned long long)highest);
        return false;
    }
    return true;
}


// Build an encrypted (CCMP) 802.11 QoS-data frame carrying an IPv4 payload.
// Frame: [802.11 hdr (to-DS)][CCMP hdr 8B][ enc{ LLC/SNAP | IPv4 } ][MIC 8B].
std::vector<uint8_t> Wpa2Supplicant::buildEncryptedData(const uint8_t* ip, size_t iplen, uint16_t ethertype) {
    if (_state != State::Done) return {};
    // LLC/SNAP + payload = the plaintext to encrypt (ethertype 0x0800 IPv4, 0x0806 ARP, ...)
    std::vector<uint8_t> plain;
    uint8_t snap[8] = {0xaa,0xaa,0x03,0x00,0x00,0x00,(uint8_t)(ethertype>>8),(uint8_t)(ethertype&0xff)};
    plain.insert(plain.end(), snap, snap+8);
    plain.insert(plain.end(), ip, ip+iplen);

    // addr3 = the L2 destination (DA). A broadcast/multicast IP MUST map to the matching L2
    // group address; otherwise the AP receives an IP broadcast that is L2-unicast to itself
    // and does NOT relay it to the DS/DHCP server — so a DHCP DISCOVER (dst 255.255.255.255)
    // never reaches dnsmasq and no lease is offered. Unicast IP goes via the AP (A3=BSSID).
    uint8_t a3[6];
    if (ethertype != 0x0800)                                        // ARP etc. -> L2 broadcast
        std::memset(a3, 0xff, 6);
    else if (iplen >= 20 && ip[16]==0xff && ip[17]==0xff && ip[18]==0xff && ip[19]==0xff)
        std::memset(a3, 0xff, 6);                                   // limited broadcast
    else if (iplen >= 20 && (ip[16] & 0xf0) == 0xe0) {              // IPv4 multicast 224-239
        a3[0]=0x01; a3[1]=0x00; a3[2]=0x5e; a3[3]=ip[17]&0x7f; a3[4]=ip[18]; a3[5]=ip[19];
    } else std::memcpy(a3, _bssid.data(), 6);                       // unicast -> via AP

    // 802.11 data header, to-DS (STA->AP). addr1=BSSID(RA), addr2=self(TA), addr3=DA.
    std::vector<uint8_t> f;
    f.push_back(0x08); f.push_back(0x01);            // FC: data, to-DS, Protected
    f.push_back(0x00); f.push_back(0x00);            // duration
    f.insert(f.end(), _bssid.begin(), _bssid.end()); // addr1 = BSSID (RA)
    f.insert(f.end(), _self.begin(), _self.end());   // addr2 = SA (TA)
    f.insert(f.end(), a3, a3 + 6);                   // addr3 = DA (L2 destination)
    f.push_back(0x00); f.push_back(0x00);            // seq ctrl (HW fills)
    f[1] |= 0x40;                                    // set Protected bit

    // CCMP header (8B): PN0 PN1 rsv keyid(ext) PN2..PN5
    uint64_t pn = _txPn++;
    uint8_t pn0=pn&0xff, pn1=(pn>>8)&0xff, pn2=(pn>>16)&0xff,
            pn3=(pn>>24)&0xff, pn4=(pn>>32)&0xff, pn5=(pn>>40)&0xff;
    f.push_back(pn0); f.push_back(pn1); f.push_back(0x00); f.push_back(0x20); // keyid0, ext-IV
    f.push_back(pn2); f.push_back(pn3); f.push_back(pn4); f.push_back(pn5);

    // CCMP nonce: priority(1)=0 | A2(6) | PN(6, big-endian)
    uint8_t nonce[13]; nonce[0]=0;
    std::memcpy(nonce+1, _self.data(), 6);
    nonce[7]=pn5; nonce[8]=pn4; nonce[9]=pn3; nonce[10]=pn2; nonce[11]=pn1; nonce[12]=pn0;
    // Proper CCMP AAD (masked, duration excluded) — the AP's MIC check requires this exact
    // form, else our uplink (DHCP DISCOVER etc.) is dropped and we never get a lease.
    uint8_t aad[30]; size_t aadLen = ccmpAad(f.data(), false, aad);
    auto enc = _c.ccmp_encrypt(_tk.data(), nonce, plain.data(), plain.size(), aad, aadLen);
    f.insert(f.end(), enc.begin(), enc.end());       // ciphertext + MIC
    return f;
}

} // namespace apfpv
