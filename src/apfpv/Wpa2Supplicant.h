#pragma once
#include <cstdint>
#include <vector>
#include <array>
#include <string>
#include <functional>
#include "Wpa2Crypto.h"
namespace apfpv {
using Mac = std::array<uint8_t,6>;
class Wpa2Supplicant {
public:
    using SendFn = std::function<bool(const std::vector<uint8_t>&)>;
    Wpa2Supplicant(Crypto c, SendFn send);
    void replaceSend(SendFn send) { _send = std::move(send); }
    int state() const { return (int)_state; }  // 0=Idle 1=WaitMsg1 2=WaitMsg3 3=Done
    void begin(const std::string& passphrase, const std::string& ssid, const Mac& self, const Mac& bssid);
    // Fast path: skip the (slow ~seconds) PBKDF2 by injecting a pre-computed PMK. The PMK
    // depends ONLY on passphrase+SSID, so it can be cached across connects. CRITICAL: the
    // AP only advertises M1 for ~4s (4 retries); if begin() does PBKDF2 inline the RX
    // doesn't switch to the EAPOL/streaming phase until after the AP gives up -> M1 missed.
    void beginCached(const std::array<uint8_t,32>& pmk, const Mac& self, const Mac& bssid);
    const std::array<uint8_t,32>& pmk() const { return _pmk; }
    bool onEapolKey(const uint8_t* body, size_t len);
    bool ready() const;
    void setPmf(int p) { _pmf = p; }                 // 802.11w: 0=off 1=capable 2=required
    bool pmfActive() const { return _pmf > 0 && _state == State::Done; }
    // Verify a protected broadcast/multicast mgmt frame (Deauth/Disassoc carrying an MME)
    // with BIP-CMAC-128 against the installed IGTK. false => forged/unverifiable -> ignore it.
    bool verifyProtectedMgmt(const uint8_t* frame, size_t len) const;
    bool decryptData(const uint8_t* frame, size_t len, std::vector<uint8_t>& plain);
    // Build an ENCRYPTED 802.11 data frame (CCMP) carrying an IP/UDP payload.
    // Used to TX DHCP (and any station uplink) after the 4-way handshake.
    // 'ipPayload' is a complete IPv4 packet (IP+UDP+BOOTP). Returns the full
    // 802.11 frame ready for send_packet (caller prepends the TX descriptor).
    std::vector<uint8_t> buildEncryptedData(const uint8_t* ipPayload, size_t len, uint16_t ethertype = 0x0800);
    const std::array<uint8_t,16>& tk() const { return _tk; }
    const std::array<uint8_t,16>& gtk() const { return _gtk; }   // Lever C.2: HW-CAM group key
    uint8_t gtkKeyId() const { return _gtkKeyId; }
private:
    enum class State { Idle, WaitMsg1, WaitMsg3, Done };
    void derivePtk(const uint8_t* aNonce);
    std::vector<uint8_t> buildMsg2();
    std::vector<uint8_t> buildMsg4();
    std::vector<uint8_t> buildGroupMsg2();              // group-rekey ACK (Key Type=Group)
    bool installGtkFromKeyData(const uint8_t* body, size_t len);  // unwrap+install GTK (M3 & rekey)
    Crypto _c; SendFn _send;
    Mac _self{}, _bssid{};
    std::array<uint8_t,32> _pmk{}; std::array<uint8_t,16> _kck{}, _kek{}, _tk{}, _gtk{}, _gtkPrev{};
    std::array<uint8_t,32> _aNonce{}, _sNonce{};
    // Two GTK slots indexed by CCMP KeyID so frames in-flight on the OLD key still decrypt
    // across a group rekey (else the ~1s around a rekey loses broadcast/multicast).
    uint8_t _gtkKeyId = 0xff, _gtkPrevId = 0xff;       // 0xff = unset
    int _pmf = 0;                                      // 802.11w mode (RSN caps in M2)
    std::array<uint8_t,16> _igtk{}; uint16_t _igtkKeyId = 0; bool _igtkSet = false;  // BIP key
    // CCMP RX replay windows (per key). A-MPDU subframes arrive OUT OF ORDER within the
    // Block-Ack window (bufsz=64), so a strict "PN must increase" check wrongly rejects every
    // reordered subframe as a replay (was ~53% drop at A-MPDU rates). Use a 64-PN SLIDING
    // WINDOW per IEEE 802.11: _rxPn* = highest PN seen; _rxPn*Win = bitmap of the 64 PNs at/below
    // it (bit i set = PN (highest-i) already received). Accept if newer, or in-window & not dup.
    // 2048-PN window (32 × 64-bit words). Without HW Block-Ack the AP retransmits whole
    // unacked A-MPDUs, so frames arrive reordered by up to ~1000 PNs (measured). A 64-PN
    // window rejected the far-reordered-but-not-yet-seen frames as "replay", keeping them
    // lost; a 2048 window recovers them (the AP's retransmit is our only copy of an
    // over-air-lost subframe), while still dropping true duplicates (bit already set).
    // 64-PN window (1 word): matches the Block-Ack reorder window (bufsz=64). Recovers
    // legitimately-reordered A-MPDU subframes (the real fix vs the old strict pn<=last check)
    // WITHOUT accepting ancient unacked-A-MPDU retransmits — those carry stale RTP seqs that
    // arrive too late to render and only inflate the seq-gap "lost" counter (a 2048 window
    // measured lost=554k vs the strict check's ~12k). 64 is the sweet spot.
    static constexpr int PN_WIN_WORDS = 1;             // 64 PNs = the BA reorder window (tuned: wider
                                                       // accepts ancient AP retransmits -> stale RTP seqs inflate "lost")
    uint64_t _rxPnPair = 0; uint64_t _rxPnGtk[4] = {0,0,0,0};
    uint64_t _rxPnPairWin[PN_WIN_WORDS] = {0};
    uint64_t _rxPnGtkWin[4][PN_WIN_WORDS] = {{0}};
    static void winClear(uint64_t* w) { for (int i=0;i<PN_WIN_WORDS;i++) w[i]=0; }
    static bool winTest(const uint64_t* w, uint64_t bit) { return (w[bit>>6] >> (bit&63)) & 1; }
    static void winSet(uint64_t* w, uint64_t bit) { w[bit>>6] |= (1ULL << (bit&63)); }
    static void winShift(uint64_t* w, uint64_t n) {       // shift the whole bitmap UP by n bits
        if (n >= (uint64_t)PN_WIN_WORDS*64) { winClear(w); return; }
        int words = (int)(n >> 6), bits = (int)(n & 63);
        for (int i = PN_WIN_WORDS-1; i >= 0; --i) {
            uint64_t v = (i-words >= 0) ? w[i-words] : 0;
            uint64_t lo = (bits && i-words-1 >= 0) ? (w[i-words-1] >> (64-bits)) : 0;
            w[i] = (bits ? (v << bits) : v) | lo;
        }
    }
    // Returns true if PN `p` is fresh (accept) + updates the window; false if dup/too-old.
    static bool pnWindowCheck(uint64_t& highest, uint64_t* win, uint64_t p) {
        bool empty = true; for (int i=0;i<PN_WIN_WORDS;i++) if (win[i]) { empty=false; break; }
        if (highest == 0 && empty) { highest = p; winSet(win, 0); return true; }  // first frame
        if (p > highest) {
            winShift(win, p - highest);
            winSet(win, 0);               // bit 0 = the new highest
            highest = p;
            return true;
        }
        uint64_t diff = highest - p;
        if (diff >= (uint64_t)PN_WIN_WORDS*64) return false; // older than the window -> replay
        if (winTest(win, diff)) return false;                // already received -> duplicate
        winSet(win, diff);
        return true;
    }
    State _state = State::Idle; uint64_t _replay = 0; uint64_t _txPn = 1; uint8_t _replayCtr[8] = {0};
    std::vector<uint8_t> _lastKeyFrame;
};
}
