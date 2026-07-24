// ============================================================================
//  Dot11Frames.cpp  —  802.11 management + EAPOL frame construction
// ============================================================================
//  Builds the raw frames StationMode injects via RtlJaguarDevice::send_packet().
//  These are TX of management/EAPOL frames — latency-tolerant (no SIFS deadline
//  applies to our *sending*; the SIFS deadline is on ACK-ing received unicast,
//  which is the hardware's job, not ours).
//
//  Frame layout we emit to send_packet(): the device's TX expects a radiotap
//  header (devourer prepends/handles the TX descriptor); here we build the
//  802.11 MPDU. Wiring of the radiotap/TX-desc prefix matches what upstream
//  devourer already does for monitor injection (see RtlJaguarDevice TX path).
//
//  Scope of this file: Auth (open seq1), Assoc-Request (with SSID + rates +
//  RSN IE for WPA2), and the EAPOL-Key frame shell for the 4-way handshake.
// ============================================================================

#include <cstdint>
#include <cstring>
#include <vector>
#include <string>
#include <array>

#include "Dot11Frames.h"
namespace apfpv {

using Mac = std::array<uint8_t,6>;

// ---- 802.11 frame-control builders ----------------------------------------
static constexpr uint16_t FC_MGMT          = 0x0000;
static constexpr uint16_t FC_SUB_ASSOC_REQ = 0x0000;  // mgmt subtype 0
static constexpr uint16_t FC_SUB_AUTH      = 0x00B0;   // mgmt subtype 11 (<<4)
static constexpr uint16_t FC_SUB_BEACON    = 0x0080;   // mgmt subtype 8  (<<4)
static constexpr uint16_t FC_DATA          = 0x0008;   // type data
static constexpr uint16_t FC_TODS          = 0x0100;   // to-DS (station->AP)
static constexpr uint16_t FC_FROMDS        = 0x0200;   // from-DS (AP->station)

// Generic 802.11 MAC header (24 bytes, 3-address)
static void put_hdr(std::vector<uint8_t>& f, uint16_t fc,
                    const Mac& a1, const Mac& a2, const Mac& a3,
                    uint16_t seq = 0) {
    f.push_back(fc & 0xFF); f.push_back(fc >> 8);   // frame control
    f.push_back(0); f.push_back(0);                 // duration (HW fills)
    f.insert(f.end(), a1.begin(), a1.end());        // addr1 = RA (dest)
    f.insert(f.end(), a2.begin(), a2.end());        // addr2 = TA (us)
    f.insert(f.end(), a3.begin(), a3.end());        // addr3 = BSSID
    f.push_back((seq << 4) & 0xFF); f.push_back((seq << 4) >> 8); // seq ctrl
}

static void put_le16(std::vector<uint8_t>& f, uint16_t v){ f.push_back(v&0xFF); f.push_back(v>>8); }

// ---- Auth, open-system, sequence 1 -----------------------------------------
std::vector<uint8_t> BuildAuthOpenSeq1(const Mac& self, const Mac& bssid) {
    std::vector<uint8_t> f;
    put_hdr(f, FC_MGMT | FC_SUB_AUTH, bssid, self, bssid);
    put_le16(f, 0);   // auth algorithm = Open System
    put_le16(f, 1);   // auth sequence  = 1
    put_le16(f, 0);   // status code    = 0
    return f;
}

// ---- Beacon (open, for the VRX EIRP-calibration mode) ----------------------
//   Broadcast beacon so a phone Wi-Fi scan lists the dongle as an AP and can read
//   its RSSI (-> EIRP). Open network (no privacy bit): we never associate, this is
//   a measurement target only.
// WPA2-PSK/CCMP RSN IE (byte-identical to the station assoc-req). Appended to beacon/probe-resp.
static void appendRsnIe(std::vector<uint8_t>& f) {
    static const uint8_t rsn[] = {
        0x30,0x14, 0x01,0x00, 0x00,0x0f,0xac,0x04,
        0x01,0x00, 0x00,0x0f,0xac,0x04, 0x01,0x00, 0x00,0x0f,0xac,0x02, 0x00,0x00 };
    f.insert(f.end(), rsn, rsn+sizeof(rsn));
}

std::vector<uint8_t> BuildBeacon(const Mac& self, const std::string& ssid, uint8_t channel, bool wpa2) {
    std::vector<uint8_t> f;
    static const Mac bcast = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    put_hdr(f, FC_MGMT | FC_SUB_BEACON, bcast, self, self);   // addr3=BSSID=self
    for (int i = 0; i < 8; ++i) f.push_back(0);   // timestamp (driver/HW may fill)
    put_le16(f, 100);                       // beacon interval (100 TU ~= 102.4 ms)
    put_le16(f, wpa2 ? 0x0011 : 0x0001);    // capability: ESS [+ Privacy when WPA2]
    f.push_back(0x00); f.push_back((uint8_t)ssid.size());          // SSID
    f.insert(f.end(), ssid.begin(), ssid.end());
    static const uint8_t rates[] = {0x82,0x84,0x8b,0x96,0x0c,0x12,0x18,0x24};
    f.push_back(0x01); f.push_back(sizeof(rates));                 // Supported Rates
    f.insert(f.end(), rates, rates+sizeof(rates));
    f.push_back(0x03); f.push_back(0x01); f.push_back(channel);    // DS Param = channel
    if (wpa2) appendRsnIe(f);                                      // RSN IE for WPA2-PSK
    return f;
}

// ---- AP-side responses: Auth (open seq2), Assoc Response, Probe Response ----
std::vector<uint8_t> BuildAuthResp(const Mac& ap, const Mac& sta) {
    std::vector<uint8_t> f;
    put_hdr(f, FC_MGMT | FC_SUB_AUTH, sta, ap, ap);   // a1=STA a2=AP a3=BSSID
    put_le16(f, 0); put_le16(f, 2); put_le16(f, 0);   // open system, seq 2, status success
    return f;
}
std::vector<uint8_t> BuildAssocResp(const Mac& ap, const Mac& sta, uint16_t aid) {
    std::vector<uint8_t> f;
    put_hdr(f, FC_MGMT | 0x0010, sta, ap, ap);        // mgmt subtype 1 = Assoc Response
    put_le16(f, 0x0011);                              // capability (ESS+Privacy)
    put_le16(f, 0);                                   // status = success
    put_le16(f, 0xC000 | (aid & 0x3FFF));             // AID (two MSBs set per spec)
    static const uint8_t rates[] = {0x82,0x84,0x8b,0x96,0x0c,0x12,0x18,0x24};
    f.push_back(0x01); f.push_back(sizeof(rates));
    f.insert(f.end(), rates, rates+sizeof(rates));
    return f;
}
std::vector<uint8_t> BuildProbeResp(const Mac& ap, const Mac& sta, const std::string& ssid,
                                    uint8_t channel, bool wpa2) {
    std::vector<uint8_t> f;
    put_hdr(f, FC_MGMT | 0x0050, sta, ap, ap);        // mgmt subtype 5 = Probe Response
    for (int i = 0; i < 8; ++i) f.push_back(0);
    put_le16(f, 100); put_le16(f, wpa2 ? 0x0011 : 0x0001);
    f.push_back(0x00); f.push_back((uint8_t)ssid.size());
    f.insert(f.end(), ssid.begin(), ssid.end());
    static const uint8_t rates[] = {0x82,0x84,0x8b,0x96,0x0c,0x12,0x18,0x24};
    f.push_back(0x01); f.push_back(sizeof(rates));
    f.insert(f.end(), rates, rates+sizeof(rates));
    f.push_back(0x03); f.push_back(0x01); f.push_back(channel);
    if (wpa2) appendRsnIe(f);
    return f;
}

// ---- Association Request (SSID + supported rates + RSN IE for WPA2) ---------
std::vector<uint8_t> BuildAssocRequest(const Mac& self, const Mac& bssid,
                                       const std::string& ssid, uint16_t rsnCaps) {
    std::vector<uint8_t> f;
    put_hdr(f, FC_MGMT | FC_SUB_ASSOC_REQ, bssid, self, bssid);

    put_le16(f, 0x1531);   // capability info (match kernel assoc-req: ESS+Privacy+ShortPre/Slot+SpectrumMgmt)
    put_le16(f, 3);        // listen interval

    // IE: SSID (0)
    f.push_back(0x00); f.push_back((uint8_t)ssid.size());
    f.insert(f.end(), ssid.begin(), ssid.end());

    // IE: Supported Rates (1). On 5GHz, CCK rates (1/2/5.5/11) are INVALID and make the AP
    // classify us as a legacy/2.4GHz client → conservative HT-1SS rate control (the MCS2 pin).
    // DEVOURER_OFDM_RATES=1 sends the kernel's exact 5GHz OFDM-only set (6/9/12/18/24/36/48/54,
    // basic bits on 6/12/24) in a single tag-1 IE with NO Extended-Rates — matching the live
    // kernel assoc-req byte-for-byte. Default keeps the 2.4GHz-compatible CCK+OFDM split.
    if (std::getenv("DEVOURER_OFDM_RATES")) {
        static const uint8_t rates5[] = {0x8c,0x12,0x98,0x24,0xb0,0x48,0x60,0x6c};
        f.push_back(0x01); f.push_back(sizeof(rates5));
        f.insert(f.end(), rates5, rates5+sizeof(rates5));
    } else {
        static const uint8_t rates[] = {0x82,0x84,0x8b,0x96,0x0c,0x12,0x18,0x24};
        f.push_back(0x01); f.push_back(sizeof(rates));
        f.insert(f.end(), rates, rates+sizeof(rates));
        // IE: Extended Supported Rates (50) — 24/36/48/54 Mbps.
        static const uint8_t extRates[] = {0x30,0x48,0x60,0x6c};
        f.push_back(0x32); f.push_back(sizeof(extRates));
        f.insert(f.end(), extRates, extRates+sizeof(extRates));
    }

    // IE: RSN (48) — WPA2-PSK / CCMP. Required so the AP starts the 4-way HS.
    //   version 1; group=CCMP; pairwise=CCMP; akm=PSK
    static const uint8_t rsn[] = {
        0x01,0x00,                      // RSN version 1
        0x00,0x0f,0xac,0x04,            // group cipher  = CCMP (00-0F-AC-4)
        0x01,0x00, 0x00,0x0f,0xac,0x04, // 1 pairwise    = CCMP
        0x01,0x00, 0x00,0x0f,0xac,0x02, // 1 AKM         = PSK  (00-0F-AC-2)
        0x00,0x00                       // RSN capabilities
    };
    f.push_back(0x30); f.push_back(sizeof(rsn));
    f.insert(f.end(), rsn, rsn+sizeof(rsn));
    f[f.size()-2] = (uint8_t)(rsnCaps & 0xff);   // RSN capabilities (PMF: MFPC b7 / MFPR b6)
    f[f.size()-1] = (uint8_t)(rsnCaps >> 8);

    // IEs the kernel's 88XXau assoc-request includes that a modern HT/VHT AP needs
    // to ACCEPT the association and start the 4-way. Missing HT Capabilities in
    // particular makes the AP reject/mis-associate us and NEVER send EAPOL M1 — our
    // exact symptom (reach Handshaking, no M1). Bytes are the kernel's exact values
    // for this 8812 (captured via usbmon on a working wpa_supplicant connect).
    static const uint8_t tail[] = {
        0xdd,0x07, 0x00,0x50,0xf2,0x02,0x00,0x01,0x00,            // WMM Information Element
        // HT Capabilities. A-MPDU Params byte was 0x1f = MaxLenExp 3 (64KB) + MinMPDUSpacing 7
        // (16µs) — we advertised 16µs spacing, which makes the AP pad/cap A-MPDUs to us and holds
        // throughput at ~30Mbps. 0x13 = MaxLenExp 3 (64KB) + MinMPDUSpacing 4 (2µs): tighter packing
        // the 8812 RX handles, so the AP can build deep aggregates (the 30→65 lever).
        // HT capinfo byte0 = 0x2c. NOTE (2026-07-14): the LIVE kernel advertises 0x6e (40MHz=1,
        // SGI40=1); ours omits both. Advertising kernel-match caps + OMN=40MHz (0x11) + OFDM-only
        // 5GHz rates was A/B-tested on the 40MHz OpenIPC VTX and did NOT change the AP's rate pin
        // (still HT-1SS MCS2, no aggregation), so it's NOT the rate-control lever. Left at 0x2c /
        // OMN 0x12 to avoid regressing 80MHz APs (needs a bandwidth-aware caps rewrite, not a
        // hardcode). The self-consistency issue is real but orthogonal to the MCS2 pin.
        0x2d,0x1a, 0x2c,0x19,0x13,0xff,0xff,                      // HT Capabilities — A-MPDU density 2µs (was 16µs)
                   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0xbf,0x0c, 0xa2,0x31,0xc0,0x03,0xfa,0xff,0x63,0x03,0xfa,0xff,0x63,0x03,  // VHT Caps (12B)
        0xc7,0x01, 0x12,                                          // Operating Mode Notification: 80MHz, 2SS
        0x7f,0x08, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40        // Extended Capabilities
    };
    f.insert(f.end(), tail, tail+sizeof(tail));

    return f;
}

// ---- EAPOL-Key frame shell (for WPA2 4-way handshake; msg 2/4 from us) ------
//   The KCK/KEK come from the PTK (derived in Wpa2Supplicant). This builds the
//   802.11 DATA + LLC/SNAP(0x888E) + EAPOL-Key body; MIC is filled by caller
//   after it computes HMAC over the assembled buffer with MIC field zeroed.
std::vector<uint8_t> BuildEapolKey(const Mac& self, const Mac& bssid,
                                   const std::vector<uint8_t>& keyBody) {
    std::vector<uint8_t> f;
    put_hdr(f, FC_DATA | FC_TODS, bssid, self, bssid);
    // LLC/SNAP for EAPOL
    static const uint8_t snap[] = {0xaa,0xaa,0x03,0x00,0x00,0x00,0x88,0x8e};
    f.insert(f.end(), snap, snap+sizeof(snap));
    f.insert(f.end(), keyBody.begin(), keyBody.end());
    return f;
}

// AP-side EAPOL-Key (from-DS): a1=DA=STA, a2=BSSID=AP, a3=SA=AP. For Authenticator M1/M3.
std::vector<uint8_t> BuildEapolKeyFromAp(const Mac& ap, const Mac& sta,
                                         const std::vector<uint8_t>& keyBody) {
    std::vector<uint8_t> f;
    put_hdr(f, FC_DATA | FC_FROMDS, sta, ap, ap);
    static const uint8_t snap[] = {0xaa,0xaa,0x03,0x00,0x00,0x00,0x88,0x8e};
    f.insert(f.end(), snap, snap+sizeof(snap));
    f.insert(f.end(), keyBody.begin(), keyBody.end());
    return f;
}


// Negotiated RSN: build the IE from the scanned pairwise/group cipher suites.
std::vector<uint8_t> BuildAssocRequest(const Mac& self, const Mac& bssid,
                                       const std::string& ssid,
                                       uint32_t pairwiseCipher, uint32_t groupCipher,
                                       uint16_t rsnCaps) {
    // Reuse the base frame, then append an RSN IE with the negotiated ciphers.
    std::vector<uint8_t> f = BuildAssocRequest(self, bssid, ssid, rsnCaps);
    // strip the trailing hardcoded RSN (last sizeof(rsn)+2 bytes) and re-add.
    // Simpler: the base already appended a CCMP/PSK RSN; if negotiated == CCMP
    // it is identical, so only rebuild when different.
    if (pairwiseCipher == 0x000FAC04 && groupCipher == 0x000FAC04) return f;
    // remove last RSN IE (0x30 ... ) — find last 0x30 tag
    for (size_t i = f.size(); i-- > 0; ) {
        if (f[i] == 0x30 && i + 1 < f.size()) { f.resize(i); break; }
    }
    auto be32 = [&](uint32_t v){ f.push_back(v>>24); f.push_back(v>>16); f.push_back(v>>8); f.push_back(v); };
    f.push_back(0x30); f.push_back(20);          // RSN IE, len 20
    f.push_back(0x01); f.push_back(0x00);        // version 1
    be32(groupCipher);                           // group
    f.push_back(0x01); f.push_back(0x00); be32(pairwiseCipher);   // 1 pairwise
    f.push_back(0x01); f.push_back(0x00); be32(0x000FAC02);       // 1 AKM = PSK
    f.push_back((uint8_t)(rsnCaps & 0xff)); f.push_back((uint8_t)(rsnCaps >> 8));  // RSN caps (PMF)
    return f;
}

} // namespace apfpv
