// ============================================================================
//  ScanProbe.cpp — beacon parser: BSSID + channel + RSN negotiation
//  Removes the hardcoded-channel / empty-BSSID / fixed-cipher assumptions.
// ============================================================================
#include "ScanProbe.h"
#include <cstring>
namespace apfpv {

static uint32_t suite(const uint8_t* p){ return (p[0]<<24)|(p[1]<<16)|(p[2]<<8)|p[3]; }

// Walk a beacon/probe-resp's IEs once, extracting SSID + channel + RSN. Returns
// false if the frame isn't a valid mgmt beacon/probe-resp. ssidOut may be empty
// (hidden SSID). Shared by parseBeacon (target filter) and parseAnyBeacon.
bool ScanProbe::parseAnyBeacon(const uint8_t* f, size_t len, std::string& ssidOut, ApInfo& out) {
    if (len < 36) return false;
    uint16_t fc = f[0] | (f[1] << 8);
    if (((fc >> 2) & 0x3) != 0x0) return false;            // mgmt
    uint8_t sub = (fc >> 4) & 0xF;
    if (sub != 0x8 && sub != 0x5) return false;             // beacon(8) or probe-resp(5)

    // addr3 = BSSID; beacon fixed params = 12 bytes (ts8 + interval2 + caps2)
    Mac bssid; std::memcpy(bssid.data(), f + 16, 6);
    const uint8_t* ie = f + 24 + 12;
    size_t ieLen = len - (24 + 12);

    std::string foundSsid; int channel = 0; ApInfo tmp; tmp.bssid = bssid;
    size_t i = 0;
    while (i + 2 <= ieLen) {
        uint8_t id = ie[i], l = ie[i+1];
        if (i + 2 + l > ieLen) break;
        const uint8_t* d = ie + i + 2;
        if (id == 0) {                                       // SSID
            foundSsid.assign((const char*)d, l);
        } else if (id == 3 && l >= 1) {                      // DS param = channel
            channel = d[0];
        } else if (id == 48 && l >= 8) {                     // RSN IE
            tmp.rsnPresent = true;
            size_t o = 2;                                    // skip version
            tmp.groupCipher = suite(d + o); o += 4;
            uint16_t pc = d[o] | (d[o+1] << 8); o += 2;
            for (int k = 0; k < pc && o + 4 <= l; k++) { tmp.pairwise.push_back(suite(d+o)); o += 4; }
            if (o + 2 <= l) { uint16_t ac = d[o] | (d[o+1] << 8); o += 2;
                for (int k = 0; k < ac && o + 4 <= l; k++) { tmp.akm.push_back(suite(d+o)); o += 4; } }
        }
        i += 2 + l;
    }
    tmp.channel = channel; tmp.found = true;
    ssidOut = foundSsid;
    out = tmp;
    return true;
}

bool ScanProbe::parseBeacon(const uint8_t* f, size_t len, const std::string& ssid, ApInfo& out) {
    std::string found; ApInfo tmp;
    if (!parseAnyBeacon(f, len, found, tmp)) return false;
    if (found != ssid) return false;
    out = tmp;
    return true;
}

uint32_t ScanProbe::chooseCipher(const std::vector<uint32_t>& adv) {
    // Prefer CCMP (00-0F-AC-04); accept GCMP-256 (00-0F-AC-09) if offered.
    for (uint32_t s : adv) if (s == 0x000FAC04) return s;     // CCMP-128
    for (uint32_t s : adv) if (s == 0x000FAC09) return s;     // GCMP-256
    for (uint32_t s : adv) if (s == 0x000FAC04 || s == 0x000FAC0A) return s;
    return adv.empty() ? 0x000FAC04 : adv[0];                 // fallback CCMP
}

} // namespace apfpv
