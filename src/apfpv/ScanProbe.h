#pragma once
#include <cstdint>
#include <array>
#include <string>
#include <functional>
#include <vector>
namespace apfpv {
using Mac = std::array<uint8_t,6>;
// Beacon/probe-response scanner: discovers the AP's BSSID, channel, and its
// advertised RSN (cipher/AKM) so we associate with NEGOTIATED parameters
// instead of hardcoded assumptions — security + discovery parity with the
// kernel wpa_supplicant the VRX uses.
struct ApInfo {
    Mac      bssid{};
    int      channel = 0;
    bool     found = false;
    // negotiated RSN, parsed from the beacon's RSN IE:
    uint32_t groupCipher = 0;          // e.g. 0x000FAC04 (CCMP)
    std::vector<uint32_t> pairwise;    // advertised pairwise ciphers
    std::vector<uint32_t> akm;         // advertised AKM suites (PSK = 0x000FAC02)
    bool     rsnPresent = false;
    int      rssi = -127;
};
class ScanProbe {
public:
    // Parse a received beacon/probe-resp frame for the target SSID. Returns true
    // and fills 'out' when the SSID matches. Feed RX mgmt frames here during scan.
    static bool parseBeacon(const uint8_t* frame, size_t len, const std::string& ssid, ApInfo& out);
    // Parse ANY beacon/probe-resp (no target filter) for an all-SSID scan:
    // returns true for a valid mgmt beacon with a non-empty SSID and fills
    // ssidOut + out (bssid / channel / RSN). Used by ApfpvStation::scanAll.
    static bool parseAnyBeacon(const uint8_t* frame, size_t len, std::string& ssidOut, ApInfo& out);
    // Pick the strongest pairwise cipher we support from the AP's advertised set.
    static uint32_t chooseCipher(const std::vector<uint32_t>& advertised);
};
}
