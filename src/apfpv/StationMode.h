#pragma once
#include <cstdint>
#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include "RtlAdapter.h"
#include "jaguar1/RadioManagementModule.h"
#include "ScanProbe.h"
namespace apfpv {
struct MacAddr { std::array<uint8_t,6> b; };
class StationMode {
public:
    using SendFrameFn = std::function<bool(const std::vector<uint8_t>&)>;
    StationMode(RtlAdapter& dev, RadioManagementModule& rm, SendFrameFn send);
    ~StationMode();
    void arm(const MacAddr& self, const MacAddr& bssid);
    bool sendAuthOpenSeq1(const MacAddr& self, const MacAddr& bssid);
    bool sendAssocRequest(const MacAddr& self, const MacAddr& bssid, const char* ssid);
    // Beacon scan: sweep channel hint + 5GHz UNII-1 (36..48), collect beacons,
    // return the matching AP (BSSID/channel/RSN). Channel set on the radio.
    ApInfo scanForSsid(const char* ssid, int channelHint, int perChannelMs);
    // Cipher negotiated from the AP's beacon, used to build the assoc RSN IE.
    void setNegotiatedCipher(uint32_t pairwise, uint32_t group) { _pairwise = pairwise; _group = group; }
    void setPmf(int p) { _pmf = p; }    // 0=off 1=capable 2=required (RSN caps in assoc-req)
    enum class Result { GO_LinkHeld, NOGO_Deauthed, NOGO_NoAssocResp, TXFAIL_NoAuthResp, Error };
    Result runProbe(const MacAddr& self, const MacAddr& bssid, const char* ssid, int holdSeconds = 10);
    void onMgmtFrame(uint16_t fc, const MacAddr& a1, const MacAddr& a2);
    // Feed raw mgmt frames during scan (beacon parsing).
    void onScanFrame(const uint8_t* frame, size_t len);
    // Optional UI funnel hook: runProbe reports 1=authenticating, 2=associating.
    void setPhaseCb(std::function<void(int)> cb) { _onPhase = std::move(cb); }
    // Port of the kernel station-association H2C firmware sequence (MACID_CFG /
    // MEDIA_STATUS_RPT / RSSI_SETTING). Registers the AP as a firmware peer.
    void sendStationH2C(uint8_t macid, const MacAddr& bssid);
    // Post-assoc transition: MSR->STATION + connected H2C (kernel order).
    void becomeStation(const MacAddr& bssid);
    // Connect-time channel width (20/40); default 20 MHz (legal DE). Fed from ApfpvStation::Params.
    void setConnectWidth(ChannelWidth_t w) { _connectWidth = w; }
private:
    std::function<void(int)> _onPhase;
    RtlAdapter& _dev; RadioManagementModule& _rm; SendFrameFn _send;
    bool _armed = false;
    uint32_t _pairwise = 0x000FAC04, _group = 0x000FAC04;   // default CCMP
    int _pmf = 0;                                           // 802.11w mode (RSN caps)
    ChannelWidth_t _connectWidth = CHANNEL_WIDTH_20;        // connect-time bandwidth (20/40), settable
    std::string _scanSsid; ApInfo _scanResult;
    // Last matched-beacon RSSI (dBm), kept STABLE across the per-channel _scanResult resets so arm()
    // can feed it to the DIG for anti-saturation. _scanResult.rssi is reset to -127 per channel, so
    // arm() must NOT read that; it reads this instead. 0 = no beacon heard yet.
    std::atomic<int> _lastBeaconRssi{0};
    std::mutex  _scanMtx;
public:
    std::atomic<bool> _alive{true};
    std::shared_ptr<std::atomic<bool>> _alivePtr;  // TOCTOU-safe: set by dispatch lambda, survives destruction
    std::atomic<bool> _gotAuthResp{false}, _gotAssocOk{false}, _gotDeauth{false};
    // The self MAC + AP BSSID we are joining. onMgmtFrame MUST match a1==self && a2==bssid
    // before believing an auth/assoc-resp — otherwise overheard frames from OTHER stations
    // on a shared/congested channel false-trigger the join (confirmed: a ch1 auth between
    // two unrelated devices flipped _gotAuthResp and produced a bogus "Associating").
    MacAddr _expectSelf{}, _expectBssid{};
};
}
