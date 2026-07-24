#pragma once
#include <cstdint>
#include <array>
#include <string>
#include <functional>
#include <atomic>
#include <thread>
#include <memory>
#include <chrono>
#include <mutex>
#include <vector>
#include <set>
#include "ScanProbe.h"
namespace apfpv {
using Mac = std::array<uint8_t,6>;
// Top-level APFPV station orchestrator: assembles arming -> auth/assoc ->
// WPA2 -> DHCP -> RTP+LQ into one connect() the JNI layer can call.
//
// DRIVER-LEVEL PERSISTENT RECONNECT (matches/exceeds the VRX): the VRX gets
// auto-reconnect from running wpa_supplicant -B as a daemon (it re-associates
// on link loss automatically). We replicate that AT THE DRIVER LEVEL with a
// supervisor thread: it watches for link loss (deauth, or RTP/beacon RX
// timeout) and re-runs the gated connect chain — no app/UI involvement, exactly
// like the supplicant daemon. We can be faster: on loss we re-arm on the SAME
// known channel (no rescan), so recovery is a re-auth/assoc/4-way, not a full
// network search.
class ApfpvStation {
public:
    enum class State { Idle, Scanning, Arming, Authenticating, Associating, Handshaking,
                       Dhcp, Streaming, FailNoAp, FailTx, FailNoAck, FailAuth,
                       FailDhcp, LinkLost, Reconnecting };
    using OnRtpFn   = std::function<void(const uint8_t*, size_t)>;
    using OnStateFn = std::function<void(State)>;
    struct Params { int channel=40; int bandwidth=20; std::string ssid="OpenIPC";
                    std::string passphrase="12345678"; bool lqFeedback=true;
                    // Static-IP option: staticIp != 0 -> SKIP DHCP and bind this IP (host byte
                    // order, e.g. 0xC0A8000A = 192.168.0.10). netmask 0 -> /24, gateway 0 -> subnet .1.
                    uint32_t staticIp=0, staticNetmask=0, staticGateway=0;
                    // 802.11w PMF: 0=off, 1=capable (MFPC, DEFAULT — modern + verified to still
                    // associate with non-PMF APs), 2=required (MFPC+MFPR). RSN caps in assoc-req+M2.
                    int  pmf=1;
                    // discovery: if scan=true we find BSSID+channel+cipher via
                    // beacons (security+discovery parity with wpa_supplicant);
                    // channel above is the starting hint / fallback.
                    bool scan=true;
                    // PHONE-ASSISTED connect: the phone's own Wi-Fi already knows
                    // the AP's channel + BSSID, so we can SKIP the dongle's slow,
                    // flaky beacon sweep and arm straight to this BSSID on `channel`.
                    std::array<uint8_t,6> bssid{};
                    bool haveBssid=false;
                    // reconnect tuning — matched to wpa_supplicant defaults the
                    // VRX relies on: immediate re-associate, ~5s scan interval.
                    bool autoReconnect=true;
                    int  rxTimeoutMs=8000;         // no RTP/beacon => lost. Tolerant on purpose:
                                                   // a brief RX gap must NOT trigger the full
                                                   // re-scan/auth/4-way reconnect — on flaky USB
                                                   // hosts (MediaTek) that caused stream-killing
                                                   // flapping (audio blips, video keyframe never lands)
                    int  reconnectBackoffMs=0;     // immediate (wpa_supplicant-like)
                    int  scanIntervalMs=5000;      // wpa_supplicant default scan_interval
                    int  maxBackoffMs=5000; };
    ApfpvStation(void* rtlUsbAdapter, void* radioMgmt, OnRtpFn onRtp, OnStateFn onState);
    // Provide the RtlJaguarDevice so the station can register the RX callback
    // (RxDeframe) as the device packet processor — the receive path.
    void setDevice(void* rtlJaguarDevice) { _rtl = rtlJaguarDevice; }
    ~ApfpvStation();
    bool connect(const Params& p);   // runs the chain + starts the supervisor
    void disconnect();               // stops supervisor + tears down
    // GENERAL-IP BRIDGE (for an Android VpnService TUN). setIpSink: downlink — receives the
    // full decrypted IPv4 packet (SSH / any TCP+UDP), making the dongle a full L3 interface
    // (not RTP-only). sendIpPacket: uplink — CCMP-encrypts + TXes an arbitrary IPv4 datagram.
    void setIpSink(OnRtpFn fn);   // propagates to _rx immediately (so arming after connect works)
    bool sendIpPacket(const uint8_t* ip, size_t len);
    // Minimal TCP/HTTP client over the link — PROVES the dongle carries general TCP, so SSH
    // and REST/HTTP (same transport) work over it. Real SYN/SYN-ACK/ACK + "GET <path>" to
    // dstIp:port; returns the response (empty on failure). Blocking; call after Streaming.
    std::string httpGet(uint32_t dstIp, uint16_t port, const std::string& path, int timeoutMs = 4000);
    // Raw bidirectional TCP over the link — enough to relay a whole SSH session (a local
    // socket listener in the host harness bridges plink <-> here). Minimal in-order stack:
    // accepts only in-order segments (the peer retransmits its own losses), ACKs everything,
    // segments uplink at a conservative MSS. Call only after Streaming, one connection at a time.
    bool tcpConnect(uint32_t dstIp, uint16_t dport, int timeoutMs = 4000);
    int  tcpPoll(std::string& out, int timeoutMs);   // >=0 = bytes appended; -1 = peer closed (FIN/RST)
    bool tcpSend(const uint8_t* data, size_t n);
    void tcpClose();
    uint32_t leaseIp() const;       // our DHCP IP (0 if none)
    uint32_t leaseServerIp() const; // DHCP server / gateway (the VTX analog)

    // All-SSID RF scan for the UI picker. Channel-hops the APFPV/5GHz/2.4 set,
    // parses every beacon, and calls onAp(ssid, info) once per NEW SSID as it is
    // discovered (live updates). Runs synchronously — call it from a worker
    // thread, and only while NOT connected (it drives the RX path). Requires
    // setDevice() to have been called.
    using OnApFn = std::function<void(const std::string&, const ApInfo&)>;
    // includeDfs=false skips the DFS 5GHz channels (52-64, 100-144) — faster, and
    // avoids tuning the radio onto radar-protected channels.
    void scanAll(int perChannelMs, bool includeDfs, const OnApFn& onAp);

    // VRX EIRP-calibration beacon: inject an open beacon with `ssid` on `channel`
    // at TX power index `txIndex`, so a second phone's Wi-Fi scan can read the
    // dongle's RSSI and estimate its EIRP. Stops any station/supervisor first.
    // Runs on its own thread; call stopBeaconCal() (or destruct) to end it.
    void startBeaconCal(const std::string& ssid, int channel, int txIndex);
    void stopBeaconCal();

    // ---- AP MODE (SoftAP) — ⚠️ NOT READY / WORK IN PROGRESS ----------------
    // ⚠️ DO NOT EXPOSE via JNI or call from the app. AP mode beacons (open/WPA2) + tracks client
    // RSSI + has a full WPA2 Authenticator, BUT a client CANNOT complete association (needs the
    // 8812 AP/master HW bring-up: HW beacon queue + TSF + per-station ACK) and the path wedges the
    // USB on exit. Kept as in-progress scaffolding only; see the banner on startAp() in the .cpp.
    // forceHwBeacon=true enables the Jaguar1 HW software-beacon path (ENSWBCN + MSR=AP + per-STA
    // ACK) without the DEVOURER_AP_HWACK env var — needed on Android where env vars aren't settable.
    void startAp(const std::string& ssid, int channel, const std::string& password = "", bool forceHwBeacon = false);
    void stopAp();
    struct ApStation { Mac mac; int rssiDbm; int state; };  // state 4=assoc, 7=authenticated
    std::vector<ApStation> apStations();

    State state() const { return _state.load(); }
    int   rssiDbm() const { return _rssi.load(); }
    int   channel() const { return _params.channel; }   // resolved AP channel (for UI)
    // RX path calls this on every received data/beacon frame so the supervisor
    // knows the link is alive (resets the loss timer). Wire from RxDeframe.
    void notifyRxAlive() { _lastRxMs.store(nowMs()); }
    // RX path calls this if a deauth/disassoc is seen (immediate loss).
    void notifyDeauth()  { _deauth.store(true); }
    // Handle ADDBA Request from the AP: send ADDBA Response (accept TID 0, immediate BA).
    // Called from RxDeframe when a Block-Ack action frame arrives. Enables A-MPDU RX.
    void handleAddbaRequest(const uint8_t* frame, size_t len);
private:
    void set(State s);
    bool runConnectChain();          // the gated arm->...->stream sequence
    void supervisorLoop();           // persistent reconnect watcher (the daemon)
    static int64_t nowMs() {
        using namespace std::chrono;
        return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    }
    void* _dev; void* _rm; void* _rtl = nullptr; OnRtpFn _onRtp; OnStateFn _onState;
    OnRtpFn _ipSink;   // downlink general-IP (TUN) sink, optional
    std::vector<std::vector<uint8_t>> _ipQ;   // httpGet: captured inbound IP packets
    std::mutex _ipQMtx;
    std::atomic<bool> _ipCapture{false};
    // active raw-TCP connection state (tcpConnect/tcpSend/tcpPoll/tcpClose)
    uint32_t _tcSrc=0,_tcDst=0; uint16_t _tcSport=0,_tcDport=0;
    uint32_t _tcSeq=0,_tcAck=0; bool _tcOpen=false;
    bool matchTcp(const std::vector<uint8_t>& pk, uint32_t& seq, uint8_t& flags,
                  const uint8_t*& payload, size_t& plen);
    std::function<void()> _gratArp;           // re-announce our IP<->MAC (keeps unicast RTP alive)
    // RX pipeline (lives across the connection so the device callback stays valid)
    std::unique_ptr<class RxDeframe> _rx;
    std::unique_ptr<class Wpa2Supplicant> _wpa;
    std::unique_ptr<class LqFeedback> _lq;
    std::unique_ptr<class ApfpvDhcp> _dhcp;
    // Cached WPA2 PMK (depends ONLY on passphrase+SSID). Precomputed in the background so
    // the slow (~seconds) PBKDF2 never runs inline in becomeStation — otherwise the RX
    // doesn't switch to the EAPOL phase until after the AP's ~4s M1 retry window expires.
    std::array<uint8_t,32> _pmkCache{};
    std::atomic<bool> _pmkValid{false};
    void ensurePmk();   // compute+cache the PBKDF2 PMK once (passphrase+SSID only)
    Params _params;
    std::atomic<State> _state{State::Idle};
    std::atomic<int> _rssi{-90};
    std::atomic<int64_t> _lastRxMs{0};
    uint8_t _pendingAddbaTids = 0;          // TIDs queued by dispatch, sent from connect thread
    std::atomic<bool> _deauth{false};
    std::atomic<bool> _raCfgSent{false};   // H2C MACID_CFG(0x40) sent this connection (once, like the kernel)
    std::atomic<bool> _run{false};   // supervisor running
    std::thread _supervisor;
    // RX runs on its OWN thread (RtlJaguarDevice::Init is a blocking read loop).
    // A single dispatch routes by phase: 0 = discovery/arm (StationMode), 1 =
    // streaming (RxDeframe). _rxReady flips true once the loop is live.
    std::thread _rxThread;
    std::atomic<int>  _rxPhase{0};
    std::atomic<bool> _rxReady{false};
    // scanAll() state — the RX collector runs on the device thread, so access is
    // mutex-guarded and _onScanAp is cleared before scanAll returns so the still-
    // running RX thread can never call into a destroyed callback.
    std::mutex _scanMtx;
    std::set<std::string> _scanSeen;
    OnApFn _onScanAp;
    // Diagnostics — MEMBERS (not scanAll locals): the RX collector is stored in
    // the device and called from its read thread, so it must not capture
    // stack-local counters by reference (that dangles -> SIGSEGV).
    std::atomic<int> _scanPkts{0};
    std::atomic<int> _scanBeacons{0};
    // VRX EIRP-calibration beacon injector
    std::atomic<bool> _beaconRun{false};
    std::thread _beaconThread;
    // AP mode (SoftAP): beacon thread reuses _beaconRun/_beaconThread; RX drives the handshake.
    std::atomic<bool> _apRun{false};
    Mac _apSelf{};
    bool _apWpa2 = false; int _apChannel = 0; std::string _apSsid;
    std::array<uint8_t,32> _apPmk{}; std::array<uint8_t,16> _apGtk{};
    std::unique_ptr<class Wpa2Authenticator> _apAuth;
    std::mutex _apMtx;
    std::vector<ApStation> _apStaList;
    void apOnRx(const uint8_t* f, size_t len, uint8_t rssiRaw);   // AP RX: auth/assoc/eapol
    void apSend(const std::vector<uint8_t>& mpdu, bool eapol);    // prepend TX desc + radiate
    void apTrack(const Mac& sta, uint8_t rssiRaw, int state);     // upsert station (state<0 keeps)
};
}
