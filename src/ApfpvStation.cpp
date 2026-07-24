// ============================================================================
//  ApfpvStation.cpp — orchestrator + DRIVER-LEVEL persistent reconnect
//  Mirrors the VRX's wpa_supplicant -B daemon behavior at the driver level:
//  a supervisor thread re-establishes the link on loss (deauth or RX timeout),
//  with no app/UI involvement. Faster than the VRX on recovery because we
//  re-arm on the known channel (no full rescan).
// ============================================================================
#include "ApfpvStation.h"
#include "StationMode.h"
#include "Wpa2Supplicant.h"
#include "Wpa2Authenticator.h"
#include "hal_com_reg.h"     // MSR / REG_BSSID / REG_RCR / RCR_* / FORCEACK for AP-mode HW config
#include "Wpa2Crypto.h"
#include "ApfpvDhcp.h"
#include "RxDeframe.h"
#include "StationTxDesc.h"
#include "Dot11Frames.h"
#include "FrameParser.h"
#include "ScanProbe.h"
#include "LqFeedback.h"
#if defined(__ANDROID__)
#include <sys/system_properties.h>
#endif
#include "PhydmWatchdog.h"
#include "RtlUsbAdapter.h"
#include "RtlJaguarDevice.h"
#include "SelectedChannel.h"
#include "RadioManagementModule.h"
#include <algorithm>
#include <cstring>
#if defined(__ANDROID__)
#include <sys/system_properties.h>
#endif

namespace {
// Android has no settable process env, so DEVOURER_* getenv knobs never fire in the app. Mirror the
// key streaming knobs to `debug.pixelpilot.*` props (adb setprop, no root). Returns true if the prop
// exists and is not "0". Host builds (no __ANDROID__) always return false → env-only there.
static bool apfpvProp(const char* name) {
#if defined(__ANDROID__)
    char v[PROP_VALUE_MAX] = {0};
    if (__system_property_get(name, v) > 0 && v[0] != '0') return true;
#else
    (void)name;
#endif
    return false;
}
// ACCEPT the AP's BlockAck (arm RX-BA regs + ADDBA-init + StatusCode 0) instead of declining.
// Env DEVOURER_ENABLE_BA (host) or `debug.pixelpilot.ba=1` (adb setprop, Android). THE fundamental-fix
// test: now that FW_RA makes the AP actively aggregate A-MPDU to us, does accepting BA make the chip's
// MAC auto-emit the SIFS compressed BlockAck so the AP retransmits lost subframes → losses recovered
// (no more reference-corruption freeze)? Prior sessions tested this only when the AP WASN'T aggregating.
static bool enableBaOn() {
    return std::getenv("DEVOURER_ENABLE_BA") != nullptr || apfpvProp("debug.pixelpilot.ba");
}
// Buffer size (in MPDUs) to negotiate in OUR ADDBA Response, overriding the AP's request (see
// handleAddbaRequest). 0/unset = echo the AP's requested value unchanged. DEVOURER_BA_BUFSZ (host)
// / debug.pixelpilot.babufsz (Android).
static int baRespBufSizeOverride() {
    if (const char* e = std::getenv("DEVOURER_BA_BUFSZ")) { int n = std::atoi(e); if (n > 0) return n; }
#if defined(__ANDROID__)
    char v[PROP_VALUE_MAX] = {0};
    if (__system_property_get("debug.pixelpilot.babufsz", v) > 0) {
        int n = std::atoi(v);
        if (n > 0) return n;
    }
#endif
    return 0;
}
} // namespace
// Platform-agnostic: <android/log.h> is the NDK header on Android and the
// compat stderr shim on native host builds (see WiFiDriver/compat), so the same
// diagnostics surface on Windows/Linux too — essential for chasing parity.
#include <android/log.h>
#define SCANLOG(...) __android_log_print(ANDROID_LOG_INFO, "apfpv-scan", __VA_ARGS__)

namespace apfpv {

// Map a primary channel + desired bandwidth (MHz) to a SelectedChannel kept
// INSIDE the legal DE APFPV band 5170-5250 MHz (5.2 GHz UNII-1; 200 mW @ 20 MHz
// under the 10 mW/MHz PSD cap). For 40 MHz we pick the HT40 secondary side that
// never crosses 5170/5250 — primary 36/44 extend ABOVE (offset LOWER = primary
// is the lower 20 MHz), primary 40/48 extend BELOW (offset UPPER); both land on
// centres 38 (5170-5210) or 46 (5210-5250). 20 MHz is pass-through; 80 MHz
// centres on 42 and fills the band exactly. NB: WFB's 5.8 GHz / 25 mW band is a
// separate code path and is unaffected by this helper.
static SelectedChannel legalApfpvChannel(int ch, int bwMHz) {
    uint8_t c = (uint8_t)(ch > 0 ? ch : 40);
    if (bwMHz >= 80)
        return SelectedChannel{ c, (uint8_t)HAL_PRIME_CHNL_OFFSET_DONT_CARE, CHANNEL_WIDTH_80 };
    if (bwMHz == 40) {
        // WFB proven 40MHz rule: odd (channel/4) -> LOWER, even -> UPPER.
        // ch36(9 odd)->LOWER ch40(10 even)->UPPER ch44(11 odd)->LOWER ch48(12 even)->UPPER.
        uint8_t off = (c > 14) ? (((c / 4) & 1) ? (uint8_t)HAL_PRIME_CHNL_OFFSET_LOWER
                                                 : (uint8_t)HAL_PRIME_CHNL_OFFSET_UPPER)
                               : ((c <= 7) ? (uint8_t)HAL_PRIME_CHNL_OFFSET_LOWER
                                           : (uint8_t)HAL_PRIME_CHNL_OFFSET_UPPER);
        return SelectedChannel{ c, off, CHANNEL_WIDTH_40 };
    }
    return SelectedChannel{ c, (uint8_t)HAL_PRIME_CHNL_OFFSET_DONT_CARE, CHANNEL_WIDTH_20 };
}

void ApfpvStation::set(State s) {
    _state.store(s);
    // 0Idle 1Scan 2Arm 3Auth 4Assoc 5Handshake 6Dhcp 7Stream 8FailNoAp 9FailTx
    // 10FailNoAck 11FailAuth 12FailDhcp 13LinkLost 14Reconnecting
    SCANLOG("apfpv state -> %d", (int)s);
    if (_onState) _onState(s);
}

ApfpvStation::ApfpvStation(void* dev, void* rm, OnRtpFn onRtp, OnStateFn onState)
    : _dev(dev), _rm(rm), _onRtp(std::move(onRtp)), _onState(std::move(onState)) {}

ApfpvStation::~ApfpvStation() { stopBeaconCal(); disconnect(); }

// VRX EIRP-calibration beacon: inject an open beacon periodically so a second
// phone can scan + measure the dongle's EIRP. Not a station while beaconing.
void ApfpvStation::startBeaconCal(const std::string& ssid, int channel, int txIndex) {
    stopBeaconCal();
    disconnect();                       // can't be a station and beacon at once
    if (!_rtl || !_dev) return;
    auto* rtl = reinterpret_cast<RtlJaguarDevice*>(_rtl);
    auto* dev = reinterpret_cast<RtlUsbAdapter*>(_dev);
    try {
        rtl->Init([](const Packet&){}, legalApfpvChannel(channel, 20));   // tune + bring up
        rtl->SetTxPower((uint8_t)std::max(0, std::min(63, txIndex)));
    } catch (...) { return; }
    // **TX-RADIATION FIX.** The init/channel-set IQK (Iqk8812a::ConfigureMac) pauses
    // all 6 TX queues (REG_TXPAUSE 0x522 = 0x3f) for calibration. The beacon/monitor
    // path had NO clear (only the station arm() path got one), so the MAC drained the
    // FIFO ("injected OK") but the scheduler never keyed the PHY/PA -> zero RF on air.
    // Release the queues unconditionally (mirrors StationMode.cpp arm step 4c).
    dev->rtw_write8(0x0522, 0x00);
    SCANLOG("beacon: REG_TXPAUSE -> 0x%02x", dev->rtw_read8(0x0522));
    // our MAC (REG_MACID); synthesize a locally-administered one if unprogrammed
    Mac self{};
    for (int i = 0; i < 6; ++i) self[i] = dev->rtw_read8(0x0610 + i);
    bool bad = true; for (int i = 0; i < 6; ++i) if (self[i] != 0 && self[i] != 0xFF) bad = false;
    if (bad) { const uint8_t m[6] = {0x02,0x11,0x22,0x33,0x44,0x55};
               for (int i = 0; i < 6; ++i) self[i] = m[i]; }
    auto mpdu = BuildBeacon(self, ssid, (uint8_t)channel);
    _beaconRun.store(true);
    _beaconThread = std::thread([this, dev, mpdu]() {
        std::vector<uint8_t> frame(40 + mpdu.size(), 0);   // 40 = 8812 TX desc
        std::memcpy(frame.data() + 40, mpdu.data(), mpdu.size());
        FillStationTxDesc(frame.data(), (uint16_t)mpdu.size(), 40,
                          0, StationFrameKind::BroadcastMgmt, 0, 0x04);   // BMC=1, no ACK
        int n = 0;
        while (_beaconRun.load()) {
            dev->rtw_write8(0x0522, 0x00);   // keep all TX queues released every beacon
            bool ok = false;
            try { ok = dev->send_packet(frame.data(), frame.size()); } catch (...) {}
            if ((n++ % 20) == 0) SCANLOG("beacon: tx #%d ok=%d len=%zu", n, (int)ok, frame.size());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));   // ~10 beacons/s
        }
    });
}

void ApfpvStation::stopBeaconCal() {
    _beaconRun.store(false);
    if (_beaconThread.joinable()) _beaconThread.join();
}

// ---- AP MODE (SoftAP) -------------------------------------------------------
void ApfpvStation::apSend(const std::vector<uint8_t>& mpdu, bool eapol) {
    auto* dev = reinterpret_cast<RtlUsbAdapter*>(_dev);
    if (!dev || mpdu.empty()) return;
    std::vector<uint8_t> frame(40 + mpdu.size(), 0);
    std::memcpy(frame.data()+40, mpdu.data(), mpdu.size());
    FillStationTxDesc(frame.data(), (uint16_t)mpdu.size(), 40, 0,
                      eapol ? StationFrameKind::EapolData : StationFrameKind::Mgmt, 0, 0x04);
    dev->rtw_write8(0x0522, 0x00);
    try { dev->send_packet(frame.data(), frame.size()); } catch (...) {}
}

void ApfpvStation::apTrack(const Mac& sta, uint8_t rssiRaw, int state) {
    int dbm = (int)(rssiRaw & 0x7f) - 110; if (dbm < -100) dbm = -100; if (dbm > -10) dbm = -10;
    std::lock_guard<std::mutex> lk(_apMtx);
    for (auto& s : _apStaList) if (s.mac == sta) { s.rssiDbm = dbm; if (state >= 0) s.state = state; return; }
    _apStaList.push_back(ApStation{sta, dbm, state >= 0 ? state : 4});
}

std::vector<ApfpvStation::ApStation> ApfpvStation::apStations() {
    std::lock_guard<std::mutex> lk(_apMtx);
    return _apStaList;
}

void ApfpvStation::apOnRx(const uint8_t* f, size_t len, uint8_t rssiRaw) {
    if (len < 24) return;
    uint16_t fc = f[0] | (f[1] << 8);
    uint8_t type = (fc >> 2) & 0x3, sub = (fc >> 4) & 0xf;
    static int g_apRxN = 0;
    if ((++g_apRxN % 100) == 1) fprintf(stderr, "[ap-rx-any] #%d type=%u sub=%u len=%zu\n", g_apRxN, type, sub, len);
    Mac a1, a2; std::memcpy(a1.data(), f+4, 6); std::memcpy(a2.data(), f+10, 6);
    bool toUs = std::memcmp(a1.data(), _apSelf.data(), 6) == 0;
    bool grp  = (f[4] & 0x01);
    if (type == 0) {                                       // management
        if ((sub==4||sub==11||sub==0) && (toUs||grp))
            fprintf(stderr, "[ap-rx] mgmt sub=%u from %02x:%02x:%02x:%02x:%02x:%02x toUs=%d\n",
                    sub, a2[0],a2[1],a2[2],a2[3],a2[4],a2[5], (int)toUs);
        if (sub == 4 && (toUs || grp)) {                  // Probe Request -> Probe Response
            apTrack(a2, rssiRaw, -1);
            apSend(BuildProbeResp(_apSelf, a2, _apSsid, (uint8_t)_apChannel, _apWpa2), false);
        } else if (sub == 11 && toUs) {                   // Auth (open seq1) -> Auth Resp (seq2)
            apTrack(a2, rssiRaw, -1);
            apSend(BuildAuthResp(_apSelf, a2), false);
        } else if (sub == 0 && toUs) {                    // Assoc Request -> Assoc Resp [+ 4-way]
            apTrack(a2, rssiRaw, 4);
            apSend(BuildAssocResp(_apSelf, a2, 1), false);
            if (_apWpa2) {
                _apAuth = std::make_unique<Wpa2Authenticator>(MakeWpa2Crypto(),
                    [this](const std::vector<uint8_t>& m){ apSend(m, true); return true; });
                _apAuth->begin(_apPmk, _apSelf, a2, _apGtk, 1);
                _apAuth->startHandshake();
            } else apTrack(a2, rssiRaw, 7);               // open: associated == connected
        }
        return;
    }
    if (type == 2 && toUs) {                               // data from the station (to-DS)
        size_t hdr = 24; if (fc & 0x0080) hdr += 2;       // QoS-data
        if (len < hdr + 8) return;
        const uint8_t* llc = f + hdr;
        static const uint8_t snap[6] = {0xaa,0xaa,0x03,0x00,0x00,0x00};
        if (std::memcmp(llc, snap, 6) == 0 && ((llc[6]<<8)|llc[7]) == 0x888E && _apAuth) {
            apTrack(a2, rssiRaw, -1);
            if (_apAuth->onEapolKey(llc + 8, (len - hdr) - 8)) apTrack(a2, rssiRaw, 7);  // 4-way done
        }
    }
}

// ⚠️⚠️ WORK IN PROGRESS — NOT a usable SoftAP yet. The dongle beacons (open/WPA2), tracks
// clients + RSSI, and the full WPA2 Authenticator (Wpa2Authenticator) is implemented + crypto
// self-tested. BUT a client cannot complete association: that needs the 8812 AP/master-mode HW
// bring-up (HW beacon queue + TSF + per-station ACK/MACID registration), which the userspace
// monitor path doesn't provide. Setting MSR=AP even stops the injected beacon from radiating
// (HW expects beacons from its own queue). Station mode is the production path; do NOT expose
// AP mode until the AP-mode HW driver path is ported. Kept here as in-progress scaffolding.
void ApfpvStation::startAp(const std::string& ssid, int channel, const std::string& password, bool forceHwBeacon) {
    stopAp(); stopBeaconCal(); disconnect();
    if (!_rtl || !_dev) return;
    auto* rtl = reinterpret_cast<RtlJaguarDevice*>(_rtl);
    auto* dev = reinterpret_cast<RtlUsbAdapter*>(_dev);
    _apWpa2 = !password.empty(); _apChannel = channel; _apSsid = ssid;
    { std::lock_guard<std::mutex> lk(_apMtx); _apStaList.clear(); }
    _apAuth.reset();

    Mac self{};                          // self MAC = BSSID (synthesize if unprogrammed)
    for (int i = 0; i < 6; ++i) self[i] = dev->rtw_read8(0x0610 + i);
    bool bad = true; for (int i=0;i<6;i++) if (self[i]!=0 && self[i]!=0xFF) bad=false;
    if (bad) { const uint8_t m[6]={0x02,0x11,0x22,0x33,0x44,0x55}; for(int i=0;i<6;i++) self[i]=m[i]; }
    _apSelf = self;

    if (_apWpa2) {
        Crypto c = MakeWpa2Crypto();
        _apPmk = c.pbkdf2_pmk(password, (const uint8_t*)ssid.data(), ssid.size());
        FILE* u = fopen("/dev/urandom","rb");                 // random GTK (broadcast key)
        if (u) { size_t r=fread(_apGtk.data(),1,16,u); (void)r; fclose(u); }
        else { for (int i=0;i<16;i++) _apGtk[i] = (uint8_t)((i*97)^0x3c); }
    }

    try {
        SelectedChannel sc = legalApfpvChannel(channel, 20);
        rtl->Init([](const Packet&){}, sc);          // device bring-up (firmware/PHY/channel)
        rtl->SetTxPower(40);
        rtl->StopAsyncRx();                           // then the PUMPED async RX (host event loop
        rtl->StartMonitorAsyncRx(                     // drives the URBs) so apOnRx actually fires —
            [this](const Packet& p){ apOnRx(p.Data.data(), p.Data.size(), p.RxAtrib.rssi[0]); }, sc);
    } catch (...) { return; }
    dev->rtw_write8(0x0522, 0x00);
    // Full AP/master HW bring-up (opt-in via DEVOURER_AP_HWACK). The Jaguar1 (8812AU) path — the
    // one PR #227 upstream marks "unsupported" — is the SOFTWARE-BEACON mode: MSR=AP + ENSWBCN
    // (REG_CR bit8) makes the MAC transmit a host-injected beacon at TBTT and auto-ACK clients.
    // The beacon MUST be queued to the BEACON queue (QSEL 0x10, StationFrameKind::Beacon) — the
    // mgmt queue (0x12) is not radiated once MSR=AP, which is exactly why the old default died.
    // Register map from aircrack rtl8812au include/hal_com_reg.h (verified addresses/bits).
    const bool hwAp = forceHwBeacon || std::getenv("DEVOURER_AP_HWACK") != nullptr;
    if (hwAp) {
        for (int i = 0; i < 4; ++i) dev->rtw_write8(REG_MACID + i, self[i]);
        dev->rtw_write16(REG_MACID + 4, (uint16_t)(self[4] | (self[5] << 8)));
        for (int i = 0; i < 4; ++i) dev->rtw_write8(REG_BSSID + i, self[i]);
        dev->rtw_write16(REG_BSSID + 4, (uint16_t)(self[4] | (self[5] << 8)));
        dev->rtw_write8(MSR, (uint8_t)((dev->rtw_read8(MSR) & 0x0C) | 0x03));  // MSR = AP (MSR_AP)
        dev->rtw_write32(REG_RCR, RCR_APM | RCR_AM | RCR_AB | RCR_ADF | RCR_ACF |
                                  RCR_APP_ICV | RCR_AMF | RCR_HTC_LOC_CTRL | RCR_APP_MIC |
                                  RCR_APP_PHYST_RXFF | RCR_APPFCS | FORCEACK);
        // ---- Beacon-related registers (hal_com_reg.h) --------------------------------------
        dev->rtw_write8 (0x0553, 0x01);           // REG_DUAL_TSF_RST: pulse-reset the TSF timer
        dev->rtw_write16(0x0554, 100);            // REG_BCN_INTERVAL = 100 TU
        dev->rtw_write16(0x0510, 0x660F);         // REG_BCNTCFG (standard)
        dev->rtw_write8 (0x0558, 0x05);           // REG_DRVERLYINT (driver early int, ~5 TU)
        dev->rtw_write8 (0x0559, 0x02);           // REG_BCNDMATIM  (beacon DMA time)
        dev->rtw_write8 (0x055A, 0x02);           // REG_ATIMWND    (ATIM window, 2 TU)
        // ENSWBCN = BIT8 of REG_CR(0x0100) => set BIT0 of 0x0101. Enables SW-beacon TX at TBTT.
        dev->rtw_write8 (0x0101, (uint8_t)(dev->rtw_read8(0x0101) | 0x01));
        // REG_BCN_CTRL(0x0550): EN_BCN_FUNCTION(0x08)|EN_TXBCN_RPT(0x04)|DIS_BCNQ_SUB(0x02).
        // DIS_TSF_UDT left CLEAR so the AP's own TSF free-runs (it owns the BSS clock).
        dev->rtw_write8 (0x0550, 0x0E);
        SCANLOG("AP HW-beacon armed: MSR=AP ENSWBCN=1 BCN_CTRL=0x0e interval=100TU (Jaguar1 SW-beacon path)");
    }

    auto beacon = BuildBeacon(self, ssid, (uint8_t)channel, _apWpa2);
    _apRun.store(true); _beaconRun.store(true);
    _beaconThread = std::thread([this, dev, beacon, hwAp]() {
        std::vector<uint8_t> frame(40 + beacon.size(), 0);
        std::memcpy(frame.data()+40, beacon.data(), beacon.size());
        // In HW-AP mode queue to the BEACON queue (QSEL 0x10); else the legacy visible-beacon path.
        FillStationTxDesc(frame.data(), (uint16_t)beacon.size(), 40, 0,
                          hwAp ? StationFrameKind::Beacon : StationFrameKind::BroadcastMgmt, 0, 0x04);
        while (_beaconRun.load()) {
            dev->rtw_write8(0x0522, 0x00);
            try { dev->send_packet(frame.data(), frame.size()); } catch (...) {}
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
    SCANLOG("AP up: ssid=\"%s\" ch=%d %s bssid=%02x:%02x:%02x:%02x:%02x:%02x", ssid.c_str(), channel,
            _apWpa2?"WPA2-PSK":"OPEN", self[0],self[1],self[2],self[3],self[4],self[5]);
}

void ApfpvStation::stopAp() {
    _apRun.store(false);
    // STOP the beacon TX thread FIRST and join it. Previously stopAp cleared _apRun but NOT
    // _beaconRun, so the beacon thread kept calling send_packet into a device being torn down →
    // the bulk-OUT endpoint jammed (USBDEVFS_BULK OUT failed: -1) and stayed wedged until a
    // physical replug. Joining here guarantees no TX is in flight before we tear the HW down.
    _beaconRun.store(false);
    if (_beaconThread.joinable()) { try { _beaconThread.join(); } catch (...) {} }
    // Tear down the AP HW-beacon state so the MAC stops auto-transmitting from the beacon queue
    // (ENSWBCN + REG_BCN_CTRL) and drop out of master mode. Leaving these set keeps the TX engine
    // busy → wedge. Best-effort (device may already be gone); registers per hal_com_reg.h.
    if (_dev) {
        auto* dev = reinterpret_cast<RtlUsbAdapter*>(_dev);
        try {
            dev->rtw_write8(0x0550, 0x00);                                        // REG_BCN_CTRL: beacon func OFF
            dev->rtw_write8(0x0101, (uint8_t)(dev->rtw_read8(0x0101) & ~0x01));   // clear ENSWBCN (REG_CR bit8)
            dev->rtw_write8(MSR, (uint8_t)(dev->rtw_read8(MSR) & 0x0C));          // MSR -> NoLink
        } catch (...) {}
    }
    stopBeaconCal();
    if (_rtl) { try { reinterpret_cast<RtlJaguarDevice*>(_rtl)->StopAsyncRx(); } catch (...) {} }  // else hangs on exit
    _apAuth.reset();
}

static uint16_t ipChecksum(const uint8_t* p, size_t n);
static std::vector<uint8_t> buildTcp(uint32_t srcIp, uint32_t dstIp, uint16_t sport, uint16_t dport,
                                     uint32_t seq, uint32_t ack, uint8_t flags,
                                     const uint8_t* payload, size_t plen);

// Compute the WPA2 PMK (PBKDF2, slow) ONCE and cache it. The PMK depends only on
// passphrase+SSID, so it survives across (re)connects. Called before the scan so
// becomeStation can use beginCached() and switch the RX to the EAPOL phase within the
// AP's ~4s M1-retry window (inline PBKDF2 in becomeStation missed every M1).
void ApfpvStation::ensurePmk() {
    if (_pmkValid.load()) return;
    auto c = MakeWpa2Crypto();
    _pmkCache = c.pbkdf2_pmk(_params.passphrase,
                             reinterpret_cast<const uint8_t*>(_params.ssid.data()),
                             _params.ssid.size());
    _pmkValid.store(true);
}

// DOWNLINK sink setter. Stores the sink AND applies it to the live RxDeframe if the RX loop
// is already up — so the VpnService can arm/disarm the bridge at runtime (after connect), not
// only before. (connect() also re-applies _ipSink once _rx exists.)
void ApfpvStation::setIpSink(OnRtpFn fn) {
    _ipSink = std::move(fn);
    if (_rx) _rx->setIpSink(_ipSink);
}

// UPLINK general-IP: CCMP-encrypt + TX an arbitrary IPv4 datagram (SSH, any TCP/UDP) the
// VpnService TUN read from the OS. Same encrypt+TX-desc path as DHCP/RTP uplink.
bool ApfpvStation::sendIpPacket(const uint8_t* ip, size_t len) {
    if (!_wpa || !_wpa->ready() || !ip || len < 20) return false;
    auto mpdu = _wpa->buildEncryptedData(ip, len);
    if (mpdu.empty()) return false;
    auto* dev = reinterpret_cast<RtlUsbAdapter*>(_dev);
    std::vector<uint8_t> frame(40 + mpdu.size(), 0);          // 40 = 8812 TX desc
    std::memcpy(frame.data() + 40, mpdu.data(), mpdu.size());
    apfpv::FillStationTxDesc(frame.data(), (uint16_t)mpdu.size(), 40,
                             1, apfpv::StationFrameKind::CcmpData, 7, 0x04);
    return dev->sendStationFrameSync(frame.data(), frame.size());
}

uint32_t ApfpvStation::leaseIp() const       { return _dhcp ? _dhcp->lease().ip : 0; }
uint32_t ApfpvStation::leaseServerIp() const { return _dhcp ? _dhcp->lease().server : 0; }

// Minimal TCP/HTTP GET over the link — proves the dongle carries general TCP (so SSH/REST work).
std::string ApfpvStation::httpGet(uint32_t dstIp, uint16_t port, const std::string& path, int timeoutMs) {
    if (!_rx || !_dhcp || !_dhcp->lease().valid) return "";
    using namespace std::chrono;
    uint32_t srcIp = _dhcp->lease().ip;
    const uint16_t sport = 0xC350;                       // ephemeral source port 50000
    { std::lock_guard<std::mutex> lk(_ipQMtx); _ipQ.clear(); }
    _ipCapture.store(true);
    _rx->setIpSink([this](const uint8_t* p, size_t n){
        if (!_ipCapture.load()) return;
        std::lock_guard<std::mutex> lk(_ipQMtx); _ipQ.emplace_back(p, p + n);
    });
    auto poll = [&](uint8_t wantFlags, uint32_t& srvSeqOut, std::string& data)->bool {
        auto t0 = steady_clock::now();
        while (steady_clock::now() - t0 < milliseconds(timeoutMs)) {
            std::vector<std::vector<uint8_t>> batch;
            { std::lock_guard<std::mutex> lk(_ipQMtx); batch.swap(_ipQ); }
            for (auto& pk : batch) {
                if (pk.size() < 40 || pk[9] != 6) continue;                 // IPv4 TCP
                uint32_t s = ((uint32_t)pk[12]<<24)|((uint32_t)pk[13]<<16)|((uint32_t)pk[14]<<8)|pk[15];
                if (s != dstIp) continue;
                size_t ihl = (size_t)(pk[0]&0xf)*4; if (pk.size() < ihl + 20) continue;
                const uint8_t* t = pk.data() + ihl;
                if ((uint16_t)((t[0]<<8)|t[1]) != port) continue;           // src port = server
                srvSeqOut = ((uint32_t)t[4]<<24)|((uint32_t)t[5]<<16)|((uint32_t)t[6]<<8)|t[7];
                size_t toff = (size_t)(t[12]>>4)*4;
                if (pk.size() > ihl + toff) data.append((const char*)(t + toff), pk.size() - ihl - toff);
                if (t[13] & wantFlags) return true;
            }
            std::this_thread::sleep_for(milliseconds(10));
        }
        return false;
    };
    uint32_t srvSeq = 0, seq = 1000; std::string resp, ignore;
    auto syn = buildTcp(srcIp, dstIp, sport, port, seq, 0, 0x02, nullptr, 0);        // SYN
    sendIpPacket(syn.data(), syn.size());
    if (!poll(0x12, srvSeq, ignore)) { _ipCapture.store(false); return ""; }         // wait SYN|ACK
    seq = 1001;
    std::string req = "GET " + path + " HTTP/1.0\r\nHost: vtx\r\n\r\n";
    auto get = buildTcp(srcIp, dstIp, sport, port, seq, srvSeq + 1, 0x18,            // ACK + GET (PSH|ACK)
                        (const uint8_t*)req.data(), req.size());
    sendIpPacket(get.data(), get.size());
    poll(0x01, srvSeq, resp);                                                        // gather until FIN/timeout
    auto fin = buildTcp(srcIp, dstIp, sport, port, seq + (uint32_t)req.size(), srvSeq + 1, 0x11, nullptr, 0);
    sendIpPacket(fin.data(), fin.size());                                            // FIN|ACK (polite close)
    _ipCapture.store(false);
    return resp;
}

// ---- raw bidirectional TCP (relay transport for SSH over the dongle) --------------------
// Parse an inbound IPv4 packet and, if it belongs to the active connection (from _tcDst:_tcDport
// to us:_tcSport, TCP), extract its seq/flags/payload.
bool ApfpvStation::matchTcp(const std::vector<uint8_t>& pk, uint32_t& seq, uint8_t& flags,
                            const uint8_t*& payload, size_t& plen) {
    if (pk.size() < 40 || pk[9] != 6) return false;                        // IPv4 TCP
    uint32_t s = ((uint32_t)pk[12]<<24)|((uint32_t)pk[13]<<16)|((uint32_t)pk[14]<<8)|pk[15];
    if (s != _tcDst) return false;
    size_t ihl = (size_t)(pk[0]&0xf)*4; if (pk.size() < ihl + 20) return false;
    const uint8_t* t = pk.data() + ihl;
    uint16_t sp = (uint16_t)((t[0]<<8)|t[1]), dp = (uint16_t)((t[2]<<8)|t[3]);
    if (sp != _tcDport || dp != _tcSport) return false;
    seq = ((uint32_t)t[4]<<24)|((uint32_t)t[5]<<16)|((uint32_t)t[6]<<8)|t[7];
    flags = t[13];
    size_t toff = (size_t)(t[12]>>4)*4;
    payload = t + toff;
    plen = (pk.size() > ihl + toff) ? (pk.size() - ihl - toff) : 0;
    return true;
}

bool ApfpvStation::tcpConnect(uint32_t dstIp, uint16_t dport, int timeoutMs) {
    if (!_rx || !_dhcp || !_dhcp->lease().valid) return false;
    _tcSrc = _dhcp->lease().ip; _tcDst = dstIp; _tcDport = dport;
    _tcSport = 0xC351;                                   // ephemeral src port 50001
    _tcSeq = 2000; _tcAck = 0; _tcOpen = false;
    { std::lock_guard<std::mutex> lk(_ipQMtx); _ipQ.clear(); }
    _ipCapture.store(true);
    _rx->setIpSink([this](const uint8_t* p, size_t n){
        if (!_ipCapture.load()) return;
        std::lock_guard<std::mutex> lk(_ipQMtx); _ipQ.emplace_back(p, p + n);
    });
    auto syn = buildTcp(_tcSrc,_tcDst,_tcSport,_tcDport,_tcSeq,0,0x02,nullptr,0);
    sendIpPacket(syn.data(), syn.size());
    using namespace std::chrono; auto t0 = steady_clock::now(); auto tRetx = t0;
    while (steady_clock::now() - t0 < milliseconds(timeoutMs)) {
        if (steady_clock::now() - tRetx > milliseconds(400)) {         // SYN retransmit (userspace TX drops)
            sendIpPacket(syn.data(), syn.size()); tRetx = steady_clock::now();
        }
        std::vector<std::vector<uint8_t>> batch;
        { std::lock_guard<std::mutex> lk(_ipQMtx); batch.swap(_ipQ); }
        for (auto& pk : batch) {
            uint32_t seq; uint8_t flags; const uint8_t* pl; size_t pn;
            if (!matchTcp(pk, seq, flags, pl, pn)) continue;
            if ((flags & 0x12) == 0x12) {                // SYN|ACK
                _tcAck = seq + 1;                        // ack their SYN
                _tcSeq += 1;                             // our SYN consumed a seq
                auto ack = buildTcp(_tcSrc,_tcDst,_tcSport,_tcDport,_tcSeq,_tcAck,0x10,nullptr,0);
                sendIpPacket(ack.data(), ack.size());
                _tcOpen = true; return true;
            }
            if (flags & 0x04) { fprintf(stderr, "[tcp] RST from %u.%u.%u.%u:%u — port closed\n",
                (_tcDst>>24)&255,(_tcDst>>16)&255,(_tcDst>>8)&255,_tcDst&255,_tcDport);
                _ipCapture.store(false); return false; }   // RST
        }
        std::this_thread::sleep_for(milliseconds(3));
    }
    fprintf(stderr, "[tcp] no SYN|ACK from %u.%u.%u.%u:%u within timeout\n",
            (_tcDst>>24)&255,(_tcDst>>16)&255,(_tcDst>>8)&255,_tcDst&255,_tcDport);
    _ipCapture.store(false); return false;
}

int ApfpvStation::tcpPoll(std::string& out, int timeoutMs) {
    if (!_tcOpen) return -1;
    using namespace std::chrono; auto t0 = steady_clock::now(); int appended = 0;
    do {
        std::vector<std::vector<uint8_t>> batch;
        { std::lock_guard<std::mutex> lk(_ipQMtx); batch.swap(_ipQ); }
        for (auto& pk : batch) {
            uint32_t seq; uint8_t flags; const uint8_t* pl; size_t pn;
            if (!matchTcp(pk, seq, flags, pl, pn)) continue;
            if (flags & 0x04) { _tcOpen = false; return appended > 0 ? appended : -1; }   // RST
            if (pn) {
                if (seq == _tcAck) {                     // in-order data
                    out.append((const char*)pl, pn);
                    _tcAck += (uint32_t)pn; appended += (int)pn;
                }
                // in-order or duplicate: (re-)ACK so the peer advances / retransmits
                auto ack = buildTcp(_tcSrc,_tcDst,_tcSport,_tcDport,_tcSeq,_tcAck,0x10,nullptr,0);
                sendIpPacket(ack.data(), ack.size());
            }
            if ((flags & 0x01) && seq + (uint32_t)pn == _tcAck) {           // FIN (in order)
                _tcAck += 1;
                auto fa = buildTcp(_tcSrc,_tcDst,_tcSport,_tcDport,_tcSeq,_tcAck,0x11,nullptr,0);
                sendIpPacket(fa.data(), fa.size());
                _tcOpen = false; return appended > 0 ? appended : -1;
            }
        }
        if (appended > 0) return appended;
        std::this_thread::sleep_for(milliseconds(2));
    } while (steady_clock::now() - t0 < milliseconds(timeoutMs));
    return 0;
}

bool ApfpvStation::tcpSend(const uint8_t* data, size_t n) {
    if (!_tcOpen || !n) return false;
    size_t off = 0;
    while (off < n) {
        size_t chunk = (n - off > 1024) ? 1024 : (n - off);
        auto seg = buildTcp(_tcSrc,_tcDst,_tcSport,_tcDport,_tcSeq,_tcAck,0x18,data+off,chunk);
        sendIpPacket(seg.data(), seg.size());
        _tcSeq += (uint32_t)chunk; off += chunk;
    }
    return true;
}

void ApfpvStation::tcpClose() {
    if (_tcOpen) {
        auto fin = buildTcp(_tcSrc,_tcDst,_tcSport,_tcDport,_tcSeq,_tcAck,0x11,nullptr,0);
        sendIpPacket(fin.data(), fin.size());
        _tcSeq += 1; _tcOpen = false;
    }
    _ipCapture.store(false);
}

static uint16_t ipChecksum(const uint8_t* p, size_t n) {
    uint32_t s = 0;
    for (size_t i = 0; i + 1 < n; i += 2) s += (uint32_t)((p[i] << 8) | p[i+1]);
    if (n & 1) s += (uint32_t)(p[n-1] << 8);
    while (s >> 16) s = (s & 0xffff) + (s >> 16);
    return (uint16_t)~s;
}
// Wrap a BOOTP/DHCP payload in IPv4+UDP (0.0.0.0:68 -> 255.255.255.255:67). The ApfpvDhcp
// emits only the BOOTP message (RX strips IP/UDP too), but buildEncryptedData expects a
// COMPLETE IPv4 packet — without these headers the AP's IP stack sees "IP version 0" and
// drops the DISCOVER before dnsmasq, so no lease is ever offered.
static std::vector<uint8_t> wrapDhcpUdpIp(const uint8_t* bootp, size_t blen) {
    size_t udpLen = 8 + blen, ipLen = 20 + udpLen;
    std::vector<uint8_t> p(ipLen, 0);
    p[0]=0x45; p[2]=(uint8_t)(ipLen>>8); p[3]=(uint8_t)(ipLen&0xff);  // ver4/ihl5, total len
    p[8]=64; p[9]=17;                                   // ttl=64, proto=UDP (src stays 0.0.0.0)
    p[16]=p[17]=p[18]=p[19]=0xff;                       // dst = 255.255.255.255
    uint16_t ick = ipChecksum(p.data(), 20);
    p[10]=(uint8_t)(ick>>8); p[11]=(uint8_t)(ick&0xff);
    p[21]=68; p[23]=67;                                 // UDP sport 68, dport 67
    p[24]=(uint8_t)(udpLen>>8); p[25]=(uint8_t)(udpLen&0xff);  // UDP len; checksum 0 (optional v4)
    std::memcpy(p.data()+28, bootp, blen);
    return p;
}

// Build an IPv4 + TCP segment (flags: SYN=0x02 ACK=0x10 PSH=0x08 FIN=0x01 RST=0x04).
static std::vector<uint8_t> buildTcp(uint32_t srcIp, uint32_t dstIp, uint16_t sport, uint16_t dport,
                                     uint32_t seq, uint32_t ack, uint8_t flags,
                                     const uint8_t* payload, size_t plen) {
    size_t tcpLen = 20 + plen, ipLen = 20 + tcpLen;
    std::vector<uint8_t> p(ipLen, 0);
    p[0]=0x45; p[2]=(uint8_t)(ipLen>>8); p[3]=(uint8_t)(ipLen&0xff); p[8]=64; p[9]=6;   // proto TCP
    p[12]=(uint8_t)(srcIp>>24);p[13]=(uint8_t)(srcIp>>16);p[14]=(uint8_t)(srcIp>>8);p[15]=(uint8_t)srcIp;
    p[16]=(uint8_t)(dstIp>>24);p[17]=(uint8_t)(dstIp>>16);p[18]=(uint8_t)(dstIp>>8);p[19]=(uint8_t)dstIp;
    uint16_t ick = ipChecksum(p.data(), 20); p[10]=(uint8_t)(ick>>8); p[11]=(uint8_t)(ick&0xff);
    uint8_t* t = p.data()+20;
    t[0]=(uint8_t)(sport>>8);t[1]=(uint8_t)sport;t[2]=(uint8_t)(dport>>8);t[3]=(uint8_t)dport;
    t[4]=(uint8_t)(seq>>24);t[5]=(uint8_t)(seq>>16);t[6]=(uint8_t)(seq>>8);t[7]=(uint8_t)seq;
    t[8]=(uint8_t)(ack>>24);t[9]=(uint8_t)(ack>>16);t[10]=(uint8_t)(ack>>8);t[11]=(uint8_t)ack;
    t[12]=0x50; t[13]=flags; t[14]=0xff; t[15]=0xff;       // data-offset 5, window 0xffff
    if (plen) std::memcpy(t+20, payload, plen);
    uint32_t s = ((srcIp>>16)&0xffff)+(srcIp&0xffff)+((dstIp>>16)&0xffff)+(dstIp&0xffff)+6u+(uint32_t)tcpLen;
    for (size_t i=0;i+1<tcpLen;i+=2) s += (uint32_t)((t[i]<<8)|t[i+1]);
    if (tcpLen&1) s += (uint32_t)(t[tcpLen-1]<<8);
    while (s>>16) s=(s&0xffff)+(s>>16);
    uint16_t ck=(uint16_t)~s; t[16]=(uint8_t)(ck>>8); t[17]=(uint8_t)(ck&0xff);
    return p;
}

// Build an IPv4 + UDP datagram (UDP checksum 0 = optional in v4). Used to carry the LQ-feedback
// percentage to the VTX aalink over the dongle's own IP stack (see the LqFeedback sink) — the host
// socket path can't reach the VTX on the libusb (Win/WSL) dongle path.
static std::vector<uint8_t> buildUdp(uint32_t srcIp, uint32_t dstIp, uint16_t sport, uint16_t dport,
                                     const uint8_t* payload, size_t plen) {
    size_t udpLen = 8 + plen, ipLen = 20 + udpLen;
    std::vector<uint8_t> p(ipLen, 0);
    p[0]=0x45; p[2]=(uint8_t)(ipLen>>8); p[3]=(uint8_t)(ipLen&0xff); p[8]=64; p[9]=17;  // proto UDP
    p[12]=(uint8_t)(srcIp>>24);p[13]=(uint8_t)(srcIp>>16);p[14]=(uint8_t)(srcIp>>8);p[15]=(uint8_t)srcIp;
    p[16]=(uint8_t)(dstIp>>24);p[17]=(uint8_t)(dstIp>>16);p[18]=(uint8_t)(dstIp>>8);p[19]=(uint8_t)dstIp;
    uint16_t ick = ipChecksum(p.data(), 20); p[10]=(uint8_t)(ick>>8); p[11]=(uint8_t)(ick&0xff);
    uint8_t* u = p.data()+20;
    u[0]=(uint8_t)(sport>>8);u[1]=(uint8_t)sport;u[2]=(uint8_t)(dport>>8);u[3]=(uint8_t)dport;
    u[4]=(uint8_t)(udpLen>>8);u[5]=(uint8_t)(udpLen&0xff);   // UDP length; checksum left 0 (optional)
    if (plen) std::memcpy(u+8, payload, plen);
    return p;
}

// The gated establishment chain (arm -> auth -> assoc -> WPA2 -> DHCP -> stream).
// Returns true only on a held, streaming link. Used for both initial connect
// AND each supervisor reconnect attempt.
bool ApfpvStation::runConnectChain() {
    _pendingAddbaTids = 0;  // fresh ADDBA debounce per connection
    _raCfgSent.store(false); // re-send MACID_CFG once on this (re)connection
    auto& dev = *reinterpret_cast<RtlUsbAdapter*>(_dev);
    auto& rm  = *reinterpret_cast<RadioManagementModule*>(_rm);
    auto sendFrame = [&dev](const std::vector<uint8_t>& f) {
        // Station TX coexisting with the kernel-tasklet async RX. libusb userspace
        // cannot push a bulk-OUT while bulk-IN URBs are pending (WinUSB + USB/IP
        // both serialise OUT behind INs, unlike the Linux kernel USB core), so a
        // bare async send_packet times out (status 2, 0 bytes). sendStationFrameSync
        // drains the RX pool, does the proven sync bulk-OUT, then re-arms RX fast
        // enough to catch the AP's reply. DEVOURER_BARE_ASYNC_TX forces the old
        // (broken) path for A/B; DEVOURER_SYNC_IO is the legacy all-sync mode.
        if (std::getenv("DEVOURER_BARE_ASYNC_TX"))
            return dev.send_packet(const_cast<uint8_t*>(f.data()), f.size());
        if (std::getenv("DEVOURER_SYNC_IO"))
            return dev.bulk_send_sync(const_cast<uint8_t*>(f.data()), f.size(), 200) == 0;
        return dev.sendStationFrameSync(const_cast<uint8_t*>(f.data()), f.size());
    };
    StationMode sta(dev, rm, sendFrame);
    // Use user-selected bandwidth: 40 or 80 MHz. If the selected bandwidth fails
    // to connect, set DEVOURER_FORCE_20MHZ=1 to force 20 MHz (proven stable).
    ChannelWidth_t userBw = _params.bandwidth >= 80 ? CHANNEL_WIDTH_80
                          : _params.bandwidth >= 40 ? CHANNEL_WIDTH_40
                          : CHANNEL_WIDTH_20;
    if (std::getenv("DEVOURER_FORCE_20MHZ")) userBw = CHANNEL_WIDTH_20;
    // Arm at the AP's ACTUAL width/center. This is why 80 MHz worked but 40/20 didn't: at 80 MHz
    // the station tuned to center 42 and MATCHED the AP, so it RXed the AP's HT/VHT data — incl.
    // the EAPOL M1 — and the 4-way completed. Arming narrower (center 36/38) than the AP's real
    // operating width mis-captures its aggregated data frames -> M1 missed -> handshake times out.
    // Wide-center arming only ever failed because the IQK wedged the synth; IQK is now disabled,
    // so we can (and must) match the AP's width here. DEVOURER_FORCE_20MHZ still forces 20 for A/B.
    sta.setConnectWidth(userBw);
    MacAddr self{}, bssid{};
    // NOTE: our MAC is read from REG_MACID AFTER the device is brought up (below).
    // Reading it before Init returned EFUSE-unloaded garbage (e.g. ea:ea:ea:...).

    // Bring the PHY/RF up BEFORE any channel change (scanForSsid + arm call
    // set_channel_bwmode -> phy_SwChnl8812, which null-derefs if never Init'd),
    // AND wire the RX so discovery (beacons -> onScanFrame) and the arm handshake
    // (auth/assoc/deauth -> onMgmtFrame) actually receive. Previously this used a
    // no-op processor, so _scanResult and _gotAuthResp were never set -> the link
    // always failed at discovery. The real RxDeframe RX is wired after handshake.
    { std::lock_guard<std::mutex> lk(_scanMtx); _scanSeen.clear(); }   // diag: SSIDs heard
    // Run the device RX loop on its OWN thread. RtlJaguarDevice::Init is a BLOCKING
    // infinite read loop ("Listening air..."); calling it inline froze the connect
    // on the hint channel so scanForSsid never hopped (it only ever heard ch40).
    // The connect thread now drives channel hops + arm while this thread feeds RX.
    // One dispatch routes by phase: discovery/arm -> StationMode; streaming -> RxDeframe.
    if (_rtl) {
        auto* rtl = reinterpret_cast<RtlJaguarDevice*>(_rtl);
        // shared_ptr<atomic> survives sta destruction; dispatch checks before calling
        auto alive = std::make_shared<std::atomic<bool>>(true);
        sta._alivePtr = alive;
        StationMode* sp = &sta;
        auto dispatch = [sp, this, alive](const Packet& pkt){
            _rxReady.store(true);
            if (_rxPhase.load() == 1) {
                // STREAMING/handshake path: routes to RxDeframe via _rx (owned by ApfpvStation,
                // outlives the per-attempt StationMode). MUST NOT be gated by `alive`: when
                // runConnectChain returns at the Streaming transition, the local `sta` is
                // destroyed (alive=false) — gating here dropped EVERY data frame -> RX silence
                // -> watchdog -> reconnect loop. RxDeframe doesn't touch `sp`, so it's safe.
                // NOTE: NO per-packet fprintf here — at 65Mbps that's 5600 blocking writes/s on
                // the RX dispatch thread, which stalls RX and shreds frame assembly.
                RxDeframe* rx = _rx.get(); if (rx) rx->onPacket(pkt); return;
            }
            // DISCOVERY/ARM path below calls into `sp` (the StationMode). Guard it: if `sta`
            // was destroyed, sp is dangling — TOCTOU-safe via the shared `alive` flag.
            if (!alive->load()) return;
            size_t n = pkt.Data.size();
            if (n < 24 || n > 4096) return;
            // Copy the frame off the RX span before parsing: pkt.Data points into the
            // USB transfer buffer, which the async RX path can recycle while the beacon
            // parser is still walking it -> use-after-free -> SIGSEGV in ScanProbe on
            // busy RF. A stable local copy removes that window.
            std::vector<uint8_t> framebuf(pkt.Data.begin(), pkt.Data.end());
            const uint8_t* f = framebuf.data();
            uint16_t fc = (uint16_t)(f[0] | (f[1] << 8));
            if (((fc>>2)&3)==0 && n >= 30) {
                uint8_t sub = (fc>>4)&0xF;
                if (sub==0x1 && n >= 24 + 6) {  // assoc response: parse HT/VHT Op IEs for BW
                    // rtw_ies_get_chbw: extract bandwidth from AP's HT/VHT Operation IEs.
                    // Without this, we never know what bandwidth the AP actually assigned us
                    // and stay at 20 MHz even when the AP supports 40/80 MHz.
                    uint8_t assocBw = 0, assocOff = 0; // 0=20, 1=40, 2=80 MHz
                    const uint8_t* ie = f + 24 + 6;  // skip mgmt hdr + fixed assoc resp (6B)
                    size_t ieLen = n - 24 - 6;
                    for (size_t pos = 0; pos + 2 <= ieLen; ) {
                        uint8_t id = ie[pos], len = ie[pos+1];
                        if (pos + 2 + len > ieLen) break;
                        if (id == 0x3D && len >= 22) {  // HT Capabilities IE
                            if (ie[pos+2+1] & 0x04) assocBw = 1;  // Supported Chan Width Set
                        } else if (id == 0x3E && len >= 3) {  // HT Operation IE
                            if (assocBw == 1) {
                                uint8_t staChWidth = ie[pos+2+2] & 0x01;  // STA Chan Width
                                if (!staChWidth) assocBw = 0;  // AP says 20 MHz only
                                uint8_t secOff = (ie[pos+2+1] & 0x03);  // Secondary Chan Offset
                                if (secOff == 1) assocOff = 2;  // SCA -> UPPER
                                else if (secOff == 3) assocOff = 1;  // SCB -> LOWER
                            }
                        } else if (id == 0xC0 && len >= 3) {  // VHT Operation IE
                            uint8_t chWidth = ie[pos+2];  // 0=20/40, 1=80, 2=160
                            if (chWidth >= 1 && assocBw >= 1) assocBw = 2;  // 80 MHz
                        }
                        pos += 2 + len;
                    }
                    if (assocBw > 0) {
                        // ADOPT the AP's advertised bandwidth automatically — the band + channel
                        // already come from the scan, and the operating width should match what the
                        // AP actually provides (a fixed user pref of 20 would waste an 80MHz AP). The
                        // firmware RA adapts the MCS within the width, so wider is strictly better when
                        // the AP offers it. DEVOURER_MAX_BW=20|40|80 caps it only if you deliberately
                        // want to limit the width; otherwise it follows the AP.
                        int apBw = assocBw==2?80 : assocBw==1?40 : 20;
                        int cap = 160; if (const char* m = std::getenv("DEVOURER_MAX_BW")) cap = atoi(m);
                        _params.bandwidth = apBw < cap ? apBw : cap;
                        SCANLOG("assoc-response: AP bw=%dMHz -> using %dMHz (auto; off=%d cap=%d)",
                            apBw, (int)_params.bandwidth, (int)assocOff, cap);
                    }
                }
            }
            sp->onScanFrame(f, n);                  // beacons -> _scanResult
            std::string ss; ApInfo bi;
            if (ScanProbe::parseAnyBeacon(f, n, ss, bi) && !ss.empty()) {
                std::lock_guard<std::mutex> lk(_scanMtx);
                if (_scanSeen.insert(ss).second)
                    SCANLOG("discovery heard \"%s\" ch%d", ss.c_str(), bi.channel);
            }
            MacAddr a1{}, a2{};
            std::memcpy(a1.b.data(), f + 4,  6);
            std::memcpy(a2.b.data(), f + 10, 6);
            sp->onMgmtFrame(fc, a1, a2);            // auth/assoc/deauth -> flags
        };
        _rxPhase.store(0); _rxReady.store(false);
        // WFB-style bandwidth from the start: use user-selected bw, not hardcoded 20.
        // The kernel's init_hw_mlme_ext does the same — sets cur_bwmode before auth.
        uint8_t initCh = (uint8_t)(_params.channel > 0 ? _params.channel : 40);
        // ALWAYS bring up at the 20 MHz primary. The Init runs an IQK on the init channel;
        // at a 40/80 MHz center (e.g. ch36@40 -> center 38) that IQK wedges the RF synth
        // (RF_CH=0xea) and — critically — the wedge PERSISTS across app restarts AND survives
        // USBDEVFS_RESET (only a physical replug clears it). After the first assoc adopts the
        // AP's 40 MHz, _params.bandwidth=40, so a bandwidth-driven initBw would re-wedge on
        // every reconnect. Keep every IQK at a 20 MHz center; the real width is applied
        // post-handshake (same channel, no band change -> no fresh IQK -> no wedge).
        ChannelWidth_t initBw = CHANNEL_WIDTH_20;
        uint8_t initOff = 0;
        SelectedChannel initSel{ .Channel=initCh, .ChannelOffset=initOff, .ChannelWidth=initBw };
        if (std::getenv("DEVOURER_SYNC_IO")) {
            // LEGACY blocking sync RX loop (monopolises libusb -> async TX can't
            // run; kept only for A/B). Spawn on its own thread.
            rtl->should_stop = true;
            if (_rxThread.joinable()) _rxThread.join();
            rtl->should_stop = false;
            _rxThread = std::thread([rtl, dispatch, initSel]() {
                try { rtl->Init(dispatch, initSel); } catch (...) {}
            });
        } else {
            // KERNEL-STYLE async RX: N bulk-IN URBs kept in flight, delivered from
            // the caller's libusb_handle_events loop. No blocking read, so the
            // async TX (send_packet) completes -> station auth/assoc actually
            // radiates. Restart cleanly on each (re)connect.
            rtl->StopAsyncRx();
            try { rtl->StartMonitorAsyncRx(dispatch, initSel); } catch (...) {}
        }
        // wait until the RX loop is live (device brought up) or a short timeout
        using namespace std::chrono;
        auto t0 = steady_clock::now();
        while (!_rxReady.load() && steady_clock::now() - t0 < seconds(4))
            std::this_thread::sleep_for(milliseconds(50));

        // ---- RF-WEDGE AUTO-RECOVERY ----------------------------------------
        // A full cold software re-init (power-on + hw_reset + firmware + PHY/RF
        // tables) runs every bring-up but does NOT clear an RF-synth wedge (RF
        // 0x18 reads 0xea -> can't tune ANY channel -> scan finds nothing / auth
        // TX fails). Proven on-device. The only lever left short of a physical
        // replug is a USB port reset (USBDEVFS_RESET), which cycles the chip's
        // reset the way VBUS removal does. On detecting the wedge, reset + re-init
        // and re-check, up to a few times. Gate off with DEVOURER_NO_USB_RESET.
        if (!std::getenv("DEVOURER_NO_USB_RESET")) {
            // USBDEVFS_RESET (the only USB reset reachable via the Java-wrapped fd) is a
            // protocol reset, NOT a VBUS power-cycle — proven on-device to NOT clear an
            // existing RF wedge (rc=0 but RF_CH stays 0xea). So this is best-effort only;
            // the real fix is prevention (all IQK at 20 MHz centers). 1 attempt by default.
            int maxTries = 1;
            if (const char* e = std::getenv("DEVOURER_USB_RESET_TRIES")) maxTries = atoi(e);
            for (int attempt = 1; attempt <= maxTries && rm.rf_wedged(); ++attempt) {
                SCANLOG("RF WEDGED (RF_CH=0xea) after bring-up -> USB port reset attempt %d/%d",
                        attempt, maxTries);
                rtl->StopAsyncRx();
                std::this_thread::sleep_for(milliseconds(50));
                int rc = dev.reset_device();   // libusb_reset_device / USBDEVFS_RESET
                SCANLOG("USB reset_device() rc=%d (0=ok; <0 = libusb err, fd likely re-enumerated)", rc);
                std::this_thread::sleep_for(milliseconds(300));  // let the chip re-appear
                _rxPhase.store(0); _rxReady.store(false);
                try { rtl->StartMonitorAsyncRx(dispatch, initSel); }   // full re-init on the reset chip
                catch (...) { SCANLOG("re-init after USB reset threw (fd invalid?) — bailing"); break; }
                auto tr = steady_clock::now();
                while (!_rxReady.load() && steady_clock::now() - tr < seconds(4))
                    std::this_thread::sleep_for(milliseconds(50));
                SCANLOG("post-reset bring-up: rf_wedged=%d", rm.rf_wedged() ? 1 : 0);
            }
            if (rm.rf_wedged())
                SCANLOG("RF STILL WEDGED after %d USB resets — likely needs a physical replug", maxTries);
        }
        // --------------------------------------------------------------------

        // TX POWER: the station auth/assoc TX uses the same chip/PA the beacon-cal
        // (startBeaconCal -> SetTxPower) and the verified-working monitor injection
        // (txdemo SetTxPower(40)) drive — but the connect path NEVER set it, so the
        // auth-req keyed the PA at unset/zero power = on-air silence. Set a solid
        // index now so the frame actually radiates. (Java applies the legal-EIRP
        // clamp via nativeStaSetTxPower; 40 here is the txdemo working value.)
        try { rtl->SetTxPower(40); SCANLOG("station TX power set to idx 40"); }
        catch (...) { SCANLOG("station SetTxPower threw"); }
    }

    // Our MAC, read NOW that the device is up (REG_MACID populated from EFUSE).
    // Used as addr2 in auth/assoc + for WPA2 PTK; a garbage MAC makes the AP
    // reply to a bogus address. 0x0610 = REG_MACID.
    for (int i = 0; i < 6; ++i) self.b[i] = dev.rtw_read8(0x0610 + i);
    bool bad = true; for (int i=0;i<6;i++){ if(self.b[i]!=0 && self.b[i]!=0xFF) bad=false; }
    if (bad) { self.b[0]=0x02; self.b[1]=0x11; self.b[2]=0x22;
               self.b[3]=0x33; self.b[4]=0x44; self.b[5]=0x55; }
    ensurePmk();   // precompute the PBKDF2 PMK now (cached) so becomeStation is instant
    dev.setTxFastPath(false);  // connect handshake wants the per-TX drain diag; Streaming flips it true

    // DISCOVERY: scan beacons for the SSID -> BSSID + channel + negotiated RSN.
    // Removes the hardcoded-channel / empty-BSSID / fixed-cipher assumptions
    // (security + discovery parity with the kernel wpa_supplicant the VRX uses).
    ApInfo ap;
    if (_params.haveBssid) {
        // PHONE-ASSISTED: the phone already resolved the AP's BSSID + channel, so
        // skip the dongle sweep entirely and arm straight to it (CCMP/PSK default).
        for (int i = 0; i < 6; ++i) bssid.b[i] = _params.bssid[i];
        sta.setNegotiatedCipher(0x000FAC04, 0x000FAC04);
        SCANLOG("phone-assisted: BSSID %02x:%02x:%02x:%02x:%02x:%02x ch%d — skipping sweep",
                bssid.b[0],bssid.b[1],bssid.b[2],bssid.b[3],bssid.b[4],bssid.b[5], _params.channel);
    } else if (_params.scan) {
        set(State::Scanning);
        // Sweep beacons for the SSID -> BSSID + channel + negotiated RSN (now works:
        // the RX thread feeds frames while this thread hops channels).
        rm.setScanMode(true);   // suppress IQK during the hop (band-cross IQK wedges the RF -> found=0)
        ap = sta.scanForSsid(_params.ssid.c_str(), _params.channel, /*ms*/300);
        rm.setScanMode(false);  // arm channel gets a clean, settled IQK below
        SCANLOG("scan: \"%s\" found=%d ch=%d", _params.ssid.c_str(), ap.found?1:0, ap.channel);
        if (!ap.found) { set(State::FailNoAp); return false; }
        bssid.b = ap.bssid;
        // Persist the discovered BSSID into _params so downstream users get the REAL AP
        // MAC — notably setSecCamKey (HW-decrypt CAM entries) and the ADDBA frames, which
        // read _params.bssid. Before this it stayed zero-initialized on the scan path, so
        // the HW-decrypt CAM entry was written with MAC 00:00:.. → the security engine
        // found no key for A2=<AP> → SWDEC=1 → HW decrypt silently never engaged. (This does
        // NOT set _params.haveBssid, so the supervisor still re-scans on reconnect.)
        _params.bssid = ap.bssid;
        _params.channel = ap.channel ? ap.channel : _params.channel;
        // NOTE: do NOT set _params.haveBssid here. Persisting it made every
        // supervisor RECONNECT skip the scan and jump straight to auth — and the
        // auth TX then FAILS (arm result 3 = TXFAIL_NoAuth), looping forever. The
        // scan both re-confirms the channel AND warms the TX path; the parseBeacon
        // crash it was meant to avoid is now covered by the _alive guard + try/catch.
        // haveBssid stays reserved for explicit phone-assisted connect only.
        sta.setNegotiatedCipher(ScanProbe::chooseCipher(ap.pairwise),
                                ap.rsnPresent ? ap.groupCipher : 0x000FAC04);
        SCANLOG("CIPHERS: pairwise=0x%08x group=0x%08x (CCMP=0x000FAC04 TKIP=0x000FAC02)",
                ScanProbe::chooseCipher(ap.pairwise), ap.rsnPresent ? ap.groupCipher : 0x000FAC04);
    }

    // RETUNE to the AP's actual channel before the arm. The scanForSsid hop leaves
    // the radio on the last scanned channel (e.g. ch46), not the AP's channel (ch44).
    // arm() uses _rm.current_channel() which must equal the AP's actual channel or
    SCANLOG("pre-arm tune: want=%d current=%d", (int)_params.channel, (int)rm.current_channel());
    // the 40 MHz offset selects the wrong secondary -> AP sends 20 MHz frames.
    rm.set_channel_bwmode((uint8_t)_params.channel, 0, CHANNEL_WIDTH_20);
    SCANLOG("pre-arm tuned: now=%d", (int)rm.current_channel());
    sta.setPmf(_params.pmf);              // 802.11w RSN caps in the assoc-req (0 = off, no change)
    set(State::Arming);
    sta.setPhaseCb([this](int p){ set(p == 1 ? State::Authenticating : State::Associating); });
    // hold=0: return the instant assoc succeeds. The AP starts the 4-way (M1) immediately
    // after assoc and only retries for ~4s; any hold here keeps the RX in the arm phase
    // (data frames dropped) so M1 is missed. becomeStation must switch RX to the EAPOL
    // phase ASAP. Link confirmation/deauth handling is the supervisor's job afterward.
    auto r = sta.runProbe(self, bssid, _params.ssid.c_str(), /*hold*/0);
    SCANLOG("arm result: %d (0=GO 1=Deauth 2=NoAssocResp 3=TXFAIL_NoAuth 4=Error)", (int)r);
    switch (r) {
        case StationMode::Result::TXFAIL_NoAuthResp: set(State::FailTx);   return false;
        case StationMode::Result::NOGO_Deauthed:     set(State::FailNoAck); return false;
        case StationMode::Result::NOGO_NoAssocResp:  set(State::FailAuth);  return false;
        case StationMode::Result::Error:             set(State::FailNoAp);  return false;
        case StationMode::Result::GO_LinkHeld: break;
    }
    // BANDWIDTH UPGRADE is DEFERRED until AFTER the 4-way handshake completes (see below,
    // right after _wpa->ready()). Auth, assoc AND the 4-way all run at the 20 MHz primary:
    // retuning to 40/80 here — right before the AP fires M1 — disrupts RX during the
    // ~4 s EAPOL window so M1/M3 are missed and the handshake times out (state 5 -> 11).
    set(State::Handshaking);
    // ONE supplicant (member), used for handshake + RX decrypt + encrypted TX.
    Mac selfMac{}, bss{};
    std::copy(self.b.begin(), self.b.end(), selfMac.begin());
    std::copy(bssid.b.begin(), bssid.b.end(), bss.begin());
    // EAPOL M2/M4 are DATA frames that need a TX descriptor. Auth/assoc add theirs in
    // StationMode and DHCP adds its in dhcpSend, but BuildEapolKey returns the bare 802.11
    // frame and sendFrame adds nothing — so M2's first 40B (the 802.11 header) were misread
    // as the descriptor => garbled on air => the AP never advanced past PTKSTART. Wrap the
    // supplicant's send to prepend the data TX descriptor (mirrors dhcpSend).
    auto wpaSend = [&dev](const std::vector<uint8_t>& mpdu) -> bool {
        std::vector<uint8_t> frame(40 + mpdu.size(), 0);          // 40 = 8812 TX desc
        std::memcpy(frame.data() + 40, mpdu.data(), mpdu.size());
        // Use the MGMT TX descriptor (macid=1, raid=7) like auth/assoc — PROVEN to radiate
        // on this chip+USB path. CcmpData with macid=0/raid=0 drains the FIFO but never
        // goes on-air (verified via VTX hostapd: assoc seen, M2 never received).
        apfpv::FillStationTxDesc(frame.data(), (uint16_t)mpdu.size(), 40,
                                 /*macid*/1, apfpv::StationFrameKind::Mgmt, /*raid*/7, /*rate*/0x04);
        return dev.sendStationFrameSync(frame.data(), frame.size());
    };
    _wpa = std::make_unique<Wpa2Supplicant>(MakeWpa2Crypto(), wpaSend);
    _wpa->setPmf(_params.pmf);                     // 802.11w: M2 RSN caps must match the assoc-req
    ensurePmk();                                   // no-op if already cached
    _wpa->beginCached(_pmkCache, selfMac, bss);    // INSTANT (no PBKDF2) -> RX switches to
                                                   // the EAPOL phase in time to catch M1

    // Real (encrypted) DHCP send path — usable once keys are installed.
    auto dhcpSend = [&](const std::vector<uint8_t>& bootp) -> bool {
        if (!_wpa || !_wpa->ready()) return false;
        auto full = wrapDhcpUdpIp(bootp.data(), bootp.size());   // BOOTP -> full IPv4/UDP packet
        auto mpdu = _wpa->buildEncryptedData(full.data(), full.size());
        if (mpdu.empty()) return false;
        std::vector<uint8_t> frame(40 + mpdu.size(), 0);   // 40 = 8812 TX desc
        std::memcpy(frame.data()+40, mpdu.data(), mpdu.size());
        // macid=1, raid=7 like auth/assoc — the only TX path proven to radiate on
        // this chip. macid=0 is special/reserved on Jaguar and drains the FIFO silently.
        apfpv::FillStationTxDesc(frame.data(), (uint16_t)mpdu.size(), 40,
                                 1, apfpv::StationFrameKind::CcmpData, 7, 0x04);
        return sendFrame(frame);
    };
    _dhcp = std::make_unique<ApfpvDhcp>(self.b, dhcpSend);

    // Register the RX path NOW so EAPOL (handshake) and DHCP replies actually
    // reach the supplicant / DHCP machine before we wait on them.
    if (_params.lqFeedback) { LqFeedback::Config lqcfg{};
                              if (const char* e = std::getenv("DEVOURER_LQ_MS")) { int ms = std::atoi(e);
                                  if (ms > 0) { lqcfg.send_interval_ms = ms; lqcfg.min_interval_ms = ms; } }
                              _lq = std::make_unique<LqFeedback>(lqcfg);
                              // Route LQ over the dongle's IP stack — OPT-IN (DEVOURER_LQ_DONGLE). Default OFF:
                              // the periodic CCMP TX from the LQ thread collapses the dongle RX during streaming
                              // (bitrate -> ~0.3 Mbps on Win + Android; OUT serialises behind the IN pipeline),
                              // and greg's aalink (air_man) is disabled anyway so the feedback does nothing. When
                              // off, LqFeedback falls back to the (harmless, dead-ending) host socket. Enable only
                              // when air_man/adaptive-bitrate is running AND the TX/RX serialisation is solved.
                              bool lqDongle;
#if defined(__ANDROID__)
                              { char v[8]={0}; __system_property_get("persist.pixelpilot.lqdongle", v);
                                lqDongle = (v[0] != '0'); }   // default ON; `setprop persist.pixelpilot.lqdongle 0` disables
#else
                              lqDongle = (std::getenv("DEVOURER_LQ_DONGLE") != nullptr);
#endif
                              if (lqDongle)
                              _lq->setSink([this](const char* b, int n){
                                  uint32_t src = (_dhcp && _dhcp->lease().valid) ? _dhcp->lease().ip
                                                                                 : 0xC0A8000Au; // 192.168.0.10
                                  auto pkt = buildUdp(src, 0xC0A80001u /*192.168.0.1*/, 0xC351, 12345,
                                                      reinterpret_cast<const uint8_t*>(b), (size_t)n);
                                  bool ok = sendIpPacket(pkt.data(), pkt.size());
                                  static int lqDbg = 0;
                                  if ((lqDbg++ % 30) == 0)
                                      std::fprintf(stderr, "[lq-tx] -> 192.168.0.1:12345 payload='%.*s' srcIp=%08x ok=%d\n",
                                                   n, b, src, (int)ok);
                              });
                              _lq->start("192.168.0.1", 12345); }
    _rx = std::make_unique<RxDeframe>(selfMac, bss, _wpa.get(), _lq.get(), _onRtp);
    _rx->setStation(this);
    { ApfpvDhcp* dh = _dhcp.get();
      _rx->setDhcpSink([dh](const uint8_t* b, size_t n){ dh->onBootpReply(b, n); }); }
    if (_ipSink) _rx->setIpSink(_ipSink);   // general-IP downlink -> VpnService TUN (SSH etc.)
    // Switch the ALREADY-RUNNING RX thread to the streaming path — do NOT re-Init
    // (that blocks). EAPOL (handshake), DHCP replies, and RTP now route through
    // RxDeframe via the dispatch. The device is already tuned to the AP's channel.
    _rxPhase.store(1);
    {
        uint32_t r = dev.rtw_read32(0x0608);
        fprintf(stderr, "[hs] RCR=0x%08x ADF=%d ACF=%d  FLT0mgmt=0x%04x FLT1ctl=0x%04x FLT2data=0x%04x  MSR=0x%02x\n",
                r, (int)((r>>11)&1), (int)((r>>12)&1),
                dev.rtw_read16(0x06A0), dev.rtw_read16(0x06A2), dev.rtw_read16(0x06A4),
                dev.rtw_read8(0x0102));
    }

    // Wait (bounded) for the 4-way handshake to install keys.
    {
        using namespace std::chrono;
        auto t0 = steady_clock::now();
        while (!_wpa->ready()) {
            if (steady_clock::now() - t0 > seconds(5)) { set(State::FailAuth); return false; }
            std::this_thread::sleep_for(milliseconds(20));
        }
    }

    // Bandwidth already set correctly by the arm (sta.setConnectWidth matches the
    // AP's negotiated width). Running pauseAsyncRx/resumeAsyncRx here — even when
    // set_channel_bwmode is a no-op — disrupts RX exactly when data starts flowing,
    // causing the "black screen + inactivity timeout" pattern. The pause/resume was
    // historically needed for IQK control-I/O isolation; IQK is now disabled by
    // default, so there is no reason to touch the RX pipeline here.

    // Enable fire-and-forget TX NOW (before LQ/DHCP start). The per-TX TXPKT_EMPTY
    // drain in sendStationFrameSync blocks the USB bus for ~100ms each call, and LQ
    // fires ~1/s + DHCP retransmits — starving RX with 100ms gaps (rx-gap log storm).
    // send_packet already blocks on the bulk write; the drain is a connect diagnostic.
    dev.setTxFastPath(true);

    // ⭐ Lever C.2 (kernel-parity HW CCMP decrypt) — DEFAULT ON (2026-07-14). The chip decrypts RX
    // in hardware so the single RX worker just de-aggregates + forwards plaintext (the kernel's
    // lean tasklet), removing per-packet SW AES-CCM from the hot path. At VHT-2SS/80MHz (~50Mbps+,
    // 5000+ frames/s) SW-CCMP on one worker falls behind and the link oscillates; HW decrypt keeps
    // it clean and is what the firmware's full-station RA path expects. PTK (unicast video) → CAM
    // entry 4 keyed to the AP BSSID/keyid0; GTK (bcast) → entry 5. RxDeframe skips SW decrypt for
    // bdecrypted frames (same default). Fixed 2026-07-14: SECCFG 0x010c + _params.bssid populated
    // → bdecrypted=4000/4000 verified. DEVOURER_NO_HW_DECRYPT reverts to the SW path.
    if (std::getenv("DEVOURER_HW_DECRYPT") != nullptr || apfpvProp("debug.pixelpilot.hwdec")) {   // HW-decrypt arms the HW reorder + compressed-BA (rig: loss 163→21). Needs RCR_APP for correct framing (debug.pixelpilot.rcrapp). `debug.pixelpilot.hwdec=1` on Android.
        const auto& tk = _wpa->tk();
        dev.setSecCamKey(4, _params.bssid.data(), 0, tk.data());          // PTK pairwise, keyid 0
        if (_wpa->gtkKeyId() != 0xff) {
            const auto& gtk = _wpa->gtk();
            dev.setSecCamKey(5, _params.bssid.data(), _wpa->gtkKeyId(), gtk.data());  // GTK group
        }
        dev.enableHwSec();                                                // REG_SECCFG = 0x0c01
        SCANLOG("HW-DECRYPT: PTK->CAM4 GTK->CAM5(kid=%u) SECCFG=0c01 (RX worker now forwards plaintext)",
                (unsigned)_wpa->gtkKeyId());
    }

    // BlockAck is DISABLED by default. This dongle's MAC cannot emit the SIFS compressed
    // BlockAck (silicon, [[apfpv-hw-blockack-fix]]), so initiating BA only makes the AP
    // aggregate frames it can't ACK -> the AP retransmits -> PN-replay decFail -> ~3x
    // throughput loss on an A-MPDU-on AP (measured Taiga: 8 -> 22 median / 35 peak Mbps when
    // BA is declined instead). Opt back into the old BA-initiating + BA-register path with
    // DEVOURER_ENABLE_BA (e.g. to re-test the HW auto-BA hypothesis on a kernel-matched AP).
    if (enableBaOn()) {
        // REG_AMPDU_MIN_SPACE: the kernel does NOT write this (leaves chip default). We forced
        // 7 = 16µs min MPDU spacing, which over-restricts the AP's downlink A-MPDU packing (at
        // high MCS a 1.5KB frame is <16µs, so 16µs spacing pads/caps the aggregate → the ~30Mbps
        // ceiling). Set 0 = no extra spacing (chip handles dense A-MPDU; matches kernel intent).
        dev.rtw_write8(0x045C, 0);                     // was 7 (16µs) — let the chip pack tight
        dev.rtw_write8(0x1C, dev.rtw_read8(0x1C) | 0x60); // BIT5|BIT6: prevent MAC reset

        // ⭐ KERNEL-EXACT BA REGISTER SET (full usbmon init→connect diff vs the kernel @867Mbps
        // A-MPDU). The streaming capture proved the BlockAck is HW-AUTOMATIC (no BACAM write, no
        // BA H2C at runtime) — so it's armed purely by these init/response registers + the HW
        // seeing the ADDBA. We were writing several of them WRONG; matching the kernel byte-for-
        // byte is the only lever for the HW auto-BA. Each value is the kernel's final write.
        if (!std::getenv("DEVOURER_SKIP_BAREGS")) {
            // ★★ REG_RXFLTMAP1 (0x06A2) BIT8 = "accept BAR". The kernel sets this in its
            // join-complete handler (HW_VAR_ENABLE_RX_BAR). The AP polls us with a BlockAckRequest
            // after each A-MPDU burst; if BIT8 is clear the HW DROPS the BAR and never emits the
            // compressed BlockAck → AP DELBAs reason 37. Diagnose+force here in the streaming path
            // (proves whether our earlier writes survived to connect time). DEVOURER_NO_RXFLT skips.
            if (!std::getenv("DEVOURER_NO_RXFLT")) {
                uint16_t fm = dev.rtw_read16(0x06A2);
                dev.rtw_write16(0x06A2, (uint16_t)(fm | 0x0700)); // BIT8 BAR | BIT9 BA | BIT10 PS-Poll
                SCANLOG("RXFLTMAP1 0x06A2 before=%04x BAR_bit8=%d -> after=%04x (forced BAR+BA)",
                        fm, (fm>>8)&1, dev.rtw_read16(0x06A2));
            }
            // ITERATION (2026-07-16 bench: userspace 6.57% incomplete-frames vs kernel 0.08%). Match
            // the kernel's RUNTIME BA-register values devourer STILL differs on (from the 30s head-to-
            // head mac_reg diff). Gated DEVOURER_BAREGS_K for clean A/B on the bench rig.
            if (std::getenv("DEVOURER_BAREGS_K") || apfpvProp("debug.pixelpilot.baregsk")) {
                dev.rtw_write16(0x06A2, 0x0520);       // RXFLTMAP1: kernel=0x0520 (block above forces 0x0700)
                dev.rtw_write32(0x0458, 0x8001ffff);   // REG_AMPDU_MAX_LENGTH: kernel runtime (D was 0xffff0180)
                dev.rtw_write8(0x0463, 0x03);          // REG_RD_RESP_PKT_TH: kernel lowered 0x05->0x03 at connect
                SCANLOG("BAREGS_K: RXFLTMAP1=%04x AMPDU_MAXLEN=%08x RD_RESP_TH=%02x",
                        dev.rtw_read16(0x06A2), (unsigned)dev.rtw_read32(0x0458), dev.rtw_read8(0x0463));
            }
            // MATCH ALL kernel RUNTIME value-diffs (from the 30s mac_reg readback diff) that devourer
            // differs on — these are VALUE diffs on registers BOTH write, so the address-level usbmon
            // diff missed them. Hunting the compressed-BA arm. Gated DEVOURER_KVALS for A/B + bisect.
            if (std::getenv("DEVOURER_KVALS") || apfpvProp("debug.pixelpilot.kvals")) {
                dev.rtw_write32(0x0434, 0x07040302);   // (kernel runtime; D=0x07050302)
                dev.rtw_write32(0x045c, 0x1000f700);   // REG_WMAC_LBK / RD region (D=0x1000f900)
                dev.rtw_write32(0x0460, 0x03086666);   // (D=0x05086666)
                dev.rtw_write32(0x047c, 0x03030002);   // (D=0x0000000c/d)
                dev.rtw_write32(0x0480, 0xa40c0420);   // REG_INIRTS_RATE_SEL (D=0xa40c0400)
                dev.rtw_write32(0x04bc, 0x00040040);   // REG_AMPDU_BURST_MODE (D=0x0004005f)
                dev.rtw_write32(0x04f0, 0x0000c100);   // REG_TX_RPT_TIME (D=0x04014100)
                dev.rtw_write32(0x0524, 0x21ff4f0f);   // REG_RD_CTRL region (D=0x21ff4fff)
                dev.rtw_write32(0x0544, 0x00400180);   // REG_RD_NAV_NXT (D=0x00400200)
                SCANLOG("KVALS applied: 0x04bc=%08x 0x0480=%08x 0x0524=%08x 0x047c=%08x",
                        (unsigned)dev.rtw_read32(0x04bc), (unsigned)dev.rtw_read32(0x0480),
                        (unsigned)dev.rtw_read32(0x0524), (unsigned)dev.rtw_read32(0x047c));
            }
            // ★★ TEST 3 (DEVOURER_T3_WAKE): MACID sleep/wake state — a HW state machine we never
            // drove. The kernel calls rtw_hal_macid_wakeup(psta->mac_id) on join. If our AP-peer
            // macid's bit is SET in REG_MACID_SLEEP (0x04D4 / _1 0x0488 / _2 0x04D0 / _3 0x0484),
            // the HW thinks the peer is power-save asleep and PS-BUFFERS our TX to it — including
            // the SIFS compressed BlockAck → the BA never radiates → AP DELBA reason 37. Read the
            // state, then wake ALL macids (clear the sleep bitmaps). Default ON (it's correct for
            // a station); DEVOURER_NO_T3WAKE reverts.
            if (!std::getenv("DEVOURER_NO_T3WAKE")) {
                uint32_t s0 = dev.rtw_read32(0x04D4), s1 = dev.rtw_read32(0x0488);
                uint32_t s2 = dev.rtw_read32(0x04D0), s3 = dev.rtw_read32(0x0484);
                dev.rtw_write32(0x04D4, 0); dev.rtw_write32(0x0488, 0);
                dev.rtw_write32(0x04D0, 0); dev.rtw_write32(0x0484, 0);
                SCANLOG("MACID_SLEEP before: 0x4D4=%08x 0x488=%08x 0x4D0=%08x 0x484=%08x (macid0_asleep=%d) -> woke all",
                        s0, s1, s2, s3, (int)(s0 & 1));
            }
            // TEST 4 (DEVOURER_T4_JOIN): remaining join-state regs we never set, from the LIVE
            // kernel dump — REG_BCN_INTERVAL/retry region 0x0540=0x80006404, 0x0544=0x00400180,
            // and REG_RETRY_LIMIT (0x0541 byte within 0x0540) RL_VAL_STA=0x3030. The HW's beacon/
            // TSF window + TX retry shape the response/BA timing. DEVOURER_NO_T4 reverts.
            if (std::getenv("DEVOURER_T4_JOIN")) {
                dev.rtw_write32(0x0540, 0x80006404);   // BCN_INTERVAL region (live kernel)
                dev.rtw_write32(0x0544, 0x00400180);   // (live kernel)
                dev.rtw_write16(0x0541, 0x3030);       // REG_RETRY_LIMIT short|long = 0x30 (RL_VAL_STA)
                // TSF SYNC VERIFY: read REG_TSFTR (0x0560 low / 0x0564 high). If TSF is synced+running
                // it is a large value counting up ~1e6/sec; if frozen/0 the HW never synced to the AP.
                uint32_t tsfL = dev.rtw_read32(0x0560), tsfH = dev.rtw_read32(0x0564);
                uint8_t bcn = dev.rtw_read8(0x0550);   // REG_BCN_CTRL — bit4(0x10)=DIS_TSF_UDT
                SCANLOG("T4 join-state set; TSF=0x%08x%08x BCN_CTRL=0x%02x (DIS_TSF_UDT=%d)",
                        tsfH, tsfL, bcn, (bcn>>4)&1);
            }
            // A-MPDU max length: we blasted 0xffffffff; kernel uses a SPECIFIC factor. The all-ones
            // value mis-sizes the HW aggregate handling (BIT31 = RX_AMPDU enable is kept).
            dev.rtw_write32(0x0458, 0xffff0180);   // REG_AMPDU_MAX_LENGTH (kernel operational)
            // ★ 0x0420 REG_FWHW_TXQ_CTRL — GROUND TRUTH from the LIVE kernel mac_reg_dump while
            // streaming @867Mbps with RX-BA active = 0xff311f00. We were writing 0x001f71ff (a
            // usbmon misread). This reg governs the HW TX/RESPONSE queues (where the chip auto-
            // generates ACK/compressed-BA). If anything host-side gates the HW BA-response
            // generation, this is it. DEVOURER_TXQ_OLD reverts to the old (wrong) value.
            if (std::getenv("DEVOURER_TXQ_OLD")) dev.rtw_write32(0x0420, 0x001f71ff);
            else                                 dev.rtw_write32(0x0420, 0xff311f00);
            dev.rtw_write32(0x0424, 0x3e30f7f7);   // REG_HWSEQ_CTRL — LIVE kernel (was 0x1230f9f9)
            dev.rtw_write32(0x04f0, 0x0000c100);   // LIVE kernel (was 0x04014100)
            { uint32_t rb = dev.rtw_read32(0x0420);
              SCANLOG("LIVEREG 0x0420 wrote=ff311f00 readback=%08x (status bytes may differ)", rb); }
            // REG_RRSR (0x0440, Response Rate Set): the rates our HW sends ACK/CTS/BlockAck at.
            // We used 0x00000fff (kernel INIT value = all rates) → the HW may answer high-MCS data
            // with a too-aggressive BA the AP misses on the weaker uplink (our measured 4-7% retry
            // → slow per-A-MPDU cadence → ~30Mbps). Kernel OPERATIONAL (trace line 19277, streaming)
            // = 0x0440=0x5001 + 0x0442=0x20 (robust response rates). Gated by DEVOURER_NO_RRSR.
            if (!std::getenv("DEVOURER_NO_RRSR")) {
                // ★ GROUND TRUTH from the LIVE kernel mac_reg_dump (0x0440=0x00200150), NOT the
                // earlier usbmon misread. RRSR = the rate the HW emits ACK/CTS/compressed-BA at.
                // Our old 0x5001 = 1M(CCK)+MCS0+MCS2 — CCK is UN-TRANSMITTABLE at 5GHz, so our
                // compressed BA never radiated -> AP got no BA -> DELBA reason 37 -> no aggregation.
                // 0x0150 = OFDM 6M/12M/24M (proper 5GHz basic response rates). DEVOURER_RRSR_OLD reverts.
                if (std::getenv("DEVOURER_RRSR_OLD")) dev.rtw_write16(0x0440, 0x5001);
                else                                  dev.rtw_write16(0x0440, 0x0150);
                dev.rtw_write8 (0x0442, 0x20);     // RRSR byte2 (kernel live = 0x20)
            }
            // Response SIFS — LIVE kernel = 0x0e0a (we had 0x0e10 = 6µs too long; the compressed
            // BA then radiates LATE and the AP misses it). 0x0e0a matches the live mac_reg_dump.
            dev.rtw_write16(0x0514, 0x0e0a);       // REG_SIFS_CTX  (kernel LIVE 0x0e0a)
            dev.rtw_write16(0x0516, 0x0e0a);       // REG_SIFS_TRX  (kernel LIVE 0x0e0a)
            dev.rtw_write16(0x063A, 0x0e0a);       // REG_SIFS (Trx) — kernel LIVE 0x0a/0x0e
            dev.rtw_write8 (0x063C, 0x08);         // SIFS_R2T_CCK  (kernel 0x08)
            dev.rtw_write8 (0x063D, 0x08);         // SIFS_R2T_OFDM (kernel 0x08)
            dev.rtw_write8 (0x063E, 0x0E);         // SIFS_T2T_CCK  (kernel 0x0E)
            dev.rtw_write8 (0x063F, 0x0A);         // SIFS_T2T_OFDM (kernel 0x0A)
            dev.rtw_write8 (0x0429, 0x0E);         // SPEC_SIFS OFDM (kernel operational 0x0E)
            dev.rtw_write8 (0x051B, 0x09);         // REG_SLOT = 9µs (kernel writes; we didn't)
            // TCR (0x0607): the kernel's *operational* value (trace line 19733, during streaming)
            // is 0x07 — it KEEPS BIT2 set. Our prior "fix" cleared BIT2 to 0x03 based on the
            // early-init write (line 5193) and got the direction backwards: BIT2 governs the VHT
            // A-MPDU response, so clearing it broke the exact thing we needed. Restore 0x07.
            uint8_t tcr3 = dev.rtw_read8(0x0607);
            dev.rtw_write8 (0x0607, (uint8_t)(tcr3 | 0x07)); // kernel operational 0x07 (set BIT0|1|2)
            // EDCA BE: kernel operational = 0x2ba45e00 (line 20081). Our prior 0x2ba40000 came
            // from a transient init write (line 19287) and zeroed the TXOP-limit field, hurting
            // aggregation. Restore the operational value.
            // ★ EDCA_BE — LIVE kernel = 0x005ea42b. Our 0x2ba45e00 had AIFS=0x00 (transmit with
            // ZERO arbitration spacing) + TXOP=0x2ba4 — both wrong (byte-order mangled). The live
            // value: AIFS=0x2b, CWmin/max=0xa4, TXOP=0x005e. DEVOURER_EDCA_OLD reverts.
            if (std::getenv("DEVOURER_EDCA_OLD")) dev.rtw_write32(0x0508, 0x2ba45e00);
            else                                  dev.rtw_write32(0x0508, 0x005ea42b);
            SCANLOG("BA-regs LIVE-kernel: 0x0420=ff311f00 0x0424=3e30f7f7 0x04f0=0000c100 0x0508=005ea42b RRSR=0150/20 SIFS=0e0a 0607|=07");
        }
        // NOTE: a "KPROTO replay" experiment (set the 96 MAC registers the kernel writes that we
        // don't — 0x0668 base, SECCFG, BFMER0/CSI_RPT VHT-sounding, SND_PTCL) was tested 2026-06-21
        // and made things MUCH WORSE (retryBit 30%→71%, throughput 17→4Mbps). Blindly injecting the
        // kernel's registers breaks coherence: our overall chip state differs, so the kernel's values
        // are wrong for OUR state. Conclusion: the BA gap is a STATE/SEQUENCE divergence, not copyable
        // individual registers. Reverted. Next: diff OUR driver's full usbmon vs the kernel's.
    }

    // ENCRYPT round-trip self-test: encrypt a dummy IPv4/UDP datagram, then decrypt it through
    // the proven-standard decrypt path. roundtrip+match => our CCMP ENCRYPT is standard (so a
    // dropped uplink is a framing/DHCP issue, not crypto); fail => the encrypt itself is wrong.
    {
        uint8_t tip[28] = {0x45,0,0,28, 0,0,0,0, 64,17,0,0, 192,168,250,9, 255,255,255,255,
                           0,68,0,67,0,8,0,0};
        auto e = _wpa->buildEncryptedData(tip, sizeof(tip));
        std::vector<uint8_t> dpl;
        bool rt = !e.empty() && _wpa->decryptData(e.data(), e.size(), dpl);
        bool match = rt && dpl.size() >= 8 + sizeof(tip) && std::memcmp(dpl.data()+8, tip, sizeof(tip))==0;
        fprintf(stderr, "[enc-selftest] roundtrip=%d match=%d encLen=%zu decLen=%zu\n",
                (int)rt, (int)match, e.size(), dpl.size());
    }

    // Initiate ADDBA Request for TID 0 (video) before DHCP. The kernel's station
    // sends issue_addba_req to initiate the BA session; the AP responds and starts
    // A-MPDU. Sending the request AFTER keys are installed ensures it's encrypted
    // (QoS-data). The response handler in RxDeframe's dispatch catches the reply.
    //
    // ⚠️ GATED behind DEVOURER_ENABLE_BA (was unconditional — the throughput REGRESSION).
    // This dongle's MAC cannot emit the SIFS compressed BlockAck ([[apfpv-hw-blockack-fix]],
    // proven CLOSED). If we ASK the AP to start A-MPDU, the AP aggregates frames we can't ACK
    // -> AP retransmits -> the VTX rate-controller collapses to the MCS floor (rate=12/20MHz ~=
    // 2-3.9 Mbps). Declining/never-initiating BA keeps the stream non-aggregated -> the rate
    // controller ramps to high MCS -> ~33 Mbps. So by default we must NOT initiate BA here;
    // it belongs with the same env gate as the BA-register block above.
    if (enableBaOn()) {
        MacAddr bssid; for (int i=0;i<6;i++) bssid.b[i] = _params.bssid[i];
        std::vector<uint8_t> addba;
        // 802.11 QoS-Data header (subtype 8, ToDS, Protected)
        addba.push_back(0x88); addba.push_back(0x41);   // FC: QoS-Data, ToDS, Protected
        addba.push_back(0x00); addba.push_back(0x00);   // duration
        addba.insert(addba.end(), bssid.b.data(), bssid.b.data() + 6); // A1=AP BSSID
        addba.insert(addba.end(), selfMac.begin(), selfMac.end());   // A2=self
        addba.insert(addba.end(), bssid.b.data(), bssid.b.data() + 6); // A3=AP BSSID
        addba.push_back(0x00); addba.push_back(0x00);   // seq ctl
        addba.push_back(0x00); addba.push_back(0x00);   // QoS control (TID=0, normal ACK)
        // LLC/SNAP: AA AA 03 00 00 00 + EtherType 0x888E (EAPOL... no, action frames
        // use a different encapsulation. Actually ADDBA is a mgmt action frame, not data.
        // For mgmt action, use plain mgmt frame (FC=0x00D0), not QoS-Data.
        // Build as mgmt action frame instead:
        addba.clear();
        addba.push_back(0xD0); addba.push_back(0x00);   // FC: mgmt, action
        addba.push_back(0x00); addba.push_back(0x00);   // duration
        addba.insert(addba.end(), bssid.b.data(), bssid.b.data() + 6); // A1=AP
        addba.insert(addba.end(), selfMac.begin(), selfMac.end());   // A2=self
        addba.insert(addba.end(), bssid.b.data(), bssid.b.data() + 6); // A3=AP
        static uint16_t mgmtSeq = 200;  // offset from response seq
        addba.push_back((u8)(mgmtSeq & 0xff));
        addba.push_back((u8)((mgmtSeq >> 4) & 0xff));
        mgmtSeq++;
        // ADDBA Request body (matches kernel issue_addba_req)
        addba.push_back(0x03); // Category: Block Ack
        addba.push_back(0x00); // Action: ADDBA Request
        addba.push_back(0x01); // Dialog token
        // BA Parameter Set: TID=0, buf=64, immediate BA
        addba.push_back(0x00); // low byte: TID=0, amsdu=0, buf[5:7]=0
        addba.push_back(0x08); // high byte: buf[3:0]=0b0000, policy=0(immediate)
        addba.push_back(0x00); addba.push_back(0x00); // BA Timeout: 0 (default)
        addba.push_back(0x00); addba.push_back(0x00); // BA Starting Seq: 0 (any)
        // Send with TX descriptor (same as ADDBA Response)
        std::vector<uint8_t> txf(40 + addba.size(), 0);
        std::memcpy(txf.data() + 40, addba.data(), addba.size());
        apfpv::FillStationTxDesc(txf.data(), (uint16_t)addba.size(), 40,
                                 1, apfpv::StationFrameKind::Mgmt, 0x0c, 0x04);
        dev.sendStationFrameSync(txf.data(), txf.size());
        SCANLOG("ADDBA Request sent for TID 0 (initiating BA session)");
    }

    // Keys installed -> DHCP (or static). For dynamic, wait briefly for the lease.
    set(State::Dhcp);
    if (_params.staticIp) _dhcp->claimStatic(_params.staticIp, _params.staticNetmask, _params.staticGateway);
    else {
        using namespace std::chrono;
        _dhcp->start();                                            // initial DISCOVER
        auto t0 = steady_clock::now(); auto lastTx = steady_clock::now();
        while (!_dhcp->lease().valid) {
            if (steady_clock::now() - t0 > seconds(8)) break;
            // Retransmit the CURRENT pending message (DISCOVER, then REQUEST) every ~700ms:
            // a single reply (OFFER or ACK) is easily missed in the RX re-arm gap after our
            // sync TX, and we must retry the REQUEST (not restart DORA) to land the ACK.
            if (steady_clock::now() - lastTx > milliseconds(700)) {
                _dhcp->retransmit(); lastTx = steady_clock::now();
            }
            std::this_thread::sleep_for(milliseconds(20));
        }
        { uint32_t ip=_dhcp->lease().ip, sv=_dhcp->lease().server;
          fprintf(stderr, "[dhcp] DORA done: valid=%d ip=%u.%u.%u.%u\n", (int)_dhcp->lease().valid,
                  (ip>>24)&255,(ip>>16)&255,(ip>>8)&255,ip&255);
          // ALSO to logcat (stderr is invisible there): this is the IP to stream RTP to.
          SCANLOG("DHCP lease valid=%d ip=%u.%u.%u.%u server=%u.%u.%u.%u  <-- stream RTP here",
                  (int)_dhcp->lease().valid, (ip>>24)&255,(ip>>16)&255,(ip>>8)&255,ip&255,
                  (sv>>24)&255,(sv>>16)&255,(sv>>8)&255,sv&255); }
        if (!_dhcp->lease().valid) _dhcp->claimStatic_192_168_0_10();  // fallback
    }

    // Gratuitous ARP: announce our leased IP<->MAC (broadcast) so peers can resolve our MAC
    // WITHOUT sending an ARP request (we don't answer those yet). Without this, the VTX that
    // unicasts RTP to our IP — and anything doing SSH/TCP to us — can't address us at L2.
    if (_dhcp->lease().valid) {
        uint32_t ip = _dhcp->lease().ip;
        // Build the gratuitous ARP once; the supervisor re-announces it every ~2s so the AP's
        // ARP entry for us never goes STALE (else unicast RTP stalls). In the real setup the LQ
        // feedback to the VTX also keeps it fresh, but this works on any subnet.
        _gratArp = [this, &dev, selfMac, ip]() {
            if (!_wpa || !_wpa->ready()) return;
            uint8_t arp[28] = {0,1, 8,0, 6,4, 0,1};
            std::memcpy(arp+8, selfMac.data(), 6);
            arp[14]=(ip>>24)&255; arp[15]=(ip>>16)&255; arp[16]=(ip>>8)&255; arp[17]=ip&255;
            arp[24]=(ip>>24)&255; arp[25]=(ip>>16)&255; arp[26]=(ip>>8)&255; arp[27]=ip&255;
            auto m = _wpa->buildEncryptedData(arp, 28, 0x0806);
            if (m.empty()) return;
            std::vector<uint8_t> fr(40 + m.size(), 0);
            std::memcpy(fr.data()+40, m.data(), m.size());
            apfpv::FillStationTxDesc(fr.data(), (uint16_t)m.size(), 40,
                                     1, apfpv::StationFrameKind::CcmpData, 7, 0x04);
            dev.sendStationFrameSync(fr.data(), fr.size());
        };
        for (int k=0;k<3;++k) _gratArp();                 // announce now
        // Answer subsequent ARP requests for our IP (keeps the unicast video stream alive).
        _rx->setArp(ip, [&dev](const std::vector<uint8_t>& mpdu){
            std::vector<uint8_t> fr(40 + mpdu.size(), 0);
            std::memcpy(fr.data()+40, mpdu.data(), mpdu.size());
            apfpv::FillStationTxDesc(fr.data(), (uint16_t)mpdu.size(), 40,
                                     1, apfpv::StationFrameKind::CcmpData, 7, 0x04);
            dev.sendStationFrameSync(fr.data(), fr.size());
        });
    }

    // mark link alive and go streaming.
    // NOTE: A-MPDU RX Block-Ack is NOT yet implemented. Without it single-frame
    // TXOPs cap at ~25Mbps regardless of MCS. 65+ Mbps requires ADDBA handshake
    // + HW Block-Ack per-TID (H2C_BA_CTRL). USB agg now matches kernel values.
    _deauth.store(false);
    _lastRxMs.store(nowMs());
    set(State::Streaming);
    // Streaming: make station TX fire-and-forget. The per-TX REG_TXPKT_EMPTY drain poll
    // in sendStationFrameSync is a connect-time diagnostic (~6ms/call); during streaming
    // the LQ-feedback loop hits it ~90-100x/s (~560ms/s of blocking) -> RX starves ->
    // decode/render stutter on the phone. send_packet already blocked on the bulk write.
    dev.setTxFastPath(true);
    // ADDBA Requests are answered directly in handleAddbaRequest (raw-fd TX),
    // which also covers requests arriving during streaming — no connect-time
    // batch/pause needed.

    // NOTE: tried clearing RCR FORCEACK (BIT26) here post-handshake to mirror the
    // Windows native driver (usbmon RCR 0x3f6800f4, FORCEACK=0). It did NOT durably
    // help — DELBA churn returned (~17/20s) and throughput stayed at the single-
    // frame ceiling. Windows runs FORCEACK=0 only because FWOffload has the firmware
    // own ACK/BA; without that mode FORCEACK=0 just loses the normal ACK. Reverted.
    // Opt back in for experiments via DEVOURER_STREAM_NOACK.
    if (std::getenv("DEVOURER_STREAM_NOACK")) {
        auto& dev = *reinterpret_cast<RtlUsbAdapter*>(_dev);
        uint32_t rcr = dev.rtw_read32(0x0608);
        rcr &= ~0x04000000u;   // clear FORCEACK (BIT26)
        dev.rtw_write32(0x0608, rcr);
        SCANLOG("STREAM_NOACK: RCR FORCEACK cleared -> 0x%08x", rcr);
    }
    return true;
}

void ApfpvStation::handleAddbaRequest(const uint8_t* frame, size_t len) {
    if (len < 24 + 9) return;
    const uint8_t* body = frame + 24;
    if (!(body[0] == 0x03 && body[1] == 0x00)) return;
    u8 tid = ((body[3] | (body[4] << 8)) >> 2) & 0x0f;
    uint16_t startSsn = (uint16_t)((body[7] | (body[8] << 8)) >> 4);   // ADDBA-Req Starting Seq

    // ⭐ BA-CAM ARM (experimental, opt-in via DEVOURER_BACAM): the AP requested an RX-BA
    // session for this TID. The HW MAC emits the SIFS compressed BlockAck ONLY when its
    // BA-CAM (REG_BACAMCMD 0x0654 / REG_BACAMCONTENT 0x0658) holds a valid per-TID/per-peer
    // entry. On the kernel the FIRMWARE auto-arms it from the ADDBA seen on the station RX
    // path; our libusb RX path never triggers that auto-arm, so BACAMCMD reads 0 and the AP
    // retransmits every unacked A-MPDU (the ~50% PN-replay / decFail at >20Mbps). Program it
    // ourselves, modeled on rtw89_fw_h2c_ba_cam's w0 layout:
    //   valid:0 init_req:1 entry_idx:3-2 tid:7-4 macid:15-8 bmap_size:19-16 ssn:31-20
    // RESULT (2026-06-21): the writes take (CMD BIT31 polls clear) but do NOT arm the SIFS
    // BlockAck — decFail stayed ~89% at 30Mbps and the CONTENT readback ≠ what we wrote, so
    // this rtw89-derived w0 layout is WRONG for the 8812's BA-CAM (the vendor driver never
    // writes it, so there's no reference format). Gated OFF; the 8812 BA-CAM arming is
    // firmware-internal (FWOffload), not reproducible by a register poke. Opt-in to keep
    // experimenting on the format via DEVOURER_BACAM.
    if (std::getenv("DEVOURER_BACAM")) {
        auto& d = *reinterpret_cast<RtlUsbAdapter*>(_dev);
        uint8_t  macid = 1;                  // our AP peer macid (TX-desc / H2C use 1)
        uint8_t  entry = tid & 0x07;         // one entry per TID for the single peer
        uint8_t  bmap  = 0;                  // 0 = 64-frame bitmap (default BA window)
        uint32_t content = (1u)              // valid
                         | (1u << 1)         // init_req
                         | ((entry & 0x3u) << 2)
                         | ((uint32_t)(tid   & 0xf)  << 4)
                         | ((uint32_t)(macid & 0xff) << 8)
                         | ((uint32_t)(bmap  & 0xf)  << 16)
                         | ((uint32_t)(startSsn & 0xfff) << 20);
        d.rtw_write32(0x0658, content);                          // REG_BACAMCONTENT
        d.rtw_write32(0x0654, 0x80000000u | 0x40000000u | entry); // poll | write | addr
        for (int p = 0; p < 20; ++p) {                          // poll until HW clears BIT31
            if ((d.rtw_read32(0x0654) & 0x80000000u) == 0) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        uint32_t cmdRb = d.rtw_read32(0x0654), contRb = d.rtw_read32(0x0658);
        SCANLOG("BACAM arm tid=%u entry=%u ssn=%u content=0x%08x -> CMD=0x%08x CONTENT=0x%08x",
                tid, entry, startSsn, content, cmdRb, contRb);
    }
    // Respond to EVERY ADDBA Request. A per-TID debounce here was actively
    // harmful: after the connect-time burst set all 8 TID bits, the AP's later
    // re-requests (session timeout / re-establish during streaming) were
    // silently dropped -> the AP got no Response -> DELBA -> re-request loop.
    // The AP only re-requests when it needs to; answering each one keeps the
    // BA session alive. (raw-fd TX makes per-request sends cheap.)

    auto& dev = *reinterpret_cast<RtlUsbAdapter*>(_dev);
    MacAddr ba; for (int i=0;i<6;i++) ba.b[i] = _params.bssid[i];
    std::vector<uint8_t> r;
    r.push_back(0xD0); r.push_back(0x00); r.push_back(0x00); r.push_back(0x00);
    r.insert(r.end(), ba.b.data(), ba.b.data()+6);
    for (int i=0;i<6;i++) r.push_back(dev.rtw_read8(0x0610+i));
    r.insert(r.end(), ba.b.data(), ba.b.data()+6);
    static uint16_t s=0; s++; r.push_back(s&0xff); r.push_back((s>>4)&0xff);
    // ADDBA Response body (802.11 §9.6.5.3): Category + Action + DialogToken +
    // StatusCode(2) + BAParameterSet(2) + BATimeout(2) = 9 bytes EXACTLY. Do NOT
    // append the request's Starting Sequence Control — that is Request-only; the
    // extra 2 bytes made the frame malformed and some APs silently reject it (so
    // they never aggregate). Echo the AP's BA param set + timeout verbatim.
    u8 dialog = body[2];
    // DECLINE the AP's ADDBA by default: this dongle can never emit the SIFS compressed
    // BlockAck (MAC-silicon, not host-reachable), so on an A-MPDU-on AP every aggregate it
    // can't ACK is retransmitted -> arrives as PN-replay -> ~50% decFail. Replying StatusCode
    // 37 (REFUSED) forces the AP to stop aggregating this TID, trading aggregation gain we
    // can't use for a clean non-aggregated stream (no PN-replay). DEVOURER_ENABLE_BA accepts
    // (StatusCode 0) to restore the old behavior.
    const bool declineBa = !enableBaOn();
    r.push_back(0x03); r.push_back(0x01); r.push_back(dialog);
    if (declineBa) { r.push_back(37); r.push_back(0x00); }      // StatusCode = 37 (REFUSED)
    else           { r.push_back(0x00); r.push_back(0x00); }    // StatusCode = 0 (success)
    // BA Parameter Set: bit0=AMSDU-supported, bit1=policy, bits2-5=TID, bits6-15=buffer size
    // (10 bits). We normally echo the AP's requested value verbatim (see the note above about
    // NOT appending extra bytes — that's still true and unrelated to this). But since we can't
    // emit a real compressed BlockAck, a lost sub-frame inside a 64-deep aggregate has no correct
    // way to be identified/retransmitted, and it shows up as a chunk of missing bitstream data
    // (visually: motion-compensated "tails" on moving objects that persist until a later frame
    // overwrites them). The responder is allowed by spec to negotiate a SMALLER buffer size than
    // requested; a smaller window means the AP bursts fewer MPDUs per aggregate, so any one loss
    // event corrupts a smaller slice of a frame instead of up to 64 frames' worth. Opt-in via
    // DEVOURER_BA_BUFSZ (host) / debug.pixelpilot.babufsz (Android) so it's A/B-testable without
    // a rebuild per value; 0 or unset = echo the AP's request unchanged (today's behavior).
    uint16_t reqParam   = (uint16_t)(body[3] | (body[4] << 8));
    uint16_t reqBufsz   = (reqParam >> 6) & 0x3ff;
    int      bufszOverride = baRespBufSizeOverride();
    uint16_t sentBufsz  = reqBufsz;
    if (bufszOverride > 0) {
        uint16_t newParam = (uint16_t)((reqParam & 0x3f) | ((uint16_t)bufszOverride << 6));
        r.push_back((uint8_t)(newParam & 0xff)); r.push_back((uint8_t)(newParam >> 8));
        sentBufsz = (uint16_t)bufszOverride;
    } else {
        r.push_back(body[3]); r.push_back(body[4]);             // BA Parameter Set (echo)
    }
    r.push_back(body[5]); r.push_back(body[6]);                 // BA Timeout (echo)
    SCANLOG("ADDBA Response BUILD dialog=%u tid=%u status=%u reqBufsz=%u sentBufsz=%u (override=%d)",
            dialog, tid, declineBa ? 37 : 0, reqBufsz, sentBufsz, bufszOverride);

    // Send the ADDBA Response DIRECTLY via send_packet. With the raw-fd
    // USBDEVFS_BULK TX path this is a synchronous kernel ioctl that the USB
    // stack interleaves with the in-flight IN URBs — it does NOT block the
    // libusb event loop or touch its async reap queue, so it is safe to call
    // from the RX dispatch here. This answers the AP within ~ms (no pause hack)
    // and also handles ADDBA Requests that arrive DURING streaming, not just
    // the connect-time burst.
    std::vector<uint8_t> txf(40 + r.size(), 0);
    memcpy(txf.data()+40, r.data(), r.size());
    FillStationTxDesc(txf.data(), (uint16_t)r.size(), 40,
                      1, StationFrameKind::Mgmt, 0x0c, 0x04);
    // RELIABLE ADDBA-Response delivery — the crux of the 28-vs-59 gap. The kernel uses
    // issue_addba_rsp_wait_ack() (3 retries, wait for the AP's ACK). We sent ONCE,
    // fire-and-forget; when that single copy is lost over-air the AP never learns we
    // accepted -> DELBA reason 37 ("peer doesn't want BA") -> re-ADDBA loop -> the BA
    // session never stays up -> intermittent A-MPDU -> ~half throughput. We can't easily
    // detect the 802.11 ACK on the libusb path, so blind-retry: for the VIDEO TID (0) send
    // 3x via the synchronous (TX-drain-confirmed) path so it's rock-solid; idle TIDs keep
    // the cheap single async send (their teardown is harmless). DEVOURER_ADDBA_ASYNC reverts.
    // Measured: making tid=0 reliable drove DELBA 56->0 (BA stable) but did NOT raise
    // throughput (still ~25; the cap is the AP's per-STA delivery rate, not BA stability),
    // and the 3x sync starves RX ~30ms/burst. So keep the cheap async send by DEFAULT;
    // DEVOURER_ADDBA_RELIABLE opts into the kernel-style reliable (3x sync) tid=0 path.
    if (tid == 0 && std::getenv("DEVOURER_ADDBA_RELIABLE")) {
        for (int rep = 0; rep < 3; ++rep) dev.sendStationFrameSync(txf.data(), txf.size());
        SCANLOG("ADDBA Response tid=0 (3x reliable sync)");
    } else {
        dev.send_packet(txf.data(), txf.size());
        SCANLOG("ADDBA Response tid=%u (async)", tid);
    }
}

bool ApfpvStation::connect(const Params& p) {
    _params = p;
    bool ok = runConnectChain();
    // Start the persistent reconnect supervisor (the daemon), even if the first
    // attempt failed — it will keep trying, exactly like wpa_supplicant -B.
    if (p.autoReconnect && !_run.exchange(true)) {
        _supervisor = std::thread(&ApfpvStation::supervisorLoop, this);
    }
    return ok;
}

// The supervisor: the driver-level equivalent of the wpa_supplicant daemon.
// Watches for link loss and re-establishes, with exponential backoff capped low
// (recovery is fast: same channel, no rescan).
void ApfpvStation::supervisorLoop() {
    using namespace std::chrono;
    int backoff = _params.reconnectBackoffMs;
    int gratTick = 0;
    while (_run.load()) {
        std::this_thread::sleep_for(milliseconds(100));
        State s = _state.load();

        if (s == State::Streaming) {
            backoff = _params.reconnectBackoffMs;   // reset backoff on healthy link
            // Keepalive cadence: ~1s normally. DEVOURER_KEEPALIVE=1 fires every loop (~100ms) —
            // a power-save probe: if the AP buffers our downlink because it thinks the injected
            // station is asleep, a frequent uplink data frame (PWR_MGT=0) keeps us "awake" and the
            // AP delivers continuously instead of dribbling. If throughput jumps, PS-buffering was it.
            // TEST 1 (DEVOURER_T1_QUIET): make the supervisor fully silent during streaming —
            // no uplink ARP TX, no H2C, no diagnostic control-reads — so NOTHING of ours contends
            // with the RX/BA path on the USB bus. Tests the "flushing frames out of the HW" thesis.
            // `debug.pixelpilot.quiet=1` (adb setprop) also enables it on Android, where env vars
            // aren't settable — for testing whether our periodic supervisor TX (ARP + H2C RA/RSSI)
            // is stalling the single RX worker and dropping bursts of video (the dongle-only freeze).
            const bool t1quiet = std::getenv("DEVOURER_T1_QUIET") != nullptr || apfpvProp("debug.pixelpilot.quiet");
            // ARP re-announce cadence. Was 10 (1s) — but the gratuitous ARP is a SYNCHRONOUS
            // bulk-OUT that briefly starves the single RX worker, and at 1Hz that periodic stall
            // dropped a burst of video every ~1s (the "stutter every few seconds" on the dongle;
            // phone-Wi-Fi has no such TX so it was smoother). 30 (3s) keeps the AP's ARP entry for
            // us fresh (ARP timeout is tens of seconds) while cutting the periodic RX stall 3x.
            static const int kaEvery = std::getenv("DEVOURER_KEEPALIVE") ? 1 : 30;
            if (++gratTick >= kaEvery) { gratTick = 0; if (_gratArp && !t1quiet) _gratArp(); }  // re-announce ARP (keep AP entry fresh)
            // PERIODIC RA/RSSI H2C (~1Hz, matching the kernel usbmon cadence). The kernel re-sends
            // H2C 0x40 (RA/MACID) + 0x42 (RSSI) every ~1s during streaming to keep the firmware's
            // per-peer rate state fresh; we previously sent them ONCE at connect. Stale FW RA state
            // may be why the AP rate-controls us conservatively (MCS5/30Mbps) vs the kernel (72Mbps).
            // Gated by DEVOURER_NO_RA_TICK. Reuses the ~1s gratTick boundary.
            // RX-FIFO occupancy probe: REG_RXPKT_NUM (0x0284) = # packets in the chip RXPKTBUF,
            // REG_RXDMA_STATUS (0x0288). If RXPKT_NUM is consistently HIGH during streaming, the
            // chip can't drain to USB fast enough → FIFO backs up → overflow drops → AP retransmit
            // (the 6-7% retry). If ~0, the FIFO drains fine and the AP is simply sending ~30Mbps.
            if (!t1quiet) {
                auto& dd = *reinterpret_cast<RtlUsbAdapter*>(_dev);
                // One-shot full BB-register dump (0x800-0xffc) to diff against the kernel's
                // bb_reg_dump and find the 2-stream MIMO RX config difference (we get 1SS, kernel 2SS).
                static bool bbDumped = false;
                if (!bbDumped && std::getenv("DEVOURER_BB_DUMP")) {
                    bbDumped = true;
                    try {
                        // MAC BA/protocol region (0x400-0x6fc) + BB (0x800-0xffc) for the
                        // kernel diff — hunting the MAC reg that enables RX-BA-CAM auto-fill.
                        for (uint32_t a = 0x400; a <= 0xffc; a += 0x10) {
                            if (a >= 0x700 && a < 0x800) continue;   // skip FIFO/sensitive gap
                            uint32_t v0 = dd.rtw_read32(a), v1 = dd.rtw_read32(a+4);
                            uint32_t v2 = dd.rtw_read32(a+8), v3 = dd.rtw_read32(a+0xc);
                            SCANLOG("BBDUMP 0x%04x 0x%08x 0x%08x 0x%08x 0x%08x", a, v0, v1, v2, v3);
                        }
                    } catch (...) {}
                }
                // RF-path register dump (0x00-0x7f, path A+B) to diff vs the kernel's rf_reg_dump.
                // Same dongle → the kernel reaches VHT-2SS, so any RX-quality gap is SOFTWARE: the
                // RF-path calibration (LNA/gain/channel/IQK-LO) lives here, NOT in the BB page.
                static bool rfDumped = false;
                if (!rfDumped && std::getenv("DEVOURER_RF_DUMP")) {
                    rfDumped = true;
                    try {
                        auto& rm = *reinterpret_cast<RadioManagementModule*>(_rm);
                        for (int p = 0; p < 2; ++p) {
                            RfPath pth = p == 0 ? RfPath::RF_PATH_A : RfPath::RF_PATH_B;
                            for (uint32_t a = 0x00; a <= 0x7f; ++a) {
                                uint32_t v = rm.phy_query_rf_reg(pth, a, 0xfffff);
                                SCANLOG("RFDUMP P%d 0x%02x 0x%05x", p, a, v);
                            }
                        }
                    } catch (...) {}
                }
                // ⚠️ THESE DIAGNOSTIC REGISTER READS ARE DEBUG-ONLY AND DEFAULT OFF. They ran EVERY
                // 100ms loop = ~8 USB control reads x10/s = ~80 control transfers/s, all contending
                // with the SINGLE libusb RX worker → continuous RX starvation (video quality
                // degradation) on the dongle (phone-Wi-Fi, which has no such traffic, was smoother).
                // Enable with DEVOURER_RXDIAG when debugging the RX/rate path.
                if (std::getenv("DEVOURER_RXDIAG")) try {
                    uint16_t rxpktnum = dd.rtw_read16(0x0284);
                    // IGI (RX gain index) — PHYDM/DIG output. Path A = 0xc50[6:0], Path B = 0xe50[6:0].
                    // Kernel adapts this to ~0x37 (55) at rssi -45. If OURS is parked far off (DIG not
                    // adapting), our RX is mis-tuned → 6-7% loss → AP caps us. Direct BB read (memory-mapped).
                    uint32_t igiA = dd.rtw_read32(0x0c50) & 0x7f;
                    uint32_t igiB = dd.rtw_read32(0x0e50) & 0x7f;
                    // TX-power index for the rates our compressed BlockAck radiates at. The BA goes
                    // out at a legacy (CCK/OFDM) basic rate per RRSR, so its airborne power comes from
                    // the CCK (0xc20) / OFDM-low (0xc24) / OFDM-high (0xc28) TX-AGC regs (path A) and
                    // 0xe20/0xe28 (path B). Each reg packs 4 per-rate bytes (index 0x00-0x3f, hi=more
                    // power). If these are LOW, our BAs reach the AP weakly → AP misses them → its BA
                    // window stalls → link sits ~95% idle (our 27 vs kernel 72 Mbps cap). Byte0 shown.
                    uint8_t pCckA  = dd.rtw_read8(0x0c20);   // path A CCK1 power idx
                    uint8_t pOfdmA = dd.rtw_read8(0x0c24);   // path A OFDM6 power idx
                    uint8_t pO54A  = dd.rtw_read8(0x0c28);   // path A OFDM24-54 power idx
                    uint8_t pCckB  = dd.rtw_read8(0x0e20);   // path B CCK1
                    uint8_t pOfdmB = dd.rtw_read8(0x0e24);   // path B OFDM6
                    static int probeN = 0, rxpktMax = 0; static long rxpktSum = 0;
                    probeN++; rxpktSum += rxpktnum; if (rxpktnum > rxpktMax) rxpktMax = rxpktnum;
                    if (probeN >= 10) {   // ~1s
                        SCANLOG("rx-fifo: RXPKT_NUM avg=%ld max=%d | IGI_A=0x%02x IGI_B=0x%02x (kernel~0x37) "
                                "| TXPWR A[cck=0x%02x ofdm=0x%02x o54=0x%02x] B[cck=0x%02x ofdm=0x%02x]",
                                rxpktSum/probeN, rxpktMax, (unsigned)igiA, (unsigned)igiB,
                                pCckA, pOfdmA, pO54A, pCckB, pOfdmB);
                        probeN = 0; rxpktMax = 0; rxpktSum = 0;
                    }
                } catch (...) {}
            }
            if (gratTick == 0 && !t1quiet && !std::getenv("DEVOURER_NO_RA_TICK")) {
                auto& d = *reinterpret_cast<RtlUsbAdapter*>(_dev);
                // Kernel-verbatim MACID_CFG + RSSI (usbmon: [00,89,1a,00,80,ff,ff] / [00,00,2e]).
                // DEVOURER_RA_OLD restores the prior (wrong) values for A/B. See StationMode.cpp.
                if (std::getenv("DEVOURER_RA_OLD")) {
                    uint32_t ra_mask = std::getenv("DEVOURER_RA_CONSERVATIVE") ? 0x3FFFFFFFu : 0xfffff010u;
                    uint8_t ra[7] = { 0x00, (uint8_t)((9 & 0x1f) | (1 << 5) | (1 << 7)), (uint8_t)(2 | (1 << 4)),
                        (uint8_t)(ra_mask & 0xff), (uint8_t)((ra_mask >> 8) & 0xff),
                        (uint8_t)((ra_mask >> 16) & 0xff), (uint8_t)((ra_mask >> 24) & 0xff) };
                    uint8_t rssi[4] = { 0x00, 0x00, 0x39, 0x06 };
                    try { d.fillH2CCmd(0x40, 7, ra); d.fillH2CCmd(0x42, 4, rssi); } catch (...) {}
                } else {
                    // ⭐ C2H feedback loop / kernel-cadence H2C for the firmware-RA uplink.
                    // MACID_CFG (0x40) sets up the RA — send ONCE per connection like the kernel.
                    // Re-sending it every tick re-inits the RA so it never converges → the uplink
                    // rate oscillates (the user-observed 0%↔100%). RSSI (0x42) is the real per-tick
                    // feedback the firmware RA uses for its rate ceiling — send the ACTUAL tracked
                    // RSSI (byte2 = rssi_dBm+100, clamped 1..100), not the old hardcoded 0x2e.
                    uint8_t ra[7] = { 0x00, 0x89, 0x1a, 0x00, 0x80, 0xff, 0xff };   // kernel-verbatim
                    if (!_raCfgSent.exchange(true)) { try { d.fillH2CCmd(0x40, 7, ra); } catch (...) {} }
                    int rdbm = PhydmWatchdog::RssiDbm();
                    int rpct = rdbm + 100; if (rpct < 1) rpct = 1; if (rpct > 100) rpct = 100;
                    uint8_t rssi[3] = { 0x00, 0x00, (uint8_t)rpct };
                    try { d.fillH2CCmd(0x42, 3, rssi); } catch (...) {}
                    static uint32_t raLogN = 0;
                    if ((++raLogN % 4) == 1)
                        SCANLOG("RA-loop: rssi=%ddBm(pct=%d) uplink_rate_idx=0x%02x",
                                rdbm, rpct, (unsigned)PhydmWatchdog::UplinkRate());
                }
            }
            bool lost = _deauth.load();
            // RX-silence watchdog: no data/beacon for rxTimeoutMs => link gone.
            if (!lost && (nowMs() - _lastRxMs.load()) > _params.rxTimeoutMs) lost = true;
            if (lost) {
                set(State::LinkLost);
                _deauth.store(false);
            }
            continue;
        }

        if (s == State::LinkLost || s == State::FailNoAp || s == State::FailNoAck ||
            s == State::FailAuth || s == State::FailTx || s == State::FailDhcp) {
            if (!_run.load()) break;
            set(State::Reconnecting);
            std::this_thread::sleep_for(milliseconds(backoff));
            if (!_run.load()) break;
            // A USB register read/write can throw (rtw_read -> ios_base::failure)
            // if the dongle hiccups or is accessed concurrently. Catch it here so
            // the supervisor THREAD degrades to a failed attempt instead of
            // terminating the whole app (std::terminate -> SIGABRT).
            bool ok = false;
            try { ok = runConnectChain(); }          // re-arm on known channel
            catch (const std::exception&) { ok = false; set(State::FailNoAp); }
            catch (...)                    { ok = false; set(State::FailNoAp); }
            if (!ok) {
                backoff = std::min(backoff * 2, _params.maxBackoffMs);  // exp backoff
            } else {
                backoff = _params.reconnectBackoffMs;
            }
        }
    }
}

void ApfpvStation::disconnect() {
    _run.store(false);
    PhydmWatchdog::SetUnlinked();   // restore monitor-mode DIG bounds (wfb-ng coexistence)
    // Join-safe: NEVER join the current thread (-> EINVAL "Invalid argument" -> uncaught
    // std::system_error -> SIGABRT, seen on replug when the JNI re-init teardown disconnect
    // races/repeats the Java stopAdapters disconnect), and tolerate an already-reaped thread.
    auto safeJoin = [](std::thread& t) {
        if (!t.joinable()) return;
        if (t.get_id() == std::this_thread::get_id()) { t.detach(); return; }
        try { t.join(); } catch (...) { if (t.joinable()) { try { t.detach(); } catch (...) {} } }
    };
    safeJoin(_supervisor);
    // Stop the RX: async URB pool (default) + legacy sync read thread.
    if (_rtl) {
        reinterpret_cast<RtlJaguarDevice*>(_rtl)->should_stop = true;
        reinterpret_cast<RtlJaguarDevice*>(_rtl)->StopAsyncRx();
    }
    safeJoin(_rxThread);
    set(State::Idle);
}

// All-SSID scan: tune the radio across the channel set, collect every beacon,
// and emit each NEW SSID. The RX collector runs on the device read thread; it
// captures only `this`, and _onScanAp is cleared (and the processor swapped to a
// no-op) before we return, so the still-running RX thread can't call a dead cb.
static inline bool isDfsChannel(int ch) {
    return (ch >= 52 && ch <= 64) || (ch >= 100 && ch <= 144);
}

void ApfpvStation::scanAll(int perChannelMs, bool includeDfs, const OnApFn& onAp) {
    if (!_rtl || !_rm) return;
    auto* rtl = reinterpret_cast<RtlJaguarDevice*>(_rtl);
    auto& rm  = *reinterpret_cast<RadioManagementModule*>(_rm);

    { std::lock_guard<std::mutex> lk(_scanMtx); _scanSeen.clear(); _onScanAp = onAp; }
    set(State::Scanning);

    // Diagnostics in MEMBERS so the stored collector (called from the device RX
    // thread) captures only `this` — capturing scanAll locals by ref dangles.
    _scanPkts.store(0); _scanBeacons.store(0);
    auto collector = [this](const Packet& pkt) {
        _scanPkts.fetch_add(1);
        std::string ssid; ApInfo info;
        if (!ScanProbe::parseAnyBeacon(pkt.Data.data(), pkt.Data.size(), ssid, info)) return;
        _scanBeacons.fetch_add(1);
        if (ssid.empty()) return;                       // skip hidden SSIDs
        info.rssi = (int)pkt.RxAtrib.rssi[0] - 110;     // approx dBm (gain byte)
        std::lock_guard<std::mutex> lk(_scanMtx);
        if (_onScanAp && _scanSeen.insert(ssid).second) _onScanAp(ssid, info);
    };
    SCANLOG("scanAll begin (perCh=%d ms)", perChannelMs);

    // hint first, then the full 5GHz set (UNII-1, UNII-2A/2C DFS, UNII-3) and all
    // 2.4GHz channels — APs (incl. phone hotspots/home routers) can sit anywhere,
    // e.g. DFS ch52-144. Passive RX-only listening on DFS is fine (no TX).
    // Trimmed for the ~per-channel cost of a real retune: UNII-1, UNII-2A DFS,
    // UNII-3 + the common 2.4 channels. (Dropped the big UNII-2C 100-144 block.)
    const int channels[] = {
        _params.channel,
        36, 40, 44, 48,            // UNII-1
        52, 56, 60, 64,            // UNII-2A (DFS)
        149, 153, 157, 161, 165,   // UNII-3
        1, 6, 11                   // 2.4GHz (common)
    };
    int last = 36; bool inited = false;
    for (int ch : channels) {
        if (ch <= 0) continue;
        if (!includeDfs && isDfsChannel(ch)) continue;   // skip DFS when disabled
        last = ch;
        SelectedChannel sc{ .Channel=(uint8_t)ch, .ChannelOffset=0,
                            .ChannelWidth=CHANNEL_WIDTH_20 };
        // Tuning is a USB register op (rtw_read/write) that can throw on a dongle
        // hiccup — catch per-channel so a single failure doesn't abort the app;
        // skip the bad channel and keep sweeping.
        try {
            if (!inited) { rtl->Init(collector, sc); inited = true; }     // bring-up + monitor RX
            else { rm.set_channel_bwmode((uint8_t)ch, 0, CHANNEL_WIDTH_20); } // actually retunes
        } catch (...) { continue; }
        std::this_thread::sleep_for(std::chrono::milliseconds(perChannelMs));
    }

    // Detach: no further emits, and replace the RX processor so the device's
    // read thread can't re-enter the collector after onAp's lifetime ends.
    { std::lock_guard<std::mutex> lk(_scanMtx); _onScanAp = nullptr; }
    SCANLOG("scanAll done: %d RX pkts, %d beacons, %zu SSIDs", _scanPkts.load(), _scanBeacons.load(), _scanSeen.size());
    try {
        rtl->Init([](const Packet&){}, SelectedChannel{ .Channel=(uint8_t)last,
                  .ChannelOffset=0, .ChannelWidth=CHANNEL_WIDTH_20 });
    } catch (...) {}
    set(State::Idle);
}

} // namespace apfpv
