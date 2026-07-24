// ============================================================================
//  StationMode.cpp  —  station-mode arming + ACK-survival probe (impl)
//  Declarations in StationMode.h. The auto-ACK arming sequence is ported from
//  the Linux driver hw_var_set_opmode STATION path (see docs/ARMING_SEQUENCE.md):
//      MACID -> MSR=STATION(0x02) -> BSSID -> RCR|=RCR_CBSSID_DATA(BIT6)
//  All register symbols confirmed in devourer hal/hal_com_reg.h.
// ============================================================================
#include "StationMode.h"
#include "Dot11Frames.h"
#include "StationTxDesc.h"
#include "jaguar1/PhydmWatchdog.h"
#if defined(__ANDROID__)
#include <android/log.h>
#endif
#include "hal_com_reg.h"

#include <cstring>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include "jaguar1/RadioManagementModule.h"
// Platform-agnostic logging (NDK on Android, compat stderr shim on host builds).
#include <android/log.h>
#define SMLOG(...) __android_log_print(ANDROID_LOG_INFO, "apfpv-arm", __VA_ARGS__)

namespace apfpv {

static constexpr uint8_t HW_STATE_STATION = 0x02;   // MSR_INFRA
static constexpr int     TXDESC_8812      = 40;

StationMode::~StationMode() { if (_alivePtr) _alivePtr->store(false); }
StationMode::StationMode(RtlAdapter& dev, RadioManagementModule& rm, SendFrameFn send)
    : _dev(dev), _rm(rm), _send(std::move(send)) {}

void StationMode::arm(const MacAddr& self, const MacAddr& bssid) {
#if defined(__ANDROID__)
    __android_log_print(ANDROID_LOG_INFO, "apfpv-scan",
        "arm ENTRY: current_channel=%d", (int)_rm.current_channel());
#endif
    // Desensitize the receiver's DIG for the AP's ACTUAL signal strength BEFORE auth. Until link-up
    // the DIG sits on monitor bounds [0x1c,0x2a] with the gain near its floor; at close range (a
    // strong beacon RSSI) that OVERLOADS the front-end -> false-alarm storm (fa>1000) -> the AP's
    // auth-response is never cleanly demodulated ("NO auth-resp" -> TXFAIL_NoAuth reconnect loop).
    // Feeding the measured beacon RSSI engages the connected DIG bounds [0x20,0x3e] and snaps IGI to
    // clamp(rssi+100, 0x20, 0x3e) — e.g. -20 dBm -> 0x3e (fully desensitized) — so a point-blank AP
    // can be received. On fail/disconnect the supervisor restores monitor mode via SetUnlinked().
    {
        int apRssi = _lastBeaconRssi.load();   // stable last-heard beacon RSSI (NOT _scanResult, reset per-channel)
        if (apRssi < 0) {
            // SetLinkRssi internally caps the derived IGI at the kernel's operating point (~0x37) so
            // a point-blank signal desensitizes enough to kill the saturation false-alarm storm
            // without going deaf (feeding raw -20 dBm would snap IGI to the 0x3e max = no RX).
            PhydmWatchdog::SetLinkRssi(apRssi);
#if defined(__ANDROID__)
            __android_log_print(ANDROID_LOG_INFO, "apfpv-scan",
                "arm: DIG desensitize for beacon RSSI=%ddBm (anti-saturation, IGI capped ~0x37)", apRssi);
#endif
        }
    }
    // (0) **SEQUENCE FIX — IQK on the OPERATING channel.** Faithful to the kernel
    // join order (rtw_mlme_ext.c join_cmd_hdl): HW_VAR_DO_IQK(TRUE) -> set_channel_
    // bwmode(final channel) so the IQK runs on the channel we will actually use,
    // -> MSR=STATION -> issue_auth. The devourer only ran the IQK during init on
    // the init/scan channel; the IQK output (per-channel TX/RX IQ calibration) is
    // then WRONG for the AP's channel, so the auth radiates mis-calibrated and the
    // AP can't decode it. Re-arm + re-run the channel-set here, on the channel the
    // scan settled on, immediately before auth. (TXPAUSE that the IQK/LCK leave is
    // cleared by step (4c) below, which runs after this.)
    if (!std::getenv("DEVOURER_SKIP_JOIN_IQK")) {
        // The async RX is drained by the caller (runProbe) for the ENTIRE arm, so the
        // IQK/LCK cal AND the MAC/MSR/RCR/BSSID writes below all run sequentially with
        // the device quiet — like the kernel. The OLD code paused RX only around the IQK
        // here and RESUMED mid-arm, so the RX-filter writes that follow still raced the
        // RX daemon -> corrupted filter -> auth-response dropped. Whole-arm serialization
        // measured 3/5 vs 1/5 (DEVOURER_NO_PAUSE_CAL disables the runProbe-level pause).
        _rm.ArmIQKOnNextChannelSet();
        // 40 MHz REQUIRES the HT40 prime-channel offset (LOWER/UPPER): with 0 (DONT_CARE) the
        // chip's subcarrier mapping is undefined ("SCMapping: DONOT CARE Mode Setting") and RX of
        // the AP's primary-channel frames is garbage — 40 MHz "connected but barely any frames".
        uint8_t off40 = (_connectWidth == CHANNEL_WIDTH_40)
                            ? _rm.prime_offset_40mhz(_rm.current_channel()) : 0;
        _rm.set_channel_bwmode(_rm.current_channel(), off40, _connectWidth);   // 20 or 40, via setConnectWidth
        SMLOG("arm: channel=%d offset=%d width=%s", (int)_rm.current_channel(), (int)off40,
              _connectWidth == CHANNEL_WIDTH_80 ? "80MHz" : _connectWidth == CHANNEL_WIDTH_40 ? "40MHz" : "20MHz");
#if defined(__ANDROID__)
        __android_log_print(ANDROID_LOG_INFO, "apfpv-scan",
            "arm set bandwidth: ch=%d off=%d width=%s",
            (int)_rm.current_channel(), (int)off40,
            _connectWidth == CHANNEL_WIDTH_80 ? "80MHz" : _connectWidth == CHANNEL_WIDTH_40 ? "40MHz" : "20MHz");
#endif
    }
    // (1) own MAC -> REG_MACID
    for (int i = 0; i < 4; ++i) _dev.rtw_write8(REG_MACID + i, self.b[i]);
    _dev.rtw_write16(REG_MACID + 4, (uint16_t)(self.b[4] | (self.b[5] << 8)));
    // (2) MSR -> STATION before auth. Faithful to the kernel's start_clnt_join,
    // which calls Set_MSR(WIFI_FW_STATION_STATE)=0x02 BEFORE start_clnt_auth/
    // issue_auth (rtw_mlme_ext.c). With MSR=STATION the AP's BSSID is the chip's
    // own network, so the HW MAC filters/auto-ACKs the AP during the auth/assoc
    // handshake. (The earlier "kernel auths in NO_LINK" theory was wrong — the
    // kernel source shows STATION pre-auth.) DEVOURER_AUTH_NOLINK forces the old
    // NO_LINK behaviour for A/B bisection.
    if (!std::getenv("DEVOURER_SKIP_MSR")) {
        uint8_t netType = std::getenv("DEVOURER_AUTH_NOLINK") ? 0x00 : HW_STATE_STATION;
        uint8_t msr = (uint8_t)((_dev.rtw_read8(MSR) & 0x0C) | netType);
        _dev.rtw_write8(MSR, msr);
    }
    // (3) AP BSSID -> REG_BSSID
    for (int i = 0; i < 4; ++i) _dev.rtw_write8(REG_BSSID + i, bssid.b[i]);
    _dev.rtw_write16(REG_BSSID + 4, (uint16_t)(bssid.b[4] | (bssid.b[5] << 8)));
    // (4) RCR: station receive config. CRITICAL FIX (usbmon diff vs kernel 88XXau):
    // the previous value (0x000000ce) omitted RCR_AMF, so the HW MAC dropped EVERY
    // management frame — the AP's auth/assoc RESPONSES never reached the driver
    // (only beacons slipped through via CBSSID_BCN). It also dropped the
    // APP_PHYST/APPFCS bits that define the RX buffer layout the FrameParser
    // expects (same as monitor mode), so the few frames that did arrive misparsed.
    // This now matches the kernel station RCR exactly (0xf40060ce) and the
    // devourer's own HalModule init RCR:
    //   RCR_AMF          accept management frames (auth/assoc/deauth responses)
    //   RCR_APP_PHYST    PHY status prepended -> parser/RSSI (monitor-mode layout)
    //   RCR_APPFCS       FCS appended         -> parser frame length
    //   FORCEACK         HW auto-ACKs unicast addressed to us (AP keeps the link)
    if (!std::getenv("DEVOURER_SKIP_RCR")) {
        // RCR_ADF (accept DATA frames) is ESSENTIAL: the WPA 4-way handshake
        // (EAPOL-Key M1..M4) rides in DATA frames, not management. Without it the HW
        // MAC drops EAPOL M1 -> onEapolKey is never called -> the 4-way times out ->
        // FailAuth. The kernel's 8812a RCR (rtl8812a_hal_init.c:4163) sets RCR_ADF;
        // RCR_ACF (control) matches its connect-mode RCR. This was THE 4-way bug.
        uint32_t rcr = RCR_APM | RCR_AM | RCR_AB | RCR_ADF | RCR_ACF |
                       RCR_CBSSID_DATA | RCR_CBSSID_BCN |
                       RCR_APP_ICV | RCR_AMF | RCR_HTC_LOC_CTRL | RCR_APP_MIC |
                       RCR_APP_PHYST_RXFF | RCR_APPFCS | FORCEACK |
                       // RCR_VHT_DACK (BIT26)=1: reply to VHT single-MPDU data with a normal
                       // ACK (kernel default), not a BlockAck. With it 0 the AP got no ACK and
                       // retransmitted every frame ~5× (measured). See RadioManagementModule.
                       RCR_VHT_DACK;
        _dev.rtw_write32(REG_RCR, rcr);
        // CRITICAL: accept ALL mgmt frame subtypes during arm phase.
        // Without this, auth/assoc responses are HW-filtered.
        _dev.rtw_write16(REG_RXFLTMAP0, 0xFFFF);
        _dev.rtw_write16(REG_RXFLTMAP2, 0xFFFF);
    }
    // (4a) Enable HW TSF update for the STA. Faithful to the kernel's
    // rtw_iface_enable_tsf_update -> rtw_hal_set_tsf_update(1) at join, which clears
    // DIS_TSF_UDT (BIT4) of REG_BCN_CTRL (0x550) so the HW syncs its TSF timer to the
    // AP's beacons/probe-resp (needed for TBTT/PS keepalive — else the AP drops us
    // after assoc). The devourer's monitor init left REG_BCN_CTRL = DIS_TSF_UDT
    // (HalModule). The kernel gates this on RCR_CBSSID_BCN, which we set above.
    // DEVOURER_SKIP_TSF disables for A/B.
    if (!std::getenv("DEVOURER_SKIP_TSF")) {
        _dev.rtw_write8(0x0550, (uint8_t)(_dev.rtw_read8(0x0550) & ~0x10));
    }
    // (4b) Join-time MAC timing the kernel 88XXau re-tunes on association but the
    // devourer's init left at monitor/default values (usbmon diff). Most important:
    // our VO/mgmt EDCA had CWmax=1023 (ECWmax 0xa) vs the kernel's 7 (0x3) — a huge
    // random backoff before every management TX. Port the kernel's exact values so
    // the auth/assoc TX is scheduled like the kernel's. Env-gated for A/B.
    if (!std::getenv("DEVOURER_SKIP_EDCA")) {
        _dev.rtw_write32(0x0500, 0x002f3222);   // REG_EDCA_VO_PARAM (mgmt/voice)
        _dev.rtw_write32(0x0504, 0x005e4322);   // REG_EDCA_VI_PARAM
        _dev.rtw_write32(0x0508, 0x005ea42b);   // REG_EDCA_BE_PARAM
        _dev.rtw_write32(0x050c, 0x0000a44f);   // REG_EDCA_BK_PARAM
        _dev.rtw_write8(0x0607, 0x07);          // REG_RRSR byte3 (response-rate ctrl)
    }
    // (4c) **THE TX-SILENCE FIX.** The IQK (Iqk8812a::ConfigureMac, run on every
    // channel-set) writes REG_TXPAUSE (0x522) = 0x3f to PAUSE all 6 TX queues during
    // calibration, but the devourer's IQK port NEVER restores it to 0 (the kernel's
    // _PHY_ReloadMACRegisters does). usbmon diff of the kernel's full init vs ours:
    // kernel TXPAUSE=0x00, devourer=0x3f. With all queues paused the chip ACCEPTS
    // the bulk-OUT into the TX FIFO ("70 bytes OK") but never transmits it -> the
    // auth/assoc never radiates and the AP never replies. Clear it here (and it is
    // the proper place: after the last channel-set/IQK, before the first TX).
    if (!std::getenv("DEVOURER_SKIP_TXPAUSE_CLR")) {
        _dev.rtw_write8(0x0522, 0x00);          // REG_TXPAUSE — release all TX queues
        SMLOG("TXPAUSE(0x522) cleared to 0x%02x", _dev.rtw_read8(0x0522));
    }
    // **THE TX-RADIATION FIX (RFE / RF front-end).** usbmon diff of the kernel's
    // full init vs ours: the kernel sets REG_RFE_CTRL 0xcb8/0xeb8 = 0x42825000
    // (routes the external PA + the antenna TX/RX switch for the TX path); the
    // devourer left them at 0x00000000. Result: the MAC transmits the frame
    // (TXPKT_EMPTY drains, "OK 70 bytes") but the signal never reaches the antenna
    // — the AP hears nothing — while RX keeps working through the LNA path. This
    // is the real cause of the "on-air silence". Also restore the TX-path AGC
    // 0xc1c/0xe1c to the kernel's value. Env-gated for A/B.
    if (std::getenv("DEVOURER_TRY_RFE")) {      // opt-in: a lone 0xcb8 write breaks RX;
                                                // needs the full RFE-config sequence.
        // 0xcb8 is PAGED: Page C = rA_RFE_Jaguar (RF front-end / PA+antenna TR
        // switch), Page C1 = rOFDM0_TxCoeff6 (a TX-IQK coefficient). Iqk8812a leaves
        // the BB in Page C1 and writes 0xcb8=0 there, so the RFE is never set to the
        // operational value AND a naive write here would corrupt the TX-IQK coeff.
        // Select Page C, then set RFE to the kernel's 8812 value (0x42825000) — our
        // 8814-derived PHY table wrongly used 0x00500000 and the IQK zeroed it.
        _dev.phy_set_bb_reg(0x82c, 0x80000000, 0x0);   // BB Page C (bit31=0)
        _dev.rtw_write32(0x0cb8, 0x42825000);          // rA_RFE_Jaguar
        _dev.rtw_write32(0x0eb8, 0x42825000);          // rB_RFE_Jaguar
        SMLOG("RFE set (Page C) 0xcb8/0xeb8=0x42825000");
    }
    // NOTE: the firmware "connected" H2C (MACID_CFG 0x40 / MEDIA_STATUS 0x01 /
    // RSSI 0x42) is deliberately NOT sent here. The kernel sends it only AFTER the
    // assoc response — see becomeStation(). Sending it pre-auth told the firmware
    // that MACID 0 was already a connected peer. DEVOURER_AUTH_STA reverts to the
    // old (pre-auth-H2C) behaviour for A/B.
    if (std::getenv("DEVOURER_AUTH_STA") && !std::getenv("DEVOURER_SKIP_H2C"))
        sendStationH2C(/*macid=*/0, bssid);
    _armed = true;
}

// Post-association transition: NOW the station is associated, so switch the chip
// to STATION network type and register the AP as a firmware peer (MACID/RA +
// media-status + RSSI H2C). This is the kernel's order — do it only after the
// assoc response, never before auth.
void StationMode::becomeStation(const MacAddr& bssid) {
    if (!std::getenv("DEVOURER_SKIP_MSR")) {
        uint8_t msr = (uint8_t)((_dev.rtw_read8(MSR) & 0x0C) | HW_STATE_STATION);
        _dev.rtw_write8(MSR, msr);
    }
    // Switch the RX filter from monitor (promiscuous, no ACK) to real-station (our-MAC + FORCEACK)
    // so the chip HW-ACKs the AP's unicast RTP — without this the AP retransmits every frame and the
    // link is choppy. Env-gated for A/B (DEVOURER_NO_STATION_ACK keeps the old monitor RX filter).
    // Windows sequence: H2C 0x34 (FW join) fires BEFORE RCR is set. The firmware
    // needs to enter offload mode before the station RX filter activates.
    if (!std::getenv("DEVOURER_SKIP_H2C")) sendStationH2C(/*macid=*/0, bssid);
    if (!std::getenv("DEVOURER_NO_STATION_ACK")) _rm.SetStationRxFilter();
    SMLOG("becomeStation: MSR->STATION + connected H2C sent");
}

// Port of the Linux 88XXau station-association H2C firmware sequence, captured by
// usbmon-diffing a working wpa_supplicant association to OpenIPC against the
// devourer's arm(). The register arming above (MACID/MSR/BSSID/RCR) is NOT enough:
// the firmware must also have the AP registered as a *peer* (MACID) with a rate
// table + RSSI, or the chip's TX scheduler cannot pick a rate for it and the
// bulk-OUT TX FIFO wedges after the first frame (the "first TX OK, then
// -7/timeout" station-mode symptom). Each command is the elementID + params the
// kernel writes to the HMEBOX mailbox (primary box 0x1D0 + ext box 0x1F0).
//
// Individual commands are env-gated so each can be isolated on the native probe:
//   DEVOURER_SKIP_MACIDCFG / DEVOURER_SKIP_MEDIASTATUS / DEVOURER_SKIP_RSSI.
void StationMode::sendStationH2C(uint8_t macid, const MacAddr& bssid) {
    (void)bssid;
    // H2C_MACID_CFG / PHYDM_H2C_RA_MASK (0x40): rate-adaptation + aggregation config.
    // Kernel PHYDM format (phydm_rainfo.c:phydm_ra_h2c): 7-byte payload with macid,
    // rate_id, SGI, init_ra_lv, bw_mode, VHT-enable, LDPC, dis_ra, dis_pt, + 32-bit mask.
    // The old simplified format (macid, 0xac=dis_ra=1, ...) told firmware to DISABLE RA
    // (and implicitly Block-Ack) for this macid. The PHYDM format arms the full engine.
    // rate_id = PHYDM_ARFR0_AC_2SS = 9 (VHT 2SS 5GHz initial rate).
    // bw_mode = 2 (80MHz), VHT=1, LDPC=1, dis_ra=0, dis_pt=0, SGI=1, init_ra_lv=0.
    // ra_mask = 0x3FFFFFFF (OFDM + HT-MCS 1/2SS + VHT 1SS partial, generous initial mask).
    // Register-based H2C (vendor format). PKT H2C (32-byte rtw88 format) was
    // tested, did not change BACAM_CMD=0 — the v52.14 firmware binary only
    // accepts the register-based (HMEBOX ElementID + buffer) format.
    bool h2cRa = true;
    if (!std::getenv("DEVOURER_SKIP_MACIDCFG")) {
        // RA H2C (peer rate-adaptation config). Kernel usbmon payload = 00 a9 12 10 f0 ff ff
        // (init_ra_lv=1, LDPC=0, ra_mask=0xfffff010). Matched init_ra_lv/LDPC to the kernel, but
        // the aggressive VHT ra_mask is OPT-IN (DEVOURER_RA_KERNEL): forcing the top VHT-2SS rates
        // for OUR uplink ACKs/BAs is only safe if our uplink is as strong as the AP's downlink;
        // when it isn't, the FW picks an unreliable TX rate and the BlockAcks are lost (measured:
        // 65Mbps run degraded with it on). The conservative mask is the safe default; the RA H2C
        // did NOT change the AP's downlink bw=0/20MHz anyway — that's gated on the HW BlockAck.
        uint8_t rate_id = 9; uint8_t init_ra_lv = 1; uint8_t sgi = 1;
        uint8_t bw_mode = 2; uint8_t vht_en = 1; uint8_t ldpc_en = 0;
        uint8_t dis_pt = 0; uint8_t dis_ra = 0;
        // Kernel-exact ra_mask (0xfffff010): OFDM-6M fallback (bit4) + ALL HT/VHT rates incl the
        // top VHT-2SS MCS8/9 (bits 30/31). Our old 0x3FFFFFFF included invalid 5GHz CCK (bits 0-3)
        // AND dropped the top VHT bits — the wrong peer profile for the FW. Affects our TX-data
        // rate only (the HW BlockAck radiates at a basic rate), so it's safe for RX video.
        // DEVOURER_RA_CONSERVATIVE falls back to the old mask if a weak uplink needs it.
        // ★ KERNEL-VERBATIM MACID_CFG (reconstructed from the kernel's live usbmon H2C box:
        // payload = [00, 89, 1a, 00, 80, ff, ff]). Our prior "kernel-exact" values were WRONG —
        // init_ra_lv=1 (s/b 0 -> byte1 0xA9 vs kernel 0x89), byte2 0x12 (s/b 0x1a, kernel sets
        // bit3), ra_mask 0xfffff010 (s/b 0xffff8000). The firmware uses this per-peer profile to
        // pick the rate for the compressed BA it auto-transmits; a wrong profile sends the BA at a
        // rate the AP misses -> AP shrinks its A-MPDU + DELBAs -> the ~28Mbps cap. DEVOURER_RA_OLD
        // restores the prior (wrong) values for A/B.
        uint8_t macidCfg[7];
        if (std::getenv("DEVOURER_RA_OLD")) {
            uint32_t ra_mask = std::getenv("DEVOURER_RA_CONSERVATIVE") ? 0x3FFFFFFF : 0xfffff010;
            macidCfg[0] = macid;
            macidCfg[1] = (uint8_t)((rate_id & 0x1f) | ((init_ra_lv & 0x3) << 5) | (sgi << 7));
            macidCfg[2] = (uint8_t)(bw_mode | (ldpc_en << 2) | (vht_en << 4) | (dis_pt << 6) | (dis_ra << 7));
            macidCfg[3] = (uint8_t)(ra_mask & 0xff); macidCfg[4] = (uint8_t)((ra_mask >> 8) & 0xff);
            macidCfg[5] = (uint8_t)((ra_mask >> 16) & 0xff); macidCfg[6] = (uint8_t)((ra_mask >> 24) & 0xff);
        } else {
            macidCfg[0] = macid;                                  // macid 0 (AP peer)
            macidCfg[1] = 0x89;                                   // rate_id=9, init_ra_lv=0, sgi=1
            macidCfg[2] = 0x1a;                                   // bw80 + VHT + bit3
            macidCfg[3] = 0x00; macidCfg[4] = 0x80; macidCfg[5] = 0xff; macidCfg[6] = 0xff;  // ra_mask 0xffff8000
        }
        h2cRa = _dev.fillH2CCmd(0x40, 7, macidCfg);
    }
    bool h2cMs = true;
    // The kernel does NOT send H2C 0x01 (media-status) for the 8812 station (usbmon: 0 writes;
    // MSR is set via REG_CR instead). We sent byte0=0x21 — an extra command that may park the FW
    // peer state wrong. Default OFF now; DEVOURER_MEDIASTATUS re-enables for A/B.
    if (std::getenv("DEVOURER_MEDIASTATUS")) {
        uint8_t msrParm[3] = { 0x21, macid, macid };
        h2cMs = _dev.fillH2CCmd(0x01, 3, msrParm);
    }
    bool h2cRssi = true;
    if (!std::getenv("DEVOURER_SKIP_RSSI")) {
        // Kernel-verbatim H2C_RSSI_SETTING: payload [macid, 0x00, rssi] (3 bytes); rssi ~0x2e at
        // -54dBm. Was [macid,0,0x39,0x06] (4 bytes, static, wrong). DEVOURER_RA_OLD keeps the old.
        if (std::getenv("DEVOURER_RA_OLD")) {
            uint8_t rssiSet[4] = { macid, 0x00, 0x39, 0x06 };
            h2cRssi = _dev.fillH2CCmd(0x42, 4, rssiSet);
        } else {
            uint8_t rssiSet[3] = { macid, 0x00, 0x2e };
            h2cRssi = _dev.fillH2CCmd(0x42, 3, rssiSet);
        }
    }
    SMLOG("station H2C: MACID_CFG=%d MEDIA_STATUS=%d RSSI=%d",
          h2cRa ? 1 : 0, h2cMs ? 1 : 0, h2cRssi ? 1 : 0);
    // Hal_PatchwithJaguar_8812 (kernel: hal_com.c:4355-4363, called after
    // H2C_MEDIA_STATUS_RPT for RTL8812 in station mode). Configures VHT LSIG
    // length and BW indication for non-Jaguar AP peers. Without this the chip
    // may mis-handle VHT A-MPDU aggregates. PLATFORM-AGNOSTIC: these are plain
    // USB register writes — was incorrectly gated #ifdef __ANDROID__, which
    // skipped the VHT A-MPDU patch on native Windows/Linux hosts and let the AP
    // DELBA the session (no compressed BA for the aggregate). Apply everywhere.
    // rVhtlen_Use_Lsig_Jaguar = 0x8c3 (BB register), 0x3F for non-Jaguar AP
    _dev.rtw_write8(0x8c3, 0x3F);
    // REG_TCR (0x0604) + 3 = 0x0607: set bits 0,1,2 for VHT A-MPDU timing
    uint8_t tcr3 = _dev.rtw_read8(0x0607);
    _dev.rtw_write8(0x0607, (uint8_t)(tcr3 | 0x07)); // BIT0|BIT1|BIT2
    // rBWIndication_Jaguar (0x834) + 3 = 0x837: clear BIT2 for non-Jaguar AP
    uint8_t bw3 = _dev.rtw_read8(0x837);
    _dev.rtw_write8(0x837, (uint8_t)(bw3 & ~0x04)); // ~BIT2
}

// Management-frame TX rate, BAND-AWARE. Faithful to the kernel
// init_mlme_ext_priv_value (rtw_mlme_ext.c): a 5 GHz channel (ch>14) has no CCK
// PHY, so auth/assoc MUST go at OFDM-6M (DESC_RATE6M=0x04); 2.4 GHz uses the
// robust CCK-1M (DESC_RATE1M=0x00). Sending CCK-1M on 5 GHz is un-transmittable —
// the frame drains from the TX FIFO ("OK 70 bytes") but never radiates, which was
// our 5 GHz on-air silence. DEVOURER_MGMT_RATE overrides for A/B.
static uint8_t mgmtTxRate(int channel) {
    if (const char* r = std::getenv("DEVOURER_MGMT_RATE"))
        return (uint8_t)strtoul(r, nullptr, 0);
    return (channel > 14) ? 0x04 : 0x00;   // OFDM-6M (5 GHz) | CCK-1M (2.4 GHz)
}
// Kernel 88XXau auth/assoc TX descriptor uses MACID=1 + RAID=8 (usbmon diff),
// NOT MACID=0/RAID=0. MACID 0 is special on Jaguar; the TX scheduler treats a
// real AP peer as a non-zero MACID. Env-overridable for A/B bisection.
static uint8_t mgmtMacId() {
    if (const char* m = std::getenv("DEVOURER_MGMT_MACID"))
        return (uint8_t)strtoul(m, nullptr, 0);
    return 1;
}
// Rate-id for the auth/assoc TX descriptor, BAND-AWARE — matches the kernel
// rtw_get_mgntframe_raid: non-CCK (5 GHz) => RATEID_IDX_G(7), 11B (2.4 GHz) =>
// RATEID_IDX_B(8). Pairs with mgmtTxRate(). DEVOURER_MGMT_RAID overrides for A/B.
static uint8_t mgmtRaid(int channel) {
    if (const char* r = std::getenv("DEVOURER_MGMT_RAID"))
        return (uint8_t)strtoul(r, nullptr, 0);
    return (channel > 14) ? 7 : 8;   // RATEID_IDX_G (5 GHz) | RATEID_IDX_B (2.4 GHz)
}
// Diagnostic: send auth/assoc as BMC=1 (broadcast TX desc — the monitor path that
// is PROVEN to radiate) while still addressed to the AP. If the AP then replies,
// the BMC=0 unicast TX path (ACK-wait/retry) was the non-radiating culprit.
static apfpv::StationFrameKind mgmtKind() {
    if (std::getenv("DEVOURER_AUTH_BMC"))
        return apfpv::StationFrameKind::BroadcastMgmt;
    return apfpv::StationFrameKind::Mgmt;
}

bool StationMode::sendAuthOpenSeq1(const MacAddr& self, const MacAddr& bssid) {
    Mac s, b; std::copy(self.b.begin(), self.b.end(), s.begin());
    std::copy(bssid.b.begin(), bssid.b.end(), b.begin());
    auto mpdu = BuildAuthOpenSeq1(s, b);
    std::vector<uint8_t> frame(TXDESC_8812 + mpdu.size(), 0);
    std::memcpy(frame.data() + TXDESC_8812, mpdu.data(), mpdu.size());
    FillStationTxDesc(frame.data(), (uint16_t)mpdu.size(), TXDESC_8812,
                      mgmtMacId(), mgmtKind(),
                      mgmtRaid(_rm.current_channel()), mgmtTxRate(_rm.current_channel()));
    if (std::getenv("DEVOURER_LOG_AUTHTX")) {
        std::fprintf(stderr, "[authtx] mpdu %zuB:", mpdu.size());
        for (size_t i = 0; i < mpdu.size(); ++i) std::fprintf(stderr, " %02x", mpdu[i]);
        std::fprintf(stderr, "\n");
    }
    return _send(frame);
}

bool StationMode::sendAssocRequest(const MacAddr& self, const MacAddr& bssid, const char* ssid) {
    Mac s, b; std::copy(self.b.begin(), self.b.end(), s.begin());
    std::copy(bssid.b.begin(), bssid.b.end(), b.begin());
    uint16_t rsnCaps = (uint16_t)((_pmf >= 1 ? 0x0080 : 0) | (_pmf >= 2 ? 0x0040 : 0)); // MFPC|MFPR
    auto mpdu = BuildAssocRequest(s, b, std::string(ssid), _pairwise, _group, rsnCaps);
    std::vector<uint8_t> frame(TXDESC_8812 + mpdu.size(), 0);
    std::memcpy(frame.data() + TXDESC_8812, mpdu.data(), mpdu.size());
    FillStationTxDesc(frame.data(), (uint16_t)mpdu.size(), TXDESC_8812,
                      mgmtMacId(), mgmtKind(),
                      mgmtRaid(_rm.current_channel()), mgmtTxRate(_rm.current_channel()));
    return _send(frame);
}

static inline uint8_t fc_type(uint16_t fc)    { return (fc >> 2) & 0x3; }
static inline uint8_t fc_subtype(uint16_t fc) { return (fc >> 4) & 0xF; }

void StationMode::onMgmtFrame(uint16_t fc, const MacAddr& a1, const MacAddr& a2) {
    if (fc_type(fc) != 0x0) return;          // mgmt only
    uint8_t sub = fc_subtype(fc);
    if (sub != 0x8) {
        SMLOG("rx mgmt subtype 0x%X", sub);   // log non-beacon mgmt (auth/assoc/deauth)
        if (std::getenv("DEVOURER_LOG_MGMT"))
            std::fprintf(stderr, "[mgmt-rx] sub=0x%X a1=%02x:%02x:%02x:%02x:%02x:%02x "
                         "a2=%02x:%02x:%02x:%02x:%02x:%02x\n", sub,
                         a1.b[0],a1.b[1],a1.b[2],a1.b[3],a1.b[4],a1.b[5],
                         a2.b[0],a2.b[1],a2.b[2],a2.b[3],a2.b[4],a2.b[5]);
    }
    // Only believe a response actually addressed to us (a1=self) FROM our AP (a2=bssid).
    // Overheard auth/assoc frames between other stations share the subtype and would
    // otherwise false-trigger the join (confirmed root cause of the bogus "Associating").
    if (!(a1.b == _expectSelf.b && a2.b == _expectBssid.b)) {
        if (std::getenv("DEVOURER_LOG_MGMT"))
            std::fprintf(stderr, "[mgmt-rx]   ^ ignored (not addressed to us from our AP)\n");
        return;
    }
    switch (sub) {
        case 0xB: _gotAuthResp = true; break;  // Auth
        case 0x1: _gotAssocOk  = true; break;  // Assoc-Resp
        case 0xC: case 0xA: _gotDeauth = true; break;  // Deauth/Disassoc
        default: break;
    }
}

StationMode::Result
StationMode::runProbe(const MacAddr& self, const MacAddr& bssid,
                      const char* ssid, int holdSeconds) {
    using namespace std::chrono;
    _expectSelf = self; _expectBssid = bssid;   // onMgmtFrame matches a1==self && a2==bssid
    // DAEMON-vs-DIRECT-CALL FIX: the IQK/LCK inside arm() does dozens of register reads
    // (control transfers). With the async RX worker thread live, those control reads
    // serialize behind the pending bulk-IN URBs on vhci/WinUSB (same root cause as the
    // bulk-OUT wedge) -> the IQK ready-poll is slowed/garbled -> the cal converges only
    // ~2/5. Drain the RX pool for the cal so its control I/O is clean, like the kernel
    // (which runs cal with no userspace RX thread contending). Resume re-arms RX so the
    // auth-response that follows is still caught.
    bool pauseForCal = !std::getenv("DEVOURER_NO_PAUSE_CAL");
    {
        // RAII, scoped to just the cal: arm()/RunIQK() below do raw register I/O
        // (rtw_read32/write32) that CAN throw (control-transfer failure, replug mid-cal).
        // The old code called resumeAsyncRx() only after arm() returned normally, so a
        // throw left the async RX pool paused AND _rxQuiesce stuck true forever — the next
        // connect's stopAsyncRx() then waits on an event thread that's permanently yielding
        // (per pauseAsyncRx's _rxQuiesce contract), times out, and frees URBs libusb still
        // owns -> UAF/destroyed-mutex SIGABRT. Guarantee resumeAsyncRx() runs on every exit
        // path instead — scoped to this block so RX resumes here, same as before, not at
        // runProbe's return (which would leave RX paused through the auth-response wait).
        struct ResumeGuard {
            RtlAdapter& dev; bool active;
            ~ResumeGuard() { if (active) dev.resumeAsyncRx(); }
        } resumeGuard{_dev, pauseForCal};
        if (pauseForCal) _dev.pauseAsyncRx();
        arm(self, bssid);
        // IQK CONVERGENCE-VERIFIED RETRY: the path-A TX I/Q cal converges only ~2/5 of the
        // time (worse under USB latency, e.g. a VM's emulated xHCI), and when it falls back
        // to the IQK-default matrix (TXiqcX=0x200, TXiqcY=0x000) the auth radiates
        // mis-modulated and the AP can't decode it -> no auth-resp -> stuck Authenticating.
        // Read the post-cal path-A TX-IQC and re-run the IQK (RX still paused for clean
        // control I/O) until it converges off the default, up to a bounded number of tries.
        auto readTxIqcX = [&]() -> uint32_t {
            try {
                uint32_t pc = _dev.rtw_read32(0x082c);
                _dev.rtw_write32(0x082c, pc | 0x80000000u);   // Page C1 -> TX-IQC matrix
                uint32_t x = _dev.rtw_read32(0x0cd4) & 0x7ff;
                _dev.rtw_write32(0x082c, pc);
                return x;
            } catch (...) { return 0x200; }
        };
        int iqkTries = 6;
        if (const char* e = std::getenv("DEVOURER_IQK_RETRIES")) iqkTries = atoi(e);
        uint32_t iqcx0 = readTxIqcX();
        fprintf(stderr, "[iqk-verify] post-arm TXiqcX=0x%03x (0x200=default/not-converged)\n", iqcx0);
        for (int it = 0; it < iqkTries && readTxIqcX() == 0x200; ++it) {
            fprintf(stderr, "[iqk-retry] TX-cal did not converge, re-running IQK (try %d)\n", it + 1);
            try { _rm.RunIQK(); } catch (...) {}
            fprintf(stderr, "[iqk-retry] -> TXiqcX=0x%03x\n", readTxIqcX());
        }
    }  // ResumeGuard fires resumeAsyncRx() here — same position as the original explicit call.
    // ACK-DRIVEN VERIFY: read back the RX-filter state the chip holds after arm(). The auth
    // radiates every run (FIFO=4095) yet the AP reply is missed in silent runs — a raced
    // RCR/MSR/BSSID write would silently drop that reply. Compare silent vs connecting.
    try {
        uint32_t rcr=_dev.rtw_read32(0x608); uint8_t msr=_dev.rtw_read8(0x102);
        uint8_t b0=_dev.rtw_read8(0x618),b1=_dev.rtw_read8(0x619),b2=_dev.rtw_read8(0x61a),
                b3=_dev.rtw_read8(0x61b),b4=_dev.rtw_read8(0x61c),b5=_dev.rtw_read8(0x61d);
        uint8_t m0=_dev.rtw_read8(0x610),m1=_dev.rtw_read8(0x611),m2=_dev.rtw_read8(0x612),
                m3=_dev.rtw_read8(0x613),m4=_dev.rtw_read8(0x614),m5=_dev.rtw_read8(0x615);
        fprintf(stderr,"[arm-verify] RCR=0x%08x MSR=0x%02x BSSID=%02x:%02x:%02x:%02x:%02x:%02x "
                "MACID=%02x:%02x:%02x:%02x:%02x:%02x SELF=%02x:%02x:%02x:%02x:%02x:%02x\n",
                rcr,msr,b0,b1,b2,b3,b4,b5,m0,m1,m2,m3,m4,m5,
                self.b[0],self.b[1],self.b[2],self.b[3],self.b[4],self.b[5]);
    } catch (...) {}
    SMLOG("arm: self %02x:%02x:%02x:%02x:%02x:%02x -> bssid %02x:%02x:%02x:%02x:%02x:%02x",
          self.b[0],self.b[1],self.b[2],self.b[3],self.b[4],self.b[5],
          bssid.b[0],bssid.b[1],bssid.b[2],bssid.b[3],bssid.b[4],bssid.b[5]);
    if (!_armed) return Result::Error;
    // CRITICAL TX-POWER FIX: arm()'s channel-set runs PHY_SetTxPowerLevel8812, which resets
    // power to the EEPROM by-rate index (~0 here = on-air SILENCE). Any earlier SetTxPower
    // (ApfpvStation connect-path) is therefore overridden, so the auth keyed the PA at zero
    // power and the AP's hostapd NEVER heard it (proven via AP-side capture), while the
    // beacon path radiated at -52 dBm because its SetTxPower is the last power write.
    // Re-apply a solid uniform index AFTER the channel-set so the auth actually radiates.
    { uint8_t p = 40; if (const char* e = std::getenv("DEVOURER_TX_PWR_IDX")) p = (uint8_t)atoi(e);
      try { _rm.SetTxPower(p); } catch (...) {} }
    // DEAUTH-BEFORE-AUTH: clear any stale association the AP still holds for our MAC from a
    // prior (possibly incomplete) attempt. Without it the AP ignores our re-auth (it still
    // thinks we're associated) and we stall at Authenticating on alternate back-to-back
    // connects. This is exactly what wpa_supplicant sends on (re)connect.
    if (!std::getenv("DEVOURER_SKIP_DEAUTH")) {
        uint8_t m[26] = {0xC0,0x00, 0x00,0x00};       // FC=mgmt/deauth, duration
        std::memcpy(m+4,  bssid.b.data(), 6);          // a1 = AP
        std::memcpy(m+10, self.b.data(),  6);          // a2 = self
        std::memcpy(m+16, bssid.b.data(), 6);          // a3 = AP
        m[24]=0x03; m[25]=0x00;                         // reason 3 (STA leaving)
        std::vector<uint8_t> df(TXDESC_8812 + sizeof(m), 0);
        std::memcpy(df.data()+TXDESC_8812, m, sizeof(m));
        FillStationTxDesc(df.data(), (uint16_t)sizeof(m), TXDESC_8812, mgmtMacId(), mgmtKind(),
                          mgmtRaid(_rm.current_channel()), mgmtTxRate(_rm.current_channel()));
        _send(df);
        std::this_thread::sleep_for(milliseconds(60));  // let the AP drop the old STA
    }
    if (_onPhase) _onPhase(1);                         // -> Authenticating
    bool sent = sendAuthOpenSeq1(self, bssid);
    // RF-front-end + TX-path state at the auth TX (fires every arm, sent or not) to
    // catch the "drains FIFO but nothing radiates" silence: RFE (antenna TR switch,
    // BB Page C1 via 0x82c bit31), ANT sel, TXPAUSE, CCA, BB path-A TX enable.
    try {
        uint32_t pc = _dev.rtw_read32(0x082c);
        _dev.rtw_write32(0x082c, pc & ~0x80000000u);
        uint32_t rfeA = _dev.rtw_read32(0x0cb8);
        _dev.rtw_write32(0x082c, pc | 0x80000000u);   // Page C1 for TX-IQC matrix
        uint32_t txY = _dev.rtw_read32(0x0ccc) & 0x7ff, txX = _dev.rtw_read32(0x0cd4) & 0x7ff;
        _dev.rtw_write32(0x082c, pc);
        // TXiqcX=0x200,TXiqcY=0x0 = IQK-default (path-A TX-cal did NOT converge this arm).
        fprintf(stderr, "[auth-tx-rf] sent=%d CR=0x%04x TXPAUSE=0x%02x ANT808=0x%02x CCA838=0x%08x RFEcb8=0x%08x TXiqcX=0x%03x TXiqcY=0x%03x BBtxA=0x%08x\n",
                (int)sent, _dev.rtw_read16(0x0100), _dev.rtw_read8(0x0522), _dev.rtw_read8(0x0808),
                _dev.rtw_read32(0x0838), rfeA, txX, txY, _dev.rtw_read32(0x0c04));
    } catch (...) { fprintf(stderr, "[auth-tx-rf] register read THREW\n"); }
    // A single sent=0 (bulk-OUT flicker) used to hard-Error here, killing the arm before the
    // retransmit loop below ever ran. Instead retry the send a few times immediately; the OUT
    // path recovers on the next attempt. Only if EVERY attempt fails do we bail (and then as
    // TXFAIL_NoAuthResp, which the supervisor retries cleanly — not the hard Error that the
    // scan/arm loop treats as fatal).
    if (!sent) {
        for (int i = 0; i < 5 && !sent; ++i) {
            std::this_thread::sleep_for(milliseconds(60));
            sent = sendAuthOpenSeq1(self, bssid);
        }
        SMLOG("auth-req initial sent=0 -> retried, now sent=%d", sent ? 1 : 0);
    }
    SMLOG("auth-req tx sent=%d, waiting 3s for auth-resp...", sent ? 1 : 0);

    auto t0 = steady_clock::now();
    auto lastTx = t0;
    int retx = 0;
    while (!_gotAuthResp && !_gotDeauth && steady_clock::now() - t0 < seconds(3)) {
        std::this_thread::sleep_for(milliseconds(50));
        // Retransmit the auth like the kernel MLME (it retries ~3x at ~200ms). If our
        // single auth-req flickered off-air on this arm, another frame may radiate and
        // reach the AP. Cheap, kernel-faithful, and directly tests per-frame TX flicker.
        if (!_gotAuthResp && retx < 8 &&
            steady_clock::now() - lastTx > milliseconds(300)) {
            sendAuthOpenSeq1(self, bssid);
            lastTx = steady_clock::now();
            ++retx;
        }
    }
    if (!_gotAuthResp) {
        try {
            fprintf(stderr, "[auth-fail] CR=0x%04x TXPKT_EMPTY=0x%04x HISR=0x%08x FIFOPAGE0x200=0x%08x REG0x10c=0x%04x\n",
                    _dev.rtw_read16(0x0100), _dev.rtw_read16(0x041a), _dev.rtw_read32(0x00b4),
                    _dev.rtw_read32(0x0200), _dev.rtw_read16(0x010c));
            // RF front-end state — RFE (antenna TR switch, BB Page C1 via 0x82c bit31),
            // antenna sel, TXPAUSE, CCA. 0 RFE = "drains FIFO but nothing radiates".
            uint32_t pc = _dev.rtw_read32(0x082c);
            _dev.rtw_write32(0x082c, pc & ~0x80000000u);
            uint32_t rfeA = _dev.rtw_read32(0x0cb8), rfeB = _dev.rtw_read32(0x0eb8);
            _dev.rtw_write32(0x082c, pc);
            fprintf(stderr, "[auth-fail-rf] TXPAUSE0x522=0x%02x ANT808=0x%02x CCA838=0x%08x RFEcb8=0x%08x RFEeb8=0x%08x BBtxA_c04=0x%08x\n",
                    _dev.rtw_read8(0x0522), _dev.rtw_read8(0x0808), _dev.rtw_read32(0x0838),
                    rfeA, rfeB, _dev.rtw_read32(0x0c04));
        } catch (...) { fprintf(stderr, "[auth-fail] chip register read THREW -> whole USB wedged (not just bulk-OUT)\n"); }
        SMLOG("NO auth-resp (deauth=%d)", _gotDeauth.load()); return Result::TXFAIL_NoAuthResp;
    }
    SMLOG("got auth-resp! sending assoc-req...");

    if (_onPhase) _onPhase(2);                          // -> Associating
    std::this_thread::sleep_for(milliseconds(100));
    if (!sendAssocRequest(self, bssid, ssid)) return Result::Error;

    auto t1 = steady_clock::now();
    while (!_gotAssocOk && !_gotDeauth && steady_clock::now() - t1 < seconds(3))
        std::this_thread::sleep_for(milliseconds(50));
    if (_gotDeauth)   return Result::NOGO_Deauthed;
    if (!_gotAssocOk) return Result::NOGO_NoAssocResp;

    // Associated! NOW become a connected station (MSR->STATION + connected H2C),
    // matching the kernel's post-assoc order. The 4-way handshake + data follow.
    becomeStation(bssid);

    auto h0 = steady_clock::now();
    while (steady_clock::now() - h0 < seconds(holdSeconds)) {
        if (_gotDeauth) return Result::NOGO_Deauthed;
        std::this_thread::sleep_for(milliseconds(100));
    }
    return Result::GO_LinkHeld;
}


// ---- beacon scan: discover BSSID + channel + RSN (security/discovery parity) -
void StationMode::onScanFrame(const uint8_t* frame, size_t len) {
    // RX thread. _scanSsid/_scanResult are also touched by scanForSsid on the scan
    // thread — snapshot the SSID under the lock (a torn std::string read during a
    // concurrent reassign crashes parseBeacon's compare), parse off-lock, then
    // update the result under the lock.
    try {
        std::string ssid;
        { std::lock_guard<std::mutex> lk(_scanMtx); ssid = _scanSsid; }
        ApInfo info;
        if (ScanProbe::parseBeacon(frame, len, ssid, info)) {
            std::lock_guard<std::mutex> lk(_scanMtx);
            if (!_scanResult.found || info.rssi > _scanResult.rssi) _scanResult = info;
            if (info.rssi > -127) _lastBeaconRssi.store(info.rssi);   // stable across per-channel resets (for arm DIG)
        }
    } catch (...) { /* garbled frame at high rate */ }
}

ApInfo StationMode::scanForSsid(const char* ssid, int channelHint, int perChannelMs) {
    using namespace std::chrono;
    // Restore MONITOR-mode DIG bounds (sensitive) for the scan. arm() switches DIG to connected
    // bounds (desensitized) for the strong-signal auth via SetLinkRssi, and on a FAILED arm the
    // reconnect loop comes straight back here with _s_linked still TRUE — leaving the scan
    // desensitized so it can't hear a weaker/normal-range AP -> FAIL_NO_AP loop. Reset each scan.
    PhydmWatchdog::SetUnlinked();
    { std::lock_guard<std::mutex> lk(_scanMtx); _scanSsid = ssid; _scanResult = ApInfo{}; }
    // Order = likelihood for a LEGAL APFPV link first, then catch-all:
    //   1) hint (the configured/last APFPV channel)
    //   2) 5.2 GHz UNII-1 (36/40/44/48) — the legal DE 200 mW @ 20 MHz APFPV band
    //      (PSD cap 10 mW/MHz x 20 MHz = 200 mW). A compliant VTX lives here.
    //   3) the rest of 5 GHz (UNII-2A/2C DFS, UNII-3 149-165) + common 2.4 GHz,
    //      so we can still find a test hotspot/router parked off the legal band.
    int channels[] = { channelHint,
                       36, 40, 44, 48,                 // UNII-1 — legal DE 200 mW APFPV
                       1, 6, 11,                       // 2.4 GHz common (test hotspots)
                       52, 56, 60, 64,                 // UNII-2A (DFS)
                       149, 153, 157, 161, 165 };      // UNII-3
    for (int ch : channels) {
        if (ch <= 0) continue;
        { std::lock_guard<std::mutex> lk(_scanMtx); _scanResult = ApInfo{}; }   // reset per channel — don't carry a bleed match over
        _rm.set_channel_bwmode((uint8_t)ch, 0, CHANNEL_WIDTH_20);  // tune radio (devourer API)
        auto t0 = steady_clock::now();
        while (steady_clock::now() - t0 < milliseconds(perChannelMs)) {
            // Accept only a CLEAN on-channel reception: the beacon's DS-param
            // channel must equal the tuned channel. This rejects 2.4GHz adjacent-
            // channel bleed (hearing a ch6 AP while tuned to ch1) that otherwise
            // returns the WRONG channel -> we arm off-channel -> auth never ACKs.
            {
                std::lock_guard<std::mutex> lk(_scanMtx);
                if (_scanResult.found && (_scanResult.channel == ch || _scanResult.channel == 0)) {
                    _scanResult.channel = ch; return _scanResult;
                }
            }
            std::this_thread::sleep_for(milliseconds(20));
        }
    }
    { std::lock_guard<std::mutex> lk(_scanMtx); return _scanResult; }   // .found == false if not seen
}

} // namespace apfpv