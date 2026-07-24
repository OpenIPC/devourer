// ============================================================================
//  RxDeframe.cpp — 802.11 data -> UDP/RTP -> port 5600 + RSSI tap (impl)
//  Declarations in RxDeframe.h. Wire onPacket() as devourer's RX callback.
// ============================================================================
#include "RxDeframe.h"
#include "ApfpvStation.h"
#include "PhydmWatchdog.h"
#include <cstring>
#include <cstdio>
#include <chrono>
#include <cstdlib>
#include <android/log.h>
#if defined(__ANDROID__)
#include <sys/system_properties.h>
#endif

// --- throughput bottleneck diag: logs timing breakdown every 500 packets ---
#define RX_DIAG_INTERVAL 500
namespace apfpv {

// HW-decrypt gate: env (host) or `debug.pixelpilot.hwdec=1` (Android prop). Must match the
// ApfpvStation gate — HW-decrypt arms the compressed-BA (rig: loss 163→21) but needs RCR_APP framing.
static bool rxHwDecryptOn() {
    if (std::getenv("DEVOURER_HW_DECRYPT") != nullptr) return true;
#if defined(__ANDROID__)
    char v[PROP_VALUE_MAX] = {0};
    if (__system_property_get("debug.pixelpilot.hwdec", v) > 0 && v[0] != '0') return true;
#endif
    return false;
}

static bool reorderEnabledInit() {
    // DEFAULT ON: deliver RTP strictly in-order to :5600 like the kernel does for phone-wifi.
    // The dongle's USB/A-MPDU delivery is bursty/out-of-order; without this the app's parser sees
    // gaps -> drops reference NALUs -> long-GOP decoder freeze (the dongle-only stutter). Opt out
    // with DEVOURER_REORDER=0 or debug.pixelpilot.reorder=0.
    if (const char* e = std::getenv("DEVOURER_REORDER")) return e[0] != '0';
#if defined(__ANDROID__)
    char v[PROP_VALUE_MAX] = {0};
    if (__system_property_get("debug.pixelpilot.reorder", v) > 0) return v[0] != '0';
#endif
    return true;
}

RxDeframe::RxDeframe(const Mac& self, const Mac& bssid, Wpa2Supplicant* wpa,
                     LqFeedback* lq, OnRtpFn onRtp)
    : _self(self), _bssid(bssid), _wpa(wpa), _lq(lq), _onRtp(std::move(onRtp)) {
    _reorderOn = reorderEnabledInit();
}

int RxDeframe::toDbm(uint8_t r) {
    // RxAtrib.rssi[] is devourer's gain_trsw byte for the path: the RX
    // descriptor's {TRSW(bit7), gain[6:0]} field (FrameParser.cpp fills it from
    // driver_data.gain_trsw). It is a GAIN index, not dBm — so the placeholder
    // r/2-95 was wrong. The RTL8812 (Jaguar) phystatus path in the Realtek HAL
    // converts the OFDM RX gain to power as:  rx_pwr_dBm = gain_index - 110
    // (the phydm rx_pwr_all base; LNA/VGA gain index maps ~1:1 to dB, offset by
    // the front-end reference of -110 dBm at gain 0). We mask the TRSW bit and
    // apply that. Clamp to a sane RF window.
    int gain = r & 0x7F;                 // strip TRSW flag, keep gain[6:0]
    int dbm  = gain - 110;               // Jaguar OFDM gain-index -> dBm
    if (dbm < -100) dbm = -100;
    if (dbm > -10)  dbm = -10;
    return dbm;
}

void RxDeframe::onPacket(const Packet& pkt) {
    // Layer diagnostic: count frames entering RxDeframe + forwarded to RTP
    static uint64_t inFrames=0, fwdFrames=0, decryptOk=0, fwdBytes=0;
    static auto layerClock = std::chrono::steady_clock::now();
    inFrames++;
    // BURST-LOSS HUNT (2026-07-15): is the periodic ~150-pkt loss a dongle RX BLACKOUT (a >15ms gap
    // with NO frames = our-side retune/stall/off-channel) or on-air/AP/chip loss (frames keep
    // flowing, only a seq hole)? Log any inter-frame gap >12ms with the elapsed ms + how many frames
    // we'd expect to have lost. If these gaps recur every ~8s = the smoking gun (RX went blind).
    {
        static auto lastPkt = std::chrono::steady_clock::now();
        static int  gapN = 0;
        auto nowp = std::chrono::steady_clock::now();
        long gapMs = (long)std::chrono::duration_cast<std::chrono::milliseconds>(nowp - lastPkt).count();
        lastPkt = nowp;
        if (gapMs > 12)
            __android_log_print(ANDROID_LOG_WARN, "rx-gap", "#%d RX gap %ldms (>12ms = RX blind this long)", ++gapN, gapMs);
    }
    if (_station) _station->notifyRxAlive();   // any RX = link alive (supervisor)
    if (_lq) _lq->update(toDbm(pkt.RxAtrib.rssi[0]), toDbm(pkt.RxAtrib.rssi[1]));
    // Feed live RX RSSI to DIG so it uses phydm *connected-mode* boundaries
    // (IGI floor tracks RSSI ~0x37 @ -45 dBm) instead of monitor coverage
    // bounds (capped 0x2a → over-gained → FA storm → AP rate-caps us).
    PhydmWatchdog::SetLinkRssi(toDbm(pkt.RxAtrib.rssi[0]));

    const uint8_t* f = pkt.Data.data();
    size_t len = pkt.Data.size();
    if (len < 24) return;

    uint16_t fc = f[0] | (f[1] << 8);
    // mgmt deauth/disassoc from our AP -> immediate link-loss signal
    if (((fc >> 2) & 0x3) == 0x0) {
        uint8_t sub = (fc >> 4) & 0xF;
        if (sub == 0xC || sub == 0xA) {
            // ONLY honor a deauth/disassoc that is addressed to US (a1==self) AND sent by
            // OUR AP (a2==bssid). Without this, in busy RF we overhear OTHER APs deauthing
            // THEIR clients and falsely trip link-loss -> a Reconnecting churn every few
            // seconds -> periodic video stutter. (Same a1/a2 filter the join path uses.)
            bool toUs   = (std::memcmp(f + 4,  _self.data(),  6) == 0);
            bool fromAp = (std::memcmp(f + 10, _bssid.data(), 6) == 0);
            if (_station && toUs && fromAp) { // deauth/disassoc from AP -> immediate link-loss
                if (!_wpa || !_wpa->pmfActive() || _wpa->verifyProtectedMgmt(f, len))
                    _station->notifyDeauth();
            }
            return;
        }
        // 802.11 Action frame (subtype 0xD): BlockAck frames
        if (sub == 0xD && len >= 27) {
            const uint8_t* b = f + 24;
            // Full hex dump of ADDBA Request (cat=3 act=0) to compare buffer size/policy
            if (b[0] == 0x03 && b[1] == 0x00 && len >= 24 + 9) {
                u16 p = b[3] | (b[4] << 8);
                __android_log_print(4, "apfpv-scan",
                    "ADDBA-REQ-IN dialog=%u param=0x%04x policy=%u tid=%u bufsz=%u timeout=%u startSeq=%u",
                    b[2], p, (p>>1)&1, (p>>2)&0x0f, (p>>6)&0x3ff,
                    b[5]|(b[6]<<8), (b[7]|(b[8]<<8))>>4);
            } else if (b[0] == 0x03 && b[1] == 0x02 && len >= 24 + 6) {
                // DELBA: param[2-3] = {initiator bit11, TID bits12-15}, reason[4-5].
                u16 dp = b[2] | (b[3] << 8);
                __android_log_print(4, "apfpv-scan", "DELBA-IN tid=%u initiator=%u reason=%u",
                    (dp >> 12) & 0xf, (dp >> 11) & 1, (unsigned)(b[4] | (b[5] << 8)));
            } else {
                __android_log_print(4, "apfpv-scan", "ACTION-RX cat=%u act=%u", b[0], b[1]);
            }
        }
        if (sub == 0xD && _station && len >= 24 + 3) {
            const uint8_t* body = f + 24; // 3-addr mgmt header
            if (body[0] == 0x03) { // BlockAck category
                if (body[1] == 0x00) { // ADDBA Request from AP -> send Response
                    _station->handleAddbaRequest(f, len);
                } else if (body[1] == 0x01) { // ADDBA Response from AP -> BA session established!
                    // AP accepted our ADDBA Request. Enable reorder for this TID.
                    u16 param = body[5] | (body[6] << 8);  // BA Parameter Set
                    u8  tid   = (param >> 2) & 0x0f;
                    u16 status = body[3] | (body[4] << 8);
                    if (status == 0 && tid < 16) {
                        _reorder[tid].enable = true;
                        _reorder[tid].wsize_b = 64;
                        _reorder[tid].indicate_seq = 0xffff;  // accept any start seq
                        _reorder[tid].pending.clear();
                        __android_log_print(4, "apfpv-scan",
                            "ADDBA Response ACCEPTED tid=%u — A-MPDU enabled!", tid);
                    }
                }
            }
        }
        return;
    }
    // CONTROL frames (type 1): BlockAckReq (subtype 8) — kernel rtw_process_bar_frame. The AP sends
    // a BAR to move the receiver's reorder window past a gap; process it so the SW reorder flushes
    // instead of stalling. Frame: FC|Dur|RA(4..9)|TA(10..15)|BAR_ctrl(16..17)|BAR_seq(18..19).
    if (((fc >> 2) & 0x3) == 0x1) {
        if (((fc >> 4) & 0xF) == 0x8 && len >= 20 && _reorderOn &&
            std::memcmp(f + 4, _self.data(), 6) == 0) {
            uint16_t barctl = f[16] | (f[17] << 8);
            uint16_t barseq = f[18] | (f[19] << 8);
            processBar((uint8_t)((barctl >> 12) & 0xf), (uint16_t)(barseq >> 4));
        }
        return;
    }
    if (((fc >> 2) & 0x3) != 0x2) return;          // data frames only
    if (!(fc & 0x0200)) return;                    // from-DS (AP->STA)
    // fprintf removed: blocks RX thread at 65Mbps — use rxd-diag instead
    // Accept unicast-to-us OR group-addressed (I/G bit in A1[0]): DHCP OFFER/ACK + ARP are
    // broadcast and FPV video may be multicast. Group frames decrypt with the GTK below.
    if (!(f[4] & 0x01) && std::memcmp(f + 4, _self.data(), 6) != 0) return;

    // Count data frames arriving, log every 500th
    static thread_local uint32_t dataFrames = 0;
    static thread_local bool firstData = true;
    if (firstData) {
        firstData = false;
#if defined(__ANDROID__)
        __android_log_print(ANDROID_LOG_INFO, "apfpv-scan",
            "1st data frame: fc=0x%04x len=%zu prot=%d a1=%02x:%02x:%02x:%02x:%02x:%02x",
            fc, len, (fc&0x4000)?1:0, f[4],f[5],f[6],f[7],f[8],f[9]);
#endif
    }
    if ((++dataFrames % 500) == 0) {
#if defined(__ANDROID__)
        __android_log_print(ANDROID_LOG_INFO, "apfpv-scan",
            "data frames: %u total, decryptOk=%u decFail=%u",
            dataFrames, decryptOk, _dbgDecFail);
#endif
    }

    size_t hdrLen = 24;
    if ((fc & 0x0300) == 0x0300) hdrLen += 6;      // 4-addr
    if (fc & 0x0080) hdrLen += 2;                  // QoS-data (subtype>=8): 2B QoS Control
                                                   // (was 0x8000 = Order flag — wrong; EAPOL
                                                   // M1 is QoS-data fc=0x0288, so this was the
                                                   // header-offset bug that hid the handshake)
    if (len <= hdrLen) return;
    // A-MSDU Present = bit 7 of the QoS Control field (the last 2B of the header, present only
    // for QoS data). When set, the (decrypted) MSDU body is a chain of [DA|SA|len|sub-MSDU]
    // subframes, NOT a bare LLC/SNAP. The greg VTX uses this WITH rtw_ampdu_enable=0 to
    // aggregate several packets into one normally-ACKed MPDU — throughput gain without BlockAck.
    const bool amsdu = (fc & 0x0080) && (f[hdrLen - 2] & 0x80);
    // 802.11 sequence-control (seq is bits 4-15 of the 2B field at offset 22) + QoS TID — for the
    // A-MPDU reorder (kernel recv_indicatepkt_reorder). Only meaningful for QoS data.
    const uint16_t mpduSeq = (uint16_t)(((f[22] | (f[23] << 8)) >> 4) & 0xfff);
    const uint8_t  qtid    = (fc & 0x0080) ? (uint8_t)(f[hdrLen - 2] & 0x0f) : 0;

    const uint8_t* body = f + hdrLen; size_t bodyLen = len - hdrLen;
    // Pre-allocated buffer per thread — avoids heap alloc per packet (5600/s at 65Mbps)
    static thread_local std::vector<uint8_t> plainBuf(2048);
    const uint8_t* llc; size_t llcLen;
    if (fc & 0x4000) {                             // Protected
        // Lever C.2: if HW CCMP decrypt is on AND the chip already decrypted this frame
        // (descriptor SWDEC=0 -> bdecrypted), skip the SW AES entirely — the body is
        // [CCMP hdr 8][plaintext LLC+payload][MIC 8]. This is the lean kernel-style path.
        // GATED: env cached once; default OFF so the proven SW path stays active.
        static const bool kHwDecrypt = rxHwDecryptOn();  // env or debug.pixelpilot.hwdec — arms HW reorder + compressed-BA; needs RCR_APP framing.
        // DIAG: count HW-decrypted vs SW-fallback frames so we can SEE whether the chip is
        // actually HW-decrypting (bdecrypted=1) after the SECCFG=0x010c fix. Logged every 4000.
        static thread_local uint32_t hwDec = 0, swDec = 0, diagN = 0, grp = 0, uni = 0;
        if (kHwDecrypt) {
            if (pkt.RxAtrib.bdecrypted) hwDec++; else swDec++;
            if (f[4] & 0x01) grp++; else uni++;   // A1 group-addressed vs unicast-to-us
            if ((++diagN % 4000) == 0)
                fprintf(stderr, "[rxd-hwdec] bdecrypted(HW)=%u SW-fallback=%u | group=%u unicast=%u (last4000)\n",
                        hwDec, swDec, grp, uni), hwDec = swDec = grp = uni = 0;
        }
        if (kHwDecrypt && pkt.RxAtrib.bdecrypted) {
            if (bodyLen <= 16) return;             // need CCMP hdr(8) + MIC(8)
            llc = body + 8; llcLen = bodyLen - 16; // strip CCMP header + trailing MIC
            decryptOk++;
        } else {
            if (!_wpa || !_wpa->ready()) return;
            if (!_wpa->decryptData(f, len, plainBuf)) {
                _dbgDecFail++;
#if defined(__ANDROID__)
                if ((_dbgDecFail % 200) == 1)
                    __android_log_print(ANDROID_LOG_WARN, "apfpv-scan",
                        "CCMP decFail count=%u a1=%02x:%02x fc=0x%04x len=%zu",
                        _dbgDecFail, f[4],f[5], fc, len);
#endif
                return;
            }
            decryptOk++;
            llc = plainBuf.data(); llcLen = plainBuf.size();
        }
    } else { llc = body; llcLen = bodyLen; }

    // Deliver ONE MSDU (LLC/SNAP + payload) up the stack. Called once for a normal frame, or
    // once per subframe for an A-MSDU. Early `return` = skip THIS MSDU (move to the next).
    auto deliverMsdu = [&](const uint8_t* llc, size_t llcLen) {
    if (llcLen < 8) return;
    static const uint8_t snap[6] = {0xaa,0xaa,0x03,0x00,0x00,0x00};
    if (std::memcmp(llc, snap, 6) != 0) return;
    uint16_t ethertype = (llc[6] << 8) | llc[7];
    // EAPOL (0x888E): the WPA2 4-way handshake. MUST route to the supplicant or
    // the handshake never completes and the link stalls at Handshaking.
    if (ethertype == 0x888E) {
#if defined(__ANDROID__)
        __android_log_print(ANDROID_LOG_INFO, "apfpv-scan",
            "[rx] EAPOL frame -> supplicant (llcLen=%zu wpa=%d)", llcLen, _wpa?1:0);
#endif
        if (_wpa) _wpa->onEapolKey(llc + 8, llcLen - 8);
        return;
    }
    // A-MPDU reorder disabled — AP sends single frames (Block-Ack HW not working).
    // The reorder buffer (processReorder) is ready; enable when HW BA works.
    #if 0
    if ((fc & 0x0088) == 0x0088 && ethertype == 0x0800 && llcLen >= 28) {
        const uint8_t* ip = llc + 8; size_t ipl = llcLen - 8;
        if (ipl >= 20 && (ip[0] >> 4) == 4 && ip[9] == 17) {
            size_t ihl = (ip[0] & 0x0f) * 4;
            if (ipl >= ihl + 8) {
                const uint8_t* u = ip + ihl;
                if (((u[2] << 8) | u[3]) == 5600) {
                    processReorder((hdrLen>=26)?(f[24]&0xf):0,
                        (uint16_t)((f[23]<<4)|(f[22]>>4)), llc, llcLen);
                    return;
                }
            }
        }
    }
    #endif

    // ARP responder: reply to "who has <ourIp>?" so peers keep a fresh entry for us and the
    // unicast RTP/SSH stream doesn't stall when their STALE entry needs re-validation.
    if (ethertype == 0x0806 && _ourIp && _arpSend && _wpa && llcLen >= 8 + 28) {
        const uint8_t* a = llc + 8;                          // ARP payload
        uint32_t tip = (a[24]<<24)|(a[25]<<16)|(a[26]<<8)|a[27];
        if (a[6]==0x00 && a[7]==0x01 && tip == _ourIp) {     // a request for OUR IP
            uint8_t r[28]; std::memcpy(r, a, 28);
            r[6]=0x00; r[7]=0x02;                            // oper = reply
            std::memcpy(r+8,  _self.data(), 6);              // sender HW = us
            std::memcpy(r+14, a+24, 4);                      // sender IP = our IP
            std::memcpy(r+18, a+8, 6);                       // target HW = requester
            std::memcpy(r+24, a+14, 4);                      // target IP = requester
            auto m = _wpa->buildEncryptedData(r, 28, 0x0806);
            if (!m.empty()) _arpSend(m);
            static uint32_t s_arpN = 0;
            if ((++s_arpN % 4) == 1)
                __android_log_print(4, "rxd-arp", "answered ARP who-has %u.%u.%u.%u (n=%u)",
                    (_ourIp>>24)&255,(_ourIp>>16)&255,(_ourIp>>8)&255,_ourIp&255, s_arpN);
        }
        return;
    }
    if (ethertype != 0x0800) return; // IPv4
    const uint8_t* ip = llc + 8; size_t ipLen = llcLen - 8;
    // DIAG (rxd-ip): log NON-RTP IPv4 arriving from the VTX — the SSH/mavlink RETURN path. If
    // SSH/mavlink time out but these appear, the VTX IS replying (issue is app-side/routing); if
    // they never appear, the VTX isn't unicasting back to our (static) IP (ARP/route gap).
    if (ipLen >= 20 && (ip[0] >> 4) == 4) {
        size_t dih = (ip[0] & 0x0f) * 4;
        uint16_t ddp = (ipLen >= dih + 4) ? ((ip[dih+2] << 8) | ip[dih+3]) : 0;
        if (!(ip[9] == 17 && ddp == 5600)) {   // anything but the RTP video
            static uint32_t s_nrN = 0;
            uint32_t sip = (ip[12]<<24)|(ip[13]<<16)|(ip[14]<<8)|ip[15];
            uint16_t dsp = (ipLen >= dih + 4) ? ((ip[dih] << 8) | ip[dih+1]) : 0;
            if ((++s_nrN % 4) == 1)
                __android_log_print(4, "rxd-ip", "RX non-RTP IPv4 proto=%u src=%u.%u.%u.%u %u->%u len=%zu",
                    ip[9], (sip>>24)&255,(sip>>16)&255,(sip>>8)&255,sip&255, dsp, ddp, ipLen);
        }
    }
    // De-dup 802.11 retransmissions of RTP BEFORE any sink: as a station we RX unicast RTP
    // and the AP retransmits each frame until ACKed (~6x). This must run ahead of the _onIp
    // (TUN) path below — otherwise the duplicates reach the decoder via the OS route. Retries
    // reuse the RTP seq, so drop a UDP/5600 packet whose (payload-type, seq) repeats the last.
    if (ipLen >= 20 && (ip[0] >> 4) == 4 && ip[9] == 17) {
        size_t ihl0 = (ip[0] & 0x0f) * 4;
        if (ipLen >= ihl0 + 8 + 4) {
            const uint8_t* u = ip + ihl0;
            if (((u[2] << 8) | u[3]) == 5600) {
                const uint8_t* r = u + 8;
                uint8_t  pt = r[1] & 0x7f;
                uint16_t sq = (uint16_t)((r[2] << 8) | r[3]);
                _dbgRx++;
                // NOTE: the RTP-seq de-dup loop (128-elem scan/pkt) was REMOVED — it was dead
                // code (dropDup=0 every run). The CCMP PN-replay window already drops every
                // 802.11 retransmit (same PN) at decrypt time, before this point. One less
                // 716k-iter/s linear scan in the hot path.
                // RX-seq-gap loss + OUT-OF-ORDER detection. Signed 16-bit delta vs the last seq:
                //   d > 0  : forward — d-1 packets missing (real loss so far, may fill in later)
                //   d < 0  : this packet is BEHIND the last = arrived OUT OF ORDER (A-MPDU reorder)
                //   d == 0 : duplicate (shouldn't reach here; PN-replay drops it earlier)
                // Hypothesis under test: the dongle has NO A-MPDU RX reorder (RxDeframe:174 disabled),
                // so aggregated frames reach the H265 depacketizer scrambled → NALU reassembly corrupts
                // → decoder freeze while bitrate stays steady. Phone-Wi-Fi's HW MAC reorders → smooth.
                // If _dbgOoo climbs, that's confirmed and the reorder buffer is the fix.
                if (_lastSeqV[pt]) {
                    int16_t d = (int16_t)(sq - _lastSeq[pt]);
                    if (d > 1 && d < 2000) _dbgLoss += (d - 1);
                    else if (d < 0)        _dbgOoo++;          // arrived behind the last = reordered
                    // Only advance the high-water seq on forward progress so OOO packets don't
                    // rewind it (which would make the next in-order packet look like a huge gap).
                    if (d > 0) { _lastSeq[pt] = sq; }
                } else { _lastSeq[pt] = sq; _lastSeqV[pt] = true; }
                _lastSeqV[pt] = true;
                // Health summary every 120 unique pkts: received / dup-dropped / decrypt-failed / lost / out-of-order.
                if ((_dbgRx % 120) == 0)
                    __android_log_print(ANDROID_LOG_INFO, "rxd-health",
                        "rx=%d dropDup=%d decFail=%d lost=%d ooo=%d amsdu=%d sub=%d (pt=%u seq=%u) rxrate=%u bw=%u",
                        _dbgRx, _dbgDrop, _dbgDecFail, _dbgLoss, _dbgOoo, _dbgAmsdu, _dbgAmsduSub, pt, sq,
                        (unsigned)pkt.RxAtrib.data_rate, (unsigned)pkt.RxAtrib.bw);
                        // rxrate: 0-3=CCK 1/2/5.5/11, 4-11=OFDM 6..54, 12-27=HT MCS0-15,
                        // 28+=VHT. Low (<12) => rate-control collapsed (ACK problem);
                        // high MCS but sparse rx => A-MPDU/Block-Ack aggregation missing.
            }
        }
    }
    // GENERAL-IP BRIDGE: hand the IPv4 packet to the TUN sink so SSH / mavlink / any TCP+UDP
    // traverses the dongle (a full L3 interface). ⚠️ EXCLUDE the RTP video (UDP/5600): it already
    // reaches the decoder via _onRtp below. Bridging it to the TUN too makes the OS RE-DELIVER it
    // to the local 5600 socket → EVERY video frame arrives TWICE. That was the "app OSD shows ~2x
    // the VTX's actual bitrate (82 vs 43 Mbps)" bug, and the duplicate/out-of-order frames stutter
    // the decoder. Phone-Wi-Fi has no TUN bridge, hence it was smooth. Skip 5600 here.
    {
        size_t ihlB = (ip[0] & 0x0f) * 4;
        bool rtp5600 = (ip[9] == 17) && (ipLen >= ihlB + 4) && (((ip[ihlB + 2] << 8) | ip[ihlB + 3]) == 5600);
        if (_onIp && !rtp5600) _onIp(ip, ipLen);
    }
    if (ipLen < 20 || (ip[0] >> 4) != 4 || ip[9] != 17) return; // IPv4/UDP
    size_t ihl = (ip[0] & 0x0f) * 4;
    if (ipLen < ihl + 8) return;
    const uint8_t* udp = ip + ihl;
    uint16_t dport = (udp[2] << 8) | udp[3];
    uint16_t udlen = (udp[4] << 8) | udp[5];
    // DHCP: replies (OFFER/ACK) arrive on UDP port 68 (BOOTP client). Forward
    // the UDP payload (BOOTP message) to the DHCP state machine so DORA can
    // complete. Without this, the client never sees OFFER and DHCP stalls.
    if (dport == 68) {
        if (_onDhcp && udlen >= 8 && (size_t)udlen <= ipLen - ihl)
            _onDhcp(udp + 8, udlen - 8);
        return;
    }
    if (dport != 5600) return;   // video port
    uint16_t ulen = udlen;
    if (ulen < 8 || (size_t)ulen > ipLen - ihl) return;
    const uint8_t* rtp = udp + 8; size_t rtpLen = ulen - 8;
    if (rtpLen && _onRtp) {
        // RTP-SEQUENCE REORDER: deliver the :5600 payload strictly in RTP-seq order so the
        // depacketizer (not reorder-tolerant) stops seeing scrambled/complete-but-gapped frames.
        // Keyed on the RTP seq (not the 802.11 MPDU seq — that was wrong for this VTX and buffered
        // everything). Gated _reorderOn.
        if (_reorderOn) {
            uint16_t rtpSeq = (uint16_t)((rtp[2] << 8) | rtp[3]);
            processReorderRtp(rtpSeq, rtp, rtpLen);
        } else {
            _onRtp(rtp, rtpLen);
        }
        fwdFrames++; fwdBytes += rtpLen;
    }
    // Layer health every 1 second
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - layerClock).count();
    if (elapsed >= 1000) {
      float secs = (float)elapsed / 1000.0f;
      __android_log_print(4, "rx-layer",
        "IN=%d/s FWD=%d/s (%.1f%%) decOK=%d/s | GOODPUT=%.1f Mbps",
        (int)(inFrames/secs), (int)(fwdFrames/secs),
        (float)fwdFrames*100.f/(float)(inFrames+1),
        (int)(decryptOk/secs),
        (float)fwdBytes * 8.0f / (secs * 1e6f));
      inFrames=fwdFrames=decryptOk=0; fwdBytes=0; layerClock = now;
    }
    // (removed: per-frame t0..t3 timing + rxd-diag breakdown — 4 steady_clock::now()/frame of
    //  pure profiling overhead; the 1s rx-layer GOODPUT line above is enough.)
    };  // end deliverMsdu

    if (amsdu) {
        // Walk A-MSDU subframes: [DA(6)|SA(6)|len(2)|MSDU(len bytes)|pad to 4B] (no pad on last).
        // Each MSDU is its own LLC/SNAP packet — the AP coalesced several into one ACKed MPDU.
        _dbgAmsdu++;
        size_t off = 0;
        while (off + 14 <= llcLen) {
            uint16_t sub = (uint16_t)((llc[off + 12] << 8) | llc[off + 13]);
            if (sub < 8 || off + 14 + sub > llcLen) break;     // truncated/garbage -> stop
            _dbgAmsduSub++;
            deliverMsdu(llc + off + 14, sub);                  // MSDU starts with its LLC/SNAP
            off = (off + 14 + sub + 3) & ~size_t(3);           // next subframe is 4-byte aligned
        }
    } else {
        deliverMsdu(llc, llcLen);                              // normal single-MSDU frame
    }
}

// Extract UDP/5600 RTP payload from a decrypted MSDU (SNAP + IPv4 + UDP) and emit it.
void RxDeframe::emitReorderRtp(const uint8_t* llc, size_t llcLen) {
    if (llcLen < 8 || !_onRtp) return;
    const uint8_t* ip = llc + 8; size_t ipl = llcLen - 8;   // skip 8B SNAP
    if (ipl < 20 || (ip[0] >> 4) != 4 || ip[9] != 17) return;
    size_t ihl = (ip[0] & 0x0f) * 4;
    if (ipl < ihl + 8) return;
    const uint8_t* u = ip + ihl;
    if (((u[2] << 8) | u[3]) != 5600) return;
    uint16_t udpLen = (u[4] << 8) | u[5];
    if (udpLen < 8) return;
    size_t rtpLen = (size_t)udpLen - 8;             // UDP len includes the 8B UDP header
    if (rtpLen == 0 || rtpLen > ipl - ihl - 8) return;
    _onRtp(u + 8, rtpLen);
}

// Reorder release-timer: how long to hold a gap waiting for a late/retransmitted seq before
// skipping it. Too short = skip a recoverable retransmit (residual loss); too long = latency.
// Env DEVOURER_REORDER_MS to sweep toward kernel parity.
static int64_t reorderTimeoutMs() {
    static int64_t v = [] {
        if (const char* e = std::getenv("DEVOURER_REORDER_MS")) { int n = std::atoi(e); if (n > 0) return (int64_t)n; }
#if defined(__ANDROID__)
        // Live-tunable without a rebuild (adb shell setprop debug.pixelpilot.reorderms N). Added
        // to test whether real compressed-BA-style retransmits under accept-BA (DEVOURER_ENABLE_BA
        // / debug.pixelpilot.ba=1) need longer than the kernel's 50ms REORDER_WAIT_TIME to arrive —
        // under real A-MPDU aggregation the AP has more airtime contention per retransmit cycle
        // than the decline-BA/no-aggregation case this constant was originally tuned against.
        char v2[PROP_VALUE_MAX] = {0};
        if (__system_property_get("debug.pixelpilot.reorderms", v2) > 0) {
            int n = std::atoi(v2);
            if (n > 0) return (int64_t)n;
        }
#endif
        return (int64_t)50;   // kernel REORDER_WAIT_TIME (include/rtw_recv.h); bench 60≈50 best
    }();
    return v;
}
#define kReorderTimeoutMs reorderTimeoutMs()
static int64_t nowMsSteady() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

// RTP-SEQUENCE reorder for the :5600 video flow. Buffers the RTP payload keyed by its 16-bit
// (mod 65536) sequence number and emits strictly in order via _onRtp. Mirrors the proven
// wrap-safe drain of processReorder but in the RTP-seq domain the depacketizer actually needs.
// A release timer (kReorderTimeoutMs) force-flushes a permanently-missing seq so a lost packet
// skips forward instead of stalling the drain forever (and leaking the pending map).
bool RxDeframe::processReorderRtp(uint16_t rtpSeq, const uint8_t* rtp, size_t rtpLen) {
    auto& rc = _reorderRtp;
    if (rtpLen == 0 || rtpLen > 1500) return false;
    int64_t now = nowMsSteady();
    if (!rc.enable) {
        rc.enable = true; rc.indicate = rtpSeq; rc.pending.clear();
        rc.lastFlushMs = now; rc.wsize = 256;
    }
    // signed 16-bit distance in [-32768, 32767]
    auto sdiff = [](uint16_t a, uint16_t b) { int d = ((int)a - (int)b) & 0xffff; return d >= 32768 ? d - 65536 : d; };
    auto drain = [&]() {
        for (auto it = rc.pending.find(rc.indicate); it != rc.pending.end();
             it = rc.pending.find(rc.indicate)) {
            _onRtp(it->second.data(), it->second.size());
            rc.pending.erase(it);
            rc.indicate = (uint16_t)(rc.indicate + 1);
            rc.lastFlushMs = now;
        }
    };
    auto lowestPending = [&]() -> uint16_t {
        uint16_t best = rc.indicate; int bestd = 0x7fffffff;
        for (auto& kv : rc.pending) { int dd = sdiff(kv.first, rc.indicate);
            if (dd >= 0 && dd < bestd) { bestd = dd; best = kv.first; } }
        return best;
    };
    static long cIn = 0, cBuf = 0, cDrain = 0, cDropOld = 0, cFarJump = 0, cFarSkip = 0, cToFlush = 0, cToSkip = 0, cLog = 0;
    size_t pendBefore = rc.pending.size();

    int d = sdiff(rtpSeq, rc.indicate);
    if (d < 0) { cDropOld++; return false; }                       // already passed (dup/retransmit)
    if (d >= rc.wsize) {                                          // far ahead: shift window, flush below
        uint16_t newInd = (uint16_t)((rtpSeq - rc.wsize + 1) & 0xffff);
        cFarJump++; cFarSkip += (long)sdiff(newInd, rc.indicate);
        while (rc.indicate != newInd) {
            auto it = rc.pending.find(rc.indicate);
            if (it != rc.pending.end()) { _onRtp(it->second.data(), it->second.size()); rc.pending.erase(it); }
            rc.indicate = (uint16_t)(rc.indicate + 1);
        }
        rc.lastFlushMs = now; drain(); d = sdiff(rtpSeq, rc.indicate);
    }
    if (d == 0) {                                                 // in order
        cIn++; _onRtp(rtp, rtpLen); rc.indicate = (uint16_t)(rc.indicate + 1); rc.lastFlushMs = now;
        size_t pb = rc.pending.size(); drain(); cDrain += (long)(pb - rc.pending.size());
    } else {                                                      // ahead, in window — buffer
        if (rc.pending.count(rtpSeq) == 0) { rc.pending[rtpSeq] = std::vector<uint8_t>(rtp, rtp + rtpLen); cBuf++; }
    }
    // release timer: a gap at indicate_seq persisting past the timeout -> skip the lost seq
    if (!rc.pending.empty() && (now - rc.lastFlushMs) > kReorderTimeoutMs) {
        uint16_t lp = lowestPending();
        cToFlush++; cToSkip += (long)sdiff(lp, rc.indicate);
        rc.indicate = lp; rc.lastFlushMs = now; drain();
    }
    (void)pendBefore;
    if ((++cLog % 4000) == 0)
        __android_log_print(ANDROID_LOG_INFO, "rxd-reo",
            "RTP inOrder=%ld buffered=%zu drained(rec)=%ld dropOld=%ld | farJump=%ld farSkip=%ld | toFlush=%ld toSkip=%ld pend=%zu",
            cIn, rc.pending.size(), cDrain, cDropOld, cFarJump, cFarSkip, cToFlush, cToSkip, rc.pending.size());
    return true;
}

// A-MPDU per-TID reorder buffer — faithful port of the kernel's recv_indicatepkt_reorder +
// rtw_reordering_ctrl_timeout_handler (release timer). QoS-data A-MPDU subframes/retransmits
// arrive out-of-order; the depacketizer (ParseRTP) is NOT reorder-tolerant, so out-of-order RTP
// makes it declare complete frames "missing". This delivers strictly in-order within a 64-seq
// window, with a release-timer flush so a permanently-lost seq skips forward instead of stalling.
// 12-bit 802.11 sequence space (mod 4096). kReorderTimeoutMs matches the kernel's ~sortterm.

bool RxDeframe::processReorder(uint8_t tid, uint16_t seq, const uint8_t* llc, size_t llcLen) {
    if (tid >= 16) return false;
    auto& rc = _reorder[tid];
    uint16_t sq = seq & 0xfff;
    int64_t now = nowMsSteady();

    // Reorder window. Kernel uses the BA bufsz (64) because its DMA RX arrives ~in-order; our USB
    // URB delivery reorders frames in large chunks (measured ~37% buffered), so 64 overruns and
    // force-skips recoverable frames. A larger window absorbs the USB out-of-order spread.
    static int win = [] { const char* e = std::getenv("DEVOURER_REORDER_WIN"); int n = e ? std::atoi(e) : 0;
                          return (n >= 16 && n <= 2000) ? n : 64; }();
    if (!rc.enable) { rc.enable = true; rc.indicate_seq = sq; rc.pending.clear(); rc.lastFlushMs = now; rc.wsize_b = (uint16_t)win; }

    // signed 12-bit distance sq - indicate_seq in [-2048, 2047]
    auto sdiff = [](uint16_t a, uint16_t b) { int d = ((int)a - (int)b) & 0xfff; return d >= 2048 ? d - 4096 : d; };
    // Wrap-safe: deliver the EXACT next expected seq via find() — NOT begin() (numeric-min of the
    // std::map), which returns the wrong entry across the 12-bit seq wrap (~every 0.8s at 120fps).
    // The old begin()==indicate_seq test stalled the drain whenever a wrapped seq sorted below
    // indicate_seq, so pending piled up until the far-ahead branch fired and SKIPPED recoverable
    // frames (the farSkip that made this reorder net-harmful and got it disabled).
    auto drain = [&]() {
        for (auto it = rc.pending.find(rc.indicate_seq); it != rc.pending.end();
             it = rc.pending.find(rc.indicate_seq)) {
            emitReorderRtp(it->second.data(), it->second.size());
            rc.pending.erase(it);
            rc.indicate_seq = (rc.indicate_seq + 1) & 0xfff;
            rc.lastFlushMs = now;
        }
    };
    // Wrap-safe lowest buffered seq at/ahead of indicate_seq (min non-negative 12-bit distance).
    auto lowestPending = [&]() -> uint16_t {
        uint16_t best = rc.indicate_seq; int bestd = 0x7fffffff;
        for (auto& kv : rc.pending) { int dd = sdiff(kv.first, rc.indicate_seq);
            if (dd >= 0 && dd < bestd) { bestd = dd; best = kv.first; } }
        return best;
    };

    // DIAG counters to find WHERE the residual loss comes from.
    static long cIn=0, cBuf=0, cDrain=0, cDropOld=0, cFarJump=0, cFarSkip=0, cToFlush=0, cToSkip=0, cLog=0;
    size_t pendBefore = rc.pending.size();

    int d = sdiff(sq, rc.indicate_seq);
    if (d < 0) { cDropOld++; return false; }          // old / duplicate — already passed
    if (d >= (int)rc.wsize_b) {
        // Far ahead — kernel advances the window to (sq - wsize + 1), FLUSHING everything below it
        // strictly in order first (so nothing is emitted out-of-order), then the frame is in-window.
        uint16_t newInd = (uint16_t)((sq - rc.wsize_b + 1) & 0xfff);
        cFarJump++; cFarSkip += sdiff(newInd, rc.indicate_seq);   // seqs skipped by the window shift
        // Flush strictly in order from indicate_seq up to newInd (wrap-safe walk): deliver any
        // present, skip the truly-missing. begin()-based iteration was wrong across the wrap.
        while (rc.indicate_seq != newInd) {
            auto it = rc.pending.find(rc.indicate_seq);
            if (it != rc.pending.end()) { emitReorderRtp(it->second.data(), it->second.size()); rc.pending.erase(it); }
            rc.indicate_seq = (rc.indicate_seq + 1) & 0xfff;
        }
        rc.lastFlushMs = now;
        drain();                                     // deliver consecutive from newInd
        d = sdiff(sq, rc.indicate_seq);              // recompute; now in-window
    }
    if (d == 0) {                                    // in-order
        cIn++;
        emitReorderRtp(llc, llcLen);
        rc.indicate_seq = (rc.indicate_seq + 1) & 0xfff;
        rc.lastFlushMs = now;
        size_t pb = rc.pending.size(); drain(); cDrain += (long)(pb - rc.pending.size());
    } else {                                         // ahead, in window — buffer
        if (rc.pending.count(sq) == 0) { rc.pending[sq] = std::vector<uint8_t>(llc, llc + llcLen); cBuf++; }
    }

    // RELEASE TIMER: if a gap at indicate_seq persists past the timeout, skip the (lost) seq —
    // advance to the lowest buffered frame and drain. Prevents a permanent-loss stall/freeze.
    if (!rc.pending.empty() && (now - rc.lastFlushMs) > kReorderTimeoutMs) {
        uint16_t lp = lowestPending();               // wrap-safe true-next buffered (not numeric-min)
        cToFlush++; cToSkip += sdiff(lp, rc.indicate_seq);  // gap seqs skipped
        rc.indicate_seq = lp;
        rc.lastFlushMs = now;
        drain();
    }
    (void)pendBefore;
    if ((++cLog % 4000) == 0)
        __android_log_print(ANDROID_LOG_INFO, "rxd-reo",
            "inOrder=%ld buffered=%ld drained(recovered)=%ld dropOld(dup)=%ld | farJump=%ld farSkip=%ld | toFlush=%ld toSkip=%ld | pend=%zu",
            cIn, cBuf, cDrain, cDropOld, cFarJump, cFarSkip, cToFlush, cToSkip, rc.pending.size());
    return true;
}

// Kernel rtw_process_bar_frame: a BlockAckReq advances the reorder window to start_seq.
void RxDeframe::processBar(uint8_t tid, uint16_t start_seq) {
    if (tid >= 16) return;
    auto& rc = _reorder[tid];
    if (!rc.enable) return;
    uint16_t ss = start_seq & 0xfff;
    int64_t now = nowMsSteady();
    // deliver everything below start_seq (in order) then set the window to start_seq
    for (auto it = rc.pending.begin(); it != rc.pending.end(); ) {
        int d = ((int)it->first - (int)ss) & 0xfff; if (d >= 2048) d -= 4096;
        if (d < 0) { emitReorderRtp(it->second.data(), it->second.size()); it = rc.pending.erase(it); }
        else break;
    }
    rc.indicate_seq = ss;
    rc.lastFlushMs = now;
    while (!rc.pending.empty() && rc.pending.begin()->first == rc.indicate_seq) {
        auto it = rc.pending.begin();
        emitReorderRtp(it->second.data(), it->second.size());
        rc.indicate_seq = (rc.indicate_seq + 1) & 0xfff;
        rc.pending.erase(it);
    }
}

} // namespace apfpv
