#pragma once
#include <cstdint>
#include <array>
#include <map>
#include <functional>
#include "FrameParser.h"
#include "Wpa2Supplicant.h"
#include "LqFeedback.h"
namespace apfpv {
using Mac = std::array<uint8_t,6>;
class ApfpvStation;  // fwd: liveness/deauth notifications for the reconnect supervisor
class RxDeframe {
public:
    using OnRtpFn = std::function<void(const uint8_t*, size_t)>;
    RxDeframe(const Mac& self, const Mac& bssid, Wpa2Supplicant* wpa, LqFeedback* lq, OnRtpFn onRtp);
    void onPacket(const Packet& pkt);
    void setStation(ApfpvStation* st) { _station = st; }
    // DHCP reply hook: received UDP -> port 68 (BOOTP client) forwarded here so
    // the DHCP state machine can advance OFFER->REQUEST->ACK->Bound.
    using OnDhcpFn = std::function<void(const uint8_t*, size_t)>;
    void setDhcpSink(OnDhcpFn fn) { _onDhcp = std::move(fn); }
    // General-IP sink (the VpnService TUN): the FULL decrypted IPv4 packet, so SSH/any
    // TCP+UDP traverses the dongle — not just RTP. The OS then routes it (RTP->5600, SSH->22).
    using OnIpFn = std::function<void(const uint8_t*, size_t)>;
    void setIpSink(OnIpFn fn) { _onIp = std::move(fn); }
    // ARP responder: answer "who has <ourIp>?" so peers (the VTX unicasting RTP video, SSH,
    // ...) keep a FRESH ARP entry for us — without it a STALE entry fails to re-validate and
    // the unicast stream stops after a frame or two. send() gets the encrypted ARP-reply MPDU.
    void setArp(uint32_t ourIp, std::function<void(const std::vector<uint8_t>&)> send) {
        _ourIp = ourIp; _arpSend = std::move(send);
    }
private:
    static int toDbm(uint8_t r);
    Mac _self, _bssid; Wpa2Supplicant* _wpa; LqFeedback* _lq; OnRtpFn _onRtp;
    OnDhcpFn _onDhcp; OnIpFn _onIp;
    uint32_t _ourIp = 0;
    std::function<void(const std::vector<uint8_t>&)> _arpSend;
    ApfpvStation* _station = nullptr;
    // Per-payload-type recent-RTP-seq window for dropping 802.11-retransmit duplicates
    // (reordered retransmits need a window, not just the previous seq) + debug counters.
    uint16_t _seqHist[128][128] = {};
    bool     _seqValid[128][128] = {};
    uint8_t  _seqPos[128] = {};
    int      _dbgRx = 0, _dbgDrop = 0;
    // RX-health instrumentation (logged to logcat tag "rxd-health"): decrypt failures + RTP-seq-gap
    // loss, to diagnose dongle jumps/rewinds (loss vs duplication vs decrypt failure).
    int      _dbgDecFail = 0, _dbgLoss = 0, _dbgOoo = 0;
    int      _dbgAmsdu = 0, _dbgAmsduSub = 0;   // A-MSDU frames seen + total subframes de-aggregated
    uint16_t _lastSeq[128] = {};
    bool     _lastSeqV[128] = {};
    // Bottleneck diag: cumulative time (ns) spent in onPacket phases per interval
    int64_t  _diagTotalNs = 0, _diagDecNs = 0, _diagFwdNs = 0;
    int      _diagPkts = 0;
    // A-MPDU RX reorder buffer (kernel: recv_indicatepkt_reorder).
    // Per-TID sliding window for in-order delivery of A-MPDU subframes.
    // Without this, out-of-order subframes are dropped -> video corruption.
    struct ReorderCtl {
        bool     enable = false;
        uint16_t indicate_seq = 0xffff;  // next expected seq (mod 4096)
        uint16_t wsize_b = 64;           // window size in frames (USB needs > kernel's BA bufsz 64)
        std::map<uint16_t, std::vector<uint8_t>> pending; // seq -> plaintext frame
        int64_t  lastFlushMs = 0;        // kernel reorder release-timer: force-flush a stuck gap
    };
    ReorderCtl _reorder[16];  // one per TID (0-15)
    // Process a decrypted QoS-data frame through the reorder buffer for TID `tid`.
    // Delivers frames in-order via onRtpFn. Returns true if delivered, false if queued/dropped.
    bool processReorder(uint8_t tid, uint16_t seq, const uint8_t* llc, size_t llcLen);
    // Kernel rtw_process_bar_frame: on a BlockAckReq, advance the reorder window to start_seq
    // (flush everything below it). Keeps the SW reorder from stalling when the AP moves on.
    void processBar(uint8_t tid, uint16_t start_seq);
    // Emit one MSDU's RTP payload to _onRtp (shared by direct + reorder paths).
    void emitReorderRtp(const uint8_t* llc, size_t llcLen);
    // RTP-SEQUENCE reorder: the depacketizer consumes UDP/5600 RTP in arrival order, but the
    // dongle's USB/A-MPDU delivery hands packets up OUT of RTP-seq order. Re-keying the reorder
    // on the 802.11 MPDU seq was WRONG for this VTX (A-MSDU/constant MPDU seq) — it buffered
    // ~everything and skipped more than it delivered (rxd-reo: buffered>>inOrder, farSkip>>0),
    // which is net-harmful. The unit the depacketizer actually needs in order is the RTP seq, so
    // buffer the RTP payload keyed by its 16-bit seq and emit strictly in order. Returns true if
    // delivered (in order or recovered), false if queued/old.
    bool processReorderRtp(uint16_t rtpSeq, const uint8_t* rtp, size_t rtpLen);
    // Per-stream RTP-seq reorder state (mod 65536). One slot: the single :5600 video flow.
    struct RtpReorderCtl {
        bool     enable = false;
        uint16_t indicate = 0;            // next expected RTP seq
        int      wsize = 256;             // ~4 video frames of slack for the bursty USB delivery
        std::map<uint16_t, std::vector<uint8_t>> pending; // rtpSeq -> payload
        int64_t  lastFlushMs = 0;
    };
    RtpReorderCtl _reorderRtp{};
    bool _reorderOn = false;  // gated: DEVOURER_REORDER / debug.pixelpilot.reorder
};
}
