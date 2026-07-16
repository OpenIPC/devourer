#ifndef DEVOURER_RX_PACKET_H
#define DEVOURER_RX_PACKET_H

#include <cstdint>
#include <optional>
#include <span>

/* Parsed RX packet types, shared across chip generations. The per-generation
 * frame parsers (Jaguar1 FrameParser, Jaguar3 FrameParserJaguar3) both produce
 * `Packet`s; the USB adapter and demos consume them. Kept neutral so neither
 * generation depends on the other's parser header. */

enum class RX_PACKET_TYPE
{
    NORMAL_RX,  /* Normal rx packet */
    TX_REPORT1, /* CCX */
    TX_REPORT2, /* TX RPT */
    HIS_REPORT, /* USB HISR RPT */
    C2H_PACKET
};

struct rx_pkt_attrib
{
    uint16_t pkt_len;
    bool physt;
    uint8_t drvinfo_sz;
    uint8_t shift_sz;
    bool qos;
    uint8_t priority;
    bool mdata;
    uint16_t seq_num;
    uint8_t frag_num;
    bool mfrag;
    bool bdecrypted;
    uint8_t encrypt; /* when 0 indicate no encrypt. when non-zero, indicate the
                        encrypt algorith */
    bool crc_err;
    bool icv_err;
    /* TSF low (4 bytes at RX-descriptor offset 20) — chip-side timestamp
     * for the frame. With the seq_num just above it, a downstream layer
     * can drop duplicates by seq and measure one-way latency by diffing
     * the chip's TSF against its own wall clock. Populated by
     * FrameParser; surfaced through examples/rx/main.cpp's rx.frame event. */
    uint32_t tsfl;
    /* AX/VHT datarate code is 9-bit (HT 0x80+, VHT 0x100+, HE 0x180+), so a
     * uint8_t truncates the format bits — VHT2SS+ and every HE rate collapse
     * onto an HT/legacy code. Kept 16-bit so the reported RX rate is truthful
     * (needed to identify HE frames when a Kestrel acts as an HE monitor). */
    uint16_t data_rate;
    uint8_t bw;
    uint8_t stbc;
    uint8_t ldpc;
    uint8_t sgi;
    /* Descrambler seed the chip recovered from this frame's SERVICE field.
     * Trustworthy only on RTL8814AU (see FrameParser.cpp); surfaced for the
     * DEVOURER_DUMP_SCRAMBLER hook in examples/rx/main.cpp. */
    uint8_t scrambler;
    /* RSSI / SNR per RF path: A, B (Jaguar 8812/8811) plus C, D (8814AU). On
     * non-8814 chips the [2..3] slots are zero — the upstream RX phy-status
     * report reserves those bytes when only 2 paths are active. */
    uint8_t rssi[4];
    int8_t snr[4];
    /* Per-stream RX EVM (A,B on 8812/8811; plus C,D on 8814). Raw RX-status
     * bytes — a link-quality metric, NOT a per-subcarrier or content signal.
     * Surfaced for the DEVOURER_DUMP_BODY Tier-2 health diagnostic. */
    int8_t evm[4];
    /* Path-A CFO tail from the OFDM phy-status (signed HW units; kHz = raw *
     * 2.5, phydm CFO_HW_RPT_2_KHZ). The carrier-frequency offset between this
     * receiver's crystal and the transmitter's — the closed-loop CFO tracker's
     * input (see IRtlDevice::SetXtalCap). 0 when the phy-status carries none. */
    int8_t cfo_tail = 0;
    /* A-MPDU RX markers. paggr: this MPDU arrived inside an aggregated PPDU
     * (rx-desc PAGGR — 8812 dword1[15], same position in the halmac layout).
     * ppdu_cnt (halmac only): the MAC's 2-bit received-PPDU counter —
     * consecutive frames sharing a value shared one PPDU, THE ground-truth
     * A-MPDU detection signal on the RX side (with paggr and the fact that
     * only an aggregate's first subframe carries a PHY status). */
    bool paggr = false;
    uint8_t ppdu_cnt = 0;
    /* RX PPDU format classification (AX RX-descriptor dword1[3:0], Kestrel
     * only): 0/1=CCK 2=OFDM 3=HT 5/6=VHT 7=HE_SU 8=HE_ERSU 9=HE_MU 10=HE_TB.
     * The proof a received frame really was the HE ER SU extended-range
     * format. 0xff on pre-AX generations (their descriptors carry no such
     * field). */
    uint8_t ppdu_type = 0xff;
    RX_PACKET_TYPE pkt_rpt_type;
};

struct Packet
{
    rx_pkt_attrib RxAtrib;
    /* Full 802.11 frame including the trailing FCS. Every Realtek RX parser
     * follows this contract; consumers remove the FCS at their protocol
     * boundary rather than making the frame length chip-specific. Retaining it
     * also lets DEVOURER_RX_KEEP_CORRUPTED and fused-FEC salvage inspect a
     * failed frame, while tools/bf_report_decode.py trims the trailing four
     * bytes when decoding beamforming reports. */
    std::span<uint8_t> Data;

    /* The transmitter's hardware TX-egress TSF, when the frame carries one.
     *
     * For the frames the 802.11 MAC regenerates itself — beacons and probe
     * responses — the sender's MAC overwrites the 8-byte timestamp field (MPDU
     * bytes 24-31) with its live TSF at the instant the frame is clocked onto
     * the air. That is a genuine hardware TX-egress timestamp, latched below the
     * CSMA/queueing layer (bench-measured sub-µs against an independent
     * receiver, vs ~100+ µs for any host-side "read the clock after send"
     * approximation). Paired with RxAtrib.tsfl (this receiver's hardware RX
     * timestamp), each such frame yields the {remote egress, local arrival}
     * pair that one-way hardware time distribution is built on — with no
     * host-clock jitter on either end, and against any beaconing AP, not just a
     * devourer transmitter.
     *
     * Returns nullopt for frames the MAC does not stamp (data frames and mgmt
     * frames other than beacon / probe-response). */
    std::optional<uint64_t> TxEgressTsf() const
    {
        if (Data.size() < 32)
            return std::nullopt;
        const uint8_t fc0 = Data[0];      /* proto=0, type=mgmt, subtype in [7:4] */
        if (fc0 != 0x80 /* beacon */ && fc0 != 0x50 /* probe response */)
            return std::nullopt;
        uint64_t tsf = 0;
        for (int i = 0; i < 8; ++i)
            tsf |= static_cast<uint64_t>(Data[24 + i]) << (8 * i);
        return tsf;
    }
};

#endif /* DEVOURER_RX_PACKET_H */
