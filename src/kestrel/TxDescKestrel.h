#ifndef KESTREL_TX_DESC_KESTREL_H
#define KESTREL_TX_DESC_KESTREL_H

#include <cstdint>
#include <cstring>
#include <vector>

namespace kestrel {

/* AX TX descriptor composer (wd_body_t 24B + wd_info_t 24B) for a band-0
 * management frame on USB, ported verbatim from mac_ax txdes_proc_mgnt_8852b
 * (mac_8852b/trx_desc_8852b.c). Only the fields a fixed-rate monitor/mgmt TX
 * needs are set; everything else is 0 (no security / aggregation / RTS). The
 * mgmt queue (MAC_AX_DMA_B0MG=8) maps to USB BULKOUTID0 per
 * get_bulkout_id_8852b, so the caller sends this to the 0th bulk-OUT endpoint. */

namespace txd {
/* AX_TXD_* field positions (phl/hal_g6/mac/txdesc.h). */
constexpr uint32_t STF_MODE = 1u << 10;    /* wd_body dword0 (USB store-fwd) */
constexpr uint8_t CH_DMA_SH = 16;          /* wd_body dword0 [19:16] */
constexpr uint32_t WDINFO_EN = 1u << 22;   /* wd_body dword0 */
constexpr uint8_t TXPKTSIZE_SH = 0;        /* wd_body dword2 [13:0] */
constexpr uint8_t QSEL_SH = 17;            /* wd_body dword2 [22:17] */
constexpr uint8_t MACID_SH = 24;           /* wd_body dword2 [30:24] */
constexpr uint8_t WIFI_SEQ_SH = 0;         /* wd_body dword3 [11:0] */
constexpr uint32_t USERATE_SEL = 1u << 30; /* wd_info dword0 (force f_rate) */
constexpr uint8_t DATARATE_SH = 16;        /* wd_info dword0 [24:16] */
constexpr uint32_t DISDATAFB = 1u << 10;   /* wd_info dword0 (no rate FB) */
constexpr uint32_t DATA_LDPC = 1u << 11;   /* wd_info dword0 */
constexpr uint32_t DATA_STBC = 1u << 12;   /* wd_info dword0 */
constexpr uint8_t GI_LTF_SH = 25;          /* wd_info dword0 [27:25] */
constexpr uint8_t DATA_BW_SH = 28;         /* wd_info dword0 [29:28] */
} /* namespace txd */

/* Map a devourer MGN_* rate (RateDefinitions.h) to the AX_TXD_DATARATE encoding
 * (rtw_general_def.h): legacy CCK 0..3 / OFDM 4..11, HT 0x80+mcs (MGN_MCS0 is
 * already 0x80), VHT 0x100+(nss-1)*16+mcs. HE (0x180) lands in M5. */
inline uint16_t mgn_to_ax_rate(uint8_t mgn) {
  switch (mgn) {          /* legacy — MGN_* are rate*2 in 500kbps units */
  case 0x02: return 0;    /* MGN_1M  -> CCK1  */
  case 0x04: return 1;    /* MGN_2M  -> CCK2  */
  case 0x0B: return 2;    /* MGN_5_5M-> CCK5.5*/
  case 0x16: return 3;    /* MGN_11M -> CCK11 */
  case 0x0C: return 4;    /* MGN_6M  -> OFDM6 */
  case 0x12: return 5;    /* MGN_9M  */
  case 0x18: return 6;    /* MGN_12M */
  case 0x24: return 7;    /* MGN_18M */
  case 0x30: return 8;    /* MGN_24M */
  case 0x48: return 9;    /* MGN_36M */
  case 0x60: return 10;   /* MGN_48M */
  case 0x6C: return 11;   /* MGN_54M */
  default: break;
  }
  /* MGN_VHT1SS_MCS0 = MGN_MCS0(0x80) + 32 = 0xA0; 10 MCS per NSS. */
  constexpr uint8_t MGN_MCS0_V = 0x80, MGN_VHT1SS_MCS0_V = 0xA0;
  if (mgn >= MGN_VHT1SS_MCS0_V) {
    unsigned off = mgn - MGN_VHT1SS_MCS0_V;
    unsigned nss = off / 10, mcs = off % 10;
    return static_cast<uint16_t>(0x100 + nss * 16 + mcs);
  }
  if (mgn >= MGN_MCS0_V)
    return mgn; /* HT: MGN_MCSn == AX 0x80+n */
  return 4;     /* fallback 6M OFDM */
}

constexpr uint8_t MAC_AX_DMA_B0MG = 8;  /* band-0 mgmt DMA channel (CH_DMA) */
constexpr uint8_t MAC_AX_MG0_SEL = 18;  /* band-0 mgmt qsel */
constexpr uint32_t WD_BODY_LEN = 24;    /* wd_body_t (8852B) */
constexpr uint32_t WD_BODY_LEN_V1 = 32; /* wd_body_t_v1 (8852C data/mgmt TX) */
constexpr uint32_t WD_INFO_LEN = 24;
constexpr uint32_t TXD_MGNT_LEN = WD_BODY_LEN + WD_INFO_LEN; /* 48 */

/* AX_TXD_DATARATE indices: CCK 1/2/5.5/11 = 0..3, OFDM 6/9/12/18/24/36/48/54 =
 * 4..11; HT/VHT/HE add 0x80/0x100/0x180 (M5). */
enum { RATE_CCK1 = 0, RATE_OFDM6 = 4, RATE_OFDM24 = 8 };

inline void txd_put_le32(uint8_t *p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
  p[2] = static_cast<uint8_t>(v >> 16);
  p[3] = static_cast<uint8_t>(v >> 24);
}

/* Prepend the 48-byte AX mgmt TX descriptor to `frame` (the 802.11 MPDU with
 * radiotap already stripped). `rate` = AX datarate index, `macid` = TX MAC id
 * (0 = self), `seq` = 12-bit wifi sequence (only used when the WD drives the
 * seq; hw_seq_mode is left 0 so the frame's own seq stands). Returns the
 * [descriptor][frame] buffer for the bulk-OUT. */
struct TxRate {
  uint16_t rate = RATE_OFDM6; /* AX_TXD_DATARATE index (see mgn_to_ax_rate) */
  uint8_t bw = 0;             /* AX_TXD_DATA_BW: 0=20 1=40 2=80 3=160 MHz */
  uint8_t gi_ltf = 0;         /* AX_TXD_GI_LTF: 0=LGI 1=SGI (HT/VHT) */
  bool ldpc = false;
  bool stbc = false;
};

inline std::vector<uint8_t> build_mgnt_txdesc(const uint8_t *frame,
                                              uint32_t frame_len,
                                              const TxRate &r, uint8_t macid,
                                              uint16_t seq,
                                              uint32_t wd_body_len = WD_BODY_LEN) {
  /* The 8852C data/mgmt TX descriptor is the 32-byte wd_body_t_v1 (dwords 6/7
   * added, dword0-5 field layout identical); the 8852B is the 24-byte wd_body_t.
   * A short WD desyncs the MAC TX parser and the frame never airs (buffer still
   * drains). wd_info + frame follow the WD body. */
  const uint32_t txd_len = wd_body_len + WD_INFO_LEN;
  std::vector<uint8_t> buf(txd_len + frame_len, 0);
  uint8_t *wd = buf.data();

  /* wd_body dword0: STF_MODE | CH_DMA=B0MG | WDINFO_EN (usb_pkt_ofst=0). */
  txd_put_le32(wd + 0, txd::STF_MODE | txd::WDINFO_EN |
                           (static_cast<uint32_t>(MAC_AX_DMA_B0MG)
                            << txd::CH_DMA_SH));
  /* dword1 = 0 */
  /* wd_body dword2: TXPKTSIZE | QSEL=MG0 | MACID. */
  txd_put_le32(wd + 8,
               (static_cast<uint32_t>(frame_len & 0x3fff) << txd::TXPKTSIZE_SH) |
                   (static_cast<uint32_t>(MAC_AX_MG0_SEL) << txd::QSEL_SH) |
                   (static_cast<uint32_t>(macid & 0x7f) << txd::MACID_SH));
  /* wd_body dword3: WIFI_SEQ (sw_seq). */
  txd_put_le32(wd + 12, static_cast<uint32_t>(seq & 0xfff) << txd::WIFI_SEQ_SH);
  /* dword4,5 (+ v1 dword6,7) = 0 */

  /* wd_info dword0: force the rate (USERATE_SEL) + DATARATE + BW + GI/LTF +
   * LDPC/STBC + no fallback. */
  uint8_t *wi = wd + wd_body_len;
  txd_put_le32(wi + 0,
               txd::USERATE_SEL | txd::DISDATAFB |
                   (static_cast<uint32_t>(r.rate & 0x1ff) << txd::DATARATE_SH) |
                   (static_cast<uint32_t>(r.bw & 0x3) << txd::DATA_BW_SH) |
                   (static_cast<uint32_t>(r.gi_ltf & 0x7) << txd::GI_LTF_SH) |
                   (r.ldpc ? txd::DATA_LDPC : 0) |
                   (r.stbc ? txd::DATA_STBC : 0));
  /* wd_info dword1..5 = 0 */

  std::memcpy(wd + txd_len, frame, frame_len);
  return buf;
}

constexpr uint8_t MAC_AX_DATA_CH0 = 0; /* AC0 (best-effort) DMA channel */

/* Build the AX data-frame TX descriptor (txdes_proc_data_8852b), best-effort
 * (TID 0 -> qsel 0, band 0, wmm 0, no security / aggregation / shortcut).
 * Differs from the mgmt path only in wd_body dword0 CH_DMA=DATA_CH0 and dword2
 * QSEL=0; the wd_info rate fields are identical. The AC0 queue maps to USB
 * BULKOUTID3, so the caller sends this to the 4th bulk-OUT endpoint. */
inline std::vector<uint8_t> build_data_txdesc(const uint8_t *frame,
                                              uint32_t frame_len,
                                              const TxRate &r, uint8_t macid,
                                              uint16_t seq,
                                              uint32_t wd_body_len = WD_BODY_LEN) {
  const uint32_t txd_len = wd_body_len + WD_INFO_LEN;
  std::vector<uint8_t> buf(txd_len + frame_len, 0);
  uint8_t *wd = buf.data();
  /* dword0: STF_MODE | CH_DMA=DATA_CH0(0) | WDINFO_EN. */
  txd_put_le32(wd + 0, txd::STF_MODE | txd::WDINFO_EN |
                           (static_cast<uint32_t>(MAC_AX_DATA_CH0)
                            << txd::CH_DMA_SH));
  /* dword1 = shcut_camid = 0 */
  /* dword2: TXPKTSIZE | QSEL=0 (TID0 AC_BE) | MACID. */
  txd_put_le32(wd + 8,
               (static_cast<uint32_t>(frame_len & 0x3fff) << txd::TXPKTSIZE_SH) |
                   (static_cast<uint32_t>(macid & 0x7f) << txd::MACID_SH));
  /* dword3: WIFI_SEQ (no ampdu_en). */
  txd_put_le32(wd + 12, static_cast<uint32_t>(seq & 0xfff) << txd::WIFI_SEQ_SH);
  /* wd_info dword0: same rate fields as mgmt. */
  uint8_t *wi = wd + wd_body_len;
  txd_put_le32(wi + 0,
               txd::USERATE_SEL | txd::DISDATAFB |
                   (static_cast<uint32_t>(r.rate & 0x1ff) << txd::DATARATE_SH) |
                   (static_cast<uint32_t>(r.bw & 0x3) << txd::DATA_BW_SH) |
                   (static_cast<uint32_t>(r.gi_ltf & 0x7) << txd::GI_LTF_SH) |
                   (r.ldpc ? txd::DATA_LDPC : 0) |
                   (r.stbc ? txd::DATA_STBC : 0));
  std::memcpy(wd + txd_len, frame, frame_len);
  return buf;
}

/* 802.11 frame-control type field (frame[0] bits [3:2]): 0=mgmt, 2=data. */
inline bool frame_is_data(const uint8_t *frame, uint32_t len) {
  return len >= 1 && ((frame[0] >> 2) & 0x3) == 0x2;
}

} /* namespace kestrel */

#endif /* KESTREL_TX_DESC_KESTREL_H */
