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
} /* namespace txd */

constexpr uint8_t MAC_AX_DMA_B0MG = 8;  /* band-0 mgmt DMA channel (CH_DMA) */
constexpr uint8_t MAC_AX_MG0_SEL = 18;  /* band-0 mgmt qsel */
constexpr uint32_t WD_BODY_LEN = 24;
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
inline std::vector<uint8_t> build_mgnt_txdesc(const uint8_t *frame,
                                              uint32_t frame_len, uint16_t rate,
                                              uint8_t macid, uint16_t seq) {
  std::vector<uint8_t> buf(TXD_MGNT_LEN + frame_len, 0);
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
  /* dword4,5 = 0 */

  /* wd_info dword0: force the rate (USERATE_SEL) + DATARATE + no fallback. */
  uint8_t *wi = wd + WD_BODY_LEN;
  txd_put_le32(wi + 0, txd::USERATE_SEL | txd::DISDATAFB |
                           (static_cast<uint32_t>(rate & 0x1ff)
                            << txd::DATARATE_SH));
  /* wd_info dword1..5 = 0 */

  std::memcpy(wd + TXD_MGNT_LEN, frame, frame_len);
  return buf;
}

} /* namespace kestrel */

#endif /* KESTREL_TX_DESC_KESTREL_H */
