#ifndef FRAME_PARSER_8822C_H
#define FRAME_PARSER_8822C_H

#include <cstddef>
#include <cstdint>

#include "basic_types.h" /* cpu_to_le32 / le16_to_cpu used by the LE macros */
#include "FrameParser.h" /* SET_BITS_TO_LE_4BYTE / LE_BITS_TO_4BYTE */

/* Jaguar3 (8822C/8812EU/8822EU) TX/RX descriptor layout.
 *
 * This is the Jaguar3 descriptor — distinct from the Jaguar1 SET_TX_DESC_*_8812
 * layout in src/FrameParser.h. Field offsets/widths transcribed verbatim from
 * Realtek halmac (halmac_tx_desc_nic.h / halmac_rx_desc_nic.h, addressed by the
 * 8822C macro layer halmac_tx_desc_chip.h). The TX descriptor is 48 bytes (vs
 * Jaguar1's 40) and carries a 16-bit descriptor checksum at offset 0x1C that the
 * USB MAC verifies — see cal_txdesc_chksum_8822c().
 *
 * Encoding uses the same SET_BITS_TO_LE_4BYTE / LE_BITS_TO_4BYTE helpers as the
 * Jaguar1 macros. Validated on hardware at M4/M5. */

namespace jaguar3 {

constexpr size_t TXDESC_SIZE_8822C = 48; /* TX_DESC_SIZE_88XX */
constexpr size_t RXDESC_SIZE_8822C = 24; /* RX_DESC_SIZE_88XX */

} /* namespace jaguar3 */

/* --- TX descriptor fields (halmac SET_TX_DESC_*_8822C -> generic offsets) --- */
#define SET_TX_DESC_TXPKTSIZE_8822C(d, v)  SET_BITS_TO_LE_4BYTE((d) + 0x00, 0, 16, v)
#define SET_TX_DESC_OFFSET_8822C(d, v)     SET_BITS_TO_LE_4BYTE((d) + 0x00, 16, 8, v)
#define SET_TX_DESC_BMC_8822C(d, v)        SET_BITS_TO_LE_4BYTE((d) + 0x00, 24, 1, v)
#define SET_TX_DESC_LS_8822C(d, v)         SET_BITS_TO_LE_4BYTE((d) + 0x00, 26, 1, v)
#define SET_TX_DESC_MACID_8822C(d, v)      SET_BITS_TO_LE_4BYTE((d) + 0x04, 0, 7, v)
#define SET_TX_DESC_QSEL_8822C(d, v)       SET_BITS_TO_LE_4BYTE((d) + 0x04, 8, 5, v)
#define SET_TX_DESC_RATE_ID_8822C(d, v)    SET_BITS_TO_LE_4BYTE((d) + 0x04, 16, 5, v)
#define SET_TX_DESC_PKT_OFFSET_8822C(d, v) SET_BITS_TO_LE_4BYTE((d) + 0x04, 24, 5, v)
#define SET_TX_DESC_USE_RATE_8822C(d, v)   SET_BITS_TO_LE_4BYTE((d) + 0x0C, 8, 1, v)
#define SET_TX_DESC_DISDATAFB_8822C(d, v)  SET_BITS_TO_LE_4BYTE((d) + 0x0C, 10, 1, v)
#define SET_TX_DESC_DATARATE_8822C(d, v)   SET_BITS_TO_LE_4BYTE((d) + 0x10, 0, 7, v)
#define SET_TX_DESC_DATA_SHORT_8822C(d, v) SET_BITS_TO_LE_4BYTE((d) + 0x14, 4, 1, v)
#define SET_TX_DESC_DATA_BW_8822C(d, v)    SET_BITS_TO_LE_4BYTE((d) + 0x14, 5, 2, v)
#define SET_TX_DESC_DATA_LDPC_8822C(d, v)  SET_BITS_TO_LE_4BYTE((d) + 0x14, 7, 1, v)
#define SET_TX_DESC_DATA_STBC_8822C(d, v)  SET_BITS_TO_LE_4BYTE((d) + 0x14, 8, 2, v)
#define SET_TX_DESC_SW_DEFINE_8822C(d, v)  SET_BITS_TO_LE_4BYTE((d) + 0x18, 0, 12, v)
#define SET_TX_DESC_TXDESC_CHECKSUM_8822C(d, v) SET_BITS_TO_LE_4BYTE((d) + 0x1C, 0, 16, v)
#define SET_TX_DESC_EN_HWSEQ_8822C(d, v)   SET_BITS_TO_LE_4BYTE((d) + 0x20, 15, 1, v)
#define GET_TX_DESC_PKT_OFFSET_8822C(d)    LE_BITS_TO_4BYTE((d) + 0x04, 24, 5)

/* --- RX descriptor fields --- */
#define GET_RX_DESC_PKT_LEN_8822C(r)       LE_BITS_TO_4BYTE((r) + 0x00, 0, 14)
#define GET_RX_DESC_CRC32_8822C(r)         LE_BITS_TO_4BYTE((r) + 0x00, 14, 1)
#define GET_RX_DESC_ICV_ERR_8822C(r)       LE_BITS_TO_4BYTE((r) + 0x00, 15, 1)
#define GET_RX_DESC_DRV_INFO_SIZE_8822C(r) LE_BITS_TO_4BYTE((r) + 0x00, 16, 4)
#define GET_RX_DESC_SHIFT_8822C(r)         LE_BITS_TO_4BYTE((r) + 0x00, 24, 2)
#define GET_RX_DESC_PHYST_8822C(r)         LE_BITS_TO_4BYTE((r) + 0x00, 26, 1)
#define GET_RX_DESC_RX_RATE_8822C(r)       LE_BITS_TO_4BYTE((r) + 0x0C, 0, 7)

namespace jaguar3 {

/* Port of fill_txdesc_check_sum_8822c (halmac_common_8822c.c): clear the 16-bit
 * checksum field, then XOR all 16-bit little-endian words of the descriptor
 * (PKT_OFFSET + TXDESC_SIZE/8) << 1 word-pairs) and store the result. Read
 * byte-wise so the on-wire result is identical on any host endianness. */
inline void cal_txdesc_chksum_8822c(uint8_t *txdesc) {
  SET_TX_DESC_TXDESC_CHECKSUM_8822C(txdesc, 0);
  auto le16 = [&](size_t word) -> uint16_t {
    return static_cast<uint16_t>(txdesc[2 * word] |
                                 (txdesc[2 * word + 1] << 8));
  };
  uint16_t pkt_offset =
      static_cast<uint16_t>(GET_TX_DESC_PKT_OFFSET_8822C(txdesc));
  uint16_t pairs =
      static_cast<uint16_t>((pkt_offset + (TXDESC_SIZE_8822C >> 3)) << 1);
  uint16_t chksum = 0;
  for (uint16_t i = 0; i < pairs; i++)
    chksum ^= static_cast<uint16_t>(le16(2 * i) ^ le16(2 * i + 1));
  SET_TX_DESC_TXDESC_CHECKSUM_8822C(txdesc, chksum);
}

/* Fill an 8822C data/monitor-inject TX descriptor (48 bytes, zeroed by caller)
 * and finalise its checksum. `bw` is the descriptor BW code (0=20,1=40,2=80),
 * `rate_hw` the DESC_RATE* index (MRateToHwRate output), `rate_id` 8(HT)/9(VHT).
 * Field choices mirror the Jaguar1 monitor-inject path (MACID 1, USE_RATE,
 * DISDATAFB, HW sequence). TODO(M5/HW): confirm QSEL routing for 8822C. */
inline void fill_data_tx_desc_8822c(uint8_t *d, uint16_t pkt_size,
                                    uint8_t rate_hw, uint8_t rate_id, uint8_t bw,
                                    bool short_gi, bool ldpc, uint8_t stbc) {
  SET_TX_DESC_TXPKTSIZE_8822C(d, pkt_size);
  SET_TX_DESC_OFFSET_8822C(d, static_cast<uint32_t>(TXDESC_SIZE_8822C));
  SET_TX_DESC_LS_8822C(d, 1);
  SET_TX_DESC_MACID_8822C(d, 0x01);
  SET_TX_DESC_QSEL_8822C(d, 0x12); /* mirror Jaguar1 monitor-inject queue */
  SET_TX_DESC_RATE_ID_8822C(d, rate_id);
  SET_TX_DESC_USE_RATE_8822C(d, 1);
  SET_TX_DESC_DISDATAFB_8822C(d, 1);
  SET_TX_DESC_DATARATE_8822C(d, rate_hw);
  SET_TX_DESC_DATA_BW_8822C(d, bw);
  SET_TX_DESC_DATA_SHORT_8822C(d, short_gi ? 1 : 0);
  SET_TX_DESC_DATA_LDPC_8822C(d, ldpc ? 1 : 0);
  SET_TX_DESC_DATA_STBC_8822C(d, stbc & 0x3);
  SET_TX_DESC_EN_HWSEQ_8822C(d, 1);
  cal_txdesc_chksum_8822c(d);
}

/* Parsed view of one 8822C RX frame within a (possibly aggregated) bulk-IN
 * buffer. `frame`/`frame_len` point at the 802.11 PSDU after the descriptor +
 * driver-info + shift padding. */
struct Rx8822cFrame {
  const uint8_t *frame;  /* 802.11 frame start (inside the input buffer) */
  uint32_t frame_len;    /* PKT_LEN */
  bool crc_err;
  bool icv_err;
  uint8_t rx_rate;       /* RX_RATE (DESC_RATE* index) */
  uint16_t drvinfo_size; /* bytes (DRV_INFO_SIZE * 8) */
  uint8_t shift;         /* SHIFT_SZ */
  uint32_t next_offset;  /* 8-byte-aligned offset of the next frame in an agg */
};

/* Decode the leading RX descriptor of `buf` and locate its PSDU. Mirrors the
 * Jaguar1 rtl8812_query_rx_desc_status path (same 24-byte descriptor + drvinfo/
 * shift layout) with 8822C field positions. Returns false on a truncated /
 * malformed buffer. M4 will layer PHY-status (RSSI/SNR) + radiotap on top. */
inline bool parse_rx_8822c(const uint8_t *buf, size_t buflen,
                           Rx8822cFrame &out) {
  if (buf == nullptr || buflen < RXDESC_SIZE_8822C)
    return false;

  out.frame_len = static_cast<uint32_t>(GET_RX_DESC_PKT_LEN_8822C(buf));
  out.drvinfo_size =
      static_cast<uint16_t>(GET_RX_DESC_DRV_INFO_SIZE_8822C(buf) * 8);
  out.shift = static_cast<uint8_t>(GET_RX_DESC_SHIFT_8822C(buf));
  out.crc_err = GET_RX_DESC_CRC32_8822C(buf) != 0;
  out.icv_err = GET_RX_DESC_ICV_ERR_8822C(buf) != 0;
  out.rx_rate = static_cast<uint8_t>(GET_RX_DESC_RX_RATE_8822C(buf));

  uint32_t frame_off =
      static_cast<uint32_t>(RXDESC_SIZE_8822C) + out.drvinfo_size + out.shift;
  uint64_t end = static_cast<uint64_t>(frame_off) + out.frame_len;
  if (out.frame_len == 0 || end > buflen)
    return false;

  out.frame = buf + frame_off;
  /* next frame in an aggregate is 8-byte aligned */
  out.next_offset = (static_cast<uint32_t>(end) + 7) & ~0x7u;
  return true;
}

} /* namespace jaguar3 */

#endif /* FRAME_PARSER_8822C_H */
