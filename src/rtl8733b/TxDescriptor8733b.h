#ifndef RTL8733B_TX_DESCRIPTOR_H
#define RTL8733B_TX_DESCRIPTOR_H

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "TxMode.h"

namespace rtl8733b {

inline constexpr size_t kTxDescSize = 40;
inline constexpr size_t kTxChecksumBytes = 32;
inline constexpr uint8_t kTxQselManagement = 0x12;
inline constexpr uint8_t kTxMacidManagement = 1;

/* RATE_ID is still consumed by parts of the MAC even when USE_RATE fixes the
 * actual descriptor rate. These values are the RTL8733B vendor RA table IDs:
 * BGN 20/40 MHz 1SS on 2.4 GHz, GN 1SS on 5 GHz, legacy BG/G, and B for
 * 2.4-GHz CCK. */
inline uint8_t tx_rate_id_8733b(uint8_t rate_hw, uint8_t bandwidth,
                                bool is_5ghz) {
  if (rate_hw <= 3)
    return !is_5ghz && bandwidth == 0 ? 8 : 0xff;
  if (rate_hw >= 4 && rate_hw <= 11)
    return is_5ghz ? 7 : 6;
  if (rate_hw >= 12 && rate_hw <= 19) {
    if (is_5ghz)
      return 5;
    return bandwidth == 1 ? 1 : 3;
  }
  return 0xff;
}

/* The modulation-admission contract, factored out of
 * Rtl8733bDevice::build_tx_block so both the SetTxMode branch and the radiotap
 * branch share one predicate and the refusals are testable without hardware.
 *
 * The advertised surface is 1SS BCC: legacy CCK/OFDM at 20 MHz and HT MCS0-7
 * at 20/40 MHz. SGI is refused because an independent RTL8812AU witness
 * decoded a descriptor-SGI probe as long GI; LDPC and STBC are refused because
 * only forced-global-BCC operation has passed witnessed injection, and the
 * descriptor encoder writes no STBC field at all. VHT and HE are outside the
 * contract on every band — both vendor trees declare this silicon 802.11b/g/n,
 * 1SS, 20/40 MHz — so the device path refuses those radiotap fields outright.
 */
inline bool ht_request_supported_8733b(unsigned mcs, unsigned bw_mhz, bool sgi,
                                       bool ldpc, bool stbc) {
  return mcs <= 7 && (bw_mhz == 20 || bw_mhz == 40) && !sgi && !ldpc && !stbc;
}

inline bool legacy_request_supported_8733b(unsigned bw_mhz, bool sgi, bool ldpc,
                                           bool stbc) {
  return bw_mhz == 20 && !sgi && !ldpc && !stbc;
}

/* Is a radiotap/TxMode legacy rate (500 kbps units) one of the four
 * long-preamble CCK rates? 1M=2, 2M=4, 5.5M=11, 11M=22. Used to pick the TSSI
 * thermal-compensation curve at setup, mirroring the vendor's rate-keyed
 * table choice. */
inline bool is_cck_rate_500kbps(uint8_t rate_500kbps) {
  return rate_500kbps == 2 || rate_500kbps == 4 || rate_500kbps == 11 ||
         rate_500kbps == 22;
}

/* Whole-TxMode admission: the PPDU family plus its modulation parameters. VHT
 * and HE fall through to false on every band. */
inline bool tx_mode_supported_8733b(const devourer::TxMode &mode) {
  switch (mode.mode) {
  case devourer::TxMode::Mode::Legacy:
    return legacy_request_supported_8733b(mode.bw_mhz, mode.sgi, mode.ldpc,
                                          mode.stbc);
  case devourer::TxMode::Mode::HT:
    return ht_request_supported_8733b(mode.ht_mcs, mode.bw_mhz, mode.sgi,
                                      mode.ldpc, mode.stbc);
  default:
    return false;
  }
}

/* RTL8733B uses the common HALMAC 87xx descriptor layout, but its normal TX
 * descriptor is 40 bytes (not the neighboring 8822B/C 48-byte form). Keep the
 * encoder byte-wise: USB buffers need not be uint32_t-aligned. */
inline uint32_t txdesc_le32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

inline void txdesc_write_le32(uint8_t *p, uint32_t value) {
  p[0] = static_cast<uint8_t>(value);
  p[1] = static_cast<uint8_t>(value >> 8);
  p[2] = static_cast<uint8_t>(value >> 16);
  p[3] = static_cast<uint8_t>(value >> 24);
}

inline void txdesc_set_bits(uint8_t *p, unsigned bit, unsigned width,
                            uint32_t value) {
  if (width == 0 || width > 32 || bit >= 32 || width > 32 - bit)
    return;
  const uint32_t value_mask = width == 32 ? ~uint32_t{0}
                                          : (uint32_t{1} << width) - 1u;
  const uint32_t field = value_mask << bit;
  txdesc_write_le32(p, (txdesc_le32(p) & ~field) |
                           ((value << bit) & field));
}

struct TxDescConfig {
  uint16_t packet_size = 0;
  uint16_t sequence = 0;
  uint8_t rate_hw = 4; // HALMAC DESC_RATE6M
  uint8_t rate_id = 7;
  uint8_t bandwidth = 0; // 0=20 MHz, 1=40 MHz
  uint8_t data_sc = 0;
  uint8_t packet_offset = 0; // 8-byte USB boundary shim count
  /* USB TX aggregation block count, set on the FIRST descriptor of a packed
   * bulk-OUT URB and left 0 on every other descriptor and on every
   * single-frame transfer (0 and 1 are both "one block" to the TXDMA, and 0
   * keeps the single-frame path byte-identical to before this field existed).
   * HALMAC DMA_TXAGG_NUM, dword7[31:24] — the same placement the 8822C
   * carries it at, alongside the checksum in the same dword's low half.
   * Capped by BLK_DESC_NUM = 3, which MAC init already programs.
   *
   * ORDERING: byte 0x1f is INSIDE the checksummed span (the fold covers 32
   * bytes and skips only 0x1c-0x1d, the checksum field itself), so this must
   * be written BEFORE the checksum. fill_tx_desc_8733b does that by
   * construction; the 8822C's patch-then-recompute shape does not port here,
   * because this family folds the checksum inside the fill. */
  uint8_t agg_num = 0;
  uint8_t retry_limit = 0;
  bool short_gi = false;
  bool ldpc = false;
  bool bmc = false;
};

inline bool valid_tx_desc_config(const TxDescConfig &cfg) {
  const bool legacy_cck = cfg.rate_hw <= 3;
  const bool legacy_ofdm = cfg.rate_hw >= 4 && cfg.rate_hw <= 11;
  const bool ht_1ss = cfg.rate_hw >= 12 && cfg.rate_hw <= 19;
  return cfg.packet_size != 0 && cfg.sequence <= 0x0fff &&
         (legacy_cck || legacy_ofdm || ht_1ss) && cfg.rate_id <= 0x1f &&
         cfg.bandwidth <= 1 && cfg.data_sc <= 0x0f &&
         cfg.packet_offset <= 1 && cfg.agg_num <= 3 &&
         cfg.retry_limit <= 0x3f &&
         (!legacy_cck || cfg.bandwidth == 0) && !cfg.ldpc &&
         (!(legacy_cck || legacy_ofdm) || !cfg.short_gi);
}

inline uint16_t txdesc_checksum_8733b(const uint8_t *desc) {
  uint16_t checksum = 0;
  for (size_t offset = 0; offset < kTxChecksumBytes; offset += 2) {
    if (offset == 0x1c)
      continue; // checksum field is treated as zero
    checksum ^= static_cast<uint16_t>(desc[offset]) |
                static_cast<uint16_t>(desc[offset + 1] << 8);
  }
  return static_cast<uint16_t>(checksum ^ 0xffffu);
}

inline bool fill_tx_desc_8733b(uint8_t *desc, size_t desc_len,
                               const TxDescConfig &cfg) {
  if (desc == nullptr || desc_len < kTxDescSize || !valid_tx_desc_config(cfg))
    return false;

  std::fill_n(desc, kTxDescSize, 0);
  txdesc_set_bits(desc + 0x00, 0, 16, cfg.packet_size);
  txdesc_set_bits(desc + 0x00, 16, 8, kTxDescSize);
  txdesc_set_bits(desc + 0x00, 24, 1, cfg.bmc ? 1 : 0);
  txdesc_set_bits(desc + 0x04, 0, 7, kTxMacidManagement);
  txdesc_set_bits(desc + 0x04, 8, 5, kTxQselManagement);
  txdesc_set_bits(desc + 0x04, 16, 5, cfg.rate_id);
  txdesc_set_bits(desc + 0x04, 24, 5, cfg.packet_offset);
  txdesc_set_bits(desc + 0x0c, 8, 1, 1);  // USE_RATE
  txdesc_set_bits(desc + 0x0c, 9, 1, 1);  // DISRTSFB
  txdesc_set_bits(desc + 0x0c, 10, 1, 1); // DISDATAFB
  txdesc_set_bits(desc + 0x10, 17, 1, 1); // RTY_LMT_EN
  txdesc_set_bits(desc + 0x10, 18, 6, cfg.retry_limit);
  txdesc_set_bits(desc + 0x10, 0, 7, cfg.rate_hw);
  txdesc_set_bits(desc + 0x14, 0, 4, cfg.data_sc);
  txdesc_set_bits(desc + 0x14, 4, 1, cfg.short_gi ? 1 : 0);
  txdesc_set_bits(desc + 0x14, 5, 2, cfg.bandwidth);
  txdesc_set_bits(desc + 0x14, 7, 1, cfg.ldpc ? 1 : 0);
  txdesc_set_bits(desc + 0x20, 15, 1, 0); // preserve caller sequence
  txdesc_set_bits(desc + 0x24, 12, 12, cfg.sequence);
  /* DMA_TXAGG_NUM before the checksum, not after: the fold covers 32 bytes
   * skipping only 0x1c-0x1d (the checksum field itself), so byte 0x1f is
   * INSIDE the checksummed span. Writing the count afterwards would leave a
   * descriptor the chip rejects. */
  txdesc_set_bits(desc + 0x1c, 24, 8, cfg.agg_num);
  txdesc_set_bits(desc + 0x1c, 0, 16, txdesc_checksum_8733b(desc));
  return true;
}

inline bool txdesc_checksum_valid_8733b(const uint8_t *desc,
                                        size_t desc_len) {
  if (desc == nullptr || desc_len < kTxChecksumBytes)
    return false;
  uint16_t folded = 0;
  for (size_t offset = 0; offset < kTxChecksumBytes; offset += 2)
    folded ^= static_cast<uint16_t>(desc[offset]) |
              static_cast<uint16_t>(desc[offset + 1] << 8);
  return folded == 0xffffu;
}

} // namespace rtl8733b

#endif /* RTL8733B_TX_DESCRIPTOR_H */
