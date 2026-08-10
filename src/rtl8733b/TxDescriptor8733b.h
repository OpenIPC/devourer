#ifndef RTL8733B_TX_DESCRIPTOR_H
#define RTL8733B_TX_DESCRIPTOR_H

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace rtl8733b {

inline constexpr size_t kTxDescSize = 40;
inline constexpr size_t kTxChecksumBytes = 32;
inline constexpr uint8_t kTxQselManagement = 0x12;
inline constexpr uint8_t kTxMacidManagement = 1;

/* RATE_ID is still consumed by parts of the MAC even when USE_RATE fixes the
 * actual descriptor rate. These values are the RTL8733B vendor RA table IDs:
 * BGN 20/40 MHz 1SS on 2.4 GHz, GN 1SS on 5 GHz, and legacy BG/G. */
inline uint8_t tx_rate_id_8733b(uint8_t rate_hw, uint8_t bandwidth,
                                bool is_5ghz) {
  if (rate_hw >= 4 && rate_hw <= 11)
    return is_5ghz ? 7 : 6;
  if (rate_hw >= 12 && rate_hw <= 19) {
    if (is_5ghz)
      return 5;
    return bandwidth == 1 ? 1 : 3;
  }
  return 0xff;
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
  uint8_t retry_limit = 0;
  bool short_gi = false;
  bool ldpc = false;
  bool bmc = false;
};

inline bool valid_tx_desc_config(const TxDescConfig &cfg) {
  const bool legacy_ofdm = cfg.rate_hw >= 4 && cfg.rate_hw <= 11;
  const bool ht_1ss = cfg.rate_hw >= 12 && cfg.rate_hw <= 19;
  return cfg.packet_size != 0 && cfg.sequence <= 0x0fff &&
         (legacy_ofdm || ht_1ss) && cfg.rate_id <= 0x1f &&
         cfg.bandwidth <= 1 && cfg.data_sc <= 0x0f &&
         cfg.packet_offset <= 1 &&
         cfg.retry_limit <= 0x3f &&
         !cfg.ldpc && (!legacy_ofdm || !cfg.short_gi);
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
