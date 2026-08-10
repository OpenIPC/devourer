#ifndef FRAME_PARSER_8733B_H
#define FRAME_PARSER_8733B_H

#include <cstddef>
#include <cstdint>
#include <limits>

#include "RxPacket.h"

namespace rtl8733b {

inline constexpr size_t kRxDescSize = 24;

inline uint32_t rx_le32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

inline uint32_t rx_bits(const uint8_t *p, unsigned bit, unsigned width) {
  const uint32_t mask = width == 32 ? (std::numeric_limits<uint32_t>::max)()
                                    : ((1u << width) - 1u);
  return (rx_le32(p) >> bit) & mask;
}

/* One packet in the RTL8733B USB RX aggregate. Descriptor layout and field
 * positions are the vendor HALMAC 87xx RX descriptor used by RTL8733B. */
struct RxFrame8733b {
  const uint8_t *frame = nullptr;
  uint32_t frame_len = 0;
  uint16_t drvinfo_size = 0;
  uint8_t shift = 0;
  uint8_t security = 0;
  uint8_t tid = 0;
  uint16_t sequence = 0;
  uint8_t fragment = 0;
  uint8_t rx_rate = 0;
  uint8_t dma_aggregate_count = 0;
  uint8_t ppdu_count = 0;
  uint32_t tsfl = 0;
  bool crc_err = false;
  bool icv_err = false;
  bool qos = false;
  bool amsdu = false;
  bool more_data = false;
  bool more_fragment = false;
  bool physt = false;
  bool c2h = false;
  bool paggr = false;
  bool ampdu_eof = false;
  bool sw_decrypted = false;
  uint32_t next_offset = 0;
};

/* Decode and bounds-check one descriptor/driver-info/body block. The next USB
 * packet starts at an 8-byte boundary (rtl8733bu_recv.c's _RND8 rule). */
inline bool parse_rx_8733b(const uint8_t *buf, size_t buflen,
                           RxFrame8733b &out) {
  out = {};
  if (buf == nullptr || buflen < kRxDescSize)
    return false;

  out.frame_len = rx_bits(buf + 0x00, 0, 14);
  out.c2h = rx_bits(buf + 0x08, 28, 1) != 0;
  out.dma_aggregate_count =
      static_cast<uint8_t>(rx_bits(buf + 0x0c, 16, 8));

  if (!out.c2h) {
    out.crc_err = rx_bits(buf + 0x00, 14, 1) != 0;
    out.icv_err = rx_bits(buf + 0x00, 15, 1) != 0;
    out.drvinfo_size =
        static_cast<uint16_t>(rx_bits(buf + 0x00, 16, 4) * 8u);
    out.security = static_cast<uint8_t>(rx_bits(buf + 0x00, 20, 3));
    out.qos = rx_bits(buf + 0x00, 23, 1) != 0;
    out.shift = static_cast<uint8_t>(rx_bits(buf + 0x00, 24, 2));
    out.physt = rx_bits(buf + 0x00, 26, 1) != 0;
    out.sw_decrypted = rx_bits(buf + 0x00, 27, 1) == 0;
    out.tid = static_cast<uint8_t>(rx_bits(buf + 0x04, 8, 4));
    out.amsdu = rx_bits(buf + 0x04, 13, 1) != 0;
    out.paggr = rx_bits(buf + 0x04, 15, 1) != 0;
    out.more_data = rx_bits(buf + 0x04, 26, 1) != 0;
    out.more_fragment = rx_bits(buf + 0x04, 27, 1) != 0;
    out.sequence = static_cast<uint16_t>(rx_bits(buf + 0x08, 0, 12));
    out.fragment = static_cast<uint8_t>(rx_bits(buf + 0x08, 12, 4));
    out.ppdu_count = static_cast<uint8_t>(rx_bits(buf + 0x08, 29, 2));
    out.rx_rate = static_cast<uint8_t>(rx_bits(buf + 0x0c, 0, 7));
    out.ampdu_eof = rx_bits(buf + 0x10, 8, 1) != 0;
    out.tsfl = rx_bits(buf + 0x14, 0, 32);
  }

  /* The vendor rxdesc2attribute path deliberately decodes only PKT_LEN and
   * C2H for firmware reports. Their payload begins immediately after the RX
   * descriptor; the remaining descriptor bits are not normal-frame metadata
   * and must not be allowed to move the body pointer. */
  const uint64_t frame_offset =
      static_cast<uint64_t>(kRxDescSize) +
      static_cast<uint32_t>(out.drvinfo_size) + out.shift;
  const uint64_t end = frame_offset + out.frame_len;
  if (out.frame_len == 0 || frame_offset > buflen || end > buflen ||
      end > (std::numeric_limits<uint32_t>::max)())
    return false;

  out.frame = buf + static_cast<size_t>(frame_offset);
  out.next_offset = (static_cast<uint32_t>(end) + 7u) & ~7u;
  return true;
}

/* RTL8733B uses Jaguar3 PHY status for OFDM/HT and its own type-6 CCK page.
 * Populate devourer's established raw signal convention: rssi = dBm + 110;
 * signed EVM/SNR retain the vendor's half-dB units. */
inline bool parse_phy_status_8733b(const uint8_t *physts, size_t physts_len,
                                   uint8_t rx_rate, uint8_t configured_bw,
                                   rx_pkt_attrib &attrib) {
  if (physts == nullptr)
    return false;
  if (rx_rate <= 3) {
    if (physts_len < 24)
      return false;
    const uint8_t rssi_msb = static_cast<uint8_t>((physts[3] >> 3) & 0x7);
    const int8_t power = static_cast<int8_t>((rssi_msb << 5) |
                                             (physts[11] >> 3));
    int encoded = static_cast<int>(power) + 110;
    if (encoded < 0)
      encoded = 0;
    if (encoded > 255)
      encoded = 255;
    attrib.rssi[0] = static_cast<uint8_t>(encoded);
    attrib.bw = 0;
    return true;
  }

  if (physts_len < 28)
    return false;
  const uint8_t page = physts[0] & 0x0f;
  if (page == 0 || page == 6)
    return false;
  attrib.rssi[0] = physts[1];
  const uint8_t flags = physts[7];
  attrib.ldpc = static_cast<uint8_t>((flags >> 5) & 1u);
  attrib.stbc = static_cast<uint8_t>((flags >> 6) & 1u);
  const uint8_t l_rxsc = physts[5] & 0x0f;
  const uint8_t ht_rxsc = (physts[5] >> 4) & 0x0f;
  const uint8_t rxsc = rx_rate <= 11 ? l_rxsc : ht_rxsc;
  /* phydm_rxsc_2_bw: RXSC 0 means the packet occupied the receiver's full
   * configured bandwidth. Legacy OFDM remains 20 MHz; the full-width sentinel
   * matters for HT, where an HT40 packet otherwise gets misreported as 20. */
  attrib.bw = rxsc == 0 && rx_rate >= 12
                  ? configured_bw
                  : rxsc >= 13 ? 2 : rxsc >= 9 ? 1 : 0;
  if (page == 1) {
    attrib.evm[0] = static_cast<int8_t>(physts[16]);
    attrib.cfo_tail = static_cast<int8_t>(physts[20]);
    attrib.snr[0] = static_cast<int8_t>(physts[24]);
  }
  return true;
}

} // namespace rtl8733b

#endif /* FRAME_PARSER_8733B_H */
