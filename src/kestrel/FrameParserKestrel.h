#ifndef FRAME_PARSER_KESTREL_H
#define FRAME_PARSER_KESTREL_H

#include <cstddef>
#include <cstdint>

/* 11ax (Kestrel / G6 mac_ax) RX descriptor parser. Transcribed from
 * reference/rtl8852bu phl/hal_g6/mac/rxdesc.h + type.h (rxd_short_t 16B /
 * rxd_long_t 32B) and the 8852bu RX-meta parse. Self-contained + header-only so
 * it unit-tests without hardware; the RtlKestrelDevice RX loop walks the
 * bulk-IN aggregate with it once M2a (MAC RX bring-up) lands.
 *
 * A bulk-IN transfer is an aggregate of sub-packets, each:
 *   [rxd (16 or 32 B)] [drv_info (drv_info_size*8 B)] [shift pad] [payload]
 * where payload is `rpkt_len` bytes; the next sub-packet starts 8-byte aligned.
 * rpkt_type routes the payload: WIFI (802.11 frame), PPDU (per-frame PHY
 * status), CH_INFO, or C2H (firmware message). */

namespace kestrel {

/* rpkt_type values (rxdesc.h). WIFI is 0 (the default data path). */
constexpr uint8_t RPKT_TYPE_WIFI = 0;
constexpr uint8_t RPKT_TYPE_PPDU = 1;
constexpr uint8_t RPKT_TYPE_CH_INFO = 2;
constexpr uint8_t RPKT_TYPE_C2H = 10;

struct KestrelRxFrame {
  const uint8_t *payload = nullptr; /* start of the sub-packet body */
  uint32_t payload_len = 0;         /* rpkt_len */
  uint8_t rpkt_type = 0;            /* RPKT_TYPE_* */
  uint16_t drvinfo_size = 0;       /* bytes (drv_info_size field * 8) */
  uint8_t shift = 0;
  bool long_rxd = false;
  bool mac_info_vld = false;
  bool crc_err = false;
  bool icv_err = false;
  bool a1_match = false;
  uint16_t rx_rate = 0; /* 9-bit datarate code (legacy/HT/VHT/HE) */
  uint8_t gi_ltf = 0;
  uint8_t bw = 0;       /* 0=20 1=40 2=80 */
  uint8_t ppdu_cnt = 0;
  uint32_t freerun_cnt = 0; /* free-running counter (TSF-ish) */
  uint32_t next_offset = 0; /* offset of the next sub-packet in the aggregate */
};

namespace detail {
inline uint32_t rd_le32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}
} /* namespace detail */

/* Parse one sub-packet at `buf` (with `buflen` bytes remaining). Returns false
 * when the buffer is too short or the descriptor is malformed (caller stops
 * walking the aggregate). On success `out.next_offset` is where the next
 * sub-packet begins. For non-WIFI rpkt_type the payload is the raw report/C2H
 * bytes; the caller routes on `out.rpkt_type`. */
inline bool parse_rx_8852b(const uint8_t *buf, size_t buflen,
                           KestrelRxFrame &out) {
  if (buf == nullptr || buflen < 16)
    return false;

  const uint32_t d0 = detail::rd_le32(buf);
  out.payload_len = d0 & 0x3fff;             /* rpkt_len [13:0] */
  out.shift = static_cast<uint8_t>((d0 >> 14) & 0x3);
  out.mac_info_vld = (d0 & (1u << 23)) != 0;
  out.rpkt_type = static_cast<uint8_t>((d0 >> 24) & 0xf);
  out.drvinfo_size = static_cast<uint16_t>(((d0 >> 28) & 0x7) * 8);
  out.long_rxd = (d0 & (1u << 31)) != 0;
  const uint32_t rxd_len = out.long_rxd ? 32u : 16u;
  if (buflen < rxd_len)
    return false;

  const uint32_t d1 = detail::rd_le32(buf + 4);
  out.ppdu_cnt = static_cast<uint8_t>((d1 >> 4) & 0x7);
  out.rx_rate = static_cast<uint16_t>((d1 >> 16) & 0x1ff);
  out.gi_ltf = static_cast<uint8_t>((d1 >> 25) & 0x7);
  out.bw = static_cast<uint8_t>((d1 >> 30) & 0x3);

  out.freerun_cnt = detail::rd_le32(buf + 8);

  const uint32_t d3 = detail::rd_le32(buf + 12);
  out.a1_match = (d3 & (1u << 0)) != 0;
  out.crc_err = (d3 & (1u << 9)) != 0;
  out.icv_err = (d3 & (1u << 10)) != 0;

  const uint32_t frame_off = rxd_len + out.drvinfo_size + out.shift;
  const uint64_t end =
      static_cast<uint64_t>(frame_off) + out.payload_len;
  if (out.payload_len == 0 || end > buflen)
    return false;

  out.payload = buf + frame_off;
  out.next_offset = (static_cast<uint32_t>(end) + 7) & ~0x7u;
  return true;
}

} /* namespace kestrel */

#endif /* FRAME_PARSER_KESTREL_H */
