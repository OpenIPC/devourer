/* Headless regression guard for the Kestrel (11ax / G6 mac_ax) RX descriptor
 * parser (src/kestrel/FrameParserKestrel.h): field extraction from the
 * rxd_short (16B) / rxd_long (32B) descriptors, the sub-packet layout
 * (rxd + drv_info + shift + payload), rpkt_type routing, and the 8-byte-aligned
 * next-offset stride. Pure byte-level parsing — no hardware. On-air decode is
 * validated separately once the MAC RX bring-up (M2a) lands. */
#include <array>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "kestrel/FrameParserKestrel.h"

using namespace kestrel;

static int failures = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      failures++;                                                              \
    }                                                                          \
  } while (0)

static void put_le32(std::vector<uint8_t> &b, size_t off, uint32_t v) {
  b[off + 0] = v & 0xFF;
  b[off + 1] = (v >> 8) & 0xFF;
  b[off + 2] = (v >> 16) & 0xFF;
  b[off + 3] = (v >> 24) & 0xFF;
}

/* Build one sub-packet descriptor. */
static void put_desc(std::vector<uint8_t> &b, size_t off, uint32_t rpkt_len,
                     uint8_t shift, bool mac_info_vld, uint8_t rpkt_type,
                     uint8_t drv_info_sz3, bool long_rxd, uint16_t rate,
                     uint8_t bw, bool crc, bool icv, uint8_t ppdu_type = 0xf) {
  uint32_t d0 = (rpkt_len & 0x3fff) | (uint32_t(shift & 0x3) << 14) |
                (mac_info_vld ? (1u << 23) : 0) |
                (uint32_t(rpkt_type & 0xf) << 24) |
                (uint32_t(drv_info_sz3 & 0x7) << 28) |
                (long_rxd ? (1u << 31) : 0);
  uint32_t d1 = uint32_t(ppdu_type & 0xf) | (uint32_t(rate & 0x1ff) << 16) |
                (uint32_t(bw & 0x3) << 30);
  uint32_t d3 = (crc ? (1u << 9) : 0) | (icv ? (1u << 10) : 0);
  put_le32(b, off + 0, d0);
  put_le32(b, off + 4, d1);
  put_le32(b, off + 8, 0x11223344); /* freerun_cnt */
  put_le32(b, off + 12, d3);
}

int main() {
  /* --- short descriptor, no drvinfo, no shift, a 100-byte WIFI frame --- */
  {
    std::vector<uint8_t> b(16 + 100, 0xAB);
    put_desc(b, 0, /*len=*/100, /*shift=*/0, /*macvld=*/false, RPKT_TYPE_WIFI,
             /*drv3=*/0, /*long=*/false, /*rate=*/0x0b, /*bw=*/0, false, false);
    KestrelRxFrame f;
    CHECK(parse_rx_8852b(b.data(), b.size(), f));
    CHECK(f.payload_len == 100);
    CHECK(f.rpkt_type == RPKT_TYPE_WIFI);
    CHECK(!f.long_rxd && f.drvinfo_size == 0 && f.shift == 0);
    CHECK(f.rx_rate == 0x0b);
    CHECK(f.payload == b.data() + 16);
    CHECK(f.freerun_cnt == 0x11223344);
    CHECK(f.next_offset == ((16 + 100 + 7) & ~7u)); /* = 120 */
  }

  /* --- long descriptor + drvinfo(2*8=16) + shift(2 -> 4 B) + mac_info(4 B),
   *     CRC error flagged. Payload offset follows the vendor
   *     hal_trx_8852b.c formula: rxd + drvinfo + shift*2 + 4 (mac_info_vld
   *     on a non-PPDU-status packet). --- */
  {
    const uint32_t len = 40;
    const uint32_t off = 32 + 16 + 2 * 2 + 4;
    std::vector<uint8_t> b(off + len, 0);
    put_desc(b, 0, len, /*shift=*/2, /*macvld=*/true, RPKT_TYPE_WIFI,
             /*drv3=*/2, /*long=*/true, /*rate=*/0x100, /*bw=*/2, /*crc=*/true,
             /*icv=*/false, /*ppdu_type=*/8 /* RX_DESC_PPDU_T_HE_ERSU */);
    KestrelRxFrame f;
    CHECK(parse_rx_8852b(b.data(), b.size(), f));
    CHECK(f.long_rxd && f.drvinfo_size == 16 && f.shift == 2);
    CHECK(f.mac_info_vld);
    CHECK(f.crc_err && !f.icv_err);
    CHECK(f.rx_rate == 0x100); /* 9-bit HE rate code */
    CHECK(f.bw == 2);          /* 80 MHz */
    CHECK(f.ppdu_type == 8);   /* dword1[3:0] -> HE_ERSU classification */
    CHECK(f.payload == b.data() + off);
    CHECK(f.payload_len == len);
    CHECK(f.next_offset == ((off + len + 7) & ~7u));
  }

  /* --- 8852C drv_info unit is 16 bytes (RX_DESC_DRV_INFO_UNIT_8852C), and
   *     PPDU-status packets get NO mac_info offset even with mac_info_vld ---
   */
  {
    const uint32_t len = 24;
    const uint32_t off = 16 + 2 * 16; /* short rxd + drvinfo 2*16, no +4 */
    std::vector<uint8_t> b(off + len, 0);
    put_desc(b, 0, len, /*shift=*/0, /*macvld=*/true, RPKT_TYPE_PPDU,
             /*drv3=*/2, /*long=*/false, /*rate=*/0, /*bw=*/0, false, false);
    KestrelRxFrame f;
    CHECK(parse_rx_8852b(b.data(), b.size(), f, /*drv_info_unit=*/16));
    CHECK(f.drvinfo_size == 32);
    CHECK(f.payload == b.data() + off);
    CHECK(f.next_offset == ((off + len + 7) & ~7u));
  }

  /* --- C2H sub-packet is routed by rpkt_type, not treated as a frame --- */
  {
    std::vector<uint8_t> b(16 + 24, 0);
    put_desc(b, 0, 24, 0, false, RPKT_TYPE_C2H, 0, false, 0, 0, false, false);
    KestrelRxFrame f;
    CHECK(parse_rx_8852b(b.data(), b.size(), f));
    CHECK(f.rpkt_type == RPKT_TYPE_C2H);
    CHECK(f.payload_len == 24);
  }

  /* --- walk an aggregate of two WIFI frames via next_offset --- */
  {
    std::vector<uint8_t> b;
    b.resize(120, 0); /* frame0: 16 desc + 100 payload -> next 120 */
    put_desc(b, 0, 100, 0, false, RPKT_TYPE_WIFI, 0, false, 6, 0, false, false);
    const size_t base = 120;
    b.resize(base + 16 + 30, 0);
    put_desc(b, base, 30, 0, false, RPKT_TYPE_WIFI, 0, false, 7, 0, false,
             false);
    uint32_t off = 0;
    int count = 0;
    while (off + 16 <= b.size()) {
      KestrelRxFrame f;
      if (!parse_rx_8852b(b.data() + off, b.size() - off, f))
        break;
      ++count;
      off += f.next_offset;
    }
    CHECK(count == 2);
  }

  /* --- truncated buffers are rejected, not read out of bounds --- */
  {
    std::vector<uint8_t> b(16 + 100, 0);
    put_desc(b, 0, 100, 0, false, RPKT_TYPE_WIFI, 0, false, 0, 0, false, false);
    KestrelRxFrame f;
    CHECK(!parse_rx_8852b(b.data(), 50, f)); /* claims 100 but only 50 given */
    CHECK(!parse_rx_8852b(b.data(), 8, f));  /* shorter than a short desc */
  }

  if (failures == 0)
    std::printf("kestrel_rxparse_selftest: all checks passed\n");
  return failures == 0 ? 0 : 1;
}
