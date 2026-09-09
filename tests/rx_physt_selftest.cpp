/* Headless guard for the RX PHY-status gate on the Jaguar2 / Jaguar3 parsers.
 *
 * Two facts are pinned here, because the RF running averages behind
 * GetRxQuality() / GetActiveRxPaths() / the CFO trim are fed from them and a
 * regression in either is silent on air:
 *
 *  1. The RX-descriptor PHYST bit (DW0 bit 26) is decoded per frame. The
 *     drvinfo space is reserved on EVERY frame (REG_RX_DRVINFO_SZ is a global
 *     register) but the PHY writes a report only where this bit is set, so on
 *     all-but-one subframe of an A-MPDU the area holds bytes left by an earlier
 *     frame. A stale page nibble can alias a valid page number, so a parse
 *     "succeeds" on garbage and contaminates the RSSI/SNR/EVM/CFO tails.
 *
 *  2. parse_phy_sts_* reports WHICH fields it filled (PhyStsFill), not just
 *     whether it liked the buffer. A CCK page carries path-A power alone, and
 *     the per-stream EVM/SNR plus the CFO tail live on one OFDM page only —
 *     folding the untouched zeros drags the averages toward zero rather than
 *     leaving them alone. Losing the Full/Power distinction would stop all
 *     OFDM SNR/EVM accounting (or restart the zero-dilution) while ctest
 *     stayed green.
 */
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "jaguar2/FrameParserJaguar2.h"
#include "jaguar3/FrameParserJaguar3.h"

namespace {
int g_fail = 0;

void expect(const char *what, bool condition) {
  if (condition)
    return;
  ++g_fail;
  std::printf("FAIL: %s\n", what);
}

void set_bits(uint8_t *p, unsigned bit, unsigned width, uint32_t value) {
  uint32_t word = static_cast<uint32_t>(p[0]) |
                  (static_cast<uint32_t>(p[1]) << 8) |
                  (static_cast<uint32_t>(p[2]) << 16) |
                  (static_cast<uint32_t>(p[3]) << 24);
  const uint32_t mask = width == 32 ? 0xffffffffu
                                    : (((1u << width) - 1u) << bit);
  word = (word & ~mask) | ((value << bit) & mask);
  p[0] = static_cast<uint8_t>(word);
  p[1] = static_cast<uint8_t>(word >> 8);
  p[2] = static_cast<uint8_t>(word >> 16);
  p[3] = static_cast<uint8_t>(word >> 24);
}

/* 24-byte descriptor + 32-byte drvinfo + a 60-byte PSDU. drvinfo is carried in
 * 8-byte units, so 4 units = the 32-byte phy-status block. */
constexpr uint16_t kFrameLen = 60;
constexpr uint8_t kDrvInfoUnits = 4;

std::vector<uint8_t> desc(size_t desc_size, bool physt) {
  std::vector<uint8_t> b(desc_size + kDrvInfoUnits * 8u + kFrameLen, 0);
  set_bits(b.data(), 0, 14, kFrameLen);
  set_bits(b.data(), 16, 4, kDrvInfoUnits);
  set_bits(b.data(), 24, 2, 0);          /* SHIFT */
  set_bits(b.data(), 26, 1, physt ? 1 : 0);
  return b;
}

/* --- The descriptor bit is decoded, on both generations --- */
void test_physt_bit_decoded() {
  {
    jaguar3::Rx8822cFrame f{};
    std::vector<uint8_t> b = desc(jaguar3::RXDESC_SIZE_8822C, true);
    expect("jgr3 physt=1 descriptor parses",
           jaguar3::parse_rx_8822c(b.data(), b.size(), f));
    expect("jgr3 DW0 bit 26 set -> Rx8822cFrame.physt true", f.physt);

    b = desc(jaguar3::RXDESC_SIZE_8822C, false);
    expect("jgr3 physt=0 descriptor parses",
           jaguar3::parse_rx_8822c(b.data(), b.size(), f));
    expect("jgr3 DW0 bit 26 clear -> Rx8822cFrame.physt false", !f.physt);
  }
  {
    jaguar2::Rx8822bFrame f{};
    std::vector<uint8_t> b = desc(jaguar2::RXDESC_SIZE_8822B, true);
    expect("jgr2 physt=1 descriptor parses",
           jaguar2::parse_rx_8822b(b.data(), b.size(), f));
    expect("jgr2 DW0 bit 26 set -> Rx8822bFrame.physt true", f.physt);

    b = desc(jaguar2::RXDESC_SIZE_8822B, false);
    expect("jgr2 physt=0 descriptor parses",
           jaguar2::parse_rx_8822b(b.data(), b.size(), f));
    expect("jgr2 DW0 bit 26 clear -> Rx8822bFrame.physt false", !f.physt);
  }
}

/* A 32-byte jgr3 phy-status page: byte0 low nibble = page number, per-path
 * pwdb at 1..4, flags at 7, rxevm at 16..19, cfo_tail at 20, rxsnr at 24..27. */
std::vector<uint8_t> jgr3_page(uint8_t page_num) {
  std::vector<uint8_t> p(32, 0);
  p[0] = page_num;
  for (int i = 0; i < 4; i++)
    p[1 + i] = static_cast<uint8_t>(90 + i); /* pwdb -> rssi */
  p[5] = 0x11;                               /* l_rxsc / ht_rxsc = 1 -> 20 MHz */
  p[7] = 0x20;                               /* bit5 = ldpc */
  for (int i = 0; i < 4; i++) {
    p[16 + i] = static_cast<uint8_t>(0xE0 + i); /* rxevm (signed) */
    p[24 + i] = static_cast<uint8_t>(30 + i);   /* rxsnr */
  }
  p[20] = 7; /* path-A cfo_tail */
  return p;
}

void test_jgr3_fill_tiers() {
  /* Short / absent buffer fills nothing. */
  {
    rx_pkt_attrib a{};
    std::vector<uint8_t> p = jgr3_page(1);
    expect("jgr3 null buffer -> None",
           jaguar3::parse_phy_sts_jgr3(nullptr, 32, a) == PhyStsFill::None);
    expect("jgr3 27-byte report -> None",
           jaguar3::parse_phy_sts_jgr3(p.data(), 27, a) == PhyStsFill::None);
    expect("jgr3 rejected report leaves the attrib untouched",
           a.rssi[0] == 0 && a.snr[0] == 0 && a.cfo_tail == 0);
  }
  /* Page 0 (CCK): path-A power only. Claiming Full here would feed a zero SNR
   * and a zero CFO tail into the trackers. */
  {
    rx_pkt_attrib a{};
    std::vector<uint8_t> p = jgr3_page(0);
    p[1] = 88;
    expect("jgr3 page 0 (CCK) -> Power",
           jaguar3::parse_phy_sts_jgr3(p.data(), 32, a) == PhyStsFill::Power);
    expect("jgr3 page 0 fills path-A rssi", a.rssi[0] == 88);
    expect("jgr3 page 0 leaves snr/evm/cfo unset",
           a.snr[0] == 0 && a.evm[0] == 0 && a.cfo_tail == 0);
  }
  /* Page 1 (OFDM type1): the only page carrying EVM/SNR/CFO. */
  {
    rx_pkt_attrib a{};
    a.data_rate = 12; /* HT -> bandwidth read from ht_rxsc */
    std::vector<uint8_t> p = jgr3_page(1);
    expect("jgr3 page 1 (OFDM type1) -> Full",
           jaguar3::parse_phy_sts_jgr3(p.data(), 32, a) == PhyStsFill::Full);
    expect("jgr3 page 1 fills per-path rssi",
           a.rssi[0] == 90 && a.rssi[3] == 93);
    expect("jgr3 page 1 fills per-stream snr/evm",
           a.snr[0] == 30 && a.snr[3] == 33 &&
               a.evm[0] == static_cast<int8_t>(0xE0));
    expect("jgr3 page 1 fills the path-A cfo tail", a.cfo_tail == 7);
    expect("jgr3 page 1 fills ldpc/bw from the common header",
           a.ldpc == 1 && a.stbc == 0 && a.bw == 0);
  }
  /* Pages 2..6 share the OFDM common header, so their per-path power IS a
   * measurement — reporting None would throw it away — but the type1-only
   * fields stay zero and must not be folded. */
  {
    rx_pkt_attrib a{};
    std::vector<uint8_t> p = jgr3_page(5);
    expect("jgr3 page 5 (other OFDM page) -> Power",
           jaguar3::parse_phy_sts_jgr3(p.data(), 32, a) == PhyStsFill::Power);
    expect("jgr3 page 5 fills per-path rssi from the common header",
           a.rssi[0] == 90 && a.rssi[3] == 93);
    expect("jgr3 page 5 leaves snr/evm/cfo unset",
           a.snr[0] == 0 && a.evm[0] == 0 && a.cfo_tail == 0);
  }
}

void test_jgr2_fill_tiers() {
  {
    rx_pkt_attrib a{};
    std::vector<uint8_t> p = jgr3_page(1); /* same 32-byte field offsets */
    expect("jgr2 null buffer -> None",
           jaguar2::parse_phy_sts_jgr2(nullptr, 32, false, a) ==
               PhyStsFill::None);
    expect("jgr2 27-byte report -> None",
           jaguar2::parse_phy_sts_jgr2(p.data(), 27, false, a) ==
               PhyStsFill::None);
  }
  {
    rx_pkt_attrib a{};
    std::vector<uint8_t> p = jgr3_page(0);
    p[1] = 88;
    expect("jgr2 CCK type0 -> Power",
           jaguar2::parse_phy_sts_jgr2(p.data(), 32, true, a) ==
               PhyStsFill::Power);
    expect("jgr2 CCK fills path-A rssi only",
           a.rssi[0] == 88 && a.snr[0] == 0 && a.cfo_tail == 0);
  }
  {
    rx_pkt_attrib a{};
    a.data_rate = 12;
    std::vector<uint8_t> p = jgr3_page(1);
    expect("jgr2 OFDM type1 -> Full",
           jaguar2::parse_phy_sts_jgr2(p.data(), 32, false, a) ==
               PhyStsFill::Full);
    expect("jgr2 type1 fills rssi/snr/evm/cfo",
           a.rssi[0] == 90 && a.snr[0] == 30 &&
               a.evm[0] == static_cast<int8_t>(0xE0) && a.cfo_tail == 7);
  }
}
} // namespace

int main() {
  test_physt_bit_decoded();
  test_jgr3_fill_tiers();
  test_jgr2_fill_tiers();
  if (g_fail == 0)
    std::printf("rx_physt_selftest: all checks passed\n");
  return g_fail == 0 ? 0 : 1;
}
