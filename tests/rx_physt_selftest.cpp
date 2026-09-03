/* Headless guard for the Jaguar3 RX-descriptor PHYST bit (DW0 bit 26,
 * "the drvinfo area of THIS frame holds a written PHY-status report").
 * Inside an A-MPDU the drvinfo space is reserved on every subframe
 * (RX_DRVINFO_SZ is a global register) but the PHY writes a report only
 * where this bit is set — parsing the reserved bytes anyway reads stale
 * garbage that can alias a valid page number, which contaminates the
 * RSSI/SNR tails. A bit-position or plumbing regression here fails ctest
 * instead of poisoning the RF EMAs. */
#include <cstdio>
#include <cstring>

#include "jaguar3/FrameParserJaguar3.h"

static int g_fail = 0;
#define CHECK(cond, ...)                                                       \
  do {                                                                         \
    if (!(cond)) {                                                             \
      ++g_fail;                                                                \
      std::printf("FAIL: " __VA_ARGS__);                                       \
      std::printf("\n");                                                       \
    }                                                                          \
  } while (0)

/* 24-byte descriptor + 32-byte drvinfo + a 60-byte PSDU. */
static constexpr uint32_t kDrvInfo = 32;
static constexpr uint32_t kFrameLen = 60;
static constexpr size_t kBufLen =
    jaguar3::RXDESC_SIZE_8822C + kDrvInfo + kFrameLen;

static void make_desc(uint8_t *buf, bool physt) {
  std::memset(buf, 0, kBufLen);
  /* DW0: PKT_LEN[13:0] = 60, DRV_INFO_SIZE[19:16] = 4 (units of 8 bytes),
   * SHIFT[25:24] = 0, PHYST = bit 26. */
  buf[0] = kFrameLen;
  buf[2] = kDrvInfo / 8;
  if (physt)
    buf[3] |= 0x04;
}

static void test_physt_bit_decoded() {
  uint8_t buf[kBufLen];
  jaguar3::Rx8822cFrame f;

  make_desc(buf, true);
  CHECK(jaguar3::parse_rx_8822c(buf, kBufLen, f), "physt=1 desc must parse");
  CHECK(f.physt, "PHYST set in DW0 bit 26 -> Rx8822cFrame.physt true");

  make_desc(buf, false);
  CHECK(jaguar3::parse_rx_8822c(buf, kBufLen, f), "physt=0 desc must parse");
  CHECK(!f.physt, "PHYST clear -> Rx8822cFrame.physt false");
}

int main() {
  test_physt_bit_decoded();
  if (g_fail == 0)
    std::printf("rx_physt_selftest: all checks passed\n");
  return g_fail == 0 ? 0 : 1;
}
