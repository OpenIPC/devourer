/* Headless guard for the Jaguar3 per-packet TX-power bank planner
 * (jaguar3::TxPktPwrBankPlanner) and the descriptor-byte layout
 * of the per-packet power fields on Jaguar3 (TXPWR_OFSET_TYPE, 0x14[29:28])
 * and the 8814A (TX_POWER_OFFSET, 0x14[30:28]). A policy or bit-position
 * regression fails ctest instead of surfacing as wrong on-air power. */
#include <cstdio>
#include <cstring>

#include "jaguar3/TxPktPwrBanks.h"
#if defined(DEVOURER_HAVE_JAGUAR3)
#include "jaguar3/FrameParserJaguar3.h"
#endif
#if defined(DEVOURER_HAVE_JAGUAR1)
#include "basic_types.h" /* jaguar1/FrameParser.h expects the LE macros pre-included */
#include "jaguar1/FrameParser.h"
#endif

static int g_fail = 0;
#define CHECK(cond, ...)                                                       \
  do {                                                                         \
    if (!(cond)) {                                                             \
      ++g_fail;                                                                \
      std::printf("FAIL: " __VA_ARGS__);                                       \
      std::printf("\n");                                                       \
    }                                                                          \
  } while (0)

static void test_planner_policy() {
  using jaguar3::TxPktPwrBankPlanner;
  TxPktPwrBankPlanner pl;

  /* Zero -> type 0, never a write, banks untouched. */
  auto p0 = pl.request(0);
  CHECK(p0.type == 0 && !p0.program, "request(0) -> type %u program %d",
        p0.type, p0.program);
  CHECK(pl.reg16() == 0 && !pl.active(), "empty planner not idle");

  /* First non-zero value programs bank 2 ([23:16] half: en bit7 + s7 idx). */
  auto p1 = pl.request(4);
  CHECK(p1.type == 2 && p1.program, "first value -> type %u program %d",
        p1.type, p1.program);
  CHECK((p1.reg16 & 0xff) == (0x80 | 4), "bank2 byte 0x%02x want 0x84",
        p1.reg16 & 0xff);
  CHECK((p1.reg16 >> 8) == 0, "bank3 byte polluted: 0x%02x", p1.reg16 >> 8);

  /* Repeat -> same type, no reprogram. */
  auto p2 = pl.request(4);
  CHECK(p2.type == 2 && !p2.program, "repeat reprogrammed (type %u prog %d)",
        p2.type, p2.program);

  /* Second distinct value -> bank 3; negative encodes s7 two's-complement. */
  auto p3 = pl.request(-4);
  CHECK(p3.type == 3 && p3.program, "second value -> type %u program %d",
        p3.type, p3.program);
  CHECK((p3.reg16 >> 8) == (0x80 | 0x7c), "bank3 byte 0x%02x want 0xfc (-4 s7)",
        p3.reg16 >> 8);
  CHECK((p3.reg16 & 0xff) == (0x80 | 4), "bank2 byte lost on bank3 program");

  /* Third distinct value evicts the LRU bank — bank2 (value 4) was touched
   * less recently than bank3 (-4). */
  auto p4 = pl.request(12);
  CHECK(p4.type == 2 && p4.program, "third value -> type %u program %d",
        p4.type, p4.program);
  CHECK((p4.reg16 & 0xff) == (0x80 | 12), "evicted bank2 byte 0x%02x",
        p4.reg16 & 0xff);
  CHECK((p4.reg16 >> 8) == (0x80 | 0x7c), "bank3 byte lost on eviction");

  /* Touch bank2 (12), then a fourth value must evict bank3 this time. */
  (void)pl.request(12);
  auto p5 = pl.request(-20);
  CHECK(p5.type == 3 && p5.program, "fourth value -> type %u", p5.type);
  CHECK((p5.reg16 >> 8) == (0x80 | ((-20) & 0x7f)),
        "bank3 byte 0x%02x want 0x%02x", p5.reg16 >> 8, 0x80 | ((-20) & 0x7f));

  /* Clamp: the 7-bit two's-complement travel is [-64, +63]. */
  CHECK(TxPktPwrBankPlanner::clamp_idx(200) == 63, "clamp +");
  CHECK(TxPktPwrBankPlanner::clamp_idx(-200) == -64, "clamp -");
  auto p6 = pl.request(63);
  CHECK((p6.reg16 & 0xff) == (0x80 | 63) || (p6.reg16 >> 8) == (0x80 | 63),
        "max idx not programmed");

  /* reset() forgets both banks. */
  pl.reset();
  CHECK(pl.reg16() == 0 && !pl.active(), "reset left state");
}

#if defined(DEVOURER_HAVE_JAGUAR3)
static void test_desc_8822c_type_bits() {
  /* TXPWR_OFSET_TYPE lives at dword5 (byte 0x14) bits [29:28] = byte 0x17
   * bits [5:4]. Bit 30 (byte 0x17 bit 6) is ANTSEL_EN_V1 on this generation
   * and must NEVER be set by the 2-bit macro — an earlier sweep wrote a
   * 3-bit 8822B-style value and silently enabled antenna-select. */
  uint8_t d[48];
  for (uint8_t type = 0; type < 4; type++) {
    std::memset(d, 0, sizeof(d));
    jaguar3::fill_data_tx_desc_8822c(d, 1500, 7, 9, 0, false, false, 0,
                                     /*bmc=*/false, /*ndpa=*/false,
                                     /*data_sc=*/0, /*pwr_ofset_type=*/type);
    CHECK(((d[0x17] >> 4) & 0x3) == type, "type %u not at 0x14[29:28]", type);
    CHECK((d[0x17] & 0x40) == 0, "type %u leaked into bit30 (ANTSEL_EN_V1)",
          type);
  }

  /* type=0 output must be byte-identical to a fill without the argument —
   * the "feature unused" path keeps today's descriptors unchanged. */
  uint8_t a[48], b[48];
  std::memset(a, 0, sizeof(a));
  std::memset(b, 0, sizeof(b));
  jaguar3::fill_data_tx_desc_8822c(a, 1500, 7, 9, 0, false, false, 0);
  jaguar3::fill_data_tx_desc_8822c(b, 1500, 7, 9, 0, false, false, 0,
                                   /*bmc=*/false, /*ndpa=*/false,
                                   /*data_sc=*/0, /*pwr_ofset_type=*/0);
  CHECK(std::memcmp(a, b, sizeof(a)) == 0, "type=0 fill not byte-identical");

  /* The type is inside the checksummed span: flipping it must change the
   * checksum the fill computed (i.e. it was set BEFORE the checksum). */
  std::memset(a, 0, sizeof(a));
  std::memset(b, 0, sizeof(b));
  jaguar3::fill_data_tx_desc_8822c(a, 1500, 7, 9, 0, false, false, 0,
                                   false, false, 0, /*pwr_ofset_type=*/0);
  jaguar3::fill_data_tx_desc_8822c(b, 1500, 7, 9, 0, false, false, 0,
                                   false, false, 0, /*pwr_ofset_type=*/3);
  const uint16_t ck_a = static_cast<uint16_t>(a[0x1C] | (a[0x1D] << 8));
  const uint16_t ck_b = static_cast<uint16_t>(b[0x1C] | (b[0x1D] << 8));
  CHECK(ck_a != ck_b, "checksum ignores TXPWR_OFSET_TYPE (set after chksum?)");
}
#endif /* DEVOURER_HAVE_JAGUAR3 */

#if defined(DEVOURER_HAVE_JAGUAR1)
static void test_desc_8814a_offset_bits() {
  /* SET_TX_DESC_TX_POWER_OFFSET_8814A = dword5 (byte 0x14) [30:28] = byte
   * 0x17 bits [6:4] — the 8822B TXPWR_OFSET position (vendor
   * rtl8814a_xmit.h:207). */
  uint8_t d[40];
  std::memset(d, 0, sizeof(d));
  SET_TX_DESC_TX_POWER_OFFSET_8814A(d, 5);
  CHECK(((d[0x17] >> 4) & 0x7) == 5, "8814A step 5 not at 0x14[30:28]");
  CHECK(d[0x14] == 0 && d[0x15] == 0 && d[0x16] == 0,
        "8814A write leaked outside byte 0x17");
  /* Within the Jaguar1 32-byte checksummed span (dword5 < 32B) — position
   * check only; rtl8812a_cal_txdesc_chksum runs after all dword writes in
   * build_tx_block. */
}
#endif /* DEVOURER_HAVE_JAGUAR1 */

int main() {
  test_planner_policy();
#if defined(DEVOURER_HAVE_JAGUAR3)
  test_desc_8822c_type_bits();
#endif
#if defined(DEVOURER_HAVE_JAGUAR1)
  test_desc_8814a_offset_bits();
#endif
  if (g_fail) {
    std::printf("%d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("txpkt bank/descriptor selftest: all OK\n");
  return 0;
}
