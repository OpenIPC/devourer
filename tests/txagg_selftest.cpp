/* Headless guard for the USB TX-aggregation URB packing (src/TxAggPlan.h) and
 * the descriptor-level agg fields it drives:
 *   - plan_tx_agg layout math: 8-byte block alignment, the never-a-bulk-
 *     multiple shim, the max_frames / max_bytes / descs_per_bulk (OQT) stops;
 *   - the HalMAC descriptor plumbing: DMA_TXAGG_NUM lands at dword7[31:24]
 *     BEFORE the checksum, PKT_OFFSET moves the frame by 8 bytes, and the
 *     8822C checksum span extends over the shim pad (it reads PKT_OFFSET).
 * Prints the failing case and exits nonzero. */
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "TxAggPlan.h"
#if defined(DEVOURER_HAVE_JAGUAR2)
#include "jaguar2/FrameParserJaguar2.h"
#endif
#if defined(DEVOURER_HAVE_JAGUAR3)
#include "jaguar3/FrameParserJaguar3.h"
#endif

static int g_fail = 0;

#define CHECK(cond, ...)                                                       \
  do {                                                                         \
    if (!(cond)) {                                                             \
      ++g_fail;                                                                \
      std::printf("FAIL %s:%d: ", __FILE__, __LINE__);                         \
      std::printf(__VA_ARGS__);                                                \
      std::printf("\n");                                                       \
    }                                                                          \
  } while (0)

static devourer::TxAggLimits lim_halmac_hs() {
  devourer::TxAggLimits lim;
  lim.desc_size = 48;
  lim.bulk_size = 512;
  lim.max_bytes = 20480;
  lim.max_frames = 255;
  lim.descs_per_bulk = 3;
  return lim;
}

static void test_plan_basic() {
  /* HalMAC policy (rtw88 parity): NO first-block reserve — blocks at
   * 8-aligned offsets, stride = 48 + rnd8(frame). */
  const size_t lens[] = {1500, 1500, 1500};
  const auto p = devourer::plan_tx_agg(lens, 3, lim_halmac_hs());
  CHECK(p.frames() == 3, "basic: frames=%zu want 3", p.frames());
  size_t expect_off = 0;
  for (size_t i = 0; i < p.frames(); ++i) {
    CHECK(p.blocks[i].offset % 8 == 0, "basic: block %zu offset %zu not 8-aligned",
          i, p.blocks[i].offset);
    CHECK(p.blocks[i].offset == expect_off, "basic: block %zu offset %zu want %zu",
          i, p.blocks[i].offset, expect_off);
    CHECK(p.blocks[i].length == 48 + 1500, "basic: block %zu len %zu", i,
          p.blocks[i].length);
    expect_off = devourer::txagg_rnd8(p.blocks[i].offset + p.blocks[i].length);
  }
  CHECK(p.total == p.blocks[2].offset + p.blocks[2].length,
        "basic: total %zu mismatch", p.total);
  CHECK(!p.shim, "basic: unexpected reserve in halmac policy");
  CHECK(p.total % 512 != 0, "basic: total %zu on bulk boundary", p.total);

  /* Jaguar1 policy (vendor parity): reserve present by default. */
  auto lim1 = lim_halmac_hs();
  lim1.desc_size = 40;
  lim1.first_reserve = true;
  const auto q = devourer::plan_tx_agg(lens, 3, lim1);
  CHECK(q.frames() == 3, "basic-j1: frames=%zu", q.frames());
  CHECK(q.shim, "basic-j1: reserve missing");
  CHECK(q.blocks[0].length == 40 + 8 + 1500, "basic-j1: block0 len %zu",
        q.blocks[0].length);
  CHECK(q.blocks[1].length == 40 + 1500, "basic-j1: block1 len %zu",
        q.blocks[1].length);
  CHECK(q.total % 512 != 0, "basic-j1: total %zu on boundary", q.total);
}

static void test_plan_shim() {
  /* HalMAC policy: a 464-byte frame alone (48+464 = 512, exact multiple) ->
   * the escape INSERTS the reserve: shim=true, total 520. */
  const size_t lens1[] = {512 - 48};
  const auto p1 = devourer::plan_tx_agg(lens1, 1, lim_halmac_hs());
  CHECK(p1.frames() == 1, "shim1: frames=%zu", p1.frames());
  CHECK(p1.shim, "shim1: reserve not inserted at total==512");
  CHECK(p1.total == 520, "shim1: total=%zu want 520", p1.total);
  CHECK(p1.blocks[0].length == 48 + 8 + (512 - 48), "shim1: block0 len %zu",
        p1.blocks[0].length);

  /* Off-boundary frame: no reserve at all. */
  const size_t lens_off[] = {456};
  const auto po = devourer::plan_tx_agg(lens_off, 1, lim_halmac_hs());
  CHECK(!po.shim, "shim-off: unexpected reserve");
  CHECK(po.total == 504, "shim-off: total=%zu want 504", po.total);

  /* Jaguar1 policy removal: reserved total 40+8+464 = 512 -> escape DROPS
   * the reserve, total 504. */
  auto lim1 = lim_halmac_hs();
  lim1.desc_size = 40;
  lim1.first_reserve = true;
  const size_t lens_rm[] = {464};
  const auto pr = devourer::plan_tx_agg(lens_rm, 1, lim1);
  CHECK(pr.frames() == 1, "shim-rm: frames=%zu", pr.frames());
  CHECK(!pr.shim, "shim-rm: reserve not removed at reserved-total==512");
  CHECK(pr.total == 504, "shim-rm: total=%zu want 504", pr.total);
  CHECK(pr.blocks[0].length == 40 + 464, "shim-rm: block0 len %zu",
        pr.blocks[0].length);

  /* Sweep both policies: no plan may ever total an exact bulk multiple. */
  for (int reserve = 0; reserve < 2; ++reserve) {
    auto lim = lim_halmac_hs();
    lim.first_reserve = reserve != 0;
    for (size_t flen = 1; flen < 1600; ++flen) {
      const size_t lens[] = {flen, flen, flen, flen};
      const auto p = devourer::plan_tx_agg(lens, 4, lim);
      if (p.frames() != 0)
        CHECK(p.total % 512 != 0, "sweep(res=%d): flen=%zu total=%zu on boundary",
              reserve, flen, p.total);
    }
  }
}

static void test_plan_oqt_guard() {
  /* 8812A-style descs_per_bulk=1 (Jaguar1 policy): a second small block would
   * start inside the first 512-byte window -> packing stops at 1 frame. */
  auto lim = lim_halmac_hs();
  lim.desc_size = 40;
  lim.descs_per_bulk = 1;
  lim.first_reserve = true;
  const size_t small[] = {100, 100, 100, 100};
  const auto p1 = devourer::plan_tx_agg(small, 4, lim);
  CHECK(p1.frames() == 1, "oqt1: frames=%zu want 1", p1.frames());

  /* Large frames span whole windows, so the guard never trips: full pack. */
  const size_t big[] = {1500, 1500, 1500, 1500};
  const auto p2 = devourer::plan_tx_agg(big, 4, lim);
  CHECK(p2.frames() == 4, "oqt2: frames=%zu want 4", p2.frames());

  /* HalMAC flat cap: max_frames clamps regardless of frame size (the chip
   * parses at most 3 descriptors per bulk transfer). */
  auto lim3 = lim_halmac_hs();
  lim3.max_frames = 3;
  const size_t tiny[] = {40, 40, 40, 40, 40, 40, 40, 40};
  const auto p3 = devourer::plan_tx_agg(tiny, 8, lim3);
  CHECK(p3.frames() == 3, "oqt3: frames=%zu want 3", p3.frames());
}

static void test_plan_caps() {
  auto lim = lim_halmac_hs();
  lim.max_frames = 2;
  const size_t lens[] = {1000, 1000, 1000};
  const auto p1 = devourer::plan_tx_agg(lens, 3, lim);
  CHECK(p1.frames() == 2, "cap-frames: %zu want 2", p1.frames());

  auto lim2 = lim_halmac_hs();
  lim2.max_bytes = 2200; /* two 1048-byte blocks + shim headroom only */
  const auto p2 = devourer::plan_tx_agg(lens, 3, lim2);
  CHECK(p2.frames() == 2, "cap-bytes: %zu want 2", p2.frames());

  /* First frame alone over the cap -> empty plan (caller falls back). */
  auto lim3 = lim_halmac_hs();
  lim3.max_bytes = 512;
  const size_t huge[] = {4000};
  const auto p3 = devourer::plan_tx_agg(huge, 1, lim3);
  CHECK(p3.frames() == 0, "cap-first: %zu want 0", p3.frames());
}

#if defined(DEVOURER_HAVE_JAGUAR2)
/* XOR checksum over the first 32 bytes, the 8822B hardware rule. */
static uint16_t xor32(const uint8_t *d) {
  uint16_t x = 0;
  for (int i = 0; i < 16; ++i)
    x = static_cast<uint16_t>(x ^ (d[2 * i] | (d[2 * i + 1] << 8)));
  return x;
}

static void test_desc_8822b() {
  uint8_t d[48];

  /* Baseline descriptor, then the packer's first-descriptor fixup: set
   * DMA_TXAGG_NUM and re-checksum. The stored checksum must equal the XOR of
   * the other 15 words (i.e. total XOR == 0), and the agg count must sit in
   * byte 0x1F. */
  std::memset(d, 0, sizeof(d));
  jaguar2::fill_data_tx_desc_8822b(d, 1500, 7, 9, 0, false, false, 0);
  SET_TX_DESC_DMA_TXAGG_NUM_8822B(d, 5);
  jaguar2::cal_txdesc_chksum_8822b(d);
  CHECK(d[0x1F] == 5, "8822b: agg num byte 0x1F=%u want 5", d[0x1F]);
  CHECK(xor32(d) == 0, "8822b: checksum does not cover the agg field");

  /* pkt_offset=1: PKT_OFFSET field set (dword1[28:24] -> byte 0x07 low bits)
   * and still checksummed. */
  std::memset(d, 0, sizeof(d));
  jaguar2::fill_data_tx_desc_8822b(d, 1500, 7, 9, 0, false, false, 0,
                                   /*bmc=*/false, /*wheader_len=*/12,
                                   /*ndpa=*/false, /*data_sc=*/0,
                                   /*pwr_ofset=*/0, /*pkt_offset=*/1);
  CHECK((d[0x07] & 0x1f) == 1, "8822b: pkt_offset field=%u want 1",
        d[0x07] & 0x1f);
  CHECK(xor32(d) == 0, "8822b: checksum stale after pkt_offset");

  /* Re-checksum idempotence: the packer re-runs it after the agg fixup. */
  const uint16_t before = static_cast<uint16_t>(d[0x1C] | (d[0x1D] << 8));
  jaguar2::cal_txdesc_chksum_8822b(d);
  const uint16_t after = static_cast<uint16_t>(d[0x1C] | (d[0x1D] << 8));
  CHECK(before == after, "8822b: checksum not idempotent (%04x -> %04x)",
        before, after);
}
#endif /* DEVOURER_HAVE_JAGUAR2 */

#if defined(DEVOURER_HAVE_JAGUAR3)
static void test_desc_8822c() {
  /* The 8822C checksum span extends by PKT_OFFSET (the hardware checksums
   * desc + pad): with pkt_offset=1 a flipped PAD byte must change the
   * checksum; with pkt_offset=0 a byte at the same position (frame area) must
   * not. Model the packer's zeroed block: 48-byte desc + 8-byte pad. */
  uint8_t blk[64];

  std::memset(blk, 0, sizeof(blk));
  jaguar3::fill_data_tx_desc_8822c(blk, 1500, 7, 9, 0, false, false, 0,
                                   /*bmc=*/false, /*ndpa=*/false,
                                   /*data_sc=*/0, /*pwr_ofset_type=*/0,
                                   /*pkt_offset=*/1);
  CHECK((blk[0x07] & 0x1f) == 1, "8822c: pkt_offset field=%u want 1",
        blk[0x07] & 0x1f);
  const uint16_t ck_pad0 = static_cast<uint16_t>(blk[0x1C] | (blk[0x1D] << 8));
  blk[48] = 0xA5; /* corrupt the pad the HW would checksum */
  jaguar3::cal_txdesc_chksum_8822c(blk);
  const uint16_t ck_pad1 = static_cast<uint16_t>(blk[0x1C] | (blk[0x1D] << 8));
  CHECK(ck_pad0 != ck_pad1, "8822c: checksum span misses the pkt_offset pad");

  std::memset(blk, 0, sizeof(blk));
  jaguar3::fill_data_tx_desc_8822c(blk, 1500, 7, 9, 0, false, false, 0);
  const uint16_t ck0 = static_cast<uint16_t>(blk[0x1C] | (blk[0x1D] << 8));
  blk[48] = 0xA5; /* first frame byte — outside the 48-byte span */
  jaguar3::cal_txdesc_chksum_8822c(blk);
  const uint16_t ck1 = static_cast<uint16_t>(blk[0x1C] | (blk[0x1D] << 8));
  CHECK(ck0 == ck1, "8822c: pkt_offset=0 checksum leaked past the descriptor");

  /* Agg-num placement mirrors the 8822B. */
  std::memset(blk, 0, sizeof(blk));
  jaguar3::fill_data_tx_desc_8822c(blk, 1500, 7, 9, 0, false, false, 0);
  SET_TX_DESC_DMA_TXAGG_NUM_8822C(blk, 9);
  jaguar3::cal_txdesc_chksum_8822c(blk);
  CHECK(blk[0x1F] == 9, "8822c: agg num byte 0x1F=%u want 9", blk[0x1F]);
}
#endif /* DEVOURER_HAVE_JAGUAR3 */

int main() {
  test_plan_basic();
  test_plan_shim();
  test_plan_oqt_guard();
  test_plan_caps();
#if defined(DEVOURER_HAVE_JAGUAR2)
  test_desc_8822b();
#endif
#if defined(DEVOURER_HAVE_JAGUAR3)
  test_desc_8822c();
#endif
  if (g_fail) {
    std::printf("txagg selftest: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("txagg selftest: OK\n");
  return 0;
}
