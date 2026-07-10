#pragma once

/* TxAggPlan — pure layout planning for USB bulk-OUT TX aggregation: packing N
 * [txdesc][frame] blocks into one URB so the whole burst costs one bulk
 * transfer (and lands in the TXDMA together, which is what lets the MAC see
 * back-to-back frames). Mirrors the vendor xmitframe_complete packing rules
 * (rtl8812au_xmit.c / rtl8822bu_xmit.c):
 *
 *  - each block starts 8-byte aligned (_RND8);
 *  - the FIRST descriptor carries the block count in dword7[31:24]
 *    (USB_TXAGG_NUM on Jaguar1, DMA_TXAGG_NUM on the HalMAC 88xx chips) —
 *    the caller sets that after building, then re-checksums (dword7 is inside
 *    the checksummed region on every generation);
 *  - the first block MAY carry an 8-byte reserve between its descriptor and
 *    frame, advertised via the descriptor PKT_OFFSET field (unit 8 bytes).
 *    Two per-family policies (lim.first_reserve):
 *      true  — Jaguar1 / vendor rtl8812au parity: the reserve is present on
 *              every aggregated transfer and removed exactly when the total
 *              would land on a bulk-MPS multiple (the vendor "usb bulk-out
 *              block check").
 *      false — HalMAC (8822B/8821C/J3) / mainline-rtw88 parity: no reserve
 *              (bench-proven on the 8822BU: the TXDMA block walker does NOT
 *              account PKT_OFFSET in its next-descriptor stride, so a
 *              reserved first block makes the chip re-air block 1
 *              DMA_TXAGG_NUM times). The reserve is ADDED only when the
 *              total would land on a bulk-MPS multiple (the sync bulk path
 *              has no ZLP; an exact multiple stalls the chip-side DMA);
 *  - the OQT guard: at most `descs_per_bulk` descriptors may START inside one
 *    bulk-size window (the vendor UsbTxAggDescNum / halmac BLK_DESC_NUM
 *    discipline — 8812A silicon overflows its output-queue tracker beyond 1
 *    per window, HalMAC chips take 3). Packing stops when the guard trips;
 *    the remainder goes into the next URB.
 *
 * Pure math, no I/O — unit-tested headless in tests/txagg_selftest.cpp. */

#include <cstddef>
#include <cstdint>
#include <vector>

namespace devourer {

struct TxAggLimits {
  size_t desc_size = 48;      /* TX descriptor bytes: 40 (Jaguar1) / 48 (J2/J3) */
  size_t bulk_size = 512;     /* USB bulk max packet: 64 FS / 512 HS / 1024 SS */
  size_t max_bytes = 20480;   /* URB buffer cap (vendor MAX_XMITBUF_SZ, USB) */
  unsigned max_frames = 255;  /* agg-num field is 8-bit; Jaguar1 caps at 64.
                               * HalMAC chips parse at most 3 descriptors per
                               * bulk transfer (rtw88 usb_tx_agg_desc_num /
                               * halmac BLK_DESC_NUM) — the caller clamps. */
  unsigned descs_per_bulk = 0; /* Jaguar1 OQT guard (descriptor STARTS per
                                * bulk window): 1 (8812A), 3 (8814A),
                                * 6 (8821A). 0 = off (HalMAC uses the flat
                                * max_frames cap instead). */
  bool first_reserve = false; /* first-block 8-byte PKT_OFFSET reserve policy
                               * (see the header comment). */
};

/* One packed block: `offset` is the descriptor start within the URB buffer;
 * `length` covers desc + (shim pad, first block only) + frame. */
struct TxAggBlock {
  size_t offset;
  size_t length;
};

struct TxAggPlan {
  std::vector<TxAggBlock> blocks; /* one per accepted input frame, in order */
  size_t total = 0;               /* final URB length (reserve included) */
  bool shim = false;              /* first desc gets PKT_OFFSET=1 (an 8-byte
                                   * pad between it and its frame) — presence
                                   * follows lim.first_reserve, flipped when
                                   * the total lands on a bulk multiple. */
  size_t frames() const { return blocks.size(); }
};

inline size_t txagg_rnd8(size_t v) { return (v + 7) & ~static_cast<size_t>(7); }

/* Greedily pack frames [0..n) (802.11 payload lengths, radiotap already
 * stripped) into ONE URB under `lim`. Returns the accepted prefix as a block
 * layout; frames() < n means the caller submits this URB and re-plans the
 * rest. An empty plan means frame 0 alone exceeds max_bytes (caller falls
 * back to the single-frame path, which has no cap). */
inline TxAggPlan plan_tx_agg(const size_t *frame_lens, size_t n,
                             const TxAggLimits &lim) {
  TxAggPlan p;
  if (n == 0 || lim.bulk_size == 0)
    return p;
  p.shim = lim.first_reserve;
  /* Reserve headroom so the boundary escape can ADD the reserve when the
   * policy starts without one. */
  const size_t cap =
      lim.first_reserve ? lim.max_bytes
                        : (lim.max_bytes > 8 ? lim.max_bytes - 8 : 0);
  size_t tail = 0;  /* end of last block (unaligned) */
  size_t next = 0;  /* aligned start of the next block */
  unsigned desc_count = 0;
  size_t bulk_ptr = lim.bulk_size; /* end of the bulk window being filled */
  for (size_t i = 0; i < n; ++i) {
    const size_t blk =
        lim.desc_size + frame_lens[i] + (p.blocks.empty() && p.shim ? 8 : 0);
    if (next + blk > cap)
      break;
    p.blocks.push_back({next, blk});
    tail = next + blk;
    next = txagg_rnd8(tail);
    if (p.blocks.size() >= lim.max_frames)
      break;
    /* Jaguar1 OQT guard, evaluated on the NEXT descriptor's start position:
     * another desc starting inside the current bulk window counts against
     * descs_per_bulk; crossing into a fresh window resets the count. */
    if (next < bulk_ptr) {
      if (lim.descs_per_bulk != 0 && ++desc_count >= lim.descs_per_bulk)
        break;
    } else {
      desc_count = 0;
      bulk_ptr = (next / lim.bulk_size + 1) * lim.bulk_size;
    }
  }
  p.total = tail;
  /* Never submit an exact bulk-size multiple (chip-side transfer-end
   * ambiguity): flip the first-block reserve — drop it when present, insert
   * it when absent. Either way every later block shifts by 8 and alignment
   * is preserved. */
  if (p.total != 0 && p.total % lim.bulk_size == 0) {
    const int d = p.shim ? -8 : 8;
    p.shim = !p.shim;
    p.blocks[0].length += d;
    for (size_t i = 1; i < p.blocks.size(); ++i)
      p.blocks[i].offset += d;
    p.total += d;
  }
  return p;
}

} /* namespace devourer */
