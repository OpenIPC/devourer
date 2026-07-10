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
 *  - the final URB length must never be an exact multiple of the bulk max
 *    packet size (the chip-side DMA would wait for more data / a ZLP). The
 *    vendor guarantees this with an 8-byte shim between the first descriptor
 *    and its frame, advertised via the descriptor PKT_OFFSET field (unit
 *    8 bytes) and inserted/removed depending on the final total. We plan the
 *    layout up-front (all lengths are known), so the shim decision is made
 *    once, deterministically;
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
  unsigned max_frames = 255;  /* agg-num field is 8-bit; Jaguar1 caps at 64 */
  unsigned descs_per_bulk = 3; /* OQT guard: 1 (8812A), 3 (8814A/HalMAC), 6 (8821A) */
};

/* One packed block: `offset` is the descriptor start within the URB buffer;
 * `length` covers desc + (shim pad, first block only) + frame. */
struct TxAggBlock {
  size_t offset;
  size_t length;
};

struct TxAggPlan {
  std::vector<TxAggBlock> blocks; /* one per accepted input frame, in order */
  size_t total = 0;               /* final URB length (shim included) */
  bool shim = false;              /* first desc gets PKT_OFFSET=1: an 8-byte pad
                                   * between it and its frame */
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
  /* Keep 8 bytes of headroom for the shim the finalize step may add. */
  const size_t cap = lim.max_bytes > 8 ? lim.max_bytes - 8 : 0;
  size_t tail = 0;  /* end of last block (unaligned) */
  size_t next = 0;  /* aligned start of the next block */
  unsigned desc_count = 0;
  size_t bulk_ptr = lim.bulk_size; /* end of the bulk window being filled */
  for (size_t i = 0; i < n; ++i) {
    const size_t blk = lim.desc_size + frame_lens[i];
    if (next + blk > cap)
      break;
    p.blocks.push_back({next, blk});
    tail = next + blk;
    next = txagg_rnd8(tail);
    if (p.blocks.size() >= lim.max_frames)
      break;
    /* Vendor OQT guard, evaluated on the NEXT descriptor's start position:
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
   * ambiguity): shim the first block — 8 bytes between desc and frame,
   * PKT_OFFSET=1 — shifting every later block by 8 (alignment preserved). */
  if (p.total != 0 && p.total % lim.bulk_size == 0) {
    p.shim = true;
    p.blocks[0].length += 8;
    for (size_t i = 1; i < p.blocks.size(); ++i)
      p.blocks[i].offset += 8;
    p.total += 8;
  }
  return p;
}

} /* namespace devourer */
