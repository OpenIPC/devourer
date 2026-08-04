/* Windowed RX receipts — app-layer delivery truth for a TX peer.
 *
 * The hardware ACK's horizon is chip-FIFO admission (SIFS timing: it can
 * never confirm host delivery), and per-frame CCX reports are rate-bounded
 * by the fw's ~1.3k reports/s emission ceiling. This is the tier above both:
 * the RECEIVER's application counts what it actually consumed and mails a
 * compact bitmap back on its existing feedback path; the transmitter merges
 * those receipts into a delivered-set that is true at the only layer that
 * matters. Same caller-side contract as UeRxAttribution: the demos feed
 * frames in from the Packet callback, the device RX loops are untouched, and
 * the library never assumes a payload convention — the caller supplies the
 * u32 frame index (the bench wires txdemo's QoS-Data payload counter).
 *
 * Wire format (versioned TLV, little-endian, one transmitter per TLV):
 *   [0..2)   magic "DR"
 *   [2]      version (1)
 *   [3]      flags (0)
 *   [4..10)  ta — the transmitter this receipt covers; the TX side absorbs
 *            only receipts naming its own SA, which (with magic + version +
 *            exact-length) keeps arbitrary payloads from parsing as receipts
 *   [10..14) base — first frame index the bitmap covers
 *   [14..16) nbits — window length in frames
 *   [16..)   bitmap, LSB-first: bit i set => frame base+i was received
 *
 * Receipts OVERLAP: every emission covers the last `window_bits` indices, so
 * each frame appears in many consecutive receipts and individual receipt
 * loss is harmless — the merge is idempotent. A frame index arriving BELOW
 * the current window (reordered by more than window_bits) is counted as
 * `late` and never receipted; size the window against the emission cadence
 * (2048 bits at 2.4k fps ≈ 0.85 s of slack, orders beyond USB reordering).
 */
#ifndef DEVOURER_CELL_RX_RECEIPT_H
#define DEVOURER_CELL_RX_RECEIPT_H

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <vector>

namespace devourer {
namespace cell {

constexpr size_t kReceiptHeader = 16;
constexpr uint8_t kReceiptVersion = 1;

constexpr size_t receipt_tlv_size(uint16_t nbits) {
  return kReceiptHeader + (static_cast<size_t>(nbits) + 7) / 8;
}

/* Receiver half: a ring bitmap over the last `window_bits` frame indices.
 *
 * SIZING: the window must exceed the largest index jump the receiver's own
 * buffering can produce between two encodes, or a post-stall backlog drain
 * evicts frames before any receipt covers them (measured: a 150 ms consumer
 * stall behind a 104-buffer spsc-fat pool drained ~3k frames in one receipt
 * interval and a 2048-bit window leaked 2,846 delivered frames out of
 * coverage). Rule: window_bits > pool_bytes / min_frame_bytes + one encode
 * interval of arrivals. The 8192 default gives ~2.7x margin over that bench
 * worst case at ~1 KB of bitmap per receipt (~1.4 ms at 6M). */
class ReceiptWindow {
public:
  explicit ReceiptWindow(uint16_t window_bits = 8192)
      : _w(window_bits ? window_bits : 8192),
        _bits((_w + 63) / 64, 0) {}

  /* Record a received frame index. Indices may repeat (duplicates are
   * idempotent) and skip forward arbitrarily; an index below the current
   * window is counted, not receipted. */
  void note(uint32_t idx) {
    std::lock_guard<std::mutex> lk(_mu);
    if (!_any) {
      _any = true;
      _max = idx;
      set_bit(idx);
      return;
    }
    if (idx > _max) {
      /* Advance: every ring slot between the old and new max is a NEW index
       * whose bit must start clear. A jump >= _w wipes the whole ring. */
      const uint32_t advance = idx - _max;
      const uint32_t to_clear = advance < _w ? advance : _w;
      for (uint32_t k = 0; k < to_clear; ++k)
        clear_bit(_max + 1 + k + (advance - to_clear));
      _max = idx;
      set_bit(idx);
      return;
    }
    if (_max - idx >= _w) {
      ++_late; /* below the window — reordered past the receipt horizon */
      return;
    }
    set_bit(idx);
  }

  /* Encode the current window into `out` (>= receipt_tlv_size(window_bits)).
   * Returns the TLV length, or 0 if nothing has been noted yet. */
  size_t encode(const uint8_t ta[6], uint8_t *out, size_t out_len) const {
    std::lock_guard<std::mutex> lk(_mu);
    if (!_any)
      return 0;
    const uint32_t base = _max >= _w - 1 ? _max - (_w - 1) : 0;
    const uint16_t nbits = static_cast<uint16_t>(_max - base + 1);
    const size_t need = receipt_tlv_size(nbits);
    if (out_len < need)
      return 0;
    out[0] = 'D';
    out[1] = 'R';
    out[2] = kReceiptVersion;
    out[3] = 0;
    std::memcpy(out + 4, ta, 6);
    put32(out + 10, base);
    put16(out + 14, nbits);
    std::memset(out + kReceiptHeader, 0, need - kReceiptHeader);
    for (uint32_t i = 0; i < nbits; ++i)
      if (get_bit(base + i))
        out[kReceiptHeader + i / 8] |= static_cast<uint8_t>(1u << (i % 8));
    return need;
  }

  uint64_t late() const {
    std::lock_guard<std::mutex> lk(_mu);
    return _late;
  }

private:
  /* Ring addressing: index -> bit slot idx % _w, and BOTH the word and the
   * bit offset derive from the slot — deriving the bit from idx % 64 is only
   * equivalent when _w is a multiple of 64, and the window size is caller-
   * chosen. Valid because the window invariant keeps every live index within
   * [_max - _w + 1, _max]. */
  void set_bit(uint32_t idx) {
    const uint32_t s = idx % _w;
    _bits[s / 64] |= 1ull << (s % 64);
  }
  void clear_bit(uint32_t idx) {
    const uint32_t s = idx % _w;
    _bits[s / 64] &= ~(1ull << (s % 64));
  }
  bool get_bit(uint32_t idx) const {
    const uint32_t s = idx % _w;
    return (_bits[s / 64] >> (s % 64)) & 1u;
  }
  static void put32(uint8_t *p, uint32_t v) {
    p[0] = v & 0xff;
    p[1] = (v >> 8) & 0xff;
    p[2] = (v >> 16) & 0xff;
    p[3] = (v >> 24) & 0xff;
  }
  static void put16(uint8_t *p, uint16_t v) {
    p[0] = v & 0xff;
    p[1] = (v >> 8) & 0xff;
  }

  mutable std::mutex _mu;
  uint32_t _w;
  std::vector<uint64_t> _bits;
  bool _any = false;
  uint32_t _max = 0;
  uint64_t _late = 0;
};

/* One decoded receipt, viewing the caller's buffer (no copy). */
struct ReceiptView {
  const uint8_t *ta = nullptr;
  uint32_t base = 0;
  uint16_t nbits = 0;
  const uint8_t *bitmap = nullptr;
  bool bit(uint32_t i) const { return (bitmap[i / 8] >> (i % 8)) & 1u; }
};

/* Strict-prefix parse: magic + version + a bitmap fully inside the buffer.
 * Trailing bytes are accepted — a received MPDU body carries the 4-byte FCS
 * after the TLV (parsers deliver it; an ACK frame logs len 14 = 10 + FCS),
 * and the absorb side additionally requires the 6-byte TA match, which keeps
 * arbitrary payloads from parsing as receipts. */
inline bool receipt_decode(const uint8_t *p, size_t len, ReceiptView &out) {
  if (!p || len < kReceiptHeader || p[0] != 'D' || p[1] != 'R' ||
      p[2] != kReceiptVersion)
    return false;
  const uint16_t nbits =
      static_cast<uint16_t>(p[14] | (static_cast<uint16_t>(p[15]) << 8));
  if (nbits == 0 || len < receipt_tlv_size(nbits))
    return false;
  out.ta = p + 4;
  out.base = static_cast<uint32_t>(p[10]) | (static_cast<uint32_t>(p[11]) << 8) |
             (static_cast<uint32_t>(p[12]) << 16) |
             (static_cast<uint32_t>(p[13]) << 24);
  out.nbits = nbits;
  out.bitmap = p + kReceiptHeader;
  return true;
}

/* Transmitter half: merge receipts into the delivered-set.
 *
 * `max_index` bounds the bitmap: receipts arrive OVER THE AIR, so `base` is
 * attacker-influencable and an unbounded resize would let one crafted TLV
 * (base near 2^32) allocate half a gigabyte. The default cap (2^26 frames =
 * 8 MB of bitmap, ~7.7 h at 2.4 k fps) is far above any bench run; a peer
 * whose real index space outgrows it should shard by session. */
class ReceiptLedger {
public:
  explicit ReceiptLedger(uint64_t max_index = 1ull << 26)
      : _max_index(max_index) {}

  /* Absorb a receipt covering our TA. Returns the number of NEWLY-learned
   * delivered indices (0 for a pure re-receipt), or -1 if the TLV does not
   * parse, names a different transmitter, or claims indices beyond the
   * ledger's cap. */
  long absorb(const uint8_t *tlv, size_t len, const uint8_t own_ta[6]) {
    ReceiptView v;
    if (!receipt_decode(tlv, len, v) || std::memcmp(v.ta, own_ta, 6) != 0)
      return -1;
    std::lock_guard<std::mutex> lk(_mu);
    const uint64_t top = static_cast<uint64_t>(v.base) + v.nbits;
    if (top > _max_index)
      return -1;
    if (top > _bits.size() * 64)
      _bits.resize((top + 63) / 64, 0);
    long fresh = 0;
    for (uint32_t i = 0; i < v.nbits; ++i) {
      if (!v.bit(i))
        continue;
      const uint64_t idx = static_cast<uint64_t>(v.base) + i;
      uint64_t &word = _bits[idx / 64];
      const uint64_t m = 1ull << (idx % 64);
      if (!(word & m)) {
        word |= m;
        ++fresh;
        ++_total;
      }
    }
    if (top > _highest_covered)
      _highest_covered = top; /* exclusive: indices below this were covered */
    ++_receipts;
    return fresh;
  }

  bool delivered(uint64_t idx) const {
    std::lock_guard<std::mutex> lk(_mu);
    return idx / 64 < _bits.size() && ((_bits[idx / 64] >> (idx % 64)) & 1u);
  }
  uint64_t delivered_total() const {
    std::lock_guard<std::mutex> lk(_mu);
    return _total;
  }
  /* Exclusive upper bound of receipt coverage: every index below it has been
   * inside at least one absorbed receipt's window. */
  uint64_t highest_covered() const {
    std::lock_guard<std::mutex> lk(_mu);
    return _highest_covered;
  }
  uint64_t receipts() const {
    std::lock_guard<std::mutex> lk(_mu);
    return _receipts;
  }

private:
  mutable std::mutex _mu;
  uint64_t _max_index;
  std::vector<uint64_t> _bits;
  uint64_t _total = 0;
  uint64_t _highest_covered = 0;
  uint64_t _receipts = 0;
};

} /* namespace cell */
} /* namespace devourer */

#endif /* DEVOURER_CELL_RX_RECEIPT_H */
