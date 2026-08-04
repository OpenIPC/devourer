/* Headless guard for the windowed RX-receipt primitives
 * (src/cell/RxReceipt.h): TLV encode/decode round-trip, ring-window eviction
 * across large index jumps, below-window late accounting, ledger merge
 * idempotence over overlapping receipts, and the strict-parse rejections
 * (magic/version/length/TA) that keep arbitrary payloads from absorbing as
 * receipts. Pure math — the on-air halves (injection cadence, feedback-path
 * loss tolerance, frame-exact agreement with the rx.seq ledger) are
 * tests/receipt_verify.py over an arq_e2e run. */
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <set>
#include <vector>

#include "cell/RxReceipt.h"

using devourer::cell::ReceiptLedger;
using devourer::cell::ReceiptView;
using devourer::cell::ReceiptWindow;
using devourer::cell::receipt_decode;
using devourer::cell::receipt_tlv_size;

static const uint8_t kTa[6] = {0x02, 0xaa, 0xbb, 0xcc, 0xdd, 0x01};
static const uint8_t kOther[6] = {0x02, 0xaa, 0xbb, 0xcc, 0xdd, 0x02};

static std::set<uint32_t> decode_set(const uint8_t *tlv, size_t len) {
  ReceiptView v;
  assert(receipt_decode(tlv, len, v));
  std::set<uint32_t> s;
  for (uint32_t i = 0; i < v.nbits; ++i)
    if (v.bit(i))
      s.insert(v.base + i);
  return s;
}

int main() {
  /* Round-trip: a sparse pattern survives encode/decode exactly. */
  {
    ReceiptWindow w(256);
    std::set<uint32_t> fed;
    for (uint32_t i : {0u, 1u, 5u, 60u, 63u, 64u, 100u, 255u}) {
      w.note(i);
      fed.insert(i);
    }
    uint8_t buf[receipt_tlv_size(256)];
    const size_t n = w.encode(kTa, buf, sizeof buf);
    assert(n == receipt_tlv_size(256));
    assert(decode_set(buf, n) == fed);
  }

  /* Eviction: indices that fell out of the ring are gone; slots reused by a
   * big jump start clear. */
  {
    ReceiptWindow w(64);
    for (uint32_t i = 0; i < 64; ++i)
      w.note(i); /* full window, all set */
    w.note(200); /* jump: window becomes [137..200], only 200 set */
    uint8_t buf[receipt_tlv_size(64)];
    const size_t n = w.encode(kTa, buf, sizeof buf);
    assert(n == receipt_tlv_size(64));
    const auto s = decode_set(buf, n);
    assert(s.size() == 1 && *s.begin() == 200);
    /* Below-window arrival counts as late, never receipts. */
    assert(w.late() == 0);
    w.note(3);
    assert(w.late() == 1);
  }

  /* Ledger: overlapping receipts merge idempotently; fresh counts only new
   * indices; coverage bound advances; foreign TA and malformed TLVs refuse. */
  {
    ReceiptWindow w(128);
    ReceiptLedger led;
    uint8_t buf[receipt_tlv_size(128)];
    for (uint32_t i = 0; i < 100; ++i)
      if (i != 40 && i != 41 && i != 90) /* three losses */
        w.note(i);
    size_t n = w.encode(kTa, buf, sizeof buf);
    assert(led.absorb(buf, n, kTa) == 97);
    assert(led.absorb(buf, n, kTa) == 0); /* pure re-receipt */
    /* Late recovery inside the window, then an overlapping re-encode. */
    w.note(90);
    n = w.encode(kTa, buf, sizeof buf);
    assert(led.absorb(buf, n, kTa) == 1);
    assert(led.delivered_total() == 98);
    assert(led.delivered(89) && led.delivered(90) && !led.delivered(40));
    assert(led.highest_covered() == 100); /* exclusive */
    assert(led.receipts() == 3);
    /* Rejections: wrong TA, truncated, magic, version. Trailing bytes are
     * ACCEPTED (a received body carries the FCS after the TLV). */
    assert(led.absorb(buf, n, kOther) == -1);
    assert(led.absorb(buf, n - 1, kTa) == -1);
    uint8_t fcs[receipt_tlv_size(128) + 4];
    std::memcpy(fcs, buf, n);
    std::memset(fcs + n, 0xee, 4);
    assert(led.absorb(fcs, n + 4, kTa) == 0); /* parses; nothing new */
    uint8_t bad[receipt_tlv_size(128)];
    std::memcpy(bad, buf, n);
    bad[0] = 'X';
    assert(led.absorb(bad, n, kTa) == -1);
    std::memcpy(bad, buf, n);
    bad[2] = 99;
    assert(led.absorb(bad, n, kTa) == -1);
    /* Over-the-air DoS guard: a crafted base near 2^32 must refuse, not
     * allocate half a gigabyte of bitmap. */
    std::memcpy(bad, buf, n);
    bad[10] = bad[11] = bad[12] = bad[13] = 0xff;
    assert(led.absorb(bad, n, kTa) == -1);
    assert(led.delivered_total() == 98); /* untouched by the rejects */
  }

  /* Ring addressing with a window that is NOT a multiple of 64 — the word
   * and bit offsets must both derive from the slot, or bits land in the
   * wrong positions once idx % _w and idx % 64 diverge. */
  {
    ReceiptWindow w(100);
    std::set<uint32_t> fed;
    for (uint32_t i = 150; i < 250; i += 7) { /* crosses the ring seam */
      w.note(i);
      fed.insert(i);
    }
    uint8_t buf[receipt_tlv_size(100)];
    const size_t n = w.encode(kTa, buf, sizeof buf);
    assert(n != 0);
    assert(decode_set(buf, n) == fed);
  }

  std::puts("receipt selftest: OK");
  return 0;
}
