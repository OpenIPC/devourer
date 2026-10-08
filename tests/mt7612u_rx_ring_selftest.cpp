/* Headless guard for the MT7612U async RX ring's completion callback
 * (rx_done in src/mt7612u/async.cpp, reached through the test hook
 * mt_async_rx_done_for_test).
 *
 * rx_done parses a completed bulk-IN transfer under the ring's lock, so the
 * parser must never take that lock itself. A parser that counted rx_invalid
 * for an RXWI whose rate word names PHY 5-7 through a helper locking the same
 * non-recursive mutex would self-deadlock, and one corrupt frame would wedge
 * the RX ring, every TX submit, the stats read and the teardown. This cell
 * feeds the callback such a frame and fails if it does not return.
 *
 * It also pins the counters for a valid frame and for a stranded slot (device
 * pointer cleared by a stop that leaked the ring): a completed transfer there
 * must parse nothing and count nothing, only give the transfer back.
 *
 * No context, no device: rx_done needs only a transfer with its status,
 * buffer and length filled in. rx_active stays 0, so nothing is resubmitted.
 * What it does NOT cover: the resubmit path and the stop/strand ordering,
 * which need libusb to own a real transfer. */
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

#include "internal.h"

namespace {

int fails;
int cb_calls;
size_t cb_len;

void expect(const char *what, bool ok) {
  if (!ok) {
    std::fprintf(stderr, "mt7612u_rx_ring: FAIL %s\n", what);
    fails++;
  }
}

void on_frame(void *, const void *, size_t len, const struct mt7612u_rx_info *) {
  cb_calls++;
  cb_len = len;
}

/* One RXWI + a 40-byte data frame, with `phy` in the rate word. The layout is
 * src/mt7612u/tests/frame_shape.cpp's. */
int fill(uint8_t *buf, size_t size, unsigned phy) {
  const uint32_t ctl = FIELD_PREP(MT_RXWI_CTL_MPDU_LEN, 40u);
  const uint16_t rate = (uint16_t)FIELD_PREP(MT_RATE_PHY, phy);

  std::memset(buf, 0, size);
  for (int i = 0; i < 4; i++)
    buf[MT_DMA_HDR_LEN + 4 + i] = (uint8_t)(ctl >> (8 * i));
  buf[MT_DMA_HDR_LEN + 10] = (uint8_t)(rate & 0xff);
  buf[MT_DMA_HDR_LEN + 11] = (uint8_t)(rate >> 8);
  buf[MT_DMA_HDR_LEN + MT_RXWI_LEN] = 0x08; /* data frame */
  return MT_DMA_HDR_LEN + MT_RXWI_LEN + 40;
}

/* Runs the completion on its own thread: a self-deadlock must FAIL the cell,
 * not hang it until ctest's timeout. A wedged thread cannot be joined, so the
 * cell leaves through _Exit. */
void complete(struct libusb_transfer *t, const char *what) {
  std::atomic<bool> done{false};
  std::thread th([&] {
    mt_async_rx_done_for_test(t);
    done = true;
  });
  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!done && std::chrono::steady_clock::now() < until)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  if (!done) {
    std::fprintf(stderr, "mt7612u_rx_ring: FAIL %s: rx_done did not return "
                 "within 2 s (self-deadlock on the ring lock)\n", what);
    std::fflush(stderr);
    std::_Exit(1);
  }
  th.join();
}

} // namespace

int main() {
  struct mt7612u_dev d{};
  struct mt_async *a = new mt_async{};
  struct libusb_transfer *t = libusb_alloc_transfer(0);
  static uint8_t buf[256];

  if (!t) {
    std::fprintf(stderr, "mt7612u_rx_ring: libusb_alloc_transfer failed\n");
    return 1;
  }
  d.chainmask = 0x0202;
  /* As in a live session: the device points at its ring. A counting helper
   * reaches the lock through d->a, so without this the cell cannot see the
   * deadlock it exists for. */
  d.a = a;
  a->cb = on_frame;
  a->rx_slot[0].d = &d;
  a->rx_slot[0].a = a;
  t->user_data = &a->rx_slot[0];
  t->buffer = buf;
  t->status = LIBUSB_TRANSFER_COMPLETED;

  /* 1. Invalid PHY: must return, and count as invalid (and dropped). */
  t->actual_length = fill(buf, sizeof buf, 5);
  a->rx_inflight = 1;
  complete(t, "invalid PHY");
  expect("invalid PHY: rx_invalid == 1", a->rx_invalid == 1);
  expect("invalid PHY: rx_dropped == 1", a->rx_dropped == 1);
  expect("invalid PHY: no frame delivered", cb_calls == 0 && a->rx_frames == 0);
  expect("invalid PHY: transfer given back", a->rx_inflight == 0);

  /* 2. Valid PHY: delivered once, at its length. */
  t->actual_length = fill(buf, sizeof buf, 1);
  a->rx_inflight = 1;
  complete(t, "valid PHY");
  expect("valid PHY: rx_frames == 1", a->rx_frames == 1);
  expect("valid PHY: delivered at 40 bytes", cb_calls == 1 && cb_len == 40);
  expect("valid PHY: rx_invalid unchanged", a->rx_invalid == 1);
  expect("valid PHY: transfer given back", a->rx_inflight == 0);

  /* 3. Stranded slot: the device is gone, so nothing is parsed or counted. */
  a->rx_slot[0].d = NULL;
  t->actual_length = fill(buf, sizeof buf, 5);
  a->rx_inflight = 1;
  complete(t, "stranded slot");
  expect("stranded: no counter moved",
         a->rx_frames == 1 && a->rx_invalid == 1 && a->rx_dropped == 1);
  expect("stranded: no frame delivered", cb_calls == 1);
  expect("stranded: transfer given back", a->rx_inflight == 0);

  libusb_free_transfer(t);
  d.a = NULL;
  delete a;
  if (fails) return 1;
  std::printf("mt7612u_rx_ring: ok\n");
  return 0;
}
