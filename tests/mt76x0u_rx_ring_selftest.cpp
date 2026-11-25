/* Headless guard for the MT76x0U receive path below the radio class: the RX
 * parser (src/mt76x0u/rx.cpp) and the ring's completion callback
 * (src/mt76x0u/async.cpp, through mt0_async_rx_done_for_test).
 *
 * Parser:
 *  - MPDU length from the RXWI, clamped to what the transfer holds;
 *  - the L2 pad folded out by moving the REAL header length (a QoS frame's
 *    26 bytes, so the QoS Control bytes survive - the MT7612U bug);
 *  - RSSI = raw + rssi_offset[0] - lna_gain from the published word;
 *  - a rate word naming PHY 5-7 is MT_RX_PARSE_INVALID, as mt76's -EINVAL.
 * Ring:
 *  - the callback runs OUTSIDE the ring lock (one that reads the stats
 *    would otherwise self-deadlock), invalid/short frames are counted and not
 *    delivered, errors and cancellations are told apart, and a stranded slot
 *    touches nothing.
 * rx_active stays 0, so nothing is resubmitted: no context, no device. */
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
struct mt76x0u_rx_info cb_info;
uint8_t cb_first[32];
struct mt76x0u_dev *g_dev;

void expect(const char *what, bool ok) {
  if (!ok) {
    std::fprintf(stderr, "mt76x0u_rx_ring: FAIL %s\n", what);
    fails++;
  }
}

void on_frame(void *, const void *frame, size_t len, const struct mt76x0u_rx_info *info) {
  struct mt76x0u_stats st;
  mt76x0u_get_stats(g_dev, &st); /* takes the ring lock */
  cb_calls++;
  cb_len = len;
  cb_info = *info;
  std::memcpy(cb_first, frame, len < sizeof cb_first ? len : sizeof cb_first);
}

void put32(uint8_t *p, uint32_t v) {
  for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i));
}

/* [DMA hdr 4][RXWI 32][frame], frame bytes taken from `f`. */
int fill(uint8_t *buf, size_t size, unsigned phy, const uint8_t *f, int flen,
         int l2pad, int8_t rssi) {
  std::memset(buf, 0, size);
  uint8_t *rxwi = buf + MT_DMA_HDR_LEN;
  put32(rxwi, l2pad ? MT_RXINFO_L2PAD : 0);
  put32(rxwi + 4, FIELD_PREP(MT_RXWI_CTL_MPDU_LEN, (uint32_t)flen));
  const uint16_t rate = (uint16_t)(FIELD_PREP(MT_RATE_PHY, phy) |
                                   FIELD_PREP(MT_RATE_INDEX, 7));
  rxwi[10] = (uint8_t)rate;
  rxwi[11] = (uint8_t)(rate >> 8);
  rxwi[12] = (uint8_t)rssi;
  uint8_t *body = rxwi + MT_RXWI_LEN;
  if (!l2pad) {
    std::memcpy(body, f, (size_t)flen);
    return MT_DMA_HDR_LEN + MT_RXWI_LEN + flen + 4;
  }
  /* hardware layout: header, 2 pad bytes, body */
  const int hdr = mt0_hdrlen_from_fc(f);
  std::memcpy(body, f, (size_t)hdr);
  body[hdr] = 0xee;
  body[hdr + 1] = 0xee;
  std::memcpy(body + hdr + 2, f + hdr, (size_t)(flen - hdr));
  return MT_DMA_HDR_LEN + MT_RXWI_LEN + flen + 2 + 4;
}

void complete(struct libusb_transfer *t, const char *what) {
  std::atomic<bool> done{false};
  std::thread th([&] {
    mt0_async_rx_done_for_test(t);
    done = true;
  });
  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!done && std::chrono::steady_clock::now() < until)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  if (!done) {
    std::fprintf(stderr, "mt76x0u_rx_ring: FAIL %s: rx_done did not return "
                 "within 2 s (callback run under the ring lock?)\n", what);
    std::fflush(stderr);
    std::_Exit(1);
  }
  th.join();
}

} // namespace

int main() {
  struct mt76x0u_dev d{};
  struct mt0_async *a = new mt0_async{};
  struct libusb_transfer *t = libusb_alloc_transfer(0);
  static uint8_t buf[512];
  uint8_t frame[64];
  const uint8_t *out = nullptr;
  struct mt76x0u_rx_info info;

  if (!t) {
    std::fprintf(stderr, "mt76x0u_rx_ring: libusb_alloc_transfer failed\n");
    return 1;
  }

  /* --- parser ------------------------------------------------------------ */
  /* A QoS data frame: FC 0x0088 (type data, QoS subtype), 26-byte header,
   * QoS Control = TID 5 at bytes 24-25, then a 10-byte body. */
  std::memset(frame, 0, sizeof frame);
  frame[0] = 0x88;
  for (int i = 2; i < 24; i++) frame[i] = (uint8_t)i;
  frame[24] = 0x05;
  frame[25] = 0x00;
  for (int i = 26; i < 36; i++) frame[i] = (uint8_t)(0xa0 + i);
  /* lna_gain 4, rssi_offset[0] -2 -> rssi = raw - 2 - 4 */
  d.rx_corr.store((4u << 8) | (uint8_t)(int8_t)-2);

  int n = fill(buf, sizeof buf, MT76X0U_PHY_HT, frame, 36, 1, -50);
  int len = mt0_rx_parse(&d, buf, n, &out, &info);
  expect("L2PAD QoS frame: MPDU length 36", len == 36);
  expect("L2PAD fold keeps the QoS Control bytes",
         out && std::memcmp(out, frame, 36) == 0);
  expect("RSSI corrected: -50 + (-2) - 4", info.rssi_raw == -50 && info.rssi_dbm == -56);
  expect("rate decoded: HT MCS 7, 1 stream",
         info.phy == MT76X0U_PHY_HT && info.mcs == 7 && info.nss == 1);

  n = fill(buf, sizeof buf, MT76X0U_PHY_OFDM, frame, 36, 0, -40);
  len = mt0_rx_parse(&d, buf, n, &out, &info);
  expect("unpadded frame delivered as is", len == 36 && std::memcmp(out, frame, 36) == 0);

  n = fill(buf, sizeof buf, 5, frame, 36, 0, -40);
  expect("PHY 5 is invalid", mt0_rx_parse(&d, buf, n, &out, &info) == MT_RX_PARSE_INVALID);

  n = fill(buf, sizeof buf, MT76X0U_PHY_OFDM, frame, 36, 0, -40);
  expect("MPDU length clamped to the transfer",
         mt0_rx_parse(&d, buf, MT_DMA_HDR_LEN + MT_RXWI_LEN + 20, &out, &info) == 20);
  expect("short transfer unusable", mt0_rx_parse(&d, buf, 30, &out, &info) == 0);

  /* --- ring --------------------------------------------------------------- */
  g_dev = &d;
  d.a = a;
  a->cb = on_frame;
  a->rx_slot[0].d = &d;
  a->rx_slot[0].a = a;
  t->user_data = &a->rx_slot[0];
  t->buffer = buf;

  t->status = LIBUSB_TRANSFER_COMPLETED;
  t->actual_length = fill(buf, sizeof buf, MT76X0U_PHY_HT, frame, 36, 1, -50);
  a->rx_inflight = 1;
  complete(t, "valid");
  expect("valid: delivered once at 36 bytes", cb_calls == 1 && cb_len == 36);
  expect("valid: delivered frame is unpadded", std::memcmp(cb_first, frame, 32) == 0);
  expect("valid: rx_frames == 1", a->rx_frames == 1);
  expect("valid: transfer given back", a->rx_inflight == 0);

  t->actual_length = fill(buf, sizeof buf, 6, frame, 36, 0, -50);
  a->rx_inflight = 1;
  complete(t, "invalid PHY");
  expect("invalid PHY: counted invalid and dropped, not delivered",
         a->rx_invalid == 1 && a->rx_dropped == 1 && cb_calls == 1);

  t->actual_length = 10;
  a->rx_inflight = 1;
  complete(t, "short");
  expect("short: dropped, not invalid", a->rx_dropped == 2 && a->rx_invalid == 1);

  t->status = LIBUSB_TRANSFER_ERROR;
  a->rx_inflight = 1;
  complete(t, "error");
  expect("error: rx_err == 1", a->rx_err == 1);
  t->status = LIBUSB_TRANSFER_CANCELLED;
  a->rx_inflight = 1;
  complete(t, "cancelled");
  expect("cancelled: nothing counted", a->rx_err == 1 && a->rx_frames == 1);

  a->rx_slot[0].d = NULL;
  t->status = LIBUSB_TRANSFER_COMPLETED;
  t->actual_length = fill(buf, sizeof buf, MT76X0U_PHY_HT, frame, 36, 0, -50);
  a->rx_inflight = 1;
  complete(t, "stranded");
  expect("stranded: not delivered, not counted", cb_calls == 1 && a->rx_frames == 1);
  expect("stranded: transfer given back", a->rx_inflight == 0);

  libusb_free_transfer(t);
  d.a = NULL;
  delete a;
  if (fails) return 1;
  std::printf("mt76x0u_rx_ring: ok\n");
  return 0;
}
