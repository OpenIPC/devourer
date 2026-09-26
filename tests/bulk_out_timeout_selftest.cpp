/* Headless guard for src/BulkOutTimeout.h, the timeout policy behind
 * DeviceConfig::Tx::no_cancel_multipkt: a bulk-OUT longer than one USB packet
 * gets no timeout (libusb cancels on timeout, and a cancel after the chip took
 * part of the transfer lets the next queued sender's bytes splice into the
 * packet and wedge the TXDMA), while a single-packet transfer, accepted whole
 * or not at all, keeps its caller's timeout. Pure math; no hardware. */
#include "BulkOutTimeout.h"

#include <cstdio>
#include <cstdlib>

using devourer::bulk_out_timeout_ms;

static int fails = 0;
#define CHECK(c)                                                               \
  do {                                                                         \
    if (!(c)) {                                                                \
      std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #c);         \
      ++fails;                                                                 \
    }                                                                          \
  } while (0)

int main() {
  /* High-speed endpoint, 512-byte packets. */
  CHECK(bulk_out_timeout_ms(93, 512, 20) == 20);  /* control frame */
  CHECK(bulk_out_timeout_ms(512, 512, 20) == 20); /* exactly one packet */
  CHECK(bulk_out_timeout_ms(513, 512, 20) == 0);  /* one byte over */
  CHECK(bulk_out_timeout_ms(4439, 512, 50) == 0); /* 3-frame aggregate */
  CHECK(bulk_out_timeout_ms(93, 512, 0) == 0);    /* infinite stays infinite */

  /* SuperSpeed endpoint, 1024-byte packets. */
  CHECK(bulk_out_timeout_ms(1000, 1024, 20) == 20);
  CHECK(bulk_out_timeout_ms(1025, 1024, 20) == 0);

  /* Unknown packet size: assume the smallest bulk packet (64, full-speed).
   * Conservative by intent: on a high-speed endpoint whose size was not
   * discovered, a 65..512-byte single-packet transfer also loses its timeout
   * — erring towards never cancelling. */
  CHECK(bulk_out_timeout_ms(64, 0, 20) == 20);
  CHECK(bulk_out_timeout_ms(93, 0, 20) == 0);

  if (fails)
    return EXIT_FAILURE;
  std::printf("bulk_out_timeout_selftest: all passed\n");
  return 0;
}
