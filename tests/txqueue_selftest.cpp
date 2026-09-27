/* Headless guard for the two pieces of pure logic behind the Jaguar2/3 TX
 * page-ring diagnostics and the Jaguar3 queue routing:
 *   - jaguar3::bulkout_id_for_qsel / per_queue_routing (src/jaguar3/
 *     TxQueueMap.h): the QSEL -> bulk-OUT index map for 1/2/3/4 endpoints,
 *     including the rule that fewer than 3 endpoints keeps every frame on
 *     endpoint 0 (the 3-bulk queue map has no LOW/NORMAL endpoint there),
 *     and jaguar3::tx_qsel / dot11_is_data: SetAmpduMode's TID applies to
 *     data frames only, DEVOURER_TX_QSEL to every frame;
 *   - devourer::pktbuf_read_fits (src/PktBufWindow.h): the dword alignment
 *     and the last-byte window bound of IRtlRadio::ReadPacketBuffer.
 * The Jaguar3 cases self-gate on DEVOURER_HAVE_JAGUAR3 (the define the
 * library exports - same pattern as tests/txagg_selftest.cpp); the
 * PktBufWindow cases are generation-neutral and always run.
 * Prints the failing case and exits nonzero. */
#include <cstdint>
#include <cstdio>

#include "PktBufWindow.h"
#if defined(DEVOURER_HAVE_JAGUAR3)
#include "jaguar3/TxQueueMap.h"
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

#if defined(DEVOURER_HAVE_JAGUAR3)
static void test_queue_map() {
  /* Expected index with >= 3 endpoints: BE/BK (TID 0-3) -> LOW (2),
   * VI/VO (TID 4-7) -> NORMAL (1), everything else -> HIGH (0). */
  for (unsigned eps = 0; eps <= 4; ++eps) {
    const bool routed = eps >= 3;
    CHECK(jaguar3::per_queue_routing(eps) == routed,
          "per_queue_routing(%u) != %d", eps, routed);
    for (unsigned q = 0; q <= 0x1f; ++q) {
      if (q > 7 && q < 0x10)
        continue; /* not a QSEL this backend writes; covered below */
      uint8_t want = 0;
      if (routed)
        want = q <= 3 ? 2 : q <= 7 ? 1 : 0;
      const uint8_t got = jaguar3::bulkout_id_for_qsel(uint8_t(q), eps);
      CHECK(got == want, "eps=%u qsel=0x%02x: id %u, want %u", eps, q,
            unsigned(got), unsigned(want));
      CHECK(eps == 0 || got < eps, "eps=%u qsel=0x%02x: id %u out of range",
            eps, q, unsigned(got));
    }
  }
  /* Unexpected QSELs (0x08..0x0f) are HIGH, never an out-of-range index. */
  for (unsigned q = 0x08; q < 0x10; ++q)
    CHECK(jaguar3::bulkout_id_for_qsel(uint8_t(q), 3) == 0,
          "qsel=0x%02x not HIGH", q);
  /* The beacon/management queues stay HIGH on every shape. */
  for (unsigned eps = 1; eps <= 4; ++eps) {
    CHECK(jaguar3::bulkout_id_for_qsel(0x10, eps) == 0, "BEACON eps=%u", eps);
    CHECK(jaguar3::bulkout_id_for_qsel(0x12, eps) == 0, "MGT eps=%u", eps);
  }
}

static void test_tx_qsel() {
  using jaguar3::dot11_is_data;
  using jaguar3::tx_qsel;
  /* Frame Control byte 0: type in bits 2-3. */
  CHECK(dot11_is_data(0x08), "data (0x08) not data");
  CHECK(dot11_is_data(0x88), "QoS data (0x88) not data");
  CHECK(!dot11_is_data(0x80), "beacon (0x80) read as data");
  CHECK(!dot11_is_data(0x40), "probe req (0x40) read as data");
  CHECK(!dot11_is_data(0xd4), "ACK (0xd4) read as data");
  for (unsigned eps = 0; eps <= 4; ++eps) {
    const bool routed = eps >= 3;
    /* No A-MPDU, no debug: management 0x12 always; data 0 only if routed. */
    CHECK(tx_qsel(false, eps, false, 5, -1) == 0x12, "mgmt eps=%u", eps);
    CHECK(tx_qsel(true, eps, false, 5, -1) == (routed ? 0 : 0x12),
          "data eps=%u", eps);
    /* A-MPDU on: its TID for DATA only - management keeps 0x12. */
    CHECK(tx_qsel(true, eps, true, 5, -1) == 5, "ampdu data eps=%u", eps);
    CHECK(tx_qsel(false, eps, true, 5, -1) == 0x12,
          "ampdu leaked onto mgmt eps=%u", eps);
    /* DEVOURER_TX_QSEL: every frame, over everything. */
    CHECK(tx_qsel(false, eps, true, 5, 0x11) == 0x11, "debug mgmt eps=%u", eps);
    CHECK(tx_qsel(true, eps, true, 5, 0x11) == 0x11, "debug data eps=%u", eps);
  }
  /* Below 3 endpoints the A-MPDU TID still lands on data while the frame
   * rides endpoint 0 - the kept descriptor/endpoint mismatch (TxQueueMap.h). */
  for (unsigned eps = 1; eps < 3; ++eps)
    CHECK(tx_qsel(true, eps, true, 5, -1) == 5 &&
              jaguar3::bulkout_id_for_qsel(tx_qsel(true, eps, true, 5, -1),
                                           eps) == 0,
          "ampdu data below 3 eps: TID on endpoint 0 (eps=%u)", eps);
  /* And the endpoint that follows: A-MPDU never moves management off HIGH. */
  CHECK(jaguar3::bulkout_id_for_qsel(tx_qsel(false, 3, true, 6, -1), 3) == 0,
        "ampdu mgmt left HIGH");
  CHECK(jaguar3::bulkout_id_for_qsel(tx_qsel(true, 3, true, 6, -1), 3) == 1,
        "ampdu VO data not NORMAL");
}

#endif /* DEVOURER_HAVE_JAGUAR3 */

static void test_pktbuf_window() {
  using devourer::pktbuf_read_fits;
  constexpr uint32_t kTx = 0x780, kLlt = 0x650;
  /* Alignment. */
  CHECK(pktbuf_read_fits(kTx, 0, 4), "aligned 4 bytes at 0");
  CHECK(!pktbuf_read_fits(kTx, 2, 4), "unaligned offset accepted");
  CHECK(!pktbuf_read_fits(kTx, 0, 6), "unaligned length accepted");
  CHECK(pktbuf_read_fits(kTx, 0, 0), "n == 0 refused");
  CHECK(!pktbuf_read_fits(kTx, 2, 0), "n == 0 with unaligned offset accepted");
  /* The last window: TX FIFO windows 0x780..0xFFF, i.e. 0x880 of them. */
  constexpr uint64_t kTxSpace = uint64_t(0x1000 - kTx) << 12;
  CHECK(pktbuf_read_fits(kTx, 0, size_t(kTxSpace)), "whole TX window space");
  CHECK(!pktbuf_read_fits(kTx, 0, size_t(kTxSpace + 4)),
        "one dword past the last window accepted");
  CHECK(pktbuf_read_fits(kTx, uint32_t(kTxSpace - 4), 4),
        "last dword of the last window refused");
  CHECK(!pktbuf_read_fits(kTx, uint32_t(kTxSpace), 4),
        "first dword past the last window accepted");
  /* A read ending exactly on a window edge touches only that window. */
  CHECK(pktbuf_read_fits(kTx, uint32_t(kTxSpace - 0x1000), 0x1000),
        "exact last window refused");
  /* The LLT base, and the probe chipstate/ap_wpa2 issue (page 1938). */
  CHECK(pktbuf_read_fits(kTx, 1938u << 7, 32), "beacon page read refused");
  CHECK(pktbuf_read_fits(kLlt, 2047 * 4, 4), "LLT[2047] refused");
  CHECK(pktbuf_read_fits(kLlt, 0, 8192), "whole LLT refused");
  /* A base already past the 12-bit field, and absurd lengths. */
  CHECK(!pktbuf_read_fits(0x1000, 0, 4), "base past the field accepted");
  CHECK(!pktbuf_read_fits(kTx, 0, size_t(-4)), "huge length accepted");
  CHECK(!pktbuf_read_fits(kTx, 0xFFFFFFFCu, 4), "offset overflow accepted");
}

int main() {
#if defined(DEVOURER_HAVE_JAGUAR3)
  test_queue_map();
  test_tx_qsel();
#endif
  test_pktbuf_window();
  if (g_fail) {
    std::printf("txqueue_selftest: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("txqueue_selftest: ok\n");
  return 0;
}
