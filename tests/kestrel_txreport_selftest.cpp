/* Headless regression guard for the Kestrel AX per-user TX-report parser
 * (src/kestrel/TxReportKestrel.h): field extraction from the C2H
 * USR_TX_RPT_INFO content — rpt_mode/macid/ac, per-AC pending, and the freerun
 * TX-egress timestamps (first_in / first_out / last_out). Pure byte-level
 * parsing — no hardware. On-air validation (report events vs witness RX
 * freerun) lands with the RX-loop hook + TX+RX run. */
#include <cstdint>
#include <cstdio>
#include <vector>

#include "kestrel/TxReportKestrel.h"

using namespace kestrel;

static int failures = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      failures++;                                                              \
    }                                                                          \
  } while (0)

static void put_le32(std::vector<uint8_t> &b, size_t off, uint32_t v) {
  b[off + 0] = v & 0xFF;
  b[off + 1] = (v >> 8) & 0xFF;
  b[off + 2] = (v >> 16) & 0xFF;
  b[off + 3] = (v >> 24) & 0xFF;
}

int main() {
  /* Synthetic report: mode=PERIOD(1), macid=0x12, ac=2; pending BE=0x0010
   * BK=0x0020 VI=0x0030 VO=0x0040; freerun first_in=0x11111111
   * first_out=0x22222222 last_out=0x33333333. */
  std::vector<uint8_t> c(24, 0);
  put_le32(c, 0, 0x1u | (0x12u << 8) | (0x2u << 16));       /* mode|macid|ac */
  put_le32(c, 4, 0x0010u | (0x0020u << 16));                /* BE | BK */
  put_le32(c, 8, 0x0030u | (0x0040u << 16));                /* VI | VO */
  put_le32(c, 12, 0x11111111u);                             /* first_in */
  put_le32(c, 16, 0x22222222u);                             /* first_out */
  put_le32(c, 20, 0x33333333u);                             /* last_out */

  KestrelTxReport r;
  CHECK(parse_usr_tx_rpt(c.data(), c.size(), r));
  CHECK(r.rpt_mode == 1);
  CHECK(r.macid == 0x12);
  CHECK(r.ac == 2);
  CHECK(r.pending_1k[0] == 0x0010 && r.pending_1k[1] == 0x0020 &&
        r.pending_1k[2] == 0x0030 && r.pending_1k[3] == 0x0040);
  CHECK(r.freerun_first_in == 0x11111111u);
  CHECK(r.freerun_first_out == 0x22222222u); /* the TX-egress timestamp */
  CHECK(r.freerun_last_out == 0x33333333u);

  /* Too-short buffer must be rejected (no partial parse). */
  KestrelTxReport r2;
  CHECK(!parse_usr_tx_rpt(c.data(), 20, r2));
  CHECK(!parse_usr_tx_rpt(nullptr, 24, r2));

  if (failures) {
    std::fprintf(stderr, "kestrel_txreport: %d FAILURES\n", failures);
    return 1;
  }
  std::printf("kestrel_txreport: all checks passed\n");
  return 0;
}
