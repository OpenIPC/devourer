#ifndef KESTREL_TX_REPORT_KESTREL_H
#define KESTREL_TX_REPORT_KESTREL_H

#include <cstddef>
#include <cstdint>

namespace kestrel {

/* Parsed AX per-user TX report (C2H FWCMD_C2H_FUNC_USR_TX_RPT_INFO, func 0x7).
 * Field layout from phl/hal_g6/mac/fw_ax/inc_hdr/fwcmd_intf.h (FWCMD_C2H_USR_
 * TX_RPT_INFO_*). The freerun_* fields are the SAME free-running HW counter the
 * RX parser reports (KestrelRxFrame.freerun_cnt), so first_out is a host-
 * visible TX-EGRESS timestamp directly comparable to RX arrival times — the
 * scheduled-TX air-departure measurement 11ac/Jaguar could not provide. */
struct KestrelTxReport {
  uint8_t rpt_mode = 0; /* echoes mac_ax_usr_tx_rpt_mode (1=PERIOD,2=LAST_PKT) */
  uint8_t macid = 0;
  uint8_t ac = 0;
  uint16_t pending_1k[4] = {0, 0, 0, 0}; /* BE, BK, VI, VO pending / 1KB units */
  uint32_t freerun_first_in = 0;  /* HW time the first frame ENTERED the queue */
  uint32_t freerun_first_out = 0; /* HW time the first frame LEFT (TX egress)  */
  uint32_t freerun_last_out = 0;  /* HW time the last frame LEFT (TX egress)   */
};

/* Parse the USR_TX_RPT_INFO report CONTENT (the 6 dwords after the C2H fwcmd
 * header). Returns false if `len` is too short. Layout (LE):
 *   dw0: RPT_MODE[2:0] | MACID[15:8] | AC[17:16]
 *   dw1: PENDING_BE_1K[15:0] | PENDING_BK_1K[31:16]
 *   dw2: PENDING_VI_1K[15:0] | PENDING_VO_1K[31:16]
 *   dw3: FREERUN_CNT_FIRST_IN     dw4: FREERUN_CNT_FIRST_OUT
 *   dw5: FREERUN_CNT_LAST_OUT */
inline bool parse_usr_tx_rpt(const uint8_t *c, size_t len, KestrelTxReport &o) {
  if (c == nullptr || len < 24)
    return false;
  auto le32 = [](const uint8_t *p) -> uint32_t {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
  };
  const uint32_t d0 = le32(c + 0), d1 = le32(c + 4), d2 = le32(c + 8);
  o.rpt_mode = static_cast<uint8_t>(d0 & 0x7);
  o.macid = static_cast<uint8_t>((d0 >> 8) & 0xff);
  o.ac = static_cast<uint8_t>((d0 >> 16) & 0x3);
  o.pending_1k[0] = static_cast<uint16_t>(d1 & 0xffff);         /* BE */
  o.pending_1k[1] = static_cast<uint16_t>((d1 >> 16) & 0xffff); /* BK */
  o.pending_1k[2] = static_cast<uint16_t>(d2 & 0xffff);         /* VI */
  o.pending_1k[3] = static_cast<uint16_t>((d2 >> 16) & 0xffff); /* VO */
  o.freerun_first_in = le32(c + 12);
  o.freerun_first_out = le32(c + 16);
  o.freerun_last_out = le32(c + 20);
  return true;
}

} /* namespace kestrel */

#endif /* KESTREL_TX_REPORT_KESTREL_H */
