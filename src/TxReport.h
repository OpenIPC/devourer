#pragma once

/* TxReport — per-frame TX-status feedback (the vendor "CCX report"): when a
 * frame's TX descriptor carries SPE_RPT=1 (DeviceConfig tx.report), the
 * firmware answers each transmission with a C2H report saying whether the
 * frame was delivered (ACKed — or simply completed, for broadcast/no-ack),
 * how many hardware retransmissions it took, how long it sat in the TX queue,
 * and the final rate after any fallback. That is the TX-side link sensor: the
 * retry count is the ground truth an adaptive rate/power controller wants,
 * and queue time is the TX-pacing signal.
 *
 * Two on-wire formats:
 *  - Jaguar1 (8812/8821): C2H id 0x03 (CCX_TX_RPT), 6-byte payload after the
 *    2-byte C2H envelope (vendor GET_8812_C2H_TX_RPT_*; queue time in 256 µs
 *    units). The 8814A firmware uses its own TX_RPT layout — not parsed here
 *    (examples/rx decodes it behind DEVOURER_TX_STATUS).
 *  - HalMAC (8822B/8821C/8822C/8822E): a C2H "pkt" — cmd_id 0xFF (C2H_EXTEND),
 *    sub_cmd_id 0x0F (CCX_RPT) — parsed INCLUDING its 4-byte header (vendor
 *    halmac_fw_offload_c2h_nic.h CCX_RPT_GET_*). Echoes the descriptor
 *    SW_DEFINE low byte, so reports correlate to frames per-packet: the
 *    devices stamp a rotating tag into SW_DEFINE when tx.report is on.
 *
 * Decode is header-only and pure; the devices emit the result as a
 * `tx.report` machine event (docs/logging.md) from their C2H RX sites. */

#include <cstddef>
#include <cstdint>

#include "Event.h"

namespace devourer {

struct TxReport {
  bool valid = false;
  /* Delivery state: 0 = done (ACKed, or completed for BMC/no-ack), 1 =
   * retry-drop (retry limit exhausted without ACK), 2/3 = firmware-specific
   * (halmac tx_state passthrough; Jaguar1 maps lifetime-drop to 2). */
  uint8_t state = 0;
  bool bmc = false;        /* broadcast/multicast frame (no ACK expected) */
  uint8_t macid = 0;
  uint8_t qsel = 0;        /* Jaguar1 only (halmac reports it too) */
  uint8_t data_retries = 0; /* hardware retransmissions used (0 = first try) */
  uint8_t rts_retries = 0;  /* halmac only */
  uint16_t queue_time_raw = 0; /* Jaguar1: 256 µs units; halmac: raw fw units */
  uint8_t final_rate = 0;  /* DESC_RATE* hw index of the final attempt */
  uint8_t sw_define = 0;   /* halmac: descriptor SW_DEFINE byte-0 echo */
  uint8_t missed_rpt = 0;  /* halmac: reports the fw dropped before this one */
};

/* Jaguar1 (8812/8821) CCX payload: `p` points AFTER the 2-byte C2H envelope
 * (cmd_id, seq). */
inline TxReport parse_ccx_8812(const uint8_t *p, size_t len) {
  TxReport r;
  if (p == nullptr || len < 6)
    return r;
  r.valid = true;
  r.qsel = p[0] & 0x1f;
  r.bmc = (p[0] >> 5) & 1;
  const bool lifetime_over = (p[0] >> 6) & 1;
  const bool retry_over = (p[0] >> 7) & 1;
  r.state = retry_over ? 1 : lifetime_over ? 2 : 0;
  r.macid = p[1];
  r.data_retries = p[2] & 0x3f;
  r.queue_time_raw = static_cast<uint16_t>(p[3] | (p[4] << 8));
  r.final_rate = p[5];
  return r;
}

/* HalMAC C2H pkt: `c2h` = the report INCLUDING the 4-byte header
 * (cmd_id[7:0], seq[15:8], sub_cmd_id[23:16], len[31:24]). */
inline bool is_ccx_halmac(const uint8_t *c2h, size_t len) {
  return c2h != nullptr && len >= 16 && c2h[0] == 0xFF && c2h[2] == 0x0F;
}

inline TxReport parse_ccx_halmac(const uint8_t *c2h, size_t len) {
  TxReport r;
  if (!is_ccx_halmac(c2h, len))
    return r;
  auto dw = [&](size_t off) -> uint32_t {
    return static_cast<uint32_t>(c2h[off]) |
           (static_cast<uint32_t>(c2h[off + 1]) << 8) |
           (static_cast<uint32_t>(c2h[off + 2]) << 16) |
           (static_cast<uint32_t>(c2h[off + 3]) << 24);
  };
  const uint32_t d1 = dw(4), d2 = dw(8), d3 = dw(12);
  r.valid = true;
  r.qsel = (d1 >> 8) & 0x1f;
  r.missed_rpt = (d1 >> 13) & 0x7;
  r.macid = (d1 >> 16) & 0x7f;
  r.queue_time_raw = static_cast<uint16_t>(d2 & 0xffff);
  r.sw_define = (d2 >> 16) & 0xff;
  r.rts_retries = (d2 >> 24) & 0xf;
  r.bmc = (d2 >> 29) & 1;
  r.state = (d2 >> 30) & 0x3;
  r.data_retries = d3 & 0x3f;
  r.final_rate = (d3 >> 8) & 0x7f;
  return r;
}

/* Emit the `tx.report` machine event (schema: docs/logging.md). `fmt` names
 * the on-wire format ("8812" / "halmac"); sw_define/rts fields are
 * halmac-only and emitted only there. */
inline void emit_tx_report(EventSink &sink, const TxReport &r,
                           const char *fmt) {
  Ev ev(sink, "tx.report");
  ev.f("state", static_cast<unsigned>(r.state))
      .f("ok", r.state == 0)
      .f("retries", static_cast<unsigned>(r.data_retries))
      .f("final_rate", static_cast<unsigned>(r.final_rate))
      .f("queue_time_raw", static_cast<unsigned>(r.queue_time_raw))
      .f("bmc", r.bmc)
      .f("macid", static_cast<unsigned>(r.macid))
      .f("fmt", fmt);
  if (fmt[0] == 'h') /* halmac extras */
    ev.f("tag", static_cast<unsigned>(r.sw_define))
        .f("rts_retries", static_cast<unsigned>(r.rts_retries))
        .f("missed", static_cast<unsigned>(r.missed_rpt));
}

} /* namespace devourer */
