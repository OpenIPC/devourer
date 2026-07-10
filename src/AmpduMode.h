#pragma once

/* AmpduMode — the first-class 802.11 A-MPDU TX session knob, bundling the
 * recipe the spike + pacing sweep proved on-air (docs/aggregation.md). It
 * replaces the raw per-field spike knobs (DEVOURER_TX_QSEL / DEVOURER_TX_AMPDU,
 * which stay as a lower-level experimentation layer that composes on top).
 *
 * Two halves, both required for net goodput above plain singles:
 *   1. Per-frame descriptor: a data QSEL (aggregation only forms on a data
 *      queue, never the monitor-inject MGMT queue 0x12), AGG_EN, MAX_AGG_NUM,
 *      AMPDU_DENSITY, and — for the broadcast/no-BlockAck-peer case that
 *      OpenIPC wfb wants — a per-frame retry limit of 0 (otherwise the MAC
 *      re-airs each aggregate to the retry limit waiting for a BlockAck no
 *      one sends: 92% wasted re-airings).
 *   2. MAC pacing registers (SetAmpduMode programs these live): the
 *      aggregate-fill timer REG_AMPDU_MAX_TIME (0x0455 on the HalMAC chips,
 *      0x0456 on Jaguar1) and the burst-mode gate REG_SW_AMPDU_BURST_MODE_CTRL
 *      (0x04BC). The bring-up timer value 0x70 paces each aggregate launch at
 *      ~3 ms — 0x20 unlocks it to ~0.8 ms (the register is a CLIFF: values
 *      <= 0x08 disable aggregation entirely). Clearing 0x04BC BIT6 (halmac
 *      sets it) is worth ~+40% on the aggregated path (8822B-proven).
 *
 * The caller still has to keep the TX queue fed deep enough for the MAC to
 * have frames to aggregate — a shallow feed makes it SIFS-burst single-MPDU
 * aggregates (txdemo DEVOURER_TX_THREADS is the bench lever; a real feeder
 * pushes via send_packets with enough in flight). See docs/aggregation.md. */

#include <cstdint>
#include <cstdlib>

namespace devourer {

struct AmpduMode {
  bool enabled = false; /* master switch — off keeps every TX byte-identical */

  /* --- descriptor half --- */
  uint8_t tid = 0;      /* QSEL / TID the aggregatable frames ride (0..7) */
  uint8_t max_num = 16; /* MAX_AGG_NUM: max MPDUs per A-MPDU (1..0x1f) */
  /* AMPDU_DENSITY (0..7 = min MPDU start spacing). Counter-intuitively 7 (the
   * most conservative spacing) is bench-fastest here — a permissive density
   * makes the MAC form well-shaped aggregates; density 0 measured ~10% slower.
   * Default 7 so a minimal "tid/max" spec is the full proven recipe. */
  uint8_t density = 7;
  /* Broadcast / no-BlockAck-peer flavor: per-frame retry limit 0, so each
   * aggregate airs exactly once (FEC covers loss, not ARQ). false = normal
   * retry limit (only sensible against a real BlockAck responder). */
  bool no_ack = true;

  /* --- MAC pacing half (programmed by SetAmpduMode) --- */
  /* REG_AMPDU_MAX_TIME value. 0 = leave the bring-up default (0x70, ~3 ms).
   * 0x20 is the proven unlock; the register is a cliff — <= 0x08 kills
   * aggregation. */
  uint8_t max_time = 0x20;
  /* Clear REG_SW_AMPDU_BURST_MODE_CTRL BIT6 (halmac bring-up sets it).
   * 8822B-proven +40%; applied where the register exists (J2 / 8814A). */
  bool clear_burst_mode = true;
};

/* Parse "tid/maxnum[/density[/noack[/maxtime_hex]]]" into an AmpduMode
 * (enabled=true). Returns false on a malformed leading field. Used by the
 * demos' DEVOURER_TX_AMPDU_MODE env front-end; the library itself takes the
 * struct. Examples: "0/16" (TID0, 16 MPDUs, defaults), "0/16/0/1/20"
 * (explicit no-ack, max_time 0x20). */
inline bool parse_ampdu_mode(const char *spec, AmpduMode &out) {
  if (spec == nullptr || *spec == '\0')
    return false;
  char *end = nullptr;
  long tid = std::strtol(spec, &end, 0);
  if (tid < 0 || tid > 7 || end == spec)
    return false;
  AmpduMode m;
  m.enabled = true;
  m.tid = static_cast<uint8_t>(tid);
  if (end && *end == '/') {
    long mx = std::strtol(end + 1, &end, 0);
    if (mx > 0)
      m.max_num = static_cast<uint8_t>(mx & 0x1f);
    if (end && *end == '/') {
      m.density = static_cast<uint8_t>(std::strtol(end + 1, &end, 0) & 0x7);
      if (end && *end == '/') {
        m.no_ack = std::strtol(end + 1, &end, 0) != 0;
        if (end && *end == '/')
          m.max_time = static_cast<uint8_t>(std::strtol(end + 1, nullptr, 0));
      }
    }
  }
  out = m;
  return true;
}

} /* namespace devourer */
