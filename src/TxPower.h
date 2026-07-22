/* Runtime TX-power control types for the adaptive link.
 *
 * The TX-power lever has two knobs on IRtlDevice, both live (applied
 * immediately on a brought-up chip, recorded and applied at InitWrite
 * otherwise) and both sticky across channel switches:
 *
 *   - SetTxPowerOffsetQdb(qdb): adjust power RELATIVE to the efuse-calibrated
 *     per-rate table in quarter-dB units, preserving the calibrated per-rate
 *     shape until individual rates saturate at the hardware rails. This is
 *     the closed-loop controller's knob ("back off 2 dB, keep the shape").
 *   - SetTxPowerIndexOverride(idx): force a flat absolute TXAGC index across
 *     all rates (the debug/SDR-visibility knob; -1 reverts to the table).
 *
 * They compose: effective = clamp(baseline + offset_steps, 0, index_max),
 * where baseline is the flat index when set, else the per-rate table value.
 * The offset folds AFTER the per-rate regulatory min() (worldwide-min
 * txpwr_lmt tables where the family has them), clamped only at the hardware
 * max — headroom above the generated table is deliberate; compliance is the
 * operator's call, exactly as with the flat override.
 *
 * Units: one TXAGC index step is family-specific (0.5 dB on Jaguar1/Jaguar2's
 * 6-bit index, 0.25 dB on Jaguar3's 7-bit reference), so the API speaks
 * quarter-dB (qdB) and reports what was actually applied after quantization.
 */
#ifndef DEVOURER_TX_POWER_H
#define DEVOURER_TX_POWER_H

#include <cstdint>

namespace devourer {

/* Static per-family capabilities of the TX-power knobs. `step_qdb` is the
 * quarter-dB value of one hardware index step (2 = 0.5 dB Jaguar1/2,
 * 1 = 0.25 dB Jaguar3); `step_measured` stays false until the family's
 * dB-per-step slope has been confirmed on-air (SDR least-squares ramp) —
 * until then the step is vendor-documentation truth, not measured truth. */
struct TxPowerCaps {
  bool supported = false;
  uint8_t index_max = 0;       /* 63 (Jaguar1/2), 127 (Jaguar3) */
  uint8_t step_qdb = 0;        /* qdB per index step */
  bool step_measured = false;  /* on-air slope validated for this family */
  int16_t offset_min_qdb = 0;  /* most negative representable offset */
  int16_t offset_max_qdb = 0;  /* most positive representable offset */
};

/* Snapshot of the TX-power knobs plus representative effective indices.
 * `flat_index` = -1 means the efuse per-rate baseline is active. The three
 * representative indices are path-A values for CCK 1M, OFDM 6M and HT MCS7;
 * `hw_readback` says whether they were read back from the TXAGC registers
 * (8814A's packed TXAGC port is write-only, so it reports the software
 * shadow with hw_readback=false). `saturated_low/high` are set when the
 * last apply clamped at least one rate at a rail — the signal a closed-loop
 * controller uses to know the knob has run out of travel. */
struct TxPowerState {
  bool valid = false;
  int16_t flat_index = -1;
  int16_t offset_qdb = 0;   /* quantized offset currently folded, in qdB */
  int16_t offset_steps = 0; /* same, in hardware index steps */
  bool saturated_low = false;
  bool saturated_high = false;
  int16_t cck_index = -1;
  int16_t ofdm_index = -1;
  int16_t mcs7_index = -1;
  bool hw_readback = false;
  bool rate_diffs_custom = false; /* A caller-supplied per-rate diff table is
                                    * CONFIGURED (SetTxPowerRateDiffs), not
                                    * necessarily live on the chip right now:
                                    * a flat override (flat_index >= 0)
                                    * temporarily flattens the chip's per-rate
                                    * table to zero diffs, and re-applies the
                                    * configured table once the override
                                    * clears. The cck/ofdm/mcs7_index summary
                                    * fields always report chip truth for the
                                    * current moment (flat during an override,
                                    * ref+diff otherwise). */
};

/* Quantize a quarter-dB offset request to a family's step size: round to
 * nearest step, ties away from zero, then clamp to the caps' offset range.
 * Returns the applied qdB (what SetTxPowerOffsetQdb reports); the step count
 * lands in *steps_out when non-null. Returns 0 for unsupported caps. */
int quantize_offset_qdb(int qdb, const TxPowerCaps &caps, int *steps_out);

/* Per-packet TX-power LUT quantizer, shared by the families whose TX
 * descriptor carries the 3-bit hardware TXPWR_OFSET LUT (Jaguar2 8822B/8821C
 * at txdesc+0x14[30:28]; 8814A same position via its own descriptor macro).
 * The field is discrete — {0, -3, -7, -11, +3, +6} dB — so an adaptive
 * link's continuous request lands on the closest rung. Pure; unit-tested in
 * tests/txpkt_pwr_selftest.cpp. */
inline uint8_t txpkt_pwr_step_for_db(int db) {
  static const struct { int db; uint8_t step; } lut[] = {
      {0, 0}, {-3, 1}, {-7, 2}, {-11, 3}, {3, 4}, {6, 5}};
  uint8_t best = 0;
  int best_err = 1 << 30;
  for (const auto &e : lut) {
    int err = db > e.db ? db - e.db : e.db - db;
    if (err < best_err) {
      best_err = err;
      best = e.step;
    }
  }
  return best;
}

/* Inverse: the nominal dB of a LUT step (for readback / logging). */
inline int txpkt_pwr_db_for_step(uint8_t step) {
  static const int db[] = {0, -3, -7, -11, 3, 6};
  return step < 6 ? db[step] : 0;
}

/* Caller-supplied per-rate TXAGC diffs (signed qdB vs the reference anchor).
 * Motivating consumer: a wall-equalized rate ladder — each rate parked a
 * uniform margin below its measured PA-compression wall. Programmed by
 * IRtlDevice::SetTxPowerRateDiffs; 8822E-only. On the 8822E one qdB equals one
 * TXAGC index step (step_qdb = 1), so values are used verbatim as index diffs.
 * The hardware diff field is 7-bit two's-complement, so the usable range is
 * [-64, 63]; SetTxPowerRateDiffs clamps to it before storing. */
struct TxRateDiffsQdb {
  int8_t cck = 0;                      /* CCK 1..11M rows; [-64, 63] qdB */
  int8_t legacy = 0;                   /* OFDM 6..54M control frames; [-64, 63] */
  int8_t mcs[8] = {0, 0, 0, 0, 0, 0, 0, 0}; /* HT MCS0..7; [-64, 63] each */
};

/* Pack four signed per-rate diffs into one 0x3a00-table word: byte j =
 * (diff_j & 0x7f), the 8822E's 7-bit two's-complement diff field. */
uint32_t pack_rate_diff_word(int8_t d0, int8_t d1, int8_t d2, int8_t d3);

} // namespace devourer

#endif /* DEVOURER_TX_POWER_H */
