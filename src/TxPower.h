/* Runtime TX-power control types for the adaptive link.
 *
 * The TX-power lever has three knobs on IRtlDevice, all live (applied
 * immediately on a brought-up chip, recorded and applied at InitWrite
 * otherwise) and all sticky across channel switches:
 *
 *   - SetTxPowerOffsetQdb(qdb): adjust power RELATIVE to the efuse-calibrated
 *     per-rate table in quarter-dB units, preserving the calibrated per-rate
 *     shape until individual rates saturate at the hardware rails. This is
 *     the closed-loop controller's knob ("back off 2 dB, keep the shape").
 *   - SetTxPowerIndexOverride(idx): force a flat absolute TXAGC index across
 *     all rates (the debug/SDR-visibility knob; -1 reverts to the table).
 *   - SetTxPowerRateDiffs(table): REPLACE the calibrated per-rate shape with a
 *     caller-supplied one (see TxRateDiffsQdb below). The other two knobs keep
 *     composing on top of it.
 *
 * The first two compose: effective = clamp(baseline + offset_steps, 0,
 * index_max), where baseline is the flat index when set, else the per-rate
 * table value (the caller's when SetTxPowerRateDiffs supplied one).
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
#include <optional>

namespace devourer {

/* Static per-family capabilities of the TX-power knobs. `step_qdb` is the
 * quarter-dB value of one hardware index step (2 = 0.5 dB Jaguar1/2,
 * 1 = 0.25 dB Jaguar3/Kestrel); `step_measured` stays false until the family's
 * dB-per-step slope has been confirmed on-air (SDR least-squares ramp) —
 * until then the step is vendor-documentation truth, not measured truth.
 *
 * The `rate_diffs*` trio describes SetTxPowerRateDiffs. `rate_diffs_hw_table`
 * separates the two mechanisms, because they have different costs and
 * different reach: a hardware per-rate TXAGC table shapes every PPDU the chip
 * emits (host frames and hardware-timed beacons alike) at no per-frame cost,
 * while a software send-time fold (Kestrel's fixed-dBm BB target) only shapes
 * frames the host sequences, costs a register rewrite at each rate change, and
 * leaves a beacon airing between host frames at the last frame's level. */
struct TxPowerCaps {
  bool supported = false;
  uint8_t index_max = 0;       /* 63 (Jaguar1/2), 127 (Jaguar3), 0 = dBm model */
  uint8_t step_qdb = 0;        /* qdB per index step */
  bool step_measured = false;  /* on-air slope validated for this family */
  int16_t offset_min_qdb = 0;  /* most negative representable offset */
  int16_t offset_max_qdb = 0;  /* most positive representable offset */
  bool rate_diffs = false;          /* SetTxPowerRateDiffs is honoured */
  bool rate_diffs_hw_table = false; /* false = software send-time fold */
  bool rate_diffs_measured = false; /* on-air effect confirmed for this variant */
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

/* Caller-supplied per-rate TX-power diffs (signed qdB vs the reference anchor),
 * programmed by IRtlDevice::SetTxPowerRateDiffs. Motivating consumer: a
 * wall-equalized rate ladder — each rate parked a uniform margin below its
 * measured PA-compression wall.
 *
 * THE CONTRACT, identical on every generation that reports `rate_diffs`: the
 * table REPLACES the chip's calibrated per-rate shape. Every rate's level is
 *
 *     anchor + its caller diff (quantized to step_qdb) + the runtime offset
 *
 * clamped to the hardware rail. The `anchor` is the level the chip's own
 * default walk produces for the section reference rate HT MCS7 (1SS, current
 * channel and bandwidth), so a table of zeros is a no-op at MCS7 and flattens
 * every other rate onto MCS7's level. Rates this struct does not describe —
 * HT MCS8+, all VHT/HE, every 2SS+ stream — sit at the anchor.
 *
 * Caller diffs are NOT clamped to the regulatory tables: the same operator's-
 * call permissiveness the offset and flat-override knobs document above.
 *
 * Resolution is the family's step: one qdB is one index step on Jaguar3 and
 * Kestrel, but Jaguar1/Jaguar2 quantize to 0.5 dB, so an odd qdB rounds (ties
 * away from zero). Storage range is [-64, 63] everywhere — the width of the
 * Jaguar3 hardware diff field; SetTxPowerRateDiffs clamps before storing.
 *
 * Kestrel is the one family without a per-rate hardware table (`rate_diffs_
 * hw_table` false): its fixed-dBm BB target is rewritten per frame from the
 * frame's own rate, so there is no calibrated shape underneath and "replace"
 * reads as "add to the flat session target". See src/kestrel/CLAUDE.md for
 * the two consequences (per-rate-change register cost, and a hardware-timed
 * beacon inheriting the last host frame's level).
 *
 * Not to be confused with Jaguar1's FastSetTxPowerOffsetQdb: the BB-swing
 * TxScale it drives sits downstream of the per-rate TXAGC table and is
 * rate-independent, so the two compose but neither can implement the other. */
struct TxRateDiffsQdb {
  int8_t cck = 0;                      /* CCK 1..11M rows; [-64, 63] qdB */
  int8_t legacy = 0;                   /* OFDM 6..54M control frames; [-64, 63] */
  int8_t mcs[8] = {0, 0, 0, 0, 0, 0, 0, 0}; /* HT MCS0..7; [-64, 63] each */
};

/* Pack four signed per-rate diffs into one 0x3a00-table word: byte j =
 * (diff_j & 0x7f), the Jaguar3 7-bit two's-complement diff field. */
uint32_t pack_rate_diff_word(int8_t d0, int8_t d1, int8_t d2, int8_t d3);

/* Quantize one caller diff (qdB) to whole hardware index steps for a family
 * whose step is `step_qdb`, rounding to nearest with ties away from zero — the
 * same rule quantize_offset_qdb uses, so a request that vanishes on one knob
 * vanishes on the other rather than the two disagreeing. 0 for step_qdb 0. */
int rate_diff_steps(int qdb, uint8_t step_qdb);

/* The caller diff (qdB) that applies to one MGN_* rate: CCK 1..11M -> .cck,
 * OFDM 6..54M -> .legacy, HT MCS0..7 -> .mcs[n], everything else 0 (the anchor).
 * Takes the raw enum value so this header stays free of RateDefinitions.h. */
int rate_diff_qdb_for_rate(uint8_t mgn_rate, const TxRateDiffsQdb &d);

/* Clamp every field to the [-64, 63] storage range each family shares.
 * std::nullopt passes through (= restore the chip's calibrated shape). */
std::optional<TxRateDiffsQdb>
clamp_rate_diffs(const std::optional<TxRateDiffsQdb> &d);

} // namespace devourer

#endif /* DEVOURER_TX_POWER_H */
