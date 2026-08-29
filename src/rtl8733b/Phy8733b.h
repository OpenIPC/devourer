#ifndef PHY_8733B_H
#define PHY_8733B_H

#include <cstddef>
#include <cstdint>
#include <optional>

#include "Halmac8733bMac.h"
#include "PhyTableLoader.h"
#include "RtlAdapter.h"
#include "SelectedChannel.h"
#include "logger.h"

namespace rtl8733b {

struct PhyState {
  uint16_t sys_func_en = 0;
  uint8_t rf_ctrl = 0;
  uint32_t hci_opt_ctrl = 0;
  uint32_t bb_enable = 0;
  uint32_t wire_control = 0;
  uint32_t xtal_control = 0;
  uint32_t rf_a_57 = 0;
  uint32_t rf_b_57 = 0;
  uint32_t rf_a_87 = 0;
  uint32_t rf_b_87 = 0;
  uint32_t rf_a_ef = 0;
  uint32_t rf_b_ef = 0;
  uint32_t rf_b_66 = 0;

  bool matches(uint8_t xtal) const;
};

struct PhyTableCounts {
  size_t mac = 0;
  size_t phy = 0;
  size_t agc = 0;
  size_t power_targets = 0;
  size_t cal_init = 0;
  size_t radio_a = 0;
  size_t radio_b = 0;
};

struct ChannelPlan8733b {
  uint8_t primary = 0;
  uint8_t center = 0;
  uint8_t primary_index = 0;
  uint8_t offset = 0;
  ChannelWidth_t width = CHANNEL_WIDTH_20;
  bool is_2g = true;
  /* 5/10 MHz divider-mapping experiment overrides (DEVOURER_NB_DAC /
   * DEVOURER_NB_ADC): force the 0x9b4[10:8] DAC-divider / 0x9f0[3:0]
   * ADC-clock codes the experimental narrowband path writes. The codes the
   * vendor stub carries are measured not to narrow the emission (see
   * docs/rtl8733b.md "Narrowband status"); the encoding is die-specific and
   * unmapped, and the Jaguar3 precedent is that the vendor's own table was
   * wrong on silicon until an SDR sweep found the live codes. Unset = the
   * stub values, so both the writes and the readback verifier track. */
  std::optional<uint8_t> nb_dac;
  std::optional<uint8_t> nb_adc;

  /* Masked to the register field widths (0x9b4[10:8] / 0x9f0[3:0]) so the
   * write and the readback verifier agree for any override value. */
  uint8_t dac_code() const {
    return (nb_dac ? *nb_dac : (width == CHANNEL_WIDTH_5 ? 1u : 2u)) & 0x7u;
  }
  uint8_t adc_code() const {
    return (nb_adc ? *nb_adc : (width == CHANNEL_WIDTH_5 ? 0xau : 0xbu)) &
           0xfu;
  }
};

struct ChannelState8733b {
  uint32_t rf_a_18 = 0;
  uint32_t rf_b_18 = 0;
  uint32_t rf_a_19 = 0;
  uint32_t rf_b_19 = 0;
  uint32_t bb_9b0 = 0;
  uint32_t bb_9b4 = 0;
  uint32_t bb_9f0 = 0;
  uint32_t bb_81c = 0;
  uint8_t data_sc = 0;
  uint32_t wmac_trxptcl = 0;
  bool synth_ready = false;

  bool matches(const ChannelPlan8733b &plan) const;
};

/* The 8733B uses one 7-bit reference for CCK and one for OFDM/HT on each RF
 * path, plus a shared signed 7-bit per-rate difference table.  Keep initial
 * injection at a deliberately low, flat index until the per-unit EFUSE power
 * walk and regulatory limit tables are ported.  This is a TX gain index, not
 * a claim of calibrated dBm. */
inline constexpr uint8_t kSafeTxAgcIndex8733b = 0x10;
/* First-light ceiling for closed-loop TSSI.  The hardware target table is in
 * quarter-dBm, so 64 represents 16 dBm.  This remains deliberately below the
 * per-unit factory targets until calibrated power is verified on a conducted
 * setup. */
inline constexpr uint8_t kSafeTssiTargetQdbm8733b = 64;

/* Highest per-rate target in the compiled PG table: 80 qdBm = 20 dBm at
 * 2.4 GHz (5 GHz tops out at 76 = 19 dBm), against the 64 qdBm safe clip
 * above.  Not a limit on anything — the runtime TX-power offset can be
 * commanded past it, deliberately (see Rtl8733bDevice::GetTxPowerCaps) — but
 * it is the reference point that says where the vendor's calibration ends and
 * the operator's own begins.  Pinned against the generated table by
 * tests/rtl8733b_txpwr_selftest.cpp so the figure quoted in the docs cannot
 * drift away from the data. */
inline constexpr uint8_t kMaxPgTargetQdbm8733b = 80;

/* Which rail the runtime TX-power offset clamped at, if any — the signal a
 * closed-loop controller uses to know the knob has run out of travel
 * (IRtlDevice::GetTxPowerState).  `low` is set when a rate's shifted target hit
 * the int8 delta field's -128 floor — a shifted target below -64 qdBm, i.e.
 * -16 dBm — and deliberately NOT at the 0 qdBm target, which the loop keeps
 * responding past by ~7 dB; `high` when a rate hit the field's +127 ceiling.
 * Both rails are that field and nothing softer, and both are per-rate facts:
 * a shift can rail one rate while the rest still move, which
 * is exactly what a shape-preserving offset does at the end of its range. */
struct TssiOffsetSat8733b {
  bool low = false;
  bool high = false;
};

struct TxAgcState8733b {
  uint8_t cck_ref_a = 0;
  uint8_t cck_ref_b = 0;
  uint8_t ofdm_ref_a = 0;
  uint8_t ofdm_ref_b = 0;
  std::array<uint32_t, 5> rate_diffs{};

  bool matches_flat(uint8_t index) const;
};

struct TssiDePlan8733b {
  std::array<int8_t, 2> cck{};
  std::array<int8_t, 2> ht40{};
  std::array<int8_t, 2> ofdm{};
  std::array<int8_t, 2> ht20{};

  bool operator==(const TssiDePlan8733b &) const = default;
};

struct TssiDeState8733b {
  TssiDePlan8733b de{};
  uint32_t path_a_lanes = 0;
  uint32_t path_b_lanes = 0;
  bool enabled = false;

  bool matches_disabled(const TssiDePlan8733b &expected) const;
};

struct TxPowerTargets8733b {
  /* [band: 0=2G, 1=5G][path][hardware rate 0..19], in quarter-dBm. */
  std::array<std::array<std::array<uint8_t, 20>, 2>, 2> qdbm{};
  std::array<std::array<bool, 2>, 2> present{};

  bool usable() const {
    return present[0][0] && present[0][1] && present[1][0];
  }
};

struct TssiBbPlan8733b {
  std::array<uint32_t, 5> rate_offsets{};
  uint8_t path = 0;
  uint32_t reg_4304 = 0;
  uint32_t reg_4308 = 0;
  uint32_t reg_430c = 0;
  uint32_t reg_4320 = 0;
  uint32_t reg_4328 = 0;
  uint32_t reg_432c = 0;
  uint32_t reg_438c = 0;
  uint32_t reg_4390 = 0;
  uint32_t reg_4394 = 0;
  uint32_t reg_4398 = 0;
  uint32_t reg_439c = 0;
  uint32_t reg_43a8 = 0;
};

struct TssiBbState8733b : TssiBbPlan8733b {
  bool enabled = false;

  bool matches_disabled(const TssiBbPlan8733b &expected) const;
};

struct TssiThermalPlan8733b {
  std::array<uint32_t, 16> offsets{};
  uint8_t baseline = 0;
  bool cck = false;

  uint32_t control() const {
    return 0x3u | (static_cast<uint32_t>(baseline) << 4);
  }
};

struct TssiThermalState8733b {
  std::array<uint32_t, 16> offsets{};
  uint32_t control = 0;
  bool enabled = false;

  bool matches_disabled(const TssiThermalPlan8733b &expected) const;
};

struct TssiAnalogState8733b {
  uint32_t bb_1860 = 0;
  uint32_t bb_1c38 = 0;
  uint32_t bb_1d40 = 0;
  uint32_t bb_1e1c = 0;
  uint32_t bb_1ca4 = 0;
  std::array<uint32_t, 2> rf_55{};
  std::array<uint32_t, 2> rf_7f{};
  bool enabled = false;

  bool matches_setup() const;
  bool matches_snapshot(const TssiAnalogState8733b &expected) const;
};

/* RTL8733B PHYDM baseline. This is deliberately limited to the immutable
 * MAC/BB/AGC/RFK-init/RF parameter images and register access. Channel
 * programming and live IQK/DPK calibration are later, separately gated
 * stages. */
class Phy8733b {
public:
  Phy8733b(RtlAdapter device, Logger_t logger);

  /* DEVOURER_NB_DAC / DEVOURER_NB_ADC — see ChannelPlan8733b::nb_dac. */
  void set_narrowband_overrides(std::optional<uint8_t> dac,
                                std::optional<uint8_t> adc) {
    _nb_dac = dac;
    _nb_adc = adc;
  }

  bool initialize(uint8_t cut, const EfuseInfo &efuse);
  PhyState read_state();
  bool set_channel(SelectedChannel channel);
  ChannelState8733b read_channel_state();
  bool set_flat_tx_power(uint8_t index);
  TxAgcState8733b read_txagc_state();
  bool prepare_tssi_bb(SelectedChannel channel, const EfuseInfo &efuse);
  TssiBbState8733b read_tssi_bb_state();
  bool prepare_tssi_thermal(const EfuseInfo &efuse, bool cck);
  TssiThermalState8733b read_tssi_thermal_state();
  bool audit_tssi_analog(SelectedChannel channel, const EfuseInfo &efuse);
  TssiAnalogState8733b read_tssi_analog_state();
  bool audit_tssi_enable(SelectedChannel channel, const EfuseInfo &efuse,
                         uint8_t max_target_qdbm);
  bool enable_tssi_tracking(SelectedChannel channel, const EfuseInfo &efuse,
                            uint8_t max_target_qdbm, int offset_qdb = 0);
  bool disable_tssi_tracking();
  /* Runtime TX-power actuator (IRtlDevice::SetTxPowerOffsetQdb). On a
   * TSSI-offset PG unit the closed loop IS the TX-power control, so moving
   * power means moving the loop's per-rate target table: the five packed
   * dwords at 0x3a00..0x3a10, rewritten IN PLACE with tracking left enabled —
   * the #389 shape fast_retune already uses for its per-channel rewrite, not
   * the ~165 ms disable/re-enable dance. Everything that can refuse is
   * computed before the first chip write, so a declined call leaves the
   * target untouched; a failed readback rolls back to the offsets the chip
   * was carrying. Returns false when tracking is not live (nothing to
   * retarget) or the plan does not resolve for this channel.
   *
   * The loop needs settling time — see docs/rtl8733b.md — so a caller
   * sweeping offsets must pace, or it measures the tracking loop rather than
   * the knob. */
  bool set_tssi_offset(SelectedChannel channel, uint8_t max_target_qdbm,
                       int offset_qdb, TssiOffsetSat8733b *sat = nullptr);
  /* Does the chip's live per-rate target table match the plan for the offset
   * the CALLER believes is applied? Recomputed from (channel, ceiling, offset)
   * rather than compared against this class's own `_fr_tssi_offsets` shadow,
   * so a disagreement between the device's session state and the PHY's
   * bookkeeping is caught too — a shadow checked against itself always agrees,
   * which is the whole failure this API is fixing. Six register reads (the
   * 0x3a00 dwords via read_txagc_state); the chip-truth half of
   * GetTxPowerState on the TSSI path. */
  bool tssi_offsets_confirmed(SelectedChannel channel, uint8_t max_target_qdbm,
                              int offset_qdb);
  /* Lean intra-band, same-bandwidth hop — the FastRetune core (see
   * docs/frequency-hopping.md; profile that sized it: full set_channel on
   * this USB-HS part is ~330 ms, of which ~165 ms is the TSSI
   * disable/re-enable dance and ~90 ms the band/bandwidth switches a hop
   * does not need). The subset a hop DOES need: the RF18 synth program
   * (bandwidth bits preserved from the cached word), RF19 sub-band bits and
   * the channel-keyed BB constants (AGC bucket, SCO fc) only when their
   * bucket changes, then the BB reset + IGI toggle that restart the RX
   * engine. TSSI tracking stays ENABLED across the hop; the per-channel
   * rate-offset dwords and the channel-bucketed TSSI-DE offsets are
   * rewritten in place, each only when the plan for the new channel
   * differs — the in-place-rewrite-with-tracking-live shape validated in
   * #389. 2.4 GHz hops also rewrite the per-channel SCO fc words and the
   * ch13/ch14-keyed constants when crossed.
   *
   * cache_rf=true runs write-only from words primed by one read each on the
   * first fast hop after a full set; cache_rf=false re-reads every hop (the
   * A/B knob measuring the read penalty). Returns false — chip untouched —
   * on a band or width change or when the radio was never tuned; the caller
   * falls back to the full set_channel. */
  bool fast_retune(SelectedChannel channel, bool tssi_live,
                   uint8_t max_target_qdbm, bool cache_rf,
                   int offset_qdb = 0);
  bool prepare_tssi_offsets(SelectedChannel channel, const EfuseInfo &efuse);
  TssiDeState8733b read_tssi_de_state();
  uint8_t read_thermal();

  uint32_t read_rf(uint8_t path, uint8_t reg, uint32_t mask = 0x000fffff);
  bool write_rf(uint8_t path, uint8_t reg, uint32_t mask, uint32_t data);

  static JaguarPhyContext table_context(uint8_t cut, uint8_t rfe_type);
  static PhyTableCounts count_selected_tables(uint8_t cut, uint8_t rfe_type);
  static std::optional<ChannelPlan8733b>
  channel_plan(SelectedChannel channel);
  static std::optional<TssiDePlan8733b>
  tssi_de_plan(const TssiPowerInfo8733b &power, uint8_t channel);
  static bool parse_tx_power_targets(const uint32_t *table, size_t len,
                                     TxPowerTargets8733b &out);
  /* Per-rate closed-loop targets as int8 deltas from the 64 qdBm anchor.
   * `max_target_qdbm` is the safety clip; `offset_qdb` is the runtime knob,
   * and its two halves do different things because offset 0 sits on that clip
   * rather than on a hardware rail (the asymmetry is argued at the
   * implementation): a NEGATIVE offset shifts what survives the clip, which
   * preserves the calibrated per-rate shape src/TxPower.h promises, while a
   * POSITIVE offset raises the clip and each rate rises only as far as its own
   * factory target. offset_qdb = 0 reproduces the pre-runtime-knob table byte
   * for byte. */
  static std::optional<std::array<int8_t, 20>>
  tssi_rate_offsets(const TxPowerTargets8733b &targets, uint8_t band,
                    uint8_t path, uint8_t max_target_qdbm = 0xff,
                    int offset_qdb = 0, TssiOffsetSat8733b *sat = nullptr);
  static std::optional<TssiBbPlan8733b>
  tssi_bb_plan(const TxPowerTargets8733b &targets, uint8_t channel,
               uint8_t rfe_type, uint8_t path,
               uint8_t max_target_qdbm = 0xff, int offset_qdb = 0,
               TssiOffsetSat8733b *sat = nullptr);
  static TssiThermalPlan8733b tssi_thermal_plan(uint8_t efuse_thermal,
                                                bool cck);

private:
  void set_bb(uint16_t reg, uint32_t mask, uint32_t data);
  uint32_t get_bb(uint16_t reg, uint32_t mask);
  void bb_table_write(uint32_t addr, uint32_t data);
  void rf_table_write(uint8_t path, uint32_t addr, uint32_t data);
  void parameter_gate(bool post);
  void set_crystal_cap(uint8_t xtal);
  void bb_reset();
  void igi_toggle();
  bool program_synth(uint32_t rf18);
  bool switch_band(const ChannelPlan8733b &plan);
  bool switch_channel(const ChannelPlan8733b &plan);
  bool switch_bandwidth(const ChannelPlan8733b &plan);
  void spur_cancellation();
  void write_tssi_anapar(bool tssi, bool is_2g);
  void apply_tssi_analog(bool is_2g,
                         std::array<uint32_t, 2> *rf_7f_immediate = nullptr);
  void restore_tssi_analog(const TssiAnalogState8733b &snapshot,
                           bool is_2g);

  RtlAdapter _device;
  Logger_t _logger;
  uint8_t _cut = 0;
  uint8_t _rfe_type = 0xff;
  std::optional<uint8_t> _nb_dac;
  std::optional<uint8_t> _nb_adc;

  /* channel_plan stays a pure static (the selftests exercise it); this is
   * the instance flavour that stamps the divider-experiment overrides so
   * the writes and the readback verifier agree. */
  std::optional<ChannelPlan8733b> planned(SelectedChannel channel) const {
    auto plan = channel_plan(channel);
    if (plan) {
      plan->nb_dac = _nb_dac;
      plan->nb_adc = _nb_adc;
    }
    return plan;
  }
  TxPowerTargets8733b _tx_power_targets{};
  std::optional<TssiBbState8733b> _tssi_digital_snapshot;
  std::optional<TssiAnalogState8733b> _tssi_analog_snapshot;
  bool _tssi_is_2g = true;
  bool _initialized = false;

  /* fast_retune state. _fr_plan is the channel the radio is actually on,
   * recorded by every successful set_channel/fast_retune — the base for the
   * same-band/same-width admission and the bucket-change comparisons.
   * _fr_rf18/_fr_rf19 are the compose cache (primed on the first fast hop;
   * invalidated by every full set_channel, whose switch functions rewrite
   * both words). _fr_tssi_offsets is the rate-offset table the chip
   * currently carries, recorded by enable_tssi_tracking and by every
   * in-place fast-hop rewrite; _fr_tssi_path is the RF path bit read at
   * enable time (RFE routing — static per session). _fr_tssi_de is the DE
   * plan the chip currently carries and _fr_tssi_power the EFUSE
   * calibration that derives it, recorded by prepare_tssi_offsets — the DE
   * buckets are only ~3 channels wide at 2.4 GHz, so most hops cross one
   * and must rewrite the DE registers or track with the previous
   * channel's calibration. */
  std::optional<ChannelPlan8733b> _fr_plan;
  std::optional<uint32_t> _fr_rf18;
  std::optional<uint32_t> _fr_rf19;
  std::optional<std::array<uint32_t, 5>> _fr_tssi_offsets;
  uint8_t _fr_tssi_path = 0;
  std::optional<TssiDePlan8733b> _fr_tssi_de;
  std::optional<TssiPowerInfo8733b> _fr_tssi_power;
};

} // namespace rtl8733b

#endif /* PHY_8733B_H */
