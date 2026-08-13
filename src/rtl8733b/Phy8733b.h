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
                            uint8_t max_target_qdbm);
  bool disable_tssi_tracking();
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
  static std::optional<std::array<int8_t, 20>>
  tssi_rate_offsets(const TxPowerTargets8733b &targets, uint8_t band,
                    uint8_t path, uint8_t max_target_qdbm = 0xff);
  static std::optional<TssiBbPlan8733b>
  tssi_bb_plan(const TxPowerTargets8733b &targets, uint8_t channel,
               uint8_t rfe_type, uint8_t path,
               uint8_t max_target_qdbm = 0xff);
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
  TxPowerTargets8733b _tx_power_targets{};
  std::optional<TssiBbState8733b> _tssi_digital_snapshot;
  std::optional<TssiAnalogState8733b> _tssi_analog_snapshot;
  bool _tssi_is_2g = true;
  bool _initialized = false;
};

} // namespace rtl8733b

#endif /* PHY_8733B_H */
