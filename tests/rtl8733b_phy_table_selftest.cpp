#include <cstdint>
#include <cstdio>
#include <utility>
#include <vector>

#include "PhyTableLoader.h"
#include "hal8733b_tables.h"
#include "rtl8733b/Phy8733b.h"

namespace {
int failures = 0;

void expect(const char *what, bool condition) {
  if (condition)
    return;
  ++failures;
  std::printf("FAIL: %s\n", what);
}
} // namespace

int main() {
  expect("generated MAC word count", array_mp_8733b_mac_reg_len == 36);
  expect("generated PHY word count", array_mp_8733b_phy_reg_len == 2068);
  expect("generated AGC word count", array_mp_8733b_agc_tab_len == 1668);
  expect("generated RF-A word count", array_mp_8733b_radioa_len == 170);
  expect("generated RF-B word count", array_mp_8733b_radiob_len == 64);
  expect("generated RFK-init word count",
         array_mp_8733b_cal_init_len == 2298);

  rtl8733b::TxPowerTargets8733b targets;
  expect("generated target-power table parses",
         rtl8733b::Phy8733b::parse_tx_power_targets(
             array_mp_8733b_phy_reg_pg, array_mp_8733b_phy_reg_pg_len,
             targets));
  expect("required target-power paths present", targets.usable());
  expect("2G target rates decoded",
         targets.qdbm[0][0][0] == 0x50 &&
             targets.qdbm[0][0][19] == 0x48 &&
             targets.qdbm[0][1][19] == 0x48);
  expect("5G target rates decoded without CCK",
         targets.qdbm[1][0][0] == 0xff &&
             targets.qdbm[1][0][4] == 0x4c &&
             targets.qdbm[1][0][19] == 0x40 &&
             !targets.present[1][1]);
  const auto offsets_2g =
      rtl8733b::Phy8733b::tssi_rate_offsets(targets, 0, 0);
  const auto offsets_5g =
      rtl8733b::Phy8733b::tssi_rate_offsets(targets, 1, 0);
  expect("TSSI rate offsets use 16 dBm origin",
         offsets_2g && (*offsets_2g)[0] == 16 &&
             (*offsets_2g)[19] == 8 && offsets_5g &&
             (*offsets_5g)[0] == 0 && (*offsets_5g)[4] == 12);
  expect("absent TSSI target path rejected",
         !rtl8733b::Phy8733b::tssi_rate_offsets(targets, 1, 1));
  const auto capped_offsets =
      rtl8733b::Phy8733b::tssi_rate_offsets(targets, 0, 0, 64);
  expect("TSSI target ceiling is applied in quarter-dBm",
         capped_offsets && (*capped_offsets)[0] == 0 &&
             (*capped_offsets)[19] == 0);
  const auto tssi_bb_2g =
      rtl8733b::Phy8733b::tssi_bb_plan(targets, 6, 0, 0);
  expect("2G TSSI digital plan",
         tssi_bb_2g && tssi_bb_2g->path == 0 &&
             tssi_bb_2g->rate_offsets[0] == 0x10101010 &&
             tssi_bb_2g->rate_offsets[4] == 0x0808080c &&
             tssi_bb_2g->reg_4304 == 0 &&
             tssi_bb_2g->reg_4328 == 0x03280200 &&
             tssi_bb_2g->reg_43a8 == 0x77470d00);
  const auto tssi_bb_rfe4 =
      rtl8733b::Phy8733b::tssi_bb_plan(targets, 6, 4, 1);
  expect("2G path-B RFE TSSI digital plan",
         tssi_bb_rfe4 && tssi_bb_rfe4->reg_4304 == 0x00008080 &&
             tssi_bb_rfe4->reg_43a8 == 0x7747fd00);
  const auto tssi_bb_capped =
      rtl8733b::Phy8733b::tssi_bb_plan(targets, 6, 0, 0, 64);
  expect("16 dBm TSSI digital plan is flat",
         tssi_bb_capped && tssi_bb_capped->rate_offsets[0] == 0 &&
             tssi_bb_capped->rate_offsets[4] == 0);
  expect("absent 5G path-B TSSI digital plan rejected",
         !rtl8733b::Phy8733b::tssi_bb_plan(targets, 36, 0, 1));
  const auto tssi_thermal_ofdm =
      rtl8733b::Phy8733b::tssi_thermal_plan(0x20, false);
  expect("OFDM/HT TSSI thermal table is flat",
         tssi_thermal_ofdm.control() == 0x203 &&
             tssi_thermal_ofdm.offsets[0] == 0 &&
             tssi_thermal_ofdm.offsets[15] == 0);
  const auto tssi_thermal_cck =
      rtl8733b::Phy8733b::tssi_thermal_plan(0xff, true);
  expect("CCK TSSI thermal table follows generated swings",
         tssi_thermal_cck.baseline == 0x20 &&
             tssi_thermal_cck.offsets[4] == 0xfdfd0000 &&
             tssi_thermal_cck.offsets[5] == 0xfbfbfbfb &&
             tssi_thermal_cck.offsets[11] == 0x00030305 &&
             tssi_thermal_cck.offsets[15] == 0);
  const uint32_t malformed_targets[] = {0, 0, 1, 0xc20, 0xffffffff,
                                        0x50505050};
  expect("malformed target-power record rejected",
         !rtl8733b::Phy8733b::parse_tx_power_targets(
             malformed_targets,
             sizeof(malformed_targets) / sizeof(malformed_targets[0]),
             targets));

  const rtl8733b::PhyTableCounts counts =
      rtl8733b::Phy8733b::count_selected_tables(3, 0);
  expect("all MAC rows selected", counts.mac == 18);
  expect("conditional PHY rows selected",
         counts.phy > 900 && counts.phy < array_mp_8733b_phy_reg_len / 2);
  expect("all AGC rows selected", counts.agc == 834);
  expect("all target-power rows selected", counts.power_targets == 14);
  expect("all RFK-init rows selected", counts.cal_init == 1149);
  expect("all RF-A rows selected", counts.radio_a == 85);
  expect("all RF-B rows selected", counts.radio_b == 32);

  /* Small golden for the old PHYDM IF / ELSE_IF / ELSE encoding used by the
   * one conditional region in phy_reg. Cut D + RFE 0 selects the first row;
   * the same cut + RFE 1 skips it and selects ELSE_IF. */
  const uint32_t conditional[] = {
      0x83000000, 0, 0x40000000, 0, 0x100, 0xaaaa,
      0x93000001, 0, 0x40000000, 0, 0x100, 0xbbbb,
      0xa0000000, 0, 0x100, 0xcccc, 0xb0000000, 0,
  };
  auto walk = [&](uint8_t rfe) {
    std::vector<std::pair<uint32_t, uint32_t>> writes;
    PhyTableLoader::Load(
        conditional, sizeof(conditional) / sizeof(conditional[0]),
        rtl8733b::Phy8733b::table_context(3, rfe),
        [&writes](uint32_t a, uint32_t v) { writes.emplace_back(a, v); });
    return writes;
  };
  const auto rfe0 = walk(0);
  const auto rfe1 = walk(1);
  const auto rfe2 = walk(2);
  expect("cut-D RFE-0 IF branch",
         rfe0.size() == 1 && rfe0[0].second == 0xaaaa);
  expect("cut-D RFE-1 ELSE_IF branch",
         rfe1.size() == 1 && rfe1[0].second == 0xbbbb);
  expect("unmatched RFE ELSE branch",
         rfe2.size() == 1 && rfe2[0].second == 0xcccc);

  rtl8733b::PhyState state;
  const uint32_t xtal = 0x46;
  state.sys_func_en = 0x3;
  state.rf_ctrl = 0x7;
  state.hci_opt_ctrl = 7u << 24;
  state.bb_enable = 0x3;
  state.wire_control = 0x10000003;
  state.xtal_control = ((xtal | (xtal << 7)) << 10) & 0x00fffc00;
  /* 0x57 and 0x66 expose live analog state after RF enable, while stable
   * control fields retain the table's path-specific mode selectors. */
  state.rf_a_57 = 0xd48d;
  state.rf_b_57 = 0xd485;
  state.rf_a_87 = 0x7;
  state.rf_b_87 = 0x3;
  state.rf_a_ef = state.rf_b_ef = 0;
  expect("PHY ready-state predicate", state.matches(xtal));
  state.rf_b_87 = 0x7;
  expect("PHY predicate rejects RF mismatch", !state.matches(xtal));

  auto ch20 = rtl8733b::Phy8733b::channel_plan(
      SelectedChannel{6, 0, CHANNEL_WIDTH_20});
  auto ch40lo = rtl8733b::Phy8733b::channel_plan(
      SelectedChannel{36, 1, CHANNEL_WIDTH_40});
  auto ch40hi = rtl8733b::Phy8733b::channel_plan(
      SelectedChannel{40, 2, CHANNEL_WIDTH_40});
  auto ch5 = rtl8733b::Phy8733b::channel_plan(
      SelectedChannel{6, 0, CHANNEL_WIDTH_5});
  auto ch10 = rtl8733b::Phy8733b::channel_plan(
      SelectedChannel{36, 0, CHANNEL_WIDTH_10});
  expect("2G 20 MHz plan", ch20 && ch20->center == 6 && ch20->is_2g);
  expect("5G 40 lower-primary plan",
         ch40lo && ch40lo->center == 38 && ch40lo->primary_index == 2);
  expect("5G 40 upper-primary plan",
         ch40hi && ch40hi->center == 38 && ch40hi->primary_index == 1);
  expect("20 MHz offset rejected",
         !rtl8733b::Phy8733b::channel_plan(
             SelectedChannel{6, 1, CHANNEL_WIDTH_20}));
  expect("illegal 40 MHz pair rejected",
         !rtl8733b::Phy8733b::channel_plan(
             SelectedChannel{40, 1, CHANNEL_WIDTH_40}));
  expect("5 MHz refused (continuous-carrier mode on this die)", !ch5);
  expect("10 MHz plan",
         ch10 && ch10->center == 36 && ch10->primary_index == 0 &&
             !ch10->is_2g);
  expect("narrowband offset rejected",
         !rtl8733b::Phy8733b::channel_plan(
             SelectedChannel{36, 1, CHANNEL_WIDTH_10}));

  rtl8733b::ChannelState8733b cs;
  cs.rf_a_18 = cs.rf_b_18 = (1u << 16) | (1u << 8) | (1u << 11) | 38;
  cs.rf_a_19 = cs.rf_b_19 = 0;
  cs.bb_9b0 = 0x2205;
  cs.data_sc = 2;
  cs.wmac_trxptcl = 1u << 7;
  cs.synth_ready = true;
  expect("40 MHz channel-state predicate", ch40lo && cs.matches(*ch40lo));
  cs.rf_b_18 ^= 1;
  expect("channel predicate rejects path divergence",
         ch40lo && !cs.matches(*ch40lo));

  cs.rf_a_18 = cs.rf_b_18 = (1u << 16) | (1u << 8) | (3u << 10) | 36;
  cs.rf_a_19 = cs.rf_b_19 = 0;
  cs.bb_9b0 = 2u << 6;
  cs.bb_9b4 = 2u << 8;
  cs.bb_9f0 = 0xb;
  cs.bb_81c = 0;
  cs.data_sc = 0;
  cs.wmac_trxptcl = 0;
  expect("10 MHz channel-state predicate", ch10 && cs.matches(*ch10));
  cs.bb_9f0 = 0xa;
  expect("10 MHz predicate rejects 5 MHz ADC clock",
         ch10 && !cs.matches(*ch10));

  rtl8733b::TxAgcState8733b txagc;
  txagc.cck_ref_a = txagc.cck_ref_b = rtl8733b::kSafeTxAgcIndex8733b;
  txagc.ofdm_ref_a = txagc.ofdm_ref_b = rtl8733b::kSafeTxAgcIndex8733b;
  expect("flat TXAGC readback predicate",
         txagc.matches_flat(rtl8733b::kSafeTxAgcIndex8733b));
  txagc.rate_diffs[3] = 1;
  expect("TXAGC predicate rejects non-flat rate diff",
         !txagc.matches_flat(rtl8733b::kSafeTxAgcIndex8733b));
  expect("TXAGC predicate rejects out-of-range reference",
         !txagc.matches_flat(0x80));

  rtl8733b::TssiPowerInfo8733b tssi_power;
  tssi_power.path_a_de[2] = 3;
  tssi_power.path_a_de[8] = 3;
  tssi_power.path_b_de[2] = 0;
  tssi_power.path_b_de[8] = 0;
  tssi_power.trim[0][0] = 2;
  tssi_power.trim[0][1] = 3;
  const auto tssi_ch6 = rtl8733b::Phy8733b::tssi_de_plan(tssi_power, 6);
  expect("channel-6 TSSI DE plan",
         tssi_ch6 && tssi_ch6->cck[0] == 5 && tssi_ch6->cck[1] == 3 &&
             tssi_ch6->ht40[0] == 5 && tssi_ch6->ht40[1] == 3 &&
             tssi_ch6->ofdm[0] == 3 && tssi_ch6->ofdm[1] == 1 &&
             tssi_ch6->ht20 == tssi_ch6->ofdm);
  /* The fast_retune DE rewrite triggers on plan inequality across a hop.
   * Pin the property it depends on: the 2.4 GHz buckets are ~3 channels
   * wide, so plans differ across a bucket boundary and match within one. */
  const auto tssi_ch7 = rtl8733b::Phy8733b::tssi_de_plan(tssi_power, 7);
  const auto tssi_ch11 = rtl8733b::Phy8733b::tssi_de_plan(tssi_power, 11);
  expect("TSSI DE plan matches within a 2.4 GHz bucket",
         tssi_ch7 && *tssi_ch6 == *tssi_ch7);
  expect("TSSI DE plan differs across a 2.4 GHz bucket boundary",
         tssi_ch11 && !(*tssi_ch6 == *tssi_ch11));
  tssi_power.path_a_de[24] = -127;
  tssi_power.trim[7][0] = -8;
  const auto tssi_ch177 = rtl8733b::Phy8733b::tssi_de_plan(tssi_power, 177);
  expect("TSSI DE arithmetic saturates",
         tssi_ch177 && tssi_ch177->ht40[0] == -128 &&
             tssi_ch177->ofdm[0] == -128);
  expect("invalid TSSI channel rejected",
         !rtl8733b::Phy8733b::tssi_de_plan(tssi_power, 0) &&
             !rtl8733b::Phy8733b::tssi_de_plan(tssi_power, 15));
  rtl8733b::TssiDeState8733b tssi_state;
  tssi_state.de = *tssi_ch6;
  tssi_state.path_a_lanes = 0x03050503;
  tssi_state.path_b_lanes = 0x03030103;
  expect("disabled TSSI DE readback predicate",
         tssi_state.matches_disabled(*tssi_ch6));
  tssi_state.enabled = true;
  expect("TSSI DE predicate rejects enabled engine",
         !tssi_state.matches_disabled(*tssi_ch6));

  rtl8733b::TssiBbState8733b tssi_bb_state;
  static_cast<rtl8733b::TssiBbPlan8733b &>(tssi_bb_state) = *tssi_bb_2g;
  expect("disabled TSSI digital readback predicate",
         tssi_bb_state.matches_disabled(*tssi_bb_2g));
  tssi_bb_state.reg_4328 ^= 1;
  expect("TSSI digital predicate rejects control mismatch",
         !tssi_bb_state.matches_disabled(*tssi_bb_2g));

  rtl8733b::TssiThermalState8733b tssi_thermal_state;
  tssi_thermal_state.control = tssi_thermal_ofdm.control();
  expect("disabled TSSI thermal readback predicate",
         tssi_thermal_state.matches_disabled(tssi_thermal_ofdm));
  tssi_thermal_state.enabled = true;
  expect("TSSI thermal predicate rejects enabled engine",
         !tssi_thermal_state.matches_disabled(tssi_thermal_ofdm));

  rtl8733b::TssiAnalogState8733b tssi_analog;
  tssi_analog.bb_1860 = 0xb0040ff8;
  tssi_analog.bb_1c38 = 0xffb5005e;
  tssi_analog.bb_1d40 = 0;
  tssi_analog.bb_1e1c = 0xc400b000;
  tssi_analog.bb_1ca4 = 0xe0000000;
  tssi_analog.rf_55 = {0x80, 0x80};
  tssi_analog.rf_7f = {0x100, 0x100};
  expect("disabled TSSI analog setup predicate", tssi_analog.matches_setup());
  tssi_analog.rf_7f = {0, 0};
  expect("TSSI RF command bits may self-clear", tssi_analog.matches_setup());
  tssi_analog.rf_7f = {0x100, 0x100};
  const auto tssi_analog_snapshot = tssi_analog;
  expect("TSSI analog rollback predicate",
         tssi_analog.matches_snapshot(tssi_analog_snapshot));
  tssi_analog.rf_55[1] = 0;
  expect("TSSI analog predicate rejects missing RF setup",
         !tssi_analog.matches_setup());
  expect("TSSI analog rollback rejects RF mismatch",
         !tssi_analog.matches_snapshot(tssi_analog_snapshot));

  return failures == 0 ? 0 : 1;
}
