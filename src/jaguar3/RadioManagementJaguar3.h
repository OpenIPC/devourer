#ifndef RADIO_MANAGEMENT_8822C_H
#define RADIO_MANAGEMENT_8822C_H

#include <cstdint>

#include "logger.h"
#include "RtlUsbAdapter.h"
#include "SelectedChannel.h"
#include "ChipVariant.h"

namespace jaguar3 {

/* RadioManagementJaguar3 — channel / bandwidth / TX-power for the Jaguar3 family.
 *
 * The Jaguar3 channel+bandwidth path is procedural — a port of the vendor's
 * config_phydm_switch_channel_8822c / config_phydm_switch_bandwidth_8822c
 * (hal/phydm/rtl8822c/phydm_hal_api8822c.c) — rather than the Jaguar1
 * phy_PostSetBwMode8812 register writes.
 *
 * Jaguar3 — unlike Jaguar1 — exposes a true baseband clock divider, so the
 * 5/10 MHz narrowband re-clock (the whole reason for the port) is reachable:
 * set_bandwidth_dividers applies it on top of an already-tuned channel via the
 * NB_* registers below (SDR-validated). */
class RadioManagementJaguar3 {
public:
  RadioManagementJaguar3(RtlUsbAdapter device, Logger_t logger,
                         ChipVariant variant = ChipVariant::C8822C);

  void set_channel_bwmode(uint8_t channel, uint8_t channel_offset,
                          ChannelWidth_t bwmode);

  /* Narrowband re-clock applied on top of an already-tuned channel: switches
   * the baseband DAC/ADC clock + small-BW field for 5/10/20 MHz. */
  void set_bandwidth_dividers(ChannelWidth_t bwmode);

  /* Set the TX-power reference (7-bit index, both paths). With zero_diffs=true
   * (the DEVOURER_TX_PWR debug knob) the per-rate diff table is flattened so
   * every rate emits at `idx`. With zero_diffs=false (the default bring-up path)
   * the diff table applied by the BB tables is preserved, so per-rate spread is
   * kept and only the reference base is programmed. */
  void set_tx_power_ref(uint8_t idx, bool zero_diffs = true);

  /* Apply the 8822e phy_reg_pg power-by-rate table for `channel`'s band: sets
   * the OFDM/CCK reference PER PATH (ref_a -> 0x18e8/0x18a0, ref_b -> 0x41e8/
   * 0x41a0) and writes the per-rate diff table (0x3a00) so robust low rates get
   * the kernel's by-rate boost instead of a flat reference. The per-path refs
   * come from the efuse per-channel base (the kernel programs a distinct path-A/
   * path-B base, e.g. 0x4b/0x54 at ch36).
   *
   * skip_path_b_ofdm_ref: leave 0x41e8 (path-B OFDM ref) at its table default.
   * Hardware-bisected: ANY nonzero value in that one field desenses the EU's RX
   * to near-deaf (value-independent; path-A ref, CCK refs, the diff table and
   * the DPK bypass are all RX-safe) — root cause open, suspected path-B
   * TSSI/gain-stage asymmetry in devourer's bring-up vs the kernel's. TX+RX
   * callers set this so RX works at the cost of path-B OFDM TX running at the
   * table-default reference. */
  void apply_power_by_rate_8822e(uint8_t channel, uint8_t ref_a, uint8_t ref_b,
                                 bool skip_path_b_ofdm_ref = false);

private:
  /* Jaguar3 baseband bandwidth/clock registers (from
   * config_phydm_switch_bandwidth_8822c). These do NOT exist on Jaguar1. */
  static constexpr uint16_t R_RX_DFIR_8822C   = 0x810; /* [13:4] RX DFIR coeff */
  static constexpr uint16_t R_SMALL_BW_8822C  = 0x9b0; /* [7:6] small-BW field */
  static constexpr uint16_t R_CLK_DIV_8822C   = 0x9b4; /* [10:8] DAC, [22:20] ADC clk */

  /* Narrowband recipe (small-BW field at 0x9b0[7:6]):
   *   5 MHz  -> 0x1 ;  10 MHz -> 0x2 ;  20 MHz -> 0x0
   * DAC clock 0x9b4[10:8]:  5M->0x4(120M)  10M->0x6(240M)  20M->0x7(480M)
   * ADC clock 0x9b4[22:20]: 5M->0x4(40M)   10M->0x5(80M)   20M->0x6(160M)
   * RX DFIR  0x810[13:4]:   5/10M->0x2ab   20M->0x19b
   * 5 MHz has known DAC mirror/leakage; 10 MHz is the reliable target. */
  static constexpr uint8_t NB_SMALLBW_5M  = 0x1;
  static constexpr uint8_t NB_SMALLBW_10M = 0x2;

  RtlUsbAdapter _device;
  Logger_t _logger;
  ChipVariant _variant;
};

} /* namespace jaguar3 */

#endif /* RADIO_MANAGEMENT_8822C_H */
