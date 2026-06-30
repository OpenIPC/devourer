#ifndef RADIO_MANAGEMENT_8822C_H
#define RADIO_MANAGEMENT_8822C_H

#include <cstdint>

#include "logger.h"
#include "RtlUsbAdapter.h"
#include "SelectedChannel.h"

namespace jaguar3 {

/* RadioManagement8822c — channel / bandwidth / TX-power for the Jaguar3 family.
 *
 * The Jaguar3 channel+bandwidth path is procedural (the vendor implements it in
 * config_phydm_switch_channel_8822c / config_phydm_switch_bandwidth_8822c in
 * hal/phydm/rtl8822c/phydm_hal_api8822c.c) rather than via the Jaguar1
 * phy_PostSetBwMode8812 register writes. This class will port that procedure.
 *
 * The narrowband payoff (the whole reason for the Jaguar3 port) is a drop-in
 * extension of switch_bandwidth: Jaguar3 — unlike Jaguar1 — exposes a true
 * baseband clock divider, so 5/10 MHz is achievable. The exact register recipe
 * is already known (see the NB_* constants below and switch_bandwidth's TODO). */
class RadioManagement8822c {
public:
  RadioManagement8822c(RtlUsbAdapter device, Logger_t logger);

  void set_channel_bwmode(uint8_t channel, uint8_t channel_offset,
                          ChannelWidth_t bwmode);

  /* Narrowband re-clock applied on top of an already-tuned channel: switches
   * the baseband DAC/ADC clock + small-BW field for 5/10/20 MHz. */
  void set_bandwidth_dividers(ChannelWidth_t bwmode);

  /* Override the TX-power reference (7-bit index, both paths) and flatten the
   * per-rate diff table so every rate emits at that level. Wired to
   * DEVOURER_TX_PWR for stronger TX than the bring-up default. */
  void set_tx_power_ref(uint8_t idx);

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
};

} /* namespace jaguar3 */

#endif /* RADIO_MANAGEMENT_8822C_H */
