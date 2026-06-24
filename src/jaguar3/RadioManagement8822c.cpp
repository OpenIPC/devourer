#include "RadioManagement8822c.h"

#include <utility>

#include "Jaguar3Common.h"

namespace jaguar3 {

RadioManagement8822c::RadioManagement8822c(RtlUsbAdapter device, Logger_t logger)
    : _device{device}, _logger{std::move(logger)} {}

void RadioManagement8822c::set_channel_bwmode(uint8_t /*channel*/,
                                              uint8_t /*channel_offset*/,
                                              ChannelWidth_t bwmode) {
  /* TODO(M4): port config_phydm_switch_channel_8822c + switch_bandwidth_8822c
   * for 20/40/80. TODO(M6): add the CHANNEL_WIDTH_5 / CHANNEL_WIDTH_10 branch
   * using R_SMALL_BW_8822C / R_CLK_DIV_8822C / R_RX_DFIR_8822C (NB_* recipe in
   * the header). Unlike Jaguar1, the clock divider exists here, so narrowband
   * is reachable. */
  if (bwmode == CHANNEL_WIDTH_5 || bwmode == CHANNEL_WIDTH_10) {
    jaguar3_todo("narrowband channel/bandwidth set", Milestone::M6_Narrowband);
  }
  jaguar3_todo("channel/bandwidth set", Milestone::M4_RxFirst);
}

void RadioManagement8822c::set_tx_power_level(uint8_t /*channel*/) {
  /* TODO(M5): port the Jaguar3 per-rate TX-power path. Note the narrowband
   * gotcha to carry into M6: TX power can't be changed while in 5/10 MHz — set
   * it at 20 MHz, then switch bandwidth. */
  jaguar3_todo("TX power level", Milestone::M5_Tx20MHz);
}

} /* namespace jaguar3 */
