#ifndef HAL_JAGUAR2_H
#define HAL_JAGUAR2_H

#include <cstdint>

#include "logger.h"
#include "RtlUsbAdapter.h"

namespace jaguar2 {

/* HalJaguar2 — RTL8822B (Jaguar2) chip bring-up. Jaguar2 sibling of
 * src/jaguar3/HalJaguar3, but single-chip and using the HalMAC power sequence +
 * the shared check_positive PHY-table walker.
 *
 * Milestone status: M2 — power-on/off (halmac 8822b pwr_seq) + chip-version.
 * Firmware DLFW, MAC/BB/RF init and calibration are added in later milestones. */
class HalJaguar2 {
public:
  HalJaguar2(RtlUsbAdapter device, Logger_t logger);

  /* Card-enable power sequence (card-disable -> card-emulation -> active),
   * transcribed from halmac card_en_flow_8822b (USB/ALL entries). Runs
   * power_off() first so the MAC is reset from any prior (kernel-left) state. */
  void power_on();
  /* Card-disable power sequence (active -> card-emulation -> card-disable). */
  void power_off();

  /* Read REG_SYS_CFG1 (0x00F0) and decode cut / vendor / 2T2R. */
  void read_chip_version();

  struct ChipVersion {
    uint8_t cut = 0;     /* 0=A,1=B,2=C,... */
    uint8_t vendor = 0;  /* 0=TSMC,1=SMIC,2=UMC */
    uint8_t rf_2t2r = 0; /* 1 = 2T2R */
    bool test_chip = false;
  };
  ChipVersion chip_version() const { return _ver; }

  /* Read the EFUSE logical map (standard 88xx OneByteRead + 2-byte-header
   * decode) and return the RFE type at EEPROM_RFE_OPTION_8822B (0xCA). The
   * phydm BB/AGC/RF tables are gated on rfe_type; a wrong value leaves the RX
   * front-end unconfigured. */
  uint8_t read_efuse_rfe();

  /* Apply the 8822B BB (phy_reg), AGC (agc_tab) and RF (radioa/radiob) phydm
   * tables via the shared check_positive walker, bracketed by the OFDM/CCK
   * block disable/enable (config_phydm_parameter_init_8822b PRE/POST). Mirrors
   * rtl8822b_phy.c: PRE -> init_bb_reg -> init_rf_reg -> POST. rfe_type selects
   * the conditional table blocks. */
  void apply_bb_rf_agc_tables(uint8_t rfe_type);

  /* Set RF channel + bandwidth (config_phydm_switch_channel_8822b +
   * config_phydm_switch_bandwidth_8822b): RF18 tune, band AGC/fc/CCK-filter,
   * RFE antenna pins, RX-path + IGI toggle. bw: 0=20MHz (only 20 supported for
   * now). rfe_type selects the RFE-pin table. rf_2t2r drives path-B writes. */
  void set_channel_bw(uint8_t channel, uint8_t bw, uint8_t rfe_type);

  /* Enable the MAC RX engine (CR MACRXEN + promiscuous RCR for monitor). */
  void enable_rx();

private:
  /* config_phydm_parameter_init_8822b: OFDM/CCK block enable via 0x808[29:28]
   * (post=0x3) / disable (pre=0x0). */
  void phydm_pre_post_setting(bool post);
  /* BB/AGC table writer: phydm delay opcodes (0xfe..0xf9) else a full-dword BB
   * register write, +1us settle (odm_config_bb_phy_8812a pattern). */
  void bb_write(uint32_t addr, uint32_t value);
  /* RF table writer: Jaguar 3-wire LSSI write (rA/rB_LSSIWrite_Jaguar 0xC90 /
   * 0xE90) — data = (addr<<20)|(val&0xfffff), masked to 28 bits. path: 0=A,1=B. */
  void rf_write(uint8_t path, uint32_t addr, uint32_t value);
  /* RF 3-wire LSSI read (rHSSIRead_Jaguar 0x8B0 -> rA/rB SI/PI readback). */
  uint32_t rf_read(uint8_t path, uint32_t addr);
  /* Masked RF write: full-mask -> direct LSSI write; else read-modify-write. */
  void rf_set(uint8_t path, uint32_t addr, uint32_t mask, uint32_t value);
  /* phydm_rfe_ifem 2.4G/5G antenna-switch pins (rfe_type 0 path). */
  void rfe_ifem(uint8_t channel);
  /* phydm_igi_toggle_8822b: toggle 0xc50/0xe50 IGI to enter RX mode. */
  void igi_toggle();

  RtlUsbAdapter _device;
  Logger_t _logger;
  ChipVersion _ver{};
};

} /* namespace jaguar2 */

#endif /* HAL_JAGUAR2_H */
