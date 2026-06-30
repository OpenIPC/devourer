#ifndef HAL_8822C_H
#define HAL_8822C_H

#include "logger.h"
#include "RtlUsbAdapter.h"
#include "SelectedChannel.h"
#include "Halmac8822cFw.h"
#include "Halmac8822cMacInit.h"
#include "Halrf8822cIqk.h"
#include "PhyTableLoader8822c.h"

namespace jaguar3 {

/* Hal8822c — Jaguar3 chip bring-up: power sequencing, queue/page/LLT init, BB /
 * AGC / RF table application, and band/channel setup. This is the Jaguar3
 * analogue of src/HalModule (which is Jaguar1-only).
 *
 * Per the locked HalMAC decision, this class hand-rolls power-on / queue / EFUSE
 * in devourer's style and delegates only the firmware download to the verbatim
 * HalMAC port in Halmac8822cFw. */
class Hal8822c {
public:
  Hal8822c(RtlUsbAdapter device, Logger_t logger);

  /* Full RX-capable bring-up: power-on -> FW download -> MAC/BB/RF config ->
   * band+channel set. Mirrors HalModule::rtw_hal_init() for Jaguar1. */
  void rtw_hal_init(SelectedChannel channel);

  /* Clean de-init: stop TRX DMA, then run the card-disable PWR_SEQ so the chip
   * parks in a low-power, re-enumerable state. Mirrors the kernel rtw88 driver's
   * unbind path. Must run before the USB interface is released. Best-effort:
   * tolerates a chip that has already dropped off the bus. */
  void rtw_hal_deinit();

  /* Read REG_SYS_CFG1 and decode chip cut / vendor / RF type (port of
   * rtl8822c_ops.c read_chip_version). Populates _phy_ctx.cut_version used by
   * the BB/AGC/RF table walker. Reads a register, so it runs during bring-up;
   * exposed for the eventual chip-id-based family detection. */
  void read_chip_version();

  /* Decode the logical EFUSE map far enough to return EEPROM_RFE_OPTION_8822C
   * (logical offset 0xCA) — the RFE front-end type that gates BB/RFE config.
   * Returns 0 if the byte is unprogrammed (0xFF) or the read fails. */
  uint8_t read_efuse_rfe_type();

  /* Decoded chip identity (valid after read_chip_version). */
  struct ChipVersion {
    uint8_t cut = 0;       /* 0=A,1=B,... (BIT_GET_CHIP_VER) */
    uint8_t vendor = 0;    /* 0=TSMC,1=SMIC,2=UMC */
    uint8_t rf_2t2r = 0;   /* 1 = 2T2R, 0 = 1T1R */
    bool test_chip = false;
  };
  ChipVersion chip_version() const { return _ver; }

private:
  void power_off();           /* card-disable PWR_SEQ — reset from active state */
  void power_on();            /* card-enable PWR_SEQ */
  void init_rfk();            /* RF-calibration init (0x1B00 cal_init block) */
  void apply_bb_rf_agc_tables(); /* phydm BB/AGC/RF tables via PhyTableLoader */
  void bf_init();                /* rtl8822c_phy_bf_init: BF/MU + NDPA sounding */
  void config_phydm_parameter_init(); /* POST_SETTING: 3-wire + OFDM/CCK block */
  void enable_tx_path();         /* OFDM/CCK TX block + AGC/path enable (on-air TX) */
  /* monitor RX config is devourer-specific (the vendor driver has no pure
   * monitor mode). */
  void monitor_rx_cfg();

public:
  /* Run the ported IQK calibration. Must be called AFTER the channel is tuned
   * (IQK reads RF18 for band/ch). Replaces init_rfk's hardcoded gain regs. */
  void run_iqk(SelectedChannel channel);

  /* Persistently grant the shared antenna to WLAN (see
   * Halrf8822cIqk::force_wl_antenna). Call after bring-up and periodically so
   * the coex firmware can't switch the antenna away mid-TX. */
  void force_wl_antenna() { _iqk.force_wl_antenna(); }

  /* One-time WiFi-only coex bring-up (disables BT/LTE coex arbitration + locks
   * the antenna to WL). See Halrf8822cIqk::coex_wlan_only_init. */
  void coex_wlan_only_init() { _iqk.coex_wlan_only_init(); }
  void coex_keepalive() { _iqk.coex_keepalive(); }
  void coex_run_5g() { _iqk.coex_run_5g(); }
  void pwr_track() { _iqk.pwr_track(); }

  /* Runtime H2C (host-to-firmware) over the HMEBOX mailboxes (port of
   * rtw_fw_send_h2c_command): 8-byte command, round-robin across 4 boxes. */
  void send_h2c_raw(uint32_t msg, uint32_t msg_ext);

  /* WL_PHY_INFO heartbeat (H2C 0x58, port of rtw_fw_update_wl_phy_info). The
   * firmware's idle detection relies on the host reporting WL PHY activity every
   * ~2 s; without it the FW powers the RF down after ~30 s. Call periodically
   * during RX and TX. */
  void fw_update_wl_phy_info();

  /* SET_PWR_MODE (H2C 0x20) = active / all-on (port of rtw_leave_lps_core:
   * mode 0, rlbm 0, smart_ps 0, awake_interval 1, pwr_state RTW_ALL_ON). Tells
   * the firmware to keep every power domain on instead of auto-entering power
   * save. Call at bring-up and periodically. */
  void fw_set_pwr_mode_active();

  /* Coex protocol H2C. query_bt_info (0x61) asks the FW to report BT state — on a
   * WiFi-only chip the FW then settles into "BT absent". tdma_off (0x60, all
   * params 0) disables coex time-division so the FW stops time-slicing the
   * antenna away from WLAN. */
  void fw_coex_query_bt_info() { send_h2c_raw(0x61u | (1u << 8), 0); }
  void fw_coex_tdma_off() { send_h2c_raw(0x60u, 0); }

private:

  RtlUsbAdapter _device;
  Logger_t _logger;
  Halmac8822cFw _fw;
  Halmac8822cMacInit _macinit;
  Halrf8822cIqk _iqk;

  /* phydm table-selection context. cut_version is set from read_chip_version()
   * and rfe_type from read_efuse_rfe_type(), both during rtw_hal_init() before
   * apply_bb_rf_agc_tables() walks the tables. The initialiser values are just
   * the pre-bring-up defaults. */
  jaguar3::Jaguar3PhyContext _phy_ctx{/*cut_version=*/0, /*rfe_type=*/0};
  ChipVersion _ver{};
  uint8_t _h2c_box = 0; /* round-robin HMEBOX index for send_h2c_raw */
};

} /* namespace jaguar3 */

#endif /* HAL_8822C_H */
