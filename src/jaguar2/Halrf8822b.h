#ifndef HALRF_8822B_H
#define HALRF_8822B_H

#include <cstdint>

#include "logger.h"
#include "RtlUsbAdapter.h"
#include "Jaguar2Calibration.h"

namespace jaguar2 {

/* Halrf8822b — RTL8822B halrf IQK (IQ calibration), ported verbatim from
 * reference/rtl88x2bu/hal/phydm/halrf/rtl8822b/halrf_iqk_8822b.c.
 *
 * The 8822B RX front-end decodes real frames only marginally without IQK (the
 * BB shows CRC-OK but the false-alarm rate is high and few/no MPDUs reach the
 * MAC RX FIFO). This runs the software IQK — LOK/TXK/RXK per path via the
 * 0x1b00 nctl IQK-engine bank — after the channel is set, so real frames
 * resolve.
 *
 * RF access matches phydm: RF read = direct BB shadow window (0x2800/0x2c00 +
 * addr<<2); RF write = 3-wire LSSI (0xC90/0xE90). BB access via phy_set_bb_reg
 * / rtw_read32. */
class Halrf8822b : public Jaguar2Calibration {
public:
  Halrf8822b(RtlUsbAdapter device, Logger_t logger, uint8_t cut, bool is_2t2r);

  /* phy_iq_calibrate_8822b entry (SW path). band2g: true=2.4G. Runs the full
   * per-path LOK/TXK/RXK, backing up and restoring MAC/BB/RF around it. */
  void iqk_trigger(bool band2g) override;

private:
  /* --- MASKDWORD/RF primitives (dm API shims) --- */
  uint32_t bb_get(uint16_t addr, uint32_t mask);
  void bb_set(uint16_t addr, uint32_t mask, uint32_t data);
  uint32_t r32(uint16_t addr) { return _device.rtw_read32(addr); }
  void w32(uint16_t addr, uint32_t v) { _device.rtw_write32(addr, v); }
  void w8(uint16_t addr, uint8_t v) { _device.rtw_write8(addr, v); }
  uint32_t rf_get(uint8_t path, uint32_t addr); /* direct-BB read, 20-bit */
  void rf_set(uint8_t path, uint32_t addr, uint32_t mask, uint32_t data);
  void rf_set_check(uint8_t path, uint16_t addr, uint32_t data);

  /* --- setup / backup / restore --- */
  void backup_mac_bb(uint32_t *mac_bk, uint32_t *bb_bk,
                     const uint32_t *mac_reg, const uint32_t *bb_reg);
  void backup_rf(uint32_t rf_bk[][2], const uint32_t *reg);
  void restore_mac_bb(const uint32_t *mac_bk, const uint32_t *bb_bk,
                      const uint32_t *mac_reg, const uint32_t *bb_reg);
  void restore_rf(const uint32_t *reg, uint32_t rf_bk[][2]);
  void agc_bnd_int();
  void bb_reset();
  void afe_setting(bool do_iqk);
  void rfe_setting(bool ext_pa_on);
  void rf_setting();
  void configure_macbb();

  /* --- per-K settings --- */
  void lok_setting(uint8_t path);
  void txk_setting(uint8_t path);
  void rxk1_setting(uint8_t path);
  void rxk2_setting(uint8_t path, bool is_gs);
  void set_rf0x8(uint8_t path);

  /* --- nctl (0x1b00 bank) LTE-coex gnt + measurement one-shots --- */
  uint32_t ltec_read(uint16_t reg);
  void ltec_write(uint16_t reg, uint32_t mask, uint32_t val);
  bool check_cal(uint8_t path, uint8_t cmd); /* returns fail */
  bool lok_one_shot(uint8_t path);
  bool rxk_gsearch(uint8_t path, uint8_t step);
  bool one_shot(uint8_t path, uint8_t idx);
  bool rx_iqk_by_path(uint8_t path);
  void iqk_by_path_subfunction(uint8_t path);
  void iqk_by_path();
  void start_iqk();
  void iqk_init();

  RtlUsbAdapter _device;
  Logger_t _logger;
  uint8_t _cut;
  bool _2t2r;
  bool _band2g = true; /* set per iqk_trigger */
  uint8_t _bw = 0;     /* 0=20MHz (2.4G monitor) */
  uint8_t _tmp1bcc = 0x12;

  /* IQK run state (dm_iqk_info subset; fresh-IQK path, no reload/CFIR cache) */
  int _iqk_step = 1;
  int _rxiqk_step = 1;
  int _kcount = 0;
  uint8_t _retry[2][3] = {};    /* [path][TXIQK|RXIQK1|RXIQK2] */
  uint8_t _gs_retry[2][2] = {}; /* [path][gs1|gs2] */
  bool _lok_fail[2] = {true, true};
  uint8_t _rxiqk_fail_code[2] = {};
  uint8_t _lna_idx = 0;
  bool _isbnd = false;
  uint32_t _tmp_gntwl = 0;
};

} /* namespace jaguar2 */

#endif /* HALRF_8822B_H */
