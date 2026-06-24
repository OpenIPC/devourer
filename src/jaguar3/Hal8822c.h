#ifndef HAL_8822C_H
#define HAL_8822C_H

#include "logger.h"
#include "RtlUsbAdapter.h"
#include "SelectedChannel.h"
#include "Halmac8822cFw.h"
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

  /* Read REG_SYS_CFG1 and decode chip cut / vendor / RF type (port of
   * rtl8822c_ops.c read_chip_version). Populates _phy_ctx.cut_version used by
   * the BB/AGC/RF table walker. Reads a register, so it runs during M2 bring-up;
   * exposed for the eventual chip-id-based family detection. */
  void read_chip_version();

  /* Decoded chip identity (valid after read_chip_version). */
  struct ChipVersion {
    uint8_t cut = 0;       /* 0=A,1=B,... (BIT_GET_CHIP_VER) */
    uint8_t vendor = 0;    /* 0=TSMC,1=SMIC,2=UMC */
    uint8_t rf_2t2r = 0;   /* 1 = 2T2R, 0 = 1T1R */
    bool test_chip = false;
  };
  ChipVersion chip_version() const { return _ver; }

private:
  void power_on();            /* TODO(M2): hand-rolled Jaguar3 PWR_SEQ */
  void init_queue_and_pages(); /* TODO(M2): RQPN / page boundary / auto-LLT */
  void apply_bb_rf_agc_tables(); /* TODO(M4): PhyTableLoader over 8822c tables */

  RtlUsbAdapter _device;
  Logger_t _logger;
  Halmac8822cFw _fw;

  /* phydm table-selection context. TODO(M2): populate cut_version / rfe_type
   * from the Jaguar3 chip-version + EFUSE read. Defaults are placeholders that
   * let apply_bb_rf_agc_tables() compile and be unit-exercised; they MUST be set
   * from hardware before the tables are trusted (M4). */
  jaguar3::Jaguar3PhyContext _phy_ctx{/*cut_version=*/0, /*rfe_type=*/0};
  ChipVersion _ver{};
};

} /* namespace jaguar3 */

#endif /* HAL_8822C_H */
