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

private:
  RtlUsbAdapter _device;
  Logger_t _logger;
  ChipVersion _ver{};
};

} /* namespace jaguar2 */

#endif /* HAL_JAGUAR2_H */
