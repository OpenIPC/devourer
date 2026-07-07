#ifndef JAGUAR2_CALIBRATION_H
#define JAGUAR2_CALIBRATION_H

#include <cstdint>
#include <memory>

#include "ChipVariant.h"
#include "RtlAdapter.h"
#include "logger.h"

namespace jaguar2 {

/* Per-chip halrf IQ calibration. The bring-up flow in HalJaguar2/RtlJaguar2Device
 * is shared, but the SW IQK algorithm differs between the chips (rtl8822b's
 * 0x1b00 nctl per-path LOK/TXK/RXK vs rtl8821c's halrf_iqk_8821c path, 1T1R with
 * a 2.4G-BTG path-B). Each chip supplies its own implementation; the calibration
 * is constructed per-variant after read_chip_version (cut / rf-type known).
 *
 * Kept minimal: LC calibration (do_lck), RFE/BF init and coex antenna control
 * remain register-level methods on HalJaguar2 (branched on ChipVariant there),
 * because in Jaguar-2 they are already HAL methods rather than halrf routines. */
class Jaguar2Calibration {
public:
  virtual ~Jaguar2Calibration() = default;

  /* phy_iq_calibrate entry (SW path). band2g: true = 2.4 GHz. Runs the full
   * per-path LOK/TXK/RXK, backing up and restoring MAC/BB/RF around it. */
  virtual void iqk_trigger(bool band2g) = 0;
};

/* Factory: returns the calibration impl for the given chip. cut / is_2t2r come
 * from HalJaguar2::chip_version(), so this is called during bring-up (after
 * read_chip_version), not at device construction. */
std::unique_ptr<Jaguar2Calibration>
make_jaguar2_calibration(ChipVariant variant, RtlAdapter device,
                         Logger_t logger, uint8_t cut, bool is_2t2r);

} /* namespace jaguar2 */

#endif /* JAGUAR2_CALIBRATION_H */
