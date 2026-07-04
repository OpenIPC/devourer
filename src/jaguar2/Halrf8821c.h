#ifndef HALRF_8821C_H
#define HALRF_8821C_H

#include <cstdint>

#include "logger.h"
#include "RtlUsbAdapter.h"
#include "Jaguar2Calibration.h"

namespace jaguar2 {

/* Halrf8821c — RTL8821C (1T1R) halrf IQ calibration, to be ported from
 * reference/8821cu/hal/phydm/halrf/rtl8821c/halrf_iqk_8821c.c.
 *
 * Milestone status: M1 stub. iqk_trigger() is a no-op that logs; the real
 * per-path LOK/TXK/RXK port lands in M6. Bring-up runs without it (RX still
 * works pre-IQK, and DEVOURER_SKIP_IQK bypasses it entirely), so this keeps the
 * C8821C build/link intact while the earlier bring-up milestones proceed. */
class Halrf8821c : public Jaguar2Calibration {
public:
  Halrf8821c(RtlUsbAdapter device, Logger_t logger, uint8_t cut, bool is_2t2r);

  void iqk_trigger(bool band2g) override;

private:
  RtlUsbAdapter _device;
  Logger_t _logger;
  uint8_t _cut;
  bool _2t2r;
};

} /* namespace jaguar2 */

#endif /* HALRF_8821C_H */
