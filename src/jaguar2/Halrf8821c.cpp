#include "Halrf8821c.h"

#include <memory>
#include <utility>

namespace jaguar2 {

Halrf8821c::Halrf8821c(RtlUsbAdapter device, Logger_t logger, uint8_t cut,
                       bool is_2t2r)
    : _device{std::move(device)}, _logger{std::move(logger)}, _cut{cut},
      _2t2r{is_2t2r} {}

void Halrf8821c::iqk_trigger(bool band2g) {
  /* M1 stub — real halrf_iqk_8821c port lands in M6. Bring-up proceeds without
   * IQK (RX works pre-calibration); this only logs so a premature reliance is
   * visible. */
  (void)band2g;
  _logger->info("Jaguar2/8821C: IQK not yet ported (M6) — skipping");
}

/* Calibration-factory hook, called by make_jaguar2_calibration() in
 * Halrf8822b.cpp when the ChipVariant is C8821C. */
std::unique_ptr<Jaguar2Calibration>
make_calibration_8821c(RtlUsbAdapter device, Logger_t logger, uint8_t cut,
                       bool is_2t2r) {
  return std::make_unique<Halrf8821c>(std::move(device), std::move(logger), cut,
                                      is_2t2r);
}

} /* namespace jaguar2 */
