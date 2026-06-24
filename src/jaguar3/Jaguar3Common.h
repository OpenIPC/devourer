#ifndef JAGUAR3_COMMON_H
#define JAGUAR3_COMMON_H

#include <stdexcept>
#include <string>

/* Shared scaffolding for the Realtek Jaguar3 (RTL8822CU / RTL8812EU / RTL8822EU)
 * port. The port lands in milestones (see docs/jaguar3-bringup.md); every
 * hardware-touching method below is currently an explicit stub that throws via
 * jaguar3_todo() so a Jaguar3 device fails loudly and points at the milestone
 * that will implement it, rather than silently doing nothing. */
namespace jaguar3 {

/* Milestones of the bring-up roadmap. Kept here so the "not yet implemented"
 * errors name a concrete next step. */
enum class Milestone {
  M1_VendorAndTables,
  M2_PowerOnEfuse,
  M3_FirmwareDownload,
  M4_RxFirst,
  M5_Tx20MHz,
  M6_Narrowband,
};

inline const char *milestone_name(Milestone m) {
  switch (m) {
  case Milestone::M1_VendorAndTables: return "M1 (vendor sources + PHY tables)";
  case Milestone::M2_PowerOnEfuse:    return "M2 (power-on + EFUSE)";
  case Milestone::M3_FirmwareDownload:return "M3 (HalMAC firmware download)";
  case Milestone::M4_RxFirst:         return "M4 (RX-first bring-up)";
  case Milestone::M5_Tx20MHz:         return "M5 (20 MHz TX)";
  case Milestone::M6_Narrowband:      return "M6 (5/10 MHz narrowband)";
  }
  return "(unknown milestone)";
}

[[noreturn]] inline void jaguar3_todo(const char *what, Milestone m) {
  throw std::runtime_error(std::string("Jaguar3: ") + what +
                           " — not yet implemented, scheduled for " +
                           milestone_name(m));
}

} /* namespace jaguar3 */

#endif /* JAGUAR3_COMMON_H */
