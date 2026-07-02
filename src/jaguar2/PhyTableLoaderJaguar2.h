#ifndef PHY_TABLE_LOADER_JAGUAR2_H
#define PHY_TABLE_LOADER_JAGUAR2_H

#include <cstddef>
#include <cstdint>

#include "PhyTableLoader.h"

/* 8822B (Jaguar2) phydm table glue. The BB/AGC/RF/MAC register tables ship in
 * the OLD phydm `check_positive` format — flat (addr,value) pairs with
 * conditional blocks gated on cut/pkg/interface/rfe — identical to Jaguar1's
 * 8814a tables, so they are walked by the shared src/PhyTableLoader (not the
 * halbb walker src/jaguar3/PhyTableLoaderJaguar3 uses).
 *
 * This is the thin 8822B-specific adapter: it names the generated arrays
 * (hal/phydm/rtl8822b/Hal8822b_PhyTables) and drives PhyTableLoader::Load for
 * each. HalJaguar2 supplies the JaguarPhyContext (cut/rfe/...) and the register
 * writer (BB/RF/MAC) in M4. */

namespace jaguar2 {

struct PhyTable {
  const uint32_t *data;
  std::size_t len;
};

class PhyTableLoaderJaguar2 {
public:
  /* The four apply-order tables plus the MAC table. Definitions in the .cpp
   * bind to the generated `array_mp_8822b_*` externs so the generated header is
   * pulled in exactly once. */
  static PhyTable mac_reg();
  static PhyTable phy_reg();
  static PhyTable agc_tab();
  static PhyTable radioa();
  static PhyTable radiob();

  /* Walk `tbl` through the shared check_positive state machine, invoking
   * `write(addr, value)` for each entry that passes `ctx`'s conditional
   * filter. Thin wrapper over PhyTableLoader::Load for call-site clarity. */
  static void Apply(const PhyTable &tbl, const JaguarPhyContext &ctx,
                    const PhyTableLoader::Writer &write);
};

} /* namespace jaguar2 */

#endif /* PHY_TABLE_LOADER_JAGUAR2_H */
