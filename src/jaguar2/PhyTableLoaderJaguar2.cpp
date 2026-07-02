#include "PhyTableLoaderJaguar2.h"

#include "Hal8822b_PhyTables.h" /* generated: array_mp_8822b_* externs */

namespace jaguar2 {

PhyTable PhyTableLoaderJaguar2::mac_reg() {
  return {array_mp_8822b_mac_reg, array_mp_8822b_mac_reg_len};
}
PhyTable PhyTableLoaderJaguar2::phy_reg() {
  return {array_mp_8822b_phy_reg, array_mp_8822b_phy_reg_len};
}
PhyTable PhyTableLoaderJaguar2::agc_tab() {
  return {array_mp_8822b_agc_tab, array_mp_8822b_agc_tab_len};
}
PhyTable PhyTableLoaderJaguar2::radioa() {
  return {array_mp_8822b_radioa, array_mp_8822b_radioa_len};
}
PhyTable PhyTableLoaderJaguar2::radiob() {
  return {array_mp_8822b_radiob, array_mp_8822b_radiob_len};
}

void PhyTableLoaderJaguar2::Apply(const PhyTable &tbl,
                                  const JaguarPhyContext &ctx,
                                  const PhyTableLoader::Writer &write) {
  PhyTableLoader::Load(tbl.data, tbl.len, ctx, write);
}

} /* namespace jaguar2 */
