/* Stubs for the deferred halrf TX-power-limit subsystem (halrf_pwr_table.c /
 * halrf_set_pwr_table_8852c.c are not vendored). Registering the 8852C RFK ops
 * (rf_set_ops_8852c via halrf_cmn_info_self_init) makes --gc-sections keep the
 * DPK/TSSI functions, which reference these power-limit lookups — even though
 * only DACK/RX-DCK/IQK are actually driven. Return a benign default so linking
 * succeeds; these paths are not executed. Compiled as C behind the shim. */
#include "halrf_precomp.h"

s8 halrf_get_power_limit(struct rf_info *rf, enum phl_phy_idx phy, u8 path,
                         u16 rate, u8 bw, u8 bf, u8 ph, u8 ch) {
  (void)rf; (void)phy; (void)path; (void)rate; (void)bw; (void)bf; (void)ph;
  (void)ch;
  return 40 * 4; /* ~40 dBm in the s(,2) unit — unused (DPK not run) */
}
