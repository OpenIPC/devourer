/* SHIM entry for the vendored halrf-G6 8852C RF source. The vendored halrf
 * (hal/halrf/g6/vendor/) reaches this via its halrf_precomp.h's
 * `#include "../../hal_headers_le.h"`. halrf shares the entire PHL/BB type +
 * register-plane surface with halbb, so we just reuse the halbb shim (which
 * already provides the RF-register plane rtw_hal_*_rf_reg -> bridge). The
 * vendored halrf headers themselves supply struct rf_info. */
#ifndef _HAL_HEADERS_LE_H_
#include "../halbb/hal_headers_le.h"
#endif
