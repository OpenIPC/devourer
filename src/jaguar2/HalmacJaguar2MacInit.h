#ifndef HALMAC_8822B_MACINIT_H
#define HALMAC_8822B_MACINIT_H

#include <cstdint>

#include "logger.h"
#include "RtlUsbAdapter.h"
#include "SelectedChannel.h"

namespace jaguar2 {

/* HalmacJaguar2MacInit — RTL8822B MAC configuration, ported from the HalMAC 88xx
 * init flow (halmac_init_88xx.c) mirroring src/jaguar3/HalmacJaguar3MacInit.
 *
 * Milestone status: M4 (part 1) — the pre-DLFW system config
 * (pre_init_system_cfg / init_system_cfg / enable_bb_rf) needed to boot the
 * firmware. The full post-DLFW MAC config (trx/queue/protocol/edca/wmac + USB
 * cfg) lands next. */
class HalmacJaguar2MacInit {
public:
  HalmacJaguar2MacInit(RtlUsbAdapter device, Logger_t logger);

  /* Pinmux / LED / GPIO + BB-RF disabled. Runs BEFORE power_on. */
  void pre_init_system_cfg();

  /* DDMA enable, SYS_FUNC_EN, PHY_REQ_DELAY (bw), disable boot-from-flash.
   * Runs AFTER power_on + read_chip_version, BEFORE firmware download. */
  void init_system_cfg(ChannelWidth_t bw, uint8_t cut);

  /* Toggle SYS_FUNC_EN BB + RF_CTRL + WLRF1 RF-on bits. */
  void enable_bb_rf(bool enable);

private:
  RtlUsbAdapter _device;
  Logger_t _logger;
};

} /* namespace jaguar2 */

#endif /* HALMAC_8822B_MACINIT_H */
