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

  /* TX/RX DMA + priority-queue page allocation (RQPN) + LLT auto-init. On 8822B
   * this MUST run BEFORE firmware download: the DLFW rsvd-page bulk-OUT needs the
   * download queue to have TX-FIFO pages allocated, else the chip NAKs at 2048
   * bytes. (8822C's power-on RQPN defaults happen to suffice; 8822B's do not.)
   * Page numbers are identical to 8822C (TX_FIFO 262144, HQ/NQ/LQ 64/64/64). */
  bool init_trx_cfg();

  /* Reserved-page boundary computed by init_trx_cfg (feeds HalmacJaguar2Fw's
   * rsvd-page restore). Valid after init_trx_cfg(). */
  uint16_t rsvd_boundary() const { return _rsvd_boundary; }

private:
  bool priority_queue_cfg();
  void init_h2c();

  RtlUsbAdapter _device;
  Logger_t _logger;
  uint16_t _rsvd_boundary = 0;
};

} /* namespace jaguar2 */

#endif /* HALMAC_8822B_MACINIT_H */
