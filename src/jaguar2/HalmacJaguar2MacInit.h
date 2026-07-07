#ifndef HALMAC_8822B_MACINIT_H
#define HALMAC_8822B_MACINIT_H

#include <cstdint>

#include "logger.h"
#include "RtlUsbAdapter.h"
#include "SelectedChannel.h"
#include "ChipVariant.h"

namespace jaguar2 {

/* HalmacJaguar2MacInit — RTL8822B / RTL8821C MAC configuration, ported from the
 * HalMAC 88xx init flow (halmac_init_88xx.c) mirroring
 * src/jaguar3/HalmacJaguar3MacInit. Covers the pre-DLFW system config
 * (pre_init_system_cfg / init_system_cfg / enable_bb_rf) and the post-DLFW MAC
 * config (trx/queue/protocol/edca/wmac + USB cfg), per-variant where the two
 * chips diverge. */
class HalmacJaguar2MacInit {
public:
  HalmacJaguar2MacInit(RtlUsbAdapter device, Logger_t logger,
                       ChipVariant variant = ChipVariant::C8822B);

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
  /* set_bcn_boundary=false skips the beacon-queue boundary writes (BCNQ_BDNY /
   * FIFOPAGE_CTRL_2=boundary). Used for the pre-DLFW call: 8822B needs the page
   * allocation + LLT for the download-queue FIFO to drain, but setting the
   * beacon boundary to rsvd_boundary(1938) makes the page-0 rsvd-page beacon
   * download fail its bcn-valid latch. The boundary is set later, post-DLFW. */
  bool init_trx_cfg(bool set_bcn_boundary = true);

  /* Reserved-page boundary computed by init_trx_cfg (feeds HalmacJaguar2Fw's
   * rsvd-page restore). Valid after init_trx_cfg(). */
  uint16_t rsvd_boundary() const { return _rsvd_boundary; }

  /* Post-DLFW MAC configuration (halmac init_mac_cfg_88xx flow):
   * init_trx_cfg -> init_protocol_cfg -> init_edca_cfg -> init_wmac_cfg, using
   * the 8822B-specific bodies (halmac_init_8822b.c). Runs after the firmware
   * boots. */
  bool init_mac_cfg(ChannelWidth_t bw);

  /* USB RX-DMA mode + RX aggregation (init_usb_cfg_88xx) so received frames are
   * delivered to the bulk-IN endpoint. USB backend only — the PCIe RX path is
   * the buffer-descriptor ring the transport programs pre-power-on. */
  void init_usb_cfg();

  /* PCIe interface-PHY config (rtw_pci_phy_cfg's 8821C gen1 MDIO write).
   * PCIe backend only; run once before the first power-on. */
  void pcie_phy_cfg();

private:
  bool priority_queue_cfg();
  void init_h2c();
  void init_protocol_cfg();
  void init_edca_cfg();
  void init_wmac_cfg();

  RtlUsbAdapter _device;
  Logger_t _logger;
  ChipVariant _variant;
  uint16_t _rsvd_boundary = 0;
  bool _set_bcn_boundary = true;
};

} /* namespace jaguar2 */

#endif /* HALMAC_8822B_MACINIT_H */
