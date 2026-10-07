#pragma once

/* IPcieDmaPlane — the frame plane of the PCIe transport, one implementation
 * per DMA engine generation. The vfio/BAR/config-space/DMA-slab/MSI plumbing
 * in PcieTransport is chip-agnostic; what differs between a HalMAC 11ac part
 * (RTL8821CE: 88xx buffer-descriptor rings, rtw88) and an AX part (RTL8852CE:
 * HAXI TXBD/WD pages + RXQ/RPQ rings, rtw89) is everything below this
 * interface. PcieTransport::Open picks the plane from the PCI device id and
 * forwards tx_sync / rx_loop / hci_setup to it.
 *
 * A plane carves its rings and buffers out of the one IOMMU-mapped slab the
 * transport allocates (slab_bytes() tells the transport how much), and reaches
 * registers through PcieTransport::mmio_read/mmio_write. */

#include <cstddef>
#include <cstdint>
#include <functional>

namespace devourer {

class IPcieDmaPlane {
public:
  virtual ~IPcieDmaPlane() = default;

  virtual const char *name() const = 0;

  /* DMA slab bytes this plane needs (rings + buffers). Called before attach. */
  virtual size_t slab_bytes() const = 0;
  /* Carve the slab (VA + its IOVA, both page-aligned) into rings/buffers and
   * arm the RX descriptors. Register programming is NOT done here — that is
   * setup_rings(), which runs at the generation's intf_pre_init slot. */
  virtual bool attach(uint8_t *slab_va, uint64_t slab_iova) = 0;

  /* Program the ring base/num/index registers and reset the host indices
   * (rtw88 rtw_pci_reset_buf_desc / rtw89 rtw89_pci_reset_trx_rings). Where
   * this sits in the bring-up is per generation: before power-on on the 88xx
   * plane, after dmac_func_pre_en and before FWDL on the AX plane. */
  virtual void setup_rings() = 0;

  /* Synchronous TX. `hint` is the HAL's queue handle: ignored by the 88xx
   * plane (it routes on the descriptor QSEL byte) and the AX DMA channel on
   * the AX plane. Returns bytes submitted or a negative error. */
  virtual int tx_submit(uint8_t hint, uint8_t *buf, size_t len,
                        int timeout_ms) = 0;

  /* Blocking RX reap loop until should_stop(); each delivery is one chip
   * packet (descriptor-headed), never a USB-style aggregate. */
  virtual void rx_loop(const std::function<void(const uint8_t *, int)> &on_data,
                       const std::function<bool()> &should_stop) = 0;

  /* Whether this plane consumes the vfio MSI eventfd in its RX loop. The
   * transport registers the vector only for a plane that says so, so a
   * polling plane neither arms an unused IRQ nor reports "MSI" as its RX
   * mechanism. */
  virtual bool uses_msi() const { return false; }
  /* Mask the plane's RX interrupt sources (teardown, before the MSI vector is
   * dropped). Default: nothing was enabled. */
  virtual void irq_mask() {}

  /* Register-plane filter for 16-bit-addressed accesses: false refuses the
   * access (the 88xx plane rejects the USB-only 0xFE00 page). */
  virtual bool reg_allowed(uint32_t reg) const {
    (void)reg;
    return true;
  }
};

} /* namespace devourer */
