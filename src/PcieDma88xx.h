#pragma once

/* PcieDma88xx — the HalMAC-generation (11ac, rtw88) PCIe DMA plane: 88xx
 * buffer-descriptor rings, ported from rtw88 pci.{c,h} (v6.12). 8-byte BD
 * entries, 16-byte TX BD slots (entry0 = 48-byte tx desc, entry1 = payload),
 * ring base/num/idx registers at 0x300..0x3B8 + the H2C ring at 0x1320. RX
 * completion is the hardware write index in RTK_PCI_RXBD_IDX_MPDUQ (0x3B4),
 * polled or MSI-woken. First (and so far only) chip: the RTL8821CE. */

#include <atomic>
#include <cstdint>
#include <functional>

#include "PcieDmaPlane.h"
#include "logger.h"

namespace devourer {

class PcieTransport;

class PcieDma88xx final : public IPcieDmaPlane {
public:
  /* TX queues, indexing _tx. Order is fixed (ring register map). */
  enum Queue : int {
    Q_BCN = 0, /* beacon / reserved-page (the DLFW path) */
    Q_MGMT,
    Q_VO,
    Q_VI,
    Q_BE,
    Q_BK,
    Q_HI0,
    Q_H2C,
    Q_MAX
  };

  PcieDma88xx(PcieTransport &t, Logger_t logger, uint32_t rx_ring_len,
              uint32_t rx_buf_size, int rx_poll_us);

  const char *name() const override { return "88xx-bd"; }
  size_t slab_bytes() const override;
  bool attach(uint8_t *slab_va, uint64_t slab_iova) override;
  void setup_rings() override;
  int tx_submit(uint8_t hint, uint8_t *buf, size_t len,
                int timeout_ms) override;
  void rx_loop(const std::function<void(const uint8_t *, int)> &on_data,
               const std::function<bool()> &should_stop) override;
  void irq_mask() override;
  /* 0xFE00..0xFEFF is USB-only register space — undefined over MMIO. The
   * jaguar users (0xFE5B/0xFE10/0xFE11) are is_usb()-gated; catch stragglers
   * instead of poking a hole in the BAR. */
  bool reg_allowed(uint32_t reg) const override { return reg < 0xFE00; }

  /* Synchronous TX submit on `queue`: copy into the queue's bounce buffer,
   * fill the BD slot, kick the doorbell. Non-BCN queues wait for the hardware
   * read pointer to consume the slot (timeout_ms); the BCN queue returns after
   * the kick — its completion signal is the caller-polled bcn-valid latch
   * (same contract as the USB DLFW path). Returns bytes submitted or <0. */
  int tx_submit_sync(int queue, const uint8_t *buf, size_t len, int timeout_ms);

private:
  struct TxRing {
    volatile uint8_t *bd = nullptr; /* BD slots (16 B each) in the DMA slab */
    uint64_t bd_iova = 0;
    uint8_t *bounce = nullptr; /* one in-flight frame per queue (sync TX) */
    uint64_t bounce_iova = 0;
    uint32_t bounce_len = 0;
    uint32_t len = 0; /* slots */
    uint32_t wp = 0;
    uint16_t reg_desa = 0, reg_num = 0, reg_idx = 0;
  };
  struct RxRing {
    volatile uint8_t *bd = nullptr; /* 8-byte BDs */
    uint64_t bd_iova = 0;
    uint8_t *bufs = nullptr; /* rx_ring_len × rx_buf_size */
    uint64_t bufs_iova = 0;
    uint32_t len = 0;
    uint32_t rp = 0;
  };

  void arm_rx_bd(uint32_t idx);

  PcieTransport &_t;
  Logger_t _logger;
  uint32_t _rx_ring_len;
  uint32_t _rx_buf_size;
  int _rx_poll_us;
  TxRing _tx[Q_MAX];
  RxRing _rx;
};

} /* namespace devourer */
