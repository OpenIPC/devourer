#pragma once

/* PcieTransport — vfio-pci userspace transport for the PCIe RTL88xx variants
 * (first target: RTL8821CE, the PCIe sibling of the RTL8821CU).
 *
 * The caller owns vfio, mirroring the USB doctrine ("the caller owns libusb"):
 * PcieTransport::Open(bdf) is the recommended open path — it walks
 * /sys/bus/pci/devices/<bdf>/iommu_group, opens the vfio container + group,
 * maps BAR2 (the 64 KiB MMIO window exposing the same 0x0000..0xFFFF register
 * space the USB vendor-control path addresses), enables PCI bus mastering, and
 * DMA-maps one anonymous slab for the TX/RX buffer-descriptor rings + RX
 * buffers. The device must already be bound to vfio-pci
 * (tests/pcie_vfio_bind.sh).
 *
 * Registers: plain volatile loads/stores on the BAR2 mapping (rtw88 pci.c maps
 * bar_id=2 and uses readb/w/l at the register address).
 *
 * TRX: the 88xx PCIe buffer-descriptor rings, ported from rtw88 pci.{c,h}
 * (v6.12) — 8-byte BD entries, 16-byte TX BD slots (entry0 = 48-byte tx desc,
 * entry1 = payload), ring base/num/idx registers at 0x300..0x3B8 + the H2C
 * ring at 0x1320. RX completion is detected by polling the hardware write
 * index in RTK_PCI_RXBD_IDX_MPDUQ (0x3B4) — no interrupts (MSI/eventfd is a
 * later optimization; monitor-mode RX at beacon rates is comfortably within a
 * sub-millisecond poll).
 *
 * All descriptor `dma` fields and the DESA registers are 32-bit, so the slab
 * is mapped at a fixed IOVA below 4 GiB (VT-d lets us choose). x86 is
 * DMA-coherent — no cache sync beyond compiler ordering (volatile BD access +
 * the strongly-ordered MMIO doorbell write). */

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "logger.h"

namespace devourer {

class PcieTransport {
public:
  /* TX queues, indexing _tx_rings. Order is fixed (ring register map). */
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

  struct Config {
    uint32_t rx_ring_len = 512;   /* RTK_MAX_RX_DESC_NUM */
    uint32_t rx_buf_size = 11480; /* RTK_PCI_RX_BUF_SIZE (11478) 8-aligned */
    uint64_t iova_base = 0x10000000; /* slab IOVA; must stay < 4 GiB */
    int rx_poll_us = 200;            /* RX hw-index poll interval */
  };

  /* Open the vfio-pci device at `bdf` ("0000:01:00.0"). Returns null and logs
   * on any failure (group not viable, BAR2 map failed, DMA map failed...).
   * Bus mastering + ASPM-off + completion-timeout-disable are applied here.
   * (Two overloads instead of a defaulted Config arg: a nested class with
   * default member initializers cannot be a default argument inside its own
   * enclosing class.) */
  static std::shared_ptr<PcieTransport> Open(const std::string &bdf,
                                             Logger_t logger,
                                             const Config &cfg);
  static std::shared_ptr<PcieTransport> Open(const std::string &bdf,
                                             Logger_t logger);
  ~PcieTransport();
  PcieTransport(const PcieTransport &) = delete;
  PcieTransport &operator=(const PcieTransport &) = delete;

  /* ---- register plane (BAR2 MMIO) ---- */
  volatile uint8_t *mmio() const { return _mmio; }
  size_t mmio_len() const { return _mmio_len; }

  /* ---- PCI config space (via the vfio config region) ---- */
  bool cfg_read(uint32_t off, void *buf, size_t len);
  bool cfg_write(uint32_t off, const void *buf, size_t len);

  /* ---- TRX rings ---- */
  /* Program the ring base/num/idx registers — port of rtw_pci_reset_buf_desc
   * + rtw_pci_dma_reset (exact order). Call per bring-up attempt, BEFORE the
   * power-on sequence (rtw88: rtw_hci_setup precedes rtw_mac_power_on). */
  void setup_trx_rings();

  /* Synchronous TX submit on `queue`: copy into the queue's bounce buffer,
   * fill the BD slot, kick the doorbell. Non-BCN queues wait for the hardware
   * read pointer to consume the slot (timeout_ms); the BCN queue returns after
   * the kick — its completion signal is the caller-polled bcn-valid latch
   * (same contract as the USB DLFW path). Returns bytes submitted or <0. */
  int tx_submit_sync(int queue, const uint8_t *buf, size_t len, int timeout_ms);

  /* Poll-driven RX reap loop: read the hw write index at 0x3B4, hand each
   * ready MPDU buffer (rx desc + drvinfo + PSDU — NO USB-style aggregation,
   * exactly one MPDU per BD) to on_data, re-arm the BD, advance the host
   * index. Runs until should_stop(). */
  void rx_poll_loop(const std::function<void(const uint8_t *, int)> &on_data,
                    const std::function<bool()> &should_stop);

  const Config &config() const { return _cfg; }
  const std::string &bdf() const { return _bdf; }

private:
  PcieTransport(Logger_t logger, Config cfg) : _logger(std::move(logger)), _cfg(cfg) {}

  bool open_vfio(const std::string &bdf);
  bool map_bar2();
  bool setup_config_space();
  bool init_dma();

  template <typename T> T mr(uint16_t reg) {
    return *reinterpret_cast<volatile T *>(_mmio + reg);
  }
  template <typename T> void mw(uint16_t reg, T v) {
    *reinterpret_cast<volatile T *>(_mmio + reg) = v;
  }

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

  Logger_t _logger;
  Config _cfg;
  std::string _bdf;

  int _container = -1, _group = -1, _device = -1;
  volatile uint8_t *_mmio = nullptr;
  size_t _mmio_len = 0;
  uint64_t _cfg_region_off = 0;
  size_t _cfg_region_len = 0;

  uint8_t *_slab = nullptr; /* DMA slab VA (anonymous, VFIO-pinned) */
  size_t _slab_len = 0;

  TxRing _tx[Q_MAX];
  RxRing _rx;
};

} /* namespace devourer */
