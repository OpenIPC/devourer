#pragma once

/* PcieTransport — vfio-pci userspace transport for the PCIe Realtek parts
 * (RTL8821CE on the Jaguar2 HAL; RTL8852CE on the Kestrel HAL).
 *
 * The caller owns vfio, mirroring the USB doctrine ("the caller owns libusb"):
 * PcieTransport::Open(bdf) is the recommended open path — it walks
 * /sys/bus/pci/devices/<bdf>/iommu_group, opens the vfio container + group,
 * maps BAR2 (the MMIO window exposing the same register space the USB
 * vendor-control path addresses: 64 KiB on the 11ac parts, 1 MiB on the AX
 * parts, where the halbb/halrf BB window above 0x10000 is directly
 * addressable), enables PCI bus mastering, and DMA-maps one anonymous slab
 * for the rings + buffers. The device must already be bound to vfio-pci
 * (tests/pcie_vfio_bind.sh).
 *
 * Registers: plain volatile loads/stores on the BAR2 mapping.
 *
 * Frames: the DMA engine differs per generation and lives behind
 * IPcieDmaPlane (src/PcieDmaPlane.h), chosen from the PCI device id at
 * Open(): the 88xx buffer-descriptor rings (PcieDma88xx, rtw88) or the AX
 * HAXI TXBD/WD-page + RXQ/RPQ rings (PcieDmaAx, rtw89). RX completion is
 * polled on the ring hardware index (MSI/eventfd on the 88xx plane; the AX
 * plane polls — its interrupt wiring is not ported).
 *
 * All descriptor `dma` fields and the DESA registers are 32-bit, so the slab
 * is mapped at a fixed IOVA below 4 GiB (VT-d lets us choose). x86 is
 * DMA-coherent — no cache sync beyond compiler ordering (volatile BD access +
 * the strongly-ordered MMIO doorbell write). */

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "PcieDmaPlane.h"
#include "Transport.h"
#include "logger.h"

namespace devourer {

class PcieTransport final : public ITransport {
public:
  struct Config {
    /* 88xx plane only (the AX plane sizes its rings from the rtw89 contract:
     * 256-entry rings, 11494-byte RX buffers, 512 WD pages). */
    uint32_t rx_ring_len = 512;   /* RTK_MAX_RX_DESC_NUM */
    uint32_t rx_buf_size = 11480; /* RTK_PCI_RX_BUF_SIZE (11478) 8-aligned */
    uint64_t iova_base = 0x10000000; /* slab IOVA; must stay < 4 GiB */
    int rx_poll_us = 200;            /* RX hw-index poll interval */
    /* MSI-via-eventfd RX wakeups (VFIO_DEVICE_SET_IRQS) on a plane that
     * consumes them (the 88xx plane; the AX plane polls and never registers
     * the vector). The reap logic is identical; MSI only replaces the
     * fixed-interval sleep with an eventfd wait (100 ms safety timeout keeps
     * a lost edge from ever stalling RX). Falls back to pure polling
     * automatically when MSI setup fails. */
    bool use_msi = true;
  };

  /* Open the vfio-pci device at `bdf` ("0000:01:00.0"). Returns null and logs
   * on any failure (group not viable, BAR2 map failed, DMA map failed, no DMA
   * plane for this device id...). Bus mastering + ASPM-off +
   * completion-timeout-disable are applied here. (Two overloads instead of a
   * defaulted Config arg: a nested class with default member initializers
   * cannot be a default argument inside its own enclosing class.) */
  static std::shared_ptr<PcieTransport> Open(const std::string &bdf,
                                             Logger_t logger,
                                             const Config &cfg);
  static std::shared_ptr<PcieTransport> Open(const std::string &bdf,
                                             Logger_t logger);
  ~PcieTransport() override;
  PcieTransport(const PcieTransport &) = delete;
  PcieTransport &operator=(const PcieTransport &) = delete;

  /* ---- ITransport: register plane (BAR2 MMIO) ---- */
  bool is_usb() const override { return false; }
  uint8_t read8(uint16_t reg) override { return guarded_read<uint8_t>(reg); }
  uint16_t read16(uint16_t reg) override { return guarded_read<uint16_t>(reg); }
  uint32_t read32(uint16_t reg) override { return guarded_read<uint32_t>(reg); }
  bool write8(uint16_t reg, uint8_t v) override { return guarded_write(reg, v); }
  bool write16(uint16_t reg, uint16_t v) override { return guarded_write(reg, v); }
  bool write32(uint16_t reg, uint32_t v) override { return guarded_write(reg, v); }
  bool write_bytes(uint16_t reg, const uint8_t *p, size_t n) override {
    /* MMIO burst: plain byte stores (the multi-byte users of this path — MAC
     * address / key material — have no width side-effects). */
    for (size_t i = 0; i < n; i++)
      if (!guarded_write<uint8_t>(reg + static_cast<uint16_t>(i), p[i]))
        return false;
    return true;
  }
  /* 32-bit-address accesses: the BAR covers the whole address the HAL names
   * (the AX halbb/halrf window at +0x10000, the FWDL indirect-access entry at
   * 0x40000), so they are plain MMIO at that offset. An address past the BAR
   * (the 11ac parts' 64 KiB window) is refused with a warning rather than
   * silently aliased into MAC space. */
  bool write32_wide(uint32_t addr, uint32_t v) override;
  uint32_t read32_wide(uint32_t addr) override;

  /* ---- ITransport: frame plane (delegated to the DMA plane) ---- */
  bool tx_async(uint8_t ep, uint8_t *buf, size_t len,
                unsigned timeout_ms) override {
    return tx_sync(ep, buf, len, static_cast<int>(timeout_ms)) >= 0;
  }
  int tx_sync(uint8_t ep, uint8_t *buf, size_t len, int timeout_ms) override;
  void rx_loop(int buf_size, int n_xfers,
               const std::function<void(const uint8_t *, int)> &on_data,
               const std::function<bool()> &should_stop) override {
    (void)buf_size; /* USB URB tuning; the ring depth is fixed at creation */
    (void)n_xfers;
    _dma->rx_loop(on_data, should_stop);
  }
  /* The intf_pre_init slot: program the ring registers. Where the HAL calls
   * it is per generation (Jaguar2: before power-on; Kestrel: after
   * dmac_func_pre_en, before FWDL). */
  void hci_setup() override { _dma->setup_rings(); }
  TxStats tx_stats() const override;

  volatile uint8_t *mmio() const { return _mmio; }
  size_t mmio_len() const { return _mmio_len; }
  /* PCI config-space identity (the AX parts dispatch on the device id — their
   * 0x00FC byte is R_AX_SYS_CHIPINFO, not a Jaguar chip-id). */
  uint16_t pci_vendor_id() const { return _pci_vid; }
  uint16_t pci_device_id() const { return _pci_did; }

  /* ---- PCI config space (via the vfio config region) ---- */
  bool cfg_read(uint32_t off, void *buf, size_t len);
  bool cfg_write(uint32_t off, const void *buf, size_t len);

  /* Program the ring registers now (= hci_setup; kept for the staged probes). */
  void setup_trx_rings() { _dma->setup_rings(); }

  /* ---- plane-facing services ---- */
  /* Raw BAR2 access at a 32-bit offset, bounds-checked against the mapping
   * (out of range: read 0 / write dropped, with a warning).
   *
   * Alignment: a USB vendor request is byte-granular, so the HALs freely do
   * 32-bit RMWs on 16-bit-aligned registers (R_AX_SYS_FUNC_EN at 0x0002 is
   * the canonical one). Over MMIO that becomes a 4-byte TLP at a non-DWORD
   * address, which the root complex answers with all-ones on read and drops
   * on write — the BB never came out of reset on the 8852CE until this was
   * found. A misaligned access is therefore split into the aligned pieces
   * (16-bit halves, else bytes), assembled little-endian: same register
   * semantics as the USB path, minus atomicity across the pieces. */
  template <typename T> T mmio_read(uint32_t off) {
    if (off + sizeof(T) > _mmio_len) {
      warn_oob(off);
      return 0;
    }
    if ((off & (sizeof(T) - 1)) == 0)
      return *reinterpret_cast<volatile T *>(_mmio + off);
    return static_cast<T>(read_split(off, sizeof(T)));
  }
  template <typename T> void mmio_write(uint32_t off, T v) {
    if (off + sizeof(T) > _mmio_len) {
      warn_oob(off);
      return;
    }
    if ((off & (sizeof(T) - 1)) == 0) {
      *reinterpret_cast<volatile T *>(_mmio + off) = v;
      return;
    }
    write_split(off, sizeof(T), static_cast<uint32_t>(v));
  }
  int msi_fd() const { return _msi_evt; }
  const Config &config() const { return _cfg; }
  const std::string &bdf() const { return _bdf; }
  IPcieDmaPlane &dma_plane() { return *_dma; }

private:
  PcieTransport(Logger_t logger, Config cfg) : _logger(std::move(logger)), _cfg(cfg) {}

  bool open_vfio(const std::string &bdf);
  bool map_bar2();
  bool setup_config_space();
  bool select_dma_plane();
  bool init_dma();
  bool setup_msi();
  void warn_oob(uint32_t off);

  template <typename T> T guarded_read(uint16_t reg) {
    if (!_dma->reg_allowed(reg)) {
      _logger->warn("read(0x{:04x}) on PCIe: USB-page register, returning 0",
                    reg);
      return 0;
    }
    return mmio_read<T>(reg);
  }
  template <typename T> bool guarded_write(uint16_t reg, T v) {
    if (!_dma->reg_allowed(reg)) {
      _logger->warn("write(0x{:04x}) on PCIe: USB-page register, dropped", reg);
      return false;
    }
    mmio_write<T>(reg, v);
    return true;
  }
  uint32_t read_split(uint32_t off, size_t n);
  void write_split(uint32_t off, size_t n, uint32_t v);

  Logger_t _logger;
  Config _cfg;
  std::string _bdf;
  uint16_t _pci_vid = 0, _pci_did = 0;

  int _container = -1, _group = -1, _device = -1;
  int _msi_evt = -1;        /* eventfd signalled per MSI; -1 = polling mode */
  volatile uint8_t *_mmio = nullptr;
  size_t _mmio_len = 0;
  uint64_t _cfg_region_off = 0;
  size_t _cfg_region_len = 0;

  uint8_t *_slab = nullptr; /* DMA slab VA (anonymous, VFIO-pinned) */
  size_t _slab_len = 0;

  /* TX submission counters (TxStats.h contract, like the USB transport). */
  std::atomic<uint64_t> _tx_submitted{0};
  std::atomic<uint64_t> _tx_failed{0};
  std::atomic<int> _tx_last_rc{0};
  std::atomic<bool> _warned_oob{false};

  std::unique_ptr<IPcieDmaPlane> _dma;
};

} /* namespace devourer */
