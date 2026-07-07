#ifndef RTLUSBADAPTER_H
#define RTLUSBADAPTER_H

#include <iostream>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <libusb.h>
#include <memory>
#include <thread>
#include <vector>

#include "DeviceConfig.h"
#include "drv_types.h"
#include "hal_com_reg.h"
#include "logger.h"
#include "TxStats.h"

namespace devourer {
class UsbDeviceLock;
class PcieTransport;
}

#define rtw_read8 rtw_read<uint8_t>
#define rtw_read16 rtw_read<uint16_t>
#define rtw_read32 rtw_read<uint32_t>

#define rtw_write8 rtw_write<uint8_t>
#define rtw_write16 rtw_write<uint16_t>
#define rtw_write32 rtw_write<uint32_t>

#define REALTEK_USB_VENQT_READ 0xC0
#define REALTEK_USB_VENQT_WRITE 0x40
#define USB_TIMEOUT 500

enum TxSele {
  TX_SELE_HQ = 1 << (0), /* High Queue */
  TX_SELE_LQ = 1 << (1), /* Low Queue */
  TX_SELE_NQ = 1 << (2), /* Normal Queue */
  TX_SELE_EQ = 1 << (3), /* Extern Queue */
};

class RtlUsbAdapter {
  libusb_device_handle *_dev_handle = nullptr; /* null on the PCIe backend */
  libusb_context *_ctx = nullptr;
  Logger_t _logger;

  /* ---- PCIe backend (DEVOURER_PCIE) ----------------------------------
   * The adapter is bus-dual: register I/O dispatches on `_mmio` — when set,
   * rtw_read/rtw_write are plain volatile loads/stores on the BAR2 mapping
   * (the same 0x0000..0xFFFF register space the USB vendor-control transfers
   * address), and the bulk TX/RX entry points route to the PCIe TRX rings.
   * `_mmio` is a raw pointer (cheap to copy — the adapter is a copyable value
   * type); `_pcie` keeps the vfio fds + DMA rings alive until the last adapter
   * copy dies, the same lifetime rule `_usb_lock` encodes for USB. Null on
   * USB; empty/null on non-PCIe builds (a forward-declared shared_ptr member
   * costs nothing). */
  volatile uint8_t *_mmio = nullptr;
  std::shared_ptr<devourer::PcieTransport> _pcie;

  enum libusb_speed usbSpeed;
  uint8_t numOutPipes = 0;
  /* Bulk-IN endpoint address used by infinite_read(). 8812AU / 8814AU expose
   * IN endpoint at 0x81; 8821AU has a different descriptor (no 0x81), so we
   * pick this up from the interface descriptor at InitDvObj time instead of
   * hardcoding. Default 0x81 keeps existing behaviour for 88[12,14]AU when
   * the descriptor walk doesn't override. */
  uint8_t _bulk_in_ep = 0x81;
  /* Bulk-OUT endpoint addresses, in descriptor order. 8812AU exposes 4 OUT
   * EPs at 0x02/0x03/0x04/0x05 (HQ/NQ/LQ/EQ); 8821AU exposes 0x05/0x06/0x08/
   * 0x09. send_packet picks index 0 (HQ-equivalent) by default; the
   * DeviceConfig tx.ep override wins for diagnostic bisection. */
  std::vector<uint8_t> _bulk_out_eps;

  /* From DeviceConfig (see there for semantics). Plain values — RtlUsbAdapter
   * is a copyable value type, every copy carries them. */
  bool _log_writes = false;                /* debug.log_writes */
  std::optional<uint8_t> _tx_ep_override;  /* tx.ep */
  unsigned _tx_timeout_ms = USB_TIMEOUT;   /* tx.timeout_ms */

  uint16_t _idVendor = 0;
  uint16_t _idProduct = 0;

  /* Set by transfer_callback when an async TX bulk-OUT completes non-OK
   * (TIMED_OUT / stall). Consumed at the top of the next send_packet on the TX
   * thread to re-clear_halt the endpoint — a mid-stream stall (e.g. hardware
   * NDP generation on some xhci hosts) would otherwise stay wedged, since the
   * first-send clear_halt only runs once. Heap-owned via shared_ptr because
   * RtlUsbAdapter is a copyable value type (passed by value throughout); an
   * atomic member would delete the copy ctor. All copies of an adapter share
   * the one flag (only the TX-driving copy touches it). */
  std::shared_ptr<std::atomic<bool>> _tx_wedged =
      std::make_shared<std::atomic<bool>>(false);

  /* TX submission counters (the driver-drop / congestion signal, see
   * TxStats.h). Heap-shared for the same reason as _tx_wedged: RtlUsbAdapter is
   * a copyable value type, and the async transfer_callback runs on whichever
   * copy submitted — all copies must increment the one set of counters. */
  struct TxStatsCounters {
    std::atomic<uint64_t> submitted{0};
    std::atomic<uint64_t> failed{0};
    std::atomic<int> last_rc{0};
    std::atomic<bool> last_timeout{false};
  };
  std::shared_ptr<TxStatsCounters> _tx_stats =
      std::make_shared<TxStatsCounters>();

  /* Exclusive per-adapter USB lock, acquired by WiFiDriver::CreateRtlDevice and
   * held for the device's lifetime (see UsbDeviceLock.h). shared_ptr because
   * RtlUsbAdapter is a copyable value type copied into every sub-manager
   * (EepromManager / RadioManagementModule / HalModule / the device itself);
   * all copies share the one lock, so it releases only when the last copy — and
   * thus the whole device — is destroyed. Null when no lock was taken (graceful
   * degradation on a lock-infrastructure error). */
  std::shared_ptr<devourer::UsbDeviceLock> _usb_lock;

public:
  RtlUsbAdapter(libusb_device_handle *dev_handle, Logger_t logger,
                libusb_context *ctx = nullptr,
                std::shared_ptr<devourer::UsbDeviceLock> usb_lock = nullptr,
                const devourer::DeviceConfig &cfg = {});

#if defined(DEVOURER_HAVE_PCIE)
  /* PCIe backend: registers via BAR2 MMIO, TX/RX via the transport's DMA
   * rings. No USB descriptor walk / endpoint state — the endpoint-flavoured
   * members stay at their defaults and are only consumed by USB-gated code. */
  RtlUsbAdapter(std::shared_ptr<devourer::PcieTransport> transport,
                Logger_t logger, const devourer::DeviceConfig &cfg = {});
#endif

  /* True on the libusb backend, false on PCIe. The HAL bring-up gates its few
   * genuinely bus-specific steps (USB RX-aggregation config, 0xFExx USB-page
   * workarounds, power-seq table choice) on this. */
  bool is_usb() const { return _dev_handle != nullptr; }
  devourer::PcieTransport *pcie() const { return _pcie.get(); }

  /* Kernel-style async RX: keep n_urbs concurrent bulk-IN transfers in flight on
   * the discovered bulk-IN endpoint, invoking on_data(buf,len) for each non-empty
   * completion and resubmitting, until `stop` is set. Mirrors the rtw88 RX URB
   * queue (the single-shot bulk_read_raw can miss the chip's RX delivery window).
   * Requires the libusb context the handle belongs to (passed at construction). */
  void bulk_read_async_loop(int buf_size, int n_urbs,
                            const std::function<void(const uint8_t *, int)> &on_data,
                            const std::function<bool()> &should_stop);
  /* Convenience overload for callers whose stop condition is a single flag. */
  void bulk_read_async_loop(int buf_size, int n_urbs,
                            const std::function<void(const uint8_t *, int)> &on_data,
                            const volatile bool &stop) {
    bulk_read_async_loop(buf_size, n_urbs, on_data,
                         [&stop]() -> bool { return stop; });
  }

  uint16_t idVendor() const { return _idVendor; }
  uint16_t idProduct() const { return _idProduct; }
  enum libusb_speed speed() const { return usbSpeed; }
  /* First discovered bulk-OUT endpoint (HQ-equivalent), or 0x02 if none were
   * discovered. Used by the Jaguar3 DLFW rsvd-page download, whose chip
   * (e.g. RTL8812CU) has no 0x02 endpoint — it exposes 0x05/0x06/0x08. */
  uint8_t first_bulk_out_ep() const {
    return _bulk_out_eps.empty() ? 0x02 : _bulk_out_eps[0];
  }

  bool AutoloadFailFlag = false;
  bool EepromOrEfuse = false;
  uint8_t OutEpQueueSel;
  uint8_t OutEpNumber;
  uint8_t rxagg_usb_size;
  uint8_t rxagg_usb_timeout;
  bool send_packet(uint8_t* packet, size_t length);
  /* Synchronous bulk-OUT transfer that blocks until completion or timeout.
   * Returns the number of bytes actually transferred, or negative on error. */
  int bulk_send_sync(uint8_t *packet, size_t length, int timeout_ms);
  void bulk_clear_halt(uint8_t ep) {
    if (_dev_handle)
      libusb_clear_halt(_dev_handle, ep);
  }
  int bulk_send_sync_ep(uint8_t ep, uint8_t *packet, size_t length,
                        int timeout_ms);

  /* Snapshot of the TX submission counters (see TxStats.h). Safe from any
   * thread — the fields are read from atomics (a torn read across the four is
   * harmless for a monitoring counter). */
  devourer::TxStats GetTxStats() const {
    devourer::TxStats s;
    s.submitted = _tx_stats->submitted.load(std::memory_order_relaxed);
    s.failed = _tx_stats->failed.load(std::memory_order_relaxed);
    s.last_error_rc = _tx_stats->last_rc.load(std::memory_order_relaxed);
    s.last_was_timeout = _tx_stats->last_timeout.load(std::memory_order_relaxed);
    return s;
  }
  /* Raw bulk-IN read on the discovered bulk-IN endpoint, returning bytes read
   * (or a negative libusb error). For chip families whose RX descriptor is not
   * the Jaguar1 layout (e.g. Jaguar3), the caller parses the buffer itself. */
  int bulk_read_raw(uint8_t *buf, int len, int timeout_ms) {
    if (!_dev_handle)
      return -1; /* PCIe RX goes through bulk_read_async_loop (ring reap) */
    int actual = 0;
    int rc = libusb_bulk_transfer(_dev_handle, _bulk_in_ep, buf, len, &actual,
                                  timeout_ms);
    return rc < 0 ? rc : actual;
  }
  uint8_t efuse_OneByteRead(uint16_t addr, uint8_t *data);
  void phy_set_bb_reg(uint16_t regAddr, uint32_t bitMask, uint32_t data);

  template <typename T> T rtw_read(uint16_t reg_num) {
    if (_mmio) {
      /* 0xFE00..0xFEFF is USB-page register space — undefined over MMIO. The
       * few jaguar users (0xFE5B/0xFE10/0xFE11) are is_usb()-gated; catch any
       * stragglers instead of poking a hole in the BAR. */
      if (reg_num >= 0xFE00) {
        _logger->warn("rtw_read(0x{:04x}) on PCIe: USB-page register, "
                      "returning 0",
                      reg_num);
        return 0;
      }
      return *reinterpret_cast<volatile T *>(_mmio + reg_num);
    }
    T data = 0;
    if (libusb_control_transfer(_dev_handle, REALTEK_USB_VENQT_READ, 5, reg_num,
                                0, (uint8_t *)&data, sizeof(T),
                                USB_TIMEOUT) == sizeof(T)) {
      return data;
    }

    _logger->error("rtw_read({:04x}), sizeof(T) = {}", reg_num, sizeof(T));
    throw std::ios_base::failure("rtw_read");
    return 0;
  }

  template <typename T> bool rtw_write(uint16_t reg_num, T value) {
    /* debug.log_writes: emit every vendor reg write as a debug.wreg event
     * (addr/width/val mirror tests/decode_wseq.py's tuple) so devourer's
     * bring-up write set can be diffed against the kernel golden. Applies to
     * both backends — the kernel-diff tooling works unchanged over MMIO. */
    if (_log_writes)
      devourer::Ev(_logger->events(), "debug.wreg")
          .hexf("addr", reg_num, 4)
          .f("width", sizeof(T))
          .hexf("val", (unsigned long long)value, (int)(sizeof(T) * 2));
    if (_mmio) {
      if (reg_num >= 0xFE00) {
        _logger->warn("rtw_write(0x{:04x}) on PCIe: USB-page register, "
                      "dropped",
                      reg_num);
        return false;
      }
      *reinterpret_cast<volatile T *>(_mmio + reg_num) = value;
      return true;
    }
    if (libusb_control_transfer(_dev_handle, REALTEK_USB_VENQT_WRITE, 5,
                                reg_num, 0, (uint8_t *)&value, sizeof(T),
                                USB_TIMEOUT) == sizeof(T)) {
      return true;
    }
    return false;
  }

  bool WriteBytes(uint16_t reg_num, uint8_t *ptr, size_t size);

  void rtl8812au_hw_reset();
  void _8051Reset8812();
  void ReadEFuseByte(uint16_t _offset, uint8_t *pbuf);

private:
  /* Async TX bulk-OUT completion callback. user_data is the RtlUsbAdapter* so
   * it can flag _tx_wedged on a non-OK completion; a static member (not a free
   * function) to reach that private state. */
  static void transfer_callback(struct libusb_transfer *transfer);
  /* PCIe TX: sniff QSEL from the tx descriptor at packet[0] and submit on the
   * matching TRX ring (QSEL_BEACON -> BCN ring = the DLFW rsvd-page path,
   * exactly rtw88's write_data_rsvd_page -> RTW_TX_QUEUE_BCN mapping).
   * Defined only in DEVOURER_PCIE builds; every call site is _pcie-gated. */
  int pcie_tx_dispatch(uint8_t *packet, size_t length, int timeout_ms);
  void InitDvObj();
  const char *strUsbSpeed();
  void GetChipOutEP8812();
  void PHY_SetBBReg8812(uint16_t regAddr, uint32_t bitMask,
                        uint32_t dataOriginal);
};

#endif /* RTLUSBADAPTER_H */
