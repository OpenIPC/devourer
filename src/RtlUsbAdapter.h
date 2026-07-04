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

#include "drv_types.h"
#include "hal_com_reg.h"
#include "logger.h"

namespace devourer {
class UsbDeviceLock;
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
  libusb_device_handle *_dev_handle;
  libusb_context *_ctx = nullptr;
  Logger_t _logger;

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
   * 0x09. send_packet picks index 0 (HQ-equivalent) by default; DEVOURER_TX_EP
   * env override still wins for diagnostic bisection. */
  std::vector<uint8_t> _bulk_out_eps;

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
                std::shared_ptr<devourer::UsbDeviceLock> usb_lock = nullptr);

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
  void bulk_clear_halt(uint8_t ep) { libusb_clear_halt(_dev_handle, ep); }
  int bulk_send_sync_ep(uint8_t ep, uint8_t *packet, size_t length,
                        int timeout_ms);
  /* Raw bulk-IN read on the discovered bulk-IN endpoint, returning bytes read
   * (or a negative libusb error). For chip families whose RX descriptor is not
   * the Jaguar1 layout (e.g. Jaguar3), the caller parses the buffer itself. */
  int bulk_read_raw(uint8_t *buf, int len, int timeout_ms) {
    int actual = 0;
    int rc = libusb_bulk_transfer(_dev_handle, _bulk_in_ep, buf, len, &actual,
                                  timeout_ms);
    return rc < 0 ? rc : actual;
  }
  uint8_t efuse_OneByteRead(uint16_t addr, uint8_t *data);
  void phy_set_bb_reg(uint16_t regAddr, uint32_t bitMask, uint32_t data);

  template <typename T> T rtw_read(uint16_t reg_num) {
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
    /* DEVOURER_LOG_WRITES=1: emit every vendor reg write as "0xADDR N 0xVAL"
     * (matches tests/decode_wseq.py output) so devourer's bring-up write set can
     * be diffed against the kernel golden. Cached so getenv runs once. */
    static const int log_writes = getenv("DEVOURER_LOG_WRITES") ? 1 : 0;
    if (log_writes)
      fprintf(stderr, "<wlog>0x%04x %zu 0x%0*x\n", reg_num, sizeof(T),
              (int)(sizeof(T) * 2), (unsigned)value);
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
  void InitDvObj();
  const char *strUsbSpeed();
  void GetChipOutEP8812();
  void PHY_SetBBReg8812(uint16_t regAddr, uint32_t bitMask,
                        uint32_t dataOriginal);
};

#endif /* RTLUSBADAPTER_H */
