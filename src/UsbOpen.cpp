#include "UsbOpen.h"

#include "UsbDeviceLock.h"
#include "kestrel/KestrelUsbIds.h"

#if defined(__ANDROID__) || defined(_MSC_VER) || defined(__APPLE__)
#include <libusb.h>
#else
#include <libusb-1.0/libusb.h>
#endif

#include <chrono>
#include <cstring>
#include <string>
#include <thread>

namespace devourer {

int find_wifi_interface(libusb_device_handle *handle) {
  if (handle == nullptr)
    return 0;

  libusb_device *device = libusb_get_device(handle);
  libusb_device_descriptor device_descriptor{};
  if (libusb_get_device_descriptor(device, &device_descriptor) != LIBUSB_SUCCESS)
    return 0;

  for (uint8_t config_index = 0;
       config_index < device_descriptor.bNumConfigurations; ++config_index) {
    libusb_config_descriptor *config = nullptr;
    if (libusb_get_config_descriptor(device, config_index, &config) !=
        LIBUSB_SUCCESS)
      continue;

    for (uint8_t interface_index = 0;
         interface_index < config->bNumInterfaces; ++interface_index) {
      const libusb_interface *interface = &config->interface[interface_index];
      if (interface->num_altsetting == 0)
        continue;

      /* libusb uses altsetting 0 unless a caller explicitly selects another.
       * Match that active layout rather than discovering endpoints that are not
       * live on the wire. */
      const libusb_interface_descriptor *candidate = &interface->altsetting[0];
      if (candidate->bInterfaceClass != LIBUSB_CLASS_VENDOR_SPEC)
        continue;

      bool has_bulk_in = false;
      bool has_bulk_out = false;
      for (uint8_t endpoint_index = 0;
           endpoint_index < candidate->bNumEndpoints; ++endpoint_index) {
        const libusb_endpoint_descriptor *endpoint =
            &candidate->endpoint[endpoint_index];
        if ((endpoint->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) !=
            LIBUSB_TRANSFER_TYPE_BULK)
          continue;
        if (endpoint->bEndpointAddress & LIBUSB_ENDPOINT_IN)
          has_bulk_in = true;
        else
          has_bulk_out = true;
      }
      if (has_bulk_in && has_bulk_out) {
        const int interface_number = candidate->bInterfaceNumber;
        libusb_free_config_descriptor(config);
        return interface_number;
      }
    }
    libusb_free_config_descriptor(config);
  }

  return 0;
}

int claim_interface_then_reset(libusb_device_handle *handle, int iface,
                               const Logger_t &logger, bool do_reset,
                               std::shared_ptr<UsbDeviceLock> &out_lock,
                               const std::string &lock_dir) {
  if (handle == nullptr)
    return LIBUSB_ERROR_NO_DEVICE;
  libusb_device *dev = libusb_get_device(handle);

  /* Advisory exclusive lock FIRST — before detach / claim / reset — so a second
   * devourer is turned away without disturbing the device at all. This is the
   * gate that actually holds on platforms where the kernel claim below does not
   * arbitrate (WinUSB). Genuine contention -> refuse; a lock-infrastructure
   * failure (e.g. read-only tmpdir) degrades to a warning and we lean on the
   * kernel claim instead. */
  auto lock = std::make_shared<UsbDeviceLock>();
  std::string why;
  switch (lock->try_acquire(dev, &why, lock_dir)) {
  case UsbDeviceLock::Result::Busy:
    logger->error("USB adapter in use — refusing to open ({})", why);
    return LIBUSB_ERROR_BUSY;
  case UsbDeviceLock::Result::Error:
    logger->warn("USB adapter lock unavailable ({}) — leaning on the kernel "
                 "interface claim for exclusivity",
                 why);
    lock.reset();
    break;
  case UsbDeviceLock::Result::Acquired:
    logger->info("USB adapter {} locked for exclusive access", lock->key());
    break;
  }

  if (libusb_kernel_driver_active(handle, iface) == 1)
    libusb_detach_kernel_driver(handle, iface);

  /* Configure before claiming: a chip that was never kernel-configured is in
   * config 0, where claim / bulk I/O fail with ESRCH. No-op if already set. */
  int cfg = 0;
  if (libusb_get_configuration(handle, &cfg) != 0 || cfg != 1)
    libusb_set_configuration(handle, 1);

  /* The kernel's exclusive claim — a redundant second gate on Linux usbfs
   * (reports BUSY); a no-op success on WinUSB, where the lock above is the
   * real guard. */
  int rc = libusb_claim_interface(handle, iface);
  if (rc != 0) {
    if (rc == LIBUSB_ERROR_BUSY)
      logger->error("USB interface {} is BUSY — another process is already "
                    "using this adapter; refusing to open",
                    iface);
    else
      logger->error("libusb_claim_interface({}) failed: {} ({})", iface,
                    libusb_error_name(rc), rc);
    return rc;
  }

  /* Kestrel (11ax) adapters never want the USB reset: their power-on sequence
   * forces the MAC off from any retained state (the real cleaner), while a
   * USB reset on running/half-torn firmware drops the chip back to its ROM —
   * a stale-handle re-enumeration that can land in the dead ZeroCD DISK id
   * (0bda:1a2b). Bench-measured: KILLed-session churn is 6/6 clean without
   * the reset vs ZeroCD roulette with it (issue #294). */
  if (do_reset) {
    libusb_device_descriptor dd{};
    if (libusb_get_device_descriptor(dev, &dd) == 0 &&
        kestrel::variant_for_usb_id(dd.idVendor, dd.idProduct).has_value()) {
      logger->info("Kestrel adapter: skipping USB reset (power-on owns state "
                   "cleanup; a reset drops running firmware into ROM/ZeroCD)");
      do_reset = false;
    }
  }

  if (do_reset) {
    /* We hold the lock and the interface, so this re-enumeration can't disturb
     * anyone else. If the reset invalidates the handle, report it; otherwise
     * re-assert the claim (the reset may have released it). */
    int rrc = libusb_reset_device(handle);
    if (rrc == LIBUSB_ERROR_NOT_FOUND) {
      logger->error("libusb_reset_device: device re-enumerated, handle is "
                    "stale — close and reopen");
      return rrc;
    }
    rc = libusb_claim_interface(handle, iface);
    if (rc != 0) {
      logger->error("re-claim of interface {} after reset failed: {} ({})",
                    iface, libusb_error_name(rc), rc);
      return rc;
    }
  }

  out_lock = std::move(lock);
  return 0;
}

int claim_interface_reset_reopen(libusb_context *ctx,
                                 libusb_device_handle *&handle,
                                 const Logger_t &logger, bool do_reset,
                                 std::shared_ptr<UsbDeviceLock> &out_lock,
                                 const std::string &lock_dir,
                                 unsigned reopen_timeout_ms) {
  if (handle == nullptr)
    return LIBUSB_ERROR_NO_DEVICE;

  /* Record the physical identity BEFORE the reset can invalidate the handle:
   * bus number + port path name the socket; VID:PID names the device we wait
   * for (a mid-re-enumeration ZeroCD id at the same port is NOT a match). */
  libusb_device *dev = libusb_get_device(handle);
  const uint8_t bus = libusb_get_bus_number(dev);
  uint8_t ports[8] = {};
  const int nports = libusb_get_port_numbers(dev, ports, sizeof(ports));
  libusb_device_descriptor dd{};
  libusb_get_device_descriptor(dev, &dd);

  int rc = claim_interface_then_reset(handle, find_wifi_interface(handle),
                                      logger, do_reset, out_lock, lock_dir);
  /* Recovery is only meaningful when we could have reset: with do_reset=false
   * a NOT_FOUND is a claim failure (interface absent), not a re-enumeration —
   * closing the handle and polling for a comeback would just burn the timeout
   * (and on Termux would orphan a wrapped fd device-list scans can't find). */
  if (rc != LIBUSB_ERROR_NOT_FOUND || !do_reset)
    return rc;

  /* The reset re-enumerated the device (firmware reload through ROM). Close
   * the stale handle and wait for the same bus+port to come back under the
   * original VID:PID, then claim WITHOUT another reset — resetting again
   * would just drop the freshly loaded firmware once more. */
  libusb_close(handle);
  handle = nullptr;
  logger->warn("USB reset re-enumerated the device — waiting for it to "
               "return on bus {} (up to {} ms)",
               bus, reopen_timeout_ms);

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(reopen_timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    libusb_device **list = nullptr;
    const ssize_t n = libusb_get_device_list(ctx, &list);
    for (ssize_t i = 0; i < n && handle == nullptr; ++i) {
      if (libusb_get_bus_number(list[i]) != bus)
        continue;
      uint8_t p[8] = {};
      const int np = libusb_get_port_numbers(list[i], p, sizeof(p));
      if (np != nports || memcmp(p, ports, static_cast<size_t>(np)) != 0)
        continue;
      libusb_device_descriptor d{};
      if (libusb_get_device_descriptor(list[i], &d) != 0)
        continue;
      if (d.idVendor != dd.idVendor || d.idProduct != dd.idProduct)
        continue; /* ZeroCD / transient ROM id — keep waiting */
      if (libusb_open(list[i], &handle) != 0)
        handle = nullptr;
    }
    if (list != nullptr)
      libusb_free_device_list(list, 1);
    if (handle != nullptr)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
  if (handle == nullptr) {
    logger->error("device did not re-enumerate as {:04x}:{:04x} within {} ms",
                  dd.idVendor, dd.idProduct, reopen_timeout_ms);
    return LIBUSB_ERROR_NOT_FOUND;
  }
  logger->info("re-opened {:04x}:{:04x} after reset re-enumeration",
               dd.idVendor, dd.idProduct);
  return claim_interface_then_reset(handle, find_wifi_interface(handle),
                                    logger, /*do_reset=*/false, out_lock,
                                    lock_dir);
}

} // namespace devourer
