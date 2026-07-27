#ifndef DEVOURER_EXAMPLES_DEVICE_SESSION_H
#define DEVOURER_EXAMPLES_DEVICE_SESSION_H

/* DeviceSession — the teardown-order invariant for the demos, in one place.
 *
 * The caller owns libusb (see the architecture note in CLAUDE.md), which means
 * the caller also owns the order things die in. That order is not arbitrary:
 *
 *   1. destroy the IRtlDevice   — quiesces TX (cancels and reaps the in-flight
 *                                 bulk-OUT URBs) and drops the transport, all
 *                                 while the libusb context is still valid;
 *   2. libusb_release_interface — the chip is no longer being driven;
 *   3. libusb_close             — no transfer references the handle any more;
 *   4. libusb_exit              — nothing references the context any more.
 *
 * Getting this backwards is not a leak, it is a crash: a device destroyed
 * after libusb_exit reaps its completions against a freed context and dies
 * inside libusb, and only under enough TX load to keep URBs outstanding at
 * exit (which is why it hides from low-duty runs).
 *
 * Holding all four in one object makes the order structural instead of a
 * sequence every demo has to re-type at every early return: adopt each piece
 * as it is acquired, then just `return`.
 *
 * Header-only and demo-local by design — the library never touches libusb
 * lifetime. */

#include <memory>
#include <utility>

#include <libusb.h>

#include "IRtlDevice.h"
#include "UsbDeviceLock.h"
#include "UsbOpen.h" /* find_wifi_interface — the interface the claim used */
#include "logger.h"

namespace devourer {

class DeviceSession {
public:
  explicit DeviceSession(Logger_t logger) : _logger{std::move(logger)} {}

  DeviceSession(const DeviceSession &) = delete;
  DeviceSession &operator=(const DeviceSession &) = delete;

  ~DeviceSession() { close(); }

  /* Adopt each resource as it is acquired, so an early return from any point
   * in the bring-up unwinds whatever exists so far, in order. */
  void adopt_context(libusb_context *ctx) { _ctx = ctx; }
  /* `iface` defaults to whichever interface the claim helpers pick — the
   * vendor bulk one, which is 2 on a composite RTL8822BU, not 0. Pass it
   * explicitly only when the demo claimed a fixed interface itself. */
  void adopt_handle(libusb_device_handle *handle, int iface = -1) {
    _handle = handle;
    _iface = iface >= 0 ? iface : find_wifi_interface(handle);
  }
  void adopt_lock(std::shared_ptr<UsbDeviceLock> lock) {
    _lock = std::move(lock);
  }
  void adopt_device(std::unique_ptr<IRtlDevice> dev) { _dev = std::move(dev); }

  libusb_context *context() const { return _ctx; }
  libusb_device_handle *handle() const { return _handle; }
  IRtlDevice *device() const { return _dev.get(); }
  const std::shared_ptr<UsbDeviceLock> &lock() const { return _lock; }

  /* Explicit teardown for demos that have work to do after the adapter is
   * released (final statistics, a summary event). Idempotent; the destructor
   * calls it. */
  void close() {
    _dev.reset();
    if (_handle != nullptr) {
      /* Tolerant: an adapter that already dropped off the bus (a TX-path
       * wedge, an unplug) fails the release — log it, never abort. */
      const int rc = libusb_release_interface(_handle, _iface);
      if (rc != 0 && _logger)
        _logger->info("libusb_release_interface rc={} (device already gone?)",
                      rc);
      libusb_close(_handle);
      _handle = nullptr;
    }
    if (_ctx != nullptr) {
      libusb_exit(_ctx);
      _ctx = nullptr;
    }
    _lock.reset();
  }

private:
  Logger_t _logger;
  std::unique_ptr<IRtlDevice> _dev;
  std::shared_ptr<UsbDeviceLock> _lock;
  libusb_device_handle *_handle = nullptr;
  libusb_context *_ctx = nullptr;
  int _iface = 0;
};

} /* namespace devourer */

#endif /* DEVOURER_EXAMPLES_DEVICE_SESSION_H */
