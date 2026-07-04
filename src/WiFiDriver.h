#ifndef WIFI_DRIVER_H
#define WIFI_DRIVER_H

#include <memory>

#include "IRtlDevice.h"
#include "logger.h"

struct libusb_device_handle;
struct libusb_context;

namespace devourer {
class UsbDeviceLock;
}

class WiFiDriver {
  Logger_t _logger;

public:
  explicit WiFiDriver(Logger_t logger);

  /* Constructs the right device for the chip behind `dev_handle`:
   * RtlJaguarDevice for Jaguar wave-1 (8812/8811/8821/8814AU) or
   * RtlJaguar3Device for Jaguar3 (8822CU/8812EU/8822EU). Returns the common
   * IRtlDevice interface. See CreateRtlDevice() for how the family is chosen.
   *
   * `usb_lock` is the exclusive per-adapter lock. Pass the one returned by
   * devourer::claim_interface_then_reset (the recommended open path) so it is
   * held for the device lifetime and not re-acquired. When null (a caller that
   * opened/claimed the handle itself), CreateRtlDevice acquires its own as a
   * best-effort second gate, and returns nullptr if the adapter is already in
   * use. */
  std::unique_ptr<IRtlDevice>
  CreateRtlDevice(libusb_device_handle *dev_handle,
                  libusb_context *ctx = nullptr,
                  std::shared_ptr<devourer::UsbDeviceLock> usb_lock = nullptr);
};

#endif /* WIFI_DRIVER_H */
