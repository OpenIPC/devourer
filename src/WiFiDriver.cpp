#include "WiFiDriver.h"

#include <algorithm>
#include <array>
#include <utility>

#include <libusb.h>

#include "RtlJaguarDevice.h"
#include "jaguar3/RtlJaguar3Device.h"

namespace {

/* Unambiguous RTL8822C (Jaguar3) USB product IDs — the WiFi-function default
 * IDs from the vendor rtl88x2cu usb_intf.c. These do NOT collide with any
 * Jaguar1 PID, so PID matching is safe for them. (The 8812EU/8822EU variants can
 * share the 0x8812 RTL8812AU PID; telling those apart needs the SYS_CFG chip-id
 * read and is out of scope here — only the RTL8812CU/8822CU are dispatched.) */
constexpr std::array<uint16_t, 3> kJaguar3ProductIds = {
    0xC82C, /* RTL8822CU multi-function default */
    0xC82E, /* RTL8822CU multi-function default */
    0xC812, /* RTL8822CU WiFi-only default */
};

bool is_jaguar3_pid(uint16_t pid) {
  return std::find(kJaguar3ProductIds.begin(), kJaguar3ProductIds.end(), pid) !=
         kJaguar3ProductIds.end();
}

} /* namespace */

WiFiDriver::WiFiDriver(Logger_t logger) : _logger{std::move(logger)} {}

std::unique_ptr<IRtlDevice>
WiFiDriver::CreateRtlDevice(libusb_device_handle *dev_handle,
                            libusb_context *ctx) {
  uint16_t pid = 0;
  libusb_device *dev = libusb_get_device(dev_handle);
  if (dev != nullptr) {
    libusb_device_descriptor desc{};
    if (libusb_get_device_descriptor(dev, &desc) == 0)
      pid = desc.idProduct;
  }

  if (is_jaguar3_pid(pid)) {
    _logger->info("Creating RtlJaguar3Device from LibUSB (PID 0x{:04x})", pid);
    return std::make_unique<RtlJaguar3Device>(
        RtlUsbAdapter(dev_handle, _logger, ctx), _logger);
  }

  _logger->info("Creating RtlJaguarDevice from LibUSB (PID 0x{:04x})", pid);
  return std::make_unique<RtlJaguarDevice>(RtlUsbAdapter(dev_handle, _logger),
                                           _logger);
}
