#include "WiFiDriver.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>
#include <utility>

#include <libusb.h>

#include "RtlJaguarDevice.h"
#include "jaguar3/RtlJaguar3Device.h"

namespace {

/* Unambiguous RTL8822C (Jaguar3) USB product IDs — the WiFi-function default
 * IDs from the vendor rtl88x2cu/rtl88x2eu usb_intf.c. These do NOT collide with
 * any Jaguar1 PID, so PID matching is safe for them.
 *
 * CAVEAT: the 8812EU / 8822EU variants do NOT have a stable, collision-free
 * default PID — notably an 8812EU can enumerate as 0x8812, which is also the
 * RTL8812AU (Jaguar1) default. Distinguishing those two reliably needs the
 * SYS_CFG / HalVerDef chip-id read, which is deferred to M1. Until then, force
 * the family for an EU part with DEVOURER_FAMILY=jaguar3. */
constexpr std::array<uint16_t, 3> kJaguar3ProductIds = {
    0xC82C, /* RTL8822CU multi-function default */
    0xC82E, /* RTL8822CU multi-function default */
    0xC812, /* RTL8822CU WiFi-only default */
};

bool is_jaguar3_pid(uint16_t pid) {
  return std::find(kJaguar3ProductIds.begin(), kJaguar3ProductIds.end(), pid) !=
         kJaguar3ProductIds.end();
}

/* DEVOURER_FAMILY=jaguar1|jaguar3 forces the device class regardless of PID.
 * Returns 1 for jaguar1, 3 for jaguar3, 0 if unset/unrecognised. */
int forced_family() {
  const char *f = std::getenv("DEVOURER_FAMILY");
  if (f == nullptr)
    return 0;
  if (std::strcmp(f, "jaguar3") == 0 || std::strcmp(f, "3") == 0)
    return 3;
  if (std::strcmp(f, "jaguar1") == 0 || std::strcmp(f, "1") == 0)
    return 1;
  return 0;
}

} /* namespace */

WiFiDriver::WiFiDriver(Logger_t logger) : _logger{std::move(logger)} {}

std::unique_ptr<IRtlDevice>
WiFiDriver::CreateRtlDevice(libusb_device_handle *dev_handle) {
  uint16_t pid = 0;
  libusb_device *dev = libusb_get_device(dev_handle);
  if (dev != nullptr) {
    libusb_device_descriptor desc{};
    if (libusb_get_device_descriptor(dev, &desc) == 0)
      pid = desc.idProduct;
  }

  const int forced = forced_family();
  const bool jaguar3 =
      forced == 3 || (forced != 1 && is_jaguar3_pid(pid));

  if (jaguar3) {
    _logger->info("Creating RtlJaguar3Device from LibUSB (PID 0x{:04x}{})", pid,
                  forced == 3 ? ", forced by DEVOURER_FAMILY" : "");
    return std::make_unique<RtlJaguar3Device>(RtlUsbAdapter(dev_handle, _logger),
                                              _logger);
  }

  _logger->info("Creating RtlJaguarDevice from LibUSB (PID 0x{:04x})", pid);
  return std::make_unique<RtlJaguarDevice>(RtlUsbAdapter(dev_handle, _logger),
                                           _logger);
}
