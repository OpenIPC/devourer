#include "WiFiDriver.h"

#include <cstdint>
#include <utility>

#include <libusb.h>

#include "jaguar1/RtlJaguarDevice.h"
#include "RtlUsbAdapter.h"
#include "jaguar3/RtlJaguar3Device.h"

namespace {

/* Hardware chip-id at REG_SYS_CFG2 (0x00FC), read with a single vendor control
 * transfer (no full RtlUsbAdapter, so the Jaguar1 construction path is unchanged
 * and only one extra control read is added). Port of halmac get_chip_info
 * (chip_id = REG_READ_8(REG_SYS_CFG2)):
 *   0x13 = RTL8822C, 0x17 = RTL8822E (RTL8812EU / RTL8822EU)  -> Jaguar3
 *   0x04 / 0x05 / 0x08 / 0x09 = 8812A / 8821A / 8814A / 8821C -> Jaguar1
 * The chip-id (not the USB PID) is authoritative because the rtl8822e RTL8812EU
 * shares PID 0x8812 with the Jaguar1 RTL8812AU. Returns 0 on a failed read,
 * which falls through to the Jaguar1 path. */
uint8_t read_chip_id(libusb_device_handle *dev_handle) {
  uint8_t id = 0;
  libusb_control_transfer(dev_handle, REALTEK_USB_VENQT_READ, 5, 0x00FC, 0, &id,
                          sizeof(id), USB_TIMEOUT);
  return id;
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

  uint8_t chip_id = read_chip_id(dev_handle);
  if (chip_id == 0x13 || chip_id == 0x17) {
    auto variant = chip_id == 0x17 ? jaguar3::ChipVariant::C8822E
                                   : jaguar3::ChipVariant::C8822C;
    _logger->info("Creating RtlJaguar3Device (PID 0x{:04x}, chip-id 0x{:02x})",
                  pid, chip_id);
    return std::make_unique<RtlJaguar3Device>(
        RtlUsbAdapter(dev_handle, _logger, ctx), _logger, variant);
  }

  _logger->info("Creating RtlJaguarDevice (PID 0x{:04x}, chip-id 0x{:02x})", pid,
                chip_id);
  return std::make_unique<RtlJaguarDevice>(RtlUsbAdapter(dev_handle, _logger),
                                           _logger);
}
