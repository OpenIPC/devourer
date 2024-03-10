#ifndef WIFIDRIVER_H
#define WIFIDRIVER_H

#include <memory>

#include "Rtl8812aDevice.h"
#include "logger.h"

class WiFiDriver {
  Logger_t _logger;

public:
  WiFiDriver(Logger_t logger);

  std::unique_ptr<Rtl8812aDevice>
  CreateRtlDevice(libusb_device_handle *dev_handle);
};

#endif /* WIFIDRIVER_H */
