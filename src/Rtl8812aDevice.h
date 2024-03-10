#ifndef RTL8812ADEVICE_H
#define RTL8812ADEVICE_H

#include <functional>

#include "logger.h"
#include "HalModule.h"
#include "ParsedRadioPacket.h"
#include "SelectedChannel.h"
#include "EepromManager.h"
#include "RadioManagementModule.h"

using Action_ParsedRadioPacket = std::function<void(const ParsedRadioPacket&)>;

class Rtl8812aDevice {
  std::shared_ptr<EepromManager> _eepromManager;
  std::shared_ptr<RadioManagementModule> _radioManagement;
  RtlUsbAdapter _device;
  HalModule _halModule;
  Logger_t _logger;
  Action_ParsedRadioPacket _packetProcessor = nullptr;

public:
  Rtl8812aDevice(RtlUsbAdapter device, Logger_t logger);
  void Init(Action_ParsedRadioPacket packetProcessor, SelectedChannel channel);
  void SetMonitorChannel(SelectedChannel channel);

private:
  void StartWithMonitorMode(SelectedChannel selectedChannel);
  bool NetDevOpen(SelectedChannel selectedChannel);
};

#endif /* RTL8812ADEVICE_H */
