#ifndef RTL_JAGUAR3_DEVICE_H
#define RTL_JAGUAR3_DEVICE_H

#include "logger.h"
#include "IRtlDevice.h"
#include "RtlUsbAdapter.h"
#include "SelectedChannel.h"
#include "Hal8822c.h"
#include "RadioManagement8822c.h"

/* RtlJaguar3Device is the orchestrator for the Realtek "Jaguar3" 802.11ac family
 * — RTL8822CU, RTL8812EU, RTL8822EU. It is the Jaguar3 sibling of
 * RtlJaguarDevice (Jaguar1) and implements the same IRtlDevice contract so the
 * demos and WiFiDriver factory treat both uniformly.
 *
 * WORK IN PROGRESS — bring-up lands in milestones (see docs/jaguar3-bringup.md).
 * Until then every method throws a clear "not yet implemented (Mn)" error rather
 * than touching hardware. The class exists now to anchor the architecture seam
 * and the chip-family dispatch in WiFiDriver::CreateRtlDevice. */
class RtlJaguar3Device : public IRtlDevice {
public:
  RtlJaguar3Device(RtlUsbAdapter device, Logger_t logger);
  ~RtlJaguar3Device() override = default;

  void Init(Action_ParsedRadioPacket packetProcessor,
            SelectedChannel channel) override;
  void SetMonitorChannel(SelectedChannel channel) override;
  void InitWrite(SelectedChannel channel) override;
  void SetTxPower(uint8_t power) override;
  bool send_packet(const uint8_t *packet, size_t length) override;
  SelectedChannel GetSelectedChannel() override;

private:
  RtlUsbAdapter _device;
  Logger_t _logger;
  jaguar3::Hal8822c _hal;
  jaguar3::RadioManagement8822c _radioManagement;
  SelectedChannel _channel{};
  Action_ParsedRadioPacket _packetProcessor = nullptr;
};

#endif /* RTL_JAGUAR3_DEVICE_H */
