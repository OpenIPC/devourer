#ifndef IRTL_DEVICE_H
#define IRTL_DEVICE_H

#include <cstddef>
#include <cstdint>
#include <functional>

#include "SelectedChannel.h"

/* Packet is the parsed-RX type handed to the RX callback. Forward-declared here
 * (a reference in the std::function signature needs only an incomplete type) so
 * the interface header stays light; the full definition lives in FrameParser.h. */
struct Packet;

using Action_ParsedRadioPacket = std::function<void(const Packet &)>;

/* IRtlDevice is the chip-family-agnostic device contract used by the demos and
 * the WiFiDriver factory. Two implementations exist:
 *   - RtlJaguarDevice   — Realtek "Jaguar" wave-1 (8812AU/8811AU/8821AU/8814AU)
 *   - RtlJaguar3Device  — Realtek "Jaguar3" (8822CU/8812EU/8822EU)  [WIP]
 *
 * Chip-family-specific research helpers (BB-debug-port reads, the 8814 queue
 * poller, ...) are intentionally NOT part of this interface — callers that need
 * them dynamic_cast down to the concrete type. */
class IRtlDevice {
public:
  virtual ~IRtlDevice() = default;

  virtual void Init(Action_ParsedRadioPacket packetProcessor,
                    SelectedChannel channel) = 0;
  virtual void InitWrite(SelectedChannel channel) = 0;
  virtual void SetMonitorChannel(SelectedChannel channel) = 0;
  virtual void SetTxPower(uint8_t power) = 0;
  virtual bool send_packet(const uint8_t *packet, size_t length) = 0;
  virtual SelectedChannel GetSelectedChannel() = 0;
};

#endif /* IRTL_DEVICE_H */
