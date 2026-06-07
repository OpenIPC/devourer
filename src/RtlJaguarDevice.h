#ifndef RTL_JAGUAR_DEVICE_H
#define RTL_JAGUAR_DEVICE_H

#include <array>
#include <atomic>
#include <functional>
#include <iostream>
#include <iomanip>
#include <thread>

#include "logger.h"
#include "HalModule.h"
#include "SelectedChannel.h"
#include "EepromManager.h"
#include "RadioManagementModule.h"
#include "FrameParser.h"

extern "C"
{
#include "ieee80211_radiotap.h"
}

using Action_ParsedRadioPacket = std::function<void(const Packet&)>;

/* RtlJaguarDevice is the orchestrator for the Realtek "Jaguar" 802.11ac family
 * — RTL8812AU (2T2R), RTL8811AU (1T1R cut), and RTL8814AU (4T4R RF / 3-SS
 * baseband). The chip is identified at construction time via SYS_CFG bits and
 * USB PID; this class drives bring-up, RX, and TX for whichever member of the
 * family is present. */
class RtlJaguarDevice {
  std::shared_ptr<EepromManager> _eepromManager;
  std::shared_ptr<RadioManagementModule> _radioManagement;
  SelectedChannel _channel;
  RtlUsbAdapter _device;
  HalModule _halModule;
  Logger_t _logger;
  uint8_t debug;
  Action_ParsedRadioPacket _packetProcessor = nullptr;

public:
  RtlJaguarDevice(RtlUsbAdapter device, Logger_t logger);
  ~RtlJaguarDevice();
  void Init(Action_ParsedRadioPacket packetProcessor, SelectedChannel channel);
  void SetMonitorChannel(SelectedChannel channel);
  void InitWrite(SelectedChannel channel);
  void SetTxPower(uint8_t power);
  bool send_packet(const uint8_t* packet, size_t length);
  SelectedChannel GetSelectedChannel();
  bool should_stop = false;

  /* Per-queue free-page snapshot read from REG_FIFOPAGE_INFO_1..5
   * (0x0230 / 0x0234 / 0x0238 / 0x023C / 0x0240). 8814A-only — these
   * registers don't exist on 8812 / 8821, so the snapshot stays at all
   * zeros on those chips and start_queue_depth_poller() is a no-op.
   *
   * Polling cadence is set via interval_ms (DEVOURER_QUEUE_POLL_MS in
   * the demos). 0 = disabled. The poller spawns a worker thread that
   * issues a vendor-control read for each register every interval and
   * stores the raw 32-bit word — interpretation (which bits are free
   * pages vs reserved) is left to the consumer, since the per-queue
   * layout differs slightly between chip cuts. */
  void start_queue_depth_poller(uint32_t interval_ms);
  std::array<uint32_t, 5> get_queue_depth() const;

private:
  void StartWithMonitorMode(SelectedChannel selectedChannel);
  bool NetDevOpen(SelectedChannel selectedChannel);

  std::array<std::atomic<uint32_t>, 5> _qd_snap{};
  std::thread _qd_thread;
  std::atomic<bool> _qd_stop{false};
};

/* Backwards-compatibility alias. External callers using the old name still
 * compile but get a deprecation warning. Remove after one release cycle. */
using Rtl8812aDevice [[deprecated("renamed to RtlJaguarDevice")]] =
    RtlJaguarDevice;

#endif /* RTL_JAGUAR_DEVICE_H */
