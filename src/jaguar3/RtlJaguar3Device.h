#ifndef RTL_JAGUAR3_DEVICE_H
#define RTL_JAGUAR3_DEVICE_H

#include <thread>

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
 * Bring-up is ported from Realtek vendor source (rtl88x2cu/phydm/halrf):
 * power-on, firmware download, MAC/BB/RF config, IQK and DACK drive RX,
 * channel/bandwidth (incl. 5/10 MHz narrowband) and on-air TX. send_packet is
 * on-air; sustained continuous TX is kept alive by the coex runtime thread
 * (coex_runtime_loop) — see CLAUDE.md. */
class RtlJaguar3Device : public IRtlDevice {
public:
  RtlJaguar3Device(RtlUsbAdapter device, Logger_t logger);
  ~RtlJaguar3Device() override;

  void Init(Action_ParsedRadioPacket packetProcessor,
            SelectedChannel channel) override;
  void SetMonitorChannel(SelectedChannel channel) override;
  void InitWrite(SelectedChannel channel) override;
  void SetTxPower(uint8_t power) override;
  bool send_packet(const uint8_t *packet, size_t length) override;
  SelectedChannel GetSelectedChannel() override;
  void Stop() override;
  bool should_stop = false;

private:
  RtlUsbAdapter _device;
  Logger_t _logger;
  jaguar3::Hal8822c _hal;
  jaguar3::RadioManagement8822c _radioManagement;
  SelectedChannel _channel{};
  Action_ParsedRadioPacket _packetProcessor = nullptr;
  /* Optional flat TXAGC override from SetTxPower(); -1 = use the chip's
   * efuse-calibrated power. Applied during InitWrite (so it may be set before
   * the device is brought up). */
  int _tx_pwr_override = -1;
  /* Coex runtime: a background thread that drains bulk-IN, dispatches firmware
   * C2H reports (BT-info etc.) and runs the periodic coex decision so the FW's
   * PTA keeps the antenna with WLAN during sustained TX.
   * THREADING CONTRACT: while a TX session is active (between InitWrite and Stop)
   * this thread owns all chip register access. The TX hot path (send_packet) does
   * no register I/O, so it runs lock-free alongside it; callers must NOT invoke
   * SetMonitorChannel/SetTxPower (which do register RMW) during an active TX
   * session, or those reads-modify-writes will race the coex thread. */
  std::thread _coex_thread;
  volatile bool _coex_stop = false;
  void coex_runtime_loop();
};

#endif /* RTL_JAGUAR3_DEVICE_H */
