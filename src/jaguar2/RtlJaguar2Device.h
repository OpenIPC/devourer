#ifndef RTL_JAGUAR2_DEVICE_H
#define RTL_JAGUAR2_DEVICE_H

#include <optional>

#include "logger.h"
#include "IRtlDevice.h"
#include "TxMode.h"
#include "RtlUsbAdapter.h"
#include "SelectedChannel.h"
#include "HalJaguar2.h"
#include "HalmacJaguar2MacInit.h"
#include "HalmacJaguar2Fw.h"

/* RtlJaguar2Device is the orchestrator for the Realtek "Jaguar2" 802.11ac family
 * — RTL8822BU (chip 8822B, 2T2R, USB). It is the Jaguar2 sibling of
 * RtlJaguarDevice (Jaguar1) and RtlJaguar3Device (Jaguar3) and implements the
 * same IRtlDevice contract so the demos and the WiFiDriver factory treat all
 * three uniformly.
 *
 * 8822B is a hybrid of the two existing generations: firmware download, MAC init
 * and power sequencing follow the HalMAC path (like Jaguar3), while the phydm
 * BB/AGC/RF register tables use the older `check_positive` format (like Jaguar1,
 * via the shared PhyTableLoader). Bring-up is ported from the vendor
 * rtl88x2bu tree. Single chip — no per-variant strategy dispatch.
 *
 * Milestone status: scaffold (M0). The bring-up sub-modules (HalJaguar2,
 * HalmacJaguar2Fw/MacInit, RadioManagementJaguar2, FrameParserJaguar2,
 * Halrf8822b) are added as the port progresses. */
class RtlJaguar2Device : public IRtlDevice {
public:
  RtlJaguar2Device(RtlUsbAdapter device, Logger_t logger);
  ~RtlJaguar2Device() override;

  void Init(Action_ParsedRadioPacket packetProcessor,
            SelectedChannel channel) override;
  void SetMonitorChannel(SelectedChannel channel) override;
  void InitWrite(SelectedChannel channel) override;
  void SetTxPower(uint8_t power) override;
  bool send_packet(const uint8_t *packet, size_t length) override;
  SelectedChannel GetSelectedChannel() override;
  void Stop() override;
  void SetTxMode(const devourer::TxMode &mode) override;
  void ClearTxMode() override;

private:
  RtlUsbAdapter _device;
  Logger_t _logger;
  jaguar2::HalJaguar2 _hal;
  jaguar2::HalmacJaguar2MacInit _macinit;
  jaguar2::HalmacJaguar2Fw _fw;
  SelectedChannel _channel{};
  Action_ParsedRadioPacket _packetProcessor = nullptr;
  int _tx_pwr_override = -1;
  std::optional<devourer::TxMode> _tx_mode_default;
};

#endif /* RTL_JAGUAR2_DEVICE_H */
