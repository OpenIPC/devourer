#ifndef RTL8733B_DEVICE_H
#define RTL8733B_DEVICE_H

#include <atomic>
#include <mutex>
#include <optional>

#include "DeviceConfig.h"
#include "FrameParser8733b.h"
#include "Halmac8733bMac.h"
#include "IRtlDevice.h"
#include "Phy8733b.h"
#include "Rtl8733bBringup.h"
#include "RtlAdapter.h"
#include "SelectedChannel.h"
#include "logger.h"

/* Dedicated RTL8733B IRtlDevice boundary. Power, firmware, EFUSE, HALMAC,
 * PHY/RF, monitor RX, and bounded legacy/HT injection all use the production
 * path. Unsupported optional controls refuse loudly rather than silently
 * no-opping, but a refusal never tears the session down — asking for a knob
 * this backend has not ported is not a hardware-safety event. */
class Rtl8733bDevice : public IRtlDevice {
public:
  Rtl8733bDevice(RtlAdapter device, Logger_t logger,
                 devourer::DeviceConfig cfg = {});
  ~Rtl8733bDevice() override;

  void Init(Action_ParsedRadioPacket packetProcessor,
            SelectedChannel channel) override;
  void InitWrite(SelectedChannel channel) override;
  void StartRxLoop(Action_ParsedRadioPacket packetProcessor) override;
  void StopRxLoop() override { _rx_stop = true; }
  void SetMonitorChannel(SelectedChannel channel) override;
  bool send_packet(const uint8_t *packet, size_t length) override;
  void SetTxMode(const devourer::TxMode &mode) override;
  void ClearTxMode() override;
  SelectedChannel GetSelectedChannel() override;
  void SetCcaMode(bool disabled) override;
  void Stop() override;

  devourer::TxCaps GetTxCaps() override;
  devourer::AdapterCaps GetAdapterCaps() override;
  devourer::TxStats GetTxStats() override { return _device.GetTxStats(); }
  devourer::ThermalStatus GetThermalStatus() override;
  bool GetPermanentMacAddress(uint8_t out[6]) override;
  devourer::EfuseStability ProbeEfuseStability(int reads) override;
  devourer::FwBootStatus GetFwBootStatus() override;

private:
  void bring_up_to_phy();
  bool configure_tx_power(SelectedChannel channel);
  size_t build_tx_block(const uint8_t *packet, size_t length, uint8_t *out,
                        uint8_t packet_offset);

  RtlAdapter _device;
  Logger_t _logger;
  devourer::DeviceConfig _cfg;
  rtl8733b::Rtl8733bBringup _bringup;
  rtl8733b::Halmac8733bMac _mac;
  rtl8733b::Phy8733b _phy;
  rtl8733b::EfuseInfo _efuse{};
  SelectedChannel _channel{1, 0, CHANNEL_WIDTH_20};
  rtl8733b::ChipInfo _chip{};
  devourer::FwBootStatus _fw_boot{.supported = true};
  bool _power_ready = false;
  bool _firmware_ready = false;
  bool _mac_ready = false;
  bool _phy_ready = false;
  bool _tx_ready = false;
  bool _tssi_tracking = false;
  std::atomic<bool> _rx_stop{false};
  std::atomic<bool> _rx_active{false};
  std::atomic<uint8_t> _rx_configured_bw{0};
  std::atomic<uint64_t> _tx_submits{0};
  mutable std::recursive_mutex _reg_mu;
  std::optional<devourer::TxMode> _tx_mode_default;
};

#endif /* RTL8733B_DEVICE_H */
