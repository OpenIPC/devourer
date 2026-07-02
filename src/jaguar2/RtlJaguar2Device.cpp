#include "RtlJaguar2Device.h"

#include <stdexcept>
#include <utility>

/* M0 scaffold: the orchestrator compiles and constructs, but bring-up is not yet
 * wired. Each entry point logs and (for the data-path calls) throws/returns so a
 * premature use is loud rather than silently wrong. Milestones M2..M7 replace
 * these bodies with the ported HalMAC / phydm / halrf sub-modules. */

RtlJaguar2Device::RtlJaguar2Device(RtlUsbAdapter device, Logger_t logger)
    : _device{device}, _logger{logger}, _hal{device, logger},
      _macinit{device, logger}, _fw{device, logger} {}

RtlJaguar2Device::~RtlJaguar2Device() = default;

void RtlJaguar2Device::Init(Action_ParsedRadioPacket packetProcessor,
                            SelectedChannel channel) {
  _packetProcessor = std::move(packetProcessor);
  _channel = channel;
  /* M2 power-on + M3 firmware DLFW bring-up (validatable on hardware). Order
   * mirrors HalJaguar3::rtw_hal_init: pre-init -> power-on -> chip-version ->
   * init_system_cfg (DDMA/SYS_FUNC_EN) -> firmware download. The remaining
   * bring-up (post-DLFW MAC cfg, BB/AGC/RF tables, channel set, RX enable) lands
   * next in M4. */
  const uint8_t bw = static_cast<uint8_t>(channel.ChannelWidth);
  _macinit.pre_init_system_cfg();
  _hal.power_on();
  _hal.read_chip_version();
  _macinit.init_system_cfg(channel.ChannelWidth, _hal.chip_version().cut);
  /* Firmware download BEFORE trx/queue config — the HalMAC _halmac_init_hal
   * order (matches the working jaguar3 flow and the vendor rtl88x2bu golden):
   * download_firmware_88xx self-configures its own temporary HIQ page map and
   * restores it. Running init_trx_cfg first over-allocates the FIFOPAGE_INFO
   * queues, which wedges the rsvd-page bcn-valid latch on the 2nd (IMEM)
   * DLFW segment. */
  if (!_fw.download_default_firmware())
    throw std::runtime_error("RtlJaguar2Device: firmware DLFW failed");
  _logger->info("RtlJaguar2Device: firmware booted (bw={})", (int)bw);

  /* M4: post-DLFW MAC config + USB RX-DMA + BB/RF enable (HalMAC
   * _halmac_init_hal order: init_mac_flow -> init_mac_register -> EN_BB_RF). */
  if (!_macinit.init_mac_cfg(channel.ChannelWidth))
    throw std::runtime_error("RtlJaguar2Device: init_mac_cfg failed");
  _macinit.init_usb_cfg();
  _macinit.enable_bb_rf(true);
  _logger->info("RtlJaguar2Device: MAC cfg + BB/RF enabled");

  throw std::runtime_error(
      "RtlJaguar2Device: MAC/USB/BB-RF up; PHY tables + RX not yet implemented "
      "(RTL8822BU port at M4)");
}

void RtlJaguar2Device::InitWrite(SelectedChannel channel) {
  _channel = channel;
  throw std::runtime_error(
      "RtlJaguar2Device::InitWrite not yet implemented (RTL8822BU port in "
      "progress)");
}

void RtlJaguar2Device::SetMonitorChannel(SelectedChannel channel) {
  _channel = channel;
}

void RtlJaguar2Device::SetTxPower(uint8_t power) { _tx_pwr_override = power; }

bool RtlJaguar2Device::send_packet(const uint8_t * /*packet*/,
                                   size_t /*length*/) {
  return false;
}

SelectedChannel RtlJaguar2Device::GetSelectedChannel() { return _channel; }

void RtlJaguar2Device::Stop() {}

void RtlJaguar2Device::SetTxMode(const devourer::TxMode &mode) {
  _tx_mode_default = mode;
}

void RtlJaguar2Device::ClearTxMode() { _tx_mode_default.reset(); }
