#include "RtlJaguar2Device.h"

#include <stdexcept>
#include <utility>

/* M0 scaffold: the orchestrator compiles and constructs, but bring-up is not yet
 * wired. Each entry point logs and (for the data-path calls) throws/returns so a
 * premature use is loud rather than silently wrong. Milestones M2..M7 replace
 * these bodies with the ported HalMAC / phydm / halrf sub-modules. */

RtlJaguar2Device::RtlJaguar2Device(RtlUsbAdapter device, Logger_t logger)
    : _device{std::move(device)}, _logger{std::move(logger)} {}

RtlJaguar2Device::~RtlJaguar2Device() = default;

void RtlJaguar2Device::Init(Action_ParsedRadioPacket packetProcessor,
                            SelectedChannel channel) {
  _packetProcessor = std::move(packetProcessor);
  _channel = channel;
  throw std::runtime_error(
      "RtlJaguar2Device::Init not yet implemented (RTL8822BU port in progress)");
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
