#include "RtlKestrelDevice.h"

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

/* mac_ax get_chip_info() register pair (reference/rtl8852bu
 * phl/hal_g6/mac/mac_ax.c + mac_reg_ax.h). R_AX_SYS_CHIPINFO shares the
 * 0x00FC address with the 11ac SYS_CFG2 dispatch byte — but the value space
 * is the AX D-die id, not the 11ac chip-id. */
constexpr uint16_t kRegSysChipInfo = 0x00FC; /* R_AX_SYS_CHIPINFO */
constexpr uint16_t kRegSysCfg1Hi = 0x00F1;   /* R_AX_SYS_CFG1 + 1 */

const char *die_name(uint8_t die_id) {
  switch (die_id) {
  case 0x50:
    return "8852A";
  case 0x51:
    return "8852B";
  case 0x52:
    return "8852C";
  case 0x54:
    return "8851B";
  default:
    return "unknown";
  }
}

} /* namespace */

RtlKestrelDevice::RtlKestrelDevice(RtlAdapter device, Logger_t logger,
                                   kestrel::ChipVariant variant,
                                   devourer::DeviceConfig cfg)
    : _device{device}, _logger{logger}, _variant{variant},
      _cfg{std::move(cfg)}, _hal{device, logger, variant} {
  /* Confirm the PID-selected variant against the on-chip die id (the PID
   * table is authoritative for dispatch; this catches a mislabeled board or a
   * stale table row before any bring-up write happens). */
  const auto info = ReadChipInfo();
  if (!info.matches(_variant))
    _logger->warn("Kestrel: PID-selected variant {} but die-id 0x{:02x} ({}) "
                  "— check the KestrelUsbIds table row for this adapter",
                  _variant == kestrel::ChipVariant::C8852B ? "8852B" : "8852C",
                  info.die_id, die_name(info.die_id));
  else
    _logger->info("Kestrel: die-id 0x{:02x} ({}), cut {}", info.die_id,
                  die_name(info.die_id), info.cut);
}

kestrel::ChipInfo RtlKestrelDevice::ReadChipInfo() {
  kestrel::ChipInfo info{};
  info.die_id = _device.rtw_read8(kRegSysChipInfo);
  info.cut = static_cast<uint8_t>(_device.rtw_read8(kRegSysCfg1Hi) >> 4);
  return info;
}

bool RtlKestrelDevice::PowerOnAndReadEfuse(kestrel::EfuseInfo &out) {
  if (!_hal.power_on())
    return false;
  return _hal.read_efuse(out);
}

void RtlKestrelDevice::not_ported(const char *entry,
                                  const char *milestone) const {
  _logger->error("Kestrel: {} is not ported yet ({})", entry, milestone);
  throw std::runtime_error(std::string("Kestrel ") + entry +
                           " not ported yet (" + milestone + ")");
}

void RtlKestrelDevice::Init(Action_ParsedRadioPacket /*packetProcessor*/,
                            SelectedChannel channel) {
  _channel = channel;
  not_ported("Init", "M1 power-on/FW + M2 RX");
}

void RtlKestrelDevice::InitWrite(SelectedChannel channel) {
  _channel = channel;
  not_ported("InitWrite", "M1 power-on/FW + M4 TX");
}

void RtlKestrelDevice::StartRxLoop(
    Action_ParsedRadioPacket /*packetProcessor*/) {
  not_ported("StartRxLoop", "M2 RX");
}

void RtlKestrelDevice::SetMonitorChannel(SelectedChannel channel) {
  _channel = channel;
  not_ported("SetMonitorChannel", "M3 channel/BW");
}

bool RtlKestrelDevice::send_packet(const uint8_t * /*packet*/,
                                   size_t /*length*/) {
  _logger->error("Kestrel: send_packet is not ported yet (M4 TX)");
  return false;
}
