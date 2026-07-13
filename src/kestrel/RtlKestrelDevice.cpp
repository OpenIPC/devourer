#include "RtlKestrelDevice.h"

#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

#include "FrameParserKestrel.h"
#include "RxPacket.h"

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

bool RtlKestrelDevice::PowerOnEfuseAndFw(kestrel::EfuseInfo &out) {
  if (!_hal.power_on())
    return false;
  if (!_hal.read_efuse(out))
    return false;
  return _hal.download_firmware(_hal.read_cut());
}

bool RtlKestrelDevice::PowerOnFwAndTrx(kestrel::EfuseInfo &out) {
  if (!_hal.power_on())
    return false;
  if (!_hal.read_efuse(out))
    return false;
  if (!_hal.download_firmware(_hal.read_cut()))
    return false;
  /* Vendor mac_hal_init order: FWDL -> set_enable_bb_rf -> sys_init/trx_init.
   * Enabling BB/RF here (not deferred to phy_bb_rf_init) means the firmware
   * sees the BB/RF live as it starts its runtime init. */
  _hal.enable_bb_rf();
  if (!_hal.trx_dmac_init())
    return false;
  return _hal.trx_cmac_rx_init();
}

bool RtlKestrelDevice::PowerOnTrxAndPhy(kestrel::EfuseInfo &out) {
  if (!PowerOnFwAndTrx(out))
    return false;
  return _hal.phy_bb_rf_init(out.rfe_type, _hal.read_cut());
}

void RtlKestrelDevice::not_ported(const char *entry,
                                  const char *milestone) const {
  _logger->error("Kestrel: {} is not ported yet ({})", entry, milestone);
  throw std::runtime_error(std::string("Kestrel ") + entry +
                           " not ported yet (" + milestone + ")");
}

bool RtlKestrelDevice::BringUpMonitor(SelectedChannel channel) {
  _channel = channel;
  if (!_hal.power_on())
    return false;
  if (!_hal.read_efuse(_efuse))
    return false;
  if (!_hal.download_firmware(_hal.read_cut()))
    return false;
  /* Vendor mac_hal_init order: FWDL -> set_enable_bb_rf -> sys_init/trx_init.
   * Enabling BB/RF here (not deferred to phy_bb_rf_init) means the firmware
   * sees the BB/RF live as it starts its runtime init. */
  _hal.enable_bb_rf();
  if (!_hal.trx_dmac_init())
    return false;
  if (!_hal.trx_cmac_rx_init())
    return false;
  if (!_hal.phy_bb_rf_init(_efuse.rfe_type, _hal.read_cut()))
    return false;
  return _hal.set_channel(channel.Channel, channel.ChannelWidth);
}

void RtlKestrelDevice::Init(Action_ParsedRadioPacket packetProcessor,
                            SelectedChannel channel) {
  if (!BringUpMonitor(channel)) {
    _logger->error("Kestrel: monitor bring-up failed");
    return;
  }
  StartRxLoop(std::move(packetProcessor));
}

void RtlKestrelDevice::InitWrite(SelectedChannel channel) {
  _channel = channel;
  not_ported("InitWrite", "M4 TX");
}

void RtlKestrelDevice::StartRxLoop(Action_ParsedRadioPacket packetProcessor) {
  _rx_stop = false;
  _logger->info("Kestrel: starting RX loop on ch{}", _channel.Channel);
  /* Async bulk-IN ring; walk each aggregate with the 11ax rxd parser. */
  int reads = 0;
  _device.bulk_read_async_loop(
      16384, 8,
      [&](const uint8_t *data, int n) {
        if (++reads <= 12)
          _logger->info("Kestrel RX: bulk-IN completion #{} -> {} bytes "
                        "(rxd0=0x{:08x})",
                        reads, n,
                        n >= 4 ? (data[0] | (data[1] << 8) | (data[2] << 16) |
                                  (data[3] << 24))
                               : 0);
        uint32_t off = 0;
        while (off + 16 <= static_cast<uint32_t>(n)) {
          kestrel::KestrelRxFrame f;
          if (!kestrel::parse_rx_8852b(data + off, static_cast<size_t>(n) - off,
                                       f))
            break;
          if (f.rpkt_type == kestrel::RPKT_TYPE_WIFI && packetProcessor) {
            Packet p{};
            p.RxAtrib.pkt_len = static_cast<uint16_t>(f.payload_len);
            p.RxAtrib.crc_err = f.crc_err;
            p.RxAtrib.icv_err = f.icv_err;
            p.RxAtrib.data_rate = static_cast<uint8_t>(f.rx_rate);
            p.RxAtrib.tsfl = f.freerun_cnt;
            p.Data = std::span<uint8_t>(const_cast<uint8_t *>(f.payload),
                                        f.payload_len);
            packetProcessor(p);
          }
          if (f.next_offset == 0)
            break;
          off += f.next_offset;
        }
      },
      _rx_stop);
}

void RtlKestrelDevice::SetMonitorChannel(SelectedChannel channel) {
  _channel = channel;
  _hal.set_channel(channel.Channel, channel.ChannelWidth);
}

bool RtlKestrelDevice::send_packet(const uint8_t * /*packet*/,
                                   size_t /*length*/) {
  _logger->error("Kestrel: send_packet is not ported yet (M4 TX)");
  return false;
}
