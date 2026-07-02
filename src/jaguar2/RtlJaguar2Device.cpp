#include "RtlJaguar2Device.h"

#include <cstdint>
#include <span>
#include <stdexcept>
#include <utility>

#include "FrameParserJaguar2.h"
#include "RxPacket.h"
#include "SignalStop.h" /* g_devourer_should_stop */

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

  /* PHY: EFUSE rfe_type -> BB/AGC/RF phydm tables (init_bb_reg/init_rf_reg). */
  uint8_t rfe = _hal.read_efuse_rfe();
  _hal.apply_bb_rf_agc_tables(rfe);
  _logger->info("RtlJaguar2Device: PHY tables applied");

  /* TRX antenna-path / RF-mode-table setup (config_phydm_trx_mode_8822b) — puts
   * the RF paths into TRX mode so the front-end can receive. */
  _hal.config_trx_mode();

  /* Channel + bandwidth (RF18 tune, RFE pins, RX-path/IGI), then enable the
   * MAC RX engine. */
  _hal.set_channel_bw(static_cast<uint8_t>(channel.Channel), bw, rfe);
  _hal.enable_rx();
  _logger->info("RtlJaguar2Device: entering RX loop (ch={})", channel.Channel);

  /* RX loop: async bulk-IN URB queue; walk the aggregated 8822B RX descriptors
   * per completion and hand each PSDU to the packet processor. */
  uint64_t frames = 0, reads = 0;
  auto on_data = [&](const uint8_t *data, int n) {
    if (++reads <= 8)
      _logger->info("Jaguar2 RX: completion #{} -> {} bytes", reads, n);
    uint32_t off = 0;
    while (off + jaguar2::RXDESC_SIZE_8822B <= static_cast<uint32_t>(n)) {
      jaguar2::Rx8822bFrame f;
      if (!jaguar2::parse_rx_8822b(data + off, static_cast<size_t>(n) - off, f))
        break;
      if (_packetProcessor) {
        Packet p{};
        p.RxAtrib.pkt_len = static_cast<uint16_t>(f.frame_len);
        p.RxAtrib.crc_err = f.crc_err;
        p.RxAtrib.icv_err = f.icv_err;
        p.RxAtrib.data_rate = f.rx_rate;
        p.RxAtrib.drvinfo_sz = static_cast<uint8_t>(f.drvinfo_size);
        p.RxAtrib.shift_sz = f.shift;
        p.RxAtrib.pkt_rpt_type = RX_PACKET_TYPE::NORMAL_RX;
        p.Data =
            std::span<uint8_t>(const_cast<uint8_t *>(f.frame), f.frame_len);
        _packetProcessor(p);
      }
      if (++frames <= 5)
        _logger->info("Jaguar2 RX: frame len={} rate={} crc={}", f.frame_len,
                      f.rx_rate, f.crc_err);
      if (f.next_offset == 0)
        break;
      off += f.next_offset;
    }
  };
  _device.bulk_read_async_loop(32 * 1024, 8, on_data, g_devourer_should_stop);
  _logger->info("RtlJaguar2Device: RX loop exited ({} frames, {} reads)", frames,
                reads);
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
