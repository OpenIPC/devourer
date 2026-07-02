#include "RtlJaguar2Device.h"

#include <cstdint>
#include <cstdlib>
#include <chrono>
#include <span>
#include <thread>
#include <stdexcept>
#include <utility>

#include "FrameParserJaguar2.h"
#include "Halrf8822b.h"
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
  _hal.do_lck(); /* LC calibration — lock the RF LO so the front-end receives */

  /* IQK (halrf phy_iq_calibrate_8822b): LOK/TXK/RXK per path so the RX front-end
   * resolves real frames into MPDUs (marginal decode without it -> RXPKT_NUM=0). */
  if (!getenv("DEVOURER_SKIP_IQK")) {
    jaguar2::Halrf8822b halrf(_device, _logger, _hal.chip_version().cut,
                              _hal.chip_version().rf_2t2r != 0);
    halrf.iqk_trigger(channel.Channel <= 14);
  } else {
    _logger->info("Jaguar2: IQK SKIPPED (DEVOURER_SKIP_IQK)");
  }
  /* Grant the antenna to WLAN (combo chip) — must precede enable_rx or the WL
   * RX front-end stays deaf with the antenna owned by BT. */
  _hal.coex_wlan_only();
  _hal.enable_rx();

  /* RX bring-up register dump (DEVOURER_RX_DEBUG): confirms the enable_rx /
   * channel / RF-mode / coex writes landed. The phydm BB decode counters
   * (0xF04/0xF14/0xF08/0xF48) are hold-type and only advance when the DIG/FA
   * thread pulses their reset, so they are NOT a reliable live-RX signal here —
   * dumped for reference only. */
  if (getenv("DEVOURER_RX_DEBUG")) {
    _logger->info(
        "Jaguar2 RXDBG: CR=0x{:04x} RCR=0x{:08x} RXpath(0x808)=0x{:08x} "
        "RF0A/B(0x0)=0x{:05x}/0x{:05x} RF18A/B=0x{:05x}/0x{:05x} "
        "RFmodeTbl(0xc08/0xe08)=0x{:08x}/0x{:08x}",
        _device.rtw_read16(0x0100), _device.rtw_read32(0x0608),
        _device.rtw_read32(0x0808), _hal.dbg_rf_read(0, 0x0),
        _hal.dbg_rf_read(1, 0x0), _hal.dbg_rf_read(0, 0x18),
        _hal.dbg_rf_read(1, 0x18), _device.rtw_read32(0x0c08),
        _device.rtw_read32(0x0e08));
    /* MAC RX-FIFO occupancy poll (0x0284): RXPKT_NUM[31:24] = packets buffered
     * awaiting DMA, RXDMA_IDLE=BIT17, RXPKT_FULL=BIT16. Distinguishes WMAC-drop
     * (stays 0) from USB-DMA-stall (climbs). Also poll RXFF read/write ptrs
     * 0x0288 (RXFF_RD_PTR)/0x028c-ish via 0x0284 dword. */
    for (int i = 0; i < 10; i++) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      uint32_t r284 = _device.rtw_read32(0x0284);
      _logger->info("Jaguar2 MACRX[{}]: 0x284=0x{:08x} (RXPKT_NUM={} "
                    "RXDMA_IDLE={} FULL={})",
                    i, r284, (r284 >> 24) & 0xff, (r284 >> 17) & 1,
                    (r284 >> 16) & 1);
    }
  }
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
