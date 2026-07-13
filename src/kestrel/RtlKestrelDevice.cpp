#include "RtlKestrelDevice.h"

#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

#include "FrameParserKestrel.h"
#include "RadiotapPeek.h"
#include "RxPacket.h"
#include "TxDescKestrel.h"

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
  /* Vendor order: efuse is processed AFTER the MAC hal init
   * (rtl8852b_halinit.c:556) — FWDL first, then the efuse dump. */
  if (!_hal.power_on())
    return false;
  if (!_hal.download_firmware(_hal.read_cut()))
    return false;
  return _hal.read_efuse(out);
}

bool RtlKestrelDevice::PowerOnFwAndTrx(kestrel::EfuseInfo &out) {
  /* mac_hal_init order (init.c:336): pwr -> [pre-init+FWDL inside
   * download_firmware] -> set_enable_bb_rf -> sys_init -> trx_init ->
   * feat_init (host-side no-op) -> intf_init (usb) -> [efuse after]. */
  if (!_hal.power_on())
    return false;
  if (!_hal.download_firmware(_hal.read_cut()))
    return false;
  _hal.enable_bb_rf();
  _hal.mac_sys_init();
  if (!_hal.trx_dmac_init())
    return false;
  if (!_hal.trx_cmac_rx_init())
    return false;
  _hal.usb_intf_init();
  _hal.fw_err_state("post-mac-hal-init");
  return _hal.read_efuse(out);
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
  /* hal_start_8852b order (rtl8852b_halinit.c:508): mac_hal_init [pwr ->
   * FWDL -> enable_bb_rf -> sys_init -> trx_init -> intf_init] -> efuse ->
   * BB/RF tables -> channel. */
  if (!PowerOnFwAndTrx(_efuse))
    return false;
  if (!_hal.phy_bb_rf_init(_efuse.rfe_type, _hal.read_cut()))
    return false;
  /* Vendor timing: arm the SER error IMR only after the BB/RF tables. */
  _hal.enable_ser_imr();
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
  /* TX bring-up = the full monitor bring-up (the CMAC init already stands up
   * the scheduler/tmac/trxptcl/ptcl TX path) minus the RX loop. Then resolve
   * the band-0 mgmt bulk-OUT endpoint (BULKOUTID0 = 0th bulk-OUT per
   * get_bulkout_id_8852b). */
  if (!BringUpMonitor(channel)) {
    _logger->error("Kestrel: TX bring-up failed");
    return;
  }
  _tx_mgmt_ep = _device.nth_bulk_out_ep(0);
  if (_tx_mgmt_ep == 0) {
    _logger->error("Kestrel: no bulk-OUT endpoint for mgmt TX");
    return;
  }
  _logger->info("Kestrel: TX ready on ch{} — mgmt endpoint 0x{:02x}",
                channel.Channel, _tx_mgmt_ep);
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

bool RtlKestrelDevice::send_packet(const uint8_t *packet, size_t length) {
  if (_tx_mgmt_ep == 0) {
    _logger->error("Kestrel: send_packet before InitWrite (TX not up)");
    return false;
  }
  /* devourer convention: packet = [radiotap header][802.11 MPDU]. Strip the
   * radiotap; the AX descriptor carries the TX parameters instead. */
  const uint16_t rlen = devourer::radiotap_hdr_len(packet, length);
  if (rlen == 0 || static_cast<size_t>(rlen) >= length) {
    _logger->error("Kestrel: send_packet bad radiotap (len={}, rtap={})",
                   length, rlen);
    return false;
  }
  const uint8_t *frame = packet + rlen;
  const uint32_t flen = static_cast<uint32_t>(length - rlen);

  /* First-light TX: fixed 6M OFDM, MACID 0 (self), no rate fallback. Radiotap-
   * driven rate/BW selection lands in M5 with the HE rate grammar. */
  auto buf = kestrel::build_mgnt_txdesc(frame, flen, kestrel::RATE_OFDM6, 0,
                                        _tx_seq++ & 0xfff);
  int rc = _device.bulk_send_sync_ep(_tx_mgmt_ep, buf.data(),
                                     static_cast<int>(buf.size()), 1000);
  if (rc < 0 || static_cast<size_t>(rc) != buf.size()) {
    _logger->error("Kestrel: send_packet bulk-OUT ep 0x{:02x} failed (rc={}, "
                   "wanted {})",
                   _tx_mgmt_ep, rc, buf.size());
    return false;
  }
  return true;
}
