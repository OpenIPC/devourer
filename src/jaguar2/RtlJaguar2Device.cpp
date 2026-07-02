#include "RtlJaguar2Device.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <span>
#include <thread>
#include <stdexcept>
#include <utility>
#include <vector>

#include "FrameParserJaguar2.h"
#include "Halrf8822b.h"
#include "RateDefinitions.h"
#include "RxPacket.h"
#include "SignalStop.h" /* g_devourer_should_stop */
extern "C" {
#include "ieee80211_radiotap.h" /* MRateToHwRate + radiotap iterator */
}

/* M0 scaffold: the orchestrator compiles and constructs, but bring-up is not yet
 * wired. Each entry point logs and (for the data-path calls) throws/returns so a
 * premature use is loud rather than silently wrong. Milestones M2..M7 replace
 * these bodies with the ported HalMAC / phydm / halrf sub-modules. */

RtlJaguar2Device::RtlJaguar2Device(RtlUsbAdapter device, Logger_t logger)
    : _device{device}, _logger{logger}, _hal{device, logger},
      _macinit{device, logger}, _fw{device, logger} {}

RtlJaguar2Device::~RtlJaguar2Device() { stop_dig(); }

void RtlJaguar2Device::bring_up(SelectedChannel channel) {
  /* Cold bring-up shared by Init (RX) and InitWrite (TX). Order mirrors the
   * HalMAC _halmac_init_hal / vendor rtl88x2bu flow: pre-init -> power-on ->
   * chip-version -> init_system_cfg -> firmware DLFW -> post-DLFW MAC cfg +
   * USB RX-DMA + BB/RF enable -> BB/AGC/RF phydm tables -> TRX mode -> channel
   * -> LCK -> IQK -> coex WL grant -> enable RX/TX MAC engine. */
  const uint8_t bw = static_cast<uint8_t>(channel.ChannelWidth);
  _macinit.pre_init_system_cfg();
  _hal.power_on();
  _hal.read_chip_version();
  _macinit.init_system_cfg(channel.ChannelWidth, _hal.chip_version().cut);
  /* Firmware download BEFORE trx/queue config (HalMAC order): running init_trx
   * first over-allocates the FIFOPAGE queues and wedges the DLFW bcn-valid. */
  if (!_fw.download_default_firmware())
    throw std::runtime_error("RtlJaguar2Device: firmware DLFW failed");
  _logger->info("RtlJaguar2Device: firmware booted (bw={})", (int)bw);

  if (!_macinit.init_mac_cfg(channel.ChannelWidth))
    throw std::runtime_error("RtlJaguar2Device: init_mac_cfg failed");
  _macinit.init_usb_cfg();
  _macinit.enable_bb_rf(true);
  _logger->info("RtlJaguar2Device: MAC cfg + BB/RF enabled");

  uint8_t rfe = _hal.read_efuse_rfe();
  _hal.apply_bb_rf_agc_tables(rfe);
  _logger->info("RtlJaguar2Device: PHY tables applied");

  _hal.config_trx_mode(); /* RF mode table + TX/RX antenna-path HW blocks */
  _hal.set_channel_bw(static_cast<uint8_t>(channel.Channel), bw, rfe);
  _hal.do_lck(); /* LC calibration — lock the RF LO */

  if (!getenv("DEVOURER_SKIP_IQK")) {
    jaguar2::Halrf8822b halrf(_device, _logger, _hal.chip_version().cut,
                              _hal.chip_version().rf_2t2r != 0);
    halrf.iqk_trigger(channel.Channel <= 14);
  } else {
    _logger->info("Jaguar2: IQK SKIPPED (DEVOURER_SKIP_IQK)");
  }
  /* Grant the antenna to WLAN (combo chip) — must precede enable. */
  if (!getenv("DEVOURER_SKIP_COEX"))
    _hal.coex_wlan_only();
  else
    _logger->info("Jaguar2: coex WL grant SKIPPED (DEVOURER_SKIP_COEX)");
  _hal.enable_rx(); /* CR MACTX|MACRX + RCR + IGI — enables both TX and RX */
}

void RtlJaguar2Device::Init(Action_ParsedRadioPacket packetProcessor,
                            SelectedChannel channel) {
  _packetProcessor = std::move(packetProcessor);
  _channel = channel;
  bring_up(channel);

  /* RX bring-up register dump (DEVOURER_RX_DEBUG): confirms the enable_rx /
   * channel / RF-mode / coex writes landed. The phydm BB decode counters
   * (0xF04/0xF14/0xF08/0xF48) are hold-type and only advance when the DIG/FA
   * thread pulses their reset, so they are NOT a reliable live-RX signal here —
   * dumped for reference only. */
  if (const char *pf = getenv("DEVOURER_BB_PATCH")) {
    /* Canary bisect: apply "0xADDR 0xVAL" BB-register overrides from a file
     * (kernel golden values) right before RX, to find the decode-critical
     * register(s). */
    FILE *f = fopen(pf, "r");
    if (f) {
      char line[128];
      int n = 0;
      while (fgets(line, sizeof(line), f)) {
        unsigned addr, val;
        if (line[0] == 'A' && sscanf(line + 1, "%x %x", &addr, &val) == 2) {
          _hal.dbg_rf_write(0, addr, val);
          n++;
        } else if (line[0] == 'B' &&
                   sscanf(line + 1, "%x %x", &addr, &val) == 2) {
          _hal.dbg_rf_write(1, addr, val);
          n++;
        } else if (sscanf(line, "%x %x", &addr, &val) == 2) {
          _device.rtw_write32(static_cast<uint16_t>(addr), val);
          n++;
        }
      }
      fclose(f);
      _logger->info("Jaguar2: BB_PATCH applied {} registers from {}", n, pf);
    }
  }
  if (getenv("DEVOURER_BB_DUMP")) {
    /* Full BB/RF register dump in the vendor rtw_proc format for canary diff
     * against the kernel driver (tests/jaguar2_rx_canary.sh). */
    for (uint32_t a = 0x0; a <= 0x7fc; a += 0x10)
      _logger->info("BBDUMP 0x{:04x} 0x{:08x} 0x{:08x} 0x{:08x} 0x{:08x}", a,
                    _device.rtw_read32(a), _device.rtw_read32(a + 4),
                    _device.rtw_read32(a + 8), _device.rtw_read32(a + 12));
    for (uint32_t a = 0x800; a <= 0xffc; a += 0x10)
      _logger->info("BBDUMP 0x{:04x} 0x{:08x} 0x{:08x} 0x{:08x} 0x{:08x}", a,
                    _device.rtw_read32(a), _device.rtw_read32(a + 4),
                    _device.rtw_read32(a + 8), _device.rtw_read32(a + 12));
    for (uint32_t a = 0x1800; a <= 0x1afc; a += 0x10)
      _logger->info("BBDUMP 0x{:04x} 0x{:08x} 0x{:08x} 0x{:08x} 0x{:08x}", a,
                    _device.rtw_read32(a), _device.rtw_read32(a + 4),
                    _device.rtw_read32(a + 8), _device.rtw_read32(a + 12));
    for (uint32_t r = 0x0; r <= 0xff; r++)
      _logger->info("RFDUMP 0x{:02x} A=0x{:05x} B=0x{:05x}", r,
                    _hal.dbg_rf_read(0, r), _hal.dbg_rf_read(1, r));
  }
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
  /* Start the DIG thread: track IGI to the false-alarm rate so weak signals are
   * caught without an FA storm (a fixed IGI can't span the range). */
  _dig_stop = false;
  if (!getenv("DEVOURER_SKIP_DIG")) {
    const bool dig_dbg = getenv("DEVOURER_RX_DEBUG") != nullptr;
    _dig_thread = std::thread([this, dig_dbg] {
      int tick = 0;
      while (!_dig_stop && !g_devourer_should_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        _hal.dig_step();
        if (dig_dbg && (++tick % 10) == 0)
          _logger->info("Jaguar2 DIG: IGI=0x{:02x} FA={}", _hal.dbg_igi(),
                        _hal.dbg_last_fa());
      }
    });
    _logger->info("RtlJaguar2Device: DIG thread started");
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
      ++frames;
      if (getenv("DEVOURER_RX_DEBUG"))
        _logger->info("Jaguar2 RX: frame len={} rate={} crc={}", f.frame_len,
                      f.rx_rate, f.crc_err);
      if (f.next_offset == 0)
        break;
      off += f.next_offset;
    }
  };
  _device.bulk_read_async_loop(32 * 1024, 8, on_data, g_devourer_should_stop);
  stop_dig();
  _logger->info("RtlJaguar2Device: RX loop exited ({} frames, {} reads)", frames,
                reads);
}

void RtlJaguar2Device::stop_dig() {
  _dig_stop = true;
  if (_dig_thread.joinable())
    _dig_thread.join();
}

void RtlJaguar2Device::InitWrite(SelectedChannel channel) {
  _channel = channel;
  /* TX shares the full cold bring-up (config_trx_mode enables the TX antenna
   * paths, enable_rx sets CR MACTXEN). The chip transmits at its
   * efuse/table-calibrated TXAGC; DEVOURER_TX_PWR=0xNN forces a flat reference
   * (SDR-visibility debug knob). */
  bring_up(channel);
  if (const char *e = getenv("DEVOURER_TX_PWR")) {
    uint8_t idx = static_cast<uint8_t>(strtol(e, nullptr, 0) & 0x3f);
    _hal.set_tx_power_flat(idx);
  }
  _logger->info("Jaguar2: ready for TX (monitor inject, ch={})",
                channel.Channel);
}

void RtlJaguar2Device::SetMonitorChannel(SelectedChannel channel) {
  _channel = channel;
}

void RtlJaguar2Device::SetTxPower(uint8_t power) { _tx_pwr_override = power; }

bool RtlJaguar2Device::send_packet(const uint8_t *packet, size_t length) {
  /* Parse the radiotap header for rate/bw/coding, build an 8822B TX descriptor
   * (48 B) + the 802.11 frame, and synchronously bulk-OUT. The TX antenna paths
   * are enabled during bring_up, so frames go on-air at the tuned channel.
   * Ported from RtlJaguar3Device::send_packet (shared halmac 88xx descriptor). */
  if (length < sizeof(struct ieee80211_radiotap_header))
    return false;
  uint16_t radiotap_length = get_unaligned_le16(packet + 2);
  if (radiotap_length == 0 || static_cast<size_t>(radiotap_length) >= length)
    return false;
  const size_t frame_len = length - radiotap_length;

  uint8_t fixed_rate = MGN_1M;
  uint8_t sgi = 0, ldpc = 0, stbc = 0;
  ChannelWidth_t bwidth = CHANNEL_WIDTH_20;
  bool vht = (radiotap_length != 0x0d);
  bool rate_from_radiotap = false;

  auto *rtap_hdr = reinterpret_cast<struct ieee80211_radiotap_header *>(
      const_cast<uint8_t *>(packet));
  struct ieee80211_radiotap_iterator it;
  int ret = ieee80211_radiotap_iterator_init(&it, rtap_hdr, radiotap_length,
                                             nullptr);
  while (!ret) {
    ret = ieee80211_radiotap_iterator_next(&it);
    if (ret)
      continue;
    switch (it.this_arg_index) {
    case IEEE80211_RADIOTAP_RATE:
      fixed_rate = *it.this_arg;
      rate_from_radiotap = true;
      break;
    case IEEE80211_RADIOTAP_MCS: {
      uint8_t mcs_flags = it.this_arg[1];
      if ((mcs_flags & IEEE80211_RADIOTAP_MCS_BW_MASK) ==
          IEEE80211_RADIOTAP_MCS_BW_40)
        bwidth = CHANNEL_WIDTH_40;
      sgi = (mcs_flags & 0x04) ? 1 : 0;
      if (it.this_arg[0] & IEEE80211_RADIOTAP_MCS_HAVE_MCS) {
        uint8_t idx = it.this_arg[2];
        if (idx <= 31) {
          fixed_rate = MGN_MCS0 + idx;
          rate_from_radiotap = true;
        }
      }
    } break;
    case IEEE80211_RADIOTAP_VHT: {
      uint8_t known = it.this_arg[0];
      uint8_t flags = it.this_arg[2];
      if ((known & 4) && (flags & 4))
        sgi = 1;
      if ((known & 1) && (flags & 1))
        stbc = 1;
      if (known & 0x40) {
        auto b = it.this_arg[3] & 0x1f;
        if (b >= 1 && b <= 3)
          bwidth = CHANNEL_WIDTH_40;
        else if (b >= 4 && b <= 10)
          bwidth = CHANNEL_WIDTH_80;
      }
      if (it.this_arg[8] & 1)
        ldpc = 1;
      unsigned mcs = (it.this_arg[4] >> 4) & 0x0f;
      unsigned nss = it.this_arg[4] & 0x0f;
      if (nss > 0) {
        if (nss > 4)
          nss = 4;
        if (mcs > 9)
          mcs = 9;
        fixed_rate = MGN_VHT1SS_MCS0 + ((nss - 1) * 10 + mcs);
        rate_from_radiotap = true;
      }
    } break;
    default:
      break;
    }
  }

  /* Rate-less frame -> apply the SetTxMode default; per-packet radiotap wins. */
  if (!rate_from_radiotap && _tx_mode_default.has_value()) {
    const devourer::TxParams tp = devourer::tx_mode_to_params(*_tx_mode_default);
    fixed_rate = tp.fixed_rate;
    vht = tp.vht;
    sgi = tp.sgi ? 1 : 0;
    ldpc = tp.ldpc ? 1 : 0;
    stbc = tp.stbc ? 1 : 0;
    bwidth = static_cast<ChannelWidth_t>(tp.bwidth);
  }

  uint8_t bw_desc = (bwidth == CHANNEL_WIDTH_40)   ? 1
                    : (bwidth == CHANNEL_WIDTH_80) ? 2
                                                   : 0;
  uint8_t rate_id = vht ? 9 : 8;
  const uint8_t *dot11 = packet + radiotap_length;
  bool bmc = frame_len >= 6 && (dot11[4] & 0x01);

  std::vector<uint8_t> usb_frame(jaguar2::TXDESC_SIZE_8822B + frame_len, 0);
  jaguar2::fill_data_tx_desc_8822b(
      usb_frame.data(), static_cast<uint16_t>(frame_len),
      MRateToHwRate(fixed_rate), rate_id, bw_desc, sgi != 0, ldpc != 0, stbc,
      bmc);
  std::memcpy(usb_frame.data() + jaguar2::TXDESC_SIZE_8822B, dot11, frame_len);

  uint8_t tx_ep = _device.first_bulk_out_ep();
  int rc = _device.bulk_send_sync_ep(tx_ep, usb_frame.data(), usb_frame.size(),
                                     /*timeout_ms=*/20);
  return rc >= 0;
}

SelectedChannel RtlJaguar2Device::GetSelectedChannel() { return _channel; }

void RtlJaguar2Device::Stop() { stop_dig(); }

void RtlJaguar2Device::SetTxMode(const devourer::TxMode &mode) {
  _tx_mode_default = mode;
}

void RtlJaguar2Device::ClearTxMode() { _tx_mode_default.reset(); }
