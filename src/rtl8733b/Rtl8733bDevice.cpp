#include "Rtl8733bDevice.h"

#include <algorithm>
#include <cstring>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "RateDefinitions.h"
#include "RadiotapPeek.h"
#include "RadiotapTxFlags.h"
#include "RxParseAbort.h" /* rx.parse_abort — abandoned-aggregate event */
#include "rtl8733b/Rtl8733bUsbIds.h"
#include "rtl8733b/TxDescriptor8733b.h"
#include "SignalStop.h"

extern "C" {
#include "ieee80211_radiotap.h"
}

Rtl8733bDevice::Rtl8733bDevice(RtlAdapter device, Logger_t logger,
                               devourer::DeviceConfig cfg)
    : _device(device), _logger(std::move(logger)), _cfg(std::move(cfg)),
      _bringup(device, _logger), _mac(device, _logger),
      _phy(device, _logger) {}

Rtl8733bDevice::~Rtl8733bDevice() {
  try {
    Stop();
  } catch (...) {
    // Destruction must not terminate the process after a USB unplug/error.
  }
}

void Rtl8733bDevice::bring_up_to_phy() {
  if (_phy_ready)
    return;
  if (!_firmware_ready) {
    _chip = _bringup.read_chip_info();
    if (!_chip.matches())
      throw std::runtime_error(
          "RTL8733B identity changed during initialization");
    if (!_chip.efuse_autoload_ok)
      throw std::runtime_error("RTL8733B EFUSE autoload failed");
    if (!_bringup.power_on())
      throw std::runtime_error("RTL8733B card-enable sequence failed");
    _power_ready = true;
    if (!_mac.read_efuse(_efuse))
      throw std::runtime_error("RTL8733B EFUSE read/parse failed");
    _fw_boot = {.supported = true, .attempted = true};
    const bool firmware_ok = _bringup.download_default_firmware(_chip.cut);
    _fw_boot.attempted = _bringup.download_attempted();
    _fw_boot.checksum_ok = _bringup.checksum_ok();
    _fw_boot.ready_ok = _bringup.ready_ok();
    if (!firmware_ok)
      throw std::runtime_error("RTL8733B firmware download/boot failed");
    if (!_mac.initialize(_efuse))
      throw std::runtime_error("RTL8733B HALMAC/MAC initialization failed");
    _mac_ready = true;
    _firmware_ready = true;
  }
  if (!_phy.initialize(_chip.cut, _efuse))
    throw std::runtime_error("RTL8733B PHY/RF initialization failed");
  _phy_ready = true;
  _logger->info("RTL8733B factory path reached PHY-ready (cut {})",
                _chip.cut);
}

[[noreturn]] void
Rtl8733bDevice::radio_operation_unavailable(const char *operation) {
  const std::string message =
      std::string("RTL8733B ") + operation +
      " is not implemented by this backend";
  /* Several existing demos intentionally let initialization errors escape
   * main. That reaches std::terminate without a guaranteed device-destructor
   * unwind, so staged refusals must leave the hardware safe before throwing. */
  Stop();
  throw std::runtime_error(message);
}

void Rtl8733bDevice::Init(Action_ParsedRadioPacket packetProcessor,
                          SelectedChannel channel) {
  try {
    {
      std::lock_guard<std::recursive_mutex> lock(_reg_mu);
      bring_up_to_phy();
      if (!_phy.set_channel(channel))
        throw std::runtime_error("RTL8733B channel configuration failed");
      _channel = channel;
      _rx_configured_bw = channel.ChannelWidth == CHANNEL_WIDTH_40 ? 1 : 0;
      if (!_mac.configure_monitor_rx(_cfg.rx.keep_corrupted))
        throw std::runtime_error("RTL8733B monitor RX configuration failed");
    }
    StartRxLoop(std::move(packetProcessor));
  } catch (...) {
    Stop();
    throw;
  }
}

void Rtl8733bDevice::InitWrite(SelectedChannel channel) {
  try {
    std::lock_guard<std::recursive_mutex> lock(_reg_mu);
    bring_up_to_phy();
    if (!_phy.set_channel(channel))
      throw std::runtime_error("RTL8733B channel configuration failed");
    _channel = channel;
    _rx_configured_bw = channel.ChannelWidth == CHANNEL_WIDTH_40 ? 1 : 0;
    if (!configure_tx_power(channel))
      throw std::runtime_error("RTL8733B safe TX power setup failed");
    if (_cfg.rx.enable_with_tx &&
        !_mac.configure_monitor_rx(_cfg.rx.keep_corrupted))
      throw std::runtime_error("RTL8733B monitor RX configuration failed");
    const auto thermal = GetThermalStatus();
    if (thermal.valid && thermal.delta >= 25)
      throw std::runtime_error("RTL8733B thermal gate refused TX");
    _tx_ready = true;
    _tx_submits = 0;
    _logger->info(
        "RTL8733B ready for bounded monitor injection: ch={} width={} "
        "thermal={}/{}",
        channel.Channel, static_cast<unsigned>(channel.ChannelWidth),
        thermal.raw, thermal.baseline);
  } catch (...) {
    Stop();
    throw;
  }
}

bool Rtl8733bDevice::configure_tx_power(SelectedChannel channel) {
  _tssi_tracking = false;
  _tssi_cck = false;
  if (_efuse.tx_power_mode != rtl8733b::TxPowerPgMode8733b::TssiOffset)
    return _phy.set_flat_tx_power(rtl8733b::kSafeTxAgcIndex8733b);

  /* Start in the OFDM/HT thermal table. A first CCK submission switches the
   * table through select_tssi_rate_table() before its descriptor reaches USB. */
  if (!_phy.prepare_tssi_bb(channel, _efuse) ||
      !_phy.prepare_tssi_thermal(_efuse, false) ||
      !_phy.prepare_tssi_offsets(channel, _efuse) ||
      !_phy.enable_tssi_tracking(
          channel, _efuse, rtl8733b::kSafeTssiTargetQdbm8733b))
    return false;
  _tssi_tracking = true;
  return true;
}

bool Rtl8733bDevice::select_tssi_rate_table(bool cck) {
  if (_efuse.tx_power_mode != rtl8733b::TxPowerPgMode8733b::TssiOffset)
    return true;
  if (!_tssi_tracking)
    return false;
  if (_tssi_cck == cck)
    return true;

  /* The vendor chooses this table from the current TX rate. The table cannot
   * be changed while closed-loop tracking is enabled, so make the transition
   * explicit and reversible: exact rollback, table readback, then a fresh
   * capped enable with its own thermal check. */
  if (!_phy.disable_tssi_tracking())
    return false;
  _tssi_tracking = false;
  if (!_phy.prepare_tssi_thermal(_efuse, cck) ||
      !_phy.enable_tssi_tracking(
          _channel, _efuse, rtl8733b::kSafeTssiTargetQdbm8733b))
    return false;
  _tssi_tracking = true;
  _tssi_cck = cck;
  _logger->info("RTL8733B TSSI rate table selected: {}",
                cck ? "CCK" : "OFDM/HT");
  return true;
}

void Rtl8733bDevice::StartRxLoop(
    Action_ParsedRadioPacket packetProcessor) {
  Action_ParsedRadioPacket rx_packet_processor = std::move(packetProcessor);
  uint8_t rx_channel = 0;
  {
    std::lock_guard<std::recursive_mutex> lock(_reg_mu);
    if (!_phy_ready || !_mac_ready)
      throw std::runtime_error("RTL8733B RX loop requires initialized hardware");
    if (_rx_active)
      throw std::runtime_error("RTL8733B RX loop is already active");
    _rx_stop = false;
    _rx_active = true;
    rx_channel = _channel.Channel;
  }
  uint64_t completions = 0;
  uint64_t frames = 0;
  uint64_t malformed = 0;
  uint64_t aggregate_mismatch = 0;
  long long parse_aborts = 0;

  auto on_data = [&](const uint8_t *data, int length) {
    if (++completions <= 8)
      _logger->info("RTL8733B RX: completion #{} -> {} bytes", completions,
                    length);
    size_t offset = 0;
    uint8_t advertised = 0;
    uint32_t packets = 0;
    while (offset + rtl8733b::kRxDescSize <= static_cast<size_t>(length)) {
      rtl8733b::RxFrame8733b frame{};
      if (!rtl8733b::parse_rx_8733b(data + offset,
                                    static_cast<size_t>(length) - offset,
                                    frame)) {
        const size_t remaining = static_cast<size_t>(length) - offset;
        if (!devourer::rx_parse_remainder_is_zero_padding(data + offset,
                                                          remaining)) {
          ++malformed;
          devourer::emit_rx_parse_abort(
              _logger->events(), data + offset, remaining,
              static_cast<long long>(offset), length, frame.frame_len,
              frame.drvinfo_size, frame.shift, parse_aborts);
        }
        break;
      }
      if (packets == 0)
        advertised = frame.dma_aggregate_count;
      ++packets;

      Packet packet{};
      packet.RxAtrib.pkt_len = static_cast<uint16_t>(frame.frame_len);
      packet.RxAtrib.physt = frame.physt;
      packet.RxAtrib.drvinfo_sz = static_cast<uint8_t>(frame.drvinfo_size);
      packet.RxAtrib.shift_sz = frame.shift;
      packet.RxAtrib.qos = frame.qos;
      packet.RxAtrib.priority = frame.tid;
      packet.RxAtrib.mdata = frame.more_data;
      packet.RxAtrib.mfrag = frame.more_fragment;
      packet.RxAtrib.seq_num = frame.sequence;
      packet.RxAtrib.frag_num = frame.fragment;
      packet.RxAtrib.bdecrypted = frame.sw_decrypted;
      packet.RxAtrib.encrypt = frame.security;
      packet.RxAtrib.crc_err = frame.crc_err;
      packet.RxAtrib.icv_err = frame.icv_err;
      packet.RxAtrib.tsfl = frame.tsfl;
      packet.RxAtrib.data_rate = frame.rx_rate;
      packet.RxAtrib.paggr = frame.paggr;
      packet.RxAtrib.ppdu_cnt = frame.ppdu_count;
      packet.RxAtrib.pkt_rpt_type = frame.c2h
                                       ? RX_PACKET_TYPE::C2H_PACKET
                                       : RX_PACKET_TYPE::NORMAL_RX;
      bool phy_parsed = false;
      if (!frame.c2h && frame.physt && frame.drvinfo_size >= 24)
        phy_parsed = rtl8733b::parse_phy_status_8733b(
            data + offset + rtl8733b::kRxDescSize, frame.drvinfo_size,
            frame.rx_rate, _rx_configured_bw.load(),
            packet.RxAtrib);
      packet.Data = std::span<uint8_t>(const_cast<uint8_t *>(frame.frame),
                                       frame.frame_len);
      if (rx_packet_processor)
        rx_packet_processor(packet);
      if (++frames <= 8) {
        const uint8_t *status = data + offset + rtl8733b::kRxDescSize;
        _logger->info(
            "RTL8733B RX: frame len={} rate={} crc={} c2h={} physt={} drv={} "
            "page={} phy_ok={} rssi={}",
            frame.frame_len, frame.rx_rate, frame.crc_err, frame.c2h,
            frame.physt, frame.drvinfo_size,
            frame.drvinfo_size != 0 ? status[0] & 0x0f : 0xff, phy_parsed,
            packet.RxAtrib.rssi[0]);
      }

      if (frame.next_offset == 0)
        break;
      offset += frame.next_offset;
      if (advertised != 0 && packets >= advertised)
        break;
    }
    if (advertised != 0 && packets != advertised)
      ++aggregate_mismatch;
  };

  int urb_bytes = _cfg.rx.urb_bytes.value_or(16 * 1024);
  if (urb_bytes < 4096)
    urb_bytes = 4096;
  _logger->info("RTL8733B RX: entering async loop ch={} urb={}x8",
                rx_channel, urb_bytes);
  try {
    _device.bulk_read_async_loop(urb_bytes, 8, on_data, [this]() -> bool {
      return _rx_stop || g_devourer_should_stop;
    });
  } catch (...) {
    _rx_active = false;
    throw;
  }
  _rx_active = false;
  _logger->info(
      "RTL8733B RX: stopped completions={} frames={} malformed={} agg_mismatch={}",
      completions, frames, malformed, aggregate_mismatch);
}

void Rtl8733bDevice::SetMonitorChannel(SelectedChannel channel) {
  std::lock_guard<std::recursive_mutex> lock(_reg_mu);
  bring_up_to_phy();
  const bool was_tx_ready = _tx_ready;
  _tx_ready = false;
  if (_tssi_tracking) {
    if (!_phy.disable_tssi_tracking())
      throw std::runtime_error("RTL8733B TSSI rollback before retune failed");
    _tssi_tracking = false;
  }
  if (!_phy.set_channel(channel))
    throw std::runtime_error("RTL8733B channel configuration failed");
  _channel = channel;
  _rx_configured_bw = channel.ChannelWidth == CHANNEL_WIDTH_40 ? 1 : 0;
  if (was_tx_ready) {
    if (!configure_tx_power(channel)) {
      _tx_ready = false;
      throw std::runtime_error("RTL8733B safe TX power reapply failed");
    }
    _tx_ready = true;
  }
}

bool Rtl8733bDevice::send_packet(const uint8_t *packet, size_t length) {
  std::lock_guard<std::recursive_mutex> lock(_reg_mu);
  if (!_phy_ready || !_mac_ready || !_tx_ready) {
    _logger->error("RTL8733B TX rejected before InitWrite");
    return false;
  }
  if (packet == nullptr)
    return false;
  const auto thermal = GetThermalStatus();
  if (thermal.valid && thermal.delta >= 25) {
    _logger->error("RTL8733B TX thermal gate: raw={} baseline={} delta={}",
                   thermal.raw, thermal.baseline, thermal.delta);
    return false;
  }
  const uint16_t radiotap_length =
      devourer::radiotap_hdr_len(packet, length);
  if (radiotap_length == 0)
    return false;
  const size_t frame_len = length - radiotap_length;
  const size_t bulk_packet = _device.speed() >= devourer::kUsbSpeedSuper
                                 ? 1024u
                                 : _device.speed() >= devourer::kUsbSpeedHigh
                                       ? 512u
                                       : 64u;
  const uint8_t packet_offset =
      (rtl8733b::kTxDescSize + frame_len) % bulk_packet == 0 ? 1 : 0;
  std::vector<uint8_t> usb_frame(rtl8733b::kTxDescSize +
                                     static_cast<size_t>(packet_offset) * 8u +
                                     frame_len,
                                 0);
  const size_t built = build_tx_block(packet, length, usb_frame.data(),
                                      packet_offset);
  if (built == 0 || built != usb_frame.size())
    return false;

  /* QSEL_MGNT maps to HALMAC's HIGH DMA mapping in the normal 3-bulk-OUT
   * profile, hence bulk-out ID 0 / the first enumerated endpoint (0x05 on the
   * tested f72b). Use synchronous TX so every submission has an
   * immediate, bounded result and no buffer outlives this call. */
  const uint8_t endpoint = _device.first_bulk_out_ep();
  const uint8_t rate_hw = usb_frame[0x10] & 0x7f;
  const uint8_t rate_id = usb_frame[0x06] & 0x1f;
  const uint8_t bandwidth = (usb_frame[0x14] >> 5) & 0x3;
  const uint8_t short_gi = (usb_frame[0x14] >> 4) & 0x1;
  const uint8_t ldpc = (usb_frame[0x14] >> 7) & 0x1;
  if (!select_tssi_rate_table(rate_hw <= 3)) {
    _logger->error("RTL8733B TX rejected: TSSI rate-table transition failed");
    Stop();
    return false;
  }
  const int sent = _device.bulk_send_sync_ep(
      endpoint, usb_frame.data(), usb_frame.size(), 100);
  if (sent != static_cast<int>(usb_frame.size())) {
    _logger->error("RTL8733B TX failed/short on EP 0x{:02x}: {}/{}", endpoint,
                   sent, usb_frame.size());
    return false;
  }
  if (_tx_submits.fetch_add(1) == 0) {
    _logger->info(
        "RTL8733B first TX accepted: EP=0x{:02x} bytes={} frame={} shim={} "
        "rate_hw={} rate_id={} bw={} sgi={} ldpc={} fifo={} empty=0x{:04x} "
        "txdma=0x{:08x} pause=0x{:02x} bb_tx_en={} bb_tx_on={}",
        endpoint, sent, frame_len, packet_offset, rate_hw, rate_id, bandwidth,
        short_gi, ldpc,
        _device.rtw_read8(0x0207), _device.rtw_read16(0x041a),
        _device.rtw_read32(0x0210), _device.rtw_read8(0x0522),
        _device.rtw_read16(0x2de0), _device.rtw_read16(0x2de2));
  }
  return true;
}

void Rtl8733bDevice::SetTxMode(const devourer::TxMode &mode) {
  std::lock_guard<std::recursive_mutex> lock(_reg_mu);
  _tx_mode_default = mode;
}

void Rtl8733bDevice::ClearTxMode() {
  std::lock_guard<std::recursive_mutex> lock(_reg_mu);
  _tx_mode_default.reset();
}

SelectedChannel Rtl8733bDevice::GetSelectedChannel() {
  std::lock_guard<std::recursive_mutex> lock(_reg_mu);
  return _channel;
}

size_t Rtl8733bDevice::build_tx_block(const uint8_t *packet, size_t length,
                                      uint8_t *out,
                                      uint8_t packet_offset) {
  if (packet == nullptr || out == nullptr)
    return 0;
  const uint16_t radiotap_length =
      devourer::radiotap_hdr_len(packet, length);
  if (radiotap_length == 0)
    return 0;
  const size_t frame_len = length - radiotap_length;
  /* The monitor-injection path deliberately requires a normal 24-byte
   * 802.11 header and the hardware's 12-bit MPDU-length range. */
  if (frame_len < 24 || frame_len > 4095 || packet_offset > 1)
    return 0;

  uint8_t fixed_rate = MGN_6M;
  ChannelWidth_t frame_width = CHANNEL_WIDTH_20;
  int requested_channel = 0;

  /* A per-packet radiotap rate below remains authoritative. Rate-less frames
   * use the same SetTxMode default as the established backends and txdemo. */
  if (_tx_mode_default) {
    const auto &mode = *_tx_mode_default;
    if (mode.mode == devourer::TxMode::Mode::Legacy) {
      if (mode.bw_mhz != 20 || mode.sgi || mode.ldpc || mode.stbc)
        return 0;
      fixed_rate = mode.legacy_rate_500kbps;
    } else if (mode.mode == devourer::TxMode::Mode::HT) {
      if (mode.ht_mcs > 7 || (mode.bw_mhz != 20 && mode.bw_mhz != 40) ||
          mode.sgi || mode.ldpc || mode.stbc)
        return 0;
      fixed_rate = static_cast<uint8_t>(MGN_MCS0 + mode.ht_mcs);
      frame_width = mode.bw_mhz == 40 ? CHANNEL_WIDTH_40 : CHANNEL_WIDTH_20;
    } else {
      return 0;
    }
  }

  auto *header = reinterpret_cast<struct ieee80211_radiotap_header *>(
      const_cast<uint8_t *>(packet));
  struct ieee80211_radiotap_iterator iterator;
  if (ieee80211_radiotap_iterator_init(&iterator, header, radiotap_length,
                                       nullptr) != 0)
    return 0;
  while (ieee80211_radiotap_iterator_next(&iterator) == 0) {
    switch (iterator.this_arg_index) {
    case IEEE80211_RADIOTAP_RATE:
      fixed_rate = *iterator.this_arg;
      frame_width = CHANNEL_WIDTH_20;
      break;
    case IEEE80211_RADIOTAP_CHANNEL:
      requested_channel =
          devourer::freq_to_chan(get_unaligned_le16(iterator.this_arg));
      if (requested_channel <= 0)
        return 0;
      break;
    case IEEE80211_RADIOTAP_MCS: {
      const auto m = devourer::decode_radiotap_mcs_field(iterator.this_arg);
      if (!m.have_mcs || m.mcs > 7 || m.sgi != 0 || m.ldpc != 0 ||
          m.stbc != 0)
        return 0;
      fixed_rate = static_cast<uint8_t>(MGN_MCS0 + m.mcs);
      frame_width = m.bw40 ? CHANNEL_WIDTH_40 : CHANNEL_WIDTH_20;
      break;
    }
    case IEEE80211_RADIOTAP_VHT:
    case IEEE80211_RADIOTAP_HE:
      return 0; // RTL8733B contract is legacy CCK/OFDM + HT 1SS only
    default:
      break;
    }
  }

  /* This initial path does not retune during a submission. Refuse a mismatched
   * radiotap CHANNEL instead of silently airing on the session channel. */
  if (requested_channel > 0 && requested_channel != _channel.Channel)
    return 0;

  if (frame_width == CHANNEL_WIDTH_40 &&
      _channel.ChannelWidth != CHANNEL_WIDTH_40)
    return 0;
  if (_channel.ChannelWidth != CHANNEL_WIDTH_20 &&
      _channel.ChannelWidth != CHANNEL_WIDTH_40 &&
      _channel.ChannelWidth != CHANNEL_WIDTH_5 &&
      _channel.ChannelWidth != CHANNEL_WIDTH_10)
    return 0;

  uint8_t data_sc = 0;
  if (_channel.ChannelWidth == CHANNEL_WIDTH_40 &&
      frame_width == CHANNEL_WIDTH_20) {
    data_sc = _channel.ChannelOffset == 2
                  ? 1 // primary is upper 20 MHz
                  : _channel.ChannelOffset == 1 ? 2 : 0;
  }

  const uint8_t *dot11 = packet + radiotap_length;
  const uint16_t sequence = static_cast<uint16_t>(
      (static_cast<uint16_t>(dot11[22]) |
       (static_cast<uint16_t>(dot11[23]) << 8)) >>
      4);
  const bool bmc = (dot11[4] & 1u) != 0;
  rtl8733b::TxDescConfig cfg;
  cfg.packet_size = static_cast<uint16_t>(frame_len);
  cfg.sequence = sequence;
  cfg.rate_hw = MRateToHwRate(fixed_rate);
  cfg.bandwidth = frame_width == CHANNEL_WIDTH_40 ? 1 : 0;
  cfg.rate_id = rtl8733b::tx_rate_id_8733b(
      cfg.rate_hw, cfg.bandwidth, _channel.Channel > 14);
  cfg.data_sc = data_sc;
  cfg.packet_offset = packet_offset;
  cfg.retry_limit = static_cast<uint8_t>(
      std::clamp(_cfg.tx.retry_limit, 0, 63));
  cfg.short_gi = false;
  cfg.ldpc = false;
  cfg.bmc = bmc;
  if (!rtl8733b::fill_tx_desc_8733b(out, rtl8733b::kTxDescSize, cfg)) {
    _logger->error("RTL8733B TX rejected: unsupported rate/width/coding");
    return 0;
  }

  const size_t frame_offset =
      rtl8733b::kTxDescSize + static_cast<size_t>(packet_offset) * 8u;
  std::memcpy(out + frame_offset, dot11, frame_len);
  return frame_offset + frame_len;
}

void Rtl8733bDevice::SetCcaMode(bool disabled) {
  (void)disabled;
  radio_operation_unavailable("CCA control");
}

void Rtl8733bDevice::Stop() {
  std::lock_guard<std::recursive_mutex> lock(_reg_mu);
  _rx_stop = true;
  _device.quiesce_tx();
  _tx_ready = false;
  if (_tssi_tracking) {
    try {
      if (!_phy.disable_tssi_tracking())
        _logger->warn("RTL8733B TSSI rollback did not match before shutdown");
    } catch (const std::exception &e) {
      /* A disconnect can make rollback reads fail. Continue into MAC/card
       * shutdown: leaving the teardown half-run is less safe than losing the
       * readback assertion after the USB device has gone away. */
      _logger->warn("RTL8733B TSSI rollback failed during shutdown: {}",
                    e.what());
    }
    _tssi_tracking = false;
  }
  _tssi_cck = false;
  _phy_ready = false;
  if (_mac_ready) {
    _mac.stop();
    _mac_ready = false;
  }
  _firmware_ready = false;
  if (_power_ready && _cfg.tuning.teardown_power_down) {
    if (!_bringup.power_off())
      _logger->warn("RTL8733B card-disable sequence did not reach off state");
    _power_ready = false;
  }
}

devourer::TxCaps Rtl8733bDevice::GetTxCaps() {
  /* Advertised surface: one spatial stream, 2.4-GHz long-preamble CCK, legacy
   * OFDM, and HT MCS0-7 at 20/40 MHz. TxCaps has no separate CCK flag. Keep SGI
   * false: an independent receiver decoded a descriptor-SGI probe as long GI.
   * The vendor advertises TX-LDPC (but not RX-LDPC); this raw-injection backend
   * keeps it false because only forced-global-BCC operation has passed
   * independently witnessed injection. Experimental 5/10 MHz is likewise
   * omitted until an occupied-bandwidth and cross-receiver gate passes. */
  return devourer::tx_caps_for_chains(1, false, false, 40);
}

devourer::AdapterCaps Rtl8733bDevice::GetAdapterCaps() {
  devourer::AdapterCaps caps;
  caps.supported = true;
  caps.chip_name = "RTL8733B";
  caps.marketing_names = "RTL8731BU/RTL8733BU";
  caps.chip_id = rtl8733b::kChipId;
  caps.generation = devourer::ChipGeneration::Rtl8733b;
  caps.variant = "cut-selected";
  caps.transport = _device.is_usb() ? "usb" : "unknown";
  caps.tx_chains = 1;
  caps.rx_chains = 1;
  caps.tx = GetTxCaps();
  caps.bw_mask = devourer::bw_mask_for_generation(caps.generation);
  caps.tune_2g4 = {true, 2412, 2484};
  caps.characterized_2g4 = caps.tune_2g4;
  /* The production channel planner intentionally accepts only standard
   * 5-GHz channels 36..177, not the wider Jaguar extended-frequency range. */
  caps.tune_5g = {true, 5180, 5885};
  caps.characterized_5g = {true, 5180, 5825};
  caps.hw_rx_timestamp = true;
  return caps;
}

devourer::ThermalStatus Rtl8733bDevice::GetThermalStatus() {
  std::lock_guard<std::recursive_mutex> lock(_reg_mu);
  devourer::ThermalStatus status;
  if (!_phy_ready)
    return status;
  status.raw = _phy.read_thermal();
  status.baseline = _efuse.thermal;
  status.valid = status.raw != 0 && status.baseline != 0xff;
  status.delta = status.valid ? static_cast<int>(status.raw) -
                                  static_cast<int>(status.baseline)
                              : 0;
  return status;
}

bool Rtl8733bDevice::GetPermanentMacAddress(uint8_t out[6]) {
  std::lock_guard<std::recursive_mutex> lock(_reg_mu);
  if (out == nullptr || !_efuse.valid || !_efuse.mac_valid())
    return false;
  std::copy(_efuse.mac.begin(), _efuse.mac.end(), out);
  return true;
}

devourer::EfuseStability Rtl8733bDevice::ProbeEfuseStability(int reads) {
  std::lock_guard<std::recursive_mutex> lock(_reg_mu);
  if (!_mac_ready)
    return {};
  auto status = devourer::ProbeEfuseStabilityImpl(
      [this](uint8_t *map) {
        rtl8733b::EfuseInfo fresh;
        if (!_mac.read_efuse(fresh))
          return false;
        std::copy(fresh.logical.begin(), fresh.logical.end(), map);
        return true;
      },
      static_cast<uint16_t>(rtl8733b::kLogicalEfuseSize), reads);
  _logger->info(
      "RTL8733B EFUSE stability: reads={} mismatched={} invalid_id={} "
      "id=0x{:04x}",
      status.reads, status.mismatched_reads, status.invalid_id_reads,
      status.eeprom_id);
  return status;
}

devourer::FwBootStatus Rtl8733bDevice::GetFwBootStatus() {
  std::lock_guard<std::recursive_mutex> lock(_reg_mu);
  return _fw_boot;
}
