#include "Rtl8733bDevice.h"

#include <algorithm>
#include <cstring>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

#include "RateDefinitions.h"
#include "RadiotapPeek.h" /* send_packets batch pre-parse */
#include "RadiotapTxFlags.h"
#include "RxParseAbort.h" /* rx.parse_abort — abandoned-aggregate event */
#include "rtl8733b/Rtl8733bUsbIds.h"
#include "rtl8733b/TxDescriptor8733b.h"
#include "SignalStop.h"
#include "TxAggPlan.h" /* shared USB TX aggregation layout planner */

extern "C" {
#include "ieee80211_radiotap.h"
}

Rtl8733bDevice::Rtl8733bDevice(RtlAdapter device, Logger_t logger,
                               devourer::DeviceConfig cfg)
    : _device(device), _logger(std::move(logger)), _cfg(std::move(cfg)),
      _bringup(device, _logger), _mac(device, _logger),
      _phy(device, _logger) {
  _phy.set_narrowband_overrides(_cfg.tuning.nb_dac, _cfg.tuning.nb_adc);
}

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
    /* Arm the teardown BEFORE the attempt, not after it. power_on() walks the
     * card-disable -> card-emulation -> active sequence write by write and can
     * fail at any poll with several enables already landed; latching
     * _power_ready only on success would make Stop() skip power_off() in
     * exactly the case that left the card half-enabled, and the next attempt
     * would depend on the sequence being re-enterable from an undefined state.
     * power_off() is the full HALMAC card-disable flow and is safe to run
     * against a partially enabled card. */
    _power_ready = true;
    if (!_bringup.power_on())
      throw std::runtime_error("RTL8733B card-enable sequence failed");
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
  /* Every other generation applies tuning.disable_cca during bring-up
   * (jaguar1/2/3, kestrel all call SetCcaMode there), so a caller setting
   * DEVOURER_DIS_CCA=1 reasonably expects it to take effect. This backend has
   * not located and measured the HALMAC 87xx carrier-sense gate, so it cannot
   * honour the request — say so once, loudly, rather than dropping it in
   * silence, which is the one way a refused knob can look like a granted one:
   * the operator would otherwise believe carrier-sense was off and read the
   * resulting deferral as a transmitter problem.
   *
   * Deliberately a warning and not the throw SetCcaMode(true) raises: the knob
   * is on by default for the streamtx FPV downlink, and failing bring-up
   * outright is a harsh answer to a request the caller may be making only via
   * an inherited environment. The session runs with standard carrier-sense.
   *
   * Sited in bring_up_to_phy rather than InitWrite so an RX-only session that
   * set the knob is told too, and so it fires exactly once per bring-up. */
  if (_cfg.tuning.disable_cca)
    _logger->warn(
        "RTL8733B: DEVOURER_DIS_CCA / tuning.disable_cca is not implemented by "
        "this backend — carrier-sense stays ENABLED for this session");
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
    _tx_ready = true;
    _tx_submits = 0;
    /* One-shot thermal snapshot at bring-up — the PA-heating baseline for the
     * session, same as the Kestrel InitWrite snapshot. Logged, never acted on:
     * the meter is a PA-bias tracking index, not a calibrated °C sensor, and
     * docs/warm-tx-degradation.md shows it is not a validated degradation
     * predictor. Live monitoring is the caller's job via GetThermalStatus /
     * DEVOURER_THERMAL_POLL_MS. */
    const auto thermal = GetThermalStatus();
    _logger->info(
        "RTL8733B ready for bounded monitor injection: ch={} width={} "
        "thermal={}/{} delta={}",
        channel.Channel, static_cast<unsigned>(channel.ChannelWidth),
        thermal.raw, thermal.baseline, thermal.valid ? thermal.delta : 0);
  } catch (...) {
    Stop();
    throw;
  }
}

bool Rtl8733bDevice::configure_tx_power(SelectedChannel channel) {
  _tssi_tracking = false;
  /* Closed-loop TSSI is the TX-power control on a TSSI-offset PG unit, so it
   * is not optional there: the flat fallback index below is a conservative
   * bring-up value, and on-air it runs cold enough that HT rates do not
   * survive the link (measured on the DUT: MCS7 undecodable by an RTL8812AU
   * witness, 300/300 submitted, 0 captured). A unit whose EFUSE carries no
   * TSSI calibration has nothing to drive the loop and takes the flat path. */
  if (_efuse.tx_power_mode != rtl8733b::TxPowerPgMode8733b::TssiOffset) {
    /* No loop, so no actuator. A runtime offset latched BEFORE bring-up (the
     * demos' ordering: DEVOURER_TX_PWR_OFFSET_QDB is applied before InitWrite)
     * was accepted on the promise of being applied here, and there is nothing
     * to apply it to. Drop it loudly and zero the latch so the reported state
     * agrees with the chip — leaving it set would make GetTxPowerState claim an
     * offset no register carries, which is the exact failure this knob exists
     * to end. Not fatal: an unported optional knob is not a hardware-safety
     * event. */
    if (_tx_offset_qdb != 0) {
      _logger->error(
          "RTL8733B: dropping the {} qdB TX-power offset — this unit's EFUSE "
          "carries no TSSI calibration, so TX power is the fixed flat index "
          "and has no runtime actuator",
          _tx_offset_qdb);
      _tx_offset_qdb = 0;
      _tx_sat_low = false;
      _tx_sat_high = false;
    }
    return _phy.set_flat_tx_power(rtl8733b::kSafeTxAgcIndex8733b);
  }

  /* Pick the thermal-compensation curve once, from the TX mode configured at
   * this point, and leave it alone — the vendor's own setup keys the table on
   * `phydm_get_tx_rate` at TSSI-setup time and never re-selects it at runtime.
   * It is never re-selected per frame, so send_packet does no register I/O,
   * like every other generation.
   *
   * The curve choice is inert on this silicon at any temperature the part
   * reaches: the CCK and OFDM/HT tables are bit-identical for thermal deltas
   * 0..+17 and first differ at +18, while a five-minute max-duty MCS7 soak
   * plateaued at +8 after two minutes and stopped climbing. It is set from the
   * rate class anyway because that costs nothing and is what the vendor does —
   * but do not expect it to be measurable.
   *
   * Which is just as well, because this is weaker than the vendor's keying in
   * the canonical demo flow: txdemo calls InitWrite before SetTxMode, so
   * _tx_mode_default is usually unset here and the OFDM/HT table is what a
   * DEVOURER_TX_RATE=1M session loads — the CCK branch is reached only by a
   * caller that configures the mode first, or by a later SetMonitorChannel.
   * The vendor keys on the rate in flight at setup instead. Nothing chases
   * that gap, because closing it would buy a table identical to the one
   * already loaded at every delta this part reaches. */
  const bool cck_table = _tx_mode_default.has_value() &&
                         _tx_mode_default->mode ==
                             devourer::TxMode::Mode::Legacy &&
                         rtl8733b::is_cck_rate_500kbps(
                             _tx_mode_default->legacy_rate_500kbps);
  if (!_phy.prepare_tssi_bb(channel, _efuse) ||
      !_phy.prepare_tssi_thermal(_efuse, cck_table) ||
      !_phy.prepare_tssi_offsets(channel, _efuse) ||
      /* The ceiling argument stays the safe first-light value; a POSITIVE
       * session offset therefore lifts the resulting targets above it here, by
       * design (GetTxPowerCaps argues why the positive half exists). That is a
       * deliberate widening of what bring-up can program, not a hole in
       * enable_tssi_tracking's `max_target_qdbm > 64` guard, which still refuses
       * a caller that tries to raise the ceiling itself. */
      !_phy.enable_tssi_tracking(channel, _efuse,
                                 rtl8733b::kSafeTssiTargetQdbm8733b,
                                 _tx_offset_qdb))
    return false;
  _tssi_tracking = true;
  /* The enable above already installed the offset, so this recomputes the same
   * plan, writes nothing, and returns the rails it hit — which is how the
   * saturation flags stay chip-derived on the bring-up and channel-change
   * paths without a second code path for the arithmetic. */
  rtl8733b::TssiOffsetSat8733b sat;
  if (!_phy.set_tssi_offset(channel, rtl8733b::kSafeTssiTargetQdbm8733b,
                            _tx_offset_qdb, &sat))
    return false;
  _tx_sat_low = sat.low;
  _tx_sat_high = sat.high;
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
      _logger->debug("RTL8733B RX: completion #{} -> {} bytes", completions,
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
      /* First-frames breadcrumb: per-frame diagnostics belong on the debug
       * plane, like every other generation's RX walk. */
      if (++frames <= 8) {
        const uint8_t *status = data + offset + rtl8733b::kRxDescSize;
        _logger->debug(
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

  /* Floor at the device-side aggregate ceiling, not at some generic minimum: a
   * URB smaller than one aggregate lets the aggregate straddle two completions,
   * which is the exact failure the 12 KiB cap was introduced to remove (the
   * walk stays in bounds, but every split aggregate becomes a malformed abort
   * and its frames are lost). DEVOURER_RX_URB_BYTES may raise this, never
   * lower it below rtl8733b::kRxAggregateBytes8733b. */
  int urb_bytes = _cfg.rx.urb_bytes.value_or(16 * 1024);
  if (urb_bytes < rtl8733b::kRxAggregateBytes8733b) {
    if (_cfg.rx.urb_bytes)
      _logger->warn(
          "RTL8733B RX: raising urb_bytes {} -> {} (device RX aggregate cap)",
          urb_bytes, rtl8733b::kRxAggregateBytes8733b);
    urb_bytes = rtl8733b::kRxAggregateBytes8733b;
  }
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

void Rtl8733bDevice::FastRetune(uint8_t channel, bool cache_rf) {
  std::lock_guard<std::recursive_mutex> lock(_reg_mu);
  if (_phy_ready && channel == _channel.Channel)
    return;
  SelectedChannel target = _channel;
  target.Channel = channel;
  if (_phy_ready &&
      _phy.fast_retune(target, _tssi_tracking,
                       rtl8733b::kSafeTssiTargetQdbm8733b, cache_rf,
                       _tx_offset_qdb)) {
    _channel = target;
    return;
  }
  /* Fast path declined (band/width change, cold radio) — full channel set at
   * the current width/offset, under the same recursive lock. */
  SetMonitorChannel(target);
}

bool Rtl8733bDevice::send_packet(const uint8_t *packet, size_t length) {
  std::lock_guard<std::recursive_mutex> lock(_reg_mu);
  if (!_phy_ready || !_mac_ready || !_tx_ready) {
    _logger->error("RTL8733B TX rejected before InitWrite");
    return false;
  }
  if (packet == nullptr)
    return false;
  /* No thermal read here. read_thermal() is 3 RF writes + a 15 us settle + an
   * RF read, each several USB control transfers — per frame that dominates the
   * send budget on a USB-HS part, and no other generation does register I/O on
   * the send path. The bring-up snapshot in InitWrite is the baseline; live
   * monitoring rides GetThermalStatus from the caller's own cadence. */
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

size_t Rtl8733bDevice::send_packets(const TxPacketView *pkts, size_t count) {
  const unsigned agg = _cfg.tx.usb_agg_max;
  if (agg <= 1 || !_device.is_usb() || count == 0)
    return IRtlDevice::send_packets(pkts, count);

  std::lock_guard<std::recursive_mutex> lock(_reg_mu);
  if (!_phy_ready || !_mac_ready || !_tx_ready) {
    _logger->error("RTL8733B TX rejected before InitWrite");
    return 0;
  }

  devourer::TxAggLimits lim;
  lim.desc_size = rtl8733b::kTxDescSize;
  lim.bulk_size = _device.speed() >= devourer::kUsbSpeedSuper  ? 1024
                  : _device.speed() >= devourer::kUsbSpeedHigh ? 512
                                                               : 64;
  /* BLK_DESC_NUM: MAC init already programs 3 into DWBCN0_CTRL[7:4]
   * (Halmac8733bMac.cpp) — the same field and value the HalMAC 88xx siblings
   * use — so 3 descriptors per bulk transfer is what this TXDMA parses.
   * Layout is rtw88/HalMAC parity: no first-block PKT_OFFSET reserve. */
  lim.max_frames = std::min<unsigned>(agg, 3u);
  lim.descs_per_bulk = 0;
  lim.first_reserve = false;

  size_t done = 0, ok = 0;
  while (done < count) {
    /* Collect the contiguous run for ONE URB. A frame carrying a radiotap
     * CHANNEL other than the session channel ends the run — this backend does
     * not retune mid-submission, so build_tx_block refuses such a frame
     * outright and it must not be packed beside frames that would have
     * aired. */
    std::vector<size_t> lens;
    for (size_t i = done; i < count && lens.size() < lim.max_frames; ++i) {
      /* A null view is treated exactly like a malformed one: it ends the run
       * and, if it led, is skipped per the IRtlDevice::send_packets
       * contract. */
      const uint16_t rlen =
          pkts[i].data == nullptr
              ? uint16_t{0}
              : devourer::radiotap_hdr_len(pkts[i].data, pkts[i].len);
      if (rlen == 0)
        break;
      const int want =
          devourer::radiotap_peek_channel(pkts[i].data, pkts[i].len);
      if (want > 0 && want != _channel.Channel) {
        /* This backend retunes for nobody mid-submission — build_tx_block
         * refuses an off-channel frame outright. A LEADING one still has to
         * enter the run alone, so the single-frame path below refuses it and
         * `done` moves past it; ending the run empty here instead would spin
         * this loop forever on the same entry, holding _reg_mu. */
        if (lens.empty())
          lens.push_back(pkts[i].len - rlen);
        break;
      }
      lens.push_back(pkts[i].len - rlen);
    }
    if (lens.empty()) {
      /* The leading view was null or malformed. This is also the loop's
       * termination guarantee: `done` advances on every iteration whatever
       * the collector above decided, so no future run rule can reintroduce a
       * non-advancing path. */
      ++done;
      continue;
    }

    const devourer::TxAggPlan plan =
        devourer::plan_tx_agg(lens.data(), lens.size(), lim);
    if (plan.frames() <= 1) {
      /* One block (or a frame the URB cap refuses): the classic single-frame
       * path is byte-identical and uncapped. */
      if (send_packet(pkts[done].data, pkts[done].len))
        ++ok;
      ++done;
      continue;
    }

    /* The block count rides the FIRST descriptor, and it must be in place
     * before that descriptor is checksummed — hence built in, not patched on
     * afterwards the way the 8822C does it (its checksum is recomputable in
     * isolation; this one is folded inside fill_tx_desc_8733b). */
    std::vector<uint8_t> urb(plan.total, 0);
    size_t built = 0;
    for (size_t k = 0; k < plan.frames(); ++k) {
      const uint8_t poff = (k == 0 && plan.shim) ? 1 : 0;
      const uint8_t anum =
          k == 0 ? static_cast<uint8_t>(plan.frames()) : uint8_t{0};
      if (build_tx_block(pkts[done + k].data, pkts[done + k].len,
                         urb.data() + plan.blocks[k].offset, poff, anum) == 0)
        break; /* pre-validated, so only a defensive bail */
      ++built;
    }
    if (built != plan.frames()) {
      for (size_t k = 0; k < plan.frames(); ++k, ++done)
        if (send_packet(pkts[done].data, pkts[done].len))
          ++ok;
      continue;
    }

    const int rc = _device.bulk_send_sync_ep(_device.first_bulk_out_ep(),
                                             urb.data(), urb.size(),
                                             /*timeout_ms=*/100);
    /* bulk_send_sync_ep returns BYTES SUBMITTED, so `rc >= 0` also covers a
     * short write. A truncated URB means the chip got a prefix — some
     * trailing block is partial or absent — and there is no way to say which
     * frames aired, so none of them may be reported as submitted. The
     * single-frame path already refuses a short write; the aggregated one
     * must not be the looser of the two in the same backend. */
    const bool sent_all = rc == static_cast<int>(urb.size());
    if (!sent_all)
      /* Both failure shapes reach stderr — rc < 0 (transport error) and a
       * short write — because the single-frame path logs both and this
       * backend has no NAK-backoff flood concern excusing silence. */
      _logger->error("RTL8733B aggregated TX failed/short on EP 0x{:02x}: "
                     "{}/{} ({} frames dropped)",
                     _device.first_bulk_out_ep(), rc, urb.size(),
                     plan.frames());
    devourer::Ev(_logger->events(), "tx.agg")
        .f("frames", (unsigned long long)plan.frames())
        .f("bytes", (unsigned long long)urb.size())
        .f("sent", (long long)rc)
        .f("shim", plan.shim)
        .f("ok", sent_all);
    if (sent_all) {
      ok += plan.frames();
      /* Same one-shot latch send_packet uses — a session whose very first TX
       * is aggregated must still say so once, or the "first TX accepted"
       * breadcrumb goes missing exactly when the packing is what is on
       * trial. It counts URB acceptances, not frames; it is a latch, not a
       * meter. */
      if (_tx_submits.fetch_add(1) == 0)
        _logger->info(
            "RTL8733B first TX accepted (aggregated): EP=0x{:02x} frames={} "
            "bytes={} shim={}",
            _device.first_bulk_out_ep(), plan.frames(), urb.size(), plan.shim);
    }
    done += plan.frames();
  }
  return ok;
}

size_t Rtl8733bDevice::build_tx_block(const uint8_t *packet, size_t length,
                                      uint8_t *out, uint8_t packet_offset,
                                      uint8_t agg_num) {
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
    /* One predicate covers the family (VHT/HE refused outright) and its
     * modulation parameters, so the SetTxMode branch and the radiotap branch
     * below cannot drift apart — and the refusals are testable without
     * hardware (tests/rtl8733b_tx_desc_selftest.cpp). */
    if (!rtl8733b::tx_mode_supported_8733b(mode))
      return 0;
    if (mode.mode == devourer::TxMode::Mode::Legacy) {
      fixed_rate = mode.legacy_rate_500kbps;
    } else {
      fixed_rate = static_cast<uint8_t>(MGN_MCS0 + mode.ht_mcs);
      frame_width = mode.bw_mhz == 40 ? CHANNEL_WIDTH_40 : CHANNEL_WIDTH_20;
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
      if (!m.have_mcs ||
          !rtl8733b::ht_request_supported_8733b(m.mcs, m.bw40 ? 40 : 20,
                                                m.sgi != 0, m.ldpc != 0,
                                                m.stbc != 0))
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
  cfg.agg_num = agg_num;
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
  if (!disabled) {
    /* `false` is the universal default — carrier-sense + EDCCA enabled — and
     * that is exactly what this backend's HALMAC MAC bring-up leaves
     * programmed. Succeed as a no-op: callers that assert the default
     * unconditionally (examples/timesync) must not be punished for asking for
     * the state the chip is already in. */
    return;
  }
  /* `true` (DEVOURER_DIS_CCA) is not ported. The HALMAC 87xx carrier-sense
   * gate has not been located and measured on this part, and this backend
   * does not guess at PHY/MAC writes it cannot read back. Refuse loudly, per
   * the pure-virtual contract in IRtlDevice — but do NOT tear the session
   * down: an unsupported optional knob is not a hardware-safety event, and
   * card-disabling here would leave the caller with a dead chip for asking a
   * question. The session stays up with standard carrier-sense. */
  _logger->error(
      "RTL8733B: CCA disable (DEVOURER_DIS_CCA) is not implemented by this "
      "backend; carrier-sense stays enabled");
  throw std::runtime_error(
      "RTL8733B CCA disable is not implemented by this backend");
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
  /* Lean FastRetune override exists (Phy8733b::fast_retune): intra-band,
   * same-width hops with TSSI tracking kept live. Measured on the
   * validation unit: ~55 ms call / ~10 ms p50 radio-live, vs the
   * ~330-440 ms full path (USB HS). */
  caps.fastretune_ok = true;
  /* 10 MHz only — kBw5 is deliberately absent from bw_mask and WIDTH_5 is
   * refused at channel_plan: on this die the 5 MHz BB small-BW mode airs no
   * packets (measured across the full DAC/ADC divider code space), while
   * 10 MHz is SDR- and cross-decode-qualified on both bands
   * (docs/rtl8733b.md "Narrowband status"). */
  caps.narrowband_ok = true;
  caps.txpwr = GetTxPowerCaps();
  return caps;
}

/* A dBm-TARGET model, not an index model — the TSSI loop's per-rate target
 * table is quarter-dBm, so index_max stays 0 (the value TxPowerCaps reserves
 * for exactly this shape) and one step is one qdB, the same answer Kestrel's
 * fixed-dBm BB target gives.
 *
 * Offset 0 is kSafeTssiTargetQdbm8733b (16 dBm), a first-light clip the backend
 * imposes at or below this part's factory targets — which run 18..20 dBm at
 * 2.4 GHz and 16..19 dBm at 5 GHz (kMaxPgTargetQdbm8733b).
 *
 * The range is the int8 delta field at BOTH ends, and neither end is a
 * characterised PA window. The 0x3a00 bytes are a signed offset from the
 * 64 qdBm anchor, so the field spans [-128, +127] — targets from -16 dBm to
 * +47.75 dBm. Where the hardware stops responding is measured and documented
 * per end rather than clamped, the same answer Jaguar1/3 give:
 *
 *   - Down to -128 qdB, a -16 dBm target. An earlier cut stopped at -64 (a
 *     0 dBm target) on the assumption that a negative absolute target was
 *     meaningless; the sweep says otherwise. Power keeps falling past it with
 *     no sign wrap — 7.3 dB more between the 0 dBm and -4 dBm targets — and
 *     only pins from about -96 qdB (-8 dBm), where three successive rungs read
 *     52.96 / 52.97 / 52.99. Stopping at -64 threw away ~9 dB of working
 *     backoff, which is real range for a near-field bench or a link that wants
 *     to sit quiet. So the floor is the field, as at the top, and ~-96 qdB is
 *     the measured end of usable travel — documented, not enforced.
 *   - Up to +127 qdB, the delta field's positive limit — the same
 *     clamped-only-at-the-hardware-rail answer Jaguar1 (+126) and Jaguar3
 *     (+127) give. src/TxPower.h is deliberate that headroom above the
 *     generated table belongs to the operator, and a per-unit EFUSE trimmed
 *     too cold is precisely the case a bench calibration exists to correct:
 *     clamping at the PG table would make this the one backend where a
 *     measured operating point cannot be commanded.
 *
 * Measured where that goes, because "the operator's call" is only a fair
 * answer if the operator is told what they are choosing between. Sweeping UP
 * from the clip (MCS0, ch36, witness EVM alongside RSSI): +16 qdB — the top of
 * the PG table — bought 2.8 dB but EVM had already fallen from -62 to -50; by
 * +32 the witness read 8.7 dB more RSSI with EVM COLLAPSED to -18, and +48 and
 * +64 changed nothing at all (RSSI pinned, EVM pinned at -18). That is the PA
 * in hard compression: more energy, unusable constellation, and SNR never
 * moved (58..64) so it cannot be the tell — see docs/bench-testing-near-field.md.
 * Below the clip EVM stays flat at -58..-61 across the whole 16 dB of backoff.
 * So the vendor's PG table lands about where this part stops being linear:
 * treat +16 qdB as the edge of usable overdrive, not the edge of the range.
 *
 * step_measured stays false, and now for a MEASURED reason rather than an
 * unexamined one. On-air against an RTL8812AU witness (chip-RSSI ground
 * station, tests/txpwr_offset_onair.sh's method; the B210 saturates at this
 * range), two independent 6-point passes: the lever is monotone and worth
 * 14.2 / 14.8 dB of received power for the full 16 dB of command, overall
 * slope 0.222 / 0.231 dB per qdB against the 0.25 nominal. But the step is NOT
 * constant across the advertised range — the bottom 12 qdB delivered 0.125 and
 * 0.126 dB/qdB, the one structure that reproduced exactly, while everything
 * above -52 qdB ran 0.233..0.242. The closed loop compresses as its target
 * approaches 0 qdBm, so a controller near the floor gets about half the dB it
 * asked for while saturated_low still reads false (the clamp only fires at
 * -64). This is the same call the 8822E gets for the same reason: calibrate
 * your own dB-per-qdB, or lean on GetTxPowerState plus the ground's RSSI.
 *
 * The counterparts: one physical unit, one witness, near-field geometry, and
 * an integer-quantised RSSI scale. The mid-range slope scattered 0.219..0.252
 * between the two passes, so the ~0.24 figure is a bench average, not a
 * constant. No SDR has been on this silicon, as with every other RF-domain
 * claim in this backend.
 *
 * Static and state-free, per the GetAdapterCaps contract (resolved at
 * construction, callable before Init, safe from any thread). In particular it
 * does NOT consult the EFUSE PG mode, which is unknown until bring-up: a
 * flat-PG unit's lack of an actuator surfaces on SetTxPowerOffsetQdb (refused,
 * loudly) and GetTxPowerState, not by mutating the family's capabilities. */
devourer::TxPowerCaps Rtl8733bDevice::GetTxPowerCaps() {
  devourer::TxPowerCaps c;
  c.supported = true;
  c.index_max = 0;
  c.step_qdb = 1;
  c.step_measured = false;
  c.offset_min_qdb = -128; /* the int8 delta field's negative limit */
  c.offset_max_qdb = 127; /* the int8 per-rate delta field's positive limit */
  c.rate_diffs = false;
  c.rate_diffs_hw_table = false;
  c.rate_diffs_measured = false;
  return c;
}

void Rtl8733bDevice::SetTxPowerIndexOverride(int idx) {
  /* kSafeTxAgcIndex8733b is the only flat index this backend programs, and it
   * was witnessed unable to carry HT at all (MCS7, 300/300 submitted, 0
   * captured, twice), so no dB-per-step slope has ever been measured for the
   * index on this part. SetTxPowerOffsetQdb is the ported lever. */
  _logger->error("RTL8733B: SetTxPowerIndexOverride({}) ignored — the flat "
                 "TXAGC index is not ported on this backend; use "
                 "SetTxPowerOffsetQdb (GetTxPowerCaps reports the dBm model)",
                 idx);
}

int Rtl8733bDevice::SetTxPowerOffsetQdb(int qdb) {
  std::lock_guard<std::recursive_mutex> lock(_reg_mu);
  const devourer::TxPowerCaps caps = GetTxPowerCaps();
  const int applied = devourer::quantize_offset_qdb(qdb, caps, nullptr);
  const bool req_low = qdb < caps.offset_min_qdb;
  const bool req_high = qdb > caps.offset_max_qdb;

  if (_tx_ready && !_tssi_tracking) {
    /* This unit's EFUSE carries no TSSI calibration, so TX runs the flat
     * kSafeTxAgcIndex8733b path, which has no runtime actuator here. Say so
     * rather than return a number that reads like a successful apply — that
     * indistinguishability is the whole reason this knob exists. */
    _logger->error(
        "RTL8733B: SetTxPowerOffsetQdb({}) refused — no TSSI calibration on "
        "this unit, so TX power is the fixed flat index and has no runtime "
        "actuator",
        qdb);
    return 0;
  }

  if (!_tx_ready) {
    /* Recorded now, applied by configure_tx_power at InitWrite — the family
     * contract for a knob moved before the chip is up. */
    _tx_offset_qdb = static_cast<int16_t>(applied);
    _tx_sat_low = req_low;
    _tx_sat_high = req_high;
    _logger->info("RTL8733B: TX-power offset {} qdB recorded (requested {}), "
                  "applied at InitWrite",
                  applied, qdb);
    return applied;
  }

  rtl8733b::TssiOffsetSat8733b sat;
  if (!_phy.set_tssi_offset(_channel, rtl8733b::kSafeTssiTargetQdbm8733b,
                            applied, &sat))
    return 0;
  _tx_offset_qdb = static_cast<int16_t>(applied);
  _tx_sat_low = sat.low || req_low;
  _tx_sat_high = sat.high || req_high;
  _logger->info("RTL8733B: SetTxPowerOffsetQdb({}) -> applied {} qdB "
                "(target {} qdBm) sat_low={} sat_high={}",
                qdb, applied,
                rtl8733b::kSafeTssiTargetQdbm8733b + applied, _tx_sat_low,
                _tx_sat_high);
  return applied;
}

devourer::TxPowerState Rtl8733bDevice::GetTxPowerState() {
  std::lock_guard<std::recursive_mutex> lock(_reg_mu);
  devourer::TxPowerState s;
  if (!_phy_ready || !_tx_ready)
    return s; /* valid=false — no TX-power state has been programmed yet. */
  s.valid = true;

  if (!_tssi_tracking) {
    /* Flat-PG unit: no actuator, but the TXAGC registers ARE the level and
     * they read back, so report chip truth. set_flat_tx_power writes one index
     * to both references and zeroes the per-rate diffs, so every
     * representative rate sits at that index. */
    const rtl8733b::TxAgcState8733b agc = _phy.read_txagc_state();
    /* Note the reading this shares with src/TxPower.h's convention: there,
     * flat_index >= 0 means "a flat override is active" and clearing it is the
     * caller's move.  Here nothing can have set one — SetTxPowerIndexOverride
     * refuses — and there is nothing to clear.  The index is simply what this
     * unit runs at, because a no-TSSI-calibration EFUSE leaves bring-up's flat
     * index as the level.  A consumer reaching for
     * SetTxPowerIndexOverride(-1) to "release" it gets a logged refusal, which
     * is the honest answer: this unit has no runtime power actuator at all. */
    s.flat_index = agc.ofdm_ref_a;
    s.cck_index = agc.cck_ref_a;
    s.ofdm_index = agc.ofdm_ref_a;
    s.mcs7_index = agc.ofdm_ref_a;
    s.hw_readback = true;
    return s;
  }

  /* TSSI path: a dBm-target model, so there is no TXAGC index to report and
   * flat_index / the three representative fields stay -1 rather than carrying
   * quarter-dBm targets in fields declared as indices (Kestrel reports the
   * same shape). hw_readback says the offset below was confirmed against the
   * chip's live target table, not just read out of this shadow — the
   * distinction a consumer needs when the whole failure mode being fixed was a
   * shadow that always agreed with itself. */
  s.offset_qdb = _tx_offset_qdb;
  s.offset_steps = _tx_offset_qdb; /* 1 step == 1 qdB on the dBm model */
  s.saturated_low = _tx_sat_low;
  s.saturated_high = _tx_sat_high;
  s.hw_readback = _phy.tssi_offsets_confirmed(
      _channel, rtl8733b::kSafeTssiTargetQdbm8733b, _tx_offset_qdb);
  /* Latched, because this is a getter a control loop polls: an unconfirmed
   * chip would otherwise emit one warning per poll forever. The state field is
   * the machine-readable signal; the log line only has to fire the first
   * time. */
  if (!s.hw_readback && !_tx_readback_warned) {
    _tx_readback_warned = true;
    _logger->warn("RTL8733B: TX-power state unconfirmed — the chip's TSSI "
                  "target table does not match the {} qdB offset this session "
                  "believes it applied (warned once)",
                  _tx_offset_qdb);
  }
  return s;
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
