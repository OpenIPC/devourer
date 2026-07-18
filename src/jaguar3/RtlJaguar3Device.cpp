#include "RtlJaguar3Device.h"

#include <algorithm>
#include <climits> /* INT_MIN — "no radiotap DBM_TX_POWER" sentinel */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <utility>
#include <vector>

#include "AckResponder.h"
#include "RadiotapPeek.h"   /* send_packets batch pre-parse */
#include "RadiotapTxFlags.h" /* HT MCS field decoder (LDPC/STBC) */
#include "TxAggPlan.h"      /* USB TX aggregation URB packing */
#include "TxReport.h"     /* CCX TX-status report decode + tx.report event */

#include "BeamformingSounder.h" /* generation-neutral BF self-sounding recipe */

#include "ChannelFreq.h" /* radiotap CHANNEL freq -> channel (per-packet hop) */
#include "FrameParserJaguar3.h"
#include "NhmReader.h"       /* frame-free NHM power histogram (shared) */
#include "RateDefinitions.h" /* MGN_* rate enum (shared across the family) */
#include "SignalStop.h" /* g_devourer_should_stop — set by demo signal handlers */
#include "ToneMask.h"   /* DEVOURER_RX_CSI_MASK / DEVOURER_RX_NBI knobs */

extern "C" {
#include "ieee80211_radiotap.h" /* MRateToHwRate + radiotap iterator */
}

/* 8822e TXAGC reference base (7-bit index, ~0.25 dB/step) programmed by default
 * when DEVOURER_TX_PWR is unset. Matches the efuse-calibrated HT/OFDM base the
 * kernel derives for this family (its tx_power_idx proc reports base 75 for 5G
 * HT); verified on-air that programming it lifts the MCS7 signal from ~7 dB to
 * the kernel's ~10 dB above the SDR noise floor. See InitWrite for context. */
static constexpr uint8_t JAGUAR3_TXPWR_REF_BASE = 0x4b;
/* 8822c (CU) flat reference: the value the demo previously imposed (40 = 0x28),
 * kept so the SDR-validated CU behaviour is unchanged after the demo stopped
 * setting a default. */
static constexpr uint8_t JAGUAR3_TXPWR_REF_BASE_8822C = 0x28;

RtlJaguar3Device::RtlJaguar3Device(RtlAdapter device, Logger_t logger,
                                   jaguar3::ChipVariant variant,
                                   devourer::DeviceConfig cfg)
    : _device{device}, _cfg{std::move(cfg)}, _logger{logger},
      _variant{variant}, _hal{device, logger, variant, _cfg},
      _radioManagement{device, logger, variant, _cfg}, _phydm{device, logger} {
  _logger->info("RtlJaguar3Device constructed ({})",
                variant == jaguar3::ChipVariant::C8822E ? "8822E/EU" : "8822C/CU");
}

void RtlJaguar3Device::Init(Action_ParsedRadioPacket packetProcessor,
                            SelectedChannel channel) {
  _channel = channel;
  _rx_wanted = true;
  _hal.rtw_hal_init(channel);  /* full vendor-source bring-up */
  /* Tune the channel/bandwidth (5/10 MHz ChannelWidth re-clocks to narrowband),
   * then run IQK calibration (it reads RF18 for the tuned channel).
   *
   * The 8822C's IQK must run with the RF at 20 MHz: the vendor kernel only
   * ever calibrates at its bring-up bandwidth and carries the coefficients
   * across `iw set freq ... 80` (usbmon: the 20->80 transition contains no
   * IQK-engine traffic), and devourer's IQK executed AT 80 MHz loads RxIQC
   * coefficients that leave the receiver unable to sync ANY frame
   * (hardware-bisected: SKIP_IQK alone revives 80 MHz RX). Calibrate at
   * 20 MHz on the target channel, then retune to the requested width. */
  const bool iqk_at_20 = _variant == jaguar3::ChipVariant::C8822C &&
                         (channel.ChannelWidth == CHANNEL_WIDTH_40 ||
                          channel.ChannelWidth == CHANNEL_WIDTH_80);
  _radioManagement.set_channel_bwmode(channel.Channel, channel.ChannelOffset,
                                      iqk_at_20 ? CHANNEL_WIDTH_20
                                                : channel.ChannelWidth);
  SelectedChannel iqk_ch = channel;
  if (iqk_at_20)
    iqk_ch.ChannelWidth = CHANNEL_WIDTH_20; /* IQK command set follows the RF */
  _hal.run_iqk(iqk_ch);
  if (iqk_at_20)
    _radioManagement.set_channel_bwmode(channel.Channel, channel.ChannelOffset,
                                        channel.ChannelWidth);
  _hal.enable_rx_path(); /* RF into RX mode (IGI toggle) — must follow channel set */
  _hal.config_rfe(channel.Channel); /* 8822e RFE/PAPE antenna-switch pins */
  _hal.config_channel_8822e(channel.Channel); /* 8822e band TX scaling/backoff + shaping */
  _hal.coex_wlan_only_init(); /* lock antenna to WLAN (disable BT/LTE coex) */
  /* 8822E DPDT/eFEM pin-mux — post-coex, so an RX-only session also gets both
   * receive chains (GPIO13 -> RFE engine). Kernel parity: _efem_pinmux_config
   * runs in rtl8822e_init_misc in both directions. See InitWrite for the TX
   * bring-up's copy of this call. No-op on non-8822E. */
  apply_dpdt_route_8822e();
  _brought_up = true;

  /* DEVOURER_XTAL_CAP — crystal-cap trim (issue #217, narrowband CFO lever). */
  if (_cfg.tuning.xtal_cap)
    SetXtalCap(*_cfg.tuning.xtal_cap);

  /* tuning.disable_cca — MAC EDCCA-disable research knob (see SetCcaMode). */
  if (_cfg.tuning.disable_cca)
    SetCcaMode(true);

  /* DEVOURER_BF_ARM_BFEE=aa:bb:cc:dd:ee:ff — beamforming self-sounding probe
   * (beamformee side), Jaguar-3 variant. Arms the hardware CSI responder to
   * reply to NDPA+NDP from the given beamformer MAC with a VHT Compressed
   * Beamforming report, no association. Uses the shared MAC recipe with the
   * Jaguar-2/3 config (0xDB, 16-bit CSI param, RX-filter + own-AID gates, and
   * crucially NO 0x9B4 write — that address is the narrowband clock divider on
   * this generation). See BeamformingSounder.h. */
  if (_cfg.bf.beamformee_of) {
    const uint8_t *mac = _cfg.bf.beamformee_of->data();
    /* Jaguar-3 bring-up never programs the self-MAC (0x0610), so the NDPA
     * RA has nothing to match. Give the beamformee a known identity here so
     * the sounder can address it; log it for the test harness. */
    static const uint8_t kBfeeMac[6] = {0x00, 0xe0, 0x4c, 0x88, 0x22, 0xce};
    for (uint16_t i = 0; i < 6; ++i)
      _device.rtw_write8(0x0610 + i, kBfeeMac[i]);
    /* bf.mu upgrades the responder to an MU beamformee, whose report appends
     * the per-subcarrier delta-SNR (MU Exclusive Beamforming Report) the SU
     * report omits. */
    if (_cfg.bf.mu) {
      devourer::bf::arm_beamformee_mu(_device, mac, devourer::bf::kBfeeJaguar23);
      _logger->info("Jaguar3 BF MU-beamformee armed for beamformer "
                    "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} — "
                    "beamformee MAC 00:e0:4c:88:22:ce",
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else {
      devourer::bf::arm_beamformee(_device, mac, devourer::bf::kBfeeJaguar23);
      _logger->info("Jaguar3 BF beamformee armed for beamformer "
                    "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} — "
                    "beamformee MAC 00:e0:4c:88:22:ce",
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
  }

  if (_cfg.rx.ack_responder)
    SetAckResponder(*_cfg.rx.ack_responder); /* DEVOURER_ACK_RESPONDER */
  apply_replay_wseq(); /* DEVOURER_REPLAY_WSEQ — end of both bring-ups,
                        * like Jaguar2's. */
  if (_cfg.debug.bb_dump) {
    /* Same MAC+BB end-state dump as InitWrite's, at the end of the RX
     * bring-up (kernel rtw_proc read_reg diffing). */
    for (uint32_t a = 0x000; a <= 0x4ffc; a += 0x10)
      _logger->info("BBDUMP 0x{:04x} 0x{:08x} 0x{:08x} 0x{:08x} 0x{:08x}", a,
                    _device.rtw_read32(a), _device.rtw_read32(a + 4),
                    _device.rtw_read32(a + 8), _device.rtw_read32(a + 12));
  }
  StartRxLoop(std::move(packetProcessor));
}

void RtlJaguar3Device::StartRxLoop(Action_ParsedRadioPacket packetProcessor) {
  _packetProcessor = std::move(packetProcessor);
  /* Restartable: clear any stop request left by a prior StopRxLoop(). */
  _rx_stop = false;
  /* Take over bulk-IN from the coex thread's C2H drain up front — also during
   * the register restore below, so its 200 ms bulk reads don't interleave with
   * the RX-path enable sequence. */
  _rx_loop_active = true;

  /* An InitWrite bring-up is TX-oriented: it closed the RX filters
   * (0x6A0-0x6A4, so a TX-only session's unread frames can't fill the on-chip
   * RX FIFO) and never ran enable_rx_path (the 3-wire IGI toggle that puts the
   * RF into RX mode — without it the chip delivers zero frames). This caller
   * wants RX, so restore the monitor_rx_cfg accept-all filters and enable the
   * RX path. Serialized against the coex housekeeping tick, the only other
   * register writer right now. */
  if (_rx_filters_closed) {
    /* A register read can transiently fail right after InitWrite (the FW is
     * still settling from the coex H2C burst) — give it a beat and retry a few
     * times instead of letting the exception tear down the caller's RX
     * thread. */
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    for (int attempt = 0; attempt < 5; ++attempt) {
      try {
        std::lock_guard<std::mutex> lk(_reg_mu);
        _device.rtw_write<uint16_t>(0x06A0, 0xFFFF);
        _device.rtw_write<uint16_t>(0x06A2, 0xFFFF);
        _device.rtw_write<uint16_t>(0x06A4, 0xFFFF);
        _hal.enable_rx_path();
        _rx_filters_closed = false;
        _logger->info("Jaguar3: RX filters re-opened + RX path enabled for "
                      "concurrent TX+RX");
        break;
      } catch (const std::exception &e) {
        _logger->error("Jaguar3: RX-path restore failed ({}){}", e.what(),
                       attempt < 4 ? " — retrying" : " — giving up");
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      }
    }
  }

  /* DEVOURER_RX_CSI_MASK / DEVOURER_RX_NBI — RX-side per-subcarrier masking
   * (receive-equalizer CSI mask / narrowband notch). Applied after the channel
   * set so it is the final word for a single-channel capture; serialized
   * against the coex housekeeping tick like every other register writer here.
   * See ToneMask.h. */
  if (_cfg.rx.csi_mask || _cfg.rx.nbi) {
    std::lock_guard<std::mutex> lk(_reg_mu);
    devourer::tonemask::apply(
        _device, _logger, devourer::tonemask::Family::JGR3, _channel, 2,
        _cfg.rx.csi_mask ? _cfg.rx.csi_mask->c_str() : nullptr,
        _cfg.rx.nbi ? _cfg.rx.nbi->c_str() : nullptr);
  }

  _logger->info("Jaguar3: entering RX loop (kernel-style async URB queue)");
  uint64_t frames = 0, reads = 0;
  /* Closed-loop CFO tracking runs on the RECEIVER (#217): tick the controller
   * on a ~2 s wall-clock cadence from within the RX loop, off the per-frame
   * CFO fed below. (The coex thread's copy only runs on the TX path.) The
   * crystal-cap register (0x1040) is written READ-FREE from a base cached
   * before the RX flood — a control-transfer READ races the async bulk-IN and
   * throws under load, so the tick composes the field onto the cached base and
   * write32s it. */
  uint32_t reg1040_base = 0;
  bool reg1040_ok = false;
  if (_cfg.tuning.cfo_track) {
    try {
      reg1040_base = _device.rtw_read<uint32_t>(0x1040) & ~0x00FFFC00u;
      reg1040_ok = true;
    } catch (...) {
      _logger->info("Jaguar3 cfo.track: 0x1040 base read failed — disabled");
    }
  }
  /* phydm dynamic mechanisms on RX-only sessions: the vendor watchdog's ~2 s
   * cadence on a dedicated thread (register I/O CANNOT run on this thread —
   * a sync control transfer from the bulk-IN event thread starves its own
   * event loop; a separate thread under _reg_mu is the proven pattern, same
   * as the TX-side coex thread, which owns the tick in TX+RX mode). */
  std::atomic<bool> phydm_stop{false};
  std::thread phydm_thread;
  if (!_coex_thread.joinable()) {
    phydm_thread = std::thread([this, &phydm_stop] {
      auto next = std::chrono::steady_clock::now() + std::chrono::seconds(2);
      while (!phydm_stop && !g_devourer_should_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (std::chrono::steady_clock::now() < next)
          continue;
        next += std::chrono::seconds(2);
        try {
          std::lock_guard<std::mutex> lk(_reg_mu);
          _phydm.tick(_channel.Channel, !_cca_disabled);
        } catch (...) {
          break; /* chip gone — the RX loop will wind down too */
        }
      }
    });
  }
  auto cfo_next = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  auto cfo_tick = [&]() {
    if (!_cfg.tuning.cfo_track || !reg1040_ok)
      return;
    if (std::chrono::steady_clock::now() < cfo_next)
      return;
    cfo_next += std::chrono::seconds(2);
    double avg_khz = 0;
    const int cur = _xtal_cap < 0 ? 0x20 : _xtal_cap;
    const int nc = _cfo.step(cur, 0x7f, &avg_khz);
    DVR_DEBUG(_logger, "Jaguar3 cfo.track tick: cfo~{} (raw*2.5) cap=0x{:02x} {}",
              static_cast<int>(avg_khz), cur, nc >= 0 ? "step" : "hold");
    if (nc >= 0) {
      const uint32_t field = static_cast<uint32_t>(nc) |
                             (static_cast<uint32_t>(nc) << 7); /* 14-bit */
      try {
        std::lock_guard<std::mutex> lk(_reg_mu);
        _device.rtw_write<uint32_t>(0x1040,
                                    reg1040_base | ((field << 10) & 0x00FFFC00u));
        _xtal_cap = nc;
      } catch (...) {
        return;
      }
    }
    if (nc >= 0) /* log only on an actual step, not every idle tick */
      _logger->info("Jaguar3 cfo.track: cfo~{} (raw*2.5) xtal_cap=0x{:02x}",
                    static_cast<int>(avg_khz), _xtal_cap);
  };
  /* Process one bulk-IN completion: walk the aggregated 8822C RX descriptors. */
  auto on_data = [&](const uint8_t *data, int n) {
    cfo_tick();
    if (++reads <= 8)
      _logger->info("Jaguar3 RX: async completion #{} -> {} bytes", reads, n);
    uint32_t off = 0;
    while (off + jaguar3::RXDESC_SIZE_8822C <= static_cast<uint32_t>(n)) {
      jaguar3::Rx8822cFrame f;
      if (!jaguar3::parse_rx_8822c(data + off, static_cast<size_t>(n) - off, f))
        break;
      if (_packetProcessor) {
        Packet p{};
        p.RxAtrib.pkt_len = static_cast<uint16_t>(f.frame_len);
        p.RxAtrib.crc_err = f.crc_err;
        p.RxAtrib.icv_err = f.icv_err;
        p.RxAtrib.data_rate = f.rx_rate;
        p.RxAtrib.tsfl = f.tsfl;
        p.RxAtrib.paggr = f.paggr;
        p.RxAtrib.ppdu_cnt = f.ppdu_cnt;
        p.RxAtrib.drvinfo_sz = static_cast<uint8_t>(f.drvinfo_size);
        p.RxAtrib.shift_sz = f.shift;
        /* RX desc word2 BIT(28) = FW C2H report, not an 802.11 frame. During
         * concurrent TX+RX the FW emits one small C2H per TX, so tag them for
         * the packetProcessor (which skips C2H) instead of surfacing a flood
         * of short "frames". Same check the coex drain uses. */
        bool is_c2h = (data[off + 11] & 0x10) != 0;
        p.RxAtrib.pkt_rpt_type = is_c2h ? RX_PACKET_TYPE::C2H_PACKET
                                        : RX_PACKET_TYPE::NORMAL_RX;
        /* CCX TX report (DEVOURER_TX_REPORT / SPE_RPT feedback): the halmac
         * C2H pkt 0xFF/0x0F carries per-frame delivery + retry count —
         * decode + emit tx.report (src/TxReport.h). */
        if (is_c2h && devourer::is_ccx_halmac(f.frame, f.frame_len))
          devourer::emit_tx_report(
              _logger->events(),
              devourer::parse_ccx_halmac(f.frame, f.frame_len), "halmac");
        /* Decode the jgr3 PHY-status report (per-frame RSSI/SNR/EVM) when it is
         * present (monitor_rx_cfg enables APP_PHYSTS + RX_DRVINFO_SZ=4, so the
         * 32-byte report is counted in drvinfo). Skips C2H reports and any
         * frame whose drvinfo is too short (e.g. CCK, which carries no OFDM
         * report). The report sits immediately after the 24-byte descriptor. */
        if (!is_c2h && f.drvinfo_size >= 28)
          jaguar3::parse_phy_sts_jgr3(data + off + jaguar3::RXDESC_SIZE_8822C,
                                      f.drvinfo_size, p.RxAtrib);
        p.Data = std::span<uint8_t>(const_cast<uint8_t *>(f.frame), f.frame_len);
        if (!p.RxAtrib.crc_err) {
          _rxq.add(p.RxAtrib.rssi[0], p.RxAtrib.snr[0], p.RxAtrib.evm[0]);
          _rxpaths.add(p.RxAtrib.rssi, 2); /* 8822C/8822E are 2T2R */
          if (_cfg.tuning.cfo_track)
            _cfo.add(p.RxAtrib.cfo_tail); /* closed-loop CFO input (#217) */
        }
        /* TX-BF apply gate (DEVOURER_BF_TXBF): a VHT Compressed Beamforming
         * Report (category 0x15, action 0x00) from the target peer (addr2) means
         * the WMAC just ingested a fresh V matrix — enable the apply toggle so
         * subsequent TX is steered. Enabling it before any CBR (no V) degrades
         * the link, so this is the gate. One-shot; V refreshes on each periodic
         * re-sound automatically. */
        if (_bf_txbf_armed && !p.RxAtrib.crc_err && f.frame_len >= 26) {
          const uint8_t *fb = f.frame;
          if (fb[24] == 0x15 && fb[25] == 0x00 &&
              std::memcmp(fb + 10, _bf_peer, 6) == 0) {
            _bf_cbr_count.fetch_add(1, std::memory_order_relaxed);
            if (!_bf_apply_on.load(std::memory_order_relaxed)) {
              std::lock_guard<std::mutex> lk(_reg_mu);
              devourer::bf::apply_vmatrix(
                  _device, true, static_cast<uint8_t>(_channel.ChannelWidth));
              _bf_apply_on.store(true, std::memory_order_relaxed);
              _logger->info("Jaguar3 BF: CBR from peer ingested — TXBF apply "
                            "ENABLED (steering subsequent TX)");
            }
          }
        }
        _packetProcessor(p);
      }
      if (++frames <= 5)
        _logger->info("Jaguar3 RX: frame len={} rate={} crc={}", f.frame_len,
                      f.rx_rate, f.crc_err);
      if (f.next_offset == 0)
        break;
      off += f.next_offset;
    }
  };
  /* _rx_loop_active was set on entry (bulk-IN handover from the coex thread's
   * C2H drain — its one in-flight 200 ms bulk_read_raw may steal at most one
   * completion, harmless). Give the endpoint back on exit. */
  _device.bulk_read_async_loop(32 * 1024, 8, on_data, [this]() -> bool {
    return _rx_stop || g_devourer_should_stop;
  });
  _rx_loop_active = false;
  phydm_stop = true;
  if (phydm_thread.joinable())
    phydm_thread.join();
}

/* Coex runtime (port of the rtw88 watchdog's coex path): drain the firmware's
 * C2H reports off bulk-IN so the on-chip C2H buffer never fills, and every ~2 s
 * re-apply the 5 GHz coex decision + FW heartbeats so the PTA keeps the antenna
 * with WLAN during sustained TX. Runs concurrently with the TX loop. */
/* Ensure the coex thread is always joined, even if Stop() was never called
 * (exception path / caller that drops the device) — a joinable std::thread in
 * the destructor would otherwise std::terminate. Idempotent with Stop(). */
RtlJaguar3Device::~RtlJaguar3Device() {
  /* Safety net: restore the chip if a CW tone is still armed (before the coex
   * thread is joined — StopCwTone serializes on _reg_mu with it). */
  StopCwTone();
  _coex_stop = true;
  if (_coex_thread.joinable())
    _coex_thread.join();
}

void RtlJaguar3Device::coex_runtime_loop() {
  std::vector<uint8_t> buf(16 * 1024);
  uint64_t tick = 0, c2h = 0, rx = 0;
  /* The coex decision + FW heartbeats run on a fixed ~2 s WALL-CLOCK cadence
   * (steady_clock), independent of how fast bulk-IN completes — a busy bulk-IN
   * pipe must not turn the keepalive into an H2C storm that floods the HMEBOX. */
  auto next_tick = std::chrono::steady_clock::now();
  const auto period = std::chrono::seconds(2);
  while (!_coex_stop && !g_devourer_should_stop) {
    if (!_rx_loop_active.load()) {
      /* Drain bulk-IN (empties the FW C2H buffer). 200 ms timeout bounds
       * shutdown latency when the pipe is idle. */
      int n =
          _device.bulk_read_raw(buf.data(), static_cast<int>(buf.size()), 200);
      if (n >= static_cast<int>(jaguar3::RXDESC_SIZE_8822C)) {
        ++rx;
        if (buf[11] & 0x10) /* RX desc word2 BIT(28) = C2H report */
          ++c2h;
        /* TX-only sessions get their CCX TX reports (DEVOURER_TX_REPORT) on
         * this drain — walk the buffer and emit tx.report for each
         * (src/TxReport.h); the RX loop covers TX+RX sessions. */
        if (_cfg.tx.report) {
          size_t off = 0;
          jaguar3::Rx8822cFrame f{};
          while (off + jaguar3::RXDESC_SIZE_8822C <= static_cast<size_t>(n) &&
                 jaguar3::parse_rx_8822c(buf.data() + off,
                                         static_cast<size_t>(n) - off, f)) {
            if ((buf[off + 11] & 0x10) &&
                devourer::is_ccx_halmac(f.frame, f.frame_len))
              devourer::emit_tx_report(
                  _logger->events(),
                  devourer::parse_ccx_halmac(f.frame, f.frame_len), "halmac");
            if (f.next_offset == 0)
              break;
            off += f.next_offset;
          }
        }
      }
    } else {
      /* StartRxLoop owns bulk-IN — its async URB queue sees the C2H reports
       * as part of the RX stream. Keep the loop pacing without reading. */
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (std::chrono::steady_clock::now() < next_tick)
      continue;
    next_tick += period;
    try {
      std::lock_guard<std::mutex> lk(_reg_mu);
      _hal.coex_run_5g();
      _hal.pwr_track(); /* thermal TX-power compensation (sustains upper 5 GHz) */
      /* phydm dynamic mechanisms (vendor watchdog parity): FA/CCA window
       * statistics -> DIG -> CCK-PD -> EDCCA. EDCCA tracking is owned by
       * SetCcaMode when the EDCCA-disable knob is active. */
      _phydm.tick(_channel.Channel, !_cca_disabled);
      _hal.fw_update_wl_phy_info();
      _hal.fw_set_pwr_mode_active();
      _hal.fw_coex_query_bt_info();
    } catch (...) { break; }
    if (++tick <= 3 || tick % 15 == 0)
      _logger->info("Jaguar3 coex: tick {} (bulk-IN reads={}, C2H={})", tick, rx,
                    c2h);
  }
}

namespace {

/* --- 8822E eFEM GPIO pin-function tables --------------------------------
 * Transcribed from halmac_gpio_8822e.c PINMUX_LIST_GPIO{2,3,4,13,14}_8822E /
 * PINMUX_LIST_EECS_8822E, truncated at the target function. halmac's
 * pinmux_switch_8822e walks a pin's priority list top-down: every claim
 * ABOVE the target is disabled (field := ~val & msk), the target itself is
 * enabled (field := val & msk), and the walk stops — entries below the
 * target are never touched. Byte-register RMW throughout. */
struct EfemEnt {
  uint8_t off; /* MAC byte register */
  uint8_t msk;
  uint8_t val; /* the claim's ENABLE value; disable writes ~val & msk */
};
struct EfemFix { uint8_t off; uint8_t bit; bool set; };
struct EfemFunc {
  const char *name;
  const EfemEnt *ents;
  size_t n_ents;        /* last entry is the target */
  const EfemFix *fixes; /* post-switch WL/BT ownership fixups */
  size_t n_fixes;
};

constexpr uint8_t B0 = 0x01, B1 = 0x02, B2 = 0x04, B3 = 0x08, B4 = 0x10,
                  B5 = 0x20, B6 = 0x40, B7 = 0x80;

/* RFE_CTRL_3 = GPIO3 -> WL_LNAON_SEL */
constexpr EfemEnt kEntsRfe3[] = {
    {0x66, B2, B2},      /* BT_GPIO3 */
    {0x4F, B6, B6},      /* BT_ANT_SW_3 */
    {0x41, B1, 0},       /* WL_PRI (BT_PTA) */
    {0x41, B2, B2},      /* BT_PRI (WL_PTA) */
    {0x40, B1 | B0, B0}, /* WLMAC_DBG */
    {0x40, B1 | B0, B1}, /* WLPHY_DBG */
    {0x40, B1 | B0, B1 | B0}, /* BT_DBG */
    {0x4F, B2, B2},      /* SW LNAON_SEL */
    {0x42, B0, B0},      /* BT_RFE_CTRL_5 (BT_LNAON_SEL) */
    {0x42, B0, B0},      /* RFE_CTRL_3 (WL_LNAON_SEL) — target */
};
constexpr EfemFix kFixRfe3[] = {{0x43, B7, false}, {0x67, B4, true}};

/* RFE_CTRL_5 = GPIO14 -> WLPHY_RFE_CTRL2GPIO */
constexpr EfemEnt kEntsRfe5[] = {
    {0x4E, B6, B6}, /* UART_WAKE (GPIO13_14_WL_CTRL_EN) */
    {0x4F, B6, B6}, /* BT_ANT_SW_1 */
    {0x40, B2, B2}, /* RFE_CTRL_5_4_5 — target */
};
constexpr EfemFix kFixRfe5[] = {{0x3F, B2, true}};

/* RFE_CTRL_7 = EECS pad -> WLPHY_RFE_CTRL2GPIO_2 */
constexpr EfemEnt kEntsRfe7[] = {
    {0x67, B1, B1}, /* BT_GPIO17 */
    {0x40, B3, B3}, /* RFE_CTRL_7 — target */
};

/* RFE_CTRL_8 = GPIO4 -> WL_DPDT_SEL */
constexpr EfemEnt kEntsRfe8[] = {
    {0x66, B4, B4},      /* BT_SPI_D0 */
    {0x42, B3, B3},      /* WL_SPI_D0 */
    {0x67, B0, B0},      /* BT_JTAG_TRST */
    {0x65, B7, B7},      /* WL_JTAG_TRST */
    {0x73, B3, B3},      /* DBG_GNT_WL */
    {0x40, B1 | B0, B0}, /* WLMAC_DBG */
    {0x40, B1 | B0, B1}, /* WLPHY_DBG */
    {0x40, B1 | B0, B1 | B0}, /* BT_DBG */
    {0x4E, B7, B7},      /* ANT_SWB (SW_DPDT_SEL) */
    {0x42, B1, B1},      /* BT_RFE_CTRL_0 (BT_DPDT_SEL) */
    {0x42, B1, B1},      /* WL_RFE_CTRL_8 (WL_DPDT_SEL) — target */
};
constexpr EfemFix kFixRfe8[] = {{0x43, B2, true}, {0x4F, B0, true}};

/* RFE_CTRL_9 = GPIO13 -> WL_DPDT_SEL — the DPDT transfer switch itself. */
constexpr EfemEnt kEntsRfe9[] = {
    {0x4E, B6, B6}, /* BT_WAKE (GPIO13_14_WL_CTRL_EN — 0x4c[22]) */
    {0x4F, B6, B6}, /* BT_ANT_SW_0 */
    {0x4E, B7, B7}, /* ANT_SW_GPIO13 (SW_DPDT_SEL) */
    {0x42, B1, B1}, /* BT_RFE_CTRL_1 (BT_DPDT_SEL) */
    {0x42, B1, B1}, /* WL_RFE_CTRL_9 (WL_DPDT_SEL) — target */
};
constexpr EfemFix kFixRfe9[] = {
    {0x43, B3, false}, {0x43, B4, true}, {0x4F, B0, true}};

/* RFE_CTRL_11 = GPIO2 -> WLPHY_RFE_CTRL2GPIO */
constexpr EfemEnt kEntsRfe11[] = {
    {0x66, B2, B2},      /* BT_GPIO2 */
    {0x4F, B6, B6},      /* BT_ANT_SW_2 */
    {0x41, B1, 0},       /* WL_STATE (BT_PTA) */
    {0x41, B2, B2},      /* BT_STATE (WL_PTA) */
    {0x40, B1 | B0, B0}, /* WLMAC_DBG */
    {0x40, B1 | B0, B1}, /* WLPHY_DBG */
    {0x40, B1 | B0, B1 | B0}, /* BT_DBG */
    {0x40, B2, B2},      /* RFE_CTRL_11_4_5 — target */
};
constexpr EfemFix kFixRfe11[] = {{0x3F, B0, false}};

/* Kernel order: rtw_halmac_rfe_ctrl_cfg(28..33) = RFE_CTRL_3,5,7,8,9,11. */
constexpr EfemFunc kEfemFuncs[] = {
    {"RFE_CTRL_3/GPIO3", kEntsRfe3, std::size(kEntsRfe3), kFixRfe3,
     std::size(kFixRfe3)},
    {"RFE_CTRL_5/GPIO14", kEntsRfe5, std::size(kEntsRfe5), kFixRfe5,
     std::size(kFixRfe5)},
    {"RFE_CTRL_7/EECS", kEntsRfe7, std::size(kEntsRfe7), nullptr, 0},
    {"RFE_CTRL_8/GPIO4", kEntsRfe8, std::size(kEntsRfe8), kFixRfe8,
     std::size(kFixRfe8)},
    {"RFE_CTRL_9/GPIO13", kEntsRfe9, std::size(kEntsRfe9), kFixRfe9,
     std::size(kFixRfe9)},
    {"RFE_CTRL_11/GPIO2", kEntsRfe11, std::size(kEntsRfe11), kFixRfe11,
     std::size(kFixRfe11)},
};

} /* namespace */

void RtlJaguar3Device::efem_pinmux_8822e() {
  for (const EfemFunc &f : kEfemFuncs) {
    for (size_t i = 0; i < f.n_ents; ++i) {
      const EfemEnt &e = f.ents[i];
      uint8_t v = _device.rtw_read8(e.off);
      v &= static_cast<uint8_t>(~e.msk);
      if (i + 1 == f.n_ents)
        v |= static_cast<uint8_t>(e.val & e.msk); /* enable the target */
      else
        v |= static_cast<uint8_t>(~e.val & e.msk); /* disable the claim */
      _device.rtw_write8(e.off, v);
    }
    for (size_t i = 0; i < f.n_fixes; ++i) {
      const EfemFix &x = f.fixes[i];
      uint8_t v = _device.rtw_read8(x.off);
      v = x.set ? static_cast<uint8_t>(v | x.bit)
                : static_cast<uint8_t>(v & ~x.bit);
      _device.rtw_write8(x.off, v);
    }
  }
  _logger->info("Jaguar3(8822e): eFEM pin-mux applied (RFE_CTRL 3/5/7/8/9/11 "
                "-> RFE engine; DPDT under hardware TX/RX control)");
}

/* 8822E DPDT antenna-transfer-switch routing — mode dispatch + PAD_CTRL1
 * post-coex re-assert. Called from BOTH Init (RX) and InitWrite (TX), each
 * time right after coex_wlan_only_init: coex sets GPIO_MUXCFG BT-PTA bits that
 * would mask the RFE routing, so this MUST follow it. Kernel parity — the
 * vendor runs _efem_pinmux_config from rtl8822e_init_misc in both directions.
 *
 * The eFEM (default) path replaces the improvised b5a6df7 DPDT write, which
 * took two 0x4c bits in isolation (set [24], clear [22]) and parked the DPDT
 * transfer switch in a static TX-favoring position: TX MCS4+ worked, but RX
 * path B lost its antenna on every 8812EU (chain-B pwdb pinned ~10 / -99 dBm
 * — invisible to total-frame validation, chain A masks it). The full pinmux
 * routes GPIO13 to the RFE engine's RFE_CTRL_9 (WL_DPDT_SEL) so the switch
 * follows TX/RX in hardware: PA on TX, BOTH LNAs on RX. Gated on rfe_type
 * 21..24 like the kernel (as config_rfe already does); non-eFEM boards keep
 * the reset-default pins.
 * DEVOURER_DPDT_MODE knob for A/B: efem (default) | legacy (b5a6df7 write) |
 * bit24 ([24] only) | skip. */
void RtlJaguar3Device::apply_dpdt_route_8822e() {
  if (_variant != jaguar3::ChipVariant::C8822E)
    return;
  const devourer::Dpdt8822eMode dpdt_mode = _cfg.tuning.dpdt_8822e;
  const uint8_t rfe = _hal.rfe_type();
  if (dpdt_mode == devourer::Dpdt8822eMode::EfemPinmux) {
    if (rfe >= 21 && rfe <= 24)
      efem_pinmux_8822e();
    else
      _logger->warn("Jaguar3(8822e): eFEM pin-mux SKIPPED — rfe_type={} "
                    "not in 21..24",
                    rfe);
  } else if (dpdt_mode != devourer::Dpdt8822eMode::Skip) {
    const uint32_t v4c = _device.rtw_read<uint32_t>(0x4c);
    if (dpdt_mode == devourer::Dpdt8822eMode::Bit24)
      _device.rtw_write<uint32_t>(0x4c, v4c | 0x01000000u);
    else
      _device.rtw_write<uint32_t>(0x4c, (v4c & ~0x00400000u) | 0x01000000u);
    _logger->warn("Jaguar3(8822e): DPDT legacy write mode={} "
                  "(0x4c was 0x{:08x})",
                  dpdt_mode == devourer::Dpdt8822eMode::Bit24 ? "bit24"
                                                            : "legacy",
                  v4c);
  } else {
    _logger->warn("Jaguar3(8822e): DPDT/eFEM pin-mux SKIPPED");
  }
  if (dpdt_mode != devourer::Dpdt8822eMode::Skip) {
    /* PAD_CTRL1[29:28] route the WL PAPE/antenna pads. MacInit sets both
     * (halmac pre-init), but the FW/coex bring-up steps clear bit29 —
     * re-assert post-coex (bench-validated cold-TX fix). */
    const uint32_t v64 = _device.rtw_read<uint32_t>(0x0064);
    if ((v64 & 0x30000000u) != 0x30000000u)
      _device.rtw_write<uint32_t>(0x0064, v64 | 0x30000000u);
  }
}

/* Golden-init replay — same file format as Jaguar2's ("%x %u %llx" = addr
 * width value per line, tests/decode_wseq.py output). */
void RtlJaguar3Device::apply_replay_wseq() {
  if (_cfg.debug.replay_wseq.empty())
    return;
  FILE *fp = fopen(_cfg.debug.replay_wseq.c_str(), "r");
  if (!fp) {
    _logger->error("replay_wseq: cannot open {}", _cfg.debug.replay_wseq);
    return;
  }
  unsigned addr, width;
  unsigned long long val;
  size_t n = 0;
  while (fscanf(fp, "%x %u %llx", &addr, &width, &val) == 3) {
    if (width == 1)
      _device.rtw_write8(static_cast<uint16_t>(addr),
                         static_cast<uint8_t>(val));
    else if (width == 2)
      _device.rtw_write16(static_cast<uint16_t>(addr),
                          static_cast<uint16_t>(val));
    else
      _device.rtw_write32(static_cast<uint16_t>(addr),
                          static_cast<uint32_t>(val));
    if (++n % 1000 == 0)
      _logger->info("replay_wseq: {} writes applied", n);
  }
  fclose(fp);
  _logger->info("replay_wseq: DONE — {} writes from {}", n,
                _cfg.debug.replay_wseq);
}

/* Clean shutdown — see IRtlDevice::Stop. Best-effort: a chip that already
 * dropped off the bus will make the de-init writes fail, which is fine. */
void RtlJaguar3Device::Stop() {
  _coex_stop = true;
  if (_coex_thread.joinable())
    _coex_thread.join();
  try {
    _hal.rtw_hal_deinit();
  } catch (...) {
    _logger->info("Jaguar3: Stop() de-init writes failed (chip already gone?)");
  }
}

void RtlJaguar3Device::InitWrite(SelectedChannel channel) {
  _channel = channel;
  /* Concurrent TX+RX intent (DEVOURER_TX_WITH_RX / a later StartRxLoop on this
   * bring-up): enable the RX path at the same point in the sequence Init does
   * and keep the RX filters open. Retrofitting RX state after the TX-oriented
   * bring-up completes is not reliable (validated on 8822EU: enable_rx_path
   * after the coex/FW steps leaves the RX path dead, and the register RMWs
   * race the running TX). */
  const bool want_rx = _cfg.rx.enable_with_tx;
  _rx_wanted = want_rx;
  _hal.rtw_hal_init(channel);  /* full vendor-source bring-up */
  /* 8822C at 40/80 MHz: IQK at 20 MHz, then retune — see Init. */
  const bool iqk_at_20 = _variant == jaguar3::ChipVariant::C8822C &&
                         (channel.ChannelWidth == CHANNEL_WIDTH_40 ||
                          channel.ChannelWidth == CHANNEL_WIDTH_80);
  _radioManagement.set_channel_bwmode(channel.Channel, channel.ChannelOffset,
                                      iqk_at_20 ? CHANNEL_WIDTH_20
                                                : channel.ChannelWidth);
  SelectedChannel iqk_ch = channel;
  if (iqk_at_20)
    iqk_ch.ChannelWidth = CHANNEL_WIDTH_20; /* IQK command set follows the RF */
  _hal.run_iqk(iqk_ch);
  if (iqk_at_20)
    _radioManagement.set_channel_bwmode(channel.Channel, channel.ChannelOffset,
                                        channel.ChannelWidth);
  if (want_rx)
    _hal.enable_rx_path(); /* RF into RX mode — same slot as the Init path */
  _hal.dpk_force_bypass_8822e(); /* 8822e rfe 21/22: kernel bypasses DPK (after IQK) */
  _hal.config_rfe(channel.Channel); /* 8822e RFE/PAPE antenna-switch pins (PA enable) */
  _hal.config_channel_8822e(channel.Channel); /* 8822e band TX scaling/backoff + shaping */

  /* DEVOURER_CW_TONE — a bare RF LO carrier. Armed HERE (before the FW power-mode
   * / coex H2C steps below, which on the 8812EU at 5 GHz leave the chip NAKing
   * control-IN reads that StartCwTone's BB reads need). A bare tone needs none of
   * the per-rate TX power / coex / FW-coex setup. On the 8822e the RFE PA-enable
   * pins (GPIO_MUXCFG 0x40 / PAD_CTRL 0x64) still must be driven for 5 GHz PA
   * output — but that write ALSO breaks subsequent BB reads, so we snapshot the
   * two registers first, arm the tone (reads intact), then blind-write the PA
   * pins. No coex thread (its tick would re-drive RF 0x00 back to RX). */
  if (_cfg.tx.cw_tone) {
    const uint8_t g = _cfg.tx.cw_tone_gain & 0x1F;
    const bool is_eu = _variant == jaguar3::ChipVariant::C8822E;
    uint32_t v40 = 0, v64 = 0;
    if (is_eu) {
      v40 = _device.rtw_read<uint32_t>(0x0040);
      v64 = _device.rtw_read<uint32_t>(0x0064);
    }
    for (int attempt = 1;; ++attempt) {
      try {
        StartCwTone(g);
        break;
      } catch (const std::exception &ex) {
        if (attempt >= 4) {
          _logger->error("CW tone arm failed after retries ({})", ex.what());
          break;
        }
        _logger->error("CW tone arm: USB glitch ({}) — retry {}/3", ex.what(),
                       attempt);
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
      }
    }
    if (is_eu) { /* drive the RFE PA-enable pins with blind writes (no read) */
      _device.rtw_write<uint32_t>(0x0040, v40 | 0x14030008u);
      _device.rtw_write<uint32_t>(0x0064, v64 & ~0x02040000u);
    }
    _logger->info("Jaguar3: CW tone hold (minimal bring-up, no coex thread)");
    return;
  }

  /* TXAGC from the current runtime-knob state (flat override / offset folded
   * onto the efuse-calibrated refs) — see apply_tx_power_current. Pre-coex,
   * so no _reg_mu needed here. NOTE: the FW coex/power-mode H2Cs below rewrite
   * the OFDM refs wholesale — the authoritative apply is the post-coex
   * re-apply at the end of this function; this early one just keeps the
   * intermediate bring-up steps on sane references. */
  apply_tx_power_current(/*full=*/true);
  _brought_up = true;
  /* WiFi-only coex bring-up: disable the BT/LTE antenna arbitration and lock the
   * antenna to WLAN so on-air TX is not killed by the coex firmware. */
  _hal.coex_wlan_only_init();
  /* RFE GPIO/pad pinmux — the HalMAC "Config PIN Mux" (halmac_init_8822e) that
   * devourer's hand-rolled MAC init skips: route + drive the RFE PA-enable /
   * antenna-switch control pins. Without it the 8822e's PA pins are never driven
   * and cold-start TX is ~3-5 dB weak on ALL bands (the kernel-primed chip only
   * "worked" because the kernel's HalMAC had set this and it survived rmmod).
   * MUST run AFTER coex_wlan_only_init — coex sets GPIO_MUXCFG BT-PTA bits and
   * would otherwise mask the RFE routing. Set the RFE mux bits of GPIO_MUXCFG
   * (0x40) and clear the pad-block bits of PAD_CTRL1 (0x64). Validated on-air
   * (cold, power-cycled): lifts cold 5 GHz from ~5% to ~86% duty @6 dB = kernel
   * parity. 8822e only (rfe 21-24 front-end). */
  if (_variant == jaguar3::ChipVariant::C8822E) {
    /* Raw read-modify-write: phy_set_bb_reg shifts the value by the mask's
     * trailing-zero count (contiguous-field semantics), which mangles these
     * non-contiguous multi-bit masks. */
    uint32_t v40 = _device.rtw_read<uint32_t>(0x0040);
    _device.rtw_write<uint32_t>(0x0040, v40 | 0x14030008u); /* GPIO_MUXCFG: RFE mux */
    uint32_t v64 = _device.rtw_read<uint32_t>(0x0064);
    _device.rtw_write<uint32_t>(0x0064, v64 & ~0x02040000u); /* PAD_CTRL1: RFE pads */
  }

  _hal.fw_set_pwr_mode_active(); /* keep all FW power domains on (no auto-PS) */
  _hal.fw_coex_query_bt_info();  /* make the FW confirm BT is absent */
  _hal.fw_coex_tdma_off();       /* disable coex time-division (WL keeps antenna) */
  /* DEVOURER_BF_ARM_SOUNDER=1 — beamforming self-sounding probe (beamformer
   * side): arm the MAC's hardware sounding engine so a TX-descriptor-marked
   * NDPA (DEVOURER_TX_NDPA=1) is followed by a hardware-generated NDP. The MAC
   * sounding registers are family-neutral (BeamformingSounder.h); the Jaguar-3
   * sounding-protocol control byte is 0xDB (hal_txbf_8822b_enter, shared
   * 8822B/C/E) where Jaguar-1 uses 0xCB. */
  if (_cfg.bf.arm_sounder || _cfg.bf.sounder_self_mac) {
    /* Jaguar-3 bring-up never programs the self-MAC (0x0610); the sounding
     * engine matches the injected NDPA's TA against it before firing the NDP.
     * bf.sounder_self_mac programs that MAC (use the NDPA TA the caller
     * injects — the txdemo's canonical SA); unset leaves it unprogrammed
     * (Jaguar-1 semantics). */
    if (_cfg.bf.sounder_self_mac) {
      const uint8_t *m = _cfg.bf.sounder_self_mac->data();
      for (uint16_t i = 0; i < 6; ++i)
        _device.rtw_write8(0x0610 + i, m[i]);
      _logger->info("Jaguar3 BF sounder: self-MAC programmed to "
                    "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                    m[0], m[1], m[2], m[3], m[4], m[5]);
    }
    _hal.txbf_rfmode_sounder(); /* RF/BB sounding config — needed for the NDP */
    devourer::bf::arm_sounder(_device, /*snd_ptcl_ctrl=*/0xDB);
    _logger->info("Jaguar3 BF sounder armed (beamformer side)");
  }

  /* DEVOURER_BF_TXBF=aa:bb:cc:dd:ee:ff — TX beamforming APPLY (needs the sounder
   * above + DEVOURER_TX_WITH_RX so the beamformer's RX can receive the peer's
   * report). Configure the beamformer entry for `peer_mac` so the WMAC accepts
   * and ingests the peer's Compressed Beamforming Report into the V matrix (in
   * hardware). The apply toggle is NOT flipped here — it is enabled from the RX
   * loop once a CBR from the peer is seen (a blind apply with no V degrades the
   * link). Periodic re-sounding (DEVOURER_TX_NDPA=N>1) keeps V fresh. */
  if (_cfg.bf.txbf_peer) {
    const uint8_t *m = _cfg.bf.txbf_peer->data();
    for (int i = 0; i < 6; ++i) _bf_peer[i] = m[i];
    devourer::bf::arm_beamformer_entry(_device, _bf_peer, /*rx_nss=*/2);
    _bf_txbf_armed = true;
    _logger->info("Jaguar3 BF TXBF entry configured for peer "
                  "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} — apply gates "
                  "on the first CBR (needs rx.enable_with_tx)",
                  m[0], m[1], m[2], m[3], m[4], m[5]);
  }
  /* TX-only: the bring-up enables accept-all RX (monitor_rx_cfg), but the TX
   * path never reads bulk-IN — so on a trafficked channel the on-chip RX FIFO
   * fills with unread frames and throttles TX. Close the RX filter so no
   * over-the-air frames are buffered (the coex thread still receives FW C2H
   * reports). Skipped when the caller wants concurrent RX — the RX loop will
   * drain bulk-IN. */
  if (!want_rx) {
    _device.rtw_write<uint16_t>(0x06A0, 0x0000);
    _device.rtw_write<uint16_t>(0x06A2, 0x0000);
    _device.rtw_write<uint16_t>(0x06A4, 0x0000);
    _rx_filters_closed = true; /* StartRxLoop re-opens them (best effort) */
  }
  /* DEVOURER_CW_TONE — radiate a bare RF LO carrier at the channel center (MP
   * single-tone). Armed BEFORE the coex thread starts so the arming writes don't
   * contend with the coex tick. DEVOURER_CW_TONE_GAIN=0..31 sets RF 0x00[4:0]. */
  /* Start the coex runtime thread: it drains C2H and re-applies the 5 GHz coex
   * decision every ~2 s so sustained TX isn't silenced by the FW's PTA. (The CW
   * tone path returned earlier and never reaches here.) */
  /* DEVOURER_DIS_CCA=1 — measure-first EDCCA-disable knob (see SetCcaMode).
   * Applied before the coex thread starts so the writes don't contend. */
  if (_cfg.tuning.disable_cca)
    SetCcaMode(true);
  /* DEVOURER_XTAL_CAP — crystal-cap trim (issue #217); before the coex thread
   * so the AFE write doesn't contend with the periodic coex re-apply. */
  if (_cfg.tuning.xtal_cap)
    SetXtalCap(*_cfg.tuning.xtal_cap);
  /* Re-apply TXAGC as the LAST bring-up register step: the FW power-mode /
   * coex H2Cs above reprogram the OFDM TXAGC references (0x18e8/0x41e8)
   * wholesale in firmware, clobbering the pre-coex apply — construction-time
   * TX-power state (flat override / offset) only sticks when applied after
   * them. The coex thread's ~2 s ticks do not rewrite the refs. */
  apply_tx_power_current(/*full=*/true);
  /* Per-packet power banks: the BB init table reset 0x1e70 (0x00001000, all
   * banks disabled) and may have cleared the per-STA RAM — re-sync the
   * hardware to the planner state (a pre-bring-up SetTxPacketPowerOffsetQdb
   * or a re-init with banks in use). Pre-coex-thread, so no _reg_mu needed. */
  _txpkt_ram_cleared = false;
  if (_txpkt_banks.active())
    apply_txpkt_banks_locked();
  else
    _txpkt_img.store(0, std::memory_order_relaxed);
  apply_dpdt_route_8822e(); /* 8822E DPDT/eFEM pin-mux (post-coex) */
  apply_replay_wseq(); /* DEVOURER_REPLAY_WSEQ golden-init replay (debug) */
  if (_cfg.debug.bb_dump) {
    /* Full MAC+BB dump (0x000..0x4ffc — MAC plane, then BB incl. the RF
     * direct-read windows at 0x3c00/0x4c00) in the same "BBDUMP" format as
     * Jaguar2's, for end-state diffing against the vendor kernel's rtw_proc
     * read_reg. */
    for (uint32_t a = 0x000; a <= 0x4ffc; a += 0x10)
      _logger->info("BBDUMP 0x{:04x} 0x{:08x} 0x{:08x} 0x{:08x} 0x{:08x}", a,
                    _device.rtw_read32(a), _device.rtw_read32(a + 4),
                    _device.rtw_read32(a + 8), _device.rtw_read32(a + 12));
  }
  _coex_thread = std::thread([this] { coex_runtime_loop(); });
  if (_cfg.rx.ack_responder)
    SetAckResponder(*_cfg.rx.ack_responder); /* DEVOURER_ACK_RESPONDER */
  if (_cfg.tx.ampdu)
    SetAmpduMode(*_cfg.tx.ampdu); /* DEVOURER_TX_AMPDU_MODE */
  _logger->info("Jaguar3: ready for TX (monitor inject)");
}

/* MP single-tone (CW carrier), Jaguar3 (rtl8822c / rtl8822e) path A. Ported from
 * the vendor phydm_mp_set_single_tone_jgr3 + phydm_set_pmac_txon_jgr3. Two
 * Jaguar3-specific quirks the older chips don't have:
 *   1. RF 0x00 (the RF-mode register) is WRITE-ONLY through the HSSI 3-wire port
 *      (0x1808 path A); the direct BB->RF window (0x3c00) is a read-only shadow
 *      for RF 0x00, so a window write is silently dropped. RF 0x58 (LO enable)
 *      is a normal RF reg and uses the direct window.
 *   2. The bare LO only reaches the PA once the BB/MAC TX path is keyed on via
 *      PMAC (0x1d08[0]=1) + TX-OFDM-on (0x1e70[3:0]=4).
 * The full RF 0x00 word (TX-mode 0x2 + gain, over the snapshot) is written to
 * HSSI as (addr<<20)|(data&0xFFFFF); addr=0 so it's just data&0xFFFFF. */
void RtlJaguar3Device::StartCwTone(uint8_t gain) {
  std::lock_guard<std::mutex> lk(_reg_mu);
  if (_cw_active)
    return;

  constexpr uint16_t HSSI_WR_A = 0x1808;         /* path-A HSSI RF-write port */
  const uint16_t rf58 = 0x3c00 + (0x58 << 2);    /* RF 0x58 via direct window */
  const bool is_2g = _channel.Channel <= 14;

  /* Disable CCA so the bare carrier isn't gated by energy detection. (Pausing
   * MAC TX / disabling CCK TX here was tried and BREAKS the RF 0x00 HSSI write —
   * it re-drives RF 0x00 back to RX mode — so it's deliberately omitted.) */
  if (is_2g) {
    _device.phy_set_bb_reg(0x1a9c, 1u << 20, 0x0);
    _device.phy_set_bb_reg(0x1a14, 0x300, 0x3);
  }
  _device.phy_set_bb_reg(0x1d58, 0xff8, 0x1ff); /* OFDM CCA off */

  /* Key the BB/MAC TX path: PMAC on + TX OFDM on (phydm_set_pmac_txon_jgr3). */
  _device.phy_set_bb_reg(0x1d08, 1u << 0, 0x1);
  _device.phy_set_bb_reg(0x1e70, 0xf, 0x0);
  _device.phy_set_bb_reg(0x1e70, 0xf, 0x4);

  /* Hold the BB in continuous TX (phydm_start_ofdm_cont_tx_jgr3). Without this
   * the BB stays in RX and its RF state machine re-drives RF 0x00 back to RX
   * mode within ~ms, so the manual TX-mode write below doesn't persist and no
   * carrier comes out. */
  _device.phy_set_bb_reg(0x1c3c, 1u << 0, 0x1); /* OFDM block on */
  _device.phy_set_bb_reg(0x1a00, 0x3, 0x0);     /* CCK test mode off */
  _device.phy_set_bb_reg(0x1a00, 1u << 3, 0x1); /* scramble on */
  _device.phy_set_bb_reg(0x1ca4, 0x7, 0x1);     /* continuous TX on */

  /* Snapshot RF 0x00 (read the direct-window shadow — the correct read path). */
  _cw_rf00 = _device.rtw_read<uint32_t>(0x3c00) & 0x000fffff;

  /* RF 0x00 -> TX mode (0x00[19:16]=2) + gain (0x00[4:0]) via the HSSI port. */
  const uint32_t rf0 =
      (_cw_rf00 & ~0xf0000u & ~0x1fu) | (0x2u << 16) | (gain & 0x1f);
  _device.rtw_write<uint32_t>(HSSI_WR_A, rf0 & 0x000fffff);

  /* RF LO enable (0x58 bit1) via the direct window. */
  _device.phy_set_bb_reg(rf58, 1u << 1, 0x1);

  _cw_active = true;
  _logger->info("CW single-tone armed @ ch{} gain={} ({})", _channel.Channel,
                static_cast<int>(gain & 0x1f),
                _variant == jaguar3::ChipVariant::C8822E ? "8822e" : "8822c");
}

/* Mirror of the vendor STOP path: disable the LO, restore RF 0x00 (HSSI),
 * un-key the TX path, re-enable CCA. Serializes on _reg_mu (runs while the coex
 * thread is active). */
void RtlJaguar3Device::StopCwTone() {
  std::lock_guard<std::mutex> lk(_reg_mu);
  if (!_cw_active)
    return;

  constexpr uint16_t HSSI_WR_A = 0x1808;
  const uint16_t rf58 = 0x3c00 + (0x58 << 2);
  const bool is_2g = _channel.Channel <= 14;

  _device.phy_set_bb_reg(rf58, 1u << 1, 0x0);                 /* LO off */
  _device.rtw_write<uint32_t>(HSSI_WR_A, _cw_rf00 & 0x000fffff); /* restore RF 0x00 */
  /* Stop continuous TX (phydm_stop_ofdm_cont_tx_jgr3): test modes off, then BB
   * reset. */
  _device.phy_set_bb_reg(0x1ca4, 0x7, 0x0);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  _device.phy_set_bb_reg(0x1d0c, 1u << 16, 0x0);
  _device.phy_set_bb_reg(0x1d0c, 1u << 16, 0x1);
  _device.phy_set_bb_reg(0x1e70, 0xf, 0x0);     /* TX OFDM off */
  _device.phy_set_bb_reg(0x1d08, 1u << 0, 0x0); /* PMAC off */
  _device.phy_set_bb_reg(0x1d58, 0xff8, 0x0);   /* OFDM CCA on */
  if (is_2g) {
    _device.phy_set_bb_reg(0x1a9c, 1u << 20, 0x1);
    _device.phy_set_bb_reg(0x1a14, 0x300, 0x0);
  }

  _cw_active = false;
  _logger->info("CW single-tone stopped — chip restored");
}

/* Modulated continuous TX (Jaguar3). The JGR3 hardware-continuous hold
 * (0x1ca4) only engages under the full PMAC packet-generator setup, which emits
 * a test pattern rather than decodable frames — no good for a link probe that
 * needs per-frame SNR/EVM. So the HW 100%-duty path is NOT engaged here; we
 * apply the rate and let the caller's back-to-back send_packet loop supply the
 * modulated stimulus at beacon duty. True HW continuous is a documented
 * follow-up (needs the PMAC TX-packet config). */
void RtlJaguar3Device::StartContinuousTx(const devourer::TxMode &mode) {
  SetTxMode(mode);
  std::lock_guard<std::mutex> lk(_reg_mu);
  if (_cont_active)
    return;

  /* --- Stop the normal TRX (phydm_stop_ic_trx) so the PMAC can drive TX --- */
  _cont_txpause = _device.rtw_read8(0x522);        /* save TX-queue bitmap */
  _device.rtw_write8(0x522, 0xff);                 /* pause all TX queues */
  _device.phy_set_bb_reg(0x1d58, 0xff8, 0x1ff);    /* disable OFDM RX CCA */
  _cont_ccktx = _device.rtw_read<uint32_t>(0x1a04) & 0xf0000000u; /* save */
  _device.phy_set_bb_reg(0x1a14, 0x300, 0x3);      /* CCK RxIQ weight = 0 */
  _device.phy_set_bb_reg(0x1a04, 0xf0000000, 0x0); /* disable CCK Tx */

  /* --- Start OFDM continuous TX (phydm_start_ofdm_cont_tx_jgr3) --- */
  _device.phy_set_bb_reg(0x1c3c, 1u << 0, 0x1);    /* OFDM block on */
  _device.phy_set_bb_reg(0x1a00, 0x3, 0x0);        /* CCK test mode off */
  _device.phy_set_bb_reg(0x1a00, 1u << 3, 0x1);    /* scrambler on */
  _device.phy_set_bb_reg(0x1ca4, 0x7, 0x1);        /* continuous TX hold on */

  /* --- Define the PMAC packet: legacy 6M OFDM (phydm_set_sig_jgr3 +
   * phydm_set_mac_phy_txinfo_jgr3). L-SIG bytes {0x0b,0x7d,0x02} = rate 6M,
   * length 1000 B, parity — the vendor's hardcoded 6M packet. --- */
  _device.phy_set_bb_reg(0x1eb4, 0xfffff, 0x1);    /* packet_count = 1 */
  _device.phy_set_bb_reg(0x908, 0xffffff, 0x00027d0b); /* L-SIG (6M) */
  _device.phy_set_bb_reg(0xa58, 0x003f8000, 0x04); /* tx_rate = ODM_RATE6M */
  _device.phy_set_bb_reg(0x900, 1u << 1, 0x0);     /* ndp_sound = 0 */
  _device.phy_set_bb_reg(0x900, 0xff000000, 0x0);  /* txsc/bw/stbc = 0 (20 MHz) */
  _device.phy_set_bb_reg(0x1ae0, 0x7000, 0x0);     /* tx_sc = duplicate */
  _device.phy_set_bb_reg(0x900, 1u << 0, 0x0);     /* not HT */
  _device.phy_set_bb_reg(0x900, 1u << 2, 0x0);     /* not VHT (legacy) */
  _device.phy_set_bb_reg(0x9b8, 0xffff0000, 2000); /* packet period ~500us */

  /* --- Turn on PMAC + TX-OFDM (phydm_set_pmac_txon_jgr3) --- */
  _device.phy_set_bb_reg(0x1d08, 1u << 0, 0x1);    /* PMAC on */
  _device.phy_set_bb_reg(0x1e70, 0xf, 0x0);
  _device.phy_set_bb_reg(0x1e70, 0xf, 0x4);        /* TX OFDM on */

  _cont_active = true;
  _logger->info("Modulated continuous TX armed @ ch{} (Jaguar3 PMAC 6M HW "
                "100%%-duty carrier; idle-hold, StopContinuousTx to end)",
                _channel.Channel);
}

void RtlJaguar3Device::StopContinuousTx() {
  std::lock_guard<std::mutex> lk(_reg_mu);
  if (!_cont_active)
    return;

  /* Stop OFDM continuous (phydm_stop_ofdm_cont_tx_jgr3). */
  _device.phy_set_bb_reg(0x1ca4, 0x7, 0x0);        /* continuous hold off */
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  _device.phy_set_bb_reg(0x1d0c, 1u << 16, 0x0);   /* BB reset pulse */
  _device.phy_set_bb_reg(0x1d0c, 1u << 16, 0x1);
  _device.phy_set_bb_reg(0x1e70, 0xf, 0x0);        /* TX OFDM off */
  _device.phy_set_bb_reg(0x1d08, 1u << 0, 0x0);    /* PMAC off */

  /* Revert the TRX stop (phydm_stop_ic_trx REVERT). */
  _device.rtw_write8(0x522, _cont_txpause);        /* release TX queues */
  _device.phy_set_bb_reg(0x1d58, 0xff8, 0x0);      /* re-enable OFDM RX CCA */
  _device.phy_set_bb_reg(0x1a14, 0x300, 0x0);      /* restore CCK RxIQ */
  _device.phy_set_bb_reg(0x1a04, 0xf0000000, _cont_ccktx); /* restore CCK Tx */

  _cont_active = false;
  _logger->info("Modulated continuous TX stopped — chip restored");
}

/* Frame-free RX energy snapshot for the Jaguar3 (8822C/E) BB. Ported from
 * phydm_fa_cnt_statistics_jgr3 + phydm_reset_bb_hw_cnt (reference/rtl88x2cu
 * phydm_dig.c / phydm_api.c): the newer BB holds the same FA/CCA facilities as
 * the AC classic map, just at different addresses. CCA (0x2c08) is the primary
 * channel-busy signal (a CW tone spikes OFDM CCA); OFDM FA is the vendor sum of
 * the sub-counters. Read-then-reset for a per-call delta; serialized on _reg_mu
 * so it does not race the coex thread's register access. */
RxEnergy RtlJaguar3Device::GetRxEnergy() {
  std::lock_guard<std::mutex> lk(_reg_mu);
  RxEnergy e;
  auto rd = [this](uint16_t addr) { return _device.rtw_read<uint32_t>(addr); };

  const uint32_t cca = rd(0x2c08);
  e.cca_ofdm = (cca >> 16) & 0xffff;
  e.cca_cck = cca & 0xffff;
  e.fa_cck = rd(0x1a5c) & 0xffff;
  /* OFDM FA = parity + rate-illegal + crc8 + mcs + fast-fsync + sb-search +
   * mcs-vht + crc8-vhta (phydm_fa_cnt_statistics_jgr3). */
  const uint32_t r2d04 = rd(0x2d04), r2d08 = rd(0x2d08), r2d10 = rd(0x2d10),
                 r2d20 = rd(0x2d20), r2d0c = rd(0x2d0c);
  e.fa_ofdm = ((r2d04 >> 16) & 0xffff) + (r2d08 & 0xffff) +
              ((r2d08 >> 16) & 0xffff) + (r2d10 & 0xffff) + (r2d20 & 0xffff) +
              ((r2d20 >> 16) & 0xffff) + ((r2d10 >> 16) & 0xffff) +
              (r2d0c & 0xffff);
  e.valid_fa = true;
  e.igi = static_cast<uint8_t>(rd(0x1d70) & 0x7f);
  e.valid_igi = true;

  /* NHM 12-bucket power histogram (frame-free, JGR3 register map). Runs its own
   * ~2 ms measurement window before the FA-counter reset below (0x1eb4[25] also
   * clears BB HW counters). Holds _reg_mu across the short wait — tolerable at
   * the emitter's >=100 ms cadence vs the coex thread's ~2 s tick. */
  devourer::read_nhm(
      devourer::nhm_regs_jgr3(), e.igi, rd,
      [this](uint16_t a, uint32_t m, uint32_t v) {
        _device.phy_set_bb_reg(a, m, v);
      },
      e);

  /* Reset: CCK FA 0x1a2c[15:14] 0->2, CCK CCA 0x1a2c[13:12] 0->2, then OFDM
   * CCA/FA (phydm_reset_bb_hw_cnt jgr3: 0x1eb4[25] 1->0, wrapped by the
   * 0x1d2c[31] rx-clk-gate disable/enable). */
  _device.phy_set_bb_reg(0x1a2c, 0x3u << 14, 0x0);
  _device.phy_set_bb_reg(0x1a2c, 0x3u << 14, 0x2);
  _device.phy_set_bb_reg(0x1a2c, 0x3u << 12, 0x0);
  _device.phy_set_bb_reg(0x1a2c, 0x3u << 12, 0x2);
  _device.phy_set_bb_reg(0x1d2c, 1u << 31, 0x0);
  _device.phy_set_bb_reg(0x1eb4, 1u << 25, 0x1);
  _device.phy_set_bb_reg(0x1eb4, 1u << 25, 0x0);
  _device.phy_set_bb_reg(0x1d2c, 1u << 31, 0x1);

  /* No active absolute noise floor on Jaguar3 (#202): the vendor 8822C driver
   * has no idle-noise path (phydm_noisemonitor.c dispatches the report only to
   * 8822B/8821C and returns 0 for the 8822C), so e.valid_noise_floor stays
   * false. The passive rssi-snr floor (RxQuality.noise_floor_dbm) is J3's only
   * floor. */
  return e;
}

/* Disable / restore the MAC carrier-sense gate that defers TX. Two 0x520 bits:
 * BIT_DIS_CCA 0x520[14] (primary carrier-sense of a decodable preamble) and
 * BIT_DIS_EDCCA 0x520[15] (energy detect), plus BIT_EDCCA_MSK_COUNTDOWN
 * 0x524[11]. Caller holds _reg_mu.
 *
 * The primary-CCA bit [14] is the one that matters for the host-injected data
 * path: monitor injection is NOT CCA-free — it defers to a co-channel 802.11
 * transmitter and drops ~40% of its submit rate (on-air, 8822EU, co-channel
 * flooder vs a far-channel USB-contention control; tests/dis_cca_tx_onair.sh).
 * Clearing [14] removes that deferral (~1.5x recovery, back to ~90% of the
 * unimpeded rate) so injected/beacon TX punches through a busy channel — the
 * "keep transmitting through interference" lever the OpenIPC-FPV community wants
 * (issue #199). The energy bit [15] alone is null against a decodable preamble;
 * it is kept because it also relaxes deferral to non-802.11 in-band energy.
 *
 * The vendor dis_cca recipe ALSO writes three BB registers (0x1a9c[20],
 * 0x1a14[9:8], 0x1d58[0xff8]); those are deliberately NOT done here.
 * 0x1d58[0xff8]=0x1ff is the OFDM-CCA-off write (the CW-tone path uses it to stop
 * OFDM detection for a bare carrier), so applying it makes the RX deaf to OFDM —
 * MEASURED: the full recipe dropped 8822EU delivery from ~6800 to ~10 frames.
 * These MAC 0x520/0x524 bits gate TX only and are safe on a live RX. */
void RtlJaguar3Device::apply_cca_mode_locked(bool disabled) {
  uint32_t v520 = _device.rtw_read<uint32_t>(0x0520);
  uint32_t v524 = _device.rtw_read<uint32_t>(0x0524);
  if (disabled) {
    v520 |= (1u << 15) | (1u << 14);   /* DIS_EDCCA (energy) + DIS_CCA (carrier-sense) */
    v524 &= ~(1u << 11);
  } else {
    v520 &= ~((1u << 15) | (1u << 14));
    v524 |= (1u << 11);
  }
  _device.rtw_write<uint32_t>(0x0520, v520);
  _device.rtw_write<uint32_t>(0x0524, v524);
}

void RtlJaguar3Device::SetCcaMode(bool disabled) {
  std::lock_guard<std::mutex> lk(_reg_mu);
  _cca_disabled = disabled;
  if (_brought_up)
    apply_cca_mode_locked(disabled);
  _logger->info("Jaguar3: MAC carrier-sense {}",
                disabled ? "DISABLED (dis_cca: CCA+EDCCA)" : "enabled (default)");
}

void RtlJaguar3Device::SetMonitorChannel(SelectedChannel channel) {
  _phydm.on_channel_change();
  /* Serialize against the coex thread's housekeeping tick (and any concurrent
   * FastRetune) — channel config is register RMW. Init/InitWrite call the
   * radio-management core directly (no lock needed: the coex thread isn't
   * running yet), so locking here cannot self-deadlock. */
  std::lock_guard<std::mutex> lk(_reg_mu);
  const bool ch_changed = channel.Channel != _channel.Channel;
  _channel = channel;
  _radioManagement.set_channel_bwmode(channel.Channel, channel.ChannelOffset,
                                      channel.ChannelWidth);
  /* Runtime TX-power knobs in use: re-fold them against the NEW channel
   * group's efuse refs (8822E bases are per-group). Gated on a knob being
   * active so the legacy no-knob path stays byte-identical. */
  if (ch_changed)
    _pwr_ref_valid = false;
  if (_brought_up && (_tx_pwr_offset_steps != 0 || _tx_pwr_override >= 0))
    apply_tx_power_current(/*full=*/true);
  /* dis_cca is sticky — the channel set rewrote the BB CCA registers, so
   * re-assert the disable if it was armed. */
  if (_brought_up && _cca_disabled)
    apply_cca_mode_locked(true);
  /* Per-packet power banks are sticky too (the lever contract): the channel
   * set doesn't touch 0x1e70[31:16] today, but a cheap RMW re-assert keeps
   * the contract robust against future channel-path changes. */
  if (_brought_up && _txpkt_banks.active())
    apply_txpkt_banks_locked();
}

void RtlJaguar3Device::FastRetune(uint8_t channel, bool cache_rf) {
  std::lock_guard<std::mutex> lk(_reg_mu);
  if (channel == _channel.Channel)
    return;
  if (_radioManagement.fast_retune(channel, _channel.ChannelOffset,
                                   _channel.ChannelWidth, cache_rf)) {
    _channel.Channel = channel;
    return;
  }
  /* Fast path declined (band change / never tuned) — full channel set at the
   * current bandwidth + offset, under the same lock (the core is unlocked). */
  _channel.Channel = channel;
  _radioManagement.set_channel_bwmode(channel, _channel.ChannelOffset,
                                      _channel.ChannelWidth);
}

void RtlJaguar3Device::FastSetBandwidth(ChannelWidth_t bw) {
  std::lock_guard<std::mutex> lk(_reg_mu);
  if (bw == _channel.ChannelWidth)
    return;
  auto in_set = [](ChannelWidth_t b) {
    return b == CHANNEL_WIDTH_20 || b == CHANNEL_WIDTH_5 ||
           b == CHANNEL_WIDTH_10;
  };
  /* Only a same-channel toggle within {20, 5, 10} — a 40/80 endpoint moves the
   * RF bandwidth, needing the full tune. */
  if (in_set(bw) && in_set(_channel.ChannelWidth) &&
      _radioManagement.fast_set_bandwidth(bw)) {
    _channel.ChannelWidth = bw;
    return;
  }
  /* Fast path declined (40/80 endpoint, cold radio) — full channel set, under
   * the same lock (the core is unlocked). */
  _radioManagement.set_channel_bwmode(_channel.Channel, _channel.ChannelOffset,
                                      bw);
  _channel.ChannelWidth = bw;
}

/* Re-program TXAGC from the current knob state (see header). Both TXAGC
 * paths are written in every mode, TX+RX included — with the DPDT/pad
 * pin-mux applied at bring-up, a live 0x41e8 write does not disturb the
 * EU's RX (tests/eu_41e8_desense_recheck.sh). */
void RtlJaguar3Device::apply_tx_power_current(bool full) {
  const int off = _tx_pwr_offset_steps;
  const int flat = _tx_pwr_override;
  _txpwr_sat_low = false;
  _txpwr_sat_high = false;
  auto clamp127 = [&](int v) -> uint8_t {
    if (v < 0) {
      v = 0;
      _txpwr_sat_low = true;
    }
    if (v > 0x7f) {
      v = 0x7f;
      _txpwr_sat_high = true;
    }
    return static_cast<uint8_t>(v);
  };

  if (flat >= 0) {
    /* Flat semantics: reference = flat + offset, per-rate diffs zeroed (once —
     * offset-only steps skip the 32-dword re-zero). */
    _radioManagement.set_tx_power_ref(clamp127(flat + off),
                                      /*zero_diffs=*/!_diffs_zeroed);
    _diffs_zeroed = true;
    return;
  }

  if (_variant == jaguar3::ChipVariant::C8822E) {
    /* Default TX power (8822e). The TXAGC reference base is the per-channel
     * power index the kernel programs, from the efuse power-by-rate table
     * (logical 0x22 path A / 0x4C path B, per channel-group); when the byte is
     * valid (<= txgi_max 127) it is used verbatim for kernel-faithful
     * per-channel power. On bare modules the byte is unprogrammed (0xFF) and
     * devourer falls back to JAGUAR3_TXPWR_REF_BASE — an empirically
     * on-air-matched default that is not below the kernel's. The per-rate
     * diffs come from the BB phy_reg_pg table (apply_power_by_rate_8822e);
     * full TSSI-offset power-by-rate is a follow-up. The derived base refs
     * are cached so an offset-only step is just the light ref writes. */
    if (full || !_pwr_ref_valid) {
      uint8_t efuse_a = 0xFF, efuse_b = 0xFF;
      _hal.read_efuse_txpwr_base_8822e(_channel.Channel, efuse_a, efuse_b);
      _pwr_ref_a = (efuse_a <= 127) ? efuse_a : JAGUAR3_TXPWR_REF_BASE;
      _pwr_ref_b = (efuse_b <= 127) ? efuse_b : JAGUAR3_TXPWR_REF_BASE;
      _pwr_ref_valid = true;
    }
    const uint8_t ra = clamp127(static_cast<int>(_pwr_ref_a) + off);
    const uint8_t rb = clamp127(static_cast<int>(_pwr_ref_b) + off);
    if (full || _diffs_zeroed) {
      /* Full apply: refs + the per-rate diff walk (also the path back from
       * flat semantics, which zeroed the diffs). */
      _radioManagement.apply_power_by_rate_8822e(_channel.Channel, ra, rb);
      _diffs_zeroed = false;
    } else {
      _radioManagement.apply_tx_power_refs_8822e(ra, rb);
    }
    return;
  }

  /* 8822c (CU): preserve the flat reference the demo used to impose (40) so
   * the SDR-validated CU TX power is unchanged. The CU's efuse-calibrated
   * per-rate default is a follow-up; the offset shifts this reference. */
  _radioManagement.set_tx_power_ref(
      clamp127(static_cast<int>(JAGUAR3_TXPWR_REF_BASE_8822C) + off),
      /*zero_diffs=*/!_diffs_zeroed);
  _diffs_zeroed = true;
}

devourer::TxPowerCaps RtlJaguar3Device::GetTxPowerCaps() {
  devourer::TxPowerCaps caps;
  caps.supported = true;
  caps.index_max = 127;
  caps.step_qdb = 1; /* 0.25 dB per reference step */
  /* On-air slope (tests/txpwr_offset_onair.sh): 8822CU 0.325 dB/idx @ ch36
   * (classifies at the 0.25 nominal, rms 0.60). The 8822EU measures ~0.55
   * dB/idx with an accelerating curve (0.3 low range -> 0.9 around ref 72) —
   * its TSSI/kfree trims reshape the ref->power transfer, so one reference
   * step is NOT a constant 0.25 dB there and the measured flag stays false:
   * a controller should calibrate its own dB-per-step on the E (or lean on
   * GetTxPowerState readback + the ground's RSSI). */
  caps.step_measured = _variant == jaguar3::ChipVariant::C8822C;
  caps.offset_min_qdb = -127;
  caps.offset_max_qdb = 127;
  return caps;
}

int RtlJaguar3Device::SetTxPowerOffsetQdb(int qdb) {
  if (_cw_active) {
    _logger->warn("SetTxPowerOffsetQdb refused: CW tone active (TXAGC does "
                  "not modulate a bare LO carrier)");
    return 0;
  }
  int steps = 0;
  const int applied = devourer::quantize_offset_qdb(qdb, GetTxPowerCaps(),
                                                    &steps);
  _tx_pwr_offset_steps = steps;
  if (_brought_up) {
    std::lock_guard<std::mutex> lk(_reg_mu);
    apply_tx_power_current(/*full=*/false);
  }
  _logger->info("TX-power offset: {} qdB requested -> {} qdB applied "
                "({} steps){}",
                qdb, applied, steps,
                _brought_up ? "" : " (recorded; applies at bring-up)");
  return applied;
}

void RtlJaguar3Device::SetTxPowerIndexOverride(int idx) {
  if (_cw_active) {
    _logger->warn("SetTxPowerIndexOverride refused: CW tone active");
    return;
  }
  _tx_pwr_override = idx < 0 ? -1 : (idx > 0x7f ? 0x7f : idx);
  if (_brought_up) {
    std::lock_guard<std::mutex> lk(_reg_mu);
    /* Clearing back to the efuse baseline needs the full apply (the flat
     * override zeroed the 8822E per-rate diffs). */
    apply_tx_power_current(/*full=*/idx < 0);
  }
}

bool RtlJaguar3Device::ReApplyTxPower() {
  if (!_brought_up || _cw_active)
    return false;
  std::lock_guard<std::mutex> lk(_reg_mu);
  apply_tx_power_current(/*full=*/true);
  return true;
}

/* --- Per-packet TX-power banks (see TxPktPwrBanks.h) --- */

int RtlJaguar3Device::txpkt_idx_for_qdb(int qdb) const {
  /* Round to the nearest bank step (ties away from zero), like
   * quantize_offset_qdb. Step from cfg (default 4 qdB = 1 dB — the vendor's
   * "each tx_pwr_ofst step will be 1dB" for the 22C class; the on-air slope
   * run pins it per variant). */
  const int step = _cfg.tuning.txpkt_step_qdb > 0 ? _cfg.tuning.txpkt_step_qdb : 4;
  const int num = qdb >= 0 ? qdb * 2 + step : qdb * 2 - step;
  return num / (2 * step);
}

void RtlJaguar3Device::apply_txpkt_banks_locked() {
  if (!_txpkt_ram_cleared) {
    /* One-time defensive clear of the macid-1 per-STA BB-RAM offsets (types
     * 0/1): descriptor MACID is 0x01, and stale RAM content (a prior driver /
     * killed session) would silently offset every type-0 "baseline" frame.
     * Vendor 0x1e84 write protocol (phydm_wt_ram_pwr / phydm_rst_ram_pwr):
     * all offset+en+hwigi fields zero, macid[29:24], write_en(30); then a
     * read_en(31) pulse latches; then release. */
    _device.rtw_write<uint32_t>(0x1e84, ((1u & 0x3f) << 24) | (1u << 30));
    _device.rtw_write<uint32_t>(0x1e84, 0x80000000u);
    _device.rtw_write<uint32_t>(0x1e84, 0x00000000u);
    _txpkt_ram_cleared = true;
  }
  /* Both global banks live in one dword: 0x1e70[23:16] = en|idx (type 2),
   * [31:24] = en|idx (type 3). Offsets are 7-bit two's-complement power-index
   * steps. Plain BB RMW — the 0x1c90[15] gate applies to TXAGC-table writes
   * only (vendor programs 0x1e70 with a bare odm_set_bb_reg). */
  const uint16_t img = _txpkt_banks.reg16();
  _device.phy_set_bb_reg(0x1e70, 0xFFFF0000u, img);
  _txpkt_img.store(img, std::memory_order_relaxed);
}

uint8_t RtlJaguar3Device::txpkt_type_for_idx(int idx) {
  idx = jaguar3::TxPktPwrBankPlanner::clamp_idx(idx);
  if (idx == 0)
    return 0;
  /* Lock-free hit check against the committed 0x1e70 image (one relaxed
   * load): the common case for a UEP stream cycling 1-2 offset levels. */
  const uint16_t img = _txpkt_img.load(std::memory_order_relaxed);
  const uint8_t want =
      static_cast<uint8_t>(0x80 | (static_cast<uint8_t>(idx) & 0x7f));
  if ((img & 0xff) == want)
    return 2;
  if ((img >> 8) == want)
    return 3;
  /* Miss: allocate/evict a bank and program it. Serialized on _reg_mu against
   * the coex tick and channel retunes; unconditional re-apply also heals a
   * stale mirror (e.g. a pre-bring-up recorded default). */
  std::lock_guard<std::mutex> lk(_reg_mu);
  const auto plan = _txpkt_banks.request(idx);
  apply_txpkt_banks_locked();
  return plan.type;
}

int RtlJaguar3Device::SetTxPacketPowerOffsetQdb(int qdb) {
  const int step = _cfg.tuning.txpkt_step_qdb > 0 ? _cfg.tuning.txpkt_step_qdb : 4;
  const int idx =
      jaguar3::TxPktPwrBankPlanner::clamp_idx(txpkt_idx_for_qdb(qdb));
  uint8_t type = 0;
  {
    std::lock_guard<std::mutex> lk(_reg_mu);
    if (idx != 0) {
      const auto plan = _txpkt_banks.request(idx);
      type = plan.type;
      if (_brought_up)
        apply_txpkt_banks_locked();
    }
  }
  _txpkt_dflt_idx.store(idx, std::memory_order_relaxed);
  _txpkt_dflt_type.store(type, std::memory_order_relaxed);
  const int applied = idx * step;
  _logger->info("per-packet TX-power default: {} qdB requested -> {} qdB "
                "(bank idx {}, type {}){}",
                qdb, applied, idx, type,
                _brought_up ? "" : " (recorded; applies at bring-up)");
  return applied;
}

devourer::TxCaps RtlJaguar3Device::GetTxCaps() {
  return devourer::tx_caps_for_chains(2); /* 8822C/8822E are 2T2R */
}

int RtlJaguar3Device::SetXtalCap(int cap) {
  /* phydm_set_crystal_cap_reg (8822C/8822E): a 7-bit code into
   * 0x1040[23:17] AND 0x1040[16:10] (Xo and Xi), reg = cap | (cap << 7).
   * cap < 0 reverts to the default (efuse readback is the 2-byte 8822C
   * format, not wired here — fall back to 0x20). */
  const uint8_t c = cap < 0 ? 0x20 : static_cast<uint8_t>(cap & 0x7F);
  const uint32_t reg = static_cast<uint32_t>(c) | (static_cast<uint32_t>(c) << 7);
  std::lock_guard<std::mutex> lk(_reg_mu);
  _device.phy_set_bb_reg(0x1040, 0x00FFFC00, reg);
  _xtal_cap = c;
  _logger->info("Jaguar3: crystal-cap set to 0x{:02x}{}", c,
                cap < 0 ? " (default)" : "");
  return c;
}

devourer::AdapterCaps RtlJaguar3Device::GetAdapterCaps() {
  devourer::AdapterCaps c;
  c.supported = true;
  c.generation = devourer::ChipGeneration::Jaguar3;
  c.transport = _device.is_usb() ? "usb" : "pcie";
  c.tx = GetTxCaps();
  c.txpwr = GetTxPowerCaps();
  c.tx_chains = 2; /* 8822C/8822E are 2T2R */
  c.rx_chains = 2;
  c.per_chain_rssi = true;
  /* Per-packet TX power: the TXPWR_OFSET_TYPE bank selector + programmable
   * 0x1e70 offset banks (SetTxPacketPowerOffsetQdb / radiotap DBM_TX_POWER;
   * TxPktPwrBanks.h). Continuous in step_qdb units, ±63/-64 index travel, 2
   * concurrent non-zero levels. On-air-measured on BOTH variants
   * (tests/txpkt_pwr_ofset_onair.sh, ch36 MCS3, chip-RSSI ground): 8822CU
   * 5/5 LUT-mapped steps + 2/2 radiotap cells sign-correct (+6 nominal ->
   * +4..+6 raw, -11 -> -13..-14); 8822EU 5/5 + 2/2 (+6 -> +8 raw; deep cuts
   * compress: -7 and -11 both land ~-6 raw — its TSSI/kfree transfer, same
   * caveat as its SetTxPowerOffsetQdb slope). Banks survive
   * SetMonitorChannel + FastRetune (tests/txpkt_hop_persist.sh: +6 dB held
   * across 36<->40 hops). */
  {
    const int step =
        _cfg.tuning.txpkt_step_qdb > 0 ? _cfg.tuning.txpkt_step_qdb : 4;
    c.per_packet_txpower = true;
    c.per_pkt_txpwr_steps = 0; /* continuous (bank-programmable) */
    c.per_pkt_txpwr_step_qdb = static_cast<uint8_t>(step);
    c.per_pkt_txpwr_min_qdb = static_cast<int16_t>(-64 * step);
    c.per_pkt_txpwr_max_qdb = static_cast<int16_t>(63 * step);
    c.per_pkt_txpwr_measured = true;
  }
  c.bw_mask = devourer::bw_mask_for_generation(c.generation);
  c.fastretune_ok = true;
  c.narrowband_ok = true; /* 5/10 MHz baseband re-clock — Jaguar3 only */
  c.hw_rx_timestamp = true;  /* FrameParserJaguar3 fills RxAtrib.tsfl */
  c.hw_beacon_txtsf = true;  /* StartBeacon: MAC inserts the egress TSF into beacons */
  c.xtal_cap_max = 0x7f;   /* 7-bit AFE crystal-cap trim (0x1040) */
  c.xtal_cap_default = 0x20;
  /* LDPC RX: both variants decode HT+VHT LDPC (bench: encoding-matrix
   * devourer↔devourer cells at full delivery, 8812CU + 8812EU-paired 8812AU
   * cross-checked reporting ldpc=1) and report it per-frame from PHY-status
   * byte7[5] (parse_phy_sts_jgr3). */
  c.ldpc_rx_ht = true;
  c.ldpc_rx_vht = true;
  c.ldpc_rx_flag = true;
  devourer::set_standard_freq_ranges(c);

  if (_variant == jaguar3::ChipVariant::C8822E) {
    c.chip_name = "RTL8822E";
    c.marketing_names = "RTL8812EU/RTL8822EU";
    c.chip_id = 0x17;
    c.variant = "C8822E";
  } else {
    c.chip_name = "RTL8822C";
    c.marketing_names = "RTL8812CU/RTL8822CU";
    c.chip_id = 0x13;
    c.variant = "C8822C";
  }
  return c;
}

devourer::EfuseStability RtlJaguar3Device::ProbeEfuseStability(int reads) {
  if (!_brought_up)
    return {}; /* supported=false — the efuse read needs a powered chip */
  std::lock_guard<std::mutex> lk(_reg_mu); /* serialize vs the coex tick */
  constexpr uint16_t kMapLen = 0x100;      /* sizeof(HalJaguar3::_efuse_cache) */
  auto st = devourer::ProbeEfuseStabilityImpl(
      [this](uint8_t *buf) { return _hal.probe_efuse_map(buf, kMapLen); },
      kMapLen, reads);
  if (st.reads == 0)
    return {}; /* 8822E: probe refused (OTP unreliable post-bring-up) */
  _logger->info(
      "efuse-stability: reads={} mismatched={} invalid_id={} id=0x{:04x}",
      st.reads, st.mismatched_reads, st.invalid_id_reads, st.eeprom_id);
  return st;
}

devourer::LaResult RtlJaguar3Device::la_capture(const devourer::LaParams &p) {
  if (!_la)
    _la = std::make_unique<devourer::LaCapture>(_device, _logger,
                                                devourer::la_regs_jgr3());
  /* Serialize against the coex runtime tick like every register-touching
   * entry point. */
  std::lock_guard<std::mutex> lk(_reg_mu);
  return _la->run(p);
}

devourer::TxPowerState RtlJaguar3Device::GetTxPowerState() {
  devourer::TxPowerState s;
  s.valid = true;
  s.flat_index = static_cast<int16_t>(_tx_pwr_override.load());
  s.offset_steps = static_cast<int16_t>(_tx_pwr_offset_steps.load());
  s.offset_qdb = s.offset_steps; /* 1 qdB per step on Jaguar3 */
  s.saturated_low = _txpwr_sat_low;
  s.saturated_high = _txpwr_sat_high;
  if (_brought_up && !_cw_active) {
    /* Reference readback (the Jaguar3 TXAGC refs ARE readable): OFDM ref
     * 0x18e8[16:10]; MCS7 is the per-rate diff table's zero anchor, so it
     * equals the OFDM ref; CCK ref 0x18a0[22:16]. Under _reg_mu for a
     * coherent snapshot vs the coex tick. */
    std::lock_guard<std::mutex> lk(_reg_mu);
    const uint32_t ofdm = (_device.rtw_read32(0x18e8) >> 10) & 0x7f;
    s.ofdm_index = static_cast<int16_t>(ofdm);
    s.mcs7_index = static_cast<int16_t>(ofdm);
    s.cck_index =
        static_cast<int16_t>((_device.rtw_read32(0x18a0) >> 16) & 0x7f);
    s.hw_readback = true;
  }
  return s;
}

devourer::ThermalStatus RtlJaguar3Device::GetThermalStatus() {
  devourer::ThermalStatus t;
  if (!_brought_up || _cw_active)
    return t; /* RF reads need a live, non-CW-held chip */
  uint8_t raw = 0, baseline = 0xFF;
  bool ok;
  {
    /* The meter trigger is an RF 0x42 RMW — the same register the coex
     * tick's pwr_track toggles. */
    std::lock_guard<std::mutex> lk(_reg_mu);
    ok = _hal.read_thermal(raw, baseline);
  }
  t.raw = raw;
  t.baseline = baseline;
  t.valid = ok && baseline != 0xFF;
  t.delta = t.valid ? static_cast<int>(raw) - static_cast<int>(baseline) : 0;
  return t;
}

bool RtlJaguar3Device::send_packet(const uint8_t *packet, size_t length) {
  /* The coex runtime thread (coex_runtime_loop) drives the periodic coex
   * decision + FW heartbeats + C2H draining, so the TX hot path stays lean. */
  /* Build one TXDMA block (48-byte descriptor + frame, build_tx_block) and
   * synchronously bulk-OUT. The TX path is enabled during rtw_hal_init
   * (3-wire RF + DACK + bf_init + enable_tx_path), so frames go on-air at the
   * tuned channel.
   *
   * Synchronous, bounded bulk-OUT. The async submit path (_device.send_packet)
   * never reaped its completions on the Jaguar3 TX loop (nothing calls
   * libusb_handle_events there), leaking a libusb_transfer per send. A blocking
   * transfer with a finite timeout completes-or-fails each send and stays
   * responsive to the caller's stop flag. No per-send clear_halt: resetting the
   * data-toggle on the hot path corrupts the chip's USB state machine (see
   * bulk_send_sync_ep). The caller backs off when these fail repeatedly — until
   * the TX-path enable registers are programmed the chip NAKs every frame, and
   * hammering a non-draining endpoint is exactly what wedged its USB core. */
  const uint16_t rlen = devourer::radiotap_hdr_len(packet, length);
  if (rlen == 0)
    return false;
  std::vector<uint8_t> usb_frame(jaguar3::TXDESC_SIZE_8822C + (length - rlen),
                                 0);
  if (build_tx_block(packet, length, usb_frame.data(), 0) == 0)
    return false;
  int rc = _device.bulk_send_sync_ep(_device.first_bulk_out_ep(),
                                     usb_frame.data(), usb_frame.size(),
                                     /*timeout_ms=*/20);
  return rc >= 0;
}

size_t RtlJaguar3Device::send_packets(const TxPacketView *pkts, size_t count) {
  /* USB TX aggregation (DEVOURER_TX_USB_AGG): pack consecutive frames into
   * shared bulk-OUT URBs — each frame keeps its own 48-byte descriptor, blocks
   * start 8-byte aligned, the FIRST descriptor carries the block count
   * (DMA_TXAGG_NUM). Packing rules in src/TxAggPlan.h. Knob off / PCIe -> the
   * interface-default per-frame loop. */
  const unsigned agg = _cfg.tx.usb_agg_max;
  if (agg <= 1 || !_device.is_usb() || count == 0)
    return IRtlDevice::send_packets(pkts, count);

  devourer::TxAggLimits lim;
  lim.desc_size = jaguar3::TXDESC_SIZE_8822C;
  lim.bulk_size = _device.speed() >= devourer::kUsbSpeedSuper  ? 1024
                  : _device.speed() >= devourer::kUsbSpeedHigh ? 512
                                                               : 64;
  /* HalMAC chips parse at most 3 descriptors per bulk transfer (rtw88
   * usb_tx_agg_desc_num / halmac BLK_DESC_NUM) — beyond that the TXDMA
   * misparses (bench-proven on the 8822BU sibling: block 1 re-aired agg-num
   * times). Layout is rtw88-parity: no first-block PKT_OFFSET reserve. */
  lim.max_frames = std::min<unsigned>(agg, 3u);
  lim.descs_per_bulk = 0;
  lim.first_reserve = false;

  size_t done = 0, ok = 0;
  while (done < count) {
    /* Collect the contiguous run for ONE URB: well-formed frames staying on
     * one channel. A frame whose radiotap CHANNEL differs ends the run — the
     * pending URB airs on the old channel, and the retune happens inside
     * build_tx_block when that frame leads the next URB. Same rule for a
     * differing radiotap DBM_TX_POWER: a first-seen power value can reprogram
     * an offset bank (0x1e70), which must land between URBs — an already-
     * packed frame would otherwise air at the new bank value. */
    std::vector<size_t> lens;
    int run_chan = 0; /* 0 = no per-packet CHANNEL seen yet (current channel) */
    int run_pwr = INT_MIN; /* INT_MIN = session-default power */
    for (size_t i = done; i < count && lens.size() < lim.max_frames; ++i) {
      const uint16_t rlen =
          devourer::radiotap_hdr_len(pkts[i].data, pkts[i].len);
      if (rlen == 0) {
        if (lens.empty())
          ++done; /* skip a malformed leading frame (contract: skipped) */
        break;
      }
      const int want =
          devourer::radiotap_peek_channel(pkts[i].data, pkts[i].len);
      const int want_pwr =
          devourer::radiotap_peek_dbm_tx_power(pkts[i].data, pkts[i].len);
      if (lens.empty()) {
        run_chan = want;
        run_pwr = want_pwr;
      } else if ((want > 0 &&
                  want != (run_chan > 0 ? run_chan : _channel.Channel)) ||
                 want_pwr != run_pwr) {
        break;
      }
      lens.push_back(pkts[i].len - rlen);
    }
    if (lens.empty())
      continue;

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

    std::vector<uint8_t> urb(plan.total, 0);
    size_t built = 0;
    for (size_t k = 0; k < plan.frames(); ++k) {
      const uint8_t poff = (k == 0 && plan.shim) ? 1 : 0;
      if (build_tx_block(pkts[done + k].data, pkts[done + k].len,
                         urb.data() + plan.blocks[k].offset, poff) == 0)
        break; /* pre-validated, so only a defensive bail */
      ++built;
    }
    if (built != plan.frames()) {
      for (size_t k = 0; k < plan.frames(); ++k, ++done)
        if (send_packet(pkts[done].data, pkts[done].len))
          ++ok;
      continue;
    }

    /* First descriptor advertises the block count. Dword7 sits inside the
     * checksummed span, so re-checksum (idempotent — the checksum field is
     * re-zeroed first, and the 8822C span extension reads the PKT_OFFSET
     * field that is already in place). */
    uint8_t *first = urb.data() + plan.blocks[0].offset;
    SET_TX_DESC_DMA_TXAGG_NUM_8822C(first, plan.frames());
    jaguar3::cal_txdesc_chksum_8822c(first);

    const int rc = _device.bulk_send_sync_ep(_device.first_bulk_out_ep(),
                                             urb.data(), urb.size(),
                                             /*timeout_ms=*/50);
    devourer::Ev(_logger->events(), "tx.agg")
        .f("frames", (unsigned long long)plan.frames())
        .f("bytes", (unsigned long long)urb.size())
        .f("shim", plan.shim)
        .f("ok", rc >= 0);
    if (rc >= 0)
      ok += plan.frames();
    done += plan.frames();
  }
  return ok;
}

size_t RtlJaguar3Device::build_tx_block(const uint8_t *packet, size_t length,
                                        uint8_t *out, uint8_t pkt_offset) {
  /* Parse the radiotap header (same fields the Jaguar1 path reads) and fill
   * the 8822C TX descriptor + 802.11 frame at `out` (contract in the header
   * decl). */
  if (length < sizeof(struct ieee80211_radiotap_header))
    return 0;
  uint16_t radiotap_length = get_unaligned_le16(packet + 2);
  if (radiotap_length == 0 || static_cast<size_t>(radiotap_length) >= length)
    return 0;
  const size_t frame_len = length - radiotap_length;

  uint8_t fixed_rate = MGN_1M;
  uint8_t sgi = 0, ldpc = 0, stbc = 0;
  ChannelWidth_t bwidth = CHANNEL_WIDTH_20;
  bool vht = (radiotap_length != 0x0d);
  bool rate_from_radiotap = false; /* did the frame's radiotap carry a rate? */
  /* Per-packet hop target from a radiotap CHANNEL field (0 = none present). */
  int radiotap_channel = 0;
  /* Per-packet TX-power delta from a radiotap DBM_TX_POWER field (dB vs the
   * calibrated table, the Jaguar2 convention). INT_MIN = not present -> the
   * SetTxPacketPowerOffsetQdb session default applies. */
  int radiotap_pkt_pwr_db = INT_MIN;

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
    case IEEE80211_RADIOTAP_CHANNEL:
      /* 2 x __le16: frequency (MHz), then flags. Frequency is authoritative
       * for the per-packet hop target; flags are ignored (rate/BW come from
       * the RATE/MCS/VHT fields). Same contract as the Jaguar1 path. */
      radiotap_channel =
          devourer::freq_to_chan(get_unaligned_le16(it.this_arg));
      break;
    case IEEE80211_RADIOTAP_DBM_TX_POWER:
      /* Signed dB delta for THIS frame, resolved to a power-offset bank
       * below (txpkt_type_for_idx). Mirrors the Jaguar2 per-packet path. */
      radiotap_pkt_pwr_db = *reinterpret_cast<const int8_t *>(it.this_arg);
      break;
    case IEEE80211_RADIOTAP_MCS: {
      /* One shared reading of the HT MCS known/flag/index bytes (bw/sgi/mcs +
       * LDPC/STBC). The J3 path historically read bw/sgi/mcs only and dropped
       * the FEC/STBC known bits, so an HT frame requesting LDPC/STBC aired as
       * plain BCC/no-STBC — only VHT frames were honoured. Mirrors the Jaguar2
       * path (decode_radiotap_mcs_field, RadiotapTxFlags.h); the STBC-cap guard
       * below still clamps to what the chip can do. */
      const devourer::RadiotapMcsField m =
          devourer::decode_radiotap_mcs_field(it.this_arg);
      if (m.bw40)
        bwidth = CHANNEL_WIDTH_40;
      sgi = m.sgi;
      ldpc = m.ldpc;
      stbc = m.stbc;
      if (m.have_mcs) {
        fixed_rate = MGN_MCS0 + m.mcs;
        rate_from_radiotap = true;
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
        auto bw = it.this_arg[3] & 0x1f;
        if (bw >= 1 && bw <= 3)
          bwidth = CHANNEL_WIDTH_40;
        else if (bw >= 4 && bw <= 10)
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

  /* Radiotap CHANNEL is authoritative for per-packet frequency, exactly as
   * RATE/MCS are for rate: a frame asking for a different channel triggers a
   * lean FastRetune before the descriptor is built (so the 40-in-80 DATA_SC
   * below keys off up-to-date channel state). FastRetune takes _reg_mu for the
   * retune only — the TX hot path stays lock-free when not hopping. */
  if (radiotap_channel > 0 && radiotap_channel != _channel.Channel)
    FastRetune(static_cast<uint8_t>(radiotap_channel), /*cache_rf=*/true);

  /* Radiotap carried no rate -> apply the runtime TX-mode default (SetTxMode).
   * Without this a rate-less frame (e.g. the demo's beacon) falls back to
   * MGN_1M, so an "MCS7" flood silently went on-air at 1 Mbps. Per-packet
   * radiotap always wins. Ported from RtlJaguarDevice (was Jaguar1-only). */
  if (!rate_from_radiotap && _tx_mode_default.has_value()) {
    const devourer::TxParams tp = devourer::tx_mode_to_params(*_tx_mode_default);
    fixed_rate = tp.fixed_rate;
    vht = tp.vht;
    sgi = tp.sgi ? 1 : 0;
    ldpc = tp.ldpc ? 1 : 0;
    stbc = tp.stbc ? 1 : 0;
    bwidth = static_cast<ChannelWidth_t>(tp.bwidth);
  }

  /* DEVOURER_TX_NDPA=N — beamforming-sounding probe: mark injected frames as
   * NDPA so the armed sounding engine (DEVOURER_BF_ARM_SOUNDER, InitWrite)
   * follows each with a hardware NDP. N is the PERIOD: 1 (or non-numeric) =
   * every frame (self-sounding capture, the original behaviour); N>1 = every Nth
   * frame, so the interleaved data frames stay steerable for TX beamforming. */
  const int ndpa_period = _cfg.bf.ndpa_period;
  bool ndpa = false;
  if (ndpa_period > 0) {
    ndpa = (_ndpa_ctr % static_cast<uint64_t>(ndpa_period)) == 0;
    ++_ndpa_ctr;
  }
  /* Periodic sounding (N>1, the TX-BF case) needs a 2-stream NDP to fill the
   * 2-antenna V matrix, so force those NDPA frames to VHT-2SS regardless of the
   * data rate (measured: a 1SS NDP yields no CBR); the interleaved data frames
   * keep their radiotap rate (1SS/HT), which is what gets steered. N==1
   * (every-frame self-sounding capture) keeps the caller's rate unchanged. */
  if (ndpa && ndpa_period > 1) {
    fixed_rate = MGN_VHT2SS_MCS0;
    vht = true;
  }

  uint8_t bw_desc = (bwidth == CHANNEL_WIDTH_40)   ? 1
                    : (bwidth == CHANNEL_WIDTH_80) ? 2
                                                   : 0;
  uint8_t rate_id = vht ? 9 : 8;

  /* 40-in-80: a 40 MHz frame on an 80 MHz-configured channel needs a data
   * sub-channel telling the PHY which 40 MHz half to use — the lower 40 (which
   * carries the primary), VHT_DATA_SC_40_LOWER_OF_80MHZ = 10. The 40 MHz
   * waveform then rides the 80 MHz RF path and stays decodable by a standard
   * HT40 receiver on the lower-40 channel (the `iw 80MHz` + 40 MHz-radiotap
   * equivalent). */
  uint8_t data_sc = 0;
  if (_channel.ChannelWidth == CHANNEL_WIDTH_80 && bwidth == CHANNEL_WIDTH_40)
    data_sc = 10; /* VHT_DATA_SC_40_LOWER_OF_80MHZ */

  /* Group-address (broadcast/multicast) detection: addr1 is the first MAC
   * address in the 802.11 header (after the 2-byte FC + 2-byte duration). Its
   * I/G bit (bit0 of the first octet) marks a group address; set BMC to match
   * the kernel's descriptor for group-addressed frames. */
  const uint8_t *dot11 = packet + radiotap_length;
  bool bmc = frame_len >= 6 && (dot11[4] & 0x01);
  /* STBC guard (IRtlDevice contract) — 8822C/8822E are 2T2R so this never
   * fires today, but keeps the invariant uniform across families: never air an
   * STBC frame the chip can't do. */
  if (stbc && !GetTxCaps().stbc_ok)
    stbc = 0;
  /* Per-packet TX-power bank selector: radiotap DBM_TX_POWER wins per frame
   * (resolved to a bank, programming 0x1e70 on a first-seen value), else the
   * SetTxPacketPowerOffsetQdb session default (pre-resolved, lock-free). */
  uint8_t pwr_type = _txpkt_dflt_type.load(std::memory_order_relaxed);
  if (radiotap_pkt_pwr_db != INT_MIN)
    pwr_type = txpkt_type_for_idx(txpkt_idx_for_qdb(radiotap_pkt_pwr_db * 4));
  jaguar3::fill_data_tx_desc_8822c(
      out, static_cast<uint16_t>(frame_len), MRateToHwRate(fixed_rate), rate_id,
      bw_desc, sgi != 0, ldpc != 0, stbc, bmc, ndpa, data_sc, pwr_type,
      pkt_offset);
  if (_cfg.tx.report) {
    /* DEVOURER_TX_REPORT: SPE_RPT asks the fw for a per-frame CCX TX report;
     * the report echoes SW_DEFINE's low byte, so stamp a rotating tag for
     * per-frame correlation (src/TxReport.h). Both fields sit inside the
     * checksummed span — re-checksum (idempotent). */
    SET_TX_DESC_SPE_RPT_8822C(out, 1);
    SET_TX_DESC_SW_DEFINE_8822C(out, _tx_rpt_tag.fetch_add(1) & 0xff);
    jaguar3::cal_txdesc_chksum_8822c(out);
  }
  const devourer::AmpduMode am = _ampdu; /* one lock-free load */
  if (am.enabled || _cfg.debug.tx_qsel || _cfg.debug.tx_ampdu_max) {
    /* A-MPDU descriptor half. The product SetAmpduMode state applies first,
     * then the raw DEVOURER_TX_QSEL / DEVOURER_TX_AMPDU spike knobs override
     * for register-level experimentation. All inside the checksummed span —
     * re-checksum once at the end. */
    if (am.enabled) {
      SET_TX_DESC_QSEL_8822C(out, am.tid);
      SET_TX_DESC_AGG_EN_8822C(out, 1);
      SET_TX_DESC_MAX_AGG_NUM_8822C(out, am.max_num & 0x1f);
      SET_TX_DESC_AMPDU_DENSITY_8822C(out, am.density & 0x7);
      SET_TX_DESC_RTS_DATA_RTY_LMT_8822C(out, am.no_ack ? 0 : 12);
    }
    if (_cfg.debug.tx_qsel)
      SET_TX_DESC_QSEL_8822C(out, *_cfg.debug.tx_qsel);
    if (_cfg.debug.tx_ampdu_max) {
      SET_TX_DESC_AGG_EN_8822C(out, 1);
      SET_TX_DESC_MAX_AGG_NUM_8822C(out, *_cfg.debug.tx_ampdu_max & 0x1f);
      SET_TX_DESC_AMPDU_DENSITY_8822C(out, _cfg.debug.tx_ampdu_density & 0x7);
      if (_cfg.debug.tx_ampdu_rty)
        SET_TX_DESC_RTS_DATA_RTY_LMT_8822C(out, *_cfg.debug.tx_ampdu_rty);
    }
    jaguar3::cal_txdesc_chksum_8822c(out);
  }
  const size_t frame_off =
      jaguar3::TXDESC_SIZE_8822C + static_cast<size_t>(pkt_offset) * 8;
  std::memcpy(out + frame_off, packet + radiotap_length, frame_len);
  return frame_off + frame_len;
}

SelectedChannel RtlJaguar3Device::GetSelectedChannel() { return _channel; }

uint64_t RtlJaguar3Device::ReadTsf() {
  /* REG_TSFTR 0x0560 (low) / 0x0564 (high); hi/lo/hi with a wrap retry. Under
   * _reg_mu (shared with the coex runtime thread). Starved to 0 under a heavy
   * RX bulk-IN flood — reliable from a quiet TX. */
  std::lock_guard<std::mutex> lk(_reg_mu);
  uint32_t hi = _device.rtw_read<uint32_t>(0x0564);
  uint32_t lo = _device.rtw_read<uint32_t>(0x0560);
  if (_device.rtw_read<uint32_t>(0x0564) != hi) {
    hi = _device.rtw_read<uint32_t>(0x0564);
    lo = _device.rtw_read<uint32_t>(0x0560);
  }
  return (static_cast<uint64_t>(hi) << 32) | lo;
}

void RtlJaguar3Device::WriteTsf(uint64_t tsf) {
  /* REG_TSFTR 0x0560 (low) / 0x0564 (high). Serialized on _reg_mu against the
   * coex tick. The counter keeps running, so this sets it to ~tsf. */
  std::lock_guard<std::mutex> lk(_reg_mu);
  _device.rtw_write<uint32_t>(0x0560, static_cast<uint32_t>(tsf));
  _device.rtw_write<uint32_t>(0x0564, static_cast<uint32_t>(tsf >> 32));
}

bool RtlJaguar3Device::SetAckResponder(const devourer::MacAddr &mac) {
  /* Hardware ACK responder (src/AckResponder.h): port identity + net_type so
   * the MAC auto-ACKs unicast frames to `mac`. Same registers the proven
   * StartBeacon/AP path programs, minus the beacon machinery. Serialized on
   * _reg_mu like every other register-touching control call. */
  std::lock_guard<std::mutex> lk(_reg_mu);
  devourer::ack::enable(_device, mac.data());
  _logger->info("Jaguar3: hardware ACK responder armed for "
                "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                mac.bytes[0], mac.bytes[1], mac.bytes[2], mac.bytes[3],
                mac.bytes[4], mac.bytes[5]);
  return true;
}

void RtlJaguar3Device::ClearAckResponder() {
  std::lock_guard<std::mutex> lk(_reg_mu);
  devourer::ack::disable(_device);
  _logger->info("Jaguar3: hardware ACK responder disarmed (net_type=NoLink)");
}

bool RtlJaguar3Device::SetAmpduMode(const devourer::AmpduMode &mode) {
  /* A-MPDU TX mode (src/AmpduMode.h): record the descriptor state the TX path
   * reads and program the 8822C aggregate-fill timer live. The descriptor
   * half is applied per-frame in build_tx_block. The 8822C has no
   * bring-up 0x04BC write (unlike the 8822B), so clear_burst_mode is a no-op
   * here — the 0x0455 timer is the pacing lever on this family (the on-air
   * unlock was bench-proven on the 8822B; the 8822C shares the register). */
  {
    std::lock_guard<std::mutex> lk(_reg_mu);
    if (mode.enabled) {
      if (mode.max_time != 0)
        _device.rtw_write8(0x0455, mode.max_time);
    } else {
      _device.rtw_write8(0x0455, 0x70); /* restore the bring-up default */
    }
  }
  _ampdu = mode;
  if (mode.enabled)
    _logger->info("Jaguar3: A-MPDU mode ON (tid={} max={} density={} {} "
                  "max_time=0x{:02x})",
                  mode.tid, mode.max_num, mode.density,
                  mode.no_ack ? "no-ack" : "ack", mode.max_time);
  else
    _logger->info("Jaguar3: A-MPDU mode OFF (pacing restored)");
  return true;
}

void RtlJaguar3Device::ClearAmpduMode() { SetAmpduMode(devourer::AmpduMode{}); }

bool RtlJaguar3Device::StartBeacon(const uint8_t *beacon, size_t len,
                                      int interval_tu) {
  std::lock_guard<std::mutex> lk(_reg_mu);
  /* The caller may pass [radiotap][802.11 MPDU]; the rsvd-page beacon must be the
   * RAW 802.11 MPDU (the TX descriptor carries the PHY, not a radiotap header).
   * radiotap it_len is bytes [2:3] LE. Strip it. */
  size_t rt = (len >= 4) ? (size_t)(beacon[2] | (beacon[3] << 8)) : 0;
  if (rt > len) rt = 0;
  const uint8_t *mpdu = beacon + rt;
  size_t mpdu_len = len - rt;
  /* Load the beacon into the beacon rsvd-page (halmac send_fw_page: QSEL_BEACON
   * bulk-OUT + bcn-valid latch), same mechanism as J2. */
  if (!_hal.download_beacon_page(mpdu, static_cast<uint32_t>(mpdu_len))) {
    _logger->error("beacon-tbtt(J3): rsvd-page beacon download failed");
    return false;
  }
  /* Port identity (rtw88 rtw_vif_port_config) — the beaconing port needs its own
   * MAC address (REG_MACID 0x0610) and BSSID (REG_BSSID 0x0618). devourer's
   * monitor bring-up sets NEITHER (usbmon diff vs the kernel IBSS), so the port
   * has no identity and the MAC won't transmit its beacon. Take them from the
   * MPDU's addr2 (SA) / addr3 (BSSID). */
  if (mpdu_len >= 24) {
    const uint8_t *sa = mpdu + 10, *bs = mpdu + 16;
    _device.rtw_write<uint32_t>(0x0610, (uint32_t)sa[0] | (sa[1] << 8) |
                                            (sa[2] << 16) | ((uint32_t)sa[3] << 24));
    _device.rtw_write16(0x0614, (uint16_t)(sa[4] | (sa[5] << 8)));
    _device.rtw_write<uint32_t>(0x0618, (uint32_t)bs[0] | (bs[1] << 8) |
                                            (bs[2] << 16) | ((uint32_t)bs[3] << 24));
    _device.rtw_write16(0x061c, (uint16_t)(bs[4] | (bs[5] << 8)));
  }
  /* Port-0 network type = AP (rtw88 rtw_vif_port_config: net_type at REG_CR
   * [17:16] = REG_CR+2 byte [1:0]); a beacon airs only in AP/Ad-hoc mode. */
  uint8_t nt = _device.rtw_read8(0x0102 /* REG_CR+2 */);
  _device.rtw_write8(0x0102, static_cast<uint8_t>((nt & ~0x03u) | 0x03u));
  /* Beacon interval; the TSF free-runs from init so TBTT fires on TSF % interval. */
  _device.rtw_write16(0x0554 /* REG_BCN_INTERVAL */,
                      static_cast<uint16_t>(interval_tu));
  /* BCN_CTRL = EN_BCN_FUNCTION | DIS_TSF_UDT (rtw88 bcn_ctrl for AP/Ad-hoc). */
  _device.rtw_write8(0x0550 /* REG_BCN_CTRL */, (1u << 3) | (1u << 4));
  /* THE enable: BIT_EN_BCNQ_DL (BIT22) in REG_FWHW_TXQ_CTRL — rtw88 turns
   * beaconing on here (mac80211 BSS_CHANGED_BEACON_ENABLED: rtw_write32_set(
   * REG_FWHW_TXQ_CTRL, BIT_EN_BCNQ_DL)). Without it the beacon never leaves the
   * queue no matter what BCN_CTRL / net_type say. */
  uint32_t txq = _device.rtw_read<uint32_t>(0x0420 /* REG_FWHW_TXQ_CTRL */);
  _device.rtw_write<uint32_t>(0x0420, txq | (1u << 22) /* BIT_EN_BCNQ_DL */);
  /* H2C RSVD_PAGE (cmd 0x00): rtw88 sends this after each beacon download to tell
   * the FW the rsvd-page locations. The combo FW gates beacon TX, so it may need
   * this to start beaconing. Payload from the golden dump (probe/pspoll/null page
   * offsets) — approximate for the beacon-only layout, a probe of the hypothesis. */
  _hal.send_h2c_raw(0x690c0100u, 0x00000000u);
  _logger->info("beacon-tbtt(J3): beacon loaded, net_type->AP, BCN_CTRL=0x18, "
                "EN_BCNQ_DL set + H2C RSVD_PAGE (TXQ 0x{:08x}, interval {} TU)",
                _device.rtw_read<uint32_t>(0x0420), interval_tu);

  /* A SINGLE download is enough — the hardware auto-transmits the beacon at every
   * TBTT (bench-verified: one download airs ~8 beacons/s indefinitely on both
   * bands), so there is no periodic re-download. rtw88 re-downloads each interval
   * only to refresh dynamic beacon CONTENT (TSF/TIM); that would need a
   * content-update path (not this static beacon) and belongs behind a
   * DeviceConfig knob, not env — the library reads no environment. */
  _bcn_interval_tu = interval_tu > 0 ? interval_tu : 100;
  _tbtt_off_us = 0;  // fresh beacon function: TBTT grid at TSF % period == 0
  return true;
}

bool RtlJaguar3Device::UpdateBeaconPayload(const uint8_t *beacon, size_t len) {
  std::lock_guard<std::mutex> lk(_reg_mu);
  if (_bcn_interval_tu <= 0) {
    _logger->error("beacon-tbtt(J3): UpdateBeaconPayload without an active beacon");
    return false;
  }
  /* Same buffer contract as StartBeacon: strip a leading radiotap header. */
  size_t rt = (len >= 4) ? (size_t)(beacon[2] | (beacon[3] << 8)) : 0;
  if (rt > len) rt = 0;
  /* A fresh rsvd-page download replaces the TBTT engine's buffer; the J3
   * latch is stable across it (no steer-style re-latch), so the enable path,
   * interval, TBTT phase and port identity are all untouched. */
  if (!_hal.download_beacon_page(beacon + rt, static_cast<uint32_t>(len - rt))) {
    _logger->error("beacon-tbtt(J3): UpdateBeaconPayload rsvd-page download failed");
    return false;
  }
  return true;
}

bool RtlJaguar3Device::StopBeacon() {
  std::lock_guard<std::mutex> lk(_reg_mu);
  if (_bcn_interval_tu <= 0)
    return false;
  /* EN_BCN_FUNCTION off (keep DIS_TSF_UDT), beacon-queue download off,
   * net_type back to No Link — the StartBeacon enables, reversed. */
  _device.rtw_write8(0x0550 /* REG_BCN_CTRL */, (1u << 4));
  uint32_t txq = _device.rtw_read<uint32_t>(0x0420 /* REG_FWHW_TXQ_CTRL */);
  _device.rtw_write<uint32_t>(0x0420, txq & ~(1u << 22) /* BIT_EN_BCNQ_DL */);
  uint8_t nt = _device.rtw_read8(0x0102);
  _device.rtw_write8(0x0102, static_cast<uint8_t>(nt & ~0x03u));
  _bcn_interval_tu = 0;
  _logger->info("beacon-tbtt(J3): stopped (EN_BCN off, EN_BCNQ_DL off, "
                "net_type->NoLink)");
  return true;
}

int32_t RtlJaguar3Device::AdjustBeaconTiming(int32_t microseconds) {
  int nominal;
  {
    std::lock_guard<std::mutex> lk(_reg_mu);
    nominal = _bcn_interval_tu;
  }
  if (nominal <= 0) return 0;  // no active beacon
  /* Round to whole TU (REG_BCN_INTERVAL is integer TU); sign follows the request
   * (>0 = later/retard => longer one-shot interval, <0 = earlier/advance). */
  int delta_tu = (microseconds >= 0 ? microseconds + 512 : microseconds - 512) / 1024;
  if (delta_tu == 0) return 0;  // below 1-TU resolution
  int one = nominal + delta_tu;
  if (one < 1) {  // can't shorten below one TU
    one = 1;
    delta_tu = one - nominal;
  }
  /* Latch one interval at the tweaked length; after exactly one TBTT fires under
   * it the phase has advanced/retarded by delta_tu TU, then restore nominal. The
   * TSF free-runs — this steers the beacon-engine TBTT counter, not the TSF
   * (WriteTsf can't; bench-proven).
   *
   * The interval register latches at a TBTT, so the shift count = the number of
   * TBTTs between the tweak-latch and the restore-latch — a fixed sleep races
   * with the beacon phase (bench-caught on the J2 8821CE, same latch semantics:
   * an advance whose write+restore land inside one period is a silent no-op;
   * one held past two shortened periods double-shifts). Phase-align off the TSF
   * instead (TBTT fires at TSF % interval == 0): read the in-period position,
   * write the tweak, then time the restore to land mid-way into the FIRST
   * tweaked interval — the restore latches at that interval's closing TBTT, so
   * exactly one fires under the tweak. Positions are measured against the TBTT
   * grid (TSF % period == _tbtt_off_us — prior coarse steers move the grid off
   * the TSF). Serialize register access on _reg_mu but release it across the
   * waits so the coex tick isn't starved. */
  const int64_t period_us = static_cast<int64_t>(nominal) * 1024;
  auto grid_pos = [&]() {  // in-period position vs the TBTT grid; under _reg_mu
    uint32_t hi = _device.rtw_read<uint32_t>(0x0564);
    uint32_t lo = _device.rtw_read<uint32_t>(0x0560);
    int64_t p = static_cast<int64_t>(((static_cast<uint64_t>(hi) << 32) | lo) %
                                     static_cast<uint64_t>(period_us));
    return ((p - _tbtt_off_us) % period_us + period_us) % period_us;
  };
  int64_t pos;
  {
    std::lock_guard<std::mutex> lk(_reg_mu);
    pos = grid_pos();
  }
  /* Keep the tweak write clear of the next TBTT (so the latching TBTT is
   * unambiguous even against register/sleep jitter). */
  if (pos > period_us - 20000)
    std::this_thread::sleep_for(
        std::chrono::microseconds(period_us - pos + 5000));
  {
    std::lock_guard<std::mutex> lk(_reg_mu);
    pos = grid_pos();
    _device.rtw_write16(0x0554 /* REG_BCN_INTERVAL */, static_cast<uint16_t>(one));
  }
  /* latch TBTT in (period - pos) µs; restore mid-first-tweaked-interval. */
  std::this_thread::sleep_for(std::chrono::microseconds(
      (period_us - pos) + static_cast<int64_t>(one) * 512));
  {
    std::lock_guard<std::mutex> lk(_reg_mu);
    _device.rtw_write16(0x0554, static_cast<uint16_t>(nominal));
    _tbtt_off_us = ((_tbtt_off_us + static_cast<int64_t>(delta_tu) * 1024) %
                        period_us + period_us) % period_us;
  }
  _logger->info("beacon(J3): TBTT shift {} TU ({} us) via one-shot interval "
                "{}->{}->{} TU",
                delta_tu, delta_tu * 1024, nominal, one, nominal);
  return delta_tu * 1024;
}

int32_t RtlJaguar3Device::AdjustBeaconTimingFine(int32_t microseconds) {
  std::lock_guard<std::mutex> lk(_reg_mu);
  if (_bcn_interval_tu <= 0) return 0;  // no active beacon
  /* A bare TSF write with the beacon function running leaves the TBTT latched
   * (bench-proven: WriteTsf moved the reported TSF but not the air-time). The
   * vendor reset_tsf path clears EN_BCN_FUNCTION first, so do the same: toggle
   * the beacon function off, shift the port-0 TSF, toggle on — the TBTT counter
   * re-derives from the shifted TSF at microsecond resolution. TBTT fires at
   * TSF % interval, so subtracting `microseconds` advances (<0) / retards (>0)
   * the next boundary by that many µs. */
  uint32_t hi = _device.rtw_read<uint32_t>(0x0564);
  uint32_t lo = _device.rtw_read<uint32_t>(0x0560);
  uint64_t tsf = (static_cast<uint64_t>(hi) << 32) | lo;
  uint64_t nt = tsf - static_cast<uint64_t>(static_cast<int64_t>(microseconds));
  uint8_t bc = _device.rtw_read8(0x0550 /* REG_BCN_CTRL */);
  _device.rtw_write8(0x0550, static_cast<uint8_t>(bc & ~(1u << 3)));  // clear EN_BCN_FUNCTION
  _device.rtw_write<uint32_t>(0x0560, static_cast<uint32_t>(nt));
  _device.rtw_write<uint32_t>(0x0564, static_cast<uint32_t>(nt >> 32));
  _device.rtw_write8(0x0550, static_cast<uint8_t>(bc | (1u << 3)));   // set EN_BCN_FUNCTION
  _tbtt_off_us = 0;  // the re-latch re-derives the TBTT grid from the TSF
  _logger->info("beacon(J3): fine TBTT shift {} us (TSF toggle: EN_BCN off/shift/on)",
                microseconds);
  return microseconds;
}

int32_t RtlJaguar3Device::PinBeaconTbtt(int32_t offset_us) {
  std::lock_guard<std::mutex> lk(_reg_mu);
  if (_bcn_interval_tu <= 0) return 0;  // no active beacon
  const int64_t period_us = static_cast<int64_t>(_bcn_interval_tu) * 1024;
  const int64_t off =
      ((static_cast<int64_t>(offset_us) % period_us) + period_us) % period_us;
  /* TSF-preserving absolute pin (the J2 PinBeaconTbtt pattern; see
   * RtlJaguar2Device for the discipline-loop rationale): shift + re-latch so
   * the TBTT re-derives at TSF % period == off, then immediately write the
   * TSF back onto its original timeline — a bare TSF write does not move the
   * TBTT, so the pinned phase survives. No reserved-page re-download here:
   * the J3 engine keeps its bcn-valid latch across the re-latch. */
  auto read_tsf = [&]() {
    uint32_t hi = _device.rtw_read<uint32_t>(0x0564);
    uint32_t lo = _device.rtw_read<uint32_t>(0x0560);
    if (_device.rtw_read<uint32_t>(0x0564) != hi) {
      hi = _device.rtw_read<uint32_t>(0x0564);
      lo = _device.rtw_read<uint32_t>(0x0560);
    }
    return (static_cast<uint64_t>(hi) << 32) | lo;
  };
  uint64_t nt = read_tsf() - static_cast<uint64_t>(off);
  uint8_t bc = _device.rtw_read8(0x0550 /* REG_BCN_CTRL */);
  _device.rtw_write8(0x0550, static_cast<uint8_t>(bc & ~(1u << 3)));
  _device.rtw_write<uint32_t>(0x0560, static_cast<uint32_t>(nt));
  _device.rtw_write<uint32_t>(0x0564, static_cast<uint32_t>(nt >> 32));
  _device.rtw_write8(0x0550, static_cast<uint8_t>(bc | (1u << 3)));  // re-latch
  uint64_t back = read_tsf() + static_cast<uint64_t>(off);  // original timeline
  _device.rtw_write<uint32_t>(0x0560, static_cast<uint32_t>(back));
  _device.rtw_write<uint32_t>(0x0564, static_cast<uint32_t>(back >> 32));
  _tbtt_off_us = off;
  _logger->info("beacon(J3): TBTT pinned to TSF%%interval == {} us "
                "(TSF-preserving; requested {})", (long long)off, offset_us);
  return static_cast<int32_t>(off);
}

void RtlJaguar3Device::SetTxMode(const devourer::TxMode &mode) {
  _tx_mode_default = mode;
}
void RtlJaguar3Device::ClearTxMode() { _tx_mode_default.reset(); }
