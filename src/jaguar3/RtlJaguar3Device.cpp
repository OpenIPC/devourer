#include "RtlJaguar3Device.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <utility>
#include <vector>

#include "BeamformingSounder.h" /* generation-neutral BF self-sounding recipe */

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

RtlJaguar3Device::RtlJaguar3Device(RtlUsbAdapter device, Logger_t logger,
                                   jaguar3::ChipVariant variant)
    : _device{device}, _logger{logger}, _variant{variant},
      _hal{device, logger, variant}, _radioManagement{device, logger, variant} {
  _logger->info("RtlJaguar3Device constructed ({})",
                variant == jaguar3::ChipVariant::C8822E ? "8822E/EU" : "8822C/CU");
}

void RtlJaguar3Device::Init(Action_ParsedRadioPacket packetProcessor,
                            SelectedChannel channel) {
  _channel = channel;
  _hal.rtw_hal_init(channel);  /* full vendor-source bring-up */
  /* Tune the channel/bandwidth (5/10 MHz ChannelWidth re-clocks to narrowband),
   * then run IQK calibration (it reads RF18 for the tuned channel). */
  _radioManagement.set_channel_bwmode(channel.Channel, channel.ChannelOffset,
                                      channel.ChannelWidth);
  _hal.run_iqk(channel);
  _hal.enable_rx_path(); /* RF into RX mode (IGI toggle) — must follow channel set */
  _hal.config_rfe(channel.Channel); /* 8822e RFE/PAPE antenna-switch pins */
  _hal.config_channel_8822e(channel.Channel); /* 8822e band TX scaling/backoff + shaping */
  _hal.coex_wlan_only_init(); /* lock antenna to WLAN (disable BT/LTE coex) */

  /* DEVOURER_BF_ARM_BFEE=aa:bb:cc:dd:ee:ff — beamforming self-sounding probe
   * (beamformee side), Jaguar-3 variant. Arms the hardware CSI responder to
   * reply to NDPA+NDP from the given beamformer MAC with a VHT Compressed
   * Beamforming report, no association. Uses the shared MAC recipe with the
   * Jaguar-2/3 config (0xDB, 16-bit CSI param, RX-filter + own-AID gates, and
   * crucially NO 0x9B4 write — that address is the narrowband clock divider on
   * this generation). See BeamformingSounder.h. */
  if (const char *bfer = std::getenv("DEVOURER_BF_ARM_BFEE")) {
    unsigned m[6];
    if (std::sscanf(bfer, "%x:%x:%x:%x:%x:%x", &m[0], &m[1], &m[2], &m[3],
                    &m[4], &m[5]) == 6) {
      uint8_t mac[6];
      for (int i = 0; i < 6; ++i) mac[i] = static_cast<uint8_t>(m[i]);
      /* Jaguar-3 bring-up never programs the self-MAC (0x0610), so the NDPA
       * RA has nothing to match. Give the beamformee a known identity here so
       * the sounder can address it; log it for the test harness. */
      static const uint8_t kBfeeMac[6] = {0x00, 0xe0, 0x4c, 0x88, 0x22, 0xce};
      for (uint16_t i = 0; i < 6; ++i)
        _device.rtw_write8(0x0610 + i, kBfeeMac[i]);
      /* DEVOURER_BF_ARM_BFEE_MU=1 upgrades the responder to an MU beamformee,
       * whose report appends the per-subcarrier delta-SNR (MU Exclusive
       * Beamforming Report) the SU report omits. Pair with the sounder's
       * DEVOURER_TX_NDPA_MU=1 (MU feedback bit in the NDPA STA-info). */
      if (std::getenv("DEVOURER_BF_ARM_BFEE_MU")) {
        devourer::bf::arm_beamformee_mu(_device, mac, devourer::bf::kBfeeJaguar23);
        _logger->info("Jaguar3 BF MU-beamformee armed for beamformer {} — "
                      "beamformee MAC 00:e0:4c:88:22:ce", bfer);
      } else {
        devourer::bf::arm_beamformee(_device, mac, devourer::bf::kBfeeJaguar23);
        _logger->info("Jaguar3 BF beamformee armed for beamformer {} — "
                      "beamformee MAC 00:e0:4c:88:22:ce", bfer);
      }
    } else {
      _logger->error("DEVOURER_BF_ARM_BFEE — bad MAC '{}'", bfer);
    }
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
  if (std::getenv("DEVOURER_RX_CSI_MASK") || std::getenv("DEVOURER_RX_NBI")) {
    std::lock_guard<std::mutex> lk(_reg_mu);
    devourer::tonemask::apply_from_env(
        _device, _logger, devourer::tonemask::Family::JGR3, _channel, 2);
  }

  _logger->info("Jaguar3: entering RX loop (kernel-style async URB queue)");
  uint64_t frames = 0, reads = 0;
  /* Process one bulk-IN completion: walk the aggregated 8822C RX descriptors. */
  auto on_data = [&](const uint8_t *data, int n) {
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
        p.RxAtrib.drvinfo_sz = static_cast<uint8_t>(f.drvinfo_size);
        p.RxAtrib.shift_sz = f.shift;
        /* RX desc word2 BIT(28) = FW C2H report, not an 802.11 frame. During
         * concurrent TX+RX the FW emits one small C2H per TX, so tag them for
         * the packetProcessor (which skips C2H) instead of surfacing a flood
         * of short "frames". Same check the coex drain uses. */
        bool is_c2h = (data[off + 11] & 0x10) != 0;
        p.RxAtrib.pkt_rpt_type = is_c2h ? RX_PACKET_TYPE::C2H_PACKET
                                        : RX_PACKET_TYPE::NORMAL_RX;
        /* Decode the jgr3 PHY-status report (per-frame RSSI/SNR/EVM) when it is
         * present (monitor_rx_cfg enables APP_PHYSTS + RX_DRVINFO_SZ=4, so the
         * 32-byte report is counted in drvinfo). Skips C2H reports and any
         * frame whose drvinfo is too short (e.g. CCK, which carries no OFDM
         * report). The report sits immediately after the 24-byte descriptor. */
        if (!is_c2h && f.drvinfo_size >= 28)
          jaguar3::parse_phy_sts_jgr3(data + off + jaguar3::RXDESC_SIZE_8822C,
                                      f.drvinfo_size, p.RxAtrib);
        p.Data = std::span<uint8_t>(const_cast<uint8_t *>(f.frame), f.frame_len);
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
      _hal.fw_update_wl_phy_info();
      _hal.fw_set_pwr_mode_active();
      _hal.fw_coex_query_bt_info();
    } catch (...) { break; }
    if (++tick <= 3 || tick % 15 == 0)
      _logger->info("Jaguar3 coex: tick {} (bulk-IN reads={}, C2H={})", tick, rx,
                    c2h);
  }
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
  const bool want_rx = std::getenv("DEVOURER_TX_WITH_RX") != nullptr;
  _hal.rtw_hal_init(channel);  /* full vendor-source bring-up */
  _radioManagement.set_channel_bwmode(channel.Channel, channel.ChannelOffset,
                                      channel.ChannelWidth);
  _hal.run_iqk(channel);
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
  if (std::getenv("DEVOURER_CW_TONE")) {
    uint8_t g = 0;
    if (const char *e = std::getenv("DEVOURER_CW_TONE_GAIN"))
      g = static_cast<uint8_t>(std::atoi(e)) & 0x1F;
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

  if (_tx_pwr_override >= 0) {
    _radioManagement.set_tx_power_ref(static_cast<uint8_t>(_tx_pwr_override));
  } else if (_variant == jaguar3::ChipVariant::C8822E) {
    /* Default TX power (8822e). The TXAGC reference base is the per-channel 5 GHz
     * power index the kernel programs. It reads that from the efuse power-by-rate
     * table (logical 0x22 path A / 0x4C path B, per channel-group); when the byte
     * is valid (<= txgi_max 127) we use it verbatim for kernel-faithful
     * per-channel power. On bare modules the byte is unprogrammed (0xFF); the
     * kernel's own fallback there is a fixed default (0x33 for the plain path,
     * reshaped by the TSSI codeword on 8822e), and devourer falls back to
     * JAGUAR3_TXPWR_REF_BASE — an empirically on-air-matched default that is not
     * below the kernel's. The per-rate diffs come from the BB phy_reg_pg table
     * (apply_power_by_rate_8822e); full TSSI-offset power-by-rate is a follow-up. */
    uint8_t efuse_a = 0xFF, efuse_b = 0xFF;
    _hal.read_efuse_txpwr_base_8822e(channel.Channel, efuse_a, efuse_b);
    uint8_t ref_a = (efuse_a <= 127) ? efuse_a : JAGUAR3_TXPWR_REF_BASE;
    uint8_t ref_b = (efuse_b <= 127) ? efuse_b : JAGUAR3_TXPWR_REF_BASE;
    /* TX+RX mode: the path-B OFDM reference (0x41e8) desenses the EU's RX
     * (hardware-bisected, value-independent) — skip that one write and keep
     * the rest of the per-rate power intact. See apply_power_by_rate_8822e. */
    _radioManagement.apply_power_by_rate_8822e(channel.Channel, ref_a, ref_b,
                                               /*skip_path_b_ofdm_ref=*/want_rx);
    if (want_rx)
      _logger->info("Jaguar3(8822e): TX+RX mode — path-B OFDM TXAGC ref left "
                    "at table default (keeps RX alive)");
  } else {
    /* 8822c (CU): preserve the flat reference the demo used to impose (40) so
     * the SDR-validated CU TX power is unchanged now that the demo no longer
     * sets it. The CU's efuse-calibrated per-rate default is a follow-up. */
    _radioManagement.set_tx_power_ref(JAGUAR3_TXPWR_REF_BASE_8822C,
                                      /*zero_diffs=*/true);
  }
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
  if (const char *snd = std::getenv("DEVOURER_BF_ARM_SOUNDER")) {
    /* Jaguar-3 bring-up never programs the self-MAC (0x0610); the sounding
     * engine matches the injected NDPA's TA against it before firing the NDP.
     * DEVOURER_BF_ARM_SOUNDER=aa:bb:cc:dd:ee:ff programs that MAC (use the
     * NDPA TA the caller injects — the txdemo's canonical SA); a bare "1"
     * leaves it unprogrammed (Jaguar-1 semantics). */
    unsigned m[6];
    if (std::sscanf(snd, "%x:%x:%x:%x:%x:%x", &m[0], &m[1], &m[2], &m[3],
                    &m[4], &m[5]) == 6) {
      for (uint16_t i = 0; i < 6; ++i)
        _device.rtw_write8(0x0610 + i, static_cast<uint8_t>(m[i]));
      _logger->info("Jaguar3 BF sounder: self-MAC programmed to {}", snd);
    }
    _hal.txbf_rfmode_sounder(); /* RF/BB sounding config — needed for the NDP */
    devourer::bf::arm_sounder(_device, /*snd_ptcl_ctrl=*/0xDB);
    _logger->info("Jaguar3 BF sounder armed (beamformer side)");
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
  _coex_thread = std::thread([this] { coex_runtime_loop(); });
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
  return e;
}

void RtlJaguar3Device::SetMonitorChannel(SelectedChannel channel) {
  _channel = channel;
  _radioManagement.set_channel_bwmode(channel.Channel, channel.ChannelOffset,
                                      channel.ChannelWidth);
}

void RtlJaguar3Device::SetTxPower(uint8_t power) {
  /* Record a flat TXAGC reference override; applied during InitWrite (may be
   * called before bring-up). Without it the chip uses its efuse-calibrated
   * power. If the device is already brought up, apply immediately. */
  _tx_pwr_override = power;
  if (_coex_thread.joinable())
    _radioManagement.set_tx_power_ref(power);
}

bool RtlJaguar3Device::send_packet(const uint8_t *packet, size_t length) {
  /* The coex runtime thread (coex_runtime_loop) drives the periodic coex
   * decision + FW heartbeats + C2H draining, so the TX hot path stays lean. */
  /* Parse the radiotap header (same fields the Jaguar1 path reads) and build an
   * 8822C TX descriptor + synchronous bulk-OUT. The TX path is enabled during
   * rtw_hal_init (3-wire RF + DACK + bf_init + enable_tx_path), so frames go
   * on-air at the tuned channel. */
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
  bool rate_from_radiotap = false; /* did the frame's radiotap carry a rate? */

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
      uint8_t mcs_known = it.this_arg[0];
      uint8_t mcs_flags = it.this_arg[1];
      if ((mcs_flags & IEEE80211_RADIOTAP_MCS_BW_MASK) ==
          IEEE80211_RADIOTAP_MCS_BW_40)
        bwidth = CHANNEL_WIDTH_40;
      sgi = (mcs_flags & 0x04) ? 1 : 0;
      if (mcs_known & IEEE80211_RADIOTAP_MCS_HAVE_MCS) {
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
  /* DEVOURER_TX_NDPA=1 — beamforming self-sounding probe: mark injected frames
   * as NDPA so the armed sounding engine (DEVOURER_BF_ARM_SOUNDER, InitWrite)
   * follows each with a hardware NDP. Same knob as the Jaguar-1 path. */
  static const bool ndpa = std::getenv("DEVOURER_TX_NDPA") != nullptr;
  std::vector<uint8_t> usb_frame(jaguar3::TXDESC_SIZE_8822C + frame_len, 0);
  jaguar3::fill_data_tx_desc_8822c(
      usb_frame.data(), static_cast<uint16_t>(frame_len),
      MRateToHwRate(fixed_rate), rate_id, bw_desc, sgi != 0, ldpc != 0, stbc,
      bmc, ndpa, data_sc);
  std::memcpy(usb_frame.data() + jaguar3::TXDESC_SIZE_8822C,
              packet + radiotap_length, frame_len);

  /* Synchronous, bounded bulk-OUT. The async submit path (_device.send_packet)
   * never reaped its completions on the Jaguar3 TX loop (nothing calls
   * libusb_handle_events there), leaking a libusb_transfer per send. A blocking
   * transfer with a finite timeout completes-or-fails each send and stays
   * responsive to the caller's stop flag. No per-send clear_halt: resetting the
   * data-toggle on the hot path corrupts the chip's USB state machine (see
   * bulk_send_sync_ep). The caller backs off when these fail repeatedly — until
   * the TX-path enable registers are programmed the chip NAKs every frame, and
   * hammering a non-draining endpoint is exactly what wedged its USB core. */
  uint8_t tx_ep = _device.first_bulk_out_ep();
  int rc = _device.bulk_send_sync_ep(tx_ep, usb_frame.data(),
                                     usb_frame.size(), /*timeout_ms=*/20);
  return rc >= 0;
}

SelectedChannel RtlJaguar3Device::GetSelectedChannel() { return _channel; }

void RtlJaguar3Device::SetTxMode(const devourer::TxMode &mode) {
  _tx_mode_default = mode;
}
void RtlJaguar3Device::ClearTxMode() { _tx_mode_default.reset(); }
