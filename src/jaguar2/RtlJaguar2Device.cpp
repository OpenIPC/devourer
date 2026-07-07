#include "RtlJaguar2Device.h"

#include <climits>
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

#include "BeamformingSounder.h"
#include "ChannelFreq.h" /* radiotap CHANNEL freq -> channel (per-packet hop) */
#include "FrameParserJaguar2.h"
#include "Jaguar2Calibration.h"
#include "NhmReader.h"
#include "ToneMask.h"
#include "RateDefinitions.h"
#include "RxPacket.h"
#include "SignalStop.h" /* g_devourer_should_stop */
extern "C" {
#include "ieee80211_radiotap.h" /* MRateToHwRate + radiotap iterator */
}

RtlJaguar2Device::RtlJaguar2Device(RtlAdapter device, Logger_t logger,
                                   jaguar2::ChipVariant variant,
                                   devourer::DeviceConfig cfg)
    : _device{device}, _cfg{std::move(cfg)}, _logger{logger},
      _variant{variant}, _hal{device, logger, variant, _cfg},
      _macinit{device, logger, variant}, _fw{device, logger, variant} {}

RtlJaguar2Device::~RtlJaguar2Device() {
  /* Safety net: restore the chip if a CW tone is still armed. */
  StopCwTone();
  stop_dig();
}

void RtlJaguar2Device::bring_up(SelectedChannel channel) {
  /* Cold bring-up shared by Init (RX) and InitWrite (TX). Order mirrors the
   * HalMAC _halmac_init_hal / vendor rtl88x2bu flow: pre-init -> power-on ->
   * chip-version -> init_system_cfg -> firmware DLFW -> post-DLFW MAC cfg +
   * USB RX-DMA + BB/RF enable -> BB/AGC/RF phydm tables -> TRX mode -> channel
   * -> LCK -> IQK -> coex WL grant -> enable RX/TX MAC engine. */
  const uint8_t bw = static_cast<uint8_t>(channel.ChannelWidth);
  /* DLFW download BEFORE trx/queue config (HalMAC order): running init_trx first
   * over-allocates the FIFOPAGE queues and wedges the DLFW bcn-valid.
   *
   * Retry the FULL power-on + DLFW as a unit. On a warm/idle chip the FW-boot
   * handshake (0x80=0xC078) intermittently hangs, and re-resetting only the CPU
   * (download_firmware's own retry) is often not enough — the combo chip needs a
   * fresh power cycle. The vendor recovers the same way: _halmac_init_hal's
   * power_on re-toggles OFF/ON on a "warm reboot" (PWR_UNCHANGE) and the driver
   * layer retries init on failure. Each attempt here re-runs the complete
   * pre-init -> power-on -> system-cfg -> DLFW, giving the chip a clean slate. */
  bool fw_ok = false;
  constexpr int kBringupTries = 4;
  /* rtw88 applies the PCIe interface-PHY MDIO config once at probe. */
  if (!_device.is_usb())
    _macinit.pcie_phy_cfg();
  for (int attempt = 0; attempt < kBringupTries && !fw_ok; attempt++) {
    if (attempt > 0)
      _logger->error("RtlJaguar2Device: DLFW failed — full power-cycle retry "
                     "{}/{}",
                     attempt + 1, kBringupTries);
    /* rtw88 order: rtw_hci_setup precedes rtw_mac_power_on — on PCIe this
     * programs the TRX buffer-descriptor ring registers (the DLFW rsvd-page
     * path needs the BCN ring live), and it re-runs per attempt because the
     * power seq touches 0x300. No-op on USB. */
    _device.hci_setup();
    _macinit.pre_init_system_cfg();
    _hal.power_on();
    _hal.read_chip_version();
    _macinit.init_system_cfg(channel.ChannelWidth, _hal.chip_version().cut);
    fw_ok = _fw.download_default_firmware();
  }
  if (!fw_ok)
    throw std::runtime_error("RtlJaguar2Device: firmware DLFW failed");
  _logger->info("RtlJaguar2Device: firmware booted (bw={})", (int)bw);

  if (!_macinit.init_mac_cfg(channel.ChannelWidth))
    throw std::runtime_error("RtlJaguar2Device: init_mac_cfg failed");
  if (_device.is_usb())
    _macinit.init_usb_cfg(); /* PCIe RX = the BD ring, no RX-DMA agg cfg */
  _macinit.enable_bb_rf(true);
  _logger->info("RtlJaguar2Device: MAC cfg + BB/RF enabled");

  uint8_t rfe = _hal.read_efuse_rfe();
  /* The kernel uses rfe_type=3 for the T3U even with an unprogrammed efuse
   * (devourer reads 0xff -> defaults 0). rfe_type selects the BB/RF phydm
   * conditional blocks AND the RFE antenna-switch pins; the wrong variant leaves
   * the TX front-end mis-routed. DEVOURER_RFE=N overrides. */
  if (_cfg.tuning.rfe_type)
    rfe = *_cfg.tuning.rfe_type;
  _rfe = rfe; /* cache for SetMonitorChannel retune */
  _hal.apply_bb_rf_agc_tables(rfe);
  _logger->info("RtlJaguar2Device: PHY tables applied");

  _hal.config_trx_mode(); /* RF mode table + TX/RX antenna-path HW blocks */
  _hal.set_channel_bw(static_cast<uint8_t>(channel.Channel), bw, rfe,
                      channel.ChannelOffset);
  _hal.do_lck(); /* LC calibration — lock the RF LO */

  /* IQK runs by default on both variants (DEVOURER_SKIP_IQK to skip). The 8821C
   * halrf_iqk_8821c port reports a clean pass and — with the AFE quad
   * (0xc58/0xc5c/0xc60/0xc6c) added to its backup/restore so afe_setting(false)'s
   * IQK-exit values don't persist — no longer disturbs the OFDM/HT TX path. */
  if (!_cfg.tuning.skip_iqk) {
    auto cal = jaguar2::make_jaguar2_calibration(
        _variant, _device, _logger, _hal.chip_version().cut,
        _hal.chip_version().rf_2t2r != 0);
    cal->iqk_trigger(channel.Channel <= 14);
  } else {
    _logger->info("Jaguar2: IQK SKIPPED (tuning.skip_iqk)");
  }
  /* Re-assert the TX/RX antenna-path routing AFTER IQK. In the vendor flow
   * config_phydm_trx_mode runs from the post-calibration channel-set (PHY_SwChnl),
   * i.e. AFTER IQK — devourer ran it before IQK, and IQK's TX-path loopback +
   * one-shot apply-bit toggling (0xc94/0x80c/0x93c) leave the TX antenna path
   * disturbed with nothing to restore it (RX is unaffected, which is why RX
   * worked but TX radiated no valid frame). Gated for A/B bisection. */
  if (!_cfg.tuning.skip_trx_reassert) {
    _hal.config_trx_mode();
    _logger->info("Jaguar2: TX/RX path re-asserted post-IQK");
  }
  /* phydm DM-init RFE mux + TXBF init — the kernel runs these from
   * odm_dm_init (phydm_rfe_init) and rtl8822b_phy_bf_init after calibration.
   * devourer previously skipped both, leaving 0x1990=0 (RFE source mux) and
   * 0x1c94=0x5fff5fff (BB-table default, not the bf_init 0xafffafff).
   * 8821C: phydm_rfe_8821c is #if 0 in the vendor (no RFE-mux writes) so
   * rfe_init is skipped; the 8821C antenna is routed by switch_rf_set +
   * coex_wlan_only_8821c. bf_init IS run (rtl8821c_phy_bf_init) but via the
   * 8821C-specific MU/TXBF setup, not the 8822B 0x1c94-only write. */
  if (!_cfg.tuning.skip_rfe_init) {
    if (_variant == jaguar2::ChipVariant::C8821C) {
      _hal.bf_init_8821c();
    } else {
      _hal.rfe_init();
      _hal.bf_init();
    }
  }
  /* Program per-rate TXAGC from the EFUSE power-by-rate calibration (the level
   * the kernel uses), through the runtime-knob composer so a flat override /
   * offset recorded before bring-up folds in here. DEVOURER_TX_PWR (flat
   * override) applied later in InitWrite still wins for debug.
   * DEVOURER_SKIP_TXPWR keeps the BB-table default. */
  if (!_cfg.tuning.skip_txpwr)
    apply_tx_power_current();
  /* Grant the antenna to WLAN (combo chip) — must precede enable. */
  if (!_cfg.tuning.skip_coex)
    _hal.coex_wlan_only(channel.Channel > 14);
  else
    _logger->info("Jaguar2: coex WL grant SKIPPED (tuning.skip_coex)");
  _hal.enable_rx(); /* CR MACTX|MACRX + RCR + IGI — enables both TX and RX */
  _brought_up = true;
}

void RtlJaguar2Device::apply_tx_power_current() {
  const int off = _tx_pwr_offset_steps;
  const int flat = _tx_pwr_override;
  if (flat >= 0) {
    /* Flat override + offset, composed BEFORE set_tx_power_flat (whose &0x3f
     * would wrap an over-range compose instead of clamping). */
    int idx = flat + off;
    bool lo = false, hi = false;
    if (idx < 0) { idx = 0; lo = true; }
    if (idx > 63) { idx = 63; hi = true; }
    _hal.set_txpwr_saturation(lo, hi);
    _hal.set_tx_power_flat(static_cast<uint8_t>(idx));
    return;
  }
  _hal.apply_tx_power(static_cast<uint8_t>(_channel.Channel),
                      static_cast<uint8_t>(_channel.ChannelWidth), _rfe, off);
}

void RtlJaguar2Device::Init(Action_ParsedRadioPacket packetProcessor,
                            SelectedChannel channel) {
  _channel = channel;
  bring_up(channel);

  /* DEVOURER_BF_ARM_BFEE=aa:bb:cc:dd:ee:ff — beamforming self-sounding
   * (beamformee side), Jaguar-2 variant. Arms the hardware CSI responder to
   * reply to NDPA+NDP from the given beamformer MAC with a VHT Compressed
   * Beamforming report, no association. Uses the shared MAC recipe with the
   * Jaguar-2/3 config (0xDB, 16-bit CSI param, RX-filter + own-AID gates) —
   * kBfeeJaguar23 is transcribed from the vendor hal_txbf_8822b_enter(), i.e.
   * it IS the 8822B recipe. Must come after bring_up: the recipe RMWs the
   * RXFLTMAP registers init_wmac_cfg writes. See BeamformingSounder.h. */
  if (_cfg.bf.beamformee_of) {
    const uint8_t *mac = _cfg.bf.beamformee_of->data();
    /* Jaguar-2 bring-up never programs the self-MAC (0x0610), so the NDPA
     * RA has nothing to match. Give the beamformee a known identity here so
     * the sounder can address it; log it for the test harness. */
    static const uint8_t kBfeeMac[6] = {0x00, 0xe0, 0x4c, 0x88, 0x22, 0xbb};
    for (uint16_t i = 0; i < 6; ++i)
      _device.rtw_write8(0x0610 + i, kBfeeMac[i]);
    /* bf.mu upgrades the responder to an MU beamformee, whose report appends
     * the per-subcarrier delta-SNR (MU Exclusive Beamforming Report) the SU
     * report omits. */
    if (_cfg.bf.mu) {
      devourer::bf::arm_beamformee_mu(_device, mac, devourer::bf::kBfeeJaguar23);
      _logger->info("Jaguar2 BF MU-beamformee armed for beamformer "
                    "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} — "
                    "beamformee MAC 00:e0:4c:88:22:bb",
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else {
      devourer::bf::arm_beamformee(_device, mac, devourer::bf::kBfeeJaguar23);
      _logger->info("Jaguar2 BF beamformee armed for beamformer "
                    "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} — "
                    "beamformee MAC 00:e0:4c:88:22:bb",
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
  }

  if (_cfg.debug.bb_dump) {
    /* Full BB/RF register dump in the vendor rtw_proc format for canary diff
     * against the kernel driver (tests/jaguar2_rx_canary.sh). */
    for (uint32_t a = 0x0; a <= 0x7fc; a += 0x10)
      _logger->info("BBDUMP 0x{:04x} 0x{:08x} 0x{:08x} 0x{:08x} 0x{:08x}", a,
                    _device.rtw_read32(a), _device.rtw_read32(a + 4),
                    _device.rtw_read32(a + 8), _device.rtw_read32(a + 12));
    for (uint32_t a = 0x800; a <= 0x2ffc; a += 0x10)
      _logger->info("BBDUMP 0x{:04x} 0x{:08x} 0x{:08x} 0x{:08x} 0x{:08x}", a,
                    _device.rtw_read32(a), _device.rtw_read32(a + 4),
                    _device.rtw_read32(a + 8), _device.rtw_read32(a + 12));
    for (uint32_t r = 0x0; r <= 0xff; r++)
      _logger->info("RFDUMP 0x{:02x} A=0x{:05x} B=0x{:05x}", r,
                    _hal.dbg_rf_read(0, r), _hal.dbg_rf_read(1, r));
  }

  StartRxLoop(std::move(packetProcessor));
}

void RtlJaguar2Device::StartRxLoop(Action_ParsedRadioPacket packetProcessor) {
  _packetProcessor = std::move(packetProcessor);
  /* Restartable: clear any stop request left by a prior StopRxLoop(). */
  _rx_stop = false;
  /* Start the DIG thread: track IGI to the false-alarm rate so weak signals are
   * caught without an FA storm (a fixed IGI can't span the range). */
  _dig_stop = false;
  if (!_cfg.tuning.skip_dig) {
    _dig_thread = std::thread([this] {
      while (!_dig_stop && !g_devourer_should_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        _hal.dig_step();
      }
    });
    _logger->info("RtlJaguar2Device: DIG thread started");
  }

  /* DEVOURER_RX_CSI_MASK / DEVOURER_RX_NBI — RX-side per-subcarrier masking
   * (receive-equalizer CSI mask / narrowband notch). Applied after the channel
   * set so it is the final word for a single-channel capture. See ToneMask.h. */
  devourer::tonemask::apply(
      _device, _logger, devourer::tonemask::Family::AC2_8822B, _channel, 2,
      _cfg.rx.csi_mask ? _cfg.rx.csi_mask->c_str() : nullptr,
      _cfg.rx.nbi ? _cfg.rx.nbi->c_str() : nullptr);

  _logger->info("RtlJaguar2Device: entering RX loop (ch={})", _channel.Channel);

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
        /* RX desc word2 BIT(28) (GET_RX_DESC_C2H, halmac_rx_desc_nic.h:230) marks
         * a firmware C2H report, not an 802.11 frame. Tag it so the processor
         * skips the SA/frame path (the vendor routes C2H to its own handler and
         * never treats it as a recvframe). Mirrors the Jaguar-1/-3 RX loops. */
        const bool is_c2h = (data[off + 11] & 0x10) != 0;
        p.RxAtrib.pkt_rpt_type = is_c2h ? RX_PACKET_TYPE::C2H_PACKET
                                        : RX_PACKET_TYPE::NORMAL_RX;
        /* Per-frame RSSI/SNR/EVM from the jgr2 PHY-status (present when
         * APP_PHYSTS is on, i.e. drvinfo carries the 32-byte report). CCK rates
         * (DESC_RATE1M..11M = 0..3) use type0, everything else type1. C2H has no
         * phy-status (drvinfo=0), so the size guard already skips it. */
        if (!is_c2h && f.drvinfo_size >= 28)
          jaguar2::parse_phy_sts_jgr2(data + off + jaguar2::RXDESC_SIZE_8822B,
                                      f.drvinfo_size, f.rx_rate <= 3, p.RxAtrib);
        p.Data =
            std::span<uint8_t>(const_cast<uint8_t *>(f.frame), f.frame_len);
        if (!p.RxAtrib.crc_err)
          _rxq.add(p.RxAtrib.rssi[0], p.RxAtrib.snr[0], p.RxAtrib.evm[0]);
        _packetProcessor(p);
      }
      ++frames;
      if (f.next_offset == 0)
        break;
      off += f.next_offset;
    }
  };
  _device.bulk_read_async_loop(32 * 1024, 8, on_data, [this]() -> bool {
    return _rx_stop || g_devourer_should_stop;
  });
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
  /* DEVOURER_TX_PWR=0xNN forces a flat per-rate TXAGC reference (SDR-visibility
   * debug knob) over the efuse-calibrated level bring_up applied — routed
   * through the runtime flat-override knob so it composes with the offset and
   * shows up in GetTxPowerState. */
  if (_cfg.tx.power_index)
    SetTxPowerIndexOverride(*_cfg.tx.power_index & 0x3f);
  /* DEVOURER_BF_ARM_SOUNDER — beamforming self-sounding (beamformer side):
   * arm the MAC's hardware sounding engine so a TX-descriptor-marked NDPA
   * (DEVOURER_TX_NDPA=1) is followed by a hardware-generated NDP. The MAC
   * sounding registers are family-neutral (BeamformingSounder.h); the
   * sounding-protocol control byte is 0xDB (hal_txbf_8822b_enter — Jaguar-1
   * uses 0xCB). No RF mode-table poke here, unlike Jaguar-3: the vendor's
   * hal_txbf_8822b_rf_mode() body is entirely #if 0'd out. */
  if (_cfg.bf.arm_sounder || _cfg.bf.sounder_self_mac) {
    /* Jaguar-2 bring-up never programs the self-MAC (0x0610); the sounding
     * engine matches the injected NDPA's TA against it before firing the NDP.
     * bf.sounder_self_mac programs that MAC (use the NDPA TA the caller
     * injects — the txdemo's canonical SA); unset leaves it unprogrammed
     * (Jaguar-1 semantics). */
    if (_cfg.bf.sounder_self_mac) {
      const uint8_t *m = _cfg.bf.sounder_self_mac->data();
      for (uint16_t i = 0; i < 6; ++i)
        _device.rtw_write8(0x0610 + i, m[i]);
      _logger->info("Jaguar2 BF sounder: self-MAC programmed to "
                    "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                    m[0], m[1], m[2], m[3], m[4], m[5]);
    }
    devourer::bf::arm_sounder(_device, /*snd_ptcl_ctrl=*/0xDB);
    _logger->info("Jaguar2 BF sounder armed (beamformer side)");
  }
  /* DEVOURER_CW_TONE — radiate a bare RF LO carrier at the channel center (MP
   * single-tone). The channel was tuned in bring_up, so the LO sits at the
   * center frequency. DEVOURER_CW_TONE_GAIN=0..31 sets RF 0x00[4:0]. */
  if (_cfg.tx.cw_tone)
    StartCwTone(_cfg.tx.cw_tone_gain & 0x1F);
  _logger->info("Jaguar2: ready for TX (monitor inject, ch={})",
                channel.Channel);
}

/* MP single-tone (CW carrier), Jaguar2 path A. Ported from the vendor
 * hal_mpt_SetSingleToneTx() 8822B / 8821C branches: disable the OFDM/CCK
 * modulators, force RF path A into TX mode at `gain`, enable the LO, and set the
 * per-variant RFE pinmux so the TX path is keyed — leaving a clean local-
 * oscillator carrier at the tuned channel center. The 1T1R 8821C differs from
 * the 2T2R 8822B in the RFE pinmux and in the LO-enable register (RF 0x75[16] in
 * 2.4 GHz BTG mode vs RF 0x58[1]); both branch on _variant below. The pre-tone
 * RF/RFE state is snapshotted for StopCwTone(). */
void RtlJaguar2Device::StartCwTone(uint8_t gain) {
  if (_cw_active)
    return;

  /* RF register addresses (named locally — the Jaguar2 HAL is self-contained and
   * does not pull in Hal8812PhyReg.h). RFREGOFFSETMASK (0xFFFFF) reads/writes
   * the full 20-bit RF register. */
  constexpr uint32_t RF_AC = 0x00;             /* RF mode / gain */
  constexpr uint32_t RF_LNA_LOW_GAIN_3 = 0x58; /* RF LO enable (bit1) */
  /* BB registers. */
  constexpr uint16_t REG_OFDMCCKEN = 0x808;
  constexpr uint16_t rA_RFE_Pinmux = 0xCB0;
  constexpr uint16_t rB_RFE_Pinmux = 0xEB0;
  constexpr uint16_t rA_RFE_Inverse = 0xCBC;
  constexpr uint16_t rB_RFE_Inverse = 0xEBC;
  constexpr uint32_t bMaskDWord = 0xFFFFFFFF;
  constexpr uint32_t bMaskLWord = 0x0000FFFF;

  /* Snapshot for a clean restore: RF 0x00 (full 20-bit) + the four RFE-pinmux
   * words (BB reads are full-dword via rtw_read32). */
  _cw_rf00 = _hal.dbg_rf_read(/*path A*/ 0, RF_AC);
  _cw_bb[0] = _device.rtw_read32(rA_RFE_Pinmux);
  _cw_bb[1] = _device.rtw_read32(rB_RFE_Pinmux);
  _cw_bb[2] = _device.rtw_read32(rA_RFE_Pinmux + 4);
  _cw_bb[3] = _device.rtw_read32(rB_RFE_Pinmux + 4);

  /* Disable OFDM + CCK modulators (0x808[29:28] = 0). */
  _device.phy_set_bb_reg(REG_OFDMCCKEN, (1u << 28) | (1u << 29), 0x0);

  /* RF path A -> TX mode (0x00[19:16]=2), gain index (0x00[4:0]). */
  _hal.dbg_rf_set(/*path A*/ 0, RF_AC, 0xF0000, 0x2);
  _hal.dbg_rf_set(/*path A*/ 0, RF_AC, 0x1F, gain & 0x1F);

  const bool is_8821c = _variant == jaguar2::ChipVariant::C8821C;
  if (is_8821c) {
    /* 8821C (1T1R) LO enable: on 2.4 GHz the RF is in BTG (shared BT/G-band)
     * mode and the LO gate is RF 0x75[16]; on 5 GHz it is the normal RF 0x58[1]
     * (hal_mpt_SetSingleToneTx 8821C branch). */
    if (_channel.Channel <= 14)
      _hal.dbg_rf_set(/*path A*/ 0, 0x75, (1u << 16), 0x1);
    else
      _hal.dbg_rf_set(/*path A*/ 0, RF_LNA_LOW_GAIN_3, (1u << 1), 0x1);
    /* 8821C RFE pinmux — path A only. (ExternalPA_2G/5G parts also set
     * 0xCB4[0xA00000]=0x1; this HAL doesn't expose those flags and the validated
     * CF-811AC has no external PA, so that optional write is skipped.) */
    _device.phy_set_bb_reg(rA_RFE_Pinmux, 0xF0F0, 0x707);
  } else {
    /* 8822B LO enable (RF 0x58[1]) + RFE pinmux + inverse to force the TX path. */
    _hal.dbg_rf_set(/*path A*/ 0, RF_LNA_LOW_GAIN_3, (1u << 1), 0x1);
    _device.phy_set_bb_reg(rA_RFE_Pinmux, bMaskDWord, 0x77777777);
    _device.phy_set_bb_reg(rB_RFE_Pinmux, bMaskDWord, 0x77777777);
    _device.phy_set_bb_reg(rA_RFE_Pinmux + 4, bMaskLWord, 0x7777);
    _device.phy_set_bb_reg(rB_RFE_Pinmux + 4, bMaskLWord, 0x7777);
    _device.phy_set_bb_reg(rA_RFE_Inverse, 0xFFF, 0xb);
    _device.phy_set_bb_reg(rB_RFE_Inverse, 0xFFF, 0x830);
  }

  _cw_active = true;
  _logger->info("CW single-tone armed @ ch{} gain={} ({})", _channel.Channel,
                static_cast<int>(gain & 0x1F), is_8821c ? "8821C" : "8822B");
}

/* Mirror of the vendor STOP path: re-enable OFDM/CCK, restore RF 0x00 and the
 * RFE-pinmux words captured in StartCwTone(), disable the LO, and clear the
 * 8822B RFE-inverse pins. */
void RtlJaguar2Device::StopCwTone() {
  if (!_cw_active)
    return;

  constexpr uint32_t RF_AC = 0x00;
  constexpr uint32_t RF_LNA_LOW_GAIN_3 = 0x58;
  constexpr uint32_t RF_OFFSET_MASK = 0x000FFFFF;
  constexpr uint16_t REG_OFDMCCKEN = 0x808;
  constexpr uint16_t rA_RFE_Pinmux = 0xCB0;
  constexpr uint16_t rB_RFE_Pinmux = 0xEB0;
  constexpr uint16_t rA_RFE_Inverse = 0xCBC;
  constexpr uint16_t rB_RFE_Inverse = 0xEBC;
  constexpr uint32_t bMaskDWord = 0xFFFFFFFF;

  _device.phy_set_bb_reg(REG_OFDMCCKEN, (1u << 28) | (1u << 29), 0x3);
  _device.phy_set_bb_reg(rA_RFE_Pinmux, bMaskDWord, _cw_bb[0]);
  _device.phy_set_bb_reg(rB_RFE_Pinmux, bMaskDWord, _cw_bb[1]);
  _device.phy_set_bb_reg(rA_RFE_Pinmux + 4, bMaskDWord, _cw_bb[2]);
  _device.phy_set_bb_reg(rB_RFE_Pinmux + 4, bMaskDWord, _cw_bb[3]);
  _hal.dbg_rf_set(/*path A*/ 0, RF_AC, RF_OFFSET_MASK, _cw_rf00);
  if (_variant == jaguar2::ChipVariant::C8821C) {
    /* 8821C: disable both LO gates (BTG RF 0x75[16] + normal RF 0x58[1]); no
     * RFE-inverse to restore. */
    _hal.dbg_rf_set(/*path A*/ 0, 0x75, (1u << 16), 0x0);
    _hal.dbg_rf_set(/*path A*/ 0, RF_LNA_LOW_GAIN_3, (1u << 1), 0x0);
  } else {
    _hal.dbg_rf_set(/*path A*/ 0, RF_LNA_LOW_GAIN_3, (1u << 1), 0x0);
    _device.phy_set_bb_reg(rA_RFE_Inverse, 0xFFF, 0x0);
    _device.phy_set_bb_reg(rB_RFE_Inverse, 0xFFF, 0x0);
  }

  _cw_active = false;
  _logger->info("CW single-tone stopped — chip restored");
}

/* Modulated continuous TX. On Jaguar2 the vendor 0x914 continuous-TX register
 * mode wedges the USB TX FIFO (bulk-OUT NAKs) rather than holding a carrier, so
 * the hardware-continuous path is NOT engaged here (unlike Jaguar1). Instead we
 * apply the rate and let the caller's back-to-back send_packet loop
 * (DEVOURER_TX_GAP_US=0) supply the modulated stimulus at beacon duty — enough
 * for the link probe's per-frame SNR/EVM read. True 100%-duty HW continuous on
 * Jaguar2 is a documented follow-up (needs the FIFO-safe MP setup). */
void RtlJaguar2Device::StartContinuousTx(const devourer::TxMode &mode) {
  if (_cont_active)
    return;
  SetTxMode(mode);

  /* Jaguar2 HW continuous TX. Setting 0x914[18:16]=1 alone wedges the USB TX
   * FIFO; the missing piece is rCCAonSec (0x838)=0x6d (the vendor sets it before
   * continuous mode, ioctl_mp.c). With it, the 0x914 continuous mode radiates a
   * 100%-duty modulated carrier from BB state (SDR-verified: flat ~18 MHz OFDM
   * block) — no send_packet feed required (the demo idle-holds). */
  constexpr uint16_t REG_RFMOD = 0x800;       /* bOFDMEn = bit25 */
  constexpr uint16_t REG_CCK0_SYSTEM = 0xa00; /* [1:0]=BBmode, [3]=scramble */
  constexpr uint16_t REG_CCAonSec = 0x838;
  constexpr uint16_t REG_CONT_TX = 0x914;     /* [18:16] = OFDM_TX_MODE */

  _cont_cca838 = _device.rtw_read32(REG_CCAonSec);
  _device.phy_set_bb_reg(REG_CCAonSec, 0xff, 0x6d);
  if (!(_device.rtw_read32(REG_RFMOD) & 0x2000000u))
    _device.phy_set_bb_reg(REG_RFMOD, 0x2000000u, 1); /* OFDM block on */
  _device.phy_set_bb_reg(REG_CCK0_SYSTEM, 0x3u, 0);   /* CCK test mode off */
  _device.phy_set_bb_reg(REG_CCK0_SYSTEM, 0x8u, 1);   /* scrambler on */
  _device.phy_set_bb_reg(REG_CONT_TX, (7u << 16), 0x1); /* OFDM_ContinuousTx */

  _cont_active = true;
  _logger->info("Modulated continuous TX armed @ ch{} (Jaguar2 HW 100%%-duty "
                "carrier; idle-hold, StopContinuousTx to end)",
                _channel.Channel);
}

void RtlJaguar2Device::StopContinuousTx() {
  if (!_cont_active)
    return;
  constexpr uint16_t REG_CONT_TX = 0x914;
  constexpr uint16_t REG_PMAC_RESET = 0x100; /* bBBResetB = bit8 */
  constexpr uint16_t REG_CCAonSec = 0x838;

  _device.phy_set_bb_reg(REG_CONT_TX, (7u << 16), 0x0);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  _device.phy_set_bb_reg(REG_PMAC_RESET, 0x100u, 0x0);
  _device.phy_set_bb_reg(REG_PMAC_RESET, 0x100u, 0x1);
  _device.phy_set_bb_reg(REG_CCAonSec, 0xff, _cont_cca838 & 0xff);
  _cont_active = false;
  _logger->info("Modulated continuous TX stopped — chip restored");
}

void RtlJaguar2Device::SetMonitorChannel(SelectedChannel channel) {
  _channel = channel;
  /* Retune the RF/BB to the new channel. set_channel_bw is a pure tune (RF18 +
   * bandwidth registers) — no per-channel LCK/IQK/TX-power — so it is cheap
   * enough to drive a live spectrum sweep. The stale IQK from bring-up slightly
   * degrades RX quality on the new channel but does not affect the frame-free
   * CCA/FA energy counters the sweep reads. */
  _hal.set_channel_bw(static_cast<uint8_t>(channel.Channel),
                      static_cast<uint8_t>(channel.ChannelWidth), _rfe,
                      channel.ChannelOffset);
  /* Runtime TX-power knobs in use: re-fold them against the NEW channel's
   * efuse group so the offset stays relative to the calibrated table (TXAGC
   * registers are not per-channel — a cross-group move would otherwise keep
   * the old group's absolute level). Gated on a knob being active so the
   * legacy no-knob path stays byte-identical to the pure tune above. */
  if (_brought_up && (_tx_pwr_offset_steps != 0 || _tx_pwr_override >= 0))
    apply_tx_power_current();
}

void RtlJaguar2Device::FastRetune(uint8_t channel, bool cache_rf) {
  if (channel == _channel.Channel)
    return;
  if (_hal.fast_retune(channel, static_cast<uint8_t>(_channel.ChannelWidth),
                       _channel.ChannelOffset, cache_rf)) {
    _channel.Channel = channel;
    return;
  }
  /* Fast path declined (band change / never tuned) — full channel set at the
   * current bandwidth + offset. */
  _channel.Channel = channel;
  _hal.set_channel_bw(channel, static_cast<uint8_t>(_channel.ChannelWidth),
                      _rfe, _channel.ChannelOffset);
}

RxEnergy RtlJaguar2Device::GetRxEnergy() {
  /* Scalar FA/CCA/IGI come from the DIG thread's cached snapshot (no USB);
   * append a fresh NHM power histogram (11AC register map). NHM's registers
   * (0x994/0x990/0x998.. + 0xfa8/0xfb4) don't overlap the DIG thread's
   * (0xc50 IGI + FA counters), so the concurrent read is race-free enough for
   * this diagnostic. */
  RxEnergy e = _hal.last_energy();
  devourer::read_nhm(
      devourer::nhm_regs_11ac(), e.igi,
      [this](uint16_t a) { return _device.rtw_read<uint32_t>(a); },
      [this](uint16_t a, uint32_t m, uint32_t v) {
        _device.phy_set_bb_reg(a, m, v);
      },
      e);
  return e;
}

devourer::TxPowerCaps RtlJaguar2Device::GetTxPowerCaps() {
  devourer::TxPowerCaps caps;
  caps.supported = true;
  caps.index_max = 63;
  caps.step_qdb = 2; /* 0.5 dB per TXAGC index step */
  /* On-air slope (tests/txpwr_offset_onair.sh): 8822BU 0.488, 8821CU
   * 0.581 dB/idx @ ch36 — nominal confirmed. */
  caps.step_measured = true;
  caps.offset_min_qdb = -126;
  caps.offset_max_qdb = 126;
  return caps;
}

int RtlJaguar2Device::SetTxPowerOffsetQdb(int qdb) {
  if (_cw_active) {
    _logger->warn("SetTxPowerOffsetQdb refused: CW tone active (TXAGC does "
                  "not modulate a bare LO carrier)");
    return 0;
  }
  int steps = 0;
  const int applied = devourer::quantize_offset_qdb(qdb, GetTxPowerCaps(),
                                                    &steps);
  _tx_pwr_offset_steps = steps;
  if (_brought_up)
    apply_tx_power_current();
  _logger->info("TX-power offset: {} qdB requested -> {} qdB applied "
                "({} steps){}",
                qdb, applied, steps,
                _brought_up ? "" : " (recorded; applies at bring-up)");
  return applied;
}

void RtlJaguar2Device::SetTxPowerIndexOverride(int idx) {
  if (_cw_active) {
    _logger->warn("SetTxPowerIndexOverride refused: CW tone active");
    return;
  }
  _tx_pwr_override = idx < 0 ? -1 : (idx > 63 ? 63 : idx);
  if (_brought_up)
    apply_tx_power_current();
}

bool RtlJaguar2Device::ReApplyTxPower() {
  if (!_brought_up || _cw_active)
    return false;
  apply_tx_power_current();
  return true;
}

devourer::TxPowerState RtlJaguar2Device::GetTxPowerState() {
  devourer::TxPowerState s;
  s.valid = true;
  s.flat_index = static_cast<int16_t>(_tx_pwr_override.load());
  s.offset_steps = static_cast<int16_t>(_tx_pwr_offset_steps.load());
  s.offset_qdb = static_cast<int16_t>(s.offset_steps * 2);
  s.saturated_low = _hal.txpwr_saturated_low();
  s.saturated_high = _hal.txpwr_saturated_high();
  /* The Jaguar2 TXAGC block (0x1d00/0x1d80) is write-only — reads return 0
   * (hardware-verified on 8822BU + 8821CU) — so the representative indices
   * come from the HAL's software shadow of the last apply, like the 8814A. */
  int cck = -1, ofdm = -1, mcs7 = -1;
  _hal.txagc_shadow(cck, ofdm, mcs7);
  s.cck_index = static_cast<int16_t>(cck);
  s.ofdm_index = static_cast<int16_t>(ofdm);
  s.mcs7_index = static_cast<int16_t>(mcs7);
  s.hw_readback = false;
  return s;
}

void RtlJaguar2Device::SetTxPacketPowerStep(uint8_t step) {
  _tx_pkt_pwr_step = step & 0x7;
  _logger->info("Jaguar2: per-packet TXPWR_OFSET default step = {} "
                "(0=none 1=-3 2=-7 3=-11 4=+3 5=+6 dB)",
                step & 0x7);
}

devourer::TxCaps RtlJaguar2Device::GetTxCaps() {
  /* 8821C is 1T1R (no STBC); 8822B is 2T2R. */
  const uint8_t chains = _variant == jaguar2::ChipVariant::C8821C ? 1 : 2;
  return devourer::tx_caps_for_chains(chains);
}

devourer::EfuseStability RtlJaguar2Device::ProbeEfuseStability(int reads) {
  if (!_brought_up)
    return {}; /* supported=false — the efuse read needs a powered chip */
  constexpr uint16_t kMapLen = 0x200;
  auto st = devourer::ProbeEfuseStabilityImpl(
      [this](uint8_t *buf) {
        _hal.read_efuse_logical_map(buf, kMapLen, /*dump=*/false);
        return true;
      },
      kMapLen, reads);
  _logger->info(
      "efuse-stability: reads={} mismatched={} invalid_id={} id=0x{:04x}",
      st.reads, st.mismatched_reads, st.invalid_id_reads, st.eeprom_id);
  return st;
}

devourer::ThermalStatus RtlJaguar2Device::GetThermalStatus() {
  devourer::ThermalStatus t;
  if (!_brought_up)
    return t; /* RF reads need a powered chip */
  uint8_t raw = 0, baseline = 0xFF;
  _hal.read_thermal(raw, baseline);
  t.raw = raw;
  t.baseline = baseline;
  t.valid = baseline != 0xFF;
  t.delta = t.valid ? static_cast<int>(raw) - static_cast<int>(baseline) : 0;
  return t;
}

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
  /* Per-packet hop target from a radiotap CHANNEL field (0 = none present). */
  int radiotap_channel = 0;
  /* Per-packet TX-power delta (dB) from a radiotap DBM_TX_POWER field; on an
   * INJECTED frame we treat it as a delta to the rate's calibrated power and
   * quantize to the descriptor TXPWR_OFSET LUT (the vendor skips this field, so
   * there is no absolute-dBm convention to violate). INT_MIN = not present. */
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
       * the RATE/MCS/VHT fields). Same contract as the Jaguar1/Jaguar3 paths. */
      radiotap_channel =
          devourer::freq_to_chan(get_unaligned_le16(it.this_arg));
      break;
    case IEEE80211_RADIOTAP_DBM_TX_POWER:
      /* Signed 1-byte dB — the per-packet power delta (see the field decl). */
      radiotap_pkt_pwr_db = static_cast<int8_t>(*it.this_arg);
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

  /* Radiotap CHANNEL is authoritative for per-packet frequency, exactly as
   * RATE/MCS are for rate: a frame asking for a different channel triggers a
   * lean FastRetune before the descriptor is built (same contract as the
   * Jaguar1/Jaguar3 paths). */
  if (radiotap_channel > 0 && radiotap_channel != _channel.Channel)
    FastRetune(static_cast<uint8_t>(radiotap_channel), /*cache_rf=*/true);

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

  /* Sub-channel (rtl8821c_sc_mapping): when the frame BW is narrower than the
   * tuned channel BW, tell the PHY which 20/40 MHz slice it occupies so the
   * frame lands on the primary rather than the block centre. Same-BW -> 0
   * (DONT_CARE), byte-identical to the prior behaviour. _channel.ChannelOffset
   * is the primary_ch_idx (40 MHz: 1=lower/2=upper; 80 MHz: 1..4 lowest..
   * highest, per off80 = +6/+2/-2/-6). VHT_DATA_SC_* codes from rtw_rf.h. */
  uint8_t data_sc = 0;
  {
    const uint8_t pidx = _channel.ChannelOffset;
    if (_channel.ChannelWidth == CHANNEL_WIDTH_80) {
      if (bwidth == CHANNEL_WIDTH_40)
        data_sc = (pidx <= 2) ? 10 : 9; /* 40 LOWER / UPPER of 80 */
      else if (bwidth == CHANNEL_WIDTH_20)
        data_sc = (pidx == 1)   ? 4      /* 20 LOWEST  */
                  : (pidx == 2) ? 2      /* 20 LOWER   */
                  : (pidx == 3) ? 1      /* 20 UPPER   */
                  : (pidx == 4) ? 3      /* 20 UPPERST */
                                : 0;
    } else if (_channel.ChannelWidth == CHANNEL_WIDTH_40 &&
               bwidth == CHANNEL_WIDTH_20) {
      data_sc = (pidx == 2) ? 1 : (pidx == 1) ? 2 : 0; /* 20 UPPER/LOWER of 40 */
    }
  }

  /* Drop STBC on the 1T1R variant (8821C) — STBC needs >=2 TX chains, so an
   * STBC-marked frame there is malformed and never decodes. Warn once. */
  if (stbc && !GetTxCaps().stbc_ok) {
    static bool warned = false;
    if (!warned) {
      _logger->warn("STBC requested on a 1T1R chip (8821C) — dropping the STBC "
                    "flag to keep frames decodable");
      warned = true;
    }
    stbc = 0;
  }

  const uint8_t *dot11 = packet + radiotap_length;
  bool bmc = frame_len >= 6 && (dot11[4] & 0x01);

  /* 802.11 header length for the descriptor WHEADER_LEN field: base 24 B, +2
   * for QoS data, +6 for 4-address (ToDS+FromDS). */
  uint8_t hdrlen = 24;
  if (frame_len >= 2) {
    uint8_t fc0 = dot11[0], fc1 = dot11[1];
    if ((fc0 & 0x0c) == 0x08 && (fc0 & 0x80))
      hdrlen += 2; /* QoS data */
    if ((fc1 & 0x03) == 0x03)
      hdrlen += 6; /* 4-address */
  }

  /* DEVOURER_TX_NDPA=1 — beamforming self-sounding: mark injected frames as
   * NDPA so the armed sounding engine (DEVOURER_BF_ARM_SOUNDER, InitWrite)
   * follows each with a hardware-generated NDP. Same knob as Jaguar-1/-3. */
  const bool ndpa = _cfg.bf.ndpa_period > 0;
  /* Per-packet TX power: a radiotap DBM_TX_POWER on this frame (quantized to
   * the descriptor LUT) wins per-packet; otherwise the SetTxPacketPowerStep
   * default. Zero cost — a descriptor bitfield, no register write. */
  const uint8_t pkt_pwr_step =
      radiotap_pkt_pwr_db != INT_MIN
          ? jaguar2::txpkt_pwr_step_for_db(radiotap_pkt_pwr_db)
          : _tx_pkt_pwr_step.load();
  std::vector<uint8_t> usb_frame(jaguar2::TXDESC_SIZE_8822B + frame_len, 0);
  jaguar2::fill_data_tx_desc_8822b(
      usb_frame.data(), static_cast<uint16_t>(frame_len),
      MRateToHwRate(fixed_rate), rate_id, bw_desc, sgi != 0, ldpc != 0, stbc,
      bmc, static_cast<uint8_t>(hdrlen >> 1), ndpa, data_sc, pkt_pwr_step);
  std::memcpy(usb_frame.data() + jaguar2::TXDESC_SIZE_8822B, dot11, frame_len);

  int rc = _device.bulk_send_sync_ep(_device.first_bulk_out_ep(),
                                     usb_frame.data(), usb_frame.size(),
                                     /*timeout_ms=*/20);
  return rc >= 0;
}

SelectedChannel RtlJaguar2Device::GetSelectedChannel() { return _channel; }

void RtlJaguar2Device::Stop() { stop_dig(); }

void RtlJaguar2Device::SetTxMode(const devourer::TxMode &mode) {
  _tx_mode_default = mode;
}

void RtlJaguar2Device::ClearTxMode() { _tx_mode_default.reset(); }
