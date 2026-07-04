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

#include "BeamformingSounder.h"
#include "FrameParserJaguar2.h"
#include "Jaguar2Calibration.h"
#include "ToneMask.h"
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

RtlJaguar2Device::RtlJaguar2Device(RtlUsbAdapter device, Logger_t logger,
                                   jaguar2::ChipVariant variant)
    : _device{device}, _logger{logger}, _variant{variant},
      _hal{device, logger, variant}, _macinit{device, logger, variant},
      _fw{device, logger, variant} {}

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
  /* The kernel uses rfe_type=3 for the T3U even with an unprogrammed efuse
   * (devourer reads 0xff -> defaults 0). rfe_type selects the BB/RF phydm
   * conditional blocks AND the RFE antenna-switch pins; the wrong variant leaves
   * the TX front-end mis-routed. DEVOURER_RFE=N overrides. */
  if (const char *e = getenv("DEVOURER_RFE"))
    rfe = static_cast<uint8_t>(strtol(e, nullptr, 0));
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
  if (!getenv("DEVOURER_SKIP_IQK")) {
    auto cal = jaguar2::make_jaguar2_calibration(
        _variant, _device, _logger, _hal.chip_version().cut,
        _hal.chip_version().rf_2t2r != 0);
    cal->iqk_trigger(channel.Channel <= 14);
  } else {
    _logger->info("Jaguar2: IQK SKIPPED (DEVOURER_SKIP_IQK)");
  }
  /* Re-assert the TX/RX antenna-path routing AFTER IQK. In the vendor flow
   * config_phydm_trx_mode runs from the post-calibration channel-set (PHY_SwChnl),
   * i.e. AFTER IQK — devourer ran it before IQK, and IQK's TX-path loopback +
   * one-shot apply-bit toggling (0xc94/0x80c/0x93c) leave the TX antenna path
   * disturbed with nothing to restore it (RX is unaffected, which is why RX
   * worked but TX radiated no valid frame). Gated for A/B bisection. */
  if (!getenv("DEVOURER_SKIP_TRX_REASSERT")) {
    _hal.config_trx_mode();
    _logger->info("Jaguar2: TX/RX path re-asserted post-IQK");
  }
  /* phydm DM-init RFE mux + TXBF init — the kernel runs these from
   * odm_dm_init (phydm_rfe_init) and rtl8822b_phy_bf_init after calibration.
   * devourer previously skipped both, leaving 0x1990=0 (RFE source mux) and
   * 0x1c94=0x5fff5fff (BB-table default, not the bf_init 0xafffafff). */
  if (!getenv("DEVOURER_SKIP_RFEINIT")) {
    _hal.rfe_init();
    _hal.bf_init();
  }
  /* Program per-rate TXAGC from the EFUSE power-by-rate calibration (the level
   * the kernel uses). DEVOURER_TX_PWR (flat override) applied later in InitWrite
   * still wins for debug. DEVOURER_SKIP_TXPWR keeps the BB-table default. */
  if (!getenv("DEVOURER_SKIP_TXPWR"))
    _hal.apply_tx_power(static_cast<uint8_t>(channel.Channel), bw, rfe);
  /* Grant the antenna to WLAN (combo chip) — must precede enable. */
  if (!getenv("DEVOURER_SKIP_COEX"))
    _hal.coex_wlan_only(channel.Channel > 14);
  else
    _logger->info("Jaguar2: coex WL grant SKIPPED (DEVOURER_SKIP_COEX)");
  _hal.enable_rx(); /* CR MACTX|MACRX + RCR + IGI — enables both TX and RX */
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
  if (const char *bfer = std::getenv("DEVOURER_BF_ARM_BFEE")) {
    unsigned m[6];
    if (std::sscanf(bfer, "%x:%x:%x:%x:%x:%x", &m[0], &m[1], &m[2], &m[3],
                    &m[4], &m[5]) == 6) {
      uint8_t mac[6];
      for (int i = 0; i < 6; ++i) mac[i] = static_cast<uint8_t>(m[i]);
      /* Jaguar-2 bring-up never programs the self-MAC (0x0610), so the NDPA
       * RA has nothing to match. Give the beamformee a known identity here so
       * the sounder can address it; log it for the test harness. */
      static const uint8_t kBfeeMac[6] = {0x00, 0xe0, 0x4c, 0x88, 0x22, 0xbb};
      for (uint16_t i = 0; i < 6; ++i)
        _device.rtw_write8(0x0610 + i, kBfeeMac[i]);
      /* DEVOURER_BF_ARM_BFEE_MU=1 upgrades the responder to an MU beamformee,
       * whose report appends the per-subcarrier delta-SNR (MU Exclusive
       * Beamforming Report) the SU report omits. Pair with the sounder's
       * DEVOURER_TX_NDPA_MU=1 (MU feedback bit in the NDPA STA-info). */
      if (std::getenv("DEVOURER_BF_ARM_BFEE_MU")) {
        devourer::bf::arm_beamformee_mu(_device, mac, devourer::bf::kBfeeJaguar23);
        _logger->info("Jaguar2 BF MU-beamformee armed for beamformer {} — "
                      "beamformee MAC 00:e0:4c:88:22:bb", bfer);
      } else {
        devourer::bf::arm_beamformee(_device, mac, devourer::bf::kBfeeJaguar23);
        _logger->info("Jaguar2 BF beamformee armed for beamformer {} — "
                      "beamformee MAC 00:e0:4c:88:22:bb", bfer);
      }
    } else {
      _logger->error("DEVOURER_BF_ARM_BFEE — bad MAC '{}'", bfer);
    }
  }

  if (getenv("DEVOURER_BB_DUMP")) {
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
  if (!getenv("DEVOURER_SKIP_DIG")) {
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
  devourer::tonemask::apply_from_env(_device, _logger,
                                     devourer::tonemask::Family::AC2_8822B,
                                     _channel, 2);

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
        p.RxAtrib.pkt_rpt_type = RX_PACKET_TYPE::NORMAL_RX;
        p.Data =
            std::span<uint8_t>(const_cast<uint8_t *>(f.frame), f.frame_len);
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
   * debug knob) over the efuse-calibrated level bring_up applied. */
  if (const char *e = getenv("DEVOURER_TX_PWR")) {
    uint8_t idx = static_cast<uint8_t>(strtol(e, nullptr, 0) & 0x3f);
    _hal.set_tx_power_flat(idx);
  }
  /* DEVOURER_BF_ARM_SOUNDER — beamforming self-sounding (beamformer side):
   * arm the MAC's hardware sounding engine so a TX-descriptor-marked NDPA
   * (DEVOURER_TX_NDPA=1) is followed by a hardware-generated NDP. The MAC
   * sounding registers are family-neutral (BeamformingSounder.h); the
   * sounding-protocol control byte is 0xDB (hal_txbf_8822b_enter — Jaguar-1
   * uses 0xCB). No RF mode-table poke here, unlike Jaguar-3: the vendor's
   * hal_txbf_8822b_rf_mode() body is entirely #if 0'd out. */
  if (const char *snd = std::getenv("DEVOURER_BF_ARM_SOUNDER")) {
    /* Jaguar-2 bring-up never programs the self-MAC (0x0610); the sounding
     * engine matches the injected NDPA's TA against it before firing the NDP.
     * DEVOURER_BF_ARM_SOUNDER=aa:bb:cc:dd:ee:ff programs that MAC (use the
     * NDPA TA the caller injects — the txdemo's canonical SA); a bare "1"
     * leaves it unprogrammed (Jaguar-1 semantics). */
    unsigned m[6];
    if (std::sscanf(snd, "%x:%x:%x:%x:%x:%x", &m[0], &m[1], &m[2], &m[3],
                    &m[4], &m[5]) == 6) {
      for (uint16_t i = 0; i < 6; ++i)
        _device.rtw_write8(0x0610 + i, static_cast<uint8_t>(m[i]));
      _logger->info("Jaguar2 BF sounder: self-MAC programmed to {}", snd);
    }
    devourer::bf::arm_sounder(_device, /*snd_ptcl_ctrl=*/0xDB);
    _logger->info("Jaguar2 BF sounder armed (beamformer side)");
  }
  /* DEVOURER_CW_TONE — radiate a bare RF LO carrier at the channel center (MP
   * single-tone). The channel was tuned in bring_up, so the LO sits at the
   * center frequency. DEVOURER_CW_TONE_GAIN=0..31 sets RF 0x00[4:0]. */
  if (std::getenv("DEVOURER_CW_TONE")) {
    uint8_t g = 0;
    if (const char *e = std::getenv("DEVOURER_CW_TONE_GAIN"))
      g = static_cast<uint8_t>(std::atoi(e)) & 0x1F;
    StartCwTone(g);
  }
  _logger->info("Jaguar2: ready for TX (monitor inject, ch={})",
                channel.Channel);
}

/* MP single-tone (CW carrier), Jaguar2 (RTL8822BU) path A. Ported from the
 * vendor hal_mpt_SetSingleToneTx() 8822B branch: disable the OFDM/CCK
 * modulators, force RF path A into TX mode at `gain`, flip the LO enable, and
 * set the 8822B RFE pinmux + RFE-inverse so the TX path is keyed — leaving a
 * clean local-oscillator carrier at the tuned channel center. The pre-tone
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

  /* RF path A -> TX mode (0x00[19:16]=2), gain index (0x00[4:0]), LO enable
   * (0x58 bit1). */
  _hal.dbg_rf_set(/*path A*/ 0, RF_AC, 0xF0000, 0x2);
  _hal.dbg_rf_set(/*path A*/ 0, RF_AC, 0x1F, gain & 0x1F);
  _hal.dbg_rf_set(/*path A*/ 0, RF_LNA_LOW_GAIN_3, (1u << 1), 0x1);

  /* 8822B RFE pinmux + inverse to force the TX path. */
  _device.phy_set_bb_reg(rA_RFE_Pinmux, bMaskDWord, 0x77777777);
  _device.phy_set_bb_reg(rB_RFE_Pinmux, bMaskDWord, 0x77777777);
  _device.phy_set_bb_reg(rA_RFE_Pinmux + 4, bMaskLWord, 0x7777);
  _device.phy_set_bb_reg(rB_RFE_Pinmux + 4, bMaskLWord, 0x7777);
  _device.phy_set_bb_reg(rA_RFE_Inverse, 0xFFF, 0xb);
  _device.phy_set_bb_reg(rB_RFE_Inverse, 0xFFF, 0x830);

  _cw_active = true;
  _logger->info("CW single-tone armed @ ch{} gain={} (8822B)", _channel.Channel,
                static_cast<int>(gain & 0x1F));
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
  _hal.dbg_rf_set(/*path A*/ 0, RF_LNA_LOW_GAIN_3, (1u << 1), 0x0);
  _device.phy_set_bb_reg(rA_RFE_Inverse, 0xFFF, 0x0);
  _device.phy_set_bb_reg(rB_RFE_Inverse, 0xFFF, 0x0);

  _cw_active = false;
  _logger->info("CW single-tone stopped — chip restored");
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
  static const bool ndpa = std::getenv("DEVOURER_TX_NDPA") != nullptr;
  std::vector<uint8_t> usb_frame(jaguar2::TXDESC_SIZE_8822B + frame_len, 0);
  jaguar2::fill_data_tx_desc_8822b(
      usb_frame.data(), static_cast<uint16_t>(frame_len),
      MRateToHwRate(fixed_rate), rate_id, bw_desc, sgi != 0, ldpc != 0, stbc,
      bmc, static_cast<uint8_t>(hdrlen >> 1), ndpa);
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
