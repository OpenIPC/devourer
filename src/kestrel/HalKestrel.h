#ifndef KESTREL_HAL_KESTREL_H
#define KESTREL_HAL_KESTREL_H

#include <array>
#include <cstdint>

#include "ChipVariant.h"
#include "KestrelFw.h"
#include "MacRegAx.h" /* mac_ax role enums */
#include "RtlAdapter.h"
#include "SelectedChannel.h" /* ChannelWidth_t */
#include "logger.h"

struct kestrel_halbb_ctx; /* opaque C handle (hal/halbb/g6) */
struct kestrel_halrf_ctx; /* opaque C handle (hal/halrf/g6) */

namespace kestrel {

/* Parsed logical-efuse contents devourer consumes (see MacRegAx.h for the
 * offsets). Defaults are the vendor "not autoloaded" fallbacks. */
struct EfuseInfo {
  std::array<uint8_t, 6> mac{};
  uint8_t xtal_cap = 0x3F;
  uint8_t rfe_type = 0x01;
  uint8_t thermal_a = 0x22;
  uint8_t thermal_b = 0x22;
  bool autoload_ok = false; /* header parsed to a plausible MAC */
};

/* HalKestrel owns the low-level MAC bring-up for the Kestrel (G6) generation:
 * register-op helpers (poll, xtal_si, efuse), the USB power-on sequence, and
 * the physical→logical efuse dump. Ported from reference/rtl8852bu mac_ax
 * (pwr_seq_func_8852b.c, hw.c, efuse.c, _usb_8852b.c). The firmware-download
 * layer (KestrelFw) and TRX init build on top of this. */
class HalKestrel {
public:
  HalKestrel(RtlAdapter device, Logger_t logger, ChipVariant variant);

  /* USB power-on: usb_pre_init + mac_pwr_on_usb_8852b. Leaves the MAC powered
   * (DMAC/CMAC func-enabled) and ready for firmware download. Returns false on
   * a poll timeout (chip wedged — needs a VBUS cold). The 8852C diverges via
   * power_on_8852c (LDO/SPS/CMAC1/XTAL deltas); the rest is shared. */
  bool power_on();

  /* Dump the physical WiFi efuse (register read loop) and parse it to the
   * logical map, filling `out`. Independent of firmware (driver path only).
   * `raw_phys` (optional) receives the 1536-byte physical map for diagnostics.
   * Must run after power_on (the efuse block is gated by the power sequence). */
  bool read_efuse(EfuseInfo &out, std::array<uint8_t, 1536> *raw_phys = nullptr);

  /* set_enable_bb_rf(1) (hw.c) — release the BB from reset + enable the RF/AFE
   * clocks. The vendor calls this in mac_hal_init right after FWDL and BEFORE
   * sys_init/trx_init, so the BB/RF is live when the firmware starts running.
   * Must run after download_firmware, before trx_dmac_init. */
  void enable_bb_rf();

  /* Firmware download: hci_func_en + DLE/HFC pre-init + FWDL of the
   * cut-appropriate NICCE image. Must run after power_on. `cut` is the chip
   * cut version (from ReadChipInfo / R_AX_SYS_CFG1[15:12]). */
  bool download_firmware(uint8_t cut);

  /* mac_sys_init (init.c:309): DMAC func-en (incl. CRPRT), CMAC clock + func
   * enables (CK_EN then CMAC_FUNC_EN), chip_func_en OCP patch. Vendor order:
   * FWDL -> set_enable_bb_rf -> THIS -> trx init. */
  void mac_sys_init();

  /* intf_init (usb_init_8852b): LFPS filter, RX bulk size, per-endpoint NUMP.
   * Vendor order: runs AFTER trx init (+ feat_init, host-side no-op). */
  void usb_intf_init() { usb_init(); }

  /* The DMAC half of mac_trx_init: re-init the DLE to the NIC-mode (SCC)
   * quota, station scheduler, MPDU processor, security engine. Must run after
   * download_firmware. Returns false on the sta-scheduler poll timeout. */
  bool trx_dmac_init();

  /* The CMAC half of mac_trx_init (RX path): promiscuous RX filter, receiver MAC
   * (RCR channel-enable + DLK timeouts + RX max-len), CCA control, TX
   * subcarrier / RRSR, RX DMA full-mode clear, and USB RX aggregation. Must
   * run after trx_dmac_init. Actual reception also requires the BB/RF/channel
   * bring-up (phy_bb_rf_init + set_channel). */
  bool trx_cmac_rx_init();

  /* SER error-interrupt-mask enable (mac_enable_imr + mac_err_imr_ctrl). The
   * vendor arms this only after the first BB cmd_ofld batches — call it after
   * phy_bb_rf_init, not inside the CMAC init. */
  void enable_ser_imr() { mac_enable_imr(); }

  /* set_host_rpr (hw.c) — program the WD-release / TX-report path so the fw
   * releases WD/PLE pages after each frame is transmitted (STF mode, AGGNUM=121
   * + timeout=255). Without it the mgmt bulk-OUT page pool fills and NAKs after
   * ~103 frames. Vendor order: end of mac_trx_init, after the SER IMR enable. */
  void set_host_rpr();

  /* mac_port_init (mport.c) band-0 port-0, net_type=NO_LINK — enable the CMAC
   * PORT so the transmit engine has a BSS/PTCL context to air frames from.
   * ROOT CAUSE of the ~103-frame mgmt-TX stall: without an enabled port the
   * CMAC accepts frames into the queue but never transmits them (zero TX
   * completions) — PORT_CFG_P0's port-enable gates the transmit engine
   * (mac_port_init, mport.c). Call at the end of the TX bring-up, after
   * cmac_init. */
  void port_init();

  /* set_hw_sch_tx_en (cmac_tx.c): RMW-enable the scheduler contention TX queues
   * (R_AX_CTN_TXEN). A queue whose bit is 0 never wins a TX opportunity, so its
   * frames stall in PLE — the ~103-frame mgmt-TX stall. Call in the TX
   * bring-up. */
  void sch_tx_en();

  /* AX beacon engine (mac_send_bcn_h2c + the AP branch of mac_port_init): hand
   * the fw the beacon body and reconfigure band-0 port-0 for HW-timed AP
   * beaconing (NET_TYPE=AP, BCN_PRCT, BCNTX_EN, TBTT/BCN timing). The MAC then
   * airs the beacon every `interval_tu` TU with the live TSF inserted. Single
   * BSS; the power-save DTIM/HIQ buffering PCFGs are omitted. `rate_ax` = AX
   * datarate for the beacon (e.g. MAC_AX_OFDM6). */
  bool start_beacon(const uint8_t *body, uint32_t len, uint16_t interval_tu,
                    uint8_t bss_color, uint16_t rate_ax);

  /* PHY bring-up: apply the halbb BB register + gain tables and the
   * halrf radio-A/B tables (via the vendored halbb/halrf loaders). `rfe_type`
   * / `cut`
   * select the table variant (from the efuse / chip id). Must run after the
   * MAC TRX init. This programs the baseband + RF registers; channel tuning
   * and the RX loop build on it. */
  bool phy_bb_rf_init(uint8_t rfe_type, uint8_t cut);

  /* Tune the BB + RF to a monitor channel. Ports the
   * halbb ctrl_ch/ctrl_bw/cck_en/bb_reset + the halrf RF18 channel setting.
   * Must run after phy_bb_rf_init. */
  /* `offset` = SelectedChannel.ChannelOffset (HAL_PRIME_CHNL_OFFSET: 1=primary
   * lower / secondary above, 2=primary upper / secondary below); ignored for
   * 20 MHz. For 40 MHz the RF tunes to the block center = primary +/- 2. */
  bool set_channel(uint8_t channel, ChannelWidth_t bw, uint8_t offset = 0,
                   uint8_t band = 0);

  /* Lean intra-band, same-bandwidth retune (frequency hopping): RF channel
   * switch + band-specific gain/RPL re-apply + BB reset, skipping the BB
   * bandwidth config and the per-channel RX-DCK (the slow ~1.2 ms part). Caller
   * must ensure same band + same width (20 MHz). */
  void fast_retune(uint8_t channel);

  /* Lean same-channel 20 <-> 5/10 MHz narrowband toggle: only the BB small-BW
   * field + ACI-detect change (the RF stays in 20 MHz mode for all of 20/5/10,
   * so no RF re-tune, no gain/channel re-apply). ~a BB write + reset vs the full
   * set_channel. Caller must pass 20/5/10 only. */
  void fast_set_bw(ChannelWidth_t bw);

  /* halbb_set_txpwr_dbm_8852b: force a fixed BB TX power target (overriding the
   * per-rate/TSSI tables — the pragmatic injection-driver model, like the
   * Jaguar SetTxPowerIndexOverride). `dbm_q2` is s(9,2): power in 0.25-dB units
   * (e.g. 20 dBm = 80). Two BB writes: 0x09a4[16]=1 arms fixed-dBm mode, then
   * 0x4594[30:22]=target. Devourer otherwise sets no TX power at all, leaving it
   * at the phy_reg-table default (adequate on 2.4 GHz, weak on 5 GHz). */
  void set_txpwr_dbm(int16_t dbm_q2);

  /* Session default TX power (dBm) applied at every set_channel. Override via
   * DEVOURER_TX_PWR (interpreted as whole dBm on Kestrel — distinct from the
   * Jaguar2 TXAGC-index meaning). Default 20 dBm. */
  void set_default_txpwr_dbm(int dbm) {
    _txpwr_dbm_q2 = static_cast<int16_t>(dbm * 4);
  }

  /* Runtime power offset (quarter-dB) relative to the base dBm — the adaptive-
   * link lever. Because the base is already s(9,2) (quarter-dB), the offset adds
   * directly. Stores the offset (sticky across set_channel) and re-applies the
   * effective power NOW at the current channel. */
  void set_txpwr_offset_qdb(int16_t qdb) {
    _txpwr_offset_qdb = qdb;
    set_txpwr_dbm(static_cast<int16_t>(_txpwr_dbm_q2 + _txpwr_offset_qdb));
  }
  int16_t txpwr_offset_qdb() const { return _txpwr_offset_qdb; }
  int16_t txpwr_base_qdb() const { return _txpwr_dbm_q2; }
  int16_t txpwr_effective_qdb() const {
    return static_cast<int16_t>(_txpwr_dbm_q2 + _txpwr_offset_qdb);
  }

  /* Enable the per-user TX report so the fw emits C2H USR_TX_RPT_INFO
   * with the freerun TX-egress timestamps. Call after the TX bring-up; the
   * reports are received + parsed in the RX loop. */
  bool enable_tx_report(uint8_t mode, uint8_t macid = 0, uint8_t port = 0,
                        uint32_t period_us = 100000) {
    return _fw.enable_usr_tx_rpt(mode, macid, port, period_us);
  }

  /* halrf_get_thermal_8852b: read the per-path RF thermal meter (RF 0x42): pulse
   * the trigger bit (BIT19: 1-0-1), settle 200 us, return the 6-bit value
   * [6:1] (0..63 thermal units, ~1.5-2 C each; not absolute deg C). Control-
   * thread only (does RF writes). */
  uint8_t read_thermal(uint8_t path);

  /* Diagnostic: route the fw log to C2H packets (probe packet-C2H delivery). */
  bool enable_fw_log_c2h() { return _fw.enable_fw_log_c2h(); }

  /* Pseudo-STA — register a station-role MACID with the firmware
   * (mac_fw_role_maintain, CREATE). This is the linchpin that lets the fw
   * track the MACID so per-MACID TX features engage (USR_TX_RPT frame-stat,
   * data-frame path, power-by-rate). Call in InitWrite before enable_tx_report.
   * addr-cam / default cctl (the rest of _add_role) land as follow-ups. */
  bool register_sta_role(uint8_t macid = 0, uint8_t band = 0, uint8_t port = 0) {
    return _fw.fw_role_maintain(macid, reg::MAC_AX_SELF_ROLE_CLIENT,
                                reg::MAC_AX_WIFI_ROLE_STATION,
                                reg::MAC_AX_ROLE_CREATE, band, port);
  }

  /* The "rest of _add_role" (role.c) after fw_role_maintain: program the
   * hardware ADDR_CAM entry + the CMAC control table for `macid` so the TX
   * engine has a resolvable BSS/rate/antenna context. Without this the CMAC
   * queues injected frames but never airs them (the mgmt bulk-OUT stalls at
   * ~103 as the PLE page pool fills). `self_mac` is the injected frames' SA.
   * 8852B is single-path: ntx_path_en=1 (RF_PATH_A), path_map_a=0 (from
   * halbb_cfg_cmac_tx_ant_8852b). Default rate OFDM6, broadcast (bmc). */
  bool add_self_sta(const uint8_t self_mac[6], uint8_t macid = 0,
                    uint8_t net_type = reg::MAC_AX_NET_TYPE_NO_LINK) {
    bool ok = _fw.fw_upd_addr_cam(macid, self_mac, net_type, /*addr_cam_idx=*/0,
                                  /*bssid_cam_idx=*/0);
    ok = _fw.fw_upd_cctl_basic(macid, /*addr_cam_idx=*/0, reg::MAC_AX_OFDM6,
                               /*ntx_path_en=*/1, /*path_map_a=*/0,
                               /*bmc=*/true) &&
         ok;
    return ok;
  }

  /* Register the self MACID as an ACCESS POINT role (self_role=AP,
   * wifi_role=AP, net_type=AP), not a client. The fw's AP-side machinery — the
   * beacon engine AND the UL-OFDMA trigger scheduler — keys off the fw role;
   * with a CLIENT role the fw ignores AP-side scheduling even though the port
   * NET_TYPE register is set to AP for beaconing (role.c requires net_type=AP
   * to pair with an AP self_role). The vendor constraint (role.c): for an AP
   * self_role the SMA and TMA must equal the BSSID, so `bssid` is used for all
   * three in the ADDR_CAM entry. Call in place of register_sta_role +
   * add_self_sta when the device acts as an AP. */
  bool register_ap_role(const uint8_t bssid[6], uint8_t macid = 0,
                        uint8_t band = 0, uint8_t port = 0) {
    bool ok = _fw.fw_role_maintain(macid, reg::MAC_AX_SELF_ROLE_AP,
                                   reg::MAC_AX_WIFI_ROLE_AP,
                                   reg::MAC_AX_ROLE_CREATE, band, port);
    ok = _fw.fw_upd_addr_cam(macid, bssid, reg::MAC_AX_NET_TYPE_AP,
                             /*addr_cam_idx=*/0, /*bssid_cam_idx=*/0) &&
         ok;
    ok = _fw.fw_upd_cctl_basic(macid, /*addr_cam_idx=*/0, reg::MAC_AX_OFDM6,
                               /*ntx_path_en=*/1, /*path_map_a=*/0,
                               /*bmc=*/false) &&
         ok;
    return ok;
  }

  /* Register an associated peer STA (an AP-mode client we schedule UL for):
   * the same role + ADDR_CAM + CMAC-control chain as add_self_sta but for a
   * distinct macid and net_type=AP, so the fw tracks the peer and a Trigger's
   * per-user grant scores against its macid. `peer_mac` is the peer's address
   * (the trigger's RA / the grant's target); `addr_cam_idx` must be unique per
   * peer. The 802.11 AID the grant uses is carried per-frame in the trigger
   * (TriggerConfig user aid12), independent of this CAM entry. */
  bool register_peer_sta(const uint8_t peer_mac[6], uint8_t macid,
                         uint8_t addr_cam_idx,
                         uint8_t net_type = reg::MAC_AX_NET_TYPE_AP) {
    bool ok = _fw.fw_role_maintain(macid, reg::MAC_AX_SELF_ROLE_CLIENT,
                                   reg::MAC_AX_WIFI_ROLE_STATION,
                                   reg::MAC_AX_ROLE_CREATE, /*band=*/0,
                                   /*port=*/0);
    ok = _fw.fw_upd_addr_cam(macid, peer_mac, net_type, addr_cam_idx,
                             /*bssid_cam_idx=*/0) &&
         ok;
    ok = _fw.fw_upd_cctl_basic(macid, addr_cam_idx, reg::MAC_AX_OFDM6,
                               /*ntx_path_en=*/1, /*path_map_a=*/0,
                               /*bmc=*/false) &&
         ok;
    return ok;
  }

  /* ---- 802.11ax scheduled-UL levers (KestrelFwSched.cpp encoders) ---- */

  /* Fire one HE Basic Trigger (UL-OFDMA grant) via the F2P command. */
  bool send_trigger(const devourer::TriggerConfig &cfg) {
    return _fw.f2p_trigger(cfg);
  }

  /* Create (act=ADD) / modify (act=MOD) a TWT agreement. mac_ax_twt_act_tp:
   * ADD=0, DEL=1, MOD=2. */
  bool configure_twt(const devourer::TwtConfig &cfg) {
    return _fw.twt_info_upd(cfg, /*act=ADD*/ 0);
  }
  bool teardown_twt(const devourer::TwtConfig &cfg) {
    return _fw.twt_info_upd(cfg, /*act=DEL*/ 1);
  }
  /* Bind / unbind a STA macid to a TWT config (add/del/terminate/suspend/
   * resume — see TwtStaAct::action). */
  bool twt_bind_sta(const devourer::TwtStaAct &act) { return _fw.twt_act(act); }
  bool twt_announce(uint8_t macid) { return _fw.twt_announce(macid); }

  /* TWT-OFDMA autonomous trigger cadence (fw func 0x03 — may be absent from the
   * shipped fw; the caller falls back to configure_ul_ofdma). */
  bool configure_twt_ofdma(const devourer::TwtOfdmaConfig &cfg) {
    return _fw.twt_ofdma_info_upd(cfg);
  }

  /* Program the production UL-OFDMA scheduler table (mode=tf_periodic makes the
   * fw air Triggers autonomously). */
  bool configure_ul_ofdma(const devourer::UlOfdmaConfig &cfg) {
    return _fw.ul_fixinfo(cfg);
  }

  /* ---- HE sounding (mac_init_snd_mer/_mee + mac_set_snd_para) ----
   * The HE sounding command surface: arm the BB/MAC sounding engine, register
   * the beamformee, then drive a sounding intended to air NDPA -> NDP -> BFRP
   * and receive the report HE TB PPDU. Band 0 only. NB the shipped client NIC
   * firmware accepts the SET_SND_PARA H2C but does not air the sequence (the fw
   * sounding-transmit engine is AP-firmware-only); see docs/he-trigger-ul.md. */

  /* mac_init_snd_mer + mac_init_snd_mee: arm the beamformer CSI-parse offsets +
   * the beamformee response/CSI options so the AP can send NDPA/BFRP and parse
   * the returned compressed-beamforming report. Idempotent register writes;
   * call once in InitWrite (after phy_bb_rf_init). */
  bool init_sounding() {
    namespace k = kestrel::reg;
    /* mer: BFMER_CTRL_0 = NDP_BFEN | HT/VHT/HE CSI payload offsets. */
    _device.rtw_write32(
        k::R_AX_BFMER_CTRL_0,
        k::B_AX_BFMER_NDP_BFEN |
            (static_cast<uint32_t>(k::HT_PAYLOAD_OFFSET)
             << k::B_AX_BFMER_HT_CSI_OFFSET_SH) |
            (static_cast<uint32_t>(k::VHT_PAYLOAD_OFFSET)
             << k::B_AX_BFMER_VHT_CSI_OFFSET_SH) |
            (static_cast<uint32_t>(k::HE_PAYLOAD_OFFSET)
             << k::B_AX_BFMER_HE_CSI_OFFSET_SH));
    /* mee: CSI rate set. */
    _device.rtw_write32(k::R_AX_TRXPTCL_RESP_CSI_RRSC, k::CSI_RRSC_BMAP);
    /* 8852C: unmask the RMAC CSI error indication (else the fw drops the
     * incoming report as an error). */
    if (_variant == ChipVariant::C8852C)
      clr32(k::R_AX_TRXPTCL_ERROR_INDICA_MASK, k::B_AX_RMAC_CSI);
    /* RESP_OPTION: NDPA response enable + zero the NDP/BFRP report-wait standby
     * timers (the _patch_snd_ple_modify / _patch_snd_mu_err patches). Zeroing
     * both is load-bearing: otherwise the fw holds a hardware slot per sounding
     * waiting for a report that never returns, and the sounding pipeline backs
     * up after ~7 pending soundings, wedging the CH12 H2C queue (rc=-7). */
    field32(k::R_AX_BFMEE_RESP_OPTION, k::NDP_RX_STANDBY_TIMER,
            k::B_AX_BFMEE_NDP_RX_STANDBY_TIMER_MSK,
            k::B_AX_BFMEE_NDP_RX_STANDBY_TIMER_SH);
    field32(k::R_AX_BFMEE_RESP_OPTION, k::BFRP_RX_STANDBY_TIMER,
            k::B_AX_BFMEE_BFRP_RX_STANDBY_TIMER_MSK,
            k::B_AX_BFMEE_BFRP_RX_STANDBY_TIMER_SH);
    set32(k::R_AX_BFMEE_RESP_OPTION, k::B_AX_BFMEE_HT_NDPA_EN |
                                         k::B_AX_BFMEE_VHT_NDPA_EN |
                                         k::B_AX_BFMEE_HE_NDPA_EN);
    /* CSI_CTRL_0: use the per-STA CCTL bf params (BFPARAM_SEL) + NSTS/GID/force. */
    set32(k::R_AX_TRXPTCL_RESP_CSI_CTRL_0,
          k::B_AX_BFMEE_BFPARAM_SEL | k::B_AX_BFMEE_USE_NSTS |
              k::B_AX_BFMEE_CSI_GID_SEL | k::B_AX_BFMEE_CSI_FORCE_RETE_EN);
    /* CSIRPT_OPTION: match the report by AID (VHT-SU + HE-SU) + clear the
     * empty-report append-zero (the _patch_csi_append_zero patch). */
    set32(k::R_AX_CSIRPT_OPTION,
          k::B_AX_CSIPRT_VHTSU_AID_EN | k::B_AX_CSIPRT_HESU_AID_EN);
    clr32(k::R_AX_CSIRPT_OPTION, k::B_AX_CSIRPT_EMPTY_APPZERO);
    _logger->info("Kestrel: init_sounding ({}) done",
                  _variant == ChipVariant::C8852C ? "8852C" : "8852B");
    return true;
  }

  /* Register `macid` as a beamformee to sound: program the per-STA CSI/bf CCTL
   * fields (nc/nr/ng/cb/cs + csi_para_en) and map the sounding-status + CSI
   * buffer index registers to the macid. `snd_idx`/`csi_buf` are the hw
   * resource slots (0 for the first beamformee). */
  bool register_beamformee(uint8_t macid, uint8_t addr_cam_idx,
                           const devourer::StaBfCaps &bf, uint8_t snd_idx = 0,
                           uint16_t csi_buf = 0) {
    namespace k = kestrel::reg;
    bool ok = _fw.fw_upd_cctl_bf(macid, addr_cam_idx, bf);
    /* snd-status index -> macid (R_AX_BFMER_ASSOCIATED_SU0 + SND_SH*idx). */
    _device.rtw_write32(
        static_cast<uint16_t>(k::R_AX_BFMER_ASSOCIATED_SU0 +
                              k::SND_STS_SH * snd_idx),
        k::B_AX_MER_SU_BFMEE0_EN | macid);
    /* CSI buffer index -> macid (R_AX_BFMER_CSI_BUFF_IDX0 + CSI_SH*buf). */
    _device.rtw_write32(
        static_cast<uint16_t>(k::R_AX_BFMER_CSI_BUFF_IDX0 +
                              k::CSI_BUFF_SH * csi_buf),
        ((static_cast<uint32_t>(csi_buf) & k::B_AX_MER_CSI_BUFF_IDX_MSK)
         << k::B_AX_MER_CSI_BUFF_IDX_SH) |
            (macid & k::B_AX_MER_CSI_BUFF_MACID_MSK));
    _logger->info("Kestrel: register_beamformee (macid={} a_idx={} snd_idx={} "
                  "csi_buf={}) -> {}",
                  macid, addr_cam_idx, snd_idx, csi_buf, ok ? "ok" : "FAILED");
    return ok;
  }

  /* Drive one HE sounding (mac_set_snd_para): the fw airs NDPA -> NDP -> BFRP
   * and receives the beamformee's report HE TB PPDU. */
  bool start_sounding(const devourer::SoundingConfig &cfg) {
    return _fw.set_snd_para(cfg);
  }

  /* Firmware SER probe: returns the latched mac_ax_err_info code if the fw has
   * posted a halt error (HALT_C2H_CTRL set), else 0. Logs a labeled line so the
   * bring-up can pinpoint which init step crashes the running fw. */
  uint32_t fw_err_state(const char *where);

  /* Poll HALT_C2H for `ms` with no other bus traffic — detects an async fw
   * self-crash after boot (vs a write-triggered one). */
  void fw_err_settle(const char *where, uint32_t ms);

  /* Chip cut version, read fresh from R_AX_SYS_CFG1[15:12]. */
  uint8_t read_cut();

  /* Multi-secure-section key index (__mss_index): reads physical efuse
   * 0x5EC/0x5ED and matches the OTP key tables. Selects which appended
   * signature the secure firmware image uses. 0 for a stock (uncustomized)
   * chip. Used by the FWDL security-section handling. */
  uint8_t read_mss_index();

  /* Security-record bit (OTP 0x5ED[7], _security_rec). 0 => secure IC. */
  uint8_t read_sec_rec();

  ChipVariant variant() const { return _variant; }

private:
  /* Read-modify-write helpers on 32/16/8-bit registers. */
  void set32(uint16_t reg, uint32_t bits);
  void clr32(uint16_t reg, uint32_t bits);
  void field32(uint16_t reg, uint32_t val, uint32_t msk, uint8_t sh);
  /* Poll reg until (read & mask) == expect, PWR_POLL cadence. false = timeout. */
  bool poll32(uint16_t reg, uint32_t mask, uint32_t expect);
  /* XTAL_SI indirect write: reg[bitmask] <- val (masked). false = timeout. */
  bool write_xtal_si(uint8_t offset, uint8_t val, uint8_t bitmask);
  /* XTAL_SI indirect read: returns the byte at `offset` (0 on timeout). */
  uint8_t read_xtal_si(uint8_t offset);
  /* mac_pwr_on_usb_8852c (pwr_seq_func_8852c.c) — the C8852C power-on sequence
   * (distinct LDO/SPS/CMAC1/XTAL deltas vs 8852B). power_on() dispatches here
   * for the C8852C variant. */
  bool power_on_8852c();

  bool usb_pre_init();
  void usb_init(); /* runtime USB init (endpoint NUMP/burst) — usb_init_8852b */
  /* BB/RF channel helpers (all over the wIndex=1 window). */
  void bb_rmw(uint32_t addr, uint32_t mask, uint32_t val); /* masked BB write */
  uint32_t bb_read(uint32_t addr, uint32_t mask); /* masked+shifted BB read */
  uint32_t rf_read(uint8_t path, uint8_t rf_addr);         /* DDV RF read */
  void rf_write(uint8_t path, uint8_t rf_addr, uint32_t val); /* DDV RF write */
  /* DAV (a-die) RF write via the BB 0x370 serial command (full 20-bit only,
   * which is all the channel path needs). */
  void rf_write_dav(uint8_t path, uint8_t rf_addr, uint32_t val);
  /* DAV (a-die) RF read via the BB 0x378 serial command + 0x174c readback. */
  uint32_t rf_read_dav(uint8_t path, uint8_t rf_addr);
  /* halbb_write/read_rf_reg_8852b: dispatch on BIT(16) of addr (DAV=a-die
   * serial, DDV=d-die window) with a masked RMW; addr is the full RF address
   * incl. the 0x10000 DDV flag. rf_rrf returns the masked+shifted field. */
  void rf_wrf(uint8_t path, uint32_t addr, uint32_t mask, uint32_t val);
  uint32_t rf_rrf(uint8_t path, uint32_t addr, uint32_t mask);
  uint8_t _cur_band_type = 1; /* band of the last set_channel (for FastRetune) */

public:
  /* RF calibration at bring-up: the vendored halrf DACK + RX-DCK (both chips;
   * MSBK/DADCK/biasK included). On the 8852B the RF 0x0/0x1/0x5 registers are
   * saved/restored around the cal so the operational radio state survives
   * (the vendor's later TRX/channel flow rewrites them; devourer's does not).
   * Removes the ADC/DAC DC terms the CCA energy detector reads as busy. */
  void dac_cal();

  void bb_reset_all();
  /* DMAC sub-inits (trxcfg.c). */
  bool dle_init_nic();
  bool hfc_init_nic(); /* runtime page/credit quotas so CH12 IO-offload flows */
  bool sta_sch_init();
  void mpdu_proc_init();
  void sec_eng_init();
  bool chk_dle_rdy(uint16_t status_reg, uint32_t rdy_bits, const char *what);
  /* CMAC sub-inits (trxcfg.c / rx_filter.c) — the full band-0 cmac_init. */
  void scheduler_init();
  bool addr_cam_init();
  void rx_fltr_init();
  void nav_ctrl_init();
  void spatial_reuse_init();
  void tmac_init();
  void trxptcl_init();
  void rmac_init();
  void cca_ctrl_init();
  void cmac_com_init();
  void ptcl_init();
  void cmac_dma_init();
  void usb_rx_agg_cfg();
  void coex_mac_init();                    /* LTE/BT coex bring-up */
  bool write_lte(uint32_t offset, uint32_t val); /* LTE-coex indirect write */
  void mac_enable_imr(); /* DMAC+CMAC0 error-interrupt masks (SER) + err-IMR */
  /* Physical efuse read loop (read_hw_efuse, DDV bank) into `phys`. */
  bool read_phys_efuse(uint8_t *phys, uint32_t size);
  void enable_efuse_pwr_cut();
  void disable_efuse_pwr_cut();

  RtlAdapter _device;
  Logger_t _logger;
  ChipVariant _variant;
  KestrelFw _fw; /* persistent: owns the CH12 H2C transport + IO-offload */
  int16_t _txpwr_dbm_q2 = 20 * 4; /* fixed BB TX power, s(9,2); 20 dBm default */
  int16_t _txpwr_offset_qdb = 0;  /* runtime offset (quarter-dB), sticky */
  bool _cca_on = false; /* DEVOURER_KESTREL_CCA_ON: keep CCA gates on (test) */

  /* Vendored halbb-G6 8852C RX bring-up (hal/halbb/g6/kestrel_halbb_glue).
   * The static callbacks route the vendor C's register/OS plane to this device
   * (dev cookie = the HalKestrel*). */
  /* Build the bridge + halbb/halrf ctxs once (idempotent). */
  void ensure_vnd_ctx(uint8_t cut, uint8_t rfe_type);
  void vnd_bb_bringup(uint8_t cut, uint8_t rfe_type);
  void vnd_bb_set_gain(uint8_t channel, uint8_t band_type);
  /* Full vendor per-channel BB config (halbb_ctrl_bw_ch -> per-chip
   * backend). bw = devourer ChannelWidth_t. */
  void vnd_bb_ctrl_bw_ch(uint8_t pri_ch, uint8_t center, ChannelWidth_t bw,
                             uint8_t band_type);
  /* Vendored T-MAC TX path-com routing (halbb_ctrl_tx_path_tmac_8852c):
   * connects the BB IFFT output to the TX chain on the 8852C (glue no-op on
   * the 8852B, which selects the TX antenna per-STA via the CMAC model). */
  void vnd_bb_ctrl_tx_path();
  ::kestrel_halbb_ctx *_halbb_ctx = nullptr;
  void *_halbb_bridge = nullptr; /* heap kestrel_halbb_bridge, outlives ctx */
  static unsigned int halbb_r32(void *dev, unsigned int addr);
  static void halbb_w32(void *dev, unsigned int addr, unsigned int val);
  static unsigned int halbb_rpwr(void *dev, unsigned int addr);
  static void halbb_wpwr(void *dev, unsigned int addr, unsigned int val);
  static void halbb_delay(void *dev, unsigned int us);
  /* fwcmd H2C plane (bridge send_h2c): forwards ONLY the OUTSRC radio-page
   * classes into KestrelFw's encoder; other halrf H2Cs stay inert. */
  static int halbb_send_h2c(void *dev, unsigned char h2c_class,
                            unsigned char h2c_func, const unsigned int *data,
                            unsigned short len_bytes);
  /* XTAL-SI plane (bridge read_xsi/write_xsi) -> the xtal_si helpers. */
  static unsigned char halbb_read_xsi(void *dev, unsigned char offset);
  static void halbb_write_xsi(void *dev, unsigned char offset,
                              unsigned char val);
  /* RF-register plane callbacks (shared bridge) -> 3-wire rf_rrf/rf_wrf. */
  static unsigned int halbb_rrf(void *dev, unsigned int path, unsigned int addr,
                                unsigned int mask);
  static void halbb_wrf(void *dev, unsigned int path, unsigned int addr,
                        unsigned int mask, unsigned int val);
  /* Logical-efuse field read callback -> vendored halrf_get_efuse_info_8852c
   * against the parsed efuse shadow. Feeds the halrf TSSI-DE + thermal reads. */
  static int halbb_efuse_get_info(void *dev, unsigned int id, void *value,
                                  unsigned int size);
  std::array<uint8_t, 2048> _efuse_log_map{}; /* parsed logical efuse shadow */
  bool _efuse_valid = false;                  /* autoload OK + map populated */
  /* Vendored halrf-G6 8852C RF calibrations (hal/halrf/g6). Share the
   * halbb bridge (its RF-reg callbacks). */
  void vnd_rf_dac_cal();
  void vnd_rf_rx_dck();
  /* Set the halrf channel + run IQK (per-channel). band 0=2.4G,1=5G. */
  void vnd_rf_iqk(uint8_t center, uint8_t band, ChannelWidth_t bw);
  /* Vendored RF channel/band/bw tune + synth relock (halrf_ctl_band_ch_bw_8852c
   * + halrf_lck_8852c) — the correct RF tune for all bands incl. 6 GHz. */
  void vnd_rf_tune(uint8_t band_type, uint8_t center,
                                 ChannelWidth_t bw);
  struct kestrel_halrf_ctx *_halrf_ctx = nullptr;
  bool _halrf_rfk_inited = false; /* NCTL engine loaded (one-time, lazy) */

public:
  ~HalKestrel();
  void set_cca_on(bool on) { _cca_on = on; }
};

} /* namespace kestrel */

#endif /* KESTREL_HAL_KESTREL_H */
