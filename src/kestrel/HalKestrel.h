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
 * (pwr_seq_func_8852b.c, hw.c, efuse.c, _usb_8852b.c). Milestone M1a. The
 * firmware-download layer (M1b) and TRX init (M2) build on top of this. */
class HalKestrel {
public:
  HalKestrel(RtlAdapter device, Logger_t logger, ChipVariant variant);

  /* USB power-on: usb_pre_init + mac_pwr_on_usb_8852b. Leaves the MAC powered
   * (DMAC/CMAC func-enabled) and ready for firmware download. Returns false on
   * a poll timeout (chip wedged — needs a VBUS cold). 8852C shares the 8852B
   * recipe for M1; a variant divergence hook lands if bring-up proves it. */
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

  /* Firmware download (M1b): hci_func_en + DLE/HFC pre-init + FWDL of the
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

  /* M2a — the DMAC half of mac_trx_init: re-init the DLE to the NIC-mode (SCC)
   * quota, station scheduler, MPDU processor, security engine. Must run after
   * download_firmware. Returns false on the sta-scheduler poll timeout. The
   * CMAC half (rx_fltr / rmac / cmac_dma) + BB/RF/channel that complete RX
   * land in later steps. */
  bool trx_dmac_init();

  /* M2a — the CMAC half (RX path): promiscuous RX filter, receiver MAC
   * (RCR channel-enable + DLK timeouts + RX max-len), CCA control, TX
   * subcarrier / RRSR, RX DMA full-mode clear, and USB RX aggregation. Must
   * run after trx_dmac_init. The TX/protocol CMAC sub-inits (scheduler EDCA,
   * NAV, spatial-reuse, tmac, trxptcl, ptcl, addr-cam) are not needed for
   * monitor RX and are omitted. Actual reception also requires the BB/RF/
   * channel bring-up (M3). */
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
   * completions). Found via a usbmon golden diff vs the in-kernel rtw89 driver
   * (which writes PORT_CFG_P0; devourer never did). Call at the end of the TX
   * bring-up, after cmac_init. */
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

  /* M3 — PHY bring-up: apply the halbb BB register + gain tables and the
   * halrf radio-A/B tables (via PhyTableLoaderKestrel). `rfe_type` / `cut`
   * select the table variant (from the efuse / chip id). Must run after the
   * MAC TRX init. This programs the baseband + RF registers; channel tuning
   * and the RX loop build on it. */
  bool phy_bb_rf_init(uint8_t rfe_type, uint8_t cut);

  /* M3 — tune the BB + RF to a monitor channel (2.4/5 GHz, 20 MHz). Ports the
   * halbb ctrl_ch/ctrl_bw/cck_en/bb_reset + the halrf RF18 channel setting.
   * Must run after phy_bb_rf_init. */
  /* `offset` = SelectedChannel.ChannelOffset (HAL_PRIME_CHNL_OFFSET: 1=primary
   * lower / secondary above, 2=primary upper / secondary below); ignored for
   * 20 MHz. For 40 MHz the RF tunes to the block center = primary +/- 2. */
  bool set_channel(uint8_t channel, ChannelWidth_t bw, uint8_t offset = 0);

  /* Lean intra-band, same-bandwidth retune (frequency hopping): RF channel
   * switch + band-specific gain/RPL re-apply + BB reset, skipping the BB
   * bandwidth config and the per-channel RX-DCK (the slow ~1.2 ms part). Caller
   * must ensure same band + same width (20 MHz). */
  void fast_retune(uint8_t channel);

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

  /* M4/M5 — enable the per-user TX report so the fw emits C2H USR_TX_RPT_INFO
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

  /* M6 pseudo-STA — register a station-role MACID with the firmware
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
  /* M3 BB/RF channel helpers (all over the wIndex=1 window). */
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
  /* Full halrf channel setting (halrf_ctrl_ch_8852b): DAV+DDV x path A/B with
   * the path-A synth lock (halrf_set_s0_arfc18: 0xd3[8]/poll 0xb7[8]). */
  void rf_ctrl_ch(uint8_t channel, bool is_2g);
  /* halrf_bw_setting_8852b: set the RF18 bandwidth bits [11:10] (DAV+DDV x
   * path A/B): 40 MHz=BIT11, 80 MHz=BIT10, 20/5/10=both. Called after
   * rf_ctrl_ch on a bandwidth change. */
  void rf_ctrl_bw(ChannelWidth_t bw);
  /* halrf_rx_dck_8852b (RFC path): per-path RX DC-offset calibration. Corrects
   * the RX DC term the CCA/EDCCA energy detector otherwise reads as a perpetual
   * medium-busy. Run after the channel is tuned. */
  void rx_dck();

public:
  /* halrf_dac_cal_8852b (ADC/ADDCK subset): DRCK + ADC DC-offset calibration —
   * removes the ADC DC term the CCA energy detector reads as busy. Run once at
   * init (after the BB/RF tables). RF 0x0/0x1/0x5 are saved/restored so the
   * operational radio state survives the cal. */
  void dac_cal();

private:
  void afe_init();
  void dack_reset();
  void drck();
  void addck();
  void addck_reload();
  void dack(); /* DAC-side MSBK + DADCK auto-cal (halrf_dack_8852b s0/s1) */
  uint16_t _addck_d[2][2] = {}; /* [path][ic/qc] ADC DC-offset backup */

public:
  void bb_reset_all();
  /* M2a DMAC sub-inits (trxcfg.c). */
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
  /* halbb RX gain cache (bb_gain_i, halbb_cfg_bb_gain_8852b): per-band per-path
   * LNA/TIA gain-error, parsed once from array_mp_8852b_phy_reg_gain and applied
   * to the band-specific BB registers on every channel set (halbb_set_gain_
   * error_8852b). Without it the 5 GHz RX front-end runs at wrong gain and hears
   * nothing (2.4 GHz tolerates the HW defaults; 5 GHz does not). Bands: 0=2.4G,
   * 1=5G-L(36-64), 2=5G-M(100-144), 3=5G-H(149-177); 2 paths; 7 LNA + 2 TIA. */
  static constexpr int kGainBandNum = 4;
  static constexpr int kGainPathNum = 2;
  int8_t _lna_gain[kGainBandNum][kGainPathNum][7] = {};
  int8_t _tia_gain[kGainBandNum][kGainPathNum][2] = {};
  /* RPL (received-power-level) offset cache (halbb_cfg_bb_rpl_ofst, cfg_type 1)
   * — per-band/path/RXSC-subchannel offsets used in the BB RX RSSI/RPL
   * computation, applied to BB 0x49b0/b4/b8 per channel by set_rxsc_rpl_comp.
   * Corrects the reported RSSI accuracy. 20 MHz = 1 value; 40 MHz = 9 RXSC
   * (0 + 1..8); 80 MHz = 13 RXSC. */
  int8_t _rpl_ofst_20[kGainBandNum][kGainPathNum] = {};
  int8_t _rpl_ofst_40[kGainBandNum][kGainPathNum][9] = {};
  int8_t _rpl_ofst_80[kGainBandNum][kGainPathNum][13] = {};
  bool _gain_cached = false;
  static uint8_t gain_band_determine(uint8_t channel);
  void init_gain_table(uint32_t rfe_type, uint32_t cut); /* populate the cache */
  void set_gain_error(uint8_t channel); /* apply the cache to BB regs per band */
  void cfg_rpl_ofst(uint8_t band, uint8_t path, uint8_t bw, uint8_t rxsc_start,
                    uint32_t data);           /* decode a cfg_type-1 RPL entry */
  void set_rxsc_rpl_comp(uint8_t channel); /* apply RPL offsets to BB per band */
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

public:
  void set_cca_on(bool on) { _cca_on = on; }
};

} /* namespace kestrel */

#endif /* KESTREL_HAL_KESTREL_H */
