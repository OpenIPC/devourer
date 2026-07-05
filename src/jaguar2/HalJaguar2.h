#ifndef HAL_JAGUAR2_H
#define HAL_JAGUAR2_H

#include <cstdint>
#include <memory>

#include "logger.h"
#include "RtlUsbAdapter.h"
#include "RxSense.h"
#include "ChipVariant.h"
#include "Jaguar2PhyTables.h"

namespace jaguar2 {

/* HalJaguar2 — RTL8822B / RTL8821C (Jaguar2) chip bring-up. Jaguar2 sibling of
 * src/jaguar3/HalJaguar3, using the HalMAC power sequence + the shared
 * check_positive PHY-table walker, with per-variant PHY tables and calibration
 * selected via ChipVariant. */
class HalJaguar2 {
public:
  HalJaguar2(RtlUsbAdapter device, Logger_t logger,
             ChipVariant variant = ChipVariant::C8822B);

  /* Card-enable power sequence (card-disable -> card-emulation -> active),
   * transcribed from halmac card_en_flow_8822b (USB/ALL entries). Runs
   * power_off() first so the MAC is reset from any prior (kernel-left) state. */
  void power_on();
  /* Card-disable power sequence (active -> card-emulation -> card-disable). */
  void power_off();

  /* Read REG_SYS_CFG1 (0x00F0) and decode cut / vendor / 2T2R. */
  void read_chip_version();

  struct ChipVersion {
    uint8_t cut = 0;     /* 0=A,1=B,2=C,... */
    uint8_t vendor = 0;  /* 0=TSMC,1=SMIC,2=UMC */
    uint8_t rf_2t2r = 0; /* 1 = 2T2R */
    bool test_chip = false;
  };
  ChipVersion chip_version() const { return _ver; }

  /* Read the EFUSE logical map (standard 88xx OneByteRead + 2-byte-header
   * decode) and return the RFE type at EEPROM_RFE_OPTION_8822B (0xCA). The
   * phydm BB/AGC/RF tables are gated on rfe_type; a wrong value leaves the RX
   * front-end unconfigured. */
  uint8_t read_efuse_rfe();

  /* Walk the physical EFUSE header chain and decode it into a logical map
   * (map[i] = 0xFF where unprogrammed). Shared by read_efuse_rfe and
   * apply_tx_power. `dump`=true logs the raw physical bytes + a few key fields. */
  void read_efuse_logical_map(uint8_t *map, uint16_t map_size, bool dump);

  /* Program the per-rate TXAGC (0x1d00 path A / 0x1d80 path B) from the EFUSE
   * power-by-rate calibration for `channel` at bandwidth `bw` (0=20/1=40/2=80) —
   * the efuse-calibrated level the kernel uses. Without it the TXAGC sits at the
   * hot BB-table default which overdrives high-order QAM (MCS5/7) into PA
   * compression. Ports phy_get_pg_txpwr_idx (base + per-BW/Nss diff) +
   * config_phydm_write_txagc_8822b. Both bands. */
  void apply_tx_power(uint8_t channel, uint8_t bw = 0, uint8_t rfe_type = 0);

  /* Apply the 8822B BB (phy_reg), AGC (agc_tab) and RF (radioa/radiob) phydm
   * tables via the shared check_positive walker, bracketed by the OFDM/CCK
   * block disable/enable (config_phydm_parameter_init_8822b PRE/POST). Mirrors
   * rtl8822b_phy.c: PRE -> init_bb_reg -> init_rf_reg -> POST. rfe_type selects
   * the conditional table blocks. */
  void apply_bb_rf_agc_tables(uint8_t rfe_type);

  /* Set RF channel + bandwidth (config_phydm_switch_channel_8822b +
   * config_phydm_switch_bandwidth_8822b): RF18 tune, band AGC/fc/CCK-filter,
   * RFE antenna pins, RX-path + IGI toggle. bw: 0=20/1=40/2=80 MHz.
   * primary_ch_idx = sub-channel index for 40/80 (the vendor primary_ch_idx;
   * from SelectedChannel.ChannelOffset). rfe_type selects the RFE-pin table
   * and the BW80 extra writes; rf_2t2r drives path-B writes. */
  void set_channel_bw(uint8_t channel, uint8_t bw, uint8_t rfe_type,
                      uint8_t primary_ch_idx = 0);

  /* Enable the MAC RX engine (CR MACRXEN + promiscuous RCR for monitor). */
  void enable_rx();

  /* config_phydm_trx_mode_8822b: RF mode table (0xc08/0xe08 = 0x3231), TX/RX
   * antenna-path HW-block enable, RX-path config, and the RF 0xef/0x33/0x3e/
   * 0x3f mode-table sync. Puts the RF paths into TRX mode — without it the RF
   * never enters RX and the chip delivers no frames. Called before the channel
   * set. 2T2R -> tx=rx=AB (0x3). */
  void config_trx_mode();

  /* halrf LC calibration (_phy_lc_calibrate_8822b): locks the RF LO tank at the
   * current channel. Without it the RF synthesizer does not lock and the
   * front-end receives nothing. Run after the channel is set. On C8821C this
   * dispatches to do_lck_8821c() (a different RF sequence). */
  void do_lck();

  /* Grant the antenna to WLAN (WiFi-only coex). 8822B is a WiFi+BT combo: on
   * power-up the PTA/antenna switch can be owned by BT, leaving the WL RX
   * front-end deaf (correct BB/RF config but no energy reaches the LNA). Ported
   * verbatim from ex_hal8822b_wifi_only_hw_config: switch to WL-side controller,
   * gnt_wl=1/gnt_bt=0, antenna mux to WL. Must run before enable_rx. */
  void coex_wlan_only(bool is_5g);

  /* phydm_rfe_8822b_init: RFE chip-top-mux init (0x64/0x4c/0x40 top mux, 0x1990
   * s0/s1 select, 0x974 in/out). The kernel runs this from odm_dm_init
   * (phydm_rfe_init) after BB/RF config; devourer skipped it, leaving 0x1990 at 0
   * (kernel = 0xc30) so the RFE antenna-mux source select was never applied. */
  void rfe_init();

  /* rtl8822b_phy_bf_init: MU-MIMO / TXBF init. Only the grouping-bitmap write
   * (0x1c94 = 0xafffafff, overriding the BB-table default 0x5fff5fff) affects a
   * non-beamformed frame, but ported whole for parity. Runs after rfe_init. */
  void bf_init();
  /* rtl8821c_phy_bf_init: 8821C MU-MIMO/TXBF MAC setup (0x14C0/0x167C/0x1680/
   * 0x042F/0x045F/0x6DF + 0x1c94). Replaces the 8822B bf_init (which only wrote
   * 0x1c94 and missed the MU/TXBF MAC registers). */
  void bf_init_8821c();

  /* Force a flat per-rate TXAGC power index (0..63) on both paths — a debug /
   * SDR-visibility knob (DEVOURER_TX_PWR). 8822B TXAGC is 4 rates packed per
   * dword at 0x1d00 (path A) / 0x1d80 (path B). */
  void set_tx_power_flat(uint8_t idx);

  /* One DIG (dynamic initial gain) iteration: read the per-window false-alarm
   * count (OFDM 0xf48 + CCK 0xa5c), reset the counters, and nudge IGI
   * (0xc50/0xe50) up when FA is high or down when FA is low, bounded
   * [0x1c, 0x3e]. Ported from phydm phydm_new_igi_by_fa (fa_th {250,500,750},
   * step {2,1,2}). Called on a ~100 ms cadence by the orchestrator's DIG thread
   * so weak signals are caught (low IGI) without drowning in false alarms. */
  void dig_step();
  uint8_t dbg_igi() { return static_cast<uint8_t>(_device.rtw_read8(0x0c50) & 0x7f); }
  uint32_t dbg_last_fa() const { return _last_fa; }
  /* Frame-free RX energy snapshot from the last dig_step window (FA/CCA/IGI).
   * dig_step already reads+resets these counters on its ~100 ms cadence, so the
   * energy poller piggybacks it rather than racing the reset. Invalid until
   * dig_step has run at least once (i.e. requires the DIG thread — default on). */
  RxEnergy last_energy() const { return _energy; }

  /* Debug: direct RF register read (direct-BB window). For RX bring-up probes. */
  uint32_t dbg_rf_read(uint8_t path, uint32_t addr) { return rf_read(path, addr); }
  /* Debug: RF register write (3-wire LSSI). For canary-patch bisect. */
  void dbg_rf_write(uint8_t path, uint32_t addr, uint32_t val) {
    rf_write(path, addr, val);
  }
  /* Masked RF register write (read-modify-write; full-register write when
   * mask == RFREGOFFSETMASK). Used by the CW single-tone path. */
  void dbg_rf_set(uint8_t path, uint32_t addr, uint32_t mask, uint32_t val) {
    rf_set(path, addr, mask, val);
  }

private:
  /* config_phydm_parameter_init_8822b: OFDM/CCK block enable via 0x808[29:28]
   * (post=0x3) / disable (pre=0x0). */
  void phydm_pre_post_setting(bool post);
  /* BB/AGC table writer: phydm delay opcodes (0xfe..0xf9) else a full-dword BB
   * register write, +1us settle (odm_config_bb_phy_8812a pattern). */
  void bb_write(uint32_t addr, uint32_t value);
  /* RF table writer: Jaguar 3-wire LSSI write (rA/rB_LSSIWrite_Jaguar 0xC90 /
   * 0xE90) — data = (addr<<20)|(val&0xfffff), masked to 28 bits. path: 0=A,1=B. */
  void rf_write(uint8_t path, uint32_t addr, uint32_t value);
  /* RF 3-wire LSSI read (rHSSIRead_Jaguar 0x8B0 -> rA/rB SI/PI readback). */
  uint32_t rf_read(uint8_t path, uint32_t addr);
  /* Masked RF write: full-mask -> direct LSSI write; else read-modify-write. */
  void rf_set(uint8_t path, uint32_t addr, uint32_t mask, uint32_t value);
  /* phydm_rfe_ifem 2.4G/5G antenna-switch pins (rfe_type 0 path). */
  void rfe_ifem(uint8_t channel);
  /* phydm_igi_toggle_8822b: toggle 0xc50/0xe50 IGI to enter RX mode. */
  void igi_toggle();

  /* --- 8821C-specific (C8821C variant) channel/RF-set/LCK, transcribed from
   * reference/8821cu phydm_hal_api8821c.c + halrf_8821c.c. The 8822B channel-set
   * reuse diverges materially for the 1T1R 8821C (BTG/WLG RF-set mux, 0xc1c AGC
   * index, RF-firmware LCK), so these replace the 8822B path when _variant is
   * C8821C. `rfe_raw` is the raw efuse RFE byte (rfe_type_expand). --- */
  void set_channel_bw_8821c(uint8_t channel, uint8_t bw, uint8_t rfe_raw,
                            uint8_t primary_ch_idx);
  /* config_phydm_switch_rf_set_8821c: RX front-end mux (0xcb8/0xa84/0xa80).
   * rf_set: 0=BTG (2.4G path B), 1=WLG (2.4G path A), 2=WLA (5G). */
  void switch_rf_set_8821c(uint8_t rf_set);
  /* ex_hal8821c_wifi_only_hw_config + switch_antenna: 8821C-specific WiFi-only
   * coex (GNT->WL + 0xcb4[29:28] SPDT antenna switch), distinct from the 8822B
   * coex_wlan_only register set. */
  void coex_wlan_only_8821c(bool is_5g);
  /* _phy_lc_calibrate_8821c: RF-firmware LCK (RF 0xcc/0xc4), no poll. */
  void do_lck_8821c();

  RtlUsbAdapter _device;
  Logger_t _logger;
  ChipVariant _variant;
  /* Per-chip phydm BB/AGC/RF table data, selected by _variant. */
  std::unique_ptr<Jaguar2PhyTables> _tables;
  ChipVersion _ver{};
  bool _aac_checked = false;
  uint32_t _last_fa = 0; /* last DIG-window false-alarm count (telemetry) */
  RxEnergy _energy{};    /* last DIG-window FA/CCA/IGI snapshot (telemetry) */
  /* Decoded logical EFUSE map, read once (a full physical walk is ~hundreds of
   * USB control transfers ≈ 0.5s). read_efuse_rfe populates it; apply_tx_power
   * reuses it — avoids a second walk that would slow every cold init. */
  uint8_t _efuse_map[0x200];
  bool _efuse_valid = false;

  /* 8821C CCK TX-filter defaults (0xa24/0xa28/0xaac) snapshotted from the BB
   * table at parameter-init POST; config_phydm_switch_channel_8821c restores
   * them for 2.4G non-ch14. Saved once when the 8821C tables are applied. */
  uint32_t _cck_a24_8821c = 0, _cck_a28_8821c = 0, _cck_aac_8821c = 0;
  bool _cck_saved_8821c = false;
};

} /* namespace jaguar2 */

#endif /* HAL_JAGUAR2_H */
