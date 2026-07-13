#ifndef KESTREL_HAL_KESTREL_H
#define KESTREL_HAL_KESTREL_H

#include <array>
#include <cstdint>

#include "ChipVariant.h"
#include "KestrelFw.h"
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

  /* M3 — PHY bring-up: apply the halbb BB register + gain tables and the
   * halrf radio-A/B tables (via PhyTableLoaderKestrel). `rfe_type` / `cut`
   * select the table variant (from the efuse / chip id). Must run after the
   * MAC TRX init. This programs the baseband + RF registers; channel tuning
   * and the RX loop build on it. */
  bool phy_bb_rf_init(uint8_t rfe_type, uint8_t cut);

  /* M3 — tune the BB + RF to a monitor channel (2.4/5 GHz, 20 MHz). Ports the
   * halbb ctrl_ch/ctrl_bw/cck_en/bb_reset + the halrf RF18 channel setting.
   * Must run after phy_bb_rf_init. */
  bool set_channel(uint8_t channel, ChannelWidth_t bw);

  /* Chip cut version, read fresh from R_AX_SYS_CFG1[15:12]. */
  uint8_t read_cut();

  /* Multi-secure-section key index (__mss_index): reads physical efuse
   * 0x5EC/0x5ED and matches the OTP key tables. Selects which appended
   * signature the secure firmware image uses. 0 for a stock (uncustomized)
   * chip. Used by the FWDL security-section handling. */
  uint8_t read_mss_index();

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

  bool usb_pre_init();
  void usb_init(); /* runtime USB init (endpoint NUMP/burst) — usb_init_8852b */
  /* M3 BB/RF channel helpers (all over the wIndex=1 window). */
  void bb_rmw(uint32_t addr, uint32_t mask, uint32_t val); /* masked BB write */
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
};

} /* namespace kestrel */

#endif /* KESTREL_HAL_KESTREL_H */
