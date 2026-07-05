#ifndef RADIO_MANAGEMENT_8822C_H
#define RADIO_MANAGEMENT_8822C_H

#include <cstdint>

#include "logger.h"
#include "RtlUsbAdapter.h"
#include "SelectedChannel.h"
#include "ChipVariant.h"

namespace jaguar3 {

/* RadioManagementJaguar3 — channel / bandwidth / TX-power for the Jaguar3 family.
 *
 * The Jaguar3 channel+bandwidth path is procedural — a port of the vendor's
 * config_phydm_switch_channel_8822c / config_phydm_switch_bandwidth_8822c
 * (hal/phydm/rtl8822c/phydm_hal_api8822c.c) — rather than the Jaguar1
 * phy_PostSetBwMode8812 register writes.
 *
 * Jaguar3 — unlike Jaguar1 — exposes a true baseband clock divider, so the
 * 5/10 MHz narrowband re-clock (the whole reason for the port) is reachable:
 * set_bandwidth_dividers applies it on top of an already-tuned channel via the
 * NB_* registers below (SDR-validated). */
class RadioManagementJaguar3 {
public:
  RadioManagementJaguar3(RtlUsbAdapter device, Logger_t logger,
                         ChipVariant variant = ChipVariant::C8822C);

  void set_channel_bwmode(uint8_t channel, uint8_t channel_offset,
                          ChannelWidth_t bwmode);

  /* Lean intra-band, same-bandwidth hop retune — the FastRetune core (see
   * docs/frequency-hopping.md). The subset of set_channel_bwmode a hop needs:
   * the RF18 channel write inside its 3-wire bracket + force-anapar, the BB
   * reset, and the channel-keyed constants (SCO fc, TX DFIR, AGC table) only
   * when their bucket changes. Everything bandwidth-keyed (clock/DFIR block,
   * RXBB, MAC BW/TXSC, narrowband dividers, DAC-FIFO reset) and band-keyed
   * (CCK-RxIQ, RF 0xdf) is untouched — set by the last full set at this
   * BW/band. In 5/10 MHz mode the TX-DFIR write is skipped entirely (the
   * narrowband re-clock owns 0x808 on the 8822e), so fast hops preserve the
   * narrowband clocking without re-running the divider recipe.
   *
   * cache_rf=true reuses the cached full RF18 value (primed by one BB-window
   * read on the first fast hop; the 8822e composes RF18 from scratch and
   * needs no cache/read at all). Returns false — chip untouched — when the
   * hop crosses the 2.4/5 GHz boundary or the radio was never tuned; the
   * caller falls back to the full set_channel_bwmode. */
  bool fast_retune(uint8_t channel, uint8_t channel_offset,
                   ChannelWidth_t bwmode, bool cache_rf);

  /* Narrowband re-clock applied on top of an already-tuned channel: switches
   * the baseband DAC/ADC clock + small-BW field for 5/10/20 MHz. */
  void set_bandwidth_dividers(ChannelWidth_t bwmode);

  /* MAC-side bandwidth + TX sub-channel (halmac cfg_bw_88xx / cfg_pri_ch_idx):
   * REG_WMAC_TRXPTCL_CTL 0x668 BIT7 (40M) and REG_DATA_SC 0x483 TXSC fields.
   * pri is the BB primary-sub-channel index (0 for 20M). */
  void set_mac_bw_txsc(ChannelWidth_t bw, uint8_t pri);

  /* Set the TX-power reference (7-bit index, both paths). With zero_diffs=true
   * (the DEVOURER_TX_PWR debug knob) the per-rate diff table is flattened so
   * every rate emits at `idx`. With zero_diffs=false (the default bring-up path)
   * the diff table applied by the BB tables is preserved, so per-rate spread is
   * kept and only the reference base is programmed. */
  void set_tx_power_ref(uint8_t idx, bool zero_diffs = true);

  /* Apply the 8822e phy_reg_pg power-by-rate table for `channel`'s band: sets
   * the OFDM/CCK reference PER PATH (ref_a -> 0x18e8/0x18a0, ref_b -> 0x41e8/
   * 0x41a0) and writes the per-rate diff table (0x3a00) so robust low rates get
   * the kernel's by-rate boost instead of a flat reference. The per-path refs
   * come from the efuse per-channel base (the kernel programs a distinct path-A/
   * path-B base, e.g. 0x4b/0x54 at ch36).
   *
   * skip_path_b_ofdm_ref: leave 0x41e8 (path-B OFDM ref) at its table default.
   * Hardware-bisected: ANY nonzero value in that one field desenses the EU's RX
   * to near-deaf (value-independent; path-A ref, CCK refs, the diff table and
   * the DPK bypass are all RX-safe) — root cause open, suspected path-B
   * TSSI/gain-stage asymmetry in devourer's bring-up vs the kernel's. TX+RX
   * callers set this so RX works at the cost of path-B OFDM TX running at the
   * table-default reference. */
  void apply_power_by_rate_8822e(uint8_t channel, uint8_t ref_a, uint8_t ref_b,
                                 bool skip_path_b_ofdm_ref = false);

private:
  /* Jaguar3 baseband bandwidth/clock registers (from
   * config_phydm_switch_bandwidth_8822c). These do NOT exist on Jaguar1. */
  static constexpr uint16_t R_RX_DFIR_8822C   = 0x810; /* [13:4] RX DFIR coeff */
  static constexpr uint16_t R_SMALL_BW_8822C  = 0x9b0; /* [7:6] small-BW field */
  static constexpr uint16_t R_CLK_DIV_8822C   = 0x9b4; /* [10:8] DAC, [22:20] ADC clk */

  /* Narrowband recipe (both variants — the 8822c codes):
   *   small-BW 0x9b0[7:6]:    5M->0x1  10M->0x2  20M->0x0
   *   DAC clk  0x9b4[10:8]:   5M->0x4  10M->0x6  20M->0x7 (120/240/480M)
   *   ADC clk  0x9b4[22:20]:  5M->0x4  10M->0x5  20M->0x6 (40/80/160M)
   *   RX DFIR  0x810[13:4]:   5/10M->0x2ab  20M->0x19b
   * The 8822e vendor phydm ships lower DAC codes (5M=0x2/10M=0x4/20M=0x6,
   * same clock comments) but on real 8812EU silicon 0x2 does not transmit —
   * SDR-bisected in set_bandwidth_dividers; the 8822c codes air on both
   * chips. On the 8822e, narrowband ALSO needs the MAC-clock reconfig
   * (REG_AFE_CTRL1 0x24[21:20] + the 0x55c/0x638 us-tick clocks) applied in
   * set_bandwidth_dividers — without it 5 MHz never reaches the air at any
   * DAC code. */
  static constexpr uint8_t NB_SMALLBW_5M  = 0x1;
  static constexpr uint8_t NB_SMALLBW_10M = 0x2;

  /* 8822e AFE write-with-readback-check + DACK soft reset (the vendor's
   * halrf_ex_dac_fifo_rst: "fix dac fifo error after TXCK setting" — required
   * after a DAC-clock change or the FIFO can misalign and emit images).
   * Duplicates Halrf8822e's private helpers; folding both into a shared
   * Jaguar3RfAccess base is the already-noted follow-up. */
  void write_check_afe_8822e(uint16_t add, uint32_t data);
  void dack_soft_rst_8822e();

  /* Shared by the full and fast channel paths so they cannot diverge. */
  static void central_and_pri(uint8_t channel, uint8_t channel_offset,
                              ChannelWidth_t bwmode, uint8_t &central,
                              uint8_t &pri);
  static uint32_t sco_for(uint8_t central);      /* BB 0xc30[11:0] value */
  static uint32_t rf18_for(uint8_t central, ChannelWidth_t bwmode,
                           uint32_t base);       /* merge into base (0 = none) */
  void select_agc_tables(uint8_t central, ChannelWidth_t bwmode);
  void apply_tx_dfir(uint8_t central);
  void bb_reset_toggle();                        /* MAC 0x0 BIT16 1->0->1 */

  RtlUsbAdapter _device;
  Logger_t _logger;
  ChipVariant _variant;
  uint8_t _last_channel = 0; /* set by set_channel_bwmode; 0 = not yet tuned */

  /* fast_retune write-on-change caches. Invalidated whenever the full
   * set_channel_bwmode runs (it rewrites all of them); refreshed by the fast
   * path's own writes, so they only ever mirror fast-path state — the J1
   * RadioManagementModule pattern. */
  uint32_t _rf18_cache = 0;   /* 8822c full RF18 (BB-window read, 20-bit) */
  bool _rf18_cached = false;
  uint32_t _last_sco = 0xffffffff;
  uint32_t _last_dfir = 0xffffffff; /* packed (0x808[22:20]<<8)|[6:4] */
  int _last_agc_key = -1;           /* band/sub-band bucket + bw20 flag */
};

} /* namespace jaguar3 */

#endif /* RADIO_MANAGEMENT_8822C_H */
