/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MT76x0U channel set and calibration: a port of mt76/mt76x0/phy.c
 * (mt76x0_phy_set_channel and what it calls, mt76x0_phy_calibrate, the
 * calibration work) and mt76x0/main.c (mt76x0_set_channel), GPL-2.0-only;
 * (c) 2002-2010 Ralink Technology, Inc.; Copyright (C) 2014 Felix Fietkau,
 * (C) 2015 Jakub Kicinski, (C) 2018 Stanislaw Gruszka, Lorenzo Bianconi. Plus
 * the mt76x02 helpers it calls (mt76x02_phy.c, mt76x02_mac.c, mt76x02_dfs.c;
 * BSD-3-Clause-Clear). reference/mt76 @ be5ce79.
 *
 * 20 and 40 MHz. 80 MHz is the same function with four channel groups and is
 * deliberately refused until it is ported and tested.
 *
 * Left out, all on the transmit side (this backend never sends):
 * mt76x0_phy_set_txpower, the TSSI DC calibration in the power-on calibrate,
 * the TSSI calibration in the periodic work, and mt76x02_edcca_tx_enable.
 */
#include <stdlib.h>
#include "internal.h"
#include "phy.h"
#include "initvals.h"

#define ARRAY_SIZE(a) ((int)(sizeof(a) / sizeof((a)[0])))

/* --- pure: what a channel request means ------------------------------------ */

static int in_plan(uint8_t ch)
{
	for (int i = 0; i < ARRAY_SIZE(mt76x0_frequency_plan); i++)
		if (mt76x0_frequency_plan[i].channel == ch)
			return 1;
	return 0;
}

/* A 20 MHz primary this backend tunes: the mt76 band tables intersected with
 * the frequency plan (which stops at 173, so 177 is out). */
static int valid_primary(uint8_t ch)
{
	if (ch >= 1 && ch <= 14)
		return 1;
	if ((ch >= 36 && ch <= 64) || (ch >= 100 && ch <= 144))
		return ch % 4 == 0;
	if (ch >= 149 && ch <= 173)
		return (ch - 149) % 4 == 0;
	return 0;
}

/* The lower 20 MHz of each 40 MHz pair in 5 GHz. */
static int lower_of_5g_pair(uint8_t ch)
{
	static const uint8_t lower[] = { 36, 44, 52, 60, 100, 108, 116, 124,
	                                 132, 140, 149, 157, 165 };
	for (int i = 0; i < ARRAY_SIZE(lower); i++)
		if (lower[i] == ch)
			return 1;
	return 0;
}

/* mt76x0_phy_set_channel's ext_cca_chan[] (groups 0 and 1 are the 20/40 MHz
 * ones). */
static uint32_t ext_cca_for(int group)
{
	static const uint32_t tab[4] = {
		FIELD_PREP(MT_EXT_CCA_CFG_CCA0, 0) | FIELD_PREP(MT_EXT_CCA_CFG_CCA1, 1) |
		FIELD_PREP(MT_EXT_CCA_CFG_CCA2, 2) | FIELD_PREP(MT_EXT_CCA_CFG_CCA3, 3) |
		FIELD_PREP(MT_EXT_CCA_CFG_CCA_MASK, BIT(0)),
		FIELD_PREP(MT_EXT_CCA_CFG_CCA0, 1) | FIELD_PREP(MT_EXT_CCA_CFG_CCA1, 0) |
		FIELD_PREP(MT_EXT_CCA_CFG_CCA2, 2) | FIELD_PREP(MT_EXT_CCA_CFG_CCA3, 3) |
		FIELD_PREP(MT_EXT_CCA_CFG_CCA_MASK, BIT(1)),
		FIELD_PREP(MT_EXT_CCA_CFG_CCA0, 2) | FIELD_PREP(MT_EXT_CCA_CFG_CCA1, 3) |
		FIELD_PREP(MT_EXT_CCA_CFG_CCA2, 1) | FIELD_PREP(MT_EXT_CCA_CFG_CCA3, 0) |
		FIELD_PREP(MT_EXT_CCA_CFG_CCA_MASK, BIT(2)),
		FIELD_PREP(MT_EXT_CCA_CFG_CCA0, 3) | FIELD_PREP(MT_EXT_CCA_CFG_CCA1, 2) |
		FIELD_PREP(MT_EXT_CCA_CFG_CCA2, 1) | FIELD_PREP(MT_EXT_CCA_CFG_CCA3, 0) |
		FIELD_PREP(MT_EXT_CCA_CFG_CCA_MASK, BIT(3)),
	};
	return tab[group & 3];
}

int mt0_chan_shape_for(uint8_t primary, int ht40, struct mt0_chan_shape *s,
                       const char **err)
{
	if (!valid_primary(primary)) {
		if (err) *err = "channel not supported by this adapter";
		return -1;
	}
	s->primary = primary;
	s->is_5ghz = primary > 14;
	s->rf_bw_band = s->is_5ghz ? RF_A_BAND : RF_G_BAND;
	/* Radar channels in every regulatory domain mt76 serves (52-64,
	 * 100-144). The kernel takes this from the channel's RADAR flag. */
	s->radar = (primary >= 52 && primary <= 64) || (primary >= 100 && primary <= 144);

	if (ht40 == 0) {
		s->width = 20;
		s->ch_group = 0;
		s->center = primary;
		s->rf_bw_band |= RF_BW_20;
	} else {
		int ok;
		if (primary == 14) {
			if (err) *err = "channel 14 is 20 MHz only";
			return -1;
		}
		if (s->is_5ghz)
			ok = ht40 > 0 ? lower_of_5g_pair(primary)
			              : lower_of_5g_pair((uint8_t)(primary - 4));
		else
			ok = ht40 > 0 ? primary + 4 <= 13 : primary - 4 >= 1;
		if (!ok) {
			if (err) *err = ht40 > 0 ? "no 40 MHz channel with the secondary above"
			                         : "no 40 MHz channel with the secondary below";
			return -1;
		}
		/* mt76x0_phy_set_channel, NL80211_CHAN_WIDTH_40: center_freq1 >
		 * freq is group 0 (HT40+); the RF goes to the 40 MHz center. */
		s->width = 40;
		s->ch_group = ht40 > 0 ? 0 : 1;
		s->center = (uint8_t)(primary + 2 - s->ch_group * 4);
		s->rf_bw_band |= RF_BW_40;
	}
	if (!in_plan(s->center)) {
		if (err) *err = "channel not in the RF frequency plan";
		return -1;
	}
	s->ext_cca = ext_cca_for(s->ch_group);
	return 0;
}

uint32_t mt0_monitor_filter_value(uint32_t rxfilter, int accept_control)
{
	/* mt76x0_config(IEEE80211_CONF_CHANGE_MONITOR): PROMISC off;
	 * mt76x02_configure_filter: OTHER_BSS always off, control-frame drops
	 * off when the monitor asks for control frames (FIF_CONTROL /
	 * FIF_PSPOLL). CRC and PHY errors stay dropped (no fcsfail/plcpfail). */
	uint32_t v = rxfilter & ~(MT_RX_FILTR_CFG_PROMISC | MT_RX_FILTR_CFG_OTHER_BSS);
	if (accept_control)
		v &= ~(MT_RX_FILTR_CFG_ACK | MT_RX_FILTR_CFG_CTS |
		       MT_RX_FILTR_CFG_CFEND | MT_RX_FILTR_CFG_CFACK |
		       MT_RX_FILTR_CFG_BA | MT_RX_FILTR_CFG_CTRL_RSV |
		       MT_RX_FILTR_CFG_PSPOLL);
	return v;
}

/* --- RF helpers ------------------------------------------------------------ */

static int rf_rmw(struct mt76x0u_dev *d, uint32_t o, uint8_t mask, uint8_t val)
{
	const int cur = mt0_rf_rr(d, o);
	if (cur < 0)
		return -1;
	val |= (uint8_t)(cur & ~mask);
	return mt0_rf_wr(d, o, val);
}
static int rf_set(struct mt76x0u_dev *d, uint32_t o, uint8_t v) { return rf_rmw(d, o, 0, v); }
static int rf_clear(struct mt76x0u_dev *d, uint32_t o, uint8_t m) { return rf_rmw(d, o, m, 0); }

/* --- the channel set ------------------------------------------------------- */

/* mt76x02_phy_set_bw */
static void phy_set_bw(struct mt76x0u_dev *d, int width, uint8_t ctrl)
{
	const int core_val = width == 40 ? 2 : 0;
	const int agc_val = width == 40 ? 3 : 1;

	mt0_rmw(d, MT_BBP(CORE, 1), MT_BBP_CORE_R1_BW, FIELD_PREP(MT_BBP_CORE_R1_BW, core_val));
	mt0_rmw(d, MT_BBP(AGC, 0), MT_BBP_AGC_R0_BW, FIELD_PREP(MT_BBP_AGC_R0_BW, agc_val));
	mt0_rmw(d, MT_BBP(AGC, 0), MT_BBP_AGC_R0_CTRL_CHAN, FIELD_PREP(MT_BBP_AGC_R0_CTRL_CHAN, ctrl));
	mt0_rmw(d, MT_BBP(TXBE, 0), MT_BBP_TXBE_R0_CTRL_CHAN,
	        FIELD_PREP(MT_BBP_TXBE_R0_CTRL_CHAN, ctrl));
}

/* mt76x02_phy_set_band */
static void phy_set_band_tx(struct mt76x0u_dev *d, int is_5ghz, int primary_upper)
{
	if (is_5ghz) {
		mt0_clear(d, MT_TX_BAND_CFG, MT_TX_BAND_CFG_2G);
		mt0_set(d, MT_TX_BAND_CFG, MT_TX_BAND_CFG_5G);
	} else {
		mt0_set(d, MT_TX_BAND_CFG, MT_TX_BAND_CFG_2G);
		mt0_clear(d, MT_TX_BAND_CFG, MT_TX_BAND_CFG_5G);
	}
	mt0_rmw(d, MT_TX_BAND_CFG, MT_TX_BAND_CFG_UPPER_40M,
	        primary_upper ? MT_TX_BAND_CFG_UPPER_40M : 0);
}

/* mt76x0_phy_set_band */
static int phy_set_band(struct mt76x0u_dev *d, int is_5ghz)
{
	if (!is_5ghz) {
		if (mt0_mcu_wr_rp(d, MT_MCU_MEMMAP_RF, mt76x0_rf_2g_channel_0_tab,
		                  ARRAY_SIZE(mt76x0_rf_2g_channel_0_tab)) ||
		    mt0_rf_wr(d, MT_RF(5, 0), 0x45) || mt0_rf_wr(d, MT_RF(6, 0), 0x44))
			return -1;
		mt0_wr(d, MT_TX_ALC_VGA3, 0x00050007);
		mt0_wr(d, MT_TX0_RF_GAIN_CORR, 0x003E0002);
	} else {
		if (mt0_mcu_wr_rp(d, MT_MCU_MEMMAP_RF, mt76x0_rf_5g_channel_0_tab,
		                  ARRAY_SIZE(mt76x0_rf_5g_channel_0_tab)) ||
		    mt0_rf_wr(d, MT_RF(5, 0), 0x44) || mt0_rf_wr(d, MT_RF(6, 0), 0x45))
			return -1;
		mt0_wr(d, MT_TX_ALC_VGA3, 0x00000005);
		mt0_wr(d, MT_TX0_RF_GAIN_CORR, 0x01010102);
	}
	return 0;
}

/* mt76x0_phy_set_chan_rf_params. `channel` is the RF (center) channel. */
static int set_chan_rf_params(struct mt76x0u_dev *d, uint8_t channel, uint16_t rf_bw_band)
{
	uint16_t rf_band = rf_bw_band & 0xff00;
	const uint16_t rf_bw = rf_bw_band & 0x00ff;
	int b_sdm = 0, rc = 0;

	for (int i = 0; i < ARRAY_SIZE(mt76x0_sdm_channel); i++)
		if (channel == mt76x0_sdm_channel[i]) {
			b_sdm = 1;
			break;
		}

	for (int i = 0; i < ARRAY_SIZE(mt76x0_frequency_plan); i++) {
		if (channel != mt76x0_frequency_plan[i].channel)
			continue;
		rf_band = (uint16_t)mt76x0_frequency_plan[i].band;
		const struct mt0_freq_item *f =
			b_sdm ? &mt76x0_sdm_frequency_plan[i] : &mt76x0_frequency_plan[i];

		rc |= mt0_rf_wr(d, MT_RF(0, 37), f->pllR37);
		rc |= mt0_rf_wr(d, MT_RF(0, 36), f->pllR36);
		rc |= mt0_rf_wr(d, MT_RF(0, 35), f->pllR35);
		rc |= mt0_rf_wr(d, MT_RF(0, 34), f->pllR34);
		rc |= mt0_rf_wr(d, MT_RF(0, 33), f->pllR33);
		rc |= rf_rmw(d, MT_RF(0, 32), 0xe0, f->pllR32_b7b5);
		/* R32<4:0> pll_den: (Denomina - 8) */
		rc |= rf_rmw(d, MT_RF(0, 32), MT_RF_PLL_DEN_MASK, f->pllR32_b4b0);
		/* R31<7:5> */
		rc |= rf_rmw(d, MT_RF(0, 31), 0xe0, f->pllR31_b7b5);
		/* R31<4:0> pll_k (Nominator) */
		rc |= rf_rmw(d, MT_RF(0, 31), MT_RF_PLL_K_MASK, f->pllR31_b4b0);
		/* R30<7> sdm_reset_n */
		if (b_sdm) {
			rc |= rf_clear(d, MT_RF(0, 30), MT_RF_SDM_RESET_MASK);
			rc |= rf_set(d, MT_RF(0, 30), MT_RF_SDM_RESET_MASK);
		} else {
			rc |= rf_rmw(d, MT_RF(0, 30), MT_RF_SDM_RESET_MASK, f->pllR30_b7);
		}
		/* R30<6:2> sdmmash_prbs,sin */
		rc |= rf_rmw(d, MT_RF(0, 30), MT_RF_SDM_MASH_PRBS_MASK, f->pllR30_b6b2);
		/* R30<1> sdm_bp */
		rc |= rf_rmw(d, MT_RF(0, 30), MT_RF_SDM_BP_MASK, (uint8_t)(f->pllR30_b1 << 1));
		/* R30<0> R29<7:0> pll_n */
		rc |= mt0_rf_wr(d, MT_RF(0, 29), (uint8_t)(f->pll_n & 0xff));
		rc |= rf_rmw(d, MT_RF(0, 30), 0x1, (uint8_t)((f->pll_n >> 8) & 0x1));
		/* R28<7:6> isi_iso, <5:4> pfd_dly, <3:2> clksel */
		rc |= rf_rmw(d, MT_RF(0, 28), MT_RF_ISI_ISO_MASK, f->pllR28_b7b6);
		rc |= rf_rmw(d, MT_RF(0, 28), MT_RF_PFD_DLY_MASK, f->pllR28_b5b4);
		rc |= rf_rmw(d, MT_RF(0, 28), MT_RF_CLK_SEL_MASK, f->pllR28_b3b2);
		/* R28<1:0> R27<7:0> R26<7:0> sdm_k */
		rc |= mt0_rf_wr(d, MT_RF(0, 26), (uint8_t)(f->pll_sdm_k & 0xff));
		rc |= mt0_rf_wr(d, MT_RF(0, 27), (uint8_t)((f->pll_sdm_k >> 8) & 0xff));
		rc |= rf_rmw(d, MT_RF(0, 28), 0x3, (uint8_t)((f->pll_sdm_k >> 16) & 0x3));
		/* R24<1:0> xo_div */
		rc |= rf_rmw(d, MT_RF(0, 24), MT_RF_XO_DIV_MASK, f->pllR24_b1b0);
		break;
	}
	if (rc)
		return -1;

	for (int i = 0; i < ARRAY_SIZE(mt76x0_rf_bw_switch_tab); i++) {
		const struct mt0_rf_switch_item *it = &mt76x0_rf_bw_switch_tab[i];
		if (rf_bw == it->bw_band ||
		    (rf_bw == (it->bw_band & 0xff) && (rf_band & it->bw_band)))
			if (mt0_rf_wr(d, it->rf_bank_reg, it->value))
				return -1;
	}
	for (int i = 0; i < ARRAY_SIZE(mt76x0_rf_band_switch_tab); i++)
		if (mt76x0_rf_band_switch_tab[i].bw_band & rf_band)
			if (mt0_rf_wr(d, mt76x0_rf_band_switch_tab[i].rf_bank_reg,
			              mt76x0_rf_band_switch_tab[i].value))
				return -1;

	mt0_clear(d, MT_RF_MISC, 0xc);

	/* mt76x02_ext_pa_enabled: the PA is external when PA_INT is CLEAR. */
	const uint16_t conf0 = mt0_ee_get(d->eeprom, MT_EE_NIC_CONF_0);
	const int ext_pa = (rf_band & RF_G_BAND) ? !(conf0 & MT_EE_NIC_CONF_0_PA_INT_2G)
	                                         : !(conf0 & MT_EE_NIC_CONF_0_PA_INT_5G);
	if (ext_pa) {
		/* MT_RF_MISC [2]: external A band PA, [3]: external G band PA */
		mt0_set(d, MT_RF_MISC, (rf_band & RF_A_BAND) ? BIT(2) : BIT(3));
		for (int i = 0; i < ARRAY_SIZE(mt76x0_rf_ext_pa_tab); i++)
			if (mt76x0_rf_ext_pa_tab[i].bw_band & rf_band)
				if (mt0_rf_wr(d, mt76x0_rf_ext_pa_tab[i].rf_bank_reg,
				              mt76x0_rf_ext_pa_tab[i].value))
					return -1;
	}

	uint32_t mac_reg;
	if (rf_band & RF_G_BAND) {
		mt0_wr(d, MT_TX0_RF_GAIN_ATTEN, 0x63707400);
		/* Atten mode = 2 for G band, disable TX inc DCOC. */
		if (mt0_rr_chk(d, MT_TX_ALC_CFG_1, &mac_reg))
			return -1;
		mt0_wr(d, MT_TX_ALC_CFG_1, mac_reg & 0x896400FF);
	} else {
		mt0_wr(d, MT_TX0_RF_GAIN_ATTEN, 0x686A7800);
		/* Atten mode = 0; for ext A band, disable TX inc DCOC cal. */
		if (mt0_rr_chk(d, MT_TX_ALC_CFG_1, &mac_reg))
			return -1;
		mt0_wr(d, MT_TX_ALC_CFG_1, mac_reg & 0x890400FF);
	}
	return 0;
}

/* mt76x0_phy_set_chan_bbp_params: the AGC8 gain carries the per-channel LNA
 * correction (the wifit3 5 GHz fix). */
static void set_chan_bbp_params(struct mt76x0u_dev *d, uint16_t rf_bw_band)
{
	for (int i = 0; i < ARRAY_SIZE(mt76x0_bbp_switch_tab); i++) {
		const struct mt0_bbp_switch_item *it = &mt76x0_bbp_switch_tab[i];

		if ((rf_bw_band & it->bw_band) != rf_bw_band)
			continue;
		if (it->reg_pair.reg == MT_BBP(AGC, 8)) {
			uint32_t val = it->reg_pair.value;
			uint8_t gain = (uint8_t)FIELD_GET(MT_BBP_AGC_GAIN, val);

			gain = (uint8_t)(gain - d->lna_gain * 2);
			val &= ~MT_BBP_AGC_GAIN;
			val |= FIELD_PREP(MT_BBP_AGC_GAIN, gain);
			mt0_wr(d, it->reg_pair.reg, val);
		} else {
			mt0_wr(d, it->reg_pair.reg, it->reg_pair.value);
		}
	}
}

/* mt76x02_init_agc_gain */
static void init_agc_gain(struct mt76x0u_dev *d)
{
	d->agc_gain_init[0] = (uint8_t)FIELD_GET(MT_BBP_AGC_GAIN, mt0_rr(d, MT_BBP(AGC, 8)));
	d->agc_gain_init[1] = (uint8_t)FIELD_GET(MT_BBP_AGC_GAIN, mt0_rr(d, MT_BBP(AGC, 9)));
	d->agc_gain_cur[0] = d->agc_gain_init[0];
	d->agc_gain_cur[1] = d->agc_gain_init[1];
	d->low_gain = -1;
}

int mt0_phy_calibrate(struct mt76x0u_dev *d, int power_on)
{
	if (d->is_mt7630)
		return 0;
	if (!d->shape_valid) {
		ERR("calibrate before any channel set");
		return -1;
	}
	const struct mt0_chan_shape *s = &d->shape;
	uint32_t tx_alc, reg_val, val;
	int rc = 0;

	if (power_on) {
		rc |= mt0_mcu_calibrate(d, MCU_CAL_R, 0);
		rc |= mt0_mcu_calibrate(d, MCU_CAL_VCO, s->primary);
		mt0_usleep(15);
		/* mt76x0_phy_tssi_dc_calibrate (when the EFUSE enables TSSI) is a
		 * transmit-power calibration and toggles MAC TX on; not run. */
	}

	if (mt0_rr_chk(d, MT_TX_ALC_CFG_0, &tx_alc))
		return -1;
	mt0_wr(d, MT_TX_ALC_CFG_0, 0);
	mt0_usleep(600);

	if (mt0_rr_chk(d, MT_BBP(IBI, 9), &reg_val))
		return -1;
	mt0_wr(d, MT_BBP(IBI, 9), 0xffffff7e);

	if (s->is_5ghz)
		val = s->primary < 100 ? 0x701 : s->primary < 140 ? 0x801 : 0x901;
	else
		val = 0x600;

	rc |= mt0_mcu_calibrate(d, MCU_CAL_FULL, val);
	rc |= mt0_mcu_calibrate(d, MCU_CAL_LC, (uint32_t)s->is_5ghz);
	mt0_usleep(17500);

	mt0_wr(d, MT_BBP(IBI, 9), reg_val);
	mt0_wr(d, MT_TX_ALC_CFG_0, tx_alc);
	rc |= mt0_mcu_calibrate(d, MCU_CAL_RXDCOC, 1);
	return rc ? -1 : 0;
}

/* mt76x02_mac_cc_reset: arm the channel-time counters and clear them. */
static void mac_cc_reset(struct mt76x0u_dev *d)
{
	mt0_wr(d, MT_CH_TIME_CFG, MT_CH_TIME_CFG_TIMER_EN | MT_CH_TIME_CFG_TX_AS_BUSY |
	                          MT_CH_TIME_CFG_RX_AS_BUSY | MT_CH_TIME_CFG_NAV_AS_BUSY |
	                          MT_CH_TIME_CFG_EIFS_AS_BUSY | MT_CH_CCA_RC_EN |
	                          FIELD_PREP(MT_CH_TIME_CFG_CH_TIMER_CLR, 1));
	(void)mt0_rr(d, MT_CH_BUSY);
	(void)mt0_rr(d, MT_CH_IDLE);
}

/* mt76x02_edcca_init with ed_monitor off (mac80211's default outside ETSI
 * DFS operation), minus mt76x02_edcca_tx_enable. */
static void edcca_init(struct mt76x0u_dev *d)
{
	mt0_set(d, MT_TX_LINK_CFG, MT_TX_CFACK_EN);
	mt0_clear(d, MT_TXOP_CTRL_CFG, MT_TXOP_ED_CCA_EN);
	mt0_wr(d, MT_BBP(AGC, 2), 0x003a6464);
	mt0_clear(d, MT_TXOP_HLDR_ET, MT_TXOP_HLDR_TX40M_BLK_EN);
}

int mt0_set_channel(struct mt76x0u_dev *d, uint8_t primary, int ht40)
{
	struct mt0_chan_shape s;
	const char *err = NULL;

	if (mt0_chan_shape_for(primary, ht40, &s, &err)) {
		ERR("channel %u%s refused: %s", primary,
		    ht40 > 0 ? " HT40+" : ht40 < 0 ? " HT40-" : "", err);
		return -1;
	}
	if (!d->mcu_running || !d->eeprom_loaded) {
		ERR("channel set before bring-up");
		return -1;
	}
	if (s.is_5ghz ? !d->has_5ghz : !d->has_2ghz) {
		ERR("channel %u: band not available on this card", primary);
		return -1;
	}

	mt0_io_clear(d);

	/* mt76x0_phy_bbp_set_bw (USB): the firmware's BW_SETTING. */
	if (mt0_mcu_function_select(d, BW_SETTING, s.width == 40 ? 1 : 0))
		return -1;
	phy_set_bw(d, s.width, s.ch_group);
	phy_set_band_tx(d, s.is_5ghz, s.ch_group & 1);
	mt0_rmw(d, MT_EXT_CCA_CFG,
	        MT_EXT_CCA_CFG_CCA0 | MT_EXT_CCA_CFG_CCA1 | MT_EXT_CCA_CFG_CCA2 |
	        MT_EXT_CCA_CFG_CCA3 | MT_EXT_CCA_CFG_CCA_MASK, s.ext_cca);

	if (phy_set_band(d, s.is_5ghz) || set_chan_rf_params(d, s.center, s.rf_bw_band))
		return -1;

	/* Japan TX filter at channel 14 (the RF channel, as the kernel). */
	if (s.center == 14)
		mt0_set(d, MT_BBP(CORE, 1), 0x20);
	else
		mt0_clear(d, MT_BBP(CORE, 1), 0x20);

	/* mt76x0_read_rx_gain: for the primary channel (chan->hw_value). */
	d->lna_gain = mt0_ee_lna_gain(d->eeprom, s.primary);
	mt0_ee_rssi_offset(d->eeprom, s.primary, d->rssi_offset);
	/* Published as one word for the RX path (rx.cpp). */
	d->rx_corr.store(((uint32_t)(uint8_t)d->lna_gain << 8) |
	                 (uint32_t)(uint8_t)d->rssi_offset[0],
	                 std::memory_order_relaxed);
	set_chan_bbp_params(d, s.rf_bw_band);

	/* enable vco */
	if (rf_set(d, MT_RF(0, 4), BIT(7)))
		return -1;

	d->shape = s;
	d->shape_valid = 1;
	init_agc_gain(d);
	if (mt0_phy_calibrate(d, 0))
		return -1;

	/* mt76x0_set_channel's wrapper */
	mac_cc_reset(d);
	edcca_init(d);

	if (mt0_io_errors(d)) {
		ERR("channel set: %u register transfer(s) failed", mt0_io_errors(d));
		return -1;
	}
	LOG("channel %u %s (RF %u%s), LNA gain %d", s.primary,
	    s.width == 40 ? (s.ch_group ? "HT40-" : "HT40+") : "20 MHz", s.center,
	    s.radar ? ", radar" : "", d->lna_gain);
	return 0;
}

/* --- the periodic calibration ---------------------------------------------- */

/* mt76x0_phy_set_gain_val */
static void set_gain_val(struct mt76x0u_dev *d)
{
	const uint8_t gain = (uint8_t)(d->agc_gain_cur[0] - d->agc_gain_adjust);

	mt0_rmw(d, MT_BBP(AGC, 8), MT_BBP_AGC_GAIN, FIELD_PREP(MT_BBP_AGC_GAIN, gain));

	if (d->shape.radar && !d->is_mt7630) {
		/* mt76x02_phy_dfs_adjust_agc, non-mt76x2 arm */
		const uint32_t agc_r8 = mt0_rr(d, MT_BBP(AGC, 8));
		const uint32_t agc_r4 = mt0_rr(d, MT_BBP(AGC, 4));
		const uint32_t val_r8 = (agc_r8 & 0x00007e00) >> 9;
		uint32_t val_r4 = agc_r4 & ~0x1f000000u;

		val_r4 += ((val_r8 + 1) >> 1) << 24;
		mt0_wr(d, MT_BBP(AGC, 4), val_r4);
		uint32_t dfs_r31 = FIELD_GET(MT_BBP_AGC_LNA_HIGH_GAIN, val_r4);
		dfs_r31 += val_r8;
		dfs_r31 -= (agc_r8 & 0x00000038) >> 3;
		dfs_r31 = (dfs_r31 << 16) | 0x00000307;
		mt0_wr(d, MT_BBP(DFS, 31), dfs_r31);
		/* disable, then enable, the hardware radar detector */
		mt0_wr(d, MT_BBP(DFS, 0), 0);
		mt0_wr(d, MT_BBP(DFS, 0), MT_DFS_CH_EN << 16);
	}
}

/* mt76x02_phy_adjust_vga_gain */
static int adjust_vga_gain(struct mt76x0u_dev *d)
{
	const uint8_t limit = d->low_gain > 0 ? 16 : 4;
	int ret = 0;

	d->false_cca = FIELD_GET(MT_RX_STAT_1_CCA_ERRORS, mt0_rr(d, MT_RX_STAT_1));
	if (d->false_cca > 800 && d->agc_gain_adjust < limit) {
		d->agc_gain_adjust += 2;
		ret = 1;
	} else if ((d->false_cca < 10 && d->agc_gain_adjust > 0) ||
	           (d->agc_gain_adjust >= limit && d->false_cca < 500)) {
		d->agc_gain_adjust -= 2;
		ret = 1;
	}
	return ret;
}

/* mt76x0_phy_update_channel_gain. The kernel averages the RSSI of ASSOCIATED
 * stations and falls back to -75 dBm without any; a monitor has none, so it
 * is always -75 here, exactly what the kernel does for a monitor vif. */
static void update_channel_gain(struct mt76x0u_dev *d)
{
	const int avg_rssi = -75;
	const int w40 = d->shape.width == 40;
	const int thresh = w40 ? -65 : -68;     /* mt76x02_get_rssi_gain_thresh */
	const int low_thresh = w40 ? -79 : -82; /* ..._low_rssi_gain_thresh */
	const int low_gain = (avg_rssi > thresh) + (avg_rssi > low_thresh);
	const int gain_change = d->low_gain < 0 || ((d->low_gain & 2) ^ (low_gain & 2));

	d->low_gain = low_gain;
	if (!gain_change) {
		if (adjust_vga_gain(d))
			set_gain_val(d);
		return;
	}
	d->agc_gain_adjust = low_gain == 2 ? 0 : 10;
	const uint8_t gain_delta = low_gain == 2 ? 10 : 0;
	d->agc_gain_cur[0] = (uint8_t)(d->agc_gain_init[0] - gain_delta);
	set_gain_val(d);
	/* clear false CCA counters */
	(void)mt0_rr(d, MT_RX_STAT_1);
}

/* mt76x0_phy_temp_sensor */
static void temp_sensor(struct mt76x0u_dev *d)
{
	const int rf_b7_73 = mt0_rf_rr(d, MT_RF(7, 73));
	const int rf_b0_66 = mt0_rf_rr(d, MT_RF(0, 66));
	const int rf_b0_67 = mt0_rf_rr(d, MT_RF(0, 67));

	if (rf_b7_73 < 0 || rf_b0_66 < 0 || rf_b0_67 < 0)
		return;
	mt0_rf_wr(d, MT_RF(7, 73), 0x02);
	mt0_rf_wr(d, MT_RF(0, 66), 0x23);
	mt0_rf_wr(d, MT_RF(0, 67), 0x01);

	mt0_wr(d, MT_BBP(CORE, 34), 0x00080055);
	if (!mt0_poll(d, MT_BBP(CORE, 34), BIT(4), 0, 200000)) {
		mt0_clear(d, MT_BBP(CORE, 34), BIT(4));
	} else {
		int8_t val = (int8_t)mt0_rr(d, MT_BBP(CORE, 35));
		val = (int8_t)((35 * (val - d->temp_offset)) / 10 + 25);

		if (abs(val - d->temp_vco) > 20) {
			mt0_mcu_calibrate(d, MCU_CAL_VCO, d->shape.primary);
			d->temp_vco = val;
		}
		if (abs(val - d->temp) > 30) {
			mt0_phy_calibrate(d, 0);
			d->temp = val;
		}
	}
	mt0_rf_wr(d, MT_RF(7, 73), (uint8_t)rf_b7_73);
	mt0_rf_wr(d, MT_RF(0, 66), (uint8_t)rf_b0_66);
	mt0_rf_wr(d, MT_RF(0, 67), (uint8_t)rf_b0_67);
}

int mt0_cal_tick(struct mt76x0u_dev *d)
{
	if (!d->shape_valid)
		return -1;
	update_channel_gain(d);
	/* mt76x0_tssi_enabled: NIC_CONF_1 TX_ALC_EN. The kernel then runs the
	 * TSSI (transmit power) calibration INSTEAD of the temperature check,
	 * so on such a card it does no temperature recalibration either. */
	if (mt0_ee_get(d->eeprom, MT_EE_NIC_CONF_1) & BIT(13)) {
		if (!d->tssi_noted) {
			LOG("card uses TSSI: the periodic work is TX-power calibration, "
			    "not run by this receive-only backend");
			d->tssi_noted = 1;
		}
	} else {
		temp_sensor(d);
	}
	return 0;
}

void mt0_set_monitor_filter(struct mt76x0u_dev *d, int accept_control)
{
	const uint32_t v = mt0_monitor_filter_value(d->rxfilter, accept_control);
	const uint32_t mac_hi = (uint32_t)d->macaddr[4] | ((uint32_t)d->macaddr[5] << 8);

	mt0_wr(d, MT_RX_FILTR_CFG, v);
	/* wifit3, measured on an MT7610U: with the kernel's mac_setaddr values
	 * (U2ME_MASK = 0xff, MBSS mode/local/beacon bits) the address match
	 * still drops unicast DATA addressed elsewhere - EAPOL included - even
	 * with PROMISC cleared. The bare MAC in both registers lets it through,
	 * as mt76x2u's monitor path does. */
	mt0_wr(d, MT_MAC_ADDR_DW1, mac_hi);
	mt0_wr(d, MT_MAC_BSSID_DW1, mac_hi);
	LOG("monitor RX filter 0x%08x (from 0x%08x), address match bare", v, d->rxfilter);
}

/* --- public ---------------------------------------------------------------- */

/*
 * Twice, by default. wifit3 captured the kernel on the wire and found
 * mac80211 runs mt76x0_phy_set_channel twice per `iw set channel` (once from
 * .config(CONF_CHANGE_CHANNEL), once from the chandef update): 2x BW_SETTING,
 * 2x the RF writes, 6x calibration commands in every window. On hardware a
 * single pass "leaves the chip's MCU mid-state and the next channel switch's
 * first command wedges". The transfer-log comparison (src/mt76x0u/README.md) shows the
 * first pass here matching wifit3's write for write; the second is what the
 * kernel sends. mt76x0u_set_tune_twice(dev, 0) restores a single pass.
 */
int mt76x0u_set_channel(struct mt76x0u_dev *d, uint8_t primary, int ht40)
{
	if (!d)
		return -1;
	if (mt0_set_channel(d, primary, ht40))
		return -1;
	return d->tune_once ? 0 : mt0_set_channel(d, primary, ht40);
}

void mt76x0u_set_tune_twice(struct mt76x0u_dev *d, int twice)
{
	if (d)
		d->tune_once = !twice;
}

int mt76x0u_cal_tick(struct mt76x0u_dev *d)
{
	return d ? mt0_cal_tick(d) : -1;
}

int mt76x0u_set_monitor_filter(struct mt76x0u_dev *d, int accept_control)
{
	if (!d || !d->eeprom_loaded)
		return -1;
	mt0_set_monitor_filter(d, accept_control);
	return 0;
}
