/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MT76x0U PHY init: a port of mt76/mt76x0/phy.c (GPL-2.0-only; (c) 2002-2010
 * Ralink Technology, Inc.; Copyright (C) 2018 Stanislaw Gruszka, Lorenzo
 * Bianconi) and mt76x02_phy.c's rxpath/txdac (BSD-3-Clause-Clear),
 * reference/mt76 @ be5ce79.
 *
 * Phase 3 covers mt76x0_phy_init(): antenna select, RF init, RX path, TX DAC.
 * The channel set and the periodic calibration are phase 4.
 *
 * On USB the RF block is only reachable through the MCU (CMD_RANDOM_*,
 * MT_MCU_MEMMAP_RF), so everything here needs the firmware up and Q_SELECT
 * sent - mt0_rf_wr/rr refuse otherwise.
 */
#include "internal.h"
#include "phy.h"
#include "initvals.h"

#define ARRAY_SIZE(a) ((int)(sizeof(a) / sizeof((a)[0])))

/* mt76x0_rf_rmw/set/clear. -1 on failure. */
static int rf_rmw(struct mt76x0u_dev *d, uint32_t offset, uint8_t mask, uint8_t val)
{
	const int cur = mt0_rf_rr(d, offset);

	if (cur < 0)
		return -1;
	val |= (uint8_t)(cur & ~mask);
	return mt0_rf_wr(d, offset, val) ? -1 : val;
}
static int rf_set(struct mt76x0u_dev *d, uint32_t o, uint8_t v) { return rf_rmw(d, o, 0, v); }
static int rf_clear(struct mt76x0u_dev *d, uint32_t o, uint8_t m) { return rf_rmw(d, o, m, 0); }

int mt0_phy_wait_bbp_ready(struct mt76x0u_dev *d)
{
	/* mt76x0_phy_wait_bbp_ready: up to 20 reads for a value that is
	 * neither 0 nor all ones. */
	for (int i = 0; i < 20; i++) {
		uint32_t v;
		if (!mt0_rr_chk(d, MT_BBP(CORE, 0), &v) && v && ~v) {
			LOG("BBP version %08x", v);
			return 0;
		}
	}
	ERR("BBP is not ready");
	return -1;
}

/* mt76x0_phy_ant_select. All five writes, as the kernel - wifit3 omits the
 * MT_CMB_CTRL / MT_CSR_EE_CFG1 / MT_COEXCFG0 ones (its MT76X0U.md "residual
 * gaps"), which also loses the is_mt7630 antenna bits. */
static void phy_ant_select(struct mt76x0u_dev *d)
{
	uint16_t ee_ant = mt0_ee_get(d->eeprom, MT_EE_ANTENNA);
	const uint16_t ee_cfg1 = mt0_ee_get(d->eeprom, MT_EE_CFG1_INIT);
	const uint16_t nic_conf2 = mt0_ee_get(d->eeprom, MT_EE_NIC_CONF_2);
	uint32_t wlan = mt0_rr(d, MT_WLAN_FUN_CTRL);
	uint32_t coex3 = mt0_rr(d, MT_COEXCFG3);

	ee_ant &= (uint16_t)~(BIT(14) | BIT(12));
	wlan &= ~(BIT(6) | BIT(5));
	coex3 &= ~GENMASK(5, 2);

	if (ee_ant & MT_EE_ANTENNA_DUAL) {
		/* dual antenna mode */
		const int ant_div = !(nic_conf2 & MT_EE_NIC_CONF_2_ANT_OPT) &&
		                    (nic_conf2 & MT_EE_NIC_CONF_2_ANT_DIV);
		if (ant_div)
			ee_ant |= BIT(12);
		else
			coex3 |= BIT(4);
		coex3 |= BIT(3);
		if (d->has_2ghz)
			wlan |= BIT(6);
	} else {
		/* single antenna mode */
		if (d->has_5ghz) {
			coex3 |= BIT(3) | BIT(4);
		} else {
			wlan |= BIT(6);
			coex3 |= BIT(1);
		}
	}
	if (d->is_mt7630)
		ee_ant |= BIT(14) | BIT(11);

	mt0_wr(d, MT_WLAN_FUN_CTRL, wlan);
	mt0_rmw(d, MT_CMB_CTRL, GENMASK(15, 0), ee_ant);
	mt0_rmw(d, MT_CSR_EE_CFG1, GENMASK(15, 0), ee_cfg1);
	mt0_clear(d, MT_COEXCFG0, BIT(2));
	mt0_wr(d, MT_COEXCFG3, coex3);
}

/* mt76x0_rf_patch_reg_array, USB arms only: is_mt7610e() is MMIO-only, so it
 * is false here, and RF(0,3) takes the non-MMIO 0x73. */
int mt0_rf_patch_value(uint32_t reg, uint8_t val, int is_mt7630)
{
	switch (reg) {
	case MT_RF(0, 3):
		return 0x73;
	case MT_RF(0, 21):
		return 0x12;
	case MT_RF(5, 2):
		return is_mt7630 ? 0x1d : 0x0c;
	default:
		return val;
	}
}

static int rf_patch_reg_array(struct mt76x0u_dev *d, const struct mt0_reg_pair *rp,
                              int len)
{
	for (int i = 0; i < len; i++)
		if (mt0_rf_wr(d, rp[i].reg,
		              (uint8_t)mt0_rf_patch_value(rp[i].reg, (uint8_t)rp[i].value,
		                                          d->is_mt7630)))
			return -1;
	return 0;
}

/* mt76x0_phy_rf_init */
static int phy_rf_init(struct mt76x0u_dev *d)
{
	if (rf_patch_reg_array(d, mt76x0_rf_central_tab, ARRAY_SIZE(mt76x0_rf_central_tab)) ||
	    rf_patch_reg_array(d, mt76x0_rf_2g_channel_0_tab,
	                       ARRAY_SIZE(mt76x0_rf_2g_channel_0_tab)) ||
	    mt0_mcu_wr_rp(d, MT_MCU_MEMMAP_RF, mt76x0_rf_5g_channel_0_tab,
	                  ARRAY_SIZE(mt76x0_rf_5g_channel_0_tab)) ||
	    mt0_mcu_wr_rp(d, MT_MCU_MEMMAP_RF, mt76x0_rf_vga_channel_0_tab,
	                  ARRAY_SIZE(mt76x0_rf_vga_channel_0_tab)))
		return -1;

	for (int i = 0; i < ARRAY_SIZE(mt76x0_rf_bw_switch_tab); i++) {
		const struct mt0_rf_switch_item *it = &mt76x0_rf_bw_switch_tab[i];

		if (it->bw_band == RF_BW_20 ||
		    ((RF_G_BAND | RF_BW_20) & it->bw_band) == (RF_G_BAND | RF_BW_20))
			if (mt0_rf_wr(d, it->rf_bank_reg, it->value))
				return -1;
	}
	for (int i = 0; i < ARRAY_SIZE(mt76x0_rf_band_switch_tab); i++)
		if (mt76x0_rf_band_switch_tab[i].bw_band & RF_G_BAND)
			if (mt0_rf_wr(d, mt76x0_rf_band_switch_tab[i].rf_bank_reg,
			              mt76x0_rf_band_switch_tab[i].value))
				return -1;

	/* Frequency calibration. E1: B0.R22<6:0>: xo_cxo<6:0>;
	 * E2: B0.R21<0>: xo_cxo<0>, B0.R22<7:0>: xo_cxo<8:1> */
	if (mt0_rf_wr(d, MT_RF(0, 22), d->freq_offset < 0xbf ? d->freq_offset : 0xbf) ||
	    mt0_rf_rr(d, MT_RF(0, 22)) < 0)
		return -1;

	/* Reset procedure DAC during power-up: set, clear, set B0.R73<7>. */
	if (rf_set(d, MT_RF(0, 73), BIT(7)) < 0 ||
	    rf_clear(d, MT_RF(0, 73), BIT(7)) < 0 ||
	    rf_set(d, MT_RF(0, 73), BIT(7)) < 0)
		return -1;

	/* vcocal_en: initiate VCO calibration (reset after completion) */
	return rf_set(d, MT_RF(0, 4), 0x80) < 0 ? -1 : 0;
}

/* mt76x02_phy_set_rxpath: chainmask 0x101 on mt76x0, so BIT(3) clears. */
static void phy_set_rxpath(struct mt76x0u_dev *d)
{
	uint32_t val;

	if (mt0_rr_chk(d, MT_BBP(AGC, 0), &val))
		return;
	val &= ~BIT(4);
	if ((d->chainmask & 0xf) == 2)
		val |= BIT(3);
	else
		val &= ~BIT(3);
	mt0_wr(d, MT_BBP(AGC, 0), val);
	(void)mt0_rr(d, MT_BBP(AGC, 0));
}

/* mt76x02_phy_set_txdac */
static void phy_set_txdac(struct mt76x0u_dev *d)
{
	if (((d->chainmask >> 8) & 0xf) == 2)
		mt0_set(d, MT_BBP(TXBE, 5), 0x3);
	else
		mt0_clear(d, MT_BBP(TXBE, 5), 0x3);
}

/* mt76x0_phy_init (minus INIT_DELAYED_WORK: the calibration tick is phase 4) */
int mt0_phy_init(struct mt76x0u_dev *d)
{
	phy_ant_select(d);
	if (phy_rf_init(d))
		return -1;
	phy_set_rxpath(d);
	phy_set_txdac(d);
	return 0;
}
