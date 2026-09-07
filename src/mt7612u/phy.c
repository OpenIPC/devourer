/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * Band / bandwidth / TX-power register setup and the channel sequence.
 * Ported from mt76/mt76x02_phy.c, mt76x2/phy.c and mt76x2/usb_phy.c.
 *
 * The RF synthesizer itself is never touched here - CMD_SWITCH_CHANNEL_OP
 * hands the channel to firmware, which owns synthesis, AGC and calibration.
 */
#include <string.h>
#include "internal.h"

#define BAND_2GHZ 0
#define BAND_5GHZ 1

static int ext_pa_enabled(struct mt7612u_dev *d, int band)
{
	uint16_t conf0 = mt_ee(d, MT_EE_NIC_CONF_0);

	return band == BAND_5GHZ ? !(conf0 & MT_EE_NIC_CONF_0_PA_INT_5G)
	                         : !(conf0 & MT_EE_NIC_CONF_0_PA_INT_2G);
}

static void phy_set_band(struct mt7612u_dev *d, int band, int primary_upper)
{
	if (band == BAND_2GHZ) {
		mt_set(d, MT_TX_BAND_CFG, MT_TX_BAND_CFG_2G);
		mt_clear(d, MT_TX_BAND_CFG, MT_TX_BAND_CFG_5G);
	} else {
		mt_clear(d, MT_TX_BAND_CFG, MT_TX_BAND_CFG_2G);
		mt_set(d, MT_TX_BAND_CFG, MT_TX_BAND_CFG_5G);
	}
	mt_rmw(d, MT_TX_BAND_CFG, MT_TX_BAND_CFG_UPPER_40M,
	       FIELD_PREP(MT_TX_BAND_CFG_UPPER_40M, (uint32_t)!!primary_upper));
}

static void phy_set_bw(struct mt7612u_dev *d, uint8_t bw, uint8_t ctrl)
{
	int core_val, agc_val;

	switch (bw) {
	case MT7612U_BW_80: core_val = 3; agc_val = 7; break;
	case MT7612U_BW_40: core_val = 2; agc_val = 3; break;
	default:            core_val = 0; agc_val = 1; break;
	}
	mt_rmw(d, MT_BBP(CORE, 1), MT_BBP_CORE_R1_BW,
	       FIELD_PREP(MT_BBP_CORE_R1_BW, (uint32_t)core_val));
	mt_rmw(d, MT_BBP(AGC, 0), MT_BBP_AGC_R0_BW,
	       FIELD_PREP(MT_BBP_AGC_R0_BW, (uint32_t)agc_val));
	mt_rmw(d, MT_BBP(AGC, 0), MT_BBP_AGC_R0_CTRL_CHAN,
	       FIELD_PREP(MT_BBP_AGC_R0_CTRL_CHAN, ctrl));
	mt_rmw(d, MT_BBP(TXBE, 0), MT_BBP_TXBE_R0_CTRL_CHAN,
	       FIELD_PREP(MT_BBP_TXBE_R0_CTRL_CHAN, ctrl));
}

static void phy_set_txpower_regs(struct mt7612u_dev *d, int band)
{
	uint32_t pa_mode[2], pa_mode_adj;

	if (band == BAND_2GHZ) {
		pa_mode[0] = 0x010055ff;
		pa_mode[1] = 0x00550055;
		mt_wr(d, MT_TX_ALC_CFG_2, 0x35160a00);
		mt_wr(d, MT_TX_ALC_CFG_3, 0x35160a06);
		if (ext_pa_enabled(d, band)) {
			mt_wr(d, MT_RF_PA_MODE_ADJ0, 0x0000ec00);
			mt_wr(d, MT_RF_PA_MODE_ADJ1, 0x0000ec00);
		} else {
			mt_wr(d, MT_RF_PA_MODE_ADJ0, 0xf4000200);
			mt_wr(d, MT_RF_PA_MODE_ADJ1, 0xfa000200);
		}
	} else {
		pa_mode[0] = 0x0000ffff;
		pa_mode[1] = 0x00ff00ff;
		if (ext_pa_enabled(d, band)) {
			mt_wr(d, MT_TX_ALC_CFG_2, 0x2f0f0400);
			mt_wr(d, MT_TX_ALC_CFG_3, 0x2f0f0476);
			pa_mode_adj = 0x04000000;
		} else {
			mt_wr(d, MT_TX_ALC_CFG_2, 0x1b0f0400);
			mt_wr(d, MT_TX_ALC_CFG_3, 0x1b0f0476);
			pa_mode_adj = 0;
		}
		mt_wr(d, MT_RF_PA_MODE_ADJ0, pa_mode_adj);
		mt_wr(d, MT_RF_PA_MODE_ADJ1, pa_mode_adj);
	}

	mt_wr(d, MT_BB_PA_MODE_CFG0, pa_mode[0]);
	mt_wr(d, MT_BB_PA_MODE_CFG1, pa_mode[1]);
	mt_wr(d, MT_RF_PA_MODE_CFG0, pa_mode[0]);
	mt_wr(d, MT_RF_PA_MODE_CFG1, pa_mode[1]);

	if (ext_pa_enabled(d, band)) {
		uint32_t val = (band == BAND_2GHZ) ? 0x3c3c023c : 0x363c023c;

		mt_wr(d, MT_TX0_RF_GAIN_CORR, val);
		mt_wr(d, MT_TX1_RF_GAIN_CORR, val);
		mt_wr(d, MT_TX_ALC_CFG_4, 0x00001818);
	} else if (band == BAND_2GHZ) {
		mt_wr(d, MT_TX0_RF_GAIN_CORR, 0x0f3c3c3c);
		mt_wr(d, MT_TX1_RF_GAIN_CORR, 0x0f3c3c3c);
		mt_wr(d, MT_TX_ALC_CFG_4, 0x00000606);
	} else {
		mt_wr(d, MT_TX0_RF_GAIN_CORR, 0x383c023c);
		mt_wr(d, MT_TX1_RF_GAIN_CORR, 0x24282e28);
		mt_wr(d, MT_TX_ALC_CFG_4, 0);
	}
}

static void configure_tx_delay(struct mt7612u_dev *d, int band, uint8_t bw)
{
	uint32_t cfg0, cfg1;

	if (ext_pa_enabled(d, band)) {
		cfg0 = bw ? 0x000b0c01 : 0x00101101;
		cfg1 = 0x00011414;
	} else {
		cfg0 = bw ? 0x000b0b01 : 0x00101001;
		cfg1 = 0x00021414;
	}
	mt_wr(d, MT_TX_SW_CFG0, cfg0);
	mt_wr(d, MT_TX_SW_CFG1, cfg1);
	mt_rmw(d, MT_XIFS_TIME_CFG, MT_XIFS_TIME_CFG_OFDM_SIFS,
	       FIELD_PREP(MT_XIFS_TIME_CFG_OFDM_SIFS, 15));
}

static void adjust_high_lna_gain(struct mt7612u_dev *d, int reg, int8_t offset)
{
	int8_t gain = (int8_t)FIELD_GET(MT_BBP_AGC_LNA_HIGH_GAIN,
	                                mt_rr(d, MT_BBP(AGC, reg)));
	gain -= offset / 2;
	mt_rmw(d, MT_BBP(AGC, reg), MT_BBP_AGC_LNA_HIGH_GAIN,
	       FIELD_PREP(MT_BBP_AGC_LNA_HIGH_GAIN, (uint32_t)gain));
}

static void adjust_agc_gain(struct mt7612u_dev *d, int reg, int8_t offset)
{
	int8_t gain = (int8_t)FIELD_GET(MT_BBP_AGC_GAIN,
	                                mt_rr(d, MT_BBP(AGC, reg)));
	gain += offset;
	mt_rmw(d, MT_BBP(AGC, reg), MT_BBP_AGC_GAIN,
	       FIELD_PREP(MT_BBP_AGC_GAIN, (uint32_t)gain));
}

static void apply_gain_adj(struct mt7612u_dev *d)
{
	adjust_high_lna_gain(d, 4, d->cal.high_gain[0]);
	adjust_high_lna_gain(d, 5, d->cal.high_gain[1]);
	adjust_agc_gain(d, 8, d->cal.high_gain[0]);
	adjust_agc_gain(d, 9, d->cal.high_gain[1]);
}

static void channel_calibrate(struct mt7612u_dev *d, int is_5ghz)
{
	if (d->cal.channel_cal_done)
		return;

	if (is_5ghz)
		mt_mcu_calibrate(d, MCU_CAL_LC, 0);

	mt_mcu_calibrate(d, MCU_CAL_TX_LOFT, (uint32_t)is_5ghz);
	mt_mcu_calibrate(d, MCU_CAL_TXIQ, (uint32_t)is_5ghz);
	mt_mcu_calibrate(d, MCU_CAL_RXIQC_FI, (uint32_t)is_5ghz);
	mt_mcu_calibrate(d, MCU_CAL_TEMP_SENSOR, 0);
	mt_mcu_calibrate(d, MCU_CAL_TX_SHAPING, 0);

	apply_gain_adj(d);

	/* mt76x02_edcca_init(), ed_monitor off: energy-detect CCA disabled,
	 * which is what an injector wants - the MAC will not withhold a frame
	 * because it sees energy on the channel. */
	mt_set(d, MT_TX_LINK_CFG, MT_TX_CFACK_EN);
	mt_clear(d, MT_TXOP_CTRL_CFG, MT_TXOP_ED_CCA_EN);
	mt_wr(d, MT_BBP(AGC, 2), 0x00007070);
	mt_set(d, MT_TXOP_HLDR_ET, MT_TXOP_HLDR_TX40M_BLK_EN);

	d->cal.channel_cal_done = 1;
}

/* mt76x2_tssi_enabled(): TX_ALC_EN set and temperature-compensated ALC off. */
int mt_tssi_enabled(struct mt7612u_dev *d)
{
	uint16_t c1 = mt_ee(d, MT_EE_NIC_CONF_1);

	return !(c1 & MT_EE_NIC_CONF_1_TEMP_TX_ALC) && (c1 & MT_EE_NIC_CONF_1_TX_ALC_EN);
}
#define tssi_enabled mt_tssi_enabled

/* ---- per-rate TX power ---- */

static uint32_t tx_power_mask(uint8_t v1, uint8_t v2, uint8_t v3, uint8_t v4)
{
	return ((uint32_t)(v1 & 0x3f)) | ((uint32_t)(v2 & 0x3f) << 8) |
	       ((uint32_t)(v3 & 0x3f) << 16) | ((uint32_t)(v4 & 0x3f) << 24);
}

static void add_rate_power_offset(struct mt_rate_power *r, int offset)
{
	for (unsigned i = 0; i < sizeof r->all; i++)
		r->all[i] = (int8_t)(r->all[i] + offset);
}

static void limit_rate_power(struct mt_rate_power *r, int limit)
{
	for (unsigned i = 0; i < sizeof r->all; i++)
		if (r->all[i] > limit)
			r->all[i] = (int8_t)limit;
}

static int get_min_rate_power(const struct mt_rate_power *r)
{
	int8_t ret = 0;

	for (unsigned i = 0; i < sizeof r->all; i++) {
		if (!r->all[i]) continue;
		ret = ret ? (r->all[i] < ret ? r->all[i] : ret) : r->all[i];
	}
	return ret;
}

/*
 * mt76x2_phy_set_txpower(). Builds the per-rate power table from the EEPROM,
 * normalises it against the chain target powers, and writes the eight
 * MT_TX_PWR_CFG_* registers plus the two ALC chain-init fields.
 * Until this ran, those registers held the initvals 0x3a3a3a3a.
 */
void mt_phy_set_txpower(struct mt7612u_dev *d, int band)
{
	struct mt_tx_power_info txp;
	struct mt_rate_power t;
	int txp_0, txp_1, delta = 0, base_power, gain;

	mt_get_power_info(d, &txp, d->chan, band);

	if (d->bw == MT7612U_BW_40)      delta = txp.delta_bw40;
	else if (d->bw == MT7612U_BW_80) delta = txp.delta_bw80;

	mt_get_rate_power(d, &t, band);
	add_rate_power_offset(&t, txp.target_power + delta);
	limit_rate_power(&t, d->txpower_conf);

	base_power = get_min_rate_power(&t);
	delta = base_power - txp.target_power;
	txp_0 = txp.chain[0].target_power + txp.chain[0].delta + delta;
	txp_1 = txp.chain[1].target_power + txp.chain[1].delta + delta;

	gain = txp_0 < txp_1 ? txp_0 : txp_1;
	if (gain < 0) {
		base_power -= gain;
		txp_0 -= gain;
		txp_1 -= gain;
	} else if (gain > 0x2f) {
		base_power -= gain - 0x2f;
		txp_0 = 0x2f;
		txp_1 = 0x2f;
	}

	add_rate_power_offset(&t, -base_power);
	d->target_power = (int8_t)txp.target_power;
	d->target_power_delta[0] = (int8_t)(txp_0 - txp.chain[0].target_power);
	d->target_power_delta[1] = (int8_t)(txp_1 - txp.chain[0].target_power);
	d->rate_power = t;

	mt_rmw(d, MT_TX_ALC_CFG_0, MT_TX_ALC_CFG_0_CH_INIT_0,
	       FIELD_PREP(MT_TX_ALC_CFG_0_CH_INIT_0, (uint32_t)txp_0));
	mt_rmw(d, MT_TX_ALC_CFG_0, MT_TX_ALC_CFG_0_CH_INIT_1,
	       FIELD_PREP(MT_TX_ALC_CFG_0_CH_INIT_1, (uint32_t)txp_1));

	mt_wr(d, MT_TX_PWR_CFG_0, tx_power_mask(t.cck[0], t.cck[2], t.ofdm[0], t.ofdm[2]));
	mt_wr(d, MT_TX_PWR_CFG_1, tx_power_mask(t.ofdm[4], t.ofdm[6], t.ht[0], t.ht[2]));
	mt_wr(d, MT_TX_PWR_CFG_2, tx_power_mask(t.ht[4], t.ht[6], t.ht[8], t.ht[10]));
	mt_wr(d, MT_TX_PWR_CFG_3, tx_power_mask(t.ht[12], t.ht[14], t.ht[0], t.ht[2]));
	mt_wr(d, MT_TX_PWR_CFG_4, tx_power_mask(t.ht[4], t.ht[6], 0, 0));
	mt_wr(d, MT_TX_PWR_CFG_7, tx_power_mask(t.ofdm[7], t.vht[0], t.ht[7], t.vht[1]));
	mt_wr(d, MT_TX_PWR_CFG_8, tx_power_mask(t.ht[14], 0, t.vht[0], t.vht[1]));
	mt_wr(d, MT_TX_PWR_CFG_9, tx_power_mask(t.ht[7], 0, t.vht[0], t.vht[1]));
}

/* mt76x02_tx_get_max_txpwr_adj(): the per-rate ceiling for this frame. */
int8_t mt_tx_get_max_txpwr_adj(struct mt7612u_dev *d,
                               const struct mt7612u_tx_rate *r)
{
	const struct mt_rate_power *t = &d->rate_power;

	switch (r->phy) {
	case MT7612U_PHY_VHT:
		if (r->mcs == 8 || r->mcs == 9)
			return t->vht[0];
		return t->ht[(((r->nss ? r->nss - 1 : 0) << 3) + r->mcs) & 0xf];
	case MT7612U_PHY_HT:
	case MT7612U_PHY_HT_GF:
		return t->ht[r->mcs & 0xf];
	case MT7612U_PHY_CCK:
		return t->cck[r->mcs & 0x3];
	default:
		return t->ofdm[r->mcs & 0x7];
	}
}

/* mt76x02_tx_get_txpwr_adj(): the 4-bit per-packet trim in txwi.ctl2. */
int8_t mt_tx_get_txpwr_adj(struct mt7612u_dev *d, int8_t txpwr, int8_t max_adj)
{
	int v = txpwr < d->txpower_conf ? txpwr : d->txpower_conf;

	v -= (d->target_power + d->target_power_delta[0]);
	if (v > max_adj) v = max_adj;

	if (!d->enable_tpc)
		return 0;
	if (v >= 0)
		return (int8_t)(v < 7 ? v : 7);
	return (int8_t)(v < -16 ? 8 : (v + 32) / 2);
}

/*
 * Control channel + width -> the three values the rest of the tune needs:
 * the *hardware* channel, which is the centre of the widened channel and not
 * the control channel (36 at 80 MHz tunes 42); `bw_index`, which the firmware
 * reads; and `ch_group_index`, which says which 20 MHz slot of the group the
 * control channel occupies and so picks both the CCA ordering and the
 * primary-channel bits in the BBP.
 *
 * mt76 does not compute any of this: mt76x2u_phy_set_channel() gets the
 * segment centre from cfg80211's chandef, which carries `center_freq1`
 * alongside the control channel. This API takes a bare channel number, so the
 * centre has to be derived from the standard groupings instead. That
 * derivation is an assumption about how the channel is meant to be grouped,
 * and getting it wrong transmits a full-width signal centred somewhere the
 * caller did not ask for - silently, because every register write still
 * succeeds. So the 80 MHz result is checked against the centres the spec
 * actually defines rather than trusted.
 *
 * Returns 0, or -1 when the control channel cannot carry the width.
 */
int mt_chan_group(uint8_t chan, uint8_t bw, uint8_t *hw_chan,
                  uint8_t *bw_index, uint8_t *ch_group)
{
	/*
	 * Legal *centre* channels per band and width. Validating the centre
	 * rather than the control channel catches three failures with one
	 * check: an off-grid control channel, one whose widened span leaves
	 * the band this driver declares, and the integer wrap that a uint8_t
	 * would otherwise hide (control channel 254 at 40 MHz computes 256,
	 * which truncates to 0 and would tune the MCU to channel index 0 with
	 * the 5 GHz register set loaded).
	 *
	 * The ceilings match caps.c's band_5g_max_mhz = 5825, i.e. channel
	 * 165. That is why the 165/169/173/177 VHT80 group (centre 171 =
	 * 5855 MHz, spanning to 5895) and the 40 MHz centres 167 and 175 are
	 * absent: mt76's own channel list has them, but tuning them would put
	 * most of the carrier outside the band this driver advertises. Widen
	 * the declared band first if they are ever wanted.
	 */
	static const uint8_t centre_40_5g[] = {
		38, 46, 54, 62, 102, 110, 118, 126, 134, 142, 151, 159,
	};
	/* 2.4 GHz 40 MHz reaches only centres 6-9 through the pairing below,
	 * i.e. control channels 4-11. 1-3 would need a secondary at or below
	 * channel 0 and 12-13 one above 13; both are what the old wrap
	 * produced silently. */
	static const uint8_t centre_40_2g[] = { 6, 7, 8, 9 };
	/* VHT80 centres. 5.35-5.47 GHz (centres 74 and 90) is not allocated. */
	static const uint8_t centre_80_5g[] = { 42, 58, 106, 122, 138, 155 };

	const uint8_t *ok = NULL;
	size_t n_ok = 0, i;
	/* int, not uint8_t: the arithmetic below can leave 0..255 before it has
	 * been validated. The grid check is what actually catches that - no
	 * wrapped value aliases onto a legal centre at either width, so a
	 * uint8_t here would still be refused - but it would be refused while
	 * reporting the wrapped number, and "would centre on 0" sends whoever
	 * reads that message looking for the wrong bug. */
	int idx = 0, group = 0, hw = chan;

	switch (bw) {
	case MT7612U_BW_20:
		goto out;               /* the control channel is the centre */
	case MT7612U_BW_40:
		/* mt76x2u_phy_set_channel()'s 40 MHz case. Which side the
		 * secondary sits on follows the standard pairing: 36/44/149/157
		 * take the upper half, 40/48/153/161 the lower. */
		if ((chan / 4) & 1) { idx = 1; group = 0; }
		else                { idx = 3; group = 1; }
		hw = chan + 2 - group * 4;
		if (chan <= 14) {
			ok = centre_40_2g;
			n_ok = sizeof centre_40_2g / sizeof centre_40_2g[0];
		} else {
			ok = centre_40_5g;
			n_ok = sizeof centre_40_5g / sizeof centre_40_5g[0];
		}
		break;
	case MT7612U_BW_80:
		/* mt76 computes this as (freq - freq1 + 30) / 20, i.e. the
		 * control channel's offset from the segment centre in 20 MHz
		 * steps. Over the 5 GHz grid that is the same value as the
		 * channel's position within its group of four, and this form
		 * needs no centre to be supplied by the caller. It stays
		 * correct across the U-NII-3 discontinuity (the group starts at
		 * 149, not 148) because the integer divide absorbs the +1 that
		 * the whole group carries. */
		group = ((chan / 4) + 3) & 3;
		idx = group;
		hw = chan + 6 - group * 4;
		/* 802.11ac defines no 80 MHz channel in 2.4 GHz. */
		if (chan <= 14) {
			ERR("80 MHz needs a 5 GHz channel (asked for %u)", chan);
			return -1;
		}
		ok = centre_80_5g;
		n_ok = sizeof centre_80_5g / sizeof centre_80_5g[0];
		break;
	default:
		ERR("unsupported bandwidth %u", bw);
		return -1;
	}

	for (i = 0; i < n_ok; i++)
		if (hw == ok[i])
			goto out;
	ERR("channel %u cannot carry %d MHz: it would centre on %d, which is "
	    "not a usable %d MHz centre channel in this band",
	    chan, 20 << bw, hw, 20 << bw);
	return -1;

out:
	if (hw_chan)  *hw_chan  = (uint8_t)hw;
	if (bw_index) *bw_index = (uint8_t)idx;
	if (ch_group) *ch_group = (uint8_t)group;
	return 0;
}

/* fast=1 skips the firmware calibration burst, which is what a retune would do
 * if the chip tolerates it. Measured cost of each path: see BRINGUP-RESULTS. */
int mt_set_channel_ex(struct mt7612u_dev *d, uint8_t chan, uint8_t bw, int fast)
{
	static const uint32_t ext_cca_chan[4] = {
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
	int band = chan > 14 ? BAND_5GHZ : BAND_2GHZ;
	uint8_t bw_index = 0, ch_group_index = 0, hw_chan = chan;

	if (mt_chan_group(chan, bw, &hw_chan, &bw_index, &ch_group_index))
		return -1;

	/* Same bracket as the bring-up: the tune is a long run of best-effort
	 * writes, and a channel reported as set while half its registers never
	 * landed is worse than a refusal. */
	mt_io_clear(d);

	d->cal.channel_cal_done = fast;
	d->chan = chan;
	d->bw = bw;
	/* The TX "never widen" notice is once per width, not once per device:
	 * a later tune to a narrower channel is a new situation and deserves
	 * its own warning. Without this, a clamp consumed by a startup-ordering
	 * quirk silences every genuine one that follows. */
	d->bw_clamp_warned = 0;

	mt_read_rx_gain(d, chan, band);
	phy_set_txpower_regs(d, band);
	configure_tx_delay(d, band, bw);
	mt_phy_set_txpower(d, band);
	phy_set_band(d, band, ch_group_index & 1);
	phy_set_bw(d, bw, ch_group_index);

	mt_rmw(d, MT_EXT_CCA_CFG,
	       MT_EXT_CCA_CFG_CCA0 | MT_EXT_CCA_CFG_CCA1 | MT_EXT_CCA_CFG_CCA2 |
	       MT_EXT_CCA_CFG_CCA3 | MT_EXT_CCA_CFG_CCA_MASK,
	       ext_cca_chan[ch_group_index]);

	/* The BBP is already at the new width by here, so a failed synth leaves
	 * the part genuinely half-tuned. Clear d->chan on the way out: it is the
	 * "never tuned" sentinel the TX width clamp and the power setter already
	 * test, and claiming a channel we did not reach is worse than claiming
	 * none. */
	if (mt_mcu_set_channel(d, hw_chan, bw, bw_index, 0)) {
		d->chan = 0;
		return -1;
	}
	if (mt_mcu_init_gain(d, hw_chan, d->cal.mcu_gain, 1)) {
		d->chan = 0;
		return -1;
	}

	/* rev >= E3: enable LDPC Rx */
	mt_set(d, MT_BBP(RXO, 13), BIT(10));

	/* The width-dependent half of mt76x2_phy_update_channel_gain()
	 * (mt76x2/phy.c:300-323). That function is upstream's periodic RX gain
	 * worker, which this port does not have; but two of its writes depend
	 * only on the channel width, not on the measured gain, so they belong
	 * with the tune. Without them an 80 MHz channel runs the RX front end
	 * on the 20/40 MHz values the firmware init left behind - the part
	 * still receives, just with the wrong AGC shape. The gain-tracking
	 * remainder (low_gain, AGC 26's low nibble, the per-width RSSI
	 * thresholds) stays unported and is noted in the README. */
	if (bw == MT7612U_BW_80) {
		mt_wr(d, MT_BBP(RXO, 14), 0x00560211);
		mt_wr(d, MT_BBP(AGC, 35), 0x10101014);
	} else {
		mt_wr(d, MT_BBP(RXO, 14), 0x00560423);
		mt_wr(d, MT_BBP(AGC, 35),
		      band == BAND_2GHZ ? 0x11111516 : 0x11111116);
	}

	if (!d->cal.init_cal_done) {
		uint8_t v = d->eeprom[MT_EE_BT_RCAL_RESULT];

		if (v != 0xff)
			mt_mcu_calibrate(d, MCU_CAL_R, 0);
	}
	mt_mcu_calibrate(d, MCU_CAL_RXDCOC, hw_chan);
	if (!d->cal.init_cal_done)
		mt_mcu_calibrate(d, MCU_CAL_RC, 0);
	d->cal.init_cal_done = 1;

	mt_wr(d, MT_BBP(AGC, 61), 0xff64a4e2);
	mt_wr(d, MT_BBP(AGC, 7), 0x08081010);
	mt_wr(d, MT_BBP(AGC, 11), 0x00000404);
	mt_wr(d, MT_BBP(AGC, 2), 0x00007070);
	mt_wr(d, MT_TXOP_CTRL_CFG, 0x04101b3f);

	mt_set(d, MT_BBP(TXO, 4), BIT(25));
	mt_set(d, MT_BBP(RXO, 13), BIT(8));

	channel_calibrate(d, band == BAND_5GHZ);

	if (fast)
		return 0;

	/* mt76x02_init_agc_gain(): host-side snapshot of the AGC gain the
	 * firmware settled on, used later by the RX gain tracking. */
	d->cal.agc_gain_init[0] = FIELD_GET(MT_BBP_AGC_GAIN, mt_rr(d, MT_BBP(AGC, 8)));
	d->cal.agc_gain_init[1] = FIELD_GET(MT_BBP_AGC_GAIN, mt_rr(d, MT_BBP(AGC, 9)));

	if (tssi_enabled(d)) {
		uint32_t flag = 0;

		mt_rmw(d, MT_TX_ALC_CFG_1, MT_TX_ALC_CFG_1_TEMP_COMP,
		       FIELD_PREP(MT_TX_ALC_CFG_1_TEMP_COMP, 0x38));
		mt_rmw(d, MT_TX_ALC_CFG_2, MT_TX_ALC_CFG_2_TEMP_COMP,
		       FIELD_PREP(MT_TX_ALC_CFG_2_TEMP_COMP, 0x38));

		if (band == BAND_5GHZ)
			flag |= BIT(0);
		if (ext_pa_enabled(d, band))
			flag |= BIT(8);
		mt_mcu_calibrate(d, MCU_CAL_TSSI, flag);
		d->cal.tssi_cal_done = 1;
	}
	if (mt_io_errors(d)) {
		ERR("channel %u set with %u failed register transfers - the tune "
		    "is incomplete", chan, mt_io_errors(d));
		d->chan = 0;
		return -1;
	}
	return 0;
}

int mt_set_channel(struct mt7612u_dev *d, uint8_t chan, uint8_t bw)
{
	return mt_set_channel_ex(d, chan, bw, 0);
}

/* Public: absolute TX power limit in dBm. Takes effect at the next channel
 * set, which is where the per-rate table is recomputed. */
int mt7612u_set_txpower(struct mt7612u_dev *d, int dbm)
{
	if (dbm < 0 || dbm > 30) return -1;
	d->txpower_conf = (int8_t)(dbm * 2);
	if (d->chan)
		mt_phy_set_txpower(d, d->chan > 14);
	return 0;
}

/*
 * Public channel set. `chan` is an 802.11 channel number, not a frequency;
 * the width may narrow a frame below the channel but never widen it.
 *
 * `chan` is always the *control* channel, at every width. mt_chan_group()
 * derives the centre the hardware actually tunes, and refuses a channel that
 * cannot carry the requested width rather than tuning near it.
 */
int mt7612u_set_channel(struct mt7612u_dev *d, uint8_t chan, enum mt7612u_bw bw)
{
	if (!d || !chan) return -1;
	if (bw != MT7612U_BW_20 && bw != MT7612U_BW_40 && bw != MT7612U_BW_80) {
		ERR("set_channel: bandwidth %d is not a MT7612U_BW_* value", (int)bw);
		return -1;
	}
	return mt_set_channel(d, chan, (uint8_t)bw);
}

/*
 * 0x202 = 2T2R, 0x101 = 1T1R. Stored now, applied at the next channel set,
 * which is where the chainmask reaches the firmware in CMD_SWITCH_CHANNEL_OP.
 */
int mt7612u_set_chainmask(struct mt7612u_dev *d, uint16_t chainmask)
{
	if (!d) return -1;
	if (chainmask != 0x0202 && chainmask != 0x0101) {
		ERR("set_chainmask: 0x%04x is neither 2T2R (0x0202) nor 1T1R (0x0101)",
		    chainmask);
		return -1;
	}
	d->chainmask = chainmask;
	return 0;
}
