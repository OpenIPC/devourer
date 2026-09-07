/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * EEPROM read and parse. The whole 512-byte image comes back through 128
 * EP0 vendor reads (bRequest 0x09) - there is no hardware EEPROM state
 * machine to drive. Ported from mt76/mt76x2/usb_init.c + mt76x02_eeprom.c.
 */
#include <string.h>
#include "internal.h"

uint16_t mt_ee(const struct mt7612u_dev *d, unsigned off)
{
	if (off + 1 >= sizeof d->eeprom)
		return 0;
	return (uint16_t)d->eeprom[off] | ((uint16_t)d->eeprom[off + 1] << 8);
}

int mt_eeprom_init(struct mt7612u_dev *d)
{
	/* Do NOT treat 0xffffffff as a read error by VALUE: unprogrammed EEPROM
	 * cells legitimately read as all-ones (this image is 0xff from 0x010
	 * onward for a stretch), so the value alone cannot tell a blank cell
	 * from a failed transfer.
	 *
	 * The accumulator can. mt_rr() returns ~0u on failure AND bumps
	 * d->io_err, so bracketing the slurp separates the two cases without
	 * giving up the blank-cell behaviour. This matters more here than
	 * anywhere else in the driver: a failure outside the two cells checked
	 * below leaves all-ones power and calibration bytes in memory, and the
	 * device then opens successfully and transmits at whatever those
	 * garbage terms produce. */
	mt_io_clear(d);
	for (unsigned i = 0; i + 4 <= MT7612U_EEPROM_SIZE; i += 4) {
		uint32_t v = mt_rr(d, EEP_ADDR(i));

		d->eeprom[i]     = v & 0xff;
		d->eeprom[i + 1] = (v >> 8) & 0xff;
		d->eeprom[i + 2] = (v >> 16) & 0xff;
		d->eeprom[i + 3] = (v >> 24) & 0xff;
	}
	if (mt_io_errors(d)) {
		ERR("EEPROM read failed on %u of %u transfers - refusing to "
		    "calibrate from a partial image",
		    mt_io_errors(d), MT7612U_EEPROM_SIZE / 4);
		return -1;
	}

	if (mt_ee(d, MT_EE_CHIP_ID) != 0x7612) {
		ERR("EEPROM chip id 0x%04x != 0x7612 - read path is wrong",
		    mt_ee(d, MT_EE_CHIP_ID));
		return -1;
	}

	memcpy(d->macaddr, d->eeprom + MT_EE_MAC_ADDR, 6);
	if (d->macaddr[0] == 0xff || (d->macaddr[0] & 1)) {
		ERR("EEPROM MAC looks invalid: %02x:%02x:%02x:%02x:%02x:%02x",
		    d->macaddr[0], d->macaddr[1], d->macaddr[2],
		    d->macaddr[3], d->macaddr[4], d->macaddr[5]);
		return -1;
	}

	/* mt76x02_init_device() hardcodes this for every mt76x2 - it is not
	 * derived from NIC_CONF_0. 0x202 = 2T2R. */
	d->chainmask = 0x202;

	LOG("EEPROM: chip 0x%04x  MAC %02x:%02x:%02x:%02x:%02x:%02x  "
	    "NIC_CONF 0/1/2 = %04x/%04x/%04x",
	    mt_ee(d, MT_EE_CHIP_ID),
	    d->macaddr[0], d->macaddr[1], d->macaddr[2],
	    d->macaddr[3], d->macaddr[4], d->macaddr[5],
	    mt_ee(d, MT_EE_NIC_CONF_0), mt_ee(d, MT_EE_NIC_CONF_1),
	    mt_ee(d, MT_EE_NIC_CONF_2));
	return 0;
}

/* ---- TX power tables ----
 * Ported from mt76/mt76x2/eeprom.c. Note mt76x02_sign_extend()'s convention:
 * the top bit SET means positive, clear means negate. Getting that backwards
 * silently halves or doubles output power.
 */
static int field_valid(uint8_t v) { return v != 0 && v != 0xff; }

static int sign_extend(uint32_t val, unsigned size)
{
	int sign = val & (1u << (size - 1));

	val &= (1u << (size - 1)) - 1;
	return sign ? (int)val : -(int)val;
}

static int sign_extend_optional(uint32_t val, unsigned size)
{
	return (val & (1u << size)) ? sign_extend(val, size) : 0;
}

static int8_t rate_power_val(uint8_t v)
{
	return field_valid(v) ? (int8_t)sign_extend_optional(v, 7) : 0;
}

void mt_get_rate_power(struct mt7612u_dev *d, struct mt_rate_power *t, int band)
{
	int is_5ghz = band != 0;
	uint16_t v;

	memset(t, 0, sizeof *t);

	v = mt_ee(d, MT_EE_TX_POWER_CCK);
	t->cck[0] = t->cck[1] = rate_power_val(v & 0xff);
	t->cck[2] = t->cck[3] = rate_power_val(v >> 8);

	v = mt_ee(d, is_5ghz ? MT_EE_TX_POWER_OFDM_5G_6M : MT_EE_TX_POWER_OFDM_2G_6M);
	t->ofdm[0] = t->ofdm[1] = rate_power_val(v & 0xff);
	t->ofdm[2] = t->ofdm[3] = rate_power_val(v >> 8);

	v = mt_ee(d, is_5ghz ? MT_EE_TX_POWER_OFDM_5G_24M : MT_EE_TX_POWER_OFDM_2G_24M);
	t->ofdm[4] = t->ofdm[5] = rate_power_val(v & 0xff);
	t->ofdm[6] = t->ofdm[7] = rate_power_val(v >> 8);

	v = mt_ee(d, MT_EE_TX_POWER_HT_MCS0);
	t->ht[0] = t->ht[1] = rate_power_val(v & 0xff);
	t->ht[2] = t->ht[3] = rate_power_val(v >> 8);
	v = mt_ee(d, MT_EE_TX_POWER_HT_MCS4);
	t->ht[4] = t->ht[5] = rate_power_val(v & 0xff);
	t->ht[6] = t->ht[7] = rate_power_val(v >> 8);
	v = mt_ee(d, MT_EE_TX_POWER_HT_MCS8);
	t->ht[8] = t->ht[9] = rate_power_val(v & 0xff);
	t->ht[10] = t->ht[11] = rate_power_val(v >> 8);
	v = mt_ee(d, MT_EE_TX_POWER_HT_MCS12);
	t->ht[12] = t->ht[13] = rate_power_val(v & 0xff);
	t->ht[14] = t->ht[15] = rate_power_val(v >> 8);

	/* The double shift on 2.4 GHz is upstream's, not a transcription slip:
	 * mt76x2_get_rate_power() is byte-for-byte this, and rate_power_val()
	 * takes a u8 on both sides. So on 2.4 GHz these two entries decode from
	 * zero, i.e. VHT MCS8/9 there get no per-rate offset off the base target
	 * power. The EEPROM field is a 5 GHz one and 2.4 GHz VHT is an extension
	 * outside 802.11ac, so this is left matching mt76 deliberately - fixing
	 * it here would be a silent divergence from the reference this port is
	 * checked against. */
	v = mt_ee(d, MT_EE_TX_POWER_VHT_MCS8);
	if (!is_5ghz)
		v >>= 8;
	t->vht[0] = t->vht[1] = rate_power_val(v >> 8);
}

/* mt76x2_get_cal_channel_group() */
static int cal_channel_group(int ch)
{
	if (ch >= 184 && ch <= 196) return 0;  /* JAPAN */
	if (ch <= 48)  return 1;               /* UNII-1 */
	if (ch <= 64)  return 2;               /* UNII-2 */
	if (ch <= 114) return 3;               /* UNII-2E-1 */
	if (ch <= 144) return 4;               /* UNII-2E-2 */
	return 5;                              /* UNII-3 */
}

static void power_info_5g(struct mt7612u_dev *d, struct mt_tx_power_info *t,
                          uint8_t ch, int chain, unsigned offset)
{
	int delta_idx;
	const uint8_t *p;

	offset += (unsigned)cal_channel_group(ch) * MT_TX_POWER_GROUP_SIZE_5G;

	if      (ch >= 192) delta_idx = 4;
	else if (ch >= 184) delta_idx = 3;
	else if (ch < 44)   delta_idx = 3;
	else if (ch < 52)   delta_idx = 4;
	else if (ch < 58)   delta_idx = 3;
	else if (ch < 98)   delta_idx = 4;
	else if (ch < 106)  delta_idx = 3;
	else if (ch < 116)  delta_idx = 4;
	else if (ch < 130)  delta_idx = 3;
	else if (ch < 149)  delta_idx = 4;
	else if (ch < 157)  delta_idx = 3;
	else                delta_idx = 4;

	if (offset + 5 > sizeof d->eeprom) return;
	p = d->eeprom + offset;
	t->chain[chain].tssi_slope   = (int8_t)p[0];
	t->chain[chain].tssi_offset  = (int8_t)p[1];
	t->chain[chain].target_power = (int8_t)p[2];
	t->chain[chain].delta = (int8_t)sign_extend_optional(p[delta_idx], 7);

	t->target_power = mt_ee(d, MT_EE_RF_2G_RX_HIGH_GAIN) & 0xff;
}

static void power_info_2g(struct mt7612u_dev *d, struct mt_tx_power_info *t,
                          uint8_t ch, int chain, unsigned offset)
{
	int delta_idx = ch < 6 ? 3 : (ch < 11 ? 4 : 5);
	const uint8_t *p;

	if (offset + 6 > sizeof d->eeprom) return;
	p = d->eeprom + offset;
	t->chain[chain].tssi_slope   = (int8_t)p[0];
	t->chain[chain].tssi_offset  = (int8_t)p[1];
	t->chain[chain].target_power = (int8_t)p[2];
	t->chain[chain].delta = (int8_t)sign_extend_optional(p[delta_idx], 7);

	t->target_power = mt_ee(d, MT_EE_RF_2G_TSSI_OFF_TXPOWER) >> 8;
}

void mt_get_power_info(struct mt7612u_dev *d, struct mt_tx_power_info *t,
                       uint8_t chan, int band)
{
	uint16_t bw40 = mt_ee(d, MT_EE_TX_POWER_DELTA_BW40);
	uint16_t bw80 = mt_ee(d, MT_EE_TX_POWER_DELTA_BW80);

	memset(t, 0, sizeof *t);

	if (band != 0) {
		bw40 >>= 8;
		power_info_5g(d, t, chan, 0, MT_EE_TX_POWER_0_START_5G);
		power_info_5g(d, t, chan, 1, MT_EE_TX_POWER_1_START_5G);
	} else {
		power_info_2g(d, t, chan, 0, MT_EE_TX_POWER_0_START_2G);
		power_info_2g(d, t, chan, 1, MT_EE_TX_POWER_1_START_2G);
	}

	/* mt76 also takes chain 0's value whenever TSSI is enabled, not only
	 * when the EEPROM field is unprogrammed - and TSSI IS enabled on this
	 * part (NIC_CONF_1 TX_ALC_EN set, TEMP_TX_ALC clear). */
	if (mt_tssi_enabled(d) || !field_valid(t->target_power))
		t->target_power = (uint8_t)t->chain[0].target_power;

	t->delta_bw40 = rate_power_val(bw40 & 0xff);
	t->delta_bw80 = rate_power_val(bw80 & 0xff);
}

/* ---- RX gain / RSSI correction ----
 * mt76x2_read_rx_gain(). Without this the per-chain RSSI the RX path reports
 * is the raw chip value: no LNA gain removed and no per-chain offset applied.
 */
static int8_t se4(uint8_t v)  { return (int8_t)sign_extend(v, 4); }

static uint8_t get_5g_rx_gain(struct mt7612u_dev *d, uint8_t chan)
{
	switch (cal_channel_group(chan)) {
	case 0: return mt_ee(d, MT_EE_RF_5G_GRP0_1_RX_HIGH_GAIN) & 0xff;
	case 1: return mt_ee(d, MT_EE_RF_5G_GRP0_1_RX_HIGH_GAIN) >> 8;
	case 2: return mt_ee(d, MT_EE_RF_5G_GRP2_3_RX_HIGH_GAIN) & 0xff;
	case 3: return mt_ee(d, MT_EE_RF_5G_GRP2_3_RX_HIGH_GAIN) >> 8;
	case 4: return mt_ee(d, MT_EE_RF_5G_GRP4_5_RX_HIGH_GAIN) & 0xff;
	default: return mt_ee(d, MT_EE_RF_5G_GRP4_5_RX_HIGH_GAIN) >> 8;
	}
}

void mt_read_rx_gain(struct mt7612u_dev *d, uint8_t chan, int band)
{
	int8_t lna_2g, lna_5g[3];
	uint16_t rssi_off, v;
	uint8_t gain, lna = 0;

	gain = (band == 0) ? (uint8_t)(mt_ee(d, MT_EE_RF_2G_RX_HIGH_GAIN) >> 8)
	                   : get_5g_rx_gain(d, chan);

	/* mt76x2_set_rx_gain_group(): two nibbles, sign-extended from 4 bits. */
	if (field_valid(gain)) {
		d->cal.high_gain[0] = se4(gain & 0xf);
		d->cal.high_gain[1] = se4((gain >> 4) & 0xf);
	} else {
		d->cal.high_gain[0] = d->cal.high_gain[1] = 0;
	}

	v = mt_ee(d, MT_EE_LNA_GAIN);
	lna_2g    = (int8_t)(v & 0xff);
	lna_5g[0] = (int8_t)(v >> 8);
	lna_5g[1] = (int8_t)(mt_ee(d, MT_EE_RSSI_OFFSET_2G_1) >> 8);
	lna_5g[2] = (int8_t)(mt_ee(d, MT_EE_RSSI_OFFSET_5G_1) >> 8);
	if (!field_valid((uint8_t)lna_5g[1])) lna_5g[1] = lna_5g[0];
	if (!field_valid((uint8_t)lna_5g[2])) lna_5g[2] = lna_5g[0];

	rssi_off = (band == 0) ? mt_ee(d, MT_EE_RSSI_OFFSET_2G_0)
	                       : mt_ee(d, MT_EE_RSSI_OFFSET_5G_0);
	d->cal.rssi_offset[0] = field_valid(rssi_off & 0xff)
		? (int8_t)sign_extend_optional(rssi_off & 0xff, 7) : 0;
	d->cal.rssi_offset[1] = field_valid(rssi_off >> 8)
		? (int8_t)sign_extend_optional(rssi_off >> 8, 7) : 0;

	/* mt76x02_get_lna_gain(): which LNA entry applies to this channel. */
	if (band == 0)          lna = (uint8_t)lna_2g;
	else if (chan <= 64)    lna = (uint8_t)lna_5g[0];
	else if (chan <= 128)   lna = (uint8_t)lna_5g[1];
	else                    lna = (uint8_t)lna_5g[2];
	if (lna == 0xff) lna = 0;

	/* mt76x2_has_ext_lna(): an external LNA means its gain is NOT removed. */
	{
		uint16_t c1 = mt_ee(d, MT_EE_NIC_CONF_1);
		int ext = (band == 0) ? (c1 & MT_EE_NIC_CONF_1_LNA_EXT_2G)
		                      : (c1 & MT_EE_NIC_CONF_1_LNA_EXT_5G);
		d->cal.lna_gain = ext ? 0 : (int8_t)sign_extend(lna, 8);
	}

	d->cal.mcu_gain  = (uint32_t)(lna_2g & 0xff);
	d->cal.mcu_gain |= (uint32_t)(lna_5g[0] & 0xff) << 8;
	d->cal.mcu_gain |= (uint32_t)(lna_5g[1] & 0xff) << 16;
	d->cal.mcu_gain |= (uint32_t)(lna_5g[2] & 0xff) << 24;
}
