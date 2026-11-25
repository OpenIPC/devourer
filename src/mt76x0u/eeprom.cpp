/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MT76x0U calibration data: the on-die EFUSE read and what mt76x0 derives from
 * it. A port of mt76/mt76x0/eeprom.c (GPL-2.0-only; Copyright (C) 2014 Felix
 * Fietkau, (C) 2015 Jakub Kicinski, (C) 2018 Stanislaw Gruszka) and the shared
 * mt76x02_eeprom.c helpers (BSD-3-Clause-Clear; (C) 2016 Felix Fietkau,
 * (C) 2018 Lorenzo Bianconi), reference/mt76 @ be5ce79.
 *
 * USB parts have no OF/MTD EEPROM, so mt76_eeprom_init() finds nothing and
 * mt76x0_load_eeprom() goes straight to the EFUSE: a usage-map sanity check,
 * then 512 bytes read 16 at a time through the MT_EFUSE_CTRL KICK protocol
 * (NOT the MT_VEND_READ_EEPROM request, which is for an external EEPROM this
 * silicon does not have - wifit3 notes the same).
 */
#include <string.h>
#include "internal.h"

/* --- pure helpers (tests/mt76x0u_eeprom_selftest.cpp) --------------------- */

uint16_t mt0_ee_get(const uint8_t *ee, unsigned field)
{
	return (uint16_t)(ee[field] | (ee[field + 1] << 8));
}

int mt0_field_valid(uint8_t val)
{
	return val != 0 && val != 0xff;
}

/* mt76x02_sign_extend: bit (size-1) is the sign, the rest a MAGNITUDE (this
 * is sign-and-magnitude, not two's complement), and the sign bit SET means
 * positive. */
int mt0_sign_extend(uint32_t val, unsigned size)
{
	const int sign = (val & (1u << (size - 1))) != 0;

	val &= (1u << (size - 1)) - 1;
	return sign ? (int)val : -(int)val;
}

int8_t mt0_ee_lna_gain(const uint8_t *ee, uint8_t chan)
{
	/* mt76x02_get_rx_gain */
	uint16_t val = mt0_ee_get(ee, MT_EE_LNA_GAIN);
	const uint8_t lna_2g = (uint8_t)(val & 0xff);
	uint8_t lna_5g[3];

	lna_5g[0] = (uint8_t)(val >> 8);
	lna_5g[1] = (uint8_t)(mt0_ee_get(ee, MT_EE_RSSI_OFFSET_2G_1) >> 8);
	lna_5g[2] = (uint8_t)(mt0_ee_get(ee, MT_EE_RSSI_OFFSET_5G_1) >> 8);
	if (!mt0_field_valid(lna_5g[1]))
		lna_5g[1] = lna_5g[0];
	if (!mt0_field_valid(lna_5g[2]))
		lna_5g[2] = lna_5g[0];

	/* mt76x02_get_lna_gain: band by channel number, 5 GHz sub-band by
	 * hw_value. The result is stored in an s8 (cal.rx.lna_gain). */
	uint8_t lna;
	if (chan <= 14)
		lna = lna_2g;
	else if (chan <= 64)
		lna = lna_5g[0];
	else if (chan <= 128)
		lna = lna_5g[1];
	else
		lna = lna_5g[2];
	return (int8_t)(lna != 0xff ? lna : 0);
}

void mt0_ee_rssi_offset(const uint8_t *ee, uint8_t chan, int8_t out[2])
{
	/* mt76x0_read_rx_gain: per-chain offsets for the band, |x| <= 10. */
	const uint16_t v = mt0_ee_get(ee, chan <= 14 ? MT_EE_RSSI_OFFSET_2G_0
	                                             : MT_EE_RSSI_OFFSET_5G_0);
	for (int i = 0; i < 2; i++) {
		int8_t s = (int8_t)(uint8_t)(v >> (8 * i));
		out[i] = (s < -10 || s > 10) ? 0 : s;
	}
}

int mt0_ee_parse(struct mt76x0u_dev *d)
{
	const uint8_t *ee = d->eeprom;
	const uint16_t ver = mt0_ee_get(ee, MT_EE_VERSION);

	d->ee_version = (uint8_t)(ver >> 8);
	d->ee_fae = (uint8_t)ver;
	if (d->ee_version > MT76X0U_EE_MAX_VER)
		WARN("unsupported EEPROM version %02x", d->ee_version);
	LOG("EEPROM ver:%02x fae:%02x", d->ee_version, d->ee_fae);

	memcpy(d->macaddr, ee + MT_EE_MAC_ADDR, 6);

	/* mt76x0_set_chip_cap -> mt76x02_eeprom_parse_hw_cap */
	const uint16_t nic_conf0 = mt0_ee_get(ee, MT_EE_NIC_CONF_0);
	uint16_t nic_conf1 = mt0_ee_get(ee, MT_EE_NIC_CONF_1);
	switch (FIELD_GET(MT_EE_NIC_CONF_0_BOARD_TYPE, nic_conf0)) {
	case BOARD_TYPE_5GHZ: d->has_2ghz = 0; d->has_5ghz = 1; break;
	case BOARD_TYPE_2GHZ: d->has_2ghz = 1; d->has_5ghz = 0; break;
	default:              d->has_2ghz = 1; d->has_5ghz = 1; break;
	}
	if (d->no_2ghz)
		d->has_2ghz = 0;
	if (d->is_mt7630)
		d->has_5ghz = 0;
	if (!mt0_field_valid((uint8_t)(nic_conf1 & 0xff)))
		nic_conf1 &= 0xff00;
	if (nic_conf1 & MT_EE_NIC_CONF_1_HW_RF_CTRL)
		LOG("EEPROM asks for HW RF control, which the driver does not support");
	if (mt0_field_valid((uint8_t)(nic_conf0 >> 8)) &&
	    (FIELD_GET(MT_EE_NIC_CONF_0_RX_PATH, nic_conf0) > 1 ||
	     FIELD_GET(MT_EE_NIC_CONF_0_TX_PATH, nic_conf0) > 1))
		ERR("EEPROM: invalid tx-rx stream");

	/* mt76x0_set_freq_offset */
	uint8_t v = (uint8_t)mt0_ee_get(ee, MT_EE_FREQ_OFFSET);
	if (!mt0_field_valid(v))
		v = 0;
	d->freq_offset = v;
	v = (uint8_t)(mt0_ee_get(ee, MT_EE_TSSI_BOUND4) >> 8);
	if (!mt0_field_valid(v))
		v = 0;
	d->freq_offset = (uint8_t)(d->freq_offset - mt0_sign_extend(v, 8));

	/* mt76x0_set_temp_offset */
	v = (uint8_t)(mt0_ee_get(ee, MT_EE_2G_TARGET_POWER) >> 8);
	d->temp_offset = mt0_field_valid(v) ? (int8_t)mt0_sign_extend(v, 8) : -10;

	LOG("bands:%s%s freq_offset 0x%02x temp_offset %d",
	    d->has_2ghz ? " 2.4GHz" : "", d->has_5ghz ? " 5GHz" : "",
	    d->freq_offset, d->temp_offset);
	return 0;
}

/* --- EFUSE access ---------------------------------------------------------- */

/* mt76x02_efuse_read: one 16-byte block. */
static int efuse_read(struct mt76x0u_dev *d, uint16_t addr, uint8_t *data,
                      int mode)
{
	uint32_t val;

	if (mt0_rr_chk(d, MT_EFUSE_CTRL, &val))
		return -1;
	val &= ~(MT_EFUSE_CTRL_AIN | MT_EFUSE_CTRL_MODE);
	val |= FIELD_PREP(MT_EFUSE_CTRL_AIN, addr & ~0xfu);
	val |= FIELD_PREP(MT_EFUSE_CTRL_MODE, (uint32_t)mode);
	val |= MT_EFUSE_CTRL_KICK;
	mt0_wr(d, MT_EFUSE_CTRL, val);

	if (!mt0_poll(d, MT_EFUSE_CTRL, MT_EFUSE_CTRL_KICK, 0, 1000000)) {
		ERR("EFUSE read at 0x%03x timed out", addr);
		return -1;
	}
	mt0_usleep(2);

	if (mt0_rr_chk(d, MT_EFUSE_CTRL, &val))
		return -1;
	/* An unburned block reads AOUT == 0x3f: all ones, as the kernel. */
	if ((val & MT_EFUSE_CTRL_AOUT) == MT_EFUSE_CTRL_AOUT) {
		memset(data, 0xff, 16);
		return 0;
	}
	for (int i = 0; i < 4; i++) {
		uint32_t w;
		if (mt0_rr_chk(d, MT_EFUSE_DATA(i), &w))
			return -1;
		data[4 * i + 0] = (uint8_t)w;
		data[4 * i + 1] = (uint8_t)(w >> 8);
		data[4 * i + 2] = (uint8_t)(w >> 16);
		data[4 * i + 3] = (uint8_t)(w >> 24);
	}
	return 0;
}

static int get_efuse_data(struct mt76x0u_dev *d, uint16_t base, uint8_t *buf,
                          int len, int mode)
{
	for (int i = 0; i + 16 <= len; i += 16)
		if (efuse_read(d, (uint16_t)(base + i), buf + i, mode))
			return -1;
	return 0;
}

/* mt76x0_efuse_physical_size_check */
static int efuse_physical_size_check(struct mt76x0u_dev *d)
{
	enum { READS = (MT_EFUSE_USAGE_MAP_SIZE + 15) / 16 };
	uint8_t data[READS * 16];
	uint32_t start = 0, end = 0;

	if (get_efuse_data(d, MT_EE_USAGE_MAP_START, data, sizeof data,
	                   MT_EE_PHYSICAL_READ))
		return -1;
	for (int i = 0; i < MT_EFUSE_USAGE_MAP_SIZE; i++)
		if (!data[i]) {
			if (!start)
				start = MT_EE_USAGE_MAP_START + i;
			end = MT_EE_USAGE_MAP_START + i;
		}
	const uint32_t cnt_free = end - start + 1;
	if (MT_EFUSE_USAGE_MAP_SIZE - cnt_free < 5) {
		ERR("driver does not support default EEPROM (EFUSE usage map "
		    "nearly empty: the card's calibration was never burned)");
		return -1;
	}
	return 0;
}

int mt0_eeprom_init(struct mt76x0u_dev *d)
{
	uint32_t ctrl;

	/* mt76x0u_probe: "vendor driver skips this check for MT76X0U". */
	if (!mt0_rr_chk(d, MT_EFUSE_CTRL, &ctrl) && !(ctrl & MT_EFUSE_CTRL_SEL))
		WARN("eFUSE not present");

	if (efuse_physical_size_check(d))
		return -1;
	if (get_efuse_data(d, 0, d->eeprom, MT76X0_EEPROM_SIZE, MT_EE_READ))
		return -1;
	d->eeprom_loaded = 1;
	return mt0_ee_parse(d);
}

int mt76x0u_eeprom(const struct mt76x0u_dev *d, uint8_t *out, size_t len)
{
	if (!d || !d->eeprom_loaded || !out || len < MT76X0_EEPROM_SIZE)
		return -1;
	memcpy(out, d->eeprom, MT76X0_EEPROM_SIZE);
	return MT76X0_EEPROM_SIZE;
}

const uint8_t *mt76x0u_mac_addr(const struct mt76x0u_dev *d)
{
	return d && d->eeprom_loaded ? d->macaddr : NULL;
}
