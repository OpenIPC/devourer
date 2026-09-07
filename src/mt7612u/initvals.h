/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/* GENERATED - do not hand-edit.
 * Source: openwrt/mt76 commit be5ce79, mt76_write_mac_initvals() in mt76x2/init.c,
 * with register names and the DEFAULT_PROT_CFG_* macros resolved
 * against mt76x02_regs.h.
 * Regenerate:  tools/extract_mt7612u_tables.py
 * Verify:      tools/extract_mt7612u_tables.py --check
 * Table SHA-256 (LE reg,val stream): ba381217626e876b27a2aa9150440ff7c6af86740a3c64afbb36e81347df3719
 */
#ifndef MT7612U_INITVALS_H
#define MT7612U_INITVALS_H

static const struct { uint32_t reg; uint32_t val; } mt7612u_mac_initvals[] = {
	{ 0x0400, 0x00080c00 },  /* MT_PBF_SYS_CTRL */
	{ 0x0404, 0x1efebcff },  /* MT_PBF_CFG */
	{ 0x0800, 0x00000001 },  /* MT_FCE_PSE_CTRL */
	{ 0x1004, 0x00000000 },  /* MT_MAC_SYS_CTRL */
	{ 0x1018, 0x003e3f00 },  /* MT_MAX_LEN_CFG */
	{ 0x1030, 0xaaa99887 },  /* MT_AMPDU_MAX_LEN_20M1S */
	{ 0x1034, 0x000000aa },  /* MT_AMPDU_MAX_LEN_20M2S */
	{ 0x1100, 0x33a40d0a },  /* MT_XIFS_TIME_CFG */
	{ 0x1104, 0x00000209 },  /* MT_BKOFF_SLOT_CFG */
	{ 0x1118, 0x00422010 },  /* MT_TBTT_SYNC_CFG */
	{ 0x1204, 0x00000000 },  /* MT_PWR_PIN_CFG */
	{ 0x1238, 0x001700c8 },
	{ 0x1330, 0x00101001 },  /* MT_TX_SW_CFG0 */
	{ 0x1334, 0x00010000 },  /* MT_TX_SW_CFG1 */
	{ 0x1338, 0x00000000 },  /* MT_TX_SW_CFG2 */
	{ 0x1340, 0x0400583f },  /* MT_TXOP_CTRL_CFG */
	{ 0x1344, 0x00ffff20 },  /* MT_TX_RTS_CFG */
	{ 0x1348, 0x000a2290 },  /* MT_TX_TIMEOUT_CFG */
	{ 0x134c, 0x47f01f0f },  /* MT_TX_RETRY_CFG */
	{ 0x1380, 0x002c00dc },  /* MT_EXP_ACK_TIME */
	{ 0x13e0, 0xe3f42004 },  /* MT_TX_PROT_CFG6 */
	{ 0x13e4, 0xe3f42084 },  /* MT_TX_PROT_CFG7 */
	{ 0x13e8, 0xe3f42104 },  /* MT_TX_PROT_CFG8 */
	{ 0x13ec, 0x00060fff },  /* MT_PIFS_TX_CFG */
	{ 0x1400, 0x00015f97 },  /* MT_RX_FILTR_CFG */
	{ 0x1408, 0x0000017f },  /* MT_LEGACY_BASIC_RATE */
	{ 0x140c, 0x00004003 },  /* MT_HT_BASIC_RATE */
	{ 0x150c, 0x00000003 },  /* MT_PN_PAD_MODE */
	{ 0x1608, 0x00000002 },  /* MT_TXOP_HLDR_ET */
	{ 0x0a44, 0x00000000 },
	{ 0x0260, 0x00000000 },  /* MT_HEADER_TRANS_CTRL_REG */
	{ 0x0250, 0x00000000 },  /* MT_TSO_CTRL */
	{ 0x120c, 0x00000000 },  /* MT_AUX_CLK_CFG */
	{ 0x1264, 0x00000000 },  /* MT_DACCLK_EN_DLY_CFG */
	{ 0x13c0, 0x00000000 },  /* MT_TX_ALC_CFG_4 */
	{ 0x13c8, 0x00000000 },  /* MT_TX_ALC_VGA3 */
	{ 0x1314, 0x3a3a3a3a },  /* MT_TX_PWR_CFG_0 */
	{ 0x1318, 0x3a3a3a3a },  /* MT_TX_PWR_CFG_1 */
	{ 0x131c, 0x3a3a3a3a },  /* MT_TX_PWR_CFG_2 */
	{ 0x1320, 0x3a3a3a3a },  /* MT_TX_PWR_CFG_3 */
	{ 0x1324, 0x3a3a3a3a },  /* MT_TX_PWR_CFG_4 */
	{ 0x13d4, 0x3a3a3a3a },  /* MT_TX_PWR_CFG_7 */
	{ 0x13d8, 0x0000003a },  /* MT_TX_PWR_CFG_8 */
	{ 0x13dc, 0x0000003a },  /* MT_TX_PWR_CFG_9 */
	{ 0x0024, 0x0000d000 },  /* MT_EFUSE_CTRL */
	{ 0x0a38, 0x0000000a },  /* MT_PAUSE_ENABLE_CONTROL1 */
	{ 0x0824, 0x60401c18 },  /* MT_FCE_WLAN_FLOW_CONTROL1 */
	{ 0x0210, 0x94ff0000 },  /* MT_WPDMA_DELAY_INT_CFG */
	{ 0x1478, 0x00000004 },  /* MT_TX_SW_CFG3 */
	{ 0x1384, 0x00001818 },  /* MT_HT_FBK_TO_LEGACY */
	{ 0x1358, 0xedcba980 },  /* MT_VHT_HT_FBK_CFG1 */
	{ 0x1648, 0x00830083 },  /* MT_PROT_AUTO_TX_CFG */
	{ 0x1410, 0x000001ff },  /* MT_HT_CTRL_CFG */
	{ 0x1350, 0x00001020 },  /* MT_TX_LINK_CFG */
	{ 0x1364, 0x07f40003 },  /* MT_CCK_PROT_CFG */
	{ 0x1368, 0x07f42004 },  /* MT_OFDM_PROT_CFG */
	{ 0x136c, 0x01752004 },  /* MT_MM20_PROT_CFG */
	{ 0x1370, 0x03f52084 },  /* MT_MM40_PROT_CFG */
	{ 0x1374, 0x01752004 },  /* MT_GF20_PROT_CFG */
	{ 0x1378, 0x03f52084 },  /* MT_GF40_PROT_CFG */
};
#endif
