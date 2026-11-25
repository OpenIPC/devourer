/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MT76x0U bring-up: the power-on sequence and hardware init. A port of
 * mt76/mt76x0/init.c and mt76x0/usb.c (GPL-2.0-only; Copyright (C) 2014 Felix
 * Fietkau, (C) 2015 Jakub Kicinski, (C) 2018 Stanislaw Gruszka, Lorenzo
 * Bianconi) plus the mt76x02 MAC helpers it calls (BSD-3-Clause-Clear),
 * reference/mt76 @ be5ce79.
 *
 * Order, from mt76x0u_probe() + mt76x0u_init_hardware(reset = true):
 *   (no USB reset: devourer's open path skips it for MT76x0U ids - it
 *    desyncs the running firmware's command ring; bench 2026-11-19)
 *   chip off            "otherwise MCU fail to initialize on hot reboot"
 *   wait for MAC
 *   chip on, with reset (pulses WLAN_RESET only if WLAN_EN is still set -
 *                        and the chip-off above has just cleared it)
 *   wait for MAC
 *   firmware            (fw.cpp; kept if already running, never re-uploaded)
 *   warm only: drain EP 0x84 if RX_BUSY, pulse PBF DMA|PBF reset
 *   USB DMA init
 *   mt76x0_init_hardware: WPDMA idle, MAC, CSR/BBP reset, Q_SELECT, MAC
 *     tables, TX/RX idle, BBP tables, key + WCID clear, EFUSE, PHY init
 *   US cycle count, TXOP control
 * The kernel's mt76x02u_init_beacon_config is not ported: this backend
 * never beacons.
 *
 * Nothing here enables MAC RX: that waits for an RX ring to drain EP 0x84
 * (phase 5), the ordering the MT7612U needs to avoid wedging.
 */
#include <random>
#include <string.h>
#include "internal.h"
#include "phy.h"
#include "initvals.h"

#define ARRAY_SIZE(a) ((int)(sizeof(a) / sizeof((a)[0])))

/* mt76x0_set_wlan_state. WLAN_CLK is deliberately never turned off (the
 * kernel's note: the device then does not respond properly on probe). */
static void set_wlan_state(struct mt76x0u_dev *d, uint32_t val, int enable)
{
	const uint32_t mask = MT_CMB_CTRL_XTAL_RDY | MT_CMB_CTRL_PLL_LD;

	if (enable)
		val |= MT_WLAN_FUN_CTRL_WLAN_EN | MT_WLAN_FUN_CTRL_WLAN_CLK_EN;
	else
		val &= ~MT_WLAN_FUN_CTRL_WLAN_EN;
	mt0_wr(d, MT_WLAN_FUN_CTRL, val);
	mt0_usleep(20);

	if (enable && !mt0_poll(d, MT_CMB_CTRL, mask, mask, 2000000))
		ERR("PLL and XTAL check failed");
}

/* mt76x0_chip_onoff */
void mt0_chip_onoff(struct mt76x0u_dev *d, int enable, int reset)
{
	uint32_t val;

	if (mt0_rr_chk(d, MT_WLAN_FUN_CTRL, &val)) {
		ERR("MT_WLAN_FUN_CTRL unreadable - chip %s skipped", enable ? "on" : "off");
		return;
	}
	if (reset) {
		val |= MT_WLAN_FUN_CTRL_GPIO_OUT_EN;
		val &= ~MT_WLAN_FUN_CTRL_FRC_WL_ANT_SEL;
		if (val & MT_WLAN_FUN_CTRL_WLAN_EN) {
			val |= MT_WLAN_FUN_CTRL_WLAN_RESET | MT_WLAN_FUN_CTRL_WLAN_RESET_RF;
			mt0_wr(d, MT_WLAN_FUN_CTRL, val);
			mt0_usleep(20);
			val &= ~(MT_WLAN_FUN_CTRL_WLAN_RESET | MT_WLAN_FUN_CTRL_WLAN_RESET_RF);
		} else {
			/* Logged because it is the open warm-start question (see
			 * fw.cpp): after the probe-time chip-off this is the path
			 * the kernel takes too. */
			LOG("chip on: WLAN_EN clear, so no WLAN reset pulse");
		}
	}
	mt0_wr(d, MT_WLAN_FUN_CTRL, val);
	mt0_usleep(20);
	set_wlan_state(d, val, enable);
}

/* mt0_wait_for_mac() is the transport's (usb.cpp). */

static void reset_csr_bbp(struct mt76x0u_dev *d)
{
	mt0_wr(d, MT_MAC_SYS_CTRL, MT_MAC_SYS_CTRL_RESET_CSR | MT_MAC_SYS_CTRL_RESET_BBP);
	mt0_usleep(200000);
	mt0_clear(d, MT_MAC_SYS_CTRL, MT_MAC_SYS_CTRL_RESET_CSR | MT_MAC_SYS_CTRL_RESET_BBP);
}

/*
 * Not in the kernel. A warm bring-up (firmware kept running) after a session
 * that died without stopping RX (kill -9) finds USB_DMA_CFG RX_BUSY set and
 * frames parked in EP 0x84, which hold back MCU replies on EP 5 - Q_SELECT
 * then times out. Bench 2026-11-19: drain EP 0x84 when RX_BUSY is set, then
 * pulse the PBF block's DMA and PBF resets (not MCU/MAC: the firmware and its
 * command ring must survive) so the RX DMA starts from an empty state.
 */
static void warm_dma_reset(struct mt76x0u_dev *d)
{
	const uint32_t rst = MT_PBF_SYS_CTRL_DMA_RESET | MT_PBF_SYS_CTRL_PBF_RESET;
	uint32_t val;

	if (!mt0_rr_chk(d, MT_USB_DMA_CFG, &val) && (val & MT_USB_DMA_CFG_RX_BUSY)) {
		int reads = 256;
		size_t total = 0, n;

		/* Until a drain comes back empty, within one 256-read budget. */
		while (reads > 0 && (n = mt0_rx_flush(d, &reads)) > 0)
			total += n;
		LOG("warm start: USB_DMA_CFG 0x%08x RX_BUSY set - drained %zu B "
		    "from EP 0x84", val, total);
	}
	mt0_set(d, MT_PBF_SYS_CTRL, rst);
	mt0_usleep(1000);
	mt0_clear(d, MT_PBF_SYS_CTRL, rst);
	LOG("warm start: PBF DMA|PBF reset pulsed");
}

/* mt76x0_init_usb_dma (usb.c) */
static void init_usb_dma(struct mt76x0u_dev *d)
{
	uint32_t val;

	if (mt0_rr_chk(d, MT_USB_DMA_CFG, &val))
		return;
	val |= MT_USB_DMA_CFG_RX_BULK_EN | MT_USB_DMA_CFG_TX_BULK_EN;
	/* "disable AGGR_BULK_RX in order to receive one frame in each rx urb
	 * and avoid copies" - so one RX transfer carries one frame. */
	val &= ~MT_USB_DMA_CFG_RX_BULK_AGG_EN;
	mt0_wr(d, MT_USB_DMA_CFG, val);

	/* The kernel reads MT_COM_REG0 here only to log "MCU not ready" at
	 * debug level; the read is kept so the transfer sequence matches. */
	(void)mt0_rr(d, MT_MCU_COM_REG0);

	if (mt0_rr_chk(d, MT_USB_DMA_CFG, &val))
		return;
	val |= MT_USB_DMA_CFG_RX_DROP_OR_PAD;
	mt0_wr(d, MT_USB_DMA_CFG, val);
	val &= ~MT_USB_DMA_CFG_RX_DROP_OR_PAD;
	mt0_wr(d, MT_USB_DMA_CFG, val);
}

/* mt76x0_init_mac_registers. The RANDOM_WRITE tables go through the MCU
 * (MT_MCU_MEMMAP_WLAN), so Q_SELECT must have gone first. */
static int init_mac_registers(struct mt76x0u_dev *d)
{
	if (mt0_mcu_wr_rp(d, MT_MCU_MEMMAP_WLAN, common_mac_reg_table,
	                  ARRAY_SIZE(common_mac_reg_table)) ||
	    mt0_mcu_wr_rp(d, MT_MCU_MEMMAP_WLAN, mt76x0_mac_reg_table,
	                  ARRAY_SIZE(mt76x0_mac_reg_table)))
		return -1;
	/* Release BBP and MAC reset MAC_SYS_CTRL[1:0] = 0x0 */
	mt0_clear(d, MT_MAC_SYS_CTRL, 0x3);
	/* Set 0x141C[15:12]=0xF */
	mt0_set(d, MT_EXT_CCA_CFG, 0xf000);
	mt0_clear(d, MT_FCE_L2_STUFF, MT_FCE_L2_STUFF_WR_MPDU_LEN_EN);
	/* tx_ring 9 is for mgmt frames, tx_ring 8 for in-band commands;
	 * WMM_RG0/1_TXQMA tell the FCE the rules for each. */
	mt0_rmw(d, MT_WMM_CTRL, 0x3ff, 0x201);
	return 0;
}

/* mt76x0_init_bbp */
static int init_bbp(struct mt76x0u_dev *d)
{
	if (mt0_phy_wait_bbp_ready(d))
		return -1;
	if (mt0_mcu_wr_rp(d, MT_MCU_MEMMAP_WLAN, mt76x0_bbp_init_tab,
	                  ARRAY_SIZE(mt76x0_bbp_init_tab)))
		return -1;
	for (int i = 0; i < ARRAY_SIZE(mt76x0_bbp_switch_tab); i++) {
		const struct mt0_bbp_switch_item *it = &mt76x0_bbp_switch_tab[i];
		if (((RF_G_BAND | RF_BW_20) & it->bw_band) == (RF_G_BAND | RF_BW_20))
			mt0_wr(d, it->reg_pair.reg, it->reg_pair.value);
	}
	return mt0_mcu_wr_rp(d, MT_MCU_MEMMAP_WLAN, mt76x0_dcoc_tab,
	                     ARRAY_SIZE(mt76x0_dcoc_tab));
}

/* mt76x02_mac_shared_key_setup(.., NULL) for every slot and
 * mt76x02_mac_wcid_setup(.., 0, NULL) for every WCID - the same clear the
 * MT7612U backend does (src/mt7612u/init.cpp wcid_and_key_clear). */
static void wcid_and_key_clear(struct mt76x0u_dev *d)
{
	uint8_t zero32[32] = { 0 };

	for (int bss = 0; bss < 16; bss++)
		for (int k = 0; k < 4; k++) {
			uint32_t v;
			if (mt0_rr_chk(d, MT_SKEY_MODE(bss), &v))
				continue;
			v &= ~(MT_SKEY_MODE_MASK << MT_SKEY_MODE_SHIFT(bss, k));
			mt0_wr(d, MT_SKEY_MODE(bss), v);
			mt0_wr_copy(d, MT_SKEY(bss, k), zero32, 32);
		}
	for (int i = 0; i < 256; i++) {
		mt0_wr(d, MT_WCID_ATTR(i), 0);
		if (i < 128)
			mt0_wr_copy(d, MT_WCID_ADDR(i), zero32, 8);
	}
}

/* mt76x02_mac_setaddr. An invalid EFUSE address (multicast, or all zero)
 * gets a random locally administered one, as the kernel's eth_random_addr. */
static void mac_setaddr(struct mt76x0u_dev *d)
{
	uint8_t *a = d->macaddr;
	static const uint8_t zero6[6] = { 0 };

	if ((a[0] & 0x01) || !memcmp(a, zero6, 6)) {
		std::random_device rd;
		for (int i = 0; i < 6; i++)
			a[i] = (uint8_t)rd();
		a[0] = (uint8_t)((a[0] & 0xfe) | 0x02);
		LOG("invalid MAC address in EFUSE, using random "
		    "%02x:%02x:%02x:%02x:%02x:%02x", a[0], a[1], a[2], a[3], a[4], a[5]);
	}
	const uint32_t dw0 = (uint32_t)a[0] | ((uint32_t)a[1] << 8) |
	                     ((uint32_t)a[2] << 16) | ((uint32_t)a[3] << 24);
	const uint32_t dw1 = (uint32_t)a[4] | ((uint32_t)a[5] << 8);

	mt0_wr(d, MT_MAC_ADDR_DW0, dw0);
	mt0_wr(d, MT_MAC_ADDR_DW1, dw1 | FIELD_PREP(MT_MAC_ADDR_DW1_U2ME_MASK, 0xff));
	mt0_wr(d, MT_MAC_BSSID_DW0, dw0);
	mt0_wr(d, MT_MAC_BSSID_DW1, dw1 | FIELD_PREP(MT_MAC_BSSID_DW1_MBSS_MODE, 3) |
	                                MT_MAC_BSSID_DW1_MBSS_LOCAL_BIT);
	mt0_rmw(d, MT_MAC_BSSID_DW1, MT_MAC_BSSID_DW1_MBEACON_N,
	        FIELD_PREP(MT_MAC_BSSID_DW1_MBEACON_N, 7));
	/* mt76x02_mac_set_bssid(i, null) for i < 16: the index is masked to 3
	 * bits, so APC slots 0..7 (as in mt7612u's mac_setaddr). */
	for (int i = 0; i < 16; i++) {
		mt0_wr(d, MT_MAC_APC_BSSID_L(i & 7), 0);
		mt0_rmw(d, MT_MAC_APC_BSSID_H(i & 7), MT_MAC_APC_BSSID_H_ADDR, 0);
	}
}

/* mt76x0_init_hardware */
static int init_hardware(struct mt76x0u_dev *d)
{
	if (!mt0_poll(d, MT_WPDMA_GLO_CFG,
	              MT_WPDMA_GLO_CFG_TX_DMA_BUSY | MT_WPDMA_GLO_CFG_RX_DMA_BUSY,
	              0, 1000)) {
		ERR("WPDMA still busy");
		return -1;
	}
	/* Wait for ASIC ready after FW load. */
	if (!mt0_wait_for_mac(d)) {
		ERR("MAC did not come up after firmware load");
		return -1;
	}
	reset_csr_bbp(d);
	if (mt0_mcu_function_select(d, Q_SELECT, 1)) {
		ERR("Q_SELECT failed");
		return -1;
	}
	if (init_mac_registers(d))
		return -1;
	if (!mt0_poll(d, MT_MAC_STATUS, MT_MAC_STATUS_TX | MT_MAC_STATUS_RX, 0, 100000)) {
		ERR("MAC TX/RX not idle");
		return -1;
	}
	if (init_bbp(d))
		return -1;
	d->rxfilter = mt0_rr(d, MT_RX_FILTR_CFG);
	wcid_and_key_clear(d);
	if (mt0_eeprom_init(d))
		return -1;
	mac_setaddr(d);
	return mt0_phy_init(d);
}

int mt0_bring_up(struct mt76x0u_dev *d, const char *fw_dir, int force_fw)
{
	int uploaded = 0;

	d->chainmask = 0x101;   /* mt76x02_init_device: !is_mt76x2 -> 1T1R */
	d->mcu_running = 0;
	d->q_selected = 0;

	/* mt76x0u_probe */
	mt0_chip_onoff(d, 0, 0);
	if (!mt0_wait_for_mac(d)) {
		ERR("MAC did not come up after chip off");
		return -1;
	}

	/* mt76x0u_init_hardware(reset = true) */
	mt0_chip_onoff(d, 1, 1);
	if (!mt0_wait_for_mac(d)) {
		ERR("MAC did not come up after chip on");
		return -1;
	}
	if (mt76x0u_load_firmware(d, fw_dir, force_fw, &uploaded))
		return -1;
	if (!uploaded)
		warm_dma_reset(d);
	init_usb_dma(d);

	mt0_io_clear(d);
	if (init_hardware(d))
		return -1;
	mt0_rmw(d, MT_US_CYC_CFG, MT_US_CYC_CNT, FIELD_PREP(MT_US_CYC_CNT, 0x1e));
	mt0_wr(d, MT_TXOP_CTRL_CFG, FIELD_PREP(MT_TXOP_TRUN_EN, 0x3f) |
	                            FIELD_PREP(MT_TXOP_EXT_CCA_DLY, 0x58));
	if (mt0_io_errors(d)) {
		ERR("bring-up: %u register transfer(s) failed", mt0_io_errors(d));
		return -1;
	}
	LOG("hardware initialised (%s firmware)", uploaded ? "uploaded" : "running");
	return 0;
}

int mt76x0u_bring_up(struct mt76x0u_dev *d, const char *fw_dir, int force_fw)
{
	return d ? mt0_bring_up(d, fw_dir, force_fw) : -1;
}

int mt76x0u_bands(const struct mt76x0u_dev *d, int *has_2ghz, int *has_5ghz)
{
	if (!d || !d->eeprom_loaded)
		return -1;
	if (has_2ghz) *has_2ghz = d->has_2ghz;
	if (has_5ghz) *has_5ghz = d->has_5ghz;
	return 0;
}
