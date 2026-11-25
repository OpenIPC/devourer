/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MT76x0U MAC start/stop: mt76x02u_mac_start (mt76x02_usb_core.c,
 * BSD-3-Clause-Clear), mt76x0_mac_stop (mt76x0/init.c, GPL-2.0-only;
 * Copyright (C) 2014 Felix Fietkau, (C) 2015 Jakub Kicinski, (C) 2018
 * Stanislaw Gruszka), and mt76x0u_start's power-on calibration (mt76x0/usb.c).
 * reference/mt76 @ be5ce79.
 *
 * The ordering rule is the MT7612U's, measured there: the receiver must never
 * run with nothing draining EP 0x84 - on that part it wedges the chip below
 * the USB level, recoverable only by a replug. Same DMA family here, so the
 * same rule until a bench says otherwise: ring first, then MAC RX; on the
 * way down, MAC RX off, then the ring.
 */
#include "internal.h"

int mt0_mac_start(struct mt76x0u_dev *d)
{
	if (!(d->a && d->a->rx_active) ||
	    d->ep4_owner.load(std::memory_order_acquire) != MT_EP4_RING) {
		ERR("MAC start refused: no RX ring draining EP 0x84 - start it first");
		return -1;
	}

	/* mt76x02_mac_reset_counters: the RX/TX statistics are read-and-clear. */
	(void)mt0_rr(d, MT_RX_STAT_0);
	(void)mt0_rr(d, MT_RX_STAT_1);
	(void)mt0_rr(d, MT_RX_STAT_2);
	(void)mt0_rr(d, MT_TX_STA_0);
	(void)mt0_rr(d, MT_TX_STA_1);
	(void)mt0_rr(d, MT_TX_STA_2);
	for (int i = 0; i < 16; i++)
		(void)mt0_rr(d, MT_TX_AGG_CNT(i));
	for (int i = 0; i < 16; i++)
		(void)mt0_rr(d, MT_TX_STAT_FIFO);

	mt0_wr(d, MT_MAC_SYS_CTRL, MT_MAC_SYS_CTRL_ENABLE_TX);
	if (!mt0_poll(d, MT_WPDMA_GLO_CFG,
	              MT_WPDMA_GLO_CFG_TX_DMA_BUSY | MT_WPDMA_GLO_CFG_RX_DMA_BUSY,
	              0, 200000)) {
		ERR("MAC start: WPDMA stayed busy");
		return -1;
	}
	mt0_wr(d, MT_RX_FILTR_CFG, d->rxfilter);
	/* TX and RX together, as the kernel: wifit3 notes the chip misbehaves
	 * with only one set. The TX engine idles - nothing is ever queued. */
	mt0_wr(d, MT_MAC_SYS_CTRL, MT_MAC_SYS_CTRL_ENABLE_TX | MT_MAC_SYS_CTRL_ENABLE_RX);
	d->mac_started = 1;
	/* mt76x02_wait_for_wpdma(dev, 50): 50 us, i.e. one read over USB. */
	if (!mt0_poll(d, MT_WPDMA_GLO_CFG,
	              MT_WPDMA_GLO_CFG_TX_DMA_BUSY | MT_WPDMA_GLO_CFG_RX_DMA_BUSY,
	              0, 50))
		WARN("MAC start: WPDMA busy right after enabling RX (the kernel "
		     "fails here; a USB read takes longer than its 50 us budget)");
	return 0;
}

void mt0_mac_rx_disable(struct mt76x0u_dev *d)
{
	mt0_clear(d, MT_MAC_SYS_CTRL, MT_MAC_SYS_CTRL_ENABLE_RX);
}

/* mt76x0_mac_stop */
int mt0_mac_stop(struct mt76x0u_dev *d)
{
	int ok = 0;

	mt0_clear(d, MT_TXOP_CTRL_CFG, MT_TXOP_ED_CCA_EN);

	/* Page count on TxQ */
	for (int i = 0; i < 200; i++) {
		if (!mt0_rr(d, 0x0438) && !(mt0_rr(d, 0x0a30) & 0x000000ff) &&
		    !(mt0_rr(d, 0x0a34) & 0x00ff00ff))
			break;
		mt0_usleep(10000);
	}
	if (!mt0_poll(d, MT_MAC_STATUS, MT_MAC_STATUS_TX, 0, 1000))
		WARN("MAC TX did not stop");

	mt0_clear(d, MT_MAC_SYS_CTRL, MT_MAC_SYS_CTRL_ENABLE_RX | MT_MAC_SYS_CTRL_ENABLE_TX);
	d->mac_started = 0;

	/* Page count on RxQ */
	for (int i = 0; i < 200; i++) {
		if (!(mt0_rr(d, MT_RXQ_STA) & 0x00ff0000) && !mt0_rr(d, 0x0a30) &&
		    !mt0_rr(d, 0x0a34)) {
			if (ok++ > 5)
				break;
			continue;
		}
		mt0_usleep(1000);
	}
	if (!mt0_poll(d, MT_MAC_STATUS, MT_MAC_STATUS_RX, 0, 1000))
		WARN("MAC RX did not stop");
	return 0;
}

/* --- public ---------------------------------------------------------------- */

int mt76x0u_start(struct mt76x0u_dev *d, int accept_control)
{
	if (!d)
		return -1;
	if (!d->shape_valid) {
		ERR("start before a channel is set");
		return -1;
	}
	mt0_io_clear(d);
	if (mt0_mac_start(d))
		return -1;
	/* After the start, which wrote the init filter back (the MT7612U's rule
	 * 2: a filter written before the start is simply overwritten). */
	mt0_set_monitor_filter(d, accept_control);
	/* mt76x0u_start: the power-on calibration, after the MAC is up. */
	if (mt0_phy_calibrate(d, 1))
		WARN("power-on calibration reported a failure");
	if (mt0_io_errors(d)) {
		ERR("start: %u register transfer(s) failed", mt0_io_errors(d));
		return -1;
	}
	return 0;
}

int mt76x0u_rx_quiesce(struct mt76x0u_dev *d)
{
	if (!d)
		return -1;
	mt0_mac_rx_disable(d);
	return 0;
}

int mt76x0u_stop(struct mt76x0u_dev *d)
{
	if (!d)
		return -1;
	/* The kernel checks the USB TX DMA first (mt76x0u_stop). */
	if (!mt0_poll(d, MT_USB_DMA_CFG, MT_USB_DMA_CFG_TX_BUSY, 0, 1000))
		WARN("TX DMA did not stop");
	return mt0_mac_stop(d);
}
