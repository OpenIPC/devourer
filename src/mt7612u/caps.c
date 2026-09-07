/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/* TSF access and the static capability descriptor. */
#include <string.h>
#include "internal.h"

/*
 * DW0 is the LOW word.
 *
 * mt76 assembles it the other way round, in mt76x02u_restart_pre_tbtt_timer()
 * (mt76x02_usb_core.c:155-158):
 *
 *     dw0 = mt76_rr(dev, MT_TSF_TIMER_DW0);
 *     dw1 = mt76_rr(dev, MT_TSF_TIMER_DW1);
 *     tsf = (u64)dw0 << 32 | dw1;
 *     dev_dbg(dev->mt76.dev, "TSF: %llu us TBTT %u us\n", tsf, tbtt);
 *
 * `tsf` there is consumed only by the dev_dbg() on the next line, so the order
 * is never exercised and the mistake has survived upstream. Copying it here
 * produced a clock that advanced by 8.6e14 "us" per 200 ms; measured,
 * (DW1 << 32) | DW0 gives 200159 us over a 200000 us sleep. The `caps` gate
 * prints both orders against a known sleep so the claim is re-checkable on
 * any sample.
 */
uint64_t mt7612u_read_tsf(struct mt7612u_dev *d)
{
	uint32_t dw0 = mt_rr(d, MT_TSF_TIMER_DW0);
	uint32_t dw1 = mt_rr(d, MT_TSF_TIMER_DW1);

	return ((uint64_t)dw1 << 32) | dw0;
}

void mt7612u_write_tsf(struct mt7612u_dev *d, uint64_t tsf)
{
	mt_wr(d, MT_TSF_TIMER_DW0, (uint32_t)tsf);
	mt_wr(d, MT_TSF_TIMER_DW1, (uint32_t)(tsf >> 32));
}

void mt7612u_get_caps(const struct mt7612u_dev *d, struct mt7612u_caps *c)
{
	memset(c, 0, sizeof *c);
	c->chip_name = "MT7612U";
	c->rev = d->rev;
	c->nss_rx = c->nss_tx = (uint8_t)((d->chainmask & 0xf) > 1 ? 2 : 1);
	/* 20, 40 and 80 MHz. Note the mask is per width, not a ceiling: the
	 * widths above 20 are 5 GHz-only on this backend, because a bare
	 * control-channel number cannot name the secondary side in 2.4 GHz
	 * except for channels 4-11. mt7612u_set_channel() refuses the rest. */
	c->bw_mask = 0x7;               /* 20, 40 and 80 MHz */
	/* TX is this backend's buffer, RX is the MAC's MT_MAX_LEN_CFG ceiling
	 * minus the FCS. See the struct for how each was measured. */
	c->max_mpdu_tx = MT_TX_BUF_MAX - 32;
	c->max_mpdu_rx = d->max_mpdu_rx;
	c->band_5g_min_mhz = 5180; c->band_5g_max_mhz = 5825;
	c->band_2g_min_mhz = 2412; c->band_2g_max_mhz = 2484;
	c->ampdu_tx = 1;
	c->per_chain_rssi = 1;
	c->narrowband = 0;              /* MT_RATE_BW has no 5/10 MHz encoding */
	c->fast_retune = 0;             /* measured 48 ms even with calibration skipped */
}

/*
 * Hardware ACK responder.
 *
 * On this MAC the immediate-response engine answers frames whose address 1
 * matches MT_MAC_ADDR_DW0/DW1, gated by MT_AUTO_RSP_EN. There is no separate
 * "responder address" register as on the Realtek parts, so arming means
 * retargeting the port identity - and, following devourer's own finding that
 * closing the gate alone does not stop a die that matches on identity, the
 * clear path moves the identity back rather than only clearing the gate.
 */
int mt7612u_set_ack_responder(struct mt7612u_dev *d, const uint8_t mac[6])
{
	uint32_t dw0, rb;

	if (!mac || (mac[0] & 0x01)) {
		ERR("ack responder address must be unicast");
		return -1;
	}

	if (!d->ack_saved) {
		memcpy(d->ack_saved_mac, d->macaddr, 6);
		d->ack_saved = 1;
	}

	dw0 = (uint32_t)mac[0] | ((uint32_t)mac[1] << 8) |
	      ((uint32_t)mac[2] << 16) | ((uint32_t)mac[3] << 24);
	mt_wr(d, MT_MAC_ADDR_DW0, dw0);
	mt_wr(d, MT_MAC_ADDR_DW1, (uint32_t)mac[4] | ((uint32_t)mac[5] << 8) |
	      FIELD_PREP(MT_MAC_ADDR_DW1_U2ME_MASK, 0xff));
	mt_set(d, MT_AUTO_RSP_CFG, MT_AUTO_RSP_EN);

	/* Verify the arm. The U2ME byte of DW1 is write-only on this silicon,
	 * so only DW0 and the low half of DW1 can be read back. */
	rb = mt_rr(d, MT_MAC_ADDR_DW0);
	if (rb != dw0) {
		ERR("ack responder arm not verified: MT_MAC_ADDR_DW0 %08x != %08x",
		    rb, dw0);
		return -1;
	}
	if (!(mt_rr(d, MT_AUTO_RSP_CFG) & MT_AUTO_RSP_EN)) {
		ERR("ack responder arm not verified: MT_AUTO_RSP_EN clear");
		return -1;
	}
	return 0;
}

void mt7612u_clear_ack_responder(struct mt7612u_dev *d)
{
	if (!d->ack_saved)
		return;

	/* Move the identity off the responder address first: on a MAC that
	 * matches on address 1, clearing the gate alone leaves it answering
	 * for whatever address is still programmed.
	 *
	 * MT_AUTO_RSP_EN is deliberately NOT cleared here. mt_init_hardware()
	 * writes MT_AUTO_RSP_CFG = 0x13 (init.c), and MT_AUTO_RSP_EN is BIT(0),
	 * so the gate is already on before any caller arms a responder - the
	 * mt_set() in mt7612u_set_ack_responder() is a no-op on it. Clearing it
	 * here would leave the device in a state its own init never produces;
	 * restoring the address is what actually stops it answering. */
	{
		const uint8_t *a = d->ack_saved_mac;

		mt_wr(d, MT_MAC_ADDR_DW0, (uint32_t)a[0] | ((uint32_t)a[1] << 8) |
		      ((uint32_t)a[2] << 16) | ((uint32_t)a[3] << 24));
		mt_wr(d, MT_MAC_ADDR_DW1, (uint32_t)a[4] | ((uint32_t)a[5] << 8) |
		      FIELD_PREP(MT_MAC_ADDR_DW1_U2ME_MASK, 0xff));
	}
	d->ack_saved = 0;
}
