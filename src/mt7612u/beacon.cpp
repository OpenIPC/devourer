/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * Static reserved-page beacon.
 *
 * The MT76 MAC auto-transmits a beacon written to MT_BEACON_BASE at every
 * TBTT, filling the TSF timestamp and the 802.11 sequence number in hardware
 * (MT_TXWI_FLAGS_TS + MT_TXWI_ACK_CTL_NSEQ, requested via MT_TXOPT_BEACON).
 * So an always-on AP needs no host involvement per beacon: no pre-TBTT timer,
 * no worker thread. That is the one USB-userspace limitation the AP design
 * engineers out - see docs/mt7612u-ap-mode.md.
 *
 * Dynamic beacon content (a live TIM bitmap for power-saving clients) is NOT
 * covered here; it would need the pre-TBTT machinery mt76 runs on PCIe/USB.
 *
 * Ported from mt76/mt76x02_beacon.c and mt76x02_usb_core.c @ be5ce79.
 * Copyright (C) 2016 Felix Fietkau, (C) 2018 Lorenzo Bianconi / Stanislaw Gruszka.
 */
#include <cstring>
#include "internal.h"

/* mt76x02u: 5 USB beacon slots, each (8192 / 5) & ~63 = 1600 bytes. The 8 kB
 * reserved page is shared with PS-buffered frames upstream; we use slot 0. */
#define MT_BCN_NSLOTS    5
#define MT_BCN_SLOT_SIZE ((8192 / MT_BCN_NSLOTS) & ~63)

/* mt76x02_set_beacon_offsets(): each slot's (byte offset / 64) is packed into
 * MT_BCN_OFFSET, four slots to a 32-bit register. */
static void mt_beacon_set_offsets(struct mt7612u_dev *d)
{
	uint32_t regs[4] = { 0 };
	int i;

	for (i = 0; i < MT_BCN_NSLOTS; i++) {
		uint32_t val = (uint32_t)i * MT_BCN_SLOT_SIZE;

		regs[i / 4] |= (val / 64) << (8 * (i % 4));
	}
	for (i = 0; i < 4; i++)
		mt_wr(d, MT_BCN_OFFSET(i), regs[i]);
}

/*
 * mt76x02_init_beacon_config(): quiet the beacon engine, select sync mode,
 * suppress every beacon slot while the page is being set up, and lay out the
 * slot offsets. Run once before the first beacon is written. The address
 * programming (BSSID, MBSS mode, per-slot beacon count) is done by
 * mac_setaddr() at init. BCN_BYPASS_MASK=0xffff suppresses all slots here;
 * mt_beacon_write() clears the bit for the slot it loads so that one airs.
 */
void mt_beacon_init(struct mt7612u_dev *d)
{
	mt_clear(d, MT_BEACON_TIME_CFG,
	         MT_BEACON_TIME_CFG_TIMER_EN | MT_BEACON_TIME_CFG_TBTT_EN |
	         MT_BEACON_TIME_CFG_BEACON_TX);
	mt_set(d, MT_BEACON_TIME_CFG, MT_BEACON_TIME_CFG_SYNC_MODE);
	mt_wr(d, MT_BCN_BYPASS_MASK, 0xffff);
	mt_beacon_set_offsets(d);
}

/*
 * mt76x02_mac_set_beacon(): write [TXWI][beacon MPDU] into slot 0.
 *
 * mt_tx_build() emits [TXINFO 4][TXWI 20][802.11][pad], which is the shape the
 * TX queue wants; the reserved page wants no TXINFO and no trailing zero word,
 * so skip the first 4 bytes and copy from the TXWI onward. A beacon header is
 * 24 bytes (4-aligned), so mt_tx_build() inserts no interior L2 pad and the
 * copied region is exactly [TXWI][MPDU] rounded up to a word.
 */
int mt_beacon_write(struct mt7612u_dev *d, const void *frame, size_t len,
                    const struct mt7612u_tx_rate *rate)
{
	uint8_t buf[MT_BCN_SLOT_SIZE];
	int total;

	if (len + MT_TXWI_LEN > MT_BCN_SLOT_SIZE) {
		ERR("beacon %zu B + TXWI exceeds the %d B slot", len,
		    (int)MT_BCN_SLOT_SIZE);
		return -1;
	}
	total = mt_tx_build(d, buf, sizeof buf, frame, len, rate, 0xff,
	                    MT_TXOPT_BEACON, 0, 0);
	if (total < 0)
		return -1;

	mt_wr_copy(d, MT_BEACON_BASE, buf + MT_DMA_HDR_LEN,
	           total - MT_DMA_HDR_LEN);

	/*
	 * Unsuppress the slot just written. BCN_BYPASS_MASK is inverted: a set
	 * bit suppresses that slot (mt_beacon_init() set all 16). mt76 clears the
	 * low N bits down from bit 7 for N written beacons - 0xff00 | ~(0xff00 >>
	 * beacon_data_count) in mt76x02u_pre_tbtt_work() - and the static path
	 * writes exactly one (slot 0), so N = 1. Without this the beacon never
	 * airs even though the TSF and beacon timer run.
	 */
	mt_wr(d, MT_BCN_BYPASS_MASK, 0xff00u | ~(0xff00u >> 1));
	return 0;
}

/*
 * mt76x02_mac_set_bssid(): the per-BSS address the MAC matches receptions
 * against. mac_setaddr() zeroes all eight APC slots at init, which is right for
 * an injector; an AP must publish its own BSSID in the slot its MBSS index
 * selects (0 for a single BSS) or the MAC matches nothing for the BSS - a
 * station's auth is then neither accepted nor auto-ACKed, and it retries
 * forever. Called by the AP path; the injector never needs it.
 */
int mt_ap_set_bssid(struct mt7612u_dev *d, uint8_t idx, const uint8_t *addr)
{
	uint32_t lo = (uint32_t)addr[0] | ((uint32_t)addr[1] << 8) |
	              ((uint32_t)addr[2] << 16) | ((uint32_t)addr[3] << 24);
	uint32_t hi = (uint32_t)addr[4] | ((uint32_t)addr[5] << 8);

	idx &= 7;
	if (mt_wr_chk(d, MT_MAC_APC_BSSID_L(idx), lo))
		return -1;
	/* mt_rmw() skips the write entirely when its read half fails, so an
	 * unchecked call can leave bytes 4-5 zero - a half-programmed BSSID that
	 * matches nothing while the L half still reads back correct. mt76 returns
	 * void here because MMIO cannot fail; USB can. */
	return mt_rmw(d, MT_MAC_APC_BSSID_H(idx), MT_MAC_APC_BSSID_H_ADDR, hi);
}

/*
 * mt76x02_mac_set_beacon_enable(), static path. No pre-TBTT timer: the MAC
 * transmits the reserved-page beacon on its own once BEACON_TX|TBTT_EN|TIMER_EN
 * are set. interval_tu is the beacon interval in TU (1024 us); the register
 * counts in 1/16 TU, so it is shifted left by 4 (mt76x02_bss_info_changed).
 */
int mt_beacon_set_enable(struct mt7612u_dev *d, int on, unsigned interval_tu)
{
	const uint32_t bits = MT_BEACON_TIME_CFG_BEACON_TX |
	                      MT_BEACON_TIME_CFG_TBTT_EN |
	                      MT_BEACON_TIME_CFG_TIMER_EN;

	if (on) {
		/* INTVAL is 16 bits of 1/16-TU, so the interval caps at 4095 TU. */
		if (interval_tu == 0 || interval_tu > 0xffffu / 16) {
			ERR("beacon interval %u TU out of range (1..4095)", interval_tu);
			return -1;
		}
		mt_rmw(d, MT_BEACON_TIME_CFG, MT_BEACON_TIME_CFG_INTVAL,
		       FIELD_PREP(MT_BEACON_TIME_CFG_INTVAL, interval_tu << 4));
		mt_set(d, MT_BEACON_TIME_CFG, bits);
	} else {
		mt_clear(d, MT_BEACON_TIME_CFG, bits);
	}
	return 0;
}

/* --- public ABI ---------------------------------------------------------
 *
 * The three calls devourer's IRadio beacon surface maps onto. Everything they
 * do is the sequence bringup's Stage A and Stage B gates run and that was
 * device-verified on 2026-09-08 (docs/mt7612u-ap-mode.md); this is that
 * sequence behind the public header, so a consumer does not have to reach into
 * internal.h to be an AP.
 */

/* Split a radiotap-framed buffer into rate + MPDU, exactly as
 * mt7612u_send_packet() does. A bare MPDU (no radiotap) is not an error here -
 * IRadio's contract strips the header "if present" - and takes the rate a
 * beacon wants: OFDM 6 Mbps, the basic rate every station must decode. */
static int beacon_split(const void *buf, size_t len, const uint8_t **mpdu,
                        size_t *mpdu_len, struct mt7612u_tx_rate *r)
{
	const uint8_t *p = (const uint8_t *)buf;
	int rlen;

	if (!p || len == 0) return -1;

	rlen = mt_radiotap_parse(p, len, r);
	if (rlen > 0 && (size_t)rlen < len) {
		*mpdu = p + rlen;
		*mpdu_len = len - (size_t)rlen;
	} else {
		r->phy = MT7612U_PHY_OFDM;
		r->mcs = 0;
		r->nss = 1;
		r->bw = MT7612U_BW_20;
		r->no_ack = 1;   /* a broadcast beacon is never ACKed */
		*mpdu = p;
		*mpdu_len = len;
	}
	/* addr3 lives at offset 16, so anything shorter has no BSSID to publish
	 * and is not a beacon whatever else it is. */
	if (*mpdu_len < 24) {
		ERR("beacon: %zu B is too short for an 802.11 header", *mpdu_len);
		return -1;
	}
	return 0;
}

int mt7612u_beacon_start(struct mt7612u_dev *dev, const void *buf, size_t len,
                         unsigned interval_tu)
{
	struct mt7612u_tx_rate rate;
	const uint8_t *mpdu = NULL;
	size_t mpdu_len = 0;

	if (!dev) return -1;
	if (beacon_split(buf, len, &mpdu, &mpdu_len, &rate)) return -1;

	/* Both refusals are in the header's contract. They are refusals rather
	 * than warnings because each one airs a beacon that looks perfect on a
	 * scan and then ACKs nothing - the operator debugs the RF link instead of
	 * the configuration. */
	if (dev->macaddr[0] & 0x02) {
		ERR("beacon: adapter MAC %02x:.. is locally administered; APC slot 0 "
		    "is not the slot that MAC selects (mt76 derives 1+n)",
		    dev->macaddr[0]);
		return -1;
	}
	if (memcmp(mpdu + 16, dev->macaddr, 6) != 0) {
		ERR("beacon: BSSID %02x:%02x:%02x:%02x:%02x:%02x is not the adapter's "
		    "own MAC; the MAC ACKs against MT_MAC_ADDR and this call does not "
		    "retarget it, so that BSS would answer nothing",
		    mpdu[16], mpdu[17], mpdu[18], mpdu[19], mpdu[20], mpdu[21]);
		return -1;
	}

	if (mt_ap_set_bssid(dev, 0, dev->macaddr)) return -1;
	mt_beacon_init(dev);
	if (mt_beacon_write(dev, mpdu, mpdu_len, &rate)) return -1;
	return mt_beacon_set_enable(dev, 1, interval_tu);
}

int mt7612u_beacon_update(struct mt7612u_dev *dev, const void *buf, size_t len)
{
	struct mt7612u_tx_rate rate;
	const uint8_t *mpdu = NULL;
	size_t mpdu_len = 0;

	if (!dev) return -1;
	if (beacon_split(buf, len, &mpdu, &mpdu_len, &rate)) return -1;
	/* No mt_beacon_init() and no set_enable(): the engine is already armed and
	 * re-initialising it would re-suppress every slot mid-flight. */
	return mt_beacon_write(dev, mpdu, mpdu_len, &rate);
}

int mt7612u_beacon_stop(struct mt7612u_dev *dev)
{
	if (!dev) return -1;
	return mt_beacon_set_enable(dev, 0, 0);
}
