/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * TX path: build the 20-byte TXWI and the 4-byte TXINFO, then submit one bulk
 * transfer. That is the whole of it - no firmware involvement, no per-station
 * rate table. Ported from mt76/mt76x02_usb_core.c and mt76x02_mac.c.
 *
 * Wire layout (INVESTIGATION.md §7):
 *   [TXINFO 4B][TXWI 20B][802.11 hdr][pad][payload][pad to 4][4B zero]
 */
#include <string.h>
#include "internal.h"


static void put_le16(uint8_t *p, uint16_t v) { p[0] = v & 0xff; p[1] = v >> 8; }
static void put_le32(uint8_t *p, uint32_t v)
{
	p[0] = v & 0xff; p[1] = (v >> 8) & 0xff;
	p[2] = (v >> 16) & 0xff; p[3] = (v >> 24) & 0xff;
}

/* The 16-bit per-packet rate word. Everything the PHY needs is in here, and
 * because MT_TXWI_FLAGS_TX_RATE_LUT is never set the MAC uses it verbatim. */
uint16_t mt_tx_rate_word(const struct mt7612u_tx_rate *r)
{
	uint32_t idx, word;

	switch (r->phy) {
	case MT7612U_PHY_VHT:
		/* index is NSS-1 in bits 5:4, MCS in bits 3:0 */
		idx = FIELD_PREP(MT_RATE_VHT_IDX, r->mcs) |
		      FIELD_PREP(MT_RATE_VHT_NSS, (uint32_t)(r->nss ? r->nss - 1 : 0));
		break;
	case MT7612U_PHY_HT:
	case MT7612U_PHY_HT_GF:
		/* HT folds NSS into the MCS number: nss = 1 + (idx >> 3) */
		idx = r->mcs;
		break;
	default:
		idx = r->mcs;
		break;
	}

	word = FIELD_PREP(MT_RATE_INDEX, idx) |
	       FIELD_PREP(MT_RATE_PHY, (uint32_t)r->phy) |
	       FIELD_PREP(MT_RATE_BW, (uint32_t)r->bw);
	if (r->sgi)  word |= MT_RATE_SGI;
	if (r->ldpc) word |= MT_RATE_LDPC;
	/* mt76 only sets STBC at a single spatial stream, and so do we. */
	if (r->stbc && r->nss <= 1) word |= MT_RATE_STBC;

	return (uint16_t)word;
}

/*
 * Low-level submit. wcid selects a hardware rate-LUT entry (0xff = none) and
 * use_rate_lut sets MT_TXWI_FLAGS_TX_RATE_LUT, which is the bit mt76 defines
 * and never sets. Exposed so the Gate-G control can make the LUT and the
 * descriptor disagree and see which one airs.
 */
/*
 * ieee80211_hdrlen(), ported from the kernel's net/mac80211 helper - the same
 * function mt76 reaches through ieee80211_get_hdrlen_from_skb() on both the
 * TX (mt76_insert_hdr_pad) and RX (mt76x02_remove_hdr_pad) sides.
 *
 * Enumerating "the control frames that are 16 bytes" gets this wrong: the
 * default for control frames is 16, and only CTS and ACK are 10. Listing
 * RTS and PS-Poll as the 16-byte cases leaves BlockAckReq, BlockAck and both
 * CF-End subtypes at 10, which inserts the L2 pad *inside* the frame.
 */
int mt_hdrlen_from_fc(const uint8_t *f)
{
	unsigned fc = (unsigned)f[0] | ((unsigned)f[1] << 8);
	unsigned type = (fc >> 2) & 3;
	int len = 24;

	if (type == 2) {                        /* data */
		if ((fc & 0x0300) == 0x0300)
			len = 30;               /* 4-address */
		if (fc & 0x0080) {              /* QoS subtype bit */
			len += 2;               /* QoS Control */
			if (fc & 0x8000)        /* Order -> HT Control */
				len += 4;
		}
		return len;
	}
	if (type == 0)                          /* management */
		return (fc & 0x8000) ? 28 : 24; /* Order -> HT Control */
	if (type == 1)                          /* control */
		return ((fc & 0x00e0) == 0x00c0) ? 10 : 16;  /* CTS, ACK */
	return len;
}

/* mt76x02_mac_wcid_setup(): a station-table entry, needed before the
 * hardware will treat frames as belonging to a peer. */
void mt_wcid_setup(struct mt7612u_dev *d, uint8_t idx, const uint8_t *mac)
{
	uint8_t addr[8] = { 0 };

	mt_wr(d, MT_WCID_ATTR(idx), 0);
	if (mac) memcpy(addr, mac, 6);
	if (idx < 128) mt_wr_copy(d, MT_WCID_ADDR(idx), addr, 8);
}

/*
 * Build one [TXINFO][TXWI][802.11][pad to 4] block into `buf`, returning its
 * length. `next_vld` sets MT_TXD_INFO_NEXT_VLD, which tells the TXDMA another
 * block follows in the same bulk transfer - the hook USB aggregation hangs on.
 * mt76 never sets this bit; the packing in send_packets is the one thing here
 * that is not a port. `trailer` appends the 4-byte zero terminator, which only
 * the last block in a transfer carries.
 */
int mt_tx_build(struct mt7612u_dev *d, uint8_t *buf, size_t bufsz,
                const void *frame, size_t len,
                const struct mt7612u_tx_rate *rate, uint8_t wcid, unsigned opts,
                int next_vld, int trailer)
{
	uint8_t *txwi = buf + 4;
	uint32_t info;
	int hdrlen, hdr_pad = 0, body, padded, pad, total, rc, n = 0;
	const uint8_t *f = frame;

	if (len < 10 || len + 32 > bufsz) { ERR("bad frame length %zu", len); return -1; }

	/* mt76_insert_hdr_pad(): 2 bytes after the header when the 802.11
	 * header is not a multiple of 4, so the body stays 4-aligned. The MAC
	 * strips it. A 3-address data header is 24 bytes, so normally none. */
	hdrlen = mt_hdrlen_from_fc(f);
	if (hdrlen > (int)len) hdrlen = (int)len;
	if (hdrlen % 4) hdr_pad = 2;

	memset(txwi, 0, MT_TXWI_LEN);
	{
		uint16_t fl = (opts & MT_TXOPT_RATE_LUT) ? MT_TXWI_FLAGS_TX_RATE_LUT : 0;

		if (opts & MT_TXOPT_AMPDU)
			fl |= MT_TXWI_FLAGS_AMPDU |
			      FIELD_PREP(MT_TXWI_FLAGS_MPDU_DENSITY, 4);
		put_le16(txwi + 0, fl);
	}
	/* "A frame may narrow below the channel but never widen it" was only
	 * ever a comment; enforce it here, where the tuned width is known. The
	 * BBP is configured for one width at a time, so a rate word asking for
	 * a wider frame than the channel was tuned to does not give a wider
	 * frame - it gives an unspecified one. Clamping rather than dropping
	 * keeps a caller that set its TX mode before its channel on the air,
	 * which is an ordering the C++ layer allows. Warn once: per frame would
	 * flood a TX loop. */
	if (d->chan && rate->bw > d->bw) {
		struct mt7612u_tx_rate narrowed = *rate;

		if (!d->bw_clamp_warned) {
			d->bw_clamp_warned = 1;
			WARN("rate asks for bw %u on a bw %u channel; narrowing "
			    "to the channel width (further occurrences silent)",
			    rate->bw, d->bw);
		}
		narrowed.bw = d->bw;
		put_le16(txwi + 2, mt_tx_rate_word(&narrowed));
	} else {
		put_le16(txwi + 2, mt_tx_rate_word(rate));      /* rate */
	}
	/* ack_ctl bit0 REQ: set it only when an ACK is wanted. Leaving it
	 * clear is how a frame becomes no-ACK, per packet. */
	txwi[4] = rate->no_ack ? 0 : MT_TXWI_ACK_CTL_REQ;
	if (opts & MT_TXOPT_AMPDU)
		txwi[4] |= FIELD_PREP(MT_TXWI_ACK_CTL_BA_WINDOW, 63);
	txwi[5] = wcid;                                         /* 0xff = none */
	put_le16(txwi + 6, (uint16_t)len);                      /* len_ctl: true MPDU len */
	put_le32(txwi + 8, 0);                                  /* iv  */
	put_le32(txwi + 12, 0);                                 /* eiv */
	txwi[16] = 0;                                           /* aid */
	/* nstreams > 1 and rev >= E4 -> 0x13, as mt76x02_mac_write_txwi(). */
	txwi[17] = ((d->chainmask & 0xf) > 1) ? 0x13 : 0;
	/* mt76x02_mac_write_txwi(): the 4-bit trim is derived from the per-rate
	 * ceiling and the chain target power. rate->power_adj overrides it when
	 * the caller wants explicit control. */
	{
		int8_t adj = rate->power_adj;

		if (!adj) {
			int8_t max_adj = mt_tx_get_max_txpwr_adj(d, rate);
			adj = mt_tx_get_txpwr_adj(d, d->txpower_conf, max_adj);
		}
		txwi[18] = FIELD_PREP(MT_TX_PWR_ADJ, (uint32_t)(adj & 0xf));
	}
	txwi[19] = 0;                                           /* pktid */

	/* frame, with the header pad inserted if needed */
	memcpy(buf + 4 + MT_TXWI_LEN, f, (size_t)hdrlen);
	if (hdr_pad)
		memset(buf + 4 + MT_TXWI_LEN + hdrlen, 0, 2);
	body = (int)len - hdrlen;
	if (body > 0)
		memcpy(buf + 4 + MT_TXWI_LEN + hdrlen + hdr_pad, f + hdrlen, (size_t)body);

	padded = (int)len + hdr_pad;

	info = FIELD_PREP(MT_TXD_INFO_LEN, (uint32_t)((MT_TXWI_LEN + padded + 3) & ~3)) |
	       FIELD_PREP(MT_TXD_INFO_DPORT, WLAN_PORT) |
	       FIELD_PREP(MT_TXD_INFO_QSEL,
	                  (opts & MT_TXOPT_QSEL_MGMT) ? MT_QSEL_MGMT : MT_QSEL_EDCA) |
	       MT_TXD_INFO_80211 |
	       MT_TXD_INFO_WIV;   /* no hardware IV insertion - unencrypted */
	if (next_vld)
		info |= MT_TXD_INFO_NEXT_VLD;
	put_le32(buf, info);

	{
		int upto = 4 + MT_TXWI_LEN + padded;

		pad = ((upto + 3) & ~3) - upto + (trailer ? 4 : 0);
		memset(buf + upto, 0, (size_t)pad);
		total = upto + pad;
	}
	(void)rc; (void)n;
	return total;
}

int mt_tx_raw(struct mt7612u_dev *d, const void *frame, size_t len,
              const struct mt7612u_tx_rate *rate, uint8_t wcid, unsigned opts)
{
	uint8_t buf[MT_TX_BUF_MAX];
	int total = mt_tx_build(d, buf, sizeof buf, frame, len, rate, wcid, opts, 0, 1);
	int rc, n = 0;

	if (total < 0) return -1;

	/* Async pool when one is running - it only blocks when every slot is in
	 * flight. Otherwise fall back to a synchronous transfer. */
	if (d->a)
		return mt_async_tx_submit(d, buf, total);

	rc = mt_bulk(d, MT_EP_OUT_AC_BE, buf, total, &n, 500);
	if (rc) { ERR("tx bulk out: %s", libusb_error_name(rc)); return -1; }
	if (n != total) { ERR("tx short write %d/%d", n, total); return -1; }
	return 0;
}

int mt7612u_tx(struct mt7612u_dev *d, const void *frame, size_t len,
               const struct mt7612u_tx_rate *rate)
{
	return mt_tx_raw(d, frame, len, rate, 0xff, 0);
}
