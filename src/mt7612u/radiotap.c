/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * Minimal radiotap TX parser, so a caller can hand us the same
 * "radiotap header + 802.11 MPDU" buffer devourer's send_packet() takes
 * instead of filling a struct.
 *
 * Only the inject-relevant fields are decoded; everything else is skipped by
 * the alignment/size table, which is what makes skipping correct rather than
 * lucky. Field order, alignment and size follow the radiotap spec.
 */
#include <string.h>
#include "internal.h"

/* {align, size} per radiotap bit index. size 0 = unknown -> stop parsing. */
static const struct { uint8_t align, size; } rt_field[] = {
	{ 8, 8 }, /*  0 TSFT              */ { 1, 1 }, /*  1 FLAGS   */
	{ 1, 1 }, /*  2 RATE              */ { 2, 4 }, /*  3 CHANNEL */
	{ 2, 2 }, /*  4 FHSS              */ { 1, 1 }, /*  5 DBM_ANTSIGNAL */
	{ 1, 1 }, /*  6 DBM_ANTNOISE      */ { 2, 2 }, /*  7 LOCK_QUALITY */
	{ 2, 2 }, /*  8 TX_ATTENUATION    */ { 2, 2 }, /*  9 DB_TX_ATTENUATION */
	{ 1, 1 }, /* 10 DBM_TX_POWER      */ { 1, 1 }, /* 11 ANTENNA */
	{ 1, 1 }, /* 12 DB_ANTSIGNAL      */ { 1, 1 }, /* 13 DB_ANTNOISE */
	{ 2, 2 }, /* 14 RX_FLAGS          */ { 2, 2 }, /* 15 TX_FLAGS */
	{ 1, 1 }, /* 16 RTS_RETRIES       */ { 1, 1 }, /* 17 DATA_RETRIES */
	{ 0, 0 }, /* 18 (unused)          */ { 1, 3 }, /* 19 MCS */
	{ 4, 8 }, /* 20 AMPDU_STATUS      */ { 2, 12 },/* 21 VHT */
	{ 8, 12 },/* 22 TIMESTAMP         */ { 2, 12 },/* 23 HE */
};
#define RT_RATE 2
#define RT_DBM_TX_POWER 10
#define RT_TX_FLAGS 15
#define RT_MCS 19
#define RT_VHT 21
#define RT_TX_FLAGS_NOACK 0x0008

/* One diagnostic per process for a field we parse but cannot honour. */
static int warned_tx_power;

static uint16_t rd16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static uint32_t rd32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Legacy radiotap RATE is in 500 kbps units; the hardware wants an index. */
static int legacy_rate_index(uint8_t r500, enum mt7612u_phy *phy)
{
	switch (r500) {
	case 2:  *phy = MT7612U_PHY_CCK;  return 0;   /* 1 Mbps   */
	case 4:  *phy = MT7612U_PHY_CCK;  return 1;   /* 2        */
	case 11: *phy = MT7612U_PHY_CCK;  return 2;   /* 5.5      */
	case 22: *phy = MT7612U_PHY_CCK;  return 3;   /* 11       */
	case 12: *phy = MT7612U_PHY_OFDM; return 0;   /* 6        */
	case 18: *phy = MT7612U_PHY_OFDM; return 1;   /* 9        */
	case 24: *phy = MT7612U_PHY_OFDM; return 2;   /* 12       */
	case 36: *phy = MT7612U_PHY_OFDM; return 3;   /* 18       */
	case 48: *phy = MT7612U_PHY_OFDM; return 4;   /* 24       */
	case 72: *phy = MT7612U_PHY_OFDM; return 5;   /* 36       */
	case 96: *phy = MT7612U_PHY_OFDM; return 6;   /* 48       */
	case 108:*phy = MT7612U_PHY_OFDM; return 7;   /* 54       */
	default: *phy = MT7612U_PHY_OFDM; return 0;
	}
}

/*
 * Radiotap VHT bandwidth code -> the width the frame is actually sent at.
 *
 * The code names a channel width *and* the sub-channel within it: 2 is
 * "40 (20L)", a 20 MHz frame in the lower half of a 40 MHz channel, not a
 * 40 MHz frame. Treating 1-3 as 40 and >=4 as 80 airs a requested 20-in-40
 * at 40 MHz and a requested 20-in-80 at 80.
 *
 * This project's own ieee80211_radiotap.h names the equivalent HT codes
 * IEEE80211_RADIOTAP_MCS_BW_20L (2) and _20U (3), and the HT branch below
 * already reads them that way - the VHT branch used to disagree with it.
 */
static enum mt7612u_bw vht_bandwidth(uint8_t code)
{
	static const uint8_t width[] = {
		MT7612U_BW_20,  /*  0  20        */
		MT7612U_BW_40,  /*  1  40        */
		MT7612U_BW_20,  /*  2  40 (20L)  */
		MT7612U_BW_20,  /*  3  40 (20U)  */
		MT7612U_BW_80,  /*  4  80        */
		MT7612U_BW_40,  /*  5  80 (40L)  */
		MT7612U_BW_40,  /*  6  80 (40U)  */
		MT7612U_BW_20,  /*  7  80 (20LL) */
		MT7612U_BW_20,  /*  8  80 (20LU) */
		MT7612U_BW_20,  /*  9  80 (20UL) */
		MT7612U_BW_20,  /* 10  80 (20UU) */
	};

	if (code < sizeof width / sizeof width[0])
		return (enum mt7612u_bw)width[code];
	/* 11 and above are 160 MHz and its sub-channels. This part has no
	 * 160 MHz encoding in the rate word, and narrowing silently would be
	 * worse than saying so. */
	LOG("radiotap VHT bandwidth code %u is 160 MHz or a sub-channel of it; "
	    "unsupported, sending at 20 MHz", code);
	return MT7612U_BW_20;
}

/*
 * Parse a radiotap header into a tx_rate. Returns the header length, or 0 if
 * the buffer is not a usable radiotap header.
 */
int mt_radiotap_parse(const uint8_t *buf, size_t len, struct mt7612u_tx_rate *r)
{
	uint32_t present[8];
	unsigned n_present = 0, bit = 0;
	size_t rlen, off;

	if (!buf || len < 8 || buf[0] != 0) return 0;      /* version must be 0 */
	rlen = rd16(buf + 2);
	if (rlen < 8 || rlen > len) return 0;

	/* Walk the extended present bitmaps. */
	off = 4;
	do {
		if (off + 4 > rlen || n_present == 8) return 0;
		present[n_present] = rd32(buf + off);
		off += 4;
	} while (present[n_present++] & 0x80000000u);

	memset(r, 0, sizeof *r);
	r->phy = MT7612U_PHY_OFDM;
	r->nss = 1;
	r->bw  = MT7612U_BW_20;

	for (unsigned w = 0; w < n_present; w++) {
		for (unsigned b = 0; b < 31; b++, bit++) {
			const uint8_t *p;
			uint8_t align, size;

			if (!(present[w] & (1u << b)))
				continue;
			if (bit >= sizeof rt_field / sizeof rt_field[0])
				return (int)rlen;          /* unknown tail: stop */
			align = rt_field[bit].align;
			size  = rt_field[bit].size;
			if (!size)
				return (int)rlen;
			off = (off + align - 1) & ~((size_t)align - 1);
			/* A declared field that runs past the declared header
			 * length is a malformed header, not a short one. Returning
			 * rlen here reported success, and both injection entry
			 * points then transmitted the frame at whatever defaults
			 * or half-parsed rate had accumulated - a silently wrong
			 * rate rather than a refused frame. An unknown *trailing*
			 * present bit is different and still stops cleanly above:
			 * there the header is well formed, we just cannot read the
			 * rest of it. */
			if (off + size > rlen) {
				ERR("radiotap: field for present bit %u runs past the "
				    "declared %zu-byte header", bit, rlen);
				return -1;
			}
			p = buf + off;
			off += size;

			switch (bit) {
			case RT_RATE:
				if (!(present[0] & (1u << RT_MCS)))
					r->mcs = (uint8_t)legacy_rate_index(p[0], &r->phy);
				break;
			case RT_TX_FLAGS:
				if (rd16(p) & RT_TX_FLAGS_NOACK) r->no_ack = 1;
				break;
			case RT_DBM_TX_POWER:
				/* Absolute dBm is a device-level knob here
				 * (mt7612u_set_txpower); txwi carries only a
				 * 4-bit relative trim, and mapping an absolute
				 * target onto it needs the per-rate EEPROM
				 * ceiling for the current channel. Say so
				 * rather than accept the field and drop it. */
				if (!warned_tx_power) {
					warned_tx_power = 1;
					LOG("radiotap DBM_TX_POWER (%d dBm) ignored: "
					    "use mt7612u_set_txpower() for the base level "
					    "and mt7612u_tx_rate.power_adj for per-frame trim",
					    (int)(int8_t)p[0]);
				}
				break;
			case RT_MCS: {
				uint8_t known = p[0], flags = p[1];

				r->phy = MT7612U_PHY_HT;
				r->mcs = p[2];
				r->nss = (uint8_t)(1 + (p[2] >> 3));
				/* The bandwidth field is declared by HAVE_BW (0x01),
				 * NOT by HAVE_MCS (0x02). Gating it on 0x02 narrowed a
				 * requested 40 MHz frame to 20 whenever a caller
				 * declared bandwidth without also declaring an MCS
				 * index - legal radiotap, and silent on air. Same shape
				 * as the VHT bandwidth bug fixed earlier; this is its
				 * HT twin. Flags bits 1:0 are 0=20 1=40 2=20L 3=20U, so
				 * only 1 is a 40 MHz frame. */
				if ((known & 0x01) && ((flags & 0x03) == 1))
					r->bw = MT7612U_BW_40;
				if (known & 0x04) r->sgi  = (flags >> 2) & 1;
				if (known & 0x10) r->ldpc = (flags >> 4) & 1;
				if (known & 0x20) r->stbc = ((flags >> 5) & 3) ? 1 : 0;
				break;
			}
			case RT_VHT: {
				uint16_t known = rd16(p);
				uint8_t flags = p[2], bwc = p[3], mcs_nss = p[4], coding = p[8];

				r->phy = MT7612U_PHY_VHT;
				r->mcs = (uint8_t)(mcs_nss >> 4);
				r->nss = (uint8_t)(mcs_nss & 0x0f);
				if (!r->nss) r->nss = 1;
				if (known & 0x0004) r->stbc = flags & 1;

				if (flags & 0x04)   r->sgi = 1;
				if (coding & 0x01)  r->ldpc = 1;
				r->bw = vht_bandwidth(bwc);
				break;
			}
			default:
				break;
			}
		}
	}
	return (int)rlen;
}

/*
 * devourer's send_packet() contract: one buffer, radiotap header followed by
 * the 802.11 MPDU. Per-frame rate comes from the header.
 */
int mt7612u_send_packet(struct mt7612u_dev *d, const void *buf, size_t len)
{
	struct mt7612u_tx_rate r;
	const uint8_t *p = buf;
	int rlen = mt_radiotap_parse(p, len, &r);

	if (rlen <= 0 || (size_t)rlen >= len) {
		ERR("send_packet: no usable radiotap header");
		return -1;
	}
	return mt7612u_tx(d, p + rlen, len - (size_t)rlen, &r);
}

/*
 * devourer's send_packets() contract: several radiotap-framed MPDUs in one
 * call. MT7612U packs them into a single bulk-OUT transfer using
 * MT_TXD_INFO_NEXT_VLD, so a burst costs one USB transaction rather than one
 * per frame. Returns the number accepted.
 */
size_t mt7612u_send_packets(struct mt7612u_dev *d,
                            const struct mt7612u_tx_view *pkts, size_t count)
{
	uint8_t buf[MT_USB_AGG_BUF];
	size_t sent = 0, i = 0;

	if (!pkts) return 0;

	while (i < count) {
		size_t off = 0, n_in_buf = 0, j;
		struct { const uint8_t *mpdu; size_t len; struct mt7612u_tx_rate r; }
			sel[MT_USB_AGG_MAX];

		/*
		 * Pass 1 selects and fully validates. The radiotap parse happens
		 * here, not in pass 2: a frame that pass 2 could still reject
		 * would break the chain it is building - NEXT_VLD and the single
		 * trailing zero word are assigned by position, so dropping the
		 * frame that happens to be last leaves the transfer unterminated.
		 */
		while (i < count && n_in_buf < MT_USB_AGG_MAX) {
			const uint8_t *p = pkts[i].data;
			size_t plen = pkts[i].len, need;
			struct mt7612u_tx_rate r;
			int rlen;

			if (!p || plen < 8) { i++; continue; }
			rlen = mt_radiotap_parse(p, plen, &r);
			if (rlen <= 0 || (size_t)rlen >= plen) { i++; continue; }

			/* Worst case for one block: TXINFO + TXWI + hdr pad +
			 * MPDU + alignment + trailer. */
			need = 4 + MT_TXWI_LEN + 2 + (plen - (size_t)rlen) + 3 + 4;
			/* The same per-frame ceiling mt7612u_tx() applies. Without
			 * it this path accepted anything that fit the 16 KB
			 * aggregate buffer, so one public entry point refused a
			 * frame the other aired - and the caller could not tell
			 * which limit it was under. */
			if (need > MT_TX_BUF_MAX) {
				ERR("frame of %zu bytes exceeds the %d-byte per-frame "
				    "ceiling", plen - (size_t)rlen, MT_TX_BUF_MAX);
				i++;
				continue;
			}
			if (off + need > sizeof buf) break;

			sel[n_in_buf].mpdu = p + rlen;
			sel[n_in_buf].len  = plen - (size_t)rlen;
			sel[n_in_buf].r    = r;
			n_in_buf++;
			off += need;
			i++;
		}
		if (!n_in_buf) break;

		/* Pass 2 only builds. NEXT_VLD on every block except the last,
		 * and only the last carries the 4-byte zero trailer. */
		off = 0;
		for (j = 0; j < n_in_buf; j++) {
			int last = (j + 1 == n_in_buf);
			int blk = mt_tx_build(d, buf + off, sizeof buf - off,
			                      sel[j].mpdu, sel[j].len, &sel[j].r,
			                      0xff, 0, !last, last);

			if (blk < 0) {
				/* Cannot happen after pass 1's checks, but if it
				 * ever does the chain is unterminated - drop the
				 * whole transfer rather than air a truncated one. */
				ERR("send_packets: block %zu failed to build", j);
				off = 0;
				break;
			}
			off += (size_t)blk;
		}
		if (!off) break;

		if (d->a) {
			if (mt_async_tx_submit(d, buf, (int)off) == 0) sent += n_in_buf;
		} else {
			int n = 0;

			if (mt_bulk(d, MT_EP_OUT_AC_BE, buf, (int)off, &n, 500) == 0)
				sent += n_in_buf;
		}
	}
	return sent;
}
