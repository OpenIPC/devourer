/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * Frame-shape tests: 802.11 header length, and the RX L2-pad fold that
 * depends on it. No hardware, no privileges - mt_rx_parse() only reads the
 * device struct for the chainmask and the EEPROM gain terms.
 *
 * Both cases here are regressions, not hypotheticals:
 *
 *  - hdrlen treated only RTS and PS-Poll as 16-byte control frames, leaving
 *    BlockAckReq, BlockAck and the two CF-End subtypes at 10. On TX that
 *    inserts the L2 pad ten bytes in, i.e. inside the frame.
 *  - the RX fold moved a fixed 24 bytes. L2PAD is only ever set when the
 *    header is *not* 4-aligned - 26 bytes (QoS) or 30 (4-address) - so the
 *    last two header bytes were left behind and overwritten by the pad. On
 *    a QoS frame those two bytes are the QoS Control field, so every TID
 *    and ack-policy read as zero.
 */
#include <stdio.h>
#include <string.h>
#include "internal.h"

static int fails;

static void expect_hdrlen(const char *what, uint8_t b0, uint8_t b1, int want)
{
	uint8_t fc[2] = { b0, b1 };
	int got = mt_hdrlen_from_fc(fc);

	if (got != want) {
		printf("  FAIL %-28s fc=%02x%02x want %d got %d\n",
		       what, b0, b1, want, got);
		fails++;
	}
}

static void test_hdrlen(void)
{
	printf("mt_hdrlen_from_fc:\n");

	/* management */
	expect_hdrlen("beacon",              0x80, 0x00, 24);
	expect_hdrlen("action +Order",       0xd0, 0x80, 28);

	/* control: 16 by default, 10 only for CTS and ACK */
	expect_hdrlen("BlockAckReq",         0x84, 0x00, 16);
	expect_hdrlen("BlockAck",            0x94, 0x00, 16);
	expect_hdrlen("PS-Poll",             0xa4, 0x00, 16);
	expect_hdrlen("RTS",                 0xb4, 0x00, 16);
	expect_hdrlen("CTS",                 0xc4, 0x00, 10);
	expect_hdrlen("ACK",                 0xd4, 0x00, 10);
	expect_hdrlen("CF-End",              0xe4, 0x00, 16);
	expect_hdrlen("CF-End+CF-Ack",       0xf4, 0x00, 16);

	/* data */
	expect_hdrlen("data 3-addr",         0x08, 0x00, 24);
	expect_hdrlen("QoS data",            0x88, 0x00, 26);
	expect_hdrlen("QoS data +Order",     0x88, 0x80, 30);
	expect_hdrlen("data 4-addr",         0x08, 0x03, 30);
	expect_hdrlen("QoS data 4-addr",     0x88, 0x03, 32);
}

/*
 * Build [FCE 4][RXWI 32][26-byte QoS header][2 pad][body] and parse it.
 * The QoS Control field sits at header offset 24..25.
 */
static void test_rx_l2pad(void)
{
	struct mt7612u_dev d;
	uint8_t buf[256];
	const uint8_t *frame = NULL;
	struct mt7612u_rx_info info;
	static const uint8_t body[] = "payload";
	const int hdrlen = 26, pad = 2;
	const int mpdu = hdrlen + (int)sizeof body;
	uint8_t *hdr;
	uint32_t rxinfo = MT_RXINFO_L2PAD;
	uint32_t ctl = FIELD_PREP(MT_RXWI_CTL_MPDU_LEN, (uint32_t)mpdu);
	int n, len;

	printf("mt_rx_parse, L2 pad on a QoS frame:\n");

	memset(&d, 0, sizeof d);
	d.chainmask = 0x0202;
	memset(buf, 0, sizeof buf);

	for (int i = 0; i < 4; i++) buf[MT_DMA_HDR_LEN + i] = (uint8_t)(rxinfo >> (8 * i));
	for (int i = 0; i < 4; i++) buf[MT_DMA_HDR_LEN + 4 + i] = (uint8_t)(ctl >> (8 * i));
	/* rate word: OFDM, index 0 - not what this test is about, but it must
	 * decode without tripping the VHT/HT index maths. */
	buf[MT_DMA_HDR_LEN + 10] = 0x00;
	buf[MT_DMA_HDR_LEN + 11] = 0x20;

	hdr = buf + MT_DMA_HDR_LEN + MT_RXWI_LEN;
	hdr[0] = 0x88;                 /* QoS data */
	hdr[1] = 0x00;
	memset(hdr + 4, 0xff, 6);      /* addr1 */
	hdr[24] = 0x07;                /* QoS Control: TID 7 ... */
	hdr[25] = 0x00;                /* ... normal ack policy */
	memset(hdr + hdrlen, 0xaa, pad);            /* the pad the MAC inserted */
	memcpy(hdr + hdrlen + pad, body, sizeof body);
	n = MT_DMA_HDR_LEN + MT_RXWI_LEN + hdrlen + pad + (int)sizeof body;

	len = mt_rx_parse(&d, buf, n, &frame, &info);

	if (len != mpdu) {
		printf("  FAIL length: want %d got %d\n", mpdu, len);
		fails++;
		return;
	}
	if (frame[24] != 0x07 || frame[25] != 0x00) {
		printf("  FAIL QoS Control zeroed by the pad fold: %02x %02x\n",
		       frame[24], frame[25]);
		fails++;
	}
	if (memcmp(frame + hdrlen, body, sizeof body) != 0) {
		printf("  FAIL body misaligned after the fold\n");
		fails++;
	}
	if (frame[0] != 0x88) {
		printf("  FAIL frame control lost: %02x\n", frame[0]);
		fails++;
	}

	/*
	 * Negative control. Redo the fold the old way - a fixed 24 bytes - and
	 * confirm it produces exactly the corruption described above. Without
	 * this, a test that merely passes proves nothing about what it caught.
	 */
	{
		uint8_t again[256];
		uint8_t *h;

		memcpy(again, buf, sizeof again);
		h = again + MT_DMA_HDR_LEN + MT_RXWI_LEN;
		/* re-lay the pre-fold bytes, since mt_rx_parse mutated buf */
		memset(h, 0, hdrlen + pad + sizeof body);
		h[0] = 0x88;
		h[24] = 0x07;
		h[25] = 0x00;
		memset(h + hdrlen, 0xaa, pad);
		memcpy(h + hdrlen + pad, body, sizeof body);

		memmove(h + 2, h, 24);          /* the old, fixed-24 fold */
		if (h[2 + 24] == 0x07) {
			printf("  FAIL negative control: the old fold preserved "
			       "QoS Control, so this test could not have caught it\n");
			fails++;
		}
	}
}

/*
 * Radiotap VHT bandwidth: the code names a width *and* a sub-channel within
 * it, so "40 (20L)" is a 20 MHz frame. Mapping 1-3 to 40 and >=4 to 80 aired
 * a requested 20-in-40 at 40 MHz.
 */
static void test_vht_bandwidth(void)
{
	/* code -> expected width, from the radiotap VHT bandwidth table */
	static const struct { uint8_t code; enum mt7612u_bw bw; const char *what; } cases[] = {
		{  0, MT7612U_BW_20, "20"        }, {  1, MT7612U_BW_40, "40"        },
		{  2, MT7612U_BW_20, "40 (20L)"  }, {  3, MT7612U_BW_20, "40 (20U)"  },
		{  4, MT7612U_BW_80, "80"        }, {  5, MT7612U_BW_40, "80 (40L)"  },
		{  6, MT7612U_BW_40, "80 (40U)"  }, {  7, MT7612U_BW_20, "80 (20LL)" },
		{  8, MT7612U_BW_20, "80 (20LU)" }, {  9, MT7612U_BW_20, "80 (20UL)" },
		{ 10, MT7612U_BW_20, "80 (20UU)" },
	};
	/* radiotap: present = VHT(21) only; then 12 bytes of VHT at offset 8,
	 * 2-byte aligned. known = BANDWIDTH, bandwidth byte at VHT+3. */
	uint8_t buf[8 + 12 + 32];
	struct mt7612u_tx_rate r;

	printf("radiotap VHT bandwidth:\n");
	for (unsigned i = 0; i < sizeof cases / sizeof cases[0]; i++) {
		memset(buf, 0, sizeof buf);
		buf[2] = 20;                    /* radiotap length */
		buf[4] = 0x00; buf[5] = 0x00;
		buf[6] = 0x20; buf[7] = 0x00;   /* present bit 21 = VHT */
		buf[8] = 0x40;                  /* known: BANDWIDTH */
		buf[11] = cases[i].code;        /* VHT+3 = bandwidth */
		buf[12] = 0x10;                 /* mcs_nss: MCS1 NSS0 -> nss 1 */

		if (mt_radiotap_parse(buf, sizeof buf, &r) != 20) {
			printf("  FAIL code %u: header not parsed\n", cases[i].code);
			fails++;
			continue;
		}
		if (r.bw != cases[i].bw) {
			printf("  FAIL code %2u %-10s want bw %d got %d\n",
			       cases[i].code, cases[i].what, (int)cases[i].bw, (int)r.bw);
			fails++;
		}
	}
}

/*
 * MT_RATE_PHY is three bits, so 5-7 are representable and name no format.
 * mt76 drops such a frame (mt76x02_mac_process_rate returns -EINVAL); before
 * this was ported, one decoded through the default arm and was reported as
 * the lowest legacy rate - a real-looking CCK frame rather than garbage.
 */
static void test_invalid_phy(void)
{
	struct mt7612u_dev d;
	uint8_t buf[128];
	const uint8_t *frame = NULL;
	struct mt7612u_rx_info info;
	uint32_t ctl = FIELD_PREP(MT_RXWI_CTL_MPDU_LEN, 40u);

	printf("mt_rx_parse, invalid PHY in the rate word:\n");
	for (unsigned phy = 0; phy < 8; phy++) {
		uint16_t rate = (uint16_t)FIELD_PREP(MT_RATE_PHY, phy);
		int len;

		memset(&d, 0, sizeof d);
		d.chainmask = 0x0202;
		memset(buf, 0, sizeof buf);
		for (int i = 0; i < 4; i++)
			buf[MT_DMA_HDR_LEN + 4 + i] = (uint8_t)(ctl >> (8 * i));
		buf[MT_DMA_HDR_LEN + 10] = (uint8_t)(rate & 0xff);
		buf[MT_DMA_HDR_LEN + 11] = (uint8_t)(rate >> 8);
		buf[MT_DMA_HDR_LEN + MT_RXWI_LEN] = 0x08;  /* data frame */

		len = mt_rx_parse(&d, buf, MT_DMA_HDR_LEN + MT_RXWI_LEN + 40,
		                  &frame, &info);
		if (phy <= 4 && len != 40) {
			printf("  FAIL phy %u is valid but was dropped\n", phy);
			fails++;
		}
		if (phy > 4 && len != 0) {
			printf("  FAIL phy %u names no format but decoded as rate %u\n",
			       phy, info.mcs);
			fails++;
		}
	}
}

/*
 * Channel grouping. The hardware tunes the *centre* of a widened channel, so
 * at 80 MHz control channel 36 tunes 42. Getting this wrong is silent: every
 * register write succeeds and the part transmits, just not where it was asked
 * to. Only a correctly tuned receiver or a spectrum analyser would show it,
 * which is why it is pinned here instead.
 */
static void test_chan_group(void)
{
	static const struct {
		uint8_t chan, bw, hw, idx, group;
	} ok[] = {
		/* 20 MHz: the control channel is the hardware channel. */
		{  36, MT7612U_BW_20,  36, 0, 0 },
		{ 149, MT7612U_BW_20, 149, 0, 0 },
		{   1, MT7612U_BW_20,   1, 0, 0 },   /* 2.4 GHz is fine at 20 */

		/* 40 MHz: centre is control +/- 2, and which side alternates. */
		{  36, MT7612U_BW_40,  38, 1, 0 },
		{  40, MT7612U_BW_40,  38, 3, 1 },
		{ 149, MT7612U_BW_40, 151, 1, 0 },
		{ 161, MT7612U_BW_40, 159, 3, 1 },

		/* 80 MHz: all four control channels of a group tune one centre,
		 * and ch_group_index says which quarter the control channel is. */
		{  36, MT7612U_BW_80,  42, 0, 0 },
		{  40, MT7612U_BW_80,  42, 1, 1 },
		{  44, MT7612U_BW_80,  42, 2, 2 },
		{  48, MT7612U_BW_80,  42, 3, 3 },
		{  52, MT7612U_BW_80,  58, 0, 0 },
		{  64, MT7612U_BW_80,  58, 3, 3 },
		{ 100, MT7612U_BW_80, 106, 0, 0 },
		{ 112, MT7612U_BW_80, 106, 3, 3 },
		{ 116, MT7612U_BW_80, 122, 0, 0 },
		{ 128, MT7612U_BW_80, 122, 3, 3 },
		{ 132, MT7612U_BW_80, 138, 0, 0 },
		{ 144, MT7612U_BW_80, 138, 3, 3 },
		/* U-NII-3 is the case a naive centre formula gets wrong: the
		 * group starts at 149, not 148, so every channel in it carries
		 * a +1 that the integer divide has to absorb. */
		{ 149, MT7612U_BW_80, 155, 0, 0 },
		{ 153, MT7612U_BW_80, 155, 1, 1 },
		{ 157, MT7612U_BW_80, 155, 2, 2 },
		{ 161, MT7612U_BW_80, 155, 3, 3 },
		/* 2.4 GHz 40 MHz reaches centres 6-9, i.e. control 4-11. */
		{   4, MT7612U_BW_40,   6, 1, 0 },
		{   6, MT7612U_BW_40,   8, 1, 0 },
		{   8, MT7612U_BW_40,   6, 3, 1 },
		{  11, MT7612U_BW_40,   9, 3, 1 },
	};
	static const struct { uint8_t chan, bw; const char *why; } refused[] = {
		/* 5.35-5.47 GHz: the arithmetic yields centres 74 and 90, which
		 * are not allocated. Tuning them is what the grid check stops. */
		{  68, MT7612U_BW_80, "centre 74 is not allocated" },
		{  96, MT7612U_BW_80, "centre 90 is not allocated" },
		/* Off-grid control channels. 38 is a legal 40 MHz *centre*, so a
		 * caller could pass it by mistake; at 80 MHz it names no group. */
		{  38, MT7612U_BW_80, "not a control channel of any 80 group" },
		{  42, MT7612U_BW_80, "42 is a centre, not a control channel" },
		{  34, MT7612U_BW_80, "below the 5 GHz 80 MHz grid" },
		/* The 165/169/173/177 group centres on 171 = 5855 MHz and spans
		 * to 5895, past the 5825 this driver's caps declare. mt76's own
		 * channel list has it; we refuse until the band is widened. */
		{ 165, MT7612U_BW_80, "centre 171 is outside the declared band" },
		{ 177, MT7612U_BW_80, "centre 171 is outside the declared band" },
		/* 40 MHz gets the same grid check as 80. Without it these tune
		 * silently: 254 wraps a uint8_t to centre 0, and 165 centres on
		 * 167 = 5835 MHz, outside the declared band. */
		{ 254, MT7612U_BW_40, "uint8_t wrap: would centre on 0" },
		{ 255, MT7612U_BW_40, "uint8_t wrap: would centre on 1" },
		{ 165, MT7612U_BW_40, "centre 167 is outside the declared band" },
		{  15, MT7612U_BW_40, "not on the 5 GHz 40 MHz grid" },
		{  35, MT7612U_BW_40, "not on the 5 GHz 40 MHz grid" },
		/* 2.4 GHz: 1-3 would need a secondary at or below channel 0,
		 * 12-13 one above 13. Both are what the old wrap produced. */
		{   1, MT7612U_BW_40, "would centre on 255 after the wrap" },
		{   2, MT7612U_BW_40, "would centre on 0 after the wrap" },
		{  13, MT7612U_BW_40, "would need channel 15" },
		{   6, MT7612U_BW_80, "no 80 MHz in 2.4 GHz at all" },
		{  14, MT7612U_BW_80, "no 80 MHz in 2.4 GHz at all" },
	};
	unsigned i;

	printf("mt_chan_group:\n");
	for (i = 0; i < sizeof ok / sizeof ok[0]; i++) {
		uint8_t hw = 0xff, idx = 0xff, group = 0xff;

		if (mt_chan_group(ok[i].chan, ok[i].bw, &hw, &idx, &group)) {
			printf("  FAIL ch %3u bw %u refused, should be accepted\n",
			       ok[i].chan, ok[i].bw);
			fails++;
			continue;
		}
		if (hw != ok[i].hw || idx != ok[i].idx || group != ok[i].group) {
			printf("  FAIL ch %3u bw %u: want hw %u idx %u group %u, "
			       "got hw %u idx %u group %u\n",
			       ok[i].chan, ok[i].bw, ok[i].hw, ok[i].idx,
			       ok[i].group, hw, idx, group);
			fails++;
		}
	}

	for (i = 0; i < sizeof refused / sizeof refused[0]; i++) {
		uint8_t hw = 0x5a, idx = 0x5a, group = 0x5a;

		if (!mt_chan_group(refused[i].chan, refused[i].bw, &hw, &idx, &group)) {
			printf("  FAIL ch %3u bw %u accepted (%s), tuned hw %u\n",
			       refused[i].chan, refused[i].bw, refused[i].why, hw);
			fails++;
			continue;
		}
		/* A refusal must not write the outputs: a caller that checks the
		 * return but reuses the buffer would otherwise tune whatever the
		 * last accepted call left behind. */
		if (hw != 0x5a || idx != 0x5a || group != 0x5a) {
			printf("  FAIL ch %3u bw %u refused but wrote its outputs\n",
			       refused[i].chan, refused[i].bw);
			fails++;
		}
	}

	/* Negative control: the 80 MHz centre must not be the control channel.
	 * If this ever passes, the maths has degenerated to a pass-through and
	 * every positive case above would still look right at 20 MHz. */
	{
		uint8_t hw = 0;

		if (!mt_chan_group(36, MT7612U_BW_80, &hw, NULL, NULL) && hw == 36) {
			printf("  FAIL negative control: 80 MHz returned the control "
			       "channel, so this test could not detect a missing centre\n");
			fails++;
		}
	}
}

/*
 * Radiotap HT bandwidth. The MCS field's `known` byte declares bandwidth with
 * HAVE_BW (0x01); HAVE_MCS (0x02) declares the MCS index. Gating the width on
 * 0x02 silently narrowed a requested 40 MHz frame to 20 whenever a caller
 * declared bandwidth without an MCS index - legal radiotap, and invisible
 * except on air. The VHT half of this bug was caught in review; this is the
 * HT twin that came with it.
 */
static void test_ht_bandwidth(void)
{
	static const struct {
		uint8_t known, flags; enum mt7612u_bw bw; const char *what;
	} cases[] = {
		/* HAVE_BW alone is enough to select the width. */
		{ 0x01, 0x01, MT7612U_BW_40, "HAVE_BW, bw=40"           },
		{ 0x01, 0x00, MT7612U_BW_20, "HAVE_BW, bw=20"           },
		{ 0x01, 0x02, MT7612U_BW_20, "HAVE_BW, bw=20L"          },
		{ 0x01, 0x03, MT7612U_BW_20, "HAVE_BW, bw=20U"          },
		/* Both declared: still 40. */
		{ 0x03, 0x01, MT7612U_BW_40, "HAVE_BW|HAVE_MCS, bw=40"  },
		/* Bandwidth NOT declared: the field is meaningless, stay at 20
		 * even though the bits happen to read 40. */
		{ 0x02, 0x01, MT7612U_BW_20, "HAVE_MCS only, bw bits 40" },
		{ 0x00, 0x01, MT7612U_BW_20, "nothing declared"          },
	};
	uint8_t buf[8 + 3 + 32];
	struct mt7612u_tx_rate r;
	unsigned i;

	printf("radiotap HT bandwidth:\n");
	for (i = 0; i < sizeof cases / sizeof cases[0]; i++) {
		memset(buf, 0, sizeof buf);
		buf[2] = 11;                    /* radiotap length: 8 + 3 */
		buf[6] = 0x08;                  /* present bit 19 = MCS */
		buf[8]  = cases[i].known;
		buf[9]  = cases[i].flags;
		buf[10] = 0;                    /* MCS 0 */

		if (mt_radiotap_parse(buf, sizeof buf, &r) != 11) {
			printf("  FAIL %-24s header not parsed\n", cases[i].what);
			fails++;
			continue;
		}
		if (r.bw != cases[i].bw) {
			printf("  FAIL %-24s want bw %d got %d\n",
			       cases[i].what, (int)cases[i].bw, (int)r.bw);
			fails++;
		}
	}

	/* A declared field running past the declared header length is
	 * malformed. This used to return the header length, which both
	 * injection entry points read as success and then transmitted at
	 * whatever defaults had accumulated. */
	{
		memset(buf, 0, sizeof buf);
		buf[2] = 9;                     /* claims 9 bytes: 8 + 1 */
		buf[6] = 0x08;                  /* but declares MCS, which needs 3 */
		if (mt_radiotap_parse(buf, sizeof buf, &r) > 0) {
			printf("  FAIL malformed header accepted (MCS field runs "
			       "past the declared length)\n");
			fails++;
		}
	}
}


int main(void)
{
	test_hdrlen();
	test_invalid_phy();
	test_rx_l2pad();
	test_chan_group();
	test_vht_bandwidth();
	test_ht_bandwidth();
	printf("frame_shape: %s\n", fails ? "FAIL" : "PASS");
	return fails ? 1 : 0;
}
