/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * RX parse for the MT76x0U. The RXWI is mt76x02's (shared with the MT7612U),
 * so this is src/mt7612u/rx.cpp's parser adapted to one chain; the kernel
 * reference is mt76x02_mac_process_rx() / mt76x02_mac_process_rate() /
 * mt76x02_remove_hdr_pad(), reference/mt76 @ be5ce79.
 *
 * init_usb_dma() leaves MT_USB_DMA_CFG_RX_BULK_AGG_EN off ("receive one frame
 * in each rx urb"), so one transfer is one frame:
 *   [DMA header 4B][RXWI 32B][802.11 header][L2 pad?][body][FCE trailer 4B]
 * There is no FCS: the MAC strips it.
 *
 * Not reported: the MT7612U's noise floor from RXWI byte 14 (rssi[2]). That
 * was identified by measurement on a 2-chain part; nothing says the 1-chain
 * mt76x0 fills it the same way, so it stays unread until a bench says.
 */
#include <string.h>
#include "internal.h"

static uint32_t get_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint16_t get_le16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

/* ieee80211_hdrlen(), as src/mt7612u/tx.cpp's mt_hdrlen_from_fc. */
int mt0_hdrlen_from_fc(const uint8_t *f)
{
	const unsigned fc = (unsigned)f[0] | ((unsigned)f[1] << 8);
	const unsigned type = (fc >> 2) & 3;
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
		return (fc & 0x8000) ? 28 : 24;
	if (type == 1)                          /* control */
		return ((fc & 0x00e0) == 0x00c0) ? 10 : 16;  /* CTS, ACK */
	return len;
}

/* mt76x02_mac_process_rate. -1 for a PHY field naming no format (5-7),
 * which mt76 drops with -EINVAL. */
static int decode_rate(uint16_t rate, struct mt76x0u_rx_info *out)
{
	const uint32_t idx = FIELD_GET(MT_RATE_INDEX, rate);
	const uint32_t phy = FIELD_GET(MT_RATE_PHY, rate);

	if (phy > MT76X0U_PHY_VHT)
		return -1;
	out->phy  = (enum mt76x0u_phy)phy;
	out->bw   = (enum mt76x0u_bw)FIELD_GET(MT_RATE_BW, rate);
	out->sgi  = !!(rate & MT_RATE_SGI);
	out->ldpc = !!(rate & MT_RATE_LDPC);
	out->stbc = !!(rate & MT_RATE_STBC);
	switch (out->phy) {
	case MT76X0U_PHY_VHT:
		out->mcs = (uint8_t)FIELD_GET(MT_RATE_VHT_IDX, idx);
		out->nss = (uint8_t)(FIELD_GET(MT_RATE_VHT_NSS, idx) + 1);
		break;
	case MT76X0U_PHY_HT:
	case MT76X0U_PHY_HT_GF:
		out->mcs = (uint8_t)idx;
		out->nss = (uint8_t)(1 + (idx >> 3));
		break;
	default:
		out->mcs = (uint8_t)idx;
		out->nss = 1;
		break;
	}
	return 0;
}

int mt0_rx_parse(struct mt76x0u_dev *d, uint8_t *buf, int n,
                 const uint8_t **frame, struct mt76x0u_rx_info *info)
{
	if (n < MT_DMA_HDR_LEN + MT_RXWI_LEN)
		return 0;

	const uint8_t *rxwi = buf + MT_DMA_HDR_LEN;
	const uint32_t rxinfo = get_le32(rxwi);
	const uint32_t ctl = get_le32(rxwi + 4);

	memset(info, 0, sizeof *info);
	info->mpdu_len = (uint16_t)FIELD_GET(MT_RXWI_CTL_MPDU_LEN, ctl);
	info->seq = (uint16_t)(get_le16(rxwi + 8) >> 4);
	info->crc_err = !!(rxinfo & MT_RXINFO_CRCERR);
	info->ampdu = !!(rxinfo & MT_RXINFO_AMPDU);
	if (decode_rate(get_le16(rxwi + 10), info))
		return MT_RX_PARSE_INVALID;

	/* mt76x02_mac_get_rssi(chain 0): raw + rssi_offset[0] - lna_gain, the
	 * pair read in one load so a retune on another thread is either fully
	 * before or fully after this frame. */
	const uint32_t corr = d->rx_corr.load(std::memory_order_relaxed);
	const int8_t offset = (int8_t)(uint8_t)(corr & 0xff);
	const int8_t lna = (int8_t)(uint8_t)((corr >> 8) & 0xff);
	info->rssi_raw = (int8_t)rxwi[12];
	info->rssi_dbm = (int8_t)(info->rssi_raw + offset - lna);

	/* MPDU_LEN excludes the L2 pad: the buffer holds hdr + pad + body. */
	const int pad = (rxinfo & MT_RXINFO_L2PAD) ? 2 : 0;
	int len = (int)info->mpdu_len;
	if (len > n - MT_DMA_HDR_LEN - MT_RXWI_LEN - pad)
		len = n - MT_DMA_HDR_LEN - MT_RXWI_LEN - pad;
	if (len <= 0)
		return 0;

	uint8_t *base = buf + MT_DMA_HDR_LEN + MT_RXWI_LEN;
	*frame = base;
	/* mt76x02_remove_hdr_pad: move the HEADER up over the pad. The real
	 * header length, not a fixed 24 - L2PAD is set exactly when the header
	 * is not 4-aligned (QoS 26, 4-address 30), and moving 24 would leave
	 * the QoS Control bytes behind (the MT7612U bug this rule records). */
	if (pad) {
		const int avail = n - MT_DMA_HDR_LEN - MT_RXWI_LEN;
		if (avail < 2)
			return 0;
		const int hdrlen = mt0_hdrlen_from_fc(base);
		if (len < hdrlen || avail < hdrlen + pad)
			return 0;
		memmove(base + pad, base, (size_t)hdrlen);
		*frame = base + pad;
	}
	return len;
}
