/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * RX path. RX_BULK_AGG_EN is left off in init_dma(), so one bulk transfer
 * carries exactly one frame and this is a straight parse - no de-aggregation.
 *
 * Wire layout (INVESTIGATION.md §9):
 *   [FCE info 4B][RXWI 32B][802.11 frame]
 */
#include <string.h>
#include "internal.h"

#define RX_BUF_SIZE 4096

static uint32_t get_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint16_t get_le16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }

/*
 * Decode the 16-bit rate word - the same encoding the TX path writes.
 * Returns -1 when the PHY field names no format this radio can produce.
 *
 * MT_RATE_PHY is three bits, so 5, 6 and 7 are representable and mean
 * nothing; mt76 returns -EINVAL for them in mt76x02_mac_process_rate() and
 * drops the frame. Decoding one anyway lands it in the caller's `default`
 * arm and reports it as the lowest legacy rate, which reads as a real CCK
 * frame rather than as garbage.
 */
static int decode_rate(uint16_t rate, struct mt7612u_rx_info *out)
{
	uint32_t idx = FIELD_GET(MT_RATE_INDEX, rate);

	out->phy  = (enum mt7612u_phy)FIELD_GET(MT_RATE_PHY, rate);
	if (out->phy > MT7612U_PHY_VHT)
		return -1;
	out->bw   = (enum mt7612u_bw)FIELD_GET(MT_RATE_BW, rate);
	out->sgi  = !!(rate & MT_RATE_SGI);
	out->ldpc = !!(rate & MT_RATE_LDPC);
	out->stbc = !!(rate & MT_RATE_STBC);

	switch (out->phy) {
	case MT7612U_PHY_VHT:
		out->mcs = FIELD_GET(MT_RATE_VHT_IDX, idx);
		out->nss = FIELD_GET(MT_RATE_VHT_NSS, idx) + 1;
		break;
	case MT7612U_PHY_HT:
	case MT7612U_PHY_HT_GF:
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

/*
 * Parse one completed RX buffer. Returns the 802.11 frame length (excluding
 * the RXWI), or 0 if the buffer holds nothing usable. `frame` receives a
 * pointer into `buf`. Shared by the synchronous reader and the async ring, so
 * both decode identically.
 */
int mt_rx_parse(struct mt7612u_dev *d, uint8_t *buf, int n,
                const uint8_t **frame, struct mt7612u_rx_info *info)
{
	int len, pad = 0;
	uint32_t rxinfo, ctl;
	const uint8_t *rxwi;

	if (n < MT_DMA_HDR_LEN + MT_RXWI_LEN) return 0;

	rxwi   = buf + MT_DMA_HDR_LEN;
	rxinfo = get_le32(rxwi);
	ctl    = get_le32(rxwi + 4);

	memset(info, 0, sizeof *info);
	info->mpdu_len = (uint16_t)FIELD_GET(MT_RXWI_CTL_MPDU_LEN, ctl);
	info->seq      = (uint16_t)(get_le16(rxwi + 8) >> 4);
	info->crc_err  = !!(rxinfo & MT_RXINFO_CRCERR);
	info->ampdu    = !!(rxinfo & MT_RXINFO_AMPDU);
	if (decode_rate(get_le16(rxwi + 10), info)) {
		mt_async_note_invalid(d);
		return 0;
	}

	/* Per-chain RSSI is a fixed 4-byte field. Correction terms come from
	 * the EEPROM (mt76x02_mac_get_rssi); with them at zero these are the
	 * raw chip values, which is still enough to compare two chains. */
	info->n_chains = (uint8_t)((d->chainmask & 0xf) > 1 ? 2 : 1);
	for (int c = 0; c < 4; c++)
		info->rssi[c] = (int8_t)((int8_t)rxwi[12 + c] +
		                         (c < 2 ? d->cal.rssi_offset[c] : 0) -
		                         d->cal.lna_gain);

	for (int i = 0; i < 4; i++)
		info->bbp[i] = get_le32(rxwi + 16 + 4 * i);

	/* RXWI byte 14 is a noise floor - see the header for how that was
	 * established. Below -100 dBm is under the thermal floor of a 20 MHz
	 * channel, so it is reported as no estimate rather than as a very quiet
	 * channel. */
	info->noise = info->rssi[2];
	info->noise_valid = info->noise > -100 && info->noise < -30;
	info->snr_db = info->noise_valid
	             ? (int8_t)(info->rssi[0] - info->noise) : 0;

	if (rxinfo & MT_RXINFO_L2PAD)
		pad = 2;

	/* MPDU_LEN excludes the pad: mt76 trims to it only after removing the
	 * pad (mt76x02_mac_process_rx), so the buffer holds hdrlen + pad + body. */
	len = (int)info->mpdu_len;
	if (len > n - MT_DMA_HDR_LEN - MT_RXWI_LEN - pad)
		len = n - MT_DMA_HDR_LEN - MT_RXWI_LEN - pad;
	if (len < 0) return 0;

	*frame = buf + MT_DMA_HDR_LEN + MT_RXWI_LEN;
	/*
	 * Fold the L2 pad out, exactly as mt76x02_remove_hdr_pad(): move the
	 * *header* up over the pad, then start the frame two bytes in.
	 *
	 * The length moved must be the real header length. L2PAD is only ever
	 * set when the header is not 4-aligned - 26 bytes (QoS) or 30 (4-addr) -
	 * so moving a fixed 24 leaves the last two header bytes behind and
	 * overwrites them with pad. On a QoS frame those two bytes are the QoS
	 * Control field, i.e. every TID and ack-policy read as zero.
	 */
	if (pad) {
		uint8_t *base = buf + MT_DMA_HDR_LEN + MT_RXWI_LEN;
		int avail = n - MT_DMA_HDR_LEN - MT_RXWI_LEN, hdrlen;

		if (avail < 2) return 0;
		hdrlen = mt_hdrlen_from_fc(base);
		if (len < hdrlen || avail < hdrlen + pad) return 0;
		memmove(base + pad, base, (size_t)hdrlen);
		*frame = base + pad;
	}
	return len;
}

int mt_rx_one(struct mt7612u_dev *d, uint8_t *buf, int bufsize,
              const uint8_t **frame, struct mt7612u_rx_info *info,
              unsigned timeout_ms)
{
	int n = 0, rc = mt_bulk(d, MT_EP_IN_PKT_RX, buf, bufsize, &n, timeout_ms);

	if (rc == LIBUSB_ERROR_TIMEOUT) return 0;
	if (rc) return -1;
	return mt_rx_parse(d, buf, n, frame, info);
}
