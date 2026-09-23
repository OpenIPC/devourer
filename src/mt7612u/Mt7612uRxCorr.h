/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * The per-channel RSSI correction as one word.
 *
 * mt7612u_cal::rx_corr packs the two chain RSSI offsets and the LNA gain
 * (mt76x02_mac_get_rssi's three inputs) into a single uint32_t so that a
 * retune publishes all three at once and the RX parser reads them in one
 * load. Pure integer packing, no device dependency - kept in its own header
 * so the headless mapping selftest can pin the sign handling.
 *
 * Layout: byte 0 = chain-0 offset, byte 1 = chain-1 offset, byte 2 = LNA
 * gain, each a signed dB; byte 3 unused.
 */
#ifndef MT7612U_RX_CORR_H
#define MT7612U_RX_CORR_H

#include <stdint.h>

static inline uint32_t mt_rx_corr_pack(int8_t off0, int8_t off1, int8_t lna)
{
	return (uint32_t)(uint8_t)off0 | ((uint32_t)(uint8_t)off1 << 8) |
	       ((uint32_t)(uint8_t)lna << 16);
}

static inline void mt_rx_corr_unpack(uint32_t v, int8_t off[2], int8_t *lna)
{
	off[0] = (int8_t)(v & 0xff);
	off[1] = (int8_t)((v >> 8) & 0xff);
	*lna   = (int8_t)((v >> 16) & 0xff);
}

#endif
