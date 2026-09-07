/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * In-band MCU transport. Requests go out on EP 8 wrapped in the same 4-byte
 * TXINFO header as data frames (DPORT = CPU_TX_PORT, TYPE_CMD); responses come
 * back on EP 5 and are matched on the 4-bit sequence number.
 * Ported from mt76/mt76x02_usb_mcu.c and mt76x2/mcu.c.
 */
#include <string.h>
#include "internal.h"

#define MCU_RESP_URB_SIZE 1024
#define MCU_MSG_MAX       192   /* MT_INBAND_PACKET_MAX_LEN */

static void put_le32(uint8_t *p, uint32_t v)
{
	p[0] = v & 0xff; p[1] = (v >> 8) & 0xff;
	p[2] = (v >> 16) & 0xff; p[3] = (v >> 24) & 0xff;
}

static uint32_t get_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int mcu_wait_resp(struct mt7612u_dev *d, uint8_t seq)
{
	uint8_t buf[MCU_RESP_URB_SIZE];
	int len, rc;

	for (int i = 0; i < 5; i++) {
		rc = mt_bulk(d, MT_EP_IN_CMD_RESP, buf, sizeof buf, &len, 300);
		if (rc == LIBUSB_ERROR_TIMEOUT)
			continue;
		if (rc) {
			ERR("mcu resp bulk: %s", libusb_error_name(rc));
			return -1;
		}
		if (len < 4)
			continue;

		uint32_t rxfce = get_le32(buf);
		if (FIELD_GET(MT_RX_FCE_INFO_CMD_SEQ, rxfce) == seq &&
		    FIELD_GET(MT_RX_FCE_INFO_EVT_TYPE, rxfce) == MT_EVT_CMD_DONE)
			return 0;
		ERR("mcu resp mismatch: evt=%u seq=%u (want %u)",
		    FIELD_GET(MT_RX_FCE_INFO_EVT_TYPE, rxfce),
		    FIELD_GET(MT_RX_FCE_INFO_CMD_SEQ, rxfce), seq);
	}
	ERR("mcu command timed out waiting for response");
	return -1;
}

int mt_mcu_send(struct mt7612u_dev *d, int cmd, const void *data, int len,
                int wait_resp)
{
	uint8_t buf[4 + MCU_MSG_MAX + 8];
	uint8_t seq = 0;
	uint32_t info;
	int pad, total, rc;

	if (len > MCU_MSG_MAX) { ERR("mcu payload %d too long", len); return -1; }

	if (wait_resp) {
		seq = ++d->mcu_seq & 0xf;
		if (!seq)
			seq = ++d->mcu_seq & 0xf;
	}

	/* TXINFO: LEN is the payload rounded to 4, computed before the header
	 * is prepended - matching mt76x02u_skb_dma_info(). */
	info = FIELD_PREP(MT_TXD_INFO_LEN, (uint32_t)((len + 3) & ~3)) |
	       FIELD_PREP(MT_TXD_INFO_DPORT, CPU_TX_PORT) |
	       FIELD_PREP(MT_MCU_MSG_CMD_SEQ, seq) |
	       FIELD_PREP(MT_MCU_MSG_CMD_TYPE, (uint32_t)cmd) |
	       MT_MCU_MSG_TYPE_CMD;

	put_le32(buf, info);
	memcpy(buf + 4, data, len);
	/* pad the (header + payload) to 4, then a 4-byte zero terminator */
	pad = (((4 + len) + 3) & ~3) + 4 - (4 + len);
	memset(buf + 4 + len, 0, pad);
	total = 4 + len + pad;

	if (d->mculog) {
		fprintf(d->mculog, "cmd=%-2d seq=%u len=%d payload=", cmd, seq, len);
		for (int i = 0; i < len; i++)
			fprintf(d->mculog, "%02x", buf[4 + i]);
		fputc('\n', d->mculog);
	}

	rc = mt_bulk(d, MT_EP_OUT_INBAND_CMD, buf, total, NULL, 500);
	if (rc) { ERR("mcu cmd %d bulk out: %s", cmd, libusb_error_name(rc)); return -1; }

	return wait_resp ? mcu_wait_resp(d, seq) : 0;
}

int mt_mcu_function_select(struct mt7612u_dev *d, int func, uint32_t val)
{
	uint8_t msg[8];
	put_le32(msg, (uint32_t)func);
	put_le32(msg + 4, val);
	/* Q_SELECT is the one function mt76 does not wait on. */
	return mt_mcu_send(d, CMD_FUN_SET_OP, msg, sizeof msg, func != Q_SELECT);
}

int mt_mcu_set_radio_state(struct mt7612u_dev *d, int on)
{
	uint8_t msg[8];
	put_le32(msg, on ? RADIO_ON : RADIO_OFF);
	put_le32(msg + 4, 0);
	return mt_mcu_send(d, CMD_POWER_SAVING_OP, msg, sizeof msg, 0);
}

int mt_mcu_calibrate(struct mt7612u_dev *d, int type, uint32_t param)
{
	uint8_t msg[8];
	put_le32(msg, (uint32_t)type);
	put_le32(msg + 4, param);
	return mt_mcu_send(d, CMD_CALIBRATION_OP, msg, sizeof msg, 1);
}

int mt_mcu_load_cr(struct mt7612u_dev *d, uint8_t type, uint8_t temp, uint8_t ch)
{
	uint8_t msg[8];
	uint32_t val = BIT(31);

	val |= (uint32_t)(mt_ee(d, MT_EE_NIC_CONF_0) >> 8) & 0x00ff;
	val |= (uint32_t)(mt_ee(d, MT_EE_NIC_CONF_1) << 8) & 0xff00;

	msg[0] = type; msg[1] = temp; msg[2] = ch; msg[3] = 0;
	put_le32(msg + 4, val);
	return mt_mcu_send(d, CMD_LOAD_CR, msg, sizeof msg, 1);
}

int mt_mcu_init_gain(struct mt7612u_dev *d, uint8_t ch, uint32_t gain, int force)
{
	uint8_t msg[8];
	put_le32(msg, (uint32_t)ch | (force ? BIT(31) : 0));
	put_le32(msg + 4, gain);
	return mt_mcu_send(d, CMD_INIT_GAIN_OP, msg, sizeof msg, 1);
}

int mt_mcu_set_channel(struct mt7612u_dev *d, uint8_t ch, uint8_t bw,
                       uint8_t bw_index, int scan)
{
	uint8_t msg[8];

	msg[0] = ch;
	msg[1] = (uint8_t)scan;
	msg[2] = bw;
	msg[3] = 0;
	msg[4] = d->chainmask & 0xff;
	msg[5] = (d->chainmask >> 8) & 0xff;
	msg[6] = 0;          /* ext_chan: first pass carries none */
	msg[7] = 0;

	if (mt_mcu_send(d, CMD_SWITCH_CHANNEL_OP, msg, sizeof msg, 1))
		return -1;
	mt_usleep(7500);

	msg[6] = (uint8_t)(0xe0 + bw_index);
	return mt_mcu_send(d, CMD_SWITCH_CHANNEL_OP, msg, sizeof msg, 1);
}
