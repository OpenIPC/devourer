/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * ROM patch + ILM/DLM firmware load. Ported from mt76/mt76x2/usb_mcu.c and
 * mt76x02_usb_mcu.c. Chunk sizes and destination addresses are confirmed
 * against a usbmon capture of the kernel driver (INVESTIGATION.md §4).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "internal.h"

#define MCU_FW_URB_MAX_PAYLOAD      0x3900
#define MCU_ROM_PATCH_MAX_PAYLOAD   2048
#define MCU_ILM_OFFSET              0x80000
#define MCU_DLM_OFFSET              0x110000
#define MCU_ROM_PATCH_OFFSET        0x90000
#define PATCH_HDR_LEN               30
#define FW_HDR_LEN                  32

/* The two loaders below fail in two ways that call for different handling, so
 * they say which one they hit: a blob that is absent or malformed will fail the
 * same way forever, while the bulk upload itself is the transfer mt_fw_init()
 * retries. */
#define FW_ERR_FATAL   (-1)   /* blob missing, short or malformed; or no memory */
#define FW_ERR_UPLOAD  (-2)   /* the bulk upload failed on the wire */
/* The chip did not come up after a complete upload.  A DEVICE failure, not a
 * blob one: it is the same settling window the upload retry was measured
 * against, so it must retry too - `bringup swreset`'s whole verdict is
 * whether the chip takes firmware again, and it used to settle and pass. */
#define FW_ERR_DEVICE  (-3)   /* uploaded, but the chip did not start it */

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

static uint8_t *slurp(const char *dir, const char *name, size_t *out_len)
{
	char path[512];
	FILE *f;
	uint8_t *buf;
	long n;

	snprintf(path, sizeof path, "%s/%s", dir, name);
	f = fopen(path, "rb");
	if (!f) { ERR("cannot open %s", path); return NULL; }
	fseek(f, 0, SEEK_END); n = ftell(f); fseek(f, 0, SEEK_SET);
	if (n <= 0) { fclose(f); ERR("%s is empty", path); return NULL; }
	buf = malloc((size_t)n);
	if (!buf || fread(buf, 1, (size_t)n, f) != (size_t)n) {
		free(buf); fclose(f); ERR("short read on %s", path); return NULL;
	}
	fclose(f);
	*out_len = (size_t)n;
	return buf;
}

/* MT_VEND_DEV_MODE with wValue 0x1 - the vendor reset before each blob. */
static void fw_reset(struct mt7612u_dev *d)
{
	mt_vendor_req(d, MT_VEND_DEV_MODE,
	              LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR |
	              LIBUSB_RECIPIENT_DEVICE, 0x1, 0, NULL, 0);
}

/* The FCE preamble that must precede every blob upload. */
static void fce_setup(struct mt7612u_dev *d)
{
	uint32_t val = MT_USB_DMA_CFG_RX_BULK_EN | MT_USB_DMA_CFG_TX_BULK_EN |
	               FIELD_PREP(MT_USB_DMA_CFG_RX_BULK_AGG_TOUT, 0x20);

	mt_wr(d, CFG_ADDR(MT_USB_U3DMA_CFG), val);
	mt_wr(d, MT_FCE_PSE_CTRL, 0x1);
	mt_wr(d, MT_TX_CPU_FROM_FCE_BASE_PTR, 0x400230);
	mt_wr(d, MT_TX_CPU_FROM_FCE_MAX_COUNT, 0x1);
	mt_wr(d, MT_FCE_PDMA_GLOBAL_CONF, 0x44);
	mt_wr(d, MT_FCE_SKIP_FS, 0x3);
}

static int fw_send_chunk(struct mt7612u_dev *d, uint8_t *scratch,
                         const uint8_t *src, int len, uint32_t dst)
{
	uint32_t info, idx;
	int rlen, total, rc;

	info = FIELD_PREP(MT_MCU_MSG_PORT, CPU_TX_PORT) |
	       FIELD_PREP(MT_MCU_MSG_LEN, (uint32_t)len) |
	       MT_MCU_MSG_TYPE_CMD;

	put_le32(scratch, info);
	memcpy(scratch + 4, src, (size_t)len);
	memset(scratch + 4 + len, 0, 4);

	mt_single_wr(d, MT_VEND_WRITE_FCE, MT_FCE_DMA_ADDR, dst);
	rlen = (len + 3) & ~3;
	mt_single_wr(d, MT_VEND_WRITE_FCE, MT_FCE_DMA_LEN, (uint32_t)rlen << 16);

	total = 4 + rlen + 4;
	rc = mt_bulk(d, MT_EP_OUT_INBAND_CMD, scratch, total, NULL, 1000);
	if (rc) {
		ERR("fw chunk bulk out: %s", libusb_error_name(rc));
		return FW_ERR_UPLOAD;
	}

	idx = mt_rr(d, MT_TX_CPU_FROM_FCE_CPU_DESC_IDX) + 1;
	mt_wr(d, MT_TX_CPU_FROM_FCE_CPU_DESC_IDX, idx);
	return 0;
}

static int fw_send_data(struct mt7612u_dev *d, const uint8_t *data, int data_len,
                        uint32_t max_payload, uint32_t offset)
{
	int max_len = (int)max_payload - 8, pos = 0, rc = 0;
	uint8_t *scratch = malloc(max_payload);

	if (!scratch) return FW_ERR_FATAL;
	while (data_len > 0) {
		int len = data_len < max_len ? data_len : max_len;

		rc = fw_send_chunk(d, scratch, data + pos, len, offset + (uint32_t)pos);
		if (rc) break;
		data_len -= len;
		pos += len;
		mt_usleep(7500);
	}
	free(scratch);
	return rc;
}

static int load_rom_patch(struct mt7612u_dev *d, const char *dir)
{
	size_t n;
	uint8_t *fw = slurp(dir, "mt7662_rom_patch.bin", &n);
	int rc = FW_ERR_FATAL, up;

	if (!fw) return FW_ERR_FATAL;
	if (n <= PATCH_HDR_LEN) { ERR("rom patch too short"); goto out; }

	LOG("ROM patch build: %.15s  (%zu byte payload)", (char *)fw, n - PATCH_HDR_LEN);

	/* is_mt7612() -> rom_protect is false: no MT_MCU_SEMAPHORE_03 handshake
	 * and no "already applied" early-out. The patch reloads every time. */
	fce_setup(d);
	fw_reset(d);
	mt_usleep(7500);
	fce_setup(d);

	up = fw_send_data(d, fw + PATCH_HDR_LEN, (int)(n - PATCH_HDR_LEN),
	                  MCU_ROM_PATCH_MAX_PAYLOAD, MCU_ROM_PATCH_OFFSET);
	if (up) { rc = up; goto out; }

	/* enable_patch and reset_wmt are USB_TYPE_CLASS, not VENDOR. */
	{
		static const uint8_t enable[] = { 0x6f, 0xfc, 0x08, 0x01, 0x20, 0x04,
		                                  0x00, 0x00, 0x00, 0x09, 0x00 };
		static const uint8_t wmt[]    = { 0x6f, 0xfc, 0x05, 0x01,
		                                  0x07, 0x01, 0x00, 0x04 };
		uint8_t b[16];
		const uint8_t type = LIBUSB_ENDPOINT_OUT |
		                     LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_DEVICE;

		memcpy(b, enable, sizeof enable);
		mt_vendor_req(d, MT_VEND_DEV_MODE, type, 0x12, 0, b, sizeof enable);
		memcpy(b, wmt, sizeof wmt);
		mt_vendor_req(d, MT_VEND_DEV_MODE, type, 0x12, 0, b, sizeof wmt);
	}
	mt_usleep(20000);

	/* rev >= E3 -> the applied bit is MT_MCU_CLOCK_CTL bit 0. */
	if (!mt_poll(d, MT_MCU_CLOCK_CTL, BIT(0), BIT(0), 100000)) {
		ERR("ROM patch did not apply (MT_MCU_CLOCK_CTL=0x%08x)",
		    mt_rr(d, MT_MCU_CLOCK_CTL));
		rc = FW_ERR_DEVICE;
		goto out;
	}
	LOG("ROM patch applied");
	rc = 0;
out:
	free(fw);
	return rc;
}

static int load_firmware(struct mt7612u_dev *d, const char *dir)
{
	size_t n;
	uint8_t *fw = slurp(dir, "mt7662.bin", &n);
	uint32_t ilm_len, dlm_len, dlm_offset = MCU_DLM_OFFSET;
	uint16_t fw_ver, build_ver;
	int rc = FW_ERR_FATAL, up;

	if (!fw) return FW_ERR_FATAL;
	if (n < FW_HDR_LEN) { ERR("firmware too short"); goto out; }

	ilm_len = get_le32(fw);
	dlm_len = get_le32(fw + 4);
	build_ver = (uint16_t)(fw[8] | (fw[9] << 8));
	fw_ver    = (uint16_t)(fw[10] | (fw[11] << 8));

	if (n != FW_HDR_LEN + ilm_len + dlm_len) {
		ERR("firmware size %zu != 32+%u+%u", n, ilm_len, dlm_len);
		goto out;
	}
	LOG("firmware %d.%d.%02d build 0x%x  %.16s  (ilm %u dlm %u)",
	    (fw_ver >> 12) & 0xf, (fw_ver >> 8) & 0xf, fw_ver & 0xf,
	    build_ver, (char *)(fw + 16), ilm_len, dlm_len);

	fw_reset(d);
	mt_usleep(7500);
	fce_setup(d);

	up = fw_send_data(d, fw + FW_HDR_LEN, (int)ilm_len,
	                  MCU_FW_URB_MAX_PAYLOAD, MCU_ILM_OFFSET);
	if (up) { rc = up; goto out; }

	/* rev >= E3: DLM lands at 0x110800. Confirmed on the wire. */
	dlm_offset += 0x800;
	up = fw_send_data(d, fw + FW_HDR_LEN + ilm_len, (int)dlm_len,
	                  MCU_FW_URB_MAX_PAYLOAD, dlm_offset);
	if (up) { rc = up; goto out; }

	/* load IVB: MT_VEND_DEV_MODE, VENDOR type, wValue 0x12, no data. */
	mt_vendor_req(d, MT_VEND_DEV_MODE,
	              LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR |
	              LIBUSB_RECIPIENT_DEVICE, 0x12, 0, NULL, 0);

	if (!mt_poll(d, MT_MCU_COM_REG0, BIT(0), BIT(0), 100000)) {
		ERR("firmware failed to start (MT_MCU_COM_REG0=0x%08x)",
		    mt_rr(d, MT_MCU_COM_REG0));
		rc = FW_ERR_DEVICE;
		goto out;
	}
	mt_set(d, MT_MCU_COM_REG0, BIT(1));
	mt_wr(d, MT_FCE_PSE_CTRL, 0x1);
	LOG("firmware running");
	rc = 0;
out:
	free(fw);
	return rc;
}

int mt_fw_init(struct mt7612u_dev *d, const char *fw_dir)
{
	/* The caller's bracket is already open: mt_init_hardware() calls
	 * mt_io_clear() and then runs mt_power_cycle() and mt_wait_for_mac()
	 * BEFORE us, and there is only one accumulator.  Sample it so the retry
	 * can discard its own failed attempt without forgiving the power-on
	 * phase's - which would let a partly powered WLAN core report success. */
	unsigned io_err_pre = mt_io_errors(d);

	if (!fw_dir) fw_dir = "firmware";

	for (int attempt = 0; ; attempt++) {
		int rc = load_rom_patch(d, fw_dir);

		if (!rc)
			rc = load_firmware(d, fw_dir);
		if (!rc)
			return 0;

		/* Retry the upload transfer, and only that.  Measured (12/12
		 * cycles, both adapters): after a process dies mid transfer the
		 * first firmware upload times out at its first chunk, and the
		 * same upload succeeds by itself 3-8 s later with nothing
		 * repaired.  That is a settling window, not a wedge, and an open
		 * that lands inside it must not fail.  A blob that is missing or
		 * malformed fails identically 3 s later, so it fails now with
		 * the error that names the file rather than after a warning
		 * about a transfer that never started.  Everything that is NOT a
		 * blob problem retries: the wire upload (FW_ERR_UPLOAD) and the
		 * chip failing to start what was uploaded (FW_ERR_DEVICE) are
		 * both the settling window. */
		if (attempt || rc == FW_ERR_FATAL)
			return -1;

		WARN("firmware load failed (%s) - a previous run may have died "
		     "mid transfer; retrying after a 3 s settle",
		     rc == FW_ERR_UPLOAD ? "upload" : "chip did not start it");
		mt_usleep(3000000);

		/* The retry must not inherit the failed attempt's register
		 * errors.  The chunk that timed out also drove EP0 accessors
		 * either side of it - the FCE address and length writes, the
		 * CPU_DESC_IDX read-modify-write, mt_poll - and each failure
		 * bumps the accumulator that mt_init_hardware() brackets the
		 * bring-up with.  It refuses to report success while that is
		 * non-zero, so without this the firmware loads on the second
		 * attempt and mt_open() still fails with "failed register
		 * transfers".  Restored to the value on entry rather than
		 * zeroed: mt_init_hardware()'s bracket opened before the
		 * power-on phase, so zeroing would forgive its failures too. */
		mt_io_restore(d, io_err_pre);
	}
}
