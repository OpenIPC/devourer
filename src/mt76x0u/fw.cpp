/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * MT76x0U firmware load. A port of mt76/mt76x0/usb_mcu.c
 * (mt76x0u_load_firmware, mt76x0u_upload_firmware) and the shared
 * mt76x02_usb_mcu.c sender (mt76x02u_mcu_fw_send_data, mt76x02u_mcu_fw_reset),
 * reference/mt76 @ be5ce79.
 *
 * Differences from the MT7612U loader (src/mt7612u/fw.cpp), all from the
 * kernel source:
 *  - one image, no ROM patch;
 *  - the first MT_MCU_IVB_SIZE (64) bytes of the ILM are the IVB: the ILM is
 *    uploaded WITHOUT them, to destination 0x40, and they are handed to the
 *    chip as the data stage of the start command (DEV_MODE 0x12);
 *  - DLM goes to 0x80000, chunks are at most 0x38f8 - 8 payload bytes;
 *  - the USB DMA config is MAC register 0x0238, not CFG-space 0x9018;
 *  - MT_FCE_PSE_CTRL = 1 is written before the upload AND after it, as in the
 *    kernel source (wifit3 notes its cold-boot capture shows two identical
 *    writes "in succession"; whether that is these two is not established);
 *  - no upload at all when MT_MCU_COM_REG0 already reads 1: the kernel's warm
 *    path keeps the running firmware (mt76x0_firmware_running). `force` does
 *    not change that any more (see below).
 *
 * WARM START, AS MEASURED (bench 2026-11-19, MT7610U 0e8d:7610 over usbipd).
 * The kernel skips the upload when the firmware runs (usb_mcu.c:95). wifit3
 * does the opposite on purpose: its skip left "FW_READY set, but MCU bulk-IN
 * times out on every command", and it now always resets the firmware
 * (DEV_MODE 0x1) and re-uploads. On the bench:
 *  - the kernel's warm path worked (an hour of RX, no errors);
 *  - wifit3's path (`force`) did NOT: after the IVB start the chip stopped
 *    answering EP0 for more than 3 s, every attempt, and afterwards each warm
 *    start showed wifit3's symptoms - MCU commands timing out, then, once up,
 *    no frames over USB while the MAC's RX overflow counter (MT_RX_STAT_2)
 *    climbed. The receive DMA was jammed, and MCU replies share that path;
 *  - one MT_PBF_SYS_CTRL DMA|PBF reset pulse (MCU untouched) cleared it.
 * So `force` is refused on running firmware (it is honoured only in the sense
 * that a stopped firmware is always uploaded), and mt0_bring_up pulses the
 * DMA/PBF reset on every warm start, so a jam left by anything - wifit3, a
 * killed process, the Windows driver - clears without a replug.
 *
 * One more divergence, unresolved: wifit3 writes MT_FCE_PSE_CTRL twice back to
 * back before the upload because its capture of the kernel shows that; the
 * kernel source writes it once there. This port follows the source; the
 * transfer-log diff against wifit3 (src/mt76x0u/README.md) lists it.
 *
 * No retry. The MT7612U loader retries once after a 3 s settle because a
 * settling window was MEASURED on that part; nothing has been measured on this
 * one, and the kernel does not retry. What WAS measured here is the wait after
 * the start command: the chip drops the first EP0 read after it, and over
 * USB/IP that cancelled read wedges EP0 until a write goes through, so the
 * wait writes MT_FCE_PSE_CTRL after a read with no answer
 * (mt0_fw_wait_running).
 */
#include <stdlib.h>
#include <string.h>
#include <chrono>
#include "internal.h"

/* The wait after the start command (mt0_fw_wait_running). The chip drops the
 * first EP0 read that follows it: that read times out NATIVELY too (cold-start
 * usbmon trace on a native host: 300 ms, then the next read answers at once), and a
 * 2 s read gets no answer either (WSL trace), so it is lost, not slow.
 * Natively the cancelled read is harmless. Over USB/IP (WSL2 + usbipd-win
 * 5.3) it is not: every later read timed out - 30 of 30 over 9 s in one
 * trace - until an EP0 write went through (MT_FCE_PSE_CTRL = 1, the write the
 * kernel sequence makes right after this wait: its first attempt timed out,
 * the second completed, and EP0 answered normally from then on; two traces,
 * 2026-11-24). So after a read with no answer this sends that write, then
 * reads again. Reads are single attempts so that no retry burst sits on a
 * stuck EP0. */
#define MT0_FW_START_TIMEOUT_US 3000000

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

static const uint8_t kReqOut = (uint8_t)LIBUSB_ENDPOINT_OUT |
                               (uint8_t)LIBUSB_REQUEST_TYPE_VENDOR |
                               (uint8_t)LIBUSB_RECIPIENT_DEVICE;

int mt0_fw_parse(const uint8_t *buf, size_t n, struct mt0_fw_image *img,
                 const char **err)
{
	if (!buf || n < MT0_FW_HDR_LEN) {
		if (err) *err = "firmware shorter than its 32-byte header";
		return -1;
	}
	const uint32_t ilm_len = get_le32(buf);
	const uint32_t dlm_len = get_le32(buf + 4);

	/* mt76x0u_load_firmware: ilm_len must exceed the IVB, and the file
	 * must be exactly header + ILM + DLM. Computed in 64 bits so a hostile
	 * header cannot wrap the sum. */
	if (ilm_len <= MT_MCU_IVB_SIZE) {
		if (err) *err = "invalid firmware image (ILM not larger than the IVB)";
		return -1;
	}
	if ((uint64_t)MT0_FW_HDR_LEN + ilm_len + dlm_len != (uint64_t)n) {
		if (err) *err = "invalid firmware image (size != 32 + ilm + dlm)";
		return -1;
	}
	const uint8_t *payload = buf + MT0_FW_HDR_LEN;
	img->ivb = payload;
	img->ilm = payload + MT_MCU_IVB_SIZE;
	img->ilm_len = ilm_len - MT_MCU_IVB_SIZE;
	img->dlm = payload + ilm_len;
	img->dlm_len = dlm_len;
	img->build_ver = (uint16_t)(buf[8] | (buf[9] << 8));
	img->fw_ver = (uint16_t)(buf[10] | (buf[11] << 8));
	memcpy(img->build_time, buf + 16, 16);
	img->build_time[16] = '\0';
	return 0;
}

int mt0_fw_build_chunk(uint8_t *scratch, const uint8_t *src, int len)
{
	/* __mt76x02u_mcu_fw_send_data: the info word carries the UNROUNDED
	 * length, the payload follows, then four zero bytes; the bulk length
	 * uses the rounded one. */
	const uint32_t info = FIELD_PREP(MT_MCU_MSG_PORT, CPU_TX_PORT) |
	                      FIELD_PREP(MT_MCU_MSG_LEN, (uint32_t)len) |
	                      MT_MCU_MSG_TYPE_CMD;
	const int rlen = (len + 3) & ~3;

	put_le32(scratch, info);
	memcpy(scratch + 4, src, (size_t)len);
	/* The kernel zeroes 4 bytes after `len`; the bytes between len and
	 * rlen are zero too here so the rounded tail never carries stale
	 * scratch from a previous, longer chunk. */
	memset(scratch + 4 + len, 0, (size_t)(rlen - len) + 4);
	return 4 + rlen + 4;
}

int mt0_fw_running(struct mt76x0u_dev *d)
{
	uint32_t v;

	if (mt0_rr_chk(d, MT_MCU_COM_REG0, &v))
		return -1;
	return v == 1;
}

static void fw_reset(struct mt76x0u_dev *d)
{
	/* mt76x02u_mcu_fw_reset: DEV_MODE, wValue 0x1, no data. */
	mt0_vendor_req(d, MT_VEND_DEV_MODE, kReqOut, 0x1, 0, NULL, 0);
}

static int fw_send_chunk(struct mt76x0u_dev *d, uint8_t *scratch,
                         const uint8_t *src, int len, uint32_t dst)
{
	const int total = mt0_fw_build_chunk(scratch, src, len);
	const int rlen = (len + 3) & ~3;
	uint32_t idx;

	mt0_single_wr(d, MT_VEND_WRITE_FCE, MT_FCE_DMA_ADDR, dst);
	mt0_single_wr(d, MT_VEND_WRITE_FCE, MT_FCE_DMA_LEN, (uint32_t)rlen << 16);

	const int rc = mt0_bulk(d, MT_EP_OUT_INBAND_CMD, scratch, total, NULL, 1000);
	if (rc) {
		ERR("firmware upload failed: %s", libusb_error_name(rc));
		return -1;
	}
	if (mt0_rr_chk(d, MT_TX_CPU_FROM_FCE_CPU_DESC_IDX, &idx))
		return -1;
	mt0_wr(d, MT_TX_CPU_FROM_FCE_CPU_DESC_IDX, idx + 1);
	return 0;
}

/* mt76x02u_mcu_fw_send_data: chunks of max_payload - 8, destination advancing
 * with the source, 5-10 ms apart. */
static int fw_send_data(struct mt76x0u_dev *d, const uint8_t *data,
                        uint32_t data_len, uint32_t offset)
{
	/* Per call, not static: two adapters may load at once. The kernel
	 * allocates max_payload; header + rounded payload + trailer is at most
	 * that, since a chunk carries at most max_payload - 8. */
	uint8_t *scratch = (uint8_t *)malloc(MT0_FW_URB_MAX_PAYLOAD);
	const uint32_t max_len = MT0_FW_URB_MAX_PAYLOAD - 8;
	uint32_t pos = 0;
	int rc = 0;

	if (!scratch)
		return -1;
	while (pos < data_len) {
		const uint32_t len = data_len - pos < max_len ? data_len - pos : max_len;

		rc = fw_send_chunk(d, scratch, data + pos, (int)len, offset + pos);
		if (rc)
			break;
		pos += len;
		mt0_usleep(7500);
	}
	free(scratch);
	return rc;
}

int mt0_fw_load(struct mt76x0u_dev *d, const uint8_t *blob, size_t n,
                int force, int *uploaded)
{
	struct mt0_fw_image img;
	const char *perr = NULL;
	uint32_t val;
	int rc;

	if (uploaded) *uploaded = 0;
	mt0_io_clear(d);

	mt0_wr(d, MT_USB_DMA_CFG,
	       MT_USB_DMA_CFG_RX_BULK_EN | MT_USB_DMA_CFG_TX_BULK_EN);

	rc = mt0_fw_running(d);
	if (rc < 0) {
		ERR("MT_MCU_COM_REG0 unreadable - cannot tell whether firmware runs");
		return -1;
	}
	if (rc == 1) {
		/* The kernel's warm path: keep what is running. A forced re-upload
		 * onto running firmware is refused. On an MT7610U (bench,
		 * 2026-11-19, WSL2 + usbipd) DEV_MODE fw_reset + upload + IVB
		 * start left the chip silent on EP0 for over 3 s, and every later
		 * warm start then found the RX DMA jammed (MAC RX overflow, no
		 * frames over USB) until the DMA/PBF reset in mt0_bring_up. The
		 * kernel never re-uploads onto running firmware either. */
		if (force)
			WARN("forced firmware re-upload refused: it jams this chip's "
			     "RX DMA; keeping the running firmware");
		LOG("firmware already running - not uploading (warm start)");
		d->mcu_running = 1;
		d->q_selected = 0;
		return 0;
	}

	if (mt0_fw_parse(blob, n, &img, &perr)) {
		ERR("%s", perr);
		return -1;
	}
	LOG("firmware %d.%d.%02d build %x %.16s (ILM %u + IVB %u, DLM %u)",
	    (img.fw_ver >> 12) & 0xf, (img.fw_ver >> 8) & 0xf, img.fw_ver & 0xf,
	    img.build_ver, img.build_time, img.ilm_len, MT_MCU_IVB_SIZE,
	    img.dlm_len);

	/* mt76x0u_load_firmware writes the raw address 0x1004, which is
	 * MT_MAC_SYS_CTRL (mt76x02_regs.h:269). */
	mt0_wr(d, MT_MAC_SYS_CTRL, 0x2c);
	mt0_set(d, MT_USB_DMA_CFG,
	        MT_USB_DMA_CFG_RX_BULK_EN | MT_USB_DMA_CFG_TX_BULK_EN |
	        FIELD_PREP(MT_USB_DMA_CFG_RX_BULK_AGG_TOUT, 0x20));
	fw_reset(d);
	mt0_usleep(5500);

	mt0_wr(d, MT_FCE_PSE_CTRL, 1);
	mt0_wr(d, MT_TX_CPU_FROM_FCE_BASE_PTR, 0x400230);  /* FCE tx_fs_base_ptr */
	mt0_wr(d, MT_TX_CPU_FROM_FCE_MAX_COUNT, 1);        /* FCE tx_fs_max_cnt */
	mt0_wr(d, MT_FCE_PDMA_GLOBAL_CONF, 0x44);          /* FCE pdma enable */
	mt0_wr(d, MT_FCE_SKIP_FS, 3);                      /* FCE skip_fs_en */

	if (mt0_rr_chk(d, MT_USB_DMA_CFG, &val)) {
		ERR("MT_USB_DMA_CFG unreadable before upload");
		return -1;
	}
	val |= MT_USB_DMA_CFG_UDMA_TX_WL_DROP;
	mt0_wr(d, MT_USB_DMA_CFG, val);
	val &= ~MT_USB_DMA_CFG_UDMA_TX_WL_DROP;
	mt0_wr(d, MT_USB_DMA_CFG, val);

	/* mt76x0u_upload_firmware */
	rc = fw_send_data(d, img.ilm, img.ilm_len, MT_MCU_IVB_SIZE);
	if (!rc)
		rc = fw_send_data(d, img.dlm, img.dlm_len, MT_MCU_DLM_OFFSET);
	if (!rc) {
		/* The IVB is the data stage of the start command. The buffer is
		 * copied: the transport may hold it past this call on a timeout. */
		uint8_t ivb[MT_MCU_IVB_SIZE];
		memcpy(ivb, img.ivb, sizeof ivb);
		if (mt0_vendor_req(d, MT_VEND_DEV_MODE, kReqOut, 0x12, 0, ivb,
		                   sizeof ivb) < 0) {
			ERR("firmware start (DEV_MODE 0x12 + IVB) failed");
			rc = -1;
		}
	}
	if (!rc && mt0_fw_wait_running(d, MT0_FW_START_TIMEOUT_US))
		rc = -1;

	/* After the upload whether or not it worked, as the kernel does. */
	mt0_wr(d, MT_FCE_PSE_CTRL, 1);

	if (rc)
		return -1;
	if (mt0_io_errors(d)) {
		ERR("firmware load: %u register transfer(s) failed", mt0_io_errors(d));
		return -1;
	}
	LOG("firmware running");
	d->mcu_running = 1;
	d->q_selected = 0;
	if (uploaded) *uploaded = 1;
	return 0;
}

/* mt76x0u_upload_firmware's MT_MCU_COM_REG0 poll, except that a failed read
 * means "not answering yet", not "failed": it is not logged per request, not
 * left in the I/O-error accumulator (the caller's sequence check would fail
 * the bring-up on it), and the wait goes on to the deadline. */
int mt0_fw_wait_running(struct mt76x0u_dev *d, int timeout_us)
{
	using clock = std::chrono::steady_clock;
	const auto t0 = clock::now();
	const auto deadline = t0 + std::chrono::microseconds(timeout_us < 0 ? 0 : timeout_us);
	const unsigned io_before = mt0_io_errors(d);
	const int quiet_before = d->ep0_quiet;
	unsigned failed = 0;
	int ok = 0;

	const int tries_before = d->ep0_tries;

	d->ep0_quiet = 1;
	for (;;) {
		uint32_t v;

		d->ep0_tries = 1;
		if (mt0_rr_chk(d, MT_MCU_COM_REG0, &v)) {
			failed++;
			/* Unstick EP0 (see above); normal retries for the write. */
			d->ep0_tries = tries_before;
			mt0_wr(d, MT_FCE_PSE_CTRL, 1);
		} else if (v & 1) {
			ok = 1;
			break;
		}
		if (clock::now() >= deadline)
			break;
		mt0_usleep(1000);
	}
	d->ep0_quiet = quiet_before;
	d->ep0_tries = tries_before;
	mt0_io_restore(d, io_before);

	const long ms = (long)std::chrono::duration_cast<std::chrono::milliseconds>(
		clock::now() - t0).count();
	if (ok) {
		if (failed)
			LOG("firmware started after %ld ms; %u register read(s) got no "
			    "answer while it booted", ms, failed);
		return 0;
	}
	ERR("firmware failed to start (MT_MCU_COM_REG0 never read 1 in %ld ms, "
	    "%u read(s) failed)", ms, failed);
	return -1;
}

/* --- firmware files -------------------------------------------------------- */

/* Whole file into a malloc'd buffer. NULL (and no message) when it does not
 * exist, so the caller can fall back; a message for anything else. */
static uint8_t *slurp(const char *dir, const char *name, size_t *out_len,
                      int *exists)
{
	char path[1024];
	FILE *f;
	uint8_t *buf;
	long n;

	*exists = 0;
	snprintf(path, sizeof path, "%s/%s", dir, name);
	f = fopen(path, "rb");
	if (!f)
		return NULL;
	*exists = 1;
	if (fseek(f, 0, SEEK_END) || (n = ftell(f)) <= 0 || fseek(f, 0, SEEK_SET)) {
		fclose(f);
		ERR("%s is empty or unreadable", path);
		return NULL;
	}
	buf = (uint8_t *)malloc((size_t)n);
	if (!buf || fread(buf, 1, (size_t)n, f) != (size_t)n) {
		free(buf);
		fclose(f);
		ERR("short read on %s", path);
		return NULL;
	}
	fclose(f);
	*out_len = (size_t)n;
	return buf;
}

int mt76x0u_load_firmware(struct mt76x0u_dev *d, const char *fw_dir,
                          int force, int *uploaded)
{
	if (uploaded) *uploaded = 0;
	if (!d) return -1;

	/* Running firmware needs no file: check first, as the kernel does
	 * (mt76x0u_load_firmware returns before mt76x0_get_firmware). */
	const int running = mt0_fw_running(d);
	if (running < 0) {
		ERR("MT_MCU_COM_REG0 unreadable - cannot tell whether firmware runs");
		return -1;
	}
	if (running) /* force is refused on running firmware: see mt0_fw_load */
		return mt0_fw_load(d, NULL, 0, force, uploaded);

	if (!fw_dir) fw_dir = "firmware";
	/* mt76x0_get_firmware: mt7610e.bin first, mt7610u.bin as the fallback. */
	size_t n = 0;
	int exists = 0;
	const char *name = MT7610E_FIRMWARE_NAME;
	uint8_t *blob = slurp(fw_dir, name, &n, &exists);
	if (!blob) {
		if (exists)
			return -1;
		LOG("%s/%s not found, switching to %s", fw_dir,
		    MT7610E_FIRMWARE_NAME, MT7610U_FIRMWARE_NAME);
		name = MT7610U_FIRMWARE_NAME;
		blob = slurp(fw_dir, name, &n, &exists);
		if (!blob) {
			if (!exists)
				ERR("neither %s nor %s in %s (linux-firmware ships them "
				    "zstd-compressed on most distributions: decompress "
				    "into a directory and point the firmware dir at it)",
				    MT7610E_FIRMWARE_NAME, MT7610U_FIRMWARE_NAME, fw_dir);
			return -1;
		}
	}
	LOG("firmware file %s/%s (%zu B)", fw_dir, name, n);
	const int rc = mt0_fw_load(d, blob, n, force, uploaded);
	free(blob);
	return rc;
}

int mt76x0u_firmware_running(struct mt76x0u_dev *d)
{
	return d ? mt0_fw_running(d) : -1;
}

int mt76x0u_mcu_running(const struct mt76x0u_dev *d)
{
	return d ? d->mcu_running : 0;
}
