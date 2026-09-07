/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * libusb transport for MT7612U. Replaces mt76/usb.c's vendor-request and URB
 * plumbing; the wire encoding is identical (verified against usbmon, see
 * ../../INVESTIGATION.md §11).
 */
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <time.h>
#include <unistd.h>
#include <stdarg.h>

#include "internal.h"

/* See the LOG/WARN/ERR contract in internal.h. The whole line is formatted
 * first and emitted with one fwrite + fflush: two stdio calls could interleave
 * with a line from the libusb event thread, and an unflushed stderr can stall
 * a piped consumer mid-bring-up. Truncation is silent and deliberate - a
 * diagnostic is not worth a heap allocation on a path that may already be
 * failing. */
void mt_diag(char level, const char *fmt, ...)
{
	char line[512];
	int n;
	va_list ap;

	n = snprintf(line, sizeof line, "devourer [%c] mt7612u: ", level);
	if (n < 0 || (size_t)n >= sizeof line)
		return;
	va_start(ap, fmt);
	n += vsnprintf(line + n, sizeof line - (size_t)n - 1, fmt, ap);
	va_end(ap);
	if (n < 0)
		return;
	/* vsnprintf returns what it WOULD have written, so clamp before using
	 * it as a length - otherwise a truncated line writes past the buffer. */
	if ((size_t)n > sizeof line - 2)
		n = (int)(sizeof line - 2);
	line[n++] = '\n';

	fwrite(line, 1, (size_t)n, stderr);
	fflush(stderr);
}

#define REQ_IN   (LIBUSB_ENDPOINT_IN  | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE)
#define REQ_OUT  (LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE)
/* mt76's MT_VEND_REQ_TOUT_MS / MT_VEND_REQ_MAX_RETRY. The product of the two
 * is the worst-case cost of one register access, so it bounds every poll
 * loop below - which is why the timeout is 300 ms and not something longer. */
#define CTRL_TIMEOUT_MS 300
#define VEND_RETRIES 10

void mt_usleep(unsigned us)
{
	struct timespec ts = { .tv_sec = us / 1000000, .tv_nsec = (us % 1000000) * 1000 };
	nanosleep(&ts, NULL);
}

/* Held for the process lifetime; flock releases it on any exit. */
static int g_lock_fd = -1;

static uint64_t now_us(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)(ts.tv_nsec / 1000);
}

int mt_vendor_req(struct mt7612u_dev *d, uint8_t req, uint8_t type,
                  uint16_t val, uint16_t idx, void *buf, size_t len)
{
	int rc = LIBUSB_ERROR_OTHER;

	for (int i = 0; i < VEND_RETRIES; i++) {
		rc = libusb_control_transfer(d->h, type, req, val, idx,
		                             (unsigned char *)buf, (uint16_t)len,
		                             CTRL_TIMEOUT_MS);
		if (rc >= 0 || rc == LIBUSB_ERROR_NO_DEVICE)
			return rc;
		mt_usleep(5000);
	}
	ERR("vendor req %02x idx %04x failed: %s", req, idx, libusb_error_name(rc));
	return rc;
}

/* Address bits 31:30 select the space, exactly as mt76's __mt76u_rr/wr do. */
static uint8_t rd_req(uint32_t addr)
{
	if (addr & MT_VEND_TYPE_EEPROM) return MT_VEND_READ_EEPROM;
	if (addr & MT_VEND_TYPE_CFG)    return MT_VEND_READ_CFG;
	return MT_VEND_MULTI_READ;
}

static uint8_t wr_req(uint32_t addr)
{
	if (addr & MT_VEND_TYPE_CFG) return MT_VEND_WRITE_CFG;
	return MT_VEND_MULTI_WRITE;
}

/*
 * A register read that reports failure separately from the value. This matters
 * because 0xffffffff is a legitimate read on this part - MT_MAC_CSR0 returns
 * it while the core is still coming up - so it cannot double as a sentinel.
 * Returns 0 and fills *val on success, -1 on a transport failure.
 */
int mt_rr_chk(struct mt7612u_dev *d, uint32_t addr, uint32_t *val)
{
	uint8_t req = rd_req(addr), b[4] = { 0 };
	uint32_t a = addr & ~MT_VEND_TYPE_MASK;

	if (mt_vendor_req(d, req, REQ_IN, (uint16_t)(a >> 16), (uint16_t)a,
	                  b, sizeof b) != (int)sizeof b) {
		d->io_err++;
		return -1;
	}
	*val = (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
	       ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
	return 0;
}

/* Convenience form for the places that genuinely cannot act on a failure
 * (logging, one-shot identity reads). It still bumps d->io_err, so a caller
 * that cares can notice after the fact. */
uint32_t mt_rr(struct mt7612u_dev *d, uint32_t addr)
{
	uint32_t v;

	return mt_rr_chk(d, addr, &v) ? ~0u : v;
}

void mt_wr(struct mt7612u_dev *d, uint32_t addr, uint32_t val)
{
	uint8_t req = wr_req(addr), b[4];
	uint32_t a = addr & ~MT_VEND_TYPE_MASK;

	b[0] = val & 0xff; b[1] = (val >> 8) & 0xff;
	b[2] = (val >> 16) & 0xff; b[3] = (val >> 24) & 0xff;
	if (mt_vendor_req(d, req, REQ_OUT, (uint16_t)(a >> 16), (uint16_t)a,
	                  b, sizeof b) != (int)sizeof b)
		d->io_err++;

	/* Oracle-diff log: same shape decode.py renders from usbmon. */
	if (d->wrlog)
		fprintf(d->wrlog, "req=0x%02x addr=0x%04x data=%02x%02x%02x%02x\n",
		        req, (unsigned)(a & 0xffff), b[0], b[1], b[2], b[3]);
}

/*
 * Read-modify-write. A failed read MUST NOT be written back: mt_rr's ~0u would
 * turn the operation into "set every bit", and the addresses this is used on
 * (MT_WLAN_FUN_CTRL, MT_MAC_SYS_CTRL, the BBP AGC block) are exactly the ones
 * where that is destructive. Returns 0 on success, -1 if nothing was written.
 */
/*
 * Register-I/O failures accumulate per device rather than being returned from
 * every accessor.
 *
 * mt_wr() discarded mt_vendor_req()'s result entirely, so a write that
 * exhausted its retries mid-bring-up left the hardware partly configured while
 * the public call still returned success. Threading a status through every
 * writer would touch several hundred call sites in an initialisation sequence
 * that is deliberately a verbatim port of mt76's, and that churn would bury
 * the thing it is meant to protect.
 *
 * So this follows the shape the reads already had - mt_rr_chk() has always
 * bumped this same counter. Writes stay best-effort at the call site, and a
 * SEQUENCE checks the accumulator at its boundary: mt_io_clear() on entry,
 * mt_io_errors() on exit, and the whole setup fails if any access failed.
 * Optional or diagnostic writes stay best-effort by not being bracketed.
 */
void mt_io_clear(struct mt7612u_dev *d)      { d->io_err = 0; }
unsigned mt_io_errors(struct mt7612u_dev *d) { return d->io_err; }

/* Checked single write, for a caller that wants to fail at the write rather
 * than at a sequence boundary. */
int mt_wr_chk(struct mt7612u_dev *d, uint32_t addr, uint32_t val)
{
	unsigned before = d->io_err;

	mt_wr(d, addr, val);
	return d->io_err == before ? 0 : -1;
}

int mt_rmw(struct mt7612u_dev *d, uint32_t addr, uint32_t mask, uint32_t val)
{
	uint32_t cur;

	if (mt_rr_chk(d, addr, &cur)) {
		ERR("rmw 0x%05x skipped: read failed", addr & ~MT_VEND_TYPE_MASK);
		return -1;
	}
	mt_wr(d, addr, (cur & ~mask) | val);
	return 0;
}

/*
 * Poll against a real deadline rather than a count of sleeps. One register
 * access can itself cost up to VEND_RETRIES * CTRL_TIMEOUT_MS, so counting
 * iterations would let a caller asking for 200 ms block for seconds.
 */
int mt_poll(struct mt7612u_dev *d, uint32_t addr, uint32_t mask,
            uint32_t val, int timeout_us)
{
	uint64_t deadline = now_us() + (uint64_t)(timeout_us < 0 ? 0 : timeout_us);

	for (;;) {
		uint32_t cur;

		if (mt_rr_chk(d, addr, &cur)) {
			ERR("poll 0x%05x aborted: read failed",
			    addr & ~MT_VEND_TYPE_MASK);
			return 0;
		}
		if ((cur & mask) == val)
			return 1;
		if (now_us() >= deadline)
			return 0;
		mt_usleep(1000);
	}
}

void mt_single_wr(struct mt7612u_dev *d, uint8_t req, uint16_t off, uint32_t val)
{
	if (mt_vendor_req(d, req, REQ_OUT, (uint16_t)(val & 0xffff), off, NULL, 0) < 0)
		d->io_err++;
	if (mt_vendor_req(d, req, REQ_OUT, (uint16_t)(val >> 16),
	                  (uint16_t)(off + 2), NULL, 0) < 0)
		d->io_err++;
}

int mt_bulk(struct mt7612u_dev *d, uint8_t ep, void *buf, int len,
            int *xfered, unsigned timeout_ms)
{
	int n = 0;
	int rc = libusb_bulk_transfer(d->h, ep, (unsigned char *)buf, len,
	                              &n, timeout_ms);
	if (xfered) *xfered = n;
	return rc;
}

/* mt76x02_wait_for_mac(): MAC_CSR0 reads 0 or ~0 until the core is alive.
 * Both are legitimate values here, which is why this uses the checked read -
 * a transport failure is a different condition from "still coming up". */
int mt_wait_for_mac(struct mt7612u_dev *d)
{
	for (int i = 0; i < 500; i++) {
		uint32_t v;

		if (!mt_rr_chk(d, MT_MAC_CSR0, &v) && v != 0 && v != ~0u)
			return 1;
		mt_usleep(5000);
	}
	return 0;
}

/* Seed the TX power and confirm this really is an MT7612 before anything
 * writes to it. Shared by both open paths. */
static int mt_identify(struct mt7612u_dev *d, const char **err)
{
	/* 0.5 dB units, as mt76's txpower_conf = power_level * 2. 20 dBm is a
	 * conservative seed; mt7612u_set_txpower() overrides it. */
	if (!d->txpower_conf)
		d->txpower_conf = 40;

	d->rev = mt_rr(d, MT_ASIC_VERSION);
	if ((d->rev >> 16) != 0x7612) {
		if (err) *err = "not an MT7612 (unexpected MT_ASIC_VERSION)";
		return -1;
	}
	return 0;
}

/*
 * Adopt a handle the caller opened, reset and claimed interface 0 on. No
 * reset here: it would invalidate the caller's own handle. No detach either -
 * a caller that got this far already dealt with the kernel driver.
 */
int mt_adopt(struct mt7612u_dev *d, libusb_device_handle *h,
             libusb_context *ctx, const char **err)
{
	if (!h) { if (err) *err = "no USB handle"; return -1; }
	d->h = h;
	d->ctx = ctx;
	d->owns_handle = 0;
	d->kernel_was_attached = 0;
	return mt_identify(d, err);
}

/*
 * Open one MT7612U, honouring MT7612U_DEV when more than one is attached.
 *
 * libusb_open_device_with_vid_pid() returns whichever matching device
 * enumerates first, which is fine with one adapter and silently ambiguous
 * with two - a measurement then attributes itself to whichever unit the bus
 * happened to hand over. MT7612U_DEV takes a "bus-port" as lsusb and sysfs
 * spell it ("2-1"), or a bare index into the matches in enumeration order.
 */
/*
 * Exclusive per-adapter lock - the same lock devourer's own UsbDeviceLock
 * takes, deliberately byte-identical in key and path so the two contend.
 *
 * Two consumers on one MT7612U is not a race that resolves badly, it is the
 * one failure neither driver recovers from: the loser blocks inside a USB
 * ioctl in uninterruptible sleep, where SIGKILL does not reach it.
 * src/UsbDeviceLock.h describes the same symptom in the same words, which is
 * why this mirrors it rather than inventing a second scheme.
 *
 * That mirroring is the whole point. A lock file of our own would make
 * `bringup` and `rxdemo` invisible to each other and reproduce the wedge
 * across the two tools, which is exactly the case this is meant to stop:
 *   key   bus + USB port path, e.g. "3-1.4", with UsbDeviceLock's
 *         "-a<address>" fallback when the backend reports no port path
 *   path  "/tmp" + "/devourer-usb-" + key + ".lock"  (UsbDeviceLock's default)
 *   flags O_CREAT|O_RDWR|O_NOFOLLOW, 0666, then flock(LOCK_EX|LOCK_NB)
 *
 * Fail-open vs fail-closed follows UsbDeviceLock too: genuine contention
 * refuses, while an infrastructure failure (read-only tmpdir) warns and
 * proceeds, so a quirky environment never bricks an otherwise-working open.
 *
 * flock is released by the kernel on process death however it arrives, so
 * there are no stale locks to clean up - which matters precisely because
 * these processes sometimes die badly.
 */
static void adapter_key(libusb_device *dev, char *out, size_t n)
{
	uint8_t ports[8];
	int np = libusb_get_port_numbers(dev, ports, sizeof ports);
	int off = snprintf(out, n, "%u", libusb_get_bus_number(dev));

	if (np <= 0) {
		snprintf(out + off, n - (size_t)off, "-a%u",
		         libusb_get_device_address(dev));
		return;
	}
	for (int i = 0; i < np && off > 0 && (size_t)off < n; i++)
		off += snprintf(out + off, n - (size_t)off, "%s%u",
		                i ? "." : "-", ports[i]);
}

/* Returns a held fd, -1 to proceed unlocked (infrastructure failure), or
 * -2 when another process holds the adapter and the caller must refuse. */
static int lock_adapter(libusb_device *dev, const char **err)
{
	/* "/tmp" literally, and deliberately NOT getenv("TMPDIR"): the whole
	 * point of this lock is to contend with the C++ UsbDeviceLock, and that
	 * one takes its directory from DeviceConfig usb.lock_dir defaulting to
	 * "/tmp" (UsbDeviceLock.cpp:101) without ever consulting the
	 * environment. Reading TMPDIR here would put the two on different files
	 * whenever it is set, and the exclusion would lapse silently - which
	 * costs exactly what the comment below describes. A caller that sets a
	 * non-default usb.lock_dir is out of reach of this harness either way. */
	const char *dir = "/tmp";
	char key[64], path[256];
	int fd;

	adapter_key(dev, key, sizeof key);
	snprintf(path, sizeof path, "%s/devourer-usb-%s.lock", dir, key);

	/* O_NOFOLLOW: the path is world-writable and predictable, so a symlink
	 * planted there must not redirect the open. UsbDeviceLock does the same. */
	fd = open(path, O_CREAT | O_RDWR | O_NOFOLLOW | O_CLOEXEC, 0666);
	if (fd < 0) {
		WARN("cannot open lock file %s (%s) - proceeding without "
		    "exclusivity", path, strerror(errno));
		return -1;
	}
	if (flock(fd, LOCK_EX | LOCK_NB) == 0)
		return fd;
	close(fd);
	if (errno == EWOULDBLOCK || errno == EAGAIN) {
		if (err)
			*err = "adapter is already open in another process "
			       "(devourer or bringup); a second opener would wedge it "
			       "beyond SIGKILL";
		return -2;
	}
	WARN("cannot lock %s (%s) - proceeding without exclusivity",
	    path, strerror(errno));
	return -1;
}

static libusb_device_handle *open_selected(libusb_context *ctx, const char **err)
{
	const char *sel = getenv("MT7612U_DEV");
	libusb_device **list = NULL;
	libusb_device_handle *h = NULL;
	ssize_t n = libusb_get_device_list(ctx, &list);
	int matches = 0;

	if (n < 0) {
		if (err) *err = "libusb_get_device_list failed";
		return NULL;
	}

	for (ssize_t i = 0; i < n; i++) {
		struct libusb_device_descriptor desc;
		uint8_t ports[8];
		char id[32];
		int np, off;

		if (libusb_get_device_descriptor(list[i], &desc))
			continue;
		if (desc.idVendor != MT7612U_VID || desc.idProduct != MT7612U_PID)
			continue;

		np = libusb_get_port_numbers(list[i], ports, sizeof ports);
		off = snprintf(id, sizeof id, "%u", libusb_get_bus_number(list[i]));
		for (int j = 0; j < np && off > 0 && off < (int)sizeof id; j++)
			off += snprintf(id + off, sizeof id - (size_t)off, "%s%u",
			                j ? "." : "-", ports[j]);

		if (!sel || !*sel) {
			LOG("MT7612U at %s%s", id, matches ? "" : "  <- selected (first)");
		} else if (!strcmp(sel, id)) {
			LOG("MT7612U at %s  <- selected by MT7612U_DEV", id);
		} else {
			char idx[8];

			snprintf(idx, sizeof idx, "%d", matches);
			if (strcmp(sel, idx)) { matches++; continue; }
			LOG("MT7612U at %s  <- selected by MT7612U_DEV index %d", id, matches);
		}

		if (!h) {
			int lk = lock_adapter(list[i], err);

			if (lk == -2) {          /* held by someone else */
				libusb_free_device_list(list, 1);
				return NULL;
			}
			g_lock_fd = lk;
			if (libusb_open(list[i], &h))
				h = NULL;
		}
		matches++;
		if (h && sel && *sel)
			break;
	}

	if (matches > 1 && (!sel || !*sel))
		WARN("%d MT7612U adapters attached and MT7612U_DEV is unset - "
		    "using the first. Set MT7612U_DEV=<bus-port> to be explicit.",
		    matches);
	libusb_free_device_list(list, 1);
	if (!h && err)
		*err = matches ? "MT7612U found but could not be opened (try sudo)"
		               : "MT7612U not found";
	return h;
}

int mt_open(struct mt7612u_dev *d, const char **err)
{
	int rc;

	if (libusb_init(&d->ctx)) { if (err) *err = "libusb_init failed"; return -1; }
	d->owns_handle = 1;

	d->h = open_selected(d->ctx, err);
	if (!d->h) {
		libusb_exit(d->ctx); d->ctx = NULL;
		return -1;
	}

	d->kernel_was_attached = libusb_kernel_driver_active(d->h, 0) == 1;
	if (d->kernel_was_attached) {
		rc = libusb_detach_kernel_driver(d->h, 0);
		if (rc) {
			if (err) *err = "could not detach mt76x2u";
			goto fail;
		}
		LOG("detached kernel driver from interface 0");
	}

	/* A USB port reset before claiming. Without it the chip keeps whatever
	 * FCE/DMA state the previous run left behind, and the next firmware
	 * upload times out mid-chunk - reproducible after a few init cycles.
	 * devourer does the same thing on open for the same reason. */
	rc = libusb_reset_device(d->h);
	if (rc == LIBUSB_ERROR_NOT_FOUND) {
		/* Re-enumerated under a new address: reopen and re-detach. */
		libusb_close(d->h);
		mt_usleep(200000);
		d->h = open_selected(d->ctx, NULL);
		if (!d->h) {
			if (err) *err = "device vanished after USB reset";
			libusb_exit(d->ctx); d->ctx = NULL;
			return -1;
		}
		if (libusb_kernel_driver_active(d->h, 0) == 1)
			libusb_detach_kernel_driver(d->h, 0);
	} else if (rc) {
		WARN("USB reset returned %s", libusb_error_name(rc));
	}

	rc = libusb_claim_interface(d->h, 0);
	if (rc) {
		if (err) *err = "could not claim interface 0 (another process using it?)";
		goto fail;
	}

	if (mt_identify(d, err)) {
		libusb_release_interface(d->h, 0);
		goto fail;
	}
	return 0;

fail:
	if (d->kernel_was_attached)
		libusb_attach_kernel_driver(d->h, 0);
	libusb_close(d->h); d->h = NULL;
	libusb_exit(d->ctx); d->ctx = NULL;
	return -1;
}

void mt_close(struct mt7612u_dev *d)
{
	if (d->wrlog) { fclose(d->wrlog); d->wrlog = NULL; }
	if (d->mculog) { fclose(d->mculog); d->mculog = NULL; }
	if (d->transfers_stranded) {
		/* Deliberately leaks the handle and context. Transfers submitted
		 * on them are still owned by libusb with no event thread left to
		 * complete them; releasing the interface or closing underneath
		 * that is undefined, and a leaked handle on a process that is
		 * already tearing down is the cheaper failure. */
		ERR("close: transfers still owned by libusb - leaking the USB "
		    "handle and context rather than closing underneath them");
		d->h = NULL;
		d->ctx = NULL;
		if (g_lock_fd >= 0) { close(g_lock_fd); g_lock_fd = -1; }
		return;
	}
	if (d->h) {
		if (d->owns_handle) {
			libusb_release_interface(d->h, 0);
			if (d->kernel_was_attached && !d->keep_detached) {
				if (libusb_attach_kernel_driver(d->h, 0) == 0)
					LOG("reattached kernel driver");
			}
			libusb_close(d->h);
		}
		d->h = NULL;
	}
	if (d->ctx && d->owns_handle) libusb_exit(d->ctx);
	d->ctx = NULL;
	if (g_lock_fd >= 0) { close(g_lock_fd); g_lock_fd = -1; }
}

/* Block write, as mt76u_copy(): one MULTI_WRITE per batch, wValue 0.
 * The kernel uses this for the WCID address table (8 B) and the shared-key
 * table (32 B) - 192 transfers that would otherwise be ~700 4-byte writes. */
void mt_wr_copy(struct mt7612u_dev *d, uint32_t offset, const void *data, int len)
{
	const uint8_t *p = data;
	uint8_t buf[64];

	/* The hardware wants whole 32-bit words, but only `len` bytes belong to
	 * the caller. Round the *transfer* up and zero-fill the tail; rounding
	 * `len` up instead reads past the end of the caller's buffer. */
	for (int i = 0; i < len; ) {
		int n = len - i, xfer;

		if (n > (int)sizeof buf) n = (int)sizeof buf;
		xfer = (n + 3) & ~3;
		memset(buf, 0, (size_t)xfer);
		memcpy(buf, p + i, (size_t)n);
		if (mt_vendor_req(d, MT_VEND_MULTI_WRITE, REQ_OUT, 0,
		                  (uint16_t)(offset + i), buf, (size_t)xfer) < 0)
			return;
		i += n;
	}
}

/* --- public lifecycle helpers that belong with the transport --- */

void mt7612u_keep_detached(struct mt7612u_dev *d, int keep)
{
	if (d) d->keep_detached = keep;
}

uint32_t mt7612u_asic_version(const struct mt7612u_dev *d)
{
	return d ? d->rev : 0;
}

const uint8_t *mt7612u_mac_addr(const struct mt7612u_dev *d)
{
	return d ? d->macaddr : NULL;
}
