/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * libusb transport for the MT76x0U. Copied from src/mt7612u/usb.cpp (the
 * vendor-request register I/O and the synchronous-transfer machinery are
 * mt76x02 generation plumbing the two MACs share - mt76/usb.c serves both);
 * internal symbols renamed mt_ -> mt0_ so both subtrees link into one
 * library. What differs per chip - identify, adopt, close - is at the end.
 * The wire encoding was verified against usbmon on the MT7612U
 * (docs/mt7612u.md); it is mt76's.
 */
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include <atomic>
#include <new>
#include "internal.h"

/* See the LOG/WARN/ERR contract in internal.h. The whole line is formatted
 * first and emitted with one fwrite + fflush: two stdio calls could interleave
 * with a line from the libusb event thread, and an unflushed stderr can stall
 * a piped consumer mid-bring-up. Truncation is silent and deliberate - a
 * diagnostic is not worth a heap allocation on a path that may already be
 * failing. */
/* Set before threads start, read from the RX event thread; see the contract on
 * mt76x0u_set_log_sink() in the public header. */
static mt76x0u_log_sink g_log_sink;
static void            *g_log_user;

void mt76x0u_set_log_sink(mt76x0u_log_sink sink, void *user)
{
	g_log_sink = sink;
	g_log_user = user;
}

/* The built-in sink, and the only place this library names stderr or devourer's
 * line format. A host that installs its own sink gets the bare message and
 * applies its own prefix, so nothing double-prefixes. */
static void default_sink(void *user, char level, const char *line)
{
	char out[544];
	int n;

	(void)user;
	n = snprintf(out, sizeof out, "devourer [%c] mt76x0u: %s\n", level, line);
	if (n < 0)
		return;
	if ((size_t)n > sizeof out - 1)
		n = (int)(sizeof out - 1);
	/* One line, one fwrite + fflush: per-line atomicity against the event
	 * thread, and no pipe-buffering stall for a subprocess supervisor. Same
	 * reasoning as devourer's Logger::emit and src/Event.h. */
	fwrite(out, 1, (size_t)n, stderr);
	fflush(stderr);
}

void mt0_diag(char level, const char *fmt, ...)
{
	char msg[512];
	int n;
	va_list ap;

	va_start(ap, fmt);
	n = vsnprintf(msg, sizeof msg, fmt, ap);
	va_end(ap);
	if (n < 0)
		return;
	/* vsnprintf returns what it WOULD have written; the buffer is already
	 * NUL-terminated at the truncation point, so nothing more is needed. */

	if (g_log_sink)
		g_log_sink(g_log_user, level, msg);
	else
		default_sink(NULL, level, msg);
}

/* Each operand is cast to the uint8_t that libusb's bmRequestType actually is:
 * these are three DIFFERENT libusb enum types, and C++20 deprecates a bitwise
 * operation between different enumeration types ([depr.ee.conv]). Casting keeps
 * the value identical while making the operation an ordinary integer OR. */
#define REQ_IN   ((uint8_t)LIBUSB_ENDPOINT_IN  | (uint8_t)LIBUSB_REQUEST_TYPE_VENDOR | \
                  (uint8_t)LIBUSB_RECIPIENT_DEVICE)
#define REQ_OUT  ((uint8_t)LIBUSB_ENDPOINT_OUT | (uint8_t)LIBUSB_REQUEST_TYPE_VENDOR | \
                  (uint8_t)LIBUSB_RECIPIENT_DEVICE)
/* mt76's MT_VEND_REQ_TOUT_MS / MT_VEND_REQ_MAX_RETRY. The product of the two
 * is the worst-case cost of one register access, so it bounds every poll
 * loop below - which is why the timeout is 300 ms and not something longer. */
#define CTRL_TIMEOUT_MS 300
#define VEND_RETRIES 10

void mt0_usleep(unsigned us)
{
	/* Not interruptible, where nanosleep(&ts, NULL) was: libstdc++ retries
	 * sleep_for on EINTR, so a signal no longer cuts the wait short (measured:
	 * a 200 ms request under 100 Hz SIGALRM returned after 10 ms before, 201 ms
	 * now). That is the behaviour this call wants — every use is a hardware
	 * settle or poll interval, and a delivered signal is not a reason for the
	 * chip to be ready sooner. Worth knowing if a caller ever wants to
	 * interrupt a long bring-up. */
	std::this_thread::sleep_for(std::chrono::microseconds(us));
}

static uint64_t now_us(void)
{
	/* steady_clock, matching CLOCK_MONOTONIC: never stepped by a wall-clock
	 * adjustment, which is what an elapsed-time measurement needs. */
	return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(
	           std::chrono::steady_clock::now().time_since_epoch()).count();
}

/*
 * Synchronous transfers, built on libusb's async API.
 *
 * libusb's own libusb_control_transfer()/libusb_bulk_transfer() are not used
 * here because this library runs a second event thread (async.cpp) on the same
 * context. libusb's sync layer decides the transfer is over by reading its
 * completion flag with no synchronization against the thread that set it -
 * on the path where handle_events returns early for an expired timeout there
 * is no lock between the event thread's last touch of the transfer and the
 * caller's libusb_free_transfer(), which destroys the transfer's mutex. On
 * x86 program order hides it; on the ARM hosts this library is meant for
 * nothing does. ThreadSanitizer reports it against real hardware whenever a
 * register access runs while the RX ring is up (a retune, the 1 Hz tick).
 *
 * So: submit through the async API, and wait on our own flag with
 * acquire/release semantics. The callback's release store happens after
 * everything libusb did with the transfer on the event thread, and the
 * acquire load below orders the free after it. While waiting, this thread
 * pumps events itself when no one else is (a caller with no RX ring), or
 * parks as an event waiter when the ring's thread holds the events lock -
 * the same two behaviours the sync API has, minus the unordered read.
 *
 * A transfer that has not completed is never freed, and its callback never
 * outlives its state: everything the callback can touch - the transfer, its
 * buffer and the completion word - lives on the heap in one sync_xfer that
 * the LAST of the two parties to arrive frees. Past the transfer's own timeout
 * plus a margin the waiter cancels it; if even the cancellation does not come
 * back it hands ownership to the callback (state ABANDONED) and returns, and
 * the device is marked stranded so mt0_close() leaks the USB handle and context
 * instead of closing underneath a transfer libusb still owns - the same policy
 * the async ring already has.
 */
namespace {
enum { XFER_PENDING = 0, XFER_DONE = 1, XFER_ABANDONED = 2 };

struct sync_xfer {
	std::atomic<int> state{XFER_PENDING};
	struct mt76x0u_dev *d = nullptr;
	struct libusb_transfer *t = nullptr;
	unsigned char *buf = nullptr;   /* heap copy the transfer reads/writes */
};

/* Return the transfer to the device pool (see mt76x0u_dev::sync_pool) if
 * there is room, else free it. Never called on an in-flight transfer. */
void sync_xfer_free(struct sync_xfer *x)
{
	free(x->buf);
	bool pooled = false;
	{
		std::lock_guard<std::mutex> lk(x->d->sync_pool_mu);
		if (x->d->sync_pool_n < MT_SYNC_POOL) {
			x->d->sync_pool[x->d->sync_pool_n++] = x->t;
			pooled = true;
		}
	}
	if (!pooled)
		libusb_free_transfer(x->t);
	delete x;
}

void LIBUSB_CALL sync_done(struct libusb_transfer *t)
{
	auto *x = static_cast<struct sync_xfer *>(t->user_data);
	/* Release: everything libusb did with the transfer on this thread is
	 * ordered before the waiter's acquire load. If the waiter has already
	 * given up, this callback is the last party and frees. */
	if (x->state.exchange(XFER_DONE, std::memory_order_acq_rel) ==
	    XFER_ABANDONED)
		sync_xfer_free(x);
}

int status_to_rc(enum libusb_transfer_status st)
{
	switch (st) {
	case LIBUSB_TRANSFER_COMPLETED: return 0;
	case LIBUSB_TRANSFER_TIMED_OUT: return LIBUSB_ERROR_TIMEOUT;
	case LIBUSB_TRANSFER_STALL:     return LIBUSB_ERROR_PIPE;
	case LIBUSB_TRANSFER_NO_DEVICE: return LIBUSB_ERROR_NO_DEVICE;
	case LIBUSB_TRANSFER_OVERFLOW:  return LIBUSB_ERROR_OVERFLOW;
	default:                        return LIBUSB_ERROR_IO;
	}
}

/* Allocate a sync_xfer with a `len`-byte heap buffer. NULL on allocation
 * failure. */
struct sync_xfer *sync_xfer_new(struct mt76x0u_dev *d, size_t len)
{
	auto *x = new (std::nothrow) sync_xfer;
	if (!x)
		return nullptr;
	x->d = d;
	{
		std::lock_guard<std::mutex> lk(d->sync_pool_mu);
		if (d->sync_pool_n > 0)
			x->t = d->sync_pool[--d->sync_pool_n];
	}
	if (!x->t)
		x->t = libusb_alloc_transfer(0);
	x->buf = (unsigned char *)malloc(len ? len : 1);
	if (!x->t || !x->buf) {
		free(x->buf);
		libusb_free_transfer(x->t);
		delete x;
		return nullptr;
	}
	/* A pooled transfer keeps its last actual_length, and libusb writes the
	 * field only when a transfer completes - not when libusb_submit_transfer()
	 * fails, as it does once the device has left the bus. sync_bulk() reads
	 * it on every return but an abandon, so after a failed submit an IN read
	 * reported the previous transfer's length and copied that many bytes from
	 * its len-byte bounce buffer into the caller's: a 1428 B TX, the adapter
	 * deauthorized, then a 64 B read wrote 1364 B past the end. */
	x->t->actual_length = 0;
	return x;
}

/* Submit x->t (already filled with callback sync_done and user_data x) and
 * wait for it. Returns a LIBUSB_ERROR_* code, 0 on completion; on any return
 * but LIBUSB_ERROR_OTHER the transfer is complete, still owned by the caller,
 * and may be read then freed with sync_xfer_free(). LIBUSB_ERROR_OTHER means
 * the transfer was abandoned to its callback: the caller owns nothing. */
int submit_and_wait(struct mt76x0u_dev *d, struct sync_xfer *x,
                    unsigned timeout_ms)
{
	int rc = libusb_submit_transfer(x->t);
	if (rc)
		return rc;

	const auto cancel_at = std::chrono::steady_clock::now() +
	                       std::chrono::milliseconds(timeout_ms) +
	                       std::chrono::seconds(2);
	const auto give_up_at = cancel_at + std::chrono::seconds(2);
	bool cancelled = false;

	while (x->state.load(std::memory_order_acquire) != XFER_DONE) {
		struct timeval tv = { 0, 20000 };
		int r = libusb_handle_events_timeout_completed(d->ctx, &tv, NULL);
		if (r < 0 && r != LIBUSB_ERROR_INTERRUPTED)
			mt0_usleep(1000);   /* a dead context: don't spin flat out */
		const auto now = std::chrono::steady_clock::now();
		if (!cancelled && now > cancel_at) {
			libusb_cancel_transfer(x->t);
			cancelled = true;
		} else if (cancelled && now > give_up_at) {
			ERR("transfer on ep %02x never completed after cancel: "
			    "abandoning it to its callback and stranding the device",
			    x->t->endpoint);
			d->transfers_stranded = 1;
			/* The callback may have landed between the loop test and
			 * here; then WE are the last party and free. */
			if (x->state.exchange(XFER_ABANDONED,
			                      std::memory_order_acq_rel) == XFER_DONE)
				sync_xfer_free(x);
			return LIBUSB_ERROR_OTHER;
		}
	}
	return status_to_rc(x->t->status);
}

/* libusb_control_transfer(), over submit_and_wait(). Same contract: the
 * number of data bytes transferred on success, a LIBUSB_ERROR_* otherwise. */
int sync_control(struct mt76x0u_dev *d, uint8_t type, uint8_t req,
                 uint16_t val, uint16_t idx, unsigned char *data,
                 uint16_t len, unsigned timeout_ms)
{
	const bool out = (type & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_OUT;
	struct sync_xfer *x = sync_xfer_new(d, LIBUSB_CONTROL_SETUP_SIZE + len);
	if (!x)
		return LIBUSB_ERROR_NO_MEM;
	libusb_fill_control_setup(x->buf, type, req, val, idx, len);
	if (out && len)
		memcpy(x->buf + LIBUSB_CONTROL_SETUP_SIZE, data, len);
	libusb_fill_control_transfer(x->t, d->h, x->buf, sync_done, x,
	                             timeout_ms);
	x->t->flags = 0;

	int rc = submit_and_wait(d, x, timeout_ms);
	if (rc == LIBUSB_ERROR_OTHER)
		return rc;
	if (rc == 0) {
		rc = x->t->actual_length;
		if (!out && rc > 0)
			memcpy(data, x->buf + LIBUSB_CONTROL_SETUP_SIZE, (size_t)rc);
	}
	sync_xfer_free(x);
	return rc;
}

/* libusb_bulk_transfer(), over submit_and_wait(). *xfered is filled on a
 * timeout too, as libusb's is. The caller's buffer is copied both ways so
 * that an abandoned transfer holds no pointer into the caller's frame. */
int sync_bulk(struct mt76x0u_dev *d, uint8_t ep, unsigned char *buf, int len,
              int *xfered, unsigned timeout_ms)
{
	const bool in = (ep & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN;
	struct sync_xfer *x = sync_xfer_new(d, (size_t)len);
	if (!x)
		return LIBUSB_ERROR_NO_MEM;
	if (!in && len)
		memcpy(x->buf, buf, (size_t)len);
	libusb_fill_bulk_transfer(x->t, d->h, ep, x->buf, len, sync_done, x,
	                          timeout_ms);
	x->t->flags = 0;

	int rc = submit_and_wait(d, x, timeout_ms);
	if (rc == LIBUSB_ERROR_OTHER) {
		if (xfered) *xfered = 0;
		return rc;
	}
	const int n = x->t->actual_length;
	if (in && n > 0)
		memcpy(buf, x->buf, (size_t)n);
	if (xfered)
		*xfered = n;
	sync_xfer_free(x);
	return rc;
}
} /* namespace */

int mt0_vendor_req(struct mt76x0u_dev *d, uint8_t req, uint8_t type,
                  uint16_t val, uint16_t idx, void *buf, size_t len)
{
	int rc = LIBUSB_ERROR_OTHER;

	/* One control transfer at a time.  Nothing contends in a single-threaded
	 * consumer; one that sends from a second thread would otherwise
	 * interleave two transfers on EP0. */
	d->io_lock.lock();

	const int tries = d->ep0_tries > 0 ? d->ep0_tries : VEND_RETRIES;

	for (int i = 0; i < tries; i++) {
		rc = d->io_hook
		   ? d->io_hook->ctrl(d->io_hook->user, type, req, val, idx,
		                      (uint8_t *)buf, (uint16_t)len)
		   : sync_control(d, type, req, val, idx, (unsigned char *)buf,
		                  (uint16_t)len, CTRL_TIMEOUT_MS);
		if (rc >= 0 || rc == LIBUSB_ERROR_NO_DEVICE)
			goto out;
		mt0_usleep(5000);
	}
	if (!d->ep0_quiet)
		ERR("vendor req %02x idx %04x failed: %s", req, idx,
		    libusb_error_name(rc));
out:
	d->io_lock.unlock();
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
int mt0_rr_chk(struct mt76x0u_dev *d, uint32_t addr, uint32_t *val)
{
	uint8_t req = rd_req(addr), b[4] = { 0 };
	uint32_t a = addr & ~MT_VEND_TYPE_MASK;

	if (mt0_vendor_req(d, req, REQ_IN, (uint16_t)(a >> 16), (uint16_t)a,
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
uint32_t mt0_rr(struct mt76x0u_dev *d, uint32_t addr)
{
	uint32_t v;

	return mt0_rr_chk(d, addr, &v) ? ~0u : v;
}

void mt0_wr(struct mt76x0u_dev *d, uint32_t addr, uint32_t val)
{
	uint8_t req = wr_req(addr), b[4];
	uint32_t a = addr & ~MT_VEND_TYPE_MASK;

	b[0] = val & 0xff; b[1] = (val >> 8) & 0xff;
	b[2] = (val >> 16) & 0xff; b[3] = (val >> 24) & 0xff;
	if (mt0_vendor_req(d, req, REQ_OUT, (uint16_t)(a >> 16), (uint16_t)a,
	                  b, sizeof b) != (int)sizeof b)
		d->io_err++;

	/* Oracle-diff log: same shape decode.py renders from usbmon. */
	if (d->wrlog)
		fprintf(d->wrlog, "req=0x%02x addr=0x%04x data=%02x%02x%02x%02x\n",
		        req, (unsigned)(a & 0xffff), b[0], b[1], b[2], b[3]);
}

/*
 * Read-modify-write. A failed read MUST NOT be written back: mt0_rr's ~0u would
 * turn the operation into "set every bit", and the addresses this is used on
 * (MT_WLAN_FUN_CTRL, MT_MAC_SYS_CTRL, the BBP AGC block) are exactly the ones
 * where that is destructive. Returns 0 on success, -1 if nothing was written.
 */
/*
 * Register-I/O failures accumulate per device rather than being returned from
 * every accessor.
 *
 * mt0_wr() discarded mt0_vendor_req()'s result entirely, so a write that
 * exhausted its retries mid-bring-up left the hardware partly configured while
 * the public call still returned success. Threading a status through every
 * writer would touch several hundred call sites in an initialisation sequence
 * that is deliberately a verbatim port of mt76's, and that churn would bury
 * the thing it is meant to protect.
 *
 * So this follows the shape the reads already had - mt0_rr_chk() has always
 * bumped this same counter. Writes stay best-effort at the call site, and a
 * SEQUENCE checks the accumulator at its boundary: mt0_io_clear() on entry,
 * mt0_io_errors() on exit, and the whole setup fails if any access failed.
 * Optional or diagnostic writes stay best-effort by not being bracketed.
 */
void mt0_io_clear(struct mt76x0u_dev *d)      { d->io_err = 0; }
/* Put the accumulator back to a value taken earlier.  For a nested retry that
 * must discard only its OWN failed attempt: there is one accumulator, and the
 * caller's bracket may already have counted failures before the retrying code
 * was reached, so zeroing would silently forgive those too. */
void mt0_io_restore(struct mt76x0u_dev *d, unsigned v) { d->io_err = v; }
unsigned mt0_io_errors(struct mt76x0u_dev *d) { return d->io_err; }

/* Checked single write, for a caller that wants to fail at the write rather
 * than at a sequence boundary. */
int mt0_wr_chk(struct mt76x0u_dev *d, uint32_t addr, uint32_t val)
{
	unsigned before = d->io_err;

	mt0_wr(d, addr, val);
	return d->io_err == before ? 0 : -1;
}

int mt0_rmw(struct mt76x0u_dev *d, uint32_t addr, uint32_t mask, uint32_t val)
{
	uint32_t cur;

	if (mt0_rr_chk(d, addr, &cur)) {
		ERR("rmw 0x%05x skipped: read failed", addr & ~MT_VEND_TYPE_MASK);
		return -1;
	}
	mt0_wr(d, addr, (cur & ~mask) | val);
	return 0;
}

/*
 * Poll against a real deadline rather than a count of sleeps. One register
 * access can itself cost up to VEND_RETRIES * CTRL_TIMEOUT_MS, so counting
 * iterations would let a caller asking for 200 ms block for seconds.
 */
int mt0_poll(struct mt76x0u_dev *d, uint32_t addr, uint32_t mask,
            uint32_t val, int timeout_us)
{
	uint64_t deadline = now_us() + (uint64_t)(timeout_us < 0 ? 0 : timeout_us);

	for (;;) {
		uint32_t cur;

		if (mt0_rr_chk(d, addr, &cur)) {
			ERR("poll 0x%05x aborted: read failed",
			    addr & ~MT_VEND_TYPE_MASK);
			return 0;
		}
		if ((cur & mask) == val)
			return 1;
		if (now_us() >= deadline)
			return 0;
		mt0_usleep(1000);
	}
}

void mt0_single_wr(struct mt76x0u_dev *d, uint8_t req, uint16_t off, uint32_t val)
{
	if (mt0_vendor_req(d, req, REQ_OUT, (uint16_t)(val & 0xffff), off, NULL, 0) < 0)
		d->io_err++;
	if (mt0_vendor_req(d, req, REQ_OUT, (uint16_t)(val >> 16),
	                  (uint16_t)(off + 2), NULL, 0) < 0)
		d->io_err++;
}

int mt0_bulk(struct mt76x0u_dev *d, uint8_t ep, void *buf, int len,
            int *xfered, unsigned timeout_ms)
{
	int n = 0;
	int rc = d->io_hook
	       ? d->io_hook->bulk(d->io_hook->user, ep, (uint8_t *)buf, len, &n, timeout_ms)
	       : sync_bulk(d, ep, (unsigned char *)buf, len, &n, timeout_ms);
	if (xfered) *xfered = n;
	return rc;
}

/* mt76x02_wait_for_mac(): MAC_CSR0 reads 0 or ~0 until the core is alive.
 * Both are legitimate values here, which is why this uses the checked read -
 * a transport failure is a different condition from "still coming up". */
int mt0_wait_for_mac(struct mt76x0u_dev *d)
{
	for (int i = 0; i < 500; i++) {
		uint32_t v;

		if (!mt0_rr_chk(d, MT_MAC_CSR0, &v) && v != 0 && v != ~0u)
			return 1;
		mt0_usleep(5000);
	}
	return 0;
}

/* --- MT76x0U-specific: identify, adopt, close ----------------------------- */

/*
 * mt76x02.h is_mt76x0(): mt76_chip() (MT_ASIC_VERSION >> 16) is 0x7610, 0x7630
 * or 0x7650. Checked read: 0xffffffff is a legitimate register value on this
 * family (MT_MAC_CSR0 reads it while the core comes up), so a failed transfer
 * must not be mistaken for a chip id.
 */
static int mt0_identify(struct mt76x0u_dev *d, const char **err)
{
	uint32_t rev;

	if (mt0_rr_chk(d, MT_ASIC_VERSION, &rev)) {
		if (err) *err = "MT_ASIC_VERSION read failed";
		return -1;
	}
	d->rev = rev;
	const uint16_t chip = (uint16_t)(rev >> 16);
	if (chip != 0x7610 && chip != 0x7630 && chip != 0x7650) {
		if (err) *err = "not an mt76x0 (unexpected MT_ASIC_VERSION)";
		return -1;
	}
	d->is_mt7630 = chip == 0x7630;
	/* Informational: the kernel prints it beside ASIC_VERSION. It may read
	 * 0 or ~0 until the MAC is up, which bring-up waits for (phase 3). */
	if (mt0_rr_chk(d, MT_MAC_CSR0, &d->mac_rev))
		d->mac_rev = 0;
	LOG("ASIC revision %08x, MAC revision %08x%s", d->rev, d->mac_rev,
	    d->is_mt7630 ? " (MT7630 combo strap)" : "");
	return 0;
}

/* The sync helpers' transfer pool has to exist before any event thread does;
 * see mt7612u_dev::sync_pool for the ThreadSanitizer reasoning. */
void mt0_dev_state_init(struct mt76x0u_dev *d)
{
	std::lock_guard<std::mutex> lk(d->sync_pool_mu);
	while (d->sync_pool_n < MT_SYNC_POOL) {
		struct libusb_transfer *t = libusb_alloc_transfer(0);
		if (!t)
			break;
		d->sync_pool[d->sync_pool_n++] = t;
	}
}

void mt0_dev_state_destroy(struct mt76x0u_dev *d)
{
	std::lock_guard<std::mutex> lk(d->sync_pool_mu);
	while (d->sync_pool_n > 0)
		libusb_free_transfer(d->sync_pool[--d->sync_pool_n]);
}

/*
 * Adopt a handle the caller opened, reset and claimed interface 0 on. No reset
 * here (it would invalidate the caller's handle) and no kernel-driver detach
 * (the caller dealt with it).
 *
 * Deliberately NOT here yet: the kernel's probe-time sequence
 * (mt76x0u_probe: chip off before wait_for_mac, "otherwise MCU fail to
 * initialize on hot reboot"; then chip on with reset, and skip the firmware
 * upload when it is already running). That is the warm-start path, and it
 * belongs to bring-up (phase 3), which owns MT_WLAN_FUN_CTRL.
 */
int mt0_adopt(struct mt76x0u_dev *d, libusb_device_handle *h,
              libusb_context *ctx, const char **err)
{
	if (!h) { if (err) *err = "no USB handle"; return -1; }
	/* Before mt0_identify(): it reads a register, which locks io_lock. */
	mt0_dev_state_init(d);
	d->h = h;
	d->ctx = ctx;
	return mt0_identify(d, err);
}

/* The caller owns the handle and the context (the adopt path is the only one),
 * so this releases neither. */
void mt0_close(struct mt76x0u_dev *d)
{
	mt0_dev_state_destroy(d);
	if (d->wrlog) { fclose(d->wrlog); d->wrlog = NULL; }
	if (d->mculog) { fclose(d->mculog); d->mculog = NULL; }
	if (d->transfers_stranded)
		ERR("close: transfers still owned by libusb - the caller must not "
		    "release the interface or close the handle under them");
	d->h = NULL;
	d->ctx = NULL;
}

/*
 * Drain EP 4 (0x84): reads until an empty read, any error (a timeout
 * included), or *reads_left reads (64 when reads_left is null). Returns the
 * bytes drained. A timed-out read reporting a full buffer is not counted: on
 * one host (libusb 1.0.30, xhci) an idle EP reports n == len on timeout with
 * no data. Copied from mt7612u's mt_rx_flush, which documents the measurement.
 */
size_t mt0_rx_flush(struct mt76x0u_dev *d, int *reads_left)
{
	uint8_t buf[4096];
	size_t total = 0;
	int own = 64;
	int *left = reads_left ? reads_left : &own;

	while (*left > 0) {
		int n = 0;
		int rc = mt0_bulk(d, MT_EP_IN_PKT_RX, buf, sizeof buf, &n, 10);

		(*left)--;
		if (n > 0 && !(rc == LIBUSB_ERROR_TIMEOUT && n == (int)sizeof buf))
			total += (size_t)n;
		if (rc || n == 0)
			break;
	}
	return total;
}

/* Block write, as mt76u_copy(): one MULTI_WRITE per batch, wValue 0. */
void mt0_wr_copy(struct mt76x0u_dev *d, uint32_t offset, const void *data, int len)
{
	const uint8_t *p = (const uint8_t *)data;
	uint8_t buf[64];

	/* Round the TRANSFER up to whole words and zero-fill the tail; rounding
	 * `len` would read past the caller's buffer. */
	for (int i = 0; i < len; ) {
		int n = len - i, xfer;

		if (n > (int)sizeof buf) n = (int)sizeof buf;
		xfer = (n + 3) & ~3;
		memset(buf, 0, (size_t)xfer);
		memcpy(buf, p + i, (size_t)n);
		if (mt0_vendor_req(d, MT_VEND_MULTI_WRITE, REQ_OUT, 0,
		                   (uint16_t)(offset + i), buf, (size_t)xfer) < 0)
			return;
		i += n;
	}
}

/* --- public lifecycle --------------------------------------------------- */

struct mt76x0u_dev *mt0_open_hooked(const struct mt0_io_hook *hook, int no_2ghz,
                                    const char **err)
{
	struct mt76x0u_dev *d = NULL;

	try {
		d = new mt76x0u_dev{};
	} catch (...) {
		d = NULL;
	}
	if (!d) {
		if (err) *err = "out of memory";
		return NULL;
	}
	d->io_hook = hook;
	d->no_2ghz = no_2ghz ? 1 : 0;
	/* Any non-NULL handle: with the hook set, nothing dereferences it. */
	if (mt0_adopt(d, (libusb_device_handle *)hook, NULL, err)) {
		mt0_close(d);
		delete d;
		return NULL;
	}
	return d;
}

struct mt76x0u_dev *mt76x0u_open_handle(void *h, void *ctx, int no_2ghz,
                                        const char **err)
{
	struct mt76x0u_dev *d = NULL;

	/* nothrow does not cover a member constructor that allocates, and
	 * nothing may throw across the extern "C" boundary. */
	try {
		d = new mt76x0u_dev{};
	} catch (...) {
		d = NULL;
	}
	if (!d) {
		if (err) *err = "out of memory";
		return NULL;
	}
	d->no_2ghz = no_2ghz ? 1 : 0;
	if (mt0_adopt(d, (libusb_device_handle *)h, (libusb_context *)ctx, err)) {
		mt0_close(d);
		delete d;
		return NULL;
	}
	return d;
}

void mt76x0u_close(struct mt76x0u_dev *d)
{
	if (!d) return;
	mt0_async_stop(d);
	mt0_close(d);
	/* A stranded ring is leaked by mt0_async_stop() with its slots detached
	 * from `d`, so freeing the device here is safe either way. */
	delete d;
}

uint32_t mt76x0u_asic_version(const struct mt76x0u_dev *d) { return d ? d->rev : 0; }
uint32_t mt76x0u_mac_version(const struct mt76x0u_dev *d) { return d ? d->mac_rev : 0; }
int mt76x0u_is_mt7630(const struct mt76x0u_dev *d) { return d ? d->is_mt7630 : 0; }
int mt76x0u_no_2ghz(const struct mt76x0u_dev *d) { return d ? d->no_2ghz : 0; }
