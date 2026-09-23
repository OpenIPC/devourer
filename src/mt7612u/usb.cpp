/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * libusb transport for MT7612U. Replaces mt76/usb.c's vendor-request and URB
 * plumbing; the wire encoding is identical (verified against usbmon, see
 * ../../INVESTIGATION.md §11).
 */
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
/* The adapter lock below is POSIX file locking. MSVC has none of these headers;
 * see lock_adapter() for what Windows does instead. */
#if !defined(_WIN32)
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#endif

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
 * mt7612u_set_log_sink() in the public header. */
static mt7612u_log_sink g_log_sink;
static void            *g_log_user;

void mt7612u_set_log_sink(mt7612u_log_sink sink, void *user)
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
	n = snprintf(out, sizeof out, "devourer [%c] mt7612u: %s\n", level, line);
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

void mt_diag(char level, const char *fmt, ...)
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

void mt_usleep(unsigned us)
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
 * the device is marked stranded so mt_close() leaks the USB handle and context
 * instead of closing underneath a transfer libusb still owns - the same policy
 * the async ring already has.
 */
namespace {
enum { XFER_PENDING = 0, XFER_DONE = 1, XFER_ABANDONED = 2 };

struct sync_xfer {
	std::atomic<int> state{XFER_PENDING};
	struct mt7612u_dev *d = nullptr;
	struct libusb_transfer *t = nullptr;
	unsigned char *buf = nullptr;   /* heap copy the transfer reads/writes */
};

/* Return the transfer to the device pool (see mt7612u_dev::sync_pool) if
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
struct sync_xfer *sync_xfer_new(struct mt7612u_dev *d, size_t len)
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
	return x;
}

/* Submit x->t (already filled with callback sync_done and user_data x) and
 * wait for it. Returns a LIBUSB_ERROR_* code, 0 on completion; on any return
 * but LIBUSB_ERROR_OTHER the transfer is complete, still owned by the caller,
 * and may be read then freed with sync_xfer_free(). LIBUSB_ERROR_OTHER means
 * the transfer was abandoned to its callback: the caller owns nothing. */
int submit_and_wait(struct mt7612u_dev *d, struct sync_xfer *x,
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
			mt_usleep(1000);   /* a dead context: don't spin flat out */
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
int sync_control(struct mt7612u_dev *d, uint8_t type, uint8_t req,
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
int sync_bulk(struct mt7612u_dev *d, uint8_t ep, unsigned char *buf, int len,
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

int mt_vendor_req(struct mt7612u_dev *d, uint8_t req, uint8_t type,
                  uint16_t val, uint16_t idx, void *buf, size_t len)
{
	int rc = LIBUSB_ERROR_OTHER;

	/* One control transfer at a time.  Nothing contends in a single-threaded
	 * consumer; one that sends from a second thread would otherwise
	 * interleave two transfers on EP0. */
	d->io_lock.lock();

	for (int i = 0; i < VEND_RETRIES; i++) {
		rc = sync_control(d, type, req, val, idx, (unsigned char *)buf,
		                  (uint16_t)len, CTRL_TIMEOUT_MS);
		if (rc >= 0 || rc == LIBUSB_ERROR_NO_DEVICE)
			goto out;
		mt_usleep(5000);
	}
	ERR("vendor req %02x idx %04x failed: %s", req, idx, libusb_error_name(rc));
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
/* Put the accumulator back to a value taken earlier.  For a nested retry that
 * must discard only its OWN failed attempt: there is one accumulator, and the
 * caller's bracket may already have counted failures before the retrying code
 * was reached, so zeroing would silently forgive those too. */
void mt_io_restore(struct mt7612u_dev *d, unsigned v) { d->io_err = v; }
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
	int rc = sync_bulk(d, ep, (unsigned char *)buf, len, &n, timeout_ms);
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
void mt_dev_state_init(struct mt7612u_dev *d)
{
	/* io_lock is a std::recursive_mutex member, constructed with the device,
	 * so it needs nothing here. What does: the sync helpers' transfer pool,
	 * which has to exist before any event thread does (see the field). An
	 * allocation failure leaves the pool short and the helpers allocate per
	 * call instead - the cal sentinels (low_gain=-1 etc.) are reset per-tune
	 * in mt_set_channel_ex(), which always runs before the first PHY tick. */
	std::lock_guard<std::mutex> lk(d->sync_pool_mu);
	while (d->sync_pool_n < MT_SYNC_POOL) {
		struct libusb_transfer *t = libusb_alloc_transfer(0);
		if (!t)
			break;
		d->sync_pool[d->sync_pool_n++] = t;
	}
}

void mt_dev_state_destroy(struct mt7612u_dev *d)
{
	std::lock_guard<std::mutex> lk(d->sync_pool_mu);
	while (d->sync_pool_n > 0)
		libusb_free_transfer(d->sync_pool[--d->sync_pool_n]);
}

/*
 * Recover from a previous run that died mid transfer.  A USB port reset does
 * not reach any of this: afterwards register reads and writes still round-trip
 * and the MAC and RF are fine, but every bulk OUT NAKs and the next firmware
 * upload times out at its first chunk - the kernel mt76x2u driver cannot bind
 * such a device either.  Two tiers, told apart by MT_USB_U3DMA_CFG:
 *
 *   0x00c00020, TX_BUSY clear: isolated one step at a time - clear_halt on all
 *   four endpoints, MAC + USB DMA stop, WLAN_EN/WLAN_CLK_EN down and the PBF
 *   block reset each left it wedged; pulsing TX_CLR alone clears it (3/3).
 *   mt76 declares TX_CLR and writes it nowhere.
 *
 *   0x80c00020, TX_BUSY stuck: the pulse loop below has NOT been seen to clear
 *   it, nor has anything else tried so far; the vendor-derived UDMA/IFDMA reset
 *   sequence in docs/mt7612u-usb-wedge.md (bringup swreset) is the untested
 *   candidate.  The WARN is the honest verdict.
 *
 * The receive direction gets its own clean-up: mt_rx_flush() runs from
 * mt_mac_stop(), which a killed process never reaches.  Silence the receiver
 * BEFORE draining or it refills as fast as it is read.
 *
 * This runs on BOTH open paths.  It lived in mt_open() alone, which left the
 * consumer this subtree exists for - a libusb-owning caller arriving through
 * mt7612u_open_handle() / mt_adopt() - paying both mt_fw_init() attempts and
 * failing with the exact error the recovery removes.
 */
void mt_recover_usb(struct mt7612u_dev *d)
{
	uint32_t cfg;

	/* A wedge experiment must not have its recovery hidden inside open().
	 * With this set, open() observes and reports but repairs nothing.  It is
	 * a field rather than a getenv(): the tool that wants the behaviour sets
	 * it (bringup does, from MT7612U_NO_AUTORECOVER).
	 *
	 * The same is now true of adapter selection: d->dev_selector replaced the
	 * getenv("MT7612U_DEV") that open_selected() used to read, and
	 * mt7612u_open_selected() is the public way to pass it. This library reads
	 * no environment at all. */
	if (d->no_autorecover) {
		if (mt_rr_chk(d, CFG_ADDR(MT_USB_U3DMA_CFG), &cfg))
			LOG("auto-recovery disabled: U3DMA_CFG unreadable");
		else
			LOG("auto-recovery disabled: U3DMA_CFG=0x%08x", cfg);
		return;
	}

	mt_wr(d, MT_MAC_SYS_CTRL, 0);
	mt_rx_flush(d);

	/* Checked read: mt_rr() reports a failed control transfer as ~0u, which
	 * has TX_BUSY set.  One EP0 hiccup would otherwise send a healthy adapter
	 * through TX_BULK_EN off, twenty TX_CLR pulses (400 ms) and a false "still
	 * busy" verdict.  A read we cannot trust is not evidence of a wedge. */
	if (mt_rr_chk(d, CFG_ADDR(MT_USB_U3DMA_CFG), &cfg)) {
		WARN("U3DMA_CFG unreadable on open - skipping wedge recovery");
		return;
	}

	if (cfg & MT_USB_DMA_CFG_TX_BUSY) {
		WARN("USB TX DMA busy on open - a previous run died mid transfer");
		mt_clear(d, CFG_ADDR(MT_USB_U3DMA_CFG), MT_USB_DMA_CFG_TX_BULK_EN);
		for (int i = 0; i < 20; i++) {
			mt_set(d, CFG_ADDR(MT_USB_U3DMA_CFG), MT_USB_DMA_CFG_TX_CLR);
			mt_usleep(20000);
			mt_clear(d, CFG_ADDR(MT_USB_U3DMA_CFG), MT_USB_DMA_CFG_TX_CLR);
			if (!mt_rr_chk(d, CFG_ADDR(MT_USB_U3DMA_CFG), &cfg) &&
			    !(cfg & MT_USB_DMA_CFG_TX_BUSY))
				break;
		}
		mt_set(d, CFG_ADDR(MT_USB_U3DMA_CFG), MT_USB_DMA_CFG_TX_BULK_EN);
		if (mt_rr_chk(d, CFG_ADDR(MT_USB_U3DMA_CFG), &cfg))
			WARN("USB TX DMA state unreadable after recovery");
		else if (cfg & MT_USB_DMA_CFG_TX_BUSY)
			WARN("USB TX DMA still busy - this open will likely fail; "
			     "see docs/mt7612u-usb-wedge.md");
		else
			LOG("USB TX DMA recovered");
	} else {
		mt_set(d, CFG_ADDR(MT_USB_U3DMA_CFG), MT_USB_DMA_CFG_TX_CLR);
		mt_usleep(20000);
		mt_clear(d, CFG_ADDR(MT_USB_U3DMA_CFG), MT_USB_DMA_CFG_TX_CLR);
	}
}

int mt_adopt(struct mt7612u_dev *d, libusb_device_handle *h,
             libusb_context *ctx, const char **err)
{
	if (!h) { if (err) *err = "no USB handle"; return -1; }
	/* Before mt_identify(): it reads a register, which locks io_lock. */
	mt_dev_state_init(d);
	d->h = h;
	d->ctx = ctx;
	d->owns_handle = 0;
	d->kernel_was_attached = 0;
	if (mt_identify(d, err))
		return -1;
	/* Same recovery mt_open() gets: this is the path the IRadio backend
	 * takes, and a killed previous run wedges the device for it identically. */
	mt_recover_usb(d);
	return 0;
}

/*
 * Open one MT7612U, honouring the caller's selector when more than one is
 * attached.
 *
 * libusb_open_device_with_vid_pid() returns whichever matching device
 * enumerates first, which is fine with one adapter and silently ambiguous
 * with two - a measurement then attributes itself to whichever unit the bus
 * happened to hand over. The selector takes a "bus-port" as lsusb and sysfs
 * spell it ("2-1"), or a bare index into the matches in enumeration order.
 * It is passed in, never read from the environment: bringup fills it from
 * MT7612U_DEV, and a library consumer passes whatever its own config says.
 * Messages below therefore name "the selector", not that variable - a caller
 * that is not bringup would be told to set something it does not use.
 *
 * mt7612u_open_selected() is the public way to pass it; mt7612u_open() is
 * that with NULL. Nothing in this library reads the environment.
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

/* Drop this device's adapter lock, if it is holding one. */
static void unlock_adapter(struct mt7612u_dev *d)
{
#if !defined(_WIN32)
	if (d && d->lock_fd >= 0) { close(d->lock_fd); d->lock_fd = -1; }
#else
	(void)d;
#endif
}

/* Returns a held fd, -1 to proceed unlocked (infrastructure failure), or
 * -2 when another process holds the adapter and the caller must refuse. */
#if defined(_WIN32)
static int lock_adapter(libusb_device *dev, const char **err)
{
	/* Not ported. The Linux interlock above works by contending for the SAME
	 * lock file UsbDeviceLock uses; on Windows UsbDeviceLock is a named mutex
	 * instead (UsbDeviceLock.cpp), so a file lock here would exclude nobody
	 * and mirroring the mutex would be a second copy of a mechanism the
	 * devourer path already owns. Exclusivity on Windows therefore comes from
	 * UsbDeviceLock, which WiFiDriver takes before it ever reaches this
	 * library. What is genuinely unprotected is a direct mt7612u_open() with
	 * no devourer around it - the bench tool's case, and the bench is Linux.
	 * Proceed unlocked rather than refuse: fail-open is what the POSIX path
	 * does for an infrastructure failure too. */
	(void)dev; (void)err;
	return -1;
}
#else
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
#endif /* !_WIN32 */

static libusb_device_handle *open_selected(struct mt7612u_dev *d,
                                           libusb_context *ctx, const char *sel,
                                           const char **err)
{

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
			LOG("MT7612U at %s  <- selected", id);
		} else {
			char idx[8];

			snprintf(idx, sizeof idx, "%d", matches);
			if (strcmp(sel, idx)) { matches++; continue; }
			LOG("MT7612U at %s  <- selected by index %d", id, matches);
		}

		if (!h) {
			int lk = lock_adapter(list[i], err);

			if (lk == -2) {          /* held by someone else */
				libusb_free_device_list(list, 1);
				return NULL;
			}
			d->lock_fd = lk;
			if (libusb_open(list[i], &h)) {
				/* Only mt_close() releases the lock, and a failed
				 * mt_open() never reaches it - so holding it here
				 * made the process collide with its own stale lock
				 * on the very next retry. Release what this
				 * iteration took. */
				h = NULL;
				unlock_adapter(d);
			}
		}
		matches++;
		if (h && sel && *sel)
			break;
	}

	if (matches > 1 && (!sel || !*sel))
		WARN("%d MT7612U adapters attached and no selector was given - "
		    "using the first. Pass a \"<bus>-<port>\" selector to choose "
		    "(bringup takes it from MT7612U_DEV).", matches);
	libusb_free_device_list(list, 1);
	if (!h && err)
		*err = matches ? "MT7612U found but could not be opened (try sudo)"
		               : "MT7612U not found";
	return h;
}

int mt_open(struct mt7612u_dev *d, const char **err)
{
	int rc;

	/* Both open paths init this before any register I/O; see mt_adopt(). */
	mt_dev_state_init(d);

	if (libusb_init(&d->ctx)) { if (err) *err = "libusb_init failed"; return -1; }
	d->owns_handle = 1;

	d->h = open_selected(d, d->ctx, d->dev_selector, err);
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
		d->h = open_selected(d, d->ctx, d->dev_selector, NULL);
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

	/* Shared with mt_adopt(): the wedge is a property of the device, not of
	 * how this process got hold of it. */
	mt_recover_usb(d);
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
	/* Terminal teardown, and the single place io_lock is released: callers
	 * do their last register I/O (mt_mac_stop) before mt_close, and mt_close
	 * itself takes no lock.  Guarded, so it is a no-op if open never got far
	 * enough to init it.  Covers mt7612u_close() and bring_up()'s fail path;
	 * the two open-failure returns that bypass mt_close destroy it directly. */
	mt_dev_state_destroy(d);
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
		unlock_adapter(d);
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
	unlock_adapter(d);
}

/* Block write, as mt76u_copy(): one MULTI_WRITE per batch, wValue 0.
 * The kernel uses this for the WCID address table (8 B) and the shared-key
 * table (32 B) - 192 transfers that would otherwise be ~700 4-byte writes. */
void mt_wr_copy(struct mt7612u_dev *d, uint32_t offset, const void *data, int len)
{
	const uint8_t *p = (const uint8_t *)data;
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
