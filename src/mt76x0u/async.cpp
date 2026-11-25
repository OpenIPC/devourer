/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * The RX ring over libusb and the event thread that drives it: MT_RX_RING
 * transfers permanently in flight on EP 0x84, each resubmitted from its own
 * completion so the endpoint is never idle.
 *
 * Copied from src/mt7612u/async.cpp with its TX half removed (this port is
 * receive-only, and MCU commands go out synchronously on EP 0x08). The
 * locking, the stop ordering and the leak-rather-than-free policy for
 * transfers that never come back are unchanged; the comments in the original
 * carry the reasoning and the measurements.
 *
 * Each completed transfer is parsed (rx.cpp) under the ring lock and the
 * frame handed to the callback outside it.
 */
#include <stdlib.h>
#include <string.h>
#include "internal.h"

static int locked_get(struct mt0_async *a, const int *field)
{
	int v;

	a->lock.lock();
	v = *field;
	a->lock.unlock();
	return v;
}

static void evt_thread(struct mt76x0u_dev *d)
{
	struct mt0_async *a = d->a;
	struct timeval tv = { 0, 50000 };

	while (locked_get(a, &a->running))
		libusb_handle_events_timeout_completed(d->ctx, &tv, NULL);
}

static void LIBUSB_CALL rx_done(struct libusb_transfer *t)
{
	struct mt0_slot *s = (struct mt0_slot *)t->user_data;
	struct mt0_async *a = s->a;
	int resubmit;

	if (t->status == LIBUSB_TRANSFER_COMPLETED) {
		const uint8_t *frame = NULL;
		struct mt76x0u_rx_info info;
		int len = 0;

		/* The parse reads the device (the RSSI correction), so it runs
		 * under the lock a stranding stop clears s->d under: either d is
		 * live for the whole parse, or it is NULL and nothing is touched
		 * but the leaked ring. The parser takes no lock itself. */
		a->lock.lock();
		if (s->d) {
			len = mt0_rx_parse(s->d, t->buffer, t->actual_length, &frame, &info);
			if (len == MT_RX_PARSE_INVALID)
				a->rx_invalid++;
			if (len <= 0)
				a->rx_dropped++;
			else
				a->rx_frames++;
		}
		a->lock.unlock();
		/* Outside the lock, as in mt7612u: a callback may call back in. */
		if (len > 0 && a->cb)
			a->cb(a->cb_user, frame, (size_t)len, &info);
	} else if (t->status != LIBUSB_TRANSFER_CANCELLED) {
		a->lock.lock();
		a->rx_err++;
		if (t->status == LIBUSB_TRANSFER_NO_DEVICE)
			a->no_device = 1;
		a->lock.unlock();
	}

	/* Decide and resubmit as one step under the lock: a stop clears
	 * rx_active under it before its cancel pass, so a resubmitted transfer
	 * is in flight for that pass to cancel, and one decided after it is not
	 * resubmitted at all. */
	a->lock.lock();
	/* Not after NO_DEVICE: the adapter is gone (unplugged), and the ring
	 * running dry is how the consumer learns it (mt76x0u_get_stats). */
	resubmit = a->rx_active && s->d && t->status != LIBUSB_TRANSFER_CANCELLED &&
	           t->status != LIBUSB_TRANSFER_NO_DEVICE;
	if (resubmit) {
		int src = libusb_submit_transfer(t);

		if (src == 0) {
			a->lock.unlock();
			return;
		}
		a->rx_err++;
		if (src == LIBUSB_ERROR_NO_DEVICE)
			a->no_device = 1;
	}
	a->rx_inflight--;
	a->cv.notify_all();
	a->lock.unlock();
}

void mt0_async_rx_done_for_test(struct libusb_transfer *t)
{
	rx_done(t);
}

int mt0_async_start(struct mt76x0u_dev *d, mt76x0u_rx_cb cb, void *user)
{
	struct mt0_async *a;

	if (d->transfers_stranded) {
		ERR("RX ring start refused: a previous ring's transfers are still "
		    "owned by libusb on this endpoint");
		return -1;
	}
	if (d->a) return 0;
	if (!cb) {
		ERR("RX ring start refused: no callback");
		return -1;
	}
	/* try/catch, not new(nothrow): the members allocate in their
	 * constructors, and nothing may throw across the extern "C" ABI. */
	try {
		a = new mt0_async{};
	} catch (...) {
		return -1;
	}
	d->a = a;
	a->cb = cb;
	a->cb_user = user;

	for (int i = 0; i < MT_RX_RING; i++) {
		a->rx_slot[i].d = d;
		a->rx_slot[i].a = a;
		a->rx_slot[i].idx = i;
		a->rx[i] = libusb_alloc_transfer(0);
		if (!a->rx[i]) goto fail;
	}

	a->running = 1;
	try {
		a->evt = std::thread(evt_thread, d);
	} catch (...) {
		a->running = 0;
		goto fail;
	}
	a->evt_started = 1;

	{
		/* Under io_lock: an MCU wait holds it across its owner check and
		 * EP 4 drain, so the claim cannot land in between. */
		d->io_lock.lock();
		int owner = MT_EP4_NONE;
		const bool claimed =
			d->ep4_owner.compare_exchange_strong(owner, MT_EP4_RING);
		d->io_lock.unlock();
		if (!claimed) {
			ERR("RX ring not started: EP 0x84 already has a reader (owner %d)",
			    owner);
			goto fail;
		}
	}
	a->lock.lock();
	a->rx_active = 1;
	a->lock.unlock();
	for (int i = 0; i < MT_RX_RING; i++) {
		libusb_fill_bulk_transfer(a->rx[i], d->h, MT_EP_IN_PKT_RX,
		                          a->rx_buf[i], MT_RX_BUFSZ,
		                          rx_done, &a->rx_slot[i], 0);
		if (libusb_submit_transfer(a->rx[i])) {
			ERR("could not submit RX transfer %d", i);
			goto fail;
		}
		a->lock.lock();
		a->rx_inflight++;
		a->lock.unlock();
	}
	LOG("async: %d RX transfers in flight on EP 0x%02x", MT_RX_RING,
	    MT_EP_IN_PKT_RX);
	return 0;

fail:
	mt0_async_stop(d);
	return -1;
}

/*
 * Tear the ring down. libusb owns a submitted transfer until its callback
 * runs, so nothing is freed while one is in flight: cancel, wait up to 2 s,
 * and if any is still out, leak the whole ring (with its slots detached from
 * the device) and mark the device stranded rather than free memory libusb may
 * still write into. Same policy as mt7612u.
 */
int mt0_async_stop(struct mt76x0u_dev *d)
{
	struct mt0_async *a = d->a;
	int stuck_rx;

	if (!a) return 0;

	a->lock.lock();
	a->rx_active = 0;
	a->lock.unlock();

	for (int i = 0; i < MT_RX_RING; i++)
		if (a->rx[i]) libusb_cancel_transfer(a->rx[i]);

	/* The event thread is still running, so completions keep arriving. */
	a->lock.lock();
	for (int spins = 0; a->rx_inflight && spins < 200; spins++)
		a->cv.wait_for(a->lock, std::chrono::milliseconds(10));
	stuck_rx = a->rx_inflight;
	a->running = 0;
	if (stuck_rx)
		for (int i = 0; i < MT_RX_RING; i++)
			a->rx_slot[i].d = NULL;
	a->lock.unlock();

	if (a->evt_started)
		a->evt.join();

	d->a = NULL;
	if (stuck_rx) {
		/* EP 4 stays owned by the leaked ring: no MCU-wait drain may read
		 * it underneath transfers libusb still holds. */
		d->transfers_stranded = 1;
		ERR("RX ring stop: %d transfers still in flight after 2 s - leaking "
		    "the ring rather than freeing memory libusb still owns", stuck_rx);
		return -1;
	}
	{
		int ring = MT_EP4_RING;
		d->ep4_owner.compare_exchange_strong(ring, MT_EP4_NONE);
	}
	for (int i = 0; i < MT_RX_RING; i++)
		if (a->rx[i]) libusb_free_transfer(a->rx[i]);
	delete a;
	return 0;
}

int mt76x0u_rx_start(struct mt76x0u_dev *d, mt76x0u_rx_cb cb, void *user)
{
	if (!d) return -1;
	return mt0_async_start(d, cb, user);
}

int mt76x0u_rx_stop(struct mt76x0u_dev *d)
{
	if (!d) return -1;
	return mt0_async_stop(d);
}

void mt76x0u_get_stats(struct mt76x0u_dev *d, struct mt76x0u_stats *out)
{
	if (!out) return;
	memset(out, 0, sizeof *out);
	if (!d) return;
	out->io_errors = d->io_err;
	struct mt0_async *a = d->a;
	if (!a) return;
	a->lock.lock();
	out->rx_frames = a->rx_frames;
	out->rx_dropped = a->rx_dropped;
	out->rx_invalid = a->rx_invalid;
	out->rx_err = a->rx_err;
	out->rx_no_device = a->no_device;
	out->rx_ring_dead = a->rx_active && a->rx_inflight == 0;
	a->lock.unlock();
}
