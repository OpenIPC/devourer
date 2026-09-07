/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * Async TX and RX rings over libusb, plus the event thread that drives them.
 * This is what mt76 gets from URBs and NAPI; here it is one pthread calling
 * libusb_handle_events plus two pools of libusb_transfer.
 *
 * RX: MT_RX_RING transfers permanently in flight on EP 4 IN. A completion
 * parses the RXWI and resubmits immediately, so the endpoint is never idle -
 * which is also what keeps the chip from wedging (BRINGUP-RESULTS.md).
 *
 * TX: a pool of MT_TX_RING transfers on EP 4 OUT with a free list. Submitting
 * does not wait for the wire; mt7612u_tx only blocks when every slot is in
 * flight, which is the back-pressure point.
 */
#include <stdlib.h>
#include <string.h>
#include "internal.h"

/*
 * Every field shared between the event thread and the caller lives under
 * a->lock. `volatile` alone is not a memory model: it orders nothing and
 * makes no read-modify-write atomic, and rx_inflight is decremented from the
 * completion callback while mt_async_stop() waits on it.
 */
static int locked_get(struct mt_async *a, const int *field)
{
	int v;

	pthread_mutex_lock(&a->lock);
	v = *field;
	pthread_mutex_unlock(&a->lock);
	return v;
}

static void *evt_thread(void *arg)
{
	struct mt7612u_dev *d = arg;
	struct mt_async *a = d->a;
	struct timeval tv = { .tv_sec = 0, .tv_usec = 50000 };

	while (locked_get(a, &a->running))
		libusb_handle_events_timeout_completed(d->ctx, &tv, NULL);
	return NULL;
}

static void LIBUSB_CALL rx_done(struct libusb_transfer *t)
{
	struct mt_slot *s = t->user_data;
	struct mt7612u_dev *d = s->d;
	struct mt_async *a = s->a;
	int resubmit;

	if (t->status == LIBUSB_TRANSFER_COMPLETED) {
		const uint8_t *frame = NULL;
		struct mt7612u_rx_info info;
		int len = mt_rx_parse(d, t->buffer, t->actual_length, &frame, &info);

		if (len <= 0) {
			/* A frame the parser rejected used to move no counter at
			 * all, which is indistinguishable from one never sent.
			 *
			 * This does NOT cover the oversize case, and it was
			 * measured not to: frames above the MAC's MT_MAX_LEN_CFG
			 * ceiling never reach here, never complete a transfer and
			 * never raise rx_err. The MAC discards them before USB, so
			 * that loss is invisible from this layer by construction -
			 * see mt7612u_caps.max_mpdu_rx. What this counts is a
			 * short or malformed transfer. */
			pthread_mutex_lock(&a->lock);
			a->rx_dropped++;
			pthread_mutex_unlock(&a->lock);
		} else {
			pthread_mutex_lock(&a->lock);
			a->rx_frames++;
			pthread_mutex_unlock(&a->lock);
			/* Outside the lock: a callback is allowed to transmit,
			 * and mt_async_tx_submit() takes this same mutex. */
			if (a->cb)
				a->cb(a->cb_user, frame, (size_t)len, &info);
		}
	} else if (t->status != LIBUSB_TRANSFER_CANCELLED) {
		pthread_mutex_lock(&a->lock);
		a->rx_err++;
		pthread_mutex_unlock(&a->lock);
	}

	resubmit = locked_get(a, &a->rx_active) &&
	           t->status != LIBUSB_TRANSFER_CANCELLED;
	if (resubmit && libusb_submit_transfer(t) == 0)
		return;

	/* Not resubmitted: this transfer is now owned by us again. */
	pthread_mutex_lock(&a->lock);
	if (resubmit)
		a->rx_err++;
	a->rx_inflight--;
	pthread_cond_broadcast(&a->cv);
	pthread_mutex_unlock(&a->lock);
}

static void LIBUSB_CALL tx_done(struct libusb_transfer *t)
{
	struct mt_slot *s = t->user_data;
	struct mt_async *a = s->a;

	pthread_mutex_lock(&a->lock);
	if (t->status == LIBUSB_TRANSFER_COMPLETED &&
	    t->actual_length == t->length)
		a->tx_done_n++;
	else
		a->tx_err++;
	a->tx_busy[s->idx] = 0;
	a->tx_inflight--;
	pthread_cond_broadcast(&a->cv);
	pthread_mutex_unlock(&a->lock);
}

int mt_async_start(struct mt7612u_dev *d, mt7612u_rx_cb cb, void *user)
{
	if (d->transfers_stranded) {
		ERR("async start refused: a previous ring's transfers are still "
		    "owned by libusb on these endpoints");
		return -1;
	}
	struct mt_async *a;

	if (d->a) return 0;
	a = calloc(1, sizeof *a);
	if (!a) return -1;
	d->a = a;
	a->cb = cb;
	a->cb_user = user;
	pthread_mutex_init(&a->lock, NULL);
	pthread_cond_init(&a->cv, NULL);

	for (int i = 0; i < MT_TX_RING; i++) {
		a->tx_slot[i].d = d;
		a->tx_slot[i].a = a;
		a->tx_slot[i].idx = i;
		a->tx[i] = libusb_alloc_transfer(0);
		if (!a->tx[i]) goto fail;
	}
	for (int i = 0; i < MT_RX_RING; i++) {
		a->rx_slot[i].d = d;
		a->rx_slot[i].a = a;
		a->rx_slot[i].idx = i;
		a->rx[i] = libusb_alloc_transfer(0);
		if (!a->rx[i]) goto fail;
	}

	a->running = 1;
	if (pthread_create(&a->evt, NULL, evt_thread, d)) { a->running = 0; goto fail; }
	a->evt_started = 1;

	if (cb) {
		pthread_mutex_lock(&a->lock);
		a->rx_active = 1;
		pthread_mutex_unlock(&a->lock);
		for (int i = 0; i < MT_RX_RING; i++) {
			libusb_fill_bulk_transfer(a->rx[i], d->h, MT_EP_IN_PKT_RX,
			                          a->rx_buf[i], MT_RX_BUFSZ,
			                          rx_done, &a->rx_slot[i], 0);
			if (libusb_submit_transfer(a->rx[i])) {
				ERR("could not submit RX transfer %d", i);
				goto fail;
			}
			pthread_mutex_lock(&a->lock);
			a->rx_inflight++;
			pthread_mutex_unlock(&a->lock);
		}
		LOG("async: %d RX transfers in flight, %d TX slots",
		    MT_RX_RING, MT_TX_RING);
	} else {
		LOG("async: %d TX slots (RX ring not started)", MT_TX_RING);
	}
	return 0;

fail:
	mt_async_stop(d);
	return -1;
}

/*
 * Tear the rings down. The ordering matters and the failure mode is not a
 * leak but a use-after-free: libusb owns a submitted transfer until its
 * callback runs, so nothing may be freed while it is still in flight.
 *
 * A wedged chip is the case that makes this real - TX URBs that never
 * complete are exactly the situation this driver's own notes describe - so
 * both rings are cancelled, both are waited on, and if either still has
 * transfers outstanding when the deadline expires we deliberately leak the
 * whole mt_async rather than free memory the kernel may still write into.
 */
void mt_async_stop(struct mt7612u_dev *d)
{
	struct mt_async *a = d->a;
	int stuck_tx, stuck_rx;

	if (!a) return;

	pthread_mutex_lock(&a->lock);
	a->rx_active = 0;
	pthread_mutex_unlock(&a->lock);

	/* Cancel *both* rings. Cancelling only RX leaves TX transfers owned by
	 * libusb, and the wait below would then time out with them in flight. */
	for (int i = 0; i < MT_RX_RING; i++)
		if (a->rx[i]) libusb_cancel_transfer(a->rx[i]);
	for (int i = 0; i < MT_TX_RING; i++)
		if (a->tx[i]) libusb_cancel_transfer(a->tx[i]);

	/* The event thread is still running, so completions keep arriving. */
	pthread_mutex_lock(&a->lock);
	for (int spins = 0; (a->tx_inflight || a->rx_inflight) && spins < 200; spins++) {
		struct timespec ts;

		clock_gettime(CLOCK_REALTIME, &ts);
		ts.tv_nsec += 10000000;
		if (ts.tv_nsec >= 1000000000) { ts.tv_sec++; ts.tv_nsec -= 1000000000; }
		pthread_cond_timedwait(&a->cv, &a->lock, &ts);
	}
	stuck_tx = a->tx_inflight;
	stuck_rx = a->rx_inflight;
	a->running = 0;
	pthread_mutex_unlock(&a->lock);

	if (a->evt_started)
		pthread_join(a->evt, NULL);

	d->a = NULL;
	if (stuck_tx || stuck_rx) {
		/* Leaking the ring is the safe half. The other half is that libusb
		 * still owns those transfers while the event thread has just been
		 * joined, so nothing will ever complete them - and releasing the
		 * interface, closing the handle or exiting the context underneath
		 * them is undefined. Mark the device stranded: mt_close() then
		 * leaks the USB objects too rather than freeing what libusb holds,
		 * and mt_async_start() refuses to submit a second ring onto the
		 * same endpoints. Consistent with the leak, not a new policy. */
		d->transfers_stranded = 1;
		ERR("async stop: %d TX and %d RX transfers still in flight after 2 s "
		    "- leaking the ring, and the USB handle with it, rather than "
		    "freeing memory libusb still owns", stuck_tx, stuck_rx);
		return;
	}

	for (int i = 0; i < MT_TX_RING; i++)
		if (a->tx[i]) libusb_free_transfer(a->tx[i]);
	for (int i = 0; i < MT_RX_RING; i++)
		if (a->rx[i]) libusb_free_transfer(a->rx[i]);
	pthread_mutex_destroy(&a->lock);
	pthread_cond_destroy(&a->cv);
	free(a);
}

/*
 * Hand a fully framed buffer to the TX pool. Blocks only when every slot is
 * in flight. Returns 0 on submit, -1 on error.
 */
int mt_async_tx_submit(struct mt7612u_dev *d, const uint8_t *buf, int len)
{
	struct mt_async *a = d->a;
	int idx = -1, rc;

	if (!a || len > MT_TX_BUFSZ) return -1;

	pthread_mutex_lock(&a->lock);
	for (;;) {
		/* A teardown must not leave a caller parked here forever. */
		if (!a->running) { pthread_mutex_unlock(&a->lock); return -1; }
		for (int i = 0; i < MT_TX_RING; i++)
			if (!a->tx_busy[i]) { idx = i; break; }
		if (idx >= 0) break;
		pthread_cond_wait(&a->cv, &a->lock);
	}
	a->tx_busy[idx] = 1;
	a->tx_inflight++;
	pthread_mutex_unlock(&a->lock);

	memcpy(a->tx_buf[idx], buf, (size_t)len);
	libusb_fill_bulk_transfer(a->tx[idx], d->h, MT_EP_OUT_AC_BE,
	                          a->tx_buf[idx], len, tx_done,
	                          &a->tx_slot[idx], 1000);
	rc = libusb_submit_transfer(a->tx[idx]);

	pthread_mutex_lock(&a->lock);
	if (rc) {
		a->tx_busy[idx] = 0;
		a->tx_inflight--;
		a->tx_err++;
		pthread_cond_broadcast(&a->cv);
	} else {
		a->tx_submitted++;
	}
	pthread_mutex_unlock(&a->lock);
	return rc ? -1 : 0;
}

/*
 * Consistent snapshot of the ring counters. Reading the fields directly races
 * with the event thread, and after a teardown that had to leak a stuck ring
 * there is no ring to read at all - so callers go through this.
 */
void mt_async_stats(struct mt7612u_dev *d, struct mt_async_stats *out)
{
	struct mt_async *a = d->a;

	memset(out, 0, sizeof *out);
	if (!a) return;
	pthread_mutex_lock(&a->lock);
	out->tx_submitted = a->tx_submitted;
	out->tx_done      = a->tx_done_n;
	out->tx_err       = a->tx_err;
	out->rx_frames    = a->rx_frames;
	out->rx_err       = a->rx_err;
	out->rx_invalid   = a->rx_invalid;
	out->rx_dropped   = a->rx_dropped;
	pthread_mutex_unlock(&a->lock);
}

int mt7612u_rx_start(struct mt7612u_dev *d, mt7612u_rx_cb cb, void *user)
{
	if (!cb) return -1;
	return mt_async_start(d, cb, user);
}

int mt7612u_rx_stop(struct mt7612u_dev *d)
{
	mt_async_stop(d);
	return 0;
}

/* A frame whose rate word named no valid PHY. Counted under the ring's lock
 * when one is running; on the synchronous bring-up path there is no ring and
 * nothing to count into, which is fine - that path prints every frame. */
void mt_async_note_invalid(struct mt7612u_dev *d)
{
	struct mt_async *a = d->a;

	if (!a) return;
	pthread_mutex_lock(&a->lock);
	a->rx_invalid++;
	pthread_mutex_unlock(&a->lock);
}

/* Public form of the snapshot above. */
void mt7612u_get_stats(struct mt7612u_dev *d, struct mt7612u_stats *out)
{
	struct mt_async_stats st;

	mt_async_stats(d, &st);
	out->tx_submitted = st.tx_submitted;
	out->tx_done      = st.tx_done;
	out->tx_err       = st.tx_err;
	out->rx_frames    = st.rx_frames;
	out->rx_err       = st.rx_err;
	out->rx_invalid   = st.rx_invalid;
	out->rx_dropped   = st.rx_dropped;
}
