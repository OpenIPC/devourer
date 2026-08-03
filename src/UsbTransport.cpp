#include "UsbTransport.h"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "Event.h"
#include "UsbDeviceLock.h"
#include "UsbOpen.h"

namespace devourer {

namespace {
/* Saturating atomic min/max for the RX-ring telemetry — relaxed and
 * best-effort (a rare lost update is acceptable for a diagnostic counter). */
inline void atomic_min(std::atomic<int> &m, int v) {
  int cur = m.load(std::memory_order_relaxed);
  while (v < cur && !m.compare_exchange_weak(cur, v, std::memory_order_relaxed))
    ;
}
inline void atomic_max(std::atomic<long long> &m, long long v) {
  long long cur = m.load(std::memory_order_relaxed);
  while (v > cur && !m.compare_exchange_weak(cur, v, std::memory_order_relaxed))
    ;
}

/* Shared state for the async RX URB queue. */
struct AsyncRxShared {
  const std::function<void(const uint8_t *, int)> *cb;
  const std::function<bool()> *stop;
  /* Atomic: in co-running (TX + self-capture RX) mode both the RX loop's own
   * event pump and the TX event loop may run this callback, so `active` is
   * written from the pump thread while the loop below reads it. */
  std::atomic<int> active{0};

  /* Diagnostic telemetry, populated only when `telemetry` (DEVOURER_RX_RING_MS
   * > 0) — the default path pays nothing. `armed` is the count of URBs
   * currently posted to the host controller and awaiting a frame: THIS is the
   * depth that collapses when a slow inline consumer delays resubmit, and it is
   * distinct from `active` (which counts URBs not yet permanently retired and
   * so sits at ~n_urbs regardless of starvation). `min_armed` is the low-water
   * mark since the last rx.ring emit; `cb_max_us` the worst inline-consume
   * latency in the window; `resubmit_fail` cumulative wanted-but-failed
   * resubmits. */
  bool telemetry = false;
  std::atomic<int> armed{0};
  std::atomic<int> min_armed{0};
  std::atomic<long long> cb_max_us{0};
  std::atomic<unsigned long long> resubmit_fail{0};
  /* completions = URB callbacks; empties = those that left the ring with zero
   * URBs posted. empties/completions is the robust host-starvation rate: under
   * RF loss frames rarely arrive so the ring stays armed (empties ~0); under
   * host starvation frames arrive faster than resubmit so the ring drains to
   * empty (empties high) — and unlike the poll-loop rx.ring emit, this is
   * counted in the callback, so it survives the pump-thread starvation that
   * makes the periodic telemetry sparse exactly when it matters. */
  std::atomic<unsigned long long> completions{0};
  std::atomic<unsigned long long> empties{0};

  /* reorder-pool mode (RxMode::ReorderPool): a free-list of spare RX buffers.
   * On completion the callback swaps a fresh buffer onto the wire and resubmits
   * the URB BEFORE consuming the received one, so the ring stays armed through
   * the slow inline consume — the fix for the burst-starvation of the default
   * async ring. Zerocopy-safe by construction: the buffer being DMA'd is never
   * the one being read. The mutex guards the free-list because the callback can
   * run from the RX pump and, in co-running TX+RX, the TX pump. */
  bool reorder = false;
  int buf_size = 0;
  std::mutex pool_mu;
  std::vector<uint8_t *> free_bufs;

  /* spsc-fat mode (RxMode::SpscFat): the pump callback only reaps + re-arms the
   * URB (microseconds, never blocked) and hands the received buffer to a
   * SEPARATE consumer thread over this queue; the consumer runs on_data and
   * returns the buffer to the pool. This keeps the ring fully armed even while
   * the consumer stalls or is preempted — converting a chip-FIFO overflow
   * (dropped frames) into bounded host-queue backlog (delayed frames). The fix
   * for a stalling/preempted CONSUMER, where reorder-pool can't help because it
   * still consumes on the pump thread.
   *
   * ARQ caveat, bench-measured (tests/arq_e2e_delivery.sh): keeping the ring
   * armed means the chip ADMITS AND ACKS every frame — so a frame dropped at
   * pool exhaustion is an ACKed-but-undelivered loss the hardware-ARQ peer
   * will log as delivered and never retry. The default async ring loses the
   * same frames chip-side instead, where the 8812EU declines the ACK and the
   * ARQ loop recovers them (14,214/14,214 stall-window drops reported ok=0
   * there, vs 5,667 ok=1-but-lost under the drop policy here).
   *
   * PoolExhaust picks which contract survives overload: Backpressure (the
   * default) PARKS the completed URB unarmed — the payload still reaches the
   * consumer queue, the ring shrinks, the chip FIFO fills and the chip
   * declines further ACKs, so overload loss stays ARQ-visible; parked URBs
   * re-arm as the consumer returns buffers. Drop keeps the ring armed and
   * discards the payload (counted in pool_dropped) — smoother, but only for
   * consumers that accept silent loss. */
  bool spsc = false;
  bool bp = true; /* PoolExhaust::Backpressure */
  std::atomic<unsigned long long> pool_dropped{0};
  /* Backpressure-policy state, guarded by pool_mu with free_bufs: URBs parked
   * bufferless at exhaustion, re-armed from the consumer's buffer-return path.
   * parking_closed is teardown's gate — set (under pool_mu) before the cancel
   * pass, so no late re-arm can launch an uncancellable infinite-timeout URB.
   * pool_stalls counts park events (the backpressure counterpart of
   * pool_dropped; nothing is lost host-side on this path). */
  std::vector<libusb_transfer *> parked;
  bool parking_closed = false;
  std::atomic<unsigned long long> pool_stalls{0};
  /* Device-gone latch. Plain async self-terminates on unplug (every URB
   * error-retires, active hits 0, rx_loop returns) — parked URBs never
   * complete, so backpressure mode needs an explicit signal or a dead device
   * leaves the loop spinning until Stop(). Set from a NO_DEVICE transfer
   * status or submit error; rx_loop exits on it and retires the parked set
   * through the normal teardown path. */
  std::atomic<bool> dead{false};
  std::mutex queue_mu;
  std::condition_variable queue_cv;
  std::deque<std::pair<uint8_t *, int>> queue;
  bool consumer_stop = false;
};

/* Run the caller's consumer, timing it for the rx.ring cb_max_us telemetry. */
inline void rx_consume(AsyncRxShared *s, const uint8_t *buf, int len) {
  if (len <= 0)
    return;
  if (s->telemetry) {
    const auto t0 = std::chrono::steady_clock::now();
    (*s->cb)(buf, len);
    atomic_max(s->cb_max_us,
               std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::steady_clock::now() - t0)
                   .count());
  } else {
    (*s->cb)(buf, len);
  }
}

extern "C" void LIBUSB_CALL devourer_rx_cb(libusb_transfer *t) {
  auto *s = static_cast<AsyncRxShared *>(t->user_data);
  if (t->status == LIBUSB_TRANSFER_NO_DEVICE)
    s->dead.store(true, std::memory_order_relaxed);
  /* This URB just completed — it has left the wire until resubmitted. */
  if (s->telemetry) {
    const int a = s->armed.fetch_sub(1, std::memory_order_relaxed) - 1;
    atomic_min(s->min_armed, a);
    s->completions.fetch_add(1, std::memory_order_relaxed);
    if (a <= 0)
      s->empties.fetch_add(1, std::memory_order_relaxed);
  }
  const bool resubmit = !(*s->stop)() &&
                        (t->status == LIBUSB_TRANSFER_COMPLETED ||
                         t->status == LIBUSB_TRANSFER_TIMED_OUT);
  const int rlen = t->status == LIBUSB_TRANSFER_COMPLETED ? t->actual_length : 0;

  if (s->reorder) {
    uint8_t *received = t->buffer;
    uint8_t *fresh = nullptr;
    if (resubmit) {
      std::lock_guard<std::mutex> lk(s->pool_mu);
      if (!s->free_bufs.empty()) {
        fresh = s->free_bufs.back();
        s->free_bufs.pop_back();
      }
    }
    if (fresh) {
      /* Re-arm the ring with a DIFFERENT buffer, then consume the received one
       * — the ring is never idle during the slow consume. */
      t->buffer = fresh;
      t->length = s->buf_size;
      if (libusb_submit_transfer(t) == 0) {
        if (s->telemetry)
          s->armed.fetch_add(1, std::memory_order_relaxed);
        rx_consume(s, received, rlen);
        std::lock_guard<std::mutex> lk(s->pool_mu);
        s->free_bufs.push_back(received);
        return;
      }
      /* submit failed — return the spare, restore, fall to the inline path. */
      {
        std::lock_guard<std::mutex> lk(s->pool_mu);
        s->free_bufs.push_back(fresh);
      }
      t->buffer = received;
      if (s->telemetry)
        s->resubmit_fail.fetch_add(1, std::memory_order_relaxed);
    }
    /* Pool exhausted (consumer hopelessly behind), submit failed, or teardown:
     * degrade to the inline consume-then-resubmit-same behaviour. */
    rx_consume(s, received, rlen);
    if (resubmit && libusb_submit_transfer(t) == 0) {
      if (s->telemetry)
        s->armed.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    if (resubmit && s->telemetry)
      s->resubmit_fail.fetch_add(1, std::memory_order_relaxed);
    s->active--;
    return;
  }

  if (s->spsc) {
    uint8_t *received = t->buffer;
    uint8_t *fresh = nullptr;
    if (resubmit) {
      std::lock_guard<std::mutex> lk(s->pool_mu);
      if (!s->free_bufs.empty()) {
        fresh = s->free_bufs.back();
        s->free_bufs.pop_back();
      }
    }
    if (fresh) {
      /* Re-arm the ring with a fresh buffer, then hand the received one to the
       * consumer thread — the pump never runs on_data, so it can re-arm in
       * microseconds and the ring stays armed through any consumer stall. */
      t->buffer = fresh;
      t->length = s->buf_size;
      if (libusb_submit_transfer(t) == 0) {
        if (s->telemetry)
          s->armed.fetch_add(1, std::memory_order_relaxed);
        if (rlen > 0) {
          std::lock_guard<std::mutex> lk(s->queue_mu);
          s->queue.emplace_back(received, rlen);
          s->queue_cv.notify_one();
        } else { /* empty completion — nothing to consume, recycle the buffer */
          std::lock_guard<std::mutex> lk(s->pool_mu);
          s->free_bufs.push_back(received);
        }
        return;
      }
      {
        std::lock_guard<std::mutex> lk(s->pool_mu);
        s->free_bufs.push_back(fresh);
      }
      t->buffer = received;
      if (s->telemetry)
        s->resubmit_fail.fetch_add(1, std::memory_order_relaxed);
    }
    /* Pool exhausted (consumer hopelessly behind under sustained overload) or
     * submit failed. Two policies (see the mode comment above):
     *
     * Backpressure (default): the payload is NOT lost — hand it to the
     * consumer queue and PARK this URB bufferless; the ring shrinks, the chip
     * FIFO fills and the chip declines further ACKs, so the overload loss
     * happens chip-side where the ARQ peer can see and retry it. The
     * consumer's buffer-return path re-arms parked URBs. `active` is NOT
     * decremented: a parked URB is pending, not retired (teardown retires the
     * parked set explicitly before its cancel pass). */
    if (s->bp && resubmit && rlen > 0) {
      /* Queue the payload FIRST, park LAST. Once this URB is on the parked
       * list the teardown path may retire it, finish, and destroy the shared
       * state — so parked.push_back must be this callback's final shared-
       * state access (the parked-path analogue of the retire paths' trailing
       * `active--`, which teardown likewise blocks on). */
      t->buffer = nullptr; /* payload buffer now belongs to the queue */
      {
        std::lock_guard<std::mutex> lk(s->queue_mu);
        s->queue.emplace_back(received, rlen);
      }
      s->queue_cv.notify_one();
      {
        std::lock_guard<std::mutex> lk(s->pool_mu);
        if (!s->parking_closed) {
          s->pool_stalls.fetch_add(1, std::memory_order_relaxed);
          s->parked.push_back(t);
          return; /* lock releases on return; nothing may follow the park */
        }
      }
      s->active--; /* parking closed = teardown: retire without resubmit */
      return;
    }
    /* Drop policy: preserve the pump's never-block invariant by re-arming
     * with the received buffer and DROPPING this frame — a bounded loss, vs
     * the cascade an inline consume would trigger. The chip already ACKed
     * this frame: count every received frame dropped here, exhaustion and
     * failed-re-arm alike — both are post-ACK host drops, and the re-arm
     * failure is separable because it also ticks resubmit_fail. `resubmit`
     * gates the count: a teardown-window frame (stop requested) is
     * intentional loss, not an overload signal. */
    if (resubmit && rlen > 0)
      s->pool_dropped.fetch_add(1, std::memory_order_relaxed);
    if (resubmit && libusb_submit_transfer(t) == 0) {
      if (s->telemetry)
        s->armed.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    if (resubmit && s->telemetry)
      s->resubmit_fail.fetch_add(1, std::memory_order_relaxed);
    s->active--;
    return;
  }

  /* Default async ring: consume inline, then resubmit the same URB. */
  rx_consume(s, t->buffer, rlen);
  if (resubmit && libusb_submit_transfer(t) == 0) {
    if (s->telemetry)
      s->armed.fetch_add(1, std::memory_order_relaxed); /* back on the wire */
    return;
  }
  if (resubmit && s->telemetry)
    s->resubmit_fail.fetch_add(1, std::memory_order_relaxed);
  s->active--; /* not resubmitted -> this URB is done */
}
} // namespace

UsbTransport::UsbTransport(libusb_device_handle *dev_handle, Logger_t logger,
                           libusb_context *ctx,
                           std::shared_ptr<devourer::UsbDeviceLock> usb_lock,
                           bool rx_zerocopy, RxMode rx_mode, int pool_spare,
                           int ring_ms, PoolExhaust pool_exhaust)
    : _dev_handle{dev_handle}, _ctx{ctx}, _logger{std::move(logger)},
      _rx_zerocopy{rx_zerocopy}, _rx_mode{rx_mode}, _pool_spare{pool_spare},
      _ring_ms{ring_ms}, _pool_exhaust{pool_exhaust},
      _usb_lock{std::move(usb_lock)} {
  libusb_device_descriptor desc{};
  if (libusb_get_device_descriptor(libusb_get_device(_dev_handle), &desc) ==
      LIBUSB_SUCCESS) {
    _info.vid = desc.idVendor;
    _info.pid = desc.idProduct;
    _logger->info("USB device {:04x}:{:04x}", _info.vid, _info.pid);
  }
  // Endpoint addresses are not stable across Realtek USB variants. In
  // particular, RTL8822BU/"RTL8812BU" is composite: interfaces 0 and 1 are
  // Bluetooth, while Wi-Fi uses interface 2 with bulk IN endpoint 0x84 rather
  // than the 0x81 used by RTL8812AU. Discovering the active bulk interface is
  // therefore required before firmware download, RX, or TX can work.
  discover_endpoints();
  _info.valid = true;
}

UsbTransport::~UsbTransport() {
  /* Backstop only. The device's Stop()/destructor quiesces TX while every
   * owner is alive, which is the path that makes teardown safe; reaching the
   * transport destructor with transfers still in flight means the caller tore
   * libusb down first, and by then _ctx may already be freed — pumping it is
   * the SIGSEGV this diagnostic exists to name. The transport is shared_ptr-
   * owned, so it is not guaranteed the device drops the last reference; hence
   * the drain attempt stays. */
  if (!_tx_shutdown.load(std::memory_order_acquire) &&
      _tx_inflight.load(std::memory_order_acquire) > 0)
    _logger->error("USB transport destroyed with TX in flight — the owner "
                   "should quiesce (IRtlDevice::Stop) and release the device "
                   "BEFORE libusb_close/libusb_exit; see the teardown order in "
                   "examples/common/DeviceSession.h");
  quiesce_tx();
}

bool UsbTransport::write_bytes(uint16_t reg_num, const uint8_t *ptr, size_t n) {
  return libusb_control_transfer(_dev_handle, REALTEK_USB_VENQT_WRITE, 5,
                                 reg_num, 0, const_cast<uint8_t *>(ptr), n,
                                 USB_TIMEOUT) == static_cast<int>(n);
}

void UsbTransport::rx_loop(
    int buf_size, int n_urbs,
    const std::function<void(const uint8_t *, int)> &on_data,
    const std::function<bool()> &should_stop) {
  if (_rx_mode == RxMode::Sync) {
    rx_loop_sync(buf_size, on_data, should_stop);
    return;
  }
  if (_rx_mode == RxMode::Decoupled)
    _logger->warn("RX: mode {} not yet implemented — falling back to async ring",
                  static_cast<int>(_rx_mode));
  const bool reorder = _rx_mode == RxMode::ReorderPool;
  const bool spsc = _rx_mode == RxMode::SpscFat;
  AsyncRxShared sh{&on_data, &should_stop};
  sh.telemetry = _ring_ms > 0;
  sh.reorder = reorder;
  sh.spsc = spsc;
  sh.bp = _pool_exhaust == PoolExhaust::Backpressure;
  sh.buf_size = buf_size;
  /* reorder-pool and spsc-fat post n_urbs URBs but allocate pool_spare extra
   * buffers so a burst / stall backlog is absorbed in the host pool instead of
   * overflowing the chip RX FIFO. Plain async has no spares. */
  const int spare = (reorder || spsc) && _pool_spare > 0 ? _pool_spare : 0;
  const int n_pool = n_urbs + spare;
  std::vector<libusb_transfer *> xfers;
  /* Zerocopy RX ring: allocate each buffer from kernel DMA memory
   * (libusb_dev_mem_alloc = USBDEVFS_ALLOC on Linux) so the bulk-IN DMAs a
   * frame straight into this mmap'd buffer and usbfs skips the copy-to-user on
   * reap. dev_mem_alloc returns NULL on backends/HCDs that don't support it (or
   * when disabled) — fall back per-buffer to a heap allocation, the historical
   * copy-on-reap path. Buffers outlive every in-flight transfer (freed only
   * after the drain below), so the mmap'd memory is never released under a live
   * URB. */
  std::vector<uint8_t *> bufs(n_pool, nullptr);
  std::vector<bool> is_devmem(n_pool, false);
  int zc = 0;
  for (int i = 0; i < n_pool; i++) {
    if (_rx_zerocopy) {
      bufs[i] = libusb_dev_mem_alloc(_dev_handle, buf_size);
      if (bufs[i]) {
        is_devmem[i] = true;
        ++zc;
      }
    }
    if (!bufs[i])
      bufs[i] = static_cast<uint8_t *>(malloc(buf_size));
  }
  /* Hand the reorder spares (the pool tail) to the callback's free-list BEFORE
   * submitting any URB, so an early completion (e.g. a co-running TX pump
   * draining events) never races an unpopulated list. */
  for (int i = n_urbs; i < n_pool; i++)
    if (bufs[i])
      sh.free_bufs.push_back(bufs[i]);
  for (int i = 0; i < n_urbs; i++) {
    if (!bufs[i])
      continue; /* alloc failed — skip this URB (buffer freed in the sweep) */
    libusb_transfer *t = libusb_alloc_transfer(0);
    if (!t)
      continue; /* buffer freed in the by-kind sweep below */
    /* timeout=0 (infinite): a persistent RX ring — each URB stays posted until
     * a frame arrives (COMPLETED), the queue is torn down (CANCELLED, below),
     * or the device errors. This is the kernel rtw88 RX-URB idiom. A finite
     * timeout here would fire once per idle interval on a quiet channel, and
     * libusb's darwin backend logs every LIBUSB_TRANSFER_TIMED_OUT at WARNING
     * level — a continuous "transfer error: timed out" flood that carries no
     * information (RX is healthy; bulk-IN is simply idle) and can bloat a long
     * capture's stderr. The devourer_rx_cb resubmit-on-TIMED_OUT branch is kept
     * as a defensive no-op should a backend still surface a timeout. */
    libusb_fill_bulk_transfer(t, _dev_handle, _info.bulk_in_ep, bufs[i],
                              buf_size, devourer_rx_cb, &sh, 0);
    if (libusb_submit_transfer(t) == 0) {
      xfers.push_back(t);
      sh.active++;
    } else {
      libusb_free_transfer(t);
    }
  }
  _logger->info("RX: {} ring of {} URBs submitted (+{} spare, {} zerocopy DMA, "
                "{} heap)",
                spsc ? "spsc-fat" : reorder ? "reorder-pool" : "async",
                sh.active.load(), static_cast<int>(sh.free_bufs.size()), zc,
                n_pool - zc);
  /* spsc-fat: the consumer thread. The pump callback only reaps + re-arms +
   * enqueues; this thread drains the queue and runs on_data, returning each
   * buffer to the pool. Joined at teardown after the URBs are drained. */
  std::thread consumer;
  if (spsc) {
    consumer = std::thread([&sh]() {
      for (;;) {
        std::pair<uint8_t *, int> item;
        {
          std::unique_lock<std::mutex> lk(sh.queue_mu);
          sh.queue_cv.wait(
              lk, [&sh] { return !sh.queue.empty() || sh.consumer_stop; });
          if (sh.queue.empty()) /* stop requested and drained */
            return;
          item = sh.queue.front();
          sh.queue.pop_front();
        }
        rx_consume(&sh, item.first, item.second);
        /* Buffer return. Backpressure policy: a parked (bufferless) URB gets
         * this buffer and goes straight back on the wire — the ring re-arms
         * exactly as fast as the consumer frees capacity. The submit stays
         * under pool_mu so teardown's parking_closed flip (also under
         * pool_mu) strictly precedes-or-follows any re-arm: no late launch
         * after the cancel pass. (libusb_submit_transfer is thread-safe and
         * the event pump never holds pool_mu while inside libusb, so there is
         * no lock-order inversion.) */
        std::lock_guard<std::mutex> lk(sh.pool_mu);
        if (sh.bp && !sh.parking_closed && !sh.parked.empty()) {
          libusb_transfer *t = sh.parked.back();
          sh.parked.pop_back();
          t->buffer = item.first;
          t->length = sh.buf_size;
          const int rc = libusb_submit_transfer(t);
          if (rc == 0) {
            if (sh.telemetry)
              sh.armed.fetch_add(1, std::memory_order_relaxed);
            continue;
          }
          /* Submit failure: re-park the URB, pool the buffer. Device gone
           * latches `dead` so rx_loop exits and retires the parked set —
           * re-park-forever would otherwise pin `active` above zero with no
           * completion ever coming. */
          t->buffer = nullptr;
          sh.parked.push_back(t);
          if (rc == LIBUSB_ERROR_NO_DEVICE)
            sh.dead.store(true, std::memory_order_relaxed);
          if (sh.telemetry)
            sh.resubmit_fail.fetch_add(1, std::memory_order_relaxed);
        }
        sh.free_bufs.push_back(item.first);
      }
    });
  }
  if (sh.telemetry) {
    /* Seed both the live depth and its low-water mark to the just-submitted URB
     * count. min_armed{0} would otherwise pin the first window's low-water at 0
     * (atomic_min only lowers), reporting a starvation that never happened; the
     * store also wipes any transient recorded by a callback that fired between
     * URB submission above and this seed. */
    const int a0 = sh.active.load(std::memory_order_relaxed);
    sh.armed.store(a0, std::memory_order_relaxed);
    sh.min_armed.store(a0, std::memory_order_relaxed);
  }
  auto last_ring = std::chrono::steady_clock::now();
  while (!should_stop() && sh.active > 0 &&
         !sh.dead.load(std::memory_order_relaxed)) {
    struct timeval tv {0, 100000};
    libusb_handle_events_timeout_completed(_ctx, &tv, nullptr);
    if (sh.telemetry) {
      const auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_ring)
              .count() >= _ring_ms) {
        last_ring = now;
        const int a = sh.armed.load(std::memory_order_relaxed);
        long long pool_free = -1; /* -1 = no host-side pool (plain async) */
        long long qdepth = 0;
        if (reorder || spsc) {
          std::lock_guard<std::mutex> lk(sh.pool_mu);
          pool_free = static_cast<long long>(sh.free_bufs.size());
        }
        if (spsc) {
          std::lock_guard<std::mutex> lk(sh.queue_mu);
          qdepth = static_cast<long long>(sh.queue.size());
        }
        Ev(_logger->events(), "rx.ring")
            .t()
            .f("mode",
               spsc ? "spsc-fat" : reorder ? "reorder-pool" : "async")
            .f("n_urbs", n_urbs)
            .f("armed", a)
            .f("min_armed", sh.min_armed.exchange(a, std::memory_order_relaxed))
            .f("cb_max_us",
               sh.cb_max_us.exchange(0, std::memory_order_relaxed))
            .f("resubmit_fail", (unsigned long long)sh.resubmit_fail.load(
                                    std::memory_order_relaxed))
            .f("completions", (unsigned long long)sh.completions.load(
                                  std::memory_order_relaxed))
            .f("empties", (unsigned long long)sh.empties.load(
                              std::memory_order_relaxed))
            .f("pool_free", pool_free)
            .f("qdepth", qdepth)
            .f("pool_dropped", (unsigned long long)sh.pool_dropped.load(
                                   std::memory_order_relaxed))
            .f("pool_stalls", (unsigned long long)sh.pool_stalls.load(
                                  std::memory_order_relaxed));
      }
    }
  }
  /* Close parking BEFORE the cancel pass (under pool_mu, so no consumer
   * re-arm is mid-submit), then retire the parked URBs: they are not in
   * flight, cancel would be a no-op on them, and the active-drain below would
   * otherwise wait forever for completions that can never come. The transfer
   * objects stay in `xfers` and are freed centrally. */
  {
    std::lock_guard<std::mutex> lk(sh.pool_mu);
    sh.parking_closed = true;
    sh.active -= static_cast<int>(sh.parked.size());
    sh.parked.clear();
  }
  for (auto *t : xfers)
    libusb_cancel_transfer(t);
  while (sh.active > 0) {
    struct timeval tv {0, 100000};
    libusb_handle_events_timeout_completed(_ctx, &tv, nullptr);
  }
  for (auto *t : xfers)
    libusb_free_transfer(t);
  /* spsc-fat: stop and join the consumer before releasing the pool, so no
   * on_data is in flight over a buffer we are about to free. */
  if (spsc) {
    {
      std::lock_guard<std::mutex> lk(sh.queue_mu);
      sh.consumer_stop = true;
    }
    sh.queue_cv.notify_all();
    if (consumer.joinable())
      consumer.join();
  }
  /* All URBs are drained (sh.active == 0) — no transfer references a buffer and
   * the free-list is quiescent, so releasing the whole pool is safe. Ownership
   * is `bufs` (every buffer, including the ones reorder swapped onto transfers
   * or parked on the free-list); free each once, by the way it was allocated. */
  for (int i = 0; i < n_pool; i++) {
    if (!bufs[i])
      continue;
    if (is_devmem[i])
      libusb_dev_mem_free(_dev_handle, bufs[i], buf_size);
    else
      free(bufs[i]);
  }
}

void UsbTransport::rx_loop_sync(
    int buf_size, const std::function<void(const uint8_t *, int)> &on_data,
    const std::function<bool()> &should_stop) {
  /* Legacy single-buffer blocking bulk-IN read — the pre-async-ring reference
   * (floppyhammer-era model). There is no posted-URB ring: while on_data runs,
   * nothing is armed on the wire, so this starves on consume latency rather
   * than resubmit latency. Kept as an A/B baseline for the burst-starvation
   * study; the 200 ms read timeout bounds should_stop() responsiveness on an
   * idle channel. */
  std::vector<uint8_t> buf(buf_size);
  _logger->info("RX: synchronous blocking-read loop (buf {} B)", buf_size);
  const bool telemetry = _ring_ms > 0;
  auto last_ring = std::chrono::steady_clock::now();
  long long cb_max_us = 0;
  while (!should_stop()) {
    int n = rx_raw(buf.data(), buf_size, 200);
    if (n > 0) {
      if (telemetry) {
        const auto t0 = std::chrono::steady_clock::now();
        on_data(buf.data(), n);
        const long long us = std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now() - t0)
                                 .count();
        if (us > cb_max_us)
          cb_max_us = us;
      } else {
        on_data(buf.data(), n);
      }
    } else if (n == LIBUSB_ERROR_NO_DEVICE) {
      _logger->error("RX: device gone (sync read) — stopping");
      break;
    }
    /* n == LIBUSB_ERROR_TIMEOUT (idle channel) or a transient error: loop. */
    if (telemetry) {
      const auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_ring)
              .count() >= _ring_ms) {
        last_ring = now;
        Ev(_logger->events(), "rx.ring")
            .t()
            .f("mode", "sync")
            .f("n_urbs", 1)
            .f("armed", 0) /* no ring: nothing armed while consuming */
            .f("min_armed", 0)
            .f("cb_max_us", cb_max_us)
            .f("resubmit_fail", (unsigned long long)0)
            .f("pool_free", -1);
        cb_max_us = 0;
      }
    }
  }
}

int UsbTransport::rx_raw(uint8_t *buf, int len, int timeout_ms) {
  int actual = 0;
  int rc = libusb_bulk_transfer(_dev_handle, _info.bulk_in_ep, buf, len,
                                &actual, timeout_ms);
  return rc < 0 ? rc : actual;
}

const char *UsbTransport::speed_str() const {
  switch (_info.speed) {
  case LIBUSB_SPEED_UNKNOWN:
    return "UNKNOWN";
  case LIBUSB_SPEED_LOW:
    return "1.5MBit/s";
  case LIBUSB_SPEED_FULL:
    return "12MBit/s";
  case LIBUSB_SPEED_HIGH:
    return "480MBit/s";
  case LIBUSB_SPEED_SUPER:
    return "5000MBit/s";
  case LIBUSB_SPEED_SUPER_PLUS:
    return "10000MBit/s";
  default:
    return NULL;
  }
}

void UsbTransport::discover_endpoints() {
  libusb_device *dev = libusb_get_device(_dev_handle);
  _info.speed = libusb_get_device_speed(dev);
  _logger->info("Running USB bus at {}", speed_str());

  libusb_device_descriptor desc;
  int ret = libusb_get_device_descriptor(dev, &desc);
  if (ret < 0) {
    return;
  }

  for (uint8_t k = 0; k < desc.bNumConfigurations; k++) {
    libusb_config_descriptor *config;
    ret = libusb_get_config_descriptor(dev, k, &config);
    if (LIBUSB_SUCCESS != ret) {
      continue;
    }

    // Do not assume config->interface[0]. Composite adapters expose Bluetooth
    // first; reuse the interface selected before the caller claimed the device.
    const libusb_interface_descriptor *interface_desc = nullptr;
    const int wifi_interface = find_wifi_interface(_dev_handle);
    for (uint8_t interface_index = 0;
         interface_index < config->bNumInterfaces && interface_desc == nullptr;
         interface_index++) {
      const libusb_interface *interface = &config->interface[interface_index];
      if (interface->num_altsetting == 0)
        continue;
      const libusb_interface_descriptor *candidate = &interface->altsetting[0];
      if (candidate->bInterfaceNumber == wifi_interface)
        interface_desc = candidate;
    }

    if (interface_desc == nullptr) {
      libusb_free_config_descriptor(config);
      continue;
    }
    _logger->info("selected USB interface {}", interface_desc->bInterfaceNumber);

    bool found_bulk_in = false;
    for (uint8_t j = 0; j < interface_desc->bNumEndpoints; j++) {
      const libusb_endpoint_descriptor *endpoint = &interface_desc->endpoint[j];
      uint8_t endPointAddr = endpoint->bEndpointAddress;
      const bool is_bulk = (endpoint->bmAttributes & 0b11) ==
                           LIBUSB_ENDPOINT_TRANSFER_TYPE_BULK;
      _logger->info("endpoint[{}]: addr=0x{:X} attrs=0x{:X} bulk={} in={}",
                    (int)j, (int)endPointAddr, (int)endpoint->bmAttributes,
                    is_bulk ? 1 : 0,
                    (endPointAddr & LIBUSB_ENDPOINT_IN) ? 1 : 0);

      if (is_bulk && !(endPointAddr & LIBUSB_ENDPOINT_IN)) {
        _info.bulk_out_eps.push_back(endPointAddr);
      }
      /* First bulk IN endpoint wins. 8812AU/8814AU expose 0x81; 8821AU's
       * descriptor offers a different IN endpoint, so libusb's
       * submit_bulk_transfer to 0x81 would return "endpoint not found on any
       * open interface". Capture whatever IN endpoint the chip actually
       * exposes and use it in rx_raw()/rx_loop(). */
      if (is_bulk && (endPointAddr & LIBUSB_ENDPOINT_IN) && !found_bulk_in) {
        _info.bulk_in_ep = endPointAddr;
        found_bulk_in = true;
        _logger->info("selected bulk IN endpoint: 0x{:X}",
                      (int)_info.bulk_in_ep);
      }
    }
    if (!_info.bulk_out_eps.empty()) {
      std::string ep_list;
      for (auto ep : _info.bulk_out_eps) {
        char buf[8];
        snprintf(buf, sizeof(buf), "0x%02X ", ep);
        ep_list += buf;
      }
      _logger->info("bulk OUT endpoints: {}", ep_list);
    }
    /* Clear any HALT state on the bulk IN endpoint. The fwdl sequence and
     * USB reset can leave the IN EP in a stalled state from the chip side;
     * without clear_halt the chip's USB engine would never push RX bytes
     * even though the host's libusb_bulk_transfer succeeds at submission. */
    if (found_bulk_in) {
      int hr = libusb_clear_halt(_dev_handle, _info.bulk_in_ep);
      _logger->info("libusb_clear_halt(bulk IN 0x{:X}) rc={}",
                    (int)_info.bulk_in_ep, hr);
    }

    libusb_free_config_descriptor(config);
    break;
  }
}

void UsbTransport::transfer_callback(struct libusb_transfer *transfer) {
  auto *self = static_cast<UsbTransport *>(transfer->user_data);
  if (transfer->status == LIBUSB_TRANSFER_COMPLETED &&
      transfer->actual_length == transfer->length) {
    DVR_DEBUG(self->_logger, "Packet sent successfully, length: {}",
              transfer->length);
  } else {
    /* Re-clear_halt ONLY on a genuine endpoint STALL — the rare mid-stream
     * hardware stall (e.g. NDP generation on some xhci hosts) the wedge path
     * exists for. TX transfers use an infinite timeout, so a spurious
     * TIMED_OUT should not occur, and treating every non-OK completion as a
     * wedge would fire a ~ms clear_halt control transfer per hiccup — a
     * clear_halt storm — now that completions are actually reaped. */
    if (transfer->status == LIBUSB_TRANSFER_STALL)
      self->_tx_wedged.store(true, std::memory_order_relaxed);
    /* A CANCELLED completion is quiesce_tx doing its job, not a failure —
     * counting it would show phantom drops at the end of every session. */
    if (transfer->status != LIBUSB_TRANSFER_CANCELLED) {
      self->_tx_failed.fetch_add(1, std::memory_order_relaxed);
      self->_tx_last_rc.store(-transfer->status, std::memory_order_relaxed);
      self->_tx_last_timeout.store(
          transfer->status == LIBUSB_TRANSFER_TIMED_OUT,
          std::memory_order_relaxed);
      self->_logger->error(
          "Failed to send packet, status: {}, actual length: {}",
          transfer->status, transfer->actual_length);
      devourer::Ev(self->_logger->events(), "tx.fail")
          .f("status", (long long)transfer->status)
          .f("actual_len", transfer->actual_length)
          .f("timeout", transfer->status == LIBUSB_TRANSFER_TIMED_OUT);
    }
  }
  {
    std::lock_guard<std::mutex> lk(self->_tx_mu);
    auto &live = self->_tx_live;
    auto it = std::find(live.begin(), live.end(), transfer);
    if (it != live.end())
      live.erase(it);
  }
  /* The payload is transport-owned (allocated in tx_async) — free it with the
   * allocator that made it, never libusb's LIBUSB_TRANSFER_FREE_BUFFER: on
   * Windows a separately-linked libusb frees onto a different CRT heap. */
  std::free(transfer->buffer);
  libusb_free_transfer(transfer);
  /* Last, so a quiesce_tx loop that sees _tx_inflight == 0 knows every buffer
   * and transfer is already freed and nothing more will touch this object. */
  self->_tx_inflight.fetch_sub(1, std::memory_order_release);
}

/* Cancel + drain, called while the caller's libusb context is still alive.
 * See IRtlTransport::quiesce_tx for the contract. */
void UsbTransport::quiesce_tx() {
  if (_tx_shutdown.exchange(true, std::memory_order_acq_rel))
    return; /* already quiesced (Stop() then the destructor) */

  /* Snapshot, then cancel outside the lock — libusb_cancel_transfer can
   * complete the transfer inline, re-entering transfer_callback, which takes
   * the same mutex. Cancellation is asynchronous: SUCCESS means "unlink
   * submitted", so the drain below is what actually establishes quiescence.
   * NOT_FOUND means the transfer already completed — its callback owns the
   * bookkeeping, so there is nothing to do here. */
  std::vector<libusb_transfer *> pending;
  {
    std::lock_guard<std::mutex> lk(_tx_mu);
    pending = _tx_live;
  }
  for (auto *t : pending)
    libusb_cancel_transfer(t);

  /* Drain. Generous deadline: an unlink normally retires in microseconds (or
   * instantly with NO_DEVICE once the adapter is unplugged), and hanging
   * teardown forever is worse than reporting the anomaly. */
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (_tx_inflight.load(std::memory_order_acquire) > 0 &&
         std::chrono::steady_clock::now() < deadline) {
    struct timeval tv {0, 20000};
    if (libusb_handle_events_timeout_completed(_ctx, &tv, nullptr) != 0)
      break; /* context is gone or broken — nothing left to reap with */
  }

  const int residual = _tx_inflight.load(std::memory_order_acquire);
  if (residual > 0) {
    /* Proceeding means libusb_close will run with live transfers, which is
     * undefined behaviour — so say so loudly rather than fail silently. */
    _logger->error("TX quiesce timed out with {} transfer(s) still in flight — "
                   "the USB close that follows is unsafe",
                   residual);
    devourer::Ev(_logger->events(), "tx.quiesce_timeout").f("inflight", residual);
  }
}

bool UsbTransport::tx_async(uint8_t tx_ep, uint8_t *packet, size_t length,
                            unsigned timeout_ms) {
  /* Reap completed async-TX transfers before submitting the next one — in the
   * caller's own thread, so there is no background pump to race libusb
   * teardown. A non-blocking handle_events (timeout 0) processes every ready
   * completion, freeing those transfers and draining the kernel URB queue so
   * this submit doesn't fail with a full queue. This is what the vendor driver
   * gets for free from the kernel USB core; without it the queue fills, submits
   * fail, and TX collapses (issue #240). If the in-flight depth is already high
   * (a fast caller outrunning the air), block briefly to reap — bounded
   * backpressure that also caps latency. */
  /* Refused before the pump below, so a late send can't re-enter libusb after
   * teardown has begun. Counts as neither submitted nor failed: a frame handed
   * to us after we were told to stop is not a drop. */
  if (_tx_shutdown.load(std::memory_order_acquire))
    return false;
  {
    struct timeval zero {0, 0};
    libusb_handle_events_timeout_completed(_ctx, &zero, nullptr);
    /* Soft cap: keep at most kMaxInflight transfers pending. Beyond it, wait
     * for completions rather than pile onto the kernel queue. */
    constexpr int kMaxInflight = 256;
    while (_tx_inflight.load(std::memory_order_relaxed) >= kMaxInflight) {
      struct timeval tv {0, 2000};
      if (libusb_handle_events_timeout_completed(_ctx, &tv, nullptr) != 0)
        break; /* don't spin forever on a libusb error */
    }
  }

  libusb_transfer *transfer = libusb_alloc_transfer(0);
  if (!transfer) {
    _logger->error("Failed to allocate transfer");
    return false;
  }

  /* The URB outlives this call by definition (that is what "async" buys), so
   * it cannot reference the caller's buffer: every caller so far builds its
   * frame in a local that dies on return. Copy into a transport-owned block,
   * freed in transfer_callback. One memcpy per frame is far below the
   * libusb_alloc_transfer above it and the kernel's own copy at submit. */
  auto *payload = static_cast<uint8_t *>(std::malloc(length));
  if (!payload) {
    _logger->error("Failed to allocate TX payload ({} bytes)", length);
    libusb_free_transfer(transfer);
    return false;
  }
  std::memcpy(payload, packet, length);

  /* Recover a bulk-OUT that a prior async TX wedged (TIMED_OUT / stall). Only
   * the first send used to clear_halt; a mid-stream stall (e.g. hardware NDP
   * generation on some xhci hosts) then stayed wedged forever. */
  if (_tx_wedged.exchange(false)) {
    int hr = libusb_clear_halt(_dev_handle, tx_ep);
    _logger->info("TX EP 0x{:02X} re-clear_halt after wedge rc={}", (int)tx_ep,
                  hr);
  }

  /* On the FIRST send only, dump the bulk-OUT bytes to compare against
   * the OOT-driver wire trace. */
  static bool first_pkt_dump = true;
  if (first_pkt_dump) {
    first_pkt_dump = false;
    /* Clear any HALT state on the TX EP. The fwdl process can leave the
     * TX EP in a stalled state from the chip side; without clear_halt the
     * USB controller would NAK every subsequent bulk OUT URB. */
    int chr = libusb_clear_halt(_dev_handle, tx_ep);
    _logger->info("libusb_clear_halt(EP 0x{:02X}) rc={}", (int)tx_ep, chr);
    size_t dump_len = std::min<size_t>(length, 64);
    char hex[64 * 2 + 1] = {0};
    for (size_t k = 0; k < dump_len; ++k) {
      static const char hd[] = "0123456789abcdef";
      hex[2 * k] = hd[packet[k] >> 4];
      hex[2 * k + 1] = hd[packet[k] & 0xF];
    }
    _logger->info("first TX bulk-OUT len={} bytes: {}", length, hex);
  }

  /* On the FIRST send only, dump chip state via vendor reads. Surfaces any
   * register clobber between init-end and first TX (e.g. SetMonitorChannel
   * could be resetting REG_CR or related). */
  static bool first_dump = true;
  if (first_dump) {
    first_dump = false;
    uint16_t cr = ctrl_read<uint16_t>(0x0100);
    uint8_t txpause = ctrl_read<uint8_t>(0x0522);
    uint32_t txdma_off_chk = ctrl_read<uint32_t>(0x020C);
    uint32_t fwhw_txq = ctrl_read<uint32_t>(0x0420);
    uint32_t mcufwdl = ctrl_read<uint32_t>(0x0080);
    uint32_t hci_susp = ctrl_read<uint32_t>(0xFE10); /* USB suspend ctrl */
    _logger->info("pre-1st-TX: CR=0x{:04x} TXPAUSE=0x{:02x} TXDMA_OFFC=0x{:08x}",
                  cr, txpause, txdma_off_chk);
    _logger->info("pre-1st-TX: FWHW_TXQ=0x{:08x} MCUFWDL=0x{:08x} HCIPWR=0x{:08x}",
                  fwhw_txq, mcufwdl, hci_susp);
  }

  /* Infinite timeout (0), like the persistent-ring RX URBs: a TX transfer
   * waits until the chip accepts it — natural backpressure, not a drop — and
   * it matches the pre-reaping behaviour (nothing enforced timeouts before).
   * Over-submission is bounded by the in-flight soft cap above, not by
   * dropping frames on a timer. `timeout_ms` is kept for the sync path. */
  (void)timeout_ms;
  libusb_fill_bulk_transfer(transfer, _dev_handle, tx_ep, payload, length,
                            &UsbTransport::transfer_callback, (void *)this,
                            /*timeout=*/0);
  /* Upstream OOT (rtl8814a/usb/rtl8814au_xmit.c) sets URB_ZERO_PACKET on
   * every TX URB. libusb equivalent: LIBUSB_TRANSFER_ADD_ZERO_PACKET.
   * Without it the chip's SuperSpeed bulk OUT controller can wait
   * indefinitely for transfer-end signaling and NAK every URB until libusb
   * cancels — matches the usbmon trace we captured: 6977 submitted URBs,
   * every completion with status=-2 (ENOENT/cancelled), data_len=0. */
  transfer->flags |= LIBUSB_TRANSFER_ADD_ZERO_PACKET;
  /* Count the submission here; async completion (incl. TIMED_OUT) is counted
   * in transfer_callback, a submit error just below. */
  _tx_submitted.fetch_add(1, std::memory_order_relaxed);
  /* Register and count BEFORE submitting: the completion can be delivered from
   * another thread's pump the instant the URB is queued, and it must find its
   * own entry to remove. */
  {
    std::lock_guard<std::mutex> lk(_tx_mu);
    _tx_live.push_back(transfer);
  }
  _tx_inflight.fetch_add(1, std::memory_order_relaxed);
  int rc = libusb_submit_transfer(transfer);
  if (rc == LIBUSB_SUCCESS) {
    DVR_DEBUG(_logger, "Packet sent successfully, length: {}", length);
    return true;
  }
  /* Never submitted, so no callback will run — undo the bookkeeping here. */
  {
    std::lock_guard<std::mutex> lk(_tx_mu);
    auto it = std::find(_tx_live.begin(), _tx_live.end(), transfer);
    if (it != _tx_live.end())
      _tx_live.erase(it);
  }
  _tx_inflight.fetch_sub(1, std::memory_order_relaxed);
  _tx_failed.fetch_add(1, std::memory_order_relaxed);
  _tx_last_rc.store(rc, std::memory_order_relaxed);
  _tx_last_timeout.store(rc == LIBUSB_ERROR_TIMEOUT, std::memory_order_relaxed);
  _logger->error("Failed to send packet, error code: {}", rc);
  devourer::Ev(_logger->events(), "tx.fail")
      .f("rc", rc)
      .f("timeout", rc == LIBUSB_ERROR_TIMEOUT);
  std::free(payload);
  libusb_free_transfer(transfer);
  return false;
}

int UsbTransport::tx_sync(uint8_t ep, uint8_t *packet, size_t length,
                          int timeout_ms) {
  /* No libusb_clear_halt here. rtw88_8814au's usbmon shows the first bulk
   * OUT is preceded by 0 CLEAR_FEATUREs; later CLEAR_FEATUREs happen during
   * normal TX-queue operation, not the per-send hot path. Resetting the
   * data toggle bit corrupts the chip's state machine. */
  int actual = 0;
  _tx_submitted.fetch_add(1, std::memory_order_relaxed);
  int rc = libusb_bulk_transfer(_dev_handle, ep, packet,
                                static_cast<int>(length), &actual, timeout_ms);
  if (rc != LIBUSB_SUCCESS) {
    _tx_failed.fetch_add(1, std::memory_order_relaxed);
    _tx_last_rc.store(rc, std::memory_order_relaxed);
    _tx_last_timeout.store(rc == LIBUSB_ERROR_TIMEOUT,
                           std::memory_order_relaxed);
    _logger->error("bulk_send EP {} FAIL rc={} got {}/{}", (int)ep, rc, actual,
                   (int)length);
    return rc;
  }
  _logger->info("bulk_send EP {} OK {} bytes", (int)ep, actual);
  return actual;
}

TxStats UsbTransport::tx_stats() const {
  TxStats s;
  s.submitted = _tx_submitted.load(std::memory_order_relaxed);
  s.failed = _tx_failed.load(std::memory_order_relaxed);
  s.last_error_rc = _tx_last_rc.load(std::memory_order_relaxed);
  s.last_was_timeout = _tx_last_timeout.load(std::memory_order_relaxed);
  return s;
}

} /* namespace devourer */
