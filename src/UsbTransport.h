#pragma once

/* UsbTransport — the libusb implementation of ITransport. Everything
 * USB-wire-specific that used to live inside the adapter is here: vendor
 * control transfers for the register plane, sync/async bulk-OUT TX with the
 * wedge (mid-stream stall) recovery and TX submission counters, the
 * kernel-rtw88-style async RX URB queue, and the interface-descriptor walk
 * that discovers the bulk endpoints. The exclusive per-adapter UsbDeviceLock
 * rides here too — its lifetime is the transport's. */

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

#include <libusb.h>

#include "DeviceConfig.h"
#include "Transport.h"
#include "logger.h"

namespace devourer {
class UsbDeviceLock;
}

#define REALTEK_USB_VENQT_READ 0xC0
#define REALTEK_USB_VENQT_WRITE 0x40
#define USB_TIMEOUT 500

namespace devourer {

class UsbTransport final : public ITransport {
public:
  UsbTransport(libusb_device_handle *dev_handle, Logger_t logger,
               libusb_context *ctx = nullptr,
               std::shared_ptr<devourer::UsbDeviceLock> usb_lock = nullptr,
               bool rx_zerocopy = true, RxMode rx_mode = RxMode::Async,
               int pool_spare = 0, int ring_ms = 0,
               PoolExhaust pool_exhaust = PoolExhaust::Backpressure,
               bool tx_no_cancel_multipkt = false);
  ~UsbTransport() override;

  bool is_usb() const override { return true; }

  uint8_t read8(uint16_t reg) override { return ctrl_read<uint8_t>(reg); }
  uint16_t read16(uint16_t reg) override { return ctrl_read<uint16_t>(reg); }
  uint32_t read32(uint16_t reg) override { return ctrl_read<uint32_t>(reg); }
  bool write8(uint16_t reg, uint8_t v) override { return ctrl_write(reg, v); }
  bool write16(uint16_t reg, uint16_t v) override { return ctrl_write(reg, v); }
  bool write32(uint16_t reg, uint32_t v) override { return ctrl_write(reg, v); }
  bool write32_wide(uint32_t addr, uint32_t v) override {
    /* Realtek USB register addressing: wValue = addr[15:0], wIndex =
     * addr[31:16]. Lets the BB/RF window (addr + 0x10000) reach wIndex=1
     * instead of colliding with the MAC/system space at wIndex=0. */
    /* A pipelined write that cannot be submitted (no usable slot, submit
     * rejected) falls through to the synchronous transfer below: EP0 keeps
     * submission order, so it lands behind whatever is still queued and no
     * register write is silently dropped. Transfers are counted where they
     * are issued (async_submit / here), so a fallback counts as its own. */
    if (_batch && async_write(static_cast<uint16_t>(addr & 0xFFFF),
                              static_cast<uint16_t>(addr >> 16), &v, sizeof(v)))
      return true;
    _ctrl_xfers.fetch_add(1, std::memory_order_relaxed);
    const bool ok =
        libusb_control_transfer(
               _dev_handle, REALTEK_USB_VENQT_WRITE, 5,
               static_cast<uint16_t>(addr & 0xFFFF),
               static_cast<uint16_t>(addr >> 16), (uint8_t *)&v, sizeof(v),
               USB_TIMEOUT) == static_cast<int>(sizeof(v));
    /* Only a fallback that ALSO failed is a batch write error; a refused
     * async submit that the synchronous path completed is not. */
    if (!ok && _batch_open)
      _aw->write_errors++;
    return ok;
  }
  uint32_t read32_wide(uint32_t addr) override {
    uint32_t data = 0;
    if (_batch) {
      if (async_read(static_cast<uint16_t>(addr & 0xFFFF),
                     static_cast<uint16_t>(addr >> 16), &data, sizeof(data)))
        return data;
      /* A pipelined read that could not be submitted or completed is a
       * queue problem, not a register problem: say so, then read it
       * synchronously below — EP0 order still places that read behind
       * whatever is queued — so only a real transfer failure yields the
       * all-ones sentinel. */
      _logger->error("USB: pipelined read32_wide(0x{:05x}) failed; reading "
                     "synchronously", addr);
    }
    _ctrl_xfers.fetch_add(1, std::memory_order_relaxed);
    if (libusb_control_transfer(_dev_handle, REALTEK_USB_VENQT_READ, 5,
                                static_cast<uint16_t>(addr & 0xFFFF),
                                static_cast<uint16_t>(addr >> 16),
                                (uint8_t *)&data, sizeof(data),
                                USB_TIMEOUT) == static_cast<int>(sizeof(data)))
      return data;
    return 0xFFFFFFFFu; /* INVALID_RF_DATA-style sentinel on a failed read */
  }
  bool write_bytes(uint16_t reg, const uint8_t *p, size_t n) override;
  void write_batch_begin() override;
  bool write_batch_end() override;
  void flush_writes() override;
  uint64_t ctrl_xfers() const override {
    return _ctrl_xfers.load(std::memory_order_relaxed);
  }

  bool tx_async(uint8_t ep, uint8_t *buf, size_t len,
                unsigned timeout_ms) override;
  int tx_sync(uint8_t ep, uint8_t *buf, size_t len, int timeout_ms) override;
  int tx_sync_data(uint8_t ep, uint8_t *buf, size_t len,
                   int timeout_ms) override;
  void rx_loop(int buf_size, int n_urbs,
               const std::function<void(const uint8_t *, int)> &on_data,
               const std::function<bool()> &should_stop) override;
  int rx_raw(uint8_t *buf, int len, int timeout_ms) override;
  void clear_halt(uint8_t ep) override { libusb_clear_halt(_dev_handle, ep); }
  void quiesce_tx() override;

  UsbLinkInfo usb_info() const override { return _info; }
  TxStats tx_stats() const override;

private:
  template <typename T> T ctrl_read(uint16_t reg);
  template <typename T> bool ctrl_write(uint16_t reg, T value);
  /* Pipelined-write machinery (see ITransport::write_batch_begin). */
  /* Register transfers are 1/2/4 bytes; async_write/async_read refuse a
   * larger payload rather than overrun the inline setup buffer. */
  static constexpr size_t kAsyncMaxPayload = 4;
  /* Bookkeeping the completion callback writes to. It is owned jointly by
   * the transport and every slot (shared_ptr), not by the transport alone:
   * a slot that could not be reaped outlives the transport (see the
   * destructor), and its callback may still fire later through a libusb
   * context another adapter in the process keeps pumping. It then updates
   * this block, which the leaked slot keeps alive, instead of a freed
   * UsbTransport. */
  struct AsyncWrite;
  /* The submitter and the completion callback are normally the same thread
   * (this transport pumps its own events while it waits), but a second
   * adapter sharing the libusb context can pump it from another thread and
   * run the callback there, right after libusb_submit_transfer returns. So
   * the counters are atomic, the free list is under a mutex, and a slot is
   * marked in flight BEFORE it is submitted (rolled back if the submit is
   * refused) — the callback never sees a completed slot that the submitter
   * has not yet accounted for. */
  struct AsyncPool {
    std::mutex mu; /* guards `free` */
    std::vector<AsyncWrite *> free;
    std::atomic<int> inflight{0};
    std::atomic<uint64_t> completed{0};
    /* Failed/short WRITE completions, write submit refusals and retired
     * slots: what write_batch_end reports. A failed read is reported to its
     * caller directly (false / throw) and is NOT counted here — a read
     * glitch that the caller retries and recovers must not fail the batch. */
    std::atomic<int> write_errors{0};
    /* Bumped by every write_batch_begin; a slot carries the generation it
     * was submitted under, and a completion from an older generation (a
     * slot retired by a drain that finishes late) never touches the current
     * batch's verdict — it was already counted when it was retired. */
    std::atomic<uint64_t> generation{0};
  };
  struct AsyncWrite {
    libusb_transfer *t;
    uint8_t buf[LIBUSB_CONTROL_SETUP_SIZE + kAsyncMaxPayload];
    std::shared_ptr<AsyncPool> pool;
    std::atomic<bool> done{false};
    /* Submitted and not yet reaped: libusb owns `t` and `buf` while set, so
     * the slot must not be reused, freed, or handed back to the free list. */
    std::atomic<bool> inflight{false};
    /* The completion callback is inside the slot: set first thing on entry,
     * cleared as its very last store. `inflight` has to clear before the
     * free-list push (a taker must see a finished slot), so it cannot double
     * as the destructor's "safe to free" signal — this is. */
    std::atomic<bool> cb_busy{false};
    bool is_read = false; /* set before submit; decides which failure it is */
    uint64_t gen = 0;     /* batch generation the slot was submitted under */
    int status = -1;
    int actual = 0;
  };
  static constexpr int kAsyncWriteDepth = 8;
  bool async_write(uint16_t wvalue, uint16_t windex, const void *data,
                   size_t n);
  /* Read queued behind the pending writes (EP0 order) and waited for on its
   * own completion only: a read-modify-write pair costs one wakeup, not two.
   * Returns false on failure (data untouched). */
  bool async_read(uint16_t wvalue, uint16_t windex, void *data, size_t n);
  AsyncWrite *async_take_slot();
  bool async_submit(AsyncWrite *w); /* in-flight accounting before submit */
  /* Pump until this pool's completion counter moves past `before` (a
   * snapshot the caller took BEFORE checking whatever it is waiting for,
   * so a completion landing in between is not missed); false on a 2 s
   * deadline or an event-loop error. */
  bool async_wait_progress(uint64_t before);
  bool pump_once(int ms);      /* one bounded handle_events turn; false on error */
  static void LIBUSB_CALL async_write_cb(libusb_transfer *t);
  /* `_batch`: pipelined submission is enabled right now. `_batch_open`: the
   * caller's batch is open. They part when a drain retires slots — that
   * disables pipelining at once but must keep the batch's verdict (its
   * write errors) for write_batch_end to return. */
  bool _batch = false;
  bool _batch_open = false;
  int _batch_depth = 0; /* begin/end nest; the outermost pair owns the verdict */
  std::shared_ptr<AsyncPool> _aw = std::make_shared<AsyncPool>();
  std::vector<AsyncWrite *> _aw_all;
  /* Set when a drain gave up with transfers still submitted: the destructor
   * then leaks those slots instead of freeing a transfer libusb still owns. */
  bool _aw_abandoned = false;
  /* Vendor control transfers (register reads + writes) this transport has
   * issued; per instance, so two adapters in one process do not
   * cross-attribute their InitTimer stage counts. */
  std::atomic<uint64_t> _ctrl_xfers{0};
  bool _tx_no_cancel_multipkt = false; /* DeviceConfig::Tx; tx_sync_data only */
  /* Smallest bulk-OUT wMaxPacketSize, from discover_endpoints: what
   * tx_sync_data's never-cancel rule measures a transfer against
   * (src/BulkOutTimeout.h). 0 = unknown. */
  unsigned _bulk_out_mps = 0;
  void discover_endpoints(); /* was InitDvObj */
  const char *speed_str() const;
  static void transfer_callback(struct libusb_transfer *transfer);

  libusb_device_handle *_dev_handle;
  libusb_context *_ctx = nullptr;
  Logger_t _logger;
  UsbLinkInfo _info;

  /* Set by transfer_callback when an async TX bulk-OUT completes non-OK
   * (TIMED_OUT / stall). Consumed at the top of the next tx_async on the TX
   * thread to re-clear_halt the endpoint — a mid-stream stall (e.g. hardware
   * NDP generation on some xhci hosts) would otherwise stay wedged, since the
   * first-send clear_halt only runs once. */
  std::atomic<bool> _tx_wedged{false};

  /* TX submission counters (the driver-drop / congestion signal, TxStats.h).
   * The async transfer_callback increments them from the libusb event thread. */
  std::atomic<uint64_t> _tx_submitted{0};
  std::atomic<uint64_t> _tx_failed{0};
  std::atomic<int> _tx_last_rc{0};
  std::atomic<bool> _tx_last_timeout{false};

  /* Async-TX completions must be reaped by libusb_handle_events or the kernel
   * URB queue fills, submits start failing, and TX throughput collapses (the
   * Jaguar1 issue #240: its send path is tx_async and a TX-only session has no
   * other event pump). We reap in the CALLER's thread — each tx_async drains
   * completed transfers before submitting the next — rather than a background
   * pump thread, which would race the caller-owned libusb teardown (an earlier
   * attempt crashed on a usbi_mutex assertion). _tx_inflight tracks
   * submitted-but-not-yet-reaped transfers so the destructor can drain them
   * before the device handle / context go away, and so a soft cap can throttle
   * over-submission. */
  std::atomic<int> _tx_inflight{0};

  /* Submitted-but-not-yet-completed transfers, so quiesce_tx can cancel them
   * by handle. transfer_callback removes its own entry, and it runs on
   * whichever thread pumped the event — the submitting one, or another
   * tx_async caller under DEVOURER_TX_THREADS. Never hold _tx_mu across a
   * libusb_handle_events call: the callback re-enters and takes it. */
  std::mutex _tx_mu;
  std::vector<libusb_transfer *> _tx_live;

  /* Latched by quiesce_tx. Refuses further submissions (and further event
   * pumping) so nothing re-enters libusb once teardown has begun, and makes
   * quiesce idempotent for the Stop()-then-destructor path. */
  std::atomic<bool> _tx_shutdown{false};

  /* Allocate the async RX ring from kernel DMA memory (dev_mem_alloc) for a
   * zerocopy bulk-IN path; falls back to heap buffers per-URB when the alloc is
   * unsupported. See rx_loop and DeviceConfig::Usb::rx_zerocopy. */
  bool _rx_zerocopy = true;

  /* RX-ring servicing strategy + buffer-pool depth + diagnostic telemetry
   * cadence, from DeviceConfig::Rx. rx_loop reads these; the defaults preserve
   * the historic inline async ring with no extra buffers and no telemetry. */
  RxMode _rx_mode = RxMode::Async;
  int _pool_spare = 0;
  int _ring_ms = 0;
  PoolExhaust _pool_exhaust = PoolExhaust::Backpressure;

  /* rx_loop helpers for the servicing strategies dispatched off _rx_mode. */
  void rx_loop_sync(int buf_size,
                    const std::function<void(const uint8_t *, int)> &on_data,
                    const std::function<bool()> &should_stop);

  /* Exclusive per-adapter USB lock (UsbDeviceLock.h), held for the transport
   * lifetime; released when the device (and thus the transport) dies. */
  std::shared_ptr<devourer::UsbDeviceLock> _usb_lock;
};

template <typename T> T UsbTransport::ctrl_read(uint16_t reg_num) {
  T data = 0;
  if (_batch) {
    if (async_read(reg_num, 0, &data, sizeof(T)))
      return data;
    /* Fall through to the synchronous read (see read32_wide): only a
     * transfer that fails there too throws. */
    _logger->error("rtw_read({:04x}) pipelined failed; reading synchronously",
                   reg_num);
  }
  _ctrl_xfers.fetch_add(1, std::memory_order_relaxed);
  if (libusb_control_transfer(_dev_handle, REALTEK_USB_VENQT_READ, 5, reg_num,
                              0, (uint8_t *)&data, sizeof(T),
                              USB_TIMEOUT) == sizeof(T)) {
    return data;
  }
  _logger->error("rtw_read({:04x}), sizeof(T) = {}", reg_num, sizeof(T));
  throw std::ios_base::failure("rtw_read");
  return 0;
}

template <typename T> bool UsbTransport::ctrl_write(uint16_t reg_num, T value) {
  /* Unsubmittable pipelined write -> synchronous, in order (see write32_wide). */
  if (_batch && async_write(reg_num, 0, &value, sizeof(T)))
    return true;
  _ctrl_xfers.fetch_add(1, std::memory_order_relaxed);
  const bool ok =
      libusb_control_transfer(_dev_handle, REALTEK_USB_VENQT_WRITE, 5, reg_num,
                              0, (uint8_t *)&value, sizeof(T),
                              USB_TIMEOUT) == sizeof(T);
  if (!ok && _batch_open)
    _aw->write_errors++; /* the fallback failed too (see write32_wide) */
  return ok;
}

} /* namespace devourer */
