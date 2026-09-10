#ifndef MT7612U_RX_QUEUE_H
#define MT7612U_RX_QUEUE_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

#include "RxPacket.h"

namespace mt7612u {

/*
 * The hand-off between the C library's USB event thread and the thread that
 * called StartRxLoop.
 *
 * WHY IT EXISTS. The library's event thread is the SOLE servicer of both RX
 * and TX completions, so a packet processor that transmits - examples/chanmig
 * does, from its RX callback - would park that thread in mt_async_tx_submit
 * waiting for a TX slot only that same thread can free. MAC RX stays enabled,
 * EP4 stops being drained, and this part wedges below the USB level, where
 * libusb_reset_device, the sysfs authorized toggle and rebinding the kernel
 * driver all fail and only a physical replug recovers it. Anything that merely
 * takes the radio's control lock from the processor has a milder version of the
 * same problem: it stalls the drain for a 526 ms channel change.
 *
 * Running the processor on the StartRxLoop thread instead also restores the
 * contract every Realtek backend keeps, and keeps the library's "must not
 * block, must not call back" rule an internal invariant rather than one
 * silently exported to consumers. The copy costs ~2 MB/s at the measured
 * 1400 fps.
 *
 * TWO PROPERTIES THAT ARE NOT FREE TO CHANGE.
 *
 *  1. A full queue drops the NEWEST frame and counts it. Blocking the producer
 *     is precisely the wedge above; dropping the oldest would reorder frames
 *     under a consumer that is only briefly behind. The count is what keeps the
 *     loss visible instead of silent - StopRxLoop logs it.
 *
 *  2. pop_begin() hands back a pointer that stays valid, without the lock held,
 *     until pop_commit(). That is what lets user code run outside the queue
 *     lock, and it holds because the producer never writes the slot at the tail:
 *     one slot of the ring is always left empty, so `full` is one short of the
 *     allocation. Capacity is therefore slots - 1.
 */
class RxQueue {
public:
  struct Slot {
    std::vector<uint8_t> data;
    rx_pkt_attrib attrib{};
  };

  /* Allocate `slots` ring entries, rewind, and zero the drop count.
   *
   * Only safe while no producer is running AND no slot is outstanding: this
   * reallocates the ring, so it invalidates any pointer pop_begin() handed
   * back, and even a same-size reset empties every slot under a consumer still
   * reading one. The radio calls it before arming the ring, with the previous
   * session's event thread already joined - see StartRxLoop.
   *
   * `slots` counts the always-empty sentinel, so usable capacity is slots - 1;
   * values below 2 are raised to 2, since a ring of one could never hold a
   * frame and would drop the entire stream silently. */
  void reset(size_t slots) {
    if (slots < 2)
      slots = 2;
    std::lock_guard<std::mutex> lock(_mu);
    _q.assign(slots, Slot{});
    _head = _tail = 0;
    _dropped.store(0, std::memory_order_relaxed);
  }

  /* Producer side, called from the library's event thread. Returns false when
   * the frame was dropped, having counted it. */
  bool push(const rx_pkt_attrib &attrib, const uint8_t *frame, size_t len) {
    {
      std::lock_guard<std::mutex> lock(_mu);
      /* Before the modulo: an un-reset queue has no slots to write and a
       * `% 0` would be undefined rather than a drop. */
      if (_q.empty()) {
        _dropped.fetch_add(1, std::memory_order_relaxed);
        return false;
      }
      const size_t next = (_head + 1) % _q.size();
      if (next == _tail) {
        _dropped.fetch_add(1, std::memory_order_relaxed);
        return false;
      }
      Slot &slot = _q[_head];
      slot.attrib = attrib;
      slot.data.assign(frame, frame + len);
      _head = next;
    }
    _cv.notify_one();
    return true;
  }

  /* Consumer side. Waits up to `wait` for a frame; `stop` ends the WAIT early
   * but does not end the DRAIN - a queued frame is still handed back after
   * stop is set, and nullptr means "the ring is empty", never "we are
   * stopping". That is load-bearing: the radio's consumer loop leaves only on
   * a nullptr, so a pop_begin that short-circuited on `stop` would silently
   * discard whatever was still queued at teardown.
   *
   * The returned slot is valid, without the lock held, until pop_commit() -
   * the producer refuses at a full ring rather than writing the tail slot. */
  Slot *pop_begin(std::chrono::milliseconds wait, const std::atomic<bool> &stop) {
    std::unique_lock<std::mutex> lock(_mu);
    _cv.wait_for(lock, wait, [this, &stop] {
      return _head != _tail || stop.load();
    });
    if (_head == _tail)
      return nullptr;
    return &_q[_tail];
  }

  /* Releases the slot pop_begin() handed back - pass back the same pointer.
   *
   * Checking it is not ceremony: the pairing is otherwise enforced by nothing,
   * and a second commit for one pop would advance the tail past an
   * undelivered frame, silently and uncounted. Handing back a slot that is no
   * longer the tail is therefore a no-op rather than a skipped frame. */
  void pop_commit(const Slot *slot) {
    std::lock_guard<std::mutex> lock(_mu);
    if (_q.empty() || _head == _tail || slot != &_q[_tail])
      return;
    _tail = (_tail + 1) % _q.size();
  }

  /* Wakes every waiter without enqueuing anything - the stop path, so a
   * consumer usually returns at once instead of sitting out its timeout after
   * StopRxLoop. Usually, not always: the stop flag lives outside this lock, so
   * a consumer that evaluates the predicate just before the flag moves still
   * sleeps out the wait. The timeout stays the bound - 20 ms in the radio. */
  void wake() { _cv.notify_all(); }

  uint64_t dropped() const { return _dropped.load(std::memory_order_relaxed); }

  /* Frames the ring can hold at once. One short of the allocation; see
   * property 2 above. */
  size_t capacity() const {
    std::lock_guard<std::mutex> lock(_mu);
    return _q.empty() ? 0 : _q.size() - 1;
  }

private:
  std::vector<Slot> _q;
  size_t _head = 0; /* next slot to write */
  size_t _tail = 0; /* next slot to read  */
  std::atomic<uint64_t> _dropped{0};
  mutable std::mutex _mu;
  std::condition_variable _cv;
};

} // namespace mt7612u

#endif /* MT7612U_RX_QUEUE_H */
