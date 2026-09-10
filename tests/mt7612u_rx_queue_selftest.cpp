/* Headless guard for the MT7612U RX hand-off queue
 * (src/mt7612u/Mt7612uRxQueue.h).
 *
 * The queue exists because delivering RX on the C library's event thread
 * wedges the part below the USB level, and the two properties that make it
 * safe are both invisible on a healthy bench: a full queue must drop the
 * NEWEST frame and count it (blocking is the wedge; dropping the oldest
 * reorders), and the slot pop_begin() returns must stay valid while the
 * consumer runs user code with the lock released. On hardware a broken
 * version of either looks like "the link is fine" right up until a slow
 * packet processor turns it into a silent reorder, a use-after-free, or a
 * replug. Hence a cell rather than a comment.
 *
 * What this does NOT cover: the ordering rules around the queue - ring before
 * receiver, quiesce before the drain is removed - which need a device. Those
 * are hand-run and recorded in docs/mt7612u.md. */
#include "mt7612u/Mt7612uRxQueue.h"

#include <atomic>
#include <cstdio>
#include <thread>
#include <vector>

using mt7612u::RxQueue;

namespace {

int fails;

void expect(const char *what, bool ok) {
  if (!ok) {
    std::fprintf(stderr, "mt7612u_rx_queue: FAIL %s\n", what);
    fails++;
  }
}

/* A frame whose every byte is `tag`, so a slot mix-up is visible in the
 * payload and not only in the attrib. `pkt_len` carries the true length, which
 * is what pop_tag checks the delivered payload against. */
bool push_tagged(RxQueue &q, uint8_t tag, size_t len = 8) {
  std::vector<uint8_t> frame(len, tag);
  rx_pkt_attrib a{};
  a.pkt_len = static_cast<uint16_t>(len);
  a.priority = tag; /* travels with the frame; checked on the way out */
  return q.push(a, frame.data(), frame.size());
}

/* Pops one frame and returns its tag, or a negative code:
 *   -1 nothing queued
 *   -2 the payload is empty
 *   -3 attrib and payload came from different frames
 *   -4 the payload is not the length the frame was pushed with
 *   -5 the payload was not tagged all the way to its LAST byte
 * -4 and -5 exist because the consumer's frame length comes only from
 * data.size() (Mt7612uRadio builds its span from it) while a processor that
 * trusts attrib.pkt_len would then read past the end. A push that truncated
 * every frame passed this cell until both were added. */
int pop_tag(RxQueue &q, const std::atomic<bool> &stop) {
  RxQueue::Slot *s = q.pop_begin(std::chrono::milliseconds(1), stop);
  if (!s)
    return -1;
  int tag = s->data.empty() ? -2 : s->data[0];
  if (tag >= 0 && s->attrib.priority != tag)
    tag = -3;
  if (tag >= 0 && s->data.size() != s->attrib.pkt_len)
    tag = -4;
  if (tag >= 0 && s->data.back() != s->data.front())
    tag = -5;
  q.pop_commit(s);
  return tag;
}

} // namespace

int main() {
  std::atomic<bool> stop{false};

  /* --- capacity is one short of the allocation ---
   * These four pin the SENTINEL-SLOT MECHANISM, which the header documents
   * (capacity() == slots - 1) but which is not the property the design rests
   * on. The property is "the producer never writes the slot the consumer is
   * holding" — asserted mechanism-independently further down, under "the
   * popped slot survives the producer filling the rest of the ring". A rewrite
   * that kept a count instead of a sentinel would fail these four and still be
   * correct; it would be changing the documented capacity, so they are here
   * deliberately, as the allocation's documentation and not as the guarantee. */
  {
    RxQueue q;
    q.reset(4);
    expect("reset(4) holds 3", q.capacity() == 3);
    expect("push 1 accepted", push_tagged(q, 1));
    expect("push 2 accepted", push_tagged(q, 2));
    expect("push 3 accepted", push_tagged(q, 3));
    /* The 4th must be refused: the tail slot is never written, which is what
     * lets the consumer hold a popped slot without the lock. */
    expect("push 4 refused (ring keeps one slot empty)", !push_tagged(q, 4));
    expect("refusal counted", q.dropped() == 1);
  }

  /* --- a full queue drops the NEWEST, never the oldest --- */
  {
    RxQueue q;
    q.reset(4);
    push_tagged(q, 10);
    push_tagged(q, 11);
    push_tagged(q, 12);
    expect("overflow frame refused", !push_tagged(q, 99));
    /* 99 is the frame that was dropped; 10, 11, 12 survive IN ORDER. A
     * drop-oldest queue would answer 11, 12, 99 here. */
    expect("oldest survives", pop_tag(q, stop) == 10);
    expect("order preserved (2nd)", pop_tag(q, stop) == 11);
    expect("order preserved (3rd)", pop_tag(q, stop) == 12);
    expect("the newest is the one that went missing",
           pop_tag(q, stop) == -1);
  }

  /* --- the ring wraps, and stays FIFO across the wrap --- */
  {
    RxQueue q;
    q.reset(4);
    for (int round = 0; round < 5; ++round) {
      const uint8_t tag = static_cast<uint8_t>(20 + round);
      expect("wrap: push accepted", push_tagged(q, tag));
      expect("wrap: same frame comes back", pop_tag(q, stop) == 20 + round);
    }
    expect("wrap: nothing dropped", q.dropped() == 0);
  }

  /* --- reset() rewinds AND zeroes the count --- */
  {
    RxQueue q;
    q.reset(2);
    expect("reset(2) holds 1", q.capacity() == 1);
    push_tagged(q, 1);
    expect("2nd refused at capacity 1", !push_tagged(q, 2));
    expect("count is 1 before reset", q.dropped() == 1);
    q.reset(8);
    expect("reset zeroes the drop count", q.dropped() == 0);
    expect("reset empties the ring", pop_tag(q, stop) == -1);
    expect("reset resizes", q.capacity() == 7);
  }

  /* --- a ring of one could never hold a frame: raised to exactly 2 --- */
  {
    RxQueue q;
    q.reset(1);
    expect("reset(1) is raised to the smallest usable ring",
           q.capacity() == 1);
    expect("reset(1) can still take a frame", push_tagged(q, 7));
    expect("reset(1) hands it back", pop_tag(q, stop) == 7);
    q.reset(0);
    expect("reset(0) likewise", q.capacity() == 1);
  }

  /* --- frames of different lengths arrive whole ---
   * The consumer's frame length comes only from data.size(); attrib.pkt_len is
   * what a processor would trust. A push that dropped a byte, or truncated to
   * one, passed every other cell here. */
  {
    RxQueue q;
    q.reset(8);
    for (size_t len : {size_t{1}, size_t{2}, size_t{60}, size_t{1500},
                       size_t{3836}}) {
      expect("varying length accepted", push_tagged(q, 70, len));
      RxQueue::Slot *s = q.pop_begin(std::chrono::milliseconds(1), stop);
      expect("varying length handed back", s != nullptr);
      if (s) {
        expect("payload is the length it was pushed with", s->data.size() == len);
        expect("payload is intact to its last byte",
               s->data.front() == 70 && s->data.back() == 70);
        q.pop_commit(s);
      }
    }
    /* A zero-length frame is a real shape on this wire (an ACK with the FCS
     * and the FCE trailer already stripped can reach here empty). It must be
     * queued, not silently dropped. */
    rx_pkt_attrib a{};
    const uint8_t empty_frame[1] = {0};
    expect("a zero-length frame is accepted", q.push(a, empty_frame, 0));
    RxQueue::Slot *s = q.pop_begin(std::chrono::milliseconds(1), stop);
    expect("a zero-length frame is handed back", s != nullptr);
    if (s) {
      expect("...as zero length", s->data.empty());
      q.pop_commit(s);
    }
  }

  /* --- an un-reset queue drops rather than dividing by zero --- */
  {
    RxQueue q;
    expect("un-reset capacity is 0", q.capacity() == 0);
    expect("un-reset push refused", !push_tagged(q, 1));
    expect("un-reset push counted", q.dropped() == 1);
    expect("un-reset pop is empty", pop_tag(q, stop) == -1);
    q.pop_commit(nullptr); /* must not fall off the end of an empty vector */
  }

  /* --- the popped slot survives the producer filling the rest of the ring ---
   * This is the property that lets user code run outside the queue lock. */
  {
    RxQueue q;
    q.reset(4);
    push_tagged(q, 30);
    RxQueue::Slot *held = q.pop_begin(std::chrono::milliseconds(1), stop);
    expect("held slot handed back", held != nullptr);
    if (held) {
      /* Fill every slot the producer is allowed to touch, plus one refused. */
      push_tagged(q, 31);
      push_tagged(q, 32);
      expect("producer stops at the held slot", !push_tagged(q, 33));
      expect("held payload untouched", held->data[0] == 30);
      expect("held attrib untouched", held->attrib.priority == 30);
      /* A commit for a slot that is no longer the tail must be a no-op, not a
       * skipped frame. Without the identity check a second commit here would
       * advance past tag 31 and lose it, silently and uncounted. */
      q.pop_commit(held);
      q.pop_commit(held);
      expect("a repeat commit does not skip the next frame",
             pop_tag(q, stop) == 31);
    }
    /* And the slot is only reusable after the commit. */
    expect("commit frees a slot", push_tagged(q, 34));
  }

  /* --- a commit for a slot that was never popped changes nothing --- */
  {
    RxQueue q;
    q.reset(4);
    push_tagged(q, 60);
    push_tagged(q, 61);
    RxQueue::Slot bogus{};
    q.pop_commit(&bogus);
    expect("a stray commit does not consume", pop_tag(q, stop) == 60);
    expect("...nor reorder what follows", pop_tag(q, stop) == 61);
  }

  /* --- a stale commit that has wrapped back ONTO the tail is still a no-op ---
   * The identity check alone does not cover this: hold a slot pointer, drain
   * the ring until the tail comes round to that same slot, and the pointer
   * matches again while the ring is empty. Advancing there desynchronises head
   * from tail, and the queue then hands out capacity-1 slots of stale bytes as
   * if they were frames. */
  {
    RxQueue q;
    q.reset(2); /* capacity 1, so the tail wraps every other frame */
    push_tagged(q, 80);
    RxQueue::Slot *first = q.pop_begin(std::chrono::milliseconds(1), stop);
    expect("wrap-stale: first slot handed back", first != nullptr);
    q.pop_commit(first);
    push_tagged(q, 81);
    RxQueue::Slot *second = q.pop_begin(std::chrono::milliseconds(1), stop);
    expect("wrap-stale: second slot handed back", second != nullptr);
    expect("wrap-stale: it is the other slot", second != first);
    q.pop_commit(second); /* tail is now back on `first`, ring empty */
    q.pop_commit(first);  /* the stale pointer matches again */
    expect("wrap-stale: ring is still empty", pop_tag(q, stop) == -1);
    expect("wrap-stale: and still usable", push_tagged(q, 82));
    expect("wrap-stale: delivering the right frame", pop_tag(q, stop) == 82);
  }

  /* --- `stop` ends the WAIT, never the DRAIN ---
   * The radio's consumer loop leaves only when pop_begin returns nullptr
   * (Mt7612uRadio::StartRxLoop), so a pop_begin that short-circuited on `stop`
   * would silently discard everything still queued at teardown - up to 63
   * frames. Deterministic on purpose: the concurrent block below happens to
   * catch that mutation most of the time, which is not the same as catching
   * it. */
  {
    RxQueue q;
    q.reset(8);
    std::atomic<bool> stopped{true}; /* set BEFORE a single pop */
    push_tagged(q, 50);
    push_tagged(q, 51);
    push_tagged(q, 52);
    expect("stop drains, 1st", pop_tag(q, stopped) == 50);
    expect("stop drains, 2nd", pop_tag(q, stopped) == 51);
    expect("stop drains, 3rd", pop_tag(q, stopped) == 52);
    expect("...and only then reports empty", pop_tag(q, stopped) == -1);
  }

  /* --- pop_begin returns on stop instead of waiting out its timeout --- */
  {
    RxQueue q;
    q.reset(4);
    std::atomic<bool> local_stop{false};
    const auto t0 = std::chrono::steady_clock::now();
    expect("empty queue times out",
           q.pop_begin(std::chrono::milliseconds(30), local_stop) == nullptr);
    const auto waited = std::chrono::steady_clock::now() - t0;
    expect("...having actually waited",
           waited >= std::chrono::milliseconds(25));

    local_stop.store(true);
    const auto t1 = std::chrono::steady_clock::now();
    expect("stop returns nullptr",
           q.pop_begin(std::chrono::seconds(30), local_stop) == nullptr);
    expect("...immediately, not after the timeout",
           std::chrono::steady_clock::now() - t1 <
               std::chrono::seconds(5));
  }

  /* --- wake() releases a waiter that is ALREADY blocked when stop flips ---
   * StopRxLoop sets the flag and then calls wake(). A waiter that entered
   * wait_for before the flag moved has no notification of its own coming, so
   * without the wake it sits out the rest of its timeout - 20 ms in the radio,
   * but the teardown then joins nothing and the contract "the ring is down
   * when StopRxLoop returns" is what pays for it. Timeout here is 30 s so a
   * missing wake fails the cell rather than passing slowly. */
  {
    RxQueue q;
    q.reset(4);
    std::atomic<bool> late_stop{false};
    std::atomic<bool> woke{false};
    std::thread waiter([&] {
      q.pop_begin(std::chrono::seconds(30), late_stop);
      woke.store(true);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const auto t0 = std::chrono::steady_clock::now();
    late_stop.store(true);
    q.wake();
    waiter.join();
    expect("waiter released by wake()", woke.load());
    expect("...promptly, not at the timeout",
           std::chrono::steady_clock::now() - t0 < std::chrono::seconds(5));
  }

  /* --- wake() releases EVERY waiter, which is what notify_all buys ---
   * One consumer today, so a notify_one would be invisible in the radio; the
   * header says "every waiter", and an untested "every" is a comment. */
  {
    RxQueue q;
    q.reset(4);
    std::atomic<bool> late_stop{false};
    std::atomic<int> released{0};
    std::thread a([&] {
      q.pop_begin(std::chrono::seconds(30), late_stop);
      released.fetch_add(1);
    });
    std::thread b([&] {
      q.pop_begin(std::chrono::seconds(30), late_stop);
      released.fetch_add(1);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const auto t0 = std::chrono::steady_clock::now();
    late_stop.store(true);
    q.wake();
    a.join();
    b.join();
    expect("both waiters released by one wake()", released.load() == 2);
    /* Without the time bound a notify_one still passes this cell - the second
     * waiter just sits out its 30 s timeout and then reports itself released. */
    expect("...both promptly, not one of them at the timeout",
           std::chrono::steady_clock::now() - t0 < std::chrono::seconds(5));
  }

  /* --- a push wakes a BLOCKED consumer, rather than leaving it to time out ---
   * Deleting the producer's notify passes every other cell in this file,
   * because they all poll. In the radio it would turn hand-off latency into
   * the 20 ms poll period, per frame, on a video link. */
  {
    RxQueue q;
    q.reset(4);
    std::atomic<bool> never{false};
    std::atomic<bool> got{false};
    const auto t0 = std::chrono::steady_clock::now();
    std::thread consumer([&] {
      RxQueue::Slot *s = q.pop_begin(std::chrono::seconds(30), never);
      if (s) {
        got.store(s->data[0] == 41);
        q.pop_commit(s);
      }
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    push_tagged(q, 41);
    consumer.join();
    expect("push delivered to a blocked consumer", got.load());
    expect("...on the notify, not on a timeout",
           std::chrono::steady_clock::now() - t0 < std::chrono::seconds(5));
  }

  /* --- concurrent producer and consumer: nothing is invented, nothing is
   *     lost except what the queue says it dropped --- */
  {
    RxQueue q;
    q.reset(8);
    constexpr int kFrames = 20000;
    std::atomic<bool> run_stop{false};
    std::atomic<int> received{0};
    std::atomic<int> out_of_order{0};

    std::thread consumer([&] {
      int last = -1;
      for (;;) {
        RxQueue::Slot *s = q.pop_begin(std::chrono::milliseconds(5), run_stop);
        if (!s) {
          if (run_stop.load())
            break;
          continue;
        }
        const int seq = s->attrib.pkt_len;
        if (seq <= last)
          out_of_order.fetch_add(1);
        last = seq;
        received.fetch_add(1);
        q.pop_commit(s);
      }
    });

    int pushed = 0;
    for (int i = 1; i <= kFrames; ++i) {
      std::vector<uint8_t> frame(16, static_cast<uint8_t>(i));
      rx_pkt_attrib a{};
      a.pkt_len = static_cast<uint16_t>(i); /* monotonic sequence */
      if (q.push(a, frame.data(), frame.size()))
        pushed++;
    }
    run_stop.store(true);
    q.wake();
    consumer.join();

    /* Every frame is either delivered or counted. Both counts are read after
     * the join, so neither is a snapshot of a moving value. */
    expect("delivered + dropped == offered",
           received.load() + static_cast<int>(q.dropped()) == kFrames);
    expect("accepted == delivered", pushed == received.load());
    expect("delivery is in order", out_of_order.load() == 0);
    /* Not an assertion about the drop count: at this speed the consumer may
     * or may not fall behind, and a cell that required drops would be a cell
     * that fails on a fast machine. */
  }

  if (fails) {
    std::fprintf(stderr, "mt7612u_rx_queue: %d failure(s)\n", fails);
    return 1;
  }
  std::printf("mt7612u_rx_queue: all checks passed\n");
  return 0;
}
