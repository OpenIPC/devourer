/* Harq.h — M2 (#256) software HARQ for the scheduled downlink, with an
 * explicit UE-feedback protocol.
 *
 * Why software: on Kestrel there is no per-frame TX report (no CCX path), so
 * the DU cannot learn a hardware ACK's outcome; and on any chip a no-ACK
 * "TX complete" proves only that a frame aired. Delivery is therefore what the
 * UE says it received.
 *
 * Model. The sender numbers every new DL frame (32-bit seq; the HARQ process
 * id is seq % kHarqProcesses, carried for the receiver's diagnostics). Each
 * frame also carries `low`, the sender's window low edge: every seq below it
 * is resolved (delivered or given up), so the receiver may stop waiting for
 * it. The receiver keeps `base` (every seq < base is received or abandoned)
 * and a bitmap of the 32 seqs above it, and reports both as HarqFeedback.
 *
 * Sender rules (HarqSender):
 *   ACK   — a seq < feedback.base, or one whose bitmap bit is set.
 *   NACK  — an in-flight seq below the highest seq the feedback shows
 *           received: a later frame got through, this one did not. Judged by
 *           when the UE GENERATED the report (HarqFeedback::gen_sf), not when
 *           it arrived: only an attempt at least `nack_min_age_sf`
 *           superframes older than the report counts, so a delayed report
 *           that predates a retransmission never NACKs it.
 *   timeout — no ACK within `timeout_sf` superframes of its last attempt.
 *   A NACKed or timed-out seq is due for retransmission; due retransmissions
 *   go before new data, oldest first. After `max_attempts` it is dropped
 *   (counted) and `low` moves past it.
 *   The in-flight seq span (newest - oldest unresolved) stays <= max_span
 *   (at most 31: the newest is then bit 30 of a report based at the oldest),
 *   so a report always covers every in-flight seq; when the span is full only
 *   retransmissions are sent.
 * Receiver rules (HarqReceiver): duplicate suppression (a seq below base or
 * already in the bitmap is not delivered again), and `low` advances base past
 * abandoned gaps.
 * Pure header; cell_m2 ctest covers both state machines. */
#ifndef DEVOURER_CELL_HARQ_H
#define DEVOURER_CELL_HARQ_H

#include <cstddef>
#include <cstdint>
#include <deque>
#include <iterator>
#include <optional>

namespace devourer::cell {

constexpr uint32_t kHarqProcesses = 8;
constexpr uint32_t kHarqWindow = 32; /* bitmap width = max in-flight span */

/* What a UE reports: all seq < base received (or abandoned); bit i of bitmap
 * = seq base + 1 + i received (seq == base is, by definition, missing).
 * gen_sf: the (DU) superframe the UE generated the report in. */
struct HarqFeedback {
  uint32_t base = 0;
  uint32_t bitmap = 0;
  uint64_t gen_sf = 0;
  bool operator==(const HarqFeedback &) const = default;
};

/* Seq arithmetic that survives the 32-bit wrap (RFC 1982 style). */
inline bool seq_lt(uint32_t a, uint32_t b) { return static_cast<int32_t>(a - b) < 0; }

class HarqReceiver {
public:
  /* A received DL frame. Returns true when it is new (deliver it), false for
   * a duplicate. `low` is the sender's window low edge carried in the frame. */
  bool on_frame(uint32_t seq, uint32_t low) {
    if (!init_) {
      base_ = low;
      init_ = true;
    }
    advance_to(low);
    if (seq_lt(seq, base_))
      return false; /* already received or abandoned */
    const uint32_t off = seq - base_;
    if (off >= kHarqWindow + 1) {
      /* The sender's span is < kHarqWindow, so a seq this far ahead means the
       * receiver missed `low` moving (a lost frame carried it): catch up. */
      advance_to(seq - kHarqWindow);
      return on_frame(seq, low);
    }
    if (off == 0) {
      advance(base_ + 1, true); /* base itself just arrived */
      return true;
    }
    const uint32_t bit = 1u << (off - 1);
    if (bitmap_ & bit)
      return false;
    bitmap_ |= bit;
    return true;
  }
  /* The report as of superframe `sf` (the UE's view of the DU grid). */
  HarqFeedback feedback(uint64_t sf) const { return {base_, bitmap_, sf}; }
  bool started() const { return init_; }

private:
  /* Move base up to `to` (abandoning missing seqs below it), then on past
   * any run of received seqs, so base is again the lowest missing seq.
   * `base_have`: the current base was just received. Each step shifts the
   * bitmap: bit 0 (base + 1) becomes the new base. */
  void advance(uint32_t to, bool base_have) {
    bool have = base_have;
    while (seq_lt(base_, to) || have) {
      have = bitmap_ & 1u;
      bitmap_ >>= 1;
      base_ += 1;
    }
  }
  void advance_to(uint32_t to) { advance(to, false); }

  bool init_ = false;
  uint32_t base_ = 0;
  uint32_t bitmap_ = 0;
};

struct HarqConfig {
  int max_attempts = 4;   /* first transmission + 3 retransmissions */
  int timeout_sf = 3;     /* superframes without an ACK before a retransmission */
  int nack_min_age_sf = 1; /* a NACK applies to attempts this much older than the report */
  uint32_t max_span = kHarqWindow - 1; /* max newest - oldest unresolved seq (<= 31) */
};

struct HarqTx {
  uint32_t seq = 0;
  uint32_t pid = 0;     /* seq % kHarqProcesses */
  uint32_t low = 0;     /* the window low edge to put in the frame */
  int attempt = 1;      /* 1 = first transmission */
  bool retx = false;
  uint64_t prev_last_sf = 0; /* for unsend(): the entry's previous attempt time */
};

struct HarqStats {
  uint64_t new_tx = 0, retx = 0, acked = 0, dropped = 0, nacks = 0, timeouts = 0, implausible = 0;
};

class HarqSender {
public:
  explicit HarqSender(HarqConfig c = {}) : cfg_(c) {
    if (cfg_.max_span == 0 || cfg_.max_span >= kHarqWindow)
      cfg_.max_span = kHarqWindow - 1;
    if (cfg_.nack_min_age_sf < 0)
      cfg_.nack_min_age_sf = 0;
    if (cfg_.max_attempts < 1)
      cfg_.max_attempts = 1;
    if (cfg_.timeout_sf < 1)
      cfg_.timeout_sf = 1;
  }

  /* The frame to send in superframe `sf`'s DL slot: a due retransmission
   * first, else new data if `have_new` and the window has room, else none. */
  std::optional<HarqTx> next(uint64_t sf, bool have_new) {
    expire(sf);
    for (Entry &e : q_) {
      if (e.due) {
        const uint64_t prev = e.last_sf;
        e.due = false;
        e.attempts += 1;
        e.last_sf = sf;
        ++st_.retx;
        return HarqTx{e.seq, e.seq % kHarqProcesses, low(), e.attempts, true, prev};
      }
    }
    /* max_span is the largest permitted newest - oldest difference */
    if (!have_new || next_seq_ - low() > cfg_.max_span)
      return std::nullopt;
    Entry e;
    e.seq = next_seq_++;
    e.attempts = 1;
    e.last_sf = sf;
    q_.push_back(e);
    ++st_.new_tx;
    return HarqTx{e.seq, e.seq % kHarqProcesses, low(), 1, false};
  }

  /* The frame next() returned was NOT submitted (the radio refused it): undo
   * that attempt, so a failed submission never consumes a delivery attempt.
   * Only the most recent next() result may be passed. */
  void unsend(const HarqTx &tx) {
    for (auto it = q_.begin(); it != q_.end(); ++it) {
      if (it->seq != tx.seq)
        continue;
      if (tx.retx) {
        it->attempts -= 1;
        it->last_sf = tx.prev_last_sf;
        it->due = true; /* still owed */
        --st_.retx;
      } else if (std::next(it) == q_.end() && tx.seq + 1 == next_seq_) {
        q_.erase(it); /* the new frame never left: reuse its seq */
        --next_seq_;
        --st_.new_tx;
      }
      return;
    }
  }

  /* Apply a UE report (its gen_sf says when the UE observed it). A report
   * claiming a seq this sender never sent (base past the next new seq, or a
   * bitmap bit beyond it) is implausible — spoofed or corrupted — and is
   * ignored whole, so it cannot erase frames still in flight. */
  void on_feedback(const HarqFeedback &fb) {
    if (seq_lt(next_seq_, fb.base)) {
      ++st_.implausible;
      return;
    }
    for (int i = 0; i < static_cast<int>(kHarqWindow); ++i)
      if ((fb.bitmap & (1u << i)) && !seq_lt(fb.base + 1 + static_cast<uint32_t>(i), next_seq_)) {
        ++st_.implausible;
        return;
      }
    /* the highest seq the feedback shows received, if any above base */
    std::optional<uint32_t> top;
    for (int i = static_cast<int>(kHarqWindow) - 1; i >= 0; --i)
      if (fb.bitmap & (1u << i)) {
        top = fb.base + 1 + static_cast<uint32_t>(i);
        break;
      }
    for (auto it = q_.begin(); it != q_.end();) {
      const uint32_t s = it->seq;
      bool acked = seq_lt(s, fb.base);
      if (!acked && !seq_lt(s, fb.base + 1)) {
        const uint32_t off = s - fb.base - 1;
        acked = off < kHarqWindow && (fb.bitmap & (1u << off));
      }
      if (acked) {
        ++st_.acked;
        it = q_.erase(it);
        continue;
      }
      if (top && seq_lt(s, *top) && !it->due &&
          it->last_sf + static_cast<uint64_t>(cfg_.nack_min_age_sf) <= fb.gen_sf) {
        it->due = true; /* a later frame arrived, this one did not */
        ++st_.nacks;
      }
      ++it;
    }
  }

  /* Window low edge: the oldest unresolved seq, or the next new one. */
  uint32_t low() const { return q_.empty() ? next_seq_ : q_.front().seq; }
  size_t inflight() const { return q_.size(); }
  const HarqStats &stats() const { return st_; }

private:
  struct Entry {
    uint32_t seq = 0;
    int attempts = 0;
    uint64_t last_sf = 0;
    bool due = false;
  };
  /* Timeouts become due; exhausted entries are dropped. */
  void expire(uint64_t sf) {
    for (auto it = q_.begin(); it != q_.end();) {
      if (!it->due && sf >= it->last_sf + static_cast<uint64_t>(cfg_.timeout_sf)) {
        it->due = true;
        ++st_.timeouts;
      }
      if (it->due && it->attempts >= cfg_.max_attempts) {
        ++st_.dropped;
        it = q_.erase(it);
        continue;
      }
      ++it;
    }
  }

  HarqConfig cfg_;
  std::deque<Entry> q_;
  uint32_t next_seq_ = 0;
  HarqStats st_;
};

} /* namespace devourer::cell */

#endif /* DEVOURER_CELL_HARQ_H */
