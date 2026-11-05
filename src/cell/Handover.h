/* Handover.h — M6 (#260) the headless parts of mobility: what decides that
 * a UE moves cell to cell, and what keeps a duplicated uplink from being
 * delivered twice.
 *
 *   L3Filter    the per-(UE, cell) measurement filter: the 3GPP layer-3
 *               filter F_n = (1 - a) F_{n-1} + a M_n, a = 1 / 2^(k / 4),
 *               over the per-frame RX quality the DUs attribute to a UE (its
 *               uplink beacons). A filter not updated for `stale_ms` is not
 *               valid: a missing measurement is never a weak one.
 *   A3Trigger   the CHO-style condition (3GPP event A3): neighbour n beats
 *               the serving cell s by an offset,
 *                 enter  F_n - hyst > F_s + offset
 *                 leave  F_n + hyst < F_s + offset
 *               held continuously for time-to-trigger (one timer per
 *               neighbour; leaving the condition, or the neighbour going
 *               stale, resets it). After a handover the UE dwells
 *               `min_dwell_ms` before another can trigger (ping-pong guard).
 *               The serving measurement must be valid: a gap in it is no
 *               reason to move (radio-link failure is a separate path).
 *   MeasTable   one DU's filtered RSSI/SNR per (cell, UE): its own UEs and
 *               the other cells' it hears; what its F1 report carries.
 *   SeqDedup    sequence duplicate suppression for DAPS: the same uplink
 *               frame arriving over two cells is delivered once. A sliding
 *               window behind the highest sequence seen, in serial-number
 *               arithmetic (wraps at 2^32); anything older than the window
 *               is dropped (it cannot be told from a replay).
 *
 * Pure header; the cell_m6 ctest covers it, including a noisy cell-edge
 * crossing where raw-RSSI triggering ping-pongs and the filtered A3 moves
 * once. */
#ifndef DEVOURER_CELL_HANDOVER_H
#define DEVOURER_CELL_HANDOVER_H

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <map>
#include <optional>
#include <utility>
#include <vector>

namespace devourer::cell {

struct L3FilterConfig {
  int k = 4;               /* a = 1 / 2^(k/4); k = 0: no filtering */
  int64_t stale_ms = 2000; /* no sample this long: not valid */
};

class L3Filter {
public:
  explicit L3Filter(L3FilterConfig c = {}) : cfg_(c), a_(1.0 / std::pow(2.0, c.k / 4.0)) {}

  /* One measurement (dB/dBm) at a monotonic time. */
  void update(double m, int64_t now_ms) {
    /* The first sample seeds it, and so does the first after a stale gap: a
     * link that comes back is measured afresh, not blended with the past. */
    const bool fresh = have_ && now_ms - last_ms_ <= cfg_.stale_ms;
    v_ = fresh ? (1.0 - a_) * v_ + a_ * m : m;
    have_ = true;
    last_ms_ = now_ms;
  }
  bool valid(int64_t now_ms) const { return have_ && now_ms - last_ms_ <= cfg_.stale_ms; }
  double value() const { return v_; }
  int64_t last_ms() const { return last_ms_; }
  /* A long gap: the next sample seeds the filter afresh (no stale memory). */
  void reset() { have_ = false; }

private:
  L3FilterConfig cfg_;
  double a_;
  double v_ = 0;
  bool have_ = false;
  int64_t last_ms_ = 0;
};

struct A3Config {
  double offset_db = 3.0;
  double hysteresis_db = 1.0;
  int64_t ttt_ms = 640;       /* time-to-trigger */
  int64_t min_dwell_ms = 2000; /* after a handover, no new trigger this long */
};

class A3Trigger {
public:
  explicit A3Trigger(uint16_t serving, A3Config c = {}) : cfg_(c), serving_(serving) {}

  /* One evaluation. `serving` / `neigh` are the filtered measurements, each
   * with its validity. Returns the target cell when its condition has held
   * for time-to-trigger (the best such neighbour), else nullopt. */
  std::optional<uint16_t> evaluate(std::optional<double> serving,
                                   const std::map<uint16_t, std::optional<double>> &neigh,
                                   int64_t now_ms) {
    std::optional<uint16_t> best;
    double best_v = 0;
    for (const auto &kv : neigh) {
      const uint16_t cell = kv.first;
      if (cell == serving_)
        continue;
      auto it = since_.find(cell);
      if (!serving || !kv.second) { /* stale either side: no evidence */
        if (it != since_.end())
          since_.erase(it);
        continue;
      }
      const double n = *kv.second, s = *serving;
      const bool enter = n - cfg_.hysteresis_db > s + cfg_.offset_db;
      const bool leave = n + cfg_.hysteresis_db < s + cfg_.offset_db;
      if (it == since_.end()) {
        if (!enter)
          continue;
        it = since_.emplace(cell, now_ms).first; /* ttt 0: may fire right away */
      } else if (leave) {
        since_.erase(it);
        continue;
      }
      /* In the band between leave and enter the timer keeps running. */
      if (now_ms - it->second >= cfg_.ttt_ms && (!best || n > best_v)) {
        best = cell;
        best_v = n;
      }
    }
    for (auto it = since_.begin(); it != since_.end();) /* neighbours no longer reported */
      it = neigh.count(it->first) ? std::next(it) : since_.erase(it);
    if (best && have_ho_ && now_ms - last_ho_ms_ < cfg_.min_dwell_ms)
      return std::nullopt; /* dwell: the timers keep running, nothing fires */
    return best;
  }

  /* The handover to `cell` happened: it serves now, timers restart. */
  void handed_over(uint16_t cell, int64_t now_ms) {
    serving_ = cell;
    since_.clear();
    have_ho_ = true;
    last_ho_ms_ = now_ms;
  }
  uint16_t serving() const { return serving_; }
  /* When the condition toward `cell` started holding (for logs). */
  std::optional<int64_t> holding_since(uint16_t cell) const {
    auto it = since_.find(cell);
    if (it == since_.end())
      return std::nullopt;
    return it->second;
  }

private:
  A3Config cfg_;
  uint16_t serving_;
  std::map<uint16_t, int64_t> since_;
  bool have_ho_ = false;
  int64_t last_ho_ms_ = 0;
};

/* The filtered uplink quality per (cell, UE) one DU measures: its own UEs
 * (cell = its own id) and the other cells' it hears. RSSI and SNR each go
 * through an L3Filter; snapshot() lists only the valid (fresh) entries. */
struct MeasEntry {
  uint16_t cell = 0, ue = 0;
  double rssi_dbm = 0, snr_db = 0;
};
class MeasTable {
public:
  explicit MeasTable(L3FilterConfig c = {}) : cfg_(c) {}
  void update(uint16_t cell, uint16_t ue, double rssi_dbm, double snr_db, int64_t now_ms) {
    auto it = m_.find({cell, ue});
    if (it == m_.end())
      it = m_.emplace(std::make_pair(cell, ue), Pair{L3Filter(cfg_), L3Filter(cfg_)}).first;
    it->second.rssi.update(rssi_dbm, now_ms);
    it->second.snr.update(snr_db, now_ms);
  }
  /* Valid entries now; entries stale for 5x stale_ms are forgotten. */
  std::vector<MeasEntry> snapshot(int64_t now_ms) {
    std::vector<MeasEntry> v;
    for (auto it = m_.begin(); it != m_.end();) {
      if (now_ms - it->second.rssi.last_ms() > 5 * cfg_.stale_ms) {
        it = m_.erase(it);
        continue;
      }
      if (it->second.rssi.valid(now_ms))
        v.push_back({it->first.first, it->first.second, it->second.rssi.value(),
                     it->second.snr.value()});
      ++it;
    }
    return v;
  }

private:
  struct Pair {
    L3Filter rssi, snr;
  };
  L3FilterConfig cfg_;
  std::map<std::pair<uint16_t, uint16_t>, Pair> m_;
};

/* DAPS duplicate suppression (see the header). */
class SeqDedup {
public:
  static constexpr uint32_t kWindow = 256;

  /* true: new (deliver); false: a duplicate, or older than the window. */
  bool accept(uint32_t seq) {
    if (!init_) {
      init_ = true;
      top_ = seq;
      bits_.fill(0);
      set(0);
      return true;
    }
    const int32_t d = static_cast<int32_t>(seq - top_); /* serial arithmetic */
    if (d > 0) {
      shift(static_cast<uint32_t>(d));
      top_ = seq;
      set(0);
      return true;
    }
    const uint32_t back = static_cast<uint32_t>(-static_cast<int64_t>(d));
    if (back >= kWindow)
      return false;
    if (get(back))
      return false;
    set(back);
    return true;
  }
  uint32_t top() const { return top_; }

private:
  /* bit i = seq top_ - i seen */
  bool get(uint32_t i) const { return (bits_[i / 64] >> (i % 64)) & 1u; }
  void set(uint32_t i) { bits_[i / 64] |= uint64_t{1} << (i % 64); }
  void shift(uint32_t n) {
    if (n >= kWindow) {
      bits_.fill(0);
      return;
    }
    std::array<uint64_t, kWindow / 64> out{};
    for (uint32_t i = 0; i + n < kWindow; ++i)
      if (get(i))
        out[(i + n) / 64] |= uint64_t{1} << ((i + n) % 64);
    bits_ = out;
  }
  bool init_ = false;
  uint32_t top_ = 0;
  std::array<uint64_t, kWindow / 64> bits_{};
};

} /* namespace devourer::cell */

#endif /* DEVOURER_CELL_HANDOVER_H */
