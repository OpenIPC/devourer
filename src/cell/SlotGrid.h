/* SlotGrid.h — the M1 (#255) fixed-grid slot plan on a disciplined TSF.
 *
 * One superframe = one beacon interval (an integer number of TU, so the TBTT
 * grid and the slot grid are the same grid). It starts with the beacon slot,
 * then one DL slot per UE, an explicit DL->UL turnaround guard, then one UL
 * slot per UE (M1's restricted numerology: fixed superframe, one periodic UL
 * opportunity per UE, no channel or bandwidth switching).
 *
 * Every slot length is a full budget, not a payload size (EPIC #254
 * "slot-budget accounting"):
 *   attempts x (PSDU airtime + [SIFS + ACK airtime])
 *   + (attempts - 1) x retry gap      (ACK timeout + DIFS + backoff)
 *   + radio turnaround + timing guard (the M0-measured departure guard)
 * Airtime is the 802.11a/g legacy-OFDM formula (5 GHz, 20 MHz): 20 us of
 * preamble + SIGNAL, then 4 us symbols of 24*rate/6 data bits carrying
 * SERVICE(16) + 8*len + tail(6). M1 schedules legacy rates only; HT/HE
 * airtime is not modelled here.
 *
 * Phase math: the grid is anchored at tsf % period == offset_us (the cell's
 * TBTT phase, e.g. PHASE_US of tests/pcie_ptp_tsf_discipline). Pure header,
 * no radio dependency; cell_selftest covers it. */
#ifndef DEVOURER_CELL_SLOTGRID_H
#define DEVOURER_CELL_SLOTGRID_H

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace devourer::cell {

constexpr int64_t kTuUs = 1024;
constexpr int64_t kSifsUs = 16;      /* 5 GHz OFDM */
constexpr int64_t kSlotTimeUs = 9;   /* 5 GHz OFDM */
/* UEs one cell's grid may carry: the grant map (GrantMap.h) advertises at
 * most this many grants in one beacon IE. */
constexpr size_t kMaxUesPerCell = 24;
/* Idle air a RACH slot must leave before the next beacon: a joiner's msgA
 * can depart up to ~4.8 ms late on Kestrel (src/cell/Rach.h). */
constexpr int64_t kRachTailGapUs = 5000;
/* Input bounds that keep slot_len_us's int64 arithmetic exact: an MPDU up to
 * the 802.11ac maximum (11454 bytes), a retry ladder far past any hardware
 * limit, margins up to 10 s. */
constexpr size_t kMaxMpduLen = 11454;
constexpr int kMaxAttempts = 64;
constexpr int64_t kMaxMarginUs = 10'000'000;

/* Legacy-OFDM PPDU airtime (us) for an MPDU of `mpdu_len` bytes including the
 * FCS, at `rate_mbps` in {6,9,12,18,24,36,48,54}. Returns 0 for other rates. */
inline int64_t ofdm_airtime_us(size_t mpdu_len, int rate_mbps) {
  switch (rate_mbps) {
  case 6: case 9: case 12: case 18: case 24: case 36: case 48: case 54: break;
  default: return 0;
  }
  const int64_t ndbps = 4 * rate_mbps; /* data bits per 4 us symbol */
  const int64_t bits = 16 + 8 * static_cast<int64_t>(mpdu_len) + 6;
  return 20 + 4 * ((bits + ndbps - 1) / ndbps);
}

/* What one slot must carry, and the margins around it. */
struct SlotBudget {
  size_t mpdu_len = 0;       /* bytes incl. FCS */
  int rate_mbps = 6;
  bool acked = false;        /* unicast with a hardware ACK */
  int ack_rate_mbps = 6;     /* control-response rate */
  int attempts = 1;          /* 1 + retry limit the slot must absorb */
  int64_t retry_gap_us = 0;  /* per retry: ACK timeout + DIFS + backoff */
  int64_t turnaround_us = 0; /* radio RX<->TX switch */
  int64_t guard_us = 0;      /* departure/sync guard (M0 contract) */
};

/* Total slot length (us) for a budget; 0 (invalid) for an unsupported rate,
 * attempts outside 1..kMaxAttempts, an MPDU over kMaxMpduLen, or a margin
 * outside 0..kMaxMarginUs — a negative margin would silently under-budget
 * the slot. */
inline int64_t slot_len_us(const SlotBudget &b) {
  auto margin_ok = [](int64_t v) { return v >= 0 && v <= kMaxMarginUs; };
  if (b.attempts < 1 || b.attempts > kMaxAttempts || b.mpdu_len > kMaxMpduLen ||
      !margin_ok(b.retry_gap_us) || !margin_ok(b.turnaround_us) ||
      !margin_ok(b.guard_us))
    return 0;
  const int64_t psdu = ofdm_airtime_us(b.mpdu_len, b.rate_mbps);
  if (psdu == 0)
    return 0;
  int64_t per = psdu;
  if (b.acked) {
    const int64_t ack = ofdm_airtime_us(14, b.ack_rate_mbps); /* ACK + FCS */
    if (ack == 0)
      return 0;
    per += kSifsUs + ack;
  }
  return b.attempts * per + (b.attempts - 1) * b.retry_gap_us +
         b.turnaround_us + b.guard_us;
}

enum class SlotKind : uint8_t { Beacon = 0, Downlink = 1, Turnaround = 2, Uplink = 3, Idle = 4,
                                Rach = 5 /* contention: joining UEs (M3, Rach.h) */ };

struct Slot {
  SlotKind kind = SlotKind::Idle;
  uint16_t ue = 0;       /* UE id for DL/UL slots, 0 otherwise */
  int64_t start_us = 0;  /* from the superframe start (the TBTT) */
  int64_t len_us = 0;
};

class SlotGrid {
public:
  /* period_tu: the beacon interval; offset_us: the TBTT phase on the TSF. */
  SlotGrid(int period_tu, int64_t offset_us)
      : period_us_(static_cast<int64_t>(period_tu) * kTuUs),
        offset_us_(period_us_ > 0 ? ((offset_us % period_us_) + period_us_) % period_us_ : 0) {}

  int64_t period_us() const { return period_us_; }
  int64_t offset_us() const { return offset_us_; }
  const std::vector<Slot> &slots() const { return slots_; }

  /* Append a slot after the last one. False (grid unchanged) when it would
   * overrun the superframe or len_us <= 0. */
  bool append(SlotKind kind, uint16_t ue, int64_t len_us) {
    const int64_t start = used_us();
    if (len_us <= 0 || start + len_us > period_us_)
      return false;
    slots_.push_back({kind, ue, start, len_us});
    return true;
  }
  int64_t used_us() const {
    return slots_.empty() ? 0 : slots_.back().start_us + slots_.back().len_us;
  }

  /* Superframe number and position for a 64-bit TSF (us). */
  struct At {
    int64_t superframe = 0;  /* floor((tsf - offset) / period) */
    int64_t into_us = 0;     /* 0 .. period-1 */
    std::optional<size_t> slot; /* index into slots(), or nullopt (idle tail) */
  };
  At at(int64_t tsf) const {
    At a;
    if (period_us_ <= 0)
      return a;
    const int64_t rel = tsf - offset_us_;
    a.superframe = rel >= 0 ? rel / period_us_ : -((-rel + period_us_ - 1) / period_us_);
    a.into_us = rel - a.superframe * period_us_;
    for (size_t i = 0; i < slots_.size(); ++i)
      if (a.into_us >= slots_[i].start_us &&
          a.into_us < slots_[i].start_us + slots_[i].len_us) {
        a.slot = i;
        break;
      }
    return a;
  }

  /* TSF (us) at which slot `idx` of superframe `sf` starts. */
  int64_t slot_start_tsf(int64_t sf, size_t idx) const {
    return offset_us_ + sf * period_us_ + (idx < slots_.size() ? slots_[idx].start_us : 0);
  }

  /* First slot of `kind` for `ue`, or nullopt. */
  std::optional<size_t> find(SlotKind kind, uint16_t ue) const {
    for (size_t i = 0; i < slots_.size(); ++i)
      if (slots_[i].kind == kind && slots_[i].ue == ue)
        return i;
    return std::nullopt;
  }

private:
  int64_t period_us_;
  int64_t offset_us_;
  std::vector<Slot> slots_;
};

/* The M1 layout: beacon slot, one DL slot per UE, a turnaround guard, one UL
 * slot per UE, in UE-list order, then (rach_slot_us > 0, M3) one contention
 * slot for joining UEs. nullopt when it does not fit the period, or when
 * there are more UEs than one grant map can advertise (kMaxUesPerCell). */
inline std::optional<SlotGrid> build_fixed_grid(int period_tu, int64_t offset_us,
                                                int64_t beacon_slot_us,
                                                const std::vector<uint16_t> &ues,
                                                int64_t dl_slot_us,
                                                int64_t turnaround_us,
                                                int64_t ul_slot_us,
                                                int64_t rach_slot_us = 0) {
  if (ues.size() > kMaxUesPerCell)
    return std::nullopt;
  SlotGrid g(period_tu, offset_us);
  if (!g.append(SlotKind::Beacon, 0, beacon_slot_us))
    return std::nullopt;
  for (uint16_t ue : ues)
    if (!g.append(SlotKind::Downlink, ue, dl_slot_us))
      return std::nullopt;
  if (!g.append(SlotKind::Turnaround, 0, turnaround_us))
    return std::nullopt;
  for (uint16_t ue : ues)
    if (!g.append(SlotKind::Uplink, ue, ul_slot_us))
      return std::nullopt;
  if (rach_slot_us > 0 && !g.append(SlotKind::Rach, 0, rach_slot_us))
    return std::nullopt;
  /* A RACH slot needs kRachTailGapUs of idle air before the next beacon: a
   * joiner's msgA can depart that late (Rach.h), and must not hit it. */
  if (rach_slot_us > 0 && g.used_us() + kRachTailGapUs > g.period_us())
    return std::nullopt;
  return g;
}

} /* namespace devourer::cell */

#endif /* DEVOURER_CELL_SLOTGRID_H */
