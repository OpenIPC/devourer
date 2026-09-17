/* DwellExecutor — one channel-survey dwell, driven by the caller.
 *
 * This was the loop in examples/chanscout. It moved because a survey executor
 * is the thing an integrator needs and the thing no test could reach: the
 * scheduler has a selftest and so does the consumer, but the retune/barrier/
 * observe sequencing between them was demo code, so a regression in it was
 * findable only on air.
 *
 * Three phases, because the caller owns the sleeps (see SenseWindow.h):
 *
 *   plan = sched.next(now_ms);
 *   if (!exec.begin(plan, now_ms, d)) { emit(d); sched.complete(plan, ..., false); }
 *   truncated  = !caller_sleep(settle);
 *   exec.barrier(now_ms);
 *   truncated |= !caller_sleep(dwell);
 *   exec.finish(truncated, d);
 *   emit(d); sched.complete(plan, d.t_end_ms, ok);
 *
 * It owns exactly one piece of cross-dwell state, and it is the subtle one:
 * FastRetune is the lean SAME-WIDTH path, so the first bin dwell after a wide
 * verification dwell must go through the full SetMonitorChannel gate to
 * restore 20 MHz. Lose that and every later "bin" silently observes at the
 * candidate's width — a wrong reading that still looks entirely plausible. */
#ifndef DEVOURER_SENSING_DWELL_EXECUTOR_H
#define DEVOURER_SENSING_DWELL_EXECUTOR_H

#include <atomic>
#include <cstdint>
#include <exception>

#include "IRadio.h"
#include "IRtlRadio.h"
#include "RxPacket.h"
#include "chanmig/ScanPlan.h"
#include "chanmig/SurveyRecord.h"
#include "sensing/SenseWindow.h"
#include "sensing/SurveyFrameAgg.h"

namespace devourer {
namespace sensing {

struct DwellExecConfig {
  int settle_ms = 30; /* stamped into the record; the CALLER does the sleeping */
  bool with_nhm = true;
  uint32_t counter_ceiling_per_ms = 1000;
  /* Stamped into every record — the demo maps its env onto these. */
  uint32_t plan_hash = 0;
  uint32_t scout_id = 0;
  uint8_t adapter_gen = 0;
  MonotonicUs clock = nullptr;
};

class DwellExecutor {
public:
  /* `rtl` may be null (non-Realtek radio): the dwell still retunes, still
   * folds frames, and reports its energy fields invalid rather than zero. */
  DwellExecutor(IRadio *radio, IRtlRadio *rtl, SurveyFrameAggregator *agg,
                const DwellExecConfig &cfg)
      : radio_(radio), agg_(agg), cfg_(cfg), sense_(radio, rtl, cfg.clock),
        clock_(cfg.clock ? cfg.clock : MonotonicUs(&steady_us)) {}

  /* Phase 1 — open the record and retune.
   *
   * Returns false when the retune threw: `out` then carries kFlagRetuneFailed
   * with observe_ms == 0 and t_end_ms set, and is ready to emit as-is. The
   * width latch is NOT advanced on a failed retune — a throw part-way through
   * a wide tune leaves the chip's width unknown, so the next dwell must take
   * the full gate. */
  bool begin(const chanmig::ScanScheduler::DwellPlan &plan, int64_t now_ms,
             chanmig::SurveyDwell &out) {
    out = chanmig::SurveyDwell{};
    out.seq = seq_++;
    out.def = plan.def;
    out.round = plan.round;
    out.plan_hash = cfg_.plan_hash;
    out.t_start_ms = now_ms;
    out.settle_ms = cfg_.settle_ms;
    out.scout_id = cfg_.scout_id;
    out.adapter_gen = cfg_.adapter_gen;
    if (plan.full_width)
      out.flags |= chanmig::kFlagFullWidth;

    const int64_t rt0 = clock_();
    const bool full_gate = plan.full_width || last_was_wide_;
    bool tuned = false;
    try {
      if (full_gate) {
        radio_->SetMonitorChannel(plan.def.to_selected());
        last_was_wide_ = plan.full_width;
      } else {
        radio_->FastRetune(plan.bin_ch, /*cache_rf=*/true);
      }
      tuned = true;
    } catch (const std::exception &) {
      /* A throw part-way through a FULL-GATE tune leaves the chip's width
       * unknown — it may have been reconfigured before the failure. Latch so
       * the next dwell takes the full gate rather than the lean same-width
       * path, which would otherwise observe every later bin at the
       * candidate's width. A FastRetune throw is different: that path never
       * changes width, so the latch is left alone. */
      if (full_gate)
        last_was_wide_ = true;
    }
    out.retune_us = clock_() - rt0;

    if (!tuned) {
      out.flags |= chanmig::kFlagRetuneFailed;
      out.t_end_ms = now_ms;
      out.observe_ms = 0;
      return false;
    }
    return true;
  }

  /* Phase 2 — the discard barrier. Resets the chip's delta counters AND zeroes
   * the frame aggregator, so frames still draining from the previous channel
   * never land in this record. Call it after the settle sleep. */
  void barrier(int64_t now_ms) {
    SenseOptions opt;
    opt.with_nhm = cfg_.with_nhm;
    opt.counter_ceiling_per_ms = cfg_.counter_ceiling_per_ms;
    sense_.barrier(opt);
    if (agg_)
      agg_->reset();
    /* observe_ms spans from here, so the settle is excluded from the window
     * the counters are judged against. The ms value is the caller's epoch; the
     * us mark is what actually measures the window, so the duration includes
     * the observation read's own bus round-trips (the hardware kept counting
     * during them, and excluding that time would inflate every derived rate). */
    obs_start_ms_ = now_ms;
    obs_start_us_ = clock_();
    obs_open_ = true;
  }

  /* Phase 3 — read energy, drain frames, reduce, latch.
   *
   * observe_ms stays on the caller's MILLISECOND clock, exactly as the demo
   * computed it. The measured microsecond window is available to the counter
   * check, but the record field keeps its old meaning on purpose: the
   * plausibility ceiling is derived from it, so silently switching clocks
   * would move a flag boundary during what is meant to be a faithful lift. */
  void finish(bool truncated, chanmig::SurveyDwell &out) {
    SenseOptions opt;
    opt.with_nhm = cfg_.with_nhm;
    opt.counter_ceiling_per_ms = cfg_.counter_ceiling_per_ms;
    const SenseResult r = sense_.read(opt);

    /* Timed here rather than from a caller-supplied `now`: the argument would
     * have to be evaluated BEFORE this call and would silently exclude the
     * read above, shortening the very window the plausibility ceiling is
     * computed from. */
    out.observe_ms =
        obs_open_ ? (clock_() - obs_start_us_ + 500) / 1000 : 0;
    out.t_end_ms = obs_start_ms_ + out.observe_ms;
    if (truncated)
      out.flags |= chanmig::kFlagTruncated;

    const RxEnergy &e = r.energy;
    out.valid_fa = e.valid_fa;
    out.fa_ofdm = e.fa_ofdm;
    out.fa_cck = e.fa_cck;
    out.cca_ofdm = e.cca_ofdm;
    out.cca_cck = e.cca_cck;
    out.valid_igi = e.valid_igi;
    out.igi = e.igi;

    out.valid_nhm = r.nhm.valid;
    if (r.nhm.valid) {
      for (int k = 0; k < 12; k++)
        out.nhm[k] = e.nhm[k];
      out.nhm_dur = r.nhm.duration;
      out.nhm_peak = r.nhm.peak_bucket;
      out.nhm_busy_pct = r.nhm.busy_pct;
      out.nhm_env_pct = r.nhm.env_pct;
    } else {
      out.flags |= chanmig::kFlagNhmMissing;
    }
    /* Busy airtime from whichever facility this backend has — Realtek CCX CLM
     * derived from the read above, or the neutral IRadio::GetChannelBusy on
     * any other family. busy_source records which, because the two count busy
     * differently and a consumer comparing across adapters needs to know. */
    out.valid_clm = r.busy.valid_busy;
    out.clm_ratio_pct = r.busy.busy_pct;
    out.busy_source = static_cast<uint8_t>(r.busy.source);

    /* Plausibility against the record's own observation window, matching what
     * the demo did — not against the finer measured one. */
    if (counters_implausible(e, out.observe_ms * 1000,
                             cfg_.counter_ceiling_per_ms))
      out.flags |= chanmig::kFlagCounterSuspect;
    if (r.read_failed)
      out.flags |= chanmig::kFlagReadFailed;

    if (agg_) {
      const SurveyFrameWindow w = agg_->drain();
      out.frames = w.frames;
      out.rssi_mean_raw = w.rssi_mean_raw;
      out.rssi_max_raw = w.rssi_max_raw;
      out.snr_mean_raw = w.snr_mean_raw;
      out.snr_min_raw = w.snr_min_raw;
      out.evm_mean_raw = w.evm_mean_raw;
      out.evm_valid = w.evm_valid;
      out.dvr_frames = w.own_frames;
      out.dvr_air_us = w.own_air_us;
      out.oth_air_us = w.other_air_us;
    }
    obs_open_ = false;
  }

  bool energy_ever_valid() const { return sense_.energy_ever_valid(); }
  int energy_invalid_streak() const { return sense_.invalid_streak(); }
  /* True when the NEXT begin() will take the full SetMonitorChannel gate
   * rather than the lean FastRetune: either the last dwell was full-width, or
   * a full-gate tune threw and left the chip's width unknown. */
  bool needs_full_gate() const { return last_was_wide_; }

private:
  IRadio *radio_;
  SurveyFrameAggregator *agg_;
  DwellExecConfig cfg_;
  SenseWindow sense_;
  MonotonicUs clock_;
  uint64_t seq_ = 0;
  bool last_was_wide_ = false;
  int64_t obs_start_ms_ = 0;
  int64_t obs_start_us_ = 0;
  bool obs_open_ = false;
};

/* Bind an RX sink for `agg`, ready to hand to IRadio::Init or StartRxLoop.
 * Drops C2H report packets — they are chip status, not air traffic. The
 * optional counter is bumped per accepted frame (the demos' liveness tick).
 *
 * LIFETIME: the returned function holds `agg` by reference and runs on the RX
 * thread, so `agg` must outlive the RX loop. Construct it before Init and
 * destroy it after the thread running StartRxLoop is joined — the library
 * spawns no thread, per the IRadio threading contract. */
inline Action_ParsedRadioPacket
rx_sink(SurveyFrameAggregator &agg,
        std::atomic<int> *frame_counter = nullptr) {
  return [&agg, frame_counter](const Packet &packet) {
    if (packet.RxAtrib.pkt_rpt_type == RX_PACKET_TYPE::C2H_PACKET)
      return;
    if (frame_counter)
      frame_counter->fetch_add(1, std::memory_order_relaxed);
    const auto &a = packet.RxAtrib;
    agg.add(packet.Data.data(), packet.Data.size(), a.data_rate, a.bw,
            a.sgi != 0, a.fcs_present, a.rssi[0], a.snr[0], a.evm[0]);
  };
}

} /* namespace sensing */
} /* namespace devourer */

#endif /* DEVOURER_SENSING_DWELL_EXECUTOR_H */
