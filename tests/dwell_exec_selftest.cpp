/* Headless guard for the channel-survey executor (src/sensing/).
 *
 * This loop was demo code until now, which meant a regression in it was
 * findable only on air: the scheduler has a selftest and so does the consumer,
 * but the retune/barrier/observe sequencing between them had none. These cases
 * pin the parts whose failure produces records that still parse and still look
 * entirely plausible.
 *
 * Everything is driven by a scripted fake radio and a scripted clock, so no
 * duration is timing-dependent and the whole file is deterministic under
 * ctest and under the sanitizers. */
#include "chanmig/ScanPlan.h"
#include "sensing/DwellExecutor.h"

#include <cstdio>
#include <deque>
#include <string>
#include <vector>

using devourer::chanmig::ChannelDef;
using devourer::chanmig::ScanPlanConfig;
using devourer::chanmig::ScanScheduler;
using devourer::chanmig::SurveyDwell;
using devourer::sensing::DwellExecConfig;
using devourer::sensing::DwellExecutor;
using devourer::sensing::SurveyAggConfig;
using devourer::sensing::SurveyFrameAggregator;

namespace {

int g_fail = 0;
void check(const char *what, long got, long want) {
  if (got != want) {
    std::printf("FAIL %s: got %ld want %ld\n", what, got, want);
    ++g_fail;
  }
}
void check_str(const char *what, const std::string &got,
               const std::string &want) {
  if (got != want) {
    std::printf("FAIL %s:\n  got  %s\n  want %s\n", what, got.c_str(),
                want.c_str());
    ++g_fail;
  }
}

/* A Realtek radio whose every interesting behaviour is scripted: what it
 * logs, what energy it returns, whether it throws, and what happens at the
 * exact instant of the barrier. */
struct FakeRtl final : IRtlRadio {
  std::vector<std::string> log;
  std::deque<RxEnergy> energies;
  bool throw_on_set = false, throw_on_fast = false;
  std::function<void()> on_barrier; /* fires on GetRxEnergy(false) */
  SelectedChannel ch{};

  void Init(Action_ParsedRadioPacket, SelectedChannel) override {}
  void InitWrite(SelectedChannel) override {}
  void StartRxLoop(Action_ParsedRadioPacket) override {}
  bool send_packet(const uint8_t *, size_t) override { return false; }
  SelectedChannel GetSelectedChannel() override { return ch; }
  void SetCcaMode(bool) override {}

  void SetMonitorChannel(SelectedChannel c) override {
    if (throw_on_set)
      throw std::runtime_error("set failed");
    ch = c;
    log.push_back("set:" + std::to_string(c.Channel) + "/" +
                  std::to_string(static_cast<int>(c.ChannelWidth)));
  }
  void FastRetune(uint8_t c, bool) override {
    if (throw_on_fast)
      throw std::runtime_error("fast failed");
    ch.Channel = c;
    log.push_back("fast:" + std::to_string(c));
  }
  RxEnergy GetRxEnergy(bool with_nhm) override {
    log.push_back(with_nhm ? "energy:1" : "energy:0");
    if (!with_nhm && on_barrier)
      on_barrier();
    if (energies.empty())
      return {};
    RxEnergy e = energies.front();
    energies.pop_front();
    return e;
  }
};

/* A plain neutral radio: no IRtlRadio, therefore no phydm counters. */
struct FakeNeutral final : IRadio {
  std::vector<std::string> log;
  SelectedChannel ch{};
  /* Read-and-clear, like the real MAC channel timers: the barrier read takes
   * whatever accumulated during retune+settle, and only what arrives after it
   * reaches the record. */
  uint8_t pending_busy = 0;
  bool has_busy = false;
  devourer::ChannelBusy GetChannelBusy() override {
    log.push_back("busy");
    devourer::ChannelBusy b;
    if (has_busy) {
      b.valid = true;
      b.valid_busy = true;
      b.source = devourer::BusySource::ChTime;
      b.busy_pct = pending_busy;
      b.window_us = 100000;
    }
    pending_busy = 0;
    has_busy = false;
    return b;
  }
  void Init(Action_ParsedRadioPacket, SelectedChannel) override {}
  void InitWrite(SelectedChannel) override {}
  void StartRxLoop(Action_ParsedRadioPacket) override {}
  bool send_packet(const uint8_t *, size_t) override { return false; }
  SelectedChannel GetSelectedChannel() override { return ch; }
  void SetCcaMode(bool) override {}
  void SetMonitorChannel(SelectedChannel c) override {
    ch = c;
    log.push_back("set:" + std::to_string(c.Channel));
  }
  void FastRetune(uint8_t c, bool) override {
    ch.Channel = c;
    log.push_back("fast:" + std::to_string(c));
  }
};

/* The scripted clock. Nothing advances it but the test, so every duration in
 * a record is an exact integer and the file is deterministic under ctest and
 * under the sanitizers. */
int64_t g_now_us = 0;
devourer::sensing::MonotonicUs scripted_clock() {
  return [] { return g_now_us; };
}

std::string join(const std::vector<std::string> &v) {
  std::string s;
  for (size_t i = 0; i < v.size(); i++) {
    if (i)
      s += ",";
    s += v[i];
  }
  return s;
}

ChannelDef bin20(uint8_t ch) {
  ChannelDef d{};
  d.band = 5;
  d.primary = ch;
  d.width = CHANNEL_WIDTH_20;
  return d;
}

RxEnergy fa(uint32_t cca, uint32_t fa_ofdm) {
  RxEnergy e;
  e.valid_fa = true;
  e.cca_ofdm = cca;
  e.fa_ofdm = fa_ofdm;
  return e;
}

/* Run one dwell end to end with a fixed 100 ms observation window. */
SurveyDwell run_dwell(DwellExecutor &exec, const ScanScheduler::DwellPlan &plan,
                      int64_t t0_ms, bool truncated = false) {
  SurveyDwell d;
  if (!exec.begin(plan, t0_ms, d))
    return d;
  g_now_us += 30000; /* settle */
  exec.barrier(t0_ms + 30);
  g_now_us += 100000; /* the observation window */
  exec.finish(truncated, d);
  return d;
}

} // namespace

int main() {
  /* --- barrier ordering: the whole point of the discipline --- */
  {
    FakeRtl r;
    SurveyFrameAggregator agg;
    DwellExecConfig cfg;
    cfg.clock = scripted_clock();
    DwellExecutor exec(&r, &r, &agg, cfg);
    auto sink = devourer::sensing::rx_sink(agg);

    /* A frame folded BEFORE the barrier belongs to the previous channel. */
    agg.add(nullptr, 100, 4 /*6M*/, 0, false, true, 40, 20, -50);
    /* ...and one folded DURING the barrier read must also be discarded, since
     * the reset happens after the chip read inside the same phase. */
    r.on_barrier = [&] {
      agg.add(nullptr, 100, 4, 0, false, true, 40, 20, -50);
    };
    ScanScheduler::DwellPlan plan{};
    plan.valid = true;
    plan.bin_ch = 36;
    plan.def = bin20(36);
    SurveyDwell d;
    exec.begin(plan, 1000, d);
    g_now_us += 30000;
    exec.barrier(1030);
    /* Frames arriving inside the window DO count. */
    agg.add(nullptr, 100, 4, 0, false, true, 40, 20, -50);
    g_now_us += 100000;
    exec.finish(false, d);

    check_str("barrier call order", join(r.log), "fast:36,energy:0,energy:1");
    check("only in-window frames counted", d.frames, 1);
  }

  /* --- width restoration: FastRetune is same-width --- */
  {
    FakeRtl r;
    DwellExecConfig cfg;
    cfg.clock = scripted_clock();
    DwellExecutor exec(&r, &r, nullptr, cfg);

    auto step = [&](uint8_t ch, bool wide) {
      ScanScheduler::DwellPlan p{};
      p.valid = true;
      p.bin_ch = ch;
      p.def = bin20(ch);
      if (wide) {
        p.full_width = true;
        p.def.width = CHANNEL_WIDTH_80;
      }
      run_dwell(exec, p, 0);
    };
    step(36, false);
    step(40, false);
    step(36, true); /* wide verification dwell */
    step(36, false);
    step(40, false);

    std::vector<std::string> tunes;
    for (const auto &l : r.log)
      if (l.rfind("energy", 0) != 0)
        tunes.push_back(l);
    /* The dwell after the wide one MUST take the full gate; the one after
     * that may go lean again. */
    check_str("width restoration", join(tunes),
              "fast:36,fast:40,set:36/2,set:36/0,fast:40");
  }

  /* --- counter plausibility: exact ceiling, > not >= --- */
  {
    for (int i = 0; i < 2; i++) {
      FakeRtl r;
      DwellExecConfig cfg;
    cfg.clock = scripted_clock();
      /* 100 ms window * 1000/ms = 100000 events is the ceiling. */
      r.energies.push_back({});                             /* barrier read */
      r.energies.push_back(fa(i == 0 ? 100000 : 100001, 0)); /* observation */
      DwellExecutor exec(&r, &r, nullptr, cfg);
      ScanScheduler::DwellPlan p{};
      p.valid = true;
      p.bin_ch = 36;
      p.def = bin20(36);
      const SurveyDwell d = run_dwell(exec, p, 0);
      check(i == 0 ? "at ceiling is clean" : "above ceiling is suspect",
            (d.flags & devourer::chanmig::kFlagCounterSuspect) != 0, i);
    }
    /* An absent counter is not a suspect counter, however absurd the value. */
    FakeRtl r;
    RxEnergy e;
    e.valid_fa = false;
    e.cca_ofdm = 9999999;
    r.energies.push_back({});
    r.energies.push_back(e);
    DwellExecConfig cfg;
    cfg.clock = scripted_clock();
    DwellExecutor exec(&r, &r, nullptr, cfg);
    ScanScheduler::DwellPlan p{};
    p.valid = true;
    p.bin_ch = 36;
    p.def = bin20(36);
    const SurveyDwell d = run_dwell(exec, p, 0);
    check("absent counter is not suspect",
          (d.flags & devourer::chanmig::kFlagCounterSuspect) != 0, 0);
  }

  /* --- read-failure latch: unported is not failed --- */
  {
    FakeRtl r;
    DwellExecConfig cfg;
    cfg.clock = scripted_clock();
    DwellExecutor exec(&r, &r, nullptr, cfg);
    ScanScheduler::DwellPlan p{};
    p.valid = true;
    p.bin_ch = 36;
    p.def = bin20(36);

    /* Never valid: a generation with no counters. NOT a failure. */
    const SurveyDwell d0 = run_dwell(exec, p, 0);
    check("never-valid is not a read failure",
          (d0.flags & devourer::chanmig::kFlagReadFailed) != 0, 0);

    /* Valid once, then gone: that IS a failure. */
    r.energies.push_back({});
    r.energies.push_back(fa(10, 1));
    const SurveyDwell d1 = run_dwell(exec, p, 0);
    check("valid read is clean",
          (d1.flags & devourer::chanmig::kFlagReadFailed) != 0, 0);
    const SurveyDwell d2 = run_dwell(exec, p, 0);
    check("once-valid then invalid is a read failure",
          (d2.flags & devourer::chanmig::kFlagReadFailed) != 0, 1);
    check("invalid streak counts", exec.energy_invalid_streak(), 1);
  }

  /* --- retune failure: no energy calls, and the width latch is preserved --- */
  {
    FakeRtl r;
    r.throw_on_fast = true;
    DwellExecConfig cfg;
    cfg.clock = scripted_clock();
    DwellExecutor exec(&r, &r, nullptr, cfg);
    ScanScheduler::DwellPlan p{};
    p.valid = true;
    p.bin_ch = 36;
    p.def = bin20(36);
    SurveyDwell d;
    const bool ok = exec.begin(p, 500, d);
    check("begin reports failure", ok, 0);
    check("retune failure flagged",
          (d.flags & devourer::chanmig::kFlagRetuneFailed) != 0, 1);
    check("no observation on a failed retune", d.observe_ms, 0);
    check("t_end is set", d.t_end_ms, 500);
    check("no energy read after a failed retune", (long)r.log.size(), 0);
  }
  {
    /* A throw part-way through a WIDE tune leaves the width unknown, so the
     * next bin dwell must still take the full gate — the edge the demo's
     * `continue` hides. */
    FakeRtl r;
    r.throw_on_set = true;
    DwellExecConfig cfg;
    cfg.clock = scripted_clock();
    DwellExecutor exec(&r, &r, nullptr, cfg);
    ScanScheduler::DwellPlan wide{};
    wide.valid = true;
    wide.bin_ch = 36;
    wide.def = bin20(36);
    wide.def.width = CHANNEL_WIDTH_80;
    wide.full_width = true;
    SurveyDwell d;
    exec.begin(wide, 0, d);
    /* The chip may have been part-way reconfigured before the throw, so the
     * width is unknown and the next dwell must take the full gate. */
    check("wide throw forces the full gate next", exec.needs_full_gate(), 1);
    r.throw_on_set = false;
    ScanScheduler::DwellPlan bin{};
    bin.valid = true;
    bin.bin_ch = 40;
    bin.def = bin20(40);
    exec.begin(bin, 0, d);
    /* Not "fast:40" — the failed wide tune must not licence the lean path. */
    check_str("next dwell after a failed wide tune", join(r.log), "set:40/0");
  }

  /* --- frame attribution is configuration, not a constant --- */
  {
    const uint8_t ours[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                              0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
    SurveyAggConfig c;
    c.own_sa = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
    c.own_sa_valid = true;
    SurveyFrameAggregator a(c);
    a.add(ours, 16, 4, 0, false, true, 40, 20, 0);
    auto w = a.drain();
    check("own frame attributed to us", w.own_frames, 1);
    check("own frame not counted foreign", (long)w.other_air_us, 0);

    /* Flip the key and the same frame becomes foreign. */
    SurveyAggConfig c2;
    c2.own_sa = {0xde, 0xad, 0xbe, 0xef, 0x00, 0x01};
    c2.own_sa_valid = true;
    SurveyFrameAggregator a2(c2);
    a2.add(ours, 16, 4, 0, false, true, 40, 20, 0);
    auto w2 = a2.drain();
    check("attribution follows the configured SA", w2.own_frames, 0);
    check("foreign airtime counted", w2.other_air_us > 0, 1);

    /* No key configured: everything is foreign. */
    SurveyFrameAggregator a3;
    a3.add(ours, 16, 4, 0, false, true, 40, 20, 0);
    check("no key means no ownership", a3.drain().own_frames, 0);

    /* FCS absent: 4 bytes added back, so airtime is strictly larger. */
    SurveyFrameAggregator withf, without;
    withf.add(ours, 16, 4, 0, false, /*fcs_present=*/true, 40, 20, 0);
    without.add(ours, 16, 4, 0, false, /*fcs_present=*/false, 40, 20, 0);
    check("missing FCS adds airtime",
          without.drain().other_air_us > withf.drain().other_air_us, 1);

    /* rssi <= 0 is not a quality sample, but its airtime still counts. */
    SurveyFrameAggregator q;
    q.add(ours, 16, 4, 0, false, true, 0, 0, 0);
    auto wq = q.drain();
    check("unmeasurable frame is not a quality sample", wq.frames, 0);
    check("unmeasurable frame still occupied the channel",
          wq.other_air_us > 0, 1);
  }

  /* --- THE case this change exists for: a non-Realtek radio --- */
  {
    FakeNeutral n;
    SurveyFrameAggregator agg;
    DwellExecConfig cfg;
    cfg.clock = scripted_clock();
    DwellExecutor exec(&n, /*rtl=*/nullptr, &agg, cfg);
    ScanScheduler::DwellPlan p{};
    p.valid = true;
    p.bin_ch = 36;
    p.def = bin20(36);
    agg.add(nullptr, 100, 4, 0, false, true, 40, 20, -50);
    SurveyDwell d;
    exec.begin(p, 0, d);
    g_now_us += 30000;
    exec.barrier(30);
    agg.add(nullptr, 100, 4, 0, false, true, 40, 20, -50);
    g_now_us += 100000;
    exec.finish(false, d);

    check("neutral radio retunes and reads busy twice",
          join(n.log) == "fast:36,busy,busy", 1);
    check("neutral radio reports no energy", d.valid_fa, 0);
    check("neutral radio reports no NHM",
          (d.flags & devourer::chanmig::kFlagNhmMissing) != 0, 1);
    check("absent counters are not a read failure",
          (d.flags & devourer::chanmig::kFlagReadFailed) != 0, 0);
    check("neutral radio still yields frame evidence", d.frames, 1);
    check("neutral radio still yields airtime", d.oth_air_us > 0, 1);
    /* No reading offered -> no reading recorded. Not 0% busy. */
    check("neutral radio with no busy counter records none", d.valid_clm, 0);
  }
  {
    /* THE claim this PR rests on: a radio with only GetChannelBusy — no
     * IRtlRadio anywhere — produces a dwell carrying real busy airtime, with
     * its provenance, so chanmig can rank on evidence from it. */
    FakeNeutral n;
    n.has_busy = true;
    n.pending_busy = 41;
    SurveyFrameAggregator agg;
    DwellExecConfig cfg;
    cfg.clock = scripted_clock();
    DwellExecutor exec(&n, /*rtl=*/nullptr, &agg, cfg);
    ScanScheduler::DwellPlan p{};
    p.valid = true;
    p.bin_ch = 36;
    p.def = bin20(36);
    SurveyDwell d;
    exec.begin(p, 0, d);
    g_now_us += 30000;
    exec.barrier(30); /* consumes the pre-window count */
    n.has_busy = true;
    n.pending_busy = 41; /* what the window itself saw */
    g_now_us += 100000;
    exec.finish(false, d);

    check("non-Realtek radio yields busy airtime", d.valid_clm, 1);
    check("busy airtime passes through", d.clm_ratio_pct, 41);
    check("provenance recorded", d.busy_source,
          static_cast<long>(devourer::BusySource::ChTime));
  }

  /* --- round bookkeeping: composes with the real scheduler --- */
  {
    FakeRtl r;
    ScanPlanConfig pc;
    pc.candidates = {bin20(36), bin20(40), bin20(44)};
    ScanScheduler sched(pc);
    DwellExecConfig cfg;
    cfg.clock = scripted_clock();
    cfg.plan_hash = 0xabcd1234;
    cfg.scout_id = 0x11223344;
    cfg.adapter_gen = 3;
    DwellExecutor exec(&r, &r, nullptr, cfg);

    uint64_t expect_seq = 0;
    int64_t now = 0;
    for (int i = 0; i < 6; i++) {
      auto plan = sched.next(now);
      if (!plan.valid)
        break;
      SurveyDwell d;
      exec.begin(plan, now, d);
      g_now_us += 30000;
      exec.barrier(now + 30);
      g_now_us += 100000;
      exec.finish(false, d);
      sched.complete(plan, d.t_end_ms, true);
      check("seq is gapless", (long)d.seq, (long)expect_seq++);
      check("plan_hash stamped", (long)d.plan_hash, 0xabcd1234);
      check("scout_id stamped", (long)d.scout_id, 0x11223344);
      check("adapter_gen stamped", d.adapter_gen, 3);
      check("settle_ms stamped", d.settle_ms, cfg.settle_ms);
      check("observe_ms spans the dwell", (long)d.observe_ms, 100);
      check("t_start <= t_end", d.t_start_ms <= d.t_end_ms, 1);
      now += 200;
    }
    check("a full sweep advanced the round", sched.rounds_complete() >= 1, 1);
  }

  if (g_fail) {
    std::printf("dwell_executor: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("dwell_executor: all checks passed\n");
  return 0;
}
