/* survey.dwell JSONL binding — the one place the record schema exists.
 *
 * chanscout emits through emit_survey_dwell(); every C++ consumer (the
 * advise-mode replay path, the aggregator selftest) parses through
 * survey_dwell_from_jsonl(). Keeping both directions in one header means the
 * schema cannot drift between producer and consumer, and the selftest
 * round-trips a record through both to pin it. Python consumers read the
 * same fields via tests/devourer_events.py.
 *
 * Nullable-field convention follows rx.energy: a chip counter the generation
 * doesn't expose is JSON null, never a fake zero — so "quiet channel" and
 * "sensor absent" stay distinguishable offline. */
#ifndef DEVOURER_CHANMIG_SURVEY_JSONL_H
#define DEVOURER_CHANMIG_SURVEY_JSONL_H

#include <string>

#include "Event.h"
#include "chanmig/JsonlLite.h"
#include "chanmig/SurveyRecord.h"

namespace devourer {
namespace chanmig {

inline void emit_survey_dwell(EventSink &sink, const SurveyDwell &d) {
  Ev ev(sink, "survey.dwell");
  ev.f("v", kSurveySchemaV).f("seq", (unsigned long long)d.seq);
  char chan[20];
  d.def.format(chan, sizeof(chan));
  ev.f("chan", chan).f("round", (unsigned long long)d.round);
  ev.hexf("plan", d.plan_hash, 8);
  ev.f("start_ms", (long long)d.t_start_ms)
      .f("end_ms", (long long)d.t_end_ms)
      .f("retune_us", (long long)d.retune_us)
      .f("settle_ms", d.settle_ms)
      .f("observe_ms", (long long)d.observe_ms);
  if (d.valid_fa)
    ev.f("cca_ofdm", d.cca_ofdm)
        .f("cca_cck", d.cca_cck)
        .f("fa_ofdm", d.fa_ofdm)
        .f("fa_cck", d.fa_cck);
  else
    ev.f("cca_ofdm", nullptr)
        .f("cca_cck", nullptr)
        .f("fa_ofdm", nullptr)
        .f("fa_cck", nullptr);
  if (d.valid_igi)
    ev.f("igi", d.igi);
  else
    ev.f("igi", nullptr);
  if (d.valid_nhm) {
    int hist[12];
    for (int i = 0; i < 12; i++)
      hist[i] = d.nhm[i];
    ev.f("nhm_busy", d.nhm_busy_pct)
        .f("nhm_peak", d.nhm_peak)
        .f("nhm_dur", d.nhm_dur)
        .arr("nhm", hist, 12);
  } else {
    ev.f("nhm_busy", nullptr);
  }
  ev.f("frames", d.frames)
      .f("rssi_mean", d.rssi_mean_raw)
      .f("rssi_max", d.rssi_max_raw)
      .f("snr_mean", d.snr_mean_raw)
      .f("snr_min", d.snr_min_raw);
  if (d.evm_valid)
    ev.f("evm_mean", d.evm_mean_raw);
  else
    ev.f("evm_mean", nullptr);
  ev.f("dvr_frames", d.dvr_frames)
      .f("dvr_air_us", (unsigned long long)d.dvr_air_us)
      .f("oth_air_us", (unsigned long long)d.oth_air_us)
      .f("flags", d.flags);
  ev.hexf("scout_id", d.scout_id, 8);
  ev.f("agen", d.adapter_gen);
}

/* Parse one survey.dwell line. False for any other event line, a schema
 * version this build doesn't know, or an unparseable channel token. */
inline bool survey_dwell_from_jsonl(std::string_view line, SurveyDwell &d) {
  if (!jsonl_ev_is(line, "survey.dwell"))
    return false;
  long long v = 0;
  if (!jsonl_int(line, "v", &v) || v != kSurveySchemaV)
    return false;
  d = SurveyDwell{};
  std::string chan, err, hex;
  if (!jsonl_str(line, "chan", &chan) || !parse_chan_token(chan, d.def, err))
    return false;
  long long x = 0;
  if (jsonl_int(line, "seq", &x))
    d.seq = static_cast<uint64_t>(x);
  if (jsonl_int(line, "round", &x))
    d.round = static_cast<uint64_t>(x);
  if (jsonl_str(line, "plan", &hex))
    d.plan_hash = static_cast<uint32_t>(std::strtoul(hex.c_str(), nullptr, 16));
  if (jsonl_int(line, "start_ms", &x))
    d.t_start_ms = x;
  if (jsonl_int(line, "end_ms", &x))
    d.t_end_ms = x;
  if (jsonl_int(line, "retune_us", &x))
    d.retune_us = x;
  if (jsonl_int(line, "settle_ms", &x))
    d.settle_ms = static_cast<int>(x);
  if (jsonl_int(line, "observe_ms", &x))
    d.observe_ms = x;
  if (jsonl_int(line, "cca_ofdm", &x)) {
    d.valid_fa = true;
    d.cca_ofdm = static_cast<uint32_t>(x);
    if (jsonl_int(line, "cca_cck", &x))
      d.cca_cck = static_cast<uint32_t>(x);
    if (jsonl_int(line, "fa_ofdm", &x))
      d.fa_ofdm = static_cast<uint32_t>(x);
    if (jsonl_int(line, "fa_cck", &x))
      d.fa_cck = static_cast<uint32_t>(x);
  }
  if (jsonl_int(line, "igi", &x)) {
    d.valid_igi = true;
    d.igi = static_cast<uint8_t>(x);
  }
  if (jsonl_int(line, "nhm_busy", &x)) {
    d.valid_nhm = true;
    d.nhm_busy_pct = static_cast<uint8_t>(x);
    if (jsonl_int(line, "nhm_peak", &x))
      d.nhm_peak = static_cast<uint8_t>(x);
    if (jsonl_int(line, "nhm_dur", &x))
      d.nhm_dur = static_cast<uint16_t>(x);
    int hist[12] = {};
    int n = 0;
    if (jsonl_arr(line, "nhm", hist, 12, &n))
      for (int i = 0; i < n; i++)
        d.nhm[i] = static_cast<uint8_t>(hist[i]);
  }
  if (jsonl_int(line, "frames", &x))
    d.frames = static_cast<uint32_t>(x);
  if (jsonl_int(line, "rssi_mean", &x))
    d.rssi_mean_raw = static_cast<int>(x);
  if (jsonl_int(line, "rssi_max", &x))
    d.rssi_max_raw = static_cast<int>(x);
  if (jsonl_int(line, "snr_mean", &x))
    d.snr_mean_raw = static_cast<int>(x);
  if (jsonl_int(line, "snr_min", &x))
    d.snr_min_raw = static_cast<int>(x);
  if (jsonl_int(line, "evm_mean", &x)) {
    d.evm_valid = true;
    d.evm_mean_raw = static_cast<int>(x);
  }
  if (jsonl_int(line, "dvr_frames", &x))
    d.dvr_frames = static_cast<uint32_t>(x);
  if (jsonl_int(line, "dvr_air_us", &x))
    d.dvr_air_us = static_cast<uint64_t>(x);
  if (jsonl_int(line, "oth_air_us", &x))
    d.oth_air_us = static_cast<uint64_t>(x);
  if (jsonl_int(line, "flags", &x))
    d.flags = static_cast<uint16_t>(x);
  if (jsonl_str(line, "scout_id", &hex))
    d.scout_id = static_cast<uint32_t>(std::strtoul(hex.c_str(), nullptr, 16));
  if (jsonl_int(line, "agen", &x))
    d.adapter_gen = static_cast<uint8_t>(x);
  return true;
}

} /* namespace chanmig */
} /* namespace devourer */

#endif /* DEVOURER_CHANMIG_SURVEY_JSONL_H */
