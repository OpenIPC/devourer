/* SurveyDwell — the versioned record one scout dwell produces, and the
 * airtime approximation that splits decoded occupancy into "our own video"
 * vs "other traffic".
 *
 * A dwell is one visit to one 20 MHz bin (or, for the optional wide
 * verification dwell, one full-width candidate): retune → settle → discard
 * barrier (reset the chip's delta counters + drain frames still in the USB
 * pipeline from the previous channel) → observe → read. The FA/CCA fields
 * are therefore deltas over exactly the observe window — the counter hygiene
 * the plain rxdemo sweep does not do (its per-bin delta includes its own
 * retune+settle).
 *
 * Every field that can be invalid carries a flag bit instead of a fake zero:
 * a consumer must be able to tell "quiet channel" from "sensor absent" from
 * "dwell aborted". */
#ifndef DEVOURER_CHANMIG_SURVEY_RECORD_H
#define DEVOURER_CHANMIG_SURVEY_RECORD_H

#include <cstdint>

#include "chanmig/ChannelDef.h"

namespace devourer {
namespace chanmig {

inline constexpr int kSurveySchemaV = 1;

enum SurveyFlag : uint16_t {
  kFlagTruncated = 1u << 0,      /* dwell cut short (shutdown) */
  kFlagRetuneFailed = 1u << 1,   /* retune threw — no observation happened */
  kFlagReadFailed = 1u << 2,     /* energy read invalid at dwell end */
  kFlagCounterSuspect = 1u << 3, /* delta implausible for the window (wrap) */
  kFlagNhmMissing = 1u << 4,     /* generation exposes no NHM this dwell */
  kFlagFullWidth = 1u << 5,      /* wide verification dwell (full retune) */
};

struct SurveyDwell {
  uint64_t seq = 0;      /* monotonic dwell counter — a gap = a lost record */
  ChannelDef def;        /* what was tuned: a 20 MHz bin def, or the wide
                            candidate when kFlagFullWidth */
  uint64_t round = 0;    /* completed-full-sweep count at dwell time */
  uint32_t plan_hash = 0;

  int64_t t_start_ms = 0; /* caller monotonic clock, dwell start (pre-retune) */
  int64_t t_end_ms = 0;
  int64_t retune_us = 0;
  int settle_ms = 0;
  int64_t observe_ms = 0; /* the valid observation window the deltas span */

  /* Frame-free energy (deltas over observe; validity per source). */
  bool valid_fa = false;
  uint32_t fa_ofdm = 0, fa_cck = 0, cca_ofdm = 0, cca_cck = 0;
  bool valid_igi = false;
  uint8_t igi = 0;
  bool valid_nhm = false;
  uint8_t nhm[12] = {};
  uint16_t nhm_dur = 0;
  uint8_t nhm_busy_pct = 0; /* % of NHM samples above the lowest bucket */
  uint8_t nhm_peak = 0;     /* fullest bucket index */

  /* Frame-driven aggregate over the observe window (raw devourer units). */
  uint32_t frames = 0;
  int rssi_mean_raw = 0, rssi_max_raw = 0;
  int snr_mean_raw = 0, snr_min_raw = 0;
  int evm_mean_raw = 0;
  bool evm_valid = false;

  /* Occupancy attribution: canonical-SA (our own video) vs everything else,
   * as decoded-airtime estimates. Undecodable energy shows only in FA/CCA/
   * NHM — that asymmetry is why the scoring layer treats these as a SPLIT of
   * explained occupancy, never as total occupancy. */
  uint32_t dvr_frames = 0;
  uint64_t dvr_air_us = 0;
  uint64_t oth_air_us = 0;

  uint16_t flags = 0;
  uint32_t scout_id = 0;  /* stable adapter identity — calibration domain */
  uint8_t adapter_gen = 0;
};

/* Approximate on-air duration of one decoded frame from its DESC_RATE code:
 * preamble + payload bits at the code's data rate. Deliberately coarse (long
 * preamble assumed for CCK, one PPDU per MPDU, no aggregation accounting) —
 * it only SPLITS explained occupancy between transmitters, it is never an
 * absolute energy unit. `bw` is the rx-descriptor code (0=20,1=40,2=80). */
inline uint32_t frame_airtime_us(uint16_t desc_rate, uint32_t len_bytes,
                                 uint8_t bw, bool sgi) {
  static const uint32_t kCck[4] = {1000, 2000, 5500, 11000};
  static const uint32_t kOfdm[8] = {6000,  9000,  12000, 18000,
                                    24000, 36000, 48000, 54000};
  static const uint32_t kMcs20[10] = {6500,  13000, 19500, 26000, 39000,
                                      52000, 58500, 65000, 78000, 86700};
  uint32_t kbps = 6000, preamble_us = 20;
  if (desc_rate <= 3) {
    kbps = kCck[desc_rate];
    preamble_us = 192;
  } else if (desc_rate <= 11) {
    kbps = kOfdm[desc_rate - 4];
  } else if (desc_rate <= 43) { /* HT MCS0..31 */
    const uint32_t mcs = desc_rate - 12;
    kbps = kMcs20[mcs % 8] * (mcs / 8 + 1);
    if (bw >= 1)
      kbps = kbps * 27 / 13; /* 40 MHz: 108/52 data subcarriers */
    preamble_us = 36;
  } else if (desc_rate <= 83) { /* VHT 1SS..4SS MCS0..9 */
    const uint32_t idx = desc_rate - 44;
    kbps = kMcs20[idx % 10] * (idx / 10 + 1);
    if (bw == 1)
      kbps = kbps * 27 / 13;
    else if (bw >= 2)
      kbps = kbps * 9 / 2; /* 80 MHz: 234/52 */
    preamble_us = 40;
  }
  if (sgi)
    kbps = kbps * 10 / 9;
  if (kbps == 0)
    kbps = 6000;
  const uint64_t bits = static_cast<uint64_t>(len_bytes) * 8u;
  uint64_t us = preamble_us + (bits * 1000u + kbps - 1) / kbps;
  if (us > 20000)
    us = 20000; /* one frame never plausibly exceeds ~20 ms of air */
  return static_cast<uint32_t>(us);
}

} /* namespace chanmig */
} /* namespace devourer */

#endif /* DEVOURER_CHANMIG_SURVEY_RECORD_H */
