/* NHM utility ratios — the pure reduction of a raw 12-bucket NHM histogram to
 * the two percentages the vendor's environment monitor actually reports.
 *
 * Separated from NhmReader.h on purpose: this is arithmetic over 12 bytes plus
 * the 11 thresholds that produced them, so it is testable without a radio.
 *
 * The distinction that matters:
 *
 *   nhm_ratio      total histogram mass above the lowest bucket. This is the
 *                  naive "how much of the window was above the IGI reference"
 *                  number, and on a quiet channel it reads BUSY — the ambient
 *                  floor already sits above th[0], so the ratio rails.
 *
 *   nhm_env_ratio  the same mass with the receiver's own noise floor removed:
 *                  the leading cluster of buckets whose power weight is at or
 *                  below NHM_IC_NOISE_TH (-80 dBm) is subtracted, because that
 *                  is the IC hearing itself, not the environment. What remains
 *                  is energy that is really out there.
 *
 * It is nhm_env_ratio, not nhm_ratio, that the vendor prints as the ACS
 * "interference" column, and it is the railing of the naive form that has kept
 * the NHM histogram out of devourer's channel scoring.
 *
 * Ported from phydm_nhm_cal_wgt / phydm_nhm_cal_nhm_env / phydm_nhm_get_utility
 * (reference/rtl88x2bu/hal/phydm/phydm_ccx.c). */
#ifndef DEVOURER_NHM_ENV_MATH_H
#define DEVOURER_NHM_ENV_MATH_H

#include <cstdint>

namespace devourer {

inline constexpr int kNhmRptNum = 12; /* histogram buckets */
inline constexpr int kNhmThNum = 11;  /* thresholds between them */
/* Bucket weights at or below this are the IC's own floor. The vendor's own
 * comment: 60/2 - 10 = 20 = -80 dBm, via NTH_TH_2_RSSI(th) = (th>>1) - 10. */
inline constexpr uint8_t kNhmIcNoiseTh = 60;
/* The hardware divides the window into at most this many samples, so a bucket
 * sum is a fraction of 255 rather than of the sample count. */
inline constexpr uint16_t kNhmRptMax = 255;

/* phydm_ccx_get_rpt_ratio: rpt/denom as a rounded percentage. */
inline uint8_t ccx_rpt_ratio(uint16_t rpt, uint16_t denom) {
  if (denom == 0)
    return 0;
  const uint32_t numer = static_cast<uint32_t>(rpt) * 100u + (denom >> 1);
  const uint32_t r = numer / denom;
  return r > 255u ? 255u : static_cast<uint8_t>(r);
}

/* phydm_nhm_cal_wgt: the representative power of each bucket, in the same
 * U(8,1) PWdB unit as the thresholds. Bucket i sits between th[i-1] and th[i],
 * so its weight is their midpoint; the two open-ended end buckets borrow their
 * neighbouring threshold offset by 2. */
inline void nhm_bucket_weights(const uint8_t th[kNhmThNum],
                               uint8_t wgt[kNhmRptNum]) {
  wgt[0] = th[0] > 2 ? static_cast<uint8_t>(th[0] - 2) : 0;
  for (int i = 1; i < kNhmRptNum - 1; i++)
    wgt[i] = static_cast<uint8_t>((static_cast<int>(th[i - 1]) + th[i]) >> 1);
  wgt[kNhmRptNum - 1] =
      static_cast<uint8_t>(th[kNhmThNum - 1] > 253 ? 255 : th[kNhmThNum - 1] + 2);
}

struct NhmUtility {
  bool valid = false;      /* false when the histogram is not self-consistent */
  uint16_t sum = 0;        /* total bucket mass */
  uint8_t nhm_ratio = 0;   /* % above bucket 0 */
  uint8_t env_ratio = 0;   /* % above the IC's own noise floor */
};

/* phydm_nhm_get_utility + phydm_nhm_cal_nhm_env over one raw histogram.
 *
 * `th` must be the thresholds that were programmed for THIS measurement — the
 * env reduction is a statement about where the buckets sit in absolute power,
 * so feeding it thresholds from a different IGI silently changes the answer. */
inline NhmUtility nhm_utility(const uint8_t nhm[kNhmRptNum],
                              const uint8_t th[kNhmThNum]) {
  NhmUtility u;
  uint16_t sum = 0;
  for (int i = 0; i < kNhmRptNum; i++)
    sum = static_cast<uint16_t>(sum + nhm[i]);
  u.sum = sum;
  /* The vendor keeps the sum in a u8 and rejects an overflowing report; a sum
   * over kNhmRptMax means the read did not describe one window. */
  if (sum == 0 || sum > kNhmRptMax)
    return u;

  uint8_t wgt[kNhmRptNum];
  nhm_bucket_weights(th, wgt);

  int env = static_cast<int>(sum);

  /* Exclude the first populated cluster while it is still at or below the
   * IC-noise threshold — up to 4 buckets, stopping at the first that is not. */
  int first = -1;
  for (int i = 0; i < kNhmRptNum; i++) {
    if (nhm[i]) {
      first = i;
      break;
    }
  }
  if (first >= 0) {
    for (int i = 0; i < 4; i++) {
      const int k = first + i;
      if (k >= kNhmRptNum || wgt[k] > kNhmIcNoiseTh)
        break;
      env -= nhm[k];
    }
  }

  /* And exclude bucket 0 outright when even it is above the threshold — the
   * whole histogram is then sitting high and bucket 0 is the floor of it. */
  if (wgt[0] > kNhmIcNoiseTh)
    env -= nhm[0];

  if (env < 0)
    env = 0;

  u.valid = true;
  u.nhm_ratio = ccx_rpt_ratio(static_cast<uint16_t>(sum - nhm[0]), kNhmRptMax);
  u.env_ratio = ccx_rpt_ratio(static_cast<uint16_t>(env), kNhmRptMax);
  return u;
}

} /* namespace devourer */

#endif /* DEVOURER_NHM_ENV_MATH_H */
