/* Headless guard for the NHM utility reduction (src/NhmEnvMath.h): the bucket
 * weights, the rounded ratio helper, and the noise-floor-cluster exclusion that
 * separates nhm_env_ratio from the naive nhm_ratio. Pure math; no hardware.
 *
 * The case that matters most is the last one: a histogram whose mass sits
 * entirely in the receiver's own floor must reduce to env_ratio 0 while the
 * naive ratio reads busy. That difference is the whole reason the env form
 * exists. */
#include "NhmEnvMath.h"

#include <cstdio>
#include <cstring>

using devourer::ccx_rpt_ratio;
using devourer::kNhmRptNum;
using devourer::kNhmThNum;
using devourer::nhm_bucket_weights;
using devourer::nhm_utility;
using devourer::NhmUtility;

static int g_fail = 0;

static void check(const char *what, long got, long want) {
  if (got != want) {
    std::printf("FAIL %s: got %ld want %ld\n", what, got, want);
    ++g_fail;
  }
}

/* The NHM_BACKGROUND threshold recipe read_nhm() programs, for a given IGI. */
static void th_for_igi(int igi, uint8_t th[kNhmThNum]) {
  int base = (igi - 14) * 2;
  if (base < 0)
    base = 0;
  for (int i = 0; i < kNhmThNum; i++) {
    const int v = base + 4 * i;
    th[i] = v > 255 ? 255 : static_cast<uint8_t>(v);
  }
}

int main() {
  /* --- ccx_rpt_ratio: rounds half up, and never divides by zero. --- */
  check("ratio 0/255", ccx_rpt_ratio(0, 255), 0);
  check("ratio 255/255", ccx_rpt_ratio(255, 255), 100);
  check("ratio 128/255", ccx_rpt_ratio(128, 255), 50);  /* 50.2 -> 50 */
  check("ratio 1/255", ccx_rpt_ratio(1, 255), 0);       /* 0.39 -> 0 */
  check("ratio 2/255", ccx_rpt_ratio(2, 255), 1);       /* 0.78 -> 1 */
  check("ratio denom 0", ccx_rpt_ratio(10, 0), 0);
  /* CLM uses it with a period denominator, not 255. */
  check("ratio 250/500", ccx_rpt_ratio(250, 500), 50);
  check("ratio 500/500", ccx_rpt_ratio(500, 500), 100);

  /* --- bucket weights: midpoints, with the end buckets offset by 2. --- */
  {
    uint8_t th[kNhmThNum], wgt[kNhmRptNum];
    th_for_igi(0x20, th); /* base = (32-14)*2 = 36, step 4 */
    check("th[0] igi32", th[0], 36);
    check("th[10] igi32", th[10], 76);
    nhm_bucket_weights(th, wgt);
    check("wgt[0]", wgt[0], 34);                 /* th[0] - 2 */
    check("wgt[1]", wgt[1], (36 + 40) / 2);      /* midpoint th0..th1 */
    check("wgt[10]", wgt[10], (72 + 76) / 2);
    check("wgt[11]", wgt[11], 78);               /* th[10] + 2 */
    /* Weights must not decrease, or the cluster walk below is meaningless. */
    for (int i = 1; i < kNhmRptNum; i++)
      if (wgt[i] < wgt[i - 1])
        check("wgt monotone", i, -1);
  }

  /* A low IGI clamps th[0] to 0, so wgt[0] clamps to 0 rather than wrapping. */
  {
    uint8_t th[kNhmThNum], wgt[kNhmRptNum];
    th_for_igi(0x0e, th); /* base = 0 */
    check("th[0] igi14", th[0], 0);
    nhm_bucket_weights(th, wgt);
    check("wgt[0] clamped", wgt[0], 0);
  }

  /* --- utility reduction --- */
  uint8_t th_lo[kNhmThNum]; /* all weights <= 60: the whole span is floor */
  th_for_igi(0x1c, th_lo);  /* base = (28-14)*2 = 28, th[10] = 68 */
  uint8_t th_hi[kNhmThNum]; /* all weights > 60: the whole span is above floor */
  th_for_igi(0x32, th_hi);  /* base = (50-14)*2 = 72, wgt[0] = 70 */

  {
    /* Empty histogram: no measurement, not a clean channel. */
    uint8_t h[kNhmRptNum] = {};
    const NhmUtility u = nhm_utility(h, th_lo);
    check("empty invalid", u.valid, 0);
    check("empty sum", u.sum, 0);
  }
  {
    /* Over-full histogram: the read did not describe one window. */
    uint8_t h[kNhmRptNum] = {};
    for (int i = 0; i < kNhmRptNum; i++)
      h[i] = 30; /* 360 > 255 */
    const NhmUtility u = nhm_utility(h, th_lo);
    check("overfull invalid", u.valid, 0);
    check("overfull sum", u.sum, 360);
  }
  {
    /* THE case: all mass in the leading low-power cluster. The naive ratio
     * counts everything outside bucket 0 as activity; the env form recognises
     * the whole cluster as the IC's own floor and reports nothing. */
    uint8_t h[kNhmRptNum] = {};
    h[0] = 50;
    h[1] = 100;
    h[2] = 55;
    const NhmUtility u = nhm_utility(h, th_lo);
    check("floor-only valid", u.valid, 1);
    check("floor-only sum", u.sum, 205);
    check("floor-only nhm_ratio", u.nhm_ratio, ccx_rpt_ratio(155, 255)); /* 61 */
    check("floor-only env_ratio", u.env_ratio, 0);
  }
  {
    /* A real interferer parks mass high, outside the 4-bucket cluster window.
     * The floor is still excluded; the hot bucket survives. */
    uint8_t h[kNhmRptNum] = {};
    h[0] = 50;
    h[1] = 60;
    h[8] = 90;
    const NhmUtility u = nhm_utility(h, th_lo);
    check("hot valid", u.valid, 1);
    check("hot env_ratio", u.env_ratio, ccx_rpt_ratio(90, 255)); /* 35 */
    check("hot nhm_ratio", u.nhm_ratio, ccx_rpt_ratio(150, 255));
  }
  {
    /* The cluster walk stops at 4 buckets even when more remain below the
     * threshold, so a broad low hump is not silently zeroed. */
    uint8_t h[kNhmRptNum] = {};
    for (int i = 0; i < 6; i++)
      h[i] = 20;
    const NhmUtility u = nhm_utility(h, th_lo);
    check("wide-hump sum", u.sum, 120);
    check("wide-hump env_ratio", u.env_ratio, ccx_rpt_ratio(40, 255));
  }
  {
    /* High IGI: even bucket 0 sits above -80 dBm, so it is excluded outright
     * and the leading cluster is not excluded at all. */
    uint8_t h[kNhmRptNum] = {};
    h[0] = 100;
    h[3] = 60;
    const NhmUtility u = nhm_utility(h, th_hi);
    uint8_t wgt[kNhmRptNum];
    nhm_bucket_weights(th_hi, wgt);
    check("th_hi wgt[0] above floor", wgt[0] > devourer::kNhmIcNoiseTh, 1);
    check("high-igi env_ratio", u.env_ratio, ccx_rpt_ratio(60, 255));
    check("high-igi nhm_ratio", u.nhm_ratio, ccx_rpt_ratio(60, 255));
  }
  {
    /* Mass only in bucket 11 (railed hot): nothing is excluded. */
    uint8_t h[kNhmRptNum] = {};
    h[11] = 255;
    const NhmUtility u = nhm_utility(h, th_lo);
    check("railed env_ratio", u.env_ratio, 100);
    check("railed nhm_ratio", u.nhm_ratio, 100);
  }
  {
    /* Exclusion can never drive the result negative. */
    uint8_t h[kNhmRptNum] = {};
    h[0] = 200;
    const NhmUtility u = nhm_utility(h, th_hi); /* bucket 0 excluded twice */
    check("no underflow", u.env_ratio, 0);
  }

  if (g_fail) {
    std::printf("nhm_env_math: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("nhm_env_math: all checks passed\n");
  return 0;
}
