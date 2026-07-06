#!/usr/bin/env python3
"""antenna_decorrelation.py — measure inter-chain envelope correlation and the
realised diversity gain of a multi-chain Realtek adapter (spatial-diversity axis).

Every spatial-diversity claim in the roadmap rests on the RF chains' antennas
being *decorrelated*. On a small airframe they may not be — closely spaced,
same-polarisation antennas see almost the same fading, and the theoretical
~10*log10(N) dB array gain / N-branch diversity order collapses. This tool
measures the decorrelation that a given antenna layout actually delivers,
instead of assuming it.

Input is the per-chain PHY link metric emitted by ``rxdemo`` under
``DEVOURER_RX_ALLPATHS=1`` — one ``<devourer-rxpath>`` line per received frame:

    <devourer-rxpath>seq=N rssi=a,b,c,d snr=a,b,c,d evm=a,b,c,d

Paths C/D are meaningful only on the 8814AU (4T4R); on 2T2R parts they read 0
and are dropped automatically. The two-path ``<devourer-stream>`` line is also
parsed as a fallback (rssi=a,b / snr=a,b) so a 2T2R capture works with no demo
flag.

What it computes, per capture:
  * per-chain sample count, mean level, spread;
  * the pairwise *envelope* correlation matrix (Pearson on linear amplitude —
    the standard diversity-relevant correlation, not on dB);
  * combining analysis — empirical fade CDF of selection-combining (SC) and
    maximal-ratio-combining (MRC) vs the best single chain, and the dB gain at
    the 10 % and 1 % outage points;
  * an effective diversity order estimated from the low-outage CDF slope;
  * a verdict keyed off the worst pairwise correlation (rho < 0.7 is the classic
    "diversity still useful" threshold).

Usage:
  # analyse a capture file (or stdin)
  uv run python antenna_decorrelation.py capture.log
  cat capture.log | uv run python antenna_decorrelation.py -

  # hardware-independent self-test: synthesise chains with a known correlation
  # and confirm the estimator + combining math recover it
  uv run python antenna_decorrelation.py --self-test

The live end-to-end capture (build + run the demo + pipe here) is driven by
run_antenna_decorrelation.sh.
"""

from __future__ import annotations

import argparse
import re
import sys

import numpy as np

# <devourer-rxpath>seq=12 rssi=40,38,0,0 snr=25,24,0,0 evm=10,11,0,0
_RXPATH = re.compile(
    r"<devourer-rxpath>seq=(\d+)\s+rssi=([\-\d,]+)\s+snr=([\-\d,]+)\s+evm=([\-\d,]+)"
)
# fallback: canonical two-path stream line
_STREAM = re.compile(
    r"<devourer-stream>.*?rssi=(-?\d+),(-?\d+)\s+evm=(-?\d+),(-?\d+)\s+snr=(-?\d+),(-?\d+)"
)

# Correlation bands for the verdict (worst pairwise |rho|).
GOOD, MODERATE = 0.3, 0.7
# A valid envelope-correlation estimate needs a *fading* channel. A static
# near-field bench beacon barely varies (std ~0.5 unit), so the correlation just
# tracks quantisation noise and means nothing. Require at least this much
# per-chain spread (metric units) on the strongest-varying chain before trusting
# the estimate; below it the run is inconclusive and needs motion/multipath.
MIN_FADING_STD = 2.0
# A chain pinned to a near-constant value (often a railed/saturated per-path
# reading, e.g. path C clamping high) carries no information.
RAIL_STD = 0.15


def parse(lines, metric: str = "rssi"):
    """Return an (n_frames, n_paths) float array of the chosen metric.

    Trailing all-zero columns (idle paths C/D on a 2T2R part) are trimmed.
    """
    mi = {"rssi": 0, "snr": 1, "evm": 2}[metric]
    rows: list[list[float]] = []
    for ln in lines:
        m = _RXPATH.search(ln)
        if m:
            triplet = m.groups()[1:]  # rssi, snr, evm CSV strings
            vals = [int(x) for x in triplet[mi].split(",")]
            rows.append([float(v) for v in vals])
            continue
        m = _STREAM.search(ln)
        if m:
            g = [int(x) for x in m.groups()]
            pick = {"rssi": g[0:2], "snr": g[4:6], "evm": g[2:4]}[metric]
            rows.append([float(v) for v in pick])
    if not rows:
        return np.empty((0, 0))
    width = max(len(r) for r in rows)
    arr = np.array([r + [0.0] * (width - len(r)) for r in rows], dtype=float)
    # drop trailing columns that are identically zero (unused chains)
    active = arr.shape[1]
    while active > 1 and np.all(arr[:, active - 1] == 0.0):
        active -= 1
    return arr[:, :active]


def envelope_correlation(rssi_dbm: np.ndarray) -> np.ndarray:
    """Pairwise Pearson correlation of the linear voltage envelope.

    RSSI in the descriptor is a dB-scaled level; envelope correlation is defined
    on the linear amplitude, so convert dB -> linear amplitude first
    (amp = 10**(dBm/20)). Returns an (N, N) matrix; diagonal is 1.
    """
    amp = np.power(10.0, rssi_dbm / 20.0)
    # guard against a dead/constant chain (zero variance -> nan correlation)
    if amp.shape[0] < 2:
        return np.full((amp.shape[1], amp.shape[1]), np.nan)
    with np.errstate(invalid="ignore"):
        c = np.corrcoef(amp, rowvar=False)
    return np.atleast_2d(c)


def _outage_gain_db(single_snr_db, combined_snr_db, pct):
    """dB gain of `combined` over `single` at the `pct` outage point.

    The outage level is the SNR that is exceeded (100-pct)% of the time — i.e.
    the pct-th percentile of each series. The gain is how much more link margin
    the combiner buys at that (low) percentile, which is where diversity pays.
    """
    s = np.percentile(single_snr_db, pct)
    c = np.percentile(combined_snr_db, pct)
    return c - s


def combining_analysis(level_db: np.ndarray) -> dict:
    """Selection- and maximal-ratio-combining gain over the best single chain.

    `level_db` is per-chain per-frame level in dB (RSSI or SNR). Treated as a
    relative power proxy: linear power = 10**(dB/10).
    SC = max over chains; MRC = sum over chains (coherent power sum).
    """
    n_frames, n = level_db.shape
    pwr = np.power(10.0, level_db / 10.0)
    best_chain = int(np.argmax(level_db.mean(axis=0)))
    single_db = level_db[:, best_chain]
    sc_db = 10.0 * np.log10(pwr.max(axis=1))
    mrc_db = 10.0 * np.log10(pwr.sum(axis=1))
    out = {"n_paths": n, "best_chain": best_chain}
    for pct in (10, 1):
        out[f"sc_gain_p{pct}"] = _outage_gain_db(single_db, sc_db, pct)
        out[f"mrc_gain_p{pct}"] = _outage_gain_db(single_db, mrc_db, pct)
    out["mrc_mean_gain"] = float(mrc_db.mean() - single_db.mean())
    out["theoretical_array_gain"] = 10.0 * np.log10(n)
    return out


def summarise(arr: np.ndarray, metric: str) -> dict:
    n_frames, n = arr.shape
    corr = envelope_correlation(arr)
    # worst off-diagonal |rho|
    off = corr.copy()
    np.fill_diagonal(off, np.nan)
    worst = float(np.nanmax(np.abs(off))) if n > 1 else float("nan")
    combo = combining_analysis(arr)
    std = arr.std(axis=0)
    # Effective number of independent branches = participation ratio of the
    # correlation matrix, N_eff = (Sum lambda_i)^2 / Sum lambda_i^2 =
    # n^2 / Sum_ij rho_ij^2 (since trace(R)=n and trace(R^2)=Sum rho_ij^2).
    # n uncorrelated chains -> N_eff = n; fully correlated -> N_eff = 1. This is
    # the "how many chains actually buy me diversity" number. Only meaningful
    # when the channel actually fades (see fading_ok).
    neff = float(n * n / np.nansum(corr**2)) if n > 1 else 1.0
    return {
        "n_frames": n_frames,
        "n_paths": n,
        "metric": metric,
        "mean": arr.mean(axis=0),
        "std": std,
        "corr": corr,
        "worst_rho": worst,
        "combo": combo,
        "max_std": float(std.max()),
        "railed": [i for i in range(n) if std[i] < RAIL_STD],
        "fading_ok": float(std.max()) >= MIN_FADING_STD,
        "eff_branches": neff,
    }


def verdict(s: dict) -> str:
    if not s["fading_ok"]:
        return (f"INCONCLUSIVE — channel too static (max per-chain std "
                f"{s['max_std']:.2f} < {MIN_FADING_STD:.1f} units). Envelope "
                f"correlation needs a fading channel: move an antenna / add "
                f"multipath / vary the path, or measure a mobile link.")
    worst = s["worst_rho"]
    if np.isnan(worst):
        return "N/A (single chain)"
    if worst < GOOD:
        return "GOOD — chains well decorrelated, near-full diversity available"
    if worst < MODERATE:
        return "MODERATE — partial correlation, diversity gain reduced"
    return "POOR — chains highly correlated, diversity gain largely lost"


def report(s: dict) -> str:
    L = []
    L.append(f"frames={s['n_frames']}  active_chains={s['n_paths']}  metric={s['metric']}")
    if s["n_frames"] < 30:
        L.append("  WARNING: <30 frames — correlation estimate is unreliable")
    labels = ["A", "B", "C", "D"][: s["n_paths"]]
    L.append("per-chain: " + "  ".join(
        f"{lb}: mean={m:6.2f} std={sd:5.2f}"
        for lb, m, sd in zip(labels, s["mean"], s["std"])
    ))
    if s["railed"]:
        rl = ",".join(labels[i] for i in s["railed"])
        L.append(f"  NOTE: chain(s) {rl} near-constant (std < {RAIL_STD}) — "
                 f"likely railed/saturated per-path reading, treat as no-signal")
    if not s["fading_ok"]:
        L.append(f"  WARNING: max per-chain std {s['max_std']:.2f} < "
                 f"{MIN_FADING_STD:.1f} — channel too static for a valid "
                 f"correlation estimate (see VERDICT)")
    if s["n_paths"] > 1:
        L.append("envelope correlation matrix (linear amplitude):")
        header = "      " + "".join(f"{lb:>7}" for lb in labels)
        L.append(header)
        for lb, row in zip(labels, s["corr"]):
            L.append(f"   {lb:>3}" + "".join(f"{v:7.3f}" for v in row))
        L.append(f"worst pairwise |rho| = {s['worst_rho']:.3f}")
        if s["fading_ok"]:
            L.append(f"effective branches (N_eff) = {s['eff_branches']:.2f} "
                     f"of {s['n_paths']} chains")
        c = s["combo"]
        L.append(
            f"combining (best chain = {labels[c['best_chain']]}, "
            f"theoretical array gain = {c['theoretical_array_gain']:.1f} dB):"
        )
        L.append(
            f"   MRC gain over best single: mean={c['mrc_mean_gain']:.2f} dB  "
            f"@10%out={c['mrc_gain_p10']:.2f} dB  @1%out={c['mrc_gain_p1']:.2f} dB"
        )
        L.append(
            f"   SC  gain over best single: @10%out={c['sc_gain_p10']:.2f} dB  "
            f"@1%out={c['sc_gain_p1']:.2f} dB"
        )
    L.append(f"VERDICT: {verdict(s)}")
    return "\n".join(L)


# --------------------------------------------------------------------------- #
# Self-test: synthesise correlated Rayleigh-fading chains with a known rho and
# confirm the estimator (and the combining math) recover the expected values.
# --------------------------------------------------------------------------- #
def _synth_chains(n_frames, n_paths, rho_env, seed=0):
    """Correlated complex-Gaussian fading -> Rayleigh envelope -> RSSI dBm.

    `rho_env` is the desired *envelope* correlation. For complex-Gaussian
    (Rayleigh) fading the envelope/power correlation equals |field correlation|^2,
    so the underlying complex field is coloured with rho_field = sqrt(rho_env):
    an equicorrelation matrix (off-diagonals = rho_field) colours i.i.d. complex
    Gaussians through its Cholesky factor.
    """
    rng = np.random.default_rng(seed)
    rho = float(np.sqrt(rho_env))
    R = np.full((n_paths, n_paths), rho) + (1.0 - rho) * np.eye(n_paths)
    Lc = np.linalg.cholesky(R)
    re = (rng.standard_normal((n_frames, n_paths)) @ Lc.T)
    im = (rng.standard_normal((n_frames, n_paths)) @ Lc.T)
    env = np.sqrt(re**2 + im**2)  # Rayleigh envelope
    # map to a plausible RSSI window (~ -70 dBm nominal + fading, as positive
    # "gain_trsw"-style magnitude the demo reports)
    rssi = 40.0 + 20.0 * np.log10(env / np.sqrt(2.0) + 1e-9)
    return rssi


def self_test() -> int:
    print("=== antenna_decorrelation self-test ===")
    ok = True

    # 1) estimator recovers a known correlation (independent and correlated)
    for target in (0.0, 0.5, 0.9):
        rssi = _synth_chains(20000, 2, target, seed=1)
        est = envelope_correlation(rssi)[0, 1]
        err = abs(est - target)
        status = "ok" if err < 0.05 else "FAIL"
        ok &= err < 0.05
        print(f"[{status}] rho target={target:.2f} estimated={est:.3f} (err={err:.3f})")

    # 2) independent 4-chain MRC mean gain ~ theoretical 10log10(4)=6.0 dB
    rssi4 = _synth_chains(40000, 4, 0.0, seed=2)
    combo = combining_analysis(rssi4)
    gain, theo = combo["mrc_mean_gain"], combo["theoretical_array_gain"]
    # MRC mean-dB gain over the best single chain is at least the array gain
    # 10log10(N), and legitimately exceeds it because summing branches reduces
    # variance and so shrinks the log-bias of the single-branch mean. Require
    # it >= theory and within a few dB above.
    status = "ok" if (theo - 0.5) < gain < (theo + 3.5) else "FAIL"
    ok &= (theo - 0.5) < gain < (theo + 3.5)
    print(f"[{status}] indep 4-chain MRC mean gain={gain:.2f} dB (theory {theo:.1f} dB)")

    # 3) correlated chains yield less low-outage diversity gain than independent
    indep = combining_analysis(_synth_chains(40000, 2, 0.0, seed=3))["mrc_gain_p1"]
    corr = combining_analysis(_synth_chains(40000, 2, 0.9, seed=4))["mrc_gain_p1"]
    status = "ok" if corr < indep else "FAIL"
    ok &= corr < indep
    print(f"[{status}] 1%-outage MRC gain: independent={indep:.2f} dB > "
          f"correlated(rho=.9)={corr:.2f} dB")

    # 4) parser round-trips both line formats
    sample = [
        "<devourer-rxpath>seq=1 rssi=40,38,20,18 snr=25,24,10,9 evm=5,6,7,8",
        "junk line",
        "<devourer-stream>rate=4 len=100 crc_err=0 icv_err=0 rssi=41,39 "
        "evm=5,6 snr=26,25 seq=2 tsfl=0 bw=0 stbc=0 ldpc=0 sgi=0 body=deadbeef",
    ]
    a = parse(sample, "rssi")
    status = "ok" if a.shape == (2, 4) else "FAIL"  # stream row zero-padded to 4
    ok &= a.shape == (2, 4)
    print(f"[{status}] parser: 2 rows x 4 cols from mixed formats -> {a.shape}")

    # 5) fading guard: a near-static channel (like a bench beacon) is flagged
    #    INCONCLUSIVE, and a pinned chain is flagged railed.
    rng = np.random.default_rng(5)
    static = 78.0 + 0.3 * rng.standard_normal((2000, 4))
    static[:, 2] = 82.0  # railed chain
    s = summarise(static, "rssi")
    cond = (not s["fading_ok"]) and (2 in s["railed"]) and \
        verdict(s).startswith("INCONCLUSIVE")
    status = "ok" if cond else "FAIL"
    ok &= cond
    print(f"[{status}] fading guard: static channel -> INCONCLUSIVE, "
          f"railed={s['railed']}, max_std={s['max_std']:.2f}")

    # 6) effective-branch metric: independent 4-chain ~ 4; correlated < 4.
    indep_neff = summarise(_synth_chains(20000, 4, 0.0, seed=6), "rssi")["eff_branches"]
    corr_neff = summarise(_synth_chains(20000, 4, 0.8, seed=7), "rssi")["eff_branches"]
    cond = (indep_neff > 3.5) and (corr_neff < indep_neff)
    status = "ok" if cond else "FAIL"
    ok &= cond
    print(f"[{status}] N_eff: independent 4-chain={indep_neff:.2f} (~4), "
          f"correlated(rho=.8)={corr_neff:.2f} (< independent)")

    print("=== PASS ===" if ok else "=== FAIL ===")
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", nargs="?", default="-",
                    help="capture file with <devourer-rxpath> lines, or '-' for stdin")
    ap.add_argument("--metric", choices=("rssi", "snr", "evm"), default="rssi",
                    help="per-chain metric to analyse (default rssi)")
    ap.add_argument("--self-test", action="store_true",
                    help="run the hardware-independent estimator self-test and exit")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    if args.input == "-":
        lines = sys.stdin.readlines()
    else:
        with open(args.input, "r", errors="replace") as f:
            lines = f.readlines()

    arr = parse(lines, args.metric)
    if arr.size == 0:
        print("no <devourer-rxpath> or <devourer-stream> lines found — did you "
              "run rxdemo with DEVOURER_RX_ALLPATHS=1 (and receive "
              "frames)?", file=sys.stderr)
        return 2
    print(report(summarise(arr, args.metric)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
