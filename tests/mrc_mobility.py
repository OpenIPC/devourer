#!/usr/bin/env python3
"""mrc_mobility.py — measure RX combining gain under motion (spatial-diversity).

#130 found that at a *static* position spatial combining (4-chain MRC) barely
beats the best single chain — one antenna dominates a fixed spot. The mobility
hypothesis is that under motion each chain's fading is time-varying and the best
chain keeps changing, so a *fixed* single antenna repeatedly falls into nulls it
can't escape while MRC fills them. This tool measures that.

It consumes a capture from `rxdemo` run with the RX-path *toggle*:

    DEVOURER_RX_PATHS=0x22:0xFF@300   # alternate chain-B-only and all-4 every 300ms
    DEVOURER_RX_ALLPATHS=1            # emit per-chain rx.path events

The library emits an `rx.path_mask` event on every switch, inline with the
frame stream, so each frame is attributed to the mask active when it arrived:

    {"ev":"rx.path_mask","t":1234,"mask":"0x22"}
    {"ev":"rx.path","seq":..,"rssi":[a,b,c,d],"snr":[...],"evm":[...]}
    ...
    {"ev":"rx.path_mask","t":1534,"mask":"0xff"}
    ...

Because the toggle is fast relative to hand-motion, the two configs sample the
same fading process — an unconfounded comparison the sequential #130 sweep could
not make. Every marker window is the same nominal duration, so **frames per
window is directly comparable across masks** (no need to know the beacon fps).

Run it twice — once MOVING the RX (translate it through several wavelengths), once
STATIC at matched mean level — and compare: a combining gain that appears only
when moving is the confirmation that spatial diversity's value is mobility, not
static delivery.

What it reports, per mask:
  * windows, total frames, mean/min/std frames-per-window;
  * zero-frame (deep-fade) window count — the fixed chain's nulls;
  * delivery ratio of all-4 (0xff) vs each single-chain mask (the MRC gain);
and, from the all-4 windows, how often each chain is the strongest (the best
chain changing under motion — the mechanism).

Usage:
  uv run python mrc_mobility.py capture.log
  uv run python mrc_mobility.py --self-test
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

from devourer_events import parse_event

# How many frames to drop right after each switch. Toggling 0x808 causes a brief
# RX/AGC transient that drops a few frames on the leading edge of a window;
# dropping them de-biases the per-mask delivery. Tune with --settle if the
# ALL-mask windows show anomalous low-min/high-std (under-settled) — or slow the
# toggle so the transient is a smaller fraction of each window.
SETTLE_FRAMES = 5


def parse_windows(lines):
    """Return a list of (mask:int, rssi_rows:list[list[int]]) — one per marker
    window. Frames before the first marker are ignored (unknown mask)."""
    windows = []
    cur_mask = None
    cur_rows = []
    for ln in lines:
        ev = parse_event(ln)
        if ev is None:
            continue
        if ev["ev"] == "rx.path_mask":
            if cur_mask is not None:
                windows.append((cur_mask, cur_rows))
            cur_mask = int(ev["mask"], 16)
            cur_rows = []
        elif ev["ev"] == "rx.path" and cur_mask is not None:
            cur_rows.append([int(x) for x in ev["rssi"]])
    if cur_mask is not None:
        windows.append((cur_mask, cur_rows))
    return windows


def per_mask_stats(windows, settle=SETTLE_FRAMES):
    """Aggregate frames-per-window by mask (dropping the first `settle` frames of
    each window as switch-transient settling)."""
    from collections import defaultdict
    counts = defaultdict(list)   # mask -> [frames_per_window]
    for mask, rows in windows:
        counts[mask].append(max(0, len(rows) - settle))
    out = {}
    for mask, cs in counts.items():
        a = np.array(cs, dtype=float)
        out[mask] = {
            "windows": len(cs),
            "total": int(a.sum()),
            "mean": float(a.mean()) if len(a) else 0.0,
            "min": float(a.min()) if len(a) else 0.0,
            "std": float(a.std()) if len(a) else 0.0,
            "zero_windows": int((a == 0).sum()),
        }
    return out


def best_chain_trace(windows):
    """From the all-paths (0xff) windows, how often is each chain strongest?
    Uses per-window mean RSSI; a shifting winner under motion is the mechanism."""
    wins = np.zeros(4, dtype=int)
    n = 0
    for mask, rows in windows:
        if mask != 0xFF or not rows:
            continue
        a = np.array(rows, dtype=float)
        if a.shape[1] < 4:
            continue
        wins[int(np.argmax(a.mean(axis=0)))] += 1
        n += 1
    return wins, n


def _popcount_low(mask):
    return bin(mask & 0x0F).count("1") or bin(mask & 0xF0).count("1")


def report(windows, settle=SETTLE_FRAMES):
    L = []
    stats = per_mask_stats(windows, settle)
    if not stats:
        return "no mask windows found — was the capture run with a " \
               "DEVOURER_RX_PATHS toggle spec (e.g. 0x22:0xFF@300)?"
    labels = {0x11: "A", 0x22: "B", 0x44: "C", 0x88: "D", 0xFF: "ALL"}
    L.append(f"{'mask':>6} {'chains':>6} {'windows':>8} {'mean/win':>9} "
             f"{'min':>5} {'std':>6} {'0-frame win':>12}")
    for mask in sorted(stats):
        s = stats[mask]
        lbl = labels.get(mask, "?")
        L.append(f"  0x{mask:02x} {lbl:>4} {s['windows']:>8} {s['mean']:>9.1f} "
                 f"{s['min']:>5.0f} {s['std']:>6.1f} {s['zero_windows']:>12}")

    # combining gain: all-4 vs each single-chain mask present
    if 0xFF in stats:
        allmean = stats[0xFF]["mean"]
        L.append("")
        L.append(f"combining gain (all-4 mean/win = {allmean:.1f}):")
        singles = [m for m in stats if m in (0x11, 0x22, 0x44, 0x88)]
        for m in sorted(singles):
            sm = stats[m]["mean"]
            gain = (allmean / sm - 1.0) * 100 if sm > 0 else float("inf")
            L.append(f"   vs chain {labels[m]} (mean {sm:.1f}): "
                     f"{gain:+.0f}%  |  chain {labels[m]} deep-fade windows: "
                     f"{stats[m]['zero_windows']}/{stats[m]['windows']}")

    wins, n = best_chain_trace(windows)
    if n:
        L.append("")
        L.append(f"best chain over {n} all-4 windows (mechanism — shifts under "
                 f"motion): " + "  ".join(
                     f"{c}={w}" for c, w in zip("ABCD", wins)))
        dominant = wins.max() / n
        L.append(f"  most-frequent-best chain holds {dominant*100:.0f}% of "
                 f"windows ({'one chain dominates → static-like' if dominant > 0.8 else 'best chain shifts → mobile diversity'})")
    return "\n".join(L)


def self_test() -> int:
    print("=== mrc_mobility self-test ===")
    ok = True
    # Synthesise a toggle capture: ALL windows deliver ~40 frames; the fixed
    # single chain B fades (some windows near-zero), so ALL should beat B and B
    # should show deep-fade windows.
    rng = np.random.default_rng(0)
    lines = []
    for w in range(40):
        # ALL window: steady ~40 frames, all chains ~70
        lines.append('{"ev":"rx.path_mask","t":0,"mask":"0xff"}')
        for _ in range(40):
            lines.append('{"ev":"rx.path","seq":1,"rssi":[70,71,69,72],'
                         '"snr":[1,1,1,1],"evm":[0,0,0,0]}')
        # B-only window: fades — half the windows deliver few frames
        lines.append('{"ev":"rx.path_mask","t":0,"mask":"0x22"}')
        nb = 40 if rng.random() > 0.5 else int(rng.integers(0, 5))
        for _ in range(nb):
            lines.append('{"ev":"rx.path","seq":1,"rssi":[0,70,0,0],'
                         '"snr":[1,1,1,1],"evm":[0,0,0,0]}')

    windows = parse_windows(lines)
    stats = per_mask_stats(windows)
    cond1 = 0xFF in stats and 0x22 in stats
    cond2 = stats[0xFF]["mean"] > stats[0x22]["mean"]        # ALL beats fixed B
    cond3 = stats[0x22]["zero_windows"] > 0                   # B has deep fades
    cond4 = stats[0xFF]["zero_windows"] == 0                  # ALL never fully fades
    for name, c in [("parse both masks", cond1), ("ALL>B", cond2),
                    ("B has fade windows", cond3), ("ALL no fade", cond4)]:
        print(f"[{'ok' if c else 'FAIL'}] {name}")
        ok &= c
    print(report(windows))
    print("=== PASS ===" if ok else "=== FAIL ===")
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", nargs="?", default="-")
    ap.add_argument("--settle", type=int, default=SETTLE_FRAMES,
                    help=f"frames to drop after each mask switch (default {SETTLE_FRAMES})")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        return self_test()
    lines = (sys.stdin.readlines() if args.input == "-"
             else open(args.input, errors="replace").readlines())
    print(report(parse_windows(lines), args.settle))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
