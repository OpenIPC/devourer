#!/usr/bin/env python3
"""Discover / characterise the chip scrambler seed for the precoder.

The 802.11 scrambler is initialised to a (pseudo-random) non-zero 7-bit seed
*per PPDU*. To pre-descramble a PSDU so the chip emits a chosen subcarrier
pattern (encode_subcarriers.py), we must know the seed the chip will use. Two
strategies, mirroring the plan:

  --mode rx  (primary intent)
      Read the descrambler seed the chip recovers from frames it *receives*.
      Run `rxdemo` with DEVOURER_DUMP_SCRAMBLER=1 on a second adapter
      pointed at the precoder TX; it prints `<devourer-scrambler>seed=0xNN`
      lines. This script parses them and reports whether the seed is CONSTANT
      (one shaped PSDU works) or VARYING per frame (brute-force needed).

      CAVEAT: the seed field is only trustworthy when the RX adapter is an
      RTL8814AU — the 8812/8821 RX descriptor doesn't carry it there (see
      FrameParser.cpp). On 8812/8821 use --mode bruteforce.

  --mode bruteforce  (robust fallback, single adapter)
      Expand one target pattern into 128 candidate PSDUs (one per possible
      seed) via encode_subcarriers. Transmit them all; whichever frames the
      chip scrambles with seed k will show the target only when the matching
      variant-k PSDU was sent. Costs ~128x airtime but needs no seed knowledge
      and no second adapter.

Examples:
    # Spawn the RX demo on adapter 0x8813 and characterise for 20 s:
    uv run python seed_probe.py --mode rx --rx-pid 0x8813 --channel 6 --duration 20

    # Or parse a previously captured log / piped stdout:
    uv run python seed_probe.py --mode rx --from-log demo.log

    # Generate the 128-variant brute-force set from a target pattern:
    uv run python seed_probe.py --mode bruteforce --pattern target.txt --out bf
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from collections import Counter

import encode_subcarriers as enc

_SEED_RE = re.compile(rb"<devourer-scrambler>seed=0x([0-9a-fA-F]{2})")


def _iter_seed_lines(stream) -> "list[int]":
    seeds = []
    for raw in stream:
        if isinstance(raw, str):
            raw = raw.encode("utf-8", "replace")
        m = _SEED_RE.search(raw)
        if m:
            seeds.append(int(m.group(1), 16))
    return seeds


def _report(seeds: "list[int]") -> int:
    if not seeds:
        print("seed_probe: no <devourer-scrambler> lines seen. Is the RX demo "
              "running with DEVOURER_DUMP_SCRAMBLER=1, pointed at the precoder "
              "TX on the same channel?", file=sys.stderr)
        return 1
    hist = Counter(seeds)
    uniq = sorted(hist)
    print(f"seed_probe: {len(seeds)} frame(s), {len(uniq)} distinct seed(s)")
    for s, n in hist.most_common(8):
        print(f"  seed=0x{s:02x}  {n:5d} frame(s)  ({100*n/len(seeds):.1f}%)")
    if len(uniq) > 8:
        print(f"  ... and {len(uniq) - 8} more")
    if len(uniq) == 1:
        print(f"\nVERDICT: CONSTANT seed 0x{uniq[0]:02x} — a single shaped PSDU "
              f"works. Encode with --scrambler-seed 0x{uniq[0]:02x}.")
    else:
        print("\nVERDICT: seed VARIES across frames — the chip re-seeds per "
              "PPDU. Use --mode bruteforce (you cannot pin a single PSDU).")
    return 0


def mode_rx(args) -> int:
    if args.from_log:
        if args.from_log == "-":
            return _report(_iter_seed_lines(sys.stdin.buffer))
        with open(args.from_log, "rb") as fh:
            return _report(_iter_seed_lines(fh))

    # Spawn the RX demo and tee its output while collecting seeds.
    env = dict(os.environ, DEVOURER_DUMP_SCRAMBLER="1")
    if args.rx_pid:
        env["DEVOURER_PID"] = args.rx_pid
    if args.channel:
        env["DEVOURER_CHANNEL"] = str(args.channel)
    cmd = [args.demo_bin]
    print(f"seed_probe: launching {' '.join(cmd)} (DEVOURER_DUMP_SCRAMBLER=1, "
          f"pid={args.rx_pid or 'any'}, channel={args.channel or 'default'}) "
          f"for {args.duration}s", file=sys.stderr)
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, env=env)
    except FileNotFoundError:
        print(f"seed_probe: demo binary not found: {args.demo_bin} "
              f"(build it, or pass --demo-bin / --from-log)", file=sys.stderr)
        return 2
    seeds: "list[int]" = []
    try:
        import time
        end = time.monotonic() + args.duration
        assert proc.stdout is not None
        while time.monotonic() < end:
            line = proc.stdout.readline()
            if not line:
                break
            m = _SEED_RE.search(line)
            if m:
                seeds.append(int(m.group(1), 16))
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
    return _report(seeds)


def mode_bruteforce(args) -> int:
    if not args.pattern:
        print("--mode bruteforce requires --pattern", file=sys.stderr)
        return 2
    phy = enc.phy_params(args.phy)
    targets = enc.parse_pattern_file(args.pattern, phy.n_sd)
    prefix = args.out or "bf"
    for seed in range(128):
        res = enc.encode_pattern(targets, seed=seed, phy=phy, offset=args.offset)
        with open(f"{prefix}.seed_{seed:02x}.bin", "wb") as fh:
            fh.write(res.psdu_bytes)
    print(f"seed_probe: wrote 128 candidate PSDUs to {prefix}.seed_*.bin "
          f"({phy.name}, {targets.shape[0]} symbol(s)).")
    print("Transmit them in a loop (e.g. cycle precoder --psdu over each "
          "file); frames whose variant index matches the chip's per-frame seed "
          "will carry the target pattern. Capture with fft_capture.py.")
    return 0


def main(argv: "list[str] | None" = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=("rx", "bruteforce"), required=True)
    # rx
    ap.add_argument("--demo-bin", default="../../build/rxdemo",
                    help="path to rxdemo (rx mode)")
    ap.add_argument("--rx-pid", help="DEVOURER_PID for the RX adapter (rx mode)")
    ap.add_argument("--channel", type=int, help="capture channel (rx mode)")
    ap.add_argument("--duration", type=float, default=20.0,
                    help="seconds to collect (rx mode, default 20)")
    ap.add_argument("--from-log", help="parse seeds from a file or '-' (stdin) "
                    "instead of spawning the demo (rx mode)")
    # bruteforce
    ap.add_argument("--pattern", help="+-1 target pattern file (bruteforce mode)")
    ap.add_argument("--out", help="output filename prefix (bruteforce mode)")
    ap.add_argument("--phy", choices=("ht", "legacy"), default="legacy")
    ap.add_argument("--offset", type=int, default=0)
    args = ap.parse_args(argv)
    return mode_rx(args) if args.mode == "rx" else mode_bruteforce(args)


if __name__ == "__main__":
    raise SystemExit(main())
