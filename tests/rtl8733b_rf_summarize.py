#!/usr/bin/env python3
"""Companion analyzer for rtl8733b_rf_characterize.sh.

window <log> <byte0> <byte1> <tag> <level>
    Summarize witness rx.frame EVM/RSSI inside one cell's byte window of the
    witness log. Exits nonzero when the cell decoded nothing at all (a CCK
    cell reporting frames without EVM is fine — the witness carries no EVM
    for CCK — and is printed as such).

soak <log>
    First-quarter vs last-quarter median EVM over the whole log — the
    stability read for the bounded TSSI soak.
"""
import json
import statistics
import sys


def frames(path, start=0, end=None):
    with open(path, errors="replace") as fh:
        fh.seek(start)
        data = fh.read(None if end is None else end - start)
    for line in data.splitlines():
        if '"ev":"rx.frame"' not in line:
            continue
        try:
            yield json.loads(line)
        except ValueError:
            continue


def main() -> int:
    mode = sys.argv[1]
    if mode == "window":
        path, b0, b1, tag, lvl = (sys.argv[2], int(sys.argv[3]),
                                  int(sys.argv[4]), sys.argv[5], sys.argv[6])
        evm, rssi, n_frames = [], [], 0
        for ev in frames(path, b0, b1):
            n_frames += 1
            if ev.get("evm") and ev["evm"][0] not in (None, 0):
                evm.append(ev["evm"][0])
                rssi.append(ev["rssi"][0])
        if evm:
            med = f"{statistics.median(evm):.0f}"
            p90 = f"{sorted(evm)[int(len(evm) * 0.9)]:.0f}"
            mr = f"{statistics.median(rssi):.0f}"
            print(f"{tag:<14} {lvl:<12} n={len(evm):<6} medEVM={med:<5} "
                  f"p90EVM={p90:<5} medRSSI={mr}")
        elif n_frames:
            print(f"{tag:<14} {lvl:<12} n={n_frames:<6} (no EVM reported — "
                  f"expected for CCK)")
        else:
            print(f"{tag:<14} {lvl:<12} NO FRAMES — cell failed")
            return 1
        return 0
    if mode == "soak":
        evm = [ev["evm"][0] for ev in frames(sys.argv[2])
               if ev.get("evm") and ev["evm"][0] not in (None, 0)]
        if len(evm) < 400:
            print(f"soak: only {len(evm)} EVM frames — failed")
            return 1
        q = len(evm) // 4
        print(f"soak witness EVM: n={len(evm)} "
              f"first-quarter med={statistics.median(evm[:q]):.0f} "
              f"last-quarter med={statistics.median(evm[-q:]):.0f}")
        return 0
    print(f"unknown mode {mode}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
