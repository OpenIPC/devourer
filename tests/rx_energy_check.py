#!/usr/bin/env python3
"""Verdict for tests/rx_energy_probe.sh: is a CW interferer detectable in the
frame-free RX energy telemetry?

Parses the sensor's rx.energy events from a baseline (tone off) capture
and a tone-on capture, and asserts the two are clearly separable in the CCA
counter. A strong co-located CW pushes the sensor's cca_ofdm out of its ambient
band in one of two ways, both a valid detection:

  - SPIKE: cca_ofdm rises far above baseline (the CCA registers the carrier as
    busy). Seen on the 2T2R 8822CU (~13x).
  - COLLAPSE: cca_ofdm falls toward zero because the carrier saturates the AGC
    and the RX goes deaf. Seen on the 1T1R 8821AU / 8821C.

Pass if the tone-on median differs from the baseline median by >= 3x (either
direction), or the tone-on median collapses below a small floor while baseline
was clearly active.

  python3 rx_energy_check.py baseline.log tone.log
"""
from __future__ import annotations
import sys
import statistics

from devourer_events import iter_events


def cca_series(path: str) -> list[int]:
    out = []
    try:
        with open(path) as f:
            for ev in iter_events(f, ev="rx.energy"):
                v = ev.get("cca_ofdm")
                if v is not None:
                    out.append(int(v))
    except FileNotFoundError:
        pass
    # drop the first sample (often the pre-bring-up 0)
    return out[1:] if len(out) > 1 else out


def main() -> int:
    if len(sys.argv) != 3:
        sys.stderr.write("usage: rx_energy_check.py baseline.log tone.log\n")
        return 2
    base = cca_series(sys.argv[1])
    tone = cca_series(sys.argv[2])
    if not base or not tone:
        print(f"FAIL: not enough samples (baseline={len(base)}, tone={len(tone)})")
        return 1

    b = statistics.median(base)
    t = statistics.median(tone)
    print(f"baseline cca_ofdm median={b:.0f} (n={len(base)}, "
          f"min={min(base)} max={max(base)})")
    print(f"tone     cca_ofdm median={t:.0f} (n={len(tone)}, "
          f"min={min(tone)} max={max(tone)})")

    # SPIKE: tone median >= 3x baseline (the CCA registers the carrier as busy).
    # COLLAPSE: baseline was clearly active and the tone drives the CCA at least
    # 2.5x lower with its whole distribution below the baseline median (AGC
    # saturated / RX deaf). Both directions are a valid detection.
    spike = b > 0 and t >= 3 * b
    collapse = b >= 30 and t * 2.5 <= b and max(tone) < b
    if spike:
        print(f"PASS: tone SPIKES cca_ofdm {t/max(b,1):.1f}x above baseline "
              f"— interferer detected")
        return 0
    if collapse:
        print(f"PASS: tone COLLAPSES cca_ofdm {b/max(t,1):.1f}x (AGC saturation "
              f"/ RX deaf) — interferer detected")
        return 0
    print("FAIL: tone-on and baseline cca_ofdm not clearly separable "
          "(tone too weak, wrong channel, or not armed?)")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
