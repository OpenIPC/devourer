#!/usr/bin/env python3
"""Point extraction + analysis for nitroqam_waterfall.sh — 256-QAM VHT on the
2.4 GHz band ("NitroQAM"/"TurboQAM") measured against the 11n 64-QAM ceiling.

Two subcommands:

  point   — called per cell by the .sh: slices the ground RX log by byte range,
            counts delivered frames, and decodes the sampled rx.txhit rates to
            prove *which* modulation actually flew. Emits one JSON line.

  report  — reads the accumulated points.jsonl: per-encoding delivery-vs-index
            curves, the interpolated dB gap at each threshold, and a per-cell
            modulation verdict.

The modulation verdict is the point of this harness. A cell can deliver frames
while the chip quietly aired something other than what was commanded, and a
delivery curve alone cannot tell the two apart. rx.txhit carries the decoded
DESC_RATE, so every cell reports how many sampled frames matched the commanded
(mode, mcs, nss) — a cell with delivery but no matching frames is a FALLBACK,
not a pass.

Note rx.txhit is sampled by rxdemo (first 10 canonical-SA frames, then every
100th), so mod_total is a sample count, never the delivered count.
"""
import argparse
import json
import os
import re
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from devourer_events import desc_rate_decode, parse_event  # noqa: E402


def expected_modulation(enc: str):
    """DEVOURER_TX_RATE spec -> ("ht"|"vht", mcs, nss), or None for a legacy
    rate (nothing to prove — the gate only guards HT/VHT modulation)."""
    tok = enc.split("/")[0].strip().upper()
    m = re.fullmatch(r"VHT(\d)SS_MCS(\d)", tok)
    if m:
        return ("vht", int(m.group(2)), int(m.group(1)))
    m = re.fullmatch(r"MCS(\d+)", tok)
    if m:
        mcs = int(m.group(1))
        return ("ht", mcs, mcs // 8 + 1)
    return None


def cmd_point(args):
    sent = failed = -1
    with open(args.txlog, errors="replace") as f:
        for line in f:
            ev = parse_event(line, "tx.stats")
            if ev is not None:
                sent = int(ev.get("submitted", -1))
                failed = int(ev.get("failed", 0))

    with open(args.rxlog, "rb") as f:
        f.seek(args.off0)
        blob = f.read(args.off1 - args.off0).decode(errors="replace")

    want = expected_modulation(args.enc)
    delivered = mod_total = mod_match = 0
    seen = defaultdict(int)
    for line in blob.splitlines():
        ev = parse_event(line)
        if ev is None:
            continue
        if ev["ev"] == "rx.energy":
            delivered += int(ev.get("frames") or 0)
        elif ev["ev"] == "rx.txhit":
            got = desc_rate_decode(ev.get("rate"))
            if got is None:
                continue
            mod_total += 1
            seen["%s%dss_mcs%d" % (got[0], got[2], got[1])] += 1
            if want is not None and got == want:
                mod_match += 1

    print(json.dumps({
        "enc": args.enc, "idx": args.idx, "sent": sent, "failed": failed,
        "delivered": delivered, "want": list(want) if want else None,
        "mod_total": mod_total, "mod_match": mod_match, "seen": dict(seen),
    }))


def crossing(curve, thr):
    """First index (interpolated) where the delivery ratio rises through thr.
    curve: sorted [(idx, ratio)]. Ported from ldpc_waterfall.py."""
    prev_i = prev_r = None
    for i, r in curve:
        if prev_r is not None and prev_r < thr <= r:
            return prev_i + (thr - prev_r) * (i - prev_i) / (r - prev_r)
        prev_i, prev_r = i, r
    return None


def cmd_report(args):
    curves = defaultdict(list)
    fallback = []
    for line in open(args.points, errors="replace"):
        line = line.strip()
        if not line:
            continue
        p = json.loads(line)
        if p["sent"] <= 0:
            print("! dropping enc=%s idx=%s: sent=%s (no final tx.stats — TX "
                  "died?)" % (p["enc"], p["idx"], p["sent"]), file=sys.stderr)
            continue
        ratio = min(1.0, p["delivered"] / p["sent"])
        curves[p["enc"]].append(p | {"ratio": ratio})
        # A cell that delivered frames but whose sampled rates never matched
        # the commanded modulation is airing something else.
        if p["want"] and p["mod_total"] > 0 and p["mod_match"] == 0:
            fallback.append(p)

    for enc in sorted(curves):
        pts = sorted(curves[enc], key=lambda p: p["idx"])
        print("\n== %s" % enc)
        print("  idx | sent  | delivered | ratio | modulation (matched/sampled)")
        for p in pts:
            bar = "#" * int(p["ratio"] * 30)
            if p["mod_total"] == 0:
                verdict = "no sample"
            elif p["mod_match"] == p["mod_total"]:
                verdict = "OK %d/%d" % (p["mod_match"], p["mod_total"])
            elif p["mod_match"] == 0:
                verdict = "FALLBACK 0/%d %s" % (p["mod_total"], p["seen"])
            else:
                verdict = "MIXED %d/%d %s" % (p["mod_match"], p["mod_total"],
                                              p["seen"])
            print("  %3d | %5d | %9d | %5.1f%% %-30s %s"
                  % (p["idx"], p["sent"], p["delivered"], p["ratio"] * 100,
                     bar, verdict))

    if fallback:
        print("\n!! %d cell(s) delivered frames but decoded as a DIFFERENT "
              "modulation than commanded — their delivery is not evidence for "
              "the commanded rate:" % len(fallback))
        for p in fallback:
            print("   enc=%s idx=%s want=%s seen=%s"
                  % (p["enc"], p["idx"], p["want"], p["seen"]))

    base = args.baseline
    if base not in curves:
        print("\n(no baseline encoding %r in the points — no dB gap readout)"
              % base)
        return
    base_curve = [(p["idx"], p["ratio"])
                  for p in sorted(curves[base], key=lambda p: p["idx"])]
    thrs = [float(t) for t in args.thresholds.split(",")]
    for enc in sorted(curves):
        if enc == base:
            continue
        cand = [(p["idx"], p["ratio"])
                for p in sorted(curves[enc], key=lambda p: p["idx"])]
        print("\n== SNR premium: %s vs %s (step %.2f dB/idx)"
              % (enc, base, args.step_qdb / 4.0))
        for thr in thrs:
            cb = crossing(base_curve, thr)
            cc = crossing(cand, thr)
            if cb is None or cc is None:
                print("  @%.0f%% delivery: not bracketed (base=%s, cand=%s) — "
                      "widen/lower the sweep" % (thr * 100, cb, cc))
                continue
            premium = (cc - cb) * args.step_qdb / 4.0
            print("  @%.0f%% delivery: base idx %.1f vs cand idx %.1f -> "
                  "%+.2f dB premium for %s"
                  % (thr * 100, cb, cc, premium, enc))


def self_test():
    """Cover the decode + crossing math the on-air runs cannot reach.

    A bench link that tops out at 16-QAM never produces a 256-QAM txhit, a
    legacy-rate fallback, or a threshold crossing between two curves — so those
    paths would otherwise ship unexercised."""
    # DESC_RATE decode across every section, including the multi-stream VHT
    # rates the 2-tuple desc_rate_to_mcs() deliberately drops.
    cases = {
        0x00: ("cck", 0, 1), 0x03: ("cck", 3, 1),
        0x04: ("ofdm", 0, 1), 0x0b: ("ofdm", 7, 1),
        0x0c: ("ht", 0, 1), 0x13: ("ht", 7, 1),
        0x14: ("ht", 8, 2), 0x1b: ("ht", 15, 2),
        0x2c: ("vht", 0, 1), 0x35: ("vht", 9, 1),
        0x36: ("vht", 0, 2), 0x3f: ("vht", 9, 2),
        0x40: ("vht", 0, 3), 0x53: ("vht", 9, 4),
    }
    for code, want in cases.items():
        got = desc_rate_decode(code)
        assert got == want, "desc_rate_decode(%#x) = %r, want %r" % (
            code, got, want)
    assert desc_rate_decode(None) is None
    assert desc_rate_decode(0x54) is None, "past VHT4SS_MCS9 must not decode"

    # The commanded-modulation parser the fallback verdict compares against.
    assert expected_modulation("VHT2SS_MCS9/40") == ("vht", 9, 2)
    assert expected_modulation("VHT1SS_MCS0") == ("vht", 0, 1)
    assert expected_modulation("MCS15/40/SGI") == ("ht", 15, 2)
    assert expected_modulation("MCS7/20") == ("ht", 7, 1)
    assert expected_modulation("6M/20") is None, "legacy has nothing to prove"

    # A commanded VHT2SS_MCS9 frame arriving as HT MCS15 is the fallback the
    # whole harness exists to catch — the two must not compare equal.
    assert desc_rate_decode(0x1b) != expected_modulation("VHT2SS_MCS9/40")
    assert desc_rate_decode(0x3f) == expected_modulation("VHT2SS_MCS9/40")

    # Threshold crossing: interpolates between bracketing points, and reports
    # None rather than guessing when the sweep never reaches the threshold.
    assert crossing([(0, 0.0), (10, 1.0)], 0.5) == 5.0
    assert crossing([(0, 0.0), (4, 0.2), (8, 0.6)], 0.5) == 7.0
    assert crossing([(0, 0.0), (10, 0.3)], 0.5) is None
    assert crossing([], 0.5) is None
    print("nitroqam_waterfall self-test: OK")


def main():
    if "--self-test" in sys.argv:
        self_test()
        return
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("point", help="extract one cell (called by the .sh)")
    p.add_argument("txlog")
    p.add_argument("rxlog")
    p.add_argument("off0", type=int)
    p.add_argument("off1", type=int)
    p.add_argument("enc")
    p.add_argument("idx", type=int)
    p.set_defaults(func=cmd_point)

    r = sub.add_parser("report", help="analyze points.jsonl")
    r.add_argument("points")
    r.add_argument("--baseline", default="MCS15/40/SGI",
                   help="encoding the others are compared against")
    r.add_argument("--step-qdb", type=float, default=2.0,
                   help="qdB per TXAGC index (2 = 0.5 dB on Jaguar1/2, "
                        "1 = 0.25 dB on Jaguar3)")
    r.add_argument("--thresholds", default="0.1,0.5,0.9")
    r.set_defaults(func=cmd_report)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
