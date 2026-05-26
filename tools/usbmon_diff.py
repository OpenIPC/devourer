#!/usr/bin/env python3
"""Diff two usbmon captures by URB-level properties.

Use to compare a libusb-routed TX path vs. a kernel-routed (qemu USB passthrough)
TX path on the same chip — both go through host xhci so host usbmon sees both.
The decisive question is what URB-level property the kernel preserves and the
libusb userspace path drops.

Format: usbmon text format (the `Nu` debugfs file on kernel 6.18 emits this).
See Documentation/usb/usbmon.rst. Line layout:

    <URB_tag> <timestamp_us> <S|C|E> <Type<dir>:<bus>:<dev>:<ep>> \
        <status> <length> [<data_tag> <hex_data...>]

For BULK OUT submits ("S Bo:..."), status is -115 (-EINPROGRESS), length is the
transfer size, and data follows after "=". For completes ("C Bo:..."), status is
the kernel return code (0 = success, -110 = -ETIMEDOUT, -108 = -ESHUTDOWN), and
length is the actual transferred (0 on failure).
"""

import argparse
import collections
import re
import sys
from pathlib import Path

# Lines look like:
#   ffff8d61566acc00 2417645431 S Bo:4:002:2 -115 4136 = 00102804 ...
#   ffff8d6031d030c0 2429359860 C Bo:4:002:2 -108 0
#   ffff8d61566acc00 2417645472 C Bo:4:002:2 0 4136 >
LINE_RE = re.compile(
    r"^([0-9a-f]+)\s+(\d+)\s+([SCE])\s+"
    r"([BCIZ])([io]):(\d+):(\d+):(\d+)\s+"
    r"(-?\d+)\s+(\d+)"
)

XFER_LETTER = {"B": "BULK", "C": "CTRL", "I": "INTR", "Z": "ISO"}
ERRNO = {
    0: "OK",
    -2: "-ENOENT",
    -32: "-EPIPE",
    -71: "-EPROTO",
    -75: "-EOVERFLOW",
    -84: "-EILSEQ",
    -108: "-ESHUTDOWN",
    -110: "-ETIMEDOUT",
    -115: "-EINPROGRESS",
    -121: "-EREMOTEIO",
}


def parse_file(path):
    records = []
    with open(path, "r", errors="replace") as f:
        for line in f:
            m = LINE_RE.match(line)
            if not m:
                continue
            tag, ts, evt, xtype, dir_, bus, dev, ep, status, length = m.groups()
            records.append({
                "tag": tag,
                "ts": int(ts),         # microseconds since boot (kernel)
                "type": evt,
                "xfer_type": xtype,
                "dir": dir_,
                "bus": int(bus),
                "dev": int(dev),
                "ep": int(ep),
                "status": int(status),
                "length": int(length),
            })
    return records


def filter_bulk(records, *, ep, dev, direction="o"):
    return [r for r in records
            if r["xfer_type"] == "B"
            and r["dir"] == direction
            and r["ep"] == ep
            and r["dev"] == dev]


def pair_s_c(records):
    """Pair S and C records by URB tag (kernel URB pointer)."""
    submits = {}
    pairs = []
    orphan_c = 0
    for r in records:
        if r["type"] == "S":
            submits[r["tag"]] = r
        elif r["type"] == "C":
            s = submits.pop(r["tag"], None)
            if s is None:
                orphan_c += 1
            else:
                pairs.append((s, r))
    return pairs, len(submits), orphan_c


def pct(arr, p):
    if not arr:
        return None
    sa = sorted(arr)
    return sa[min(int(len(sa) * p / 100), len(sa) - 1)]


def summarize(label, pairs):
    if not pairs:
        return {"label": label, "n": 0}
    lat = [c["ts"] - s["ts"] for (s, c) in pairs]
    statuses = collections.Counter(ERRNO.get(c["status"], str(c["status"]))
                                   for (s, c) in pairs)
    sub_lengths = collections.Counter(s["length"] for (s, c) in pairs)
    s_ts = sorted(s["ts"] for (s, c) in pairs)
    intervals = [s_ts[i + 1] - s_ts[i] for i in range(len(s_ts) - 1)]
    return {
        "label": label,
        "n": len(pairs),
        "lat_p50_us": pct(lat, 50),
        "lat_p99_us": pct(lat, 99),
        "lat_mean_us": int(sum(lat) / len(lat)),
        "status_dist": dict(statuses.most_common()),
        "length_dist": dict(sub_lengths.most_common(5)),
        "interval_p50_us": pct(intervals, 50),
        "interval_p99_us": pct(intervals, 99),
        "duration_s": (s_ts[-1] - s_ts[0]) / 1_000_000 if len(s_ts) > 1 else 0,
        "rate_per_s": len(pairs) / max(1, (s_ts[-1] - s_ts[0]) / 1_000_000)
                       if len(s_ts) > 1 else 0,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file-a", required=True)
    ap.add_argument("--file-b", required=True)
    ap.add_argument("--ep", default="0x02", help="bulk-OUT endpoint (default 0x02)")
    ap.add_argument("--devnum-a", type=int, required=True)
    ap.add_argument("--devnum-b", type=int, required=True)
    ap.add_argument("--dump-first", type=int, default=2)
    args = ap.parse_args()

    ep = int(args.ep, 0)

    recs_a = parse_file(args.file_a)
    recs_b = parse_file(args.file_b)
    print(f"file A: {len(recs_a)} lines parsed  ({args.file_a})")
    print(f"file B: {len(recs_b)} lines parsed  ({args.file_b})")

    fa = filter_bulk(recs_a, ep=ep, dev=args.devnum_a)
    fb = filter_bulk(recs_b, ep=ep, dev=args.devnum_b)
    print(f"  filtered BULK OUT ep={ep} A.dev={args.devnum_a} B.dev={args.devnum_b}:"
          f" A={len(fa)} B={len(fb)}")

    pa, sa_orph, ca_orph = pair_s_c(fa)
    pb, sb_orph, cb_orph = pair_s_c(fb)
    print(f"  paired: A={len(pa)} (orphan S={sa_orph}/C={ca_orph})"
          f"  B={len(pb)} (orphan S={sb_orph}/C={cb_orph})")

    if args.dump_first > 0:
        print("\n--- first paired URBs (A) ---")
        for s, c in pa[:args.dump_first]:
            print(f"  S tag={s['tag']} ts={s['ts']} len={s['length']}")
            print(f"  C tag={c['tag']} ts={c['ts']} "
                  f"status={ERRNO.get(c['status'], c['status'])} len={c['length']} "
                  f"(latency={c['ts'] - s['ts']} µs)")
        print("\n--- first paired URBs (B) ---")
        for s, c in pb[:args.dump_first]:
            print(f"  S tag={s['tag']} ts={s['ts']} len={s['length']}")
            print(f"  C tag={c['tag']} ts={c['ts']} "
                  f"status={ERRNO.get(c['status'], c['status'])} len={c['length']} "
                  f"(latency={c['ts'] - s['ts']} µs)")

    sumA = summarize("A (devourer libusb)", pa)
    sumB = summarize("B (VM kernel xhci)", pb)

    print("\n========== DIFF ==========")
    keys = ["n", "duration_s", "rate_per_s",
            "lat_p50_us", "lat_p99_us", "lat_mean_us",
            "interval_p50_us", "interval_p99_us",
            "status_dist", "length_dist"]
    for k in keys:
        va = sumA.get(k)
        vb = sumB.get(k)
        print(f"  {k:18s}  A={va}")
        print(f"  {'':18s}  B={vb}")
        print()


if __name__ == "__main__":
    main()
