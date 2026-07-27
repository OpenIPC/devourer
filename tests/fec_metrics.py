#!/usr/bin/env python3
"""Fused-FEC delivery accounting shared by the jammer harnesses.

One place that knows how a captured rxdemo event stream becomes the three
numbers worth reporting, so the fixed-hop matrix (jammer_resilience.py) and the
adaptive-hopset harness (hopset_adaptive_jammer.sh) cannot drift apart and
report the same word for different arithmetic:

  fec_delivery  = sbi_packets  / P   RS outer code + sub-block salvage
  base_delivery = base_packets / P   drop-the-whole-corrupt-frame baseline
  raw_frames    = (frames_seen - frames_corrupt) / frames_sent

P is the source-packet count of the exact payload the transmitter was fed,
obtained by encoding it and decoding it back through a clean loopback — not
estimated from a byte count, because the RS block padding at flush makes those
differ.

As a CLI it takes one capture and prints one JSON line, which is how the bash
harness consumes it:

  python3 tests/fec_metrics.py --capture rx_adapt.log --payload payload.bin \\
      --sent 4200 --label adapt
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_ROOT, "tools", "precoder"))

# The fused-FEC config. Read out of fused_fec_tx.py's own argument parser
# rather than re-declared here: P is computed in this module while the decode
# runs in fused_fec_rx.py, and the harness invokes both with no flags, so a
# default changed in one place and copied in another would silently rescale
# every delivery figure with nothing raising.
def _defaults() -> dict:
    import argparse as _ap
    from fused_fec_tx import add_fec_args
    p = _ap.ArgumentParser()
    add_fec_args(p)
    return vars(p.parse_args([]))


_D = _defaults()
FEC_K = _D["k"]
FEC_SYMBOL = _D["symbol_size"]
FEC_OVERHEAD = _D["overhead"]
FEC_BLOCKS_PER_BODY = _D["blocks_per_body"]
FEC_SCHEME = _D["scheme"]
FEC_CRC_BYTES = _D["crc_bytes"]


def build_payload(nbytes: int, seed: int = 0xC0FFEE) -> bytes:
    """Deterministic payload, so P and the run are reproducible across cells."""
    import numpy as np
    return np.random.default_rng(seed).integers(
        0, 256, size=nbytes, dtype=np.uint8).tobytes()


def _config():
    from stream_fec import FecConfig
    return FecConfig(k=FEC_K, symbol_size=FEC_SYMBOL, overhead=FEC_OVERHEAD,
                     scheme=FEC_SCHEME)


def packet_ladder(payload: bytes) -> list[int]:
    """Cumulative source-packet count after each body, clean loopback.

    The ladder, not just the total, because a phase ends on the clock and not
    on the payload: the honest denominator for a run that aired N bodies is the
    packets carried by the FIRST N bodies, and nothing beyond them can be
    counted as lost when it was never sent.
    """
    from fused_fec_link import FusedFecSender, FusedFecReceiver
    cfg = _config()
    snd = FusedFecSender(cfg, FEC_BLOCKS_PER_BODY, crc_bytes=FEC_CRC_BYTES)
    bodies = snd.add_bytes(payload) + snd.flush()
    rcv = FusedFecReceiver(cfg, FEC_BLOCKS_PER_BODY, crc_bytes=FEC_CRC_BYTES)
    ladder, run = [0], 0
    for b in bodies:
        run += len(rcv.add_frame(b, False))
        ladder.append(run)
    return ladder


def fec_counts(payload: bytes) -> tuple[int, int]:
    """(P source packets, B bodies) for `payload` under the default config."""
    ladder = packet_ladder(payload)
    return ladder[-1], len(ladder) - 1


def body_stride() -> int:
    """On-air bytes of one SBI sub-block (CRC + envelope)."""
    from fused_fec_link import env_size
    return FEC_CRC_BYTES + env_size(_config())


def scan_capture(path: str) -> dict:
    """Cheap pre-pass over the capture: how many received frames actually
    carried payload.

    txdemo airs three kinds of frame on this SA — payload frames, the
    marker-only frames of a recovery-probe dwell, and the authenticated
    schedule-control frames — and only the first kind can deliver a shard.
    Counting all three as delivery attempts would quietly inflate the frame
    ratio above 1. They separate on length alone.
    """
    from fec_subblock import SBI_HDR_LEN
    from devourer_events import iter_events
    floor = 24 + SBI_HDR_LEN + body_stride()  # 802.11 header + one sub-block
    out = {"frames": 0, "corrupt": 0, "payload_frames": 0,
           "payload_corrupt": 0}
    if not os.path.exists(path):
        return out
    with open(path, "rb") as fh:
        for ev in iter_events(fh, ev="rx.frame"):
            bad = bool(ev.get("crc") or 0) or bool(ev.get("icv") or 0)
            out["frames"] += 1
            out["corrupt"] += bad
            if int(ev.get("len") or 0) >= floor:
                out["payload_frames"] += 1
                out["payload_corrupt"] += bad
    return out


def parse_fec_report(stderr_text: str) -> dict:
    """Pull the counters out of fused_fec_rx.py's stderr report."""
    out = {"frames_seen": 0, "frames_corrupt": 0,
           "base_packets": 0, "sbi_packets": 0}
    for line in stderr_text.splitlines():
        for key, tok in (("frames_seen", "frames="),
                         ("frames_corrupt", "corrupt=")):
            if tok in line:
                try:
                    out[key] = int(line.split(tok)[1].split()[0])
                except (IndexError, ValueError):
                    pass
        if "baseline blocks=" in line and "pkts=" in line:
            out["base_packets"] = int(line.split("pkts=")[1].split()[0])
        if line.strip().startswith("fused_fec_rx: sbi") and "pkts=" in line:
            out["sbi_packets"] = int(line.split("pkts=")[1].split()[0])
    return out


def decode_capture(capture_path: str, py: str = sys.executable) -> dict:
    """Run the captured rxdemo event stream through fused_fec_rx.py offline.

    Offline and not piped live: the per-frame RS decode is slower than the
    frame rate, so a live pipe backs up, stalls rxdemo's USB reaping and drops
    the frames the run is trying to measure.
    """
    fec_rx = os.path.join(_ROOT, "tools", "precoder", "fused_fec_rx.py")
    if not os.path.exists(capture_path):
        return parse_fec_report("")
    with open(capture_path, "rb") as fh:
        r = subprocess.run([py, fec_rx], stdin=fh,
                           stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    return parse_fec_report(r.stderr.decode("utf-8", "replace"))


def cell_metrics(capture_path: str, packets: int, sent: int,
                 py: str = sys.executable, secs: float = 0.0, **extra) -> dict:
    """One capture -> one `jam.cell` record. `sent` is the transmitter's own
    count of bodies it aired (txdemo's tx.stdin / streamtx's stream.done), so
    a body that never left is never scored as one that failed to arrive."""
    rep = decode_capture(capture_path, py)
    scan = scan_capture(capture_path)
    seen = scan["payload_frames"] or rep["frames_seen"]
    corrupt = scan["payload_corrupt"] if scan["payload_frames"] \
        else rep["frames_corrupt"]
    clean = max(0, seen - corrupt)
    res = {
        "ev": "jam.cell",
        "P": packets,
        "frames_sent": sent,
        "frames_seen": seen,
        "frames_corrupt": corrupt,
        "frames_all": scan["frames"],
        "base_packets": rep["base_packets"],
        "sbi_packets": rep["sbi_packets"],
        "raw_frames": round(clean / sent, 4) if sent else 0.0,
        "base_delivery": round(rep["base_packets"] / packets, 4) if packets else 0.0,
        "fec_delivery": round(rep["sbi_packets"] / packets, 4) if packets else 0.0,
    }
    if secs > 0:
        # Ratios alone hide half the story on a CSMA link: a transmitter that
        # defers to an interferer does not lose the packets it would have sent
        # on that channel, it never sends them. That cost is a rate, and only
        # a rate shows it.
        res["secs"] = round(secs, 1)
        res["body_rate"] = round(sent / secs, 1)
        res["pkt_rate"] = round(rep["sbi_packets"] / secs, 1)
    res.update(extra)
    return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--capture",
                    help="rxdemo JSONL capture (DEVOURER_STREAM_OUT=1)")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--payload", help="the payload file the TX was fed")
    g.add_argument("--packets", type=int, help="P, if already known")
    ap.add_argument("--sent", type=int, default=0,
                    help="bodies the transmitter had aired by the END of the "
                         "measured window (0 = unknown)")
    ap.add_argument("--sent-from", type=int, default=0,
                    help="bodies already aired when the window OPENED, so a "
                         "settled slice of a longer run is scored against the "
                         "packets sent inside it and nothing else")
    ap.add_argument("--secs", type=float, default=0.0,
                    help="window duration, for the throughput columns")
    ap.add_argument("--label", default="",
                    help="free-form cell name carried through to the record")
    ap.add_argument("--gen-payload", type=int, default=0, metavar="BYTES",
                    help="instead of measuring: write BYTES of deterministic "
                         "payload to --payload and print P/B")
    args = ap.parse_args(argv)

    if args.gen_payload:
        if not args.payload:
            ap.error("--gen-payload needs --payload")
        data = build_payload(args.gen_payload)
        with open(args.payload, "wb") as fh:
            fh.write(data)
        ladder = packet_ladder(data)
        # Cache it: the encode is the expensive half, and every phase of a run
        # needs the same ladder against a different body count.
        with open(args.payload + ".pcum", "w") as fh:
            json.dump(ladder, fh)
        print(json.dumps({"ev": "fec.payload", "bytes": args.gen_payload,
                          "P": ladder[-1], "B": len(ladder) - 1}), flush=True)
        return 0

    if not args.capture or (args.payload is None and args.packets is None):
        ap.error("measuring needs --capture and one of --payload / --packets")
    if args.packets is not None:
        P = args.packets
    else:
        ladder = None
        try:
            with open(args.payload + ".pcum") as fh:
                ladder = json.load(fh)
        except (OSError, ValueError):
            with open(args.payload, "rb") as fh:
                ladder = packet_ladder(fh.read())
        # The transmitter may have looped the payload more than once, so the
        # ladder wraps: whole passes contribute their full packet count and
        # the remainder indexes the ladder.
        nb, total = len(ladder) - 1, ladder[-1]
        cum = lambda n: (n // nb) * total + ladder[n % nb]
        hi = args.sent if args.sent else nb
        lo = min(args.sent_from, hi)
        P = cum(hi) - cum(lo)
    rec = cell_metrics(args.capture, P, max(0, args.sent - args.sent_from),
                       secs=args.secs,
                       **({"cell": args.label} if args.label else {}))
    print(json.dumps(rec), flush=True)
    # Human line on stderr so a shell harness can show it without re-parsing
    # the record it just wrote to its own results file.
    sys.stderr.write(
        "      FEC  fec_delivery={fec_delivery:.3f}  "
        "base_delivery={base_delivery:.3f}  raw_frames={raw_frames:.3f}  "
        "(sent={frames_sent} seen={frames_seen} corrupt={frames_corrupt} "
        "P={P})\n".format(**rec))
    if args.secs > 0:
        sys.stderr.write(
            "           throughput {body_rate} bodies/s aired, "
            "{pkt_rate} packets/s recovered over {secs}s\n".format(**rec))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
