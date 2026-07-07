#!/usr/bin/env python3
"""Corruption analysis for the precoder stream link.

Reads `rx.frame` JSONL events on stdin (typically piped from
`rxdemo` with both `DEVOURER_STREAM_OUT=1` and
`DEVOURER_RX_KEEP_CORRUPTED=1`), reconstructs what each received body
*should* have been from a known source file, and reports byte/bit-level
error statistics.

Workflow (TX side):
    python3 tools/precoder/stream_tx.py --input source.bin --repeat 1 | \
        ./build/streamtx

Workflow (RX side, this tool):
    DEVOURER_STREAM_OUT=1 DEVOURER_RX_KEEP_CORRUPTED=1 ./build/rxdemo |
        python3 tools/precoder/corruption_analysis.py --source source.bin

The TX side encodes `source.bin` deterministically into N body frames. RX
captures every body matching the canonical SA, including those the chip
flagged with crc / icv (without `KEEP_CORRUPTED` the parser drops
them; with it on they reach us with the descriptor flags set). For each
captured body we:

  1. Read the (possibly corrupt) seq from envelope bytes 2-3,
  2. Look up what the encoder would have produced for that seq,
  3. XOR the received body prefix against the expected envelope,
  4. Accumulate per-byte and per-bit error counts plus a histogram of
     errors against in-frame byte offset (helpful for spotting whether
     the corruption is uniform, clustered near the start/end, or
     coincides with the 802.11 SERVICE field offset).

What this tells you that the chip's CRC bit doesn't:
  * Whether corrupted frames are mostly clean with a single byte off
    (good FEC opportunity) or wholly scrambled.
  * Whether errors are uniformly distributed across the body or
    concentrated in a band (e.g. last few bytes, where the chip's
    802.11 FCS sits and the trailing OFDM symbols are most fragile).
  * Whether a particular seq pattern (e.g. wrap-around) corrupts more
    often than others.

The output is plain text; pipe to `column -t` or similar if you want a
quick table.
"""

from __future__ import annotations

import argparse
import collections
import os
import sys
from pathlib import Path
from typing import Optional

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tests"))

import stream  # noqa: E402
from devourer_events import parse_event  # noqa: E402


def _effective_snr(snr: Optional[tuple[int, int]]) -> Optional[int]:
    """Pick the SNR value that actually drove decode quality.

    The two-path field carries SNR for paths A and B; on single-antenna
    USB sticks path B reads 0 (no signal, not "0 dB SNR"), so a naive
    min(A,B) would always report 0 and the BER-vs-SNR view collapses.
    `max(A,B)` works for both 1T1R (B is 0, A drives) and 2T2R single-
    stream operation (the chip picks the stronger path for the only
    stream). For an honest 2T2R two-stream analysis a finer model
    would be needed; this is a single-stream PoC.
    """
    if snr is None:
        return None
    return max(snr)


def _snr_bucket(snr: Optional[tuple[int, int]]) -> str:
    """Group SNR into 5-dB buckets. Returns 'no-snr' when absent."""
    eff = _effective_snr(snr)
    if eff is None:
        return "no-snr"
    base = (eff // 5) * 5
    return f"{base:>3d}-{base + 5} dB"


def _expected_bodies(source: bytes, mtu: int, body_bytes: int,
                     seq_start: int = 0) -> dict[int, bytes]:
    """Reproduce the TX side's encoded envelopes for `source`. Byte mode
    only — shape mode's bodies are seed/offset/state-dependent and would
    need their full encoder state to reconstruct."""
    frames = stream.pack_stream(source, mtu=mtu, seq_start=seq_start)
    return {f.seq: f.envelope_bytes(body_bytes) for f in frames}


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--source", required=True,
                    help="path to the byte stream the TX side sent")
    ap.add_argument("--mtu", type=int,
                    default=stream.DEFAULT_BODY_BYTES - stream.ENVELOPE_LEN,
                    help="payload size per frame (must match the TX side)")
    ap.add_argument("--body-bytes", type=int,
                    default=stream.DEFAULT_BODY_BYTES,
                    help="encoded body size per frame (must match TX)")
    ap.add_argument("--seq-start", type=int, default=0,
                    help="starting seq the TX side used (must match)")
    ap.add_argument("--top-positions", type=int, default=20,
                    help="how many top-error byte positions to print")
    args = ap.parse_args(argv)

    src = Path(args.source).read_bytes()
    expected = _expected_bodies(src, args.mtu, args.body_bytes,
                                seq_start=args.seq_start)
    if not expected:
        sys.stderr.write("corruption: source is empty — no expected frames\n")
        return 2
    sys.stderr.write(
        f"corruption: {len(expected)} expected frame(s), body={args.body_bytes}B, "
        f"mtu={args.mtu}B, seq range [{min(expected)}..{max(expected)}]\n"
    )

    total_captured = 0
    total_corrupted = 0
    total_clean = 0
    matched_seq = 0
    unmatched_seq = 0
    bits_compared = 0
    bit_errors = 0
    byte_pos_errors = collections.Counter()
    byte_pos_examined = collections.Counter()
    per_frame_byte_errs: list[int] = []
    per_frame_bit_errs: list[int] = []
    # Per-frame phy metrics (parsed but only used when present).
    snr_clean: list[int] = []
    snr_corrupt: list[int] = []
    # (snr_bucket, corrupted_or_not) -> count; for the BER-vs-SNR table.
    bucket_frames: collections.Counter = collections.Counter()
    bucket_bit_errors: collections.Counter = collections.Counter()
    bucket_bits_compared: collections.Counter = collections.Counter()

    for line in sys.stdin:
        ev = parse_event(line, ev="rx.frame")
        if ev is None:
            continue
        total_captured += 1
        crc_err = int(ev.get("crc") or 0)
        icv_err = int(ev.get("icv") or 0)
        snr = tuple(ev["snr"]) if ev.get("snr") else None
        eff = _effective_snr(snr)
        if eff is not None:
            (snr_corrupt if crc_err or icv_err else snr_clean).append(eff)
        if crc_err or icv_err:
            total_corrupted += 1
        else:
            total_clean += 1
        body = bytes.fromhex(ev.get("body") or "")
        if len(body) < stream.HEADER_LEN:
            unmatched_seq += 1
            continue
        # Seq lives at bytes 2-3 of the envelope; magic at 0-1 may be wrong
        # if the descriptor is intact but the body got mangled, so we read
        # seq regardless and match against expected.
        seq = int.from_bytes(body[2:4], "little")
        if seq not in expected:
            unmatched_seq += 1
            continue
        matched_seq += 1
        exp = expected[seq]
        compare_len = min(len(body), len(exp))
        frame_byte_errs = 0
        frame_bit_errs = 0
        for i in range(compare_len):
            byte_pos_examined[i] += 1
            xor = body[i] ^ exp[i]
            if xor:
                frame_byte_errs += 1
                bits = bin(xor).count("1")
                frame_bit_errs += bits
                byte_pos_errors[i] += 1
        bits_compared += compare_len * 8
        bit_errors += frame_bit_errs
        per_frame_byte_errs.append(frame_byte_errs)
        per_frame_bit_errs.append(frame_bit_errs)
        bucket = _snr_bucket(snr)
        bucket_frames[bucket] += 1
        bucket_bit_errors[bucket] += frame_bit_errs
        bucket_bits_compared[bucket] += compare_len * 8

    if not matched_seq:
        sys.stderr.write(
            "corruption: no captured frames matched a known seq — check "
            "--source / --seq-start / --mtu / --body-bytes versus the TX side\n"
        )
        return 1

    ber = bit_errors / max(1, bits_compared)
    print(f"=== corruption analysis ({matched_seq} matched / "
          f"{total_captured} captured) ===")
    print(f"captured        : {total_captured}")
    print(f"  chip-clean    : {total_clean}")
    print(f"  chip-corrupt  : {total_corrupted}  (crc or icv set)")
    print(f"matched seq     : {matched_seq}")
    print(f"unmatched seq   : {unmatched_seq}  (likely lost, foreign, or "
          f"seq-bytes corrupted)")
    print(f"bits compared   : {bits_compared}")
    print(f"bit errors      : {bit_errors}")
    print(f"BER (compared)  : {ber:.3e}")

    if per_frame_byte_errs:
        clean_frames = sum(1 for e in per_frame_byte_errs if e == 0)
        print(f"per-frame errors:")
        print(f"  fully clean   : {clean_frames}/{matched_seq}")
        print(f"  byte errors   : "
              f"avg={sum(per_frame_byte_errs) / matched_seq:.2f}, "
              f"max={max(per_frame_byte_errs)}")
        print(f"  bit errors    : "
              f"avg={sum(per_frame_bit_errs) / matched_seq:.2f}, "
              f"max={max(per_frame_bit_errs)}")

    if byte_pos_errors:
        print(f"\nbyte-position error histogram "
              f"(top {args.top_positions} positions):")
        print(f"  pos   err/exam   pct")
        for pos, count in byte_pos_errors.most_common(args.top_positions):
            exam = byte_pos_examined[pos]
            pct = 100.0 * count / max(1, exam)
            print(f"  {pos:3d}   {count:5d}/{exam:5d}   {pct:5.1f}%")

    # Phy-metrics correlation. Two views: distribution of weakest-path SNR
    # for chip-clean vs chip-corrupt frames, and per-SNR-bucket BER.
    if snr_clean or snr_corrupt:
        def _stat(xs: list[int]) -> str:
            if not xs:
                return "n=0"
            xs = sorted(xs)
            n = len(xs)
            return (f"n={n} min={xs[0]} p25={xs[n // 4]} "
                    f"med={xs[n // 2]} p75={xs[(3 * n) // 4]} max={xs[-1]}")
        print(f"\nphy SNR (stronger path, dB):")
        print(f"  chip-clean    : {_stat(snr_clean)}")
        print(f"  chip-corrupt  : {_stat(snr_corrupt)}")

    if bucket_frames and any(b != "no-snr" for b in bucket_frames):
        print(f"\nBER by SNR bucket (stronger path, 5-dB buckets):")
        print(f"  bucket       frames   bits-cmp   bit-err    BER")
        for bucket in sorted(bucket_frames):
            n = bucket_frames[bucket]
            bits = bucket_bits_compared[bucket]
            errs = bucket_bit_errors[bucket]
            ber = errs / max(1, bits)
            print(f"  {bucket:>11s}   {n:6d}   {bits:8d}   {errs:6d}   "
                  f"{ber:.3e}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
