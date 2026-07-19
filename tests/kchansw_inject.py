#!/usr/bin/env python3
"""High-rate tagged-frame injector for the kernel channel-switch bench.

Feeds the on-air oracle of tests/kchansw_bench.py: a continuous stream of
canonical-SA probe requests on a kernel-driver monitor interface, each frame
carrying a payload trailer

    'KCSW' magic | u32 run-nonce | u32 rolling counter | u64 send mono_ns

so the devourer RX oracle can attribute every decoded frame to a send instant
and a position in the stream from the rx.frame `body` hex alone — robust
against the kernel rewriting the 802.11 sequence-control field. The 802.11 SC
is still set from the counter as the cheap secondary check, and the fixed
`--size` PSDU length is the tertiary fingerprint.

Per-frame JSONL on stdout (one object per line, first field `ev`):

    {"ev":"inj.tx","n":123,"mono_ns":456789,"errno":0}

A nonzero errno is itself queue-behavior data: mac80211 stops TX queues
around a channel switch and AF_PACKET send() surfaces that as an error or a
block, depending on ring state.

Unlike tests/inject_beacon.py (scapy sendp per frame), the frame here is
prebuilt once and per-frame fields are patched into a bytearray — a blocking
AF_PACKET send() then paces at multi-kHz without scapy overhead.
"""

import argparse
import json
import os
import socket
import struct
import sys
import time

CANONICAL_SA = "57:42:75:05:d6:00"
MAGIC = b"KCSW"
# Trailer layout right after the 24-byte 802.11 mgmt header.
TRAILER_FMT = "<4sIIQ"
TRAILER_LEN = struct.calcsize(TRAILER_FMT)  # 20

# Radiotap: header(8) + Rate u8 @8 + pad @9 + TX-flags u16 @10 = 12 bytes.
# present = Rate (bit 2) | TX_FLAGS (bit 15); TX flags 0x0008 = NOACK.
_RT_PRESENT = (1 << 2) | (1 << 15)


def build_frame(sa: str, rate_500k: int, psdu_len: int, nonce: int) -> tuple[bytearray, int]:
    """Prebuild radiotap + 802.11 PSDU. Returns (buf, trailer_offset)."""
    rt = struct.pack("<BBHI", 0, 0, 12, _RT_PRESENT) + bytes([rate_500k, 0]) + struct.pack("<H", 0x0008)
    sa_b = bytes(int(x, 16) for x in sa.split(":"))
    hdr = (
        bytes([0x40, 0x00])          # FC: mgmt / probe request
        + bytes([0x00, 0x00])        # duration
        + b"\xff" * 6                # DA broadcast
        + sa_b                       # SA — the oracle's stream-gate key
        + sa_b                       # BSSID
        + bytes([0x00, 0x00])        # SC (patched per frame)
    )
    trailer = struct.pack(TRAILER_FMT, MAGIC, nonce & 0xFFFFFFFF, 0, 0)
    body_pad = max(0, psdu_len - len(hdr) - len(trailer))
    buf = bytearray(rt + hdr + trailer + b"\x00" * body_pad)
    return buf, len(rt) + len(hdr)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--iface", required=True, help="monitor-mode wlan iface")
    ap.add_argument("--hz", type=float, default=2000.0,
                    help="target frame rate (default 2000 → 0.5 ms dead-time "
                         "resolution at the oracle)")
    ap.add_argument("--size", type=int, default=96,
                    help="802.11 PSDU length in bytes — the oracle's length "
                         "fingerprint (default 96)")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="seconds to run; 0 = until SIGTERM/SIGINT")
    ap.add_argument("--rate", type=int, default=12,
                    help="radiotap legacy rate in 500 kbps units "
                         "(default 12 = 6M OFDM — decodable on both bands)")
    ap.add_argument("--sa", default=CANONICAL_SA,
                    help=f"source address (default canonical {CANONICAL_SA})")
    ap.add_argument("--nonce", type=lambda v: int(v, 0), default=os.getpid(),
                    help="u32 run nonce embedded per frame (default: pid)")
    args = ap.parse_args()

    buf, t_off = build_frame(args.sa, args.rate, args.size, args.nonce)
    sc_off = t_off - 2  # SC is the last header field before the trailer

    s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW)
    s.bind((args.iface, 0))

    out = sys.stdout
    period_ns = int(1e9 / args.hz) if args.hz > 0 else 0
    end_ns = time.monotonic_ns() + int(args.duration * 1e9) if args.duration > 0 else None
    n = 0
    next_ns = time.monotonic_ns()
    try:
        while True:
            now = time.monotonic_ns()
            if end_ns is not None and now >= end_ns:
                break
            if period_ns:
                if now < next_ns:
                    time.sleep((next_ns - now) / 1e9)
                    now = time.monotonic_ns()
                # Deadline pacing without drift; if we fell behind more than
                # 20 periods (a stopped TX queue), resync rather than burst.
                next_ns += period_ns
                if now - next_ns > 20 * period_ns:
                    next_ns = now + period_ns
            n += 1
            struct.pack_into("<H", buf, sc_off, (n & 0xFFF) << 4)
            struct.pack_into(TRAILER_FMT, buf, t_off, MAGIC, args.nonce & 0xFFFFFFFF,
                             n & 0xFFFFFFFF, now & 0xFFFFFFFFFFFFFFFF)
            err = 0
            try:
                s.send(buf)
            except OSError as e:
                err = e.errno or -1
            out.write(json.dumps({"ev": "inj.tx", "n": n, "mono_ns": now,
                                  "errno": err}, separators=(",", ":")) + "\n")
            if n % 64 == 0:
                out.flush()
    except KeyboardInterrupt:
        pass
    finally:
        out.flush()
        s.close()
    sys.stderr.write(f"kchansw_inject: sent {n} frames on {args.iface}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
