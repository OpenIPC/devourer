#!/usr/bin/env python3
"""Inject the canonical devourer TX-validation beacon on a kernel-driver
monitor interface.

The frame mirrors txdemo/main.cpp's hardcoded beacon: a probe request with
SA = 57:42:75:05:d6:00. WiFiDriverDemo and WiFiDriverTxDemo both grep for
this SA on RX (the `<devourer-tx-hit>` matcher) so the same beacon works
as the TX source whether the RX side is devourer or tcpdump.

Run from tests/regress.py's kernel-TX cell:

    sudo python3 tests/inject_beacon.py --iface wlpXX --count 500 --interval 0.002

Requires the iface to already be in monitor mode on the chosen channel
(regress.py sets that up). Idempotent: run multiple times safely.
"""

import argparse
import time

from scapy.all import RadioTap, Dot11, sendp

# Source MAC matches the canonical beacon SA in txdemo/main.cpp and the
# `<devourer-tx-hit>` matcher in demo/main.cpp. Don't change without
# updating both sides.
CANONICAL_SA = "57:42:75:05:d6:00"


def build_beacon():
    """Mgmt / probe-request frame matching txdemo's beacon_frame[]. The body
    payload doesn't matter for hit-count testing — only SA is matched."""
    return (
        RadioTap()
        / Dot11(
            type=0,  # mgmt
            subtype=4,  # probe request
            addr1="ff:ff:ff:ff:ff:ff",  # DA broadcast
            addr2=CANONICAL_SA,  # SA — matched by RX side
            addr3=CANONICAL_SA,  # BSSID
        )
        / b"\x00\x00\x00\x00\x00\x00\x00\x00"  # ssid IE (empty)
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", required=True, help="monitor-mode wlan iface")
    ap.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="seconds to inject (default 30)",
    )
    ap.add_argument(
        "--interval",
        type=float,
        default=0.002,
        help="inter-frame gap seconds (default 0.002 = 500 fps, matches txdemo)",
    )
    args = ap.parse_args()

    pkt = build_beacon()
    end = time.monotonic() + args.duration
    sent = 0
    while time.monotonic() < end:
        try:
            sendp(pkt, iface=args.iface, verbose=False)
            sent += 1
        except OSError as e:
            # iface went down mid-test — bail rather than spin.
            print(f"inject_beacon: sendp failed after {sent} frames: {e}")
            break
        time.sleep(args.interval)
    print(f"inject_beacon: sent {sent} frames on {args.iface}")


if __name__ == "__main__":
    main()
