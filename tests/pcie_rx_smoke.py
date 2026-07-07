#!/usr/bin/env python3
"""pcie_rx_smoke.py — ambient-beacon RX validation for the PCIe transport
(RTL8821CE via vfio-pci). Single-adapter: no 2x2 matrix is possible on the
PCIe rig, so the pass signal is ambient traffic — management beacons decoded
CRC-clean on both bands.

    sudo python3 tests/pcie_rx_smoke.py [--bdf 0000:01:00.0] \
        [--rxdemo build/rxdemo] [--secs 15] [--channels 6,36]

Per channel: bind to vfio-pci (tests/pcie_vfio_bind.sh), run rxdemo with
DEVOURER_PCIE_BDF + DEVOURER_RX_DUMP_ALL, parse the rx.corrupt event stream
(fc = 802.11 frame-control word), and require >= MIN_BEACONS frames with
fc 0x0080 (beacon) and a zero chip-CRC-fail count among them. Exit 0 = all
channels pass. The adapter is left bound to vfio-pci (use
tests/pcie_vfio_bind.sh --restore to hand it back to rtw88).
"""

import argparse
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MIN_BEACONS = 5  # per channel; ambient APs beacon at ~10/s each


def run_channel(rxdemo, bdf, channel, secs):
    env = dict(os.environ)
    env.update({
        "DEVOURER_PCIE_BDF": bdf,
        "DEVOURER_CHANNEL": str(channel),
        "DEVOURER_RX_DUMP_ALL": "1",
        "DEVOURER_LOG_LEVEL": "warn",
    })
    proc = subprocess.run(
        ["timeout", str(secs + 30), rxdemo],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        timeout=secs + 60,
    )
    # rxdemo runs until killed; wrap with `timeout` for the dwell.
    total = beacons = beacon_crc_fail = 0
    for line in proc.stdout.decode(errors="replace").splitlines():
        if not line.startswith('{"ev":"rx.corrupt"'):
            continue
        try:
            ev = json.loads(line)
        except json.JSONDecodeError:
            continue
        total += 1
        if ev.get("fc") == "0x0080":
            beacons += 1
            if ev.get("crc"):
                beacon_crc_fail += 1
    return total, beacons, beacon_crc_fail


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bdf", default="0000:01:00.0")
    ap.add_argument("--rxdemo", default=os.path.join(HERE, "..", "build", "rxdemo"))
    ap.add_argument("--secs", type=int, default=15)
    ap.add_argument("--channels", default="6,36")
    args = ap.parse_args()

    if os.geteuid() != 0:
        print("ERROR: run as root (vfio + register access)", file=sys.stderr)
        return 2

    bind = subprocess.run(
        ["bash", os.path.join(HERE, "pcie_vfio_bind.sh"), args.bdf])
    if bind.returncode != 0:
        print("FAIL: vfio bind", file=sys.stderr)
        return 1

    ok = True
    for ch in [int(c) for c in args.channels.split(",")]:
        # `timeout` sends SIGTERM; rxdemo installs handlers and exits cleanly.
        total, beacons, bcrc = run_channel(args.rxdemo, args.bdf, ch, args.secs)
        verdict = "PASS" if (beacons >= MIN_BEACONS and bcrc == 0) else "FAIL"
        print(f"ch{ch}: frames={total} beacons={beacons} "
              f"beacon_crc_fail={bcrc} -> {verdict}")
        if verdict == "FAIL":
            ok = False

    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
