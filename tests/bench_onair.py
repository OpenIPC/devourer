#!/usr/bin/env python3
"""On-air TX throughput per chip per band (devourer), via USRP duty-cycle.

For each plugged Jaguar chip and each band, floods with WiFiDriverTxDemo at a
fixed HT MCS/BW and measures channel occupancy with sdr_duty.py (ceiling-free).
Emits a markdown table for the README. Optionally also measures wfb-ng (svpcom
driver + a raw-AF_PACKET blaster) for the parity comparison.

  sudo python3 tests/bench_onair.py                  # devourer, all chips/bands
  sudo python3 tests/bench_onair.py --wfb            # also wfb-ng on 5 GHz
"""
from __future__ import annotations
import argparse, re, subprocess, sys, time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import regress  # noqa

# chipset -> (sysfs_id, vid, pid). sysfs_id is bench-specific (the USB port the
# adapter is plugged into) — adjust to your topology.
CHIPS = {
    "RTL8812AU": ("9-2", "0x0bda", "0x8812"),
    "RTL8814AU": ("4-2.3.2", "0x0bda", "0x8813"),
    "RTL8821AU": ("9-1.4", "0x2357", "0x0120"),
    "RTL8812CU": ("3-2.4", "0x0bda", "0xc812"),   # Jaguar3 (rtl8822c)
    "RTL8812EU": ("3-2.3.3", "0x0bda", "0xa81a"),   # Jaguar3 EU (rtl8822e); adjust sysfs
    "RTL8822BU": ("4-2.3.3", "0x2357", "0x012d"),   # Jaguar2 (8822b); TP-Link Archer T3U
}
BANDS = [("2.4 GHz (ch6)", 6, 2437e6), ("UNII-1 (ch36)", 36, 5180e6),
         ("UNII-2/3 (ch149)", 149, 5745e6)]
KDRIVERS = ["rtw88_8812au", "rtw88_8814au", "rtw88_8821au", "rtw88_8822cu",
            "rtw88_8822bu", "8812eu", "rtl88x2eu", "rtl88xxau_wfb"]
DUTY_RE = re.compile(r"duty=([\d.]+)%\s+noise=([-\d.]+)dB.*on_air~=([\d.]+)Mbps")


def free_chip(sysfs: str) -> None:
    for drv in KDRIVERS:
        base = f"/sys/bus/usb/drivers/{drv}"
        try:
            for d in __import__("os").listdir(base):
                if d[0].isdigit():
                    subprocess.run(["tee", f"{base}/unbind"], input=d.encode(),
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            pass
    time.sleep(1)


def sdr_duty(freq: float, mcs: int, bw: int, noise_db: float | None,
             secs: float, return_noise: bool = False):
    rate = "50e6" if bw == 40 else "25e6"
    cmd = ["python3", str(HERE / "sdr_duty.py"), "--freq", f"{freq:.0f}",
           "--rate", rate, "--secs", str(secs), "--mcs", str(mcs), "--bw", str(bw)]
    if noise_db is not None:
        cmd += ["--noise-db", str(noise_db)]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=90)
    m = DUTY_RE.search(r.stdout + r.stderr)
    if not m:
        return None
    duty, noise, mbps = float(m.group(1)), float(m.group(2)), float(m.group(3))
    return (duty, noise, mbps) if return_noise else (duty, mbps)


def devourer_flood(vid, pid, ch, mcs, bw, size):
    env = dict(__import__("os").environ,
               DEVOURER_VID=vid, DEVOURER_PID=pid, DEVOURER_CHANNEL=str(ch),
               DEVOURER_TX_RATE=f"MCS{mcs}/{bw}",
               DEVOURER_TX_PAYLOAD_BYTES=str(size), DEVOURER_TX_GAP_US="0")
    return regress._register_local_proc(subprocess.Popen(
        [str(ROOT / "build" / "WiFiDriverTxDemo")], env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=regress._child_preexec))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mcs", type=int, default=7)
    ap.add_argument("--bw", type=int, default=20)
    ap.add_argument("--size", type=int, default=1500)
    ap.add_argument("--secs", type=float, default=4.0)
    ap.add_argument("--noise-db", type=float, default=-62.0)
    ap.add_argument("--skip-rail-check", action="store_true",
                    help="skip the CU-control 5 GHz rail-sag pre-flight (rail_check.sh)")
    ap.add_argument("--only", help="comma-separated chip labels to bench "
                    "(default: every plugged-in chip in CHIPS)")
    args = ap.parse_args()
    regress._install_cleanup_handlers()
    present = {c: v for c, v in CHIPS.items()
               if Path(f"/sys/bus/usb/devices/{v[0]}").exists()}
    if args.only:
        want = {s.strip().upper() for s in args.only.split(",")}
        present = {c: v for c, v in present.items() if c.upper() in want}
    print(f"# chips: {', '.join(present)}  | MCS{args.mcs}/{args.bw}MHz/{args.size}B\n")

    # Pre-flight rail-sag guard (CLAUDE.md defence #1): the bus-powered hub rail
    # can brown out the 5 GHz PA, collapsing on-air power while 2.4 GHz still
    # works — making every 5 GHz number untrustworthy. Check the known-good
    # control adapter first so a sagging rail is flagged loudly instead of
    # mis-read as a per-chip 5 GHz deficit.
    if not args.skip_rail_check:
        rc = subprocess.run(["bash", str(HERE / "rail_check.sh")])
        if rc.returncode != 0:
            print("\n" + "!" * 70)
            print("!! RAIL SAG DETECTED — 5 GHz duty numbers below are NOT trustworthy.")
            print("!! Power-cycle the hub / use a powered hub or root port, then re-run.")
            print("!! (2.4 GHz numbers are unaffected. Pass --skip-rail-check to override.)")
            print("!" * 70 + "\n")

    results: dict = {}
    for label, ch, freq in BANDS:
        # Calibrate this band's idle noise floor (all chips quiet) so the
        # threshold is right — a fixed floor mis-reads a noisier 2.4 GHz band.
        for sysfs, _, _ in CHIPS.values():
            free_chip(sysfs)
        cal = sdr_duty(freq, args.mcs, args.bw, None, 2.0, return_noise=True)
        floor = cal[1] if cal else args.noise_db  # cal = (duty, noise, mbps)
        print(f"  [{label}] idle noise floor {floor:.1f} dB", flush=True)
        for chip, (sysfs, vid, pid) in present.items():
            print(f"  {chip} {label} …", flush=True)
            d = None
            for _ in range(2):  # one retry
                free_chip(sysfs)
                proc = devourer_flood(vid, pid, ch, args.mcs, args.bw, args.size)
                time.sleep(6)
                d = sdr_duty(freq, args.mcs, args.bw, floor, args.secs)
                regress._terminate(proc)
                if d and d[0] > 5:
                    break
            results[(chip, label)] = d
            print(f"     -> {d[1]:.1f} Mbps ({d[0]:.0f}% duty)" if d else "     -> FAIL")

    # markdown
    print("\n| Part | " + " | ".join(l for l, _, _ in BANDS) + " |")
    print("|------|" + "|".join("------" for _ in BANDS) + "|")
    for chip in present:
        cells = []
        for label, _, _ in BANDS:
            d = results.get((chip, label))
            cells.append(f"{d[1]:.0f} Mbps" if d and d[1] > 0.5 else "—")
        print(f"| {chip} | " + " | ".join(cells) + " |")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
