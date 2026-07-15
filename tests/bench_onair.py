#!/usr/bin/env python3
"""On-air TX throughput per chip per band (devourer), via USRP duty-cycle.

For each plugged Jaguar chip and each band, floods with txdemo at a
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
    "RTL8811CU": ("9-1.3", "0x0bda", "0xc811"),     # Jaguar2 (8821c); COMFAST CF-811AC
}
BANDS = [("2.4 GHz (ch6)", 6, 2437e6), ("UNII-1 (ch36)", 36, 5180e6),
         ("UNII-2/3 (ch149)", 149, 5745e6)]
KDRIVERS = ["rtw88_8812au", "rtw88_8814au", "rtw88_8821au", "rtw88_8822cu",
            "rtw88_8822bu", "8812eu", "rtl88x2eu", "rtl88xxau_wfb"]
DUTY_RE = re.compile(r"duty=([\d.]+)%\s+noise=([-\d.]+)dB.*on_air~=([\d.]+)Mbps")


def resolve_sysfs(vid: str, pid: str, fallback: str) -> str | None:
    """Find the current sysfs id for a VID:PID (topology-independent), so the
    hardcoded CHIPS ports don't have to match this rig. Returns the fallback if
    the scan finds nothing (and None if even the fallback is absent)."""
    vlo, plo = vid.lower().replace("0x", ""), pid.lower().replace("0x", "")
    root = Path("/sys/bus/usb/devices")
    for d in root.glob("*"):
        try:
            if (d / "idVendor").read_text().strip() == vlo and \
               (d / "idProduct").read_text().strip() == plo:
                return d.name
        except OSError:
            continue
    return fallback if (root / fallback).exists() else None


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


def devourer_flood(vid, pid, ch, mcs, bw, size, ampdu=False, threads=1):
    """Flood txdemo. Default = single-frame injection (the baseline table
    metric). ampdu=True adds the A-MPDU recipe (QoS-Data on a data queue +
    SetAmpduMode + a deep multi-thread feed) so the MAC actually aggregates;
    threads>1 alone is the feed-depth control (no aggregation)."""
    env = dict(__import__("os").environ,
               DEVOURER_VID=vid, DEVOURER_PID=pid, DEVOURER_CHANNEL=str(ch),
               DEVOURER_TX_RATE=f"MCS{mcs}/{bw}",
               DEVOURER_TX_PAYLOAD_BYTES=str(size), DEVOURER_TX_GAP_US="0")
    if threads > 1:
        env["DEVOURER_TX_THREADS"] = str(threads)
    if ampdu:
        env.update(DEVOURER_TX_QOS_DATA="1", DEVOURER_TX_QOS_NOACK="1",
                   DEVOURER_TX_AMPDU_MODE="0/16",
                   DEVOURER_TX_BATCH="3", DEVOURER_TX_USB_AGG="3")
    return regress._register_local_proc(subprocess.Popen(
        [str(ROOT / "build" / "txdemo")], env=env,
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
    ap.add_argument("--ampdu", action="store_true",
                    help="also measure the deep-fed A-MPDU flood + a feed-depth "
                    "control, emitting a singles/feed/A-MPDU comparison")
    ap.add_argument("--threads", type=int, default=4,
                    help="parallel-sender depth for the A-MPDU + feed modes")
    ap.add_argument("--bands", help="comma-separated band substrings to bench "
                    "(default: all; e.g. --bands 149 for the clean 5 GHz band)")
    args = ap.parse_args()
    regress._install_cleanup_handlers()
    # Resolve each chip's actual sysfs port from its VID:PID so a stale
    # hardcoded CHIPS port doesn't drop a plugged-in adapter.
    present = {}
    for c, (sysfs, vid, pid) in CHIPS.items():
        s = resolve_sysfs(vid, pid, sysfs)
        if s:
            present[c] = (s, vid, pid)
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
        if rc.returncode == 2:
            # rail_check's ground arbitration: the control airs strongly (the
            # ground hears it) but the B210 reads low — the SDR itself is in
            # its degraded-read mode, so EVERY duty number this session is
            # unreliable. Abort rather than emit a garbage table.
            print("\n" + "!" * 70)
            print("!! SDR DEGRADED — the B210's readings can't be trusted this session")
            print("!! (the ground hears the control at strong RSSI while the SDR reads")
            print("!! low). Reset/reseat the B210 or power-cycle its port, then re-run.")
            print("!" * 70 + "\n")
            return 2
        if rc.returncode != 0:
            print("\n" + "!" * 70)
            print("!! RAIL SAG DETECTED — 5 GHz duty numbers below are NOT trustworthy.")
            print("!! Power-cycle the hub / use a powered hub or root port, then re-run.")
            print("!! (2.4 GHz numbers are unaffected. Pass --skip-rail-check to override.)")
            print("!" * 70 + "\n")

    bands = BANDS
    if args.bands:
        want = [b.strip() for b in args.bands.split(",")]
        bands = [b for b in BANDS if any(w in b[0] for w in want)]

    # Modes to measure per chip/band. Default: just the single-frame baseline
    # (the historical table metric). --ampdu adds the feed-depth control and the
    # full A-MPDU recipe, so any gain is attributable (feed depth vs aggregation).
    modes = [("singles", dict())]
    if args.ampdu:
        modes = [("singles", dict()),
                 ("feed", dict(threads=args.threads)),
                 ("ampdu", dict(ampdu=True, threads=args.threads))]

    def flood(vid, pid, ch, opts):
        return devourer_flood(vid, pid, ch, args.mcs, args.bw, args.size, **opts)

    results: dict = {}
    for label, ch, freq in bands:
        # Calibrate this band's idle noise floor (all chips quiet) so the
        # threshold is right — a fixed floor mis-reads a noisier 2.4 GHz band.
        for sysfs, _, _ in CHIPS.values():
            free_chip(sysfs)
        cal = sdr_duty(freq, args.mcs, args.bw, None, 2.0, return_noise=True)
        floor = cal[1] if cal else args.noise_db  # cal = (duty, noise, mbps)
        print(f"  [{label}] idle noise floor {floor:.1f} dB", flush=True)
        for chip, (sysfs, vid, pid) in present.items():
            for mode, opts in modes:
                print(f"  {chip} {label} {mode} …", flush=True)
                d = None
                for _ in range(2):  # one retry
                    free_chip(sysfs)
                    proc = flood(vid, pid, ch, opts)
                    time.sleep(6)
                    d = sdr_duty(freq, args.mcs, args.bw, floor, args.secs)
                    regress._terminate(proc)
                    if d and d[0] > 5:
                        break
                results[(chip, label, mode)] = d
                print(f"     -> {d[1]:.1f} Mbps ({d[0]:.0f}% duty)" if d
                      else "     -> FAIL")

    def cell(chip, label, mode):
        d = results.get((chip, label, mode))
        return f"{d[1]:.0f}" if d and d[1] > 0.5 else "—"

    if not args.ampdu:
        # markdown — the historical single-frame table
        print("\n| Part | " + " | ".join(l for l, _, _ in bands) + " |")
        print("|------|" + "|".join("------" for _ in bands) + "|")
        for chip in present:
            cells = [cell(chip, l, "singles") + " Mbps" if cell(chip, l, "singles") != "—"
                     else "—" for l, _, _ in bands]
            print(f"| {chip} | " + " | ".join(cells) + " |")
        return 0

    # A-MPDU comparison: per band, singles / feed / A-MPDU on-air Mbps (SDR
    # duty x PHY rate). NB this metric is channel occupancy x modulation rate;
    # for a chip already near the PHY ceiling it can't rise even as A-MPDU
    # raises payload goodput (see docs/aggregation.md).
    for label, _, _ in bands:
        print(f"\n### {label}  (on-air Mbps = SDR duty x PHY rate)")
        print("| Part | singles | +deep feed | +A-MPDU |")
        print("|------|--------:|-----------:|--------:|")
        for chip in present:
            s, f, a = (cell(chip, label, m) for m in ("singles", "feed", "ampdu"))
            print(f"| {chip} | {s} | {f} | {a} |")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
