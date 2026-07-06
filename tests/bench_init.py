#!/usr/bin/env python3
"""bench_init.py — init/startup-time benchmark: devourer vs kernel driver.

Measures, per plugged DUT, how long it takes from process start until the
driver can actually move frames:

  * devourer RX  — `rxdemo` until `init-timing: demo.first_rx_frame`
                   (exec → first 802.11 frame delivered).
  * devourer TX  — `txdemo` until `init-timing: txdemo.first_tx_submit`
                   (exec → first bulk-OUT submitted; includes txdemo's settle
                   sleep, deliberately — that's the user-visible latency).
  * kernel       — VM mode only (`--vm-name`/`--vm-ssh`, same rig as
                   regress.py): virsh attach → wlan iface appears (driver
                   probe + fwdl) → monitor-mode setup → first tcpdump frame.

Per-stage breakdown comes from the `init-timing:` lines emitted by the
devourer library (src/InitTimer.h); this script only parses and aggregates.

A/B variants isolate extrinsic overheads (libusb log level, USB reset,
TX-power loop). Each (cell, variant) is repeated --reps times with a USB
port power-cycle in between so every rep is a cold init.

Usage:
  sudo python3 tests/bench_init.py                       # devourer cells only
  sudo python3 tests/bench_init.py --vm-name devourer-testrig \
      --vm-ssh user@VM-IP                                # + kernel cells
  sudo python3 tests/bench_init.py --variants debug,quiet,quiet+skipreset
"""

from __future__ import annotations

import argparse
import os
import re
import statistics
import subprocess
import sys
import time
from pathlib import Path

# Reuse regress.py's rig plumbing: DUT discovery, VM routing, monitor setup,
# process-group hygiene.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import regress  # noqa: E402
from regress import (  # noqa: E402
    CANONICAL_SA,
    Dut,
    KernelHost,
    detach_from_host_kernel,
    discover_duts,
    usb_port_power_cycle,
)

INIT_TIMING_RE = re.compile(r"init-timing: ([\w.]+) = (\d+) ms")

# Variant name → env deltas applied on top of the demo defaults.
# "quiet" (libusb WARNING) is the demo default; "debug" opts into libusb
# DEBUG logging via DEVOURER_USB_DEBUG=1.
VARIANTS: dict[str, dict[str, str]] = {
    "debug": {"DEVOURER_USB_DEBUG": "1"},
    "quiet": {},
    "quiet+skipreset": {"DEVOURER_SKIP_RESET": "1"},
    "quiet+skiptxpwr": {"DEVOURER_SKIP_TXPWR": "1"},
}
DEFAULT_VARIANTS = ["debug", "quiet"]


# ---------------------------------------------------------------------------
# Devourer cells.
# ---------------------------------------------------------------------------


def _devourer_env(dut: Dut, channel: int, extra: dict[str, str]) -> dict:
    env = dict(os.environ)
    env["DEVOURER_VID"] = f"0x{dut.vid}"
    env["DEVOURER_PID"] = f"0x{dut.pid}"
    env["DEVOURER_CHANNEL"] = str(channel)
    env.pop("DEVOURER_USB_QUIET", None)  # variant decides
    env.pop("DEVOURER_USB_DEBUG", None)
    env.update(extra)
    return env


def run_devourer_cell(
    devourer_root: Path,
    dut: Dut,
    channel: int,
    variant: str,
    mode: str,  # "rx" | "tx"
    timeout_s: float,
    log_path: Path,
) -> dict[str, int] | None:
    """One cold-init run. Returns {stage: ms} or None on timeout/failure.

    The end-marker stage is `demo.first_rx_frame` (rx) /
    `txdemo.first_tx_submit` (tx); the process is killed as soon as the
    marker appears, or at timeout.
    """
    binary = "rxdemo" if mode == "rx" else "txdemo"
    end_marker = "demo.first_rx_frame" if mode == "rx" else "txdemo.first_tx_submit"

    detach_from_host_kernel(dut)
    usb_port_power_cycle(dut)

    fh = open(log_path, "w")
    proc = regress._register_local_proc(subprocess.Popen(
        [str(devourer_root / "build" / binary)],
        env=_devourer_env(dut, channel, VARIANTS[variant]),
        stdout=fh, stderr=subprocess.DEVNULL,
        preexec_fn=regress._child_preexec,
    ))
    try:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                break  # crashed / exited early
            if end_marker in log_path.read_text(errors="replace"):
                break
            time.sleep(0.2)
    finally:
        regress._terminate(proc)
        regress._unregister_local_proc(proc)
        fh.close()

    stages: dict[str, int] = {}
    for m in INIT_TIMING_RE.finditer(log_path.read_text(errors="replace")):
        # First occurrence wins: the second channel_set (from mlme init)
        # re-emits channel_set.* — keep init-order stages stable. The
        # end-marker only ever appears once.
        stages.setdefault(m.group(1), int(m.group(2)))
    if end_marker not in stages:
        return None
    return stages


# ---------------------------------------------------------------------------
# Kernel cells (VM mode only — aircrack-ng 88XXau doesn't build on modern
# host kernels, and 8814AU kernel-side work is VM-only on this rig).
# ---------------------------------------------------------------------------


def run_kernel_cell(
    kh: KernelHost,
    dut: Dut,
    channel: int,
    timeout_s: float,
    log_path: Path,
) -> dict[str, int] | None:
    """One kernel-driver bring-up, timed in three stages:

      kernel.probe         virsh attach → wlan iface appears (enumeration +
                           driver probe incl. fwdl)
      kernel.monitor_setup ip link down/monitor/up + iw set channel
      kernel.first_frame   tcpdump start → first captured frame on channel
      kernel.total         attach → first frame
    """
    # Cold start: make sure the DUT is host-side and power-cycled, then move
    # it into the VM.
    kh.release_dut(dut)
    time.sleep(1.0)
    usb_port_power_cycle(dut)

    t0 = time.monotonic()
    kh.take_dut(dut)
    try:
        iface = kh.wait_for_wlan_iface(dut, timeout=timeout_s)
    except RuntimeError as e:
        sys.stderr.write(f"  kernel probe failed: {e}\n")
        kh.release_dut(dut)
        return None
    t_probe = time.monotonic()

    try:
        kh.iface_to_monitor(iface, channel)
    except RuntimeError as e:
        sys.stderr.write(f"  kernel monitor setup failed: {e}\n")
        kh.release_dut(dut)
        return None
    t_mon = time.monotonic()

    # First frame on air = RX actually works. No SA filter — any traffic
    # proves the RX path (same standard the devourer cell is held to:
    # demo.first_rx_frame counts any frame).
    fh = open(log_path, "w")
    proc = kh.popen(
        ["sh", "-c", f"tcpdump -i {iface} -nn -l -c 1 2>/dev/null"],
        stdout=fh, stderr=subprocess.DEVNULL,
    )
    t_frame = None
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            t_frame = time.monotonic()
            break
        time.sleep(0.1)
    regress._terminate(proc)
    fh.close()
    kh.release_dut(dut)
    if t_frame is None:
        return None

    return {
        "kernel.probe": int((t_probe - t0) * 1000),
        "kernel.monitor_setup": int((t_mon - t_probe) * 1000),
        "kernel.first_frame": int((t_frame - t_mon) * 1000),
        "kernel.total": int((t_frame - t0) * 1000),
    }


# ---------------------------------------------------------------------------
# Aggregation + report.
# ---------------------------------------------------------------------------


def median_stages(runs: list[dict[str, int]]) -> dict[str, int]:
    out: dict[str, int] = {}
    for k in {k for r in runs for k in r}:
        vals = [r[k] for r in runs if k in r]
        if vals:
            out[k] = int(statistics.median(vals))
    return out


def emit_report(results: dict, reps: int, channel: int) -> str:
    lines = [
        f"# Init-time benchmark (median of {reps}, channel {channel})",
        "",
        "| DUT | cell | variant | ready (ms) | runs ok |",
        "|---|---|---|---|---|",
    ]
    for (chipset, cell, variant), runs in sorted(results.items()):
        ok = [r for r in runs if r is not None]
        med = median_stages(ok) if ok else {}
        key = {
            "rx": "demo.first_rx_frame",
            "tx": "txdemo.first_tx_submit",
            "kernel": "kernel.total",
        }[cell]
        ready = med.get(key, "—")
        lines.append(
            f"| {chipset} | {cell} | {variant} | {ready} | {len(ok)}/{len(runs)} |"
        )
    lines.append("")
    # Per-stage detail for each (DUT, cell, variant) that has data.
    for (chipset, cell, variant), runs in sorted(results.items()):
        ok = [r for r in runs if r is not None]
        if not ok:
            continue
        med = median_stages(ok)
        lines.append(f"## {chipset} / {cell} / {variant} — stage medians (ms)")
        lines.append("")
        for k in sorted(med, key=med.get, reverse=True):
            lines.append(f"- `{k}` = {med[k]}")
        lines.append("")
    return "\n".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--channel", type=int, default=6)
    ap.add_argument("--timeout", type=float, default=45.0,
                    help="per-run ready-marker timeout (s)")
    ap.add_argument("--variants", default=",".join(DEFAULT_VARIANTS),
                    help=f"comma list from {sorted(VARIANTS)} or 'all'")
    ap.add_argument("--pids", default="",
                    help="restrict to comma list of PIDs (e.g. 8812,8813)")
    ap.add_argument("--vm-name", default="", help="libvirt domain for kernel cells")
    ap.add_argument("--vm-ssh", default="", help="user@ip of the VM")
    ap.add_argument("--skip-tx", action="store_true",
                    help="skip devourer TX cells")
    ap.add_argument("--out", default="/tmp/devourer-bench-init.md")
    args = ap.parse_args()

    devourer_root = Path(__file__).resolve().parent.parent
    variants = (sorted(VARIANTS) if args.variants == "all"
                else [v.strip() for v in args.variants.split(",") if v.strip()])
    for v in variants:
        if v not in VARIANTS:
            ap.error(f"unknown variant {v!r}; pick from {sorted(VARIANTS)}")

    kh = (KernelHost.via_ssh(args.vm_ssh, args.vm_name)
          if args.vm_name and args.vm_ssh else None)

    duts = discover_duts()
    if args.pids:
        wanted = {p.strip().lower().removeprefix("0x")
                  for p in args.pids.split(",")}
        duts = [d for d in duts if d.pid in wanted]
    if not duts:
        sys.exit("no supported DUTs found (or all filtered by --pids)")

    regress._install_cleanup_handlers()
    logdir = Path("/tmp/devourer-bench-init-logs")
    logdir.mkdir(exist_ok=True)

    results: dict[tuple, list] = {}
    for dut in duts:
        cells: list[tuple[str, str]] = []
        for v in variants:
            cells.append(("rx", v))
            if not args.skip_tx:
                cells.append(("tx", v))
        if kh:
            cells.append(("kernel", "-"))

        for cell, variant in cells:
            key = (dut.chipset, cell, variant)
            results[key] = []
            for rep in range(args.reps):
                tag = f"{dut.pid}-{cell}-{variant.replace('+','_')}-{rep}"
                log = logdir / f"{tag}.log"
                print(f"[{dut.chipset}] {cell}/{variant} rep {rep + 1}/{args.reps} ...",
                      flush=True)
                if cell == "kernel":
                    r = run_kernel_cell(kh, dut, args.channel, args.timeout, log)
                else:
                    r = run_devourer_cell(devourer_root, dut, args.channel,
                                          variant, cell, args.timeout, log)
                ready = None
                if r:
                    ready = r.get("demo.first_rx_frame") or \
                        r.get("txdemo.first_tx_submit") or r.get("kernel.total")
                print(f"    -> {'%d ms' % ready if ready else 'FAILED/timeout'}",
                      flush=True)
                results[key].append(r)

    report = emit_report(results, args.reps, args.channel)
    Path(args.out).write_text(report)
    print()
    print(report)
    print(f"\nlogs: {logdir}/   report: {args.out}")


if __name__ == "__main__":
    main()
