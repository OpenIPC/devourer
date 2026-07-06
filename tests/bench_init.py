#!/usr/bin/env python3
"""bench_init.py — init/startup-time benchmark: devourer vs kernel driver.

Measures, per plugged DUT, how long it takes from process start until the
driver can actually move frames:

  * devourer RX  — `rxdemo` until `init-timing: demo.first_rx_frame`
                   (exec → first 802.11 frame delivered).
  * devourer TX  — `txdemo` until `init-timing: txdemo.first_tx_submit`
                   (exec → first bulk-OUT submitted; includes txdemo's settle
                   sleep, deliberately — that's the user-visible latency).
  * kernel       — two flavours, same stage names:
                   Host mode (`--kernel-host`): insmod the vendor .ko built
                   in reference/ (88XXau for Jaguar1; rtl88x2bu / 8821cu /
                   rtl88x2cu / rtl88x2eu for Jaguar2/3) on the host itself →
                   netdev appears (probe) → monitor setup (ifup = hal init +
                   fwdl on these drivers) → first tcpdump frame. The mode to
                   use for startup timing.
                   VM mode (`--vm-name`/`--vm-ssh`, same rig as regress.py):
                   virsh attach → wlan iface appears (driver probe + fwdl) →
                   monitor-mode setup → first tcpdump frame. Pinned-kernel
                   driver-behaviour comparisons only — virtualized USB adds
                   latency to every stage, so don't cite its timings.

Per-stage breakdown comes from the `init-timing:` lines emitted by the
devourer library (src/InitTimer.h); this script only parses and aggregates.

A/B variants isolate extrinsic overheads (libusb log level, USB reset,
TX-power loop). Each (cell, variant) is repeated --reps times with a USB
port power-cycle in between so every rep is a cold init.

Usage:
  sudo python3 tests/bench_init.py                       # devourer cells only
  sudo python3 tests/bench_init.py --vm-name devourer-testrig \
      --vm-ssh user@VM-IP                                # + kernel cells (VM)
  sudo python3 tests/bench_init.py --kernel-host         # + kernel cells (host
                                                         #   reference/ .ko)
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

# vid:pid → (module name, .ko path relative to the devourer root) for the
# vendor kernel drivers kept (and built) under reference/. Used by
# --kernel-host; a DUT absent from this map just skips its kernel cell.
KMOD_FOR_VIDPID: dict[str, tuple[str, str]] = {
    "0bda:8812": ("88XXau_ohd", "reference/rtl8812au/88XXau_ohd.ko"),
    "0bda:8813": ("88XXau_ohd", "reference/rtl8812au/88XXau_ohd.ko"),
    "2357:0120": ("88XXau_ohd", "reference/rtl8812au/88XXau_ohd.ko"),
    "0bda:0811": ("88XXau_ohd", "reference/rtl8812au/88XXau_ohd.ko"),
    "0bda:a811": ("88XXau_ohd", "reference/rtl8812au/88XXau_ohd.ko"),
    "0bda:b811": ("88XXau_ohd", "reference/rtl8812au/88XXau_ohd.ko"),
    "2357:012d": ("88x2bu_ohd", "reference/rtl88x2bu/88x2bu_ohd.ko"),
    "0bda:b82c": ("88x2bu_ohd", "reference/rtl88x2bu/88x2bu_ohd.ko"),
    "0bda:c811": ("8821cu", "reference/8821cu/8821cu.ko"),
    "0bda:c812": ("88x2cu_ohd", "reference/rtl88x2cu/88x2cu_ohd.ko"),
    "0bda:c82c": ("88x2cu_ohd", "reference/rtl88x2cu/88x2cu_ohd.ko"),
    "0bda:a81a": ("rtl88x2eu_ohd", "reference/rtl88x2eu/rtl88x2eu_ohd.ko"),
    "0bda:e822": ("rtl88x2eu_ohd", "reference/rtl88x2eu/rtl88x2eu_ohd.ko"),
}


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
# Traffic source — the "first frame" markers (devourer demo + kernel tcpdump)
# need frames on the bench channel. In an RF-quiet spot ambient traffic is
# zero and every RX cell times out, so --traffic-from PID dedicates one
# plugged non-DUT adapter to a devourer beacon flood (~500 fps) for the whole
# run; every cell then measures RX-path readiness, not beacon-interval luck.
# ---------------------------------------------------------------------------


def start_traffic_source(
    devourer_root: Path,
    pid: str,
    channel: int,
    logdir: Path,
) -> subprocess.Popen:
    duts = [d for d in discover_duts() if d.pid == pid.lower().removeprefix("0x")]
    if not duts:
        sys.exit(f"--traffic-from {pid}: no such adapter plugged")
    src = duts[0]
    detach_from_host_kernel(src)
    env = dict(os.environ)
    env["DEVOURER_VID"] = f"0x{src.vid}"
    env["DEVOURER_PID"] = f"0x{src.pid}"
    env["DEVOURER_CHANNEL"] = str(channel)
    log_path = logdir / f"traffic-{src.pid}.log"
    fh = open(log_path, "w")
    proc = regress._register_local_proc(subprocess.Popen(
        [str(devourer_root / "build" / "txdemo")],
        env=env, stdout=fh, stderr=subprocess.STDOUT,
        preexec_fn=regress._child_preexec,
    ))
    deadline = time.monotonic() + 45.0
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            sys.exit(f"traffic source died at startup — see {log_path}")
        if "txdemo.first_tx_submit" in log_path.read_text(errors="replace"):
            print(f"[traffic] {src.chipset} beaconing on ch {channel}",
                  flush=True)
            return proc
        time.sleep(0.2)
    sys.exit(f"traffic source never reached first_tx_submit — see {log_path}")


# ---------------------------------------------------------------------------
# Kernel cells, host mode — insmod the vendor .ko built under reference/.
# Same stage names as the VM path so the report and README read the same.
# ---------------------------------------------------------------------------


def _iface_for_dut(dut: Dut) -> str | None:
    """Net iface whose USB interface path belongs to dut (any driver)."""
    for p in Path("/sys/class/net").iterdir():
        dev = os.path.realpath(p / "device")
        if f"/{dut.sysfs_id}:" in dev + "/":
            return p.name
    return None


def run_kernel_host_cell(
    devourer_root: Path,
    dut: Dut,
    channel: int,
    timeout_s: float,
    log_path: Path,
) -> dict[str, int] | None:
    """One host-kernel bring-up from the reference/ vendor module, timed as:

      kernel.probe         insmod → netdev registered (USB probe, efuse read)
      kernel.monitor_setup down/monitor/up + set channel (ifup runs hal init
                           + firmware download on these vendor drivers)
      kernel.first_frame   tcpdump start → first captured frame on channel
      kernel.total         insmod → first frame

    Cold per rep: rmmod + USB port power-cycle before the timed window, and
    whatever auto-bound on re-enumeration (in-tree rtw88) is detached — the
    same pre-state the devourer cells start from.
    """
    kmod, rel = KMOD_FOR_VIDPID[dut.vidpid]
    ko = devourer_root / rel
    if not ko.exists():
        sys.stderr.write(f"  {ko} not built — skipping kernel-host cell\n")
        return None

    log = open(log_path, "w")

    def _fail(msg: str) -> None:
        log.write(msg + "\n")
        sys.stderr.write(f"  {msg}\n")

    # Outside the timed window: dependency modules + cold chip/module state.
    subprocess.run(["modprobe", "cfg80211"], capture_output=True)
    subprocess.run(["rmmod", kmod], capture_output=True)
    usb_port_power_cycle(dut)
    detach_from_host_kernel(dut)

    try:
        t0 = time.monotonic()
        r = subprocess.run(["insmod", str(ko)], capture_output=True, text=True)
        if r.returncode != 0:
            _fail(f"insmod {ko.name}: {r.stderr.strip()}")
            return None
        iface = None
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            iface = _iface_for_dut(dut)
            if iface:
                break
            time.sleep(0.05)
        if not iface:
            _fail(f"no netdev appeared for {dut.vidpid} after insmod")
            return None
        t_probe = time.monotonic()

        for attempt in (1, 2):  # netdev can lag registration / udev-rename
            ok = True
            for cmd in (
                ["ip", "link", "set", iface, "down"],
                ["iw", "dev", iface, "set", "monitor", "none"],
                ["ip", "link", "set", iface, "up"],
                ["iw", "dev", iface, "set", "channel", str(channel)],
            ):
                c = subprocess.run(cmd, capture_output=True, text=True)
                if c.returncode != 0:
                    if attempt == 1:
                        # udev may have renamed the netdev right after we
                        # caught its registration name — re-resolve.
                        time.sleep(0.3)
                        iface = _iface_for_dut(dut) or iface
                    else:
                        _fail(f"{' '.join(cmd)}: {c.stderr.strip()}")
                    ok = False
                    break
            if ok:
                break
        if not ok:
            return None
        t_mon = time.monotonic()

        # Same standard as the other cells: any frame proves the RX path.
        proc = regress._register_local_proc(subprocess.Popen(
            ["tcpdump", "-i", iface, "-nn", "-l", "-c", "1"],
            stdout=log, stderr=subprocess.DEVNULL,
            preexec_fn=regress._child_preexec,
        ))
        t_frame = None
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                if proc.returncode == 0:
                    t_frame = time.monotonic()
                break
            time.sleep(0.05)
        regress._terminate(proc)
        regress._unregister_local_proc(proc)
        if t_frame is None:
            _fail("no frame captured before timeout")
            return None

        return {
            "kernel.probe": int((t_probe - t0) * 1000),
            "kernel.monitor_setup": int((t_mon - t_probe) * 1000),
            "kernel.first_frame": int((t_frame - t_mon) * 1000),
            "kernel.total": int((t_frame - t0) * 1000),
        }
    finally:
        log.close()
        subprocess.run(["rmmod", kmod], capture_output=True)


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
    ap.add_argument("--kernel-host", action="store_true",
                    help="kernel cells via the reference/ vendor .ko on the "
                         "host (Jaguar2/3 DUTs; see KMOD_FOR_VIDPID)")
    ap.add_argument("--traffic-from", default="",
                    help="PID of a plugged NON-DUT adapter to run a devourer "
                         "beacon flood on the bench channel for the whole "
                         "run (RF-quiet bench spots have no ambient frames)")
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

    traffic_proc = None
    if args.traffic_from:
        src_pid = args.traffic_from.lower().removeprefix("0x")
        if any(d.pid == src_pid for d in duts):
            sys.exit("--traffic-from adapter is among the benched DUTs; "
                     "restrict with --pids or pick another adapter")
        traffic_proc = start_traffic_source(devourer_root, src_pid,
                                            args.channel, logdir)

    results: dict[tuple, list] = {}
    for dut in duts:
        cells: list[tuple[str, str]] = []
        for v in variants:
            cells.append(("rx", v))
            if not args.skip_tx:
                cells.append(("tx", v))
        if kh:
            cells.append(("kernel", "-"))
        elif args.kernel_host:
            if dut.vidpid in KMOD_FOR_VIDPID:
                cells.append(("kernel", "-"))
            else:
                print(f"[{dut.chipset}] no reference/ module known — "
                      "skipping kernel cell", flush=True)

        for cell, variant in cells:
            key = (dut.chipset, cell, variant)
            results[key] = []
            for rep in range(args.reps):
                tag = f"{dut.pid}-{cell}-{variant.replace('+','_')}-{rep}"
                log = logdir / f"{tag}.log"
                print(f"[{dut.chipset}] {cell}/{variant} rep {rep + 1}/{args.reps} ...",
                      flush=True)
                if cell == "kernel":
                    r = (run_kernel_cell(kh, dut, args.channel, args.timeout, log)
                         if kh else
                         run_kernel_host_cell(devourer_root, dut, args.channel,
                                              args.timeout, log))
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

    if traffic_proc is not None:
        regress._terminate(traffic_proc)
        regress._unregister_local_proc(traffic_proc)

    report = emit_report(results, args.reps, args.channel)
    Path(args.out).write_text(report)
    print()
    print(report)
    print(f"\nlogs: {logdir}/   report: {args.out}")


if __name__ == "__main__":
    main()
