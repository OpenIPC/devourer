#!/usr/bin/env python3
"""Cross-driver regression matrix for devourer.

Runs a 4-cell test on a host with two compatible USB Wi-Fi adapters, comparing
this project's userspace stack ("devourer") against the kernel driver
(aircrack-ng / mainline rtw88) for both TX and RX:

                      TX = devourer        TX = kernel
    RX = devourer     [end-to-end dvr]     [does dvr RX kernel-TX frame?]
    RX = kernel       [does dvr emit       [baseline / rig sanity]
                       valid frames?]

Each cell injects/receives a known-SA beacon (57:42:75:05:d6:00, matching
the canonical frame in txdemo/main.cpp and the RX matcher in demo/main.cpp).
A cell passes if the RX side observes >= --pass-threshold hits within the
test duration. The baseline cell is run first; if it fails, the rig itself
is broken (interference, channel, antennas) and the matrix is aborted.

Designed to be run manually after building devourer:

    cd /path/to/devourer && cmake --build build -j
    sudo python3 tests/regress.py --channel 100

Supports any modern Linux distro: tool paths are resolved via `which`, wlan
interfaces are discovered via `iw dev`, and the kernel driver claiming each
DUT is read from sysfs (no hardcoded module names). NetworkManager users:
either stop NM for the duration, or `nmcli device set <iface> managed no`
on the test interfaces before running.

VM-readiness: the kernel-cell shell-out goes through `run_kernel_cmd()`,
which today executes locally. To migrate the kernel-driver side into a
pinned-kernel VM (recommended once host kernel upgrades start breaking the
aircrack-ng/rtl8812au driver), point KERNEL_CELL_RUNNER at an `ssh` wrapper
and arrange USB passthrough to the VM via libvirt's USB hot-plug.
"""

from __future__ import annotations

import argparse
import dataclasses
import glob
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Optional

# Source MAC of the canonical beacon — must match txdemo/main.cpp and the
# `<devourer-tx-hit>` matcher in demo/main.cpp. tcpdump filter and scapy
# injector both use this.
CANONICAL_SA = "57:42:75:05:d6:00"

# Map every supported PID to the chipset family used for log readability.
# Detection of the kernel driver claiming the device is dynamic (via sysfs);
# this table is informational only.
SUPPORTED_DUTS = {
    "0bda:8812": "RTL8812AU",
    "0bda:8813": "RTL8814AU",
    "2357:0120": "RTL8821AU (TP-Link T2U Plus)",
    "0bda:0811": "RTL8811AU",
    "0bda:a811": "RTL8811AU",
    "0bda:b811": "RTL8811AU",
}

# Required external tools. Each entry: (binary, distro-agnostic install hint).
REQUIRED_TOOLS = [
    ("iw", "your distro's `iw` package"),
    ("tcpdump", "your distro's `tcpdump` package"),
    ("ip", "iproute2 (almost always preinstalled)"),
    ("modprobe", "kmod (almost always preinstalled)"),
]
# Python module deps (checked separately; scapy is the heavy one).
REQUIRED_PY_MODS = [
    ("scapy", "pip install scapy  # or your distro's `python3-scapy`"),
]


# ---------------------------------------------------------------------------
# Subprocess helpers — wrap shell-outs with structured output.
# ---------------------------------------------------------------------------


def run(cmd: list[str], **kw) -> subprocess.CompletedProcess:
    """Run a command synchronously, capturing output. Raises on non-zero
    exit unless check=False is passed in **kw."""
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def run_kernel_cmd(cmd: list[str], **kw) -> subprocess.CompletedProcess:
    """Kernel-cell shell-out. Today: same as `run` (local exec). When
    migrating the kernel-driver side into a VM, wrap this with ssh / virsh
    exec — every kernel-side call goes through here."""
    return run(cmd, **kw)


# ---------------------------------------------------------------------------
# Preflight — check prerequisites with friendly errors.
# ---------------------------------------------------------------------------


def preflight(devourer_root: Path) -> None:
    """Bail early if anything required is missing. Better to fail with one
    actionable message than to crash 4 cells in with a cryptic Python
    traceback."""
    missing = []
    for tool, hint in REQUIRED_TOOLS:
        if shutil.which(tool) is None:
            missing.append(f"  - command `{tool}` not on PATH (install: {hint})")
    for mod, hint in REQUIRED_PY_MODS:
        try:
            __import__(mod)
        except ImportError:
            missing.append(f"  - python module `{mod}` not importable (install: {hint})")
    for binary in ("WiFiDriverDemo", "WiFiDriverTxDemo"):
        if not (devourer_root / "build" / binary).is_file():
            missing.append(
                f"  - devourer binary `build/{binary}` missing — run "
                f"`cmake -S {devourer_root} -B {devourer_root}/build && "
                f"cmake --build {devourer_root}/build -j`"
            )
    if os.geteuid() != 0:
        missing.append(
            "  - this script needs root (modprobe / iw / tcpdump / sysfs writes). "
            "Re-run with `sudo`."
        )
    if missing:
        sys.stderr.write("Prerequisites not met:\n" + "\n".join(missing) + "\n")
        sys.exit(2)


# ---------------------------------------------------------------------------
# DUT discovery — find plugged-in adapters via sysfs.
# ---------------------------------------------------------------------------


@dataclasses.dataclass
class Dut:
    sysfs_id: str  # e.g. "1-14" (the USB device path component)
    vid: str  # e.g. "0bda"
    pid: str  # e.g. "8812"
    chipset: str  # human-readable

    @property
    def vidpid(self) -> str:
        return f"{self.vid}:{self.pid}"

    @property
    def iface_id(self) -> str:
        """Interface address sysfs expects for bind/unbind."""
        return f"{self.sysfs_id}:1.0"


def discover_duts() -> list[Dut]:
    duts: list[Dut] = []
    for d in glob.glob("/sys/bus/usb/devices/*"):
        try:
            with open(f"{d}/idVendor") as f:
                vid = f.read().strip()
            with open(f"{d}/idProduct") as f:
                pid = f.read().strip()
        except (FileNotFoundError, IsADirectoryError, PermissionError):
            continue
        key = f"{vid}:{pid}"
        if key in SUPPORTED_DUTS:
            duts.append(
                Dut(
                    sysfs_id=os.path.basename(d),
                    vid=vid,
                    pid=pid,
                    chipset=SUPPORTED_DUTS[key],
                )
            )
    return duts


def kernel_driver_for_dut(dut: Dut) -> Optional[str]:
    """Return the kernel driver name currently bound to dut, or None if
    nothing is bound."""
    link = f"/sys/bus/usb/devices/{dut.iface_id}/driver"
    if not os.path.islink(link):
        return None
    return os.path.basename(os.readlink(link))


def wlan_iface_for_dut(dut: Dut) -> Optional[str]:
    """If the kernel driver bound to dut has surfaced a net interface, return
    its name (e.g. 'wlp0s20f0u14' or 'wlan0'). Otherwise None."""
    net_dir = f"/sys/bus/usb/devices/{dut.iface_id}/net"
    if not os.path.isdir(net_dir):
        return None
    ifaces = os.listdir(net_dir)
    return ifaces[0] if ifaces else None


# ---------------------------------------------------------------------------
# Driver-state orchestration — bind / unbind a DUT to its kernel driver.
# ---------------------------------------------------------------------------


def detach_from_kernel(dut: Dut) -> None:
    """Unbind dut from whatever kernel driver claims it, so devourer can
    claim_interface(). If nothing claims it, no-op."""
    drv = kernel_driver_for_dut(dut)
    if drv is None:
        return
    try:
        with open(f"/sys/bus/usb/drivers/{drv}/unbind", "w") as f:
            f.write(dut.iface_id)
    except OSError as e:
        sys.stderr.write(f"detach_from_kernel({dut.iface_id}): {e}\n")


def attach_to_kernel(dut: Dut) -> None:
    """Re-bind dut to its kernel driver. Caller is responsible for ensuring
    the right module is modprobe'd first."""
    # The kernel picks the right driver automatically based on the device's
    # modalias when we write to .../drivers_probe.
    try:
        with open("/sys/bus/usb/drivers_probe", "w") as f:
            f.write(dut.iface_id)
    except OSError as e:
        sys.stderr.write(f"attach_to_kernel({dut.iface_id}): {e}\n")


def wait_for_wlan_iface(dut: Dut, timeout: float = 15.0) -> str:
    """Block until the kernel driver surfaces a wlan iface for dut.

    15s default accommodates the typical fwdl + association timeline for
    rtw88 / rtl8812au drivers; the iface usually appears in 2-4s but the
    out-of-tree aircrack-ng driver can take up to 10s on first probe."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        iface = wlan_iface_for_dut(dut)
        if iface is not None:
            return iface
        time.sleep(0.25)
    raise RuntimeError(
        f"no wlan iface appeared for {dut.vidpid} after {timeout}s — kernel "
        f"driver may have failed to bind (check `dmesg | tail -20` for "
        f"firmware-download or probe errors)"
    )


def kernel_iface_to_monitor(iface: str, channel: int) -> None:
    """Put a kernel wlan iface into monitor mode on the given channel.
    Idempotent — safe to call repeatedly. Surfaces command stderr on failure
    so 'permission denied' / 'No such device' / 'Operation not supported'
    error paths are actionable."""
    def _run(cmd):
        r = run_kernel_cmd(cmd)
        if r.returncode != 0:
            raise RuntimeError(
                f"{' '.join(cmd)} exit={r.returncode}: "
                f"{(r.stderr or r.stdout).strip() or '(no stderr)'}"
            )
    _run(["ip", "link", "set", iface, "down"])
    _run(["iw", "dev", iface, "set", "type", "monitor"])
    _run(["ip", "link", "set", iface, "up"])
    # set channel requires the iface to be up.
    _run(["iw", "dev", iface, "set", "channel", str(channel)])


# ---------------------------------------------------------------------------
# Cell implementations — each returns (hits, total_attempts).
# ---------------------------------------------------------------------------


@dataclasses.dataclass
class CellResult:
    hits: int  # frames matching CANONICAL_SA observed by the RX side
    tx_attempts: int  # frames the TX side reports having submitted
    tx_failures: int  # devourer-only: # of "Failed to send packet" errors
    duration_s: float
    notes: str = ""

    def passed(self, threshold: int) -> bool:
        return self.hits >= threshold

    def fmt(self, threshold: int) -> str:
        mark = "✓" if self.passed(threshold) else "✗"
        attempts_str = f"{self.tx_attempts} TX"
        if self.tx_failures > 0:
            attempts_str += f" ({self.tx_failures} fail)"
        return f"{self.hits} hits / {attempts_str} / {self.duration_s:.0f}s {mark}"


def _spawn_devourer_rx(
    devourer_root: Path, dut: Dut, channel: int, log_path: Path
) -> subprocess.Popen:
    env = os.environ.copy()
    env["DEVOURER_VID"] = f"0x{dut.vid}"
    env["DEVOURER_PID"] = f"0x{dut.pid}"
    env["DEVOURER_CHANNEL"] = str(channel)
    env["DEVOURER_USB_QUIET"] = "1"
    fh = open(log_path, "w")
    return subprocess.Popen(
        [str(devourer_root / "build" / "WiFiDriverDemo")],
        env=env,
        stdout=fh,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _spawn_devourer_tx(
    devourer_root: Path, dut: Dut, channel: int, log_path: Path
) -> subprocess.Popen:
    env = os.environ.copy()
    env["DEVOURER_VID"] = f"0x{dut.vid}"
    env["DEVOURER_PID"] = f"0x{dut.pid}"
    env["DEVOURER_CHANNEL"] = str(channel)
    env["DEVOURER_USB_QUIET"] = "1"
    fh = open(log_path, "w")
    return subprocess.Popen(
        [str(devourer_root / "build" / "WiFiDriverTxDemo")],
        env=env,
        stdout=fh,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _spawn_kernel_rx(
    iface: str, channel: int, log_path: Path
) -> subprocess.Popen:
    kernel_iface_to_monitor(iface, channel)
    # tcpdump -e shows the link-layer header (so we see SA); filter by SA.
    fh = open(log_path, "w")
    return subprocess.Popen(
        [
            "tcpdump",
            "-i",
            iface,
            "-e",
            "-nn",
            "-l",
            "ether",
            "src",
            CANONICAL_SA,
        ],
        stdout=fh,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def _spawn_kernel_tx(
    devourer_root: Path, iface: str, channel: int, duration: float, log_path: Path
) -> subprocess.Popen:
    kernel_iface_to_monitor(iface, channel)
    injector = devourer_root / "tests" / "inject_beacon.py"
    fh = open(log_path, "w")
    return subprocess.Popen(
        [
            sys.executable,
            str(injector),
            "--iface",
            iface,
            "--duration",
            str(duration),
        ],
        stdout=fh,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _terminate(proc: subprocess.Popen, grace: float = 2.0) -> None:
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=grace)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait()


def _count_devourer_rx_hits(log_path: Path) -> int:
    """devourer RX hits show up as `<devourer-tx-hit>...hits=N` lines, where
    N is monotonically increasing per match. Take the max N seen."""
    last = 0
    try:
        for line in log_path.read_text().splitlines():
            if "<devourer-tx-hit>" in line:
                # line format: "<devourer-tx-hit>txdemo SA match: hits=N total_rx=M len=L"
                for tok in line.split():
                    if tok.startswith("hits="):
                        try:
                            last = max(last, int(tok.split("=", 1)[1]))
                        except ValueError:
                            pass
    except FileNotFoundError:
        pass
    return last


def _count_devourer_tx_attempts(log_path: Path) -> tuple[int, int]:
    """Returns (max_tx_count, send_failures).

    WiFiDriverTxDemo rate-limits its `<devourer-tx>TX #N rc=X` prints
    (first 10 + every 500th N). The max N seen estimates total attempts —
    BUT when send_packet fails, the inner loop runs slower than expected
    (libusb error path takes longer than success path) and N may never
    reach the next 500 boundary, so the max N from prints undercounts
    badly. Surface the failure count alongside so the user can see what's
    going on.
    """
    last = 0
    failures = 0
    try:
        for line in log_path.read_text().splitlines():
            if line.startswith("<devourer-tx>TX #"):
                tok = line.split("#", 1)[1].split()[0]
                try:
                    last = max(last, int(tok))
                except ValueError:
                    pass
            elif "Failed to send packet" in line:
                failures += 1
    except FileNotFoundError:
        pass
    return last, failures


def _count_tcpdump_hits(log_path: Path) -> int:
    """tcpdump -e emits one line per packet. The filter narrows to SA matches,
    so line count == hit count."""
    try:
        return sum(1 for _ in log_path.read_text().splitlines())
    except FileNotFoundError:
        return 0


def _count_kernel_tx_sent(log_path: Path) -> int:
    """inject_beacon.py emits `inject_beacon: sent N frames on IFACE` at exit."""
    try:
        for line in log_path.read_text().splitlines():
            if line.startswith("inject_beacon: sent "):
                return int(line.split()[2])
    except (FileNotFoundError, ValueError):
        pass
    return 0


def run_cell(
    devourer_root: Path,
    tx_dut: Dut,
    rx_dut: Dut,
    tx_side: str,  # "devourer" | "kernel"
    rx_side: str,  # "devourer" | "kernel"
    channel: int,
    duration: float,
    tmpdir: Path,
) -> CellResult:
    """Run one matrix cell end-to-end.

    State contract: this function is responsible for the full bind/unbind
    dance of its DUTs. On entry, it first re-attaches both DUTs to the
    kernel (idempotent — works whether the previous cell left them bound
    or not), so a crashed previous cell doesn't poison the next one's
    starting state. On exit (always — try/finally), it re-attaches anything
    it detached.
    """
    cell_id = f"tx-{tx_side}_rx-{rx_side}"
    tx_log = tmpdir / f"{cell_id}.tx.log"
    rx_log = tmpdir / f"{cell_id}.rx.log"

    # Restore baseline: re-attach both DUTs to whatever kernel driver they
    # match. Idempotent. The wait after attach gives the kernel time to
    # finish its probe path before we either use it or detach it again.
    attach_to_kernel(tx_dut)
    attach_to_kernel(rx_dut)
    time.sleep(1.5)

    # Then detach the ones that this cell needs devourer to own.
    if tx_side == "devourer":
        detach_from_kernel(tx_dut)
    if rx_side == "devourer":
        detach_from_kernel(rx_dut)

    rx_proc: Optional[subprocess.Popen] = None
    tx_proc: Optional[subprocess.Popen] = None
    try:
        # RX side starts first so it's listening when TX begins.
        if rx_side == "devourer":
            rx_proc = _spawn_devourer_rx(devourer_root, rx_dut, channel, rx_log)
            # devourer needs ~5s for fwdl + channel set before it's actually RXing.
            time.sleep(6.0)
        else:
            rx_iface = wait_for_wlan_iface(rx_dut)
            rx_proc = _spawn_kernel_rx(rx_iface, channel, rx_log)
            time.sleep(1.0)

        # TX side.
        if tx_side == "devourer":
            tx_proc = _spawn_devourer_tx(devourer_root, tx_dut, channel, tx_log)
            # txdemo also has a 5s warmup (sleep(5) before the TX loop).
            tx_warmup = 6.0
        else:
            tx_iface = wait_for_wlan_iface(tx_dut)
            tx_proc = _spawn_kernel_tx(
                devourer_root, tx_iface, channel, duration, tx_log
            )
            tx_warmup = 0.5

        time.sleep(tx_warmup)
        measure_start = time.monotonic()
        time.sleep(duration)
        measure_end = time.monotonic()

        # Parse counts (do this BEFORE we terminate processes so they get a
        # chance to flush their stdout buffers — terminate sends SIGINT which
        # devourer traps and exits cleanly, flushing in the process).
        _terminate(tx_proc)
        # Let any in-flight frames drain before we close the RX side.
        time.sleep(1.0)
        _terminate(rx_proc)

        if rx_side == "devourer":
            hits = _count_devourer_rx_hits(rx_log)
        else:
            hits = _count_tcpdump_hits(rx_log)
        if tx_side == "devourer":
            tx_attempts, tx_failures = _count_devourer_tx_attempts(tx_log)
        else:
            tx_attempts = _count_kernel_tx_sent(tx_log)
            tx_failures = 0

        return CellResult(
            hits=hits,
            tx_attempts=tx_attempts,
            tx_failures=tx_failures,
            duration_s=measure_end - measure_start,
        )
    finally:
        # Always clean up subprocesses + restore kernel binding, even if the
        # cell raised. This prevents one cell's failure from poisoning the
        # next cell's starting state.
        if tx_proc is not None and tx_proc.poll() is None:
            _terminate(tx_proc)
        if rx_proc is not None and rx_proc.poll() is None:
            _terminate(rx_proc)
        if tx_side == "devourer":
            attach_to_kernel(tx_dut)
        if rx_side == "devourer":
            attach_to_kernel(rx_dut)


# ---------------------------------------------------------------------------
# Matrix driver — runs the 4 cells in baseline-first order.
# ---------------------------------------------------------------------------


def run_matrix(
    devourer_root: Path,
    tx_dut: Dut,
    rx_dut: Dut,
    channel: int,
    duration: float,
    threshold: int,
    tmpdir: Path,
    abort_on_baseline_fail: bool = True,
) -> dict[tuple[str, str], CellResult]:
    cells = [
        # Baseline first — if the rig itself is broken, the other cells'
        # results are uninterpretable. Default is to abort the matrix when
        # this fails; override with --no-baseline-abort for partial-rig
        # diagnostics (e.g. one chipset has no working kernel driver but
        # devourer-only cells are still worth running).
        ("kernel", "kernel"),
        ("devourer", "kernel"),
        ("kernel", "devourer"),
        ("devourer", "devourer"),
    ]
    results: dict[tuple[str, str], CellResult] = {}
    for tx_side, rx_side in cells:
        cell_label = f"TX={tx_side:>8s}  RX={rx_side:>8s}"
        print(f"[{time.strftime('%H:%M:%S')}] running cell {cell_label} ...",
              flush=True)
        try:
            r = run_cell(
                devourer_root,
                tx_dut,
                rx_dut,
                tx_side,
                rx_side,
                channel,
                duration,
                tmpdir,
            )
        except Exception as e:
            print(f"  ✗ cell crashed: {e}", flush=True)
            r = CellResult(hits=0, tx_attempts=0, tx_failures=0,
                           duration_s=0.0, notes=str(e))
        results[(tx_side, rx_side)] = r
        print(f"  → {r.fmt(threshold)}", flush=True)
        if (
            (tx_side, rx_side) == ("kernel", "kernel")
            and not r.passed(threshold)
            and abort_on_baseline_fail
        ):
            print(
                "BASELINE cell failed — the rig itself isn't moving frames. "
                "Aborting remaining cells (channel busy? antennas? "
                "wrong kernel driver?). Re-run with --no-baseline-abort "
                "to attempt the remaining cells anyway.",
                file=sys.stderr,
                flush=True,
            )
            for remaining in cells[1:]:
                results[remaining] = CellResult(
                    hits=0,
                    tx_attempts=0,
                    tx_failures=0,
                    duration_s=0.0,
                    notes="skipped (baseline failed)",
                )
            break
    return results


def emit_markdown(
    tx_dut: Dut,
    rx_dut: Dut,
    channel: int,
    duration: float,
    threshold: int,
    results: dict[tuple[str, str], CellResult],
) -> str:
    out = []
    out.append(f"## Regression matrix — channel {channel}, "
               f"{time.strftime('%Y-%m-%d %H:%M:%S')}\n")
    out.append(f"- TX adapter: `{tx_dut.vidpid}` ({tx_dut.chipset})")
    out.append(f"- RX adapter: `{rx_dut.vidpid}` ({rx_dut.chipset})")
    out.append(f"- Cell duration: {duration:.0f}s")
    out.append(f"- Pass threshold: ≥ {threshold} hits\n")
    out.append("|   | TX = devourer | TX = kernel |")
    out.append("|---|---|---|")
    for rx_side in ("devourer", "kernel"):
        cells = []
        for tx_side in ("devourer", "kernel"):
            r = results.get((tx_side, rx_side))
            if r is None:
                cells.append("—")
            else:
                cells.append(r.fmt(threshold))
        out.append(f"| RX = {rx_side} | {cells[0]} | {cells[1]} |")
    return "\n".join(out) + "\n"


# ---------------------------------------------------------------------------
# CLI.
# ---------------------------------------------------------------------------


def main():
    ap = argparse.ArgumentParser(
        description="Cross-driver regression matrix for devourer.",
    )
    ap.add_argument(
        "--devourer-root",
        type=Path,
        default=Path(__file__).resolve().parent.parent,
        help="repo root with build/WiFiDriverDemo + build/WiFiDriverTxDemo "
             "(default: parent of this script)",
    )
    ap.add_argument(
        "--channel",
        type=int,
        default=36,
        help="Wi-Fi channel to test on (default 36 = 5GHz quiet, pick a busy "
             "5GHz channel like 100 if your AP is there)",
    )
    ap.add_argument(
        "--duration",
        type=float,
        default=15.0,
        help="seconds each cell injects/receives (default 15)",
    )
    ap.add_argument(
        "--pass-threshold",
        type=int,
        default=1,
        help="min hits for a cell to pass (default 1 — generous because air "
             "interference + short windows make absolute counts unreliable; "
             "bump to 5-10 for higher-confidence runs on a quiet channel)",
    )
    ap.add_argument(
        "--tx-pid",
        help="USB PID hex of TX adapter (default: first auto-detected DUT)",
    )
    ap.add_argument(
        "--rx-pid",
        help="USB PID hex of RX adapter (default: second auto-detected DUT)",
    )
    ap.add_argument(
        "--keep-logs",
        action="store_true",
        help="don't delete the per-cell log files after the run",
    )
    ap.add_argument(
        "--no-baseline-abort",
        action="store_true",
        help="run all 4 cells even if kernel-kernel baseline fails (useful "
             "when one chipset has no working kernel driver but devourer-only "
             "cells are still worth checking)",
    )
    args = ap.parse_args()

    preflight(args.devourer_root)

    duts = discover_duts()
    if len(duts) < 2:
        sys.stderr.write(
            f"Need at least 2 supported DUTs plugged in, found {len(duts)}:\n"
        )
        for d in duts:
            sys.stderr.write(f"  - {d.vidpid} ({d.chipset}) at {d.sysfs_id}\n")
        sys.stderr.write(
            "Plug another compatible adapter or extend SUPPORTED_DUTS table.\n"
        )
        sys.exit(2)

    def pick(pid_arg, default_idx):
        if pid_arg is None:
            return duts[default_idx]
        for d in duts:
            if d.pid == pid_arg.lower().removeprefix("0x"):
                return d
        sys.stderr.write(f"No plugged DUT has PID {pid_arg}\n")
        sys.exit(2)

    tx_dut = pick(args.tx_pid, 0)
    rx_dut = pick(args.rx_pid, 1)
    if tx_dut.sysfs_id == rx_dut.sysfs_id:
        sys.stderr.write("TX and RX must be different physical devices.\n")
        sys.exit(2)

    print(f"TX: {tx_dut.vidpid} ({tx_dut.chipset}) at {tx_dut.sysfs_id}")
    print(f"RX: {rx_dut.vidpid} ({rx_dut.chipset}) at {rx_dut.sysfs_id}")
    print(f"Channel: {args.channel}   Duration/cell: {args.duration}s   "
          f"Pass threshold: ≥{args.pass_threshold} hits\n")

    with tempfile.TemporaryDirectory(prefix="devourer-regress-") as td:
        tmpdir = Path(td)
        results = run_matrix(
            devourer_root=args.devourer_root,
            tx_dut=tx_dut,
            rx_dut=rx_dut,
            channel=args.channel,
            duration=args.duration,
            threshold=args.pass_threshold,
            tmpdir=tmpdir,
            abort_on_baseline_fail=not args.no_baseline_abort,
        )
        print()
        md = emit_markdown(
            tx_dut, rx_dut, args.channel, args.duration,
            args.pass_threshold, results,
        )
        print(md)
        if args.keep_logs:
            kept = Path(tempfile.gettempdir()) / "devourer-regress-last"
            if kept.is_symlink() or kept.exists():
                kept.unlink()
            kept.symlink_to(tmpdir)
            print(f"(logs kept at {kept} — symlink, valid until next run)")
            # Detach from cleanup by exiting before TemporaryDirectory wipes it.
            # NOTE: this is sticky; the symlink will dangle next run.
            os._exit(0)


if __name__ == "__main__":
    main()
