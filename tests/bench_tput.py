#!/usr/bin/env python3
"""Per-chip TX/RX throughput + latency benchmark for devourer vs the kernel
driver, across bands and frame sizes. Fills the README "Hardware landscape"
numbers.

TX rate is measured from **usbmon bulk-OUT completions at the source chip** —
the true frames-accepted rate, sniffer-independent (counting at a sniffer
measures the *sniffer's* RX ceiling, a known trap). RX is measured by flooding
from a peer chip on the same channel and counting frames at the DUT; it is
capped by the flooder's own TX rate (also measured), so RX Mbps is a
received-fraction, not link capacity.

  sudo tests/bench_tput.py --quick          # ~1 cell smoke
  sudo tests/bench_tput.py                   # full matrix (resumable)

Reuses tests/regress.py for DUT discovery, kernel bind/unbind, USB power-cycle,
process hygiene and log parsers. Kernel cells run on the HOST kernel for chips
the host rtw88 binds (8812/8814 → usbmon metric); chips it doesn't (8821au) fall
back to --vm-ssh (injector self-count metric) or are annotated "—".
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import os
import re
import statistics
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import regress  # noqa: E402

CANONICAL_SA = regress.CANONICAL_SA
BANDS = {"2g": 6, "unii1": 36, "unii2_3": 149}
SIZES = [1500, 3994]
USBMON_NODE = "/sys/kernel/debug/usb/usbmon/{bus}u"
TXDESC = 40  # on-wire bulk-OUT = PSDU + TXDESC


# --------------------------------------------------------------------------- #
# usbmon plumbing
# --------------------------------------------------------------------------- #
def ensure_usbmon() -> None:
    subprocess.run(["modprobe", "usbmon"], check=False)
    if not os.path.isdir("/sys/kernel/debug/usb/usbmon"):
        subprocess.run(["mount", "-t", "debugfs", "none", "/sys/kernel/debug"],
                       check=False)
    if not os.path.isdir("/sys/kernel/debug/usb/usbmon"):
        sys.exit("usbmon node missing — need root + CONFIG_USB_MON")


def bus_dev(dut: regress.Dut) -> tuple[int, int]:
    """Read live busnum/devnum — they change after every re-enumerate."""
    base = f"/sys/bus/usb/devices/{dut.sysfs_id}"
    with open(f"{base}/busnum") as f:
        bus = int(f.read())
    with open(f"{base}/devnum") as f:
        dev = int(f.read())
    return bus, dev


class UsbmonReader:
    """Stream a usbmon 'u' text node in a thread, tagging each line with a host
    monotonic timestamp so we can window the capture precisely."""

    def __init__(self, bus: int, log_path: Path | None = None):
        self.bus = bus
        self.log_path = log_path
        self.lines: list[tuple[float, str]] = []
        self._stop = threading.Event()
        self._thr = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thr.start()

    def _run(self) -> None:
        try:
            f = open(USBMON_NODE.format(bus=self.bus), "r", errors="replace")
        except OSError:
            return
        out = open(self.log_path, "w") if self.log_path else None
        try:
            for line in f:
                if self._stop.is_set():
                    break
                self.lines.append((time.monotonic(), line))
                if out:
                    out.write(line)
        except OSError:
            pass
        finally:
            f.close()
            if out:
                out.close()

    def stop(self) -> None:
        self._stop.set()

    def parse_tx(self, dev: int, size: int, t0: float, t1: float) -> dict:
        """Bulk-OUT metrics within [t0, t1]. Byte-sum drives Mbps (robust to
        mixed URB sizes); full-frame completions drive fps + latency."""
        prefix = f"Bo:{self.bus}:{dev:03d}:"
        submit_ts: dict[str, float] = {}
        lat_us: list[float] = []
        bytes_sum = 0
        frames = 0
        inflight = 0
        max_inflight = 0
        errors = 0
        full = size + TXDESC
        for mono, line in self.lines:
            parts = line.split()
            if len(parts) < 4 or prefix not in parts[3]:
                continue
            ev = parts[2]
            if ev == "S":
                inflight += 1
                max_inflight = max(max_inflight, inflight)
                submit_ts[parts[0]] = mono
            elif ev == "C":
                inflight = max(0, inflight - 1)
                if not (t0 <= mono <= t1):
                    submit_ts.pop(parts[0], None)
                    continue
                try:
                    ln = int(parts[5])
                except (IndexError, ValueError):
                    ln = 0
                if ln > 0:
                    bytes_sum += ln
                if ln >= full * 0.9:
                    frames += 1
                    s = submit_ts.pop(parts[0], None)
                    if s is not None:
                        lat_us.append((mono - s) * 1e6)
            elif ev == "E":
                errors += 1
        window = max(t1 - t0, 1e-6)
        return {
            "mbps": bytes_sum * 8 / window / 1e6,
            "fps": frames / window,
            "p50_lat_us": statistics.median(lat_us) if lat_us else 0.0,
            "p99_lat_us": (sorted(lat_us)[int(len(lat_us) * 0.99)]
                           if len(lat_us) > 5 else 0.0),
            "max_inflight": max_inflight,
            "frames": frames,
            "errors": errors,
        }


# --------------------------------------------------------------------------- #
# device routing helpers
# --------------------------------------------------------------------------- #
def for_devourer(dut: regress.Dut) -> None:
    """Make sure no kernel driver holds the device."""
    drv = regress.host_kernel_driver_for_dut(dut)
    if drv:
        regress.detach_from_host_kernel(dut)
        time.sleep(1)


def host_iface(dut: regress.Dut) -> str | None:
    base = f"/sys/bus/usb/devices/{dut.iface_id}/net"
    try:
        return os.listdir(base)[0]
    except (OSError, IndexError):
        return None


def _devourer_tx_env(dut: regress.Dut, channel: int, size: int, mcs: int,
                     gap_us: int = 0) -> dict:
    env = os.environ.copy()
    env["DEVOURER_VID"] = f"0x{dut.vid}"
    env["DEVOURER_PID"] = f"0x{dut.pid}"
    env["DEVOURER_CHANNEL"] = str(channel)
    env["DEVOURER_TX_HT_MCS"] = "1"
    env["DEVOURER_TX_MCS"] = str(mcs)
    env["DEVOURER_TX_PAYLOAD_BYTES"] = str(size)
    env["DEVOURER_TX_GAP_US"] = str(gap_us)
    return env


def _spawn(binary: str, env: dict, log: Path, stdin=None) -> subprocess.Popen:
    fh = open(log, "w")
    devroot = HERE.parent
    return regress._register_local_proc(subprocess.Popen(
        [str(devroot / "build" / binary)], env=env,
        stdout=fh, stderr=subprocess.STDOUT, stdin=stdin,
        preexec_fn=regress._child_preexec))


# --------------------------------------------------------------------------- #
# measurement primitives
# --------------------------------------------------------------------------- #
def _tx_devourer_pass(dut, channel, size, mcs, gap_us, window, warmup, tmpdir,
                      tag) -> tuple[dict, int]:
    for_devourer(dut)
    bus, dev = bus_dev(dut)
    reader = UsbmonReader(bus, tmpdir / f"tx-dev-{dut.pid}-{tag}-usbmon.log")
    reader.start()
    log = tmpdir / f"tx-dev-{dut.pid}-ch{channel}-{size}-{tag}.log"
    proc = _spawn("WiFiDriverTxDemo",
                  _devourer_tx_env(dut, channel, size, mcs, gap_us=gap_us), log)
    try:
        time.sleep(warmup)
        bus, dev = bus_dev(dut)  # re-read post-init (devnum may have shifted)
        t0 = time.monotonic()
        time.sleep(window)
        t1 = time.monotonic()
    finally:
        regress._terminate(proc)
        reader.stop()
        time.sleep(0.3)
    return reader.parse_tx(dev, size, t0, t1), dev


def measure_tx_devourer(dut, channel, size, mcs, *, window, warmup,
                        tmpdir) -> dict:
    # Throughput pass: gap=0 (max rate); send_packet has no backpressure so URBs
    # pile up — the completion (chip-accept) rate is the headline Mbps.
    m, _ = _tx_devourer_pass(dut, channel, size, mcs, 0, window, warmup, tmpdir,
                             "tput")
    # Latency pass: gap=2000us keeps the queue shallow (in-flight ~1-2) so the
    # per-URB submit->completion time reflects real per-frame latency, not the
    # backlog drain time of the saturated pass.
    lat, _ = _tx_devourer_pass(dut, channel, size, mcs, 2000, min(4.0, window),
                               2.0, tmpdir, "lat")
    m["p50_lat_us"] = lat["p50_lat_us"]
    m["p99_lat_us"] = lat["p99_lat_us"]
    m["ok"] = m["frames"] > 0
    return m


def measure_tx_kernel(dut, channel, size, mcs, *, window, warmup,
                      tmpdir) -> dict:
    """Host-kernel TX: bind rtw88, monitor, AF_PACKET max-rate inject, usbmon."""
    drv = regress.host_kernel_driver_for_dut(dut)
    if not drv:
        regress.attach_to_host_kernel(dut)
        time.sleep(2)
        drv = regress.host_kernel_driver_for_dut(dut)
    if not drv:
        return {"ok": False, "note": "host kernel does not bind this chip"}
    iface = host_iface(dut)
    if not iface:
        return {"ok": False, "note": "no kernel wlan iface"}
    kh = regress.KernelHost.local()
    kh.iface_to_monitor(iface, channel)
    bus, dev = bus_dev(dut)
    reader = UsbmonReader(bus, tmpdir / f"tx-ker-{dut.pid}-usbmon.log")
    reader.start()
    log = tmpdir / f"tx-ker-{dut.pid}-ch{channel}-{size}.log"
    fh = open(log, "w")
    proc = regress._register_local_proc(subprocess.Popen(
        ["python3", str(HERE / "inject_beacon.py"), "--iface", iface,
         "--max-rate", "--size", str(size), "--mcs", str(mcs),
         "--duration", str(window + warmup + 2)],
        stdout=fh, stderr=subprocess.STDOUT, preexec_fn=regress._child_preexec))
    try:
        time.sleep(warmup)
        bus, dev = bus_dev(dut)
        t0 = time.monotonic()
        time.sleep(window)
        t1 = time.monotonic()
    finally:
        regress._terminate(proc)
        reader.stop()
        time.sleep(0.3)
    m = reader.parse_tx(dev, size, t0, t1)
    m["ok"] = m["frames"] > 0
    return m


def measure_rx(dut_rx, side, flooder, channel, size, mcs, *, window, warmup,
               tmpdir) -> dict:
    """Flood from `flooder` (devourer) and count at dut_rx (devourer|kernel)."""
    # Bring up the RX side first.
    rx_log = tmpdir / f"rx-{side}-{dut_rx.pid}-ch{channel}-{size}.log"
    rx_proc = None
    tcpdump = None
    if side == "devourer":
        for_devourer(dut_rx)
        env = os.environ.copy()
        env.update({"DEVOURER_VID": f"0x{dut_rx.vid}",
                    "DEVOURER_PID": f"0x{dut_rx.pid}",
                    "DEVOURER_CHANNEL": str(channel)})
        rx_proc = _spawn("WiFiDriverDemo", env, rx_log)
    else:
        if not regress.host_kernel_driver_for_dut(dut_rx):
            regress.attach_to_host_kernel(dut_rx)
            time.sleep(2)
        iface = host_iface(dut_rx)
        if not iface:
            return {"ok": False, "note": "no kernel rx iface"}
        regress.KernelHost.local().iface_to_monitor(iface, channel)
        fh = open(rx_log, "w")
        tcpdump = regress._register_local_proc(subprocess.Popen(
            ["tcpdump", "-i", iface, "-nn", "-e", "-l",
             f"ether src {CANONICAL_SA}"],
            stdout=fh, stderr=subprocess.DEVNULL,
            preexec_fn=regress._child_preexec))
    time.sleep(warmup)  # RX fwdl / monitor settle

    # Start the flooder (devourer TX, max rate).
    for_devourer(flooder)
    fbus, fdev = bus_dev(flooder)
    freader = UsbmonReader(fbus, tmpdir / f"rx-flood-{flooder.pid}-usbmon.log")
    freader.start()
    flog = tmpdir / f"rx-flood-{flooder.pid}-ch{channel}-{size}.log"
    fproc = _spawn("WiFiDriverTxDemo",
                   _devourer_tx_env(flooder, channel, size, mcs), flog)
    try:
        time.sleep(2)  # flooder init
        fbus, fdev = bus_dev(flooder)
        t0 = time.monotonic()
        time.sleep(window)
        t1 = time.monotonic()
    finally:
        regress._terminate(fproc)
        freader.stop()
        if rx_proc:
            regress._terminate(rx_proc)
        if tcpdump:
            regress._terminate(tcpdump)
        time.sleep(0.3)

    flood = freader.parse_tx(fdev, size, t0, t1)
    if side == "devourer":
        recv = regress._count_devourer_rx_hits(rx_log)
    else:
        recv = regress._count_tcpdump_hits(rx_log)
    sent = flood["frames"]
    return {
        "ok": recv > 0,
        "recv_fps": recv / window,
        "recv_mbps": recv * size * 8 / window / 1e6,
        "flooder_fps": flood["fps"],
        "loss_pct": (100 * (sent - recv) / sent) if sent > 0 else 0.0,
        "recv": recv, "sent": sent,
    }


# --------------------------------------------------------------------------- #
# matrix driver
# --------------------------------------------------------------------------- #
@dataclasses.dataclass
class Key:
    chipset: str
    band: str
    size: int
    direction: str
    side: str

    def s(self) -> str:
        return f"{self.chipset}|{self.band}|{self.size}|{self.direction}|{self.side}"


def median_metric(samples: list[dict], field: str) -> float:
    vals = [s[field] for s in samples if s.get("ok") and field in s]
    return statistics.median(vals) if vals else 0.0


def run(args) -> None:
    ensure_usbmon()
    regress._install_cleanup_handlers()
    duts = {d.chipset.split()[0]: d for d in regress.discover_duts()}
    if not duts:
        sys.exit("no supported DUTs found")
    print(f"# DUTs: {', '.join(f'{k}({v.vidpid})' for k, v in duts.items())}")

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    state_path = out / "state.json"
    state: dict = {}
    if args.resume and state_path.exists():
        state = json.loads(state_path.read_text())
    csv_path = out / "bench-tput.csv"

    bands = {b: BANDS[b] for b in args.bands}
    chips = list(duts.values())
    if args.quick:
        chips = chips[:1]
        bands = {"2g": 6}
        args.sizes = [1500]
        args.reps = 1

    rows: list[dict] = []
    for dut in chips:
        chip = dut.chipset.split()[0]
        # flooder = a reliable peer chip (8812 has the most dependable TX; avoid
        # the 8814 as flooder — its monitor-injection TX is flaky).
        flooder = None
        for pref in ("RTL8812AU", "RTL8821AU", "RTL8814AU"):
            flooder = next((d for d in duts.values()
                            if d.chipset.split()[0] == pref and d.pid != dut.pid),
                           None)
            if flooder:
                break
        for band, ch in bands.items():
            for size in args.sizes:
                for direction in args.directions:
                    for side in args.sides:
                        key = Key(chip, band, size, direction, side)
                        if args.resume and key.s() in state:
                            rows.append(state[key.s()]); continue
                        samples = []
                        for rep in range(args.reps):
                            print(f"  {key.s()} rep{rep+1}/{args.reps} ch{ch}…",
                                  flush=True)
                            try:
                                regress.usb_port_power_cycle(dut)
                            except Exception:
                                pass
                            m = _run_one(dut, side, direction, ch, size,
                                         args.mcs, flooder, args, out)
                            m.update(rep=rep, channel=ch)
                            samples.append(m)
                            _append_csv(csv_path, key, m)
                        agg = {"key": key.s(), "chipset": chip, "band": band,
                               "size": size, "direction": direction,
                               "side": side, "channel": ch}
                        if direction == "tx":
                            agg["mbps"] = round(median_metric(samples, "mbps"), 2)
                            agg["fps"] = round(median_metric(samples, "fps"))
                            agg["p50_lat_us"] = round(
                                median_metric(samples, "p50_lat_us"))
                            agg["max_inflight"] = max(
                                (s.get("max_inflight", 0) for s in samples),
                                default=0)
                        else:
                            agg["mbps"] = round(
                                median_metric(samples, "recv_mbps"), 2)
                            agg["recv_fps"] = round(
                                median_metric(samples, "recv_fps"))
                            agg["loss_pct"] = round(
                                median_metric(samples, "loss_pct"))
                        agg["ok"] = any(s.get("ok") for s in samples)
                        agg["note"] = next((s["note"] for s in samples
                                            if s.get("note")), "")
                        rows.append(agg)
                        state[key.s()] = agg
                        state_path.write_text(json.dumps(state, indent=1))
        # restore host kernel for this chip
        try:
            regress.attach_to_host_kernel(dut)
        except Exception:
            pass

    _emit_markdown(out / "bench-tput.md", rows, bands)
    print(f"\nCSV: {csv_path}\nMD : {out / 'bench-tput.md'}")


def _run_one(dut, side, direction, ch, size, mcs, flooder, args, out) -> dict:
    try:
        if direction == "tx" and side == "devourer":
            return measure_tx_devourer(dut, ch, size, mcs,
                                       window=args.window, warmup=args.warmup,
                                       tmpdir=out)
        if direction == "tx" and side == "kernel":
            return measure_tx_kernel(dut, ch, size, mcs,
                                     window=args.window, warmup=args.warmup,
                                     tmpdir=out)
        if direction == "rx":
            if flooder is None:
                return {"ok": False, "note": "no flooder peer"}
            return measure_rx(dut, side, flooder, ch, size, mcs,
                              window=args.window, warmup=args.warmup,
                              tmpdir=out)
    except Exception as e:  # never let one cell kill the matrix
        return {"ok": False, "note": f"exc: {type(e).__name__}: {e}"}
    finally:
        try:
            regress.attach_to_host_kernel(dut)
        except Exception:
            pass
    return {"ok": False, "note": "unhandled"}


def _append_csv(path: Path, key: Key, m: dict) -> None:
    new = not path.exists()
    with open(path, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["chipset", "band", "size", "direction", "side", "rep",
                        "channel", "ok", "mbps", "fps", "recv_fps", "loss_pct",
                        "p50_lat_us", "p99_lat_us", "max_inflight", "frames",
                        "errors", "note"])
        w.writerow([key.chipset, key.band, key.size, key.direction, key.side,
                    m.get("rep", 0), m.get("channel", ""), int(m.get("ok", 0)),
                    round(m.get("mbps", 0), 2), round(m.get("fps", 0)),
                    round(m.get("recv_fps", 0)), round(m.get("loss_pct", 0)),
                    round(m.get("p50_lat_us", 0)), round(m.get("p99_lat_us", 0)),
                    m.get("max_inflight", 0), m.get("frames", 0),
                    m.get("errors", 0), m.get("note", "")])


def _emit_markdown(path: Path, rows: list[dict], bands: dict) -> None:
    by = {(r["chipset"], r["band"], r["size"], r["direction"], r["side"]): r
          for r in rows}
    chips = sorted({r["chipset"] for r in rows})
    lines = ["### Measured throughput\n",
             "Median of N runs. TX rate = usbmon bulk-OUT completions at the "
             "source chip; RX = frames counted at the DUT under a same-channel "
             "flood (capped by the flooder's TX rate). HT MCS, 20 MHz, monitor "
             "injection. PSDU 1500 / 3994 B. dev = devourer, ker = host kernel. "
             "`—` = unsupported/degenerate.\n"]
    for band in bands:
        lines.append(f"\n#### {band} (ch{BANDS[band]})\n")
        lines.append("| Part | TX dev 1500/3994 (Mbps) | TX ker 1500/3994 | "
                     "TX lat dev (µs) | RX dev 1500/3994 | RX ker 1500/3994 |")
        lines.append("|------|------|------|------|------|------|")
        for chip in chips:
            def cell(direction, side):
                a = by.get((chip, band, 1500, direction, side))
                b = by.get((chip, band, 3994, direction, side))
                fa = f"{a['mbps']:.1f}" if a and a.get("ok") else "—"
                fb = f"{b['mbps']:.1f}" if b and b.get("ok") else "—"
                return f"{fa}/{fb}"
            lat = by.get((chip, band, 3994, "tx", "devourer"))
            latv = f"{lat['p50_lat_us']}" if lat and lat.get("ok") else "—"
            lines.append(f"| {chip} | {cell('tx','devourer')} | "
                         f"{cell('tx','kernel')} | {latv} | "
                         f"{cell('rx','devourer')} | {cell('rx','kernel')} |")
    path.write_text("\n".join(lines) + "\n")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bands", default="2g,unii1,unii2_3",
                    type=lambda s: s.split(","))
    ap.add_argument("--sizes", default="1500,3994",
                    type=lambda s: [int(x) for x in s.split(",")])
    ap.add_argument("--directions", default="tx,rx",
                    type=lambda s: s.split(","))
    ap.add_argument("--sides", default="devourer,kernel",
                    type=lambda s: s.split(","))
    ap.add_argument("--mcs", type=int, default=7)
    ap.add_argument("--window", type=float, default=10.0)
    ap.add_argument("--warmup", type=float, default=6.0)
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--out-dir", default="/tmp/devourer-bench-tput")
    run(ap.parse_args())


if __name__ == "__main__":
    main()
