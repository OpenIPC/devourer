#!/usr/bin/env python3
"""CPU-per-Mbit: userspace (devourer / libusb) vs kernel driver, at equal work.

The claim under test: "the libusb approach burns more host CPU than a kernel
driver." Throughput parity is already established (devourer ~80% vs vendor
~78% on-air duty); this isolates the *CPU cost of the software path* by flooding
the same HT rate at several frame sizes and measuring, for each driver:

  * system-wide CPU actually spent (all cores, /proc/stat busy) minus a tightly
    paired idle baseline — captures userspace + kernel softirq/kworker
    (usbfs/xhci, or mac80211/netstack) alike, so neither side hides work;
  * the flooding process's own CPU (/proc/<pid>/stat utime+stime) — a clean
    cross-check (for usbfs and AF_PACKET the frame copy + submit run in the
    caller's syscall context, so this captures most of the cost);
  * frames the host pushed into its TX path (the denominator = the work done).

Reported as core-ms per 1000 frames and per Mbit of payload. Per-cell paired
baseline + median-of-reps defeats the variable background load on this host.
Per-frame overhead is worst at small frames (high frame rate) — hence the sweep.

  sudo python3 tests/cpu_per_mbit.py [--sizes 1500,512,128,64] [--secs 5]
                                     [--reps 3] [--kdriver vendor|rtw88]
                                     [--skip-kernel]

Same adapter (RTL8812AU, USB2 High-Speed) for both arms so the per-frame chip
work is identical; only the host software path differs.
"""
import argparse, os, re, signal, statistics, subprocess, sys, time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CLK = os.sysconf("SC_CLK_TCK")
NCORES = os.cpu_count() or 1

VID, PID = "0x0bda", "0x8812"
CHANNEL = 149
MCS, BW = 7, 20
KDRIVERS = ["rtw88_8812au", "88XXau", "8812au", "rtl88xxau_wfb", "rtw88_8821au"]
VENDOR_KO = ROOT / "reference" / "rtl8812au" / "88XXau_ohd.ko"

_procs = []
def _spawn(cmd, **kw):
    p = subprocess.Popen(cmd, **kw); _procs.append(p); return p

def _kill_all():
    for p in _procs:
        try: p.send_signal(signal.SIGINT)
        except Exception: pass
    time.sleep(0.4)
    for p in _procs:
        try: p.kill()
        except Exception: pass

# --- CPU sampling ------------------------------------------------------------
def _sys_busy():
    v = [int(x) for x in open("/proc/stat").readline().split()[1:]]
    idle = v[3] + (v[4] if len(v) > 4 else 0)
    return sum(v) - idle

def _pid_cpu(pid):
    try:
        s = open(f"/proc/{pid}/stat").read()
        tail = s[s.rfind(")") + 2:].split()
        return int(tail[11]) + int(tail[12])       # utime + stime (jiffies)
    except Exception:
        return 0

def _pid_ustime(pid):
    """(utime, stime) jiffies — split userspace (feeder/interpreter) from
    kernel-in-syscall (usbfs copy+submit, or mac80211/driver xmit)."""
    try:
        s = open(f"/proc/{pid}/stat").read()
        tail = s[s.rfind(")") + 2:].split()
        return int(tail[11]), int(tail[12])
    except Exception:
        return 0, 0

# The only big variable background load on this host is the coding agent itself
# (comm == 'claude', ~0.6 core and drifting). Subtract it deterministically from
# BOTH flood and baseline windows so it can't masquerade as TX-path CPU. kworker
# / ksoftirqd are NOT excluded — that is the kernel driver's cost we want to keep.
NOISE_COMMS = {"claude", "node"}
def _noise_cpu():
    tot = 0
    for d in Path("/proc").iterdir():
        if not d.name.isdigit():
            continue
        try:
            if open(d / "comm").read().strip() in NOISE_COMMS:
                s = open(d / "stat").read(); tail = s[s.rfind(")") + 2:].split()
                tot += int(tail[11]) + int(tail[12])
        except Exception:
            continue
    return tot

def _window(secs, pid=None):
    """Averaged cores over `secs`: (system−noise, (pid_utime,pid_stime), system)."""
    b0 = _sys_busy(); z0 = _noise_cpu(); ut0, st0 = _pid_ustime(pid) if pid else (0, 0)
    time.sleep(secs)
    b1 = _sys_busy(); z1 = _noise_cpu(); ut1, st1 = _pid_ustime(pid) if pid else (0, 0)
    sysc = (b1 - b0) / CLK / secs
    noise = (z1 - z0) / CLK / secs
    pid_ut = (ut1 - ut0) / CLK / secs
    pid_st = (st1 - st0) / CLK / secs
    return (sysc - noise, (pid_ut, pid_st), sysc)

# --- USB bind helpers --------------------------------------------------------
def _sysfs_for(vid, pid):
    for d in Path("/sys/bus/usb/devices").glob("*"):
        try:
            if (d / "idVendor").read_text().strip() == vid.replace("0x", "") and \
               (d / "idProduct").read_text().strip() == pid.replace("0x", ""):
                return d.name
        except OSError:
            continue
    return None

def _unbind_kernel():
    for drv in KDRIVERS:
        base = Path(f"/sys/bus/usb/drivers/{drv}")
        if not base.exists():
            continue
        for d in base.iterdir():
            if d.name[0].isdigit():
                subprocess.run(["tee", str(base / "unbind")], input=d.name.encode(),
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1)

def _wlan_iface():
    r = subprocess.run(["iw", "dev"], capture_output=True, text=True)
    m = re.search(r"Interface (\w+)", r.stdout)
    return m.group(1) if m else None

def _setup_kernel_monitor(kdriver):
    _unbind_kernel()
    for drv in ["rtw88_8812au", "88xxau", "8812au"]:
        subprocess.run(["modprobe", "-r", drv],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1)
    if kdriver == "vendor":
        if VENDOR_KO.exists():
            subprocess.run(["insmod", str(VENDOR_KO)], stderr=subprocess.DEVNULL)
        subprocess.run(["modprobe", "88XXau"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        subprocess.run(["modprobe", "rtw88_8812au"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    iface = None
    for _ in range(20):
        iface = _wlan_iface()
        if iface:
            break
        time.sleep(0.5)
    if not iface:
        return None
    for cmd in (["ip", "link", "set", iface, "down"],
                ["iw", "dev", iface, "set", "type", "monitor"],
                ["ip", "link", "set", iface, "up"],
                ["iw", "dev", iface, "set", "channel", str(CHANNEL)]):
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1)
    return iface

# --- flood counters ----------------------------------------------------------
def _last_txframe(ev):
    n = 0
    try:
        for line in open(ev, "r", errors="ignore"):
            if '"ev":"tx.frame"' in line:
                m = re.search(r'"n":(\d+)', line)
                if m:
                    n = max(n, int(m.group(1)))
    except FileNotFoundError:
        pass
    return n

# --- measurement arms --------------------------------------------------------
def measure_devourer(size, secs, reps, agg=1):
    """agg>1 drives the send_packets USB TX aggregation (agg frames per bulk-OUT
    URB via DEVOURER_TX_BATCH + DEVOURER_TX_USB_AGG) — the lever that amortizes
    the per-URB submit syscall."""
    _unbind_kernel()
    ev = ROOT / "build" / f".cpu_dvr_{size}_{agg}.jsonl"
    if ev.exists():
        ev.unlink()
    env = dict(os.environ, DEVOURER_VID=VID, DEVOURER_PID=PID,
               DEVOURER_CHANNEL=str(CHANNEL), DEVOURER_TX_RATE=f"MCS{MCS}/{BW}",
               DEVOURER_TX_PAYLOAD_BYTES=str(size), DEVOURER_TX_GAP_US="0",
               DEVOURER_EVENTS="stdout", DEVOURER_LOG_LEVEL="warn")
    if agg > 1:
        env["DEVOURER_TX_USB_AGG"] = str(agg)
        env["DEVOURER_TX_BATCH"] = str(agg)
    base_pre, _, _ = _window(secs)                        # idle, adapter unbound
    with open(ev, "wb") as fout:
        p = _spawn([str(ROOT / "build" / "txdemo")], env=env,
                   stdout=fout, stderr=subprocess.DEVNULL)
        time.sleep(2.0)                                # init + TX ramp
        if p.poll() is not None:
            return None
        n0 = _last_txframe(ev); t0 = time.time()
        floods, put, pst = [], [], []
        for _ in range(reps):
            s, (ut, st), _ = _window(secs, p.pid)
            floods.append(s); put.append(ut); pst.append(st)
        dt = time.time() - t0; n1 = _last_txframe(ev)
        p.send_signal(signal.SIGINT); time.sleep(0.4); p.kill()
    base_post, _, _ = _window(secs)
    base = (base_pre + base_post) / 2
    frames = max(0, n1 - n0)
    return _rollup("devourer", size, frames, dt, statistics.median(floods) - base,
                   statistics.median(put), statistics.median(pst), (base_pre, base_post))

# Prefer the compiled C injector (negligible feeder overhead, and it runs on
# Python-less targets like OpenIPC cameras); fall back to the Python twin.
RAW_INJECT = ROOT / "build" / "raw_inject"
def _inject_cmd(iface, size, total):
    if RAW_INJECT.exists():
        return [str(RAW_INJECT), iface, str(MCS), str(size), str(int(total))]
    return ["python3", str(HERE / "kernel_tx_inject.py"),
            iface, str(MCS), str(size), str(int(total))]

RX_SRC_PID = "0xc811"  # 8821CU flood source for the RX-DUT measurement
def _last_field(path, ev, field):
    v = 0
    try:
        for line in open(path, "r", errors="ignore"):
            if f'"ev":"{ev}"' in line:
                m = re.search(rf'"{field}":(\d+)', line)
                if m:
                    v = max(v, int(m.group(1)))
    except FileNotFoundError:
        pass
    return v

def measure_rx(size, secs, reps, zerocopy):
    """RX DUT CPU (rxdemo on the 8812AU) under a flood from a second adapter,
    with the zerocopy DMA ring on vs off. Denominator = frames received."""
    _unbind_kernel()
    ev = ROOT / "build" / f".cpu_rx_{size}_{int(zerocopy)}.jsonl"
    if ev.exists():
        ev.unlink()
    src_env = dict(os.environ, DEVOURER_PID=RX_SRC_PID, DEVOURER_CHANNEL=str(CHANNEL),
                   DEVOURER_TX_RATE=f"MCS{MCS}/{BW}", DEVOURER_TX_PAYLOAD_BYTES=str(size),
                   DEVOURER_TX_GAP_US="0", DEVOURER_LOG_LEVEL="warn")
    src = _spawn([str(ROOT / "build" / "txdemo")], env=src_env,
                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(3.0)
    if src.poll() is not None:
        print("    RX source (c811) failed to start"); return None
    rx_env = dict(os.environ, DEVOURER_PID=PID, DEVOURER_CHANNEL=str(CHANNEL),
                  DEVOURER_RX_ZEROCOPY="1" if zerocopy else "0",
                  DEVOURER_EVENTS="stdout", DEVOURER_LOG_LEVEL="warn")
    with open(ev, "wb") as fout:
        rx = _spawn([str(ROOT / "build" / "rxdemo")], env=rx_env,
                    stdout=fout, stderr=subprocess.DEVNULL)
        time.sleep(2.0)
        if rx.poll() is not None:
            src.kill(); return None
        n0 = _last_field(ev, "rx.count", "total"); t0 = time.time()
        put, pst = [], []
        for _ in range(reps):
            _, (ut, st), _ = _window(secs, rx.pid)
            put.append(ut); pst.append(st)
        dt = time.time() - t0; n1 = _last_field(ev, "rx.count", "total")
        rx.send_signal(signal.SIGINT); time.sleep(0.3); rx.kill()
    src.send_signal(signal.SIGINT); time.sleep(0.3); src.kill()
    frames = max(0, n1 - n0)
    fps = frames / dt if dt else 0
    self_cores = statistics.median(put) + statistics.median(pst)
    per_kframe_ms = (self_cores / fps * 1e6) if fps else 0
    return dict(size=size, zerocopy=zerocopy, frames=frames, fps=fps,
                self_cores=self_cores, put=statistics.median(put),
                pst=statistics.median(pst), per_kframe_ms=per_kframe_ms)

def measure_kernel(iface, size, secs, reps):
    total = secs * reps + 3
    r_out = ROOT / "build" / f".cpu_ker_{size}.out"
    base_pre, _, _ = _window(secs)                        # driver bound, not TX'ing
    with open(r_out, "wb") as fout:
        p = _spawn(_inject_cmd(iface, size, total),
                   stdout=fout, stderr=subprocess.STDOUT)
        time.sleep(1.5)                                # let the sendto loop spin up
        if p.poll() is not None:
            print("    kernel inject exited early:",
                  open(r_out, errors="ignore").read().strip()[:160]); return None
        floods, put, pst = [], [], []
        t0 = time.time()
        for _ in range(reps):
            s, (ut, st), _ = _window(secs, p.pid)
            floods.append(s); put.append(ut); pst.append(st)
        dt = time.time() - t0
        try: p.wait(timeout=total)
        except Exception: p.kill()
    base_post, _, _ = _window(secs)
    base = (base_pre + base_post) / 2
    txt = open(r_out, errors="ignore").read()
    m = re.search(r"injected (\d+) frames in ([\d.]+)s", txt)
    if not m:
        print("    no inject count:", txt.strip()[:160]); return None
    tot_frames, tot_dt = int(m.group(1)), float(m.group(2))
    frames = int(tot_frames * (dt / tot_dt))           # frames during our windows
    return _rollup("kernel", size, frames, dt, statistics.median(floods) - base,
                   statistics.median(put), statistics.median(pst), (base_pre, base_post))

def _rollup(who, size, frames, dt, sys_cores, pid_ut, pid_st, base):
    fps = frames / dt if dt else 0
    sys_cores = max(sys_cores, 0.0)
    # Driver-attributable CPU excludes the userspace feeder's own interpreter/
    # loop overhead (pid utime), which differs by feeder language (C++ txdemo vs
    # Python injector) and is NOT a property of the driver. What remains =
    # kernel-in-syscall (stime: usbfs copy+submit / mac80211 xmit) + softirq/
    # kworker (sys − pid_self). This is the fair driver-vs-driver number.
    driver_cores = max(sys_cores - pid_ut, 0.0)
    # Headline = TOTAL system CPU to sustain the flood (feeder included). Fair
    # because the feeder's own overhead is measured to be negligible on the
    # kernel arm (Python blocks in sendto; utime ~5 mcore), so total CPU is the
    # honest whole cost of each approach as used.
    per_kframe_ms = (sys_cores / fps * 1e6) if fps else 0        # core-ms / 1000 fr
    mbit_s = fps * size * 8 / 1e6
    per_mbit_ms = (sys_cores / mbit_s * 1000) if mbit_s else 0   # core-ms / Mbit
    return dict(who=who, size=size, frames=frames, dt=dt, fps=fps,
                sys_cores=sys_cores, driver_cores=driver_cores,
                pid_ut=pid_ut, pid_st=pid_st, base=base,
                per_kframe_ms=per_kframe_ms, per_mbit_ms=per_mbit_ms)

def _print_row(r):
    print(f"     {r['frames']:>7} fr {r['fps']:>6.0f} fps  sys={r['sys_cores']*1000:.0f}"
          f"  feeder_ut={r['pid_ut']*1000:.0f}  kern_st={r['pid_st']*1000:.0f}"
          f"  driver={r['driver_cores']*1000:.0f}mcore"
          f"  -> {r['per_mbit_ms']:.2f} ms/Mbit  {r['per_kframe_ms']:.3f} ms/kfr"
          f"   [base {r['base'][0]*1000:.0f}->{r['base'][1]*1000:.0f}]")

def _print_table(rows, sizes):
    print("\n### CPU cost of the software TX path (system-wide, idle-subtracted)")
    print("core-ms per Mbit payload | core-ms per 1000 frames | fps  (lower = cheaper)\n")
    print("| size |  dvr ms/Mbit | ker ms/Mbit | dvr ms/kfr | ker ms/kfr | dvr fps | ker fps |")
    print("|-----:|-------------:|------------:|-----------:|-----------:|--------:|--------:|")
    for s in sizes:
        d = next((r for r in rows if r["who"] == "devourer" and r["size"] == s), None)
        k = next((r for r in rows if r["who"] == "kernel" and r["size"] == s), None)
        c = lambda r, key: f"{r[key]:.2f}" if r else "—"
        cf = lambda r: f"{r['fps']:.0f}" if r else "—"
        print(f"| {s} | {c(d,'per_mbit_ms')} | {c(k,'per_mbit_ms')} | "
              f"{c(d,'per_kframe_ms')} | {c(k,'per_kframe_ms')} | {cf(d)} | {cf(k)} |")

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sizes", default="1500,512,128,64")
    ap.add_argument("--secs", type=float, default=5.0)
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--kdriver", choices=["vendor", "rtw88"], default="vendor")
    ap.add_argument("--skip-kernel", action="store_true")
    ap.add_argument("--tx-agg", default="",
                    help="comma list of USB TX aggregation depths to also "
                    "measure on the devourer arm, e.g. 8,32 (frames per URB)")
    ap.add_argument("--rx", action="store_true",
                    help="RX mode: measure rxdemo DUT CPU with the zerocopy DMA "
                    "ring on vs off, under a flood from a second adapter")
    args = ap.parse_args()
    if os.geteuid() != 0:
        print("needs root (USB bind + modprobe + monitor iface)"); return 2
    sizes = [int(s) for s in args.sizes.split(",")]
    if not _sysfs_for(VID, PID):
        print(f"RTL8812AU ({VID}:{PID}) not found"); return 2
    print(f"# CPU-per-Mbit  RTL8812AU  MCS{MCS}/{BW}MHz ch{CHANNEL}  {NCORES} cores  "
          f"{args.secs:.0f}s x{args.reps}/cell\n")

    if args.rx:
        rx_rows = []
        try:
            for size in sizes:
                for zc in (True, False):
                    tag = "zerocopy" if zc else "heap"
                    print(f"[{size:>4}B] RX DUT ({tag}) …")
                    r = measure_rx(size, args.secs, args.reps, zc)
                    if r:
                        rx_rows.append(r)
                        print(f"     {r['frames']:>7} fr {r['fps']:>6.0f} fps  "
                              f"self={r['self_cores']*1000:.0f}mcore "
                              f"(u={r['put']*1000:.0f} s={r['pst']*1000:.0f})  "
                              f"-> {r['per_kframe_ms']:.3f} ms/kfr")
        finally:
            _kill_all()
        print("\n### RX zerocopy DMA ring — rxdemo DUT CPU per 1000 frames (core-ms)")
        print("| size | zerocopy | heap | drop |")
        print("|-----:|---------:|-----:|-----:|")
        for s in sizes:
            z = next((r for r in rx_rows if r["size"] == s and r["zerocopy"]), None)
            h = next((r for r in rx_rows if r["size"] == s and not r["zerocopy"]), None)
            drop = (f"{(1 - z['per_kframe_ms']/h['per_kframe_ms'])*100:.0f}%"
                    if z and h and h["per_kframe_ms"] else "")
            c = lambda r: f"{r['per_kframe_ms']:.3f}" if r else "—"
            print(f"| {s} | {c(z)} | {c(h)} | {drop} |")
        return 0

    aggs = [1] + [int(a) for a in args.tx_agg.split(",") if a.strip()]
    rows = []
    agg_rows = []
    try:
        for size in sizes:
            for agg in aggs:
                tag = "libusb" if agg == 1 else f"libusb+agg{agg}"
                print(f"[{size:>4}B] devourer ({tag}) …")
                d = measure_devourer(size, args.secs, args.reps, agg=agg)
                if d:
                    d["agg"] = agg
                    if agg == 1:
                        rows.append(d)
                    agg_rows.append(d)
                    _print_row(d)
        if not args.skip_kernel:
            print(f"\n[kernel] binding {args.kdriver} + monitor …")
            iface = _setup_kernel_monitor(args.kdriver)
            if not iface:
                print("  no monitor iface — skipping kernel arm")
            else:
                print(f"  monitor iface = {iface}")
                for size in sizes:
                    print(f"[{size:>4}B] kernel ({args.kdriver}) …")
                    k = measure_kernel(iface, size, args.secs, args.reps)
                    if k: rows.append(k); _print_row(k)
    finally:
        _kill_all()
    _print_table(rows, sizes)
    if len(aggs) > 1:
        _print_agg_table(agg_rows, sizes, aggs)
    return 0

def _print_agg_table(agg_rows, sizes, aggs):
    print("\n### USB TX aggregation — devourer CPU per 1000 frames (core-ms)")
    print("agg = frames packed per bulk-OUT URB (amortizes the submit syscall)\n")
    head = "| size | " + " | ".join(f"agg{a}" if a > 1 else "single" for a in aggs) + " | best drop |"
    print(head)
    print("|-----:" + "|-------:" * len(aggs) + "|----------:|")
    for s in sizes:
        cells, base = [], None
        for a in aggs:
            r = next((x for x in agg_rows if x["size"] == s and x["agg"] == a), None)
            v = r["per_kframe_ms"] if r else None
            if a == 1:
                base = v
            cells.append(f"{v:.1f}" if v is not None else "—")
        drop = ""
        best = min((x["per_kframe_ms"] for x in agg_rows
                    if x["size"] == s and x["agg"] > 1), default=None)
        if base and best is not None and base > 0:
            drop = f"{(1 - best / base) * 100:.0f}%"
        print(f"| {s} | " + " | ".join(cells) + f" | {drop} |")

if __name__ == "__main__":
    try:
        sys.exit(main())
    finally:
        _kill_all()
