#!/usr/bin/env python3
"""kchansw VM-side DUT driver (phase-b-vm).

Runs INSIDE the pinned-kernel VM as root, next to kchansw_trace.py and
kchansw_inject.py. Mirrors the bench's set-channel / ROC switch loops on
the vendor 88x2bu DUT: ftrace session with the vendor kprobes (a kernel
wedge is contained to the VM), 2 kHz tagged injector, per-switch
trace_marker + control.jsonl records — all on the VM's CLOCK_MONOTONIC.
The host orchestrator measures the VM→host clock offset, pulls the files
back, and stamps `vm_clock_shift_ns` into meta.json for the analyzer.

No recovery ladder here: a mid-run USB drop or silent-TX wedge is recorded
(kchansw.wedge) and the config aborts — the host side decides whether to
force-reset the VM.
"""

import argparse
import json
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kchansw_trace  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def log(msg):
    sys.stderr.write(f"[vm_dut {time.strftime('%H:%M:%S')}] {msg}\n")
    sys.stderr.flush()


def run(cmd, timeout=15):
    return subprocess.run(cmd, capture_output=True, text=True,
                          timeout=timeout)


def ch_to_freq(ch):
    return 5000 + 5 * ch if ch > 14 else 2407 + 5 * ch


def set_monitor(iface, ch):
    run(["ip", "link", "set", iface, "down"])
    r = run(["iw", "dev", iface, "set", "type", "monitor"])
    if r.returncode != 0:
        raise RuntimeError(f"set type monitor: {r.stderr.strip()}")
    run(["ip", "link", "set", iface, "up"])
    r = run(["iw", "dev", iface, "set", "channel", str(ch)])
    if r.returncode != 0:
        raise RuntimeError(f"set channel {ch}: {r.stderr.strip()}")


def spawn_injector(iface, out_dir, hz, size=96):
    out = open(os.path.join(out_dir, "inject.jsonl"), "a")
    err = open(os.path.join(out_dir, "inject.stderr"), "a")
    return subprocess.Popen(
        [sys.executable, os.path.join(HERE, "kchansw_inject.py"),
         "--iface", iface, "--hz", str(hz), "--size", str(size)],
        stdout=out, stderr=err)


def loop_set_channel(a, ctl, ts):
    total = a.warmup + a.switches
    for i in range(total):
        frm, to = ((a.from_ch, a.to_ch) if i % 2 == 0
                   else (a.to_ch, a.from_ch))
        t_req = time.monotonic_ns()
        ts.marker(f"KCHANSW i={i} prim=set_channel cfg={a.cfg} "
                  f"from={frm} to={to} mono={t_req}")
        ctl.write(json.dumps({"ev": "kchansw.req", "i": i, "from": frm,
                              "to": to, "mono_ns": t_req}) + "\n")
        r = run(["iw", "dev", a.iface, "set", "channel", str(to)],
                timeout=10)
        t_done = time.monotonic_ns()
        ctl.write(json.dumps({"ev": "kchansw.done", "i": i,
                              "mono_ns": t_done, "rc": r.returncode,
                              "err": r.stderr.strip()[:120]}) + "\n")
        if r.returncode != 0:
            log(f"i={i}: iw rc={r.returncode} {r.stderr.strip()[:80]}")
            if "No such device" in r.stderr or "(-19)" in r.stderr:
                ctl.write(json.dumps({"ev": "kchansw.wedge", "i": i,
                                      "mono_ns": time.monotonic_ns(),
                                      "kind": "usb-drop"}) + "\n")
                log("usb-drop — aborting config (no in-VM recovery)")
                return False
        time.sleep(a.dwell_ms / 1e3)
        if i % 50 == 49:
            ctl.flush()
            log(f"progress {i + 1}/{total}")
    return True


def loop_roc(a, ctl, ts):
    total = a.warmup + a.switches
    for i in range(total):
        t_req = time.monotonic_ns()
        ts.marker(f"KCHANSW i={i} prim=roc cfg={a.cfg} "
                  f"from={a.from_ch} to={a.to_ch} mono={t_req}")
        ctl.write(json.dumps({"ev": "kchansw.req", "i": i,
                              "mono_ns": t_req}) + "\n")
        r = run(["iw", "dev", a.iface, "offchannel",
                 str(ch_to_freq(a.to_ch)), str(a.roc_ms)], timeout=10)
        t_done = time.monotonic_ns()
        ctl.write(json.dumps({"ev": "kchansw.done", "i": i,
                              "mono_ns": t_done, "rc": r.returncode,
                              "err": r.stderr.strip()[:120]}) + "\n")
        if r.returncode != 0 and i == 0:
            log(f"offchannel rejected: {r.stderr.strip()[:100]} — aborting")
            return False
        time.sleep((a.roc_ms + 120) / 1e3)
        if i % 50 == 49:
            ctl.flush()
            log(f"progress {i + 1}/{total}")
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("prim", choices=["set-channel", "roc"])
    ap.add_argument("--cfg", required=True, help="config name for markers")
    ap.add_argument("--iface", required=True)
    ap.add_argument("--from-ch", dest="from_ch", type=int, required=True)
    ap.add_argument("--to-ch", dest="to_ch", type=int, required=True)
    ap.add_argument("--switches", type=int, required=True)
    ap.add_argument("--warmup", type=int, default=10)
    ap.add_argument("--dwell-ms", type=int, default=150)
    ap.add_argument("--roc-ms", type=int, default=20)
    ap.add_argument("--hz", type=float, default=2000.0)
    ap.add_argument("--roc-monitor-vif", type=int, default=1,
                    help="0 = trace-only ROC: do NOT add the second monitor "
                         "vif (the vendor tdls build deadlocks the kernel "
                         "in the add-interface op — no CONCURRENT_MODE)")
    ap.add_argument("--settle-s", type=float, default=8.0,
                    help="air the injector on from-ch before switching, so "
                         "the host can run its oracle-rate gate")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    if os.geteuid() != 0:
        sys.stderr.write("vm_dut: needs root\n")
        return 2
    os.makedirs(a.out, exist_ok=True)

    roc_mon = None
    if a.prim == "set-channel":
        set_monitor(a.iface, a.from_ch)
        inj_iface = a.iface
    else:
        # ROC needs a managed iface; a second monitor vif carries the
        # injector on the base channel if the combination is allowed.
        run(["ip", "link", "set", a.iface, "down"])
        run(["iw", "dev", a.iface, "set", "type", "managed"])
        run(["ip", "link", "set", a.iface, "up"])
        inj_iface = None
        r = (run(["iw", "dev", a.iface, "interface", "add", "kcmon0",
                  "type", "monitor"]) if a.roc_monitor_vif
             else subprocess.CompletedProcess([], 1, "", "skipped"))
        if r.returncode == 0:
            run(["ip", "link", "set", "kcmon0", "up"])
            if run(["iw", "dev", "kcmon0", "set", "channel",
                    str(a.from_ch)]).returncode == 0:
                roc_mon = "kcmon0"
                inj_iface = roc_mon
        log(f"roc monitor vif: {roc_mon}")

    inj = spawn_injector(inj_iface, a.out, a.hz) if inj_iface else None
    if inj:
        log(f"injector up on {inj_iface} @ {a.hz:.0f} Hz; "
            f"settling {a.settle_s:.0f}s")
        time.sleep(a.settle_s)

    ctl = open(os.path.join(a.out, "control.jsonl"), "w")
    ok = False
    try:
        with kchansw_trace.TraceSession(
                a.out, kprobes=kchansw_trace.VENDOR_PROBES) as ts:
            if a.prim == "set-channel":
                ok = loop_set_channel(a, ctl, ts)
            else:
                ok = loop_roc(a, ctl, ts)
            with open(os.path.join(a.out, "trace_inventory.json"), "w") as f:
                json.dump(ts.inventory, f, indent=1)
    finally:
        if inj and inj.poll() is None:
            inj.terminate()
        ctl.write(json.dumps({"ev": "kchansw.aliveness",
                              "inj": bool(inj and inj.poll() is None)})
                  + "\n")
        ctl.close()
        if roc_mon:
            run(["iw", "dev", roc_mon, "del"])
        with open(os.path.join(a.out, "dmesg.log"), "w") as f:
            f.write(run(["dmesg"], timeout=10).stdout)
        with open(os.path.join(a.out, "vmdut_result.json"), "w") as f:
            json.dump({"ok": ok, "prim": a.prim,
                       "roc_monitor_vif": roc_mon is not None}, f)
    log(f"done ok={ok}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
