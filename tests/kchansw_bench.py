#!/usr/bin/env python3
"""Kernel channel-switch primitive bench — the hardware orchestrator.

Measures every channel-switch primitive a standard Linux/Realtek kernel
driver exposes (monitor set-channel, AP/CSA, remain-on-channel, single-freq
active scan, TDLS capability, vendor MCC/FCS attempt) with:

  - ftrace instrumentation (tests/kchansw_trace.py): cfg80211/mac80211 events
    + kchansw/ kprobes on driver internals, trace_clock=mono;
  - an on-air oracle: two devourer rxdemo receivers pinned to the source and
    destination channels (DEVOURER_STREAM_OUT=1 + DEVOURER_RX_AGG_SA), their
    stdout stamped per line with time.monotonic_ns() by reader threads;
  - a continuous tagged-frame injector (tests/kchansw_inject.py) on the DUT
    monitor interface for RF-evidence timestamps.

Per config it writes <out>/<cfg>/{meta.json, control.jsonl, trace.txt,
oracle_a.jsonl, oracle_b.jsonl, inject.jsonl, wpa.log, dmesg.log}; offline
analysis is tests/kchansw_analyze.py. A switch whose API succeeded but shows
no destination-channel RF evidence is a FAILED measurement by design.

Subcommands:
  inventory   capability matrix (~2 min, no long runs)
  smoke       20 instrumented switches + 20 uninstrumented (overhead A/B)
  phase-a     in-tree rtw88 DUT full matrix   [--primitive ...]
  cold        ~10 driver-cold first-switch trials
  phase-b     vendor 88x2bu DUT (module must already be loaded — see
              tests/kchansw_vendor_build.sh)
  phase-c     in-tree rtw89 8852BU quick pass (lifts/restores the kestrel
              blacklist itself, VBUS-cycles the DUT after)

Run as root. Reuses tests/regress.py's adapter machinery (sysfs bind/unbind,
VBUS map, process hygiene) by import — regress.py is __main__-guarded.
"""

import argparse
import dataclasses
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import regress  # noqa: E402  (module-level is constants + defs only)
import kchansw_trace  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
RXDEMO = REPO / "build" / "rxdemo"
INJECT = Path(__file__).resolve().parent / "kchansw_inject.py"
BLACKLIST_KESTREL = "/etc/modprobe.d/zz-temp-blacklist-kestrel.conf"
BLACKLIST_HOLD = BLACKLIST_KESTREL + ".kchansw-hold"

# Roles beyond regress.SUPPORTED_DUTS (informational names only).
EXTRA_IDS = {
    "35bc:0108": "RTL8852BU (TP-Link TX20U Nano, rtw89)",
    "0cf3:9271": "AR9271 (ath9k_htc helper AP)",
}

DRIVER_SETS = {
    "rtw88": {"module": "rtw88_8822bu",
              "probes": kchansw_trace.RTW88_PROBES,
              "probe_map": {"drv": "kc_drv_set_channel",
                            "chip": "kc_chip_set_channel", "h2c": "kc_h2c"}},
    "vendor": {"module": "88x2bu_ohd",
               "probes": kchansw_trace.VENDOR_PROBES,
               "probe_map": {"drv": "kc_drv_set_channel",
                             "chip": "kc_chip_set_channel", "h2c": "kc_h2c"}},
    "rtw89": {"module": "rtw89_8852bu",
              "probes": kchansw_trace.RTW89_PROBES,
              "probe_map": {"drv": "kc_drv_set_channel",
                            "chip": "kc_chip_set_channel", "h2c": "kc_h2c"}},
}


_LOG_FILE = None


def log(msg: str) -> None:
    line = f"[kchansw {time.strftime('%H:%M:%S')}] {msg}\n"
    sys.stderr.write(line)
    if _LOG_FILE:
        _LOG_FILE.write(line)
        _LOG_FILE.flush()


def ch_to_freq(ch: int) -> int:
    return 2407 + 5 * ch if ch <= 14 else 5000 + 5 * ch


def run(cmd, timeout=15, **kw):
    return subprocess.run(cmd, capture_output=True, text=True,
                          timeout=timeout, **kw)


# ---------------------------------------------------------------------------
# Adapter roles.
# ---------------------------------------------------------------------------

def find_dut(vidpid: str) -> regress.Dut:
    vid, pid = vidpid.lower().split(":")
    for d in glob.glob("/sys/bus/usb/devices/*"):
        try:
            with open(f"{d}/idVendor") as f:
                if f.read().strip() != vid:
                    continue
            with open(f"{d}/idProduct") as f:
                if f.read().strip() != pid:
                    continue
        except (OSError, IsADirectoryError):
            continue
        name = regress.SUPPORTED_DUTS.get(vidpid, EXTRA_IDS.get(vidpid, "?"))
        return regress.Dut(sysfs_id=os.path.basename(d), vid=vid, pid=pid,
                           chipset=name)
    raise SystemExit(f"adapter {vidpid} not found on USB — plug it or fix "
                     f"--*-pid")


def dut_iface(dut: regress.Dut, timeout=25.0) -> str:
    return regress.KernelHost.local().wait_for_wlan_iface(dut, timeout)


def iface_phy(iface: str) -> str:
    return os.path.basename(os.readlink(f"/sys/class/net/{iface}/phy80211"))


def iface_mac(iface: str) -> str:
    with open(f"/sys/class/net/{iface}/address") as f:
        return f.read().strip()


def set_monitor(iface: str, ch: int, ht: str = "") -> None:
    for cmd in ([["ip", "link", "set", iface, "down"],
                 ["iw", "dev", iface, "set", "type", "monitor"],
                 ["ip", "link", "set", iface, "up"]]):
        r = run(cmd)
        if r.returncode != 0:
            raise RuntimeError(f"{' '.join(cmd)}: {r.stderr.strip()}")
    args = ["iw", "dev", iface, "set", "channel", str(ch)] + ([ht] if ht else [])
    r = run(args)
    if r.returncode != 0:
        raise RuntimeError(f"{' '.join(args)}: {r.stderr.strip()}")


def set_managed_idle(iface: str) -> None:
    run(["ip", "link", "set", iface, "down"])
    r = run(["iw", "dev", iface, "set", "type", "managed"])
    if r.returncode != 0:
        raise RuntimeError(f"set managed: {r.stderr.strip()}")
    run(["ip", "link", "set", iface, "up"])


def iface_channel(iface: str):
    r = run(["iw", "dev", iface, "info"], timeout=4)
    m = re.search(r"channel (\d+)", r.stdout)
    return int(m.group(1)) if m else None


def iface_alive(iface: str) -> bool:
    """The netdev still answers nl80211 (a managed-idle iface legitimately
    reports no channel — don't confuse that with a wedge)."""
    r = run(["iw", "dev", iface, "info"], timeout=4)
    return r.returncode == 0 and "Interface" in r.stdout


def unmanage(iface: str) -> None:
    """Keep NetworkManager (if any) off the bench interface."""
    if shutil.which("nmcli"):
        run(["nmcli", "dev", "set", iface, "managed", "no"], timeout=6)


# ---------------------------------------------------------------------------
# Long-lived child processes (oracles / injector / wpa) with stamped readers.
# ---------------------------------------------------------------------------

class Child:
    """Popen wrapper: stdout consumed by a stamping reader thread."""

    def __init__(self, name, argv, out_path, env=None, stamp=True,
                 stderr_path=None, append=False):
        self.name = name
        self.lines = 0
        self.frame_lines = 0  # oracle rx.frame decode counter
        self._stamp = stamp
        self._out = open(out_path, "a" if append else "w")
        self._err = open(stderr_path, "w") if stderr_path else subprocess.DEVNULL
        self.proc = subprocess.Popen(
            argv, stdout=subprocess.PIPE, stderr=self._err,
            env=env, text=True, bufsize=1,
            preexec_fn=regress._child_preexec)
        regress._register_local_proc(self.proc)
        self._thread = threading.Thread(target=self._pump, daemon=True)
        self._thread.start()

    def _pump(self):
        for line in self.proc.stdout:
            t = time.monotonic_ns()
            line = line.rstrip("\n")
            if not line:
                continue
            if self._stamp:
                self._out.write(json.dumps(
                    {"host_mono_ns": t, "raw": line},
                    separators=(",", ":")) + "\n")
            else:
                self._out.write(line + "\n")
            self.lines += 1
            if '"ev":"rx.frame"' in line:
                self.frame_lines += 1
        self._out.flush()

    def alive(self) -> bool:
        return self.proc.poll() is None

    def stop(self, timeout=5.0):
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self._thread.join(timeout=3.0)
        regress._unregister_local_proc(self.proc)
        try:
            self._out.close()
        except OSError:
            pass
        if self._err is not subprocess.DEVNULL:
            self._err.close()


def spawn_oracle(role, dut, channel, agg_sa, out_dir) -> Child:
    regress.detach_from_host_kernel(dut)
    env = dict(os.environ)
    env.update({
        "DEVOURER_VID": f"0x{dut.vid}", "DEVOURER_PID": f"0x{dut.pid}",
        "DEVOURER_CHANNEL": str(channel),
        "DEVOURER_EVENTS": "stdout", "DEVOURER_LOG_LEVEL": "warn",
        "DEVOURER_STREAM_OUT": "1", "DEVOURER_RX_AGG_SA": agg_sa,
    })
    c = Child(f"oracle_{role}", [str(RXDEMO)],
              os.path.join(out_dir, f"oracle_{role}.jsonl"), env=env,
              stamp=True, stderr_path=os.path.join(out_dir,
                                                   f"oracle_{role}.stderr"))
    log(f"oracle_{role}: rxdemo on {dut.vidpid} ch{channel} sa={agg_sa}")
    return c


def spawn_injector(iface, out_dir, hz, size=96, append=False) -> Child:
    return Child("inject", [sys.executable, str(INJECT), "--iface", iface,
                            "--hz", str(hz), "--size", str(size)],
                 os.path.join(out_dir, "inject.jsonl"), stamp=False,
                 stderr_path=os.path.join(out_dir, "inject.stderr"),
                 append=append)


class WpaAp:
    """hostapd AP on the DUT. (wpa_supplicant AP-mode was tried first and
    its CHAN_SWITCH returns FAIL before any nl80211 call reaches cfg80211 —
    zero rdev_channel_switch events in the trace — so hostapd is the CSA
    path; that refusal is itself a capability-matrix row.)"""

    def __init__(self, iface, freq, out_dir, beacon_int=100):
        self.iface = iface
        self.ctrl = os.path.join(out_dir, "hostapd")
        conf = os.path.join(out_dir, "hostapd.conf")
        ch = (freq - 5000) // 5 if freq >= 5000 else (freq - 2407) // 5
        with open(conf, "w") as f:
            f.write(f"interface={iface}\n"
                    f"ctrl_interface={self.ctrl}\n"
                    "driver=nl80211\n"
                    "ssid=kchansw-bench\n"
                    f"hw_mode={'a' if freq >= 5000 else 'g'}\n"
                    f"channel={ch}\n"
                    f"beacon_int={beacon_int}\n"
                    "country_code=US\n"
                    "auth_algs=1\n")
        set_managed_idle(iface)
        self.child = Child("hostapd", ["hostapd", conf],
                           os.path.join(out_dir, "wpa.log"), stamp=False,
                           stderr_path=os.path.join(out_dir, "wpa.stderr"))

    def wait_beaconing(self, timeout=25.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            r = run(["iw", "dev", self.iface, "info"], timeout=4)
            if "type AP" in r.stdout and "channel" in r.stdout:
                return True
            if not self.child.alive():
                return False
            time.sleep(0.5)
        return False

    def chan_switch(self, count, freq):
        return run(["hostapd_cli", "-p", self.ctrl, "-i", self.iface,
                    "chan_switch", str(count), str(freq)], timeout=8)

    def stop(self):
        self.child.stop()


# ---------------------------------------------------------------------------
# Config + common runner scaffold.
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class Cfg:
    name: str
    prim: str            # set_channel | csa | roc | scan
    from_ch: int
    to_ch: int
    switches: int
    warmup: int = 10
    dwell_ms: int = 150
    ht: str = ""         # "HT40+" etc. applied to both endpoints
    csa_count: int = 1
    beacon_int: int = 100
    roc_ms: int = 20
    inject_hz: float = 2000.0
    need_rf: bool = True
    fingerprint: str = "kcsw"
    rdev_entry: str = "rdev_set_monitor_channel"


class Session:
    """Everything shared across one bench invocation."""

    def __init__(self, args, driver_key):
        self.args = args
        self.driver = DRIVER_SETS[driver_key]
        self.driver_key = driver_key
        self.out_root = Path(args.out)
        self.out_root.mkdir(parents=True, exist_ok=True)
        self.dut = find_dut(args.dut_pid)
        self.dut_vidpid = args.dut_pid
        # Controller BDF for the deepest recovery rung (bus_recover).
        try:
            bus = self.dut.sysfs_id.split("-")[0]
            self.dut_ctrl_bdf = re.search(
                r"(\d{4}:[0-9a-f]{2}:[0-9a-f]{2}\.\d)",
                os.path.realpath(f"/sys/bus/usb/devices/usb{bus}")).group(1)
        except (OSError, AttributeError):
            self.dut_ctrl_bdf = ""
        self.oracle_a_dut = find_dut(args.oracle_a_pid)
        self.oracle_b_dut = find_dut(args.oracle_b_pid)
        self.dmesg_since = time.strftime("%Y-%m-%d %H:%M:%S")

    # -- DUT bring-up --------------------------------------------------------

    def dut_up(self) -> str:
        mod = self.driver["module"]
        if self.driver_key != "vendor":
            r = run(["modprobe", mod], timeout=30)
            if r.returncode != 0:
                raise SystemExit(f"modprobe {mod}: {r.stderr.strip()}")
        regress.attach_to_host_kernel(self.dut)
        dut_iface(self.dut)
        time.sleep(2.5)          # let udev finish the wlan0→wlxMAC rename
        iface = dut_iface(self.dut)
        unmanage(iface)
        run(["rfkill", "unblock", "wifi"], timeout=6)
        return iface

    # -- per-config scaffold -------------------------------------------------

    def cfg_dir(self, cfg: Cfg) -> str:
        d = self.out_root / cfg.name
        d.mkdir(parents=True, exist_ok=True)
        return str(d)

    def write_meta(self, cfg: Cfg, d: str, oa_ch, ob_ch, extra=None):
        meta = {
            "cfg": cfg.name, "prim": cfg.prim, "from": cfg.from_ch,
            "to": cfg.to_ch, "warmup": cfg.warmup,
            "oracle_a_ch": oa_ch, "oracle_b_ch": ob_ch,
            "rdev_entry": cfg.rdev_entry,
            "probe_map": self.driver["probe_map"],
            "fingerprint": cfg.fingerprint, "need_rf": cfg.need_rf,
            "driver": self.driver_key, "module": self.driver["module"],
        }
        meta.update(extra or {})
        with open(os.path.join(d, "meta.json"), "w") as f:
            json.dump(meta, f, indent=1)

    def dmesg_dump(self, d: str):
        r = run(["journalctl", "-k", "--since", self.dmesg_since,
                 "--no-pager"], timeout=20)
        with open(os.path.join(d, "dmesg.log"), "w") as f:
            f.write(r.stdout)
        bad = [ln for ln in r.stdout.splitlines()
               if re.search(r"WARNING|BUG:|Oops|Call Trace|xhci.*(die|halt)",
                            ln)]
        if bad:
            log(f"!! dmesg flagged {len(bad)} suspicious lines (see dmesg.log)")
        return bad

    def control_writer(self, d: str):
        return open(os.path.join(d, "control.jsonl"), "w")


def oracle_rate_gate(oracle: Child, min_frames=50, timeout=20.0) -> bool:
    """Liveness: the oracle must be DECODING (rx.frame lines, not just init
    chatter — the smoke run showed the loose any-line gate passing on a
    channel the oracle could barely hear)."""
    start = oracle.frame_lines
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if oracle.frame_lines - start >= min_frames:
            return True
        if not oracle.alive():
            return False
        time.sleep(0.5)
    return oracle.frame_lines - start >= min_frames


def _port_disable_path(sysfs_id: str) -> str:
    """sysfs per-port power node for a device id: '10-2' (root port) →
    usb10/10-0:1.0/usb10-port2/disable; '3-2.3.4.3' (nested hub) →
    3-2.3.4/3-2.3.4:1.0/3-2.3.4-port3/disable."""
    bus, _, chain = sysfs_id.partition("-")
    if "." not in chain:
        return (f"/sys/bus/usb/devices/usb{bus}/{bus}-0:1.0/"
                f"usb{bus}-port{chain}/disable")
    parent, _, port = chain.rpartition(".")
    pdev = f"{bus}-{parent}"
    return (f"/sys/bus/usb/devices/{pdev}/{pdev}:1.0/"
            f"{pdev}-port{port}/disable")


def bus_recover(s: "Session") -> bool:
    """Deepest rung: the device fell off the bus or its xHCI hung.
    (Observed live: rtw88 churn wedged a T3U so hard its uPD720201 host
    controller failed re-probe with -110, and only a PCI bus reset +
    D3hot→D0 bounce + rescan, then a long per-port power-off, brought the
    chip back — on the USB2 companion bus.) Try: 12 s per-port power-off at
    the last-known port; if the controller's buses are gone, PCI-revive the
    controller first."""
    bdf = s.dut_ctrl_bdf
    bus = s.dut.sysfs_id.split("-")[0]
    if bdf and not os.path.isdir(f"/sys/bus/usb/devices/usb{bus}"):
        log(f"bus_recover: usb{bus} gone — PCI reviving {bdf}")
        for cmd in ([f"echo 1 > /sys/bus/pci/devices/{bdf}/reset"],
                    [f"setpci -s {bdf} CAP_PM+4.b=03"],
                    [f"setpci -s {bdf} CAP_PM+4.b=00"],
                    [f"echo 1 > /sys/bus/pci/devices/{bdf}/remove"],
                    ["echo 1 > /sys/bus/pci/rescan"]):
            subprocess.run(["sh", "-c", cmd[0]], capture_output=True)
            time.sleep(2)
        time.sleep(6)
    p = _port_disable_path(s.dut.sysfs_id)
    if os.path.exists(p):
        try:
            with open(p, "w") as f:
                f.write("1")
            time.sleep(12)
            with open(p, "w") as f:
                f.write("0")
            time.sleep(8)
        except OSError as e:
            log(f"bus_recover: port cycle {p}: {e}")
    # The device may return on a different bus (SS↔HS companion switch) —
    # rediscover by VID:PID before any sysfs writes.
    end = time.monotonic() + 25
    while time.monotonic() < end:
        try:
            s.dut = find_dut(s.dut_vidpid)
            return True
        except SystemExit:
            time.sleep(2)
    return False


def recover_dut(s: "Session", ch: int, ht: str = "", start: str = "rebind"):
    """Recovery ladder after a DUT wedge/USB drop (all four rungs proven
    necessary live on this rig): re-probe → module reload →
    authorized/VBUS power-cycle → bus/controller revive. Returns
    (iface, rung) or (None, 'exhausted'). Every rung rediscovers the
    device by VID:PID first — wedge recoveries can move it across buses."""
    mod = s.driver["module"]
    ladder = ("rebind", "module-reload", "power-cycle", "bus-recover")
    for action in ladder[ladder.index(start):]:
        try:
            try:
                s.dut = find_dut(s.dut_vidpid)
            except SystemExit:
                if action != "bus-recover":
                    log(f"recover_dut: {action}: device off the bus — "
                        f"escalating")
                    continue
            if action == "module-reload" and s.driver_key != "vendor":
                run(["modprobe", "-r", mod], timeout=30)
                run(["modprobe", mod], timeout=60)
            elif action == "power-cycle":
                regress.usb_port_power_cycle(s.dut, settle_s=3.0)
                if s.driver_key != "vendor":
                    run(["modprobe", mod], timeout=60)
            elif action == "bus-recover":
                if not bus_recover(s):
                    continue
                if s.driver_key != "vendor":
                    run(["modprobe", mod], timeout=60)
            regress.attach_to_host_kernel(s.dut)
            iface = dut_iface(s.dut, timeout=20)
            unmanage(iface)
            set_monitor(iface, ch, ht)
            log(f"recover_dut: '{action}' worked → {iface}")
            return iface, action
        except Exception as e:  # noqa: BLE001 — every rung may legitimately fail
            log(f"recover_dut: {action} failed: {e}")
    return None, "exhausted"


# ---------------------------------------------------------------------------
# Primitive runners.
# ---------------------------------------------------------------------------

def run_set_channel(s: Session, cfg: Cfg, iface: str, uninstrumented=False):
    d = s.cfg_dir(cfg)
    s.write_meta(cfg, d, cfg.from_ch, cfg.to_ch)
    oa = spawn_oracle("a", s.oracle_a_dut, cfg.from_ch, "canon", d)
    ob = spawn_oracle("b", s.oracle_b_dut, cfg.to_ch, "canon", d)
    set_monitor(iface, cfg.from_ch, cfg.ht)
    inj = spawn_injector(iface, d, cfg.inject_hz)
    ok = oracle_rate_gate(oa)
    if not ok:
        log(f"{cfg.name}: oracle_a not decoding the injector stream — "
            f"placement/saturation? continuing, rows may fail no_rf")
    ctl = s.control_writer(d)
    trace_ctx = (kchansw_trace.TraceSession(d, kprobes=s.driver["probes"])
                 if not uninstrumented else None)
    per_switch_wall = []
    try:
        if trace_ctx:
            trace_ctx.__enter__()
        total = cfg.warmup + cfg.switches
        last_prog = -1
        for i in range(total):
            frm, to = ((cfg.from_ch, cfg.to_ch) if i % 2 == 0
                       else (cfg.to_ch, cfg.from_ch))
            t_req = time.monotonic_ns()
            if trace_ctx:
                trace_ctx.marker(f"KCHANSW i={i} prim={cfg.prim} "
                                 f"cfg={cfg.name} from={frm} to={to} "
                                 f"mono={t_req}")
            ctl.write(json.dumps({"ev": "kchansw.req", "i": i,
                                  "from": frm, "to": to,
                                  "mono_ns": t_req}) + "\n")
            argv = ["iw", "dev", iface, "set", "channel", str(to)]
            if cfg.ht:
                argv.append(cfg.ht)
            r = run(argv, timeout=10)
            t_done = time.monotonic_ns()
            per_switch_wall.append(t_done - t_req)
            ctl.write(json.dumps({"ev": "kchansw.done", "i": i,
                                  "mono_ns": t_done, "rc": r.returncode,
                                  "err": r.stderr.strip()[:120]}) + "\n")
            if r.returncode != 0:
                log(f"{cfg.name} i={i}: iw rc={r.returncode} "
                    f"{r.stderr.strip()[:80]}")
                if "No such device" in r.stderr or "(-19)" in r.stderr:
                    # USB drop mid-config (seen in smoke): recover, respawn
                    # the injector (its socket died with the old ifindex),
                    # and mark everything after as post-recovery.
                    ctl.write(json.dumps({"ev": "kchansw.wedge", "i": i,
                                          "mono_ns": time.monotonic_ns(),
                                          "kind": "usb-drop"}) + "\n")
                    inj.stop()
                    iface2, rung = recover_dut(s, to, cfg.ht)
                    ctl.write(json.dumps({"ev": "kchansw.recovered", "i": i,
                                          "rung": rung,
                                          "ok": iface2 is not None}) + "\n")
                    if iface2 is None:
                        log(f"{cfg.name}: recovery exhausted — aborting")
                        break
                    iface = iface2
                    inj = spawn_injector(iface, d, cfg.inject_hz, append=True)
            time.sleep(cfg.dwell_ms / 1e3)
            if i % 25 == 24:
                ctl.flush()
                if not (inj.alive() and oa.alive() and ob.alive()):
                    log(f"{cfg.name}: child died (inj={inj.alive()} "
                        f"oa={oa.alive()} ob={ob.alive()}) — aborting config")
                    break
                prog = oa.frame_lines + ob.frame_lines
                if prog == last_prog:
                    # Injector alive and erroring 0, yet neither oracle
                    # decodes anything new: the silent-TX wedge (probed on
                    # this rig — dmesg stays clean; module reload clears it).
                    log(f"{cfg.name} i={i}: silent-TX stall — recovering")
                    ctl.write(json.dumps({"ev": "kchansw.wedge", "i": i,
                                          "mono_ns": time.monotonic_ns(),
                                          "kind": "silent-tx"}) + "\n")
                    inj.stop()
                    cur_ch = to
                    iface2, rung = recover_dut(s, cur_ch, cfg.ht,
                                               start="module-reload")
                    ctl.write(json.dumps({"ev": "kchansw.recovered", "i": i,
                                          "rung": rung,
                                          "ok": iface2 is not None}) + "\n")
                    if iface2 is None:
                        log(f"{cfg.name}: recovery exhausted — aborting")
                        break
                    iface = iface2
                    inj = spawn_injector(iface, d, cfg.inject_hz, append=True)
                last_prog = oa.frame_lines + ob.frame_lines
    finally:
        if trace_ctx:
            trace_ctx.__exit__(None, None, None)
            with open(os.path.join(d, "trace_inventory.json"), "w") as f:
                json.dump(trace_ctx.inventory, f, indent=1)
        alive = {"inj": inj.alive(), "oa": oa.alive(), "ob": ob.alive(),
                 "iface_ch": iface_channel(iface)}
        inj.stop()
        oa.stop()
        ob.stop()
        ctl.write(json.dumps({"ev": "kchansw.aliveness", **alive}) + "\n")
        ctl.close()
        s.dmesg_dump(d)
    log(f"{cfg.name}: done ({cfg.warmup}+{cfg.switches} switches)")
    return per_switch_wall, iface


def run_csa(s: Session, cfg: Cfg, iface: str):
    d = s.cfg_dir(cfg)
    mac = iface_mac(iface)
    s.write_meta(cfg, d, cfg.from_ch, cfg.to_ch,
                 {"bssid": mac, "csa_count": cfg.csa_count,
                  "beacon_int": cfg.beacon_int})
    oa = spawn_oracle("a", s.oracle_a_dut, cfg.from_ch, mac, d)
    ob = spawn_oracle("b", s.oracle_b_dut, cfg.to_ch, mac, d)
    ap = WpaAp(iface, ch_to_freq(cfg.from_ch), d, cfg.beacon_int)
    ctl = s.control_writer(d)
    try:
        if not ap.wait_beaconing():
            ctl.write(json.dumps({"ev": "kchansw.capability",
                                  "prim": "csa",
                                  "status": "ap-start-failed"}) + "\n")
            log(f"{cfg.name}: wpa_supplicant AP failed to start — "
                f"CSA row = blocked (see wpa.log)")
            return
        if not oracle_rate_gate(oa, min_frames=10, timeout=15):
            log(f"{cfg.name}: oracle_a hears no beacons yet — continuing")
        with kchansw_trace.TraceSession(d, kprobes=s.driver["probes"]) as ts:
            total = cfg.warmup + cfg.switches
            cur = cfg.from_ch
            consec_fail = 0
            any_ok = False
            for i in range(total):
                to = cfg.to_ch if cur == cfg.from_ch else cfg.from_ch
                t_req = time.monotonic_ns()
                ts.marker(f"KCHANSW i={i} prim=csa cfg={cfg.name} "
                          f"from={cur} to={to} mono={t_req}")
                ctl.write(json.dumps({"ev": "kchansw.req", "i": i,
                                      "from": cur, "to": to,
                                      "mono_ns": t_req}) + "\n")
                r = ap.chan_switch(cfg.csa_count, ch_to_freq(to))
                rc = r.returncode if "OK" in r.stdout else 1
                # completion: driver reports the new channel
                deadline = time.monotonic() + 5.0
                moved = False
                while time.monotonic() < deadline:
                    if iface_channel(iface) == to:
                        moved = True
                        break
                    time.sleep(0.02)
                t_done = time.monotonic_ns()
                ctl.write(json.dumps({"ev": "kchansw.done", "i": i,
                                      "mono_ns": t_done, "rc": rc,
                                      "moved": moved,
                                      "err": (r.stdout + r.stderr).strip()[:120]
                                      }) + "\n")
                if not moved:
                    log(f"{cfg.name} i={i}: CSA did not complete "
                        f"({(r.stdout + r.stderr).strip()[:80]})")
                    consec_fail += 1
                    if consec_fail >= 3 and not any_ok:
                        # The driver refuses CSA outright (observed on
                        # rtw88: hostapd "CSA is not supported" — the wiphy
                        # never advertises AP_CSA, nothing reaches
                        # cfg80211). Record the classification and stop
                        # burning 5 s per attempt.
                        ctl.write(json.dumps({
                            "ev": "kchansw.capability", "prim": "csa",
                            "status": "policy-rejected",
                            "detail": "AP up + beaconing, but chan_switch "
                                      "refused pre-nl80211 (no AP_CSA drv "
                                      "flag); 0 rdev_channel_switch events",
                        }) + "\n")
                        log(f"{cfg.name}: CSA refused by stack — recording "
                            f"capability row, aborting config")
                        break
                else:
                    consec_fail = 0
                    any_ok = True
                cur = to if moved else cur
                time.sleep(0.3)
                if i % 50 == 49:
                    ctl.flush()
                    if not ap.child.alive():
                        log(f"{cfg.name}: wpa_supplicant died — aborting")
                        break
            with open(os.path.join(d, "trace_inventory.json"), "w") as f:
                json.dump(ts.inventory, f, indent=1)
    finally:
        ap.stop()
        oa.stop()
        ob.stop()
        ctl.close()
        s.dmesg_dump(d)
    log(f"{cfg.name}: done")


def run_roc(s: Session, cfg: Cfg, iface: str):
    d = s.cfg_dir(cfg)
    # Enabler: a second monitor vif pins the base channel and carries the
    # injector, so ROC departure/return both leave RF evidence. If the
    # driver's iface combinations refuse it, fall back to trace-only timing.
    mon = "kcmon0"
    have_mon = False
    set_managed_idle(iface)
    r = run(["iw", "dev", iface, "interface", "add", mon, "type", "monitor"])
    if r.returncode == 0:
        try:
            run(["ip", "link", "set", mon, "up"])
            r2 = run(["iw", "dev", mon, "set", "channel", str(cfg.from_ch)])
            have_mon = r2.returncode == 0
        except Exception:
            have_mon = False
    cfg.need_rf = have_mon
    s.write_meta(cfg, d, cfg.from_ch, cfg.to_ch,
                 {"monitor_vif": have_mon, "roc_ms": cfg.roc_ms})
    oa = spawn_oracle("a", s.oracle_a_dut, cfg.from_ch, "canon", d)
    ob = spawn_oracle("b", s.oracle_b_dut, cfg.to_ch, "canon", d)
    inj = spawn_injector(mon, d, cfg.inject_hz) if have_mon else None
    if inj:
        oracle_rate_gate(oa)
    ctl = s.control_writer(d)
    try:
        with kchansw_trace.TraceSession(d, kprobes=s.driver["probes"]) as ts:
            total = cfg.warmup + cfg.switches
            for i in range(total):
                t_req = time.monotonic_ns()
                ts.marker(f"KCHANSW i={i} prim=roc cfg={cfg.name} "
                          f"from={cfg.from_ch} to={cfg.to_ch} mono={t_req}")
                ctl.write(json.dumps({"ev": "kchansw.req", "i": i,
                                      "mono_ns": t_req}) + "\n")
                r = run(["iw", "dev", iface, "offchannel",
                         str(ch_to_freq(cfg.to_ch)), str(cfg.roc_ms)],
                        timeout=10)
                t_done = time.monotonic_ns()
                ctl.write(json.dumps({"ev": "kchansw.done", "i": i,
                                      "mono_ns": t_done, "rc": r.returncode,
                                      "err": r.stderr.strip()[:120]}) + "\n")
                if r.returncode != 0 and i == 0:
                    log(f"{cfg.name}: offchannel rejected: "
                        f"{r.stderr.strip()[:100]} — aborting config")
                    break
                time.sleep((cfg.roc_ms + 120) / 1e3)
                if i % 50 == 49:
                    ctl.flush()
            with open(os.path.join(d, "trace_inventory.json"), "w") as f:
                json.dump(ts.inventory, f, indent=1)
    finally:
        if inj:
            inj.stop()
        oa.stop()
        ob.stop()
        ctl.close()
        run(["iw", "dev", mon, "del"])
        s.dmesg_dump(d)
    log(f"{cfg.name}: done (monitor_vif={have_mon})")


def run_scan(s: Session, cfg: Cfg, iface: str):
    d = s.cfg_dir(cfg)
    set_managed_idle(iface)
    mac = iface_mac(iface)
    s.write_meta(cfg, d, cfg.from_ch, cfg.to_ch, {"scan_sa": mac})
    # Only the destination oracle matters (probe requests on the target
    # frequency); a base-channel "last frame" doesn't exist for an idle STA.
    oa = spawn_oracle("a", s.oracle_a_dut, cfg.from_ch, mac, d)
    ob = spawn_oracle("b", s.oracle_b_dut, cfg.to_ch, mac, d)
    ctl = s.control_writer(d)
    try:
        with kchansw_trace.TraceSession(d, kprobes=s.driver["probes"]) as ts:
            total = cfg.warmup + cfg.switches
            for i in range(total):
                t_req = time.monotonic_ns()
                ts.marker(f"KCHANSW i={i} prim=scan cfg={cfg.name} "
                          f"from={cfg.from_ch} to={cfg.to_ch} mono={t_req}")
                ctl.write(json.dumps({"ev": "kchansw.req", "i": i,
                                      "mono_ns": t_req}) + "\n")
                r = run(["iw", "dev", iface, "scan", "trigger", "freq",
                         str(ch_to_freq(cfg.to_ch))], timeout=10)
                t_done = time.monotonic_ns()
                ctl.write(json.dumps({"ev": "kchansw.done", "i": i,
                                      "mono_ns": t_done, "rc": r.returncode,
                                      "err": r.stderr.strip()[:120]}) + "\n")
                if r.returncode != 0 and i == 0 and "Operation not supported" \
                        in r.stderr:
                    log(f"{cfg.name}: scan trigger unsupported — aborting")
                    break
                time.sleep(0.4)  # single-freq scan completes well within
                if i % 50 == 49:
                    ctl.flush()
            with open(os.path.join(d, "trace_inventory.json"), "w") as f:
                json.dump(ts.inventory, f, indent=1)
    finally:
        oa.stop()
        ob.stop()
        ctl.close()
        s.dmesg_dump(d)
    log(f"{cfg.name}: done")


def probe_tdls(s: Session, iface: str, out_root: Path):
    """Capability row only: what the stack/driver claim about TDLS ch-sw."""
    phy = iface_phy(iface)
    r = run(["iw", "phy", phy, "info"], timeout=10)
    caps = {
        "tdls_supported": "tdls" in r.stdout.lower(),
        "tdls_channel_switch_ext": "TDLS_CHANNEL_SWITCH" in r.stdout
                                   or "tdls channel switch" in r.stdout.lower(),
        "phy_info_excerpt": [ln.strip() for ln in r.stdout.splitlines()
                             if "tdls" in ln.lower()],
        "peer_prereqs": "blocked-hardware-absent (no helper AP on rig; "
                        "TDLS needs two associated STAs)",
    }
    with open(out_root / "tdls_capability.json", "w") as f:
        json.dump(caps, f, indent=2)
    log(f"TDLS capability: supported={caps['tdls_supported']} "
        f"ch-switch={caps['tdls_channel_switch_ext']}")


def run_cold(s: Session, trials: int):
    mod = s.driver["module"]
    d = str(s.out_root / "cold")
    os.makedirs(d, exist_ok=True)
    cfg = Cfg(name="cold", prim="set_channel", from_ch=36, to_ch=40,
              switches=1, warmup=0)
    s.write_meta(cfg, d, 36, 40, {"trials": trials})
    oa = spawn_oracle("a", s.oracle_a_dut, 36, "canon", d)
    ob = spawn_oracle("b", s.oracle_b_dut, 40, "canon", d)
    ctl = s.control_writer(d)
    inj = None
    try:
        with kchansw_trace.TraceSession(d, kprobes=s.driver["probes"]) as ts:
            for i in range(trials):
                log(f"cold trial {i}: power-cycle + module reload")
                if inj:
                    inj.stop()
                    inj = None
                regress.usb_port_power_cycle(s.dut, settle_s=3.0)
                run(["modprobe", "-r", mod], timeout=30)
                t_load = time.monotonic_ns()
                run(["modprobe", mod], timeout=60)
                regress.attach_to_host_kernel(s.dut)
                try:
                    iface = dut_iface(s.dut, timeout=30)
                except RuntimeError as e:
                    ctl.write(json.dumps({"ev": "kchansw.done", "i": i,
                                          "rc": 1, "err": str(e)[:120]}) + "\n")
                    continue
                unmanage(iface)
                set_monitor(iface, 36)
                inj = spawn_injector(iface, d, 2000)
                time.sleep(1.0)  # stream on 36 so last-src exists
                t_req = time.monotonic_ns()
                ts.marker(f"KCHANSW i={i} prim=set_channel cfg=cold "
                          f"from=36 to=40 mono={t_req}")
                ctl.write(json.dumps({"ev": "kchansw.req", "i": i,
                                      "mono_ns": t_req,
                                      "t_load_ns": t_load}) + "\n")
                r = run(["iw", "dev", iface, "set", "channel", "40"])
                ctl.write(json.dumps({"ev": "kchansw.done", "i": i,
                                      "mono_ns": time.monotonic_ns(),
                                      "rc": r.returncode}) + "\n")
                time.sleep(1.0)  # stream on 40 so first-dst exists
            with open(os.path.join(d, "trace_inventory.json"), "w") as f:
                json.dump(ts.inventory, f, indent=1)
    finally:
        if inj:
            inj.stop()
        oa.stop()
        ob.stop()
        ctl.close()
        s.dmesg_dump(d)
    log("cold: done")


# ---------------------------------------------------------------------------
# MCC attempt (phase-b, time-boxed, honest failure rows).
# ---------------------------------------------------------------------------

def run_mcc_attempt(s: Session, iface: str, helper_pid: str):
    d = str(s.out_root / "mcc")
    os.makedirs(d, exist_ok=True)
    result = {"prim": "mcc", "status": "attempted", "steps": []}

    def step(name, ok, detail=""):
        result["steps"].append({"step": name, "ok": ok, "detail": detail})
        log(f"mcc: {name}: {'ok' if ok else 'FAILED'} {detail}")
        return ok

    try:
        helper = find_dut(helper_pid)
    except SystemExit:
        result["status"] = "blocked-hardware-absent"
        step("helper adapter", False, f"{helper_pid} not plugged")
        with open(os.path.join(d, "mcc_result.json"), "w") as f:
            json.dump(result, f, indent=2)
        return
    # Helper AP on ch36 via rtw88 + wpa_supplicant (the 8821AU is 5G-capable).
    run(["modprobe", "rtw88_8821au"], timeout=30)
    regress.attach_to_host_kernel(helper)
    try:
        h_iface = dut_iface(helper)
    except RuntimeError as e:
        step("helper iface", False, str(e))
        result["status"] = "blocked-helper-bringup"
        with open(os.path.join(d, "mcc_result.json"), "w") as f:
            json.dump(result, f, indent=2)
        return
    unmanage(h_iface)
    ap = WpaAp(h_iface, ch_to_freq(36), d)
    ok = step("helper AP beaconing on 36", ap.wait_beaconing())
    sta_conf = os.path.join(d, "wpa_sta.conf")
    sta = None
    if ok:
        with open(sta_conf, "w") as f:
            f.write(f"ctrl_interface=DIR={d}/wpa_sta\ncountry=US\n"
                    "network={\n    ssid=\"kchansw-bench\"\n"
                    "    key_mgmt=NONE\n}\n")
        set_managed_idle(iface)
        sta = Child("wpa_sta", ["wpa_supplicant", "-Dnl80211", "-i", iface,
                                "-c", sta_conf],
                    os.path.join(d, "wpa_sta.log"), stamp=False)
        end = time.monotonic() + 30
        assoc = False
        while time.monotonic() < end:
            r = run(["iw", "dev", iface, "link"], timeout=4)
            if "Connected to" in r.stdout:
                assoc = True
                break
            time.sleep(1)
        ok = step("DUT STA associated to helper", assoc)
    if ok:
        # Second vif for the concurrent leg.
        r = run(["iw", "dev", iface, "interface", "add", "kcap0",
                 "type", "managed"])
        ok = step("second vif add", r.returncode == 0, r.stderr.strip()[:100])
    if ok:
        # Vendor proc knobs (only exist on the vendor driver).
        knobs = glob.glob(f"/proc/net/*/{iface}/mcc_*")
        ok = step("mcc proc knobs present", bool(knobs), str(knobs[:3]))
        if ok:
            base = os.path.dirname(knobs[0])
            try:
                with open(os.path.join(base, "mcc_enable"), "w") as f:
                    f.write("1")
                step("mcc_enable=1", True)
                for k in ("mcc_info", "mcc_policy_table"):
                    p = os.path.join(base, k)
                    if os.path.exists(p):
                        with open(p) as f:
                            result[k] = f.read()[:4000]
            except OSError as e:
                step("mcc_enable", False, str(e))
    if not ok:
        result["status"] = "blocked-see-steps"
    with open(os.path.join(d, "mcc_result.json"), "w") as f:
        json.dump(result, f, indent=2)
    if sta:
        sta.stop()
    ap.stop()
    run(["iw", "dev", "kcap0", "del"])
    s.dmesg_dump(d)


# ---------------------------------------------------------------------------
# Phase B in the VM. Kprobing the vendor module on the host deadlocked the
# host kernel outright (ssh dead, dmesg blocked, power cycle to recover) —
# twice — so the DUT + trace session live inside the pinned-kernel VM
# (tests/kchansw_vm.sh prepare) and the oracles stay on the host. The
# VM-side files carry VM CLOCK_MONOTONIC; a min-RTT midpoint estimator
# measures the VM→host offset and the analyzer shifts them into the host
# timeline (VM-internal spans are differences, i.e. shift-invariant).
# ---------------------------------------------------------------------------

class VmDut:
    SSH_OPTS = ["-o", "StrictHostKeyChecking=no",
                "-o", "UserKnownHostsFile=/dev/null",
                "-o", "LogLevel=ERROR", "-o", "ConnectTimeout=8"]
    # The bench runs under sudo; offer the invoking user's key.
    _sudo_user = os.environ.get("SUDO_USER")
    if _sudo_user:
        for _k in ("id_ed25519", "id_rsa"):
            _key = os.path.expanduser(f"~{_sudo_user}/.ssh/{_k}")
            if os.path.exists(_key):
                SSH_OPTS += ["-i", _key]
                break

    def __init__(self, dest: str):
        self.dest = dest

    def ssh(self, cmd: str, timeout=30):
        return run(["ssh", *self.SSH_OPTS, self.dest, cmd], timeout=timeout)

    def popen(self, cmd: str, **kw):
        return subprocess.Popen(["ssh", *self.SSH_OPTS, self.dest, cmd],
                                **kw)

    def clock_shift_ns(self, rounds=25):
        """host_mono ≈ vm_mono + shift, from the min-RTT midpoint sample."""
        p = self.popen(
            "python3 -u -c 'import sys,time\n"
            "for _ in sys.stdin: print(time.monotonic_ns(), flush=True)'",
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
            bufsize=1)
        best = None
        try:
            for _ in range(rounds):
                t0 = time.monotonic_ns()
                p.stdin.write("x\n")
                p.stdin.flush()
                line = p.stdout.readline()
                t1 = time.monotonic_ns()
                if not line.strip():
                    raise RuntimeError("clock_shift_ns: ssh pipe died "
                                       "(VM wedged?)")
                vm_ns = int(line.strip())
                rtt = t1 - t0
                if best is None or rtt < best[1]:
                    best = ((t0 + t1) // 2 - vm_ns, rtt)
        finally:
            p.kill()
        if best is None:
            raise RuntimeError("clock_shift_ns: no samples")
        return best

    def iface(self) -> str:
        r = self.ssh("ls /sys/class/net | grep -v -e lo -e '^en' | head -1")
        name = r.stdout.strip()
        if r.returncode != 0 or not name:
            raise SystemExit("phase-b-vm: no wlan netdev in the VM — "
                             "run tests/kchansw_vm.sh prepare first")
        return name


def run_vm_config(s: Session, vm: VmDut, cfg: Cfg, vm_iface: str):
    d = s.cfg_dir(cfg)
    oa = spawn_oracle("a", s.oracle_a_dut, cfg.from_ch, "canon", d)
    ob = spawn_oracle("b", s.oracle_b_dut, cfg.to_ch, "canon", d)
    shift0, rtt0 = vm.clock_shift_ns()
    log(f"{cfg.name}: vm clock shift {shift0 / 1e6:.3f} ms "
        f"(min-RTT {rtt0 / 1e6:.2f} ms)")
    vm_out = f"/tmp/kchansw-vm/{cfg.name}"
    vm.ssh(f"sudo rm -rf {vm_out}")
    prim = "set-channel" if cfg.prim == "set_channel" else "roc"
    p = vm.popen(
        f"sudo python3 /opt/kchansw/kchansw_vm_dut.py {prim} "
        f"--cfg {cfg.name} --iface {vm_iface} --from-ch {cfg.from_ch} "
        f"--to-ch {cfg.to_ch} --switches {cfg.switches} "
        f"--warmup {cfg.warmup} --dwell-ms {cfg.dwell_ms} "
        f"--roc-ms {cfg.roc_ms} --hz {cfg.inject_hz:.0f} "
        f"--roc-monitor-vif 0 --out {vm_out}",
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True,
        bufsize=1)
    threading.Thread(
        target=lambda: [log(f"[vm] {ln.rstrip()}") for ln in p.stderr],
        daemon=True).start()
    if prim == "set-channel":
        oracle_rate_gate(oa)  # runs inside the vm_dut settle window
    budget = (cfg.warmup + cfg.switches) * (cfg.dwell_ms / 1e3 + 0.8) + 180
    try:
        rc = p.wait(timeout=budget)
    except subprocess.TimeoutExpired:
        log(f"{cfg.name}: vm_dut overran {budget:.0f}s — killing "
            f"(VM wedged? `virsh reset` and re-prepare)")
        p.kill()
        rc = -9
    shift1 = None
    try:
        shift1, _ = vm.clock_shift_ns()
        if abs(shift1 - shift0) > 5_000_000:
            log(f"{cfg.name}: VM clock drifted "
                f"{(shift1 - shift0) / 1e6:.2f} ms over the config")
    except (RuntimeError, subprocess.TimeoutExpired, OSError):
        log(f"{cfg.name}: post-run clock probe failed (VM dead?)")
    pull = subprocess.run(
        ["ssh", *vm.SSH_OPTS, vm.dest, f"sudo tar -C {vm_out} -c ."],
        capture_output=True)
    if pull.returncode == 0 and pull.stdout:
        subprocess.run(["tar", "-C", d, "-x"], input=pull.stdout,
                       capture_output=True)
    else:
        log(f"{cfg.name}: artifact pull FAILED "
            f"({pull.stderr.decode()[:100]})")
    extra = {"vm": True, "vm_ssh_rtt_ns": rtt0, "vm_dut_rc": rc,
             "vm_clock_shift_ns": ((shift0 + shift1) // 2
                                   if shift1 is not None else shift0),
             "vm_clock_shift_start_ns": shift0,
             "vm_clock_shift_end_ns": shift1}
    res_path = os.path.join(d, "vmdut_result.json")
    if cfg.prim == "roc" and os.path.exists(res_path):
        with open(res_path) as f:
            have_mon = json.load(f).get("roc_monitor_vif", False)
        cfg.need_rf = have_mon
        extra["monitor_vif"] = have_mon
        extra["roc_ms"] = cfg.roc_ms
    s.write_meta(cfg, d, cfg.from_ch, cfg.to_ch, extra)
    oa.stop()
    ob.stop()
    log(f"{cfg.name}: done (vm rc={rc})")
    return rc == 0


def run_mcc_attempt_vm(s: Session, vm: VmDut, vm_iface: str,
                       helper_pid: str):
    """The host run_mcc_attempt with the DUT leg driven over ssh. The
    helper AP (8821AU + hostapd) stays on the host; association is over
    the air, the /proc mcc knobs live in the VM."""
    d = str(s.out_root / "mcc")
    os.makedirs(d, exist_ok=True)
    result = {"prim": "mcc", "status": "attempted", "venue": "vm",
              "steps": []}

    def finish():
        with open(os.path.join(d, "mcc_result.json"), "w") as f:
            json.dump(result, f, indent=2)

    def step(name, ok, detail=""):
        result["steps"].append({"step": name, "ok": ok, "detail": detail})
        log(f"mcc: {name}: {'ok' if ok else 'FAILED'} {detail}")
        finish()  # persist per step — a VM kernel wedge loses nothing
        return ok

    _real_ssh = vm.ssh

    def _safe_ssh(cmd, timeout=30):
        # A wedged VM leaves ssh hanging; convert that into a failed step
        # instead of an unhandled TimeoutExpired.
        try:
            return _real_ssh(cmd, timeout=timeout)
        except subprocess.TimeoutExpired:
            return subprocess.CompletedProcess([], 124, "",
                                               "ssh timeout (VM wedged?)")
    vm.ssh = _safe_ssh

    try:
        helper = find_dut(helper_pid)
    except SystemExit:
        result["status"] = "blocked-hardware-absent"
        step("helper adapter", False, f"{helper_pid} not plugged")
        finish()
        return
    run(["modprobe", "rtw88_8821au"], timeout=30)
    regress.attach_to_host_kernel(helper)
    try:
        dut_iface(helper)
        time.sleep(2.5)          # let udev finish the wlan0→wlxMAC rename
        h_iface = dut_iface(helper)
    except RuntimeError as e:
        step("helper iface", False, str(e))
        result["status"] = "blocked-helper-bringup"
        finish()
        return
    unmanage(h_iface)
    ap = WpaAp(h_iface, ch_to_freq(36), d)
    ok = step("helper AP beaconing on 36", ap.wait_beaconing())
    if ok:
        # Open BSS — iw's own open-system connect avoids needing
        # wpa_supplicant inside the VM.
        vm.ssh(f"sudo ip link set {vm_iface} down; "
               f"sudo iw dev {vm_iface} set type managed; "
               f"sudo ip link set {vm_iface} up; "
               f"sudo iw dev {vm_iface} connect kchansw-bench")
        assoc = False
        end = time.monotonic() + 30
        while time.monotonic() < end:
            r = vm.ssh(f"iw dev {vm_iface} link")
            if "Connected to" in r.stdout:
                assoc = True
                break
            time.sleep(1)
        ok = step("DUT STA associated to helper", assoc)
    if ok:
        # CONCURRENT_MODE builds auto-create the second vif at insmod.
        r = vm.ssh(f"ls /sys/class/net | grep -v -e lo -e '^en' "
                   f"| grep -v '^{vm_iface}$' | head -1")
        second = r.stdout.strip()
        if not second:
            r = vm.ssh(f"sudo iw dev {vm_iface} interface add kcap0 "
                       f"type managed")
            second = "kcap0" if r.returncode == 0 else ""
        ok = step("second vif", bool(second), second or
                  r.stderr.strip()[:100])
    if ok:
        r = vm.ssh(f"ls /proc/net/*/{vm_iface}/mcc/mcc_* 2>/dev/null")
        knobs = r.stdout.split()
        ok = step("mcc proc knobs present", bool(knobs), str(knobs[:3]))
        if ok:
            base = os.path.dirname(knobs[0])
            r = vm.ssh(f"echo 1 | sudo tee {base}/mcc_enable")
            step("mcc_enable=1", r.returncode == 0, r.stderr.strip()[:80])
            for k in ("mcc_info", "mcc_policy_table"):
                r = vm.ssh(f"sudo cat {base}/{k} 2>/dev/null")
                if r.stdout:
                    result[k] = r.stdout[:4000]
    if not ok:
        result["status"] = "blocked-see-steps"
    finish()
    vm.ssh(f"sudo pkill wpa_supplicant; sudo iw dev kcap0 del 2>/dev/null; "
           f"true")
    ap.stop()
    s.dmesg_dump(d)


def cmd_phase_b_vm(s: Session, a):
    if not a.vm_ssh:
        raise SystemExit("phase-b-vm: --vm-ssh user@ip required "
                         "(tests/kchansw_vm.sh status shows the IP)")
    vm = VmDut(a.vm_ssh)
    r = vm.ssh("lsmod | grep -qE '^(88x2bu_ohd|8812bu)' && uname -r")
    if r.returncode != 0:
        raise SystemExit("phase-b-vm: vendor module not loaded in the VM — "
                         "run tests/kchansw_vm.sh prepare first")
    with open(s.out_root / "vm_manifest.json", "w") as f:
        json.dump({"vm_uname": r.stdout.strip(),
                   "vm_ssh": a.vm_ssh,
                   "vm_modinfo": vm.ssh(
                       "modinfo /opt/kchansw/rtl88x2bu/88x2bu_ohd.ko "
                       "| head -8").stdout}, f, indent=2)
    write_manifest(s)
    vm_iface = vm.iface()
    log(f"phase-b-vm: VM kernel {r.stdout.strip()}, DUT iface {vm_iface}")
    configs = [
        Cfg("b-set-36-40", "set_channel", 36, 40, a.switches),
        Cfg("b-roc-36-40", "roc", 36, 40, a.switches_aux,
            rdev_entry="rdev_remain_on_channel"),
    ]
    for cfg in configs:
        if a.primitive not in ("all",) and a.primitive not in cfg.name \
                and a.primitive != cfg.prim:
            continue
        if not run_vm_config(s, vm, cfg, vm_iface):
            log(f"{cfg.name}: vm run reported failure — continuing")
        if not iface_alive_vm(vm, vm_iface):
            log("phase-b-vm: VM DUT iface dead — stopping")
            return
    if a.primitive in ("all", "mcc"):
        log("phase-b-vm: rebuilding VARIANT=mcc in VM for the MCC attempt")
        r = subprocess.run(["tests/kchansw_vm.sh", "mcc"], cwd=str(REPO),
                           capture_output=True, text=True, timeout=900)
        if r.returncode != 0:
            log(f"mcc variant build/load failed: {r.stdout[-200:]} "
                f"{r.stderr[-200:]}")
            return
        run_mcc_attempt_vm(s, vm, vm.iface(), a.helper_pid)


def iface_alive_vm(vm: VmDut, iface: str) -> bool:
    r = vm.ssh(f"iw dev {iface} info")
    return r.returncode == 0 and "Interface" in r.stdout


# ---------------------------------------------------------------------------
# Manifest, inventory, smoke.
# ---------------------------------------------------------------------------

def write_manifest(s: Session):
    def sh(cmd):
        try:
            return run(cmd, timeout=15).stdout.strip()
        except Exception as e:
            return f"error: {e}"

    mod = s.driver["module"]
    man = {
        "date_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "uname": sh(["uname", "-a"]),
        "reg": sh(["iw", "reg", "get"]),
        "lsusb": sh(["lsusb"]),
        "lsusb_t": sh(["lsusb", "-t"]),
        "driver": s.driver_key,
        "module": mod,
        "modinfo_srcversion": sh(["modinfo", "-F", "srcversion", mod]),
        "modinfo_filename": sh(["modinfo", "-F", "filename", mod]),
        "devourer_sha": sh(["git", "-C", str(REPO), "rev-parse", "HEAD"]),
        "roles": {
            "dut": dataclasses.asdict(s.dut),
            "oracle_a": dataclasses.asdict(s.oracle_a_dut),
            "oracle_b": dataclasses.asdict(s.oracle_b_dut),
        },
        "fw_version_dmesg": "\n".join(
            ln for ln in sh(["journalctl", "-k", "--no-pager", "-n", "2000"])
            .splitlines() if re.search(r"[Ff]irmware.*(rtw|version)", ln))[-2000:],
        "vbus_map": os.environ.get("REGRESS_VBUS_MAP", ""),
    }
    with open(s.out_root / "manifest.json", "w") as f:
        json.dump(man, f, indent=2)


def cmd_inventory(s: Session):
    inv = {
        "tools": {t: bool(shutil.which(t)) for t in
                  ("iw", "wpa_supplicant", "wpa_cli", "hostapd", "uhubctl",
                   "tcpdump", "journalctl")},
        "modules": {},
        "trace_events": kchansw_trace.discover_events(),
        "kprobe_symbols": {},
    }
    for key, spec in DRIVER_SETS.items():
        mod = spec["module"]
        r = run(["modinfo", "-F", "filename", mod], timeout=10)
        inv["modules"][mod] = r.stdout.strip() if r.returncode == 0 else "absent"
    iface = None
    try:
        iface = s.dut_up()
        phy = iface_phy(iface)
        r = run(["iw", "phy", phy, "info"], timeout=10)
        (s.out_root / "iw_phy_info.txt").write_text(r.stdout)
        inv["iface"] = iface
        inv["iface_modes"] = re.findall(r"\* (\w[\w/-]*)", r.stdout.split(
            "Supported interface modes:")[1].split("Band")[0]) \
            if "Supported interface modes:" in r.stdout else []
        combos = ""
        if "valid interface combinations:" in r.stdout:
            combos = r.stdout.split("valid interface combinations:")[1][:600]
        inv["iface_combinations"] = combos
        inv["kprobe_symbols"] = kchansw_trace.kallsyms_present(
            sorted({sym for _, sym, _ in s.driver["probes"]}))
    except Exception as e:
        inv["dut_error"] = str(e)
    (s.out_root / "inventory.json").write_text(json.dumps(inv, indent=2))
    print(json.dumps(inv, indent=2))
    write_manifest(s)
    log(f"inventory → {s.out_root}/inventory.json")


def cmd_smoke(s: Session):
    iface = s.dut_up()
    write_manifest(s)
    cfg_i = Cfg(name="smoke-instrumented", prim="set_channel",
                from_ch=36, to_ch=40, switches=20, warmup=2)
    w_inst, iface = run_set_channel(s, cfg_i, iface)
    cfg_u = Cfg(name="smoke-bare", prim="set_channel",
                from_ch=36, to_ch=40, switches=20, warmup=2)
    w_bare, iface = run_set_channel(s, cfg_u, iface, uninstrumented=True)
    import statistics
    med_i = statistics.median(w_inst) / 1e6 if w_inst else 0
    med_b = statistics.median(w_bare) / 1e6 if w_bare else 0
    log(f"smoke: per-switch iw wall median instrumented={med_i:.2f} ms "
        f"bare={med_b:.2f} ms delta={med_i - med_b:+.2f} ms")
    log(f"now run: tests/.venv/bin/python tests/kchansw_analyze.py {s.out_root}")


# ---------------------------------------------------------------------------
# Phases.
# ---------------------------------------------------------------------------

def phase_a_configs(a) -> list:
    return [
        # Headline endurance config: directions alternate, so ×2 yields the
        # issue's ≥1000 switches in EACH direction.
        Cfg("a-set-36-40", "set_channel", 36, 40, 2 * a.switches),
        Cfg("a-set-1-6", "set_channel", 1, 6, a.switches_aux),
        Cfg("a-set-ht40-36-44", "set_channel", 36, 44, a.switches_aux,
            ht="HT40+"),
        Cfg("a-set-xband-36-6", "set_channel", 36, 6, a.switches_crossband),
        Cfg("a-scan-40", "scan", 36, 40, a.switches_aux, need_rf=False,
            fingerprint="sa", rdev_entry="rdev_scan"),
        Cfg("a-csa-36-40", "csa", 36, 40, a.switches, fingerprint="sa",
            rdev_entry="rdev_channel_switch"),
        Cfg("a-csa-1-6", "csa", 1, 6, 100, fingerprint="sa",
            rdev_entry="rdev_channel_switch"),
        Cfg("a-roc-36-40", "roc", 36, 40, a.switches,
            rdev_entry="rdev_remain_on_channel"),
        Cfg("a-roc-40-36", "roc", 40, 36, a.switches_aux,
            rdev_entry="rdev_remain_on_channel"),
    ]


def run_phase(s: Session, configs, only: str):
    iface = s.dut_up()
    write_manifest(s)
    probe_tdls(s, iface, s.out_root)
    for cfg in configs:
        if only != "all" and only not in cfg.prim and only not in cfg.name:
            continue
        log(f"=== {cfg.name} ({cfg.prim} {cfg.from_ch}↔{cfg.to_ch} "
            f"×{cfg.switches}) ===")
        # A config can end with the DUT wedged (heavy-TX runs do, every
        # 100-200 switches) — verify and recover before the next bring-up.
        if not iface_alive(iface):
            log("inter-config: DUT unresponsive — recovering")
            iface2, rung = recover_dut(s, cfg.from_ch, start="module-reload")
            if iface2 is None:
                log("inter-config recovery exhausted — stopping phase")
                break
            iface = iface2
        try:
            if cfg.prim == "set_channel":
                _, iface = run_set_channel(s, cfg, iface)
            elif cfg.prim == "csa":
                run_csa(s, cfg, iface)
                set_managed_idle(iface)
            elif cfg.prim == "roc":
                run_roc(s, cfg, iface)
            elif cfg.prim == "scan":
                run_scan(s, cfg, iface)
        except Exception as e:
            import traceback
            log(f"{cfg.name}: EXCEPTION {e!r} — continuing with next config")
            log(traceback.format_exc())
    log(f"phase done → analyze with: tests/.venv/bin/python "
        f"tests/kchansw_analyze.py {s.out_root}")


def phase_c_guard(fn):
    """Lift the kestrel rtw89 blacklist for the run; restore byte-identical."""
    def wrapper(s: Session, *a, **kw):
        held = False
        if os.path.exists(BLACKLIST_KESTREL):
            os.rename(BLACKLIST_KESTREL, BLACKLIST_HOLD)
            held = True
            log(f"lifted {BLACKLIST_KESTREL} (held aside)")
        try:
            return fn(s, *a, **kw)
        finally:
            if held:
                os.rename(BLACKLIST_HOLD, BLACKLIST_KESTREL)
                log(f"restored {BLACKLIST_KESTREL}")
            for mod in ("rtw89_8852bu", "rtw89_8852b", "rtw89_usb"):
                run(["modprobe", "-r", mod], timeout=30)
            regress.usb_port_power_cycle(s.dut, settle_s=3.0)
            log("phase-c: rtw89 modules removed, DUT power-cycled")
    return wrapper


@phase_c_guard
def cmd_phase_c(s: Session, a):
    configs = [
        Cfg("c-set-36-40", "set_channel", 36, 40, a.switches_aux),
        Cfg("c-csa-36-40", "csa", 36, 40, a.switches_crossband * 4,
            fingerprint="sa", rdev_entry="rdev_channel_switch"),
        Cfg("c-roc-36-40", "roc", 36, 40, a.switches_aux,
            rdev_entry="rdev_remain_on_channel"),
    ]
    run_phase(s, configs, a.primitive)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("cmd", choices=["inventory", "smoke", "phase-a", "cold",
                                    "phase-b", "phase-b-vm", "phase-c"])
    ap.add_argument("--vm-ssh", default="",
                    help="user@ip of the prepared VM (phase-b-vm)")
    ap.add_argument("--primitive", default="all",
                    help="filter configs by primitive/name substring")
    ap.add_argument("--dut-pid", default="2357:012d")
    ap.add_argument("--oracle-a-pid", default="2357:0120",
                help="8821AU: on this rig the 8812AU position "
                     "decodes <10%% of near-field injector frames")
    ap.add_argument("--oracle-b-pid", default="0bda:c812")
    ap.add_argument("--helper-pid", default="2357:0120",
                    help="helper AP adapter for the MCC attempt (8821AU)")
    ap.add_argument("--switches", type=int, default=1000)
    ap.add_argument("--switches-aux", type=int, default=300)
    ap.add_argument("--switches-crossband", type=int, default=50)
    ap.add_argument("--cold-trials", type=int, default=10)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    if os.geteuid() != 0:
        sys.stderr.write("kchansw_bench: needs root (sysfs/tracefs/iw)\n")
        return 2
    if not RXDEMO.is_file():
        sys.stderr.write(f"missing {RXDEMO} — build first\n")
        return 2
    if not args.out:
        args.out = f"/tmp/devourer-kchansw-{time.strftime('%Y%m%d-%H%M%S')}-" \
                   f"{args.cmd}"

    regress._install_cleanup_handlers()
    driver_key = {"phase-b": "vendor", "phase-b-vm": "vendor",
                  "phase-c": "rtw89"}.get(args.cmd, "rtw88")
    if args.cmd == "phase-c":
        args.dut_pid = "35bc:0108"
    s = Session(args, driver_key)
    global _LOG_FILE
    _LOG_FILE = open(s.out_root / "bench.log", "a")
    log(f"out dir: {s.out_root}  driver: {driver_key}  DUT: {s.dut.vidpid} "
        f"({s.dut.chipset})")

    if args.cmd == "inventory":
        cmd_inventory(s)
    elif args.cmd == "smoke":
        cmd_smoke(s)
    elif args.cmd == "phase-a":
        run_phase(s, phase_a_configs(args), args.primitive)
    elif args.cmd == "cold":
        iface = s.dut_up()
        write_manifest(s)
        set_monitor(iface, 36)
        run_cold(s, args.cold_trials)
    elif args.cmd == "phase-b":
        # Kprobing the vendor module on the host deadlocked the host kernel
        # (power cycle to recover) — twice. Use phase-b-vm.
        if os.environ.get("KCHANSW_HOST_VENDOR_OK") != "1":
            sys.stderr.write(
                "phase-b (host insmod) hangs the whole machine — use "
                "phase-b-vm (tests/kchansw_vm.sh prepare). Set "
                "KCHANSW_HOST_VENDOR_OK=1 to override.\n")
            return 2
        # Module must already be inserted by kchansw_vendor_build.sh.
        if run(["sh", "-c", "lsmod | grep -q '^88x2bu_ohd'"]).returncode != 0:
            sys.stderr.write("phase-b: vendor module 88x2bu_ohd not loaded — "
                             "run tests/kchansw_vendor_build.sh load first\n")
            return 2
        configs = [
            Cfg("b-set-36-40", "set_channel", 36, 40, args.switches),
            Cfg("b-roc-36-40", "roc", 36, 40, args.switches_aux,
                rdev_entry="rdev_remain_on_channel"),
        ]
        run_phase(s, configs, args.primitive)
        if args.primitive in ("all", "mcc"):
            iface = dut_iface(s.dut)
            run_mcc_attempt(s, iface, args.helper_pid)
    elif args.cmd == "phase-b-vm":
        cmd_phase_b_vm(s, args)
    elif args.cmd == "phase-c":
        cmd_phase_c(s, args)
    # Hand the artifacts to the invoking user so the (non-root) analyzer can
    # write summaries next to them.
    sudo_user = os.environ.get("SUDO_USER")
    if sudo_user:
        subprocess.run(["chown", "-R", f"{sudo_user}:", str(s.out_root)],
                       capture_output=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
