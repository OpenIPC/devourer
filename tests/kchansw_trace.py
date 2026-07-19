#!/usr/bin/env python3
"""tracefs session for the kernel channel-switch bench (no trace-cmd needed).

Owns everything under /sys/kernel/tracing for a bench run:

  - selects the `mono` trace clock so ftrace timestamps share a timeline with
    python time.monotonic_ns() and the bench's stamped oracle/injector JSONL;
  - enables a curated set of cfg80211/mac80211 trace events (runtime-discovered
    — absent names become capability-inventory rows, not errors);
  - registers kretprobe/kprobe pairs on driver symbols under the private
    `kchansw/` group (symbols checked in /proc/kallsyms first; a missing
    symbol degrades to an inventory row);
  - streams trace_pipe to <out_dir>/trace.txt from a reader thread (consuming
    the pipe keeps the ring empty, so no overflow accounting);
  - exposes trace_marker writes for per-switch segmentation markers;
  - restores every knob it touched (clock, buffer size, events, probes) via
    context-manager exit + atexit + signal hooks, and `force_clean()` heals
    the leftovers of a crashed previous run.

Standalone: `sudo python3 tests/kchansw_trace.py --inventory` prints the
event/symbol availability map as JSON and exits (no state left behind).
"""

import argparse
import atexit
import json
import os
import select
import signal
import sys
import threading
import time

TRACEFS = "/sys/kernel/tracing"

# Curated event want-list. Names are checked against the live tracefs event
# tree; whatever is absent lands in the inventory as event-absent and the
# analyzer treats its columns as nullable.
WANT_EVENTS = [
    # cfg80211: the nl80211-request edge (host-command latency = entry→return).
    "cfg80211/rdev_set_monitor_channel",
    "cfg80211/rdev_channel_switch",
    "cfg80211/rdev_remain_on_channel",
    "cfg80211/rdev_cancel_remain_on_channel",
    "cfg80211/rdev_mgmt_tx",
    "cfg80211/rdev_scan",
    "cfg80211/rdev_return_int",
    "cfg80211/rdev_return_void",
    "cfg80211/cfg80211_ready_on_channel",
    "cfg80211/cfg80211_ready_on_channel_expired",
    "cfg80211/cfg80211_ch_switch_notify",
    "cfg80211/cfg80211_ch_switch_started_notify",
    "cfg80211/cfg80211_scan_done",
    # mac80211: driver-ops edge + queue behavior around the switch.
    "mac80211/drv_config",
    "mac80211/drv_return_void",
    "mac80211/drv_return_int",
    "mac80211/drv_remain_on_channel",
    "mac80211/drv_cancel_remain_on_channel",
    "mac80211/drv_hw_scan",
    "mac80211/drv_sw_scan_start",
    "mac80211/drv_sw_scan_complete",
    "mac80211/drv_flush",
    "mac80211/drv_start_ap",
    "mac80211/drv_stop_ap",
    "mac80211/drv_add_chanctx",
    "mac80211/drv_remove_chanctx",
    "mac80211/drv_change_chanctx",
    "mac80211/drv_assign_vif_chanctx",
    "mac80211/drv_unassign_vif_chanctx",
    "mac80211/drv_switch_vif_chanctx",
    "mac80211/drv_tdls_channel_switch",
    "mac80211/stop_queue",
    "mac80211/wake_queue",
]

# Driver kprobe candidates per phase. (probe_name, symbol, with_retprobe).
# Names carry a kc_ prefix because trace lines print only the event name,
# not the kchansw/ group — the prefix keeps them unambiguous to the parser.
# The bench writes each config's probe_name→role mapping into meta.json so
# the analyzer stays driver-agnostic ("drv"/"chip"/"h2c" roles).
RTW88_PROBES = [
    ("kc_ops_config", "rtw_ops_config", True),
    ("kc_drv_set_channel", "rtw_set_channel", True),
    ("kc_chip_set_channel", "rtw8822b_set_channel", True),
    ("kc_h2c", "rtw_fw_send_h2c_command", True),
]
VENDOR_PROBES = [
    ("kc_drv_set_channel", "rtw_set_chbw_cmd", True),
    ("kc_hal_set_chnl_bw", "rtw_hal_set_chnl_bw", True),
    ("kc_chip_set_channel", "phy_SwChnlAndSetBwMode8822B", True),
    ("kc_h2c", "rtw_hal_fill_h2c_cmd", True),
    ("kc_tdls_ch_sw_offload", "rtw_hal_ch_sw_oper_offload", True),
]
RTW89_PROBES = [
    ("kc_drv_set_channel", "rtw89_set_channel", True),
    ("kc_chip_set_channel", "rtw89_chip_set_channel", True),
    ("kc_roc_start", "rtw89_roc_start", True),
    ("kc_roc_end", "rtw89_roc_end", True),
    ("kc_h2c", "rtw89_h2c_tx", True),
]


def _p(*parts: str) -> str:
    return os.path.join(TRACEFS, *parts)


def _read(path: str) -> str:
    with open(path, "r") as f:
        return f.read()


def _write(path: str, val: str) -> None:
    with open(path, "w") as f:
        f.write(val)


def _append(path: str, line: str) -> None:
    """Append one line to a tracefs control file. Python's text-mode "a"
    lseeks to EOF at open, which tracefs rejects with EINVAL — use a raw
    O_APPEND fd like the shell's >> does."""
    fd = os.open(path, os.O_WRONLY | os.O_APPEND)
    try:
        os.write(fd, line.encode())
    finally:
        os.close(fd)


def kallsyms_present(symbols: list) -> dict:
    """Which of `symbols` exist in /proc/kallsyms (exact-match, text syms)."""
    want = set(symbols)
    found = set()
    with open("/proc/kallsyms", "r") as f:
        for line in f:
            parts = line.split()
            if len(parts) >= 3:
                name = parts[2].split("\t")[0]
                if name in want:
                    found.add(name)
    return {s: (s in found) for s in symbols}


def discover_events(want=WANT_EVENTS) -> dict:
    """Map each want-list event to present/absent in the live event tree."""
    out = {}
    for ev in want:
        out[ev] = os.path.isdir(_p("events", *ev.split("/")))
    return out


def force_clean() -> None:
    """Remove any stale kchansw/ probes and disable their group. Idempotent;
    safe to call when no prior state exists."""
    kev = _p("kprobe_events")
    if not os.path.exists(kev):
        return
    try:
        _write(_p("events", "kchansw", "enable"), "0")
    except OSError:
        pass
    stale = []
    for line in _read(kev).splitlines():
        # Lines look like: p:kchansw/set_channel rtw_set_channel
        if ":kchansw/" in line:
            name = line.split()[0].split(":", 1)[1]
            stale.append(name)
    for name in stale:
        try:
            _append(kev, f"-:{name}\n")
        except OSError as e:
            sys.stderr.write(f"kchansw_trace: force_clean -:{name}: {e}\n")


class TraceSession:
    """Context manager owning one instrumented tracing window."""

    def __init__(self, out_dir: str, kprobes=None, want_events=WANT_EVENTS,
                 buffer_kb: int = 8192):
        self.out_dir = out_dir
        self.kprobes = list(kprobes or [])
        self.want_events = list(want_events)
        self.buffer_kb = buffer_kb
        self.inventory = {"events": {}, "kprobes": {}}
        self._saved = {}
        self._enabled_events = []
        self._registered_probes = []
        self._marker_f = None
        self._pipe_fd = None
        self._reader = None
        self._stop = threading.Event()
        self._trace_path = os.path.join(out_dir, "trace.txt")
        self._active = False

    # -- lifecycle -----------------------------------------------------------

    def __enter__(self) -> "TraceSession":
        os.makedirs(self.out_dir, exist_ok=True)
        force_clean()
        self._saved["clock"] = self._current_clock()
        self._saved["buffer_size_kb"] = _read(_p("buffer_size_kb")).strip()
        self._saved["tracing_on"] = _read(_p("tracing_on")).strip()
        _write(_p("tracing_on"), "0")
        _write(_p("trace_clock"), "mono")
        _write(_p("buffer_size_kb"), str(self.buffer_kb))
        _write(_p("trace"), "")  # clear ring
        self.inventory["events"] = discover_events(self.want_events)
        for ev, present in self.inventory["events"].items():
            if not present:
                continue
            path = _p("events", *ev.split("/"), "enable")
            try:
                _write(path, "1")
                self._enabled_events.append(path)
            except OSError as e:
                self.inventory["events"][ev] = f"enable-failed: {e}"
        self._register_kprobes()
        self._marker_f = open(_p("trace_marker"), "w")
        self._pipe_fd = os.open(_p("trace_pipe"), os.O_RDONLY | os.O_NONBLOCK)
        self._reader = threading.Thread(target=self._pump, daemon=True)
        self._reader.start()
        _write(_p("tracing_on"), "1")
        self._active = True
        atexit.register(self._teardown)
        return self

    def __exit__(self, *exc) -> None:
        self._teardown()

    def _teardown(self) -> None:
        if not self._active:
            return
        self._active = False
        # Give the last events time to reach the ring, then let the pump
        # drain what's left before switching tracing off.
        time.sleep(0.5)
        self._stop.set()
        if self._reader:
            self._reader.join(timeout=5.0)
        try:
            _write(_p("tracing_on"), "0")
        except OSError:
            pass
        for path in self._enabled_events:
            try:
                _write(path, "0")
            except OSError:
                pass
        if self._registered_probes:
            # Disarm the group before removal — removing a still-enabled
            # kretprobe returns EBUSY.
            try:
                _write(_p("events", "kchansw", "enable"), "0")
            except OSError:
                pass
        for name in self._registered_probes:
            for attempt in (0.0, 0.2, 0.5):
                time.sleep(attempt)
                try:
                    _append(_p("kprobe_events"), f"-:{name}\n")
                    break
                except OSError as e:
                    if attempt == 0.5:
                        sys.stderr.write(f"kchansw_trace: remove {name}: {e}\n")
        for key, path in (("clock", "trace_clock"),
                          ("buffer_size_kb", "buffer_size_kb"),
                          ("tracing_on", "tracing_on")):
            try:
                _write(_p(path), self._saved[key])
            except OSError:
                pass
        if self._marker_f:
            self._marker_f.close()
            self._marker_f = None
        if self._pipe_fd is not None:
            os.close(self._pipe_fd)
            self._pipe_fd = None

    # -- internals -----------------------------------------------------------

    @staticmethod
    def _current_clock() -> str:
        # trace_clock reads like: "[local] global counter uptime ... mono ..."
        for tok in _read(_p("trace_clock")).split():
            if tok.startswith("[") and tok.endswith("]"):
                return tok[1:-1]
        return "local"

    def _register_kprobes(self) -> None:
        syms = kallsyms_present([s for _, s, _ in self.kprobes])
        for name, sym, want_ret in self.kprobes:
            if not syms.get(sym):
                self.inventory["kprobes"][name] = f"symbol-absent: {sym}"
                continue
            try:
                _append(_p("kprobe_events"), f"p:kchansw/{name} {sym}\n")
                self._registered_probes.append(f"kchansw/{name}")
                if want_ret:
                    _append(_p("kprobe_events"), f"r:kchansw/{name}_ret {sym}\n")
                    self._registered_probes.append(f"kchansw/{name}_ret")
                self.inventory["kprobes"][name] = "ok"
            except OSError as e:
                self.inventory["kprobes"][name] = f"register-failed: {e}"
        if self._registered_probes:
            _write(_p("events", "kchansw", "enable"), "1")

    def _pump(self) -> None:
        with open(self._trace_path, "wb") as out:
            while True:
                r, _, _ = select.select([self._pipe_fd], [], [], 0.2)
                if r:
                    try:
                        chunk = os.read(self._pipe_fd, 1 << 16)
                    except BlockingIOError:
                        chunk = b""
                    if chunk:
                        out.write(chunk)
                        continue
                if self._stop.is_set():
                    # Final drain: anything that raced the stop flag.
                    while True:
                        r, _, _ = select.select([self._pipe_fd], [], [], 0.1)
                        if not r:
                            break
                        try:
                            chunk = os.read(self._pipe_fd, 1 << 16)
                        except BlockingIOError:
                            break
                        if not chunk:
                            break
                        out.write(chunk)
                    out.flush()
                    return

    # -- API -----------------------------------------------------------------

    def marker(self, text: str) -> None:
        """Write a segmentation marker; shows in the trace as
        tracing_mark_write: <text>."""
        self._marker_f.write(text + "\n")
        self._marker_f.flush()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--inventory", action="store_true",
                    help="print event/symbol availability as JSON and exit")
    ap.add_argument("--force-clean", action="store_true",
                    help="remove stale kchansw/ probes from a crashed run")
    args = ap.parse_args()
    if os.geteuid() != 0:
        sys.stderr.write("kchansw_trace: needs root (tracefs)\n")
        return 2
    if args.force_clean:
        force_clean()
        print("kchansw_trace: cleaned")
        return 0
    if args.inventory:
        all_syms = {s for probes in (RTW88_PROBES, VENDOR_PROBES, RTW89_PROBES)
                    for _, s, _ in probes}
        print(json.dumps({
            "events": discover_events(),
            "symbols": kallsyms_present(sorted(all_syms)),
        }, indent=2))
        return 0
    ap.print_help()
    return 1


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
    sys.exit(main())
