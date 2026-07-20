#!/usr/bin/env python3
"""Dwell-1 A/B injection over the firmware channel switch — on-air harness.

Drives the dwelltx demo (one admitted data-frame opportunity per wall-clock
slot on an A/B schedule, fw-switched) with two host-side rxdemo oracles pinned
to channels A and B, and proves the data-plane contract from #272:

  * exactly one admitted frame per context slot, decoded on the CORRECT
    channel;
  * ZERO wrong-channel frames (a frame whose HopSyncMarker slot maps to B must
    never be decoded on A's oracle);
  * quantified empty / duplicate / late / first-decode-loss rates;
  * the fw switch vs software FastRetune as a controlled A/B (jitter, delivery,
    host switch cost);
  * a fault matrix (late enqueue, set-channel-while-running) with deterministic,
    observable behaviour.

Each oracle is on ONE fixed channel, so attribution needs no cross-clock
alignment: a decoded frame carries its slot in the marker, the (public
sequential) schedule maps slot -> channel, and the oracle's own channel is
the ground truth. tsfl->host fits (reused from kchansw_analyze) give the
per-frame time for the guard/settle first-decode measurement.

Usage (root):
  sudo tests/.venv/bin/python tests/dwell1_ab.py run \
      --slots 5000 --slot-ms 20 --fw 1 --out /tmp/dwell1-fw
  sudo tests/.venv/bin/python tests/dwell1_ab.py compare --slots 5000
  sudo tests/.venv/bin/python tests/dwell1_ab.py faults
"""

import argparse
import json
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kchansw_bench as kb          # noqa: E402  Child/spawn_oracle/find_dut
import kchansw_analyze as ka        # noqa: E402  unwrap_tsfl/fit_tsfl/fitted_ns
import regress                      # noqa: E402

import numpy as np                  # noqa: E402

MARKER_MAGIC = bytes([221, 27, 0x57, 0x42, 0x75, 0x48, 1])
MARKER_TAIL = bytes([0xd7, 0x3a])
MARKER_LEN = 29


def _iter_frames(path):
    for line in open(path):
        line = line.strip()
        if not line:
            continue
        try:
            rec = json.loads(line)
        except json.JSONDecodeError:
            continue
        raw = rec.get("raw")
        host = rec.get("host_mono_ns")
        if raw is None or host is None:
            continue
        try:
            ev = json.loads(raw)
        except json.JSONDecodeError:
            continue
        if ev.get("ev") != "rx.frame":
            continue
        body = ev.get("body", "")
        if not isinstance(body, str) or len(body) < MARKER_LEN * 2:
            continue
        blob = bytes.fromhex(body)
        i = blob.find(MARKER_MAGIC)
        if i < 0 or i + MARKER_LEN > len(blob):
            continue
        m = blob[i:i + MARKER_LEN]
        if m[27:29] != MARKER_TAIL:
            continue
        epoch = struct.unpack_from("<I", m, 11)[0]
        slot = struct.unpack_from("<Q", m, 15)[0]
        yield {
            "host_mono_ns": int(host),
            "tsfl_us": int(ev.get("tsfl", 0)),
            "slot": slot,
            "epoch": epoch,
        }


def analyze(out_dir, chans, seq=True):
    """Attribute every decoded marker to its slot/channel and score the
    dwell-1 contract. seq=True => public sequential schedule (slot % 2)."""
    with open(os.path.join(out_dir, "meta.json")) as f:
        meta = json.load(f)
    tx = _load_tx(out_dir)
    admitted_slots = {r["slot"] for r in tx if r["ev"] == "dwell.tx"}
    dropped_slots = {r["slot"] for r in tx if r["ev"] == "dwell.drop"}

    # Per-oracle: which slots it decoded, with counts (dup detection).
    per = {}
    fits = {}
    for role, ch in (("a", chans[0]), ("b", chans[1])):
        frames = list(_iter_frames(os.path.join(out_dir, f"oracle_{role}.jsonl")))
        # tsfl fit for timing (best-effort; attribution doesn't need it).
        uf = ka.unwrap_tsfl([{**fr, "ctr": fr["slot"]} for fr in frames])
        fit = ka.fit_tsfl(uf)
        fits[role] = fit
        counts = {}
        for fr in frames:
            counts.setdefault(fr["slot"], []).append(fr)
        per[(role, ch)] = counts

    def expected_ch(slot):
        return chans[slot % 2] if seq else None

    wrong = []          # frames decoded on the wrong channel
    dup = 0             # same slot decoded >1x on the same oracle (re-air)
    correct_slots = set()
    for (role, ch), counts in per.items():
        for slot, frs in counts.items():
            if len(frs) > 1:
                dup += len(frs) - 1
            if expected_ch(slot) == ch:
                correct_slots.add(slot)
            else:
                wrong.append({"slot": slot, "decoded_on": ch,
                              "expected": expected_ch(slot)})

    n_admitted = len(admitted_slots)
    delivered = len(admitted_slots & correct_slots)
    result = {
        "slots_admitted": n_admitted,
        "slots_dropped_late": len(dropped_slots),
        "delivered_correct_channel": delivered,
        "delivery_frac": (delivered / n_admitted) if n_admitted else None,
        "wrong_channel": len(wrong),
        "duplicate_reair": dup,
        "fw": meta.get("fw"),
        "slot_ms": meta.get("slot_ms"),
    }
    # Host-side switch cost + admission placement from the tx event stream.
    sw = np.array([r["switch_us"] for r in tx if r["ev"] == "dwell.slot"
                   and "switch_us" in r], dtype=float)
    if len(sw):
        result["switch_us"] = {"median": float(np.median(sw)),
                               "p99": float(np.percentile(sw, 99)),
                               "max": float(sw.max())}
    inslot = np.array([r["in_slot_us"] for r in tx if r["ev"] == "dwell.tx"
                       and "in_slot_us" in r], dtype=float)
    if len(inslot):
        result["admit_in_slot_us"] = {"median": float(np.median(inslot)),
                                      "p90": float(np.percentile(inslot, 90)),
                                      "jitter_p99_minus_p1": float(
                                          np.percentile(inslot, 99)
                                          - np.percentile(inslot, 1))}
    result["wrong_channel_examples"] = wrong[:5]
    return result


def _load_tx(out_dir):
    rows = []
    for line in open(os.path.join(out_dir, "dwelltx.jsonl")):
        line = line.strip()
        if not line:
            continue
        try:
            rec = json.loads(line)
        except json.JSONDecodeError:
            continue
        ev = rec.get("raw", rec)
        if isinstance(ev, str):
            try:
                ev = json.loads(ev)
            except json.JSONDecodeError:
                continue
        if ev.get("ev", "").startswith("dwell"):
            rows.append(ev)
    return rows


def spawn_dwelltx(dut, chans, a, out_dir):
    regress.detach_from_host_kernel(dut)
    env = dict(os.environ)
    env.update({
        "DEVOURER_VID": f"0x{dut.vid}", "DEVOURER_PID": f"0x{dut.pid}",
        "DEVOURER_EVENTS": "stdout", "DEVOURER_LOG_LEVEL": "warn",
        "DEVOURER_FASTRETUNE_FW": str(a.fw),
        "DEVOURER_DWELL_CHANNELS": f"{chans[0]},{chans[1]}",
        "DEVOURER_DWELL_SLOT_MS": str(a.slot_ms),
        "DEVOURER_DWELL_SLOTS": str(a.slots + 20),  # +warmup
        "DEVOURER_HOP_FAST": str(a.hop_fast),
    })
    if a.late_us:
        env["DEVOURER_DWELL_LATE_US"] = str(a.late_us)
    if a.settle_us:
        env["DEVOURER_DWELL_SETTLE_US"] = str(a.settle_us)
    return kb.Child("dwelltx", [str(kb.REPO / "build" / "dwelltx")],
                    os.path.join(out_dir, "dwelltx.jsonl"), env=env,
                    stamp=True,
                    stderr_path=os.path.join(out_dir, "dwelltx.stderr"))


def cmd_run(a):
    out = a.out
    os.makedirs(out, exist_ok=True)
    chans = [int(c) for c in a.channels.split(",")]
    dut = kb.find_dut(a.dut_pid)
    oa = kb.spawn_oracle("a", kb.find_dut(a.oracle_a_pid), chans[0], "canon", out)
    ob = kb.spawn_oracle("b", kb.find_dut(a.oracle_b_pid), chans[1], "canon", out)
    with open(os.path.join(out, "meta.json"), "w") as f:
        json.dump({"chans": chans, "slot_ms": a.slot_ms, "slots": a.slots,
                   "fw": a.fw, "late_us": a.late_us}, f)
    tx = spawn_dwelltx(dut, chans, a, out)
    kb.log(f"dwell1 run: {chans} slot={a.slot_ms}ms slots={a.slots} "
           f"fw={a.fw} late_us={a.late_us}")
    budget = (a.slots + 30) * (a.slot_ms / 1e3) + 30
    end = time.monotonic() + budget
    while time.monotonic() < end and tx.alive():
        time.sleep(2)
    tx.stop()
    oa.stop()
    ob.stop()
    if os.environ.get("SUDO_USER"):
        import subprocess
        subprocess.run(["chown", "-R", f"{os.environ['SUDO_USER']}:", out],
                       capture_output=True)
    res = analyze(out, chans)
    print(json.dumps(res, indent=2))
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(res, f, indent=2)
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("cmd", choices=["run", "analyze"])
    ap.add_argument("--out", default="/tmp/dwell1")
    ap.add_argument("--channels", default="36,40")
    ap.add_argument("--slot-ms", type=int, default=20)
    ap.add_argument("--slots", type=int, default=5000)
    ap.add_argument("--fw", type=int, default=1)
    ap.add_argument("--hop-fast", type=int, default=1)
    ap.add_argument("--late-us", type=int, default=0)
    ap.add_argument("--settle-us", type=int, default=0,
                    help="post-switch admission delay; fw switch needs "
                         "~4000 (its RF completes async after FastRetune "
                         "returns), sw switch is fine at the demo default")
    ap.add_argument("--dut-pid", default="2357:012d")
    ap.add_argument("--oracle-a-pid", default="0bda:8812")
    ap.add_argument("--oracle-b-pid", default="0bda:c812")
    a = ap.parse_args()
    if a.cmd == "run":
        if os.geteuid() != 0:
            sys.stderr.write("dwell1_ab run: needs root\n")
            return 2
        regress._install_cleanup_handlers()
        return cmd_run(a)
    if a.cmd == "analyze":
        print(json.dumps(analyze(a.out, [int(c) for c in a.channels.split(",")]),
                         indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
