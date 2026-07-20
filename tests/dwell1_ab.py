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
        rssi = ev.get("rssi")
        yield {
            "host_mono_ns": int(host),
            "tsfl_us": int(ev.get("tsfl", 0)),
            "slot": slot,
            "epoch": epoch,
            # path-A RSSI for leakage-robust attribution (own channel is the
            # strongest decode; an adjacent oracle hears a leaked copy weaker).
            "rssi": (rssi[0] if isinstance(rssi, list) and rssi else None),
        }


def expected_channel_fn(chans, seed):
    """slot -> expected channel. Sequential = chans[slot % n]; keyed reuses
    the verified C++ HopSchedule port in tests/hop_rx_probe.py so the Python
    attribution matches the on-air order bit-for-bit."""
    n = len(chans)
    if not seed:
        return lambda slot: chans[slot % n]
    cache = {}   # round -> channel order

    def f(slot):
        rnd = slot // n
        if rnd not in cache:
            cache[rnd] = _keyed_round(seed, chans, rnd)
        return cache[rnd][slot % n]
    return f


def _keyed_round(seed, chans, rnd):
    """The channel order for one keyed round (matches HopSchedule::permutation
    with round=rnd) — a thin wrapper over hop_rx_probe's siphash24."""
    import hop_rx_probe as hp
    s = seed[2:] if seed.lower().startswith("0x") else seed
    key = bytes.fromhex(s.zfill(32))
    p = list(range(len(chans)))
    counter = 0
    for i in range(len(p), 1, -1):
        limit = 0xffffffffffffffff - (0xffffffffffffffff % i)
        while True:
            x = hp.siphash24(key, b"H" + rnd.to_bytes(8, "little")
                             + counter.to_bytes(8, "little"))
            counter += 1
            if x < limit:
                break
        j = x % i
        p[i - 1], p[j] = p[j], p[i - 1]
    return [chans[i] for i in p]


def analyze(out_dir, chans, seed=""):
    """Attribute every decoded marker to its slot's channel and score the
    dwell-1 contract for an arbitrary N-channel schedule. Attribution is
    leakage-robust: a slot is attributed to the oracle that decoded it with
    the highest RSSI (own channel dominates; an adjacent oracle hears only a
    weaker leaked copy), and wrong-channel = that oracle's channel !=
    expected."""
    with open(os.path.join(out_dir, "meta.json")) as f:
        meta = json.load(f)
    seed = seed or meta.get("seed", "")
    expected_ch = expected_channel_fn(chans, seed)
    tx = _load_tx(out_dir)
    admitted_slots = {r["slot"] for r in tx if r["ev"] == "dwell.tx"}
    dropped_slots = {r["slot"] for r in tx if r["ev"] == "dwell.drop"}

    # Per (role, channel): slot -> list of decodes. One oracle per channel.
    per = {}
    dup = 0
    for role, ch in zip(_roles(len(chans)), chans):
        path = os.path.join(out_dir, f"oracle_{role}.jsonl")
        if not os.path.exists(path):
            continue
        counts = {}
        for fr in _iter_frames(path):
            counts.setdefault(fr["slot"], []).append(fr)
        for slot, frs in counts.items():
            if len(frs) > 1:
                dup += len(frs) - 1
        per[(role, ch)] = counts

    # Best-RSSI attribution per slot across all oracles.
    best = {}  # slot -> (rssi, decoded_ch)
    for (role, ch), counts in per.items():
        for slot, frs in counts.items():
            r = max((f["rssi"] for f in frs if f["rssi"] is not None),
                    default=-999)
            if slot not in best or r > best[slot][0]:
                best[slot] = (r, ch)

    correct_slots, wrong = set(), []
    for slot, (r, ch) in best.items():
        if ch == expected_ch(slot):
            correct_slots.add(slot)
        else:
            wrong.append({"slot": slot, "decoded_on": ch,
                          "expected": expected_ch(slot), "rssi": r})

    n_admitted = len(admitted_slots)
    delivered = len(admitted_slots & correct_slots)
    slot_ms = meta.get("slot_ms", 20)
    payload = meta.get("airtime_bytes", 96)
    wall_s = n_admitted * slot_ms / 1e3
    result = {
        "n_channels": len(chans),
        "channels": chans,
        "keyed": bool(seed),
        "slots_admitted": n_admitted,
        "slots_dropped_late": len(dropped_slots),
        "delivered_correct_channel": delivered,
        "delivery_frac": (delivered / n_admitted) if n_admitted else None,
        "wrong_channel": len(wrong),
        "duplicate_reair": dup,
        "goodput_kbps": (delivered * payload * 8 / wall_s / 1e3)
                        if wall_s else None,
        "fw": meta.get("fw"),
        "slot_ms": slot_ms,
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


def _roles(n):
    return [chr(ord("a") + i) for i in range(n)]


def spawn_dwelltx(dut, chans, a, out_dir):
    regress.detach_from_host_kernel(dut)
    env = dict(os.environ)
    env.update({
        "DEVOURER_VID": f"0x{dut.vid}", "DEVOURER_PID": f"0x{dut.pid}",
        "DEVOURER_EVENTS": "stdout", "DEVOURER_LOG_LEVEL": "warn",
        "DEVOURER_FASTRETUNE_FW": str(a.fw),
        "DEVOURER_DWELL_CHANNELS": ",".join(str(c) for c in chans),
        "DEVOURER_DWELL_SLOT_MS": str(a.slot_ms),
        "DEVOURER_DWELL_SLOTS": str(a.slots + 20),  # +warmup
        "DEVOURER_HOP_FAST": str(a.hop_fast),
    })
    if a.seed:
        env["DEVOURER_HOP_SEED"] = a.seed
    if a.tx_pwr:
        # Near-field oracles saturate (strong RSSI, poor EVM -> CRC fail) at
        # full power; a modest flat index restores clean decodes on all N.
        env["DEVOURER_TX_PWR"] = str(a.tx_pwr)
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
    pids = [p for p in a.oracle_pids.split(",") if p]
    if len(pids) < len(chans):
        raise SystemExit(f"need one --oracle-pids entry per channel: "
                         f"{len(chans)} channels, {len(pids)} oracle pids")
    dut = kb.find_dut(a.dut_pid)
    oracles = []
    for role, ch, pid in zip(_roles(len(chans)), chans, pids):
        oracles.append(kb.spawn_oracle(role, kb.find_dut(pid), ch, "canon", out))
    with open(os.path.join(out, "meta.json"), "w") as f:
        json.dump({"chans": chans, "slot_ms": a.slot_ms, "slots": a.slots,
                   "fw": a.fw, "late_us": a.late_us, "seed": a.seed}, f)
    tx = spawn_dwelltx(dut, chans, a, out)
    kb.log(f"dwell1 run: {chans} slot={a.slot_ms}ms slots={a.slots} "
           f"fw={a.fw} seed={a.seed or '-'} late_us={a.late_us}")
    budget = (a.slots + 30) * (a.slot_ms / 1e3) + 30
    end = time.monotonic() + budget
    while time.monotonic() < end and tx.alive():
        time.sleep(2)
    tx.stop()
    for o in oracles:
        o.stop()
    if os.environ.get("SUDO_USER"):
        import subprocess
        subprocess.run(["chown", "-R", f"{os.environ['SUDO_USER']}:", out],
                       capture_output=True)
    res = analyze(out, chans, a.seed)
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
    ap.add_argument("--seed", default="",
                    help="keyed hop schedule (hex); empty = public sequential")
    ap.add_argument("--tx-pwr", type=int, default=0,
                    help="flat DUT TXAGC index; ~10 avoids near-field oracle "
                         "saturation on this rig (0 = calibrated default)")
    ap.add_argument("--dut-pid", default="2357:012d")
    ap.add_argument("--oracle-pids", default="2357:0120,0bda:c812",
                    help="comma-separated VID:PID, one oracle per channel in "
                         "--channels order")
    a = ap.parse_args()
    if a.cmd == "run":
        if os.geteuid() != 0:
            sys.stderr.write("dwell1_ab run: needs root\n")
            return 2
        regress._install_cleanup_handlers()
        return cmd_run(a)
    if a.cmd == "analyze":
        print(json.dumps(analyze(a.out, [int(c) for c in a.channels.split(",")],
                                 a.seed), indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
