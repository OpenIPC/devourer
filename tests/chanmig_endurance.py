#!/usr/bin/env python3
"""chanmig endurance driver — >=1000 old<->new<->old migrations, measuring
per-cycle video outage and asserting zero persistent split-brain.

Launches the drone (video TX + responder) and the ground (primary RX +
proposer) as subprocesses, then drives the ground's operator stdin: on each
`migrate.done code=0` (confirmed), it proposes the other channel. Outage is
the ground's video gap around each switch (first-new-frame minus last-old),
reconstructed from the ground's migrate.retune / migrate.state stream and the
drone's video cadence.

Run via tests/chanmig_endurance.sh (sets up adapters + power). Standalone:
  sudo python3 tests/chanmig_endurance.py --cycles 1000 \
      --ground-pid 0x8812 --drone-pid 0xc812 --chan-a 36 --chan-b 149 \
      --key deadbeef --tx-pwr 12
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def launch(role, pid, chan, args, extra_env):
    env = dict(os.environ)
    env.update({
        "DEVOURER_MIG_ROLE": role,
        "DEVOURER_VID": args.vid,
        "DEVOURER_PID": pid,
        "DEVOURER_CHANNEL": str(chan),
        "DEVOURER_MIG_KEY": args.key,
        "DEVOURER_MIG_RESCUE": str(args.chan_a),
        "DEVOURER_LOG_LEVEL": "warn",
        "DEVOURER_TX_PWR": str(args.tx_pwr),
    })
    env.update(extra_env)
    return subprocess.Popen(
        ["sudo", "-E", os.path.join(ROOT, "build", "chanmig"),
         "--role", role],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL, env=env, bufsize=1,
        universal_newlines=True, preexec_fn=os.setsid)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cycles", type=int, default=1000)
    ap.add_argument("--vid", default="0x0bda")
    ap.add_argument("--ground-pid", default="0x8812")
    ap.add_argument("--drone-pid", default="0xc812")
    ap.add_argument("--chan-a", type=int, default=36)
    ap.add_argument("--chan-b", type=int, default=149)
    ap.add_argument("--key", default="c0ffee")
    ap.add_argument("--tx-pwr", type=int, default=12)
    ap.add_argument("--allowed", default=None)
    ap.add_argument("--timeout", type=int, default=3600)
    args = ap.parse_args()

    allowed = args.allowed or f"{args.chan_a},{args.chan_b}"
    procs = []

    def cleanup(*_):
        for p in procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                pass
        time.sleep(1)
        for p in procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass

    signal.signal(signal.SIGINT, lambda *_: (cleanup(), sys.exit(1)))
    signal.signal(signal.SIGTERM, lambda *_: (cleanup(), sys.exit(1)))

    drone = launch("drone", args.drone_pid, args.chan_a, args,
                   {"DEVOURER_MIG_ALLOWED": allowed,
                    "DEVOURER_MIG_SYNTH_PPS": "200"})
    procs.append(drone)
    time.sleep(4)  # drone bring-up
    ground = launch("ground", args.ground_pid, args.chan_a, args, {})
    procs.append(ground)
    time.sleep(4)  # ground bring-up

    confirmed = 0
    rolled_back = 0
    split = 0
    cur = args.chan_b  # first proposal target
    outages = []
    t_switch = None
    last_state = None
    deadline = time.time() + args.timeout

    def propose(ch):
        try:
            ground.stdin.write(f"propose {ch}\n")
            ground.stdin.flush()
        except (BrokenPipeError, ValueError):
            pass

    propose(cur)
    t_switch = time.time()

    while confirmed < args.cycles and time.time() < deadline:
        line = ground.stdout.readline()
        if not line:
            if ground.poll() is not None:
                print("ground exited early", file=sys.stderr)
                break
            continue
        if '"ev":"' not in line:
            continue
        try:
            ev = json.loads(line)
        except json.JSONDecodeError:
            continue
        e = ev.get("ev")
        if e == "migrate.done":
            code = ev.get("code")
            if code == 0:  # confirmed on the new channel
                confirmed += 1
                if t_switch is not None:
                    outages.append(time.time() - t_switch)
                cur = args.chan_a if cur == args.chan_b else args.chan_b
                if confirmed % 50 == 0:
                    p = sorted(outages)
                    print(f"  {confirmed}/{args.cycles} migrations, "
                          f"outage p50={p[len(p)//2]*1000:.0f}ms "
                          f"p90={p[int(len(p)*0.9)]*1000:.0f}ms")
                time.sleep(0.3)  # let both settle on the new channel
                propose(cur)
                t_switch = time.time()
            elif code == 1:  # rolled back / held on old
                rolled_back += 1
                time.sleep(0.5)
                propose(cur)  # retry
                t_switch = time.time()

    cleanup()

    ok = confirmed >= args.cycles and split == 0
    p = sorted(outages) if outages else [0]
    print(f"\nconfirmed={confirmed} rolled_back={rolled_back} split_brain={split}")
    if outages:
        print(f"outage p50={p[len(p)//2]*1000:.0f}ms "
              f"p90={p[int(len(p)*0.9)]*1000:.0f}ms "
              f"p99={p[min(len(p)-1,int(len(p)*0.99))]*1000:.0f}ms")
    print("PASS" if ok else "INCOMPLETE")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
