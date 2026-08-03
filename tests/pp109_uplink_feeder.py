#!/usr/bin/env python3
"""Feed examples/duplex's stdin with PixelPilot's uplink traffic shape.

PixelPilot's ground station sends its adaptive-link + mavlink uplink as
wfb-ng FEC blocks: k=1 n=5 means every message is put on the air five times, and
Transmitter::sendPacket emits a block's fragments back-to-back with no spacing.
Measured on the phone: ~102 frames/s arriving as bursts of 10-12 frames inside
1-2 ms, once per ~100 ms adaptive-link period. That burst shape is the input
here -- BURST frames written to the pipe at once (the duplex TX thread drains
them back-to-back), then idle until the next period.

The run is a sequence of PHASES, each PHASE_S seconds. A phase is either idle
(no uplink at all -- the "muted" control) or an uplink at one rate. Every phase
opens with a SET_RATE control op, which makes duplex emit exactly one
`stream.ctl` JSONL line: the Nth `stream.ctl` in duplex's event stream is the
start of the Nth phase, so the analyzer can bin `rx.frame` counts per phase from
file order alone -- no clock correlation between two processes.

Alternating idle/uplink phases inside ONE duplex session is deliberate: it holds
the RF path, the chip's thermal state and the flooder constant across arms, so
the only thing changing is whether (and how) this adapter transmits.

  python3 tests/pp109_uplink_feeder.py --phases idle,6M,MCS0,MCS7 --cycles 4
"""
import argparse
import struct
import sys
import time

CTL_SET_RATE = 2


def write_ctl(out, op, payload=b""):
    """<u32_le (0x80000000 | len)><op:u8><payload> -- duplex's control escape."""
    body = bytes([op]) + payload
    out.write(struct.pack("<I", 0x80000000 | len(body)) + body)
    out.flush()


def write_psdu(out, body):
    out.write(struct.pack("<I", len(body)) + body)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--phases", default="idle,6M,MCS0,MCS7",
                    help="comma list; 'idle' = uplink muted (control arm)")
    ap.add_argument("--cycles", type=int, default=4)
    ap.add_argument("--phase-s", type=float, default=5.0)
    ap.add_argument("--period-ms", type=float, default=100.0,
                    help="adaptive-link period (one burst per period)")
    ap.add_argument("--burst", type=int, default=10,
                    help="frames per burst (wfb-ng k=1 n=5 x 2 messages)")
    ap.add_argument("--psdu", type=int, default=100, help="PSDU body bytes")
    ap.add_argument("--warmup-s", type=float, default=3.0,
                    help="idle lead-in before the first phase marker")
    a = ap.parse_args()

    # A phase is "<rate>" or "<rate>:<burst>" -- the per-burst frame count is
    # the second, rate-independent way to scale airtime, so a rate sweep and a
    # burst sweep can disagree and that disagreement is informative.
    phases = [p.strip() for p in a.phases.split(",") if p.strip()]
    out = sys.stdout.buffer
    body = bytes(range(256)) * ((a.psdu // 256) + 1)
    body = body[:a.psdu]

    time.sleep(a.warmup_s)
    plan = []
    for c in range(a.cycles):
        for ph in phases:
            # Phase marker + live rate switch. 'idle' still switches (to 6M) so
            # every phase costs exactly one stream.ctl -- the marker must not
            # itself differ between arms.
            rate, burst = (ph.split(":", 1) + [None])[:2] if ":" in ph else (ph, None)
            burst = int(burst) if burst else a.burst
            spec = "6M" if rate == "idle" else rate
            write_ctl(out, CTL_SET_RATE, spec.encode())
            t_end = time.monotonic() + a.phase_s
            sent = 0
            if rate == "idle":
                time.sleep(a.phase_s)
            else:
                period = a.period_ms / 1000.0
                nxt = time.monotonic()
                while time.monotonic() < t_end:
                    for _ in range(burst):
                        write_psdu(out, body)
                    out.flush()
                    sent += burst
                    nxt += period
                    d = nxt - time.monotonic()
                    if d > 0:
                        time.sleep(d)
            plan.append((c, ph, sent))
            print(f"[feeder] cycle={c} phase={ph} frames={sent}", file=sys.stderr,
                  flush=True)
    print("[feeder] done: " + ";".join(f"{c}:{p}:{n}" for c, p, n in plan),
          file=sys.stderr, flush=True)


if __name__ == "__main__":
    try:
        main()
    except BrokenPipeError:
        # The pipe reader (duplex) went away — a bench teardown or crash, not
        # a feeder bug. Exit quietly; os._exit skips the interpreter's stdout
        # flush-at-exit, which would raise the same error again.
        print("[feeder] reader closed the pipe — stopping", file=sys.stderr,
              flush=True)
        import os
        os._exit(1)
