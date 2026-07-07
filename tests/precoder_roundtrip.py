#!/usr/bin/env python3
"""Two-adapter, no-SDR round-trip test for the precoder PoC.

TX one devourer adapter (precoder, shaped PSDU) and RX a second devourer
adapter (rxdemo, monitor mode) on the same channel, then check:

  1. TRANSPORT   - the canonical-SA frame is received at all (hits > 0).
  2. PHY RATE    - it flew as 6 Mbps legacy OFDM (RX rate index == DESC_RATE6M
                   = 0x04), NOT the 1 Mbps CCK fallback (0x00) that an HT-MCS
                   radiotap would cause (see the precoder README). This is the
                   on-hardware check for the legacy-vs-CCK correction.
  3. BYTES       - the received PSDU body equals the encoder's shaped bytes.

WHAT THIS DOES *NOT* PROVE: per-subcarrier IQ. A bit-level Wi-Fi RX demodulates
to bytes (it runs the same descramble/Viterbi/deinterleave that inverts whatever
the chip did), so the bytes always round-trip regardless of whether our
subcarrier model matches the chip. Confirming that chosen OFDM data subcarriers
carry chosen values needs the IQ domain — i.e. the SDR FFT (fft_capture.py) or
a CSI-capable rig. Two Wi-Fi adapters verify the *transport + rate* precondition
for that, not the subcarrier shaping itself.

Targets RTL8812AU / RTL8821AU / RTL8811AU at 2.4 GHz (devourer RX is solid
there). Run as root, two adapters plugged:

    # Generate the shaped PSDU as your normal user (uv-managed deps):
    cd tools/precoder && uv run python encode_subcarriers.py \\
        --pattern target.txt --scrambler-seed 0x5d --psdu-out /tmp/shaped.bin
    # Then run the round-trip as root:
    sudo python3 tests/precoder_roundtrip.py \\
        --tx-pid 0x8812 --rx-pid 0x8813 --psdu /tmp/shaped.bin --channel 6

Omit --psdu and the script will try to generate one via `uv run` in
tools/precoder (needs the uv env; awkward under sudo — prefer pre-generating).
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
PRECODER = REPO / "tools" / "precoder"

sys.path.insert(0, str(HERE))
from devourer_events import iter_events, parse_event  # noqa: E402

DESC_RATE6M = 0x04  # legacy OFDM 6 Mbps; CCK is 0x00-0x03, HT/VHT MCS is 0x0c+
N_SD_LEGACY = 48

# rxdemo emits machine events as JSON Lines on stdout:
#   {"ev":"rx.txhit",...}  — canonical-SA frame match
#   {"ev":"rx.body",...}   — DEVOURER_DUMP_BODY frame body + Tier-2 health
# Human diagnostics go to stderr as `devourer [I] ...` and never parse as
# events, so merging stderr into the reader below stays safe.


def rate_name(idx: int) -> str:
    if idx <= 0x03:
        return f"CCK {[1, 2, 5.5, 11][idx]}M (DSSS — NOT OFDM!)"
    if idx <= 0x0B:
        return f"legacy OFDM {[6, 9, 12, 18, 24, 36, 48, 54][idx - 4]}M"
    return f"HT/VHT MCS (idx 0x{idx:02x})"


def make_pattern(path: Path, n_sym: int, n_sd: int = N_SD_LEGACY) -> None:
    """Deterministic ±1 pattern file (stdlib only — no numpy in the harness)."""
    import random
    rng = random.Random(0xC0DE)
    vals = [rng.choice((1, -1)) for _ in range(n_sym * n_sd)]
    path.write_text("# deterministic precoder_roundtrip pattern\n"
                    + "\n".join(str(v) for v in vals) + "\n")


def gen_shaped(out: Path, seed: int, n_sym: int) -> bytes:
    """Generate a shaped PSDU via the uv-managed encoder; return its bytes."""
    pat = out.with_suffix(".pattern.txt")
    make_pattern(pat, n_sym)
    cmd = ["uv", "run", "python", "encode_subcarriers.py",
           "--pattern", str(pat), "--phy", "legacy",
           "--scrambler-seed", f"0x{seed:02x}", "--psdu-out", str(out)]
    print(f"[gen] {' '.join(cmd)}  (cwd={PRECODER})")
    subprocess.run(cmd, cwd=PRECODER, check=True)
    return out.read_bytes()


class Reader(threading.Thread):
    """Drain a subprocess' stdout into a line list (so readline can't wedge us)."""

    def __init__(self, proc: subprocess.Popen):
        super().__init__(daemon=True)
        self.proc = proc
        self.lines: list[str] = []
        self._stop = False

    def run(self) -> None:
        assert self.proc.stdout is not None
        for line in self.proc.stdout:
            self.lines.append(line)
            if self._stop:
                break

    def stop(self) -> None:
        self._stop = True


def launch(binary: str, pid: str, vid: str, channel: int, extra_env: dict,
           args_list: "list[str] | None" = None) -> subprocess.Popen:
    env = dict(os.environ, DEVOURER_PID=pid, DEVOURER_VID=vid,
               DEVOURER_CHANNEL=str(channel), DEVOURER_USB_QUIET="1", **extra_env)
    return subprocess.Popen([binary, *(args_list or [])], env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1)


def run_test(args) -> int:
    # 1. shaped PSDU bytes (the expected body).
    if args.psdu:
        expected = Path(args.psdu).read_bytes()
        print(f"[gen] using supplied PSDU {args.psdu} ({len(expected)} bytes)")
    else:
        expected = gen_shaped(Path(args.workdir) / "shaped.bin",
                              args.seed, args.n_sym)
    shaped_path = args.psdu or str(Path(args.workdir) / "shaped.bin")

    if args.dry_run:
        print(f"[dry-run] would TX {args.tx_bin} --psdu {shaped_path} on "
              f"pid={args.tx_pid}, RX {args.rx_bin} on pid={args.rx_pid}, "
              f"channel {args.channel}, {args.duration}s; expect rate=0x04 "
              f"(6M OFDM) and body[:{len(expected)}] == shaped.")
        return 0

    # 2. RX first, give it time to bring the radio up.
    print(f"[rx] launching {args.rx_bin} vid={args.rx_vid} pid={args.rx_pid} "
          f"ch{args.channel}")
    rx = launch(args.rx_bin, args.rx_pid, args.rx_vid, args.channel,
                {"DEVOURER_DUMP_BODY": "1"})
    rx_reader = Reader(rx)
    rx_reader.start()
    time.sleep(args.rx_warmup)

    # 3. TX.
    print(f"[tx] launching {args.tx_bin} --psdu {shaped_path} vid={args.tx_vid} "
          f"pid={args.tx_pid}")
    tx_env = dict(os.environ, DEVOURER_PID=args.tx_pid, DEVOURER_VID=args.tx_vid,
                  DEVOURER_CHANNEL=str(args.channel), DEVOURER_USB_QUIET="1")
    tx = subprocess.Popen([args.tx_bin, "--psdu", shaped_path], env=tx_env,
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # 4. collect for the duration.
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            if any(parse_event(l, "rx.body") for l in rx_reader.lines):
                time.sleep(1.0)  # let a couple more land
                break
            time.sleep(0.5)
    finally:
        for p in (tx, rx):
            p.terminate()
        rx_reader.stop()
        for p in (tx, rx):
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()

    # 5. verdict.
    hits = sum(1 for _ in iter_events(rx_reader.lines, ev="rx.txhit"))
    bodies = list(iter_events(rx_reader.lines, ev="rx.body"))
    if args.keep:
        log = Path(args.workdir) / "rx.log"
        log.write_text("".join(rx_reader.lines))
        print(f"[keep] RX log -> {log}")

    print(f"\n--- precoder round-trip ({args.tx_pid} TX -> {args.rx_pid} RX) ---")
    ok = True

    print(f"[1/3] transport: {hits} canonical-SA frame(s) received",
          "PASS" if hits else "FAIL")
    ok &= hits > 0

    if not bodies:
        print("[2/3] phy rate: no rx.body event — FAIL")
        print("[3/3] bytes:    n/a — FAIL")
        return 1 if not args.allow_fail else 0

    ev = bodies[0]
    rate = int(ev["rate"])
    rx_body = bytes.fromhex(ev.get("body", ""))
    print(f"[2/3] phy rate: idx 0x{rate:02x} = {rate_name(rate)} — "
          + ("PASS" if rate == DESC_RATE6M else "FAIL"))
    ok &= rate == DESC_RATE6M

    # Tier-2 link-health diagnostics (info only — content-blind, never a
    # per-subcarrier or control signal; see the precoder README).
    if "rssi" in ev:
        fmt = lambda k: ",".join(str(v) for v in ev.get(k, []))  # noqa: E731
        print(f"[ -- ] link health (info, not control): rssi={fmt('rssi')} "
              f"evm={fmt('evm')} snr={fmt('snr')} crc_err={ev.get('crc', 0)}")

    n = min(len(expected), len(rx_body))
    match = n > 0 and rx_body[:n] == expected[:n]
    print(f"[3/3] bytes:    {n}/{len(expected)} body bytes compared, "
          f"{'identical' if match else 'DIVERGENT'} — "
          + ("PASS" if match else "FAIL"))
    ok &= match
    if not match and n:
        diff = next((i for i in range(n) if rx_body[i] != expected[i]), None)
        print(f"        first diff at byte {diff}: rx={rx_body[diff]:#04x} "
              f"expected={expected[diff]:#04x}")

    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok or args.allow_fail else 1


def main(argv: "list[str] | None" = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tx-pid", required=True, help="DEVOURER_PID of the TX adapter")
    ap.add_argument("--rx-pid", required=True, help="DEVOURER_PID of the RX adapter")
    ap.add_argument("--tx-vid", default="0x0bda", help="DEVOURER_VID of TX adapter "
                    "(default 0x0bda; e.g. 0x2357 for a TP-Link T2U Plus)")
    ap.add_argument("--rx-vid", default="0x0bda", help="DEVOURER_VID of RX adapter")
    ap.add_argument("--channel", type=int, default=6)
    ap.add_argument("--psdu", help="pre-generated shaped PSDU (recommended); "
                    "omit to auto-generate via uv")
    ap.add_argument("--seed", type=lambda s: int(s, 0), default=0x5D,
                    help="scrambler seed for auto-generation")
    ap.add_argument("--n-sym", type=int, default=8,
                    help="OFDM symbols of shaped payload (auto-generation)")
    ap.add_argument("--tx-bin", default=str(REPO / "build" / "precoder"))
    ap.add_argument("--rx-bin", default=str(REPO / "build" / "rxdemo"))
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--rx-warmup", type=float, default=12.0,
                    help="seconds to let the RX radio come up before TX. Some "
                         "chips are slow to init (RTL8812AU RX needed ~15s on "
                         "the bench; 10s was flaky) — raise this if transport "
                         "reports 0 frames")
    ap.add_argument("--workdir", default="/tmp/precoder-roundtrip")
    ap.add_argument("--keep", action="store_true", help="save the RX log")
    ap.add_argument("--dry-run", action="store_true",
                    help="generate the shaped PSDU and print the plan; no USB")
    ap.add_argument("--allow-fail", action="store_true",
                    help="always exit 0 (report only)")
    args = ap.parse_args(argv)
    os.makedirs(args.workdir, exist_ok=True)
    return run_test(args)


if __name__ == "__main__":
    raise SystemExit(main())
