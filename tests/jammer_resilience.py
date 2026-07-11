#!/usr/bin/env python3
"""M5 jammer-resilience measurement (parked narrowband jammer).

Parks a USRP B210 narrowband jammer (tests/sdr_interferer.py) on ONE channel of
the hopset and measures end-to-end FEC-decoded delivery of the fused-FEC link
(tools/precoder/fused_fec_tx.py -> streamtx -> rxdemo -> fused_fec_rx.py) under
four transmit strategies:

  static_clean    - TX+RX parked on an UN-jammed channel   (upper bound ~100%)
  static_jammed   - TX+RX parked on the JAMMED channel     (lower bound ~0%)
  sequential      - TX+RX lockstep round-robin over the hopset
  keyed           - TX+RX lockstep keyed (SipHash) permutation

Against a blind parked jammer sequential and keyed are expected to be
statistically identical (both spend 1/N of dwells on the jammed channel, and the
RS+SBI code recovers that erasure fraction) — that equivalence is the result
that motivates the follower-jammer experiment (run_follower_jammer), where only
keyed survives a reactive jammer.

Delivery metrics per cell (P = total source packets in the fixed payload):
  fec_delivery  = sbi_packets  / P   (RS + sub-block-integrity salvage)
  base_delivery = base_packets / P   (drop-whole-corrupt-frame baseline)
  raw_frames    = (frames_seen - frames_corrupt) / frames_sent

Run under sudo (USB claim + UHD both need root); system python3 (UHD is a system
package). Three radios: TX=RTL8812AU, RX=RTL8812CU, jammer=B210.

  sudo python3 tests/jammer_resilience.py \
      --tx-pid 0x8812 --rx-pid 0xc812 \
      --channels 36,40,44,48 --jam-channel 40 --slot-ms 50 \
      --seed 00112233445566778899aabbccddeeff --jammer-gain 60 --secs 20
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_ROOT, "tools", "precoder"))
sys.path.insert(0, _HERE)


def chan_to_freq(ch: int) -> float:
    return (2407 + 5 * ch) * 1e6 if ch <= 14 else (5000 + 5 * ch) * 1e6


def build_payload(nbytes: int) -> bytes:
    # Deterministic (fixed-seed) payload so P and the run are reproducible.
    import numpy as np
    return np.random.default_rng(0xC0FFEE).integers(
        0, 256, size=nbytes, dtype=np.uint8).tobytes()


def fec_counts(payload: bytes) -> tuple[int, int]:
    """(P source packets, B bodies/frames) for the default fused-FEC config
    (must match fused_fec_tx.py defaults: k=8, symbol=64, overhead=0.5,
    blocks=4, rs). P via a real encode so the decoder's packet count matches."""
    from stream_fec import FecConfig
    from fused_fec_link import FusedFecSender, FusedFecReceiver
    cfg = FecConfig(k=8, symbol_size=64, overhead=0.5, scheme="rs")
    snd = FusedFecSender(cfg, 4, crc_bytes=2)
    bodies = snd.add_bytes(payload) + snd.flush()
    rcv = FusedFecReceiver(cfg, 4, crc_bytes=2)
    pk = sum(len(rcv.add_frame(b, False)) for b in bodies)  # clean-loopback P
    return pk, len(bodies)


class Proc:
    """Popen wrapper in its own session so a whole pipe can be signalled."""

    def __init__(self, argv, **kw):
        kw.setdefault("start_new_session", True)
        self.p = subprocess.Popen(argv, **kw)

    def stop(self, sig=signal.SIGINT):
        if self.p.poll() is None:
            try:
                os.killpg(os.getpgid(self.p.pid), sig)
            except ProcessLookupError:
                pass

    def kill(self):
        if self.p.poll() is None:
            try:
                os.killpg(os.getpgid(self.p.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass


def parse_fec_report(stderr_text: str) -> dict:
    """Pull the counters out of fused_fec_rx.py's stderr report."""
    out = {"frames_seen": 0, "frames_corrupt": 0,
           "base_packets": 0, "sbi_packets": 0}
    for line in stderr_text.splitlines():
        for key, tok in (("frames_seen", "frames="),
                         ("frames_corrupt", "corrupt=")):
            if tok in line:
                try:
                    out[key] = int(line.split(tok)[1].split()[0])
                except (IndexError, ValueError):
                    pass
        if "baseline blocks=" in line and "pkts=" in line:
            out["base_packets"] = int(line.split("pkts=")[1].split()[0])
        if line.strip().startswith("fused_fec_rx: sbi") and "pkts=" in line:
            out["sbi_packets"] = int(line.split("pkts=")[1].split()[0])
    return out


def hop_env(mode: str, args, static_channel: int) -> tuple[dict, dict, int]:
    """(tx_env, rx_env, rx_park_channel) for a mode. Lockstep modes set the same
    hop vars on both ends; static modes park both on one channel."""
    tx, rx = {}, {}
    if mode in ("static_clean", "static_jammed"):
        return tx, rx, static_channel
    common = {
        "DEVOURER_HOP_CHANNELS": args.channels,
        "DEVOURER_HOP_SLOT_MS": str(args.slot_ms),
        "DEVOURER_HOP_FAST": "1",
    }
    if mode == "keyed":
        common["DEVOURER_HOP_SEED"] = args.seed
    tx.update(common)
    rx.update(common)
    return tx, rx, int(args.channels.split(",")[0])


def run_cell(mode: str, args, payload_path: str, P: int, B: int,
             jam: bool) -> dict:
    chans = [int(c) for c in args.channels.split(",")]
    jam_ch = args.jam_channel
    clean_ch = next(c for c in chans if c != jam_ch)
    static_ch = jam_ch if mode == "static_jammed" else clean_ch
    tx_hop, rx_hop, rx_ch = hop_env(mode, args, static_ch)
    tx_ch = static_ch if mode.startswith("static") else chans[0]

    py = sys.executable
    interferer = os.path.join(_HERE, "sdr_interferer.py")
    fec_tx = os.path.join(_ROOT, "tools", "precoder", "fused_fec_tx.py")
    fec_rx = os.path.join(_ROOT, "tools", "precoder", "fused_fec_rx.py")
    rxdemo = os.path.join(args.build, "rxdemo")
    streamtx = os.path.join(args.build, "streamtx")

    # Reap any demo the previous cell left behind before claiming the adapters.
    for name in ("rxdemo", "streamtx"):
        subprocess.run(["pkill", "-9", "-x", name],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for pat in ("sdr_interferer.py", "sdr_follower_jammer.py"):
        subprocess.run(["pkill", "-9", "-f", pat],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.5)

    procs: list[Proc] = []
    sys.stderr.write(f"\n=== cell mode={mode} jam={'on' if jam else 'off'} "
                     f"(jammer ch{jam_ch}, RX ch{rx_ch}, TX ch{tx_ch}) ===\n")
    try:
        # --- RX chain: rxdemo | fused_fec_rx ---
        rx_env = dict(os.environ)
        rx_env.update({
            "DEVOURER_PID": args.rx_pid,
            "DEVOURER_CHANNEL": str(rx_ch),
            "DEVOURER_STREAM_OUT": "1",
            "DEVOURER_RX_KEEP_CORRUPTED": "1",
            "DEVOURER_LOG_LEVEL": "warn",
        })
        rx_env.update(rx_hop)
        # Capture rxdemo's event stream to a FILE, not a pipe into fused_fec_rx:
        # the per-frame RS decode is slower than the frame rate, so piping backs
        # up the pipe, stalls rxdemo's USB reaping, and drops frames. Decode the
        # captured stream offline after the run instead.
        rx_capture = os.path.join("/tmp", f"devourer-jam-rx-{mode}.jsonl")
        rx_fh = open(rx_capture, "wb")
        rxp = Proc([rxdemo], env=rx_env, stdout=rx_fh,
                   stderr=subprocess.DEVNULL)
        rx_fh.close()          # rxdemo holds the write end
        procs += [rxp]

        # Let RX bring up (and lockstep-acquire once TX starts). Static needs
        # only chip bring-up; give everyone a common head start.
        time.sleep(args.rx_warmup)

        # --- Jammer (parked) ---
        if jam and args.jammer == "follower":
            # Full-duplex reactive/predictive follower (sdr_follower_jammer.py).
            # Reactive vs keyed, predictive vs the public sequential order.
            follower = os.path.join(_HERE, "sdr_follower_jammer.py")
            predict = "seq" if mode == "sequential" else "off"
            jarg = [py, follower, "--channels", args.channels,
                    "--predict", predict, "--tx-gain", str(args.jammer_gain),
                    "--secs", str(args.secs + 20),
                    "--log", f"/tmp/devourer-follow-{mode}.jsonl"]
            jam_err = open(f"/tmp/devourer-jam-follower-{mode}.log", "wb")
            procs.append(Proc(jarg, stdout=subprocess.DEVNULL, stderr=jam_err))
            time.sleep(3.0)
        elif jam:
            jarg = [py, interferer, "--channel", str(jam_ch),
                    "--tx-gain", str(args.jammer_gain),
                    "--mode", args.jammer_mode, "--rate", "20e6"]
            jam_err = open(f"/tmp/devourer-jam-interferer-{mode}.log", "wb")
            procs.append(Proc(jarg, stdout=subprocess.DEVNULL, stderr=jam_err))
            time.sleep(2.0)   # let the B210 tune + start radiating

        # --- TX chain: fused_fec_tx | streamtx ---
        tx_env = dict(os.environ)
        tx_env.update({
            "DEVOURER_PID": args.tx_pid,
            "DEVOURER_CHANNEL": str(tx_ch),
            "DEVOURER_LOG_LEVEL": "warn",
        })
        # Bench-geometry calibration: the RX and data TX are near-field (very
        # high SNR), so a strong link ignores the jammer entirely. Backing the
        # data TX power off sets a realistic link margin the jammer can contest.
        if args.data_tx_pwr is not None:
            tx_env["DEVOURER_TX_PWR"] = str(args.data_tx_pwr)
        tx_env.update(tx_hop)
        tx_capture = os.path.join("/tmp", f"devourer-jam-tx-{mode}.jsonl")
        tx_fh = open(tx_capture, "wb")
        ftx = Proc([py, fec_tx, "--input", payload_path],
                   stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        # streamtx emits its JSONL events (incl. stream.done sent=N) on stdout.
        stx = Proc([streamtx, "--interval-ms", str(args.interval_ms)],
                   stdin=ftx.p.stdout, env=tx_env,
                   stdout=tx_fh, stderr=subprocess.DEVNULL)
        ftx.p.stdout.close()
        tx_fh.close()
        procs += [ftx, stx]

        # TX runs until the whole payload is drained (streamtx exits on stdin
        # EOF); --secs+margin is only a safety cap.
        t0 = time.monotonic()
        while stx.p.poll() is None and time.monotonic() - t0 < args.secs + 10:
            time.sleep(0.2)
        stx.stop(); ftx.stop()
        # Actual frames on the wire from streamtx's stream.done (fall back to B).
        sent = B
        try:
            with open(tx_capture, "rb") as f:
                for line in f:
                    if b'"ev":"stream.done"' in line:
                        sent = json.loads(line.decode()).get("sent", B)
        except OSError:
            pass

        # Let the last in-flight frames land, then stop rxdemo (flushes + closes
        # the capture file).
        time.sleep(2.0)
        rxp.stop()
        rxp.p.wait(timeout=10)
        # Offline decode of the captured event stream — no live backpressure.
        fec = subprocess.run([py, fec_rx], stdin=open(rx_capture, "rb"),
                             stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        rep = parse_fec_report(fec.stderr.decode("utf-8", "replace"))
    finally:
        for pr in reversed(procs):
            pr.stop()
        time.sleep(0.4)
        for pr in procs:
            pr.kill()

    seen, corrupt = rep["frames_seen"], rep["frames_corrupt"]
    clean = max(0, seen - corrupt)
    res = {
        "ev": "jam.cell", "mode": mode, "jam": jam, "jam_ch": jam_ch,
        "rx_ch": rx_ch, "tx_ch": tx_ch, "P": P,
        "frames_sent": sent, "frames_seen": seen, "frames_corrupt": corrupt,
        "base_packets": rep["base_packets"], "sbi_packets": rep["sbi_packets"],
        "raw_frames": round(clean / sent, 4) if sent else 0.0,
        "base_delivery": round(rep["base_packets"] / P, 4) if P else 0.0,
        "fec_delivery": round(rep["sbi_packets"] / P, 4) if P else 0.0,
    }
    print(json.dumps(res), flush=True)
    sys.stderr.write(
        f"    frames seen={seen} corrupt={corrupt} clean={clean}  "
        f"base_deliv={res['base_delivery']:.3f} "
        f"fec_deliv={res['fec_delivery']:.3f}\n")
    return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tx-pid", default="0x8812")
    ap.add_argument("--rx-pid", default="0xc812")
    ap.add_argument("--channels", default="36,40,44,48")
    ap.add_argument("--jam-channel", type=int, default=40)
    ap.add_argument("--slot-ms", type=int, default=50)
    ap.add_argument("--seed", default="00112233445566778899aabbccddeeff")
    ap.add_argument("--jammer-gain", type=float, default=60.0)
    ap.add_argument("--data-tx-pwr", type=int, default=None,
                    help="data TX power index (DEVOURER_TX_PWR); lower = weaker "
                         "link margin so the jammer can contest the channel")
    ap.add_argument("--jammer-mode", default="noise", choices=("noise", "cw"))
    ap.add_argument("--jammer", default="parked", choices=("parked", "follower"),
                    help="parked = fixed on --jam-channel; follower = B210 "
                         "full-duplex chaser (reactive vs keyed, predictive vs "
                         "sequential)")
    ap.add_argument("--secs", type=float, default=20.0,
                    help="max seconds per cell (TX also stops at payload end)")
    ap.add_argument("--interval-ms", type=int, default=3,
                    help="streamtx inter-frame gap (frame rate = 1000/this)")
    ap.add_argument("--rx-warmup", type=float, default=4.0)
    ap.add_argument("--build", default=os.path.join(_ROOT, "build"))
    ap.add_argument("--modes",
                    default="static_clean,static_jammed,sequential,keyed")
    ap.add_argument("--no-jammer-baseline", action="store_true",
                    help="also run every mode with the jammer OFF (control)")
    args = ap.parse_args(argv)

    # Size the payload so the TX spans ~--secs at the chosen frame rate:
    # frames ~= secs*1000/interval_ms; ~152 payload bytes per frame/body.
    fps = 1000.0 / max(1, args.interval_ms)
    payload = build_payload(int(fps * args.secs * 152))
    P, B = fec_counts(payload)
    payload_path = "/tmp/devourer-jammer-payload.bin"
    with open(payload_path, "wb") as f:
        f.write(payload)
    sys.stderr.write(
        f"payload {len(payload)} bytes -> P={P} source packets, B={B} frames; "
        f"jammer ch{args.jam_channel} gain={args.jammer_gain} "
        f"mode={args.jammer_mode}\n")

    modes = [m for m in args.modes.split(",") if m]
    results = []
    for mode in modes:
        if args.no_jammer_baseline:
            results.append(run_cell(mode, args, payload_path, P, B, jam=False))
        results.append(run_cell(mode, args, payload_path, P, B, jam=True))

    # Summary table.
    print("\n=== jammer-resilience summary ===", flush=True)
    print(f"{'mode':<15}{'jam':<5}{'raw_frames':<12}"
          f"{'base_deliv':<12}{'fec_deliv':<12}", flush=True)
    for r in results:
        print(f"{r['mode']:<15}{'on' if r['jam'] else 'off':<5}"
              f"{r['raw_frames']:<12.3f}{r['base_delivery']:<12.3f}"
              f"{r['fec_delivery']:<12.3f}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
