#!/usr/bin/env python3
"""Two-adapter byte-stream round-trip on the precoder stream link.

Streams `--bytes N` random bytes from one devourer adapter (StreamTxDemo,
fed by tools/precoder/stream_tx.py) to a second devourer adapter
(WiFiDriverDemo with DEVOURER_STREAM_OUT=1), then decodes the received
<devourer-stream> lines via stream.decode_body and checks:

  1. TRANSPORT  — at least `--min-frames` frames decoded.
  2. RX RATE    — every decoded frame's RX rate index == DESC_RATE6M (0x04),
                   i.e. the chip really transmitted legacy 6M OFDM.
  3. BYTES      — concatenating decoded payloads (by ascending seq, after
                   dedup) reproduces the source bytes exactly.

With `--shape '0:+1,10:-1,...'` the same shape is used on both sides AND each
captured body is fed back through `emulate_chip` to confirm the encoded
subcarriers honour the pin pattern. That is a model-bound check (it proves
"the bytes we sent encode a body that, under our model, puts ±1 at the
pinned subcarriers"); proving the chip actually radiates those subcarriers
on-air still needs an SDR / BB-dbgport observer — see tools/precoder/README.md.

Defaults match PrecoderDemo's matrix-validated cell: 2.4 GHz channel 6,
RTL8812AU TX and RTL8821AU / RTL8811AU RX. RTL8814AU is out of scope (TX
flakiness, issue #36).

    sudo python3 tests/precoder_stream_roundtrip.py \\
        --tx-pid 0x8812 --rx-pid 0x8813 --channel 6 --bytes 4096
    sudo python3 tests/precoder_stream_roundtrip.py \\
        --tx-pid 0x8812 --rx-pid 0x8813 --channel 6 --bytes 1024 \\
        --shape '0:+1,8:-1,16:+1,24:-1,32:+1'
"""

from __future__ import annotations

import argparse
import os
import random
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
PRECODER = REPO / "tools" / "precoder"
sys.path.insert(0, str(PRECODER))
import stream  # noqa: E402
import encode_subcarriers as enc  # noqa: E402

DESC_RATE6M = 0x04
_STREAM_RE = re.compile(
    r"<devourer-stream>rate=(?P<rate>\d+)\s+len=(?P<len>\d+)"
    r"(?:\s+crc_err=(?P<crc_err>\d+))?"
    r"(?:\s+icv_err=(?P<icv_err>\d+))?"
    r"(?:\s+rssi=(?P<rssi>-?\d+,-?\d+))?"
    r"(?:\s+evm=(?P<evm>-?\d+,-?\d+))?"
    r"(?:\s+snr=(?P<snr>-?\d+,-?\d+))?"
    r"\s+body=(?P<hex>[0-9a-fA-F]*)"
)


def parse_shape(s: str | None):
    if not s:
        return None
    out = {}
    for tok in s.split(","):
        k, v = tok.split(":")
        sign = +1 if v.strip().lstrip("+") in ("1", "") else -1
        out[int(k.strip(), 0)] = sign
    return out


class Reader(threading.Thread):
    """Drain a subprocess' stdout into a line list."""

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


def _check_shape_honoured(body: bytes, plen: int, shape: dict, *, seed: int,
                          offset: int, entry_state: int) -> tuple[bool, int]:
    """Run emulate_chip over the encoded-symbol prefix of `body` and verify
    pinned subcarriers match shape.

    `plen` is the framed payload length recovered from the decoded envelope;
    it lets us compute exactly how many OFDM symbols the encoder used (the
    received body trails extra chip bytes — 4 B FCS + RX trailer — that the
    encoder didn't shape and would otherwise look like violations).

    Returns (ok, n_violations) where n_violations counts (symbol,
    subcarrier) cells whose model output differs from the requested pin.
    """
    import numpy as np

    phy = enc._LEGACY_BPSK
    layout = stream.plan_body(plen, shape, phy=phy)
    bytes_per_sym = phy.n_dbps // 8
    expected = layout.n_sym * bytes_per_sym
    if len(body) < expected:
        return False, -1
    psdu_bits = enc.bytes_to_bits(body[:expected])
    sub = enc.emulate_chip(psdu_bits, seed, phy, layout.n_sym,
                           offset=offset, entry_state=entry_state)
    violations = 0
    for sc, want in shape.items():
        violations += int(np.sum(sub[:, sc] != want))
    return violations == 0, violations


def run_test(args) -> int:
    rng = random.Random(args.data_seed)
    data = bytes(rng.randint(0, 255) for _ in range(args.bytes))
    print(f"[data] {len(data)} bytes (seed=0x{args.data_seed:x})")

    workdir = Path(args.workdir)
    workdir.mkdir(parents=True, exist_ok=True)
    data_path = workdir / "tx_data.bin"
    data_path.write_bytes(data)

    if args.dry_run:
        print(f"[dry-run] would TX {args.tx_bin} via stream_tx.py "
              f"({args.bytes}B, shape={args.shape!r}), RX {args.rx_bin} on "
              f"ch{args.channel} for up to {args.duration}s")
        return 0

    # 1. RX up first.
    rx_env = dict(os.environ, DEVOURER_PID=args.rx_pid, DEVOURER_VID=args.rx_vid,
                  DEVOURER_CHANNEL=str(args.channel),
                  DEVOURER_STREAM_OUT="1", DEVOURER_USB_QUIET="1")
    print(f"[rx] launching {args.rx_bin} vid={args.rx_vid} pid={args.rx_pid}")
    rx = subprocess.Popen([args.rx_bin], env=rx_env,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          text=True, bufsize=1)
    rx_reader = Reader(rx)
    rx_reader.start()
    time.sleep(args.rx_warmup)

    # 2. TX side: stream_tx.py | StreamTxDemo, with shape/seed/offset/etc.
    tx_env = dict(os.environ, DEVOURER_PID=args.tx_pid, DEVOURER_VID=args.tx_vid,
                  DEVOURER_CHANNEL=str(args.channel), DEVOURER_USB_QUIET="1")
    pyenv = dict(os.environ)
    if args.shape:
        pyenv["DEVOURER_STREAM_SHAPE"] = args.shape
    if args.seed is not None:
        pyenv["DEVOURER_STREAM_SEED"] = hex(args.seed)
    if args.offset is not None:
        pyenv["DEVOURER_STREAM_OFFSET"] = str(args.offset)
    if args.entry_state is not None:
        pyenv["DEVOURER_STREAM_ENTRY_STATE"] = hex(args.entry_state)

    encoder_cmd = ["uv", "run", "python", "stream_tx.py", "--input",
                   str(data_path), "--repeat", str(args.repeat)]
    print(f"[tx] {' '.join(encoder_cmd)} | {args.tx_bin}")
    encoder = subprocess.Popen(encoder_cmd, cwd=str(PRECODER), env=pyenv,
                               stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE)
    tx = subprocess.Popen([args.tx_bin, "--interval-ms",
                           str(args.interval_ms)],
                          env=tx_env, stdin=encoder.stdout,
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if encoder.stdout is not None:
        encoder.stdout.close()  # let the encoder receive SIGPIPE if tx dies

    expected_frames = max(1, (args.bytes + 13) // 14)  # mtu=14 default

    # 3. Collect until expected frames received or duration elapsed.
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            hits = sum(1 for l in rx_reader.lines if _STREAM_RE.search(l))
            if hits >= expected_frames + 2:  # small buffer for retransmits
                time.sleep(0.5)
                break
            time.sleep(0.5)
    finally:
        for p in (tx, encoder, rx):
            try:
                p.terminate()
            except ProcessLookupError:
                pass
        rx_reader.stop()
        for p in (tx, encoder, rx):
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()

    if args.keep:
        (workdir / "rx.log").write_text("".join(rx_reader.lines))
        encoder_err = encoder.stderr.read() if encoder.stderr else b""
        (workdir / "encoder.log").write_bytes(encoder_err)
        print(f"[keep] logs -> {workdir}")

    # 4. Verdict.
    shape = parse_shape(args.shape)
    seed = args.seed if args.seed is not None else stream.DEFAULT_SEED
    offset = args.offset if args.offset is not None else 0
    entry_state = (args.entry_state if args.entry_state is not None else 0)

    by_seq: dict[int, bytes] = {}
    rate_mismatch = 0
    malformed = 0
    shape_violations = 0
    shape_checked = 0
    bodies_for_shape: list[tuple[bytes, int]] = []  # (body, plen)

    for l in rx_reader.lines:
        m = _STREAM_RE.search(l)
        if not m:
            continue
        rate = int(m.group("rate"))
        if rate != DESC_RATE6M:
            rate_mismatch += 1
        body = bytes.fromhex(m.group("hex"))
        frame = stream.decode_body(body, shape=shape, seed=seed,
                                   offset=offset, entry_state=entry_state)
        if frame is None:
            malformed += 1
            continue
        if frame.seq not in by_seq:
            by_seq[frame.seq] = frame.payload
            if shape:
                bodies_for_shape.append((body, len(frame.payload)))

    print(f"\n--- stream round-trip ({args.tx_pid} TX → {args.rx_pid} RX) ---")
    ok = True

    print(f"[1/3] transport: {len(by_seq)} unique frame(s) decoded, "
          f"expected ~{expected_frames}",
          "PASS" if len(by_seq) >= args.min_frames else "FAIL")
    ok &= len(by_seq) >= args.min_frames

    print(f"[2/3] phy rate:  {rate_mismatch} non-OFDM frame(s) out of "
          f"{len(by_seq) + malformed + rate_mismatch} captured",
          "PASS" if rate_mismatch == 0 else "FAIL")
    ok &= rate_mismatch == 0

    rx_data = b"".join(by_seq[s] for s in sorted(by_seq))
    common = min(len(rx_data), len(data))
    match = common > 0 and rx_data[:common] == data[:common]
    if len(by_seq) >= expected_frames:
        match = match and rx_data == data
    print(f"[3/3] bytes:     {common}/{len(data)} compared, "
          + ("identical" if match else "DIVERGENT"),
          "PASS" if match else "FAIL")
    ok &= match
    if not match and common:
        diff = next((i for i in range(common) if rx_data[i] != data[i]), None)
        print(f"        first diff at byte {diff}: rx={rx_data[diff]:#04x} "
              f"expected={data[diff]:#04x}")

    if shape:
        for body, plen in bodies_for_shape:
            shape_checked += 1
            okp, violations = _check_shape_honoured(
                body, plen, shape,
                seed=seed, offset=offset, entry_state=entry_state)
            if not okp:
                shape_violations += max(violations, 1)
        print(f"[4/4] shape:     {shape_checked} body/bodies model-checked, "
              f"{shape_violations} subcarrier violation(s) "
              + ("PASS" if shape_violations == 0 else "FAIL"))
        ok &= shape_violations == 0
        print("        NB: model-bound check only — confirms encoded bytes "
              "carry the shape; on-air shape needs SDR / BB-dbgport.")

    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok or args.allow_fail else 1


def main(argv: "list[str] | None" = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--tx-pid", required=True)
    ap.add_argument("--rx-pid", required=True)
    ap.add_argument("--tx-vid", default="0x0bda")
    ap.add_argument("--rx-vid", default="0x0bda")
    ap.add_argument("--channel", type=int, default=6)
    ap.add_argument("--bytes", type=int, default=512,
                    help="number of stream bytes to send (default 512)")
    ap.add_argument("--data-seed", type=lambda s: int(s, 0), default=0xBEEF)
    ap.add_argument("--shape", default=None,
                    help="shape spec passed to both TX and RX, e.g. "
                         "'0:+1,10:-1,20:+1'. Adds a model-bound subcarrier "
                         "check on every captured body.")
    ap.add_argument("--seed", type=lambda s: int(s, 0), default=None,
                    help="chip scrambler seed for the encoder/decoder model "
                         "(byte mode ignores this)")
    ap.add_argument("--offset", type=int, default=None,
                    help="scrambler-phase offset of the body (default 0 — the "
                         "shape pins are honoured by the model but not "
                         "on-air; pass 208 to match PrecoderDemo placement)")
    ap.add_argument("--entry-state", type=lambda s: int(s, 0), default=None)
    ap.add_argument("--tx-bin", default=str(REPO / "build" / "StreamTxDemo"))
    ap.add_argument("--rx-bin", default=str(REPO / "build" / "WiFiDriverDemo"))
    ap.add_argument("--interval-ms", type=int, default=2)
    ap.add_argument("--repeat", type=int, default=4,
                    help="TX-side per-frame replication to combat early-frame "
                         "loss during the RX warmup (default 4)")
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--rx-warmup", type=float, default=12.0,
                    help="seconds to let the RX radio come up before TX. Some "
                         "chips need ~15s; raise if transport reports 0")
    ap.add_argument("--min-frames", type=int, default=1,
                    help="minimum decoded frames to call the transport check "
                         "a PASS (default 1)")
    ap.add_argument("--workdir", default="/tmp/precoder-stream-roundtrip")
    ap.add_argument("--keep", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--allow-fail", action="store_true")
    args = ap.parse_args(argv)
    return run_test(args)


if __name__ == "__main__":
    raise SystemExit(main())
