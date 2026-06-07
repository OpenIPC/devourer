#!/usr/bin/env python3
"""TUN ⇄ stream-link bridge — IP-over-precoder for one P2P peer.

First-cut "real-world P2P" demo built on the stream layer. Opens a Linux TUN
device, spawns StreamTxDemo and/or WiFiDriverDemo as subprocesses, and runs
two threads:

    tun fd ──read──► encode_body ─length-prefix─► StreamTxDemo (libusb TX)
    WiFiDriverDemo (libusb RX) ─<devourer-stream>─► decode_body ──write──► tun fd

ONE IP PACKET = ONE STREAM FRAME (seq increments per packet, total=0 = unbounded
stream). body_bytes defaults to 1500 so a 1490-byte tun MTU fits cleanly with
the 10-byte envelope; tweak with --body-bytes if you want to trade airtime
per packet for throughput per packet.

NOT IN THIS V1 (deliberately, documented):

* No reliability. Probe-request has no MAC ACK and we add no ARQ or FEC. Lost
  packets are lost; out-of-order packets are written to TUN in arrival order
  (the kernel's IP/TCP stack tolerates that). Use --repeat N to blindly
  duplicate every TX body — rough loss insurance at N× the airtime cost.
* No dedup across the wire. If TX peer uses --repeat>1 the RX peer writes
  every duplicate to TUN. TCP/IP at the endpoints filters them out; UDP may
  see them. For a clean demo, run with --repeat 1.
* No congestion control or flow control. TUN reads are blocking; if the link
  is slow the upstream socket back-pressures naturally via the TUN's queue.
* Single-pair-of-peers. The body has no per-peer address field; the canonical
  SA carries every frame. Multiple bridges on the same channel will hear each
  other's traffic.

USAGE — duplex on one host with two adapters (requires CAP_NET_ADMIN):

    sudo python3 tools/precoder/tun_p2p.py \\
        --tx-pid 0x8812 \\
        --rx-pid 0x0120 --rx-vid 0x2357 \\
        --tun-name dvr0 --tun-addr 10.99.0.1/24 \\
        --channel 6

Pair this with a peer running the same command but with --tx-pid / --rx-pid
swapped and --tun-addr 10.99.0.2/24. ping between 10.99.0.1 and 10.99.0.2
once both sides are up.

ONE-WAY DEMO (also useful as a hardware smoke):

    # Sender (peer A):
    sudo python3 tools/precoder/tun_p2p.py --mode tx-only \\
        --tx-pid 0x8812 --tun-name dvr0 --tun-addr 10.99.0.1/30 --channel 6

    # Receiver (peer B):
    sudo python3 tools/precoder/tun_p2p.py --mode rx-only \\
        --rx-pid 0x0120 --rx-vid 0x2357 --tun-name dvr0 --tun-addr 10.99.0.5/30 \\
        --channel 6

Shape mode: pass --shape '0:+1,8:-1,16:+1' to both peers; same caveat as
stream_tx.py — offset/entry_state default to 0 so the shape is model-bound,
not on-air-honoured. Useful for "the bytes encode a shape" demos, not for
proving per-subcarrier IQ at the antenna.
"""

from __future__ import annotations

import argparse
import collections
import fcntl
import os
import re
import shutil
import signal
import struct
import subprocess
import sys
import threading
from pathlib import Path
from typing import Optional

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

import stream  # noqa: E402

# Linux TUN/TAP constants. See <linux/if_tun.h>.
TUNSETIFF = 0x400454CA
IFF_TUN = 0x0001
IFF_NO_PI = 0x1000

_STREAM_RE = re.compile(
    r"<devourer-stream>rate=(?P<rate>\d+) len=(?P<len>\d+) body=(?P<hex>[0-9a-fA-F]*)"
)


# --------------------------------------------------------------------------- #
# Shape spec — same parser as stream_tx.py / stream_rx.py
# --------------------------------------------------------------------------- #
def parse_shape(s: Optional[str]) -> Optional[dict]:
    if not s:
        return None
    out: dict[int, int] = {}
    for tok in s.split(","):
        tok = tok.strip()
        if not tok or ":" not in tok:
            continue
        k, v = tok.split(":", 1)
        v = v.strip()
        sign = +1 if v in ("+1", "+", "1") else -1 if v in ("-1", "-") else 0
        if sign == 0:
            raise ValueError(f"bad shape value {v!r}; want ±1")
        out[int(k.strip(), 0)] = sign
    return out or None


# --------------------------------------------------------------------------- #
# TUN device
# --------------------------------------------------------------------------- #
def open_tun(name: str, mtu: int, addr_cidr: Optional[str]) -> int:
    """Open /dev/net/tun, attach the named interface in TUN mode (L3, no PI),
    set MTU, optionally assign an IP, bring up. Returns the fd."""
    fd = os.open("/dev/net/tun", os.O_RDWR)
    ifr = struct.pack("16sH", name.encode().ljust(16, b"\x00")[:16],
                      IFF_TUN | IFF_NO_PI)
    fcntl.ioctl(fd, TUNSETIFF, ifr)
    ip = shutil.which("ip") or "/sbin/ip"
    subprocess.run([ip, "link", "set", name, "mtu", str(mtu)], check=True)
    if addr_cidr:
        # `replace` instead of `add` so a re-run after a crash doesn't EEXIST.
        subprocess.run([ip, "addr", "replace", addr_cidr, "dev", name],
                       check=True)
    subprocess.run([ip, "link", "set", name, "up"], check=True)
    return fd


# --------------------------------------------------------------------------- #
# Subprocess launchers — mirrors precoder_stream_roundtrip.py's setup but
# without log-collection threads (we read live).
# --------------------------------------------------------------------------- #
def launch_tx(args) -> subprocess.Popen:
    env = dict(os.environ, DEVOURER_PID=args.tx_pid, DEVOURER_VID=args.tx_vid,
               DEVOURER_CHANNEL=str(args.channel), DEVOURER_USB_QUIET="1")
    cmd = [args.tx_bin, "--interval-ms", str(args.interval_ms),
           "--max-psdu", str(args.body_bytes * 4 + 256)]
    return subprocess.Popen(cmd, env=env, stdin=subprocess.PIPE,
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                            bufsize=0)


def launch_rx(args) -> subprocess.Popen:
    env = dict(os.environ, DEVOURER_PID=args.rx_pid, DEVOURER_VID=args.rx_vid,
               DEVOURER_CHANNEL=str(args.channel),
               DEVOURER_STREAM_OUT="1", DEVOURER_USB_QUIET="1")
    return subprocess.Popen([args.rx_bin], env=env,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL,
                            text=True, bufsize=1)


def launch_duplex(args) -> subprocess.Popen:
    """Single binary, one chip, both directions. stdin = length-prefixed
    PSDU bodies (TX side), stdout = `<devourer-stream>` lines (RX side)."""
    env = dict(os.environ, DEVOURER_PID=args.duplex_pid,
               DEVOURER_VID=args.duplex_vid,
               DEVOURER_CHANNEL=str(args.channel), DEVOURER_USB_QUIET="1")
    cmd = [args.duplex_bin, "--interval-ms", str(args.interval_ms),
           "--max-psdu", str(args.body_bytes * 4 + 256)]
    # stdout text-mode so the RX thread reads lines; stdin binary (default
    # for Popen.stdin handle obtained via PIPE).
    return subprocess.Popen(cmd, env=env, stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL,
                            text=False, bufsize=0)


# --------------------------------------------------------------------------- #
# Worker threads
# --------------------------------------------------------------------------- #
class StopBit:
    """Tiny shared flag for clean shutdown. We never need full Event semantics
    so a single attribute is enough."""

    def __init__(self) -> None:
        self.stop = False


def tx_thread(stop: StopBit, tun_fd: int, tx_stdin, body_bytes: int,
              shape: Optional[dict], seed: int, offset: int, entry_state: int,
              repeat: int, counters: dict) -> None:
    seq = 0
    try:
        while not stop.stop:
            try:
                pkt = os.read(tun_fd, body_bytes)
            except OSError as e:
                if stop.stop:
                    return
                sys.stderr.write(f"tun read: {e}\n")
                return
            if not pkt:
                return
            if len(pkt) > body_bytes - stream.ENVELOPE_LEN:
                # Should never happen with --tun-mtu sized correctly.
                counters["tun_oversize"] += 1
                continue
            frame = stream.StreamFrame(seq=seq, total=0, payload=pkt)
            seq = (seq + 1) & 0xFFFF
            body, _ = stream.encode_body(
                frame, shape=shape, body_bytes=body_bytes,
                seed=seed, offset=offset, entry_state=entry_state,
            )
            chunk = struct.pack("<I", len(body)) + body
            try:
                for _ in range(repeat):
                    tx_stdin.write(chunk)
                tx_stdin.flush()
            except (BrokenPipeError, ValueError):
                return
            counters["tx_pkts"] += 1
            counters["tx_bytes"] += len(pkt)
    finally:
        stop.stop = True


class SeqWindow:
    """Sliding-window seq dedup. `size` keeps the last K seqs seen; the K+1'th
    pushes out the oldest. Wrap-aware in the sense that re-using a u16 seq
    after `size` other seqs have been seen is treated as a fresh packet (the
    old entry has aged out), so the window must be sized large enough to
    cover the worst-case TX-side --repeat fan-out at the expected packet
    rate but small enough that legitimate seq reuse after wrap is allowed.
    """

    def __init__(self, size: int) -> None:
        self.size = max(1, size)
        self._set: set[int] = set()
        self._order: collections.deque[int] = collections.deque()

    def seen_or_add(self, seq: int) -> bool:
        if seq in self._set:
            return True
        self._set.add(seq)
        self._order.append(seq)
        if len(self._order) > self.size:
            self._set.discard(self._order.popleft())
        return False


def rx_thread(stop: StopBit, rx_stdout, tun_fd: int,
              shape: Optional[dict], seed: int, offset: int, entry_state: int,
              dedup: Optional[SeqWindow], counters: dict) -> None:
    try:
        for line in rx_stdout:
            if stop.stop:
                return
            m = _STREAM_RE.search(line)
            if not m:
                continue
            if int(m.group("rate")) != 0x04:
                counters["rate_mismatch"] += 1
            body = bytes.fromhex(m.group("hex"))
            frame = stream.decode_body(
                body, shape=shape,
                seed=seed, offset=offset, entry_state=entry_state,
            )
            if frame is None:
                counters["malformed"] += 1
                continue
            if dedup is not None and dedup.seen_or_add(frame.seq):
                # Duplicate from --repeat fan-out (or a real radio repeat).
                # Drop before it ever touches the IP stack.
                counters["dedup_dropped"] += 1
                continue
            try:
                os.write(tun_fd, frame.payload)
            except OSError as e:
                sys.stderr.write(f"tun write: {e}\n")
                return
            counters["rx_pkts"] += 1
            counters["rx_bytes"] += len(frame.payload)
    finally:
        stop.stop = True


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def main(argv: Optional[list[str]] = None) -> int:
    repo = _HERE.parent.parent
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--mode", choices=("duplex", "tx-only", "rx-only",
                                       "duplex-split"),
                    default="duplex",
                    help="duplex (default) = one chip both directions via "
                         "StreamDuplexDemo (needs --duplex-pid); "
                         "duplex-split = two chips, --tx-pid + --rx-pid "
                         "(the pre-duplex layout); "
                         "tx-only / rx-only drop the unused subprocess")
    ap.add_argument("--duplex-pid", default=None,
                    help="DEVOURER_PID for the single chip in duplex mode")
    ap.add_argument("--duplex-vid", default="0x0bda")
    ap.add_argument("--tx-pid", default=None,
                    help="DEVOURER_PID for the TX adapter "
                         "(tx-only / duplex-split)")
    ap.add_argument("--rx-pid", default=None,
                    help="DEVOURER_PID for the RX adapter "
                         "(rx-only / duplex-split)")
    ap.add_argument("--tx-vid", default="0x0bda")
    ap.add_argument("--rx-vid", default="0x0bda")
    ap.add_argument("--channel", type=int, default=6)
    ap.add_argument("--tx-bin", default=str(repo / "build" / "StreamTxDemo"))
    ap.add_argument("--rx-bin", default=str(repo / "build" / "WiFiDriverDemo"))
    ap.add_argument("--duplex-bin",
                    default=str(repo / "build" / "StreamDuplexDemo"))
    ap.add_argument("--interval-ms", type=int, default=2)
    ap.add_argument("--repeat", type=int, default=1,
                    help="blind per-frame replication (combine with the "
                         "default --dedup to collapse the fan-out at RX so "
                         "the IP stack sees one packet per source packet)")
    ap.add_argument("--no-dedup", dest="dedup", action="store_false",
                    default=True,
                    help="disable RX-side seq dedup (default: on). With "
                         "--repeat>1 OR a real radio retransmission, leaving "
                         "this on prevents duplicate IP packets reaching the "
                         "kernel.")
    ap.add_argument("--dedup-window", type=int, default=4096,
                    help="dedup window size in distinct seqs (default 4096). "
                         "Bigger = tolerates higher --repeat at higher packet "
                         "rates; smaller = allows seq reuse sooner")
    ap.add_argument("--tun-name", default="dvr0")
    ap.add_argument("--tun-addr", default=None,
                    help="address/CIDR to assign to the TUN, e.g. "
                         "'10.99.0.1/24'. Omit to leave addressing to the "
                         "caller (e.g. `ip addr add` after we open).")
    ap.add_argument("--tun-mtu", type=int, default=None,
                    help="MTU for the TUN interface; default = body_bytes - "
                         "envelope (10 B). One IP packet fits in one stream "
                         "frame as long as this stays at or below that limit.")
    ap.add_argument("--body-bytes", type=int, default=1500,
                    help="stream body size (default 1500). Larger = fewer "
                         "frames per IP packet at the cost of longer airtime "
                         "per frame; smaller = the opposite.")
    ap.add_argument("--shape", default=None,
                    help="shape spec (e.g. '0:+1,8:-1,16:+1'); same caveat as "
                         "stream_tx.py — model-bound, not on-air-honoured at "
                         "the default offset/entry_state.")
    ap.add_argument("--seed", type=lambda s: int(s, 0),
                    default=stream.DEFAULT_SEED)
    ap.add_argument("--offset", type=int, default=0)
    ap.add_argument("--entry-state", type=lambda s: int(s, 0), default=0)
    ap.add_argument("--report-interval", type=float, default=5.0,
                    help="seconds between stderr counter prints; 0 = silent")
    args = ap.parse_args(argv)

    do_tx = args.mode in ("duplex", "duplex-split", "tx-only")
    do_rx = args.mode in ("duplex", "duplex-split", "rx-only")
    single_binary = args.mode == "duplex"
    if single_binary:
        if not args.duplex_pid:
            ap.error("--duplex-pid is required for --mode=duplex")
    else:
        if do_tx and not args.tx_pid:
            ap.error("--tx-pid is required unless --mode=rx-only")
        if do_rx and not args.rx_pid:
            ap.error("--rx-pid is required unless --mode=tx-only")
    if args.body_bytes < stream.ENVELOPE_LEN + 1:
        ap.error(f"--body-bytes must be >= {stream.ENVELOPE_LEN + 1}")
    # Byte mode requires body_bytes to be a whole-symbol count
    # (legacy 6M = 24 info bits/symbol = 3 bytes/symbol). Round up rather
    # than erroring so a hand-picked --body-bytes doesn't trip on first packet.
    bytes_per_sym = stream._LEGACY_BPSK.n_dbps // 8
    if args.body_bytes % bytes_per_sym != 0:
        rounded = ((args.body_bytes + bytes_per_sym - 1) // bytes_per_sym
                   * bytes_per_sym)
        sys.stderr.write(
            f"tun_p2p: --body-bytes {args.body_bytes} not a multiple of "
            f"{bytes_per_sym} (one OFDM symbol); rounding up to {rounded}\n"
        )
        args.body_bytes = rounded
    tun_mtu = args.tun_mtu if args.tun_mtu is not None else (
        args.body_bytes - stream.ENVELOPE_LEN)
    if tun_mtu > args.body_bytes - stream.ENVELOPE_LEN:
        ap.error(f"--tun-mtu {tun_mtu} exceeds payload capacity "
                 f"{args.body_bytes - stream.ENVELOPE_LEN} for body "
                 f"{args.body_bytes}B")

    shape = parse_shape(args.shape)
    sys.stderr.write(
        f"tun_p2p: mode={args.mode} tun={args.tun_name} "
        f"mtu={tun_mtu} body={args.body_bytes}B "
        f"shape={'on' if shape else 'off'}\n"
    )

    tun_fd = open_tun(args.tun_name, tun_mtu, args.tun_addr)
    stop = StopBit()
    counters = {
        "tx_pkts": 0, "tx_bytes": 0, "tun_oversize": 0,
        "rx_pkts": 0, "rx_bytes": 0, "malformed": 0, "rate_mismatch": 0,
        "dedup_dropped": 0,
    }
    dedup = SeqWindow(args.dedup_window) if args.dedup else None

    if single_binary:
        duplex_proc = launch_duplex(args)
        tx_proc = duplex_proc
        rx_proc = duplex_proc
        # Wrap the binary stdout in a text-mode reader for the existing
        # rx_thread's `for line in rx_stdout:` loop.
        import io
        rx_stdout = io.TextIOWrapper(duplex_proc.stdout, encoding="ascii",
                                     errors="replace", newline="\n")
    else:
        tx_proc = launch_tx(args) if do_tx else None
        rx_proc = launch_rx(args) if do_rx else None
        rx_stdout = rx_proc.stdout if rx_proc is not None else None

    threads: list[threading.Thread] = []
    if do_tx:
        assert tx_proc is not None and tx_proc.stdin is not None
        t = threading.Thread(
            target=tx_thread, daemon=True,
            args=(stop, tun_fd, tx_proc.stdin, args.body_bytes, shape,
                  args.seed, args.offset, args.entry_state, args.repeat,
                  counters),
        )
        t.start()
        threads.append(t)
    if do_rx:
        assert rx_stdout is not None
        t = threading.Thread(
            target=rx_thread, daemon=True,
            args=(stop, rx_stdout, tun_fd, shape,
                  args.seed, args.offset, args.entry_state, dedup, counters),
        )
        t.start()
        threads.append(t)

    def shutdown(*_):
        stop.stop = True
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    last_print = 0.0
    import time
    try:
        while not stop.stop:
            time.sleep(0.5)
            if args.report_interval and (time.monotonic() - last_print) >= args.report_interval:
                last_print = time.monotonic()
                sys.stderr.write(
                    f"tun_p2p: tx={counters['tx_pkts']}pkt/"
                    f"{counters['tx_bytes']}B "
                    f"rx={counters['rx_pkts']}pkt/{counters['rx_bytes']}B "
                    f"dedup-drop={counters['dedup_dropped']} "
                    f"mal={counters['malformed']} "
                    f"rate-mismatch={counters['rate_mismatch']} "
                    f"tun-oversize={counters['tun_oversize']}\n"
                )
    finally:
        stop.stop = True
        seen_procs: set[int] = set()
        for p in (tx_proc, rx_proc):
            if p is None or id(p) in seen_procs:
                continue
            seen_procs.add(id(p))
            try:
                p.terminate()
                p.wait(timeout=3)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                try:
                    p.kill()
                except ProcessLookupError:
                    pass
        try:
            os.close(tun_fd)
        except OSError:
            pass
        for t in threads:
            t.join(timeout=2)

    sys.stderr.write(
        f"tun_p2p: shutdown. final tx={counters['tx_pkts']}pkt/"
        f"{counters['tx_bytes']}B rx={counters['rx_pkts']}pkt/"
        f"{counters['rx_bytes']}B\n"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
