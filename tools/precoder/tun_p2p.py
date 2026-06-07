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

FEC MODE (RaptorQ, RFC 6330)

Pass --fec-k > 0 on BOTH peers to swap the byte-mode per-IP-packet framing
for a RaptorQ block code. Each block packs K source symbols' worth of
concatenated IP packets, then ships K + ceil(K*--fec-overhead) RaptorQ
encoded symbols. The receiver decodes once it has roughly K+ε of them,
regardless of which ones were dropped. Defaults to K=16, overhead=1.0,
which decodes reliably at ≤ ~40% loss.

  --fec-k 16 --fec-overhead 1.0     ≈ 50% airtime, recovers up to ~40% loss
  --fec-k 16 --fec-overhead 0.5     ≈ 33% airtime, recovers up to ~25% loss
  --fec-k 16 --fec-overhead 2.0     ≈ 67% airtime, recovers up to ~55% loss

Latency floor: K source packets × per-packet airtime + --fec-flush-ms.
At the default --fec-flush-ms=50 the bridge waits at most 50 ms before
force-encoding a partial block, so a single ping per second still flies
within ≈ 50 ms of encode buffer + 1 ms × number of envelopes shipped.

FEC mode forces dedup off (RaptorQ symbols self-dedup via SBN/ESI). The
periodic stderr report adds a `fec=[...]` segment with sym-tx/sym-rx/
blk-ok/blk-lost counters so you can tell at a glance whether the channel
is healthy enough for the current overhead setting.
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

# stream_fec pulls in `raptorq` from PyPI. Defer the import so byte-mode
# users running on a python without the wheel installed don't get
# blindsided at startup. We only need the module when --fec-k > 0.
stream_fec = None  # type: ignore[assignment]
def _import_stream_fec():  # noqa: E302
    global stream_fec
    if stream_fec is None:
        import stream_fec as _sf  # noqa: F401
        stream_fec = _sf
    return stream_fec

# Linux TUN/TAP constants. See <linux/if_tun.h>.
TUNSETIFF = 0x400454CA
IFF_TUN = 0x0001
IFF_NO_PI = 0x1000

_STREAM_RE = re.compile(
    r"<devourer-stream>rate=(?P<rate>\d+)\s+len=(?P<len>\d+)"
    r"(?:\s+crc_err=(?P<crc_err>\d+))?"
    r"(?:\s+icv_err=(?P<icv_err>\d+))?"
    r"(?:\s+rssi=(?P<rssi>-?\d+,-?\d+))?"
    r"(?:\s+evm=(?P<evm>-?\d+,-?\d+))?"
    r"(?:\s+snr=(?P<snr>-?\d+,-?\d+))?"
    r"\s+body=(?P<hex>[0-9a-fA-F]*)"
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
              repeat: int, counters: dict,
              fec_enc: "Optional[stream_fec.FecEncoder]" = None,
              fec_lock: "Optional[threading.Lock]" = None) -> None:
    """Read IP packets off the TUN and ship them through the stream link.

    In byte mode each packet is a single StreamFrame. In FEC mode each
    packet is added to the FecEncoder (concatenation-packed into source
    symbols, then RaptorQ-encoded when a K-symbol block fills); the
    encoder returns a list of inner-envelope blobs, each of which becomes
    its own StreamFrame.
    """
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

            if fec_enc is not None:
                # FEC mode: feed the packet to the encoder; ship whatever
                # envelopes come back. Most adds return [] until the K-th
                # source symbol triggers a block encode.
                if len(pkt) > fec_enc.cfg.max_packet_size:
                    counters["tun_oversize"] += 1
                    continue
                assert fec_lock is not None
                with fec_lock:
                    envelopes = fec_enc.add_packet(pkt)
                counters["tx_pkts"] += 1
                counters["tx_bytes"] += len(pkt)
            else:
                # Byte mode: one StreamFrame per IP packet.
                if len(pkt) > body_bytes - stream.ENVELOPE_LEN:
                    counters["tun_oversize"] += 1
                    continue
                envelopes = [pkt]
                counters["tx_pkts"] += 1
                counters["tx_bytes"] += len(pkt)

            for env in envelopes:
                frame = stream.StreamFrame(seq=seq, total=0, payload=env)
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
                if fec_enc is not None:
                    counters["fec_symbols_tx"] += 1
    finally:
        stop.stop = True


def fec_flush_thread(stop: StopBit, tx_stdin, body_bytes: int,
                     shape: Optional[dict], seed: int, offset: int,
                     entry_state: int, repeat: int, counters: dict,
                     fec_enc: "stream_fec.FecEncoder",
                     fec_lock: "threading.Lock",
                     flush_ms: int) -> None:
    """Periodically flush the FecEncoder's pending block. Without this, a
    sparse traffic pattern (single ping per second) would never accumulate
    K source packets and the link would go silent until traffic spiked.

    Maintains its own seq counter — in FEC mode the outer SeqWindow dedup
    is off (RaptorQ symbols are inherently dedup-friendly via SBN+ESI), so
    the tx_thread's and flush_thread's seq spaces don't need to share or
    coordinate.
    """
    import time as _time
    interval = max(0.001, flush_ms / 1000.0)
    seq = 0x8000  # arbitrary mid-range start; just a counter, not significant
    while not stop.stop:
        _time.sleep(interval)
        if stop.stop:
            return
        with fec_lock:
            if fec_enc.pending_packets == 0:
                continue
            envelopes = fec_enc.flush()
        for env in envelopes:
            frame = stream.StreamFrame(seq=seq, total=0, payload=env)
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
            counters["fec_symbols_tx"] += 1
        counters["fec_flushes"] += 1


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
              dedup: Optional[SeqWindow], counters: dict,
              fec_dec: "Optional[stream_fec.FecDecoder]" = None,
              fec_block_expire_ms: int = 500) -> None:
    """Read `<devourer-stream>` lines off the WiFiDriverDemo, decode the
    stream envelope, then either:
      * byte mode: write the StreamFrame payload to the TUN as one IP packet,
      * FEC mode: feed the payload to the FecDecoder; when a block decodes,
        write each unpacked IP packet to the TUN.

    Block expiry runs on a cheap monotonic-clock check (once per second
    is enough; we don't want to pay it per symbol).
    """
    import time as _time
    last_expire = _time.monotonic()
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
                counters["dedup_dropped"] += 1
                continue

            if fec_dec is not None:
                ip_pkts = fec_dec.add_symbol(frame.payload)
                counters["fec_symbols_rx"] += 1
                for pkt in ip_pkts:
                    try:
                        os.write(tun_fd, pkt)
                    except OSError as e:
                        sys.stderr.write(f"tun write: {e}\n")
                        return
                    counters["rx_pkts"] += 1
                    counters["rx_bytes"] += len(pkt)
                # Periodic expiry — cheap; once a second is plenty.
                now = _time.monotonic()
                if now - last_expire >= 1.0:
                    last_expire = now
                    fec_dec.expire_blocks_older_than(
                        fec_block_expire_ms / 1000.0)
                    # Bridge counter mirrors the decoder's own counter so
                    # the periodic stderr report sees it.
                    counters["fec_blocks_unrecoverable"] = (
                        fec_dec.blocks_unrecoverable)
                    counters["fec_blocks_decoded"] = fec_dec.blocks_decoded
            else:
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

    # FEC mode (RaptorQ, RFC 6330). When --fec-k > 0, packets are
    # concatenation-packed into K source symbols per block, RaptorQ-encoded
    # into K + ceil(K*overhead) output symbols, and shipped through the
    # existing stream framing. RX dedup is forced off — RaptorQ symbols are
    # inherently dedup-friendly via SBN+ESI and the codec just discards any
    # excess once the block has decoded.
    ap.add_argument("--fec-k", type=int, default=0,
                    help="source symbols per RaptorQ block; 0 = FEC off "
                         "(default). Sensible range 8-64; latency floor "
                         "scales with K.")
    ap.add_argument("--fec-overhead", type=float, default=1.0,
                    help="repair-to-source ratio (default 1.0 = 50%% airtime "
                         "overhead, decodes reliably at ~40%% loss). Raise "
                         "for noisier links, lower for clean ones.")
    ap.add_argument("--fec-symbol-size", type=int, default=1477,
                    help="bytes per RaptorQ symbol (default 1477, fits one "
                         "1500 B stream body with the 9 B FEC + 4 B RaptorQ "
                         "headers). Must be the same on both peers.")
    ap.add_argument("--fec-flush-ms", type=int, default=50,
                    help="ms before a partial block is force-encoded (default "
                         "50). At sparse traffic this bounds the encoder-side "
                         "latency floor.")
    ap.add_argument("--fec-block-expire-ms", type=int, default=500,
                    help="ms before the RX side gives up on a block whose "
                         "first symbol arrived this long ago (default 500).")
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

    # FEC config / sanity. When FEC is on, force-off the byte-mode dedup
    # (RaptorQ has its own block-level dedup via SBN/ESI) and cap the
    # tun MTU at the symbol size minus the per-packet length prefix so a
    # max-size packet still fits in one source symbol.
    fec_cfg = None
    if args.fec_k > 0:
        _import_stream_fec()
        fec_cfg = stream_fec.FecConfig(
            k=args.fec_k,
            symbol_size=args.fec_symbol_size,
            overhead=args.fec_overhead,
        )
        # body must hold the stream envelope (10 B) + inner FEC header
        # (9 B) + the raptorq packet (≤ symbol_size B; cberner's lib caps
        # packet bytes at the configured `mtu`).
        min_body = (fec_cfg.symbol_size
                    + stream_fec.FEC_HEADER_LEN
                    + stream.ENVELOPE_LEN)
        if args.body_bytes < min_body:
            sys.stderr.write(
                f"tun_p2p: --body-bytes {args.body_bytes} too small for FEC "
                f"symbol_size {fec_cfg.symbol_size}; need at least "
                f"~{min_body}. Increase --body-bytes or lower "
                f"--fec-symbol-size.\n"
            )
            return 2
        fec_tun_cap = fec_cfg.max_packet_size
        if tun_mtu > fec_tun_cap:
            sys.stderr.write(
                f"tun_p2p: clamping --tun-mtu {tun_mtu} → {fec_tun_cap} "
                f"(FEC max_packet_size with symbol={fec_cfg.symbol_size})\n"
            )
            tun_mtu = fec_tun_cap
        if args.dedup:
            sys.stderr.write(
                "tun_p2p: FEC on → forcing --no-dedup (RaptorQ symbols dedup "
                "themselves; the outer SeqWindow would just waste cycles)\n"
            )
            args.dedup = False

    tun_fd = open_tun(args.tun_name, tun_mtu, args.tun_addr)
    stop = StopBit()
    counters = {
        "tx_pkts": 0, "tx_bytes": 0, "tun_oversize": 0,
        "rx_pkts": 0, "rx_bytes": 0, "malformed": 0, "rate_mismatch": 0,
        "dedup_dropped": 0,
        # FEC counters; stay at 0 in byte mode.
        "fec_symbols_tx": 0, "fec_symbols_rx": 0, "fec_flushes": 0,
        "fec_blocks_decoded": 0, "fec_blocks_unrecoverable": 0,
    }
    dedup = SeqWindow(args.dedup_window) if args.dedup else None
    fec_enc = stream_fec.FecEncoder(fec_cfg) if fec_cfg else None
    fec_dec = stream_fec.FecDecoder(fec_cfg) if fec_cfg else None
    fec_lock = threading.Lock() if fec_enc else None

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
                  counters, fec_enc, fec_lock),
        )
        t.start()
        threads.append(t)
        if fec_enc is not None:
            t = threading.Thread(
                target=fec_flush_thread, daemon=True,
                args=(stop, tx_proc.stdin, args.body_bytes, shape,
                      args.seed, args.offset, args.entry_state, args.repeat,
                      counters, fec_enc, fec_lock, args.fec_flush_ms),
            )
            t.start()
            threads.append(t)
    if do_rx:
        assert rx_stdout is not None
        t = threading.Thread(
            target=rx_thread, daemon=True,
            args=(stop, rx_stdout, tun_fd, shape,
                  args.seed, args.offset, args.entry_state, dedup, counters,
                  fec_dec, args.fec_block_expire_ms),
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
                fec_str = ""
                if fec_cfg is not None:
                    fec_str = (
                        f" fec=[K={fec_cfg.k} sym-tx={counters['fec_symbols_tx']} "
                        f"sym-rx={counters['fec_symbols_rx']} "
                        f"blk-ok={counters['fec_blocks_decoded']} "
                        f"blk-lost={counters['fec_blocks_unrecoverable']} "
                        f"flush={counters['fec_flushes']}]"
                    )
                sys.stderr.write(
                    f"tun_p2p: tx={counters['tx_pkts']}pkt/"
                    f"{counters['tx_bytes']}B "
                    f"rx={counters['rx_pkts']}pkt/{counters['rx_bytes']}B "
                    f"dedup-drop={counters['dedup_dropped']} "
                    f"mal={counters['malformed']} "
                    f"rate-mismatch={counters['rate_mismatch']} "
                    f"tun-oversize={counters['tun_oversize']}"
                    f"{fec_str}\n"
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
