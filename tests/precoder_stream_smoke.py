"""Repo-level smoke for the stream-link layer (tools/precoder/stream.py).

Pairs with the exhaustive in-subtree tests in tools/precoder/test_stream.py
(36 cases, requires the uv env). This file is what CI / a casual contributor
runs without the uv toolchain — it skips cleanly when numpy isn't installed
on the system Python.

It drives the same encode → simulated-on-wire → decode path the two-adapter
hardware harness uses, but synthesises the `<devourer-stream>` line that
WiFiDriverDemo would print, so no USB is involved.

The smoke covers:
  * byte-mode round-trip — frame the input, encode, decode, reassemble;
  * shape-mode round-trip — same, plus verifying each encoded body's pinned
    subcarriers match the requested ±1 under `emulate_chip` (model-bound;
    on-air verification still needs SDR / BB-dbgport).
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")

PRECODER = Path(__file__).resolve().parent.parent / "tools" / "precoder"
sys.path.insert(0, str(PRECODER))
import encode_subcarriers as enc  # noqa: E402
import stream  # noqa: E402


def _frame_psdu_for_wire(body: bytes) -> bytes:
    """Round-trip the StreamTxDemo wire format: length-prefixed PSDU body."""
    return struct.pack("<I", len(body)) + body


def _parse_length_prefixed(buf: bytes) -> list[bytes]:
    out: list[bytes] = []
    i = 0
    while i < len(buf):
        (n,) = struct.unpack("<I", buf[i:i + 4])
        i += 4
        out.append(buf[i:i + n])
        i += n
    return out


def _build_shape_dict() -> dict:
    return {0: +1, 8: -1, 16: +1, 24: -1, 32: +1}


def test_byte_mode_repo_smoke():
    data = bytes(range(140))  # 10 frames at the default mtu=14
    frames = stream.pack_stream(data)
    bodies = []
    for f in frames:
        body, _ = stream.encode_body(f)
        bodies.append(body)

    # Simulate the wire: length-prefixed PSDU bodies in, the same bodies
    # back out (StreamTxDemo emits them, demo's DEVOURER_STREAM_OUT dumps
    # them — identity on the byte plane).
    wire = b"".join(_frame_psdu_for_wire(b) for b in bodies)
    rx_bodies = _parse_length_prefixed(wire)
    assert rx_bodies == bodies

    decoded = [stream.decode_body(b) for b in rx_bodies]
    assert all(d is not None for d in decoded)
    by_seq = {d.seq: d.payload for d in decoded}
    assert b"".join(by_seq[s] for s in sorted(by_seq)) == data


def test_shape_mode_repo_smoke():
    data = bytes(range(80))
    shape = _build_shape_dict()
    frames = stream.pack_stream(data)
    bodies_layouts = [stream.encode_body(f, shape=shape) for f in frames]

    # Per-body shape check: emulate_chip(encoded_body) must put the
    # requested ±1 at every pinned subcarrier of every body symbol.
    for body, layout in bodies_layouts:
        psdu_bits = enc.bytes_to_bits(body)[: layout.n_sym * stream._LEGACY_BPSK.n_dbps]
        sub = enc.emulate_chip(psdu_bits, stream.DEFAULT_SEED, stream._LEGACY_BPSK,
                               n_sym=layout.n_sym, offset=0, entry_state=0)
        for sc, want in shape.items():
            assert np.all(sub[:, sc] == want), \
                f"shape violation at subcarrier {sc}"

    # Wire-roundtrip the bodies and re-decode with the same shape.
    wire = b"".join(_frame_psdu_for_wire(b) for b, _ in bodies_layouts)
    rx_bodies = _parse_length_prefixed(wire)
    decoded = [stream.decode_body(b, shape=shape) for b in rx_bodies]
    assert all(d is not None for d in decoded)
    by_seq = {d.seq: d.payload for d in decoded}
    assert b"".join(by_seq[s] for s in sorted(by_seq)) == data


def test_shape_mode_grows_body_when_pin_rank_rises():
    """As more subcarriers are pinned, per-symbol capacity drops and the body
    must grow to fit the envelope at the reduced rate."""
    f = stream.StreamFrame(seq=0, total=1, payload=b"x" * 14)  # max byte-mode payload
    body0, layout0 = stream.encode_body(f)
    assert layout0.body_bytes == stream.DEFAULT_BODY_BYTES
    body1, layout1 = stream.encode_body(f, shape={0: +1, 8: -1, 16: +1, 24: -1, 32: +1})
    assert layout1.body_bytes > layout0.body_bytes
    assert layout1.capacity_per_sym < layout0.capacity_per_sym
