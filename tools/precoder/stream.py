"""Stream framing on top of the precoder.

Wraps PSDU bodies in a tiny seq/length/CRC envelope so a TX→RX pair can carry
a sequenced byte stream over the legacy-6M-OFDM probe-request link from
precoder. Optionally also encodes a per-OFDM-symbol shape constraint
(pinned per-subcarrier ±1 values) on every body symbol — the remaining
null-space dimensions of the BCC+interleaver linear map carry the framing
bits, so one frame demonstrates frequency-domain control AND sequenced data
delivery in the same packet.

Frame layout (body bytes after the MAC header, before the chip's scrambler):

    MAGIC (2)   = 0xD5DE little-endian
    SEQ   (2)   = little-endian wrapping counter
    TOTAL (2)   = total frames in this message (0 = unbounded stream)
    PLEN  (2)   = payload length in bytes
    PAYLOAD (PLEN bytes)
    PAD     (body - 10 - PLEN bytes; byte-mode only, zeros)
    CRC16 (2)   = CRC-16-CCITT(0xFFFF init) over MAGIC..PAD, little-endian

Envelope overhead is 10 bytes. A 24-byte body therefore carries up to 14 bytes
of payload when no shape is requested; with shape, the body grows to fit (see
`plan_body` / `encode_body` for the math).

CONVENTION — the bytes returned/consumed here are the DESCRAMBLED body bytes.
They go directly into `send_packet`'s 802.11 body (the chip applies its own
scrambler before BCC) and they come directly out of devourer's
`DEVOURER_DUMP_BODY` (the chip has already descrambled them on RX). The chip
scrambler is therefore invisible to the byte-stream caller; only the shape
codec needs to know the scrambler seed/offset to invert the per-subcarrier
constraint mapping.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Iterator, Optional, Union

import numpy as np

from encode_subcarriers import (
    DEFAULT_SEED,
    PhyParams,
    _LEGACY_BPSK,
    bcc_encode,
    bcc_final_state,
    bits_to_bytes,
    bpsk_demap,
    bytes_to_bits,
    interleave,
    scrambler_sequence,
)


MAGIC = 0xD5DE
HEADER_LEN = 8            # MAGIC + SEQ + TOTAL + PLEN
TRAILER_LEN = 2           # CRC16
ENVELOPE_LEN = HEADER_LEN + TRAILER_LEN   # 10 bytes overhead

# Default body size matches precoder's 24-byte probe-request body.
DEFAULT_BODY_BYTES = 24


# --------------------------------------------------------------------------- #
# CRC-16-CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xorout)
# --------------------------------------------------------------------------- #
def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    crc = init & 0xFFFF
    for byte in data:
        crc ^= (byte & 0xFF) << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# --------------------------------------------------------------------------- #
# Generator matrix for one OFDM symbol
# --------------------------------------------------------------------------- #
def compute_generator(phy: PhyParams = _LEGACY_BPSK) -> np.ndarray:
    """M ∈ GF(2)^{n_cbps × n_dbps} so sub = M @ scrambled_info ⊕ b(entry_state).

    The BCC+interleaver pipeline is GF(2)-linear in the per-symbol scrambled
    input bits, with the 6-bit BCC entry_state contributing an affine offset
    (see `compute_state_offset`). M is built by running the forward pipeline
    on each unit input vector from a zero entry state.
    """
    M = np.zeros((phy.n_cbps, phy.n_dbps), dtype=np.uint8)
    for k in range(phy.n_dbps):
        unit = np.zeros(phy.n_dbps, dtype=np.uint8)
        unit[k] = 1
        coded = bcc_encode(unit, init_state=0)
        sub = interleave(coded, phy)
        M[:, k] = sub
    return M


def compute_state_offset(entry_state: int,
                         phy: PhyParams = _LEGACY_BPSK) -> np.ndarray:
    """b(entry_state) = pipeline(all-zero scrambled-input, entry_state).

    Encodes the affine constant in sub = M @ scrambled_info ⊕ b(entry_state).
    """
    coded = bcc_encode(np.zeros(phy.n_dbps, dtype=np.uint8),
                       init_state=entry_state)
    return interleave(coded, phy)


# --------------------------------------------------------------------------- #
# GF(2) linear algebra
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class GF2Solution:
    consistent: bool
    particular: np.ndarray     # (n,)
    null_basis: np.ndarray     # (n, k); each column is a kernel vector
    free_cols: np.ndarray      # (k,); column indices the free vars live at

    @property
    def n_free(self) -> int:
        return int(self.free_cols.size)


def gf2_solve(A: np.ndarray, b: np.ndarray) -> GF2Solution:
    """Solve A x = b over GF(2) by RREF.

    `null_basis` columns are constructed so column j has exactly one `1` at
    row `free_cols[j]` and zero at every other row in `free_cols`; that
    convention makes the encoder/decoder symmetric — encode sets `x[free_cols[j]]
    = data[j]`, decode reads `data[j] = x[free_cols[j]] ⊕ particular[free_cols[j]]`.

    When `A` has zero rows (no constraints) the identity basis on n variables
    is returned, with `free_cols = arange(n)`.
    """
    A = np.asarray(A, dtype=np.uint8).copy()
    b = np.asarray(b, dtype=np.uint8).copy()
    m, n = A.shape
    if m != b.shape[0]:
        raise ValueError(f"A rows ({m}) != b length ({b.shape[0]})")
    if m == 0:
        return GF2Solution(
            consistent=True,
            particular=np.zeros(n, dtype=np.uint8),
            null_basis=np.eye(n, dtype=np.uint8),
            free_cols=np.arange(n, dtype=np.int64),
        )
    aug = np.column_stack([A, b])
    pivot_cols: list[int] = []
    pivot_rows: list[int] = []
    row = 0
    for col in range(n):
        pivot = -1
        for r in range(row, m):
            if aug[r, col]:
                pivot = r
                break
        if pivot < 0:
            continue
        if pivot != row:
            aug[[row, pivot]] = aug[[pivot, row]]
        for r in range(m):
            if r != row and aug[r, col]:
                aug[r] ^= aug[row]
        pivot_cols.append(col)
        pivot_rows.append(row)
        row += 1
        if row == m:
            break
    for r in range(row, m):
        if aug[r, n]:
            return GF2Solution(
                consistent=False,
                particular=np.zeros(n, dtype=np.uint8),
                null_basis=np.zeros((n, 0), dtype=np.uint8),
                free_cols=np.empty(0, dtype=np.int64),
            )
    particular = np.zeros(n, dtype=np.uint8)
    for pr, pc in zip(pivot_rows, pivot_cols):
        particular[pc] = aug[pr, n]
    pivot_set = set(pivot_cols)
    free_cols = np.array([c for c in range(n) if c not in pivot_set],
                         dtype=np.int64)
    null = np.zeros((n, free_cols.size), dtype=np.uint8)
    for j, fc in enumerate(free_cols):
        null[fc, j] = 1
        for pr, pc in zip(pivot_rows, pivot_cols):
            null[pc, j] = aug[pr, fc]
    return GF2Solution(
        consistent=True,
        particular=particular,
        null_basis=null,
        free_cols=free_cols,
    )


def gf2_rank(A: np.ndarray) -> int:
    if A.shape[0] == 0:
        return 0
    sol = gf2_solve(A, np.zeros(A.shape[0], dtype=np.uint8))
    return A.shape[1] - sol.n_free


# --------------------------------------------------------------------------- #
# Shape spec
# --------------------------------------------------------------------------- #
ShapeSpec = Union[np.ndarray, dict, None]


def normalise_shape(shape: ShapeSpec, n_sd: int) -> tuple[np.ndarray, np.ndarray]:
    """Normalise a shape spec to (pinned_subcarrier_indices, pinned_bits).

    Accepts either a dict {sc_idx: ±1} or an array of length n_sd with 0 =
    don't care, ±1 = pinned. Returns the sorted index array and the matching
    uint8 bit values (0 for +1, 1 for −1).
    """
    if shape is None:
        return np.empty(0, dtype=np.int64), np.empty(0, dtype=np.uint8)
    if isinstance(shape, dict):
        items = sorted(shape.items())
        idx = np.array([k for k, _ in items], dtype=np.int64)
        vals = np.array([v for _, v in items], dtype=np.int8)
    else:
        arr = np.asarray(shape, dtype=np.int8)
        if arr.shape != (n_sd,):
            raise ValueError(f"shape array must have length {n_sd}, got {arr.shape}")
        idx = np.nonzero(arr != 0)[0].astype(np.int64)
        vals = arr[idx]
    if idx.size:
        if idx.min() < 0 or idx.max() >= n_sd:
            raise ValueError(f"shape index out of range [0, {n_sd})")
        if len(set(idx.tolist())) != len(idx):
            raise ValueError("shape has duplicate subcarrier indices")
        if not np.all(np.isin(vals, [1, -1])):
            raise ValueError("shape values must be +1 or -1")
    return idx, bpsk_demap(vals).astype(np.uint8)


# --------------------------------------------------------------------------- #
# Stream frame envelope
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class StreamFrame:
    seq: int
    total: int
    payload: bytes

    def envelope_bytes(self, body_bytes: int) -> bytes:
        """Build the (descrambled) envelope image at exactly `body_bytes` bytes.

        Layout: HEADER(8) | PAYLOAD(plen) | CRC(2) | PAD(body - 10 - plen).
        CRC sits immediately after the payload — at the PLEN-determined offset
        — so a parser can locate it from PLEN alone, ignoring any trailing
        bytes the chip appends (FCS, RX trailer) or the encoder adds for
        body-size alignment.
        """
        max_payload = body_bytes - ENVELOPE_LEN
        if len(self.payload) > max_payload:
            raise ValueError(
                f"payload {len(self.payload)}B exceeds capacity "
                f"{max_payload}B (body={body_bytes}B)"
            )
        out = bytearray()
        out += MAGIC.to_bytes(2, "little")
        out += (self.seq & 0xFFFF).to_bytes(2, "little")
        out += (self.total & 0xFFFF).to_bytes(2, "little")
        out += len(self.payload).to_bytes(2, "little")
        out += bytes(self.payload)
        crc = crc16_ccitt(bytes(out))
        out += crc.to_bytes(2, "little")
        pad = body_bytes - len(out)
        if pad > 0:
            out += bytes(pad)
        return bytes(out)


def parse_envelope(body: bytes) -> Optional[StreamFrame]:
    """Inverse of `StreamFrame.envelope_bytes`. Returns None on a magic /
    CRC / PLEN mismatch (treated as "not one of our frames").

    PLEN is authoritative for envelope size, so trailing bytes added by the
    chip (typically the 4-byte 802.11 FCS plus any RX trailer that survives
    `examples/rx/main.cpp`'s body dump) are ignored. The CRC is read from
    `HEADER_LEN + plen .. HEADER_LEN + plen + 2`, NOT from `body[-2:]`.
    """
    if len(body) < ENVELOPE_LEN:
        return None
    if int.from_bytes(body[0:2], "little") != MAGIC:
        return None
    plen = int.from_bytes(body[6:8], "little")
    envelope_size = HEADER_LEN + plen + TRAILER_LEN
    if envelope_size > len(body):
        return None
    expected = int.from_bytes(
        body[envelope_size - TRAILER_LEN:envelope_size], "little")
    actual = crc16_ccitt(bytes(body[:envelope_size - TRAILER_LEN]))
    if expected != actual:
        return None
    seq = int.from_bytes(body[2:4], "little")
    total = int.from_bytes(body[4:6], "little")
    payload = bytes(body[HEADER_LEN:HEADER_LEN + plen])
    return StreamFrame(seq=seq, total=total, payload=payload)


# --------------------------------------------------------------------------- #
# Body planning
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class BodyLayout:
    """Resolved geometry for one shaped or byte-mode body.

    `n_sym` is the count of OFDM data symbols the body spans; `body_bytes` is
    `n_sym × n_dbps / 8` (legacy: 3 bytes/symbol). `capacity_per_sym` is the
    data-bit budget per OFDM symbol after the shape constraint (= n_dbps when
    no shape, = n_dbps - rank(M[S,:]) otherwise).
    """
    body_bytes: int
    n_sym: int
    capacity_per_sym: int
    payload_capacity: int


def plan_body(payload_len: int,
              shape: ShapeSpec = None,
              *,
              body_bytes: Optional[int] = None,
              phy: PhyParams = _LEGACY_BPSK) -> BodyLayout:
    """Resolve n_sym, body_bytes, and per-symbol capacity for a given payload
    size and shape spec.

    Without shape: body is fixed at `body_bytes` (default 24); payload must
    fit in `body_bytes - ENVELOPE_LEN`. With shape: capacity per symbol drops
    to `n_dbps - rank(M[S,:])` and `n_sym` grows to fit the envelope at this
    reduced rate; `body_bytes`, when passed, acts as a floor.
    """
    if phy.n_dbps % 8 != 0:
        raise NotImplementedError(
            f"this PoC assumes n_dbps ({phy.n_dbps}) is a multiple of 8 "
            "so one OFDM symbol = whole bytes; legacy 6M (24) satisfies this"
        )
    bytes_per_sym = phy.n_dbps // 8

    pin_idx, _ = normalise_shape(shape, phy.n_sd)
    if pin_idx.size == 0:
        if body_bytes is None:
            body_bytes = DEFAULT_BODY_BYTES
        if body_bytes % bytes_per_sym != 0:
            raise ValueError(
                f"body_bytes ({body_bytes}) must be a multiple of "
                f"{bytes_per_sym} (one OFDM symbol)"
            )
        if payload_len > body_bytes - ENVELOPE_LEN:
            raise ValueError(
                f"payload {payload_len}B exceeds {body_bytes - ENVELOPE_LEN}B "
                f"(body={body_bytes}B, envelope={ENVELOPE_LEN}B)"
            )
        return BodyLayout(
            body_bytes=body_bytes,
            n_sym=body_bytes // bytes_per_sym,
            capacity_per_sym=phy.n_dbps,
            payload_capacity=body_bytes - ENVELOPE_LEN,
        )

    M = compute_generator(phy)
    rank = gf2_rank(M[pin_idx, :])
    capacity = phy.n_dbps - rank
    if capacity == 0:
        raise ValueError(
            "shape pins all 24 degrees of freedom (rank=24); no room for "
            "framing bits — drop a pin or switch to byte mode"
        )
    envelope_bits = (ENVELOPE_LEN + payload_len) * 8
    needed_sym = (envelope_bits + capacity - 1) // capacity
    floor_sym = (body_bytes or 0) // bytes_per_sym
    n_sym = max(needed_sym, floor_sym)
    return BodyLayout(
        body_bytes=n_sym * bytes_per_sym,
        n_sym=n_sym,
        capacity_per_sym=capacity,
        payload_capacity=(n_sym * capacity) // 8 - ENVELOPE_LEN,
    )


# --------------------------------------------------------------------------- #
# Body encode / decode
# --------------------------------------------------------------------------- #
def encode_body(
    frame: StreamFrame,
    shape: ShapeSpec = None,
    *,
    body_bytes: Optional[int] = None,
    phy: PhyParams = _LEGACY_BPSK,
    seed: int = DEFAULT_SEED,
    offset: int = 0,
    entry_state: int = 0,
) -> tuple[bytes, BodyLayout]:
    """Build the descrambled body bytes for one stream frame.

    Returns (body, layout). Byte mode (`shape=None`): returns the envelope
    sized to `layout.body_bytes`. Shape mode: each symbol's info bits are
    chosen so that the chip's on-air subcarriers honour the pin pattern AND
    the symbol's free dimensions carry the envelope bits.

    `offset` and `entry_state` describe the start of the body within a real
    PSDU's scrambler/BCC stream (README's offset=208 / entry_state derived
    from SERVICE+MAC-header example). Byte mode ignores them.
    """
    layout = plan_body(len(frame.payload), shape, body_bytes=body_bytes, phy=phy)

    if shape is None:
        return frame.envelope_bytes(layout.body_bytes), layout

    # Shape mode: the envelope is byte-tight (10 + payload bytes) and its bits
    # are bit-packed at `capacity_per_sym` bits/symbol across `n_sym` symbols.
    envelope = frame.envelope_bytes(ENVELOPE_LEN + len(frame.payload))
    envelope_bits = bytes_to_bits(envelope)
    slot_bits = layout.n_sym * layout.capacity_per_sym
    assert envelope_bits.size <= slot_bits, "plan_body undersized the shaped body"
    packed = np.zeros(slot_bits, dtype=np.uint8)
    packed[:envelope_bits.size] = envelope_bits

    pin_idx, pin_vals = normalise_shape(shape, phy.n_sd)
    M = compute_generator(phy)
    M_S = M[pin_idx, :]

    body_info_bits = np.empty(layout.n_sym * phy.n_dbps, dtype=np.uint8)
    state = entry_state & 0x3F
    cur_offset = offset
    for i in range(layout.n_sym):
        b_state = compute_state_offset(state, phy)
        scr_chunk = scrambler_sequence(seed, cur_offset + phy.n_dbps)[cur_offset:]
        rhs = pin_vals.copy()
        rhs ^= (M_S @ scr_chunk) & 1
        rhs ^= b_state[pin_idx]
        sol = gf2_solve(M_S, rhs)
        if not sol.consistent:
            raise ValueError(
                f"shape pin values are inconsistent at symbol {i} "
                f"(state={state:#x}, offset={cur_offset}); the pinned "
                "subcarriers are linearly dependent with conflicting "
                "targets — pick a different ±1 pattern on the pinned set"
            )
        data_slice = packed[i * layout.capacity_per_sym:
                            (i + 1) * layout.capacity_per_sym]
        info_i = (sol.particular ^ ((sol.null_basis @ data_slice) & 1)).astype(np.uint8)
        body_info_bits[i * phy.n_dbps:(i + 1) * phy.n_dbps] = info_i
        scrambled = info_i ^ scr_chunk
        state = bcc_final_state(scrambled, init_state=state)
        cur_offset += phy.n_dbps

    return bits_to_bytes(body_info_bits), layout


def decode_body(
    body: bytes,
    shape: ShapeSpec = None,
    *,
    phy: PhyParams = _LEGACY_BPSK,
    seed: int = DEFAULT_SEED,
    offset: int = 0,
    entry_state: int = 0,
) -> Optional[StreamFrame]:
    """Recover a StreamFrame from a received descrambled body.

    Byte mode: the body IS the envelope (passthrough to parse_envelope).
    Shape mode: per-symbol info bits are projected onto the null basis to
    recover the embedded envelope bits. Returns None on magic/CRC/PLEN
    failure (treated as "not one of our frames").
    """
    if shape is None:
        return parse_envelope(bytes(body))

    pin_idx, pin_vals = normalise_shape(shape, phy.n_sd)
    M = compute_generator(phy)
    M_S = M[pin_idx, :]
    rank = gf2_rank(M_S)
    capacity = phy.n_dbps - rank
    if capacity == 0:
        return None

    bytes_per_sym = phy.n_dbps // 8
    if len(body) < bytes_per_sym:
        return None
    n_sym_avail = len(body) // bytes_per_sym
    info_bits = bytes_to_bits(bytes(body))

    # Decode just enough symbols to cover the envelope; trailer / FCS /
    # chip-padding past that isn't shape-encoded and would trip the
    # null-space check below. PLEN is known only after the first
    # ceil(HEADER_LEN*8 / capacity) symbols, so the limit grows mid-loop.
    n_sym_header = (HEADER_LEN * 8 + capacity - 1) // capacity
    if n_sym_avail < n_sym_header:
        return None

    recovered = np.zeros(n_sym_avail * capacity, dtype=np.uint8)
    n_sym_needed = n_sym_header
    state = entry_state & 0x3F
    cur_offset = offset
    decoded = 0
    for i in range(n_sym_avail):
        b_state = compute_state_offset(state, phy)
        scr_chunk = scrambler_sequence(seed, cur_offset + phy.n_dbps)[cur_offset:]
        info_i = info_bits[i * phy.n_dbps:(i + 1) * phy.n_dbps].copy()
        rhs = pin_vals.copy()
        rhs ^= (M_S @ scr_chunk) & 1
        rhs ^= b_state[pin_idx]
        sol = gf2_solve(M_S, rhs)
        if not sol.consistent:
            return None
        delta = (info_i ^ sol.particular) & 1
        data_slice = delta[sol.free_cols].astype(np.uint8)
        if not np.array_equal((sol.null_basis @ data_slice) & 1, delta):
            return None
        recovered[i * capacity:(i + 1) * capacity] = data_slice
        scrambled = info_i ^ scr_chunk
        state = bcc_final_state(scrambled, init_state=state)
        cur_offset += phy.n_dbps
        decoded = i + 1

        if decoded == n_sym_header:
            header_bytes_dec = bits_to_bytes(recovered[:HEADER_LEN * 8])
            if int.from_bytes(header_bytes_dec[0:2], "little") != MAGIC:
                return None
            plen = int.from_bytes(header_bytes_dec[6:8], "little")
            envelope_size = ENVELOPE_LEN + plen
            n_sym_needed = (envelope_size * 8 + capacity - 1) // capacity
            if n_sym_needed > n_sym_avail:
                return None
        if decoded >= n_sym_needed:
            break

    if decoded < n_sym_needed:
        return None
    header_bytes = bits_to_bytes(recovered[:HEADER_LEN * 8])
    plen = int.from_bytes(header_bytes[6:8], "little")
    envelope = bits_to_bytes(recovered[:(ENVELOPE_LEN + plen) * 8])
    return parse_envelope(envelope)


# --------------------------------------------------------------------------- #
# Stream helpers — chunk a byte buffer into frames and back
# --------------------------------------------------------------------------- #
def pack_stream(data: bytes,
                mtu: int = DEFAULT_BODY_BYTES - ENVELOPE_LEN,
                *,
                seq_start: int = 0) -> list[StreamFrame]:
    """Chunk `data` into StreamFrames of payload size <= `mtu`."""
    if mtu <= 0:
        raise ValueError("mtu must be positive")
    if not data:
        return []
    chunks = [data[i:i + mtu] for i in range(0, len(data), mtu)]
    total = len(chunks)
    return [
        StreamFrame(seq=(seq_start + i) & 0xFFFF, total=total, payload=chunks[i])
        for i in range(total)
    ]


def unpack_stream(frames: Iterable[StreamFrame]) -> Iterator[tuple[int, bytes]]:
    """Yield `(seq, payload)` for each frame in arrival order; drop duplicates.

    Gaps are NOT filled — caller's responsibility to detect via the yielded
    `seq`. Stops after `total` distinct frames have arrived (when total > 0).
    """
    seen: set[int] = set()
    total_target = 0
    delivered = 0
    for f in frames:
        if f.seq in seen:
            continue
        seen.add(f.seq)
        if f.total and not total_target:
            total_target = f.total
        yield f.seq, f.payload
        delivered += 1
        if total_target and delivered >= total_target:
            return
