"""Sub-Block Integrity (SBI) framing — the "fused FEC" middle layer.

Packs several fixed-size outer-code symbols (FEC envelopes from
`stream_fec.make_encoder`) into one radio body, each guarded by its own CRC,
so a frame that fails the 802.11 FCS — surfaced via
`DEVOURER_RX_KEEP_CORRUPTED` — yields its *surviving* sub-blocks as good
symbols and erases only the corrupted ones, instead of the whole frame
becoming a single erasure.

Why this matters
----------------
Today (`tun_p2p.py`) one FEC envelope == one radio frame, so a CRC-failed
frame = one whole erased symbol. The outer erasure code (RaptorQ / RLC / RS)
then has to reconstruct that symbol from repair. Real 802.11 corruption is
*localized* ("All Bits Are Not Equal", INFOCOM 2009): a frame that fails the
FCS is usually mostly-correct. By splitting the body into independently
checksummed sub-blocks we convert one whole-frame erasure into a few
sub-block erasures — and the outer decoder is **unchanged**: a dropped
sub-block is simply a symbol that didn't arrive, which is exactly what FEC
recovers. This is the concatenation
    PHY-MCS FEC (inner)  ⊕  sub-block integrity  ⊕  outer erasure code
that befinitiv sketched for wifibroadcast but never built.

Wire format
-----------
    body = SBI_HDR | block[0] | block[1] | … | block[n-1]
    SBI_HDR  = MAGIC(2 LE) | VER(1) | STREAM_ID(1) | BLOCK_PAYLOAD(2 LE) | N_BLOCKS(1)  (7 B)
    block    = CRC16(2 LE over PAYLOAD) | PAYLOAD(BLOCK_PAYLOAD bytes)

STREAM_ID multiplexes independent sub-block streams over one SBI framing — used
by the per-SVC-layer UEP encoder (`svc_uep_fec.py`) to tag which temporal layer
a body belongs to so the receiver routes it to that layer's FEC decoder.

Fixed-size sub-blocks on purpose
--------------------------------
The blocks are NOT self-delimiting. A self-delimiting (length-prefixed)
framing would desync every following sub-block the moment a length byte got
flipped — defeating the entire purpose. With a fixed `block_payload` the
receiver partitions the body at *known* offsets, so a hit in block i is
contained to block i. The receiver therefore trusts its own configured
`block_payload`, not the (possibly corrupt) header — the header is a sanity
tag for clean frames, never load-bearing under corruption.

False-accept note
-----------------
A corrupted sub-block whose CRC16 happens to match (~1/65536) feeds a bad
symbol to the outer decoder. CRC32 (`crc_bytes=4`) drops that to ~1/2^32 at
+2 B/block; CRC16 is the default to match `stream.py`. The PoC reports the
overhead so the trade is explicit.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterable


def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    """CRC-16-CCITT-FALSE (poly 0x1021, init 0xFFFF) — byte-identical to
    `stream.crc16_ccitt`. Inlined (not imported) so this module stays a
    dependency-light, numpy-free primitive usable inside the GNU Radio
    Python env (the SDR→SBI bridge) as well as the precoder toolchain."""
    crc = init & 0xFFFF
    for byte in data:
        crc ^= (byte & 0xFF) << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) \
                else (crc << 1) & 0xFFFF
    return crc


SBI_MAGIC = 0xF5B0
SBI_VERSION = 0
SBI_HDR_STRUCT = "<HBBHB"         # MAGIC, VER, STREAM_ID, BLOCK_PAYLOAD, N_BLOCKS
SBI_HDR_LEN = struct.calcsize(SBI_HDR_STRUCT)   # 7 bytes


def _crc(width: int, data: bytes) -> int:
    """CRC over `data`, `width` ∈ {2, 4}. CRC32 reuses Python's zlib so we
    keep the strong option dependency-free; CRC16 mirrors stream.py."""
    if width == 2:
        return crc16_ccitt(data) & 0xFFFF
    if width == 4:
        import zlib
        return zlib.crc32(data) & 0xFFFFFFFF
    raise ValueError(f"crc_bytes must be 2 or 4 (got {width})")


@dataclass(frozen=True)
class UnpackResult:
    """Outcome of unpacking one radio body.

    `survivors` are the envelope payloads whose sub-block CRC validated, in
    body order — feed each straight to the outer decoder's `add_symbol`.
    `n_blocks` / `n_failed` quantify the salvage (n_failed == 0 on a clean
    body; on a kept-corrupt body it's how many sub-blocks were lost, i.e. the
    erasures the outer code now has to cover — far fewer than the whole frame).
    """
    survivors: list[bytes]
    n_blocks: int
    n_failed: int
    header_ok: bool
    stream_id: int = 0


class SubBlockPacker:
    """Batch fixed-size FEC envelopes into SBI radio bodies.

    Stateful so a caller can stream envelopes in (mirrors the FEC encoder's
    `add`/`flush` shape): `add(env)` returns a body once `blocks_per_body`
    have accumulated, `flush()` emits a short final body.

    All envelopes in one packer must share `block_payload` (true for a single
    FEC scheme/config — RLC and RaptorQ both pad symbols to a fixed size).
    """

    def __init__(self, block_payload: int, blocks_per_body: int,
                 crc_bytes: int = 2, stream_id: int = 0) -> None:
        if block_payload <= 0 or block_payload > 0xFFFF:
            raise ValueError(f"block_payload must be 1..65535 (got {block_payload})")
        if blocks_per_body <= 0 or blocks_per_body > 0xFF:
            raise ValueError(f"blocks_per_body must be 1..255 (got {blocks_per_body})")
        if crc_bytes not in (2, 4):
            raise ValueError("crc_bytes must be 2 or 4")
        if not (0 <= stream_id <= 0xFF):
            raise ValueError(f"stream_id must be 0..255 (got {stream_id})")
        self.block_payload = block_payload
        self.blocks_per_body = blocks_per_body
        self.crc_bytes = crc_bytes
        self.stream_id = stream_id
        self._pending: list[bytes] = []
        self.bodies_out = 0
        self.blocks_out = 0

    @property
    def block_stride(self) -> int:
        return self.crc_bytes + self.block_payload

    def body_capacity(self) -> int:
        """Max body size this packer emits (full body)."""
        return SBI_HDR_LEN + self.blocks_per_body * self.block_stride

    def add(self, envelope: bytes) -> list[bytes]:
        if len(envelope) != self.block_payload:
            raise ValueError(
                f"envelope {len(envelope)}B != block_payload {self.block_payload}B "
                "(all envelopes in one packer must be the same fixed size)")
        self._pending.append(bytes(envelope))
        out: list[bytes] = []
        while len(self._pending) >= self.blocks_per_body:
            batch = self._pending[:self.blocks_per_body]
            self._pending = self._pending[self.blocks_per_body:]
            out.append(self._build_body(batch))
        return out

    def flush(self) -> list[bytes]:
        if not self._pending:
            return []
        batch = self._pending
        self._pending = []
        return [self._build_body(batch)]

    def _build_body(self, batch: list[bytes]) -> bytes:
        out = bytearray(struct.pack(SBI_HDR_STRUCT, SBI_MAGIC, SBI_VERSION,
                                    self.stream_id, self.block_payload,
                                    len(batch)))
        for env in batch:
            crc = _crc(self.crc_bytes, env)
            out += crc.to_bytes(self.crc_bytes, "little")
            out += env
        self.bodies_out += 1
        self.blocks_out += len(batch)
        return bytes(out)


def pack(envelopes: Iterable[bytes], block_payload: int, blocks_per_body: int,
         crc_bytes: int = 2, stream_id: int = 0) -> list[bytes]:
    """Stateless convenience wrapper around SubBlockPacker."""
    p = SubBlockPacker(block_payload, blocks_per_body, crc_bytes, stream_id)
    out: list[bytes] = []
    for env in envelopes:
        out += p.add(env)
    out += p.flush()
    return out


def unpack(body: bytes, block_payload: int, crc_bytes: int = 2) -> UnpackResult:
    """Split a radio body into sub-blocks and return the CRC-valid survivors.

    `block_payload` / `crc_bytes` come from the receiver's config and are
    authoritative — the body's header is sanity-checked but never trusted to
    drive partitioning, so a corrupted header can't desync the scan. Works
    identically on a clean body (n_failed == 0) and a kept-corrupt one.
    """
    stride = crc_bytes + block_payload
    header_ok = False
    stream_id = 0
    if len(body) >= SBI_HDR_LEN:
        magic, ver, stream_id, hdr_bp, _hdr_n = struct.unpack_from(
            SBI_HDR_STRUCT, body)
        header_ok = (magic == SBI_MAGIC and ver == SBI_VERSION
                     and hdr_bp == block_payload)

    region = body[SBI_HDR_LEN:]
    n_blocks = len(region) // stride
    survivors: list[bytes] = []
    n_failed = 0
    for i in range(n_blocks):
        off = i * stride
        crc_field = int.from_bytes(region[off:off + crc_bytes], "little")
        payload = region[off + crc_bytes:off + stride]
        if _crc(crc_bytes, payload) == crc_field:
            survivors.append(bytes(payload))
        else:
            n_failed += 1
    return UnpackResult(survivors=survivors, n_blocks=n_blocks,
                        n_failed=n_failed, header_ok=header_ok,
                        stream_id=stream_id)


def peek_stream_id(body: bytes):
    """Return the SBI STREAM_ID from a body's header, or None on a bad header.

    The header sits at a fixed 7-byte offset independent of block_payload, so a
    multiplexing receiver can route a body to the right stream before it knows
    that stream's envelope size. Under corruption the routing may be wrong, but
    the wrong stream's decoder then rejects the mismatched sub-blocks — a
    dropped body, never a mis-decode.
    """
    if len(body) < SBI_HDR_LEN:
        return None
    magic, ver, stream_id, _bp, _n = struct.unpack_from(SBI_HDR_STRUCT, body)
    if magic != SBI_MAGIC or ver != SBI_VERSION:
        return None
    return stream_id


def overhead_bytes(blocks_per_body: int, crc_bytes: int = 2) -> int:
    """Per-body framing overhead (header + per-block CRCs), for sizing logs."""
    return SBI_HDR_LEN + blocks_per_body * crc_bytes
