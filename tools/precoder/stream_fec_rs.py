"""Reed-Solomon block erasure FEC for the stream link.

The third outer scheme alongside RaptorQ (`stream_fec_raptorq.py`) and RLC
(`stream_fec_rlc.py`). RS is **MDS** — any K of the N = K + repair symbols
reconstruct the block exactly, with zero overhead — which makes it the best
choice for the small, low-latency blocks a video downlink wants (where
RaptorQ's probabilistic overhead is worst). This mirrors what
wfb-ng/wifibroadcast use: a systematic Reed-Solomon over GF(2^8) built from a
Vandermonde matrix (poly 0x11d, the zfec/Rizzo construction).

Wire format (inside the outer `StreamFrame.payload`):

    RS_MAGIC      (2)   = 0xF540 little-endian
    VERSION/FLAGS (1)   = 0
    K             (1)   = source symbols per block
    KREAL         (1)   = real source symbols in THIS block (1..K); the
                          trailing (K - KREAL) are zero pads (flush)
    SYMBOL_SIZE   (2)   = LE u16
    BLOCK_ID      (2)   = LE u16 wraps
    ESI           (1)   = symbol index in the block, 0..N-1 (0..K-1 source,
                          systematic; K..N-1 repair)
    N             (1)   = total symbols in the block (K + repair)
    PAYLOAD       (var) = `symbol_size` bytes

Source symbols are concatenation-packed IP packets — the SAME length-prefix
scheme as the other two schemes (`stream_fec.PACKET_LEN_PREFIX`).

Decode is erasure-only: the systematic source symbols pass through untouched,
and once any K distinct ESIs of a block have arrived the K×K submatrix of the
encoding matrix is inverted over GF(2^8) to recover every source symbol.
Pure-Python GF math — fine for the research toolchain's modest rates.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field
from typing import Optional

RS_MAGIC = 0xF540
RS_HEADER_LEN = 11
RS_HEADER_STRUCT = "<HBBBHHBB"  # MAGIC, FLAGS, K, KREAL, SYMBOL_SIZE, BLOCK_ID, ESI, N

PACKET_LEN_PREFIX = 2

# --------------------------------------------------------------------------- #
# GF(2^8) with the standard 0x11d primitive polynomial (matches zfec/wfb-ng).
# --------------------------------------------------------------------------- #
_GF_POLY = 0x11D
_GF_EXP = [0] * 512
_GF_LOG = [0] * 256


def _init_gf() -> None:
    x = 1
    for i in range(255):
        _GF_EXP[i] = x
        _GF_LOG[x] = i
        x <<= 1
        if x & 0x100:
            x ^= _GF_POLY
    for i in range(255, 512):
        _GF_EXP[i] = _GF_EXP[i - 255]


_init_gf()


def _mul(a: int, b: int) -> int:
    if a == 0 or b == 0:
        return 0
    return _GF_EXP[_GF_LOG[a] + _GF_LOG[b]]


def _inv(a: int) -> int:
    if a == 0:
        raise ZeroDivisionError("GF inverse of 0")
    return _GF_EXP[255 - _GF_LOG[a]]


def _pow(a: int, e: int) -> int:
    if e == 0:
        return 1
    if a == 0:
        return 0
    return _GF_EXP[(_GF_LOG[a] * e) % 255]


def _mat_mul(A: list[list[int]], B: list[list[int]]) -> list[list[int]]:
    n, m, p = len(A), len(B), len(B[0])
    out = [[0] * p for _ in range(n)]
    for i in range(n):
        Ai = A[i]
        Oi = out[i]
        for k in range(m):
            a = Ai[k]
            if a == 0:
                continue
            la = _GF_LOG[a]
            Bk = B[k]
            for j in range(p):
                b = Bk[j]
                if b:
                    Oi[j] ^= _GF_EXP[la + _GF_LOG[b]]
    return out


def _mat_inv(M: list[list[int]]) -> list[list[int]]:
    """Gauss-Jordan inverse over GF(2^8). Raises if singular."""
    n = len(M)
    a = [row[:] + [1 if i == j else 0 for j in range(n)]
         for i, row in enumerate(M)]
    for col in range(n):
        piv = next((r for r in range(col, n) if a[r][col] != 0), None)
        if piv is None:
            raise ValueError("singular matrix (should not happen for K rows)")
        if piv != col:
            a[col], a[piv] = a[piv], a[col]
        inv_p = _inv(a[col][col])
        a[col] = [_mul(v, inv_p) for v in a[col]]
        for r in range(n):
            if r != col and a[r][col]:
                f = a[r][col]
                lf = _GF_LOG[f]
                ac = a[col]
                ar = a[r]
                for j in range(2 * n):
                    v = ac[j]
                    if v:
                        ar[j] ^= _GF_EXP[lf + _GF_LOG[v]]
    return [row[n:] for row in a]


_MATRIX_CACHE: dict[tuple[int, int], list[list[int]]] = {}


def _encoding_matrix(k: int, n: int) -> list[list[int]]:
    """N×K systematic MDS matrix: top K rows = I, any K rows invertible.

    Build a Vandermonde V[i][j] = x_i^j (x_i = i, distinct for i<256), then
    right-multiply by (V[:K])^-1 so the top block becomes identity — the
    Rizzo/zfec construction. Right-multiplying by an invertible matrix
    preserves the any-K-rows-invertible property of a Vandermonde.
    """
    key = (k, n)
    cached = _MATRIX_CACHE.get(key)
    if cached is not None:
        return cached
    if n > 256:
        raise ValueError(f"RS needs K+repair = N <= 256 (got N={n})")
    V = [[_pow(i, j) for j in range(k)] for i in range(n)]
    top_inv = _mat_inv([row[:] for row in V[:k]])
    A = _mat_mul(V, top_inv)
    _MATRIX_CACHE[key] = A
    return A


def _lincomb(coeffs: list[int], symbols: list[bytes], size: int) -> bytes:
    """GF(2^8) linear combination of `symbols` with `coeffs`, byte-wise."""
    acc = bytearray(size)
    for c, sym in zip(coeffs, symbols):
        if c == 0:
            continue
        lc = _GF_LOG[c]
        for idx in range(size):
            s = sym[idx]
            if s:
                acc[idx] ^= _GF_EXP[lc + _GF_LOG[s]]
    return bytes(acc)


# --------------------------------------------------------------------------- #
# Header helpers
# --------------------------------------------------------------------------- #
def _pack_header(k: int, kreal: int, symbol_size: int, block_id: int,
                 esi: int, n: int) -> bytes:
    return struct.pack(RS_HEADER_STRUCT, RS_MAGIC, 0, k, kreal, symbol_size,
                       block_id & 0xFFFF, esi, n)


def _unpack_header(env: bytes) -> Optional[tuple[int, int, int, int, int, int]]:
    if len(env) < RS_HEADER_LEN:
        return None
    magic, ver, k, kreal, ss, bid, esi, n = struct.unpack_from(
        RS_HEADER_STRUCT, env)
    if magic != RS_MAGIC or ver != 0:
        return None
    return k, kreal, ss, bid, esi, n


# --------------------------------------------------------------------------- #
# Encoder
# --------------------------------------------------------------------------- #
class RsEncoder:
    """Concatenation-packs IP packets, then Reed-Solomon-encodes K source
    symbols into N = K + repair_count systematic + parity symbols."""

    def __init__(self, cfg) -> None:
        self.cfg = cfg
        self._n = cfg.k + cfg.repair_count
        if self._n > 256:
            raise ValueError(
                f"RS block N = K({cfg.k}) + repair({cfg.repair_count}) = "
                f"{self._n} exceeds GF(2^8) limit of 256")
        self._pending_symbols: list[bytes] = []
        self._current_symbol = bytearray()
        self._block_id = 0
        self.blocks_encoded = 0
        self.packets_in = 0
        self.symbols_out = 0
        self.bytes_in = 0

    def add_packet(self, pkt: bytes) -> list[bytes]:
        n = len(pkt)
        if n > self.cfg.max_packet_size:
            raise ValueError(
                f"packet {n}B exceeds max_packet_size {self.cfg.max_packet_size}B "
                f"(symbol_size {self.cfg.symbol_size} - {PACKET_LEN_PREFIX} prefix)"
            )
        self.packets_in += 1
        self.bytes_in += n
        needed = PACKET_LEN_PREFIX + n
        if needed > self.cfg.symbol_size - len(self._current_symbol):
            self._seal_current_symbol()
            ready = self._maybe_encode_full_block()
            if ready:
                self._append_to_current(pkt)
                return ready
        self._append_to_current(pkt)
        return []

    def _append_to_current(self, pkt: bytes) -> None:
        self._current_symbol += struct.pack("<H", len(pkt))
        self._current_symbol += pkt

    def _seal_current_symbol(self) -> None:
        if not self._current_symbol:
            return
        pad = self.cfg.symbol_size - len(self._current_symbol)
        if pad:
            self._current_symbol += b"\x00" * pad
        self._pending_symbols.append(bytes(self._current_symbol))
        self._current_symbol = bytearray()

    def _maybe_encode_full_block(self) -> list[bytes]:
        if len(self._pending_symbols) >= self.cfg.k:
            return self._encode_block(kreal=self.cfg.k)
        return []

    def flush(self) -> list[bytes]:
        self._seal_current_symbol()
        if not self._pending_symbols:
            return []
        kreal = len(self._pending_symbols)
        while len(self._pending_symbols) < self.cfg.k:
            self._pending_symbols.append(b"\x00" * self.cfg.symbol_size)
        return self._encode_block(kreal=kreal)

    def _encode_block(self, kreal: int) -> list[bytes]:
        src = self._pending_symbols[:self.cfg.k]
        self._pending_symbols = self._pending_symbols[self.cfg.k:]
        k, n, ss = self.cfg.k, self._n, self.cfg.symbol_size
        A = _encoding_matrix(k, n)
        bid = self._block_id
        self._block_id = (self._block_id + 1) & 0xFFFF

        out: list[bytes] = []
        for esi in range(n):
            if esi < k:
                payload = src[esi]              # systematic passthrough
            else:
                payload = _lincomb(A[esi], src, ss)
            out.append(_pack_header(k, kreal, ss, bid, esi, n) + payload)
        self.blocks_encoded += 1
        self.symbols_out += len(out)
        return out

    @property
    def pending_packets(self) -> int:
        if self._current_symbol:
            return 1
        return len(self._pending_symbols)


# --------------------------------------------------------------------------- #
# Decoder
# --------------------------------------------------------------------------- #
@dataclass
class _RsBlock:
    k: int
    n: int
    kreal: int
    first_seen: float
    symbols: dict[int, bytes] = field(default_factory=dict)  # esi -> payload
    decoded: bool = False


class RsDecoder:
    def __init__(self, cfg) -> None:
        self.cfg = cfg
        self._blocks: dict[int, _RsBlock] = {}
        self.blocks_decoded = 0
        self.blocks_unrecoverable = 0
        self.symbols_in = 0
        self.symbols_dropped_bad_cfg = 0
        self.symbols_dropped_stale_block = 0
        self.packets_out = 0
        self.bytes_out = 0

    def add_symbol(self, envelope: bytes) -> list[bytes]:
        header = _unpack_header(envelope)
        if header is None:
            return []
        k, kreal, ss, block_id, esi, n = header
        if k != self.cfg.k or ss != self.cfg.symbol_size:
            self.symbols_dropped_bad_cfg += 1
            return []
        if not (1 <= kreal <= k) or not (0 <= esi < n) or n < k:
            self.symbols_dropped_bad_cfg += 1
            return []
        payload = envelope[RS_HEADER_LEN:]
        if len(payload) != ss:
            self.symbols_dropped_bad_cfg += 1
            return []
        self.symbols_in += 1

        st = self._blocks.get(block_id)
        if st is None:
            st = _RsBlock(k=k, n=n, kreal=kreal, first_seen=time.monotonic())
            self._blocks[block_id] = st
        elif st.decoded:
            self.symbols_dropped_stale_block += 1
            return []
        st.symbols.setdefault(esi, bytes(payload))

        if len(st.symbols) < k:
            return []
        return self._solve(st)

    def _solve(self, st: _RsBlock) -> list[bytes]:
        k, n, ss = st.k, st.n, self.cfg.symbol_size
        esis = sorted(st.symbols)[:k]
        # Fast path: all K systematic source symbols present -> no inverse.
        if all(e < k for e in esis) and len(esis) == k:
            source = [st.symbols[e] for e in range(k)]
        else:
            A = _encoding_matrix(k, n)
            sub = [A[e][:] for e in esis]
            inv = _mat_inv(sub)
            recv = [st.symbols[e] for e in esis]
            source = [_lincomb(inv[j], recv, ss) for j in range(k)]
        st.decoded = True
        self.blocks_decoded += 1
        ip_pkts = self._unpack(source, st.kreal)
        self.packets_out += len(ip_pkts)
        self.bytes_out += sum(len(p) for p in ip_pkts)
        return ip_pkts

    def _unpack(self, source: list[bytes], kreal: int) -> list[bytes]:
        ss = self.cfg.symbol_size
        out: list[bytes] = []
        for i in range(kreal):
            symbol = source[i]
            pos = 0
            while pos + PACKET_LEN_PREFIX <= len(symbol):
                ln = int.from_bytes(symbol[pos:pos + PACKET_LEN_PREFIX], "little")
                if ln == 0:
                    break
                end = pos + PACKET_LEN_PREFIX + ln
                if end > len(symbol):
                    break
                out.append(bytes(symbol[pos + PACKET_LEN_PREFIX:end]))
                pos = end
        return out

    def expire_blocks_older_than(self, max_age_s: float) -> int:
        if not self._blocks:
            return 0
        now = time.monotonic()
        expired = [bid for bid, st in self._blocks.items()
                   if (now - st.first_seen) > max_age_s]
        unrecoverable = 0
        for bid in expired:
            if not self._blocks[bid].decoded:
                unrecoverable += 1
            del self._blocks[bid]
        self.blocks_unrecoverable += unrecoverable
        return unrecoverable

    @property
    def in_flight_blocks(self) -> int:
        return len(self._blocks)
