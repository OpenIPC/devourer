"""RaptorQ (RFC 6330) block-FEC implementation for the stream link.

Originally the only FEC scheme (see PR #86). Lives here so `stream_fec.py`
can stay a thin dispatcher that picks between RaptorQ (this module) and
RLC (`stream_fec_rlc.py`) at runtime based on `FecConfig.scheme`.

Wire format (inside the outer `StreamFrame.payload`):

    FEC_MAGIC     (2)   = 0xF52E little-endian
    VERSION/FLAGS (1)   = 0 (reserved)
    K             (1)   = source symbols per block (encoder config)
    KREAL         (1)   = real source symbols in THIS block (1..K). For
                          flush-padded last blocks KREAL < K and the
                          trailing (K - KREAL) decoded source symbols are
                          zero pads.
    SYMBOL_SIZE   (2)   = LE u16, must match TX config
    BLOCK_ID      (2)   = LE u16 wraps; identifies which K-symbol block
    RAPTORQ_PKT   (var) = cberner's serialised packet (≈ 4-byte SBN+ESI
                          plus symbol bytes).

Source symbols are concatenation-packed IP packets — same length-prefix
scheme as `stream_fec_rlc.py`. See `stream_fec.PACKET_LEN_PREFIX`.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Optional

import raptorq

FEC_MAGIC = 0xF52E
FEC_HEADER_LEN = 9
FEC_HEADER_STRUCT = "<HBBBHH"  # MAGIC, FLAGS, K, KREAL, SYMBOL_SIZE, BLOCK_ID

PACKET_LEN_PREFIX = 2


def _pack_header(k: int, kreal: int, symbol_size: int, block_id: int) -> bytes:
    return struct.pack(
        FEC_HEADER_STRUCT,
        FEC_MAGIC, 0, k, kreal, symbol_size, block_id & 0xFFFF,
    )


def _unpack_header(env: bytes) -> Optional[tuple[int, int, int, int, int]]:
    if len(env) < FEC_HEADER_LEN:
        return None
    magic, ver, k, kreal, ss, bid = struct.unpack_from(FEC_HEADER_STRUCT, env)
    if magic != FEC_MAGIC:
        return None
    return ver, k, kreal, ss, bid


class RaptorQEncoder:
    """Concatenation-packs IP packets, then RaptorQ-encodes K source symbols
    into K + repair_count output symbols."""

    def __init__(self, cfg) -> None:
        self.cfg = cfg
        self._pending_symbols: list[bytes] = []
        self._current_symbol = bytearray()
        self._block_id = 0
        # Stats read by tun_p2p.
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
        data = b"".join(self._pending_symbols)
        self._pending_symbols.clear()
        encoder = raptorq.Encoder.with_defaults(data, self.cfg.symbol_size)
        packets = encoder.get_encoded_packets(self.cfg.repair_count)
        bid = self._block_id
        self._block_id = (self._block_id + 1) & 0xFFFF
        self.blocks_encoded += 1
        self.symbols_out += len(packets)
        header = _pack_header(self.cfg.k, kreal, self.cfg.symbol_size, bid)
        return [header + p for p in packets]

    @property
    def pending_packets(self) -> int:
        if self._current_symbol:
            return 1
        return len(self._pending_symbols)


@dataclass
class _BlockState:
    decoder: "raptorq.Decoder"
    kreal: int
    first_seen: float
    decoded: bool = False


class RaptorQDecoder:
    def __init__(self, cfg) -> None:
        self.cfg = cfg
        self._blocks: dict[int, _BlockState] = {}
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
        version, k, kreal, symbol_size, block_id = header
        if version != 0:
            return []
        if k != self.cfg.k or symbol_size != self.cfg.symbol_size:
            self.symbols_dropped_bad_cfg += 1
            return []
        if not (1 <= kreal <= k):
            self.symbols_dropped_bad_cfg += 1
            return []
        packet = envelope[FEC_HEADER_LEN:]
        self.symbols_in += 1

        state = self._blocks.get(block_id)
        if state is None:
            state = _BlockState(
                decoder=raptorq.Decoder.with_defaults(
                    k * symbol_size, symbol_size),
                kreal=kreal,
                first_seen=time.monotonic(),
            )
            self._blocks[block_id] = state
        elif state.decoded:
            self.symbols_dropped_stale_block += 1
            return []

        try:
            result = state.decoder.decode(packet)
        except Exception:
            return []
        if result is None:
            return []
        state.decoded = True
        self.blocks_decoded += 1
        ip_pkts = self._unpack(result, state.kreal)
        self.packets_out += len(ip_pkts)
        self.bytes_out += sum(len(p) for p in ip_pkts)
        return ip_pkts

    def _unpack(self, decoded: bytes, kreal: int) -> list[bytes]:
        ss = self.cfg.symbol_size
        out: list[bytes] = []
        for i in range(kreal):
            symbol = decoded[i * ss:(i + 1) * ss]
            pos = 0
            while pos + PACKET_LEN_PREFIX <= len(symbol):
                ln = int.from_bytes(
                    symbol[pos:pos + PACKET_LEN_PREFIX], "little")
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
        expired = [
            bid for bid, st in self._blocks.items()
            if (now - st.first_seen) > max_age_s
        ]
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
