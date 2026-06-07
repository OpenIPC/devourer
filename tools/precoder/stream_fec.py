"""RaptorQ (RFC 6330) fountain-code FEC layer for the stream link.

Sits between the TUN device and `stream.py`'s framing. The channel underneath
is a packet-erasure channel (whole frames either arrive clean or are dropped
by the stream layer's CRC check), so a real erasure code can recover from a
loss rate the blind `--repeat N` knob in `tun_p2p.py` can't match without
infinite N. RaptorQ was designed for exactly this regime (broadcast, no ACK,
variable loss) — see the corruption survey in PR #85 for the empirical
30-70% loss numbers that motivated this.

Wire format. The OUTER stream framing in `stream.py` is unchanged; FEC lives
INSIDE `StreamFrame.payload` as a fixed 9-byte inner envelope followed by
the serialised raptorq packet (lib-managed SBN + ESI + symbol):

    FEC_MAGIC     (2)   = 0xF52E little-endian
    VERSION/FLAGS (1)   = 0 (reserved)
    K             (1)   = source symbols per block (encoder config)
    KREAL         (1)   = real source symbols in THIS block (1..K). For
                          flush-padded last blocks KREAL < K and the trailing
                          (K - KREAL) decoded source symbols are zero pads.
    SYMBOL_SIZE   (2)   = LE u16, must match TX config
    BLOCK_ID      (2)   = LE u16 wraps; identifies which K-symbol block
    RAPTORQ_PKT   (var) = lib's serialised packet (≈ 4 byte SBN+ESI plus
                          the symbol bytes; size depends on lib internals)

Source symbols (the K bytes-segments the codec encodes) are themselves
concatenations of length-prefixed IP packets:

    [u16 len_a LE][packet_a]…[u16 len_b LE][packet_b]…[zero pad to SYMBOL_SIZE]

A `len = 0` sentinel or remaining bytes < 2 marks "no more packets in this
symbol". This packs small packets densely (ACK floods don't burn one symbol
each) and keeps a packet whose size is close to symbol_size in exactly one
symbol. Tun MTU must be at most `SYMBOL_SIZE - 2`.

Block-decoding model — cberner's `raptorq` lib (RFC 6330 reference port)
exposes block-incremental decoding: feed packets one at a time, the lib
returns the K source symbols (concatenated) when enough have arrived. We
keep one `raptorq.Decoder` per in-flight block id; entries are dropped
either after successful decode or after `expire_blocks_older_than` (the
tun_p2p bridge calls this periodically with a configurable max age).
"""

from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass
from typing import Optional

import raptorq

FEC_MAGIC = 0xF52E
FEC_HEADER_LEN = 9
FEC_HEADER_STRUCT = "<HBBBHH"  # MAGIC, FLAGS, K, KREAL, SYMBOL_SIZE, BLOCK_ID

# Per-packet length prefix inside a source symbol.
PACKET_LEN_PREFIX = 2

DEFAULT_K = 16
DEFAULT_SYMBOL_SIZE = 1477  # cberner's `mtu`, packet bytes = symbol_size - 1
DEFAULT_OVERHEAD = 1.0


@dataclass(frozen=True)
class FecConfig:
    """Configuration shared between TX and RX peers."""

    k: int = DEFAULT_K
    symbol_size: int = DEFAULT_SYMBOL_SIZE
    overhead: float = DEFAULT_OVERHEAD

    def __post_init__(self) -> None:
        if self.k <= 0 or self.k > 255:
            raise ValueError(f"k must be in 1..255 (got {self.k})")
        if self.symbol_size <= PACKET_LEN_PREFIX:
            raise ValueError(
                f"symbol_size must be > {PACKET_LEN_PREFIX} (got {self.symbol_size})"
            )
        if self.overhead < 0:
            raise ValueError(f"overhead must be >= 0 (got {self.overhead})")

    @property
    def repair_count(self) -> int:
        """Number of repair packets per block (= ceil(k * overhead))."""
        return math.ceil(self.k * self.overhead)

    @property
    def max_packet_size(self) -> int:
        """Largest IP packet (without length prefix) that fits in one symbol."""
        return self.symbol_size - PACKET_LEN_PREFIX


def _pack_header(cfg: FecConfig, kreal: int, block_id: int) -> bytes:
    return struct.pack(
        FEC_HEADER_STRUCT,
        FEC_MAGIC, 0, cfg.k, kreal, cfg.symbol_size, block_id & 0xFFFF,
    )


def _unpack_header(env: bytes) -> Optional[tuple[int, int, int, int, int]]:
    """Return (version, k, kreal, symbol_size, block_id) or None on bad magic."""
    if len(env) < FEC_HEADER_LEN:
        return None
    magic, ver, k, kreal, ss, bid = struct.unpack_from(FEC_HEADER_STRUCT, env)
    if magic != FEC_MAGIC:
        return None
    return ver, k, kreal, ss, bid


# --------------------------------------------------------------------------- #
# Encoder
# --------------------------------------------------------------------------- #
class FecEncoder:
    """Concatenation-packs IP packets, then RaptorQ-encodes K source symbols
    into K + repair_count output symbols. Each output symbol is wrapped in
    the inner envelope and returned as bytes ready to drop into a
    `StreamFrame.payload`.
    """

    def __init__(self, cfg: FecConfig) -> None:
        self.cfg = cfg
        self._pending_symbols: list[bytes] = []
        self._current_symbol = bytearray()
        self._block_id = 0
        # Stats — read by tun_p2p for periodic counter prints.
        self.blocks_encoded = 0
        self.packets_in = 0
        self.symbols_out = 0
        self.bytes_in = 0

    def add_packet(self, pkt: bytes) -> list[bytes]:
        """Append `pkt` to the current packing symbol. If the symbol fills, it
        is moved to the pending list; if pending reaches K, a block is
        encoded and the K + repair_count envelopes are returned. Otherwise
        returns [].

        Raises ValueError if `pkt` is too large to fit in even an empty
        symbol — the caller (tun_p2p) is responsible for capping TUN MTU at
        `cfg.max_packet_size`.
        """
        n = len(pkt)
        if n > self.cfg.max_packet_size:
            raise ValueError(
                f"packet {n}B exceeds max_packet_size {self.cfg.max_packet_size}B "
                f"(symbol_size {self.cfg.symbol_size} - {PACKET_LEN_PREFIX} prefix)"
            )
        self.packets_in += 1
        self.bytes_in += n
        needed = PACKET_LEN_PREFIX + n
        # If this packet doesn't fit alongside what's already in the current
        # symbol, seal that symbol off first.
        if needed > self.cfg.symbol_size - len(self._current_symbol):
            self._seal_current_symbol()
            ready = self._maybe_encode_full_block()
            if ready:
                # Stash the new packet for the next block, then return the
                # encoded one. (We append below.)
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
        """Force-encode whatever's pending. The current packing symbol is
        sealed (zero-padded), then pending is zero-padded to K, then the
        block is encoded with KREAL set to the count of real (non-padded)
        source symbols. Decoder uses KREAL to discard the trailing zero
        symbols when unpacking IP packets.

        Returns [] if there's literally nothing pending.
        """
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
        header = _pack_header(self.cfg, kreal, bid)
        return [header + p for p in packets]

    @property
    def pending_packets(self) -> int:
        """Whether there is in-flight data that hasn't been flushed. Used by
        the tun_p2p flush timer thread to decide whether to call flush()."""
        if self._current_symbol:
            return 1
        return len(self._pending_symbols)


# --------------------------------------------------------------------------- #
# Decoder
# --------------------------------------------------------------------------- #
@dataclass
class _BlockState:
    decoder: "raptorq.Decoder"
    kreal: int
    first_seen: float
    decoded: bool = False


class FecDecoder:
    """Buffers incoming inner-envelope symbols by block-id, runs RaptorQ's
    block-incremental decoder, unpacks length-prefixed IP packets out of
    the first KREAL source symbols when a block decodes.
    """

    def __init__(self, cfg: FecConfig) -> None:
        self.cfg = cfg
        self._blocks: dict[int, _BlockState] = {}
        # Stats.
        self.blocks_decoded = 0
        self.blocks_unrecoverable = 0
        self.symbols_in = 0
        self.symbols_dropped_bad_cfg = 0
        self.symbols_dropped_stale_block = 0
        self.packets_out = 0
        self.bytes_out = 0

    def add_symbol(self, envelope: bytes) -> list[bytes]:
        """Feed one inner-envelope blob; returns recovered IP packets when a
        block decodes (possibly empty)."""
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
            # Late symbol for an already-decoded block; the entry will be
            # GC'd by expire_blocks_older_than. We don't re-decode.
            self.symbols_dropped_stale_block += 1
            return []

        # Decoder.decode returns the concatenated source bytes when enough
        # symbols have arrived for the block; None otherwise.
        try:
            result = state.decoder.decode(packet)
        except Exception:
            # Malformed packet bytes; raptorq raises on bad SBN/ESI fields.
            return []
        if result is None:
            return []
        # Mark this block decoded but keep the entry alive so late symbols
        # (we ship K + repair_count, decoder typically needs K + ε ~< all of
        # them) don't get fed into a fresh Decoder and trigger a second
        # spurious decode. The `state.decoded` branch above silently drops
        # those late symbols. `expire_blocks_older_than` GCs the entry.
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
                    # Malformed (truncated) — stop scanning this symbol.
                    break
                out.append(bytes(symbol[pos + PACKET_LEN_PREFIX:end]))
                pos = end
        return out

    def expire_blocks_older_than(self, max_age_s: float) -> int:
        """Drop block entries whose first symbol arrived more than `max_age_s`
        ago. Both decoded and undecoded entries are GC'd; for undecoded
        ones, `blocks_unrecoverable` is incremented.

        Returns the count of UNRECOVERABLE (= still undecoded) blocks
        evicted — useful as a counter for the bridge's stderr report. The
        return value intentionally excludes successfully-decoded blocks
        even though they're also evicted.
        """
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
