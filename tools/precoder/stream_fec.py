"""Stream-link FEC dispatcher.

Two FEC schemes coexist here, sharing the same outer `StreamFrame.payload`
inner-envelope-then-symbol pattern but picking different MAGICs so a
receive dispatcher can route per-frame:

    0xF52E → RaptorQ (RFC 6330; cberner's PyO3 binding).
              Block code: K source symbols are buffered, then K + R encoded
              symbols are emitted; receiver decodes after ≈ K + ε arrive.
              Best for bulk throughput; per-packet latency floor is high.
              Implementation lives in `stream_fec_raptorq.py`.

    0xF534 → Sliding-window RLC (RFC 8681; wraps Inria's swif-codec).
              Source symbols are emitted systematically (zero encoder
              buffer), repair symbols are linear combinations over the
              last `window` source symbols. Best for low-latency
              interactive traffic. Implementation lives in
              `stream_fec_rlc.py`.

The public interface is identical for both schemes — the encoder exposes
`add_packet(pkt) -> list[bytes]`, `flush() -> list[bytes]`,
`pending_packets`, `blocks_encoded`, `symbols_out`, … and the decoder
exposes `add_symbol(env) -> list[bytes]`, `expire_blocks_older_than(s)`,
`blocks_decoded`, `blocks_unrecoverable`, … . `tun_p2p.py` reads them
without caring which scheme is underneath.

`FecConfig.scheme` selects between the two; the default is "raptorq" so
existing tests and callers that construct `FecConfig(k=…)` without a
scheme argument keep the historical behaviour. `tun_p2p.py`'s
`--fec-scheme` flag overrides this with "rlc" as the user-facing default.

Source-symbol packing — packets carried in a source symbol are
length-prefixed (u16 LE) and concatenated; a `len = 0` sentinel or
remaining bytes < 2 marks "no more packets in this symbol". This packs
small packets densely (ACK floods don't burn one symbol each) and keeps
a packet whose size is close to symbol_size in exactly one symbol. Both
schemes use the SAME packing; only the codec under the symbol differs.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

# --------------------------------------------------------------------------- #
# Shared constants
# --------------------------------------------------------------------------- #
PACKET_LEN_PREFIX = 2  # u16 length prefix per IP packet inside a source symbol

# Defaults inherited from the original RaptorQ-only implementation; RLC
# adds its own (window, density_threshold) below.
DEFAULT_SCHEME = "raptorq"
DEFAULT_K = 16
DEFAULT_WINDOW = 32  # RLC sliding-window size
DEFAULT_SYMBOL_SIZE = 1477
DEFAULT_OVERHEAD = 1.0
DEFAULT_DENSITY_THRESHOLD = 4  # RLC; range 0..15, see RFC 8681 §3.4

# Magic constants for envelope dispatch. Both modules re-export these so
# their per-scheme code can import from here without circular dependency.
FEC_MAGIC_RAPTORQ = 0xF52E
FEC_MAGIC_RLC = 0xF534
FEC_MAGIC_RS = 0xF540

# Legacy alias — pre-RLC code references `FEC_MAGIC` and the 9-byte
# RaptorQ header length. Keep them aliased to RaptorQ for any importer
# we haven't updated.
FEC_MAGIC = FEC_MAGIC_RAPTORQ
FEC_HEADER_LEN = 9


@dataclass(frozen=True)
class FecConfig:
    """Shared encoder/decoder config.

    `scheme` picks between schemes; defaults to RaptorQ for backward
    compatibility. tun_p2p.py supplies `scheme="rlc"` to default new
    bridges to the low-latency scheme.

    RaptorQ-specific knobs: `k`, `overhead`, `symbol_size`.
    RLC-specific knobs: `window`, `density_threshold`, `symbol_size`,
    `overhead` (RLC overhead is repair-per-source-symbol).
    """

    k: int = DEFAULT_K
    symbol_size: int = DEFAULT_SYMBOL_SIZE
    overhead: float = DEFAULT_OVERHEAD
    scheme: str = DEFAULT_SCHEME
    window: int = DEFAULT_WINDOW
    density_threshold: int = DEFAULT_DENSITY_THRESHOLD

    def __post_init__(self) -> None:
        if self.scheme not in ("raptorq", "rlc", "rs"):
            raise ValueError(
                f"scheme must be 'raptorq', 'rlc' or 'rs' (got {self.scheme!r})")
        if self.k <= 0 or self.k > 255:
            raise ValueError(f"k must be in 1..255 (got {self.k})")
        if self.symbol_size <= PACKET_LEN_PREFIX:
            raise ValueError(
                f"symbol_size must be > {PACKET_LEN_PREFIX} (got {self.symbol_size})"
            )
        if self.overhead < 0:
            raise ValueError(f"overhead must be >= 0 (got {self.overhead})")
        if self.window <= 0 or self.window > 255:
            raise ValueError(f"window must be in 1..255 (got {self.window})")
        if not (0 <= self.density_threshold <= 15):
            raise ValueError(
                f"density_threshold must be in 0..15 (got {self.density_threshold})"
            )

    @property
    def repair_count(self) -> int:
        """Number of repair packets per RaptorQ block (= ceil(k * overhead))."""
        return math.ceil(self.k * self.overhead)

    @property
    def max_packet_size(self) -> int:
        """Largest IP packet (without length prefix) that fits in one symbol."""
        return self.symbol_size - PACKET_LEN_PREFIX


# --------------------------------------------------------------------------- #
# Factory functions — pick the right per-scheme implementation
# --------------------------------------------------------------------------- #
def make_encoder(cfg: FecConfig):
    """Return a scheme-appropriate encoder. The returned object exposes the
    duck-typed interface tun_p2p.py expects (add_packet, flush,
    pending_packets, counters)."""
    if cfg.scheme == "raptorq":
        from stream_fec_raptorq import RaptorQEncoder
        return RaptorQEncoder(cfg)
    if cfg.scheme == "rlc":
        from stream_fec_rlc import RlcEncoder
        return RlcEncoder(cfg)
    if cfg.scheme == "rs":
        from stream_fec_rs import RsEncoder
        return RsEncoder(cfg)
    raise ValueError(f"unknown FEC scheme {cfg.scheme!r}")


def make_decoder(cfg: FecConfig):
    """Return a scheme-appropriate decoder. The returned object exposes
    add_symbol, expire_blocks_older_than, and the per-scheme counters."""
    if cfg.scheme == "raptorq":
        from stream_fec_raptorq import RaptorQDecoder
        return RaptorQDecoder(cfg)
    if cfg.scheme == "rlc":
        from stream_fec_rlc import RlcDecoder
        return RlcDecoder(cfg)
    if cfg.scheme == "rs":
        from stream_fec_rs import RsDecoder
        return RsDecoder(cfg)
    raise ValueError(f"unknown FEC scheme {cfg.scheme!r}")


# --------------------------------------------------------------------------- #
# Backward-compat aliases — pre-RLC code does
#    import stream_fec; enc = stream_fec.FecEncoder(cfg); dec = stream_fec.FecDecoder(cfg)
# Keep those callable by routing them through the factory.
# --------------------------------------------------------------------------- #
def FecEncoder(cfg: FecConfig):  # noqa: N802 — preserving the historical name
    return make_encoder(cfg)


def FecDecoder(cfg: FecConfig):  # noqa: N802
    return make_decoder(cfg)
