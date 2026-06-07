"""Sliding-Window RLC (RFC 8681) FEC for the stream link.

Wraps Inria's `swif-codec` C library (vendored at
`vendor/swif-codec/`; Python binding via cffi at `_swif_build.py`).

Why this scheme

The companion RaptorQ implementation (`stream_fec_raptorq.py`) is a
block code: K source IP packets accumulate, the encoder emits K + R
encoded symbols, the decoder waits for K + ε before any source is
delivered. The unavoidable consequence is a per-packet latency floor
of K × per-pkt-airtime + flush_ms. PR #86's bench measured 95 ms even
at K=8 / flush=20 ms.

RFC 8681 RLC is a sliding-window code: every source symbol is shipped
**systematically** (zero encoder buffer), repair symbols are linear
combinations over the last `window` source symbols, and the decoder
can emit each source the instant it arrives unmodified. Better suited
for interactive traffic (ping / voice / video).

Wire format (inside the outer `StreamFrame.payload`):

    RLC_MAGIC     (2)  = 0xF534 LE
    VERSION       (1)  = 0
    SYMBOL_TYPE   (1)  = 0 source, 1 repair
    SYMBOL_SIZE   (2)  = LE u16
    ESI           (4)  = LE u32; source: own ESI; repair: ESI of the
                          first source symbol covered by this repair
    WINDOW_SIZE   (1)  = number of source symbols covered by this repair
                          (1 for source symbols)
    REPAIR_KEY    (2)  = LE u16 TinyMT32 seed (RFC 8682) for the
                          coefficient PRNG; 0 for source symbols
    DT            (1)  = density threshold 0..15; 0 for source symbols
    PAYLOAD       (var) = `symbol_size` bytes — source data or linear
                          combination

Inner overhead = 14 bytes. Source-symbol payload uses the SAME
length-prefix-concat packing as RaptorQ — see `stream_fec.PACKET_LEN_PREFIX`.

Encoder model

* Source symbol = filled-and-sealed packing buffer of length `symbol_size`.
* Encoder maintains a `window`-deep ring of source-symbol buffers (cffi
  keeps the C-side window in lockstep).
* Each `add_packet(pkt)` may seal one or more source symbols. For each
  newly-sealed source symbol, the encoder emits its source envelope
  AND `repair_per_source = max(1, round(overhead))` repair envelopes
  over the current encoding window.

Decoder model

* Source envelopes (SYMBOL_TYPE=0) are fed straight to the codec's
  `decode_with_new_source_symbol` AND emit immediately to the IP layer
  (systematic delivery).
* Repair envelopes (SYMBOL_TYPE=1) bring the coding window into sync
  (`reset_coding_window`, `add_source_symbol_to_coding_window` for each
  ESI in the repair's window, `generate_coding_coefs` with the shipped
  `repair_key` and `dt`) and then `decode_with_new_repair_symbol`.
  Any source symbols the codec recovers are surfaced via callback and
  emitted to the IP layer.
* IP-packet order is preserved by emitting source-symbol bytes in ESI
  order: late recovery via repair feeds the same "in-ESI-order emit"
  queue, so the decoder gives the IP stack packets in the same order
  the encoder shipped them.
"""

from __future__ import annotations

import os
import random
import struct
import subprocess
import sys
import time
from typing import Optional

# ---------------------------------------------------------------------- #
# cffi extension auto-build on first import
# ---------------------------------------------------------------------- #
def _ensure_swif_ext():
    try:
        from _swif_ext import ffi, lib  # noqa: F401
        return
    except ImportError:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    build_script = os.path.join(here, "_swif_build.py")
    if not os.path.isfile(build_script):
        raise ImportError(
            "stream_fec_rlc: vendored swif-codec extension not built and "
            f"build script {build_script} is missing"
        )
    sys.stderr.write(
        "stream_fec_rlc: _swif_ext not found; building cffi extension…\n"
    )
    subprocess.check_call([sys.executable, build_script])


_ensure_swif_ext()

from _swif_ext import ffi, lib  # noqa: E402

# Header constants — match the wire format laid out in the module docstring.
RLC_MAGIC = 0xF534
RLC_HEADER_LEN = 14
RLC_HEADER_STRUCT = "<HBBHIBHB"  # MAGIC, VER, TYPE, SS, ESI, WIN, KEY, DT

SYMBOL_TYPE_SOURCE = 0
SYMBOL_TYPE_REPAIR = 1

# Match stream_fec.py's concatenation packing.
PACKET_LEN_PREFIX = 2


def _pack_header(symbol_type: int, symbol_size: int, esi: int,
                 window_size: int, repair_key: int, dt: int) -> bytes:
    return struct.pack(
        RLC_HEADER_STRUCT, RLC_MAGIC, 0, symbol_type, symbol_size,
        esi & 0xFFFFFFFF, window_size, repair_key & 0xFFFF, dt,
    )


def _unpack_header(env: bytes
                   ) -> Optional[tuple[int, int, int, int, int, int]]:
    """Return (symbol_type, symbol_size, esi, window_size, repair_key, dt)
    or None on bad magic / short envelope."""
    if len(env) < RLC_HEADER_LEN:
        return None
    magic, ver, stype, ss, esi, win, key, dt = struct.unpack_from(
        RLC_HEADER_STRUCT, env)
    if magic != RLC_MAGIC or ver != 0:
        return None
    return stype, ss, esi, win, key, dt


# ---------------------------------------------------------------------- #
# Encoder
# ---------------------------------------------------------------------- #
class RlcEncoder:
    def __init__(self, cfg) -> None:
        self.cfg = cfg
        self._enc = lib.swif_rlc_encoder_create(
            lib.SWIF_CODEPOINT_RLC_GF_256_FULL_DENSITY_CODEC,
            0,  # verbosity
            cfg.symbol_size,
            cfg.window,
        )
        if self._enc == ffi.NULL:
            raise RuntimeError("swif_rlc_encoder_create returned NULL")
        # We must keep Python references to every source buffer alive for as
        # long as it sits in the C-side window — the codec stores the buffer
        # pointer, not a copy. Map ESI → cffi buffer object so GC doesn't pull
        # the rug out from under it.
        self._kept_src: dict[int, "ffi.CData"] = {}
        self._current_symbol = bytearray()
        self._next_esi = 0           # ESI of the next source symbol to seal
        self._window_first = 0       # ESI of the leftmost symbol in window
        self._rng = random.Random(0xC0FFEE)  # for repair_key
        self.repair_per_source = max(1, round(cfg.overhead))

        # Stats (mirror RaptorQ's surface — tun_p2p reads these).
        self.blocks_encoded = 0   # for RLC we count "sealed source symbols"
        self.packets_in = 0
        self.symbols_out = 0
        self.bytes_in = 0

    def __del__(self) -> None:
        try:
            if hasattr(self, "_enc") and self._enc != ffi.NULL:
                lib.swif_rlc_encoder_release(self._enc)
        except Exception:
            pass

    def add_packet(self, pkt: bytes) -> list[bytes]:
        n = len(pkt)
        if n > self.cfg.max_packet_size:
            raise ValueError(
                f"packet {n}B exceeds max_packet_size {self.cfg.max_packet_size}B "
                f"(symbol_size {self.cfg.symbol_size} - {PACKET_LEN_PREFIX} prefix)"
            )
        self.packets_in += 1
        self.bytes_in += n
        envelopes: list[bytes] = []
        needed = PACKET_LEN_PREFIX + n
        if needed > self.cfg.symbol_size - len(self._current_symbol):
            envelopes += self._seal_current_symbol_and_emit()
        self._current_symbol += struct.pack("<H", n)
        self._current_symbol += pkt
        # Eagerly seal a symbol that's exactly full — there's no room left
        # for a length prefix anyway, so the next packet would trigger a
        # seal regardless. Sealing now lets the decoder emit this symbol's
        # packets without waiting for a follow-up call.
        if len(self._current_symbol) == self.cfg.symbol_size:
            envelopes += self._seal_current_symbol_and_emit()
        return envelopes

    def flush(self) -> list[bytes]:
        return self._seal_current_symbol_and_emit() if self._current_symbol else []

    def _seal_current_symbol_and_emit(self) -> list[bytes]:
        """Seal the current packing buffer as a source symbol, register it
        with the codec, emit the source envelope + N repair envelopes."""
        if not self._current_symbol:
            return []
        pad = self.cfg.symbol_size - len(self._current_symbol)
        if pad > 0:
            self._current_symbol += b"\x00" * pad
        source_bytes = bytes(self._current_symbol)
        self._current_symbol = bytearray()

        esi = self._next_esi & 0xFFFFFFFF
        self._next_esi += 1

        # Hand the codec a stable C-side buffer; keep Python ref alive.
        cbuf = ffi.new(f"uint8_t[{self.cfg.symbol_size}]", source_bytes)
        rc = lib.swif_rlc_encoder_add_source_symbol_to_coding_window(
            self._enc, cbuf, esi)
        if rc != lib.SWIF_STATUS_OK:
            raise RuntimeError(
                f"swif encoder add_source_symbol failed (esi={esi}, rc={rc})")
        self._kept_src[esi] = cbuf

        # Auto-evict left edge of window if we just pushed past `window`.
        if esi - self._window_first + 1 > self.cfg.window:
            evict = esi - self.cfg.window + 1
            for old in range(self._window_first, evict):
                # The C-side codec evicts implicitly when we add past window;
                # we just drop our Python-side reference.
                self._kept_src.pop(old, None)
            self._window_first = evict

        # Emit the source envelope.
        out = [_pack_header(SYMBOL_TYPE_SOURCE, self.cfg.symbol_size, esi,
                            1, 0, 0) + source_bytes]
        self.symbols_out += 1
        self.blocks_encoded += 1

        # Emit `repair_per_source` repair envelopes over the current window.
        win_first = self._window_first
        win_size = esi - win_first + 1
        for _ in range(self.repair_per_source):
            key = self._rng.randrange(1, 0x10000)
            rc = lib.swif_encoder_generate_coding_coefs(
                self._enc, key, self.cfg.density_threshold)
            if rc != lib.SWIF_STATUS_OK:
                raise RuntimeError(
                    f"swif generate_coding_coefs failed (rc={rc})")
            # Pre-allocate the repair buffer ourselves so the codec doesn't
            # malloc (otherwise we'd need to track-and-free); the codec
            # writes into our buffer when **new_buf is non-NULL.
            repair_buf = ffi.new(f"uint8_t[{self.cfg.symbol_size}]")
            rp = ffi.new("void**", repair_buf)
            rc = lib.swif_rlc_build_repair_symbol(self._enc, rp)
            if rc != lib.SWIF_STATUS_OK:
                raise RuntimeError(f"swif build_repair_symbol failed (rc={rc})")
            repair_bytes = bytes(ffi.buffer(repair_buf, self.cfg.symbol_size)[:])
            out.append(
                _pack_header(SYMBOL_TYPE_REPAIR, self.cfg.symbol_size,
                             win_first, win_size, key,
                             self.cfg.density_threshold)
                + repair_bytes)
            self.symbols_out += 1

        return out

    @property
    def pending_packets(self) -> int:
        return 1 if self._current_symbol else 0


# ---------------------------------------------------------------------- #
# Decoder
# ---------------------------------------------------------------------- #
class RlcDecoder:
    def __init__(self, cfg) -> None:
        self.cfg = cfg
        self._dec = lib.swif_rlc_decoder_create(
            lib.SWIF_CODEPOINT_RLC_GF_256_FULL_DENSITY_CODEC,
            0,
            cfg.symbol_size,
            cfg.window,
            cfg.window,  # max_linear_system_size — match window
        )
        if self._dec == ffi.NULL:
            raise RuntimeError("swif_rlc_decoder_create returned NULL")

        # Stats.
        self.blocks_decoded = 0   # for RLC: source symbols delivered
        self.blocks_unrecoverable = 0
        self.symbols_in = 0
        self.symbols_dropped_bad_cfg = 0
        self.symbols_dropped_stale_block = 0
        self.packets_out = 0
        self.bytes_out = 0

        # Source symbols arrive in two ways:
        #   1. Direct (SYMBOL_TYPE_SOURCE) — emit immediately to TUN.
        #   2. Recovered from repair via codec callback — buffered until the
        #      "emit in ESI order" gate releases them.
        # We track first_emitted_esi as a wall against re-emission, and a
        # buffer for out-of-order recovered symbols.
        self._next_emit_esi = 0
        self._pending_recovered: dict[int, bytes] = {}
        self._kept_bufs: list = []     # cffi buffer GC anchor

        # The codec calls us back when it recovers source symbols. cffi
        # callbacks must be kept alive (otherwise the trampoline goes away).
        self._cb_removed = ffi.callback(
            "void(void*, esi_t)", self._on_symbol_removed)
        self._cb_decodable = ffi.callback(
            "void*(void*, esi_t)", self._on_decodable)
        self._cb_decoded = ffi.callback(
            "void*(void*, void*, esi_t)", self._on_decoded)
        lib.swif_rlc_decoder_set_callback_functions(
            self._dec, self._cb_removed, self._cb_decodable,
            self._cb_decoded, ffi.NULL)

        # Track when each block-id-equivalent (we use ESI of first symbol in
        # the latest repair window) first arrived, for expiry bookkeeping.
        self._block_first_seen: dict[int, float] = {}

        # For draining recovered packets out to the caller, we accumulate
        # IP packets in `_emit_queue` and the public `add_symbol` returns
        # whatever's in there per-call.
        self._emit_queue: list[bytes] = []

    def __del__(self) -> None:
        try:
            if hasattr(self, "_dec") and self._dec != ffi.NULL:
                lib.swif_rlc_decoder_release(self._dec)
        except Exception:
            pass

    def _on_symbol_removed(self, ctx, esi):
        # Just a hook for the codec; nothing to do here.
        pass

    def _on_decodable(self, ctx, esi):
        # Allocate a stable buffer the codec will fill in. We keep a Python
        # reference so GC doesn't free it while the codec is writing.
        buf = ffi.new(f"uint8_t[{self.cfg.symbol_size}]")
        self._kept_bufs.append(buf)
        return buf

    def _on_decoded(self, ctx, buf, esi):
        # The decoder has just finished writing this source symbol's bytes
        # into the buffer we handed it in `_on_decodable`. Capture as a
        # bytes object (so we don't depend on the buffer staying alive)
        # and stash for in-order emission.
        symbol_bytes = bytes(ffi.buffer(buf, self.cfg.symbol_size)[:])
        self._pending_recovered[int(esi)] = symbol_bytes
        return ffi.NULL

    def _drain_in_order(self) -> None:
        """Move recovered (or directly-arrived) source-symbol bytes into the
        emit queue as long as the next-expected ESI is available."""
        while self._next_emit_esi in self._pending_recovered:
            sym = self._pending_recovered.pop(self._next_emit_esi)
            self._emit_packets_from_symbol(sym)
            self._next_emit_esi += 1

    def _emit_packets_from_symbol(self, symbol_bytes: bytes) -> None:
        ss = self.cfg.symbol_size
        pos = 0
        while pos + PACKET_LEN_PREFIX <= ss:
            ln = int.from_bytes(
                symbol_bytes[pos:pos + PACKET_LEN_PREFIX], "little")
            if ln == 0:
                break
            end = pos + PACKET_LEN_PREFIX + ln
            if end > ss:
                break
            pkt = bytes(symbol_bytes[pos + PACKET_LEN_PREFIX:end])
            self._emit_queue.append(pkt)
            self.packets_out += 1
            self.bytes_out += len(pkt)
            pos = end
        self.blocks_decoded += 1

    def add_symbol(self, envelope: bytes) -> list[bytes]:
        header = _unpack_header(envelope)
        if header is None:
            return []
        stype, ss, esi, win, key, dt = header
        if ss != self.cfg.symbol_size:
            self.symbols_dropped_bad_cfg += 1
            return []
        if win < 1 or win > self.cfg.window:
            self.symbols_dropped_bad_cfg += 1
            return []
        payload = envelope[RLC_HEADER_LEN:]
        if len(payload) != ss:
            self.symbols_dropped_bad_cfg += 1
            return []
        self.symbols_in += 1
        # Block bookkeeping (for expiry): key the timer on the leftmost ESI
        # of the symbol's window — repair groups share this, source symbols
        # are their own block.
        bid = esi if stype == SYMBOL_TYPE_SOURCE else esi
        self._block_first_seen.setdefault(bid, time.monotonic())

        if stype == SYMBOL_TYPE_SOURCE:
            # Feed the codec a stable buffer (it stores the pointer).
            buf = ffi.new(f"uint8_t[{ss}]", payload)
            self._kept_bufs.append(buf)
            rc = lib.swif_rlc_decoder_decode_with_new_source_symbol(
                self._dec, buf, esi)
            if rc != lib.SWIF_STATUS_OK:
                self.symbols_dropped_bad_cfg += 1
                return []
            # Buffer the directly-received source bytes for in-order emit.
            self._pending_recovered.setdefault(int(esi), payload)
            # Advance the emit gate to the lowest known ESI if we're still at 0.
            if self._next_emit_esi == 0 and self._pending_recovered:
                self._next_emit_esi = min(self._pending_recovered)
            self._drain_in_order()
        else:
            # Repair: rebuild the decoder's coding window for this repair's
            # source range, regenerate the same coefficients (TinyMT32 with
            # the shipped repair_key + dt), then decode.
            lib.swif_rlc_decoder_reset_coding_window(self._dec)
            for e in range(esi, esi + win):
                lib.swif_rlc_decoder_add_source_symbol_to_coding_window(
                    self._dec, e & 0xFFFFFFFF)
            lib.swif_decoder_generate_coding_coefs(self._dec, key, dt)
            buf = ffi.new(f"uint8_t[{ss}]", payload)
            self._kept_bufs.append(buf)
            rc = lib.swif_rlc_decoder_decode_with_new_repair_symbol(
                self._dec, buf, esi)
            if rc != lib.SWIF_STATUS_OK:
                return []
            # Codec callbacks have populated _pending_recovered for any newly
            # decoded source symbols. Drain in ESI order.
            self._drain_in_order()

        out = self._emit_queue
        self._emit_queue = []
        return out

    def expire_blocks_older_than(self, max_age_s: float) -> int:
        if not self._block_first_seen:
            return 0
        now = time.monotonic()
        expired = [bid for bid, t in self._block_first_seen.items()
                   if (now - t) > max_age_s]
        unrecoverable = 0
        for bid in expired:
            # An expired block is "unrecoverable" if it was a source
            # symbol that never made it past _next_emit_esi (we dropped
            # waiting for an earlier one) OR was a repair group whose
            # source coverage was incomplete. Approximate: count the
            # bids that are still below our emit gate AND aren't in the
            # recovered queue.
            if bid < self._next_emit_esi and bid not in self._pending_recovered:
                pass  # already delivered
            else:
                unrecoverable += 1
            del self._block_first_seen[bid]
        self.blocks_unrecoverable += unrecoverable
        # Also age out _pending_recovered entries below the new emit gate
        # by advancing the gate past stale unfilled entries.
        if expired and self._pending_recovered:
            min_age = max(expired) + 1
            if self._next_emit_esi < min_age:
                # Skip the gap; subsequent recoveries past this point are
                # treated as fresh.
                self._next_emit_esi = max(self._next_emit_esi, min_age)
            self._drain_in_order()
        return unrecoverable

    @property
    def in_flight_blocks(self) -> int:
        return len(self._block_first_seen)
