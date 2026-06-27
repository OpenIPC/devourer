"""Fused-FEC link core — RS + sub-block-integrity (SBI) sender/receiver that
runs over the devourer chip↔chip stream link (StreamTxDemo / WiFiDriverDemo).

This is the chip-path (scenario 1) sibling of the SDR capstone
(`~/git/sdr2wifi/fused_fec_rung1.py`): the same SBI-over-RS framing, but the
receiver consumes `<devourer-stream>` lines (with DEVOURER_RX_KEEP_CORRUPTED)
instead of gr-ieee802-11 PDUs. The chip gives only hard corrupted bytes (no
LLRs), so erasure localization is purely the per-sub-block CRC.

Both ends share a `FecConfig` (scheme=rs) and `blocks_per_body`; the receiver
derives the envelope size the same way the sender does, so a body partitions
deterministically even when the chip flagged it corrupt.

The receiver runs TWO decoders in lockstep so an on-air run reports the gain
directly:
  * baseline — drops any crc_err frame (today's behaviour),
  * sbi      — keeps it and salvages the CRC-valid sub-blocks.

CLI wrappers: `fused_fec_tx.py` (bytes→bodies→StreamTxDemo) and
`fused_fec_rx.py` (`<devourer-stream>`→recovered bytes + gain report).
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import fec_subblock  # noqa: E402
import stream_fec  # noqa: E402
from stream_fec import FecConfig  # noqa: E402


def env_size(cfg: FecConfig) -> int:
    """Outer-code envelope (symbol) size for a config — by a throwaway encode,
    so no scheme header length is hard-coded. Identical on both ends."""
    enc = stream_fec.make_encoder(cfg)
    envs: list[bytes] = []
    for _ in range(cfg.k):
        envs += enc.add_packet(b"\x00" * min(8, cfg.max_packet_size))
    envs += enc.flush()
    return len(envs[0])


def _chunks(data: bytes, n: int):
    for i in range(0, len(data), n):
        yield data[i:i + n]


class FusedFecSender:
    def __init__(self, cfg: FecConfig, blocks_per_body: int,
                 crc_bytes: int = 2, stream_id: int = 0) -> None:
        self.cfg = cfg
        self.enc = stream_fec.make_encoder(cfg)
        self.env = env_size(cfg)
        self.packer = fec_subblock.SubBlockPacker(
            self.env, blocks_per_body, crc_bytes=crc_bytes, stream_id=stream_id)
        self.mtu = cfg.max_packet_size

    def add_bytes(self, data: bytes) -> list[bytes]:
        bodies: list[bytes] = []
        for chunk in _chunks(data, self.mtu):
            for envelope in self.enc.add_packet(chunk):
                bodies += self.packer.add(envelope)
        return bodies

    def flush(self) -> list[bytes]:
        bodies: list[bytes] = []
        for envelope in self.enc.flush():
            bodies += self.packer.add(envelope)
        bodies += self.packer.flush()
        return bodies


@dataclass
class RxReport:
    frames_seen: int = 0
    frames_corrupt: int = 0
    subblocks_total: int = 0
    subblocks_salvaged: int = 0     # CRC-valid sub-blocks from corrupt frames
    base_blocks: int = 0
    sbi_blocks: int = 0
    base_packets: int = 0
    sbi_packets: int = 0


class FusedFecReceiver:
    """Consumes (body, crc_err) frames; runs baseline + SBI decoders in
    parallel. `add_frame` returns the packets the SBI decoder recovered from
    this frame (the live output); the gain vs baseline is in `report()`."""

    def __init__(self, cfg: FecConfig, blocks_per_body: int,
                 crc_bytes: int = 2) -> None:
        self.cfg = cfg
        self.env = env_size(cfg)
        self.crc_bytes = crc_bytes
        self.base_dec = stream_fec.make_decoder(cfg)
        self.sbi_dec = stream_fec.make_decoder(cfg)
        self.r = RxReport()

    def add_frame(self, body: bytes, crc_err: bool) -> list[bytes]:
        self.r.frames_seen += 1
        if crc_err:
            self.r.frames_corrupt += 1
        res = fec_subblock.unpack(body, self.env, self.crc_bytes)
        self.r.subblocks_total += res.n_blocks
        if crc_err:
            self.r.subblocks_salvaged += len(res.survivors)

        # Baseline: a crc_err frame is dropped wholesale.
        if not crc_err:
            for env in res.survivors:
                got = self.base_dec.add_symbol(env)
                self.r.base_packets += len(got)
        # SBI: salvage the surviving sub-blocks regardless.
        out: list[bytes] = []
        for env in res.survivors:
            got = self.sbi_dec.add_symbol(env)
            out += got
            self.r.sbi_packets += len(got)
        return out

    def report(self) -> RxReport:
        self.r.base_blocks = self.base_dec.blocks_decoded
        self.r.sbi_blocks = self.sbi_dec.blocks_decoded
        return self.r
