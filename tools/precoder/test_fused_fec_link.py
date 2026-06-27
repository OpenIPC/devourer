"""Tests for the chip-path fused-FEC link core (`fused_fec_link.py`).

Software loopback: sender → (optional localized corruption) → receiver. Proves
the clean round-trip and that SBI salvage beats the drop-whole-frame baseline
under the localized corruption a real noisy chip link produces.
"""

from __future__ import annotations

import random

from fused_fec_link import FusedFecReceiver, FusedFecSender, env_size
from stream_fec import FecConfig


def test_clean_round_trip_reconstructs_stream():
    cfg = FecConfig(k=8, symbol_size=64, overhead=0.5, scheme="rs")  # N=12
    snd = FusedFecSender(cfg, blocks_per_body=4)  # 12 env / 4 = 3 bodies/block
    msg = bytes((i * 7) & 0xFF for i in range(8 * 62 * 3))  # whole packets
    bodies = snd.add_bytes(msg) + snd.flush()

    rcv = FusedFecReceiver(cfg, blocks_per_body=4)
    out = b"".join(pkt for b in bodies for pkt in rcv.add_frame(b, crc_err=False))
    assert out == msg
    rep = rcv.report()
    assert rep.frames_corrupt == 0 and rep.sbi_blocks == rep.base_blocks > 0


def _corrupt_one_subblock(body: bytes, env: int, rng, crc_bytes: int = 2) -> bytes:
    import fec_subblock
    stride = crc_bytes + env
    n = (len(body) - fec_subblock.SBI_HDR_LEN) // stride
    j = rng.randrange(n)
    b = bytearray(body)
    base = fec_subblock.SBI_HDR_LEN + j * stride + crc_bytes
    b[base + rng.randrange(env)] ^= 0xFF
    return bytes(b)


def test_sbi_beats_baseline_one_block_per_frame():
    # One whole RS block per radio frame (N=10, R=2). A single FCS-failed frame
    # is a total block loss for the baseline but recoverable by SBI (only the
    # one corrupt sub-block is lost, 9 >= K=8 survive).
    cfg = FecConfig(k=8, symbol_size=32, overhead=0.25, scheme="rs")  # N=10
    snd = FusedFecSender(cfg, blocks_per_body=10)
    env = env_size(cfg)
    rng = random.Random(3)

    NB = 40
    frames: list[tuple[bytes, bool]] = []
    for blk in range(NB):
        msg = bytes(((blk * 17 + i) & 0xFF) for i in range(8 * 30))
        bodies = snd.add_bytes(msg) + snd.flush()
        # Each block = exactly one body (10 env / 10 per body). Corrupt ~25%.
        for body in bodies:
            if rng.random() < 0.25:
                frames.append((_corrupt_one_subblock(body, env, rng), True))
            else:
                frames.append((body, False))

    rcv = FusedFecReceiver(cfg, blocks_per_body=10)
    for body, crc_err in frames:
        rcv.add_frame(body, crc_err)
    rep = rcv.report()

    assert rep.frames_corrupt > 0
    # SBI recovers the corrupt frames' blocks; baseline drops them.
    assert rep.sbi_blocks > rep.base_blocks
    assert rep.sbi_blocks == NB                 # SBI gets every block
    assert rep.base_blocks == NB - rep.frames_corrupt
    assert rep.subblocks_salvaged > 0
