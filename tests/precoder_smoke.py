"""Smoke test for the pre-modulator subcarrier encoder (tools/precoder).

End-to-end round-trip the PoC plan calls for, wired into the repo's pytest
suite. Runs on any host with numpy and skips cleanly without it — no USB, no
SDR, no hardware. The exhaustive DSP known-answer tests live in
tools/precoder/test_pipeline.py and run under that subtree's uv env:

    cd tools/precoder && uv venv && uv sync && uv run pytest
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")

PRECODER = Path(__file__).resolve().parent.parent / "tools" / "precoder"
sys.path.insert(0, str(PRECODER))
import encode_subcarriers as enc  # noqa: E402

PHYS = [enc.phy_params("ht"), enc.phy_params("legacy")]
PHY_IDS = ["ht", "legacy"]


@pytest.mark.parametrize("phy", PHYS, ids=PHY_IDS)
def test_representable_pattern_roundtrips_exactly(phy):
    """encode -> emulate chip pipeline -> constellation == input, bit-exact."""
    rng = np.random.default_rng(0)
    seed = enc.DEFAULT_SEED
    # A reachable target: random info -> forward model -> use its constellation.
    info = rng.integers(0, 2, size=phy.n_dbps, dtype=np.uint8)
    psdu_bits = enc.apply_scrambler(info, seed)
    target = enc.emulate_chip(psdu_bits, seed, phy, n_sym=1)

    res = enc.encode_pattern(target, seed=seed, phy=phy)
    assert res.representable and res.hamming_distance == 0
    back = enc.emulate_chip(res.psdu_bits, seed, phy, n_sym=1)
    assert np.array_equal(back, target)


@pytest.mark.parametrize("phy", PHYS, ids=PHY_IDS)
def test_alternating_example_reports_nearest(phy):
    """The plan's [+1,-1,...] example: generally not a codeword, so the encoder
    must report a nearest-match distance and the emulation must differ from the
    request in exactly that many subcarriers."""
    target = enc.alternating_pattern(phy.n_sd)
    res = enc.encode_pattern(target, phy=phy)  # non-strict
    back = enc.emulate_chip(res.psdu_bits, res.seed, phy, n_sym=1)
    assert int(np.count_nonzero(back != target)) == res.hamming_distance


def test_psdu_bytes_are_emitted():
    """The deliverable: encode_pattern yields packable PSDU bytes."""
    phy = enc.phy_params("ht")
    target = enc.alternating_pattern(phy.n_sd, n_sym=2)
    res = enc.encode_pattern(target, phy=phy)
    assert isinstance(res.psdu_bytes, (bytes, bytearray))
    assert len(res.psdu_bytes) == (2 * phy.n_dbps + 7) // 8
