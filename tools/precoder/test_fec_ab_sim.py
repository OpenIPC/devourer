"""Tests for the SBI-vs-plain-FEC A/B (`fec_ab_sim.py`).

Pins the structural conclusions: at zero loss both schemes deliver everything and
SBI's CRC tax is pure waste; as loss rises SBI (partial-frame salvage) overtakes
plain (whole-frame erasure) when corruption is localized; and when corruption is
frame-wide neither inter-frame scheme rescues a near-100%-corrupt channel.
"""

from __future__ import annotations

import random

import fec_ab_sim as ab


def _ch(rate, shape="chip"):
    return {"corrupt_rate": rate, "n_sub": 10,
            "survivor_hist": ab.CHANNELS[shape]["survivor_hist"]}


def test_zero_loss_both_deliver():
    rng = random.Random(0)
    ch = _ch(0.0)
    dp, _ = ab.sim_interframe(ch, 0.25, 12, 2000, rng, sbi=False)
    ds, _ = ab.sim_interframe(ch, 0.25, 12, 2000, rng, sbi=True)
    assert dp == 1.0 and ds == 1.0          # no loss → both perfect


def test_sbi_never_worse_localized():
    """For localized corruption, SBI-inter >= PLAIN at every overhead/rate."""
    for rate in (0.1, 0.3, 0.5, 0.8):
        for ov in (0.25, 0.5, 1.0):
            rng = random.Random(7)
            ch = _ch(rate, "chip")
            dp, _ = ab.sim_interframe(ch, ov, 12, 3000, rng, sbi=False)
            ds, _ = ab.sim_interframe(ch, ov, 12, 3000, rng, sbi=True)
            assert ds >= dp - 0.02          # SBI at least as good (MC noise slack)


def test_sbi_decisive_at_high_localized_loss():
    """At 50% loss / 50% overhead, plain collapses while SBI holds."""
    rng = random.Random(7)
    ch = _ch(0.5, "chip")
    dp, _ = ab.sim_interframe(ch, 0.5, 12, 4000, rng, sbi=False)
    ds, _ = ab.sim_interframe(ch, 0.5, 12, 4000, rng, sbi=True)
    assert dp < 0.4 and ds > 0.9            # plain dead, SBI alive


def test_low_loss_gap_is_small():
    """Below ~10% loss the schemes are close — SBI's tax is not worth it."""
    rng = random.Random(7)
    ch = _ch(0.07, "chip")
    dp, _ = ab.sim_interframe(ch, 0.25, 12, 4000, rng, sbi=False)
    ds, _ = ab.sim_interframe(ch, 0.25, 12, 4000, rng, sbi=True)
    assert ds - dp < 0.1


def test_framewide_corruption_neither_inter_scheme_saves_full_loss():
    """sdr_hard: ~100% corrupt, frame-wide — inter-frame schemes can't recover."""
    rng = random.Random(7)
    ch = ab.CHANNELS["sdr_hard"]
    dp, _ = ab.sim_interframe(ch, 0.5, 12, 3000, rng, sbi=False)
    ds, _ = ab.sim_interframe(ch, 0.5, 12, 3000, rng, sbi=True)
    assert dp < 0.05 and ds < 0.1


def test_intra_overhead_monotone():
    """SBI-intra overhead rises as k_s falls (more parity per frame)."""
    rng = random.Random(0)
    ovs = [ab.sim_intra(_ch(0.5), 500, rng, k)[1] for k in (9, 8, 7, 6)]
    assert ovs == sorted(ovs)               # k=9 cheapest, k=6 dearest
