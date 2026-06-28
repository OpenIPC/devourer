"""Tests for the adaptive-link link model (`link_model.py`).

Pins the delivery monotonicities the controller relies on: P_deliver rises with
SNR and with FEC overhead; the SNR a row needs rises with MCS and falls with
overhead; and a measured calibration channel overrides the nominal one.
"""

from __future__ import annotations

import link_model as lm


def test_p_deliver_rises_with_snr():
    m = lm.LinkModel(trials=600)
    vals = [m.p_deliver(snr, mcs=3, overhead=0.25) for snr in range(8, 24, 2)]
    assert vals[0] <= vals[-1]
    assert vals[0] < 0.5 < vals[-1]                            # crosses the waterfall
    # non-decreasing within Monte-Carlo noise
    assert all(b >= a - 0.05 for a, b in zip(vals, vals[1:]))


def test_more_overhead_helps():
    m = lm.LinkModel(trials=800)
    snr = 12.0
    light = m.p_deliver(snr, mcs=3, overhead=0.25)
    heavy = m.p_deliver(snr, mcs=3, overhead=1.00)
    assert heavy >= light


def test_snr_required_rises_with_mcs():
    m = lm.LinkModel(trials=600)
    req = [m.snr_required(mcs, 0.25, 0.99) for mcs in range(8)]
    # higher MCS needs more SNR (allow ties from the 0.5 dB grid)
    assert all(b >= a for a, b in zip(req, req[1:]))
    assert req[7] > req[0]


def test_heavier_fec_lowers_snr_required():
    m = lm.LinkModel(trials=800)
    assert m.snr_required(5, 1.00, 0.99) < m.snr_required(5, 0.25, 0.99)


def test_synth_channel_shape():
    # at high SNR a corrupt frame keeps most sub-blocks (localized); at low SNR few
    hi = lm.synth_channel(3, 20.0)
    lo = lm.synth_channel(3, 4.0)
    mean_hi = sum(k * v for k, v in hi["survivor_hist"].items()) / sum(hi["survivor_hist"].values())
    mean_lo = sum(k * v for k, v in lo["survivor_hist"].items()) / sum(lo["survivor_hist"].values())
    assert mean_hi > mean_lo
    assert hi["corrupt_rate"] < lo["corrupt_rate"]            # less corrupt at high SNR


def test_calibration_override(tmp_path):
    import json
    # a measured channel that is perfect at MCS7/10dB (override the fragile nominal)
    ch = {"corrupt_rate": 0.0, "n_sub": 10, "survivor_hist": {10: 100}}
    p = tmp_path / "link.json"
    p.write_text(json.dumps({"channels": {"7:10": ch}}))
    m = lm.LinkModel(calib_path=str(p), trials=400)
    assert m.channel(7, 10.0)["corrupt_rate"] == 0.0
    assert m.p_deliver(10.0, mcs=7, overhead=0.10) > 0.99     # uses the measured channel
