"""Is SBI worth its complexity vs. just spending the same overhead on more parity?

The strategic A/B for the fused-FEC initiative. Three schemes decode the SAME
real channel (measured survivor distributions, below) at EQUAL on-air overhead:

  * PLAIN  — inter-frame Reed-Solomon, frame = atomic symbol group, NO per-block
             CRC. A frame either passes the FCS (all its slots usable) or fails
             (whole frame erased — plain cannot tell which bytes survived). The
             bytes SBI spends on per-sub-block CRCs become extra PARITY here.
             This is the wfb-ng / wifibroadcast model.
  * SBI-INTER — fused: CRC-guarded sub-blocks are the RS symbols, RS spans
             frames. A corrupt frame contributes its CRC-valid survivor
             sub-blocks (partial-frame recovery) AND the block spans frames.
             Pays the CRC tax.
  * SBI-INTRA — the shipped config: one body = one RS block (N = blocks/body).
             Per-frame recovery only (no inter-frame protection); a frame too
             corrupt to clear its own RS threshold is a total loss for its block.

Overhead accounting (bytes): an RS symbol carries b = 32 source/parity bytes.
PLAIN slot = b on air; SBI slot = b + c (c = 2-byte sub-block CRC) + amortised
SBI header. So at EQUAL airtime PLAIN buys ~c/b = 6 % more parity. The question
is whether SBI's partial-frame salvage outweighs that, and the answer depends
entirely on how LOCALIZED the corruption is — which is why this runs over the
real measured channels, not an assumed one.

Channel = (corrupt-frame rate, per-corrupt-frame survivor histogram over the 10
sub-blocks), measured by `~/git/sdr2wifi/survivor_hist.py` (SDR) and the chip
raw log. Corruption is drawn i.i.d. from the marginal histogram — i.e. SBI gets
NO credit for real burst locality, so this is conservative toward PLAIN.
"""

from __future__ import annotations

import argparse
import random

# Measured channels (n_sub = 10 sub-blocks/frame). survivor_hist[s] = number of
# corrupt frames that kept exactly s CRC-valid sub-blocks.
CHANNELS = {
    # Realtek 8821, hardware soft decode, MCS7 @ TX_PWR=6 (real silicon).
    "chip": {"corrupt_rate": 0.8315, "n_sub": 10,
             "survivor_hist": {0: 15, 1: 11, 2: 20, 3: 24, 4: 44, 5: 53, 6: 77,
                               7: 132, 8: 159, 9: 195, 10: 15}},
    # B210 + gr-ieee802-11 fork, SOFT Viterbi (localized; bimodal).
    "sdr_soft": {"corrupt_rate": 0.7232, "n_sub": 10,
                 "survivor_hist": {0: 173, 1: 14, 2: 1, 3: 7, 4: 18, 5: 33,
                                   6: 70, 7: 164, 8: 305, 9: 501, 10: 18}},
    # B210 + fork, HARD Viterbi (frame-wide divergence; spread).
    "sdr_hard": {"corrupt_rate": 0.9968, "n_sub": 10,
                 "survivor_hist": {0: 374, 1: 80, 2: 152, 3: 208, 4: 294, 5: 282,
                                   6: 235, 7: 139, 8: 67, 9: 14}},
}

B = 32      # RS symbol payload bytes
C = 2       # SBI per-sub-block CRC bytes
HDR = 7     # SBI body header bytes (amortised over n_sub slots)


def _hist_sampler(hist: dict[int, int]):
    keys = sorted(hist)
    weights = [hist[k] for k in keys]
    total = sum(weights)
    cum = []
    acc = 0
    for w in weights:
        acc += w
        cum.append(acc)

    def sample(rng: random.Random) -> int:
        x = rng.randrange(total)
        for k, c in zip(keys, cum):
            if x < c:
                return k
        return keys[-1]
    return sample


def _frame(rng, rate, sampler, n_sub):
    """Return (is_clean, surviving_slots) for one received frame."""
    if rng.random() >= rate:
        return True, n_sub
    return False, sampler(rng)


# --------------------------------------------------------------------------- #
# Per-scheme on-air byte cost of one frame (10 slots), and source-delivery sim.
# --------------------------------------------------------------------------- #
def frame_bytes_plain(n_sub):
    return n_sub * B


def frame_bytes_sbi(n_sub):
    return n_sub * (B + C) + HDR


def sim_interframe(channel, ov, frames_per_gen, trials, rng, sbi):
    """PLAIN (sbi=False) or SBI-INTER (sbi=True). RS spans `frames_per_gen`
    frames; recover all source iff surviving slots >= source slots.
    `ov` is on-air redundancy beyond source bytes (CRC tax counts for SBI)."""
    rate = channel["corrupt_rate"]
    n = channel["n_sub"]
    sampler = _hist_sampler(channel["survivor_hist"])
    total_slots = n * frames_per_gen

    # Split slots into source S and parity P so on-air overhead == ov.
    # PLAIN:  airtime = total_slots*B ; source_bytes = S*B  -> ov = P/S (+0 tax)
    # SBI:    airtime = total_slots*(B+C)+HDR*frames ; source_bytes = S*B
    if sbi:
        airtime = total_slots * (B + C) + HDR * frames_per_gen
    else:
        airtime = total_slots * B
    # source bytes target = airtime / (1+ov); S = that / B (clamped)
    S = max(1, min(total_slots, round(airtime / (1.0 + ov) / B)))
    if S >= total_slots:      # not enough room for any parity at this ov
        return 0.0, S
    delivered = 0
    for _ in range(trials):
        surviving = 0
        for _ in range(frames_per_gen):
            clean, surv = _frame(rng, rate, sampler, n)
            surviving += n if (clean) else (surv if sbi else 0)
        if surviving >= S:
            delivered += 1
    return delivered / trials, S


def sim_intra(channel, trials, rng, k_s):
    """SBI-INTRA: each frame is its own RS(n_sub, k_s) block; recover its k_s
    source slots iff survivors >= k_s. Returns (delivery, overhead)."""
    rate = channel["corrupt_rate"]
    n = channel["n_sub"]
    sampler = _hist_sampler(channel["survivor_hist"])
    ov = (frame_bytes_sbi(n) - k_s * B) / (k_s * B)
    rec = 0
    for _ in range(trials):
        clean, surv = _frame(rng, rate, sampler, n)
        s = n if clean else surv
        if s >= k_s:
            rec += 1
    return rec / trials, ov


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--channel", choices=list(CHANNELS) + ["all"], default="all")
    p.add_argument("--frames-per-gen", type=int, default=12,
                   help="inter-frame RS block span (frames)")
    p.add_argument("--trials", type=int, default=4000)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--ov", type=float, nargs="*",
                   default=[0.15, 0.25, 0.35, 0.50, 0.70, 1.00])
    p.add_argument("--rate-sweep", action="store_true",
                   help="vary corrupt-rate over a fixed survivor SHAPE to find "
                        "the regime where SBI overtakes plain")
    p.add_argument("--shape", choices=["chip", "sdr_soft", "sdr_hard"],
                   default="chip", help="survivor shape for --rate-sweep")
    p.add_argument("--sweep-ov", type=float, default=0.50,
                   help="fixed overhead for --rate-sweep")
    a = p.parse_args()

    if a.rate_sweep:
        shape = CHANNELS[a.shape]["survivor_hist"]
        print(f"rate-sweep: localized SHAPE='{a.shape}' "
              f"(mean survivors {sum(k*v for k,v in shape.items())/sum(shape.values()):.1f}/10), "
              f"fixed overhead {a.sweep_ov:.0%}, gen={a.frames_per_gen}, {a.trials} trials")
        print(f"{'corrupt%':>8} | {'PLAIN':>7} | {'SBI-inter':>9} | gap")
        print("-" * 40)
        for rate in [0.05, 0.10, 0.20, 0.30, 0.40, 0.50, 0.65, 0.80, 0.95]:
            ch = {"corrupt_rate": rate, "n_sub": 10, "survivor_hist": shape}
            rng = random.Random(a.seed)
            dp, _ = sim_interframe(ch, a.sweep_ov, a.frames_per_gen, a.trials, rng, sbi=False)
            ds, _ = sim_interframe(ch, a.sweep_ov, a.frames_per_gen, a.trials, rng, sbi=True)
            print(f"{rate*100:7.0f}% | {dp:7.3f} | {ds:9.3f} | {ds - dp:+.3f}")
        return 0

    names = list(CHANNELS) if a.channel == "all" else [a.channel]
    for name in names:
        ch = CHANNELS[name]
        rng = random.Random(a.seed)
        print(f"\n=== channel '{name}': corrupt-rate {ch['corrupt_rate']:.0%}, "
              f"mean survivors/corrupt-frame "
              f"{sum(k*v for k,v in ch['survivor_hist'].items())/sum(ch['survivor_hist'].values()):.1f}"
              f"/{ch['n_sub']} === (gen={a.frames_per_gen} frames, {a.trials} trials)")
        print(f"{'overhead':>9} | {'PLAIN':>7} | {'SBI-inter':>9} | inter-vs-plain")
        print("-" * 48)
        for ov in a.ov:
            dp, _ = sim_interframe(ch, ov, a.frames_per_gen, a.trials, rng, sbi=False)
            ds, _ = sim_interframe(ch, ov, a.frames_per_gen, a.trials, rng, sbi=True)
            print(f"{ov:9.2f} | {dp:7.3f} | {ds:9.3f} | {ds - dp:+.3f}")
        # SBI-intra discrete points (the shipped per-frame config)
        print("  SBI-intra (shipped, per-frame):", end=" ")
        for k_s in (9, 8, 7, 6):
            di, ovi = sim_intra(ch, a.trials, rng, k_s)
            print(f"k={k_s}(ov {ovi:.0%})->{di:.2f}", end="  ")
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
