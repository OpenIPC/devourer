# Adaptive link — simulation, on-air validation, and open questions

This document records how the energy-minimizing adaptive video link
([design + comparison](adaptive-link.md)) has been evaluated — in simulation and
on the bench — what it has and hasn't confirmed, and exactly what remains, so the
work can be reproduced and completed. The conceptual design lives in
[`adaptive-link.md`](adaptive-link.md); this is the lab notebook and reproduction
guide.

The evaluation has two halves:

- **Simulation** — the [`linklab`](https://github.com/josephnef/linklab) sandbox,
  which imports this repo's energy/link/controller models as the single source of
  truth, so its accounting is identical to what ships here.
- **On-air** — the closed-loop harnesses in `tests/` driving real RTL88xxAU
  adapters plus a USRP B210 as a controllable interferer.

## Simulation results

All software, reproducible without hardware. The controller, energy model, and
link model are this repo's; linklab wraps them in a Gymnasium environment and a
GPU-batched twin that reproduce `tests/sim_loop.py` tick for tick.

**Energy.** Over a "fly out and back" path-loss schedule, the adaptive controller
spends **34 % less energy per delivered bit** than the best static energy-aware
operating point and **53 % less** than an over-provisioned robust profile, while
holding the per-layer delivery SLA. Reproduce: `tests/sim_loop.py`.

**Graceful degradation.** Pushing synthetic SVC-HEVC through the per-layer UEP
pipeline reproduces the staircase — as the link weakens the top enhancement layer
sheds first, then the next, while base/IDR holds near-perfect delivery far below
the point where enhancement is gone. Reproduce: `tools/precoder/svc_pipeline.py`,
`tests/gen_svc_nals.py`.

**Learned policy vs. controller.** On a *static*, fully-observed flight the
controller is an exact argmin over the channel model, so reinforcement learning
can at best match it. Under **time-correlated fading with a short SNR history**,
on the long-range / deep-fade flights a trained policy **holds the delivery SLA
that the fade-blind controller misses (≈ 0.99 vs ≈ 0.92), at lower energy** — it
Pareto-dominates exactly where it matters for long distance. This is the evidence
that a fade-aware control improvement is worth pursuing. Reproduce: linklab,
`python -m linklab.train.run --history 8 --fading-k 4`.

## The fade margin (opt-in)

The simulation finding motivated an **opt-in, default-off** addition to the
controller: a variance-aware **fade margin**. The controller tracks the variance
of its path-loss estimate (TX power removed, so its own power moves don't feed
back in) and, when `fade_margin_k > 0`, adds proportional **TXAGC headroom** —
power only — to the chosen operating point after selection.

The "power only, after selection" detail is load-bearing. Adding the margin to
operating-point *selection* instead makes the energy-min argmin satisfy the higher
SNR demand by dropping MCS / adding FEC (cheaper in power than TXAGC), which
raises airtime and *overloads* the channel during fades — making delivery worse.
Buying the margin with power keeps airtime fixed.

In simulation (linklab, fading), `fade_margin_k = 1` lifts mean delivery by ~1 pp
and worst-flight delivery by ~0.5 pp at ~+11 % energy. It is a worst-case
robustness knob for the long-range mission, **not** a full SLA restore — deep
fades need FEC time-diversity or the learned policy. It is off by default pending
the on-air comparison below.

## On-air validation

The bench is two RTL88xxAU adapters (an 8812AU transmitter, an 8821AU receiver)
driven by `StreamDuplexDemo` and `tools/precoder/adaptive_link.py`, plus a USRP
B210 raising the receiver's noise floor as a controllable, reproducible
interferer.

**Harness:** `tests/fade_sla_onair.sh` runs the closed loop twice at the same
interferer level — once with **static** noise, once with **time-correlated
fading** — and compares ground-station delivery. The two are **mean-power
matched** (the fading interferer uses a unit-mean log-normal power envelope, so a
given gain delivers the same mean power in both modes — `tests/sdr_interferer.py
--fade-coherence`). The harness sweeps interferer gain to find the regime where
the link is stressed but not jammed.

**Delivery is measured exactly.** The transmitter stamps each video frame with a
12-bit application sequence number, and the receiver computes loss from that — so
it counts true end-to-end per-video-frame delivery. (The raw 802.11 sequence
number advances per *transmitted* frame, including retries and other traffic, so
gap-based loss computed from it over-counts; `score.seq_gap_loss` documents this.)

### What the on-air sweep shows

At matched mean interference, **time-correlated fading degrades delivery far more
than equivalent static noise** — the core prediction of the simulation:

| interferer gain | static mean (p10) | fading mean (p10) |
|---|---|---|
| 46 | 0.86 (0.67) | 0.86 (0.80) |
| 52 | 0.89 (0.82) | 0.75 (**0.09**) |
| 58 | 0.84 (0.59) | 0.77 (0.19) |
| 64 | **0.74** (0.31) | **0.17** (0.00) |

(8812AU → 8821AU + B210, ch 6, exact app-sequence metric.) The mean collapses
0.74 → 0.17 at the strongest setting, and the worst-window p10 craters
(0.82 → 0.09 at gain 52) — fades punch through the controller's fixed margin even
when the average looks acceptable.

This is a clean **relative** confirmation. It is **not** an absolute SLA test: on
this bench the static baseline never reaches 0.99 (it tops out ≈ 0.89), because
the adapters sit inches apart with tens of dB of margin, so the interferer is
never in the "link holds at 0.99, then fades break it" regime. The residual loss
is real end-to-end loss (on-air + processing), now measured honestly.

## Open questions (need hardware not currently available)

1. **Absolute SLA-break, and the fade-margin go/no-go.** The relative fade
   penalty is confirmed; the absolute "baseline holds 0.99 → fading breaks it"
   test — and the decision to enable the fade margin by default — needs a channel
   that can be parked at ~0.99 and stressed in a controlled way. The bench can't
   reach that over the air. **An attenuator rig is the unblock** (see below). With
   it, the test is: at a fixed setpoint where static holds ~0.99, sweep fading and
   compare `fade_margin_k` off vs on.

2. **Calibrate a real build.** linklab's RF-chain model and this repo's energy
   model are nominal. `tests/calibrate_energy.py` (thermal + SDR power sweeps) and
   `tests/calibrate_link.py` (interferer sweeps) fit them to a specific build;
   `linklab`'s `calib/fit.py` + `export.py` then produce a devourer
   `energy_calib.json`. This needs the sweeps run on hardware to anchor the
   absolute energy numbers (relative savings hold without it).

3. **On-air adaptive SVC.** The per-temporal-layer ladder flies at fixed MCS today
   (`SvcTxDemo`, `tests/svc_uep_onair.sh`); retuning the per-layer ladder and shed
   set live from the ground controller is validated in simulation (linklab) but
   not yet on air.

4. **Real RC uplink.** The rendezvous/failsafe watchdog input is abstracted; a
   real command-radio uplink wires into it directly.

### What the attenuator rig looks like

Replace the over-the-air coupling with cabled (conducted) RF through attenuators
so the link SNR and the interferer-to-signal ratio are set exactly and
reproducibly:

```
  8812AU TX ──[ A_link ]──┐
                          │   resistive / Wilkinson
   B210 TX ──[ A_intf ]───┤   combiner (reciprocal)  ├── 8821AU RX
                          │
            unused ports → 50 Ω terminations
```

- `A_link` (fixed pad, ~30–50 dB) sets the link budget; because the combiner is
  reciprocal, the receiver's feedback flows back through it to the transmitter
  (this link is half-duplex bidirectional).
- `A_intf` (a USB step attenuator, e.g. Mini-Circuits RUDAT/RCDAT) sets the
  interferer level and is the swept knob — replacing the B210 gain sweep with a
  reproducible one.
- **Critical:** shield the adapters (RF enclosures) or separate them enough that
  the cabled path dominates the air path — near-field leakage at inches apart
  defeats the attenuators. Verify by unplugging one cable: if frames still flow,
  the air path is leaking.

Calibrate by setting `A_link` so the un-interfered receiver reports ~25–30 dB SNR
(≈ 0.99 delivery), then sweep `A_intf`. `tests/fade_sla_onair.sh` is ~90 % of the
way there; the remaining change is swapping the B210 gain sweep for step-attenuator
control.

## Reproduce

Software only (no hardware):

```sh
# energy savings + closed-loop A/B
python3 tests/sim_loop.py
# SVC-HEVC per-layer UEP staircase
cd tools/precoder && uv run pytest test_svc_pipeline.py
# learned policy vs controller under fading (the linklab repo)
#   git clone https://github.com/josephnef/linklab as a sibling of this repo
cd linklab && uv sync --extra rl
uv run python -m linklab.train.run --history 8 --fading-k 4 --n-envs 512
```

On-air (needs two RTL88xxAU adapters + a USRP B210):

```sh
cmake -S . -B build && cmake --build build -j        # build StreamDuplexDemo
# baseline link diagnosis (no interferer): localize any frame loss
sudo bash tests/halfduplex_baseline.sh
# fade-SLA sweep: static vs mean-matched fading across interferer gains
sudo bash tests/fade_sla_onair.sh
```

Per-cell logs land in `/tmp/`. See `tests/README.md` for adapter wiring and the
out-of-band regression rig.
