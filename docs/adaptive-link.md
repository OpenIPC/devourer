# Energy-minimizing adaptive video link

This document describes the **adaptive video link** for the OpenIPC /
wifibroadcast-style one-way downlink from a drone transmitter (**VTX**) to a
ground station (**VRX**), and how it differs from the existing adaptive systems
in the field — principally OpenIPC's `alink`.

Its one-line thesis: **most adaptive links maximize video quality within a power
budget; this one minimizes energy-per-delivered-bit subject to a video-quality
floor.** It is the *dual* of `alink`, built for the case where battery endurance
(or thermal headroom, or shared-spectrum politeness) is the scarce resource and
"good enough" video is the constraint, not the objective.

All of it is userspace. The radio knobs it drives already exist in
`src/RtlJaguarDevice` (per-packet radiotap MCS, `SetTxPowerOverride` +
`ApplyTxPower`, `SetMonitorChannel`, `GetThermalStatus`); the policy lives in
Python under `tools/precoder/`, and the C++ demo gains only mechanical control
hooks.

## The problem

A long-range drone video link runs one-way and broadcast: there are no
link-layer ACKs, so the transmitter cannot learn per-packet success from the MAC
the way an 802.11 station does. The classic deployment picks a single operating
point — one MCS, one TX power, one FEC overhead — sized for the *worst* moment of
the flight. That is wasteful in two directions:

- **Close to the operator** the link has 20–30 dB of margin it never uses. Full
  TX power into a strong link buys nothing but heat, battery drain, and a larger
  interference footprint; heavy FEC and a robust low MCS burn airtime that could
  have been idle (energy) or carried more video (quality).
- **At long range** the fixed point either falls off a cliff (too aggressive) or
  was so conservative that the whole flight paid for the worst case.

The levers available, ranked by how they move **energy per delivered bit**:

| Lever | Link effect | Energy effect |
|---|---|---|
| **MCS / FEC overhead** (time-on-air) | strong | **strong** — less airtime → less PA-on time → fewer Joules/bit |
| **Channel / bandwidth** | strong | moderate |
| **TX power (TXAGC)** | **strongest** | **weak** — a 6.3× power increase costs only ~40% more energy/bit because the baseline circuit draw dominates |

The asymmetry in the last two rows is the whole game: **time-on-air is the
dominant energy lever and radiated power is the dominant *link* lever but a weak
*energy* lever.** An energy-minimizing controller therefore rides the highest MCS
the link will bear (short airtime) and spends only the *minimum* TX power needed
to clear that MCS — the opposite reflex of a throughput-maximizer, which spends
power freely to unlock a still-higher MCS.

## Objective: energy-min subject to a UEP SLA

Formally, per FEC block on a given video layer, the cost is

```
            P_baseline + airtime · P_pa(txagc)
  E_bit  =  ──────────────────────────────────
                 src_bitrate · P_deliver
```

- `P_baseline` — LO + baseband + USB + RX, always on (the floor that makes power
  a weak energy lever).
- `airtime = t_pre + payload_bits / R_phy(MCS, BW, SGI)` — the on-air fraction;
  this is what MCS/FEC move.
- `P_pa(txagc)` — incremental PA draw at the chosen TXAGC index.
- `P_deliver` — post-FEC block-delivery probability at the link SNR (from the
  link model). Failed delivery means the Joules were spent for nothing, so it
  divides the cost.

The controller minimizes `E_bit` **subject to** an unequal-error-protection
service-level agreement, not a single quality number:

- **base / IDR layers: ≥ 99 % post-FEC delivery** — non-negotiable, never shed.
- **enhancement layers: best-effort** — the slack the controller sheds first
  when SNR will not support them at any feasible energy.

This is the cross-layer UEP of Abdel-Khalek & Heath (joint MCS + FEC source-aware
protection): importance is protected on *both* the PHY-rate and the outer-FEC-
rate knobs at once, producing a graceful-degradation staircase instead of one
cliff. See [Fused FEC](fused-fec.md) for the error-correction stack the SLA is
stated against.

## Architecture

```
 VTX (drone, one chip, StreamDuplexDemo)              VRX (ground, one chip / SDR, StreamDuplexDemo)
   video NALs ─▶ classify + per-layer UEP-FEC ─┐        ┌─ <devourer-stream> (RSSI / SNR / crc / seq)
   per-layer radiotap MCS ─▶ duplex.stdin       │ video  │  ▶ sliding-window SCORE + post-FEC residual
   apply control ◀── parse RCF                  │ ─────▶ │  ▶ energy-min CONTROLLER → operating point
     SetTxPowerOverride / ApplyTxPower          │  air   │  ▶ RCF (profile + power + FEC) every ~100 ms
     per-layer MCS ladder, FEC overhead          │ ◀───── │  └─ DISC beacon when the VTX is lost
   watchdog ─▶ MAX_RANGE failsafe / DISCOVERY    │  RCF
```

Two design decisions shape it:

- **Ground-station-authoritative.** The VRX has the clean receive-side view
  (RSSI/SNR/CRC/seq + post-FEC residual loss), so it computes both the link
  *score* and the *target operating point* and ships them in the feedback frame.
  The VTX applies them mechanically, overlaying only local overrides (thermal
  back-off, failsafe). One advisory bit is reserved for a future drone-decides
  mode. This mirrors `alink`'s "GS decides" stance.
- **Python policy + thin C++ control surface.** The whole control loop, FEC,
  protocol, and simulation are Python in `tools/precoder/`; the C++ duplex binary
  (`txdemo/stream_duplex_demo/main.cpp`) gains only a stdin control-opcode escape
  (`SET_PWR` / `SET_RATE` / `SET_CHAN`) so the policy can move the knobs with no
  USB churn and no restart.

### Module map

| Module | Role |
|---|---|
| `tools/precoder/energy_model.py` | `R_phy` rate tables, airtime, `E_bit`; the nominal `P_baseline` + `P_pa[0..63]` + TXAGC-gain calibration (`DEFAULT_CALIB`), overridable by a metered JSON |
| `tools/precoder/link_model.py` | `(MCS, SNR) → P_deliver` via the measured-channel model in `fec_ab_sim`; `snr_required` per (MCS, overhead, target) |
| `tools/precoder/op_table.py` | enumerates `(MCS, FEC-overhead, BW)` link rows, precomputes `snr_req`, and resolves each to the **minimum** TXAGC that clears it + its `e_bit` |
| `tools/precoder/controller.py` | the per-layer energy-min loop + `SvcController` bank; argmin-`e_bit` over SLA-feasible rows, asymmetric hysteresis, failsafe |
| `tools/precoder/rc_proto.py` | RCF / DISC / DISC_ACK codec (CRC16-guarded, drop-not-misapply) + the shared profile table |
| `tools/precoder/score.py` | sliding-window link score from `<devourer-stream>` lines, weighted by post-FEC residual loss (not raw FCS) |
| `tools/precoder/rendezvous.py` | VTX / VRX receiver-initiated discovery state machines |
| `tools/precoder/adaptive_link.py` | the `--role vtx\|vrx` orchestrator wiring it all to the duplex binary |
| `tools/precoder/svc_pipeline.py` | end-to-end SVC-HEVC UEP simulator + closed-loop adaptive variant |
| `tests/sim_loop.py` | the offline fly-out-and-back headline (energy saved vs static baselines) |

## The control algorithm

Each VRX feedback sample drives one update:

1. **Estimate path loss, not SNR.** TX power changes the *received* SNR, so EWMA-
   ing SNR across a power change corrupts the estimate (and delivery collapses at
   every transition). The controller instead EWMA-s **path loss =
   `reported_snr − gain_db(reported_txagc)`**, which is TXAGC-independent. The
   EWMA is **asymmetric** — it reacts fast when the link weakens
   (`ema_alpha_down = 0.8`, so power goes up in time) and slowly when it
   strengthens (`ema_alpha = 0.3`, so it doesn't chase noise upward).
2. **Score the link feasible rows.** For every `(MCS, FEC-overhead)` row whose
   `snr_req` is met by the estimate (minus a `margin_db = 2.0` entry hysteresis)
   *and* whose PHY rate carries the layer's `src_bitrate`, resolve the **minimum
   TXAGC** that supplies `snr_req` at the estimated path loss.
3. **Pick argmin `e_bit`.** Among feasible rows, choose the cheapest energy-per-
   delivered-bit — the inversion of `alink`'s argmax-throughput.
4. **Hysteresis, slow-up / fast-down.** A cheaper row is only adopted if it is
   ≥ `improve_frac = 3 %` cheaper and `min_between_changes_ms = 150` has elapsed;
   after a downgrade the controller holds for `hold_after_downgrade_ms = 4000`.
   A *failing* current row downgrades immediately. TXAGC (the cheap, non-
   disruptive lever) tracks the estimate freely between row changes.
5. **Failsafe.** No feedback for `feedback_timeout_ms = 1000` → the controller
   returns `MAX_RANGE` (most robust MCS, heaviest FEC, full power) and the VTX
   keeps transmitting; persistent loss escalates to rendezvous.

TXAGC is deliberately **not** a row dimension: it is chosen at runtime as the
minimum index that clears the chosen row, so the table stays small and power is
always the least that works.

### SVC unequal error protection

`SvcController` is a bank of the controllers above, one per temporal layer, with
per-layer targets and shed permission:

| Layer | Target | Shed? | PHY MCS (default ladder) | Outer-FEC overhead |
|---|---|---|---|---|
| critical (IDR / VPS/SPS/PPS) | 0.999 | never | MCS0 20 MHz LDPC STBC | 1.00 |
| T0 base | 0.99 | never | MCS1 20 MHz LDPC STBC | 0.75 |
| T1 | 0.95 | yes | MCS4 20 MHz | 0.50 |
| T2 | 0.90 | yes | MCS7 40 MHz SGI | 0.25 |

The PHY-MCS ladder is the C++ `svc::LayerPolicy` (`txdemo/svc_tx_demo/svc_tx.h`,
`DEVOURER_SVC_LADDER`); the FEC-overhead ladder is `svc_uep_fec.default_uep_policy`.
Both halves protect the same layers. One PA serves every layer, so the commanded
TX power is the **max** any active layer needs; a layer that no feasible row can
carry is shed (its airtime and energy are saved) rather than delivered badly.

`svc_pipeline.run_svc_pipeline_adaptive` runs this closed loop in software end to
end: a `SvcController` picks each layer's MCS, the shed set, and the shared power
from a reported sample; the pipeline transmits only the active layers at the
commanded MCS over the resulting effective SNR. The observed behaviour:

```
reported SNR @ txagc 32   active layers   shared txagc   per-layer NAL delivery
        +40 dB            [crit,T0,T1,T2]       0         all 1.000   (power backed off)
         +4 dB            [crit,T0]            52         crit/T0 1.000, T1/T2 shed
```

## Energy and link models — "model now, meter later"

The controller needs numbers it cannot yet measure on this bench, so both models
ship with a **documented nominal calibration** and a clean hook to anchor to
hardware later:

- **Energy** (`energy_model.DEFAULT_CALIB`): `P_baseline = 0.7 W` floor; a PA
  curve `P_pa[idx]` rising ~0.1 W → ~1.5 W across TXAGC 0..63 and compressing
  near the top; a concave TXAGC→gain curve to ~25 dB; `t_pre` preamble airtime.
  `tests/calibrate_energy.py` replaces these by fitting the duty-sweep + TXAGC-
  sweep from `tests/thermal_gain_sweep.py` (thermal `delta` = PA-dissipated
  proxy) and `tests/sdr_power_probe.py` (USRP radiated proxy).
- **Link** (`link_model`): a nominal per-MCS FCS waterfall + sub-block survivor
  shape, fed through `fec_ab_sim.sim_interframe` so delivery and airtime are
  priced with the *same* measured-channel accounting the fused-FEC work uses.
  `tests/calibrate_link.py` replaces the waterfall with histograms swept from
  `tests/sdr_interferer.py` + `fused_fec_link.FusedFecReceiver.report()`.

The *shape* — which operating point is cheapest — is correct from the nominal
model; only the absolute Joules and SNR thresholds move once metered. Relative
energy savings are valid without a DC meter; an absolute figure needs one, and
the JSON hook is where it lands.

## Feedback and rendezvous protocol

`rc_proto.py` defines three CRC16-guarded frames carried as sub-block bodies
([SBI framing](fused-fec.md), so they share the corrupt-frame-salvage path):

- **RCF** (VRX→VTX, ~100 ms): magic `"RC"`, flags (AUTH / FAILSAFE / DISCOVERY),
  VTX-ID, seq, ack-seq, profile index, an `alink`-style 1000–2000 score, explicit
  `pwr_idx` and `fec_overhead`, and per-layer delivery. A bad CRC drops the frame
  — it is never half-applied.
- **DISC / DISC_ACK** (rendezvous): VTX-ID, nonce, op-channel/width, profile-table
  version, init profile, capability bits.

**Receiver-initiated rendezvous** (the community pattern, formalized after RIT /
802.11ba wake-up-radio): when the VTX loses the RC uplink it enters a low-duty
discovery listen (~50 ms on / ~1 s period on a single 2.4 GHz discovery channel,
dodging the 5 GHz Vbus-sag gotcha); the wall-powered VRX beacons DISC carrying
that VTX's ID *fast*, so any listen window overlaps ≥ 2 beacons. The duty is
deliberately asymmetric — a cheap battery-powered listener and an expensive
mains-powered beaconer. DISC → DISC_ACK → both `SET_CHAN` to the op channel →
session. The watchdog input is kept abstract (`last_rc_monotonic`) so a real RC
uplink is a one-line wire-in; today `ADAPTIVE_RC_SILENCE_AFTER_MS` fires it
deterministically for tests.

## How it compares

The open long-range FPV field has converged on adaptive Wi-Fi video, but every
system optimizes for *quality, latency, or survival* — **none makes energy the
objective, and none does per-temporal-layer SVC unequal error protection.** Those
two are this design's distinguishing axes. The systems below are the closest
relatives; a feature matrix summarizing all of them follows.

### wifibroadcast / wfb-ng (the substrate)

The common ancestor and, for the OpenIPC world, the substrate: a **pure one-way
FEC broadcast** link — no ACKs, no ARQ, video sprayed as FEC-coded blocks with
RX-side diversity and distributed bonding (several ground receivers, the
best-signal one wins). By itself it is **static** — one MCS, one block-FEC `k/n`,
one power, chosen for the worst case and paid for the whole flight. In the
fly-out-and-back simulation a fixed *robust* profile costs **2.1× the energy per
delivered bit** of the adaptive loop. Everything adaptive in the OpenIPC ecosystem
(including `alink`) sits *on top of* this layer. See
[wfb-ng tuning](wfb-ng-tuning.md) for the static knobs.

### OpenIPC `alink`

`alink` is **not a separate radio stack — it is an adaptive-control sidecar that
rides on wfb-ng.** The ground station scores the link from per-packet RSSI/SNR
into a 1000–2000 quality number and selects a **TX profile**; each profile in
`txprofiles.conf` bundles a full operating point — **video bitrate + MCS + FEC
`k/n` + guard interval + GOP/keyframe + TX power + ROI-QP** (region-of-interest
quantization, a *spatial* quality bias within a frame) — and the air unit applies
the commanded profile, with hysteresis to avoid flapping. Its objective is the
**highest sustainable video quality**. It is mature, deployed, and the direct
inspiration for this design's *structure*: ground-authoritative scoring, ~100 ms
cadence, hysteresis, a max-range failsafe.

The difference is the **objective**, and it changes the reflexes:

| | OpenIPC `alink` | This link |
|---|---|---|
| Objective | **max quality** within link/power budget | **min energy/bit** subject to a quality floor |
| Operating point | a hand-authored **profile** (preset MCS+FEC+power+bitrate) per score band | rows resolved at runtime; **TXAGC chosen as the minimum that clears the MCS** |
| TX power | a per-profile preset | a continuous lever — the least power that works |
| When the link is strong | push bitrate up | **back power and FEC off** (idle the PA, save Joules) |
| Unequal protection | **ROI-QP** — spatial, within a frame | **per-temporal-layer** PHY-MCS ⊕ FEC ladder, enhancement **shed** |
| Energy model | not a first-class term | explicit `E_bit` (airtime × power), the thing minimized |

`alink` is the right tool when the mission wants the best picture the spectrum
allows; this link is the right tool when endurance, thermal headroom, or a small
RF footprint is the scarce resource and the picture only has to stay *good
enough*. They are duals — the operating-point machinery is nearly identical; the
cost function is inverted, and the UEP is temporal rather than spatial.

### RubyFPV

A **complete, self-contained air+ground FPV system** (its own raw-Wi-Fi protocol,
not wfb-ng) on RTL8812AU/8812EU radios. Two things set it apart from the wfb
lineage: it runs an **ARQ retransmission** layer (wifibroadcast deliberately does
not), and its adaptive loop is **predictive** — it synthesizes "Video Quality and
Prediction" (VQP) parameters from both-end statistics (missing data, RSSI,
error-correction used, retransmission requests, link latency) to describe link
quality *in the near future* and pre-empt breakups, not just react. It adapts in
graduated steps — **FEC rate + H.264 params + bitrate** first, escalating to
**lowering the radio data rate (MCS)** only when those are exhausted — plus an
**adaptive keyframe interval**.

Two structural contrasts with this link:

- **Authority is inverted.** RubyFPV is **vehicle-authoritative** — the air unit
  computes VQP and applies changes, using controller feedback as an *input*, and
  falls back to a vehicle-only algorithm when that feedback is lost (the common
  long-range case where the uplink dies first). This link is
  ground-authoritative, with a low-duty rendezvous to re-establish the session
  rather than a vehicle-only mode.
- **No energy objective and no per-layer UEP.** RubyFPV's goal is explicitly
  **robustness — it trades quality *and* latency for link survival**; FEC is
  adapted globally, not per temporal layer, and **TX power is not an adaptation
  target**. Its multi-band "parallel links" (433/868/915 MHz, 2.4, 5.8 GHz) and
  relaying are *redundancy/resilience* features, not energy or throughput
  optimization. Source is C/C++ under a custom non-OSI "Ruby Licence" (no
  military use).

### OpenHD

The other major open ecosystem, descended from the same befinitiv wifibroadcast
root as wfb-ng but an **independent fork** with its **own** C++ broadcast library
(`OpenHD/wifibroadcast`, GPLv3) and Realtek driver fork. Its design priority is
**latency** — FEC instead of ARQ, ~100 ms glass-to-glass, H.265 to save every
millisecond. Its adaptation is narrower than the others':

- **Bitrate is the only automatic, closed-loop knob** — variable bitrate is on by
  default and the encoder bitrate is **reduced on transmission errors**. (OpenHD's
  public docs do not specify the exact metric or whether ground-measured RSSI/SNR
  is relayed to the air unit to drive it, so the loop's authority is less defined
  than `alink`'s or this design's.)
- **MCS / channel width and TX power are operator knobs, not closed loops** — MCS
  is switched manually in flight via an RC channel (`MCS_VIA_RC`); TX-power index
  (0–63) is runtime-adjustable but manual; keyframe interval is manual. FEC is
  "optimized" but not documented as adaptively retuned.
- **No per-layer UEP/SVC and no energy objective.** It does provide ground **RX
  diversity** (up to two receivers, best-signal auto-select) and bidirectional
  MAVLink. GPLv3, C++.

So OpenHD adapts *one* lever automatically (bitrate) where `alink` and this link
adapt the whole operating point; and like every system here it treats neither
energy nor temporal-layer protection as a control variable.

### DJI OcuSync / O3 / O4

Proprietary and closed. OcuSync already does adaptive coding & modulation,
adaptive bitrate, and frequency agility, and is generally understood to bias
toward *latency and quality*. There is no public evidence of an energy objective
or of exposed per-layer protection. This design targets the same adaptivity in
the open stack and adds the explicit energy objective and per-temporal-layer UEP
that a closed system does not expose.

### Feature matrix

`A` = automatic / closed-loop, `M` = manual operator knob, `—` = absent or not
public. "Per-layer UEP" means temporal-layer (SVC) unequal protection on the FEC
*and* MCS knobs.

| Dimension | wfb-ng | OpenIPC alink | RubyFPV | OpenHD | DJI O3/O4 | **This link** |
|---|---|---|---|---|---|---|
| Relation to stack | broadcast link layer | **sidecar on wfb-ng** | own raw-Wi-Fi (+ARQ) | own wifibroadcast fork | proprietary | controller on devourer |
| Adaptation objective | none (static) | max quality | robustness (survive) | latency-first quality | latency / quality | **min energy/bit s.t. UEP SLA** |
| Auto video bitrate | — | A | A (fine steps) | A (default) | A | layer **shed** vs lowered |
| Auto MCS / data rate | — | A (per profile) | A (escalation) | M (RC switch) | A (ACM) | A (per layer) |
| Auto FEC overhead | — | A (per profile) | A (global) | — | A | **A (per-layer ladder)** |
| Auto TX power | — | preset per profile | — | M (runtime) | A (closed) | **A (continuous min-power)** |
| Feedback authority | n/a | **ground** (RSSI→score) | **vehicle** (VQP, predictive) | mgmt bidir. (metric undoc.) | proprietary | **ground** |
| ARQ / retransmit | — | — | **yes** | — (latency) | — | — |
| Per-layer (SVC) UEP | — | spatial ROI-QP only | — | — | — | **yes (PHY ⊕ FEC, shed)** |
| Corrupt-frame salvage | — | — | — | — | — | **yes (SBI sub-blocks)** |
| Energy-aware | — | — | — | — | — | **yes (the objective)** |
| License | GPL | GPL | custom (non-OSI) | GPLv3 | closed | (devourer) |

The two fully-populated rows unique to this link — **energy-aware** and
**per-layer UEP + corrupt-frame salvage** — are the contribution; everything else
in its column is shared with the mature systems it learned from.

### 802.11 rate adaptation (Minstrel-HT, SampleRate, RRAA)

The standard in-kernel rate adapters are **throughput-maximizers driven by
per-packet ACKs**. They do not apply here for two reasons: the link is
ACK-less broadcast injection (no MAC success signal to adapt on), and they are
oblivious to application FEC, video-layer importance, and energy. This controller
adapts on an explicit out-of-band score, not ACKs, and optimizes energy under a
UEP constraint — a different problem.

### Academic energy-aware rate adaptation (e.g. ERAA)

Energy-rate adaptation research establishes the core result this design rests on
— minimizing energy-per-bit means riding a high MCS (short airtime) and using
the least power that sustains it, with reported savings around ~44 % at ~90 % of
peak throughput. Those schemes are typically unicast/ACK-driven and not source-
aware. This link applies the same bits-per-Joule principle to an ACK-less
broadcast video downlink and couples it to cross-layer SVC UEP and a receiver-
initiated rendezvous for session establishment.

## Results

Software, the headline is `tests/sim_loop.py` — a time-varying "fly out and back"
path-loss schedule pushed through the controller + link model + energy model,
compared against two static baselines tuned on the same models:

| Strategy | Energy / delivered bit | Delivery | Notes |
|---|---|---|---|
| **Adaptive (this link)** | **205.7 nJ** | 0.999 | 2 operating-point changes over 200 ticks (no flapping) |
| Static energy-min profile | 310.6 nJ | 1.000 | best single fixed point — adaptive saves **34 %** |
| Static robust profile | 435.2 nJ | 1.000 | over-provisioned worst-case — adaptive saves **53 %** |

The SVC pipeline (`svc_pipeline.py`) shows the UEP staircase end to end against a
synthetic HEVC stream (`tests/gen_svc_nals.py`): as SNR drops, T2 sheds first,
then T1, then T0, while the critical/IDR layer holds at 1.000 delivery far below
where enhancement is gone — and SBI sub-block salvage delivers materially more
than whole-frame erasure on the marginal layer.

The whole `tools/precoder` suite — controller, protocol, rendezvous, SVC pipeline
— runs headlessly in CI (`.github/workflows/precoder-tests.yml`).

On hardware, `tests/adaptive_onair.sh` closes the loop over two adapters (8812
VTX ↔ 8821 VRX) with an optional B210 interferer: the VRX scores the link and
commands an operating point, the VTX applies it, and the *witness* is the peer's
own `<devourer-stream>` — `rate=` changes when a `SET_RATE` lands, `rssi=` rises
when a `SET_PWR` raises power, with no extra instrumentation. The base link
adapts MCS and power on air and rides the failsafe/rendezvous transitions.

## Current scope and integration points

- **Energy is modeled, not metered.** Relative savings are valid on the nominal
  calibration; an absolute Joule figure needs the DC-meter anchor that the
  calibration JSON hook accepts. Thermal `delta` and SDR `dbfs` cross-check the
  shape.
- **No real RC uplink in-repo.** RC-loss is driven through the abstract watchdog
  input; a real uplink wires into `last_rc_monotonic`.
- **On-air SVC today is non-adaptive.** `SvcTxDemo` already flies each HEVC
  temporal layer at its own MCS on air (`tests/svc_uep_onair.sh`); the
  *adaptive* SVC path — the VRX's `SvcController` retuning the per-layer ladder
  and shed set live via a per-frame layer-tagged radiotap in
  `stream_duplex_demo` — is the integration point between the software-validated
  loop and the on-air binary.

## References

- OpenIPC project — <https://github.com/OpenIPC>
- OpenIPC Adaptive-Link (`alink`) — <https://github.com/OpenIPC/adaptive-link>
- wfb-ng (wifibroadcast-NG), svpcom — <https://github.com/svpcom/wfb-ng>
- RubyFPV — <https://github.com/RubyFPV/RubyFPV>,
  [adaptive video link](https://rubyfpv.com/resource_adaptive_video_link.php)
- OpenHD — <https://github.com/openhd/openhd>, broadcast library
  <https://github.com/OpenHD/wifibroadcast>,
  [variable bitrate](https://openhdfpv.org/software-setup/variable-bitrate/)
- A. Abdel-Khalek and R. W. Heath, "Joint MCS and FEC for unequal error
  protection of scalable video," *IEEE JSAC*, 2012 — cross-layer UEP.
- "All Bits Are Not Equal: A Study of IEEE 802.11 Communication Bit Errors,"
  *IEEE INFOCOM*, 2009 — localized corruption, the basis for sub-block salvage.
- Energy-rate adaptation (ERAA and related) — energy-per-bit minimization,
  high-MCS / minimum-power result.
- IEEE 802.11ba (Wake-Up Radio) and Receiver-Initiated Transmission (RIT) — the
  asymmetric-duty rendezvous pattern.
- [Fused FEC](fused-fec.md) — the concatenated error-correction stack the UEP SLA
  is stated against.
- [wfb-ng tuning](wfb-ng-tuning.md) — the static-link baseline.
