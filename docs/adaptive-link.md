# Energy-minimizing adaptive video link

A long-range FPV link carries **video** one way — drone to ground, broadcast,
with no per-packet acknowledgement — but the link as a whole is **two-way**: an
RC uplink and a telemetry downlink already exist, and every adaptive system runs
its feedback over that existing return channel. So the radio can't learn
per-packet success the way an acknowledged Wi-Fi station does, yet the ground
*can* tell the drone how the link looks. An *adaptive* link uses that feedback to
change its transmit parameters as radio conditions change instead of fixing them
for the worst case. Several mature open systems already do this. This document
describes a design whose **objective differs from all of them** — it minimizes
the energy spent per delivered bit subject to a video-quality floor — and sets it
side by side with the rest of the field.

## The problem

A fixed link picks one operating point — one modulation, one transmit power, one
FEC strength — sized for the worst moment of the flight, and pays for it the
whole flight. That wastes resources in both directions:

- **Close to the operator** the link has 20–30 dB of margin it never uses. Full
  power into a strong link buys nothing but heat, battery drain, and a wider
  interference footprint; heavy FEC and a slow, robust modulation burn airtime
  that could have carried more video or simply been idle.
- **At long range** the fixed point either falls off a cliff (too aggressive) or
  was so conservative the whole flight paid the worst-case tax.

Adaptation closes that gap. The interesting question is *toward what* — because
the objective, not the mechanism, is what separates the systems below.

## Two questions that separate every system

**What does it change (the levers)?** Video bitrate, modulation and coding (the
MCS / data rate), forward-error-correction strength, transmit power,
channel/bandwidth, and keyframe cadence are the knobs in play. Few systems move
all of them; which ones they move automatically vs. leave to the operator is a
real differentiator.

**What does it optimize for (the objective)?** This is the axis that matters
most, because it dictates the reflexes when the link is strong:

- **maximum sustained quality** — push bitrate up whenever the link allows;
- **minimum latency** — never retransmit, keep the pipeline short;
- **maximum survival** — trade quality *and* latency to keep *a* picture alive;
- **minimum energy per delivered bit** — the objective of the design here.

These are not interchangeable. A quality-maximizer spends transmit power freely
to unlock a higher modulation; an energy-minimizer does the opposite. Both are
"adaptive," and they behave like opposites on a strong link.

## This design: energy-min under a quality floor

The objective is to minimize energy per *delivered* bit — total Joules (the
always-on baseline draw plus the amplifier) amortized over the video bits that
actually arrive — while holding a delivery floor on the important parts of the
picture. Two ideas make that tractable.

**Time-on-air is the dominant energy lever; transmit power is the dominant *link*
lever but a weak *energy* lever.** A radio's baseline draw (oscillator, baseband,
USB, receiver) is always on, so the power amplifier is only a fraction of the
total; pushing it hard costs far less energy than the airtime it saves. Concretely,
a several-fold increase in radiated power adds only tens of percent to energy per
bit, while a faster modulation cuts airtime — and therefore PA-on time — directly.

| Lever | Effect on the link | Effect on energy/bit |
|---|---|---|
| Modulation / FEC (time-on-air) | strong | **strong** — less airtime, fewer Joules/bit |
| Channel / bandwidth | strong | moderate |
| Transmit power | **strongest** | **weak** — baseline draw dominates |

So the energy-minimizing reflex is to ride the highest modulation the link will
bear (short airtime) and spend only the *minimum* power that clears it. When the
link is strong, it backs power and FEC off and lets the amplifier idle — exactly
where a quality-maximizer would instead spend the headroom on bitrate.

**The quality floor is per-layer, not a single number.** Scalable video splits
into a base layer (decodable alone, low frame rate) plus enhancement layers that
refine it. The base and key frames are small but indispensable; the enhancement
layers are large but optional. So protection is unequal by design:

- base / key frames are held to a near-perfect delivery target and never dropped;
- enhancement layers are best-effort — the slack the controller sheds first when
  the link can't carry them at any sensible energy.

Protection tracks importance on *both* knobs at once — the robust layers fly at
the most robust modulation *and* carry the heaviest FEC — which yields a graceful
staircase of degradation (enhancement fades, then thins, while the base holds)
instead of a single cliff.

**The ground station decides.** The receiver has the clean view of link quality,
so it scores the link and chooses the operating point, and the drone applies it
with only local safety overrides (thermal back-off, a max-range failsafe when
feedback is lost). This is the same "ground decides" stance the OpenIPC world
uses; the difference is purely the cost function it optimizes.

**Re-finding each other.** When the command uplink drops, the drone falls to a
robust failsafe and then to a low-duty listen on a known channel; the
mains-powered ground station beacons for it quickly. The duty cycle is
deliberately asymmetric — a cheap battery-powered listener, an expensive
always-on beaconer — so rendezvous is quick without costing the drone the energy
the whole design is trying to save.

## The landscape

### wifibroadcast / wfb-ng

The common ancestor for the OpenIPC world and the substrate the OpenIPC adaptive
layer rides on: a pure one-way FEC broadcast — no acknowledgements, no
retransmission, video sprayed as FEC-coded blocks, with receive-side diversity
across several ground antennas. On its own it is **static**: one modulation, one
FEC ratio, one power, chosen up front. Robust and simple, but it carries the
worst-case airtime and power for the entire flight. Everything adaptive in that
ecosystem sits on top of this layer.

### OpenIPC adaptive link (`alink`)

Not a separate radio stack but an **adaptive-control sidecar on wfb-ng**. The
ground station scores the link from signal strength into a single quality number
and selects a pre-authored **profile**; each profile bundles a complete operating
point — bitrate, modulation, FEC ratio, guard interval, keyframe cadence,
transmit power, and a region-of-interest quality bias within the frame. The
objective is the **highest sustainable video quality**, with hysteresis to avoid
flapping. It is mature and widely deployed, and its structure — ground scores,
profile applied on the air, max-range failsafe — is the reference shape for
OpenIPC adaptive links. Its one form of unequal protection is *spatial*
(sharper centre of frame), not per-temporal-layer.

### RubyFPV

A complete, self-contained air-and-ground system with **its own raw-Wi-Fi
protocol**, not wfb-ng. Two traits set it apart. First, it runs a
**retransmission (ARQ)** layer, which the wifibroadcast lineage deliberately
avoids. Second, its control loop is **predictive**: it fuses statistics from both
ends (missing packets, signal strength, FEC consumed, retransmission requests,
latency) into a forward-looking estimate of link quality and adjusts *before* the
picture breaks, rather than purely reacting. It adapts in graduated steps — FEC,
encoder parameters, and bitrate first, dropping the radio data rate only when
those are exhausted — and adjusts the keyframe interval. Its objective is
explicit **robustness**: it will trade both quality and latency to keep the link
alive. Notably, the **drone is authoritative** — it makes and applies the
decision, using ground feedback as an input and falling back to a drone-only mode
when that feedback is lost — the opposite of the ground-decides systems, and
**transmit power is not an adaptation target**. Its multi-band parallel links and relaying
are redundancy features, not energy or throughput optimization.

### OpenHD

The other large open ecosystem, an **independent fork** of the same
wifibroadcast root with its own broadcast library and driver. Its priority is
**latency** — FEC rather than retransmission, glass-to-glass around 100 ms, H.265
to save every millisecond. Its adaptation is narrower than the others': **video
bitrate is the one automatic, closed-loop knob** (reduced when the link shows
errors), while modulation, channel width, transmit power, and keyframe interval
are **operator controls** — adjustable in flight (some via an RC switch) but not
closed loops. It provides ground receive diversity and two-way telemetry, but
treats neither energy nor per-layer protection as a control variable.

### DJI OcuSync (O3 / O4)

Proprietary and closed, included as the commercial reference point. OcuSync does
adaptive coding and modulation, adaptive bitrate, and frequency agility, and is
understood to bias toward **latency and quality**. There is no public evidence of
an energy objective or of any exposed per-layer protection.

### 802.11 rate adaptation and academic energy-aware work

The standard in-kernel rate adapters (Minstrel-HT and kin) are
**throughput-maximizers driven by per-packet acknowledgements** — neither
assumption holds for an ACK-less video broadcast, and they are blind to
application FEC, video-layer importance, and energy. Separately, academic
energy-rate adaptation establishes the principle this design rests on:
minimizing energy per bit means riding a high modulation and using the least
power that sustains it. Those schemes are typically unicast/ACK-driven and not
source-aware; this design applies the same bits-per-Joule principle to a
broadcast video downlink and couples it to per-layer protection.

## Comparison at a glance

`A` = automatic / closed-loop · `M` = manual operator knob · `—` = absent or not
public. "Per-layer protection" means temporal-layer (scalable-video) unequal
protection on both the FEC and modulation knobs.

| | wfb-ng | OpenIPC alink | RubyFPV | OpenHD | DJI O3/O4 | **This design** |
|---|---|---|---|---|---|---|
| Stack | broadcast layer | sidecar on wfb-ng | own protocol (+ARQ) | own wifibroadcast fork | proprietary | sidecar on devourer |
| Objective | none (static) | max quality | survival | min latency | latency / quality | **min energy/bit** |
| Quality floor | — | one target | one target | one target | — | **per-layer (base ≥ 99 %)** |
| Auto bitrate | — | A | A | A | A | layer **shed** vs lowered |
| Auto modulation | — | A (profile) | A | M | A | A (per layer) |
| Auto FEC | — | A (profile) | A (global) | — | A | A (per-layer) |
| Auto TX power | — | profile preset | — | M | A | **A (min that works)** |
| Decides | n/a | ground | **drone** | mixed | proprietary | ground |
| Retransmit (ARQ) | — | — | yes | — | — | — |
| Per-layer protection | — | spatial only | — | — | — | **yes (modulation + FEC)** |
| Energy-aware | — | — | — | — | — | **yes (the objective)** |
| Open | yes | yes | source-available | yes | no | yes |

Read down the last column: almost everything in it is shared with the mature
systems this design learned from. The two rows that are unique to it —
**energy as the objective** and **per-temporal-layer unequal protection** — are
the contribution. It is closest in *shape* to OpenIPC's `alink` (ground decides,
profile applied on the air) but inverts the cost function from quality to energy;
it is the structural opposite of RubyFPV (drone-authoritative, survival-first,
retransmitting); and it adapts more of the operating point automatically than
OpenHD, which closes the loop on bitrate alone.

## Results

A time-varying "fly out and back" path-loss schedule, run in simulation against
two static baselines tuned on the same models:

| Strategy | Energy / delivered bit | Delivery |
|---|---|---|
| **Adaptive (this design)** | **206 nJ** | 0.999 |
| Best fixed energy-min profile | 311 nJ | 1.000 |
| Over-provisioned robust profile | 435 nJ | 1.000 |

The adaptive link saves **34 %** of the energy per delivered bit versus the best
single fixed point and **53 %** versus an over-provisioned worst-case profile,
while holding the delivery floor and without flapping between operating points.

Separately, pushing synthetic scalable-video through the link reproduces the
graceful staircase: as the link weakens the top enhancement layer sheds first,
then the next, while the base and key frames hold near-perfect delivery far
below the point where enhancement is gone.

## Scope

- **Energy is modeled, not metered.** The savings are *relative* figures on a
  documented nominal power model; an absolute Joule number needs a DC-meter
  anchor, for which the model leaves a hook. The *shape* — which operating point
  is cheapest — holds without it.
- **The command uplink is abstracted.** There is no real RC radio in the repo, so
  uplink loss is simulated to exercise the failsafe and rendezvous; a real uplink
  drops into the same watchdog input.
- **On-air scalable video is not yet adaptive.** Each temporal layer can already
  fly at its own modulation on air; the closed loop that retunes the per-layer
  ladder live is validated in software and is the next on-air integration step.

## References

- OpenIPC — <https://github.com/OpenIPC> ·
  Adaptive-Link — <https://github.com/OpenIPC/adaptive-link>
- wfb-ng (wifibroadcast-NG) — <https://github.com/svpcom/wfb-ng>
- RubyFPV — <https://github.com/RubyFPV/RubyFPV> ·
  [adaptive video link](https://rubyfpv.com/resource_adaptive_video_link.php)
- OpenHD — <https://github.com/openhd/openhd> ·
  [variable bitrate](https://openhdfpv.org/software-setup/variable-bitrate/)
- A. Abdel-Khalek and R. W. Heath, "Joint MCS and FEC for unequal error
  protection of scalable video," *IEEE JSAC*, 2012.
- "All Bits Are Not Equal: A Study of IEEE 802.11 Communication Bit Errors,"
  *IEEE INFOCOM*, 2009 — the basis for sub-block salvage.
- IEEE 802.11ba (Wake-Up Radio) / Receiver-Initiated Transmission — the
  asymmetric-duty rendezvous pattern.
- [Fused FEC](fused-fec.md) — the error-correction stack the quality floor is
  stated against · [wfb-ng tuning](wfb-ng-tuning.md) — the static baseline.
