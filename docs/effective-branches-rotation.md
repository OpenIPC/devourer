# Measuring effective diversity branches by rotation

A multi-chain adapter's spec sheet counts **RF chains** (2T2R, 4T4R). What a
diversity receiver actually gets is **effective branches** — the number of chains
whose fading is independent enough to add diversity. Those are not the same
number. Two adapters built on identical 4T4R silicon can deliver very different
effective diversity if one wires four decorrelated antennas and the other feeds
four chains from two antennas. This document is a lab procedure for finding the
truth experimentally, using nothing but a second adapter and your hands to rotate
the device under test.

It is written as an instruction: the concept first, then the method, then how to
run it, read it, and not fool yourself.

## Why chain count is not branch count

Diversity works because independent antennas see independent fades — when one is
in a null, another usually is not, so combining them (selection or
maximal-ratio) fills the nulls. The benefit collapses as the antennas' fading
becomes **correlated**: two co-located, same-polarisation antennas ride the same
fades and combining them buys almost nothing. So the quantity that matters is the
**envelope correlation** between chains, and from it the **effective branch
count**:

- **Envelope correlation** ρ — the correlation of the chains' signal *amplitude*
  (not dB) over a fading channel. ρ near 0 = independent; ρ near 1 = redundant.
  The classic rule of thumb is that diversity is still useful up to ρ ≈ 0.7.
- **Effective branches** N_eff — a single number summarising the whole
  correlation matrix as "how many independent branches is this really." It is the
  participation ratio of the correlation matrix: N chains that are mutually
  uncorrelated give N_eff = N; N chains that all ride the same fade give
  N_eff = 1. A 4-chain adapter whose chains pair up gives something in between.

The array/combining gain a real receiver extracts tracks N_eff, not the chain
count on the box.

## Why a static bench cannot measure it

Envelope correlation is only defined over a channel that **fades** — that varies
in time. A transmitter and receiver sitting still on a bench a few inches apart
present a *static* channel: the per-chain levels barely move (fractions of a dB),
so any correlation you compute is correlation of measurement noise, and it means
nothing. This is the first trap, and the measurement tool refuses to report a
verdict when it detects it (it calls the run inconclusive).

To measure decorrelation you must make the channel move. Three ways, cheapest
first:

1. **Rotation** — physically rotate and tilt the receiving adapter through many
   orientations during the capture. As it turns, each antenna's radiation pattern
   and polarisation sweep against the fixed transmitter, so every chain traces its
   own level curve. Antennas that are genuinely different trace *different* curves
   (low ρ); antennas that share a feed or point the same way trace the *same*
   curve (high ρ). Rotation is doable from one desk in a minute.
2. **Relocation into multipath** — put the transmitter in another room or on a
   balcony so the link becomes non-line-of-sight through walls and reflections.
   Ambient movement then fades it. This is the more deployment-representative
   channel.
3. **A mobile link** — the real thing: one end actually moving, as in flight.

Rotation measures **pattern/orientation decorrelation** under strong line of
sight. That is a *proxy* for the multipath envelope correlation a deployed link
sees — not identical to it, but it captures the property that dominates whether an
adapter's chains are redundant: do the antennas respond differently to the same
incoming wave. The *relative* comparison between two adapters on the same bench is
robust; the absolute ρ shifts in a real multipath field. Treat rotation as the
quick screen and relocation/mobility as the confirmation.

## The method

The receiver only needs a controlled, steady signal to measure against, and a way
to report each chain's level per frame:

1. **A controlled beacon transmitter.** A second adapter continuously injects a
   known frame (a fixed source address) on the test channel. Using our own beacon
   — rather than ambient traffic — means every measured frame comes from one
   steady source, so the only thing varying is the receiver's channel, which is
   exactly what you want to isolate.
2. **Per-chain metrics from the receiver.** The receiver reports each RF chain's
   received level (and SNR/EVM) for every beacon frame. Over the capture this
   builds a per-chain time series.
3. **Rotate the receiver** continuously through the whole capture window — full
   turns plus tilts and flips, not a gentle quarter-turn. Vigour matters (see
   pitfalls).
4. **Compute** the pairwise envelope correlation matrix and the effective branch
   count, plus the combining gain the chains would have delivered.

### Reading the numbers

- **Per-chain level spread.** First sanity check: did the channel actually move?
  If the strongest chain's spread is tiny, the rotation was insufficient or the
  link too static — the result is inconclusive, full stop. Rotate harder or add
  multipath.
- **Correlation matrix.** Look for structure. Chains that pair up at high |ρ| are
  sharing an antenna or pointing together. Both positive and negative correlation
  count as dependence (a chain that always goes *down* when another goes *up* is
  not independent).
- **Effective branches N_eff.** The headline number. Close to the chain count =
  the adapter delivers its full diversity order; well below it = some chains are
  redundant.
- **Combining gain.** Selection- and maximal-ratio-combining gain over the best
  single chain, reported at the median and at the low-outage tail (the 10 % and
  1 % worst moments). Diversity shows up most at the tail — a well-decorrelated
  multi-chain adapter buys many dB exactly when a single chain is deepest in a
  fade, which is the moment a video link drops.

## Running it

The receiver's per-chain emission is opt-in (it does not disturb the normal
output): the RX demo publishes an all-chains line per beacon frame when asked,
and the analyser turns a capture into the report above. A capture is only valid
if the operator rotates the adapter for its full duration.

- **One adapter, one shot.** `tests/run_antenna_decorrelation.sh` builds
  everything, brings up the beacon transmitter (and waits for it to actually
  start injecting before capturing — a dead beacon window is a common way to get
  an empty capture), then records a rotation window and prints the report. Its
  rotation mode announces a short countdown so you can start turning the adapter.
- **Comparing several identical adapters.** `tests/compare_8814_decorrelation.sh`
  finds every matching adapter and measures each in turn against one beacon, so
  the only variable between runs is the antenna front-end. When two adapters share
  the same USB identity (same vendor/product/**serial**), it selects between them
  by USB topology (bus/port) rather than identity.
- **The analysis alone.** `tests/antenna_decorrelation.py` runs on a saved
  capture, and has a hardware-independent self-test that validates the correlation
  estimator, the effective-branch metric, and the combining math against
  synthesised correlated fading — run that first if you change the tool.

The metric to capture is the received level; SNR and EVM are also emitted if you
prefer to correlate on those.

## Pitfalls (each one has bitten this measurement)

- **Gentle rotation lies.** A slow quarter-turn produces smooth, *correlated*
  level sweeps on all chains and reports high correlation — the opposite of the
  truth. Vigorous full-turn-plus-flip rotation explores enough orientations to
  reveal the real (low) correlation of good antennas. Only trust a capture whose
  per-chain spread is well above the tool's fading floor; if it is marginal,
  rotate harder and re-run.
- **A static capture is not a measurement.** If you forget to rotate, the tool
  will say inconclusive. Believe it.
- **A pinned/railed chain carries no information.** Under a strong static
  line-of-sight signal, a chain's reported level can sit on an automatic-gain rail
  (a constant value) and look dead — then vary normally once the adapter moves.
  This is exactly why rotation is required, not optional. The tool flags a
  near-constant chain so you don't mistake an AGC rail for a real reading.
- **The beacon must actually be on air.** Both ends take several seconds to
  initialise, and a flaky transmitter can fail to claim on the first try. Confirm
  the beacon is injecting before trusting a zero-frame capture as "no signal."
- **Identical adapters need topology selection.** Two dongles with the same
  USB identity down to the serial can only be told apart by which bus/port they
  are on — select the intended one explicitly, or you will measure the same device
  twice.
- **Rotation ≠ multipath.** The rotation result is orientation/pattern
  decorrelation under line of sight. It is the right quick screen and the right
  *relative* comparator, but confirm an important conclusion with a
  non-line-of-sight or mobile capture before treating an absolute correlation
  figure as the deployed one.

## Worked example: two 4T4R dongles, identical silicon

Two USB adapters built on the same 4T4R chip, differing only in antenna
front-end, measured on one bench against one beacon on the 2.4 GHz test channel,
each rotated vigorously through a capture:

| Adapter | Antennas | Worst \|ρ\| | Effective branches N_eff | MRC gain at 1 % outage | Verdict |
|---|---|---|---|---|---|
| 4-internal-antenna puck | 4 on-PCB internal | 0.19 | **3.8** of 4 | ~14 dB | well decorrelated |
| 2-external-dipole stick | 2 external dipoles | 0.64 | **2.6** of 4 | ~11 dB | partially correlated |

The stick drives four RF chains from only two physical antennas, so two of its
chains are strongly coupled and its effective branch count sits near its antenna
count, not its chain count. The puck's four genuine antennas decorrelate on every
pair and it delivers close to its full four-branch diversity — and materially more
combining gain at the low-outage tail, which is where a long-range link lives.
Same silicon, different truth. That difference is invisible on the spec sheet and
obvious in five minutes of rotation.

## Toward the gold standard

Rotation answers "are these chains independent enough to matter" cheaply and
comparatively. To turn a relative screen into a deployment number, repeat the
capture with the transmitter relocated into genuine non-line-of-sight multipath
(another room, a balcony, across a floor) and, ultimately, with one end in
motion. The tooling is identical — only the stimulus changes; the beacon can run
on a second host anywhere the receiver can still hear it. Expect the *ordering*
between adapters to hold and the absolute correlation to rise toward the values a
flying link actually experiences.
