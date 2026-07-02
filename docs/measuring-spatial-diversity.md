# Measuring spatial diversity on a multi-chain adapter

A multi-chain adapter's spec sheet counts **RF chains** (2T2R, 4T4R). What a
diversity receiver actually gets is **effective branches** — the number of chains
whose fading is independent enough to add diversity — and, ultimately, the
**combining gain** those branches deliver. Those are not the same as the chain
count. Two adapters built on identical 4T4R silicon can behave very differently if
one wires four decorrelated antennas and the other feeds four chains from two.
This document is a lab procedure for finding the truth experimentally, using
nothing but a second adapter and your hands — with three escalating stimuli
(**rotate** the device in place, **relocate** the transmitter into multipath, and
**move** the receiver), because how much diversity you can measure depends
entirely on how much you make the channel vary.

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
incoming wave. Treat rotation as the quick screen for *potential*; relocation and
motion are where you learn what a deployment actually gets — and, as the closing
sections show, that can shift or even invert the rotation ranking.

## The method — correlation screen (rotation)

This section covers the quick screen: measure the antennas' *correlation* and
effective-branch count by rotating the receiver. It answers "could these chains
help?" The two sections after it turn that potential into a realised
combining-gain number ("do they help?") — first with path-masking, then under
motion. All three share the same beacon and per-chain readout; only the stimulus
and what you compute change.

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
if the operator keeps the adapter moving for its full duration — rotating for the
correlation screen, translating for the motion measurement.

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
- **Combining gain and motion.** The path-masking sweep restricts the active
  chains via `DEVOURER_RX_PATHS=0xNN` (see the next section); the motion
  measurement toggles that mask (`DEVOURER_RX_PATHS=0x22:0xFF@300`) while you move
  the receiver, and `tests/mrc_mobility.py` reports the per-mask delivery and the
  shifting-best-chain trace.

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

| Adapter | Antennas | Worst \|ρ\| | Effective branches N_eff | Verdict |
|---|---|---|---|---|
| 4-internal-antenna puck | 4 on-PCB internal | 0.19 | **3.8** of 4 | well decorrelated |
| 2-external-dipole stick | 2 external dipoles | 0.64 | **2.6** of 4 | partially correlated |

The stick drives four RF chains from only two physical antennas, so two of its
chains are strongly coupled and its effective branch count sits near its antenna
count, not its chain count. The puck's four genuine antennas decorrelate on every
pair, so on this rotation screen it shows the higher *potential* — N_eff 3.8 vs
2.6.

But note the word *potential*: N_eff from rotation says how decorrelated the
antennas **could** be if you exercise every orientation. Whether that potential
turns into realised **combining gain** is a separate question — the one the next
two sections answer — and the answer can **invert this ranking.** The puck's four
antennas are packed close together and see nearly the same channel at rest, so
their potential only pays off under motion; the stick's two widely-spaced dipoles
decorrelate even stationary. Same silicon, different truth — and "more antennas"
is not the same as "more diversity."

## From branch count to combining gain

Effective branches tell you how many chains *could* help; the payoff is the
**combining gain** the receiver actually extracts from them. A multi-antenna
802.11 chip combines its RX chains in hardware and hands up one decoded frame —
there is no per-chain baseband to combine in software — so the gain is measured
by *starving* the chip of chains and watching what it loses:

- Restrict the chip to a subset of its RX chains (one, then two, then all) and
  compare frame delivery and post-combine link quality at each step. On this
  driver the active-chain set is a mask on the RX-path-enable register; masking a
  path drops its per-chain level to the noise floor, confirming the chip is
  genuinely down a chain (not just hiding a readout).
- The catch: at a **strong** signal a single chain already decodes everything, so
  every subset looks identical and you learn nothing. The gain only appears at a
  **marginal** link — deep enough that a single chain drops frames the full set
  still recovers. So this measurement, like the correlation one, requires a weak
  or fading channel (attenuation, distance, or the relocation below), not a
  bench with the transmitter inches away.
- Read the result as a curve: frames delivered (or effective SNR) versus number
  of active chains. Its slope is the realised diversity/array gain, and it should
  flatten once the added chains are too correlated to help — tying directly back
  to the effective-branch count. An adapter with N_eff ≈ 2 will stop improving
  past two chains even if it has four.

Combining gain and effective branches are two views of the same property:
N_eff says how many independent chains exist; the path-masking curve says how
much link margin they actually buy.

## Diversity under motion — where it actually pays

Everything above measures *potential* — how decorrelated the antennas could be.
The decisive question for a real link is whether combining delivers a gain **while
the receiver moves**, because a deployed link (a flying drone, a walking operator)
is never static. This is a different and more revealing measurement than the
static combining sweep, and it needs the comparison to be motion-fair.

**The trap.** Comparing a single-chain capture against an all-chains capture
*sequentially* is invalid under motion: the receiver is at a different point in
its fade during each, so any difference is motion, not combining. The comparison
must sample the *same* fading process. The fix is to **alternate the active-chain
set fast relative to the motion** — toggle a fixed single chain against all chains
every few hundred milliseconds while the operator moves the receiver continuously,
and tag every frame with the set that was active when it arrived. Because the
toggle is fast and the motion slow, both configurations see the same fades.

**The stimulus.** Translate the receiver through space — slide it back and forth
through several wavelengths — rather than rotating it in place. Translation sweeps
the antennas through the multipath standing-wave field, which is what makes each
chain's fading rise and fall and the best chain change; rotation only re-points a
fixed pattern.

**What to read.** Per toggle window (equal duration, so directly comparable):
frames delivered, and especially the *worst* windows. The signatures of real
diversity under motion are (1) the **best chain keeps changing** — no single
antenna wins most windows, where a static position had one clear winner; (2) a
*fixed* single antenna hits **deep-fade windows** (dropouts), which combining
**fills**; and (3) combining **halves the delivery variance**. The headline is not
the modest mean gain — it is the **elimination of the dropouts** that a
single-antenna video link would show as frozen frames.

**Always run a static control** at the same spot with motion off. The result that
matters is the *difference* between moving and static: a combining gain (and a
shifting best chain) that appear only when moving are the confirmation that the
value is diversity, not link margin.

### What this reveals about antenna design

Run across adapters, the moving-vs-static contrast exposes a design truth the
static numbers hide: **physical antenna spacing decides whether diversity helps at
rest.** Widely-separated antennas sit at different points of the standing-wave
field and so decorrelate *even stationary* — they show a real combining gain
static and only a little more moving. Antennas packed close together (a compact
internal array) see nearly the same channel at rest — one dominates, combining is
worthless — and only decorrelate once the array moves. Both converge to a similar
gain under motion, and both eliminate the fixed-antenna dropouts. So a *stationary*
ground station wants widely-spaced external antennas; a compact internal array
earns its extra chains only on a *moving* platform. Note this can invert the
static rotation ranking — rotation measures orientation potential, but real fading
decorrelation rewards spacing, so fewer well-separated antennas can beat more
closely-packed ones.

The tooling is the same throughout — only the stimulus (rotate, relocate, move)
and the mask schedule (fixed vs toggled) change; the beacon can run on a second
host anywhere the receiver still hears it.
