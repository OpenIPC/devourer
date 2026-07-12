# Frequency-hopping spread spectrum on a userspace Wi-Fi driver

devourer is a userspace re-implementation of Realtek's RTL88xxAU driver: it
talks to the chip over libusb instead of a kernel module, and it is used by
OpenIPC for long-range digital video links. This article is about turning that
driver into a **frequency-hopping** radio — one that changes channel fast enough
to hop mid-stream, that can hop in a **keyed, unpredictable** order, that a
single second adapter can **follow in lockstep**, and that measurably survives a
jammer. It covers the design, the wire protocol, and every test we ran to
convince ourselves it works, on real hardware.

Two companion documents go deeper where this one summarises: the per-generation
retune mechanics are in [frequency-hopping.md](frequency-hopping.md), and the
jammer-resilience methodology is in [jammer-resilience.md](jammer-resilience.md).

## Why hop at all

A fixed-channel link has two problems in a contested or congested band. It is
**easy to interfere with** — anyone parked on your channel, deliberately or not,
takes you down — and it is **easy to find**: a static carrier is a beacon for
direction-finding. Frequency hopping addresses both. Spreading transmission
across N channels means a narrowband interferer only touches 1/N of your
traffic, which an error-correcting code can ride through; and a short dwell per
channel gives a watcher little to lock onto. If the hop *order* is also secret,
an adversary cannot even predict where to listen or jam next. That last property
is the difference between a hopping link that merely spreads interference and one
that resists a jammer actively trying to chase it.

The catch, on this hardware, is that a Wi-Fi NIC has exactly one local
oscillator per radio and speaks over USB. Every hop is a full RF retune pushed
across a USB control path. Doing that at FHSS rates — hundreds of hops a second —
without stalling the data stream is the engineering problem.

## The hop engine: a lean retune

`IRtlDevice::FastRetune(channel, cache_rf)` is the generation-agnostic entry
point. The default implementation is the full `SetMonitorChannel`; every chip
family overrides it with a stripped-down path. The vendor's channel-set routine
does a great deal that does not change between two channels of the same band and
width — RF calibration, AGC tables, band-edge filters — so the fast path skips
all of it, caches the RF register writes from the first tune, and on subsequent
hops replays only the two-or-so registers that actually encode the new
frequency. The full derivation of each trick is in
[frequency-hopping.md](frequency-hopping.md); the payoff is a per-hop cost that
turns a hop from a visible glitch into a rounding error:

| DUT | full path | fast (cached) |
|-----|-----------|---------------|
| RTL8812AU (Jaguar1) | ~277 ms | **~1.6 ms** |
| RTL8822BU (Jaguar2) | ~65 ms | **~2.5 ms** |
| RTL8821CU (Jaguar2) | ~30 ms | **~0.55 ms** |
| RTL8822CU (Jaguar3) | ~12 ms | **~1.9 ms** |
| RTL8812EU (Jaguar3) | ~12 ms | **~2.4 ms** |

The first hop after a full set primes the cache (~50 ms on Jaguar1); every
same-band hop after that is the number in the right-hand column. Hopping is
radiotap-driven — a frame can carry its own `CHANNEL` field — so it is per-packet
and needs no API change, exactly like rate or bandwidth.

## The schedule: sequential and keyed

A hopset is an ordered list of channels (`DEVOURER_HOP_CHANNELS`, e.g.
`36,40,44,48`). The transmitter advances through it one **slot** at a time.
`DEVOURER_HOP_SLOT_MS` makes the slot a fixed wall-clock interval; the slot
number is just elapsed-time / slot-length, so the schedule is a pure function of
time.

Two orderings share one code path (`src/HopSchedule.h`):

- **Sequential** — the public round-robin `channels[slot mod N]`. Predictable by
  anyone who knows the hopset.
- **Keyed** — set `DEVOURER_HOP_SEED` to a 128-bit key and each *round* (a full
  pass over the hopset) is a fresh Fisher-Yates permutation, its randomness drawn
  from **SipHash-2-4** keyed by the seed and salted by the round number. Every
  channel still occurs exactly once per round, but the order is a keyed
  pseudo-random permutation an adversary cannot predict without the key.

The important design property is that the keyed schedule is **stateless**:
`channel(slot) = permutation(slot / N)[slot mod N]`, computed from the key and
the absolute slot with no running RNG state. A receiver — or a re-joining one —
derives the same channel for any slot without replaying history. The
permutation uses rejection sampling for an unbiased shuffle, and a headless test
(`tests/hop_schedule_selftest.cpp`, wired into `ctest`) pins the SipHash output
against the reference vectors, checks that every round is a permutation, checks
key- and round-sensitivity, and checks the stateless-lookup identity.

## Lockstep receive: one adapter that follows

A hopping transmitter is invisible to a receiver parked on one channel — it only
hears the ~1/N of frames that happen to land there. The usual answers are N
receivers or a wideband SDR. devourer adds a third: a single adapter that
**hops in lockstep** with the transmitter.

The transmitter periodically emits a small **synchronization marker** — a private
vendor frame carrying the schedule fingerprint (which key/order), a per-process
epoch (to detect a transmitter restart), the current slot, and the in-slot phase
in microseconds. `txdemo` piggybacks it on its beacon; `streamtx` sends it on its
own frame every few data frames, so an application's FEC payload is never
touched. The marker is the shared clock: from `(slot, phase, arrival time)` the
receiver solves for the transmitter's slot-zero epoch and then, because the
schedule is stateless and it knows the key (or that the order is sequential),
computes which channel to be on for every future slot.

`rxdemo` runs a small state machine: **acquire** (scan the hopset until a marker
decodes), **track** (retune per slot on the recovered clock, low-pass-filtering
the phase estimate so it rides out jitter), and re-acquire after a few
marker-free slots. `hop.rx` events expose the state, per-hop retune time, and the
dead time from retune to first decode. Because the marker mechanism is identical
for keyed and sequential, both orders are trackable; the seed is optional.

## Validation

Two independent methods, because each has a blind spot.

### Two-adapter endurance

An RTL8812AU transmits a keyed hopping stream; an RTL8812CU follows it in
lockstep; both are pure userspace devourer. This measures the receiver's ability
to actually decode through the retunes over a long run. A representative session:
**598.3 seconds, 11 968 transmitted slots at 50 ms on channels 36/40/44/48**. The
receiver acquired once, tracked all 11 965 unique slots with no loss/reacquire
transition, and decoded within **11 891 of them (99.38 %)** — every one of the 74
misses an isolated single slot. Cross-channel RX retune ran 2.18 ms median /
2.65 ms p95; post-retune first decode 1.82 ms median. This is the ground truth
that lockstep tracking is real and robust, not a statistical artefact.

### Wideband SDR

The endurance test proves the *receiver* decodes, but a decoder is a poor witness
for the *transmitter*: a sensitive receiver can mask a weak or misordered hop. So
a USRP B210 captures all the hop channels at once and reconstructs the dominant
channel over time, checking it against the keyed schedule the key predicts
(`tests/hop_rx_probe.py`, driven by `tests/run_hop_validation.sh`).

Getting this right was itself instructive. The SDR analysis is a chain of places
to be wrong, and a false negative there looks exactly like a transmitter bug:

- The keyed-sequence matcher originally required an exact contiguous run anchored
  at the first observed dwell, with no tolerance for a dropped one. It is now a
  gap-tolerant aligner that slides over every offset and resyncs across dropped
  or spurious dwells, gated on a match fraction.
- The per-window "which channel is loudest" decision averaged power. The
  near-field transmitter is bursty, so averaging let a steady band-edge pedestal
  (edge channels sit ~9 dB higher on the B210) win every window and collapse the
  sequence. A high-percentile statistic tracks the bursty dwell instead.
- A bounded transmitter run finished during the B210's multi-second bring-up, so
  the capture saw only a sliver of hopping. The transmitter now hops
  continuously through the whole capture.
- The B210 presents an IQ-mirrored spectrum, handled as a channel involution.

With those fixed, the SDR confirms the keyed order end-to-end: **14 keyed rounds,
overflow-free, at 0.814 match fraction** — comfortably past the strict 10-round
threshold. The aligner ships an offline `--self-test` (no SDR needed) that
reproduces the original matcher failure as a regression guard.

## Jammer resilience

Hopping is only worth the complexity if it buys resilience, so we measured it
end-to-end over the fused-FEC link. Full methodology and knobs are in
[jammer-resilience.md](jammer-resilience.md); the results:

### A parked narrowband jammer

The B210 parks on one channel of the hopset; we measure FEC-decoded delivery
under four transmit strategies (channels 36/40/44/48, jammer on ch 40, 50 ms
slots):

| strategy | fec_delivery |
|----------|--------------|
| static, on a clean channel  | ~1.00 |
| static, on the jammed channel | ~0.00 |
| sequential hop | ~0.95 |
| keyed hop | ~0.95 |

The story is clear: a static link that lands on the jammed channel is fully
denied, while hopping bounds the damage to the 1/N of dwells that touch it and
the Reed-Solomon + sub-block code recovers that erasure fraction. **Against a
blind parked jammer, sequential and keyed are identical** (repeat runs 0.948 /
0.950 vs 0.947 / 0.950) — the jammer cannot exploit an order it does not
observe. Which is the whole point: the order only has to be secret against a
jammer that *reacts*.

### A following jammer

So we built one. `tests/sdr_follower_jammer.py` chases the hopping transmitter.
The B210 is a 2×2 (two receive and two transmit frontends off one AD9361), so it
**senses on RX while jamming on TX simultaneously** — no time-multiplexing. A
wideband RX burst (one FFT spans the hopset) finds the transmitter's current
channel; a CW tone on the TX frontend denies it, retuned when the target moves.
Two strategies, matched to the order it faces:

- **reactive** (against keyed): jam where the transmitter was just sensed. A hit
  needs the transmitter still there after the sense-and-retune latency.
- **predictive** (against sequential): jam the channel the public order says
  comes *next*, which cancels the follower's own latency.

Measured, the follower's reaction is dominated by the B210 transmit
retune at **~3.5 ms** (wideband sensing itself costs ~0.3 ms). That latency is
the floor on how fast such a reactive follower can move, and it **favours the
defender**. The two strategies' chase dynamics diverge sharply: over a 20-second
run against a 50 ms-slot transmitter, the **reactive** follower issues ~2100
retunes (constant correction — always a step behind an unpredictable hop) while
the **predictive** follower issues ~450 (it pre-positions and holds). That ≈5×
gap *is* the keyed schedule forcing the jammer into a latency-bound reactive
chase instead of a free ride.

The dwell threshold where following breaks is set by that reaction latency: once
the slot dwell falls below it (~3.5 ms here), a reactive jammer can no longer
land on a keyed hopper, while a predictive jammer against a sequential hopper is
limited only by its retune time and holds to much shorter dwells. The gap
between those two thresholds is exactly what a keyed permutation buys.

## Where it stands

What works today, on hardware: fast intra-band retune on all three chip
generations; sequential and keyed slot-hopping driven purely by radiotap and
environment; single-adapter lockstep receive validated to 99.38 % over ten
minutes and cross-checked by a wideband SDR; and a measured resilience story
against both a parked and a following jammer.

The honest edges: retune is intra-band (a cross-band hop correctly falls back to
the slow full set); per-hop TX power is not re-tuned, so a hopset spanning a wide
5 GHz range wants a periodic full set to refresh per-rate power; the follower
experiment uses a 3-channel hopset because a single B210 needs ≥60 MS/s to span
a 60 MHz hopset in one FFT and trips a UHD tuning assertion at the usual
61.44 MS/s. And the schedule visits every configured channel — there is no
adaptive exclusion of a persistently-jammed one; the fused-FEC layer absorbs
those dwells as erasures.
