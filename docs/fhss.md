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

## Adaptive hopset: authenticated state and commit protocol

The fixed schedule visits every configured channel; a persistently-jammed one
costs 1/N of every FEC block forever. The adaptive layer (`src/hopset/`) is the
deterministic state model and authenticated control protocol that lets both
ends *exclude* channels — it deliberately does not decide which channel is bad
(that observation policy is a separate stage; today the lever is a scripted
commit).

### State model

The configured base hopset is **immutable**; every adaptive state names its
active channels by stable base-hopset bit positions:

```
HopsetState { epoch, generation, active_mask, activate_round, activate_slot }
```

Generation 0 is defined as the legacy fixed schedule (full mask, raw master
key) — byte-identical to non-adaptive operation, pinned by KAT. From
generation 1 on, each round's permutation derives from `(schedule subkey,
generation, round, active_mask)`: SipHash words tagged `'A'` drive the same
rejection-sampled Fisher–Yates over the popcount-of-mask active positions,
mapped back through the mask to base indices. Including the generation in the
word means a mask change produces a *fresh* keyed order — not a filtered form
of the old permutation a follower-jammer could have partially learned. The
lookup stays a pure function of (keys, committed state, absolute slot), so a
receiver still joins mid-generation with no RNG state
(`hopset/HopsetState.h`, `AdaptiveScheduleView`).

Two keys derive from `DEVOURER_HOP_SEED` with explicit domain separation
(`devourer-hopset-s*/c*`): the schedule subkey orders the hops, the control
key MACs the protocol. A schedule fingerprint is **not** authentication; the
`base_fp`/`mask_fp` fields are config-mismatch tripwires, the SipHash-2-4 MAC
is the auth. Sequential (keyless) mode therefore cannot be adaptive — the
demos reject the combination.

### Protocol

Three versioned, MAC'd messages (`hopset/HopsetWire.h`, byte layouts pinned in
`hopset_wire_kat`): **HopsetProposal** (RX → TX: proposed mask, observation
count, reason bitmap, nonce), **HopsetCommit** (TX → RX, broadcast: the new
generation/mask and the activation instant as both a round index and an
absolute slot), **HopsetStatus** (the authority's periodic beacon: current
generation/mask/round + activation anchor; doubles as the proposal-rejection
answer via a nonce echo).

The TX is the schedule authority (`HopsetAuthority`): a receiver proposes,
the transmitter validates policy — mask legality against the base, a
configurable minimum-diversity floor, monotonic generations, an activation
lead of at least 8 rounds — then repeatedly broadcasts an identical commit
until the named slot arrives, and both sides swap state exactly there
(`HopsetFollower`). Rejections carry a reason code; every transition emits a
`hopset.*` JSONL event (propose / commit / activate / reject / gen_mismatch /
recover).

Anti-replay is structural, the chanmig recipe: epochs are random per process
start and never persisted (a restart orphans every in-flight generation for
free), generations are monotonic within an authority epoch with a ceiling
whose recovery is a fresh epoch, proposals are remembered by (epoch, nonce),
and a commit's absolute activation slot must sit inside the follower's window.

### Recovery

Acquisition always scans the immutable base hopset, so a receiver that missed
a transition can always hear frames on the channels the reduced set still
uses. In adaptive mode the sync marker is v2 — the v1 layout plus the live
(generation, mask fingerprint), each version rejecting the other — so one
decoded marker exposes a missed transition; the follower drops to the acquire
scan and re-synchronizes from the authority's repeated commit or its status
beacon, which carry the full mask and activation anchor. The
missed-every-commit path, the synchronized activation, and the failure matrix
(lost / duplicated / reordered / forged / replayed frames, both restarts,
rollover, invalid masks, window violations) are ctest gates:
`hopset_schedule_math`, `hopset_wire_kat`, `hopset_proto_matrix`,
`hopset_activation_sync`.

### Deciding: receiver-driven exclusion

The receiver decides which channel to drop, because its delivery measurement
describes the endpoint that actually has to decode — a transmitter's own view
of the channel is not evidence that the video arrived. Each closed dwell is
scored (did the marker arrive, how many frames, how many failed FCS, RSSI/SNR/
EVM, retune-to-first-decode latency) and folded into a per-channel window
(`src/hopset/HopsetPolicy.h`). Delivery is authoritative; the energy metrics
and link verdicts only *classify* an impairment — interference, weak signal,
lost synchronization — into explanation bits. A quiet channel with a dead link
and a noisy channel that still delivers are opposite decisions, so a channel is
never excluded on energy alone.

The shipped policy is deliberately timid, because an adaptive exclusion loop is
an attack surface: anyone who can make channels look bad can otherwise herd the
link onto one channel and then jam it. So: at least 8 scored rounds before any
decision; exactly one channel per update; exclusion only after delivery stays
under 30 % for 5 consecutive visits *while another channel is still healthy*;
a hard floor of `max(3, configured)` active channels; at least 10 rounds
between updates (a refused proposal spends that budget too, so a receiver whose
proposals keep bouncing cannot flood the control channel); nothing excluded
when most of the band degrades together; and a cap on the excluded fraction.
The transmitter enforces its own copy of the shape limits — one channel per
update, the update gap, the diversity floor — because the receiver decides
*which* channel, never *how much* of the schedule may move at once.

Restoration is the other half: an excluded channel cannot prove it recovered
unless something goes back and looks. Every `P` rounds one data opportunity is
replaced by a **keyed recovery probe** on an excluded channel — which round of
the window, which position in it, and which excluded channel are all keyed
draws, so recovery does not create a public periodic target for an adversary to
sit on. Only sync/control rides a probe dwell, never caller FEC payload. Three
consecutive probes above 70 % delivery restore the channel.

### The transmitter's own view, and who wins when they disagree

The receiver decides because it measures the endpoint that must decode. But
when the return channel dies, nothing decides at all — so the transmitter also
senses, both as a supplement and as a failsafe. It never gets its own schedule:
every autonomous change travels through the same authenticated commit as a
receiver-driven one.

A radio cannot transmit and listen on one channel at once, so the only honest
local observation is a window in which it deliberately does not send. Each
armed dwell runs retune, settle, a discard-barrier read that resets the
counters, a quiet window, then the measurement read — and only then the
payload. An optional second window near the end of the dwell listens on the
channel just hammered, which is the only way to see an emitter that keys up
*because* of us rather than before us. Windows are measured, never assumed: the
counters keep counting through the read's own bus round trips.

Three situations make the transmitter refuse to sense rather than sense badly.
It will not arm alongside the auxiliary flood threads, which send from their
own threads and return on submit rather than on air, so a "gated" window would
still contain unknown self-transmission and read as interference. It will not
arm when the window would fall below about a millisecond and a half, where the
delta is mostly counter quantisation. And it skips any dwell with a commit in
flight, because a commit landing inside our own quiet window is self-jamming
that would then feed the veto.

**Whose measurement wins.** The tempting design is to let the transmitter veto
a proposal it disagrees with. That is wrong, and wrong in exactly the case this
exists to serve: a hidden interferer beside the receiver produces precisely
that signature — delivery collapses while the transmitter, elsewhere, hears
nothing — so the veto would fire most confidently when the transmitter's
reading is cleanest. Its confidence is anti-correlated with its correctness.

So the transmitter has no standing to contradict the receiver *about the
target*. It may answer one question only: does this move leave the transmitter
worse off, where it sits? Two grounds qualify. A uniformly dirty band means it
cannot attribute anything to a channel, so it asks for another look — a delay,
not a refusal, and one that does not spend the receiver's update budget. Every
surviving channel being markedly worse than the target means the move is a
cul-de-sac, which is a fact only the transmitter possesses. Both are
unanimity-gated, and an unmeasured escape route is not a road it may declare
closed. Disagreement itself is surfaced on every decision and never averaged
in: the applied mask is always one endpoint's proposal verbatim.

Four fusion policies select how much say it has — `rx` (observes only),
`veto` (the default), `either`, and `failsafe`, where the transmitter may
originate only after the return channel has been silent for a configured
number of rounds. Because a receiver with nothing to propose is otherwise
indistinguishable from a deaf one, a receiver running the exclusion policy also
sends a periodic status of its own; that status carries its committed
generation and mask, so a split brain can be seen rather than inferred.

Autonomous changes go through their own authority entry point that applies the
same one-channel-per-update and update-gap limits a follower proposal gets.
The operator's scripted lever stays exempt — but a machine reacting to its own
local evidence is precisely the actor those limits exist to bound.

### Measuring it in payload, not in markers

The receiver's policy scores dwells: a dwell counts as delivered when a sync
marker arrived before the next retune. That is the right quantity for the
policy — it is what the policy acts on — but it is the wrong one for judging
whether adaptation helps, and the two disagree by a lot. A marker is one small
frame at a robust rate, so a channel can decode markers while delivering
payload badly; and a dwell that decoded one marker out of twenty frames scores
1.0.

So the jammer harness runs the real thing: RS-coded, sub-block-framed bodies
from `tools/precoder` fed to the transmitter, decoded by the same salvage path
the video link uses, reported next to the proxy. Measured on the 4-channel
2.4 GHz base with a narrowband interferer parked on one member (25 s fixed,
70 s adaptive, both windows opened only once the receiver was tracking):

| measure | fixed keyed hopset | with the jammed channel excluded |
|---|---|---|
| marker proxy (dwells) | 0.762 | 0.946 |
| FEC delivery (packets) | 0.861 | 0.927 |
| recovered packets/s | 1001 | 986 |

A repeat run of the same cells read 0.760 → 0.944 and 0.872 → 0.922, so the
spread is under a point and the gap between the two measures is not noise: the
proxy claims roughly three times the improvement the payload sees, and the
delivered *rate* does not improve at all. Both facts have the same cause: on a
CSMA link the transmitter defers to the interferer rather than transmitting
into it, so a fixed hopset spends fewer frames on the jammed channel than its
dwell share suggests, and the outer code already absorbs what it does spend.
What exclusion buys here is margin — the erasure burst stops arriving at the
decoder — not throughput. On a link with carrier sense disabled, where the
transmitter really does spend a quarter of its frames into the jammer, the
ratio and the rate would move together.

Two failure modes only the payload measurement can see, both of which cost a
bench session to find. A receiver that loses slot lock keeps scoring ~1.0 on
the dwells it is present for, so the proxy reports a healthy link through an
outage — the FEC figure and the fraction of the window actually tracked are
what expose it. And a measurement window that opens at process start is mostly
chip bring-up and acquisition: one 70 s phase read 0.30 FEC delivery whose
settled part ran at 0.90.

Two protocol details follow directly from that instrument, and both matter on
any link running the salvage path:

- The sync marker is a plain vendor IE whose only integrity is the frame FCS,
  so a frame the chip flagged must never reach it. With corrupt frames surfaced
  — which sub-block salvage requires — a corrupted marker whose seed
  fingerprint happened to survive reads as a genuine generation change, and the
  follower drops lockstep on a frame it should have discarded.
- Both endpoints swap generation at the same absolute slot, but a marker
  stamped just before that boundary is decoded just after it. It carries the
  old generation truthfully, so the follower judges staleness by the slot the
  marker was *stamped* in, not by when it arrived. Treating one such frame as
  disagreement costs a full re-acquisition at the exact instant an exclusion
  takes effect — measured at 46 s, because a scanning follower coincides with
  the authority's status beacon only occasionally.

### What the sensor can and cannot see

The occupancy scale is measured, not assumed. On this bench a narrowband
interferer that destroys delivery on one channel reads about 2.8 CCA and 3.0
false alarms per millisecond at the transmitter, against roughly 0.6 on a clean
channel — a five-to-one separation. False alarms are weighted above CCA because
they separate better and CCA also counts legitimate traffic. A railed reading
contributes nothing: the gain index at its floor is not a measurement, and
because the power histogram's buckets are placed relative to that index, a
floored index rails the histogram at fully-busy on every channel as a
consequence rather than as an observation.

Two limits are worth stating plainly. The instrument is the front end, so a
part is only useful on the band it was built for — a 5 GHz-only PA part shows
no jammed-versus-clean separation at 2.4 GHz and cannot get a proposal out
there either. And the pre-versus-post distinction only pays against an emitter
that *locks on* after detecting a burst; measured against a follower sweeping
every few milliseconds — far shorter than a dwell — both windows read alike,
which is a fact about that adversary rather than about the sensor.

Sensing costs what it stops sending. Measured with a 25 ms window on one dwell
in two of a 200 ms slot: 7.7 % fewer frames per second, against 8.8 % predicted
by summing the windows' own stamps. The two agreeing is the check that nothing
unrecorded is being paid. Re-measured with both windows armed on the parked
bench: 393.8 → 363.8 frames/s, 7.6 % measured against 8.1 % accounted.

In payload terms that cost is a *rate*, not a ratio. Across the adaptation the
sensing run aired 367 bodies/s against 399 without sensing — the same ~8 % —
while FEC delivery was 0.937 against 0.930, i.e. unchanged. Sensing buys the
veto and the failsafe with throughput, and takes nothing from the delivery of
what it does send.

### Who decides, measured four ways

The same parked interferer under each fusion policy, one exclusion available,
60 s per row and 180 s for the failsafe (it may not act until the feedback
timeout has expired *and* its own observation window has filled — a shorter row
measures the un-adapted link under the failsafe's name). Post-burst sensing is
armed on every row so the throughput column compares policies rather than
windows:

| fusion | marker | FEC delivery | frames/s | exclusions |
|---|---|---|---|---|
| `rx` | 0.912 | 0.940 | 451 | 1 |
| `veto` | 0.921 | 0.932 | 457 | 1 |
| `either` | 0.922 | 0.932 | 471 | 1 |
| `failsafe` (receiver muted) | 0.908 | 0.932 | 453 | 1 |

The four agree within the run-to-run spread, and that is the result: when the
receiver is right, the fusion mode decides *who may act*, not what happens.
The failsafe reaches the same place with no uplink at all, only later.

### What the receiver's own uplink costs it

The receiver is half duplex, so every frame it airs is a slot it cannot hear —
and a heartbeat that transmits in the slots it needs for listening is a bug
this feature has had before. Three cells with no interferer at all, differing
only in whether the receiver talks: policy off, policy on with the uplink muted
(it decides and airs nothing), policy on and live. Muted-versus-live is the
comparison that isolates the transport, because both cells make the same
decisions.

Measured over 60 s cells: 19 control frames per minute — one status per 64
slots — costing under 5 ms of air per minute, and FEC delivery of 0.910 muted
against 0.943 live, i.e. no measurable cost at all against a cell-to-cell
spread of ±3.6 points. All three cells held lockstep for the whole window. The
cadence is therefore not the limiting factor and does not need tightening; what
does cost is losing lock, which is worth tens of seconds and is why the two
marker rules above exist.

### Driving it

`DEVOURER_HOP_ADAPTIVE=1` on `txdemo`/`rxdemo` (with a keyed seed and
`DEVOURER_HOP_SLOT_MS`) arms authority and follower;
`DEVOURER_HOP_POLICY=1` adds the receiver-driven policy (and
`DEVOURER_HOP_POLICY_EVENTS=2` its per-channel evidence);
`DEVOURER_HOP_PROBE_ROUNDS` is the probe period (default 8, 0 = off) and must
match on both ends, like the seed. `DEVOURER_HOP_ADAPTIVE_SCRIPT="slot:mask,..."`
is the operator's own lever for scripted authority-originated commits, exempt
from the one-channel-per-update limit; `DEVOURER_HOP_MIN_ACTIVE` lowers the
authority's floor for protocol tests on a small base hopset (the policy's
`max(3, ...)` is not lowerable). The proposal travels on the receiver's own
claimed handle — one bring-up, RX loop plus an occasional control frame — and
the transmitter listens for it under `DEVOURER_TX_WITH_RX=thread`. `streamtx`
emits the v2 marker at generation 0 so an adaptive receiver tracks it. Control
frames ride their own small 802.11 frames — caller FEC payload bytes are never
touched.

Transmitter sensing is `DEVOURER_TX_SENSE=1` on top of adaptive mode, with
`_WINDOW_US` / `_POSTBURST_US` / `_SETTLE_US` / `_EVERY` / `_NHM` shaping the
windows and `DEVOURER_HOP_FUSION=rx|veto|either|failsafe` choosing how much say
it has; `DEVOURER_HOP_MUTE=1` on the receiver is the fault-injection lever that
produces a genuine one-way outage. All of it is inert unset, so the
receiver-driven behaviour is exactly what it was.

Note that the classifier will not propose anything of its own without
post-burst evidence, so any run in which the transmitter must originate — the
failsafe, or any test of its opinion — has to arm `_POSTBURST_US`. Without it
the transmitter observes and holds, which looks identical to a transmitter that
is not sensing at all unless the hold reason is read.

`DEVOURER_TX_SENSE_INJECT="<idx>:<occ>[,...]"` (with an optional `*:<occ>`
default) replaces the transmitter's per-channel view with fabricated occupancy,
bypassing the hardware read but keeping the windows' timing so the duty cycle
is unchanged. It is a test lever in the same spirit as
`DEVOURER_HOP_ADAPTIVE_SCRIPT` and `DEVOURER_HOP_MUTE`, and it exists because
the two endpoint-disagreement cases cannot otherwise be built: on a bench the
adapters sit inches apart, and any interferer strong enough to degrade one is
equally audible at the other. Injected evidence earns no more authority than
measured evidence — the floors, the hysteresis and the cleaner-alternative rule
all still apply to it.

`DEVOURER_TX_STDIN=1` makes `txdemo` carry caller payload from stdin in the
`<u32_le len><PSDU>` framing `streamtx` reads, which is what lets FEC delivery
be measured across an adaptation: the authority, the sensing windows and the
recovery probes all live in that demo, so the payload has to come to them.
Probe dwells consume no body — the shard is not spent on an excluded channel
and not thrown away either.

On-air: `tests/hopset_adaptive_onair.sh` (protocol), and
`tests/hopset_adaptive_jammer.sh` for the decision loop against a real B210
interferer. `MODE=parked` is exclude → improve → restore once the jammer
leaves; `MODE=sense` repeats it with the transmitter's own evidence armed and
asserts it does not contradict a correct receiver; `MODE=herding` moves the
jammer onto a newly-active channel after every exclusion, and with
`FUSION=failsafe MUTE=1` it does so against *transmitter*-originated exclusions,
which is where the anti-herding limits carry the whole load. `MODE=fusionmatrix`
and `MODE=policycost` produce the two tables above. `MODE=hidden` and
`MODE=mirror` are the two disagreement scenarios, built with the injection
lever.

Measured on a 4-channel 2.4 GHz base with a narrowband interferer parked on one
member: the jammed channel excluded and probe-restored after the interferer
stopped, with delivery and FEC delivery as tabulated above. Under herding, with
the transmitter deciding alone, the active set held at the floor of 3, every
committed update moved exactly one channel, exclusion depth stayed at 1 and
delivery did not collapse (0.840 → 0.841) — the interferer is tracked, not
out-run. In the hidden-node row the receiver's exclusion passed unvetoed and
the applied mask was bit-identical to its proposal, with the disagreement
recorded rather than acted on; in the mirror row a transmitter told a healthy
channel was filthy spent nothing, and said so on the record.

Three rig notes that each cost a session: a flat TXAGC index chosen to back
5 GHz off can put 2.4 GHz below the receiver's sensitivity and look exactly
like a dead link; an interferer loud enough to splatter across the whole band
is broad degradation, not a channel fault, and the policy correctly refuses to
act on it; and a cell that starts before the previous one's demos have released
their adapters measures nothing at all, because the advisory per-adapter lock
is held until the chip has been de-initialised.

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
61.44 MS/s. Adaptive exclusion is receiver-driven and opt-in: with the policy
off the schedule visits every configured channel and the fused-FEC layer
absorbs jammed dwells as erasures, and with it on the transmitter senses for
itself only when asked to. A herding jammer cannot be out-run, only survived:
the floor keeps the link diverse rather than winning the exchange.

And the effect is smaller in payload than the dwell proxy suggests. Against a
parked narrowband interferer the outer code already carries a fixed hopset
through its jammed dwells, and carrier sense keeps the transmitter from
spending many frames there in the first place, so exclusion improves the
delivered ratio by a few points and the delivered *rate* not at all. The two
disagreement scenarios that decide whether the veto is safe are demonstrated
with fabricated transmitter evidence, not with a real hidden node: a bench
where the adapters sit inches apart cannot produce one, and that limit is a
property of the bench rather than of the design.
