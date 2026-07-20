# Dwell-1 A/B injection over the channel switch

A two-context A/B hopping **data plane**: the radio spends one wall-clock slot
per channel and each slot carries exactly one admitted data-frame
opportunity — "dwell-1". This is the fourth standard-driver FHSS experiment
(after the [baseline](kernel-channel-switch-baseline.md), the
[firmware offload](kernel-channel-switch-offload.md), and the rejected
[MCC/FCS scheduler](mcc-fcs-investigation.md)), and it answers the question
MCC could not: can firmware-grade channel time-slicing carry real per-slot
traffic with deterministic queue ownership?

It can — with the **synchronous** channel switch. The result also draws a
sharp line between devourer's two channel-switch primitives: the
fire-and-confirm-later firmware switch (H2C 0x1D) that wins continuous
per-packet hopping is the *wrong* primitive for tight slot placement, and the
synchronous software `FastRetune` is the right one. They have opposite sweet
spots.

## What dwell-1 means here

One dwell = one admitted devourer data-frame opportunity, placed inside a
bounded window that guarantees it finishes airing before the slot flips:

```
 slot start                                            slot end
 |── switch ──|── settle ──|═══ admit window ═══|── guard ──|
              ^            ^                     ^
              FastRetune   admit_open            admit_close
              returns      = start+settle        = end-guard-airtime
```

A frame is admitted only inside `[admit_open, admit_close]`. Past
`admit_close` it is **dropped** with a structured reason, never transmitted —
so a frame built for context A can never air on B. This bounded admission is
pure caller policy (`examples/dwelltx`); the library is unchanged. It is the
data-plane discipline the per-packet hop (radiotap `CHANNEL`, ~1 ms retune)
lacks on its own.

Each per-slot frame carries a `HopSyncMarker` (`src/HopSchedule.h`) whose
`slot` field, run through the shared public/keyed schedule, tells any receiver
which channel the slot belongs to. An oracle pinned to channel A that decodes
a frame whose slot maps to B is therefore a self-evident **wrong-channel**
event, and the unique per-slot marker doubles as the duplicate key — no extra
tag, and attribution needs no cross-clock alignment.

Harness: `examples/dwelltx` (the admission scheduler) + `tests/dwell1_ab.py`
(two host oracles on A/B, per-slot accounting, fault injection). DUT
RTL8822BU; oracles 8821AU (ch A) + 8812CU (ch B).

## Result — the synchronous switch carries dwell-1 cleanly

36↔40, 20 MHz, 20 ms slots, software `FastRetune`:

| run | slots | delivered on correct channel | wrong-channel | dup | late-drop |
|---|---|---|---|---|---|
| primary | 5 020 | 97.2 % | **0** | 0 | 0 |
| reliability | 100 013 | 96.2 % | **0** | **0** | 7 (0.007 %) |

Admission is tight: the in-slot placement of the admitted frame has a
p99−p1 jitter of **132 µs**, and the switch itself costs ~2.1 ms median
(p99 2.3 ms). Across **100 013** slots not one frame aired on the wrong
channel and not one was a duplicate re-air; the ~4 % shortfall is
first-decode/oracle loss, not mis-delivery. That is the acceptance
criterion — ≥ 100 k slots, zero wrong-channel, a quantified 0.007 %
late-drop rate — met.

**Fault — late enqueue.** Delaying the host enqueue to 18 ms into a 20 ms
slot (past `admit_close`): of 516 slots, 511 were dropped-late and 5 admitted,
with **zero wrong-channel**. The overload path is deterministic and
observable: a frame that cannot make its slot is dropped, never aired stale.

## The two switch primitives have opposite sweet spots

Running the identical dwell-1 loop over the **firmware** switch
(`DEVOURER_FASTRETUNE_FW=1`) instead inverts the schedule at 20 ms slots —
frames land on the previous slot's channel. The firmware switch is
fire-and-confirm-later: `FastRetune` returns after submitting the H2C, and in
the *sparse* dwell-1 call pattern (one retune per 20 ms, then idle) the RF
completes far later than the ~1 ms it shows under continuous per-packet
hopping. A post-switch settle sweep (20 ms slots) shows how late:

| settle before admit | delivery (correct channel) |
|---|---|
| 2 ms | 3 % |
| 6 ms | 36 % |
| 9 ms | 63 % |
| 12 ms | 82 % |

Even 12 ms of settle inside a 20 ms slot does not fully clear it; a 40 ms slot
with 15 ms settle reaches 97 %. So the firmware switch *can* carry dwell-1,
but only with a guard so large it erases the point of a tight slot. The
synchronous software `FastRetune` — RF programmed by the time the call
returns — places the frame deterministically with a ~2 ms settle and is the
correct dwell-1 primitive.

This does not contradict experiments 2–3: there the firmware switch halves
the dark time of *continuous* hopping, where frames flow immediately after the
switch and its late completion is masked. Dwell-1 exposes that latency. The
two results compose into one rule: **firmware switch for throughput hopping,
synchronous switch for deterministic slot placement.**

## Lockstep receive

A single receiver hopping the same A/B schedule (`rxdemo` lockstep mode —
`src/HopSchedule.h` `HopSyncMarker` tracking, anchor phase-correction) acquires
the schedule from the per-slot markers alone: 889 tracked `hop.rx` decodes over
a 12 s window against the dwell-1 transmitter. A cold restart of the receiver
re-acquires from scratch (861 decodes in the next 12 s) with no persistent
phase inversion — the marker carries the absolute slot, so acquisition needs no
prior state and survives a full restart.

## Go / no-go

**Go — advance to experiment 5.** Firmware-grade channel time-slicing carries
a real per-slot data plane: over 100 013 A/B slots, every admitted frame that
was delivered landed on the correct channel (zero wrong-channel, zero
duplicate), overload is deterministic (late frames dropped, never mis-aired),
and a single receiver acquires and recovers the schedule. This is the
capability MCC/FCS could not provide (it never ran a schedule from the
association path, and its slots are TU-coarse and two-context-bound —
[mcc-fcs-investigation.md](mcc-fcs-investigation.md)); the caller-side
admission scheduler over the ordinary channel switch does.

The load-bearing design choice for experiment 5: use the **synchronous**
`FastRetune`, not the firmware switch, as the slot-boundary primitive. The
firmware switch (`DEVOURER_FASTRETUNE_FW`) remains the right tool for
continuous per-packet hopping (experiments 2–3), but its fire-and-confirm-later
completion is too late and too variable for tight dwell-1 placement. The two
primitives are complementary, not interchangeable, and the data plane must pick
the synchronous one.

Open items experiment 5 inherits: the ~4 % first-decode/settle loss (a frame
occasionally airs before the switch fully settles — recoverable by a slightly
larger settle at a small duty cost), and bidirectional/ACK turnaround, which
this experiment deferred to keep the admission contract unidirectional and
clean.

The A/B contract here generalises to arbitrary N channels with no per-channel
penalty — [N-channel hopping](n-channel-hopping.md).
