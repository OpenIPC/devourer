# N-channel dwell-1 hopping

The fifth and last standard-driver FHSS experiment: an arbitrary N-channel
dwell-1 hop sequence — one admitted data-frame opportunity per wall-clock slot,
the radio stepping through N channels — with keyed order, per-target register
parity, and a single receiver tracking it. It closes the series that began with
the [kernel channel-switch baseline](kernel-channel-switch-baseline.md) and ran
through the [firmware offload](kernel-channel-switch-offload.md), the rejected
[MCC/FCS scheduler](mcc-fcs-investigation.md), and the two-context
[dwell-1 A/B data plane](dwell1-ab-injection.md).

## Why this is not the MCC "ping-pong" the issue proposed

Experiment 5 was framed around escaping MCC's two-context limit by rewriting
the *inactive* MCC context while the active one carries traffic. That requires
a *running* MCC scheduler — and experiment 3 established that MCC never reaches
`DOING_MCC` in this environment (it engages only for non-groupable channel
pairs and only from a P2P GO/GC association path, unreachable here). There is no
running scheduler whose inactive context could be rewritten.

The premise is moot anyway. Ping-pong exists only to work around a two-context
firmware limit; the dwell-1 data plane of experiment 4 runs over the ordinary
**synchronous** channel switch, which has *no* context limit. Going from two
channels to N is not a context-juggling protocol — it is `channel_index(slot,
N)`. The `HopSchedule` (`src/HopSchedule.h`) already produces an N-channel
order, sequential or keyed, for any N. So this experiment delivers exp-5's real
goal — arbitrary N-channel dwell-1 hopping with the same rigor — over that data
plane, and records the MCC ping-pong mechanism as a no-go inherited from #271.

## Method

`examples/dwelltx` takes `DEVOURER_DWELL_CHANNELS=c0,c1,…` (any N ≥ 2) and runs
the exact dwell-1 admission of experiment 4 — one frame per slot inside a
bounded window so it can never cross into the next slot's channel — with the
per-slot channel chosen by `channel_index(slot, N)`. Each frame carries a
`HopSyncMarker` whose slot maps, through the same schedule, to the channel that
slot belongs to.

`tests/dwell1_ab.py` runs **one oracle per channel** and attributes each slot to
the oracle that decoded it with the **highest RSSI** — the own-channel oracle
dominates, and an adjacent oracle that hears a weaker leaked copy is not counted
as a wrong-channel event; a true mis-air (a frame aired on the wrong channel)
still surfaces because the wrong oracle is then the strongest. `wrong_channel` =
strongest-RSSI oracle's channel ≠ `expected_ch(slot)`. Keyed attribution reuses
the SipHash-2-4 + Fisher-Yates port in `tests/hop_rx_probe.py`; the Python order
matches the C++ on-air order slot-for-slot (verified below).

Rig: RTL8822BU DUT, four oracles — 8821AU / 8812CU / 8811AU / 8852BU on the four
channels. Near-field oracles saturate at full power (strong RSSI, poor EVM), so
the DUT runs at a modest flat TXAGC index (`--tx-pwr 10`); this is an
instrument constraint, not a data-plane one.

## Results — N-channel delivery

36/40/44 (and +48), 20 MHz, 20 ms slots, software `FastRetune`, sequential
schedule:

| schedule | slots | delivered on correct channel | wrong-channel | dup |
|---|---|---|---|---|
| 3-channel (36/40/44) | 100 020 | 96.7 % | **0** | 0 |
| 4-channel (36/40/44/48) | 100 020 | 95.4 % | **0** | 0 |

Over **200 040** dwell-1 slots across the two schedules, not one frame aired on
a channel other than its slot's, and not one was a duplicate re-air. Per-channel
delivery is 92–98 %; the spread is oracle sensitivity, not the data plane —
near-field oracles saturate at full power, so the DUT runs at a reduced flat
TXAGC index. **Late-enqueue fault** (host enqueue delayed to 18 ms into a 20 ms
slot) drops deterministically: 613/614 slots dropped-late, zero mis-aired.

## Keyed schedule determinism

The Python attribution (`keyed_sequence`, SipHash-2-4 + Fisher-Yates, reused
from `tests/hop_rx_probe.py`) reproduces the C++ `HopSchedule` on-air order
slot-for-slot — e.g. seed `deadbeef` over `{36,40,44,48}` gives
`44,40,48,36,48,36,40,44,…` on both sides. A keyed 3-channel run is delivered
with **zero wrong-channel** under that attribution, and a single receiver
tracks it in lockstep: `rxdemo` acquires the keyed schedule from the markers
(798 tracked decodes / 12 s) and **re-acquires from a cold restart** (784), no
persistent phase inversion — the marker's absolute slot needs no prior state.

## Register parity per target, across AGC buckets

`tests/dwell1_parity.sh` proves every fast hop leaves the chip in the same
channel/BW register state as a full `SetMonitorChannel` to that target —
**6/6 PASS**, from a UNII-1 init (36, AGC bucket 1) to targets 40/44/48
(same bucket) and 149/157/165 (UNII-3, AGC bucket 3). The cross-bucket hops
reprogram the AGC index, CFO `fc`, VCO band and spur state and match the full
path exactly, so a dwell-1 frame on any target — including across sub-band
buckets — airs with correct calibration: channel, bandwidth and per-channel
state change as one logical generation.

## Benchmark — N-channel scaling

Software `FastRetune` (the dwell-1 primitive per experiment 4), 100 k slots
each:

| channels | switch med / p99 (µs) | admit jitter p99−p1 (µs) | delivery | wrong-ch |
|---|---|---|---|---|
| 2 | 2107 / 2346 | 132 | 96.2 % | 0 |
| 3 | 2108 / 2341 | 109 | 96.7 % | 0 |
| 4 | 1952 / 2311 | 123 | 95.4 % | 0 |

The transition cost, admission jitter and delivery are **flat from 2 to 4
channels** — N-channel hopping carries no per-channel penalty, because a hop is
one synchronous retune regardless of how many channels the schedule spans. The
ping-pong-MCC and fixed-A/B-MCC benchmark rows the issue asked for are recorded
**N/A: MCC never runs** in this environment (#271), so there is nothing to time.

## Go / no-go — closing the series

**N-channel dwell-1 hopping works**, and it needed none of the MCC
context-juggling exp-5 was framed around. Arbitrary N-channel schedules —
sequential or keyed — run at ≥ 100 k slots with zero wrong-channel, deterministic
overload, register-correct hops across AGC buckets, and single-receiver lockstep
recovery, all over the ordinary synchronous channel switch whose cost does not
grow with N.

The MCC "ping-pong" mechanism is a **no-go**, inherited from #271: there is no
running scheduler whose inactive context could be rewritten, and no need for one
— the two-context firmware limit that ping-pong exists to escape simply does not
apply to a caller-side admission scheduler over a switch with no contexts.

This closes the five-experiment series. Its through-line: the kernel drivers'
own retune paths are too slow or unsuitable (baseline #269, MCC #271), the chip
firmware's H2C 0x1D switch is fast for continuous hopping (#270, shipped as
`DEVOURER_FASTRETUNE_FW`), and the synchronous switch plus a caller-side dwell-1
admission scheduler is the deterministic per-slot data plane — scaling cleanly
from two channels (#272) to arbitrary N.
