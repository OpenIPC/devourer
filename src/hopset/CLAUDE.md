# src/hopset/ — keyed FHSS, adaptive hopset, TX sensing

Deep facts for this subtree, loaded alongside the root CLAUDE.md. Everything
here is header-only and pure (no device access, no env reads) and ctest-gated;
the demos map the env vars. The device-side primitives these build on
(`FastRetune`, `FastSetBandwidth`, the radiotap `CHANNEL` field) live in the
core and are described in the root file. Article-length treatment:
`docs/fhss.md`, `docs/frequency-hopping.md`, `docs/jammer-resilience.md`.

## Demo knobs

Hop set and pacing: `DEVOURER_HOP_CHANNELS` (SweepSpec grammar: `1,6,11`,
`36-48/4`, `5170-5250/5` MHz) + `DEVOURER_HOP_DWELL_FRAMES` / `_ROUNDS` /
`_FAST` / `_RADIOTAP` / `_BW` / `_OFFSET`. `DEVOURER_HOP_PROF=1` emits
per-stage `debug.hop_prof` timing.

Keyed slot mode: `DEVOURER_HOP_SLOT_MS` (monotonic wall-clock slots) and
`DEVOURER_HOP_SEED` (≤32 hex, a 128-bit key). `DEVOURER_HOP_SYNC_EVERY` is
`streamtx`'s sync-marker cadence.

Adaptive: `DEVOURER_HOP_ADAPTIVE=1` (keyed slot mode only),
`DEVOURER_HOP_ADAPTIVE_SCRIPT="slot:mask,..."`, `DEVOURER_HOP_MIN_ACTIVE`,
`DEVOURER_HOP_POLICY=1`, `DEVOURER_HOP_PROBE_ROUNDS` (default 8, **must match
both ends**), `DEVOURER_HOP_FUSION=rx|veto|either|failsafe`.

Sensing: `DEVOURER_TX_SENSE=1` + `_WINDOW_US` / `_POSTBURST_US` / `_SETTLE_US`
/ `_EVERY` / `_NHM`.

## Keyed FHSS + lockstep RX

`src/HopSchedule.h`: the seed replaces the public round-robin with a stateless
SipHash-2-4 Fisher-Yates permutation per round, so a receiver joins without RNG
state. In slot mode the TX demos emit a sync marker (`txdemo` its beacon;
`streamtx` its own frame every `DEVOURER_HOP_SYNC_EVERY`, FEC PSDU untouched),
and `rxdemo` hops in lockstep when `DEVOURER_HOP_CHANNELS` + `_SLOT_MS` are set
(seed optional → keyed/sequential), emitting `hop.rx` acquire/track/retune
events.

## Adaptive hopset

The immutable base hopset carries a per-generation `active_mask`. **TX is the
schedule authority**: RX proposes, TX commits an absolute future activation
slot, repeated until it arrives, over SipHash-MAC'd Proposal/Commit/Status
frames whose schedule and control keys are domain-separated from
`DEVOURER_HOP_SEED`. Generation 0 ≡ the legacy fixed schedule byte-for-byte;
gen ≥ 1 re-keys the permutation from (subkey, generation, round, mask).

Acquisition always scans the base hopset; the v2 sync marker advertises
(generation, mask fp) so a follower that missed the commits recovers from the
status beacon.

## Exclusion policy (`HopsetPolicy.h`)

Per-dwell delivery evidence drives conservative exclude/restore decisions.
**Delivery is authoritative** — energy and link verdicts only classify. The
guards, all deliberate:

- ≥8 scored rounds before anything is proposed
- one channel per update
- <30% delivery for 5 visits **with a healthy alternative**
- hard `max(3, configured)` active floor
- ≥10 rounds between updates — a *refused* proposal spends the budget too, so
  bouncing proposals cannot flood control
- nothing under broad degradation; excluded-fraction cap

Proposals air from rxdemo's own claimed handle; txdemo hears them under
`DEVOURER_TX_WITH_RX=thread`, and the authority enforces its own shape limits
(`max_mask_delta`, `min_update_gap_rounds`).

Excluded channels are revisited by **keyed recovery probes**: every P rounds
one data slot is replaced, with round, position and channel all keyed so
recovery is not a periodic target. Probes carry sync/control only, never caller
FEC payload. Events: `hopset.*`, including `hopset.decision` and
`hopset.probe`.

## TX sensing + endpoint fusion (`HopsetSense.h`, `HopsetFusion.h`)

The transmitter opens quiet windows (retune → settle → discard barrier →
window → read) and scores occupancy from CCA/FA/IGI/NHM. It refuses to arm
beside `DEVOURER_TX_THREADS>1`, under a ~1.5 ms window, or while committing (a
commit inside the window is self-jamming that would feed the veto).

Saturation constants are **measured**: ~3 events/ms = fully jammed, ~0.6 =
clean. A railed IGI or NHM contributes nothing.

The veto may only argue *"this move leaves ME worse off"* — broad TX
degradation → delay; every survivor worse → reject. It may never argue "I
disagree about your target": that signature IS the hidden-node case, where the
RX is right. Autonomous changes go through `start_local_change` (structural
limits apply; the scripted lever stays exempt).

The classifier needs `_POSTBURST_US` armed before it proposes anything of its
own — without it the TX observes and holds, which looks exactly like a TX that
is not sensing.

Test levers, **not production knobs**: `DEVOURER_TX_SENSE_INJECT="<idx>:<occ>[,*:<occ>]"`
fabricates the TX's per-channel view (bypasses the hardware read, keeps the
window timing) — the only way to build the hidden-node case and its mirror on a
bench where both adapters hear the same interferer. `DEVOURER_TX_STDIN=1` makes
txdemo carry caller PSDU bodies from stdin (`streamtx` framing) so FEC delivery
can be measured across an adaptation. `DEVOURER_HOP_MUTE=1` on rxdemo injects a
one-way outage.

## Validation

`tests/run_hop_validation.sh`, `tests/hop_parity_check.sh` (register parity
full-vs-fast), `tests/run_jammer_resilience.sh` +
`tests/sdr_follower_jammer.py` (B210 follower, reactive vs predictive),
`tests/hopset_adaptive_onair.sh` (protocol), `tests/tx_sense_probe.sh` (is the
telemetry live at all), and `tests/hopset_adaptive_jammer.sh` (B210 interferer;
`MODE=parked|herding|sense|failsafe|hidden|mirror|fusionmatrix|policycost|prepost|overhead`,
every mode reporting FEC delivery beside the marker proxy over a window that
opens only once the RX is tracking).

**Pick adapters by band** — a 5 GHz-only PA part senses nothing and cannot
answer on 2.4 GHz.

### What the measurements actually say

Sensing costs ~8% of frames (25 ms window, 1-in-2 dwells) — a throughput cost,
not a delivery-ratio one; FEC delivery is unchanged.

**The marker proxy overstates adaptation.** Against a parked jammer it reads
0.762 → 0.946, where payload reads 0.861 → 0.927 and the delivered packet rate
does not move at all — carrier sense already keeps the TX out of the jammed
dwells. Quote both numbers together or the feature reads better than it is.
