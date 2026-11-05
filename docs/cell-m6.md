# M6 — handover between PTP-synced cells

M6 (#260, EPIC #254) moves a UE's schedule from one cell to another,
make-before-break, on the shared PTP timebase (M4) under the CU (M5). There
is no scan, no clock re-acquisition and no association exchange.

## The pieces

| piece | where | what |
|---|---|---|
| filtered measurements | `src/cell/Handover.h` `L3Filter`, `MeasTable` | 3GPP L3 filter per (cell, UE) over uplink RSSI/SNR. Stale = absent, never weak. |
| measurement reports | F1 `Report.meas` / `heard` | each DU's own UEs and the other cells' UEs it hears, with sample ages |
| A3 (CHO) condition | `Handover.h` `A3Trigger`; `tools/ran-controller` | offset, hysteresis, per-neighbour time-to-trigger, dwell |
| the command | `src/cell/HoCommand.h` `HoCommand`, F1 `Type::Handover` | UE, target id and position, switch instant, TSF delta |
| the UE's copy | `HoIe` in the source DU's beacon | CRC and cell-key tag (forgery is inert); `HoReceiver` rejects a replay |
| clock continuity | `ho_tsf_delta_us`, `TsfSync::ShiftRemote` | the target's TSF is the source's plus a known constant |
| DAPS (same channel) | `Handover.h` `SeqDedup`, `UlStreams`; F1 `Type::Uplink` | the uplink delivered through both cells during a crossing, once at the CU |

## TSF continuity

Every DU holds its TSF grid to PTP time *t*. At network superframe *N*, DU
*k*'s TBTT is at *t = phase_k + N·P*, with its local superframe
*N − net_off_k*, so:

    tsf_k(t) = t − phase_k − net_off_k · P
    tsf_target − tsf_source = (phase_s − phase_t) + (net_off_s − net_off_t) · P

- The CU computes the delta from the two DUs' reports (`phase_us`,
  `net_off`).
- At the switch, the UE shifts its `TsfSync` fit by the delta
  (`ShiftRemote`). The fit and its history stay, `Ready()` stays, and the
  target's beacons extend the same line.

## The sequence

**1. Trigger (CU).** Each plan round, the CU runs the A3 per UE: the serving
DU's filtered own measurement against every other DU's heard one. A value
counts only while its sample age is under `RAN_MEAS_STALE_MS`.
- `RAN_HO_EXECUTE=0` logs candidates only (`ran.ho_candidate` /
  `ran.ho_clear`).
- With execution on, a firing candidate becomes a command. The CU picks the
  target's lowest free UE id and first free position, sets the switch
  `RAN_HO_LEAD_SF` (24) superframes ahead, and computes the TSF delta.

**2. Two-phase commit.** The command goes to the target DU first, and to the
source only once the target has staged the UE. So a refusing target never
leaves a command on the air.
- The source gets it on the target's ack, not a round later; the lead would
  otherwise be spent waiting.
- **Cancel:** any failure or timeout sends the target a cancel
  (`target_pos = 0`), retried until acknowledged. A DU cancels only the id it
  staged or published, so a cancel is safe to send on any failure and never
  removes another UE that holds the same id.
- **Stale replies:** a `Stale` reply answers a reordered copy of a resent
  command and is not a verdict.
- **No clock, no command:** the CU sends nothing to a DU without a network
  superframe. A DU that is Lost neither airs a handover IE nor publishes an
  arrival.

**3. Target DU.** It pre-admits the UE under the commanded id (the registry
must hand out exactly that id). It publishes the UE's grant in the commanded
position with the switch instant as the map's effective superframe.
- The arrival is recorded before the registry admit, so the UE can never be
  granted as an ordinary joiner.
- An arrival that misses its publish window expires (`cell.ho.target_expired`)
  and its id comes back. A deferred arrival removes only its own grant from
  the pass's map; every other change stands.

**4. Source DU.** It airs the tagged `HoIe` until the switch.
- *CELL_HO_RELEASE_SF* (20) superframes after the switch, it releases the
  UE, but only if the UE has stopped uplinking there (its last uplink as ours
  is at most the switch superframe + 2).
- If the UE is still heard as its own, it never switched, so it keeps its
  grant (`cell.ho.kept`).

**5. UE.**
- It accepts the newest fresh IE (`HoReceiver`: a newer id, an instant still
  ahead), and buffers the target's grant maps from the target's beacons.
- At the instant it shifts its clock, takes its target identity (MAC, ACK
  responder, uplink beacon) and runs the target's map.
- *Conditional:* with no target grant within `CELL_HO_FALLBACK_SF` (10)
  superframes, it shifts back and resumes at the source, which still holds it.

**6. Done (CU).** `ran.ho_done` is logged when the target reports the UE. The
UE's old key is retired, so the source's last stale reports cannot trigger
again, and the moved UE starts its dwell.

## Rig result

**Repeated and asserted** with testrig `scripts/m6_handover.sh`: 5 normal
runs and 3 target-crash runs (`--kill-target-on-stage`), 90 s each. All 8
pass.

| KPI | min | median | max |
|---|---|---|---|
| interruption: first in-slot uplink at the target after the switch (superframes) | 1 | 1 | 1 |
| CU `ran.ho_done` after the instant (superframes) | 5 | 6 | 16 |
| fallback delay after the switch, target crash (superframes) | 10 | 10 | 10 |

The 1-superframe interruption is the UE's hardware beacon in the switch
superframe itself, which still sits at its old offset until the UE's one TSF
step. The fallback delay is `CELL_HO_FALLBACK_SF`.

The first runs, single, measured with testrig `scripts/m5_ran_coord.sh` plus
`--cu-env`:

- **Roles:** two co-channel cells on ch149; cell 1 is j6412-1 (phase 0) and
  cell 2 is j6412-3 (phase 13450 µs). Cell 1's UE runs on j6412-2.
- **Trigger:** `RAN_A3_OFFSET_DB=-2` forces the candidate, since the UEs are
  stationary. A long dwell allows exactly one handover per run.

| | normal | fault: target DU killed right after staging |
|---|---|---|
| command | one: target staged, then source | first refused by the target (stale position): the source never saw it; the retry was staged by both |
| UE switch | source sf 143 → target sf 137 = DU2's effective sf for the new grant | at source sf 172 |
| target hears the UE | from the first superframe after the switch: 1115 / 1117 uplinks in slot | — (target gone) |
| UE outcome | committed on the target's grant | no target grant: fell back 10 superframes later |
| source | released the UE at switch + 21 | kept the UE (still heard as its own): 1092 / 1092 uplinks in slot after the switch |
| CU | `ran.ho_done` 5 superframes after the instant | `ran.ho_timeout` |

- **Timing advance:** the target's TA loop starts from 0 for the new UE.
  The first arrivals at the target sat about −418 µs from the UL target, the
  same offset the source's TA had converged to. Carrying the TA in the
  command is a possible refinement.
- **The UE's uplink in the fault case:** while no target map is in force,
  the UE does not rebuild its uplink beacon. Its uplink never left the
  source, which is part of why the fallback is seamless.

## DAPS: the duplicated uplink (same channel)

**Common sequence.** A UE's uplink is its hardware beacon, which carries the
UE's own egress TSF. Every DU that hears one copy sees the same stamp, so
`seq = round(egress / P)` (`ul_seq_from_egress`) identifies that uplink at
every DU. The UE needs no change.

**Forwarding.** Each DU forwards its own UEs' uplinks to the CU (F1
`Uplink`) whenever F1 is on. With `CELL_DAPS=1` (the default), during a
handover it also forwards the UE's *other* identity:
- the target forwards the UE's source identity, staged with the command,
  until the switch plus the release window;
- the source forwards the UE's target identity after the switch.

**Delivery.** The CU delivers each uplink once per UE stream (`UlStreams`
over `SeqDedup`). The command *aliases* the target identity onto the
source's stream, so both identities' copies dedup together and the stream
runs on across the switch.
- Every gap in a stream's delivered sequence is logged (`ran.ul_gap`): it is
  the interruption a crossing or a lost path costs.
- A summary goes out every 10 s (`ran.ul`: delivered, duplicates, and which
  DU delivered first).

**Rig test** (testrig `scripts/m6_daps.sh`, 3 runs per mode). The source DU
is killed the moment its UE accepts the handover, so the path dies
mid-crossing about 2 s before the switch.

| uplinks missing from the crossing UE's stream | min | median | max |
|---|---|---|---|
| DAPS (`CELL_DAPS=1`) | 0 | 0 | 1 |
| no DAPS (`CELL_DAPS=0`) | 13 | 14 | 14 |

- **Without DAPS:** the stream loses everything from the kill to the switch,
  and resumes via the target once the UE uplinks under its target identity.
- **With DAPS:** the target had been delivering the source-identity copies
  all along. In one representative run, after the source died the stream
  went on entirely via the target: of 1156 delivered, 31 came first through
  the source and 1125 through the target.

## Bugs the rig found

**A DU with no UE did not pace its main loop.** Its superframe count ran away
from the TSF (186 → 3.5 M), and so did the network superframe it reported.
Every M1–M5 run had a static UE, which hid it. The loop now waits for its
superframe when there is nothing to serve.

**First attempts at execution, in order:**
1. The source's stale reports re-triggered the A3 for the UE that had just
   left, which issued a duplicate command. The CU now retires the old key.
2. A command refused by the target was already on air at the source. This is
   what led to two-phase.
3. The two-phase source send waited for the next round, so the source got
   every command too late. It is now sent on the target's ack.
4. The release check was off by one: the UE's last old-identity uplink falls
   in the switch superframe itself.

**From the review:**
- a lost target ack or a timeout leaves the target's reservation behind;
- a reordered Stale reply aborts a good handover;
- a missed arrival window freezes the target's map;
- handover runs without sync;
- a retired key can be reused;
- an unchecked ACK-responder re-arm;
- the pre-admit race;
- a cancel consumed before its publish;
- a flag read after it was overwritten.

## Not covered yet

- **DAPS beyond same-channel uplink:**
  - downlink duplication;
  - the different-channel (sequential) variant, with its retune and deaf-time
    budget.
- **A moving UE:** the rig's UEs are stationary, so the crossing is forced
  with a negative A3 offset.
- **Different-channel handover:** this needs `FastRetune` and the
  deaf-time budget.
- **Security context:** the cells share one key on the rig (`CELL_PSK`).
  Pre-staging a per-cell context (802.11r-style) is open.
- **EVM:** not in the measurement reports.
