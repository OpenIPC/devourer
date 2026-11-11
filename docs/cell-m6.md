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
| measurement gaps | `src/cell/MeasGap.h`; F1 `Type::Neighbours`, `Report.ue_meas` | the UE measures another channel's beacon in a gap clear of its slots |
| phase planning | `src/cell/PhasePlan.h`; F1 `Type::Phase` | the CU places a joining DU's TBTT so the cells can measure each other |
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

## Inter-frequency handover (the target on another channel)

**The channel travels with the command.** `HoCommand.target_channel` comes
from the target DU's report, and the UE's IE carries it (`HoIe.channel`, IE
version 2).

**The switch.** At the instant, the UE does everything it does on the same
channel: it shifts the clock by the TSF delta, takes the target identity and
runs the target's map. In addition it:
1. stops its uplink beacon;
2. retunes (`FastRetune`, Kestrel's lean intra-band 20 MHz path);
3. restarts the beacon on the new channel.

It needs no beacon of the target beforehand, since the delta already places
its clock. It cannot have buffered the target's grant maps, because it never
heard the target, so its grant arrives with the target's first beacon after
the retune.

**The deadline to return.** With no target grant within
`CELL_HO_FALLBACK_SF` (10) superframes, the UE retunes back and resumes at
the source, which still holds it. This bounds the deaf time of a failed
handover.

**Trigger.** With one radio per DU, no DU hears a UE on another channel, so
the A3 needs the UE's own measurements: see "Measurement gaps" below.
`RAN_HO_FORCE=SRC:UE:TGT@SECONDS` remains for tests: it issues one command
at that time through the normal execution path.

**Rig result** (testrig `scripts/m6_handover.sh --channel2 157`). Cell 1 is
on ch149 and cell 2 on ch157, both UNII-3. There were 5 normal runs and 3
with the target killed after staging; all 8 pass.

| KPI | min | median | max |
|---|---|---|---|
| switch lands on the target's effective superframe | 8/8 | | |
| `FastRetune` 149↔157 on the 8852CE (11 retunes) | 23.5 ms | 23.6 ms | 23.7 ms |
| first target beacon after the retune | 37 ms | 52 ms | 101 ms |
| interruption: first in-slot uplink at the target after the switch (superframes) | 2 | 2 | 3 |
| CU `ran.ho_done` after the instant (superframes) | 5 | 6 | 16 |
| target crash: fallback (superframes) | 10 | 10 | 10 |
| target crash: uplinks the source missed (superframes) | 10 | 10 | 10 |
| target crash: first source beacon after the retune back | 56 ms | 57 ms | 91 ms |

- **Interruption:** one superframe more than on the same channel. That is
  the retune (about 24 ms) and the beacon restart; the first target beacon
  comes within one beacon interval.
- **A failed handover:** the UE is away from the source for exactly the
  fallback window. This is the sequential, deaf-time cost #260 asks to
  budget.
- **An unexplained observation:** in one target-crash run, the UE received a
  beacon of the source (ch149) while still tuned to ch157, about 1 s after
  the retune, before its own retune back. Its cause is not established. The
  rig script does not count it as the post-return beacon.

## Measurement gaps (the inter-frequency trigger)

The UE measures the cells' beacons (downlink): its serving DU's all the
time, and a neighbour's on another channel in a measurement gap
(`src/cell/MeasGap.h`).

**Where to listen.** The CU sends each DU its neighbours' channels and TBTT
phases (F1 `Neighbours`, every round). Every synced DU airs them as offsets
on its own grid, in a tagged NL IE with a generation, empty lists included:

    offset = (phase_neighbour − phase_serving) mod P

On the shared timebase a neighbour's beacon always falls at that offset of
the UE's serving superframe, so the UE needs nothing beyond its serving
clock.

- **Newer only:** a UE takes only a list generation at least as new as the
  last it accepted.
- **Clearing:** it drops a list not heard for 3 s, and clears it on a
  handover.
- **Provenance:** the NL's tag is also what lets a UE count a beacon as a
  measurement of a cell. A frame merely carrying a cell-shaped address is not
  measured.
- **Residual risk:** a genuine beacon replayed by someone else, which is
  only ever the real cell's beacon at the replayer's power.

**The gap.** Each measurable other-channel neighbour is revisited every
`CELL_UE_GAP_SF` (10) superframes; with several, the gaps are spaced
accordingly. For each, the UE computes `gap_window()`:

    [offset − retune − guard, offset + beacon + retune + guard)

The window must clear the serving beacon and the UE's own DL and UL slots.

- **Retune bound:** both retunes are planned with a conservative bound,
  `CELL_UE_GAP_RETUNE_US` (26 ms), raised by any slower retune measured.
  The 8852CE's retune measured 23.5–23.7 ms on the rig, so 26 ms is that
  plus about 10 %. The first gap results below ran with 30 ms. A band change,
  which takes the full channel path, never gets a gap.
- **No fit:** the UE skips that neighbour (`cell.ue.gap_skip`), parks it, and
  re-checks it rarely. No slot is ever sacrificed.
- **Late wake-up:** if the UE wakes for a gap later than half the guard, it
  skips it (`cell.ue.gap_late`).
- **Outbound overrun:** an overrun shortens the listening, never the return
  budget.
- **Failed return:** a return that fails is retried. If it still fails, the
  beacon is stopped and the normal retune path recovers
  (`cell.ue.gap_return_fail`).
- **TSF steps after a gap:** these wait 4 superframes. On the rig, the only
  uplink losses seen with gaps (twice, two beacons each, in 10 runs) followed
  a gap with a TA step in the next pass; the 20 gaps without one lost
  nothing. The mechanism is not established. With the hold, 8 runs and 21
  gaps lost none.

**Reports.** The UE L3-filters every authenticated cell beacon it hears,
serving or neighbour. It reports the filtered RSSI/SNR, with each entry's
sample age, in a tagged MR IE in its uplink beacon, refreshed each
`CELL_UE_MR_MS` (1 s).
- **Freshness:** the MR's generation is the serving superframe, strictly
  increasing. The DU takes only a generation newer than the last and near
  its own count, so a replayed report is refused.
- **Relay:** the DU forwards it over F1 (`Report.ue_meas`). Ages are sample
  age plus report age, each UE's serving entry goes first, and the cap is
  216, whole UEs only.

**The A3 on UE reports.** For each neighbour, the CU's A3 uses one
comparable pair:
- the UE's own (serving beacon, neighbour beacon) when the MR has both
  (metric `ue_rssi`);
- otherwise the DUs' (serving uplink, neighbour-heard uplink).

The A3 runs on the margins, so a neighbour the UE did not report keeps its
DU-heard evidence.

**Rig result** (testrig `scripts/m6_measgap.sh`; 5 runs, and 8 on the final
code). Cell 1 is on
ch149 at phase 0; cell 2 is on ch157 at phase 60000 µs, so its beacon falls
in an idle part of cell 1's grid. The A3 runs at its **defaults** (offset
3 dB, hysteresis 1 dB, time-to-trigger 640 ms), with nothing forced. All
pass.

| KPI | min | median | max |
|---|---|---|---|
| gap deaf time (21 gaps, final code: 30 ms retune bound + 3 ms guard) | 60.0 ms | 60.1 ms | 60.2 ms |
| neighbour beacon in gaps | −46 dBm | −46 dBm | −46 dBm |
| A3 held before firing | 1002 ms | 1024 ms | 1024 ms |
| switch after the network instant (the UE's poll latency) | 21 ms | 21 ms | 28 ms |
| interruption (sf, first in-slot uplink at the target after the switch) | 3 | 3 | 3 |

- **The trigger:** the serving beacon read −55 to −57 dBm, and the
  neighbour's −46 dBm in the gaps. That beat the 3 dB offset plus hysteresis,
  so the A3 fired on UE-reported values in every run.
- **The gaps cost no uplink:** the source heard at least 98 % of the
  superframes before the switch.
- **The extra interruption superframe** compared with the forced runs is
  the phase layout. The UE switches at the source's boundary of network
  superframe N, and cell 2's superframe N starts 60 ms later. The UE's grant
  there takes effect at that boundary.
- **A limit of this layout:** cell 2's UE cannot measure cell 1. Cell 1's
  beacon falls at offset 42400 µs of cell 2's grid, too close to that UE's
  own uplink slot, so it logs `gap_skip`. Phase planning (next section)
  removes this.

**TSF continuity, stated precisely.** The UE switches at the network
instant, the source's boundary of network superframe N. On the target's
clock that instant is:

    eff_target · P − ((phase_t − phase_s) mod P)

The UE's shifted clock reads it plus its poll latency. The rig scripts
assert exactly this, across every same-channel, inter-frequency and
measurement-gap run.

## Phase planning (where a joining cell's TBTT goes)

A gap fits only where the neighbour's beacon falls in an idle part of the
UE's grid, so the phase layout decides who can measure whom. The CU places
a **joining** DU's TBTT; running cells keep theirs (re-phasing a live cell
would move every UE's clock).
- **The scoring:** see `plan_phase` in `src/cell/PhasePlan.h`.
- **The exchange:** a DU with `CELL_PHASE_AUTO=1` asks before it starts its
  beacon (a Report with phase −1) and gets F1 `Phase` (v8). The settings are
  in the env headers of `examples/cell/main.cpp` and
  `tools/ran-controller/main.cpp`.
- **The bound:** the CU plans with the UEs' gap bound (26 + 2 ms on both
  sides).

**The rig grid** has a 1500 µs beacon and three positions (DL 4000, a
1000 turnaround, UL 2000), so the active span is 20.5 ms. Against a cell at
phase 0 on another channel:
- with a 26 + 2 ms gap, both ways work only for a phase in
  [48500, 53900] µs, and the plan takes 51200;
- with the old 30 + 3 ms bound, no phase works both ways.

A third cell on a third channel cannot be measured both ways by both others
at this bound; its plan is partial (score below the maximum), not refused.
Suspend gaps (below) cover that case.

**Rig result** (testrig `scripts/m6_phaseplan.sh`, 5 runs, all pass).
- **Layout:** cell 1 is on ch149 at phase 0. Cell 2 is on ch157 with no phase
  of its own (`m5_ran_coord.sh --phase2 auto`).
- **CU role:** the CU only observes handovers here. The point is that both
  UEs measure the other cell.

| KPI | min | median | max |
|---|---|---|---|
| assigned phase (score 2/2 every run) | 51200 µs | 51200 µs | 51200 µs |
| gaps per UE per run (60 s) | 57 | 57 | 57 |
| of them, the other cell heard | 56 | 57 | 57 |
| gap deaf time (568 gaps, 26 + 2 ms bound) | 54.0 ms | 54.1 ms | 54.6 ms |
| cell 2 in cell 1's UE's gaps | −47 dBm | −46.7 dBm | −46 dBm |
| cell 1 in cell 2's UE's gaps | −57.9 dBm | −57.8 dBm | −57.6 dBm |

- **No gap skipped**, and neither cell missed one of its UE's uplinks (0 of
  about 572 superframes per cell per run).
- **The measurement-gap handover run** (`m6_measgap.sh`, phase 60000) passes
  at the new bound too: 2/2, with gap deaf time 54.1 ms.

**What the first runs found.** The first runs started the UEs a fixed 8 s
after the DUs, with the DU asking for its phase after its beacon had started.
- **Symptom:** in 3 of 3 such runs, cell 2's UE had already locked to cell
  2's beacon when one jump of about 1.3 ms in its timing arrived. Its
  `TsfSync` fit is cumulative from the first beacon, so it took the jump as
  skew (hundreds to thousands of ppm, decaying slowly). Its gaps then missed
  cell 1's beacon. In one run (0 of 42 heard), each gap also cost its uplink.
- **What changed:** the DU now asks before starting its beacon, and the rig
  starts the UEs only once both DUs log `cell.du.start`. Five runs since
  have the UEs' skew at −3 to −8 ppm.
- **Not established:** the source of the jump. The UE now survives one
  (next section).

### Beyond two cells: suspend gaps

**Why idle gaps cannot do it.**
- A gap is 26 + 2 ms on each side of a 0.5 ms beacon: 56.5 ms of a 102.4 ms
  superframe.
- It must clear the 20.5 ms active span. So one direction works only for a
  neighbour offset in [48.5, 73.9] ms, and both directions only for a phase
  difference in [48.5, 53.9] ms: half a superframe.
- Three cells cannot be pairwise half a superframe apart. Whatever the
  layout, and whether or not live cells could be re-phased, some pair of
  three cells on three channels can never measure each other in idle gaps.

**A suspend gap** (`src/cell/MeasGap.h`, `CELL_GAP_SUSPEND_SF`, the same
value on the DU and its UEs) may cover the UE's own slots, the 3GPP way.
- **When:** only in the UE's suspend superframes, one in N (20 on the rig),
  staggered by UE id so a cell's UEs are not all deaf at once.
- **The uplink beacon:** the UE stops it before it retunes (it must never
  fire on the neighbour's channel) and restarts it after.
- **The DL:** the DU airs that UE no DL frame in its suspend superframe or
  the one after, since the gap may run into it. An unACKed frame would cost
  the next one up to CWmax on Kestrel.
- **Agreement:** whether a UE needs suspend gaps at all is decided at the
  floor gap bound on both sides (`needs_suspend`), so the DU and the UE
  agree. A neighbour an idle window reaches is still measured that way.
- **The cost:** at most two superframes' DL and uplink in N (the rig
  measured 1–2 uplinks in 20), and only for a UE some neighbour leaves no
  idle window.

**The planner** (`RAN_GAP_SUSPEND=1`): a direction only a suspend gap
reaches counts 0.25 per position instead of 0.
- It still prefers idle windows: two cells still get 51200.
- A third cell on a third channel is covered both ways with every other cell
  (cell_m6 ctest).

**Rig result** (testrig `scripts/m6_suspend.sh`, 2 runs per case, all
pass). Cell 1 is on ch149 and cell 2 on ch157 at phase 20000: the beacons are
20 ms apart, so neither cell's UE has an idle window for the other's. Suspend
gaps run one in 20 superframes.

| case | each UE's reports carrying the other cell | suspend gaps per UE (heard) | uplink heard by its DU |
|---|---|---|---|
| suspend gaps, idle gaps off (2) | 100 % | 43 (100 %) | UE1 90 %, UE2 95 % |
| none, the counterfactual (2) | 0 % | 0 | 100 % |
| suspend gaps + the A3 executing (2) | 100 % | 42–44 (100 %) | 91–95 % before the switch |

- **No DL frame in a deaf window:** in all suspend runs, no DU aired its UE
  a DL frame while the UE was in a suspend gap. Every skipped DL frame fell
  in the UE's suspend superframe or the one after.
- **No stray beacon:** the witness on ch149 never saw a cell-2 uplink, so a
  stopped beacon never fired on the other channel.
- **The cost** is the uplink's 1 or 2 superframes in 20.
  - UE1's gap (its neighbour 20 ms into its grid) runs from 94.4 ms into
    its suspend superframe S to 46 ms into S + 1, so it covers S + 1's
    uplink slot.
  - UE2's gap (offset 82.4 ms) runs from 54.4 ms into S to 6 ms into S + 1,
    clear of both uplink slots.
  - Both UEs also lose the first uplink slot after the gap (UE1 S + 2, UE2
    S + 1), though that slot comes 10–70 ms after the gap ends: the
    restarted beacon first airs a superframe later. Why is not established.
- **The gaps:** deaf time 54.0–54.6 ms, the other cell at −58 to −45.5 dBm.
- **In slot:** each DU heard its UE in slot 100 % from the first suspend gap
  on. One run had a single out-of-slot uplink 15 superframes before its
  first gap (cause not established; not the gaps).
- **Handover on suspend gaps alone:** in both handover runs UE1 handed over
  to cell 2 on measurements only suspend gaps gave it (A3 at defaults,
  metric `ue_rssi`). The target heard it in slot 99.7–100 %.

### Three cells on the rig

The testrig `m5_ran_coord.sh --du3` adds a third cell. Its DU runs on the
usual witness radio (j6412-2 09:00; no witness in this test), and its UE on
j6412-1 05:00, the module with the weak TX chain, which is fine for
measuring.
- **Channels:** cell 1 on ch149 at phase 0, cell 2 on ch157, cell 3 on ch36.
- **Planning:** cells 2 and 3 ask the CU for their phases.
- **Retunes:** to and from ch36 take the same ~23.7 ms as 149 ↔ 157.

**The plan** (`RAN_GAP_SUSPEND=1`, every run):
- cell 2 at 51200: both ways idle with cell 1, score 2/2;
- cell 3 at 51200 too: both ways idle with cell 1 (2.0), and with cell 2
  through suspend gaps only (0.5).

The planner without suspend credit picks the same phases and leaves 2 ↔ 3 at
0.

**Rig result** (testrig `scripts/m6_three_cells.sh`, all pass). Each cell
has one UE, and a direction c→o is UE c measuring cell o.

| direction | suspend gaps (3 runs): share of reports | measured by | no suspend gaps (3 runs) |
|---|---|---|---|
| 1→2, 1→3 | 100 % | idle gaps (86 per run each) | 100 % |
| 2→1, 3→1 | 100 % | idle gaps (86) | 100 % |
| 2→3, 3→2 | 100 % | suspend gaps (43 per run each) | **0 %** |

- **No DL in a deaf window:** in all suspend runs, no DU aired its UE a DL
  frame inside a suspend gap's deaf interval.
- **In slot:** every DU heard its UE in slot 100 % from the first gap on.
- **No missed windows:** no suspend superframe went by without its gap.
- **The cost** falls only on the cells that need suspend gaps: DU2 and DU3
  heard their UEs' uplinks in 90 % of superframes, and DU1 in 100 %.
- **The counterfactual:** without suspend gaps, the geometric limit is real
  on air. Cells 2 and 3 never measure each other, in any of 3 runs.

### Handover among three cells

The three-cell layout, with the CU's A3 executing on the UEs' own
measurements. Each UE is strongest to the DU on its own box:

| UE (serving) | cell 1 | cell 2 | cell 3 |
|---|---|---|---|
| UE1 (1) | −55 | −46 | **−33** |
| UE2 (2) | −57 | −39 | −38 |
| UE3 (3) | **−33** | −56 | −48 |

**Expected moves** (each UE moves at most once; the A3 dwell is long):
- At the A3 defaults, UE1 goes 1→3 and UE3 3→1, each to the stronger of its
  two neighbours: a swap.
- At a −2 dB offset, UE2 also goes 2→3. That is a handover into the cell
  UE2 could measure only in suspend gaps, since cells 2 and 3 share a phase.

**The first runs found two bugs:**
- **The A3 fired for a weaker neighbour.** It fired for the best neighbour
  *among those past their time-to-trigger*. UE1's timer toward cell 2
  started a second before its timer toward cell 3, so the CU sent UE1 to
  cell 2 at −51 dBm, with cell 3 at −32. Now only the strongest neighbour
  meeting the entry condition can fire, once it has held it for the
  time-to-trigger (cell_m6 ctest).
- **A re-plan cost an arrival its window.** When UE1 left cell 1, the CU
  re-planned and sent DU1 a schedule effective one superframe before UE3's
  arrival there. A DU publishes one instant at a time, so the arrival's
  grant never got its window: it expired, and UE3 fell back to cell 3. The
  CU now sends no new schedule to a DU while it is a handover's source or
  target; it re-plans once the handover concludes.
  - Meanwhile a schedule the DU refused is not dropped but goes on as the
    keepalive, since any fresh-sequence schedule is CU contact. So a long
    handover cannot cost the DU its CU contact; the fresh version follows
    after.
- **A refused command, recovered:** the first command for UE3's arrival
  named id 2, from DU1's report taken before it freed UE1's id 1. DU1's
  registry gives the lowest free id, so it refused the command (invalid);
  the CU's next command, with id 1, was staged.

**Rig result** (testrig `scripts/m6_three_ho.sh`): 2 runs per case, all pass.

| case | moves (each to the strongest neighbour in its last report) | target heard in slot after the switch |
|---|---|---|
| A3 defaults (2) | UE1 1→3 (−32 vs −51), UE3 3→1 (−31 vs −51); UE2 stays | 99.9 % |
| A3 −2 dB (2) | the same, plus UE2 2→3 (−37/−38 vs −57), measured in 9 and 13 suspend gaps, never in an idle one | 99.9–100 % |

- **The misses behind 99.9 %:** every switch between cells of different
  phase (1↔3: 51.2 ms apart) had exactly one target uplink out of slot, of
  724–786. It came in the second superframe after the switch, 49–51 ms off,
  about the phase difference. The switch 2→3, between cells of the same
  phase, had none (0 of 530 and 596), and no fallback followed. The cause
  is Kestrel's beacon timer: the first beacon after the switch's TSF step
  fires at the old phase (see "The switch's own interruption").
- **Every switch:** TSF continuity (18–134 ms after the network instant,
  under 2 P), `ran.ho_done` 5–17 superframes after it, and no fallback.
- **No DL in a deaf window:** no DU aired a UE a DL frame inside a suspend
  gap, on either side of its switch (each side on its own clock).
- **Measuring from the new cell:** every moved UE went on measuring both
  other cells in 100 % of its reports there. UE2 in cell 3 measures cell 2
  in suspend gaps and cell 1 in idle ones.
- **Cell 3 ends with two UEs:** UE1 and UE2, each with its own suspend
  pattern.
- **Regression:** the two-cell emulated crossing (A3 at defaults, and the
  hover case), the measurement-gap handover and the plain and fault
  handovers pass with the A3 change.

### A UE crossing three cells

The emulated crossing (TX ramps, the A3 at its defaults, nothing forced) on
the three-cell layout, with one UE.

**What the rig allows:**
- A DU's TX offset reaches −20 dB at most: Kestrel's effective power floor
  is 0 dBm, and the base is ~20 dBm.
- Each UE radio hears the DU on its own box far above the others, so no
  radio can be made to *enter* all three cells.
- The j6412-3 radio comes closest: cell 1 at −57, cell 2 at −39 and cell 3
  at −38 dBm, all at full power. It can start in cell 1 and cross 1 → 2 →
  3, but entering cell 1 from either other cell is out of reach.
- The second crossing is on the pair it measures only in suspend gaps
  (cells 2 and 3 share a phase).

**The ramps** (DU1 stays at 0):
- DU2 −20 → 0 dB over 40–55 s, holds, then back to −20 over 70–85 s;
- DU3 −20 → 0 dB over 70–85 s.

The CU's DL stream and both DAPS directions are on.

**Rig result** (testrig `scripts/m6_crossing3.sh`, 3 crossings and 2 with
a source failure, all pass).

| per run | 1 → 2 (idle-gap measurements) | 2 → 3 (suspend-gap measurements only) |
|---|---|---|
| trigger margin (A3 defaults) | 5–8 dB | 5–9 dB |
| crossing → switch | 49–69 sf | 39–52 sf |
| measurements of the target before the switch | 13–23 idle gaps | 16–17 suspend gaps, no idle one |

- **Every switch:** TSF continuity (22–82 ms after the network instant),
  the target heard the UE in slot ≥ 99 % until the next switch, and the
  source released it.
- **No DL in a deaf window:** no DU aired the UE a DL frame inside a suspend
  gap, in any serving segment.
- **DL stream:** 0 PDUs missing in the handover windows. The longest DL
  silence was 0.6–1.0 s, at a cost of 156–191 duplicates per run.
- **Uplinks** at the CU: 90–100 missing per run, nearly all of them
  expected.
  - The suspend gaps' own cost: each gap's superframe and the two after it,
    while the UE served in cells 2 and 3.
  - Each switch's own superframe and the next: an inter-frequency switch
    retunes the UE.
  - Beyond those, 0–4 per run.

**A source failure mid-crossing** (DU1 killed the moment the UE accepts
1 → 2, ~2 s before the switch):
- Both switches still happened.
- **DL:** DAPS lost 0 PDUs, though the DL silence grew to 1.3–1.5 s.
- **UL:** 12–14 uplinks were lost in the 1 → 2 window, everything from the
  kill to the switch.
  - Cells 1 and 2 are on different channels, so the target never hears the
    UE's source identity, and uplink DAPS needs one channel.
  - This is the open different-channel DAPS variant, measured here, not
    asserted.

**What the first runs found: a neighbour measured only in suspend gaps
flickered stale at the CU.**
- A UE's report of a cell counted while its sample age plus the report's
  age stayed within `RAN_MEAS_STALE_MS` (2 s).
- Cell 3, measured once per suspend period (2.05 s), kept going stale, and
  the A3 restarted its time-to-trigger each time.
- The 2 → 3 handover fired 112 and 120 superframes after the crossing (one
  run 41), at a 19 dB margin.
- **The fix, scoped per UE:** the CU now works out, per UE, which
  neighbours it reaches only in suspend gaps. It uses the reports: phases,
  channels, the UE's slots, and the suspend period each DU reports (F1 v13,
  `Report.gap_suspend_sf`, so the CU cannot drift from the cells).
- A UE's report of such a neighbour counts for k suspend periods plus 1.5 s,
  where k is how many such neighbours it measures in turn. Every other
  report keeps `RAN_MEAS_STALE_MS`.
- **Not run:** with k ≥ 2 the UE's own measurement table must also keep a
  sample that long (`CELL_MEAS_STALE_MS`, default 3 s).

## The UE's clock fit across a jump of its DU's TSF

The UE fits the DU's TSF against its own free-running counter
(`src/TsfSync.h`), which no TSF write on the UE side touches. So a
discontinuity can only come from the DU's side. The fit is cumulative: a
jump read as skew biases every translation for a long time, as in the first
phase-planning runs above.

**Step detection** (`TsfSync::SetStepDetect`; the UE's settings are in the
`examples/cell/main.cpp` env header):
- **Held out:** a beacon more than 200 µs off the fit is not fitted.
- **Confirmed:** six in a row agreeing on one offset (within 50 µs) are a
  step. Six beacons outlast the DU's own discipline pass (500 ms), so a jump
  the DU undoes itself is never followed. The remote axis is re-anchored by it (`ShiftRemote`: a TSF step
  changes the offset, never the crystals' rate) and the held beacons join
  the fit (`cell.ue.fit_step`).
- **Dropped:** held beacons that never agree are discarded, so one stray
  frame never bends the line.

**Rig result** (testrig `scripts/m6_fitstep.sh`, 3 runs of each case, all
pass; both cells on ch149).
- **Restart:** DU1 is killed and restarted under its attached UE. Its new
  TSF lands on another superframe count, a lasting jump (about −31 s), and
  the restarted DU's acquisition adds a few more steps.
- **Transient:** DU1 adds 1300 µs to its TSF behind its discipline's back
  (`CELL_DU_TEST_TSF_STEP`, test only). The discipline undoes it one pass
  (~500 ms) later.

| case | steps followed | UE skew, max \|ppm\| | DL frames in the DL area after | uplinks off-slot, 20 sf after |
|---|---|---|---|---|
| restart, detection on | the restart jump (−30.5 to −31.1 s) and the acquisition steps | 8.0–10.1 | 100 % | see re-attachment (below) |
| restart, detection off (1 run) | — | 996 125 | 11 % | not judged |
| transient, detection on | none (5 beacons held, then dropped) | 8.4–11.9 | 100 % | 0 |

- **Without detection,** the restart wrecks the fit: the skew reads about
  10⁶ ppm and the DL frames scatter over the whole superframe.
- **The confirm window matters:** with 3 beacons (shorter than the DU's
  undo), the UE followed the transient out and back, 3 beacons late each way.
  That put 4–5 of the next 20 uplinks off-slot, in 3 of 3 runs.

## Re-attachment after a DU restart

A restarted DU's clock is followed (above), but its grant maps begin again
at a low epoch on the new superframe count. A UE's grant receiver refuses
them as old, so without re-attachment the UE keeps its old map and is never
heard in the new grid.

- **The rule** (`GrantMapReceiver::offer_after_restart`): a map at a strictly
  lower epoch than the newest accepted, but due within 16 superframes of the
  beacon carrying it, starts the receiver over. A stale map from before the
  restart is due far from the new count, so it is still refused. The same
  epoch again is the live map repeated, not a restart.
- **When:** only for `CELL_UE_REATTACH_SF` (200) superframes after the UE's
  clock fit confirms a step of the DU's TSF. A DU that never jumped is never
  re-attached to.
- **What else resets** (`cell.ue.reattach`): what belonged to the old DU
  instance, as on a handover switch. That is the DL HARQ sequence space, the
  MR generation (the DU checks it against its own new count), the neighbour
  list and gap parking, and any handover IE the old instance aired.
- **Authentication:** grant maps carry the cell key's tag (v2,
  `src/cell/GrantMap.h`), so with `CELL_PSK` a forged map is refused before
  either rule sees it. It can neither move a UE's slots nor, through the
  restart path, reset its per-DU state. What remains is a replayed genuine
  map. A pre-restart one is due far from the new count and refused; one
  from the new instance is that DU's own.

**Rig result** (testrig `scripts/m6_fitstep.sh`, restart case, 3 runs, all
pass).
- **The case:** DU1 is killed 30 s in and restarted at once, under its
  attached UE. Service start is the restarted DU's first in-loop discipline
  pass.

| KPI (3 runs) | value |
|---|---|
| re-attachments per run | 1 (old epoch 7–11 → epoch 1, at the new DU's superframe 6) |
| uplink heard by the restarted DU, from 10 sf after service start | 99.8–100 % of superframes |
| of them in slot | 99.8 % |
| first in-slot uplink after service start | superframe 0 |
| UE DL frames in their granted slot after the re-attach | 100 % |

- **Before the restarted DU serves,** its grid is still moving: it is
  acquiring, with its beacon on. The UE has already re-attached to it, and
  its uplinks in that time are off-slot against the moving grid. They are
  not counted.
- **With detection off,** the UE's clock fit breaks (above). Re-attachment
  is never armed, and the restarted DU never hears the UE.
- **No spurious re-attachment** was seen in the transient runs or in the
  regression runs. Those were same-channel handover 2/2 plus fallback 1/1,
  measurement-gap handover 1/1 and phase planning 1/1, all passing.
- **A logging fix found on the way:** the UE judged its DL frames against
  the slot matching its id, not the slot of its grant. A CU move to another
  position showed as DL out of slot (0.05 in slot); the frames themselves
  were right. It now uses the grant in force.

## Re-joining through the RACH after a DU restart

Re-attachment (above) brings back a static UE: the restarted DU grants its
id again. A UE that joined through the RACH holds an id the new DU instance
does not know, so after re-attaching it is in no map.

- **The rule:** a joined UE that the authentic map in force has not granted
  for `CELL_UE_REJOIN_SF` (30) superframes drops its id and joins again
  (`cell.ue.rejoin`). That covers a DU that restarted and one that released
  it. The rule is skipped mid-handover; static UEs never re-join.
- **What the re-join resets:** its uplink beacon goes off and its ACK
  responder is cleared. The DL HARQ space of the old id goes. The RACH
  client is new, with a **fresh nonce**: a DU answers a repeated (MAC,
  nonce) request from its cache, which would be the old id.
- **The stable joiner MAC** (`CELL_JOINER_MAC`, or derived from host and
  radio) is kept. A DU that still holds the UE's registry entry gives the
  same position back.

**A driver bug it found.** The first re-joins never got an answer. Every
msgA arrived at the same phase (~29.9 ms, drifting with the UE's own
crystal), past the RACH slot, whatever offset the UE drew.
- **Not the cause:** the UE's timing checks passed.
- **The A/B:** with its uplink beacon left running, the msgA landed in slot.
- **The cause:** Kestrel's `StopBeacon` cleared the beacon-TX enable but
  left the TBTT protection `StartBeacon` turns on (`TBTT_PROHIB_EN`,
  `BRK_SETUP`). The port went on holding injected frames to its old beacon
  grid. `StopBeacon` now clears both.
- **Not established:** that 400 × 32 µs of `BCN_HOLD` after the UE's TBTT
  is exactly the ~13 ms observed.

**Rig result** (testrig `scripts/m6_rejoin.sh`).
- **The case:** UE1 joins cell 1 through the RACH (`CELL_UE=0`, an 8 ms RACH
  slot), and DU1 is killed and restarted under it 35 s in. Keyed, as all
  runs now are. 3 runs, all pass.

| KPI (3 runs) | value |
|---|---|
| re-attach, then one re-join | every run |
| second join, after the restarted DU's service start | −4 to +28 sf (median 11) |
| its RACH attempts | 4–8 (the first fall in the restarted DU's acquisition, its grid still moving) |
| uplink of the new id at the restarted DU, from 10 sf after the join | heard 100 %, in slot 100 % |

- **With `CELL_UE_REJOIN_SF=0`** (the counterfactual, 1 run), the UE keeps a
  dead id and the restarted DU never hears it.
- **Regression for the `StopBeacon` change,** all passing:
  - inter-frequency handover 2, plus fallback 1 (each a beacon stop and
    restart);
  - measurement-gap handover 2;
  - clock-fit restart and transient 1 + 1;
  - same-channel handover 1.

## Per-cell keys and the handover context (802.11r-style)

Before this, every cell, DU and UE shared one key (`CELL_PSK`), so whoever
held it could forge any cell. Now each key comes from one master that only
the CU and provisioning hold (`src/cell/Keys.h` gives the derivation and who
holds what):
- **The cell key** `K_c` tags everything that controls cell c's UEs: the
  grant map, the handover IE, the DL body, the UE beacon's own tag, the MR
  and the RACH. A compromised DU or UE of one cell can forge that cell only.
- **The area key** `K_A` tags the neighbour list, the provenance of a
  measured neighbour beacon. With signed neighbour lists (below) a UE no
  longer takes it as proof.
- **The UE's own area key** `A_u = HMAC(K_u, 'V')` tags the second, area
  tag on its beacon (v3). A neighbour DU checks that tag before it counts
  the UE as heard or forwards it for DAPS; before, it took "heard" from any
  frame with the right address.
  - **Delivery:** the CU derives every `A_u` from the homes the DUs report.
    Each plan round it sends every DU the other cells' UEs' keys (F1 v11
    `AreaKeys`), each wrapped under that DU's cell key with a fresh nonce,
    because F1 is authenticated but not encrypted.
  - **Strict:** under per-cell keys a heard UE counts only with its own
    key. One the DU has no key for is not counted.
  - **The consequence of a wrong UE key:** such a UE is invisible to every
    other DU, so coordination cannot keep another cell's UE off its slot.
    On the rig, a UE holding another UE's key, on one channel with a
    handover in flight, lost about 45 % of its uplinks to collisions with
    cell 2's UE. A UE that cannot prove who it is gets no protection that
    rests on its presence elsewhere.
- **The UE key** `K_(c,u)` belongs to the UE's home identity.

**The handover context.** The UE needs the target's key before the switch,
to take the target's grant maps.
- **The CU** (`RAN_MASTER_KEY`) wraps `K_t` for that UE alone, under its UE
  key: a per-handover pad plus a 4-byte key check (`wrap_context`). The
  context is carried in the F1 handover command (v9).
- **The source DU** relays the 24 opaque bytes in its handover IE (v3).
  Neither DU can read them, and nor can the other UEs of the source cell:
  they share `K_s`, not that UE's key.
- **The UE** unwraps them when it accepts the command. It refuses a context
  that is not its own (`cell.ue.ho_no_context`): the key check fails for
  another UE's key, another handover or a tampered context. It verifies the
  target's buffered maps with `K_t`, switches to it at the switch, and goes
  back to `K_s` on a fallback.
- **The CU follows a UE's home** across handovers (`home_of`), so a second
  handover still wraps for the UE's own key.

**A RACH-joined UE's key, and where homes come from.** A joiner has no home
(cell, UE) identity: its id comes from the join. Its UE key is therefore
bound to the MAC it joins with (`CELL_JOINER_MAC`, or derived from host and
radio).
- **The key:** `K_j = HMAC(master, 'J' mac)`, provisioned out of band like a
  SIM's (`CELL_UE_KEY`).
- **Homes are the DUs' record:** a DU records every UE's home when it admits
  it. A static UE gets its (cell, UE), a RACH joiner the authenticated MAC
  it joined with, and a handover arrival the home its handover command
  carries (F1 v10 `HoCommand::home`). The DU reports the homes of exactly the
  UEs a report names, under the same lock (`Report.homes`).
- **The CU keeps none of it:** it wraps each context for the home in the
  *source* DU's current report. A restarted CU, or an id the DU has reused
  for another UE, cannot select a stale home; every admission writes its
  own. With no home reported there is no context, never a guessed key.
- **A false MAC gains nothing:** a UE that joins claiming another's MAC
  only gets contexts it cannot unwrap.
- **The datagram bound:** both F1 receive buffers are `kMaxDatagram`
  (8192). A report at every list bound is 3675 bytes, which the 2048-byte
  buffers before this could already not hold.

Without `CELL_AREA_KEY` all of this collapses to the one shared key, as
before.

**Rig result** (testrig `scripts/m6_cellkeys.sh`, the testrig scripts
deriving every role's keys from one master; all pass).

| case | runs | result |
|---|---|---|
| handover under per-cell keys | 2 | context in every command (home 1:1); switch, target in slot 99.8 %, `ran.ho_done`; 0 maps refused |
| a UE holding cell 2's key in cell 1 (a cell-2 insider forging cell 1) | 2 | 0 of 879 maps taken, never heard in slot; cell 2 unaffected |
| a UE holding another UE's key | 2 | all 7 commands refused (no context), no switch, DU1 kept it (heard 100 %, in slot 99.8 %) |
| coordination, "heard" from area-tagged UE beacons | 1 | `ran_coord_analyze` pass; both cells' UEs heard by the other DU |

**Signed neighbour lists, per neighbour DU.** An HMAC tag a UE can check is
one it could forge: with the area key, any UE could fake any cell's
neighbour list, and with it which beacons count as that cell's. So each DU
also signs its list with its own Ed25519 key (`src/cell/MeasGap.h`).
- **What a DU airs:** its NL, an NS IE (the signature over the exact NL IE
  bytes, 64 B) and an NC IE (its public key with the area CA's signature
  over it: a certificate, 96 B). They ride every beacon; the budget check at
  start counts them.
- **What a UE holds:** only the CA's *public* key (`CELL_CA_PUB`). It
  checks CA → cell key → NL (`NlVerifier`) and can forge nothing.
- **Caching:** the UE verifies a certificate once per key and a list once per
  content, then caches the exact verified bytes per cell. A beacon repeating
  them costs a byte compare.
- **Provisioning:** the DU signing seeds and the CA seed derive from the
  master (`Keys.h`, two HMAC-SHA1 outputs to 32 bytes). The rig's Python
  provisioning (testrig `scripts/cell_keys.py`) matches the C++ derivation
  and OpenSSL's Ed25519 byte for byte.
- **Replay** of a genuine signed beacon is the residual of any provenance;
  it has its own section below.

The rig result (testrig `scripts/m6_nlsign.sh`, per-cell keys and
signing):
| case | runs | result |
|---|---|---|
| both DUs certified | 2 | each UE reports the other cell in every measurement report (865–869 each); coordination passes |
| DU2 holds the area key and every cell key but signs with a key the CA never certified | 2 | no UE ever counts a beacon of cell 2, not even its own UE (0 %); DU2 still serves that UE through its cell key, in slot 99.8 % |

With signing on, the per-cell suite passes:
- measurement-gap handover (the gap targets come from the UE's signed
  serving list);
- the emulated crossing;
- DAPS;
- phase planning;
- the per-cell key set;
- joiner keys;
- per-UE area credentials.

A shared-key handover passes too.

**Per-UE area credentials** (testrig `scripts/m6_ueareakeys.sh`).
Two co-channel cells under per-cell keys; all runs pass.

| case | runs | result |
|---|---|---|
| each DU counts the other cell's UE as heard (plan edges 1/1, 2/1) from per-UE area tags | 2 | both edges; `ran_coord_analyze` pass |
| UE1 holds the area key but a wrong UE key (so a wrong `A_u`) | 2 | DU2 never counts it as heard (no edge 1/1; UE2's 2/1 stays); DU1 still serves it, in slot 99.7–99.8 % |

DAPS now forwards only by the handing-over UE's own area key: 0 uplinks
missing, against 15 without DAPS. A source failure mid-crossing missed 0
in the window, against 10 without. Joiner handover, the re-join and a
shared-key handover also pass.

**A joined UE's handover** (testrig `scripts/m6_joinerkeys.sh`).
UE1 joins cell 1 through the RACH with a fixed joiner MAC, holding that
MAC's `K_j`, under per-cell keys. It is handed over by the A3 at a −2 dB
offset. All runs pass.

| case | runs | result |
|---|---|---|
| the joiner hands over | 2 | joined as UE 2; the command's context is for its MAC (home `J:02cefeaabb01`); switch, target in slot 99.9 %, `ran.ho_done` |
| the joiner holds another MAC's key | 2 | every command refused (6–7, no context), no switch; DU1 kept it (heard ≥ 99.9 %, in slot 100 %) |
| a crossing there and back, the CU killed and restarted between the two handovers | 1 | 1 → 2 → 1; the restarted CU, knowing nothing, wrapped the return for the joiner (home `J:02cefeaabb01`, from DU2's report of the home the first command carried) |

Regression for F1 v10: the static per-cell key set (handover, forge,
isolate, coordination) and the re-join after a DU restart under per-cell
keys both pass.

**The M6 suite under per-cell keys** (`M5_MASTER`, 1 run of each; all pass):
- inter-frequency handover and fallback;
- DAPS;
- measurement-gap handover (the neighbour beacon's provenance now from the
  area key);
- phase planning;
- the crossing and its source failure;
- clock fit across a DU restart, and the re-join after one.

One shared-key handover also passes, so the old mode is unchanged.

The first restart runs failed, but in the rig script: the restarted DU was
started without its keys, and the keyed UE rightly refused its 546
unauthenticated maps. The script fix was to start it with them.

## An emulated crossing (nothing forced)

Every earlier handover here had its trigger forced or was a standing
difference between two cells. Here the UE crosses. Both cells are on one
channel, with one UE (UE1, starting in cell 1).
- **The emulation** (`CELL_DU_TX_RAMP`): the DUs ramp their TX power
  against each other, piecewise linear on one wall-clock zero
  (`CELL_DU_TX_RAMP_T0`; the DU hosts are NTP-synced). Beacons and DL alike
  follow it.
- **What the UE sees:** it measures both beacons (no gaps on one channel)
  and reports them (`cell.ue.mr`).
- **Who decides:** the CU's A3 at its defaults (offset 3 dB, hysteresis
  1 dB, time-to-trigger 640 ms, dwell 2 s).

**The ramps** (testrig `scripts/m6_crossing.sh`): cell 1 fades by 16 dB over
15 s while cell 2 rises by as much. They hold, then reverse.
- **The UE's measured margin** (the other cell against the serving one)
  sweeps from about −13 to +24 dB and back.
- **The rig's power-to-RSSI curve** is not linear: −8 dB of offset reads
  about −9 dB at the UE, and −9 dB about −12 dB. Its cause is not
  established; the tests use the measured margins, not the offsets.

**Crossing, there and back** (3 runs, all pass: exactly 1 → 2 then 2 → 1 in
each).

| KPI (6 handovers) | min | median | max |
|---|---|---|---|
| UE margin reaching the A3 entry (4 dB) → switch | 40 sf | 48 sf | 55 sf |
| margin at the CU's trigger | 9 dB | 11 dB | 15 dB |
| interruption (first in-slot uplink at the target) | 1 sf | 1 sf | 2 sf |
| uplinks missing at the CU over the run (DAPS) | 0 | 0 | 1 |

- **Where the time goes:** 24 sf of the crossing-to-switch is the
  handover lead (`RAN_HO_LEAD_SF`). The rest is the time-to-trigger plus the
  1 s reporting chain: the UE's MR (`CELL_UE_MR_MS`), the DU's F1 report
  (`CELL_F1_REPORT_MS`) and the CU's A3 round (`RAN_PLAN_MS`).
- **Why the trigger overshoots:** at ~2 dB/s of margin, the margin keeps
  moving during that chain, so the trigger lands 5–11 dB past the entry.
- **Each switch** keeps TSF continuity and is heard in slot at the target
  (≥ 99 %), and the source releases the UE.

**Hover** (3 runs, all pass). The ramps stop with the margin held just under
the entry: median +3 dB, p10–p90 +2 to +4 dB. 13–15 % of the UE's reports
were at or above the entry.
- **Criterion and result:** at most one handover and never a switch back
  (0 handovers in all 3 runs). The time-to-trigger and the filtering never
  let a noise excursion through.
- **Why the criterion allows one:** one handover at a hover is correct A3
  behaviour, not a failure: at +3 dB the filtered margin may hold the 4 dB
  entry for the time-to-trigger. One hover run of an earlier set (the CU fix below does
  not touch the A3) did exactly that, once, and did not switch back. What
  the test must catch is the ping-pong: a switch back.

**The source failing mid-crossing** (2 + 2 runs). DU1 is killed the moment
its UE accepts the handover, with the switch still the 24 sf lead away.
- **The window:** uplinks missing from the CU's stream, from 24 sf before
  the switch to 20 after (the uplink seq is the network superframe).
- **With DAPS:** 0 and 0 missing (asserted ≤ 1). The target delivers the
  UE's uplinks until the switch, and the switch happens.
- **Without DAPS:** 11 and 13 missing (reported). The source's path dies
  with the DU.
- **Outside the window,** with one cell left, 4–5 isolated single losses
  over the rest of the run are that path's own (~0.4 %).

**A CU bug the crossing found.** At first the target heard the UE in slot
only 91–98 % after a switch.
- **The symptom:** the CU re-issued the target's two UE positions in turn,
  every plan round. Each swap cost the moving UE one uplink in its old slot.
- **The cause:** the CU planned from the DUs' *reported* positions. A
  schedule takes effect `RAN_LEAD_SF` (12 sf) ahead, more than the plan
  round, so the report still showed the positions before the last
  schedule. With two equally good layouts, the planner "kept" the stale one
  and reversed itself each round.
- **Why M5 never saw it:** a move only ever went to a strictly better
  position there. Here a handover arrival made two layouts equal.
- **The fix:** the CU plans each UE from its own issued schedule, and from
  the report only for a UE the schedule does not name yet (a handover
  arrival). After it, 8 schedules in a whole run, and every switch in slot.

## Grant-map authentication, and the suite keyed

The grant map IE (v2, `src/cell/GrantMap.h`) carries the cell key's tag
after its CRC, as the NL, MR and handover IEs do.
- **With `CELL_PSK`:** a UE takes only authentic maps. A forged map neither
  moves its slots nor, through the restart path, resets its grant state.
- **Without a key:** an unauthenticated development cell, or a passive
  witness, checks the structure only.
- **v1 maps** (untagged) are refused.
- **Counters:** the UE counts serving maps taken and refused (`gm_ok`,
  `gm_refused` in `cell.ue.done`; the first refusal logs
  `cell.ue.gm_refused`).

**On air** (testrig `scripts/m6_gmauth.sh`, 2 runs, pass). Every role holds
the cell key except UE2, which holds another. To UE2 each genuine map is a
map under the wrong key, as a forged one is to a keyed UE.

| | maps taken | maps refused | heard in slot by its DU |
|---|---|---|---|
| UE1 (cell key) | 586–587 | 0 | 99.8 % |
| UE2 (another key) | 0 | 586–587 | never |

**The M6 suite re-run keyed** (all roles on one `CELL_PSK`; all pass). No
keyed UE refused a map, apart from UE2 above.

| test | runs | result |
|---|---|---|
| same-channel handover + fallback | 2 + 1 | interruption 1 sf; fallback 10 sf |
| inter-frequency handover | 2 | interruption 2–3 sf, retune 23.6 ms |
| DAPS (source killed mid-crossing) | 2 + 2 | missing uplinks 0 / 0, against 11 / 16 without |
| measurement-gap handover | 2 | deaf 54.1 ms, A3 fired at defaults |
| phase planning | 2 | DU2 at 51200, both UEs 57 gaps heard |
| clock-fit jump + DU restart | 1 + 1 | restart followed and re-attached; transient ignored |

## DAPS: the duplicated uplink (same channel)

**Common sequence.** A UE uplinks once per superframe. Every DU that hears
one transmission hears it at the same instant on the shared PTP timebase,
so it numbers the copy by its reception time in network superframes,
`floor((rx_tsf + phase + net_off·P) / P)` (`ul_seq_from_network_time`).
- Nothing in the frame sets the sequence, so a forged or replayed frame
  cannot advance the CU's dedup window and suppress real copies. At worst it
  is one more copy of "now".
- The UE needs no change.
- An earlier revision used the beacon's timestamp. The tag doesn't cover the
  timestamp, so it was open to exactly that suppression.

**Forwarding.** Each DU forwards its own UEs' uplinks to the CU (F1
`Uplink`) whenever F1 is on, on a non-blocking socket, so the radio RX path
never waits on the network; drops are counted. With `CELL_DAPS=1` (the default), during a
handover it also forwards the UE's *other* identity:
- the target forwards the UE's source identity, staged with the command,
  until the switch plus the release window;
- the source forwards the UE's target identity after the switch.

A copy heard under the other identity is forwarded only if its tag verifies.
The rig's cells share one network key; per-cell keys would need the other
cell's context pre-staged, which is open.

**Delivery.** The CU delivers each uplink once per UE stream (`UlStreams`
over `SeqDedup`). The command *aliases* the target identity onto the
source's stream, so both identities' copies dedup together and the stream
runs on across the switch.
- **Gaps:** a gap in a stream's delivered sequence is logged (`ran.ul_gap`),
  one event per run of missing uplinks. A run is reported once its last
  uplink is 32 behind the newest, so a late copy recovers it silently. It is
  the interruption a crossing or a lost path costs.
- **Accepted identities:** the CU only takes identities a configured DU
  reports, or that an active handover aliased.
- **Cleanup:** idle streams retire after 60 s. A failed, timed-out or
  cancelled handover drops its provisional alias, and a completed one drops
  the source identity after its retirement.
- A summary goes out every 10 s (`ran.ul`: delivered, duplicates, and which
  DU delivered first).

**Rig test** (testrig `scripts/m6_daps.sh`, 3 runs per mode). The source DU
is killed the moment its UE accepts the handover, so the path dies
mid-crossing about 2 s before the switch.

| uplinks missing from the crossing UE's stream | min | median | max |
|---|---|---|---|
| DAPS (`CELL_DAPS=1`) | 0 | 0 | 0 |
| no DAPS (`CELL_DAPS=0`) | 14 | 15 | 18 |

These are the final-code runs. An earlier pair of runs, on the
egress-timestamp sequence, gave 0/0/1 and 13/14/14.

- **Without DAPS:** the stream loses everything from the kill to the switch,
  and resumes via the target once the UE uplinks under its target identity.
- **With DAPS:** the target had been delivering the source-identity copies
  all along. In one representative run, after the source died the stream
  went on entirely via the target: of 1156 delivered, 31 came first through
  the source and 1125 through the target.

## DAPS: downlink duplication

Until now the DL data was each DU's own (HARQ frames with nothing above
them), so nothing ran on across a handover to lose. The downlink now has a
**stream at the CU**, like the uplink's:

- **The CU originates it** (`RAN_DL_MS`): one PDU per UE every 100 ms on the
  rig, numbered in the UE's stream (`sn` from 1), sent to its serving DU over
  F1 `Downlink` (v12).
  - The stream has an **epoch**: the CU's realtime clock when it created the
    stream. A re-created stream (a re-joined UE, a restarted CU) is always
    newer.
- **The DU queues it** per UE (64 deep, oldest dropped) and airs up to 8
  sns per DL frame, inside the tagged body, with their epoch.
  - It takes a PDU only for a newer epoch, which replaces the queue, or for
    a newer sn in the same epoch. A replayed datagram cannot displace live
    ones; a reordered one is lost.
  - A UE admitted under an id gets fresh DL state, and a joiner never
    inherits a queue left under its id.
  - A HARQ retransmission re-airs the same sns.
  - The DL frame is a fixed size (protocol v4, 41 bytes more than before).
  - A forged sn cannot make the UE drop a real PDU as a duplicate.
- **The UE delivers** each sn once (`DlStreamRx`, `src/cell/Handover.h`),
  whichever DU it came through.
  - The sn space runs on across switches. A newer epoch starts over, and
    what was still pending of the old stream is counted lost; an older epoch
    is ignored.
  - A run of missing sns is a gap once it is 32 behind the newest; a late
    copy within that recovers it.
  - Gaps are logged as `cell.ue.dl_gap`, and the totals appear in
    `cell.ue.done` (`dl_pdus`).
- **The stream follows the UE:** at `ran.ho_done` the CU moves it to the
  target identity (`ran.dl_moved`).

**Duplication** (`RAN_DL_DAPS=1`, the default):
- From the handover command until `ran.ho_done`, the CU sends every PDU to
  the target too, for the UE's target identity.
- At the command it also sends the stream's last 16 PDUs
  (`RAN_DL_DAPS_BACKLOG`; data forwarding). A link fading toward the command
  loses PDUs at the source before any copy exists. The emulated crossing
  showed it: without the backlog, 6 and 9 PDUs were lost on the return
  handover, all first aired on the fading source just before the command.
- The target queues them for the UE it has only staged, and airs them from
  its first DL frame after the switch, oldest first.
- The UE drops what it already had from the source as duplicates.
- Without duplication, PDUs go to the source until `ho_done`: what the
  source holds or is sent after the UE left is lost.

**Rig result** (testrig `scripts/m6_daps_dl.sh`, all 16 runs pass). UE1
goes from cell 1 to cell 2 on one channel. In the forced cases the A3 fires
at a −2 dB offset; in the fault cases the source DU is killed the moment
UE1 accepts, ~2 s before the switch. The crossing is the emulated one (TX
ramps, the A3 at its defaults, there and back: two handovers per run).

| UE1's DL PDUs missing in the handover windows | DAPS | no DAPS |
|---|---|---|
| source killed mid-crossing (3 runs) | 0 / 0 / 0 | 21 / 23 / 33 |
| plain handover (3 runs) | 0 / 0 / 0 | 8 / 18 / 19 |
| emulated crossing, 2 handovers per run (2 runs) | 0 / 0 | 57 / 62 |

- **The window** of a handover runs from the stream's sn at the command,
  less the 16-PDU backlog, to where the stream moved at `ho_done`, + 8. The
  CU logs both points (`ran.dl_cmd`, `ran.dl_moved`). A random HARQ give-up
  elsewhere in the run is not the handover's.
- **Whole run:** DAPS lost 0 except one crossing, which lost 6 PDUs just
  before its return handover's window (sns 576–582). They were first aired
  on the fading source more than 16 PDUs (1.6 s) before the A3 fired, so
  the backlog did not reach them. A longer backlog would, at the cost of
  more duplicates.
- **DL silence at UE1** (ms between frames that delivered a new PDU;
  min / median / max over the case's runs):

  | case | DAPS | no DAPS |
  |---|---|---|
  | source killed | 1447 / 1755 / 2062 | 2062 / 2369 / 3294 |
  | plain | 525 / 526 / 731 | 833 / 1857 / 1857 |
  | crossing | 3561 / 3912 / 4262 | 3036 / 3240 / 3444 |

  DAPS recovers the PDUs, not the time. While the source link fades or is
  dead, nothing new arrives until the target airs its buffer after the
  switch.
- **The cost:** DL duplicates per run were 58–104 with DAPS against 36–52
  without. The target holds ~40 PDUs at the switch and drains them at 8 per
  frame.
- **The rest of the handover**, every run:
  - one switch (two in the crossing), no fallback;
  - no re-association: no RACH, re-join or re-attach by UE1;
  - TSF continuity: the switch 2–200 ms after the network instant, under
    2 P;
  - the target heard UE1 in slot ≥ 99 % after each switch;
  - control-plane completion (`ran.ho_done`) 5–17 superframes after the
    switch instant;
  - the static UE2 lost no PDU;
  - with DAPS, the uplink through the target lost 0 uplinks (12–15 without,
    when the source died).
- **Not duplicated:** the source does not keep airing to the UE after the
  switch, and the UE takes DL frames only from its serving cell.

## DAPS across channels (sequential, uplink)

Uplink DAPS needs the target DU to hear the UE's uplink under its source
identity. Across channels it cannot: the UE has one radio, tuned to the
source. With the source killed mid-handover, every uplink from the kill to
the switch was lost (the three-cell crossing: 12–14).

**The visit** (`examples/cell`, `CELL_UE_DAPS_VISIT` [1]):
- **When:** between accepting an inter-frequency handover and its switch,
  the UE visits the target's channel once a superframe, through the one
  before the switch.
- **What it does:** after its own uplink slot, it retunes to the target's
  channel and airs a copy of its uplink beacon there, at an instant clear of
  the target cell's active span. It then retunes back before its serving
  beacon.
- **The window:** it is the idle part of the UE's own grid, so the source
  link loses nothing. Its own hardware beacon fires at its uplink slot,
  outside the window.
- **Holding the channel:** `send_packet` only queues the frame, so the UE
  holds the target channel 4 ms after queuing it. There is no per-frame TX
  report on Kestrel; without the hold, 9 of 10 copies were lost.
- **Loop cadence:** during the window the UE loop runs every 30 ms, not
  100 ms, so the next superframe's visit is in reach after one ends
  ~75 ms into its superframe.
- **What a visit checks:**
  - the whole send, from queuing through the hold, must be clear of the
    target's active span;
  - a late wake skips the visit;
  - an outbound retune that overran the instant skips the send and retunes
    straight back (`cell.ue.daps_visit_skip`);
  - both retunes feed the adaptive bound.
- **The step hold:** a visit holds TSF steps only through its own
  superframe; steps held are counted in `cell.ue.done`.

**Its sequence number at the CU** (F1 v14):
- The target DU forwards the copy as before (`daps_ids`, area tag), now with
  its reception instant on the network timebase (`Uplink.net_us`).
- The CU maps EVERY forwarded uplink to the UE's last own uplink slot at or
  before it (`Handover.h` `ul_seq_for_copy`), the serving DU's own copies
  included. It uses the identity's last-known slot (its cell's phase and its
  uplink position from the latest report naming it, kept 30 s).
- Mapping every copy keeps both copies of one uplink on one seq even with
  the slot next to a network-superframe boundary, where a timing advance
  puts the serving copy before it. A copy arriving after the source released
  the UE still maps.
- The visit, sent ~30 ms after the slot and possibly in the next network
  superframe, therefore dedups with the source's copy of the same uplink.
  The UE needs no global clock.

**The switch clears the TSF step hold.** A hold set after a gap or a visit
counts in the source's superframes. A visit just before the switch carried
it ~9 superframes into the target's count, so the UE's uplink stayed 40 ms
off. The target's TA loop took that for propagation delay, and it took
~12 s to settle.

**Rig result** (testrig `scripts/m6_daps_xch.sh`): 2 runs per case, all pass. Cell 1 is on ch149 and cell 2 on ch157 (phase
60000); the A3 at its defaults hands UE1 over 1 → 2, with the CU's DL
stream on.

| uplinks missing at the CU around the switch | before the switch | at the switch |
|---|---|---|
| source killed at the accept, visits | **0, 0** | 1, 2 |
| source killed at the accept, no visits | 15, 15 | 2, 2 |
| no fault, visits | 0, 0 | 2, 1 |

- **20–21 visits per handover**, every one sent and back, none skipped;
  deaf time 55.6–56.8 ms each.
- **They cost the source nothing:** without a fault, DU1 heard UE1's uplink
  in every superframe from the accept to the switch, in slot 100 %.
- **The switch's own cost:** the switch superframe (with the source dead,
  nobody hears it) and the next. The first uplink after the UE's beacon
  restart on the target is lost, as after a suspend gap; why is not
  established.
- **Every run:** the target heard the UE in slot 99.9 %, and DL DAPS lost no
  PDU.
- **What the first runs found:** besides the TX hold, the loop cadence and
  the step hold (above), visits stopping two superframes before the switch
  left those two uncovered.
- **Regression:** the inter-frequency measurement-gap handover, same-channel
  DAPS, the plain and fault handovers, the two-cell crossing with a source
  failure, and the three-cell crossing with one all pass.

### The downlink across channels: the target answers each visit

Before the switch the UE takes DL only from its serving cell. A dead source
therefore left it without DL from the kill to the switch, though the
target had been holding every PDU since the command (DAPS DL).

**Each visit is answered** (`examples/cell`):
- **When:** the target DU hears a visit copy from a UE it has staged for an
  inter-frequency arrival, before the switch.
- **What:** it answers at once with the queued DL PDUs no answer has offered
  yet, up to 8. The frame (`build_visit_dl_frame`) is broadcast-addressed,
  so nothing waits for an ACK and there is no contention-window escalation.
  It is tagged with the target cell's key, and the body names the UE's
  target identity.
- **The UE** (still holding the target's channel for 4 ms) takes the PDUs
  into its DL stream (`cell.ue.visit_dl`).
- **The queue keeps everything:** nothing ACKs an answer, so after the
  switch the target still airs the offered PDUs. It now sends the
  not-yet-offered ones first, and the UE takes a late copy within its
  reorder window. Without that ordering, the newest PDU waited behind ~5
  frames of PDUs the UE already had: 0.53–0.63 s of DL silence at the
  switch.
- **Only across channels:** the target answers only when its neighbour list
  puts the source on another channel. On one channel it hears every uplink
  of the UE, and an answer there would land in the source cell's schedule.
- **Sending:** the answer is aired by its own thread, never from the radio's
  RX callback. On the rig, sending from inside it froze the DU (the RX loop
  is what recycles the TX pages).

**Rig result** (testrig `scripts/m6_daps_xch.sh`): 4 runs per case, all pass.
- **The trigger:** the handover is triggered by the CU's A3 at its defaults
  on the UEs' own measurements, with the UEs static: cell 2 is simply the
  stronger at UE1's radio. It is not the emulated crossing (see "An
  emulated crossing").
- **The layout:** the same as the uplink runs (cell 1 on ch149, cell 2 on
  ch157), with the CU's DL stream on.

| per run (min / median / max over 4) | DL gap across the switch | longest DL silence | uplinks lost before the switch |
|---|---|---|---|
| source killed at the accept, visits answered | 225 / 225 / 226 ms | 225 / 225 / 226 ms | 0 / 0 / 0 |
| source killed at the accept, no visits | 1497 / 1549 / 1805 ms | the same | 10 / 10.5 / 14 |
| no fault, visits answered | 225 / 225 / 226 ms | 225 / 225.5 / 307 ms | 0 / 0 / 0 |

The DL gap across the switch runs from the last new PDU before the switch
to the first after it.
- **Answers:** 18–20 per handover, none expired; DL DAPS lost no PDU in
  any run.
- **The bookkeeping** (from the review):
  - An answer that is not queued, expires (3 ms after the copy that
    prompted it) or fails gives its PDUs back: the offered mark rolls back.
  - After the switch, `pick_dl_pdus` sends not-yet-offered PDUs first, but
    every queued offered PDU that the fresh one would push out of the UE's
    reorder window goes before it. A lost answer's PDU then reaches the UE
    inside its reorder window (selftests: a lost answer behind a 60-PDU
    backlog; a 40-PDU backlog with every answer lost, through DlStreamRx).
    (This rule is from the switch-interruption change below; the first
    version, "before it falls 24 sns behind", let a large offered backlog
    lead the queue.)
- **What is left** is the switch's own pause: ~2 superframes (225 ms). The
  UE leaves at the instant, retunes, and the target's grant takes effect
  from its next superframe.
- **Same channel:** the target sent no answers, and same-channel DL DAPS
  kept its silence (526 ms plain, 1754 ms with the source killed).
- **The first runs found:**
  - The DU froze when it sent from inside the RX callback; the answer
    thread fixed that.
  - The target replayed its whole buffer after the switch: 0.53–0.63 s of
    silence. Fixed by sending not-yet-offered PDUs first.
  - The first answers included same-channel handovers, where an answer would
    land in the source's schedule; they are now restricted to inter-frequency
    arrivals.
- **Regression:** the uplink results are unchanged (0 lost before the
  switch with visits). Same-channel DL DAPS, the three-cell crossing with a
  source failure, and the plain and fault handovers all pass.

### The switch's own interruption (inter-frequency)

With the visits, the only losses left were at the switch itself. A plain
inter-frequency handover cost the UE ~2 superframes of DL (225 ms) and
1–2 uplinks. The timeline (rig, cell 2's phase 60 ms after cell 1's):
- **The switch:** the UE switches at the instant, on time (0.07 ms into the
  source superframe, 42 ms into the target's), and retunes in 24 ms.
- **The target's grant** takes effect from its next superframe, 60 ms
  after the instant; its first DL frame there is received.
- **The uplink beacon** was restarted only by the UE loop's NEXT pass, 100 ms
  later, past that superframe's uplink slot. That uplink was lost.
- **The next uplink** fired 40.7 ms off the slot: at the source's phase.

**Kestrel's beacon timer.** The first beacon after a large TSF step fires
at the OLD phase, whether the step comes before the beacon's restart or
right after it. The engine evidently schedules each TBTT from the previous
one and takes a TSF write only from the TBTT after next (a hypothesis that
fits every observation). The UE now predicts where that beacon lands, at
the target slot + the step: 57.2 ms predicted, 57.3 observed. This is
also the "one uplink out of slot per switch, about the phase difference
off" of the three-cell handover.

**What changed** (`examples/cell`, `tools/ran-controller`):
- **The pass after a switch** is scheduled for the new cell's first
  superframe (+1 ms), not a full loop period later.
- **The TSF is placed for the target's uplink slot** right after the
  switch's retune, while the beacon is stopped (`cell.ue.switch_place`), so
  every beacon from the second one is in the target's slot.
- **The stale-phase beacon** is harmless in the target's idle time, and
  there it is heard and forwarded. If it would land in the target's active
  span, the UE holds the restart until that TBTT has passed: one uplink
  given up rather than a collision. The beacon restarts no earlier than
  the expedited pass, so the stale TBTT is the first at that phase from
  then. A failed clock read or write keeps the placement pending: it is
  retried before the restart (3 tries, then the ordinary step corrects it a
  pass later), and the expedited pass itself survives a failed clock read
  (retried 2 ms later). On the rig the guard held 5 of 18 switches, all in
  three-cell layouts.
- **The CU** records the target identity's uplink slot from the handover
  command. The first uplinks after the switch come before the target's
  report names the UE; before this, they were mapped to the wrong network
  superframe, and the switch superframe counted as lost.
- **The A3 of a UE in flight is frozen** until its handover concludes. The
  UE leaves at the instant and the source's reports lose its own
  measurements; with a slow conclusion (the target's report naming the UE
  up to ~15 superframes later, as before), the DUs' stale uplink evidence
  cleared the candidate and raised a second one for the same move (no
  second command, but a wrong record). A flight that ends without the move
  (lost DU, timeout) restarts that A3, so a new candidate needs its own
  time-to-trigger.

**Rig result** (testrig `scripts/m6_daps_xch.sh`): 4 runs per case, source cell
on 149, target on 157, phase difference 60 ms, a CU DL stream of one PDU per
100 ms. "Before the switch" counts uplinks lost in the window before it; "DL
gap" is the last DL PDU received from the source to the first from the
target.

| case | UL lost before the switch | DL silence (ms) | DL gap across the switch (ms) |
|---|---|---|---|
| source dies, visits | 0 / 0 / 0 / 0 | 205 (all 4) | 122–123 |
| source dies, no visits | 10–16 (median 13.5) | 1498–2010 (median 1805) | the same |
| source alive, visits | 0 / 0 / 0 / 0 | 205 (all 4) | 123 (all 4) |

Before this change the gap across the switch was 225 ms with one run at
327: the target's queue after the switch led with its large offered
backlog, so the first fresh PDU waited. Now a DL frame takes the
not-yet-offered PDUs first (`pick_dl_pdus`). Only the offered PDUs that the
fresh one would push out of the UE's reorder window go before it (the
answers may have been lost: the offered mark is not the UE's top), so a
lost answer's PDU is still a late copy, not a confirmed gap. (The queue
after a switch spans ~41 sns on the rig; those out-of-window PDUs ride the
fresh one's frame — see *The DL cadence at the switch*.)
Uplinks lost in the whole window, switch superframe included: 0 in 7 of the
8 visit runs, 1 in one.

Re-run on the final build (review fixes; `cell.dl` now logs, after visit
answers, a frame's first sn, how many of its PDUs were not yet offered, and
the offered mark), 3 runs per case: gap across the switch 123 ms in 4 of
the 6 visit runs, 225 in 2. In all four 123 ms runs the first frame after
the switch carried the next fresh PDU; in both 225 ms runs the target's
queue held no fresh PDU yet when that frame was built (31 PDUs, all
offered — the CU sends one per 100 ms), and the next frame led with it.
Not queue order — but not the PDU cadence either, as first read: see
*The DL cadence at the switch* below. Over all the runs on this branch's
builds, 18 of 24 visit runs were at 122–123 ms. No UL was lost before the
switch in any visit run.

### The DL cadence at the switch

The remaining 225 ms switch gaps (about 1 run in 4) had three causes, each
found with timestamps on the target DU's clock (`cell.dl.pdu_in`: when
each PDU reached a DAPS target's queue; `cell.dl` `pick_ms`; the answers'
`t_ms`):

- **The DU took a frame's PDUs a superframe early.** The DL loop built the
  frame for superframe N right after sending N-1's, then slept to its slot:
  a PDU arriving in those ~100 ms waited for N+1. Now the frame is picked
  and built `CELL_DL_PICK_LEAD_US` (3 ms) before its slot. DL send lateness
  unchanged (p50 3 µs, p99 8 µs).
- **The CU sent its PDUs up to 50 ms late.** Its loop polled with a fixed
  50 ms timeout and sent a due PDU only when something woke it, so PDUs
  reached the DUs unevenly (one >119 ms after the previous). The poll now
  also wakes for the next PDU: spacing at the target 99–102 ms in every run.
- **The fresh PDU did not fit the frame.** At the switch the target's queue
  holds the command's backlog (16) and every PDU since (~25): ~41 sns. Every
  queued PDU more than the reorder window behind the fresh one goes first
  (review fix above) — up to 8 of them, the whole 8-PDU frame, and the
  fresh one waited a superframe (seen directly: it had arrived 51 ms before
  the pick). A DL frame now carries up to 32 PDU sns (fixed length: +96
  bytes, ~32 µs at 24 Mb/s), at least the queue's (64) out-of-window
  maximum plus the fresh one (a `static_assert`). The DU's queue takes any
  newer sn, so with gaps every slot could be out-of-window: `pick_dl_pdus`
  keeps the first fresh PDU's place in any case (from the review).
- **A frame ready too late is not aired.** The pick and build now run inside
  the lead; a frame whose submission would start more than
  `CELL_DL_GUARD_US` (500 µs) into its slot is skipped (`cell.dl.late_skip`),
  its PDUs and HARQ attempt given back. DL send lateness over the runs:
  p99.9 12–13 µs before and after; single outliers of 319 µs (before this
  change) and 264–373 µs (after), cause not established.
- **Found by that skip: a DU served its UEs in grant order, not slot
  order.** With two UEs (a handed-over UE at position 1, the DU's own at
  position 2; later 2 and 3), the UE whose slot comes first was served
  after the other's slot. Every one of its frames went out ~4.1 ms late,
  inside the other UE's slot (404 of 404 in a `m6_daps_dl` crossing run on
  the code before this change). With the skip they were dropped (440 per
  run), and that crossing lost 5–6 uplinks of the arriving UE where it had
  lost 0–1; why dropped DL frames cost uplinks was not established. The DU
  now serves in DL-slot order. On this build the `m6_daps_dl` crossing (DAPS, 2 runs) lost 0
  uplinks and 0 DL PDUs, and its longest DL silence was 0.39 and 1.03 s,
  where the table above had 3.6–4.3 s. Those earlier runs aired every frame
  to the arriving UE ~4 ms late; that this was the silence is likely, not
  established. The 1.03 s ends before the command (the source fading before
  the A3 fires); the 0.39 s is at a switch.

**Rig result** (testrig `scripts/m6_daps_xch.sh`): 4 runs per case (cells 149 → 157).

| case | UL lost before the switch | DL gap across the switch (ms) | first target frame |
|---|---|---|---|
| source dies, visits | 0 (all 4) | 122, 122, 123, 123 | fresh PDU in all 4 |
| source dies, no visits | 12–15 | 1395–1703 | — |
| source alive, visits | 0 (all 4) | 122, 123, 123, 166 | fresh PDU in all 4 |

The 166 ms run: the source's own DL frame had delivered the newest PDU
43 ms before the last answer (which then had nothing new), so the gap
counts from there; the target's first frame carried the next one. Each
first target frame carried 32 PDUs. Before these fixes 6 of 24 visit runs
were at 225 ms. The longest DL silence elsewhere in a run reached 307 ms
in two runs, not at the switch; its cause was not established here. Regression passes: measgap,
three_ho, crossing3, handover, daps, daps_dl, crossing.

## Replay of a signed neighbour list

A signature proves who wrote a beacon, not when it was aired, so the
residual of the signed lists is a genuine beacon recorded and re-aired: a
ghost of a cell that is no longer there (or never was here), which a UE would
measure and the CU could hand over to. Two checks make a re-aired copy
useless unless it is relayed live and on time (`src/cell/MeasGap.h`,
`examples/cell/main.cpp`).

- **A signed window.** The NS IE (version 2) carries `win`, the network
  superframe / `kNlWindowSf` (64, ~6.5 s at 100 TU), inside the signature
  (`'S' || NL IE || win`). The DU re-signs at every window change, and signs
  only once it knows the network superframe. The UE takes its serving
  cell's window as "now" and refuses a serving window that goes back. It
  takes a neighbour's beacon only if the neighbour's window is within one of
  the serving window, i.e. only if it was signed in the last ~13 s.
- **The phase.** All DUs share the PTP timebase, and the serving list names
  each neighbour's TBTT offset on the serving grid. The UE keeps the phase
  where its serving beacons land (on its fitted DU clock) and refuses a
  neighbour beacon that lands more than `CELL_UE_NBR_PHASE_US` (300 µs) from
  serving phase + offset. A copy re-aired off the cell's own TBTT fails this
  even inside the window.
- **Unlisted.** A neighbour the serving list does not name is not measured
  either.
- **The references cannot be dragged:**
  - A serving copy off the reference phase by more than the tolerance is
    refused, not taken as the new reference. The reference follows genuine
    beacons only slowly (1/8 of the error per beacon).
  - Only 20 off-phase serving beacons in a row, with none on the reference,
    re-take the reference; that covers a cell that really moved.
  - A serving window that has not advanced for two windows' time is no
    "now": neighbours are then refused as stale.
  - At a cell switch the new serving cell's first window may be one behind
    the old cell's, and no more.
- **The DU signs only what it airs:**
  - A beacon carries a signature only for its current list and window; a
    failed re-sign airs none rather than an old one.
  - A signing DU without a network superframe (no `CELL_REF_PTP`, or sync
    lost) airs its list unsigned. It warns at start and logs
    `cell.nl.unsigned_no_net` / `cell.nl.signed_again`.

Refusals are logged per reason (`cell.ue.nbr_refused`, counted in
`cell.ue.done`). `CELL_UE_NBR_FRESH=0` turns the checks off, which the rig
uses for the counterfactual.

**What remains** is a real-time repeater placed on the right phase: it
relays the live beacon within the tolerance, so it is the real cell, only
louder. Only distance bounding would stop that.

**Costs.** A neighbour is measured only once the serving phase is known. On
the rig that is about the first 16 neighbour beacons after the UE starts or
switches. A UE whose serving DU is gone measures no neighbour until it
re-attaches. With no serving cell, its measurement reports have nowhere to
go anyway.

**Rig result** (testrig `scripts/m6_replay.sh`). Per-cell keys and signed
lists, both cells on one channel. The witness radio runs the test-only
replayer role (`CELL_ROLE=replayer`): it records cell 2's genuine DU
beacons for 45 s, then re-airs the last one every 100 ms. DU2 is killed at
50 s, so from then on only the replayed beacon of cell 2 is on air.

| case | runs | UE1's reports with cell 2: early (genuine) | last 200 (after DU2 is gone) | refused (stale / unlisted / phase) |
|---|---|---|---|---|
| checks on | 2 | 100 % | **0 %** | 562–618 / 0 / 86–103 |
| checks off (`CELL_UE_NBR_FRESH=0`) | 2 | 100 % | 100 %: the ghost is measured | — |

The replayer sent 1049 copies in every run. With the checks on, the ghost
is refused first on phase (its window is still current), then as stale.

Regression with the checks on, 1 run of each, all pass:
- per-cell keys with signing (`M5_MASTER`): measurement-gap handover, the
  emulated crossing, phase planning, handover with and without a fault, and
  DAPS;
- the per-cell key set, joiner keys and per-UE area credentials (their own
  scripts);
- a shared-key handover.

Apart from the replay runs, refusals were only the ~16 at the start of each
UE (before its serving phase was known) plus the dead-serving case above.

## Bugs the rig found

**A DU with no UE did not pace its main loop.** Its superframe count ran away
from the TSF (186 → 3.5 M), and so did the network superframe it reported.
Every M1–M5 run had a static UE, which hid it. The loop now waits for its
superframe when there is nothing to serve.

**The neighbour phase reference was cleared every superframe.** A brace
missing around the "no list heard lately" reset in the UE's gap block wiped
the serving phase each loop. Most genuine neighbour beacons were then
refused on phase (781 of ~865 for one UE). The first regression of the
replay checks caught it in the signed-list run: UE2 reported cell 1 in only
74.8 % of its reports. With the fix, 99.9 %.

**Suspend gaps lost to the UE loop's own timing.** The UE loop wakes every
100 ms; a gap blocks a pass for its wait plus ~54 ms deaf.
- **Silent misses:** in one of the first three-cell runs, UE2's idle gaps to
  cell 1 always fell in the superframe before its suspend superframe. Each
  ended so late that the next pass came after the suspend gap's start, and
  that window was missed silently: 1 suspend gap in a run, and 2→3 never
  measured.
- **Half the idle gaps:** a pass that had slept through a suspend gap went
  on to schedule an idle gap from its stale superframe count, already in
  the past. That consumed it, halving the idle rate.
- **What changed:**
  - an idle gap that would run into the next suspend gap is deferred a
    superframe;
  - a pass that ran a suspend gap skips the idle block;
  - an idle gap whose start went by is retried;
  - a suspend superframe gone by without its gap is logged
    (`cell.ue.suspend_missed`): every one since the last accounted for, or
    one aggregate after a long pause;
  - only a gap that ran consumes its suspend superframe; a failed one (late,
    the beacon would not stop) is left to that log.

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

- **DAPS across channels, beyond one visit per superframe:** the uplink and
  the downlink are duplicated through the visits, one per superframe from
  the accept to the switch. A UE whose source fades before the command still
  loses what it misses before the visits begin.
- **More than three cells, and a full tour:** the rig runs three cells,
  with handovers among them and a UE crossing 1 → 2 → 3. The way back into
  cell 1 is out of reach of the DUs' 20 dB TX range on this rig. Re-phasing
  live cells is still open; it is not needed for measurability.
- **A real moving UE:** the crossing below is emulated by the DUs' TX
  power. The uplink path loss stays the same, there is one UE, and the ramp
  rate is one setting (~2 dB/s of margin). No over-the-air motion,
  fading or crowd has been run.
- **Unsigned neighbour lists:** without the signing material
  (`CELL_NL_SIGN_KEY`, `CELL_NL_CERT`, `CELL_CA_PUB`), an area-key holder
  can still forge a neighbour list, which is measurement provenance only.
  With it, and the replay checks, the residual is a real-time repeater on
  the right phase (see the replay section).
- **Keyed rig runs only from the grant-map authentication on:** the
  handover, DAPS, inter-frequency, measurement-gap, phase-planning and
  clock-fit rig results above were run without `CELL_PSK`, an
  unauthenticated development cell. Their tags were zeros and were not
  checked; F1 was keyed throughout. The testrig scripts now give every cell
  role one key. The suite re-run keyed is in the next section.
- **EVM:** not in the measurement reports.
