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
- **The area key** `K_A` tags what *another* cell must check: the
  neighbour list (the provenance of a measured neighbour beacon) and a
  second, area tag on every UE beacon (v3). A neighbour DU checks that tag
  before it counts a UE as heard or forwards it for DAPS; before, it took
  "heard" from any frame with the right address.
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
- **Phase planning beyond two cells:** a joining cell is placed against the
  running ones, which never move. With three or more cells on different
  channels, the 26 ms retune leaves no layout where every pair can measure
  both ways. Re-phasing live cells, or longer superframes, is open.
- **A real moving UE:** the crossing below is emulated by the DUs' TX
  power. The uplink path loss stays the same, there is one UE, and the ramp
  rate is one setting (~2 dB/s of margin). No over-the-air motion,
  fading or crowd has been run.
- **Per-cell context for joined UEs:** a RACH-joined UE has no provisioned
  UE key, so the CU cannot wrap a context for it. Under per-cell keys it
  refuses every handover command (`cell.ue.ho_no_context`) and stays in its
  cell. Provisioning a key at the join, from the RACH exchange, is open.
- **Area-key exposure:** anyone holding the area key can forge the
  neighbour list and a UE beacon's area tag. That can fake measurement
  provenance and "heard" or DAPS presence, but never a cell's control.
  Per-UE area credentials are open.
- **Keyed rig runs only from the grant-map authentication on:** the
  handover, DAPS, inter-frequency, measurement-gap, phase-planning and
  clock-fit rig results above were run without `CELL_PSK`, an
  unauthenticated development cell. Their tags were zeros and were not
  checked; F1 was keyed throughout. The testrig scripts now give every cell
  role one key. The suite re-run keyed is in the next section.
- **EVM:** not in the measurement reports.
