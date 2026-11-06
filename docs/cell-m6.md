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
- **The exposure:** grant maps carry no tag (they never did), so the epoch
  rule orders maps; it does not authenticate them. A forged map could
  already redirect a UE's slots. The restart path widens that: within the
  re-attach window after a clock step, a forged lower-epoch map due near the
  beacon's count also resets the UE's per-DU state (HARQ, MR generation,
  neighbours, pending handover IEs). Grant-map authentication is open
  (below).

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
- **A moving UE:** the rig's UEs are stationary, so the crossing is forced
  with a negative A3 offset.
- **Phase planning beyond two cells:** a joining cell is placed against the
  running ones, which never move. With three or more cells on different
  channels, the 26 ms retune leaves no layout where every pair can measure
  both ways. Re-phasing live cells, or longer superframes, is open.
- **Grant-map authentication:** the grant map IE has no tag, unlike the
  NL, MR and handover IEs. A tag keyed per cell (as `CELL_PSK`), checked
  before any `offer`, would close the forged-map exposure, including the
  restart path. It is a wire change for the DU, UE and witness.
- **A DU restart under RACH-joined UEs:** re-attachment (above) is for the
  rig's static UEs, whose id the restarted DU grants again. A UE that joined
  through the RACH holds an id the new DU instance does not know; re-joining
  it is open.
- **No crossing test:** every handover result here is from stationary UEs.
  The trigger is either forced or the measured standing difference between
  two cells. No A→B path-loss ramp has been run, so none of these numbers
  validates a crossing. An emulated crossing (the source DU ramping its TX
  power, which its beacons follow) is the way to get one.
- **Security context:** the cells share one key on the rig (`CELL_PSK`).
  Pre-staging a per-cell context (802.11r-style) is open.
- **EVM:** not in the measurement reports.
