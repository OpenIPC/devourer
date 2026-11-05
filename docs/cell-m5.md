# M5 — a CU coordinating co-channel cells

M5 (#259, EPIC #254) adds the CU. The CU owns the slot map across cells: a UE
that two cells hear never shares airtime with the other cell. The pieces:

- **`src/cell/Coord.h`:** the planner.
- **`src/cell/F1.h`:** the DU ↔ CU messages, and the DU's side of the
  failure model (`Stager`).
- **`tools/ran-controller`:** the CU. It never touches a radio, and it builds
  from `src/` headers alone, so it runs on the PTP hub (`n300-1` on the rig).
- **`examples/cell`:** the DU's F1 link (`CELL_F1_CU`). The DU reports, stages
  and applies the CU's schedules; the UEs follow their grants.

The `cell_m5` ctest covers the planner and F1. `ran_coord_analyze` guards the
rig KPIs.

## The model

Every DU holds its beacon grid to the PTP fabric (M4, the DU service mode). A
slot that starts `start_us` after a DU's TBTT is then at network time
`phase + start_us`, modulo the shared superframe.

- **Network superframe:** `floor((PTP − phase) / period)`. Every DU counts the
  same number, so an `effective_net_sf` names one instant network-wide.
- **Positions:** a DU's capacity grid has capacity positions, each a DL + UL
  slot pair, plus fixed items (the beacon, and the RACH with its tail gap).
  - A UE's id is its identity (MAC, HARQ, registry).
  - Its position is the CU's choice. The UE accepts any position's slot pair,
    and the DU's TA loop targets the UL slot of the grant the UE runs.

## The planner (`coord_plan`)

**Interference relation.** UE *u* of cell *c* is "heard by" DU *o* when *o*
reports *(c, u)* at RSSI ≥ `RAN_EDGE_DBM`. That RSSI comes from the other
cells' uplink beacons, which the radio receives anyway. Two co-channel items at
different DUs conflict when they overlap in network time and a UE of either is
heard by the other DU. A fixed item stands for all of its DU's UEs.

**Steps:**

1. **Tiers:** a DU that is not tight-sync and relates to a co-channel DU gets a
   spare channel, never time slots. With no spare it is reported as
   `unseparated`, and its UEs count as unresolved.
2. **Placement:** UEs are placed greedily, the most-related first. Each keeps
   its current position if that is clean (no churn); otherwise it takes the
   first conflict-free free position.
3. **Retune:** a UE with no conflict-free position moves its DU to a spare
   channel, and the plan restarts. Otherwise it counts as unresolved.

**Guarantee:** a plan with nothing unresolved has no conflicts. The selftest
checks this property over 400 random scenarios.

**The slot model is conservative.** A slot is its full budget, not the frame's
airtime. On the rig, a stagger of one beacon slot (1500 µs) puts cell 2's
beacon inside cell 1's DL slot (a planned conflict), yet the two do not
collide in the air. Kestrel's DL airs about 0.49 ms into its slot, which is
the M0 departure latency, and the beacon has ended by then.

## F1 and the failure model

The messages are UDP datagrams with an HMAC-SHA1 tag, keyed by `CELL_F1_KEY` /
`RAN_F1_KEY`.

- **Key required:** a DU refuses an F1 link without a key unless
  `CELL_F1_INSECURE=1` marks it as a development link.
- **Peer check:** a DU takes schedules only from the configured CU address.
- **Report freshness:** reports carry a freshness `seq` too. The CU ignores a
  report at or below the last one it took from that DU, so a replayed report
  cannot redirect that DU's schedules.

| message | direction | content |
|---|---|---|
| Report | DU → CU | tier, sync state, mode, channel, phase, network superframe, freshness `seq`, applied/staged versions, grid, UEs and positions, the other cells' UEs heard |
| Schedule | CU → DU | version, `effective_net_sf`, freshness `seq`, every UE's position |
| Ack | DU → CU | staged / applied / stale / late / invalid |

**The `Stager`** (from #259's control-plane failure model):

- **Versions move forward:** an older version is Stale. CU versions start
  from the CU's start time, so a restarted CU's versions still move forward.
- **Lead time:** a schedule arriving less than `CELL_F1_MIN_LEAD_SF` (8)
  before its instant is Late.
- **Invalid:** a schedule that is not a one-to-one assignment of exactly the
  DU's live UEs to distinct positions changes nothing.
- **Re-validation at publish:** a UE admitted after the schedule was staged
  moves off any pair the schedule takes. If no pair is free, the schedule is
  not applied. The deadline is checked again right before the publish.
- **No clock:** without a valid network superframe (the conversion is
  dropped whenever SyncHealth is Lost), a new schedule is Late. The
  keepalive of the version the DU already runs is still answered and
  still counts as CU contact.
- **Instant:** the DU publishes the schedule in its grant map
  `CELL_F1_PUBLISH_LEAD_SF` (5) before the instant, with exactly that
  instant as the map's effective superframe. One pending map at a time: no
  other change can republish over an announced instant.
- **Missed instant:** a schedule whose publication misses its deadline is
  abandoned, never applied off its instant.
- **Replays:** each send carries a fresh `seq` (the CU's realtime clock). A
  replayed datagram is Stale and does not count as CU contact.
- **CU outage:** with no fresh CU message for `CELL_F1_TIMEOUT_MS` (3 s), the
  DU is autonomous (the M4 mode). A DU is also autonomous until it first
  hears a CU.

**The CU:**

- **Keepalives:** every round it resends every DU's current schedule,
  including rounds where it cannot plan, for example while one DU is
  re-locking.
- **Versions:** a new version is above both the CU's own last version and
  whatever the DU reports applied or staged.
- **Channels:** the positions it pushes are always planned on the channels
  the DUs are actually on. Spare-channel proposals come from a separate
  advisory plan.

**Not applied:** the planner's channel proposals are logged (`ran.retune`)
but not applied, because a DU changes channel only at start. Moving a running
cell and its UEs to another channel is M6 territory.

## Rig result

Measured with testrig `scripts/m5_ran_coord.sh`:

- **Roles:**
  - two DUs on ch149: j6412-1 (cell 1, phase 0) and j6412-3 (cell 2, phase
    13450 µs);
  - one UE each, on j6412-2 and j6412-3;
  - a two-cell witness on j6412-2;
  - the CU on n300-1.
- **Grid:** capacity 3; each UE is heard by the other DU.
- **Duration:** 150 s per run.
- **Stagger:** 13450 µs puts cell 2's DL on air exactly on cell 1's UE uplink.
  The stagger comes from witness stamps, so the baseline really collides.

Final-code runs:

| | baseline (`--plan 0`: CU keeps each UE in position 1) | coordinated (`--plan 1`) |
|---|---|---|
| CU plan | 2 conflicts kept | 2 → 0 (both UEs to position 2) |
| instant | both DUs on one `effective_net_sf` | both DUs on one `effective_net_sf` |
| CU send → applied | 10 superframes | 10 superframes |
| cell 1 UL heard by its DU | **62.6 %** (840 / 1341) | **100 %** (1342 / 1342) |
| cell 1 DL retransmissions | **361** (the feedback rides the lost uplinks) | 0 |
| cell 2 DL / UL | 100 % / 100 % | 100 % / 100 % |
| DL submit lateness p99 | 8–9 µs | 7–10 µs |
| verdict | fail (UL ratio) | pass |

An earlier pair of runs, before the review fixes, gave 55.6 % / 435
retransmissions against 99.9 % / 2.

The witness decodes at most one frame of a colliding pair. In the baseline it
counted only 1 cross-cell overlap (cell 1's UL with cell 2's DL), but its
cell 1 UL count dropped (1308 vs 1341). That makes it a lower bound, and the
delivery KPIs are the measure.

**CU outage** (`--cu-outage 20`):

- Both DUs went autonomous 3 s after the CU was killed.
- A new CU's schedule, with a higher version, was staged and then applied by
  both DUs on one instant.
- The DUs had resumed coordinated access a few superframes earlier, on the
  positions they already ran. Those positions are still conflict-free, since
  no DU can change them without a CU.
- The post-recovery window passes.

**Also on the rig:** at a stagger of 1500 µs, coordination is clean (0
conflicts planned, 100 % both ways). The M1 standalone run passes on this
code.

## Not covered

- Applying a channel change at runtime: frequency separation is proposed,
  not applied.
- A USB (coarse-tier) DU: the rig has none. The tier rule is selftest-only.
- More than two DUs on the rig: the planner handles N, and the selftest runs
  2–4.
