# M1 reference cell — one DU, N UEs on a fixed slot grid

`examples/cell` is the first demoable piece of the scheduled RAN in EPIC #254 /
M1 #255: one gNB-DU and N UEs sharing one channel on a TSF-locked superframe,
with scheduled DL and UL and a per-UE timing advance. The reusable parts live
in `src/cell/`: `SlotGrid.h` (slot-budget accounting, the fixed layout) and
`GrantMap.h` (the beacon-carried grant map). The node logic is
`examples/cell/cell.h`, which is header-only and covered by the `cell_node`
ctest.

## Grid and frames

One superframe is one beacon interval (default 100 TU). The layout, in
superframe order:

| slot | length (default) | who transmits | what |
|---|---|---|---|
| beacon | 1.5 ms | DU | hardware beacon at its TBTT, carrying the grant map |
| DL k | 4 ms each | DU | one unicast QoS-Data to UE k, hardware-ACKed by the UE |
| turnaround | 1 ms | — | DL→UL guard |
| UL k | 2 ms each | UE k | UE k's own hardware beacon |

The DU submits each DL frame when its own TSF reaches the slot start. The UE
uplink is a **hardware beacon**, not `send_packet`. `send_packet` gives no
sub-slot control of departure (`docs/scheduled-mac.md`), while a beacon leaves a
fixed 22 µs after its TBTT once `SetBeaconBackoff(false)` is set.

## Clocks and the two loops

- **UE downlink sync.** Kestrel stamps every RX frame with the MAC's
  free-running counter, which TSF writes never move. A UE fits the DU's TSF
  (the beacons' egress timestamps) against that counter with
  `devourer::TsfSync`. The fit stays valid while the UE steps its own TSF.
- **UE uplink placement.** A UE's beacon fires when its own `tsf % period == 0`.
  So the UE keeps its TSF at `du_tsf − ul_slot_start + ta`, stepping it with
  `WriteTsf` whenever the error exceeds 2 µs. A read-add-write lands a few µs
  late, so each step is checked against the `tsf − freerun` offset read back
  after it, and the residual is folded into the next step.
- **DU timing advance.** The DU measures each UE uplink's arrival on its own
  TSF against the slot's target point. Every 2 s it moves that UE's TA by half
  the mean error, and republishes the map with a new epoch. The new map takes
  effect three superframes ahead, because a beacon update lands one to two
  beacons later (M0). A UE applies a map only at its effective superframe, and
  only for its own cell (`GrantMapReceiver`).

## Measured on the epic-254 rig

RTL8852CE over PCIe throughout: the DU, two UEs on two other boxes, and a fourth
radio as witness. Channel 149, a 120 s run, judged over the window the UEs were
running, after a 20 s TA settle.

| KPI | UE1 | UE2 |
|---|---|---|
| UL arrivals / superframes (DU) | 962 / 962 | 970 / 970 |
| UL arrival error rms / p99 / max | 1.8 / 6 / 13 µs | 1.4 / 3 / 7 µs |
| DL received / sent, in slot (UE) | 1156 / 1156, 100% | 1156 / 1156, 100% |
| DL air start after slot start p50 / p99 (witness) | 31 / 41 µs | 31 / 40 µs |
| DL retransmissions (witness) | 16 | 0 |

The witness saw every beacon, DL frame and uplink in its own slot, with 0
overlaps among 4837 frames. `tests/cell_2ue_analyze.py` computes all of this:
the DU's UL errors, the UEs' DL phases, and the witness's mapping onto the DU
clock. On the rig it is driven by testrig `scripts/m1_cell_2ue.sh`.

### What the first runs taught

- **An unACKed unicast escalates the contention window.** Kestrel's TX
  descriptor has no no-ACK path. A unicast to a station that does not ACK
  counts as a failure, and the data queue's CW climbs to CWmax. The next frame
  then waits a uniform 0–1023 slots (0–9.2 ms).
  - The signature, from a DL-spacing sweep (DU + witness only): the second DL
    frame never aired later than about 9.55 ms after the first, for any
    spacing from 0.4 to 8 ms, and it was always on time at 16 ms.
  - Hence the UE arms `SetAckResponder` on its own MAC, and the DU sends
    normal-ACK DL with a small retry limit (`DEVOURER_TX_RETRY_LIMIT=2`) that
    the DL slot budget covers.
- **Only the beacon queue's backoff is off.** The data queue keeps its EDCA
  backoff. With frames ACKed, that is at most CWmin (15 slots, 135 µs), well
  inside a 4 ms slot.

### Open

- Without UEs, the DU's DL is retried (three copies), and the retries leave
  their slot. The analyser excludes that period. A DU should stop scheduling
  a UE that has stopped uplinking (that belongs to M3, admission).
- Each UE re-steps its TSF by about +3 µs on most 100 ms passes. Its crystal
  skew against the DU (5–8 ppm) accounts for only ~0.7 µs of that, and the
  write-compensation residual reads ~0, so the cause is not established. The
  UL error stays at 1.4–1.8 µs rms. Each step costs one beacon on the old
  schedule (Kestrel's TBTT behaviour after a TSF step), which is at most one
  step-size off.

## Running it

```sh
# DU (UE list in grid order), two UEs, a witness: same channel and grid env
CELL_ROLE=du      CELL_UES=1,2 DEVOURER_TX_RETRY_LIMIT=2 DEVOURER_PCIE_BDF=... build/cell
CELL_ROLE=ue      CELL_UE=1    DEVOURER_PCIE_BDF=... build/cell
CELL_ROLE=ue      CELL_UE=2    DEVOURER_PCIE_BDF=... build/cell
CELL_ROLE=witness              DEVOURER_PCIE_BDF=... build/cell
python3 tests/cell_2ue_analyze.py --du du.jsonl --ue ue1.jsonl --ue ue2.jsonl --witness w.jsonl
```

The environment knobs are documented at the top of `examples/cell/main.cpp`.
The radio must advertise `tsf_write_ok`, `tbtt_follows_tsf`,
`beacon_backoff_ctl` and `hw_beacon_txtsf` (Kestrel). A UE also needs
`ack_responder_ok`.
