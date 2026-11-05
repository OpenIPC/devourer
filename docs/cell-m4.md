# M4 — a DU on the PTP fabric

M4 (#258, EPIC #254) holds every DU's beacon grid to one wired PTP
grandmaster. That makes a slot boundary the same instant on every cell, and
it lets a DU notice when it is no longer true. The pieces:

- **`src/cell/TsfDiscipline.h`:** the discipline loop, headless. The bench
  tool `tests/pcie_ptp_tsf_discipline.cpp` runs on it.
- **`src/cell/SyncHealth.h`:** loss-of-sync detection.
- **`sync_tier()` in `src/AdapterCaps.h`:** the formal sync tiers.
- **`examples/cell`:** the DU service mode, set by `CELL_REF_PTP`. The DU
  follows its health between a coordinated and an autonomous schedule, and
  its UEs follow the DU.

The `cell_m4` ctest drives the loop and the health against a simulated radio
and PHC. `adapter_caps` covers the tiers.

## The discipline loop

Each iteration takes one **bracketed** sample: a PHC read, the radio's
(TSF, free-running counter) pair, then a second PHC read.

1. **Bracket filter.** A sample whose bracket exceeds the median of the last
   32 by 5 µs is rejected: the host was preempted between the reads.
2. **Fit.** It fits `ref = a·freerun + b` over the last 64 accepted samples.
   - TSF writes never move the free-running counter, so each step's true
     effect is measured, never booked.
   - Parts without that counter use the booked-steps ledger axis instead.
3. **Phase error.** The next TBTT is the next TSF multiple of the interval.
   The loop maps it to reference time and takes its phase error against
   `CELL_PHASE_US`.
4. **Correction.** A PI with a capture-gated integrator returns the TSF step.
   - A failed write drops the fit, since the counter may be half-updated.

Between corrections the grid drifts by ppm × loop period: 20 ppm at 500 ms
is 10 µs. That stepping sawtooth is inherent to a stepped actuator. The
selftest checks the true pre-correction phase (rms ≈ 1.1 µs) and bounds the
sawtooth separately.

## Sync tiers

`sync_tier(caps)` is derived from the timing caps, so it cannot drift from
them.

| tier | condition | hold | M5 use |
|---|---|---|---|
| **tight** | PCIe, `tsf_write_ok`, `tbtt_follows_tsf`, `hw_beacon_txtsf` | ~µs (8852CE: on-air TBTT p95 4 µs) | same-channel coordinated slots |
| **coarse** | a hardware beacon and a writable TSF, otherwise (USB; Jaguar2/3) | ~0.5–1.5 ms | ms guards or frequency separation |
| **none** | no hardware beacon or no TSF write | — | cannot hold a shared grid |

The tier describes what the TSF-write actuator achieves. A part steered
another way (`PinBeaconTbtt`, 0.67 µs on the 8821CE) is not promoted by it.
The DU service mode requires **tight**.

## Loss-of-sync detection

`SyncHealth` moves through three states: **Lost → Locked → Degraded**.
Coordinated scheduling is allowed only while **Locked**.

**Locked** needs `CELL_SYNC_LOCK_ITERS` (8) acting samples in a row, each
within `CELL_SYNC_LOCK_US` (10 µs).

From Locked, an acting sample out of tolerance moves the cell to **Degraded**,
where its coordinated schedule is suspended.

These go to **Lost** at once:

- the reference check fails;
- the beacon check fails;
- a TSF write fails;
- the phase error exceeds `CELL_SYNC_LOST_US` (200 µs).

These go to Lost after `CELL_SYNC_LOST_ITERS` (20) in a row:

- out-of-tolerance samples;
- non-acting samples (a rejected read, or the fit still filling).

A non-acting sample carries no phase, so it neither degrades nor locks. On
the rig, a single preempted read had suspended a locked cell about once a
minute until this rule was added.

The checks behind those inputs:

- **Reference:** `CELL_REF_CHECK` is a command whose exit 0 means the
  reference is good. Service mode requires it, because a readable PHC is not
  proof of sync: it free-runs on after ptp4l stops.
  - It runs every `CELL_REF_CHECK_S` (2 s), off the radio path.
  - A check still running after `CELL_REF_CHECK_TIMEOUT_MS` (1.5 s) is killed
    and counts as a failure.
  - A failed PHC read is a failure too.
  - The rig uses `pmc` to require ptp4l's port in SLAVE.
- **Beacon engine:** there is no beacon TX counter on this path, so the DU
  uses a proxy. Once a UE in the live map has been heard, the check fails if
  all of them go silent for `CELL_SYNC_UL_SF` (30) superframes. Three failed
  beacon updates in a row also fail it.
  - The proxy cannot tell a stalled beacon from all UEs leaving. Either way
    the cell suspends, which is the conservative reading.
  - A stall **before any UE has been heard** goes undetected. Closing that
    gap needs a direct beacon-TX signal from the radio.
- **TSF write:** a failed write is a timing failure (Lost), and so is a
  failed read-back of the new offset. Until the next superframe's refresh,
  RX attributes no uplink or RACH frame.

## Coordinated vs autonomous

| | coordinated | autonomous |
|---|---|---|
| carrier sense (`SetCcaMode`) | off: the cell owns its slots | on: defer to neighbours |
| beacon departure (`SetBeaconBackoff`) | deterministic, fixed AIFS | random EDCA backoff |
| grant map `flags` | 0 | `kGrantAutonomous` |

**The DU** starts autonomous. It acquires lock before serving, for up to
`CELL_SYNC_ACQUIRE_S` (30 s), because the first steps can move the grid by up
to half a superframe. It logs every switch as `cell.mode`.

- **Suspend** is immediate. The desynced DU is the side that must defer.
- **Resume** waits until the map that clears the flag is on air and in force.
  So the DU is never deterministic while its UEs are told to defer.

**The UEs** apply the flag from the map in force. That is the publish
superframe + 3, about 300 ms later, and they log `cell.ue.mode`.

**A standalone cell** (no `CELL_REF_PTP`) is always coordinated, which is the
M1–M3 behaviour.

## Rig result

Measured with testrig `scripts/m4_cell_sync.sh`:

- **Roles:** the DU on j6412-1 (8852CE, PTP slave of the n300 hub), two UEs
  and a witness.
- **Run:** 180 s. ptp4l on the DU node was stopped for 30 s at +71 s, then
  started again.
- **Bounded check:** with a hung check (`CELL_REF_CHECK='sleep 60'`), the DU
  stays autonomous and exits on time, with no process left behind. Without a
  check, service mode refuses to start.

| | |
|---|---|
| acquisition | Locked after 30 samples (~15 s); coordinated from sf 152, when the flag-clear map was in force |
| phase while locked | rms 1.6 µs, p95 3.5 µs, max 6.8 µs (261 samples, 5 rejected reads) |
| PTP stop | Lost at the first failed check (`ref_ok=0`, sf 688) → autonomous; both UEs followed |
| PTP back | ptp4l SLAVE again at sample 212, Locked 7 samples later; DU coordinated at sf 1101, once that map was in force; both UEs followed |
| flaps | none: 4 DU mode switches in total (lock, outage, recovery, and the UEs leaving at the end) |
| slots | `cell_2ue_analyze` pass: UL and DL in slot, no overlaps |

The final transition is the beacon proxy firing when the UEs' runs ended
20 s before the DU's. No beacon stall was injected.

**Not yet measured: the multi-DU test with a USB DU** (#258's
`cell_ptp_multi.sh`). The rig's DUs are all PCIe 8852CEs, so only the tight
tier is measured here. The coarse tier's ~0.5–1.5 ms is the USB actuator
figure from `docs/time-distribution.md`, not a measurement made with this
mode.
