# DL departure: the send_packet→air guard-time contract (M0 contract 1)

A scheduled MAC (5G-NR RAN epic) promises collision-free downlink slots, which
requires knowing how long after the host calls `send_packet` the frame is
actually on the air — per transport, as a **distribution**, not an average.
This is that measurement and its per-transport contract.

## Method

One witness receiver captures, from the SAME frame, the transmitter's embedded
submit stamps and its own hardware RX timestamp (`RxAtrib.tsfl`, MAC-latched):

- **TD v2 tag** (`examples/tdma/tdma.h`, version byte 2): `tx_tsf` = ReadTsf
  near send (TX hardware clock) + `host_ns` = `steady_clock` immediately
  before the `send_packet` call (the clock a host-side slot scheduler actually
  controls).
- `tests/txegress_analyze.py` least-squares fits `rx_tsfl` against each stamp;
  the line absorbs clock offset, crystal skew and propagation, so the residual
  is per-frame submit→air jitter. Robust (3×MAD) rejection separates the
  **floor** (frames that aired immediately = transport + MAC-pipeline jitter)
  from the **tail** (channel deferral — which a scheduler must budget too).
- The contract number: `guard_us` = p99.9 of all host-clock residuals above
  the median, emitted as a machine-checkable `txeg.verdict` JSONL line with
  p50/p90/p99/p99.9/max and a `fine_dl_slots` go/no-go against `--slot-us`.

Probes: `tests/dl_departure_tx.cpp` (any USB adapter),
`tests/pcie_txegress_tx.cpp` (8821CE over vfio). Orchestration:
`tests/dl_departure_matrix.sh` (fixed witness, TX swept over the transports;
the PCIe cell ships the probe to the remote rig, vfio-binds, runs, restores).

## Measured (ch36, 6M legacy, ~2000 frames/cell, 8821AU witness)

| transport | floor RMS | p90 | p99 | p99.9 (=guard) | max |
|---|---|---|---|---|---|
| Jaguar1 8812AU (async USB2) | 22 µs | 28 µs | 101 µs | 0.76 ms | 2.1 ms |
| Jaguar2 8812BU (sync USB3) | 14 µs | 64 µs | 1.7 ms | 3.1 ms | 3.3 ms |
| Jaguar3 8822CU (sync USB3) | 16 µs | 61 µs | 2.2 ms | 3.2 ms | 3.3 ms |
| 8821CE (PCIe, vfio) | 11 µs | 54 µs | 2.4 ms | 3.2 ms | 3.3 ms |

Crystal ppm sane (−17 ppm, same reference witness) on every cell — the robust
fits locked. The tail is run-to-run ambient-dependent: the same J1 cell has
measured p99.9 between ~0.8 ms and ~3.2 ms across runs on the same channel.

## The contract

1. **The transport floor is NOT the bottleneck.** All four transports place
   the bulk of frames within tens of µs of nominal (floor RMS 11–26 µs; p90
   ≤ 64 µs). PCIe is the tightest (11 µs) but the USB floors are the same
   order — transport choice does not gate slot design at ≥ ms slot sizes.
2. **The tail is channel deferral, and it does not go away.** Even at 5 GHz on
   a mostly-idle channel, ~1% of frames air 0.1–2.4 ms late (ambient beacons +
   CSMA — `SetCcaMode` relaxes energy-CCA, not preamble deferral). p99.9 sits
   at ~1–3 ms on every transport.
3. **Go/no-go: fine (sub-ms) DL slots are REFUTED on all transports** for a
   p99.9-grade deadline on a real channel. The M1 design consequence is the
   fallback the epic anticipated: a **submission-ahead scheduler** — submit a
   slot's frame `guard_us` before the slot boundary and size slots ≥ ~2× the
   measured guard (i.e. multi-ms slots), OR accept a bounded deadline-miss
   ratio (~1% at a 1 ms guard, per the p99 row) and let HARQ absorb it.
   Re-measure `guard_us` per deployment environment; the verdict line exists
   so that check is one script run.

The hardware-beacon path (MAC-timed TBTT, `PinBeaconTbtt`) is unaffected by
any of this — beacons depart on the TBTT grid below the CSMA/queueing layer,
which is why scheduled **UL** rides beacon-steered timing, not `send_packet`.
