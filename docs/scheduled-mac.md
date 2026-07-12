# Scheduled-MAC building blocks — four measured contracts

A scheduled, cellular-style MAC on devourer — collision-free TDMA downlink
slots, a grant map carried in the beacon body, per-frame delivery detection
with hardware retransmission, per-UE link adaptation — rests on four radio
capabilities. Each section states the capability, how it was measured on
hardware, and the resulting per-generation contract a scheduler can build on.
The shared-clock and TBTT machinery these lean on is
[time-distribution.md](time-distribution.md); the multi-cell coordination
story they add up to is [multi-ap-cellular.md](multi-ap-cellular.md).

## TX departure: the send_packet→air guard time

A scheduled MAC that promises collision-free downlink slots needs to know how
long after the host calls `send_packet` the frame is actually on the air —
per transport, as a **distribution**, not an average. This is that
measurement and its per-transport contract.

### Method

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

### Measured (ch36, 6M legacy, ~2000 frames/cell, 8821AU witness)

| transport | floor RMS | p90 | p99 | p99.9 (=guard) | max |
|---|---|---|---|---|---|
| Jaguar1 8812AU (async USB2) | 22 µs | 28 µs | 101 µs | 0.76 ms | 2.1 ms |
| Jaguar2 8812BU (sync USB3) | 14 µs | 64 µs | 1.7 ms | 3.1 ms | 3.3 ms |
| Jaguar3 8822CU (sync USB3) | 16 µs | 61 µs | 2.2 ms | 3.2 ms | 3.3 ms |
| 8821CE (PCIe, vfio) | 11 µs | 54 µs | 2.4 ms | 3.2 ms | 3.3 ms |

Crystal ppm sane (−17 ppm, same reference witness) on every cell — the robust
fits locked. The tail is run-to-run ambient-dependent: the same J1 cell has
measured p99.9 between ~0.8 ms and ~3.2 ms across runs on the same channel.

### The contract

1. **The transport floor is NOT the bottleneck.** All four transports place
   the bulk of frames within tens of µs of nominal (floor RMS 11–26 µs; p90
   ≤ 64 µs). PCIe is the tightest (11 µs) but the USB floors are the same
   order — transport choice does not gate slot design at ≥ ms slot sizes.
2. **The tail is channel deferral, and it does not go away.** Even at 5 GHz on
   a mostly-idle channel, ~1% of frames air 0.1–2.4 ms late (ambient beacons +
   CSMA — `SetCcaMode` relaxes energy-CCA, not preamble deferral). p99.9 sits
   at ~1–3 ms on every transport.
3. **Go/no-go: fine (sub-ms) DL slots are REFUTED on all transports** for a
   p99.9-grade deadline on a real channel. The design consequence is a
   **submission-ahead scheduler** — submit a slot's frame `guard_us` before
   the slot boundary and size slots ≥ ~2× the measured guard (i.e. multi-ms
   slots), OR accept a bounded deadline-miss ratio (~1% at a 1 ms guard, per
   the p99 row) and let HARQ absorb it. Re-measure `guard_us` per deployment
   environment; the verdict line exists so that check is one script run.

The hardware-beacon path (MAC-timed TBTT, `PinBeaconTbtt`) is unaffected by
any of this — beacons depart on the TBTT grid below the CSMA/queueing layer,
which is why scheduled **UL** rides beacon-steered timing, not `send_packet`.

## Dynamic beacon-content delivery

A scheduled MAC that carries its DCI-style grant map in the beacon body must
be able to **change the airing beacon's content** without missing, duplicating
or tearing beacons. The primitive is
`IRtlDevice::UpdateBeaconPayload(beacon, len)` — an in-place content swap for
an active `StartBeacon` (same buffer contract; interval, TBTT phase and port
identity untouched) riding the same reserved-page re-download the TBTT steers
use. Its companion `StopBeacon()` silences the beacon function: the chip
beacons **autonomously**, so a beaconing session that ends without a device
power-cycle must call it — a killed process leaves the beacon airing
indefinitely (bench-bitten: a stale beacon with the same SA contaminated the
next test's witness).

Per generation: Jaguar2 replaces the retained `_bcn_mpdu` and re-downloads via
the steer path (the J2 engine loses the bcn-valid latch on re-latch, so the
download is also the re-arm); Jaguar1 is a fresh BCNQ-boundary store bracket
(no re-ignite needed — the port stays configured); Jaguar3 is a fresh HalMAC
`download_beacon_page` (its latch is stable — no steer machinery involved).

### Measured (ch36, 100 TU interval, 30 updates/gen every ~1 s, 8821AU witness)

`tests/beacon_update_check.sh` — the probe beacons a versioned vendor IE
(u32 version + version-derived 32-byte pattern + CRC16, so one frame proves a
torn swap) and calls `UpdateBeaconPayload` every 10 intervals; the witness
records `(hw seq, tsfl, version, crc_ok)` per beacon and
`tests/beacon_update_analyze.py` reconstructs the TBTT grid from `tsfl`,
separating update-caused missing slots from background witness loss.

| generation | updates aired | excess skips/update | torn | version regress | update→air p50 | p99 |
|---|---|---|---|---|---|---|
| Jaguar1 8812AU | 30/30 | 0.0 | 0 | 0 | 40 ms | 67 ms |
| Jaguar2 8812BU | 30/30 | 0.0 | 0 | 0 | 63 ms | 174 ms |
| Jaguar3 8822CU | 30/30 | 0.0 | 0 | 0 | 72 ms | 99 ms |

### The contract

1. **Dynamic beacon grants are GO on all three generations.** Every update
   aired; the background-corrected skip cost was 0 in this measurement (the
   ≤ 1 skipped beacon per re-download that TBTT steers pay was not even
   resolvable above witness loss at this cadence). No torn frames — the swap
   is frame-atomic as observed on air (the CRC never caught a half-old,
   half-new body). No old content re-airing after the new version's first
   appearance.
2. **Update→air latency is TBTT-quantized**: content lands on the next (or
   next-but-one) beacon — p50 ≈ half a period to a period, p99 ≤ ~2 periods at
   100 TU. A grant map published via the beacon is therefore *effective* one
   to two beacon intervals after the scheduler decides it, and the scheduler's
   grant timing must budget that pipeline (grants for slot epoch N+2, decided
   at epoch N).
3. **Not atomic versus TBTT by design**: a beacon airing during the download
   may still carry the previous content, and the API guarantees only
   whole-version frames (measured, via the CRC), not a bounded switchover
   instant. The `effective_tbtt` discipline lives in the grant-map payload
   (epoch field), not in the radio primitive.
4. A static-beacon + scheduled-unicast-delta fallback is not needed on any of
   the three generations.

Tooling: `tests/beacon_update_probe.cpp` (TX + witness modes; build line in
header), `tests/beacon_update_analyze.py` (`--selftest` covers the
skip/dup/late/stale/torn classifier on synthetic streams),
`tests/beacon_update_check.sh` (per-generation orchestration).

## Unicast ACK + TxReport capability matrix

A scheduled MAC's reliability layer (per-UE delivery detection, HARQ-style
retransmission, link adaptation) rests on two capabilities per generation:
an **injected unicast descriptor solicits a hardware ACK** (with autonomous
MAC retransmission until it arrives), and **`TxReport` reports the per-frame
ACK / no-ACK outcome** to the host. This measures both, per generation, plus
the report delivery rate — `tests/ack_txreport_matrix.sh` /
`tests/ack_txreport_analyze.py` (`--selftest` covers the verdict logic).

### Method

Fixed hardware-ACK responder (`SetAckResponder` on a second adapter); per TX
generation three phases: **on** (responder armed with MAC1, unicast QoS-Data
to MAC1 → expect ~100% `tx.report ok`, retries ~0), **retarget** (responder
re-armed to a different MAC2, TX to MAC2 → proves RA and responder MAC are
arbitrary), **off** (no responder → expect 0% ok, retries pinned at the
descriptor limit: the no-ACK outcome must be *visible*, per frame).
`report_coverage` = reports / frames sent (`tx.stats.submitted`); HalMAC adds
SW_DEFINE tag-echo gap counting.

TX sessions run `DEVOURER_TX_WITH_RX=thread`: CCX reports arrive on the C2H
RX path, so J1/J2 TX-only sessions never see them (measured: J2 TX-only = 0
reports; only J3 drains C2H off its coex runtime without an RX loop). A
scheduled MAC runs TX+RX anyway, so this is the relevant session shape.

### Measured (ch36, MCS3, unicast TA, 8814AU responder, ~8 s/phase)

| TX generation | on: ACK rate / mean retries | retarget | off: retries pinned | report coverage | tag gaps |
|---|---|---|---|---|---|
| Jaguar1 8812AU | 1.00 / 0.34 | 1.00 / 0.25 | yes (12) | 1.00 | n/a (8812 fmt) |
| Jaguar2 8812BU | 0.91 / 2.1 (run-to-run 0.12–0.91) | 0.64 / 5.3 | yes (12) | 0.86 | 0 |
| Jaguar3 8822CU | 1.00 / 0.24 | 1.00 / 0.13 | yes (12) | 0.96 | 0 |

Responder-side capability (same setup, J3 TX as the reference soliciting
station): **8814AU** closes the loop at retries ~0.1 (the bench responder of
choice); **8812AU** works but degraded (97% delivery at ~7 mean retries —
its SIFS ACKs only land intermittently); **8821AU never closed the loop**
(TX retries stayed pinned with it armed); the 8812BU responder was
separately proven (`tests/ack_responder_check.sh`).

### The contract

1. **Per-frame delivery detection is GO on all three generations**: the OFF
   phase pins retries at the descriptor limit with `state=1` on every report —
   a no-ACK outcome is unambiguously visible per frame, which is all a
   software retransmission layer needs. Report coverage 86–100% with zero
   HalMAC tag gaps (interior losses); the reliability layer must tolerate a
   ~5–15% report-less frame tail (treat missing report as "unknown", not
   "delivered").
2. **Closed-loop hardware ACK + autonomous retry is GO on Jaguar1 and
   Jaguar3** (100% delivery, retries ≈ 0.2–0.3) including retargeting an
   arbitrary UE MAC mid-session (re-arm `SetAckResponder`, change the
   descriptor RA — both fully dynamic).
3. **Jaguar2 as the soliciting TX is MARGINAL as measured**: ACK closure
   varied 12–91% across identical runs (mean retries 2–11) against both
   8814AU and 8812AU responders, and its TX pace in the TX+RX-thread shape is
   ~24 ms/frame regardless of the requested gap (~37 fps vs J1/J3's ~150).
   As a *responder* J2 is proven good. Prefer J1/J3 (or the 8821CE) for the
   soliciting role — the J2 anomaly is measured but unexplained; treat it as
   open, not as silicon folklore.
4. Bench quirk recorded: the J3 report's `missed` field reads a constant 4×
   the report count while tag continuity shows zero loss — the 8822C
   `missed_rpt` offset likely decodes something else; trust `tag` gaps on J3.

## Per-UE RX attribution

`GetRxQuality()` is device-wide by design: one draining accumulator fed by
every decoded frame, whoever sent it. A cell scheduler adapting per-UE rate and
power needs the same windowed statistics **attributed to each transmitter** —
that is `devourer::cell::UeRxAttribution` (`src/cell/UeRxAttribution.h` —
`src/cell/` holds the caller-side per-cell helpers built on the device API).

### The contract

Pure caller-side logic — the device RX loops are untouched and `GetRxQuality`
stays the radio-wide diagnostic. Everything needed is already per-frame in the
`Packet` callback:

- **key** — the transmitter address (802.11 addr2/TA), extracted by
  `cell::extract_ta`: bytes [10..16) of the MPDU for every frame type except
  the two control subtypes that end at addr1 (CTS, ACK).
- **values** — `rx_pkt_attrib`'s path-A RSSI/SNR/EVM plus the hardware RX
  timestamp `tsfl`, folded with the exact `RxQualityAccumulator` conventions
  (`rssi_raw <= 0` is not a sample; SNR/EVM folded only when present; passive
  noise floor = `(rssi_raw − 110) − snr_raw/2`).

`add()` (or `add_mpdu()`, which extracts the TA itself) per frame;
`snapshot()` drains the whole table into one `UeRxWindow` per TA (delta
semantics, like `GetRxQuality`) with converted units, window mean/extremes and
`last_tsfl` for staleness. The table is bounded (default 64 TAs per window);
overflow frames are counted in `evicted_frames`, never silently lost.

### Measured

`tests/ue_rx_attribution_check.sh`: two transmitters with distinct unicast SAs
(8812AU at a 2 ms inter-frame gap, 8822CU at 8 ms) against one `ue_rx_probe`
witness (8812BU), 12 s. The probe attributed the streams separately —
TX1 2955 frames at −51 dBm mean, TX2 741 frames at −44 dBm mean, a 4.0×
count ratio exactly matching the 4× cadence ratio — confirming per-UE frame
counts and per-UE signal statistics don't bleed between transmitters.

### Tooling

- `tests/ue_rx_probe.cpp` — on-air probe: feeds every decoded frame into a
  `UeRxAttribution`, drains once a second, emits one `ue.rx` JSONL event per
  UE per window (`ta`, `frames`, `rssi_dbm`, `rssi_max_dbm`, `snr_db`,
  `snr_min_db`, `evm_db`, `nf_dbm`, `last_tsfl`) plus `ue.rx.evicted` when the
  cap was hit. Build line in the header.
- `tests/ue_rx_attribution_selftest.cpp` — headless ctest guard
  (`ue_rx_attribution_derive`): TA extraction over frame types, folding
  conventions, drain semantics, eviction accounting.
- `tests/ue_rx_attribution_check.sh` — the two-TX on-air validation above.

A cell scheduler wraps this into its UE registry (association state, timing
advance, the per-UE RX window as the link-adaptation input).
