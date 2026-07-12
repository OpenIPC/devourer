# Dynamic beacon-content delivery (M0 contract 2)

A scheduled MAC (5G-NR RAN epic) carries its DCI-style grant map in the beacon
body, so the DU must be able to **change the airing beacon's content** without
missing, duplicating or tearing beacons. The primitive is
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

## Measured (ch36, 100 TU interval, 30 updates/gen every ~1 s, 8821AU witness)

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

## The contract

1. **Dynamic beacon grants are GO on all three generations.** Every update
   aired; the background-corrected skip cost was 0 in this bench (the ≤ 1
   skipped beacon per re-download that TBTT steers pay was not even resolvable
   above witness loss at this cadence). No torn frames — the swap is
   frame-atomic as observed on air (the CRC never caught a half-old,
   half-new body). No old content re-airing after the new version's first
   appearance.
2. **Update→air latency is TBTT-quantized**: content lands on the next (or
   next-but-one) beacon — p50 ≈ half a period to a period, p99 ≤ ~2 periods at
   100 TU. A grant map published via the beacon is therefore *effective* one
   to two beacon intervals after the scheduler decides it. M1's grant timing
   must budget that pipeline (grants for slot epoch N+2, decided at epoch N).
3. **Not atomic versus TBTT by design**: a beacon airing during the download
   may still carry the previous content, and the API guarantees only
   whole-version frames (measured, via the CRC), not a bounded switchover
   instant. The `effective_tbtt` discipline lives in the grant-map payload
   (epoch field), not in the radio primitive.
4. The M0 fallback (static beacon grant + scheduled unicast deltas) is NOT
   needed. Kept as the escape hatch if a future generation/transport shows a
   real per-update skip cost.

Tooling: `tests/beacon_update_probe.cpp` (TX + witness modes; build line in
header), `tests/beacon_update_analyze.py` (`--selftest` covers the
skip/dup/late/stale/torn classifier on synthetic streams),
`tests/beacon_update_check.sh` (per-generation orchestration).
