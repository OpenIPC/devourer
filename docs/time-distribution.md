# Over-the-air time distribution (LTE-eNB style)

devourer exposes the chip's hardware TSF (the 802.11 MAC's free-running 64-bit
microsecond clock) two ways: a per-frame receive stamp (`rx_pkt_attrib::tsfl`,
the low 32 bits latched in the MAC at reception, on all three generations) and a
direct `IRtlDevice::ReadTsf()`. Together they are the primitive an LTE eNB uses
to give its UEs a common timebase — one node holds a reference and distributes it
over the air, and every other node slaves to it with no GPS of its own. The
`timesync` example (`examples/timesync/`) is a worked demonstration.

## The idea

An LTE eNB broadcasts its frame timing; UEs acquire it from the downlink and run
their whole schedule in the eNB's timebase. Only the eNB needs an absolute
reference (typically GPS); a UE derives everything from the signal it hears. The
802.11 analog:

- the **master** (eNB) periodically broadcasts a beacon carrying its hardware
  TSF — the SFN,
- each **slave** (UE) relates that broadcast TSF to its *own* per-frame hardware
  TSF with a running least-squares fit, and predicts the master clock from it.

Both the broadcast TSF and the local `tsfl` are clean MAC-latched microsecond
clocks, so the fit residual is the true lock quality — not the ~1 ms host-callback
jitter a wall-clock scheme would carry. The distribution costs one node a
reference and the rest nothing but reception.

## Downlink: distributing the clock

`DEVOURER_TSYNC_ROLE=master` brings the chip up TX-only and broadcasts a beacon
every `DEVOURER_TSYNC_INTERVAL_MS` (default 100), each stamped with `ReadTsf()` —
reliable on a transmitter, where no bulk-IN flood starves the control read.

`DEVOURER_TSYNC_ROLE=slave` brings the chip up RX-only. For every beacon it
decodes it feeds the pair (broadcast master TSF, local `tsfl`) into the fit, then
*predicts* the beacon's master TSF from the fit built on prior beacons — the
prediction error is how tightly this UE tracks the eNB. A slave never reads a
host or wall clock.

Two slaves predicting the **same** beacon (matched by its sequence number) agree
far more tightly than either's absolute lock. Each slave's absolute lock is
bounded by the master's *software* stamp→air jitter (it calls `ReadTsf()` then
sends), but that jitter is common to every slave — it is the same frame, aired
once — so it cancels in the difference. What survives is only the independent
per-slave MAC-latch noise: the sub-microsecond floor of a two-receiver TSF
correlation.

Bench (master RTL8812AU, slaves RTL8822CU + RTL8822EU, ch36, 50 ms beacons):

| metric | value |
|--------|-------|
| per-slave absolute lock | ~94 µs RMS (master stamp-jitter bound) |
| inter-slave (inter-UE) agreement | 4.7 µs RMS, mean +1.7 µs |

The inter-slave agreement tightens with beacon rate (≈18 µs at 100 ms → ≈5 µs at
50 ms) as the fits densify and balance. Every generation exposes the per-frame
`tsfl` a slave needs and the slave role is bench-validated on all three
(Jaguar2/3, and Jaguar1: an 8821AU slave locks at **0.30 µs RMS** to a
hardware-beacon master); the master can be any transmitter (`ReadTsf()`,
including Jaguar1).

The ~94 µs absolute bound is the **transport's submit-to-air floor**, not a
protocol property: measured with an embedded-submit-TSF frame stream and a
robust (outlier-rejecting) fit against a witness receiver, USB floors at
~93 µs RMS (dominated by the USB submit path) while **PCIe floors at ~12 µs
RMS** (the residual MAC TX pipeline — the transport stops being the limiter).
A PCIe master (8821CE) therefore lifts every slave's *absolute* software-path
lock ~8×; the inter-slave difference and the hardware-beacon path below are
unaffected (already transport-free). Tool: `tests/pcie_txegress_tx.cpp` +
`tests/txegress_witness.cpp`. The `timesync` demo drives a PCIe master via
`DEVOURER_PCIE_BDF` — but note the ~12 µs floor is a *robust-fit* transport
number: on a busy channel the raw slave lock is deferral-limited regardless of
transport (measured 420 µs RMS on a crowded ch6 — the channel, not the bus),
so prefer the hardware beacon below for PCIe masters.

Run it with `tests/timesync_demo.sh` (one master + two slaves; joins the two
`{"ev":"timesync.lock"}` streams by beacon seq into the inter-UE error).

## Hardware beacon — sub-µs downlink (`DEVOURER_TSYNC_HWBEACON`)

The software downlink above is bounded by the master's stamp→air jitter. The
hardware path removes it entirely: `IRtlDevice::StartBeacon` loads a beacon into
the MAC's beacon reserved-page and lets the chip **auto-transmit it at each
TBTT** — hardware-timed, and the MAC inserts the live 64-bit TSF into the
beacon's timestamp field at the transmit instant. No `ReadTsf()`, no
`send_packet`, no software in the timing path. One call suffices; the chip
beacons indefinitely. Implemented on all three generations: Jaguar2 (8822B/
8812BU/8821C) and Jaguar3 (8822C/8822E) via the HalMAC reserved-page download,
and Jaguar1 (8812A/8811A/8821A — bench-proven on the 8812AU and 8821AU) via
the pre-HalMAC BCNQ-boundary store bracket, byte-matched to a golden usbmon
dump of the in-tree `rtw88_8821au` beacon download. The J1 sequence is: the
port-0 AP enable first (MSR=AP, DUAL_TSF_RST BIT0, BCN_CTRL 0x18, the 8821A
"BCN on port 0" 0x454[5] clear, ResumeTxBeacon), then the store bracket —
BCN_VALID (0x20A[0]) W1C → CR+1 SW-beacon-DMA on → beacon function off →
bulk a **minimal** descriptor (LAST_SEG + OFFSET + PKT_SIZE + QSEL_BEACON +
rate-FB-limit; OWN/FIRST_SEG/BMC/HWSEQ mark a live TX and the store path
rejects them) → function on → SW-beacon-DMA off — and finally an internal
`PinBeaconTbtt(0)` **igniter**: the J1 engine does not start airing from the
download/enable alone; it needs a TSF write with a real value edge inside the
EN_BCN bracket plus a post-arm re-download (the same-value-rewrite trap — the
pin shifts by a full extra beacon period so the grid is unchanged but the
edge is guaranteed).
The 8814A (bench-proven, own golden dump) differs in three ways: its valid
latch is 0x204[15] with the head held at the BCNQ boundary during the store
(pointing the head at page 0 — the firmware-download bracket's shape — stores
the beacon where the TBTT engine never reads), it needs 0x420[12] + a
0x454[2:0]=0x05 enable, and its stored beacon airs with the 802.11 sequence
pinned at 0 (kernel rtw88 parity — seq-based beacon-health checks don't apply
there).

The slave then reads the *standard 802.11 beacon timestamp* (the master's live
TSF) instead of a software tag, and fits it against its own arrival `tsfl`.

The last limit is CSMA: a TBTT beacon still defers to carrier-sense and airs
after a variable backoff, so its scheduled-TSF stamp and delayed air time
diverge by ~hundreds of µs on a shared channel. In a time-distribution setup the
master **owns** the channel, so that backoff is pure loss — the master disables
the MAC carrier-sense gate (`SetCcaMode`: primary CCA + EDCCA, on by default
here; opt out with `DEVOURER_TSYNC_CSMA=1`) and the beacon airs exactly on
schedule. `SetCcaMode` is implemented on Jaguar2/3 and is a deliberate no-op on
Jaguar1: the J1 baseband EDCCA is
already disabled by its init table (`0x8A4 = 0x7F7F7F7F`, thresholds
unreachable), and the measured J1 downlink needs no MAC-side gate — see the
bench row below. (Porting the J2 MAC-register recipe to J1 was bench-refuted:
no improvement, and its `0x524[11]` clear conflicts with the vendor
beacon-enable state.)

Bench (HW-beacon master + slave, **crowded** ch6, 100 ms beacons):

| config | downlink residual |
|--------|-------------------|
| software stamp | ~94 µs RMS |
| HW beacon, CSMA on (J2/J3) | ~470 µs RMS (backoff jitter) |
| HW beacon + no-CSMA (default) | **0.31 µs RMS** (8822C) / 0.39 µs (8812BU) |
| HW beacon, Jaguar1 (no gate needed) | **0.34–0.40 µs RMS** (8821AU master, 8822CU slave) |

So the master↔slave clocks track to sub-µs on any channel — the offset moves
only with the crystal drift. `DEVOURER_TSYNC_HWBEACON=1` on both master and
slave; `tests/beacon_ts_check.cpp` reads the raw beacon TSF for validation.

Note: the beacon body may carry full kernel-AP content. A well-formed 70-byte
body (10-char SSID + 8 supported rates + DS param + TIM + ERP IEs) inserts the
live hardware TSF correctly — bench-verified on an 8822C, the timestamp field
steps ~102400 µs per beacon exactly as the minimal body does
(`tests/beacon_fullbody.cpp`). An earlier report that an extended body broke the
TSF insertion was a *malformed* test frame (bad IE lengths), not a hardware
limit; there is no minimal-body requirement. `build_std_beacon` keeps a compact
layout only because the timesync demo needs no more.

On-wire kernel-equivalence is verifiable with `tests/beacon_wire_check.cpp`: the
beacon carries the right frame control (0x0080), an 802.11 sequence number that
**increments by 1 per beacon** (the hardware sequence numbering a kernel AP does
via `EN_HWSEQ`), and the live hardware TSF — bench-confirmed on all three
generations (J3 8822C, J2 8812BU, J1 8821AU). Exception: the 8814A airs its
stored beacon with the sequence pinned at 0, matching the kernel rtw88 driver
on the same chip — judge its health by presence/cadence/timestamp instead. On
the other dies a frozen sequence number indicates a degraded engine (a beacon
left in the post-re-latch drop state), not a healthy beacon.

The same hardware beacon is also the foundation for full **infrastructure AP
mode** — a real Linux station discovers devourer, associates, gets an IP, and
pings it (open or WPA2-PSK encrypted). That is a distinct feature with its own
write-up: see **`docs/ap-mode.md`** (`tests/beacon_kernel_scan.sh`,
`probe_responder`, `ap_responder`, `ap_wpa2`, `ap_ping_demo.sh`).

## Uplink timing advance (experimental)

`DEVOURER_TSYNC_UPLINK=1` adds the LTE closed-loop half: a full-duplex master
phase-measures each UE uplink against its own TSF slot grid (the arrival `tsfl`
mod the slot period — reliable per-frame, needing no `ReadTsf()` the RX loop
would starve) and integrates a timing advance it broadcasts back in the beacon; a
full-duplex UE transmits one uplink per beacon, advanced by that correction, so
its frames should land on the master's slot boundary. Both nodes come up
full-duplex (`DEVOURER_TX_WITH_RX=thread`, InitWrite + StartRxLoop).

With the default `send_packet` uplink the loop does **not** converge, and a
fixed-advance authority test shows why: changing the advance shifts the UE's
send-*call* time but not the master-measured *arrival* phase. Under full-duplex,
`send_packet` queues the frame and the chip airs it on its own schedule, so
userspace call-timing has no sub-slot control over air departure — the loop has
nothing fine to actuate (bench baseline: arrival phase RMS ~5.4 ms, drifting
across the whole slot).

**Closed loop (`DEVOURER_TSYNC_HWBEACON=1` on the UE).** The fix is a
hardware-timed uplink: the UE airs its uplink from the **beacon engine**
(`StartBeacon` stores a tdma Uplink frame in the rsvd page; the engine airs it at
the UE's TBTT) and steers that TBTT with `AdjustBeaconTimingFine` per the master's
TA — the sub-slot air-departure control `send_packet` lacked. The master's
measurement path is unchanged (it matches the Uplink frame by class). Bench-proven
to converge: master = 8812AU (Jaguar1, full-duplex — the master needs no fine
steering, so it need not be J3), UE = 8822CU (J3, HW-beacon + µs-fine steer), ch36
slot 20 ms. The arrival phase drops from ~6 ms to a bounded oscillation around
zero at **~1.25 ms RMS steady-state** (gain 0.30) — a 4× improvement over the
non-converging `send_packet` baseline. The residual is the fine actuator's USB
read→write jitter (~0.5–1.2 ms per correction, §Microsecond-fine steering); it is
the floor for a userspace-USB TSF write (over the PCIe transport the same
actuator's register path is µs-scale — see the 8821CE row below). A
`PinBeaconTbtt`-based UE actuator was **bench-refuted on USB** (equal at steer
cadence, unstable-to-worse at low cadence with matched loop gain: every pin
re-rolls the ~1 ms USB placement error, so the noise the fine steer pays per
correction the pin pays per pin) — the fine steer remains the right USB UE
actuator, and the µs-class UE path is a PCIe UE. Harness:
`tests/timesync_ta_demo.sh` with `HWBEACON=1`.

**Steering the TBTT.** The actuator is `AdjustBeaconTiming(microseconds)`, not a
TSF write: on Jaguar2/3, `WriteTsf` moves the reported TSF (and the beacon-body
timestamp) but NOT the TBTT air-time — a separate per-port timer drives the
beacon, so the TBTT is deaf to `REG_TSFTR`. (Jaguar1 is the opposite
architecture: its TBTT is hardware-locked to the TSF grid, so a TSF write moves
both — see the `PinBeaconTbtt` per-generation notes.) A one-shot
beacon-interval tweak *does* steer the J2/J3 TBTT: running
one interval at (nominal ± Δ) TU then restoring advances/retards the next TBTT —
and the cadence thereafter — by Δ TU. Bench-proven to the microsecond on an
8822C: `AdjustBeaconTiming(-20480)` advanced the TBTT by exactly 20 TU
(observer arrival phase stepped 88018 → 67565 µs at the tweak). This is the
802.11 IBSS/TSF-merge mechanism. Granularity is **1 TU = 1.024 ms** (the
`REG_BCN_INTERVAL` field is integer TU) — a coarse slot/guard-alignment lever.
The interval register **latches at a TBTT**, so the actuator phase-aligns off
the TSF (write the tweak early in a beacon period, restore mid-way into the
first tweaked interval) — a fixed hold instead races with the beacon phase:
bench-caught on the 8821CE, an advance can silently no-op (write+restore inside
one period) or double-shift (two shortened TBTTs before the restore latches).
Consecutive coarse steers move the TBTT grid off `TSF % period == 0`; the
actuator tracks that offset (a fine steer or `StartBeacon` re-zeroes it).

**Microsecond-fine steering** (`AdjustBeaconTimingFine`): the reason a
bare `WriteTsf` doesn't move the TBTT is that the counter is latched while the
beacon function runs — the vendor `reset_tsf` path clears `EN_BCN_FUNCTION`
first. Toggling the beacon function off, shifting the port-0 TSF by the desired
µs, and toggling it back on makes the TBTT re-derive from the shifted TSF at
**microsecond resolution**. Bench-proven on an 8822C (arrival-phase steps, all
sub-TU): `-5000 → -4479 µs`, `-10000 → -8997 µs`, `+6000 (retard) → +7172 µs`.
The magnitude undershoots/overshoots the request by a **sub-ms USB
read→write latency** (~0.5–1.2 ms, the real TSF advances during the register
sequence) — a systematic offset a closed timing-advance loop absorbs; the
resolution is what matters. It also shifts this port's TSF + beacon-body
timestamp by the same amount, which is the intended UE-advances-its-own-timebase
behaviour.

**Jaguar1/2 steer-then-re-download.** The Jaguar1 and Jaguar2 beacon engines lose their
bcn-valid latch on *any* TBTT re-latch — bench-proven on the 8812BU, the beacon
stops airing after both the interval tweak and the beacon-function toggle — and
the hardware does not retain the reserved-page bytes to re-arm it. The driver
does (`StartBeacon` keeps the MPDU), so the J1/J2 actuators steer and then re-run
the reserved-page beacon download to re-assert the latch — at most one skipped
beacon per correction. On Jaguar1 the interval tweak is additionally **inert**
(8821AU: the beacon survives but the phase never moves), so its coarse
`AdjustBeaconTiming` rides the fine TSF-toggle mechanism, TU-quantized (note it
then also shifts the reported TSF). Bench (fine steers, observer arrival phase;
hardware seq consecutive across every steer, except the 8814A's pinned-at-0
seq):

- **8812BU (USB)**: 0–1 beacon lost per steer, ~9 ms per fine steer; the fine
  shift undershoots the request by a systematic ~2–3 ms (USB register latency,
  larger than J3's ~0.5–1.2 ms) that a closed loop absorbs.
- **8821AU (USB, Jaguar1)**: 0–1 beacon lost per steer, ~8 ms per fine steer,
  systematic undershoot ~1.6 ms with ±40 µs consistency (−3392…−3472 µs for a
  −5000 µs request).
- **8814AU (USB, Jaguar1)**: 0–3 beacons lost per steer (RF-weak bench unit).
  Its TBTT counter free-runs across the EN_BCN toggle (it only pauses while
  off), so the fine steer additionally pulses port-0 `DUAL_TSF_RST` to
  re-derive the grid from the shifted TSF — but that pulse also **zeroes the
  reported TSF** (the fine steer rewinds the 8814 clock to ~0 every call), so
  a controller fitting against the 8814 TSF must not use the fine steer;
  `PinBeaconTbtt(0)` reconstructs the timeline from the host clock (~1 ms
  class). Open-loop single-steer accuracy ±few ms.
- **8821CE (PCIe MMIO)**: 0–1 beacon lost per steer, ~0–1 ms per fine steer;
  fine accuracy **−4842…−5047 µs for a −5000 µs request** (systematic offset
  ~30 µs — the MMIO register path is µs-scale) and coarse −20 TU steers land
  within ~±150 µs, including back-to-back. This is the actuator that closes
  the AP-beacon ↔ network-PTP discipline loop on the Radxa X4's 8821CE.

At a realistic discipline cadence (~10 s between corrections against ~40 ppm
crystal drift and a ~500 µs guard) the cost is ~1 lost beacon per 100. Jaguar3
needs no re-download — its engine survives the re-latch.

**Steering corrupts the controller's own clock — use `PinBeaconTbtt`.**
`AdjustBeaconTimingFine` necessarily jumps the reported TSF (the TBTT re-derives
from a shifted TSF), so a discipline loop whose phase estimate is a fit against
that TSF (`ref = a·tsf + b`) breaks its own sensor at every steer. Bench-observed
on the 8821CE AP↔PTP loop: high-authority controllers (PI, large clamped steps)
chase the corrupted estimate into a limit cycle, and the working ~60 µs
proportional loop is a **sweet spot, not a floor** — its steers are only small
enough (≈0.5·e) not to wreck the fit, so more authority makes it worse, and
`SetXtalCap` can't buy a lower steer cadence (the AFE trim moves only ~10 of the
~42 ppm crystal offset). The escape is `PinBeaconTbtt(offset_us)`: the same
shift + re-latch, immediately followed by a TSF write back onto the original
timeline — on the latched J2/J3 engines a bare TSF write does not move the
TBTT, so the steered phase survives while the clock the loop reads stays
continuous. Bench (8821CE PCIe):
TSF discontinuity **~10 µs** per correction (vs the full steer magnitude for
the fine variant — ~500× less fit disturbance), on-air phase pinned within
~±150 µs of the commanded offset, zero beacons lost — and the closed AP↔PTP
loop holds **~±1 µs** with it (converged from −41 ms, smoothly tracking the
~11 ppm residual drift; ~60× tighter than the fine-steer proportional sweet
spot). Semantics are **absolute** (TBTT fires at `TSF % interval == offset`),
so the controller commands the target offset directly instead of integrating
steps — and a PTP-disciplined TSF drags the pinned TBTT with it between
corrections.

Per generation: **Jaguar2** — full pin support (~10 µs disturbance over PCIe
MMIO; ~0.5–1 ms over USB, the restore-write latency). **Jaguar3** — full pin
support (USB; ~0.5–1.5 ms restore disturbance, still far below a fine steer's
full-magnitude jump). **Jaguar1** — offset 0 only: its TBTT is
**hardware-locked to the TSF grid** (bench on all three dies: a nonzero pin
never holds, the phase follows the restored TSF), so a J1 master needs no
TBTT actuator at all — discipline the TSF and the TBTT tracks it in hardware;
`AdjustBeaconTimingFine` (which moves both together) remains the manual
lever. Where a pin's ~1 ms USB restore disturbance is worse than a tiny
steer, the controller-side fix is a steering ledger: add the cumulative
commanded shifts back onto the raw TSF before fitting, so the fit sees a
continuous virtual clock and full authority is safe again.

Actuator characterization: `tests/beacon_interval_shift.sh <short_TU>` (TU tweak),
or `FINE_US=<µs> tests/beacon_interval_shift.sh` (µs-fine). Steer *survival* +
repeated-steer accuracy (incl. a PCIe master via `MASTER_CMD='ssh …'`):
`tests/beacon_steer_survival.sh [n] [steer_us] [period_s]`.

Harness: `tests/timesync_ta_demo.sh` (+ `tests/timesync_ta_analyze.py`).

## Bridging to network PTP (IEEE 1588)

The missing half of a wired-time → Wi-Fi-time bridge is holding the Wi-Fi TSF
against a real PTP clock. On the Radxa X4 the 8821CE's TSF is exposed as a
Linux PTP Hardware Clock (`tests/pcie_phc/` — a kernel module serving
`/dev/ptpN` from the TSF over BAR2 MMIO), and `phc2sys` disciplines it to the
board's Intel I226 (a full IEEE-1588 NIC):

| metric | value |
|--------|-------|
| held residual vs the I226 PHC | **~290 ns RMS**, mean ~0, stable over minutes |
| raw crystal offset removed by the servo | ~+42 ppm |

The ~290 ns residual is the TSF's own **1 µs-quantization floor** (uniform
±0.5 µs → 289 ns RMS) — once the crystal offset is servoed out, the Wi-Fi MAC
TSF holds like genuine PTP hardware. Combined with `PinBeaconTbtt` above, this
closes the loop, and the closed loop is a shipped tool:
`tests/pcie_ptp_beacon.cpp` runs the hardware beacon on the 8821CE and holds
its TBTT to the I226 PHC (read via `FD_TO_CLOCKID`, no system-clock detour)
with a full-gain PI on the pin actuator. End-to-end bench, everything running
concurrently (the PCIe discipline loop + a USB slave over the air):

| link in the chain | measured |
|-------------------|----------|
| 8821CE TBTT vs the I226 PTP reference | **1.24 µs RMS**, max 5.5 µs (156 pins, ~90 s) |
| USB slave lock through the live discipline | 13.9 µs RMS (575 beacons, ~0 lost through 160+ pins) |

The slave number carries a caveat: its all-history least-squares fit assumes a
free-running master, so it *measures the discipline corrections themselves*;
a tracking filter (PLL-style) at the slave would follow the pinned timebase
tighter. PHC validation: `tests/pcie_phc/ptp_crosscheck.sh` (the Wi-Fi side
must be in monitor mode so the MAC/TSF is clocked).

What this unlocks at the multi-AP level — coordinated scheduling between
cells, make-before-break handover, robots as roaming UEs — is mapped out in
[`multi-ap-cellular.md`](multi-ap-cellular.md); the four measured
per-generation contracts a slot scheduler builds on (submit→air guard time,
dynamic beacon grants, hardware ACK/TxReport, per-UE RX attribution) are in
[`scheduled-mac.md`](scheduled-mac.md).

## Env knobs

- `DEVOURER_TSYNC_ROLE=master|slave|ue`
- `DEVOURER_TSYNC_INTERVAL_MS=N` — master beacon period (default 100)
- `DEVOURER_TSYNC_RATE=<spec>` — beacon TX rate (default 6M; must be heard)
- `DEVOURER_TSYNC_SECS=N` — run duration (0 = until signalled)
- `DEVOURER_TSYNC_HWBEACON=1` — hardware-timed beacon downlink (StartBeacon) →
  sub-µs; set on both master and slave. On the **UE** (role=ue) it instead
  selects the hardware-beacon uplink fine-steered by `AdjustBeaconTimingFine` —
  the converging closed loop (bench-validated with a J3 UE)
- `DEVOURER_TSYNC_CSMA=1` — keep CSMA on the HW-beacon master (default: EDCCA
  off so the beacon airs exactly at TBTT — the master owns the channel)
- `DEVOURER_TSYNC_UPLINK=1` — enable the uplink timing-advance loop (experimental)
- `DEVOURER_TSYNC_SLOT_MS=N` — uplink slot grid on the master TSF (default 20)
- `DEVOURER_TSYNC_TA_GAIN=g` — TA integrator gain (default 0.3)
- `DEVOURER_CHANNEL=N`, `DEVOURER_PID` / `DEVOURER_VID` — as elsewhere
