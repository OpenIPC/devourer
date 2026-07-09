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
50 ms) as the fits densify and balance. Slaves must be Jaguar2/3 (per-frame
`tsfl`); the master can be any transmitter (`ReadTsf()`, including Jaguar1).

Run it with `tests/timesync_demo.sh` (one master + two slaves; joins the two
`{"ev":"timesync.lock"}` streams by beacon seq into the inter-UE error). A
hardware-timed master beacon (a MAC-transmitted beacon at TBTT, rather than a
software-stamped injection) would drop the absolute-lock floor toward the
inter-slave sub-µs, but that is a separate reserved-page/beacon-queue build.

## Uplink timing advance (experimental)

`DEVOURER_TSYNC_UPLINK=1` adds the LTE closed-loop half: a full-duplex master
phase-measures each UE uplink against its own TSF slot grid (the arrival `tsfl`
mod the slot period — reliable per-frame, needing no `ReadTsf()` the RX loop
would starve) and integrates a timing advance it broadcasts back in the beacon; a
full-duplex UE transmits one uplink per beacon, advanced by that correction, so
its frames should land on the master's slot boundary. Both nodes come up
full-duplex (`DEVOURER_TX_WITH_RX=thread`, InitWrite + StartRxLoop).

The control loop converges in the headless selftest and the full-duplex plumbing
works on-air — with a **fixed** advance the uplinks arrive tightly clustered
(~±0.2 ms). But the closed loop does **not** converge on the bench, and a
fixed-advance authority test shows why: changing the advance shifts the UE's
send-*call* time but not the master-measured *arrival* phase. Under full-duplex,
`send_packet` queues the frame and the chip airs it on its own schedule, so
userspace call-timing has no sub-slot control over air departure — the loop has
nothing fine to actuate. (The bench also has only one clean full-duplex
Jaguar2/3 adapter; the RTL8822EU desenses its RX in TX+RX mode, adding jitter.)
The measurement, feedback, and math are in place; closing the loop needs
hardware-timed TX departure, which this path does not have.

Harness: `tests/timesync_ta_demo.sh` (+ `tests/timesync_ta_analyze.py`).

## Env knobs

- `DEVOURER_TSYNC_ROLE=master|slave|ue`
- `DEVOURER_TSYNC_INTERVAL_MS=N` — master beacon period (default 100)
- `DEVOURER_TSYNC_RATE=<spec>` — beacon TX rate (default 6M; must be heard)
- `DEVOURER_TSYNC_SECS=N` — run duration (0 = until signalled)
- `DEVOURER_TSYNC_UPLINK=1` — enable the uplink timing-advance loop (experimental)
- `DEVOURER_TSYNC_SLOT_MS=N` — uplink slot grid on the master TSF (default 20)
- `DEVOURER_TSYNC_TA_GAIN=g` — TA integrator gain (default 0.3)
- `DEVOURER_CHANNEL=N`, `DEVOURER_PID` / `DEVOURER_VID` — as elsewhere
