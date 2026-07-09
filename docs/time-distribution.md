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
`{"ev":"timesync.lock"}` streams by beacon seq into the inter-UE error).

## Hardware beacon — sub-µs downlink (`DEVOURER_TSYNC_HWBEACON`)

The software downlink above is bounded by the master's stamp→air jitter. The
hardware path removes it entirely: `IRtlDevice::StartBeacon` loads a beacon into
the MAC's beacon reserved-page and lets the chip **auto-transmit it at each
TBTT** — hardware-timed, and the MAC inserts the live 64-bit TSF into the
beacon's timestamp field at the transmit instant. No `ReadTsf()`, no
`send_packet`, no software in the timing path. One call suffices; the chip
beacons indefinitely. Implemented on both HalMAC generations (Jaguar2 8822B/
8812BU/8821C and Jaguar3 8822C/8822E); Jaguar1 has no reserved-page path.

The slave then reads the *standard 802.11 beacon timestamp* (the master's live
TSF) instead of a software tag, and fits it against its own arrival `tsfl`.

The last limit is CSMA: a TBTT beacon still defers to carrier-sense and airs
after a variable backoff, so its scheduled-TSF stamp and delayed air time
diverge by ~hundreds of µs on a shared channel. In a time-distribution setup the
master **owns** the channel, so that backoff is pure loss — the master disables
EDCCA (`SetCcaMode`, on by default here; opt out with `DEVOURER_TSYNC_CSMA=1`)
and the beacon airs exactly on schedule.

Bench (HW-beacon master + slave, **crowded** ch6, 100 ms beacons):

| config | downlink residual |
|--------|-------------------|
| software stamp | ~94 µs RMS |
| HW beacon, CSMA on | ~470 µs RMS (backoff jitter) |
| HW beacon + no-CSMA (default) | **0.31 µs RMS** (8822C) / 0.39 µs (8812BU) |

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
via `EN_HWSEQ`), and the live hardware TSF — bench-confirmed on both HalMAC
generations (J3 8822C and J2 8812BU). A frozen sequence number indicates a
degraded engine (e.g. a J2 beacon left in the post-drop state after an
`AdjustBeaconTiming` tweak, which J2 does not survive), not a healthy beacon.

**End-to-end interop.** The strongest check is that a *real* Linux 802.11 station
accepts the beacon: `tests/beacon_kernel_scan.sh` airs a full-content beacon from
devourer and scans for it with a second Realtek adapter bound to `rtw88`
(`iw dev <if> scan`). Proven — the kernel lists it and parses every element:

    BSS 57:42:75:05:d6:00   TSF <live>   freq 2437   beacon interval 25 TUs
    capability: ESS   SSID: devourerAP
    Supported rates: 1.0* 2.0* 5.5* 11.0* 18.0 24.0 36.0 54.0
    DS Parameter set: channel 6   TIM: DTIM Count 0 Period 1   ERP: <no flags>

So devourer's beacon is not just wire-correct but accepted as a legitimate AP by
the mac80211/rtw88 BSS parser — live TSF, capability, SSID, rates (with basic-rate
flags), DS param, TIM/DTIM and ERP all interpreted correctly. Confirmed on **both
bands** — ch6 (2437 MHz) and ch36 (5180 MHz). A dense beacon interval (e.g. 25 TU,
`DEVOURER_BCN_TU`) is needed so the scan's per-channel dwell catches it.

devourer also answers **active scans** like a kernel AP: `tests/probe_responder.cpp`
runs full-duplex, matches probe-requests in the RX callback and `send_packet`s a
probe-response addressed to the requester — with **no beacon aired**, a real rtw88
station's `iw scan` still lists `devourerAP` (discovery via the probe response).
send_packet must run off the RX event thread (queue the requester, TX from another
thread — libusb returns BUSY otherwise). This works because management-frame timing
is tens of ms (the userspace RX→TX round-trip is a few ms), unlike SIFS ACKs.

**A real kernel station fully associates.** `tests/ap_responder.cpp` completes the
whole open-network handshake (probe → auth → assoc): a real rtw88 station
authenticates, associates, and stays `Connected` (wpa_supplicant
`CTRL-EVENT-CONNECTED`, `iw link` Connected, stable). The station's auth/assoc
requests arrive with **retry=0** — devourer hardware-ACKs them (the ACK engine
matches `REG_MACID`, which `StartBeacon` sets to the BSSID). The one non-obvious
requirement: **the BSSID must be unicast**. The canonical test SA `57:42:75:05:d6:00`
has the I/G (multicast) bit set in `0x57`; a station cannot unicast-auth to a
multicast address, so rtw88 silently drops the auth before it hits the air
(bench-proven across two station chips — `0x57` → no auth on air, `0x02` → auth on
air and association completes). Use a locally-administered unicast BSSID (`0x02..`).

**Data plane — a real station gets an IP and pings devourer's AP.** `ap_responder`
answers the post-association data plane over 802.11 from-DS data frames: a minimal
**DHCP server** (DISCOVER→OFFER, REQUEST→ACK leasing 192.168.99.2), **ARP**
replies for the AP IP, and **ICMP** echo replies. So a Linux station associates,
`dhcpcd` **automatically leases 192.168.99.2** from it, and `ping 192.168.99.1`
returns **0% packet loss, ~2 ms RTT** — the full self-service chain (beacon → auth
→ assoc → DHCP → ARP/ICMP → ping), like a hostapd+dnsmasq AP, in one userspace
process. `tests/ap_ping_demo.sh` runs it end to end.

**WPA2-PSK.** `tests/ap_wpa2.cpp` runs the WPA2 **4-way handshake** authenticator —
the beacon/assoc advertise an RSN IE and the AP does msg1→4 with PMK/PTK
(PBKDF2+PRF), HMAC-SHA1 MIC and an AES-key-wrapped GTK (openssl in userspace). A
real station completes it: wpa_supplicant reports `WPA: Key negotiation completed …
[PTK=CCMP GTK=CCMP]` + `CTRL-EVENT-CONNECTED`. The handshake needs no CCMP
(EAPOL-Key frames are cleartext, MIC-protected), so it works without the chip
crypto engine. And the **data plane is encrypted too**: the AP decrypts the
station's CCMP frames (software AES-CCM with the TK) and answers ARP + ICMP + DHCP
encrypted, so a real station **leases 192.168.99.2 over encrypted DHCP** and
**pings the AP over WPA2/CCMP at 0% loss, ~2.5 ms RTT**. The station runs hardware
CCMP, the AP software CCMP — they interoperate. So devourer is a complete
zero-config WPA2-PSK AP: associate → 4-way → encrypted DHCP → **encrypted** IP.
(Hardware CCMP offload would need porting the J3 security TX/RX descriptor fields,
absent in devourer; software CCMP sidesteps that.)

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
the floor for a userspace-USB TSF write (a kernel PCIe driver with MMIO would be
tighter). Harness: `tests/timesync_ta_demo.sh` with `HWBEACON=1`.

**Steering the TBTT.** The actuator is `AdjustBeaconTiming(microseconds)`, not a
TSF write: `WriteTsf` moves the reported TSF (and the beacon-body timestamp) but
NOT the TBTT air-time — a separate per-port timer drives the beacon, so the TBTT
is deaf to `REG_TSFTR`. A one-shot beacon-interval tweak *does* steer it: running
one interval at (nominal ± Δ) TU then restoring advances/retards the next TBTT —
and the cadence thereafter — by Δ TU. Bench-proven to the microsecond on an
8822C: `AdjustBeaconTiming(-20480)` advanced the TBTT by exactly 20 TU
(observer arrival phase stepped 88018 → 67565 µs at the tweak). This is the
802.11 IBSS/TSF-merge mechanism. Granularity is **1 TU = 1.024 ms** (the
`REG_BCN_INTERVAL` field is integer TU) — a coarse slot/guard-alignment lever.

**Microsecond-fine steering** (`AdjustBeaconTimingFine`, Jaguar3): the reason a
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

Beacon-TBTT steering is **Jaguar3-only**. The Jaguar2 8822B beacon engine drops
the beacon on *any* TBTT re-latch — bench-proven on the 8812BU, the beacon stops
airing after both the interval tweak and the beacon-function toggle (the
bcn-valid latch is lost, and J2 does not retain the beacon bytes to re-download).
So `AdjustBeaconTiming` / `AdjustBeaconTimingFine` refuse on J2 (return 0) rather
than silently kill the beacon; the downlink (`StartBeacon` + `SetCcaMode`) is
unaffected.

Actuator characterization: `tests/beacon_interval_shift.sh <short_TU>` (TU tweak),
or `FINE_US=<µs> tests/beacon_interval_shift.sh` (µs-fine, Jaguar3).

Harness: `tests/timesync_ta_demo.sh` (+ `tests/timesync_ta_analyze.py`).

## Env knobs

- `DEVOURER_TSYNC_ROLE=master|slave|ue`
- `DEVOURER_TSYNC_INTERVAL_MS=N` — master beacon period (default 100)
- `DEVOURER_TSYNC_RATE=<spec>` — beacon TX rate (default 6M; must be heard)
- `DEVOURER_TSYNC_SECS=N` — run duration (0 = until signalled)
- `DEVOURER_TSYNC_HWBEACON=1` — hardware-timed beacon downlink (StartBeacon) →
  sub-µs; set on both master and slave. On the **UE** (role=ue) it instead
  selects the hardware-beacon uplink fine-steered by `AdjustBeaconTimingFine` —
  the converging closed loop (J3 UE required for µs steering)
- `DEVOURER_TSYNC_CSMA=1` — keep CSMA on the HW-beacon master (default: EDCCA
  off so the beacon airs exactly at TBTT — the master owns the channel)
- `DEVOURER_TSYNC_UPLINK=1` — enable the uplink timing-advance loop (experimental)
- `DEVOURER_TSYNC_SLOT_MS=N` — uplink slot grid on the master TSF (default 20)
- `DEVOURER_TSYNC_TA_GAIN=g` — TA integrator gain (default 0.3)
- `DEVOURER_CHANNEL=N`, `DEVOURER_PID` / `DEVOURER_VID` — as elsewhere
