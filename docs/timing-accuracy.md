# Timing accuracy — devourer's TSF sync vs NTP / PTP over Wi-Fi

A common question: how does devourer's hardware-TSF time distribution
(`docs/time-distribution.md`) compare with running NTP or PTP over the same
Wi-Fi? Below are **measured** numbers from a two-node bench (this rig), the
methodology, and the honest caveats. Reproduce the transport half with the
`reglat` microbench; the NTP half with `chronyd -Q` as described.

## Bench

- **devourer TSF downlink**: master beacon TSF (inserted by the MAC at TX) vs
  slave arrival `tsfl` (latched by the MAC at RX), least-squares fit over many
  beacons. Both timestamps are taken *in the MAC hardware*.
- **NTP**: two separate machines associated to the same AP (WPA2, 5 GHz);
  `chronyd -Q` on the client measures the offset to a `local`-clock chrony server
  on the other node over the Wi-Fi path, without disciplining the clock. Software
  timestamps (see PTP below).
- **PTP**: `ptp4l` on the same link. The hardware-timestamped mode PTP needs
  for its precision is unsupported (`ethtool -T` → `PTP Hardware Clock: none`),
  so it cannot start; software mode (`-S`) is no better than the NTP row.

## Results

| Method | Precision over Wi-Fi (measured) | Timestamp source |
|---|---|---|
| **NTP** (`chrony`, software TS) | **~0.76 ms** RMS (quiet link); tens of ms under power-save/contention | app/kernel software |
| **PTP** (`ptp4l`) on the Wi-Fi radio | **no hardware mode** — stock driver exposes no HW timestamps; TX egress unavailable even patched (see below) | 802.11 MAC (RX only) |
| **devourer TSF downlink** | **~0.25 µs** RMS residual | 802.11 MAC hardware (TSF) |

So the TSF downlink is roughly **~3000×** tighter than NTP over the same Wi-Fi,
and the reason is the same one that stops PTP entirely.

## Why: the Wi-Fi MAC jitter, and the one timestamp that's immune

NTP/PTP precision over Wi-Fi is bounded by the **round-trip delay variation** of
the medium. Measured on this link (ICMP, node↔node): RTT swung **1.6 ms → 110 ms**
(mean 44 ms, mdev 45 ms) under load, ~0.3 ms mdev when quiet — the CSMA
backoff / retransmit / power-save latency of the 802.11 MAC. Software timestamps
sit *above* that layer, so they inherit all of it.

The 802.11 **TSF** is latched *inside* the MAC at the instant of TX/RX, *below*
that queueing/contention layer, so it is immune to the jitter that limits
NTP/PTP. devourer reads that per-frame RX TSF (`RxPacket.tsfl`) and the 64-bit
timer directly — which is exactly what a PTP hardware-timestamping NIC would
provide.

### Why stock PTP can't run on the Wi-Fi radio

Out of the box, `ptp4l` reports *"interface does not support requested
timestamping mode"* and `ethtool -T` shows `PTP Hardware Clock: none`: neither the
vendor drivers under `reference/` nor the in-tree `rtw88` expose the hardware
timestamp through the kernel timestamping API, even though both read the TSF
internally (the RX-descriptor `TSFL`). The hardware timestamp exists; the
OS-facing drivers just don't surface it — which is what devourer does
(`RxPacket.tsfl`).

But surfacing the RX timestamp is only half of PTP. Separating the two halves is
what makes the picture honest:

- **RX** is a genuine per-frame MAC timestamp, latched below the contention
  layer, and it *can* be surfaced through the standard kernel API — attaching it
  to each received frame lets an ordinary `SO_TIMESTAMPING` socket read it, and
  it advances in step with real airtime. This holds on both transports (the USB
  full-MAC path and the PCIe/`rtw88` `mac80211` path).
- **TX egress** — the instant the frame actually leaves the antenna — is the
  wall. The Realtek firmware reports no on-chip TX-egress timestamp for data
  frames, so a two-way PTP endpoint could only guess the send instant from a
  host- or bus-completion time, reintroducing exactly the jitter PTP exists to
  remove. This is a firmware limitation, not a driver one.

So PTP fails on the Wi-Fi radio for the TX half, not the clock. For contrast, a
dedicated PTP NIC on the *same board* — the Intel I226 Ethernet on the Radxa X4 —
runs `ptp4l` as a hardware grandmaster with **both** TX and RX hardware
timestamps and its own PHC. It has the TX-egress timestamp the Wi-Fi silicon
does not.

### The TX-egress wall — can it be cracked?

The one-way broadcast below is the ceiling because no Realtek part reports *when
a frame actually leaves the antenna*. This was checked across the vendor and
in-tree drivers, the firmware TX feedback, the 802.11 ranging path, and the
literature; the wall is in the silicon, not the driver:

- **The only TX feedback Realtek firmware emits is an ACK-status report, not a
  timestamp.** The rtw88/rtw89 C2H "TX report" carries transmit status, a retry
  count and a sequence number — no TSF or egress-time field — and it feeds the
  standard mac80211 ACK-status path, not a timestamp path.
- **The mac80211/cfg80211 hardware-timestamping framework exists** (kernel
  ~6.5+), but it is scoped to Timing-Measurement / Fine-Timing-Measurement
  action frames, and only Intel and NXP drivers implement it; the Realtek
  drivers register none of the required operations.
- **Fine Timing Measurement is not a backdoor.** Where Realtek exposes ranging
  at all it returns only a firmware-computed distance — the underlying TX/RX
  timestamps are never surfaced.
- **The one commodity Wi-Fi chip that _can_ do two-way hardware PTP is the
  Atheros AR9388 (ath9k)**, which latches the TSF on both TX and RX and reports
  the TX-status TSF to the host — the exact capability Realtek withholds. That
  is the lone published exception, not a general result.
- **Beacons are the one exception — and they prove the rule.** The Realtek MAC
  *does* hardware-stamp a single class of frame at egress: it inserts the live
  TSF into a beacon's timestamp field at the instant of transmission. Measured
  against an independent receiver's hardware RX timestamp, that egress stamp
  lands at **0.289 µs** RMS. But it is embedded in the outgoing frame for
  receivers to read, not reported back to the host — so it powers a one-way
  broadcast, not a two-way endpoint.

So two-way `ptp4l` over Wi-Fi needs ath9k-class silicon. On Realtek the honest
ceiling is a receive-only reference-broadcast scheme — timestamp a periodic AP
broadcast at reception via the TSF, no transmit timestamp required — which is
what devourer's TSF downlink already is, and what the published state of the art
for commodity Wi-Fi (OpenWiFiSync / RBIS) independently settled on, at
one-to-two-digit-µs accuracy.

The tempting host-side shortcut — reading the TSF register right after handing a
frame to the radio, as a stand-in for egress — does not close the gap. Measured
against a witness receiver's hardware RX timestamp, that read lands **~117 µs**
RMS from true air on a quiet 5 GHz channel and **~473 µs** on a busy 2.4 GHz
one — 400–1600× the 0.289 µs the beacon's hardware egress stamp achieves,
because it inherits exactly the USB-queue and CSMA-backoff jitter it was meant to
avoid. So the broadcast path (hardware-stamped beacon, read at reception) is not
just convenient — it is the only sub-µs option on this silicon.

If a future firmware or driver ever surfaces a TX-egress timestamp, this wall
falls and the RX timestamping already in place becomes a full two-way endpoint.

Evidence behind the current state (so it can be re-checked if the situation
changes):

- rtw88/rtw89 C2H TX-report contents — transmit status / retry count only, no
  timestamp:
  <https://lkml.org/lkml/2025/9/20/269>,
  <https://lists.openwall.net/linux-kernel/2025/11/04/1050>
- rtw89 registers no `set_hw_timestamp` / TSF timestamping ops:
  <https://github.com/lwfinger/rtw89/blob/main/mac80211.c>
- mac80211 hardware-timestamping API, scoped to TM/FTM frames (Intel/NXP only):
  <https://patchwork.kernel.org/project/linux-wireless/patch/20230223114629.2286397aa296.Iccc08869ea8156f1c71a3111a47f86dd56234bd0@changeid/>,
  <https://docs.kernel.org/driver-api/80211/mac80211.html>,
  <https://github.com/torvalds/linux/blob/master/include/net/mac80211.h>
- Realtek ranging HAL returns only a firmware-computed distance (no t1–t4/TSF):
  <https://github.com/kimocoder/android_wifi_hal/blob/master/hardware/realtek/wlan/wifi_hal/rtw_wifi_rtt.cpp>
- Atheros ath9k hardware-PTP-over-Wi-Fi precedent (USENIX ATC '21):
  <https://www.usenix.org/system/files/atc21-chen.pdf>
- OpenWiFiSync / RBIS one-way broadcast sync (IEEE ETFA 2024):
  <https://arxiv.org/pdf/2410.08742>

### Wi-Fi TSF vs real PTP hardware — measured on one board

Because that I226 is a true IEEE-1588 clock, it doubles as a local reference:
cross-comparing the Wi-Fi MAC TSF against it — both exposed as PTP hardware
clocks and disciplined slave-to-master on the same machine, no network in
between — puts a number on how good the TSF really is.

| Quantity | Measured | Reading |
|---|---|---|
| TSF vs I226 offset | **~290 ns** RMS (mean ≈ 0) | sub-µs tracking of genuine PTP hardware |
| TSF crystal offset | **~+42 ppm** | the Wi-Fi reference oscillator's raw frequency error |

The ~290 ns residual is essentially the TSF's own 1 µs-resolution quantization
floor (a uniform ±0.5 µs rounds to ~289 ns RMS): once the +42 ppm crystal offset
is taken out, the Wi-Fi MAC clock is as steady as the reference — the servo has
nothing left to chase but the last microsecond of resolution.

## Transport latency — USB vs PCIe (the actuator floor)

The closed-loop *uplink* timing advance is limited by the actuator, not the sync
measurement: `AdjustBeaconTimingFine` is a TSF read-modify-write of ~5–7 register
ops, and over USB each op is a vendor-control transfer. `tests/reglat.cpp` times a
single `RtlAdapter::rtw_read32` over each transport:

| Transport | mean / op | jitter (p99−p50) |
|---|---|---|
| USB (8822EU) | ~68 µs | ~64 µs |
| **PCIe 8821CE (MMIO)** | **~2.2 µs** | **~0.08 µs** |

PCIe MMIO is **~30× faster and ~600–900× lower jitter** per op, so the chained
actuator collapses from ~0.5–1.2 ms of jitter over USB toward ~µs over PCIe — i.e.
the transport, not the method, is the closed-loop-TA floor, and a PCIe path would
tighten the ~1.3 ms uplink residual toward tens of µs.

The same floor is what makes a *clock read* practical over PCIe. A PTP hardware
clock backed by the 8821CE TSF, read over that MMIO window, measures a gettime
read window of **~3.7 µs** (p99 ~5.6 µs) — cheap and steady enough to discipline
against, where the equivalent read over USB would cost tens of µs of jittery
vendor-control transfer. That is why a genuine PHC is tractable on the PCIe part
and not on the USB ones.

## Honest caveats

- devourer's ~0.25 µs is the **precision/stability** of a *one-way* broadcast fit
  (relative clock + slave-to-slave common view), not an independently-verified
  *absolute* offset. NTP/PTP are two-way and cancel path asymmetry; the TSF is
  1 µs resolution, so sub-µs comes from averaging the fit over many beacons.
- The NTP figure here is `chrony`'s offset over a *healthy* link; it degrades to
  tens of ms under Wi-Fi power-save/contention (the RTT data above). It is also
  the practical daemon result, close to the RTT-jitter bound that NTP cannot beat.
- FTM (802.11mc) round-trip ranging, where a driver exposes it, can beat a one-way
  µs-resolution TSF on *absolute* accuracy; devourer's advantage is availability
  (works on stock silicon today) and broadcast common-view, not a fundamentally
  finer clock than dedicated PTP hardware.
