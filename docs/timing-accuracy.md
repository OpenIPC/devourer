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
- **PTP**: attempted with `ptp4l -S` on the same link.

## Results

| Method | Precision over Wi-Fi (measured) | Timestamp source |
|---|---|---|
| **NTP** (`chrony`, software TS) | **~0.76 ms** RMS (quiet link); tens of ms under power-save/contention | app/kernel software |
| **PTP** (`ptp4l`) | **does not run** — see below | (would need HW/driver TS) |
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

### Why PTP can't run here at all

`ptp4l` reports *"interface does not support requested timestamping mode"*, and
`ethtool -T` shows `PTP Hardware Clock: none` with no timestamp modes. This is not
specific to the in-tree `rtw88`: grepping the **vendor** drivers under
`reference/` for `SOF_TIMESTAMPING` / `SIOCSHWTSTAMP` / `skb_hwtstamps` /
`get_ts_info` / `ptp_clock` finds **nothing** — neither the vendor nor the
mainline driver exposes the hardware timestamp through the kernel timestamping
API, even though both read the TSF internally (`CONFIG_HW_P0_TSF_SYNC`, the
RX-descriptor `TSFL`). The hardware timestamp exists; the OS-facing drivers just
don't surface it. devourer does — which is what makes TSF-based sub-µs sync
possible where PTP returns an error.

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
