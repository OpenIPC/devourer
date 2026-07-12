# devourer

**The Realtek 11ac driver that simply devours its competitors.**

Devourer is a userspace Wi-Fi driver for Realtek's 802.11ac USB adapters —
the cheap, everywhere-available dongles that power most long-range FPV video
links. It talks to the chip directly over libusb: no kernel module, no DKMS
tree to patch every time your kernel updates, no root filesystem to taint.
Build one static library, link it, and you have raw monitor-mode RX and
packet injection on three generations of Realtek silicon, from a single API.

It is the [OpenIPC](https://openipc.org) project's driver of choice for
long-range digital video links.

## Why devourer

- **No kernel driver, no driver hell.** Everything runs in your process via
  libusb. The same code works on Linux, macOS, Windows, and Android (Termux)
  — including platforms the vendor drivers never supported.
- **Faster on-air than the kernel driver.** Ready-to-receive and
  ready-to-transmit come up quicker than the vendor `.ko` on every supported
  chip, and raw injection skips the kernel networking stack the vendor driver
  drags every frame through (mac80211 → cfg80211 → qdisc → skb → driver xmit) —
  so it sustains the same channel occupancy at 3–4× less host CPU, which matters
  most on an embedded transmitter ([numbers](docs/performance-tuning.md)).
- **Per-packet control.** Every injected frame carries its own radiotap
  header: rate, bandwidth, guard interval, coding, STBC — even TX power and
  channel — can change frame by frame. That turns one dongle into an
  adaptive-link engine: unequal error protection for video layers, live
  power control, per-packet frequency hopping.
- **Frequency hopping at FHSS speed.** A channel hop costs ~0.5–2.5 ms
  depending on chip — fast enough to hop on every packet
  ([how](docs/frequency-hopping.md)).
- **Narrowband modes the kernel can't do.** 5 and 10 MHz channels on every
  supported generation — including the decade-old RTL8812AU and RTL8814AU the
  vendor never gave narrowband — half/quarter the bandwidth, more range from
  the same power ([how](docs/narrowband.md)).
- **Hardware time, for coordinating radios.** Every received frame is stamped
  with the chip's microsecond MAC clock (TSF) on all three generations, and the
  64-bit timer reads back directly — the primitive multi-radio setups need.
  Independent receivers correlate their clocks to sub-microsecond, and a
  time-division burst schedule locks to a transmitter ~25× tighter than the host
  clock manages — enough to interleave a robust narrowband link and a wide
  high-throughput one on one shared channel
  ([bandwidth TDMA](docs/narrowband.md)).
- **Aggregation and hardware ACKs in userspace.** USB TX aggregation, 802.11
  A-MPDU for +30% on-air goodput, and a hardware ACK/BlockAck responder that
  turns unicast into a reliable hardware-ARQ link — with per-frame TX-status
  reports as the transmit-side link sensor ([how](docs/aggregation.md)).
- **A radio lab in a dongle.** Channel sounding, per-antenna signal quality,
  beamforming report capture (enough to do
  [motion sensing](docs/beamforming-victim-sensing.md)), spectrum sweeps,
  link-health diagnosis that tells you whether to add or *back off* power.
- **Clean library API.** One `DeviceConfig` struct at construction, runtime
  setters for everything that changes mid-flight, zero environment-variable
  magic inside the library.

New to low-level RF? Start with the [visual RF primer](docs/rf-primer.md) —
eight short animations that make the rest click. Its sibling, the
[visual driver primer](docs/driver-primer.md), does the same for the silicon
and vendor-driver vocabulary (firmware, efuse, DMAC/CMAC, halbb/halrf, IQK…).

## Supported hardware

Bandwidth cells are devourer's measured on-air TX throughput (Mbps, HT MCS7,
20 MHz) per band:

| Part                          | RF / streams      | 2.4 GHz (ch6) | UNII-1 (ch36) | UNII-2/3 (ch149) | Notes                                       |
| ----------------------------- | ----------------- | ------------- | ------------- | ---------------- | ------------------------------------------- |
| **RTL8812AU**                 | 2T2R              | 56            | 52            | 52               | [CHANEVE CHW50L](https://www.aliexpress.com/item/4000762461362.html) (`0bda:8812`). 5/10 MHz capable |
| **RTL8811AU**                 | 1T1R              | —             | —             | —                | 1T1R cut of 8812 silicon; rides the 8812 code path. Not benchmarked. 5/10 MHz capable |
| **RTL8814AU**                 | 4T4R, 3-SS max    | 65            | †(32)         | †(32)            | `0bda:8813`; tested on COMFAST CF-938AC and CF-960AC — antenna builds differ in realised [RX diversity](docs/measuring-spatial-diversity.md). 5/10 MHz capable |
| **RTL8821AU**                 | 1T1R + BT         | 54            | 32            | 28               | TP-Link Archer T2U Plus (`2357:0120`) |
| **RTL8822BU**                 | 2T2R + BT         | 52            | 50            | 49               | TP-Link Archer T3U (`2357:012d`). 5/10 MHz capable |
| **RTL8812BU**                 | 1T1R + BT         | —             | —             | —                | 1T1R cut of 8822B silicon; rides the 8822BU code path. Not benchmarked. 5/10 MHz capable |
| **RTL8811CU**                 | 1T1R + BT         | 36            | 29            | 28               | COMFAST CF-811AC (`0bda:c811`). 5/10 MHz capable |
| **RTL8821CU**                 | 1T1R + BT         | —             | —             | —                | rides the 8811CU (8821C) code path. 5/10 MHz capable |
| **RTL8812CU**                 | 2T2R              | 65            | 60            | 60               | LB-LINK WDN1300H (`0bda:c812`). 5/10 MHz capable |
| **RTL8822CU**                 | 2T2R + BT         | —             | —             | —                | not benchmarked (`0bda:c82c`). 5/10 MHz capable |
| **RTL8812EU**                 | 2T2R              | ‡             | 51            | 47               | LB-LINK BL-M8812EU2 (`0bda:a81a`); bare 5 GHz FPV module. 5/10 MHz capable. ‡ 2.4 GHz TX airs energy but no receiver decodes it — the vendor kernel driver behaves identically on this module ([quirks](docs/8822e-quirks.md)) |
| **RTL8822EU**                 | 2T2R + BT         | —             | —             | —                | not benchmarked. 5/10 MHz capable |
| **RTL8821CE** (PCIe)          | 1T1R + BT         | —             | —             | —                | Radxa X4 onboard Wi-Fi (`10ec:c821`); not benchmarked |

`†` = works on-air but the reading varies run-to-run (bracketed = best clean
reading).

These cells are single-frame injection (the default TX path), measured as
channel occupancy × PHY rate. A-MPDU (`SetAmpduMode`) does **not** move them on
a chip already near the PHY ceiling — it raises *goodput* (delivered payload)
~30% at MCS7/20 by amortizing per-frame overhead, which an occupancy metric
can't show. See [aggregation & hardware ACK](docs/aggregation.md).

Out of scope: the pre-HalMAC PCIe parts (RTL8812AE/8821AE) and the 11ax
"Kestrel" generation — same branding, different bus or baseband.

> Heads up — some Realtek sticks ship in "ZeroCD" mode and first enumerate as
> a USB flash drive holding a Windows installer (`0bda:1a2b` is the canonical
> offender). If the device won't open, check `lsusb`; `usb_modeswitch` flips
> it to the real NIC.

## Quick start

Toolchain: CMake ≥ 3.15, a C++20 compiler, libusb-1.0.

```sh
# Debian/Ubuntu
sudo apt install build-essential cmake pkg-config libusb-1.0-0-dev
# macOS (Homebrew)
brew install cmake pkg-config libusb

cmake -S . -B build
cmake --build build -j
```

On Windows, install libusb via vcpkg (`vcpkg install libusb`) and set
`VCPKG_ROOT` before configuring.

Then, with a supported dongle plugged in:

```sh
sudo ./build/rxdemo                    # receive: monitor mode, prints frames
sudo ./build/txdemo                    # transmit: injects a test beacon
DEVOURER_CHANNEL=100 DEVOURER_TX_RATE=MCS7/40 sudo -E ./build/txdemo
```

The demos find the first supported adapter automatically; `DEVOURER_PID` /
`DEVOURER_VID` pin a specific one. Every configuration knob the demos accept
is an environment variable — the complete catalogue, with value grammar, is
the `env:` tags in [`src/DeviceConfig.h`](src/DeviceConfig.h).

### Example binaries

| binary | what it shows |
|---|---|
| `rxdemo` | monitor-mode RX loop with per-frame signal telemetry |
| `txdemo` | packet injection, rate/power/channel control, hopping |
| `streamtx` / `duplex` | stdin-driven TX / full-duplex packet link |
| `svctx` | per-video-layer rate ladders (unequal error protection) |
| `txpower` | runtime TX-power API walkthrough |
| `tdma` | TSF-slotted burst TDMA (narrowband ↔ wide on one channel) |
| `timesync` | over-the-air clock distribution (master / slave / UE roles) |
| `sense` | Wi-Fi motion sensing from beamforming reports |
| `doctor` | adapter-health triage → HEALTHY / SUSPECT / FAILING |
| `pcieprobe` | PCIe transport bring-up validation, layer by layer |
| `precoder` | OFDM subcarrier shaping proof-of-concept |

All chips compile in by default; per-chip CMake options (`DEVOURER_JAGUAR1`,
`DEVOURER_8814`, `DEVOURER_JAGUAR2_8822B`, `DEVOURER_JAGUAR2_8821C`,
`DEVOURER_JAGUAR3_8822C`, `DEVOURER_JAGUAR3_8822E`) drop unneeded firmware
and tables — an 8812AU-only `rxdemo` is ~1.0 MB versus ~2.6 MB with
everything on.

## Using the library

You own libusb: init it, open the device, detach any kernel driver, claim
interface 0 — then hand the handle to the factory. `examples/rx/main.cpp` is
the full boilerplate; the minimal RX path is:

```cpp
auto logger = std::make_shared<Logger>();
WiFiDriver driver(logger);
auto dev = driver.CreateRtlDevice(handle);     // handle is already claimed
dev->Init(packetProcessor, SelectedChannel{
    .Channel      = 36,
    .ChannelOffset = 0,
    .ChannelWidth = CHANNEL_WIDTH_20,
});
```

`packetProcessor` is your `void(const Packet&)` callback. For TX, call
`InitWrite` and then `send_packet(buffer, len)`, where the buffer starts with
a radiotap header describing how the frame should fly.

Construction-time options travel in a `devourer::DeviceConfig`
([`src/DeviceConfig.h`](src/DeviceConfig.h) documents every field):

```cpp
devourer::DeviceConfig cfg;
cfg.rx.keep_corrupted = true;                  // deliver CRC-failed frames too
auto dev = driver.CreateRtlDevice(handle, ctx, lock, cfg);
```

Anything that changes mid-session is a runtime setter on the device:
`SetTxMode`, `SetTxPowerOffsetQdb`, `SetRxPathMask`, `FastRetune`, ...
The device class is chosen automatically from the chip behind the handle;
one `IRtlDevice` interface covers all three generations.

## Going deeper

**Primers** — start here:

- [Visual RF primer](docs/rf-primer.md) — animated intro to the concepts
  behind everything below.
- [Visual driver primer](docs/driver-primer.md) — animated intro to the chip
  and vendor-driver machinery: registers, efuse, firmware, MAC, PHY tables,
  calibration, coexistence.

**Link engineering:**

- [Adaptive link](docs/adaptive-link.md) — the energy-minimizing video-link
  controller design, [its validation](docs/adaptive-link-validation.md), and
  the [building blocks](docs/adaptive-link-building-blocks.md): what each knob
  (power, rate, bandwidth, hopping) measurably buys.
- [Fused FEC](docs/fused-fec.md) — the cross-layer error-protection stack:
  per-layer PHY rates, corrupt-frame salvage, outer erasure code.
- [Aggregation & hardware ACK](docs/aggregation.md) — USB TX aggregation,
  per-frame CCX TX-status reports, 802.11 A-MPDU (`SetAmpduMode`, +30% on-air
  goodput), and the hardware ACK/BlockAck responder for reliable-unicast links.
- [wfb-ng tuning](docs/wfb-ng-tuning.md) — the most efficient wfb-ng
  configuration, and the SDR-measured devourer-vs-wfb-ng TX comparison.

**Spectrum agility:**

- [Frequency hopping](docs/frequency-hopping.md) — how per-packet hopping
  works and what it costs on each chip.
- [FHSS](docs/fhss.md) — the anti-jam design article: keyed SipHash hop
  schedules, slot-locked lockstep RX, and
  [jammer resilience](docs/jammer-resilience.md) — measured delivery against
  parked and following jammers, and where a follower breaks.
- [Narrowband](docs/narrowband.md) — 5/10 MHz channels across all three
  generations: the baseband re-clock, the per-chip register machinery, and the
  walls (RF re-latch edges, per-die clock coupling, the 5 MHz/5 GHz CFO limit).
- [Spectrum sensing](docs/rx-spectrum-sensing.md) — RX energy sweeps down to
  5 MHz bins: a coarse per-bin H(f) from the dongle itself.
- [Pseudo preamble puncturing](docs/pseudo-preamble-puncturing.md) — how close
  per-tone RX masks/notches get to using a wide channel with a dirty slice.

**Timing & coordination:**

- [Time distribution](docs/time-distribution.md) — LTE-eNB-style over-the-air
  clock distribution off the hardware beacon TSF: sub-µs downlink, TSF adoption,
  µs-fine TBTT steering and a converging closed-loop uplink timing advance.
- [Timing accuracy](docs/timing-accuracy.md) — measured comparison vs NTP/PTP over
  Wi-Fi (why the hardware TSF beats software timestamps ~3000×, why PTP can't run),
  and the USB-vs-PCIe transport-latency microbench (`tests/reglat.cpp`).
- [AP mode](docs/ap-mode.md) — devourer as an infrastructure access point a real
  Linux station associates with: beacon → probe/auth/assoc → DHCP/ARP/ICMP → ping,
  open or WPA2-PSK (4-way handshake + software CCMP), validated against rtw88.
- [Scheduled MAC](docs/scheduled-mac.md) — four measured contracts under a slot
  scheduler: submit→air guard time, dynamic beacon grants, hardware ACK/TxReport,
  per-UE RX attribution.
- [Multi-AP cellular](docs/multi-ap-cellular.md) — what the shared clock
  enables: coordinated cells, make-before-break handover, roaming robot UEs.

**Measurement & instrumentation:**

- [LA-mode IQ capture](docs/la-capture.md) — raw complex baseband into the TX
  packet buffer; per-tone H(k)/CSI offline, from the dongle alone.
- [Spatial diversity](docs/measuring-spatial-diversity.md),
  [bench testing near-field](docs/bench-testing-near-field.md) — measurement
  guides for the built-in radio instrumentation.
- [Beamforming self-sounding](docs/beamforming-self-sounding.md) — per-subcarrier
  CSI from two adapters via the VHT sounding exchange; and its sibling
  [victim sensing](docs/beamforming-victim-sensing.md) — motion sensing from
  captured beamforming reports.
- [Adapter doctor](docs/adapter-doctor.md) — dying-dongle triage: EFUSE
  read-stability, firmware-boot and RX-smoke probes with a
  HEALTHY / SUSPECT / FAILING verdict.
- [Performance](docs/performance-tuning.md) — devourer vs. kernel driver on
  startup time, on-air throughput, and host CPU (3–4× lower); the TX
  submission modes and the tuning levers, with the methodology.

**Chip specifics & internals:**

- [8822E quirks](docs/8822e-quirks.md) — the RTL8812EU/8822EU definitive
  quirks list: what the chip needs, what devourer does, the reproducers.
- [Logging](docs/logging.md) — the two-plane output schema: JSONL machine
  events on stdout, human diagnostics on stderr.

## Testing

Headless selftests run with `ctest`. Hardware regression is
`tests/regress.py`: a TX/RX matrix between devourer and the kernel driver
across plugged-in adapters, with optional full-pair, encoding-sweep, and
third-adapter-sniffer modes — see [`tests/README.md`](tests/README.md).

## License

GPL-2.0. See [LICENSE](LICENSE).
