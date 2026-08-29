# devourer

**The Realtek Wi-Fi driver that simply devours its competitors.**

Devourer is a userspace Wi-Fi driver for Realtek's 802.11n, 802.11ac and 802.11ax
USB adapters —
the cheap, everywhere-available dongles that power most long-range FPV video
links. It talks to the chip directly over libusb: no kernel module, no DKMS
tree to patch every time your kernel updates, no root filesystem to taint.
Build one static library, link it, and you have raw monitor-mode RX and
packet injection across five Realtek hardware backends, from a single API.

It is the [OpenIPC](https://openipc.org) project's driver of choice for
long-range digital video links.

## Why devourer

- **No kernel driver, no driver hell.** Everything runs in your process via
  libusb — on Linux, macOS, Windows and Android alike, including platforms the
  vendor drivers never supported. On Android that means no root and no custom
  kernel: [PixelPilot](https://github.com/OpenIPC/PixelPilot) uses devourer as
  the receive path of an FPV ground station running on an ordinary phone, with
  the adapter opened straight from the USB permission the app is granted.
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
- **Frequency hopping at FHSS speed, and a hopset that adapts.** A channel hop
  costs ~0.5–2.5 ms depending on chip — fast enough to hop on every packet
  ([how](docs/frequency-hopping.md)) — in a keyed order an observer cannot
  predict. The link also learns: a channel that stops delivering is dropped
  from the schedule by authenticated agreement between the two ends, and
  revisited later by keyed probes in case it recovers
  ([how](docs/fhss.md)).
- **Wi-Fi 6, on the same API.** The 802.11ax parts (RTL8852BU/8852CU) run one
  HAL over both dies, with HE injection, 160 MHz on the 8852C and 6 GHz on the
  8832CU. Including the standard's long-range corner — **HE ER SU + DCM**, worth
  roughly 8–10 dB stacked, and the only extended-range lever that works against
  *someone else's* 802.11ax gear ([how](docs/he-extended-range.md)). Trigger
  frames air correctly for scheduled-uplink work, though the hardware-timed
  response needs AP firmware these parts don't ship
  ([what closes, what doesn't](docs/he-trigger-ul.md)).
- **A link that changes channel on evidence, not on a timer.** Beside per-slot
  hopping there's the slow lever: a second adapter surveys candidates while the
  video keeps flowing, delivery on the live channel decides when a move is
  worth it, and the two ends migrate under an authenticated
  ground-proposes/drone-commits protocol that cannot split-brain
  ([how](docs/adaptive-channel-migration.md)).
- **Narrowband modes the kernel can't do.** 5 and 10 MHz channels on the
  backends that advertise them — including the decade-old RTL8812AU and
  RTL8814AU the vendor never gave narrowband — half/quarter the bandwidth,
  more range from the same power. RTL8733B has an experimental path that stays
  unadvertised pending RF validation ([how](docs/narrowband.md)).
- **Hardware time, for coordinating radios.** Every received frame is stamped
  with the chip's microsecond MAC clock (TSF) on every generation, and the
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
fifteen short animations that make the rest click. Its sibling, the
[visual driver primer](docs/driver-primer.md), does the same for the silicon
and vendor-driver vocabulary (firmware, efuse, DMAC/CMAC, halbb/halrf, IQK…).

## Supported hardware

Bandwidth cells are devourer's measured on-air TX throughput (Mbps, HT MCS7,
20 MHz) per band:

| Part                          | RF / streams      | 2.4 GHz (ch6) | UNII-1 (ch36) | UNII-2/3 (ch149) | 6 GHz (ch5) | Notes                                       |
| ----------------------------- | ----------------- | ------------- | ------------- | ---------------- | ---------------- | ------------------------------------------- |
| **RTL8812AU**                 | 2T2R              | 56            | 52            | 52               | —                | [CHANEVE CHW50L](https://www.aliexpress.com/item/4000762461362.html) (`0bda:8812`). 5/10 MHz capable |
| **RTL8811AU**                 | 1T1R              | —             | —             | —                | —                | 1T1R cut of 8812 silicon; rides the 8812 code path. Not benchmarked. 5/10 MHz capable |
| **RTL8814AU**                 | 4T4R, 3-SS max    | 65            | †(32)         | †(32)            | —                | `0bda:8813`; tested on COMFAST CF-938AC and CF-960AC — antenna builds differ in realised [RX diversity](docs/measuring-spatial-diversity.md). 5/10 MHz capable |
| **RTL8821AU**                 | 1T1R + BT         | 54            | 32            | 28               | —                | TP-Link Archer T2U Plus (`2357:0120`) |
| **RTL8822BU**                 | 2T2R + BT         | 52            | 50            | 49               | —                | TP-Link Archer T3U (`2357:012d`). 5/10 MHz capable |
| **RTL8812BU**                 | 1T1R + BT         | —             | —             | —                | —                | 1T1R cut of 8822B silicon; rides the 8822BU code path. Not benchmarked. 5/10 MHz capable |
| **RTL8811CU**                 | 1T1R + BT         | 36            | 29            | 28               | —                | COMFAST CF-811AC (`0bda:c811`). 5/10 MHz capable |
| **RTL8821CU**                 | 1T1R + BT         | —             | —             | —                | —                | rides the 8811CU (8821C) code path. 5/10 MHz capable |
| **RTL8812CU**                 | 2T2R              | 65            | 60            | 60               | —                | LB-LINK WDN1300H (`0bda:c812`). 5/10 MHz capable |
| **RTL8822CU**                 | 2T2R + BT         | —             | —             | —                | —                | not benchmarked (`0bda:c82c`). 5/10 MHz capable |
| **RTL8812EU**                 | 2T2R              | ‡             | 51            | 47               | —                | LB-LINK BL-M8812EU2 (`0bda:a81a`); bare 5 GHz FPV module. 5/10 MHz capable. ‡ 2.4 GHz TX airs energy but no receiver decodes it — the vendor kernel driver behaves identically on this module ([quirks](docs/8822e-quirks.md)) |
| **RTL8822EU**                 | 2T2R + BT         | —             | —             | —                | —                | not benchmarked. 5/10 MHz capable |
| **RTL8731BU**                 | 1T1R              | —             | —             | —                | —                | bare unbranded 1T1R module (`0bda:f72b`, cut D): monitor RX and CCK/legacy/HT raw TX validated on 2.4 GHz, legacy/HT on 5 GHz, 20/40 MHz. Not benchmarked. [Status and test limits](docs/rtl8733b.md) |
| **RTL8733BU**                 | 1T1R + BT         | 62            | 50            | 50               | —                | LB-LINK BL-M8733BU2-L (`0bda:b733`); rides the RTL8731BU (RTL8733B) code path |
| **RTL8821CE** (PCIe)          | 1T1R + BT         | —             | —             | —                | —                | Radxa X4 onboard Wi-Fi (`10ec:c821`); not benchmarked |
| **RTL8852BU** (11ax)          | 2T2R + BT         | 43            | 36            | 33               | —          | TP-Link Archer TX20U Nano (`35bc:0108`); Wi-Fi 6, dual-band. 5/10 MHz capable; HE ER SU + DCM extended range |
| **RTL8832BU** (11ax)          | 2T2R              | —             | —             | —                | —          | Wi-Fi-only SKU of the 8852B die; rides the 8852BU code path. Not benchmarked. 5/10 MHz capable; HE ER SU + DCM extended range |
| **RTL8832CU** (11ax)          | 2T2R + BT         | 40            | 33            | 32               | 32          | TP-Link Archer TX50UH (`35bc:0101`); Wi-Fi 6E tri-band (2.4/5/6 GHz). 5/10 and 160 MHz capable; HE ER SU + DCM extended range. Host-push injection over USB 2.0 (~50% duty ceiling); [6G TX+RX validated](tests/kestrel_8832cu_6g_txrx.sh) |
| **RTL8852CU** (11ax)          | 2T2R + BT         | —             | —             | —                | —          | "8852" branding of the same 8852C die; rides the 8832CU code path. Not benchmarked. 5/10 and 160 MHz capable; HE ER SU + DCM extended range |

`†` = works on-air but the reading varies run-to-run (bracketed = best clean
reading).

These cells are single-frame injection (the default TX path), measured as
channel occupancy × PHY rate. A-MPDU (`SetAmpduMode`) does **not** move them on
a chip already near the PHY ceiling — it raises *goodput* (delivered payload)
~30% at MCS7/20 by amortizing per-frame overhead, which an occupancy metric
can't show. See [aggregation & hardware ACK](docs/aggregation.md).

Out of scope: the pre-HalMAC PCIe parts (RTL8812AE/8821AE). The 11ax
"Kestrel" generation (RTL8852BU / RTL8852CU, a fourth HAL under `src/kestrel/`,
vendor references `reference/rtl8852bu` + `reference/rtl8852cu`) has RX, TX, and
channel/bandwidth (5/10/20/40/80 MHz on both dies, 160 MHz on the 8852C)
on-air validated — and the tri-band **RTL8832CU** adds 6 GHz (WiFi 6E),
benchmarked above at ~5 GHz-parity throughput. The BB/RF plane is Realtek's own
halbb/halrf C compiled verbatim (register tables, per-channel config, DACK/
RX-DCK, plus IQK on the 8852C); TSSI/DPK on both dies and IQK on the 8852B are
gated off with on-air evidence — they degrade TX under the fixed-power model.
The 8852A-family (e.g. RTL8832AU) stays out of scope — its only vendor driver
is a frozen 2021 drop.

The RTL8733B backend covers the 1T1R 802.11n RTL8731BU/RTL8733BU family with
20/40 MHz monitor RX and raw CCK/OFDM/HT injection (CCK is 2.4 GHz, 20 MHz,
long preamble only). It intentionally does not advertise VHT, LDPC, SGI, STBC,
or experimental 5/10 MHz operation; see
[the validation record](docs/rtl8733b.md) for the tested and deferred matrix.

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
| `chanmig` / `chanscout` | evidence-driven channel migration: the protocol, and the survey adapter that feeds it |
| `dwelltx` | dwell-1 hopping data plane on the standard Linux driver |
| `kestrelprobe` | Wi-Fi 6 (RTL8852B/C) bring-up probe, layer by layer |
| `rtl8733bprobe` | RTL8731BU/RTL8733BU identity, power, firmware and PHY diagnostics |
| `timesync` | over-the-air clock distribution (master / slave / UE roles) |
| `sense` | Wi-Fi motion sensing from beamforming reports |
| `doctor` | adapter-health triage → HEALTHY / SUSPECT / FAILING |
| `pcieprobe` | PCIe transport bring-up validation, layer by layer |
| `precoder` | OFDM subcarrier shaping proof-of-concept |

All chips compile in by default; per-chip CMake options (`DEVOURER_JAGUAR1`,
`DEVOURER_8814`, `DEVOURER_JAGUAR2_8822B`, `DEVOURER_JAGUAR2_8821C`,
`DEVOURER_JAGUAR3_8822C`, `DEVOURER_JAGUAR3_8822E`, `DEVOURER_8733B`,
`DEVOURER_KESTREL_8852B`, `DEVOURER_KESTREL_8852C`) drop unneeded firmware and
tables — an 8812AU-only
`rxdemo` is ~1.6 MB against ~6.3 MB with everything on, and dropping just the
two Wi-Fi 6 dies takes it to ~4.2 MB (their verbatim-vendored halbb/halrf plane
is the single largest contributor). `DEVOURER_PCIE` (default OFF, Linux-only)
adds the vfio-pci transport for the RTL8821CE.

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
one `IRtlDevice` interface covers all five hardware backends.

## Going deeper

**Start here.** There is a lot below; this is the order that works. Read the
[visual RF primer](docs/rf-primer.md) first — fifteen animations covering the
concepts every other doc assumes (subcarriers, EVM, AGC, hopping, OFDMA,
extended range). Then pick the one thing you came for: building a video link →
[adaptive link](docs/adaptive-link.md); surviving interference →
[FHSS](docs/fhss.md); getting more range →
[narrowband](docs/narrowband.md); coordinating several radios →
[time distribution](docs/time-distribution.md); making the driver itself do
something new → [visual driver primer](docs/driver-primer.md), then
[logging](docs/logging.md) for the event schema every test script reads. If a
chip is misbehaving, skip to [adapter doctor](docs/adapter-doctor.md) and the
per-chip quirks notes at the bottom.

**Primers:**

- [Visual RF primer](docs/rf-primer.md) — animated intro to the concepts
  behind everything below.
- [Visual driver primer](docs/driver-primer.md) — animated intro to the chip
  and vendor-driver machinery: registers, efuse, firmware, MAC, PHY tables,
  calibration, coexistence, firmware offload.

**Wi-Fi 6 (802.11ax):**

- [HE extended range](docs/he-extended-range.md) — the ER SU / DCM range
  ladder, what each rung buys and costs, and the on-air matrix across both
  dies.
- [HE trigger-based uplink](docs/he-trigger-ul.md) — Trigger frames, resource
  units, TWT and sounding: the API, and an honest account of which paths the
  shipped client firmware executes and which it silently drops.

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
  works and what it costs on each chip, including the 8822B/C/E firmware
  channel-switch fast path (`DEVOURER_FASTRETUNE_FW`).
- [Kernel channel-switch baseline](docs/experiments/kernel-channel-switch-baseline.md) +
  [firmware offload](docs/experiments/kernel-channel-switch-offload.md) +
  [MCC/FCS](docs/experiments/mcc-fcs-investigation.md) +
  [dwell-1 A/B injection](docs/experiments/dwell1-ab-injection.md) +
  [N-channel hopping](docs/experiments/n-channel-hopping.md) — how the standard
  Linux/Realtek drivers retune measured against devourer, where the chip
  firmware's own H2C 0x1D switch beats them, and a two-context per-slot data
  plane with zero wrong-channel over 100 k slots.
- [FHSS](docs/fhss.md) — the anti-jam design article: keyed SipHash hop
  schedules, slot-locked lockstep RX, and
  [jammer resilience](docs/jammer-resilience.md) — measured delivery against
  parked and following jammers, and where a follower breaks. It carries on
  into the adaptive half: how the two ends agree on a change to the hopset
  without either trusting the other, why the endpoint that must decode is the
  one that decides, why a transmitter may argue only that a move leaves *it*
  worse off rather than that it disagrees, and what stops an adversary who can
  make channels look bad from herding the link onto one it then jams.
- [Adaptive channel migration](docs/adaptive-channel-migration.md) — the slow
  counterpart to hopping: a scout adapter surveying candidates while the video
  keeps flowing, a scoring engine where the receiver's delivery is
  authoritative, and a gate that only moves a working link on evidence. The
  [wire protocol](docs/channel-migration-protocol.md) is the authenticated
  ground-proposes/drone-commits exchange, and
  [its validation](docs/channel-migration-validation.md) is the failure matrix
  every row of which converges without split-brain.
- [Narrowband](docs/narrowband.md) — 5/10 MHz channels across the Jaguar
  generations, plus the unadvertised RTL8733B experiment: the baseband
  re-clock, the per-chip register machinery, and the walls (RF re-latch edges,
  per-die clock coupling, the 5 MHz/5 GHz CFO limit).
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
- [8852C quirks](docs/8852c-quirks.md) — the same for the Wi-Fi 6 die,
  including which sensing facilities are 2.4 GHz-only and why.
- [Logging](docs/logging.md) — the two-plane output schema: JSONL machine
  events on stdout, human diagnostics on stderr.

## Testing

Headless selftests run with `ctest`. Hardware regression is
`tests/regress.py`: a TX/RX matrix between devourer and the kernel driver
across plugged-in adapters, with optional full-pair, encoding-sweep, and
third-adapter-sniffer modes — see [`tests/README.md`](tests/README.md).

## License

GPL-2.0. See [LICENSE](LICENSE).
