# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with
code in this repository. It holds **cross-cutting** facts only. Deep subtree
facts live in nested `CLAUDE.md` files, auto-loaded when working there:
`src/{jaguar1,jaguar2,jaguar3,kestrel,rtl8733b}/` for per-generation registers,
descriptors and per-chip mechanisms; `src/hopset/` for keyed FHSS and the
adaptive hopset; `src/chanmig/` for channel migration. Add new facts to the
narrowest file that covers them.

Two standing rules for this file: never duplicate what a header already
doc-comments (point at `src/DeviceConfig.h`, `src/TxPower.h`,
`src/AdapterCaps.h` instead), and never quote a favourable measurement without
its adversarial counterpart in the same breath.

## What this is

Userspace re-implementation of Realtek's USB Wi-Fi drivers (11n RTL873x, 11ac
RTL88xx and 11ax RTL8852 families) — speaks to the chip directly via libusb
instead of a kernel module. Static library `devourer` (CMake target) + example
executables under `examples/` (`rxdemo` and `txdemo` are the canonical RX/TX
demos). Used by the OpenIPC project for long-range video links.

Five hardware backends, each behind its own self-contained HAL, dispatched at
construction from the `SYS_CFG2` chip-id (Kestrel: PID-first):

- **Jaguar1** (`src/jaguar1/`): RTL8812AU (2T2R reference), RTL8811AU (1T1R
  cut, rides the 8812 path), RTL8814AU (4T4R RF / 3-SS baseband — a failed
  FW-boot poll of its 3081 MCU means dead TX while RX still works), RTL8821AU
  (1T1R + BT). 5/10 MHz narrowband on the 8812AU/8811AU/8814AU
  (bench-characterized TX+RX); the 8821A is excluded and falls back to
  20 MHz. See `docs/narrowband.md`.
- **Jaguar2** (`src/jaguar2/`): RTL8822BU / RTL8812BU (chip-id `0x0a`) and
  RTL8811CU / RTL8821CU (chip 8821C, chip-id `0x09`). A HalMAC + phydm
  hybrid. RX + TX on 2.4/5 GHz at 20/40/80 MHz plus 5/10 MHz narrowband on
  both variants; 5 MHz at 5 GHz is CFO-limited.
- **Jaguar3** (`src/jaguar3/`): rtl8822c (RTL8812CU/8822CU, chip-id `0x13`)
  and rtl8822e (RTL8812EU/8822EU, chip-id `0x17`). 5/10 MHz narrowband,
  80 MHz (incl. a 40-in-80 frame), halrf calibration. Sustained 5 GHz TX
  needs the coex runtime thread (started in `InitWrite`) — without it the
  combo chip's coex firmware silences the antenna. 8822e constraints:
  `docs/8822e-quirks.md`.
- **Kestrel** (`src/kestrel/`): the Wi-Fi 6 / 802.11ax generation —
  RTL8852BU/8832BU (variant C8852B) and RTL8852CU/8832CU (C8852C), one HAL
  serving both dies on all platforms (MSVC included). Two planes: hand-ported
  mac_ax C++ + Realtek's halbb/halrf C compiled verbatim
  (`hal/hal{bb,rf}/g6/`). Dispatched PID-first (`kestrel/KestrelUsbIds.h`) —
  the 0x00FC byte is not a chip-id on AX silicon. Blacklist rtw89 when
  testing: a kernel driver pre-initializing the chip masks the cold-boot
  path. On-air-validated: monitor RX (both dies, 2.4/5 GHz), TX injection
  (legacy/HT/VHT/HE + HE ER SU/DCM — `docs/he-extended-range.md`),
  5/10/20/40/80 MHz on both dies + 160 MHz on the 8852C only; 6 GHz TX tops
  out at 80 MHz (the 6G+160 TX-enable path is un-ported). TX power is a fixed
  BB dBm (`DEVOURER_TX_PWR`, whole dBm here). The 8852A-family (RTL8832AU) is
  deliberately excluded. Quirks: `docs/8852c-quirks.md`.
- **RTL8733B** (`src/rtl8733b/`): the 802.11n generation — RTL8731BU/RTL8733BU
  (chip-id `0x16`), a HALMAC 87xx part, not a Jaguar variant: its own power,
  firmware, MAC, descriptor and PHY paths. 1T1R, and the advertised surface is
  only what an independent witness decoded — legacy OFDM + HT MCS0-7, BCC,
  20/40 MHz on 2.4/5 GHz, plus long-preamble CCK on 2.4 GHz at 20 MHz, plus
  10 MHz narrowband (5 MHz refused — `src/rtl8733b/CLAUDE.md`).
  Everything the backend has not ported (TSF/beacons, hardware ACK, A-MPDU,
  the flat-index and per-rate TX-power knobs) falls through to `IRtlDevice`'s
  not-ported defaults rather than being faked, so read the base class before
  assuming a cross-generation feature below applies here. `FastRetune` IS
  ported (intra-band, TSSI kept live — `src/rtl8733b/CLAUDE.md`). SGI, LDPC, STBC, VHT
  and HE are refused outright. Scope, one-unit validation record and the
  deferred matrix: `docs/rtl8733b.md`.

Naming traps: **RTL8821AU is Jaguar1** (not Jaguar2, despite the Jaguar2
RTL8821C's similar name); RTL8822**B**U (Jaguar2) ≠ RTL8822**C**U (Jaguar3);
the TP-Link TX50UH is RTL8832**C**U (8852C-family) despite lab lore calling it
8832AU, while the TX20U **Nano** is RTL8852BU; RTL8733B**U** is USB and
supported, RTL8733B**S** is the SDIO sibling and has no transport here. Full
chip / bench-throughput table: README **Supported hardware**.

**PCIe** (`DEVOURER_PCIE=ON`, Linux-only, default OFF): the RTL8821CE — the
PCIe sibling of the 8821CU — rides the same Jaguar2 HAL through a vfio-pci
transport (`src/PcieTransport.{h,cpp}`: BAR2 MMIO registers over the same
0x0000..0xFFFF space the USB vendor-control path addresses, 88xx
buffer-descriptor DMA rings for TX/RX). USB and PCIe are independent
transports behind `devourer::IRtlTransport` (`src/RtlTransport.h`); the
bus-neutral `RtlAdapter` the HALs hold forwards to whichever it was built
with. The few genuinely bus-specific bring-up steps gate on `is_usb()` (PCIe
power-seq rows, PQ map, no USB RX-agg, no DLFW 512-pad) or ride `hci_setup()`
(pre-power TRX ring programming, no-op on USB). Factory:
`WiFiDriver::CreateRtlDevicePcie(PcieTransport::Open(bdf, logger))` — the
caller owns vfio like it owns libusb. Demos: `DEVOURER_PCIE_BDF=0000:01:00.0`
on rxdemo and txdemo; `pcieprobe <bdf>` validates the layers bottom-up.
Bind/restore: `tests/pcie_vfio_bind.sh` — driver_override, **not** new_id,
because of the in-tree rtw88 auto-probe race. Validation: `sudo python3
tests/pcie_rx_smoke.py` against a vfio-bound 8821CE.

## Build

```sh
cmake -S . -B build
cmake --build build -j
```

libusb-1.0 via `pkg-config` (Linux/macOS) or vcpkg (`VCPKG_ROOT`, Windows).

The vendor kernel drivers under `reference/` are git submodules (fork rationale
+ layout in `reference/README.md`); `git submodule update --init --recursive`
before running `tools/extract_*.py` or the hardware-testing kernel cells.

Per-chip options, all default ON: `DEVOURER_JAGUAR1`, `DEVOURER_8814` (requires
JAGUAR1), `DEVOURER_JAGUAR2_8822B`, `DEVOURER_JAGUAR2_8821C`,
`DEVOURER_JAGUAR3_8822C`, `DEVOURER_JAGUAR3_8822E`, `DEVOURER_8733B`,
`DEVOURER_KESTREL_8852B`, `DEVOURER_KESTREL_8852C`.
`DEVOURER_PCIE` (default OFF, Linux-only, requires
JAGUAR2_8821C) adds the vfio-pci transport + `pcieprobe`; OFF builds are
byte-identical to before it existed. Turning groups off drops their firmware
blobs + PHY tables (an 8812AU-only `rxdemo` is ~1.6 MB vs ~6.3 MB all-on — the
verbatim-vendored Kestrel halbb/halrf plane dominates the difference).
Configure fails on no-chip-selected or 8814-without-JAGUAR1. Each group
exports a PUBLIC `DEVOURER_HAVE_*` define; sites referencing a dropped group
sit behind `#if defined(DEVOURER_HAVE_*)`, and the factory returns `nullptr`
(logs) for a chip whose support isn't built.

`DEVOURER_SANITIZE=address|address+undefined|thread` (default off) builds the
library, demos and selftests instrumented; the vendored halbb/halrf C is
exempted from UBSan (Realtek's sources are full of unaligned loads and signed
shifts — noise we would never act on). MSVC has AddressSanitizer only and
rejects the other values rather than silently degrading. The demos are worth
running under it on hardware, not just `ctest`: the lifetime bugs it finds
(teardown, in-flight URBs) need a real device —
`tests/tx_teardown_asan.sh` (max-duty TX killed mid-flight, incl. a real
VBUS-cut wedge) and `tests/teardown_gen_sanity.sh` (every plugged generation
init + teardown).

CI (`.github/workflows/cmake-multi-platform.yml`): GCC/Clang/MSVC ×
Ubuntu/macOS/Windows matrix, a `build-mingw` job, a `build-configs` matrix over
each per-chip subset, `build-sanitizers` (ASan+UBSan `ctest`), and
`reject-bad-configs` for the invalid option combos.
`ctest` runs in every job (headless selftests — math guards + the
`stream_stdin_binary` framing round-trip). Hardware testing is out-of-band.

## Hardware testing

`tests/regress.py` — 2×2 TX/RX matrix (devourer vs. kernel driver) over two
plugged adapters. Run after building.

```sh
sudo python3 tests/regress.py                 # local kernel cells
sudo python3 tests/regress.py \
    --vm-name devourer-testrig --vm-ssh <user>@<VM-IP>   # pinned-kernel VM
```

VM mode (provision once with `tests/setup_vm.sh`) is required for chips whose
vendor driver doesn't build on bleeding-edge kernels — notably the RTL8814AU.
Default channel 6; pass `--channel 36` / `--channel 100` for 5 GHz — a
single-band matrix is not comprehensive. Caveat: with the 8814 as TX, kernel-TX
cells read 0 on every channel (`aircrack-ng/88XXau` doesn't emit host-pushed
*beacon* injection) — judge 8814 TX by the devourer-TX cells.

Layered modes: `--full-matrix` (every ordered DUT pair × 4 driver combos),
`--encoding-matrix --tx-pid --rx-pid` (radiotap encoding combos; the kernel-TX
rows are not authoritative for LDPC/STBC — that driver strips those bits —
so truth-table runs add `--modes devourer:devourer`; per-cell `rx.txhit`
events carry the decoded `rate`/`ldpc`/`stbc` as proof of what flew),
`--sniffer-iface IFACE` (3rd-adapter capture, intended for AR9271).
`--keep-logs` puts per-cell logs at `/tmp/devourer-regress-last/`. Full
semantics: `tests/README.md`.

Startup-time benchmarking: `tests/bench_init.py` (per-stage `init.timing`
events from `src/InitTimer.h`; methodology + numbers in `docs/performance-tuning.md`).
On-air TX throughput: measure **Mbps via SDR duty × PHY rate**
(`tests/bench_onair.py`), never monitor-sniffer frame counts — a sensitive
receiver decodes weak frames and masks a real drop. Keep a known-good control
adapter, re-check it each session, and take one clean SDR read per session (a
second back-to-back `sdr_duty` read can fail to reacquire and report ~0).

Suspect a DUT itself (deaf with a green init, chronic FW-boot fails):
`build/doctor` grades adapter health — EFUSE read-stability ×N, fw-boot,
RX smoke → HEALTHY/SUSPECT/FAILING in the exit code;
`tests/adapter_doctor_cold.sh` wraps it in per-rep VBUS cold + a vouched
flood for a definitive verdict (`docs/adapter-doctor.md`). Two cold-init
traps it encodes: the in-tree rtw88 modules auto-probe (and fw-download
into) every Realtek dongle at each enumeration — `modprobe -r` does NOT
survive re-enumeration, temp-blacklist instead; and `authorized`-toggle
"cold" leaves chip state (real VBUS cold via `REGRESS_VBUS_MAP` /
uhubctl — hub ports with per-port power switching only, not xhci root
ports).

## Logging

Two planes (`docs/logging.md` is the schema source of truth): **machine events**
= JSON Lines on stdout, one object per line, first field always
`{"ev":"<name>",...}` — so `grep -F '"ev":"rx.txhit"'` works without a JSON
parser, and `tests/devourer_events.py` (`iter_events`/`parse_event`) is the
python helper every test script uses. **Human diagnostics** = stderr,
`devourer [I] msg` (level letter T/D/I/W/E). Every line is written with one
fwrite + flush, so piped consumers never stall on buffering and threads never
interleave mid-line. `2>/dev/null` gives a pure event stream.

Demo knobs: `DEVOURER_LOG_LEVEL=trace..silent` (stderr verbosity, default
debug), `DEVOURER_EVENTS=stdout|stderr|off`, `DEVOURER_EVENT_FLUSH=0`
(max-rate benches). Compile-time floor: `-DDEVOURER_LOG_MAX_LEVEL=WARN`
compiles trace/debug out entirely (args included at `DVR_TRACE`/`DVR_DEBUG`
sites) for production builds; unset = NDEBUG-derived. Exceptions kept as
diffable text on the diagnostic plane: canary / bb / efuse / txpwr register
dumps (kernel cross-validation format).

## Configuration

**The library reads no environment.** Construction-time knobs live in
`devourer::DeviceConfig` (`src/DeviceConfig.h` — rx / tx / bf / tuning / debug /
usb sections, every field doc-tagged with its env-var spelling and value
grammar), passed as `CreateRtlDevice`'s defaulted fourth argument. Mid-session
knobs are runtime setters on `IRtlDevice` (`SetTxMode`, `SetTxPowerOffsetQdb`,
`SetTxPowerIndexOverride`, `SetRxPathMask`, `SetCcaMode`, `FastRetune`, ...).

**Adapter capabilities**: `IRtlDevice::GetAdapterCaps()` (`src/AdapterCaps.h`)
aggregates chip identity, chain counts, the composed `GetTxCaps` +
`GetTxPowerCaps`, channel widths, per-band tunable + characterized frequency
spans, and feature flags — resolved at construction, thread-safe, callable
pre-`Init`; the demos emit it as the `adapter.caps` JSONL event. Every flag is
doc-commented at its declaration, including the bench-derived (not
vendor-advertised) `ldpc_rx_*` and `vht_2g4_ok` truth tables and the three
per-packet-TX-power hardware shapes — read the header, not a copy of it.

Two facts that live only here: LDPC TX is per-packet radiotap-driven on all
generations (`TxCaps.ldpc_ok`) with bench-measured coding gain ≈ +3 dB at the
10%-delivery crossing, MCS7/20 MHz (`tests/ldpc_waterfall.sh`) — prefer
`/LDPC` on any link whose RX side can decode it. And `GetActiveRxPaths()` is
the live companion to the static caps: a best-effort per-chain-RSSI estimate
of which antennas actually carry signal (needs an RX loop + traffic).

The 5 GHz synthesizer tunes past the UNII channels (extended range
~5080–6165 MHz, chan up to 253, `freq = 5000 + 5*chan`); out-of-band channels
tune but their TX power / per-channel constants are extrapolated from the
nearest characterized channel (one-shot `W` diagnostic). No regulatory
enforcement — the caller owns compliance.

**Env vars are the demos' interface**: `examples/common/env_config.{h,cpp}` is
the authoritative mapping of every library-level `DEVOURER_*` var onto
`DeviceConfig`, so the test scripts drive everything through env. For the
per-var reference read the `env:` tags in `DeviceConfig.h` — do not look for a
second copy here. Demo-local vars (device selection, timing, telemetry
cadence) have no `DeviceConfig` field and are parsed in each demo's own code;
those are the ones listed below.

- `DEVOURER_PID=0xNNNN` / `DEVOURER_VID=0xNNNN` — restrict the device-open
  loop (default VID `0x0bda`, all Realtek PIDs). `DEVOURER_USB_BUS=N` +
  `DEVOURER_USB_PORT=a.b.c` select by USB topology when two adapters share
  VID:PID **and** serial.
- `DEVOURER_CHANNEL=N` — monitor channel.
- `DEVOURER_TX_RATE=<rate>[/<bw>][/SGI][/LDPC][/STBC][/ER|/ER106][/DCM]` — TX
  mode for rate-less frames (`MCS7/40/SGI`, `VHT2SS_MCS3/80/LDPC`, `1M`...).
  Unset = 6M legacy. CCK rates are 2.4 GHz-only; `1M` buys ~9 dB link budget
  over `6M`. Rate resolution never reads the band, so `VHT*` rates air on
  2.4 GHz too — the non-standard NitroQAM/TurboQAM extension, confirmed on the
  dies whose `AdapterCaps.vht_2g4_ok` is set, peer-decode required and the
  256-QAM MCS8/9 points still unmeasured (`docs/vht-on-2g4.md`). `/ER`, `/ER106`, `/DCM` are HE-only (Kestrel): the HE ER SU
  extended-range PPDU + dual-carrier modulation (`docs/he-extended-range.md`).
  The library itself is radiotap-driven — a frame carrying its own rate
  radiotap overrides the mode per-packet (ER SU = radiotap-HE FORMAT=EXT_SU).
  Programmatic: `SetTxMode` / `ClearTxMode`.
- `DEVOURER_SKIP_RESET=1` — skip `libusb_reset_device` before claim (only
  helps when firmware state is intact). Kestrel adapters skip the reset
  unconditionally — a USB reset on running firmware can land the chip in the
  dead ZeroCD DISK id (`src/kestrel/CLAUDE.md`).
- `DEVOURER_TX_GAP_US=N` — txdemo inter-frame gap (default 2000, ~500 fps;
  `0` = max duty for heating experiments).
- `DEVOURER_TX_FRAMES=N` — txdemo finite frame bound (default `0` =
  unbounded). Exits through the ordinary `Stop()` path once N frames have been
  submitted, so first-light and regression captures get a clean teardown
  instead of a killed timed flood.
- `DEVOURER_USB_DEBUG=1` — libusb DEBUG log level (~7 MB / 15 s, has filled
  `/tmp` mid-capture; adds 0.5–0.8 s to init).
- `DEVOURER_THERMAL_POLL_MS=N` — emit `thermal` events from the RF 0x42 meter,
  on every generation (the poller rides `IRtlDevice::GetThermalStatus`).
  `raw` is 0..63 thermal units (~1.5–2 °C each, **not** absolute °C); `delta`
  = raw − EFUSE baseline. **Telemetry only**: the poller emits and warns
  (`DEVOURER_THERMAL_WARN_DELTA`, default 15) and never stops RX; no HAL gates
  its send path on the meter. A rising delta is *suggestive*, not a validated
  degradation predictor — `docs/warm-tx-degradation.md` has delivery scattered
  63–83% with no relation to the meter, and inside one uninterrupted session
  the meter stays pinned while delivery drifts.
- `DEVOURER_LINKHEALTH=1` (rxdemo, needs `DEVOURER_RX_ENERGY_MS=N`) — classify
  the RX sensor tuple via `src/LinkHealth.h`. **EVM, not SNR, is the
  saturation tell**: strong RSSI + poor EVM means back power OFF, which is the
  opposite of the weak-link response (`docs/bench-testing-near-field.md`).
- `DEVOURER_LA_CAPTURE=<trig>/<rate>M/dma0/port:0x880` (rxdemo) — one-shot
  LA-mode IQ capture to a `DVLA` file, offline per-tone H(k) via
  `tools/la_csi.py`. Not on the 8812A/8821A (no LA block). Packing, per-chip
  windows, trigger semantics and wedge risks: `docs/la-capture.md`.

Behavioural traps the per-field docs can't carry:

- `DEVOURER_TX_WITH_RX=thread` (concurrent TX+RX on one claimed handle:
  `InitWrite` once, then `StartRxLoop` on a thread) must be set **before**
  `InitWrite` on Jaguar3. This is the single-radio beamforming self-sounding
  station: pair with `DEVOURER_BF_ARM_SOUNDER` / `DEVOURER_TX_NDPA` /
  `DEVOURER_BF_DETECT_REPORT` (`docs/beamforming-self-sounding.md`).
  Non-`thread` values select a `fork()` RX child that only works on Termux;
  on regular Linux the forked bring-ups race and die.
- `DEVOURER_RX_PATHS` (Jaguar1 RX-chain mask) routes through `SetRxPathMask`
  and is **sticky** across `SetMonitorChannel`. Toggle spec
  `0xAA:0xBB[:0xCC]@<ms>` cycles masks on a timer for mobility/MRC
  measurements (`docs/measuring-spatial-diversity.md`,
  `tests/mrc_mobility.py`). `DEVOURER_RX_ALLPATHS=1` emits per-chain
  RSSI/SNR/EVM as a separate `rx.path` event on the four Jaguar/Kestrel
  generations (Kestrel parses its halbb physts path pages; C/D nonzero only on
  the 8814AU — the other dies are ≤2 RX chains; the 1T1R RTL8733B does not
  emit it). Windowed per-antenna means ride `GetActiveRxPaths()` / the
  `adapter.rxpaths` event (rssi/snr/evm per chain).
- `DEVOURER_RX_KEEP_CORRUPTED=1` is the entry point for the fused-FEC salvage
  layer (`docs/fused-fec.md`), and stays opt-in for a reason: a body with a
  corrupt tail is the worst-case input for an IP-stack consumer that didn't
  ask for it.
- `DEVOURER_RX_CSI_MASK` / `DEVOURER_RX_NBI` apply at RX-loop start and
  **revert on a channel switch**. Measured inert against a *jammed* slice —
  that loss is pre-FCS sync/AGC, upstream of the equalizer. They target
  in-band spurs on otherwise decodable frames
  (`docs/pseudo-preamble-puncturing.md`).
- `DEVOURER_DIS_CCA=1` (runtime `SetCcaMode` — the interface method is pure
  virtual so no backend can silently no-op it)
  disables the MAC carrier-sense gate — **both** primary CCA (`0x520[14]`)
  and EDCCA (`[15]`). The default is carrier-sense + EDCCA **enabled** on
  Jaguar1/2/3; on Jaguar1 the enable is real work — its BB table parks the
  EDCCA thresholds (`0x8a4`) at never-trigger, so bring-up programs the
  vendor adaptivity operating point (IGI-coupled; the phydm watchdog
  re-tracks it when running). The primary-CCA bit is the one that matters:
  monitor injection is not CCA-free, it defers ~40–60% to a co-channel
  802.11 transmitter, and clearing `[14]` recovers ~1.5–2.2× (on-air
  8822EU/8812CU, `tests/dis_cca_tx_onair.sh`); the energy bit `[15]` alone
  is null against a decodable preamble. **On by default on the streamtx FPV
  downlink** (the link owns the channel — CSMA backoff only stutters it);
  `DEVOURER_DIS_CCA=0` forces standard carrier-sense back. On Kestrel the
  8852C runs the same enabled default (measured: full-rate TX, 2.4x flood
  deferral); the 8852B TX bring-up still clears the gates and WARNS pending
  its measurement arm (`tests/kestrel_cca_default_check.sh`). On the RTL8733B
  the disable is **not ported** — the HALMAC 87xx carrier-sense gate has not
  been located and measured there, so `SetCcaMode(true)` throws (loudly, but
  without tearing the session down) while `SetCcaMode(false)`, the state its
  MAC bring-up already leaves programmed, succeeds as a no-op; setting the
  config knob warns once at bring-up (RX-only sessions included) and airs
  with carrier-sense. Does NOT apply
  the vendor BB CCA-off writes (they deafen the RX). RX-decode side is a
  separate null (`tests/dis_cca_onair.sh`).

**Runtime TX power** — the adaptive-link power lever, three knobs on the four
Jaguar/Kestrel generations: `SetTxPowerOffsetQdb` (relative,
shape-preserving), `SetTxPowerIndexOverride` (flat absolute),
`SetTxPowerRateDiffs` (replace the
calibrated per-rate shape). The RTL8733B ports **only the first**, and on a
different mechanism: its closed-loop TSSI target table, not a TXAGC index, so
its caps report the dBm model (`index_max = 0`) over the int8 delta field's
`[-128, +127] qdB`, centred on a safe 16 dBm first-light target. Neither end is
re-clamped at something softer; where the chip stops responding — about
-96 qdB down, about +32 up, where the PA compresses and only EVM shows it — is
measured and documented rather than enforced. The flat index is refused there
because it was measured unable to carry HT at all. The contract — how they
compose, the MCS7-anchor
semantics, family step sizes, the write-only-family `hw_readback=false`
shadow, and Kestrel's software send-time fold — is documented at the
declarations in `src/TxPower.h`; the per-chip mechanics are in each
`src/<gen>/CLAUDE.md`. All three apply live and stick across
`SetMonitorChannel` and `FastRetune`. None is regulatory-clamped: the
operator owns compliance.

`txpower` (`examples/txpower/`) is the reference consumer —
`--rate-diffs cck,legacy,m0..m7|clear`, `--offset-start`/`--offset-stop`,
`--flat`; `txdemo` maps the third knob as `DEVOURER_TX_RATE_DIFFS`. What each
validation script actually proves matters on a write-only family:
`tests/txpwr_offset_regcheck.sh` and `tests/txpwr_rate_diffs_regcheck.sh`
check **register/shadow state management**, not radiated power —
`tests/txpwr_rate_diffs_onair.sh` is the one that measures the antenna, per
rate.

**Per-packet TX power** — a radiotap `DBM_TX_POWER` dB-delta per frame, zero
USB cost once armed. The three hardware shapes (Jaguar2 + 8814A descriptor
LUT, Jaguar3 programmable BB banks, Kestrel fixed-dBm BB rewrite) are
documented with the `per_pkt_txpwr_*` caps in `src/AdapterCaps.h`, and in
full in each `src/<gen>/CLAUDE.md`. The 8812AU/8821AU have no descriptor field
at all — their compensating fast lever is `FastSetTxPowerOffsetQdb`
(BB-swing). Sweep harnesses: `tests/txpkt_pwr_ofset_onair.sh` (TX_PID/TX_VID
select the DUT; Kestrel DUTs need `TX_PWR=14`-style dBm bases),
`tests/txpkt_fastswing_onair.sh`, `tests/txpkt_hop_persist.sh`.

Per-packet unequal error protection: `svctx` classifies stdin HEVC NALs by
temporal layer and injects each at its ladder's rate
(`DEVOURER_SVC_LADDER="CRIT=<spec>;T0=<spec>;..."`); the application-FEC half
(RS outer code + corrupt-frame salvage) lives in `tools/precoder/`
(`docs/fused-fec.md`).

## Frequency hopping

`IRtlDevice::FastRetune(channel)` — lean intra-band, same-bandwidth retune on
all five generations (RF channel switch only, write-only from a
compose cache); falls back to full `SetMonitorChannel` on a band change.
FHSS-grade on the Jaguar/Kestrel dies: ~0.5–2.5 ms per hop depending on chip.
On the RTL8733B the hop keeps TSSI tracking live (per-channel rate-offset
dwords rewritten in place) and costs ~55 ms call / ~10 ms p50 radio-live on
the USB-HS validation unit — vs its ~330-440 ms full path — with a measured
1-in-20 40 ms radio-live tail; one unit, no SDR (`src/rtl8733b/CLAUDE.md`). On the Jaguar2 dies (8822B, 8821C) and Jaguar3
(8822C, 8822E), `DEVOURER_FASTRETUNE_FW=1` hands the hop to the chip firmware
instead (H2C 0x1D, fire-and-confirm-later): ~1.4 ms dead air on the 8822B (a
tie on-air on the 8822C/8822E, but ~3× cheaper host-side), and `=2` extends it
to **cross-band** hops (~2–2.6 ms vs the ~90 ms full path) — protocol + bench:
`docs/experiments/kernel-channel-switch-offload.md`. `send_packet` honours a radiotap `CHANNEL` field, so
hopping is per-packet and radiotap-driven like rate. Demos hop via
`DEVOURER_HOP_CHANNELS` (SweepSpec grammar: `1,6,11`, `36-48/4`, `5170-5250/5`
MHz) + `DEVOURER_HOP_DWELL_FRAMES` / `_ROUNDS` / `_FAST` / `_RADIOTAP` /
`_BW` / `_OFFSET`; `DEVOURER_HOP_PROF=1` emits per-stage `debug.hop_prof`
timing. Validation:
`tests/run_hop_validation.sh`, `tests/hop_parity_check.sh` (register parity
full-vs-fast). Implementation + per-generation ports:
`docs/frequency-hopping.md`.

Above that primitive sits the **keyed FHSS / adaptive hopset** subsystem
(`src/hopset/`, header-only and pure): keyed slot schedules and lockstep RX
(`DEVOURER_HOP_SLOT_MS` + `DEVOURER_HOP_SEED`), a receiver-driven channel
exclusion policy, and TX-side quiet-window sensing with endpoint fusion. Knob
reference, policy thresholds, measured sensing constants and the on-air
harnesses: `src/hopset/CLAUDE.md`. Article + results: `docs/fhss.md`,
`docs/jammer-resilience.md`.

`IRtlDevice::FastSetBandwidth(bw)` is the bandwidth analogue — a lean
same-channel toggle between 20 MHz and 5/10 MHz narrowband (baseband re-clock
only; ~0.18 ms on the 8812AU vs ~90 ms for the full `SetMonitorChannel`);
falls back to the full path for a 40/80 MHz endpoint. Validation:
`tests/fast_bw_parity.sh`. See `docs/narrowband.md`.

RX counterpart: `DEVOURER_RX_SWEEP` dwells FastRetune-cheap bins emitting
per-bin energy + frame stats; `tests/sounding_sweep.sh` + `tests/sounding_map.py`
recover a coarse per-bin H(f) — down to 5 MHz bins on Jaguar3
(`docs/rx-spectrum-sensing.md`).

## Adaptive channel migration

Slow, evidence-driven whole-link channel moves — the deliberate complement to
per-slot FHSS. Pure caller-side logic under `src/chanmig/`: a passive scout on
a second adapter (`chanscout`), a two-leg scoring engine, an authenticated
ground-proposes / drone-commits protocol (`examples/chanmig --role
ground|drone`), and a deterministic automation gate
(`DEVOURER_MIG_MODE=off|advisory|manual|automatic`, default `advisory`).
Control frames are their own 802.11 frames — video PSDUs are never touched,
and there is no regulatory DB: the caller owns compliance. Layer detail,
validation matrix and the near-field bench note: `src/chanmig/CLAUDE.md`.

## Hardware time, beacons, AP mode

`ReadTsf()` reads the 64-bit MAC TSF and every received frame carries the
MAC-latched `tsfl` RX timestamp — on the four Jaguar/Kestrel generations,
µs-grade (`docs/time-distribution.md`, measured vs NTP/PTP:
`docs/timing-accuracy.md`).
`StartBeacon` loads a beacon into the MAC's reserved page and the chip
auto-transmits at each TBTT with the live TSF stamped at the TX instant — the
sub-µs downlink. The chip beacons **autonomously**, so `StopBeacon()` is what
stops it mid-session; a session that ends via `Stop()` or destruction powers the
chip down and takes the beacon with it (Jaguar1/Jaguar3 — Jaguar2 has no
teardown power-down yet, so there it keeps airing until the adapter is
re-enumerated).
`UpdateBeaconPayload` swaps the airing content in place (frame-atomic on air,
TBTT-quantized latency); `PinBeaconTbtt` steers the TBTT to an absolute TSF
instant without corrupting the clock (Jaguar1: offset 0 only — its TBTT is
hardware-locked to the TSF grid); `AdjustBeaconTimingFine` is the µs-fine
manual lever. Demos: `timesync` (`DEVOURER_TSYNC_ROLE=master|slave|ue` +
`DEVOURER_TSYNC_*` knobs, incl. the PCIe master + I226-PTP discipline loop)
and `tdma` (TSF-slotted narrowband↔wide bursts on one channel). The beacon is
also the seed of AP mode — a real Linux station associates, open or WPA2-PSK
(`docs/ap-mode.md`); the multi-cell architecture it enables is
`docs/multi-ap-cellular.md`, and the measured scheduler contracts (submit→air
guard time, dynamic beacon grants, ACK/TxReport, per-UE RX attribution) are
`docs/scheduled-mac.md`. None of this section is ported on the RTL8733B —
`ReadTsf` returns 0 and `StartBeacon` returns false there.

## Aggregation, hardware ACK, TX reports

`SetAmpduMode` enables 802.11 A-MPDU on injected frames: ~+30% *goodput* at
the PHY ceiling by amortizing per-frame overhead — an occupancy metric can't
show it, count delivered payload. That number is high-MCS broadcast; the
unicast-ARQ shape measured **−8%** delivered at MCS3, and per-frame CCX
accounting does not survive AGG_EN (docs/aggregation.md) — under A-MPDU the
receipts tier is the delivery truth. `SetAckResponder(mac)` arms the hardware
ACK/BlockAck responder; with a unicast TA on the soliciting frame this closes
a hardware-ARQ loop (autonomous MAC retransmission until ACK). The ACK's
horizon is chip-FIFO **admission** (bench: 8812EU responder,
`tests/arq_e2e_delivery.sh` per-frame ledgers): receiver-side congestion
upstream of admission declines the ACK, so the loop sees and retries it — but
a host stage that drops on a full queue while keeping URBs armed turns the
same loss into ACKed-but-undelivered that the TX peer logs as delivered and
never retries. The spsc-fat ring's `DEVOURER_RX_POOL_EXHAUST` policy is that
choice (`src/DeviceConfig.h`): `backpressure` (default) parks exhausted URBs
so the chip declines further ACKs and the loss stays ARQ-visible
(`pool_stalls` in `rx.ring`; bench: 0 ACKed-but-undelivered under stalls that
lose 5.5k+ frames on the `drop` policy, which remains opt-in and counted as
`pool_dropped`). Per-frame TX
outcomes surface as `tx.report` events (CCX via C2H) — the TX-side link
sensor; C2H rides the RX path, so J1/J2 TX-only sessions see none (run
`DEVOURER_TX_WITH_RX=thread`; J3's coex thread drains C2H regardless).
`GetRxQuality()` is the device-wide windowed RX sensor;
`cell::UeRxAttribution` is its per-transmitter (per-UE) counterpart. Docs:
`docs/aggregation.md`, measured per-generation matrix `docs/scheduled-mac.md`.

## Architecture

**The caller owns libusb.** `WiFiDriver::CreateRtlDevice` is intentionally
thin — `libusb_init`, device open, kernel-driver detach, and
`libusb_claim_interface(handle, 0)` must happen **before** handing the handle
to the factory. `examples/rx/main.cpp` is the canonical boilerplate;
`devourer::claim_interface_then_reset` (src/UsbOpen.h) is the recommended
open path (advisory per-adapter lock before reset).

Owning libusb means owning the **teardown order**: destroy the `IRtlDevice`
first, then release the interface, close the handle, and only then
`libusb_exit`. The device is what quiesces TX (`IRtlDevice::Stop`, and the
destructor as a backstop: Jaguar1's async bulk-OUT URBs must be cancelled and
reaped while the context still exists), so tearing libusb down first is a
crash, not a leak — and only under enough TX load to keep URBs outstanding at
exit. `examples/common/DeviceSession.h` is the demos' RAII holder for exactly
that order; the transport logs a diagnostic naming this if it is destroyed
with TX still in flight.

**Chip identity is resolved at construction** from the `SYS_CFG2` chip-id +
USB PID. `CreateRtlDevice` returns an `IRtlDevice` (`Init` = bring-up + RX
loop; `InitWrite` = TX bring-up; `StartRxLoop` = blocking RX worker on an
already-up chip, enabling TX+RX on one handle; `send_packet`) and constructs
`RtlJaguarDevice` / `RtlJaguar2Device` / `RtlJaguar3Device` / `RtlKestrelDevice`
/ `Rtl8733bDevice` per backend. `Rtl8812aDevice` is a deprecated alias of
`RtlJaguarDevice`. Optional device methods are **virtual with not-ported
defaults**, not pure virtual — a backend that hasn't ported a feature inherits
`false`/`0`/a full-path fallback rather than a fake. Check the override list in
the backend's header before believing a cross-generation claim.

Generation-agnostic core in `src/` (always compiled; depends on no HAL):

- `WiFiDriver` — the factory (`CreateRtlDevice`).
- `DeviceConfig.h` — construction-time configuration struct; every component
  copies the sub-struct it consumes at construction.
- `RtlAdapter` — the bus-neutral register/frame accessor; a copyable value
  type shared by every component, forwarding to the `IRtlTransport` it was
  built with (`UsbTransport` = libusb vendor control + bulk; `PcieTransport` =
  BAR2 MMIO + DMA rings). `RtlUsbAdapter` is a deprecated alias.
- `Radiotap.c` — radiotap iterator. TX buffers passed to `send_packet` **must**
  begin with a radiotap header; rate/MCS/VHT/STBC/LDPC/SGI/bandwidth are read
  from it.
- `RateDefinitions.h`, `RxPacket.h`, `TxDescBits.h` — symbols all generations'
  parsers build on, kept neutral so no generation's header pulls in another's.
- `PhyTableLoader` — runtime walker for Realtek's phydm-format register tables
  (`check_positive` + opcode state machine, without pulling in phydm itself).
  Shared by Jaguar1 + Jaguar2; Jaguar3 has its own `PhyTableLoaderJaguar3`.
- `cell/` — caller-side per-cell helpers built on the device API
  (`UeRxAttribution`: per-transmitter windowed RX statistics keyed by 802.11
  TA); the device RX loops are untouched.

Per-backend HALs are self-contained under `src/jaguar1/`, `src/jaguar2/`,
`src/jaguar3/`, `src/kestrel/`, `src/rtl8733b/`; each subtree's `CLAUDE.md`
maps its files, strategy seams and chip-specific mechanisms.

`hal/` holds vendor headers and tables. The per-chip PHY/limit tables and most
firmware blobs are **generated** by `tools/extract_*.py` (check `tools/` for
which — the set has grown) — edit the generators, never the output files. The
newer extractors (`extract_8733b_*.py`) pin per-source and per-array SHA-256
hashes and carry a `--check` mode that reproduces the checked-in output
byte-for-byte; prefer that shape when adding one.

## Hardware gotchas

- **ZeroCD trap**: some Realtek dongles enumerate first as USB mass-storage
  (`0bda:1a2b`) exposing a Windows installer, then re-enumerate as the NIC. If
  `libusb_open_device_with_vid_pid` returns NULL, check `lsusb` — may need
  `usb_modeswitch`.
- **rmmod/sysfs-unbind actively de-inits the chip** (RF off, MAC DMA off).
  After detaching a kernel driver, expect a cold re-init; `DEVOURER_SKIP_RESET`
  only helps when firmware state is intact.
- **The chip retains state across soft re-init** — cold-bisect hardware
  problems with a VBUS power-cycle, not a re-run.
- **Qualify the ground station before believing a delivery number.** Delivery is
  a property of a *link*; a receiver whose modulation cliff sits at the rate
  under test measures itself, not the transmitter, and swings between "fine" and
  "zero" on a few dB of ambient while the robust rates stay pinned — which looks
  exactly like a transmitter that degrades and recovers. Same TX, same channel,
  minutes apart: a TP-Link Archer T3U (8822BU, **internal** antennas) ran
  MCS3 97.8 / MCS4 98.3 / MCS5 79.1 / MCS6 49.0 / **MCS7 2.9**, while an
  RTL8814AU (two **external** antennas) was flat at ~80% across the same ladder.
  Both healthy — one just has less margin. `tests/ground_station_qualify.sh`
  sweeps the ladder and refuses the pairing (exit 1) when the test rate is off
  the flat part. Prefer external-antenna adapters as ground stations for
  high-MCS work, and cross-check with a second, independent receiver.
- **A single delivery probe is worth ±3 points, so small effects are not
  findings.** Ten identical back-to-back MCS7/20 probes on an 8812AU (nothing
  changed between them) gave sd 1.8 and a 5.7-point spread; the MCS1 control
  over the same run held 98.0 ± 0.2. Anything under ~4 points needs repetition
  before it means anything — several plausible-looking "decay curves" have
  turned out to sit inside that band. `tests/probe_repeatability.sh` measures
  the floor for a given pair; run it before believing a delivery difference.
- **Do not attribute a rate ceiling to chip temperature without varying
  temperature independently.** Delivery does correlate with the `thermal` meter
  within a probe sequence, and that correlation is a trap: a sweep varying
  *only* the power-off duration (identical reset every arm) found delivery
  scattered 63–83% with no relation to off-time or temperature, and inside a
  single uninterrupted session the meter stays pinned while delivery drifts.
  Read the `thermal` event (`DEVOURER_THERMAL_POLL_MS`) beside a rate-ceiling
  number by all means, but a power cycle changes chip state *and* temperature
  together, so it can never separate them (`docs/warm-tx-degradation.md`).
- **MediaTek Android hosts cap bulk-IN reads at 16 KB**: some MTK xhci/usbfs
  stacks (Dimensity 810, Helio G99, MT6765) never complete a larger bulk-IN
  transfer — `LIBUSB_ERROR_TIMEOUT` forever, zero RX with a green init
  (OpenIPC/PixelPilot#6). The 11ac RX rings therefore post 16 KB URBs paired
  with ≤16 KB device-side aggregation caps (`DeviceConfig::Rx::urb_bytes`,
  `DEVOURER_RX_URB_BYTES`); raise both together or MTK hosts go silent, and
  never let an aggregate exceed the URB size or the parse walk breaks.
  Kestrel is exempt (8852C RXAGG LEN_TH ~20 KB needs its 32 KB ring) and
  unvalidated on MTK hosts. The RTL8733B is the worked example of the failure:
  its vendor-default 20 KiB aggregate was observed being split by xHCI across
  16 KiB completions, tail and body landing separately. It now caps the device
  at 12 KiB and floors its URB at the same constant, tied together by a
  `static_assert` — raising one alone reintroduces the straddle.

## TX path

```cpp
auto logger = std::make_shared<Logger>();
WiFiDriver driver(logger);
auto dev = driver.CreateRtlDevice(handle);  // handle is already claimed
dev->InitWrite(SelectedChannel{ .Channel = 36, .ChannelOffset = 0,
                                .ChannelWidth = CHANNEL_WIDTH_20 });
dev->send_packet(buffer, len);  // buffer[0..] = radiotap header, then 802.11
```

The canonical test beacon (`examples/tx/main.cpp`) uses SA
`57:42:75:05:d6:00` — the same constant is hardcoded into
`examples/rx/main.cpp` as the `rx.txhit` event matcher and into
`tests/regress.py` (`CANONICAL_SA`). Change all three together if it ever
moves.

**TX transfer mode is deliberately per-generation**
(`docs/performance-tuning.md`): Jaguar1 submits asynchronously
(`RtlAdapter::send_packet` → `tx_async`, caller-thread completion reaping —
its USB2 round-trip is too long for a blocking send to saturate the link);
Jaguar2/Jaguar3 send synchronously (`bulk_send_sync_ep` → `tx_sync` — their
USB3 round-trip saturates on one blocking thread, and sync gives the HalMAC
bring-up a clean per-send NAK backoff); the RTL8733B is synchronous too, so
every submission has a bounded result and no buffer outlives the `send_packet`
call. Don't unify the modes — either direction regresses throughput or
bring-up safety.

**Nothing reads a register per frame on the send path.** Measured on one
RTL8733B unit during bring-up: a single thermal read (3 RF writes + a 15 µs
settle + an RF read, each several USB control transfers) cost 2.51 ms of a
2.71 ms per-frame budget on USB high speed — 93%, for a meter that tracks PA
bias and is not a validated degradation predictor. One bench, one part, and a
USB3 host would divide it; the shape of the result is the transferable part.
Sensor reads belong on the caller's own cadence via the runtime getters, never
inside `send_packet`.
