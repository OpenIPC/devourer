# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with
code in this repository. Per-generation deep facts (registers, descriptors,
per-chip mechanisms) live in `src/{jaguar1,jaguar2,jaguar3,kestrel}/CLAUDE.md`,
auto-loaded when working in that subtree — add new per-generation facts there,
cross-cutting facts here.

## What this is

Userspace re-implementation of Realtek's USB Wi-Fi drivers (11ac RTL88xx and
11ax RTL8852 families) — speaks to the chip directly via libusb instead of a
kernel module. Static library `devourer` (CMake target) + example executables
under `examples/` (`rxdemo` and `txdemo` are the canonical RX/TX demos). Used
by the OpenIPC project for long-range video links.

Four chip generations, each behind its own self-contained HAL, dispatched at
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
  (legacy/HT/VHT/HE + HE ER SU/DCM extended range —
  `docs/he-extended-range.md`), 5/10/20/40/80 MHz on both dies + 160 MHz on
  the 8852C (the B die has no 160 MHz); 6 GHz TX tops out at 80 MHz (the
  6G+160 TX-enable path is un-ported). TX power is a fixed BB dBm
  (`DEVOURER_TX_PWR`, whole dBm on this family). The capstone is 11ax
  trigger-based UL + TWT (issue #236); the 8852A-family (RTL8832AU) is
  deliberately excluded. 8852C behavioural quirks: `docs/8852c-quirks.md`.

Naming traps: **RTL8821AU is Jaguar1** (not Jaguar2, despite the Jaguar2
RTL8821C's similar name); RTL8822**B**U (Jaguar2) ≠ RTL8822**C**U (Jaguar3);
the TP-Link TX50UH is RTL8832**C**U (8852C-family) despite lab lore calling it
8832AU, while the TX20U **Nano** is RTL8852BU. Full chip / bench-throughput
table: README **Supported hardware**.

**PCIe** (`DEVOURER_PCIE=ON`, Linux-only, default OFF): the RTL8821CE — the
PCIe sibling of the 8821CU — rides the same Jaguar2 HAL through a vfio-pci
transport (`src/PcieTransport.{h,cpp}`): registers are BAR2 MMIO (the same
0x0000..0xFFFF space the USB vendor-control path addresses), TX/RX are the
88xx buffer-descriptor DMA rings (rtw88 pci.{c,h} layout; TX = the data/MGMT
BD rings behind the unchanged `send_packet`), RX completion polled by default
(MSI+eventfd wakeups when available). USB and PCIe are independent transports
behind `devourer::IRtlTransport` (`src/RtlTransport.h`); the bus-neutral
`RtlAdapter` value type the HALs hold forwards to whichever it was built
with. The few genuinely bus-specific bring-up steps gate on `is_usb()` (PCIe
power-seq rows, PQ map, no USB RX-agg, no DLFW 512-pad) or ride `hci_setup()`
(pre-power TRX ring programming, no-op on USB). Factory:
`WiFiDriver::CreateRtlDevicePcie(PcieTransport::Open(bdf, logger))` — the
caller owns vfio like it owns libusb. Demos: `DEVOURER_PCIE_BDF=0000:01:00.0`
on rxdemo and txdemo; `pcieprobe <bdf> [id|power|fw]` validates the layers
bottom-up. Bind/restore: `tests/pcie_vfio_bind.sh` (driver_override, not
new_id — the in-tree rtw88 auto-probe race). Validation: `sudo python3
tests/pcie_rx_smoke.py` against a vfio-bound 8821CE (rig specifics: local
`INVENTORY.md`) — ambient beacons CRC-clean on ch 6 + 36.

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
`DEVOURER_JAGUAR3_8822C`, `DEVOURER_JAGUAR3_8822E`. `DEVOURER_PCIE` (default
OFF, Linux-only, requires JAGUAR2_8821C) adds the vfio-pci transport +
`pcieprobe`; OFF builds are byte-identical to before it existed. Turning groups off drops
their firmware blobs + PHY tables (an 8812AU-only `rxdemo` is ~1.0 MB vs
~2.6 MB). Configure fails on no-chip-selected or 8814-without-JAGUAR1. Each
group exports a PUBLIC `DEVOURER_HAVE_*` define; sites referencing a dropped
group sit behind `#if defined(DEVOURER_HAVE_*)`, and the factory returns
`nullptr` (logs) for a chip whose support isn't built.

CI (`.github/workflows/cmake-multi-platform.yml`): GCC/Clang/MSVC ×
Ubuntu/macOS/Windows matrix, a `build-mingw` job, a `build-configs` matrix over
each per-chip subset, and `reject-bad-configs` for the invalid option combos.
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
returns a static aggregate of chip identity (name / generation / variant /
transport / chip-id), TX/RX chain counts, the composed `GetTxCaps` +
`GetTxPowerCaps`, the supported channel-width set, per-band tunable +
characterized frequency spans, and feature flags — resolved at construction,
thread-safe, callable pre-`Init`; the demos emit it as the `adapter.caps`
JSONL event. The `ldpc_rx_*` flags are the bench-derived LDPC RX truth table
(deliberately NOT the vendor `HAL_DEF_RX_LDPC` interop-advertisement policy,
which reads all-false on Jaguar1 while the 8812A demonstrably decodes LDPC):
every supported chip decodes HT+VHT LDPC except the 8821A (VHT-LDPC RX
broken, HT fine) and the 8814A reports no per-frame flag. LDPC TX is
per-packet radiotap-driven on all generations (`TxCaps.ldpc_ok`);
bench-measured coding gain ≈ +3 dB at the 10%-delivery crossing, MCS7/20 MHz
(`tests/ldpc_waterfall.sh`) — prefer `/LDPC` on any link whose RX side can
decode it. `GetActiveRxPaths()` is the live companion: a best-effort
per-chain-RSSI estimate of which antennas actually carry signal (needs an RX
loop + traffic). The 5 GHz synthesizer tunes past the UNII channels (extended
range ~5080–6165 MHz, chan up to 253, `freq = 5000 + 5*chan`); out-of-band
channels tune but their TX power / per-channel constants are extrapolated
from the nearest characterized channel (one-shot `W` diagnostic). No
regulatory enforcement — the caller owns compliance.

**Env vars are the demos' interface**: `examples/common/env_config.{h,cpp}`
maps every library-level `DEVOURER_*` var onto `DeviceConfig`, so the test
scripts drive everything through env. For the per-var reference, read the
`env:` tags in `DeviceConfig.h`; demo-local vars (device selection, timing)
are parsed in each demo's own code. The ones needed daily:

- `DEVOURER_PID=0xNNNN` / `DEVOURER_VID=0xNNNN` — restrict the device-open
  loop (default VID `0x0bda`, all Realtek PIDs). `DEVOURER_USB_BUS=N` +
  `DEVOURER_USB_PORT=a.b.c` select by USB topology when two adapters share
  VID:PID **and** serial.
- `DEVOURER_CHANNEL=N` — monitor channel.
- `DEVOURER_TX_RATE=<rate>[/<bw>][/SGI][/LDPC][/STBC][/ER|/ER106][/DCM]` — TX
  mode for rate-less frames (`MCS7/40/SGI`, `VHT2SS_MCS3/80/LDPC`, `1M`...).
  Unset = 6M legacy. CCK rates are 2.4 GHz-only; `1M` buys ~9 dB link budget
  over `6M`. `/ER`, `/ER106`, `/DCM` are HE-only (Kestrel): the HE ER SU
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
- `DEVOURER_USB_DEBUG=1` — libusb DEBUG log level (~7 MB / 15 s, has filled
  `/tmp` mid-capture; adds 0.5–0.8 s to init).

Knob-specific facts that aren't obvious from the field docs:

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
  RSSI/SNR/EVM as a separate `rx.path` event (C/D nonzero only on the
  8814AU).
- `DEVOURER_RX_CSI_MASK` / `DEVOURER_RX_NBI` (RX per-tone equalizer mask /
  narrowband notch, `src/ToneMask.h`) apply at RX-loop start and revert on a
  channel switch. Measured: inert against a *jammed* slice (that loss is
  pre-FCS sync/AGC, upstream of the equalizer) — they target in-band spurs on
  decodable frames (`docs/pseudo-preamble-puncturing.md`).
- `DEVOURER_RX_KEEP_CORRUPTED=1` passes FCS/ICV-failed frames up with
  `crc_err`/`icv_err` set — the entry point for the fused-FEC salvage layer
  (`docs/fused-fec.md`). Opt-in: a body with a corrupt tail is the worst-case
  input for an IP-stack consumer that didn't ask for it.
- `DEVOURER_DIS_CCA=1` (Jaguar2/3, runtime `SetCcaMode`) disables the MAC
  carrier-sense gate — **both** primary CCA (`0x520[14]`) and EDCCA (`[15]`).
  The primary-CCA bit is the one that matters: monitor injection is not
  CCA-free, it defers ~40–60% to a co-channel 802.11 transmitter, and
  clearing `[14]` recovers ~1.5–2.2× (on-air 8822EU/8812CU,
  `tests/dis_cca_tx_onair.sh`); the energy bit `[15]` alone is null against a
  decodable preamble. **On by default on the streamtx FPV downlink** (the
  link owns the channel — CSMA backoff only stutters it);
  `DEVOURER_DIS_CCA=0` forces standard carrier-sense back. Does NOT apply the
  vendor BB CCA-off writes (they deafen the RX). RX-decode side is a separate
  null (`tests/dis_cca_onair.sh`).
- `DEVOURER_LA_CAPTURE=<trig>/<rate>M/dma0/port:0x880` (rxdemo) — one-shot
  LA-mode IQ capture into the TX packet buffer (`src/LaCapture.h`): raw
  complex baseband to a `DVLA` file, offline per-tone H(k) via
  `tools/la_csi.py`. 8814A/8822B/8821C/8822C/8822E (+ 8821CE PCIe); the
  8812A/8821A have no LA block. Sample packing, per-chip windows, trigger
  semantics, validation scripts and wedge risks: `docs/la-capture.md`.
- `DEVOURER_THERMAL_POLL_MS=N` emits `thermal` events from the RF 0x42 meter:
  `raw` is 0..63 thermal units (~1.5–2 °C each, **not** absolute °C), `delta`
  = raw − EFUSE baseline; a rising delta is the early TX-degradation warning.
- `DEVOURER_LINKHEALTH=1` (rxdemo, needs `DEVOURER_RX_ENERGY_MS=N` — the RX
  energy-window cadence in ms) classifies the RX sensor tuple into SATURATED
  / INTERFERENCE / WEAK / MARGINAL / HEALTHY / NO_SIGNAL
  (`src/LinkHealth.h`) — distinguishing near-field saturation (strong RSSI +
  poor **EVM** — back OFF power) from a weak link (add power). EVM, not SNR,
  is the saturation tell. `docs/bench-testing-near-field.md`.

**Runtime TX power** (all generations — the adaptive-link power lever, see
`src/TxPower.h`): `SetTxPowerOffsetQdb(qdb)` shifts power in quarter-dB
relative to the efuse per-rate table (shape preserved until rates saturate at
the rails; flags in `GetTxPowerState`); `SetTxPowerIndexOverride(idx)`
forces/clears a flat index. Both apply live and stick across
`SetMonitorChannel` (re-folded on the new channel) and `FastRetune` (never
rewrites TXAGC). `GetTxPowerCaps` reports the family step: 0.5 dB Jaguar1/2,
0.25 dB Jaguar3. Write-only TXAGC hardware (Jaguar2, the 8814A's packed port)
reports the software shadow (`hw_readback=false`). `txpower`
(examples/txpower/) is the reference consumer; register-level validation:
`tests/txpwr_offset_regcheck.sh`.

**Per-packet TX power** (a radiotap `DBM_TX_POWER` dB-delta per frame, zero
USB cost once armed; see the `per_pkt_txpwr_*` caps): Jaguar2 and the 8814A
use a 3-bit descriptor `TXPWR_OFSET` hardware LUT (session default
`SetTxPacketPowerStep`); Jaguar3 selects programmable BB offset banks via
`TXPWR_OFSET_TYPE` (inert until programmed); Kestrel rewrites its fixed-dBm
BB target between frames; the 8812AU/8821AU have no descriptor field — their
compensating fast lever is `FastSetTxPowerOffsetQdb` (BB-swing). Full
mechanisms: each `src/<gen>/CLAUDE.md`. Sweep harnesses:
`tests/txpkt_pwr_ofset_onair.sh` (TX_PID/TX_VID select the DUT; Kestrel DUTs
need `TX_PWR=14`-style dBm bases), `tests/txpkt_fastswing_onair.sh`,
`tests/txpkt_hop_persist.sh`.

Per-packet unequal error protection: `svctx` classifies stdin HEVC NALs by
temporal layer and injects each at its ladder's rate
(`DEVOURER_SVC_LADDER="CRIT=<spec>;T0=<spec>;..."`); the application-FEC half
(RS outer code + corrupt-frame salvage) lives in `tools/precoder/`
(`docs/fused-fec.md`).

## Frequency hopping

`IRtlDevice::FastRetune(channel)` — lean intra-band, same-bandwidth retune on
every generation (RF channel switch only, write-only from a compose cache);
falls back to full `SetMonitorChannel` on a band change. FHSS-grade: ~0.5–2.5 ms
per hop depending on chip. On the 8822B and 8822C/8822E,
`DEVOURER_FASTRETUNE_FW=1` hands the hop to the chip firmware instead (H2C
0x1D, fire-and-confirm-later): ~1.4 ms dead air on the 8822B (a tie on-air on
the 8822C, but ~3× cheaper host-side), and `=2` extends it to **cross-band**
hops (~2–2.6 ms vs the ~90 ms full path) — protocol + bench:
`docs/kernel-channel-switch-offload.md`. `send_packet` honours a radiotap `CHANNEL` field, so
hopping is per-packet and radiotap-driven like rate. Demos hop via
`DEVOURER_HOP_CHANNELS` (SweepSpec grammar: `1,6,11`, `36-48/4`, `5170-5250/5`
MHz) + `DEVOURER_HOP_DWELL_FRAMES` / `_ROUNDS` / `_FAST` / `_RADIOTAP` /
`_BW` / `_OFFSET`; `DEVOURER_HOP_PROF=1` emits per-stage `debug.hop_prof`
timing. Validation:
`tests/run_hop_validation.sh`, `tests/hop_parity_check.sh` (register parity
full-vs-fast). Implementation + per-generation ports:
`docs/frequency-hopping.md`.

**Keyed FHSS + lockstep RX** (`src/HopSchedule.h`): `DEVOURER_HOP_SLOT_MS`
selects monotonic wall-clock slots; `DEVOURER_HOP_SEED` (≤32 hex, a 128-bit
key) replaces the public round-robin with a stateless SipHash-2-4
Fisher-Yates permutation per round, so a receiver joins without RNG state. In
slot mode the TX demos (`txdemo` beacon; `streamtx` own frame every
`DEVOURER_HOP_SYNC_EVERY`, FEC PSDU untouched) emit a sync marker, and
`rxdemo` hops in lockstep when
`DEVOURER_HOP_CHANNELS`+`_SLOT_MS` are set (seed optional →
keyed/sequential), emitting `hop.rx` acquire/track/retune events. Jammer
resilience: `tests/run_jammer_resilience.sh` + `tests/sdr_follower_jammer.py`
(B210 follower, reactive vs predictive). Article + results: `docs/fhss.md`,
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
per-slot FHSS. Pure caller-side logic under `src/chanmig/` (namespace
`devourer::chanmig`, header-mostly, each ctest-covered; the library reads no
env, the demos map it). Five composable layers:

- **Scout** (`chanscout`): a second adapter passively surveys a candidate
  plan (`DEVOURER_SCOUT_PLAN`) while the primary RX stays on the video
  channel, emitting versioned `survey.dwell` records with a counter-hygiene
  discard barrier (the FA/CCA counters are delta-on-read). Measures only;
  retunes nothing but itself. Grid-legality
  validation, **no regulatory DB** — the caller owns compliance.
- **Scoring** (`ChannelScore`, `DEVOURER_SCOUT_ADVISE`): a pure two-leg
  recommendation engine — the primary receiver's *delivery* is authoritative
  on the active channel (scout energy there is confounded by wanted video),
  the scout's occupancy on candidates — emitting explainable
  `channel.recommend`/`hold`.
- **Protocol** (`examples/chanmig --role ground|drone`): an authenticated
  ground-proposes/drone-commits migration (SipHash-MAC'd wire codec
  `MigWire.h`, pure `MigProposer`/`MigResponder` state machines, random
  per-boot epochs). The drone arms
  activation only after the ground echoes its nonce, and the ground follows
  the drone's authoritative STATUS — every failure-matrix row converges
  without split-brain. Control frames are their own canonical-SA 802.11
  frames — video PSDUs are never touched.
- **Automation** (`MigGate`, `DEVOURER_MIG_MODE=off|advisory|manual|automatic`,
  default advisory): a pure, deterministic gate hedged with cooldown /
  residency / per-channel backoff / move-cap / probation and an operator kill
  switch.
- **Drone-side validation**: legality/caps checks are the free product
  default; the pre-commit probe (`DEVOURER_MIG_PROBE`) is opt-in research
  (real outage cost); probation is the gate's.

Validated headless (`chanmig_wire_kat`, `chanmig_proto_matrix` — a 14-row
failure matrix + drop-every-message sweep, `chan_score_policy`,
`chanmig_gate_policy`, `chanmig_clock_math`) and on-air
(`tests/chanmig_endurance.sh`, `tests/chanscout_stress.sh`,
`tests/chanmig_soak.sh`). Docs: `docs/adaptive-channel-migration.md`,
`docs/channel-migration-protocol.md`, `docs/channel-migration-validation.md`.
Near-field bench note: two adapters ~30 cm apart saturate the RX front end at
full power (`link.health` SATURATED), so migration tests reduce TX power
(`DEVOURER_TX_PWR=12`).

## Hardware time, beacons, AP mode

`ReadTsf()` reads the 64-bit MAC TSF and every received frame carries the
MAC-latched `tsfl` RX timestamp — on all generations, µs-grade
(`docs/time-distribution.md`, measured vs NTP/PTP: `docs/timing-accuracy.md`).
`StartBeacon` loads a beacon into the MAC's reserved page and the chip
auto-transmits at each TBTT with the live TSF stamped at the TX instant — the
sub-µs downlink. The chip beacons **autonomously**: a session that ends
without a power-cycle must call `StopBeacon()` or the beacon keeps airing.
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
`docs/scheduled-mac.md`.

## Aggregation, hardware ACK, TX reports

`SetAmpduMode` enables 802.11 A-MPDU on injected frames: ~+30% *goodput* at
the PHY ceiling by amortizing per-frame overhead — an occupancy metric can't
show it, count delivered payload. `SetAckResponder(mac)` arms the hardware
ACK/BlockAck responder; with a unicast TA on the soliciting frame this closes
a hardware-ARQ loop (autonomous MAC retransmission until ACK). Per-frame TX
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

**Chip identity is resolved at construction** from the `SYS_CFG2` chip-id +
USB PID. `CreateRtlDevice` returns an `IRtlDevice` (`Init` = bring-up + RX
loop; `InitWrite` = TX bring-up; `StartRxLoop` = blocking RX worker on an
already-up chip, enabling TX+RX on one handle; `send_packet`) and constructs
`RtlJaguarDevice` / `RtlJaguar2Device` / `RtlJaguar3Device` per generation.
`Rtl8812aDevice` is a deprecated alias of `RtlJaguarDevice`.

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

Per-generation HALs are self-contained under `src/jaguar1/`, `src/jaguar2/`,
`src/jaguar3/`, `src/kestrel/`; each subtree's `CLAUDE.md` maps its files,
strategy seams and chip-specific mechanisms.

`hal/` holds vendor headers and tables. The per-chip PHY/limit tables and the
8821C firmware blob are **generated** by `tools/extract_*.py` — edit the
generators, never the output files.

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
- **MediaTek Android hosts cap bulk-IN reads at 16 KB**: some MTK xhci/usbfs
  stacks (Dimensity 810, Helio G99, MT6765) never complete a larger bulk-IN
  transfer — `LIBUSB_ERROR_TIMEOUT` forever, zero RX with a green init
  (OpenIPC/PixelPilot#6). The 11ac RX rings therefore post 16 KB URBs paired
  with ≤16 KB device-side aggregation caps (`DeviceConfig::Rx::urb_bytes`,
  `DEVOURER_RX_URB_BYTES`); raise both together or MTK hosts go silent, and
  never let an aggregate exceed the URB size or the parse walk breaks.
  Kestrel is exempt (8852C RXAGG LEN_TH ~20 KB needs its 32 KB ring) and
  unvalidated on MTK hosts.

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
bring-up a clean per-send NAK backoff). Don't unify the two onto one mode —
either direction regresses throughput or bring-up safety.
