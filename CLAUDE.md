# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Userspace re-implementation of Realtek's RTL88xxAU Wi-Fi driver — speaks to the
chip directly via libusb instead of a kernel module. Static library `devourer`
(CMake target) + example executables under `examples/` (`rxdemo` and `txdemo`
are the canonical RX/TX demos). Used by the OpenIPC project for long-range
video links.

Three chip generations, each behind its own self-contained HAL, dispatched at
construction from the `SYS_CFG2` chip-id (see **Architecture**):

- **Jaguar1** (`src/jaguar1/`): RTL8812AU (2T2R reference), RTL8811AU (1T1R cut,
  rides the 8812 path), RTL8814AU (4T4R RF / 3-SS baseband — host-pushed TX
  requires the on-chip 3081 MCU booted during FW download; a failed FW-boot
  poll means dead TX while RX still works), RTL8821AU (1T1R + BT).
- **Jaguar2** (`src/jaguar2/`): RTL8822BU / RTL8812BU (chip-id `0x0a`) and
  RTL8811CU / RTL8821CU (chip 8821C, chip-id `0x09`). A hybrid: HalMAC FW
  download / MAC init / power sequencing like Jaguar3, phydm `check_positive`
  register tables like Jaguar1 (shared `PhyTableLoader`). RX + TX on 2.4/5 GHz
  at 20/40/80 MHz, per-rate bandwidth-aware efuse TX power clamped to generated
  `txpwr_lmt` tables.
- **Jaguar3** (`src/jaguar3/`): rtl8822c (RTL8812CU/8822CU, chip-id `0x13`) and
  rtl8822e (RTL8812EU/8822EU, chip-id `0x17`). Adds **5/10 MHz narrowband**
  the Jaguar1 silicon lacks, 80 MHz (incl. a 40-in-80 frame via TX-descriptor
  DATA_SC), and halrf calibration (DACK/IQK/TXGAPK/thermal tracking).
  Sustained 5 GHz TX needs the **coex runtime thread**
  (`RtlJaguar3Device::coex_runtime_loop`, started in `InitWrite`) — without its
  ~2 s WiFi-only coex re-apply + FW heartbeats, the combo chip's coex firmware
  silences the antenna.

Naming traps: **RTL8821AU is Jaguar1** (not Jaguar2, despite the Jaguar2
RTL8821C's similar name); RTL8822**B**U (Jaguar2) ≠ RTL8822**C**U (Jaguar3).
The 8822BE (PCIe) and the Kestrel 11ax families are out of scope. Full chip /
bench-throughput table: README **Supported hardware**.

## Build

```sh
cmake -S . -B build
cmake --build build -j
```

libusb-1.0 via `pkg-config` (Linux/macOS) or vcpkg (`VCPKG_ROOT`, Windows).

Per-chip options, all default ON: `DEVOURER_JAGUAR1`, `DEVOURER_8814` (requires
JAGUAR1), `DEVOURER_JAGUAR2_8822B`, `DEVOURER_JAGUAR2_8821C`,
`DEVOURER_JAGUAR3_8822C`, `DEVOURER_JAGUAR3_8822E`. Turning groups off drops
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
rows are not authoritative for LDPC/STBC — that driver strips those bits),
`--sniffer-iface IFACE` (3rd-adapter capture, intended for AR9271).
`--keep-logs` puts per-cell logs at `/tmp/devourer-regress-last/`. Full
semantics: `tests/README.md`.

Startup-time benchmarking: `tests/bench_init.py` (per-stage `init-timing:`
lines from `src/InitTimer.h`; methodology + numbers in `docs/startup-time.md`).
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
uhubctl — never on xhci root ports on this rig).

## Configuration

**The library reads no environment.** Construction-time knobs live in
`devourer::DeviceConfig` (`src/DeviceConfig.h` — rx / tx / bf / tuning / debug /
usb sections, every field doc-tagged with its env-var spelling and value
grammar), passed as `CreateRtlDevice`'s defaulted fourth argument. Mid-session
knobs are runtime setters on `IRtlDevice` (`SetTxMode`, `SetTxPowerOffsetQdb`,
`SetTxPowerIndexOverride`, `SetRxPathMask`, `SetCcaMode`, `FastRetune`, ...).

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
- `DEVOURER_TX_RATE=<rate>[/<bw>][/SGI][/LDPC][/STBC]` — TX mode for rate-less
  frames (`MCS7/40/SGI`, `VHT2SS_MCS3/80/LDPC`, `1M`...). Unset = 6M legacy.
  CCK rates are 2.4 GHz-only; `1M` buys ~9 dB link budget over `6M`. The
  library itself is radiotap-driven — a frame carrying its own rate radiotap
  overrides the mode per-packet. Programmatic: `SetTxMode` / `ClearTxMode`.
- `DEVOURER_SKIP_RESET=1` — skip `libusb_reset_device` before claim (only
  helps when firmware state is intact).
- `DEVOURER_TX_GAP_US=N` — txdemo inter-frame gap (default 2000, ~500 fps;
  `0` = max duty for heating experiments).
- `DEVOURER_USB_DEBUG=1` — libusb DEBUG log level (~7 MB / 15 s, has filled
  `/tmp` mid-capture; adds 0.5–0.8 s to init).

Knob-specific facts that aren't obvious from the field docs:

- `DEVOURER_TX_WITH_RX=thread` (concurrent TX+RX on one claimed handle:
  `InitWrite` once, then `StartRxLoop` on a thread) must be set **before**
  `InitWrite` on Jaguar3 — the bring-up keeps the RX filters open; retrofitting
  RX later is unreliable. On the 8822E, TX+RX mode leaves the path-B OFDM TXAGC
  reference (0x41e8) at table default — any nonzero value there desenses the
  EU's RX to near-deaf (hardware-bisected, value-independent). This is the
  single-radio beamforming self-sounding station: pair with
  `DEVOURER_BF_ARM_SOUNDER` / `DEVOURER_TX_NDPA` / `DEVOURER_BF_DETECT_REPORT`
  (`docs/beamforming-self-sounding.md`). Non-`thread` values select a
  `fork()` RX child that only works on Termux; on regular Linux the forked
  bring-ups race and die.
- `DEVOURER_RX_PATHS` (Jaguar1 RX-chain mask, `0x808` byte 0) routes through
  `SetRxPathMask` and is **sticky** across `SetMonitorChannel` (IQK
  saves/restores `0x808`). Toggle spec `0xAA:0xBB[:0xCC]@<ms>` cycles masks on
  a timer for mobility/MRC measurements (`docs/measuring-spatial-diversity.md`,
  `tests/mrc_mobility.py`). `DEVOURER_RX_ALLPATHS=1` emits per-chain
  RSSI/SNR/EVM on a separate `<devourer-rxpath>` tag (C/D nonzero only on the
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
- `DEVOURER_THERMAL_POLL_MS=N` emits `<devourer-thermal>` lines from the RF
  0x42 meter: `raw` is 0..63 thermal units (~1.5–2 °C each, **not** absolute
  °C — hence bucketed status, not a fake temperature), `delta` = raw −
  EFUSE baseline. Jaguar1 has no hard thermal TX shutdown — a rising delta is
  the early warning. On the 8814 the baseline is read at the 8812 offset, so
  absolute delta may be off; the trend is valid.
- `DEVOURER_LINKHEALTH=1` (rxdemo, needs `DEVOURER_RX_ENERGY_MS=N` — the RX
  energy-window cadence in ms) classifies
  the RX sensor tuple into SATURATED / INTERFERENCE / WEAK / MARGINAL /
  HEALTHY / NO_SIGNAL (`src/LinkHealth.h`). Its purpose: distinguish
  near-field saturation (strong RSSI + poor **EVM** — back OFF power) from a
  weak link (add power). EVM, not SNR, is the saturation tell.
  `docs/bench-testing-near-field.md`.

**Runtime TX power** (all generations — the adaptive-link power lever, see
`src/TxPower.h`): `SetTxPowerOffsetQdb(qdb)` shifts power in quarter-dB
relative to the efuse per-rate table (shape preserved until rates saturate at
the rails; flags in `GetTxPowerState`); `SetTxPowerIndexOverride(idx)`
forces/clears a flat index. Both apply live and stick across
`SetMonitorChannel` (re-folded on the new channel) and `FastRetune` (never
rewrites TXAGC). `GetTxPowerCaps` reports the family step: 0.5 dB Jaguar1/2,
0.25 dB Jaguar3. The Jaguar2 TXAGC block and the 8814A's packed port are
write-only, so their `GetTxPowerState` reports the software shadow
(`hw_readback=false`). `txpower` (examples/txpower/) is the reference
consumer; register-level validation: `tests/txpwr_offset_regcheck.sh`.

**Per-packet TX power is Jaguar2-only**: the 8822B/8821C TX descriptor's
`TXPWR_OFSET` field is a hardware LUT (0/-3/-7/-11/+3/+6 dB) applied per-frame
at zero USB cost — `send_packet` quantizes a radiotap `DBM_TX_POWER` delta to
it (`jaguar2::txpkt_pwr_step_for_db`), and
`RtlJaguar2Device::SetTxPacketPowerStep` sets a session default. The 8812AU
has no such field (its per-packet power *is* per-rate selection via radiotap);
Jaguar3's `TXPWR_OFSET_TYPE` is measured inert.

Per-packet unequal error protection: `svctx` classifies stdin HEVC NALs by
temporal layer and injects each at its ladder's rate
(`DEVOURER_SVC_LADDER="CRIT=<spec>;T0=<spec>;..."`); the application-FEC half
(RS outer code + corrupt-frame salvage) lives in `tools/precoder/`
(`docs/fused-fec.md`).

## Frequency hopping

`IRtlDevice::FastRetune(channel)` — lean intra-band, same-bandwidth retune on
every generation (RF channel switch only, write-only from a compose cache);
falls back to full `SetMonitorChannel` on a band change. FHSS-grade: ~0.5–2.5 ms
per hop depending on chip. `send_packet` honours a radiotap `CHANNEL` field, so
hopping is per-packet and radiotap-driven like rate. Demos hop via
`DEVOURER_HOP_CHANNELS` (SweepSpec grammar: `1,6,11`, `36-48/4`, `5170-5250/5`
MHz) + `DEVOURER_HOP_DWELL_FRAMES` / `_ROUNDS` / `_FAST` / `_RADIOTAP` /
`_BW` / `_OFFSET`; `debug.hop_prof` (env `DEVOURER_HOP_PROF=1`) emits per-stage
timing. Validation: `tests/run_hop_validation.sh`, `tests/hop_parity_check.sh`
(register parity full-vs-fast). Implementation + per-generation ports:
`docs/frequency-hopping.md`.

RX counterpart: `DEVOURER_RX_SWEEP` dwells FastRetune-cheap bins emitting
per-bin energy + frame stats; `tests/sounding_sweep.sh` + `tests/sounding_map.py`
recover a coarse per-bin H(f) — down to 5 MHz bins on Jaguar3
(`docs/rx-spectrum-sensing.md`).

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
- `RtlUsbAdapter` — libusb wrapper (vendor control + bulk transfers); a
  copyable value type shared by every component.
- `Radiotap.c` — radiotap iterator. TX buffers passed to `send_packet` **must**
  begin with a radiotap header; rate/MCS/VHT/STBC/LDPC/SGI/bandwidth are read
  from it.
- `RateDefinitions.h`, `RxPacket.h`, `TxDescBits.h` — symbols all generations'
  parsers build on, kept neutral so no generation's header pulls in another's.
- `PhyTableLoader` — runtime walker for Realtek's phydm-format register tables
  (`check_positive` + opcode state machine, without pulling in phydm itself).
  Shared by Jaguar1 + Jaguar2; Jaguar3 has its own `PhyTableLoaderJaguar3`.

Per-generation HALs (each self-contained):

- `src/jaguar1/`: `RtlJaguarDevice` (orchestrator), `HalModule` (bring-up,
  power seq, table apply — most chip-family divergence, `_8812A`/`_8814A`/
  `_8821A` suffixed methods), `RadioManagementModule` (channel/BW/TX power, up
  to 4 RF paths), `EepromManager` (EFUSE + `cut_version`/`rfe_type` for
  `CheckPositive`), `FirmwareManager` (8814 = 3081-DDMA path, 8812/8821 =
  page-write), `FrameParser` (RX parsing + `SET_TX_DESC_*_8812` descriptors).
- `src/jaguar2/`: shared core (`RtlJaguar2Device`, `HalJaguar2`,
  `HalmacJaguar2Fw`/`MacInit`, `FrameParserJaguar2.h`) + per-variant strategy
  interfaces selected by `ChipVariant`: `Jaguar2PhyTables` →
  `Phy8822bTables`/`Phy8821cTables`, `Jaguar2Calibration` →
  `Halrf8822b`/`Halrf8821c`. Every strategy seam defaults to `C8822B`, so the
  8822B path stays byte-identical when the 8821C variant is compiled out.
- `src/jaguar3/`: `RtlJaguar3Device`, `HalJaguar3` (power seq, table apply,
  3-wire RF, bf_init, efuse incl. 8822e OTP burst-mode), `HalmacJaguar3Fw`/
  `MacInit`, `RadioManagementJaguar3` (channel/BW/per-path power, the
  `0x9b0`/`0x9b4` narrowband dividers, RF18 encoding), strategy interfaces
  `Jaguar3Calibration` → `Halrf8822c`/`Halrf8822e` and `Jaguar3PhyTables`.

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
`examples/rx/main.cpp` as the `<devourer-tx-hit>` matcher and into
`tests/regress.py` (`CANONICAL_SA`). Change all three together if it ever
moves.
