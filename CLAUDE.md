# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Userspace re-implementation of Realtek's RTL88xxAU Wi-Fi driver — speaks to the chip
directly via libusb instead of a kernel module. Static library `WiFiDriver` + two
demo binaries (`WiFiDriverDemo` for RX, `WiFiDriverTxDemo` for TX). Used by the
OpenIPC project for long-range video links.

Two chip generations are supported, dispatched at construction (see
**Architecture**):

**Jaguar (1st-gen 802.11ac).** The original family: **RTL8812AU** (2T2R,
reference), **RTL8811AU** (1T1R cut of 8812 silicon — rides the 8812 code
path with `RFType=RF_TYPE_1T1R` selected via `REG_SYS_CFG` bit 27),
**RTL8814AU** (4T4R RF / 3-SS baseband; host-pushed TX requires the
on-chip 3081 MCU, which devourer boots during firmware download —
a failed FW-boot poll means dead TX while RX still works), and
**RTL8821AU** (1T1R AC + BT combo). These share one HAL (`src/jaguar1/`,
e.g. `HalModule`, `RtlJaguarDevice`) with chip-family branches.

**Jaguar3.** A second, self-contained HAL under `src/jaguar3/` covering two PHY
generations that share one core: **rtl8822c** (RTL8812CU / RTL8822CU, `0bda:c812`)
and **rtl8822e** (RTL8812EU / RTL8822EU, `0bda:a81a`). Bring-up is ported from
Realtek's vendor source — power-on, HalMAC firmware download, MAC/BB/RF init,
halrf calibration (DACK, IQK, TXGAPK, thermal tracking), RX, and on-air TX at
20 MHz plus the **5/10 MHz narrowband** re-clock the Jaguar-1 silicon lacks.
Per-generation PHY/RF tables and calibration sit behind strategy interfaces
(`Jaguar3PhyTables`, `Jaguar3Calibration`); the flow and table-walker are shared.

Sustained 5 GHz TX needs the **coex runtime thread**
(`RtlJaguar3Device::coex_runtime_loop`, started in `InitWrite`): it drains the
firmware C2H reports off bulk-IN and every ~2 s re-applies the WiFi-only coex
decision (GNT_WL, antenna owner = WL), the FW heartbeats, and thermal TX-power
tracking. Without it the combo chip's coex firmware silences the antenna.

TX power: by default the chip transmits at its efuse-calibrated per-path,
per-channel level (the rtl8822e TXAGC references match the kernel driver's).
`DEVOURER_TX_PWR=0xNN` forces a flat TXAGC reference — a debug/SDR-visibility
knob, not for sustained use.

**RTL8821AU is Jaguar wave 1** (CHIP_8821 = 7, shares the enum with CHIP_8812),
not Jaguar2. The **Jaguar2 (8812BU, 8822BU/BE, …) and Kestrel (11ax) families are
out of scope** — same "AU"/"BU" branding, different baseband/HAL.

## Build

```sh
cmake -S . -B build
cmake --build build -j
```

Per-chip build options select which drivers are compiled in — all default ON,
so the command above builds every chip. Turn off the ones you don't need to
drop their firmware blobs + PHY tables (an 8812AU-only `WiFiDriverDemo` is
~1.0 MB vs ~2.6 MB): `DEVOURER_JAGUAR1` (8812/8811/8821), `DEVOURER_8814`
(requires JAGUAR1), `DEVOURER_JAGUAR3_8822C`, `DEVOURER_JAGUAR3_8822E`.
Configure fails on no-chip-selected or 8814-without-JAGUAR1. Each group exports
a PUBLIC `DEVOURER_HAVE_*` define; sites referencing a dropped group are behind
`#if defined(DEVOURER_HAVE_*)`, and the factory returns `nullptr` (logs) for a
chip whose support isn't built.

libusb-1.0 is required: `pkg-config` on Linux/macOS, vcpkg on Windows
(`VCPKG_ROOT` must be set so the toolchain file resolves). CI matrix builds
across GCC/Clang/MSVC on Ubuntu/macOS/Windows, plus a separate `build-mingw`
job (mingw-w64 via MSYS2, libusb from pkg-config) covering the Windows-GCC
toolchain the MSVC matrix cell doesn't; the `build-configs` matrix builds each
per-chip subset and `reject-bad-configs` asserts the invalid option combos fail
(`.github/workflows/cmake-multi-platform.yml`). `ctest` runs in every CI job;
the one registered test (`stream_stdin_binary`) round-trips the stream demos'
binary-stdin framing (`txdemo/stream_stdin.h`) headlessly, so a Windows
text-mode regression fails CI instead of only surfacing on a radio. Hardware
regression testing happens out-of-band via `tests/regress.py`.

## Regression testing

`tests/regress.py` runs a 2x2 TX/RX matrix (devourer vs. kernel driver) using
two USB Wi-Fi adapters plugged into the host. Run **after** building devourer.

```sh
# Local mode — kernel cells use whatever driver is bound on the host
sudo python3 tests/regress.py

# VM mode (recommended for RTL8814AU and other chips whose kernel driver
# doesn't build on bleeding-edge kernels) — kernel cells run inside a
# pinned-kernel (Ubuntu 22.04 / 5.15) libvirt VM with aircrack-ng/rtl8812au
# preloaded. Provision once with tests/setup_vm.sh, then:
sudo python3 tests/regress.py \
    --vm-name devourer-testrig --vm-ssh <user>@<VM-IP>
```

Default channel is `6` (2.4GHz). Pass `--channel 36` / `--channel 100`
to exercise 5GHz; do not assume a single-band matrix is comprehensive.
Matrix-interpretation caveat: with the 8814 as TX, the `kernel`-TX cells
read 0 at every channel — `aircrack-ng/88XXau` host-push *beacon*
injection (what `inject_beacon.py` does) doesn't emit on that driver,
even though its probe-request injection does. Judge 8814 TX by the
devourer-TX cells.

Three specialised modes layered on top of the default 4-cell matrix:

- `--full-matrix`: iterates every ordered (TX, RX) pair of plugged DUTs
  across all four driver-side combinations (N×(N−1)×4 cells; ~16 min for
  N=3 in VM mode). Use for cross-chipset interop regressions.
- `--encoding-matrix --tx-pid 0xNNNN --rx-pid 0xNNNN`: for one ordered
  pair, iterates (driver-mode × radiotap encoding combo) — 4 HT
  combos (BCC / LDPC / STBC=1 / LDPC+STBC) + 2 VHT combos
  (BCC / LDPC), 24 cells total.
- `--sniffer-iface IFACE`: when set, captures on a 3rd monitor-mode
  adapter alongside every cell and reports decoded radiotap encoding
  distribution. Intended for AR9271 — vanilla radiotap, no driver-side
  filtering. NB: `aircrack-ng/88XXau` on the kernel-TX side strips the
  radiotap LDPC + STBC bits before air, so the `k/k` and `k/d` rows of
  `--encoding-matrix` are NOT authoritative for LDPC/STBC asymmetries
  (MCS index + HT/VHT distinction do survive).

Init/startup-time benchmarking lives in `tests/bench_init.py`: per-DUT
cold-init timing of devourer RX (`exec → first RX frame`), devourer TX
(`exec → first bulk-OUT`), and — with `--vm-name`/`--vm-ssh` — the kernel
driver (`virsh attach → monitor up → first frame`). Per-stage numbers come
from the `init-timing: <scope>.<stage> = N ms` lines the library emits
(`src/InitTimer.h`); A/B variants isolate libusb log level, USB reset, and
the TX-power loop.

DUTs are routed between host and VM per cell via `virsh attach-device`.
Re-run with `--keep-logs` to inspect per-cell logs (and per-cell pcaps
when `--sniffer-iface` is active) at `/tmp/devourer-regress-last/`. See
`tests/README.md` for the full matrix semantics and prerequisites.

## Demo env vars

Both `WiFiDriverDemo` and `WiFiDriverTxDemo` honour:

- `DEVOURER_PID=0xNNNN` — restrict the open loop to a single PID (e.g.
  `0x8813` for RTL8814AU); without it, the demo iterates every Realtek PID.
- `DEVOURER_VID=0xNNNN` — override VID (default `0x0bda`); needed for
  OEM-rebadged dongles like the TP-Link Archer T2U Plus (`2357:0120`).
- `DEVOURER_CHANNEL=N` — override monitor channel.
- `DEVOURER_SKIP_RESET=1` — skip `libusb_reset_device` before claim; useful
  when picking up a chip whose firmware is already running.
- `DEVOURER_SKIP_TXPWR=1` — skip the per-rate TX-power loop during channel
  switch (runs by default on every chip; escape hatch for RX-only
  experiments).
- `DEVOURER_RX_KEEP_CORRUPTED=1` — pass frames that fail the 802.11 FCS (CRC32)
  or decryption-ICV check up to the host instead of dropping them at the WMAC
  filter (sets RCR `ACRC32|AICV`). They arrive with `crc_err`/`icv_err` set on
  the RX descriptor; `WiFiDriverDemo` surfaces them in its `<devourer-stream>`
  output. This is the entry point for the fused-FEC sub-block-salvage layer
  (see `docs/fused-fec.md`); opt-in, since a body with a corrupt tail is the
  worst-case input for an IP-stack consumer that didn't ask for it.
- `DEVOURER_USB_DEBUG=1` — raise libusb log level from the default WARNING to
  DEBUG (produces ~7 MB per 15 s — has filled `/tmp` mid-capture and adds
  0.5-0.8 s to init even with stderr discarded). `DEVOURER_USB_QUIET` is
  accepted as a no-op for backwards compatibility.
- `DEVOURER_THERMAL_POLL_MS=N` — emit periodic `<devourer-thermal>` lines from
  the chip thermal meter (RF[A][0x42][15:10]) paired with the EFUSE baseline:
  `raw` (0..63 thermal units, ~1.5-2 °C each, not absolute °C), `baseline`,
  `delta = raw − baseline`, and a coarse `status` bucket (cool/warm/hot/critical,
  keyed off delta — the meter has no calibrated °C, so this is deliberately
  bucketed rather than a fake temperature). Works on every Jaguar chip; read-only (does not
  alter TX-power tracking). 0/unset = disabled. In `WiFiDriverDemo` (RX) this
  spawns a background poller at the given cadence; in `WiFiDriverTxDemo` it is
  read inline on the TX thread (no extra USB contention) every `N/2` frames.
  Jaguar-1 has no hard thermal TX shutdown — a rising `delta` is the early
  warning that the PA is heating and TX power is being backed off. NB: on the
  8814 the EFUSE baseline is read at the 8812 offset, so the absolute `delta`
  may be off there; the raw trend is still valid.
- `DEVOURER_THERMAL_WARN_DELTA=N` — thermal-units-above-baseline threshold at
  which a one-shot `warn` fires (default `15`); re-arms once the chip cools
  back below it.

`WiFiDriverTxDemo` selects the on-air TX mode with a single env var that it
parses into a `devourer::TxMode` and hands to `RtlJaguarDevice::SetTxMode`
(the canonical runtime API; the demo sends a rate-less beacon so this mode
applies). The library itself is radiotap-driven — a frame that carries its own
rate radiotap overrides the mode per-packet — so there is **no** `DEVOURER_TX_HT_MCS`
gate any more (an HT-MCS radiotap is honoured unconditionally):

- `DEVOURER_TX_RATE=<rate>[/<bw>][/SGI][/LDPC][/STBC]` (case-insensitive). Unset
  = `6M` legacy. Examples: `MCS7`, `MCS7/40/SGI`, `VHT2SS_MCS3/80/LDPC`, `54M`.
  - `<rate>`: `6M`/`9M`/`12M`/`18M`/`24M`/`36M`/`48M`/`54M` (legacy OFDM),
    `MCS0`..`MCS31` (HT), or `VHT1SS_MCS0`..`VHT4SS_MCS9` (VHT).
  - `<bw>`: `20`|`40`|`80`|`160` (default 20; legacy is always 20).
  - `SGI` / `LDPC` / `STBC`: optional modifiers.

(Programmatic equivalent: `dev->SetTxMode(devourer::TxMode{...})`;
`dev->ClearTxMode()` reverts to the built-in default.)

`SvcTxDemo` is a per-packet-rate showcase: it reads length-prefixed HEVC NALs
from stdin, classifies each by `temporal_id` / IRAP-or-parameter-set criticality
(`txdemo/svc_tx_demo/svc_tx.h`), and injects each at the PHY rate its SVC-T layer
deserves — a per-layer unequal-error-protection ladder (robust MCS for base/IDR,
fast MCS for enhancement), since the radiotap is honoured per-packet. The ladder
is `DEVOURER_SVC_LADDER="CRIT=<spec>;T0=<spec>;T1=<spec>;..."` where each `<spec>`
is a `DEVOURER_TX_RATE` string; unset uses the built-in default. On-air check:
`tests/gen_svc_nals.py` (synthetic 1:4:8:16 layer mix) + `tests/svc_uep_onair.sh`.

The SVC ladder is the PHY-MCS half of cross-layer unequal error protection. The
application-FEC half — a sub-block-integrity layer that salvages FCS-failed
frames (`DEVOURER_RX_KEEP_CORRUPTED`) plus a Reed-Solomon outer code with
per-layer redundancy — lives in `tools/precoder/` and is documented in
**`docs/fused-fec.md`**. `StreamTxDemo` adds `DEVOURER_TX_PWR_OVERRIDE=N`
(absolute per-rate TXAGC index 0..63, held) for the marginal-SNR bench setups
that exercise the salvage path.

## Frequency hopping

`RtlJaguarDevice::FastRetune(channel)` is a lean intra-band, same-bandwidth
channel retune (~1.5 ms vs ~275 ms for a full `SetMonitorChannel`) — it does the
RF channel switch only, skipping the per-rate TX-power loop, bandwidth post-set,
and thermal tick a hop doesn't need, and writes `RF_CHNLBW` from a cached value
so it never pays the C-cut RF-read sleep. It falls back to the full path on a
band change. `send_packet` also honours a radiotap `CHANNEL` field, so hopping is
per-packet and radiotap-driven like rate. Both demos hop via
`DEVOURER_HOP_CHANNELS="1,6,11"` (+ `DEVOURER_HOP_DWELL_FRAMES`,
`DEVOURER_HOP_ROUNDS`, `DEVOURER_HOP_FAST=0|1|2`, `DEVOURER_HOP_RADIOTAP`,
`DEVOURER_HOP_BW`/`DEVOURER_HOP_OFFSET`). Per-packet hopping doubles as a
frequency-diversity interleaver for the outer FEC. Validated by
`tests/run_hop_validation.sh` (B210 wideband), `tests/hop_parity_check.sh`
(register parity for the 40/80 path), and `tools/precoder/hop_diversity_sim.py`.
The implementation and the in-code techniques are documented in
**`docs/frequency-hopping.md`** (also a porting guideline for other chip
generations).

`WiFiDriverTxDemo` also honours a TX-gain ramp + duty knob for thermal /
TX-power characterisation (drives `RtlJaguarDevice::SetTxPowerOverride` +
`ApplyTxPower`):

- `DEVOURER_TX_PWR_START=N` — force an absolute per-rate TXAGC index (0..63),
  bypassing the EFUSE per-rate table. Unset = normal EFUSE-driven power.
- `DEVOURER_TX_PWR_STOP=N` / `DEVOURER_TX_PWR_STEP=N` / `DEVOURER_TX_PWR_STEP_MS=N`
  — step the override from START up to STOP by STEP every STEP_MS, in one
  uninterrupted TX session, emitting a `<devourer-txpwr>index=N` marker per step.
  The override only moves on-air power for OFDM/HT/VHT rates — drive HT with
  `DEVOURER_TX_RATE=MCS1` (the CCK path tracks the index in-register but the
  SDR-measured swing is dominated by the CCK path).
- `DEVOURER_TX_GAP_US=N` — inter-frame gap in microseconds (default 2000,
  ~500 fps). `0` = back-to-back for maximum TX duty (heating experiments).
- `DEVOURER_TX_PWR_READBACK=1` — after each override apply, print
  `<devourer-txpwr-rb>` with the read-back TXAGC registers (0xc20 CCK 1M /
  0xc24 OFDM 6M) to confirm the write landed.

The reusable experiment harness lives in `tests/thermal_gain_sweep.py`
(orchestrator), `tests/sdr_power_probe.py` (USRP receive-power ground truth),
and `tests/run_thermal_gain_sweep.sh` (build + uv venv + sudo run).

## Architecture

**The caller owns libusb.** `WiFiDriver::CreateRtlDevice` is intentionally
thin — `libusb_init`, device open, kernel driver detach, and
`libusb_claim_interface(handle, 0)` must happen **before** handing the handle
to the factory. `demo/main.cpp` is the canonical boilerplate.

**Chip identity is resolved at construction** from SYS_CFG bits + USB PID.
`CreateRtlDevice` returns an `IRtlDevice` (`Init` / `InitWrite` / `send_packet`)
and dispatches on chip generation: Jaguar-1 PIDs construct `RtlJaguarDevice`;
Jaguar3 PIDs construct `RtlJaguar3Device`, with the generation (`rtl8822c` vs
`rtl8822e`) selected from the `SYS_CFG2` chip-id (`0x13` → C8822C, `0x17` →
C8822E). Each orchestrator drives bring-up, RX, and TX through its own HAL.
`RtlJaguarDevice` was previously named `Rtl8812aDevice` — a deprecated alias
still exists for one release cycle.

Generation-agnostic core in `src/` (always compiled; depends on neither
generation's HAL):

- `WiFiDriver` — the factory (`CreateRtlDevice`).
- `RtlUsbAdapter` — libusb wrapper (vendor control + bulk transfers).
- `Radiotap.c` — radiotap header iterator. TX buffers passed to
  `send_packet` **must** begin with a radiotap header; rate / MCS / VHT /
  STBC / LDPC / SGI / bandwidth are read from it.
- `RateDefinitions.h` (`MGN_RATE`), `RxPacket.h` (`Packet` + RX descriptor
  types), `TxDescBits.h` (`SET_BITS_TO_LE_4BYTE` + LE bit-field helpers) —
  the symbols both generations' parsers build on, kept neutral so neither
  generation's header pulls in the other's.

The Jaguar1 HAL lives under `src/jaguar1/`:

- `RtlJaguarDevice` — top-level orchestrator (Init / InitWrite / send_packet).
- `HalModule` — chip bring-up, power sequencing, BB/AGC/RF table application,
  USB EP priority, FIFO/page-boundary init. Most of the chip-family-specific
  divergence lives here (`_8812A`, `_8814A`, `_8821A` suffixed methods).
- `RadioManagementModule` — channel, bandwidth, TX power; supports up to 4
  RF paths for 8814.
- `EepromManager` — EFUSE / EEPROM read + autoload state; supplies the
  `cut_version` / `rfe_type` used by `PhyTableLoader::CheckPositive`.
- `FirmwareManager` — chip-specific FW download (`FirmwareDownload_8814A`
  uses the 3081-DDMA path; 8812/8821 use the page-write path).
- `PhyTableLoader` — runtime walker for Realtek's phydm-format register
  tables (opcode-encoded conditional blocks). Replicates upstream phydm's
  `check_positive` + state machine **without pulling in phydm itself**.
- `FrameParser` — RX parsing and TX descriptor layout (`SET_TX_DESC_*_8812`
  macros are shared across the Jaguar family).

The Jaguar3 HAL is a parallel, self-contained set under `src/jaguar3/`. The core
uses generation-neutral names; per-generation behaviour is behind two strategy
interfaces selected by `ChipVariant`:

- `RtlJaguar3Device` — orchestrator (Init / InitWrite / send_packet / Stop).
- `HalJaguar3` — bring-up: power-on/off PWR_SEQ, MAC/USB config, BB/AGC/RF table
  apply, `config_phydm_parameter_init` (3-wire RF), `bf_init`, `enable_tx_path`,
  efuse access (incl. the 8822e OTP burst-mode / 2-byte-header decode), RFE pins.
- `HalmacJaguar3Fw` / `HalmacJaguar3MacInit` — HalMAC firmware download + MAC init.
- `Jaguar3Calibration` (interface) → `Halrf8822c` / `Halrf8822e`: halrf DACK, IQK,
  TXGAPK, thermal TX-power tracking, coex/antenna control.
- `Jaguar3PhyTables` (interface) → `Phy8822cTables` / `Phy8822eTables`.
- `RadioManagementJaguar3` — channel / bandwidth / per-path TX power, the
  `0x9b0`/`0x9b4` narrowband divider recipe, and the RF18 channel-tune encoding.
- `FrameParserJaguar3.h` / `PhyTableLoaderJaguar3` — TX/RX descriptors and the
  phydm table walker (shared across both generations).

`hal/` holds vendor headers and tables ported from Realtek's tree. The
8814-specific tables under `hal/phydm/rtl8814a/Hal8814_PhyTables.{c,h}` are
**generated** from the upstream aircrack-ng/rtl8814au source by
`tools/extract_8814a_phy_tables.py`; the Jaguar3 tables under
`hal/phydm/rtl8822{c,e}/` by `tools/extract_8822c_phy_tables.py`. Edit the
generators, not the output. The runtime parser is `src/jaguar1/PhyTableLoader`
(Jaguar1) / `src/jaguar3/PhyTableLoaderJaguar3`, not the upstream phydm parser.

## Hardware gotchas

- **ZeroCD trap**: some Realtek dongles enumerate first as a USB
  mass-storage device (`0bda:1a2b` is the canonical offender) exposing the
  Windows installer, then re-enumerate as the NIC after a mode switch. If
  `libusb_open_device_with_vid_pid` returns NULL, check `lsusb` — may need
  `usb_modeswitch` first.
- **rmmod/sysfs-unbind actively de-inits the chip** (RF off, MAC DMA off).
  After detaching a kernel driver, expect to re-init from cold, not warm.
  `DEVOURER_SKIP_RESET=1` only helps when firmware state is still intact.
- **Bench measurement**: measure TX as on-air **Mbps via SDR duty × PHY rate**
  (`tests/bench_onair.py`), not monitor-sniffer frame counts — a sensitive
  receiver decodes weak frames and masks a real drop. Keep a known-good control
  adapter and re-check it each session; take one clean SDR read per session (a
  second back-to-back `sdr_duty` read can fail to reacquire and report ~0).

## TX path

```cpp
auto logger = std::make_shared<Logger>();
WiFiDriver driver(logger);
auto dev = driver.CreateRtlDevice(handle);  // handle is already claimed
dev->InitWrite(SelectedChannel{ .Channel = 36, .ChannelOffset = 0,
                                .ChannelWidth = CHANNEL_WIDTH_20 });
dev->send_packet(buffer, len);  // buffer[0..] = radiotap header, then 802.11
```

The canonical test beacon (`txdemo/main.cpp`) uses SA `57:42:75:05:d6:00` —
the same constant is hardcoded into `demo/main.cpp` as the `<devourer-tx-hit>`
matcher and into `tests/regress.py` (`CANONICAL_SA`). Change all three
together if it ever moves.
