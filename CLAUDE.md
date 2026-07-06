# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Userspace re-implementation of Realtek's RTL88xxAU Wi-Fi driver — speaks to the chip
directly via libusb instead of a kernel module. Static library `WiFiDriver` + two
demo binaries (`WiFiDriverDemo` for RX, `WiFiDriverTxDemo` for TX). Used by the
OpenIPC project for long-range video links.

Three chip generations are supported, dispatched at construction (see
**Architecture**):

**Jaguar (1st-gen 802.11ac).** The original family: **RTL8812AU** (2T2R,
reference), **RTL8811AU** (1T1R cut of 8812 silicon — rides the 8812 code
path with `RFType=RF_TYPE_1T1R` selected via `REG_SYS_CFG` bit 27),
**RTL8814AU** (4T4R RF / 3-SS baseband; host-pushed TX requires the
on-chip 3081 MCU, which devourer boots during firmware download —
a failed FW-boot poll means dead TX while RX still works), and
**RTL8821AU** (1T1R AC + BT combo). These share one HAL (`src/jaguar1/`,
e.g. `HalModule`, `RtlJaguarDevice`) with chip-family branches.

**Jaguar2.** A second, self-contained HAL under `src/jaguar2/` covering two chips
that share one core: **RTL8822BU** (chip 8822B, 2T2R USB, `2357:012d` Archer T3U /
`0bda:b82c` — the T3U reports 2T2R via `REG_SYS_CFG` bit 27, its 1T1R cut
**RTL8812BU** rides the same path) and the 1T1R AC600 + BT **RTL8821C**
(RTL8811CU / RTL8821CU, `0bda:c811`, `SYS_CFG2` chip-id `0x09`). It is a hybrid of
the other two: HalMAC firmware download, MAC init and power sequencing follow the
Jaguar3 path, while the BB/AGC/RF register tables use the older phydm
`check_positive` format (like Jaguar1, via the shared `PhyTableLoader`). Bring-up
is ported from the vendor rtl88x2bu / rtl8821cu trees — power-on, DLFW, MAC/BB/RF
init, LC calibration + IQK, RX, and on-air TX across **2.4 and 5 GHz at 20/40/80
MHz** with per-rate, bandwidth-aware efuse TX power. Per-generation PHY/RF tables
and calibration sit behind strategy interfaces (`Jaguar2PhyTables`,
`Jaguar2Calibration`) selected by `ChipVariant`, mirroring the Jaguar3 split; the
flow and table-walker are shared. Each variant's `txpwr_lmt` regulatory-limit
table is generated (`tools/extract_8822b_txpwr_lmt.py` worldwide-min rfe_type-3
for the T3U; `tools/extract_8821c_txpwr_lmt.py` for the 8821C); the RF/AGC tune to
the wide channel's **central** channel, and the RF18/LCK `CHNLBW` value keeps the
5 GHz band bits (8/16) the vendor preserves.

**Jaguar3.** A third, self-contained HAL under `src/jaguar3/` covering two PHY
generations that share one core: **rtl8822c** (RTL8812CU / RTL8822CU, `0bda:c812`)
and **rtl8822e** (RTL8812EU / RTL8822EU, `0bda:a81a`). Bring-up is ported from
Realtek's vendor source — power-on, HalMAC firmware download, MAC/BB/RF init,
halrf calibration (DACK, IQK, TXGAPK, thermal tracking), RX, and on-air TX at
20/40 MHz, the **5/10 MHz narrowband** re-clock the Jaguar-1 silicon lacks, and
an **80 MHz** channel that also carries a **40-in-80** frame (a 40 MHz frame on
an 80 MHz-tuned channel, landing on the lower 40 via the TX-descriptor DATA_SC —
the userspace equivalent of `iw 80MHz` + a 40 MHz radiotap; the 40 MHz frame
decodes on a standard HT40 receiver on the lower-40 channel). HT40 tunes the
central channel (primary ± 2, `DEVOURER_HOP_BW=40` / `DEVOURER_HOP_OFFSET`);
80 MHz takes the block's lowest 20 MHz as primary (`DEVOURER_HOP_BW=80`).
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

Runtime TX-power control (all three generations, the adaptive-link power
lever — see `src/TxPower.h`): `SetTxPowerOffsetQdb(qdb)` adjusts power in
quarter-dB relative to the efuse per-rate table (per-rate shape preserved
until rates saturate at the rails; saturation flags in `GetTxPowerState`),
`SetTxPowerIndexOverride(idx)` forces/clears the flat index (`SetTxPower`
forwards here), both live mid-stream and sticky across `SetMonitorChannel`
(re-folded against the new channel's table) and `FastRetune` (hop path never
rewrites TXAGC). `GetTxPowerCaps` reports the family step (0.5 dB Jaguar1/2,
0.25 dB Jaguar3 — on-air-measured via `tests/txpwr_offset_onair.sh`, which
uses a second chip's per-frame RSSI as the sensor: at bench range the B210
front-end limits on the frames at any gain and cannot see TXAGC slopes).
`GetThermalStatus` reads the RF 0x42 meter with the family baseline on every
generation. `TxPowerStepDemo` (examples/txpower/, CLI-only) is the reference
consumer; register-level validation lives in `tests/txpwr_offset_regcheck.sh`
(canary parity vs master, exact-step moves, rails, stickiness, the 8822E
TX+RX 0x41e8 quirk cell). The 8822B's VHT TXAGC sections are programmed from
the same stream-count bases as HT (vendor parity) so the offset covers VHT
there; the Jaguar2 TXAGC block and the 8814A's packed port are write-only, so
their `GetTxPowerState` reports the software shadow (`hw_readback=false`).

**Per-packet TX power (Jaguar2 only).** Distinct from the per-rate TXAGC that
`SetTxPowerOffsetQdb` shifts, the 8822B/8821C TX descriptor has a `TXPWR_OFSET`
field (`+0x14[30:28]`) — a hardware LUT applied per-frame *on top of* the
rate's power at **zero USB cost** (a bitfield in the descriptor `send_packet`
already builds): `0`=none, `1`=-3, `2`=-7, `3`=-11, `4`=+3, `5`=+6 dB
(vendor `PHYDM_OFFSET_*`). `send_packet` honours a per-frame radiotap
`DBM_TX_POWER` field as a dB delta quantized to that LUT
(`jaguar2::txpkt_pwr_step_for_db`, selftest `tests/txpkt_pwr_selftest.cpp`) —
the adaptive-link per-packet power knob (wfb-style injection sets it per
frame); `RtlJaguar2Device::SetTxPacketPowerStep` sets a session default for
rate-less frames. On-air-confirmed both paths (`tests/txpkt_pwr_ofset_onair.sh`:
the LUT tracks on-air power ±dB). The classic 8812AU has no such descriptor
field — its per-packet power *is* per-rate selection (radiotap rate → the
TXAGC table), which the library already does; Jaguar3's `TXPWR_OFSET_TYPE` is
a 2-bit variant left unwired.

**RTL8821AU is Jaguar wave 1** (CHIP_8821 = 7, shares the enum with CHIP_8812),
not Jaguar2 — distinct from the Jaguar2 **RTL8821C** (8811CU/8821CU) despite the
similar name. The Jaguar2 **8822BU/8812BU** and **8811CU/8821CU** USB parts are
supported (see the Jaguar2 HAL above); the **8822BE** (PCIe variant) and the
**Kestrel (11ax)** families remain out of scope — same "AU"/"BU"/"CU" branding,
different bus/baseband.

## Build

```sh
cmake -S . -B build
cmake --build build -j
```

Per-chip build options select which drivers are compiled in — all default ON,
so the command above builds every chip. Turn off the ones you don't need to
drop their firmware blobs + PHY tables (an 8812AU-only `WiFiDriverDemo` is
~1.0 MB vs ~2.6 MB): `DEVOURER_JAGUAR1` (8812/8811/8821), `DEVOURER_8814`
(requires JAGUAR1), `DEVOURER_JAGUAR2_8822B` (8822BU/8812BU),
`DEVOURER_JAGUAR2_8821C` (8811CU/8821CU), `DEVOURER_JAGUAR3_8822C`,
`DEVOURER_JAGUAR3_8822E`.
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
- `DEVOURER_USB_BUS=N` (+ optional `DEVOURER_USB_PORT=a.b.c`) — select a device
  by USB topology instead of first-match VID:PID. Needed when two adapters share
  one VID:PID **and serial** and can't otherwise be told apart (two RTL8814AU
  dongles, e.g. CF-938AC vs CF-960AC). `DEVOURER_USB_PORT` is the dotted libusb
  port path (sysfs `devpath` / `lsusb -t`). Unset = the normal VID:PID open loop.
  Used by `tests/compare_8814_decorrelation.sh`.
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
- `DEVOURER_RX_ALLPATHS=1` — emit a `<devourer-rxpath>` line per canonical-SA
  frame carrying all four RX chains (A,B,C,D) of per-stream `rssi`/`snr`/`evm`,
  where the canonical `<devourer-stream>`/`<devourer-body>` lines surface only
  A,B. Paths C/D are non-zero only on the 8814AU (4T4R); 0 on 2T2R parts.
  Opt-in and on a distinct tag so the two-path format its regex consumers key on
  is untouched. Consumed by `tests/antenna_decorrelation.py` to measure
  inter-chain envelope correlation and realised diversity gain.
- `DEVOURER_RX_PATHS=0xNN` — (Jaguar1 RX) restrict which RX chains the chip
  enables/combines by masking the RX-path-enable register (`0x808` byte 0: bits
  0/4 = path A CCK/OFDM, 1/5 = B, 2/6 = C, 3/7 = D). Default all paths (`0xFF`).
  `0x11` = A only, `0x33` = A+B, `0x77` = A+B+C. Validated on the 8814: a masked
  path's per-chain RSSI collapses to the noise floor while the enabled ones stay
  up. Routes through the runtime `RtlJaguarDevice::SetRxPathMask(mask)` /
  `GetRxPathMask()` API (the adaptive-link spatial-diversity lever — a
  controller with a motion/fade signal switches chains live instead of on the
  toggle's fixed timer); once set, the mask is **sticky** — re-applied after
  every `SetMonitorChannel` (a channel set runs IQK, which saves/restores
  `0x808`), so it survives channel hops. The knob for measuring the chip's
  hardware maximal-ratio-combining gain — compare frame delivery at
  `0x11`/`0x33`/`0x77`/`0xFF` over a *marginal* link (see
  `docs/measuring-spatial-diversity.md`); register + stickiness validation in
  `tests/rx_path_mask_check.sh`. A **toggle spec**
  `0xAA:0xBB[:0xCC]@<ms>` (e.g. `0x22:0xFF@300`) cycles the mask on a timer
  thread instead of setting it once, printing a `<devourer-rxpath-mask>0xNN`
  marker inline with the frame stream on each switch — the stimulus for the
  mobile/fading combining measurement (alternate a fixed chain vs all-4 fast
  relative to RX motion so both sample the same fading). Analyse with
  `tests/mrc_mobility.py`.
- `DEVOURER_RX_CSI_MASK=<f_lo>[-<f_hi>][/wgt]` — (all generations, RX)
  de-weight a frequency range (MHz) in the receive equalizer's per-tone CSI
  mask — the RX half of pseudo preamble puncturing (`src/ToneMask.h`,
  `docs/pseudo-preamble-puncturing.md`). Masks every 312.5 kHz subcarrier in
  the range (e.g. `5230-5250` = the top 20 MHz slice of the ch36 80 MHz
  block); `/wgt` (0..7, default 7) is the Jaguar3 per-tone weight. Applied
  when the RX loop starts, after the channel set; a channel switch reverts it
  (same contract as `DEVOURER_RX_PATHS`). Register-landing checked per chip by
  `tests/tone_mask_regcheck.sh`. NB measured: inert against a *jammed* slice
  (that loss is pre-FCS sync/AGC, upstream of the equalizer) — it targets
  in-band spurs riding on decodable frames.
- `DEVOURER_RX_NBI=<f_mhz>` — (all generations, RX) arm the narrowband-
  interference notch filter at one in-channel frequency (vendor-parity,
  LUT-quantized — a single narrow notch, not a slice mask).
- `DEVOURER_TX_WITH_RX=thread` — (`WiFiDriverTxDemo`) run the RX worker loop on
  a `std::thread` alongside the TX loop, on the **same claimed adapter**: one
  bring-up (`InitWrite`), then `StartRxLoop(packetProcessor)` — the programmatic
  API for concurrent TX+RX on one handle (all three generations). On Jaguar3 the
  env must be set **before** `InitWrite` (it keeps the RX filters open and
  enables the RX path during bring-up — retrofitting RX onto a TX-only bring-up
  is unreliable), and the coex thread's C2H drain yields bulk-IN to the RX loop;
  Jaguar2's shared bring-up always enables RX, so it has no such constraint.
  On the 8822E, TX+RX mode leaves the path-B OFDM TXAGC reference (0x41e8) at
  its table default — any nonzero value in that one field desenses the EU's RX
  to near-deaf (hardware-bisected, value-independent; the rest of the per-rate
  power applies normally). This is the single-radio beamforming self-sounding
  ground station — pair with `DEVOURER_BF_ARM_SOUNDER`/`DEVOURER_TX_NDPA` and
  `DEVOURER_BF_DETECT_REPORT` to sound and capture your own reports in one
  process (`docs/beamforming-self-sounding.md`; hardware-validated on Jaguar1
  (8814AU), Jaguar2 (8822BU) and both Jaguar3 variants — 50k+ self-captured
  reports per 20 s at full sounding rate). Any other
  non-empty value selects the legacy `fork()` RX child — Termux-only
  (`libusb_wrap_sys_device` keeps the fd shared across fork); on regular Linux
  the forked bring-ups race and die with `rtw_read: iostream error`.
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
- `DEVOURER_LINKHEALTH=1` — (`WiFiDriverDemo` RX, needs `DEVOURER_RX_ENERGY_MS`)
  emit a `<devourer-linkhealth>` verdict line per energy window: the RX sensor
  tuple classified into a plain-language cause + fix (`src/LinkHealth.h`). The
  point is to tell a **near-field saturation** problem (strong RSSI + poor EVM —
  *back OFF* TX power, add attenuation/distance) apart from a genuine weak link
  (*add* power) so a user isn't chasing the wrong remedy — EVM, not SNR, is the
  saturation tell (SNR looks fine while the constellation collapses). Verdicts:
  `SATURATED` / `INTERFERENCE` / `WEAK` / `MARGINAL` / `HEALTHY` / `NO_SIGNAL`.
  Uses `rssi_max` (window peak) as the strength signal since near-field
  saturation drags the mean down. Thresholds calibrated on-air
  (`tests/saturation_knee_sweep.sh`, `tests/j3_dig_penalty_sweep.sh`), unit-
  guarded (`tests/link_health_selftest.cpp`), SAT-vs-HEALTHY verified on-air
  (`tests/link_health_onair.sh`). See **`docs/bench-testing-near-field.md`**.

`WiFiDriverTxDemo` selects the on-air TX mode with a single env var that it
parses into a `devourer::TxMode` and hands to `RtlJaguarDevice::SetTxMode`
(the canonical runtime API; the demo sends a rate-less beacon so this mode
applies). The library itself is radiotap-driven — a frame that carries its own
rate radiotap overrides the mode per-packet — so there is **no** `DEVOURER_TX_HT_MCS`
gate any more (an HT-MCS radiotap is honoured unconditionally):

- `DEVOURER_TX_RATE=<rate>[/<bw>][/SGI][/LDPC][/STBC]` (case-insensitive). Unset
  = `6M` legacy. Examples: `MCS7`, `MCS7/40/SGI`, `VHT2SS_MCS3/80/LDPC`, `54M`.
  - `<rate>`: `1M`/`2M`/`5.5M`/`11M` (CCK, 2.4 GHz only — the chip drops CCK at
    5 GHz; `1M` is the most robust rate, ~9 dB more link budget than `6M` for a
    long-range beacon), `6M`/`9M`/`12M`/`18M`/`24M`/`36M`/`48M`/`54M` (legacy
    OFDM), `MCS0`..`MCS31` (HT), or `VHT1SS_MCS0`..`VHT4SS_MCS9` (VHT).
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

`IRtlDevice::FastRetune(channel)` is a lean intra-band, same-bandwidth channel
retune every generation implements (default = the full `SetMonitorChannel`): it
does the RF channel switch only, skipping the steady-state work a hop doesn't
need, and runs write-only from a compose cache (full dwords primed once per
epoch, bit changes composed in memory — a masked register write is otherwise a
read+write USB round-trip, and hop cost is purely transfer count × the chip's
EP0 latency). Measured medians: 8812AU ~277→1.6 ms, 8822BU ~65→2.5 ms, 8821CU
~30→0.55 ms, 8822CU/8812EU ~12→2-2.4 ms — FHSS-grade on every generation
(soak: 12k consecutive dwell-1 per-packet hops, zero TX stalls; kickless
hopping RX holds a constant catch rate). `DEVOURER_HOP_PROF=1` emits per-stage
hop timing. It falls back to the full path on a band change; on Jaguar3 it
serializes on the coex thread's register mutex (the sanctioned in-session hop)
and preserves the 5/10 MHz narrowband dividers across hops. `send_packet` on
all three generations also honours a radiotap `CHANNEL` field, so hopping is
per-packet and radiotap-driven like rate. Both demos hop via
`DEVOURER_HOP_CHANNELS="1,6,11"` (SweepSpec grammar — also `36-48/4` channel
ranges and `5170-5250/5` MHz ranges; + `DEVOURER_HOP_DWELL_FRAMES`,
`DEVOURER_HOP_ROUNDS`, `DEVOURER_HOP_FAST=0|1|2`, `DEVOURER_HOP_RADIOTAP`,
`DEVOURER_HOP_BW`/`DEVOURER_HOP_OFFSET`). Per-packet hopping doubles as a
frequency-diversity interleaver for the outer FEC. Validated by
`tests/run_hop_validation.sh` (B210 wideband), `tests/hop_parity_check.sh`
(family-aware register parity — all three generations dump the same canary
from both paths), and `tools/precoder/hop_diversity_sim.py`. The
implementation, the in-code techniques, and the per-generation ports are
documented in **`docs/frequency-hopping.md`**.

The hop-driven RX counterpart is the sweep/sounding pair: `DEVOURER_RX_SWEEP`
dwells FastRetune-cheap bins emitting per-bin energy + frame stats
(`DEVOURER_RX_SWEEP_FULL=1` forces full retunes; `DEVOURER_RX_AGG_SA=canon`
filters the frame stats to the canonical probe SA), and
`tests/sounding_sweep.sh` + `tests/sounding_map.py` run the two-ended active
sounding that recovers a coarse per-bin H(f) of the link — down to 5 MHz bins
on Jaguar3 (`docs/rx-spectrum-sensing.md`).

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
`CreateRtlDevice` returns an `IRtlDevice` (`Init` / `InitWrite` / `send_packet` /
`StartRxLoop` — the last is the blocking RX worker loop on an already-brought-up
chip, so one process can `InitWrite` once and run TX + RX concurrently on the
same claimed handle; `Init` = bring-up + `StartRxLoop` for RX-only callers)
and dispatches on chip generation: Jaguar-1 PIDs construct `RtlJaguarDevice`;
Jaguar2 PIDs construct `RtlJaguar2Device`, with the variant selected from the
`SYS_CFG2` chip-id (`0x0a` → C8822B, `0x09` → C8821C); Jaguar3 PIDs construct
`RtlJaguar3Device`, with the generation (`rtl8822c` vs `rtl8822e`) selected from
the `SYS_CFG2` chip-id (`0x13` → C8822C, `0x17` → C8822E). Each orchestrator
drives bring-up, RX, and TX through its own HAL.
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

- `RtlJaguarDevice` — top-level orchestrator (Init / InitWrite / send_packet /
  StartRxLoop).
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

The Jaguar2 HAL is a parallel, self-contained set under `src/jaguar2/`, structured
like Jaguar3: a shared core (`RtlJaguar2Device`, `HalJaguar2`,
`HalmacJaguar2Fw`/`MacInit`, `FrameParserJaguar2.h`) with per-variant behaviour
behind two strategy interfaces selected by `ChipVariant` (`C8822B` / `C8821C`):

- `Jaguar2PhyTables` (interface) → `Phy8822bTables` / `Phy8821cTables`: the
  `check_positive`-format MAC/BB/AGC/RF register arrays, walked by the shared core
  `PhyTableLoader`.
- `Jaguar2Calibration` (interface) → `Halrf8822b` / `Halrf8821c`: IQK/LOK/TXK/RXK,
  LCK, and WiFi-only coex/antenna control.

`HalJaguar2::set_channel_bw` does channel + 20/40/80 MHz bandwidth
(central-channel tune, with the 8821C 1T1R BTG/WLG/WLA RF-set); `apply_tx_power`
programs the per-rate, bandwidth-aware efuse TXAGC clamped to the variant's
generated `txpwr_lmt` table. Every strategy seam defaults to `C8822B`, so the
8822B path stays byte-identical when the 8821C variant is compiled out.

The Jaguar3 HAL is a parallel, self-contained set under `src/jaguar3/`. The core
uses generation-neutral names; per-generation behaviour is behind two strategy
interfaces selected by `ChipVariant`:

- `RtlJaguar3Device` — orchestrator (Init / InitWrite / send_packet /
  StartRxLoop / Stop).
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
`hal/phydm/rtl8822{c,e}/` and the Jaguar2 tables under `hal/phydm/rtl8822b/` and
`hal/phydm/rtl8821c/` by `tools/extract_8822c_phy_tables.py` (`--chip 8822b` /
`--chip 8821c`); the `Hal8822b_TxpwrLmt.h` / `Hal8821c_TxpwrLmt.h` regulatory
limits by `tools/extract_8822b_txpwr_lmt.py` / `tools/extract_8821c_txpwr_lmt.py`;
and the 8821C firmware blob (`hal/hal8821c_fw.{c,h}`) by
`tools/extract_8821c_fw.py`. Edit the generators, not the output. The runtime
table walker is the shared `PhyTableLoader` (Jaguar1 + Jaguar2, the latter fed by
`Jaguar2PhyTables`) / `src/jaguar3/PhyTableLoaderJaguar3` (Jaguar3), not the
upstream phydm parser.

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
