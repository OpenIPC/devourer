# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Userspace re-implementation of Realtek's RTL88xxAU Wi-Fi driver — speaks to the chip
directly via libusb instead of a kernel module. Static library `WiFiDriver` + two
demo binaries (`WiFiDriverDemo` for RX, `WiFiDriverTxDemo` for TX). Used by the
OpenIPC project for long-range video links.

Targets the Realtek "Jaguar" 1st-gen 802.11ac family: **RTL8812AU** (2T2R,
reference), **RTL8811AU** (1T1R cut of 8812 silicon — rides the 8812 code
path with `RFType=RF_TYPE_1T1R` selected via `REG_SYS_CFG` bit 27),
**RTL8814AU** (4T4R RF / 3-SS baseband; RX solid, TX validated on
fresh-chip single-cell runs but unstable after USB passthrough cycles —
issue #36), and **RTL8821AU** (1T1R AC + BT combo; proper 8821-specific
init flow landed in PR #42, Android hotplug confirmed end-to-end).

NOT 8821AU's family confusion: it IS Jaguar wave 1 (CHIP_8821 = 7 in
Realtek's HalVerDef, shares the enum with CHIP_8812), not Jaguar2 as
sometimes claimed. The **Jaguar2 (8812BU, 8822BU/BE, ...) and Kestrel
(11ax) families are out of scope** — they share the "AU"/"BU" branding
but the baseband and HAL differ enough to need their own driver.

## Build

```sh
cmake -S . -B build
cmake --build build -j
```

libusb-1.0 is required: `pkg-config` on Linux/macOS, vcpkg on Windows
(`VCPKG_ROOT` must be set so the toolchain file resolves). CI matrix builds
across GCC/Clang/MSVC on Ubuntu/macOS/Windows
(`.github/workflows/cmake-multi-platform.yml`). `ctest` runs in CI but no CMake
tests are registered — regression testing happens out-of-band via
`tests/regress.py`.

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

Default channel is `6` (2.4GHz). Devourer's 5GHz path has known broken
cells for 8814 RX, 8821 TX, and 8821 RX — at 2.4GHz every chip combo
except 8814 TX works. Pass `--channel 36` / `--channel 100` to exercise
5GHz; do not assume a single-band matrix is comprehensive. (The repo
history's matrix tables in PR bodies #34/#42/#49 were all captured at
`--channel 100` and document the 5GHz state.)

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
- `DEVOURER_USB_QUIET=1` — downgrade libusb log level from DEBUG to WARNING
  (DEBUG produces ~7 MB per 15 s and has filled `/tmp` mid-capture).

`WiFiDriverTxDemo` additionally honours radiotap-encoding knobs that
patch the beacon's MCS info field (or, with `_VHT=1`, replace it with a
VHT info field) before the bulk-OUT loop:

- `DEVOURER_TX_MCS=N` — HT MCS index (0..31). Default 1.
- `DEVOURER_TX_LDPC=1` — set FEC type LDPC (vs default BCC).
- `DEVOURER_TX_STBC=N` — STBC stream count (0..3). Default 0.
- `DEVOURER_TX_BW=20|40|80|160` — HT honours 20/40; VHT honours 20-160.
- `DEVOURER_TX_VHT=1` — switch from HT MCS field (13-byte radiotap) to
  VHT info field (22-byte radiotap). Exposes:
- `DEVOURER_TX_VHT_MCS=N` — VHT MCS index (0..9 typical).
- `DEVOURER_TX_VHT_NSS=N` — VHT spatial streams.

`_LDPC` / `_STBC` / `_BW` apply to whichever (HT/VHT) mode is active.

## Architecture

**The caller owns libusb.** `WiFiDriver::CreateRtlDevice` is intentionally
thin — `libusb_init`, device open, kernel driver detach, and
`libusb_claim_interface(handle, 0)` must happen **before** handing the handle
to the factory. `demo/main.cpp` is the canonical boilerplate.

**Chip identity is resolved at construction** from SYS_CFG bits + USB PID;
`RtlJaguarDevice` (the orchestrator) then drives bring-up, RX, and TX
through chip-family branches. The class name was previously `Rtl8812aDevice`
— a deprecated alias still exists for one release cycle.

Module layout in `src/`:

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
- `RtlUsbAdapter` — libusb wrapper (vendor control + bulk transfers).
- `FrameParser` — RX parsing and TX descriptor layout (`SET_TX_DESC_*_8812`
  macros are shared across the Jaguar family).
- `Radiotap.c` — radiotap header iterator. TX buffers passed to
  `send_packet` **must** begin with a radiotap header; rate / MCS / VHT /
  STBC / LDPC / SGI / bandwidth are read from it.

`hal/` holds vendor headers and tables ported from Realtek's tree. The
8814-specific BB/AGC/RF tables under `hal/phydm/rtl8814a/Hal8814_PhyTables.{c,h}`
are **generated** from the upstream aircrack-ng/rtl8814au source by
`tools/extract_8814a_phy_tables.py` — edit the generator, not the output.
The runtime parser for these tables is `src/PhyTableLoader`, not the upstream
phydm parser.

## Hardware gotchas

- **ZeroCD trap**: some Realtek dongles enumerate first as a USB
  mass-storage device (`0bda:1a2b` is the canonical offender) exposing the
  Windows installer, then re-enumerate as the NIC after a mode switch. If
  `libusb_open_device_with_vid_pid` returns NULL, check `lsusb` — may need
  `usb_modeswitch` first.
- **RTL8814AU TX is flaky after USB passthrough cycles** (issue #36):
  fresh-chip single-cell runs send ~4000 frames with 0 submit failures,
  but after one or more virsh attach/detach cycles `libusb_bulk_transfer`
  starts returning `LIBUSB_ERROR_IO` on 90%+ of submits. Every full-
  matrix run since #34 has reproduced this. `RX = devourer` 8814 cells
  are also still 0 (RX path itself is separately broken — not in scope
  for #36).
- **rmmod/sysfs-unbind actively de-inits the chip** (RF off, MAC DMA off).
  After detaching a kernel driver, expect to re-init from cold, not warm.
  `DEVOURER_SKIP_RESET=1` only helps when firmware state is still intact.

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
