# devourer

The Realtek 11ac driver that simply devours its competitors.

Devourer is a userspace re-implementation of Realtek's RTL88xxAU Wi-Fi
driver, speaking to the chip directly through libusb. It covers two chip
generations: the first-generation **Jaguar** 802.11ac family (RTL8812AU,
RTL8814AU, and RTL8821AU shipping on every band, RTL8811AU via the 8812
code path) and the second-generation **Jaguar3** parts — `rtl8822c`
(RTL8812CU / RTL8822CU) and `rtl8822e` (RTL8812EU / RTL8822EU) — which
additionally reach **5/10 MHz narrowband** operation the Jaguar-1 silicon
physically can't. No kernel module, no
`rtl8812au` DKMS tree — just a C++20 static library (`WiFiDriver`) plus two
demo executables for RX and TX. It is the OpenIPC project's driver of choice
for long-range video links built on top of cheap Realtek 11ac USB radios.

## Hardware landscape

Devourer targets **RTL8812AU**, **RTL8811AU**, **RTL8814AU**, and
**RTL8821AU** — all members of Realtek's first-generation 802.11ac
silicon family, internally codenamed **"Jaguar"**. The HAL,
register-table layout, firmware-download plumbing, and
`SET_TX_DESC_*_8812` macros in `src/jaguar1/FrameParser.h` are shared across
the family; chip-specific EEPROM handling, firmware blobs, and RF tables are
layered on top.

It also targets the second-generation **Jaguar3** parts through a
self-contained HAL under `src/jaguar3/`, dispatched at the factory from the
`SYS_CFG2` chip-id and USB PID: the `rtl8822c` generation (**RTL8812CU** /
**RTL8822CU**, `0bda:c812`, `0bda:c82c`) and the `rtl8822e` generation
(**RTL8812EU** / **RTL8822EU**, `0bda:a81a`). These add **5/10 MHz narrowband**
operation the Jaguar-1 silicon lacks. Bring-up is ported from Realtek's vendor
source.

Band cells are **devourer on-air TX throughput** (Mbps, HT MCS7, 20 MHz) via
USRP channel-occupancy (`tests/bench_onair.py`). `†` = on-air but the reading
varies run-to-run (bracketed = best clean reading).

| Part                          | RF / streams      | 2.4 GHz (ch6) | UNII-1 (ch36) | UNII-2/3 (ch149) | Notes                                       |
| ----------------------------- | ----------------- | ------------- | ------------- | ---------------- | ------------------------------------------- |
| **RTL8812AU**                 | 2T2R              | 56            | 52            | 52               | `0bda:8812`; reference part |
| **RTL8811AU**                 | 1T1R              | mirrors 8812  | mirrors 8812  | mirrors 8812     | 1T1R cut of 8812 silicon; rides the 8812 code path (`RFType=RF_TYPE_1T1R` from `REG_SYS_CFG` bit 27). Not benchmarked |
| **RTL8814AU**                 | 4T4R, 3-SS max    | 65            | †(32)         | †(32)            | `0bda:8813`; tested on COMFAST CF-938AC (2 ext antennas) and CF-960AC (4 internal) — effective RX-diversity branches differ (N_eff ≈ 3.8 vs 2.6) despite identical silicon, see [`docs/effective-branches-rotation.md`](docs/effective-branches-rotation.md) |
| **RTL8821AU**                 | 1T1R AC + BT      | 54            | 32            | 28               | TP-Link Archer T2U Plus (`2357:0120`) |
| **RTL8812CU**                 | 2T2R              | 65            | 60            | 60               | LB-LINK WDN1300H (`0bda:c812`) |
| **RTL8822CU**                 | 2T2R + BT         | —             | —             | —                | not benchmarked (`0bda:c82c`) |
| **RTL8812EU**                 | 2T2R              | 8             | 51            | 47               | LB-LINK BL-M8812EU2 (`0bda:a81a`); bare 5 GHz FPV module |
| **RTL8822EU**                 | 2T2R + BT         | —             | —             | —                | not benchmarked (`rtl8822e`) |

The **`Jaguar2`** / **`Jaguar+`** family (8812BU, 8822**B**U/BE, etc.) and
the later **`Kestrel`** 11ax generation are **out of scope**: they share
the Realtek "AU" / "BU" branding but the baseband and HAL differ enough
that they would need their own driver. Two naming traps worth calling out:
RTL8821AU is Jaguar wave 1 (CHIP_8821 = 7 in Realtek's HalVerDef, shares
the enum with CHIP_8812), **not** Jaguar2; and the in-scope RTL8822**C**U
(Jaguar3, `rtl8822c`) is a different chip from the out-of-scope RTL8822**B**U
(Jaguar2) despite the shared "8822" number.

> Heads up — some Realtek USB sticks ship in "ZeroCD" mode and enumerate first
> as a USB mass-storage device exposing the Windows driver installer
> (`0bda:1a2b` is the canonical offender), then re-enumerate as the NIC after
> a mode switch. If `libusb_open_device_with_vid_pid(ctx, 0x0bda, 0x8812)`
> returns NULL, check `lsusb` — you may need `usb_modeswitch` to flip it
> first.

## Building

Toolchain: CMake ≥ 3.15, a C++20 compiler, and libusb-1.0.

### Linux / macOS

libusb is located via `pkg-config`:

```sh
# Debian/Ubuntu
sudo apt install build-essential cmake pkg-config libusb-1.0-0-dev
# macOS (Homebrew)
brew install cmake pkg-config libusb

cmake -S . -B build
cmake --build build -j
```

### Windows

Dependencies come from vcpkg. Set `VCPKG_ROOT` so the CMake toolchain file at
`$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake` resolves:

```powershell
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat
.\vcpkg integrate install
.\vcpkg install libusb
```

Then the same `cmake -S . -B build && cmake --build build` from the project
root.

### Build artifacts

- `WiFiDriver` — static library; link this from your application.
- `WiFiDriverDemo` — RX example built from `demo/main.cpp`. Walks every
  Realtek device under VID `0bda` and tries to open it; sets monitor mode
  on channel 36 / 20 MHz, runs the read loop.
- `WiFiDriverTxDemo` — TX example built from `txdemo/main.cpp`. Opens the
  device via `libusb_open_device_with_vid_pid` by default, or wraps a USB
  fd passed as `argv[1]` (the Termux-on-Android pattern using
  `libusb_wrap_sys_device`). Forks RX into a child, TX-loops a hardcoded
  beacon in the parent.

### Selecting which chips to build

All chips are compiled in by default. Turn off the ones you don't need to
drop their firmware blobs and generated PHY tables and shrink the binary —
an 8812AU-only `WiFiDriverDemo` is ~1.0 MB versus ~2.6 MB with everything on.

| CMake option | default | chips |
|---|---|---|
| `DEVOURER_JAGUAR1` | ON | RTL8812AU / 8811AU / 8821AU |
| `DEVOURER_8814` | ON | RTL8814AU (requires `DEVOURER_JAGUAR1`) |
| `DEVOURER_JAGUAR3_8822C` | ON | RTL8812CU / 8822CU |
| `DEVOURER_JAGUAR3_8822E` | ON | RTL8812EU / 8822EU |

```sh
# 8812AU/8811AU/8821AU only
cmake -S . -B build -DDEVOURER_8814=OFF \
      -DDEVOURER_JAGUAR3_8822C=OFF -DDEVOURER_JAGUAR3_8822E=OFF
```

Configure fails if no chip is selected, or if `DEVOURER_8814` is on without
`DEVOURER_JAGUAR1`. A chip whose support isn't built is rejected at runtime
(the factory returns `nullptr` and logs which).

### Demo env vars

Common to both demos:

- `DEVOURER_PID=0xNNNN` — restrict the device-open loop to a single PID
  (e.g. `0x8813` for RTL8814AU, `0xc812` for the Jaguar3 RTL8812CU). The
  factory picks the Jaguar1 or Jaguar3 HAL from the PID automatically.
- `DEVOURER_VID=0xNNNN` — override VID (default `0x0bda`). Needed for
  OEM-rebadged dongles like the TP-Link Archer T2U Plus (`2357:0120`).
- `DEVOURER_CHANNEL=N` — override the demo's monitor channel (e.g. `6`
  for 2.4 GHz, `36` for 5 GHz).
- `DEVOURER_SKIP_RESET=1` — skip `libusb_reset_device` before claim. Useful
  when picking up a chip whose firmware is already running (e.g. after
  unbinding a kernel driver that left fw state intact).
- `DEVOURER_SKIP_TXPWR=1` — skip the per-rate TX-power loop entirely on
  every chip (it runs by default, including 8814). Useful for fast
  iteration during BB/RF debugging when the per-rate indices aren't
  relevant to what you're measuring.
- `DEVOURER_FORCE_IQK=1` — run phydm I/Q calibration on every channel-set,
  not just band transitions. For 8814, IQK is otherwise off by default —
  the kernel doesn't run it on `iw set channel` either, and devourer
  matches that behaviour.
- `DEVOURER_DISABLE_IQK=1` — never run IQK, even when armed by a band
  transition. Diagnostic — IQK-output BB regs stay at their BB-init seeds.
- `DEVOURER_PHYDM_WATCHDOG=1` — start the periodic phydm DM watchdog
  thread (FA-counter statistics + DIG IGI walk every ~2 s). Off by
  default because the watchdog's BB reads/writes share libusb's
  transfer queue with the TX bulk path and measurably drop sustained
  TX throughput on Jaguar chips. Use for canary-diff workflows and
  RX-only DIG tuning.
- `DEVOURER_DUMP_CANARY=1` — emit a canonical post-channel-set dump of
  BB/MAC/RF anchor registers. Feeds the `tests/canary_diff.py`
  cross-validation tool against `tools/canary_kernel_dump.sh` output.
- `DEVOURER_USB_DEBUG=1` — raise libusb log level from the default WARNING
  to DEBUG (produces ~7 MB per 15 s, can fill `/tmp` mid-capture, and slows
  init measurably). `DEVOURER_USB_QUIET` is accepted as a no-op.

`WiFiDriverTxDemo`-only knobs:

- `DEVOURER_TX_RATE=<rate>[/<bw>][/SGI][/LDPC][/STBC]` — the on-air TX mode,
  parsed into a `devourer::TxMode` and applied via `RtlJaguarDevice::SetTxMode`.
  `<rate>` = `6M`..`54M` (legacy OFDM) | `MCS0`..`MCS31` (HT) |
  `VHT1SS_MCS0`..`VHT4SS_MCS9` (VHT); `<bw>` = `20|40|80|160`. Unset = `6M`.
  Examples: `MCS7`, `MCS7/40/SGI`, `VHT2SS_MCS3/80/LDPC`.
- `DEVOURER_TX_PAYLOAD_BYTES=N` — pad the 802.11 PSDU up to `N` bytes (on-wire
  `N + 40`). For throughput testing — `N=3993` is wfb-ng's max frame payload.

On-air TX throughput vs wfb-ng (SDR-verified parity; how to reproduce) is
documented in [`docs/wfb-ng-tuning.md`](docs/wfb-ng-tuning.md).

## Using the library

The caller owns libusb: you must `libusb_init`, open the device, detach any
kernel driver, and `libusb_claim_interface(handle, 0)` **before** handing
the handle to `WiFiDriver::CreateRtlDevice`. The factory is intentionally
thin — see `demo/main.cpp` for the full boilerplate. A minimal RX path:

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

`packetProcessor` is your `void(const Packet&)` callback. `Init` runs the
RX loop until `should_stop` is set, then returns. For TX, use `InitWrite`
on a channel followed by `send_packet(buffer, len)` where the buffer begins
with a radiotap header (the iterator in `src/Radiotap.c` extracts
rate/MCS/VHT/STBC/LDPC/SGI/bandwidth from it).

## Testing

Out-of-band regression rig in `tests/regress.py` — runs a cross-driver
TX/RX matrix between devourer and the kernel driver across plugged-in
USB Wi-Fi adapters. Default mode is a 4-cell matrix on one ordered
pair; `--full-matrix` extends to all ordered pairs; `--encoding-matrix`
adds radiotap encoding sweeps (HT BCC/LDPC/STBC + VHT BCC/LDPC); and
`--sniffer-iface IFACE` adds a 3rd-adapter monitor capture for
verifying what actually flies on-air.

See [`tests/README.md`](tests/README.md) for setup (host + libvirt VM
modes), per-mode CLI knobs, and the regression matrix's known
limitations (notably: `aircrack-ng/88XXau` strips radiotap LDPC + STBC
on the kernel-TX path, so the `kernel`-TX rows of `--encoding-matrix`
are not authoritative for LDPC/STBC asymmetries — devourer-TX rows
ARE).

### Video link design

The long-range video link's design documents:
[`docs/adaptive-link.md`](docs/adaptive-link.md) — the energy-minimizing adaptive
controller (VTX ↔ VRX), and how it compares to OpenIPC's `alink` and other
adaptive systems; [`docs/adaptive-link-validation.md`](docs/adaptive-link-validation.md)
— its simulation + on-air results, the measurement methodology, and the open
questions, with the [linklab](https://github.com/josephnef/linklab) simulation
sandbox; and [`docs/fused-fec.md`](docs/fused-fec.md) — the cross-layer
(PHY-MCS ⊕ sub-block-integrity ⊕ outer erasure) FEC stack the link's per-layer
quality SLA is stated against.

### Startup time

Devourer reaches ready-to-RX/TX faster than the `aircrack-ng/88XXau`
kernel driver on every supported chip, in both directions (RTL8812AU
~2s, RTL8814AU ~6s, RTL8821AU ~1s cold-init to first frame). Run your
own numbers with `tests/bench_init.py` — it benchmarks cold init per
adapter, devourer vs kernel driver, with a per-stage breakdown from the
library's `init-timing:` log lines.

## Project layout

```
hal/      Vendor headers and tables ported from Realtek's tree
          Hal8812PhyReg.h, hal8812a_fw.[ch], rtl8812a_spec.h
          Hal8814PhyReg.h, hal8814a_fw.[ch], Hal8814PwrSeq.[ch]
          rtl8814a/Hal8814_PhyTables.[ch]    (8814 BB/AGC/RF tables)
          Hal8812a_PhyRegPg.h                (per-rate TX-power PG table)
          Hal8812a_TxpwrLmt.h                (per-region TX-power limit table)
          Hal8812a_TxPwrTrack.[h,cpp]        (phydm thermal-meter delta-swing tables)
          hal8822c_fw.[ch], phydm/rtl8822c/  (Jaguar3 firmware blob + BB/AGC/RF tables)
src/      Driver implementation
          Generation-agnostic core (always compiled):
          WiFiDriver             thin factory (dispatches Jaguar1 vs Jaguar3 by chip-id)
          IRtlDevice             chip-family-agnostic device interface
          RtlUsbAdapter          libusb wrapper (vendor + bulk transfers)
          Radiotap.c             radiotap header iterator
          RateDefinitions.h      MGN_* rate enum (shared by both generations)
          RxPacket.h             RX packet / descriptor types (shared)
          TxDescBits.h           little-endian TX-descriptor bit-field macros (shared)
          TxMode                 runtime TX-mode parsing

          jaguar1/               Jaguar1 (8812/8811/8821/8814) HAL
          RtlJaguarDevice        Jaguar1 orchestrator (RX + TX entry points)
          HalModule              chip bring-up / power sequencing
          RadioManagementModule  channel, bandwidth, TX power, up to 4 RF paths
          EepromManager          EFUSE / EEPROM read + autoload state
          FirmwareManager        chip-specific firmware download
          PhyTableLoader         applies chip-cut-conditional BB/AGC tables
          PowerTracking8812a     phydm thermal-meter TX BB-swing compensation
          Iqk8812a / Iqk8814a    phydm I/Q calibration (8812/8821; 8814 4-path)
          PhydmWatchdog          opt-in periodic DM thread (FA stats + DIG)
          FrameParser            RX parsing, TX descriptor layout

          jaguar3/               Jaguar3 (rtl8822c / rtl8822e) HAL — RtlJaguar3Device,
                                 HalMAC firmware download, halrf calibration, 5/10 MHz
                                 narrowband, per-generation PHY/RF tables + TX/RX descriptors
demo/     RX example
txdemo/   TX example (Android / Termux pattern)
```

Each chip generation can be compiled out to shrink the binary — see
**Selecting which chips to build** below.

## License

GPL-2.0. See [LICENSE](LICENSE).
