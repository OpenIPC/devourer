# devourer

The Realtek 11ac driver that simply devours its competitors.

Devourer is a userspace re-implementation of Realtek's RTL88xxAU Wi-Fi
driver (Jaguar family: RTL8812AU + RTL8821AU shipping, RTL8811AU
supported via the 8812 code path, RTL8814AU RX solid + TX validated on
fresh-chip runs), speaking to the chip directly through libusb. No
kernel module, no `rtl8812au` DKMS tree — just a C++20 static library
(`WiFiDriver`) plus two demo executables for RX and TX. It is the
OpenIPC project's driver of choice for long-range video links built on
top of cheap Realtek 11ac USB radios.

## Hardware landscape

Devourer targets **RTL8812AU**, **RTL8811AU**, **RTL8814AU**, and
**RTL8821AU** — all members of Realtek's first-generation 802.11ac
silicon family, internally codenamed **"Jaguar"**. The HAL,
register-table layout, firmware-download plumbing, and
`SET_TX_DESC_*_8812` macros in `src/FrameParser.h` are shared across the
family; chip-specific EEPROM handling, firmware blobs, and RF tables are
layered on top.

| Part           | RF / streams    | Status        | Notes                                       |
| -------------- | --------------- | ------------- | ------------------------------------------- |
| **RTL8812AU**  | 2T2R            | Supported     | VID/PID `0bda:8812`; reference part         |
| **RTL8811AU**  | 1T1R            | Supported     | 1T1R cut of 8812 silicon; rides 8812 code path with `RFType=RF_TYPE_1T1R` selected from `REG_SYS_CFG` bit 27 |
| **RTL8814AU**  | 4T4R, 3-SS max  | RX + flaky TX | VID/PID `0bda:8813`; 2-SS effective on USB-2; TX works on fresh-chip single-cell runs but degrades to `LIBUSB_ERROR_IO` after virsh USB passthrough cycles |
| **RTL8821AU**  | 1T1R AC + BT    | Supported     | OEM-rebadged as TP-Link Archer T2U Plus (`2357:0120`) etc; Android hotplug works end-to-end |

Successor families (`Jaguar2` / `Jaguar+` — 8812BU, 8822BU/BE, etc., and
the later `Kestrel` 11ax generation) are **out of scope**: they share
the Realtek "AU" / "BU" branding but the baseband and HAL differ enough
that they would need their own driver. NB: RTL8821AU itself is Jaguar
wave 1 (CHIP_8821 = 7 in Realtek's HalVerDef, shares the enum with
CHIP_8812), not Jaguar2 — the naming is a known trap.

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
- `WiFiDriverTxDemo` — TX example built from `txdemo/main.cpp`. Takes a USB
  fd as `argv[1]` (the Termux-on-Android pattern using
  `libusb_wrap_sys_device`), forks RX into a child, and TX-loops a hardcoded
  beacon in the parent.

### Demo env vars

Common to both demos:

- `DEVOURER_PID=0xNNNN` — restrict the device-open loop to a single PID
  (e.g. `0x8813` for RTL8814AU).
- `DEVOURER_VID=0xNNNN` — override VID (default `0x0bda`). Needed for
  OEM-rebadged dongles like the TP-Link Archer T2U Plus (`2357:0120`).
- `DEVOURER_CHANNEL=N` — override the demo's monitor channel (e.g. `6`
  for 2.4 GHz, `36` for 5 GHz).
- `DEVOURER_SKIP_RESET=1` — skip `libusb_reset_device` before claim. Useful
  when picking up a chip whose firmware is already running (e.g. after
  unbinding a kernel driver that left fw state intact).
- `DEVOURER_FORCE_TXPWR=1` — force the per-rate TX-power loop to run during
  channel switch. Skipped by default in 8814 monitor mode: the loop issues
  ~300 vendor control transfers and the resulting per-rate indices are
  unused for RX-only operation.
- `DEVOURER_USB_QUIET=1` — downgrade libusb log level from DEBUG to
  WARNING (DEBUG produces ~7 MB per 15 s and can fill `/tmp` mid-capture).

`WiFiDriverTxDemo`-only knobs patch the canonical beacon's radiotap
header before the TX loop:

- `DEVOURER_TX_MCS=N` — HT MCS index. Default 1.
- `DEVOURER_TX_LDPC=1` — FEC type LDPC (vs default BCC).
- `DEVOURER_TX_STBC=N` — STBC stream count (0..3). Default 0.
- `DEVOURER_TX_BW=20|40|80|160` — bandwidth.
- `DEVOURER_TX_VHT=1` — switch from HT MCS field (radiotap bit 19) to
  VHT info field (bit 21). Exposes `DEVOURER_TX_VHT_MCS=N` (VHT MCS
  index, 0..9 typical) and `DEVOURER_TX_VHT_NSS=N` (spatial streams).
  `_LDPC` / `_STBC` / `_BW` apply to whichever (HT/VHT) mode is active.

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

## Project layout

```
hal/      Vendor headers and tables ported from Realtek's tree
          Hal8812PhyReg.h, hal8812a_fw.[ch], rtl8812a_spec.h
          Hal8814PhyReg.h, hal8814a_fw.[ch], Hal8814PwrSeq.[ch]
          rtl8814a/Hal8814_PhyTables.[ch]  (8814 BB/AGC/RF tables)
src/      Driver implementation
          WiFiDriver             thin factory
          RtlJaguarDevice        orchestrator (RX + TX entry points)
          HalModule              chip bring-up / power sequencing
          RadioManagementModule  channel, bandwidth, TX power, up to 4 RF paths
          EepromManager          EFUSE / EEPROM read + autoload state
          FirmwareManager        chip-specific firmware download
          PhyTableLoader         applies chip-cut-conditional BB/AGC tables
          RtlUsbAdapter          libusb wrapper (vendor + bulk transfers)
          FrameParser            RX parsing, TX descriptor layout
          Radiotap.c             radiotap header iterator
demo/     RX example
txdemo/   TX example (Android / Termux pattern)
```

## License

GPL-2.0. See [LICENSE](LICENSE).
