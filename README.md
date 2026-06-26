# devourer

The Realtek 11ac driver that simply devours its competitors.

Devourer is a userspace re-implementation of Realtek's RTL88xxAU Wi-Fi
driver (Jaguar family: RTL8812AU, RTL8814AU, and RTL8821AU shipping on
every band, RTL8811AU supported via the 8812 code path), speaking to
the chip directly through libusb. No kernel module, no `rtl8812au`
DKMS tree — just a C++20 static library (`WiFiDriver`) plus two demo
executables for RX and TX. It is the OpenIPC project's driver of choice for long-range
video links built on top of cheap Realtek 11ac USB radios.

## Hardware landscape

Devourer targets **RTL8812AU**, **RTL8811AU**, **RTL8814AU**, and
**RTL8821AU** — all members of Realtek's first-generation 802.11ac
silicon family, internally codenamed **"Jaguar"**. The HAL,
register-table layout, firmware-download plumbing, and
`SET_TX_DESC_*_8812` macros in `src/FrameParser.h` are shared across the
family; chip-specific EEPROM handling, firmware blobs, and RF tables are
layered on top.

Band cells below show **measured devourer TX throughput, Mbps @ 1500 / 3994 B
PSDU** (monitor injection, HT MCS7, 20 MHz; median of runs via
`tests/bench_tput.py`). See [Measured throughput](#measured-throughput) for the
kernel-driver comparison and per-frame latency.

| Part           | RF / streams    | 2.4 GHz (ch6) | 5 GHz UNII-1 (ch36) | 5 GHz UNII-2/3 (ch149) | Notes                                       |
| -------------- | --------------- | ------------- | ------------------- | ---------------------- | ------------------------------------------- |
| **RTL8812AU**  | 2T2R            | 46 / 58       | 49 / 61             | 49 / 62                | VID/PID `0bda:8812`; reference part — fastest + most consistent TX |
| **RTL8811AU**  | 1T1R            | mirrors 8812  | mirrors 8812        | mirrors 8812           | 1T1R cut of 8812 silicon; rides 8812 code path with `RFType=RF_TYPE_1T1R` selected from `REG_SYS_CFG` bit 27. No working unit on the bench — status mirrored from 8812, not separately benchmarked |
| **RTL8814AU**  | 4T4R, 3-SS max  | 0.3 / 22 ⚠    | 2.7 / 0.3 ⚠         | 19 / 23 ⚠              | VID/PID `0bda:8813`; 2-SS effective on USB-2. ⚠ TX throughput is **flaky** (high run-to-run variance, frequent near-zero) — the 8814 monitor-TX path is the least reliable of the family |
| **RTL8821AU**  | 1T1R AC + BT    | 41 / 60       | 23 / 34             | 22 / 33                | OEM-rebadged as TP-Link Archer T2U Plus (`2357:0120`). 2.4 GHz on par with 8812; the ~½ throughput at 5 GHz UNII bands reflects the documented UNII-2/3 cross-receiver asymmetry |

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

### Measured throughput

Numbers below are from `tests/bench_tput.py`, monitor-mode injection at HT MCS7,
20 MHz, PSDU 1500 and 3994 B (the practical max single radio frame). **TX rate is
measured from usbmon bulk-OUT completions at the source chip** — the true
frames-accepted rate, which (unlike counting at a sniffer) is not capped by a
receiver. `dev` = devourer, `ker` = host kernel driver (`rtw88`/`88XXau`,
AF_PACKET injection). Per-frame TX **latency** is the submit→completion time
measured at a non-saturating rate. `—` = unsupported or degenerate.

| Band | Part | TX dev 1500 / 3994 (Mbps) | TX ker 1500 / 3994 | TX latency dev (µs) |
| ---- | ---- | ------------------------- | ------------------ | ------------------- |
| 2.4 GHz (ch6)     | RTL8812AU | 46 / 58 | 0.9 / — | 116 |
| 2.4 GHz (ch6)     | RTL8814AU | 0.3 / 22 ⚠ | 1.0 / — | 29 |
| 2.4 GHz (ch6)     | RTL8821AU | 41 / 60 | 0.8 / — | 128 |
| UNII-1 (ch36)     | RTL8812AU | 49 / 61 | 5.8 / — | 116 |
| UNII-1 (ch36)     | RTL8814AU | 2.7 / 0.3 ⚠ | 5.2 / — | 29 |
| UNII-1 (ch36)     | RTL8821AU | 23 / 34 | 5.3 / — | 122 |
| UNII-2/3 (ch149)  | RTL8812AU | 49 / 62 | 5.9 / — | 115 |
| UNII-2/3 (ch149)  | RTL8814AU | 19 / 23 ⚠ | 5.6 / — | 17 |
| UNII-2/3 (ch149)  | RTL8821AU | 22 / 33 | 5.9 / — | 127 |

Takeaways: (1) devourer's direct-USB TX is **8–60× faster than kernel AF_PACKET
monitor injection** — devourer pipelines bulk-OUT URBs (no per-frame syscall),
the kernel path blocks on the TX ring per frame. (2) The **kernel monitor path
cannot inject 3994 B frames at all** (AF_PACKET > iface-MTU); devourer sends them
at up to 62 Mbps. (3) Throughput scales with frame size (the chip's per-frame
overhead amortises), so 3994 B beats 1500 B everywhere it works. (4) The **8814
TX path is the family's least reliable** (high variance). (5) **RX throughput is
not tabulated**: on a 2-USB-bus bench it cannot be measured cleanly — same-bus
TX/RX pairs contend, the only cross-bus flooder (8812→8814) saturates the
receiver at full TX rate, and the 8814 RX path is itself flaky. RX is functional
(the chips receive); see `tests/README.md` for the measurement caveats.

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
- `DEVOURER_TX_PAYLOAD_BYTES=N` — pad the 802.11 PSDU up to `N` bytes (the
  on-wire frame becomes `N + 40`-byte TX descriptor). Used by the throughput
  benchmark to send 1500 / 3994 B frames. Pad-up only; default = the small
  probe-request beacon.

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
src/      Driver implementation
          WiFiDriver             thin factory
          RtlJaguarDevice        orchestrator (RX + TX entry points)
          HalModule              chip bring-up / power sequencing
          RadioManagementModule  channel, bandwidth, TX power, up to 4 RF paths
          EepromManager          EFUSE / EEPROM read + autoload state
          FirmwareManager        chip-specific firmware download
          PhyTableLoader         applies chip-cut-conditional BB/AGC tables
          PowerTracking8812a     phydm thermal-meter TX BB-swing compensation
          Iqk8812a               phydm I/Q calibration for 8812 / 8821
          Iqk8814a               phydm I/Q calibration for 8814 (4-path)
          PhydmWatchdog          opt-in periodic DM thread (FA stats + DIG)
          RtlUsbAdapter          libusb wrapper (vendor + bulk transfers)
          FrameParser            RX parsing, TX descriptor layout
          Radiotap.c             radiotap header iterator
demo/     RX example
txdemo/   TX example (Android / Termux pattern)
```

## License

GPL-2.0. See [LICENSE](LICENSE).
