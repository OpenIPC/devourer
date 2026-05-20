# devourer

The RTL8812AU driver that simply devours its competitors.

Devourer is a userspace re-implementation of Realtek's RTL8812AU Wi-Fi driver,
speaking to the chip directly through libusb. No kernel module, no `rtl8812au`
DKMS tree — just a C++20 static library (`WiFiDriver`) plus two demo
executables for RX and TX. It is the OpenIPC project's driver of choice for
long-range video links built on top of cheap Realtek 11ac USB radios.

## Hardware landscape

Devourer currently targets **RTL8812AU** on `master`, with **RTL8811AU**
support in flight on `feat/rtl8811au-support`. Both are members of Realtek's
first-generation 802.11ac silicon family, internally codenamed **"Jaguar"** —
which is why the HAL, register tables (`hal/Hal8812PhyReg.h`), firmware blob
(`hal/hal8812a_fw.[ch]`) and the `SET_TX_DESC_*_8812` macros in
`src/FrameParser.h` are reusable across the whole family with mostly EEPROM
and RF-path adjustments rather than a from-scratch rewrite.

| Part           | Streams    | Status in devourer                          | Notes                                          |
| -------------- | ---------- | ------------------------------------------- | ---------------------------------------------- |
| **RTL8812AU**  | 2T2R       | Supported (`master`, VID/PID `0bda:8812`)   | Reference part                                 |
| **RTL8811AU**  | 1T1R       | In progress on `feat/rtl8811au-support`     | Pin/feature-reduced cut of 8812AU              |
| **RTL8821AU**  | 1T1R + BT  | Not supported                               | Same Jaguar PHY combined with Bluetooth        |
| **RTL8814AU**  | 4T4R       | Not supported                               | Jaguar baseband scaled to four streams         |

Successor families (`Jaguar2` / `Jaguar+` — 8812BU, 8822BU/BE, etc., and the
later `Kestrel` 11ax generation) are **out of scope**: they share the Realtek
"AU" branding but the baseband and HAL differ enough that they would need
their own driver.

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
- `WiFiDriverDemo` — RX example built from `demo/main.cpp`. Opens
  `0bda:8812`, sets monitor mode on channel 36 / 20 MHz, runs the read loop.
- `WiFiDriverTxDemo` — TX example built from `txdemo/main.cpp`. Takes a USB
  fd as `argv[1]` (the Termux-on-Android pattern using
  `libusb_wrap_sys_device`), forks RX into a child, and TX-loops a hardcoded
  beacon in the parent.

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

## Project layout

```
hal/      Vendor headers and tables ported from Realtek's tree
          (Hal8812PhyReg.h, hal8812a_fw.[ch], rtl8812a_spec.h, …)
src/      Driver implementation
            WiFiDriver           thin factory
            Rtl8812aDevice       orchestrator (RX + TX entry points)
            HalModule            chip bring-up / power sequencing
            RadioManagementModule  channel, bandwidth, TX power
            EepromManager        EFUSE / EEPROM read + autoload state
            RtlUsbAdapter        libusb wrapper (vendor + bulk transfers)
            FrameParser          RX parsing, TX descriptor layout
            Radiotap.c           radiotap header iterator
demo/     RX example
txdemo/   TX example (Android / Termux pattern)
```

## License

GPL-2.0. See [LICENSE](LICENSE).
