# src/mt7612u — MediaTek MT7612U

**Not reachable from `CMakeLists.txt` yet.** This subtree is a complete,
self-contained C library for the part — a public header, its own transport, no
dependency on `RtlAdapter` — plus the bring-up harness that produced every
measurement in `docs/mt7612u.md`. Wiring it in behind `IRadio` is a
follow-up PR; nothing in the shipped library links against this today.

It builds and tests on its own:

```sh
make -C src/mt7612u            # -> src/mt7612u/bringup
make -C src/mt7612u check      # offline tests: no hardware, no privileges
sudo ./src/mt7612u/bringup regs
```

Measurements, methods and limits: [`../../docs/mt7612u.md`](../../docs/mt7612u.md).

## Layout

| file | what |
|---|---|
| `usb.c` | libusb transport: EP0 vendor register access, sync bulk, open/claim/reset |
| `async.c` | event thread, 16-deep RX ring, 32-slot TX pool |
| `mcu.c` | in-band MCU command framing (EP 8 out, EP 5 in, 4-bit sequence) |
| `fw.c` | ROM patch + ILM/DLM firmware upload |
| `eeprom.c` | 512-byte EEPROM: identity, TX power tables, RX gain |
| `init.c` | power-on, MAC initvals, mac_start/stop, EP-4 flush |
| `phy.c` | band/bandwidth/TX power registers, channel + calibration sequence |
| `tx.c` | TXWI + TXINFO construction |
| `rx.c` | RXWI parse, per-chain RSSI, rate decode |
| `radiotap.c` | `send_packet` / `send_packets` (USB chaining via `NEXT_VLD`) |
| `caps.c` | TSF, capability descriptor, ACK responder |
| `tools/bringup.c` | one subcommand per verified gate |
| `tests/` | offline tests (`make check`): public-API link, frame shapes |
| `initvals.h` | **generated** — see Provenance |

## The receiver must never run undrained

Enabling MAC RX with nothing reading the bulk-IN endpoint wedges this part
*below* the USB level: `libusb_reset_device`, the sysfs `authorized` toggle
and rebinding the kernel driver all fail to recover it, and only a physical
replug does. So `mt_mac_start()` takes the receiver as an explicit argument,
`mt7612u_start()` enables RX only when `mt7612u_rx_start()` is already
running, and every gate that turns RX on starts the ring *first*.

## Portability

Done here, because these are correctness issues regardless of compiler:

- `FIELD_PREP`/`FIELD_GET` no longer use `__builtin_ctz`. MSVC has no such
  builtin, and its `_BitScanForward` takes an out-parameter, so it cannot
  appear in a constant expression - which these must be, since `FIELD_PREP`
  initialises static tables. `MT_CTZ` is a constant expression everywhere and
  folds to one instruction. `tests/field_macros` checks it against the
  builtin over all 32 single-bit and all 528 contiguous masks, and fails to
  compile if it ever stops being constant-foldable.
- The shift macro was named `_SHIFT`. Leading underscore plus a capital is
  reserved to the implementation in every scope.
- `<libusb.h>` (this project's spelling) is tried first, with the
  distribution's `<libusb-1.0/libusb.h>` as the fallback.

**Not** done here: `async.c` uses pthreads and `usb.c` uses `nanosleep` /
`clock_gettime`. This project has no C threading or time shim - its shim is
the C++ standard library, which every other backend uses directly. Building
a throwaway C shim now would be deleted at integration, so those two files
keep POSIX until the subtree joins the build. They are the only two.

## Firmware

Needs `mt7662_rom_patch.bin` and `mt7662.bin` from `linux-firmware`
(`/lib/firmware/mediatek/`, zstd-compressed on most distributions). Not
vendored here. Point `bringup` at a directory holding the decompressed pair:

```sh
zstd -d /lib/firmware/mediatek/mt7662{,_rom_patch}.bin.zst -o firmware/
```

## Gates

Each subcommand is a hardware check that fails loudly, in dependency order:

```
regs   registers + EEPROM round-trip        chan   channel set, 20 MHz
fw     ROM patch + firmware + MCU ack       tx     inject at a fixed rate
init   full bring-up + register-stream log  rx     monitor receive
caps   capabilities, TSF, 40 MHz            soak   sync vs async throughput
pwr    TX power vs the kernel's values      ampdu  aggregation A/B
gateg  per-frame rate control               ack    ACK responder (needs a stimulus)
rtap   send_packet / send_packets           hop    channel-switch cost
```

`make` here builds it as `./bringup`, which is what the hardware notes use.
The integration PR adds a CMake target for the same source, named
`mt7612uprobe` to sit beside `pcieprobe` / `kestrelprobe` / `rtl8733bprobe`, so
the chip-specific tool is not the one part of this backend that only a second
build system can produce.

`sweep`, `coding` and `vht` take a width as their fourth argument, in the
`MT7612U_BW_*` numbering — `0` = 20, `1` = 40, `2` = 80 MHz:

```sh
./bringup sweep 149 120 2      # VHT ladder at 80 MHz, control channel 149
```

The witness has to listen at the same width (`DEVOURER_BW=40|80` for
devourer's own `rxdemo`). A 20 MHz receiver decodes *none* of an 80 MHz
frame — which makes it a good negative control and a misleading oracle.

At 80 MHz the HT ladder is skipped: 802.11n has no 80 MHz, so a rate word
naming `PHY=HT` with `BW=80` is not a wide HT frame, it is an unspecified one.

## Provenance

Register sequences and descriptor layouts are derived from `openwrt/mt76`
(`mt76x2/`, `mt76x02*`, `usb.c`), BSD-3-Clause-Clear, Copyright (C) 2016 Felix
Fietkau, (C) 2018 Lorenzo Bianconi / Stanislaw Gruszka. Files carrying ported
sequences keep that notice. The tree is pinned as `reference/mt76` at commit
`be5ce79`.

`initvals.h` is **generated** from it, not transcribed:

```sh
tools/extract_mt7612u_tables.py            # regenerate
tools/extract_mt7612u_tables.py --check    # byte-compare the checked-in file
```

The generator resolves the symbolic register names against `mt76x02_regs.h` and
evaluates the four `DEFAULT_PROT_CFG_*` macros, so a mistyped address cannot
survive as a plausible-looking number. It reproduces the previously hand-typed
table byte for byte, all sixty rows.

Two things here are **not** ports and were proven on air rather than copied:
the `MT_TXD_INFO_NEXT_VLD` USB chaining in `radiotap.c`, and the ACK responder
in `caps.c`. One thing copied from mt76 was **wrong** — see the TSF note in the
docs.
