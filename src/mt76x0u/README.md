# src/mt76x0u — MediaTek MT7610U / MT7630U / MT7650U

**Receive-only, measured on hardware** (`../../docs/mt76x0u.md`). Compiled by
`CMakeLists.txt` under `DEVOURER_MT76X0U` (default OFF). `Mt76x0uRadio` is
the `IRadio` backend `WiFiDriver::CreateRadio` constructs for an MT76x0U
adapter (`Mt76x0uUsbIds.h`; `7392:b711`, listed by both mt76x0u and
mt76x2u, is routed by reading `MT_ASIC_VERSION` — `../MediatekProbe.h`).

Ported from the kernel's mt76 (`reference/mt76` @ `be5ce79`): `mt76x0/` is
GPL-2.0-only, the shared `mt76x02*` code BSD-3-Clause-Clear, and each file says
which it derives from. wifit3 (github.com/derv82/wifit3, GPLv2, a
hardware-tested Python driver for the same chip) is the cross-check.

## Layout

| file | what |
|---|---|
| `usb.cpp` | transport (copied from mt7612u, symbols `mt0_`): EP0 register I/O, sync transfers, identify, adopt; the test I/O hook |
| `async.cpp` | the EP 0x84 RX ring and its event thread (mt7612u's, minus TX) |
| `mcu.cpp` | in-band MCU channel; `CMD_RANDOM_READ/WRITE` register pairs (refused before `Q_SELECT`); RF access |
| `fw.cpp` | firmware: `mt7610e.bin` then `mt7610u.bin`, IVB split, kept when running (a forced re-upload is refused) |
| `eeprom.cpp` | EFUSE read (usage-map check, KICK protocol) and what mt76x0 derives from it |
| `init.cpp` | power-on sequence and `mt76x0_init_hardware` |
| `phy.cpp` | `mt76x0_phy_init`: antenna, RF init, RX path, TX DAC |
| `chan.cpp` | 20/40 MHz channel set, calibration, the periodic calibration pass, monitor filter |
| `mac.cpp` | MAC start/stop (ring first, receiver second) |
| `rx.cpp` | RXWI parse, L2-pad fold, RSSI correction |
| `initvals.h` | **generated** by `tools/extract_mt76x0u_tables.py` (`--check` is a test) |
| `Mt76x0uRadio.*`, `Mt76x0uMapping.h` | the IRadio backend and its pure RX translations |

## Deliberate departures from the kernel

* Receive-only: no TX power, no TSSI calibrations, no `edcca_tx_enable`.
* 80 MHz refused (ported 20/40 only).
* Radar channels fixed at 52–64, 100–144 (the kernel asks the regdomain).
* Monitor filter adds wifit3's measured address-match override (unicast data
  to other stations is otherwise dropped).
* Each channel set runs **twice** by default — see below.
* Warm start (firmware kept) drains EP 0x84 if RX_BUSY and pulses the
  `MT_PBF_SYS_CTRL` DMA|PBF reset right after the firmware step, and a forced
  firmware re-upload onto running firmware is refused (measured why:
  `docs/mt76x0u.md`, "Warm start").
* The RX loop returns when the adapter is unplugged (a transfer reports
  NO_DEVICE) or the RX ring runs dry, so the caller sees the loss and can
  reclaim.
* After the firmware start command, a read with no answer is followed by a
  `MT_FCE_PSE_CTRL` write before the next read (`mt0_fw_wait_running`): over
  USB/IP the dropped first read otherwise wedges EP0 (`docs/mt76x0u.md`,
  "Cold start over USB/IP").

## Transfer-level comparison with wifit3

`tests/mt76x0u_wire_model.py` is a scripted MT7610U that answers EP0 and bulk
transfers and logs each one canonically. The library runs against it through
`tests/mt76x0u_wire_harness` (ctest `mt76x0u_wire`: cold boot, warm start keeps
the firmware, forced re-upload refused). `tools/mt76x0u_wire_vs_wifit3.py` runs
wifit3's driver against the same model (in a container with no network; it
needs only a pyusb stub). Same firmware (`mt7610e.bin`, byte-identical to
wifit3's bundled copy), cold boot, then channels 1, 6, 36, 100, 157.

Writes and commands only (reads are polling and differ by design), with the
kernel's batched table clears expanded to words:

| phase | ours | wifit3 | in order, identical |
|---|---|---|---|
| bring-up | 1598 | 1610 | 1593 |
| channel set, 2.4 GHz (each) | 344 | 368 | 332–333 |
| channel set, 5 GHz (each) | 296 | 320 | 284 |

Every remaining difference, classified:

| difference | verdict |
|---|---|
| wifit3 writes `MT_FCE_PSE_CTRL` twice before the upload | wifit3 follows its kernel capture; we follow the kernel source (one write) |
| wifit3 random-reads MAC 0x1000 after `Q_SELECT` | its MCU smoke test; diagnostic |
| we kick the EFUSE usage map (2 writes) | kernel `mt76x0_efuse_physical_size_check`; wifit3 skips it |
| we write `CMB_CTRL`, `CSR_EE_CFG1`, `COEXCFG0` in antenna select | kernel; wifit3 documents omitting them |
| wifit3 ends bring-up with MAC start + filter + power-on cal | we do these in `StartRxLoop` (`mt76x0u_start`) |
| wifit3 writes TX-power registers per tune | we are receive-only |
| we write `CH_TIME_CFG` and the EDCCA registers per tune | the kernel's `mt76x0_set_channel` wrapper; wifit3 omits it |
| wifit3 reads back RF registers after a tune | diagnostic |

**What it changed:** wifit3 runs the channel set twice per tune, because its
capture of the kernel shows mac80211 doing so, and it reports that a single
pass "leaves the chip's MCU mid-state and the next channel switch's first
command wedges". The library now does the same by default
(`mt76x0u_set_tune_twice`, `DEVOURER_MT76X0U_TUNE_TWICE=0` to undo). It also
caught, earlier, that the MCU calibration ids are mt76x0's, not mt76x2's.

What this does NOT show: that the chip behaves like the model. Timing, the
warm-start state after a USB reset, RSSI scale and RX itself were measured on
hardware instead (`docs/mt76x0u.md`).

## Measurements

Bring-up, RX against the kernel driver, retune cost, the soak and loss
handling, natively and over WSL2 + usbipd: [`../../docs/mt76x0u.md`](../../docs/mt76x0u.md).
