# MT7610U — a receive-only MediaTek backend, measured

`src/mt76x0u/` drives the MediaTek mt76x0 MAC over USB — MT7610U (the part
measured here), MT7630U and MT7650U ids — behind `IRadio` as `Mt76x0uRadio`.
It is a port of the kernel's `mt76x0u` (`reference/mt76` @ `be5ce79`), built
for one job: monitor capture. It is **receive-only by design**: `send_packet`
is refused, there is no TX power or TSSI calibration, and no beacons.

Build with `-DDEVOURER_MT76X0U=ON`. The firmware (`mt7610e.bin`, falling back to
`mt7610u.bin`, as the kernel does) is not embedded — like the MT7612U's it is
linux-firmware's, under its own licence — and is searched for at runtime
(`DeviceConfig.mt76x0u.firmware_dir`, `DEVOURER_MT76X0U_FW_DIR`).

Code map and deliberate departures from the kernel: `src/mt76x0u/README.md`.

## Setup measured

- Adapter: MT7610U `0e8d:7610` (ASIC `76100002`, EEPROM v1 fae 08), two units.
- Hosts: a native Linux host (Ubuntu 26.04, kernel 7.0, USB 2.0 hub port) and
  a Windows host running WSL2 (kernel 6.18) with the adapter attached through
  usbipd-win 5.3.
- Reference: the kernel `mt76x0u` driver on **the same adapter**, run
  sequentially (one adapter, so never simultaneously).
- Default configuration throughout (two-pass channel set, control frames kept).

## Bring-up

| | native | WSL2 + usbipd |
|---|---|---|
| cold (after a replug, firmware uploaded) | first RX 1.2 s after start | 1.95 s (see below) |
| warm (firmware kept), 5 restarts | 5/5, first frame 0.51–0.64 s | 5/5, 0.86–1.29 s |
| `kill -9` under traffic, then restart | 3/3 (RX_BUSY drained 8–15 KB) | 1/1 (24,960 B drained) |

Counterpart: each cold figure is a single replug per host after the fix below
— enough to show the path works, not a distribution.

**Cold start over USB/IP.** The chip does not answer the first EP0 read after
the firmware start command (DEV_MODE 0x12 + IVB). That is true natively too
(usbmon: the read times out at 300 ms and the next answers at once), where the
cancelled read is harmless. Over usbipd it is not: every later read timed out —
30 of 30 over 9 s; a 2 s read got nothing either — until an EP0 write went
through. Before the fix a cold start under WSL only came up on the caller's
reopen (5.5 s, reproduced twice). `mt0_fw_wait_running` now reads once per
attempt and, after a read with no answer, writes `MT_FCE_PSE_CTRL = 1` (the
write the kernel sequence makes right after this wait) before reading again.
With it the first attempt succeeds on both hosts (native: started after
307 ms, one extra write). Why a write releases EP0 under usbipd was observed
in three traces, not explained.

**Warm start.** The kernel's warm path keeps running firmware; wifit3's forces
a re-upload, which on this chip left EP0 silent for over 3 s and the RX DMA
jammed on every later warm start. So a forced re-upload onto running firmware
is refused, and every warm bring-up drains EP 0x84 if RX_BUSY and pulses the
`MT_PBF_SYS_CTRL` DMA|PBF reset. devourer's USB reset at open is skipped for
this chip's exclusive ids: it rewinds the in-band command ring while the
running firmware's index stays put, and bring-ups then alternated fail/ok.

## Receive

| what | result | counterpart |
|---|---|---|
| beacon ratio, strong APs (−33…−53 dBm) | 0.96–0.99 of expected, ch 1/6/11/36 | reference was the kernel on the same adapter, run sequentially (no second receiver); weaker APs scatter more |
| RSSI vs kernel | within 0.0–0.8 dB (injected frames, ch 6 at 20/40 MHz, ch 36; beacons ch 1/6/11/36) | same adapter, sequential runs |
| HT20, injected MCS 0/3/7 from a known transmitter | 100/100 unique frames each, ch 6 and ch 36 — same as the kernel | one transmitter, one distance |
| HT40 | received (MCS 5 and 0, 1 stream, ch 44) | only 12 frames: nearby traffic is 802.11ax or 2-stream |
| EAPOL 4-way handshake | 5/5 reconnects with all four messages (WPA3-SAE) | one AP, one client |
| control frames, unicast data both directions | received | — |
| malformed frames | 0 in every capture (largest: 300k frames) | — |
| FCS | not delivered by this part; the radiotap FCS flag stays clear | — |

**What it cannot receive.** The MT7610U is 1T1R: it cannot decode 2-stream
frames (HT MCS 8+, VHT NSS 2) or 802.11ax at all, with any driver. Near a
modern AP and client, their data shows up only as the legacy-rate
ACK/CTS/block-ack around it. **VHT reception is not demonstrated**: no
1-stream 802.11ac transmitter was available (an Intel client limited to one
TX chain crashed its own firmware on association).

**Downstream.** Fed to a monitor-mode device classifier for 30 minutes each,
back to back on the same adapter, kernel driver then this backend: 97 vs 94
devices, 90 in both and all 90 classified identically; beacons 56,990 vs
58,893 from the same 88 BSSIDs. The 12 % gap in total frames was control
frames on one busy channel and probe requests from randomised MACs — traffic,
not reception. Counterpart: two sequential windows, not one shared one.

## Channel set

| | native | WSL2 + usbipd |
|---|---|---|
| two passes (default), median | 105 ms (0.130 s wall-clock per call) | 192 ms |
| one pass (`DEVOURER_MT76X0U_TUNE_TWICE=0`), median | 51 ms | 102 ms |

Two passes is the default because wifit3 reports that one pass "leaves the
chip's MCU mid-state and the next channel switch's first command wedges".
Counterpart: that wedge was **not** reproduced — 2,150 single-pass tunes over
a 43-entry hop list, 0 failures — so the default is caution, not a measured
need. There is no `FastRetune` (the base class falls back to the full path),
and no 80 MHz (only the 20/40 MHz path is ported).

DFS channels tune (52–64, 100–144 are flagged radar, fixed rather than asked
of a regdomain); RX on them is untested for lack of an AP.

## Long run and loss

- 24 h soak under a hopping monitor application: 8,003,335 frames received
  and written, 7,049 channel sets, 0 errors; afterwards 27.8 h continuous with
  the only channel-set errors those during a deliberate unplug.
- Unplug while running: the RX loop returns (`rx_no_device`/`rx_ring_dead` in
  `mt76x0u_stats`) instead of leaving the caller believing RX is up; the
  caller reclaims on replug (4.6 s after the replug in the last test).
- usbipd detach/attach while running: recovered twice, 2.5–4 s after the
  adapter reappeared.

## Offline tests

`ctest` with the option on: USB-id table vs the kernel's, channel math (every
entry of a 43-entry real hop list, HT40 groups, refusals), EEPROM decode,
firmware image parsing and chunk framing, the firmware-start wait over a
simulated USB/IP-style EP0, the RX ring, the RXWI mapping, and `mt76x0u_wire`
— the library against a scripted MT7610U (`tests/mt76x0u_wire_model.py`):
cold boot, warm start keeps the firmware, forced re-upload refused.

## Open list

- VHT RX, with a 1-stream 802.11ac transmitter.
- RX on DFS channels with traffic.
- MT7630U / MT7650U: ids are routed, no unit was tested.
- macOS / Windows (MSVC) builds: compiled by CI only once this option is in
  the matrix; never run there.
