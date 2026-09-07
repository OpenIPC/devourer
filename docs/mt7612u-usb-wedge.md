# MT7612U hard USB TX wedge: reset evidence

Research date: 2026-09-07. Devourer HEAD initially: `4e7a793` on
`feat/mt7612u-mediatek-backend`; concurrent PHY/I/O work was subsequently
committed as `5757712`. Its separate host-side libusb hang report does not
establish the cause of this persistent device-side wedge.
Reference: `openwrt/mt76` at `be5ce7910521492d4a2e4ce7ee3843680a46c047`.
**Status: source-level investigation only; no new device measurements.**

There is a concrete reset candidate beyond `TX_CLR`: the vendor-derived
MT76x2U source has separate UDMA TX and IFDMA/FCE reset controls. Its
`RTMPSwReset()` uses CFG `0x9014[6]` and CFG `0x0064[22:21]`, respectively.
Neither appears in the supplied list of failed experiments. Whether this
sequence clears this adapter's hard wedge remains untested. It is premature
to conclude that the silicon requires re-enumeration.

## Observed failure, not a new measurement

The handover reports EP0 CFG/MAC read/write round-trips still working,
`MT_ASIC_VERSION=0x76120044`, and bulk OUT timing out at the first ROM-patch
chunk on EP8. Kernel-driver binding also fails.

| CFG `0x9018` | Reported behavior |
|---|---|
| `0x00c00020` | Soft wedge, TX_BUSY clear; bit19 `TX_CLR` set, 20 ms, clear recovered 3/3 trials. |
| `0x80c00020` | Hard wedge, TX_BUSY set; repeated TX_CLR, MAC/PBF/WLAN resets, USB port reset, halt clearing, authorization toggle and xHCI rebind failed; physical replug recovered. |

The earlier undrained-RX report in `mt7612u.md` describes vendor requests
timing out too. Do not conflate that failure with this EP0-responsive state.
Physical replug changes power as well as enumeration; its success does not
prove that logical re-enumeration alone is sufficient.

## The missing vendor sequence

Source: [RTMPSwReset(), vendor-derived MT76x2U tree][vendor-reset]. The tree
[identifies itself as JEDI.MP2.mt76x2u.wifi.v3.2.1][vendor-version] and
[lists MT7612U and MT7662U among its targets][vendor-readme]. This is a pinned
public mirror, not an authenticated copy of an untouched MediaTek release.

Use checked read-modify-write operations. In this table, **pulse** means set
the mask, wait **15 ms**, read again, clear the mask, wait **15 ms**.

| Order | Space / offset | Operation | Source's purpose |
|---|---|---|---|
| 1 | CFG `0x9018` | Clear `0x00c00000` | Disable UDMA TX and RX |
| 2 | CFG `0x9080` | Pulse `0x03f00000` | Drop OUT EP4–EP9 data |
| 3 | CFG `0x9014` | Pulse `0x00000040` | Reset UDMA TX |
| 4 | CFG `0x0064` | Pulse `0x00600000` | Reset IFDMA/FCE |
| 5 | MAC `0x0400` | Pulse `0x0000000c` | Reset MAC/PBF |
| 6 | CFG `0x9014` | Pulse `0x00000020` | Reset UDMA RX |
| 7 | CFG `0x9018` | Set `0x00c00000`; wait 15 ms | Enable UDMA TX and RX |

The function's surrounding contract calls for cancelling bulk OUT URBs,
disabling MAC TX/RX (`MAC 0x1004 = 0`), and polling MAC TX idle
(`MAC 0x1200 & 1 == 0`) first. Those actions, firmware reload and final MAC
enable are **commented out in this helper**. Its contract says reload firmware
without asserting WLAN reset at CFG `0x0064[19]`.

There is an actual [call at the start of rt28xx_init()][vendor-caller], before
top-level initialization and `mcu_sys_init()`. Thus the helper is used during
bring-up; it is not evidence of a tested recovery from SIGKILL with TX_BUSY
stuck. Its active reset operations use EP0 register access and delays, with
no USB reset, configuration change or re-enumeration request.

For a userspace experiment, stop new submissions and cancel/reap outstanding
transfers before touching the DMA. Bound and record the MAC-idle poll; do not
turn it into an infinite wait on the fault being investigated. Leave MAC RX
disabled until a live drain exists. Firmware/FCE/channel state must be
reinitialized after the sequence; resuming old queued frames is not a valid
success test. These are integration requirements, not measurements.

## Exact EP0 encoding

The vendor's [CFG accessors][vendor-access] agree with the pinned
[mt76 register accessors][mt76-access]. All offsets above fit in 16 bits.

| Operation | bmRequestType | bRequest | wValue | wIndex | Data |
|---|---|---|---|---|---|
| Read CFG | `0xc0` | `0x47` | `0` | offset | Read 4 bytes, little-endian |
| Write CFG | `0x40` | `0x46` | `0` | offset | Write 4-byte little-endian value |
| Read MAC | `0xc0` | `0x07` | `0` | offset | Read 4 bytes, little-endian |
| Write MAC | `0x40` | `0x06` | `0` | offset | Write 4-byte little-endian value |

In this backend, use `CFG_ADDR(offset)` with the checked register accessors
for CFG space; pass a plain offset for MAC space. Require exactly four bytes
for each read/write and stop on a failed transaction. Never perform an RMW
from a fallback `0xffffffff` read. `0x40` in the vendor's **address-space**
notation is not itself the register-request opcode.

No EEPROM/eFuse write or unknown vendor opcode is needed for this sequence.

## Other vendor requests and FCE state

The following are established controls, but none is independently documented
as a hard USB-DMA recovery command.

| Control | Exact request or write | Meaning / relevance |
|---|---|---|
| Firmware vendor reset | EP0 `(type=0x40, request=0x01, value=0x0001, index=0, length=0)` | [mt76x02u_mcu_fw_reset()][mt76-fw]. Already called by local `fw_reset()` before the failing patch upload. |
| WMT reset | EP0 `(type=0x20, request=0x01, value=0x0012, index=0, length=8)`, payload `6f fc 05 01 07 01 00 04` | [mt76x2u_mcu_reset_wmt()][mt76-wmt]. **Class**, not vendor request. Normally follows patch upload/enable, with 20 ms settling; the reported failure occurs before that point. No proof it works as a pre-upload DMA reset. |
| WRITE_FCE | Request `0x42`; two zero-data OUT requests encode the low/high 16-bit halves in `wValue`, at `wIndex=offset` and `offset+2` | [mt76u_single_wr()][mt76-single]. Writes FCE upload address/length; not a special reset command. Do not confuse this wire format with the four-byte CFG/MAC writes above. |

The normal MAC-space FCE setup is `0x0800=1` (PSE), `0x09a0=0x400230`
(descriptor base), `0x09a4=1` (count), `0x09c4=0x44` (PDMA configuration),
and `0x0a6c=3` (skip FS). These are the [mt76 upload preamble][mt76-fce],
already present in local `fce_setup()`. Rewriting them is initialization,
not the dedicated CFG `0x0064[22:21]` reset above.

`MT_TX_CPU_FROM_FCE_CPU_DESC_IDX` is MAC `0x09a8`. The
[upload code][mt76-fw] increments it **after** a successful bulk transfer.
Neither inspected loader establishes that zeroing or advancing it can repair
a NAKing endpoint. A stalled downstream descriptor is plausible, but the
observations do not isolate it; overwriting the index could instead lose the
relationship between submitted data and descriptors. Prefer the sourced
IFDMA/FCE reset over inventing an index value.

## TXOP_HALT, EP_OUT_VALID and better diagnostics

The [vendor USB layout][vendor-usb] calls TX_BUSY and EP_OUT_VALID debug
status. EP_OUT_VALID describes endpoint data validity, not whether the USB
configuration exposes an endpoint. Zero during healthy idle is consequently
not surprising; zero with TX_BUSY set does not identify where the engine is
waiting. The same header names bit20 `WL_LPK_EN` for MT76xx while retaining
the older TXOP-countdown comment. The [older rt2x00 header][rt2800-regs]
describes TXOP_HALT as pausing the countdown when the TX buffer fills.
Neither supplies evidence that bit20 aborts or resets an active UDMA
transaction. Do not infer a recovery write from that generic mt76 name.

The vendor's [mt76x2_polling_txq_empty()][vendor-diagnostics] provides more
specific observations to record before and after each reset stage:

| Space / offset | Idle/empty condition |
|---|---|
| CFG `0x2240`, `0x2250`, `0x2260`, `0x2270`, `0x2280`, `0x2290` | Bit17 set for each OUT endpoint EP4–EP9 |
| CFG `0x9100` | `(value & 0x07f00000) == 0`, UDMA TX state idle |
| MAC `0x0a30` | `(value & 0x000000ff) == 0`, FCE TX1 empty |
| MAC `0x0a34` | `(value & 0x0000ff00) == 0`, FCE TX2 empty |

Capture these plus CFG `0x9018`, `0x9014`, `0x9080`, `0x0064` and MAC
`0x0400`, `0x09a8`. An idle status alone is insufficient: the subsequent
bounded bulk transfer and firmware/MCU exchange must also succeed.

## SET_CONFIGURATION and SET_INTERFACE

`libusb_set_configuration(handle, current_configuration)` requests a reset
of USB configuration state: alternate settings, endpoint halts and toggles.
Cancel/reap transfers, keep automatic kernel reattachment disabled, and
release claimed interfaces first; reclaim afterward. Discover the actual
configuration value rather than assuming 1. Use the library API so the host
stack tracks the change. The setup tuple is `(0x00, 0x09, configuration, 0, 0)`.
[libusb device-handling documentation][libusb-dev]

`libusb_set_interface_alt_setting(handle, interface, advertised_alt)` maps to
`(0x01, 0x0b, alt, interface, 0)` with the interface claimed and its transfers
quiesced. Do not invent alt 1. Linux allows a single-alt device to stall this
request and then performs a host-side fallback, so API success alone need not
mean a device-side reset occurred. [Linux usb_set_interface()][linux-interface]

Neither API promises to reset MediaTek's internal UDMA/FCE state. Moreover,
Linux's [usb_reset_and_verify_device()][linux-reset] already sends
SET_CONFIGURATION while restoring an existing configuration after a successful
port reset. If the reported `libusb_reset_device()` succeeded along that path,
this request has effectively already been exercised. A separate request is
still an order-dependent experiment, not a stronger reset or an established
solution. Confirm its presence in a USB trace if that distinction matters.

## Related-chip evidence and remaining limit

Linux [mt7601u firmware setup][mt7601-fw] disables FCE (`MAC 0x0800=0`)
before its vendor reset, then reinitializes FCE and pulses TX_CLR. Its USB DMA
register is **normal `0x0238`**, not MT7612U CFG `0x9018`. This supplies a
related-chip ordering precedent, not evidence that its unrelated magic
initialization values are safe or necessary on MT7662.

Linux rt2x00 defines TX_CLEAR bit19 at **normal `0x02a0`**. Neither
[rt2800usb.c][rt2800-usb] nor [rt2800lib.c][rt2800-lib] in v6.12 uses that
field. It adds no stronger hard-wedge recovery sequence than the dedicated
MT76x2U resets identified above.

The next discriminating experiment is the complete vendor sequence, with a
known hard-wedged starting state, no open-time USB reset hidden in the test,
and no power or enumeration transition. Record stage-by-stage registers and
control return lengths, then verify firmware upload, MCU reply, and ordinary
TX/RX against a live witness. Repeat on independently reproduced hard wedges
and include an unchanged recovery attempt as the failing control. If the full
sequence works, isolate its stages in later trials; do not attribute success
to bit6 alone when multiple resets were applied.

Until that measurement exists, **physical replug is the only reported working
hard-wedge recovery; it is not a proven silicon requirement**. The sources
show separate MAC/PBF, UDMA and IFDMA/FCE reset controls, which explains why
the previously attempted reset names do not cover every block. They do not
publish enough MT7662 USB-core internals to prove an unrecoverable latch,
specific deadlock, or mandatory power-on-reset condition. No driver change,
hardware test, or recovery claim is made by this report.

[vendor-reset]: https://github.com/caruofc/MT7612U-Driver/blob/749e6fd1a559e454b17ae91959fe07d6ef58e6af/common/rtmp_init.c#L4214-L4392
[vendor-version]: https://github.com/caruofc/MT7612U-Driver/blob/749e6fd1a559e454b17ae91959fe07d6ef58e6af/include/mt76x2_version.h#L19
[vendor-readme]: https://github.com/caruofc/MT7612U-Driver/blob/749e6fd1a559e454b17ae91959fe07d6ef58e6af/README.md
[vendor-caller]: https://github.com/caruofc/MT7612U-Driver/blob/749e6fd1a559e454b17ae91959fe07d6ef58e6af/common/rtmp_init_inf.c#L37-L57
[vendor-access]: https://github.com/caruofc/MT7612U-Driver/blob/749e6fd1a559e454b17ae91959fe07d6ef58e6af/common/rtusb_io.c#L377-L425
[vendor-usb]: https://github.com/caruofc/MT7612U-Driver/blob/749e6fd1a559e454b17ae91959fe07d6ef58e6af/include/mac_ral/mac_usb.h#L48-L117
[vendor-diagnostics]: https://github.com/caruofc/MT7612U-Driver/blob/749e6fd1a559e454b17ae91959fe07d6ef58e6af/chips/mt76x2.c#L1225-L1273
[mt76-access]: https://github.com/openwrt/mt76/blob/be5ce7910521492d4a2e4ce7ee3843680a46c047/usb.c#L104-L200
[mt76-single]: https://github.com/openwrt/mt76/blob/be5ce7910521492d4a2e4ce7ee3843680a46c047/usb.c#L226-L255
[mt76-fw]: https://github.com/openwrt/mt76/blob/be5ce7910521492d4a2e4ce7ee3843680a46c047/mt76x02_usb_mcu.c#L207-L251
[mt76-wmt]: https://github.com/openwrt/mt76/blob/be5ce7910521492d4a2e4ce7ee3843680a46c047/mt76x2/usb_mcu.c#L43-L58
[mt76-fce]: https://github.com/openwrt/mt76/blob/be5ce7910521492d4a2e4ce7ee3843680a46c047/mt76x2/usb_mcu.c#L94-L130
[mt7601-fw]: https://github.com/torvalds/linux/blob/v6.12/drivers/net/wireless/mediatek/mt7601u/mcu.c#L446-L478
[rt2800-regs]: https://github.com/torvalds/linux/blob/v6.12/drivers/net/wireless/ralink/rt2x00/rt2800.h#L527-L550
[rt2800-usb]: https://github.com/torvalds/linux/blob/v6.12/drivers/net/wireless/ralink/rt2x00/rt2800usb.c
[rt2800-lib]: https://github.com/torvalds/linux/blob/v6.12/drivers/net/wireless/ralink/rt2x00/rt2800lib.c
[libusb-dev]: https://libusb.sourceforge.io/api-1.0/group__libusb__dev.html
[linux-interface]: https://github.com/torvalds/linux/blob/v6.12/drivers/usb/core/message.c#L1512-L1665
[linux-reset]: https://github.com/torvalds/linux/blob/v6.12/drivers/usb/core/hub.c#L6159-L6197
