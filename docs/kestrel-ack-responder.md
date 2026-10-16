# Hardware ACK responder on Kestrel (RTL8852BU, RTL8832CU)

How `SetAckResponder` arms the AX MAC's SIFS ACK on both Kestrel dies, what
was measured to be needed, and what is still open. The bench numbers are the
`ack_responder_ok` row in `src/AdapterCaps.h`.

## Evidence behind the mechanism

What `SetAckResponder` / `ClearAckResponder` do and verify is documented once,
on their declaration in `src/kestrel/RtlKestrelDevice.h`. This page keeps what
was measured to get there.

- **Why it used to measure 0%.** `add_self_sta`'s NO_LINK entry is accepted
  over H2C and never committed: the physical entry, read through
  `FILTER_MODEL_ADDR`/`INDIR_ACCESS_ENTRY` at 0x18850000, stays valid=0.
- **What commits it** was captured from the vendor Windows driver over
  USBPcap: `join_info(INFRA, connect)` THEN the station addr_cam (SHORT
  entry, BSSID CAM at index 1). Without the join the entry never commits. The
  vendor's surrounding commands (role CLIENT, macid unpause, CMAC/DMAC tables,
  H2C REC/DONE_ACK) were each dropped with no change in commit or ACK rate,
  so `register_infra_sta` sends only the two.
- **The commit is asynchronous.** Typically read back ~2.2 ms after the H2Cs
  (a single read takes about that long over USB), once 19.5 ms. A one-shot
  read-back refused healthy arms, so `SetAckResponder` polls for up to 100 ms.
- **Response-abort gates.** The arm clears `R_AX_TRXPTCL_RESP_0` bits 23-28.
  On ch149, shared with a neighbour AP, one of three runs without the clear
  dropped to 82.4% single-shot; three with it stayed at >= 99.6%. On a quiet
  ch161 it made no difference.
- **Own slot.** The slot itself is documented at `ack_slot()`. Measured: the
  8852B firmware commits entry 127 in the same ~2.2 ms as entry 1 or 0. The
  CAM has 128 entries on both dies, laid out at a per-die stride (the vendor's
  get_addr_cam_size): 0x40 on the 8852B, 0x20 on the 8852C. An earlier
  revision used the 8852B's stride on both, so its 8852C "entry 63" was
  hardware entry 126 and the 0xdeadbeef it read "from entry 64 up" was the
  end of the 128-entry CAM. Dword 5 of an 8852C entry reads 0xdeadbeef
  whatever is written (the other seven read back as written), so the host's
  read-back skips it. With the station at entry 1 during development, entry 0 read back
  byte-identical before the arm, after it and after the clear. Sharing entry 0
  (the previous layout) had no measurable cost: a witness counted the same >=
  2900 of 3000 injected frames armed and unarmed, on both layouts.
- **Unexplained cell.** In about 55 armed cells across development (station at
  entry 1), one read the entry back committed (after 10 ms rather than ~2.2
  ms) and answered no ACKs; other slow commits were fine and it has not
  reproduced. It has the shape of the 8852C bring-ups described below, much
  rarer.
- **Clear needs the host.** Measured at entry 1: after `join_info(disconnect)`
  and a NO_LINK addr_cam, the entry stays VALID with the responder's address;
  a role REMOVE, or role CREATE plus NO_LINK, leaves it valid too. Clearing
  VALID from the host takes, and re-read for 2 s it stays clear. Because those
  H2Cs land asynchronously, the clear repeats until the entry has read back
  empty for 50 ms, and only that counts as disarmed. rxdemo's
  `DEVOURER_ACK_DISARM_AFTER_MS` exercises it live.
- **8852C:** see the next two sections.
- Measured near field (two dongles, injector at -15 dB), single-shot, no CCX:
  the numbers are in `AdapterCaps.h`. The responder's own transmissions are
  not part of that matrix: with carrier sense off (this die's TX default) a
  responder that also sends unicast loses some ACKs to its own queued frames
  (the response scheduler abort counter at 0xCC18[23:16] moves).

## 8852C: the host writes the entry

The 8852C firmware ignores the addr_cam H2C altogether. After it, neither the
address CAM entry nor the BSSID CAM entry it carries is written: the address
CAM still holds the uninitialised SRAM it powered up with, and the BSSID CAM
stays zero. Nothing changed that, including a byte-for-byte replay of the
vendor Windows driver's four association H2Cs (CMAC table, DMAC table, a
12-byte join_info, addr_cam), a 12-byte join_info alone, and a 64-byte
addr_cam (the vendor Linux `struct fwcmd_addrcam_info`). The same replay
commits on the 8852B. Both dies report non-secure (`is_sec_ic=0`), and the two
ways of reading the window (per-dword address, or the vendor's page base plus
offset) agree, so the read-back itself is not the issue.

The RMAC still answers from the CAM, so on this die the arm writes the entry
from the host instead, the vendor's no-firmware path (`mac_upd_addr_cam`'s
indirect SRAM write); what it writes and how is at
`HalKestrel::write_addr_cam_sta`, and the entry encoding is shared with the
8852B's H2C (`KestrelAddrCam.h`, pinned by a selftest against hardware
read-backs). Measured on its own:

- **Only the address CAM entry matters.** Leaving out the join_info H2C, the
  addr_cam H2C or the BSSID CAM entry, alone or all together, kept the
  single-shot rate at 97.6-99.4%. Host-writing the BSSID CAM entry the vendor
  path also writes changed nothing measurable.
- **The response-abort gates are mandatory here.** This die's EDCCA reads its
  own floor as busy (-67 dBm, `busy=1` at every tune), and with the gates
  armed the RMAC dropped every ACK: 0/2060 and 0/2019, against >= 97.6% with
  them cleared.
- **The lookup hits the entry.** The long RX descriptor (dword 5:
  ADDR_CAM_VLD, addr_cam index, macid) shows every frame addressed to the
  responder matching the host-written entry (macid 63). That was measured on
  the earlier layout, where the index read 126: the entry the 0x40-stride
  write had landed in, counted at the 8852C's 0x20 stride.
- **Clear** reads back empty and the soliciting peer then gets no ACK. On
  this die a 0 after a disarm only counts when the same session answered
  before it (a dead bring-up also reads 0), so the live-disarm row in
  `AdapterCaps.h` solicits across the disarm.

## 8852C: bring-ups that answered no ACKs

**In short.** Some 8852C bring-ups armed, read the entry back, and answered no
ACK for the whole session. The chip did build the ACK, but every frame also
carries a selection of which transmit paths to use, and the selection the MAC
attached to the ACK was garbage in those bring-ups: it enabled a path this
2T2R die does not have, or mapped path A off the signal, so no antenna sent the
ACK. That selection comes from MAC state nothing initialises, which is why it
changed from one bring-up to the next. The fix makes the baseband take the
path selection from its own registers (P-MAC) instead of from the MAC.

**How it was found.** The TX-info debug port (`R_AX_WMAC_TX_INFO_DEBUG_SEL`
0xCAE4 select 0, read at 0xCAE8 with `R_AX_TCR1` bit 26 set) returns the TX
info the MAC hands the baseband, in the baseband's `INTF_TXINFO_*` layout
(`halbb_cr_info_8852c.h`, 0x1800): PATH_EN is bits [15:12], PATH_MAP_A..D bits
[23:16]. It reads 0 until the first response and then holds one value for the
session:

| TX-info word | paths enabled | path A mapped to | sessions without the fix |
|---|---|---|---|
| 0x10e41012 | A | 0 | all answered |
| 0x10??f012, map A = 3 | A, B, C, D | 3 | all answered |
| 0x10??f012 / 0x10??b012 / 0x10??7012, map A = 1 | several | 1 | 10 of 10 dead |
| 0x10914012 (second bench) | C only | 1 | dead |

The selection is not taken from the per-macid CMAC table (searched for in all
128 entries, every session), the MAC TX power tables (a good session's tables
written before every arm still gave dead sessions), the firmware version (the
Windows driver's 0.29.123.4 gave dead sessions too), the slot, temperature, or
the 5 GHz EDCCA level (#491). Where the MAC takes it from is still not found.
The two experiments #490 proposed were run after the cause was found:

- **Firmware RAM is not it.** The firmware's data memory reads back through
  the indirect window at the CPU's own addresses (0x20100000 up: the code
  section reads back byte-identical to the image). Its writable parts,
  0x20161000-0x20180000, 0x2020c000-0x20278000 and 0x20300000-0x20308000
  (about 590 KB), were dumped 8 s after the arm in 26 sessions (2026-10-15,
  21:18-21:49, UTC-3) and grouped by the TX-info word (that build had the
  fix, so every session answered): 15 sessions with 0x10e41012 against 11
  with the 0x10??f012 family. No bit is constant within each group and
  different between them (the same test with shuffled labels also finds
  none), and the 160 words the firmware keeps with only 2-4 values across
  sessions take each value in sessions of both groups. Only one session had
  the map-A-1 word, and it shares the common values. The path selection is
  not something the firmware's memory records.
- **The vendor driver does not show it.** The same TX50UH on the Realtek
  Windows driver (5001.19.128.1, firmware 0.29.123.4), associated as a
  station to an AP on ch161, was restarted 45 times (`pnputil
  /restart-device`, a full driver bring-up each time; 2026-10-16
  00:15-00:33) and solicited for 8 s after each reconnect by the TX20U Nano
  (MCS1, one attempt per frame): 0 dead of 45, every one 88.6-98.3%
  (associated, it also carries the AP's traffic). Without the fix, devourer
  on the same unit had 4 dead of 30 earlier that day. So the vendor
  bring-up initialises the response's path selection where devourer's does
  not; which write does it is still not found.

The attribution rests on the TX-info word predicting every outcome seen
(the map-A-1 selection dead 10 of 10 times, every other value alive) and on
the fix removing the dead sessions, including those with that selection.

**The fix.** Right after the T-MAC path routing at bring-up, the halbb glue
calls `halbb_ctrl_tx_path_pmac_8852c(RF_PATH_AB)`: 0x09a4 bits [4:2] select
P-MAC as the source of the path enable and the path A/B maps, set to paths A+B
with A on IFFT 0 and B on IFFT 1, plus the vendor's per-path TSSI reset pulse.
It applies to every PPDU, not only responses.

**Measured.** This bench: TX50UH responder, TX20U Nano soliciting, ch161,
MCS1, 8 s per session, all on 2026-10-15 (local time, UTC-3), since the dead
rate drifts through a day (#490):

| when | build | dead sessions |
|---|---|---|
| 10:12-10:19 | without the fix | 6 of 15 |
| 12:51-14:45 | without the fix | 3 of 21, 3 of 15, 2 of 25, 1 of 31, 1 of 12 |
| 14:47-16:04 | with the fix (register-level, then the PoC) | 0 of 93 |
| 16:15-16:31 | without the fix (master with #491) | 4 of 30 |
| 17:27-17:43 | with the fix (this commit) | 0 of 30 |

With the fix, 15 sessions had the map-A-1 selection; every session answered
96.8-100% and the selection registers held the P-MAC values at the end of every
session. The 4-of-30 series without the fix ran between series with it, so the
difference is not the time of day. An earlier day's matrix had 3 of 12 dead
(time not recorded). Second bench (two TX50UH, an 8812CU soliciting): up to
about 85% dead in the afternoon, then 0 dead of 20 on ch6 and ch161, both
units, an hour later with the fix. Injection from the 8852C, compared with the
same build without the change: unchanged for one stream (MCS1 and 6M
~1900/2000 to the 8852B on ch6 and on ch161 with carrier sense on). That is a
before/after check of this change only, not an absolute 5 GHz TX figure; the
vendor-driver injection A/B that #490 asks for before trusting 5 GHz Kestrel TX
numbers was not run. Two-stream injection from the 8852C does not decode with or without
the fix (MCS12, VHT2SS_MCS1: 0/2000, while the 8852B's are received by the
8852C), so what the fix does to two-stream TX is not measured.

The 8852B showed the same shape far more rarely (about one bring-up in 55, not
reproduced since); it routes TX paths per station through its CMAC antenna
model and is not changed.
