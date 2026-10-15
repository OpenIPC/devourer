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

## 8852C: bring-ups that answer no ACKs

Some 8852C bring-ups answer no ACK at all: the responder arms, the entry reads
back, and the adapter answers nothing for the whole session, where a good
session answers >= 97.5%. How often varies and no cause is established. On
the first bench it looked tied to uptime: over a day of back-to-back sessions
it rose from about one in ten to three in four (10 of 83 overall), and after a
long power-off the first 12 sessions answered 97.5-99.4% before 3 of the next
12 came up dead. A second bench (two TX50UH, an 8812CU soliciting) does not
support that reading: dead sessions came interleaved with good ones
(dead/good/dead/good/dead in one series), one came on the second session after
a 3 s VBUS cut, a VBUS cold cycle before every session still left 1 of 13
dead, and the RF thermal meter sat at delta 2-3 throughout. Back on the first
bench, with the entry at hardware entry 63, 6 of 15 sessions were dead, in
runs (3 dead, 3 good, 2 dead, 4 good). It is a property of the bring-up, not
of the arm: in one failed session three
separate injector processes in a row all saw 0, and no session has ever
recovered or failed partway. Measured in failed sessions:

- Every frame addressed to the responder still matches the entry in the RX
  descriptor (ADDR_CAM_VLD, macid 63), and the adapter airs its own frames
  normally: only the SIFS response is missing.
- The response abort counters (0xCC14 NAV, 0xCC18 CCA/EDCCA/scheduler) stay at
  0: the hardware does not attempt the response and abort it.
- Every MAC configuration register read (0xC000, 0xC300, 0xC400, 0xC600,
  0xC800, 0xCA00, 0xCC00, 0xCE00 and the BB path-com block at 0xD800) is
  identical to a good session's; only counters differ.
- What does differ: `R_AX_WMAC_NAV_UP_INFO` (0xCC84) records intra-BSS NAV
  updates and `R_AX_PS_RXINFO` (0xCEA0) flags port-0 data, i.e. the MAC
  classifies the soliciting frames as its own BSS's traffic, which it does not
  in a good session.
- Ruled out by measurement: the response NAV checks (`R_AX_RSP_CHK_SIG` bits
  19-21 cleared, still fails), the BSSID CAM (all zero in good and failed
  sessions alike; writing entry 1 valid, still fails), the injector, the
  host's per-frame RX cost, and the arm's timing relative to the RX loop.
  The slot is not ruled out: development tested hardware entry 126 only, and
  on a second bench entry 126 gave more dead sessions than entry 63 (8 of 21
  against 1 of 16), but entry 63 gave 6 of 15 on the first, so the slot does
  not explain it.

What decides it is not found. The 8852B shows the same shape far more rarely
(about one bring-up in 55). Because a dead session reads back like a good one,
`GetAdapterCaps` reports `ack_responder_ok` false on the 8852C (true on the
8852B), while `SetAckResponder` still arms it. A caller that needs the ACK on
the 8852C should treat a soliciting peer's retries as the tell and bring the
adapter up again.
