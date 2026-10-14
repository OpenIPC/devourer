# Hardware ACK responder on Kestrel (RTL8852BU)

How `SetAckResponder` arms the AX MAC's SIFS ACK on the 8852B, what was
measured to be needed, and what is still open. The bench numbers are the
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
- **Own macid.** The station sits at macid 127 / ADDR_CAM entry 127 (the last
  entry of the 0x7f search range), apart from the injection context at macid 0
  / entry 0 and from the low slots peer registrations take (`RegisterPeerSta`
  callers use 1, 2, ...; it and `RegisterBeamformee` now refuse 127). The
  firmware commits entry 127 in the same ~2.2 ms as entry 1 or 0. With the
  station at entry 1 during development, entry 0 read back byte-identical
  before the arm, after it and after the clear. Sharing entry 0 (the previous
  layout) had no measurable cost: a witness counted the same >= 2900 of 3000
  injected frames armed and unarmed, on both layouts. The separation is for
  the injection context's integrity, not a delivery fix.
- **Unexplained cell.** In about 55 armed cells across development (station at
  entry 1), one read the entry back committed (after 10 ms rather than ~2.2
  ms) and answered no ACKs; other slow commits were fine and it has not
  reproduced.
- **Clear needs the host.** Measured at entry 1: after `join_info(disconnect)`
  and a NO_LINK addr_cam, the entry stays VALID with the responder's address;
  a role REMOVE, or role CREATE plus NO_LINK, leaves it valid too. Clearing
  VALID from the host takes, and re-read for 2 s it stays clear. Because those
  H2Cs land asynchronously, the clear repeats until the entry has read back
  empty for 25 ms, and only that counts as disarmed. rxdemo's
  `DEVOURER_ACK_DISARM_AFTER_MS` exercises it live.
- **8852C:** its firmware accepts both commands and leaves the entry invalid,
  so the arm refuses. The vendor's 8852C association sends far more before the
  addr_cam (BB/RF offload, port configuration); which of it the C needs is open.
- Measured near field (two dongles, injector at -15 dB), single-shot, no CCX:
  the numbers are in `AdapterCaps.h`. The responder's own transmissions are
  not part of that matrix: with carrier sense off (this die's TX default) a
  responder that also sends unicast loses some ACKs to its own queued frames
  (the response scheduler abort counter at 0xCC18[23:16] moves).
