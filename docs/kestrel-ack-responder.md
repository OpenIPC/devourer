# Hardware ACK responder on Kestrel (RTL8852BU)

How `SetAckResponder` arms the AX MAC's SIFS ACK on the 8852B, what was
measured to be needed, and what is still open. The bench numbers are the
`ack_responder_ok` row in `src/AdapterCaps.h`.

## Mechanism

The AX MAC has no Jaguar-style port identity register to retarget: the RMAC
answers for an address only when the firmware has COMMITTED a valid
ADDR_CAM entry carrying it as SMA. `add_self_sta`'s NO_LINK entry is accepted
over H2C and never committed (the physical entry, read through
`FILTER_MODEL_ADDR`/`INDIR_ACCESS_ENTRY` at 0x18850000, stays valid=0), which
is why the responder used to measure 0%. What commits it is registering a
station the way a real association does (captured from the vendor Windows
driver over USBPcap): `join_info(INFRA, connect)` THEN the station addr_cam
(SMA = responder address, TMA = a BSSID, SHORT entry, BSSID CAM at index 1).
The join is the necessary step: without it the entry never commits. The
vendor's surrounding commands (role CLIENT, macid unpause, CMAC/DMAC tables,
H2C REC/DONE_ACK) were each dropped with no change in commit or ACK rate, so
`register_infra_sta` sends only the two.

- **The commit is asynchronous.** Typically read back ~2.2 ms after the H2Cs
  (a single read takes about that long over USB), once 19.5 ms. A one-shot
  read-back refused healthy arms, so `SetAckResponder` polls for up to 100 ms.
- **Response-abort gates.** The arm clears `R_AX_TRXPTCL_RESP_0` bits 23-28.
  On ch149, shared with a neighbour AP, one of three runs without the clear
  dropped to 82.4% single-shot; three with it stayed at >= 99.6%. On a quiet
  ch161 it made no difference.
- **Clear** re-registers the bring-up NO_LINK entry (which does not commit)
  and restores the gates; the read-back confirms the CAM no longer holds the
  address. rxdemo's `DEVOURER_ACK_DISARM_AFTER_MS` exercises it.
- **8852C:** its firmware accepts both commands and leaves the entry invalid,
  so the arm refuses. The vendor's 8852C association sends far more before the
  addr_cam (BB/RF offload, port configuration); which of it the C needs is open.
- Measured near field (two dongles, injector at -15 dB), single-shot, no CCX:
  the numbers are in `AdapterCaps.h`. The responder's own transmissions are
  not part of that matrix: with carrier sense off (this die's TX default) a
  responder that also sends unicast loses some ACKs to its own queued frames
  (the response scheduler abort counter at 0xCC18[23:16] moves).
