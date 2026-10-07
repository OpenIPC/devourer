# MT7612U transmit retries, measured off the chip

The MT7612U retransmits an unacknowledged, ACK-requested frame autonomously.
The depth of that ladder can be derived by arithmetic - the initvals leave
`MT_TX_RETRY_CFG` at a short limit of 15 (frames over the 2032-byte long
threshold use the long limit, 31), CWmin 15 / CWmax 1023 from
`MT_WMM_CWMIN/CWMAX`, a 9 µs slot, ~46 ms against ~45.5 ms measured per
unanswered unicast frame - but that is an inference. The MAC reports the
number itself.

`MT_TX_STAT_FIFO` (0x1718) and `MT_TX_STAT_FIFO_EXT` (0x1798), declared in
`src/mt7612u/regs.h`, hold one entry per MPDU - success, ACK-requested, and
the retry count in the EXT half - for every frame whose txwi `pktid` is
non-zero (`mt76x02_mac_load_tx_status`). `MT_TXOPT_TXS` sets that `pktid`,
and `MT_TXOPT_PKTID(id)` chooses it. The library's normal send path leaves it
0 and files no status: an undrained status FIFO is traffic nobody wants on a
path that must stay free of per-frame register I/O.

## The retry-limit knob

`DEVOURER_TX_RETRY_LIMIT` / `DeviceConfig::tx.retry_limit` means the same on
every chip: the number of hardware retries an ACK-requested frame gets. On
the MT7612U, `Mt7612uRadio` programs it at every bring-up through
`mt7612u_set_retry_limit()`, which writes the value into BOTH the short and
the long limit of the global `MT_TX_RETRY_CFG` - so a frame over 2032 bytes
gets the same limit as a short one - as a checked read-modify-write with a
readback. A failure is fatal, so a session never airs with a retry depth it
did not ask for. The range is at the field declaration in
`src/DeviceConfig.h`.

The library default is 0, so a default MT7612U session airs ACK-requested
frames with **0 retries**, not the initvals' 15 (31 over 2032 bytes). The
default stream radiotap is NOACK (below), which never retries whatever the
limit says, so that is what a default session sends anyway. A unicast link
that relies on MAC retransmission sets a nonzero limit and requests ACKs.

## The reproducer: `mt7612uprobe txs`

`mt7612uprobe txs [chan] [frames/arm] [peer MAC]` (the bring-up tool,
`src/mt7612u/tools/bringup.cpp`, built by CMake as `mt7612uprobe`). HT MCS7,
20 MHz, 1400-byte QoS data (so the short limit applies), eight arms, each run
with the MAC receiver off and then on:

| arm | addr1 | addr2 | ack request | WCID | other |
|---|---|---|---|---|---|
| a | broadcast | static source | No Ack | 0xff | |
| b | peer | static source | Normal | 0xff | |
| c | peer | static source | No Ack | 0xff | |
| d | peer | port's own address | Normal | 0xff | |
| e | peer | port's own address | No Ack | 1 | |
| f | peer | port's own address | No Ack | 0xff | MGMT queue |
| g | peer | port's own address | No Ack | 0xff | A-MPDU, MGMT queue |
| h | broadcast | static source | No Ack | 1 | |

How it reads the status FIFO:

- **EXT first, then the main word**, as mt76 does: reading `MT_TX_STAT_FIFO`
  pops the entry, so the EXT half must be read before it.
- **One pktid per arm** (3..18, inside mt76's 3..127 skb range), counting only
  entries that echo the current arm's pktid. Entries carrying the previous
  arm's pktid are reported as late, any other as foreign - so late status from
  an unsettled arm never lands in the next arm's columns.
- **Paced to the chip, then to the status.** A submit only queues the USB
  transfer, so each frame first waits up to 1.5 s for that transfer to
  complete - one that fails within that wait is resent and not counted as
  sent, and the arm says how many did; one still in flight then counts as
  sent, since a TX transfer has no timeout and a resend would air it twice
  (should it fail later, the failure is charged to the next frame) - then
  for its own
  status entry, bounded by the ladder at the
  effective limit and never less than 2 s; an expired wait counts as a
  per-frame status timeout. A final settle loop collects anything still
  owed. `DEVOURER_TXS_TRACE=1` logs every submit, expired wait and popped
  entry with its raw words (`TXS` lines; the table is unchanged).
- On receiver-ON passes the 1 Hz PHY tick runs from those waits, and WCID 1
  must read back as installed or the gate fails.

`DEVOURER_TX_RETRY_LIMIT=N` programs the limit with the same setter
`Mt7612uRadio` uses. **Unset, the gate does not program the register**: it
runs the initvals (short 15 / long 31), which is not what a library session
runs. It prints the `MT_TX_RETRY_CFG` word in both cases. Exit: 0 reported,
1 device failure or no status filed, 2 bad argument, 3 interrupted.

```sh
sudo DEVOURER_TX_RETRY_LIMIT=5 build/mt7612uprobe txs 36   # Normal arms: mean retry 6.0
sudo DEVOURER_TX_RETRY_LIMIT=0 build/mt7612uprobe txs 36   # the library default: mean retry 1.0
sudo build/mt7612uprobe txs 36                             # initvals: mean retry 16.0
```

With no ACK responder armed, every Normal arm is unacknowledged and runs its
ladder to exhaustion. With one armed for `peer` on the same channel (e.g.
`DEVOURER_ACK_RESPONDER=<peer> DEVOURER_CHANNEL=<chan> rxdemo` on a Realtek
adapter), the receiver-ON pass reports "ACKs to our TAs" against both
addresses the arms send from.

## Result

`mt7612uprobe txs 36` with the gate as it is in this tree, one MT7612U, near
field, one run per setting, 40 frames per arm, no peer ACK responder, 0
submit failures, WCID 1 read back as installed on every pass. The gate's
printed register word confirms each setting, including `47f01f0f` for the
initvals.

Columns: own-pktid entries / sent, successes, mean retry, max retry, fps,
then per-frame status timeouts `T`, late entries from the previous arm `L`,
foreign entries `F`; `U` = UNSETTLED (fewer entries than frames sent, or
an arm cut short before it sent all of its frames). fps includes each frame's wait for its
own status entry: on a clean row it is per-frame submit-to-status time, on a
row with timeouts it is the wait bound, not a rate.

```
  DEVOURER_TX_RETRY_LIMIT=0 - the library default   (MT_TX_RETRY_CFG 47f00000)
       receiver OFF                          receiver ON
  a    39/40  39  0.0  0    5 U  T40 F1      39/40  39  0.0  0    5 U  T40 L1
  b    40/40   0  1.0  1   47                40/40   0  1.0  1   42
  c    40/40  40  0.0  0   46                40/40  40  0.0  0   44
  d    40/40   0  1.0  1   45                40/40   0  1.0  1   45
  e    40/40  40  0.0  0   48                40/40  40  0.0  0   45
  f    40/40  40  0.0  0   47                39/40  39  0.0  0    5 U  T40 L1
  g    40/40  40  0.0  0   47                40/40  40  0.0  0   42
  h    39/40  39  0.0  0    5 U  T40 L1      39/40  39  0.0  0    5 U  T40 L1
  receiver saw 21011 frames, 0 ACKs to our TAs.

  DEVOURER_TX_RETRY_LIMIT=5   (MT_TX_RETRY_CFG 47f00505)
       receiver OFF                          receiver ON
  a    39/40  39  0.0  0    5 U  T40 F1      39/40  39  0.0  0    5 U  T40 L1
  b    40/40   0  6.0  6    8    T40         39/40   0  6.0  6    5 U  T40 L1
  c    40/40  40  0.0  0   46                39/40  39  0.0  0    5 U  T40 L1
  d    39/40   0  6.0  6    5 U  T40 L1      40/40   0  6.0  6    8    T40
  e    39/40  39  0.0  0    5 U  T40 L1      40/40  40  0.0  0   47
  f    39/40  39  0.0  0    5 U  T40 L1      39/40  39  0.0  0    5 U  T40 L1
  g    40/40  40  0.0  0   47                39/40  39  0.0  0    5 U  T40 L1
  h    39/40  39  0.0  0    5 U  T40 L1      39/40  39  0.0  0    5 U  T40 L1
  receiver saw 39870 frames, 0 ACKs to our TAs.

  DEVOURER_TX_RETRY_LIMIT unset - initvals, short 15 / long 31, not
  programmed by the gate   (MT_TX_RETRY_CFG 47f01f0f)
       receiver OFF                          receiver ON
  a    39/40  39  0.0  0    5 U  T40 F1      39/40  39  0.0  0    5 U  T40 L1
  b    40/40   0 16.0 16    5    T40         40/40   0 16.0 16    5    T40
  c    39/40  39  0.0  0    5 U  T40 L1      40/40  40  0.0  0   46
  d    40/40   0 16.0 16    5    T40         39/40   0 16.0 16    4 U  T40 L1
  e    40/40  40  0.0  0   48                39/40  39  0.0  0    5 U  T40 L1
  f    40/40  40  0.0  0   46                40/40  40  0.0  0   35
  g    39/40  39  0.0  0    5 U  T40 L1      40/40  40  0.0  0   35
  h    39/40  39  0.0  0    5 U  T40 L1      40/40  40  0.0  0  503
  receiver saw 25743 frames, 0 ACKs to our TAs.
```

A default `Mt7612uRadio` session programs the same register: `txdemo` on
this adapter logs `MT7612U: hardware retry limit 0` with no retry setting and
`... 5` with `DEVOURER_TX_RETRY_LIMIT=5`, each after the setter's readback
passed.

What it shows:

- **The knob reaches the retry engine.** Unacknowledged Normal arms b/d read
  mean 1.0 / max 1 at limit 0, 6.0 / 6 at limit 5 and 16.0 / 16 on the
  initvals, 0 successes, in both receiver states: the limit plus the first
  attempt. At the library default an unacknowledged frame is sent once.
- **No-Ack frames are not retried.** Every No-Ack arm reads 0.0 mean / max
  0, in both receiver states, at every setting, clean or lagging. The no-ack
  request reaches the retry engine: the MAC marks these frames done on the
  first attempt.
- **At limit 0 the Normal arms settle cleanly** - 40/40, no timeouts, 42-47
  fps, the same per-frame cost as the No-Ack rows. At limit 5 and on the
  initvals they time out on every per-frame wait (T40), and three of the
  eight such rows are 39/40 with the lag below.
- **A one-step status lag - a stale EXT read on the arm's first entry.** In
  some arms EVERY per-frame wait times out (T40) and the arm ends 39/40, with
  one late entry (or, for the very first arm, one foreign entry) in that
  SAME arm's row. Counting 39/40 rows, it hit five of sixteen at limit 0,
  eleven at limit 5 and seven on the initvals - one run each. Arm a lags in
  every pass; which other arms lag varies from pass to pass. This table
  seemed to rule out "status posted only on the next TX" (at limit 0,
  receiver ON, arm f lags with L1 although arm e before it settled 40/40 and
  owed nothing, and arm g after it shows no late entry) - a later trace
  shows exactly that for some arms; see "Status posted on the next
  submit" below. What fits THIS table's first-entry case is the two-transfer
  read:
  when the FIFO is empty at the EXT read and an entry is filed before the
  main read, the popped entry is paired with the stale EXT word of the
  previous entry. On an arm's first entry that is the previous arm's pktid,
  so the entry was counted late, the arm stayed one short, and every
  per-frame wait timed out. A race on the poll timing, which is why it
  varies from pass to pass and with the host. The gate now claims that
  entry back (`txs_drain`), under all of: it is the arm's first popped entry
  (no own entry yet), the arm has submitted a frame, and its pktid is that
  of the last arm that sent a frame, which must have settled with no entry
  owed - or, on the session's first arm, any pktid. A claimed entry counts
  in entries, and in success when its SUCCESS bit is set; it stays out of
  the retry columns, and is reported as "stale-EXT entries claimed". These
  tables were taken before that change. With the claim keyed to the
  previous arm, the author's unit then settled 16/16 arms at limits 5 and 0
  (recorded on issue #461); keying it to the last arm that sent a frame,
  so an arm with every submit failed is skipped, has since been run on
  hardware (#467). If the arm's own entries still reach `sent`, the claimed
  entry was not its own (one arm read 61/60 with one claimed), and the gate
  hands it back to the late/foreign column at arm end. The settled condition
  cascades: one UNSETTLED arm disables the claim for every following arm
  until an arm settles on its own, so on a unit whose arms do not settle the
  claim buys almost nothing.
- **Arms e-h**: e, f and g read like c whenever they are clean; nothing
  distinguishes them. h (broadcast, WCID 1) lagged in five of six passes.

## Status posted on the next submit (#461)

`tests/mt7612u_sta_uplink.sh` at ch6, retry limit 15, the RTL8812CU peer
answering for the DUT's target in its arm A and for another address in arm
B, with `DEVOURER_TXS_TRACE=1`: one run at 60 frames per arm and two at 20
(the second pair differing only in the data-frame QSEL, below) - six harness
arms, each running all sixteen gate arms.

**The gate used to lose frames, not status.** Before the gate waited for
each transfer, its per-frame wait (60 + 50 ms) was far shorter than an
unacknowledged frame's ~1.1 s on this channel, so it submitted ahead of the
air: the chip's queue filled, its bulk OUT stopped accepting, and the async
ring's 1000 ms transfer timeout (since removed: TX transfers now carry
none, as mt76's do not) cancelled 9-50 of every 60 frames per unicast arm.
Those were counted as sent; the surviving backlog filed its
status seconds later in later arms' rows, as late or foreign. With the gate
pacing on the transfer, no transfer failed in any of the three runs.

**Some arms then file each frame's status only when the NEXT frame is
submitted.** The trace shows it frame by frame: SUBMIT, the whole 2 s wait
expires with nothing, and the entry pops within ~1 ms of the next frame's
bulk transfer - so every wait times out, and the arm's last entry lands in
the next arm as late (N-1/N, `late=1`). Which arms, across the six harness
arms (rx0/rx1 = receiver off/on):

| arm | rx0 | rx1 |
|---|---|---|
| a broadcast, No Ack | 6/6 | 6/6 |
| h broadcast, wcid 1 | 6/6 | 6/6 |
| e, f, g unicast No Ack (ownSA) | 0/6 | in all 3 peer-answering arms (A), one to three of e/f/g each; 0/3 in B |
| b unicast Normal (static SA, never ACKed) | 1/6 | 1/6 |
| c unicast No Ack; d unicast Normal (ownSA) | 0/6 | 0/6 |

So broadcast holds every time, in both receiver states; unicast No-Ack holds
only with the receiver on and a peer answering on the channel. Why the MAC
holds the entry is not established. The 2 s floor does not cause it - no wait
helps when the entry needs the next submit - but it makes a held arm cost
2 s a frame, which is what makes the harness slow (35 min at 60 frames).

**The data-frame queue is not it.** `MT_QSEL_EDCA` here is 1, which is
mt76's HCCA value (`enum mt76_qsel`: MGMT 0, HCCA 1, EDCA 2). The two 20-frame
runs aired data at QSEL 1 and at QSEL 2 from one build and read the same:
the same arms held and the same ~1.1 s unacknowledged retire. The define is
left as it is.

## Radiotap NOACK disables the retry

`src/mt7612u/radiotap.cpp` maps radiotap TX_FLAGS NOACK to `rate->no_ack`, and
`tx.cpp` then leaves the TXWI `ACK_CTL_REQ` clear, so the MAC completes the
frame on its first attempt and the ladder above never runs, whatever the
retry limit says. `build_stream_radiotap(mode)` sets NOACK - correct for the
broadcast FPV downlink, where nothing ACKs.
`build_stream_radiotap(mode, /*no_ack=*/false)` builds the same header with
NOACK clear, for unicast to a peer that ACKs. Group-addressed frames must keep
NOACK. Only the MT7612U reads the bit (`src/RadiotapBuilder.h`).

What the difference is worth, measured with the MT7612U sending unicast data
to an RTL8812CU peer operating as an AP that ACKs (ch36, 6M, 2 Mbit/s, 30 s,
one run per arm; losses counted per frame by sequence or CCMP PN, and checked
against an independent monitor witness). **This table is not reproducible
from this tree alone**: it was taken with an associated-client harness and an
AP-side per-frame ledger that are not part of it. `mt7612uprobe txs` covers the
mechanism (No-Ack arms settle at 0 retries, Normal arms retry), not this
end-to-end delivery figure.

| arm | frames sent | lost at the peer | of those seen on air | not seen on air |
|---|---|---|---|---|
| vendor rtl88x2cu AP (open), MT7612U sending NOACK | 5358 | 20 | 1 | 19 |
| devourer AP (WPA2), MT7612U sending NOACK | 5386 | 55 | 22 | 33 |
| devourer AP (WPA2), **MT7612U requesting ACKs** | 5386 | **0** | 0 | 0 |

With ACKs requested, 63 retry-bit frames and 33 repeated PNs appear on the
air, and every frame arrives. The limits on that: one run per arm, one channel,
near field, one DUT and one peer. The frames "not seen on air" in the NOACK
arms are not explained - the witness's own miss rate was 0.5-1.7% - but they
vanish with retries on. The vendor-vs-devourer difference among the frames
that were on the air (1 vs 22) is one run each and was not pursued.

A witness seeing zero retries from the MT7612U proves nothing about the peer's
ACKs while NOACK is set - there are never going to be retries.

## The open question

Why does a unicast frame that the MAC completes successfully on the first
attempt, with zero retries, still cost ~20 ms, when the same frame addressed to
broadcast costs ~0.3 ms? The gate measures per-frame submit-to-status time
directly: the clean No-Ack unicast rows (c, e, f, g) read 35-48 fps, ~21-29 ms
a frame - consistent with ~20 ms - and at limit 0 the unacknowledged Normal
rows cost the same (42-47 fps), so a single attempt, ACK-requested or not,
is where the time goes. Against it, the broadcast side is thin: one
broadcast row settled clean (h, receiver ON, initvals) at 503 fps, ~2 ms a
frame including the gate's status polling - about 10x the unicast rows,
against the ~40x of the unpaced bisect in `docs/mt7612u.md` ("Unicast
injection is a 40x cliff"); every other broadcast row lagged. One row, one
run. Arm g (A-MPDU on the MGMT queue) reads the same as c/e/f.

Candidates not yet separated: a per-WCID queue serialization with the
no-station index `0xff`, TXOP/EDCA admission for a unicast RA, or a USB
completion path that only retires these transfers lazily. `MT_TX_STAT_FIFO`
answers "how many attempts"; it does not answer "how long did each take".

It is the difference between `docs/mt7612u.md`'s "address one-way links to
broadcast or multicast" guidance and a usable one-way *unicast* link, which is
why it is worth an issue of its own.

## Open: ~1.1 s per unacknowledged frame on ch6

On ch6 an unacknowledged frame at limit 15 retires in 1.1-1.4 s (arms b and d
in arm B, b in both; the status reads 16 retries at OFDM 6M, rate word
0x2000), against ~46 ms on ch36 above - ~70 ms an attempt where the EDCA
arithmetic gives ~3-5 ms. Not the queue (above). What the registers say,
decoded against mt76's field layout, none of it decisive:

- `MT_TX_RTS_CFG` 0x00ffff20: RTS retry limit 32, threshold 0xffff.
- `MT_CCK_PROT_CFG` 0x07f40003 / `MT_OFDM_PROT_CFG` 0x07f42004: CTRL 0 (no
  protection), RTS-threshold enable set - so no RTS at 1400 bytes.
- `MT_MM20/GF20_PROT_CFG` 0x01752004, `MM40/GF40` 0x03f52084: CTRL 1, RTS/CTS
  FORCED for HT frames, at OFDM 24M. The first attempts at HT MCS7 would
  each be an unanswered RTS, retried to 32 - but the same initvals ran on
  ch36, and the final attempts (OFDM 6M) fall under OFDM_PROT, which is off.
- `MT_TX_TIMEOUT_CFG` 0x000a2290: ACK timeout 0x22 (34 us).

Nothing in-tree writes these after the initvals.

## Caveats

- One adapter, near field, 40 frames per arm, one run per setting, one
  channel, 1400-byte frames only: the long-limit path (frames over 2032
  bytes) is unmeasured.
- No run with a peer ACK responder armed is recorded here, so the gate's
  ACK-terminated ladder (a Normal arm settling at 0 retries) is not in these
  tables; the delivery table above shows ACK-requesting retries working
  against a real AP.
- UNSETTLED rows are read for the retry value, never for the counts.
- The one-step status lag is attributed to the stale-EXT race above from
  this table's pattern, not from a bus trace. The claim recovers at most one
  entry per arm - the first - and only on the conditions above; after an
  UNSETTLED arm a previous-pktid entry is still reported as late. A deficit
  of more than one entry is not this race.
- `mt7612uprobe txs` reads two registers per status poll, so its `fps` is
  per-frame submit-to-status time, not comparable with any steady-state
  injection figure.
