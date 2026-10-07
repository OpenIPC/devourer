# What an MT7612U station needs programmed

The measurement record behind the MT7612U half of `IRadio::SetStationIdentity`
and behind `AdapterCaps::station_mode_ok`. The contract lives at those
declarations (`src/IRadio.h`, `src/AdapterCaps.h`) and at
`mt7612u_set_station_identity()` (`src/mt7612u/include/mt7612u/mt7612u.h`);
the decision logic is `src/mt7612u/StationIdentity.h`, covered headlessly by
ctest `mt7612u_station_identity` (`tests/mt7612u_station_selftest.cpp`) and on
hardware by `mt7612uprobe staid`. This page holds the numbers.

Two questions had to be measured rather than read off registers:

- **BSSID** — does programming the joined BSSID (`MT_MAC_BSSID`, the
  `MT_MAC_APC_BSSID` slot table) change what a *managed station* receives, and
  is a wrong value silent, harmless or fatal?
- **Auto-ACK** — does this MAC acknowledge unicast addressed to its own
  address with nothing armed, and what does moving `MT_MAC_ADDR` (the port
  identity, which the ACK responder and the beacon path also write) do to it?

Rig for every cell: DUT = MT7612U, own MAC `40:a5:ef:5a:32:f8`, driven by
`mt7612uprobe` (not through `IRadio`); channel 6, near field, one DUT.

## Withdrawn numbers — read this first

The first run of the BSSID gate called `mt7612u_set_monitor_rx()` in every
arm. That function writes `MT_RX_FILTR_CFG = PHY_ERR|CRC_ERR` and nothing else
— every address and BSS drop bit off — so all six arms ran promiscuous and
were identical by construction. Its null result is withdrawn. The reasoning
that let it through was also wrong: in the managed filter `0x00015f97`, bit 3
(`OTHER_BSS`) is clear but bit **2** (`PROMISC`) is set, and bit 2 is the
address drop (mt76x2 sets it whenever the phy is not in monitor mode; an
earlier version of this page said mt76 maps it to `FIF_OTHER_BSS`, which is
not what `mt76x2u_config()` does - the decode is in the managed-filter section
below). The gate now leaves the managed value `mt_mac_start()`
programs, prints it per arm, and flags an arm that is not running it.

Also withdrawn: a "0.8% retried vs 98% control" auto-ACK figure from the
probe-response method (below), whose control ran with the monitor filter and
retargeted `MT_MAC_ADDR` at the same time. Everything below is re-measured
under the managed filter.

## Which APC slot a station's BSSID lives in

mt76 keys the slot on the station's **own** address, not on the BSSID:
`mt76x02_add_interface()` gives an interface index 0 (or `1 + (((base[0] ^
addr[0]) >> 2) & 7)` for a locally administered own address) and adds 8 for a
station; `mt76x02_bss_info_changed()` then writes the AP's BSSID into APC slot
`idx & 7`. The base is `MT_MAC_BSSID`, which mt76's station configuration
leaves equal to the station's own address. For this DUT (factory address
`40:a5:ef:5a:32:f8`, not locally administered) that is **slot 0**. The gates
compute it with `sta_station_slot()` (`src/mt7612u/tools/bringup.cpp`).

The first two runs below used the AP-side rule instead — the one
`beacon.cpp` applies to an AP's own address, which for an AP is the BSSID —
applied to the station's BSSID, giving slot 1. Slot 1 is not where a station's
BSSID lives, so every row that wrote the "derived slot" tested a slot the
station does not use. Those rows are marked below and are **not evidence**.
The third run (auto-ACK arm E) uses the station slot.

## BSSID: `MT_MAC_BSSID` and slot 0 do not gate a managed station's receive

`sudo AP_SYSFS=6-1 DUT_SYSFS=7-1 CH=6 tests/mt7612u_sta_identity.sh`
(`mt7612uprobe sta`). AP = RTL8812AU on the in-tree rtw88 driver, hostapd
2.10, BSSID `02:42:75:05:d6:aa`. Unicast at the DUT comes from a monitor vif on
the AP's phy (`tests/sta_unicast_inject.py`); hostapd sends an unassociated
station none. 20 s per arm; no arm touches `MT_MAC_ADDR`.

| arm | configuration | unicast to us (first run) | (re-run, AP = RTL8812BU) |
|---|---|---|---|
| A | init only, nothing programmed | 6250 | 6478 |
| B | `MT_MAC_BSSID` = AP | 5877 | 5875 |
| C | APC slot 0 = AP | 5878 | 5857 |
| D | APC slot 1 = AP *(not the station slot)* | 5891 | 5858 |
| E | `MT_MAC_BSSID` + slot 1 = AP *(slot: not evidence)* | 5888 | 5870 |
| **F** | **`MT_MAC_BSSID` WRONG** + slot 1 wrong *(slot: not evidence)* | **5877** | **5909** |

`filtr=00015f97` in every arm of both runs. Arms D-F of these two runs wrote
slot 1, which is not the station's slot, so those rows say nothing about the
slot table.

Against these two runs:

- The second run inherited state. Its D/E/F rows show slot 1's BIT(16) SET,
  left behind by the auto-ACK harness's `bssen` arm in an earlier process
  (the chip keeps registers across bring-up tool runs, and the gate reset only
  the address halves). The gate now clears the whole slot, enable bit
  included, and checks every other slot reads empty.
- The first run's `MT_MAC_BSSID` was not read back against the value written.
  The gate now reads the base and every slot back after `mt_mac_start()`, and
  an arm that does not read back as written makes it INCONCLUSIVE. So does an
  all-zero `to_us` column.

### On the station-slot gate

The gate as it now stands: station slot 0, every write read back, the
stimulus started only after the gate reports its bring-up done. Rig: DUT
MT7612U at 480 Mbit/s, AP RTL8812BU on rtw_8822bu on a USB3 port, hostapd,
ch6, near field; the injector achieved 35964 frames in 124 s (about 290/s of
300 asked). No MCU timeouts.

| arm | configuration | rx_total | from_bss | beacons | unicast to us | vs A |
|---|---|---|---|---|---|---|
| A | init only, nothing programmed | 6591 | 6470 | 193 | 6277 | - |
| B | `MT_MAC_BSSID` = AP | 6196 | 6050 | 193 | 5857 | -6.7% |
| C | APC slot 0 = AP | 6194 | 6055 | 194 | 5861 | -6.6% |
| D | station slot (mt76 rule, = slot 0) = AP | 6151 | 6040 | 194 | 5846 | -6.9% |
| E | `MT_MAC_BSSID` + slot 0 = AP | 6145 | 6046 | 194 | 5852 | -6.8% |
| **F** | **both WRONG** (base and slot 0) | 6261 | 6193 | 190 | **6003** | -4.4% |

Every write verified, every other slot empty, BIT(16) clear in C-F,
`filtr=00015f97` in every arm, and the gate reported "measured". A
deliberately wrong BSSID in both the base and the station's slot (F) received
as much as the correct ones (B-E) - slightly more.

What "no gating" can and cannot mean at this spread:

- It can mean: neither register decides whether a managed station accepts
  unicast addressed to it. A gate would show as a collapse of F (or of A,
  where both are empty), not a few percent.
- It cannot mean: the registers are without effect. B-F sit 4-7% below A,
  and most of that drop is B-E against A. That is the same first-arm excess
  every run of this gate has shown (6%, 10%, and now 6.7%), unexplained; with
  one run per arm it cannot be separated from ambient drift, and a real
  effect of a few percent would hide inside it.
- The receiver measured is the managed filter `0x00015f97`, not the monitor
  filter the library's own RX loop installs. One unit, one AP, one run per
  arm; BIT(16) was clear in every row here, and the enabled-slot case rests
  on the auto-ACK harness's arm E (acknowledgement, not reception).

### Runs that measured nothing: the rtw88 AP stalls (inferred)

Twice, on the station-slot gate before the stimulus ordering was added, the
gate's bring-up logged eight `mcu command timed out waiting for response`
(its channel calibrations), every arm then read 0-8 frames and no beacons,
and the gate returned INCONCLUSIVE, as it should. The AP (RTL8812BU on
rtw88) sat on a hub port that enumerated at FULL SPEED (12 Mbit/s); its
transmit path stalled at about 30 frames/s of the 300 asked (about 230/s in
the good run) and its beacons stopped, while the DUT's calibrations timed out
during the flood - the late-reply failure `mcu.cpp` documents under a strong
nearby transmitter. With both adapters on high-speed ports the same harness
then measured cleanly (above).

**The full-speed hub is not the whole story.** On the second unit's rig (below)
the AP - a TP-Link T3U, also RTL8812BU on rtw88, at a SuperSpeed root port -
stalled mid-table: `rtw88_8822bu: failed to get tx report from firmware` in
the kernel log, the injector at 12289 frames in 123 s (about 100/s), beacons
down to about 120 per arm, and `to_us` 0 from arm D on. So the rtw88 8822bu
transmit stall happens on a fast port too; a full-speed hub makes it likely,
it does not explain it. The good run on the earlier harness also had the AP at
the full-speed position. The cause is inferred, not proven, and it is on the
AP side: the DUT-side code before the timeouts is identical between the runs,
and no kernel driver touched the DUT during them.

Two guards stay in the harness: the stimulus starts only after the gate's
bring-up, and a table the injector fed at under half its rate is refused as
an AP-side stall - which is what refused the second unit's table. The harness
header states the rig requirement as necessary, not sufficient: both adapters
at high speed or better and no full-speed hub, and a stalled run is re-run,
not read.

This is the opposite of the AP-side finding in `docs/mt7612u-ap-mode.md`
(a wrong APC slot "beacons perfectly, acknowledges nobody"), which is about
acknowledgement, not reception; the two do not conflict.

## Auto-ACK: acknowledged with nothing armed, and `MT_AUTO_RSP_EN` is the gate

`sudo tests/mt7612u_sta_autoack.sh`. The instrument asks the *transmitter*: a
Realtek peer (RTL8812CU; Jaguar3 drains C2H off its coex runtime) injects
unicast QoS-Data at the DUT through `txdemo` (`DEVOURER_TX_QOS_DATA=1`,
`DEVOURER_TX_RA=<DUT>`, `DEVOURER_TX_REPORT=1`, `DEVOURER_TX_RETRY_LIMIT=12`)
and reads its own per-frame CCX `tx.report`. The DUT receives under
`mt7612uprobe norsp` (arms A, D) or `bssen` (arm E); A and D share one code
path and filter and differ by one bit.

| arm | first run: reports / ok / retries | second run: ok / retries | third run (station slot): ok / retries |
|---|---|---|---|
| **A** — DUT receiving, **nothing armed** | 1279 / **100.0%** / **0.45** | 845/845 / 0.12 | 858/858 / 0.04 |
| B — destination nobody holds | 400 / 0.0% / 12.00 | 0% / 12.00 | 0% / 12.00 |
| C — DUT not running | 400 / 0.0% / 12.00 | 0% / 12.00 | 0% / 12.00 |
| **D** — DUT receiving, `MT_AUTO_RSP_EN` **cleared** | 400 / 0.0% / 12.00 | 0% / 12.00 | 0% / 12.00 |
| **E** — wrong BSSID in the enabled **station** slot | *(slot 1: not evidence)* | *(slot 1: not evidence)* | **867/867** |

The report counts differ because an acknowledged frame retires at once while
an unacknowledged one holds the descriptor for 12 retries; the comparison is on
`ok`, a ratio. A vs B and C: the DUT acknowledges unicast to its own address,
and only its own. A vs D: `MT_AUTO_RSP_EN` gates it — which is why the seam
refuses when that bit is clear.

**Arm E: a wrong BSSID in the enabled station slot does not gate the
station.** The first two runs wrote slot 1 (above) and are not evidence. The
third run's DUT log reads `WRONG BSSID 02:00:00:de:ad:02 in station APC slot
0, BIT(16) SET (high reg 000102ad), other slots empty, MT_MAC_BSSID = own
address (verified)` - the slot mt76 would program, enabled, holding a BSSID
nobody has, with `MT_MAC_BSSID` left where mt76's station configuration leaves
it - and the peer's frames were acknowledged 867/867. So the BSSID plane does
not gate acknowledgement for a station on this part even with the enable
set. Against it: one run, one peer, one DUT, and acknowledgement only - no
reception count was taken in that arm.

Against it: one peer, one DUT, one run per arm per session. A monitor-filter
run of the same arms (arm A with `mt7612uprobe arx`, so A and D then differed
in filter and init path as well as the bit) read A 100% / 0.10 (887 reports),
B/C/D 0% — the same shape, but not a single-variable A/D pair.

**Two methods that cannot answer this**, kept so they are not retried:
capturing the DUT's ACKs on a monitor vif on the peer's own phy read zero with
the DUT present *and* absent (a radio cannot hear an ACK to its own
transmission; mac80211 injects no-ack); and counting retried probe responses
(`mt7612uprobe staack`) cannot fail, because its single-variable control
(clear `MT_AUTO_RSP_EN`) does not move — hostapd does not retransmit an
unacknowledged probe response.

`staack`'s arm C is still evidence of something else: with `MT_MAC_ADDR`
retargeted under the managed filter, the DUT's reception of the AP's unicast
went from 103 frames to **zero**. The port identity gates what a managed
station receives, not only whether it acknowledges — the reason the seam must
not write it, and must refuse when something else holds it.

## Uplink: what the station transmits is acknowledged

`sudo tests/mt7612u_sta_uplink.sh`. The DUT's own `MT_TX_STAT_FIFO`
(`mt7612uprobe txs`, `docs/mt7612u-tx-retry.md`) gives the per-MPDU retry
count; the peer is a Realtek adapter running `rxdemo` with
`DEVOURER_ACK_RESPONDER`. The row is the gate's arm `d` — unicast from the
DUT's own address, ACK requested — from its "MAC receiver ON" table (with the
receiver off the MAC cannot hear an ACK at all).

| arm | first run | second run (limit programmed) | third run (limit programmed) |
|---|---|---|---|
| **A** — peer answers for the address we transmit to | **200/200**, 0.0 retries (max 1) | 200/200, 0.0 retries | 200/200, 0.0 retries (max 1) |
| B — peer answers for a *different* address (control) | 0/200, 16.0 retries | not completed (outer timeout) | 0/157, 16.0 retries |

The first run used the chip's **initvals retry limit** (short limit 15). The
harness now passes `DEVOURER_TX_RETRY_LIMIT=15` explicitly and requires the
gate's read-back line; the re-run confirmed it (`retry limit set to 15
(MT_TX_RETRY_CFG 47f00f0f)`). The second run's control arm B was cut off by
an outer timeout: each harness arm runs the whole eight-arm gate twice at
about 6 frames/s, about 9 minutes for arm A at FRAMES=200 and longer for B,
where nothing is acknowledged; the script header carries the runtime budget.
The third run completed both arms; its control settled 157 of 200 status
entries (the UNSETTLED floor below). It says nothing about a default library session, which airs NOACK
stream radiotap and `tx.retry_limit` 0 and so sends each unicast once — a
station session must request ACKs and set a nonzero limit
(`IRadio::SetStationIdentity`; `Mt7612uRadio` warns at arm time when the limit
is 0).

Against it: arm B trips the gate's UNSETTLED marker by construction (every
frame runs the full ladder, and the 16-slot status ring cannot keep up). It is
accepted only as a floor: misattributed entries could come only from the
neighbouring 200/200 arms, so contamination can only make B look *better*. The
harness refuses an UNSETTLED arm that claims success. Peer is an ACK
responder, not an AP; one run per arm per session.

## Second unit, second rig

The maintainer's run: DUT MT7612U Comfast CF-922AC (`40:a5:ef:5f:65:51`, USB3
hub port 4-2.3.2), peer RTL8812CU (0bda:c812, high-speed port 3-2.4), AP
TP-Link T3U (RTL8812BU) on rtw88 at a SuperSpeed root port, hostapd 2.11, ch6,
near field, one run per arm.

| cell | second unit | first unit (b94e119) |
|---|---|---|
| `mt7612uprobe staid` | 12 passed, 0 failed | 12/12 |
| autoack A (nothing armed) | 1706/1706, 0.01 retries | 860/860, 0.06 |
| autoack B, C, D | 0% at 12.00 | 0% at 12.00 |
| autoack E (wrong BSSID, enabled station slot 0) | 1695/1695, 0.02 | 873/873, 0.11 |
| uplink A (FRAMES=60, limit 15 read back) | 60/60, **1.9** mean retries (max 3) | 200/200, **0.0** (max 1) |
| uplink B (control) | 0/60 at 16.0 (UNSETTLED floor) | 0/157 at 16.0 |
| staack arm C (`MT_MAC_ADDR` moved) | received nothing | received nothing |
| `sta` (BSSID) table | **not reproduced**: A 6141, B 5747 (−6.4%), then the AP stalled from arm C; the harness refused the table | A 6268 … F 6018, measured |

What the second unit reproduces: the auto-ACK half entire (claim, all three
controls, and arm E), the uplink claim and its control, and the arm-C
deafness. What it does not: the BSSID receive table, lost to the rtw88 AP
stall above (no non-rtw88 AP was available on that rig).

Against it, and against the comparison:

- **The uplink's mean retries differ between units: 1.9 against 0.0.** Both
  acknowledged every frame, so the claim holds on both, but 1.9 retries per
  frame is not "answered at once". One run per unit, different peers'
  placement and different RF; it is not explained here.
- B is 6.4% below A on the second unit's partial table - the same unexplained
  first-arm excess every run shows.
- Two units, one peer model, one channel, near field, one run per arm.

## The managed receive filter belongs to the armed station

Every cell above ran the managed filter `0x00015f97`, while
`Mt7612uRadio::StartRxLoop` installs the monitor filter (`PHY_ERR|CRC_ERR`).
So a station driven through `IRadio` used to run promiscuous, and the
property that justifies the seam's refusal - "moving `MT_MAC_ADDR` makes a
station deaf" - did not hold for it: under the monitor filter it keeps
receiving and only stops acknowledging (issue #461).

**The role is selected by the arm, with no new API.** A successful
`SetStationIdentity` reads `MT_RX_FILTR_CFG`, writes
`MT_RX_FILTR_CFG_MANAGED` and reads it back; `ClearStationIdentity` writes the
recorded value back and returns true only once that reads back (a failure
keeps the arm recorded, so a second clear retries). A refusal returns before
the filter is read, so it writes nothing; a managed write that does not read
back is undone (the undo read back too) and refused, and an undo that does
not read back either is recorded, so the clear still restores the pre-arm
value and a retried arm does not take the stranded managed filter for it.
While armed, `mt7612u_set_monitor_rx()` - which
`StartRxLoop` calls after every MAC start - keeps the managed filter and only
records the request, so the arm is order-independent. A beacon or ACK
responder that moves the port identity drops the arm and puts the pre-arm
filter back (the AP and responder paths depend on the monitor filter's `DUP`
clear); a failed beacon start that restores the arm reinstalls the managed
filter. All of it runs under `Mt7612uRadio::_mu`, the lock the existing
filter write and every channel change already take; the only other writes
are `mt_mac_start()` and `StartRxLoop`'s `mt7612u_set_monitor_rx()`, the
latter mediated as above, and the RX thread itself never writes it. The drop and restore writes are read back and a miss is
logged (the clear re-verifies). `Stop()` does not clear a still-armed
station: every bring-up rewrites the filter (the initvals, then
`mt_mac_start()`), and so does mt76, so a filter left armed at a close
reaches no later opener. The
policy half is `mt7612u_sta_rx_filter_request()` / `mt7612u_sta_arm()` in
`src/mt7612u/StationIdentity.h`, covered by ctest `mt7612u_station_identity`.

**The value is the measured one, unchanged.** These are DROP bits
(`regs.h`, from mt76's `mt76x02_regs.h`):

| bit | name | `0x00015f97` | what a station gets |
|---|---|---|---|
| 0 | CRC_ERR | drop | no FCS failures (`rx.keep_corrupted` applies to the monitor filter only; FCS-bad frames are dropped while armed) |
| 1 | PHY_ERR | drop | |
| 2 | PROMISC | **drop** | unicast whose addr1 is not `MT_MAC_ADDR` is dropped - the deaf-on-move property |
| 3 | OTHER_BSS | keep | frames of every BSS still arrive |
| 4 | VER_ERR | drop | |
| 5 | MCAST | keep | group-addressed data |
| 6 | BCAST | keep | beacons of every BSS, broadcast probe responses, broadcast data |
| 7 | DUP | drop | hardware duplicate drop, as mt76 runs a station (sta_client's `DupDetector` stays) |
| 8-12 | CFACK, CFEND, ACK, CTS, RTS | drop | control frames a station has no use for |
| 13 | PSPOLL | keep | (mt76's `configure_filter` would drop it; immaterial to a station) |
| 14 | BA | drop | |
| 15 | BAR | keep | |
| 16 | CTRL_RSV | drop | |

What `tests/sta_client.cpp` needs while armed is all kept: beacons and probe
responses from every BSS (broadcast, or unicast to `own`) for a scan and a
re-join, authentication / association / EAPOL / data addressed to `own`, and
group-addressed data. What it loses is only what `StationSm::on_rx` already
refused: another station's unicast (`not-for-us`) and probe responses to
other stations. No disarm-while-scanning is needed.

**Witness.** `tests/sta_client_onair.sh` (an MT7612U DUT) injects two plaintext unicast
streams from the AP's BSSID while the station is associated: one at an address
nobody holds, one at the station's own address. The own stream is the positive
witness that the injection reaches the DUT - the station counts it as
`plaintext refused` (a WPA2 link), and it must reach half of what was
injected or the check is INCONCLUSIVE. Then armed (`wpa2`), `not-for-us` must
stay under 1% of the foreign stream; unarmed (`noarm`, the monitor filter) at
least half of it must arrive. Hardware gate:
`mt7612uprobe staid` checks the filter value across arm, re-request, refusal,
clear and drop. On-air numbers: TBD.

## What is not established

- **The managed filter under a live association is not yet measured on
  air** (TBD, above). The cells that measured it ran unassociated, through
  the bring-up tool.
- **No BSSID/auto-ACK cell drove `SetStationIdentity` through `IRadio`.** The
  seam writes no identity register - `mt7612uprobe staid` reads
  `MT_MAC_ADDR`, `MT_MAC_BSSID` and all eight APC slots before and after
  arming and clearing and checks them unchanged - and installs the managed
  filter those cells ran, so the measured state is what a successful arm
  leaves behind, but "arm the seam, then measure" is unexercised by those
  cells.
- **Every cell is an unassociated station** receiving traffic it did not
  negotiate: power save, TIM parsing, cross-BSS duplicate detection and
  hardware key lookup are untested.
- Two DUTs for the auto-ACK, uplink and arm-C cells, one for the BSSID
  table; one peer model, one channel, near field, no soak.
