# The Realtek station arm: bench record

`IRadio::SetStationIdentity` on Jaguar1/2/3. The contract is on the
declaration in `src/IRadio.h`. How the Realtek arm differs from the MT7612U's
is in `src/StationArm.h`. The flag is `AdapterCaps::station_mode_ok`. This
page holds the run behind that flag and its limits.

## The cell

`tests/realtek_station_onair.sh`. Its header defines the arms (A–H), their
controls and the verdicts. Both halves are read off the transmitter's own CCX
reports (`tx.report`), at retry limit 12, MCS3, with 200-byte frames, a 5 ms
gap and 10 s per arm.

## Runs

The rig:
- ch6, near field, one run per arm per record;
- an RTL8812CU (8822C) and an RTL8812BU (8822B), each the other's peer;
- the AP is an MT7612U on mt76x2u running hostapd;
- rtw88 was not blacklisted: the demos detached it, and the harness handed
  every adapter back.

There are two records on this rig: the first on the arm's first version, and
the current one on the reviewed code, which adds arm H. Every run exited 0
with 5 verdicts passed. The second record matches the first arm for arm,
within a few frames and a few hundredths of a retry. Both were scored before
the harness gained its transmitter-liveness gate and per-arm submission
floor, and before it judged reception against reported rather than
submitted frames; a run on the current harness is pending.

Each cell is reports / submitted, ok, mean retries, then rx_distinct where
the arm counts reception.

| arm | 8812CU station | 8812BU station |
|---|---|---|
| A | 1616 / 1658, 100.0%, 0.03, 1616 | 860 / 910, 100.0%, 0.33, 860 |
| B | 861 / 1694, 0.0%, 12.00, 870 | 85 / 902, 0.0%, 12.00, 86 |
| C | 858 / 1684, 0.0%, 12.00 | 88 / 911, 0.0%, 12.00 |
| D | 861 / 1698, 0.0%, 12.00, 1101 | 87 / 918, 0.0%, 12.00, 90 |
| E | 859 / 1694, 0.0%, 12.00, 1098 | 88 / 918, 0.0%, 12.00, 91 |
| F | 2291 / 2341, 100.0%, 0.09 | 3095 / 3137, 100.0%, 0.20 |
| G | 222 / 1369, 0.0%, 12.00 | 1615 / 2547, 0.0%, 12.00 |
| H | 2399 / 2449, 100.0%, 0.08 | 3202 / 3244, 100.0%, 0.20 |

The first record has no arm H; the other arms read:

| arm | 8812CU station | 8812BU station |
|---|---|---|
| A | 1616 / 1658, 100.0%, 0.02, 1616 | 847 / 897, 100.0%, 0.28, 847 |
| B | 862 / 1689, 0.0%, 12.00, 870 | 88 / 917, 0.0%, 12.00, 91 |
| C | 862 / 1688, 0.0%, 12.00 | 87 / 917, 0.0%, 12.00 |
| D | 858 / 1690, 0.0%, 12.00, 1100 | 89 / 926, 0.0%, 12.00, 93 |
| E | 866 / 1691, 0.0%, 12.00, 1096 | 87 / 918, 0.0%, 12.00, 90 |
| F | 2317 / 2367, 100.0%, 0.11 | 3095 / 3137, 100.0%, 0.20 |
| G | 229 / 1377, 0.0%, 12.00 | 1605 / 2538, 0.0%, 12.00 |

## What the arm is for

Arm H is F's uplink with the DUT NOT armed. It was ACKed 100% on both dies,
at the same retries as armed F. So on Jaguar2/3 the UP half of the bar does
not depend on the arm: the AP acknowledges by address, and the transmitter
counts that ACK whether or not MACID holds the station's address. What
needs the arm is the DOWN half. With the arm absent (D) or cleared (E), the
DUT ACKs nothing addressed to it.

`station_mode_ok` rests on both halves being met while armed, which is how
a station runs. H adds that the uplink half holds without the arm too. It
is reported and never scored.

## Limits

- **The run's scope.** One unit per die, two runs per arm on one rig (one
  for H), near field, one channel, one AP type, and unassociated
  throughout. Power save, TIM, cross-BSS duplicate detection, hardware key
  lookup and the managed receive filter are all untested.
- **What "received" means.** `rx_distinct` is the DUT's count of distinct
  frames from the peer, taken from rxdemo's `rx.seq` stream. The station runs
  the promiscuous monitor filter, so it also received in the controls where
  the frames were not addressed to it, or where it was unarmed: B, D and E
  received about 870–1100 frames on the CU and about 86–93 on the BU. In
  arm A, "received" means only that the frames arrived. The arm changes the ACK,
  which is read off the peer's reports, not reception.
- **Submitted exceeds reports on every arm.** By arm group:
  - acknowledged arms (A, F, H): 40–50 frames;
  - unacknowledged downlink arms (B to E): about half the submissions on the
    CU (about 860 of 1690) and about nine in ten on the BU (about 87 of 910);
  - the G arms: 222–229 of about 1370 reported on the CU (83–84% missing),
    and 1605–1615 of about 2540 on the BU (36–37% missing).

  That fits frames still queued in the chip when the window closes, because
  an unacknowledged frame airs 13 times before its report. It is not proven.
  In arm A, `rx_distinct` equals the report count on both units.
- **Dies not measured by this cell:** the 8822E, the 8821C, and every
  Jaguar1 die. On the 8812, arm D is predicted to answer, because bring-up
  programs the EFUSE MAC into MACID; run it with `EXPECT_UNARMED_SILENT=0`.
