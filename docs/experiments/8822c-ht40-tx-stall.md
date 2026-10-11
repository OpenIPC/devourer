# RTL8812CU (8822C) 40 MHz injection stall

Why devourer's 8822C could not transmit a single 40 MHz frame on most
channels while the same card does HT40 under mainline rtw88, and the two
fixes that came out of it (`src/jaguar3/FrameParserJaguar3.h` DISDATAFB,
`PhydmRuntimeJaguar3::set_igi_floor`). Bench: txdemo on the CU (host),
rxdemo on an 8812EU ground-station card at the same channel/width,
1 kB QoS-data frames, MCS3/40/LDPC, 50 s per run; "aired" = frames the
EU received with CRC ok and `bw:1`.

## Symptom

HT20: 20 k frames in 50 s, every channel. HT40 on 144(-), 36(+), 149(+):
257 frames accepted by bulk-OUT, then rc=-7 and nothing on air — at the
abort the TX FIFO is full (`LQ 64/0 PUB 1745/19`, 2048 x 128 B pages), i.e.
the MAC never completed the FIRST 40 MHz frame. The vendor 88x2cu driver
in monitor mode trickles (273/7500 at 7-15 fps); mainline `rtw88_8822cu`
as a hostapd HT40 AP with a phone client sent 5.2 k MCS15/40 data frames
in 45 s on the same card — the chip is fine.

## Dead ends (all measured, 12+ runs)

- Firmware RA entry for the inject MACID — H2C 0x40 RA_INFO in every
  layout variant (macid 0/1, MEDIA_STATUS_RPT first, dis_ra, rate_id
  5/1/10+VHT, no_update_bw, init_lv, plus the 0x1D channel-switch H2C):
  the desc path stalls identically. The fw-driven path (USE_RATE=0)
  drains but at 20 MHz, and its own C2H RA report (0x0c, `curr_bw` byte 6)
  says it CHOSE 20 regardless of `ra_bw_mode`. The fw only honours a
  bandwidth through the vendor's `phydm_fw_fix_rate` mailbox — MAC 0x450 =
  `0x60<<24 | macid<<16 | bw<<8 | rate` — which does give a clean 40 MHz
  fw-driven drain on a quiet 149 (4446/0 failed) but is a per-MACID fixed
  rate, not devourer's per-packet design.
- Firmware version: the vendor blob is v9.17, rtw88 loads v9.9 — devourer
  built with v9.9 behaves the same.
- IQK (`DEVOURER_SKIP_IQK`), TX power (index 16), USB port, channel pair,
  `DEVOURER_DIS_CCA`, 0x878 bw-indication, AGC table index, the RF 0x3f
  mode-table write, DFS (52 — DFS, low band — also hung; see below).

## Fault 1 — DISDATAFB

devourer's Jaguar3 inject descriptor writes DISDATAFB=0 on purpose: it
keeps the firmware's per-retry rate-fallback ladder
(`DEVOURER_TX_RETRY_FALLBACK`, #362/#371). With USE_RATE=1 the MAC's
per-frame rate-fallback lookup still consults the fw's per-MACID rate
entry, which an injecting MACID lacks; on the 8822C that collision wedges
a 40 MHz frame outright. With DISDATAFB=1 (what the vendor's fixed-rate
xmit branch writes) the hard hang is gone — but only on some channels.

The fix sets DISDATAFB=1 on the 8822C's 40/80 MHz frames only, so they
have no retry ladder at either knob setting. 20 MHz frames keep
DISDATAFB=0 and the ladder: 20 MHz with 0 is the configuration #362
measured, and the HT20 rows below.

## Fault 2 — receiver gain on a quiet channel

With DISDATAFB=1: 36(+) perfect (20.7 k frames; a neighbour's RTS/CTS/BA
at ~200 fps on it), 149(+) works with 500 ms pauses that track the gaps
in a neighbour AP's traffic, 52/100/120/144 hang after 0-130 frames —
the SILENT channels. A 40 MHz TX needs the secondary 20 MHz to read
idle; the FA/CCA counters show nothing unusual, but the idle-vs-live
register diff shows the receiver parked in a different gain state
(RF 0x08/0x58, CCK/AGC status 0x1a30-0x1a68). Two proofs:

- the GS's second card sending MCS0/20 at 50 fps on silent 144 turns the
  immediate hang into 20.4 k frames / 0 failures;
- pinning IGI (0x1d70) on silent 144: 0x35, 0x2c, 0x26, 0x24 → ~20.6 k
  frames / 0 failures; 0x22 → 22 frames then hang. devourer's DIG floors
  were 0x1e (coverage) / 0x20 (DFS). The AGC lower bound 0x828[7:3]
  (vendor measures it per table; devourer uses 0xd) does not substitute
  (0x10/0x14: hang, 0x18: 139 frames).

Fix: `DIG_MIN_8822C_WIDE 0x26` (one step over the threshold) on the C die
at 40/80 MHz, honoured by DIG. It is armed for the session by the first
wide frame the device transmits, and from then on follows the tuned
width: applied while tuned to 40/80 MHz, dropped at a narrower width, with
IGI clamped straight back into the normal window. `Init`, `InitWrite` and
`Stop()` clear the arm.
Cost: idle-channel RX sensitivity at those widths on that die — estimated
from the 1 dB IGI step, **not measured**: 6 dB over the DFS floor 0x20
(144, 52), 4-8 dB over the 0x1e..0x22 coverage window on a non-DFS channel
(36, 149). Only on a device that transmits wide frames; a CU ground station
(40 MHz receive, 20 MHz control frames) never arms it. Nothing changes at
20 MHz or on the 8822E (its descriptor and DIG are byte-identical to before;
re-flown below).

## Result (no knobs)

| channel | width | submitted | failed | aired (EU, bw) |
|---|---|---|---|---|
| 144(-) | 40 | 20552 | 0 | 20500 MCS3 bw:1 |
| 52(+) | 40 | 20525 | 0 | 20425 MCS3 bw:1 |
| 149(+) | 40 | 20590 | 0 | 20477 MCS3 bw:1 |
| 36(+) | 40 | 20483 | 0 | 19723 MCS3 bw:1 |
| 144(-) | 20 | 20679 | 0 | 20629 MCS3 bw:0 (IGI stays 0x20) |

## Re-test after the arming review fix

Code at the time (arming needed a wide frame AND a wide tune; `Stop()`
disarmed).
Both adapters on one host, 1 kB QoS-data, run-unique SA, txdemo default
2 ms gap, 30 s per cell. Witness = the other local card. Before the run, an
8 s witness scan of 36/52/100/120/144/149/157 at 40 MHz heard 0 frames.

| cell | DUT -> witness | tune | frames | submitted | failed | witness | IGI |
|---|---|---|---|---|---|---|---|
| 144(-) | CU -> EU | 40 | MCS3/40 | 14161 | 0 | 14109 MCS3 bw:1 | 0x26 |
| 149(+) | CU -> EU | 40 | MCS3/40 | 14146 | 0 | 14094 MCS3 bw:1 | 0x26 |
| 144 | CU -> EU | **20** | MCS3/40 | 14282 | 0 | 0 (expected) | 0x20 |
| 144 | CU -> EU | 20 | MCS3/20 | 14198 | 0 | 14142 MCS3 bw:0 | 0x20 |
| 144(-) | **EU** -> CU | 40 | MCS3/40 | 14455 | 0 | 14321 MCS3 bw:1 | 0x20 |

The third row is the review fix: 40 MHz frames into a 20 MHz tune leave the
floor unarmed. The fourth is the 20 MHz control. In the fifth, the 8822E
transmitter stays at 0x20, and so does the CU witness, which receives at
40 MHz and never transmits.

What this does not show:
- The witness is near-field, one desk away. These are pass/fail results,
  not range numbers.
- Every cell is a single run.
- The send rate is gap-limited (~460 fps), so a per-frame lock cost would
  not show in `submitted`. The third row checks that the floor stays
  unarmed, not that the lock is no longer taken.

## Re-test after the #484 review

The code now sets DISDATAFB=1 on 40/80 MHz frames only, arms per session,
and clamps IGI when the floor drops. The bench is the same, on silent 144:

| cell | tune | frames | submitted | failed | witness | IGI |
|---|---|---|---|---|---|---|
| CU -> EU, 144(-) | 40 | MCS3/40 | 14117 | 0 | 14067 MCS3 bw:1 | 0x26 |
| CU -> EU, 144(-) | 40 | **MCS3/20** (DISDATAFB=0) | 14163 | 0 | 14111 MCS3 bw:0 | 0x20 |
| CU -> EU, 144 | 20 | MCS3/20 | 14285 | 0 | 14232 MCS3 bw:0 | 0x20 |
| EU -> CU, 144(-) | 40 | MCS3/40 | 14456 | 0 | 14406 MCS3 bw:1 | 0x20 |

The second row is the new combination: a 20 MHz frame with DISDATAFB=0 on
a 40 MHz tune does not wedge. The fault keys on the frame's width, not the
tune's.

Retry ladder, `tests/retry_ladder_probe.sh` shape: unACKed unicast, MCS3,
limit 8, CU -> EU on 144.
- **20 MHz:** 1223/1224 frames flew MCS3 x4 > MCS2 > 6M x4, the ladder
  #371 documents, restored.
- **40 MHz:** 1208/1209 frames flew MCS3 x9, pinned by DISDATAFB=1, as
  documented.

Not exercised on air: the IGI clamp when the floor drops (no demo changes
width mid-session), and the retune/first-wide-frame race, which is a
lock-ordering argument, not a measurement.

Open: 80 MHz inherits the floor untested; 2.4 GHz HT40 (ch 6) drained on
the TX side but the EU received none of it — unexplained.
