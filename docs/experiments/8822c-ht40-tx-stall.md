# RTL8812CU (8822C) 40 MHz injection stall

Why devourer's 8822C could not transmit a single 40 MHz frame on most
channels while the same card does HT40 under mainline rtw88, and the two
fixes that came out of it (`src/jaguar3/FrameParserJaguar3.h` DISDATAFB on
the C die, and a wide-TX IGI floor on both Jaguar3 dies —
`DeviceConfig::Tuning::wide_tx_igi_floor`, applied by
`PhydmRuntimeJaguar3::set_igi_floor`). Bench: txdemo on the CU, rxdemo on
an 8812EU witness at the same channel/width, 1 kB QoS-data frames,
MCS3/LDPC. The witness setups are under Results.

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
measured (see Results for the ladder on the final code).

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

**Silence, not DFS, is the trigger.** Every silent channel above is a DFS
channel, where DIG pins IGI at 0x20, so the two were confounded until a
silent non-DFS channel was run: on 157(+) (an 8 s scan heard 0-4 frames)
the CU with the floor off hangs the same way at IGI 0x1e — 257 / 8 / 0,
and 335 / 12 / 74 on a second run — and 36(+), with a few 20 MHz frames a
second of ambient traffic, still wedges after 850. The floor therefore
applies on every channel, not only DFS ones.

**The edge is per unit, and it is not C-die-only.** A second bench (the
#484 review: LB-LINK WDN1300H 8812CU, chip-id 0x13 cut 3, and an LB-LINK
BL-M8812EU2 8812EU, rfe 21) found the same stall on its CU with a higher
edge, and on its EU, where our EU never showed it — see Results. Four
units so far:

| unit | 0x20 | 0x24 | 0x26 | 0x2c | 0x35 |
|---|---|---|---|---|---|
| 8812CU, this bench | hang (0x1e on 157 too) | clean | clean | clean | clean |
| 8812CU, second bench | hang | | hang | ~1.5 k then wedge | clean |
| 8812EU, this bench | clean (0x1e on 157 too) | | | | clean |
| 8812EU, second bench | hang | | | clean | clean |

(silent 144(-), MCS3/40; blank = not run; "hang" = the 257-submitted /
0-aired shape; this bench's 0x22 also hung). The vendor phydm has no
bandwidth-raised DIG bound to lean on (`phydm_dig_abs_boundary_decision`
pins DFS at `DIG_MIN_DFS` at every width), so the number is devourer's own.

The frame shape does not matter: the demo's default 148-byte management
frame (no `DEVOURER_TX_QOS_DATA`) stalls the same way on silent 144(-) at
40 MHz on both dies, master and the first floor revision alike (second
bench: 953 submitted / 8 failed / 0 witnessed).

## The fix

`wide_tx_igi_floor` (env `DEVOURER_WIDE_TX_IGI_FLOOR`, 0 = off), default
**0x35** — the lowest value measured to clear every unit above, not a
margin over a characterized edge: the second CU's edge sits somewhere in
0x2c..0x35 and was not bisected, and a fifth unit may need more. A unit
whose own edge has been measured can take a lower value. On both dies,
honoured by DIG. It is armed for the session by the first wide frame the
device transmits, and from then on follows the tuned width: applied while
tuned to 40/80 MHz, dropped at a narrower width, with IGI clamped straight
back into the normal window. `Init`, `InitWrite` and `Stop()` clear the
arm.

Cost: idle-channel RX sensitivity at those widths, estimated from the
~1 dB IGI step, **not measured**: 0x35 is 21 steps over the DFS floor 0x20
(144, 52) and 19-23 over the 0x1e..0x22 coverage window on a non-DFS
channel (36, 149). Only on a device that transmits wide frames; a ground
station (40 MHz receive, 20 MHz control frames) never arms it. An earlier
revision used 0x26 on the C die only — one step over this bench's CU, and
inert on the second bench's.

DISDATAFB stays C-only: the second bench's EU flew 40 MHz with
DISDATAFB=0 and the floor.

## Results

Four bench sessions, all 1 kB QoS-data at MCS3/LDPC with no other knobs.
"Decoded" means CRC-ok frames the witness heard at the expected rate and
width.
- **GS:** the CU on the host, witnessed by an 8812EU on a separate ground
  station. 50 s per run, on the first version of the fix (DISDATAFB=1 on
  every C-die frame, floor 0x26 on the C die). For 40 MHz frames on a
  40 MHz tune, that version and the 0x26 revision behave the same.
- **Local:** both adapters on one host, each witnessing the other
  near-field, with a run-unique SA. txdemo's default 2 ms gap, 30 s per
  cell, on the 0x26 / C-die-only revision unless marked †. Beforehand, an
  8 s witness scan of 36/52/100/120/144/149/157 at 40 MHz heard 0 frames.
- **Final:** the local pair again, on the final code (default floor 0x35
  on both dies unless the floor column says otherwise, via
  `DEVOURER_WIDE_TX_IGI_FLOOR`), 30 s of frames per cell bounded by
  `DEVOURER_TX_FRAMES=13800` (submitted counts txdemo's own 50 lead
  frames on top), run-unique SA, TX started only once the witness was in
  its RX loop. The EU-TX cells ran at `DEVOURER_TX_PWR_OFFSET_QDB=-80`
  (-20 dB): at full power the near-field CU witness read RSSI 88 / EVM -33,
  the saturation signature. 8 s scans at 40 MHz: 144(-) and 157(+) 0
  frames, 36(+) 36, 44(+) 103, 149(+) 71 (all 20 MHz).
- **Second:** the #484 review's bench (units above), near-field, no rtw88
  loaded, 2 ms gap, 20 s per cell, single runs. 144(-) and 100(+) silent
  (8 s witness scan at 40 MHz: 0 frames). "PR" = the 0x26 / C-die-only
  revision; a floor column other than 0x26 on the CU, or any floor on the
  EU, is that tree with the floor overridden or the C-die test removed.

| bench | channel | DUT -> witness | tune | frames | floor | submitted | failed | decoded |
|---|---|---|---|---|---|---|---|---|
| GS | 144(-) | CU -> EU | 40 | MCS3/40 | 0x26 | 20552 | 0 | 20500, bw:1 |
| GS | 52(+) | CU -> EU | 40 | MCS3/40 | 0x26 | 20525 | 0 | 20425, bw:1 |
| GS | 149(+) | CU -> EU | 40 | MCS3/40 | 0x26 | 20590 | 0 | 20477, bw:1 |
| GS | 36(+) | CU -> EU | 40 | MCS3/40 | 0x26 | 20483 | 0 | 19723, bw:1 |
| local | 144(-) | CU -> EU | 40 | MCS3/40 | 0x26 | 14117 | 0 | 14067, bw:1 |
| local | 144(-) | CU -> EU | 40 | MCS3/20 | none (0x20) | 14163 | 0 | 14111, bw:0 |
| local | 144 | CU -> EU | 20 | MCS3/20 | none (0x20) | 14285 | 0 | 14232, bw:0 |
| local † | 144 | CU -> EU | 20 | MCS3/40 | none (0x20) | 14282 | 0 | 0 (expected) |
| local | 144(-) | EU -> CU | 40 | MCS3/40 | none (0x20) | 14456 | 0 | 14406, bw:1 |
| second, master | 144(-) | CU -> EU | 40 | MCS3/40 | – | 257 | 8 | 0 |
| second, PR | 144(-) | CU -> EU | 40 | MCS3/40 | 0x26 | 257 | 8 | 0 |
| second, PR + `DIS_CCA=1` | 144(-) | CU -> EU | 40 | MCS3/40 | 0x26 | 257 | 8 | 0 |
| second, PR | 100(+) | CU -> EU | 40 | MCS3/40 | 0x26 | 308 | 8 | 51 |
| second, PR | 36(+), busy | CU -> EU | 40 | MCS3/40 | 0x26 | 7842 | 0 | 7727 |
| second, PR + flooder | 144(-) | CU -> EU | 40 | MCS3/40 | 0x26 | 7321 | 8 | 7064 |
| second | 144(-) | CU -> EU | 40 | MCS3/40 | 0x2c | 1752 | 8 | 1495 |
| second | 144(-) | CU -> EU | 40 | MCS3/40 | 0x35 | 7851 | 0 | 7801 |
| second | 100(+) | CU -> EU | 40 | MCS3/40 | 0x35 | 7837 | 0 | 7787 |
| second, PR | 144(-) | EU -> CU | 40 | MCS3/40 | none (0x20) | 257 | 8 | 0 |
| second, PR | 100(+) | EU -> CU | 40 | MCS3/40 | none | 269 | 8 | 12 |
| second, PR + flooder | 144(-) | EU -> CU | 40 | MCS3/40 | none | 4811 | 343 | 4095 |
| second | 144(-) | EU -> CU | 40 | MCS3/40 | 0x2c | 7858 | 6 | 7794 |
| second | 144(-) | EU -> CU | 40 | MCS3/40 | 0x35 | 7787 | 4 | 7725 |
| second | 100(+) | EU -> CU | 40 | MCS3/40 | 0x35 | 7897 | 3 | 7844 |
| second, PR | 144(-) | CU -> EU | 40 | MCS3/20 | none (0x20) | 7857 | 0 | 7807, bw:0 |
| final | 144(-) | CU -> EU | 40 | MCS3/40 | off (0x20) | 257 | 8 | 0 |
| final | 144(-) | CU -> EU | 40 | MCS3/40 | 0x35 | 13850 | 0 | 13800, bw:1 |
| final | 157(+) | CU -> EU | 40 | MCS3/40 | off (0x1e) | 335 | 12 | 74 |
| final | 157(+) | CU -> EU | 40 | MCS3/40 | off (0x1e) | 257 | 8 | 0 |
| final | 157(+) | CU -> EU | 40 | MCS3/40 | 0x35 | 13850 | 0 | 13748, bw:1 |
| final | 36(+), light | CU -> EU | 40 | MCS3/40 | off (0x1e) | 850 | 8 | 786 |
| final | 144(-) | EU -> CU | 40 | MCS3/40 | off (0x20) | 13850 | 0 | 13800, bw:1 |
| final | 157(+) | EU -> CU | 40 | MCS3/40 | off (0x1e) | 13850 | 0 | 13800, bw:1 |
| final | 157(+) | EU -> CU | 40 | MCS3/40 | 0x35 | 13850 | 0 | 13800, bw:1 |

Before the fix, every GS and local 40 MHz row aired nothing. "Flooder" =
an 8822BU sending MCS0/20 at 50 fps on the channel; in the CU flooder row
the 8 failures are the last 8 sends, after the flooder stopped ~2 s before
the DUT did — the hang resumes the moment the channel goes quiet.

What the local rows show:
- **40 MHz frames on a 40 MHz tune** arm the floor.
- **20 MHz frames on a 40 MHz tune** carry DISDATAFB=0 and do not wedge,
  so the fault keys on the frame's width, not the tune's. The floor stays
  unarmed (both benches).
- **40 MHz frames on a 20 MHz tune** leave the floor off. † is from one
  revision earlier, which then required a wide tune to arm. Later code
  arms the session but applies no floor at 20 MHz, so the result is the
  same by construction; this case was not re-run.
- **This bench's 8822E transmitting HT40** needed no floor (it ran under
  the C-die-only revision), and the CU witness, which receives at 40 MHz
  and never transmits, stays at 0x20. The second bench's EU contradicts
  the first half — which is why the floor now arms on both dies.

Retry ladder, 0x26 revision (DISDATAFB is unchanged since), in the
`tests/retry_ladder_probe.sh` shape: unACKed unicast, MCS3, retry limit 8,
CU -> EU on 144.
- **20 MHz:** 1223/1224 frames flew MCS3 x4 > MCS2 > 6M x4, the ladder
  #371 documents.
- **40 MHz:** 1208/1209 frames flew MCS3 x9, pinned by DISDATAFB=1.

What this does not show:
- The 0x35 default has been flown on this bench's two units (final
  rows) and, forced by hand, on the second bench's; a fifth unit is
  unknown. The EU-at-0x35 rows show only that arming it costs this EU
  nothing near-field — this EU never needed it.
- Every row is a single run, so the 0x2c and the partial rows are
  indicative; the 0-vs-7.8k rows are outside any noise band. Both local
  benches are near-field, so the rows are pass/fail, not range numbers.
- The RX-sensitivity cost of the floor — the thing a lower default would
  buy back — is estimated, not measured.
- The 36(+) GS row's lower decode count (96.3 % vs ~99.5 % elsewhere) was
  not repeated.
- The send rate is gap-limited (~460 fps), so a per-frame lock cost would
  not show in `submitted`.
- Not exercised on air: the IGI clamp when the floor drops (no demo
  changes width mid-session), and the retune/first-wide-frame race, which
  rests on lock ordering.
- Only one silent non-DFS channel (157(+)) was available; 36(+) is the
  only lightly-loaded one run.

Open: 80 MHz is only partly working — the second bench's CU -> EU on
149(+), VHT1SS_MCS3/80 with floor 0x26 ran 470 submitted / 10 failed /
209 decoded, and 0x35 has not been tried there. 2.4 GHz HT40 (ch 6)
drained on the TX side but the EU received none of it — unexplained.
