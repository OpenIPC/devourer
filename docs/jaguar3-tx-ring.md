# Jaguar2/Jaguar3 TX page ring — the beacon overwrite, and the fix

**The mechanism is known and fixed on Jaguar3 and Jaguar2.** It is a
one-constant porting defect: devourer enabled only the DMA bits of `REG_CR`
before the LLT init, where halmac enables all eight - and a bit bisection
names the one that matters: with the DMA bits, PROTOCOL_EN
(bit 4) was enough (`0x1F`); PROTOCOL without the DMA bits was not measured.
An earlier workaround that wrote the ring terminator into the LLT directly
matched the vendor chip's end state but not how it gets there; it never
landed here. The durable per-chip facts live in `src/jaguar3/CLAUDE.md` and
`src/jaguar2/CLAUDE.md`; this file is the measurement record.

Several figures below come from running a devourer AP (`tests/ap_wpa2.cpp`)
against an MT7612U station through an on-air station harness. Those harness
pieces - the MT7612U station, `tests/sta_d2d_onair.sh`, and the `ap_wpa2`
diagnostics named below (`DEVOURER_AP_INJECT`, `DEVOURER_AP_PKTBUF`,
`DEVOURER_AP_PKTBUF_BNDY`, the TX-DMA watchdog) - **land with the station-mode
PR and are not runnable on this branch**; every figure that came from them is
marked "station harness". What this branch carries, and what a reader can
run here:

- the fix;
- the in-tree reproducer, `txdemo` with `DEVOURER_TX_BEACON_TU` (see the
  regression set), and `txdemo`'s `txdma_status` field;
- the read-only diagnostics it was found with - `IRtlRadio::GetTxDmaStatus`,
  `ReadPacketBuffer`, `DumpMacRegisters`, `DumpChipState` (contracts and
  which backends implement them: their declarations) - reachable from
  `build/chipstate --pktbuf N` / `--mac-dump` (see the regression set for
  when those reads mean anything).

It also carries a Jaguar3 queue change found on the way: 802.11 data frames
leave the 64-page HIGH queue the beacon shares. The contract is in the
headers (`src/jaguar3/TxQueueMap.h`, `src/AmpduMode.h`,
`RtlJaguar3Device.h`); the measured facts and the unmeasured shapes are in
`src/jaguar3/CLAUDE.md`. For this record: every in-tree A-MPDU harness
(`ampdu_ba_check`, `ampdu_spike`, `ampdu_pacing_sweep`, `ampdu_onair_ab`,
`arq_e2e_delivery`, `bench_onair.py`) sends `DEVOURER_TX_QOS_DATA=1`, so the
data-frames-only A-MPDU rule changes no recorded A-MPDU figure.

The queue measurement behind it (8812CU as AP, downlink load, station
harness), HIGH/LOW/NORMAL/public pages as configured/available:

| state | HQ | LQ | NQ | PUB |
|---|---|---|---|---|
| healthy | 64/64 | 64/64 | 64/64 | 1745/1745 |
| TX wedged | 64/0 | 64/64 | 64/64 | 1745/1449 |

Every frame went to the HIGH endpoint, so HQ drained while LOW and NORMAL
were never used; with HQ empty the beacon could not be loaded and the AP
answered none of seventeen received authentication requests while its
receiver kept decoding. HQ running dry turned out to be a consequence of
the page-ring wedge, not its cause: routing data to LOW moved the exhaustion
there without touching the wedge. Moving QSEL alone, without the endpoint,
changed nothing (HQ still drained 64 -> 0).

And a beacon-arm rollback on Jaguar2 and Jaguar3 (contract at `StartBeacon`
in `RtlJaguar2Device.h` / `RtlJaguar3Device.h`), not exercised on hardware -
it needs a transfer to fail mid-arm.

## The defect in one paragraph

The TX FIFO is 2048 pages of 128 bytes, chained by the LLT. The auto-LLT init
links every page to the next, `0 -> 1 -> ... -> 2047`, and page
`rsvd_boundary` (1938) is where the beacon lives. On a correctly configured
chip the hardware terminates the data ring at the boundary itself -
`LLT[1937]` reads `0x792` at init and `0` by the end of any run that has gone
past one traversal (the moment it changes was not observed); with `REG_CR`
lacking PROTOCOL_EN at the LLT init, it did not - the data ring ran on into
the reserved region. Under sustained TX with a beacon armed, the data
allocator is eventually handed page 1938 and overwrites the beacon; the next
TBTT reads a data frame as a beacon descriptor and `TXDMA_STATUS` latches
`BIT_TXPKTBUF_REQ_ERR`. The chip transmits nothing more for the life of the
process. The rtl88x2cu vendor driver's chip, read live on the same adapter,
differs from ours in exactly one LLT entry: `LLT[1937] = 0`. The chip writes
that entry itself - when the MAC is configured the way the vendor configures
it. Plain monitor injection never showed it: with no beacon armed, nothing
reads page 1938.

| RTL8812CU as AP, MCS7, ch6 | before | `REG_CR` fix |
|---|---|---|
| injection with the beacon armed (4000 frames; `ap_wpa2` stress, station-mode PR) | fault at ~172–208 frames | 4000/4000 |
| `txdemo` 8000 frames, max duty | clean | 8050/8050, `txdma_status` 0 |
| downlink goodput (station harness, later PR) | 0.445 Mbit/s | 29.6 Mbit/s, 1.2% loss (top rung offered) |
| our beacon under downlink load (station harness) | 8% of idle | 100% |
| flood ping (station harness) | ~21 rt/s, 78–89% loss | 305 rt/s, 5.2% loss; re-run 351 rt/s, 1.4% |

Single runs each; the flood pair shows the run-to-run spread is several
points. (`txdemo`'s `submitted` is the transport's count of every bulk-OUT,
bring-up's included; the demo's own frames stop at exactly 8000, and the
extra 50 (8822C/E) / 42 (8822B) fit each die's firmware-download chunk count
- fits, not verified.)

**Bidirectional soak on the fix, 5 GHz** (station harness; ch36, MCS7,
4 Mbit/s each way at once): 8812CU AP 30 min and 8812BU AP 15 min, both 5/5 -
one association, zero MIC failures, both ledgers close (~643k and ~322k frames
each way), no quarter-on-quarter degradation beyond the grader's 20%
allowance (8812CU up 3.984 -> 3.985, down 3.862 -> 3.860 Mbit/s; 8812BU up
3.996 -> 3.996, down 3.919 -> 3.856). Counterparts: downlink loss
averaged 3.3% (8812CU, worst chunk 5.0%) and 3.1% (8812BU, worst 3.9%) with no
retransmission; AP resident memory grew 16 kB in 30 min on the 8812CU but
**532 kB in 15 min on the 8812BU**. That was later separated as warm-up, not
a leak (2026-09-26, one run): a 30-min soak with the 8812BU as AP (harness
retry defaults, MT7612U station, ch36), RSS sampled every 30 s by an external
sampler, went 11516 -> 12008 kB inside the first minute and then held
12012 kB for all 61 samples; that soak was itself 5/5, up 0.00% every chunk,
down 0.00-0.06%. The first 8812BU soak did NOT survive: see Jaguar2 below.

## 1. The mechanism - how the vendor's chip gets `LLT[1937] = 0`

**Its hardware writes it, because the vendor enables the whole MAC -
PROTOCOL_EN in particular - before the LLT init. devourer enabled only the
DMA bits.**

halmac's `MAC_TRX_ENABLE` for the 8822C (and 8822E, and 8822B) is `0xFF`:
HCI TX/RX DMA, TX/RX DMA, PROTOCOL, SCHEDULE, MACTX, MACRX. `init_trx_cfg`
writes it to `REG_CR` just before `priority_queue_cfg` runs the auto-LLT init.
devourer's port had `MAC_TRX_ENABLE = 0x0F` - the four DMA bits only - and set
the rest later, after the LLT init had already run.

How it was established, each step with the control that makes it mean
something:

| step | result |
|---|---|
| **A/B, GENERAL_INFO** (the first lead; both arms with the direct LLT write disabled) | control (no H2C): fault at 208 frames, `LLT[1937]` stays `0x792`. With GENERAL_INFO + PHYDM_INFO: fault at 206 frames, `LLT[1937]` stays `0x792`. |
| were the H2C packets really delivered? | both built byte-identical to the vendor's 80-byte transfers (headless test against the usbmon bytes); the H2C queue's hardware write pointer AND the firmware's read pointer both advanced by 64 bytes - received and consumed. **GENERAL_INFO is not the mechanism.** |
| does the vendor host write the LLT? | no - its whole captured bring-up touches the packet-buffer window once, for the GENERAL_INFO poll of the H2C queue |
| the vendor chip, freshly bound, idle | `LLT[1936..1939] = 791 792 793 794` - **identical to ours**. The terminator is not there at init. |
| the vendor chip after 500 injected frames, monitor mode, **no AP, no beacon** | `LLT[1937] = 0`. The allocator wraps at the boundary on its own. |
| diff of the vendor's register writes vs ours, TRX enable through LLT init (usbmon, both captures) | exactly one difference: `REG_CR` `0xFF` vs `0x0F` |
| devourer with `REG_CR = 0xFF`, no direct write | 2001/2001 then 4000/4000 frames, zero faults; after the run `LLT[1937] = 0` - written by the hardware - and page 1938 still holds the beacon descriptor |
| **which bit** (8812CU, ch36, 4000 frames each) | `0x1F` (DMA + PROTOCOL): **4002/4002, clean, terminator written**. `0x2F` (DMA + SCHEDULE): fault at 212. `0xCF` (DMA + MACTX + MACRX): fault at 175. PROTOCOL_EN (together with the DMA bits) is the one that matters, on the 8822C; PROTOCOL without the DMA bits was not run. The 8822E and 8822B were fixed with the vendor's full `0xFF` and not bisected. |

The usbmon diff covered the window from the TRX enable to the LLT init. The
bisection is what places the requirement AT the LLT init: the later full
`REG_CR` write (`0x06FF`, which includes PROTOCOL_EN) comes after the PHY
tables, and the `0x0F` arms still faulted.

The GENERAL_INFO port (a header-only byte-exact builder, its selftest, the
H2C-packet send path with its own sequence counter, and the A/B switches) is
not in the tree: it is not needed for this, and sending it in bring-up would
change firmware state under every existing Jaguar3 validation for no measured
benefit. The captured bytes are below in case a future feature needs the
packet path.

<details><summary>The captured GENERAL_INFO / PHYDM_INFO bytes</summary>

usbmon, vendor rtl88x2cu on an RTL8812CU, bulk-OUT endpoint `0x05`, 80 bytes
each (48-byte descriptor `TXPKTSIZE=32 QSEL=0x13`, checksum `0x1320`, then the
32-byte packet):

```
GENERAL_INFO  01 ff 0d 00 0c 00 00 00  00 00 38 00 ...   FW_TX_BOUNDARY 56, seq 0
PHYDM_INFO    01 ff 11 00 10 00 01 00  03 02 05 33 00 07 00 00 ...
              rfe 3, HALMAC_RF_2T2R, cut 5, rx|tx ant 3|3, ext_pa 0,
              package_type 7 (from its MAC-hidden report), seq 1
```

Both firmware blobs (8822C 9.0.17, 8822E) report H2C format version 15.
</details>

## 2. The 8822E had the defect, and the fix removes it

Measured 2026-09-25 on an 8812EU (`0bda:a81a`). Same `rsvd_boundary` (1938)
as the 8822C.

| 8812EU | result |
|---|---|
| at init, before the beacon (both arms) | `LLT[1936..1939] = 791 792 793 794`, the same as the 8822C |
| control, `MAC_TRX_ENABLE` temporarily back to `0x0F`, 4000 frames with the beacon armed (`ap_wpa2` stress, station-mode PR) | fault at 172 frames, page 1938 = `5a 5a ...`, `LLT[1937]` never terminated |
| `0xFF` | 4000/4000, no fault, end-of-run `LLT[1937] = 0`, beacon page intact |
| `txdemo` 8000 frames, max duty | 8050/8050, `txdma_status` 0 |
| as AP, MT7612U station, **ch36** (station harness) | beacons 3/3 (ours 103% of idle under downlink); throughput 2/2 - up 19.9 Mbit/s at 0.27%, down 29.8 Mbit/s at 0.81% |
| as AP, **ch6** (station harness) | the station never completes the four-way. UNATTRIBUTED: consistent with the documented 2.4 GHz TX limitation of this module (`docs/8822e-quirks.md`), but ch6 was never tried before the fix, so "not this fix" is untested |

And an observation that contradicts that quirk entry (which records no
receiver decoding any 2.4 GHz TX from this module): on ch6 the MT7612U
decoded about half of the 8812EU's beacons (~20/s of 39/s aired). One run;
the quirk entry is annotated.

## 3. Jaguar1 - checked, clear. Jaguar2 - had the same defect, fixed

**Jaguar1 (RTL8812AU, on air 2026-09-25): no instance of either Jaguar3
defect** (the page ring, or data queued as management - see
`src/jaguar3/CLAUDE.md`).

- *The page ring* cannot run into the beacon by construction: the 8812A/8821A
  LLT init is the old manual one (`HalModule::InitLLTTable8812A`), and it ends
  the data free list explicitly - `LLT[txpktbuf_bndy - 1] = 0xFF` - with the
  beacon pages above the boundary on a separate ring. (The 8814A uses the
  auto-LLT and is not covered by this; no 8814AU was on the bench.)
- *The queue mapping* is consistent: every frame carries QSEL `0x12` (MGT) on
  the first bulk-OUT endpoint, and Jaguar1's own priority init maps MGT to
  the HIGH queue that endpoint feeds. Data therefore competes with
  management for HIGH-queue pages - a throughput question, not a fault.
- *On air*, the 8812AU as the `ap_wpa2` AP with an MT7612U station (station
  harness), ch6, MCS7: beacons 3/3, our beacon 101% of idle under downlink
  load; the AP aired 4320 frames with 0 send failures. Downlink clean to
  13.8 Mbit/s at 0.28% loss, saturating at ~12.5-12.8 Mbit/s above that -
  about half the Jaguar3 AP's ceiling, degrading smoothly, never collapsing.
- **Against it:** the uplink into this 8812AU lost a flat ~18-21% at every
  rate, 1 to 30 Mbit/s offered, MCS1 no better than MCS7. The station sent
  those frames without requesting an ACK, so it is single-shot loss: frames
  the 8812AU did not decode. Passive reception on the same unit was no worse
  than the kernel's (devourer `rxdemo` 84.0%, rtw88 80.5%, one 1 Mbit/s rung
  each, separate runs; an 8812CU got 95-99% of the same frames), so the unit
  or its placement is the leading explanation - but devourer's Jaguar1
  receive path is not excluded. Not re-measured.

**Jaguar2 had the identical `REG_CR` defect - fixed and measured 2026-09-25**
on an 8812BU (`0bda:b812`). `src/jaguar2/HalmacJaguar2MacInit.cpp` had the
same DMA-only `MAC_TRX_ENABLE = 0x0F`; halmac's is `0xFF` for both the 8822B
and the 8821C. Same `rsvd_boundary`, 1938.

| 8812BU | result |
|---|---|
| unchanged code (`0x0F`), 4000 frames with the beacon armed (`ap_wpa2` stress, station-mode PR) | fault at 358 frames: page 1938 overwritten with `5a 5a ...`, `TXDMA_STATUS` 0 -> `0x10` -> `0x15`, then every bulk-OUT times out |
| `0xFF` | 4000/4000, no fault, end-of-run `LLT[1937] = 0`, beacon page intact |
| `txdemo` 8000 frames, max duty | 8042/8042, `txdma_status` 0 |
| as AP, MT7612U station, ch6 (station harness) | beacons 3/3 (ours 100% of idle under downlink, 4317 aired, 0 failed) |
| as AP, **ch36** (station harness) | throughput 2/2 - up 19.9 Mbit/s at 0.16%, down 29.6 Mbit/s at 1.34% |
| as AP, **ch6**, single-shot (no retries either end) | throughput FAILS its 5% gate: downlink up to 28.4 Mbit/s but at a flat ~5.5-6% loss; uplink 40-47% at every rate in one run, 31% falling to 2.5% with rate in another |
| as AP, **ch6**, retries on at both ends (AP retry limit 3, station 5) | passes: uplink 0.00% to 20 Mbit/s; downlink 0.67-3.10% per rung. Same-session control, the 8812CU as AP: downlink 0.12-0.34%. One run each |

**The ch6 single-shot losses are UNATTRIBUTED.** Passive RX on this unit is
no worse than the kernel's (rtw88 74.9%, devourer 89.0%, separate runs; the
8812CU of those runs 99.2%). Of the downlink, 5365 data frames submitted,
5197 (96.9%) were seen on air by an 8812CU witness and 4986 (92.9%) decrypted
by the station; the witness's own miss rate was not measured, and the same
station decoded the 8812CU AP's ch6 downlink at ~98.8%, so a
transmitter-side share (RF/EVM of this 8812BU, or its programmed TX power) is
not excluded.

**A second Jaguar2 defect found on the way, fixed:** the first throughput run
killed the AP. Its DIG thread (`RtlJaguar2Device::StartRxLoop`) called
`dig_step()` every 100 ms with no exception handling, and a USB control read
(`rtw_read(0c50)`, the IGI) threw `iostream error` under a 14-20 Mbit/s
uplink - `std::terminate`, core dump. The file already documented that such
reads "race the async bulk-IN and throw under load" and guarded the CFO
tracker for it; DIG and the thermal-track thread were not guarded. Both now
skip and count the tick. It fired about once per ladder afterwards - a
recurring event, not a one-off. **And it is not only DIG's problem:** the
first 15-minute 8812BU soak died at minute 9 the same way, through an
unguarded caller-side `GetTxDmaStatus` poll (the `ap_wpa2` TX-DMA watchdog,
later PR, polling every 100 ms). The log shows 12 isolated read failures over
those 9 minutes with bulk traffic succeeding between every one: the chip was
healthy, the reads just fail about once a minute under a 4+4 Mbit/s load.
The poller contract is at `IRtlRadio::GetTxDmaStatus`, and `txdemo`'s
`tx.stats` follows it (`docs/logging.md`). With the poll guarded the soak
rerun passed 5/5 with 17 failed reads absorbed. Jaguar2's `ReadPacketBuffer`
and `GetTxDmaStatus` are ported from Jaguar3 (read-only; the 88xx common
`read_buf` addressing).

**In-tree check on the 8812BU, 2026-09-27** (the review-round-5 tree; ch36,
beacon armed with `DEVOURER_TX_BEACON_TU=25`, QoS data 1400 B, RX thread,
adapter re-enumerated before each run; control = the same tree with the
Jaguar2 `MAC_TRX_ENABLE` back at `0x0F`):

- **`txdemo` does NOT reproduce the fault on Jaguar2.** Unfixed: 3 x 8000
  frames, each 8043 submitted / 0 failed, plus 1 x 30000 frames at a 1450-byte
  payload, 30043 / 0 failed, `txdma_status` 0 - identical to the fixed build
  (3 x 8043 / 0). The Jaguar2 fault record stays the `ap_wpa2` stress above
  (fault at 358 frames), which lands with the station-mode PR.
- **But the LLT shows the mechanism, with a control.** `chipstate --pid
  0xb812 --pktbuf 1938`, read-only right after a 2000-frame beacon-armed run
  (Jaguar2 stays powered after a session), 2 runs per arm, identical both
  times:

  | build | `LLT[1936]` | `LLT[1937]` | `LLT[1938]` | TX page 1938, first bytes |
  |---|---|---|---|---|
  | `0x0F` (unfixed) | `0x791` | `0x792` | `0x793` | `00 00 00 00 ...` (no beacon descriptor) |
  | `0xFF` (fixed) | `0x791` | `0x000` | `0x793` | `38 00 30 81 00 10 00 00 ...` (the beacon descriptor) |

  Without the fix the data ring is not terminated at `rsvd_boundary` and the
  beacon page does not hold the beacon; with it the hardware writes
  `LLT[1937] = 0` and the page holds the descriptor. The unfixed page reads
  as above - what wrote those zeros was not established.
- The fixed build's beacon arm + stop (review rounds 3-5) worked on every
  run: armed, then stopped.

**Untested: the 8821C (USB) and the PCIe 8821CE.** They share
`HalmacJaguar2MacInit.cpp`, so they now run `MAC_TRX_ENABLE = 0xFF` too -
halmac's own value for the 8821C - but neither was on the bench, before or
after the change.

## What the fix may cost the plain injector - measured, near the noise floor

The fix changes the monitor/injection path too (every Jaguar2/3 bring-up
now enables the MAC protocol engine), so it was A/B'd against upstream
master on that path (2026-09-26): an 8812CU injecting 8000 canonical beacons
at MCS7, full duty, ch36 (a busy neighbour BSS on the channel), counted
frame-exactly by an 8812EU `rxdemo` witness (`DEVOURER_STREAM_OUT=1`, the
same witness binary for every arm):

| arm | witness-decoded of 8050 submitted | mean |
|---|---|---|
| upstream master | 7493 7546 7682 7565 7550 7383 7659 | 7554 (93.8%) |
| this fix | 7194 7373 7398 7437 7451 7487 7483 | 7403 (92.0%) |
| fix, `MAC_TRX_ENABLE` back to `0x0F` | 7471 7601 7664 | 7579 (94.1%) |
| fix, sends forced to the first endpoint | 7385 7446 7499 | 7443 (92.5%) |

Submission was identical in every run (8050, 0 failed, ~6.31 s). The fix
decodes ~1.9 points fewer, and reverting the `REG_CR` enable alone restores
master's figure while reverting the endpoint selection does not - so the
cost, if real, is the protocol engine the fix needs. Read it for what it is:
seven runs per main arm and three per variant, one TX, one witness, one
channel, inside the ~3-point single-probe band this bench measures
(`tests/probe_repeatability.sh`) though consistent in sign. The mechanism is
not known (deferral to the neighbour BSS is the obvious candidate and was
not measured). An SDR duty read (`tests/bench_onair.py`) on a quiet channel
is the measurement that would settle it. (The "fix" arms ran on the
station-mode branch, which carries this fix plus the station code that comes
later; it was not re-run on this PR's tree alone.)

## The on-air regression set for any change here

Available with this PR:

- **The in-tree reproducer** - `txdemo` with a hardware beacon armed,
  sending data-sized QoS-Data frames with the RX loop running:

  ```sh
  DEVOURER_TX_BEACON_TU=25 DEVOURER_TX_FRAMES=8000 DEVOURER_TX_GAP_US=0 \
  DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_PAYLOAD_BYTES=1400 \
  DEVOURER_TX_WITH_RX=thread build/txdemo
  ```

  What `DEVOURER_TX_BEACON_TU` does: its comment in `examples/tx/main.cpp`.
  It arms on Jaguar2 and Jaguar3 only - the dies with this defect. Elsewhere
  it warns and runs unbeaconed: Jaguar1's `StartBeacon` reports success even
  when its closing `PinBeaconTbtt(0)` re-download fails, so the arm cannot be
  confirmed there, and Kestrel has no `StopBeacon` at all (both
  pre-existing).

  Measured 2026-09-27 on an RTL8812CU, ch36, this tree vs the same tree with
  `MAC_TRX_ENABLE = 0x0F`, the adapter re-enumerated before each run:

  | build | runs | result |
  |---|---|---|
  | `0x0F` (unfixed) | 3 | sends start timing out (libusb rc -7, `was_timeout` 1) and txdemo stops on `DEVOURER_TX_MAXFAIL` after 845 / 1760 / 861 submitted, 8 failed each |
  | `0xFF` (this fix) | 3 | 8051 submitted, 0 failed, `txdma_status` 0, each run |

  The verdict is the send failures, not `txdma_status`: on the faulting runs
  the periodic `tx.stats` still read `txdma_status` 0 up to its last sample
  (it is taken once per 500 frames, so a latch just before the stop would
  not show). One adapter, one channel. On the 8822B (8812BU) this txdemo
  form does NOT reproduce - unfixed and fixed alike ran clean (item 3); the
  Jaguar2 verification is the LLT check below. The 8822E form is unmeasured
  (the `ap_wpa2` stress, station-mode PR, is its record). Those runs used a radiotap-prefixed beacon; the demo now passes
  the same MPDU without radiotap, which the Jaguar backends strip anyway,
  so the beacon page they load is byte-identical. Re-run after the review
  round-2 send-path refactor (8812CU, same command): 8051 submitted, 0
  failed; the aggregated path (`DEVOURER_TX_BATCH=4 DEVOURER_TX_USB_AGG=4`)
  0 failed. Re-run after review round 3 (8812CU): the reproducer 8051
  submitted / 0 failed with the beacon armed and then stopped, the
  aggregated path 0 failed, and A-MPDU over QoS data 0 failed.

  **The maintainer's bench (josephnef), 2026-09-27:** the reproducer
  reproduces and the fix clears it on an 8812CU, and the 8812BU LLT check
  gives the same result as above. And a counterpart for `txdma_status`: an
  8812CU on USB2 at `DEVOURER_TX_GAP_US=0` with 1400-byte QoS data read
  `0x2000` (bit 13, `BIT_PAYLOAD_OVF_8822C`) from the first sample, on the
  fixed and the control build alike, while TX completed 8051/8051; at the
  default 2 ms gap it read 0. So a nonzero `txdma_status` is not by itself
  the wedge - bit 18 (`BIT_TXPKTBUF_REQ_ERR`) is the bit measured with it
  (`IRtlRadio::GetTxDmaStatus`).

  **The canonical-frame form does not reproduce**: without
  `DEVOURER_TX_QOS_DATA`/`DEVOURER_TX_PAYLOAD_BYTES`/`DEVOURER_TX_WITH_RX`,
  the unfixed build ran 20051 submitted / 0 failed / `txdma_status` 0, the
  same as the fix (one run each). Data-sized frames with the RX loop running
  are what it needs; which of those matters was not separated.
- `txdemo` without the beacon knob, `DEVOURER_TX_FRAMES=8000
  DEVOURER_TX_GAP_US=0`: every frame must complete and `txdma_status` must
  not latch bit 18 (bit 13 can latch at max duty on USB2 while TX
  continues - see above). This never reproduced the defect - it guards the
  fix against breaking plain TX.
- The injection A/B above: `tests/probe_repeatability.sh` for the floor, and
  the witness count per arm.
- **The in-tree Jaguar2 verification** - the LLT and beacon page, read
  right after a beacon-armed run:

  ```sh
  DEVOURER_PID=0xb812 DEVOURER_CHANNEL=36 DEVOURER_TX_BEACON_TU=25 \
  DEVOURER_TX_FRAMES=2000 DEVOURER_TX_GAP_US=0 DEVOURER_TX_QOS_DATA=1 \
  DEVOURER_TX_PAYLOAD_BYTES=1400 DEVOURER_TX_WITH_RX=thread build/txdemo
  build/chipstate --pid 0xb812 --pktbuf 1938
  ```

  Expected with the fix: `LLT[1937] = 0x000` and TX page 1938 starting with
  the beacon descriptor (`38 00 30 81 00 10 00 00 ...` on the 8812BU).
  Expected with `MAC_TRX_ENABLE = 0x0F`: `LLT[1937] = 0x792` (the ring not
  terminated) and page 1938 not holding the beacon (`00 00 00 00 ...` on the
  8812BU). Measured 2 runs per arm (item 3). The read is only meaningful
  after a run past one traversal - `LLT[1937]` reads `0x792` at init either
  way - and only where the chip stays up after the session: Jaguar2 (no
  teardown power-down yet). Jaguar3's `Stop()` powers the chip down, so
  there it reads the power-down fill; `--mac-dump` adds the MAC registers.

**Lands with the station-mode PR - not runnable on this branch:**

```sh
sudo TX_RATE=MCS7 tests/sta_d2d_onair.sh beacons   # 3/3; ours 100% under downlink
sudo TX_RATE=MCS7 tests/sta_d2d_onair.sh thru      # both directions ~20 Mbit/s
sudo TX_RATE=MCS7 tests/sta_d2d_onair.sh flood     # 3/3; books close; ~350 rt/s
```

And, also with the station-mode PR, the `ap_wpa2` stress the figures above
were measured with: `DEVOURER_AP_INJECT=4000 DEVOURER_AP_PKTBUF=1` on
`tests/ap_wpa2.cpp` (neither knob exists in this branch's `ap_wpa2`) - zero
send failures, no `TXDMA_STATUS` transition, and in the `end of run` probe
`LLT[1937]=0` (written by the hardware during the run - it reads `0x792` at
init, correctly) with page 1938 still `59 00 30 85 ...`. The probe reads
around page 1938 (`DEVOURER_AP_PKTBUF_BNDY=N` for another die's boundary; all
three measured dies - 8822C, 8822E, 8822B - use 1938) through
`IRtlRadio::ReadPacketBuffer`, which this PR carries.

## The answer-key technique

What broke this open was running the **vendor driver on the same adapter** and
reading its live state: `/proc/net/rtl88x2cu/<if>/mac_reg_dump` (our side:
`IRtlRadio::DumpMacRegisters`, whose declaration gives the ranges and the
diff recipe), `fifo_dump` (`echo "<sel> <hex off> <size>"`, sel 0 =
TX FIFO, 4 = LLT - `ReadPacketBuffer` is our side of it), `read_reg`, plus
usbmon for its descriptors. A state diff cannot see a bit that bring-up sets
late but needed early; the `REG_CR` difference was found by diffing register
WRITES from usbmon, from the TRX enable to the LLT init. The vendor driver
cannot move its phy into a netns (`-95`), so run hostapd in the root
namespace and put the devourer station in the netns instead.
