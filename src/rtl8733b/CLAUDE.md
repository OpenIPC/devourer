# src/rtl8733b/ — RTL8733B (HALMAC 87xx, 11n) working context

Deep per-backend facts for this subtree, loaded alongside the root CLAUDE.md.
Chips: RTL8731BU / RTL8733BU, chip-id `0x16`, USB PIDs `0bda:f72b` and
`0bda:b733`. 1T1R 802.11b/g/n.

This is **not** a Jaguar variant. HALMAC 87xx has its own power sequence,
firmware layout, MAC register map, descriptor geometry and PHY path, which is
why it lives outside `src/jaguar*/` instead of behind a strategy seam in one of
them. Do not reach for a Jaguar file expecting a shared mechanism.

## HAL layout

`Rtl8733bDevice` (the `IRadio` boundary), `Rtl8733bBringup` (card
enable/disable power sequence, system-cfg), `Halmac8733bMac` (MAC init,
firmware download, EFUSE read + packed-map decode, monitor RX config),
`Phy8733b` (BB/RF table apply, channel plan, TXAGC, TSSI), plus the header-only
`FrameParser8733b` (RX descriptor + PHY status), `TxDescriptor8733b` (40-byte
TX descriptor encoder and the modulation-admission predicates) and
`Rtl8733bUsbIds`.

BB/AGC/RF tables ride the shared `PhyTableLoader` (`check_positive` walker)
with a Jaguar-style context; firmware and tables are generated into
`hal/hal8733b_{fw,tables}.c` — edit `tools/extract_8733b_*.py`, never the
output.

## Dispatch

Chip-id first: `SYS_CFG2` must read `0x16`. The PID table exists for discovery
and for one safety property — a device whose PID is a known 8733B identity but
whose chip-id read fails is **refused**, not allowed to fall through to the
Jaguar1 default. A wrong-HAL bring-up on this part writes an
unrelated register map.

## Chip facts

- **40-byte TX descriptor**, not the 48-byte form its 8822B/8822C neighbours
  use. Encoded byte-wise because USB buffers are not guaranteed
  `uint32_t`-aligned. The checksum is an XOR fold over the first 32 bytes with
  the `0x1c` field skipped, stored inverted — so a valid descriptor folds to
  `0xffff`, the opposite polarity to some sibling parts. Get this wrong and the
  chip silently drops the frame.
- **RX aggregation is capped at 12 KiB** (`kRxAggregateBytes8733b`, HALMAC
  `RXDMA_AGG` size field in 4 KiB pages). The vendor default of 20 KiB exceeds
  one bulk-IN URB and was observed being split by xHCI, leaving a descriptor
  tail and its body in separate completions. The RX loop floors its URB size at
  the same constant and a `static_assert` ties the two together — raising
  either alone reintroduces the straddle, from opposite sides.
- **TSSI closed loop.** Power runs from a safe ceiling
  (`kSafeTssiTargetQdbm8733b` = 16 dBm). On a TSSI-offset PG unit the loop
  **is** the TX-power control, so it is not
  optional there — an attempt to make it opt-in with a fall back to the flat
  `kSafeTxAgcIndex8733b` could not carry HT at all (witnessed: MCS7, 300/300
  submitted, 0 captured, twice). A unit whose EFUSE carries no TSSI calibration
  has nothing to drive the loop and takes the flat path.
- **The runtime TX-power lever is that loop's target, and only the relative
  knob is ported.** `SetTxPowerOffsetQdb` shifts every per-rate target below the
  ceiling (`tssi_rate_offsets`: cap first, then shift — a lowered ceiling would
  move only the rates above it and flatten the calibrated spread). The write is
  the five packed dwords at `0x3a00..0x3a10`, rewritten **in place with tracking
  live**, the same #389 shape `fast_retune` uses — not the ~165 ms
  disable/re-enable pair. Caps therefore report the dBm model (`index_max = 0`,
  one qdB per step) over `[-128, +127]` — the int8 delta field at both ends,
  the same clamped-only-at-the-hardware-rail answer Jaguar1/3 give. The 0x3a00
  bytes are a signed offset from the 64 qdBm anchor, so that field spans
  targets from −16 to +47.75 dBm. Neither end is re-clamped at something
  softer: not at the PG table on the way up (an EFUSE trimmed too cold is what
  an operator calibrates their way out of), and not at a 0 dBm target on the
  way down (an earlier cut did, and it cost ~9 dB of working backoff — see
  below). Where the chip stops *responding* is measured and documented, not
  enforced.
  `SetTxPowerIndexOverride`, `SetTxPowerRateDiffs` and `ReApplyTxPower` stay
  unported.
- **The backoff floor is ~−96 qdB, not the 0 dBm target.** Sweeping past the
  old −64 clamp (MCS0, ch36): the 0 dBm target read 62.07, and −80 qdB
  (−4 dBm) read **54.80 — another 7.3 dB down, with no sign wrap**. It pins
  from about −96 qdB (−8 dBm): 52.96 / 52.97 / 52.99 at −96 / −112 / −128,
  flat within 0.03 dB. So the usable travel is ~23 dB below the 16 dBm clip,
  not the ~16 dB the first cut allowed, and EVM only softens from −59 to −53
  across it. The lesson is the one this whole knob is about: the earlier floor
  was a guess about what a negative absolute target *must* mean, and the guess
  was worth 9 dB.
- **Overdrive above the clip buys ~3 dB and then the PA compresses, and EVM is
  the only tell.** Sweeping UP from the clip (MCS0, ch36, witness reporting EVM
  beside RSSI): +16 qdB — the top of the PG table — gave +2.8 dB with EVM
  already down from −62 to −50; **+32 qdB read 8.7 dB louder with EVM collapsed
  to −18**, and +48/+64 changed nothing at all (RSSI and EVM both pinned). More
  energy, unusable constellation. **SNR held 58..64 throughout and never saw
  it** — precisely the case `docs/bench-testing-near-field.md` is about. So the
  counterpart to leaving the range open: the vendor's PG table lands about
  where this part stops being linear, and +16 qdB is the edge of *usable*
  overdrive even though the field allows +127. Below the clip EVM stays flat at
  −58..−61 across all 16 dB of backoff — the negative half is clean.
- **The lever is worth ~14 dB, and it compresses at the bottom.** On-air
  against an RTL8812AU witness (chip-RSSI ground station, the
  `tests/txpwr_offset_onair.sh` method), two independent 6-point passes at
  ch36: monotone, 14.2 / 14.8 dB of received power for the full 16 dB of
  command, overall slope 0.222 / 0.231 dB per qdB against the 0.25 nominal.
  The counterpart in the same breath: the step is **not** constant. The bottom
  12 qdB delivered 0.125 and 0.126 dB/qdB — the one structure that reproduced
  exactly across both passes — while everything above −52 qdB ran 0.233..0.242,
  and the mid-range scattered 0.219..0.252 between passes. So the loop gives
  about half the commanded dB as its target nears 0 qdBm, while
  `saturated_low` still reads false (that clamp only fires at −64).
  `step_measured` stays false for exactly that reason, the same call the 8822E
  gets: calibrate your own dB-per-qdB or lean on the ground's RSSI. One unit,
  one witness, near-field, integer-quantised RSSI, no SDR.
  The pre-change binary measured **flat in the same session and geometry**
  (0.3 dB across the same 64 qdB) — the do-nothing control that makes the
  14 dB readable.
- **USB TX aggregation is ported, and this is the family it matters most on.**
  The mechanism (block count placement, the checksum-ordering constraint, the
  URB-vs-frame accounting) is doc-commented at
  `TxDescriptor8733b.h`'s `agg_num` and `Rtl8733bDevice::send_packets` — read
  it there. What only lives here:
  - **Bring-up needed no change.** MAC init *already* programmed
    `BLK_DESC_NUM = 3` into `DWBCN0_CTRL[7:4]` (0x0208), the same field and
    value the 88xx siblings use. The port was two things — a descriptor field
    and the packer — because of that.
  - **Why it is worth having here specifically.** One bulk submission costs
    **~248 µs of CPU on the CV610 craft** against ~22 µs on x86, ~87% of it
    the kernel USB path. Craft A/B at ~1750 fps: **248 → 148 µs per frame,
    43.0 → 26.7% of one core**, frame rate unchanged. The counterparts, in the
    same breath: it buys **nothing on air** — same frames, same airtime, purely
    host CPU; it buys nothing at all unless the caller uses `send_packets`
    (waybeam-link does not); and the win shrinks with the host, being only
    ~11 µs/frame on x86.
  - **Frame counts cannot verify it.** The 8822BU precedent is that wrong
    packing makes the TXDMA re-air block 1 `agg_num` times, and `rx_hits` is
    identical either way — it was 23900 in both arms here. Only per-frame
    stamps discriminate (`DEVOURER_TX_QOS_DATA` + `DEVOURER_RX_PCTR`, count
    distinct `pctr`): **ratio 1.00 = distinct, ~3.00 = re-airing**. Verified
    1.00 with the boundary shim both off and on — and the shim needs a payload
    chosen for it (MPDU length ≡ 472 mod 512 at USB HS), because a fixed-payload
    sweep never reaches it.
- **The thermal table is chosen once per channel set, not per frame.** The CCK
  and OFDM/HT variants of the thermal-compensation table are different tables.
  `configure_tx_power` picks one from the configured TX mode and leaves it,
  which is what the vendor does: `_halrf_tssi_set_tmeter_tbl_8733b` is
  reachable only from full TSSI setup, keyed on `phydm_get_tx_rate` at that
  instant, and never re-selected at runtime. So nothing reads or writes a
  register per frame on the send path, matching the other four HALs.
- **The curve choice is inert at any temperature this part reaches, so do not
  reintroduce runtime switching.** The CCK and OFDM/HT tables are
  bit-identical for thermal deltas 0..+17 and first differ at **+18** (the
  swing ramp starts there; the words either side of the baseline are zero in
  both). A five-minute max-duty MCS7 soak on the validation unit plateaued at
  **+8** after two minutes and stopped climbing — less than half the delta
  needed for the tables to differ by a single entry. A runtime switch
  therefore costs **84 ms / 136 USB register round trips** per crossing
  (OpenIPC/devourer#389) to install a table that is bit-identical to the one
  already loaded. An opt-in knob for it was written, measured and deleted on
  that evidence. If a future board reaches +18 — a sealed module at high
  ambient might — the switch is worth revisiting, but implement it as the
  in-place rewrite validated in #389 (13.8 ms, tracking left enabled), not the
  teardown/rebuild.
- **The loop needs settling time, so a fast rate-switching run misreports
  power.** Alternating CCK and OFDM at a few ms per frame leaves CCK
  transmitting above its settled level until the loop converges; pacing the
  same stream to tens of ms per frame lands on the settled value. Pace
  single-rate and let it settle before reading any power figure — otherwise
  the tracking loop is what is being measured. The magnitude of the
  overshoot has not been characterized (the SDR campaigns so far ran
  single-rate, per this very rule); it is not a number to quote.
- **Thermal is telemetry, like everywhere else.** `InitWrite` logs one
  bring-up snapshot and `GetThermalStatus` serves the caller's own cadence.
  Nothing in the send path reads it: measured at 2.51 ms of a 2.71 ms
  per-frame budget on USB high speed, for a meter that tracks PA bias rather
  than junction temperature.

The timing figures above (84 ms, ~11 fps, 2.51/2.71 ms) come from the original
`f72b` validation unit and have no in-repo oracle. They are register-sequence
and USB-transfer costs. Treat them as the order of magnitude to design
against, not as constants. (RF-domain quantities are measured on the
`b733` sample — see Validation status.)
- **EFUSE**: physical packed map walked into a logical map. Running off the end
  of the readable span is address-space exhaustion, not corruption — a fully
  written map has no `0xff` terminator left to find, so that case returns
  success-with-whatever-parsed. Truncated headers and out-of-range block
  targets are still hard rejections.
- **Power sequencing**: the teardown flag is armed *before* `power_on()` is
  attempted, because the sequence can fail at any poll with several card-enable
  writes already landed. `power_off()` is the full card-disable flow and is
  safe against a partially enabled card; latching the flag on success would
  skip rollback in exactly the case that needs it.

## Admission contract

What is admitted and why each refusal exists is doc-commented at the predicates
themselves in `TxDescriptor8733b.h` — read it there, not a copy here. Two
things that header cannot tell you:

- **The predicates are not uniformly shared.** `ht_request_supported_8733b`
  gates both the `SetTxMode` and radiotap MCS branches, so the HT contract
  genuinely cannot drift. `legacy_request_supported_8733b` is reached only
  through `SetTxMode`; the radiotap `RATE` branch takes the byte as given and
  is constrained downstream by `valid_tx_desc_config`'s `rate_id <= 0x1f`
  range check instead. Adequate today only because the radiotap RATE field
  cannot express SGI/LDPC/STBC — do not assume the two legacy paths are kept
  in step by construction.
- **The CCK band gate closes end-to-end**, not in two halves:
  `tx_rate_id_8733b` returns a `0xff` sentinel off 2.4 GHz/20 MHz, and that
  same range check refuses the sentinel rather than encoding it.

**SGI airs, and the old refusal measured the witness.** The record said a
descriptor-SGI probe decoded as long GI at an RTL8812AU; the descriptor bit
(dword5[4]) is exactly the vendor's injection placement and nothing in the
vendor tree enables SGI anywhere else, so the air was measured decode-free
instead: SDR burst-length histogram (`tests/sdr_burst_len.py`, ch36, 1500 B
frames, 4 ms gap), b733 unit against an RTL8822BU control transmitter.
MCS0/20: 1890 us bin long GI -> 1700 us short GI on both parts (computed
1920/1732 for 471 data symbols at 4.0/3.6 us); MCS7/20: 220 -> 200 us on
both. An RTL8822BU witness then decoded 8733B MCS7/20/SGI at 98.6% (1987/2015
distinct payload counters) against 96.9% (1953/2015) long GI, every frame at
rate index 19. SGI is therefore admitted for HT (`ht_request_supported_8733b`)
and advertised (`TxCaps.sgi_ok`). What the witnesses here cannot do is
*report* GI — only the Jaguar1 RX descriptor path exposes it, and no Jaguar1
unit was on the bench — so the per-frame `rx.frame.sgi` cross-check remains
unrun; the SDR shape is the primary evidence and does not depend on any
parser. One unit, one control, one witness; 2.4 GHz SGI unmeasured.

**LDPC TX airs; the global enable was the blocker, and RX-LDPC is absent.**
The descriptor bit (dword5[7]) is the vendor's placement, and it is gated by
`REG_BF_TIMEOUT_EN` 0x1430[8] (`BIT_EN_HT_LDPC_8733B`): MAC init leaves
0x1430 = 0x0300, the vendor's default build never touches it (its clear sits
under `CONFIG_TWO_MAC_DRIVER`), and the first port cleared both bits on the
belief that raw HT only worked that way. Four cells, b733 → RTL8822BU
witness (`rx.frame.ldpc` per frame), MCS7/20, 2015 frames each, ch36:

| 0x1430[8] | descriptor LDPC | delivered | witness LDPC flag |
|---|---|---|---|
| cleared (old port) | 0 | 98.5% | 0 on all |
| set (vendor) | 0 | 98.5% | 0 on all |
| set (vendor) | 1 | 98.0% | **1 on all** |
| cleared | 1 | 98.2% | 0 on all — the bit is silently ignored |

So BCC is unaffected by the enable, the enable is what makes the descriptor
bit real, and the register now stays at the vendor value (the MAC readback
check requires it). The counterparts: the other direction is dead — the same
b733 decoded **0 of 1542** MCS7/20/LDPC frames from the 8822BU against
1435/1542 (93%) BCC in the same shape, so `ldpc_rx_*` stay false and the
vendor's TX-only LDPC declaration is confirmed rather than inherited. And
the coding gain is **not measured**: a three-point TX-power-offset probe
(0 / −64 / −96 qdB, the loop's floor) kept both codings at 79–89% delivery
near-field, with BCC-vs-LDPC differences of −7 and +3 points — inside the
±3-point single-probe noise this repo warns about. `tests/ldpc_waterfall.sh`
cannot run here as written (it steps the flat TXAGC index this family
refuses), and a near-field bench has no waterfall to trace anyway; the
"+3 dB at the 10% crossing" figure from the Jaguars is not claimed for this
die.

**STBC stays refused, with the vendor's reason.** `core/rtw_xmit.c` leaves
`pattrib->stbc` unset on injected frames with the comment "873xb has only
1tx; enable it will cause irregular TX spectrum"; a 1T1R part has nothing to
space-time code across, and the encoder writes no STBC field.

## FastRetune

Ported: intra-band, same-width hops (`Phy8733b::fast_retune`). The full
`SetMonitorChannel` on this USB-HS part costs ~330-440 ms — profiled as
~165 ms TSSI disable/re-enable, ~90 ms band+bandwidth switches, ~60 ms
channel switch — and the fast path keeps only what a hop needs: the RF18
synth program (bandwidth bits preserved via a compose cache primed on the
first fast hop after a full set), RF19 sub-band bits and the channel-keyed
BB constants on bucket change only, then BB reset + IGI toggle. **TSSI
tracking stays enabled across the hop**, with the per-channel rate-offset
dwords AND the channel-bucketed TSSI-DE offsets rewritten in place when
their plans differ — the in-place shape #389 validated; every full set
still runs the disable/re-enable pair. The DE rewrite is the one that
matters intra-band: the rate offsets are band-keyed and never change
across a 2.4 GHz hop, while the DE buckets are ~3 channels wide
(boundaries at ch 2/5/8/11/14, trim at 7/14), so most hops cross one —
skipping them leaves the loop tracking with the previous channel's
calibration (found by review on the first cut of this port). The rewrite
replays prepare_tssi_offsets' field sequence minus its tracking-disable
write; measured across 22 ch1→ch13 hops on the validation unit: readback
parity 22/22, tracking-enable field stayed 7 throughout, settle p50
unchanged (10.4 ms). Measured
on the validation unit (20-cycle settle harness, 1 kHz witness emitter):
call ~55 ms, radio-live 10.0 ms p50 from hop start (min 3.6 / p90 12.9 /
**max 40.3 ms — a 1-in-20 tail, not noise**), vs 70-100 ms radio-live
through the full path. Channel-state readback parity held 7/7 hops, and a
300-frame post-hop burst decoded 299/300 at an independent witness — the
TSSI-live claim is air-verified. The counterparts: one physical unit, like
every on-air claim in this subtree; no SDR, so radiated power ACROSS a hop
is uncharacterized; and cross-band or width-change hops decline (chip
untouched) and fall back to the full path.

## Not ported

`ReadTsf`/beacons, `FastSetBandwidth`, the flat-index / per-rate-diff TX-power knobs
(`SetTxPowerIndexOverride`, `SetTxPowerRateDiffs`, `ReApplyTxPower` — only the
relative `SetTxPowerOffsetQdb` is ported) and `rx.path` per-chain telemetry.
These inherit `IRadio`'s not-ported defaults (`false`, `0`, or a full-path
fallback) rather than being faked.

The TX-power knobs are the exceptions to the silent-default rule:

- `SetTxPowerOffsetQdb` **refuses loudly and returns 0** on a unit whose EFUSE
  carries no TSSI calibration, rather than reporting a successful zero-offset
  apply. That indistinguishability is what this knob exists to end — a consumer
  measured 18 dB of commanded offset moving nothing while its state read
  `{"applied_qdb":0,"saturated_low":false}`, which is exactly what a healthy
  actuator with travel remaining looks like. An offset latched *before*
  bring-up on such a unit is **dropped loudly and zeroed** by
  `configure_tx_power`, so the reported state never claims an offset no
  register carries. (That path has no hardware coverage — every unit seen so
  far is TSSI-offset PG — but it writes no registers, only a log and a reset.)
- `SetTxPowerIndexOverride` is overridden **solely to log a refusal**. The
  `IRadio` default returns `void` and ignores the value, so silence would
  be the caller's only answer on the one backend where the flat index really is
  unported — a knob that looks granted, in the PR that exists to abolish them.
  `SetTxPowerRateDiffs` needs no such override: its `false` return already says
  it.

## Carrier-sense gates

`SetCcaGates` / `GetCcaGates` / `SetCcaMode` are ported on the Jaguar3 recipe,
and the register map is not a guess: `halmac_bit_8733b.h` names `0x0520[14]`
`BIT_DIS_CCA_8733B`, `[15]` `BIT_DIS_EDCCA_8733B` and `0x0524[11]`
`BIT_EDCCA_MSK_CNTDOWN_EN_8733B`, and the shipped MAC init table writes
`0x520 = 0x6f` (both gates ON). MAC bits only — the vendor's BB CCA-off
writes (`0x1d58[11:3]`, and this die's own `0x2a24[13]` CCK-CCA bit, which
`Phy8733b` toggles only around a band switch) stop detection and deafen the
RX. `tests/cca_gates_regcheck.sh` passes every cell on the b733 (api, regs,
cntdown, legacy, retune + fast-retune within and across bands; the `track`
cell skips — no EDCCA tracker runs here). Bring-up resets the pair to
`tuning.disable_cca` (MAC init rewrites 0x520), retunes re-assert it.

On air, this family agrees with Jaguar3 about which bit matters. The b733
injecting MCS3 at maximum duty on ch36, host submit rate in fps over the
~5 s TX window, three repeats each:

| cell | gates on | primary `[14]` off | EDCCA `[15]` off | both off |
|---|---|---|---|---|
| alone (this bench's ch36 ambient) | 3008 / 2938 / 3015 | 6903 / 6914 / 7021 | — | — |
| co-channel 8822BU 6M flooder | 4001 / 4159 / 4079 | 7148 / 7237 / 7127 | 4100 / 4140 / 4112 | 7206 / 7131 / 7162 |
| far-channel (ch149) flooder | 2920 / 3011 / 2911 | 7147 / 7036 / 7080 | 3043 / 2942 / 2926 | — |

Three things to read off it, with their counterparts. "Alone" is not an idle
channel: this bench's ch36 carries a resident network, and carrier-sense
costs the injector 57% of its rate against it, which is why the primary gate
buys 2.3× with no flooder of ours running (one earlier alone cell read
6934 fps — ambient varies, and a single alone reading is not a baseline).
Against the co-channel flooder the injector keeps 57% with the gates on and
recovers to the gates-off ceiling (~7.15k) with `[14]` cleared, 1.75×. The
far-channel flooder costs nothing beyond ambient (2.95k vs 3.0k alone), so
there is no USB-contention term to subtract on this pairing. EDCCA alone is
null in every arm. One unit, one flooder, one channel; the RX-decode side with
the gates off decodes the same flood as with them on (`DEVOURER_DIS_CCA=1`
rxdemo cell, below), so the MAC-bits-only rule holds here as on Jaguar3.

## Hardware ARQ

All three hardware-ARQ knobs — the ACK responder, the retry limit and the
per-frame CCX `tx.report` — are measured true on this die.

The detailed ARQ characterization below used the original `f72b` RTL8731BU,
an RTL8812AU peer, and an RTL8812CU passive witness. The live responder-disarm
cell was independently repeated on the `b733` sample by the maintainer and on
a second vehicle-mounted `f72b`; both agreed. The vendor-driver control
(`docs/rtl8733b.md` "Vendor-driver control") covers raw TX/RX decode, not
these ARQ rows, which remain single-bench results rather than population
qualification. Witness copy counts are conservative observations because a
passive monitor can miss airings.

`DeviceConfig::tx::ack_timeout_us` is honoured. The field's contract — range,
clamp, default, registers and range budget — is doc-commented at its
declaration in `src/DeviceConfig.h`. What is specific to this backend:
`init_wmac()` writes vendor defaults first, then `bring_up_to_phy` overwrites
and verifies both REG_ACKTO 0x0640 (OFDM/HT) and REG_ACKTO_CCK 0x0639. A
failure or mismatched readback aborts bring-up instead of reporting a partly
applied range knob. Both registers read back 33 and 200 in direct runs. At 11M
CCK to a dead RA, retry 8 and maximum duty, an 8-second run submitted 1513
frames at 33 µs versus 1129 at 200 µs. That establishes the expected timing
direction; it is not a precise calibration of the register's timebase.

**`SetAckResponder` is ported and measured.** It writes the shared recipe's
three registers — not an assumed map, since the vendor's own port-0 descriptor
names them (`hal/rtl8733b/rtl8733b_ops.c` `port_cfg[0]`:
net_type `REG_CR_8733B + 2` = 0x0102 shift 0, macaddr 0x0610, bssid 0x0618).
Its behavior is deliberately not described as the generic gate story: MAC
bring-up leaves net_type at NoLink and programs the EFUSE MAC into MACID, and
NoLink does not make this die passive. The authoritative behavior and numbers
live in `src/AckResponder.h` and `src/AdapterCaps.h`.

Three properties of the port worth knowing, none of them local inventions: the
arm refuses a group MAC and is read back before it is reported, both through
the shared `ack::is_unicast` / `ack::verify` beside `enable()` — the register
map lives in one file, so no backend carries a copy that can drift from it. A
config-driven arm that fails **fails the bring-up** rather than handing back a
session that quietly answers nothing. And the disarm is unconditional inside
`Halmac8733bMac::stop()`, not a flag-guarded special case at the device layer:
`stop()` clears only REG_CR's low half, so net_type at 0x0102 survives it, and
siting the clear there means no future path can reach `stop()` and leave an
unowned SIFS-timed transmitter on the air. After an armed session ends with
`teardown_power_down` off, the peer reads ack_rate 0.00 with retries pinned at
12 — that silence comes from `stop()`'s RCR and CR writes taking the MAC down,
not from the net_type clear, which is inert on this die (`src/AckResponder.h`
retarget(), `Rtl8733bDevice::disarm_ack_responder`).

Responder capability and its measured limits live in `src/AdapterCaps.h`
(`ack_responder_ok`); the arm/disarm recipe and why this die needs a retarget
live in `src/AckResponder.h` and `Rtl8733bDevice::ClearAckResponder`. Read
those rather than a copy here. The harness is
`tests/ack_txreport_matrix.sh` with the 8733B as RESPONDER; its `disarmed`
phase is the only cell that measures a DISARM (it arms, then disarms mid-run
via the RTL8733B-only `DEVOURER_ACK_DISARM_AFTER_MS` hook after completed
bring-up), because its `off` cell never arms at all.

Soliciting-TX ACK recognition is measured independently.
`tests/rtl8733b_arq_tx_onair.sh` puts the RTL8733B in the soliciting-TX role,
the RTL8812AU in the responder role, and uses an RTL8812CU only as a passive
payload-counter witness. At MCS3, responder on/off produced 1.032/12.948
copies per observed frame with 99.7/100% coverage. At 11M CCK the result was
1.002/11.908 with 95.8/92.3% coverage. This proves that the RTL8733B recognizes
a real ACK and stops autonomous retry in these normal-ACK cells; it does not
substitute for the unavailable per-frame delivery report.

**BlockAck response is measured without CCX.** Per-frame CCX accounting is not
a valid retry oracle under A-MPDU (`docs/aggregation.md`), so
`tests/rtl8733b_blockack_onair.sh` uses a Jaguar2 `0bda:b812` TX, this RTL8733B
as responder, and a Jaguar1 `0bda:8812` passive witness. At ch36/MCS3 with
retry limit 12, the fully initialized but unarmed control produced 1,605 unique
payloads at 12.720 copies/payload and zero matching BlockAck frames; armed
produced 128,702 at 1.001 plus 14,402 addressed `0x94` BlockAcks, every one
carrying a nonzero bitmap. Both arms were real A-MPDUs (`paggr` 0.665/1.000,
max burst 9). The control-frame event checks RA=the Jaguar2 TA and TA=the
configured RTL8733B MAC, so ambient BlockAcks do not count. This establishes
the RTL8733B responder on that measured combination only.

**`tx.retry_limit` drives real autonomous retransmission** — `tx_retry_limit_ok`
is true. `tests/rtl8733b_retry_limit_onair.sh` judges from the air because this
die has no TX-side CCX reports: unicast to an unowned RA ensures no ACK ever
returns, while a passive monitor counts clean airings per submitted frame. A
dose-response rather than an on/off pair gives multi-level evidence. The
harness uses a run-specific unicast SA, counts every clean payload-counter
event, refuses fewer than three distinct levels or a missing zero baseline,
and rejects partial runs. Measured 0 -> 1.00, 3 -> 4.00, 12 -> 12.32–12.33
copies/frame against expected 1 + N, repeatable across a 0/3/12/0/12 ladder.
The retry-12 shortfall from the ideal 13 may be passive-monitor loss or
genuinely fewer airings, so the ratio is reported as an observation, not an
exact hardware count.

**CCX `tx.report` is ported, and the thing that hid it was the bring-up, not
the firmware.** The descriptor stamps are the generic halmac pair the 8733B
macros alias (`SPE_RPT` dword2[19], `SW_DEFINE` dword6[11:0], inside the
checksummed span — `TxDescriptor8733b.h`), the report is the fw-offload C2H pkt
`0xFF`/`0x0F` that `src/TxReport.h`'s `parse_ccx_halmac` already decodes, and
it rides bulk-IN like every frame, so the RX loop hands it to `DeliverTxReport`
when the descriptor's C2H bit is set. None of that was ever the gap. The gap
was `download_firmware` skipping the download when `MCUFW_CTRL` said a firmware
was already running: that image answers the ready poll and consumes H2C, but
emits **no C2H at all**. Measured on the b733 unit, MCS3 unicast, ~950 frames
per 8 s cell, RTL8822BU responder: after a download 920-970 reports per cell;
on a reused image 0, in every arm — including arms that first cleared the
0x0420 report gates and sent MEDIA_STATUS_RPT. The vendor reloads at every hal
init, and so does this backend now (~200 ms of a ~2.8 s USB-HS bring-up).

Two firmware facts that fell out of the same bench, both measured rather than
read:

- **MEDIA_STATUS_RPT (H2C 0x01) is not a prerequisite.** With a fresh image the
  reports flow for the descriptor MACID whether or not the firmware was told
  the MACID is a connected station (arm with neither mechanism: 920/935).
  The mailbox itself is ported (`Halmac8733bMac::send_h2c`, the legacy HMEBOX
  path at 0x01CC/0x01D0/0x01F0 the vendor USB driver uses — its halmac
  packet-queue H2C is SDIO-only) and `rtl8733bprobe`'s `fw` stage uses a
  consumed MEDIA_STATUS_RPT as the MCU-liveness check; the bring-up sends no
  H2C.
- **`REG_FWHW_TXQ_CTRL` 0x0420 bit 18 (`BIT_TXFAILRPT_DIS`, a bit the 88xx
  siblings' maps do not name) selects the report shape.** MAC init leaves it
  set, and the firmware then emits one terminal report per frame — the
  Jaguar2/3 shape (`state` 0 on ACK, 1 with `retries` at the limit on a dead
  peer). Clearing it makes the firmware report every failed attempt: 12036
  reports for 941 frames to a dead RA at retry limit 12 (one per retry with the
  running count, then the terminal one), while an ACKed stream stays at one
  per frame. Not exposed — a consumer counting reports per frame would be
  misled by it — but it is a real per-attempt retry trace if one is ever
  wanted. Bit 19 (`BIT_TXRPT_DIS`) reads clear after init.

The shipped shape, `tests/ack_txreport_matrix.sh` with the 8733B as the
soliciting TX (`CELLS="8733b:0x0bda:0xb733" RESP_PID=0xb812`, ch36, MCS3,
retry limit 12, 8 s per phase): on 1.00 ACK / 0.01 mean retries, retarget
1.00 / 0.02, off 0.00 ACK with retries pinned 12.0 (pin rate 1.00), report
coverage 0.984-0.985, zero SW_DEFINE tag gaps. `missed` is the constant 4 on
every report, as on the 8812CU — drop detection is the tag, not that field.
One unit, one responder, one bench; the mixed case (a frame ACKed after some
retries) shows up as `retries` 1-4 in the on cells and nothing more was
measured about it.

Consequence a consumer can now plan on: the measured 8733BU is a complete
hardware-ARQ endpoint — responder, autonomous retry, and the per-frame delivery
report, so `TxReport.state == 1` (the peer stopped ACKing) is available here
like on the Jaguars. The reports need an RX loop (`DEVOURER_TX_WITH_RX=thread`
in txdemo), the same as Jaguar1/2.

## A-MPDU TX

`SetAmpduMode` is ported, and the two things that make it work here are
measured rather than copied from Jaguar2. The descriptor half is the generic
halmac placement (data QSEL dword1[12:8], `AGG_EN` dword2[12], density
dword2[22:20], `MAX_AGG_NUM` dword3[21:17]); the QSEL also picks the bulk-OUT
endpoint through `TXDMA_PQ_MAP` (BE/BK → LOW, VI/VO → NORMAL, MGMT → HIGH,
`ep_for_qsel`), as the vendor's `rtw_get_ff_hwaddr` does. On its own that
half does nothing on this die: with `AGG_EN`, a data QSEL, 0x0456 at 0x20,
0x04BC[6] cleared and three frames per URB, the b733 aired every frame as a
single (RTL8812EU witness `paggr` 0.00, control: an RTL8822BU through the
same witness reads `paggr` 1.00), at ~305 fps.

- **The MAC aggregates only frames it numbers itself.** `EN_HWSEQ`
  (dword8[15]), which the vendor sets on every data frame and the inject path
  does not, flipped `paggr` from 0.00 to 1.00 with nothing else changed. So an
  aggregated frame's sequence number is the MAC's SSN counter, not the
  caller's; a consumer that keys on its own sequence numbers must not run
  this mode (the payload counter, `DEVOURER_TX_QOS_DATA` + `DEVOURER_RX_PCTR`,
  is the right ledger for it).
- **Data queues are paced by the EDCA TXOP limit, not by 0x0456/0x04BC.**
  Isolated by QSEL alone (same endpoint, no `AGG_EN`): BE ~310 fps, VO
  ~600 fps, MGMT ~2600 fps — the vendor init's BE parameter 0x005ea42b holds
  each launch for 0x5e × 32 µs ≈ 3.0 ms and VO's 0x2f for 1.5 ms; MACID 0 vs
  1, MEDIA_STATUS_RPT and the endpoint changed nothing. Zeroing the TXOP limit
  took the BE queue to 1446 fps alone and, with `EN_HWSEQ`, to the aggregated
  figures below. `SetAmpduMode` zeroes the TID's AC register (0x0500/04/08/0c)
  and restores it on clear; 0x0456 and 0x04BC are programmed too, measured
  inert here.

Delivered distinct payload counters per second at the RTL8812EU witness,
ch36, 1000 B QoS-Data, no-ack (`0/16/7/1/0x20`), ~9 s cells:

| | singles (MGMT queue) | A-MPDU, 3 frames/URB (`send_packets`) | A-MPDU, depth-1 `send_packet` |
|---|---|---|---|
| MCS7/20 | 2050/s | **3829/s** (+87%, `paggr` 1.00) | 3671/s (+79%) |
| MCS3/20 | 1360/s | **1758/s** (+29%) | — |

Counterparts: one unit, one witness, a busy ch36 (the singles baseline moved
between 2050 and 2700/s across the session, so the gain is the within-run
ratio, not the absolute); the witness reports `paggr` but not burst
boundaries (every MPDU carries its own `tsfl` on the 8812EU), so the
aggregate size is unmeasured; the unicast-ARQ shape (BlockAck responder,
`no_ack=0`) that cost Jaguar2 −8% at MCS3 was not run here; and the depth-1
result means this die aggregates from a sync single-frame feed too, unlike
the Jaguar2 recipe's batched-feed requirement.

## Validation status

On-air claims rest on **two** physical units: the original bare unbranded 1T1R
RTL8731BU module (`0bda:f72b`, cut D, USB high speed, RTL8812AU witness) and a
second sample, the LB-LINK BL-M8733BU2-L combo module (`0bda:b733`, also
cut D), which re-ran the core evidence set — including true VBUS cold boots —
and agreed everywhere. SDR characterization exists for the b733 unit (on-air
throughput, occupied bandwidth at 10/20/40 MHz, stitched spectral mask to
±25 MHz, witness EVM, per-rate level flatness, TSSI-stability soak); absolute
power remains unmeasured — the bench has no calibrated instrument. **Narrowband is 10 MHz only** (caps
contract at its declarations in `src/AdapterCaps.h`): SDR OBW plus two-way
cross-decode with an RTL8812CU narrowband peer on both bands, legacy and HT;
`WIDTH_5` is refused at `channel_plan` because the 5 MHz BB mode airs no
packets across the whole DAC/ADC divider code space, with a
continuous-carrier failure mode observed warm
(`DEVOURER_NB_DAC`/`DEVOURER_NB_ADC` map the codes for any future attempt).
The vendor module builds and runs on the 6.18 host
(`tests/rtl8733b_vendor_build.sh`, `tests/rtl8733b_vendor_ko_smoke.sh`); the
`regress.py` encoding matrices against a devourer RTL8822BU peer are in
`docs/rtl8733b.md` "Vendor-driver control" — RX parity on every encoding on
both bands, including zero HT-LDPC under both drivers. The full
tested/deferred matrix, the provenance pins and the
second `f72b` unit that overheated and was excluded: `docs/rtl8733b.md`.

Headless coverage: `tests/rtl8733b_{efuse,phy_table,rx_parse,tx_desc}_selftest.cpp`
in `ctest`. Hardware: `tests/rtl8733b_lifecycle_soak.sh` (bounded warm
lifecycle, explicitly not a true VBUS cycle) and `examples/rtl8733bprobe`
(staged identity → power/EFUSE → firmware → MAC/PHY → TSSI audit).

## Frame-free sensing: CLM busy airtime, and nothing else

`IRadio::ArmChannelBusy()` / `GetChannelBusy()` work on this die. The vendor
phydm puts the 8733B on JGR3 — at the pinned `reference/rtl8733bu-20230626`,
`hal/phydm/phydm_pre_define.h:513` lists `ODM_RTL8733B` in
`PHYDM_IC_SUPPORT_IFS_CLM`, and `:523-525` define
`PHYDM_IC_JGR3_SERIES_SUPPORT` when `RTL8733B_SUPPORT` is set. The engine
answers on that register map through
the shared `devourer::ClmWindow` — `with_ccx()` in `Rtl8733bDevice.h` lends it
`nhm_regs_jgr3()` under `_reg_mu` then the CCX mutex, exactly as Jaguar2/3 do.

The flag relationship and the armed-only behaviour are the contract, and it
lives at `src/AdapterCaps.h` on `busy_airtime_ok` / `rx_energy_ok` — the one
place it can be kept true. What is specific to this die:

- **Why CLM ports without a phydm block at all.** `arm_clm_only()` /
  `read_clm_only()` take no IGI argument: busy airtime is a hardware tick
  count, not a histogram referenced to the receiver's own noise floor the way
  NHM's thresholds are. That is what makes it separable from FA/CCA here.
- There is no NHM read on this die, so the `Interrupted` spoiler is
  unreachable by construction: nothing can re-arm the shared engine
  mid-window. `Retuned` and `NotElapsed` both fire normally.

Measured with `tests/busy_window_probe.sh` (RTL8733BU sensor, MT7612U flooder,
ch165): **0.0% quiet, 69% under a steady load** with spread 0, and against a
50/450 ms burst (true duty ~9%) a mean of 6-10% over five windows with a
**19-point spread** — a 240 ms window inside a 500 ms burst period misses
whole bursts, so single windows read 0-19% and only the mean is a measurement. A Jaguar3 8812CU read
that same flooder at 69% as well, so the two device paths onto the JGR3 map
agree on one load. The retune and premature-read spoilers each refused with
their reason, and a re-armed window never returned the previous latched value.

**`src/sensing/` reaches this reading through the armed window.**
`SenseWindow` gates its source on `AdapterCaps::rx_energy_ok` (false here) and
arms `ArmChannelBusy` to the dwell the executor carries, so a `chanscout` on
this die records CLM busy airtime per dwell (`src/sensing/CLAUDE.md`).

**The CCA gate can bias the reading.** CLM counts CCA-busy, so a session that
has disabled the gates (`SetCcaGates`, `DEVOURER_DIS_CCA`) may under-report
busy airtime; whether it does on this die is unmeasured. Read the gate state
with `GetCcaGates` beside a CLM figure.

**`Stop()` forgets the window, and that is not obvious.** A stop is not a
retune, so nothing would spoil an armed window — but clearing `_phy_ready`
does not protect it either, because `SetMonitorChannel` and `FastRetune` both
call `bring_up_to_phy()`, which sets that flag true again. Without a reset in
`Stop()`, a window armed before a stop comes back to life on the revived chip
and the retune note hands the caller a `Retuned` spoil earned by a hardware
session that no longer exists: invalid either way, but the reason would be a
lie. The reset is scoped under the `_reg_mu` `Stop()` already holds.

All of that is about a window armed BEFORE the Stop. For an arm issued
*after* one, the flag does its job, and this backend closes both sides of the
hazard: SEQUENTIALLY, `Stop()` clears `_phy_ready`, which is what `with_ccx`
gates on, so a later `ArmChannelBusy()` is refused rather than arming a
torn-down chip; CONCURRENTLY, `Stop()` holds the recursive `_reg_mu` across
its whole body and `with_ccx` takes that lock first, so an arm racing the
teardown blocks instead of slipping in after the reset.

Jaguar1/2/3 reset the window in `Stop()` the same way but stop there — they
have NEITHER half: no `Stop()` clears `_brought_up`, and none holds a
register lock across its teardown (Jaguar1 has no family-wide one at all).
So on those an arm issued after a Stop still succeeds against a torn-down
chip, and one racing the teardown can still slip in. Each Jaguar guide
records its own generation's teardown, the race is in each `Stop()`'s own
comment, and `IRadio::ArmChannelBusy` carries the rule itself.

Retune notes live in `SetMonitorChannel` and `FastRetune`, both **scoped**:
`FastRetune` calls `SetMonitorChannel` on its declined path while already
holding `_reg_mu` (recursive, so that part is fine), and the CCX mutex is NOT
recursive — holding it across the tune would self-deadlock on that path. One
note per site suffices because `_reg_mu` is held across the whole tune and
`with_ccx` takes `_reg_mu` first, so a concurrent arm cannot interleave; that
is the Jaguar2/3 situation, not Jaguar1's.
