# src/rtl8733b/ — RTL8733B (HALMAC 87xx, 11n) working context

Deep per-backend facts for this subtree, loaded alongside the root CLAUDE.md.
Chips: RTL8731BU / RTL8733BU, chip-id `0x16`, USB PIDs `0bda:f72b` and
`0bda:b733`. 1T1R 802.11b/g/n.

This is **not** a Jaguar variant. HALMAC 87xx has its own power sequence,
firmware layout, MAC register map, descriptor geometry and PHY path, which is
why it lives outside `src/jaguar*/` instead of behind a strategy seam in one of
them. Do not reach for a Jaguar file expecting a shared mechanism.

## HAL layout

`Rtl8733bDevice` (the `IRtlDevice` boundary), `Rtl8733bBringup` (card
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
historical Jaguar1 default. A wrong-HAL bring-up on this part writes an
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
  the tracking loop is what is being measured. No SDR has been on this part,
  so the magnitude of the overshoot is not characterized; that is
  SDR-gated work, not a number to quote.
- **Thermal is telemetry, like everywhere else.** `InitWrite` logs one
  bring-up snapshot and `GetThermalStatus` serves the caller's own cadence.
  Nothing in the send path reads it: measured at 2.51 ms of a 2.71 ms
  per-frame budget on USB high speed, for a meter that tracks PA bias rather
  than junction temperature.

The timing figures above (84 ms, ~11 fps, 2.51/2.71 ms) come from the single
validation unit described below and have no in-repo oracle. They are
register-sequence and USB-transfer costs, which is why they are quotable at
all — nothing here asserts an RF-domain quantity, because no SDR has been on
this part. Treat them as the order of magnitude to design against, not as
constants.
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

The SGI refusal is the one worth re-opening: the descriptor bit was set and the
frame submitted, but the witness decoded long GI. Descriptor and air disagree,
and the reason is not yet understood.

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

`ReadTsf`/beacons, hardware ACK/BlockAck, A-MPDU,
`FastSetBandwidth`, the flat-index / per-rate-diff TX-power knobs
(`SetTxPowerIndexOverride`, `SetTxPowerRateDiffs`, `ReApplyTxPower` — only the
relative `SetTxPowerOffsetQdb` is ported), `rx.path` per-chain telemetry,
and CCA disable. These inherit `IRtlDevice`'s not-ported defaults (`false`,
`0`, or a full-path fallback) rather than being faked. `SetCcaMode` is the one
exception to the silent-default rule: it is pure virtual, so `true` throws
loudly — without tearing the session down, since an unported optional knob is
not a hardware-safety event — while `false` succeeds as a no-op because that is
the state MAC bring-up already leaves programmed.

The TX-power knobs are the other exceptions, in the same spirit:

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
  `IRtlDevice` default returns `void` and ignores the value, so silence would
  be the caller's only answer on the one backend where the flat index really is
  unported — a knob that looks granted, in the PR that exists to abolish them.
  `SetTxPowerRateDiffs` needs no such override: its `false` return already says
  it.

`DeviceConfig::tuning::disable_cca` cannot be honoured either, and bring-up
warns rather than dropping it — a config knob must not be the one door where a
request the setter refuses loudly instead vanishes without a word. The warning
sits in `bring_up_to_phy`, not `InitWrite`, so an RX-only session that set the
knob is told too, and so it fires exactly once per bring-up.

## Validation status

On-air claims rest on **two** physical units: the original bare unbranded 1T1R
RTL8731BU module (`0bda:f72b`, cut D, USB high speed, RTL8812AU witness) and a
second sample, the LB-LINK BL-M8733BU2-L combo module (`0bda:b733`, also
cut D), which re-ran the core evidence set — including true VBUS cold boots —
and agreed everywhere. SDR characterization exists for the b733 unit (on-air
throughput, occupied bandwidth at 10/20/40 MHz); spectral mask, EVM and
absolute power remain unmeasured. **Narrowband is 10 MHz only** (caps
contract at its declarations in `src/AdapterCaps.h`): SDR OBW plus two-way
cross-decode with an RTL8812CU narrowband peer on both bands, legacy and HT;
`WIDTH_5` is refused at `channel_plan` because the 5 MHz BB mode airs no
packets across the whole DAC/ADC divider code space, with a
continuous-carrier failure mode observed warm
(`DEVOURER_NB_DAC`/`DEVOURER_NB_ADC` map the codes for any future attempt).
No vendor-driver A/B control was obtained (its module does not build on
modern kernels). The full tested/deferred matrix, the provenance pins and the
second `f72b` unit that overheated and was excluded: `docs/rtl8733b.md`.

Headless coverage: `tests/rtl8733b_{efuse,phy_table,rx_parse,tx_desc}_selftest.cpp`
in `ctest`. Hardware: `tests/rtl8733b_lifecycle_soak.sh` (bounded warm
lifecycle, explicitly not a true VBUS cycle) and `examples/rtl8733bprobe`
(staged identity → power/EFUSE → firmware → MAC/PHY → TSSI audit).
