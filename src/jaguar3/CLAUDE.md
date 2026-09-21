# src/jaguar3/ — Jaguar3 (11ac gen3) working context

Deep per-generation facts for this subtree, loaded alongside the root
CLAUDE.md. Chips: rtl8822c (RTL8812CU/8822CU, chip-id `0x13`) and rtl8822e
(RTL8812EU/8822EU, chip-id `0x17`).

## HAL layout

`RtlJaguar3Device`, `HalJaguar3` (power seq, table apply, 3-wire RF, bf_init,
efuse incl. 8822e OTP burst-mode), `HalmacJaguar3Fw`/`MacInit`,
`RadioManagementJaguar3` (channel/BW/per-path power, the `0x9b0`/`0x9b4`
narrowband dividers, RF18 encoding), strategy interfaces `Jaguar3Calibration`
→ `Halrf8822c`/`Halrf8822e` and `Jaguar3PhyTables` (its own
`PhyTableLoaderJaguar3`, separate from the Jaguar1/2 `PhyTableLoader`).

## Chip facts

- **Coex runtime thread** (`RtlJaguar3Device::coex_runtime_loop`, started in
  `InitWrite`): sustained 5 GHz TX needs it — without its ~2 s WiFi-only coex
  re-apply + FW heartbeats, the combo chip's coex firmware silences the
  antenna. It also drains C2H, so TX-only sessions still see `tx.report`.
- **5/10 MHz narrowband**: the re-clock lives in the `0x9b0`/`0x9b4`
  dividers (vs the `0x8ac` block the Jaguar1/2 chips share). 80 MHz works,
  incl. a 40-in-80 frame via TX-descriptor DATA_SC. See `docs/narrowband.md`.
- halrf calibration: DACK/IQK/TXGAPK/thermal tracking.
- **Firmware channel switch** (H2C 0x1D via `fastretune_fw`, both dies): the
  fw-switch H2C must ride `HalJaguar3::send_h2c_raw`'s HMEBOX box counter —
  the coex runtime thread shares it (both callers hold `_reg_mu`); a second
  counter would corrupt the mailbox rotation. On the 8822C the fw and sw
  fast paths tie on-air (~2.3 ms — RF settle dominates); the fw win is ~3×
  lower per-hop host/USB cost, plus ~2.6 ms cross-band. 8822E spur channels
  decline every fast path, fw included.
- `DEVOURER_TX_WITH_RX=thread` must be set **before** `InitWrite` — the
  bring-up keeps the RX filters open; retrofitting RX later is unreliable.
- The rtl8822e's hardware-bisected constraints (DPDT/pin-mux front end,
  single-path 1SS TX, spur channels, LCK, the 2.4 GHz TX kernel-parity
  limitation) live in `docs/8822e-quirks.md`.

## Bring-up cost and the pipelined register writes

`InitWrite` is ~14k USB register transfers and nothing else. The stage
timing shows it: `init.timing` events under the `j3hal.*` (HAL bring-up)
and `j3init.*` (`InitWrite`) scopes, field schema in `src/InitTimer.h` /
`docs/logging.md`; `bench_init.py` parses them but reports only `ms`. The
batching contract itself — ordering, what waits, single-threadedness,
failure propagation — is documented once, at `ITransport::write_batch_begin`
(`src/Transport.h`) and in `UsbTransport`; this file carries only how
Jaguar3 uses it:

- `InitWrite` runs its whole bring-up inside one `WriteBatchScope`
  (`RtlJaguar3Device.cpp`), ended before the coex thread starts because that
  thread shares the transport. A queued write that completed failed or
  short fails the batch close, and `InitWrite` throws there rather than
  start the coex thread over an incompletely programmed chip. `Init`
  (RX-only) opens no batch yet — not measured on a ground-station card.
- Every settle delay drains the queue first, µs ones included, on both
  dies: the `write_bb` / `rf_writer` table delay markers, `delay_us` and
  `delay_ms` on `Halrf8822c` and `Halrf8822e`, the efuse power-cut. A settle
  that sleeps while its writes are still queued is no settle; the drain is
  free on an empty queue and bounded by its depth otherwise.
- Measured: 1.30 → 0.65 s warm, 2.04 → ~0.7 s cold on one drone-side
  8812EU (ssc338q host). The transfer-count reduction is deterministic; the
  wall-clock figure is one unit, one host.

The RF radio-table load is write-only: bits [31:20] of the direct window
(`0x3c00`/`0x4c00 + addr*4`) are not storage, so the vendor's `MASK20BITS`
read-modify-write preserved nothing at the price of a synchronous read per
entry. Scope of that claim (`tests/j3_rf_window_readback.sh`): every one of
the 512 window words (both paths) poked with the high 12 bits set read back
0, on one 8812CU and one 8812EU; the post-bring-up histogram (all 512 words
0) is only a control, since the write-only load itself clears those bits.

## TX power

Both dies drive the SAME TXAGC block (`set_tx_power_ref` is the port of
`rtw8822c_set_write_tx_power_ref`): per-path references at
`0x18e8`/`0x41e8[16:10]` (OFDM/HT/VHT) and `0x18a0`/`0x41a0[22:16]` (CCK),
plus a per-rate DIFF table at `0x3a00 + (hw_rate & 0xfc)` — 4 rates per dword,
7-bit two's-complement, so `[-64, 63]`. Every TXAGC write must be preceded by
clearing the `0x1c90[15]` gate. The diff table is offset-invariant (an offset
shifts the reference; the shape rides on top), so a runtime offset step is ~8
register writes instead of 8 + 32.

The one real divergence is the *default shape*: the **E** derives per-path
references from the efuse power-by-rate table and walks `phy_reg_pg` for the
diffs; the **C** uses a flat `JAGUAR3_TXPWR_REF_BASE_8822C` on both paths with
no calibrated shape underneath. A caller table (`SetTxPowerRateDiffs`) replaces
that shape on either die. Note the 2SS consequence on the 2T2R C: the caller
struct describes the 1SS ladder only, so MCS8..15 stay at the reference while
MCS0..7 carries the caller's shape.

`GetTxPowerState` reads the references back from the chip (`hw_readback=true`),
but the diff half is the software copy — `0x3a00` is not read back.

## Per-packet TX power

The descriptor `TXPWR_OFSET_TYPE` is a bank *selector*: types 2/3 pick two
programmable 7-bit-signed power-index offsets in BB `0x1e70[31:16]`
(~1 dB/step, `DEVOURER_TXPKT_STEP_QDB` recalibrates), LRU-managed by
`SetTxPacketPowerOffsetQdb` (`TxPktPwrBanks.h`) — the banks reset *disabled*
at BB-table load, so the descriptor field alone is inert until programmed
(types 0/1 = per-STA BB-RAM by descriptor MACID, left as the 0 dB baseline).
On-air-validated on 8822CU + 8822EU, sticky across
`SetMonitorChannel`/`FastRetune`; the E compresses deep cuts (≈−6 dB floor,
same TSSI reshape as its offset slope).

## CCX energy sensing (`clm` / `nhm_env`)

**`Stop()` forgets any armed busy window** — the rule, and the residual it
does not close, are at `IRadio::ArmChannelBusy`, the one declaration site
where they can be kept true. What is specific to this die:

Measured on an RTL8812CU with the reset removed: arm, `Stop()`, retune, read
reports `spoil=retuned`; with it, `spoil=none` and no reading (`Stop()` runs
`rtw_hal_deinit()`). The reset sits OUTSIDE `_reg_mu`, unlike the RTL8733B's,
and deliberately: `Stop()` joins the coex thread, and that thread takes
`_reg_mu`, so holding it across the join would deadlock. The coex loop never
takes the CCX lock, which is what makes this ordering safe.

**An armed busy window (`ArmChannelBusy`) is DESTROYED by an NHM read on this
map.** Measured on an RTL8812CU: a clean 240 ms window read 60.4-61.6% under
load, while the same window with one `GetRxEnergy(with_nhm=true)` mid-way came
back as the 2 ms re-arm (311-326 of 62500 ticks). The 11AC families survive the
same intrusion and merely read 3-4 points high, so this is the generation where
the shared-engine rule is not optional. `GetRxQuality()` takes that NHM read,
which makes the trap easy to spring from a caller that never touches the busy
API.

`GetRxEnergy(with_nhm=true)` runs the shared CCX window (`src/NhmReader.h`) on
the JGR3 register map (CLM period is the low half of `0x1e40`, trigger
`0x1e60[0]`, ready+result `0x2d88`); on-air validated on an RTL8812CU, ch100.

This is the generation where `nhm_env` works as intended, because
`PhydmRuntimeJaguar3.cpp` clamps DIG to `DIG_MIN_COVERAGE 0x1e` …
`DIG_MAX_OF_MIN_COVERAGE 0x22` — four steps — so the gain reference barely
moves and the histogram mass is free to march up under an interferer. Against a
5 MHz non-802.11 carrier on a traffic-free channel: 0 frames decoded, `clm` 6,
`nhm_env` 56, against a quiet 0/0/0; 802.11 traffic at MCS1 read 606 frames /
15 / 15. The discriminator is the ratio — `nhm_env`/`clm` ≈ 1 under 802.11, ≈ 9
under the carrier. Note `fa_ofdm` moved 0 → 1776 on that same arm and remains
the more sensitive counter, and that the magnitudes are session-specific (an
earlier run of the same arms read 34 / 98 with `fa_ofdm` 2926 — a stronger
carrier at the receiver for the same SDR gain). Compare arms within one
session.

In a **TX session** with a 300 ms quiet window the counters are alive (clean
0/0/0, carrier `clm` 5 / `fa` 1118 / `cca` 1122) — on the same 8812CU and code
path that previously read *inert* with 4–20 ms windows, so window length rather
than generation is the live variable in that older result.
