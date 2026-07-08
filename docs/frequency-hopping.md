# Single-adapter frequency hopping

This document describes how devourer makes one Realtek Jaguar adapter transmit
on several channels in quick succession — a per-packet frequency hop — and the
specific techniques that bring the retune cost down from a quarter of a second
to ~1.5 ms. It doubles as a porting guideline: the same anatomy (find the
channel write, strip the per-hop work down to it, kill the read-modify-write
penalty) applies to any Realtek generation, so the "Porting to other
generations" section at the end generalises each trick.

## Why frequency is not like MCS

devourer already lets a single frame pick its own rate / bandwidth / STBC /
LDPC / SGI by reading them from the radiotap header — those are **per-packet
fields baked into the TX descriptor**, applied by the PHY at zero cost frame to
frame. It is tempting to treat frequency the same way, but frequency is
categorically different:

- There is **one RF synthesiser (LO) per chip**. The 2/4 RF paths share it
  within a band, so the radio sits on exactly one centre frequency at any
  instant. You cannot transmit two channels at once from one adapter (that
  needs two synthesisers or a wideband DAC like an SDR).
- There is **no channel field in the TX descriptor**. Centre frequency is
  hardware PLL state, set by writing RF registers.

So hopping is not "tag the frame and forget it" — it is a real hardware state
change between packets, and the entire engineering problem is making that state
change cheap enough to do between packets without collapsing throughput.

The baseline cost of a full channel switch (`set_channel_bwmode`) on an
RTL8812AU is **~275 ms** for an intra-band 2.4 GHz hop. At ~345 fps (the
chip-limited monitor-injection rate) the inter-frame gap is ~2.9 ms, so the
unoptimised switch is ~95 frame-times — hopping would reduce the link to a
trickle. The work below gets it to ~1.5 ms (~0.5 frame-times).

## Anatomy of a channel switch

`set_channel_bwmode` → `phy_SwChnlAndSetBwMode8812` runs four stages, each
timed by the `InitTimer` (`channel_set.<stage>` lines). Measured on an 8812AU
C-cut, intra-band 2.4 GHz:

| Stage | Function | Cost | What it does |
|-------|----------|------|--------------|
| `sw_chnl` | `phy_SwChnl8812` | ~98 ms | RF centre-channel + MOD_AG writes, spur fix, fc_area |
| `set_bw` | `phy_PostSetBwMode8812` | ~53 ms | bandwidth + secondary-channel registers |
| `tx_power` | `PHY_SetTxPowerLevel8812` | ~100 ms | per-rate TXAGC fan-out (~150-200 writes) + tx-power training |
| `pwrtrk` | `_pwrTrk.TickThermalMeter` | ~22 ms | thermal-meter read + BB-swing index (8812 only) |

Two surprises drive everything:

1. **IQK does not run on an intra-band hop.** `_needIQK` is only set on a band
   transition (`phy_SwBand8812`), so the calibration that everyone assumes
   dominates a retune is simply not in the path. (The `channel_set.iqk` timer
   stage label is misleading — for an intra-band hop it is measuring the
   `pwrtrk` tick, not IQK.)
2. **`sw_chnl` is ~98 ms for only a handful of RF writes.** That is the
   read-modify-write penalty, explained below. It is the single biggest lever.

## The retune cost model

The per-hop cost decomposes cleanly, and each component maps to a technique:

```
full switch  = sw_chnl(98) + set_bw(53) + tx_power(100) + pwrtrk(22)  ≈ 275 ms
                  │             │            │               │
                  │             │            │               └─ skip (Trick 1)
                  │             │            └─ skip (Trick 1)
                  │             └─ skip for same-BW, or cache (Trick 4)
                  └─ cache the RF writes (Trick 2) + cache constants (Trick 3)
```

The end state is ~1.5 ms: two RF writes (one per path) of a cached register
value, with everything else either skipped or written only when it actually
changes.

## Trick 1 — skip the stages a hop does not need

An **intra-band, same-bandwidth** hop changes only the centre frequency.
Therefore:

- **`tx_power` (the per-rate TXAGC loop, ~100 ms) is skippable.** Per-rate TX
  power barely moves across adjacent channels in one band; it was set by the
  last full channel set (e.g. `InitWrite`) and stays close enough. (There is a
  long-standing escape hatch, `DEVOURER_SKIP_TXPWR`, that proves the rest of
  the system tolerates skipping it.)
- **`pwrtrk` (the thermal tick, ~22 ms) is skippable.** It is a slow
  steady-state tracker, not a per-frame requirement.
- **`set_bw` (~53 ms) is skippable when the bandwidth is unchanged** (the 20 MHz
  hop case), and reducible to a few writes when it is not (40/80, Trick 4).
- **IQK is already absent** intra-band.

This alone (the non-cached fast path) gets a 20 MHz hop to ~98 ms — the
irreducible `sw_chnl`. The flag `_fast_skip_heavy` makes the shared
`phy_SwChnlAndSetBwMode8812` skip `tx_power` + `pwrtrk` for one call, which is
how the 40/80 fallback path reuses the full code while still being faster.

**The gate matters.** `fast_retune` declines (returns `false`, touches nothing)
when the hop crosses the 2.4/5 GHz boundary, and the caller falls back to the
full `set_channel_bwmode` — because a band change genuinely needs the RFE/AGC
reconfiguration, the TX-FIFO drain, and the IQK that a hop skips. Never fast-path
a band change.

## Trick 2 — cached full-register RF writes (the big one)

`phy_SwChnl8812` sets the channel with **masked** RF writes:

```cpp
phy_set_rf_reg(path, RF_CHNLBW_Jaguar, BIT18|BIT17|BIT16|BIT9|BIT8, mod_ag); // RF_MOD_AG
phy_set_rf_reg(path, RF_CHNLBW_Jaguar, bMaskByte0, channel);                 // channel byte
```

A masked RF write is a **read-modify-write**: `phy_set_rf_reg` reads the current
register (to preserve the unmasked bits), merges, and writes. And on **C-cut
silicon an RF read carries a 20 ms sleep** (`phy_RFSerialRead`, gated by
`IS_C_CUT`). So:

```
2 masked writes/path × 2 paths × 20 ms read  ≈  80 ms   ← the bulk of sw_chnl
```

The fix is to **never read**. `RF_CHNLBW_Jaguar` (0x18) holds the channel byte,
the MOD_AG field, and the bandwidth bits. Across a hop only the channel byte
changes; the rest is constant. So maintain a per-path cache of the full 0x18
value and write it with the **full LSSI-write mask** (`bLSSIWrite_data_Jaguar`,
0x000fffff), which `phy_set_rf_reg` recognises as "write all 20 bits, no read":

- `_rf_chnlbw_cache[4]`, `_rf_chnlbw_cached` — primed once by reading 0x18 per
  path (paying the 20 ms reads exactly once), then every subsequent hop merges
  the new channel + MOD_AG fields into the cached value in memory and does one
  full write per path. No reads, no sleeps.
- The merge must replicate `phy_set_rf_reg`'s exact arithmetic
  (`(cached & ~mask) | (data << shift)`, no masking of the shifted field) so the
  cached write produces the bit-identical value the masked write would have,
  given the cache equals the real register.
- **Invalidate the cache whenever the full path runs** (`set_channel_bwmode`
  clears `_rf_chnlbw_cached`), because the full path can rewrite 0x18's
  bandwidth bits.

This is the single change that takes `sw_chnl` from ~98 ms to a few ms. It is
the most important trick to carry to any C-cut-afflicted generation.

`phy_SwChnl8812_fast` is the cached variant of `phy_SwChnl8812`.

## Trick 3 — write constants only when they change

The remaining `sw_chnl` work is per-path-independent BB state that is constant
across most hop sets. Cache the last value and skip the write when unchanged:

- **`fc_area` (BB 0x860)** is constant within a band (e.g. always one value for
  2.4 GHz). `_last_fc_area` skips the write unless the band-area bucket changes.
- **The spur fix (`phy_FixSpur_8812A`)** writes *global* (not per-path) BB
  ADC-clock registers, and its output depends only on `(bandwidth, channel ∈
  {11,13,14})`. The original loop called it once per RF path — redundantly. The
  fast path computes a small "spur class" and calls it **once**, only when the
  class changes (`_last_spur_class`).

For a 1→6→11 hop set, both collapse to a single write on the first hop and
nothing thereafter.

## Trick 4 — 40/80 MHz: the secondary channel depends on the offset, not the channel

A wider-bandwidth hop *seems* to need the full `set_bw`, because the
secondary-channel registers (`REG_DATA_SC`, `rRFMOD[0x3C]`, `rCCAonSec`,
`rCCK_System`) change. The insight that makes it cheap:
`phy_GetSecondaryChnl_8812` computes the secondary-channel value from the
**prime-channel offset** (`_cur40MhzPrimeSc`/`_cur80MhzPrimeSc`) — *not* the
channel number. So across a same-bandwidth, same-offset hop set, the
secondary-channel value is **constant**, and the only thing that changes is the
RF centre channel.

Consequences for the lean 40/80 path:

- Write the **centre channel** (`rtw_get_center_ch(primary, bw, offset)`) — not
  the primary — to RF 0x18, via the same cached full-register write as 20 MHz.
  (The full path also writes the centre, because `set_channel_bwmode` stores the
  centre as `_currentChannel`.)
- Write the secondary-channel registers **once**, skipping them when unchanged
  (`_last_subchnl`).
- Leave the bandwidth-only registers (`phy_SetRegBW`, the BW-constant
  `rRFMOD` field, `rADC_Buf_Clk`, `rL1PeakTH`, `PHY_RF6052SetBandwidth`)
  untouched — they were set by the last full set at this bandwidth and do not
  change across a same-BW hop. The bandwidth bits already live in the cached
  RF 0x18 value, so the cached write preserves them.

Result: 40/80 MHz hops at ~1.5 ms, the same as 20 MHz. There is also a
non-cached 40/80 fallback that reuses the full `set_channel_bwmode` with
`_fast_skip_heavy` (~153 ms) — kept because it is correct-by-construction and
useful when the cache is cold or for chips without the cached path.

## Trick 5 — radiotap-driven per-packet hopping

To make hopping usable by any caller without a side channel, the library reads
the radiotap `CHANNEL` field in `send_packet` (just as it reads RATE/MCS): the
frequency is mapped to a channel and, if it differs from the current channel, a
`FastRetune` runs **before** the descriptor is built (and before the 5 GHz CCK
clamp, so the clamp keys off the new channel). It is a no-op when the field
matches the current channel, so an informational CHANNEL field costs nothing.
The honest cost: unlike MCS, this stalls that one `send_packet` by the retune
latency — but at ~1.5 ms that is acceptable.

This turns frequency into a per-packet property at the API level while keeping
the truth (it is a hardware state change) visible in the cost.

## Correctness and validation methodology

Two independent methods were used; both generalise to any port.

### On-air, with a wideband SDR

A single wideband receiver (B210 / AD9361 clone) can see all hop channels at
once and so distinguish "the radio retuned" from "the radio dropped frames" — a
narrowband receiver tuned to one channel cannot. `tests/hop_rx_probe.py`
captures wideband IQ and detects, per channel, the near-field bursts, then
checks the dominant channel cycles through the full hop order. Pitfalls learned,
all relevant to anyone building such a rig:

- **Capture must run in compiled code.** A Python `recv` loop cannot sustain
  61.44 MSps and overflows constantly — dropped samples read as false "frame
  loss". Use a GNU Radio `usrp_source → file_sink` flowgraph (or any C++
  capturer) and verify zero overflows.
- **The AD9361 analog bandwidth caps at ~56 MHz**, regardless of the FPGA. For
  channels near the band edge (e.g. 1/6/11 at ±25 MHz around centre) integrate
  power in a narrow window (±3 MHz) around each carrier rather than the full
  20 MHz, and notch DC (a channel parked at the SDR centre sees the receiver's
  DC offset).
- **Some clones deliver a spectrally-mirrored spectrum** (apparent channel
  order reverses, with the centre channel fixed). Accept either direction when
  matching the hop sequence.
- **Don't trust a single on-air reading.** Take one clean SDR read per session
  and compare full-path vs fast-path: if the *full* path is equally weak, the
  weakness isn't your change.

### Register parity (when clean on-air is unavailable)

When a clean on-air reading isn't available at a given band/BW, prove
the fast path leaves the chip in the **same channel/BW register state** as the
known-good full path. `DumpCanary()` reads a fixed set of BB/MAC/RF registers;
`tests/hop_parity_check.sh` drives the chip to the same target channel via the
full path and then the fast path and diffs the dumps. Crucial details:

- **The fast path must emit the canary too.** The cached path does not go
  through `phy_SwChnlAndSetBwMode8812`, so the dumper was factored out into
  `DumpCanary()` and called from the cached branches; otherwise the diff
  compares the fast run's *stale init* dump and reports a phantom mismatch.
- **Classify the expected differences.** A self-validating script runs the full
  path twice as a control to discover run-variant registers (the free-running
  TSF timer; IQK/RxIQC measurement-jitter outputs), and excludes those plus the
  tx-power registers the fast path deliberately skips. Anything left that
  differs is a real channel/BW break. The passing result: every channel/BW
  register (RF 0x18 centre, fc_area, rRFMOD/SubChnl, ADC-clk, IQK matrix) is
  bit-identical fast-vs-full.

## Cross-layer payoff: frequency-diversity FEC

Hopping is most valuable not as raw redundancy but as a frequency-diversity
code on top of the existing outer FEC (see `docs/fused-fec.md`). With per-packet
hopping (dwell = 1), the outer code's symbols — emitted in order — land on
channels round-robin, so a block of N symbols is spread across the hop set "for
free": ESI `i` goes to channel `i mod N_ch`. A narrowband fade or interferer
that wipes one channel then erases only ⌈N / N_ch⌉ of each block's symbols, and
an MDS (Reed-Solomon) outer code recovers the block as long as

```
repair_count  ≥  ⌈N / N_ch⌉
```

This converts a catastrophic single-channel outage (which, without hopping,
loses whole blocks regardless of how much parity you add) into a recoverable
erasure. The receiver side is purely a combining problem: an erasure decoder
dedups by `(block_id, symbol_index)`, so feeding it the symbols recovered on
each channel — from N adapters, a wideband SDR, or a lockstep-hopping single
adapter — simply adds them up. `tools/precoder/hop_diversity_sim.py` and
`hop_rx_combine.py` (with tests) prove both properties against the real codec.

## Measured results (RTL8812AU, C-cut)

| Path | 20 MHz | 40/80 MHz |
|------|--------|-----------|
| Full `set_channel_bwmode` | ~275 ms | ~275 ms |
| Skip heavy stages only (no cache) | ~98 ms | ~153 ms |
| **Cached fast path** | **~1.6 ms** | **~1.5 ms** |

First hop after a full set is ~50 ms (primes the RF cache + writes the
constants once); every subsequent same-band hop is ~1.5 ms.

## Limitations

- Intra-band only; a cross-band hop falls back to the full path (correct, slow).
- TX power is not re-tuned per hop — fine across a band's adjacent channels,
  but a hop set spanning a wide 5 GHz range may want a periodic full set to
  refresh per-rate power.
- The receiver still needs N adapters, a wideband SDR, or lockstep hopping to
  hear all channels — one adapter has one LO on RX too.

## The ports: all three generations

`IRtlDevice::FastRetune(channel, cache_rf)` is the generation-agnostic entry
point (default = the full `SetMonitorChannel` at the current width/offset), and
every generation overrides it with a lean path built from the tricks above:

| DUT | full path | fast (cached) | USB op cost | fast ops/hop |
|-----|-----------|---------------|-------------|--------------|
| RTL8812AU (Jaguar1) | ~277 ms | **~1.6 ms** | ~0.8 ms | 2 |
| RTL8822BU (Jaguar2) | ~65 ms | **~2.5 ms** | ~1.0 ms | 2 |
| RTL8821CU (Jaguar2) | ~30 ms | **~0.55 ms** | ~0.5 ms | 1 |
| RTL8822CU (Jaguar3) | ~12 ms | **~1.9 ms** | ~0.21 ms | 9 |
| RTL8812EU (Jaguar3) | ~12 ms | **~2.4 ms** | ~0.27 ms | 9 |

(Median `hop.dwell` switch_us over a 1/6/11 hop set; per-stage numbers from
`DEVOURER_HOP_PROF=1`. Every hop microsecond is USB round-trips: one register
read or write is one synchronous control transfer, whose latency is a property
of the chip's EP0 handling — it varies 5× across the family — so the only code
lever is op count. FHSS-soak-validated: 12,000 (8822CU) and 9,000 (8822BU)
consecutive dwell-1 per-packet hops, zero bulk-OUT failures; a kickless hopping
receiver holds a constant catch rate over ~850 retunes.)

Two techniques carried the newer generations to the table above, beyond the
original tricks:

- **The compose cache (Trick 2 generalised to every masked write).** A masked
  `phy_set_bb_reg` is a read-modify-write — two control transfers. The fast
  path primes the full dwords of every register it touches ONCE per epoch
  (first fast hop, lazily — priming at full-set time would cache values that
  init calibration then rewrites), composes bit changes in memory, and writes
  whole dwords: the steady hop is write-only. Correct by construction — the
  untouched bits are written back as read.
- **No per-hop RX kick.** The vendor `switch_channel` tail (Jaguar2's RF 0xb8
  toggle, RX-path toggle, IGI toggle — 13 of the 19 transfers the first Jaguar2
  port paid) belongs to the full path only: hardware A/B on both Jaguar2
  variants, both directions, shows identical hopping-RX catch rate (no decay
  over hundreds of kickless retunes) and identical hopping-TX delivery without
  it. A hop needs the channel write, not the state-machine kick.

**Jaguar2** (`HalJaguar2::fast_retune`, both variants): one composed
full-register RF18 write per path collapses the full path's read-modify-write
rounds (three of them on the 8821C's band/channel/BW split) — the 8821CU hop is
a **single LSSI write**; AGC index, CFO fc, the 8822B RF 0xBE VCO band / ch144
RF 0xDF flag / 2G spur registers and the 8821C 2G CCK filter are composed
writes on bucket change. RFE pins and the 8821C switch-band/RF-set block are
band-keyed and stay untouched. 5/10 MHz narrowband (8821C) survives fast hops
for free: the re-clock state lives in the bandwidth-keyed
`0x8ac`/`0x8c4`/`0x8c8` block the hop never touches, and the NB RF18 BW bits
equal the 20 MHz encoding, so the cached RF18 write is already correct.

**Jaguar3** (`RadioManagementJaguar3::fast_retune`, both variants): the RF18
window write inside its 3-wire bracket + force-anapar + BB reset every hop —
nine composed writes total; SCO fc, TX DFIR (both nibbles in one dword) and the
AGC table on bucket change. In 5/10 MHz narrowband mode fast hops preserve the
divider re-clock (no per-dwell `set_bandwidth_dividers` + DAC-FIFO reset), and
the TX-DFIR write uses the NB-owned `0x808[6:4]` value on the 8822e.

**End-state parity, not step parity** — the subtlety the Jaguar3 port
hardware-taught: init-time calibration (halrf IQK) rewrites BW-keyed state
*after* the channel set (e.g. it clears the 40 MHz TX_CCK_IND bit in the
8822e's RF 0x1a), and the full path re-imposes the channel-set values on every
retune. "Skip BW-keyed work" must therefore mean "re-assert it once per
fast-hop epoch", not "never touch it" — the parity oracle defines correct as
"the fast path ends in the same register state the full path ends in".

## Porting to another generation (the method that produced the above)

1. **Find the channel write and the per-stage cost.** Instrument the channel
   set with a per-stage timer. Identify which stages are channel-dependent and
   which are steady-state (tx-power, thermal). Skip the steady-state stages for
   an intra-band hop (Trick 1).
2. **Check whether RF writes are masked read-modify-writes, and whether reads
   are slow.** This is the dominant cost on C-cut Jaguar. If a generation reads
   the RF register on every masked write — or polls a serial-read-done bit, or
   sleeps — cache the full register value and write it whole (Trick 2). If the
   generation offloads channel switching to firmware (an H2C "channel switch"
   command), that may already be the fast path; measure it.
3. **Identify the constant-across-hop registers** (band-area, spur/ADC-clock,
   AGC sub-band) and write them only on change (Trick 3).
4. **For wide BW, find what the secondary-channel registers actually depend
   on.** If, as on Jaguar, they depend on the prime offset rather than the
   channel number, the wide-BW hop is just the centre-channel write plus a
   one-time secondary-channel write (Trick 4). Otherwise, reuse the full BW
   post-set with a skip-heavy flag.
5. **Gate band changes out of the fast path** — they need the front-end
   reconfiguration and recalibration a hop skips. And check what the
   generation's *calibration* rewrites after a channel set: anything it touches
   that the full path re-imposes must be re-asserted once per fast-hop epoch
   (the end-state parity rule above).
6. **Validate two ways:** wideband SDR on-air for end-to-end behaviour
   (`tests/run_hop_validation.sh`), and register-parity against the full path
   (`tests/hop_parity_check.sh` — family-aware; all three generations emit the
   same `DumpCanary` format from both paths). Always run a full-vs-full control
   to separate inherent register variance (timers, calibration jitter, live
   AGC/thermal state) from real parity breaks.

The throughline: a channel switch is mostly work a *hop* does not need. Strip it
to the one register that actually changes, and remove the read that change
implies.
