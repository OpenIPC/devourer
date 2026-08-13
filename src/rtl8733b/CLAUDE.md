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
- **TSSI closed loop.** Power runs from a fixed safe target
  (`kSafeTssiTargetQdbm8733b`); none of the runtime TX-power levers are ported.
  The CCK and OFDM/HT variants of the thermal-compensation table are different
  tables, and the table cannot be changed while tracking is enabled — so
  `select_tssi_rate_table` disables tracking, rewrites, and re-enables, with
  each stage verified by register readback. Note what the failure path is and
  is not: `Phy8733b::enable_tssi_tracking` restores its own analog/BB snapshot
  when its verdict fails, but the *transition* has no rollback to the previous
  table — `select_tssi_rate_table` returns false with tracking left off, and
  the caller responds by tearing the session down rather than transmitting at
  an unverified power setting. That transition costs **84 ms / 136 USB
  register round trips** (bench, one unit), so a stream alternating CCK and
  OFDM rates is capped around 11 fps; a single-rate stream early-returns and
  pays nothing.
- **The loop needs settling time.** Alternating CCK and OFDM at ~9 ms/frame
  leaves CCK airing ~5 dB above its settled level; the same stream paced to
  86 ms/frame lands on the settled value. Any power measurement taken during a
  fast rate-switching run is measuring the tracking loop, not the transmitter.
- **Thermal is telemetry, like everywhere else.** `InitWrite` logs one
  bring-up snapshot and `GetThermalStatus` serves the caller's own cadence.
  Nothing in the send path reads it: measured at 2.51 ms of a 2.71 ms
  per-frame budget on USB high speed, for a meter that tracks PA bias rather
  than junction temperature.

All four bench figures above (84 ms, 11 fps, 5 dB, 2.51/2.71 ms) come from the
single validation unit described below and have no in-repo oracle — treat them
as the order of magnitude to design against, not as constants to assert.
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

The advertised surface is only what an independent witness decoded. The
admission predicates live in `TxDescriptor8733b.h` so they are testable without
hardware (`tests/rtl8733b_tx_desc_selftest.cpp`), but note which path enforces
what — they are not uniformly shared:

- `tx_mode_supported_8733b` gates the whole `SetTxMode` default (and calls the
  two below).
- `ht_request_supported_8733b` additionally gates the radiotap MCS branch, so
  the HT contract is the one thing both paths genuinely share.
- `legacy_request_supported_8733b` is **only** reached through `SetTxMode`. The
  radiotap `RATE` branch takes the byte as given; a legacy rate arriving that
  way is constrained downstream by `valid_tx_desc_config` (`rate_id <= 0x1f`)
  rather than by the predicate. That is adequate today only because the
  radiotap RATE field cannot express SGI/LDPC/STBC — do not assume the two
  legacy paths are kept in step by construction.

- Admitted: 1SS BCC — legacy OFDM and HT MCS0-7, 20/40 MHz, 2.4/5 GHz; plus
  long-preamble CCK restricted to 2.4 GHz at 20 MHz (`tx_rate_id_8733b`
  returns a `0xff` sentinel otherwise, which `valid_tx_desc_config`'s
  `rate_id <= 0x1f` range check then also refuses — the band gate is closed
  end-to-end, not in two halves).
- Refused: SGI (a descriptor with the short-GI bit set submitted successfully,
  but an independent RTL8812AU witness decoded the probes as **long GI** — the
  descriptor and the air disagree, and the reason is not yet understood), LDPC
  and STBC (only forced-global-BCC operation has passed witnessed injection;
  the encoder writes no STBC field at all), VHT and HE on every band.

## Not ported

`ReadTsf`/beacons, hardware ACK/BlockAck, A-MPDU, `FastRetune`,
`FastSetBandwidth`, the runtime TX-power knobs, `rx.path` per-chain telemetry,
and CCA disable. These inherit `IRtlDevice`'s not-ported defaults (`false`,
`0`, or a full-path fallback) rather than being faked. `SetCcaMode` is the one
exception to the silent-default rule: it is pure virtual, so `true` throws
loudly — without tearing the session down, since an unported optional knob is
not a hardware-safety event — while `false` succeeds as a no-op because that is
the state MAC bring-up already leaves programmed.

`DeviceConfig::tuning::disable_cca` is currently dropped at bring-up here
rather than refused, which is the one place a request still vanishes without a
word.

## Validation status

Every on-air claim rests on **one** physical unit: a bare unbranded 1T1R
RTL8731BU module, `0bda:f72b`, cut D, USB high speed, witnessed by an
RTL8812AU. No SDR was available, so occupied bandwidth, spectral mask, EVM and
absolute power are unmeasured — which is also why `narrowband_ok` stays false
despite the 5/10 MHz register sequence reading back. The code is more
permissive than the cap: `build_tx_block` accepts a 5/10 MHz session width, so
narrowband is reachable while unadvertised. No `0bda:b733` combo
module has been seen, no vendor-driver A/B control was obtained, and the bench
hub could not switch VBUS, so only warm re-init and physical replug were
tested. The full tested/deferred matrix, the provenance pins and the second
unit that overheated and was excluded: `docs/rtl8733b.md`.

Headless coverage: `tests/rtl8733b_{efuse,phy_table,rx_parse,tx_desc}_selftest.cpp`
in `ctest`. Hardware: `tests/rtl8733b_lifecycle_soak.sh` (bounded warm
lifecycle, explicitly not a true VBUS cycle) and `examples/rtl8733bprobe`
(staged identity → power/EFUSE → firmware → MAC/PHY → TSSI audit).
