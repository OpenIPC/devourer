# src/jaguar1/ — Jaguar1 (11ac gen1) working context

Deep per-generation facts for this subtree, loaded alongside the root
CLAUDE.md. Chips: RTL8812AU (2T2R reference), RTL8811AU (1T1R cut, rides the
8812 path), RTL8814AU (4T4R RF / 3-SS baseband), RTL8821AU (1T1R + BT).

## HAL layout

`RtlJaguarDevice` (orchestrator), `HalModule` (bring-up, power seq, table
apply — most chip-family divergence, `_8812A`/`_8814A`/`_8821A` suffixed
methods), `RadioManagementModule` (channel/BW/TX power, up to 4 RF paths),
`EepromManager` (EFUSE + `cut_version`/`rfe_type` for `CheckPositive`),
`FirmwareManager` (8814 = 3081-DDMA path, 8812/8821 = page-write),
`FrameParser` (RX parsing + `SET_TX_DESC_*_8812` descriptors).

## Chip facts

- **8814AU**: host-pushed TX requires the on-chip 3081 MCU booted during FW
  download; a failed FW-boot poll means dead TX while RX still works.
- **5/10 MHz narrowband** (8812AU/8811AU + 8814AU): both dies share the
  Jaguar2 `0x8ac` ADC/DAC clock-divider block, so the re-clock trick works
  even though the vendor never wired it — TX+RX bench-characterized. The two
  dies use different field encodings of the same register: the 8812A divides
  via `[9:8]`/`[21:20]`, the 8814A via the 8822B's
  `[9:8]+[16]`/`[21:20]+[28]`. The 8821A is excluded (dividing its DAC clock
  starves the 1T1R TX DMA/USB path) and falls back to 20 MHz. See
  `docs/narrowband.md`.
- `DEVOURER_RX_PATHS` (RX-chain mask) writes `0x808` byte 0 via
  `SetRxPathMask`; sticky across `SetMonitorChannel` because IQK
  saves/restores `0x808`.
- Thermal (RF 0x42 meter): Jaguar1 has no hard thermal TX shutdown — a
  rising delta is the early warning, hence bucketed status rather than a
  fake temperature. On the 8814 the EFUSE baseline is read at the 8812
  offset, so absolute delta may be off; the trend is valid.
- The 8812A/8821A have no LA capture block (the 8814A does).
- The 8821A's VHT-LDPC RX is broken (field-reported; HT-LDPC fine); the
  8814A decodes LDPC but reports no per-frame flag (`ldpc_rx_flag=0`,
  `RxAtrib.ldpc` reads 0).

## Teardown

`RtlJaguarDevice::Stop()` and the destructor run `HalModule::rtw_hal_deinit()`:
halt the MAC engines (`REG_CR`, `REG_RCR`), then the die's card-disable power
sequence via the existing `HalPwrSeqCmdParsing` (`rtl8812_card_disable_flow` /
`rtl8814A_` / `rtl8821A_`, dispatched exactly like `InitPowerOn` dispatches the
enable flows). Both call sites are best-effort — a chip already off the bus
makes the writes fail, which is fine on a teardown path.

This is not tidiness: left in ACT the chip keeps its RF front end live
indefinitely after the owning process exits, and an autonomously-airing beacon
keeps transmitting. Justified on those grounds alone — **no performance claim is
attached**, and the delivery recovery an earlier revision cited came from a
ground station too marginal to support it (`docs/warm-tx-degradation.md`).
`PowerOff()` clears `_macPwrCtrlOn` so a re-init on the same `HalModule` still
runs the power-on sequence; `tuning.teardown_power_down=0` disables it, which a
post-mortem needs since a powered-down chip reads back the CARDEMU fill.

## TX power

Every per-rate index funnels through one composer, `ComputeTxPowerIndex(path,
rate, ntx)` — flat override, else the caller per-rate table, else the EFUSE
walk — then `+ txpwr_offset_steps_`, clamped to the 6-bit rail. The register
write is per rate on both ports (the 8812A/8821A `0xc20..0xc4c` fanout, the
8814A packed `0x1998`), so a caller table (`SetTxPowerRateDiffs`) needs no
write-path change and is sticky by construction: every channel-set re-runs the
same walk.

Its **anchor** is `GetTxPowerIndexBase(path, MGN_MCS7, 1SS, bw, ch)` — the
EFUSE index of the section reference rate at the current channel and
bandwidth, cached once per apply pass (4 lookups on a 4-path 8814A instead of
one per rate). Resolution is the family's 0.5 dB step, so an odd qdB rounds.

`FastSetTxPowerOffsetQdb` drives the BB-swing TxScale words, a global scaler
DOWNSTREAM of the per-rate TXAGC table: it composes with a caller table but is
rate-independent, so neither can implement the other.

## Per-packet TX power

- **8814A**: the 3-bit descriptor `TXPWR_OFSET` LUT at the 8822B position
  (dword5 [30:28]); session default `SetTxPacketPowerStep` on
  `RtlJaguarDevice`.
- **8812AU/8821AU**: no descriptor field exists — per-rate selection via
  radiotap is their only per-packet lever. The compensating fast lever is
  `FastSetTxPowerOffsetQdb` (BB-swing TxScale `0xc1c/0xe1c`: global
  per-burst, 1–4 writes, 0.5 dB steps, −12..+2 dB, folded through the 8812A
  thermal tracker; on-air-validated on the 8812AU).
