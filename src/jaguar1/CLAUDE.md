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

## Per-packet TX power

- **8814A**: the 3-bit descriptor `TXPWR_OFSET` LUT at the 8822B position
  (dword5 [30:28]); session default `SetTxPacketPowerStep` on
  `RtlJaguarDevice`.
- **8812AU/8821AU**: no descriptor field exists — per-rate selection via
  radiotap is their only per-packet lever. The compensating fast lever is
  `FastSetTxPowerOffsetQdb` (BB-swing TxScale `0xc1c/0xe1c`: global
  per-burst, 1–4 writes, 0.5 dB steps, −12..+2 dB, folded through the 8812A
  thermal tracker; on-air-validated on the 8812AU).
