# src/jaguar2/ — Jaguar2 (11ac gen2) working context

Deep per-generation facts for this subtree, loaded alongside the root
CLAUDE.md. Chips: RTL8822BU / RTL8812BU (chip-id `0x0a`) and RTL8811CU /
RTL8821CU (chip 8821C, chip-id `0x09`); the RTL8821CE (PCIe) rides this same
HAL through `PcieTransport`.

## HAL layout

Shared core (`RtlJaguar2Device`, `HalJaguar2`, `HalmacJaguar2Fw`/`MacInit`,
`FrameParserJaguar2.h`) + per-variant strategy interfaces selected by
`ChipVariant`: `Jaguar2PhyTables` → `Phy8822bTables`/`Phy8821cTables`,
`Jaguar2Calibration` → `Halrf8822b`/`Halrf8821c`. Every strategy seam
defaults to `C8822B`, so the 8822B path stays byte-identical when the 8821C
variant is compiled out. A hybrid generation: HalMAC FW download / MAC init /
power sequencing like Jaguar3, phydm `check_positive` register tables like
Jaguar1 (shared `PhyTableLoader`).

## Chip facts

- **5/10 MHz narrowband on both variants**: a baseband ADC/DAC re-clock
  packed into BB `0x8ac`; the RF stays in 20 MHz mode; applied as an
  end-of-bring-up retune. The 8822B RF synth only re-latches on an RF18
  *value edge*, so the narrowband path writes RF18 twice — same-value
  rewrites do nothing; see the `set_channel_bw` NB branch and
  `docs/narrowband.md`.
- 5 MHz at 5 GHz is CFO-limited: subcarrier spacing shrinks 4× and a
  far-offset TX/RX crystal pair syncs bimodally per bring-up — at 2.4 GHz
  the same pair is stable (`tests/narrowband_cross_rx.sh` header).
- Per-rate bandwidth-aware efuse TX power clamped to generated `txpwr_lmt`
  tables (narrowband folds to the 20 MHz column).
- Golden-init replay (`DEVOURER_REPLAY_WSEQ`, a captured kernel write stream
  applied verbatim at the end of Init) is the debugging lever that found the
  RF18-edge bug.
- The TXAGC block is write-only, so `GetTxPowerState` reports the software
  shadow (`hw_readback=false`).
- **Firmware channel switch** (H2C 0x1D `SINGLE_CHANNELSWITCH_V2`,
  `fw_channel_switch` + the `fastretune_fw` knob; both Jaguar2 dies — the
  path is variant-generic, 8822B on-air-validated, 8821C code-covered
  pending hardware): the fw executes the whole
  retune in ~1–2 ms and reports C2H `CUR_CHANNEL`. Never poll RF18 *during*
  the switch — the PI reads contend with the firmware's RF-bus writes and
  triple the on-air dark time; confirm the previous switch at the next hop
  instead (`fw_switch_confirm`). Classic 8-byte H2Cs ride the HMEBOX
  mailboxes (0x1d0/0x1f0 + 0x1cc busy bits), distinct from MacInit's 32-byte
  h2c-pkt queue.

## TX power

Per-rate indices are computed in software and written 4 rates per dword into
`0x1d00` (path A) / `0x1d80` (path B). Two shapes feed that block: the 8821C's
programmed-efuse vendor formula (`base + (min(by_rate, lmt) - rs)`) and the
shared efuse walk both variants use otherwise.

A caller table (`SetTxPowerRateDiffs`) replaces either shape via
`write_txagc_diffs`, anchored on the HT 1SS section index taken **pre-offset
and pre-rail** — clamping the anchor first would rebuild the shape off a
saturated base. Resolution is the family's 0.5 dB step. Rows the table does not
describe (MCS8-15, VHT) are written explicitly AT the anchor rather than left
alone, because the previous apply left per-section-distinct values there. On an
unprogrammed EFUSE there is no baseline to anchor on: that path warns loudly
and leaves TXAGC at the BB-table default rather than silently flattening.

`SetMonitorChannel` and a cross-band `FastRetune` re-apply only when a knob is
active — the table counts as one, or a channel change would silently revert to
the calibrated shape.

## Per-packet TX power

The 8822B/8821C descriptor `TXPWR_OFSET` is a hardware LUT
(0/-3/-7/-11/+3/+6 dB); session default via `SetTxPacketPowerStep`.
