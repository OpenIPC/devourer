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

## Monitor RX filters

`DeviceConfig::Rx::drop_ctrl` and `own_addr_only` are applied in
`RadioManagementModule::hw_var_set_monitor`. Two things differ from the
HalMAC generations. Control frames are gated by RCR `ACF` here: closing
RXFLTMAP1 alone left every control subtype arriving on an 8812AU (4184 of
4234), while clearing `ACF` took a 10 s cell from 4951 to 143 — and every one
of the 143 was a BlockAck, which this MAC passes with data frames, so
`rx.blockack` readers still see them. And bring-up programs MACID from EFUSE,
so `own_addr_only` works without an identity arm. With `drop_ctrl` on, the
8812AU as transmitter still consumed its peer's ACKs and BlockAcks (reports
OK 87% vs 86% open, 0% with no responder; A-MPDU 77% vs 78%, 0%). One unit,
ch36; the 8812AU as ACK *responder* under `drop_ctrl` is unmeasured (the
cross-rig loop did not close with the knobs off either).

Under `own_addr_only` a beamformee arm and an ACK responder cannot share a
session on the 8812 die: the beamformee answers on the EFUSE address, a
responder on another address is refused at bring-up because it moves MACID,
and a responder on the EFUSE address is refused by the arm itself (this die
cannot be armed to its pre-arm MACID).

## Teardown

**`Stop()` forgets any armed busy window** — the rule, and the residual it
does not close, are at `IRadio::ArmChannelBusy`, the one declaration site
where they can be kept true. What is specific to this die:

Measured on an RTL8812AU with the reset removed: arm, `Stop()`, retune, read
reports `spoil=retuned` — a reason earned by a session that no longer exists;
with it, `spoil=none` and no reading (this die's `Stop()` powers the card
down). The reset sits ABOVE the `teardown_power_down=0` early return, so the
"leave the chip powered" path forgets the window too. It takes the CCX lock
alone: this generation has no family-wide register lock to order against
(`_port0_mu` is narrower and never taken under the CCX lock).

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

## CCX energy sensing (`clm` / `nhm_env`)

`GetRxEnergy(with_nhm=true)` runs the shared CCX window (`src/NhmReader.h`) on
the 11AC register map — the same map validated on the Jaguar2 8822BU, so CLM
lands here with no Jaguar1-specific code. **Measured on an RTL8812AU**
(`docs/rx-spectrum-sensing.md`): a 240 ms armed window read 70.6-70.9% against
a flooder a MediaTek adapter independently measured, 0.1-1.0% on a quiet
channel, so `busy_airtime_measured` is true.

In 5/10 MHz narrowband the CCX tick follows the baseband clock, so the 8812A
and 8814A scale their CLM/NHM periods by it (`ccx_tick_shift_now`); the 8821A
never re-clocks and stays unscaled. **Not measured on Jaguar1** — the rule is
the vendor's and the measurement is the Jaguar2/Jaguar3/RTL8733B one in
`docs/rx-spectrum-sensing.md` "The armed busy window".

Window behaviour matches the Jaguar2 and NOT the Jaguar3: an NHM read inside an
armed window leaves it readable but 4 points HIGH (74.8 vs 70.9), where the
JGR3 map truncates it to the 2 ms re-arm. Both are spoiled — `ClmWindow`
invalidates either way — but only one of them looks broken, which is why the
rule is enforced per family rather than per symptom.

The `nhm_env` reduction is referenced to the live IGI, so what matters is how
far DIG may walk it. `PhydmWatchdog` clamps to `0x1c`–`0x2a` (14 steps,
`PhydmWatchdog.h`) — but the watchdog is **opt-in** (`DEVOURER_PHYDM_WATCHDOG=1`,
`HalModule.cpp`; the default path runs watchdog-less because its BB traffic cost
4500→1000 TX submits on an 8821 and 2300→0 RX hits on an 8814). A default
session therefore walks IGI not at all, a stiffer reference than Jaguar3's
4-step clamp, so `nhm_env` should behave like the 8812CU rather than the
8822BU — and should degrade toward the 8822BU once the watchdog is enabled and
hands DIG 14 steps of travel. Predicted from those constants, not measured.

## AC lifetime (`tx.data_on_be`, `tx.ac_lifetime_us`)

Applied on the RTL8812A with three bulk-OUT endpoints only (`ac_queue_ok`);
the 8814A, 8821A and two-endpoint parts log that the knobs are not applied.
DATA moves to QSEL 0 (BE) and the LOW endpoint; VI goes to NORMAL and
VO/MGT/HIGH to HIGH, as `_InitNormalChipThreeOutEpPriority_8812AUsb` maps
them. The lifetime registers are the halmac addresses; the unit is 256 us by
measurement, not the 128 the vendor header names.

Measured on one 8812AU sending to an MT7612U, channel 100, near field:

| | Stock upstream | This change, knobs off | Knobs on |
|---|---|---|---|
| Peer never ACKs, delivered age max (10 s run) | 3.3 s | 3.1 s | 21.0 ms at 20000, 68.8 ms at 68000 |
| Saturated MCS2, Mbit/s (7 s runs, interleaved) | 9.5 / 10.6 / 10.9 | 10.1 / 10.7 | 11.6 / 11.8 |
| Unicast CCX reports seen as unicast | 0 | all | all |

Stock's age grows with the run (9.5 s in a 14 s run): frames wait out every
retry. Run-to-run spread on the saturated cell is about 2 Mbit/s.

The cost: with the knobs on, a frame still queued at the lifetime is dropped,
including one a slow-to-ACK peer would still have taken, and the drop is
reported nowhere; count delivery at the receiver (the full statement is at
`DeviceConfig::Tx::ac_lifetime_us`).

Clearing the BE TXOP (the "fast retry" the halmac generations can pair with
a lifetime) is deliberately not offered here: on the 8812A it cut saturated
MCS2 to 3.5-10.1 Mbit/s, and the lifetime alone holds the bound (68.7 ms
with it, 68.9 ms without).

The TX descriptor's BMC bit follows addr1 on the 8812A. It was a constant 1,
which the firmware echoes into the CCX report, so every unicast report read
as a group frame. Delivery and retries were unchanged across the switch. The
8814A and 8821A keep the constant: unmeasured.
