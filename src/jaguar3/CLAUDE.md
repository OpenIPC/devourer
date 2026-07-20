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
