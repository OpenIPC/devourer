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

`InitWrite` is ~14k USB control transfers and nothing else (stage timing:
`InitTimer` events `j3hal.*` / `j3init.*`, each carrying both `ms` and the
`xfers` it spent; `bench_init.py` parses these events but reports only `ms`).
Synchronous, a transfer costs 76–80 µs on an embedded host (ssc338q) and
~27 µs pipelined 8-deep — EP0 completes URBs
in submission order, so `UsbTransport` queues writes asynchronously inside
a `write_batch_begin/end` scope and only reads (submitted behind the queue,
waited on their own completion), bulk transfers and `flush_writes` wait.
`InitWrite` runs its whole bring-up in one batch (RAII scope, ended before
the coex thread starts): 1.30 → 0.65 s warm, 2.04 → ~0.7 s cold, one
drone-side unit. **`Init` (RX-only) opens no batch yet** — not measured on a
ground-station card.
Batches are single-threaded by contract. The ms-scale settle delays
(`write_bb` 0xfc–0xfe, `rf_writer` 0xffe, `Halrf8822e::delay_ms`, the efuse
power-cut) flush first.

The RF radio-table load is write-only: bits [31:20] of the direct window
(`0x3c00`/`0x4c00 + addr*4`) read back 0 for all 1540 entries, cold and
warm (one 8812EU unit), so the vendor's `MASK20BITS` read-modify-write
preserved nothing at the price of a synchronous read per entry.

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
