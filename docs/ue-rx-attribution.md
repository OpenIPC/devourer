# Per-UE RX attribution (M0 contract 4)

`GetRxQuality()` is device-wide by design: one draining accumulator fed by
every decoded frame, whoever sent it. A cell scheduler adapting per-UE rate and
power needs the same windowed statistics **attributed to each transmitter** —
that is `devourer::cell::UeRxAttribution` (`src/cell/UeRxAttribution.h`), the
seed of the scheduled-MAC `UeRegistry` (5G-NR RAN epic, `src/cell/` = the
per-cell DU library).

## The contract

Pure caller-side logic — the device RX loops are untouched and `GetRxQuality`
stays the radio-wide diagnostic. Everything needed is already per-frame in the
`Packet` callback:

- **key** — the transmitter address (802.11 addr2/TA), extracted by
  `cell::extract_ta`: bytes [10..16) of the MPDU for every frame type except
  the two control subtypes that end at addr1 (CTS, ACK).
- **values** — `rx_pkt_attrib`'s path-A RSSI/SNR/EVM plus the hardware RX
  timestamp `tsfl`, folded with the exact `RxQualityAccumulator` conventions
  (`rssi_raw <= 0` is not a sample; SNR/EVM folded only when present; passive
  noise floor = `(rssi_raw − 110) − snr_raw/2`).

`add()` (or `add_mpdu()`, which extracts the TA itself) per frame;
`snapshot()` drains the whole table into one `UeRxWindow` per TA (delta
semantics, like `GetRxQuality`) with converted units, window mean/extremes and
`last_tsfl` for staleness. The table is bounded (default 64 TAs per window);
overflow frames are counted in `evicted_frames`, never silently lost.

## Measured (bench)

`tests/ue_rx_attribution_check.sh`: two transmitters with distinct unicast SAs
(8812AU at a 2 ms inter-frame gap, 8822CU at 8 ms) against one `ue_rx_probe`
witness (8812BU), 12 s. The probe attributed the streams separately —
TX1 2955 frames at −51 dBm mean, TX2 741 frames at −44 dBm mean, a 4.0×
count ratio exactly matching the 4× cadence ratio — confirming per-UE frame
counts and per-UE signal statistics don't bleed between transmitters.

## Tooling

- `tests/ue_rx_probe.cpp` — on-air probe: feeds every decoded frame into a
  `UeRxAttribution`, drains once a second, emits one `ue.rx` JSONL event per
  UE per window (`ta`, `frames`, `rssi_dbm`, `rssi_max_dbm`, `snr_db`,
  `snr_min_db`, `evm_db`, `nf_dbm`, `last_tsfl`) plus `ue.rx.evicted` when the
  cap was hit. Build line in the header.
- `tests/ue_rx_attribution_selftest.cpp` — headless ctest guard
  (`ue_rx_attribution_derive`): TA extraction over frame types, folding
  conventions, drain semantics, eviction accounting.
- `tests/ue_rx_attribution_check.sh` — the two-TX on-air validation above.

M1 wraps this into `UeRegistry` (association state, timing advance, and the
per-UE RX window as the link-adaptation input).
