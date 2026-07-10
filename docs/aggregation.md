# Packet aggregation & TX-status reports

Three layers, separable:

1. **USB TX aggregation** — many `[txdesc][frame]` blocks per bulk-OUT URB
   (host↔chip transport only, no on-air change). Shipped, on-air-validated.
2. **Per-frame TX-status reports** — the firmware's CCX report per
   transmission (delivered / retry count / queue time / final rate). Shipped.
3. **802.11 A-MPDU formation** — the MAC aggregating co-queued frames into
   one PPDU (the air-time amortization). EXPERIMENTAL: hardware formation is
   bench-proven via the spike knobs, but net goodput is still gated on an
   aggregate-launch pacing quirk (below), so there is no first-class session
   API yet.

## USB TX aggregation (`send_packets`)

`IRtlDevice::send_packets(TxPacketView*, n)` + `DeviceConfig tx.usb_agg_max`
(env `DEVOURER_TX_USB_AGG`, default 0 = off → per-frame loop, byte-identical
descriptors). Packing rules live in `src/TxAggPlan.h` (pure math, ctest'd):
blocks 8-byte aligned, the FIRST descriptor carries the block count
(dword7[31:24]: `USB_TXAGG_NUM` Jaguar1 / `DMA_TXAGG_NUM` HalMAC — inside the
checksummed span, so agg-num is set before the checksum), and the URB total
never lands on an exact bulk-MPS multiple (the sync bulk path has no ZLP).

Per-family hardware rules (both bench-validated on-air):

- **HalMAC (8822B/8821C/8822C/8822E)**: at most **3 descriptors per bulk
  transfer** (mainline rtw88 `usb_tx_agg_desc_num` / halmac `BLK_DESC_NUM`) —
  the library clamps. Layout is rtw88-parity: no first-block reserve; the
  8-byte PKT_OFFSET shim is inserted only to escape a bulk-boundary total.
- **Jaguar1 (8812A/8811A/8821A/8814A)**: vendor-parity — first block carries
  the 8-byte PKT_OFFSET reserve (dropped at a boundary total), and the OQT
  guard caps descriptor STARTS per bulk window (8812A = 1, 8814A = 3,
  8821A = 6). When the knob is on, bring-up programs the kernel's TDECTRL
  block-desc count to match.

Bench (8822BU → 8812CU, ch6, MCS7, 1026-byte MPDUs, per-frame counter
stamped for uniqueness): batched URBs deliver every frame distinctly
(receptions == uniques). A multi-frame sync URB is flow-controlled by the
chip (~ms scale between URB acceptances under sustained flood), so batching
buys per-frame host/USB overhead, not a higher flood ceiling — at MCS7 the
air is the bottleneck either way. `tests/txagg_bench.sh` is the A/B harness.

Bench-instrumentation trap (cost this project half a day): when TXing
"identical" frames to measure aggregation, stamp EVERY frame uniquely
(txdemo batch mode does, at MPDU bytes 26-29). Stamping one shared buffer per
batch makes every reception of a batch look like the same frame — which is
byte-for-byte indistinguishable from "the chip re-aired block 1 N times".

## TX-status reports (`tx.report`)

`DeviceConfig tx.report` (env `DEVOURER_TX_REPORT`) sets `SPE_RPT` in every
data descriptor; the fw answers each transmission with a CCX report, decoded
at the C2H RX sites into `tx.report` events (`src/TxReport.h`): `state`
(0 = delivered/completed, 1 = retry-drop), `retries` (hardware
retransmissions), `queue_time_raw`, `final_rate`, `bmc`. HalMAC reports echo
the descriptor SW_DEFINE low byte — the devices stamp a rotating 8-bit `tag`
for per-frame correlation. Jaguar1 uses the 6-byte 8812 format (queue time
in 256 µs units; the 8814A fw layout differs and is left to the examples/rx
best-effort decode). Jaguar3 TX-only sessions get reports via the coex
thread's C2H drain; TX+RX sessions via the RX loop.

This is the TX-side link sensor: retry counts feed adaptive rate/power the
way RSSI/EVM feed the RX side.

## A-MPDU (experimental spike surface)

Spike knobs (DeviceConfig debug section, all generations):
`DEVOURER_TX_QSEL=<0..7|0x12>` (descriptor queue),
`DEVOURER_TX_AMPDU="max[/density[/rty]]"` (AGG_EN + MAX_AGG_NUM +
AMPDU_DENSITY + optional per-frame retry-limit override), plus txdemo's
`DEVOURER_TX_QOS_DATA` / `DEVOURER_TX_QOS_NOACK` / `DEVOURER_TX_RA` QoS-Data
frame shape. RX side surfaces `paggr` (rx-desc "inside an aggregate") and
`ppdu_cnt` in `rx_pkt_attrib` and the rxdemo `rx.frame`/`rx.txhit` events.
`tests/ampdu_spike.sh` runs the matrix.

Bench-established (8822BU TX, monitor-inject USE_RATE frames):

- **The MAC aggregates host-pushed frames** on a data queue: QoS-Data TID0 +
  `AGG_EN` → true multi-MPDU aggregates (6 subframes at the default max-time,
  shared RX `tsfl`, `paggr` on every subframe) — broadcast RA included. The
  aggregation engine renumbers subframe sequence numbers consecutively.
- **The MGMT queue (0x12, the monitor-inject default) never aggregates**;
  AGG_EN there wedges the queue.
- **QoS No-Ack policy does NOT stop aggregate retransmission**: the MAC
  retries each aggregate to the retry limit waiting for a BlockAck no
  monitor receiver sends (92% retry-flagged receptions, ~12× re-airings,
  unique goodput 7× below singles). The `rty` component set to 0 suppresses
  it completely (receptions == uniques, 0% retry-flagged) — the
  broadcast/no-ack flavor is `DEVOURER_TX_AMPDU=<max>/<density>/0`.
- **Aggregate-launch pacing — solved by register pokes + feed depth**
  (tests/ampdu_pacing_sweep.sh, tests/ampdu_onair_ab.sh; SDR duty is the
  ground truth — RX-capture comparisons vary wildly between sessions):
  - `REG_AMPDU_MAX_TIME_V1` (0x455): the bring-up value 0x70 paces each
    aggregate launch at ~3 ms; **0x20 unlocks it** (URB acceptance
    ~0.8 ms). Values ≤ 0x08 DISABLE aggregation entirely (frames revert to
    slow singles) — the register is a cliff, not a dial.
  - `REG_SW_AMPDU_BURST_MODE_CTRL` (0x4BC): **clearing it** (the halmac
    bring-up sets BIT6) is worth ~+40% on the aggregated path.
  - Both pokes ride `DEVOURER_REPLAY_WSEQ`, which now also applies at the
    end of InitWrite (TX bring-ups can be swept like RX ones).
  - **Feed depth is the other half**: with the sync 3-frame URB feed the
    queue holds ≤3 frames and the MAC SIFS-bursts single-MPDU aggregates.
    Parallel senders (txdemo `DEVOURER_TX_THREADS=N`; sync bulk from N
    threads queues N URBs in flight) restore multi-MPDU amortization —
    saturating at ~4 threads.
  - On-air result (8822BU, ch149, MCS7/20, 1026-byte MPDUs, B210 duty):
    singles 73.3% duty at ~3500 fps (28.7 Mbps MAC goodput) vs deep-fed
    A-MPDU 79.3% duty at ~4550 fps (**37.4 Mbps, +30%**, +20% frames per
    duty-point). The multiplier grows with PHY rate (overhead fraction);
    a session API should bundle: data QSEL + AGG_EN + rty0 + 0x455=0x20 +
    0x4BC=0 + a ≥4-deep feed.
  - `ppdu_cnt` reads 0 on the 8812CU RX used for the bench; `paggr` +
    `tsfl` clustering are the working RX markers.

## Ack stack (deferred)

Hardware ACK responder mode (the `StartBeacon` MACID+MSR recipe as a
first-class monitor-mode API) and reliable-unicast measurement via TX reports
are the next round; see `docs/ap-mode.md` for the proven responder behaviour.
