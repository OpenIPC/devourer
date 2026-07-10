# Packet aggregation, TX reports, and the hardware ACK/BlockAck responder

devourer's TX path supports three independent, separable capabilities:

1. **USB TX aggregation** — many `[txdesc][frame]` blocks per bulk-OUT URB
   (host↔chip transport only, no on-air change).
2. **Per-frame TX-status reports** — the firmware's CCX report per
   transmission (delivered / retry count / queue time / final rate).
3. **802.11 A-MPDU formation** — the MAC aggregating co-queued frames into one
   PPDU (the air-time amortization), via `SetAmpduMode`: +30% goodput at
   MCS7/20 on the HalMAC generations, more at higher rates, given a deep TX
   feed. (This is a *goodput* gain — delivered payload — not a channel-occupancy
   one; see the note under the A-MPDU section.)

A fourth capability — the hardware **ACK/BlockAck responder** — makes a monitor
radio auto-acknowledge frames addressed to it, which turns both single-frame
and A-MPDU unicast into reliable (hardware-ARQ) links.

## USB TX aggregation (`send_packets`)

`IRtlDevice::send_packets(TxPacketView*, n)` + `DeviceConfig tx.usb_agg_max`
(env `DEVOURER_TX_USB_AGG`, default 0 = off → per-frame loop, byte-identical
descriptors). Packing rules live in `src/TxAggPlan.h` (pure math, ctest'd):
blocks 8-byte aligned, the FIRST descriptor carries the block count
(dword7[31:24]: `USB_TXAGG_NUM` Jaguar1 / `DMA_TXAGG_NUM` HalMAC — inside the
checksummed span, so agg-num is set before the checksum), and the URB total
never lands on an exact bulk-MPS multiple (the sync bulk path has no ZLP).

Per-family hardware rules:

- **HalMAC (8822B/8821C/8822C/8822E)**: at most **3 descriptors per bulk
  transfer** (mainline rtw88 `usb_tx_agg_desc_num` / halmac `BLK_DESC_NUM`) —
  the library clamps. Layout is rtw88-parity: no first-block reserve; the
  8-byte PKT_OFFSET shim is inserted only to escape a bulk-boundary total.
- **Jaguar1 (8812A/8811A/8821A/8814A)**: vendor-parity — the first block
  carries the 8-byte PKT_OFFSET reserve (dropped at a boundary total), and the
  OQT guard caps descriptor STARTS per bulk window (8812A = 1, 8814A = 3,
  8821A = 6). With the knob on, bring-up programs the kernel's TDECTRL
  block-desc count to match.

A multi-frame sync URB is flow-controlled by the chip (~ms scale between URB
acceptances under a sustained flood), so batching removes per-frame host/USB
overhead rather than raising the flood ceiling — at MCS7 the air is the
bottleneck either way. `tests/txagg_bench.sh` is the single-vs-batch A/B
(8822BU → 8812CU, MCS7, 1026-byte MPDUs): batched URBs deliver every frame
distinctly.

Benchmarking note: to measure aggregation by TXing "identical" frames, stamp
EVERY frame uniquely (txdemo batch mode writes a per-frame counter at MPDU
bytes 26-29). A single shared buffer per batch makes every reception look like
the same frame, which is byte-for-byte indistinguishable from the chip
re-airing one frame N times.

## TX-status reports (`tx.report`)

`DeviceConfig tx.report` (env `DEVOURER_TX_REPORT`) sets `SPE_RPT` in every
data descriptor; the firmware answers each transmission with a CCX report,
decoded at the C2H RX sites into `tx.report` events (`src/TxReport.h`): `state`
(0 = delivered/completed, 1 = retry-drop), `retries` (hardware
retransmissions), `queue_time_raw`, `final_rate`, `bmc`. HalMAC reports echo
the descriptor SW_DEFINE low byte — the devices stamp a rotating 8-bit `tag`
for per-frame correlation. Jaguar1 uses the 6-byte 8812 format (queue time in
256 µs units; the 8814A firmware layout differs and is left to the examples/rx
best-effort decode). Jaguar3 TX-only sessions get reports via the coex thread's
C2H drain; TX+RX sessions via the RX loop.

This is the TX-side link sensor: retry counts feed adaptive rate/power the way
RSSI/EVM feed the RX side.

## A-MPDU (`SetAmpduMode`)

`IRtlDevice::SetAmpduMode(AmpduMode)` / `ClearAmpduMode()` / `GetAmpduMode()`
(env `DEVOURER_TX_AMPDU_MODE="tid/maxnum[/density[/noack[/maxtime_hex]]]"`,
`src/AmpduMode.h`, all generations) configure A-MPDU TX in one call: it marks
every data frame aggregatable (data QSEL + AGG_EN + MAX_AGG_NUM +
AMPDU_DENSITY + retry-limit) AND programs the MAC pacing registers live. The
struct defaults are the tuned values, so the minimal spec `0/16` (TID 0,
16 MPDUs) means density 7, no-ack, max_time 0x20, burst-mode cleared. Off keeps
every TX byte-identical.

The caller supplies the frame shape (QoS-Data on the mode's TID — txdemo
`DEVOURER_TX_QOS_DATA`) and, critically, a **deep TX feed**: `send_packets`
with enough frames in flight, or txdemo `DEVOURER_TX_THREADS=N` (N parallel
senders; ~4 saturates). A shallow feed leaves the MAC SIFS-bursting
single-MPDU aggregates — the descriptor says "aggregatable" but there is
nothing co-queued to aggregate with.

Two lower-level per-field knobs compose on top for register experimentation,
applied AFTER the mode in the descriptor: `DEVOURER_TX_QSEL=<0..7|0x12>` (raw
QSEL) and `DEVOURER_TX_AMPDU="max[/density[/rty]]"` (raw AGG fields). The RX
side surfaces `paggr` (rx-desc "inside an aggregate") and `ppdu_cnt` in
`rx_pkt_attrib` + the rxdemo `rx.frame`/`rx.txhit` events.
`tests/ampdu_spike.sh` sweeps the aggregation-condition matrix,
`tests/ampdu_pacing_sweep.sh` the pacing registers, `tests/ampdu_onair_ab.sh`
the SDR A/B.

Hardware behaviour (8822BU TX, monitor-inject USE_RATE frames; SDR duty is the
airtime ground truth):

- The MAC aggregates host-pushed frames on a **data queue** (QoS-Data TID0 +
  AGG_EN), broadcast RA included; the aggregation engine renumbers subframe
  seqs consecutively. The **MGMT queue (0x12, the monitor-inject default)
  never aggregates** — AGG_EN there wedges the queue.
- QoS No-Ack ack-policy does **not** stop aggregate retransmission: without a
  per-frame retry limit the MAC re-airs each aggregate to the retry limit
  waiting for a BlockAck (92% wasted re-airings when no responder exists).
  `no_ack` = retry-limit 0 airs each aggregate once (FEC covers loss, not ARQ)
  — the broadcast flavor.
- **Pacing registers** (programmed by `SetAmpduMode`): the aggregate-fill
  timer `REG_AMPDU_MAX_TIME` (0x0455 HalMAC, **0x0456 Jaguar1**) — the
  bring-up value 0x70 paces launches at ~3 ms, `0x20` paces at ~0.8 ms, and
  values ≤ 0x08 disable aggregation (a cliff, not a dial). The burst-mode gate
  `REG_SW_AMPDU_BURST_MODE_CTRL` (0x04BC BIT6, set by halmac/8814A bring-up)
  cleared is worth ~+40%. `AMPDU_DENSITY` wants 7 (a permissive inter-MPDU
  spacing forms better aggregates; density 0 measures ~10% slower).
- **Feed depth is half the win**: a shallow sync-URB feed SIFS-bursts
  single-MPDU aggregates; `DEVOURER_TX_THREADS=N` (N URBs in flight) restores
  multi-MPDU amortization, saturating at ~4. This works on the HalMAC
  generations (J2/J3), whose TX path is synchronous bulk (each sender thread
  blocks on its own transfer). **Jaguar1 uses the fire-and-forget async TX
  path, which does not parallelize** — a multi-thread feed collapses its
  throughput (8812AU 55 → 14 Mbps at threads=4 with no aggregation at all), so
  the deep feed A-MPDU needs is currently HalMAC-only. Jaguar1's descriptor
  path still marks frames aggregatable, but the multiplier is not reachable
  without a feed rework.
- **The gain is goodput, not channel occupancy.** On-air (8822BU, ch149,
  MCS7/20, B210 duty), aggregation raised delivered goodput **~+30%** (73.3% →
  79.2% duty plus a per-airtime payload increase; RX-capture cross-check +33%
  delivered unique frames). But `tests/bench_onair.py`'s metric — SDR duty ×
  PHY rate — is channel *occupancy*, and these chips already run at ~80% duty
  near the 65 Mbps PHY ceiling, so that number moves only ~+2% (8822BU 51 → 52,
  8812CU 51 → 52). The occupancy metric structurally cannot show A-MPDU's
  payload gain on a near-saturated chip; measure goodput (delivered payload)
  to see it. The multiplier grows with PHY rate, since preamble overhead is a
  bigger fraction at high MCS.
- `ppdu_cnt` reads 0 on the 8812CU RX used for the bench; `paggr` + `tsfl`
  clustering are the working RX markers.

`tests/bench_onair.py --ampdu` measures the singles / feed-depth / A-MPDU
comparison per chip per band (SDR duty), and is the harness the occupancy
numbers above came from.

## Hardware ACK/BlockAck responder — reliable unicast

`IRtlDevice::SetAckResponder(mac)` / `ClearAckResponder()` (env
`DEVOURER_ACK_RESPONDER=<unicast mac>`, all generations; `src/AckResponder.h`)
arms the MAC's autonomous ACK engine while monitor RX/injection continue
unchanged: port identity (MACID/BSSID 0x610/0x618 = `mac`) + net_type (0x102
[1:0] = AP). The identity+net_type pair is the whole gate — no beacon
machinery, no ADDBA session state, no CAM entry.

With a responder armed, a peer TXing unicast QoS-Data (normal ack-policy) to
`mac` runs a full hardware ARQ loop — SIFS-timed ACKs from the responder,
autonomous retransmission on the TX, both ends host-free.
`tests/ack_responder_check.sh` is the closed-loop A/B judged by the TX side's
CCX reports: responder ON = 100% delivered at mean 0.4 retries (67%
first-try); OFF = 0% delivered, every frame pinned at the 12-retry limit. The
retry distribution is the per-frame TX-side link-quality sensor.

The same responder is a hardware **BlockAck** responder: the MAC's
immediate-response engine generates a SIFS-timed BlockAck for a received
A-MPDU addressed to its MACID, on the same MACID + net_type gate. So
reliable-unicast **ACKed A-MPDU** works end to end — the TX runs `SetAmpduMode`
with `no_ack = false` (normal ack-policy, retry limit kept) and the peer runs
`SetAckResponder`. `tests/ampdu_ba_check.sh`: with the responder armed,
aggregates deliver at 100% / mean 0.1 retries and ~27× the throughput of the
responder-off case (where every aggregate re-airs to the retry limit). The
`no_ack = true` default is the broadcast/FEC flavor (OpenIPC wfb — no
responder, no re-air storm); `false` is the reliable-unicast flavor against a
BA responder.

Every MAC address in the loop must be **unicast** (I/G bit clear): the
responder `mac` (an ACK/BlockAck cannot target a group address) and the TX
frame's TA/addr2 (the response's RA is the soliciting frame's addr2). The
canonical TX SA `57:42:75:05:d6:00` is a group address, so txdemo's QoS shape
takes `DEVOURER_TX_SA` to override it — a group TA yields retry-limit-pinned
reports even with the responder perfectly armed.

Arming a responder turns a passive monitor into an active transmitter, so it
is opt-in. The hardware ARQ (ACK, BlockAck, autonomous retransmission) is
complete; devourer layers no software ARQ policy above the reports.
