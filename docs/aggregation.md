# Packet aggregation & TX-status reports

Three layers, separable:

1. **USB TX aggregation** — many `[txdesc][frame]` blocks per bulk-OUT URB
   (host↔chip transport only, no on-air change). Shipped, on-air-validated.
2. **Per-frame TX-status reports** — the firmware's CCX report per
   transmission (delivered / retry count / queue time / final rate). Shipped.
3. **802.11 A-MPDU formation** — the MAC aggregating co-queued frames into
   one PPDU (the air-time amortization). Shipped as `SetAmpduMode`
   (`DEVOURER_TX_AMPDU_MODE`); +30% on-air goodput at MCS7/20, more at higher
   rates. Needs a deep TX feed to reach the multiplier (below).

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

## A-MPDU (`SetAmpduMode`)

`IRtlDevice::SetAmpduMode(AmpduMode)` / `ClearAmpduMode()` / `GetAmpduMode()`
(env `DEVOURER_TX_AMPDU_MODE="tid/maxnum[/density[/noack[/maxtime_hex]]]"`,
`src/AmpduMode.h`, all generations) bundle the whole proven recipe in one
call: it marks every data frame aggregatable (data QSEL + AGG_EN +
MAX_AGG_NUM + AMPDU_DENSITY + retry-limit) AND programs the MAC pacing
registers live. The struct defaults are the tuned recipe, so the minimal spec
`0/16` (TID 0, 16 MPDUs) is: density 7, no-ack, max_time 0x20, burst-mode
cleared. Off keeps every TX byte-identical.

The caller still supplies the frame shape (QoS-Data on the mode's TID — txdemo
`DEVOURER_TX_QOS_DATA`) and, critically, a **deep TX feed**: send_packets with
enough frames in flight, or txdemo `DEVOURER_TX_THREADS=N` (N parallel
senders; ~4 saturates). A shallow feed leaves the MAC SIFS-bursting
single-MPDU aggregates — the descriptor says "aggregatable" but there's
nothing co-queued to aggregate with.

The lower-level spike knobs stay as an experimentation layer that composes on
top: `DEVOURER_TX_QSEL=<0..7|0x12>` (raw QSEL), `DEVOURER_TX_AMPDU=
"max[/density[/rty]]"` (raw AGG fields), both applied AFTER the product mode
in the descriptor. RX side surfaces `paggr` (rx-desc "inside an aggregate")
and `ppdu_cnt` in `rx_pkt_attrib` + the rxdemo `rx.frame`/`rx.txhit` events.
`tests/ampdu_spike.sh` runs the discovery matrix; `tests/ampdu_pacing_sweep.sh`
the register sweep; `tests/ampdu_onair_ab.sh` the SDR A/B.

What the sweep + on-air validation established (8822BU TX, monitor-inject
USE_RATE frames; SDR duty is the ground truth — RX-capture goodput varies
session to session):

- **The MAC aggregates host-pushed frames** on a data queue (QoS-Data TID0 +
  AGG_EN), broadcast RA included; the aggregation engine renumbers subframe
  seqs consecutively. **The MGMT queue (0x12, the monitor-inject default)
  never aggregates** — AGG_EN there wedges the queue.
- **QoS No-Ack does NOT stop aggregate retransmission**: without a per-frame
  retry limit the MAC re-airs each aggregate to the retry limit waiting for a
  BlockAck no one sends (92% wasted re-airings). `no_ack` = retry-limit 0
  airs each aggregate once (FEC covers loss, not ARQ) — the broadcast flavor.
- **Pacing registers** (programmed by `SetAmpduMode`): the aggregate-fill
  timer `REG_AMPDU_MAX_TIME` (0x0455 HalMAC, **0x0456 Jaguar1**) — bring-up
  0x70 paces launches at ~3 ms, `0x20` unlocks to ~0.8 ms; a CLIFF, values
  ≤ 0x08 disable aggregation. The burst-mode gate
  `REG_SW_AMPDU_BURST_MODE_CTRL` (0x04BC BIT6, halmac/8814A set it) cleared =
  ~+40%. `AMPDU_DENSITY` counter-intuitively wants 7 (permissive spacing
  forms better aggregates; density 0 measured ~10% slower).
- **Feed depth is half the win**: a shallow sync-URB feed SIFS-bursts
  single-MPDU aggregates; `DEVOURER_TX_THREADS=N` (N URBs in flight)
  restores multi-MPDU amortization, saturating at ~4.
- **On-air result** (8822BU, ch149, MCS7/20, 1026-byte MPDUs, B210 duty):
  singles 73.3% duty vs deep-fed `SetAmpduMode` 79.2% duty — **+30% goodput**
  (RX-capture cross-check: +33% delivered unique frames). The multiplier
  grows with PHY rate (overhead is a bigger fraction at high MCS).
- `ppdu_cnt` reads 0 on the 8812CU RX used for the bench; `paggr` + `tsfl`
  clustering are the working RX markers.

## Hardware ACK responder — reliable unicast

`IRtlDevice::SetAckResponder(mac)` / `ClearAckResponder()` (env
`DEVOURER_ACK_RESPONDER=<unicast mac>`, all generations; `src/AckResponder.h`)
arms the MAC's autonomous ACK engine while monitor RX/injection continue
unchanged: port identity (MACID/BSSID 0x610/0x618 = `mac`) + net_type (0x102
[1:0] = AP). No beacon machinery needed — the identity+net_type pair alone is
the gate (bench-bisected; `StartBeacon`'s extra registers are not required).

With a responder armed, a peer TXing unicast QoS-Data (normal ack-policy) to
`mac` gets the full hardware ARQ loop — SIFS-timed ACKs from the responder,
autonomous retransmission on the TX, both ends host-free.
`tests/ack_responder_check.sh` is the closed-loop A/B, judged by the TX
side's CCX reports: responder ON = 100% delivered at mean 0.4 retries (67%
first-try); OFF = 0% delivered, every frame pinned at the 12-retry limit.
The retry distribution doubles as the per-frame TX-side link-quality sensor.

**The same responder is a hardware BlockAck responder** — no ADDBA session
state, no separate API. The MAC's immediate-response engine generates a
SIFS-timed BlockAck for a received A-MPDU addressed to its MACID exactly as it
generates an ACK for a unicast frame; the gate is the same MACID + net_type
pair. So reliable-unicast **ACKed A-MPDU** works end to end: the TX runs
`SetAmpduMode` with `no_ack = false` (normal ack-policy, retry limit kept) and
the peer runs `SetAckResponder`. `tests/ampdu_ba_check.sh` proves it: with the
responder armed, aggregates deliver at 100% / mean 0.1 retries and ~27× the
throughput of the responder-off case (where every aggregate re-airs to the
retry limit for a BlockAck no one sends). The `no_ack = true` default stays the
broadcast/FEC flavor (OpenIPC wfb — no responder, no re-air storm); `false` is
the reliable-unicast flavor against a BA responder.

THE footgun (three strikes now): every MAC in the loop must be UNICAST (I/G
bit clear) — the responder `mac` (an ACK/BlockAck can't target a group
address), and the TX frame's TA/addr2 (the response's RA is the soliciting
frame's addr2; the canonical TX SA `57:42:75:05:d6:00` is a GROUP address, so
txdemo's QoS shape takes `DEVOURER_TX_SA` to override). A group TA silently
yields retry-limit-pinned reports with the responder perfectly armed.

Opt-in only: arming turns a passive monitor into an active transmitter.
Remaining ack-stack item: a software ARQ policy above the CCX reports (the
hardware ARQ — ACK, BlockAck, autonomous retransmit — is complete).
