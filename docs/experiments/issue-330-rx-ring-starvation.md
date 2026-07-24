# Async RX-ring residual loss on a constrained host (issue #330)

Instrumentation, reproduction, and candidate fixes for the ~1–2 % residual RX
loss a MediaTek phone (RTL8812AU @ USB2) shows on the async multi-URB RX ring —
the PixelPilot#6 follow-up. The prior investigation established the mechanism was
*possible* but could not reproduce it at realistic load on a fast x86 rig. This
campaign built proper mechanism telemetry, reproduced the starvation on a host we
drive, implemented three ring-servicing fixes, and root-caused why the bench
cannot resolve the end-to-end delivery benefit.

## The mechanism

`UsbTransport::rx_loop` posts N bulk-IN URBs; `devourer_rx_cb` runs the consumer
(`on_data` → `FrameParser` → the caller's sink) **inline on the libusb pump
thread, before resubmitting the URB**. If the consumer stalls or the pump thread
is preempted, no URB is re-armed, the posted-URB depth collapses, and the chip
RX FIFO overflows — dropping frames. `FrameParser` returns `Packet.Data` as a
*view into the URB's DMA buffer* across every aggregated subframe, so any fix
must not reuse that buffer until the consumer is done with it.

## Instrumentation (all off by default; the async path is unchanged when unset)

| Knob / event | What |
|---|---|
| `rx.ring` (`DEVOURER_RX_RING_MS=N`) | periodic ring telemetry: `armed` (posted-URB depth), `min_armed`, `cb_max_us` (worst inline consume), `resubmit_fail`, `completions`/`empties`, `pool_free`, `qdepth` |
| `DEVOURER_RX_MODE` | `async` (default) / `sync` (blocking single-buffer reference) / `reorder-pool` / `spsc-fat` / `decoupled` (unimpl.) |
| `DEVOURER_RX_POOL_SPARE=K` | spare buffers for the pool modes |
| `DEVOURER_RX_SINK_SPIN_US` | busy-spin per delivered frame (models the wfb consumer cost) |
| `DEVOURER_RX_SINK_STALL_MS` / `_EVERY` | a periodic consumer hiccup (GC/scheduler pause) |
| `rx.seq` (`DEVOURER_RX_PCTR`) | lean per-frame event: the txdemo payload counter + `tsfl` + crc — ground-truth delivery |
| txdemo `DEVOURER_TX_BURST_ON_MS` / `_OFF_MS` | keyframe-burst emulation (reuses the QoS-Data u32 counter stamp) |

Harness `tests/rxq_starve.sh` (one run per invocation: TX flood + constrained RX
capture, with `taskset`/`nice`/`chrt` clamps, continuous `BUSY_THREADS`, and an
intermittent RT-priority preemptor `tests/rxq_preempt.c` modelling a VR
compositor). Analyzer `tests/rxq_analyze.py` computes delivery from the `pctr`
range (independent of `tx.stats`), a gap-width histogram, the ring telemetry, and
a differential across runs.

## Discriminating host-starvation from RF loss

- The `armed` counter decrements at callback *entry*, not physical URB
  completion, so with a single pump thread it rarely reaches 0 — a weak depth
  proxy. `cb_max_us` (ms-scale under a stalled consumer vs ~100 µs under RF loss)
  is the robust single-run signal.
- Catastrophic RF loss *also* produces wide multi-frame gaps, so gap width alone
  false-positives. **The definitive method is differential**: hold the RF link
  constant, vary the host lever, and attribute the delivery drop to the host.

## Reproduction (host we drive)

Desktop RTL8812AU (`0bda:8812`, USB2 — the reporter's exact chip), 2.4 GHz, RF
held constant:

| condition | delivery | cb_max |
|---|---|---|
| urbs=2, spin=500 µs, no clamp | 70 % | 1.5 ms |
| urbs=2, spin=500 µs, `taskset -c 0` + 4 busy threads | **24 %** | **23 ms** |

**+46 pts host-induced loss** at a modest consumer cost, purely by removing CPU
headroom — the lever the prior x86 attempts lacked. The pump thread is
descheduled ~23 ms behind the busy threads (≈ a VR compositor frame). Shrinking
the ring alone (urbs 1→8 at modest spin) did **not** reproduce it — the x86 host
re-arms even a depth-1 ring fast enough; you must also remove the headroom.

## Candidate fixes

- **reorder-pool** — swap a spare buffer onto the wire and resubmit *before*
  consuming the received one (zerocopy-safe). Helps only when the pump is
  **scheduled** but the consume blocks resubmit; the re-arm still runs on the
  pump, so it does not help pump preemption or a throughput deficit.
- **spsc-fat** — the pump only reaps + re-arms + enqueues; a separate consumer
  thread runs `on_data`. Keeps the ring armed through a **consumer** stall,
  converting chip-FIFO overflow (dropped) into bounded host-queue backlog
  (delayed) up to the pool depth. Under a periodic 100 ms stall its `qdepth` grew
  to the pool cap and `pool_free` cycled fully — the queue demonstrably absorbed
  the backlog the async ring drops.
- Neither fixes a genuine **throughput deficit** (consumer permanently too slow):
  that needs more CPU or a faster consumer, not buffering.

## Why the bench cannot resolve the ~1–2 % delivery benefit

1. **Noise floor.** SNR pins at ~4–5 dB across every 2.4 GHz channel and TX-power
   offset (RSSI swings −30…−46 dBm but SNR does not), USB3/switching
   self-interference between the two bench adapters. 6M delivery ceilings ~65–75 %
   — small host effects sit under the RF variance. 5 GHz does not close here.
2. **Large ring buffering.** 16 KB URBs (8 KB device-side aggregation on USB2)
   hold tens of 256-byte frames each, so even a shallow urbs=2 ring buffers tens
   of ms and absorbs typical transient stalls in *all* modes. Only extreme
   continuous conditions exceed the floor, and those are throughput deficits no
   ring change fixes.

This confirms and root-causes the prior "rig unsuitable" finding. The fixes are
implemented and their mechanisms proven to engage (telemetry), but the end-to-end
delivery gain is measurable only on a clean-RF link (conducted / attenuators —
absent on this rig) or the actual constrained device.

## The Meta Quest 3 (Snapdragon XR2 Gen 2, Android)

A genuinely constrained mobile host, driven **fully headlessly from the desktop**
— no in-headset interaction. The vehicle is a minimal auto-grant APK
(`tests/quest_apk/`, hand-built with `aapt`/`d8`/`apksigner`) modelled on how
PixelPilot avoids the USB permission dialog: a `USB_DEVICE_ATTACHED`
intent-filter + `device_filter` (VID 0x0bda / PID 0x8812) auto-grants USB
permission on attach, and a tiny JNI `fork()`+`execve()` helper hands the
`UsbDeviceConnection` fd to `rxdemo` (Android's `ProcessBuilder` closes fds ≥3
before exec, so the fd must be inherited across an in-process fork, not a
spawned child). This also required porting the Android fd path
(`libusb_wrap_sys_device` + `LIBUSB_OPTION_NO_DEVICE_DISCOVERY`) into
`examples/rx/main.cpp` — it previously existed only in the TX demo; without it
libusb's device enumeration is SELinux-denied under the `untrusted_app` domain.
Harness: `tests/quest_rxq_run.sh` (desktop TX flood + per-mode capture + pull)
and `tests/quest_rxq_analyze.py` (delivery from the 802.11 `seq_num` sequence +
ring telemetry).

**Bench traps found and pinned:** the Quest's own adb-over-WiFi rides 5180 MHz
(**channel 36**), so any 5 GHz experiment self-desenses the USB adapter inches
away in the same chassis — the experiment must run on 2.4 GHz (ch 6). The 8821AU
1T1R nano airs but couples poorly to the Quest adapter; the 2T2R flooder reaches
it. 1M CCK from the flooder airs but the Quest decodes ~0; **6M OFDM** works.
Off-head standby drops adb-WiFi (keep-awake: `prox_close` broadcast +
`svc power stayon`).

**What the constrained host confirmed.** The slow-consumer-on-pump-thread signal
is real and *large*: `cb_max_us` reaches **1–1.8 ms at idle and ~15 ms under a
1.5 ms/frame consumer spin**, vs `< 10 µs` on the desktop. Android also denies
`dev_mem_alloc` to the app sandbox, so the zerocopy DMA ring falls back to heap
buffers — the exact constrained condition the phone reports run under.

**What it did not show: a delivery benefit from the ring-servicing mode.** Across
three load regimes, `async` vs `reorder-pool` vs `spsc-fat` are within
run-to-run noise (Δ swings ±1.5 pts with no consistent ordering over repeats):

| regime | delivery (all modes) | telemetry |
|---|---|---|
| idle (`spin=0`, ~667 fps) | ~66 % | `cb_max` 1–1.8 ms, `min_armed` 3, `resubmit_fail` 0 |
| sustained overload (`spin=1500 µs`) | ~38 % | inline modes' pump `completions` collapse to ~1.2 k (blocked ~12 ms/frame); `spsc-fat`'s pump keeps draining (~16.7 k) — **same delivery** |
| transient burst (`3–4 ms` on / `30 ms` off + moderate spin) | ~63–66 % | Δdeliv across reorder-pool/spsc-fat swings ±1.5 pts over repeats = noise |

The reasons mirror the desktop: at low load the ~34 % loss is the marginal-RF
baseline (near-field 2.4 GHz here) and the 4-URB(+spare) ring absorbs the
transient stalls (`min_armed` never below 3); at high load the consumer is
throughput-deficient (`spin × rate > 1 CPU`) and no buffering strategy recovers
a sustained deficit — `spsc-fat` provably keeps the pump draining but delivery is
unchanged because the *consumer* is the wall. The narrow window where a deeper
pool should win (bursts that overflow 4 URBs yet fit the pool, with average
headroom to drain) did not produce a signal above the RF-baseline noise.

**Conclusion.** The three fixes are correct and their mechanisms provably engage
(telemetry), and they ship **off by default** (`DEVOURER_RX_MODE=async`
unchanged). But neither the desktop nor the Quest attributes the reporter's
~1–2 % residual to ring-servicing strategy: at realistic load the ring is not the
bottleneck (RF or consumer-throughput is), and the deep 16 KB-URB ring (#317)
already absorbs the transient stalls a shallower ring would drop. The shippable
product is therefore the **`rx.ring` telemetry as a field diagnostic**: if a
reporter's own phone shows `cb_max_us` / `qdepth` spiking coincident with
keyframe loss, the loss is consumer/pump-limited and `spsc-fat` is the remedy;
if not (the likely case), the residual is a CPU-provisioning limit that no ring
change fixes. Enablement + autonomous-driving details: `tests/quest_rxq.md`.
