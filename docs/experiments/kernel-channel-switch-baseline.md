# Kernel-driver channel-switch baseline

Like-for-like latency and behavior of every channel-switch primitive the
standard Linux/Realtek kernel drivers expose on devourer-supported silicon —
the baseline the firmware-assisted-FHSS investigation measures itself
against. Companion harness: `tests/kchansw_bench.py` (orchestrator),
`tests/kchansw_trace.py` (ftrace session), `tests/kchansw_inject.py`
(tagged-frame injector), `tests/kchansw_analyze.py` (offline analysis +
gate verdicts). Devourer's own numbers for the same operation:
[fhss.md](../fhss.md) — FastRetune 0.55–2.5 ms per generation.

## Method

One timeline, three instruments:

- **Kernel side**: ftrace with `trace_clock=mono` (== `CLOCK_MONOTONIC` ==
  the harness's `time.monotonic_ns()`), the cfg80211 `rdev_*` and mac80211
  `drv_*`/queue trace events, and `kchansw/` kprobe pairs on the driver's
  internal switch functions. Host-command latency = the `rdev_*`
  entry→return pair, i.e. the nl80211 request as the driver experiences it
  (the `iw` exec overhead is excluded by construction). A `trace_marker`
  write brackets every switch for segmentation.
- **On-air oracle** (mandatory — an API success without destination-channel
  RF evidence is a *failed* measurement): two devourer `rxdemo` receivers
  pinned to the source and destination channels
  (`DEVOURER_STREAM_OUT=1`, SA-gated via `DEVOURER_RX_AGG_SA`), stdout
  stamped per line at read time. Per-frame placement does not use the noisy
  pipe stamps: each oracle's MAC-latched µs RX timestamp (`tsfl`) is mapped
  to host time by a robust linear fit (Theil-Sen + inlier least-squares;
  observed residual σ 70–320 µs, slope error < 30 ppm), so frame times are
  chip-clock-precise. **Dead air** per switch = first frame decoded on the
  destination channel minus last frame decoded on the source channel.
- **Traffic**: a 2 kHz AF_PACKET injector on the DUT's monitor interface;
  every frame carries a payload counter + send timestamp (decodable from
  the oracle's `rx.frame` body hex, immune to kernel sequence-number
  rewrites) — the loss/duplication ledger. For CSA the AP's own beacons are
  the stream; for the scan row the DUT's probe requests are.

Instrumentation overhead, measured by an instrumented/bare A/B of the same
20 switches: +2.5 ms on a ~114 ms operation (~2 %).

Gates (issue criteria, applied to on-air dead-time p99 per config):
**< 5 ms** advance as an FHSS primitive; **5–20 ms** slow slot-hopping
only; **> 20 ms** control row.

## Capability matrix

DUT silicon: RTL8822BU (TP-Link Archer T3U, `2357:012d`). Kernel
6.18.33-1-lts, regdom US. Every row verified live on this rig (evidence in
parentheses).

| Primitive | rtw88_8822bu (in-tree, mac80211) | rtl88x2bu vendor (cfg80211) | rtw89_8852bu (in-tree, RTL8852BU) |
|---|---|---|---|
| Monitor set-channel | functional (measured below) | functional (measured below) | functional (measured below) |
| AP + CSA | measured below (wpa_supplicant AP mode) | not attempted — no `channel_switch` cfg80211 op in the fork | measured below (chanctx driver) |
| Remain-on-channel | mac80211 software ROC (driver has no `remain_on_channel` op — module symtab) | `.remain_on_channel` op functional (measured below, trace-only) | hardware ROC (`rtw89_ops_remain_on_channel` in symtab) |
| Single-freq active scan | measured below (`drv_hw_scan` fw-offload) | not measured | not measured |
| TDLS channel switch | **unsupported** — no `tdls_channel_switch` op in symtab, zero TDLS mentions in `iw phy` | code present, compile-gated `CONFIG_TDLS=y` → `rtw_hal_ch_sw_oper_offload` → **H2C 0x1C** `H2C_CHNL_SWITCH_OPER_OFFLOAD`; runtime-gated `rtw_wifi_spec=1`; end-to-end **blocked-hardware-absent** (needs two associated STAs + AP; no helper AP on rig this session) | no `tdls_channel_switch` op |
| MCC / FCS | **not implemented** (no chanctx ops; "interface combinations are not supported") | machinery live (measured below): MCC build auto-creates the second vif, STA associates, `mcc/` proc knobs accept `mcc_enable=1`, policy table exposed (100 ms interval, 20/80 duration splits); engine idle until *both* vifs are linked on different channels — driving that is exp-3 (#271) | MCC exists (chanctx) but is P2P-driven — documented-only |
| `set_txpower` | **rejected** `EOPNOTSUPP` (no userspace TX-power control) | not probed | not probed |

Vendor-build facts: the OpenHD `rtl88x2bu` fork builds `88x2bu_ohd.ko` on
6.18 after one compat fix (cfg80211 `tdls_mgmt` gained `int link_id` ≥ 6.0
— the TDLS code had never been compiled on the 6.x branch) and on the
measurement VM's 5.15 after a second (`timer_container_of` is ≥ 6.16;
older kernels spell it `from_timer`); the TDLS and MCC firmware-offload
families cannot coexist in one binary, and the MCC make variant renames
the module `8812bu.ko`.

**The vendor driver must be measured in a VM.** With the TDLS-variant
module loaded (no `CONFIG_CONCURRENT_MODE`), `iw dev <if> interface add
<x> type monitor` — adding *any* second vif — deadlocks the kernel inside
the driver's add-interface path with RTNL held: ssh dies, `dmesg` blocks,
nothing panics, only a power cycle recovers. This livelock took the whole
host down twice before a step-by-step probe (VM-contained, reproduced on
demand) isolated it to that single operation — the kprobes and the switch
churn are innocent. `tests/kchansw_vm.sh` therefore runs the vendor phase
inside the pinned-kernel VM (Ubuntu 22.04 / 5.15) with the DUT
USB-passed-through (still pinned to its USB2 port — passthrough URBs ride
the host controller) and the oracles left on the host; a repeat wedge
costs a `virsh reset`. The bench maps the VM's CLOCK_MONOTONIC onto the
host timeline with a min-RTT-midpoint ssh probe (±0.15 ms, drift-checked
before and after each config); dead-air numbers never cross that bridge —
both oracles are host-side.

## Results — rtw88_8822bu (in-tree, kernel 6.18.33)

DUT on its USB2-HS link (see Endurance for why that matters). Warm-ups
excluded; every row below carries on-air RF evidence; zero failed switches
in all set-channel configs.

### Ordinary set-channel (monitor interface)

| config | n | dead air med | p90 | p99 | max | host-cmd med | driver med | gate |
|---|---|---|---|---|---|---|---|---|
| 36↔40, 20 MHz | 2000 | **52.0 ms** | 57.7 | 60.6 | 86.6 | 84.0 ms | 42.7 ms | control |
| 1↔6, 20 MHz | 300 | **6.1 ms** | 11.8 | 16.6 | 22.7 | 84.4 ms | 42.2 ms | **slow-slot-only** |
| 36↔44, HT40+ | 300 | 55.4 ms | 56.2 | 57.7 | 60.4 | 83.6 ms | 42.2 ms | control |
| 36↔6 cross-band (control) | 50 | 57.6 ms | 60.9 | 64.3 | 66.6 | 83.7 ms | 42.2 ms | control |

The structure is striking: the driver-internal call (`rtw_set_channel`
kprobe pair) costs a band-independent ~42 ms and the nl80211 round trip
~84 ms — but the RADIO's actual dark time is 6 ms intra-2.4 GHz versus
52 ms whenever 5 GHz is involved on either end (cross-band is *not*
bimodal: leaving **or** entering 5 GHz pays the full price, so the cost sits
in the 5 GHz RF-path programming, not in a generic "band switch"). On
2.4 GHz the radio is already airing frames ~78 ms before the nl80211 call
returns. Host-command latency alone (the number an API-only benchmark
would report) overstates the 2.4 GHz RF transition by 14× and understates
nothing on 5 GHz — the on-air oracle is not optional.

The monitor set-channel path involves **no firmware H2C**
(`rtw_fw_send_h2c_command` count = 0 per switch) and does **not** stop
mac80211 TX queues — frames submitted during the switch are simply lost
(gap ledger: ~89 lost frames per 5 GHz switch at 2 kHz ≈ 45 ms of blind
TX, independently cross-checking the oracle's dead-air figure).

### Remain-on-channel (software ROC — no driver op)

Trace-only timing (single-vif driver: no parallel monitor vif possible, so
no on-air evidence; `need_rf` waived for this primitive):

| metric | 36→ROC(40) n=1000 | 40→ROC(36) n=300 |
|---|---|---|
| request → ready-on-channel | 45.9 ms med / 54.5 p99 | 46.7 / 54.9 |
| granted dwell (20 ms asked) | 22.1 ms med | 22.0 ms |
| TX queues stopped (total blackout) | 116.4 ms med / 127.6 p99 | 117.1 / 128.1 |

ROC *does* stop queues (unlike set-channel), and the ~46 ms grant latency
matches the one-way 5 GHz retune cost seen above. A 20 ms off-channel
errand costs ~116 ms of link blackout round-trip.

### Single-frequency active scan (`drv_hw_scan` firmware offload)

The closest in-tree cousin of firmware-assisted hopping — and the slowest
path measured: request → first probe request decoded on the target channel
**2.23 s median** (n=295, RF-evidenced), host-cmd 2.15 s, queues stopped
226 ms per scan. The firmware scan machinery is built for background
discovery, not agile retuning. Control row.

### AP/CSA — policy-rejected (no latency row possible)

hostapd brings a beaconing AP up on the rtw88 DUT without complaint (open
BSS, ch 36 and ch 1 both verified on-air: the source-channel oracle decodes
the beacon stream), but every `chan_switch` is refused **before any
nl80211 call**: hostapd logs `CSA is not supported` (the wiphy never
advertises the AP-CSA driver flag) and wpa_supplicant's AP-mode
`CHAN_SWITCH` fails the same way. Zero `rdev_channel_switch` events reach
cfg80211 across all attempts; the destination-channel oracle hears
nothing. Classification: **policy-rejected at the driver-capability
level** — CSA on this driver cannot even be benchmarked, let alone used.

### Cold first switch

Each in-run wedge recovery is a driver-cold bring-up (module reload or
deeper) immediately followed by an instrumented switch, so the endurance
data doubles as the cold sample: post-recovery first switches show the
same ~52 ms dead-air profile as steady-state — the expensive part of a
cold start is the firmware download + bring-up (seconds), not the first
retune afterwards.

### Endurance and the USB-link dependence

The issue's endurance criterion (no cumulative wedge across 1,000
switches) fails outright on the T3U's native USB3/SuperSpeed link and
passes completely on USB2:

- **USB3 (Bus 10 root port)**: sustained 2 kHz monitor injection + switch
  churn wedged the DUT repeatedly — first as EPROTO register-write
  failures escalating to full USB disconnect, then as a *silent-TX* state
  (sends accepted, nothing airs, dmesg clean) with a ~15–30 k-frame
  cadence, and terminally as a uPD720201 host-controller hang (xhci
  re-probe -110) that required a PCI bus reset + D3hot→D0 bounce + rescan
  plus a 12 s per-port power-off. The recovery ladder in
  `kchansw_bench.py` (re-probe → module reload → authorized/VBUS cycle →
  bus/controller revive) automates all four rungs.
- **USB2 (same adapter, companion bus after the hard reset)**: 2000 + 300
  + 300 + 50 + 1010 + 310 switches with **zero wedges**.

For comparison, devourer's own TX path sustains this duty for hours on the
same silicon over USB3. An injection-heavy kernel-driver deployment on
this hardware should pin the adapter to USB2 or expect the wedge ladder.

Instrument note: a low-rate B210 partial-band energy capture during the
5 GHz run confirms the macroscopic on/off cycling (37 dB contrast, zero
overflows) but near-field adjacent-channel splatter fragments the silent
periods, so the energy view cannot resolve the decode-vs-energy edge bias
below ~10 ms; the decode oracle remains the primary instrument
(`tests/kchansw_b210_gap.py`).

## Results — rtl88x2bu vendor driver (VM, kernel 5.15, `rtw_wifi_spec=1`)

Same silicon, same channels, same oracles as Phase A — different driver.

### Ordinary set-channel (monitor interface)

| config | n | dead air med | p90 | p99 | max | host-cmd med | gate |
|---|---|---|---|---|---|---|---|
| 36↔40, 20 MHz | 998/1000 | **1.08 ms** | 1.68 | 4.42 | 32.9 | 66.3 ms | **advance** |

The headline of the whole experiment: the vendor driver's internal switch
path (`rtw_set_chbw_cmd` → `rtw_hal_set_chnl_bw` →
`phy_SwChnlAndSetBwMode8822B`) goes dark for a median **1.08 ms** on the
same 36↔40 5 GHz hop that costs the in-tree rtw88 driver 52 ms — a 48×
difference, and *under* the < 5 ms FHSS gate at p99. The ~66 ms nl80211
round trip is pure bookkeeping: the radio is already airing on the
destination channel ~65 ms before the `iw` call returns. The gap ledger
cross-checks (731 lost frames over 998 switches at 2 kHz ≈ 0.37 ms mean
blind time), and both oracle fits are chip-clock tight (σ 40/280 µs,
< 20 ppm). Two switches failed the RF-evidence gate; max observed dead
air 32.9 ms.

This is direct evidence that the RTL8822BU hardware/firmware retune is
FHSS-capable through a kernel driver, and that rtw88's 52 ms lives in
that driver's software sequence (RF-K/calibration replay on the 5 GHz
path), not in the silicon.

### Remain-on-channel (driver `.remain_on_channel` op, trace-only)

No second vif is possible on this build (see the deadlock note), so no
injector — trace spans only (n=300, VM clock, spans shift-invariant):
grant (rdev entry → `cfg80211_ready_on_channel`) median **0.2 ms** /
p90 0.4 / p99 99.6; granted dwell median 62 ms for 20 ms requested; full
request→expired cycle median 67 ms. The instant grant is consistent with
the ~1 ms RF switch; the dwell over-run and the p99 tail (~100 ms, an
internal command-queue beat) make it a scheduler curiosity rather than a
hopping primitive.

### TDLS / MCC offload paths

TDLS channel-switch offload stays **blocked-hardware-absent** end-to-end
(needs two associated STAs under an AP), but the target is fully named
from source + build evidence: `rtw_hal_ch_sw_oper_offload`
(`hal/hal_com.c`) composes **H2C 0x1C `CHNL_SWITCH_OPER_OFFLOAD`**, built
and runtime-armed in the measured module. The MCC attempt got further
than expected (capability matrix): everything up to `mcc_enable=1` works;
the firmware engine (H2C family 0x10 `MCC_LOCATION`/`FCS_RSVDPAGE`, 0x11
`FCS_INFO`, 0x18 `MCC_CTRL`, 0x19 `TIME_SETTING`) idles until both vifs
are linked on different channels.

## Results — rtw89_8852bu (in-tree, RTL8852BU, kernel 6.18.33)

Quick pass (×200 per primitive, host venue — the in-tree driver showed no
host-side instability in ~700 switches):

| primitive | n | dead air med | p90 | p99 | max | host-cmd med | gate |
|---|---|---|---|---|---|---|---|
| set-channel 36↔40 | 200/200 | 31.5 ms | 33.8 | 36.3 | 39.0 | 66.3 ms | control |
| CSA 36↔40 (AP, count=1) | 116/200 | 102.9 ms | 203.8 | 205.8 | 205.8 | 66.4 ms | control |
| ROC 36→40 (hardware) | 200/200 | trace-only | | | | 52.6 ms | — |

- **CSA actually works on rtw89** — hostapd accepts `chan_switch`, the
  full `rdev_channel_switch` → `ch_switch_notify` chain fires (~70 ms),
  and the beacon stream provably moves channel. rtw88's "CSA is not
  supported" is that driver's missing capability flag, not a Realtek-USB
  constraint. The dead air is TBTT-quantized: median ≈ one 100 ms beacon
  interval, p90+ ≈ two (a missed TBTT), and 84/200 attempts left no
  destination-channel RF evidence in the dwell window — usable for slow
  planned migration, useless for hopping.
- Monitor set-channel is a uniform ~31 ms dark (tighter distribution than
  rtw88's 52 ms, still 6× the FHSS gate) with zero per-switch H2C — the
  channel programming is register-direct here too.
- Hardware ROC grants in 52.7 ms med (p99 57) and dwells 74.5 ms for the
  20 ms asked — the off-channel errand is cheaper and much more
  deterministic than rtw88's software ROC (46 ms grant, 116 ms blackout)
  but still far from agile.

## Go/no-go

Gate verdicts across every measured row:

| driver / primitive | on-air p99 | verdict |
|---|---|---|
| **vendor rtl88x2bu set-channel (5 GHz)** | **4.4 ms** | **advance** |
| rtw88 set-channel 2.4 GHz | 16.6 ms | slow-slot-only |
| rtw88 set-channel 5 GHz / 40 MHz / cross-band | 58–64 ms | control |
| rtw89 set-channel 5 GHz | 36.3 ms | control |
| rtw88 scan / ROC, rtw89 CSA / ROC | ≥ 57 ms | control |
| rtw88 CSA | — | policy-rejected |

**Go.** The experiment's core question — can a standard-driver path
approach FHSS-grade switching? — has a measured *yes*: the RTL8822BU
retunes in ~1 ms when the driver's software sequence stays out of the
way. The nominations for the follow-on experiments:

- **Exp-2 (#270, TDLS-style offload)**: target
  `rtw_hal_ch_sw_oper_offload` → **H2C 0x1C
  `CHNL_SWITCH_OPER_OFFLOAD`** (vendor `hal/hal_com.c`,
  `include/hal_com_h2c.h`). Compiled, loaded and runtime-armed
  (`rtw_wifi_spec=1`) in the exact module that measured 1.08 ms — the
  offload hands the same fast internal switch to the firmware with a
  channel list and timing, removing even the 66 ms host round trip.
  Devourer's own FastRetune on this chip is 2.5 ms; the kernel-measured
  1.08 ms says there is roughly a millisecond of headroom worth chasing.
- **Exp-3 (#271, MCC/FCS)**: target the **H2C 0x10/0x11/0x18/0x19
  family** (`MCC_LOCATION`/`FCS_RSVDPAGE`, `FCS_INFO`, `MCC_CTRL`,
  `TIME_SETTING`). Everything up to `mcc_enable=1` is verified live;
  the remaining step — both vifs linked on different channels so the
  firmware schedule engages — is precisely that experiment's setup.
- The in-tree drivers are the control arm: nothing they expose (rtw88 or
  rtw89, any primitive) comes within 6× of the gate, and rtw88's 2.4 GHz
  6 ms median is the only slow-slot candidate — band-asymmetric and
  unhoppable on 5 GHz, so not a link design anyone should build on.
