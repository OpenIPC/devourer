# Realtek MCC / FCS as an FHSS primitive — investigation

Multi-Channel Concurrency (MCC) and its firmware Fast Channel Switch (FCS —
Realtek's "fast channel switch", not the 802.11 frame check sequence) let one
Realtek radio hold two channel contexts and time-share between them under a
firmware schedule. This note determines whether that schedule can be
repurposed as a frequency-hopping primitive, the third in the standard-driver
FHSS series after the
[channel-switch baseline](kernel-channel-switch-baseline.md) and the
[firmware channel-switch offload](kernel-channel-switch-offload.md).

The verdict is up front: **MCC is a two-context time-share for concurrent
connections, not a hopping engine.** Its schedule is coarse (TU-quantized
slots, a fixed ~102 ms period, two contexts, non-groupable channel pairs
only), and the one part of it worth having — the fast firmware RF switch at a
slot boundary — is the same H2C 0x1D switch already isolated in experiment 2
and shipped directly as `DEVOURER_FASTRETUNE_FW`, without MCC's association
and scheduler machinery.

## State machine (vendor `rtl88x2bu`, `hal/hal_mcc.c` + `core/rtw_mlme_ext.c`)

```
  two vifs associate ──► mlme_join_done (2nd vif)
                              │
                     MCC_EN(adapter)?  ── no ──► single-radio union channel
                              │ yes
              rtw_hal_set_mcc_setting_join_done_chk_ch
                              │
              rtw_is_chbw_grouped(chA, chB)?
                     │yes                       │no  (chbw_allow == FALSE)
        one radio covers both          rtw_hal_set_mcc_setting(START_CONNECT)
        (NO MCC — union path)                   │
                                  upload schedule + contexts (H2C, below)
                                                │
                                   C2H MCC_RPT_READY ──► mcc_status = DOING_MCC
                                                │
                          firmware autonomously alternates context 0/1
                          per the policy table; C2H SWITCH_CHANNEL_NOTIFY
                          on each slot boundary
                                                │
                    disconnect / stop ──► H2C MCC_CTRL(totalnum=0xff)
                                          C2H MCC_RPT_STOPMCC ──► teardown
```

The gate that matters for hopping is `rtw_is_chbw_grouped`: MCC starts **only
when the two channels cannot be grouped into one radio channel**. Adjacent
20 MHz channels that fit inside one 40/80 MHz channel (36+40, 36+44, …) take
the single-radio union path and never engage MCC — so the series' nominal
"36↔40" candidate is precisely a pair MCC declines. A non-groupable pair
(different sub-bands, e.g. 36+149, or 2.4 GHz 1+11) is required to reach the
firmware schedule.

## Schedule protocol

Host→firmware, in start order (`rtw_hal_set_mcc_start_setting`):

| step | H2C | id | len | carries |
|---|---|---|---|---|
| contexts | `MCC_LOCATION` | 0x10 | 7 (V2) | reserved-page indices per context (null-data/CTS frame + PHY/IQK snapshot) |
| MAC ids | `MCC_MACID_BITMAP` | 0x16 | 6 | per-context MAC-ID bitmap |
| control | `MCC_CTRL_V2` / `MCC_CTRL` | 0x17 / 0x18 | 7 | order, total contexts, centre ch, primary-ch idx, bw, role, report flags |
| timing | `MCC_TIME_SETTING` | 0x19 | 6 | TSF-sync offset, start-time offset, interval, order-0 duration |
| (V1 IQK) | `MCC_IQK_PARAM` | 0x1A | 7 | per-path IQK cal (V2 offloads to firmware) |

Firmware→host: `C2H_MCC` (0x17). Status codes (`MCC_RPT_*`):
`READY=3` (schedule armed → `DOING_MCC`), `SWICH_CHANNEL_NOTIFY=7` (per
slot-boundary switch — the natural per-transition instrument), `TSF=9`,
`STOPMCC=2` (stop complete), `SUCCESS=0`, `TXNULL_FAIL=1`.

**Policy table** (`mcc_switch_channel_policy_table`, live-read from the DUT,
all values in TU = 1024 µs):

| policy | order-0 duration | TSF-sync | start-offset | interval | guards |
|---|---|---|---|---|---|
| 0 | 20 | 50 | 40 | **100** | 0/0 |
| 1 | 80 | 50 | 10 | **100** | 0/0 |
| 2 | 36 | 50 | 32 | **100** | 0/0 |
| 3 | 30 | 50 | 35 | **100** | 0/0 |

The period is a fixed **100 TU ≈ 102.4 ms** in every policy; the split is
order-0 duration / (interval − duration). The smallest context slot the
firmware exposes is 20 TU ≈ 20.5 ms. There is no dwell below TU granularity
and no path to more than two contexts. Transitions are firmware-internal: the
two contexts are pre-loaded into reserved pages once, and the firmware flips
between them on its own timer — a slot boundary is not a per-slot host H2C, so
the switch itself is at least as fast as the ~1 ms on-chip RF retune measured
in experiment 2 (which uses the same switch engine).

## What was reached on hardware

Measured on the RTL8822BU (T3U) with the MCC-variant vendor build
(`CONFIG_MCC_MODE=y CONFIG_CONCURRENT_MODE`, two auto-created vifs), in the
pinned-kernel VM (the vendor module deadlocks the host — see the offload
note), two host helper APs, and the DUT's two STA vifs associating to them:

- **The MCC capability gate engages.** With two vifs linked on a
  non-groupable pair (2.4 GHz ch1 + ch11), `en_mcc:1` and both vifs hold
  `STA ASOC` stably. On a groupable pair (36+40) the driver takes the union
  path and one vif starves — direct confirmation of the `chbw_grouped` gate.
- **The firmware schedule does not start from the cfg80211 association
  path.** Across every pairing, `mcc_status` stays 0 (never `DOING_MCC`),
  `mcc_rf_ch[0/1]` stay 0, the radio stays parked on one channel, and no
  `MCC_RPT_READY` appears. The start hook lives in the vendor's internal
  `mlme_join_done` flow and, for a plain dual-STA case driven by `iw
  connect`, is never taken to the point of `rtw_hal_set_mcc_setting(START)`.
  The reserved-page context builder emits NOA content for a P2P GO; this
  matches the in-tree rtw89 note that MCC is P2P-driven — the schedule
  appears to require a GO/GC (P2P) anchor, not two open-BSS STAs.

Bringing the schedule to a running state would need the vendor's private
association path (a P2P GO/GC bring-up, or `wpa_supplicant` driving the
vendor SME) — neither reachable in the constrained measurement VM
(no `wpa_supplicant`/`hostapd`, package egress firewalled). Per the
experiment's own rule, no on-air dwell is claimed from a schedule that was
not observed running; the schedule structure above is source-derived and
confirmed against the live policy table, not from an on-air A/B capture.

## Go / no-go

**Reject MCC/FCS as an FHSS hopping primitive.** Even with the schedule
running, it is structurally a two-channel time-share:

- slots are TU-quantized with a ~20 ms floor and a **fixed ~102 ms period** —
  no dwell-1, no agile per-packet hopping (the decision gate's requirement);
- exactly two contexts — no arbitrary-N channel set;
- restricted to non-groupable channel pairs;
- gated behind a full dual-interface association (P2P GO/GC in practice).

The only element with hopping value is the slot-boundary RF switch, and that
is the **H2C 0x1D firmware channel switch** already isolated in experiment 2
and delivered as `DEVOURER_FASTRETUNE_FW` (1–2 ms, arbitrary channel,
including cross-band) — with none of MCC's association, reserved-page, or
two-context constraints. Any firmware-assisted hopping work (experiments
4–5) should build on that direct switch, not on the MCC scheduler. MCC
remains what it is designed to be: concurrent-connection time-sharing for
STA+GO / STA+AP, off the FHSS path.
